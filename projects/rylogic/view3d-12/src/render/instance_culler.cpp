//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2026
//*********************************************
#include "view3d-12/src/render/instance_culler.h"
#include "pr/view3d-12/main/renderer.h"
#include "pr/view3d-12/instance/instance.h"
#include "pr/view3d-12/model/nugget.h"
#include "pr/view3d-12/shaders/shader.h"

namespace pr::rdr12
{
	using namespace ::pr::compute;

	namespace
	{
		// Constants of one cull. Mirrors 'InstanceCullConstants' in 'instance_cull.hlsl'.
		struct InstanceCullConstants
		{
			v4 planes[4];
			uint32_t arg_words;
			uint32_t capacity;
			float radius_scale;
			uint32_t pad;
		};
		static_assert(sizeof(InstanceCullConstants) == 80);

		// Root parameters of the cull kernels. See the bindings in 'instance_cull.hlsl'.
		enum class ECullParam
		{
			Constants,
			Source,
			Bounds,
			View,
		};

		// Create a default-heap buffer of 'size' bytes for the kernels to write. It starts, and returns to, the common state.
		D3DPtr<ID3D12Resource> CreateViewBuffer(ID3D12Device4* device, int64_t size)
		{
			// Describe a plain buffer. Buffers must use the row-major layout.
			auto heap = D3D12_HEAP_PROPERTIES{ .Type = D3D12_HEAP_TYPE_DEFAULT };
			auto desc = D3D12_RESOURCE_DESC{
				.Dimension = D3D12_RESOURCE_DIMENSION_BUFFER,
				.Alignment = 0,
				.Width = s_cast<uint64_t>(size),
				.Height = 1,
				.DepthOrArraySize = 1,
				.MipLevels = 1,
				.Format = DXGI_FORMAT_UNKNOWN,
				.SampleDesc = { .Count = 1, .Quality = 0 },
				.Layout = D3D12_TEXTURE_LAYOUT_ROW_MAJOR,
				.Flags = D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS,
			};

			// Create the resource and register its default state for command list state tracking.
			D3DPtr<ID3D12Resource> res;
			Check(device->CreateCommittedResource(&heap, D3D12_HEAP_FLAG_NONE, &desc, D3D12_RESOURCE_STATE_COMMON, nullptr, __uuidof(ID3D12Resource), (void**)res.address_of()));
			DefaultResState(res.get(), D3D12_RESOURCE_STATE_COMMON);
			DebugName(res, "InstanceCullView");
			return res;
		}
	}

	// Combine the key's pointers and view index.
	size_t InstanceCuller::KeyHash::operator()(Key const& key) const
	{
		// Mix each part into the running hash
		auto h = std::hash<void const*>{}(std::get<0>(key));
		h ^= std::hash<void const*>{}(std::get<1>(key)) + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2);
		h ^= std::hash<int>{}(std::get<2>(key)) + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2);
		return h;
	}

	// Create the cull pipeline.
	InstanceCuller::InstanceCuller(Renderer& rdr)
		: m_rdr(rdr)
		, m_signature()
		, m_pso_reset()
		, m_pso_cull()
		, m_buffers()
		, m_frame()
	{
		// The kernels share one root signature. See 'instance_cull.hlsl' for the bindings.
		auto device = rdr.d3d();
		m_signature = RootSig(ERootSigFlags::ComputeOnly)
			.CBuf(hlsl::ECBufReg::b0)
			.SRV(hlsl::ESRVReg::t0)
			.SRV(hlsl::ESRVReg::t1)
			.UAV(hlsl::EUAVReg::u0)
			.Create(device, "InstanceCullSig");

		m_pso_reset = ComputePSO(m_signature.get(), shader_code::instance_reset_cs).Create(device, "InstanceResetPSO");
		m_pso_cull = ComputePSO(m_signature.get(), shader_code::instance_cull_cs).Create(device, "InstanceCullPSO");
	}

	// Release the pipeline and view buffers after the GPU has finished with them.
	InstanceCuller::~InstanceCuller()
	{
		// Queued frames may still read the view buffers
		for (auto& [key, buf] : m_buffers)
			m_rdr.DeferRelease(buf.m_res);

		m_rdr.DeferRelease(m_pso_cull);
		m_rdr.DeferRelease(m_pso_reset);
		m_rdr.DeferRelease(m_signature);
	}

	// Start a frame.
	void InstanceCuller::BeginFrame()
	{
		// Release the buffers of elements that the previous frame did not draw, so removed objects do not keep their buffers.
		std::erase_if(m_buffers, [&](auto const& entry)
		{
			// Keep buffers used by the frame that just ended
			if (entry.second.m_frame == m_frame)
				return false;

			m_rdr.DeferRelease(entry.second.m_res);
			return true;
		});
		++m_frame;
	}

	// Cull the instances of 'nugget' of 'inst' for one view.
	void InstanceCuller::Cull(GfxCmdList& cmd_list, GpuUploadBuffer& upload, BaseInstance const& inst, Nugget const& nugget, int view, m4x4 const& w2s)
	{
		// A drawlist can hold the same element more than once, for example the two faces of a two-sided alpha nugget. One cull serves them all.
		auto const& args = nugget.m_draw_args;
		assert(args.m_buffer != nullptr && args.m_bounds != nullptr && args.m_capacity >= 1 && args.m_capacity <= NuggetDrawArgs::MaxCapacity);
		auto& buf = m_buffers[Key{ &inst, &nugget, view }];
		if (buf.m_res != nullptr && buf.m_frame == m_frame)
			return;

		// Grow the view buffer to hold the arguments and a full list. Earlier frames may still read the old buffer, so defer its release.
		buf.m_frame = m_frame;
		auto size = int64_t(ListOffset) + int64_t(args.m_capacity) * int64_t(sizeof(uint32_t));
		if (buf.m_size < size)
		{
			m_rdr.DeferRelease(buf.m_res);
			buf.m_res = CreateViewBuffer(m_rdr.d3d(), size);
			buf.m_size = size;
		}

		// The world-space side planes of the frustum, from the rows of the world to clip transform. Inside means 'dot(n, p) + d >= 0'.
		// Each plane is normalised in world space, so its signed distance is in world units.
		v4 planes[4];
		{
			auto row = [&](int i)
			{
				return v4{ w2s.x[i], w2s.y[i], w2s.z[i], w2s.w[i] };
			};
			auto r0 = row(0), r1 = row(1), r3 = row(3);
			planes[0] = r3 + r0;
			planes[1] = r3 - r0;
			planes[2] = r3 + r1;
			planes[3] = r3 - r1;
			for (auto& p : planes)
				p /= Length(p.xyz);
		}

		// Express the planes in object space, so the GPU tests the bounds without transforming them. Because 'dot(P, o2w * x) == dot(transpose(o2w) * P, x)',
		// the object-space planes give the same world-space distances. Object-space radii become world-space radii through the largest axis scale.
		auto const& o2w = GetO2W(inst);
		auto o2w_t = Transpose(o2w);
		auto cb = InstanceCullConstants{
			.planes = {},
			.arg_words = nugget.m_irange.empty() ? 4U : 5U,
			.capacity = s_cast<uint32_t>(args.m_capacity),
			.radius_scale = std::max({ Length(o2w.x.xyz), Length(o2w.y.xyz), Length(o2w.z.xyz) }),
			.pad = 0,
		};
		for (int i = 0; i != 4; ++i)
			cb.planes[i] = o2w_t * planes[i];

		// Bind the owner's arguments and bounds, which the owner keeps readable, and the view buffer.
		auto cb_addr = upload.Add(cb, D3D12_CONSTANT_BUFFER_DATA_PLACEMENT_ALIGNMENT, false);
		BarrierBatch barriers(cmd_list);
		barriers.Transition(buf.m_res.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		barriers.Commit();

		cmd_list.SetComputeRootSignature(m_signature.get());
		cmd_list.SetComputeRootConstantBufferView(ECullParam::Constants, cb_addr);
		cmd_list.SetComputeRootShaderResourceView(ECullParam::Source, GpuAddress(args.m_buffer, args.m_offset));
		cmd_list.SetComputeRootShaderResourceView(ECullParam::Bounds, GpuAddress(args.m_bounds));
		cmd_list.SetComputeRootUnorderedAccessView(ECullParam::View, GpuAddress(buf.m_res));

		// Reset the arguments, then append the visible instances. The count is only known on the GPU, so dispatch for the full capacity.
		cmd_list.SetPipelineState(m_pso_reset.get());
		cmd_list.Dispatch(1, 1, 1);
		barriers.UAV(buf.m_res.get());
		barriers.Commit();

		cmd_list.SetPipelineState(m_pso_cull.get());
		cmd_list.Dispatch(DispatchCount(args.m_capacity, 64), 1, 1);

		// Leave the view buffer readable as draw arguments and by the vertex shader.
		barriers.Transition(buf.m_res.get(), DrawState);
		barriers.Commit();
	}

	// The view buffer that this frame's 'Cull' wrote for (inst, nugget, view).
	ID3D12Resource* InstanceCuller::Find(BaseInstance const& inst, Nugget const& nugget, int view)
	{
		// Only buffers written this frame hold valid lists
		auto iter = m_buffers.find(Key{ &inst, &nugget, view });
		return iter != m_buffers.end() && iter->second.m_frame == m_frame ? iter->second.m_res.get() : nullptr;
	}
}
