//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2026
//*********************************************
#include "pr/view3d-12/population/population.h"
#include "pr/view3d-12/main/renderer.h"
#include "pr/view3d-12/model/model.h"
#include "pr/view3d-12/model/nugget.h"
#include "pr/view3d-12/material/material.h"
#include "pr/view3d-12/material/material_pass.h"
#include "pr/view3d-12/material/components/shader_overlays.h"
#include "pr/view3d-12/shaders/shader.h"

namespace pr::rdr12
{
	using namespace ::pr::compute;

	namespace
	{
		// Constants of one classification. Mirrors 'CullConstants' in 'population_cull.hlsl'.
		struct CullConstants
		{
			v4 planes[4];
			v4 eye;
			v4 range_origin;
			v4 bound;
			v4 lod_size[PopulationMaxLods / 4];
			v4 lod_fade[PopulationMaxLods / 4];
			float size_scale;
			float draw_distance;
			uint32_t lod_count;
			uint32_t arg_count;
			uint32_t list_base;
			uint32_t pad0;
			uint32_t pad1;
			uint32_t pad2;
		};
		static_assert(sizeof(CullConstants) == 208, "Must match CullConstants in population_cull.hlsl");

		// A block that passed the CPU culling test. Mirrors 'VisibleBlock' in 'population_cull.hlsl'.
		struct VisibleBlock
		{
			uint32_t first;
			uint32_t capacity;
			uint32_t block;
			uint32_t pad;
		};

		// The template of an indirect draw argument record. Mirrors 'ArgTemplate' in 'population_cull.hlsl'.
		struct ArgTemplate
		{
			uint32_t lod;
			uint32_t count;
			uint32_t start;
			uint32_t pad;
		};

		// Root parameters of the classification kernels.
		enum class ECullParam
		{
			Constants,
			Instances,
			Blocks,
			LiveCounts,
			Args,
			View,
		};

		// The state that shaders read population buffers in.
		constexpr auto ShaderReadState = D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE | D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE;

		// Create a default-heap buffer of 'size' bytes. Buffers start, and return to, the common state.
		D3DPtr<ID3D12Resource> CreateBuffer(ID3D12Device4* device, int64_t size, D3D12_RESOURCE_FLAGS flags, char const* name)
		{
			// Describe a plain buffer. Buffers must use the row-major layout.
			auto heap = D3D12_HEAP_PROPERTIES{ .Type = D3D12_HEAP_TYPE_DEFAULT };
			auto desc = D3D12_RESOURCE_DESC{
				.Dimension = D3D12_RESOURCE_DIMENSION_BUFFER,
				.Alignment = 0,
				.Width = s_cast<uint64_t>(PadTo<int64_t>(std::max<int64_t>(size, 4), 4)),
				.Height = 1,
				.DepthOrArraySize = 1,
				.MipLevels = 1,
				.Format = DXGI_FORMAT_UNKNOWN,
				.SampleDesc = { .Count = 1, .Quality = 0 },
				.Layout = D3D12_TEXTURE_LAYOUT_ROW_MAJOR,
				.Flags = flags,
			};

			// Create the resource and register its default state for command list state tracking.
			D3DPtr<ID3D12Resource> res;
			Check(device->CreateCommittedResource(&heap, D3D12_HEAP_FLAG_NONE, &desc, D3D12_RESOURCE_STATE_COMMON, nullptr, __uuidof(ID3D12Resource), (void**)res.address_of()));
			DefaultResState(res.get(), D3D12_RESOURCE_STATE_COMMON);
			DebugName(res, name);
			return res;
		}

		// Create a command signature for a single draw argument with a 'ArgStride' byte stride.
		D3DPtr<ID3D12CommandSignature> CreateDrawSignature(ID3D12Device4* device, D3D12_INDIRECT_ARGUMENT_TYPE type, char const* name)
		{
			// Population draws change no root arguments, so the signature has no root signature.
			auto arg = D3D12_INDIRECT_ARGUMENT_DESC{ .Type = type };
			auto desc = D3D12_COMMAND_SIGNATURE_DESC{
				.ByteStride = PopulationCuller::ArgStride,
				.NumArgumentDescs = 1,
				.pArgumentDescs = &arg,
				.NodeMask = 0,
			};
			D3DPtr<ID3D12CommandSignature> sig;
			Check(device->CreateCommandSignature(&desc, nullptr, __uuidof(ID3D12CommandSignature), (void**)sig.address_of()));
			DebugName(sig, name);
			return sig;
		}

		// True if a world-space box is on the inside of 'plane'. The test is conservative: a box crossing the plane counts as inside.
		bool BoxInside(v4 const& plane, BBox const& bbox)
		{
			// Compare the box's projected half-size along the plane normal with the signed distance of its centre.
			auto n = plane.w0();
			return Dot(n, bbox.Centre().w0()) + plane.w + Dot(Abs(n), bbox.Radius().w0()) >= 0;
		}

		// Reject materials that population draws cannot honour. Population draws are opaque, so blended transparency is not supported,
		// and vertices are placed by the stock population vertex shaders, so forward and shadow vertex shader overlays are not supported. Alpha testing is supported.
		void ValidateMaterials(Population::Proxy const& proxy, Model const& model)
		{
			// Check every nugget, including dependents.
			for (auto& nugget : Enumerate(model.m_nuggets))
			{
				for (auto& nug : nugget.Dependents())
				{
					// Blending only matters for nuggets that have a forward pass
					auto const& material = nug.mat();
					auto const* pass = material.Pass(ERenderStep::RenderForward);
					if (pass != nullptr && pass->RequiresAlpha(proxy.m_base, material, nug))
						throw std::invalid_argument(std::format("Population model '{}' has a material that requires alpha blending", model.m_name));

					// Vertex shader overlays would replace the population vertex shaders of the forward and shadow passes
					if (auto const* overlays = material.Component<materials::ShaderOverlays>())
					{
						for (auto const& entry : overlays->m_overlays)
						{
							if ((entry.m_rdr_step == ERenderStep::RenderForward || entry.m_rdr_step == ERenderStep::ShadowMap) && entry.m_overlay->m_code.VS)
								throw std::invalid_argument(std::format("Population model '{}' has a material with a forward or shadow vertex shader overlay", model.m_name));
						}
					}
				}
			}
		}
	}

	// Validate 'lods' and create a prototype.
	PopulationPrototype::PopulationPrototype(std::span<PopulationLod const> lods, float draw_distance)
		: RefCounted<PopulationPrototype>()
		, m_lods(lods.begin(), lods.end())
		, m_draw_distance(draw_distance)
		, m_bound()
	{
		// Check the LOD count and draw distance.
		if (lods.empty() || isize(lods) > PopulationMaxLods)
			throw std::invalid_argument(std::format("A population prototype needs 1 to {} LODs", PopulationMaxLods));
		if (!(draw_distance >= 0) || !std::isfinite(draw_distance))
			throw std::invalid_argument("Population draw distance must be finite and non-negative");

		// Check each LOD, and that thresholds decrease without overlapping transition bands.
		for (int i = 0, iend = isize(lods); i != iend; ++i)
		{
			auto const& lod = lods[i];
			if (lod.m_model == nullptr)
				throw std::invalid_argument(std::format("Population LOD {} has no model", i));
			if (!(lod.m_min_size >= 0) || !std::isfinite(lod.m_min_size) || !(lod.m_transition >= 0) || !std::isfinite(lod.m_transition))
				throw std::invalid_argument(std::format("Population LOD {} has an invalid size threshold or transition", i));
			if (i != 0 && !(lod.m_min_size < lods[i - 1].m_min_size))
				throw std::invalid_argument(std::format("Population LOD {} threshold must be less than LOD {}'s", i, i - 1));
			if (i != 0 && lod.m_min_size * (1 + lod.m_transition) > lods[i - 1].m_min_size)
				throw std::invalid_argument(std::format("Population LOD {} transition band overlaps LOD {}'s threshold", i, i - 1));
		}

		// Bound all LODs with one sphere around the union of their boxes.
		auto bbox = BBox::Reset();
		for (auto const& lod : lods)
			Grow(bbox, lod.m_model->m_bbox);

		if (!bbox.valid())
			throw std::invalid_argument("Population LOD models have no valid bounds");

		m_bound = bbox.Centre().w0() + v4{ 0, 0, 0, Length(bbox.Radius().xyz) };
	}

	// Ref-counting clean up function
	void PopulationPrototype::RefCountZero(RefCounted<PopulationPrototype>* doomed)
	{
		delete static_cast<PopulationPrototype*>(doomed);
	}

	// Create a population with fixed capacities.
	Population::Population(Renderer& rdr, PopulationPrototypePtr prototype, int instance_capacity, int block_capacity, int payload_size)
		: RefCounted<Population>()
		, m_rdr(&rdr)
		, m_prototype(prototype)
		, m_instance_capacity(instance_capacity)
		, m_payload_size(payload_size)
		, m_instances()
		, m_live_counts()
		, m_payload()
		, m_blocks(s_cast<size_t>(std::max(block_capacity, 0)))
		, m_proxies(prototype != nullptr ? prototype->m_lods.size() : 0)
		, m_views()
		, m_pending()
		, m_revision()
		, m_mutex()
	{
		// Check the description.
		if (prototype == nullptr)
			throw std::invalid_argument("A population requires a prototype");
		if (instance_capacity <= 0 || block_capacity <= 0 || payload_size < 0)
			throw std::invalid_argument("Population capacities must be positive, and the payload size non-negative");

		// Create the GPU buffers up front, so capacity failures happen here rather than mid-frame.
		auto device = rdr.d3d();
		m_instances = CreateBuffer(device, int64_t(instance_capacity) * sizeof(PopulationInstance), D3D12_RESOURCE_FLAG_NONE, "Population:Instances");
		m_live_counts = CreateBuffer(device, int64_t(block_capacity) * sizeof(uint32_t), D3D12_RESOURCE_FLAG_NONE, "Population:LiveCounts");
		if (payload_size != 0)
			m_payload = CreateBuffer(device, payload_size, D3D12_RESOURCE_FLAG_NONE, "Population:Payload");

		// Each LOD draws through a proxy at the origin. Population vertices are placed by the instance records, not by the proxy transform.
		for (int i = 0, iend = isize(m_proxies); i != iend; ++i)
		{
			m_proxies[i].m_model = prototype->m_lods[i].m_model;
			m_proxies[i].m_i2w = m4x4::Identity();
			ValidateMaterials(m_proxies[i], *m_proxies[i].m_model.get());
		}

		// Blocks have no live instances until they are defined.
		auto zeros = std::vector<uint8_t>(s_cast<size_t>(block_capacity) * sizeof(uint32_t));
		Enqueue(m_live_counts.get(), 0, std::as_bytes(std::span{ zeros }));
	}

	// Release the GPU buffers once the GPU has finished with them.
	Population::~Population()
	{
		m_rdr->DeferRelease(m_instances);
		m_rdr->DeferRelease(m_live_counts);
		if (m_payload != nullptr)
			m_rdr->DeferRelease(m_payload);

		for (auto& [key, view] : m_views)
			m_rdr->DeferRelease(view.m_res);
	}

	// The number of block slots.
	int Population::BlockCapacity() const
	{
		return isize(m_blocks);
	}

	// Define block 'index'.
	void Population::SetBlock(int index, BBox const& bbox, int first, int capacity, int live_count)
	{
		// Validate the block's slot range.
		if (index < 0 || index >= BlockCapacity())
			throw std::out_of_range(std::format("Population block index {} is out of range [0,{})", index, BlockCapacity()));
		if (first < 0 || capacity <= 0 || capacity > PopulationCuller::MaxBlockCapacity || int64_t(first) + capacity > m_instance_capacity)
			throw std::out_of_range(std::format("Population block slots [{},{}) are outside the instance capacity {}", first, int64_t(first) + capacity, m_instance_capacity));
		if (live_count < 0 || live_count > capacity)
			throw std::out_of_range(std::format("Population block live count {} is outside [0,{}]", live_count, capacity));
		if (!bbox.valid())
			throw std::invalid_argument("Population block bounds are invalid");

		// Record the block and upload its live count.
		std::lock_guard lock(m_mutex);
		m_blocks[index] = PopulationBlock{ .m_bbox = bbox, .m_first = first, .m_capacity = capacity, .m_active = true };
		auto count = s_cast<uint32_t>(live_count);
		Enqueue(m_live_counts.get(), int64_t(index) * sizeof(uint32_t), byte_span(std::span<uint32_t const>{ &count, 1 }));
		++m_revision;
	}

	// Stop drawing block 'index'.
	void Population::RemoveBlock(int index)
	{
		// Inactive blocks are skipped by the CPU block test, so their GPU live count does not matter.
		if (index < 0 || index >= BlockCapacity())
			throw std::out_of_range(std::format("Population block index {} is out of range [0,{})", index, BlockCapacity()));

		std::lock_guard lock(m_mutex);
		m_blocks[index].m_active = false;
		++m_revision;
	}

	// Call 'fn' with the bounds of each active block, and return the content revision read under the same lock.
	uint64_t Population::ActiveBlocks(std::function<void(BBox const&)> fn)
	{
		// The lock keeps the blocks and the revision consistent with each other.
		std::lock_guard lock(m_mutex);
		for (auto const& blk : m_blocks)
		{
			if (blk.m_active)
				fn(blk.m_bbox);
		}
		return m_revision;
	}

	// Copy 'instances' into slots [first, first + instances.size()).
	void Population::UpdateInstances(int first, std::span<PopulationInstance const> instances)
	{
		// Validate the slot range.
		if (first < 0 || int64_t(first) + isize(instances) > m_instance_capacity)
			throw std::out_of_range(std::format("Population instance slots [{},{}) are outside the capacity {}", first, int64_t(first) + isize(instances), m_instance_capacity));
		if (instances.empty())
			return;

		std::lock_guard lock(m_mutex);
		Enqueue(m_instances.get(), int64_t(first) * sizeof(PopulationInstance), byte_span(instances));
		++m_revision;
	}

	// Copy 'data' into the payload buffer at byte offset 'ofs'.
	void Population::UpdatePayload(int64_t ofs, std::span<uint8_t const> data)
	{
		// Validate the byte range.
		if (ofs < 0 || ofs + isize(data) > m_payload_size)
			throw std::out_of_range(std::format("Population payload range [{},{}) is outside the payload size {}", ofs, ofs + isize(data), m_payload_size));
		if (data.empty())
			return;

		std::lock_guard lock(m_mutex);
		Enqueue(m_payload.get(), ofs, std::as_bytes(data));
		++m_revision;
	}

	// Record the pending uploads into 'cmd_list'.
	void Population::Flush(GfxCmdList& cmd_list, GpuUploadBuffer& upload)
	{
		// Take the pending uploads, so callers can queue more while this frame records.
		std::vector<PendingUpload> pending;
		{
			std::lock_guard lock(m_mutex);
			pending.swap(m_pending);
		}
		if (pending.empty())
			return;

		// Move every destination to the copy state in one batch.
		BarrierBatch barriers(cmd_list);
		for (auto const& p : pending)
			barriers.Transition(p.m_dst, D3D12_RESOURCE_STATE_COPY_DEST);

		barriers.Commit();

		// Copy each range in submission order, so later updates of the same bytes win.
		for (auto const& p : pending)
		{
			auto alloc = upload.Alloc(isize(p.m_data), 16);
			memcpy(alloc.m_mem + alloc.m_ofs, p.m_data.data(), p.m_data.size());
			cmd_list.CopyBufferRegion(p.m_dst, s_cast<uint64_t>(p.m_ofs), alloc);
		}

		// Leave the buffers readable by the culling and drawing shaders.
		for (auto const& p : pending)
			barriers.Transition(p.m_dst, ShaderReadState);

		barriers.Commit();
	}

	// Return the classification scratch buffer for 'view_key'.
	ID3D12Resource* Population::ViewBuffer(uint64_t view_key, int64_t size)
	{
		// Grow the view's buffer when it is too small. The old buffer may still be in use by frames in flight.
		std::lock_guard lock(m_mutex);
		auto& view = m_views[view_key];
		if (view.m_res == nullptr || view.m_size < size)
		{
			if (view.m_res != nullptr)
				m_rdr->DeferRelease(view.m_res);

			view.m_res = CreateBuffer(m_rdr->d3d(), size, D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS, "Population:View");
			view.m_size = size;
		}
		return view.m_res.get();
	}

	// Ref-counting clean up function
	void Population::RefCountZero(RefCounted<Population>* doomed)
	{
		delete static_cast<Population*>(doomed);
	}

	// Queue a copy of 'data' to 'dst' at byte offset 'ofs'. Callers hold 'm_mutex' or have exclusive access.
	void Population::Enqueue(ID3D12Resource* dst, int64_t ofs, std::span<std::byte const> data)
	{
		m_pending.push_back(PendingUpload{
			.m_dst = dst,
			.m_ofs = ofs,
			.m_data = std::vector<std::byte>(data.begin(), data.end()),
		});
	}

	// Create the classification pipeline.
	PopulationCuller::PopulationCuller(Renderer& rdr)
		: m_signature()
		, m_pso_clear()
		, m_pso_count()
		, m_pso_prefix()
		, m_pso_scatter()
		, m_draw_indexed()
		, m_draw()
	{
		auto device = rdr.d3d();

		// The kernels share one root signature. See 'population_cull.hlsl' for the bindings.
		m_signature = RootSig(ERootSigFlags::ComputeOnly)
			.CBuf(hlsl::ECBufReg::b0)
			.SRV(hlsl::ESRVReg::t0)
			.SRV(hlsl::ESRVReg::t1)
			.SRV(hlsl::ESRVReg::t2)
			.SRV(hlsl::ESRVReg::t3)
			.UAV(hlsl::EUAVReg::u0)
			.Create(device, "PopulationCullSig");

		m_pso_clear = ComputePSO(m_signature.get(), shader_code::population_clear_cs).Create(device, "PopulationClearPSO");
		m_pso_count = ComputePSO(m_signature.get(), shader_code::population_count_cs).Create(device, "PopulationCountPSO");
		m_pso_prefix = ComputePSO(m_signature.get(), shader_code::population_prefix_cs).Create(device, "PopulationPrefixPSO");
		m_pso_scatter = ComputePSO(m_signature.get(), shader_code::population_scatter_cs).Create(device, "PopulationScatterPSO");

		// Indirect draws read one argument record per call.
		m_draw_indexed = CreateDrawSignature(device, D3D12_INDIRECT_ARGUMENT_TYPE_DRAW_INDEXED, "PopulationDrawIndexedSig");
		m_draw = CreateDrawSignature(device, D3D12_INDIRECT_ARGUMENT_TYPE_DRAW, "PopulationDrawSig");
	}

	// Classify 'pop' for 'view'.
	PopulationViewLists PopulationCuller::Cull(GfxCmdList& cmd_list, GpuUploadBuffer& upload, Population& pop, PopulationViewDesc const& view, ERenderStep step)
	{
		auto const& proto = *pop.m_prototype.get();
		auto lod_count = isize(proto.m_lods);

		// The world-space side planes of the frustum, from the rows of the world to clip transform. Inside means 'dot(n, p) + d >= 0'.
		v4 planes[4];
		{
			auto row = [&](int i)
			{
				return v4{ view.m_w2s.x[i], view.m_w2s.y[i], view.m_w2s.z[i], view.m_w2s.w[i] };
			};
			auto r0 = row(0), r1 = row(1), r3 = row(3);
			planes[0] = r3 + r0;
			planes[1] = r3 - r0;
			planes[2] = r3 + r1;
			planes[3] = r3 - r1;
			for (auto& p : planes)
				p /= Length(p.xyz);
		}

		// Select the blocks that can contain visible instances. The GPU tests individual instances within these.
		auto visible = std::vector<VisibleBlock>{};
		auto max_capacity = 0;
		{
			std::lock_guard lock(pop.m_mutex);
			for (int b = 0, bend = isize(pop.m_blocks); b != bend; ++b)
			{
				auto const& blk = pop.m_blocks[b];
				if (!blk.m_active)
					continue;
				if (!std::ranges::all_of(planes, [&](v4 const& p) { return BoxInside(p, blk.m_bbox); }))
					continue;
				if (proto.m_draw_distance > 0)
				{
					// Instance spheres are inside the box, so the box is beyond the draw distance only when its closest point is.
					auto closest = Max(Abs(blk.m_bbox.Centre().w0() - view.m_range_origin.w0()) - blk.m_bbox.Radius().w0(), v4::Zero());
					if (Length(closest) > proto.m_draw_distance)
						continue;
				}

				visible.push_back(VisibleBlock{ .first = s_cast<uint32_t>(blk.m_first), .capacity = s_cast<uint32_t>(blk.m_capacity), .block = s_cast<uint32_t>(b), .pad = 0 });
				max_capacity = std::max(max_capacity, blk.m_capacity);
				if (isize(visible) == MaxVisibleBlocks)
					break;
			}
		}

		// Build the draw list: an indexed and a non-indexed argument record for each drawable nugget of each LOD.
		auto lists = PopulationViewLists{ .m_population = &pop, .m_scratch = nullptr, .m_list_base = 0, .m_draws = {} };
		auto args = std::vector<ArgTemplate>{};
		for (int lod = 0; lod != lod_count; ++lod)
		{
			PopulationNuggets(*proto.m_lods[lod].m_model.get(), step, [&](Nugget const& nug)
			{
				// Records 2k and 2k+1 are the indexed and non-indexed draws of the nugget.
				auto i = isize(args);
				args.push_back(ArgTemplate{ .lod = s_cast<uint32_t>(lod), .count = s_cast<uint32_t>(nug.m_irange.size()), .start = s_cast<uint32_t>(nug.m_irange.m_beg), .pad = 0 });
				args.push_back(ArgTemplate{ .lod = s_cast<uint32_t>(lod), .count = s_cast<uint32_t>(nug.m_vrange.size()), .start = s_cast<uint32_t>(nug.m_vrange.m_beg), .pad = 0 });
				lists.m_draws.push_back(PopulationViewDraw{
					.m_proxy = &pop.m_proxies[lod],
					.m_nugget = &nug,
					.m_lod = lod,
					.m_args_indexed = PopulationHeaderSize + int64_t(i + 0) * ArgStride,
					.m_args_vertex = PopulationHeaderSize + int64_t(i + 1) * ArgStride,
				});
			});
		}

		// Size the view buffer: header, argument records, then up to two list entries per instance (a transition adds a complementary entry).
		auto list_base = PadTo<int64_t>(PopulationHeaderSize + isize(args) * ArgStride, 16);
		auto view_size = list_base + int64_t(pop.m_instance_capacity) * 2 * sizeof(PopulationEntry);
		auto scratch = pop.ViewBuffer(view.m_key, view_size);
		lists.m_scratch = scratch;
		lists.m_list_base = s_cast<uint32_t>(list_base);

		// Classification constants.
		auto cb = CullConstants{};
		for (int i = 0; i != 4; ++i)
			cb.planes[i] = planes[i];

		cb.eye = v4{ view.m_eye.x, view.m_eye.y, view.m_eye.z, view.m_orthographic ? 1.0f : 0.0f };
		cb.range_origin = view.m_range_origin.w1();
		cb.bound = proto.m_bound;
		for (int lod = 0; lod != lod_count; ++lod)
		{
			cb.lod_size[lod / 4][lod % 4] = proto.m_lods[lod].m_min_size;
			cb.lod_fade[lod / 4][lod % 4] = proto.m_lods[lod].m_transition;
		}
		cb.size_scale = view.m_size_scale;
		cb.draw_distance = proto.m_draw_distance;
		cb.lod_count = s_cast<uint32_t>(lod_count);
		cb.arg_count = s_cast<uint32_t>(args.size());
		cb.list_base = s_cast<uint32_t>(list_base);

		// Upload the per-view inputs. Root SRVs need a valid address even when a table is empty.
		auto upload_table = [&]<typename T>(std::vector<T> const& table)
		{
			auto alloc = upload.Alloc(std::max<int64_t>(isize(table), 1) * sizeof(T), 16);
			if (!table.empty())
				memcpy(alloc.m_mem + alloc.m_ofs, table.data(), table.size() * sizeof(T));

			return GpuAddress(alloc);
		};
		auto cb_addr = upload.Add(cb, D3D12_CONSTANT_BUFFER_DATA_PLACEMENT_ALIGNMENT, false);
		auto blocks_addr = upload_table(visible);
		auto args_addr = upload_table(args);

		// The kernels read the population buffers and write the view buffer. The payload is read only by the draws that follow.
		BarrierBatch barriers(cmd_list);
		barriers.Transition(pop.m_instances.get(), ShaderReadState);
		barriers.Transition(pop.m_live_counts.get(), ShaderReadState);
		if (pop.m_payload != nullptr)
			barriers.Transition(pop.m_payload.get(), ShaderReadState);

		barriers.Transition(scratch, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		barriers.Commit();

		cmd_list.SetComputeRootSignature(m_signature.get());
		cmd_list.SetComputeRootConstantBufferView(ECullParam::Constants, cb_addr);
		cmd_list.SetComputeRootShaderResourceView(ECullParam::Instances, GpuAddress(pop.m_instances));
		cmd_list.SetComputeRootShaderResourceView(ECullParam::Blocks, blocks_addr);
		cmd_list.SetComputeRootShaderResourceView(ECullParam::LiveCounts, GpuAddress(pop.m_live_counts));
		cmd_list.SetComputeRootShaderResourceView(ECullParam::Args, args_addr);
		cmd_list.SetComputeRootUnorderedAccessView(ECullParam::View, GpuAddress(scratch));

		// Zero the header, count the visible entries, lay out the lists, then write the entries. Each kernel reads the previous kernel's writes.
		auto groups_x = DispatchCount(max_capacity, 64);
		auto blocks_y = isize(visible);
		cmd_list.SetPipelineState(m_pso_clear.get());
		cmd_list.Dispatch(1, 1, 1);
		barriers.UAV(scratch);
		barriers.Commit();

		if (blocks_y != 0)
		{
			cmd_list.SetPipelineState(m_pso_count.get());
			cmd_list.Dispatch(groups_x, blocks_y, 1);
			barriers.UAV(scratch);
			barriers.Commit();
		}

		cmd_list.SetPipelineState(m_pso_prefix.get());
		cmd_list.Dispatch(1, 1, 1);
		barriers.UAV(scratch);
		barriers.Commit();

		if (blocks_y != 0)
		{
			cmd_list.SetPipelineState(m_pso_scatter.get());
			cmd_list.Dispatch(groups_x, blocks_y, 1);
			barriers.UAV(scratch);
			barriers.Commit();
		}

		// Leave the view buffer readable as draw arguments and by the vertex and pixel shaders.
		barriers.Transition(scratch, D3D12_RESOURCE_STATE_INDIRECT_ARGUMENT | ShaderReadState);
		barriers.Commit();
		return lists;
	}

	// The command signature for an indexed or non-indexed draw.
	ID3D12CommandSignature* PopulationCuller::DrawSignature(bool indexed)
	{
		return indexed ? m_draw_indexed.get() : m_draw.get();
	}

	// Return the nuggets of a LOD that draw for 'step'.
	void PopulationNuggets(Model const& model, ERenderStep step, std::function<void(Nugget const&)> fn)
	{
		// Populations draw the default variant of each nugget, in the same way as an instance without overrides.
		for (auto& nug : Enumerate(model.m_nuggets))
		{
			for (auto const& dep : nug.Dependents())
			{
				if (dep.m_variant != DefaultNugget)
					continue;
				if (AllSet(dep.m_nflags, ENuggetFlag::Hidden))
					continue;
				if (dep.mat().Pass(step) == nullptr)
					continue;

				fn(dep);
			}
		}
	}

	// Describe the view of 'cam' rendered into a viewport 'viewport_height' pixels high.
	PopulationViewDesc PopulationCameraView(uint64_t key, Camera const& cam, float viewport_height)
	{
		// The projected radius in pixels is 'radius * c2s.y.y * height / 2', divided by the distance for perspective projections.
		auto c2s = cam.CameraToScreen();
		auto c2w = cam.CameraToWorld();
		return PopulationViewDesc{
			.m_key = key,
			.m_w2s = c2s * InvertOrthonormal(c2w),
			.m_eye = c2w.pos,
			.m_range_origin = c2w.pos,
			.m_size_scale = 0.5f * viewport_height * c2s.y.y,
			.m_orthographic = cam.Orthographic(),
		};
	}

	// Describe a view given by its world to clip transform.
	PopulationViewDesc PopulationProjectionView(uint64_t key, m4x4 const& w2s, float viewport_height, v4 range_origin)
	{
		// Clip w is the fourth row. Orthographic projections have a constant clip w, so the row has no xyz part.
		auto row3 = v4{ w2s.x.w, w2s.y.w, w2s.z.w, w2s.w.w };
		auto orthographic = Length(row3.xyz) <= 1e-6f * Abs(row3.w);

		// A perspective eye maps to clip x = y = w = 0, so the inverse maps the clip direction (0,0,1,0) to a homogeneous multiple of the eye.
		// Orthographic projections have no eye position, and their projected size does not depend on distance.
		auto eye = range_origin.w1();
		if (!orthographic)
		{
			auto h = Invert(w2s) * v4{ 0, 0, 1, 0 };
			eye = h / h.w;
		}

		// Clip y is the second row. Its xyz length is the projection's y scale because the view part of 'w2s' is orthonormal.
		auto row1 = v4{ w2s.x.y, w2s.y.y, w2s.z.y, w2s.w.y };
		return PopulationViewDesc{
			.m_key = key,
			.m_w2s = w2s,
			.m_eye = eye,
			.m_range_origin = range_origin.w1(),
			.m_size_scale = 0.5f * viewport_height * Length(row1.xyz),
			.m_orthographic = orthographic,
		};
	}

	// Throw if 'prototype' is null.
	void Validate(PopulationPrototype const* prototype)
	{
		if (prototype == nullptr)
			throw std::runtime_error("Population prototype pointer is null");
	}

	// Throw if 'population' is null.
	void Validate(Population const* population)
	{
		if (population == nullptr)
			throw std::runtime_error("Population pointer is null");
	}
}
