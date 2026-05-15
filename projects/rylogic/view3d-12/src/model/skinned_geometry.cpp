//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#include "pr/view3d-12/model/skinned_geometry.h"
#include "pr/view3d-12/main/renderer.h"
#include "pr/view3d-12/model/model.h"
#include "pr/view3d-12/model/pose.h"
#include "pr/view3d-12/model/skin.h"
#include "pr/view3d-12/resource/resource_store.h"
#include "pr/view3d-12/shaders/shader.h"

namespace pr::rdr12
{
	using namespace ::pr::compute;

	// Hash a model/pose cache key.
	size_t SkinnedGeometryCache::KeyHash::operator () (Key const& key) const
	{
		auto hash = std::hash<Model const*>{}(key.m_model);
		hash = hash * 16777619u ^ std::hash<Pose const*>{}(key.m_pose);
		return hash;
	}

	// Create the shared skinned-geometry compute cache.
	SkinnedGeometryCache::SkinnedGeometryCache(Renderer& rdr)
		: m_rdr(rdr)
		, m_mutex()
		, m_model_deleted(rdr.store().ModelDeleted += std::bind(&SkinnedGeometryCache::OnModelDeleted, this, _1, _2))
		, m_cache()
		, m_signature()
		, m_pso()
	{
		auto device = rdr.d3d();

		m_signature = RootSig(ERootSigFlags::ComputeOnly)
			.U32<CBufSkinning>(hlsl::ECBufReg::b0)
			.SRV(hlsl::ESRVReg::t0)
			.SRV(hlsl::ESRVReg::t1)
			.SRV(hlsl::ESRVReg::t2)
			.UAV(hlsl::EUAVReg::u0)
			.Create(device, "SkinnedGeometrySig");

		m_pso = ComputePSO(m_signature.get(), shader_code::skinning_cs)
			.Create(device, "SkinnedGeometryPSO");
	}

	// Destroy the shared skinned-geometry cache.
	SkinnedGeometryCache::~SkinnedGeometryCache()
	{
		for (auto& [key, entry] : m_cache)
			ReleaseEntry(entry);

		m_cache.clear();
	}

	// Return the skinned vertex buffer resource and view for 'model' at 'pose'.
	SkinnedGeometryCache::VBuffer SkinnedGeometryCache::VBuf(GfxCmdList& cmd_list, GpuUploadBuffer& upload_buffer, Model& model, PosePtr const& pose)
	{
		// Not a skinned model, use the regular vertex buffer.
		if (!model.m_skin || pose == nullptr)
		{
			return VBuffer{
				.m_resource = model.m_vb.get(),
				.m_view = &model.m_vb_view,
				.m_pose_revision = 0,
			};
		}

		if (model.m_vstride.size() != sizeof(Vert))
			throw std::runtime_error("Skinned geometry compute path requires the standard View3D vertex layout");

		std::lock_guard lock(m_mutex);

		// Update the pose transforms
		pose->Update(cmd_list, upload_buffer);

		// Get/Create a cache entry for this model/pose pair.
		auto key = Key{ &model, pose.get() };
		auto [iter, added] = m_cache.try_emplace(key);
		auto& entry = iter->second;

		// If the entry is new, or the vertex count has changed (e.g. because the model's geometry was updated), create a new entry.
		if (added || entry.m_vcount != model.m_vcount)
		{
			// If the entry is stale, release its resources before creating a new entry.
			if (!added)
				ReleaseEntry(entry);

			// Create a new entry for this model/pose pair.
			entry = CreateEntry(model, pose);
		}

		BarrierBatch barriers(cmd_list);

		// If the pose revision has changed since the last time we computed skinned vertices for this entry, we need to dispatch the compute shader again.
		if (entry.m_pose_revision != pose->Revision())
		{
			auto cb = CBufSkinning{
				.m_vertex_count = s_cast<uint32_t>(model.m_vcount),
				.m_pad = {},
				.m_model_to_object = model.m_m2root,
				.m_object_to_model = Invert(model.m_m2root),
			};

			barriers.Transition(entry.m_vb.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			barriers.Commit();

			cmd_list.SetPipelineState(m_pso.get());
			cmd_list.SetComputeRootSignature(m_signature.get());
			cmd_list.SetComputeRoot32BitConstants(ERootParam::Constants, cb);
			cmd_list.SetComputeRootShaderResourceView(ERootParam::RestVertices, model.m_vb->GetGPUVirtualAddress());
			cmd_list.SetComputeRootShaderResourceView(ERootParam::Skin, model.m_skin.m_res->GetGPUVirtualAddress());
			cmd_list.SetComputeRootShaderResourceView(ERootParam::Pose, pose->m_res->GetGPUVirtualAddress());
			cmd_list.SetComputeRootUnorderedAccessView(ERootParam::OutputVertices, entry.m_vb->GetGPUVirtualAddress());
			cmd_list.Dispatch(DispatchCount(s_cast<int>(model.m_vcount), 128), 1, 1);

			barriers.UAV(entry.m_vb.get());
			barriers.Transition(entry.m_vb.get(), D3D12_RESOURCE_STATE_VERTEX_AND_CONSTANT_BUFFER | D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
			barriers.Commit();

			entry.m_pose_revision = pose->Revision();
		}
		else
		{
			barriers.Transition(entry.m_vb.get(), D3D12_RESOURCE_STATE_VERTEX_AND_CONSTANT_BUFFER | D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
			barriers.Commit();
		}

		return VBuffer{
			.m_resource = entry.m_vb.get(),
			.m_view = &entry.m_vb_view,
			.m_pose_revision = entry.m_pose_revision,
		};
	}

	// Return the effective vertex buffer for raster, ray-cast, and later ray-tracing consumers.
	D3D12_VERTEX_BUFFER_VIEW const& SkinnedGeometryCache::VBufView(GfxCmdList& cmd_list, GpuUploadBuffer& upload_buffer, Model& model, PosePtr const& pose)
	{
		return *VBuf(cmd_list, upload_buffer, model, pose).m_view;
	}

	// Remove cached geometry for 'model'.
	void SkinnedGeometryCache::Invalidate(Model const& model)
	{
		std::lock_guard lock(m_mutex);
		for (auto iter = begin(m_cache); iter != end(m_cache); )
		{
			if (iter->first.m_model != &model)
			{
				++iter;
				continue;
			}

			ReleaseEntry(iter->second);
			iter = m_cache.erase(iter);
		}
	}

	// Create the compute output buffer for a skinned model/pose pair.
	SkinnedGeometryCache::Entry SkinnedGeometryCache::CreateEntry(Model& model, PosePtr const& pose)
	{
		auto desc = ResDesc::VBuf<Vert>(model.m_vcount, {}).usage(EUsage::UnorderedAccess).def_state(D3D12_RESOURCE_STATE_COMMON);
		
		// VBuffer needs to be in bytes, not count of 'Vert'
		auto d3d_desc = desc;
		d3d_desc.Width *= desc.ElemStride;

		// ResDesc stores buffer width as an element count; D3D12 expects the byte count when creating buffers directly.
		D3DPtr<ID3D12Resource> vb;
		Check(m_rdr.d3d()->CreateCommittedResource(&desc.HeapProps, desc.HeapFlags, &d3d_desc, D3D12_RESOURCE_STATE_COMMON, nullptr, __uuidof(ID3D12Resource), (void**)vb.address_of()));
		DefaultResState(vb.get(), D3D12_RESOURCE_STATE_COMMON);
		DebugName(vb, "SkinnedGeometry:VB");

		// Create a vertex buffer view for binding to the pipeline.
		auto vb_view = D3D12_VERTEX_BUFFER_VIEW{
			.BufferLocation = vb->GetGPUVirtualAddress(),
			.SizeInBytes = s_cast<UINT>(model.m_vcount * sizeof(Vert)),
			.StrideInBytes = sizeof(Vert),
		};

		// Return the cache entry
		return Entry{
			.m_vb = std::move(vb),
			.m_vb_view = vb_view,
			.m_pose = pose,
			.m_pose_revision = ~0ULL,
			.m_vcount = model.m_vcount,
		};
	}

	// Defer-release resources owned by a cache entry.
	void SkinnedGeometryCache::ReleaseEntry(Entry& entry)
	{
		m_rdr.DeferRelease(entry.m_vb);
		entry.m_vb = nullptr;
		entry.m_pose = nullptr;
		entry.m_pose_revision = ~0ULL;
		entry.m_vcount = 0;
	}

	// Remove entries that refer to a model that is being destroyed.
	void SkinnedGeometryCache::OnModelDeleted(Model& model, EmptyArgs const&)
	{
		Invalidate(model);
	}
}
