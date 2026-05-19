//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#include "pr/view3d-12/model/skinned_geometry.h"
#include "pr/view3d-12/main/renderer.h"
#include "pr/view3d-12/model/model.h"
#include "pr/view3d-12/model/pose.h"
#include "pr/view3d-12/model/skin.h"
#include "pr/view3d-12/model/skeleton.h"
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
		, m_model_deleted(rdr.store().ModelDeleted += std::bind(&SkinnedGeometryCache::OnModelDeleted, this, _1, _2))
		, m_cache()
		, m_signature()
		, m_pso()
		, m_mutex()
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

		// Sanity checks
		{
			assert(model.m_vstride.size() == sizeof(Vert) && "Skinned geometry compute path requires the standard View3D vertex layout");
			assert(model.m_skin.m_min_skin_index >= -1 && "Skinned geometry skin index is below the dead-vertex sentinel");
			assert(model.m_skin.m_max_skin_index < model.m_skin.m_influence_count && "Skinned geometry skin index exceeds the skin influence buffer");
			assert(model.m_skin.m_skel_id == pose->m_skeleton->Id() && "Skinned geometry pose skeleton mismatch");
			assert(model.m_skin.m_max_bone_index < pose->BoneCount() && "Skinned geometry pose bone index mismatch");
		}

		std::lock_guard lock(m_mutex);

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
			InitialiseEntry(entry, model, pose);
		}

		// Update the pose transforms. This also updates cached skinned bboxes via the synchronous PoseUpdated event.
		pose->Update(cmd_list, upload_buffer);

		BarrierBatch barriers(cmd_list);

		// If the pose revision has changed since the last time we computed skinned vertices for this entry, we need to dispatch the compute shader again.
		if (entry.m_pose_revision != pose->Revision())
		{
			auto cb = CBufSkinning{
				.m_vertex_count = s_cast<uint32_t>(model.m_vcount),
				.m_skin_count = s_cast<uint32_t>(model.m_skin.m_influence_count),
				.m_pad = {},
				.m_model_to_object = model.m_m2root,
				.m_object_to_model = Invert(model.m_m2root),
			};

			// The source buffers are normally used outside compute, but this pass reads them as SRVs.
			barriers.Transition(entry.m_vb.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			barriers.Transition(model.m_vb.get(), D3D12_RESOURCE_STATE_VERTEX_AND_CONSTANT_BUFFER | D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
			barriers.Transition(model.m_skin.m_res.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
			barriers.Transition(pose->m_res.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
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

	// Return the latest cached current-pose bounding box for 'model' at 'pose'.
	std::optional<BBox> SkinnedGeometryCache::SkinnedModelBBox(Model const& model, Pose const& pose) const
	{
		std::lock_guard lock(m_mutex);

		auto iter = m_cache.find(Key{ &model, &pose });
		return iter != end(m_cache)
			? iter->second.m_bbox
			: std::nullopt;
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

	// Initialise the compute output buffer and bbox subscription for a skinned model/pose pair.
	void SkinnedGeometryCache::InitialiseEntry(Entry& entry, Model& model, PosePtr const& pose)
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

		entry.m_vb = std::move(vb);
		entry.m_vb_view = vb_view;
		entry.m_pose = pose;
		entry.m_pose_revision = ~0ULL;
		entry.m_bbox.reset();
		entry.m_bbox_revision = ~0ULL;
		entry.m_vcount = model.m_vcount;

		auto* entry_ptr = &entry;
		auto* model_ptr = &model;
		entry.m_pose_updated = AutoSub(pose->PoseUpdated += [this, entry_ptr, model_ptr](Pose&, PoseUpdatedArgs const& args)
		{
			UpdateBBox(*entry_ptr, *model_ptr, args);
		});

		vector<m4x4> pose_matrices(pose->BoneCount());
		pose->EvaluatePose(pose_matrices);
		UpdateBBox(entry, model, PoseUpdatedArgs{ pose_matrices, pose->Revision() });
	}

	// Update the cached current-pose bounding box for a skinned model/pose pair.
	void SkinnedGeometryCache::UpdateBBox(Entry& entry, Model const& model, PoseUpdatedArgs const& args)
	{
		auto const& bone_indices = model.m_skin.m_bound_bone_indices;
		auto const& bone_spheres = model.m_skin.m_bound_spheres;
		assert(bone_indices.size() == bone_spheres.size() && "Skinned bbox bone index/sphere arrays should be parallel");

		auto bbox = BBox::Reset();
		for (int i = 0, iend = isize(bone_spheres); i != iend; ++i)
		{
			auto bone_index = bone_indices[i];
			if (bone_index < 0 || bone_index >= isize(args.m_pose))
			{
				assert(false && "Skinned bbox bone index exceeds the pose buffer");
				continue;
			}

			auto const& pose = args.m_pose[bone_index];
			auto sphere = bone_spheres[i];
			auto centre = sphere.w1();
			centre = pose * centre;

			auto scale_x = Length(pose.x.xyz);
			auto scale_y = Length(pose.y.xyz);
			auto scale_z = Length(pose.z.xyz);
			auto radius = sphere.w * std::max(scale_x, std::max(scale_y, scale_z)) * 1.05f;
			auto radius3 = v4{ radius, radius, radius, 0 };

			bbox.Grow(centre + radius3);
			bbox.Grow(centre - radius3);
		}

		entry.m_bbox = bbox.valid()
			? std::optional<BBox>{ bbox }
			: std::nullopt;
		entry.m_bbox_revision = args.m_revision;
	}

	// Defer-release resources owned by a cache entry.
	void SkinnedGeometryCache::ReleaseEntry(Entry& entry)
	{
		entry.m_pose_updated.reset();
		m_rdr.DeferRelease(entry.m_vb);
		entry.m_vb = nullptr;
		entry.m_pose = nullptr;
		entry.m_pose_revision = ~0ULL;
		entry.m_bbox.reset();
		entry.m_bbox_revision = ~0ULL;
		entry.m_vcount = 0;
	}

	// Remove entries that refer to a model that is being destroyed.
	void SkinnedGeometryCache::OnModelDeleted(Model& model, EmptyArgs const&)
	{
		Invalidate(model);
	}
}
