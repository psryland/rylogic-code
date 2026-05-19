//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#pragma once
#include "pr/view3d-12/forward.h"
#include "pr/view3d-12/model/vertex_layout.h"

namespace pr::rdr12
{
	// Shared GPU-computed current-pose vertex buffers for skinned models.
	struct SkinnedGeometryCache
	{
	private:

		using GfxCmdList = ::pr::compute::GfxCmdList;
		using GpuUploadBuffer = ::pr::compute::GpuUploadBuffer;

		struct Key
		{
			Model const* m_model;
			Pose const* m_pose;

			friend bool operator == (Key const&, Key const&) = default;
		};
		struct KeyHash
		{
			size_t operator () (Key const& key) const;
		};
		struct Entry
		{
			D3DPtr<ID3D12Resource> m_vb;
			D3D12_VERTEX_BUFFER_VIEW m_vb_view;
			PosePtr m_pose;
			uint64_t m_pose_revision;
			int64_t m_vcount;
		};
		struct CBufSkinning
		{
			uint32_t m_vertex_count;
			uint32_t m_skin_count;
			uint32_t m_pad[2];
			m4x4 m_model_to_object;
			m4x4 m_object_to_model;
		};
		struct ERootParam
		{
			enum
			{
				Constants,
				RestVertices,
				Skin,
				Pose,
				OutputVertices,
			};
		};

		Renderer& m_rdr;
		std::mutex m_mutex;
		AutoSub m_model_deleted;
		std::unordered_map<Key, Entry, KeyHash> m_cache;
		D3DPtr<ID3D12RootSignature> m_signature;
		D3DPtr<ID3D12PipelineState> m_pso;

	public:

		struct VBuffer
		{
			ID3D12Resource* m_resource;
			D3D12_VERTEX_BUFFER_VIEW const* m_view;
			uint64_t m_pose_revision;
		};

		SkinnedGeometryCache(Renderer& rdr);
		SkinnedGeometryCache(SkinnedGeometryCache&&) = delete;
		SkinnedGeometryCache(SkinnedGeometryCache const&) = delete;
		SkinnedGeometryCache& operator = (SkinnedGeometryCache&&) = delete;
		SkinnedGeometryCache& operator = (SkinnedGeometryCache const&) = delete;
		~SkinnedGeometryCache();

		// Return the skinned vertex buffer resource and view for 'model' at 'pose'.
		VBuffer VBuf(GfxCmdList& cmd_list, GpuUploadBuffer& upload_buffer, Model& model, PosePtr const& pose);

		// Return the vertex buffer that represents 'model' at 'pose', dispatching compute work only when the cached pose is stale.
		D3D12_VERTEX_BUFFER_VIEW const& VBufView(GfxCmdList& cmd_list, GpuUploadBuffer& upload_buffer, Model& model, PosePtr const& pose);

		// Remove cached geometry for 'model' because its rest-pose geometry or skinning data has changed.
		void Invalidate(Model const& model);

	private:

		// Create the compute output buffer for a cache entry.
		Entry CreateEntry(Model& model, PosePtr const& pose);

		// Release the GPU resources owned by 'entry' after the renderer has finished using them.
		void ReleaseEntry(Entry& entry);

		// Remove cached geometry for a model deletion notification.
		void OnModelDeleted(Model& model, EmptyArgs const&);
	};
}
