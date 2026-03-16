//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#pragma once
#include "pr/view3d-12/forward.h"
#include "pr/view3d-12/model/skin.h"
#include "pr/view3d-12/model/pose.h"
#include "pr/view3d-12/utility/update_resource.h"

namespace pr::rdr12
{
	struct Model :RefCounted<Model>
	{
		// Notes:
		//  - Models without index buffers are not supported because they are a rare case and
		//    they would add loads of if statements. Just create a dummy index buffer, and create
		//    nuggets with a zero range for the index buffer.
		//  - 'm_m2root' records the model's position in model-space. Normally you'd want models
		//    to be created at the origin, but for multi-part models (e.g. like a robot, or a plane
		//    with moving parts) we need to record the relative positions of each part. This is also
		//    needed for skinning because the skeleton is in model space but the model may have an offset.
		//  - 'm_m2root' is model-to-root and not model-to-parent, because models do not track parenting.

		enum class EDbgFlags
		{
			None                   = 0,
			WarnedNoRenderNuggets  = 1 << 0,
			WarnedInvalidTransform = 1 << 1,
			NormalsVisible         = 1 << 2,
			_flags_enum            = 0,
		};

		Renderer*                m_rdr;       // The renderer that owns this model
		D3DPtr<ID3D12Resource>   m_vb;        // The vertex buffer
		D3DPtr<ID3D12Resource>   m_ib;        // The index buffer
		D3D12_VERTEX_BUFFER_VIEW m_vb_view;   // Buffer views for shader binding
		D3D12_INDEX_BUFFER_VIEW  m_ib_view;   // Buffer views for shader binding
		TNuggetChain             m_nuggets;   // The nuggets for this model
		int64_t                  m_vcount;    // The count of elements in the V-buffer
		int64_t                  m_icount;    // The count of elements in the I-buffer
		m4x4                     m_m2root;    // Model to root transform. Use for multi-part models, like skinned characters
		Skin                     m_skin;      // Skinning data for this model.
		BBox                     m_bbox;      // A bounding box for the model. Set by the client
		string32                 m_name;      // A human readable name for the model
		SizeAndAlign16           m_vstride;   // The size and alignment (in bytes) of a single V-element
		SizeAndAlign16           m_istride;   // The size and alignment (in bytes) of a single I-element
		mutable EDbgFlags        m_dbg_flags; // Flags used by PR_DBG_RDR to output info once only

		Model(Renderer& rdr,
			int64_t vcount,
			int64_t icount,
			SizeAndAlign16 vstride,
			SizeAndAlign16 istride,
			ID3D12Resource* vb,
			ID3D12Resource* ib,
			BBox const& bbox,
			m4x4 const& m2root,
			std::string_view name
		);
		Model(Model&&) = delete;
		Model(Model const&) = delete;
		Model& operator =(Model&&) = delete;
		Model& operator =(Model const&) = delete;
		~Model();

		// Renderer access
		Renderer const& rdr() const;
		Renderer& rdr();

		// Allow update of the vertex/index buffers
		GfxUpdateSubresourceScope UpdateVertices(GfxCmdList& cmd_list, GpuUploadBuffer& upload, Range vrange = Range::Reset());
		GfxUpdateSubresourceScope UpdateIndices(GfxCmdList& cmd_list, GpuUploadBuffer& upload, Range vrange = Range::Reset());

		// Create a nugget from a range within this model
		// Ranges are model relative, i.e. the first vert in the model is range [0,1)
		// Remember you might need to delete render nuggets first
		void CreateNugget(ResourceFactory& factory, NuggetDesc const& props);

		// Call to release the nuggets that this model has been
		// divided into. Nuggets are the contiguous sub groups
		// of the model geometry that use the same data.
		void DeleteNuggets();

		// Ref-counting clean up function
		static void RefCountZero(RefCounted<Model>* doomed);
	};
}
