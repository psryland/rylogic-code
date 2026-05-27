//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#pragma once
#include "pr/view3d-12/forward.h"

namespace pr::rdr12
{
	struct ModelDesc
	{
		using VBStreams = std::span<VertexStreamDesc const>;

		ResDesc   m_vb;         // The vertex buffer description and initialisation data
		ResDesc   m_ib;         // The index buffer description and initialisation data
		BBox      m_bbox;       // Model space bounding box
		m4x4      m_m2root;     // Model to root transform
		VBStreams m_vb_streams; // Optional model-owned streams parallel to the vertex buffer.
		string32  m_name;       // Debugging name for the model

		ModelDesc()
			: m_vb()
			, m_ib()
			, m_bbox(BBox::Reset())
			, m_m2root(m4x4::Identity())
			, m_vb_streams()
			, m_name()
		{}

		// Fluent interface for constructing a model description
		ModelDesc& name(std::string_view name)
		{
			m_name = name;
			return *this;
		}
		ModelDesc& bbox(BBox const& bbox)
		{
			m_bbox = bbox;
			return *this;
		}
		ModelDesc& m2root(m4x4 const& m2root)
		{
			m_m2root = m2root;
			return *this;
		}
		ModelDesc& vbuf(ResDesc const& vb)
		{
			m_vb = vb;
			return *this;
		}
		ModelDesc& ibuf(ResDesc const& ib)
		{
			m_ib = ib;
			return *this;
		}
		ModelDesc& vb_streams(VBStreams vb_streams)
		{
			m_vb_streams = vb_streams;
			return *this;
		}
		ModelDesc& vbuf(std::span<Vert const> data = {})
		{
			m_vb = ResDesc::VBuf<Vert>(data.size(), data);
			return *this;
		}
		ModelDesc& ibuf(geometry::index_cspan ib)
		{
			m_ib = ResDesc::IBuf(ib.size(), ib.stride(), ib);
			return *this;
		}
		template <typename TVert> ModelDesc& vbuf(std::span<TVert const> data = {})
		{
			m_vb = ResDesc::VBuf<TVert>(data.size(), data);
			return *this;
		}
		template <typename TIndx> ModelDesc& ibuf(std::span<TIndx const> data = {})
		{
			m_ib = ResDesc::IBuf<TIndx>(data.size(), data);
			return *this;
		}
		template <typename TVert, size_t VSize> ModelDesc& vbuf(TVert const (&vert)[VSize])
		{
			m_vb = ResDesc::VBuf<TVert>(VSize, { &vert[0], VSize });
			return *this;
		}
		template <typename TIndx, size_t ISize> ModelDesc& ibuf(TIndx const (&idxs)[ISize])
		{
			m_ib = ResDesc::IBuf<TIndx>(ISize, { &idxs[0], ISize });
			return *this;
		}
	};
}
