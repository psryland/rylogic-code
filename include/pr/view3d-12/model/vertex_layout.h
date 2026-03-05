//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#pragma once
#include "pr/view3d-12/forward.h"

namespace pr::rdr12
{
	// Only support 1 vertex format because extra formats multiply the number of shader
	// permutations. If in the future more data is needed in the vertex format, just add it
	// and update the shaders to handle the case when the data is missing/invalid.
	// To do this, prefer to do degenerate calculations than 'if' statements in the shaders.
	// Also, using full fat v4s to allow for encoding extra info into unused members.
	//
	// Although there is only one format, code the rest of the renderer assuming 'Vert'
	// is a template parameter. Specialised shaders may wish to create specific vertex
	// formats (e.g. texture transforming shader, say)

	// *The* vertex format
	struct Vert
	{
		// This allows code templated on vertex type to ask what geometry components are supported
		// In many cases, a model will have nuggets with a subset of these geom flags.
		static constexpr EGeom GeomMask = EGeom::Vert | EGeom::Colr | EGeom::Norm | EGeom::Tex0;

		v4     m_vert;
		Colour m_diff;
		v4     m_norm;
		v2     m_tex0;
		iv2    m_idx0;

		// The vertex layout description
		static D3D12_INPUT_ELEMENT_DESC const (&Layout())[5]
		{
			static D3D12_INPUT_ELEMENT_DESC const s_desc[] =
			{
				{"POSITION" , 0 , DXGI_FORMAT_R32G32B32A32_FLOAT , 0 , offsetof(Vert , m_vert) , D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA , 0},
				{"COLOR"    , 0 , DXGI_FORMAT_R32G32B32A32_FLOAT , 0 , offsetof(Vert , m_diff) , D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA , 0},
				{"NORMAL"   , 0 , DXGI_FORMAT_R32G32B32A32_FLOAT , 0 , offsetof(Vert , m_norm) , D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA , 0},
				{"TEXCOORD" , 0 , DXGI_FORMAT_R32G32_FLOAT       , 0 , offsetof(Vert , m_tex0) , D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA , 0},
				{"INDICES"  , 0 , DXGI_FORMAT_R32G32_SINT        , 0 , offsetof(Vert , m_idx0) , D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA , 0},
			};
			return s_desc;
		}
		static D3D12_INPUT_LAYOUT_DESC LayoutDesc()
		{
			return D3D12_INPUT_LAYOUT_DESC {
				.pInputElementDescs = &Layout()[0],
				.NumElements = _countof(Layout()),
			};
		}
	};
	static_assert(std::is_trivially_copyable_v<Vert> && std::is_trivially_default_constructible_v<Vert>);

	constexpr v4 GetP(Vert const& vert)
	{
		return vert.m_vert;
	}
	constexpr Colour const& GetC(Vert const& vert)
	{
		return vert.m_diff;
	}
	constexpr v4 GetN(Vert const& vert)
	{
		return vert.m_norm;
	}
	constexpr v2 const& GetT(Vert const& vert)
	{
		return vert.m_tex0;
	}

	// Don't set values that aren't given, allows these functions to be composed
	constexpr void SetP(Vert& vert, v4 pos)
	{
		vert.m_vert = pos;
	}
	constexpr void SetC(Vert& vert, Colour const& col)
	{
		vert.m_diff = col;
	}
	constexpr void SetN(Vert& vert, v4 norm)
	{
		vert.m_norm = norm;
	}
	constexpr void SetT(Vert& vert, v2 const& uv)
	{
		vert.m_tex0 = uv;
	}
	constexpr void SetPC(Vert& vert, v4 pos, Colour const& col)
	{
		vert.m_vert = pos;
		vert.m_diff = col;
	}
	constexpr void SetPT(Vert& vert, v4 pos, v2 const& uv)
	{
		vert.m_vert = pos;
		vert.m_tex0 = uv;
	}
	constexpr void SetPCN(Vert& vert, v4 pos, Colour const& col, v4 norm)
	{
		vert.m_vert = pos;
		vert.m_diff = col;
		vert.m_norm = norm;
	}
	constexpr void SetPCNT(Vert& vert, v4 pos, Colour const& col, v4 norm, v2 const& uv)
	{
		vert.m_vert = pos;
		vert.m_diff = col;
		vert.m_norm = norm;
		vert.m_tex0 = uv;
	}
	constexpr void SetPCNTI(Vert& vert, v4 pos, Colour const& col, v4 norm, v2 const& uv, iv2 const& idx)
	{
		vert.m_vert = pos;
		vert.m_diff = col;
		vert.m_norm = norm;
		vert.m_tex0 = uv;
		vert.m_idx0 = idx;
	}
}
namespace pr
{
	inline rdr12::Vert const& Grow(BBox& bbox, rdr12::Vert const& vert)
	{
		Grow(bbox, vert.m_vert);
		return vert;
	}
}
