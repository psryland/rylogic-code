//*********************************************
// Compute
//  Copyright (c) Rylogic Ltd 2026
//*********************************************
#pragma once
#include "pr/compute/forward.h"

namespace pr
{
	// D3D12_PRIMITIVE_TOPOLOGY / pr::compute::ETopo
	template <> struct Convert<D3D12_PRIMITIVE_TOPOLOGY, pr::compute::ETopo>
	{
		constexpr static D3D12_PRIMITIVE_TOPOLOGY Func(pr::compute::ETopo v)
		{
			switch (v)
			{
				case pr::compute::ETopo::Undefined    : return D3D12_PRIMITIVE_TOPOLOGY::D3D_PRIMITIVE_TOPOLOGY_UNDEFINED;
				case pr::compute::ETopo::PointList    : return D3D12_PRIMITIVE_TOPOLOGY::D3D_PRIMITIVE_TOPOLOGY_POINTLIST;
				case pr::compute::ETopo::LineList     : return D3D12_PRIMITIVE_TOPOLOGY::D3D_PRIMITIVE_TOPOLOGY_LINELIST;
				case pr::compute::ETopo::LineStrip    : return D3D12_PRIMITIVE_TOPOLOGY::D3D_PRIMITIVE_TOPOLOGY_LINESTRIP;
				case pr::compute::ETopo::TriList      : return D3D12_PRIMITIVE_TOPOLOGY::D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST;
				case pr::compute::ETopo::TriStrip     : return D3D12_PRIMITIVE_TOPOLOGY::D3D_PRIMITIVE_TOPOLOGY_TRIANGLESTRIP;
				case pr::compute::ETopo::LineListAdj  : return D3D12_PRIMITIVE_TOPOLOGY::D3D_PRIMITIVE_TOPOLOGY_LINELIST_ADJ;
				case pr::compute::ETopo::LineStripAdj : return D3D12_PRIMITIVE_TOPOLOGY::D3D_PRIMITIVE_TOPOLOGY_LINESTRIP_ADJ;
				case pr::compute::ETopo::TriListAdj   : return D3D12_PRIMITIVE_TOPOLOGY::D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST_ADJ;
				case pr::compute::ETopo::TriStripAdj  : return D3D12_PRIMITIVE_TOPOLOGY::D3D_PRIMITIVE_TOPOLOGY_TRIANGLESTRIP_ADJ;
				default: throw std::runtime_error("Topology type not supported");
			}
		}
	};
	template <> struct Convert<D3D12_PRIMITIVE_TOPOLOGY_TYPE, pr::compute::ETopo>
	{
		constexpr static D3D12_PRIMITIVE_TOPOLOGY_TYPE Func(pr::compute::ETopo v)
		{
			switch (v)
			{
				case pr::compute::ETopo::Undefined    : return D3D12_PRIMITIVE_TOPOLOGY_TYPE::D3D12_PRIMITIVE_TOPOLOGY_TYPE_UNDEFINED;
				case pr::compute::ETopo::PointList    : return D3D12_PRIMITIVE_TOPOLOGY_TYPE::D3D12_PRIMITIVE_TOPOLOGY_TYPE_POINT;
				case pr::compute::ETopo::LineList     : return D3D12_PRIMITIVE_TOPOLOGY_TYPE::D3D12_PRIMITIVE_TOPOLOGY_TYPE_LINE;
				case pr::compute::ETopo::LineStrip    : return D3D12_PRIMITIVE_TOPOLOGY_TYPE::D3D12_PRIMITIVE_TOPOLOGY_TYPE_LINE;
				case pr::compute::ETopo::TriList      : return D3D12_PRIMITIVE_TOPOLOGY_TYPE::D3D12_PRIMITIVE_TOPOLOGY_TYPE_TRIANGLE;
				case pr::compute::ETopo::TriStrip     : return D3D12_PRIMITIVE_TOPOLOGY_TYPE::D3D12_PRIMITIVE_TOPOLOGY_TYPE_TRIANGLE;
				case pr::compute::ETopo::LineListAdj  : return D3D12_PRIMITIVE_TOPOLOGY_TYPE::D3D12_PRIMITIVE_TOPOLOGY_TYPE_LINE;
				case pr::compute::ETopo::LineStripAdj : return D3D12_PRIMITIVE_TOPOLOGY_TYPE::D3D12_PRIMITIVE_TOPOLOGY_TYPE_LINE;
				case pr::compute::ETopo::TriListAdj   : return D3D12_PRIMITIVE_TOPOLOGY_TYPE::D3D12_PRIMITIVE_TOPOLOGY_TYPE_TRIANGLE;
				case pr::compute::ETopo::TriStripAdj  : return D3D12_PRIMITIVE_TOPOLOGY_TYPE::D3D12_PRIMITIVE_TOPOLOGY_TYPE_TRIANGLE;
				default: throw std::runtime_error("Topology type not supported");
			}
		}
	};

	// D3D12_RESOURCE_STATES / std::string
	template <> struct Convert<std::string, D3D12_RESOURCE_STATES>
	{
		constexpr static std::string Func(D3D12_RESOURCE_STATES v)
		{
			std::string s;
			if (v == D3D12_RESOURCE_STATE_COMMON)                                  s.append(s.empty() ? "" : " | ").append("COMMON");
			if (AllSet(v, D3D12_RESOURCE_STATE_VERTEX_AND_CONSTANT_BUFFER))        s.append(s.empty() ? "" : " | ").append("VERTEX_AND_CONSTANT_BUFFER");
			if (AllSet(v, D3D12_RESOURCE_STATE_INDEX_BUFFER))                      s.append(s.empty() ? "" : " | ").append("INDEX_BUFFER");
			if (AllSet(v, D3D12_RESOURCE_STATE_RENDER_TARGET))                     s.append(s.empty() ? "" : " | ").append("RENDER_TARGET");
			if (AllSet(v, D3D12_RESOURCE_STATE_UNORDERED_ACCESS))                  s.append(s.empty() ? "" : " | ").append("UNORDERED_ACCESS");
			if (AllSet(v, D3D12_RESOURCE_STATE_DEPTH_WRITE))                       s.append(s.empty() ? "" : " | ").append("DEPTH_WRITE");
			if (AllSet(v, D3D12_RESOURCE_STATE_DEPTH_READ))                        s.append(s.empty() ? "" : " | ").append("DEPTH_READ");
			if (AllSet(v, D3D12_RESOURCE_STATE_ALL_SHADER_RESOURCE))               s.append(s.empty() ? "" : " | ").append("ALL_SHADER_RESOURCE");
			else if (AllSet(v, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE))    s.append(s.empty() ? "" : " | ").append("NON_PIXEL_SHADER_RESOURCE");
			else if (AllSet(v, D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE))        s.append(s.empty() ? "" : " | ").append("PIXEL_SHADER_RESOURCE");
			if (AllSet(v, D3D12_RESOURCE_STATE_STREAM_OUT))                        s.append(s.empty() ? "" : " | ").append("STREAM_OUT");
			if (AllSet(v, D3D12_RESOURCE_STATE_INDIRECT_ARGUMENT))                 s.append(s.empty() ? "" : " | ").append("INDIRECT_ARGUMENT");
			if (AllSet(v, D3D12_RESOURCE_STATE_COPY_DEST))                         s.append(s.empty() ? "" : " | ").append("COPY_DEST");
			if (AllSet(v, D3D12_RESOURCE_STATE_COPY_SOURCE))                       s.append(s.empty() ? "" : " | ").append("COPY_SOURCE");
			if (AllSet(v, D3D12_RESOURCE_STATE_RESOLVE_DEST))                      s.append(s.empty() ? "" : " | ").append("RESOLVE_DEST");
			if (AllSet(v, D3D12_RESOURCE_STATE_RESOLVE_SOURCE))                    s.append(s.empty() ? "" : " | ").append("RESOLVE_SOURCE");
			if (AllSet(v, D3D12_RESOURCE_STATE_RAYTRACING_ACCELERATION_STRUCTURE)) s.append(s.empty() ? "" : " | ").append("RAYTRACING_ACCELERATION_STRUCTURE");
			if (AllSet(v, D3D12_RESOURCE_STATE_SHADING_RATE_SOURCE))               s.append(s.empty() ? "" : " | ").append("SHADING_RATE_SOURCE");
			if (AllSet(v, D3D12_RESOURCE_STATE_GENERIC_READ))                      s.append(s.empty() ? "" : " | ").append("GENERIC_READ");
			if (AllSet(v, D3D12_RESOURCE_STATE_PREDICATION))                       s.append(s.empty() ? "" : " | ").append("PREDICATION");
			if (AllSet(v, D3D12_RESOURCE_STATE_VIDEO_DECODE_READ))                 s.append(s.empty() ? "" : " | ").append("VIDEO_DECODE_READ");
			if (AllSet(v, D3D12_RESOURCE_STATE_VIDEO_DECODE_WRITE))                s.append(s.empty() ? "" : " | ").append("VIDEO_DECODE_WRITE");
			if (AllSet(v, D3D12_RESOURCE_STATE_VIDEO_PROCESS_READ))                s.append(s.empty() ? "" : " | ").append("VIDEO_PROCESS_READ");
			if (AllSet(v, D3D12_RESOURCE_STATE_VIDEO_PROCESS_WRITE))               s.append(s.empty() ? "" : " | ").append("VIDEO_PROCESS_WRITE");
			if (AllSet(v, D3D12_RESOURCE_STATE_VIDEO_ENCODE_READ))                 s.append(s.empty() ? "" : " | ").append("VIDEO_ENCODE_READ");
			if (AllSet(v, D3D12_RESOURCE_STATE_VIDEO_ENCODE_WRITE))                s.append(s.empty() ? "" : " | ").append("VIDEO_ENCODE_WRITE");
			return s;
		}
	};

	// D3D12_RANGE / pr::compute::Range
	template <> struct Convert<D3D12_RANGE, pr::compute::Range>
	{
		constexpr static D3D12_RANGE Func(pr::compute::Range const& r)
		{
			return D3D12_RANGE{
				.Begin = static_cast<SIZE_T>(r.m_beg),
				.End = static_cast<SIZE_T>(r.m_end),
			};
		}
	};
	template <> struct Convert<pr::compute::Range, D3D12_RANGE>
	{
		constexpr static pr::compute::Range Func(D3D12_RANGE const& r)
		{
			return pr::compute::Range(
				static_cast<int64_t>(r.Begin),
				static_cast<int64_t>(r.End));
		}
	};
}
