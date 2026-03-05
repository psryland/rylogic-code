//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#pragma once
#include "pr/view3d-12/forward.h"
#include "pr/view3d-12/view3d-dll.h"
#include "pr/view3d-12/model/animation.h"
#include "pr/view3d-12/utility/ray_cast.h"
#include "pr/view3d-12/utility/wrappers.h"

namespace pr
{
	// D3D12_PRIMITIVE_TOPOLOGY / rdr12::ETopo
	template <> struct Convert<D3D12_PRIMITIVE_TOPOLOGY, rdr12::ETopo>
	{
		constexpr static D3D12_PRIMITIVE_TOPOLOGY Func(rdr12::ETopo v)
		{
			switch (v)
			{
				case rdr12::ETopo::Undefined    : return D3D12_PRIMITIVE_TOPOLOGY::D3D_PRIMITIVE_TOPOLOGY_UNDEFINED;
				case rdr12::ETopo::PointList    : return D3D12_PRIMITIVE_TOPOLOGY::D3D_PRIMITIVE_TOPOLOGY_POINTLIST;
				case rdr12::ETopo::LineList     : return D3D12_PRIMITIVE_TOPOLOGY::D3D_PRIMITIVE_TOPOLOGY_LINELIST;
				case rdr12::ETopo::LineStrip    : return D3D12_PRIMITIVE_TOPOLOGY::D3D_PRIMITIVE_TOPOLOGY_LINESTRIP;
				case rdr12::ETopo::TriList      : return D3D12_PRIMITIVE_TOPOLOGY::D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST;
				case rdr12::ETopo::TriStrip     : return D3D12_PRIMITIVE_TOPOLOGY::D3D_PRIMITIVE_TOPOLOGY_TRIANGLESTRIP;
				case rdr12::ETopo::LineListAdj  : return D3D12_PRIMITIVE_TOPOLOGY::D3D_PRIMITIVE_TOPOLOGY_LINELIST_ADJ;
				case rdr12::ETopo::LineStripAdj : return D3D12_PRIMITIVE_TOPOLOGY::D3D_PRIMITIVE_TOPOLOGY_LINESTRIP_ADJ;
				case rdr12::ETopo::TriListAdj   : return D3D12_PRIMITIVE_TOPOLOGY::D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST_ADJ;
				case rdr12::ETopo::TriStripAdj  : return D3D12_PRIMITIVE_TOPOLOGY::D3D_PRIMITIVE_TOPOLOGY_TRIANGLESTRIP_ADJ;
				default: throw std::runtime_error("Topology type not supported");
			}
		}
	};
	template <> struct Convert<D3D12_PRIMITIVE_TOPOLOGY_TYPE, rdr12::ETopo>
	{
		constexpr static D3D12_PRIMITIVE_TOPOLOGY_TYPE Func(rdr12::ETopo v)
		{
			switch (v)
			{
				case rdr12::ETopo::Undefined    : return D3D12_PRIMITIVE_TOPOLOGY_TYPE::D3D12_PRIMITIVE_TOPOLOGY_TYPE_UNDEFINED;
				case rdr12::ETopo::PointList    : return D3D12_PRIMITIVE_TOPOLOGY_TYPE::D3D12_PRIMITIVE_TOPOLOGY_TYPE_POINT;
				case rdr12::ETopo::LineList     : return D3D12_PRIMITIVE_TOPOLOGY_TYPE::D3D12_PRIMITIVE_TOPOLOGY_TYPE_LINE;
				case rdr12::ETopo::LineStrip    : return D3D12_PRIMITIVE_TOPOLOGY_TYPE::D3D12_PRIMITIVE_TOPOLOGY_TYPE_LINE;
				case rdr12::ETopo::TriList      : return D3D12_PRIMITIVE_TOPOLOGY_TYPE::D3D12_PRIMITIVE_TOPOLOGY_TYPE_TRIANGLE;
				case rdr12::ETopo::TriStrip     : return D3D12_PRIMITIVE_TOPOLOGY_TYPE::D3D12_PRIMITIVE_TOPOLOGY_TYPE_TRIANGLE;
				case rdr12::ETopo::LineListAdj  : return D3D12_PRIMITIVE_TOPOLOGY_TYPE::D3D12_PRIMITIVE_TOPOLOGY_TYPE_LINE;
				case rdr12::ETopo::LineStripAdj : return D3D12_PRIMITIVE_TOPOLOGY_TYPE::D3D12_PRIMITIVE_TOPOLOGY_TYPE_LINE;
				case rdr12::ETopo::TriListAdj   : return D3D12_PRIMITIVE_TOPOLOGY_TYPE::D3D12_PRIMITIVE_TOPOLOGY_TYPE_TRIANGLE;
				case rdr12::ETopo::TriStripAdj  : return D3D12_PRIMITIVE_TOPOLOGY_TYPE::D3D12_PRIMITIVE_TOPOLOGY_TYPE_TRIANGLE;
				default: throw std::runtime_error("Topology type not supported");
			}
		}
	};

	// D3D12_RESOURCE_STATES / std::string_view
	template <> struct Convert<std::string, D3D12_RESOURCE_STATES>
	{
		constexpr static std::string Func(D3D12_RESOURCE_STATES v)
		{
			std::string s;
			if (v == D3D12_RESOURCE_STATE_COMMON)                                 s.append(s.empty() ? "" : " | ").append("COMMON");
			if (AllSet(v, D3D12_RESOURCE_STATE_VERTEX_AND_CONSTANT_BUFFER))       s.append(s.empty() ? "" : " | ").append("VERTEX_AND_CONSTANT_BUFFER");
			if (AllSet(v, D3D12_RESOURCE_STATE_INDEX_BUFFER))                     s.append(s.empty() ? "" : " | ").append("INDEX_BUFFER");
			if (AllSet(v, D3D12_RESOURCE_STATE_RENDER_TARGET))                    s.append(s.empty() ? "" : " | ").append("RENDER_TARGET");
			if (AllSet(v, D3D12_RESOURCE_STATE_UNORDERED_ACCESS))                 s.append(s.empty() ? "" : " | ").append("UNORDERED_ACCESS");
			if (AllSet(v, D3D12_RESOURCE_STATE_DEPTH_WRITE))                      s.append(s.empty() ? "" : " | ").append("DEPTH_WRITE");
			if (AllSet(v, D3D12_RESOURCE_STATE_DEPTH_READ))                       s.append(s.empty() ? "" : " | ").append("DEPTH_READ");
			if (AllSet(v, D3D12_RESOURCE_STATE_ALL_SHADER_RESOURCE))              s.append(s.empty() ? "" : " | ").append("ALL_SHADER_RESOURCE");
			else if (AllSet(v, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE))   s.append(s.empty() ? "" : " | ").append("NON_PIXEL_SHADER_RESOURCE");
			else if (AllSet(v, D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE))       s.append(s.empty() ? "" : " | ").append("PIXEL_SHADER_RESOURCE");
			if (AllSet(v, D3D12_RESOURCE_STATE_STREAM_OUT))                       s.append(s.empty() ? "" : " | ").append("STREAM_OUT");
			if (AllSet(v, D3D12_RESOURCE_STATE_INDIRECT_ARGUMENT))                s.append(s.empty() ? "" : " | ").append("INDIRECT_ARGUMENT");
			if (AllSet(v, D3D12_RESOURCE_STATE_COPY_DEST))                        s.append(s.empty() ? "" : " | ").append("COPY_DEST");
			if (AllSet(v, D3D12_RESOURCE_STATE_COPY_SOURCE))                      s.append(s.empty() ? "" : " | ").append("COPY_SOURCE");
			if (AllSet(v, D3D12_RESOURCE_STATE_RESOLVE_DEST))                     s.append(s.empty() ? "" : " | ").append("RESOLVE_DEST");
			if (AllSet(v, D3D12_RESOURCE_STATE_RESOLVE_SOURCE))                   s.append(s.empty() ? "" : " | ").append("RESOLVE_SOURCE");
			if (AllSet(v, D3D12_RESOURCE_STATE_RAYTRACING_ACCELERATION_STRUCTURE))s.append(s.empty() ? "" : " | ").append("RAYTRACING_ACCELERATION_STRUCTURE");
			if (AllSet(v, D3D12_RESOURCE_STATE_SHADING_RATE_SOURCE))              s.append(s.empty() ? "" : " | ").append("SHADING_RATE_SOURCE");
			if (AllSet(v, D3D12_RESOURCE_STATE_GENERIC_READ))                     s.append(s.empty() ? "" : " | ").append("GENERIC_READ");
			if (AllSet(v, D3D12_RESOURCE_STATE_PREDICATION))                      s.append(s.empty() ? "" : " | ").append("PREDICATION");
			if (AllSet(v, D3D12_RESOURCE_STATE_VIDEO_DECODE_READ))                s.append(s.empty() ? "" : " | ").append("VIDEO_DECODE_READ");
			if (AllSet(v, D3D12_RESOURCE_STATE_VIDEO_DECODE_WRITE))               s.append(s.empty() ? "" : " | ").append("VIDEO_DECODE_WRITE");
			if (AllSet(v, D3D12_RESOURCE_STATE_VIDEO_PROCESS_READ))               s.append(s.empty() ? "" : " | ").append("VIDEO_PROCESS_READ");
			if (AllSet(v, D3D12_RESOURCE_STATE_VIDEO_PROCESS_WRITE))              s.append(s.empty() ? "" : " | ").append("VIDEO_PROCESS_WRITE");
			if (AllSet(v, D3D12_RESOURCE_STATE_VIDEO_ENCODE_READ))                s.append(s.empty() ? "" : " | ").append("VIDEO_ENCODE_READ");
			if (AllSet(v, D3D12_RESOURCE_STATE_VIDEO_ENCODE_WRITE))               s.append(s.empty() ? "" : " | ").append("VIDEO_ENCODE_WRITE");
			return s;
		}
	};

	// D3D12_RANGE / rdr12::Range
	template <> struct Convert<D3D12_RANGE, rdr12::Range>
	{
		constexpr static D3D12_RANGE Func(rdr12::Range const& r)
		{
			return D3D12_RANGE{
				.Begin = static_cast<SIZE_T>(r.m_beg),
				.End = static_cast<SIZE_T>(r.m_end),
			};
		}
	};
	template <> struct Convert<rdr12::Range, D3D12_RANGE>
	{
		constexpr static rdr12::Range Func(D3D12_RANGE const& r)
		{
			return rdr12::Range(
				static_cast<int64_t>(r.Begin),
				static_cast<int64_t>(r.End));
		}
	};

	// Colour32 / view3d::Colour
	template <> struct Convert<Colour32, view3d::Colour>
	{
		constexpr static Colour32 Func(view3d::Colour const& v)
		{
			return Colour32(v);
		}
	};
	template <> struct Convert<view3d::Colour, Colour32>
	{
		constexpr static view3d::Colour Func(Colour32 const& v)
		{
			return view3d::Colour(v.argb);
		}
	};

	// iv2 / SIZE
	template <> struct Convert<iv2, SIZE>
	{
		constexpr static iv2 Func(SIZE const& v)
		{
			return iv2(v.cx, v.cy);
		}
	};
	template <> struct Convert<SIZE, iv2>
	{
		constexpr static SIZE Func(iv2 const& v)
		{
			return SIZE{v.x, v.y};
		}
	};

	// EAnimStyle
	template <> struct Convert<rdr12::EAnimStyle, std::string_view>
	{
		static rdr12::EAnimStyle Func(std::string_view s)
		{
			if (str::EqualI(s, "NoAnimation")) return rdr12::EAnimStyle::NoAnimation;
			if (str::EqualI(s, "Once")) return rdr12::EAnimStyle::Once;
			if (str::EqualI(s, "Repeat")) return rdr12::EAnimStyle::Repeat;
			if (str::EqualI(s, "Continuous")) return rdr12::EAnimStyle::Continuous;
			if (str::EqualI(s, "PingPong")) return rdr12::EAnimStyle::PingPong;
			throw std::runtime_error(std::format("Unknown AnimStyle value: {}", s));
		}
	};

	// v2 / view3d::Vec2
	template <> struct Convert<v2, view3d::Vec2>
	{
		constexpr static v2 Func(view3d::Vec2 const& v)
		{
			return v2(v.x, v.y);
		}
	};
	template <> struct Convert<view3d::Vec2, v2>
	{
		constexpr static view3d::Vec2 Func(v2 const& v)
		{
			return view3d::Vec2{v.x, v.y};
		}
	};

	// v4 / view3d::Vec4
	template <> struct Convert<v4, view3d::Vec4>
	{
		constexpr static v4 Func(view3d::Vec4 const& v)
		{
			return v4(v.x, v.y, v.z, v.w);
		}
	};
	template <> struct Convert<view3d::Vec4, v4>
	{
		constexpr static view3d::Vec4 Func(v4 v)
		{
			return view3d::Vec4{v.x, v.y, v.z, v.w};
		}
	};

	// m4x4 / view3d::Mat4x4
	template <> struct Convert<m4x4, view3d::Mat4x4>
	{
		constexpr static m4x4 Func(view3d::Mat4x4 const& m)
		{
			return m4x4(To<v4>(m.x),To<v4>(m.y),To<v4>(m.z),To<v4>(m.w));
		}
	};
	template <> struct Convert<view3d::Mat4x4, m4x4>
	{
		constexpr static view3d::Mat4x4 Func(m4x4 const& m)
		{
			return view3d::Mat4x4{To<view3d::Vec4>(m.x), To<view3d::Vec4>(m.y), To<view3d::Vec4>(m.z), To<view3d::Vec4>(m.w)};
		}
	};

	// BBox / view3d::BBox
	template <> struct Convert<BBox, view3d::BBox>
	{
		constexpr static BBox Func(view3d::BBox const& bbox)
		{
			return BBox(To<v4>(bbox.centre), To<v4>(bbox.radius));
		}
	};
	template <> struct Convert<view3d::BBox, BBox>
	{
		constexpr static view3d::BBox Func(BBox const& bbox)
		{
			return view3d::BBox{To<view3d::Vec4>(bbox.m_centre), To<view3d::Vec4>(bbox.m_radius)};
		}
	};

	// rdr12::MultiSamp / view3d::MultiSamp
	template <> struct Convert<rdr12::MultiSamp, view3d::MultiSamp>
	{
		constexpr static rdr12::MultiSamp Func(view3d::MultiSamp ms)
		{
			return { s_cast<uint32_t>(ms.m_count), s_cast<uint32_t>(ms.m_quality) };
		}
	};
	template <> struct Convert<view3d::MultiSamp, rdr12::MultiSamp>
	{
		constexpr static view3d::MultiSamp Func(rdr12::MultiSamp ms)
		{
			return { s_cast<int>(ms.Count), s_cast<int>(ms.Quality) };
		}
	};

	// rdr12::ESnapMode / view3d::ESnapMode
	template <> struct Convert<rdr12::ESnapMode, view3d::ESnapMode>
	{
		constexpr static rdr12::ESnapMode Func(view3d::ESnapMode v)
		{
			return static_cast<rdr12::ESnapMode>(static_cast<int>(v));
		}
	};

	// rdr12::HitTestRay / view3d::HitTestRay
	template <> struct Convert<rdr12::HitTestRay, view3d::HitTestRay>
	{
		static rdr12::HitTestRay Func(view3d::HitTestRay h);
	};

	// rdr12::HitTestReset / view3d::HitTestReset
	template <> struct Convert<rdr12::HitTestResult, view3d::HitTestResult>
	{
		static rdr12::HitTestResult Func(view3d::HitTestResult const& hit);
	};
	template <> struct Convert<view3d::HitTestResult, rdr12::HitTestResult>
	{
		static view3d::HitTestResult Func(rdr12::HitTestResult const& hit);
	};
}
