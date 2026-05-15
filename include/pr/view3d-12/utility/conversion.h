//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#pragma once
#include "pr/view3d-12/forward.h"
#include "pr/view3d-12/view3d-dll.h"
#include "pr/view3d-12/model/animation.h"
#include "pr/view3d-12/utility/ray_cast.h"

namespace pr
{
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

	// compute::MultiSamp / view3d::MultiSamp
	template <> struct Convert<compute::MultiSamp, view3d::MultiSamp>
	{
		constexpr static compute::MultiSamp Func(view3d::MultiSamp ms)
		{
			return { s_cast<uint32_t>(ms.m_count), s_cast<uint32_t>(ms.m_quality) };
		}
	};
	template <> struct Convert<view3d::MultiSamp, compute::MultiSamp>
	{
		constexpr static view3d::MultiSamp Func(compute::MultiSamp ms)
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
