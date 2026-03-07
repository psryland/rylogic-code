//********************************
// Ldraw Script Text Serialiser
//  Copyright (c) Rylogic Ltd 2025
//********************************
#pragma once
#include "pr/view3d-12/ldraw/ldraw.h"
#include "pr/view3d-12/ldraw/ldraw_serialiser.h"

namespace pr::rdr12::ldraw
{
	struct TextWriter
	{
		template <typename T> struct no_overload;
		template <typename TOut, typename Type> static void Append(TOut& out, Type)
		{
			// Note: if you hit this error, it's probably because Append(out, ???) is being
			// called where ??? is a type not handled by the overloads of Append.
			// Also watch out for the error being a typedef of a common type,
			// e.g. I've seen 'Type=std::ios_base::openmode' as an error, but really it was
			// 'Type=int' that was missing, because of 'typedef int std::ios_base::openmode'
			no_overload<Type> missing_overload;
			//static_assert(std::is_same_v<Type, std::false_type>, "no overload for 'Type'");
		}
		template <typename TOut> static void Append(TOut& out, std::string_view s)
		{
			if (s.empty()) return;
			if (*s.data() != '}' && *s.data() != ')' && *s.data() != ' ' && traits<TOut>::last(out) != '{')
			{
				traits<TOut>::append(out, " ");
			}
			traits<TOut>::append(out, s);
		}
		template <typename TOut> static void Append(TOut& out, std::string s)
		{
			Append(out, std::string_view{ s });
		}
		template <typename TOut> static void Append(TOut& out, string32 s)
		{
			Append(out, std::string_view{ s });
		}
		template <typename TOut> static void Append(TOut& out, char const* s)
		{
			Append(out, std::string_view{ s });
		}
		template <typename TOut> static void Append(TOut& out, bool b)
		{
			Append(out, b ? "true" : "false");
		}
		template <typename TOut> static void Append(TOut& out, int i)
		{
			Append(out, To<string32>(i));
		}
		template <typename TOut> static void Append(TOut& out, long i)
		{
			Append(out, To<string32>(i));
		}
		template <typename TOut> static void Append(TOut& out, float f)
		{
			Append(out, To<string32>(f));
		}
		template <typename TOut> static void Append(TOut& out, double f)
		{
			Append(out, To<string32>(f));
		}
		template <typename TOut> static void Append(TOut& out, uint32_t u)
		{
			Append(out, std::format("{:08X}", u));
		}
		template <typename TOut> static void Append(TOut& out, Colour32 c)
		{
			Append(out, c.argb);
		}
		template <typename TOut> static void Append(TOut& out, v2 v)
		{
			Append(out, To<string32>(v));
		}
		template <typename TOut> static void Append(TOut& out, v3 v)
		{
			Append(out, To<string32>(v));
		}
		template <typename TOut> static void Append(TOut& out, v4 v)
		{
			Append(out, To<string32>(v));
		}
		template <typename TOut> static void Append(TOut& out, m4x4 m)
		{
			Append(out, To<string32>(m));
		}
		//template <typename TOut, Scalar S> static void Append(TOut& out, Vec2<S, void> v)
		//{
		//	Append(out, To<string32>(v));
		//}
		//template <typename TOut, Scalar S> static void Append(TOut& out, Vec3<S, void> v)
		//{
		//	Append(out, To<string32>(v));
		//}
		//template <typename TOut, Scalar S> static void Append(TOut& out, Vec4<S, void> v)
		//{
		//	Append(out, To<string32>(v));
		//}
		//template <typename TOut, Scalar S> static void Append(TOut& out, Mat4x4<S, void, void> m)
		//{
		//	Append(out, To<string32>(m));
		//}
		template <typename TOut> static void Append(TOut& out, EAddrMode addr)
		{
			switch (addr)
			{
				case EAddrMode::Wrap:       return Append(out, "Wrap");
				case EAddrMode::Mirror:     return Append(out, "Mirror");
				case EAddrMode::Clamp:      return Append(out, "Clamp");
				case EAddrMode::Border:     return Append(out, "Border");
				case EAddrMode::MirrorOnce: return Append(out, "MirrorOnce");
				default: throw std::runtime_error("Unknown texture addressing mode");
			}
		}
		template <typename TOut> static void Append(TOut& out, EFilter filter)
		{
			switch (filter)
			{
				case EFilter::Point:             return Append(out, "Point");
				case EFilter::PointPointLinear:  return Append(out, "PointPointLinear");
				case EFilter::PointLinearPoint:  return Append(out, "PointLinearPoint");
				case EFilter::PointLinearLinear: return Append(out, "PointLinearLinear");
				case EFilter::LinearPointPoint:  return Append(out, "LinearPointPoint");
				case EFilter::LinearPointLinear: return Append(out, "LinearPointLinear");
				case EFilter::LinearLinearPoint: return Append(out, "LinearLinearPoint");
				case EFilter::Linear:            return Append(out, "Linear");
				case EFilter::Anisotropic:       return Append(out, "Anisotropic");
				default: throw std::runtime_error("Unknown texture addressing mode");
			}
		}
		template <typename TOut> static void Append(TOut& out, EPointStyle style)
		{
			switch (style)
			{
				case EPointStyle::Square:   return Append(out, "Square");
				case EPointStyle::Circle:   return Append(out, "Circle");
				case EPointStyle::Triangle: return Append(out, "Triangle");
				case EPointStyle::Star:     return Append(out, "Star");
				case EPointStyle::Annulus:  return Append(out, "Annulus");
				default: throw std::runtime_error("Unknown arrow type");
			}
		}
		template <typename TOut> static void Append(TOut& out, ELineStyle style)
		{
			switch (style)
			{
				case ELineStyle::LineSegments:  return Append(out, "LineSegments");
				case ELineStyle::LineStrip:     return Append(out, "LineStrip");
				case ELineStyle::Direction:     return Append(out, "Direction");
				case ELineStyle::BezierSpline:  return Append(out, "BezierSpline");
				case ELineStyle::HermiteSpline: return Append(out, "HermiteSpline");
				case ELineStyle::BSplineSpline: return Append(out, "BSplineSpline");
				case ELineStyle::CatmullRom:    return Append(out, "CatmullRom");
				default: throw std::runtime_error("Unknown line style");
			}
		}
		template <typename TOut> static void Append(TOut& out, EArrowType type)
		{
			switch (type)
			{
				case EArrowType::Line:    return Append(out, "Line");
				case EArrowType::Fwd:     return Append(out, "Fwd");
				case EArrowType::Back:    return Append(out, "Back");
				case EArrowType::FwdBack: return Append(out, "FwdBack");
				default: throw std::runtime_error("Unknown arrow type");
			}
		}
		template <typename TOut> static void Append(TOut& out, VariableInt var_int)
		{
			Append(out, var_int.m_value);
		}
		template <typename TOut> static void Append(TOut& out, StringWithLength str)
		{
			Append(out, str.m_value);
		}
		template <typename TOut> static void Append(TOut& out, Name n)
		{
			if (n.m_name.empty()) return;
			Write(out, EKeyword::Name, n.m_name);
		}
		template <typename TOut> static void Append(TOut& out, Colour c)
		{
			if (c.m_colour.argb == 0xFFFFFFFF) return;
			return Write(out, c.m_kw, c.m_colour);
		}
		template <typename TOut> static void Append(TOut& out, Size s)
		{
			if (s.m_size == 0) return;
			Write(out, EKeyword::Size, s.m_size);
		}
		template <typename TOut> static void Append(TOut& out, Size2 s)
		{
			if (s.m_size == v2::Zero()) return;
			Write(out, EKeyword::Size, s.m_size);
		}
		template <typename TOut> static void Append(TOut& out, Scale s)
		{
			if (s.m_scale == 1.0f) return;
			Write(out, EKeyword::Scale, s.m_scale);
		}
		template <typename TOut> static void Append(TOut& out, Scale2 s)
		{
			if (s.m_scale == v2::One()) return;
			Write(out, EKeyword::Scale, s.m_scale);
		}
		template <typename TOut> static void Append(TOut& out, Scale3 s)
		{
			if (s.m_scale == v3::One()) return;
			Write(out, EKeyword::Scale, s.m_scale);
		}
		template <typename TOut> static void Append(TOut& out, PerItemColour c)
		{
			if (!c) return;
			Write(out, EKeyword::PerItemColour, c.m_per_item_colour);
		}
		template <typename TOut> static void Append(TOut& out, Width w)
		{
			if (!w) return;
			Write(out, EKeyword::Width, w.m_width);
		}
		template <typename TOut> static void Append(TOut& out, Depth d)
		{
			if (!d) return;
			Write(out, EKeyword::Depth, d.m_depth);
		}
		template <typename TOut> static void Append(TOut& out, Hidden h)
		{
			if (!h) return;
			Write(out, EKeyword::Hidden, h.m_hide);
		}
		template <typename TOut> static void Append(TOut& out, Wireframe w)
		{
			if (!w) return;
			Write(out, EKeyword::Wireframe, w.m_wire);
		}
		template <typename TOut> static void Append(TOut& out, Alpha a)
		{
			if (!a) return;
			Write(out, EKeyword::Alpha, a.m_has_alpha);
		}
		template <typename TOut> static void Append(TOut& out, Solid s)
		{
			if (!s) return;
			Write(out, EKeyword::Solid, s.m_solid);
		}
		template <typename TOut> static void Append(TOut& out, Smooth s)
		{
			if (!s) return;
			Write(out, EKeyword::Smooth, s.m_smooth);
		}
		template <typename TOut> static void Append(TOut& out, Dashed d)
		{
			if (!d) return;
			Write(out, EKeyword::Dashed, d.m_dash);
		}
		template <typename TOut> static void Append(TOut& out, DataPoints dp)
		{
			if (!dp) return;
			Write(out, EKeyword::Arrow, [&]
			{
				Write(out, EKeyword::Size, dp.m_size);
				if (dp.m_style != EPointStyle::Square)
					Write(out, EKeyword::Style, dp.m_style);
				if (dp.m_colour != Colour32White)
					Write(out, EKeyword::Colour, dp.m_colour);
			});
		}
		template <typename TOut> static void Append(TOut& out, LeftHanded lh)
		{
			if (!lh) return;
			Write(out, EKeyword::LeftHanded, lh.m_lh);
		}
		template <typename TOut> static void Append(TOut& out, AxisId a)
		{
			if (!a) return;
			Write(out, EKeyword::AxisId, a.m_axis.value);
		}
		template <typename TOut> static void Append(TOut& out, PointStyle p)
		{
			if (!p) return;
			Write(out, EKeyword::Style, p.m_style);
		}
		template <typename TOut> static void Append(TOut& out, LineStyle l)
		{
			if (!l) return;
			Write(out, EKeyword::Style, l.m_style);
		}
		template <typename TOut> static void Append(TOut& out, ArrowHeads a)
		{
			if (!a) return;
			Write(out, EKeyword::Arrow, a.m_type, a.m_size);
		}
		template <typename TOut> static void Append(TOut& out, Pos p)
		{
			if (p.IsOrigin()) return;
			Write(out, EKeyword::O2W, [&]
			{
				Write(out, EKeyword::Pos, p.m_pos.xyz);
			});
		}
		template <typename TOut> static void Append(TOut& out, O2W o2w)
		{
			if (o2w.IsIdentity())
				return;

			if (o2w.IsTranslation())
			{
				return Write(out, EKeyword::O2W, [&]
				{
					Write(out, EKeyword::Pos, o2w.m_mat.pos.xyz);
				});
			}

			Write(out, EKeyword::O2W, [&]
			{
				if (!o2w.IsAffine()) Write(out, EKeyword::NonAffine);
				Write(out, EKeyword::M4x4, o2w.m_mat);
			});
		}
		template <typename TOut, typename... Args> static void Append(TOut& out, Args&&... args)
		{
			(Append(out, std::forward<Args>(args)), ...);
		}

		// Write custom data within a section
		template <typename TOut, std::invocable AddBodyFn>
		static void Write(TOut& out, EKeyword keyword, Name name, Colour colour, AddBodyFn body_cb)
		{
			// Keyword
			traits<TOut>::append(out, "*");
			traits<TOut>::append(out, EKeyword_::ToStringA(keyword));

			// Optional name/colour
			if (!name.m_name.empty())
				Append(out, std::string_view{ name.m_name });
			if (!colour.IsDefault())
				Append(out, colour.m_colour);

			// Section start
			traits<TOut>::append(out, " {");

			// Write the section body
			body_cb();

			// Section end
			traits<TOut>::append(out, "}");
		}
		template <typename TOut, std::invocable AddBodyFn>
		static void Write(TOut& out, EKeyword keyword, AddBodyFn body_cb)
		{
			Write(out, keyword, {}, {}, body_cb);
		}

		// Write a single primitive type
		template <typename TOut, typename... TItem> requires (!std::is_invocable_v<TItem> && ...)
		static void Write(TOut& out, EKeyword keyword, Name name, Colour colour, TItem&&... items)
		{
			return Write(out, keyword, name, colour, [&]
			{
				(Append(out, std::forward<TItem>(items)), ...);
			});
		}
		template <typename TOut, typename... TItem> requires (!std::is_invocable_v<TItem> && ...)
		static void Write(TOut& out, EKeyword keyword, TItem&&... items)
		{
			return Write(out, keyword, {}, {}, [&]
			{
				(Append(out, std::forward<TItem>(items)), ...);
			});
		}

		// Convert values to string
		template <typename T, int N> static std::string_view conv(char(&buf)[N], T value)
		{
			auto [ptr, ec] = std::to_chars(&buf[0], &buf[0] + N, value);
			if (ec != std::errc{}) throw std::system_error(std::make_error_code(ec));
			return std::string_view{ &buf[0], static_cast<size_t>(ptr - &buf[0]) };
		}
		template <typename T, int N> static std::string_view conv(char(&buf)[N], T value, int base)
		{
			auto [ptr, ec] = std::to_chars(&buf[0], &buf[0] + N, value, base);
			if (ec != std::errc{}) throw std::system_error(std::make_error_code(ec));
			return std::string_view{ &buf[0], static_cast<size_t>(ptr - &buf[0]) };
		}
	};
}
