//********************************
// Ldraw Script Binary Serialiser
//  Copyright (c) Rylogic Ltd 2025
//********************************
#pragma once
#include "pr/view3d-12/ldraw/ldraw.h"
#include "pr/view3d-12/ldraw/ldraw_serialiser.h"

namespace pr::rdr12::ldraw
{
	struct BinaryWriter
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
			//static_assert(std::is_same_v<std::false_type, Type>, "no overload for 'Type'");
		}
		template <typename TOut> static void Append(TOut& out, std::string_view str)
		{
			traits<TOut>::write(out, { byte_ptr(str.data()), str.size() });
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
			uint8_t bool_ = b ? 1 : 0;
			traits<TOut>::write(out, { byte_ptr(&bool_), sizeof(bool_) });
		}
		template <typename TOut> static void Append(TOut& out, int8_t i)
		{
			traits<TOut>::write(out, { byte_ptr(&i), sizeof(i) });
		}
		template <typename TOut> static void Append(TOut& out, int16_t i)
		{
			traits<TOut>::write(out, { byte_ptr(&i), sizeof(i) });
		}
		template <typename TOut> static void Append(TOut& out, int32_t i)
		{
			traits<TOut>::write(out, { byte_ptr(&i), sizeof(i) });
		}
		template <typename TOut> static void Append(TOut& out, int64_t i)
		{
			traits<TOut>::write(out, { byte_ptr(&i), sizeof(i) });
		}
		template <typename TOut> static void Append(TOut& out, uint8_t u)
		{
			traits<TOut>::write(out, { byte_ptr(&u), sizeof(u) });
		}
		template <typename TOut> static void Append(TOut& out, uint16_t u)
		{
			traits<TOut>::write(out, { byte_ptr(&u), sizeof(u) });
		}
		template <typename TOut> static void Append(TOut& out, uint32_t u)
		{
			traits<TOut>::write(out, { byte_ptr(&u), sizeof(u) });
		}
		template <typename TOut> static void Append(TOut& out, uint64_t u)
		{
			traits<TOut>::write(out, { byte_ptr(&u), sizeof(u) });
		}
		template <typename TOut> static void Append(TOut& out, float f)
		{
			traits<TOut>::write(out, { byte_ptr(&f), sizeof(f) });
		}
		template <typename TOut> static void Append(TOut& out, double f)
		{
			traits<TOut>::write(out, { byte_ptr(&f), sizeof(f) });
		}
		template <typename TOut> static void Append(TOut& out, Colour32 c)
		{
			Append(out, c.argb);
		}
		template <typename TOut> static void Append(TOut& out, v2 v)
		{
			traits<TOut>::write(out, { byte_ptr(&v), sizeof(v) });
		}
		template <typename TOut> static void Append(TOut& out, v3 v)
		{
			traits<TOut>::write(out, { byte_ptr(&v), sizeof(v) });
		}
		template <typename TOut> static void Append(TOut& out, v4 v)
		{
			traits<TOut>::write(out, { byte_ptr(&v), sizeof(v) });
		}
		template <typename TOut> static void Append(TOut& out, m4x4 m)
		{
			traits<TOut>::write(out, { byte_ptr(&m), sizeof(m) });
		}
		//template <typename TOut, ScalarType S> static void Append(TOut& out, Vec2<S, void> v)
		//{
		//	traits<TOut>::write(out, { byte_ptr(&v), sizeof(v) });
		//}
		//template <typename TOut, ScalarType S> static void Append(TOut& out, Vec3<S, void> v)
		//{
		//	traits<TOut>::write(out, { byte_ptr(&v), sizeof(v) });
		//}
		//template <typename TOut, ScalarType S> static void Append(TOut& out, Vec4<S, void> v)
		//{
		//	traits<TOut>::write(out, { byte_ptr(&v), sizeof(v) });
		//}
		//template <typename TOut, ScalarType S> static void Append(TOut& out, Mat4x4<S, void, void> m)
		//{
		//	traits<TOut>::write(out, { byte_ptr(&m), sizeof(m) });
		//}
		template <typename TOut> static void Append(TOut& out, EAddrMode addr)
		{
			Append(out, s_cast<int>(addr));
		}
		template <typename TOut> static void Append(TOut& out, EFilter filter)
		{
			Append(out, s_cast<int>(filter));
		}
		template <typename TOut> static void Append(TOut& out, EPointStyle style)
		{
			using ut = std::underlying_type_t<EPointStyle>;
			Append(out, static_cast<ut>(style));
		}
		template <typename TOut> static void Append(TOut& out, ELineStyle style)
		{
			using ut = std::underlying_type_t<ELineStyle>;
			Append(out, static_cast<ut>(style));
		}
		template <typename TOut> static void Append(TOut& out, EArrowType type)
		{
			using ut = std::underlying_type_t<EArrowType>;
			Append(out, static_cast<ut>(type));
		}
		template <typename TOut> static void Append(TOut& out, VariableInt var_int)
		{
			// Variable sized int, write 6 bits at a time: xx444444 33333322 22221111 11000000
			uint8_t bits[5] = {}; int i = 5;
			for (int val = var_int.m_value; val != 0 && i-- != 0; val >>= 6)
				bits[i] = static_cast<uint8_t>(0x80 | (val & 0b00111111));

			traits<TOut>::write(out, { byte_ptr(&bits[0] + i), static_cast<size_t>(5 - i) });
		}
		template <typename TOut> static void Append(TOut& out, StringWithLength str)
		{
			Append(out, VariableInt{ static_cast<int>(str.m_value.size()) }); // length in bytes, not utf8 codes
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
			Write(out, c.m_kw, c.m_colour.argb);
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
		template <typename TOut, PrimitiveSpanType TSpan> static void Append(TOut& out, TSpan span)
		{
			traits<TOut>::write(out, { byte_ptr(span.data()), span.size() * sizeof(typename TSpan::value_type) });
		}
		template <typename TOut, typename... TItems> static void Append(TOut& out, TItems&&... items)
		{
			(Append(out, std::forward<TItems>(items)), ...);
		}

		// Write custom data within a section
		template <typename TOut, std::invocable<> AddBodyFn>
		static void Write(TOut& out, EKeyword keyword, Name name, Colour colour, AddBodyFn body_cb)
		{
			// Record the write pointer position
			auto ofs = traits<TOut>::tellp(out);

			// Write a dummy header
			SectionHeader header = { .m_keyword = keyword };
			traits<TOut>::write(out, { byte_ptr(&header), sizeof(header) });

			// Optional name/colour
			Append(out, name);
			Append(out, colour);

			// Write the section body
			body_cb();

			// Update the header with the correct size
			header.m_size = static_cast<int>(traits<TOut>::tellp(out) - ofs - sizeof(header));
			traits<TOut>::write(out, { byte_ptr(&header), sizeof(header) }, ofs);
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

		// Convert to std::byte*
		template <typename T> static constexpr std::byte const* byte_ptr(T const* t)
		{
			return reinterpret_cast<std::byte const*>(t);
		}
		template <typename T> static constexpr std::byte* byte_ptr(T* t)
		{
			return reinterpret_cast<std::byte*>(t);
		}
	};
}
