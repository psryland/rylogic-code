//********************************
// Ldraw Script type wrappers
//  Copyright (c) Rylogic Ltd 2014
//********************************
#pragma once
#include "pr/view3d-12/ldraw/ldraw.h"

namespace pr::rdr12::ldraw
{
	struct Name
	{
		std::string m_name;
		Name() :m_name() {}
		Name(std::string_view str) :m_name(Sanitise(str)) {}
		Name(std::string const& str) :m_name(Sanitise(str)) {}
		template <int N> Name(char const (&str)[N]) :m_name(Sanitise(str)) {}
		static std::string Sanitise(std::string_view name)
		{
			std::string result(name);
			for (auto& ch : result) ch = std::isalnum(ch) ? ch : '_';
			if (!result.empty() && !std::isalpha(result[0])) result.insert(0, 1, '_');
			return result;
		}
	};
	struct Colour
	{
		Colour32 m_colour;
		EKeyword m_kw;
		Colour() :m_colour(0xFFFFFFFF), m_kw(EKeyword::Colour) {}
		Colour(Colour32 c) :m_colour(c), m_kw(EKeyword::Colour) {}
		Colour(uint32_t argb) :m_colour(argb), m_kw(EKeyword::Colour) {}
		Colour(Colour32 c, EKeyword kw) :m_colour(c), m_kw(kw) {}
		bool IsDefault() const { return m_colour == 0xFFFFFFFF; }
	};
	struct Size
	{
		float m_size;
		Size() : m_size() {}
		Size(float size) : m_size(size) {}
		Size(int size) : m_size(float(size)) {}
	};
	struct Size2
	{
		v2 m_size;
		Size2() : m_size() {}
		Size2(v2 size) : m_size(size) {}
		Size2(iv2 size) : m_size(static_cast<float>(size.x), static_cast<float>(size.y)) {}
	};
	struct Scale
	{
		float m_scale;
		Scale() : m_scale(1) {}
		Scale(float scale) : m_scale(scale) {}
	};
	struct Scale2
	{
		v2 m_scale;
		Scale2() : m_scale(v2::One()) {}
		Scale2(v2 scale) : m_scale(scale) {}
	};
	struct Scale3
	{
		v3 m_scale;
		Scale3() : m_scale(v3::One()) {}
		Scale3(v3 scale) :m_scale(scale) {}
	};
	struct PerItemColour
	{
		bool m_per_item_colour;
		bool m_is_default;
		PerItemColour() : m_per_item_colour(), m_is_default(true) {}
		PerItemColour(bool has_colours) : m_per_item_colour(has_colours), m_is_default(false) {}
		explicit operator bool() const { return !m_is_default || m_per_item_colour; }
	};
	struct Width
	{
		float m_width;
		bool m_is_default;
		Width() :m_width(0), m_is_default(true) {}
		Width(float w) :m_width(w), m_is_default(false) {}
		Width(int w) :m_width(float(w)), m_is_default(false) {}
		explicit operator bool() const { return !m_is_default || m_width != 0.0f; }
	};
	struct Depth
	{
		bool m_depth;
		bool m_is_default;
		Depth() :m_depth(false), m_is_default(true) {}
		Depth(bool d) : m_depth(d), m_is_default(false) {}
		explicit operator bool() const { return !m_is_default || m_depth; }
	};
	struct Hidden
	{
		bool m_hide;
		bool m_is_default;
		Hidden() :m_hide(false), m_is_default(true) {}
		Hidden(bool h) :m_hide(h), m_is_default(false) {}
		explicit operator bool() const { return !m_is_default || m_hide; }
	};
	struct Wireframe
	{
		bool m_wire;
		bool m_is_default;
		Wireframe() :m_wire(false), m_is_default(true) {}
		Wireframe(bool w) :m_wire(w), m_is_default(false) {}
		explicit operator bool() const { return !m_is_default || m_wire; }
	};
	struct Alpha
	{
		bool m_has_alpha;
		bool m_is_default;
		Alpha() :m_has_alpha(false), m_is_default(true) {}
		Alpha(bool a) : m_has_alpha(a), m_is_default(false) {}
		explicit operator bool() const { return !m_is_default || m_has_alpha; }
	};
	struct Solid
	{
		bool m_solid;
		bool m_is_default;
		Solid() :m_solid(false), m_is_default(true) {}
		Solid(bool s) : m_solid(s), m_is_default(false) {}
		explicit operator bool() const { return !m_is_default || m_solid; }
	};
	struct Smooth
	{
		bool m_smooth;
		bool m_is_default;
		Smooth() :m_smooth(false), m_is_default(true) {}
		Smooth(bool s) : m_smooth(s), m_is_default(false) {}
		explicit operator bool() const { return !m_is_default || m_smooth; }
	};
	struct Dashed
	{
		v2 m_dash;
		bool m_is_default;
		Dashed() : m_dash(v2::Zero()), m_is_default(true) {}
		Dashed(v2 dash) : m_dash(dash), m_is_default(false) {}
		explicit operator bool() const { return !m_is_default || !All(m_dash == v2::Zero()); }
	};
	struct DataPoints
	{
		v2 m_size;
		Colour32 m_colour;
		EPointStyle m_style;
		bool m_is_default;
		DataPoints() : m_size(v2::Zero()), m_colour(Colour32White), m_style(EPointStyle::Square), m_is_default(true) {}
		DataPoints(v2 size, Colour32 colour, EPointStyle style) : m_size(size), m_colour(colour), m_style(style), m_is_default(false) {}
		explicit operator bool() const { return !m_is_default || !All(m_size == v2::Zero()) || m_colour != Colour32White || m_style != EPointStyle::Square; }
	};
	struct LeftHanded
	{
		bool m_lh;
		bool m_is_default;
		LeftHanded() :m_lh(false), m_is_default(true) {}
		LeftHanded(bool lh) : m_lh(lh), m_is_default(false) {}
		explicit operator bool() const { return !m_is_default || m_lh; }
	};
	struct AxisId
	{
		pr::AxisId m_axis;
		bool m_is_default;
		AxisId() : m_axis(pr::AxisId::None), m_is_default(true) {}
		AxisId(pr::AxisId axis) : m_axis(axis), m_is_default(false) {}
		explicit operator bool() const { return !m_is_default || m_axis != pr::AxisId::None; }
	};
	struct PointStyle
	{
		EPointStyle m_style;
		bool m_is_default;
		PointStyle() : m_style(EPointStyle::Square), m_is_default(true) {}
		PointStyle(EPointStyle style) : m_style(style), m_is_default(false) {}
		explicit operator bool() const { return !m_is_default || m_style != EPointStyle::Square; }
	};
	struct LineStyle
	{
		ELineStyle m_style;
		bool m_is_default;
		LineStyle() : m_style(ELineStyle::LineSegments), m_is_default(true) {}
		LineStyle(ELineStyle style) : m_style(style), m_is_default(false) {}
		explicit operator bool() const { return !m_is_default || m_style != ELineStyle::LineSegments; }
	};
	struct ArrowHeads
	{
		EArrowType m_type;
		float m_size;
		bool m_is_default;
		ArrowHeads() : m_type(EArrowType::Line), m_size(), m_is_default(true) {}
		ArrowHeads(EArrowType type, float size = 10) : m_type(type), m_size(size), m_is_default(false) {}
		explicit operator bool() const { return !m_is_default || m_type != EArrowType::Line; }
	};
	struct Pos
	{
		v4 m_pos;
		Pos() : m_pos(v4::Origin()) {}
		Pos(v4 pos) : m_pos(pos) {}
		Pos(m4x4 const& mat) : Pos(mat.pos) {}
		bool IsOrigin() const { return All(m_pos == v4::Origin()); }
	};
	struct O2W
	{
		m4x4 m_mat;
		O2W() : m_mat(m4x4::Identity()) {}
		O2W(m4x4 const& mat) :m_mat(mat) {}
		O2W(v4 pos) : m_mat(m4x4::Identity()) { m_mat.pos = pos; }
		bool IsIdentity() const { return All(m_mat == m4x4::Identity()); }
		bool IsTranslation() const { return All(m_mat.x == v4::XAxis()) && All(m_mat.y == v4::YAxis()) && All(m_mat.z == v4::ZAxis()) && m_mat.pos.w == 1; }
		bool IsAffine() const { return pr::IsAffine(m_mat); }
	};
	struct VariableInt
	{
		int m_value;
		VariableInt() : m_value() {}
		VariableInt(int value) : m_value(value & 0x3FFFFFFF) {}
	};
	struct StringWithLength
	{
		std::string_view m_value;
		StringWithLength() : m_value() {}
		StringWithLength(std::string_view value) : m_value(value) {}
	};

	// Traits for an output stream (e.g. 'std::ostream')
	template <typename TOut> struct traits
	{
		// Binary writer traits
		static TOut& write(TOut& out, std::span<std::byte const> data, int64_t ofs = -1)
		{
			if (ofs != -1)
				out.seekp(ofs).write(char_ptr(data.data()), data.size()).seekp(0, std::ios::end);
			else
				out.write(char_ptr(data.data()), data.size());
			return out;
		}
		static int64_t tellp(TOut& out)
		{
			return out.tellp();
		}

		// Text writer traits
		static TOut& append(TOut& out, std::string_view data)
		{
			out.write(data.data(), data.size());
			return out;
		}
		static int last(TOut&)
		{
			// Last emitted char, if available
			return std::char_traits<char>::eof();
		}
	};
	template <> struct traits<textbuf>
	{
		static textbuf& append(textbuf& out, std::string_view data)
		{
			out.append(data);
			return out;
		}
		static int last(textbuf& out)
		{
			return !out.empty() ? out.back() : std::char_traits<char>::eof();
		}
	};
	template <> struct traits<bytebuf>
	{
		static bytebuf& write(bytebuf& out, std::span<std::byte const> data, int64_t ofs = -1)
		{
			ofs = ofs != -1 ? ofs : ssize(out);
			out.resize(std::max<size_t>(out.size(), static_cast<size_t>(ofs + data.size())));
			std::memcpy(out.data() + ofs, data.data(), data.size());
			return out;
		}
		static int64_t tellp(bytebuf& out)
		{
			return out.size();
		}
	};

	// Memcpyable arrays
	template <typename T> concept PrimitiveSpanType =
		std::is_trivially_copyable_v<T> &&
		std::is_same_v<T, std::string_view> == false &&
		std::is_same_v<T, std::string> == false &&
		std::is_same_v<T, string32> == false &&
		requires(T t)
		{
			{ t.data() } -> std::same_as<typename T::value_type*>;
			{ t.size() } -> std::same_as<std::size_t>;
		};

	// The section header
	struct SectionHeader
	{
		// The hash of the keyword (4-bytes)
		EKeyword m_keyword;

		// The length of the section in bytes (excluding the header size)
		int m_size;
	};
	static_assert(sizeof(SectionHeader) == 8);
}
