//***********************************************
// LDraw
//  Copyright (c) Rylogic Ltd 2008
//***********************************************

using System.Text;
using Rylogic.Gfx;
using Rylogic.Maths;

namespace Rylogic.LDraw
{
	/// <summary>LDraw script writer interface</summary>
	public interface IWriter
	{
		/// <summary>Write a section to the script output</summary>
		void Write(EKeyword keyword, params object[] args);

		/// <summary>Append objects to the script</summary>
		void Append(params object[] items);
	}
}
namespace Rylogic.LDraw.Serialiser
{
	public class Name(string? name = null)
	{
		public string m_name = Sanitise(name ?? string.Empty);
		public static implicit operator Name(string? name) => new(name);
		public static string Sanitise(string name)
		{
			var sb = new StringBuilder();
			foreach (var ch in name) sb.Append(char.IsLetter(ch) || (sb.Length != 0 && char.IsDigit(ch)) ? ch : '_');
			return sb.ToString();
		}
	}
	public class Colour(uint? colour = null, EKeyword? kw = null)
	{
		public uint m_colour = colour ?? 0xFFFFFFFF;
		public EKeyword m_kw = kw ?? EKeyword.Colour;
		public static implicit operator Colour(uint colour) => new(colour);
		public static implicit operator Colour(uint? colour) => new(colour);
		public static implicit operator Colour(Colour32 colour) => new(colour);
		public static implicit operator Colour(Colour32? colour) => new(colour);
	}
	public class Size(float? size = null)
	{
		public float m_size = size ?? 0f;
		public static implicit operator Size(float? size) => new(size);
	}
	public class Size2(v2? size = null)
	{
		public v2 m_size = size ?? v2.Zero;
		public static implicit operator Size2(v2? size) => new(size);
	}
	public class Scale(float? scale = null)
	{
		public float m_scale = scale ?? 1f;
		public static implicit operator Scale(float? scale) => new(scale);
	}
	public class Scale2(v2? scale = null)
	{
		public v2 m_scale = scale ?? v2.One;
		public static implicit operator Scale2(v2? scale) => new(scale);
	}
	public class Scale3(v3? scale = null)
	{
		public v3 m_scale = scale ?? v3.One;
		public static implicit operator Scale3(v3? scale) => new(scale);
	}
	public class PerItemColour
	{
		public bool m_per_item_colour = false;
		public bool m_is_default = true;
		public PerItemColour() { }
		public PerItemColour(bool pic)
		{
			m_per_item_colour = pic;
			m_is_default = false;
		}
		public static implicit operator bool(PerItemColour pic) => !pic.m_is_default || pic.m_per_item_colour;
	}
	public class Width
	{
		public float m_width = 0f;
		public bool m_is_default = true;
		public Width() { }
		public Width(float width)
		{
			m_width = width;
			m_is_default = false;
		}
		public static implicit operator bool(Width w) => !w.m_is_default || w.m_width != 0f;
	}
	public class Depth
	{
		public bool m_depth = false;
		public bool m_is_default = true;
		public Depth() { }
		public Depth(bool depth)
		{
			m_depth = depth;
			m_is_default = false;
		}
		public static implicit operator bool(Depth d) => !d.m_is_default || d.m_depth;
	}
	public class Hidden
	{
		public bool m_hide = false;
		public bool m_is_default = true;
		public Hidden() { }
		public Hidden(bool hide)
		{
			m_hide = hide;
			m_is_default = false;
		}
		public static implicit operator bool(Hidden h) => !h.m_is_default || h.m_hide;
	}
	public class ZTest
	{
		public bool m_ztest = false;
		public bool m_is_default = true;
		public ZTest() { }
		public ZTest(bool ztest)
		{
			m_ztest = ztest;
			m_is_default = false;
		}
		public static implicit operator bool(ZTest s) => !s.m_is_default || s.m_ztest;
	}
	public class ZWrite
	{
		public bool m_zwrite = false;
		public bool m_is_default = true;
		public ZWrite() { }
		public ZWrite(bool zwrite)
		{
			m_zwrite = zwrite;
			m_is_default = false;
		}
		public static implicit operator bool(ZWrite s) => !s.m_is_default || s.m_zwrite;
	}
	public class Wireframe
	{
		public bool m_wire = false;
		public bool m_is_default = true;
		public Wireframe() { }
		public Wireframe(bool wire)
		{
			m_wire = wire;
			m_is_default = false;
		}
		public static implicit operator bool(Wireframe w) => !w.m_is_default || w.m_wire;
	}
	public class Alpha
	{
		public bool m_has_alpha = false;
		public bool m_is_default = true;
		public Alpha() { }
		public Alpha(bool alpha)
		{
			m_has_alpha = alpha;
			m_is_default = false;
		}
		public static implicit operator bool(Alpha a) => !a.m_is_default || a.m_has_alpha;
	}
	public class Solid
	{
		public bool m_solid = false;
		public bool m_is_default = true;
		public Solid() { }
		public Solid(bool solid)
		{
			m_solid = solid;
			m_is_default = false;
		}
		public static implicit operator bool(Solid s) => !s.m_is_default || s.m_solid;
	}
	public class Smooth
	{
		public bool m_smooth = false;
		public bool m_is_default = true;
		public Smooth() { }
		public Smooth(bool smooth)
		{
			m_smooth = smooth;
			m_is_default = false;
		}
		public static implicit operator bool(Smooth s) => !s.m_is_default || s.m_smooth;
	}
	public class Dashed
	{
		public v2 m_dash = v2.Zero;
		public bool m_is_default = true;
		public Dashed() { }
		public Dashed(v2 dash)
		{
			m_dash = dash;
			m_is_default = false;
		}
		public static implicit operator bool(Dashed d) => !d.m_is_default || d.m_dash != v2.Zero;
	}
	public class DataPoints
	{
		public v2 m_size = v2.Zero;
		public Colour32 m_colour = Colour32.White;
		public EPointStyle m_style = EPointStyle.Square;
		public bool m_is_default = true;
		public DataPoints() { }
		public DataPoints(v2 size, Colour32? colour, EPointStyle? style)
		{
			m_size = size;
			m_colour = colour ?? Colour32.White;
			m_style = style ?? EPointStyle.Square;
			m_is_default = false;
		}
		public static implicit operator bool(DataPoints dp) => !dp.m_is_default || dp.m_size != v2.Zero || dp.m_colour != Colour32.White || dp.m_style != EPointStyle.Square;
	}
	public class LeftHanded
	{
		public bool m_lh = false;
		public bool m_is_default = true;
		public LeftHanded() { }
		public LeftHanded(bool lh)
		{
			m_lh = lh;
			m_is_default = false;
		}
		public static implicit operator bool(LeftHanded lh) => !lh.m_is_default || lh.m_lh;
	}
	public class AxisId
	{
		public Maths.AxisId m_axis = EAxisId.None;
		public bool m_is_default = true;
		public AxisId() { }
		public AxisId(Maths.AxisId axis)
		{
			m_axis = axis;
			m_is_default = false;
		}
		public static implicit operator bool(AxisId ax) => !ax.m_is_default || ax.m_axis != EAxisId.None;
	}
	public class PointStyle
	{
		public EPointStyle m_style = EPointStyle.Square;
		public bool m_is_default = true;
		public PointStyle() { }
		public PointStyle(EPointStyle style)
		{
			m_style = style;
			m_is_default = false;
		}
		public static implicit operator bool(PointStyle ps) => !ps.m_is_default || ps.m_style != EPointStyle.Square;
	}
	public class LineStyle
	{
		public ELineStyle m_style = ELineStyle.LineSegments;
		public bool m_is_default = true;
		public LineStyle() { }
		public LineStyle(ELineStyle style)
		{
			m_style = style;
			m_is_default = false;
		}
		public static implicit operator bool(LineStyle ls) => !ls.m_is_default || ls.m_style != ELineStyle.LineSegments;
	}
	public class ArrowHeads
	{
		public EArrowType m_type = EArrowType.Line;
		public float m_size = 10f;
		public bool m_is_default = true;
		public ArrowHeads() { }
		public ArrowHeads(EArrowType type, float size = 10f)
		{
			m_type = EArrowType.Line;
			m_size = size;
			m_is_default = false;
		}
		public static implicit operator bool(ArrowHeads ah) => !ah.m_is_default || ah.m_type != EArrowType.Line || ah.m_size != 10f;
	}
	public class Pos(v4? pos = null)
	{
		public v4 m_pos = pos ?? v4.Origin;
		public Pos(m4x4 mat) : this(mat.pos) { }
		public static implicit operator Pos(v4? pos) => new(pos);
	}
	public class O2W(m4x4? o2w = null)
	{
		public m4x4 m_mat = o2w ?? m4x4.Identity;
		public O2W(v4 pos) : this(m4x4.Translation(pos)) { }
		public static implicit operator O2W(m4x4? o2w) => new(o2w);
	}
	public class VariableInt(int? value = null)
	{
		public int m_value = (value ?? 0) & 0x3FFFFFFF;
	}
	public class StringWithLength(string? str = null)
	{
		public string m_value = str ?? string.Empty;
	}
	public class Facets
	{
		public int m_facets = 0;
		public int m_wedges = 0;
		public bool m_is_default = true;
		public Facets() { }
		public Facets(int facets) { m_facets = facets; m_is_default = false; }
		public Facets(int layers, int wedges) { m_facets = layers; m_wedges = wedges; m_is_default = false; }
		public static implicit operator bool(Facets f) => !f.m_is_default || f.m_facets != 0;
	}
	public class CornerRadius
	{
		public float m_radius = 0f;
		public bool m_is_default = true;
		public CornerRadius() { }
		public CornerRadius(float radius) { m_radius = radius; m_is_default = false; }
		public static implicit operator bool(CornerRadius cr) => !cr.m_is_default || cr.m_radius != 0f;
	}
	public class Closed
	{
		public bool m_closed = false;
		public bool m_is_default = true;
		public Closed() { }
		public Closed(bool closed) { m_closed = closed; m_is_default = false; }
		public static implicit operator bool(Closed c) => !c.m_is_default || c.m_closed;
	}
	public class GenerateNormals
	{
		public float m_smoothing_angle = 0f;
		public bool m_is_default = true;
		public GenerateNormals() { }
		public GenerateNormals(float angle) { m_smoothing_angle = angle; m_is_default = false; }
		public static implicit operator bool(GenerateNormals gn) => !gn.m_is_default || gn.m_smoothing_angle != 0f;
	}
	public class Reflectivity
	{
		public float m_amount = 0f;
		public bool m_is_default = true;
		public Reflectivity() { }
		public Reflectivity(float amount) { m_amount = amount; m_is_default = false; }
		public static implicit operator bool(Reflectivity r) => !r.m_is_default || r.m_amount != 0f;
	}
	public class ScreenSpace
	{
		public bool m_screen_space = false;
		public bool m_is_default = true;
		public ScreenSpace() { }
		public ScreenSpace(bool ss) { m_screen_space = ss; m_is_default = false; }
		public static implicit operator bool(ScreenSpace ss) => !ss.m_is_default || ss.m_screen_space;
	}
	public class Anchor
	{
		public v2 m_anchor = v2.Zero;
		public bool m_is_default = true;
		public Anchor() { }
		public Anchor(v2 anchor) { m_anchor = anchor; m_is_default = false; }
		public static implicit operator bool(Anchor a) => !a.m_is_default || a.m_anchor != v2.Zero;
	}
	public class Padding
	{
		public float m_left = 0f, m_top = 0f, m_right = 0f, m_bottom = 0f;
		public bool m_is_default = true;
		public Padding() { }
		public Padding(float left, float top, float right, float bottom) { m_left = left; m_top = top; m_right = right; m_bottom = bottom; m_is_default = false; }
		public static implicit operator bool(Padding p) => !p.m_is_default;
	}
}
