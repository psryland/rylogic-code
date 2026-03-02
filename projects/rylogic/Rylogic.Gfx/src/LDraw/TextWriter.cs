using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Reflection;
using System.Text;
using Rylogic.Gfx;
using Rylogic.Maths;

namespace Rylogic.LDraw
{
	public class TextWriter : IWriter
	{
		private readonly StringBuilder m_sb = new();

		/// <inheritdoc/>
		public void Write(EKeyword keyword, params object[] args)
		{
			Append(keyword);

			// Optional name/colour
			var i = 0;
			if (i != args.Length && args[i] is Serialiser.Name name)
			{
				Append(name.m_name);
				++i;
			}
			if (i != args.Length && args[i] is Serialiser.Colour colour)
			{
				Append(colour.m_colour);
				++i;
			}

			// Write section content
			Append("{");
			Append(args.Skip(i));
			Append("}");
		}

		/// <inheritdoc/>
		public void Append(params object[] args)
		{
			Append(args.AsEnumerable());
		}
		private void Append(IEnumerable<object> args)
		{
			foreach (var arg in args)
			{
				if (arg == null)
					continue;
				else if (typeof(TextWriter).GetMethod(nameof(Append), BindingFlags.Instance | BindingFlags.NonPublic, null, [arg.GetType()], []) is MethodInfo meth)
					meth.Invoke(this, [arg]);
				else if (arg is Action action)
					action();
				else
					throw new Exception($"Unsupported type ({arg.GetType().Name}) for Append");
			}
		}
		private void Append(string s)
		{
			if (s.Length == 0)
				return;
			if (m_sb.Length != 0 && s[s.Length - 1] != '}' && s[s.Length - 1] != ')' && s[s.Length - 1] != ' ' && m_sb[m_sb.Length - 1] != '{')
				m_sb.Append(' ');

			m_sb.Append(s);
		}
		private void Append(EKeyword keyword)
		{
			Append($"*{keyword}");
		}
		private void Append(bool b)
		{
			Append(b ? "true" : "false");
		}
		private void Append(int i)
		{
			Append(i.ToString());
		}
		private void Append(long i)
		{
			Append(i.ToString());
		}
		private void Append(float f)
		{
			Append(f.ToString());
		}
		private void Append(double f)
		{
			Append(f.ToString());
		}
		private void Append(uint u)
		{
			Append($"{u:X8}");
		}
		private void Append(Colour32 c)
		{
			Append(c.ARGB);
		}
		private void Append(v2 v)
		{
			Append(v.ToString());
		}
		private void Append(v3 v)
		{
			Append(v.ToString());
		}
		private void Append(v4 v)
		{
			Append(v.ToString());
		}
		private void Append(m4x4 m)
		{
			Append(m.ToString4x4(delim:""));
		}
		private void Append(View3d.EAddrMode addr)
		{
			switch (addr)
			{
				case View3d.EAddrMode.Wrap:
					Append("Wrap");
					break;
				case View3d.EAddrMode.Mirror:
					Append("Mirror");
					break;
				case View3d.EAddrMode.Clamp:
					Append("Clamp");
					break;
				case View3d.EAddrMode.Border:
					Append("Border");
					break;
				case View3d.EAddrMode.MirrorOnce:
					Append("MirrorOnce");
					break;
				default:
					throw new Exception($"Unknown texture addressing mode ({addr})");
			}
		}
		private void Append(View3d.EFilter filter)
		{
			switch (filter)
			{
				case View3d.EFilter.Point:
					Append("Point");
					break;
				case View3d.EFilter.PointPointLinear:
					Append("PointPointLinear");
					break;
				case View3d.EFilter.PointLinearPoint:
					Append("PointLinearPoint");
					break;
				case View3d.EFilter.PointLinearLinear:
					Append("PointLinearLinear");
					break;
				case View3d.EFilter.LinearPointPoint:
					Append("LinearPointPoint");
					break;
				case View3d.EFilter.LinearPointLinear:
					Append("LinearPointLinear");
					break;
				case View3d.EFilter.LinearLinearPoint:
					Append("LinearLinearPoint");
					break;
				case View3d.EFilter.Linear:
					Append("Linear");
					break;
				case View3d.EFilter.Anisotropic:
					Append("Anisotropic");
					break;
				default:
					throw new Exception($"Unknown texture addressing mode ({filter})");
			}
		}
		private void Append(EPointStyle style)
		{
			switch (style)
			{
				case EPointStyle.Square: Append("Square"); break;
				case EPointStyle.Circle: Append("Circle"); break;
				case EPointStyle.Triangle: Append("Triangle"); break;
				case EPointStyle.Star: Append("Star"); break;
				case EPointStyle.Annulus: Append("Annulus"); break;
				default: throw new Exception($"Unknown point style ({style})");
			}
		}
		private void Append(ELineStyle style)
		{
			switch (style)
			{
				case ELineStyle.LineSegments: Append("LineSegments"); break;
				case ELineStyle.LineStrip: Append("LineStrip"); break;
				case ELineStyle.Direction: Append("Direction"); break;
				case ELineStyle.BezierSpline: Append("BezierSpline"); break;
				case ELineStyle.HermiteSpline: Append("HermiteSpline"); break;
				case ELineStyle.BSplineSpline: Append("BSplineSpline"); break;
				case ELineStyle.CatmullRom: Append("CatmullRom"); break;
				default: throw new Exception($"Unknown line style ({style})");
			}
		}
		private void Append(EArrowType type)
		{
			switch (type)
			{
				case EArrowType.Line: Append("Line"); break;
				case EArrowType.Fwd:  Append("Fwd"); break;
				case EArrowType.Back: Append("Back"); break;
				case EArrowType.FwdBack: Append("FwdBack"); break;
				default: throw new Exception("Unknown arrow type");
			}
		}
		private void Append(Serialiser.VariableInt vint)
		{
			Append(vint.m_value);
		}
		private void Append(Serialiser.StringWithLength str)
		{
			Append(str.m_value);
		}
		private void Append(Serialiser.Name n)
		{
			if (n.m_name.Length == 0) return;
			Write(EKeyword.Name, n.m_name);
		}
		private void Append(Serialiser.Colour c)
		{
			if (c.m_colour == 0xFFFFFFFF) return;
			Write(c.m_kw, c.m_colour);
		}
		private void Append(Serialiser.Size s)
		{
			if (s.m_size == 0) return;
			Write(EKeyword.Size, s.m_size);
		}
		private void Append(Serialiser.Size2 s)
		{
			if (s.m_size == v2.Zero) return;
			Write(EKeyword.Size, s.m_size);
		}
		private void Append(Serialiser.Scale s)
		{
			if (s.m_scale == 1f) return;
			Write(EKeyword.Scale, s.m_scale);
		}
		private void Append(Serialiser.Scale2 s)
		{
			if (s.m_scale == v2.One) return;
			Write(EKeyword.Scale, s.m_scale);
		}
		private void Append(Serialiser.Scale3 s)
		{
			if (s.m_scale == v3.One) return;
			Write(EKeyword.Scale, s.m_scale);
		}
		private void Append(Serialiser.PerItemColour c)
		{
			if (!c) return;
			Write(EKeyword.PerItemColour, c.m_per_item_colour);
		}
		private void Append(Serialiser.Width w)
		{
			if (!w) return;
			Write(EKeyword.Width, w.m_width);
		}
		private void Append(Serialiser.Depth d)
		{
			if (!d) return;
			Write(EKeyword.Depth, d.m_depth);
		}
		private void Append(Serialiser.Hidden h)
		{
			if (!h) return;
			Write(EKeyword.Hidden, h.m_hide);
		}
		private void Append(Serialiser.ZTest z)
		{
			if (!z) return;
			Write(EKeyword.NoZTest, !z.m_ztest);
		}
		private void Append(Serialiser.ZWrite z)
		{
			if (!z) return;
			Write(EKeyword.NoZWrite, !z.m_zwrite);
		}
		private void Append(Serialiser.Wireframe w)
		{
			if (!w) return;
			Write(EKeyword.Wireframe, w.m_wire);
		}
		private void Append(Serialiser.Alpha a)
		{
			if (!a) return;
			Write(EKeyword.Alpha, a.m_has_alpha);
		}
		private void Append(Serialiser.Solid s)
		{
			if (!s) return;
			Write(EKeyword.Solid, s.m_solid);
		}
		private void Append(Serialiser.Smooth s)
		{
			if (!s) return;
			Write(EKeyword.Smooth, s.m_smooth);
		}
		private void Append(Serialiser.Dashed d)
		{
			if (!d) return;
			Write(EKeyword.Dashed, d.m_dash);
		}
		private void Append(Serialiser.DataPoints dp)
		{
			if (!dp) return;
			Write(EKeyword.DataPoints, () =>
			{
				Write(EKeyword.Size, dp.m_size);
				if (dp.m_style != EPointStyle.Square)
					Write(EKeyword.Style, dp.m_style);
				if (dp.m_colour != Colour32.White)
					Write(EKeyword.Colour, dp.m_colour);
			});
		}
		private void Append(Serialiser.LeftHanded lh)
		{
			if (!lh) return;
			Write(EKeyword.LeftHanded, lh.m_lh);
		}
		private void Append(Serialiser.AxisId a)
		{
			if (!a) return;
			Write(EKeyword.AxisId, (int)a.m_axis.Id);
		}
		private void Append(Serialiser.PointStyle p)
		{
			if (!p) return;
			Write(EKeyword.Style, p.m_style);
		}
		private void Append(Serialiser.LineStyle l)
		{
			if (!l) return;
			Write(EKeyword.Style, l.m_style);
		}
		private void Append(Serialiser.ArrowHeads a)
		{
			if (!a) return;
			Write(EKeyword.Arrow, a.m_type, a.m_size);
		}		
		private void Append(Serialiser.Pos p)
		{
			if (p.m_pos == v4.Origin)
				return;
			Write(EKeyword.O2W, () =>
			{
				Write(EKeyword.Pos, p.m_pos.xyz);
			});
		}
		private void Append(Serialiser.O2W o2w)
		{
			if (o2w.m_mat == m4x4.Identity)
				return;

			if (o2w.m_mat.rot == m3x4.Identity && o2w.m_mat.pos.w == 1)
			{
				Write(EKeyword.O2W, () =>
				{
					Write(EKeyword.Pos, o2w.m_mat.pos.xyz);
				});
				return;
			}

			Write(EKeyword.O2W, () =>
			{
				if (!Math_.IsAffine(o2w.m_mat))
					Write(EKeyword.NonAffine);
				Write(EKeyword.M4x4, o2w.m_mat);
			});
		}
		private void Append(Serialiser.Facets f)
		{
			if (!f) return;
			if (f.m_wedges != 0)
				Write(EKeyword.Facets, f.m_facets, f.m_wedges);
			else
				Write(EKeyword.Facets, f.m_facets);
		}
		private void Append(Serialiser.CornerRadius cr)
		{
			if (!cr) return;
			Write(EKeyword.CornerRadius, cr.m_radius);
		}
		private void Append(Serialiser.Closed c)
		{
			if (!c) return;
			Write(EKeyword.Closed, c.m_closed);
		}
		private void Append(Serialiser.GenerateNormals gn)
		{
			if (!gn) return;
			Write(EKeyword.GenerateNormals, gn.m_smoothing_angle);
		}
		private void Append(Serialiser.Reflectivity r)
		{
			if (!r) return;
			Write(EKeyword.Reflectivity, r.m_amount);
		}
		private void Append(Serialiser.ScreenSpace ss)
		{
			if (!ss) return;
			Write(EKeyword.ScreenSpace);
		}
		private void Append(Serialiser.Anchor a)
		{
			if (!a) return;
			Write(EKeyword.Anchor, a.m_anchor);
		}
		private void Append(Serialiser.Padding p)
		{
			if (!p) return;
			Write(EKeyword.Padding, p.m_left, p.m_top, p.m_right, p.m_bottom);
		}

		/// <inheritdoc/>
		public override string ToString() => m_sb.ToString();

		/// <summary>Return the text as a utf8 memory stream</summary>
		public MemoryStream ToText()
		{
			return new MemoryStream(Encoding.UTF8.GetBytes(m_sb.ToString()));
		}
	}
}
