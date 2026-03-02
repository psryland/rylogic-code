//***********************************************
// LineDrawer helper
//  Copyright (c) Rylogic Ltd 2008
//***********************************************

using System;
using System.Collections;
using System.Collections.Generic;
using System.Diagnostics;
using System.Drawing;
using System.IO;
using System.Linq;
using System.Reflection;
using System.Text;
using Rylogic.Common;
using Rylogic.Extn;
using Rylogic.Gfx;
using Rylogic.Maths;
using Rylogic.Utility;

namespace Rylogic.LDraw
{
	[Flags] public enum ESaveFlags
	{
		None = 0,
		Binary = 1 << 0,
		Pretty = 1 << 1,
		Append = 1 << 2,
		NoThrowOnFailure = 1 << 8,
	}

	public class Builder
	{
		/// <summary>Access the objects already in the builder</summary>
		public IReadOnlyList<Builder> Objects => m_objects;
		private List<Builder> m_objects = [];

		/// <summary>Reset the builder</summary>
		public Builder Clear(int count = -1)
		{
			if (count >= 0 && count < m_objects.Count)
				m_objects.RemoveRange(count, m_objects.Count - count);
			else
				m_objects.Clear();
	
			return this;
		}

		/// <summary>Create child object</summary>
		public LdrGroup Group(Serialiser.Name? name = null, Serialiser.Colour? colour = null)
		{
			var child = new LdrGroup();
			m_objects.Add(child);
			return child.name(name ?? new()).colour(colour ?? new());
		}
		public LdrInstance Instance(Serialiser.Name? name = null, Serialiser.Colour? colour = null)
		{
			var child = new LdrInstance();
			m_objects.Add(child);
			return child.name(name ?? new()).colour(colour ?? new());
		}
		public LdrText Text(Serialiser.Name? name = null, Serialiser.Colour? colour = null)
		{
			var child = new LdrText();
			m_objects.Add(child);
			return child.name(name ?? new()).colour(colour ?? new());
		}
		public LdrLightSource LightSource(Serialiser.Name? name = null, Serialiser.Colour? colour = null)
		{
			var child = new LdrLightSource();
			m_objects.Add(child);
			return child.name(name ?? new()).colour(colour ?? new());
		}
		public LdrPoint Point(Serialiser.Name? name = null, Serialiser.Colour? colour = null)
		{
			var child = new LdrPoint();
			m_objects.Add(child);
			return child.name(name ?? new()).colour(colour ?? new());
		}
		public LdrLine Line(Serialiser.Name? name = null, Serialiser.Colour? colour = null)
		{
			var child = new LdrLine();
			m_objects.Add(child);
			return child.name(name ?? new()).colour(colour ?? new());
		}
		public LdrLineBox LineBox(Serialiser.Name? name = null, Serialiser.Colour? colour = null)
		{
			var child = new LdrLineBox();
			m_objects.Add(child);
			return child.name(name ?? new()).colour(colour ?? new());
		}
		public LdrGrid Grid(Serialiser.Name? name = null, Serialiser.Colour? colour = null)
		{
			var child = new LdrGrid();
			m_objects.Add(child);
			return child.name(name ?? new()).colour(colour ?? new());
		}
		public LdrCoordFrame CoordFrame(Serialiser.Name? name = null, Serialiser.Colour? colour = null)
		{
			var child = new LdrCoordFrame();
			m_objects.Add(child);
			return child.name(name ?? new()).colour(colour ?? new());
		}
		public LdrCircle Circle(Serialiser.Name? name = null, Serialiser.Colour? colour = null)
		{
			var child = new LdrCircle();
			m_objects.Add(child);
			return child.name(name ?? new()).colour(colour ?? new());
		}
		public LdrPie Pie(Serialiser.Name? name = null, Serialiser.Colour? colour = null)
		{
			var child = new LdrPie();
			m_objects.Add(child);
			return child.name(name ?? new()).colour(colour ?? new());
		}
		public LdrRect Rect(Serialiser.Name? name = null, Serialiser.Colour? colour = null)
		{
			var child = new LdrRect();
			m_objects.Add(child);
			return child.name(name ?? new()).colour(colour ?? new());
		}
		public LdrPolygon Polygon(Serialiser.Name? name = null, Serialiser.Colour? colour = null)
		{
			var child = new LdrPolygon();
			m_objects.Add(child);
			return child.name(name ?? new()).colour(colour ?? new());
		}
		public LdrTriangle Triangle(Serialiser.Name? name = null, Serialiser.Colour? colour = null)
		{
			var child = new LdrTriangle();
			m_objects.Add(child);
			return child.name(name ?? new()).colour(colour ?? new());
		}
		public LdrQuad Quad(Serialiser.Name? name = null, Serialiser.Colour? colour = null)
		{
			var child = new LdrQuad();
			m_objects.Add(child);
			return child.name(name ?? new()).colour(colour ?? new());
		}
		public LdrPlane Plane(Serialiser.Name? name = null, Serialiser.Colour? colour = null)
		{
			var child = new LdrPlane();
			m_objects.Add(child);
			return child.name(name ?? new()).colour(colour ?? new());
		}
		public LdrRibbon Ribbon(Serialiser.Name? name = null, Serialiser.Colour? colour = null)
		{
			var child = new LdrRibbon();
			m_objects.Add(child);
			return child.name(name ?? new()).colour(colour ?? new());
		}
		public LdrBox Box(Serialiser.Name? name = null, Serialiser.Colour? colour = null)
		{
			var child = new LdrBox();
			m_objects.Add(child);
			return child.name(name ?? new()).colour(colour ?? new());
		}
		public LdrBoxList BoxList(Serialiser.Name? name = null, Serialiser.Colour? colour = null)
		{
			var child = new LdrBoxList();
			m_objects.Add(child);
			return child.name(name ?? new()).colour(colour ?? new());
		}
		public LdrFrustum Frustum(Serialiser.Name? name = null, Serialiser.Colour? colour = null)
		{
			var child = new LdrFrustum();
			m_objects.Add(child);
			return child.name(name ?? new()).colour(colour ?? new());
		}
		public LdrSphere Sphere(Serialiser.Name? name = null, Serialiser.Colour? colour = null)
		{
			var child = new LdrSphere();
			m_objects.Add(child);
			return child.name(name ?? new()).colour(colour ?? new());
		}
		public LdrCylinder Cylinder(Serialiser.Name? name = null, Serialiser.Colour? colour = null)
		{
			var child = new LdrCylinder();
			m_objects.Add(child);
			return child.name(name ?? new()).colour(colour ?? new());
		}
		public LdrCone Cone(Serialiser.Name? name = null, Serialiser.Colour? colour = null)
		{
			var child = new LdrCone();
			m_objects.Add(child);
			return child.name(name ?? new()).colour(colour ?? new());
		}
		public LdrTube Tube(Serialiser.Name? name = null, Serialiser.Colour? colour = null)
		{
			var child = new LdrTube();
			m_objects.Add(child);
			return child.name(name ?? new()).colour(colour ?? new());
		}
		public LdrMesh Mesh(Serialiser.Name? name = null, Serialiser.Colour? colour = null)
		{
			var child = new LdrMesh();
			m_objects.Add(child);
			return child.name(name ?? new()).colour(colour ?? new());
		}
		public LdrConvexHull ConvexHull(Serialiser.Name? name = null, Serialiser.Colour? colour = null)
		{
			var child = new LdrConvexHull();
			m_objects.Add(child);
			return child.name(name ?? new()).colour(colour ?? new());
		}
		public LdrChart Chart(Serialiser.Name? name = null, Serialiser.Colour? colour = null)
		{
			var child = new LdrChart();
			m_objects.Add(child);
			return child.name(name ?? new()).colour(colour ?? new());
		}
		public LdrModel Model(Serialiser.Name? name = null, Serialiser.Colour? colour = null)
		{
			var child = new LdrModel();
			m_objects.Add(child);
			return child.name(name ?? new()).colour(colour ?? new());
		}
		public LdrEquation Equation(Serialiser.Name? name = null, Serialiser.Colour? colour = null)
		{
			var child = new LdrEquation();
			m_objects.Add(child);
			return child.name(name ?? new()).colour(colour ?? new());
		}
		public LdrCommands Command()
		{
			var child = new LdrCommands();
			m_objects.Add(child);
			return child;
		}

		/// <summary>Switch data stream modes</summary>
		public LdrBinaryStream BinaryStream()
		{
			var child = new LdrBinaryStream();
			m_objects.Add(child);
			return child;
		}
		public LdrTextStream TextStream()
		{
			var child = new LdrTextStream();
			m_objects.Add(child);
			return child;
		}

		/// <summary>Serialise to ldraw script</summary>
		public override string ToString()
		{
			var res = new TextWriter();
			WriteTo(res);
			return res.ToString();
		}
		public string ToString(bool pretty)
		{
			var res = ToString();
			if (pretty) FormatScript(res);
			return res;
		}

		/// <summary>Serialise to ldraw text</summary>
		public MemoryStream ToText()
		{
			var res = new TextWriter();
			WriteTo(res);
			return res.ToText();
		}

		/// <summary>Serialise to ldraw binary</summary>
		public MemoryStream ToBinary()
		{
			var res = new BinaryWriter();
			WriteTo(res);
			return res.ToBinary();
		}

		/// <summary>Serialise to 'res'</summary>
		public virtual void WriteTo(IWriter res)
		{
			foreach (var obj in m_objects)
				obj.WriteTo(res);
		}

		/// <summary>Write to a file</summary>
		public void Save(string filepath, ESaveFlags flags = ESaveFlags.None)
		{
			try
			{
				var rng = new Random();
				var tmp_path = Path_.CombinePath(Path.GetDirectoryName(filepath) ?? string.Empty, $"{(uint)rng.Next()}.tmp");
				if (!Path_.IsValidFilepath(tmp_path, true))
					throw new Exception("Failed to create temporary file path for LDraw save");

				var binary = flags.HasFlag(ESaveFlags.Binary);
				var append = flags.HasFlag(ESaveFlags.Append);
				var pretty = flags.HasFlag(ESaveFlags.Pretty);

				if (Path.GetDirectoryName(tmp_path) is string directory)
					Directory.CreateDirectory(directory);

				{
					using var file = File.Open(tmp_path, append ? FileMode.Append : FileMode.Create);
					var mem = binary ? ToBinary() : ToText();
					if (!binary && pretty) mem = FormatScript(mem);
					mem.CopyTo(file);
				}

				var outpath = filepath;
				if (Path.GetExtension(outpath) == string.Empty)
					outpath = Path.ChangeExtension(outpath, binary ? ".bdr" : ".ldr");

				if (File.Exists(outpath))
					File.Replace(tmp_path, outpath, null);
				else
					File.Move(tmp_path, outpath);
			}
			catch (Exception ex)
			{
				if (!flags.HasFlag(ESaveFlags.NoThrowOnFailure)) throw;
				Debug.WriteLine($"Failed to save LDraw file to '{filepath}': {ex.Message}");
			}
		}

		// Pretty format Ldraw script
		public static string FormatScript(string str)
		{
			var res = new StringBuilder(str.Length);
			var last = '\0';
			var indent = 0;
			foreach (var c in str)
			{
				if (c == '{')
				{
					++indent;
					res.Append(c);
					res.Append('\n',1).Append('\t', indent);
				}
				else if (c == '}')
				{
					--indent;
					res.Append('\n',1).Append('\t', indent);
					res.Append(c);
				}
				else
				{
					if (last == '}')
						res.Append('\n',1).Append('\t', indent);

					res.Append(c);
				}
				last = c;
			}
			return res.ToString();
		}

		// Pretty format Ldraw script from UTF8 MemoryStream
		public static MemoryStream FormatScript(MemoryStream utf8)
		{
			utf8.Position = 0;
			var result = new MemoryStream();
			using var reader = new StreamReader(utf8, Encoding.UTF8, detectEncodingFromByteOrderMarks: false, bufferSize: 1024, leaveOpen: true);
			using var writer = new StreamWriter(result, Encoding.UTF8, bufferSize: 1024, leaveOpen: true);
			writer.Write(FormatScript(reader.ReadToEnd()));
			writer.Flush();
			result.Position = 0;
			return result;
		}
	}
	public class LdrBase<TDerived> : Builder where TDerived : LdrBase<TDerived>
	{
		protected Serialiser.Name m_name = new();
		protected Serialiser.Colour m_colour = new();
		protected Serialiser.Colour m_group_colour = new();
		protected Serialiser.O2W m_o2w = new();
		protected Serialiser.Wireframe m_wire = new();
		protected Serialiser.AxisId m_axis_id = new();
		protected Serialiser.Solid m_solid = new();
		protected Serialiser.Hidden m_hidden = new();
		protected Serialiser.ZTest m_ztest = new();
		protected Serialiser.ZWrite m_zwrite = new();
		protected Serialiser.Reflectivity m_reflectivity = new();
		protected Serialiser.ScreenSpace m_screen_space = new();
		private LdrTransform? m_bake = null;
		private LdrFont? m_font = null;
		private LdrRootAnimation? m_root_anim = null;

		/// <summary>Object name</summary>
		public TDerived name(Serialiser.Name name)
		{
			m_name = name;
			return (TDerived)this;
		}

		/// <summary>Object colour</summary>
		public TDerived colour(Serialiser.Colour colour)
		{
			m_colour = colour;
			return (TDerived)this;
		}

		/// <summary>Object colour mask</summary>
		public TDerived group_colour(Serialiser.Colour colour)
		{
			m_group_colour = colour;
			m_group_colour.m_kw = EKeyword.GroupColour;
			return (TDerived)this;
		}

		/// <summary>Object to world transform</summary>
		public TDerived o2w(m4x4 o2w)
		{
			m_o2w.m_mat = o2w * m_o2w.m_mat;
			return (TDerived)this;
		}
		public TDerived o2w(m3x4 rot, v4 pos)
		{
			return o2w(new m4x4(rot, pos));
		}
		public TDerived ori(v4 dir, AxisId axis)
		{
			return ori(m3x4.Rotation(axis.Axis, dir));
		}
		public TDerived ori(m3x4 rot)
		{
			return o2w(rot, v4.Origin);
		}
		public TDerived ori(Quat q)
		{
			return o2w(m4x4.Transform(q, v4.Origin));
		}
		public TDerived pos(float x, float y, float z)
		{
			return o2w(m4x4.Translation(x, y, z));
		}
		public TDerived pos(v4 pos)
		{
			return o2w(m4x4.Translation(pos));
		}
		public TDerived pos(v3 pos)
		{
			return this.pos(pos.w1);
		}
		public TDerived scale(float s)
		{
			return scale(s, s, s);
		}
		public TDerived scale(float sx, float sy, float sz)
		{
			return ori(m3x4.Scale(sx, sy, sz));
		}
		public TDerived scale(v4 s)
		{
			return ori(m3x4.Scale(s.x, s.y, s.z));
		}
		public TDerived euler(float pitch_deg, float yaw_deg, float roll_deg)
		{
			return ori(m3x4.RotationDeg(pitch_deg, yaw_deg, roll_deg));
		}

		/// <summary>Wire frame</summary>
		public TDerived wireframe(bool w = true)
		{
			m_wire = new(w);
			return (TDerived)this;
		}

		/// <summary>Axis Id</summary>
		public TDerived axis(AxisId axis_id)
		{
			m_axis_id = new(axis_id);
			return (TDerived)this;
		}

		/// <summary>Solid</summary>
		public TDerived solid(bool solid = true)
		{
			m_solid = new(solid);
			return (TDerived)this;
		}

		/// <summary>Solid</summary>
		public TDerived hide(bool hidden = true)
		{
			m_hidden = new(hidden);
			return (TDerived)this;
		}

		/// <summary>NoZTest</summary>
		public TDerived ztest(bool on = true)
		{
			m_ztest = new(on);
			return (TDerived)this;
		}
		
		/// <summary>NoZWrite</summary>
		public TDerived zwrite(bool on = true)
		{
			m_zwrite = new(on);
			return (TDerived)this;
		}

		/// <summary>Bake a transform into the object</summary>
		public LdrTransform bake()
		{
			m_bake ??= new(EKeyword.BakeTransform);
			return m_bake;
		}

		/// <summary>The text font</summary>
		public LdrFont font()
		{
			m_font ??= new();
			return m_font;
		}

		/// <summary>Reflectivity amount</summary>
		public TDerived reflectivity(float amount)
		{
			m_reflectivity = new(amount);
			return (TDerived)this;
		}

		/// <summary>Screen space rendering</summary>
		public TDerived screen_space(bool on = true)
		{
			m_screen_space = new(on);
			return (TDerived)this;
		}

		/// <summary>Root animation</summary>
		public LdrRootAnimation root_anim()
		{
			m_root_anim ??= new();
			return m_root_anim;
		}

		/// <inheritdoc/>
		public override void WriteTo(IWriter writer)
		{
			writer.Append(m_axis_id, m_wire, m_solid, m_hidden, m_group_colour, m_reflectivity, m_screen_space, m_ztest, m_zwrite, m_o2w);
			if (m_font != null) m_font.WriteTo(writer);
			if (m_bake != null) m_bake.WriteTo(writer);
			if (m_root_anim != null) m_root_anim.WriteTo(writer);
			base.WriteTo(writer);
		}
	}

	// Modifiers
	public class LdrTransform(EKeyword kw)
	{
		private EKeyword m_kw = kw;
		private Serialiser.O2W m_o2w = new();

		public LdrTransform o2w(m4x4 o2w)
		{
			m_o2w.m_mat = o2w * m_o2w.m_mat;
			return this;
		}
		public LdrTransform o2w(m3x4 rot, v4 pos)
		{
			return o2w(new m4x4(rot, pos));
		}
		public LdrTransform ori(v4 dir, AxisId axis)
		{
			return ori(m3x4.Rotation(axis.Axis, dir));
		}
		public LdrTransform ori(m3x4 rot)
		{
			return o2w(rot, v4.Origin);
		}
		public LdrTransform ori(Quat q)
		{
			return o2w(m4x4.Transform(q, v4.Origin));
		}
		public LdrTransform pos(float x, float y, float z)
		{
			return o2w(m4x4.Translation(x, y, z));
		}
		public LdrTransform pos(v4 pos)
		{
			return o2w(m4x4.Translation(pos));
		}
		public LdrTransform pos(v3 pos)
		{
			return this.pos(pos.w1);
		}
		public LdrTransform scale(float s)
		{
			return scale(s, s, s);
		}
		public LdrTransform scale(float sx, float sy, float sz)
		{
			return ori(m3x4.Scale(sx, sy, sz));
		}
		public LdrTransform scale(v4 s)
		{
			return ori(m3x4.Scale(s.x, s.y, s.z));
		}
		public LdrTransform euler(float pitch_deg, float yaw_deg, float roll_deg)
		{
			return ori(m3x4.RotationDeg(pitch_deg, yaw_deg, roll_deg));
		}

		// Write to 'out'
		public void WriteTo(IWriter res)
		{
			res.Write(m_kw, m_o2w);
		}
	}
	public class LdrTexture
	{
		private string m_filepath = string.Empty;
		private View3d.EAddrMode[] m_addr = [View3d.EAddrMode.Wrap, View3d.EAddrMode.Wrap];
		private View3d.EFilter m_filter = View3d.EFilter.Linear;
		private Serialiser.Alpha m_has_alpha = new();
		private Serialiser.O2W m_t2s = new();

		// Texture filepath
		public LdrTexture path(string filepath)
		{
			m_filepath = filepath;
			return this;
		}

		// Addressing mode
		public LdrTexture addr(View3d.EAddrMode addrU, View3d.EAddrMode addrV)
		{
			m_addr[0] = addrU;
			m_addr[1] = addrV;
			return this;
		}

		// Filtering mode
		public LdrTexture filter(View3d.EFilter filter)
		{
			m_filter = filter;
			return this;
		}

		// Texture to surface transform
		public LdrTexture t2s(Serialiser.O2W t2s)
		{
			m_t2s = t2s;
			return this;
		}

		// Has alpha flag
		public LdrTexture alpha(Serialiser.Alpha has_alpha)
		{
			m_has_alpha = has_alpha;
			return this;
		}

		// Write to 'out'
		public void WriteTo(IWriter res)
		{
			if (m_filepath.Length == 0) return;
			res.Write(EKeyword.Texture, () =>
			{
				res.Write(EKeyword.FilePath, "\"", m_filepath, "\"");
				res.Write(EKeyword.Addr, m_addr[0], m_addr[1]);
				res.Write(EKeyword.Filter, m_filter);
				res.Append(m_has_alpha);
				res.Append(m_t2s);
			});
		}
	}
	public class LdrAnimation
	{
		private RangeI? m_frame_range = null;
		private bool m_no_translation = false;
		private bool m_no_rotation = false;

		/// <summary>Limit frame range</summary>
		public LdrAnimation frame_range(int beg, int end)
		{
			m_frame_range = new RangeI(beg, end);
			return this;
		}
		public LdrAnimation frame(int frame)
		{
			return frame_range(frame, frame + 1);
		}

		// Anim flags
		public LdrAnimation no_translation(bool on = true)
		{
			m_no_translation = on;
			return this;
		}
		public LdrAnimation no_rotation(bool on = true)
		{
			m_no_rotation = on;
			return this;
		}

		// Write to 'out'
		public void WriteTo(IWriter res)
		{
			res.Write(EKeyword.Animation, () =>
			{
				if (m_frame_range is RangeI range)
				{
					if (range.Count == 1)
						res.Write(EKeyword.Frame, range.Beg);
					else
						res.Write(EKeyword.FrameRange, range.Beg, range.End);
				}
				if (m_no_translation)
				{
					res.Write(EKeyword.NoRootTranslation);
				}
				if (m_no_rotation)
				{
					res.Write(EKeyword.NoRootRotation);
				}
			});
		}
	}
	public class LdrMontage
	{
		/// <summary>A frame reference within a montage</summary>
		public struct FrameEntry
		{
			public int m_source_index;
			public int m_frame_number;
			public float? m_duration;
			public m4x4? m_o2w;
		}

		private List<string> m_anim_sources = [];
		private List<FrameEntry> m_frame_entries = [];
		private string? m_style = null;
		private float? m_stretch = null;
		private float? m_time_bias = null;
		private bool m_no_translation = false;
		private bool m_no_rotation = false;
		private bool m_hide_when_not_animating = false;

		/// <summary>Add an additional animation source file (indices 1, 2, ...)</summary>
		public LdrMontage anim_source(string filepath)
		{
			m_anim_sources.Add(filepath);
			return this;
		}

		/// <summary>Add a frame reference</summary>
		public LdrMontage frame(int source_index, int frame_number, float? duration = null, m4x4? o2w = null)
		{
			m_frame_entries.Add(new FrameEntry
			{
				m_source_index = source_index,
				m_frame_number = frame_number,
				m_duration = duration,
				m_o2w = o2w,
			});
			return this;
		}

		/// <summary>Set the playback style</summary>
		public LdrMontage style(string style)
		{
			m_style = style;
			return this;
		}

		/// <summary>Set the playback speed multiplier</summary>
		public LdrMontage stretch(float stretch)
		{
			m_stretch = stretch;
			return this;
		}

		/// <summary>Set the time bias</summary>
		public LdrMontage time_bias(float bias)
		{
			m_time_bias = bias;
			return this;
		}

		/// <summary>Disable root bone translation</summary>
		public LdrMontage no_translation(bool on = true)
		{
			m_no_translation = on;
			return this;
		}

		/// <summary>Disable root bone rotation</summary>
		public LdrMontage no_rotation(bool on = true)
		{
			m_no_rotation = on;
			return this;
		}

		/// <summary>Hide the model when not animating</summary>
		public LdrMontage hide_when_not_animating(bool on = true)
		{
			m_hide_when_not_animating = on;
			return this;
		}

		// Write to 'out'
		public void WriteTo(IWriter res)
		{
			res.Write(EKeyword.Montage, () =>
			{
				foreach (var src in m_anim_sources)
					res.Write(EKeyword.AnimSource, $"\"{src}\"");

				foreach (var entry in m_frame_entries)
				{
					res.Write(EKeyword.Frame, () =>
					{
						res.Append(entry.m_source_index, entry.m_frame_number);
						if (entry.m_duration is float duration)
							res.Write(EKeyword.Period, duration);
						if (entry.m_o2w is m4x4 o2w)
							res.Write(EKeyword.O2W, () => res.Write(EKeyword.M4x4, () => res.Append(o2w)));
					});
				}

				if (m_style != null) res.Write(EKeyword.Style, m_style);
				if (m_stretch != null) res.Write(EKeyword.Stretch, m_stretch.Value);
				if (m_time_bias != null) res.Write(EKeyword.TimeBias, m_time_bias.Value);
				if (m_no_translation) res.Write(EKeyword.NoRootTranslation);
				if (m_no_rotation) res.Write(EKeyword.NoRootRotation);
				if (m_hide_when_not_animating) res.Write(EKeyword.HideWhenNotAnimating);
			});
		}
	}
	public class LdrFont
	{
		private string? m_name = null;
		private float? m_size = null;
		private Colour32? m_colour = null;
		private int? m_weight = null;
		private string? m_style = null;
		private int? m_stretch = null;
		private bool? m_underline = null;
		private bool? m_strikeout = null;

		public LdrFont name(string name)
		{
			m_name = !string.IsNullOrEmpty(name) ? name : null;
			return this;
		}
		public LdrFont size(float sz)
		{
			m_size = sz != 0 ? sz : null;
			return this;
		}
		public LdrFont colour(Colour32 col)
		{
			m_colour = col != Colour32.Zero ? col : null;
			return this;
		}
		public LdrFont weight(int w)
		{
			m_weight = w != 0 ? w : null;
			return this;
		}
		public LdrFont style(string s)
		{
			m_style = !string.IsNullOrEmpty(s) ? s : null;
			return this;
		}
		public LdrFont stretch(int s)
		{
			m_stretch = s != 0 ? s : null;
			return this;
		}
		public LdrFont underline(bool on = true)
		{
			m_underline = on ? true : null;
			return this;
		}
		public LdrFont strikeout(bool on = true)
		{
			m_strikeout = on ? true : null;
			return this;
		}

		// Write to 'out'
		public void WriteTo(IWriter res)
		{
			res.Write(EKeyword.Font, () =>
			{
				if (m_name != null)
					res.Write(EKeyword.Name, "\"", m_name, "\"");
				if (m_size != null)
					res.Write(EKeyword.Size, m_size.Value);
				if (m_colour != null)
					res.Write(EKeyword.Colour, m_colour.Value);
				if (m_weight != null)
					res.Write(EKeyword.Weight, m_weight.Value);
				if (m_style != null)
					res.Write(EKeyword.Style, m_style);
				if (m_stretch != null)
					res.Write(EKeyword.Stretch, m_stretch.Value);
				if (m_underline != null)
					res.Write(EKeyword.Underline, m_underline.Value);
				if (m_strikeout != null)
					res.Write(EKeyword.Strikeout, m_strikeout.Value);
			});
		}
	}

	public class LdrRootAnimation
	{
		private string? m_style = null;
		private float? m_period = null;
		private v4? m_velocity = null;
		private v4? m_accel = null;
		private v4? m_ang_velocity = null;
		private v4? m_ang_accel = null;

		public LdrRootAnimation style(string style)
		{
			m_style = style;
			return this;
		}
		public LdrRootAnimation period(float period)
		{
			m_period = period;
			return this;
		}
		public LdrRootAnimation velocity(v4 vel)
		{
			m_velocity = vel;
			return this;
		}
		public LdrRootAnimation accel(v4 acc)
		{
			m_accel = acc;
			return this;
		}
		public LdrRootAnimation ang_velocity(v4 ang_vel)
		{
			m_ang_velocity = ang_vel;
			return this;
		}
		public LdrRootAnimation ang_accel(v4 ang_acc)
		{
			m_ang_accel = ang_acc;
			return this;
		}

		// Write to 'out'
		public void WriteTo(IWriter res)
		{
			res.Write(EKeyword.RootAnimation, () =>
			{
				if (m_style != null) res.Write(EKeyword.Style, m_style);
				if (m_period != null) res.Write(EKeyword.Period, m_period.Value);
				if (m_velocity is v4 vel) res.Write(EKeyword.Velocity, vel.xyz);
				if (m_accel is v4 acc) res.Write(EKeyword.Accel, acc.xyz);
				if (m_ang_velocity is v4 ang_vel) res.Write(EKeyword.AngVelocity, ang_vel.xyz);
				if (m_ang_accel is v4 ang_acc) res.Write(EKeyword.AngAccel, ang_acc.xyz);
			});
		}
	}

	// Object types
	public class LdrPoint : LdrBase<LdrPoint>
	{
		private class Pt { public v4 pt; public Colour32 col; };
		private readonly List<Pt> m_points = [];
		private Serialiser.Size2 m_size = new();
		private Serialiser.Depth m_depth = new();
		private Serialiser.PointStyle m_style = new();
		private Serialiser.PerItemColour m_per_item_colour = new();

		public LdrPoint style(EPointStyle s)
		{
			m_style = new(s);
			return this;
		}

		// Points
		public LdrPoint pt(v4 point, Colour32? colour = null)
		{
			m_points.Add(new Pt { pt = point, col = colour ?? Colour32.White });
			if (colour != null) m_per_item_colour.m_per_item_colour = true;
			return this;
		}
		public LdrPoint pt(v3 point, Colour32? colour = null)
		{
			return pt(point.w1, colour);
		}

		// Point size (in pixels if depth == false, in world space if depth == true)
		public LdrPoint size(float s)
		{
			m_size = new v2(s);
			return this;
		}
		public LdrPoint size(v2 s)
		{
			m_size = s;
			return this;
		}

		// Points have depth
		public LdrPoint depth(bool d = true)
		{
			m_depth = new(d);
			return this;
		}

		/// <inheritdoc/>
		public override void WriteTo(IWriter res)
		{
			res.Write(EKeyword.Point, m_name, m_colour, () =>
			{
				res.Append(m_style);
				res.Append(m_size, m_depth, m_per_item_colour);
				res.Write(EKeyword.Data, () =>
				{
					foreach (var point in m_points)
					{
						res.Append(point.pt.xyz);
						if (m_per_item_colour.m_per_item_colour)
							res.Append(point.col);
					}
				});
				base.WriteTo(res);
			});
		}
	}
	public class LdrLine : LdrBase<LdrLine>
	{
		private class Pt { public v4 a; public Colour32 col; };
		private class Ln { public v4 a, b; public Colour32 col; };
		private class Block
		{
			public readonly List<Ln> m_lines = [];
			public readonly List<Pt> m_strip = [];
			public Serialiser.LineStyle m_style = new();
			public Serialiser.Smooth m_smooth = new();
			public Serialiser.Width m_width = new();
			public Serialiser.Dashed m_dashed = new();
			public Serialiser.ArrowHeads m_arrow = new();
			public Serialiser.DataPoints m_data_points = new();
			public Serialiser.PerItemColour m_per_item_colour = new();
			public static implicit operator bool(Block? b) => b != null && (b.m_lines.Count > 0 || b.m_strip.Count > 0);
		}

		private readonly List<Block> m_blocks = [];
		private Block m_current = new();

		public LdrLine style(ELineStyle sty)
		{
			m_current.m_style = new(sty);
			return this;
		}
		public LdrLine per_item_colour(bool on = true)
		{
			m_current.m_per_item_colour = new(on);
			return this;
		}
		public LdrLine smooth(bool smooth = true)
		{
			m_current.m_smooth = new(smooth);
			return this;
		}
		public LdrLine width(float w)
		{
			m_current.m_width = new(w);
			return this;
		}
		public LdrLine data_points(float size, Colour32? colour = null, EPointStyle? style = null)
		{
			return data_points(new v2(size, size), colour, style);
		}
		public LdrLine data_points(v2 size, Colour32? colour = null, EPointStyle? style = null)
		{
			m_current.m_data_points = new Serialiser.DataPoints(size, colour, style);
			return this;
		}
		public LdrLine dashed(v2 dash)
		{
			m_current.m_dashed = new Serialiser.Dashed(dash);
			return this;
		}
		public LdrLine arrow(EArrowType style = EArrowType.Fwd, float size = 10f)
		{
			m_current.m_arrow = new Serialiser.ArrowHeads(style, size);
			return this;
		}

		// Lines
		public LdrLine line(v4 a, v4 b, Colour32? colour = null)
		{
			m_current.m_lines.Add(new Ln{ a = a, b = b, col = colour ?? Colour32.White });
			if (colour != null) m_current.m_per_item_colour.m_per_item_colour = true;
			m_current.m_strip.Clear();
			return this;
		}
		public LdrLine line(v3 a, v3 b, Colour32? colour = null)
		{
			return line(a.w1, b.w1, colour);
		}
		public LdrLine lines(Span<v4> verts, Span<int> indices)
		{
			Debug.Assert((indices.Length % 2) == 0);
			for (var i = 0; i < indices.Length; i += 2)
				line(verts[indices[i+0]], verts[indices[i+1]]);

			return this;
		}

		// Line strip
		public LdrLine strip(v4 start, Colour32? colour = null)
		{
			m_current.m_strip.Add(new Pt { a = start, col = colour ?? Colour32.White });
			if (colour != null) m_current.m_per_item_colour.m_per_item_colour = true;
			m_current.m_lines.Clear();
			return this;
		}
		public LdrLine strip(v3 start, Colour32? colour = null)
		{
			return strip(start.w1, colour);
		}
		public LdrLine line_to(v4 pt, Colour32? colour = null)
		{
			if (m_current.m_strip.Count == 0) strip(pt, colour);
			return strip(pt, colour);
		}
		public LdrLine line_to(v3 pt, Colour32? colour = null)
		{
			return line_to(pt.w1, colour);
		}

		public LdrLine new_block()
		{
			m_blocks.Add(m_current);
			m_current = new();
			return this;
		}

		/// <inheritdoc/>
		public override void WriteTo(IWriter res)
		{
			res.Write(EKeyword.Line, m_name, m_colour, () =>
			{
				void WriteBlock(Block block)
				{
					res.Append(block.m_style, block.m_smooth, block.m_width, block.m_dashed, block.m_arrow, block.m_data_points, block.m_per_item_colour);
					res.Write(EKeyword.Data, () =>
					{
						foreach (var line in block.m_lines)
						{
							res.Append(line.a.xyz, line.b.xyz);
							if (block.m_per_item_colour)
								res.Append(line.col);
						}
						foreach (var pt in block.m_strip)
						{
							res.Append(pt.a.xyz);
							if (block.m_per_item_colour)
								res.Append(pt.col);
						}
					});
				}

				foreach (var block in m_blocks)
					WriteBlock(block);
				if (m_current)
					WriteBlock(m_current);

				base.WriteTo(res);
			});
		}
	}
	public class LdrLineBox : LdrBase<LdrLineBox>
	{
		private v4 m_dim = new(1f);
		private Serialiser.Width m_width = new();
		private Serialiser.Dashed m_dashed = new();

		public LdrLineBox dim(float d)
		{
			return dim(d, d, d);
		}
		public LdrLineBox dim(float w, float h, float d)
		{
			m_dim = new v4(w, h, d, 0);
			return this;
		}
		public LdrLineBox dim(v4 dim)
		{
			m_dim = dim;
			return this;
		}
		public LdrLineBox width(float w)
		{
			m_width = new(w);
			return this;
		}
		public LdrLineBox dashed(v2 dash)
		{
			m_dashed = new Serialiser.Dashed(dash);
			return this;
		}

		public override void WriteTo(IWriter res)
		{
			res.Write(EKeyword.LineBox, m_name, m_colour, () =>
			{
				res.Write(EKeyword.Data, m_dim.xyz);
				res.Append(m_width, m_dashed);
				base.WriteTo(res);
			});
		}
	}
	public class LdrGrid : LdrBase<LdrGrid>
	{
		private v2 m_wh = new(1f, 1f);
		private int m_div_w = 0;
		private int m_div_h = 0;
		private Serialiser.Width m_width = new();
		private Serialiser.Dashed m_dashed = new();

		public LdrGrid wh(float w, float h)
		{
			m_wh = new v2(w, h);
			return this;
		}
		public LdrGrid divisions(int w, int h)
		{
			m_div_w = w;
			m_div_h = h;
			return this;
		}
		public LdrGrid width(float w)
		{
			m_width = new(w);
			return this;
		}
		public LdrGrid dashed(v2 dash)
		{
			m_dashed = new Serialiser.Dashed(dash);
			return this;
		}

		public override void WriteTo(IWriter res)
		{
			res.Write(EKeyword.Grid, m_name, m_colour, () =>
			{
				if (m_div_w != 0 && m_div_h != 0)
					res.Write(EKeyword.Data, m_wh.x, m_wh.y, m_div_w, m_div_h);
				else
					res.Write(EKeyword.Data, m_wh);
				res.Append(m_width, m_dashed);
				base.WriteTo(res);
			});
		}
	}
	public class LdrCoordFrame : LdrBase<LdrCoordFrame>
	{
		private Serialiser.Scale m_scale = new();
		private Serialiser.LeftHanded m_lh = new();
		private Serialiser.Width m_width = new();

		// Scale size
		public new LdrCoordFrame scale(float s)
		{
			m_scale = s;
			return this;
		}

		// Left handed axis
		public LdrCoordFrame left_handed(bool lh = true)
		{
			m_lh = new(lh);
			return this;
		}

		// Width
		public LdrCoordFrame width(float w)
		{
			m_width = new(w);
			return this;
		}

		/// <inheritdoc/>
		public override void WriteTo(IWriter res)
		{
			res.Write(EKeyword.CoordFrame, m_name, m_colour, () =>
			{
				res.Append(m_scale, m_lh);
				res.Append(m_width);
				base.WriteTo(res);
			});
		}
	}
	public class LdrTriangle : LdrBase<LdrTriangle>
	{
		private struct Tri { public v4 a, b, c; public Colour32 col; };
		private readonly List<Tri> m_tris = [];
		private Serialiser.PerItemColour m_per_item_colour = new();

		public LdrTriangle tri(v4 a, v4 b, v4 c, Colour32? colour = null)
		{
			m_tris.Add(new Tri{ a = a, b = b, c = c, col = colour ?? Colour32.White });
			if (colour != null) m_per_item_colour.m_per_item_colour = true;
			return this;
		}
		public LdrTriangle tris(Span<v4> verts, Span<int> faces)
		{
			Debug.Assert((faces.Length % 3) == 0);
			for (var i = 0; i < faces.Length; i += 3)
				tri(verts[faces[i+0]], verts[faces[i+1]], verts[faces[i+2]]);

			return this;
		}

		/// <inheritdoc/>
		public override void WriteTo(IWriter res)
		{
			res.Write(EKeyword.Triangle, m_name, m_colour, () =>
			{
				res.Write(EKeyword.Data, () =>
				{
					foreach (var tri in m_tris)
					{
						res.Append(tri.a.xyz, tri.b.xyz, tri.c.xyz);
						if (m_per_item_colour.m_per_item_colour)
							res.Append(tri.col);
					}
				});
				base.WriteTo(res);
			});
		}
	}
	public class LdrQuad : LdrBase<LdrQuad>
	{
		private struct Qd { public v4 a, b, c, d; public Colour32 col_a, col_b, col_c, col_d; }
		private readonly List<Qd> m_quads = [];
		private Serialiser.PerItemColour m_per_item_colour = new();
		private LdrTexture m_tex = new();

		public LdrQuad quad(v4 a, v4 b, v4 c, v4 d, Colour32? col_a = null, Colour32? col_b = null, Colour32? col_c = null, Colour32? col_d = null)
		{
			m_quads.Add(new Qd
			{
				a = a, b = b, c = c, d = d,
				col_a = col_a ?? Colour32.White,
				col_b = col_b ?? Colour32.White,
				col_c = col_c ?? Colour32.White,
				col_d = col_d ?? Colour32.White,
			});
			if (col_a != null) m_per_item_colour.m_per_item_colour = true;
			return this;
		}
		public LdrTexture texture()
		{
			return m_tex;
		}

		public override void WriteTo(IWriter res)
		{
			res.Write(EKeyword.Quad, m_name, m_colour, () =>
			{
				res.Append(m_per_item_colour);
				res.Write(EKeyword.Data, () =>
				{
					foreach (var q in m_quads)
					{
						res.Append(q.a.xyz, q.b.xyz, q.c.xyz, q.d.xyz);
						if (m_per_item_colour.m_per_item_colour)
							res.Append(q.col_a, q.col_b, q.col_c, q.col_d);
					}
				});
				m_tex.WriteTo(res);
				base.WriteTo(res);
			});
		}
	}
	public class LdrPlane : LdrBase<LdrPlane>
	{
		private v4 m_position = v4.Origin;
		private v4 m_direction = v4.ZAxis;
		private v2 m_wh = new(1,1);
		private LdrTexture m_tex = new();

		public LdrPlane plane(v4 p)
		{
			pos((p.xyz * -p.w).w1);
			ori(Math_.Normalise(p.xyz.w0), EAxisId.PosZ);
			return this;
		}
		public LdrPlane wh(float width, float height)
		{
			m_wh = new(width, height);
			return this;
		}
		public LdrPlane wh(v2 wh)
		{
			m_wh = wh;
			return this;
		}

		public LdrTexture texture()
		{
			return m_tex;
		}

		/// <inheritdoc/>
		public override void WriteTo(IWriter res)
		{
			res.Write(EKeyword.Plane, m_name, m_colour, () =>
			{
				res.Write(EKeyword.Data, m_wh);
				m_tex.WriteTo(res);
				base.WriteTo(res);
			});
		}
	}
	public class LdrRibbon : LdrBase<LdrRibbon>
	{
		private class Pt { public v4 pt; public Colour32 col; }
		private readonly List<Pt> m_points = [];
		private Serialiser.PerItemColour m_per_item_colour = new();
		private Serialiser.Width m_width = new();
		private Serialiser.Smooth m_smooth = new();
		private LdrTexture m_tex = new();

		public LdrRibbon pt(v4 point, Colour32? colour = null)
		{
			m_points.Add(new Pt { pt = point, col = colour ?? Colour32.White });
			if (colour != null) m_per_item_colour.m_per_item_colour = true;
			return this;
		}
		public LdrRibbon pt(v3 point, Colour32? colour = null)
		{
			return pt(point.w1, colour);
		}
		public LdrRibbon width(float w)
		{
			m_width = new(w);
			return this;
		}
		public LdrRibbon smooth(bool on = true)
		{
			m_smooth = new(on);
			return this;
		}
		public LdrTexture texture()
		{
			return m_tex;
		}

		public override void WriteTo(IWriter res)
		{
			res.Write(EKeyword.Ribbon, m_name, m_colour, () =>
			{
				res.Append(m_per_item_colour);
				res.Write(EKeyword.Data, () =>
				{
					foreach (var p in m_points)
					{
						res.Append(p.pt.xyz);
						if (m_per_item_colour.m_per_item_colour)
							res.Append(p.col);
					}
				});
				res.Append(m_width, m_smooth);
				m_tex.WriteTo(res);
				base.WriteTo(res);
			});
		}
	}
	public class LdrCircle : LdrBase<LdrCircle>
	{
		private float m_radius = 1.0f;
		private Serialiser.Facets m_facets = new();

		public LdrCircle radius(float r)
		{
			m_radius = r;
			return this;
		}
		public LdrCircle facets(int count)
		{
			m_facets = new(count);
			return this;
		}

		/// <inheritdoc/>
		public override void WriteTo(IWriter res)
		{
			res.Write(EKeyword.Circle, m_name, m_colour, () =>
			{
				res.Write(EKeyword.Data, m_radius);
				res.Append(m_facets);
				base.WriteTo(res);
			});
		}
	}
	public class LdrPie : LdrBase<LdrPie>
	{
		private float m_angle0 = 0f;
		private float m_angle1 = 90f;
		private float m_inner_radius = 0f;
		private float m_outer_radius = 1f;
		private Serialiser.Facets m_facets = new();
		private Serialiser.Scale2 m_scale = new();

		public LdrPie pie(float angle0, float angle1, float inner_radius, float outer_radius)
		{
			m_angle0 = angle0;
			m_angle1 = angle1;
			m_inner_radius = inner_radius;
			m_outer_radius = outer_radius;
			return this;
		}
		public LdrPie facets(int count)
		{
			m_facets = new(count);
			return this;
		}
		public LdrPie scale(Serialiser.Scale2 s)
		{
			m_scale = s;
			return this;
		}

		public override void WriteTo(IWriter res)
		{
			res.Write(EKeyword.Pie, m_name, m_colour, () =>
			{
				res.Write(EKeyword.Data, m_angle0, m_angle1, m_inner_radius, m_outer_radius);
				res.Append(m_facets, m_scale);
				base.WriteTo(res);
			});
		}
	}
	public class LdrRect : LdrBase<LdrRect>
	{
		private v2 m_wh = new(1f, 1f);
		private Serialiser.CornerRadius m_corner_radius = new();
		private Serialiser.Facets m_facets = new();

		public LdrRect wh(float w, float h)
		{
			m_wh = new v2(w, h);
			return this;
		}
		public LdrRect wh(float s)
		{
			return wh(s, s);
		}
		public LdrRect corner_radius(float r)
		{
			m_corner_radius = new(r);
			return this;
		}
		public LdrRect facets(int count)
		{
			m_facets = new(count);
			return this;
		}

		public override void WriteTo(IWriter res)
		{
			res.Write(EKeyword.Rect, m_name, m_colour, () =>
			{
				res.Write(EKeyword.Data, m_wh);
				res.Append(m_corner_radius, m_facets);
				base.WriteTo(res);
			});
		}
	}
	public class LdrPolygon : LdrBase<LdrPolygon>
	{
		private class Pt { public v2 pt; public Colour32 col; }
		private readonly List<Pt> m_points = [];
		private Serialiser.PerItemColour m_per_item_colour = new();

		public LdrPolygon pt(v2 point, Colour32? colour = null)
		{
			m_points.Add(new Pt { pt = point, col = colour ?? Colour32.White });
			if (colour != null) m_per_item_colour.m_per_item_colour = true;
			return this;
		}
		public LdrPolygon pt(float x, float y, Colour32? colour = null)
		{
			return pt(new v2(x, y), colour);
		}

		public override void WriteTo(IWriter res)
		{
			res.Write(EKeyword.Polygon, m_name, m_colour, () =>
			{
				res.Append(m_per_item_colour);
				res.Write(EKeyword.Data, () =>
				{
					foreach (var p in m_points)
					{
						res.Append(p.pt);
						if (m_per_item_colour.m_per_item_colour)
							res.Append(p.col);
					}
				});
				base.WriteTo(res);
			});
		}
	}
	public class LdrSphere : LdrBase<LdrSphere>
	{
		private v4 m_radius = new(1.0f);
		private Serialiser.Facets m_facets = new();

		// Radius
		public LdrSphere radius(float r)
		{
			return radius(new v4(r));
		}
		public LdrSphere radius(v4 r)
		{
			m_radius = r;
			return this;
		}
		public LdrSphere facets(int count)
		{
			m_facets = new(count);
			return this;
		}

		// Create from bounding sphere
		public LdrSphere bsphere(BSphere bsphere)
		{
			if (bsphere == BSphere.Reset) return this;
			return radius(bsphere.Radius).pos(bsphere.Centre);
		}

		/// <inheritdoc/>
		public override void WriteTo(IWriter res)
		{
			res.Write(EKeyword.Sphere, m_name, m_colour, () =>
			{
				res.Write(EKeyword.Data, m_radius.xyz);
				res.Append(m_facets);
				base.WriteTo(res);
			});
		}

	}
	public class LdrBox : LdrBase<LdrBox>
	{
		private v4 m_dim = new(0.5f);

		// Box dimensions
		public LdrBox radii(float radii)
		{
			return dim(radii * 2);
		}
		public LdrBox radii(v4 radii)
		{
			return dim(radii * 2);
		}
		public LdrBox dim(float d)
		{
			return dim(d, d, d);
		}
		public LdrBox dim(float sx, float sy, float sz)
		{
			return dim(new v4(sx, sy, sz, 0));
		}
		public LdrBox dim(v4 dim)
		{
			m_dim = new v4(dim.x, dim.y, dim.z, 0);
			return this;
		}

		// Create from bounding box
		public LdrBox bbox(BBox bbox)
		{
			if (bbox == BBox.Reset) return this;
			return dim(2 * bbox.Radius).pos(bbox.Centre);
		}

		/// <inheritdoc/>
		public override void WriteTo(IWriter res)
		{
			res.Write(EKeyword.Box, m_name, m_colour, () =>
			{
				res.Write(EKeyword.Data, m_dim.xyz);
				base.WriteTo(res);
			});
		}
	}
	public class LdrBoxList : LdrBase<LdrBoxList>
	{
		private class Entry { public v4 dim; public v4 pos; public Colour32 col; }
		private readonly List<Entry> m_boxes = [];
		private Serialiser.PerItemColour m_per_item_colour = new();

		public LdrBoxList box(v4 dim, v4 pos, Colour32? colour = null)
		{
			m_boxes.Add(new Entry { dim = dim, pos = pos, col = colour ?? Colour32.White });
			if (colour != null) m_per_item_colour.m_per_item_colour = true;
			return this;
		}
		public LdrBoxList box(float w, float h, float d, float x, float y, float z, Colour32? colour = null)
		{
			return box(new v4(w, h, d, 0), new v4(x, y, z, 1), colour);
		}

		public override void WriteTo(IWriter res)
		{
			res.Write(EKeyword.BoxList, m_name, m_colour, () =>
			{
				res.Append(m_per_item_colour);
				res.Write(EKeyword.Data, () =>
				{
					foreach (var b in m_boxes)
					{
						res.Append(b.dim.xyz, b.pos.xyz);
						if (m_per_item_colour.m_per_item_colour)
							res.Append(b.col);
					}
				});
				base.WriteTo(res);
			});
		}
	}
	public class LdrCylinder : LdrBase<LdrCylinder>
	{
		private v2 m_radius = new(0.5f);
		private Serialiser.Scale2 m_scale = new();
		private float m_height = 1.0f;

		// Height/Radius
		public LdrCylinder cylinder(float height, float radius)
		{
			return cylinder(height, radius, radius);
		}
		public LdrCylinder cylinder(float height, float radius_base, float radius_tip)
		{
			m_radius = new v2(radius_base, radius_tip);
			m_height = height;
			return this;
		}

		// Scale
		public LdrCylinder scale(Serialiser.Scale2 scale)
		{
			m_scale = scale;
			return this;
		}

		/// <inheritdoc/>
		public override void WriteTo(IWriter res)
		{
			res.Write(EKeyword.Cylinder, m_name, m_colour, () =>
			{
				res.Write(EKeyword.Data, m_height, m_radius.x, m_radius.y);
				res.Append(m_scale);
				base.WriteTo(res);
			});
		}
	}
	public class LdrCone : LdrBase<LdrCone>
	{
		private v2 m_distance = new(0f, 1f);
		private Serialiser.Scale2 m_scale = new();
		private float m_angle = 45.0f;

		// Height/Radius
		public LdrCone angle(float solid_angle_deg)
		{
			m_angle = solid_angle_deg;
			return this;
		}
		public LdrCone height(float height)
		{
			m_distance = new v2(m_distance.x, m_distance.x + height);
			return this;
		}
		public LdrCone dist(float dist0, float dist1)
		{
			m_distance = new v2(dist0, dist1);
			return this;
		}

		// Scale
		public LdrCone scale(Serialiser.Scale2 scale)
		{
			m_scale = scale;
			return this;
		}

		/// <inheritdoc/>
		public override void WriteTo(IWriter res)
		{
			res.Write(EKeyword.Cone, m_name, m_colour, () =>
			{
				res.Write(EKeyword.Data, m_angle, m_distance.x, m_distance.y);
				res.Append(m_scale);
				base.WriteTo(res);
			});
		}
	}
	public class LdrTube : LdrBase<LdrTube>
	{
		// Cross section types
		public enum ECrossSection { Round, Square, Polygon }

		private class Pt { public v4 pt; public Colour32 col; }
		private readonly List<Pt> m_points = [];
		private ECrossSection m_cs_type = ECrossSection.Round;
		private v2 m_cs_radius = new(0.2f, 0.2f);
		private readonly List<v2> m_cs_polygon = [];
		private Serialiser.Facets m_cs_facets = new();
		private Serialiser.Smooth m_cs_smooth = new();
		private Serialiser.PerItemColour m_per_item_colour = new();
		private Serialiser.Smooth m_smooth = new();
		private Serialiser.Closed m_closed = new();

		// Cross section configuration
		public LdrTube cross_section_round(float rx, float ry = 0)
		{
			m_cs_type = ECrossSection.Round;
			m_cs_radius = new v2(rx, ry != 0 ? ry : rx);
			return this;
		}
		public LdrTube cross_section_square(float rx, float ry = 0)
		{
			m_cs_type = ECrossSection.Square;
			m_cs_radius = new v2(rx, ry != 0 ? ry : rx);
			return this;
		}
		public LdrTube cross_section_polygon(IEnumerable<v2> pts)
		{
			m_cs_type = ECrossSection.Polygon;
			m_cs_polygon.AddRange(pts);
			return this;
		}
		public LdrTube cross_section_facets(int facets)
		{
			m_cs_facets = new(facets);
			return this;
		}
		public LdrTube cross_section_smooth(bool on = true)
		{
			m_cs_smooth = new(on);
			return this;
		}

		// Path points
		public LdrTube pt(v4 point, Colour32? colour = null)
		{
			m_points.Add(new Pt { pt = point, col = colour ?? Colour32.White });
			if (colour != null) m_per_item_colour.m_per_item_colour = true;
			return this;
		}
		public LdrTube pt(v3 point, Colour32? colour = null)
		{
			return pt(point.w1, colour);
		}
		public LdrTube smooth(bool on = true)
		{
			m_smooth = new(on);
			return this;
		}
		public LdrTube closed(bool on = true)
		{
			m_closed = new(on);
			return this;
		}

		public override void WriteTo(IWriter res)
		{
			res.Write(EKeyword.Tube, m_name, m_colour, () =>
			{
				res.Write(EKeyword.CrossSection, () =>
				{
					switch (m_cs_type)
					{
						case ECrossSection.Round:
							res.Write(EKeyword.Round, m_cs_radius);
							break;
						case ECrossSection.Square:
							res.Write(EKeyword.Square, m_cs_radius);
							break;
						case ECrossSection.Polygon:
							res.Write(EKeyword.Polygon, () =>
							{
								foreach (var p in m_cs_polygon)
									res.Append(p);
							});
							break;
					}
					res.Append(m_cs_facets, m_cs_smooth);
				});
				res.Append(m_per_item_colour);
				res.Write(EKeyword.Data, () =>
				{
					foreach (var p in m_points)
					{
						res.Append(p.pt.xyz);
						if (m_per_item_colour.m_per_item_colour)
							res.Append(p.col);
					}
				});
				res.Append(m_smooth, m_closed);
				base.WriteTo(res);
			});
		}
	}
	public class LdrMesh : LdrBase<LdrMesh>
	{
		private readonly List<v4> m_verts = [];
		private readonly List<v4> m_normals = [];
		private readonly List<Colour32> m_colours = [];
		private readonly List<v2> m_tex_coords = [];
		private readonly List<int> m_faces = [];
		private readonly List<int> m_lines = [];
		private readonly List<int> m_tetras = [];
		private Serialiser.GenerateNormals m_gen_normals = new();
		private LdrTexture m_tex = new();

		public LdrMesh vert(v4 v)
		{
			m_verts.Add(v);
			return this;
		}
		public LdrMesh vert(float x, float y, float z)
		{
			return vert(new v4(x, y, z, 1));
		}
		public LdrMesh verts(IEnumerable<v4> pts)
		{
			m_verts.AddRange(pts);
			return this;
		}
		public LdrMesh normal(v4 n)
		{
			m_normals.Add(n);
			return this;
		}
		public LdrMesh normals(IEnumerable<v4> norms)
		{
			m_normals.AddRange(norms);
			return this;
		}
		public LdrMesh colour(Colour32 c)
		{
			m_colours.Add(c);
			return this;
		}
		public LdrMesh colours(IEnumerable<Colour32> cols)
		{
			m_colours.AddRange(cols);
			return this;
		}
		public LdrMesh tex_coord(v2 uv)
		{
			m_tex_coords.Add(uv);
			return this;
		}
		public LdrMesh tex_coords(IEnumerable<v2> uvs)
		{
			m_tex_coords.AddRange(uvs);
			return this;
		}
		public LdrMesh face(int i0, int i1, int i2)
		{
			m_faces.Add(i0);
			m_faces.Add(i1);
			m_faces.Add(i2);
			return this;
		}
		public LdrMesh faces(IEnumerable<int> indices)
		{
			m_faces.AddRange(indices);
			return this;
		}
		public LdrMesh line(int i0, int i1)
		{
			m_lines.Add(i0);
			m_lines.Add(i1);
			return this;
		}
		public LdrMesh lines(IEnumerable<int> indices)
		{
			m_lines.AddRange(indices);
			return this;
		}
		public LdrMesh tetra(int i0, int i1, int i2, int i3)
		{
			m_tetras.Add(i0);
			m_tetras.Add(i1);
			m_tetras.Add(i2);
			m_tetras.Add(i3);
			return this;
		}
		public LdrMesh tetras(IEnumerable<int> indices)
		{
			m_tetras.AddRange(indices);
			return this;
		}
		public LdrMesh generate_normals(float smoothing_angle)
		{
			m_gen_normals = new(smoothing_angle);
			return this;
		}
		public LdrTexture texture()
		{
			return m_tex;
		}

		public override void WriteTo(IWriter res)
		{
			res.Write(EKeyword.Mesh, m_name, m_colour, () =>
			{
				if (m_verts.Count != 0)
				{
					res.Write(EKeyword.Verts, () =>
					{
						foreach (var v in m_verts)
							res.Append(v.xyz);
					});
				}
				if (m_normals.Count != 0)
				{
					res.Write(EKeyword.Normals, () =>
					{
						foreach (var n in m_normals)
							res.Append(n.xyz);
					});
				}
				if (m_colours.Count != 0)
				{
					res.Write(EKeyword.Colours, () =>
					{
						foreach (var c in m_colours)
							res.Append(c);
					});
				}
				if (m_tex_coords.Count != 0)
				{
					res.Write(EKeyword.TexCoords, () =>
					{
						foreach (var uv in m_tex_coords)
							res.Append(uv);
					});
				}
				if (m_faces.Count != 0)
				{
					res.Write(EKeyword.Faces, () =>
					{
						foreach (var i in m_faces)
							res.Append(i);
					});
				}
				if (m_lines.Count != 0)
				{
					res.Write(EKeyword.Lines, () =>
					{
						foreach (var i in m_lines)
							res.Append(i);
					});
				}
				if (m_tetras.Count != 0)
				{
					res.Write(EKeyword.Tetra, () =>
					{
						foreach (var i in m_tetras)
							res.Append(i);
					});
				}
				res.Append(m_gen_normals);
				m_tex.WriteTo(res);
				base.WriteTo(res);
			});
		}
	}
	public class LdrConvexHull : LdrBase<LdrConvexHull>
	{
		private readonly List<v4> m_verts = [];
		private Serialiser.GenerateNormals m_gen_normals = new();

		public LdrConvexHull vert(v4 v)
		{
			m_verts.Add(v);
			return this;
		}
		public LdrConvexHull vert(float x, float y, float z)
		{
			return vert(new v4(x, y, z, 1));
		}
		public LdrConvexHull verts(IEnumerable<v4> pts)
		{
			m_verts.AddRange(pts);
			return this;
		}
		public LdrConvexHull generate_normals(float smoothing_angle)
		{
			m_gen_normals = new(smoothing_angle);
			return this;
		}

		public override void WriteTo(IWriter res)
		{
			res.Write(EKeyword.ConvexHull, m_name, m_colour, () =>
			{
				if (m_verts.Count != 0)
				{
					res.Write(EKeyword.Verts, () =>
					{
						foreach (var v in m_verts)
							res.Append(v.xyz);
					});
				}
				res.Append(m_gen_normals);
				base.WriteTo(res);
			});
		}
	}
	public class LdrFrustum : LdrBase<LdrFrustum>
	{
		private v2 m_wh = new(1f, 1f);
		private v2 m_nf = new(0.1f, 2f);
		private float m_fovY = Math_.TauBy4F;
		private float m_aspect = 1f;
		private bool m_ortho = false;

		// Orthographic
		public LdrFrustum ortho(bool ortho = true)
		{
			m_ortho = ortho;
			return this;
		}

		// Near/Far
		public LdrFrustum nf(float n, float f)
		{
			m_nf = new v2(n, f);
			return this;
		}
		public LdrFrustum nf(v2 nf_)
		{
			return nf(nf_.x, nf_.y);
		}

		// Frustum dimensions
		public LdrFrustum wh(float w, float h)
		{
			m_wh = new v2(w, h);
			m_fovY = 0;
			m_aspect = 0;
			return this;
		}
		public LdrFrustum wh(v2 sz)
		{
			return wh(sz.x, sz.y);
		}

		// Frustum angles
		public LdrFrustum fov(float fovY, float aspect)
		{
			m_ortho = false;
			m_wh = v2.Zero;
			m_fovY = fovY;
			m_aspect = aspect;
			return this;
		}

		//// From maths frustum
		//public LdrFrustum frustum(Frustum const& f)
		//{
		//	return nf(0, f.zfar()).fov(f.fovY(), f.aspect());
		//}

		// From projection matrix
		public LdrFrustum proj(m4x4 c2s)
		{
			if (c2s.w.w == 1) // If orthographic
			{
				var rh = -Math_.Sign(c2s.z.z);
				var zn = Math_.Div(c2s.w.z, c2s.z.z, 0.0f);
				var zf = Math_.Div(zn * (c2s.w.z - rh), c2s.w.z, 1.0f);
				var w = 2.0f / c2s.x.x;
				var h = 2.0f / c2s.y.y;
				return ortho(true).nf(zn, zf).wh(w,h);
			}
			else // Otherwise perspective
			{
				var rh = -Math_.Sign(c2s.z.w);
				var zn = rh * c2s.w.z / c2s.z.z;
				var zf = Math_.Div(zn * c2s.z.z, (rh + c2s.z.z), zn * 1000.0f);
				var w = 2.0f * zn / c2s.x.x;
				var h = 2.0f * zn / c2s.y.y;
				return ortho(false).nf(zn, zf).wh(w, h);
			}
		}

		/// <inheritdoc/>
		public override void WriteTo(IWriter res)
		{
			if (m_ortho)
			{
				res.Write(EKeyword.Box, m_name, m_colour, () =>
				{
					res.Write(EKeyword.Data, m_wh.x, m_wh.y, m_nf.y - m_nf.x);
					res.Append(new Serialiser.O2W(new v4(0, 0, -0.5f * (float)(m_nf.x + m_nf.y), 1)));
					base.WriteTo(res);
				});
			}
			else if (m_wh != v2.Zero)
			{
				res.Write(EKeyword.FrustumWH, m_name, m_colour, () =>
				{
					res.Write(EKeyword.Data, m_wh.x, m_wh.y, m_nf.x, m_nf.y);
					base.WriteTo(res);
				});
			}
			else
			{
				res.Write(EKeyword.FrustumFA, m_name, m_colour, () =>
				{
					res.Write(EKeyword.Data, Math_.RadiansToDegrees(m_fovY), m_aspect, m_nf.x, m_nf.y);
					base.WriteTo(res);
				});
			}
		}

	}
	public class LdrModel : LdrBase<LdrModel>
	{
		private string m_filepath = string.Empty;
		private LdrAnimation? m_anim = null;
		private LdrMontage? m_montage = null;
		private bool m_no_materials = false;

		/// <summary>Model filepath</summary>
		public LdrModel filepath(string filepath)
		{
			m_filepath = Path_.Canonicalise(filepath);
			return this;
		}

		/// <summary>Add animation to the model (mutually exclusive with montage)</summary>
		public LdrAnimation anim()
		{
			if (m_montage != null) throw new InvalidOperationException("Cannot use both *Animation and *Montage on the same *Model");
			m_anim ??= new();
			return m_anim;
		}

		/// <summary>Add a montage to the model (mutually exclusive with animation)</summary>
		public LdrMontage montage()
		{
			if (m_anim != null) throw new InvalidOperationException("Cannot use both *Animation and *Montage on the same *Model");
			m_montage ??= new();
			return m_montage;
		}

		/// <summary>Don't load materials from the model</summary>
		public LdrModel no_materials(bool on = true)
		{
			m_no_materials = on;
			return this;
		}

		/// <inheritdoc/>
		public override void WriteTo(IWriter res)
		{
			res.Write(EKeyword.Model, m_name, m_colour, () =>
			{
				res.Write(EKeyword.FilePath, $"\"{m_filepath}\"");
				m_anim?.WriteTo(res);
				m_montage?.WriteTo(res);
				if (m_no_materials) res.Write(EKeyword.NoMaterials);
				base.WriteTo(res);
			});
		}
	}
	public class LdrInstance : LdrBase<LdrInstance>
	{
		private string m_address = string.Empty;
		private LdrAnimation? m_anim = null;
		private LdrMontage? m_montage = null;

		/// <summary>Set the object address that this is an instance of</summary>
		public LdrInstance inst(string address)
		{
			m_address = address;
			return this;
		}

		/// <summary>Add animation to the instance (mutually exclusive with montage)</summary>
		public LdrAnimation anim()
		{
			if (m_montage != null) throw new InvalidOperationException("Cannot use both *Animation and *Montage on the same *Instance");
			m_anim ??= new();
			return m_anim;
		}

		/// <summary>Add a montage to the instance (mutually exclusive with animation)</summary>
		public LdrMontage montage()
		{
			if (m_anim != null) throw new InvalidOperationException("Cannot use both *Animation and *Montage on the same *Instance");
			m_montage ??= new();
			return m_montage;
		}

		/// <inheritdoc/>
		public override void WriteTo(IWriter res)
		{
			res.Write(EKeyword.Instance, m_name, m_colour, () =>
			{
				res.Write(EKeyword.Data, m_address);
				m_anim?.WriteTo(res);
				m_montage?.WriteTo(res);
				base.WriteTo(res);
			});
		}
	}
	public class LdrText : LdrBase<LdrText>
	{
		private class Block
		{
			public readonly StringBuilder m_text = new();
			public LdrFont? m_font = null;
			public bool m_screen_space = false;
			public bool m_billboard = false;
			public bool m_billboard_3d = false;
			public Colour32? m_back_colour = null;
			public Serialiser.Anchor m_anchor = new();
			public Serialiser.Padding m_padding = new();
			public string? m_format = null;
			public static implicit operator bool(Block? b) => b != null && b.m_text.Length != 0;
		}
		private readonly List<Block> m_blocks = [];
		private Block m_current = new();

		/// <summary>Direct access to the string builder for the current block</summary>
		public new StringBuilder Text => m_current.m_text;

		/// <summary>Append text to the current block</summary>
		public LdrText text(string s)
		{
			m_current.m_text.Append(s);
			return this;
		}

		/// <summary>Screen space text</summary>
		public new LdrText screen_space(bool on = true)
		{
			m_current.m_screen_space = on;
			return this;
		}

		/// <summary>Billboard text</summary>
		public LdrText billboard(bool on = true)
		{
			m_current.m_billboard = on;
			return this;
		}

		/// <summary>3D Billboard text</summary>
		public LdrText billboard_3d(bool on = true)
		{
			m_current.m_billboard_3d = on;
			return this;
		}

		/// <summary>Background colour</summary>
		public LdrText back_colour(Colour32 col)
		{
			m_current.m_back_colour = col;
			return this;
		}

		/// <summary>The text font</summary>
		public new LdrFont font()
		{
			m_current.m_font ??= new();
			return m_current.m_font;
		}

		/// <summary>Anchor position</summary>
		public LdrText anchor(v2 anchor)
		{
			m_current.m_anchor = new(anchor);
			return this;
		}

		/// <summary>Padding</summary>
		public LdrText padding(float left, float top, float right, float bottom)
		{
			m_current.m_padding = new(left, top, right, bottom);
			return this;
		}

		/// <summary>Format string</summary>
		public LdrText format(string fmt)
		{
			m_current.m_format = fmt;
			return this;
		}

		/// <summary>CString text</summary>
		public LdrText cstring(string s)
		{
			m_current.m_text.Append(s);
			return this;
		}

		// 
		public LdrText new_block()
		{
			m_blocks.Add(m_current);
			m_current = new();
			return this;
		}

		/// <inheritdoc/>
		public override void WriteTo(IWriter res)
		{
			res.Write(EKeyword.Text, m_name, m_colour, () =>
			{
				void WriteBlock(Block block)
				{
					if (block.m_font != null)
						block.m_font.WriteTo(res);
					if (block.m_screen_space)
						res.Write(EKeyword.ScreenSpace);
					if (block.m_billboard)
						res.Write(EKeyword.Billboard);
					if (block.m_billboard_3d)
						res.Write(EKeyword.Billboard3D);
					if (block.m_back_colour != null)
						res.Write(EKeyword.BackColour, block.m_back_colour.Value);
					res.Append(block.m_anchor);
					res.Append(block.m_padding);
					if (block.m_format != null)
						res.Write(EKeyword.Format, block.m_format);

					res.Write(EKeyword.Data, $"\"{block.m_text.ToString()}\"");
				}

				foreach (var block in m_blocks)
					WriteBlock(block);
				if (m_current)
					WriteBlock(m_current);

				base.WriteTo(res);
			});
		}

	}
	public class LdrGroup : LdrBase<LdrGroup>
	{
		/// <inheritdoc/>
		public override void WriteTo(IWriter res)
		{
			res.Write(EKeyword.Group, m_name, m_colour, () =>
			{
				base.WriteTo(res);
			});
		}
	}
	public class LdrLightSource : LdrBase<LdrLightSource>
	{
		private string m_style = "Point";
		private Colour32? m_ambient = null;
		private Colour32? m_diffuse = null;
		private Colour32? m_specular = null;
		private float? m_specular_power = null;
		private v2? m_range = null;
		private v2? m_cone = null;
		private float? m_cast_shadow = null;

		public LdrLightSource style(string style)
		{
			m_style = style;
			return this;
		}
		public LdrLightSource ambient(Colour32 col)
		{
			m_ambient = col;
			return this;
		}
		public LdrLightSource diffuse(Colour32 col)
		{
			m_diffuse = col;
			return this;
		}
		public LdrLightSource specular(Colour32 col, float power)
		{
			m_specular = col;
			m_specular_power = power;
			return this;
		}
		public LdrLightSource range(float range, float falloff)
		{
			m_range = new v2(range, falloff);
			return this;
		}
		public LdrLightSource cone(float inner, float outer)
		{
			m_cone = new v2(inner, outer);
			return this;
		}
		public LdrLightSource cast_shadow(float range)
		{
			m_cast_shadow = range;
			return this;
		}

		public override void WriteTo(IWriter res)
		{
			res.Write(EKeyword.LightSource, m_name, m_colour, () =>
			{
				res.Write(EKeyword.Style, m_style);
				if (m_ambient is Colour32 a) res.Write(EKeyword.Ambient, a);
				if (m_diffuse is Colour32 d) res.Write(EKeyword.Diffuse, d);
				if (m_specular is Colour32 s && m_specular_power is float p) res.Write(EKeyword.Specular, s, p);
				if (m_range is v2 r) res.Write(EKeyword.Range, r);
				if (m_cone is v2 c) res.Write(EKeyword.Cone, c);
				if (m_cast_shadow is float cs) res.Write(EKeyword.CastShadow, cs);
				base.WriteTo(res);
			});
		}
	}
	public class LdrSeries
	{
		private Serialiser.Name m_name = new();
		private Serialiser.Colour m_colour = new();
		private string m_xaxis = string.Empty;
		private string m_yaxis = string.Empty;
		private Serialiser.Width m_width = new();
		private Serialiser.Dashed m_dashed = new();
		private Serialiser.Smooth m_smooth = new();
		private Serialiser.DataPoints m_data_points = new();

		/// <summary>Series name</summary>
		public LdrSeries name(Serialiser.Name name)
		{
			m_name = name;
			return this;
		}

		/// <summary>Series colour</summary>
		public LdrSeries colour(Serialiser.Colour colour)
		{
			m_colour = colour;
			return this;
		}

		/// <summary>X-axis expression (e.g. "C0", "CI")</summary>
		public LdrSeries xaxis(string expr)
		{
			m_xaxis = expr;
			return this;
		}

		/// <summary>Y-axis expression (e.g. "C1", "abs(C2 - C1)")</summary>
		public LdrSeries yaxis(string expr)
		{
			m_yaxis = expr;
			return this;
		}

		/// <summary>Line width</summary>
		public LdrSeries width(float w)
		{
			m_width = new(w);
			return this;
		}

		/// <summary>Dashed line pattern</summary>
		public LdrSeries dashed(v2 dash)
		{
			m_dashed = new Serialiser.Dashed(dash);
			return this;
		}

		/// <summary>Smooth the line</summary>
		public LdrSeries smooth(bool on = true)
		{
			m_smooth = new(on);
			return this;
		}

		/// <summary>Data point markers</summary>
		public LdrSeries data_points(float size, Colour32? colour = null, EPointStyle? style = null)
		{
			return data_points(new v2(size, size), colour, style);
		}
		public LdrSeries data_points(v2 size, Colour32? colour = null, EPointStyle? style = null)
		{
			m_data_points = new Serialiser.DataPoints(size, colour, style);
			return this;
		}

		// Write to 'out'
		public void WriteTo(IWriter res)
		{
			res.Write(EKeyword.Series, m_name, m_colour, () =>
			{
				if (m_xaxis.Length != 0)
					res.Write(EKeyword.XAxis, $"\"{m_xaxis}\"");
				if (m_yaxis.Length != 0)
					res.Write(EKeyword.YAxis, $"\"{m_yaxis}\"");
				res.Append(m_width, m_dashed, m_smooth, m_data_points);
			});
		}
	}
	public class LdrChart : LdrBase<LdrChart>
	{
		private string? m_filepath = null;
		private int m_dim_columns = 0;
		private int m_dim_rows = 0;
		private readonly List<double> m_data = [];
		private readonly List<LdrSeries> m_series = [];

		/// <summary>Reference an external CSV data file (mutually exclusive with data)</summary>
		public LdrChart filepath(string filepath)
		{
			m_filepath = Path_.Canonicalise(filepath);
			return this;
		}

		/// <summary>Set the data dimensions (columns, and optionally rows)</summary>
		public LdrChart dim(int columns, int rows = 0)
		{
			m_dim_columns = columns;
			m_dim_rows = rows;
			return this;
		}

		/// <summary>Add data values to the chart</summary>
		public LdrChart data(params double[] values)
		{
			m_data.AddRange(values);
			return this;
		}

		/// <summary>Add data values to the chart</summary>
		public LdrChart data(IEnumerable<double> values)
		{
			m_data.AddRange(values);
			return this;
		}

		/// <summary>Add a series to the chart</summary>
		public LdrSeries Series(Serialiser.Name? name = null, Serialiser.Colour? colour = null)
		{
			var s = new LdrSeries();
			m_series.Add(s);
			return s.name(name ?? new()).colour(colour ?? new());
		}

		/// <inheritdoc/>
		public override void WriteTo(IWriter res)
		{
			res.Write(EKeyword.Chart, m_name, m_colour, () =>
			{
				if (m_filepath != null)
				{
					res.Write(EKeyword.FilePath, $"\"{m_filepath}\"");
				}
				if (m_dim_columns != 0)
				{
					if (m_dim_rows != 0)
						res.Write(EKeyword.Dim, m_dim_columns, m_dim_rows);
					else
						res.Write(EKeyword.Dim, m_dim_columns);
				}
				if (m_data.Count != 0)
				{
					res.Write(EKeyword.Data, () =>
					{
						foreach (var value in m_data)
							res.Append(value);
					});
				}
				foreach (var series in m_series)
					series.WriteTo(res);

				base.WriteTo(res);
			});
		}
	}
	public class LdrEquation : LdrBase<LdrEquation>
	{
		private string m_equation = string.Empty;
		private int m_resolution = 0;
		private readonly List<(string name, float value)> m_params = [];
		private float? m_weight = null;

		public LdrEquation equation(string eq)
		{
			m_equation = eq;
			return this;
		}
		public LdrEquation resolution(int res)
		{
			m_resolution = res;
			return this;
		}
		public LdrEquation param(string name, float value)
		{
			m_params.Add((name, value));
			return this;
		}
		public LdrEquation weight(float w)
		{
			m_weight = w;
			return this;
		}

		public override void WriteTo(IWriter res)
		{
			res.Write(EKeyword.Equation, m_name, m_colour, () =>
			{
				res.Write(EKeyword.Data, $"\"{m_equation}\"");
				if (m_resolution != 0)
					res.Write(EKeyword.Resolution, m_resolution);
				foreach (var p in m_params)
					res.Write(EKeyword.Param, p.name, p.value);
				if (m_weight is float w)
					res.Write(EKeyword.Weight, w);
				base.WriteTo(res);
			});
		}
	}
	public class LdrCommands : LdrBase<LdrCommands>
	{
		private struct Cmd
		{
			public ECommandId m_id;
			public List<object> m_params;
		}

		private readonly List<Cmd> m_cmds = [];

		// Add objects created by this script to scene 'scene_id'
		public LdrCommands add_to_scene(int scene_id)
		{
			m_cmds.Add(new Cmd{ m_id = ECommandId.AddToScene, m_params = [scene_id] });
			return this;
		}

		// Apply a transform to an object with the given name
		public LdrCommands object_transform(string object_name, m4x4 o2w)
		{
			m_cmds.Add(new Cmd{ m_id = ECommandId.ObjectToWorld, m_params = [new Serialiser.StringWithLength(object_name), o2w] });
			return this;
		}

		/// <inheritdoc/>
		public override void WriteTo(IWriter res)
		{
			res.Write(EKeyword.Commands, () =>
			{
				foreach (var cmd in m_cmds)
				{
					res.Write(EKeyword.Data, () =>
					{
						res.Append((int)cmd.m_id);
						foreach (var p in cmd.m_params)
						{
							if (p is bool   bool_ ) { res.Append(bool_ ); continue; }
							if (p is int    int_  ) { res.Append(int_  ); continue; }
							if (p is float  float_) { res.Append(float_); continue; }
							if (p is Serialiser.StringWithLength str_  ) { res.Append(str_  ); continue; }
							if (p is v2     v2_   ) { res.Append(v2_   ); continue; }
							if (p is v4     v4_   ) { res.Append(v4_   ); continue; }
							if (p is m4x4   m4_   ) { res.Append(m4_   ); continue; }
						}
					});
				}
			});
		}
	}
	public class LdrBinaryStream : LdrBase<LdrBinaryStream>
	{
		/// <inheritdoc/>
		public override void WriteTo(IWriter res)
		{
			res.Write(EKeyword.BinaryStream);
		}
	}
	public class LdrTextStream : LdrBase<LdrTextStream>
	{
		/// <inheritdoc/>
		public override void WriteTo(IWriter res)
		{
			res.Write(EKeyword.TextStream);
		}
	}

	// DEPRECATED 
	#region DEPRECATED 
	public static class Ldr
	{
		// Notes:
		//  - The LdrBuilder is the main type for building ldr strings.
		//  - This type is basically a namespace for converting types to ldr strings. It knows when a
		//    string is necessary or how to simplify a string for a specific type.

		/// <summary>Filepath for outputting ldr script using 'LdrOut' extension</summary>
		public static string OutFile = "";

		/// <summary>Append this string to the Ldr.OutFile</summary>
		public static void LdrOut(this string s, bool app = true)
		{
			using var sw = new StreamWriter(OutFile, app);
			sw.Write(s);
		}

		/// <summary>Write an ldr string to a file</summary>
		public static void Write(string ldr_str, string filepath, bool append = false)
		{
			try
			{
				// Ensure the directory exists
				var dir = Path_.Directory(filepath);
				if (!Path_.DirExists(dir)) Directory.CreateDirectory(dir);

				// Lock, then write the file
				using (Path_.LockFile(filepath))
				using (var f = new StreamWriter(new FileStream(filepath, append ? FileMode.Append : FileMode.Create, FileAccess.Write, FileShare.Read)))
					f.Write(ldr_str);
			}
			catch (Exception ex)
			{
				Debug.WriteLine($"Failed to write Ldr script to '{filepath}'. {ex.Message}");
			}
		}

		// Type to string helpers
		public static string Col(uint col)
		{
			return col != 0xFFFFFFFF ? col.ToString("X8") : string.Empty;
		}
		public static string Col(Colour32 col)
		{
			return col != 0xFFFFFFFF ? col.ARGB.ToString("X8") : string.Empty;
		}
		public static string Col(Color col)
		{
			return Col(col.ToArgbU());
		}
		public static string Vec2(v2 vec)
		{
			Debug.Assert(Math_.IsFinite(vec));
			return $"{vec.x} {vec.y}";
		}
		public static string Vec3(v2 vec)
		{
			Debug.Assert(Math_.IsFinite(vec));
			return $"{vec.x} {vec.y} 0";
		}
		public static string Vec3(v4 vec)
		{
			Debug.Assert(Math_.IsFinite(vec));
			return $"{vec.x} {vec.y} {vec.z}";
		}
		public static string Vec4(v4 vec)
		{
			Debug.Assert(Math_.IsFinite(vec));
			return $"{vec.x} {vec.y} {vec.z} {vec.w}";
		}
		public static string Mat4x4(m4x4 mat)
		{
			Debug.Assert(Math_.IsFinite(mat));
			return $"{Vec4(mat.x)} {Vec4(mat.y)} {Vec4(mat.z)} {Vec4(mat.w)}";
		}

		// Ldr element helpers
		public static string Colour(Colour32 colour)
		{
			return $"*Colour {{{colour.ARGB:X8}}}";
		}
		public static string Position(v4 position, bool newline = false)
		{
			return
				position == v4.Zero ? string.Empty :
				position == v4.Origin ? string.Empty :
				$"*o2w{{*pos{{{Vec3(position)}}}}}{(newline ? "\n" : "")}";
		}
		public static string Position(v4? position, bool newline = false)
		{
			return position != null ? Position(position.Value, newline) : string.Empty;
		}
		public static string Transform(m4x4 o2w, bool newline = false)
		{
			return
				o2w == m4x4.Identity ? string.Empty :
				o2w.rot == m3x4.Identity ? Position(o2w.pos, newline) :
				$"*o2w{{*m4x4{{{Mat4x4(o2w)}}}}}{(newline ? "\n" : "")}";
		}
		public static string Transform(m4x4? o2w, bool newline = false)
		{
			return o2w != null ? Transform(o2w.Value, newline) : string.Empty;
		}
		public static string Transform(m4x4? o2w, v4? pos, bool newline = false)
		{
			return
				o2w != null ? Transform(o2w.Value, newline) :
				pos != null ? Position(pos.Value, newline) :
				string.Empty;
		}
		public static string Name(string? name)
		{
			return name != null ? $"*Name {{{name}}}" : string.Empty;
		}
		public static string AxisId(AxisId id)
		{
			return id.Id != EAxisId.PosZ ? $"*AxisId {{{id}}}" : string.Empty;
		}
		public static string Solid(bool solid = true)
		{
			return solid ? "*Solid" : string.Empty;
		}
		public static string Billboard3D(bool billboard = true)
		{
			return billboard ? "*Billboard3D" : string.Empty;
		}
		public static string Billboard(bool billboard = true)
		{
			return billboard ? "*Billboard" : string.Empty;
		}
		public static string ScreenSpace(bool screen_space = true)
		{
			return screen_space ? "*ScreenSpace" : string.Empty;
		}
		public static string Size(double size)
		{
			return $"*Size {{{size}}}";
		}
		public static string Scale(double scale)
		{
			return scale != 1 ? $"*Scale {{{scale}}}" : string.Empty;
		}
		public static string Width(double width)
		{
			return width != 0 ? $"*Width {{{width}}}" : string.Empty;
		}
		public static string Facets(int facets)
		{
			return $"*Facets {{{facets}}}";
		}
		public static string CornerRadius(double rad)
		{
			return rad != 0 ? $"*CornerRadius {{{rad}}}" : string.Empty;
		}
		public static string Smooth(bool smooth)
		{
			return smooth ? "*Smooth" : string.Empty;
		}
		public static string NoZTest(bool no_ztest)
		{
			return no_ztest ? "*NoZTest" : string.Empty;
		}
		public static string NoZWrite(bool no_zwrite)
		{
			return no_zwrite ? "*NoZWrite" : string.Empty;
		}

		// Types
		public enum EArrowType
		{
			Line = 0,
			Fwd = 1 << 0,
			Back = 1 << 1,
			FwdBack = Fwd | Back,
		}
	}

	/// <summary>Like StringBuilder, but for ldr strings</summary>
	public class LdrBuilder
	{
		// Notes:
		//  - Fluent style Ldr String builder.

		private readonly StringBuilder m_sb;
		public LdrBuilder(StringBuilder sb) => m_sb = sb;
		public LdrBuilder() : this(new StringBuilder()) { }
		public LdrBuilder(int capacity) : this(new StringBuilder(capacity)) { }
		public LdrBuilder(string value) : this(new StringBuilder(value)) { }
		public LdrBuilder(int capacity, int maxCapacity) : this(new StringBuilder(capacity, maxCapacity)) { }
		public LdrBuilder(string value, int capacity) : this(new StringBuilder(value, capacity)) { }
		public LdrBuilder(string value, int startIndex, int length, int capacity) : this(new StringBuilder(value, startIndex, length, capacity)) { }

		public LdrBuilder Clear()
		{
			m_sb.Clear();
			return this;
		}
		public LdrBuilder Remove(int start_index, int length)
		{
			m_sb.Remove(start_index, length);
			return this;
		}
		public LdrBuilder Append(params object[] parts)
		{
			foreach (var p in parts) Append(p);
			return this;
		}
		public LdrBuilder Append(object part)
		{
			// Use this switch to convert a type directly into a string. No adorning.
			// E.g. 'part is Colour32' just inserts "FF00FF00", not "*Colour32{FF00FF00}"
			switch (part)
			{
				case null: break;
				case string str: m_sb.Append(str); break;
				case Color col: m_sb.Append(Ldr.Col(col)); break;
				case v4 vec4: m_sb.Append(Ldr.Vec3(vec4)); break;
				case v2 vec2: m_sb.Append(Ldr.Vec2(vec2)); break;
				case m4x4 o2w: m_sb.Append(Ldr.Mat4x4(o2w)); break;
				case AxisId axisid: m_sb.Append(Ldr.AxisId(axisid.Id)); break;
				case EAxisId axisid2: m_sb.Append(Ldr.AxisId(axisid2)); break;
				case IEnumerable:
				{
					foreach (var x in (IEnumerable)part)
						Append(" ").Append(x);
					break;
				}
				default:
				{
					if (part.GetType().IsAnonymousType())
					{
						Append("{");
						foreach (var prop in part.GetType().AllProps(BindingFlags.Public | BindingFlags.Instance))
						{
							if (prop is null || prop.GetValue(part) is not object val) continue;
							Append($"*{prop.Name} {{", val, "}} ");
						}
						Append("}");
						return this;
					}
					else
					{
						m_sb.Append(part.ToString());
					}
					break;
				}
			}
			return this;
		}

		public LdrBuilder Comment(string comments)
		{
			var lines = comments.Split('\n');
			foreach (var line in lines)
				Append($"// {line}\n");
			return this;
		}

		public Scope Group()
		{
			return Group(string.Empty);
		}
		public Scope Group(string name)
		{
			return Group(name, v4.Origin);
		}
		public Scope Group(string name, Colour32 colour)
		{
			return Group(name, colour, v4.Origin);
		}
		public Scope Group(string name, v4 position)
		{
			return Group(name, 0xFFFFFFFF, position);
		}
		public Scope Group(string name, Colour32 colour, v4 position)
		{
			return Group(name, colour, m4x4.Translation(position));
		}
		public Scope Group(string name, m4x4 transform)
		{
			return Group(name, 0xFFFFFFFF, transform);
		}
		public Scope Group(string name, Colour32 colour, m4x4 transform)
		{
			return Scope.Create(
				() => GroupOpen(name, colour),
				() => GroupClose(transform)
				);
		}
		public LdrBuilder GroupOpen(string name)
		{
			return GroupOpen(name, 0xFFFFFFFF);
		}
		public LdrBuilder GroupOpen(string name, Colour32 colour)
		{
			return Append("*Group ", name, " ", colour, " {\n");
		}
		public LdrBuilder GroupClose()
		{
			return GroupClose(m4x4.Identity);
		}
		public LdrBuilder GroupClose(m4x4 transform)
		{
			return Append(Ldr.Transform(transform, newline: true), "}\n");
		}

		public LdrBuilder Line(v4 start, v4 end)
		{
			return Line(Color.White, start, end);
		}
		public LdrBuilder Line(Colour32 colour, v4 start, v4 end)
		{
			return Line(string.Empty, colour, start, end);
		}
		public LdrBuilder Line(string name, Colour32 colour, v4 start, v4 end)
		{
			return Append("*Line ", name, " ", colour, " {", start, " ", end, "}\n");
		}
		public LdrBuilder Line(string name, Colour32 colour, double width, bool smooth, IEnumerable<v4> points)
		{
			if (!points.Any()) return this;
			return Append("*LineStrip ", name, " ", colour, " {", Ldr.Width(width), Ldr.Smooth(smooth), points.Select(x => Ldr.Vec3(x)), "}\n");
		}
		public LdrBuilder Line(string name, Colour32 colour, double width, bool smooth, Func<int, v4?> points)
		{
			var idx = 0;
			Append("*LineStrip ", name, " ", colour, " {", Ldr.Width(width), Ldr.Smooth(smooth));
			for (v4? pt; (pt = points(idx++)) != null;) Append(Ldr.Vec3(pt.Value));
			Append("}\n");
			return this;
		}
		public LdrBuilder LineD(string name, Colour32 colour, v4 start, v4 direction)
		{
			return LineD(name, colour, start, direction, 0);
		}
		public LdrBuilder LineD(string name, Colour32 colour, v4 start, v4 direction, double width)
		{
			return Append("*LineD ", name, " ", colour, " {", Ldr.Width(width), start, " ", direction, "}");
		}

		public LdrBuilder Arrow()
		{
			return Arrow(string.Empty, Colour32.White, Ldr.EArrowType.Fwd, 10f, true, new[] { v4.Origin, v4.XAxis.w1 });
		}
		public LdrBuilder Arrow(string name, Colour32 colour, Ldr.EArrowType type, double width, bool smooth, IEnumerable<v4> points)
		{
			if (!points.Any()) return this;
			return Append("*Arrow ", name, " ", colour, " {", type.ToString(), points.Select(x => Ldr.Vec3(x)), Ldr.Width(width), Ldr.Smooth(smooth), "}\n");
		}
		public LdrBuilder Arrow(string name, Colour32 colour, Ldr.EArrowType type, double width, bool smooth, Func<int, v4?> points)
		{
			var idx = 0;
			Append("*Arrow ", name, " ", colour, " {");
			for (v4? pt; (pt = points(idx++)) != null;) Append(Ldr.Vec3(pt.Value));
			Append(Ldr.Width(width), Ldr.Smooth(smooth), type.ToString(), "}\n");
			return this;
		}

		public LdrBuilder Grid(Colour32 colour, AxisId axis_id, int width, int height)
		{
			return Grid(string.Empty, colour, axis_id, width, height);
		}
		public LdrBuilder Grid(string name, Colour32 colour, AxisId axis_id, int width, int height)
		{
			return Grid(name, colour, axis_id, width, height, width, height, v4.Origin);
		}
		public LdrBuilder Grid(string name, Colour32 colour, AxisId axis_id, int width, int height, int wdiv, int hdiv, v4 position)
		{
			return Append("*Grid ", name, " ", colour, " {", width, " ", height, " ", wdiv, " ", hdiv, " ", axis_id, " ", Ldr.Position(position), "}\n");
		}

		public LdrBuilder Box(m4x4? o2w = null, v4? pos = null)
		{
			return Box(string.Empty, Color.White, o2w, pos);
		}
		public LdrBuilder Box(Colour32 colour, double size, m4x4? o2w = null, v4? pos = null)
		{
			return Box(string.Empty, colour, size, o2w, pos);
		}
		public LdrBuilder Box(Colour32 colour, double sx, double sy, double sz, m4x4? o2w = null, v4? pos = null)
		{
			return Box(string.Empty, colour, sx, sy, sz, o2w, pos);
		}
		public LdrBuilder Box(string name, Colour32 colour, m4x4? o2w = null, v4? pos = null)
		{
			return Box(name, colour, 1f, o2w, pos);
		}
		public LdrBuilder Box(string name, Colour32 colour, double size, m4x4? o2w = null, v4? pos = null)
		{
			return Append("*Box ", name, " ", colour, " {", size, " ", Ldr.Transform(o2w, pos), "}\n");
		}
		public LdrBuilder Box(string name, Colour32 colour, double sx, double sy, double sz, m4x4? o2w = null, v4? pos = null)
		{
			return Append("*Box ", name, " ", colour, " {", sx, " ", sy, " ", sz, " ", Ldr.Transform(o2w, pos), "}\n");
		}
		public LdrBuilder Box(string name, Colour32 colour, v4 dim, m4x4? o2w = null, v4? pos = null)
		{
			return Append("*Box ", name, " ", colour, " {", dim.x, " ", dim.y, " ", dim.z, " ", Ldr.Transform(o2w, pos), "}\n");
		}

		public LdrBuilder Sphere()
		{
			return Sphere(string.Empty, Color.White);
		}
		public LdrBuilder Sphere(string name, Colour32 colour)
		{
			return Sphere(name, colour, 1f);
		}
		public LdrBuilder Sphere(string name, Colour32 colour, double radius)
		{
			return Sphere(name, colour, radius, v4.Origin);
		}
		public LdrBuilder Sphere(string name, Colour32 colour, double radius, v4 position)
		{
			return Append("*Sphere ", name, " ", colour, " {", radius, " ", Ldr.Position(position), "}\n");
		}

		public LdrBuilder Cylinder(Colour32 colour, AxisId axis_id, double height, double radius, m4x4? o2w = null, v4? pos = null)
		{
			return Cylinder(string.Empty, colour, axis_id, height, radius, o2w, pos);
		}
		public LdrBuilder Cylinder(string name, Colour32 colour, AxisId axis_id, double height, double radius, m4x4? o2w = null, v4? pos = null)
		{
			return Append("*Cylinder ", name, " ", colour, " {", height, " ", radius, " ", axis_id, " ", Ldr.Transform(o2w, pos), "}\n");
		}

		public LdrBuilder Circle()
		{
			return Circle(string.Empty, Color.White, 3, true);
		}
		public LdrBuilder Circle(string name, Colour32 colour, AxisId axis_id, bool solid)
		{
			return Circle(name, colour, axis_id, solid, v4.Origin);
		}
		public LdrBuilder Circle(string name, Colour32 colour, AxisId axis_id, bool solid, double radius)
		{
			return Circle(name, colour, axis_id, solid, radius, v4.Origin);
		}
		public LdrBuilder Circle(string name, Colour32 colour, AxisId axis_id, bool solid, v4 position)
		{
			return Circle(name, colour, axis_id, solid, 1f, position);
		}
		public LdrBuilder Circle(string name, Colour32 colour, AxisId axis_id, bool solid, double radius, v4 position)
		{
			return Append("*Circle ", name, " ", colour, " {", radius, " ", axis_id, " ", Ldr.Solid(solid), " ", Ldr.Position(position), "}\n");
		}

		public LdrBuilder Ellipse()
		{
			return Ellipse(string.Empty, Color.White, 3, true, 1f, 0.5f);
		}
		public LdrBuilder Ellipse(string name, Colour32 colour, AxisId axis_id, bool solid, double radiusx, double radiusy)
		{
			return Ellipse(name, colour, axis_id, solid, radiusx, radiusy, v4.Origin);
		}
		public LdrBuilder Ellipse(string name, Colour32 colour, AxisId axis_id, bool solid, double radiusx, double radiusy, v4 position)
		{
			return Append("*Circle ", name, " ", colour, " {", radiusx, " ", radiusy, " ", axis_id, " ", Ldr.Solid(solid), " ", Ldr.Position(position), "}\n");
		}

		public LdrBuilder Pie()
		{
			return Pie(string.Empty, Color.White, 3, true, 0f, 45f);
		}
		public LdrBuilder Pie(string name, Colour32 colour, AxisId axis_id, bool solid, double ang0, double ang1)
		{
			return Pie(name, colour, axis_id, solid, ang0, ang1, v4.Origin);
		}
		public LdrBuilder Pie(string name, Colour32 colour, AxisId axis_id, bool solid, double ang0, double ang1, v4 position)
		{
			return Pie(name, colour, axis_id, solid, ang0, ang1, 0f, 1f, position);
		}
		public LdrBuilder Pie(string name, Colour32 colour, AxisId axis_id, bool solid, double ang0, double ang1, double rad0, double rad1, v4 position)
		{
			return Pie(name, colour, axis_id, solid, ang0, ang1, rad0, rad1, 1f, 1f, 40, position);
		}
		public LdrBuilder Pie(string name, Colour32 colour, AxisId axis_id, bool solid, double ang0, double ang1, double rad0, double rad1, double sx, double sy, int facets, v4 position)
		{
			return Append("*Pie ", name, " ", colour, " {", ang0, " ", ang1, " ", rad0, " ", rad1, " ", axis_id, " ", Ldr.Solid(solid), " ", Ldr.Facets(facets), " *Scale ", sx, " ", sy, " ", Ldr.Position(position), "}\n");
		}

		public LdrBuilder Rect()
		{
			return Rect(string.Empty, Color.White, 3, 1f, 1f, false, v4.Origin);
		}
		public LdrBuilder Rect(string name, Colour32 colour, AxisId axis_id, double width, double height, bool solid, v4 position)
		{
			return Append("*Rect ", name, " ", colour, " {", width, " ", height, " ", axis_id, " ", Ldr.Solid(solid), " ", Ldr.Position(position), "}\n");
		}
		public LdrBuilder Rect(string name, Colour32 colour, AxisId axis_id, double width, double height, bool solid, double corner_radius, v4 position)
		{
			return Append("*Rect ", name, " ", colour, " {", width, " ", height, " ", axis_id, " ", Ldr.Solid(solid), " ", Ldr.CornerRadius(corner_radius), " ", Ldr.Position(position), "}\n");
		}

		public LdrBuilder Triangle()
		{
			return Triangle(string.Empty, Colour32.White, v4.Origin, v4.XAxis.w1, v4.YAxis.w1);
		}
		public LdrBuilder Triangle(string name, Colour32 colour, v4 a, v4 b, v4 c, m4x4? o2w = null)
		{
			return Append("*Triangle ", name, " ", colour, " {", a, " ", b, " ", c, " ", Ldr.Transform(o2w), "}");
		}
		public LdrBuilder Triangle(string name, Colour32 colour, IEnumerable<v4> pts, m4x4? o2w = null)
		{
			return Append("*Triangle ", name, " ", colour, " {", pts, Ldr.Transform(o2w), "}");
		}

		public LdrBuilder Quad()
		{
			return Quad(string.Empty, Color.White);
		}
		public LdrBuilder Quad(Colour32 colour)
		{
			return Quad(string.Empty, colour);
		}
		public LdrBuilder Quad(Colour32 colour, v4 tl, v4 tr, v4 br, v4 bl)
		{
			return Quad(string.Empty, colour, tl, tr, br, bl);
		}
		public LdrBuilder Quad(string name, Colour32 colour)
		{
			return Quad(name, colour, new v4(0, 1, 0, 1), new v4(1, 1, 0, 1), new v4(1, 0, 0, 1), new v4(0, 0, 0, 1));
		}
		public LdrBuilder Quad(string name, Colour32 colour, v4 tl, v4 tr, v4 br, v4 bl)
		{
			return Quad(name, colour, tl, tr, br, bl, v4.Origin);
		}
		public LdrBuilder Quad(string name, Colour32 colour, v4 tl, v4 tr, v4 br, v4 bl, v4 position)
		{
			return Append("*Quad ", name, " ", colour, " {", bl, " ", br, " ", tr, " ", tl, " ", Ldr.Position(position), "}\n");
		}
		public LdrBuilder Quad(string name, Colour32 colour, AxisId axis_id, double w, double h, v4 position)
		{
			return Rect(name, colour, axis_id, w, h, true, position);
		}

		public LdrBuilder Axis()
		{
			return Axis(m4x4.Identity);
		}
		public LdrBuilder Axis(m3x4 basis)
		{
			return Axis(new m4x4(basis, v4.Origin));
		}
		public LdrBuilder Axis(m4x4 basis)
		{
			return Axis(string.Empty, Color_.FromArgb(0xFFFFFFFF), basis);
		}
		public LdrBuilder Axis(string name, Colour32 colour, m3x4 basis)
		{
			return Axis(name, colour, new m4x4(basis, v4.Origin));
		}
		public LdrBuilder Axis(string name, Colour32 colour, m4x4 basis)
		{
			return Axis(name, colour, basis, 0.1f);
		}
		public LdrBuilder Axis(string name, Colour32 colour, m3x4 basis, double scale)
		{
			return Axis(name, colour, new m4x4(basis, v4.Origin), scale);
		}
		public LdrBuilder Axis(string name, Colour32 colour, m4x4 basis, double scale)
		{
			return Append("*Matrix3x3 ", name, " ", colour, " {", basis.x * scale, " ", basis.y * scale, " ", basis.z * scale, " ", Ldr.Position(basis.pos), "}\n");
		}

		public LdrBuilder Ribbon()
		{
			return Ribbon(string.Empty, Colour32.White, new[] { v4.Origin, v4.XAxis.w1 }, EAxisId.PosZ, 3f, false);
		}
		public LdrBuilder Ribbon(string name, Colour32 colour, IEnumerable<v4> points, AxisId axis_id, double width, bool smooth, m4x4? o2w = null)
		{
			return Append("*Ribbon ", name, " ", colour, " {", points, " ", axis_id, Ldr.Width(width), Ldr.Smooth(smooth), Ldr.Transform(o2w), "}\n");
		}

		public LdrBuilder Mesh(string name, Colour32 colour, IList<v4>? verts, IList<v4>? normals = null, IList<Colour32>? colours = null, IList<v2>? tex = null, IList<ushort>? faces = null, IList<ushort>? lines = null, IList<ushort>? tetra = null, bool generate_normals = false, v4? position = null)
		{
			Append("*Mesh ", name, " ", colour, " {\n");
			if (verts != null) Append("*Verts {").Append(verts.Select(Ldr.Vec3)).Append("}\n");
			if (normals != null) Append("*Normals {").Append(normals.Select(Ldr.Vec3)).Append("}\n");
			if (colours != null) Append("*Colours {").Append(colours.Select(Ldr.Col)).Append("}\n");
			if (tex != null) Append("*TexCoords {").Append(tex.Select(Ldr.Vec2)).Append("}\n");
			if (verts != null && faces != null) { Debug.Assert(faces.All(i => i >= 0 && i < verts.Count)); Append("*Faces {").Append(faces).Append("}\n"); }
			if (verts != null && lines != null) { Debug.Assert(lines.All(i => i >= 0 && i < verts.Count)); Append("*Lines {").Append(lines).Append("}\n"); }
			if (verts != null && tetra != null) { Debug.Assert(tetra.All(i => i >= 0 && i < verts.Count)); Append("*Tetra {").Append(tetra).Append("}\n"); }
			if (generate_normals) Append("*GenerateNormals\n");
			if (position != null) Append(Ldr.Position(position.Value));
			Append("}\n");
			return this;
		}

		public LdrBuilder Graph(string name, AxisId axis_id, bool smooth, IEnumerable<v4> data)
		{
			var bbox = BBox.Reset;
			foreach (var d in data)
				BBox.Grow(ref bbox, d);

			using var grp = Group(name);
			Grid(string.Empty, 0xFFAAAAAA, axis_id, (int)(bbox.SizeX * 1.1), (int)(bbox.SizeY * 1.1), 50, 50, new v4(bbox.Centre.x, bbox.Centre.y, 0f, 1f));
			Line("data", 0xFFFFFFFF, 1, smooth, data);
			return this;
		}

		public LdrTextBuilder Text(string name, Colour32 colour)
		{
			return new LdrTextBuilder(this, name, colour);
		}

		public LdrBuilder Instance(string name, Colour32 colour, m4x4? o2w = null)
		{
			return Append("*Instance ", name, " ", colour, " {", Ldr.Transform(o2w), "}\n");
		}

		public override string ToString()
		{
			return m_sb.ToString();
		}
		public void ToFile(string filepath, bool append = false)
		{
			Ldr.Write(ToString(), filepath, append);
		}
		public static implicit operator string(LdrBuilder ldr) => ldr.ToString();
	}

	public class LdrTextBuilder
	{
		private readonly LdrBuilder m_builder;
		internal LdrTextBuilder(LdrBuilder builder, string name, Colour32 colour)
		{
			m_builder = builder;
			m_builder.Append("*Text ", name, " ", colour, " {");
		}
		public LdrTextBuilder String(string text)
		{
			m_builder.Append($"\"{text}\"");
			return this;
		}
		public LdrTextBuilder CString(string cstring)
		{
			m_builder.Append($"*CString \"{cstring}\"");
			return this;
		}
		public LdrTextBuilder Bkgd(Colour32 colour)
		{
			m_builder.Append("*BackColour {", colour, "}");
			return this;
		}
		public LdrTextBuilder Font(string? font_name, double size, Colour32 colour)
		{
			m_builder.Append("*Font {", Ldr.Name(font_name), Ldr.Size(size), Ldr.Colour(colour), "}");
			return this;
		}
		public LdrTextBuilder Font(double size, Colour32 colour)
		{
			m_builder.Append("*Font {", Ldr.Size(size), Ldr.Colour(colour), "}");
			return this;
		}
		public LdrTextBuilder Billboard3D()
		{
			m_builder.Append(Ldr.Billboard3D(true));
			return this;
		}
		public LdrTextBuilder Billboard()
		{
			m_builder.Append(Ldr.Billboard(true));
			return this;
		}
		public LdrTextBuilder ScreenSpace()
		{
			m_builder.Append(Ldr.ScreenSpace(true));
			return this;
		}
		public LdrTextBuilder Dim(v2 dimensions)
		{
			m_builder.Append($"*Dim {{{dimensions.x} {dimensions.y}}}");
			return this;
		}
		public LdrTextBuilder AxisId(AxisId axis)
		{
			m_builder.Append(Ldr.AxisId(axis));
			return this;
		}
		public LdrTextBuilder Anchor(v2 anchor)
		{
			m_builder.Append($"*Anchor {{{anchor.x} {anchor.y}}}");
			return this;
		}
		public LdrTextBuilder Format(string h_align, string v_align, bool wrap)
		{
			m_builder.Append($"*Format {{{h_align} {v_align} {(wrap ? "Wrap" : "")}}}");
			return this;
		}
		public LdrTextBuilder NoZTest()
		{
			m_builder.Append(Ldr.NoZTest(true));
			return this;
		}
		public LdrTextBuilder Append(params object[] parts)
		{
			m_builder.Append(parts);
			return this;
		}
		public LdrBuilder End()
		{
			return m_builder.Append("}\n");
		}
	}
	#endregion
}

#if PR_UNITTESTS
namespace Rylogic.UnitTests
{
	[TestFixture]
	public class TestLdrawBuilderText
	{
		[Test]
		public void TestTextBox()
		{
			var builder = new LDraw.Builder();
			builder.Box("b", new Colour32(0xFF00FF00)).dim(1).o2w(m4x4.Identity);
			var str = builder.ToString();
			Assert.Equal(str, "*Box b FF00FF00 {*Data {1 1 1}}");
		}

		[Test]
		public void TestLine()
		{
			var builder = new LDraw.Builder();
			builder.Line("a", 0xFF00FF00).style(LDraw.ELineStyle.LineStrip).line_to(v4.ZAxis.w1);
			var str = builder.ToString();
			Assert.Equal(str, "*Line a FF00FF00 {*Style {LineStrip} *Data {0 0 1 0 0 1}}");
		}

		[Test]
		public void TestGroup()
		{
			var builder = new LDraw.Builder();
			builder.Group("g", 0xFF00FF00u);
			var str = builder.ToString();
			Assert.Equal(str, "*Group g FF00FF00 {}");
		}

		[Test]
		public void TestInstance()
		{
			var builder = new LDraw.Builder();
			builder.Instance("i", 0xFFFF0000u).inst("model_ref");
			var str = builder.ToString();
			Assert.Equal(str, "*Instance i FFFF0000 {*Data {model_ref}}");
		}

		[Test]
		public void TestTextObj()
		{
			var builder = new LDraw.Builder();
			builder.Text("t", 0xFF00FF00u).text("hello");
			var str = builder.ToString();
			Assert.Equal(str, "*Text t FF00FF00 {*Data {\"hello\"}}");
		}

		[Test]
		public void TestLightSource()
		{
			var builder = new LDraw.Builder();
			builder.LightSource("light", 0xFF00FF00u).style("Spot").diffuse(new Colour32(0xFF0000FF));
			var str = builder.ToString();
			Assert.Equal(str, "*LightSource light FF00FF00 {*Style {Spot} *Diffuse {FF0000FF}}");
		}

		[Test]
		public void TestPoint()
		{
			var builder = new LDraw.Builder();
			builder.Point("p", 0xFF00FF00u).style(LDraw.EPointStyle.Circle).pt(new v4(1, 2, 3, 1));
			var str = builder.ToString();
			Assert.Equal(str, "*Point p FF00FF00 {*Style {Circle} *Data {1 2 3}}");
		}

		[Test]
		public void TestLineBox()
		{
			var builder = new LDraw.Builder();
			builder.LineBox("lb", 0xFF00FF00u).dim(2, 3, 4);
			var str = builder.ToString();
			Assert.Equal(str, "*LineBox lb FF00FF00 {*Data {2 3 4}}");
		}

		[Test]
		public void TestGrid()
		{
			var builder = new LDraw.Builder();
			builder.Grid("grid", 0xFF00FF00u).wh(10, 10).divisions(5, 5);
			var str = builder.ToString();
			Assert.Equal(str, "*Grid grid FF00FF00 {*Data {10 10 5 5}}");
		}

		[Test]
		public void TestCoordFrame()
		{
			var builder = new LDraw.Builder();
			builder.CoordFrame("cf", 0xFF00FF00u).scale(2);
			var str = builder.ToString();
			Assert.Equal(str, "*CoordFrame cf FF00FF00 {*Scale {2}}");
		}

		[Test]
		public void TestCircle()
		{
			var builder = new LDraw.Builder();
			builder.Circle("c", 0xFF00FF00u).radius(5).solid(true);
			var str = builder.ToString();
			Assert.Equal(str, "*Circle c FF00FF00 {*Data {5} *Solid {true}}");
		}

		[Test]
		public void TestPie()
		{
			var builder = new LDraw.Builder();
			builder.Pie("pie", 0xFF00FF00u).pie(0, 90, 0.5f, 1.5f);
			var str = builder.ToString();
			Assert.Equal(str, "*Pie pie FF00FF00 {*Data {0 90 0.5 1.5}}");
		}

		[Test]
		public void TestRect()
		{
			var builder = new LDraw.Builder();
			builder.Rect("r", 0xFF00FF00u).wh(3, 2);
			var str = builder.ToString();
			Assert.Equal(str, "*Rect r FF00FF00 {*Data {3 2}}");
		}

		[Test]
		public void TestPolygon()
		{
			var builder = new LDraw.Builder();
			builder.Polygon("pg", 0xFF00FF00u).pt(0, 0).pt(1, 0).pt(0.5f, 1);
			var str = builder.ToString();
			Assert.Equal(str, "*Polygon pg FF00FF00 {*Data {0 0 1 0 0.5 1}}");
		}

		[Test]
		public void TestTriangle()
		{
			var builder = new LDraw.Builder();
			builder.Triangle("tri", 0xFF00FF00u).tri(v4.Origin, new v4(1, 0, 0, 1), new v4(0, 1, 0, 1));
			var str = builder.ToString();
			Assert.Equal(str, "*Triangle tri FF00FF00 {*Data {0 0 0 1 0 0 0 1 0}}");
		}

		[Test]
		public void TestQuad()
		{
			var builder = new LDraw.Builder();
			builder.Quad("q", 0xFF00FF00u).quad(new v4(0, 0, 0, 1), new v4(1, 0, 0, 1), new v4(1, 1, 0, 1), new v4(0, 1, 0, 1));
			var str = builder.ToString();
			Assert.Equal(str, "*Quad q FF00FF00 {*Data {0 0 0 1 0 0 1 1 0 0 1 0}}");
		}

		[Test]
		public void TestPlane()
		{
			var builder = new LDraw.Builder();
			builder.Plane("pl", 0xFF00FF00u).wh(5, 5);
			var str = builder.ToString();
			Assert.Equal(str, "*Plane pl FF00FF00 {*Data {5 5}}");
		}

		[Test]
		public void TestRibbon()
		{
			var builder = new LDraw.Builder();
			builder.Ribbon("rib", 0xFF00FF00u).pt(new v4(0, 0, 0, 1)).pt(new v4(1, 0, 0, 1)).pt(new v4(2, 0, 0, 1)).width(0.5f);
			var str = builder.ToString();
			Assert.Equal(str, "*Ribbon rib FF00FF00 {*Data {0 0 0 1 0 0 2 0 0} *Width {0.5}}");
		}

		[Test]
		public void TestBoxList()
		{
			var builder = new LDraw.Builder();
			builder.BoxList("bl", 0xFF00FF00u).box(1, 1, 1, 0, 0, 0).box(2, 2, 2, 3, 0, 0);
			var str = builder.ToString();
			Assert.Equal(str, "*BoxList bl FF00FF00 {*Data {1 1 1 0 0 0 2 2 2 3 0 0}}");
		}

		[Test]
		public void TestFrustum()
		{
			var builder = new LDraw.Builder();
			builder.Frustum("fr", 0xFF00FF00u).wh(4, 3).nf(1, 100);
			var str = builder.ToString();
			Assert.Equal(str, "*FrustumWH fr FF00FF00 {*Data {4 3 1 100}}");
		}

		[Test]
		public void TestSphere()
		{
			var builder = new LDraw.Builder();
			builder.Sphere("s", 0xFF00FF00u).radius(3);
			var str = builder.ToString();
			Assert.Equal(str, "*Sphere s FF00FF00 {*Data {3 3 3}}");
		}

		[Test]
		public void TestCylinder()
		{
			var builder = new LDraw.Builder();
			builder.Cylinder("cyl", 0xFF00FF00u).cylinder(3, 1);
			var str = builder.ToString();
			Assert.Equal(str, "*Cylinder cyl FF00FF00 {*Data {3 1 1}}");
		}

		[Test]
		public void TestCone()
		{
			var builder = new LDraw.Builder();
			builder.Cone("cn", 0xFF00FF00u).angle(30).height(5);
			var str = builder.ToString();
			Assert.Equal(str, "*Cone cn FF00FF00 {*Data {30 0 5}}");
		}

		[Test]
		public void TestTube()
		{
			var builder = new LDraw.Builder();
			builder.Tube("tube", 0xFF00FF00u).cross_section_round(0.5f).pt(new v4(0, 0, 0, 1)).pt(new v4(1, 0, 0, 1)).pt(new v4(2, 1, 0, 1));
			var str = builder.ToString();
			Assert.Equal(str, "*Tube tube FF00FF00 {*CrossSection {*Round {0.5 0.5}} *Data {0 0 0 1 0 0 2 1 0}}");
		}

		[Test]
		public void TestMesh()
		{
			var builder = new LDraw.Builder();
			builder.Mesh("m", 0xFF00FF00u).vert(0, 0, 0).vert(1, 0, 0).vert(0, 1, 0).face(0, 1, 2);
			var str = builder.ToString();
			Assert.Equal(str, "*Mesh m FF00FF00 {*Verts {0 0 0 1 0 0 0 1 0} *Faces {0 1 2}}");
		}

		[Test]
		public void TestConvexHull()
		{
			var builder = new LDraw.Builder();
			builder.ConvexHull("ch", 0xFF00FF00u).vert(0, 0, 0).vert(1, 0, 0).vert(0, 1, 0).vert(0, 0, 1);
			var str = builder.ToString();
			Assert.Equal(str, "*ConvexHull ch FF00FF00 {*Verts {0 0 0 1 0 0 0 1 0 0 0 1}}");
		}

		[Test]
		public void TestChart()
		{
			var builder = new LDraw.Builder();
			var chart = builder.Chart("chart", 0xFF00FF00u).dim(2).data(0, 1, 1, 2, 2, 3);
			chart.Series("plot", 0xFF0000FFu).xaxis("C0").yaxis("C1");
			var str = builder.ToString();
			Assert.Equal(str, "*Chart chart FF00FF00 {*Dim {2} *Data {0 1 1 2 2 3} *Series plot FF0000FF {*XAxis {\"C0\"} *YAxis {\"C1\"}}}");
		}

		[Test]
		public void TestEquation()
		{
			var builder = new LDraw.Builder();
			builder.Equation("eq", 0xFF00FF00u).equation("sin(x)").resolution(100);
			var str = builder.ToString();
			Assert.Equal(str, "*Equation eq FF00FF00 {*Data {\"sin(x)\"} *Resolution {100}}");
		}
	}

	[TestFixture]
	public class TestLdrawBulderBinary
	{
		[Test]
		public void TestTextBox()
		{
			var builder = new LDraw.Builder();
			builder.Box("b", 0xFF00FF00).dim(1).o2w(m4x4.Identity);
			var mem = builder.ToBinary().ToArray();
			Assert.Equal(mem.Length, 49);
		}

		[Test]
		public void TestLine()
		{
			var builder = new LDraw.Builder();
			builder.Line("a", 0xFF00FF00).style(LDraw.ELineStyle.LineStrip).line_to(v4.ZAxis.w1);
			var mem = builder.ToBinary().ToArray();
			Assert.True(mem.Length > 0);
		}

		[Test]
		public void TestArrow()
		{
			var builder = new LDraw.Builder();
			builder.Line("a", 0xFF00FF00).arrow(LDraw.EArrowType.Fwd).strip(v4.Origin).line_to(v4.ZAxis.w1);
			var mem = builder.ToBinary().ToArray();
			Assert.True(mem.Length > 0);
		}

		[Test]
		public void TestCommands()
		{
			var builder = new LDraw.Builder();
			builder.Box("b", 0xFF00FF00).dim(1);
			builder.Command()
				.add_to_scene(0)
				.object_transform("b", m4x4.Transform(v4.ZAxis, 0.3f, v4.Origin));
			var mem = builder.ToBinary();
			Assert.True(mem.Length > 0);
		}

		[Test]
		public void TestBinaryGroup()
		{
			var builder = new LDraw.Builder();
			builder.Group("g", 0xFF00FF00u);
			var mem = builder.ToBinary().ToArray();
			Assert.True(mem.Length > 0);
		}

		[Test]
		public void TestBinaryInstance()
		{
			var builder = new LDraw.Builder();
			builder.Instance("i", 0xFFFF0000u).inst("model_ref");
			var mem = builder.ToBinary().ToArray();
			Assert.True(mem.Length > 0);
		}

		[Test]
		public void TestBinaryText()
		{
			var builder = new LDraw.Builder();
			builder.Text("t", 0xFF00FF00u).text("hello");
			var mem = builder.ToBinary().ToArray();
			Assert.True(mem.Length > 0);
		}

		[Test]
		public void TestBinaryLightSource()
		{
			var builder = new LDraw.Builder();
			builder.LightSource("light", 0xFF00FF00u).style("Spot");
			var mem = builder.ToBinary().ToArray();
			Assert.True(mem.Length > 0);
		}

		[Test]
		public void TestBinaryPoint()
		{
			var builder = new LDraw.Builder();
			builder.Point("p", 0xFF00FF00u).pt(new v4(1, 2, 3, 1));
			var mem = builder.ToBinary().ToArray();
			Assert.True(mem.Length > 0);
		}

		[Test]
		public void TestBinaryLineBox()
		{
			var builder = new LDraw.Builder();
			builder.LineBox("lb", 0xFF00FF00u).dim(2, 3, 4);
			var mem = builder.ToBinary().ToArray();
			Assert.True(mem.Length > 0);
		}

		[Test]
		public void TestBinaryGrid()
		{
			var builder = new LDraw.Builder();
			builder.Grid("grid", 0xFF00FF00u).wh(10, 10).divisions(5, 5);
			var mem = builder.ToBinary().ToArray();
			Assert.True(mem.Length > 0);
		}

		[Test]
		public void TestBinaryCoordFrame()
		{
			var builder = new LDraw.Builder();
			builder.CoordFrame("cf", 0xFF00FF00u).scale(2);
			var mem = builder.ToBinary().ToArray();
			Assert.True(mem.Length > 0);
		}

		[Test]
		public void TestBinaryCircle()
		{
			var builder = new LDraw.Builder();
			builder.Circle("c", 0xFF00FF00u).radius(5);
			var mem = builder.ToBinary().ToArray();
			Assert.True(mem.Length > 0);
		}

		[Test]
		public void TestBinaryPie()
		{
			var builder = new LDraw.Builder();
			builder.Pie("pie", 0xFF00FF00u).pie(0, 90, 0.5f, 1.5f);
			var mem = builder.ToBinary().ToArray();
			Assert.True(mem.Length > 0);
		}

		[Test]
		public void TestBinaryRect()
		{
			var builder = new LDraw.Builder();
			builder.Rect("r", 0xFF00FF00u).wh(3, 2);
			var mem = builder.ToBinary().ToArray();
			Assert.True(mem.Length > 0);
		}

		[Test]
		public void TestBinaryPolygon()
		{
			var builder = new LDraw.Builder();
			builder.Polygon("pg", 0xFF00FF00u).pt(0, 0).pt(1, 0).pt(0.5f, 1);
			var mem = builder.ToBinary().ToArray();
			Assert.True(mem.Length > 0);
		}

		[Test]
		public void TestBinaryTriangle()
		{
			var builder = new LDraw.Builder();
			builder.Triangle("tri", 0xFF00FF00u).tri(v4.Origin, new v4(1, 0, 0, 1), new v4(0, 1, 0, 1));
			var mem = builder.ToBinary().ToArray();
			Assert.True(mem.Length > 0);
		}

		[Test]
		public void TestBinaryQuad()
		{
			var builder = new LDraw.Builder();
			builder.Quad("q", 0xFF00FF00u).quad(new v4(0, 0, 0, 1), new v4(1, 0, 0, 1), new v4(1, 1, 0, 1), new v4(0, 1, 0, 1));
			var mem = builder.ToBinary().ToArray();
			Assert.True(mem.Length > 0);
		}

		[Test]
		public void TestBinaryPlane()
		{
			var builder = new LDraw.Builder();
			builder.Plane("pl", 0xFF00FF00u).wh(5, 5);
			var mem = builder.ToBinary().ToArray();
			Assert.True(mem.Length > 0);
		}

		[Test]
		public void TestBinaryRibbon()
		{
			var builder = new LDraw.Builder();
			builder.Ribbon("rib", 0xFF00FF00u).pt(new v4(0, 0, 0, 1)).pt(new v4(1, 0, 0, 1)).pt(new v4(2, 0, 0, 1)).width(0.5f);
			var mem = builder.ToBinary().ToArray();
			Assert.True(mem.Length > 0);
		}

		[Test]
		public void TestBinaryBox()
		{
			var builder = new LDraw.Builder();
			builder.Box("b", 0xFF00FF00u).dim(2, 3, 4);
			var mem = builder.ToBinary().ToArray();
			Assert.True(mem.Length > 0);
		}

		[Test]
		public void TestBinaryBoxList()
		{
			var builder = new LDraw.Builder();
			builder.BoxList("bl", 0xFF00FF00u).box(1, 1, 1, 0, 0, 0).box(2, 2, 2, 3, 0, 0);
			var mem = builder.ToBinary().ToArray();
			Assert.True(mem.Length > 0);
		}

		[Test]
		public void TestBinaryFrustum()
		{
			var builder = new LDraw.Builder();
			builder.Frustum("fr", 0xFF00FF00u).wh(4, 3).nf(1, 100);
			var mem = builder.ToBinary().ToArray();
			Assert.True(mem.Length > 0);
		}

		[Test]
		public void TestBinarySphere()
		{
			var builder = new LDraw.Builder();
			builder.Sphere("s", 0xFF00FF00u).radius(3);
			var mem = builder.ToBinary().ToArray();
			Assert.True(mem.Length > 0);
		}

		[Test]
		public void TestBinaryCylinder()
		{
			var builder = new LDraw.Builder();
			builder.Cylinder("cyl", 0xFF00FF00u).cylinder(3, 1);
			var mem = builder.ToBinary().ToArray();
			Assert.True(mem.Length > 0);
		}

		[Test]
		public void TestBinaryCone()
		{
			var builder = new LDraw.Builder();
			builder.Cone("cn", 0xFF00FF00u).angle(30).height(5);
			var mem = builder.ToBinary().ToArray();
			Assert.True(mem.Length > 0);
		}

		[Test]
		public void TestBinaryTube()
		{
			var builder = new LDraw.Builder();
			builder.Tube("tube", 0xFF00FF00u).cross_section_round(0.5f).pt(new v4(0, 0, 0, 1)).pt(new v4(1, 0, 0, 1));
			var mem = builder.ToBinary().ToArray();
			Assert.True(mem.Length > 0);
		}

		[Test]
		public void TestBinaryMesh()
		{
			var builder = new LDraw.Builder();
			builder.Mesh("m", 0xFF00FF00u).vert(0, 0, 0).vert(1, 0, 0).vert(0, 1, 0).face(0, 1, 2);
			var mem = builder.ToBinary().ToArray();
			Assert.True(mem.Length > 0);
		}

		[Test]
		public void TestBinaryConvexHull()
		{
			var builder = new LDraw.Builder();
			builder.ConvexHull("ch", 0xFF00FF00u).vert(0, 0, 0).vert(1, 0, 0).vert(0, 1, 0).vert(0, 0, 1);
			var mem = builder.ToBinary().ToArray();
			Assert.True(mem.Length > 0);
		}

		[Test]
		public void TestBinaryChart()
		{
			var builder = new LDraw.Builder();
			var chart = builder.Chart("chart", 0xFF00FF00u).dim(2).data(0, 1, 1, 2);
			chart.Series("plot", 0xFF0000FFu).xaxis("C0").yaxis("C1");
			var mem = builder.ToBinary().ToArray();
			Assert.True(mem.Length > 0);
		}

		[Test]
		public void TestBinaryEquation()
		{
			var builder = new LDraw.Builder();
			builder.Equation("eq", 0xFF00FF00u).equation("sin(x)").resolution(100);
			var mem = builder.ToBinary().ToArray();
			Assert.True(mem.Length > 0);
		}
	}
}

#region DEPRECATED
namespace Rylogic.UnitTests
{
	using LDraw;

	[TestFixture]
	public class TestLdr
	{
		[Test]
		public void LdrBuilder()
		{
			var ldr = new LdrBuilder();
			using (ldr.Group("g"))
			{
				ldr.Box("b", Color.FromArgb(0, 0xFF, 0));
				ldr.Sphere("s", Color.Red);
			}
			var expected =
				"*Group g FFFFFFFF {\n" +
				"*Box b FF00FF00 {1 }\n" +
				"*Sphere s FFFF0000 {1 }\n" +
				"}\n";
			Assert.Equal(expected, ldr.ToString());
		}
	}
}
#endregion
#endif