using System;
using System.Collections.Generic;
using System.Drawing;
using System.Windows.Input;
using Rylogic.Common;
using Rylogic.Gfx;
using Rylogic.LDraw;
using Rylogic.Maths;
using Rylogic.Utility;

namespace Rylogic.Gui.WPF.ChartDiagram
{
	public class QuadNode :Node
	{
		public QuadNode(string text, v4? size = null, m4x4? position = null, NodeStyle? style = null, Guid? id = null)
			: base(size ?? new v4(1, 1, 0, 0), id ?? Guid.NewGuid(), text, position, style)
		{
			Size = Style.AutoSize ? PreferredSize(SizeMax) : Size;

			// Create a texture for drawing the node content into
			var tex_size = Size.xy * Style.TexelDensity;
			Surf = new Surface(tex_size.xi, tex_size.yi);

			// Create a quad model
			var ldr = new Builder();
			ldr.Rect("node", Colour32.White).rect(Size.x, Size.y).corner_radius(Style.CornerRadius).solid().axis(EAxisId.PosZ);
			Gfx = new View3d.Object(ldr.ToBinary().GetBuffer(), null);
			Gfx.SetTexture(Surf.Surf);
		}
		protected override void Dispose(bool disposing)
		{
			Gfx = null!;
			Surf = null!;
			base.Dispose(disposing);
		}

		/// <summary>Graphics for the node</summary>
		protected View3d.Object Gfx
		{
			get;
			set
			{
				if (field == value) return;
				Util.Dispose(ref field!);
				field = value;
			}
		} = null!;

		/// <summary>Texture surface</summary>
		protected Surface Surf
		{
			get;
			private set
			{
				if (field == value) return;
				Util.Dispose(ref field!);
				field = value;
			}
		} = null!;

		/// <summary>The current node text colour given the hovering/selection state</summary>
		public Colour32 TextColour
		{
			get
			{
				var text_colour = Style.Text;
				if (!Enabled) text_colour = Style.TextDisabled;
				return text_colour;
			}
		}

		/// <summary>The current node background colour given the hovering/selection state</summary>
		public Colour32 BackgroundColour
		{
			get
			{
				// Background colour
				var fill_colour = Style.Fill;
				if (Hovered) fill_colour = Style.Hovered;
				if (Selected) fill_colour = Style.Selected;
				if (!Enabled) fill_colour = Style.Disabled;
				return fill_colour;
			}
		}

		/// <inheritdoc/>
		public override BBox Bounds => new(O2W.pos, 0.5f * Size);

		/// <inheritdoc/>
		public override IEnumerable<AnchorPoint> AnchorPoints()
		{
			// Remember, returned points are in node space.
			var units_per_anchor = (float)Style.AnchorSpacing;

			// Get the dimensions and half dimensions
			var sz = Size;
			var hsx = Math.Max(0f, 0.5f * sz.x - (float)Style.CornerRadius);
			var hsy = Math.Max(0f, 0.5f * sz.y - (float)Style.CornerRadius);
			var z = 0f;

			// Quantise to the minimum anchor spacing
			hsx = (int)(hsx / units_per_anchor) * units_per_anchor;
			hsy = (int)(hsy / units_per_anchor) * units_per_anchor;
			
			int id = 0;

			// Left
			for (var y = -hsy; y <= +hsy; y += units_per_anchor)
				yield return new AnchorPoint(this, new v4(-0.5f * sz.x, y, z, 1), -v4.XAxis, id++);

			// Top
			for (var x = -hsx; x <= +hsx; x += units_per_anchor)
				yield return new AnchorPoint(this, new v4(x, +0.5f * sz.y, z, 1), +v4.YAxis, id++);

			// Right
			for (var y = +hsy; y >= -hsy; y -= units_per_anchor)
				yield return new AnchorPoint(this, new v4(+0.5f * sz.x, y, z, 1), +v4.XAxis, id++);

			// Bottom
			for (var x = +hsx; x >= -hsx; x -= units_per_anchor)
				yield return new AnchorPoint(this, new v4(x, -0.5f * sz.y, z, 1), -v4.YAxis, id++);
		}

		/// <summary>Render the background colour to the node</summary>
		protected void RenderBkgd(View3d.Texture.Lock tex)
		{
			using var fill_brush = new SolidBrush(BackgroundColour);
			tex.Gfx.FillRectangle(fill_brush, -1, -1, tex.Size.Width + 2, tex.Size.Height + 2);
		}

		/// <summary>Render the text to the node texture</summary>
		protected void RenderText(View3d.Texture.Lock tex)
		{
			using var text_brush = new SolidBrush(TextColour);
			var loc = TextLocation(tex.Gfx, new RectangleF(PointF.Empty, tex.Size));
			tex.Gfx.DrawString(Text, Style.Font, text_brush, loc, TextFormat);
		}

		/// <inheritdoc/>
		protected override void UpdateGfxCore()
		{
			base.UpdateGfxCore();

			// Refresh the texture content
			{
				using var tex = Surf.LockSurface(discard: true);
				RenderBkgd(tex);
				RenderText(tex);
			}

			// Refresh the model geometry
			if (Size.xy != Surf.Size)
			{
				var ldr = new Builder();
				ldr.Rect("node", Colour32.White).rect(Size.x, Size.y).corner_radius(Style.CornerRadius).solid(true).axis(EAxisId.PosZ);
				Gfx.UpdateModel(ldr.ToString(), View3d.EUpdateObject.Model);
				Gfx.SetTexture(Surf.Surf);
			}
		}

		/// <inheritdoc/>
		protected override void UpdateSceneCore(View3d.Window window, View3d.Camera camera)
		{
			base.UpdateSceneCore(window, camera);
			if (Gfx == null)
				return;
			
			// Don't set 'O2W' here, leave it to derived classes

			// Set the node to world transform
			Gfx.O2P = O2W;
			window.AddObject(Gfx);
		}

		/// <inheritdoc/>
		protected override void RemoveFromSceneCore(View3d.Window window)
		{
			base.RemoveFromSceneCore(window);
			if (Gfx == null)
				return;

			window.RemoveObject(Gfx);
		}

		/// <inheritdoc/>
		public override ChartControl.HitTestResult.Hit? HitTest(v4 chart_point, v2 scene_point, ModifierKeys modifier_keys, EMouseBtns mouse_btns, View3d.Camera cam)
		{
			if (Chart == null)
				return null;

			var ray = cam.RaySS(scene_point, View3d.ESnapMode.Faces, 0f);
			var results = Chart.Scene.Window.HitTest(ray, new[] { Gfx });
			if (!results.IsHit)
				return null;

			// Convert the hit point to node space
			var pt = Math_.InvertAffine(O2W) * results.m_ws_intercept;
			var hit = new ChartControl.HitTestResult.Hit(this, pt, null);
			return hit;
		}
	}
}
