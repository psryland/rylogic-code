using System;
using System.Collections.Generic;
using System.Linq;
using System.Windows;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Threading;
using Rylogic.Common;
using Rylogic.Container;
using Rylogic.Gfx;
using Rylogic.Maths;
using Rylogic.Utility;

namespace Rylogic.Gui.WPF
{
	/// <summary>A Legend element for a collection of ChartDataSeries</summary>
	[Obsolete] public class ChartDataLegend : ChartControl.Element
	{
		// Notes:
		//  - This is deprecated, prefer 'ChartLegend'.
		//  - This legend automatically files chart elements of type ChartDataSeries
		//    and adds them to the length, callers shouldn't have to do anything.

		private List<ChartDataSeries> m_series;

		public ChartDataLegend()
			: this(Guid.NewGuid())
		{ }
		public ChartDataLegend(Guid id)
			: base(id, "Legend")
		{
			m_bk_colour = 0xFFFFFFFF;
			m_padding = new Thickness(5);
			m_anchor = new v2(+1f, +1f);
			m_font = new Typeface("tahoma");
			m_font_size = 14.0;
			m_series = new List<ChartDataSeries>();
			PositionXY = new v2(1f, 1f);
			PositionZ = 0.001f;
			ScreenSpace = true;
		}
		protected override void Dispose(bool _)
		{
			Gfx = null;
			base.Dispose(_);
		}
		protected override void SetChartCore(ChartControl? chart)
		{
			m_series.Clear();
			if (Chart != null)
			{
				Chart.Elements.ListChanging -= HandleElementListChange;
			}
			base.SetChartCore(chart);
			if (Chart != null)
			{
				Chart.Elements.ListChanging += HandleElementListChange;
			}

			// Handler
			void HandleElementListChange(object? sender, ListChgEventArgs<ChartControl.Element> e)
			{
				// Defer the invalidate to the next event since adding elements can happen during BuildScene
				if (e.After && (e.ChangeType == ListChg.ItemAdded || e.ChangeType == ListChg.ItemRemoved || e.ChangeType == ListChg.Reset))
					Dispatcher.CurrentDispatcher.BeginInvoke(new Action(() => Invalidate()));
			}
		}

		/// <summary>Legend graphics</summary>
		public View3d.Object? Gfx
		{
			get;
			private set
			{
				if (field == value) return;
				Util.Dispose(ref field);
				field = value;
			}
		}

		/// <summary>The background colour for the legend box</summary>
		public Colour32 BackColour
		{
			get => m_bk_colour;
			set => SetProp(ref m_bk_colour, value, nameof(BackColour), true, true);
		}
		private Colour32 m_bk_colour;

		/// <summary>Padding between the legend text and the border</summary>
		public Thickness Padding
		{
			get => m_padding;
			set => SetProp(ref m_padding, value, nameof(Padding), true, true);
		}
		private Thickness m_padding;

		/// <summary>The origin position of the legend graphics</summary>
		public v2 Anchor
		{
			get => m_anchor;
			set => SetProp(ref m_anchor, value, nameof(Anchor), true, true);
		}
		private v2 m_anchor;

		/// <summary>Return the font to use for the legend</summary>
		public Typeface Font
		{
			get => m_font;
			set => SetProp(ref m_font, value, nameof(Font), true, false);
		}
		private Typeface m_font;

		/// <summary>Return the font to use for the legend</summary>
		public double FontSize
		{
			get => m_font_size;
			set => SetProp(ref m_font_size, value, nameof(FontSize), true, false);
		}
		private double m_font_size;

		/// <inheritdoc/>
		protected override void UpdateGfxCore()
		{
			base.UpdateGfxCore();

			Gfx = null;
			if (Chart == null)
				return;

			// Get the ChartDataSeries objects on the chart
			m_series = Chart.Elements.OfType<ChartDataSeries>().ToList();
			if (m_series.Count != 0)
			{
				// Create the legend graphics object
				var ldr = new LDraw.Builder();
				var txt = ldr.Text("Legend")
					.screen_space()
					.back_colour(BackColour)
					.anchor(Anchor)
					.padding(Padding)
					.font(f => f.name(Font.FontFamily.Source).size(FontSize));

				bool newline = false;
				foreach (var s in m_series)
				{
					if (newline) txt.new_block();
					txt.font(f => f.colour(s.Visible ? s.Options.Colour : Colour32.Gray));
					txt.text($"\"{s.Name}\"\n");
					newline = true;
				}

				Gfx = new View3d.Object(ldr.ToBinary().GetBuffer(), Id);
			}
		}

		/// <inheritdoc/>
		protected override void UpdateSceneCore(View3d.Window window, View3d.Camera camera)
		{
			base.UpdateSceneCore(window, camera);
			if (Gfx == null) return;
			Gfx.O2P = O2W;
			window.AddObject(Gfx);
		}

		/// <inheritdoc/>
		protected override void RemoveFromSceneCore(View3d.Window window)
		{
			base.RemoveFromSceneCore(window);
			if (Gfx == null) return;
			window.RemoveObject(Gfx);
		}

		/// <inheritdoc/>
		public override ChartControl.HitTestResult.Hit? HitTest(v4 chart_point, v2 scene_point, ModifierKeys modifier_keys, EMouseBtns mouse_btns, View3d.Camera cam)
		{
			if (Gfx == null || Chart == null || !Gfx.Visible || mouse_btns == EMouseBtns.None)
				return null;

			// TODO
			//// Get the area covered by the legend in screen space
			//var loc_nss = PositionXY;
			//var area = Gfx.BBoxMS(false).ToRectXY();
			//var loc_client = Chart.NSSToClient(new Point(loc_nss.x, loc_nss.y));

			return null;
			// todo:
			//
			//
			//
			//	area = area.Shifted(loc);
			//
			//
			//
			//
			//	// If this is a hit
			//	if (!area.Contains(client_point))
			//		return null;
			//
			//	// Get the hit point in legend space
			//	var elem_point = Drawing_.Subtract(client_point, area.TopLeft()).ToPointF();
			//
			//	// Determine which series was hit
			//	var idx = (int)Math_.Frac(Padding.Top, elem_point.Y, area.Height - Padding.Bottom) * m_series.Count;
			//	if (idx < 0 || idx >= m_series.Count)
			//		return null;
			//
			//	// Return hit data
			//	var hit = new ChartControl.HitTestResult.Hit(this, elem_point, m_series[idx]);
			//	return hit;
		}
	}
}
