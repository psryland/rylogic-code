using System;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Shapes;
using Rylogic.Gfx;
using Rylogic.Gui.WPF;
using Rylogic.Maths;
using Rylogic.Windows.Extn;

namespace HantekScope.UI
{
	/// <summary>
	/// Draggable scope overlays drawn on the chart's built-in overlay canvas and axis
	/// panels (the same mechanism ChartControl.CrossHair uses). Provides a trigger-level
	/// line with a Y-axis tag, a trigger-time line with an X-axis tag (framed mode only),
	/// and a per-channel vertical-position tab pinned to the left scene edge. All state is
	/// held in chart units (volts on Y, milliseconds on X); the owner pushes current values
	/// in via the properties and formats the tag text, then calls <see cref="Reposition"/>
	/// whenever the chart moves or re-renders. Dragging a handle clamps to the visible axis,
	/// invokes the matching callback so the owner can move the trigger or a channel trace,
	/// and repositions immediately for live feedback.
	/// </summary>
	internal sealed class ScopeOverlays :IDisposable
	{
		// Which handle is currently being dragged. Mouse capture is taken on the handle
		// element at drag start, so move/up events arrive here until release.
		private enum EDrag { None, Level, Time, Ch1, Ch2 }

		private readonly ChartControl m_chart;

		// Trigger level: a horizontal line across the scene plus a draggable Y-axis tag.
		private readonly Line m_level_line;
		private readonly Border m_level_tag;
		private readonly TextBlock m_level_text;

		// Trigger time: a vertical line plus a draggable X-axis tag. Framed mode only.
		private readonly Line m_time_line;
		private readonly Border m_time_tag;
		private readonly TextBlock m_time_text;

		// Per-channel vertical-position handles at the left scene edge, in channel colour.
		private readonly Polygon m_ch1_tab;
		private readonly Polygon m_ch2_tab;

		private EDrag m_drag;

		public ScopeOverlays(ChartControl chart, Colour32 ch1_colour, Colour32 ch2_colour)
		{
			m_chart = chart;
			m_drag = EDrag.None;

			// Trigger indicators use a scope-red so they read clearly against either
			// channel colour and the dark background.
			var trig_colour = new Colour32(0xFFE0403Au);

			m_level_line = MakeIndicatorLine(trig_colour);
			m_time_line = MakeIndicatorLine(trig_colour);

			m_level_text = MakeTagText();
			m_time_text = MakeTagText();
			m_level_tag = MakeTag(trig_colour, m_level_text);
			m_time_tag = MakeTag(trig_colour, m_time_text);

			m_ch1_tab = MakeChannelTab(ch1_colour);
			m_ch2_tab = MakeChannelTab(ch2_colour);

			// Match the axis label font so the tags sit consistently with the tick text.
			m_level_text.Typeface(m_chart.YAxisPanel.Typeface, m_chart.YAxisPanel.FontSize);
			m_time_text.Typeface(m_chart.XAxisPanel.Typeface, m_chart.XAxisPanel.FontSize);

			// The lines live on the scene overlay (display-only; the overlay canvas is not
			// hit-test-visible). The draggable handles — the value tags and the channel
			// tabs — must live on the axis panels, which do receive mouse input, so their
			// Preview handlers can claim the gesture before the chart's scene navigation.
			m_chart.Overlay.Adopt(m_level_line);
			m_chart.Overlay.Adopt(m_time_line);
			m_chart.YAxisPanel.Adopt(m_level_tag);
			m_chart.XAxisPanel.Adopt(m_time_tag);
			m_chart.YAxisPanel.Adopt(m_ch1_tab);
			m_chart.YAxisPanel.Adopt(m_ch2_tab);

			// Only the tags/tabs are grab handles; the thin lines are non-interactive.
			MakeDraggable(m_level_tag, EDrag.Level);
			MakeDraggable(m_time_tag, EDrag.Time);
			MakeDraggable(m_ch1_tab, EDrag.Ch1);
			MakeDraggable(m_ch2_tab, EDrag.Ch2);
		}
		public void Dispose()
		{
			m_level_line.Detach();
			m_time_line.Detach();
			m_level_tag.Detach();
			m_time_tag.Detach();
			m_ch1_tab.Detach();
			m_ch2_tab.Detach();
		}

		/// <summary>Master visibility for the trigger level/time indicators.</summary>
		public bool ShowTrigger { get; set; } = true;

		/// <summary>True in framed mode; gates the trigger-time line/tag (undefined while scrolling).</summary>
		public bool Framed { get; set; } = true;

		/// <summary>Per-channel tab visibility (hidden when the channel is disabled).</summary>
		public bool Ch1TabVisible { get; set; } = true;
		public bool Ch2TabVisible { get; set; } = true;

		/// <summary>Trigger level in volts (Y position of the level line/tag).</summary>
		public double TriggerLevelVolts { get; set; }

		/// <summary>Trigger time offset in milliseconds (X position of the time line/tag).</summary>
		public double TriggerTimeMs { get; set; }

		/// <summary>Per-channel vertical display offset in volts (Y position of each tab).</summary>
		public double Ch1OffsetVolts { get; set; }
		public double Ch2OffsetVolts { get; set; }

		/// <summary>Owner-formatted label text for the level and time tags.</summary>
		public string LevelText { set { m_level_text.Text = value; } }
		public string TimeText { set { m_time_text.Text = value; } }

		/// <summary>Callbacks raised while dragging, with the new clamped chart value.</summary>
		public Action<double>? LevelDragged { get; set; }
		public Action<double>? TimeDragged { get; set; }
		public Action<double>? Ch1Dragged { get; set; }
		public Action<double>? Ch2Dragged { get; set; }

		/// <summary>Recolour the channel tabs when a channel's display colour changes.</summary>
		public void SetChannelColours(Colour32 ch1_colour, Colour32 ch2_colour)
		{
			m_ch1_tab.Fill = ch1_colour.ToMediaBrush();
			m_ch2_tab.Fill = ch2_colour.ToMediaBrush();
		}

		/// <summary>
		/// Recompute every element's geometry from the current chart-unit values and the
		/// chart's camera transform. Cheap enough to call each render frame and on move.
		/// </summary>
		public void Reposition()
		{
			var bounds = m_chart.SceneBounds;
			var show_time = ShowTrigger && Framed;

			// Trigger level: horizontal line at the level voltage, tag on the Y axis.
			m_level_line.Visibility = ShowTrigger ? Visibility.Visible : Visibility.Collapsed;
			m_level_tag.Visibility = ShowTrigger ? Visibility.Visible : Visibility.Collapsed;
			if (ShowTrigger)
			{
				var sy = m_chart.ChartToScene(new v4(0f, (float)TriggerLevelVolts, 0f, 1f)).y;
				m_level_line.X1 = 0;
				m_level_line.X2 = bounds.Width;
				m_level_line.Y1 = sy;
				m_level_line.Y2 = sy;
				PositionYAxisTag(m_level_tag, sy);
			}

			// Trigger time: vertical line at the time offset, tag on the X axis.
			m_time_line.Visibility = show_time ? Visibility.Visible : Visibility.Collapsed;
			m_time_tag.Visibility = show_time ? Visibility.Visible : Visibility.Collapsed;
			if (show_time)
			{
				var sx = m_chart.ChartToScene(new v4((float)TriggerTimeMs, 0f, 0f, 1f)).x;
				m_time_line.X1 = sx;
				m_time_line.X2 = sx;
				m_time_line.Y1 = 0;
				m_time_line.Y2 = bounds.Height;
				PositionXAxisTag(m_time_tag, sx);
			}

			// Channel vertical-position tabs at the left scene edge.
			PositionChannelTab(m_ch1_tab, Ch1OffsetVolts, Ch1TabVisible);
			PositionChannelTab(m_ch2_tab, Ch2OffsetVolts, Ch2TabVisible);
		}

		/// <summary>Place a Y-axis value tag at scene-Y 'sy', aligned like the tick labels.</summary>
		private void PositionYAxisTag(Border tag, double sy)
		{
			tag.Measure(new Size(double.PositiveInfinity, double.PositiveInfinity));
			var size = tag.DesiredSize;
			var axis_pt = Gui_.MapPoint(m_chart.Scene, m_chart.YAxisPanel, new Point(0, sy));

			switch (m_chart.YAxis.Options.Side)
			{
				case Dock.Left:
				{
					Canvas.SetLeft(tag, m_chart.YAxisPanel.Width - size.Width - m_chart.YAxis.Options.TickLength);
					Canvas.SetTop(tag, axis_pt.Y - size.Height / 2);
					break;
				}
				case Dock.Right:
				{
					Canvas.SetLeft(tag, m_chart.YAxis.Options.TickLength);
					Canvas.SetTop(tag, axis_pt.Y - size.Height / 2);
					break;
				}
				default:
				{
					throw new ArgumentOutOfRangeException(nameof(Dock), "Unexpected side for the Y axis");
				}
			}
		}

		/// <summary>Place an X-axis value tag at scene-X 'sx', aligned like the tick labels.</summary>
		private void PositionXAxisTag(Border tag, double sx)
		{
			tag.Measure(new Size(double.PositiveInfinity, double.PositiveInfinity));
			var size = tag.DesiredSize;
			var axis_pt = Gui_.MapPoint(m_chart.Scene, m_chart.XAxisPanel, new Point(sx, 0));

			switch (m_chart.XAxis.Options.Side)
			{
				case Dock.Bottom:
				{
					Canvas.SetLeft(tag, axis_pt.X - size.Width / 2);
					Canvas.SetTop(tag, m_chart.XAxis.Options.TickLength);
					break;
				}
				case Dock.Top:
				{
					Canvas.SetLeft(tag, axis_pt.X - size.Width / 2);
					Canvas.SetTop(tag, m_chart.XAxisPanel.Height - size.Height - m_chart.XAxis.Options.TickLength);
					break;
				}
				default:
				{
					throw new ArgumentOutOfRangeException(nameof(Dock), "Unexpected side for the X axis");
				}
			}
		}

		/// <summary>
		/// Position a channel tab on the Y-axis panel at the channel's zero-reference
		/// voltage, with its pointed tip toward the scene edge like a scope ground
		/// marker. Hosted on the axis panel (not the overlay) so it is draggable.
		/// </summary>
		private void PositionChannelTab(Polygon tab, double offset_volts, bool visible)
		{
			tab.Visibility = visible ? Visibility.Visible : Visibility.Collapsed;
			if (!visible)
				return;

			var sy = m_chart.ChartToScene(new v4(0f, (float)offset_volts, 0f, 1f)).y;
			var axis_pt = Gui_.MapPoint(m_chart.Scene, m_chart.YAxisPanel, new Point(0, sy));

			switch (m_chart.YAxis.Options.Side)
			{
				case Dock.Left:
				{
					// Pointed tip toward the scene on the right edge of the left panel.
					Canvas.SetLeft(tab, m_chart.YAxisPanel.Width - TabWidth);
					Canvas.SetTop(tab, axis_pt.Y - TabHeight / 2);
					break;
				}
				case Dock.Right:
				{
					Canvas.SetLeft(tab, 0);
					Canvas.SetTop(tab, axis_pt.Y - TabHeight / 2);
					break;
				}
				default:
				{
					throw new ArgumentOutOfRangeException(nameof(Dock), "Unexpected side for the Y axis");
				}
			}
		}

		/// <summary>Wire the drag gesture (capture on down, track on move, release on up) to a handle.</summary>
		private void MakeDraggable(FrameworkElement handle, EDrag kind)
		{
			// Use the tunnelling Preview events so the handle claims the gesture (and takes
			// mouse capture) before the chart's own navigation handlers see the press.
			handle.Cursor = kind == EDrag.Time ? Cursors.SizeWE : Cursors.SizeNS;
			handle.PreviewMouseLeftButtonDown += (s, e) =>
			{
				m_drag = kind;
				handle.CaptureMouse();
				e.Handled = true;
			};
			handle.PreviewMouseMove += (s, e) =>
			{
				if (m_drag != kind)
					return;

				// Map the pointer to chart space and clamp to the visible axis so the
				// handle can't be dragged out of view.
				var sp = e.GetPosition(m_chart.Scene);
				var cp = m_chart.SceneToChart(new v2((float)sp.X, (float)sp.Y));
				switch (kind)
				{
					case EDrag.Level:
					{
						var v = Math_.Clamp(cp.y, m_chart.YAxis.Min, m_chart.YAxis.Max);
						TriggerLevelVolts = v;
						LevelDragged?.Invoke(v);
						break;
					}
					case EDrag.Time:
					{
						var ms = Math_.Clamp(cp.x, m_chart.XAxis.Min, m_chart.XAxis.Max);
						TriggerTimeMs = ms;
						TimeDragged?.Invoke(ms);
						break;
					}
					case EDrag.Ch1:
					{
						var v = Math_.Clamp(cp.y, m_chart.YAxis.Min, m_chart.YAxis.Max);
						Ch1OffsetVolts = v;
						Ch1Dragged?.Invoke(v);
						break;
					}
					case EDrag.Ch2:
					{
						var v = Math_.Clamp(cp.y, m_chart.YAxis.Min, m_chart.YAxis.Max);
						Ch2OffsetVolts = v;
						Ch2Dragged?.Invoke(v);
						break;
					}
					default:
					{
						throw new ArgumentOutOfRangeException(nameof(kind));
					}
				}

				Reposition();
				e.Handled = true;
			};
			handle.PreviewMouseLeftButtonUp += (s, e) =>
			{
				if (m_drag != kind)
					return;

				m_drag = EDrag.None;
				handle.ReleaseMouseCapture();
				e.Handled = true;
			};
		}

		/// <summary>Create a thin, non-interactive indicator line for the scene overlay.</summary>
		private static Line MakeIndicatorLine(Colour32 colour)
		{
			return new Line
			{
				Stroke = colour.ToMediaBrush(),
				StrokeThickness = 1.0,
				StrokeStartLineCap = PenLineCap.Flat,
				StrokeEndLineCap = PenLineCap.Flat,
				IsHitTestVisible = false,
			};
		}

		/// <summary>Create the text element shown inside a value tag.</summary>
		private static TextBlock MakeTagText()
		{
			return new TextBlock
			{
				TextAlignment = TextAlignment.Center,
				Foreground = Colour32.White.ToMediaBrush(),
				IsHitTestVisible = false,
			};
		}

		/// <summary>Create a rounded, filled value tag (a grab handle) wrapping 'text'.</summary>
		private static Border MakeTag(Colour32 colour, TextBlock text)
		{
			return new Border
			{
				CornerRadius = new CornerRadius(3),
				Background = colour.ToMediaBrush(),
				Padding = new Thickness(5, 2, 5, 2),
				MinWidth = 44,
				Child = text,
			};
		}

		/// <summary>Create a right-pointing channel-position tab in the channel colour.</summary>
		private static Polygon MakeChannelTab(Colour32 colour)
		{
			// A rectangle with a pointed right edge, like a scope ground-reference marker.
			return new Polygon
			{
				Fill = colour.ToMediaBrush(),
				Points = new PointCollection
				{
					new Point(0, 0),
					new Point(TabWidth - TabPoint, 0),
					new Point(TabWidth, TabHeight / 2),
					new Point(TabWidth - TabPoint, TabHeight),
					new Point(0, TabHeight),
				},
			};
		}

		private const double TabWidth = 16.0;
		private const double TabHeight = 12.0;
		private const double TabPoint = 6.0;
	}
}
