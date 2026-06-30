using System;
using System.Collections.Generic;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using System.Windows.Threading;
using HantekScope.Device;
using HantekScope.Model;
using Rylogic.Gfx;
using Rylogic.Gui.WPF;
using Rylogic.Utility;

namespace HantekScope.UI
{
	/// <summary>Main application window hosting the live oscilloscope trace.</summary>
	public partial class MainWindow :Window
	{
		// Cap on points kept per channel in scrolling mode. The chart only renders the
		// slice inside the visible X range, so this bounds memory rather than render
		// cost; when exceeded the oldest chunk is dropped to keep the stream bounded.
		private const int MaxPointsPerChannel = 200000;
		private const int TrimChunk = 50000;

		// Cap on points retained for the framed display. One waveform record is 1200
		// samples; a few records of headroom keeps a zoomed-out frame populated without
		// the rebuilt-every-tick cost growing unbounded.
		private const int MaxFramePoints = 6000;

		// How much signal-time (ms) the follow view shows ending at the latest sample
		// when scrolling, and the default empty-chart X window otherwise.
		private const double FollowWindowMs = 5.0;

		private readonly ScopeModel m_model = new();
		private readonly ChartDataSeries m_ch1 = new("CH1", ChartDataSeries.EFormat.XRealYReal);
		private readonly ChartDataSeries m_ch2 = new("CH2", ChartDataSeries.EFormat.XRealYReal);
		private readonly DispatcherTimer m_render_timer;

		// Framed-mode rolling buffer of the most recent samples, redrawn from t=0 each
		// time new data arrives so the trace sits relative to the (software) trigger.
		private readonly List<Sample> m_frame = new();

		// Display colours. Background defaults to a dark gray; channels to the classic
		// scope yellow / cyan. All are user-editable via the Appearance menu.
		private Colour32 m_bg_colour = new(0xFF2A2A2Au);
		private Colour32 m_ch1_colour = Colour32.Yellow;
		private Colour32 m_ch2_colour = Colour32.Cyan;

		// Display mode and per-axis auto-resolution flags.
		private bool m_scrolling;
		private bool m_xauto = true;
		private bool m_yauto = true;

		// Checkable menu items kept so their IsChecked state mirrors the flags above.
		private MenuItem m_mi_scrolling = null!;
		private MenuItem m_mi_xauto = null!;
		private MenuItem m_mi_yauto = null!;

		private double m_latest_x;
		private bool m_have_xrange;
		private bool m_have_yrange;

		// Set while the code is programmatically moving the camera (auto-fit, follow,
		// reset, frame redraw). The chart raises ChartMoved for these just as it does
		// for user navigation, so the flag lets OnChartMoved ignore self-induced moves
		// and only run auto-resolution in response to genuine user zooming.
		private bool m_suppress_auto;

		public MainWindow()
		{
			InitializeComponent();

			// 2D orthographic view with a border, and the dark-gray scene background.
			m_chart.Options.Orthographic = true;
			m_chart.Options.SceneBorderThickness = 1;
			m_chart.Options.SceneBorderColour = Colour32.Black;
			m_chart.Options.BackgroundColour = m_bg_colour;

			// Model events arrive on the acquisition thread; marshal them to the UI.
			m_model.StatusChanged += OnStatusChanged;
			m_model.IdentityRead += OnIdentityRead;
			m_model.ConfigChanged += OnConfigChanged;

			// Drive rendering at ~60 fps, independent of the acquisition rate.
			m_render_timer = new DispatcherTimer(DispatcherPriority.Render)
			{
				Interval = TimeSpan.FromMilliseconds(16),
			};
			m_render_timer.Tick += OnRenderTick;

			// Series configuration creates View3d stock shaders, which need the chart's
			// render device to exist. That isn't ready until the control is loaded into
			// the visual tree, so defer it to the Loaded event.
			Loaded += OnLoaded;
		}

		/// <summary>Configure the channel series and context menus once the chart's render device exists.</summary>
		private void OnLoaded(object sender, RoutedEventArgs e)
		{
			Loaded -= OnLoaded;

			// Both channels drawn as connected lines. PointsOnLinePlot defaults true,
			// which would also build the point-sprite shader; disable it so traces are
			// pure lines (and so the unset PointSprite shader is never referenced).
			ConfigureSeries(m_ch1, m_ch1_colour);
			ConfigureSeries(m_ch2, m_ch2_colour);

			// Start with a sensible window so the empty chart isn't degenerate.
			SetRange(() =>
			{
				m_chart.Range.XAxis.Set(0, FollowWindowMs);
				m_chart.Range.YAxis.Set(-5, 5);
			});

			BuildContextMenus();
			m_chart.ChartMoved += OnChartMoved;

			UpdateConfigStatus();
			UpdateModeStatus();
		}

		/// <summary>Apply the line-plot configuration and colour to a channel series.</summary>
		private void ConfigureSeries(ChartDataSeries series, Colour32 colour)
		{
			series.Options.Colour = colour;
			series.Options.PlotType = ChartDataSeries.EPlotType.Line;
			series.Options.PointsOnLinePlot = false;
			series.Options.LineWidth = 1f;
			series.Chart = m_chart;
		}

		/// <summary>
		/// Build the scene and per-axis right-click menus in code. The scene menu carries
		/// appearance + display-mode controls; each axis menu carries auto-resolution and
		/// a reset. Custom menus are deliberately not named "ChartCMenu"/"ChartAxisCMenu"
		/// so the chart doesn't rebind their DataContext to its own command source.
		/// </summary>
		private void BuildContextMenus()
		{
			// Scene menu: Appearance submenu + horizontal-scrolling toggle.
			var appearance = new MenuItem { Header = "Appearance" };
			appearance.Items.Add(MakeItem("_Background…", OnPickBackground));
			appearance.Items.Add(MakeItem("CH_1 Colour…", OnPickCh1Colour));
			appearance.Items.Add(MakeItem("CH_2 Colour…", OnPickCh2Colour));

			m_mi_scrolling = new MenuItem { Header = "Horizontal Scrolling", IsCheckable = true, IsChecked = m_scrolling };
			m_mi_scrolling.Click += OnToggleScrolling;

			var scene = new ContextMenu();
			scene.Items.Add(appearance);
			scene.Items.Add(new Separator());
			scene.Items.Add(m_mi_scrolling);
			m_chart.SceneCMenu = scene;

			// X axis menu: auto-resolution toggle + reset zoom.
			m_mi_xauto = new MenuItem { Header = "Auto Resolution", IsCheckable = true, IsChecked = m_xauto };
			m_mi_xauto.Click += (s, e) => m_xauto = m_mi_xauto.IsChecked;

			var xmenu = new ContextMenu();
			xmenu.Items.Add(m_mi_xauto);
			xmenu.Items.Add(new Separator());
			xmenu.Items.Add(MakeItem("Reset Zoom", OnResetXZoom));
			m_chart.XAxisCMenu = xmenu;

			// Y axis menu: auto-resolution toggle + reset zoom.
			m_mi_yauto = new MenuItem { Header = "Auto Resolution", IsCheckable = true, IsChecked = m_yauto };
			m_mi_yauto.Click += (s, e) => m_yauto = m_mi_yauto.IsChecked;

			var ymenu = new ContextMenu();
			ymenu.Items.Add(m_mi_yauto);
			ymenu.Items.Add(new Separator());
			ymenu.Items.Add(MakeItem("Reset Zoom", OnResetYZoom));
			m_chart.YAxisCMenu = ymenu;
		}

		/// <summary>Create a simple clickable menu item.</summary>
		private static MenuItem MakeItem(string header, RoutedEventHandler on_click)
		{
			var item = new MenuItem { Header = header };
			item.Click += on_click;
			return item;
		}

		/// <summary>
		/// Apply a programmatic axis range and update the camera. The chart raises its
		/// ChartMoved event asynchronously (coalesced onto the dispatcher), so a flag is
		/// latched here and consumed by the deferred OnChartMoved, ensuring self-induced
		/// moves don't feed back into auto-resolution while genuine user moves still do.
		/// </summary>
		private void SetRange(Action set_axes)
		{
			m_suppress_auto = true;
			set_axes();
			m_chart.SetCameraFromRange();
		}

		/// <summary>File ▸ Exit.</summary>
		private void OnExit(object sender, RoutedEventArgs e)
		{
			Close();
		}

		/// <summary>Toggle acquisition on/off.</summary>
		private void OnRunStop(object sender, RoutedEventArgs e)
		{
			if (!m_model.IsRunning)
			{
				ResetDisplay();
				m_model.Start();
				m_render_timer.Start();
				m_btn_run.Content = "Stop";
				SetConnection("Connecting…", EConn.Connecting);
			}
			else
			{
				m_render_timer.Stop();
				m_model.Stop();
				m_btn_run.Content = "Run";
				SetConnection("Stopped", EConn.Stopped);
			}
		}

		/// <summary>Toggle between framed (trigger-relative) and horizontal-scrolling display.</summary>
		private void OnToggleScrolling(object sender, RoutedEventArgs e)
		{
			m_scrolling = m_mi_scrolling.IsChecked;
			ResetDisplay();
			UpdateModeStatus();
		}

		/// <summary>Pick the chart background colour.</summary>
		private void OnPickBackground(object sender, RoutedEventArgs e)
		{
			var dlg = new ColourPickerUI(this, m_bg_colour);
			if (dlg.ShowDialog() != true)
				return;

			m_bg_colour = dlg.Colour;
			m_chart.Options.BackgroundColour = m_bg_colour;
			m_chart.Invalidate();
		}

		/// <summary>Pick the CH1 trace colour.</summary>
		private void OnPickCh1Colour(object sender, RoutedEventArgs e)
		{
			if (PickColour(ref m_ch1_colour))
				ApplySeriesColour(m_ch1, m_ch1_colour);
		}

		/// <summary>Pick the CH2 trace colour.</summary>
		private void OnPickCh2Colour(object sender, RoutedEventArgs e)
		{
			if (PickColour(ref m_ch2_colour))
				ApplySeriesColour(m_ch2, m_ch2_colour);
		}

		/// <summary>Show the colour picker seeded with the current colour; return true if changed.</summary>
		private bool PickColour(ref Colour32 colour)
		{
			var dlg = new ColourPickerUI(this, colour);
			if (dlg.ShowDialog() != true)
				return false;

			colour = dlg.Colour;
			return true;
		}

		/// <summary>Apply a new series colour. A colour change doesn't auto-rebuild graphics, so flush.</summary>
		private void ApplySeriesColour(ChartDataSeries series, Colour32 colour)
		{
			series.Options.Colour = colour;
			series.FlushCachedGraphics();
			m_chart.Invalidate();
		}

		/// <summary>Reset the X axis to the default window (also re-running X auto-resolution).</summary>
		private void OnResetXZoom(object sender, RoutedEventArgs e)
		{
			SetRange(() => m_chart.Range.XAxis.Set(0, FollowWindowMs));
			m_chart.Invalidate();
		}

		/// <summary>Reset the Y axis to the default window.</summary>
		private void OnResetYZoom(object sender, RoutedEventArgs e)
		{
			SetRange(() => m_chart.Range.YAxis.Set(-5, 5));
			m_chart.Invalidate();
		}

		/// <summary>
		/// React to chart navigation. When an axis is zoomed and that axis has
		/// auto-resolution enabled, request the scope resolution that puts the visible
		/// span at the best available resolution. Scroll moves are ignored so following
		/// doesn't drive resolution changes.
		/// </summary>
		private void OnChartMoved(object? sender, ChartControl.ChartMovedEventArgs e)
		{
			// Ignore self-induced moves (auto-fit, follow, reset, frame redraw); only
			// genuine user zooming should change the scope's resolution. The chart
			// coalesces a programmatic change into a single deferred event, so consume
			// the latch here and let the next (user) move through.
			if (m_suppress_auto)
			{
				m_suppress_auto = false;
				return;
			}

			// Y: choose volts/div so the visible volt span fills the ADC's full-scale
			// division count (span / full-scale-divisions ≈ the per-division value).
			if (m_yauto && (e.MoveType & ChartControl.EMoveType.YZoomed) != 0)
			{
				var span_v = m_chart.Range.YAxis.Span;
				var target_vdiv = span_v / HantekProtocol.AdcFullScaleDiv;
				m_model.RequestVoltsDivIndex(HantekProtocol.NearestVoltsDivIndex(target_vdiv));
			}

			// X: choose the timebase whose record duration matches the visible time span
			// (span-seconds / record-divisions ≈ the per-division time).
			if (m_xauto && (e.MoveType & ChartControl.EMoveType.XZoomed) != 0)
			{
				var span_s = m_chart.Range.XAxis.Span / 1000.0;
				var target_tb = span_s / HantekProtocol.RecordDivisions;
				m_model.RequestTimebaseIndex(HantekProtocol.NearestTimebaseIndex(target_tb));
			}
		}

		/// <summary>Pull pending samples into the chart, branching on the display mode.</summary>
		private void OnRenderTick(object? sender, EventArgs e)
		{
			var samples = m_model.DrainPending();
			if (samples.Count != 0)
			{
				if (m_scrolling)
					RenderScrolling(samples);
				else
					RenderFramed(samples);
			}

			m_chart.Invalidate();
		}

		/// <summary>Scrolling mode: append absolute-time samples and follow the latest at the right edge.</summary>
		private void RenderScrolling(List<Sample> samples)
		{
			if (!m_have_yrange)
			{
				FitYRange(samples);
				m_have_yrange = true;
			}

			using (var lk = m_ch1.Lock())
			{
				foreach (var s in samples)
					lk.Add(new ChartDataSeries.Pt(s.XMs, s.Ch1V));
				TrimOldest(lk);
			}

			using (var lk = m_ch2.Lock())
			{
				foreach (var s in samples)
				{
					if (!double.IsNaN(s.Ch2V))
						lk.Add(new ChartDataSeries.Pt(s.XMs, s.Ch2V));
				}
				TrimOldest(lk);
			}

			// Keep the newest data at the right edge, preserving the user's zoom span.
			m_latest_x = samples[^1].XMs;
			var span = m_chart.Range.XAxis.Span;
			SetRange(() => m_chart.Range.XAxis.Set(m_latest_x - span, m_latest_x));
		}

		/// <summary>
		/// Framed mode: accumulate the most recent samples and redraw the whole frame
		/// from t=0 each tick, so the trace is anchored to the (software) trigger origin.
		/// </summary>
		private void RenderFramed(List<Sample> samples)
		{
			m_frame.AddRange(samples);

			// Trim to the most recent window so the rebuild stays bounded.
			if (m_frame.Count > MaxFramePoints)
				m_frame.RemoveRange(0, m_frame.Count - MaxFramePoints);

			if (!m_have_yrange)
			{
				FitYRange(m_frame);
				m_have_yrange = true;
			}

			// Per-sample spacing at the currently applied timebase.
			var dt_ms = m_model.SampleIntervalS * 1000.0;
			if (dt_ms <= 0)
				dt_ms = 1.0;

			using (var lk = m_ch1.Lock())
			{
				lk.Clear();
				for (var k = 0; k != m_frame.Count; ++k)
					lk.Add(new ChartDataSeries.Pt(k * dt_ms, m_frame[k].Ch1V));
			}

			using (var lk = m_ch2.Lock())
			{
				lk.Clear();
				for (var k = 0; k != m_frame.Count; ++k)
				{
					if (!double.IsNaN(m_frame[k].Ch2V))
						lk.Add(new ChartDataSeries.Pt(k * dt_ms, m_frame[k].Ch2V));
				}
			}

			// Fit the X axis once to one frame width so the trace fills the view.
			if (!m_have_xrange)
			{
				SetRange(() => m_chart.Range.XAxis.Set(0, Math.Max(1, m_frame.Count) * dt_ms));
				m_have_xrange = true;
			}
		}

		/// <summary>Set the Y axis to span both channels of the batch, with headroom.</summary>
		private void FitYRange(List<Sample> samples)
		{
			var lo = double.MaxValue;
			var hi = double.MinValue;
			foreach (var s in samples)
			{
				lo = Math.Min(lo, s.Ch1V);
				hi = Math.Max(hi, s.Ch1V);
				if (!double.IsNaN(s.Ch2V))
				{
					lo = Math.Min(lo, s.Ch2V);
					hi = Math.Max(hi, s.Ch2V);
				}
			}

			// Guard against a degenerate (flat or empty) batch.
			if (lo >= hi)
			{
				lo -= 1.0;
				hi += 1.0;
			}

			var pad = 0.2 * (hi - lo);
			SetRange(() => m_chart.Range.YAxis.Set(lo - pad, hi + pad));
		}

		/// <summary>Drop the oldest chunk once a series exceeds the cap, keeping memory bounded.</summary>
		private static void TrimOldest(ChartDataSeries.LockData lk)
		{
			if (lk.Count <= MaxPointsPerChannel)
				return;

			// Shift the surviving points down and shrink the series; infrequent, so the
			// O(n) shift is acceptable for a rolling stream.
			var keep = lk.Count - TrimChunk;
			for (var i = 0; i != keep; ++i)
				lk[i] = lk[i + TrimChunk];
			lk.Count = keep;
		}

		/// <summary>Clear both series and reset the display state ready for a fresh run/mode.</summary>
		private void ResetDisplay()
		{
			m_frame.Clear();
			m_latest_x = 0;
			m_have_xrange = false;
			m_have_yrange = false;

			using (var lk = m_ch1.Lock())
				lk.Clear();
			using (var lk = m_ch2.Lock())
				lk.Clear();

			m_chart.Invalidate();
		}

		/// <summary>Marshal a model status update onto the UI thread.</summary>
		private void OnStatusChanged(string text)
		{
			Dispatcher.BeginInvoke(() =>
			{
				// Derive a connection state from the message for the toolbar LED.
				var state =
					text.StartsWith("Error", StringComparison.OrdinalIgnoreCase) ? EConn.Error :
					text.StartsWith("Running", StringComparison.OrdinalIgnoreCase) ? EConn.Running :
					EConn.Connecting;
				SetConnection(text, state);

				// An error stops acquisition inside the model; reflect that in the UI.
				if (!m_model.IsRunning && m_render_timer.IsEnabled)
				{
					m_render_timer.Stop();
					m_btn_run.Content = "Run";
				}
			});
		}

		/// <summary>Marshal the device identity onto the status bar.</summary>
		private void OnIdentityRead(string serial, string firmware, string version)
		{
			Dispatcher.BeginInvoke(() =>
			{
				m_status_identity.Text = $"{serial}  fw {firmware}";
			});
		}

		/// <summary>A resolution change restarts the time origin; clear the frame and refresh status.</summary>
		private void OnConfigChanged()
		{
			Dispatcher.BeginInvoke(() =>
			{
				// Old and new samples decode at different dt, so don't mix them in a frame.
				m_frame.Clear();
				m_have_xrange = false;
				UpdateConfigStatus();
			});
		}

		/// <summary>Toolbar connection-state colours.</summary>
		private enum EConn { Stopped, Connecting, Running, Error }

		/// <summary>Update the toolbar connection text and LED colour.</summary>
		private void SetConnection(string text, EConn state)
		{
			m_conn_status.Text = text;
			var colour = state switch
			{
				EConn.Running => Color.FromRgb(0x2E, 0xCC, 0x71),
				EConn.Connecting => Color.FromRgb(0xF3, 0x9C, 0x12),
				EConn.Error => Color.FromRgb(0xE7, 0x4C, 0x3C),
				EConn.Stopped => Color.FromRgb(0x7F, 0x8C, 0x8D),
				_ => throw new ArgumentOutOfRangeException(nameof(state)),
			};
			m_conn_led.Fill = new SolidColorBrush(colour);
		}

		/// <summary>Refresh the status-bar resolution readout from the model's applied scale.</summary>
		private void UpdateConfigStatus()
		{
			var volts_div = m_model.VoltsPerDiv;
			var time_div = m_model.SampleIntervalS * HantekProtocol.SamplesPerDivision;
			m_status_config.Text = $"{FormatVolts(volts_div)}/div   {FormatTime(time_div)}/div";
		}

		/// <summary>Refresh the status-bar display-mode readout.</summary>
		private void UpdateModeStatus()
		{
			m_status_mode.Text = m_scrolling ? "Scrolling" : "Framed";
		}

		/// <summary>Format a volts value with a sensible mV/V unit.</summary>
		private static string FormatVolts(double volts)
		{
			return volts < 1.0
				? $"{volts * 1000.0:0.##} mV"
				: $"{volts:0.##} V";
		}

		/// <summary>Format a seconds value with a sensible ns/µs/ms/s unit.</summary>
		private static string FormatTime(double seconds)
		{
			if (seconds < 1e-6) return $"{seconds * 1e9:0.##} ns";
			if (seconds < 1e-3) return $"{seconds * 1e6:0.##} µs";
			if (seconds < 1.0) return $"{seconds * 1e3:0.##} ms";
			return $"{seconds:0.##} s";
		}

		/// <summary>Tear down acquisition and the chart series on close.</summary>
		protected override void OnClosed(EventArgs e)
		{
			m_render_timer.Stop();
			m_model.Dispose();
			Util.Dispose(m_ch1);
			Util.Dispose(m_ch2);
			Gui_.DisposeChildren(this, EventArgs.Empty);
			base.OnClosed(e);
		}
	}
}
