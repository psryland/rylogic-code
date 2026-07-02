using System;
using System.Collections.Generic;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Threading;
using HantekScope.Device;
using HantekScope.Model;
using Rylogic.Gfx;
using Rylogic.Gui.WPF;
using Rylogic.Maths;
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

		// Framed mode shows exactly one waveform record (matching the hardware's
		// screen): the device's fixed record is HantekProtocol.WaveformRecordSamples
		// samples per channel. The displayed window stays one record wide, centred on
		// a software trigger crossing at x=0. The frame buffer holds two records so the
		// re-trigger search has half a record of pre- and post-trigger headroom around
		// any crossing; older samples roll off as new ones arrive.
		private const int FrameWindowSamples = HantekProtocol.WaveformRecordSamples;
		private const int MaxFramePoints = 2 * HantekProtocol.WaveformRecordSamples;

		// How much signal-time (ms) the follow view shows ending at the latest sample
		// when scrolling, and the default empty-chart X window otherwise.
		private const double FollowWindowMs = 5.0;

		private readonly ScopeModel m_model = new();
		private readonly ChartDataSeries m_ch1 = new("CH1", ChartDataSeries.EFormat.XRealYReal);
		private readonly ChartDataSeries m_ch2 = new("CH2", ChartDataSeries.EFormat.XRealYReal);

		// Framed mode replays a fixed hardware record whose ends do not meet (the record
		// spans a non-integer number of signal periods), so stitching it into a stream
		// leaves one large step at the record boundary. Rendering each channel as a
		// single connected line-strip would draw a spurious near-vertical segment across
		// that step. Splitting the trace at the boundary into a head and tail series
		// (both sharing the channel colour) leaves a one-sample gap there instead, so the
		// seam is not drawn while the trigger-centred framing is otherwise unchanged.
		private readonly ChartDataSeries m_ch1_tail = new("CH1t", ChartDataSeries.EFormat.XRealYReal);
		private readonly ChartDataSeries m_ch2_tail = new("CH2t", ChartDataSeries.EFormat.XRealYReal);
		private readonly DispatcherTimer m_render_timer;

		// Framed-mode rolling buffer of the most recent samples, redrawn from t=0 each
		// time new data arrives so the trace sits relative to the (software) trigger.
		private readonly List<Sample> m_frame = new();

		// Rolling buffer of the most recent samples used only for the auto-measurements
		// panel, independent of the display mode. Spanning several records gives the
		// frequency estimate enough cycles for a stable reading; the cap bounds the cost
		// of the periodic measurement pass.
		private readonly List<Sample> m_recent = new();
		private const int MeasureSamples = 6000;

		// Down-count of render ticks between measurement updates. The trace renders at
		// ~60 fps but the measurement panel only needs refreshing a few times a second,
		// and a slower cadence keeps the numbers readable rather than flickering.
		private const int MeasureTickInterval = 12;
		private int m_measure_ticks;

		// Absolute time (Sample.XMs) of the last software-trigger crossing shown at x=0.
		// A given crossing keeps the same XMs across frames as the buffer scrolls, so
		// tracking it here lets the display lock onto the same edge frame-to-frame for a
		// steady phase, re-anchoring by one period only when it rolls out of the buffer.
		// NaN means "no lock yet" (fall back to a centred pick / free-run).
		private double m_trig_xms = double.NaN;

		// Display colours. Background defaults to a dark gray; channels to a green / blue
		// pair. The trigger-level indicator for each channel uses a darker shade of the
		// trace colour so the level line/tag reads as related-to but distinct-from the
		// trace. The trace colours are user-editable via the Appearance menu; the trigger
		// shades are fixed companions to the defaults.
		private Colour32 m_bg_colour = new(0xFF2A2A2Au);
		private Colour32 m_ch1_colour = new(0xFF3ABB0Cu);
		private Colour32 m_ch2_colour = new(0xFF2AA7F9u);
		private Colour32 m_ch1_trig_colour = new(0xFF2D910Bu);
		private Colour32 m_ch2_trig_colour = new(0xFF0099F9u);

		// Display mode and per-axis auto-resolution flags.
		private bool m_scrolling;
		private bool m_xauto = true;
		private bool m_yauto = true;

		// Checkable menu items kept so their IsChecked state mirrors the flags above.
		private MenuItem m_mi_scrolling = null!;
		private MenuItem m_mi_xauto = null!;
		private MenuItem m_mi_yauto = null!;

		// Callbacks that sync each checkable/radio scope-control menu item to the model's
		// desired config; all are invoked when the scene context menu opens so the menu
		// always reflects the live state without one-off wiring per item.
		private readonly List<Action> m_menu_refreshers = new();

		// The two point-and-click Trigger items, whose headers show the volts/time at the
		// right-click position, plus the chart coordinates captured when the menu opened.
		private MenuItem m_mi_trig_level = null!;
		private MenuItem m_mi_trig_time = null!;
		private double m_click_volts;
		private double m_click_seconds;

		private double m_latest_x;
		private bool m_have_xrange;
		private bool m_have_yrange;

		// Set while the code is programmatically moving the camera (auto-fit, follow,
		// reset, frame redraw). The chart raises ChartMoved for these just as it does
		// for user navigation, so the flag lets OnChartMoved ignore self-induced moves
		// and only run auto-resolution in response to genuine user zooming.
		private bool m_suppress_auto;

		// Draggable trigger/position indicators drawn on the chart's overlay canvas.
		private ScopeOverlays m_overlays = null!;

		// Master show/hide for the trigger level/time indicators (context-menu toggle).
		private bool m_show_trigger = true;

		// Framed-mode horizontal display offset (ms): the software-trigger crossing is
		// drawn at this x instead of x=0, so dragging the time tag pans the framed trace.
		private double m_trig_offset_ms;

		// Per-channel vertical display offsets (volts) added to the plotted trace so each
		// channel's zero reference can be moved independently. Display-only: the recent
		// buffer that feeds the measurements panel keeps the true, un-offset volts.
		private double m_ch1_offset_v;
		private double m_ch2_offset_v;

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
			ConfigureSeries(m_ch1_tail, m_ch1_colour);
			ConfigureSeries(m_ch2_tail, m_ch2_colour);

			// Start with a sensible window so the empty chart isn't degenerate.
			SetRange(() =>
			{
				m_chart.Range.XAxis.Set(0, FollowWindowMs);
				m_chart.Range.YAxis.Set(-5, 5);
			});

			BuildContextMenus();
			m_chart.ChartMoved += OnChartMoved;

			// Draggable trigger/position overlays live on the chart's overlay canvas.
			// Each drag callback updates the owning state (and the device, for the
			// trigger) then leaves RepositionOverlays to re-sync the visuals.
			m_overlays = new ScopeOverlays(m_chart, m_ch1_colour, m_ch2_colour, m_ch1_trig_colour, m_ch2_trig_colour)
			{
				Ch1LevelDragged = v => OnDragTriggerLevel(v),
				Ch2LevelDragged = v => OnDragTriggerLevel(v),
				TimeDragged = OnDragTriggerTime,
				Ch1Dragged = v => OnDragChannelOffset(1, v),
				Ch2Dragged = v => OnDragChannelOffset(2, v),
			};

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
			scene.Items.Add(BuildChannelsMenu());
			scene.Items.Add(BuildTriggerMenu());
			scene.Items.Add(new Separator());
			scene.Items.Add(m_mi_scrolling);
			scene.Opened += OnSceneMenuOpened;
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
		/// Create a checkable menu item that reports a bool state. The click action runs
		/// the toggle; a refresher is registered so the checkmark reflects 'get_state'
		/// whenever the menu reopens (rather than trusting WPF's optimistic auto-toggle).
		/// </summary>
		private MenuItem MakeCheck(string header, Func<bool> get_state, Action on_toggle)
		{
			var item = new MenuItem { Header = header, IsCheckable = true };
			item.Click += (s, e) => on_toggle();
			m_menu_refreshers.Add(() => item.IsChecked = get_state());
			return item;
		}

		/// <summary>
		/// Create one option of a mutually-exclusive radio group. Selecting it applies
		/// 'value'; a refresher checks it when 'get_current' equals 'value', so exactly
		/// the active option is ticked each time the menu opens.
		/// </summary>
		private MenuItem MakeRadio<T>(string header, T value, Func<T> get_current, Action<T> on_select)
		{
			var item = new MenuItem { Header = header, IsCheckable = true };
			item.Click += (s, e) => on_select(value);
			m_menu_refreshers.Add(() => item.IsChecked = EqualityComparer<T>.Default.Equals(get_current(), value));
			return item;
		}

		/// <summary>Build the Channels submenu: per-channel enable, coupling, and probe controls.</summary>
		private MenuItem BuildChannelsMenu()
		{
			var channels = new MenuItem { Header = "Channels" };
			channels.Items.Add(BuildChannelMenu(1));
			channels.Items.Add(BuildChannelMenu(2));
			return channels;
		}

		/// <summary>Build one channel's submenu (Enabled toggle + Coupling and Probe radio groups).</summary>
		private MenuItem BuildChannelMenu(int channel)
		{
			var root = new MenuItem { Header = $"CH{channel}" };

			// Enabled toggle. The new state is computed at click time from the model so
			// the ≥1-enabled rule is enforced by the model, not assumed here.
			root.Items.Add(MakeCheck("Enabled",
				() => ChannelEnabled(channel),
				() => m_model.RequestChannelEnabled(channel, !ChannelEnabled(channel))));
			root.Items.Add(new Separator());

			var coupling = new MenuItem { Header = "Coupling" };
			foreach (var c in new[] { ECoupling.AC, ECoupling.DC, ECoupling.GND })
			{
				coupling.Items.Add(MakeRadio(c.ToString(), c,
					() => ChannelCoupling(channel),
					v => m_model.RequestCoupling(channel, v)));
			}
			root.Items.Add(coupling);

			var probe = new MenuItem { Header = "Probe" };
			foreach (var p in new[] { EProbeScale.X1, EProbeScale.X10, EProbeScale.X100, EProbeScale.X1000 })
			{
				probe.Items.Add(MakeRadio(ProbeLabel(p), p,
					() => ChannelProbe(channel),
					v => m_model.RequestProbe(channel, v)));
			}
			root.Items.Add(probe);
			return root;
		}

		/// <summary>Build the Trigger submenu: point-and-click level/time plus source, slope, and sweep.</summary>
		private MenuItem BuildTriggerMenu()
		{
			var trigger = new MenuItem { Header = "Trigger" };

			// The level/time headers are filled in by OnSceneMenuOpened from the click
			// point; selecting them applies that value to the trigger.
			m_mi_trig_level = MakeItem("Trigger Level", OnSetTriggerLevel);
			m_mi_trig_time = MakeItem("Time Offset", OnSetTriggerTime);
			trigger.Items.Add(m_mi_trig_level);
			trigger.Items.Add(m_mi_trig_time);
			trigger.Items.Add(new Separator());

			var source = new MenuItem { Header = "Source" };
			source.Items.Add(MakeRadio("CH1", ETriggerSource.Ch1, () => m_model.TriggerSource, v => m_model.RequestTriggerSource(v)));
			source.Items.Add(MakeRadio("CH2", ETriggerSource.Ch2, () => m_model.TriggerSource, v => m_model.RequestTriggerSource(v)));
			trigger.Items.Add(source);

			var slope = new MenuItem { Header = "Slope" };
			slope.Items.Add(MakeRadio("Rising", ETriggerSlope.Rising, () => m_model.TriggerSlope, v => m_model.RequestTriggerSlope(v)));
			slope.Items.Add(MakeRadio("Falling", ETriggerSlope.Falling, () => m_model.TriggerSlope, v => m_model.RequestTriggerSlope(v)));
			slope.Items.Add(MakeRadio("Both", ETriggerSlope.Both, () => m_model.TriggerSlope, v => m_model.RequestTriggerSlope(v)));
			trigger.Items.Add(slope);

			var sweep = new MenuItem { Header = "Sweep" };
			sweep.Items.Add(MakeRadio("Auto", ETriggerSweep.Auto, () => m_model.TriggerSweep, v => m_model.RequestTriggerSweep(v)));
			sweep.Items.Add(MakeRadio("Normal", ETriggerSweep.Normal, () => m_model.TriggerSweep, v => m_model.RequestTriggerSweep(v)));
			trigger.Items.Add(sweep);

			// Master show/hide for the on-chart trigger indicators (level line + time line).
			trigger.Items.Add(new Separator());
			trigger.Items.Add(MakeCheck("Show Indicators",
				() => m_show_trigger,
				() => { m_show_trigger = !m_show_trigger; m_overlays.ShowTrigger = m_show_trigger; }));

			return trigger;
		}

		/// <summary>Desired enabled state for a channel (1 or 2).</summary>
		private bool ChannelEnabled(int channel)
		{
			switch (channel)
			{
				case 1: return m_model.Ch1Enabled;
				case 2: return m_model.Ch2Enabled;
				default: throw new ArgumentOutOfRangeException(nameof(channel));
			}
		}

		/// <summary>Desired coupling for a channel (1 or 2).</summary>
		private ECoupling ChannelCoupling(int channel)
		{
			switch (channel)
			{
				case 1: return m_model.Ch1Coupling;
				case 2: return m_model.Ch2Coupling;
				default: throw new ArgumentOutOfRangeException(nameof(channel));
			}
		}

		/// <summary>Desired probe attenuation for a channel (1 or 2).</summary>
		private EProbeScale ChannelProbe(int channel)
		{
			switch (channel)
			{
				case 1: return m_model.Ch1Probe;
				case 2: return m_model.Ch2Probe;
				default: throw new ArgumentOutOfRangeException(nameof(channel));
			}
		}

		/// <summary>Human-readable label for a probe attenuation.</summary>
		private static string ProbeLabel(EProbeScale probe)
		{
			switch (probe)
			{
				case EProbeScale.X1: return "×1";
				case EProbeScale.X10: return "×10";
				case EProbeScale.X100: return "×100";
				case EProbeScale.X1000: return "×1000";
				default: throw new ArgumentOutOfRangeException(nameof(probe));
			}
		}

		/// <summary>
		/// When the scene menu opens, capture the chart coordinates under the mouse for
		/// the point-and-click Trigger items and sync every checkable/radio item to the
		/// model's desired config.
		/// </summary>
		private void OnSceneMenuOpened(object sender, RoutedEventArgs e)
		{
			// Convert the mouse position (client → scene-panel → chart space) so the
			// captured point matches the chart's own coordinate system: x in ms, y in volts.
			var client_pt = Mouse.GetPosition(m_chart);
			var scene_pt = m_chart.TranslatePoint(client_pt, m_chart.Scene);
			var chart_pt = m_chart.SceneToChart(new v2((float)scene_pt.X, (float)scene_pt.Y));
			m_click_volts = chart_pt.y;
			m_click_seconds = chart_pt.x / 1000.0;

			m_mi_trig_level.Header = $"Trigger Level: {m_click_volts:0.000} V";
			m_mi_trig_time.Header = $"Time Offset: {FormatSeconds(m_click_seconds)}";

			foreach (var refresh in m_menu_refreshers)
				refresh();
		}

		/// <summary>Apply the trigger level at the captured right-click voltage.</summary>
		private void OnSetTriggerLevel(object sender, RoutedEventArgs e)
		{
			// The right-click Y is an absolute chart voltage; convert it to a threshold in
			// the source channel's true signal volts by removing that channel's display offset.
			var source_offset = m_model.TriggerSource == ETriggerSource.Ch2 ? m_ch2_offset_v : m_ch1_offset_v;
			m_model.RequestTriggerLevelVolts(m_click_volts - source_offset);
		}

		/// <summary>Apply the trigger horizontal position at the captured right-click time offset.</summary>
		private void OnSetTriggerTime(object sender, RoutedEventArgs e)
		{
			m_model.RequestTriggerTimeOffset(m_click_seconds);
		}

		/// <summary>Format a time offset with an SI-scaled unit (s / ms / µs / ns).</summary>
		private static string FormatSeconds(double seconds)
		{
			var mag = Math.Abs(seconds);
			if (mag >= 1.0) return $"{seconds:0.000} s";
			if (mag >= 1e-3) return $"{seconds * 1e3:0.000} ms";
			if (mag >= 1e-6) return $"{seconds * 1e6:0.000} µs";
			return $"{seconds * 1e9:0.0} ns";
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
			{
				ApplySeriesColour(m_ch1, m_ch1_colour);
				ApplySeriesColour(m_ch1_tail, m_ch1_colour);
				m_overlays.SetChannelColours(m_ch1_colour, m_ch2_colour);
			}
		}

		/// <summary>Pick the CH2 trace colour.</summary>
		private void OnPickCh2Colour(object sender, RoutedEventArgs e)
		{
			if (PickColour(ref m_ch2_colour))
			{
				ApplySeriesColour(m_ch2, m_ch2_colour);
				ApplySeriesColour(m_ch2_tail, m_ch2_colour);
				m_overlays.SetChannelColours(m_ch1_colour, m_ch2_colour);
			}
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
			// The overlay visuals are positioned in scene space, so they must be re-laid
			// whenever the camera moves — including self-induced moves that return early
			// below.
			m_overlays?.Reposition();

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
				// Keep a mode-independent recent buffer for the measurements panel.
				m_recent.AddRange(samples);
				if (m_recent.Count > MeasureSamples)
					m_recent.RemoveRange(0, m_recent.Count - MeasureSamples);

				if (m_scrolling)
					RenderScrolling(samples);
				else
					RenderFramed(samples);
			}

			// Refresh the measurements a few times a second rather than every frame.
			if (--m_measure_ticks <= 0)
			{
				m_measure_ticks = MeasureTickInterval;
				UpdateMeasurements();
			}

			RepositionOverlays();
			m_chart.Invalidate();
		}

		/// <summary>
		/// Push the current trigger/offset state into the overlay helper and re-lay its
		/// visuals. The level line shows in both display modes; the time line only makes
		/// sense in framed mode (scrolling has no trigger-centred window), and the
		/// per-channel tabs follow each channel's enabled state.
		/// </summary>
		private void RepositionOverlays()
		{
			if (m_overlays == null)
				return;

			m_overlays.Framed = !m_scrolling;
			m_overlays.ShowTrigger = m_show_trigger;
			m_overlays.ActiveSourceChannel = m_model.TriggerSource == ETriggerSource.Ch2 ? 2 : 1;

			// One hardware trigger => a single shared level. Feed it to both channel
			// indicators; each draws it relative to its own baseline (offset) so the two
			// lines separate vertically only by the channels' vertical-position difference.
			var trig_volts = m_model.TriggerLevelVolts;
			m_overlays.Ch1LevelVolts = trig_volts;
			m_overlays.Ch2LevelVolts = trig_volts;
			m_overlays.Ch1LevelText = FormatVolts(trig_volts);
			m_overlays.Ch2LevelText = FormatVolts(trig_volts);
			m_overlays.TriggerTimeMs = m_trig_offset_ms;
			m_overlays.TimeText = FormatTime(m_trig_offset_ms / 1000.0);

			// The tab marks each channel's zero reference at its current display offset.
			m_overlays.Ch1TabVisible = ChannelEnabled(1);
			m_overlays.Ch2TabVisible = ChannelEnabled(2);
			m_overlays.Ch1OffsetVolts = m_ch1_offset_v;
			m_overlays.Ch2OffsetVolts = m_ch2_offset_v;

			m_overlays.Reposition();
		}

		/// <summary>
		/// A trigger-level tab was dragged. There is one shared hardware trigger, so the
		/// dragged value (already expressed as a threshold in that channel's true signal
		/// volts) becomes the single trigger level for both channels. Both tags show the
		/// same absolute voltage; each line then draws relative to its channel's baseline.
		/// </summary>
		private void OnDragTriggerLevel(double volts)
		{
			m_model.RequestTriggerLevelVolts(volts);
			m_overlays.Ch1LevelText = FormatVolts(volts);
			m_overlays.Ch2LevelText = FormatVolts(volts);
			m_chart.Invalidate();
		}

		/// <summary>
		/// Trigger-time tab dragged (framed mode): store the horizontal display offset,
		/// write the hardware horizontal-position register, and re-lay the framed trace
		/// immediately so the shift is visible even between polls.
		/// </summary>
		private void OnDragTriggerTime(double offset_ms)
		{
			m_trig_offset_ms = offset_ms;
			m_model.RequestTriggerTimeOffset(offset_ms / 1000.0);
			m_overlays.TriggerTimeMs = offset_ms;
			m_overlays.TimeText = FormatTime(offset_ms / 1000.0);
			if (!m_scrolling)
				RebuildFrame();
			m_chart.Invalidate();
		}

		/// <summary>
		/// Vertical-position tab dragged: move one channel's trace on the display only.
		/// In framed mode the trace is rebuilt with the new offset; in scrolling mode the
		/// already-plotted points are shifted by the delta in-place (that mode keeps no
		/// per-sample buffer to rebuild from). The measurement buffer is never touched, so
		/// the reported values stay true to the real signal.
		/// </summary>
		private void OnDragChannelOffset(int channel, double volts)
		{
			switch (channel)
			{
				case 1:
				{
					var delta = volts - m_ch1_offset_v;
					m_ch1_offset_v = volts;
					m_overlays.Ch1OffsetVolts = volts;
					if (m_scrolling)
						ShiftScrollingChannel(m_ch1, delta);
					break;
				}
				case 2:
				{
					var delta = volts - m_ch2_offset_v;
					m_ch2_offset_v = volts;
					m_overlays.Ch2OffsetVolts = volts;
					if (m_scrolling)
						ShiftScrollingChannel(m_ch2, delta);
					break;
				}
				default:
				{
					throw new ArgumentOutOfRangeException(nameof(channel), channel, "Unknown channel");
				}
			}

			// Command the hardware vertical-position register so the physical scope moves
			// with the display. Decode compensates for the vpos code, so measurements
			// remain true to the real signal.
			m_model.RequestChannelVPos(channel, volts);

			if (!m_scrolling)
				RebuildFrame();
			m_chart.Invalidate();
		}

		/// <summary>Shift every plotted point of a scrolling series vertically by a delta (single lock pass).</summary>
		private static void ShiftScrollingChannel(ChartDataSeries series, double delta_v)
		{
			if (delta_v == 0.0)
				return;

			using var lk = series.Lock();
			for (var i = 0; i != lk.Count; ++i)
			{
				var p = lk[i];
				lk[i] = new ChartDataSeries.Pt(p.x, p.y + delta_v);
			}
		}

		/// <summary>Recompute and display the auto-measurements over the recent-sample buffer.</summary>
		private void UpdateMeasurements()
		{
			var dt_s = m_model.SampleIntervalS;

			// Treat a channel with less than half a division of swing as flat, so its
			// timing reads as blank instead of measuring noise. Both channels share the
			// volts/div setting, so one threshold serves both.
			var min_vpp = 0.5 * m_model.VoltsPerDiv;

			var ch1 = Measurements.Compute(m_recent, static s => s.Ch1V, dt_s, min_vpp);
			var ch2 = Measurements.Compute(m_recent, static s => s.Ch2V, dt_s, min_vpp);

			ShowChannelMeasurements(ch1, m_c1_vpp, m_c1_vmax, m_c1_vmin, m_c1_mean, m_c1_rms, m_c1_freq, m_c1_period);
			ShowChannelMeasurements(ch2, m_c2_vpp, m_c2_vmax, m_c2_vmin, m_c2_mean, m_c2_rms, m_c2_freq, m_c2_period);
		}

		/// <summary>Write one channel's measurements into its panel text blocks.</summary>
		private static void ShowChannelMeasurements(ChannelMeasurements m, TextBlock vpp, TextBlock vmax, TextBlock vmin, TextBlock mean, TextBlock rms, TextBlock freq, TextBlock period)
		{
			vpp.Text = FormatVolts(m.Vpp);
			vmax.Text = FormatVolts(m.Vmax);
			vmin.Text = FormatVolts(m.Vmin);
			mean.Text = FormatVolts(m.Vmean);
			rms.Text = FormatVolts(m.Vrms);
			freq.Text = FormatFreq(m.FrequencyHz);
			period.Text = FormatTime(m.PeriodS);
		}

		/// <summary>Format a voltage with an auto-ranged unit, or an em dash when undefined.</summary>
		private static string FormatVolts(double v)
		{
			if (double.IsNaN(v))
				return "\u2014";

			var a = Math.Abs(v);
			if (a >= 1.0)
				return $"{v:0.###} V";
			if (a >= 1e-3)
				return $"{v * 1e3:0.###} mV";
			return $"{v * 1e6:0.###} \u00b5V";
		}

		/// <summary>Format a frequency with an auto-ranged unit, or an em dash when undefined.</summary>
		private static string FormatFreq(double hz)
		{
			if (double.IsNaN(hz))
				return "\u2014";

			if (hz >= 1e6)
				return $"{hz / 1e6:0.000} MHz";
			if (hz >= 1e3)
				return $"{hz / 1e3:0.000} kHz";
			return $"{hz:0.0} Hz";
		}

		/// <summary>Format a time with an auto-ranged unit, or an em dash when undefined.</summary>
		private static string FormatTime(double s)
		{
			if (double.IsNaN(s))
				return "\u2014";

			var a = Math.Abs(s);
			if (a >= 1.0)
				return $"{s:0.###} s";
			if (a >= 1e-3)
				return $"{s * 1e3:0.###} ms";
			if (a >= 1e-6)
				return $"{s * 1e6:0.###} \u00b5s";
			return $"{s * 1e9:0.###} ns";
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
				{
					if (!double.IsNaN(s.Ch1V))
						lk.Add(new ChartDataSeries.Pt(s.XMs, s.Ch1V + m_ch1_offset_v));
				}
				TrimOldest(lk);
			}

			using (var lk = m_ch2.Lock())
			{
				foreach (var s in samples)
				{
					if (!double.IsNaN(s.Ch2V))
						lk.Add(new ChartDataSeries.Pt(s.XMs, s.Ch2V + m_ch2_offset_v));
				}
				TrimOldest(lk);
			}

			// Keep the newest data at the right edge, preserving the user's zoom span.
			m_latest_x = samples[^1].XMs;
			var span = m_chart.Range.XAxis.Span;
			SetRange(() => m_chart.Range.XAxis.Set(m_latest_x - span, m_latest_x));
		}

		/// <summary>
		/// Framed mode: accumulate the most recent samples and redraw a one-record-wide
		/// window centred on a software trigger crossing. The hardware triggers
		/// internally, but stitching its records back into a contiguous stream loses the
		/// per-record phase, so a free-running trace scrolls. Re-triggering on our own
		/// stream re-establishes a stable phase: the crossing is pinned to x=0, with half
		/// a record of pre-trigger to the left and half to the right (matching the
		/// hardware's centred trigger). If no crossing is found (Auto sweep, or a flat /
		/// disabled source) the view free-runs on the latest record so it never freezes.
		/// </summary>
		private void RenderFramed(List<Sample> samples)
		{
			m_frame.AddRange(samples);

			// Trim to the two-record buffer so the rebuild and search stay bounded.
			if (m_frame.Count > MaxFramePoints)
				m_frame.RemoveRange(0, m_frame.Count - MaxFramePoints);

			RebuildFrame();
		}

		/// <summary>
		/// Rebuild the framed trace from the current frame buffer without ingesting new
		/// samples. Split out from RenderFramed so a drag of the time / vertical-position
		/// tags can re-lay the trace immediately (with the new display offsets) even when
		/// acquisition is paused or between polls.
		/// </summary>
		private void RebuildFrame()
		{
			// Per-sample spacing at the currently applied timebase.
			var dt_ms = m_model.SampleIntervalS * 1000.0;
			if (dt_ms <= 0)
				dt_ms = 1.0;

			// The displayed window is one full record; the hardware trigger sits at its
			// centre, matching what the instrument itself shows.
			var window = FrameWindowSamples;
			var half = window / 2;

			// Align the window to a device record boundary so the display reproduces one
			// complete hardware-triggered record exactly as the instrument captured it.
			// The device replays a fixed-length record whose trigger is at its centre, so
			// aligning start to the record boundary places the trigger at x=0 and — unlike
			// centring on an arbitrary software-trigger crossing — guarantees the window
			// never straddles two differently-phased records (which produced the seam). A
			// still-filling first record shows from the left so the trace appears promptly.
			var count = m_frame.Count;
			var start = 0;
			if (count >= window)
				start = FindRecordStart(count, window);
			var length = Math.Min(window, count);
			if (length == 0)
				return;

			// Keep fitting the Y axis to the displayed window until a full record has been
			// buffered, then latch it. Fitting only on the first tick would scale to the
			// first partial packet (~32 samples) and clip the rest of the record.
			if (!m_have_yrange)
			{
				FitYRange(m_frame.GetRange(start, length));
				if (count >= window)
					m_have_yrange = true;
			}

			// Map buffer index start+k to a local index k, then to x = (k - half)*dt so
			// the trigger (k == half) lands at x=0 with pre-trigger to the left. The
			// per-channel vertical offset and the shared horizontal time offset are baked
			// into the plotted values so the display can be shifted without touching the
			// measurement buffer. Because the window is aligned to a record boundary, the
			// boundary step sits at the window edge, not inside it, so no internal seam is
			// normally found; the head/tail split remains as a guard for the rare case a
			// second boundary falls inside the window (records not exactly window-aligned).
			var seam = FindSeamOffset(start, length);
			FillFramedChannel(m_ch1, m_ch1_tail, start, length, half, dt_ms, seam, m_trig_offset_ms, m_ch1_offset_v, static s => s.Ch1V);
			FillFramedChannel(m_ch2, m_ch2_tail, start, length, half, dt_ms, seam, m_trig_offset_ms, m_ch2_offset_v, static s => s.Ch2V);

			// Fit the X axis once to a full record width centred on the trigger, so the
			// window always spans one complete record (12 divisions) with the trigger at
			// x=0. OnConfigChanged resets m_have_xrange so this recomputes on a timebase
			// change.
			if (!m_have_xrange)
			{
				SetRange(() => m_chart.Range.XAxis.Set(-half * dt_ms, (window - half) * dt_ms));
				m_have_xrange = true;
			}
		}

		/// <summary>
		/// Fill a channel's head/tail series pair from the frame window [start, start+length).
		/// Local index k maps to x = (k - half)*dt_ms. When <paramref name="seam"/> is a valid
		/// local offset, samples before it go to <paramref name="main"/> and samples from it
		/// onward go to <paramref name="tail"/>, leaving the record-boundary step as a gap;
		/// when it is negative the whole window goes to <paramref name="main"/> and the tail is
		/// cleared. Both series are cleared every call so a seam that comes and goes as the
		/// window scrolls never leaves a stale head or tail behind.
		/// </summary>
		private void FillFramedChannel(ChartDataSeries main, ChartDataSeries tail, int start, int length, int half, double dt_ms, int seam, double x_offset_ms, double y_offset, Func<Sample, double> selector)
		{
			var split = seam < 0 ? length : seam;

			using (var lk = main.Lock())
			{
				lk.Clear();
				for (var k = 0; k != split; ++k)
				{
					var v = selector(m_frame[start + k]);
					if (!double.IsNaN(v))
						lk.Add(new ChartDataSeries.Pt((k - half) * dt_ms + x_offset_ms, v + y_offset));
				}
			}

			using (var lk = tail.Lock())
			{
				lk.Clear();
				for (var k = split; k != length; ++k)
				{
					var v = selector(m_frame[start + k]);
					if (!double.IsNaN(v))
						lk.Add(new ChartDataSeries.Pt((k - half) * dt_ms + x_offset_ms, v + y_offset));
				}
			}
		}

		/// <summary>
		/// Choose the window start that aligns the display to a device record boundary, so
		/// the frame reproduces one complete hardware-triggered record. The device replays
		/// a fixed-length record whose replay wraps at the record boundary, producing a
		/// single large phase step in the buffered stream. Aligning start to that boundary
		/// puts the boundary at the window edge (not inside it) and the record's centred
		/// trigger at x=0. When the record that starts at the boundary has not fully
		/// arrived yet, fall back to the previous complete record ending at the boundary.
		///
		/// When no boundary is detectable — the successive records are effectively in phase
		/// (so there is no visible seam to avoid) or the signal is not periodic — fall back
		/// to software-trigger centring, which holds a steady phase without a boundary to
		/// anchor to.
		/// </summary>
		private int FindRecordStart(int count, int window)
		{
			var half = window / 2;
			var boundary = FindSeamOffset(0, count);
			if (boundary >= 0)
			{
				// Boundary-aligned: no software-trigger lock is needed, so clear it.
				m_trig_xms = double.NaN;
				var start = count - boundary >= window ? boundary : boundary - window;
				return Math.Clamp(start, 0, Math.Max(0, count - window));
			}

			// No boundary: centre a software-trigger crossing, tracking the same crossing
			// across frames for phase stability. Without a boundary there is no seam that
			// centring could expose.
			var ti = FindTriggerIndex(half, count - half, m_trig_xms);
			if (ti >= 0)
			{
				m_trig_xms = m_frame[ti].XMs;
				return ti - half;
			}
			m_trig_xms = double.NaN;
			return Math.Max(0, count - window);
		}

		/// <summary>
		/// Find the record-boundary step within the frame window [start, start+length), or -1
		/// when none is present. The device replays a fixed-length hardware-triggered record,
		/// so where the window straddles a boundary the two record ends (captured at different
		/// signal phases) meet in a single-sample step far larger than the sample-to-sample
		/// change of the band-limited trace. Both channels share the boundary sample index, so
		/// whichever channel shows the clearer step defines the split point for both.
		/// </summary>
		private int FindSeamOffset(int start, int length)
		{
			var ch1 = SeamCandidate(start, length, static s => s.Ch1V);
			var ch2 = SeamCandidate(start, length, static s => s.Ch2V);

			// Prefer the larger, more confident step; both channels wrap at the same index.
			if (ch1.Index < 0)
				return ch2.Index;
			if (ch2.Index < 0)
				return ch1.Index;
			return ch1.Jump >= ch2.Jump ? ch1.Index : ch2.Index;
		}

		/// <summary>
		/// Locate the largest single-sample step on one channel within the window and return it
		/// only when it is an extreme outlier (an order of magnitude above the mean step), which
		/// isolates the record-boundary discontinuity from ordinary signal slope and noise.
		/// Returns the local offset of the first sample of the new record and the step size, or
		/// (-1, 0) when there is no clear boundary (e.g. a flat or absent channel).
		/// </summary>
		private (int Index, double Jump) SeamCandidate(int start, int length, Func<Sample, double> selector)
		{
			var best = -1;
			var best_jump = 0.0;
			var sum = 0.0;
			var n = 0;
			for (var k = 1; k != length; ++k)
			{
				var prev = selector(m_frame[start + k - 1]);
				var curr = selector(m_frame[start + k]);
				if (double.IsNaN(prev) || double.IsNaN(curr))
					continue;

				var jump = Math.Abs(curr - prev);
				sum += jump;
				++n;
				if (jump > best_jump)
				{
					best_jump = jump;
					best = k;
				}
			}

			if (n == 0)
				return (-1, 0.0);

			// The mean is dominated by the many small steps, so the single boundary step barely
			// perturbs it; requiring a 10x margin cleanly separates a real seam from noise.
			var mean = sum / n;
			return best_jump >= 10.0 * mean && best_jump > 0.0 ? (best, best_jump) : (-1, 0.0);
		}

		/// <summary>
		/// Find the trigger crossing on the source channel within [lo, hi). A crossing
		/// needs a valid (non-NaN) sample pair straddling the trigger level with the
		/// configured slope. Returns the index of the sample at or past the level, or -1
		/// when no crossing exists (free-run).
		///
		/// Selection keeps the displayed phase steady. The frame buffer is a sliding
		/// window over a continuous stream, so any rule that picks "a crossing" can hop to
		/// an adjacent one as the window scrolls; a one-period hop is invisible for a truly
		/// periodic trace, but the record-stitch discontinuity is not periodic, so a hop
		/// would make it jump across the screen. When a previous crossing time is known
		/// (target_xms), the crossing with the nearest absolute time is chosen: the same
		/// physical edge carries the same time every frame, so it stays locked until it
		/// rolls out of range, then the next edge (one period on) is picked. With no prior
		/// lock the crossing nearest the centre of the span is used, for maximal headroom.
		/// </summary>
		private int FindTriggerIndex(int lo, int hi, double target_xms)
		{
			var source = m_model.TriggerSource;
			var slope = m_model.TriggerSlope;
			var level = m_model.TriggerLevelVolts;

			// Selection key: distance in absolute time from the tracked crossing, or, with
			// no lock yet, distance in samples from the centre of the search span.
			var have_target = !double.IsNaN(target_xms);
			var centre = (lo + hi) / 2;

			var best = -1;
			var best_key = double.MaxValue;
			for (var i = lo; i != hi; ++i)
			{
				var prev = SourceVolts(m_frame[i - 1], source);
				var curr = SourceVolts(m_frame[i], source);
				if (double.IsNaN(prev) || double.IsNaN(curr))
					continue;

				var rising = prev < level && curr >= level;
				var falling = prev > level && curr <= level;
				var hit = slope switch
				{
					ETriggerSlope.Rising => rising,
					ETriggerSlope.Falling => falling,
					ETriggerSlope.Both => rising || falling,
					_ => throw new ArgumentOutOfRangeException(nameof(slope)),
				};
				if (!hit)
					continue;

				var key = have_target ? Math.Abs(m_frame[i].XMs - target_xms) : Math.Abs(i - centre);
				if (key < best_key)
				{
					best_key = key;
					best = i;
				}
			}
			return best;
		}

		/// <summary>Select the trigger source channel's voltage from a sample.</summary>
		private static double SourceVolts(Sample s, ETriggerSource source)
		{
			return source switch
			{
				ETriggerSource.Ch1 => s.Ch1V,
				ETriggerSource.Ch2 => s.Ch2V,
				_ => throw new ArgumentOutOfRangeException(nameof(source)),
			};
		}

		/// <summary>Set the Y axis to span both channels of the batch, with headroom.</summary>
		private void FitYRange(List<Sample> samples)
		{
			var lo = double.MaxValue;
			var hi = double.MinValue;
			foreach (var s in samples)
			{
				if (!double.IsNaN(s.Ch1V))
				{
					lo = Math.Min(lo, s.Ch1V);
					hi = Math.Max(hi, s.Ch1V);
				}
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
			m_recent.Clear();
			m_trig_xms = double.NaN;
			m_latest_x = 0;
			m_have_xrange = false;
			m_have_yrange = false;

			using (var lk = m_ch1.Lock())
				lk.Clear();
			using (var lk = m_ch2.Lock())
				lk.Clear();
			using (var lk = m_ch1_tail.Lock())
				lk.Clear();
			using (var lk = m_ch2_tail.Lock())
				lk.Clear();

			// Blank the measurements until the next batch arrives.
			ShowChannelMeasurements(ChannelMeasurements.None, m_c1_vpp, m_c1_vmax, m_c1_vmin, m_c1_mean, m_c1_rms, m_c1_freq, m_c1_period);
			ShowChannelMeasurements(ChannelMeasurements.None, m_c2_vpp, m_c2_vmax, m_c2_vmin, m_c2_mean, m_c2_rms, m_c2_freq, m_c2_period);

			m_chart.Invalidate();
		}

		/// <summary>Marshal a model status update onto the UI thread.</summary>
		private void OnStatusChanged(string text)
		{
			Dispatcher.BeginInvoke(() =>
			{
				// Derive a connection state from the message for the status-bar LED. The
				// model recovers from device loss on its own (persistent reconnect), so a
				// transient failure shows as "Connecting" rather than a terminal error and
				// acquisition keeps running until the user presses Stop.
				var state =
					text.StartsWith("Error", StringComparison.OrdinalIgnoreCase) ? EConn.Error :
					text.StartsWith("Running", StringComparison.OrdinalIgnoreCase) ? EConn.Running :
					EConn.Connecting;
				SetConnection(text, state);
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
				// The measurement buffer is likewise rate- (and scale-) sensitive, so clear
				// it too rather than average across the resolution change.
				m_frame.Clear();
				m_recent.Clear();
				m_trig_xms = double.NaN;
				m_have_xrange = false;
				UpdateConfigStatus();
			});
		}

		/// <summary>Status-bar connection-state colours.</summary>
		private enum EConn { Stopped, Connecting, Running, Error }

		/// <summary>Update the status-bar connection text and LED colour.</summary>
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

		/// <summary>Tear down acquisition and the chart series on close.</summary>
		protected override void OnClosed(EventArgs e)
		{
			m_render_timer.Stop();
			m_model.Dispose();
			m_overlays?.Dispose();
			Util.Dispose(m_ch1);
			Util.Dispose(m_ch2);
			Util.Dispose(m_ch1_tail);
			Util.Dispose(m_ch2_tail);
			Gui_.DisposeChildren(this, EventArgs.Empty);
			base.OnClosed(e);
		}
	}
}
