using System;
using System.Windows;
using System.Windows.Threading;
using HantekScope.Model;
using Rylogic.Gfx;
using Rylogic.Gui.WPF;
using Rylogic.Utility;

namespace HantekScope.UI
{
	/// <summary>Main application window hosting the live oscilloscope trace.</summary>
	public partial class MainWindow :Window
	{
		// Cap on points kept per channel. The chart only renders the slice inside the
		// visible X range, so this bounds memory rather than render cost; when exceeded
		// the oldest chunk is dropped to keep the rolling stream bounded.
		private const int MaxPointsPerChannel = 200000;
		private const int TrimChunk = 50000;

		// How much signal-time (ms) the follow view shows ending at the latest sample.
		private const double FollowWindowMs = 5.0;

		private readonly ScopeModel m_model = new();
		private readonly ChartDataSeries m_ch1 = new("CH1", ChartDataSeries.EFormat.XRealYReal);
		private readonly ChartDataSeries m_ch2 = new("CH2", ChartDataSeries.EFormat.XRealYReal);
		private readonly DispatcherTimer m_render_timer;
		private double m_latest_x;
		private bool m_have_yrange;

		public MainWindow()
		{
			InitializeComponent();

			// 2D orthographic view with a black scene border, matching the scope look.
			m_chart.Options.Orthographic = true;
			m_chart.Options.SceneBorderThickness = 1;
			m_chart.Options.SceneBorderColour = Colour32.Black;

			// Model events arrive on the acquisition thread; marshal them to the UI.
			m_model.StatusChanged += OnStatusChanged;

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

		/// <summary>Configure the channel series once the chart's render device exists.</summary>
		private void OnLoaded(object sender, RoutedEventArgs e)
		{
			Loaded -= OnLoaded;

			// CH1 yellow, CH2 cyan, both drawn as connected lines.
			// PointsOnLinePlot defaults true, which would also build the point-sprite
			// shader; disable it so traces are pure lines (and so the unset PointSprite
			// shader is never referenced).
			m_ch1.Options.Colour = Colour32.Yellow;
			m_ch1.Options.PlotType = ChartDataSeries.EPlotType.Line;
			m_ch1.Options.PointsOnLinePlot = false;
			m_ch1.Options.LineWidth = 1f;
			m_ch1.Chart = m_chart;

			m_ch2.Options.Colour = Colour32.Cyan;
			m_ch2.Options.PlotType = ChartDataSeries.EPlotType.Line;
			m_ch2.Options.PointsOnLinePlot = false;
			m_ch2.Options.LineWidth = 1f;
			m_ch2.Chart = m_chart;

			// Start with a sensible window so the empty chart isn't degenerate.
			m_chart.Range.XAxis.Set(0, FollowWindowMs);
			m_chart.Range.YAxis.Set(-5, 5);
			m_chart.SetCameraFromRange();
		}

		/// <summary>Toggle acquisition on/off.</summary>
		private void OnRunStop(object sender, RoutedEventArgs e)
		{
			if (!m_model.IsRunning)
			{
				m_have_yrange = false;
				m_latest_x = 0;
				m_model.Start();
				m_render_timer.Start();
				m_btn_run.Content = "Stop";
				m_status.Text = "Connecting…";
			}
			else
			{
				m_render_timer.Stop();
				m_model.Stop();
				m_btn_run.Content = "Run";
				m_status.Text = "Stopped";
			}
		}

		/// <summary>Pull pending samples into the chart and (optionally) follow the latest.</summary>
		private void OnRenderTick(object? sender, EventArgs e)
		{
			var samples = m_model.DrainPending();
			if (samples.Count != 0)
			{
				AppendSamples(samples);
				m_latest_x = samples[^1].XMs;
			}

			// Follow mode keeps the newest data at the right edge; otherwise the user
			// is free to zoom/pan through the captured history.
			if (m_btn_follow.IsChecked == true && m_latest_x > 0)
			{
				var min = Math.Max(0, m_latest_x - FollowWindowMs);
				m_chart.Range.XAxis.Set(min, m_latest_x);
				m_chart.SetCameraFromRange();
			}

			m_chart.Invalidate();
		}

		/// <summary>Add a batch of decoded samples to the two channel series, trimming old data.</summary>
		private void AppendSamples(System.Collections.Generic.List<Sample> samples)
		{
			// Fit the Y axis to the first real batch so channel offsets (vpos) are visible.
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
		}

		/// <summary>Set the Y axis to span both channels of the first batch, with headroom.</summary>
		private void FitYRange(System.Collections.Generic.List<Sample> samples)
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

			// Guard against a degenerate (flat) first batch.
			if (lo >= hi)
			{
				lo -= 1.0;
				hi += 1.0;
			}

			var pad = 0.2 * (hi - lo);
			m_chart.Range.YAxis.Set(lo - pad, hi + pad);
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

		/// <summary>Marshal a model status update onto the UI thread.</summary>
		private void OnStatusChanged(string text)
		{
			Dispatcher.BeginInvoke(() =>
			{
				m_status.Text = text;

				// An error stops acquisition inside the model; reflect that in the UI.
				if (!m_model.IsRunning && m_render_timer.IsEnabled)
				{
					m_render_timer.Stop();
					m_btn_run.Content = "Run";
				}
			});
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
