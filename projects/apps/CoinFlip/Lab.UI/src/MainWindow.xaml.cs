using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;
using System.Windows;
using Binance.API;
using Lab.Model;
using Rylogic.Gfx;
using Rylogic.Gui.WPF;
using Rylogic.Utility;

namespace Lab.UI
{
	/// <summary>
	/// Milestone 1 shell: download Binance history into the local cache and scrub through it
	/// on a price chart. Later milestones add derived signals, transforms, and feature overlays.</summary>
	public partial class MainWindow : Window
	{
		private readonly MarketData m_data;
		private readonly CancellationTokenSource m_shutdown;
		private ChartDataSeries? m_series;
		private List<Candle> m_candles = new();
		private bool m_initialised;
		private bool m_loading;

		public MainWindow()
		{
			InitializeComponent();

			// Construct the data layer on the UI thread (the Binance client needs a SynchronizationContext).
			m_shutdown = new CancellationTokenSource();
			var db_path = Path.Combine(
				Environment.GetFolderPath(Environment.SpecialFolder.LocalApplicationData),
				"Rylogic", "Lab", "candles.db");
			m_data = new MarketData(db_path, m_shutdown.Token);

			// Chart setup - 2D orthographic view is what we want for time series.
			m_chart.Options.Orthographic = true;
			m_chart.Options.Antialiasing = true;

			// Populate the timeframe options.
			foreach (var tf in Enum.GetValues<EMarketPeriod>().Where(x => x != EMarketPeriod.None))
				m_cb_timeframe.Items.Add(tf);
			m_cb_timeframe.SelectedItem = EMarketPeriod.Day1;

			DataContext = this;
		}
		protected override void OnClosed(EventArgs e)
		{
			m_shutdown.Cancel();
			Util.Dispose(ref m_series!);
			m_data.Dispose();
			m_shutdown.Dispose();
			Gui_.DisposeChildren(this, EventArgs.Empty);
			base.OnClosed(e);
		}

		/// <summary>Chart title reflecting the current instrument</summary>
		public string ChartTitle { get; private set; } = "Price";

		/// <summary>Download (or read from cache) and display history</summary>
		private async void OnLoadClick(object sender, RoutedEventArgs e)
		{
			if (m_loading)
				return;

			var symbol = m_tb_symbol.Text.Trim().ToUpperInvariant();
			if (m_cb_timeframe.SelectedItem is not EMarketPeriod tf)
				return;
			if (!int.TryParse(m_tb_days.Text, out var days) || days <= 0)
			{
				m_tb_status.Text = "Enter a positive number of days";
				return;
			}

			m_loading = true;
			m_btn_load.IsEnabled = false;
			var progress = new Progress<string>(s => m_tb_status.Text = s);
			try
			{
				// Lazily initialise the exchange connection (symbol map + throttle limits).
				if (!m_initialised)
				{
					m_tb_status.Text = "Connecting to Binance...";
					await m_data.InitAsync();
					m_initialised = true;
				}

				var end = DateTimeOffset.UtcNow;
				var beg = end - TimeSpan.FromDays(days);

				// Download + cache on a worker thread; the DB and REST client are both usable off the UI thread.
				var candles = await Task.Run(
					() => m_data.GetHistoryAsync(symbol, tf, beg, end, progress, m_shutdown.Token),
					m_shutdown.Token);

				if (candles.Count == 0)
				{
					m_tb_status.Text = $"No data returned for {symbol} {tf}";
					return;
				}

				ShowCandles(symbol, tf, candles);
				m_tb_status.Text = $"{symbol} {tf}: {candles.Count:N0} candles " +
					$"({candles[0].TimeUtc:yyyy-MM-dd} to {candles[^1].TimeUtc:yyyy-MM-dd})";
			}
			catch (OperationCanceledException)
			{
				m_tb_status.Text = "Cancelled";
			}
			catch (Exception ex)
			{
				m_tb_status.Text = $"Error: {ex.Message}";
			}
			finally
			{
				m_loading = false;
				m_btn_load.IsEnabled = true;
			}
		}

		/// <summary>Build the price series and reset the scrub view</summary>
		private void ShowCandles(string symbol, EMarketPeriod tf, List<Candle> candles)
		{
			m_candles = candles;
			ChartTitle = $"{symbol} {tf} - Close";

			// Replace the price series (X = candle index, Y = close price).
			Util.Dispose(ref m_series!);
			m_series = new ChartDataSeries("Close", ChartDataSeries.EFormat.XRealYReal);
			m_series.Options.Colour = Colour32.DodgerBlue;
			m_series.Options.PlotType = ChartDataSeries.EPlotType.Line;
			m_series.Options.LineWidth = 1f;
			m_series.Chart = m_chart;
			{
				using var lk = m_series.Lock();
				for (var i = 0; i != candles.Count; ++i)
					lk.Add(new ChartDataSeries.Pt(i, candles[i].Close));
			}

			// Label the X axis with dates rather than raw indices.
			m_chart.XAxis.TickText = (x, step) =>
			{
				var i = (int)Math.Round(x);
				return i >= 0 && i < m_candles.Count
					? m_candles[i].TimeUtc.LocalDateTime.ToString("yyyy-MM-dd")
					: string.Empty;
			};

			m_chart.AutoRange();

			// Enable scrubbing and show the most recent window.
			m_slider.IsEnabled = true;
			if (m_slider.Value == 1.0)
				UpdateScrub();
			else
				m_slider.Value = 1.0; // triggers OnScrubChanged
		}

		/// <summary>Slider moved - reposition the visible window</summary>
		private void OnScrubChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
		{
			UpdateScrub();
		}

		/// <summary>Set the chart's visible X window (and fit Y to it) from the current scrub position</summary>
		private void UpdateScrub()
		{
			if (m_candles.Count == 0)
				return;

			var n = m_candles.Count;
			if (!int.TryParse(m_tb_window.Text, out var window) || window < 2)
				window = Math.Min(200, n);
			window = Math.Min(window, n);

			// Map slider [0,1] to the start index of the window.
			var max_start = Math.Max(0, n - window);
			var i0 = (int)Math.Round(m_slider.Value * max_start);
			var i1 = Math.Min(n - 1, i0 + window - 1);

			m_chart.XAxis.Set(i0 - 0.5, i1 + 0.5);

			// Fit Y to the visible candles with a small margin.
			var lo = double.MaxValue;
			var hi = double.MinValue;
			for (var i = i0; i <= i1; ++i)
			{
				lo = Math.Min(lo, m_candles[i].Low);
				hi = Math.Max(hi, m_candles[i].High);
			}
			if (lo <= hi)
			{
				var margin = 0.05 * Math.Max(1e-9, hi - lo);
				m_chart.YAxis.Set(lo - margin, hi + margin);
			}

			m_tb_scrub_date.Text =
				$"{m_candles[i0].TimeUtc.LocalDateTime:yyyy-MM-dd} .. {m_candles[i1].TimeUtc.LocalDateTime:yyyy-MM-dd}";

			// Apply the new axis ranges to the camera and redraw.
			m_chart.SetCameraFromRange();
			m_chart.Invalidate();
		}
	}
}
