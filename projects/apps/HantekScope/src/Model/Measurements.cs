using System;
using System.Collections.Generic;

namespace HantekScope.Model
{
	/// <summary>Automatic per-channel measurements computed over a window of samples.</summary>
	public readonly record struct ChannelMeasurements(bool Valid, double Vpp, double Vmax, double Vmin, double Vmean, double Vrms, double FrequencyHz, double PeriodS)
	{
		/// <summary>An empty result for a channel with no valid samples.</summary>
		public static ChannelMeasurements None => new(false, double.NaN, double.NaN, double.NaN, double.NaN, double.NaN, double.NaN, double.NaN);
	}

	/// <summary>Computes standard oscilloscope auto-measurements from decoded samples.</summary>
	public static class Measurements
	{
		/// <summary>
		/// Compute amplitude and timing measurements for one channel over <paramref name="window"/>.
		/// NaN samples (unfilled gaps) are ignored. <paramref name="dt_s"/> is the per-sample time
		/// spacing. <paramref name="min_vpp_for_freq"/> gates the frequency/period estimate: a channel
		/// whose peak-to-peak is below it is treated as flat (no real signal), so timing is reported as
		/// NaN rather than measuring noise.
		/// </summary>
		public static ChannelMeasurements Compute(IReadOnlyList<Sample> window, Func<Sample, double> selector, double dt_s, double min_vpp_for_freq)
		{
			// Amplitude pass: min/max for peak-to-peak, running sums for mean and RMS.
			var vmin = double.MaxValue;
			var vmax = double.MinValue;
			var sum = 0.0;
			var sum_sq = 0.0;
			var n = 0;
			for (var i = 0; i != window.Count; ++i)
			{
				var v = selector(window[i]);
				if (double.IsNaN(v))
					continue;

				vmin = Math.Min(vmin, v);
				vmax = Math.Max(vmax, v);
				sum += v;
				sum_sq += v * v;
				++n;
			}

			if (n == 0)
				return ChannelMeasurements.None;

			var mean = sum / n;
			var vpp = vmax - vmin;
			var rms = Math.Sqrt(sum_sq / n);

			var (freq, period) = EstimateFrequency(window, selector, mean, vpp, dt_s, min_vpp_for_freq);
			return new ChannelMeasurements(true, vpp, vmax, vmin, mean, rms, freq, period);
		}

		/// <summary>
		/// Estimate frequency and period from rising mean-level crossings within the window.
		///
		/// A hysteresis band around the mean rejects noise: a rising crossing only counts once the
		/// trace has first dipped below the low band, so small wiggles near the mean cannot produce
		/// spurious edges. The record the device replays contains large single-sample steps at its
		/// boundaries (a band-limited trace never moves that far between samples), which are phase
		/// discontinuities; a step larger than half the peak-to-peak breaks the crossing sequence into
		/// segments so no interval is ever measured across a boundary. Crossing positions are
		/// interpolated to sub-sample resolution, and the period is the median of the within-segment
		/// intervals, which is robust to the odd interval lost at a boundary or a NaN gap.
		/// </summary>
		private static (double FrequencyHz, double PeriodS) EstimateFrequency(IReadOnlyList<Sample> window, Func<Sample, double> selector, double mean, double vpp, double dt_s, double min_vpp_for_freq)
		{
			// Without a real signal (or a valid time base) there is nothing to time.
			if (vpp < min_vpp_for_freq || dt_s <= 0.0)
				return (double.NaN, double.NaN);

			var band = 0.10 * vpp;
			var lo = mean - band;
			var seam_step = 0.5 * vpp;

			var spacings = new List<double>();
			var last_cross = double.NaN;
			var prev = 0.0;
			var have_prev = false;
			var armed = false;
			for (var i = 0; i != window.Count; ++i)
			{
				var v = selector(window[i]);
				if (double.IsNaN(v))
				{
					// A gap ends the current segment; the next crossing starts a fresh one.
					have_prev = false;
					last_cross = double.NaN;
					armed = false;
					continue;
				}

				// A record-boundary step is a phase discontinuity, not a real edge: end the
				// segment so the interval spanning it is never counted.
				var seam = have_prev && Math.Abs(v - prev) > seam_step;
				if (seam)
				{
					last_cross = double.NaN;
					armed = false;
				}
				else if (have_prev && armed && prev < mean && v >= mean)
				{
					// Sub-sample crossing position by linear interpolation between the pair.
					var pos = (i - 1) + (mean - prev) / (v - prev);
					if (!double.IsNaN(last_cross))
						spacings.Add(pos - last_cross);
					last_cross = pos;
					armed = false;
				}

				if (v < lo)
					armed = true;

				prev = v;
				have_prev = true;
			}

			if (spacings.Count == 0)
				return (double.NaN, double.NaN);

			spacings.Sort();
			var mid = spacings.Count / 2;
			var period_samples = (spacings.Count & 1) == 1 ? spacings[mid] : 0.5 * (spacings[mid - 1] + spacings[mid]);

			var period_s = period_samples * dt_s;
			if (period_s <= 0.0)
				return (double.NaN, double.NaN);

			return (1.0 / period_s, period_s);
		}
	}
}
