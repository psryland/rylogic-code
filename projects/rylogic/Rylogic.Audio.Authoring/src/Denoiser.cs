using System;

namespace Rylogic.Audio.Authoring;

/// <summary>
/// Reduces steady background noise, such as tape or microphone hiss. The noise level of each narrow frequency band is learned from a section that holds
/// only noise. Afterwards, a band is turned down whenever it is not clearly louder than the noise, and left alone while it is. Wanted sound passes
/// unchanged, and the quiet gaps between sounds lose most of their hiss. Noise that is as loud as the wanted sound cannot be separated from it.
/// </summary>
public static class Denoiser
{
	/// <summary>The length of the analysis window, in frames. At 48 kHz this is about 43 ms, giving bands about 23 Hz wide.</summary>
	public const int WindowFrames = 2048;

	/// <summary>How far a band's power must rise above its average noise power, in decibels, before the band is treated as wanted sound.</summary>
	public const double ThresholdDb = 6.0;

	/// <summary>The time a band takes to open fully before a sound starts, so the start of a sound is not cut off, in seconds.</summary>
	public const double AttackSeconds = 0.02;

	/// <summary>The time a band takes to close fully after a sound stops, so a sound's natural decay is not cut short, in seconds.</summary>
	public const double ReleaseSeconds = 0.15;

	/// <summary>
	/// The number of bands on each side whose power is pooled with each band's when deciding whether it is open. Pooling stops random peaks of the noise
	/// opening single bands, which would otherwise be heard as a 'watery' flicker, while a strong narrow sound still opens its bands.
	/// </summary>
	public const int PooledBands = 3;

	/// <summary>Successive windows overlap by three quarters, which a Hann window needs to rebuild the audio without ripple.</summary>
	private const int Hop = WindowFrames / 4;

	/// <summary>The number of distinct frequency bands of a real signal's spectrum.</summary>
	private const int Bands = WindowFrames / 2 + 1;

	/// <summary>
	/// Reduce the noise in 'buffer' by up to 'reduction_db' decibels, learning the noise from frames 'noise_start' to 'noise_end'. The noise section must
	/// span at least <see cref="WindowFrames"/> frames and 'reduction_db' must be positive. Stereo channels share their gains, so the stereo image does not
	/// move. Returns a new buffer of the same length.
	/// </summary>
	public static AudioBuffer Process(AudioBuffer buffer, int noise_start, int noise_end, double reduction_db)
	{
		// The caller validates the parameters, so a violation here is a programming error
		if (noise_end - noise_start < WindowFrames)
			throw new ArgumentOutOfRangeException(nameof(noise_end), "The noise section must span at least one analysis window.");
		if (!(reduction_db > 0) || !double.IsFinite(reduction_db))
			throw new ArgumentOutOfRangeException(nameof(reduction_db), reduction_db, "The reduction must be a positive number of decibels.");

		// Each channel is padded by a whole window of silence at both ends, so every real frame is covered by the full set of overlapping windows
		var channels = buffer.Channels;
		var frames = buffer.Frames;
		var padded_length = (frames + 2 * WindowFrames + Hop - 1) / Hop * Hop;
		var windows = (padded_length - WindowFrames) / Hop + 1;
		var padded = new double[channels][];
		for (int c = 0; c != channels; ++c)
		{
			// The channel is copied out of the interleaved samples
			padded[c] = new double[padded_length];
			for (int i = 0; i != frames; ++i)
				padded[c][WindowFrames + i] = buffer.Samples[i * channels + c];
		}

		var fft = new Fft(WindowFrames);
		var window = HannWindow();
		var re = new double[WindowFrames];
		var im = new double[WindowFrames];

		// The noise profile is the average power of each band over the windows that lie wholly inside the noise section, summed over the channels
		var noise = new double[Bands];
		var noise_windows = 0;
		for (var start = noise_start; start + WindowFrames <= noise_end; start += Hop, ++noise_windows)
		{
			// Each window adds its power to the running sums
			for (int c = 0; c != channels; ++c)
			{
				Transform(fft, window, padded[c], WindowFrames + start, re, im);
				for (int k = 0; k != Bands; ++k)
					noise[k] += re[k] * re[k] + im[k] * im[k];
			}
		}
		var threshold = Math.Pow(10.0, ThresholdDb / 10.0) / noise_windows;
		for (int k = 0; k != Bands; ++k)
			noise[k] *= threshold;

		// A band is open (gain 1) in a window where its pooled power is clearly louder than its pooled noise, and closed (the reduction floor) otherwise
		var floor = Math.Pow(10.0, -reduction_db / 20.0);
		var pooled_noise = Pool(noise);
		var gains = new float[windows * Bands];
		var power = new double[Bands];
		for (int w = 0; w != windows; ++w)
		{
			// The channels' powers are summed, so a band opens on all channels together
			Array.Clear(power);
			for (int c = 0; c != channels; ++c)
			{
				Transform(fft, window, padded[c], w * Hop, re, im);
				for (int k = 0; k != Bands; ++k)
					power[k] += re[k] * re[k] + im[k] * im[k];
			}
			var pooled = Pool(power);
			for (int k = 0; k != Bands; ++k)
				gains[w * Bands + k] = pooled[k] > pooled_noise[k] ? 1f : (float)floor;
		}

		// Gains change smoothly over time, so a band opens before a sound starts and closes after its natural decay
		SmoothOverTime(gains, windows, reduction_db);

		// Each window is scaled band by band, transformed back and added into the output. The Hann window, applied before and after the change, sums to
		// 1.5 over four overlapping windows, which the final scale removes.
		var output = new float[buffer.Samples.Length];
		for (int c = 0; c != channels; ++c)
		{
			// Each channel is rebuilt separately with the shared gains
			var rebuilt = new double[padded_length];
			for (int w = 0; w != windows; ++w)
			{
				// Bands 'k' and 'WindowFrames - k' are mirror images in a real signal's spectrum, so both take the band's gain
				Transform(fft, window, padded[c], w * Hop, re, im);
				for (int k = 0; k != Bands; ++k)
				{
					var gain = gains[w * Bands + k];
					re[k] *= gain;
					im[k] *= gain;
					if (k != 0 && k != WindowFrames / 2)
					{
						re[WindowFrames - k] *= gain;
						im[WindowFrames - k] *= gain;
					}
				}
				fft.Inverse(re, im);
				for (int i = 0; i != WindowFrames; ++i)
					rebuilt[w * Hop + i] += re[i] * window[i];
			}
			for (int i = 0; i != frames; ++i)
				output[i * channels + c] = (float)(rebuilt[WindowFrames + i] / 1.5);
		}
		return new AudioBuffer(output, channels);
	}

	/// <summary>A periodic Hann window of <see cref="WindowFrames"/> points.</summary>
	private static double[] HannWindow()
	{
		// The periodic form overlaps to a constant sum at a quarter-window hop
		var window = new double[WindowFrames];
		for (int i = 0; i != WindowFrames; ++i)
			window[i] = 0.5 - 0.5 * Math.Cos(2 * Math.PI * i / WindowFrames);

		return window;
	}

	/// <summary>The spectrum of the window of 'samples' starting at 'start', written to 're' and 'im'.</summary>
	private static void Transform(Fft fft, double[] window, double[] samples, int start, double[] re, double[] im)
	{
		// The windowed samples are the real input
		for (int i = 0; i != WindowFrames; ++i)
		{
			re[i] = samples[start + i] * window[i];
			im[i] = 0;
		}
		fft.Forward(re, im);
	}

	/// <summary>
	/// Let each band's gain open early, over <see cref="AttackSeconds"/>, and close late, over <see cref="ReleaseSeconds"/>. 'gains' holds 'windows' rows
	/// of <see cref="Bands"/> gains, each between the floor and 1.
	/// </summary>
	private static void SmoothOverTime(float[] gains, int windows, double reduction_db)
	{
		// The gain may fall from 1 to the floor no faster than the release time going forwards, and no faster than the attack time going backwards.
		// The larger of the two results keeps both the lead-in and the tail.
		var hop_seconds = (double)Hop / AudioBuffer.SampleRate;
		var release = (float)Math.Pow(10.0, -reduction_db / 20.0 * hop_seconds / ReleaseSeconds);
		var attack = (float)Math.Pow(10.0, -reduction_db / 20.0 * hop_seconds / AttackSeconds);
		var forward = (float[])gains.Clone();
		for (int w = 1; w != windows; ++w)
		{
			// Each window holds at least the decayed gain of the window before it
			for (int k = 0; k != Bands; ++k)
				forward[w * Bands + k] = Math.Max(forward[w * Bands + k], forward[(w - 1) * Bands + k] * release);
		}
		for (int w = windows - 2; w >= 0; --w)
		{
			// Each window holds at least the decayed gain of the window after it
			for (int k = 0; k != Bands; ++k)
				gains[w * Bands + k] = Math.Max(gains[w * Bands + k], gains[(w + 1) * Bands + k] * attack);
		}
		for (int i = 0; i != gains.Length; ++i)
			gains[i] = Math.Max(gains[i], forward[i]);
	}

	/// <summary>The sum of each band's 'values' with those of the <see cref="PooledBands"/> bands on either side that exist.</summary>
	private static double[] Pool(double[] values)
	{
		// A running sum is used, so each pooled value costs the same however wide the pool is
		var pooled = new double[Bands];
		var sum = 0.0;
		for (int k = 0; k != PooledBands && k != Bands; ++k)
			sum += values[k];

		for (int k = 0; k != Bands; ++k)
		{
			// The band entering the pool on the high side is added, and the one leaving on the low side is removed
			if (k + PooledBands < Bands)
				sum += values[k + PooledBands];
			if (k - PooledBands - 1 >= 0)
				sum -= values[k - PooledBands - 1];

			pooled[k] = sum;
		}
		return pooled;
	}

	/// <summary>An in-place complex fast Fourier transform of a fixed power-of-two size.</summary>
	private sealed class Fft
	{
		private readonly int m_size;
		private readonly double[] m_cos;
		private readonly double[] m_sin;
		private readonly int[] m_reverse;

		/// <summary>Prepare transforms of 'size' points, which must be a power of two.</summary>
		public Fft(int size)
		{
			// The twiddle factors and the bit-reversed order are computed once and shared by every transform
			m_size = size;
			m_cos = new double[size / 2];
			m_sin = new double[size / 2];
			for (int i = 0; i != size / 2; ++i)
			{
				m_cos[i] = Math.Cos(2 * Math.PI * i / size);
				m_sin[i] = Math.Sin(2 * Math.PI * i / size);
			}
			var bits = System.Numerics.BitOperations.Log2((uint)size);
			m_reverse = new int[size];
			for (int i = 0; i != size; ++i)
			{
				// Reverse the low 'bits' bits of 'i'
				var r = 0;
				for (int b = 0; b != bits; ++b)
					r |= ((i >> b) & 1) << (bits - 1 - b);

				m_reverse[i] = r;
			}
		}

		/// <summary>Replace 're' and 'im' with their spectrum.</summary>
		public void Forward(double[] re, double[] im)
		{
			Run(re, im, -1);
		}

		/// <summary>Replace the spectrum 're' and 'im' with the signal it came from.</summary>
		public void Inverse(double[] re, double[] im)
		{
			// The unscaled inverse transform is scaled by the size to undo the forward transform exactly
			Run(re, im, +1);
			for (int i = 0; i != m_size; ++i)
			{
				re[i] /= m_size;
				im[i] /= m_size;
			}
		}

		/// <summary>The iterative radix-2 transform, with 'sign' -1 for forward and +1 for inverse.</summary>
		private void Run(double[] re, double[] im, int sign)
		{
			// The input is put in bit-reversed order so that each pass combines adjacent half-size transforms
			for (int i = 0; i != m_size; ++i)
			{
				// Each pair is swapped once
				var j = m_reverse[i];
				if (j > i)
				{
					(re[i], re[j]) = (re[j], re[i]);
					(im[i], im[j]) = (im[j], im[i]);
				}
			}

			// Each pass doubles the transform length, combining pairs with the twiddle factors
			for (int length = 2; length <= m_size; length *= 2)
			{
				// The twiddle table is for the full size, so a shorter transform steps through it
				var half = length / 2;
				var step = m_size / length;
				for (int start = 0; start != m_size; start += length)
				{
					// Butterflies combine element 'i' with its partner half a transform later
					for (int i = 0; i != half; ++i)
					{
						var wr = m_cos[i * step];
						var wi = sign * m_sin[i * step];
						var a = start + i;
						var b = a + half;
						var tr = re[b] * wr - im[b] * wi;
						var ti = re[b] * wi + im[b] * wr;
						re[b] = re[a] - tr;
						im[b] = im[a] - ti;
						re[a] += tr;
						im[a] += ti;
					}
				}
			}
		}
	}
}
