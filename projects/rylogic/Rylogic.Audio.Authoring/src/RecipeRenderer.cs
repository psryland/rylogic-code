using System;
using System.Collections.Generic;
using System.IO;

namespace Rylogic.Audio.Authoring;

/// <summary>
/// Applies a recipe to decoded original audio. Rendering is a pure function of the original samples and the enabled steps, so the same inputs always
/// give identical output on any run, whether for preview, live update or cooking.
/// </summary>
public static class RecipeRenderer
{
	/// <summary>Identifies the renderer's behaviour. Changing any step's processing must change this, so that every cached render is rebuilt.</summary>
	public const string PreparationVersion = "rylogic-audio-prep-1";

	/// <summary>The crossfade used to join the two sides of a cut, in seconds.</summary>
	public const double CutCrossfadeSeconds = 0.010;

	/// <summary>Render 'recipe' applied to 'original'. 'original' is not modified. Throws <see cref="InvalidDataException"/> naming the bad step.</summary>
	public static AudioBuffer Render(AudioBuffer original, IReadOnlyList<RecipeStep> recipe)
	{
		// Each enabled step transforms the result of the previous one; bypassed steps are skipped entirely
		var buffer = original.Clone();
		for (int i = 0; i != recipe.Count; ++i)
		{
			var step = recipe[i];
			if (step.Bypass)
				continue;

			try
			{
				buffer = Apply(buffer, step);
			}
			catch (InvalidDataException ex)
			{
				throw new InvalidDataException($"Step {i + 1} ({step.Step}): {ex.Message}", ex);
			}
		}
		return buffer;
	}

	/// <summary>Apply one step.</summary>
	private static AudioBuffer Apply(AudioBuffer buffer, RecipeStep step)
	{
		switch (step.Step)
		{
			case ERecipeStep.Trim:
			{
				// Keep the region; an unset end means the end of the sound
				var start = buffer.FrameAt(NonNegative(step.Start, "start"));
				var end = step.End is double end_s ? buffer.FrameAt(end_s) : buffer.Frames;
				if (end <= start)
					throw new InvalidDataException("The trim keeps nothing; 'end' must be after 'start'.");

				return Slice(buffer, start, end);
			}
			case ERecipeStep.Cut:
			{
				var start = buffer.FrameAt(NonNegative(step.Start, "start"));
				var end = buffer.FrameAt(Required(step.End, "end"));
				if (end <= start)
					throw new InvalidDataException("'end' must be after 'start'.");

				return Cut(buffer, start, end);
			}
			case ERecipeStep.FadeIn:
			{
				Fade(buffer, Positive(step.Duration, "duration"), step.Curve ?? EFadeCurve.Sine, fade_in: true);
				return buffer;
			}
			case ERecipeStep.FadeOut:
			{
				Fade(buffer, Positive(step.Duration, "duration"), step.Curve ?? EFadeCurve.Sine, fade_in: false);
				return buffer;
			}
			case ERecipeStep.Gain:
			{
				Scale(buffer, DbToLinear(Required(step.Db, "db")));
				return buffer;
			}
			case ERecipeStep.Loudness:
			{
				// The gain is measured from the audio reaching this step, then applied as one static gain
				var target = Required(step.Lufs, "lufs");
				var measured = LoudnessMeter.Integrated(buffer);
				if (double.IsNegativeInfinity(measured))
					throw new InvalidDataException("The audio is silent, so its loudness cannot be matched.");

				Scale(buffer, DbToLinear(target - measured));
				return buffer;
			}
			case ERecipeStep.HighPass:
			case ERecipeStep.LowPass:
			{
				// A slope of 12n dB per octave is an order-2n Butterworth filter, made from n second-order sections
				var frequency = Frequency(step.Frequency);
				var slope = step.Slope ?? 12;
				if (slope is not (12 or 24 or 36 or 48))
					throw new InvalidDataException("'slope' must be 12, 24, 36 or 48 dB per octave.");

				var sections = slope / 12;
				for (int k = 1; k <= sections; ++k)
				{
					var q = 1.0 / (2.0 * Math.Cos(Math.PI * (2 * k - 1) / (4.0 * sections)));
					var biquad = step.Step == ERecipeStep.HighPass ? Biquad.HighPass(frequency, q) : Biquad.LowPass(frequency, q);
					biquad.Process(buffer);
				}
				return buffer;
			}
			case ERecipeStep.Eq:
			{
				var q = Positive(step.Q, "q");
				Biquad.Peaking(Frequency(step.Frequency), Required(step.Db, "db"), q).Process(buffer);
				return buffer;
			}
			case ERecipeStep.Loop:
			{
				var length = buffer.FrameAt(Positive(step.Duration, "duration"));
				if (length == 0 || length > buffer.Frames / 2)
					throw new InvalidDataException("The loop crossfade must be at least one sample and at most half the sound's length.");

				return Loop(buffer, length);
			}
			default:
			{
				throw new InvalidDataException($"Unknown step '{step.Step}'.");
			}
		}
	}

	/// <summary>Frames 'start' to 'end'.</summary>
	private static AudioBuffer Slice(AudioBuffer buffer, int start, int end)
	{
		var channels = buffer.Channels;
		return new AudioBuffer(buffer.Samples[(start * channels)..(end * channels)], channels);
	}

	/// <summary>Remove frames 'start' to 'end', overlapping the two sides with an equal-power crossfade so the join has no click.</summary>
	private static AudioBuffer Cut(AudioBuffer buffer, int start, int end)
	{
		// The crossfade cannot be longer than the audio on either side of the cut
		var channels = buffer.Channels;
		var frames = buffer.Frames;
		var xfade = Math.Min((int)Math.Round(CutCrossfadeSeconds * AudioBuffer.SampleRate), Math.Min(start, frames - end));
		var output = new float[(start + (frames - end) - xfade) * channels];

		// The head runs up to the crossfade, the overlap blends the head's end into the tail's start, then the tail continues
		var src = buffer.Samples;
		Array.Copy(src, 0, output, 0, (start - xfade) * channels);
		for (int i = 0; i != xfade; ++i)
		{
			var t = (i + 0.5) / xfade;
			var fade_out = Math.Cos(t * Math.PI / 2);
			var fade_in = Math.Sin(t * Math.PI / 2);
			for (int c = 0; c != channels; ++c)
			{
				var head = src[(start - xfade + i) * channels + c];
				var tail = src[(end + i) * channels + c];
				output[(start - xfade + i) * channels + c] = (float)(head * fade_out + tail * fade_in);
			}
		}
		Array.Copy(src, (end + xfade) * channels, output, start * channels, (frames - end - xfade) * channels);
		return new AudioBuffer(output, channels);
	}

	/// <summary>Fade the start in from silence, or the end out to silence, over 'seconds'.</summary>
	private static void Fade(AudioBuffer buffer, double seconds, EFadeCurve curve, bool fade_in)
	{
		// 't' runs from 0 at the silent end to 1 where the fade meets full level
		var length = Math.Min(buffer.FrameAt(seconds), buffer.Frames);
		var channels = buffer.Channels;
		for (int i = 0; i != length; ++i)
		{
			var t = (double)i / length;
			var gain = curve switch
			{
				EFadeCurve.Linear => t,
				EFadeCurve.Sine => Math.Sin(t * Math.PI / 2),
				_ => throw new InvalidDataException($"Unknown fade curve '{curve}'."),
			};
			var frame = fade_in ? i : buffer.Frames - 1 - i;
			for (int c = 0; c != channels; ++c)
				buffer.Samples[frame * channels + c] = (float)(buffer.Samples[frame * channels + c] * gain);
		}
	}

	/// <summary>
	/// Crossfade the last 'length' frames into the first 'length' frames and drop them. The result's last frame is followed, when looped, by what
	/// originally followed it, so the seam is continuous.
	/// </summary>
	private static AudioBuffer Loop(AudioBuffer buffer, int length)
	{
		// At the start the tail dominates (continuing from the end of the result); by the end of the overlap only the head remains
		var channels = buffer.Channels;
		var frames = buffer.Frames - length;
		var src = buffer.Samples;
		var output = src[..(frames * channels)];
		for (int i = 0; i != length; ++i)
		{
			var t = (double)i / length;
			var head_gain = Math.Sin(t * Math.PI / 2);
			var tail_gain = Math.Cos(t * Math.PI / 2);
			for (int c = 0; c != channels; ++c)
				output[i * channels + c] = (float)(src[i * channels + c] * head_gain + src[(frames + i) * channels + c] * tail_gain);
		}
		return new AudioBuffer(output, channels);
	}

	/// <summary>Multiply every sample by 'gain'.</summary>
	private static void Scale(AudioBuffer buffer, double gain)
	{
		var samples = buffer.Samples;
		for (int i = 0; i != samples.Length; ++i)
			samples[i] = (float)(samples[i] * gain);
	}

	/// <summary>Convert decibels to a linear amplitude factor.</summary>
	public static double DbToLinear(double db)
	{
		return Math.Pow(10.0, db / 20.0);
	}

	/// <summary>A required parameter.</summary>
	private static double Required(double? value, string name)
	{
		return value is double v && double.IsFinite(v) ? v : throw new InvalidDataException($"'{name}' must be set to a finite number.");
	}

	/// <summary>A required, non-negative parameter.</summary>
	private static double NonNegative(double? value, string name)
	{
		return Required(value, name) is var v && v >= 0 ? v : throw new InvalidDataException($"'{name}' must not be negative.");
	}

	/// <summary>A required, positive parameter.</summary>
	private static double Positive(double? value, string name)
	{
		return Required(value, name) is var v && v > 0 ? v : throw new InvalidDataException($"'{name}' must be positive.");
	}

	/// <summary>A filter frequency, which must be between 0 and the Nyquist frequency.</summary>
	private static double Frequency(double? value)
	{
		return Positive(value, "frequency") is var f && f < AudioBuffer.SampleRate / 2.0 ? f : throw new InvalidDataException($"'frequency' must be below {AudioBuffer.SampleRate / 2} Hz.");
	}
}

/// <summary>A second-order IIR filter section, processed independently on each channel from silence.</summary>
public readonly record struct Biquad(double B0, double B1, double B2, double A1, double A2)
{
	/// <summary>Make a section from coefficients with 'a0' not yet divided out.</summary>
	private static Biquad Normalised(double b0, double b1, double b2, double a0, double a1, double a2)
	{
		return new Biquad(b0 / a0, b1 / a0, b2 / a0, a1 / a0, a2 / a0);
	}

	/// <summary>A low-pass section with corner 'frequency' Hz and quality 'q'.</summary>
	public static Biquad LowPass(double frequency, double q)
	{
		var (cos, alpha) = Prewarp(frequency, q);
		return Normalised((1 - cos) / 2, 1 - cos, (1 - cos) / 2, 1 + alpha, -2 * cos, 1 - alpha);
	}

	/// <summary>A high-pass section with corner 'frequency' Hz and quality 'q'.</summary>
	public static Biquad HighPass(double frequency, double q)
	{
		var (cos, alpha) = Prewarp(frequency, q);
		return Normalised((1 + cos) / 2, -(1 + cos), (1 + cos) / 2, 1 + alpha, -2 * cos, 1 - alpha);
	}

	/// <summary>A peaking section that changes the level by 'db' around 'frequency' Hz with bandwidth 'q'.</summary>
	public static Biquad Peaking(double frequency, double db, double q)
	{
		var (cos, alpha) = Prewarp(frequency, q);
		var a = Math.Pow(10.0, db / 40.0);
		return Normalised(1 + alpha * a, -2 * cos, 1 - alpha * a, 1 + alpha / a, -2 * cos, 1 - alpha / a);
	}

	/// <summary>The shared terms of the standard audio-EQ section formulas.</summary>
	private static (double cos, double alpha) Prewarp(double frequency, double q)
	{
		var w0 = 2 * Math.PI * frequency / AudioBuffer.SampleRate;
		return (Math.Cos(w0), Math.Sin(w0) / (2 * q));
	}

	/// <summary>Filter every channel of 'buffer' in place.</summary>
	public void Process(AudioBuffer buffer)
	{
		// State is kept in double precision so long filters do not drift
		var channels = buffer.Channels;
		var samples = buffer.Samples;
		for (int c = 0; c != channels; ++c)
		{
			double x1 = 0, x2 = 0, y1 = 0, y2 = 0;
			for (int i = c; i < samples.Length; i += channels)
			{
				double x0 = samples[i];
				var y0 = B0 * x0 + B1 * x1 + B2 * x2 - A1 * y1 - A2 * y2;
				x2 = x1; x1 = x0;
				y2 = y1; y1 = y0;
				samples[i] = (float)y0;
			}
		}
	}
}

/// <summary>Measures loudness as specified by ITU-R BS.1770: K-weighted, gated, integrated over the whole sound, in LUFS.</summary>
public static class LoudnessMeter
{
	/// <summary>The integrated loudness of 'buffer' in LUFS, or negative infinity for silence.</summary>
	public static double Integrated(AudioBuffer buffer)
	{
		// K-weighting is a high-frequency shelf followed by a low-cut, with the standard 48 kHz coefficients
		var weighted = buffer.Clone();
		new Biquad(1.53512485958697, -2.69169618940638, 1.19839281085285, -1.69065929318241, 0.73248077421585).Process(weighted);
		new Biquad(1.0, -2.0, 1.0, -1.99004745483398, 0.99007225036621).Process(weighted);

		// Mean-square power of overlapping 400 ms blocks every 100 ms, summed over channels; a sound shorter than a block is one block
		var block = Math.Min(AudioBuffer.SampleRate * 4 / 10, weighted.Frames);
		var hop = AudioBuffer.SampleRate / 10;
		var powers = new List<double>();
		for (var start = 0; block > 0 && start + block <= weighted.Frames; start += hop)
		{
			var sum = 0.0;
			for (int i = start * weighted.Channels, end = (start + block) * weighted.Channels; i != end; ++i)
				sum += (double)weighted.Samples[i] * weighted.Samples[i];

			powers.Add(sum / block);
		}

		// The absolute gate drops silence, then the relative gate drops blocks 10 LU quieter than the remaining average
		var absolute = powers.FindAll(p => Lufs(p) > -70.0);
		if (absolute.Count == 0)
			return double.NegativeInfinity;

		var relative_threshold = Lufs(Average(absolute)) - 10.0;
		var gated = absolute.FindAll(p => Lufs(p) > relative_threshold);
		return Lufs(Average(gated));
	}

	/// <summary>Convert a block power to LUFS.</summary>
	private static double Lufs(double power)
	{
		return -0.691 + 10.0 * Math.Log10(power);
	}

	/// <summary>The mean of 'values'.</summary>
	private static double Average(List<double> values)
	{
		var sum = 0.0;
		foreach (var value in values)
			sum += value;

		return sum / values.Count;
	}
}
