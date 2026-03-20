using System;
using System.Numerics;
using System.Collections.Generic;
using Rylogic.Common;
using Rylogic.Extn;
using System.Diagnostics;

namespace Rylogic.Maths
{
	/// <summary>Sliding window Discrete Fourier Transform (DFT)</summary>
	public class SlidingDFT
	{
		// Notes:
		// This code efficiently computes the discrete Fourier transforms (DFT) from a
		// continuous sequence of input values. It updates the DFT when each new time-domain
		// measurement arrives, effectively applying a sliding window over the last *N* samples.
		// This implementation applies the 'Hanning' window in order to minimise spectral leakage.
		// The update step has computational complexity O(N). If a new DFT is required every M
		// samples, and 'M < log2(N)', then this approach is more efficient than recalculating
		// the DFT from scratch each time.
		private class Buffer :List<Complex> {}

		/// <summary>Input signal ring buffer</summary>
		private Buffer m_in;

		/// <summary>The frequency domain values (*without* windowing).</summary>
		private Buffer m_out;

		/// <summary>Forward ('-tau * e^iw') and inverse ('+tau * e^iw') coefficients normalized (divided by WindowSize)</summary>
		private Buffer m_coeffs;

		/// <summary>
		/// Create a sliding DFT instance
		/// 'window_size' is the number of samples in the sliding window. 
		/// 'inverse' is true if this is actually an inverse DFT</summary>
		public SlidingDFT()
			:this(0, 0.0, false)
		{}
		public SlidingDFT(int window_size, double sampling_frequency, bool inverse = false)
		{
			m_in                 = new Buffer();
			m_out                = new Buffer();
			m_coeffs             = new Buffer();
			SamplingFrequency = 0.0;
			DampingFactor     = 1.0 - double.Epsilon;
			Count              = 0;

			if (window_size != 0)
				Reset(window_size, sampling_frequency, inverse);
		}

		/// <summary>The size of the moving window</summary>
		public int WindowSize
		{
			get { return m_in.Count; }
		}

		/// <summary>The number of time-domain samples added to 'm_in'.</summary>
		public int Count
		{
			get;
			private set;
		}

		/// <summary>A damping factor guarantee stability.</summary>
		public double DampingFactor
		{
			get;
			private set;
		}

		/// <summary>
		/// The frequency that the values provided to 'Add' are sampled at.
		/// Required to determine the output frequency range.</summary>
		public double SamplingFrequency
		{
			get;
			private set;
		}

		/// <summary>Return the frequency domain range</summary>
		public RangeF FreqRange
		{
			get { return new RangeF(0.0, SamplingFrequency * 0.5); }
		}

		/// <summary>
		/// Reset the Sliding DFT and set a window size
		/// 'window_size' is the number of samples in the sliding window. 
		/// 'inverse' is true if this is actually an inverse DFT</summary>
		public void Reset(int window_size, double sampling_frequency, bool inverse = false)
		{
			// Erase previous data
			m_in    .Clear(); m_in    .Resize(window_size);
			m_out   .Clear(); m_out   .Resize(window_size);
			m_coeffs.Clear(); m_coeffs.Resize(window_size);
			SamplingFrequency = sampling_frequency;
			Count = 0;

			// Initialize 'e-to-the-i-thetas' for theta = 0..tau in increments of 1/N
			var k = (!inverse ? +Math_.Tau : -Math_.Tau) * new Complex(0,1) / (double)window_size;
			for (int i = 0; i != window_size; ++i)
				m_coeffs[i] = Complex.Exp(k * (double)i);
		}

		/// <summary>Add the next sample. Samples should represent the input signal sampled at 'm_sampling_frequency'</summary>
		public void Add(double value)
		{
			Add(new Complex(value, 0));
		}
		public void Add(Complex value)
		{
			// The window size
			var N = WindowSize;

			// The ring buffer index
			var idx = Count % N;

			// The incoming and outgoing values
			var old = m_in[idx];
			var nue = m_in[idx] = value;

			// Update the DFT. Apply damping for stability
			var damping = Math.Pow(DampingFactor, N);
			for (var k = 0; k != N; ++k)
				m_out[k] = m_coeffs[k] * (DampingFactor*m_out[k] - damping*old + nue);

			++Count;
		}

		/// <summary>
		/// Return the spectral power for the given frequency.
		/// Note: this must be less than 'm_sampling_frequency / 2.0' due to the Nyquist limit.
		/// It should be in the range returned from 'FreqRange'.
		/// 'freq' is quantised to the nearest buffer index.</summary>
		public double Power(double freq)
		{
			var fidx = FIndexAt(freq);
			if (fidx < 0)
				return 0.0;

			// Return the spectral power for the buffer index
			var idx = (int)fidx;
			var a = ValueAt(idx+0).Magnitude;
			var b = ValueAt(idx+1).Magnitude;
			return Math_.Lerp(a, b, fidx - idx);
		}

		/// <summary>Return the frequency domain values</summary>
		public double[] Spectrum()
		{
			var buf = new double[WindowSize / 2];
			return Spectrum(buf, buf.Length);
		}

		/// <summary>Return the frequency domain values in 'buffer'</summary>
		public double[] Spectrum(double[] buffer, int length)
		{
			Debug.Assert(length >= WindowSize / 2, "Insufficient buffer space");
			var N = WindowSize;

			// 'm_out' contains the spectrum twice, with the upper portion mirrored
			// Only need to copy range [0,N/2). Apply a 'Hanning window' to minimise spectral leakage.
			var a = m_out[N-1];
			var b = m_out[0];
			var c = m_out[1];
			for (int i = 0; i != N/2; ++i, a = b, b = c, c = m_out[i+1])
				buffer[i] = (0.5*b - 0.25*(a + c)).Magnitude;

			return buffer;
		}

		/// <summary>Convert a frequency domain value to a fractional buffer index. Returns -1 for out of range frequencies.</summary>
		private double FIndexAt(double freq)
		{
			return FIndexAt(freq, SamplingFrequency, WindowSize);
		}

		/// <summary>Return the frequency domain value at buffer index 'index' with a 'Hanning window' applied.</summary>
		private Complex ValueAt(int idx)
		{
			var N = WindowSize;
			idx += N;
			return 0.5 * m_out[idx % N] - 0.25 * (m_out[(idx-1) % N] + m_out[(idx+1) % N]);
		}

		/// <summary>Convert a frequency domain value to a fractional buffer index. Returns -1 for out of range frequencies.</summary>
		private static double FIndexAt(double freq, double sampling_frequency, int buffer_size)
		{
			// Convert the frequency into a buffer index
			var fidx = freq * buffer_size / sampling_frequency;
			return fidx >= 0 && fidx < 0.5*buffer_size ? fidx : -1.0;
		}

		/// <summary>
		/// Convert a fractional buffer index to a frequency domain value.
		/// Returns -1 if 'fidx' is outside the range [0, buffer_size/2).</summary>
		private static double FreqAt(double fidx, double sampling_frequency, int buffer_size)
		{
			if (fidx < 0 || fidx >= 0.5*buffer_size) return -1.0;
			return sampling_frequency * fidx / buffer_size;
		}
	}
}


#if PR_UNITTESTS
namespace Rylogic.UnitTests
{
	using System.IO;
	using System.Text;
	using Maths;

	[TestFixture] public class TestFFT
	{
		[Test] public void SlidingDFT()
		{
			const double freq0    =   2.0; // hz
			const double freq1    =  10.0; // hz
			const double freq2    =  37.0; // hz
			const double freq3    =  60.0; // hz
			const double freq4    = 200.0; // hz
			const double SampFreq = 1000.0;

			// Create a sinusoidal signal
			var signal = new double[8192];
			for (int i = 0, iend = signal.Length; i != iend; ++i)
			{
				signal[i] =
					Math.Sin(Math_.Tau * freq0 * i / SampFreq) +
					Math.Sin(Math_.Tau * freq1 * i / SampFreq) +
					Math.Sin(Math_.Tau * freq2 * i / SampFreq) +
					Math.Sin(Math_.Tau * freq3 * i / SampFreq) +
					Math.Sin(Math_.Tau * freq4 * i / SampFreq);
			}

			// Naive DFT
#if false // disabled, cause slow
			{
				BufferC naive(signal.size());
				DFTNaive(signal.data(), naive.data(), int(signal.size()));

				{// Output the frequency response
					std::string s_out;
					for (int i = 0, iend = int(naive.size() / 2); i != iend; ++i)
					{
						auto x = dft::FreqAt(double(i), SampFreq, naive.size());
						auto y = Length(naive[i]);
						s_out.append(pr::FmtS("%f, %f\n", x, y));
					}
					pr::BufferToFile(s_out, "\\dump\\frequencies0.csv");
				}
			}
#endif
#if false
			{// Standard DFT
				var freq = DFTRadix2(signal.data(), int(signal.size()));

				{// Output the frequency response
					std::string s_out;
					for (int i = 0, iend = int(freq.size()/2); i != iend; ++i)
					{
						auto x = dft::FreqAt(double(i), SampFreq, freq.size());
						auto y = Length(freq[i]);
						s_out.append(pr::FmtS("%f, %f\n", x, y));
					}
					pr::BufferToFile(s_out, "\\dump\\frequencies1.csv");
				}
			}
#endif
			{// Sliding Window DFT

				// Add the signal to the DFT
				const int window_size = 512;
				var dft = new SlidingDFT(window_size, SampFreq);
				for (int i = 0, iend = window_size; i != iend; ++i)
					dft.Add(signal[i]);

				{// Output the frequency response
					var s_out = new StringBuilder();
					var freq_range = dft.FreqRange;
					for (double x = freq_range.Beg; x < freq_range.End; x += 0.1)
						s_out.AppendLine($"{x}, {dft.Power(x)}");

					//File.WriteAllText("\\dump\\frequencies2.csv", s_out.ToString());
				}
			}
		}
	}
}
#endif
