//*****************************************************************************
// Maths library
//  Copyright (c) Rylogic Ltd 2002
//*****************************************************************************
/* 
 * Free FFT and convolution (C++)
 * 
 * Copyright (c) 2021 Project Nayuki. (MIT License)
 * https://www.nayuki.io/page/free-small-fft-in-multiple-languages
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 * - The above copyright notice and this permission notice shall be included in
 *   all copies or substantial portions of the Software.
 * - The Software is provided "as is", without warranty of any kind, express or
 *   implied, including but not limited to the warranties of merchantability,
 *   fitness for a particular purpose and noninfringement. In no event shall the
 *   authors or copyright holders be liable for any claim, damages or other
 *   liability, whether in an action of contract, tort or otherwise, arising from,
 *   out of or in connection with the Software or the use or other dealings in the
 *   Software.
 */
#pragma once
#include <complex>
#include "pr/common/cast.h"
#include "pr/common/bit_fields.h"
#include "pr/maths/maths.h"

namespace pr::algorithm::fft
{
	// Implementation
	namespace impl
	{
		// Forwards
		template <typename Real> void DFTRadix2(Real* real, Real* imag, size_t pow2_length);
		template <typename Real> void DFTBluestein(Real* real, Real* imag, size_t length);
		template <typename Real> void Convolve(Real* xr, Real* xi, Real* yr, Real* yi, Real* outr, Real* outi, size_t pow2_length);

		 // Computes the discrete Fourier transform (DFT) of the given complex vector in-place.
		 // The vector can have any length. (real and imag have the same length).
		template <typename Real>
		void DFT(Real* real, Real* imag, size_t length)
		{
			if (length == 0)
				return;

			if (IsPowerOfTwo(length))
				DFTRadix2(real, imag, length);
			else
				DFTBluestein(real, imag, length);
		}

		// Computes the DFT of the given complex vector in place.
		// The vector length must be a power of 2. Uses the 'Cooley-Tukey' decimation-in-time radix-2 algorithm.
		template <typename Real>
		void DFTRadix2(Real* real, Real* imag, size_t pow2_length)
		{
			auto length = pow2_length;
			if (!IsPowerOfTwo(length))
				throw std::runtime_error("Length is not a power of 2");

			// Compute levels = floor(log2(n))
			int levels = 0;
			for (auto i = length; i > 1; i >>= 1)
				++levels;

			// Bit-reversed addressing permutation
			for (auto i = 0; i != s_cast<int>(length); ++i)
			{
				auto j = ReverseBits64(i, levels);
				if (j > i)
				{
					std::swap(real[i], real[j]);
					std::swap(imag[i], imag[j]);
				}
			}

			// 'Cooley-Tukey' decimation-in-time radix-2 FFT
			for (auto size = 2ULL; size <= pow2_length; size *= 2)
			{
				auto halfsize = size / 2;
				auto tablestep = length / size;
				for (auto i = 0ULL; i < length; i += size)
				{
					for (auto j = i, k = 0ULL; j < i + halfsize; ++j, k += tablestep)
					{
						auto l = j + halfsize;
						auto cos_k = std::cos(constants<double>::tau * k / length);
						auto sin_k = std::sin(constants<double>::tau * k / length);
						auto re =  real[l] * cos_k + imag[l] * sin_k;
						auto im = -real[l] * sin_k + imag[l] * cos_k;

						real[l] = real[j] - re;
						imag[l] = imag[j] - im;
						real[j] += re;
						imag[j] += im;
					}
				}

				// Prevent overflow in 'size *= 2'
				if (size == length)
					break;
			}
		}

		// Computes DFT of the given complex vector in place.
		// The vector can have any length. This requires the convolution function, which in turn requires
		// the radix-2 FFT function. Uses 'Bluestein's chirp z-transform algorithm.
		template <typename Real>
		void DFTBluestein(Real* real, Real* imag, size_t length)
		{
			// Find a power-of-2 convolution length m such that m >= length * 2 + 1
			size_t m = 1ULL;
			for (; m / 2 <= length; m *= 2)
			{
				if (m > limits<size_t>::max() / 2)
					throw std::length_error("Vector too large");
			}

			std::vector<Real> areal(m), aimag(m);
			std::vector<Real> breal(m+1), bimag(m+1);
			std::vector<Real> creal(m), cimag(m);

			// Temporary vectors and preprocessing
			for (auto i = 0; i != s_cast<int>(length); ++i)
			{
				// An accurate version of: angle = pi * i * i / length;
				auto angle = constants<double>::tau_by_2 * ((i * i) % (length * 2)) / length;
				auto cos_i = std::cos(angle);
				auto sin_i = std::sin(angle);

				areal[i] =  real[i] * cos_i + imag[i] * sin_i;
				aimag[i] = -real[i] * sin_i + imag[i] * cos_i;
				breal[i] = breal[m - i] = cos_i;
				bimag[i] = bimag[m - i] = sin_i;
			}

			// Convolution
			Convolve(&areal[0], &aimag[0], &breal[0], &bimag[0], &creal[0], &cimag[0], m);

			// Post-processing
			for (auto i = 0; i != s_cast<int>(length); ++i)
			{
				// An accurate version of: angle = pi * i * i / length;
				auto angle = constants<double>::tau_by_2 * ((i * i) % (length * 2)) / length;
				auto cos_i = std::cos(angle);
				auto sin_i = std::sin(angle);

				real[i] =  creal[i] * cos_i + cimag[i] * sin_i;
				imag[i] = -creal[i] * sin_i + cimag[i] * cos_i;
			}
		}

		// Computes the circular convolution of the given complex vectors. Each vector's length must be the same and a power of 2.
		template <typename Real>
		void Convolve(Real* xr, Real* xi, Real* yr, Real* yi, Real* outr, Real* outi, size_t length)
		{
			// DFT 'x' and 'y'
			DFT(xr, xi, length);
			DFT(yr, yi, length);

			// Compute the product in 'out'
			for (auto i = 0; i != s_cast<int>(length); ++i)
			{
				outr[i] = xr[i] * yr[i] - xi[i] * yi[i];
				outi[i] = xi[i] * yr[i] + xr[i] * yi[i];
			}

			// Inverse DFT of product
			DFT(outi, outr, length);

			// Scaling (because DFT omits it)
			for (auto i = 0; i != s_cast<int>(length); ++i)
			{
				outr[i] /= length;
				outi[i] /= length;
			}
		}

		// Naive DFT implementation, used for testing. O(N²)
		template <typename Real>
		void DFTNaive(Real const* real, Real const* imag, Real* outr, Real* outi, size_t length, bool inverse)
		{	
			auto coef = Bool2SignI(inverse) * constants<double>::tau / length;

			// For each output element
			for (auto k = 0ULL; k != length; ++k)
			{
				auto sumr = 0.0;
				auto sumi = 0.0;

				// For each input element
				for (int t = 0; t != s_cast<int>(length); ++t)
				{
					auto angle = coef * (static_cast<long long>(t) * k % length);
					sumr += real[t] * std::cos(angle) - imag[t] * std::sin(angle);
					sumi += real[t] * std::sin(angle) + imag[t] * std::cos(angle);
				}
				
				outr[k] = static_cast<Real>(sumr);
				outi[k] = static_cast<Real>(sumi);
			}
		}

		// Naive Convolve implementation, used for testing. O(N²)
		template <typename Real>
		void ConvolveNaive(Real const* xr, Real const* xi, Real const* yr, Real const* yi, Real* outr, Real* outi, size_t length)
		{
			for (auto i = 0; i != s_cast<int>(length); ++i)
			{
				for (auto j = 0ULL; j != length; ++j)
				{
					auto k = (i + j) % length;
					outr[k] += xr[i] * yr[j] - xi[i] * yi[j];
					outi[k] += xr[i] * yi[j] + xi[i] * yr[j];
				}
			}
		}
	}

	 // Computes the discrete Fourier transform (DFT) of the given complex vector.
	 // The vector can have any length. (real and imag have the same length).
	template <typename Real>
	void DiscreteFourierTransform(Real const* inputr, Real const* inputi, Real* outr, Real* outi, size_t length)
	{
		if (inputr != outr) memcpy(outr, inputr, length * sizeof(Real));
		if (inputi != outi) memcpy(outi, inputi, length * sizeof(Real));
		impl::DFT(outr, outi, length);
	}

	 // Computes the discrete Fourier transform (DFT) of the given signal.
	 // Returns the magnitudes of the transformed result. The vector can have any length.
	template <typename Real>
	std::vector<Real> DiscreteFourierTransform(Real const* inputr, size_t length)
	{
		std::vector<Real> outr(length), outi(length);
		memcpy(outr.data(), inputr, length * sizeof(Real));
		impl::DFT(outr.data(), outi.data(), length);
		
		// Convert the complex values to magnitudes
		for (auto i = 0; i != s_cast<int>(length); ++i)
			outr[i] = sqrt(norm(std::complex<double>(outr[i], outi[i])));

		return std::move(outr);
	}

	// Computes the inverse discrete Fourier transform (iDFT) of the given complex vector in-place.
	// The vector can have any length.
	template <typename Real>
	void InverseDiscreteFourierTransform(Real const* inputr, Real const* inputi, Real* outr, Real* outi, size_t length)
	{
		if (inputr != outr) memcpy(outr, inputr, length * sizeof(Real));
		if (inputi != outi) memcpy(outi, inputi, length * sizeof(Real));

		// Scale to get the true inverse
		for (auto i = 0; i != s_cast<int>(length); ++i)
		{
			outr[i] /= length;
			outi[i] /= length;
		}

		impl::DFT(outi, outr, length);
	}
	template <typename Real>
	std::vector<Real> InverseDiscreteFourierTransform(Real const* inputr, size_t length)
	{
		std::vector<Real> outr(length);
		memcpy(outr, inputr, length);
		impl::DFT(std::vector<Real>(length).data(), outr.data(), length);
		return std::move(outr);
	}

	// Convert a frequency domain value to a fractional buffer index.
	// The returned value may not be in the range [0, buffer_size/2).
	// Callers should check this before using the returned value as an index.
	template <typename Real>
	inline Real FIndexAt(Real freq, Real sampling_frequency, size_t buffer_size)
	{
		// Convert the frequency into a buffer index
		return freq * buffer_size / sampling_frequency;
	}

	// Convert a fractional buffer index to a frequency domain value.
	// The fractional buffer index 'fidx' does not need to be within the range [0, buffer_size/2).
	template <typename Real>
	inline Real FreqAt(Real fidx, Real sampling_frequency, size_t buffer_size)
	{
		return sampling_frequency * fidx / buffer_size;
	}
}

namespace pr
{
	#if 0

	// Sliding window Discrete Fourier Transform (DFT)
	template <typename Real> struct SlidingDFT
	{
	protected:

		// This code efficiently computes the discrete Fourier transforms (DFT) from a
		// continuous sequence of input values. It updates the DFT when each new time-domain
		// measurement arrives, effectively applying a sliding window over the last *N* samples.
		// This implementation applies the 'Hanning' window in order to minimise spectral leakage.
		// The update step has computational complexity O(N). If a new DFT is required every M
		// samples, and 'M < log2(N)', then this approach is more efficient than recalculating
		// the DFT from scratch each time.
		using complex = std::complex<Real>;
		using Buffer = std::vector<complex>;

		// Input signal ring buffer
		Buffer m_in;

		// The frequency domain values (*without* windowing).
		Buffer m_out;

		// Forward ('-tau * e^iw') and inverse ('+tau * e^iw') coefficients normalized (divided by WindowSize)
		Buffer m_coeffs;

		// The frequency that the values provided to 'Add' are sampled at.
		// Required to determine the output frequency range.
		Real m_sampling_frequency;

		// A damping factor guarantee stability.
		Real m_damping_factor;

		// The number of time-domain samples added to 'm_in'.
		int m_count;

	public:

		// Create a sliding DFT instance
		// 'window_size' is the number of samples in the sliding window. 
		// 'inverse' is true if this is actually an inverse DFT
		SlidingDFT()
			:SlidingDFT(0, 0.0, false)
		{}
		SlidingDFT(int window_size, Real sampling_frequency, bool inverse = false)
			:m_in()
			,m_out()
			,m_coeffs()
			,m_sampling_frequency()
			,m_damping_factor(std::nexttoward(Real(1), Real(0)))
			,m_count()
		{
			if (window_size != 0)
				Reset(window_size, sampling_frequency, inverse);
		}

		// The size of the moving window
		int WindowSize() const
		{
			return int(m_in.size());
		}

		// Return the frequency domain range
		Range<Real> FreqRange() const
		{
			return Range<Real>(Real(0), Real(m_sampling_frequency / 2));
		}

		// Reset the Sliding DFT and set a window size
		// 'window_size' is the number of samples in the sliding window. 
		// 'inverse' is true if this is actually an inverse DFT
		void Reset(int window_size, Real sampling_frequency, bool inverse = false)
		{
			// Erase previous data
			m_in    .resize(0); m_in    .resize(window_size);
			m_out   .resize(0); m_out   .resize(window_size);
			m_coeffs.resize(0); m_coeffs.resize(window_size);
			m_sampling_frequency = sampling_frequency;
			m_count = 0;

			// Initialize 'e-to-the-i-thetas' for theta = 0..tau in increments of 1/N
			auto k = (!inverse ? +maths::tau : -maths::tau) * complex(0,1) / double(window_size);
			for (int i = 0; i != window_size; ++i)
				m_coeffs[i] = std::exp(k * Real(i));
		}

		// Add the next sample. Samples should represent the input signal sampled at 'm_sampling_frequency'
		void Add(Real value)
		{
			Add(complex(value, 0));
		}
		void Add(complex value)
		{
			// The window size
			auto N = WindowSize();

			// The ring buffer index
			auto idx = m_count % N;

			// The incoming and outgoing values
			auto old = m_in[idx];
			auto nue = m_in[idx] = value;

			// Update the DFT. Apply damping for stability
			auto damping = pow(m_damping_factor, Real(N));
			for (auto k = 0; k != N; ++k)
				m_out[k] = m_coeffs[k] * (m_damping_factor*m_out[k] - damping*old + nue);

			++m_count;
		}

		// Return the spectral power for the given frequency.
		// Note: this must be less than 'm_sampling_frequency / 2.0' due to the Nyquist limit.
		// It should be in the range returned from 'FreqRange'.
		// 'freq' is quantised to the nearest buffer index.
		Real Power(Real freq) const
		{
			auto fidx = FIndexAt(freq);
			if (fidx < 0)
				return Real(0);

			// Return the spectral power for the buffer index
			auto idx = int(fidx);
			auto a = Length(ValueAt(idx+0));
			auto b = Length(ValueAt(idx+1));
			return Real(Lerp(a, b, fidx - idx));
		}

		// Return the frequency domain values
		std::vector<Real> Spectrum() const
		{
			std::vector<Real> buf(WindowSize() / 2);
			Spectrum(buf.data(), int(buf.size()));
			return std::move(buf);
		}

		// Return the frequency domain values in 'buffer'
		void Spectrum(Real* buffer, int length)
		{
			pr_assert("Insufficient buffer space" && length >= WindowSize() / 2);
			auto N = WindowSize();

			// 'm_out' contains the spectrum twice, with the upper portion mirrored
			// Only need to copy range [0,N/2). Apply a 'Hanning window' to minimise spectral leakage.
			auto a = m_out[N-1];
			auto b = m_out[0];
			auto c = m_out[1];
			for (int i = 0; i != N/2; ++i, a = b, b = c, c = m_out[i+1])
				buffer[i] = Real(Length(0.5*b - 0.25*(a + c)));
		}

		// Convert a frequency domain value to a fractional buffer index. Returns -1 for out of range frequencies.
		Real FIndexAt(Real freq) const
		{
			return dft::FIndexAt(freq, m_sampling_frequency, WindowSize());
		}

		// Return the frequency domain value at buffer index 'index' with a 'Hanning window' applied.
		complex ValueAt(int idx) const
		{
			auto N = WindowSize();
			idx += N;
			return 0.5 * m_out[idx % N] - 0.25 * (m_out[(idx-1) % N] + m_out[(idx+1) % N]);
		}
	};

	#endif
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
namespace pr::algorithm::fft::tests
{
	PRUnitTestClass(FourierTransformTests)
	{
		std::default_random_engine rng;
		TestClass_FourierTransformTests()
			:rng(1)
		{}

		// Generate a vector of random real values
		std::vector<double> RandomReals(int n)
		{
			std::vector<double> result;
			std::uniform_real_distribution<double> dist(-1.0, 1.0);
			for (auto i = 0; i != n; ++i) result.push_back(dist(rng));
			return std::move(result);
		}

		// Returns the Log10 RMS error between 'x' and 'y'
		double Log10RMSError(double const* xr, double const* xi, double const* yr, double const* yi, int length)
		{
			auto err = std::pow(10, -99 * 2);
			for (auto i = 0; i != length; ++i)
			{
				auto real = xr[i] - yr[i];
				auto imag = xi[i] - yi[i];
				err += real * real + imag * imag;
			}
			length += (length == 0);
			return std::log10(std::sqrt(err / length));
		}

		// Test run a FFT
		void TestFFT(int n, double& err0, double& err1)
		{
			auto inputr = RandomReals(n);
			auto inputi = RandomReals(n);

			std::vector<double> expectr(n), expecti(n);
			impl::DFTNaive(inputr.data(), inputi.data(), expectr.data(), expecti.data(), n, false);

			std::vector<double> actualr(n), actuali(n);
			DiscreteFourierTransform(inputr.data(), inputi.data(), actualr.data(), actuali.data(), n);
			err0 = Log10RMSError(expectr.data(), expecti.data(), actualr.data(), actuali.data(), n);

			InverseDiscreteFourierTransform(actualr.data(), actuali.data(), actualr.data(), actuali.data(), n);
			err1 = Log10RMSError(inputr.data(), inputi.data(), actualr.data(), actuali.data(), n);
		}

		// Test run a convolution
		void TestConvolution(int n, double& err)
		{
			auto input0r = RandomReals(n);
			auto input0i = RandomReals(n);
			auto input1r = RandomReals(n);
			auto input1i = RandomReals(n);

			std::vector<double> expectr(n), expecti(n);
			impl::ConvolveNaive(input0r.data(), input0i.data(), input1r.data(), input1i.data(), expectr.data(), expecti.data(), n);

			std::vector<double> actualr(n), actuali(n);
			impl::Convolve(input0r.data(), input0i.data(), input1r.data(), input1i.data(), actualr.data(), actuali.data(), n);
			err = Log10RMSError(expectr.data(), expecti.data(), actualr.data(), actuali.data(), n);
		}

		PRUnitTestMethod(DFTAndiDFTTests)
		{
			double max_err0 = -99.0, max_err1 = -99.0; //db

			// Test power-of-2 size FFTs
			for (int i = 0; i != 13; ++i)
			{
				double err0, err1;
				TestFFT(1 << i, err0, err1);
				max_err0 = std::max(max_err0, err0);
				max_err1 = std::max(max_err1, err1);
			}

			// Test small size FFTs
			for (auto i = 0; i != 30; ++i)
			{
				double err0, err1;
				TestFFT(i, err0, err1);
				max_err0 = std::max(max_err0, err0);
				max_err1 = std::max(max_err1, err1);
			}

			// Test diverse size FFTs
			for (auto i = 0, prev = 0; i != 100; ++i)
			{
				auto n = static_cast<int>(std::lround(std::pow(1500.0, i / 100.0)));
				if (n > prev)
				{
					double err0, err1;
					TestFFT(n, err0, err1);
					max_err0 = std::max(max_err0, err0);
					max_err1 = std::max(max_err1, err1);
					prev = n;
				}
			}

			PR_EXPECT(max_err0 < -10);
			PR_EXPECT(max_err1 < -10);
		}
		PRUnitTestMethod(ConvolutionTests)
		{
			double max_err = -99.0; // db

			// Test power-of-2 size convolutions
			for (auto i = 0; i != 13; ++i)
			{
				double err;
				TestConvolution(1ULL << i, err);
				max_err = std::max(max_err, err);
			}

			// Test diverse size convolutions
			for (auto i = 0, prev = 0; i != 100; ++i)
			{
				auto n = static_cast<int>(std::lround(std::pow(1500.0, i / 100.0)));
				if (n > prev)
				{
					double err;
					TestConvolution(n, err);
					max_err = std::max(max_err, err);
				}
			}

			PR_EXPECT(max_err < -10);
		}
		PRUnitTestMethod(OtherTests)
		{
			constexpr double freq0 = 2.0; // hz
			constexpr double freq1 = 10.0; // hz
			constexpr double freq2 = 37.0; // hz
			constexpr double freq3 = 60.0; // hz
			constexpr double freq4 = 200.0; // hz
			constexpr double SampFreq = 1000.0;

			// Create a sinusoidal signal
			std::vector<double> signal(8192);
			for (auto i = 0, iend = s_cast<int>(signal.size()); i != iend; ++i)
			{
				signal[i] =
					std::sin(constants<double>::tau * freq0 * i / SampFreq) +
					std::sin(constants<double>::tau * freq1 * i / SampFreq) +
					std::sin(constants<double>::tau * freq2 * i / SampFreq) +
					std::sin(constants<double>::tau * freq3 * i / SampFreq) +
					std::sin(constants<double>::tau * freq4 * i / SampFreq);
			}

			{// Standard DFT
				auto frequencies = DiscreteFourierTransform(signal.data(), signal.size());

				// Output the frequencies
				#if 0
				{
					std::ofstream out("\\dump\\frequencies1.csv");
					for (auto i = 0, length = s_cast<int>(frequencies.size()); i != length / 2; ++i)
					{
						auto x = FreqAt(1.0 * i, SampFreq, length);
						auto y = frequencies[i];
						out << x << ", " << y << "\n";
					}
				}
				#else
				(void)frequencies;
				#endif
			}

			#if 0 // disabled, cause slow
			{// Naive DFT
				std::vector<double> imag(length);
				std::vector<double> outr(length);
				std::vector<double> outi(length);
				impl::DFTNaive(signal.data(), imag.data(), outr.data(), outi.data(), length, false);

				{// Output the frequency response
					std::ofstream out("\\dump\\frequencies0.csv");
					for (auto i = 0; i != s_cast<int>(length) / 2; ++i)
					{
						auto x = FreqAt(static_cast<double>(i), SampFreq, length);
						auto y = Sqrt(norm(std::complex<double>(outr[i], outi[i]));
						out << x << ", " << y << "\n";
					}
				}
			}
			#endif
			#if 0
			{// Sliding Window DFT
				// Add the signal to the DFT
				int const window_size = 512;
				SlidingDFT<double> dft(window_size, SampFreq);
				for (int i = 0, iend = window_size; i != iend; ++i)
					dft.Add(signal[i]);

				{// Output the frequency response
					std::ofstream out("\\dump\\frequencies2.csv");
					auto freq_range = dft.FreqRange();
					for (double x = freq_range.m_beg; x < freq_range.m_end; x += 0.1)
						out << x << ", " << dft.Power(x) << "\n";
				}
			}
			#endif
		}
	};
}
#endif
