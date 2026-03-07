//*****************************************************************************
// Maths library - Stat Tests
//  Copyright (c) Rylogic Ltd 2002
//*****************************************************************************
#if PR_UNITTESTS
#pragma once
#include "pr/common/unittests.h"
#include "pr/math/math.h"
#include <cmath>

namespace pr::math::tests
{
	PRUnitTestClass(StatTests)
	{
		// Brute force mean for verification
		template <typename T>
		static T BruteMean(T const* data, int count)
		{
			auto sum = T(0);
			for (int i = 0; i != count; ++i)
				sum += data[i];
			return sum / count;
		}

		// Brute force population variance for verification
		template <typename T>
		static T BrutePopVar(T const* data, int count)
		{
			auto mean = BruteMean(data, count);
			auto var = T(0);
			for (int i = 0; i != count; ++i)
			{
				auto diff = data[i] - mean;
				var += diff * diff;
			}
			return var / count;
		}

		// Brute force sample variance for verification
		template <typename T>
		static T BruteSamVar(T const* data, int count)
		{
			auto mean = BruteMean(data, count);
			auto var = T(0);
			for (int i = 0; i != count; ++i)
			{
				auto diff = data[i] - mean;
				var += diff * diff;
			}
			return var / (count - 1);
		}

		PRUnitTestMethod(StatBasic, float, double)
		{
			// Known data set: {1, 2, 3, 4, 5}
			T data[] = {T(1), T(2), T(3), T(4), T(5)};
			auto count = 5;

			auto s = Stat<T>{};
			for (int i = 0; i != count; ++i)
				s.Add(data[i]);

			PR_EXPECT(s.Count() == count);
			PR_EXPECT(s.Min() == T(1));
			PR_EXPECT(s.Max() == T(5));
			PR_EXPECT(FEql(s.Mean(), T(3)));
			PR_EXPECT(FEql(s.Sum(), T(15)));

			// Population variance: E[(X - μ)²] = ((1-3)²+(2-3)²+(3-3)²+(4-3)²+(5-3)²)/5 = 10/5 = 2
			PR_EXPECT(FEqlAbsolute(s.PopStdVar(), T(2), T(1e-6)));

			// Sample variance: 10/4 = 2.5
			PR_EXPECT(FEqlAbsolute(s.m_variance.SamStdVar(count), T(2.5), T(1e-6)));

			// Standard deviations
			PR_EXPECT(FEqlAbsolute(s.PopStdDev(), static_cast<T>(std::sqrt(T(2))), T(1e-5)));
			PR_EXPECT(FEqlAbsolute(s.SamStdDev(), static_cast<T>(std::sqrt(T(2.5))), T(1e-5)));
		}
		PRUnitTestMethod(StatSingleValue, float, double)
		{
			auto s = Stat<T>{};
			s.Add(T(42));

			PR_EXPECT(s.Count() == 1);
			PR_EXPECT(s.Min() == T(42));
			PR_EXPECT(s.Max() == T(42));
			PR_EXPECT(FEql(s.Mean(), T(42)));
			PR_EXPECT(FEql(s.Sum(), T(42)));

			// Population variance with 1 element = 0
			PR_EXPECT(FEql(s.PopStdVar(), T(0)));
			PR_EXPECT(FEql(s.PopStdDev(), T(0)));
		}
		PRUnitTestMethod(StatReset, float, double)
		{
			auto s = Stat<T>{};
			s.Add(T(10));
			s.Add(T(20));
			PR_EXPECT(s.Count() == 2);

			s.Reset();
			PR_EXPECT(s.Count() == 0);

			// After reset, add new data
			s.Add(T(100));
			PR_EXPECT(s.Count() == 1);
			PR_EXPECT(FEql(s.Mean(), T(100)));
			PR_EXPECT(s.Min() == T(100));
			PR_EXPECT(s.Max() == T(100));
		}
		PRUnitTestMethod(StatConstantValues, float, double)
		{
			// All same values should give zero variance
			auto s = Stat<T>{};
			for (int i = 0; i != 100; ++i)
				s.Add(T(7));

			PR_EXPECT(s.Count() == 100);
			PR_EXPECT(FEql(s.Mean(), T(7)));
			PR_EXPECT(s.Min() == T(7));
			PR_EXPECT(s.Max() == T(7));
			PR_EXPECT(FEqlAbsolute(s.PopStdVar(), T(0), T(1e-10)));
			PR_EXPECT(FEqlAbsolute(s.PopStdDev(), T(0), T(1e-10)));
		}
		PRUnitTestMethod(StatBruteForce, float, double)
		{
			// Verify running stats match brute force for a non-trivial data set
			T data[] = {T(2.5), T(-1.3), T(7.8), T(0.1), T(4.4), T(-3.2), T(9.9), T(1.1)};
			auto count = 8;

			auto s = Stat<T>{};
			for (int i = 0; i != count; ++i)
				s.Add(data[i]);

			auto expected_mean = BruteMean(data, count);
			auto expected_pop_var = BrutePopVar(data, count);
			auto expected_sam_var = BruteSamVar(data, count);

			PR_EXPECT(FEqlAbsolute(s.Mean(), expected_mean, T(1e-5)));
			PR_EXPECT(FEqlAbsolute(s.PopStdVar(), expected_pop_var, T(1e-4)));
			PR_EXPECT(FEqlAbsolute(s.m_variance.SamStdVar(count), expected_sam_var, T(1e-4)));
			PR_EXPECT(s.Min() == T(-3.2));
			PR_EXPECT(s.Max() == T(9.9));
		}
		PRUnitTestMethod(AvrBasic, float, double)
		{
			T data[] = {T(1), T(2), T(3), T(4), T(5)};
			auto count = 5;

			auto avr = Avr<T>{};
			for (int i = 0; i != count; ++i)
				avr.Add(data[i]);

			PR_EXPECT(avr.Count() == count);
			PR_EXPECT(FEql(avr.Mean(), T(3)));
			PR_EXPECT(FEql(avr.Sum(), T(15)));

			avr.Reset();
			PR_EXPECT(avr.Count() == 0);

			avr.Add(T(42));
			PR_EXPECT(avr.Count() == 1);
			PR_EXPECT(FEql(avr.Mean(), T(42)));
		}
		PRUnitTestMethod(AvrVarBasic, float, double)
		{
			T data[] = {T(1), T(2), T(3), T(4), T(5)};
			auto count = 5;

			auto av = AvrVar<T>{};
			for (int i = 0; i != count; ++i)
				av.Add(data[i]);

			PR_EXPECT(av.Count() == count);
			PR_EXPECT(FEql(av.Mean(), T(3)));
			PR_EXPECT(FEql(av.Sum(), T(15)));

			// Pop variance = 2.0, Sample variance = 2.5
			PR_EXPECT(FEqlAbsolute(av.PopStdVar(), T(2), T(1e-6)));
			PR_EXPECT(FEqlAbsolute(av.SamStdVar(), T(2.5), T(1e-6)));
			PR_EXPECT(FEqlAbsolute(av.PopStdDev(), static_cast<T>(std::sqrt(T(2))), T(1e-5)));
			PR_EXPECT(FEqlAbsolute(av.SamStdDev(), static_cast<T>(std::sqrt(T(2.5))), T(1e-5)));
		}
		PRUnitTestMethod(AvrVarBruteForce, float, double)
		{
			// Compare running AvrVar against brute force
			T data[] = {T(2.5), T(-1.3), T(7.8), T(0.1), T(4.4), T(-3.2), T(9.9), T(1.1)};
			auto count = 8;

			auto av = AvrVar<T>{};
			for (int i = 0; i != count; ++i)
				av.Add(data[i]);

			auto expected_mean = BruteMean(data, count);
			auto expected_pop_var = BrutePopVar(data, count);
			auto expected_sam_var = BruteSamVar(data, count);

			PR_EXPECT(FEqlAbsolute(av.Mean(), expected_mean, T(1e-5)));
			PR_EXPECT(FEqlAbsolute(av.PopStdVar(), expected_pop_var, T(1e-4)));
			PR_EXPECT(FEqlAbsolute(av.SamStdVar(), expected_sam_var, T(1e-4)));
		}
		PRUnitTestMethod(AvrVarReset, float, double)
		{
			auto av = AvrVar<T>{};
			av.Add(T(10));
			av.Add(T(20));

			av.Reset();
			PR_EXPECT(av.Count() == 0);

			av.Add(T(5));
			PR_EXPECT(av.Count() == 1);
			PR_EXPECT(FEql(av.Mean(), T(5)));
			PR_EXPECT(FEql(av.PopStdVar(), T(0)));
		}
		PRUnitTestMethod(ExpMovingAvrBasic, float, double)
		{
			// Before window is full, should behave like standard running average
			auto ema = ExpMovingAvr<T>(5);
			PR_EXPECT(ema.WindowSize() == 5);
			PR_EXPECT(ema.Count() == 0);

			ema.Add(T(1));
			PR_EXPECT(ema.Count() == 1);
			PR_EXPECT(FEql(ema.Mean(), T(1)));

			ema.Add(T(3));
			PR_EXPECT(ema.Count() == 2);
			PR_EXPECT(FEql(ema.Mean(), T(2)));

			ema.Add(T(5));
			PR_EXPECT(ema.Count() == 3);
			PR_EXPECT(FEql(ema.Mean(), T(3)));
		}
		PRUnitTestMethod(ExpMovingAvrSteadyState, float, double)
		{
			// Once past window size, EMA should weight recent values more.
			// For constant input, mean should converge to that constant.
			auto ema = ExpMovingAvr<T>(3);
			for (int i = 0; i != 100; ++i)
				ema.Add(T(10));

			PR_EXPECT(FEqlAbsolute(ema.Mean(), T(10), T(1e-6)));
			PR_EXPECT(FEqlAbsolute(ema.PopStdVar(), T(0), T(1e-6)));
		}
		PRUnitTestMethod(ExpMovingAvrReset, float, double)
		{
			auto ema = ExpMovingAvr<T>(5);
			for (int i = 0; i != 10; ++i)
				ema.Add(T(i));

			ema.Reset(3);
			PR_EXPECT(ema.Count() == 0);
			PR_EXPECT(ema.WindowSize() == 3);

			ema.Add(T(42));
			PR_EXPECT(ema.Count() == 1);
			PR_EXPECT(FEql(ema.Mean(), T(42)));
		}
		PRUnitTestMethod(ExpMovingAvrResponsiveness, float, double)
		{
			// After a step change, the EMA should track toward the new value.
			// Smaller window = faster tracking.
			auto ema_fast = ExpMovingAvr<T>(3);
			auto ema_slow = ExpMovingAvr<T>(20);

			// Seed both with value 0
			for (int i = 0; i != 50; ++i)
			{
				ema_fast.Add(T(0));
				ema_slow.Add(T(0));
			}

			// Step change to 100
			for (int i = 0; i != 10; ++i)
			{
				ema_fast.Add(T(100));
				ema_slow.Add(T(100));
			}

			// The fast EMA should be closer to 100 than the slow one
			PR_EXPECT(ema_fast.Mean() > ema_slow.Mean());
			PR_EXPECT(ema_fast.Mean() > T(50)); // fast should have tracked most of the way
		}
		PRUnitTestMethod(MovingAvrBasic, float, double)
		{
			// Before window is full, should behave like standard running average
			auto ma = MovingAvr<16, T>(4);
			PR_EXPECT(ma.Count() == 0);

			ma.Add(T(2));
			PR_EXPECT(ma.Count() == 1);
			PR_EXPECT(FEql(ma.Mean(), T(2)));

			ma.Add(T(4));
			PR_EXPECT(ma.Count() == 2);
			PR_EXPECT(FEql(ma.Mean(), T(3)));

			ma.Add(T(6));
			PR_EXPECT(ma.Count() == 3);
			PR_EXPECT(FEql(ma.Mean(), T(4)));

			ma.Add(T(8));
			PR_EXPECT(ma.Count() == 4);
			PR_EXPECT(FEql(ma.Mean(), T(5)));
		}
		PRUnitTestMethod(MovingAvrWindowWrap, float, double)
		{
			// Once full, the window should slide: oldest value dropped, newest added.
			// Window = {2, 4, 6, 8} → mean = 5
			// Add 10 → window = {4, 6, 8, 10} → mean = 7
			auto ma = MovingAvr<16, T>(4);
			ma.Add(T(2));
			ma.Add(T(4));
			ma.Add(T(6));
			ma.Add(T(8));
			PR_EXPECT(FEql(ma.Mean(), T(5)));

			// Window wraps: drop 2, add 10
			ma.Add(T(10));
			PR_EXPECT(ma.Count() == 4); // count stays at window size
			PR_EXPECT(FEqlAbsolute(ma.Mean(), T(7), T(1e-5)));

			// Drop 4, add 12 → {6, 8, 10, 12} → mean = 9
			ma.Add(T(12));
			PR_EXPECT(FEqlAbsolute(ma.Mean(), T(9), T(1e-5)));
		}
		PRUnitTestMethod(MovingAvrConstant, float, double)
		{
			// Constant values should give zero variance
			auto ma = MovingAvr<16, T>(5);
			for (int i = 0; i != 20; ++i)
				ma.Add(T(3));

			PR_EXPECT(FEqlAbsolute(ma.Mean(), T(3), T(1e-6)));
			PR_EXPECT(FEqlAbsolute(ma.PopStdVar(), T(0), T(1e-10)));
		}
		PRUnitTestMethod(MovingAvrReset, float, double)
		{
			auto ma = MovingAvr<16, T>(4);
			ma.Add(T(1));
			ma.Add(T(2));
			ma.Add(T(3));

			ma.Reset(3); // reset with smaller window
			PR_EXPECT(ma.Count() == 0);

			ma.Add(T(10));
			PR_EXPECT(ma.Count() == 1);
			PR_EXPECT(FEql(ma.Mean(), T(10)));
		}
		PRUnitTestMethod(MovingAvrVariance, float, double)
		{
			// Window = {1, 2, 3, 4}, mean = 2.5
			// Pop var = ((1-2.5)² + (2-2.5)² + (3-2.5)² + (4-2.5)²) / 4 = (2.25+0.25+0.25+2.25)/4 = 5/4 = 1.25
			auto ma = MovingAvr<16, T>(4);
			ma.Add(T(1));
			ma.Add(T(2));
			ma.Add(T(3));
			ma.Add(T(4));

			PR_EXPECT(FEqlAbsolute(ma.Mean(), T(2.5), T(1e-6)));
			PR_EXPECT(FEqlAbsolute(ma.PopStdVar(), T(1.25), T(1e-4)));

			// Sample var = 5/3 ≈ 1.6667
			PR_EXPECT(FEqlAbsolute(ma.SamStdVar(), T(5.0/3.0), T(1e-4)));
		}
		PRUnitTestMethod(ComponentCount, float, double)
		{
			auto c = stats::Count<T>{};
			c.Reset();
			PR_EXPECT(c.m_count == 0);

			c.Add(T(1));
			c.Add(T(2));
			c.Add(T(3));
			PR_EXPECT(c.m_count == 3);
		}
		PRUnitTestMethod(ComponentMinMax, float, double)
		{
			auto mm = stats::MinMax<T>{};
			mm.Reset();

			mm.Add(T(5));
			PR_EXPECT(mm.m_min == T(5));
			PR_EXPECT(mm.m_max == T(5));

			mm.Add(T(-3));
			PR_EXPECT(mm.m_min == T(-3));
			PR_EXPECT(mm.m_max == T(5));

			mm.Add(T(10));
			PR_EXPECT(mm.m_min == T(-3));
			PR_EXPECT(mm.m_max == T(10));

			mm.Add(T(0));
			PR_EXPECT(mm.m_min == T(-3));
			PR_EXPECT(mm.m_max == T(10));
		}
		PRUnitTestMethod(ComponentMean, float, double)
		{
			auto m = stats::Mean<T>{};
			m.Reset();

			m.Add(T(10), 1);
			PR_EXPECT(FEql(m.m_mean, T(10)));

			m.Add(T(20), 2);
			PR_EXPECT(FEql(m.m_mean, T(15)));

			m.Add(T(30), 3);
			PR_EXPECT(FEql(m.m_mean, T(20)));
		}
		PRUnitTestMethod(SMABasic, float, double)
		{
			// The SMA component type uses a dynamic vector buffer
			auto sma = stats::SMA<T>(3);
			PR_EXPECT(sma.Count() == 0);

			sma.Add(T(2));
			PR_EXPECT(sma.Count() == 1);
			PR_EXPECT(FEql(sma.Mean(), T(2)));

			sma.Add(T(4));
			sma.Add(T(6));
			PR_EXPECT(sma.Count() == 3);
			PR_EXPECT(FEql(sma.Mean(), T(4)));

			// Window full, now oldest drops off: {4, 6, 8}
			sma.Add(T(8));
			PR_EXPECT(sma.Count() == 3);
			PR_EXPECT(FEqlAbsolute(sma.Mean(), T(6), T(1e-5)));
		}
		PRUnitTestMethod(SMAReset, float, double)
		{
			auto sma = stats::SMA<T>(4);
			sma.Add(T(1));
			sma.Add(T(2));

			sma.Reset(2); // reset with different window size
			PR_EXPECT(sma.Count() == 0);

			sma.Add(T(100));
			PR_EXPECT(sma.Count() == 1);
			PR_EXPECT(FEql(sma.Mean(), T(100)));
		}
		PRUnitTestMethod(EMAComponent, float, double)
		{
			// The EMA component type
			auto ema = stats::EMA<T>(5);
			PR_EXPECT(ema.Count() == 0);

			// Before window full, acts like running average
			ema.Add(T(10));
			PR_EXPECT(ema.Count() == 1);
			PR_EXPECT(FEql(ema.Mean(), T(10)));

			ema.Add(T(20));
			PR_EXPECT(ema.Count() == 2);
			PR_EXPECT(FEql(ema.Mean(), T(15)));

			// Constant values should converge
			ema.Reset(3);
			for (int i = 0; i != 50; ++i)
				ema.Add(T(7));

			PR_EXPECT(FEqlAbsolute(ema.Mean(), T(7), T(1e-6)));
		}
		PRUnitTestMethod(NegativeValues, float, double)
		{
			// Verify stats work correctly with negative values
			auto s = Stat<T>{};
			s.Add(T(-5));
			s.Add(T(-3));
			s.Add(T(-1));

			PR_EXPECT(s.Count() == 3);
			PR_EXPECT(s.Min() == T(-5));
			PR_EXPECT(s.Max() == T(-1));
			PR_EXPECT(FEql(s.Mean(), T(-3)));

			// Pop var: ((−5−(−3))²+(−3−(−3))²+(−1−(−3))²)/3 = (4+0+4)/3 ≈ 2.667
			PR_EXPECT(FEqlAbsolute(s.PopStdVar(), T(8.0/3.0), T(1e-5)));
		}
		PRUnitTestMethod(LargeDataSet, float, double)
		{
			// Test numerical stability with a larger data set
			auto s = Stat<T>{};
			auto av = AvrVar<T>{};
			auto constexpr count = 1000;

			for (int i = 0; i != count; ++i)
			{
				auto val = static_cast<T>(i);
				s.Add(val);
				av.Add(val);
			}

			// Mean of 0..999 = 499.5
			PR_EXPECT(FEqlAbsolute(s.Mean(), T(499.5), T(0.01)));
			PR_EXPECT(FEqlAbsolute(av.Mean(), T(499.5), T(0.01)));

			// Both should agree
			PR_EXPECT(FEqlAbsolute(s.Mean(), av.Mean(), T(1e-5)));
			PR_EXPECT(FEqlAbsolute(s.PopStdVar(), av.PopStdVar(), T(0.1)));
		}
	};
}
#endif
