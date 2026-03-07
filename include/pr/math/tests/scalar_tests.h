//*****************************************************************************
// Maths library
//  Copyright (c) Rylogic Ltd 2002
//*****************************************************************************
#pragma once
#include "pr/math/math.h"

#if PR_UNITTESTS
#include "pr/common/unittests.h"
namespace pr::math::tests
{
	PRUnitTestClass(MathsCoreTests)
	{
		PRUnitTestMethod(PermutationTests)
		{
			{// 4-sequential
				int arr0[] = {1, 2, 3, 4};
				int const expected[][4] = {
					{ 1, 2, 3, 4 }, // 0
					{ 1, 2, 4, 3 }, // 1
					{ 1, 3, 2, 4 }, // 2
					{ 1, 3, 4, 2 }, // 3
					{ 1, 4, 2, 3 }, // 4
					{ 1, 4, 3, 2 }, // 5
					{ 2, 1, 3, 4 }, // 6
					{ 2, 1, 4, 3 }, // 7
					{ 2, 3, 1, 4 }, // 8
					{ 2, 3, 4, 1 }, // 9
					{ 2, 4, 1, 3 }, // 10
					{ 2, 4, 3, 1 }, // 11
					{ 3, 1, 2, 4 }, // 12
					{ 3, 1, 4, 2 }, // 13
					{ 3, 2, 1, 4 }, // 14
					{ 3, 2, 4, 1 }, // 15
					{ 3, 4, 1, 2 }, // 16
					{ 3, 4, 2, 1 }, // 17
					{ 4, 1, 2, 3 }, // 18
					{ 4, 1, 3, 2 }, // 19
					{ 4, 2, 1, 3 }, // 20
					{ 4, 2, 3 ,1 }, // 21
					{ 4 ,3 ,1 ,2 }, // 22
					{ 4 ,3 ,2 ,1 }  // 23
				};
				int i = 0;
				for (auto arr : PermutationsOf<int>(arr0))
				{
					PR_EXPECT(std::ranges::equal(arr, expected[i]));
					++i;
				}
				PR_EXPECT(i == 24); // == 4!
			}
			{// non-sequential
				int arr0[] = {-1, 4, 11, 20};
				int const expected0[] = {4, -1, 11, 20}; // At i == 6
				int const expected1[] = {11, -1, 20, 4}; // At i == 13
				int i = 0;
				for (auto arr : PermutationsOf<int>(arr0))
				{
					if (i == 6) PR_EXPECT(std::ranges::equal(arr, expected0));
					if (i == 13) PR_EXPECT(std::ranges::equal(arr, expected1));
					++i;
				}
				PR_EXPECT(i == 24); // == 4!
			}
			{// large number of permutations
				int arr0[] = {-10, -9, -8, -1, 0, +1, +3, +6, +9};
				int const expected0[] = { -8, 0, 6, 9, -9, 1, 3, -10, -1 }; // At i == 100000
				int const expected1[] = { 0, 9, 1, 3, -1, -9, -8, -10, 6 }; // At i == 200000
				int i = 0;
				for (auto arr : PermutationsOf<int>(arr0))
				{
					if (i == 100000) PR_EXPECT(std::ranges::equal(arr, expected0));
					if (i == 200000) PR_EXPECT(std::ranges::equal(arr, expected1));
					++i;
				}
				PR_EXPECT(i == 362880); // == 9!
			}
		}
		PRUnitTestMethod(FloatingPointCompareTests)
		{
			// Note: not testing double here, if it's correct for float it should be correct for double.
			// Testing doubles would need different constants for each test.
			float const _6dp = 1.000000111e-6f;

			// Regular large numbers - generally not problematic
			PR_EXPECT(FEqlRelative(1000000.0f, 1000001.0f, _6dp));
			PR_EXPECT(FEqlRelative(1000001.0f, 1000000.0f, _6dp));
			PR_EXPECT(!FEqlRelative(1000000.0f, 1000010.0f, _6dp));
			PR_EXPECT(!FEqlRelative(1000010.0f, 1000000.0f, _6dp));

			// Negative large numbers
			PR_EXPECT(FEqlRelative(-1000000.0f, -1000001.0f, _6dp));
			PR_EXPECT(FEqlRelative(-1000001.0f, -1000000.0f, _6dp));
			PR_EXPECT(!FEqlRelative(-1000000.0f, -1000010.0f, _6dp));
			PR_EXPECT(!FEqlRelative(-1000010.0f, -1000000.0f, _6dp));

			// Numbers around 1
			PR_EXPECT(FEqlRelative(1.0000001f, 1.0000002f, _6dp));
			PR_EXPECT(FEqlRelative(1.0000002f, 1.0000001f, _6dp));
			PR_EXPECT(!FEqlRelative(1.0000020f, 1.0000010f, _6dp));
			PR_EXPECT(!FEqlRelative(1.0000010f, 1.0000020f, _6dp));

			// Numbers around -1
			PR_EXPECT(FEqlRelative(-1.0000001f, -1.0000002f, _6dp));
			PR_EXPECT(FEqlRelative(-1.0000002f, -1.0000001f, _6dp));
			PR_EXPECT(!FEqlRelative(-1.0000010f, -1.0000020f, _6dp));
			PR_EXPECT(!FEqlRelative(-1.0000020f, -1.0000010f, _6dp));

			// Numbers between 1 and 0
			PR_EXPECT(FEqlRelative(0.000000001000001f, 0.000000001000002f, _6dp));
			PR_EXPECT(FEqlRelative(0.000000001000002f, 0.000000001000001f, _6dp));
			PR_EXPECT(!FEqlRelative(0.000000000100002f, 0.000000000100001f, _6dp));
			PR_EXPECT(!FEqlRelative(0.000000000100001f, 0.000000000100002f, _6dp));

			// Numbers between -1 and 0
			PR_EXPECT(FEqlRelative(-0.0000000010000001f, -0.0000000010000002f, _6dp));
			PR_EXPECT(FEqlRelative(-0.0000000010000002f, -0.0000000010000001f, _6dp));
			PR_EXPECT(!FEqlRelative(-0.0000000001000002f, -0.0000000001000001f, _6dp));
			PR_EXPECT(!FEqlRelative(-0.0000000001000001f, -0.0000000001000002f, _6dp));

			// Comparisons involving zero
			PR_EXPECT(FEqlRelative(+0.0f, +0.0f, _6dp));
			PR_EXPECT(FEqlRelative(+0.0f, -0.0f, _6dp));
			PR_EXPECT(FEqlRelative(-0.0f, -0.0f, _6dp));
			PR_EXPECT(FEqlRelative(+0.000001f, +0.0f, _6dp));
			PR_EXPECT(FEqlRelative(+0.0f, +0.000001f, _6dp));
			PR_EXPECT(FEqlRelative(-0.000001f, +0.0f, _6dp));
			PR_EXPECT(FEqlRelative(+0.0f, -0.000001f, _6dp));
			PR_EXPECT(!FEqlRelative(+0.00001f, +0.0f, _6dp));
			PR_EXPECT(!FEqlRelative(+0.0f, +0.00001f, _6dp));
			PR_EXPECT(!FEqlRelative(-0.00001f, +0.0f, _6dp));
			PR_EXPECT(!FEqlRelative(+0.0f, -0.00001f, _6dp));

			// Comparisons involving extreme values (overflow potential)
			auto float_hi = limits<float>::max();
			auto float_lo = limits<float>::lowest();
			PR_EXPECT(FEqlRelative(float_hi, float_hi, _6dp));
			PR_EXPECT(!FEqlRelative(float_hi, float_lo, _6dp));
			PR_EXPECT(!FEqlRelative(float_lo, float_hi, _6dp));
			PR_EXPECT(FEqlRelative(float_lo, float_lo, _6dp));
			PR_EXPECT(!FEqlRelative(float_hi, float_hi / 2, _6dp));
			PR_EXPECT(!FEqlRelative(float_hi, float_lo / 2, _6dp));
			PR_EXPECT(!FEqlRelative(float_lo, float_hi / 2, _6dp));
			PR_EXPECT(!FEqlRelative(float_lo, float_lo / 2, _6dp));

			// Comparisons involving infinities
			PR_EXPECT(FEqlRelative(+limits<float>::infinity(), +limits<float>::infinity(), _6dp));
			PR_EXPECT(FEqlRelative(-limits<float>::infinity(), -limits<float>::infinity(), _6dp));
			PR_EXPECT(!FEqlRelative(-limits<float>::infinity(), +limits<float>::infinity(), _6dp));
			PR_EXPECT(!FEqlRelative(+limits<float>::infinity(), +limits<float>::max(), _6dp));
			PR_EXPECT(!FEqlRelative(-limits<float>::infinity(), -limits<float>::max(), _6dp));

			// Comparisons involving NaN values
			PR_EXPECT(!FEqlRelative(limits<float>::quiet_NaN(), limits<float>::quiet_NaN(), _6dp));
			PR_EXPECT(!FEqlRelative(limits<float>::quiet_NaN(), +0.0f, _6dp));
			PR_EXPECT(!FEqlRelative(-0.0f, limits<float>::quiet_NaN(), _6dp));
			PR_EXPECT(!FEqlRelative(limits<float>::quiet_NaN(), -0.0f, _6dp));
			PR_EXPECT(!FEqlRelative(+0.0f, limits<float>::quiet_NaN(), _6dp));
			PR_EXPECT(!FEqlRelative(limits<float>::quiet_NaN(), +limits<float>::infinity(), _6dp));
			PR_EXPECT(!FEqlRelative(+limits<float>::infinity(), limits<float>::quiet_NaN(), _6dp));
			PR_EXPECT(!FEqlRelative(limits<float>::quiet_NaN(), -limits<float>::infinity(), _6dp));
			PR_EXPECT(!FEqlRelative(-limits<float>::infinity(), limits<float>::quiet_NaN(), _6dp));
			PR_EXPECT(!FEqlRelative(limits<float>::quiet_NaN(), +limits<float>::max(), _6dp));
			PR_EXPECT(!FEqlRelative(+limits<float>::max(), limits<float>::quiet_NaN(), _6dp));
			PR_EXPECT(!FEqlRelative(limits<float>::quiet_NaN(), -limits<float>::max(), _6dp));
			PR_EXPECT(!FEqlRelative(-limits<float>::max(), limits<float>::quiet_NaN(), _6dp));
			PR_EXPECT(!FEqlRelative(limits<float>::quiet_NaN(), +limits<float>::min(), _6dp));
			PR_EXPECT(!FEqlRelative(+limits<float>::min(), limits<float>::quiet_NaN(), _6dp));
			PR_EXPECT(!FEqlRelative(limits<float>::quiet_NaN(), -limits<float>::min(), _6dp));
			PR_EXPECT(!FEqlRelative(-limits<float>::min(), limits<float>::quiet_NaN(), _6dp));

			// Comparisons of numbers on opposite sides of 0
			PR_EXPECT(!FEqlRelative(+1.0f, -1.0f, _6dp));
			PR_EXPECT(!FEqlRelative(-1.0f, +1.0f, _6dp));
			PR_EXPECT(!FEqlRelative(+1.000000001f, -1.0f, _6dp));
			PR_EXPECT(!FEqlRelative(-1.0f, +1.000000001f, _6dp));
			PR_EXPECT(!FEqlRelative(-1.000000001f, +1.0f, _6dp));
			PR_EXPECT(!FEqlRelative(+1.0f, -1.000000001f, _6dp));
			PR_EXPECT(FEqlRelative(2 * limits<float>::min(), 0.0f, _6dp));
			PR_EXPECT(!FEqlRelative(limits<float>::min(), -limits<float>::min(), _6dp));

			// The really tricky part - comparisons of numbers very close to zero.
			PR_EXPECT(FEqlRelative(+limits<float>::min(), +limits<float>::min(), _6dp));
			PR_EXPECT(!FEqlRelative(+limits<float>::min(), -limits<float>::min(), _6dp));
			PR_EXPECT(!FEqlRelative(-limits<float>::min(), +limits<float>::min(), _6dp));
			PR_EXPECT(FEqlRelative(+limits<float>::min(), 0.0f, _6dp));
			PR_EXPECT(FEqlRelative(-limits<float>::min(), 0.0f, _6dp));
			PR_EXPECT(FEqlRelative(0.0f, +limits<float>::min(), _6dp));
			PR_EXPECT(FEqlRelative(0.0f, -limits<float>::min(), _6dp));

			PR_EXPECT(!FEqlRelative(0.000000001f, -limits<float>::min(), _6dp));
			PR_EXPECT(!FEqlRelative(0.000000001f, +limits<float>::min(), _6dp));
			PR_EXPECT(!FEqlRelative(+limits<float>::min(), 0.000000001f, _6dp));
			PR_EXPECT(!FEqlRelative(-limits<float>::min(), 0.000000001f, _6dp));
		}
		PRUnitTestMethod(FiniteTests)
		{
			volatile auto f0 = 0.0f;
			volatile auto d0 = 0.0;
			PR_EXPECT(IsFinite(1.0f));
			PR_EXPECT(IsFinite(limits<int>::max()));
			PR_EXPECT(!IsFinite(1.0f / f0));
			PR_EXPECT(!IsFinite(0.0 / d0));
			PR_EXPECT(!IsFinite(11, 10));
		}
		PRUnitTestMethod(AbsTests, float, double)
		{
			PR_EXPECT(Abs(-T(1)) == Abs(-T(1)));
			PR_EXPECT(Abs(-T(1)) == Abs(+T(1)));
			PR_EXPECT(Abs(+T(1)) == Abs(+T(1)));
		}
		PRUnitTestMethod(AnyAllTests)
		{
			float arr0[] = {1.0f, 2.0f, 0.0f, -4.0f};
			auto are_zero = [](float x) { return x == 0.0f; };
			auto not_zero = [](float x) { return x != 0.0f; };

			PR_EXPECT(!All(arr0, are_zero));
			PR_EXPECT(!All(arr0, not_zero));
			PR_EXPECT(Any(arr0, not_zero));
			PR_EXPECT(Any(arr0, are_zero));
		}
		PRUnitTestMethod(MinMaxClampTests)
		{
			PR_EXPECT(Min(1, 2, -3, 4, -5) == -5);
			PR_EXPECT(Max(1, 2, -3, 4, -5) == 4);
			PR_EXPECT(Clamp(-1, 0, 10) == 0);
			PR_EXPECT(Clamp(3, 0, 10) == 3);
			PR_EXPECT(Clamp(12, 0, 10) == 10);
		}
		PRUnitTestMethod(WrapTests)
		{
			PR_EXPECT(Wrap(-1, 0, 3) == 2); // [0, 3)
			PR_EXPECT(Wrap(+0, 0, 3) == 0);
			PR_EXPECT(Wrap(+1, 0, 3) == 1);
			PR_EXPECT(Wrap(+2, 0, 3) == 2);
			PR_EXPECT(Wrap(+3, 0, 3) == 0);
			PR_EXPECT(Wrap(+4, 0, 3) == 1);

			PR_EXPECT(Wrap(-1.0, 0.0, 3.0) == 2.0); // [0, 3)
			PR_EXPECT(Wrap(+0.0, 0.0, 3.0) == 0.0);
			PR_EXPECT(Wrap(+1.0, 0.0, 3.0) == 1.0);
			PR_EXPECT(Wrap(+2.0, 0.0, 3.0) == 2.0);
			PR_EXPECT(Wrap(+3.0, 0.0, 3.0) == 0.0);
			PR_EXPECT(Wrap(+4.0, 0.0, 3.0) == 1.0);

			PR_EXPECT(Wrap(-3, -2, +3) == +2); // [-2,+2]
			PR_EXPECT(Wrap(-2, -2, +3) == -2);
			PR_EXPECT(Wrap(-1, -2, +3) == -1);
			PR_EXPECT(Wrap(+0, -2, +3) == +0);
			PR_EXPECT(Wrap(+1, -2, +3) == +1);
			PR_EXPECT(Wrap(+2, -2, +3) == +2);
			PR_EXPECT(Wrap(+3, -2, +3) == -2);

			PR_EXPECT(Wrap(-3.0, -2.0, +3.0) == +2.0); // [-2,+2]
			PR_EXPECT(Wrap(-2.0, -2.0, +3.0) == -2.0);
			PR_EXPECT(Wrap(-1.0, -2.0, +3.0) == -1.0);
			PR_EXPECT(Wrap(+0.0, -2.0, +3.0) == +0.0);
			PR_EXPECT(Wrap(+1.0, -2.0, +3.0) == +1.0);
			PR_EXPECT(Wrap(+2.0, -2.0, +3.0) == +2.0);
			PR_EXPECT(Wrap(+3.0, -2.0, +3.0) == -2.0);

			PR_EXPECT(Wrap(+1, +2, +5) == 4); // [+2,+5)
			PR_EXPECT(Wrap(+2, +2, +5) == 2);
			PR_EXPECT(Wrap(+3, +2, +5) == 3);
			PR_EXPECT(Wrap(+4, +2, +5) == 4);
			PR_EXPECT(Wrap(+5, +2, +5) == 2);
			PR_EXPECT(Wrap(+6, +2, +5) == 3);

			PR_EXPECT(Wrap(-3, 0, 1) == 0); // [0,1)
			PR_EXPECT(Wrap(-2, 0, 1) == 0);
			PR_EXPECT(Wrap(-1, 0, 1) == 0);
			PR_EXPECT(Wrap(+0, 0, 1) == 0);
			PR_EXPECT(Wrap(+1, 0, 1) == 0);
			PR_EXPECT(Wrap(+2, 0, 1) == 0);
			PR_EXPECT(Wrap(+3, 0, 1) == 0);

			PR_EXPECT(Wrap(-3, -1, 0) == -1); // [-1,0)
			PR_EXPECT(Wrap(-2, -1, 0) == -1);
			PR_EXPECT(Wrap(-1, -1, 0) == -1);
			PR_EXPECT(Wrap(+0, -1, 0) == -1);
			PR_EXPECT(Wrap(+1, -1, 0) == -1);
			PR_EXPECT(Wrap(+2, -1, 0) == -1);
		}
		PRUnitTestMethod(SmallestLargestElementTests)
		{
			int arr0[] = {1, 2, 3, 4, 5};
			int arr1[] = {2, 1, 3, 4, 5};
			int arr2[] = {2, 3, 1, 4, 5};
			int arr3[] = {2, 3, 4, 1, 5};
			int arr4[] = {2, 3, 4, 5, 1};

			PR_EXPECT(MinElement(arr0) == 1);
			PR_EXPECT(MinElement(arr1) == 1);
			PR_EXPECT(MinElement(arr2) == 1);
			PR_EXPECT(MinElement(arr3) == 1);
			PR_EXPECT(MinElement(arr4) == 1);

			float arr5[] = {1, 2, 3, 4, 5};
			float arr6[] = {1, 2, 3, 5, 4};
			float arr7[] = {2, 3, 5, 1, 4};
			float arr8[] = {2, 5, 3, 4, 1};
			float arr9[] = {5, 2, 3, 4, 1};
			PR_EXPECT(MaxElement(arr5) == 5);
			PR_EXPECT(MaxElement(arr6) == 5);
			PR_EXPECT(MaxElement(arr7) == 5);
			PR_EXPECT(MaxElement(arr8) == 5);
			PR_EXPECT(MaxElement(arr9) == 5);
		}
		PRUnitTestMethod(SmallestLargestElementIndexTests)
		{
			int arr0[] = {1, 2, 3, 4, 5};
			int arr1[] = {2, 1, 3, 4, 5};
			int arr2[] = {2, 3, 1, 4, 5};
			int arr3[] = {2, 3, 4, 1, 5};
			int arr4[] = {2, 3, 4, 5, 1};

			PR_EXPECT(MinElementIndex(arr0) == 0);
			PR_EXPECT(MinElementIndex(arr1) == 1);
			PR_EXPECT(MinElementIndex(arr2) == 2);
			PR_EXPECT(MinElementIndex(arr3) == 3);
			PR_EXPECT(MinElementIndex(arr4) == 4);

			float arr5[] = {1, 2, 3, 4, 5};
			float arr6[] = {1, 2, 3, 5, 4};
			float arr7[] = {2, 3, 5, 1, 4};
			float arr8[] = {2, 5, 3, 4, 1};
			float arr9[] = {5, 2, 3, 4, 1};
			PR_EXPECT(MaxElementIndex(arr5) == 4);
			PR_EXPECT(MaxElementIndex(arr6) == 3);
			PR_EXPECT(MaxElementIndex(arr7) == 2);
			PR_EXPECT(MaxElementIndex(arr8) == 1);
			PR_EXPECT(MaxElementIndex(arr9) == 0);
		}
		PRUnitTestMethod(TruncTests)
		{
			PR_EXPECT(Trunc(1.9f) == 1.0f);
			PR_EXPECT(Trunc(1.9f, ETruncate::ToNearest) == 2.0f);
			PR_EXPECT(Trunc(10000000000000.9) == 10000000000000.0);
		}
		PRUnitTestMethod(CosAngleTests, float, double)
		{
			PR_EXPECT(FEql(CosAngle(T(1), T(1), constants<T>::root2) - std::cos(DegreesToRadians(T(90))), T(0)));
			PR_EXPECT(FEql(Angle(T(1), T(1), constants<T>::root2), DegreesToRadians(T(90))));
			PR_EXPECT(FEql(Length(T(1), T(1), DegreesToRadians(T(90))), constants<T>::root2));
		}
		PRUnitTestMethod(FractionTests, float, double)
		{
			PR_EXPECT(FEql(Frac<T>(-T(5), T(2), T(5)), T(7) / T(10)));
		}
		PRUnitTestMethod(CubeRootTests)
		{
			{// 32bit
				auto a = 1.23456789123456789f;
				auto b = Cubert(a * a * a);
				PR_EXPECT(FEqlRelative(a, b, 0.000001f));
			}
			{// 64bit
				auto a = 1.23456789123456789;
				auto b = Cubert(a * a * a);
				PR_EXPECT(FEqlRelative(a, b, 0.000000000001));
			}
		}
		PRUnitTestMethod(SqrtRootTests)
		{
			PR_EXPECT(Sqrt(64.0) == 8.0);
			static_assert(ISqrt(64) == 8);
			static_assert(ISqrt(4294836225) == 65535);
			static_assert(ISqrt(10000000000000000000LL) == 3162277660LL);
			static_assert(ISqrt(18446744065119617025LL) == 4294967295LL);
		}
		PRUnitTestMethod(ArithmeticSequenceTests)
		{
			int i = 0;
			constexpr int expected[] = { 2, 7, 12, 17 };
			for (auto v : ArithmeticSequence(2, 5))
			{
				PR_EXPECT(v == expected[i]);
				if (++i == std::size(expected))
					break;
			}

			PR_EXPECT(ArithmeticSum(0, 2, 4) == 20);
			PR_EXPECT(ArithmeticSum(4, 2, 2) == 18);
			PR_EXPECT(ArithmeticSum(1, 2, 0) == 1);
			PR_EXPECT(ArithmeticSum(1, 2, 5) == 36);
		}
		PRUnitTestMethod(GeometricSequenceTests)
		{
			int i = 0;
			constexpr int expected[] = { 2, 10, 50, 250 };
			for (auto v : GeometricSequence(2, 5))
			{
				PR_EXPECT(v == expected[i]);
				if (++i == std::size(expected))
					break;
			}

			PR_EXPECT(GeometricSum(1, 2, 4) == 31);
			PR_EXPECT(GeometricSum(4, 2, 2) == 28);
			PR_EXPECT(GeometricSum(1, 3, 0) == 1);
			PR_EXPECT(GeometricSum(1, 3, 5) == 364);
		}
		PRUnitTestMethod(GCFAndLCMTests)
		{
			static_assert(GreatestCommonFactor(12, 8) == 4);
			static_assert(GreatestCommonFactor(7, 13) == 1); // co-prime
			static_assert(GreatestCommonFactor(100, 75) == 25);
			static_assert(GreatestCommonFactor(0, 5) == 5);
			static_assert(GreatestCommonFactor(17, 17) == 17);

			static_assert(LeastCommonMultiple(4, 6) == 12);
			static_assert(LeastCommonMultiple(7, 13) == 91); // co-prime
			static_assert(LeastCommonMultiple(12, 8) == 24);
		}
		PRUnitTestMethod(PadTests)
		{
			static_assert(Pad(5, 4) == 3);  // 5 → 8, pad = 3
			static_assert(Pad(8, 4) == 0);  // already aligned
			static_assert(Pad(1, 8) == 7);  // 1 → 8, pad = 7
			static_assert(Pad(0, 16) == 0); // 0 is aligned

			static_assert(PadTo(5, 4) == 8);
			static_assert(PadTo(8, 4) == 8);
			static_assert(PadTo(1, 8) == 8);
			static_assert(PadTo(0, 16) == 0);
			static_assert(PadTo(17, 16) == 32);
		}
	};
}
#endif
