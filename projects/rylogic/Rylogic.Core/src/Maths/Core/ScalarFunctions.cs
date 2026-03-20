//***************************************************
// Math
//  Copyright (c) Rylogic Ltd 2008
//***************************************************

using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using Rylogic.Common;
using Rylogic.Extn;
using Rylogic.Utility;

namespace Rylogic.Maths
{
	/// <summary>scalar functions</summary>
	public static partial class Math_
	{
		public static bool IsFinite(float x) { return !float.IsInfinity(x) && !float.IsNaN(x); }
		public static bool IsFinite(double x) { return !double.IsInfinity(x) && !double.IsNaN(x); }
		public static bool IsNaN(float x) { return float.IsNaN(x); }
		public static bool IsNaN(double x) { return double.IsNaN(x); }
		public static int SignI(bool positive) { return positive ? 1 : -1; }
		public static float SignF(bool positive) { return positive ? 1f : -1f; }
		public static double SignD(bool positive) { return positive ? 1.0 : -1.0; }
		public static decimal SignM(bool positive) { return positive ? 1m : -1m; }
		public static int Sign(int x) { return SignI(x >= 0.0f); }
		public static float Sign(float x) { return SignF(x >= 0.0f); }
		public static double Sign(double x) { return SignD(x >= 0.0); }
		public static decimal Sign(decimal x) { return SignM(x >= 0m); }
		public static int OneIfZero(int x) { return x != 0 ? x : 1; }
		public static float OneIfZero(float x) { return x != 0f ? x : 1f; }
		public static int Sqr(int x) { return x * x; }
		public static float Sqr(float x) { return x * x; }
		public static double Sqr(double x) { return x * x; }
		public static float Sqrt(float x) { return (float)Sqrt((double)x); }
		public static double Sqrt(double x) { return Math.Sqrt(x); }
		public static float Cubed(float x) { return x * x * x; }
		public static double Cubed(double x) { return x * x * x; }
		public static float CubeRoot(float x) { return (float)CubeRoot((double)x); }
		public static double CubeRoot(double x) { return Math.Pow(x, 1.0 / 3.0); }
		public static float DegreesToRadians(float degrees) { return (float)(degrees * TauBy360); }
		public static double DegreesToRadians(double degrees) { return degrees * TauBy360; }
		public static v4 DegreesToRadians(v4 degrees) { return new v4(DegreesToRadians(degrees.x), DegreesToRadians(degrees.y), DegreesToRadians(degrees.z), DegreesToRadians(degrees.w)); }
		public static float RadiansToDegrees(float radians) { return (float)(radians * _360ByTau); }
		public static double RadiansToDegrees(double radians) { return radians * _360ByTau; }
		public static v4 RadiansToDegrees(v4 degrees) { return new v4(RadiansToDegrees(degrees.x), RadiansToDegrees(degrees.y), RadiansToDegrees(degrees.z), RadiansToDegrees(degrees.w)); }
		public static void Swap<T>(ref T lhs, ref T rhs)
		{
			var tmp = lhs;
			lhs = rhs;
			rhs = tmp;
		}

		/// <summary>Length</summary>
		public static float LengthSq(float x, float y)                        { return Sqr(x) + Sqr(y); }
		public static float LengthSq(float x, float y, float z)               { return Sqr(x) + Sqr(y) + Sqr(z); }
		public static float LengthSq(float x, float y, float z, float w)      { return Sqr(x) + Sqr(y) + Sqr(z) + Sqr(w); }
		public static double LengthSq(double x, double y)                     { return Sqr(x) + Sqr(y); }
		public static double LengthSq(double x, double y, double z)           { return Sqr(x) + Sqr(y) + Sqr(z); }
		public static double LengthSq(double x, double y, double z, double w) { return Sqr(x) + Sqr(y) + Sqr(z) + Sqr(w); }
		public static float Length0(float x, float y)                          { return Sqrt(LengthSq(x, y)); }
		public static float Length0(float x, float y, float z)                 { return Sqrt(LengthSq(x, y, z)); }
		public static float Length0(float x, float y, float z, float w)        { return Sqrt(LengthSq(x, y, z, w)); }
		public static double Length0(double x, double y)                       { return Sqrt(LengthSq(x, y)); }
		public static double Length0(double x, double y, double z)             { return Sqrt(LengthSq(x, y, z)); }
		public static double Length0(double x, double y, double z, double w)   { return Sqrt(LengthSq(x, y, z, w)); }

		// Floating point comparisons

		/// <summary>
		/// Compare floats for equality.
		/// *WARNING* 'tol' is an absolute epsilon. Returns true if a is in the range (b-tol,b+tol)</summary>
		public static bool FEqlAbsolute(float a, float b, float tol)
		{
			Debug.Assert(float.IsNaN(tol) || tol >= 0); // NaN is not an error, comparisons with NaN are defined to always be false
			return Math.Abs(a - b) < tol;
		}
		public static bool FEqlAbsolute(double a, double b, double tol)
		{
			Debug.Assert(double.IsNaN(tol) || tol >= 0); // NaN is not an error, comparisons with NaN are defined to always be false
			return Math.Abs(a - b) < tol;
		}

		/// <summary>
		/// Compare floats for equality.
		/// *WARNING* 'tol' is the fraction of the largest value. i.e. abs(a-b) &lt; tol * max(abs(a), abs(b))
		/// Do not use this expecting this behaviour: a.Within(b - tol, b + tol)</summary>
		public static bool FEqlRelative(float a, float b, float tol)
		{
			// Floating point compare is dangerous and subtle.
			// See: https://randomascii.wordpress.com/2012/02/25/comparing-floating-point-numbers-2012-edition/
			// and: http://floating-point-gui.de/errors/NearlyEqualsTest.java
			// Tests against zero treat 'tol' as an absolute difference threshold.
			// Tests between two non-zero values use 'tol' as a relative difference threshold.
			// i.e.
			//    FEql(2e-30, 1e-30) == false
			//    FEql(2e-30 - 1e-30, 0) == true

			// Handles tests against zero where relative error is meaningless
			// Tests with 'b == 0' are the most common so do them first
			if (b == 0) return Math.Abs(a) < tol;
			if (a == 0) return Math.Abs(b) < tol;

			// Handle infinities and exact values
			if (a == b) return true;

			// Test relative error as a fraction of the largest value
			return FEqlAbsolute(a, b, tol * Math.Max(Math.Abs(a), Math.Abs(b)));
		}
		public static bool FEqlRelative(double a, double b, double tol)
		{
			if (a == b) return true;
			if (b == 0) return Math.Abs(a) < tol;
			if (a == 0) return Math.Abs(b) < tol;
			return FEqlAbsolute(a, b, tol * Math.Max(Math.Abs(a), Math.Abs(b)));
		}
		public static bool FEql(float a, float b)
		{
			return FEqlRelative(a, b, TinyF);
		}
		public static bool FEql(double a, double b)
		{
			return FEqlRelative(a, b, TinyD);
		}
		public static IEqualityComparer<float> FEqlRelComparer(float tol)
		{
			return Eql<float>.From((l, r) => FEqlRelative(l, r, tol));
		}
		public static IEqualityComparer<float> FEqlAbsComparer(float tol)
		{
			return Eql<float>.From((l, r) => FEqlAbsolute(l, r, tol));
		}
		public static IEqualityComparer<double> FEqlRelComparer(double tol)
		{
			return Eql<double>.From((l, r) => FEqlRelative(l, r, tol));
		}
		public static IEqualityComparer<double> FEqlAbsComparer(double tol)
		{
			return Eql<double>.From((l, r) => FEqlAbsolute(l, r, tol));
		}

		/// <summary>Absolute value of 'a'</summary>
		public static T Abs<T>(T a)
		{
			return Operators<T>.GreaterEql(a, default!) ? a : Operators<T>.Neg(a);
		}

		/// <summary>Return 'a/b', or 'def' if 'b' is zero</summary>
		public static T Div<T>(T a, T b, T def = default!)
		{
			return !Equals(b, default) ? Operators<T>.Div(a, b) : def!;
		}

		/// <summary>Minimum value</summary>
		public static T Min<T>(T lhs, T rhs)// where T : IComparable<T>
		{
			return Operators<T>.LessEql(lhs, rhs) ? lhs : rhs;
			//return lhs.CompareTo(rhs) <= 0 ? lhs : rhs;
		}
		public static T Min<T>(T lhs, params T[] rhs)// where T : IComparable<T>
		{
			foreach (var r in rhs) lhs = Min(lhs, r);
			return lhs;
		}

		/// <summary>Maximum value</summary>
		public static T Max<T>(T lhs, T rhs)// where T : IComparable<T>
		{
			return Operators<T>.GreaterEql(lhs, rhs) ? lhs : rhs;
			//return lhs.CompareTo(rhs) >= 0 ? lhs : rhs;
		}
		public static T Max<T>(T lhs, params T[] rhs)// where T : IComparable<T>
		{
			foreach (var r in rhs) lhs = Max(lhs, r);
			return lhs;
		}

		/// <summary>Clamp value to an inclusive range</summary>
		public static T Clamp<T>(T x, T min, T max)// where T : IComparable<T>
		{
			Debug.Assert(Operators<T>.LessEql(min, max));//min.CompareTo(max) <= 0);
			//Debug.Assert(min.CompareTo(max) <= 0);
			return Max(min, Min(max, x));
			//return
			//	x.CompareTo(max) > 0 ? max :
			//	x.CompareTo(min) < 0 ? min : x;
		}

		/// <summary>Wrap 'x' to range [mn, mx)</summary>
		public static int Wrap(int x, int mn, int mx)
		{
			// Given the range [mn, mx) and 'x' somewhere on the number line
			// return 'x' wrapped into the range, allowing for 'x' < 'mn'.
			var range = mx - mn;
			return mn + (((x - mn) % range) + range) % range;
		}
		public static long Wrap(long x, long mn, long mx)
		{
			// Given the range [mn, mx) and 'x' somewhere on the number line
			// return 'x' wrapped into the range, allowing for 'x' < 'mn'.
			var range = mx - mn;
			return mn + (((x - mn) % range) + range) % range;
		}
		public static float Wrap(float x, float mn, float mx)
		{
			// Given the range [mn, mx) and 'x' somewhere on the number line
			// return 'x' wrapped into the range, allowing for 'x' < 'mn'.
			var range = mx - mn;
			return mn + (((x - mn) % range) + range) % range;
		}
		public static double Wrap(double x, double mn, double mx)
		{
			// Given the range [mn, mx) and 'x' somewhere on the number line
			// return 'x' wrapped into the range, allowing for 'x' < 'mn'.
			var range = mx - mn;
			return mn + (((x - mn) % range) + range) % range;
		}
		public static T Wrap<T>(T x, T mn, T mx) where T : IComparable<T>
		{
			// Given the range [mn, mx) and 'x' somewhere on the number line
			// return 'x' wrapped into the range, allowing for 'x' < 'mn'.
			Debug.Assert(mn.CompareTo(mx) <= 0);
			var range = Operators<T>.Sub(mx, mn);
			return Operators<T>.Add(mn, Operators<T>.Mod(Operators<T>.Add(Operators<T>.Mod(Operators<T>.Sub(x, mn), range), range), range));
		}

		/// <summary>Map 'x' to a value that 'ping pongs' between [0, N) (0 and N aren't repeated)</summary>
		public static long PingPong(long x, long N)
		{
			var n = N - 1;
			var v = Wrap(x, 0, 2*n);
			return (v < n) ? (v) : (2*n - v);
		}

		/// <summary>True if 'x' is within the interval '[min-tol,max+tol]'</summary>
		public static bool Within<T>(T min, T x, T max, T tol = default!) where T : IComparable<T>
		{
			min = Operators<T>.Sub(min, tol!);
			max = Operators<T>.Add(max, tol!);
			return x.CompareTo(min) >= 0 && x.CompareTo(max) <= 0;
		}

		/// <summary>Return the average of the given values</summary>
		public static T Average<T>(params T[] values)
		{
			if (values.Length == 0)
				throw new Exception("Average: no values provided");
			if (values.Length == 1)
				return values[0];

			var sum = default(T)!;
			foreach (var v in values)
				sum = Operators<T>.Add(sum, v);

			return Operators<T, int>.Div(sum, values.Length);
		}

		/// <summary>Return the median of the given values</summary>
		public static T Median<T>(params T[] values) where T : IComparable<T>
		{
			if (values.Length == 0)
				throw new Exception("Median: no values provided");
			if (values.Length == 1)
				return values[0];

			// If there are an even number of values, find the centre two and average them.
			// Otherwise, the median is just the middle value.
			var idx = values.Length / 2;
			return (values.Length & 1) == 0
				? Average(values.NthElement(idx - 1), values.NthElement(idx))
				: values.NthElement(idx);
		}

		/// <summary>Linear interpolate</summary>
		public static int Lerp(int lhs, int rhs, double frac)
		{
			return (int)Math.Round(Lerp((double)lhs, (double)rhs, frac), 0);
		}
		public static long Lerp(long lhs, long rhs, double frac)
		{
			return (long)Math.Round(Lerp((double)lhs, (double)rhs, frac), 0);
		}
		public static float Lerp(float lhs, float rhs, double frac)
		{
			return (float)Lerp((double)lhs, (double)rhs, frac);
		}
		public static double Lerp(double lhs, double rhs, double frac)
		{
			return lhs * (1.0 - frac) + rhs * (frac);
		}
		public static decimal Lerp(decimal lhs, decimal rhs, double frac)
		{
			return lhs * (decimal)(1.0 - frac) + rhs * (decimal)(frac);
		}
		public static T Lerp<T>(T lhs, T rhs, double frac)
		{
			return Operators<T>.Add(Operators<T, double>.Mul(lhs, 1.0 - frac), Operators<T, double>.Mul(rhs, frac));
		}
		public static T Lerp<T>(double frac, params T[] values)
		{
			var len = values.Length;
			if (len == 0)
				throw new Exception("No values to interpolate");

			// Scale 'frac' up to the number of values
			var fidx = frac * (len - 1);
			if (fidx <= 0.0) return values[0];
			if (fidx >= len - 1) return values[len - 1];

			var idx = (int)fidx;
			return Lerp(values[idx], values[idx + 1], fidx - idx);
		}

		/// <summary>Normalise to x to range [min,max]</summary>
		public static double Frac(int min, int x, int max)
		{
			return Frac((long)min, (long)x, (long)max);
		}
		public static double Frac(long min, long x, long max)
		{
			Debug.Assert(max != min);
			return (double)(x - min) / (max - min);
		}
		public static double Frac(float min, float x, float max)
		{
			return Frac((double)min, (double)x, (double)max);
		}
		public static double Frac(double min, double x, double max)
		{
			Debug.Assert(Math.Abs(max - min) > double.Epsilon);
			return (x - min) / (max - min);
		}
		public static decimal Frac(decimal min, decimal x, decimal max)
		{
			Debug.Assert(Math.Abs(max - min) > 0);
			return (x - min) / (max - min);
		}
		public static T Frac<T>(T min, T x, T max)
		{
			Debug.Assert(!Equals(min, max));
			return Operators<T>.Div(Operators<T>.Sub(x, min), Operators<T>.Sub(max, min));
		}

		/// <summary>Returns values in an arithmetic series</summary>
		public static IEnumerable<T> ArithmeticSeries<T>(T first, T step)
		{
			// Note: the simple way to do this causes accumulative error.
			for (int i = 0; ; ++i)
				yield return Operators<T>.Add(first, Operators<T,int>.Mul(step, i));
		}

		/// <summary>Returns values in an geometric series</summary>
		public static IEnumerable<T> GeometricSeries<T>(T first, T step)
		{
			// Note: the simple way to do this causes accumulative error.
			for (int i = 0; ; ++i)
				yield return Operators<T>.Mul(first, Operators<T,int>.Pow(step, i));
		}

		/// <summary>Return the sum of an arithmetic series</summary>
		public static T ArithmeticSeriesSum<T>(T first, T step, int count)
		{
			// Sn = 0.5 * n * (a1 - an)
			// an = a1 + d * (n - 1), since a(i+1) = a(i) + d
			var last = Operators<T>.Add(first, Operators<T,int>.Mul(step, count - 1));   // last = first + step * (count - 1)
			return Operators<T, double>.Mul(Operators<T>.Add(first, last), count * 0.5); // Sn = (first + last) * count / 2
		}

		/// <summary>Return the sum of a geometric series</summary>
		public static T GeometricSeriesSum<T>(T first, T step, int count)
		{
			if (count == 1)
				return first;

			// Sn = first * (1 - r^n) / (1 - r)
			var one = Operators<T>.Div(step, step);
			var numer = Operators<T>.Sub(one, Operators<T, int>.Pow(step, count));
			var denom = Operators<T>.Sub(one, step);
			return Operators<T>.Div(Operators<T>.Mul(first, numer), denom);
		}

		/// <summary>
		/// Return the greatest common factor between 'a' and 'b'
		/// Uses the Euclidean algorithm. If the greatest common factor is 1, then 'a' and 'b' are co-prime</summary>
		public static int GreatestCommonFactor(int a, int b)
		{
			while (b != 0) { int t = b; b = a % b; a = t; }
			return a;
		}
		public static int LeastCommonMultiple(int a, int b)
		{
			return (a * b) / GreatestCommonFactor(a, b);
		}

		/// <summary>Returns true if 'value' is a single digit integer multiple of a power of ten. (e.g. 2000, 300, 1, 90000. Not 1200, 234)</summary>
		public static bool IsIntegerPowerOfTen(long x)
		{
			x = Math.Abs(x);
			if (x == 0) return true;
			var log = Math.Log10(x);
			var exp = (long)Math.Pow(10, Math.Floor(log));
			return x == (long)Math.Floor((double)x / exp) * exp;
		}

		/// <summary>Return the exponents of 'value' in base 'radix' (starting at exponent 0)</summary>
		public static IEnumerable<long> Exponentiate(long value, int radix = 10)
		{
			for (long e = 1; value != 0; value /= radix, e *= radix)
			{
				var val = value % radix;
				if (val != 0)
					yield return val * e;
			}
		}

		/// <summary>Returns the order of magnitude of 'x'. E.g. OrderOfMagnitude(24301) = 10000, OrderOfMagnitude(0.085) = 0.01, </summary>
		public static double OrderOfMagnitude(double x)
		{
			if (x == 0) return 0;
			var sign = x > 0 ? +1 : -1;
			return sign * Math.Pow(10, Math.Floor(Math.Log10(Math.Abs(x))));
		}

		/// <summary>Return the nearest aesthetic value greaterequal than (dir = +1), less than (dir = -1),  or nearest to (dir = 0) 'x'</summary>
		public static double AestheticValue(double x, int dir = 0)
		{
			if (x == 0)
				return 0;

			// 'oom' is signed, so 'val' is always positive
			var oom = OrderOfMagnitude(x);
			var val = x / oom;

			// 0.5 and 10.0 are impossible values. They're used so that idx + 1 and idx - 1 don't need bounds checking.
			var nice = new[] { 0.5, 1.0, 2.0, 2.5, 5.0, 10.0 };
			var idx = nice.BinarySearch(s => s.CompareTo(val));

			if (idx >= 0)
				return nice[idx] * oom;

			idx = ~idx;
			dir = x >= 0 ? +dir : -dir;

			if (dir > 0)
				return nice[idx] * oom;
			if (dir < 0)
				return nice[idx - 1] * oom;

			var n = Frac(nice[idx - 1], val, nice[idx]) < 0.5 ? nice[idx - 1] : nice[idx];
			return n * oom;
		}

		/// <summary>Returns the radius of a sphere with the given volume</summary>
		public static double SphereRadius(double volume)
		{
			// The volume of a sphere, v = 2/3τr³ => r = ³root(3v/2τ)
			return CubeRoot(1.5 * volume / Tau);
		}

		/// <summary>Returns the cosine of the angle of the corner in a triangle with side lengths a,b,c for the corner opposite 'c'.</summary>
		public static double CosAngle(double a, double b, double c)
		{
			var numer = Sqr(a) + Sqr(b) - Sqr(c);
			var denom = 2 * a * b;
			var cos_angle = Clamp(denom != 0 ? numer / denom : 1, -1, 1);
			return cos_angle;
		}

		/// <summary>Returns the angle of the corner in a triangle with side lengths a,b,c for the corner opposite 'c'.</summary>
		public static double Angle(double a, double b, double c)
		{
			return Math.Acos(CosAngle(a,b,c));
		}

		/// <summary>Return the length of a triangle side given by two adjacent side lengths and an angle between them</summary>
		public static double EdgeLength(double adj0, double adj1, double angle)
		{
			var len_sq = adj0*adj0 + adj1*adj1 - 2.0f * adj0 * adj1 * Math.Cos(angle);
			return len_sq > 0 ? Sqrt(len_sq) : 0.0;
		}

		/// <summary>Truncate a number to 'decimal_places'</summary>
		public static decimal Truncate(decimal d, int decimal_places)
		{
			if (decimal_places < 0)
				throw new ArgumentOutOfRangeException(nameof(decimal_places), "value must be >= 0");

			var m = (decimal)Math.Pow(10.0, -decimal_places);
			return d - d % m;
		}
		public static double Truncate(double d, int decimal_places)
		{
			if (decimal_places < 0)
				throw new ArgumentOutOfRangeException(nameof(decimal_places), "value must be >= 0");

			var m = Math.Pow(10.0, -decimal_places);
			return d - d % m;
		}

		/// <summary>Round a number to 'digits' significant digits</summary>
		public static long RoundSD(long d, int significant_digits, MidpointRounding rounding = MidpointRounding.ToEven)
		{
			if (significant_digits < 0)
				throw new ArgumentOutOfRangeException(nameof(significant_digits), "value must be >= 0");

			// No significant digits is always zero
			if (d == 0 || significant_digits == 0)
				return 0;

			// long.MaxValue is 19 digits
			if (significant_digits > 19)
				return d;

			var pow = (int)Math.Floor(Math.Log10(d >= 0 ? d : ~d));
			var scale = (decimal)Math.Pow(10, significant_digits - pow - 1); // Double cannot represent long.MaxValue
			var result = scale != 0 ? (long)((long)Math.Round(d * scale, 0) / scale) : 0L;
			return result;
		}
		public static double RoundSD(double d, int significant_digits, MidpointRounding rounding = MidpointRounding.ToEven)
		{
			if (significant_digits < 0)
				throw new ArgumentOutOfRangeException(nameof(significant_digits), "value must be >= 0");

			// No significant digits is always zero
			if (d == 0 || significant_digits == 0)
				return 0;

			// double's mantissa is 17 digits
			if (significant_digits > 17)
				return d;

			var pow = (int)Math.Floor(Math.Log10(Math.Abs(d)));
			var scale = Math.Pow(10, significant_digits - pow - 1);
			var result = Math.Round(d * scale, 0) / scale;
			return result;
		}
		public static decimal RoundSD(decimal d, int significant_digits, MidpointRounding rounding = MidpointRounding.ToEven)
		{
			if (significant_digits < 0)
				throw new ArgumentOutOfRangeException(nameof(significant_digits), "value must be >= 0");

			// No significant digits is always zero
			if (d == 0 || significant_digits == 0)
				return 0;

			// decimals are 29 digits
			if (significant_digits > 29)
				return d;

			var pow = (int)Math.Floor(Math.Log10(Math.Abs((double)d)));
			var scale = (decimal)Math.Pow(10, significant_digits - pow - 1);
			var result = Math.Round(d * scale, 0) / scale;
			return result;
		}

		/// <summary>Truncate a number to 'digits' significant digits</summary>
		public static int TruncSD(int d, int significant_digits)
		{
			if (significant_digits < 0)
				throw new ArgumentOutOfRangeException(nameof(significant_digits), "value must be >= 0");

			// No significant digits is always zero
			if (d == 0 || significant_digits == 0)
				return 0;

			// long.MaxValue is 19 digits
			if (significant_digits > 19)
				return d;

			var pow = (int)Math.Floor(Math.Log10(d >= 0 ? d : ~d));
			var scale = (decimal)Math.Pow(10, significant_digits - pow - 1); // Double cannot represent long.MaxValue
			var result = scale != 0 ? (int)((long)(d * scale) / scale) : 0;
			return result;
		}
		public static long TruncSD(long d, int significant_digits)
		{
			if (significant_digits < 0)
				throw new ArgumentOutOfRangeException(nameof(significant_digits), "value must be >= 0");

			// No significant digits is always zero
			if (d == 0 || significant_digits == 0)
				return 0;

			// long.MaxValue is 19 digits
			if (significant_digits > 19)
				return d;

			var pow = (int)Math.Floor(Math.Log10(d >= 0 ? d : ~d));
			var scale = (decimal)Math.Pow(10, significant_digits - pow - 1); // Double cannot represent long.MaxValue
			var result = scale != 0 ? (long)((long)(d * scale) / scale) : 0L;
			return result;
		}
		public static double TruncSD(double d, int significant_digits)
		{
			if (significant_digits < 0)
				throw new ArgumentOutOfRangeException(nameof(significant_digits), "value must be >= 0");

			// No significant digits is always zero
			if (d == 0 || significant_digits == 0)
				return 0;

			// double's mantissa is 17 digits
			if (significant_digits > 17)
				return d;

			var pow = (int)Math.Floor(Math.Log10(Math.Abs(d)));
			var scale = Math.Pow(10, significant_digits - pow - 1);
			var result = Math.Truncate(d * scale) / scale;
			return result;
		}
		public static decimal TruncSD(decimal d, int significant_digits)
		{
			if (significant_digits < 0)
				throw new ArgumentOutOfRangeException(nameof(significant_digits), "value must be >= 0");

			// No significant digits is always zero
			if (d == 0 || significant_digits == 0)
				return 0;

			// decimals are 29 digits
			if (significant_digits > 29)
				return d;

			var pow = (int)Math.Floor(Math.Log10(Math.Abs((double)d)));
			var scale = (decimal)Math.Pow(10, significant_digits - pow - 1);
			var result = Math.Truncate(d * scale) / scale;
			return result;
		}

		/// <summary>Quantise a value to 'scale'. For best results, 'scale' should be a power of 2, i.e. 256, 1024, 2048, etc</summary>
		public static double Quantise(double x, long scale)
		{
			return (long)(x*scale) / (double)scale;
		}

		/// <summary>Quantise a value to units of 'quantum'</summary>
		public static double Quantise(double x, double quantum)
		{
			return (long)((x + 0.5*Sign(x)*quantum) / quantum) * quantum;
		}
		public static decimal Quantise(decimal x, decimal quantum)
		{
			return (long)((x + 0.5m*Sign(x)*quantum) / quantum) * quantum;
		}

		/// <summary>Convert a series of floating point values into a series of integers, preserving the remainders such that the sum of integers is within 1.0 of the sum of the floats</summary>
		public static IEnumerable<int> TruncWithRemainder(IEnumerable<double> floats)
		{
			var remainder = 0.0;
			foreach (var value in floats)
			{
				var fval = value + remainder;
				var ival = (int)(fval + Sign(fval) * 0.5);
				remainder = fval - ival;
				yield return ival;
			}
		}
		public static IEnumerable<int> TruncWithRemainder(IEnumerable<float> floats)
		{
			return TruncWithRemainder(floats.Select(x => (double)x));
		}

		/// <summary>Returns 1 if 'hi' is > 'lo' otherwise 0</summary>
		public static double Step(double lo, double hi)
		{
			return lo <= hi ? 0.0 : 1.0;
		}

		/// <summary>Returns the 'Hermite' interpolation (3t² - 2t³) between 'lo' and 'hi' for t=[0,1]</summary>
		public static double SmoothStep(double lo, double hi, double t)
		{
			if (lo == hi) return lo;
			t = Clamp((t - lo)/(hi - lo), 0.0, 1.0);
			return t*t*(3 - 2*t);
		}

		/// <summary>Returns a fifth-order 'Perlin' interpolation (6t^5 - 15t^4 + 10t^3) between 'lo' and 'hi' for t=[0,1]</summary>
		public static double SmoothStep2(double lo, double hi, double t)
		{
			if (lo == hi) return lo;
			t = Clamp((t - lo)/(hi - lo), 0.0, 1.0);
			return t*t*t*(t*(t*6 - 15) + 10);
		}

		/// <summary>
		/// Scale a value on the range [-inf,+inf] to within the range [-1,+1].
		/// 'n' is a horizontal scaling factor.
		/// If n = 1, [-1,+1] maps to [-0.5, +0.5]
		/// If n = 10, [-10,+10] maps to [-0.5, +0.5], etc</summary>
		public static double Sigmoid(double x, double n = 1.0)
		{
			return Math.Atan(x/n) / TauBy4;
		}
	}

	/// <summary>Formats for CodeString output</summary>
	public enum ECodeString
	{
		CSharp,
		Cpp,
		Python,
	}
}

#if PR_UNITTESTS
namespace Rylogic.UnitTests
{
	using System.Text;
	using System.Linq;
	using Maths;

	[TestFixture]
	public class TestMathsScalar
	{
		[Test]
		public void TestWrap()
		{
			// [0, 3)
			Assert.Equal(2, Math_.Wrap(-1, 0, 3));
			Assert.Equal(0, Math_.Wrap(+0, 0, 3));
			Assert.Equal(1, Math_.Wrap(+1, 0, 3));
			Assert.Equal(2, Math_.Wrap(+2, 0, 3));
			Assert.Equal(0, Math_.Wrap(+3, 0, 3));
			Assert.Equal(1, Math_.Wrap(+4, 0, 3));

			// [-2,+2]
			Assert.Equal(+2, Math_.Wrap(-3, -2, +3));
			Assert.Equal(-2, Math_.Wrap(-2, -2, +3));
			Assert.Equal(-1, Math_.Wrap(-1, -2, +3));
			Assert.Equal( 0, Math_.Wrap(+0, -2, +3));
			Assert.Equal(+1, Math_.Wrap(+1, -2, +3));
			Assert.Equal(+2, Math_.Wrap(+2, -2, +3));
			Assert.Equal(-2, Math_.Wrap(+3, -2, +3));

			// [+2,+5)
			Assert.Equal(4, Math_.Wrap(+1, +2, +5));
			Assert.Equal(2, Math_.Wrap(+2, +2, +5));
			Assert.Equal(3, Math_.Wrap(+3, +2, +5));
			Assert.Equal(4, Math_.Wrap(+4, +2, +5));
			Assert.Equal(2, Math_.Wrap(+5, +2, +5));
			Assert.Equal(3, Math_.Wrap(+6, +2, +5));

			// [0,1)
			Assert.Equal(0, Math_.Wrap(-3, 0, 1));
			Assert.Equal(0, Math_.Wrap(-2, 0, 1));
			Assert.Equal(0, Math_.Wrap(-1, 0, 1));
			Assert.Equal(0, Math_.Wrap(+0, 0, 1));
			Assert.Equal(0, Math_.Wrap(+1, 0, 1));
			Assert.Equal(0, Math_.Wrap(+2, 0, 1));
			Assert.Equal(0, Math_.Wrap(+3, 0, 1));

			// [-1,0)
			Assert.Equal(-1, Math_.Wrap(-3, -1, 0));
			Assert.Equal(-1, Math_.Wrap(-2, -1, 0));
			Assert.Equal(-1, Math_.Wrap(-1, -1, 0));
			Assert.Equal(-1, Math_.Wrap(+0, -1, 0));
			Assert.Equal(-1, Math_.Wrap(+1, -1, 0));
			Assert.Equal(-1, Math_.Wrap(+2, -1, 0));

			// Floating point wrap
			for (var f = -1.05; f <= 1.05; f += 0.1)
			{
				var x = Math_.Wrap(f, -0.2, 1.3);
				Assert.True(x >= -0.2 && x < 1.3);
			}
		}
		[Test]
		public void TestAverage()
		{
			var a = Math_.Average(1.0, 4.0, 2.0, 7.0, -3.0);
			Assert.Equal(a, 2.2);

			var m = Math_.Median(1.0, 4.0, 2.0, 7.0, -3.0);
			Assert.Equal(m, 2.0);

			m = Math_.Median(1.0, 4.0, 2.0, 7.0, -3.0, -1.0);
			Assert.Equal(m, 1.5);
		}
		[Test]
		public void TestLerp()
		{
			var a0 = new[] { 1.0, 10.0, 2.0, 5.0 };
			Assert.Equal(Math_.Lerp(-0.1, a0), 1.0);
			Assert.Equal(Math_.Lerp(0.0, a0), 1.0);
			Assert.Equal(Math_.Lerp(1.0, a0), 5.0);
			Assert.Equal(Math_.Lerp(1.1, a0), 5.0);
			Assert.Equal(Math_.Lerp(1 / 2.0, a0), 6.0);
			Assert.Equal(Math_.Lerp(1 / 3.0, a0), 10.0);
			Assert.Equal(Math_.Lerp(2 / 3.0, a0), 2.0);
			Assert.Equal(Math_.Lerp(1 / 6.0, a0), 5.5);
			Assert.Equal(Math_.Lerp(5 / 6.0, a0), 3.5);
			Assert.Equal(Math_.Lerp(new v4(0f), new v4(1f), 0.5), new v4(0.5f));
		}
		[Test]
		public void TestExponentiate()
		{
			Assert.True(Math_.IsIntegerPowerOfTen(+0));
			Assert.True(Math_.IsIntegerPowerOfTen(-0));
			Assert.True(Math_.IsIntegerPowerOfTen(-1));
			Assert.True(Math_.IsIntegerPowerOfTen(+4));
			Assert.True(Math_.IsIntegerPowerOfTen(+20));
			Assert.True(Math_.IsIntegerPowerOfTen(-40));
			Assert.True(Math_.IsIntegerPowerOfTen(+90000000000000L));
			Assert.True(Math_.IsIntegerPowerOfTen(-90000000000000L));

			Assert.False(Math_.IsIntegerPowerOfTen(+21));
			Assert.False(Math_.IsIntegerPowerOfTen(-42));
			Assert.False(Math_.IsIntegerPowerOfTen(+90000000000001L));
			Assert.False(Math_.IsIntegerPowerOfTen(-90000000000002L));

			Assert.Equal(new long[] { 80, 100, 20000, 400000, 7000000, 10000000 }, Math_.Exponentiate(17420180));
			Assert.Equal(new long[] { 200000 }, Math_.Exponentiate(200000));
			Assert.Equal(new long[] { -400 }, Math_.Exponentiate(-400));

			Assert.Equal(new long[] { 0xa0, 0xf00, 0x1000 }, Math_.Exponentiate(0x1fa0, 16));
			Assert.Equal(new long[] { 0b10, 0b1000, 0b10000, 0b1000000 }, Math_.Exponentiate(0x5a, 2));
		}
		[Test]
		public void TestTruncSD()
		{
			Assert.Equal(0, Math_.TruncSD(0, 1));
			Assert.Equal(0, Math_.TruncSD(1234, 0));
			Assert.Equal(+100000000L, Math_.TruncSD(+123456789L, 1));
			Assert.Equal(-100000000L, Math_.TruncSD(-123456789L, 1));
			Assert.Equal(+123000000L, Math_.TruncSD(+123456789L, 3));
			Assert.Equal(-123000000L, Math_.TruncSD(-123456789L, 3));
			Assert.Equal(+123456789L, Math_.TruncSD(+123456789L, 12));
			Assert.Equal(-123456789L, Math_.TruncSD(-123456789L, 12));
			Assert.Equal(+9223372036854775807L, Math_.TruncSD(long.MaxValue, 19));
			Assert.Equal(-9223372036854775808L, Math_.TruncSD(long.MinValue, 19));
			Assert.Equal(+9223372036854775807L, Math_.TruncSD(long.MaxValue, 20));
			Assert.Equal(-9223372036854775808L, Math_.TruncSD(long.MinValue, 20));

			Assert.Equal(0.0, Math_.TruncSD(0.0, 1));
			Assert.Equal(0.0, Math_.TruncSD(1234.5678, 0));
			Assert.Equal(+0.000100000, Math_.TruncSD(+0.000123456, 1));
			Assert.Equal(-0.000100000, Math_.TruncSD(-0.000123456, 1));
			Assert.Equal(+0.000123000, Math_.TruncSD(+0.000123456, 3));
			Assert.Equal(-0.000123000, Math_.TruncSD(-0.000123456, 3));
			Assert.Equal(+1234.500000, Math_.TruncSD(+1234.567800, 5));
			Assert.Equal(-1234.500000, Math_.TruncSD(-1234.567800, 5));
			Assert.Equal(+12000000.00, Math_.TruncSD(+12345678.90, 2));
			Assert.Equal(-12000000.00, Math_.TruncSD(-12345678.90, 2));
			Assert.Equal(+1.7976931348623157E+308, Math_.TruncSD(double.MaxValue, 17));
			Assert.Equal(-1.7976931348623157E+308, Math_.TruncSD(double.MinValue, 17));
			Assert.Equal(+1.7976931348623157E+308, Math_.TruncSD(double.MaxValue, 18));
			Assert.Equal(-1.7976931348623157E+308, Math_.TruncSD(double.MinValue, 18));

			Assert.Equal(0m, Math_.TruncSD(0m, 1));
			Assert.Equal(0m, Math_.TruncSD(1234.5678m, 0));
			Assert.Equal(+0.000100000m, Math_.TruncSD(+0.000123456m, 1));
			Assert.Equal(-0.000100000m, Math_.TruncSD(-0.000123456m, 1));
			Assert.Equal(+0.000123000m, Math_.TruncSD(+0.000123456m, 3));
			Assert.Equal(-0.000123000m, Math_.TruncSD(-0.000123456m, 3));
			Assert.Equal(+1234.500000m, Math_.TruncSD(+1234.567800m, 5));
			Assert.Equal(-1234.500000m, Math_.TruncSD(-1234.567800m, 5));
			Assert.Equal(+12000000.00m, Math_.TruncSD(+12345678.90m, 2));
			Assert.Equal(-12000000.00m, Math_.TruncSD(-12345678.90m, 2));
			Assert.Equal(+79228162514264337593543950335m, Math_.TruncSD(+79228162514264337593543950335m, 29));
			Assert.Equal(-79228162514264337593543950335m, Math_.TruncSD(-79228162514264337593543950335m, 29));
			Assert.Equal(+79228162514264337593543950335m, Math_.TruncSD(+79228162514264337593543950335m, 30));
			Assert.Equal(-79228162514264337593543950335m, Math_.TruncSD(-79228162514264337593543950335m, 30));
		}
		[Test]
		public void TestRoundSD()
		{
			Assert.Equal(0L, Math_.RoundSD(0L, 1));
			Assert.Equal(0L, Math_.RoundSD(1234L, 0));
			Assert.Equal(+990000000L, Math_.RoundSD(+987654321L, 2));
			Assert.Equal(-990000000L, Math_.RoundSD(-987654321L, 2));
			Assert.Equal(+987654321L, Math_.RoundSD(+987654321L, 12));
			Assert.Equal(-987654321L, Math_.RoundSD(-987654321L, 12));
			Assert.Equal(+9223372036854775807L, Math_.RoundSD(long.MaxValue, 19));
			Assert.Equal(-9223372036854775808L, Math_.RoundSD(long.MinValue, 19));
			Assert.Equal(+9223372036854775807L, Math_.RoundSD(long.MaxValue, 20));
			Assert.Equal(-9223372036854775808L, Math_.RoundSD(long.MinValue, 20));

			Assert.Equal(0.0, Math_.RoundSD(0.0, 1));
			Assert.Equal(0.0, Math_.RoundSD(1234.576, 0));
			Assert.Equal(+0.001000000, Math_.RoundSD(+0.000987654, 1));
			Assert.Equal(-0.001000000, Math_.RoundSD(-0.000987654, 1));
			Assert.Equal(+0.000988000, Math_.RoundSD(+0.000987654, 3));
			Assert.Equal(-0.000988000, Math_.RoundSD(-0.000987654, 3));
			Assert.Equal(+9876.500000, Math_.RoundSD(+9876.543200, 5));
			Assert.Equal(-9876.500000, Math_.RoundSD(-9876.543200, 5));
			Assert.Equal(-99000000.00, Math_.RoundSD(-98765432.10, 2));
			Assert.Equal(+99000000.00, Math_.RoundSD(+98765432.10, 2));
			Assert.Equal(+1.7976931348623157E+308, Math_.RoundSD(double.MaxValue, 17));
			Assert.Equal(-1.7976931348623157E+308, Math_.RoundSD(double.MinValue, 17));
			Assert.Equal(+1.7976931348623157E+308, Math_.RoundSD(double.MaxValue, 18));
			Assert.Equal(-1.7976931348623157E+308, Math_.RoundSD(double.MinValue, 18));

			Assert.Equal(0m, Math_.RoundSD(0m, 1));
			Assert.Equal(0m, Math_.RoundSD(1234.576m, 0));
			Assert.Equal(+0.001000000m, Math_.RoundSD(+0.000987654m, 1));
			Assert.Equal(-0.001000000m, Math_.RoundSD(-0.000987654m, 1));
			Assert.Equal(+0.000988000m, Math_.RoundSD(+0.000987654m, 3));
			Assert.Equal(-0.000988000m, Math_.RoundSD(-0.000987654m, 3));
			Assert.Equal(+9876.500000m, Math_.RoundSD(+9876.543200m, 5));
			Assert.Equal(-9876.500000m, Math_.RoundSD(-9876.543200m, 5));
			Assert.Equal(+99000000.00m, Math_.RoundSD(+98765432.10m, 2));
			Assert.Equal(-99000000.00m, Math_.RoundSD(-98765432.10m, 2));
			Assert.Equal(+79228162514264337593543950335m, Math_.RoundSD(+79228162514264337593543950335m, 29));
			Assert.Equal(-79228162514264337593543950335m, Math_.RoundSD(-79228162514264337593543950335m, 29));
			Assert.Equal(+79228162514264337593543950335m, Math_.RoundSD(+79228162514264337593543950335m, 30));
			Assert.Equal(-79228162514264337593543950335m, Math_.RoundSD(-79228162514264337593543950335m, 30));
		}
		[Test]
		public void TestAestheticValues()
		{
			Assert.Equal(0.0, Math_.AestheticValue(0.0, 0));

			Assert.Equal(5000.0, Math_.AestheticValue(4322.0,  0)); // 2500, 4322, 5000.. 5000 is nearest
			Assert.Equal(2500.0, Math_.AestheticValue(4322.0, -1)); // 2500, 4322, 5000.. 2500 is nearest less than
			Assert.Equal(5000.0, Math_.AestheticValue(4322.0, +1)); // 2500, 4322, 5000.. 5000 is nearest greater than

			Assert.Equal(0.02, Math_.AestheticValue(0.022, 0)); // 0.02, 0.022, 0.025.. 0.02 is nearest
			Assert.Equal(0.02, Math_.AestheticValue(0.019, 0)); // 0.01, 0.019, 0.02.. 0.02 is nearest

			Assert.Equal(-100.0, Math_.AestheticValue(-123.0,  0)); // -200, -123, -100.. -100 is nearest
			Assert.Equal(-100.0, Math_.AestheticValue(-123.0, +1)); // -200, -123, -100.. -100 is nearest greater than
			Assert.Equal(-200.0, Math_.AestheticValue(-123.0, -1)); // -200, -123, -100.. -200 is nearest less than

			Assert.Equal(-10000.0, Math_.AestheticValue(-9999.0, 0)); // -10000, -9999, -5000.. -10000 is nearest

			Assert.Equal(-10.0, Math_.AestheticValue(-10.0,  0));
			Assert.Equal(-10.0, Math_.AestheticValue(-10.0, +1));
			Assert.Equal(-10.0, Math_.AestheticValue(-10.0, -1));
		}
		[Test]
		public void TestSeries()
		{
			var series0 = Math_.ArithmeticSeries(1.0, 0.1).Take(5).ToArray();
			Assert.True(series0.SequenceEqual(new[] { 1.0, 1.1, 1.2, 1.3, 1.4 }, Math_.FEqlRelComparer(0.0001)));

			var series1 = Math_.GeometricSeries(0.1, 3.0).Take(5).ToArray();
			Assert.True(series1.SequenceEqual(new[] { 0.1, 0.3, 0.9, 2.7, 8.1 }, Math_.FEqlRelComparer(0.0001)));

			var sum0 = Math_.ArithmeticSeriesSum(1.0, 0.1, 5);
			Assert.True(Math_.FEql(6.0, sum0));

			var sum1 = Math_.GeometricSeriesSum(0.1, 3.0, 5);
			Assert.True(Math_.FEql(12.1, sum1));
		}
	}
}
#endif