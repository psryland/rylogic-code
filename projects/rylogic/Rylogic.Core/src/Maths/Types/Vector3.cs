//***************************************************
// Vector2
//  Copyright (c) Rylogic Ltd 2008
//***************************************************

using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Runtime.InteropServices;
using System.Numerics;
using Rylogic.Extn;

namespace Rylogic.Maths
{
	[Serializable]
	[StructLayout(LayoutKind.Explicit)]
	[DebuggerDisplay("{Description,nq}")]
	public struct v3
	{
		[FieldOffset(0)] public float x;
		[FieldOffset(4)] public float y;
		[FieldOffset(8)] public float z;
		[FieldOffset(0)] public v2 xy;

		// Constructors
		public v3(float x_)
			: this(x_, x_, x_)
		{ }
		public v3(float x_, float y_, float z_)
			: this()
		{
			x = x_;
			y = y_;
			z = z_;
		}
		public v3(v2 xy_, float z_)
			: this()
		{
			xy = xy_;
			z = z_;
		}
		public v3(float[] arr)
			: this()
		{
			x = arr[0];
			y = arr[1];
			z = arr[2];
		}

		/// <summary>Get/Set components by index</summary>
		public float this[int i]
		{
			get
			{
				switch (i) {
				case 0: return x;
				case 1: return y;
				case 2: return z;
				}
				throw new ArgumentException("index out of range", "i");
			}
			set
			{
				switch (i) {
				case 0: x = value; return;
				case 1: y = value; return;
				case 2: z = value; return;
				}
				throw new ArgumentException("index out of range", "i");
			}
		}
		public float this[uint i]
		{
			get => this[(int)i];
			set => this[(int)i] = value;
		}

		/// <summary>Integer cast accessors</summary>
		public int xi => (int)x;
		public int yi => (int)y;
		public int zi => (int)z;

		/// <summary>Length</summary>
		public float LengthSq => x * x + y * y + z * z;
		public float Length => (float)Math.Sqrt(LengthSq);

		/// <summary>Explicit conversion to an array. Note not implicit because it gets called when converting v3 to an object type. e.g. v3? x = v3.TryParse4("", out v) ? v : null. </summary>
		public static explicit operator v3(float[] a)
		{
			return new(a[0], a[1], a[2]);
		}
		public static explicit operator float[](v3 p)
		{
			return new float[] { p.x, p.y, p.z };
		}
		public static explicit operator v3(double[] a)
		{
			return new((float)a[0], (float)a[1], (float)a[2]);
		}
		public static explicit operator double[](v3 p)
		{
			return new double[] { p.x, p.y, p.z };
		}

		/// <summary>Implicit conversion to System.Numerics</summary>
		public static implicit operator v3(Vector3 v)
		{
			return new v3(v.X, v.Y, v.Z);
		}
		public static implicit operator Vector3(v3 v)
		{
			return new Vector3(v.x, v.y, v.z);
		}

		public v4 w0 => new(x, y, z, 0);
		public v4 w1 => new(x, y, z, 1);

		#region Statics
		public readonly static v3 Zero = new(0f, 0f, 0f);
		public readonly static v3 XAxis = new(1f, 0f, 0f);
		public readonly static v3 YAxis = new(0f, 1f, 0f);
		public readonly static v3 ZAxis = new(0f, 0f, 1f);
		public readonly static v3 Origin = new(0f, 0f, 0f);
		public readonly static v3 One = new(1f, 1f, 1f);
		public readonly static v3 NaN = new(float.NaN, float.NaN, float.NaN);
		public readonly static v3 MinValue = new(float.MinValue, float.MinValue, float.MinValue);
		public readonly static v3 MaxValue = new(float.MaxValue, float.MaxValue, float.MaxValue);
		#endregion

		#region Operators
		public static v3 operator +(v3 vec)
		{
			return vec;
		}
		public static v3 operator -(v3 vec)
		{
			return new(-vec.x, -vec.y, -vec.z);
		}
		public static v3 operator +(v3 lhs, v3 rhs)
		{
			return new(lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z);
		}
		public static v3 operator -(v3 lhs, v3 rhs)
		{
			return new(lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z);
		}
		public static v3 operator *(v3 lhs, float rhs)
		{
			return new(lhs.x * rhs, lhs.y * rhs, lhs.z * rhs);
		}
		public static v3 operator *(v3 lhs, double rhs)
		{
			return lhs * (float)rhs;
		}
		public static v3 operator *(float lhs, v3 rhs)
		{
			return new(lhs * rhs.x, lhs * rhs.y, lhs * rhs.z);
		}
		public static v3 operator *(double lhs, v3 rhs)
		{
			return (float)lhs * rhs;
		}
		public static v3 operator *(v3 lhs, v3 rhs)
		{
			return new(lhs.x * rhs.x, lhs.y * rhs.y, lhs.z * rhs.z);
		}
		public static v3 operator /(v3 lhs, float rhs)
		{
			return new(lhs.x / rhs, lhs.y / rhs, lhs.z / rhs);
		}
		public static v3 operator /(v3 lhs, double rhs)
		{
			return lhs / (float)rhs;
		}
		public static v3 operator /(v3 lhs, v3 rhs)
		{
			return new(lhs.x / rhs.x, lhs.y / rhs.y, lhs.z / rhs.z);
		}
		public static v3 operator %(v3 lhs, float rhs)
		{
			return new(lhs.x % rhs, lhs.y % rhs, lhs.z % rhs);
		}
		public static v3 operator %(v3 lhs, double rhs)
		{
			return lhs % (float)rhs;
		}
		public static v3 operator %(v3 lhs, v3 rhs)
		{
			return new(lhs.x % rhs.x, lhs.y % rhs.y, lhs.z % rhs.z);
		}
		#endregion

		#region Equals
		public static bool operator ==(v3 lhs, v3 rhs)
		{
			return lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z;
		}
		public static bool operator !=(v3 lhs, v3 rhs)
		{
			return !(lhs == rhs);
		}
		public override bool Equals(object? o)
		{
			return o is v3 v && v == this;
		}
		public override int GetHashCode()
		{
			return new { x, y, z }.GetHashCode();
		}
		#endregion

		#region ToString
		public override string ToString() => $"{x} {y} {z}";
		public string ToString(string format) => $"{x.ToString(format)} {y.ToString(format)} {z.ToString(format)}";
		public string ToCodeString(ECodeString fmt = ECodeString.CSharp)
		{
			return fmt switch
			{
				ECodeString.CSharp => $"{x:+0.0000000;-0.0000000;+0.0000000}f, {y:+0.0000000;-0.0000000;+0.0000000}f, {z:+0.0000000;-0.0000000;+0.0000000}f",
				ECodeString.Cpp => $"{x:+0.0000000;-0.0000000;+0.0000000}f, {y:+0.0000000;-0.0000000;+0.0000000}f, {z:+0.0000000;-0.0000000;+0.0000000}f",
				ECodeString.Python => $"{x:+0.0000000;-0.0000000;+0.0000000}, {y:+0.0000000;-0.0000000;+0.0000000}, {z:+0.0000000;-0.0000000;+0.0000000}",
				_ => ToString(),
			};
		}
		#endregion

		#region Parse
		public static v3 Parse(string s)
		{
			if (s == null)
				throw new ArgumentNullException(nameof(s), "v3.Parse() string argument was null");

			var values = s.Split([' ', ',', '\t'], 3);
			if (values.Length != 3)
				throw new FormatException("v3.Parse() string argument does not represent a 3 component vector");

			return new(
				float.Parse(values[0]),
				float.Parse(values[1]),
				float.Parse(values[2]));
		}
		public static bool TryParse(string s, out v3 vec)
		{
			vec = Zero;
			if (s == null)
				return false;

			var values = s.Split([' ', ',', '\t'], 3);
			return
				values.Length == 3 &&
				float.TryParse(values[0], out vec.x) &&
				float.TryParse(values[1], out vec.y) &&
				float.TryParse(values[2], out vec.z);
		}
		public static v3? TryParse(string s)
		{
			return TryParse(s, out var vec) ? vec : null;
		}
		#endregion

		#region Random

		/// <summary>Return a random vector with components within the interval [min,max]</summary>
		public static v3 Random3(float min, float max, Random r)
		{
			return new(r.Float(min, max), r.Float(min, max), r.Float(min, max));
		}

		/// <summary>Return a random vector within a 3D sphere of radius 'rad' (Note: *not* on a sphere)</summary>
		public static v3 Random3(float rad, Random r)
		{
			var rad_sq = rad * rad;
			v3 v; for (; (v = Random3(-rad, rad, r)).LengthSq > rad_sq;) { }
			return v;
		}

		/// <summary>Return a random vector with components within the intervals given by each component of min and max</summary>
		public static v3 Random3(v3 min, v3 max, Random r)
		{
			return new(r.Float(min.x, max.x), r.Float(min.y, max.y), r.Float(min.z, max.z));
		}

		/// <summary>Return a random vector with components within the intervals given by each component of min and max</summary>
		public static v3 Random3(v3 centre, float radius, Random r)
		{
			return Random3(radius, r) + centre;
		}

		/// <summary>Return a random vector on the unit 3D sphere</summary>
		public static v3 Random3N(Random r)
		{
			v3 v; for (; Math_.FEql(v = Random3(1.0f, r), Zero);) { }
			return Math_.Normalise(v);
		}

		/// <summary>Return a random vector with components within the interval [min,max]</summary>
		public static v3 Random2(float min, float max, float z, Random r)
		{
			return new(r.Float(min, max), r.Float(min, max), z);
		}

		/// <summary>Return a random vector with components within the intervals given by each component of min and max</summary>
		public static v3 Random2(v3 min, v3 max, float z, Random r)
		{
			return new(r.Float(min.x, max.x), r.Float(min.y, max.y), z);
		}

		/// <summary>Return a random vector on the unit 2D sphere</summary>
		public static v3 Random2(float rad, float z, Random r)
		{
			var rad_sq = rad * rad;
			v3 v; for (; (v = Random2(-rad, rad, 0, r)).LengthSq > rad_sq;) { }
			v.z = z;
			return v;
		}

		/// <summary>Return a random vector on the unit 2D sphere</summary>
		public static v3 Random2N(float z, Random r)
		{
			v3 v; for (; Math_.FEql(v = Random2(1.0f, 0, r), Zero);) { }
			v = Math_.Normalise(v);
			v.z = z;
			return v;
		}

		#endregion

		/// <summary></summary>
		public string Description => $"{x}  {y}  {z}  //Len({Length})";
	}

	public static partial class Math_
	{
		/// <summary>Approximate equal</summary>
		public static bool FEqlAbsolute(v3 a, v3 b, float tol)
		{
			return
				FEqlAbsolute(a.x, b.x, tol) &&
				FEqlAbsolute(a.y, b.y, tol) &&
				FEqlAbsolute(a.z, b.z, tol);
		}
		public static bool FEqlRelative(v3 a, v3 b, float tol)
		{
			var max_a = MaxElement(Abs(a));
			var max_b = MaxElement(Abs(b));
			if (max_b == 0) return max_a < tol;
			if (max_a == 0) return max_b < tol;
			var abs_max_element = Max(max_a, max_b);
			return FEqlAbsolute(a, b, tol * abs_max_element);
		}
		public static bool FEql(v3 a, v3 b)
		{
			return FEqlRelative(a, b, TinyF);
		}

		/// <summary>Component absolute value</summary>
		public static v3 Abs(v3 vec)
		{
			return new(
				Math.Abs(vec.x),
				Math.Abs(vec.y),
				Math.Abs(vec.z));
		}

		/// <summary>Return true if all components of 'vec' are finite</summary>
		public static bool IsFinite(v3 vec)
		{
			return IsFinite(vec.x) && IsFinite(vec.y) && IsFinite(vec.z);
		}

		/// <summary>Return true if any components of 'vec' are NaN</summary>
		public static bool IsNaN(v3 vec)
		{
			return IsNaN(vec.x) || IsNaN(vec.y) || IsNaN(vec.z);
		}

		/// <summary>Return 'a/b', or 'def' if 'b' is zero</summary>
		public static v3 Div(v3 a, v3 b, v3 def)
		{
			return b != v3.Zero ? a / b : def;
		}

		/// <summary>Return a vector containing the minimum components</summary>
		public static v3 Min(v3 lhs, v3 rhs)
		{
			return new(
				Math.Min(lhs.x, rhs.x),
				Math.Min(lhs.y, rhs.y),
				Math.Min(lhs.z, rhs.z));
		}
		public static v3 Min(v3 x, params v3[] vecs)
		{
			foreach (var v in vecs)
				x = Min(x, v);
			return x;
		}

		/// <summary>Return a vector containing the maximum components</summary>
		public static v3 Max(v3 lhs, v3 rhs)
		{
			return new(
				Math.Max(lhs.x, rhs.x),
				Math.Max(lhs.y, rhs.y),
				Math.Max(lhs.z, rhs.z));
		}
		public static v3 Max(v3 x, params v3[] vecs)
		{
			foreach (var v in vecs)
				x = Max(x, v);
			return x;
		}

		/// <summary>Clamp the components of 'vec' within the ranges of 'min' and 'max'</summary>
		public static v3 Clamp(v3 vec, v3 min, v3 max)
		{
			return new(
				Clamp(vec.x, min.x, max.x),
				Clamp(vec.y, min.y, max.y),
				Clamp(vec.z, min.z, max.z));
		}

		/// <summary>Return the maximum element value in 'v'</summary>
		public static float MaxElement(v3 v)
		{
			var i = v.x >= v.y ? v.x : v.y;
			return i >= v.z ? i : v.z;
		}

		/// <summary>Return the minimum element value in 'v'</summary>
		public static float MinElement(v3 v)
		{
			var i = v.x <= v.y ? v.x : v.y;
			return i <= v.z ? i : v.z;
		}

		/// <summary>Return the index of the maximum element in 'v'</summary>
		public static int MaxElementIndex(v3 v)
		{
			int i = v.y >= v.z ? 1 : 2;
			return v.x >= v[i] ? 0 : i;
		}

		/// <summary>Return the index of the minimum element in 'v'</summary>
		public static int MinElementIndex(v3 v)
		{
			int i = v.y <= v.z ? 1 : 2;
			return v.x <= v[i] ? 0 : i;
		}

		/// <summary>Return the component-wise square root of a vector</summary>
		public static v3 Sqrt(v3 a)
		{
			return new(
				(float)Math.Sqrt(a.x),
				(float)Math.Sqrt(a.y),
				(float)Math.Sqrt(a.z));
		}

		/// <summary>Normalise 'vec' by the length of the XYZ components</summary>
		public static v3 Normalise(v3 vec)
		{
			return vec / vec.Length;
		}
		public static v3 Normalise(ref v3 vec)
		{
			return vec /= vec.Length;
		}

		/// <summary>Normalise 'vec' by the length of the XYZ components or return 'def' if zero</summary>
		public static v3 Normalise(v3 vec, v3 def)
		{
			if (vec == v3.Zero) return def;
			var norm = Normalise(vec);
			return norm != v3.Zero ? norm : def;
		}

		/// <summary>Dot product of XYZ components</summary>
		public static float Dot(v3 lhs, v3 rhs)
		{
			return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
		}

		/// <summary>Cross product of XYZ components</summary>
		public static v3 Cross(v3 lhs, v3 rhs)
		{
			return new(lhs.y * rhs.z - lhs.z * rhs.y, lhs.z * rhs.x - lhs.x * rhs.z, lhs.x * rhs.y - lhs.y * rhs.x);
		}

		/// <summary>Triple product of XYZ components</summary>
		public static float Triple(v3 a, v3 b, v3 c)
		{
			return Dot(a, Cross(b, c));
		}

		/// <summary>True if 'lhs' and 'rhs' are parallel</summary>
		public static bool Parallel(v3 lhs, v3 rhs)
		{
			return FEql(Cross(lhs, rhs).LengthSq, 0);
		}

		/// <summary>Linearly interpolate between two vectors</summary>
		public static v3 Lerp(v3 lhs, v3 rhs, float frac)
		{
			return lhs * (1f - frac) + rhs * (frac);
		}

		/// <summary>Returns a vector guaranteed to not be parallel to 'vec'</summary>
		public static v3 CreateNotParallelTo(v3 vec)
		{
			bool x_aligned = Abs(vec.x) > Abs(vec.y) && Abs(vec.x) > Abs(vec.z);
			return new(SignF(!x_aligned), 0, SignF(x_aligned));
		}

		/// <summary>Returns a vector perpendicular to 'vec'</summary>
		public static v3 Perpendicular(v3 vec)
		{
			Debug.Assert(!FEql(vec, v3.Zero), "Cannot make a perpendicular to a zero vector");
			var v = Cross(vec, CreateNotParallelTo(vec));
			v *= vec.Length / v.Length;
			return v;
		}

		/// <summary>Returns a vector perpendicular to 'vec' favouring 'previous' as the preferred perpendicular</summary>
		public static v3 Perpendicular(v3 vec, v3 previous)
		{
			Debug.Assert(!FEql(vec, v3.Zero), "Cannot make a perpendicular to a zero vector");

			// If 'previous' is parallel to 'vec', choose a new perpendicular (includes previous == zero)
			if (Parallel(vec, previous))
				return Perpendicular(vec);

			// If 'previous' is still perpendicular, keep it
			if (FEql(Dot(vec, previous), 0))
				return previous;

			// Otherwise, make a perpendicular that is close to 'previous'
			var v = Cross(Cross(vec, previous), vec);
			v *= (float)Math.Sqrt(vec.LengthSq / v.LengthSq);
			return v;
		}

		/// <summary>Return the cosine of the angle between two vectors</summary>
		public static float CosAngle(v3 lhs, v3 rhs)
		{
			Debug.Assert(lhs.LengthSq != 0 && rhs.LengthSq != 0, "CosAngle undefined for zero vectors");
			return Clamp(Dot(lhs, rhs) / (float)Math.Sqrt(lhs.LengthSq * rhs.LengthSq), -1f, 1f);
		}

		/// <summary>Return the angle between two vectors</summary>
		public static float Angle(v3 lhs, v3 rhs)
		{
			return (float)Math.Acos(CosAngle(lhs, rhs));
		}

		/// <summary>Return the average of a collection of vectors</summary>
		public static v3 Average(IEnumerable<v3> vecs)
		{
			var acc = v3.Zero;
			var count = 0;
			foreach (var v in vecs)
			{
				acc += v;
				++count;
			}
			if (count == 0) throw new Exception("Cannot average zero items");
			return acc / count;
		}
	}
}

#if PR_UNITTESTS
namespace Rylogic.UnitTests
{
	using Maths;

	[TestFixture] public class UnitTestv3
	{
		[Test] public void Basic()
		{
			var a = new v3(1,2,-3);
			Assert.True(a.x == +1);
			Assert.True(a.y == +2);
			Assert.True(a.z == -3);
		}
		[Test] public void MinMax()
		{
			var a = new v3(3,-1,2);
			var b = new v3(-2,-1,4);
			Assert.True(Math_.Max(a,b) == new v3(3,-1,4));
			Assert.True(Math_.Min(a,b) == new v3(-2,-1,2));
		}
		[Test] public void Length()
		{
			var a = new v3(3,-1,2);
			Assert.True(Math_.FEql(a.LengthSq, a.x*a.x + a.y*a.y + a.z*a.z));
			Assert.True(Math_.FEql(a.Length, (float)Math.Sqrt(a.LengthSq)));
			Assert.True(Math_.FEql(a.xy.LengthSq, a.x*a.x + a.y*a.y));
			Assert.True(Math_.FEql(a.xy.Length  , (float)Math.Sqrt(a.xy.LengthSq)));
		}
		[Test] public void Normals()
		{
			var a = new v3(3,-1,2);
			var b = Math_.Normalise(a);
			Assert.True(Math_.FEql(a.Length, 1.0f) == false);
			Assert.True(Math_.FEql(b.Length, 1.0f) == true);
		}
		[Test] public void DotProduct()
		{
			var a = new v3(-2,  4,  2);
			var b = new v3( 3, -5,  2);
			Assert.True(Math_.FEql(Math_.Dot(a,b), -22f));
		}
	}
}
#endif