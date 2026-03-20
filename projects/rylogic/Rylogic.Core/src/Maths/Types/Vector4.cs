//***************************************************
// Vector4
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
	public struct v4
	{
		[FieldOffset(0)] public float x;
		[FieldOffset(4)] public float y;
		[FieldOffset(8)] public float z;
		[FieldOffset(12)] public float w;
		[FieldOffset(0)] public v2 xy;
		[FieldOffset(4)] public v2 yz;
		[FieldOffset(8)] public v2 zw;
		[FieldOffset(0)] public v3 xyz;

		// Constructors
		public v4(float x_)
			: this(x_, x_, x_, x_)
		{ }
		public v4(float x_, float y_, float z_, float w_)
			: this()
		{
			x = x_;
			y = y_;
			z = z_;
			w = w_;
		}
		public v4(v2 xy_, float z_, float w_)
			: this()
		{
			xy = xy_;
			z = z_;
			w = w_;
		}
		public v4(v2 xy_, v2 zw_)
			: this()
		{
			xy = xy_;
			zw = zw_;
		}
		public v4(v3 xyz_, float w_)
			: this()
		{
			xyz = xyz_;
			w = w_;
		}
		public v4(float[] arr, int start = 0)
			: this()
		{
			x = arr[start + 0];
			y = arr[start + 1];
			z = arr[start + 2];
			w = arr[start + 3];
		}
		public v4(float[] arr, float w_, int start = 0)
			: this()
		{
			x = arr[start + 0];
			y = arr[start + 1];
			z = arr[start + 2];
			w = w_;
		}

		// Construct a normalised vector
		public static v4 Normal(float x_, float y_, float z_, float w_)
		{
			return Math_.Normalise(new v4(x_, y_, z_, w_));
		}

		/// <summary>Get/Set components by index</summary>
		public float this[int i]
		{
			get
			{
				switch (i)
				{
				case 0: return x;
				case 1: return y;
				case 2: return z;
				case 3: return w;
				}
				throw new ArgumentException("index out of range", "i");
			}
			set
			{
				switch (i)
				{
				case 0: x = value; return;
				case 1: y = value; return;
				case 2: z = value; return;
				case 3: w = value; return;
				}
				throw new ArgumentException("index out of range", "i");
			}
		}
		public float this[uint i]
		{
			get => this[(int)i];
			set => this[(int)i] = value;
		}

		/// <summary>To flat array</summary>
		public float[] ToArray()
		{
			return new[] { x, y, z, w };
		}

		/// <summary>Integer cast accessors</summary>
		public int xi => (int)x;
		public int yi => (int)y;
		public int zi => (int)z;
		public int wi => (int)w;

		/// <summary>Length</summary>
		public float LengthSq => x * x + y * y + z * z + w * w;
		public float Length => (float)Math.Sqrt(LengthSq);

		/// <summary>Explicit conversion to an array. Note not implicit because it gets called when converting v4 to an object type. e.g. v4? x = v4.TryParse4("", out v) ? v : null. </summary>
		public static explicit operator v4(float[] a)
		{
			return new(a[0], a[1], a[2], a[3]);
		}
		public static explicit operator float[](v4 p)
		{
			return new float[] { p.x, p.y, p.z, p.w };
		}
		public static explicit operator v4(double[] a)
		{
			return new((float)a[0], (float)a[1], (float)a[2], (float)a[3]);
		}
		public static explicit operator double[](v4 p)
		{
			return new double[] { p.x, p.y, p.z, p.w };
		}

		/// <summary>Implicit conversion to System.Numerics</summary>
		public static implicit operator v4(Vector4 v)
		{
			return new v4(v.X, v.Y, v.Z, v.W);
		}
		public static implicit operator Vector4(v4 v)
		{
			return new Vector4(v.x, v.y, v.z, v.w);
		}

		public v4 w0 => new(x, y, z, 0);
		public v4 w1 => new(x, y, z, 1);

		#region Statics
		public readonly static v4 Zero = new(0f, 0f, 0f, 0f);
		public readonly static v4 XAxis = new(1f, 0f, 0f, 0f);
		public readonly static v4 YAxis = new(0f, 1f, 0f, 0f);
		public readonly static v4 ZAxis = new(0f, 0f, 1f, 0f);
		public readonly static v4 WAxis = new(0f, 0f, 0f, 1f);
		public readonly static v4 Origin = new(0f, 0f, 0f, 1f);
		public readonly static v4 One = new(1f, 1f, 1f, 1f);
		public readonly static v4 NaN = new(float.NaN, float.NaN, float.NaN, float.NaN);
		public readonly static v4 MinValue = new(float.MinValue, float.MinValue, float.MinValue, float.MinValue);
		public readonly static v4 MaxValue = new(float.MaxValue, float.MaxValue, float.MaxValue, float.MaxValue);
		public readonly static v4 TinyF = new(Math_.TinyF, Math_.TinyF, Math_.TinyF, Math_.TinyF);
		#endregion

		#region Operators
		public static v4 operator +(v4 vec)
		{
			return vec;
		}
		public static v4 operator -(v4 vec)
		{
			return new(-vec.x, -vec.y, -vec.z, -vec.w);
		}
		public static v4 operator +(v4 lhs, v4 rhs)
		{
			return new(lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z, lhs.w + rhs.w);
		}
		public static v4 operator -(v4 lhs, v4 rhs)
		{
			return new(lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z, lhs.w - rhs.w);
		}
		public static v4 operator *(v4 lhs, float rhs)
		{
			return new(lhs.x * rhs, lhs.y * rhs, lhs.z * rhs, lhs.w * rhs);
		}
		public static v4 operator *(v4 lhs, double rhs)
		{
			return lhs * (float)rhs;
		}
		public static v4 operator *(float lhs, v4 rhs)
		{
			return new(lhs * rhs.x, lhs * rhs.y, lhs * rhs.z, lhs * rhs.w);
		}
		public static v4 operator *(double lhs, v4 rhs)
		{
			return (float)lhs * rhs;
		}
		public static v4 operator *(v4 lhs, v4 rhs)
		{
			// Not really sound mathematically, but too useful not to have
			return new(lhs.x * rhs.x, lhs.y * rhs.y, lhs.z * rhs.z, lhs.w * rhs.w);
		}
		public static v4 operator /(v4 lhs, float rhs)
		{
			return new(lhs.x / rhs, lhs.y / rhs, lhs.z / rhs, lhs.w / rhs);
		}
		public static v4 operator /(v4 lhs, double rhs)
		{
			return lhs / (float)rhs;
		}
		public static v4 operator /(float lhs, v4 rhs)
		{
			return new(lhs / rhs.x, lhs / rhs.y, lhs / rhs.z, lhs / rhs.w);
		}
		public static v4 operator /(double lhs, v4 rhs)
		{
			return (float)lhs / rhs;
		}
		public static v4 operator /(v4 lhs, v4 rhs)
		{
			return new(lhs.x / rhs.x, lhs.y / rhs.y, lhs.z / rhs.z, lhs.w / rhs.w);
		}
		public static v4 operator %(v4 lhs, float rhs)
		{
			return new(lhs.x % rhs, lhs.y % rhs, lhs.z % rhs, lhs.w % rhs);
		}
		public static v4 operator %(v4 lhs, double rhs)
		{
			return lhs % (float)rhs;
		}
		public static v4 operator %(v4 lhs, v4 rhs)
		{
			return new(lhs.x % rhs.x, lhs.y % rhs.y, lhs.z % rhs.z, lhs.w % rhs.w);
		}
		#endregion

		#region Equals
		public static bool operator ==(v4 lhs, v4 rhs)
		{
			return lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z && lhs.w == rhs.w;
		}
		public static bool operator !=(v4 lhs, v4 rhs)
		{
			return !(lhs == rhs);
		}
		public override bool Equals(object? o)
		{
			return o is v4 v && v == this;
		}
		public override int GetHashCode()
		{
			return new { x, y, z, w }.GetHashCode();
		}
		#endregion

		#region ToString
		public override string ToString() => $"{x} {y} {z} {w}";
		public string ToString(string format) => $"{x.ToString(format)} {y.ToString(format)} {z.ToString(format)} {w.ToString(format)}";
		public string ToCodeString(ECodeString fmt = ECodeString.CSharp)
		{
			return fmt switch
			{
				ECodeString.CSharp => $"{x:+0.0000000;-0.0000000;+0.0000000}f, {y:+0.0000000;-0.0000000;+0.0000000}f, {z:+0.0000000;-0.0000000;+0.0000000}f, {w:+0.0000000;-0.0000000;+0.0000000}f",
				ECodeString.Cpp => $"{x:+0.0000000;-0.0000000;+0.0000000}f, {y:+0.0000000;-0.0000000;+0.0000000}f, {z:+0.0000000;-0.0000000;+0.0000000}f, {w:+0.0000000;-0.0000000;+0.0000000}f",
				ECodeString.Python => $"{x:+0.0000000;-0.0000000;+0.0000000}, {y:+0.0000000;-0.0000000;+0.0000000}, {z:+0.0000000;-0.0000000;+0.0000000}f, {w:+0.0000000;-0.0000000;+0.0000000}",
				_ => ToString(),
			};
		}
		#endregion

		#region Parse
		public static v4 Parse(string s, float w)
		{
			if (s == null)
				throw new ArgumentNullException(nameof(s), "v4.Parse() string argument was null");

			var values = s.Split([' ', ',', '\t'], 3);
			if (values.Length != 3)
				throw new FormatException("v4.Parse() string argument does not represent a 3 component vector");

			return new(
				float.Parse(values[0]),
				float.Parse(values[1]),
				float.Parse(values[2]),
				w);
		}
		public static v4 Parse(string s)
		{
			if (s == null)
				throw new ArgumentNullException(nameof(s), "v4.Parse() string argument was null");

			var values = s.Split([' ', ',', '\t'], 4);
			if (values.Length != 4)
				throw new FormatException("v4.Parse() string argument does not represent a 4 component vector");

			return new(
				float.Parse(values[0]),
				float.Parse(values[1]),
				float.Parse(values[2]),
				float.Parse(values[3]));
		}
		public static bool TryParse(string s, out v4 vec, float w)
		{
			vec = Zero;
			if (s == null)
				return false;
			
			vec.w = w;
			var values = s.Split([' ', ',', '\t'], 3);
			return
				values.Length == 3 &&
				float.TryParse(values[0], out vec.x) &&
				float.TryParse(values[1], out vec.y) &&
				float.TryParse(values[2], out vec.z);
		}
		public static v4? TryParse3(string s, float w)
		{
			return TryParse(s, out var vec, w) ? vec : null;
		}
		public static bool TryParse(string s, out v4 vec)
		{
			vec = Zero;
			if (s == null)
				return false;

			var values = s.Split([' ', ',', '\t'], 4);
			return
				values.Length == 4 &&
				float.TryParse(values[0], out vec.x) &&
				float.TryParse(values[1], out vec.y) &&
				float.TryParse(values[2], out vec.z) &&
				float.TryParse(values[3], out vec.w);
		}
		public static v4? TryParse(string s)
		{
			return TryParse(s, out var vec) ? vec : null;
		}
		#endregion

		#region Random

		/// <summary>Return a random vector with components within the interval [min,max]</summary>
		public static v4 Random4(float min, float max, Random r)
		{
			return new(
				r.Float(min, max),
				r.Float(min, max),
				r.Float(min, max),
				r.Float(min, max));
		}

		/// <summary>Return a random vector within a 4D sphere of radius 'rad' (Note: *not* on the sphere)</summary>
		public static v4 Random4(float rad, Random r)
		{
			var rad_sq = rad * rad;
			v4 v; for (; (v = Random4(-rad, rad, r)).LengthSq > rad_sq;) { }
			return v;
		}

		/// <summary>Return a random vector with components within the intervals given by each component of min and max</summary>
		public static v4 Random4(v4 min, v4 max, Random r)
		{
			return new(
				r.Float(min.x, max.x),
				r.Float(min.y, max.y),
				r.Float(min.z, max.z),
				r.Float(min.w, max.w));
		}

		/// <summary>Create a random vector centred on 'centre' with radius 'radius'.</summary>
		public static v4 Random4(v4 centre, float radius, Random r)
		{
			return Random4(radius, r) + centre;
		}

		/// <summary>Return a random vector on the unit 4D sphere</summary>
		public static v4 Random4N(Random r)
		{
			v4 v; for (; Math_.FEql(v = Random4(1f, r), Zero);) { }
			return Math_.Normalise(v);
		}

		/// <summary>Return a random vector with components within the interval [min,max]</summary>
		public static v4 Random3(float min, float max, float w, Random r)
		{
			return new(
				r.Float(min, max),
				r.Float(min, max),
				r.Float(min, max),
				w);
		}

		/// <summary>Return a random vector within a 3D sphere of radius 'rad' (Note: *not* on a sphere)</summary>
		public static v4 Random3(float rad, float w, Random r)
		{
			var rad_sq = rad * rad;
			v4 v; for (; (v = Random3(-rad, rad, 0, r)).LengthSq > rad_sq;) { }
			v.w = w;
			return v;
		}

		/// <summary>Return a random vector with components within the intervals given by each component of min and max</summary>
		public static v4 Random3(v4 min, v4 max, float w, Random r)
		{
			return new(
				r.Float(min.x, max.x),
				r.Float(min.y, max.y),
				r.Float(min.z, max.z),
				w);
		}

		// Create a random vector centred on 'centre' with radius 'radius'.
		public static v4 Random3(v4 centre, float radius, float w, Random r)
		{
			return new(v3.Random3(centre.xyz, radius, r), w);
		}

		/// <summary>Return a random vector on the unit 3D sphere</summary>
		public static v4 Random3N(float w, Random r)
		{
			v4 v; for (; Math_.FEql(v = Random3(1, 0, r), Zero);) { }
			v = Math_.Normalise(v);
			v.w = w;
			return v;
		}

		/// <summary>Return a random vector with components within the interval [min,max]</summary>
		public static v4 Random2(float min, float max, float z, float w, Random r)
		{
			return new(
				r.Float(min, max),
				r.Float(min, max),
				z,
				w);
		}

		/// <summary>Return a random vector with components within the intervals given by each component of min and max</summary>
		public static v4 Random2(v4 min, v4 max, float z, float w, Random r)
		{
			return new(
				r.Float(min.x, max.x),
				r.Float(min.y, max.y),
				z,
				w);
		}

		/// <summary>Return a random vector on the unit 2D sphere</summary>
		public static v4 Random2(float rad, float z, float w, Random r)
		{
			var rad_sq = rad * rad;
			v4 v; for (; (v = Random2(-rad, rad, 0, 0, r)).LengthSq > rad_sq;) { }
			v.z = z;
			v.w = w;
			return v;
		}

		/// <summary>Return a random vector on the unit 2D sphere</summary>
		public static v4 Random2N(float z, float w, Random r)
		{
			v4 v; for (; Math_.FEql(v = Random2(1, 0, 0, r), Zero);) { }
			v = Math_.Normalise(v);
			v.z = z;
			v.w = w;
			return v;
		}

		#endregion

		/// <summary></summary>
		public string Description => $"{x}  {y}  {z}  {w}  //Len3({xyz.Length}) Len4({Length})";
	}

	public static partial class Math_
	{
		/// <summary>Approximate equal</summary>
		public static bool FEqlAbsolute(v4 a, v4 b, float tol)
		{
			return
				FEqlAbsolute(a.x, b.x, tol) &&
				FEqlAbsolute(a.y, b.y, tol) &&
				FEqlAbsolute(a.z, b.z, tol) &&
				FEqlAbsolute(a.w, b.w, tol);
		}
		public static bool FEqlRelative(v4 a, v4 b, float tol)
		{
			var max_a = MaxElement(Abs(a));
			var max_b = MaxElement(Abs(b));
			if (max_b == 0) return max_a < tol;
			if (max_a == 0) return max_b < tol;
			var abs_max_element = Max(max_a, max_b);
			return FEqlAbsolute(a, b, tol * abs_max_element);
		}
		public static bool FEql(v4 a, v4 b)
		{
			return FEqlRelative(a, b, TinyF);
		}

		/// <summary>Component-wise absolute value</summary>
		public static v4 Abs(v4 vec)
		{
			return new(
				Math.Abs(vec.x),
				Math.Abs(vec.y),
				Math.Abs(vec.z),
				Math.Abs(vec.w));
		}

		/// <summary>Return a vector containing the minimum components</summary>
		public static v4 Min(v4 lhs, v4 rhs)
		{
			return new(
				Math.Min(lhs.x, rhs.x),
				Math.Min(lhs.y, rhs.y),
				Math.Min(lhs.z, rhs.z),
				Math.Min(lhs.w, rhs.w));
		}
		public static v4 Min(v4 x, params v4[] vecs)
		{
			foreach (var v in vecs)
				x = Min(x, v);
			return x;
		}

		/// <summary>Return a vector containing the maximum components</summary>
		public static v4 Max(v4 lhs, v4 rhs)
		{
			return new(
				Math.Max(lhs.x, rhs.x),
				Math.Max(lhs.y, rhs.y),
				Math.Max(lhs.z, rhs.z),
				Math.Max(lhs.w, rhs.w));
		}
		public static v4 Max(v4 x, params v4[] vecs)
		{
			foreach (var v in vecs)
				x = Max(x, v);
			return x;
		}

		/// <summary>Clamp the components of 'vec' within the ranges of 'min' and 'max'</summary>
		public static v4 Clamp(v4 vec, v4 min, v4 max)
		{
			return new(
				Clamp(vec.x, min.x, max.x),
				Clamp(vec.y, min.y, max.y),
				Clamp(vec.z, min.z, max.z),
				Clamp(vec.w, min.w, max.w));
		}

		/// <summary>Return the maximum element value in 'v'</summary>
		public static float MaxElement(v4 v)
		{
			var i = v.x >= v.y ? v.x : v.y;
			var j = v.z >= v.w ? v.z : v.w;
			return i >= j ? i : j;
		}

		/// <summary>Return the minimum element value in 'v'</summary>
		public static float MinElement(v4 v)
		{
			var i = v.x <= v.y ? v.x : v.y;
			var j = v.z <= v.w ? v.z : v.w;
			return i <= j ? i : j;
		}

		/// <summary>Return the index of the maximum element in 'v'</summary>
		public static int MaxElementIndex(v4 v)
		{
			int i = v.x >= v.y ? 0 : 1;
			int j = v.z >= v.w ? 2 : 3;
			return v[i] >= v[j] ? i : j;
		}

		/// <summary>Return the index of the minimum element in 'v'</summary>
		public static int MinElementIndex(v4 v)
		{
			int i = v.x <= v.y ? 0 : 1;
			int j = v.z <= v.w ? 2 : 3;
			return v[i] <= v[j] ? i : j;
		}

		/// <summary>Return true if all components of 'vec' are finite</summary>
		public static bool IsFinite(v4 vec)
		{
			return IsFinite(vec.x) && IsFinite(vec.y) && IsFinite(vec.z) && IsFinite(vec.w);
		}

		/// <summary>Return true if any components of 'vec' are NaN</summary>
		public static bool IsNaN(v4 vec)
		{
			return IsNaN(vec.x) || IsNaN(vec.y) || IsNaN(vec.z) || IsNaN(vec.w);
		}

		/// <summary>Return the component-wise division 'a/b', or 'def' if 'b' contains zeros</summary>
		public static v4 Div(v4 a, v4 b, v4 def)
		{
			return new(
				Div(a.x, b.x, def.x),
				Div(a.y, b.y, def.y),
				Div(a.z, b.z, def.z),
				Div(a.w, b.w, def.w));
		}

		/// <summary>Return the component-wise square root of a vector</summary>
		public static v4 Sqrt(v4 a)
		{
			return new(
				(float)Math.Sqrt(a.x),
				(float)Math.Sqrt(a.y),
				(float)Math.Sqrt(a.z),
				(float)Math.Sqrt(a.w));
		}

		/// <summary>Normalise 'vec' by the length of the XYZW components</summary>
		public static v4 Normalise(v4 vec)
		{
			return vec / vec.Length;
		}
		public static v4 Normalise(ref v4 vec)
		{
			return vec /= vec.Length;
		}

		/// <summary>Normalise 'vec' by the length of the XYZW components or return 'def' if zero</summary>
		public static v4 Normalise(v4 vec, v4 def)
		{
			if (vec == v4.Zero) return def;
			var norm = Normalise(vec);
			return norm != v4.Zero ? norm : def;
		}

		/// <summary>Dot product of XYZW components</summary>
		public static float Dot(v4 lhs, v4 rhs)
		{
			return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z + lhs.w * rhs.w;
		}

		/// <summary>Cross product of XYZ components</summary>
		public static v4 Cross(v4 lhs, v4 rhs)
		{
			return new(lhs.y * rhs.z - lhs.z * rhs.y, lhs.z * rhs.x - lhs.x * rhs.z, lhs.x * rhs.y - lhs.y * rhs.x, 0.0f);
		}

		/// <summary>Triple product (a . (b x c)) of XYZ components</summary>
		public static float Triple(v4 a, v4 b, v4 c)
		{
			return Dot(a, Cross(b, c));
		}

		/// <summary>True if 'lhs' and 'rhs' are parallel</summary>
		public static bool Parallel(v4 lhs, v4 rhs)
		{
			return FEql(Cross(lhs, rhs).LengthSq, 0);
		}

		/// <summary>Linearly interpolate between two vectors</summary>
		public static v4 Lerp(v4 lhs, v4 rhs, float frac)
		{
			return lhs * (1f - frac) + rhs * (frac);
		}

		/// <summary>Returns a vector guaranteed to not be parallel to 'vec'</summary>
		public static v4 CreateNotParallelTo(v4 vec)
		{
			bool x_aligned = Abs(vec.x) > Abs(vec.y) && Abs(vec.x) > Abs(vec.z);
			return new(SignF(!x_aligned), 0.0f, SignF(x_aligned), vec.w);
		}

		/// <summary>Returns a vector perpendicular to 'vec'</summary>
		public static v4 Perpendicular(v4 vec)
		{
			Debug.Assert(vec != v4.Zero, "Cannot make a perpendicular to a zero vector");
			var v = Cross(vec, CreateNotParallelTo(vec));
			v *= (float)Math.Sqrt(vec.LengthSq / v.LengthSq);
			return v;
		}

		/// <summary>Returns a vector perpendicular to 'vec' favouring 'previous' as the preferred perpendicular</summary>
		public static v4 Perpendicular(v4 vec, v4 previous)
		{
			Debug.Assert(!FEql(vec, v4.Zero), "Cannot make a perpendicular to a zero vector");

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

		/// <summary>Returns a 3 bit bitmask of the octant the vector is in where X = 0x1, Y = 0x2, Z = 0x4</summary>
		public static uint Octant(v4 vec)
		{
			return (uint)(((int)SignF(vec.x >= 0.0f)) | ((int)SignF(vec.y >= 0.0f) << 1) | ((int)SignF(vec.z >= 0.0f) << 2));
		}

		/// <summary>Return the cosine of the angle between two vectors</summary>
		public static float CosAngle(v4 lhs, v4 rhs)
		{
			Debug.Assert(lhs.w == 0 && rhs.w == 0, "CosAngle is intended for 3-vectors");
			Debug.Assert(lhs.LengthSq != 0 && rhs.LengthSq != 0, "CosAngle undefined for zero vectors");
			return Clamp(Dot(lhs, rhs) / (float)Math.Sqrt(lhs.LengthSq * rhs.LengthSq), -1f, 1f);
		}

		/// <summary>Return the angle between two vectors</summary>
		public static float Angle(v4 lhs, v4 rhs)
		{
			return (float)Math.Acos(CosAngle(lhs, rhs));
		}

		/// <summary>Return the average of a collection of vectors</summary>
		public static v4 Average(IEnumerable<v4> vecs)
		{
			var acc = v4.Zero;
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

	[TestFixture] public class UnitTestV4
	{
		[Test] public void Basic()
		{
			var a = new v4(1,2,-3,-4);
			Assert.True(a.x == +1);
			Assert.True(a.y == +2);
			Assert.True(a.z == -3);
			Assert.True(a.w == -4);
		}
		[Test] public void MinMax()
		{
			var a = new v4(3,-1,2,-4);
			var b = new v4(-2,-1,4,2);
			Assert.True(Math_.Max(a,b) == new v4(3,-1,4,2));
			Assert.True(Math_.Min(a,b) == new v4(-2,-1,2,-4));
		}
		[Test] public void Length()
		{
			var a = new v4(3,-1,2,-4);
			Assert.True(Math_.FEql(a.LengthSq, a.x*a.x + a.y*a.y + a.z*a.z + a.w*a.w));
			Assert.True(Math_.FEql(a.Length, (float)Math.Sqrt(a.LengthSq)));
			Assert.True(Math_.FEql(a.xyz.LengthSq, a.x*a.x + a.y*a.y + a.z*a.z));
			Assert.True(Math_.FEql(a.xyz.Length  , (float)Math.Sqrt(a.xyz.LengthSq)));
			Assert.True(Math_.FEql(a.xy.LengthSq, a.x*a.x + a.y*a.y));
			Assert.True(Math_.FEql(a.xy.Length  , (float)Math.Sqrt(a.xy.LengthSq)));
		}
		[Test] public void Normals()
		{
			var a = new v4(3,-1,2,-4);
			var b = Math_.Normalise(a);
			Assert.True(Math_.FEql((float)Math.Sqrt(b.x*b.x + b.y*b.y + b.z*b.z + b.w*b.w), 1.0f));
			Assert.True(Math_.FEql(a.Length, 1.0f) == false);
			Assert.True(Math_.FEql(b.Length, 1.0f) == true);
		}
		[Test] public void DotProduct()
		{
			var a = new v4(-2,  4,  2,  6);
			var b = new v4( 3, -5,  2, -4);
			Assert.True(Math_.FEql(Math_.Dot(a,b), -46f));
		}
		[Test] public void NumericsConversion()
		{
			var vec0 = new Vector4(1,2,3,4);
			var vec1 = (v4)vec0;
			var vec2 = (Vector4)vec1;
			var vec3 = (v4)vec2;

			Assert.True(vec0 == vec2);
			Assert.True(vec1 == vec3);
			Assert.True(vec0.X == vec1.x);
			Assert.True(vec0.Y == vec1.y);
			Assert.True(vec0.Z == vec1.z);
			Assert.True(vec0.W == vec1.w);
		}
	}
}
#endif
