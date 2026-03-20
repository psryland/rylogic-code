//***************************************************
// Vector2
//  Copyright (c) Rylogic Ltd 2008
//***************************************************

using System;
using System.Diagnostics;
using System.Drawing;
using System.Runtime.InteropServices;
using System.Numerics;
using Rylogic.Extn;

namespace Rylogic.Maths
{
	[Serializable]
	[StructLayout(LayoutKind.Explicit)]
	[DebuggerDisplay("{Description,nq}")]
	public struct v2
	{
		[FieldOffset(0)] public float x;
		[FieldOffset(4)] public float y;

		// Constructors
		public v2(float x_)
			:this(x_, x_)
		{}
		public v2(float x_, float y_)
			:this()
		{
			x = x_;
			y = y_;
		}
		public v2(PointF pt)
			:this(pt.X, pt.Y)
		{}
		public v2(SizeF sz)
			:this(sz.Width, sz.Height)
		{}
		public v2(float[] arr)
			:this()
		{
			x = arr[0];
			y = arr[1];
		}

		/// <summary>Get/Set components by index</summary>
		public float this[int i]
		{
			get
			{
				switch (i) {
				case 0: return x;
				case 1: return y;
				}
				throw new ArgumentException("index out of range", "i");
			}
			set
			{
				switch (i) {
				case 0: x = value; return;
				case 1: y = value; return;
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

		/// <summary>Length</summary>
		public float LengthSq => x * x + y * y;
		public float Length => (float)Math.Sqrt(LengthSq);

		/// <summary>Explicit conversion to an array. Note not implicit because it gets called when converting v2 to an object type. e.g. v2? x = v2.TryParse4("", out v) ? v : null. </summary>
		public static explicit operator v2(float[] a)
		{
			return new(a[0], a[1]);
		}
		public static explicit operator float[] (v2 p)
		{
			return new float[] { p.x, p.y };
		}
		public static explicit operator v2(double[] a)
		{
			return new((float)a[0], (float)a[1]);
		}
		public static explicit operator double[] (v2 p)
		{
			return new double[] { p.x, p.y };
		}

		/// <summary>Implicit conversion to System.Numerics</summary>
		public static implicit operator v2(Vector2 v)
		{
			return new v2(v.X, v.Y);
		}
		public static implicit operator Vector2(v2 v)
		{
			return new Vector2(v.x, v.y);
		}

		#region Statics
		public readonly static v2 Zero     = new(0f, 0f);
		public readonly static v2 XAxis    = new(1f, 0f);
		public readonly static v2 YAxis    = new(0f, 1f);
		public readonly static v2 One      = new(1f, 1f);
		public readonly static v2 NaN      = new(float.NaN, float.NaN);
		public readonly static v2 MinValue = new(float.MinValue, float.MinValue);
		public readonly static v2 MaxValue = new(float.MaxValue, float.MaxValue);
		#endregion

		#region Operators
		public static v2 operator + (v2 vec)
		{
			return vec;
		}
		public static v2 operator - (v2 vec)
		{
			return new(-vec.x, -vec.y);
		}
		public static v2 operator + (v2 lhs, v2 rhs)
		{
			return new(lhs.x + rhs.x, lhs.y + rhs.y);
		}
		public static v2 operator - (v2 lhs, v2 rhs)
		{
			return new(lhs.x - rhs.x, lhs.y - rhs.y);
		}
		public static v2 operator * (v2 lhs, float rhs)
		{
			return new(lhs.x * rhs, lhs.y * rhs);
		}
		public static v2 operator *(v2 lhs, double rhs)
		{
			return lhs * (float)rhs;
		}
		public static v2 operator * (float lhs, v2 rhs)
		{
			return new(lhs * rhs.x, lhs * rhs.y);
		}
		public static v2 operator *(double lhs, v2 rhs)
		{
			return (float)lhs * rhs;
		}
		public static v2 operator * (v2 lhs, v2 rhs)
		{
			return new(lhs.x * rhs.x, lhs.y * rhs.y);
		}
		public static v2 operator / (v2 lhs, float rhs)
		{
			return new(lhs.x / rhs, lhs.y / rhs);
		}
		public static v2 operator /(v2 lhs, double rhs)
		{
			return lhs / (float)rhs;
		}
		public static v2 operator / (v2 lhs, v2 rhs)
		{
			return new(lhs.x / rhs.x, lhs.y / rhs.y);
		}
		public static v2 operator %(v2 lhs, float rhs)
		{
			return new(lhs.x % rhs, lhs.y % rhs);
		}
		public static v2 operator %(v2 lhs, double rhs)
		{
			return lhs % (float)rhs;
		}
		public static v2 operator %(v2 lhs, v2 rhs)
		{
			return new(lhs.x % rhs.x, lhs.y % rhs.y);
		}
		#endregion

		#region Equals
		public static bool operator == (v2 lhs, v2 rhs)
		{
			return lhs.x == rhs.x && lhs.y == rhs.y;
		}
		public static bool operator != (v2 lhs, v2 rhs)
		{
			return !(lhs == rhs);
		}
		public override bool Equals(object? o)
		{
			return o is v2 v && v == this;
		}
		public override int GetHashCode()
		{
			return new { x, y }.GetHashCode();
		}
		#endregion

		#region ToString
		public override string ToString() => $"{x} {y}";
		public string ToString(string format) => $"{x.ToString(format)} {y.ToString(format)}";
		public string ToCodeString(ECodeString fmt = ECodeString.CSharp)
		{
			return fmt switch
			{
				ECodeString.CSharp => $"{x:+0.0000000;-0.0000000;+0.0000000}f, {y:+0.0000000;-0.0000000;+0.0000000}f",
				ECodeString.Cpp => $"{x:+0.0000000;-0.0000000;+0.0000000}f, {y:+0.0000000;-0.0000000;+0.0000000}f",
				ECodeString.Python => $"{x:+0.0000000;-0.0000000;+0.0000000}, {y:+0.0000000;-0.0000000;+0.0000000}",
				_ => ToString(),
			};
		}
		#endregion

		#region Parse
		public static v2 Parse(string s)
		{
			if (s == null)
				throw new ArgumentNullException("s", "v2.Parse3() string argument was null");

			var values = s.Split([' ', ',', '\t'], 2);
			if (values.Length != 2)
				throw new FormatException("v2.Parse() string argument does not represent a 2 component vector");

			return new(
				float.Parse(values[0]),
				float.Parse(values[1]));
		}
		public static bool TryParse(string s, out v2 vec)
		{
			vec = Zero;
			if (s == null)
				return false;

			var values = s.Split([' ', ',', '\t'], 2);
			return
				values.Length == 2 &&
				float.TryParse(values[0], out vec.x) &&
				float.TryParse(values[1], out vec.y);
		}
		public static v2? TryParse(string s)
		{
			return TryParse(s, out var vec) ? vec : null;
		}
		#endregion

		#region Random

		/// <summary>Return a random vector with components within the interval [min,max]</summary>
		public static v2 Random2(float min, float max, Random r)
		{
			return new(r.Float(min, max), r.Float(min, max));
		}

		/// <summary>Return a random vector with components within the intervals given by each component of min and max</summary>
		public static v2 Random2(v2 min, v2 max, Random r)
		{
			return new(r.Float(min.x, max.x), r.Float(min.y, max.y));
		}

		/// <summary>Return a random vector on the unit 2D sphere</summary>
		public static v2 Random2(float rad, Random r)
		{
			var rad_sq = rad*rad;
			v2 v; for (; (v = Random2(-rad, rad, r)).LengthSq > rad_sq;) { }
			return v;
		}

		/// <summary>Return a random vector on the unit 2D sphere</summary>
		public static v2 Random2N(Random r)
		{
			v2 v; for (; Math_.FEql(v = Random2(1, r), Zero);) { }
			return Math_.Normalise(v);
		}

		#endregion

		#region Conversion
		/// <summary>Create from Drawing type</summary>
		public static v2 From(Point point)
		{
			return new(point.X, point.Y);
		}
		public static v2 From(PointF point)
		{
			return new(point.X, point.Y);
		}
		public static v2 From(Size size)
		{
			return new(size.Width, size.Height);
		}
		public static v2 From(SizeF size)
		{
			return new(size.Width, size.Height);
		}

		/// <summary>Convert to Drawing type</summary>
		public Point ToPoint()
		{
			return new Point((int)x, (int)y);
		}
		public PointF ToPointF()
		{
			return new PointF(x, y);
		}
		public Size ToSize()
		{
			return new Size((int)x, (int)y);
		}
		public SizeF ToSizeF()
		{
			return new SizeF(x, y);
		}
		public Rectangle ToRectangle()
		{
			return new Rectangle(Point.Empty, ToSize());
		}
		public RectangleF ToRectangleF()
		{
			return new RectangleF(PointF.Empty, ToSizeF());
		}

		/// <summary>Implicit conversion to/from drawing types</summary>
		public static implicit operator v2(PointF p)
		{
			return From(p);
		}
		public static implicit operator PointF(v2 p)
		{
			return p.ToPointF();
		}
		public static implicit operator v2(SizeF s)
		{
			return From(s);
		}
		public static implicit operator SizeF(v2 s)
		{
			return s.ToSizeF();
		}
		#endregion

		/// <summary></summary>
		public string Description => $"{x}  {y}  //Len({Length})";
	}

	public static partial class Math_
	{
		/// <summary>Approximate equal</summary>
		public static bool FEqlAbsolute(v2 a, v2 b, float tol)
		{
			return
				FEqlAbsolute(a.x, b.x, tol) &&
				FEqlAbsolute(a.y, b.y, tol);
		}
		public static bool FEqlRelative(v2 a, v2 b, float tol)
		{
			var max_a = MaxElement(Abs(a));
			var max_b = MaxElement(Abs(b));
			if (max_b == 0) return max_a < tol;
			if (max_a == 0) return max_b < tol;
			var abs_max_element = Max(max_a, max_b);
			return FEqlAbsolute(a, b, tol * abs_max_element);
		}
		public static bool FEql(v2 a, v2 b)
		{
			return FEqlRelative(a.x, b.x, TinyF);
		}

		/// <summary>Component absolute value</summary>
		public static v2 Abs(v2 vec)
		{
			return new(
				Math.Abs(vec.x),
				Math.Abs(vec.y));
		}

		/// <summary>Return true if all components of 'vec' are finite</summary>
		public static bool IsFinite(v2 vec)
		{
			return IsFinite(vec.x) && IsFinite(vec.y);
		}

		/// <summary>Return true if any components of 'vec' are NaN</summary>
		public static bool IsNaN(v2 vec)
		{
			return IsNaN(vec.x) || IsNaN(vec.y);
		}

		/// <summary>Return 'a/b', or 'def' if 'b' is zero</summary>
		public static v2 Div(v2 a, v2 b, v2 def)
		{
			return b != v2.Zero ? a / b : def;
		}

		/// <summary>Return a vector containing the minimum components</summary>
		public static v2 Min(v2 lhs, v2 rhs)
		{
			 return new(
				 Math.Min(lhs.x, rhs.x),
				 Math.Min(lhs.y, rhs.y));
		}
		public static v2 Min(v2 x, params v2[] vecs)
		{
			foreach (var v in vecs)
				x = Min(x,v);
			return x;
		}

		/// <summary>Return a vector containing the maximum components</summary>
		public static v2 Max(v2 lhs, v2 rhs)
		{
			 return new(
				 Math.Max(lhs.x, rhs.x),
				 Math.Max(lhs.y, rhs.y));
		}
		public static v2 Max(v2 x, params v2[] vecs)
		{
			foreach (var v in vecs)
				x = Max(x,v);
			return x;
		}

		/// <summary>Clamp the components of 'vec' within the ranges of 'min' and 'max'</summary>
		public static v2 Clamp(v2 vec, v2 min, v2 max)
		{
			return new(
				Clamp(vec.x, min.x, max.x),
				Clamp(vec.y, min.y, max.y));
		}

		/// <summary>Return the maximum element value in 'v'</summary>
		public static float MaxElement(v2 v)
		{
			return v.x >= v.y ? v.x : v.y;
		}

		/// <summary>Return the minimum element value in 'v'</summary>
		public static float MinElement(v2 v)
		{
			return v.x <= v.y ? v.x : v.y;
		}

		/// <summary>Return the index of the maximum element in 'v'</summary>
		public static int MaxElementIndex(v2 v)
		{
			return v.x >= v.y ? 0 : 1;
		}

		/// <summary>Return the index of the minimum element in 'v'</summary>
		public static int MinElementIndex(v2 v)
		{
			return v.x <= v.y ? 0 : 1;
		}

		/// <summary>Return the component-wise square root of a vector</summary>
		public static v2 Sqrt(v2 a)
		{
			return new(
				(float)Math.Sqrt(a.x),
				(float)Math.Sqrt(a.y));
		}

		/// <summary>Normalise 'vec' by the length of the XY components</summary>
		public static v2 Normalise(v2 vec)
		{
			return vec / vec.Length;
		}
		public static v2 Normalise(ref v2 vec)
		{
			return vec /= vec.Length;
		}

		/// <summary>Normalise 'vec' by the length of the XY components or return 'def' if zero</summary>
		public static v2 Normalise(v2 vec, v2 def)
		{
			if (vec == v2.Zero) return def;
			var norm = Normalise(vec);
			return norm != v2.Zero ? norm : def;
		}

		/// <summary>Dot product of XYZ components</summary>
		public static float Dot(v2 lhs, v2 rhs)
		{
			return lhs.x * rhs.x + lhs.y * rhs.y;
		}

		/// <summary>Cross product: Dot(Rotate90CW(lhs), rhs)</summary>
		public static float Cross(v2 lhs, v2 rhs)
		{
			return lhs.y * rhs.x - lhs.x * rhs.y;
		}

		/// <summary>True if 'lhs' and 'rhs' are parallel</summary>
		public static bool Parallel(v2 lhs, v2 rhs)
		{
			return FEql(Cross(lhs, rhs), 0);
		}

		/// <summary>Linearly interpolate between two vectors</summary>
		public static v2 Lerp(v2 lhs, v2 rhs, float frac)
		{
			return lhs * (1f - frac) + rhs * (frac);
		}

		/// <summary>Returns a vector guaranteed to not be parallel to 'vec'</summary>
		public static v2 CreateNotParallelTo(v2 vec)
		{
			bool x_aligned = Abs(vec.x) > Abs(vec.y);
			return new(SignF(!x_aligned), SignF(x_aligned));
		}

		/// <summary>Returns a vector perpendicular to 'vec'</summary>
		public static v2 Perpendicular(v2 vec)
		{
			Debug.Assert(!FEql(vec, v2.Zero), "Cannot make a perpendicular to a zero vector");
			return RotateCCW(vec);
		}

		/// <summary>Returns a vector perpendicular to 'vec' favouring 'previous' as the preferred perpendicular</summary>
		public static v2 Perpendicular(v2 vec, v2 previous)
		{
			Debug.Assert(!FEql(vec, v2.Zero), "Cannot make a perpendicular to a zero vector");

			// If 'previous' is still perpendicular, keep it
			if (FEql(Dot(vec, previous), 0))
				return previous;

			// If 'previous' is parallel to 'vec', choose a new perpendicular
			if (Parallel(vec, previous))
				return Perpendicular(vec);

			// Otherwise, make a perpendicular that is close to 'previous'
			var v = previous - (Dot(vec,previous) / vec.LengthSq) * vec;
			v *= (float)Math.Sqrt(vec.LengthSq / v.LengthSq);
			return v;
		}

		/// <summary>Return the cosine of the angle between two vectors</summary>
		public static float CosAngle(v2 lhs, v2 rhs)
		{
			// Return the cosine of the angle between two vectors
			Debug.Assert(lhs.LengthSq != 0 && rhs.LengthSq != 0, "CosAngle undefined for zero vectors");
			return Clamp(Dot(lhs,rhs) / (float)Math.Sqrt(lhs.LengthSq * rhs.LengthSq), -1f, 1f);
		}

		/// <summary>Return the angle between two vectors</summary>
		public static float Angle(v2 lhs, v2 rhs)
		{
			return (float)Math.Acos(CosAngle(lhs, rhs));
		}

		/// <summary>Rotate 'vec' counter clockwise</summary>
		public static v2 RotateCCW(v2 vec)
		{
			return new(-vec.y, vec.x);
		}

		/// <summary>Rotate 'vec' clockwise</summary>
		public static v2 RotateCW(v2 vec)
		{
			return new(vec.y, -vec.x);
		}

		/// <summary>Return a point on the unit circle at 'ang' (radians) from the X axis</summary>
		public static v2 UnitCircle(float ang)
		{
			return new((float)Math.Cos(ang), (float)Math.Sin(ang));
		}
	}
}

#if PR_UNITTESTS
namespace Rylogic.UnitTests
{
	using Maths;

	[TestFixture] public class UnitTestv2
	{
		[Test] public void Basic()
		{
			var a = new v2(1,2);
			Assert.True(a.x == +1);
			Assert.True(a.y == +2);
		}
		[Test] public void MinMax()
		{
			var a = new v2(3,-1);
			var b = new v2(-2,-1);
			Assert.True(Math_.Max(a,b) == new v2(3,-1));
			Assert.True(Math_.Min(a,b) == new v2(-2,-1));
		}
		[Test] public void Length()
		{
			var a = new v2(3,-1);

			Assert.True(Math_.FEql(a.LengthSq, a.x*a.x + a.y*a.y));
			Assert.True(Math_.FEql(a.Length  , (float)Math.Sqrt(a.LengthSq)));
		}
		[Test] public void Normals()
		{
			var a = new v2(3,-1);
			var b = Math_.Normalise(a);
			Assert.True(Math_.FEql(b.Length, 1.0f));
			Assert.True(Math_.FEql(a.Length, 1.0f) == false);
		}
		[Test] public void DotProduct()
		{
			var a = new v2(-2,  4);
			var b = new v2( 3, -5);
			Assert.True(Math_.FEql(Math_.Dot(a,b), -26f));
		}
	}
}
#endif