//***************************************************
// Vector2
//  Copyright (c) Rylogic Ltd 2008
//***************************************************

using System;
using System.Diagnostics;
using System.Drawing;
using System.Runtime.InteropServices;
using Rylogic.Extn;

namespace Rylogic.Maths
{
	[Serializable]
	[StructLayout(LayoutKind.Explicit)]
	[DebuggerDisplay("{Description,nq}")]
	public struct IVec2
	{
		[FieldOffset(0)] public int x;
		[FieldOffset(4)] public int y;

		// Constructors
		public IVec2(int x_)
			: this(x_, x_)
		{}
		public IVec2(int x_, int y_)
			:this()
		{
			x = x_;
			y = y_;
		}
		public IVec2(Point pt)
			:this(pt.X, pt.Y)
		{}
		public IVec2(Size sz)
			:this(sz.Width, sz.Height)
		{}
		public IVec2(int[] arr)
			:this()
		{
			x = arr[0];
			y = arr[1];
		}

		/// <summary>Get/Set components by index</summary>
		public int this[int i]
		{
			get
			{
				switch (i)
				{
					case 0: return x;
					case 1: return y;
					default: throw new ArgumentException("index out of range", "i");
				}
			}
			set
			{
				switch (i)
				{
				case 0: x = value; return;
				case 1: y = value; return;
				default: throw new ArgumentException("index out of range", "i");
				}
			}
		}
		public int this[uint i]
		{
			get => this[(int)i];
			set => this[(int)i] = value;
		}

		/// <summary>Length</summary>
		public int LengthSq => x * x + y * y;
		public float Length => (float)Math.Sqrt(LengthSq);

		/// <summary>ToString</summary>
		public string ToString2() => $"{x} {y}";
		public string ToString2(string format) => $"{x.ToString(format)} {y.ToString(format)}";
		public override string ToString() => ToString2();

		/// <summary>Conversions</summary>
		public static implicit operator v2(IVec2 a)
		{
			return new v2(a[0], a[1]);
		}
		public static explicit operator IVec2(int[] a)
		{
			return new IVec2(a[0], a[1]);
		}
		public static explicit operator int[] (IVec2 p)
		{
			/// <summary>Explicit conversion to an array. Note not implicit because it gets called when converting IVec2 to an object type. e.g. IVec2? x = IVec2.TryParse4("", out v) ? v : null. </summary>
			return [p.x, p.y];
		}

		// Static IVec2 types
		public readonly static IVec2 Zero     = new(0, 0);
		public readonly static IVec2 XAxis    = new(1, 0);
		public readonly static IVec2 YAxis    = new(0, 1);
		public readonly static IVec2 One      = new(1, 1);
		public readonly static IVec2 MinValue = new(int.MinValue, int.MinValue);
		public readonly static IVec2 MaxValue = new(int.MaxValue, int.MaxValue);

		/// <summary>Operators</summary>
		public static IVec2 operator + (IVec2 vec)
		{
			return vec;
		}
		public static IVec2 operator - (IVec2 vec)
		{
			return new IVec2(-vec.x, -vec.y);
		}
		public static IVec2 operator + (IVec2 lhs, IVec2 rhs)
		{
			return new IVec2(lhs.x + rhs.x, lhs.y + rhs.y);
		}
		public static IVec2 operator - (IVec2 lhs, IVec2 rhs)
		{
			return new IVec2(lhs.x - rhs.x, lhs.y - rhs.y);
		}
		public static IVec2 operator * (IVec2 lhs, int rhs)
		{
			return new IVec2(lhs.x * rhs, lhs.y * rhs);
		}
		public static IVec2 operator *(IVec2 lhs, double rhs)
		{
			return lhs * (float)rhs;
		}
		public static IVec2 operator * (int lhs, IVec2 rhs)
		{
			return new IVec2(lhs * rhs.x, lhs * rhs.y);
		}
		public static IVec2 operator *(double lhs, IVec2 rhs)
		{
			return (float)lhs * rhs;
		}
		public static IVec2 operator * (IVec2 lhs, IVec2 rhs)
		{
			return new IVec2(lhs.x * rhs.x, lhs.y * rhs.y);
		}
		public static IVec2 operator / (IVec2 lhs, int rhs)
		{
			return new IVec2(lhs.x / rhs, lhs.y / rhs);
		}
		public static IVec2 operator /(IVec2 lhs, double rhs)
		{
			return lhs / (float)rhs;
		}
		public static IVec2 operator / (IVec2 lhs, IVec2 rhs)
		{
			return new IVec2(lhs.x / rhs.x, lhs.y / rhs.y);
		}

		#region System.Drawing conversion
		/// <summary>Create from Drawing type</summary>
		public static IVec2 From(Point point)
		{
			return new IVec2(point.X, point.Y);
		}
		public static IVec2 From(Size size)
		{
			return new IVec2(size.Width, size.Height);
		}

		/// <summary>Convert to Drawing type</summary>
		public Point ToPoint()
		{
			return new Point((int)x, (int)y);
		}
		public Size ToSize()
		{
			return new Size((int)x, (int)y);
		}
		public Rectangle ToRectangle()
		{
			return new Rectangle(Point.Empty, ToSize());
		}

		/// <summary>Implicit conversion to/from drawing types</summary>
		public static implicit operator IVec2(Point p)
		{
			return From(p);
		}
		public static implicit operator Point(IVec2 p)
		{
			return p.ToPoint();
		}
		public static implicit operator IVec2(Size s)
		{
			return From(s);
		}
		public static implicit operator Size(IVec2 s)
		{
			return s.ToSize();
		}
		#endregion

		#region Parse
		public static IVec2 Parse2(string s)
		{
			if (s == null)
				throw new ArgumentNullException("s", "IVec2.Parse2() string argument was null");

			var values = s.Split([' ', ',', '\t'], 2);
			if (values.Length != 2)
				throw new FormatException("IVec2.Parse2() string argument does not represent a 2 component vector");

			return new IVec2(int.Parse(values[0]), int.Parse(values[1]));
		}
		public static bool TryParse2(string s, out IVec2 vec)
		{
			vec = Zero;
			if (s == null)
				return false;

			var values = s.Split([' ', ',', '\t'], 2);
			return values.Length == 2 && int.TryParse(values[0], out vec.x) && int.TryParse(values[1], out vec.y);
		}
		public static IVec2? TryParse2(string s)
		{
			IVec2 vec;
			return TryParse2(s, out vec) ? (IVec2?)vec : null;
		}
		#endregion

		#region Random

		/// <summary>Return a random vector with components within the interval [min,max]</summary>
		public static IVec2 Random2(int min, int max, Random r)
		{
			return new IVec2(r.Int(min, max), r.Int(min, max));
		}

		/// <summary>Return a random vector with components within the intervals given by each component of min and max</summary>
		public static IVec2 Random2(IVec2 min, IVec2 max, Random r)
		{
			return new IVec2(r.Int(min.x, max.x), r.Int(min.y, max.y));
		}

		#endregion

		#region Equals
		public static bool operator == (IVec2 lhs, IVec2 rhs)
		{
			return lhs.x == rhs.x && lhs.y == rhs.y;
		}
		public static bool operator != (IVec2 lhs, IVec2 rhs)
		{
			return !(lhs == rhs);
		}
		public override bool Equals(object? o)
		{
			return o is IVec2 v && v == this;
		}
		public override int GetHashCode()
		{
			return new { x, y }.GetHashCode();
		}
		#endregion

		/// <summary></summary>
		public string Description => $"{x}  {y}  //Len({Length})";
	}

	public static partial class Math_
	{
		/// <summary>Component absolute value</summary>
		public static IVec2 Abs(IVec2 vec)
		{
			return new IVec2(
				Math.Abs(vec.x),
				Math.Abs(vec.y));
		}

		/// <summary>Return 'a/b', or 'def' if 'b' is zero</summary>
		public static IVec2 Div(IVec2 a, IVec2 b, IVec2 def)
		{
			return b != IVec2.Zero ? a / b : def;
		}

		/// <summary>Return a vector containing the minimum components</summary>
		public static IVec2 Min(IVec2 lhs, IVec2 rhs)
		{
			 return new IVec2(
				 Math.Min(lhs.x, rhs.x),
				 Math.Min(lhs.y, rhs.y));
		}
		public static IVec2 Min(IVec2 x, params IVec2[] vecs)
		{
			foreach (var v in vecs)
				x = Min(x,v);
			return x;
		}

		/// <summary>Return a vector containing the maximum components</summary>
		public static IVec2 Max(IVec2 lhs, IVec2 rhs)
		{
			 return new IVec2(
				 Math.Max(lhs.x, rhs.x),
				 Math.Max(lhs.y, rhs.y));
		}
		public static IVec2 Max(IVec2 x, params IVec2[] vecs)
		{
			foreach (var v in vecs)
				x = Max(x,v);
			return x;
		}

		/// <summary>Clamp the components of 'vec' within the ranges of 'min' and 'max'</summary>
		public static IVec2 Clamp(IVec2 vec, IVec2 min, IVec2 max)
		{
			return new IVec2(
				Clamp(vec.x, min.x, max.x),
				Clamp(vec.y, min.y, max.y));
		}

		/// <summary>Return the maximum element value in 'v'</summary>
		public static int MaxElement(IVec2 v)
		{
			return v.x >= v.y ? v.x : v.y;
		}

		/// <summary>Return the minimum element value in 'v'</summary>
		public static int MinElement(IVec2 v)
		{
			return v.x <= v.y ? v.x : v.y;
		}

		/// <summary>Return the index of the maximum element in 'v'</summary>
		public static int MaxElementIndex(IVec2 v)
		{
			return v.x >= v.y ? 0 : 1;
		}

		/// <summary>Return the index of the minimum element in 'v'</summary>
		public static int MinElementIndex(IVec2 v)
		{
			return v.x <= v.y ? 0 : 1;
		}

		/// <summary>Dot product of XYZ components</summary>
		public static int Dot(IVec2 lhs, IVec2 rhs)
		{
			return lhs.x * rhs.x + lhs.y * rhs.y;
		}

		/// <summary>Cross product: Dot(Rotate90CW(lhs), rhs)</summary>
		public static int Cross(IVec2 lhs, IVec2 rhs)
		{
			return lhs.y * rhs.x - lhs.x * rhs.y;
		}

		/// <summary>True if 'lhs' and 'rhs' are parallel</summary>
		public static bool Parallel(IVec2 lhs, IVec2 rhs)
		{
			return FEql(Cross(lhs, rhs), 0);
		}

		/// <summary>Rotate 'vec' counter clockwise</summary>
		public static IVec2 RotateCCW(IVec2 vec)
		{
			return new IVec2(-vec.y, vec.x);
		}

		/// <summary>Rotate 'vec' clockwise</summary>
		public static IVec2 RotateCW(IVec2 vec)
		{
			return new IVec2(vec.y, -vec.x);
		}
	}
}

#if PR_UNITTESTS
namespace Rylogic.UnitTests
{
	using Maths;

	[TestFixture] public class UnitTestIVec2
	{
		[Test] public void Basic()
		{
			var a = new IVec2(1,2);
			Assert.True(a.x == +1);
			Assert.True(a.y == +2);
		}
		[Test] public void MinMax()
		{
			var a = new IVec2(3,-1);
			var b = new IVec2(-2,-1);
			Assert.True(Math_.Max(a,b) == new IVec2(3,-1));
			Assert.True(Math_.Min(a,b) == new IVec2(-2,-1));
		}
		[Test] public void Length()
		{
			var a = new IVec2(3,-1);

			Assert.True(Math_.FEql(a.LengthSq, a.x*a.x + a.y*a.y));
			Assert.True(Math_.FEql(a.Length  , (float)Math.Sqrt(a.LengthSq)));
		}
		[Test] public void DotProduct()
		{
			var a = new IVec2(-2,  4);
			var b = new IVec2( 3, -5);
			Assert.True(Math_.Dot(a,b) == -26f);
		}
	}
}
#endif