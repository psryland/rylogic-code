//***************************************************
// Matrix
//  Copyright (c) Rylogic Ltd 2008
//***************************************************

using System;
using System.Diagnostics;
using Rylogic.Extn;

namespace Rylogic.Maths
{
	[DebuggerDisplay("{Description,nq}")]
	public struct m2x2
	{
		public v2 x;
		public v2 y;

		public m2x2(float x)
			:this(new v2(x), new v2(x))
		{}
		public m2x2(v2 x, v2 y) :this()
		{
			this.x = x;
			this.y = y;
		}

		/// <summary>Get/Set columns by index</summary>
		public v2 this[int c]
		{
			get
			{
				switch (c) {
				case 0: return x;
				case 1: return y;
				}
				throw new ArgumentException("index out of range", "i");
			}
			set
			{
				switch (c) {
				case 0: x = value; return;
				case 1: y = value; return;
				}
				throw new ArgumentException("index out of range", "i");
			}
		}
		public v2 this[uint c]
		{
			get { return this[(int)c]; }
			set { this[(int)c] = value; }
		}

		/// <summary>Get/Set components by index. Note: row,col is standard (i.e. as in Matlab)</summary>
		public float this[int r, int c]
		{
			get { return this[c][r]; }
			set
			{
				var vec = this[c];
				vec[r] = value;
				this[c] = vec;
			}
		}
		public float this[uint r, uint c]
		{
			get { return this[(int)c][(int)r]; }
			set
			{
				var vec = this[c];
				vec[r] = value;
				this[c] = vec;
			}
		}

		/// <summary>To flat array</summary>
		public float[] ToArray()
		{
			return new []
			{
				x.x, x.y,
				y.x, y.y,
			};
		}

		#region Statics
		public readonly static m2x2 Zero     = new(v2.Zero , v2.Zero);
		public readonly static m2x2 Identity = new(v2.XAxis, v2.YAxis);
		#endregion

		#region Operators
		public static m2x2 operator +(m2x2 rhs)
		{
			return rhs;
		}
		public static m2x2 operator -(m2x2 rhs)
		{
			return new(-rhs.x, -rhs.y);
		}
		public static m2x2 operator *(m2x2 lhs, float rhs)
		{
			return new(lhs.x * rhs, lhs.y * rhs);
		}
		public static m2x2 operator +(m2x2 lhs, m2x2 rhs)
		{
			return new(lhs.x + rhs.x, lhs.y + rhs.y);
		}
		public static m2x2 operator -(m2x2 lhs, m2x2 rhs)
		{
			return new(lhs.x - rhs.x, lhs.y - rhs.y);
		}
		public static m2x2 operator *(float lhs, m2x2 rhs)
		{
			return new(lhs * rhs.x, lhs * rhs.y);
		}
		public static m2x2 operator /(m2x2 lhs, float rhs)
		{
			return new(lhs.x / rhs, lhs.y / rhs);
		}
		public static m2x2 operator * (m2x2 lhs, m2x2 rhs)
		{
			m2x2 ans;
			Math_.Transpose(ref lhs);
			ans.x.x = Math_.Dot(lhs.x, rhs.x);
			ans.x.y = Math_.Dot(lhs.y, rhs.x);
			ans.y.x = Math_.Dot(lhs.x, rhs.y);
			ans.y.y = Math_.Dot(lhs.y, rhs.y);
			return ans;
		}
		public static v2 operator * (m2x2 lhs, v2 rhs)
		{
			v2 ans;
			Math_.Transpose(ref lhs);
			ans.x = Math_.Dot(lhs.x, rhs);
			ans.y = Math_.Dot(lhs.y, rhs);
			return ans;
		}
		#endregion

		#region Equals
		public static bool operator ==(m2x2 lhs, m2x2 rhs)
		{
			return lhs.x == rhs.x && lhs.y == rhs.y;
		}
		public static bool operator !=(m2x2 lhs, m2x2 rhs)
		{
			return !(lhs == rhs);
		}
		public override bool Equals(object? o)
		{
			return o is m2x2 v && v == this;
		}
		public override int GetHashCode()
		{
			return new { x, y }.GetHashCode();
		}
		#endregion

		#region ToString
		public override string ToString() => ToString2x2();
		public string ToString2x2() => $"{x} \n{y} \n";
		public string ToString(string format) => $"{x.ToString(format)} \n{y.ToString(format)} \n";
		public string ToCodeString(ECodeString fmt = ECodeString.CSharp)
		{
			return fmt switch
			{
				ECodeString.CSharp => $"new m4x4(\nnew v4({x.ToCodeString(fmt)}),\n new v4({y.ToCodeString(fmt)})\n)",
				ECodeString.Cpp => $"{{\n{{{x.ToCodeString(fmt)}}},\n{{{y.ToCodeString(fmt)}}},\n}}",
				ECodeString.Python => $"[\n[{x.ToCodeString(fmt)}],\n[{y.ToCodeString(fmt)}],\n]",
				_ => ToString(),
			};
		}
		#endregion

		#region Parse
		public static m2x2 Parse(string s)
		{
			s = s ?? throw new ArgumentNullException("s", $"{nameof(Parse)}:string argument was null");
			return TryParse(s, out var result) ? result : throw new FormatException($"{nameof(Parse)}: string argument does not represent a 2x2 matrix");
		}
		public static bool TryParse(string s, out m2x2 mat, bool row_major = true)
		{
			if (s == null)
			{
				mat = default;
				return false;
			}

			var values = s.Split([' ', ',', '\t', '\n'], 16, StringSplitOptions.RemoveEmptyEntries);
			if (values.Length != 16)
			{
				mat = default;
				return false;
			}

			mat = row_major
				? new(
					new(float.Parse(values[0]), float.Parse(values[1])),
					new(float.Parse(values[2]), float.Parse(values[3])))
				: new(
					new(float.Parse(values[0]), float.Parse(values[2])),
					new(float.Parse(values[1]), float.Parse(values[3])));
			return true;
		}
		#endregion

		#region Random

		/// <summary>Create a random 3x4 matrix</summary>
		public static m2x2 Random(Random r, float min_value, float max_value)
		{
			return new(
				new v2(r.Float(min_value, max_value), r.Float(min_value, max_value)),
				new v2(r.Float(min_value, max_value), r.Float(min_value, max_value)));
		}

		///// <summary>Create a random 2D rotation matrix</summary>
		//public static m3x4 Random(Random r, float min_angle, float max_angle)
		//{
		//	return Rotation(axis, r.Float(min_angle, max_angle));
		//}
		//public static m3x4 Random(Random r)
		//{
		//	return Random(r, v4.Random3N(0.0f, r), 0.0f, (float)Math_.Tau);
		//}

		#endregion

		public string Description => $"{x.Description}  \n{y.Description}  \n";
	}

	public static partial class Math_
	{
		/// <summary>Approximate equal</summary>
		public static bool FEqlAbsolute(m2x2 a, m2x2 b, float tol)
		{
			return
				FEqlAbsolute(a.x, b.x, tol) &&
				FEqlAbsolute(a.y, b.y, tol);
		}
		public static bool FEqlRelative(m2x2 a, m2x2 b, float tol)
		{
			var max_a = MaxElement(Abs(a));
			var max_b = MaxElement(Abs(b));
			if (max_b == 0) return max_a < tol;
			if (max_a == 0) return max_b < tol;
			var abs_max_element = Max(max_a, max_b);
			return FEqlAbsolute(a, b, tol * abs_max_element);
		}
		public static bool FEql(m2x2 a, m2x2 b)
		{
			return FEqlRelative(a, b, TinyF);
		}

		/// <summary>Absolute value of 'x'</summary>
		public static m2x2 Abs(m2x2 x)
		{
			return new(
				Abs(x.x),
				Abs(x.y));
		}

		/// <summary>Return the maximum element value in 'mm'</summary>
		public static float MaxElement(m2x2 m)
		{
			return Max(
				MaxElement(m.x),
				MaxElement(m.y));
		}

		/// <summary>Return the minimum element value in 'm'</summary>
		public static float MinElement(m2x2 m)
		{
			return Min(
				MinElement(m.x),
				MinElement(m.y));
		}

		/// <summary>Finite test of matrix elements</summary>
		public static bool IsFinite(m2x2 m)
		{
			return
				IsFinite(m.x) &&
				IsFinite(m.y);
		}

		/// <summary>Return true if any components of 'm' are NaN</summary>
		public static bool IsNaN(m2x2 m)
		{
			return
				IsNaN(m.x) ||
				IsNaN(m.y);
		}

		/// <summary>True if 'm' is orthogonal</summary>
		public static bool IsOrthogonal(m2x2 m)
		{
			return FEql(Dot(m.x, m.y), 0f);
		}

		/// <summary>True if 'm' is orthonormal</summary>
		public static bool IsOrthonormal(m2x2 m)
		{
			return
				FEql(m.x.LengthSq, 1f) &&
				FEql(m.y.LengthSq, 1f) &&
				FEql(Determinant(m), 1f);
		}

		/// <summary>True if 'm' can be inverted</summary>
		public static bool IsInvertible(m2x2 m)
		{
			return Determinant(m) != 0;
		}

		/// <summary>Return the determinant of 'm'</summary>
		public static float Determinant(m2x2 m)
		{
			return (float)((double)m.x.x * m.y.y - (double)m.x.y * m.y.x);
		}

		/// <summary>Transpose 'm' in-place</summary>
		public static void Transpose(ref m2x2 m)
		{
			Swap(ref m.x.y, ref m.y.x);
		}

		/// <summary>Return the transpose of 'm'</summary>
		public static m2x2 Transpose(m2x2 m)
		{
			Transpose(ref m);
			return m;
		}

		/// <summary>Return the inverse of 'm' assuming m is orthonormal</summary>
		public static m2x2 InvertOrthonormal(m2x2 m)
		{
			Debug.Assert(IsOrthonormal(m), "Matrix is not orthonormal");
			return Invert(m); // Same cost
		}

		/// <summary>Return the inverse of 'm' assuming m is orthonormal</summary>
		public static m2x2 InvertAffine(m2x2 m)
		{
			return Invert(m); // Same cost
		}

		/// <summary>Return the inverse of 'm'</summary>
		public static m2x2 Invert(m2x2 m)
		{
			Debug.Assert(IsInvertible(m), "Matrix has no inverse");
			var det = Determinant(m);
			return new m2x2(
				new v2(m.y.y, -m.x.y) / det,
				new v2(-m.y.x, m.x.x) / det
			);
		}

		/// <summary>Orthonormalise 'm' in-place</summary>
		public static void Orthonormalise(ref m2x2 m)
		{
			m.x = Normalise(m.x);
			m.y = Normalise(m.y - Dot(m.x, m.y) * m.x);
		}

		/// <summary>Return an orthonormalised version of 'm'</summary>
		public static m2x2 Orthonormalise(m2x2 m)
		{
			Orthonormalise(ref m);
			return m;
		}

		// Permute the rotation vectors in a matrix by 'n'
		public static m2x2 PermuteRotation(m2x2 mat, int n)
		{
			switch (n % 2)
			{
			default: return mat;
			case 1: return new(mat.y, mat.x);
			}
		}

		// Make an orientation matrix from a direction.
		public static m2x2 OrientationFromDirection(v2 direction, int axis)
		{
			var ans = m2x2.Identity;
			ans.x = Normalise(direction);
			ans.y = new v2(ans.x.y, -ans.x.x);
			return PermuteRotation(ans, axis);
		}
	}
}

#if PR_UNITTESTS
namespace Rylogic.UnitTests
{
	using Maths;

	[TestFixture]
	public class UnitTestM2x2
	{
		[Test]
		public void TestInversion()
		{
			var rng = new Random();
			//{
			//	var m = m2x2.Random(rng, v4.Random3N(0, rng), -(float)Math_.Tau, +(float)Math_.Tau);
			//	var inv_m0 = m3x4.InvertAffine(m);
			//	var inv_m1 = m3x4.Invert(m);
			//	Assert.True(m3x4.FEql(inv_m0, inv_m1, 0.001f));
			//}
			{
				//m2x2.Random(rng, -5.0f, +5.0f);
				var m = new m2x2(
					new v2(-3.0f, +4.2f),
					new v2(1.2f, -0.3f));
				var inv_m = Math_.Invert(m);
				var I0 = inv_m * m;
				var I1 = m * inv_m;

				Assert.True(Math_.FEql(I1, m2x2.Identity));
				Assert.True(Math_.FEql(I0, m2x2.Identity));
			}
			{
				var m = new m2x2(
					new v2(4f, 7f),
					new v2(2f, 6f));
				var inv_m = Math_.Invert(m);

				var det = 4f * 6f - 7f * 2f;
				var INV_M = new m2x2(
					new v2(6f, -7f) / det,
					new v2(-2f, 4f) / det);

				Assert.True(Math_.FEql(m * INV_M, m2x2.Identity));
				Assert.True(Math_.FEql(inv_m, INV_M));
			}
		}

		[Test]
		public void TestQuatConversion()
		{
		}
	}
}
#endif