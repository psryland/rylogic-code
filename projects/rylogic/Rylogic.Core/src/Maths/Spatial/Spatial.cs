using System;
using System.Diagnostics;
using System.Runtime.InteropServices;
using Rylogic.Extn;

namespace Rylogic.Maths
{
	// Vector space tags
	public interface IVectorSpace { }
	public class Motion : IVectorSpace { }
	public class Force : IVectorSpace { }

	/// <summary>Spatial vector</summary>
	[Serializable]
	[StructLayout(LayoutKind.Sequential)]
	[DebuggerDisplay("Ang={ang} Lin={lin}")]
	public struct v8<T> where T: IVectorSpace
	{
		// Notes:
		//  - Generic types can't use explicit layout

		public v4 ang;
		public v4 lin;

		// Constructors
		public v8(v4 ang, v4 lin)
			: this()
		{
			this.ang = ang;
			this.lin = lin;
		}
		public v8(float wx, float wy, float wz, float vx, float vy, float vz)
			: this(new v4(wx, wy, wz, 0), new v4(vx, vy, vz, 0))
		{ }

		/// <summary>Reinterpret cast this vector to a different vector space</summary>
		public v8<U> Cast<U>() where U : IVectorSpace
		{
			return new v8<U>(ang, lin);
		}

		#region Random

		/// <summary>Return a random vector with components within the interval [min,max]</summary>
		public static v8<T> Random(float min, float max, Random r)
		{
			return new v8<T>(
				r.Float(min, max),
				r.Float(min, max),
				r.Float(min, max),
				r.Float(min, max),
				r.Float(min, max),
				r.Float(min, max));
		}

		/// <summary>Return a random vector with components within the intervals given by each component of min and max</summary>
		public static v8<T> Random(v8<T> min, v8<T> max, Random r)
		{
			return new v8<T>(
				r.Float(min.ang.x, max.ang.x),
				r.Float(min.ang.y, max.ang.y),
				r.Float(min.ang.z, max.ang.z),
				r.Float(min.lin.w, max.lin.w),
				r.Float(min.lin.w, max.lin.w),
				r.Float(min.lin.w, max.lin.w));
		}

		#endregion

		#region Equals
		public static bool operator == (v8<T> lhs, v8<T> rhs)
		{
			return lhs.ang == rhs.ang && lhs.lin == rhs.lin;
		}
		public static bool operator != (v8<T> lhs, v8<T> rhs)
		{
			return !(lhs == rhs);
		}
		public override bool Equals(object? o)
		{
			return o is v8<T> v && v == this;
		}
		public override int GetHashCode()
		{
			return new { ang, lin }.GetHashCode();
		}
		#endregion
	}

	/// <summary>Spatial matrix</summary>
	[Serializable]
	[StructLayout(LayoutKind.Sequential)]
	[DebuggerDisplay("{m00} {m01} {m10} {m11}")]
	public struct m6x8<T,U> where T : IVectorSpace where U : IVectorSpace
	{
		public m3x4 m00;
		public m3x4 m01;
		public m3x4 m10;
		public m3x4 m11;

		// Constructors
		public m6x8(m3x4 m00, m3x4 m01, m3x4 m10, m3x4 m11)
			: this()
		{
			this.m00 = m00;
			this.m01 = m01;
			this.m10 = m10;
			this.m11 = m11;
		}

		/// <summary>Reinterpret cast this matrix</summary>
		public m6x8<V,W> Cast<V,W>() where V: IVectorSpace where W:IVectorSpace
		{
			return new m6x8<V,W>(m00, m01, m10, m11);
		}

		// Operators
		public static m6x8<T,U> operator - (m6x8<T, U> m)
		{
			return new m6x8<T, U>(-m.m00, -m.m01, -m.m10, -m.m11);
		}
		public static v8<U> operator * (m6x8<T, U> lhs, v8<T> rhs)
		{
			// [m11*a + m12*b] = [m11 m12] [a]
			// [m21*a + m22*b]   [m21 m22] [b]
			return new v8<U>(
				lhs.m00 * rhs.ang + lhs.m01 * rhs.lin,
				lhs.m10 * rhs.ang + lhs.m11 * rhs.lin);
		}
		public static m6x8<T, U> operator * (m6x8<T, U> lhs, m6x8<T, U> rhs)
		{
			// This is only valid if T == U.
			//' [a11 a12] [b11 b12] = [a11*b11 + a12*b21 a11*b12 + a12*b22]
			//' [a21 a22] [b21 b22]   [a21*b11 + a22*b21 a21*b12 + a22*b22]
			return new m6x8<T,U>(
				lhs.m00 * rhs.m00 + lhs.m01 * rhs.m10, lhs.m00 * rhs.m01 + lhs.m01 * rhs.m11,
				lhs.m10 * rhs.m00 + lhs.m11 * rhs.m10, lhs.m10 * rhs.m01 + lhs.m11 * rhs.m11);
		}

		#region Equals
		public static bool operator == (m6x8<T,U> lhs, m6x8<T,U> rhs)
		{
			return
				lhs.m00 == rhs.m00 &&
				lhs.m01 == rhs.m01 &&
				lhs.m10 == rhs.m10 &&
				lhs.m11 == rhs.m11;
		}
		public static bool operator != (m6x8<T,U> lhs, m6x8<T,U> rhs)
		{
			return !(lhs == rhs);
		}
		public override bool Equals(object? o)
		{
			return o is m6x8<T,U> m && m == this;
		}
		public override int GetHashCode()
		{
			return new { m00, m01, m10, m11 }.GetHashCode();
		}
		#endregion
	}

	/// <summary>Specialised spatial matrix for transforms</summary>
	[Serializable]
	[StructLayout(LayoutKind.Sequential)]
	[DebuggerDisplay("{a2b}")]
	public struct Txfm6x8<T> where T : IVectorSpace
	{
		public m4x4 a2b;

		public Txfm6x8(m4x4 a2b)
			: this()
		{
			this.a2b = a2b;
		}

		/// <summary>Reinterpret cast this matrix</summary>
		public Txfm6x8<U> Cast<U>() where U : IVectorSpace
		{
			return new Txfm6x8<U>(a2b);
		}

		// Operators
		public static v8<Motion> operator * (Txfm6x8<T> lhs, v8<Motion> rhs)
		{
			// Spatial transform for motion vectors:
			//' a2b = [E 0] * [ 1  0] = [ E    0]
			//'       [0 E]   [-rx 1]   [-E*rx E]
			//' (E = rotation matrix 3x3)
			// So:
			//  a2b * v = [ E    0] * [v.ang] = [E*v.ang             ]
			//            [-E*rx E]   [v.lin]   [E*v.lin - E*rx*v.ang]
			return new v8<Motion>(
				lhs.a2b.rot * rhs.ang,
				lhs.a2b.rot * rhs.lin - lhs.a2b.rot * Math_.Cross(lhs.a2b.pos, rhs.ang));
		}
		public static v8<Force> operator * (Txfm6x8<T> lhs, v8<Force> rhs)
		{
			// Spatial transform for force vectors:
			//' a2b = [E 0] * [1 -rx] = [E -E*rx]
			//'       [0 E]   [0  1 ]   [0  E   ]
			//' (E = rotation matrix 3x3)
			// So:
			//  a2b * v = [E -E*rx] * [v.ang] = [E*v.ang - E*rx*v.lin]
			//            [0  E   ]   [v.lin]   [E*v.lin             ]
			return new v8<Force>(
				lhs.a2b.rot * rhs.ang - lhs.a2b.rot * Math_.Cross(lhs.a2b.pos, rhs.lin),
				lhs.a2b.rot * rhs.lin);
		}

		#region Equals
		public static bool operator ==(Txfm6x8<T> lhs, Txfm6x8<T> rhs)
		{
			return lhs.a2b == rhs.a2b;
		}
		public static bool operator !=(Txfm6x8<T> lhs, Txfm6x8<T> rhs)
		{
			return !(lhs == rhs);
		}
		public override bool Equals(object? o)
		{
			return o is Txfm6x8<T> m && m == this;
		}
		public override int GetHashCode()
		{
			return new { a2b }.GetHashCode();
		}
		#endregion
	}

	/// <summary>Maths operations</summary>
	public static partial class Math_
	{
		/// <summary>Approximate equal</summary>
		public static bool FEqlRelative<T>(v8<T> lhs, v8<T> rhs, float tol) where T: IVectorSpace
		{
			return
				FEqlRelative(lhs.ang, rhs.ang, tol) &&
				FEqlRelative(lhs.lin, rhs.lin, tol);
		}
		public static bool FEqlRelative<T,U>(m6x8<T, U> lhs, m6x8<T, U> rhs, float tol) where T : IVectorSpace where U : IVectorSpace
		{
			return
				FEqlRelative(lhs.m00, rhs.m00, tol) &&
				FEqlRelative(lhs.m01, rhs.m01, tol) &&
				FEqlRelative(lhs.m10, rhs.m10, tol) &&
				FEqlRelative(lhs.m11, rhs.m11, tol);
		}
		public static bool FEql<T>(v8<T> lhs, v8<T> rhs) where T: IVectorSpace
		{
			return
				FEql(lhs.ang, rhs.ang) &&
				FEql(lhs.lin, rhs.lin);
		}
		public static bool FEql<T,U>(m6x8<T, U> lhs, m6x8<T, U> rhs) where T : IVectorSpace where U : IVectorSpace
		{
			return
				FEql(lhs.m00, rhs.m00) &&
				FEql(lhs.m01, rhs.m01) &&
				FEql(lhs.m10, rhs.m10) &&
				FEql(lhs.m11, rhs.m11);
		}

		///<summary>
		/// Spatial dot product.
		/// The dot product is only defined for Dot(v8m,v8f) and Dot(v8f,v8m)
		/// e.g Dot(force, velocity) == power delivered</summary>
		public static float Dot(v8<Motion> lhs, v8<Force> rhs)
		{
			// v8m and v8f are vectors in the dual spaces M and F
			// A property of dual spaces is dot(m,f) = transpose(m)*f
			return Dot(lhs.ang, rhs.ang) + Dot(lhs.lin, rhs.lin);
		}
		public static float Dot(v8<Force> lhs, v8<Motion> rhs)
		{
			return Dot(rhs, lhs);
		}

		/// <summary>
		/// Spatial cross product.
		/// There are two cross product operations, one for motion vectors and one for forces</summary>
		public static v8<Motion> Cross<T>(v8<T> lhs, v8<Motion> rhs) where T: IVectorSpace
		{
			return new v8<Motion>(Cross(lhs.ang, rhs.ang), Cross(lhs.ang, rhs.lin) + Cross(lhs.lin, rhs.ang));
		}
		public static v8<Force> Cross<T>(v8<T> lhs, v8<Force> rhs) where T: IVectorSpace
		{
			return new v8<Force>(Cross(lhs.ang, rhs.ang) + Cross(lhs.lin, rhs.lin), Cross(lhs.ang, rhs.lin));
		}

		/// <summary>
		/// The spatial cross product matrix for 'a', for use with motion vectors.
		///' i.e. b = a x m = CPM(a) * m, where m is a motion vector</summary>
		public static m6x8<Motion,Motion> CPM(v8<Motion> a)
		{
			var cx_ang = CPM(a.ang);
			var cx_lin = CPM(a.lin);
			return new m6x8<Motion, Motion>(cx_ang, m3x4.Zero, cx_lin, cx_ang);
		}

		/// <summary>
		/// The spatial cross product matrix for 'a', for use with force vectors.
		/// i.e. b = a x* f = CPM(a) * f, where f is a force vector</summary>
		public static m6x8<Force, Force> CPM(v8<Force> a)
		{
			var cx_ang = CPM(a.ang);
			var cx_lin = CPM(a.lin);
			return new m6x8<Force, Force>(cx_ang, cx_lin, m3x4.Zero, cx_ang);
		}

		/// <summary>Return the transpose of a spatial matrix</summary>
		public static void Transpose<T, U>(ref m6x8<T, U> m) where T : IVectorSpace where U : IVectorSpace
		{
			Transpose(ref m.m00);
			Transpose(ref m.m01);
			Transpose(ref m.m10);
			Transpose(ref m.m11);
			Swap(ref m.m01, ref m.m10);
		}
		public static m6x8<T, U> Transpose<T, U>(m6x8<T, U> m) where T : IVectorSpace where U : IVectorSpace
		{
			Transpose(ref m);
			return m;
		}
	}
}

#if PR_UNITTESTS
namespace Rylogic.UnitTests
{
	using Maths;
	[TestFixture]
	public class UnitTestV8
	{
		[Test]
		public void DotProduct()
		{
			var a = new v8<Motion>(+1, +2, +3, +4, +5, +6);
			var b = new v8<Force>(-1, -2, -3, -4, -5, -6);
			var pwr0 = Math_.Dot(a, b);
			var pwr1 = Math_.Dot(b, a);
			Assert.True(Math_.FEql(pwr0, -91));
			Assert.True(Math_.FEql(pwr1, -91));
		}
		[Test]
		public void CrossProduct()
		{
			{
				var v0 = new v8<Motion>(1, 1, 1, 2, 2, 2);
				var v1 = new v8<Motion>(-1, -2, -3, -4, -5, -6);
				var r0 = Math_.Cross(v0, v1);
				var r1 = Math_.CPM(v0) * v1;
				Assert.True(Math_.FEql(r0, r1));
			}
			{
				var v0 = new v8<Force>(1, 1, 1, 2, 2, 2);
				var v1 = new v8<Force>(-1, -2, -3, -4, -5, -6);
				var r0 = Math_.Cross(v0, v1);
				var r1 = Math_.CPM(v0) * v1;
				Assert.True(Math_.FEql(r0, r1));
			}
			{// Test: vx* == -Transpose(vx)
				var rng = new Random(321);
				var v = v8<IVectorSpace>.Random(-5f, +5f, rng);

				var m0 = Math_.CPM(v.Cast<Motion>()); // vx
				var m1 = Math_.CPM(v.Cast<Force>());  // vx*
				var m2 = Math_.Transpose(m1);
				var m3 = (-m2).Cast<Motion,Motion>();
				Assert.True(Math_.FEql(m0, m3));
			}
		}
	}
}
#endif