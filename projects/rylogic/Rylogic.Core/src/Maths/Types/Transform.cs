//***************************************************
// Matrix4x4
//  Copyright (c) Rylogic Ltd 2008
//***************************************************

using System;
using System.Diagnostics;
using System.Runtime.InteropServices;

namespace Rylogic.Maths
{
	/// <summary>Rotation, Position, Scale transform</summary>
	[Serializable]
	[StructLayout(LayoutKind.Explicit)]
	[DebuggerDisplay("{Description,nq}")]
	public struct Xform
	{
		[FieldOffset(0)] public v4 pos;
		[FieldOffset(16)] public Quat rot;
		[FieldOffset(32)] public v4 scl;

		/// <summary>Constructors</summary>
		public Xform(v4 pos, Quat rot, v4 scl)
		{
			this.pos = pos;
			this.rot = rot;
			this.scl = scl;
		}
		public Xform(v4 pos, Quat rot)
			:this(pos, rot, v4.One)
		{}
		public Xform(v4 pos, m3x4 rot)
			:this()
		{
			var (r, s) = Math_.Normalise(rot);
			this.pos = pos;
			this.rot = new Quat(r);
			this.scl = s.w1;
		}
		public Xform(m4x4 m)
			:this(m.pos, m.rot)
		{}

		#region Statics
		public readonly static Xform Zero = new(v4.Zero, Quat.Zero, v4.Zero);
		public readonly static Xform Identity = new(v4.Origin, Quat.Identity, v4.Zero);
		#endregion

		#region Operators
		public static Xform operator +(Xform rhs)
		{
			return rhs;
		}
		public static Xform operator -(Xform rhs)
		{
			return new(-rhs.pos, -rhs.rot, -rhs.scl);
		}
		public static Xform operator *(Xform lhs, Xform rhs)
		{
			return new Xform(
				lhs.rot * (lhs.scl * rhs.pos.w0) + lhs.pos,
				lhs.rot * rhs.rot,
				lhs.scl * rhs.scl);
		}
		public static v4 operator *(Xform lhs, v4 rhs)
		{
			return
				lhs.rot * (lhs.scl * rhs.w0) +
				lhs.pos * rhs.w;
		}
		public static Quat operator *(Xform lhs, Quat rhs)
		{
			return lhs.rot * rhs;
		}
		#endregion

		#region Equals
		public static bool operator == (Xform lhs, Xform rhs)
		{
			return lhs.pos == rhs.pos && lhs.rot == rhs.rot && lhs.scl == rhs.scl;
		}
		public static bool operator != (Xform lhs, Xform rhs)
		{
			return !(lhs == rhs);
		}
		public override bool Equals(object? o)
		{
			return o is Xform m && m == this;
		}
		public override int GetHashCode()
		{
			return new { pos, rot, scl }.GetHashCode();
		}
		#endregion

		#region ToString
		public override string ToString() => ToStringRPS();
		public string ToStringRPS(string delim = "\n") => $"{pos} {delim}{rot} {delim}{scl} {delim}";
		public string ToStringRP(string delim = "\n") => $"{pos} {delim}{rot} {delim}";
		public string ToString(string format, string delim = "\n") => $"{pos.ToString(format)} {delim}{rot.ToString(format)} {delim}{scl.ToString(format)} {delim}";
		public string ToCodeString(ECodeString fmt = ECodeString.CSharp)
		{
			return fmt switch
			{
				ECodeString.CSharp => $"new Xform(\nnew v4({pos.ToCodeString(fmt)}),\n new Quat({rot.ToCodeString(fmt)}),\n new v4({scl.ToCodeString(fmt)}),\n)",
				ECodeString.Cpp => $"{{\n{{{pos.ToCodeString(fmt)}}},\n{{{rot.ToCodeString(fmt)}}},\n{{{scl.ToCodeString(fmt)}}},\n}}",
				ECodeString.Python => $"[\n[{pos.ToCodeString(fmt)}],\n[{rot.ToCodeString(fmt)}],\n[{scl.ToCodeString(fmt)}],\n]",
				_ => ToString(),
			};
		}
		#endregion

		#region Parse
		public static Xform Parse(string s)
		{
			s = s ?? throw new ArgumentNullException("s", $"{nameof(Parse)}:string argument was null");
			return TryParse(s, out var result) ? result : throw new FormatException($"{nameof(Parse)}: string argument does not represent a 4x4 matrix");
		}
		public static bool TryParse(string s, out Xform xform)
		{
			if (s == null)
			{
				xform = default;
				return false;
			}

			var values = s.Split([' ', ',', '\t', '\n'], 16, StringSplitOptions.RemoveEmptyEntries);
			if (values.Length != 16)
			{
				xform = default;
				return false;
			}

			xform = new(
					new v4(float.Parse(values[0]), float.Parse(values[1]), float.Parse(values[2]), float.Parse(values[3])),
					new Quat(float.Parse(values[4]), float.Parse(values[5]), float.Parse(values[6]), float.Parse(values[7])),
					new v4(float.Parse(values[8]), float.Parse(values[9]), float.Parse(values[10]), float.Parse(values[11])));
			return true;
		}
		#endregion

		#region Random

		// Create a random transform
		public static Xform Random(Random r)
		{
			return Random(v4.Origin, 1.0f, r);
		}
		public static Xform Random(v4 centre, float radius, Random r)
		{
			return Random(centre, radius, v2.One, r);
		}
		public static Xform Random(v4 centre, float radius, v2 scale_range, Random r)
		{
			return new(v4.Random3(centre, radius, 1, r), Quat.Random(r), v4.Random3(scale_range.x, scale_range.y, 1, r));
		}

		#endregion

		/// <summary></summary>
		public string Description => $"{pos.Description}  \n{rot.Description}  \n{scl.Description}  \n";
	}

	public static partial class Math_
	{
		/// <summary>Approximate equal</summary>
		public static bool FEqlAbsolute(Xform a, Xform b, float tol)
		{
			return
				FEqlRelative(a.rot, b.rot, tol) &&
				FEqlAbsolute(a.pos, b.pos, tol) &&
				FEqlAbsolute(a.scl, b.scl, tol);
		}
		public static bool FEqlRelative(Xform a, Xform b, float tol)
		{
			return
				FEqlRelative(a.rot, b.rot, tol) &&
				FEqlRelative(a.pos, b.pos, tol) &&
				FEqlRelative(a.scl, b.scl, tol);
		}
		public static bool FEql(Xform lhs, Xform rhs)
		{
			return FEqlRelative(lhs, rhs, TinyF);
		}

		/// <summary>Invert a transform</summary>
		public static Xform Invert(Xform xform)
		{
			var inv_rot = ~xform.rot;
			var inv_scl = 1f / xform.scl;
			var inv_pos = inv_rot * (inv_scl * -xform.pos.w0);
			return new Xform(inv_pos.w1, inv_rot, inv_scl);
		}
	}
}

#if PR_UNITTESTS
namespace Rylogic.UnitTests
{
	using Maths;

	[TestFixture]
	public class UnitTestXform
	{
		private Random m_rng = new(12345);

		[Test]
		public void ConstructionRoundTrip()
		{
			var xf1 = Xform.Random(new v4(1,1,1,1), 2f, new v2(0.2f, 1.5f), m_rng);
			var m = new m4x4(xf1);
			var xf2 = new Xform(m);

			Assert.True(Math_.FEql(xf1, xf2));
		}

		[Test]
		public void Multiply()
		{
			var xf1 = Xform.Random(m_rng);
			var xf2 = Xform.Random(m_rng);
			var xf3 = xf1 * xf2;

			var m1 = new m4x4(xf1);
			var m2 = new m4x4(xf2);
			var m3 = m1 * m2;

			var xf3_from_m = new Xform(m3);
			var m3_from_xf = new m4x4(xf3);

			Assert.True(Math_.FEql(xf3, xf3_from_m));
			Assert.True(Math_.FEql(m3, m3_from_xf));
		}

		[Test]
		public void MultiplyVector()
		{
			var v0 = new v4(1, 2, 3, 1);
			var v1 = new v4(1, 2, 3, 0);

			var xf = Xform.Random(m_rng);
			var m = new m4x4(xf);

			var r0 = xf * v0;
			var r1 = xf * v1;

			var R0 = m * v0;
			var R1 = m * v1;

			Assert.True(Math_.FEql(r0, R0));
			Assert.True(Math_.FEql(r1, R1));
		}

		[Test]
		public void MultiplyQuaternion()
		{
			var q = Quat.Random(m_rng);
			var xf = Xform.Random(m_rng);
			var m = new m4x4(xf);
			var r = xf * q;
			var R = new Quat(m.rot) * q;
			
			Assert.True(Math_.FEql(r, R));
		}

		[Test]
		public void Inversion()
		{
			var xf = Xform.Random(m_rng);
			var m = new m4x4(xf);

			var xf_inv = Math_.Invert(xf);
			var m_inv = Math_.Invert(m);

			var r = new m4x4(xf_inv);
			var R = m_inv;
			Assert.True(Math_.FEql(r, R));

			var xf2 = Math_.Invert(xf_inv);
			var m2 = Math_.Invert(m_inv);
			Assert.True(Math_.FEql(xf, xf2));
			Assert.True(Math_.FEql(m, m2));
		}
	}
}
#endif