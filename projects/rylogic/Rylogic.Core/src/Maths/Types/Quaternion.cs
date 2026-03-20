//***************************************************
// Quaternion
//  Copyright (c) Rylogic Ltd 2008
//***************************************************

using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Runtime.InteropServices;
using System.Numerics;
using Rylogic.Extn;

namespace Rylogic.Maths
{
	/// <summary>Quaternion functions. Note: a quaternion is a v4</summary>
	[Serializable]
	[StructLayout(LayoutKind.Explicit)]
	[DebuggerDisplay("{Description,nq}")]
	public struct Quat
	{
		[FieldOffset(0)] public float x;
		[FieldOffset(4)] public float y;
		[FieldOffset(8)] public float z;
		[FieldOffset(12)] public float w;
		[FieldOffset(0)] public v4 xyzw; // (same name as the C++ version)
		[FieldOffset(0)] public v3 xyz;  // (same name as the C++ version)

		public Quat(float x)
			: this()
		{
			this.x = x;
			this.y = x;
			this.z = x;
			this.w = x;
		}
		public Quat(float x, float y, float z, float w)
			: this()
		{
			this.x = x;
			this.y = y;
			this.z = z;
			this.w = w;
		}
		public Quat(v4 vec)
			: this()
		{
			this.xyzw = vec;
		}
		public Quat(v3 vec)
			: this(vec.w0)
		{ }

		/// <summary>Create a quaternion from an axis and an angle</summary>
		public Quat(v4 axis, float angle)
			: this()
		{
			var s = (float)Math.Sin(0.5 * angle);
			x = s * axis.x;
			y = s * axis.y;
			z = s * axis.z;
			w = (float)Math.Cos(0.5 * angle);
		}

		/// <summary>Create a quaternion from Euler angles. Order is: roll, pitch, yaw (to match DirectX)</summary>
		public Quat(float pitch, float yaw, float roll)
			: this()
		{
			// nicked from 'XMQuaternionRotationRollPitchYaw'
			float cos_p = (float)Math.Cos(pitch * 0.5), sin_p = (float)Math.Sin(pitch * 0.5);
			float cos_y = (float)Math.Cos(yaw * 0.5), sin_y = (float)Math.Sin(yaw * 0.5);
			float cos_r = (float)Math.Cos(roll * 0.5), sin_r = (float)Math.Sin(roll * 0.5);
			x = sin_p * cos_y * cos_r + cos_p * sin_y * sin_r;
			y = cos_p * sin_y * cos_r - sin_p * cos_y * sin_r;
			z = cos_p * cos_y * sin_r - sin_p * sin_y * cos_r;
			w = cos_p * cos_y * cos_r + sin_p * sin_y * sin_r;
		}

		/// <summary>Create a quaternion from a rotation matrix</summary>
		public Quat(m3x4 m)
			: this()
		{
			Debug.Assert(Math_.IsOrthonormal(m), "Only orientation matrices can be converted into quaternions");
			var trace = m.x.x + m.y.y + m.z.z;
			if (trace >= 0.0f)
			{
				var s = 0.5f / (float)Math.Sqrt(1.0f + trace);
				x = (m.y.z - m.z.y) * s;
				y = (m.z.x - m.x.z) * s;
				z = (m.x.y - m.y.x) * s;
				w = (0.25f / s);
			}
			else if (m.x.x > m.y.y && m.x.x > m.z.z)
			{
				var s = 0.5f / (float)Math.Sqrt(1.0f + m.x.x - m.y.y - m.z.z);
				x = (0.25f / s);
				y = (m.x.y + m.y.x) * s;
				z = (m.z.x + m.x.z) * s;
				w = (m.y.z - m.z.y) * s;
			}
			else if (m.y.y > m.z.z)
			{
				var s = 0.5f / (float)Math.Sqrt(1.0f - m.x.x + m.y.y - m.z.z);
				x = (m.x.y + m.y.x) * s;
				y = (0.25f / s);
				z = (m.y.z + m.z.y) * s;
				w = (m.z.x - m.x.z) * s;
			}
			else
			{
				var s = 0.5f / (float)Math.Sqrt(1.0f - m.x.x - m.y.y + m.z.z);
				x = (m.z.x + m.x.z) * s;
				y = (m.y.z + m.z.y) * s;
				z = (0.25f / s);
				w = (m.x.y - m.y.x) * s;
			}
		}

		/// <summary>Construct a quaternion representing a rotation from 'from' to 'to'</summary>
		public Quat(v4 from, v4 to)
			: this()
		{
			var d = Math_.Dot(from.xyz, to.xyz);
			var s = (float)Math.Sqrt(from.xyz.LengthSq * to.xyz.LengthSq) + d;
			var axis = Math_.Cross(from, to);

			// Vectors are aligned, 180 degrees apart, or one is zero
			if (Math_.FEql(s, 0))
			{
				s = 0.0f;
				axis =
					from.xyz.LengthSq > Math_.TinyF ? Math_.Perpendicular(from) :
					to.xyz.LengthSq > Math_.TinyF ? Math_.Perpendicular(to) :
					v4.ZAxis;
			}

			xyzw = Math_.Normalise(new v4(axis.x, axis.y, axis.z, s));
		}

		/// <summary>Reinterpret a vector as a quaternion</summary>
		public static Quat From(v4 vec)
		{
			return new(vec.x, vec.y, vec.z, vec.w);
		}

		/// <summary>Get/Set components by index</summary>
		public float this[int i]
		{
			readonly get
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
			readonly get => this[(int)i];
			set => this[(int)i] = value;
		}

		/// <summary>Implicit conversion to System.Numerics</summary>
		public static implicit operator Quat(Quaternion q)
		{
			return new Quat(q.X, q.Y, q.Z, q.W);
		}
		public static implicit operator Quaternion(Quat q)
		{
			return new Quaternion(q.x, q.y, q.z, q.w);
		}

		/// <summary>Length</summary>
		public readonly float LengthSq => x * x + y * y + z * z + w * w;
		public readonly float Length => (float)Math.Sqrt(LengthSq);
		public bool IsNormalised => Math_.FEql(LengthSq, 1.0f);

		/// <summary>Get the axis component of the quaternion (normalised)</summary>
		public v4 Axis => Math_.Normalise(xyzw.w0, v4.Zero);

		// Return the angle of rotation about 'Axis()'
		public float Angle => (float)Math.Acos(CosAngle);

		// Return the cosine of the angle of rotation about 'Axis()'
		public float CosAngle
		{
			get
			{
				// Trig:
				//' w == cos(θ/2)
				//' cos²(θ/2) = 0.5 * (1 + cos(θ))
				//' w² == cos²(θ/2) == 0.5 * (1 + cos(θ))
				//' cos(θ) = 2w² - 1

				// This is always the smallest arc
				return Math_.Clamp(2f * Math_.Sqr(w) - LengthSq, -1f, +1f);
			}
		}

		// Return the sine of the angle of rotation about 'Axis()'
		public float SinAngle
		{
			get
			{
				// Trig:
				//'  w == cos(θ/2)
				//'  sin(θ) = 2 * sin(θ/2) * cos(θ/2)

				// The sign is determined by the sign of w (which represents cos(θ/2))
				var sin_half_angle = xyz.Length;
				return 2f * sin_half_angle * w;
			}
		}

		/// <summary>ToArray()</summary>
		public float[] ToArray()
		{
			return [x, y, z, w];
		}

		/// <summary>Component add</summary>
		public static Quat CompAdd(Quat lhs, Quat rhs)
		{
			return new(lhs.xyzw + rhs.xyzw);
		}

		/// <summary>Component multiply</summary>
		public Quat CompMul(Quat lhs, float rhs)
		{
			return new(lhs.xyzw * rhs);
		}
		public Quat CompMul(Quat lhs, double rhs)
		{
			return CompMul(lhs, (float)rhs);
		}
		public Quat CompMul(Quat lhs, Quat rhs)
		{
			return new(lhs.xyzw * rhs.xyzw);
		}

		#region Statics
		public readonly static Quat Zero = new(0, 0, 0, 0);
		public readonly static Quat Identity = new(0, 0, 0, 1);
		#endregion

		#region Operators
		public static Quat operator +(Quat q)
		{
			return q;
		}
		public static Quat operator -(Quat q)
		{
			return new(-q.x, -q.y, -q.z, -q.w); // Note: Not conjugate
		}
		public static Quat operator ~(Quat q)
		{
			return new(-q.x, -q.y, -q.z, q.w);
		}
		public static Quat operator +(Quat lhs, Quat rhs)
		{
			return new(lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z, lhs.w + rhs.w);
		}
		public static Quat operator -(Quat lhs, Quat rhs)
		{
			return new(lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z, lhs.w - rhs.w);
		}
		public static Quat operator *(Quat lhs, float rhs)
		{
			return new(lhs.x * rhs, lhs.y * rhs, lhs.z * rhs, lhs.w * rhs);
		}
		public static Quat operator *(float lhs, Quat rhs)
		{
			return rhs * lhs;
		}
		public static Quat operator /(Quat lhs, float rhs)
		{
			return new(lhs.x / rhs, lhs.y / rhs, lhs.z / rhs, lhs.w / rhs);
		}
		public static Quat operator *(Quat lhs, Quat rhs)
		{
			// Quaternion multiply. Same semantics at matrix multiply
			return new(
				lhs.w * rhs.x + lhs.x * rhs.w + lhs.y * rhs.z - lhs.z * rhs.y,
				lhs.w * rhs.y - lhs.x * rhs.z + lhs.y * rhs.w + lhs.z * rhs.x,
				lhs.w * rhs.z + lhs.x * rhs.y - lhs.y * rhs.x + lhs.z * rhs.w,
				lhs.w * rhs.w - lhs.x * rhs.x - lhs.y * rhs.y - lhs.z * rhs.z);
		}
		public static v4 operator *(Quat lhs, v4 rhs)
		{
			// Quaternion rotate. Same semantics at matrix multiply
			return Math_.Rotate(lhs, rhs);
		}
		#endregion

		#region Equals
		public static bool operator ==(Quat lhs, Quat rhs)
		{
			return lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z && lhs.w == rhs.w;
		}
		public static bool operator !=(Quat lhs, Quat rhs)
		{
			return !(lhs == rhs);
		}
		public override bool Equals(object? o)
		{
			return o is Quat q && q == this;
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
		public static Quat Parse(string s)
		{
			if (s == null)
				throw new ArgumentNullException(nameof(s), "Quat.Parse() string argument was null");

			var values = s.Split([' ', ',', '\t'], 4);
			if (values.Length != 4)
				throw new FormatException("Quat.Parse() string argument does not represent a 4 component quaternion");

			return new(
				float.Parse(values[0]),
				float.Parse(values[1]),
				float.Parse(values[2]),
				float.Parse(values[3]));
		}
		public static bool TryParse(string s, out Quat quat)
		{
			quat = Identity;
			if (s == null)
				return false;

			var values = s.Split([' ', ',', '\t'], 4);
			return
				values.Length == 4 &&
				float.TryParse(values[0], out quat.x) &&
				float.TryParse(values[1], out quat.y) &&
				float.TryParse(values[2], out quat.z) &&
				float.TryParse(values[3], out quat.w);
		}
		public static Quat? TryParse(string s)
		{
			return TryParse(s, out var quat) ? quat : null;
		}
		#endregion

		#region Random

		/// <summary>Construct a random quaternion rotation</summary>
		public static Quat Random(v4 axis, float min_angle, float max_angle, Random r)
		{
			return new(axis, r.Float(min_angle, max_angle));
		}

		/// <summary>Construct a random quaternion rotation</summary>
		public static Quat Random(float min_angle, float max_angle, Random r)
		{
			return new(v4.Random3N(0f, r), r.Float(min_angle, max_angle));
		}

		/// <summary>Construct a random quaternion rotation</summary>
		public static Quat Random(Random r)
		{
			return new(v4.Random3N(0f, r), r.Float(0f, (float)Math_.Tau));
		}

		#endregion

		/// <summary></summary>
		public string Description => IsNormalised
			? $"{x}  {y}  {z}  {w}  //Len({Length}) Axis({Axis.xyz}) Ang({Angle})"
			: $"{x}  {y}  {z}  {w}  //Len({Length}) <unnormalised>";
	}

	public static partial class Math_
	{
		/// <summary>Approximate equal. Note: q == -q</summary>
		public static bool FEqlRelative(Quat lhs, Quat rhs, float tol)
		{
			return
				FEqlRelative(lhs.xyzw, +rhs.xyzw, tol) ||
				FEqlRelative(lhs.xyzw, -rhs.xyzw, tol);
		}
		public static bool FEql(Quat lhs, Quat rhs)
		{
			return FEqlRelative(lhs, rhs, TinyF);
		}

		/// <summary>Return true if all components of 'vec' are finite</summary>
		public static bool IsFinite(Quat q)
		{
			return IsFinite(q.x) && IsFinite(q.y) && IsFinite(q.z) && IsFinite(q.w);
		}

		/// <summary>Return true if any components of 'vec' are NaN</summary>
		public static bool IsNaN(Quat q)
		{
			return IsNaN(q.x) || IsNaN(q.y) || IsNaN(q.z) || IsNaN(q.w);
		}

		/// <summary>Normalise a quaternion to unit length</summary>
		public static Quat Normalise(Quat q)
		{
			return new(Normalise(q.xyzw));
		}
		public static Quat Normalise(Quat q, Quat def)
		{
			return new(Normalise(q.xyzw, def.xyzw));
		}

		/// <summary>Returns the value of 'cos(theta / 2)', where 'theta' is the angle between 'a' and 'b'</summary>
		public static float CosHalfAngle(Quat a, Quat b)
		{
			// The relative orientation between 'a' and 'b' is given by z = 'a * conj(b)'
			// where operator * is a quaternion multiply. The 'w' component of a quaternion
			// multiply is given by: q.w = a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z;
			// which is the same as q.w = Dot4(a,b) since conjugate negates the x,y,z
			// components of 'b'. Remember: q.w = Cos(theta/2)
			return Dot(a.xyzw, b.xyzw);
		}

		/// <summary>Returns the smallest angle between two quaternions (in radians, [0, tau/2])</summary>
		public static float Angle(Quat a, Quat b)
		{
			// q.w = Cos(theta/2)
			// Note: cos(A) = 2 * cos²(A/2) - 1
			//  and: acos(A) = 0.5 * acos(2A² - 1), for A in [0, tau/2]
			// Using the 'acos(2A² - 1)' form always returns the smallest angle
			var cos_half_ang = CosHalfAngle(a, b);
			return
				cos_half_ang > 1.0f - Math_.TinyF ? 0f :
				cos_half_ang > 0 ? 2 * (float)Math.Acos(Clamp(cos_half_ang, -1, +1)) : // better precision
				(float)Math.Acos(Clamp(2 * Sqr(cos_half_ang) - 1, -1, +1));
		}

		/// <summary>Logarithm map of quaternion to tangent space at identity. Converts a quaternion into a length-scaled direction, where length is the angle of rotation</summary>
		public static v4 LogMap(Quat q)
		{
			// Quat = [u.Sin(A/2), Cos(A/2)]
			var cos_half_ang = Math_.Clamp(q.w, -1.0f, +1.0f); // [0, tau]
			var sin_half_ang = q.xyzw.w0.Length; // Don't use 'sqrt(1 - w*w)', it's not float noise accurate enough when w ~= +/-1
			var ang_by_2 = Math.Acos(cos_half_ang); // By convention, log space uses Length = A/2
			return Math.Abs(sin_half_ang) > Math_.TinyD
				? q.xyzw.w0 * (float)(ang_by_2 / sin_half_ang)
				: q.xyzw.w0;
		}

		/// <summary>Exponential map of tangent space at identity to quaternion. Converts a length-scaled direction to a quaternion.</summary>
		public static Quat ExpMap(v3 v)
		{
			// Vec = (+/-)A * (-/+)u.
			var ang_by_2 = v.Length; // By convention, log space uses Length = A/2
			var cos_half_ang = Math.Cos(ang_by_2);
			var sin_half_ang = Math.Sin(ang_by_2); // != Sqrt(1 - cos_half_ang * cos_half_ang) when ang_by_2 > tau/2
			var s = ang_by_2 > Math_.TinyD ? (float)(sin_half_ang / ang_by_2) : 1f;
			return new Quat { xyz = v * s, w = (float)cos_half_ang };
		}
		public static Quat ExpMap(v4 v)
		{
			return ExpMap(v.xyz);
		}

		/// <summary>Evaluates 'ori' after 'time' for a constant angular velocity and angular acceleration</summary>
		public static Quat RotationAt(float time, Quat ori, v4 avel, v4 aacc)
		{
			// Orientation can be computed analytically if angular velocity
			// and angular acceleration are parallel or angular acceleration is zero.
			if (Cross(avel, aacc).LengthSq < Math_.TinyF)
			{
				var w = avel + aacc * time;
				return ExpMap(0.5f * w * time) * ori;
			}
			else
			{
				// Otherwise, use the SPIRAL(6) algorithm. 6th order accurate for moderate 'time_s'

				// 3-point Gauss-Legendre nodes for 6th order accuracy
				const float root15f = 3.87298334620741688518f;
				const float c1 = 0.5f - root15f / 10.0f;
				const float c2 = 0.5f;
				const float c3 = 0.5f + root15f / 10.0f;

				// Evaluate instantaneous angular velocity at nodes
				var w0 = avel + aacc * c1 * time;
				var w1 = avel + aacc * c2 * time;
				var w2 = avel + aacc * c3 * time;

				var u0 = ExpMap(0.5f * w0 * time / 3.0f);
				var u1 = ExpMap(0.5f * w1 * time / 3.0f);
				var u2 = ExpMap(0.5f * w2 * time / 3.0f);

				return u2 * u1 * u0 * ori;
			}
		}

		/// <summary>Scale the rotation by 'x'. i.e. 'frac' == 2 => double the rotation, 'frac' == 0.5 => halve the rotation</summary>
		public static Quat Scale(Quat q, float frac)
		{
			Debug.Assert(FEql(q.LengthSq, 1.0f), "quaternion isn't normalised");

			// Trig:
			//' sin^2(x) + cos^2(x) == 1
			//' s == sqrt(1 - w^2) == sqrt(1 - cos^2(x/2))
			//' s^2 == 1 - cos^2(x/2) == sin^2(x/2)
			//' s == sin(x/2)
			var w = Clamp(q.w, -1f, 1f);           // = cos(x/2)
			var s = (float)Math.Sqrt(1f - Sqr(w)); // = sin(x/2)
			var a = frac * (float)Math.Acos(w);    // = scaled half angle
			var sin_ha = (float)Math.Sin(a);
			var cos_ha = (float)Math.Cos(a);
			return new(
				q.x * sin_ha / s,
				q.y * sin_ha / s,
				q.z * sin_ha / s,
				cos_ha);
		}

		/// <summary>Return the axis and angle from a quaternion</summary>
		public static (v4 axis, float angle) AxisAngle(Quat quat)
		{
			var w = (float)Clamp(quat.w, -1.0, 1.0);
			var s = (float)Math.Sqrt(1.0f - w * w);
			var angle = (float)(2.0 * Math.Acos(w));
			var axis = Math.Abs(s) > Math_.TinyF
				? new(quat.x / s, quat.y / s, quat.z / s, 0.0f)
				: v4.Zero; // axis is (0,0,0) when angle == 1

			return (axis, angle);
		}

		/// <summary>Return possible Euler angles for the quaternion 'q'</summary>
		public static v4 EulerAngles(Quat q)
		{
			// From Wikipedia
			double q0 = q.w, q1 = q.x, q2 = q.y, q3 = q.z;
			return new(
				(float)Math.Atan2(2.0 * (q0 * q1 + q2 * q3), 1.0 - 2.0 * (q1 * q1 + q2 * q2)),
				(float)Math.Asin(2.0 * (q0 * q2 - q3 * q1)),
				(float)Math.Atan2(2.0 * (q0 * q3 + q1 * q2), 1.0 - 2.0 * (q2 * q2 + q3 * q3)),
				0f);
		}

		/// <summary>Spherically interpolate between quaternions</summary>
		public static Quat Slerp(Quat a, Quat b, double frac)
		{
			// Flip 'b' so that both quaternions are in the same hemisphere (since: q == -q)
			var cos_half_angle = CosHalfAngle(a, b);
			var b_ = cos_half_angle >= 0 ? b : -b;
			cos_half_angle = Math.Abs(cos_half_angle);

			if (cos_half_angle < 0.95f)
			{
				var angle = 0.5 * Math.Acos(cos_half_angle);
				var scale0 = Math.Sin((1 - frac) * angle);
				var scale1 = Math.Sin((frac) * angle);
				var sin_angle = Math.Sin(angle);
				return new((a.xyzw * (float)scale0 + b_.xyzw * (float)scale1) / (float)sin_angle);
			}
			// "a" and "b" quaternions are very close, use linear interpolation
			else
			{
				return Normalise(new Quat(Lerp(a.xyzw, b_.xyzw, frac)));
			}
		}

		/// <summary>Rotate a vector by a quaternion. This is an optimised version of: 'r = q*v*conj(q) for when v.w == 0'</summary>
		public static v4 Rotate(Quat lhs, v4 rhs)
		{
			var res = Rotate(lhs, rhs.xyz).w0;
			res.w = rhs.w;
			return res;
		}
		public static v3 Rotate(Quat lhs, v3 rhs)
		{
			float xx = lhs.x * lhs.x, xy = lhs.x * lhs.y, xz = lhs.x * lhs.z, xw = lhs.x * lhs.w;
			float yy = lhs.y * lhs.y, yz = lhs.y * lhs.z, yw = lhs.y * lhs.w;
			float zz = lhs.z * lhs.z, zw = lhs.z * lhs.w;
			float ww = lhs.w * lhs.w;
			return new(
				ww * rhs.x + 2 * yw * rhs.z - 2 * zw * rhs.y + xx * rhs.x + 2 * xy * rhs.y + 2 * xz * rhs.z - zz * rhs.x - yy * rhs.x,
				2 * xy * rhs.x + yy * rhs.y + 2 * yz * rhs.z + 2 * zw * rhs.x - zz * rhs.y + ww * rhs.y - 2 * xw * rhs.z - xx * rhs.y,
				2 * xz * rhs.x + 2 * yz * rhs.y + zz * rhs.z - 2 * yw * rhs.x - yy * rhs.z + 2 * xw * rhs.y - xx * rhs.z + ww * rhs.z
			);
		}

		/// <summary>
		/// Return the average of a collection of rotations.
		/// Note: this only really works if all the quaternions are relatively close together.
		/// For two quaternions, prefer 'Slerp'</summary>
		public static Quat Average(IEnumerable<Quat> rotations)
		{
			// Nicked from Unity3D
			// This method is based on a simplified procedure described in this document:
			// http://ntrs.nasa.gov/archive/nasa/casi.ntrs.nasa.gov/20070017872_2007014421.pdf

			// Ensure the quaternions are in the same hemisphere (since q == -q)
			var first = rotations.First();
			var avr = Average(rotations.Select(q => Dot(q.xyzw, first.xyzw) >= 0 ? q.xyzw : -q.xyzw));
			return Normalise(new Quat(avr));
		}
	}
}

#if PR_UNITTESTS
namespace Rylogic.UnitTests
{
	using System.Linq;
	using Extn;
	using Maths;

	[TestFixture]
	public class UnitTestQuat
	{
		[Test]
		public void General()
		{
		}
		[Test]
		public void EulerAngles()
		{
#if false
			Action<float,float,float> Check = (p_,y_,r_) =>
			{
				var p = Math_.DegreesToRadians(p_);
				var y = Math_.DegreesToRadians(y_);
				var r = Math_.DegreesToRadians(r_);

				// EulerAngles is not guaranteed to return the same as p,y,r,
				// only angles that combined will create the same rotation as p,y,r.
				var q0    = quat.Make(p, y, r);
				var euler = quat.EulerAngles(q0);
				var q1    = quat.Make(euler.x, euler.y, euler.z);
				Assert.True(quat.FEql(q0, q1));
			};

			Check(+10f,0,0);
			Check(-10f,0,0);
			Check(0,+10f,0);
			Check(0,-10f,0);
			Check(0,0,+10f);
			Check(0,0,-10f);

			// Random cases
			var rnd = new Random(1);
			for (int i = 0; i != 100; ++i)
			{
				var pitch = (float)rnd.NextDoubleCentred(0.0, 360.0);
				var yaw   = (float)rnd.NextDoubleCentred(0.0, 360.0);
				var roll  = (float)rnd.NextDoubleCentred(0.0, 360.0);
				Check(pitch, yaw, roll);
			}

			// Special cases
			var angles = new []{0f, 90f, -90f, 180f, -180f, 270f, -270f, 360f, -360f};
			foreach (var p in angles)
				foreach (var y in angles)
					foreach (var r in angles)
						Check(p, y, r);
#endif
		}
		[Test]
		public void Angles()
		{
			List<float> angles = [
				-Math_.TauBy2F,
				-Math_.TauBy3F,
				-Math_.TauBy4F,
				-Math_.TauBy5F,
				-Math_.TauBy6F,
				-Math_.TauBy7F,
				0f,
				+Math_.TauBy7F,
				+Math_.TauBy6F,
				+Math_.TauBy5F,
				+Math_.TauBy4F,
				+Math_.TauBy3F,
				+Math_.TauBy2F,
			];

			var axis = Math_.Normalise(new v4(1, 1, 1, 0));
			foreach (var ANG0 in angles)
			{
				foreach (var ANG1 in angles)
				{
					var q0 = new Quat(axis, ANG0);
					var q1 = new Quat(axis, ANG1);
					var expected = Math_.Min(Math_.Abs(ANG1 - ANG0), Math_.Abs(Math_.TauF - Math_.Abs(ANG1 - ANG0)));

					var ang0 = Math_.Angle(q0, q1);
					var ang1 = Math_.Angle(q1, q0);
					const float angular_tolerance = 1e-3f;
					Assert.True(Math_.FEqlAbsolute(ang0, expected, angular_tolerance));
					Assert.True(Math_.FEqlAbsolute(ang1, expected, angular_tolerance));
				}
			}
		}
		[Test]
		public void Average()
		{
			var rng = new Random(1);
			var ideal_mean = new Quat(Math_.Normalise(new v4(1, 1, 1, 0)), 0.5f);
			var actual_mean = Math_.Average(int_.Range(0, 1000).Select(i =>
			{
				var axis = Math_.Normalise(ideal_mean.Axis + v4.Random3(0.02f, 0f, rng));
				var angle = rng.FloatC(ideal_mean.Angle, 0.02f);
				var q = new Quat(axis, angle);
				return rng.Bool() ? q : -q;
			}));
			Assert.True(Math_.FEqlRelative(ideal_mean, actual_mean, 0.01f));
		}
		[Test]
		public void QuatMatrixRoundTrip()
		{
			var ori0 = Quat.Random(new Random(1));
			var m0 = new m3x4(ori0);
			var ori1 = new Quat(m0);
			Assert.True(Math_.FEql(ori0, ori1));
		}
		[Test]
		public void LogMapExpMap()
		{
			// Round trip test
			var rng = new Random(1);
			var max_angular_error = 0f;
			for (var i = 0; i != 100; ++i)
			{
				var q0 = Quat.Random(rng);
				var v0 = Math_.LogMap(q0);
				var q1 = Math_.ExpMap(v0);
				var angular_error = Math_.Angle(q0, q1);
				
				max_angular_error = Math.Max(max_angular_error, angular_error);
				Assert.True(Math.Abs(angular_error) < 0.001f);
			}
			Assert.True(max_angular_error < 0.001f);

			// Special cases
			{
				var q0 = new Quat(-2.09713704e-08f, -0.00148352725f, -6.48572168e-11f, -0.999998927f);
				q0 = Math_.Normalise(q0);
				var v0 = Math_.LogMap(q0);
				var q1 = Math_.ExpMap(v0);
				var angular_error = Math_.Angle(q0, q1);
				Assert.True(Math.Abs(angular_error) < 0.001f);
			}
		}
		[Test]
		public void RotationAt()
		{
			{// Analytic solution case
				var ori = Quat.Random(new Random(1));
				var avl = new v4(0.6f, 0, 0.6f, 0);
				var aac = new v4(0, 0, 0, 0);

				var rot = new m3x4(ori);
				for (float t = 0; t < 5.0f; t += 0.1f)
				{
					var ORI = Math_.RotationAt(t, ori, avl, aac);
					var ROT = Math_.RotationAt(t, rot, avl, aac);
					var ROT2 = new m3x4(ORI);

					Assert.True(Math_.FEql(ROT, ROT2));
				}
			}
			{// Non-analytic solution case
				var ori = Quat.Random(new Random(1));
				var avl = new v4(1.2f, 0, 0, 0);
				var aac = new v4(0, 0, 0.1f, 0);

				var rot = new m3x4(ori);
				for (float t = 0; t < 5.0f; t += 0.1f)
				{
					var ORI = Math_.RotationAt(t, ori, avl, aac);
					var ROT = Math_.RotationAt(t, rot, avl, aac);
					var ROT2 = new m3x4(ORI);

					Assert.True(Math_.FEql(ROT, ROT2));
				}
			}
		}
		[Test]
		public void NumericsConversion()
		{
			var vec0 = Quaternion.Normalize(new Quaternion(1, 2, 3, 4));
			var vec1 = (Quat)vec0;
			var vec2 = (Quaternion)vec1;
			var vec3 = (Quat)vec2;

			Assert.True(vec0 == vec2);
			Assert.True(vec1 == vec3);
		}
	}
}
#endif