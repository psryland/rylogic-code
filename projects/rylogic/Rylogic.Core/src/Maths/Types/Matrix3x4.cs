//***************************************************
// Matrix4x4
//  Copyright (c) Rylogic Ltd 2008
//***************************************************

using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Runtime.InteropServices;
using Rylogic.Extn;

namespace Rylogic.Maths
{
	[Serializable]
	[StructLayout(LayoutKind.Explicit)]
	[DebuggerDisplay("{Description,nq}")]
	public struct m3x4
	{
		[FieldOffset( 0)] public v4 x;
		[FieldOffset(16)] public v4 y;
		[FieldOffset(32)] public v4 z;

		public m3x4(float x)
			:this(new v4(x), new v4(x), new v4(x))
		{}
		public m3x4(v4 x, v4 y, v4 z) :this()
		{
			this.x = x;
			this.y = y;
			this.z = z;
		}
		public m3x4(Quat quaternion) :this()
		{
			Debug.Assert(!Math_.FEql(quaternion, Quat.Zero), "'quaternion' is a zero quaternion");

			var q = quaternion;
			var q_lensq = q.LengthSq;
			var s = 2.0f / q_lensq;

			float xs = q.x *  s, ys = q.y *  s, zs = q.z *  s;
			float wx = q.w * xs, wy = q.w * ys, wz = q.w * zs;
			float xx = q.x * xs, xy = q.x * ys, xz = q.x * zs;
			float yy = q.y * ys, yz = q.y * zs, zz = q.z * zs;

			x.x = 1.0f - (yy + zz); y.x = xy - wz;          z.x = xz + wy;
			x.y = xy + wz;          y.y = 1.0f - (xx + zz); z.y = yz - wx;
			x.z = xz - wy;          y.z = yz + wx;          z.z = 1.0f - (xx + yy);
			x.w =                   y.w =                   z.w = 0.0f;
		}

		/// <summary>Get/Set components by index</summary>
		public v4 this[int i]
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
		public v4 this[uint i]
		{
			get { return this[(int)i]; }
			set { this[(int)i] = value; }
		}

		/// <summary>Get/Set components by index. Note: row,col is standard (i.e. as in Matlab)</summary>
		private float this[int r, int c]
		{
			get { return this[c][r]; }
			set
			{
				var vec = this[c];
				vec[r] = value;
				this[c] = vec;
			}
		}
		private float this[uint r, uint c]
		{
			get { return this[(int)c][(int)r]; }
			set
			{
				var vec = this[c];
				vec[r] = value;
				this[c] = vec;
			}
		}

		/// <summary>Convert to a 4x4 matrix with zero translation</summary>
		public m4x4 m4x4 => new(this, v4.Origin);

		/// <summary>To flat array</summary>
		public float[] ToArray()
		{
			return new []
			{
				x.x, x.y, x.z, x.w,
				y.x, y.y, y.z, y.w,
				z.x, z.y, z.z, z.w,
			};
		}

		/// <summary>
		/// Create from an pitch, yaw, and roll (in radians).
		/// Order is roll, pitch, yaw because objects usually face along Z and have Y as up. (And to match DirectX)</summary>
		public static m3x4 RotationRad(float pitch, float yaw, float roll)
		{
			float cos_p = (float)Math.Cos(pitch), sin_p = (float)Math.Sin(pitch);
			float cos_y = (float)Math.Cos(yaw  ), sin_y = (float)Math.Sin(yaw  );
			float cos_r = (float)Math.Cos(roll ), sin_r = (float)Math.Sin(roll );
			return new(
				new v4( cos_y*cos_r + sin_y*sin_p*sin_r , cos_p*sin_r , -sin_y*cos_r + cos_y*sin_p*sin_r , 0.0f),
				new v4(-cos_y*sin_r + sin_y*sin_p*cos_r , cos_p*cos_r ,  sin_y*sin_r + cos_y*sin_p*cos_r , 0.0f),
				new v4( sin_y*cos_p                     ,      -sin_p ,                      cos_y*cos_p , 0.0f));
		}
		public static m3x4 RotationDeg(float pitch, float yaw, float roll)
		{
			return RotationRad(
				Math_.DegreesToRadians(pitch),
				Math_.DegreesToRadians(yaw),
				Math_.DegreesToRadians(roll));
		}

		/// <summary>Create a rotation from an axis and angle</summary>
		public static m3x4 Rotation(v4 axis_norm, v4 axis_sine_angle, float cos_angle)
		{
			Debug.Assert(Math_.FEql(axis_norm.LengthSq, 1f), "'axis_norm' should be normalised");

			var mat = new m3x4();

			v4 trace_vec = axis_norm * (1.0f - cos_angle);

			mat.x.x = trace_vec.x * axis_norm.x + cos_angle;
			mat.y.y = trace_vec.y * axis_norm.y + cos_angle;
			mat.z.z = trace_vec.z * axis_norm.z + cos_angle;

			trace_vec.x *= axis_norm.y;
			trace_vec.z *= axis_norm.x;
			trace_vec.y *= axis_norm.z;

			mat.x.y = trace_vec.x + axis_sine_angle.z;
			mat.x.z = trace_vec.z - axis_sine_angle.y;
			mat.x.w = 0.0f;
			mat.y.x = trace_vec.x - axis_sine_angle.z;
			mat.y.z = trace_vec.y + axis_sine_angle.x;
			mat.y.w = 0.0f;
			mat.z.x = trace_vec.z + axis_sine_angle.y;
			mat.z.y = trace_vec.y - axis_sine_angle.x;
			mat.z.w = 0.0f;

			return mat;
		}

		/// <summary>Create from an axis and angle. 'axis' should be normalised. 'angle' is in radians</summary>
		public static m3x4 Rotation(v4 axis_norm, float angle)
		{
			Debug.Assert(Math_.FEql(axis_norm.LengthSq, 1f), "'axis_norm' should be normalised");
			return Rotation(axis_norm, axis_norm * (float)Math.Sin(angle), (float)Math.Cos(angle));
		}

		/// <summary>Create from an angular displacement vector. length = angle(rad), direction = axis</summary>
		public static m3x4 Rotation(v4 angular_displacement) // This is ExpMap
		{
			Debug.Assert(angular_displacement.w == 0, "'angular_displacement' should be a scaled direction vector");
			var len = angular_displacement.Length;
			return len > Math_.TinyF
				? Rotation(angular_displacement/len, len)
				: Identity;
		}

		/// <summary>Create a transform representing the rotation from one vector to another. 'from' and 'to' do not have to be normalised</summary>
		public static m3x4 Rotation(v4 from, v4 to)
		{
			Debug.Assert(!Math_.FEql(from.Length, 0));
			Debug.Assert(!Math_.FEql(to.Length, 0));
			var len = from.Length * to.Length;

			// Find the cosine of the angle between the vectors
			var cos_angle = Math_.Dot(from, to) / len;
			if (cos_angle >= 1f - Math_.TinyF) return Identity;
			if (cos_angle <= Math_.TinyF - 1f) return Rotation(Math_.Normalise(Math_.Perpendicular(from - to)), (float)Math_.TauBy2);

			// Axis multiplied by sine of the angle
			var axis_sine_angle = Math_.Cross(from, to) / len;
			var axis_norm = Math_.Normalise(axis_sine_angle);

			return Rotation(axis_norm, axis_sine_angle, cos_angle);
		}

		/// <summary>Create a rotation transform that maps 'from' to 'to'</summary>
		public static m3x4 Rotation(AxisId from, AxisId to)
		{
			// 'o2f' = the rotation from Z to 'from_axis'
			// 'o2t' = the rotation from Z to 'to_axis'
			// 'f2t' = o2t * Invert(o2f)
			m3x4 o2f, o2t;
			switch (from)
			{
				case -1: o2f = RotationRad(0f, (float)+Math_.TauBy4, 0f); break;
				case +1: o2f = RotationRad(0f, (float)-Math_.TauBy4, 0f); break;
				case -2: o2f = RotationRad((float)+Math_.TauBy4, 0f, 0f); break;
				case +2: o2f = RotationRad((float)-Math_.TauBy4, 0f, 0f); break;
				case -3: o2f = RotationRad(0f, (float)+Math_.TauBy2, 0f); break;
				case +3: o2f = Identity; break;
				default: throw new Exception("axis_id must one of \uC2B11, \uC2B12, \uC2B13");
			}
			switch (to)
			{
				case -1: o2t = RotationRad(0f, (float)-Math_.TauBy4, 0f); break; // I know this sign looks wrong, but it isn't. Must be something to do with signs passed to cos()/sin()
				case +1: o2t = RotationRad(0f, (float)+Math_.TauBy4, 0f); break;
				case -2: o2t = RotationRad((float)+Math_.TauBy4, 0f, 0f); break;
				case +2: o2t = RotationRad((float)-Math_.TauBy4, 0f, 0f); break;
				case -3: o2t = RotationRad(0f, (float)+Math_.TauBy2, 0f); break;
				case +3: o2t = Identity; break;
				default: throw new Exception("axis_id must one of \uC2B11, \uC2B12, \uC2B13");
			}
			return o2t * Math_.InvertAffine(o2f);
		}

		/// <summary>Create a scale matrix</summary>
		public static m3x4 Scale(float s)
		{
			return Scale(s, s, s);
		}
		public static m3x4 Scale(v3 s)
		{
			return Scale(s.x, s.y, s.z);
		}
		public static m3x4 Scale(float sx, float sy, float sz)
		{
			return new(sx * v4.XAxis, sy * v4.YAxis, sz * v4.ZAxis);
		}

		/// <summary>Create a shear matrix</summary>
		public static m3x4 Shear(float sxy, float sxz, float syx, float syz, float szx, float szy)
		{
			return new(
				new(1.0f, sxy, sxz, 0.0f),
				new(syx, 1.0f, syz, 0.0f),
				new(szx, szy, 1.0f, 0.0f)
			);
		}

		#region Statics
		public static readonly m3x4 Zero = new(v4.Zero, v4.Zero, v4.Zero);
		public static readonly m3x4 Identity = new(v4.XAxis, v4.YAxis, v4.ZAxis);
		#endregion

		#region Operators
		public static m3x4 operator +(m3x4 rhs)
		{
			return rhs;
		}
		public static m3x4 operator -(m3x4 rhs)
		{
			return new(-rhs.x, -rhs.y, -rhs.z);
		}
		public static m3x4 operator +(m3x4 lhs, m3x4 rhs)
		{
			return new(lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z);
		}
		public static m3x4 operator -(m3x4 lhs, m3x4 rhs)
		{
			return new(lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z);
		}
		public static m3x4 operator *(m3x4 lhs, float rhs)
		{
			return new(lhs.x * rhs, lhs.y * rhs, lhs.z * rhs);
		}
		public static m3x4 operator *(float lhs, m3x4 rhs)
		{
			return new(lhs * rhs.x, lhs * rhs.y, lhs * rhs.z);
		}
		public static m3x4 operator /(m3x4 lhs, float rhs)
		{
			return new(lhs.x / rhs, lhs.y / rhs, lhs.z / rhs);
		}
		public static m3x4 operator %(m3x4 lhs, float rhs)
		{
			return new(lhs.x % rhs, lhs.y % rhs, lhs.z % rhs);
		}
		public static m3x4 operator %(m3x4 lhs, double rhs)
		{
			return lhs % (float)rhs;
		}
		public static m3x4 operator %(m3x4 lhs, m3x4 rhs)
		{
			return new(lhs.x % rhs.x, lhs.y % rhs.y, lhs.z % rhs.z);
		}
		public static m3x4 operator * (m3x4 lhs, m3x4 rhs)
		{
			Math_.Transpose(ref lhs);
			return new(
				new v4(Math_.Dot(lhs.x.xyz, rhs.x.xyz), Math_.Dot(lhs.y.xyz, rhs.x.xyz), Math_.Dot(lhs.z.xyz, rhs.x.xyz), 0f),
				new v4(Math_.Dot(lhs.x.xyz, rhs.y.xyz), Math_.Dot(lhs.y.xyz, rhs.y.xyz), Math_.Dot(lhs.z.xyz, rhs.y.xyz), 0f),
				new v4(Math_.Dot(lhs.x.xyz, rhs.z.xyz), Math_.Dot(lhs.y.xyz, rhs.z.xyz), Math_.Dot(lhs.z.xyz, rhs.z.xyz), 0f));
		}
		public static v3 operator * (m3x4 lhs, v3 rhs)
		{
			Math_.Transpose(ref lhs);
			return new v3(
				Math_.Dot(lhs.x.xyz, rhs),
				Math_.Dot(lhs.y.xyz, rhs),
				Math_.Dot(lhs.z.xyz, rhs));
		}
		public static v4 operator * (m3x4 lhs, v4 rhs)
		{
			Math_.Transpose(ref lhs);
			return new v4(
				Math_.Dot(lhs.x, rhs),
				Math_.Dot(lhs.y, rhs),
				Math_.Dot(lhs.z, rhs),
				rhs.w);
		}
		#endregion

		#region Equals
		public static bool operator == (m3x4 lhs, m3x4 rhs)
		{
			return lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z;
		}
		public static bool operator != (m3x4 lhs, m3x4 rhs)
		{
			return !(lhs == rhs);
		}
		public override bool Equals(object? o)
		{
			return o is m3x4 m && m == this;
		}
		public override int GetHashCode()
		{
			return new { x, y, z }.GetHashCode();
		}
		#endregion

		#region ToString
		public override string ToString() => ToString3x3();
		public string ToString4x3() => $"{x} \n{y} \n{z} \n";
		public string ToString3x3() => $"{x.xyz} \n{y.xyz} \n{z.xyz} \n";
		public string ToString(string format) => $"{x.ToString(format)} \n{y.ToString(format)} \n{z.ToString(format)} \n";
		public string ToCodeString(ECodeString fmt = ECodeString.CSharp)
		{
			return fmt switch
			{
				ECodeString.CSharp => $"new m4x4(\nnew v4({x.ToCodeString(fmt)}),\n new v4({y.ToCodeString(fmt)}),\n new v4({z.ToCodeString(fmt)})\n)",
				ECodeString.Cpp => $"{{\n{{{x.ToCodeString(fmt)}}},\n{{{y.ToCodeString(fmt)}}},\n{{{z.ToCodeString(fmt)}}},\n}}",
				ECodeString.Python => $"[\n[{x.ToCodeString(fmt)}],\n[{y.ToCodeString(fmt)}],\n[{z.ToCodeString(fmt)}],\n]",
				_ => ToString(),
			};
		}
		#endregion

		#region Parse
		public static m3x4 Parse(string s)
		{
			s = s ?? throw new ArgumentNullException("s", $"{nameof(Parse)}:string argument was null");
			return TryParse(s, out var result) ? result : throw new FormatException($"{nameof(Parse)}: string argument does not represent a 4x4 matrix");
		}
		public static m3x4 Parse3x3(string s)
		{
			s = s ?? throw new ArgumentNullException("s", $"{nameof(Parse3x3)}:string argument was null");
			return TryParse3x3(s, out var result) ? result : throw new FormatException($"{nameof(Parse3x3)}: string argument does not represent a 3x3 matrix");
		}
		public static m3x4 Parse4x3(string s)
		{
			s = s ?? throw new ArgumentNullException("s", $"{nameof(Parse4x3)}:string argument was null");
			return TryParse4x3(s, out var result) ? result : throw new FormatException($"{nameof(Parse4x3)}: string argument does not represent a 3x4 matrix");
		}
		public static bool TryParse(string s, out m3x4 mat, bool row_major = true)
		{
			if (s == null)
			{
				mat = default;
				return false;
			}

			var values = s.Split([' ', ',', '\t', '\n'], 16, StringSplitOptions.RemoveEmptyEntries);
			if (values.Length != 12)
			{
				mat = default;
				return false;
			}

			mat = row_major
				? new(
					new v4(float.Parse(values[0]), float.Parse(values[1]), float.Parse(values[2]), float.Parse(values[3])),
					new v4(float.Parse(values[4]), float.Parse(values[5]), float.Parse(values[6]), float.Parse(values[7])),
					new v4(float.Parse(values[8]), float.Parse(values[9]), float.Parse(values[10]), float.Parse(values[11])))
				: new(
					new v4(float.Parse(values[0]), float.Parse(values[3]), float.Parse(values[6]), float.Parse(values[9])),
					new v4(float.Parse(values[1]), float.Parse(values[4]), float.Parse(values[7]), float.Parse(values[10])),
					new v4(float.Parse(values[2]), float.Parse(values[5]), float.Parse(values[8]), float.Parse(values[11])));
			return true;
		}
		public static bool TryParse3x3(string s, out m3x4 mat, bool row_major = true)
		{
			if (s == null)
			{
				mat = default;
				return false;
			}

			var values = s.Split(new char[] { ' ', ',', '\t', '\n' }, 9, StringSplitOptions.RemoveEmptyEntries);
			if (values.Length != 9)
			{
				mat = default;
				return false;
			}

			mat = row_major
				? new(
					new v4(float.Parse(values[0]), float.Parse(values[1]), float.Parse(values[2]), 0),
					new v4(float.Parse(values[3]), float.Parse(values[4]), float.Parse(values[5]), 0),
					new v4(float.Parse(values[6]), float.Parse(values[7]), float.Parse(values[8]), 0))
				: new(
					new v4(float.Parse(values[0]), float.Parse(values[3]), float.Parse(values[6]), 0),
					new v4(float.Parse(values[1]), float.Parse(values[4]), float.Parse(values[7]), 0),
					new v4(float.Parse(values[2]), float.Parse(values[5]), float.Parse(values[8]), 0));
			return true;
		}
		public static bool TryParse4x3(string s, out m3x4 mat)
		{
			if (s == null)
			{
				mat = default;
				return false;
			}

			var values = s.Split(new char[] { ' ', ',', '\t', '\n' }, 12, StringSplitOptions.RemoveEmptyEntries);
			if (values.Length != 12)
			{
				mat = default;
				return false;
			}

			mat = new(
				new v4(float.Parse(values[0]), float.Parse(values[1]), float.Parse(values[2]), float.Parse(values[3])),
				new v4(float.Parse(values[4]), float.Parse(values[5]), float.Parse(values[6]), float.Parse(values[7])),
				new v4(float.Parse(values[8]), float.Parse(values[9]), float.Parse(values[10]), float.Parse(values[11])));
			return true;
		}
		#endregion

		#region Random

		/// <summary>Create a random 3x4 matrix</summary>
		public static m3x4 Random(float min_value, float max_value, Random r)
		{
			return new(
				new v4(r.Float(min_value, max_value), r.Float(min_value, max_value), r.Float(min_value, max_value), r.Float(min_value, max_value)),
				new v4(r.Float(min_value, max_value), r.Float(min_value, max_value), r.Float(min_value, max_value), r.Float(min_value, max_value)),
				new v4(r.Float(min_value, max_value), r.Float(min_value, max_value), r.Float(min_value, max_value), r.Float(min_value, max_value)));
		}
		public static m3x4 Random(float min_value, float max_value, float w, Random r)
		{
			return new(
				new v4(r.Float(min_value, max_value), r.Float(min_value, max_value), r.Float(min_value, max_value), w),
				new v4(r.Float(min_value, max_value), r.Float(min_value, max_value), r.Float(min_value, max_value), w),
				new v4(r.Float(min_value, max_value), r.Float(min_value, max_value), r.Float(min_value, max_value), w));
		}

		/// <summary>Create a random 3D rotation matrix</summary>
		public static m3x4 Random(v4 axis, float min_angle, float max_angle, Random r)
		{
			return Rotation(axis, r.Float(min_angle, max_angle));
		}
		public static m3x4 Random(Random r)
		{
			return Random(v4.Random3N(0.0f, r), 0.0f, (float)Math_.Tau, r);
		}

		#endregion

		/// <summary></summary>
		public string Description => $"{x.Description}  \n{y.Description}  \n{z.Description}  \n";
	}

	public static partial class Math_
	{
		/// <summary>Approximate equal</summary>
		public static bool FEqlAbsolute(m3x4 a, m3x4 b, float tol)
		{
			return
				FEqlAbsolute(a.x, b.x, tol) &&
				FEqlAbsolute(a.y, b.y, tol) &&
				FEqlAbsolute(a.z, b.z, tol);
		}
		public static bool FEqlRelative(m3x4 a, m3x4 b, float tol)
		{
			var max_a = MaxElement(Abs(a));
			var max_b = MaxElement(Abs(b));
			if (max_b == 0) return max_a < tol;
			if (max_a == 0) return max_b < tol;
			var abs_max_element = Max(max_a, max_b);
			return FEqlAbsolute(a, b, tol * abs_max_element);
		}
		public static bool FEql(m3x4 a, m3x4 b)
		{
			return
				FEqlRelative(a, b, TinyF);
		}

		/// <summary>Absolute value of 'x'</summary>
		public static m3x4 Abs(m3x4 x)
		{
			return new(
				Abs(x.x),
				Abs(x.y),
				Abs(x.z));
		}

		/// <summary>Return the maximum element value in 'mm'</summary>
		public static float MaxElement(m3x4 m)
		{
			return Max(
				MaxElement(m.x),
				MaxElement(m.y),
				MaxElement(m.z));
		}

		/// <summary>Return the minimum element value in 'm'</summary>
		public static float MinElement(m3x4 m)
		{
			return Min(
				MinElement(m.x),
				MinElement(m.y),
				MinElement(m.z));
		}

		/// <summary>Finite test of matrix elements</summary>
		public static bool IsFinite(m3x4 m)
		{
			return
				IsFinite(m.x) &&
				IsFinite(m.y) &&
				IsFinite(m.z);
		}

		/// <summary>Return true if any components of 'm' are NaN</summary>
		public static bool IsNaN(m3x4 m)
		{
			return
				IsNaN(m.x) ||
				IsNaN(m.y) ||
				IsNaN(m.z);
		}

		/// <summary>True if 'm' is orthogonal</summary>
		public static bool IsOrthogonal(m3x4 m)
		{
			return
				FEql(Dot(m.x, m.y), 0f) &&
				FEql(Dot(m.x, m.z), 0f) &&
				FEql(Dot(m.y, m.z), 0f);
		}

		/// <summary>True if 'm' is orthonormal</summary>
		public static bool IsOrthonormal(m3x4 m)
		{
			return
				FEql(m.x.LengthSq, 1f) &&
				FEql(m.y.LengthSq, 1f) &&
				FEql(m.z.LengthSq, 1f) &&
				FEql(Determinant(m), 1f);
		}

		// Return true if 'mat' is an affine transform
		public static bool IsAffine(m3x4 mat)
		{
			return
				mat.x.w == 0.0f &&
				mat.y.w == 0.0f &&
				mat.z.w == 0.0f;
		}

		/// <summary>True if 'm' can be inverted</summary>
		public static bool IsInvertible(m3x4 m)
		{
			return Determinant(m) != 0;
		}

		/// <summary>Return the determinant of 'm'</summary>
		public static float Determinant(m3x4 m)
		{
			return Triple(m.x, m.y, m.z);
		}

		/// <summary>Return the trace of 'm'</summary>
		public static float Trace(m3x4 m)
		{
			return m.x.x + m.y.y + m.z.z;
		}

		/// <summary>Return the kernel of 'm'</summary>
		public static v4 Kernel(m3x4 m)
		{
			return new v4(m.y.y* m.z.z - m.y.z * m.z.y, -m.y.x * m.z.z + m.y.z * m.z.x, m.y.x* m.z.y - m.y.y * m.z.x, 0);
		}

		/// <summary>Return the diagonal elements of 'm'</summary>
		public static v4 Diagonal(m3x4 m)
		{
			return new v4(m.x.x, m.y.y, m.z.z, 0);
		}

		/// <summary>Transpose 'm' in-place</summary>
		public static void Transpose(ref m3x4 m)
		{
			Swap(ref m.x.y, ref m.y.x);
			Swap(ref m.x.z, ref m.z.x);
			Swap(ref m.y.z, ref m.z.y);
		}

		/// <summary>Return the transpose of 'm'</summary>
		public static m3x4 Transpose(m3x4 m)
		{
			Transpose(ref m);
			return m;
		}

		/// <summary>Invert 'm' in-place assuming m is orthonormal</summary>
		public static void InvertOrthonormal(ref m3x4 m)
		{
			Debug.Assert(IsOrthonormal(m), "Matrix is not orthonormal");
			Transpose(ref m);
		}

		/// <summary>Return the inverse of 'm' assuming m is orthonormal</summary>
		public static m3x4 InvertOrthonormal(m3x4 m)
		{
			InvertOrthonormal(ref m);
			return m;
		}

		/// <summary>Invert 'm' in-place assuming m is affine</summary>
		public static void InvertAffine(ref m3x4 m)
		{
			Debug.Assert(IsAffine(m), "Matrix is not affine");
			
			var s = new v3(m.x.LengthSq, m.y.LengthSq, m.z.LengthSq);
			if (!FEql(s, v3.One))
			{
				if (s.x == 0 || s.y == 0 || s.z == 0)
					throw new Exception("Cannot invert a degenerate matrix");
				s = Sqrt(s);
			}

			// Remove scale
			m.x /= s.x;
			m.y /= s.y;
			m.z /= s.z;

			// Invert rotation
			Transpose(ref m);

			// Invert scale
			m.x /= s.x;
			m.y /= s.y;
			m.z /= s.z;
		}

		/// <summary>Return the inverse of 'm' assuming m is orthonormal</summary>
		public static m3x4 InvertAffine(m3x4 m)
		{
			InvertAffine(ref m);
			return m;
		}

		/// <summary>Invert the matrix 'm'</summary>
		public static m3x4 Invert(m3x4 m)
		{
			if (!IsInvertible(m))
				throw new Exception("Matrix is singular");

			var det = Determinant(m);
			var tmp = new m3x4(
				Cross(m.y, m.z) / det,
				Cross(m.z, m.x) / det,
				Cross(m.x, m.y) / det);

			return Transpose(tmp);
		}

		/// <summary>Normalise the columns of 'm' returning the lengths prior to renormalising</summary>
		public static (m3x4, v3) Normalise(m3x4 m)
		{
			var scale = new v3(m.x.Length, m.y.Length, m.z.Length);
			m.x /= scale.x;
			m.y /= scale.y;
			m.z /= scale.z;
			return (m, scale);
		}

		/// <summary>Orthonormalise 'm' in-place</summary>
		public static void Orthonormalise(ref m3x4 m)
		{
			Normalise(ref m.x);
			m.y = Normalise(Cross(m.z, m.x));
			m.z = Cross(m.x, m.y);
		}

		/// <summary>Return an orthonormalised version of 'm'</summary>
		public static m3x4 Orthonormalise(m3x4 m)
		{
			Orthonormalise(ref m);
			return m;
		}

		/// <summary>
		/// Permute the vectors in a rotation matrix by 'n'.<para/>
		/// n == 0 : x  y  z<para/>
		/// n == 1 : z  x  y<para/>
		/// n == 2 : y  z  x<para/></summary>
		public static m3x4 PermuteRotation(m3x4 mat, int n)
		{
			switch (n % 3)
			{
			default: return mat;
			case 1: return new(mat.z, mat.x, mat.y);
			case 2: return new(mat.y, mat.z, mat.x);
			}
		}

		/// <summary>Return possible Euler angles for the rotation matrix 'mat'</summary>
		public static v4 EulerAngles(m3x4 mat)
		{
			var q = new Quat(mat);
			return EulerAngles(q);
		}

		/// <summary>
		/// Make an orientation matrix from a direction. Note the rotation around the direction
		/// vector is not defined. 'axis' is the axis that 'direction' will become.</summary>
		public static m3x4 OriFromDir(AxisId axis, v4 dir, v4? up = null)
		{
			// Get the preferred up direction (handling parallel cases)
			var up_ = up == null || Parallel(up.Value, dir) ? Perpendicular(dir) : up.Value;

			var ori = m3x4.Identity;
			ori.z = Normalise(Sign(axis) * dir);
			ori.x = Normalise(Cross(up_, ori.z));
			ori.y = Cross(ori.z, ori.x);

			// Permute the column vectors so +Z becomes 'axis'
			return PermuteRotation(ori, Math.Abs(axis));
		}

		/// <summary>Spherically interpolate between two rotations</summary>
		public static m3x4 Slerp(m3x4 lhs, m3x4 rhs, double frac)
		{
			return new(Slerp(new Quat(lhs), new Quat(rhs), frac));
		}

		/// <summary>Return the average of a collection of rotations transforms</summary>
		public static m3x4 Average(IEnumerable<m3x4> a2b)
		{
			return new(Average(a2b.Select(x => new Quat(x))));
		}

		/// <summary>Return the cross product matrix for 'vec'</summary>
		public static m3x4 CPM(v4 vec)
		{
			// This matrix can be used to calculate the cross product with
			// another vector: e.g. Cross3(v1, v2) == CPM(v1) * v2
			return new(
				new v4(    0f, +vec.z, -vec.y, 0f),
				new v4(-vec.z,     0f, +vec.x, 0f),
				new v4(+vec.y, -vec.x,     0f, 0f));
		}

		/// <summary>Return 'exp(omega)' (Rodriges' formula)</summary>
		public static m3x4 ExpMap3x3(v4 omega)
		{
			return m3x4.Rotation(omega);
		}

		// Returns the Axis*Angle vector representation of a rotation matrix (Inverse of ExpMap)
		public static v4 LogMap(m3x4 rot)
		{
			var cos_angle = Clamp((Trace(rot) - 1.0f) / 2.0f, -1.0f, +1.0f);
			var theta = Math.Acos(cos_angle);
			if (Abs(theta) < Math_.TinyF)
				return v4.Zero;

			var s = 1.0f / (2.0f * (float)Math.Sin(theta));
			var axis = s * new v4(rot.y.z - rot.z.y, rot.z.x - rot.x.z, rot.x.y - rot.y.x, 0);
			return theta * axis;
		}
		
		/// <summary>Evaluates 'ori' after 'time' for a constant angular velocity and angular acceleration</summary>
		public static m3x4 RotationAt(float time, m3x4 ori, v4 avel, v4 aacc)
		{
			// Orientation can be computed analytically if angular velocity
			// and angular acceleration are parallel or angular acceleration is zero.
			if (Cross(avel, aacc).LengthSq < Math_.TinyF)
			{
				var w = avel + aacc * time;
				return ExpMap3x3(w * time) * ori;
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

				var u0 = ExpMap3x3(w0 * time / 3.0f);
				var u1 = ExpMap3x3(w1 * time / 3.0f);
				var u2 = ExpMap3x3(w2 * time / 3.0f);

				return u2 * u1 * u0 * ori;
			}
		}
	}
}

#if PR_UNITTESTS
namespace Rylogic.UnitTests
{
	using Maths;

	[TestFixture] public class UnitTestM3x4
	{
		[Test]
		public void TestMultiply()
		{
			var m1 = new m3x4(new v4(1,2,3,4), new v4(1,1,1,1), new v4(4,3,2,1));
			var m2 = new m3x4(new v4(1,1,1,1), new v4(2,2,2,2), new v4(-2,-2,-2,-2));
			var m3 = new m3x4(new v4(6,6,6,0), new v4(12,12,12,0), new v4(-12,-12,-12,0));
			var r = m1 * m2;
			Assert.True(Math_.FEql(r, m3));
		}

		[Test]
		public void TestOriFromDir()
		{
			var dir = new v4(0, 1, 0, 0);
			{
				var ori = Math_.OriFromDir(EAxisId.PosZ, dir, v4.ZAxis);
				Assert.True(dir == ori.z);
				Assert.True(Math_.IsOrthonormal(ori));
			}
			{
				var ori = Math_.OriFromDir(EAxisId.NegX, dir);
				Assert.True(dir == -ori.x);
				Assert.True(Math_.IsOrthonormal(ori));
			}
		}

		[Test]
		public void Inversion()
		{
			var rng = new Random(1);

			{
				var m = m3x4.Random(v4.Random3N(0, rng), -Math_.TauF, +Math_.TauF, rng);
				var inv_m0 = Math_.InvertAffine(m);
				var inv_m1 = Math_.Invert(m);
				Assert.True(Math_.FEqlRelative(inv_m0, inv_m1, 0.001f));
			}
			{
				var a2b = m3x4.Rotation(v4.Random3N(0, rng), rng.FloatC(-5f, +5f)) * m3x4.Scale(2.0f);
				Assert.True(Math_.IsAffine(a2b));

				var b2a = Math_.Invert(a2b);
				var a2a = b2a * a2b;
				Assert.True(Math_.FEql(m3x4.Identity, a2a));

				var b2a_fast = Math_.InvertAffine(a2b);
				Assert.True(Math_.FEql(b2a_fast, b2a));
			}
			{
				var a2b = m3x4.Rotation(v4.Random3N(0, rng), rng.FloatC(-5f, +5f));
				Assert.True(Math_.IsOrthonormal(a2b));

				var b2a = Math_.Invert(a2b);
				var a2a = b2a * a2b;
				Assert.True(Math_.FEql(m3x4.Identity, a2a));

				var b2a_fast = Math_.InvertOrthonormal(a2b);
				Assert.True(Math_.FEql(b2a_fast, b2a));
			}
			{
				for (; ; )
				{
					var m = m3x4.Random(-5.0f, +5.0f, 0, rng);
					if (!Math_.IsInvertible(m)) continue;
					var inv_m = Math_.Invert(m);
					var I0 = inv_m * m;
					var I1 = m * inv_m;

					Assert.True(Math_.FEqlRelative(I0, m3x4.Identity, 0.001f));
					Assert.True(Math_.FEqlRelative(I1, m3x4.Identity, 0.001f));
					break;
				}
			}

			{
				var m = new m3x4(
					new v4(0.25f, 0.5f, 1.0f, 0.0f),
					new v4(0.49f, 0.7f, 1.0f, 0.0f),
					new v4(1.0f, 1.0f, 1.0f, 0.0f));
				var INV_M = new m3x4(
					new v4(10.0f, -16.666667f, 6.66667f, 0.0f),
					new v4(-17.0f, 25.0f, -8.0f, 0.0f),
					new v4(7.0f, -8.333333f, 2.333333f, 0.0f));

				var inv_m = Math_.Invert(m);
				Assert.True(Math_.FEqlRelative(inv_m, INV_M, 0.001f));
			}
		}

		[Test]
		public void TestQuatConversion()
		{}

		[Test]
		public void Parse()
		{
			{
				var mat = new m3x4(
					new v4(1, 2, 3, 4),
					new v4(4, 3, 2, 1),
					new v4(4, 4, 4, 4));
				var MAT = m3x4.Parse4x3(mat.ToString4x3());
				Assert.Equals(mat, MAT);
			}
			{
				var mat = new m3x4(
					new v4(1, 2, 3, 0),
					new v4(4, 3, 2, 0),
					new v4(4, 4, 4, 0));
				var MAT = m3x4.Parse3x3(mat.ToString3x3());
				Assert.Equals(mat, MAT);
			}
		}

		[Test]
		public void LogExpMap()
		{
			var w = new v4(2, -1, 4, 0);

			// Wrap into [0, tau/2]
			var w_len = w.Length;
			w *= (w_len % Math_.TauBy2F) / w_len;

			var rot1 = m3x4.Rotation(w);
			var rot2 = Math_.ExpMap3x3(w);
			var W = Math_.LogMap(rot2);

			Assert.True(Math_.FEql(rot1, rot2));
			Assert.True(Math_.FEql(w, W));
		}
	}
}
#endif