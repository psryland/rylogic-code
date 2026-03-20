//***************************************************
// Matrix4x4
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
	/// <summary>4x4 Matrix</summary>
	[Serializable]
	[StructLayout(LayoutKind.Explicit)]
	[DebuggerDisplay("{Description,nq}")]
	public struct m4x4
	{
		// Notes:
		//  - Xform cannot represent shear, but if 'scl' is non-uniform then mathematically, multiplication should result
		//    in a transform containing shear. The standard way to handle this is to silently discard shear, so:
		//     - Scale multiplies component-wise
		//     - Rotation multiplies normally
		//     - Position is scaled, then rotated
		// - This means that:
		//       Mat4x4 * Mat4x4 != Xform * Xform, if scale is not uniform

		[FieldOffset( 0)] public v4   x;
		[FieldOffset(16)] public v4   y;
		[FieldOffset(32)] public v4   z;
		[FieldOffset(48)] public v4   w;

		[FieldOffset( 0)] public m3x4 rot;
		[FieldOffset(48)] public v4   pos;

		/// <summary>Constructors</summary>
		public m4x4(v4 x, v4 y, v4 z, v4 w) :this()
		{
			this.x = x;
			this.y = y;
			this.z = z;
			this.w = w;
		}
		public m4x4(m3x4 rot, v4 pos) :this()
		{
			this.rot = rot;
			this.pos = pos;
		}
		public m4x4(Quat q, v4 pos) :this()
		{
			this.rot = new m3x4(q);
			this.pos = pos;
		}
		public m4x4(Xform xform) :this()
		{
			this.rot = new m3x4(xform.rot) * m3x4.Scale(xform.scl.xyz);
			this.pos = xform.pos;
		}
		public m4x4(
			float xx, float xy, float xz, float xw,
			float yx, float yy, float yz, float yw,
			float zx, float zy, float zz, float zw,
			float wx, float wy, float wz, float ww) :this()
		{
			this.x = new(xx, xy, xz, xw);
			this.y = new(yx, yy, yz, yw);
			this.z = new(zx, zy, zz, zw);
			this.w = new(wx, wy, wz, ww);
		}
		public m4x4(float[] arr, int start = 0)
			:this(new v4(arr, 0), new v4(arr, 4), new v4(arr, 8), new v4(arr, 12))
		{}

		/// <summary>Get/Set columns by index</summary>
		public v4 this[int c]
		{
			get
			{
				switch (c) {
				case 0: return x;
				case 1: return y;
				case 2: return z;
				case 3: return w;
				}
				throw new ArgumentException("index out of range", "i");
			}
			set
			{
				switch (c) {
				case 0: x = value; return;
				case 1: y = value; return;
				case 2: z = value; return;
				case 3: w = value; return;
				}
				throw new ArgumentException("index out of range", "i");
			}
		}
		public v4 this[uint c]
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
				x.x, x.y, x.z, x.w,
				y.x, y.y, y.z, y.w,
				z.x, z.y, z.z, z.w,
				w.x, w.y, w.z, w.w,
			};
		}

		/// <summary>Implicit conversion to System.Numerics. Note: a2c = b2c * a2b == Matrix4x4.Multiply(A2B, B2C) == (A2B * B2C)</summary>
		public static implicit operator m4x4(Matrix4x4 m)
		{
			return new m4x4(
				new v4(m.M11, m.M12, m.M13, m.M14),
				new v4(m.M21, m.M22, m.M23, m.M24),
				new v4(m.M31, m.M32, m.M33, m.M34),
				new v4(m.M41, m.M42, m.M43, m.M44)
			);
		}
		public static implicit operator Matrix4x4(m4x4 m)
		{
			return new Matrix4x4(
				m.x.x, m.x.y, m.x.z, m.x.w,
				m.y.x, m.y.y, m.y.z, m.y.w,
				m.z.x, m.z.y, m.z.z, m.z.w,
				m.w.x, m.w.y, m.w.z, m.w.w
			);
		}

		/// <summary>Create a translation matrix</summary>
		public static m4x4 Translation(float dx, float dy, float dz)
		{
			return Translation(new v4(dx, dy, dz, 1f));
		}
		public static m4x4 Translation(v2 dxy, float dz)
		{
			return Translation(new v4(dxy, dz, 1f));
		}
		public static m4x4 Translation(v4 translation)
		{
			Debug.Assert(Math_.FEql(translation.w, 1f), "'translation' must be a position vector");
			return new(m3x4.Identity, translation);
		}

		/// <summary>Create a rotation/translation matrix</summary>
		public static m4x4 Transform(m3x4 rot, v4 translation)
		{
			return new(rot, translation);
		}
		public static m4x4 TransformRad(float pitch, float yaw, float roll, v4 translation)
		{
			return new(m3x4.RotationRad(pitch, yaw, roll), translation);
		}
		public static m4x4 TransformDeg(float pitch, float yaw, float roll, v4 translation)
		{
			return new(m3x4.RotationDeg(pitch, yaw, roll), translation);
		}
		public static m4x4 Transform(v4 axis_norm, float angle, v4 translation)
		{
			return new(m3x4.Rotation(axis_norm, angle), translation);
		}
		public static m4x4 Transform(v4 angular_displacement, v4 translation)
		{
			return new(m3x4.Rotation(angular_displacement), translation);
		}
		public static m4x4 Transform(v4 from, v4 to, v4 translation)
		{
			return new(m3x4.Rotation(from, to), translation);
		}
		public static m4x4 Transform(Quat rot, v4 translation)
		{
			return new(new m3x4(rot), translation);
		}

		/// <summary>Create a scale matrix</summary>
		public static m4x4 Scale(float s, v4 translation)
		{
			return Scale(s, s, s, translation);
		}
		public static m4x4 Scale(v3 s, v4 translation)
		{
			return Scale(s.x, s.y, s.z, translation);
		}
		public static m4x4 Scale(float sx, float sy, float sz, v4 translation)
		{
			return new(sx * v4.XAxis, sy * v4.YAxis, sz * v4.ZAxis, translation);
		}

		// Create a shear matrix
		public static m4x4 Shear(float sxy, float sxz, float syx, float syz, float szx, float szy, v4 translation)
		{
			return new(m3x4.Shear(sxy, sxz, syx, syz, szx, szy), translation);
		}

		// Orientation matrix to "look" at a point
		public static m4x4 LookAt(v4 eye, v4 at, v4 up)
		{
			Debug.Assert(eye.w == 1.0f && at.w == 1.0f && up.w == 0.0f, "Invalid position/direction vectors passed to LookAt");
			Debug.Assert(eye - at != v4.Zero, "LookAt 'eye' and 'at' positions are coincident");
			Debug.Assert(!Math_.Parallel(eye - at, up), "LookAt 'forward' and 'up' axes are aligned");
			var mat = new m4x4{};
			mat.z = Math_.Normalise(eye - at);
			mat.x = Math_.Normalise(Math_.Cross(up, mat.z));
			mat.y = Math_.Cross(mat.z, mat.x);
			mat.pos = eye;
			return mat;
		}

		// Construct an orthographic projection matrix
		public static m4x4 ProjectionOrthographic(float w, float h, float zn, float zf, bool righthanded)
		{
			Debug.Assert(Math_.IsFinite(w) && Math_.IsFinite(h) && w > 0 && h > 0, "invalid view rect");
			Debug.Assert(Math_.IsFinite(zn) && Math_.IsFinite(zf) && (zn - zf) != 0, "invalid near/far planes");
			var rh = Math_.SignF(righthanded);
			var mat = new m4x4{};
			mat.x.x = 2.0f / w;
			mat.y.y = 2.0f / h;
			mat.z.z = rh / (zn - zf);
			mat.w.w = 1.0f;
			mat.w.z = rh * zn / (zn - zf);
			return mat;
		}

		// Construct a perspective projection matrix
		public static m4x4 ProjectionPerspective(float w, float h, float zn, float zf, bool righthanded)
		{
			Debug.Assert(Math_.IsFinite(w) && Math_.IsFinite(h) && w > 0 && h > 0, "invalid view rect");
			Debug.Assert(Math_.IsFinite(zn) && Math_.IsFinite(zf) && zn > 0 && zf > 0 && (zn - zf) != 0, "invalid near/far planes");
			var rh = Math_.SignF(righthanded);
			var mat = new m4x4{};
			mat.x.x = 2.0f * zn / w;
			mat.y.y = 2.0f * zn / h;
			mat.z.w = -rh;
			mat.z.z = rh * zf / (zn - zf);
			mat.w.z = zn * zf / (zn - zf);
			return mat;
		}

		// Construct a perspective projection matrix offset from the centre
		public static m4x4 ProjectionPerspective(float l, float r, float t, float b, float zn, float zf, bool righthanded)
		{
			Debug.Assert(Math_.IsFinite(l)  && Math_.IsFinite(r) && Math_.IsFinite(t) && Math_.IsFinite(b) && (r - l) > 0 && (t - b) > 0, "invalid view rect");
			Debug.Assert(Math_.IsFinite(zn) && Math_.IsFinite(zf) && zn > 0 && zf > 0 && (zn - zf) != 0, "invalid near/far planes");
			var rh = Math_.SignF(righthanded);
			var mat = new m4x4{};
			mat.x.x = 2.0f * zn / (r - l);
			mat.y.y = 2.0f * zn / (t - b);
			mat.z.x = rh * (r + l) / (r - l);
			mat.z.y = rh * (t + b) / (t - b);
			mat.z.w = -rh;
			mat.z.z = rh * zf / (zn - zf);
			mat.w.z = zn * zf / (zn - zf);
			return mat;
		}

		// Construct a perspective projection matrix using field of view
		public static m4x4 ProjectionPerspectiveFOV(float fovY, float aspect, float zn, float zf, bool righthanded)
		{
			Debug.Assert(Math_.IsFinite(aspect) && aspect > 0, "invalid aspect ratio");
			Debug.Assert(Math_.IsFinite(zn) && Math_.IsFinite(zf) && zn > 0 && zf > 0 && (zn - zf) != 0, "invalid near/far planes");
			var rh = Math_.SignF(righthanded);
			var mat = new m4x4{};
			mat.y.y = (float)(1 / Math.Tan(fovY/2));
			mat.x.x = mat.y.y / aspect;
			mat.z.w = -rh;
			mat.z.z = rh * zf / (zn - zf);
			mat.w.z = zn * zf / (zn - zf);
			return mat;
		}

		#region Statics
		public readonly static m4x4 Zero = new(v4.Zero, v4.Zero, v4.Zero, v4.Zero);
		public readonly static m4x4 Identity = new(v4.XAxis, v4.YAxis, v4.ZAxis, v4.Origin);
		#endregion

		#region Operators
		public static m4x4 operator +(m4x4 rhs)
		{
			return rhs;
		}
		public static m4x4 operator -(m4x4 rhs)
		{
			return new(-rhs.x, -rhs.y, -rhs.z, -rhs.w);
		}
		public static m4x4 operator +(m4x4 lhs, m4x4 rhs)
		{
			return new(lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z, lhs.w + rhs.w);
		}
		public static m4x4 operator -(m4x4 lhs, m4x4 rhs)
		{
			return new(lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z, lhs.w - rhs.w);
		}
		public static m4x4 operator *(m4x4 lhs, float rhs)
		{
			return new(lhs.x * rhs, lhs.y * rhs, lhs.z * rhs, lhs.w * rhs);
		}
		public static m4x4 operator *(float lhs, m4x4 rhs)
		{
			return new(lhs * rhs.x, lhs * rhs.y, lhs * rhs.z, lhs * rhs.w);
		}
		public static m4x4 operator /(m4x4 lhs, float rhs)
		{
			return new(lhs.x / rhs, lhs.y / rhs, lhs.z / rhs, lhs.w / rhs);
		}
		public static m4x4 operator %(m4x4 lhs, float rhs)
		{
			return new(lhs.x % rhs, lhs.y % rhs, lhs.z % rhs, lhs.w % rhs);
		}
		public static m4x4 operator %(m4x4 lhs, double rhs)
		{
			return lhs % (float)rhs;
		}
		public static m4x4 operator %(m4x4 lhs, m4x4 rhs)
		{
			return new(lhs.x % rhs.x, lhs.y % rhs.y, lhs.z % rhs.z, lhs.w % rhs.w);
		}
		public static m4x4 operator *(m4x4 lhs, m4x4 rhs)
		{
			Math_.Transpose(ref lhs);
			return new(
				new v4(Math_.Dot(lhs.x, rhs.x), Math_.Dot(lhs.y, rhs.x), Math_.Dot(lhs.z, rhs.x), Math_.Dot(lhs.w, rhs.x)),
				new v4(Math_.Dot(lhs.x, rhs.y), Math_.Dot(lhs.y, rhs.y), Math_.Dot(lhs.z, rhs.y), Math_.Dot(lhs.w, rhs.y)),
				new v4(Math_.Dot(lhs.x, rhs.z), Math_.Dot(lhs.y, rhs.z), Math_.Dot(lhs.z, rhs.z), Math_.Dot(lhs.w, rhs.z)),
				new v4(Math_.Dot(lhs.x, rhs.w), Math_.Dot(lhs.y, rhs.w), Math_.Dot(lhs.z, rhs.w), Math_.Dot(lhs.w, rhs.w)));
		}
		public static v4   operator *(m4x4 lhs, v4 rhs)
		{
			Math_.Transpose(ref lhs);
			return new v4(
				Math_.Dot(lhs.x, rhs),
				Math_.Dot(lhs.y, rhs),
				Math_.Dot(lhs.z, rhs),
				Math_.Dot(lhs.w, rhs));
		}
		#endregion

		#region Equals
		public static bool operator == (m4x4 lhs, m4x4 rhs)
		{
			return lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z && lhs.w == rhs.w;
		}
		public static bool operator != (m4x4 lhs, m4x4 rhs)
		{
			return !(lhs == rhs);
		}
		public override bool Equals(object? o)
		{
			return o is m4x4 m && m == this;
		}
		public override int GetHashCode()
		{
			return new { x, y, z, w }.GetHashCode();
		}
		#endregion

		#region ToString
		public override string ToString() => ToString4x4();
		public string ToString4x4(string delim = "\n") => $"{x} {delim}{y} {delim}{z} {delim}{w} {delim}";
		public string ToString3x4(string delim = "\n") => $"{x} {delim}{y} {delim}{z} {delim}";
		public string ToString(string format, string delim = "\n") => $"{x.ToString(format)} {delim}{y.ToString(format)} {delim}{z.ToString(format)} {delim}{w.ToString(format)} {delim}";
		public string ToCodeString(ECodeString fmt = ECodeString.CSharp)
		{
			return fmt switch
			{
				ECodeString.CSharp => $"new m4x4(\nnew v4({x.ToCodeString(fmt)}),\n new v4({y.ToCodeString(fmt)}),\n new v4({z.ToCodeString(fmt)}),\n new v4({w.ToCodeString(fmt)})\n)",
				ECodeString.Cpp => $"{{\n{{{x.ToCodeString(fmt)}}},\n{{{y.ToCodeString(fmt)}}},\n{{{z.ToCodeString(fmt)}}},\n{{{w.ToCodeString(fmt)}}},\n}}",
				ECodeString.Python => $"[\n[{x.ToCodeString(fmt)}],\n[{y.ToCodeString(fmt)}],\n[{z.ToCodeString(fmt)}],\n[{w.ToCodeString(fmt)}],\n]",
				_ => ToString(),
			};
		}
		#endregion

		#region Parse
		public static m4x4 Parse(string s)
		{
			s = s ?? throw new ArgumentNullException("s", $"{nameof(Parse)}:string argument was null");
			return TryParse(s, out var result) ? result : throw new FormatException($"{nameof(Parse)}: string argument does not represent a 4x4 matrix");
		}
		public static bool TryParse(string s, out m4x4 mat, bool row_major = true)
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
					new v4(float.Parse(values[0]), float.Parse(values[1]), float.Parse(values[2]), float.Parse(values[3])),
					new v4(float.Parse(values[4]), float.Parse(values[5]), float.Parse(values[6]), float.Parse(values[7])),
					new v4(float.Parse(values[8]), float.Parse(values[9]), float.Parse(values[10]), float.Parse(values[11])),
					new v4(float.Parse(values[12]), float.Parse(values[13]), float.Parse(values[14]), float.Parse(values[15])))
				: new(
					new v4(float.Parse(values[0]), float.Parse(values[4]), float.Parse(values[8]), float.Parse(values[12])),
					new v4(float.Parse(values[1]), float.Parse(values[5]), float.Parse(values[9]), float.Parse(values[13])),
					new v4(float.Parse(values[2]), float.Parse(values[6]), float.Parse(values[10]), float.Parse(values[14])),
					new v4(float.Parse(values[3]), float.Parse(values[7]), float.Parse(values[11]), float.Parse(values[15])));
			return true;
		}
		#endregion

		#region Random

		// Create a random 4x4 matrix
		public static m4x4 Random(float min, float max, Random r)
		{
			return new(
				v4.Random4(min, max, r),
				v4.Random4(min, max, r),
				v4.Random4(min, max, r),
				v4.Random4(min, max, r));
		}

		// Create a random orthonormal transform matrix
		public static m4x4 Random(Random r)
		{
			return Random(v4.Origin, 1.0f, r);
		}
		public static m4x4 Random(v4 axis, float min_angle, float max_angle, v4 position, Random r)
		{
			return Transform(axis, r.Float(min_angle, max_angle), position);
		}
		public static m4x4 Random(float min_angle, float max_angle, v4 position, Random r)
		{
			return Random(v4.Random3N(0.0f, r), min_angle, max_angle, position, r);
		}
		public static m4x4 Random(v4 axis, float min_angle, float max_angle, v4 centre, float radius, Random r)
		{
			return Random(axis, min_angle, max_angle, centre + v4.Random3(0.0f, radius, 0.0f, r), r);
		}
		public static m4x4 Random(float min_angle, float max_angle, v4 centre, float radius, Random r)
		{
			return Random(v4.Random3N(0.0f, r), min_angle, max_angle, centre, radius, r);
		}
		public static m4x4 Random(v4 centre, float radius, Random r)
		{
			return Random(v4.Random3N(0.0f, r), 0.0f, (float)Math_.Tau, centre, radius, r);
		}

		#endregion

		/// <summary></summary>
		public string Description => $"{x.Description}  \n{y.Description}  \n{z.Description}  \n{w.Description}  \n";
	}

	public static partial class Math_
	{
		/// <summary>Approximate equal</summary>
		public static bool FEqlAbsolute(m4x4 a, m4x4 b, float tol)
		{
			return
				FEqlAbsolute(a.x, b.x, tol) &&
				FEqlAbsolute(a.y, b.y, tol) &&
				FEqlAbsolute(a.z, b.z, tol) &&
				FEqlAbsolute(a.w, b.w, tol);
		}
		public static bool FEqlRelative(m4x4 a, m4x4 b, float tol)
		{
			var max_a = MaxElement(Abs(a));
			var max_b = MaxElement(Abs(b));
			if (max_b == 0) return max_a < tol;
			if (max_a == 0) return max_b < tol;
			var abs_max_element = Max(max_a, max_b);
			return FEqlAbsolute(a, b, tol * abs_max_element);
		}
		public static bool FEql(m4x4 lhs, m4x4 rhs)
		{
			return FEqlRelative(lhs, rhs, TinyF);
		}

		/// <summary>Absolute value of 'x'</summary>
		public static m4x4 Abs(m4x4 x)
		{
			return new(
				Abs(x.x),
				Abs(x.y),
				Abs(x.z),
				Abs(x.w));
		}

		/// <summary>Return the maximum element value in 'm'</summary>
		public static float MaxElement(m4x4 m)
		{
			return Max(
				MaxElement(m.x),
				MaxElement(m.y),
				MaxElement(m.z),
				MaxElement(m.w));
		}

		/// <summary>Return the minimum element value in 'm'</summary>
		public static float MinElement(m4x4 m)
		{
			return Min(
				MinElement(m.x),
				MinElement(m.y),
				MinElement(m.z),
				MinElement(m.w));
		}

		/// <summary>Finite test of matrix elements</summary>
		public static bool IsFinite(m4x4 m)
		{
			return
				IsFinite(m.x) &&
				IsFinite(m.y) &&
				IsFinite(m.z) &&
				IsFinite(m.w);
		}

		/// <summary>Return true if any components of 'm' are NaN</summary>
		public static bool IsNaN(m4x4 m)
		{
			return
				IsNaN(m.x) ||
				IsNaN(m.y) ||
				IsNaN(m.z) ||
				IsNaN(m.w);
		}

		/// <summary>True if 'm' is orthogonal</summary>
		public static bool IsOrthogonal(m4x4 m)
		{
			return IsOrthogonal(m.rot);
		}

		/// <summary>True if the rotation part of this transform is orthonormal</summary>
		public static bool IsOrthonormal(m4x4 m)
		{
			return IsOrthonormal(m.rot);
		}

		// Return true if 'mat' is an affine transform
		public static bool IsAffine(m4x4 mat)
		{
			return
				mat.x.w == 0.0f &&
				mat.y.w == 0.0f &&
				mat.z.w == 0.0f &&
				mat.w.w == 1.0f;
		}

		/// <summary>True if 'mat' has an inverse</summary>
		public static bool IsInvertible(m4x4 m)
		{
			return Determinant(m) != 0;
		}

		/// <summary>Return the 4x4 determinant of the arbitrary transform 'mat'</summary>
		public static float Determinant(m4x4 m)
		{
			var c1 = (m.z.z * m.w.w) - (m.z.w * m.w.z);
			var c2 = (m.z.y * m.w.w) - (m.z.w * m.w.y);
			var c3 = (m.z.y * m.w.z) - (m.z.z * m.w.y);
			var c4 = (m.z.x * m.w.w) - (m.z.w * m.w.x);
			var c5 = (m.z.x * m.w.z) - (m.z.z * m.w.x);
			var c6 = (m.z.x * m.w.y) - (m.z.y * m.w.x);
			return
				m.x.x * (m.y.y * c1 - m.y.z * c2 + m.y.w * c3) -
				m.x.y * (m.y.x * c1 - m.y.z * c4 + m.y.w * c5) +
				m.x.z * (m.y.x * c2 - m.y.y * c4 + m.y.w * c6) -
				m.x.w * (m.y.x * c3 - m.y.y * c5 + m.y.z * c6);
		}

		/// <summary>Transpose the rotation part of an affine transform</summary>
		public static void Transpose3x3(ref m4x4 m)
		{
			Transpose(ref m.rot);
		}
		public static m4x4 Transpose3x3(m4x4 m)
		{
			Transpose3x3(ref m);
			return m;
		}

		/// <summary>Transpose the 4x4 matrix</summary>
		public static void Transpose(ref m4x4 m)
		{
			Swap(ref m.x.y, ref m.y.x);
			Swap(ref m.x.z, ref m.z.x);
			Swap(ref m.x.w, ref m.w.x);
			Swap(ref m.y.z, ref m.z.y);
			Swap(ref m.y.w, ref m.w.y);
			Swap(ref m.z.w, ref m.w.z);
		}
		public static m4x4 Transpose(m4x4 m)
		{
			Transpose(ref m);
			return m;
		}

		/// <summary>Orthonormalise the rotation part of an affine transform</summary>
		public static void Orthonormalise(ref m4x4 m)
		{
			Orthonormalise(ref m.rot);
		}
		public static m4x4 Orthonormalise(m4x4 m)
		{
			Orthonormalise(ref m);
			return m;
		}

		/// <summary>Invert 'm' in-place (assuming an orthonormal matrix</summary>
		public static void InvertOrthonormal(ref m4x4 m)
		{
			Debug.Assert(IsOrthonormal(m), "Matrix is not orthonormal");
			var trans = m.w;
			Transpose(ref m.rot);
			m.w.x = -(trans.x * m.x.x + trans.y * m.y.x + trans.z * m.z.x);
			m.w.y = -(trans.x * m.x.y + trans.y * m.y.y + trans.z * m.z.y);
			m.w.z = -(trans.x * m.x.z + trans.y * m.y.z + trans.z * m.z.z);
		}

		/// <summary>Return 'm' inverted (assuming an orthonormal matrix</summary>
		public static m4x4 InvertOrthonormal(m4x4 m)
		{
			InvertOrthonormal(ref m);
			return m;
		}

		/// <summary>Invert 'm' in-place (assuming an orthonormal matrix</summary>
		public static void InvertAffine(ref m4x4 m)
		{
			Debug.Assert(IsAffine(m));

			var translation = m.w;

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
			Transpose(ref m.rot);

			// Invert the scale
			m.x /= s.x;
			m.y /= s.y;
			m.z /= s.z;

			// Invert translation
			m.w.x = -(translation.x * m.x.x + translation.y * m.y.x + translation.z * m.z.x);
			m.w.y = -(translation.x * m.x.y + translation.y * m.y.y + translation.z * m.z.y);
			m.w.z = -(translation.x * m.x.z + translation.y * m.y.z + translation.z * m.z.z);
			m.w.w = 1;
		}

		/// <summary>Return 'm' inverted (assuming an orthonormal matrix</summary>
		public static m4x4 InvertAffine(m4x4 m)
		{
			InvertAffine(ref m);
			return m;
		}

		/// <summary>Return 'm' inverted</summary>
		public static m4x4 Invert(m4x4 m)
		{
			Debug.Assert(IsInvertible(m), "Matrix has no inverse");

			var A = Transpose(m); // Take the transpose so that row operations are faster
			var B = m4x4.Identity;

			// Loop through columns of 'A'
			for (int j = 0; j != 4; ++j)
			{
				// Select the pivot element: maximum magnitude in this row
				var pivot = 0; var val = 0f;
				if (j <= 0 && val < Math.Abs(A.x[j])) { pivot = 0; val = Math.Abs(A.x[j]); }
				if (j <= 1 && val < Math.Abs(A.y[j])) { pivot = 1; val = Math.Abs(A.y[j]); }
				if (j <= 2 && val < Math.Abs(A.z[j])) { pivot = 2; val = Math.Abs(A.z[j]); }
				if (j <= 3 && val < Math.Abs(A.w[j])) { pivot = 3; val = Math.Abs(A.w[j]); }
				if (val < TinyF)
				{
					Debug.Assert(false, "Matrix has no inverse");
					return m;
				}

				// Interchange rows to put pivot element on the diagonal
				if (pivot != j) // skip if already on diagonal
				{
					var a = A[j]; A[j] = A[pivot]; A[pivot] = a;
					var b = B[j]; B[j] = B[pivot]; B[pivot] = b;
				}

				// Divide row by pivot element. Pivot element becomes 1.0f
				var scale = A[j][j];
				A[j] /= scale;
				B[j] /= scale;

				// Subtract this row from others to make the rest of column j zero
				if (j != 0) { scale = A.x[j]; A.x -= scale * A[j]; B.x -= scale * B[j]; }
				if (j != 1) { scale = A.y[j]; A.y -= scale * A[j]; B.y -= scale * B[j]; }
				if (j != 2) { scale = A.z[j]; A.z -= scale * A[j]; B.z -= scale * B[j]; }
				if (j != 3) { scale = A.w[j]; A.w -= scale * A[j]; B.w -= scale * B[j]; }
			}

			// When these operations have been completed, A should have been transformed to the identity matrix
			// and B should have been transformed into the inverse of the original A
			B = Transpose(B);
			return B;
		}

		// Return the inverse of 'mat' using double precision floats
		public static m4x4 InvertPrecise(m4x4 mat)
		{
			var inv = new double[4, 4];
			inv[0, 0] = 0.0 + mat.y.y * mat.z.z * mat.w.w - mat.y.y * mat.z.w * mat.w.z - mat.z.y * mat.y.z * mat.w.w + mat.z.y * mat.y.w * mat.w.z + mat.w.y * mat.y.z * mat.z.w - mat.w.y * mat.y.w * mat.z.z;
			inv[0, 1] = 0.0 - mat.x.y * mat.z.z * mat.w.w + mat.x.y * mat.z.w * mat.w.z + mat.z.y * mat.x.z * mat.w.w - mat.z.y * mat.x.w * mat.w.z - mat.w.y * mat.x.z * mat.z.w + mat.w.y * mat.x.w * mat.z.z;
			inv[0, 2] = 0.0 + mat.x.y * mat.y.z * mat.w.w - mat.x.y * mat.y.w * mat.w.z - mat.y.y * mat.x.z * mat.w.w + mat.y.y * mat.x.w * mat.w.z + mat.w.y * mat.x.z * mat.y.w - mat.w.y * mat.x.w * mat.y.z;
			inv[0, 3] = 0.0 - mat.x.y * mat.y.z * mat.z.w + mat.x.y * mat.y.w * mat.z.z + mat.y.y * mat.x.z * mat.z.w - mat.y.y * mat.x.w * mat.z.z - mat.z.y * mat.x.z * mat.y.w + mat.z.y * mat.x.w * mat.y.z;
			inv[1, 0] = 0.0 - mat.y.x * mat.z.z * mat.w.w + mat.y.x * mat.z.w * mat.w.z + mat.z.x * mat.y.z * mat.w.w - mat.z.x * mat.y.w * mat.w.z - mat.w.x * mat.y.z * mat.z.w + mat.w.x * mat.y.w * mat.z.z;
			inv[1, 1] = 0.0 + mat.x.x * mat.z.z * mat.w.w - mat.x.x * mat.z.w * mat.w.z - mat.z.x * mat.x.z * mat.w.w + mat.z.x * mat.x.w * mat.w.z + mat.w.x * mat.x.z * mat.z.w - mat.w.x * mat.x.w * mat.z.z;
			inv[1, 2] = 0.0 - mat.x.x * mat.y.z * mat.w.w + mat.x.x * mat.y.w * mat.w.z + mat.y.x * mat.x.z * mat.w.w - mat.y.x * mat.x.w * mat.w.z - mat.w.x * mat.x.z * mat.y.w + mat.w.x * mat.x.w * mat.y.z;
			inv[1, 3] = 0.0 + mat.x.x * mat.y.z * mat.z.w - mat.x.x * mat.y.w * mat.z.z - mat.y.x * mat.x.z * mat.z.w + mat.y.x * mat.x.w * mat.z.z + mat.z.x * mat.x.z * mat.y.w - mat.z.x * mat.x.w * mat.y.z;
			inv[2, 0] = 0.0 + mat.y.x * mat.z.y * mat.w.w - mat.y.x * mat.z.w * mat.w.y - mat.z.x * mat.y.y * mat.w.w + mat.z.x * mat.y.w * mat.w.y + mat.w.x * mat.y.y * mat.z.w - mat.w.x * mat.y.w * mat.z.y;
			inv[2, 1] = 0.0 - mat.x.x * mat.z.y * mat.w.w + mat.x.x * mat.z.w * mat.w.y + mat.z.x * mat.x.y * mat.w.w - mat.z.x * mat.x.w * mat.w.y - mat.w.x * mat.x.y * mat.z.w + mat.w.x * mat.x.w * mat.z.y;
			inv[2, 2] = 0.0 + mat.x.x * mat.y.y * mat.w.w - mat.x.x * mat.y.w * mat.w.y - mat.y.x * mat.x.y * mat.w.w + mat.y.x * mat.x.w * mat.w.y + mat.w.x * mat.x.y * mat.y.w - mat.w.x * mat.x.w * mat.y.y;
			inv[2, 3] = 0.0 - mat.x.x * mat.y.y * mat.z.w + mat.x.x * mat.y.w * mat.z.y + mat.y.x * mat.x.y * mat.z.w - mat.y.x * mat.x.w * mat.z.y - mat.z.x * mat.x.y * mat.y.w + mat.z.x * mat.x.w * mat.y.y;
			inv[3, 0] = 0.0 - mat.y.x * mat.z.y * mat.w.z + mat.y.x * mat.z.z * mat.w.y + mat.z.x * mat.y.y * mat.w.z - mat.z.x * mat.y.z * mat.w.y - mat.w.x * mat.y.y * mat.z.z + mat.w.x * mat.y.z * mat.z.y;
			inv[3, 1] = 0.0 + mat.x.x * mat.z.y * mat.w.z - mat.x.x * mat.z.z * mat.w.y - mat.z.x * mat.x.y * mat.w.z + mat.z.x * mat.x.z * mat.w.y + mat.w.x * mat.x.y * mat.z.z - mat.w.x * mat.x.z * mat.z.y;
			inv[3, 2] = 0.0 - mat.x.x * mat.y.y * mat.w.z + mat.x.x * mat.y.z * mat.w.y + mat.y.x * mat.x.y * mat.w.z - mat.y.x * mat.x.z * mat.w.y - mat.w.x * mat.x.y * mat.y.z + mat.w.x * mat.x.z * mat.y.y;
			inv[3, 3] = 0.0 + mat.x.x * mat.y.y * mat.z.z - mat.x.x * mat.y.z * mat.z.y - mat.y.x * mat.x.y * mat.z.z + mat.y.x * mat.x.z * mat.z.y + mat.z.x * mat.x.y * mat.y.z - mat.z.x * mat.x.z * mat.y.y;

			var det = mat.x.x * inv[0, 0] + mat.x.y * inv[1, 0] + mat.x.z * inv[2, 0] + mat.x.w * inv[3, 0];
			Debug.Assert(det != 0, "matrix has no inverse");
			var inv_det = 1.0 / det;

			return new(
				new v4((float)(inv[0, 0] * inv_det), (float)(inv[0, 1] * inv_det), (float)(inv[0, 2] * inv_det), (float)(inv[0, 3] * inv_det)),
				new v4((float)(inv[1, 0] * inv_det), (float)(inv[1, 1] * inv_det), (float)(inv[1, 2] * inv_det), (float)(inv[1, 3] * inv_det)),
				new v4((float)(inv[2, 0] * inv_det), (float)(inv[2, 1] * inv_det), (float)(inv[2, 2] * inv_det), (float)(inv[2, 3] * inv_det)),
				new v4((float)(inv[3, 0] * inv_det), (float)(inv[3, 1] * inv_det), (float)(inv[3, 2] * inv_det), (float)(inv[3, 3] * inv_det)));
		}

		/// <summary>Permute the rotation vectors in a matrix by 'n'</summary>
		public static m4x4 Permute(m4x4 mat, int n)
		{
			switch (n % 3)
			{
			default: return mat;
			case 1: return new(mat.y, mat.z, mat.x, mat.w);
			case 2: return new(mat.z, mat.x, mat.y, mat.w);
			}
		}

		/// <summary>
		/// Make an orientation matrix from a direction. Note the rotation around the direction
		/// vector is not defined. 'axis' is the axis that 'direction' will become.</summary>
		public static m4x4 TxfmFromDir(AxisId axis, v4 direction, v4 translation, v4? up = null)
		{
			// Notes: use 'up' = Math_.Perpendicular(direction)
			Debug.Assert(FEql(translation.w, 1f), "'translation' must be a position vector");
			return new(OriFromDir(axis, direction, up), translation);
		}

		/// <summary>Spherically interpolate between two affine transforms</summary>
		public static m4x4 Slerp(m4x4 lhs, m4x4 rhs, double frac)
		{
			Debug.Assert(IsAffine(lhs));
			Debug.Assert(IsAffine(rhs));

			var q = Slerp(new Quat(lhs.rot), new Quat(rhs.rot), frac);
			var p = Lerp(lhs.pos, rhs.pos, frac);
			return new(q, p);
		}

		/// <summary>
		/// Return the cross product matrix for 'vec'. This matrix can be used to take the cross
		/// product of another vector: e.g. Cross(v1, v2) == CrossProductMatrix4x4(v1) * v2</summary>
		public static m4x4 CPM(v4 vec, v4 pos)
		{
			return new(CPM(vec), pos);
		}

		/// <summary>Return the average of a collection of affine transforms</summary>
		public static m4x4 Average(IEnumerable<m4x4> a2b)
		{
			var rot = Average(a2b.Select(x => x.rot));
			var pos = Average(a2b.Select(x => x.pos));
			return new(rot, pos);
		}
	}
}

#if PR_UNITTESTS
namespace Rylogic.UnitTests
{
	using Maths;

	[TestFixture]
	public class UnitTestM4x4
	{
		[Test]
		public void Basic()
		{
			var rng = new Random(1);
			var m = m4x4.Random(-5, +5, rng);

			Assert.True(m.x.x == m[0][0]);
			Assert.True(m.z.y == m[2][1]);
			Assert.True(m.w.x == m[3][0]);
		}

		[Test]
		public void Identity()
		{
			var m1 = m4x4.Identity;
			var m2 = m4x4.Identity;
			var m3 = m1 * m2;
			Assert.True(Math_.FEql(m3, m4x4.Identity));
		}

		[Test]
		public void Translation()
		{
			var m1 = new m4x4(v4.XAxis, v4.YAxis, v4.ZAxis, new v4(1.0f, 2.0f, 3.0f, 1.0f));
			var m2 = m4x4.Translation(new v4(1.0f, 2.0f, 3.0f, 1.0f));
			Assert.True(Math_.FEql(m1, m2));
		}

		[Test]
		public void CreateFrom()
		{
			var rnd = new Random(123456789);
			var V1 = v4.Random3(0.0f, 10.0f, 1.0f, rnd);
			var a2b = m4x4.Transform(v4.Random3N(0.0f, rnd), rnd.FloatC(0, (float)Math_.TauBy2), v4.Random3(0.0f, 10.0f, 1.0f, rnd));
			var b2c = m4x4.Transform(v4.Random3N(0.0f, rnd), rnd.FloatC(0, (float)Math_.TauBy2), v4.Random3(0.0f, 10.0f, 1.0f, rnd));
			Assert.True(Math_.IsOrthonormal(a2b));
			Assert.True(Math_.IsOrthonormal(b2c));

			var V2 = a2b * V1;
			var V3 = b2c * V2;
			var a2c = b2c * a2b;
			var V4 = a2c * V1;
			Assert.True(Math_.FEql(V3, V4));
		}

		[Test]
		public void CreateFrom2()
		{
			var m1 = m4x4.TransformRad(1.0f, 0.5f, 0.7f, v4.Origin);
			var m2 = new m4x4(new Quat(1.0f, 0.5f, 0.7f), v4.Origin);
			Assert.True(Math_.IsOrthonormal(m1));
			Assert.True(Math_.IsOrthonormal(m2));
			Assert.True(Math_.FEql(m1, m2));

			var rng = new Random(123456879);
			var ang = rng.FloatC(0.0f, 1.0f);
			var axis = v4.Random3N(0.0f, rng);
			m1 = m4x4.Transform(axis, ang, v4.Origin);
			m2 = new m4x4(new Quat(axis, ang), v4.Origin);
			Assert.True(Math_.IsOrthonormal(m1));
			Assert.True(Math_.IsOrthonormal(m2));
			Assert.True(Math_.FEql(m1, m2));
		}

		[Test]
		public void CreateFrom3()
		{
			var rng = new Random(123456789);
			var a2b = m4x4.Transform(v4.Random3N(0.0f, rng), rng.FloatC(0.0f, 1.0f), v4.Random3(0.0f, 10.0f, 1.0f, rng));

			var b2a = Math_.Invert(a2b);
			var a2a = b2a * a2b;
			Assert.True(Math_.FEql(m4x4.Identity, a2a));

			var b2a_fast = Math_.InvertAffine(a2b);
			Assert.True(Math_.FEql(b2a_fast, b2a));
		}

		[Test]
		public void Orthonormalise()
		{
			var a2b = new m4x4();
			a2b.x = new v4(-2.0f, 3.0f, 1.0f, 0.0f);
			a2b.y = new v4(4.0f, -1.0f, 2.0f, 0.0f);
			a2b.z = new v4(1.0f, -2.0f, 4.0f, 0.0f);
			a2b.w = new v4(1.0f, 2.0f, 3.0f, 1.0f);
			Assert.True(Math_.IsOrthonormal(Math_.Orthonormalise(a2b)));
		}

		[Test]
		public void Inverersion()
		{
			var rng = new Random(1);
			{
				var a2b = new m4x4(m3x4.Rotation(v4.Random3N(0.0f, rng), rng.FloatC(-5f, +5f)) * m3x4.Scale(2.0f), v4.Random3(0.0f, 10.0f, 1.0f, rng));
				Assert.True(Math_.IsAffine(a2b));

				var b2a = Math_.Invert(a2b);
				var a2a = b2a * a2b;
				Assert.True(Math_.FEql(m4x4.Identity, a2a));

				var b2a_fast = Math_.InvertAffine(a2b);
				Assert.True(Math_.FEql(b2a_fast, b2a));
			}
			{
				var a2b = new m4x4(m3x4.Rotation(v4.Random3N(0.0f, rng), rng.FloatC(-5f, +5f)), v4.Random3(0.0f, 10.0f, 1.0f, rng));
				Assert.True(Math_.IsOrthonormal(a2b));

				var b2a = Math_.Invert(a2b);
				var a2a = b2a * a2b;
				Assert.True(Math_.FEql(m4x4.Identity, a2a));

				var b2a_fast = Math_.InvertOrthonormal(a2b);
				Assert.True(Math_.FEql(b2a_fast, b2a));
			}
		}

		[Test]
		public void Parse()
		{
			{
				var mat = new m4x4(
					new v4(1, 2, 3, 4),
					new v4(4, 3, 2, 1),
					new v4(4, 4, 4, 4),
					new v4(5, 5, 5, 5));
				var MAT = m4x4.Parse(mat.ToString4x4());
				Assert.Equals(mat, MAT);
			}
			{
				var mat = new m4x4(
					new v4(1, 2, 3, 0),
					new v4(4, 3, 2, 0),
					new v4(4, 4, 4, 0),
					new v4(5, 5, 5, 1));
				var MAT = m3x4.Parse(mat.ToString3x4());
				Assert.Equals(mat.rot, MAT);
			}
		}

		[Test]
		public void NumericsConversion()
		{
			var vec0 = new Matrix4x4(1,2,3,4, 5,6,7,8, -1,-2,-3,-4, -5,-6,-7,-8);
			var vec1 = (m4x4)vec0;
			var vec2 = (Matrix4x4)vec1;
			var vec3 = (m4x4)vec2;

			Assert.True(vec0 == vec2);
			Assert.True(vec1 == vec3);
		}
	}
}
#endif