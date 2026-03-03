//*****************************************************************************
// Maths library
//  Copyright (c) Rylogic Ltd 2002
//*****************************************************************************
#pragma once
#include "pr/maths/forward.h"
#include "pr/maths/constants.h"
#include "pr/maths/maths_core.h"
#include "pr/maths/vector4.h"
#include "pr/maths/quaternion.h"
#include "pr/maths/matrix3x4.h"
#include "pr/maths/axis_id.h"

namespace pr
{
	template <Scalar S, typename A, typename B> 
	struct Mat4x4
	{
		#pragma warning(push)
		#pragma warning(disable:4201) // nameless struct
		union
		{
			struct { Vec4<S, void> x, y, z, w; };
			struct { Mat3x4<S,A,B> rot; Vec4<S,void> pos; };
			struct { Vec4<S, void> arr[4]; };
			//#if PR_MATHS_USE_INTRINSICS
			//__m128 vec[4];
			//#endif
		};
		#pragma warning(pop)

		// Notes:
		//  - Don't add Mat4x4(v4 const& v) or equivalent. It's ambiguous between being this:
		//    x = v4(v.x, v.x, v.x, v.x), y = v4(v.y, v.y, v.y, v.y), etc and
		//    x = v4(v.x, v.y, v.z, v.w), y = v4(v.x, v.y, v.z, v.w), etc...

		// Construct
		Mat4x4() = default;
		constexpr explicit Mat4x4(S x_)
			:x(x_)
			,y(x_)
			,z(x_)
			,w(x_)
		{}
		constexpr Mat4x4(Vec4<S,void> x_, Vec4<S,void> y_, Vec4<S,void> z_, Vec4<S,void> w_)
			:x(x_)
			,y(y_)
			,z(z_)
			,w(w_)
		{}
		constexpr Mat4x4(Mat3x4<S,A,B> const& rot_, Vec4<S,void> pos_)
			:rot(rot_)
			,pos(pos_)
		{
			// Don't assert 'pos.w == 1' here. Not all m4x4's are affine transforms
		}
		Mat4x4(Xform<S, void, void> const& xform) requires (std::floating_point<S>);
		//#if PR_MATHS_USE_INTRINSICS
		//Mat4x4(__m128 const (&mat)[4])
		//{
		//	vec[0] = mat[0];
		//	vec[1] = mat[1];
		//	vec[2] = mat[2];
		//	vec[3] = mat[3];
		//}
		//#endif

		// Reinterpret as a different matrix type
		template <typename C, typename D> explicit operator Mat4x4<S,C,D> const&() const
		{
			return reinterpret_cast<Mat4x4<S,C,D> const&>(*this);
		}
		template <typename C, typename D> explicit operator Mat4x4<S,C,D>&()
		{
			return reinterpret_cast<Mat4x4<S,C,D>&>(*this);
		}
		operator Mat4x4<S,void,void> const& () const
		{
			return reinterpret_cast<Mat4x4<S,void,void> const&>(*this);
		}
		operator Mat4x4<S,void,void>& ()
		{
			return reinterpret_cast<Mat4x4<S,void,void>&>(*this);
		}

		// Array access
		Vec4<S, void> const& operator [](int i) const
		{
			pr_assert("index out of range" && i >= 0 && i < _countof(arr));
			return arr[i];
		}
		Vec4<S, void>& operator [](int i)
		{
			pr_assert("index out of range" && i >= 0 && i < _countof(arr));
			return arr[i];
		}

		// Get/Set by row or column. Note: x,y,z are column vectors
		Vec4<S, void> col(int i) const
		{
			return arr[i];
		}
		Vec4<S, void> row(int i) const
		{
			return Vec4<S, void>{x[i], y[i], z[i], w[i]};
		}
		void col(int i, Vec4<S,void> col)
		{
			arr[i] = col;
		}
		void row(int i, Vec4<S,void> row)
		{
			x[i] = row.x;
			y[i] = row.y;
			z[i] = row.z;
			w[i] = row.w;
		}

		// Return the trace of this matrix
		Vec4<S, void> trace() const
		{
			return { x.x, y.y, z.z, w.w };
		}

		// Create a 4x4 matrix with this matrix as the rotation part
		Mat4x4 w0() const
		{
			return Mat4x4{rot, Vec4<S,void>{0, 0, 0, 1}};
		}
		Mat4x4 w1(Vec4<S,void> xyz) const
		{
			pr_assert("'pos' must be a position vector" && xyz.w == 1);
			return Mat4x4{rot, xyz};
		}

		// Return the scale of this matrix
		Mat4x4 scale() const
		{
			return Mat4x4{
				{Length(x.xyz), 0, 0, 0},
				{0, Length(y.xyz), 0, 0},
				{0, 0, Length(z.xyz), 0},
				{0, 0, 0, 1}
			};
		}

		// Return this matrix with the scale removed
		Mat4x4 unscaled() const
		{
			return Mat4x4{
				Normalise(x),
				Normalise(y),
				Normalise(z),
				w
			};
		}

		// Basic constants
		static constexpr Mat4x4 const& Zero()
		{
			static Mat4x4 s_zero{ Vec4<S,void>::Zero(), Vec4<S,void>::Zero(), Vec4<S,void>::Zero(), Vec4<S,void>::Zero() };
			return s_zero;
			//return Mat4x4{Vec4<S,void>::Zero(), Vec4<S,void>::Zero(), Vec4<S,void>::Zero(), Vec4<S,void>::Zero()};
		}
		static constexpr Mat4x4 const& Identity()
		{
			static Mat4x4 s_identity{ Vec4<S,void>::XAxis(), Vec4<S,void>::YAxis(), Vec4<S,void>::ZAxis(), Vec4<S,void>::Origin() };
			return s_identity;
			//return Mat4x4{Vec4<S,void>::XAxis(), Vec4<S,void>::YAxis(), Vec4<S,void>::ZAxis(), Vec4<S,void>::Origin()};
		}

		// Create a translation matrix
		static Mat4x4 Translation(Vec4<S,void> xyz)
		{
			// 'xyz' can be a position or an offset
			pr_assert("translation should be an affine vector" && (xyz.w == S(0) || xyz.w == S(1)));
			return Mat4x4(Mat3x4<S,A,B>::Identity(), xyz.w1());
		}
		static Mat4x4 Translation(S x, S y, S z)
		{
			return Translation(Vec4<S, void>{x,y,z, S(1)});
		}

		// Create a rotation matrix from Euler angles.  Order is: roll, pitch, yaw (to match DirectX)
		static Mat4x4 Transform(S pitch, S yaw, S roll, Vec4<S,void> pos)
		{
			return Mat4x4(Mat3x4<S,A,B>::Rotation(pitch, yaw, roll), pos);
		}

		// Create from an axis and angle. 'axis' should be normalised
		static Mat4x4 Transform(Vec4<S,void> axis, S angle, Vec4<S,void> pos)
		{
			pr_assert("'axis' should be normalised" && IsNormal(axis));
			return Mat4x4(Mat3x4<S,A,B>::Rotation(axis, angle), pos);
		}

		// Create from an angular displacement vector. length = angle(rad), direction = axis
		static Mat4x4 Transform(Vec4<S,void> angular_displacement, Vec4<S,void> pos)
		{
			return Mat4x4(Mat3x4<S,A,B>::Rotation(angular_displacement), pos);
		}

		// Create from rotation and translation
		static Mat4x4 Transform(Mat3x4<S,A,B> const& rot, Vec4<S,void> pos)
		{
			return Mat4x4(rot, pos);
		}

		// Create from quaternion
		static Mat4x4 Transform(Quat<S,A,B> const& q, Vec4<S,void> pos)
		{
			pr_assert("'q' should be a normalised quaternion" && IsNormal(q));
			return Mat4x4(Mat3x4<S,A,B>::Rotation(q), pos);
		}

		// Create a transform representing the rotation from one vector to another. (Vectors do not need to be normalised)
		static Mat4x4 Transform(Vec4<S,void> from, Vec4<S,void> to, Vec4<S,void> pos)
		{
			return Mat4x4(Mat3x4<S,A,B>::Rotation(from, to), pos);
		}

		// Create a transform from one basis axis to another
		static Mat4x4 Transform(AxisId from_axis, AxisId to_axis, Vec4<S,void> pos)
		{
			return Mat4x4(Mat3x4<S,A,B>::Rotation(from_axis, to_axis), pos);
		}

		// Create a scale matrix
		static Mat4x4 Scale(S scale, Vec4<S,void> pos)
		{
			return Mat4x4(Mat3x4<S,A,B>::Scale(scale), pos);
		}
		static Mat4x4 Scale(S sx, S sy, S sz, Vec4<S,void> pos)
		{
			return Mat4x4(Mat3x4<S,A,B>::Scale(sx, sy, sz), pos);
		}

		// Create a shear matrix
		static Mat4x4 Shear(S sxy, S sxz, S syx, S syz, S szx, S szy, Vec4<S,void> pos)
		{
			return Mat4x4(Mat3x4<S,A,B>::Shear(sxy, sxz, syx, syz, szx, szy), pos);
		}

		// Orientation matrix to "look" at a point
		static Mat4x4 LookAt(Vec4<S,void> eye, Vec4<S,void> at, Vec4<S,void> up)
		{
			pr_assert("Invalid position/direction vectors passed to LookAt" && eye.w == S(1) && at.w == S(1) && up.w == S(0));
			pr_assert("LookAt 'eye' and 'at' positions are coincident" && (eye - at != Vec4<S,void>{}));
			pr_assert("LookAt 'forward' and 'up' axes are aligned" && !Parallel(eye - at, up, S(0)));
			auto mat = Mat4x4{};
			mat.z = Normalise(eye - at);
			mat.x = Normalise(Cross3(up, mat.z));
			mat.y = Cross3(mat.z, mat.x);
			mat.pos = eye;
			return mat;
		}

		// Construct an orthographic projection matrix
		static Mat4x4 ProjectionOrthographic(S w, S h, S zn, S zf, bool righthanded)
		{
			pr_assert("invalid view rect" && IsFinite(w) && IsFinite(h) && w > S(0) && h > S(0));
			pr_assert("invalid near/far planes" && IsFinite(zn) && IsFinite(zf) && (zn - zf) != 0);
			auto rh = Bool2SignF(righthanded);
			auto mat = Mat4x4{};
			mat.x.x = S(2) / w;
			mat.y.y = S(2) / h;
			mat.z.z = rh / (zn - zf);
			mat.w.w = S(1);
			mat.w.z = rh * zn / (zn - zf);
			return mat;
		}

		// Construct a perspective projection matrix. 'w' and 'h' are measured at 'zn'
		static Mat4x4 ProjectionPerspective(S w, S h, S zn, S zf, bool righthanded)
		{
			// Getting your head around perspective transforms:
			//   p0 = c2s * Vec4<S,void>(0,0,-zn,1); p0/p0.w = (0,0,0,1)
			//   p1 = c2s * Vec4<S,void>(0,0,-zf,1); p1/p1.w = (0,0,1,1)

			pr_assert("invalid view rect" && IsFinite(w) && IsFinite(h) && w > S(0) && h > S(0));
			pr_assert("invalid near/far planes" && IsFinite(zn) && IsFinite(zf) && zn > S(0) && zf > S(0) && (zn - zf) != S(0));
			auto rh = Bool2SignF(righthanded);
			auto mat = Mat4x4{};
			mat.x.x = S(2) * zn / w;
			mat.y.y = S(2) * zn / h;
			mat.z.w = -rh;
			mat.z.z = rh * zf / (zn - zf);
			mat.w.z = zn * zf / (zn - zf);
			return mat;
		}

		// Construct a perspective projection matrix offset from the centre
		static Mat4x4 ProjectionPerspective(S l, S r, S t, S b, S zn, S zf, bool righthanded)
		{
			pr_assert("invalid view rect" && IsFinite(l)  && IsFinite(r) && IsFinite(t) && IsFinite(b) && (r - l) > S(0) && (t - b) > S(0));
			pr_assert("invalid near/far planes" && IsFinite(zn) && IsFinite(zf) && zn > S(0) && zf > S(0) && (zn - zf) != S(0));
			auto rh = Bool2SignF(righthanded);
			auto mat = Mat4x4{};
			mat.x.x = S(2) * zn / (r - l);
			mat.y.y = S(2) * zn / (t - b);
			mat.z.x = rh * (r + l) / (r - l);
			mat.z.y = rh * (t + b) / (t - b);
			mat.z.w = -rh;
			mat.z.z = rh * zf / (zn - zf);
			mat.w.z = zn * zf / (zn - zf);
			return mat;
		}

		// Construct a perspective projection matrix using field of view
		static Mat4x4 ProjectionPerspectiveFOV(S fovY, S aspect, S zn, S zf, bool righthanded)
		{
			pr_assert("invalid aspect ratio" && IsFinite(aspect) && aspect > S(0));
			pr_assert("invalid near/far planes" && IsFinite(zn) && IsFinite(zf) && zn > S(0) && zf > S(0) && (zn - zf) != S(0));
			auto rh = Bool2SignF(righthanded);
			auto mat = Mat4x4{};
			mat.y.y = S(1) / Tan(fovY / S(2));
			mat.x.x = mat.y.y / aspect;
			mat.z.w = -rh;
			mat.z.z = rh * zf / (zn - zf);
			mat.w.z = zn * zf / (zn - zf);
			return mat;
		}

		// Create a 4x4 matrix contains random values on the interval [min_value, max_value)
		template <typename Rng = std::default_random_engine>
		static Mat4x4 Random(Rng& rng, S min_value, S max_value)
		{
			std::uniform_real_distribution<S> dist(min_value, max_value);
			Mat4x4 m = {};
			m.x = Vec4<S,void>(dist(rng), dist(rng), dist(rng), dist(rng));
			m.y = Vec4<S,void>(dist(rng), dist(rng), dist(rng), dist(rng));
			m.z = Vec4<S,void>(dist(rng), dist(rng), dist(rng), dist(rng));
			m.w = Vec4<S,void>(dist(rng), dist(rng), dist(rng), dist(rng));
			return m;
		}

		// Create an affine transform matrix with a random rotation about 'axis', located at 'position'
		template <typename Rng = std::default_random_engine>
		static Mat4x4 Random(Rng& rng, Vec4<S,void> axis, S min_angle, S max_angle, Vec4<S,void> position)
		{
			std::uniform_real_distribution<S> dist(min_angle, max_angle);
			return Transform(axis, dist(rng), position);
		}

		// Create an affine transform matrix with a random orientation, located at 'position'
		template <typename Rng = std::default_random_engine>
		static Mat4x4 Random(Rng& rng, Vec4<S,void> position)
		{
			return Random(rng, Vec4<S,void>::RandomN(rng, S(0)), S(0), constants<S>::tau, position);
		}

		// Create an affine transform matrix with a random rotation about 'axis', located randomly within a sphere [centre, radius]
		template <typename Rng = std::default_random_engine>
		static Mat4x4 Random(Rng& rng, Vec4<S,void> axis, S min_angle, S max_angle, Vec4<S,void> centre, S radius)
		{
			return Random(rng, axis, min_angle, max_angle, centre + Vec4<S,void>::Random(rng, S(0), radius, S(0)));
		}

		// Create an affine transform matrix with a random orientation, located randomly within a sphere [centre, radius]
		template <typename Rng = std::default_random_engine>
		static Mat4x4 Random(Rng& rng, Vec4<S,void> centre, S radius)
		{
			return Random(rng, Vec4<S,void>::RandomN(rng, S(0)), S(0), constants<S>::tau, centre, radius);
		}

		#pragma region Operators
		friend constexpr Mat4x4 pr_vectorcall operator + (Mat4x4<S,A,B> const& mat)
		{
			return mat;
		}
		friend constexpr Mat4x4 pr_vectorcall operator - (Mat4x4<S,A,B> const& mat)
		{
			return Mat4x4{-mat.x, -mat.y, -mat.z, -mat.w};
		}
		friend Mat4x4 pr_vectorcall operator * (S lhs, Mat4x4<S,A,B> const& rhs)
		{
			return rhs * lhs;
		}
		friend Mat4x4 pr_vectorcall operator * (Mat4x4<S,A,B> const& lhs, S rhs)
		{
			return Mat4x4{lhs.x * rhs, lhs.y * rhs, lhs.z * rhs, lhs.w * rhs};
		}
		friend Mat4x4 pr_vectorcall operator / (Mat4x4<S,A,B> const& lhs, S rhs)
		{
			// Don't check for divide by zero by default. For floats +inf/-inf are valid results
			return Mat4x4{lhs.x / rhs, lhs.y / rhs, lhs.z / rhs, lhs.w / rhs};
		}
		friend Mat4x4 pr_vectorcall operator % (Mat4x4<S,A,B> const& lhs, S rhs)
		{
			// Don't check for divide by zero by default. For floats +inf/-inf are valid results
			if constexpr (std::floating_point<S>)
			{
				return Mat4x4{Fmod(lhs.x, rhs), Fmod(lhs.y, rhs), Fmod(lhs.z, rhs), Fmod(lhs.w, rhs)};
			}
			else
			{
				return Mat4x4{lhs.x % rhs, lhs.y % rhs, lhs.z % rhs, lhs.w % rhs};
			}
		}
		friend Mat4x4 pr_vectorcall operator + (Mat4x4<S,A,B> const& lhs, Mat4x4<S,A,B> const& rhs)
		{
			return Mat4x4{lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z, lhs.w + rhs.w};
		}
		friend Mat4x4 pr_vectorcall operator - (Mat4x4<S,A,B> const& lhs, Mat4x4<S,A,B> const& rhs)
		{
			return Mat4x4{lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z, lhs.w - rhs.w};
		}
		friend Mat4x4 pr_vectorcall operator + (Mat4x4<S,A,B> const& lhs, Mat3x4<S,A,B> const& rhs)
		{
			return Mat4x4{lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z};
		}
		friend Mat4x4 pr_vectorcall operator - (Mat4x4<S,A,B> const& lhs, Mat3x4<S,A,B> const& rhs)
		{
			return Mat4x4{lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z};
		}
		friend Vec4<S,B> pr_vectorcall operator * (Mat4x4<S,A,B> const& a2b, Vec4<S,A> v)
		{
			if constexpr (Vec4<S, void>::IntrinsicF)
			{
				auto x = _mm_load_ps(a2b.x.arr);
				auto y = _mm_load_ps(a2b.y.arr);
				auto z = _mm_load_ps(a2b.z.arr);
				auto w = _mm_load_ps(a2b.w.arr);

				auto brod1 = _mm_set_ps1(v.x);
				auto brod2 = _mm_set_ps1(v.y);
				auto brod3 = _mm_set_ps1(v.z);
				auto brod4 = _mm_set_ps1(v.w);

				auto ans = _mm_add_ps(
					_mm_add_ps(
						_mm_mul_ps(brod1, x),
						_mm_mul_ps(brod2, y)),
					_mm_add_ps(
						_mm_mul_ps(brod3, z),
						_mm_mul_ps(brod4, w)));
		
				return Vec4<S,B>{ans};
			}
			else
			{
				auto a2bT = Transpose4x4(a2b);
				return Vec4<S,B>{Dot4(a2bT.x, v), Dot4(a2bT.y, v), Dot4(a2bT.z, v), Dot4(a2bT.w, v)};
			}
		}
		template <typename C> friend Mat4x4<S,A,C> pr_vectorcall operator * (Mat4x4<S,B,C> const& b2c, Mat4x4<S,A,B> const& a2b)
		{
			// Note:
			//  - The reason for this order is because matrices are applied from right to left
			//    e.g.
			//       auto Va =             V = vector in space 'a'
			//       auto Vb =       a2b * V = vector in space 'b'
			//       auto Vc = b2c * a2b * V = vector in space 'c'
			//  - The shape of the result is:
			//       [   ]       [       ]       [   ]
			//       [a2c]       [  b2c  ]       [a2b]
			//       [1x3]   =   [  2x3  ]   *   [1x2]
			//       [   ]       [       ]       [   ]
			if constexpr (Vec4<S, A>::IntrinsicF)
			{
				auto ans = Mat4x4<S,A,C>{};
				auto x = _mm_load_ps(b2c.x.arr);
				auto y = _mm_load_ps(b2c.y.arr);
				auto z = _mm_load_ps(b2c.z.arr);
				auto w = _mm_load_ps(b2c.w.arr);
				for (int i = 0; i != 4; ++i)
				{
					auto brod1 = _mm_set_ps1(a2b.arr[i].x);
					auto brod2 = _mm_set_ps1(a2b.arr[i].y);
					auto brod3 = _mm_set_ps1(a2b.arr[i].z);
					auto brod4 = _mm_set_ps1(a2b.arr[i].w);
					auto row = _mm_add_ps(
						_mm_add_ps(
						_mm_mul_ps(brod1, x),
						_mm_mul_ps(brod2, y)),
						_mm_add_ps(
						_mm_mul_ps(brod3, z),
						_mm_mul_ps(brod4, w)));
					_mm_store_ps(ans.arr[i].arr, row);
				}
				return ans;
			}
			else
			{
				auto ans = Mat4x4<S,A,C>{};
				auto b2cT = Transpose4x4(b2c);
				ans.x = Vec4<S, void>(Dot4(b2cT.x, a2b.x), Dot4(b2cT.y, a2b.x), Dot4(b2cT.z, a2b.x), Dot4(b2cT.w, a2b.x));
				ans.y = Vec4<S, void>(Dot4(b2cT.x, a2b.y), Dot4(b2cT.y, a2b.y), Dot4(b2cT.z, a2b.y), Dot4(b2cT.w, a2b.y));
				ans.z = Vec4<S, void>(Dot4(b2cT.x, a2b.z), Dot4(b2cT.y, a2b.z), Dot4(b2cT.z, a2b.z), Dot4(b2cT.w, a2b.z));
				ans.w = Vec4<S, void>(Dot4(b2cT.x, a2b.w), Dot4(b2cT.y, a2b.w), Dot4(b2cT.z, a2b.w), Dot4(b2cT.w, a2b.w));
				return ans;
			}
		}
		#pragma endregion
	};
	#define PR_MAT4X4_CHECKS(scalar)\
	static_assert(sizeof(Mat4x4<scalar,void,void>) == 4*4*sizeof(scalar), "Mat<"#scalar"> has the wrong size");\
	static_assert(maths::Matrix4<Mat4x4<scalar,void,void>>, "Mat<"#scalar"> is not a Mat4");\
	static_assert(std::is_trivially_copyable_v<Mat4x4<scalar,void,void>>, "Mat<"#scalar"> must be a pod type");\
	static_assert(std::alignment_of_v<Mat4x4<scalar,void,void>> == std::alignment_of_v<Vec4<scalar,void>>, "Mat<"#scalar"> is not aligned correctly");
	PR_MAT4X4_CHECKS(float);
	PR_MAT4X4_CHECKS(double);
	PR_MAT4X4_CHECKS(int32_t);
	PR_MAT4X4_CHECKS(int64_t);
	#undef PR_MAT4X4_CHECKS

	// Return the determinant of the rotation part of this matrix
	template <Scalar S, typename A, typename B> inline S pr_vectorcall Determinant3(Mat4x4<S,A,B> const& mat)
	{
		return Triple(mat.x, mat.y, mat.z);
	}

	// Return the 4x4 determinant of the affine transform 'mat'
	template <Scalar S, typename A, typename B> inline S pr_vectorcall DeterminantFast4(Mat4x4<S,A,B> const& mat)
	{
		pr_assert("'mat' must be an affine transform to use this function" && IsAffine(mat));
		return
			(mat.x.x * mat.y.y * mat.z.z) +
			(mat.x.y * mat.y.z * mat.z.x) +
			(mat.x.z * mat.y.x * mat.z.y) -
			(mat.x.z * mat.y.y * mat.z.x) -
			(mat.x.y * mat.y.x * mat.z.z) -
			(mat.x.x * mat.y.z * mat.z.y);
	}

	// Return the 4x4 determinant of the arbitrary transform 'mat'
	template <Scalar S, typename A, typename B> inline S pr_vectorcall Determinant(Mat4x4<S,A,B> const& mat)
	{
		auto c1 = (mat.z.z * mat.w.w) - (mat.z.w * mat.w.z);
		auto c2 = (mat.z.y * mat.w.w) - (mat.z.w * mat.w.y);
		auto c3 = (mat.z.y * mat.w.z) - (mat.z.z * mat.w.y);
		auto c4 = (mat.z.x * mat.w.w) - (mat.z.w * mat.w.x);
		auto c5 = (mat.z.x * mat.w.z) - (mat.z.z * mat.w.x);
		auto c6 = (mat.z.x * mat.w.y) - (mat.z.y * mat.w.x);
		return
			mat.x.x * (mat.y.y*c1 - mat.y.z*c2 + mat.y.w*c3) -
			mat.x.y * (mat.y.x*c1 - mat.y.z*c4 + mat.y.w*c5) +
			mat.x.z * (mat.y.x*c2 - mat.y.y*c4 + mat.y.w*c6) -
			mat.x.w * (mat.y.x*c3 - mat.y.y*c5 + mat.y.z*c6);
	}

	// Returns the sum of the first 3 diagonal elements of 'mat'
	template <Scalar S, typename A, typename B> inline S pr_vectorcall Trace3(Mat4x4<S,A,B> const& mat)
	{
		return mat.x.x + mat.y.y + mat.z.z;
	}

	// Returns the sum of the diagonal elements of 'mat'
	template <Scalar S, typename A, typename B> inline S pr_vectorcall Trace4(Mat4x4<S,A,B> const& mat)
	{
		return mat.x.x + mat.y.y + mat.z.z + mat.w.w;
	}

	// Scale each component of 'mat' by the values in 'scale'
	template <Scalar S, typename A, typename B> inline Mat4x4<S,A,B> pr_vectorcall CompMul(Mat4x4<S,A,B> const& mat, Vec4<S,void> scale)
	{
		return Mat4x4<S,A,B>(
			mat.x * scale.x,
			mat.y * scale.y,
			mat.z * scale.z,
			mat.w * scale.w);
	}

	// The kernel of the matrix
	template <Scalar S, typename A, typename B> inline Vec4f<void> pr_vectorcall Kernel(Mat4x4<S,A,B> const& mat)
	{
		return Vec4<S,void>(mat.y.y*mat.z.z - mat.y.z*mat.z.y, -mat.y.x*mat.z.z + mat.y.z*mat.z.x, mat.y.x*mat.z.y - mat.y.y*mat.z.x, S(0));
	}

	// Return the cross product matrix for 'vec'.
	template <Scalar S, typename A> inline Mat4x4<S,A,A> pr_vectorcall CPM(Vec4<S,A> vec, Vec4<S,void> pos)
	{
		// This matrix can be used to take the cross product with
		// another vector: e.g. Cross4(v1, v2) == Cross4(v1) * v2
		return Mat4x4<S,A,A>{CPM(vec), pos};
	}

	// Return the 4x4 transpose of 'mat'
	template <Scalar S, typename A, typename B> inline Mat4x4<S,A,B> pr_vectorcall Transpose4x4(Mat4x4<S,A,B> const& mat)
	{
		if constexpr (Vec4<S, void>::IntrinsicF)
		{
			auto m = mat;
			_MM_TRANSPOSE4_PS(m.x.vec, m.y.vec, m.z.vec, m.w.vec);
			return m;
		}
		else
		{
			auto m = mat;
			std::swap(m.x.y, m.y.x);
			std::swap(m.x.z, m.z.x);
			std::swap(m.x.w, m.w.x);
			std::swap(m.y.z, m.z.y);
			std::swap(m.y.w, m.w.y);
			std::swap(m.z.w, m.w.z);
			return m;
		}
	}

	// Return the 3x3 transpose of 'mat'
	template <Scalar S, typename A, typename B> inline Mat4x4<S,A,B> pr_vectorcall Transpose3x3(Mat4x4<S,A,B> const& mat)
	{
		auto m = mat;
		std::swap(m.x.y, m.y.x);
		std::swap(m.x.z, m.z.x);
		std::swap(m.y.z, m.z.y);
		return m;
	}

	// Return true if 'mat' is an affine transform
	template <Scalar S, typename A, typename B> inline bool pr_vectorcall IsAffine(Mat4x4<S,A,B> const& mat)
	{
		return
			mat.x.w == S(0) &&
			mat.y.w == S(0) &&
			mat.z.w == S(0) &&
			mat.w.w == S(1);
	}

	// Return true if 'mat' is orthogonal
	template <Scalar S, typename A, typename B> inline bool pr_vectorcall IsOrthogonal(Mat4x4<S,A,B> const& mat)
	{
		return
			FEql(Dot(mat.x, mat.y), S(0)) &&
			FEql(Dot(mat.x, mat.z), S(0)) &&
			FEql(Dot(mat.x, mat.w), S(0)) &&
			FEql(Dot(mat.y, mat.z), S(0)) &&
			FEql(Dot(mat.y, mat.w), S(0)) &&
			FEql(Dot(mat.z, mat.w), S(0));
	}

	// Return true if this matrix is orthonormal
	template <Scalar S, typename A, typename B> inline bool pr_vectorcall IsOrthonormal(Mat4x4<S,A,B> const& mat)
	{
		return
			FEql(LengthSq(mat.x), S(1)) &&
			FEql(LengthSq(mat.y), S(1)) &&
			FEql(LengthSq(mat.z), S(1)) &&
			FEql(Abs(Determinant(mat.rot)), S(1));
	}

	// True if 'mat' has an inverse
	template <Scalar S, typename A, typename B> inline bool pr_vectorcall IsInvertible(Mat4x4<S,A,B> const& mat)
	{
		return Determinant(mat) != S(0);
	}

	// Return the inverse of 'mat'
	template <Scalar S, typename A, typename B> inline Mat4x4<S,B,A> pr_vectorcall Invert(Mat4x4<S,A,B> const& mat)
	{
		// This was lifted from MESA implementation of the GLU library
		if constexpr (true)
		{
			Mat4x4<S, B, A> inv;
			inv.x = Vec4<S, void>{
				+mat.y.y * mat.z.z * mat.w.w - mat.y.y * mat.z.w * mat.w.z - mat.z.y * mat.y.z * mat.w.w + mat.z.y * mat.y.w * mat.w.z + mat.w.y * mat.y.z * mat.z.w - mat.w.y * mat.y.w * mat.z.z,
				-mat.x.y * mat.z.z * mat.w.w + mat.x.y * mat.z.w * mat.w.z + mat.z.y * mat.x.z * mat.w.w - mat.z.y * mat.x.w * mat.w.z - mat.w.y * mat.x.z * mat.z.w + mat.w.y * mat.x.w * mat.z.z,
				+mat.x.y * mat.y.z * mat.w.w - mat.x.y * mat.y.w * mat.w.z - mat.y.y * mat.x.z * mat.w.w + mat.y.y * mat.x.w * mat.w.z + mat.w.y * mat.x.z * mat.y.w - mat.w.y * mat.x.w * mat.y.z,
				-mat.x.y * mat.y.z * mat.z.w + mat.x.y * mat.y.w * mat.z.z + mat.y.y * mat.x.z * mat.z.w - mat.y.y * mat.x.w * mat.z.z - mat.z.y * mat.x.z * mat.y.w + mat.z.y * mat.x.w * mat.y.z};
			inv.y = Vec4<S, void>{
				-mat.y.x * mat.z.z * mat.w.w + mat.y.x * mat.z.w * mat.w.z + mat.z.x * mat.y.z * mat.w.w - mat.z.x * mat.y.w * mat.w.z - mat.w.x * mat.y.z * mat.z.w + mat.w.x * mat.y.w * mat.z.z,
				+mat.x.x * mat.z.z * mat.w.w - mat.x.x * mat.z.w * mat.w.z - mat.z.x * mat.x.z * mat.w.w + mat.z.x * mat.x.w * mat.w.z + mat.w.x * mat.x.z * mat.z.w - mat.w.x * mat.x.w * mat.z.z,
				-mat.x.x * mat.y.z * mat.w.w + mat.x.x * mat.y.w * mat.w.z + mat.y.x * mat.x.z * mat.w.w - mat.y.x * mat.x.w * mat.w.z - mat.w.x * mat.x.z * mat.y.w + mat.w.x * mat.x.w * mat.y.z,
				+mat.x.x * mat.y.z * mat.z.w - mat.x.x * mat.y.w * mat.z.z - mat.y.x * mat.x.z * mat.z.w + mat.y.x * mat.x.w * mat.z.z + mat.z.x * mat.x.z * mat.y.w - mat.z.x * mat.x.w * mat.y.z};
			inv.z = Vec4<S, void>{
				+mat.y.x * mat.z.y * mat.w.w - mat.y.x * mat.z.w * mat.w.y - mat.z.x * mat.y.y * mat.w.w + mat.z.x * mat.y.w * mat.w.y + mat.w.x * mat.y.y * mat.z.w - mat.w.x * mat.y.w * mat.z.y,
				-mat.x.x * mat.z.y * mat.w.w + mat.x.x * mat.z.w * mat.w.y + mat.z.x * mat.x.y * mat.w.w - mat.z.x * mat.x.w * mat.w.y - mat.w.x * mat.x.y * mat.z.w + mat.w.x * mat.x.w * mat.z.y,
				+mat.x.x * mat.y.y * mat.w.w - mat.x.x * mat.y.w * mat.w.y - mat.y.x * mat.x.y * mat.w.w + mat.y.x * mat.x.w * mat.w.y + mat.w.x * mat.x.y * mat.y.w - mat.w.x * mat.x.w * mat.y.y,
				-mat.x.x * mat.y.y * mat.z.w + mat.x.x * mat.y.w * mat.z.y + mat.y.x * mat.x.y * mat.z.w - mat.y.x * mat.x.w * mat.z.y - mat.z.x * mat.x.y * mat.y.w + mat.z.x * mat.x.w * mat.y.y};
			inv.w = Vec4<S, void>{
				-mat.y.x * mat.z.y * mat.w.z + mat.y.x * mat.z.z * mat.w.y + mat.z.x * mat.y.y * mat.w.z - mat.z.x * mat.y.z * mat.w.y - mat.w.x * mat.y.y * mat.z.z + mat.w.x * mat.y.z * mat.z.y,
				+mat.x.x * mat.z.y * mat.w.z - mat.x.x * mat.z.z * mat.w.y - mat.z.x * mat.x.y * mat.w.z + mat.z.x * mat.x.z * mat.w.y + mat.w.x * mat.x.y * mat.z.z - mat.w.x * mat.x.z * mat.z.y,
				-mat.x.x * mat.y.y * mat.w.z + mat.x.x * mat.y.z * mat.w.y + mat.y.x * mat.x.y * mat.w.z - mat.y.x * mat.x.z * mat.w.y - mat.w.x * mat.x.y * mat.y.z + mat.w.x * mat.x.z * mat.y.y,
				+mat.x.x * mat.y.y * mat.z.z - mat.x.x * mat.y.z * mat.z.y - mat.y.x * mat.x.y * mat.z.z + mat.y.x * mat.x.z * mat.z.y + mat.z.x * mat.x.y * mat.y.z - mat.z.x * mat.x.z * mat.y.y};

			auto det = mat.x.x * inv.x.x + mat.x.y * inv.y.x + mat.x.z * inv.z.x + mat.x.w * inv.w.x;
			pr_assert("matrix has no inverse" && det != S(0));
			return inv * (S(1) / det);
		}
		else // Reference implementation
		{
			auto mA = Transpose4x4(mat); // Take the transpose so that row operations are faster
			auto mB = static_cast<Mat4x4<S, B, A>>(m4x4::Identity());

			// Loop through columns of 'A'
			for (int j = 0; j != 4; ++j)
			{
				// Select the pivot element: maximum magnitude in this row
				auto pivot = 0; auto val = S(0);
				if (j <= 0 && val < Abs(mA.x[j])) { pivot = 0; val = Abs(mA.x[j]); }
				if (j <= 1 && val < Abs(mA.y[j])) { pivot = 1; val = Abs(mA.y[j]); }
				if (j <= 2 && val < Abs(mA.z[j])) { pivot = 2; val = Abs(mA.z[j]); }
				if (j <= 3 && val < Abs(mA.w[j])) { pivot = 3; val = Abs(mA.w[j]); }
				if (val < maths::tiny<S>)
				{
					pr_assert("Matrix has no inverse" && false);
					return mat;
				}

				// Interchange rows to put pivot element on the diagonal
				if (pivot != j) // skip if already on diagonal
				{
					std::swap(mA[j], mA[pivot]);
					std::swap(mB[j], mB[pivot]);
				}

				// Divide row by pivot element. Pivot element becomes 1
				auto scale = mA[j][j];
				mA[j] /= scale;
				mB[j] /= scale;

				// Subtract this row from others to make the rest of column j zero
				if (j != 0) { scale = mA.x[j]; mA.x -= scale * mA[j]; mB.x -= scale * mB[j]; }
				if (j != 1) { scale = mA.y[j]; mA.y -= scale * mA[j]; mB.y -= scale * mB[j]; }
				if (j != 2) { scale = mA.z[j]; mA.z -= scale * mA[j]; mB.z -= scale * mB[j]; }
				if (j != 3) { scale = mA.w[j]; mA.w -= scale * mA[j]; mB.w -= scale * mB[j]; }
			}

			// When these operations have been completed, A should have been transformed to the identity matrix
			// and B should have been transformed into the inverse of the original A
			mB = Transpose4x4(mB);
			return mB;
		}
	}

	// Return the inverse of 'mat' (assuming an orthonormal matrix)
	template <Scalar S, typename A, typename B> inline Mat4x4<S,B,A> pr_vectorcall InvertAffine(Mat4x4<S,A,B> const& mat)
	{
		pr_assert("Matrix is not affine" && IsAffine(mat));
		auto m = mat;

		auto s = Vec3<S,void>(LengthSq(mat.x), LengthSq(mat.y), LengthSq(mat.z));
		if (!FEql(s, Vec3<S,void>::One()))
		{
			pr_assert(s.x != 0 && s.y != 0 && s.z != 0 && "Cannot invert a degenerate matrix");
			s = CompSqrt(s);
		}

		// Remove scale
		m.x /= s.x;
		m.y /= s.y;
		m.z /= s.z;

		// Invert rotation
		m.rot = Transpose(m.rot);

		// Invert scale
		m.x /= s.x;
		m.y /= s.y;
		m.z /= s.z;

		// Invert translation
		m.pos.x = -(mat.pos.x * m.x.x + mat.pos.y * m.y.x + mat.pos.z * m.z.x);
		m.pos.y = -(mat.pos.x * m.x.y + mat.pos.y * m.y.y + mat.pos.z * m.z.y);
		m.pos.z = -(mat.pos.x * m.x.z + mat.pos.y * m.y.z + mat.pos.z * m.z.z);

		return m;
	}
 
	// Return the inverse of 'mat' (assuming an orthonormal matrix) 
	template <Scalar S, typename A, typename B> inline Mat4x4<S,B,A> pr_vectorcall InvertOrthonormal(Mat4x4<S,A,B> const& mat) 
	{ 
		assert("Matrix is not orthonormal" && IsOrthonormal(mat)); 
		auto m = Mat4x4<S,B,A>{Transpose3x3(mat)}; 
		m.pos.x = -Dot3(mat.x, mat.pos); 
		m.pos.y = -Dot3(mat.y, mat.pos); 
		m.pos.z = -Dot3(mat.z, mat.pos); 
		return m; 
	} 

	// Return the inverse of 'mat' using double precision floats
	template <Scalar S, typename A, typename B> inline Mat4x4<S,B,A> pr_vectorcall InvertPrecise(Mat4x4<S,A,B> const& mat)
	{
		double inv[4][4];
		inv[0][0] = 0.0 + mat.y.y * mat.z.z * mat.w.w - mat.y.y * mat.z.w * mat.w.z - mat.z.y * mat.y.z * mat.w.w + mat.z.y * mat.y.w * mat.w.z + mat.w.y * mat.y.z * mat.z.w - mat.w.y * mat.y.w * mat.z.z;
		inv[0][1] = 0.0 - mat.x.y * mat.z.z * mat.w.w + mat.x.y * mat.z.w * mat.w.z + mat.z.y * mat.x.z * mat.w.w - mat.z.y * mat.x.w * mat.w.z - mat.w.y * mat.x.z * mat.z.w + mat.w.y * mat.x.w * mat.z.z;
		inv[0][2] = 0.0 + mat.x.y * mat.y.z * mat.w.w - mat.x.y * mat.y.w * mat.w.z - mat.y.y * mat.x.z * mat.w.w + mat.y.y * mat.x.w * mat.w.z + mat.w.y * mat.x.z * mat.y.w - mat.w.y * mat.x.w * mat.y.z;
		inv[0][3] = 0.0 - mat.x.y * mat.y.z * mat.z.w + mat.x.y * mat.y.w * mat.z.z + mat.y.y * mat.x.z * mat.z.w - mat.y.y * mat.x.w * mat.z.z - mat.z.y * mat.x.z * mat.y.w + mat.z.y * mat.x.w * mat.y.z;
		inv[1][0] = 0.0 - mat.y.x * mat.z.z * mat.w.w + mat.y.x * mat.z.w * mat.w.z + mat.z.x * mat.y.z * mat.w.w - mat.z.x * mat.y.w * mat.w.z - mat.w.x * mat.y.z * mat.z.w + mat.w.x * mat.y.w * mat.z.z;
		inv[1][1] = 0.0 + mat.x.x * mat.z.z * mat.w.w - mat.x.x * mat.z.w * mat.w.z - mat.z.x * mat.x.z * mat.w.w + mat.z.x * mat.x.w * mat.w.z + mat.w.x * mat.x.z * mat.z.w - mat.w.x * mat.x.w * mat.z.z;
		inv[1][2] = 0.0 - mat.x.x * mat.y.z * mat.w.w + mat.x.x * mat.y.w * mat.w.z + mat.y.x * mat.x.z * mat.w.w - mat.y.x * mat.x.w * mat.w.z - mat.w.x * mat.x.z * mat.y.w + mat.w.x * mat.x.w * mat.y.z;
		inv[1][3] = 0.0 + mat.x.x * mat.y.z * mat.z.w - mat.x.x * mat.y.w * mat.z.z - mat.y.x * mat.x.z * mat.z.w + mat.y.x * mat.x.w * mat.z.z + mat.z.x * mat.x.z * mat.y.w - mat.z.x * mat.x.w * mat.y.z;
		inv[2][0] = 0.0 + mat.y.x * mat.z.y * mat.w.w - mat.y.x * mat.z.w * mat.w.y - mat.z.x * mat.y.y * mat.w.w + mat.z.x * mat.y.w * mat.w.y + mat.w.x * mat.y.y * mat.z.w - mat.w.x * mat.y.w * mat.z.y;
		inv[2][1] = 0.0 - mat.x.x * mat.z.y * mat.w.w + mat.x.x * mat.z.w * mat.w.y + mat.z.x * mat.x.y * mat.w.w - mat.z.x * mat.x.w * mat.w.y - mat.w.x * mat.x.y * mat.z.w + mat.w.x * mat.x.w * mat.z.y;
		inv[2][2] = 0.0 + mat.x.x * mat.y.y * mat.w.w - mat.x.x * mat.y.w * mat.w.y - mat.y.x * mat.x.y * mat.w.w + mat.y.x * mat.x.w * mat.w.y + mat.w.x * mat.x.y * mat.y.w - mat.w.x * mat.x.w * mat.y.y;
		inv[2][3] = 0.0 - mat.x.x * mat.y.y * mat.z.w + mat.x.x * mat.y.w * mat.z.y + mat.y.x * mat.x.y * mat.z.w - mat.y.x * mat.x.w * mat.z.y - mat.z.x * mat.x.y * mat.y.w + mat.z.x * mat.x.w * mat.y.y;
		inv[3][0] = 0.0 - mat.y.x * mat.z.y * mat.w.z + mat.y.x * mat.z.z * mat.w.y + mat.z.x * mat.y.y * mat.w.z - mat.z.x * mat.y.z * mat.w.y - mat.w.x * mat.y.y * mat.z.z + mat.w.x * mat.y.z * mat.z.y;
		inv[3][1] = 0.0 + mat.x.x * mat.z.y * mat.w.z - mat.x.x * mat.z.z * mat.w.y - mat.z.x * mat.x.y * mat.w.z + mat.z.x * mat.x.z * mat.w.y + mat.w.x * mat.x.y * mat.z.z - mat.w.x * mat.x.z * mat.z.y;
		inv[3][2] = 0.0 - mat.x.x * mat.y.y * mat.w.z + mat.x.x * mat.y.z * mat.w.y + mat.y.x * mat.x.y * mat.w.z - mat.y.x * mat.x.z * mat.w.y - mat.w.x * mat.x.y * mat.y.z + mat.w.x * mat.x.z * mat.y.y;
		inv[3][3] = 0.0 + mat.x.x * mat.y.y * mat.z.z - mat.x.x * mat.y.z * mat.z.y - mat.y.x * mat.x.y * mat.z.z + mat.y.x * mat.x.z * mat.z.y + mat.z.x * mat.x.y * mat.y.z - mat.z.x * mat.x.z * mat.y.y;

		auto det = mat.x.x * inv[0][0] + mat.x.y * inv[1][0] + mat.x.z * inv[2][0] + mat.x.w * inv[3][0];
		pr_assert("matrix has no inverse" && det != S(0));
		auto inv_det = 1.0 / det;

		Mat4x4<S,B,A> m;
		for (int j = 0; j != 4; ++j)
			for (int i = 0; i != 4; ++i)
				m[j][i] = S(inv[j][i] * inv_det);
		return m;
	}

	// Return the square root of a matrix. The square root is the matrix B where B.B = mat.
	template <Scalar S, typename A, typename B> inline Mat4x4<S,A,B> pr_vectorcall Sqrt(Mat4x4<S,A,B> const& mat)
	{
		// Using 'Denman-Beavers' square root iteration. Should converge quadratically
		auto a = mat;                       // Converges to mat^0.5
		auto b = Mat4x4<S,A,B>::Identity(); // Converges to mat^-0.5
		for (int i = 0; i != 10; ++i)
		{
			auto a_next = S(0.5) * (a + Invert(b));
			auto b_next = S(0.5) * (b + Invert(a));
			a = a_next;
			b = b_next;
		}
		return a;
	}

	// Orthonormalises the rotation component of the matrix
	template <Scalar S, typename A, typename B> inline Mat4x4<S,A,B> pr_vectorcall Orthonorm(Mat4x4<S,A,B> const& mat)
	{
		auto m = mat;
		m.x = Normalise(m.x);
		m.y = Normalise(Cross3(m.z, m.x));
		m.z = Cross3(m.x, m.y);
		pr_assert(IsOrthonormal(m));
		return m;
	}

	// Return the axis and angle of a rotation matrix
	template <Scalar S, typename A, typename B> inline void pr_vectorcall GetAxisAngle(Mat4x4<S,A,B> const& mat, Vec4<S,void>& axis, S& angle)
	{
		GetAxisAngle(mat.rot, axis, angle);
	}

	// Make an object to world transform from a direction vector and position
	// 'dir' is the direction to align the 'axis'th axis to
	// 'up' is the preferred up direction, however if up is parallel to 'dir'
	// then a vector perpendicular to 'dir' will be chosen.
	template <Scalar S, typename A> inline Mat4x4<S,A,A> pr_vectorcall OriFromDir(Vec4<S,A> dir, AxisId axis, Vec4<S,void> up, Vec4<S,void> pos)
	{
		return Mat4x4<S,A,A>{OriFromDir(dir, axis, up), pos};
	}

	// Make a scaled object to world transform from a direction vector and position
	// Returns a transform for scaling and rotating the 'axis'th axis to 'dir'
	template <Scalar S, typename A> inline Mat4x4<S,A,A> pr_vectorcall ScaledOriFromDir(Vec4<S,A> dir, AxisId axis, Vec4<S,void> up, Vec4<S,void> pos)
	{
		return Mat4x4<S,A,A>{ScaledOriFromDir(dir, axis, up), pos};
	}

	// Return a vector representing the approximate rotation between two orthonormal transforms
	template <Scalar S, typename A, typename B> inline Vec4<S,void> pr_vectorcall RotationVectorApprox(Mat4x4<S,A,B> const& from, Mat4x4<S,A,B> const& to)
	{
		pr_assert(IsOrthonormal(from) && IsOrthonormal(to) && "This only works for orthonormal matrices");

		auto cpm_x_i2wR = to - from;
		auto w2iR = Transpose3x3(from).w0();
		auto cpm = cpm_x_i2wR * w2iR;
		return Vec4<S,void>{cpm.y.z, cpm.z.x, cpm.x.y, 0};
	}

	// Spherically interpolate between two affine transforms
	template <Scalar S, typename A, typename B> inline Mat4x4<S,A,B> pr_vectorcall Slerp(Mat4x4<S,A,B> const& lhs, Mat4x4<S,A,B> const& rhs, S frac)
	{
		pr_assert(IsAffine(lhs));
		pr_assert(IsAffine(rhs));

		auto q = Slerp(Quat<S,void>(lhs.rot), Quat<S,void>(rhs.rot), frac);
		auto p = Lerp(lhs.pos, rhs.pos, frac);
		return Mat4x4<S,A,B>{q, p};
	}
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
namespace pr::maths
{
	PRUnitTestClass(Matrix4x4Tests)
	{
		std::default_random_engine rng;
		TestClass_Matrix4x4Tests()
			:rng(1u)
		{
		}

		PRUnitTestMethod(Identity, float, double, int32_t, int64_t)
		{
			using S = T;
			using mat4_t = Mat4x4<S, void, void>;

			auto m1 = mat4_t::Identity();
			auto m2 = mat4_t::Identity();
			auto m3 = m1 * m2;
			PR_EXPECT(FEql(m3, mat4_t::Identity()));
		}
		PRUnitTestMethod(LargestSmallestElement, float, double, int32_t, int64_t)
		{
			using S = T;
			using vec4_t = Vec4<S, void>;
			using mat4_t = Mat4x4<S, void, void>;

			auto m1 = mat4_t{vec4_t{1, 2, 3, 4}, vec4_t{-2, -3, -4, -5}, vec4_t{1, 1, -1, 9}, vec4_t{-8, 5, 0, 0}};
			PR_EXPECT(MinComponent(m1) == -8);
			PR_EXPECT(MaxComponent(m1) == +9);
		}
		PRUnitTestMethod(FEql, float, double)
		{
			using S = T;
			using mat4_t = Mat4x4<S, void, void>;

			// Equal if the relative difference is less than tiny compared to the maximum element in the matrix.
			auto m1 = mat4_t::Zero();
			auto m2 = mat4_t::Zero();

			// Not equal, because the difference is 10% of the max value
			m1.x.x = S(0.10) * maths::tiny<S>;
			m2.x.x = S(0.11) * maths::tiny<S>;
			PR_EXPECT(FEql(MaxComponent(m1), m1.x.x));
			PR_EXPECT(FEql(MaxComponent(m2), m2.x.x));
			PR_EXPECT(!FEql(m1, m2));

			// Now they are equal because, compared to 1 the difference is tiny
			m1.y.y = S(1);
			m2.y.y = S(1);
			PR_EXPECT(FEql(MaxComponent(m1), m1.y.y));
			PR_EXPECT(FEql(MaxComponent(m2), m2.y.y));
			PR_EXPECT(FEql(m1, m2));
		}
		PRUnitTestMethod(Finite, float, double)
		{
			using S = T;
			using vec4_t = Vec4<S, void>;
			using mat4_t = Mat4x4<S, void, void>;

			volatile auto f0 = S(0);
			vec4_t arr0(0, 1, 10, 1);
			vec4_t arr1(0, 1, 1 / f0, 0 / f0);
			mat4_t m1(arr0, arr0, arr0, arr0);
			mat4_t m2(arr1, arr1, arr1, arr1);
			PR_EXPECT(IsFinite(m1));
			PR_EXPECT(!IsFinite(m2));
			PR_EXPECT(!All(m1, [](S x) { return x < S(5); }));
			PR_EXPECT(Any(m1, [](S x) { return x < S(5); }));
		}
		PRUnitTestMethod(MultiplyScalar, float, double, int32_t, int64_t)
		{
			using S = T;
			using vec4_t = Vec4<S, void>;
			using mat4_t = Mat4x4<S, void, void>;
			
			auto m1 = mat4_t{vec4_t{1, 2, 3, 4}, vec4_t{1, 1, 1, 1}, vec4_t{-2, -2, -2, -2}, vec4_t{4, 3, 2, 1}};
			auto m2 = S(2);
			auto m3 = mat4_t{vec4_t{2, 4, 6, 8}, vec4_t{2, 2, 2, 2}, vec4_t{-4, -4, -4, -4}, vec4_t{8, 6, 4, 2}};
			auto r = m1 * m2;
			PR_EXPECT(FEql(r, m3));
		}
		PRUnitTestMethod(MultiplyVector, float, double, int32_t, int64_t)
		{
			using S = T;
			using vec4_t = Vec4<S, void>;
			using mat4_t = Mat4x4<S, void, void>;

			auto m = mat4_t{vec4_t{1, 2, 3, 4}, vec4_t{1, 1, 1, 1}, vec4_t{-2, -2, -2, -2}, vec4_t{4, 3, 2, 1}};
			auto v = vec4_t{-3, 4, 2, -1};
			auto R = vec4_t{-7, -9, -11, -13};
			auto r = m * v;
			PR_EXPECT(FEql(r, R));
		}
		PRUnitTestMethod(MultiplyMatrix, float, double, int32_t, int64_t)
		{
			using S = T;
			using vec4_t = Vec4<S, void>;
			using mat4_t = Mat4x4<S, void, void>;

			auto m1 = mat4_t{vec4_t{1, 2, 3, 4}, vec4_t{1, 1, 1, 1}, vec4_t{-2, -2, -2, -2}, vec4_t{4, 3, 2, 1}};
			auto m2 = mat4_t{vec4_t{1, 1, 1, 1}, vec4_t{2, 2, 2, 2}, vec4_t{-1, -1, -1, -1}, vec4_t{-2, -2, -2, -2}};
			auto m3 = mat4_t{vec4_t{4, 4, 4, 4}, vec4_t{8, 8, 8, 8}, vec4_t{-4, -4, -4, -4}, vec4_t{-8, -8, -8, -8}};
			auto r = m1 * m2;
			PR_EXPECT(FEql(r, m3));
		}
		PRUnitTestMethod(ComponentMultiply, float, double, int32_t, int64_t)
		{
			using S = T;
			using vec4_t = Vec4<S, void>;
			using mat4_t = Mat4x4<S, void, void>;

			auto m1 = mat4_t{vec4_t{1, 2, 3, 4}, vec4_t{1, 1, 1, 1}, vec4_t{-2, -2, -2, -2}, vec4_t{4, 3, 2, 1}};
			auto m2 = vec4_t(2, 1, -2, -1);
			auto m3 = mat4_t{vec4_t{2, 4, 6, 8}, vec4_t{1, 1, 1, 1}, vec4_t{+4, +4, +4, +4}, vec4_t{-4, -3, -2, -1}};
			auto r = CompMul(m1, m2);
			PR_EXPECT(FEql(r, m3));
		}
		PRUnitTestMethod(Translation, float, double, int32_t, int64_t)
		{
			using S = T;
			using vec4_t = Vec4<S, void>;
			using mat4_t = Mat4x4<S, void, void>;

			auto m1 = mat4_t(vec4_t::XAxis(), vec4_t::YAxis(), vec4_t::ZAxis(), vec4_t(1, 2, 3, 1));
			auto m2 = mat4_t::Translation(vec4_t(1, 2, 3, 1));
			PR_EXPECT(FEql(m1, m2));
		}
		PRUnitTestMethod(CreateFrom, float, double)
		{
			using S = T;
			using vec4_t = Vec4<S, void>;
			using mat4_t = Mat4x4<S, void, void>;

			auto V1 = vec4_t::Random(rng, 0.0f, 10.0f, 1);
			auto a2b = mat4_t::Transform(vec4_t::Normal(+3, -2, -1, 0), +1.23f, vec4_t(+4.4f, -3.3f, +2.2f, 1.0f));
			auto b2c = mat4_t::Transform(vec4_t::Normal(-1, +2, -3, 0), -3.21f, vec4_t(-1.1f, +2.2f, -3.3f, 1.0f));
			PR_EXPECT(IsOrthonormal(a2b));
			PR_EXPECT(IsOrthonormal(b2c));
			vec4_t V2 = a2b * V1;
			vec4_t V3 = b2c * V2; V3;
			mat4_t a2c = b2c * a2b;
			vec4_t V4 = a2c * V1; V4;
			PR_EXPECT(FEql(V3, V4));
		}
		PRUnitTestMethod(CreateFrom2, float, double)
		{
			using S = T;
			using vec4_t = Vec4<S, void>;
			using quat_t = Quat<S, void, void>;
			using mat4_t = Mat4x4<S, void, void>;

			{
				auto q = quat_t(1.0f, 0.5f, 0.7f);
				mat4_t m1 = mat4_t::Transform(1.0f, 0.5f, 0.7f, vec4_t::Origin());
				mat4_t m2 = mat4_t::Transform(q, vec4_t::Origin());
				PR_EXPECT(IsOrthonormal(m1));
				PR_EXPECT(IsOrthonormal(m2));
				PR_EXPECT(FEql(m1, m2));
			}
			{
				std::uniform_real_distribution<S> dist(-1.0f, +1.0f);
				auto ang = S(dist(rng));
				vec4_t axis = vec4_t::RandomN(rng, 0);
				mat4_t m1 = mat4_t::Transform(axis, ang, vec4_t::Origin());
				mat4_t m2 = mat4_t::Transform(quat_t(axis, ang), vec4_t::Origin());
				PR_EXPECT(IsOrthonormal(m1));
				PR_EXPECT(IsOrthonormal(m2));
				PR_EXPECT(FEql(m1, m2));
			}
		}
		PRUnitTestMethod(GeneralInvert, float, double)
		{
			using S = T;
			using vec4_t = Vec4<S, void>;
			using mat4_t = Mat4x4<S, void, void>;

			mat4_t a2b = mat4_t::Random(rng, -S(5), +S(5));
			PR_EXPECT(IsInvertible(a2b));

			mat4_t b2a = Invert(a2b);
			mat4_t a2a = b2a * a2b;
			PR_EXPECT(FEql(mat4_t::Identity(), a2a));
		}
		PRUnitTestMethod(AffineInvert, float, double)
		{
			using S = T;
			using vec4_t = Vec4<S, void>;
			using mat3_t = Mat3x4<S, void, void>;
			using mat4_t = Mat4x4<S, void, void>;

			auto pos = vec4_t(-5, +3, +1, 1);
			auto rot = mat3_t::Rotation(vec4_t::Normal(-4, -3, +2, 0), -2.15f) * mat3_t::Scale(2);
			auto a2b = mat4_t::Transform(rot, pos);
			PR_EXPECT(IsAffine(a2b));
			
			auto b2a = Invert(a2b);
			auto a2a = b2a * a2b;
			PR_EXPECT(FEql(mat4_t::Identity(), a2a));

			auto b2a_fast = InvertAffine(a2b);
			PR_EXPECT(FEql(b2a_fast, b2a));
		}
		PRUnitTestMethod(OrthonormalInvert, float, double)
		{
			using S = T;
			using vec4_t = Vec4<S, void>;
			using mat3_t = Mat3x4<S, void, void>;
			using mat4_t = Mat4x4<S, void, void>;

			auto pos = vec4_t(-5, +3, +1, 1);
			auto rot = mat3_t::Rotation(vec4_t::Normal(-4, -3, +2, 0), -2.15f);
			auto a2b = mat4_t::Transform(rot, pos);
			PR_EXPECT(IsOrthonormal(a2b));
			
			auto b2a = Invert(a2b);
			auto a2a = b2a * a2b;
			PR_EXPECT(FEql(mat4_t::Identity(), a2a));

			auto b2a_fast = InvertOrthonormal(a2b);
			PR_EXPECT(FEql(b2a_fast, b2a));
		}
		PRUnitTestMethod(Orthonormalise, float, double)
		{
			using S = T;
			using vec4_t = Vec4<S, void>;
			using mat4_t = Mat4x4<S, void, void>;

			mat4_t a2b;
			a2b.x = vec4_t(-2.0f, +3.0f, +1.0f, +0.0f);
			a2b.y = vec4_t(+4.0f, -1.0f, +2.0f, +0.0f);
			a2b.z = vec4_t(+1.0f, -2.0f, +4.0f, +0.0f);
			a2b.w = vec4_t(+1.0f, +2.0f, +3.0f, +1.0f);
			PR_EXPECT(IsOrthonormal(Orthonorm(a2b)));
		}
		PRUnitTestMethod(CPM, float, double)
		{
			using S = T;
			using vec4_t = Vec4<S, void>;

			vec4_t a = {-2, 4, 2, 6};
			vec4_t b = {3, -5, 2, -4};
			auto a2b = CPM(a, vec4_t::Origin());

			vec4_t c = Cross3(a, b);
			vec4_t d = a2b * b;
			PR_EXPECT(FEql(c.xyz, d.xyz));
		}
	};
}
#endif
