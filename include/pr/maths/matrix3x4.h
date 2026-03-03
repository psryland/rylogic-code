//*****************************************************************************
// Maths library
//  Copyright (c) Rylogic Ltd 2002
//*****************************************************************************
#pragma once
#include "pr/maths/forward.h"
#include "pr/maths/constants.h"
#include "pr/maths/maths_core.h"
#include "pr/maths/vector3.h"
#include "pr/maths/vector4.h"
#include "pr/maths/quaternion.h"
#include "pr/maths/axis_id.h"

namespace pr
{
	template <Scalar S, typename A, typename B>
	struct Mat3x4
	{
		#pragma warning(push)
		#pragma warning(disable:4201) // nameless struct
		union {
		struct { Vec4<S, void> x, y, z; };
		struct { Vec4<S, void> arr[3]; };
		};
		#pragma warning(pop)

		// Construct
		Mat3x4() = default;
		constexpr explicit Mat3x4(S x_)
			:x(x_)
			,y(x_)
			,z(x_)
		{}
		constexpr Mat3x4(Vec3<S, void> x_, Vec3<S, void> y_, Vec3<S, void> z_)
			:x(x_, 0)
			,y(y_, 0)
			,z(z_, 0)
		{}
		constexpr Mat3x4(Vec4<S, void> x_, Vec4<S, void> y_, Vec4<S, void> z_)
			:x(x_)
			,y(y_)
			,z(z_)
		{}
		constexpr explicit Mat3x4(S const* v) // 12 scalars
			:Mat3x4(Vec4<S, void>(&v[0]), Vec4<S, void>(&v[4]), Vec4<S, void>(&v[8]))
		{}
		template <maths::Matrix3 M> constexpr explicit Mat3x4(M const& m)
			:Mat3x4(maths::comp<0>(m), maths::comp<1>(m), maths::comp<2>(m))
		{}
		explicit Mat3x4(Quat<S,A,B> q)
		{
			pr_assert("'quat' is a zero quaternion" && (q != Quat<S,A,B>{}));
			auto s = S(2) / LengthSq(q);

			S xs = q.x *  s, ys = q.y *  s, zs = q.z *  s;
			S wx = q.w * xs, wy = q.w * ys, wz = q.w * zs;
			S xx = q.x * xs, xy = q.x * ys, xz = q.x * zs;
			S yy = q.y * ys, yz = q.y * zs, zz = q.z * zs;
			x = Vec4<S, void>{S(1) - (yy + zz), xy + wz, xz - wy, S(0)};
			y = Vec4<S, void>{xy - wz, S(1) - (xx + zz), yz + wx, S(0)};
			z = Vec4<S, void>{xz + wy, yz - wx, S(1) - (xx + yy), S(0)};
		}

		// Reinterpret as a different matrix type
		template <typename C, typename D> explicit operator Mat3x4<S, C, D> const&() const
		{
			return reinterpret_cast<Mat3x4<S, C, D> const&>(*this);
		}
		template <typename C, typename D> explicit operator Mat3x4<S, C, D>&()
		{
			return reinterpret_cast<Mat3x4<S, C, D>&>(*this);
		}
		operator Mat3x4<S,void,void> const& () const
		{
			return reinterpret_cast<Mat3x4<S,void,void> const&>(*this);
		}
		operator Mat3x4<S,void,void>& ()
		{
			return reinterpret_cast<Mat3x4<S,void,void>&>(*this);
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
			return Vec4<S, void>{x[i], y[i], z[i], 0};
		}
		void col(int i, Vec4<S, void> col)
		{
			arr[i] = col;
		}
		void row(int i, Vec4<S, void> row)
		{
			x[i] = row.x;
			y[i] = row.y;
			z[i] = row.z;
		}

		// Return the trace of this matrix
		Vec4<S, void> trace() const
		{
			return { x.x, y.y, z.z, S{} };
		}

		// Return the scale of this matrix
		Mat3x4 scale() const
		{
			return Mat3x4{
				{Length(x.xyz), 0, 0, 0},
				{0, Length(y.xyz), 0, 0},
				{0, 0, Length(z.xyz), 0},
			};
		}

		// Return this matrix with the scale removed
		Mat3x4 unscaled() const
		{
			return Mat3x4{
				Normalise(x),
				Normalise(y),
				Normalise(z),
			};
		}

		// Basic constants
		static constexpr Mat3x4 Zero()
		{
			return Mat3x4{Vec4<S,void>::Zero(), Vec4<S,void>::Zero(), Vec4<S,void>::Zero()};
		}
		static constexpr Mat3x4 Identity()
		{
			return Mat3x4{Vec4<S,void>::XAxis(), Vec4<S,void>::YAxis(), Vec4<S,void>::ZAxis()};
		}

		// Construct a rotation matrix from a quaternion
		static Mat3x4 Rotation(Quat<S,A,B> q)
		{
			return Mat3x4(q);
		}

		// Construct a rotation matrix. Order is: roll, pitch, yaw (to match DirectX)
		static Mat3x4 Rotation(S pitch, S yaw, S roll)
		{
			S cos_p = Cos(pitch), sin_p = Sin(pitch);
			S cos_y = Cos(yaw  ), sin_y = Sin(yaw  );
			S cos_r = Cos(roll ), sin_r = Sin(roll );
			return Mat3x4{
				Vec4<S, void>( cos_y*cos_r + sin_y*sin_p*sin_r , cos_p*sin_r , -sin_y*cos_r + cos_y*sin_p*sin_r , S(0)),
				Vec4<S, void>(-cos_y*sin_r + sin_y*sin_p*cos_r , cos_p*cos_r ,  sin_y*sin_r + cos_y*sin_p*cos_r , S(0)),
				Vec4<S, void>( sin_y*cos_p                     ,      -sin_p ,                      cos_y*cos_p , S(0))};
		}

		// Create from an axis, angle
		static Mat3x4 pr_vectorcall Rotation(Vec4<S,void> axis_norm, Vec4<S,void> axis_sine_angle, S cos_angle)
		{
			pr_assert("'axis_norm' should be normalised" && IsNormal(axis_norm));

			auto m = Mat3x4{};
			auto trace_vec = axis_norm * (S(1) - cos_angle);

			m.x.x = trace_vec.x * axis_norm.x + cos_angle;
			m.y.y = trace_vec.y * axis_norm.y + cos_angle;
			m.z.z = trace_vec.z * axis_norm.z + cos_angle;

			trace_vec.x *= axis_norm.y;
			trace_vec.z *= axis_norm.x;
			trace_vec.y *= axis_norm.z;

			m.x.y = trace_vec.x + axis_sine_angle.z;
			m.x.z = trace_vec.z - axis_sine_angle.y;
			m.x.w = S(0);
			m.y.x = trace_vec.x - axis_sine_angle.z;
			m.y.z = trace_vec.y + axis_sine_angle.x;
			m.y.w = S(0);
			m.z.x = trace_vec.z + axis_sine_angle.y;
			m.z.y = trace_vec.y - axis_sine_angle.x;
			m.z.w = S(0);

			return m;
		}

		// Create from an axis and angle. 'axis' should be normalised
		static Mat3x4 pr_vectorcall Rotation(Vec4<S,void> axis_norm, S angle)
		{
			return Rotation(axis_norm, axis_norm * Sin(angle), Cos(angle));
		}

		// Create from an angular displacement vector. length = angle(rad), direction = axis
		static Mat3x4 pr_vectorcall Rotation(Vec4<S,void> angular_displacement) // This is ExpMap3x3.
		{
			// Rodrigues' formula:  exp(omega) = I + (sin(theta)/theta) * omega + ((1 - cos(theta)/theta²) * omega²
			pr_assert("'angular_displacement' should be a scaled direction vector" && angular_displacement.w == S(0));
			auto len = Length(angular_displacement);
			return len > maths::tiny<S>
				? Mat3x4::Rotation(angular_displacement / len, len)
				: Mat3x4::Identity();
		}

		// Create a transform representing the rotation from one vector to another. (Vectors do not need to be normalised)
		static Mat3x4 pr_vectorcall Rotation(Vec4<S,void> from, Vec4<S,void> to)
		{
			pr_assert(!FEql(from, Vec4<S,void>{}));
			pr_assert(!FEql(to  , Vec4<S,void>{}));
			auto len = Length(from) * Length(to);

			auto cos_angle = Dot3(from, to) / len;
			if (cos_angle >= 1.0f - maths::tiny<S>) return Identity();
			if (cos_angle <= maths::tiny<S> - S(1)) return Rotation(Normalise(Perpendicular(from - to)), constants<S>::tau_by_2);

			auto axis_size_angle = Cross3(from, to) / len;
			auto axis_norm = Normalise(axis_size_angle);
			return Rotation(axis_norm, axis_size_angle, cos_angle);
		}

		// Create a transform from one basis axis to another. Remember AxisId can be cast to Vec4
		static Mat3x4 Rotation(AxisId from_axis, AxisId to_axis)
		{
			// 'o2f' = the rotation from Z to 'from_axis'
			// 'o2t' = the rotation from Z to 'to_axis'
			// 'f2t' = o2t * Invert(o2f)
			Mat3x4 o2f, o2t;
			switch (from_axis)
			{
				case -1: o2f = Mat3x4::Rotation(S(0), +constants<S>::tau_by_4, S(0)); break;
				case +1: o2f = Mat3x4::Rotation(S(0), -constants<S>::tau_by_4, S(0)); break;
				case -2: o2f = Mat3x4::Rotation(+constants<S>::tau_by_4, S(0), S(0)); break;
				case +2: o2f = Mat3x4::Rotation(-constants<S>::tau_by_4, S(0), S(0)); break;
				case -3: o2f = Mat3x4::Rotation(S(0), +constants<S>::tau_by_2, S(0)); break;
				case +3: o2f = Identity(); break;
				default: pr_assert(false && "axis_id must one of +/-1, +/-2, +/-3"); o2f = Identity(); break;
			}
			switch (to_axis)
			{
				case -1: o2t = Mat3x4::Rotation(S(0), -constants<S>::tau_by_4, S(0)); break; // I know this sign looks wrong, but it isn't. Must be something to do with signs passed to cos()/sin()
				case +1: o2t = Mat3x4::Rotation(S(0), +constants<S>::tau_by_4, S(0)); break;
				case -2: o2t = Mat3x4::Rotation(+constants<S>::tau_by_4, S(0), S(0)); break;
				case +2: o2t = Mat3x4::Rotation(-constants<S>::tau_by_4, S(0), S(0)); break;
				case -3: o2t = Mat3x4::Rotation(S(0), +constants<S>::tau_by_2, S(0)); break;
				case +3: o2t = Identity(); break;
				default: pr_assert(false && "axis_id must one of +/-1, +/-2, +/-3"); o2t = Identity(); break;
			}
			return o2t * InvertAffine(o2f);
		}

		// Create a scale matrix
		static Mat3x4 Scale(S scale)
		{
			Mat3x4 mat = {};
			mat.x.x = mat.y.y = mat.z.z = scale;
			return mat;
		}
		static Mat3x4 Scale(S sx, S sy, S sz)
		{
			Mat3x4 mat = {};
			mat.x.x = sx;
			mat.y.y = sy;
			mat.z.z = sz;
			return mat;
		}
		static Mat3x4 Scale(Vec3<S, void> scale)
		{
			Mat3x4 mat = {};
			mat.x.x = scale.x;
			mat.y.y = scale.y;
			mat.z.z = scale.z;
			return mat;
		}

		// Create a shear matrix
		static Mat3x4 Shear(S sxy, S sxz, S syx, S syz, S szx, S szy)
		{
			Mat3x4 mat = {};
			mat.x = Vec4<S,void>(S(1), sxy, sxz, S(0));
			mat.y = Vec4<S,void>(syx, S(1), syz, S(0));
			mat.z = Vec4<S,void>(szx, szy, S(1), S(0));
			return mat;
		}

		// Create a 3x4 matrix containing random values on the interval [min_value, max_value)
		template <typename Rng = std::default_random_engine> static Mat3x4 Random(Rng& rng, S min_value, S max_value)
		{
			std::uniform_real_distribution<S> dist(min_value, max_value);
			Mat3x4 m = {};
			m.x = Vec4<S,void>(dist(rng), dist(rng), dist(rng), dist(rng));
			m.y = Vec4<S,void>(dist(rng), dist(rng), dist(rng), dist(rng));
			m.z = Vec4<S,void>(dist(rng), dist(rng), dist(rng), dist(rng));
			return m;
		}

		// Create a random 3D rotation matrix
		template <typename Rng = std::default_random_engine> static Mat3x4 Random(Rng& rng, Vec4<S,void> axis, S min_angle, S max_angle)
		{
			std::uniform_real_distribution<S> dist(min_angle, max_angle);
			return Rotation(axis, dist(rng));
		}

		// Create a random 3D rotation matrix
		template <typename Rng = std::default_random_engine> static Mat3x4 Random(Rng& rng)
		{
			return Random(rng, Vec4<S,void>::RandomN(rng, S(0)), S(0), constants<S>::tau);
		}

		#pragma region Operators
		friend constexpr Mat3x4 pr_vectorcall operator + (Mat3x4<S,A,B> const& mat)
		{
			return mat;
		}
		friend constexpr Mat3x4 pr_vectorcall operator - (Mat3x4<S,A,B> const& mat)
		{
			return Mat3x4{-mat.x, -mat.y, -mat.z};
		}
		friend Mat3x4 pr_vectorcall operator * (S lhs, Mat3x4<S,A,B> const& rhs)
		{
			return rhs * lhs;
		}
		friend Mat3x4 pr_vectorcall operator * (Mat3x4<S,A,B> const& lhs, S rhs)
		{
			return Mat3x4{lhs.x * rhs, lhs.y * rhs, lhs.z * rhs};
		}
		friend Mat3x4 pr_vectorcall operator / (Mat3x4<S,A,B> const& lhs, S rhs)
		{
			// Don't check for divide by zero by default. For floats +inf/-inf are valid results
			//pr_assert("divide by zero" && rhs != 0);
			return Mat3x4{lhs.x / rhs, lhs.y / rhs, lhs.z / rhs};
		}
		friend Mat3x4 pr_vectorcall operator % (Mat3x4<S,A,B> const& lhs, S rhs)
		{
			// Don't check for divide by zero by default. For floats +inf/-inf are valid results
			//pr_assert("divide by zero" && rhs != 0);
			return Mat3x4{lhs.x % rhs, lhs.y % rhs, lhs.z % rhs};
		}
		friend Mat3x4 pr_vectorcall operator + (Mat3x4<S,A,B> const& lhs, Mat3x4<S,A,B> const& rhs)
		{
			return Mat3x4{lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z};
		}
		friend Mat3x4 pr_vectorcall operator - (Mat3x4<S,A,B> const& lhs, Mat3x4<S,A,B> const& rhs)
		{
			return Mat3x4{lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z};
		}
		friend Vec4<S, B> pr_vectorcall operator * (Mat3x4<S,A,B> const& lhs, Vec4<S, A> rhs)
		{
			if constexpr (Vec4<S, A>::IntrinsicF)
			{
				auto x = _mm_load_ps(lhs.x.arr);
				auto y = _mm_load_ps(lhs.y.arr);
				auto z = _mm_load_ps(lhs.z.arr);

				auto brod1 = _mm_set_ps(0, rhs.x, rhs.x, rhs.x);
				auto brod2 = _mm_set_ps(0, rhs.y, rhs.y, rhs.y);
				auto brod3 = _mm_set_ps(0, rhs.z, rhs.z, rhs.z);

				auto ans = _mm_add_ps(
					_mm_add_ps(
					_mm_mul_ps(brod1, x),
					_mm_mul_ps(brod2, y)),
					_mm_add_ps(
					_mm_mul_ps(brod3, z),
					_mm_set_ps(rhs.w, 0, 0, 0))
				);
				return Vec4<S, B>{ans};
			}
			else
			{
				auto lhsT = Transpose(lhs);
				return Vec4<S, B>{Dot3(lhsT.x, rhs), Dot3(lhsT.y, rhs), Dot3(lhsT.z, rhs), rhs.w};
			}
		}
		friend Vec3<S,B> pr_vectorcall operator * (Mat3x4<S,A,B> const& lhs, Vec3<S,A> rhs)
		{
			if constexpr (Vec4<S, A>::IntrinsicF)
			{
				auto x = _mm_load_ps(lhs.x.arr);
				auto y = _mm_load_ps(lhs.y.arr);
				auto z = _mm_load_ps(lhs.z.arr);

				auto brod1 = _mm_set_ps(0, rhs.x, rhs.x, rhs.x);
				auto brod2 = _mm_set_ps(0, rhs.y, rhs.y, rhs.y);
				auto brod3 = _mm_set_ps(0, rhs.z, rhs.z, rhs.z);

				auto ans = _mm_add_ps(
					_mm_add_ps(
					_mm_mul_ps(brod1, x),
					_mm_mul_ps(brod2, y)),
					_mm_mul_ps(brod3, z));

				return Vec3<S, B>{ans.m128_f32[0], ans.m128_f32[1], ans.m128_f32[2]};
			}
			else
			{
				auto lhsT = Transpose(lhs);
				return Vec3<S, B>{Dot(lhsT.x.xyz, rhs), Dot(lhsT.y.xyz, rhs), Dot(lhsT.z.xyz, rhs)};
			}
		}
		template <typename C> friend Mat3x4<S,A,C> pr_vectorcall operator * (Mat3x4<S,B,C> const& lhs, Mat3x4<S,A,B> const& rhs)
		{
			if constexpr (Vec4<S, A>::IntrinsicF)
			{
				auto ans = Mat3x4<S, A, C>{};
				auto x = _mm_load_ps(lhs.x.arr);
				auto y = _mm_load_ps(lhs.y.arr);
				auto z = _mm_load_ps(lhs.z.arr);
				for (int i = 0; i != 3; ++i)
				{
					auto brod1 = _mm_set_ps(0, rhs.arr[i].x, rhs.arr[i].x, rhs.arr[i].x);
					auto brod2 = _mm_set_ps(0, rhs.arr[i].y, rhs.arr[i].y, rhs.arr[i].y);
					auto brod3 = _mm_set_ps(0, rhs.arr[i].z, rhs.arr[i].z, rhs.arr[i].z);
					auto row = _mm_add_ps(
						_mm_add_ps(
						_mm_mul_ps(brod1, x),
						_mm_mul_ps(brod2, y)),
						_mm_mul_ps(brod3, z));

					_mm_store_ps(ans.arr[i].arr, row);
				}
				return ans;
			}
			else
			{
				auto ans = Mat3x4<S, A, C>{};
				auto lhsT = Transpose(lhs);
				ans.x = Vec4<S, void>{Dot3(lhsT.x, rhs.x), Dot3(lhsT.y, rhs.x), Dot3(lhsT.z, rhs.x), S(0)};
				ans.y = Vec4<S, void>{Dot3(lhsT.x, rhs.y), Dot3(lhsT.y, rhs.y), Dot3(lhsT.z, rhs.y), S(0)};
				ans.z = Vec4<S, void>{Dot3(lhsT.x, rhs.z), Dot3(lhsT.y, rhs.z), Dot3(lhsT.z, rhs.z), S(0)};
				return ans;
			}
		}
		#pragma endregion
	};
	#define PR_MAT3X4_CHECKS(scalar)\
	static_assert(sizeof(Mat3x4<scalar,void,void>) == 3*4*sizeof(scalar), "Mat3x4<"#scalar"> has the wrong size");\
	static_assert(maths::Matrix3<Mat3x4<scalar,void,void>>, "Mat3x4<"#scalar"> is not a Mat3x4");\
	static_assert(std::is_trivially_copyable_v<Mat3x4<scalar,void,void>>, "Mat3x4<"#scalar"> must be a pod type");\
	static_assert(std::alignment_of_v<Mat3x4<scalar,void,void>> == std::alignment_of_v<Vec4<scalar,void>>, "Mat3x4<"#scalar"> is not aligned correctly");
	PR_MAT3X4_CHECKS(float);
	PR_MAT3X4_CHECKS(double);
	PR_MAT3X4_CHECKS(int32_t);
	PR_MAT3X4_CHECKS(int64_t);
	#undef PR_MAT3X4_CHECKS

	// Create a quaternion from a rotation matrix
	template <Scalar S, typename A, typename B> Quat<S,A,B>::Quat(Mat3x4<S,A,B> const& m)
	{
		pr_assert("Only orientation matrices can be converted into quaternions" && IsOrthonormal(m));
		if (m.x.x + m.y.y + m.z.z >= 0)
		{
			auto s = S(0.5) * Rsqrt1(S(1.0) + m.x.x + m.y.y + m.z.z);
			x = (m.y.z - m.z.y) * s;
			y = (m.z.x - m.x.z) * s;
			z = (m.x.y - m.y.x) * s;
			w = (S(0.25) / s);
		}
		else if (m.x.x > m.y.y && m.x.x > m.z.z)
		{
			auto s = S(0.5) * Rsqrt1(S(1.0) + m.x.x - m.y.y - m.z.z);
			x = (S(0.25) / s);
			y = (m.x.y + m.y.x) * s;
			z = (m.z.x + m.x.z) * s;
			w = (m.y.z - m.z.y) * s;
		}
		else if (m.y.y > m.z.z)
		{
			auto s = S(0.5) * Rsqrt1(S(1.0) - m.x.x + m.y.y - m.z.z);
			x = (m.x.y + m.y.x) * s;
			y = (S(0.25) / s);
			z = (m.y.z + m.z.y) * s;
			w = (m.z.x - m.x.z) * s;
		}
		else
		{
			auto s = S(0.5) * Rsqrt1(S(1.0) - m.x.x - m.y.y + m.z.z);
			x = (m.z.x + m.x.z) * s;
			y = (m.y.z + m.z.y) * s;
			z = (S(0.25) / s);
			w = (m.x.y - m.y.x) * s;
		}
	}

	// Return the determinant of 'mat'
	template <Scalar S, typename A, typename B> inline S pr_vectorcall Determinant(Mat3x4<S,A,B> const& mat)
	{
		return Triple(mat.x, mat.y, mat.z);
	}

	// Return the trace of 'mat'
	template <Scalar S, typename A, typename B> inline S pr_vectorcall Trace(Mat3x4<S,A,B> const& mat)
	{
		return mat.x.x + mat.y.y + mat.z.z;
	}

	// Return the kernel of 'mat'
	template <Scalar S, typename A, typename B> inline Vec4<S, void> pr_vectorcall Kernel(Mat3x4<S,A,B> const& mat)
	{
		return Vec4<S,void>{mat.y.y*mat.z.z - mat.y.z*mat.z.y, -mat.y.x*mat.z.z + mat.y.z*mat.z.x, mat.y.x*mat.z.y - mat.y.y*mat.z.x, S(0)};
	}

	// Return the diagonal elements of 'mat'
	template <Scalar S, typename A, typename B> inline Vec4f<void> pr_vectorcall Diagonal(Mat3x4<S,A,B> const& mat)
	{
		return Vec4<S,void>{mat.x.x, mat.y.y, mat.z.z, S(0)};
	}

	// Return the transpose of 'mat'
	template <Scalar S, typename A, typename B> inline Mat3x4<S,A,B> pr_vectorcall Transpose(Mat3x4<S,A,B> const& mat)
	{
		auto m = mat;
		std::swap(m.x.y, m.y.x);
		std::swap(m.x.z, m.z.x);
		std::swap(m.y.z, m.z.y);
		return m;
	}

	// Return true if 'mat' is an affine transform
	template <Scalar S, typename A, typename B> inline bool pr_vectorcall IsAffine(Mat3x4<S,A,B> const& mat)
	{
		return
			mat.x.w == S(0) &&
			mat.y.w == S(0) &&
			mat.z.w == S(0);
	}

	// Return true if 'mat' is orthogonal
	template <Scalar S, typename A, typename B> inline bool pr_vectorcall IsOrthogonal(Mat3x4<S,A,B> const& mat)
	{
		return
			FEql(Dot(mat.x, mat.y), S(0)) &&
			FEql(Dot(mat.x, mat.z), S(0)) &&
			FEql(Dot(mat.y, mat.z), S(0));
	}

	// Return true if 'mat' is orthonormal
	template <Scalar S, typename A, typename B> inline bool pr_vectorcall IsOrthonormal(Mat3x4<S,A,B> const& mat)
	{
		return
			FEql(LengthSq(mat.x), S(1)) &&
			FEql(LengthSq(mat.y), S(1)) &&
			FEql(LengthSq(mat.z), S(1)) &&
			FEql(Abs(Determinant(mat)), S(1));
	}

	// True if 'mat' can be inverted
	template <Scalar S, typename A, typename B> inline bool pr_vectorcall IsInvertible(Mat3x4<S,A,B> const& mat)
	{
		return Determinant(mat) != S(0);
	}

	// True if 'mat' is symmetric
	template <Scalar S, typename A, typename B> inline bool pr_vectorcall IsSymmetric(Mat3x4<S,A,B> const& mat)
	{
		return
			FEql(mat.x.y, mat.y.x) &&
			FEql(mat.x.z, mat.z.x) &&
			FEql(mat.y.z, mat.z.y);
	}

	// True if 'mat' is anti-symmetric
	template <Scalar S, typename A, typename B> inline bool pr_vectorcall IsAntiSymmetric(Mat3x4<S,A,B> const& mat)
	{
		return
			FEql(mat.x.y, -mat.y.x) &&
			FEql(mat.x.z, -mat.z.x) &&
			FEql(mat.y.z, -mat.z.y);
	}

	// Invert the 3x3 matrix 'mat'
	template <Scalar S, typename A, typename B> inline Mat3x4<S,B,A> pr_vectorcall Invert(Mat3x4<S,A,B> const& mat)
	{
		pr_assert("Matrix has no inverse" && IsInvertible(mat));
		auto det = Determinant(mat);
		Mat3x4<S,B,A> tmp = {};
		tmp.x = Cross3(mat.y, mat.z) / det;
		tmp.y = Cross3(mat.z, mat.x) / det;
		tmp.z = Cross3(mat.x, mat.y) / det;
		return Transpose(tmp);
	}

	// Invert the orthonormal matrix 'mat'
	template <Scalar S, typename A, typename B> inline Mat3x4<S,B,A> pr_vectorcall InvertAffine(Mat3x4<S,A,B> const& mat)
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
		m = Transpose(m);

		// Invert scale
		m.x /= s.x;
		m.y /= s.y;
		m.z /= s.z;

		return m;
	}
 
	// Invert the orthonormal matrix 'mat' 
	template <Scalar S, typename A, typename B> inline Mat3x4<S,B,A> pr_vectorcall InvertOrthonormal(Mat3x4<S,A,B> const& mat) 
	{ 
		assert("Matrix is not orthonormal" && IsOrthonormal(mat)); 
		return static_cast<Mat3x4<S,B,A>>(Transpose(mat)); 
	} 

	// Return the square root of a matrix. The square root is the matrix B where B.B = mat.
	template <Scalar S, typename A, typename B> inline Mat3x4<S,A,B> pr_vectorcall Sqrt(Mat3x4<S,A,B> const& mat)
	{
		// Using 'Denman-Beavers' square root iteration. Should converge quadratically
		auto a = mat;                       // Converges to mat^0.5
		auto b = Mat3x4<S,A,B>::Identity(); // Converges to mat^-0.5
		for (int i = 0; i != 10; ++i)
		{
			auto a_next = S(0.5) * (a + Invert(b));
			auto b_next = S(0.5) * (b + Invert(a));
			a = a_next;
			b = b_next;
		}
		return a;
	}
	
	// Normalise the columns of 'm' returning the lengths prior to renormalising
	template <Scalar S, typename A, typename B> inline std::tuple<Mat3x4<S,A,B>, Vec3<S,void>> Normalise(Mat3x4<S,A,B> const& mat)
	{
		auto m = mat;
		Vec3<S, void> scale = { Length(m.x), Length(m.y), Length(m.z) };
		m.x /= scale.x;
		m.y /= scale.y;
		m.z /= scale.z;
		return { m, scale };
	}

	// Orthonormalise the rotation component of 'mat'
	template <Scalar S, typename A, typename B> inline Mat3x4<S,A,B> pr_vectorcall Orthonorm(Mat3x4<S,A,B> const& mat)
	{
		auto m = mat;
		m.x = Normalise(m.x);
		m.y = Normalise(Cross3(m.z, m.x));
		m.z = Cross3(m.x, m.y);
		return m;
	}

	// Return the axis and angle of a rotation matrix
	template <Scalar S, typename A, typename B> inline void pr_vectorcall GetAxisAngle(Mat3x4<S,A,B> const& mat, Vec4<S,void>& axis, S& angle)
	{
		pr_assert("Matrix is not a pure rotation matrix" && IsOrthonormal(mat));

		angle = ACos(S(0.5) * (Trace(mat) - S(1)));
		axis = S(1000) * Kernel(Mat3x4<S,A,B>::Identity() - mat);
		if (axis == Vec4<S,void>{})
		{
			axis = Vec4<S,void>{1, 0, 0, 0};
			angle = S(0);
			return;
		}
		
		axis = Normalise(axis);
		if (axis == Vec4<S,void>{})
		{
			axis = Vec4<S,void>{1, 0, 0, 0};
			angle = S(0);
			return;
		}

		// Determine the correct sign of the angle
		auto vec = CreateNotParallelTo(axis);
		auto X = vec - Dot3(axis, vec) * axis;
		auto Xprim = mat * X;
		auto XcXp = Cross3(X, Xprim);
		if (Dot3(XcXp, axis) < S(0))
			angle = -angle;
	}

	// Diagonalise a 3x3 matrix. From numerical recipes
	template <Scalar S, typename A, typename B> inline Mat3x4<S,A,B> pr_vectorcall Diagonalise3x3(Mat3x4<S,A,B> const& mat_, Mat3x4<S,A,B>& eigen_vectors, Vec4<S,void>& eigen_values)
	{
		struct L {
		static void Rotate(Mat3x4<S,A,B>& mat, int i, int j, int k, int l, S s, S tau)
		{
			auto temp = mat[j][i];
			auto h    = mat[l][k];
			mat[j][i] = temp - s * (h + temp * tau);
			mat[l][k] = h    + s * (temp - h * tau);
		}};

		// Initialise the Eigen values and b to be the diagonal elements of 'mat'
		Vec4<S, void> b;
		eigen_values.x = b.x = mat_.x.x;
		eigen_values.y = b.y = mat_.y.y;
		eigen_values.z = b.z = mat_.z.z;
		eigen_values.w = b.w = S(0);
		eigen_vectors = Mat3x4<S,A,B>::Identity();

		Mat3x4<S,A,B> mat = mat_;
		S sum;
		S const diagonal_eps = S(1.0e-4);
		do
		{
			auto z = Vec4<S,void>{};

			// Sweep through all elements above the diagonal
			for (int i = 0; i != 3; ++i) //ip
			{
				for (int j = i + 1; j != 3; ++j) //iq
				{
					if (Abs(mat[j][i]) > diagonal_eps/S(3))
					{
						auto h     = eigen_values[j] - eigen_values[i];
						auto theta = S(0.5) * h / mat[j][i];
						auto t     = Sign(theta) / (Abs(theta) + Sqrt(S(1) + Sqr(theta)));
						auto c     = S(1) / Sqrt(S(1) + Sqr(t));
						auto s     = t * c;
						auto tau   = s / (S(1) + c);
						h          = t * mat[j][i];

						z[i] -= h;
						z[j] += h;
						eigen_values[i] -= h;
						eigen_values[j] += h;
						mat[j][i] = S(0);

						for (int k = 0; k != i; ++k)
							L::Rotate(mat, k, i, k, j, s, tau); //changes mat( 0:i-1 ,i) and mat( 0:i-1 ,j)

						for (int k = i + 1; k != j; ++k)
							L::Rotate(mat, i, k, k, j, s, tau); //changes mat(i, i+1:j-1 ) and mat( i+1:j-1 ,j)

						for (int k = j + 1; k != 3; ++k)
							L::Rotate(mat, i, k, j, k, s, tau); //changes mat(i, j+1:2 ) and mat(j, j+1:2 )

						for (int k = 0; k != 3; ++k)
							L::Rotate(eigen_vectors, k, i, k, j, s, tau); //changes EigenVec( 0:2 ,i) and evec( 0:2 ,j)
					}
				}
			}

			b = b + z;
			eigen_values = b;

			// Calculate sum of abs. values of off diagonal elements, to see if we've finished
			sum  = Abs(mat.y.x);
			sum += Abs(mat.z.x);
			sum += Abs(mat.z.y);
		}
		while (sum > diagonal_eps);
		return mat;
	}

	// Construct a rotation matrix that transforms 'from' onto the z axis
	// Other points can then be projected onto the XY plane by rotating by this
	// matrix and then setting the z value to zero
	template <Scalar S, typename A> inline Mat3x4<S,A,A> RotationToZAxis(Vec4<S,A> from)
	{
		auto r = Sqr(from.x) + Sqr(from.y);
		auto d = Sqrt(r);
		Mat3x4<S,A,A> mat = {};
		if (FEql(d, S(0)))
		{
			mat = Mat3x4<S,A,A>::Identity(); // Create an identity transform or a 180 degree rotation
			mat.x.x = from.z;                // about Y depending on the sign of 'from.z'
			mat.z.z = from.z;
		}
		else
		{
			mat.x = Vec4<S,void>{from.x*from.z/d, -from.y/d, from.x, S(0)};
			mat.y = Vec4<S,void>{from.y*from.z/d,  from.x/d, from.y, S(0)};
			mat.z = Vec4<S,void>{           -r/d,      S(0), from.z, S(0)};
		}
		return mat;
	}

	// Permute the vectors in a rotation matrix by 'n'.
	// n == 0 : x  y  z
	// n == 1 : z  x  y
	// n == 2 : y  z  x
	template <Scalar S, typename A, typename B> inline Mat3x4<S,A,B> pr_vectorcall PermuteRotation(Mat3x4<S,A,B> const& mat, int n)
	{
		switch (n % 3)
		{
			case 1: return Mat3x4<S, A, B>{mat.z, mat.x, mat.y};
			case 2: return Mat3x4<S, A, B>{mat.y, mat.z, mat.x};
			default:return mat;
		}
	}

	// Make an orientation matrix from a direction vector
	// 'dir' is the direction to align the axis 'axis_id' to. (Doesn't need to be normalised)
	// 'up' is the preferred up direction, however if up is parallel to 'dir'
	// then a vector perpendicular to 'dir' will be chosen.
	template <Scalar S, typename A> inline Mat3x4<S,A,A> pr_vectorcall OriFromDir(Vec4<S,A> dir, AxisId axis_id, Vec4<S,A> up_)
	{
		pr_assert("'dir' cannot be a zero vector" && (dir != Vec4<S,void>{}));

		// Get the preferred up direction (handling parallel cases)
		auto up = Perpendicular(dir, up_);

		Mat3x4<S,A,A> ori = {};
		ori.z = Normalise(Sign(S(axis_id)) * dir);
		ori.x = Normalise(Cross3(up, ori.z));
		ori.y = Cross3(ori.z, ori.x);

		// Permute the column vectors so +Z becomes 'axis'
		return PermuteRotation(ori, abs(axis_id));
	}
	template <Scalar S, typename A> inline Mat3x4<S,A,A> pr_vectorcall OriFromDir(Vec4<S,A> dir, AxisId axis_id)
	{
		return OriFromDir(dir, axis_id, Perpendicular(dir));
	}

	// Make a scaled orientation matrix from a direction vector
	// Returns a transform for scaling and rotating the 'axis'th axis to 'dir'
	template <Scalar S, typename A> inline Mat3x4<S,A,A> pr_vectorcall ScaledOriFromDir(Vec4<S,A> dir, AxisId axis, Vec4<S,A> up)
	{
		auto len = Length(dir);
		return len > maths::tiny<S> ? OriFromDir(dir, axis, up) * Mat3x4<S,A,A>::Scale(len) : Mat3x4<S,A,A>::Zero();
	}
	template <Scalar S, typename A> inline Mat3x4<S,A,A> pr_vectorcall ScaledOriFromDir(Vec4<S,A> dir, AxisId axis)
	{
		return ScaledOriFromDir(dir, axis, Perpendicular(dir));
	}

	// Return a vector representing the approximate rotation between two orthonormal transforms
	template <Scalar S, typename A, typename B> inline Vec4<S,void> pr_vectorcall RotationVectorApprox(Mat3x4<S,A,B> const& from, Mat3x4<S,A,B> const& to)
	{
		pr_assert("This only works for orthonormal matrices" && IsOrthonormal(from) && IsOrthonormal(to));
		
		auto cpm_x_i2wR = to - from;
		auto w2iR = Transpose(from);
		auto cpm = cpm_x_i2wR * w2iR;
		return Vec4<S,void>{cpm.y.z, cpm.z.x, cpm.x.y, S(0)};
	}

	// Spherically interpolate between two rotations
	template <Scalar S, typename A, typename B> inline Mat3x4<S,A,B> pr_vectorcall Slerp(Mat3x4<S,A,B> const& lhs, Mat3x4<S,A,B> const& rhs, S frac)
	{
		if (frac == S(0)) return lhs;
		if (frac == S(1)) return rhs;
		return Mat3x4<S,A,B>{Slerp(Quat<S,A,B>(lhs), Quat<S,A,B>(rhs), frac)};
	}

	// Create a cross product matrix for 'vec'.
	template <Scalar S, typename A> inline Mat3x4<S,A,A> pr_vectorcall CPM(Vec4<S,A> vec)
	{
		// This matrix can be used to calculate the cross product with
		// another vector: e.g. Cross3(v1, v2) == CPM(v1) * v2
		return Mat3x4<S,A,A>{
			Vec4<S,void>(  S(0),  vec.z, -vec.y, S(0)),
			Vec4<S,void>(-vec.z,   S(0),  vec.x, S(0)),
			Vec4<S,void>( vec.y, -vec.x,   S(0), S(0))};
	}

	// Return 'exp(omega)' (Rodriges' formula)
	template <Scalar S, typename A> inline Mat3x4<S, A, A> pr_vectorcall ExpMap3x3(Vec4<S, A> omega)
	{
		// Converts an angular velocity into a finite rotation that stays within SO(3).
		// If you have an angular velocity, w, that is constant over a time step,
		// then:
		//   R(t + dt) = R(t) * ExpMap(w * dt)
		//   (no need to orthonormalise)
		//
		// Rodrigues' formula:  exp(omega) = I + (sin(theta)/theta) * omega + ((1 - cos(theta)/theta²) * omega²
		// If you want the shortest rotation from R0 to R1:
		//   R(t) = R0 * ExpMap(t * LogMap(Transpose(R0) * ​R1​))
		return Mat3x4<S, A, A>::Rotation(omega);
	}

	// Returns the Axis*Angle vector representation of a rotation matrix (Inverse of ExpMap)
	template <Scalar S, typename A, typename B> inline Vec4<S, A> pr_vectorcall LogMap(Mat3x4<S,A,B> const& rot)
	{
		auto cos_angle = Clamp<S>((Trace(rot) - S(1)) / S(2), -S(1), +S(1));
		auto theta = Acos(cos_angle);
		if (Abs(theta) < maths::tiny<S>)
			return Vec4<S, A>::Zero();

		auto s = S(1) / (S(2) * Sin(theta));
		auto axis = s * Vec4<S, A>{rot.y.z - rot.z.y, rot.z.x - rot.x.z, rot.x.y - rot.y.x, S(0)};
		return theta * axis;
	}

	// Evaluates 'ori' after 'time' for a constant angular velocity and angular acceleration
	template <Scalar S, typename A, typename B> inline Mat3x4<S, A, B> pr_vectorcall RotationAt(float time, Mat3x4<S,A,B> const& ori, v4 avel, v4 aacc)
	{
		// Orientation can be computed analytically if angular velocity
		// and angular acceleration are parallel or angular acceleration is zero.
		if (LengthSq(Cross(avel, aacc)) < maths::tinyf)
		{
			auto w = avel + aacc * time;
			return ExpMap3x3(w * time) * ori;
		}
		else
		{
			// Otherwise, use the SPIRAL(6) algorithm. 6th order accurate for moderate 'time_s'

			// 3-point Gauss-Legendre nodes for 6th order accuracy
			constexpr float root15f = 3.87298334620741688518f;
			constexpr float c1 = 0.5f - root15f / 10.0f;
			constexpr float c2 = 0.5f;
			constexpr float c3 = 0.5f + root15f / 10.0f;

			// Evaluate instantaneous angular velocity at nodes
			auto w0 = avel + aacc * c1 * time;
			auto w1 = avel + aacc * c2 * time;
			auto w2 = avel + aacc * c3 * time;

			auto u0 = ExpMap3x3(w0 * time / 3.0f);
			auto u1 = ExpMap3x3(w1 * time / 3.0f);
			auto u2 = ExpMap3x3(w2 * time / 3.0f);

			return u2 * u1 * u0 * ori;
		}
	}
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
namespace pr::maths
{
	PRUnitTestClass(Matrix3x3Tests)
	{
		std::default_random_engine rng;
		TestClass_Matrix3x3Tests()
			:rng(1u)
		{
		}

		PRUnitTestMethod(MultiplyScalar, float, double, int32_t, int64_t)
		{
			using S = T;
			using vec4_t = Vec4<S, void>;
			using mat3_t = Mat3x4<S, void, void>;

			auto m1 = mat3_t{vec4_t{1,2,3,4}, vec4_t{1,1,1,1}, vec4_t{4,3,2,1}};
			auto m2 = S(2);
			auto R = mat3_t{vec4_t{2,4,6,8}, vec4_t{2,2,2,2}, vec4_t{8,6,4,2}};
			auto r = m1 * m2;
			PR_EXPECT(FEql(r, R));
		}
		PRUnitTestMethod(MultiplyVector4, float, double, int32_t, int64_t)
		{
			using S = T;
			using vec4_t = Vec4<S, void>;
			using mat3_t = Mat3x4<S, void, void>;

			auto m = mat3_t{vec4_t{1,2,3,4}, vec4_t{1,1,1,1}, vec4_t{4,3,2,1}};
			auto v = vec4_t{-3,4,2,-2};
			auto R = vec4_t{9,4,-1,-2};
			auto r = m * v;
			PR_EXPECT(FEql(r, R));
		}
		PRUnitTestMethod(MultiplyVector3, float, double, int32_t, int64_t)
		{
			using S = T;
			using vec3_t = Vec3<S, void>;
			using vec4_t = Vec4<S, void>;
			using mat3_t = Mat3x4<S, void, void>;

			auto m = mat3_t{vec4_t{1,2,3,4}, vec4_t{1,1,1,1}, vec4_t{4,3,2,1}};
			auto v = vec3_t{-3,4,2};
			auto R = vec3_t{9,4,-1};
			auto r = m * v;
			PR_EXPECT(FEql(r, R));
		}
		PRUnitTestMethod(MultiplyMatrix, float, double, int32_t, int64_t)
		{
			using S = T;
			using vec4_t = Vec4<S, void>;
			using mat3_t = Mat3x4<S, void, void>;

			auto m1 = mat3_t{vec4_t{1,2,3,4}, vec4_t{1,1,1,1}, vec4_t{4,3,2,1}};
			auto m2 = mat3_t{vec4_t{1,1,1,1}, vec4_t{2,2,2,2}, vec4_t{-2,-2,-2,-2}};
			auto R  = mat3_t{vec4_t{6,6,6,0}, vec4_t{12,12,12,0}, vec4_t{-12,-12,-12,0}};
			auto r = m1 * m2;
			PR_EXPECT(FEql(r, R));
		}
		PRUnitTestMethod(OriFromDir, float, double, int32_t, int64_t)
		{
			using S = T;
			using vec4_t = Vec4<S, void>;
			using mat3_t = Mat3x4<S, void, void>;

			if constexpr (std::floating_point<S>)
			{
				vec4_t dir(0, 1, 0, 0);
				{
					auto ori = OriFromDir(dir, AxisId::PosZ, vec4_t::ZAxis());
					PR_EXPECT(dir == +ori.z);
					PR_EXPECT(IsOrthonormal(ori));
				}
				{
					auto ori = OriFromDir(dir, AxisId::NegX);
					PR_EXPECT(dir == -ori.x);
					PR_EXPECT(IsOrthonormal(ori));
				}
				{
					auto scale = S(0.125);
					auto sdir = scale * dir;
					auto ori = ScaledOriFromDir(sdir, AxisId::PosY);
					PR_EXPECT(sdir == +ori.y);
					PR_EXPECT(IsOrthonormal((1 / scale) * ori));
				}
			}
		}
		PRUnitTestMethod(GeneralInvert, float, double)
		{
			using S = T;
			using vec4_t = Vec4<S, void>;
			using mat3_t = Mat3x4<S, void, void>;

			{
				auto m = mat3_t::Random(rng, S(-5), S(+5));
				auto inv_m = Invert(m);
				auto I0 = inv_m * m;
				auto I1 = m * inv_m;

				PR_EXPECT(FEql(I0, mat3_t::Identity()));
				PR_EXPECT(FEql(I1, mat3_t::Identity()));
			}
			{
				auto m = mat3_t(
					vec4_t(S(0.25), S(0.5), S(1), S(0)),
					vec4_t(S(0.49), S(0.7), S(1), S(0)),
					vec4_t(S(1.00), S(1.0), S(1), S(0)));
				auto INV_M = mat3_t(
					vec4_t(S(+10.0), S(-16.666667), S(+6.66667), S(0)),
					vec4_t(S(-17.0), S(+25.0), S(-8.0), S(0)),
					vec4_t(S(+7.0), S(-8.333333), S(+2.333333), S(0)));

				auto inv_m = Invert(m);
				PR_EXPECT(FEqlRelative(inv_m, INV_M, S(0.0001)));
			}
		}
		PRUnitTestMethod(AffineInvert, float, double)
		{
			using S = T;
			using vec4_t = Vec4<S, void>;
			using mat3_t = Mat3x4<S, void, void>;

			auto a2b = mat3_t::Rotation(vec4_t::Normal(-4, -3, +2, 0), -2.15f) * mat3_t::Scale(2.0f);
			PR_EXPECT(IsAffine(a2b));
			
			auto b2a = Invert(a2b);
			auto a2a = b2a * a2b;
			PR_EXPECT(FEql(mat3_t::Identity(), a2a));

			auto b2a_fast = InvertAffine(a2b);
			PR_EXPECT(FEql(b2a_fast, b2a));
		}
		PRUnitTestMethod(OrthonormalInvert, float, double)
		{
			using S = T;
			using vec4_t = Vec4<S, void>;
			using mat3_t = Mat3x4<S, void, void>;

			auto a2b = mat3_t::Rotation(vec4_t::Normal(-4, -3, +2, 0), -2.15f);
			PR_EXPECT(IsOrthonormal(a2b));
			
			auto b2a = Invert(a2b);
			auto a2a = b2a * a2b;
			PR_EXPECT(FEql(mat3_t::Identity(), a2a));

			auto b2a_fast = InvertOrthonormal(a2b);
			PR_EXPECT(FEql(b2a_fast, b2a));
		}
		PRUnitTestMethod(CPM, float, double)
		{
			using S = T;
			using vec4_t = Vec4<S, void>;
			using mat3_t = Mat3x4<S, void, void>;

			auto v = vec4_t(S(2), S(-1), S(4), S(0));
			auto m = CPM(v);

			auto a0 = vec4_t::Random(rng, vec4_t::Zero(), S(5), S(0));
			auto A0 = m * a0;
			auto A1 = Cross(v, a0);
			PR_EXPECT(FEql(A0, A1));
		}
		PRUnitTestMethod(LogMapExpMap, float, double)
		{
			using S = T;
			using vec4_t = Vec4<S, void>;
			using mat3_t = Mat3x4<S, void, void>;

			auto w = vec4_t(S(2), S(-1), S(4), S(0));

			// Wrap into [0, tau/2]
			auto w_len = Length(w);
			w *= fmod(w_len, maths::tau_by_2f) / w_len;

			auto rot1 = mat3_t::Rotation(w);
			auto rot2 = ExpMap3x3(w);
			auto W = LogMap(rot2);

			PR_EXPECT(FEql(rot1, rot2));
			PR_EXPECT(FEql(w, W));
		}
	};
}
#endif
