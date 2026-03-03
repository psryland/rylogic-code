//*****************************************************************************
// Maths library
//  Copyright (c) Rylogic Ltd 2002
//*****************************************************************************
#pragma once
#include "pr/math_new/core/forward.h"
#include "pr/math_new/core/traits.h"
#include "pr/math_new/core/constants.h"
#include "pr/math_new/types/vector4.h"
#include "pr/math_new/types/quaternion.h"

namespace pr::math
{
	template <ScalarType S>
	struct Mat3x4
	{
		#pragma warning(push)
		#pragma warning(disable:4201) // nameless struct
		union
		{
			struct { Vec4<S> x, y, z; };
			struct { Vec4<S> arr[3]; };
		};
		#pragma warning(pop)

		// Construct
		Mat3x4() = default;
		constexpr explicit Mat3x4(S x_) noexcept
			:x(x_)
			,y(x_)
			,z(x_)
		{}
		constexpr Mat3x4(Vec3<S> x_, Vec3<S> y_, Vec3<S> z_) noexcept
			:x(x_, S(0))
			,y(y_, S(0))
			,z(z_, S(0))
		{}
		constexpr Mat3x4(Vec4<S> x_, Vec4<S> y_, Vec4<S> z_) noexcept
			:x(x_)
			,y(y_)
			,z(z_)
		{}
		constexpr explicit Mat3x4(std::ranges::random_access_range auto&& v) noexcept // 12 scalars
			:Mat3x4(
				Vec4<S>(v[0], v[1], v[2], v[3]),
				Vec4<S>(v[4], v[5], v[6], v[7]),
				Vec4<S>(v[8], v[9], v[10], v[11]))
		{}

		// Explicit cast to different Scalar type
		template <ScalarType S2> constexpr explicit operator Mat3x4<S2>() const noexcept
		{
			return Mat3x4<S2>(
				static_cast<Vec4<S2>>(x),
				static_cast<Vec4<S2>>(y),
				static_cast<Vec4<S2>>(z)
			);
		}

		// Array access
		constexpr Vec4<S> const& operator [](int i) const noexcept
		{
			pr_assert(i >= 0 && i < 3 && "index out of range");
			if consteval { return i == 0 ? x : i == 1 ? y : z; }
			else { return arr[i]; }
		}
		constexpr Vec4<S>& operator [](int i) noexcept
		{
			pr_assert(i >= 0 && i < 3 && "index out of range");
			if consteval { return i == 0 ? x : i == 1 ? y : z; }
			else { return arr[i]; }
		}

		// Constants
		static constexpr Mat3x4 const& Zero() noexcept
		{
			static auto s_zero = math::Zero<Mat3x4>();
			return s_zero;
		}
		static constexpr Mat3x4 const& Identity() noexcept
		{
			static auto s_identity = math::Identity<Mat3x4>();
			return s_identity;
		}

		// Get/Set by row or column. Note: x,y,z are column vectors
		Vec4<S> col(int i) const noexcept
		{
			return arr[i];
		}
		Vec4<S> row(int i) const noexcept
		{
			return Vec4<S>{x[i], y[i], z[i], 0};
		}
		void col(int i, Vec4<S> col) noexcept
		{
			arr[i] = col;
		}
		void row(int i, Vec4<S> row) noexcept
		{
			x[i] = row.x;
			y[i] = row.y;
			z[i] = row.z;
		}

		// Create a 4x4 matrix from this 3x4 matrix
		constexpr Mat4x4<S> w1() const noexcept;
		constexpr Mat4x4<S> w1(Vec4<S> xyz) const noexcept;

		// Return the diagonal elements of this matrix
		constexpr Vec4<S> diagonal() const noexcept
		{
			return math::Diagonal(*this);
		}

		// Return the scale of this matrix
		constexpr Mat3x4 scale() const noexcept
		{
			return math::ScaleFrom<Mat3x4>(*this);
		}

		// Return this matrix with the scale removed
		constexpr Mat3x4 unscaled() const noexcept
		{
			return math::Unscaled<Mat3x4>(*this);
		}

		// Construct a rotation matrix from Euler angles. Order is: roll, pitch, yaw (to match DirectX)
		static Mat3x4 RotationRad(S pitch, S yaw, S roll) noexcept
		{
			return math::RotationRad<Mat3x4>(pitch, yaw, roll);
		}
		static Mat3x4 RotationDeg(S pitch, S yaw, S roll) noexcept
		{
			return math::RotationDeg<Mat3x4>(pitch, yaw, roll);
		}

		// Create from an axis, angle
		static Mat3x4 pr_vectorcall Rotation(Vec4<S> axis_norm, Vec4<S> axis_sine_angle, S cos_angle) noexcept
		{
			return math::Rotation<Mat3x4>(axis_norm, axis_sine_angle, cos_angle);
		}

		// Create from an axis and angle. 'axis' should be normalised
		static Mat3x4 pr_vectorcall Rotation(Vec4<S> axis_norm, S angle) noexcept
		{
			return math::Rotation<Mat3x4>(axis_norm, angle);
		}

		// Create from an angular displacement vector. length = angle(rad), direction = axis
		static Mat3x4 pr_vectorcall Rotation(Vec4<S> angular_displacement) noexcept // This is ExpMap3x3.
		{
			return math::Rotation<Mat3x4>(angular_displacement);
		}

		// Create a transform representing the rotation from one vector to another. (Vectors do not need to be normalised)
		static Mat3x4 pr_vectorcall Rotation(Vec4<S> from, Vec4<S> to) noexcept
		{
			return math::Rotation<Mat3x4>(from, to);
		}

		// Create a transform from one basis axis to another. Remember AxisId can be cast to Vec4
		static Mat3x4 Rotation(AxisId from_axis, AxisId to_axis) noexcept
		{
			return math::Rotation<Mat3x4>(from_axis, to_axis);
		}

		// Create a scale matrix
		static Mat3x4 Scale(S scale) noexcept
		{
			return math::Scale<Mat3x4>(scale);
		}
		static Mat3x4 Scale(S sx, S sy, S sz) noexcept
		{
			return math::Scale<Mat3x4>(Vec4<S>(sx, sy, sz, S(0)));
		}
		static Mat3x4 Scale(Vec3<S> scale) noexcept
		{
			return math::Scale<Mat3x4>(Vec4<S>(scale.x, scale.y, scale.z, S(0)));
		}

		// Create a shear matrix
		static Mat3x4 Shear(S sxy, S sxz, S syx, S syz, S szx, S szy) noexcept
		{
			return math::Shear<Mat3x4>(sxy, sxz, syx, syz, szx, szy);
		}

	};

	#define PR_MATH_DEFINE_TYPE(component, element)\
	template <> struct vector_traits<Mat3x4<element>>\
		: vector_traits_base<element, component, 3>\
		, vector_access_member<Mat3x4<element>, component, 3>\
	{};\
	\
	static_assert(VectorType<Mat3x4<element>>, "Mat3x4<"#element"> is not a valid vector type");\
	static_assert(IsRank2<Mat3x4<element>>, "Mat3x4<"#element"> is not rank 2");\
	static_assert(sizeof(Mat3x4<element>) == 3*4*sizeof(element), "Mat3x4<"#element"> has the wrong size");\
	static_assert(std::is_trivially_copyable_v<Mat3x4<element>>, "Mat3x4<"#element"> is not trivially copyable");

	PR_MATH_DEFINE_TYPE(Vec4<float>, float);
	PR_MATH_DEFINE_TYPE(Vec4<double>, double);
	PR_MATH_DEFINE_TYPE(Vec4<int32_t>, int32_t);
	PR_MATH_DEFINE_TYPE(Vec4<int64_t>, int64_t);
	#undef PR_MATH_DEFINE_TYPE
}



#if 0
	struct Mat3x4
	{
		#pragma region Operators
		friend constexpr Mat3x4 pr_vectorcall operator + (Mat3x4<S,A,B> const& mat) noexcept
		{
			return mat;
		}
		friend constexpr Mat3x4 pr_vectorcall operator - (Mat3x4<S,A,B> const& mat) noexcept
		{
			return Mat3x4{-mat.x, -mat.y, -mat.z};
		}
		friend Mat3x4 pr_vectorcall operator * (S lhs, Mat3x4<S,A,B> const& rhs) noexcept
		{
			return rhs * lhs;
		}
		friend Mat3x4 pr_vectorcall operator * (Mat3x4<S,A,B> const& lhs, S rhs) noexcept
		{
			return Mat3x4{lhs.x * rhs, lhs.y * rhs, lhs.z * rhs};
		}
		friend Mat3x4 pr_vectorcall operator / (Mat3x4<S,A,B> const& lhs, S rhs) noexcept
		{
			// Don't check for divide by zero by default. For floats +inf/-inf are valid results
			//pr_assert("divide by zero" && rhs != 0);
			return Mat3x4{lhs.x / rhs, lhs.y / rhs, lhs.z / rhs};
		}
		friend Mat3x4 pr_vectorcall operator % (Mat3x4<S,A,B> const& lhs, S rhs) noexcept
		{
			// Don't check for divide by zero by default. For floats +inf/-inf are valid results
			//pr_assert("divide by zero" && rhs != 0);
			return Mat3x4{lhs.x % rhs, lhs.y % rhs, lhs.z % rhs};
		}
		friend Mat3x4 pr_vectorcall operator + (Mat3x4<S,A,B> const& lhs, Mat3x4<S,A,B> const& rhs) noexcept
		{
			return Mat3x4{lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z};
		}
		friend Mat3x4 pr_vectorcall operator - (Mat3x4<S,A,B> const& lhs, Mat3x4<S,A,B> const& rhs) noexcept
		{
			return Mat3x4{lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z};
		}
		friend Vec4<S, B> pr_vectorcall operator * (Mat3x4<S,A,B> const& lhs, Vec4<S, A> rhs) noexcept
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
		friend Vec3<S,B> pr_vectorcall operator * (Mat3x4<S,A,B> const& lhs, Vec3<S,A> rhs) noexcept
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
		template <typename C> friend Mat3x4<S,A,C> pr_vectorcall operator * (Mat3x4<S,B,C> const& lhs, Mat3x4<S,A,B> const& rhs) noexcept
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
#endif
