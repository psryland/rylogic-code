//*****************************************************************************
// Maths library
//  Copyright (c) Rylogic Ltd 2002
//*****************************************************************************
#pragma once
#include "pr/maths/forward.h"
#include "pr/maths/constants.h"
#include "pr/maths/vector2.h"

namespace pr
{
	template <Scalar S, typename A, typename B>
	struct Mat2x2
	{
		#pragma warning(push)
		#pragma warning(disable:4201) // nameless struct
		union
		{
			struct { Vec2<S, void> x, y; };
			struct { Vec2<S, void> arr[2]; };
		};
		#pragma warning(pop)

		// Construct
		Mat2x2() = default;
		constexpr explicit Mat2x2(S x_)
			:x(x_)
			,y(x_)
		{}
		constexpr Mat2x2(S xx, S xy, S yx, S yy)
			:x(xx, xy)
			,y(yx, yy)
		{}
		constexpr Mat2x2(Vec2<S,void> x_, Vec2<S,void> y_)
			:x(x_)
			,y(y_)
		{}
		constexpr explicit Mat2x2(S const* v)
			:Mat2x2(Vec2<S,void>(&v[0]), Vec2<S,void>(&v[2]))
		{}

		// Reinterpret as a different vector type
		template <typename C, typename D> explicit operator Mat2x2<S,C,D> const& () const
		{
			return reinterpret_cast<Mat2x2<S,C,D> const&>(*this);
		}
		template <typename C, typename D> explicit operator Mat2x2<S,C,D>& ()
		{
			return reinterpret_cast<Mat2x2<S,C,D>&>(*this);
		}
		operator Mat2x2<S,void,void> const& () const
		{
			return reinterpret_cast<Mat2x2<S,void,void> const&>(*this);
		}
		operator Mat2x2<S,void,void>& ()
		{
			return reinterpret_cast<Mat2x2<S,void,void>&>(*this);
		}

		// Array access
		Vec2<S,void> const& operator [](int i) const
		{
			pr_assert("index out of range" && i >= 0 && i < _countof(arr));
			return arr[i];
		}
		Vec2<S,void>& operator [](int i)
		{
			pr_assert("index out of range" && i >= 0 && i < _countof(arr));
			return arr[i];
		}

		// Basic constants
		static constexpr Mat2x2 Zero()
		{
			return Mat2x2{Vec2<S,void>::Zero(), Vec2<S,void>::Zero()};
		}
		static constexpr Mat2x2 Identity()
		{
			return Mat2x2{Vec2<S,void>::XAxis(), Vec2<S,void>::YAxis()};
		}

		// Create a rotation matrix
		static Mat2x2 Rotation(S angle)
		{
			return Mat2x2{
				Vec2<S,void>(+Cos(angle), +Sin(angle)),
				Vec2<S,void>(-Sin(angle), +Cos(angle)),
			};
		}

		// Create a scale matrix
		static Mat2x2 Scale(S scale)
		{
			Mat2x2 mat = {};
			mat.x.x = mat.y.y = scale;
			return mat;
		}
		static Mat2x2 Scale(S sx, S sy, S sz)
		{
			Mat2x2 mat = {};
			mat.x.x = sx;
			mat.y.y = sy;
			mat.z.z = sz;
			return mat;
		}

		// Create a 2D matrix containing random rotation between angles [min_angle, max_angle)
		template <typename Rng = std::default_random_engine> static Mat2x2 Random(Rng& rng, S min_angle, S max_angle)
		{
			std::uniform_real_distribution<S> dist(min_angle, max_angle);
			return Rotation(dist(rng));
		}

		// Create a random 2D rotation matrix
		template <typename Rng = std::default_random_engine> static Mat2x2 Random(Rng& rng)
		{
			return Random(rng, S(0), constants<S>::tau);
		}

		#pragma region Operators
		friend constexpr Mat2x2 operator + (Mat2x2<S,A,B> const& mat)
		{
			return mat;
		}
		friend constexpr Mat2x2 operator - (Mat2x2<S,A,B> const& mat)
		{
			return Mat2x2{-mat.x, -mat.y};
		}
		friend Mat2x2 operator * (S lhs, Mat2x2<S,A,B> const& rhs)
		{
			return rhs * lhs;
		}
		friend Mat2x2 operator * (Mat2x2<S,A,B> const& lhs, S rhs)
		{
			return Mat2x2{lhs.x * rhs, lhs.y * rhs};
		}
		friend Mat2x2 operator / (Mat2x2<S,A,B> const& lhs, S rhs)
		{
			// Don't check for divide by zero by default. For floats +inf/-inf are valid results
			//pr_assert("divide by zero" && rhs != 0);
			return Mat2x2{lhs.x / rhs, lhs.y / rhs};
		}
		friend Mat2x2 operator % (Mat2x2<S,A,B> const& lhs, S rhs)
		{
			// Don't check for divide by zero by default. For floats +inf/-inf are valid results
			//pr_assert("divide by zero" && rhs != 0);
			return Mat2x2{lhs.x % rhs, lhs.y % rhs};
		}
		friend Mat2x2 operator + (Mat2x2<S,A,B> const& lhs, Mat2x2<S,A,B> const& rhs)
		{
			return Mat2x2{lhs.x + rhs.x, lhs.y + rhs.y};
		}
		friend Mat2x2 operator - (Mat2x2<S,A,B> const& lhs, Mat2x2<S,A,B> const& rhs)
		{
			return Mat2x2{lhs.x - rhs.x, lhs.y - rhs.y};
		}
		friend Vec2<S,B> operator * (Mat2x2<S,A,B> const& lhs, Vec2<S,A> rhs)
		{
			auto ans = Vec2<S,B>{};
			auto lhsT = Transpose(lhs);
			ans.x = Dot(lhsT.x, rhs);
			ans.y = Dot(lhsT.y, rhs);
			return ans;
		}
		template <typename C> friend Mat2x2<S,A,C> operator * (Mat2x2<S,B,C> const& lhs, Mat2x2<S,A,B> const& rhs)
		{
			auto ans = Mat2x2<S,A,C>{};
			auto lhsT = Transpose(lhs);
			ans.x.x = Dot(lhsT.x, rhs.x);
			ans.x.y = Dot(lhsT.y, rhs.x);
			ans.y.x = Dot(lhsT.x, rhs.y);
			ans.y.y = Dot(lhsT.y, rhs.y);
			return ans;
		}
		#pragma endregion
	};
	#define PR_MAT2X2_CHECKS(scalar)\
	static_assert(sizeof(Mat2x2<scalar,void,void>) == 2*2*sizeof(scalar), "Mat2x2<"#scalar"> has the wrong size");\
	static_assert(maths::Matrix2<Mat2x2<scalar,void,void>>, "Mat2x2<"#scalar"> is not a Mat2x2");\
	static_assert(std::is_trivially_copyable_v<Mat2x2<scalar,void,void>>, "Mat2x2<"#scalar"> must be a pod type");\
	static_assert(std::alignment_of_v<Mat2x2<scalar,void,void>> == std::alignment_of_v<Vec2<scalar,void>>, "Mat2x2<"#scalar"> is not aligned correctly");
	PR_MAT2X2_CHECKS(float);
	PR_MAT2X2_CHECKS(double);
	PR_MAT2X2_CHECKS(int32_t);
	PR_MAT2X2_CHECKS(int64_t);
	#undef PR_MAT2X2_CHECKS

	// 2x2 matrix determinant
	template <Scalar S, typename A, typename B> inline S Determinant(Mat2x2<S,A,B> const& m)
	{
		return m.x.x*m.y.y - m.x.y*m.y.x;
	}

	// 2x2 matrix transpose
	template <Scalar S, typename A, typename B> inline Mat2x2<S,A,B> Transpose(Mat2x2<S,A,B> const& mat)
	{
		auto m = mat;
		std::swap(m.x.y, m.y.x);
		return m;
	}

	// Return true if 'mat' is an affine transform
	template <Scalar S, typename A, typename B> inline bool pr_vectorcall IsAffine(Mat2x2<S,A,B> const&)
	{
		// All 2x2 matrices are Affine
		return true;
	}

	// Return true if 'mat' is orthogonal
	template <Scalar S, typename A, typename B> inline bool pr_vectorcall IsOrthogonal(Mat2x2<S,A,B> const& mat)
	{
		return FEql(Dot(mat.x, mat.y), S(0));
	}

	// Return true if 'mat' is orthonormal
	template <Scalar S, typename A, typename B> inline bool pr_vectorcall IsOrthonormal(Mat2x2<S,A,B> const& mat)
	{
		return
			FEql(LengthSq(mat.x), S(1)) &&
			FEql(LengthSq(mat.y), S(1)) &&
			FEql(Abs(Determinant(mat)), S(1));
	}

	// Returns true if 'mat' has an inverse
	template <Scalar S, typename A, typename B> inline bool IsInvertible(Mat2x2<S,A,B> const& mat)
	{
		return Determinant(mat) != 0;
	}

	// Returns the inverse of 'mat'
	template <Scalar S, typename A, typename B> inline Mat2x2<S,B,A> Invert(Mat2x2<S,A,B> const& mat)
	{
		pr_assert("Matrix is singular" && Determinant(mat) != S(0));

		auto det = Determinant(mat);
		return Mat2x2<S, B, A>{
			{ +mat.y.y / det, -mat.x.y / det },
			{ -mat.y.x / det, +mat.x.x / det },
		};
	}

	// Returns the inverse of 'mat' assuming is it a pure rotation matrix
	template <Scalar S, typename A, typename B> inline Mat2x2<S, B, A> InvertAffine(Mat2x2<S,A,B> const& mat)
	{
		return Invert(mat); // Just as cheap
	}
	 
	// Returns the inverse of 'mat' assuming is it a pure rotation matrix 
	template <Scalar S, typename A, typename B> inline Mat2x2<S,B,A> InvertOrthonormal(Mat2x2<S,A,B> const& mat) 
	{ 
		assert("Matrix is not pure rotation" && FEql(Determinant(mat), S(1))); 
		return Mat2x2<S,B,A>{
			{ +mat.y.y, -mat.x.y },
			{ -mat.y.x, +mat.x.x },
		}; 
	} 

	// Return the square root of a matrix. The square root is the matrix B where B.B = mat.
	template <Scalar S, typename A, typename B> inline Mat2x2<S,A,B> Sqrt(Mat2x2<S,A,B> const& mat)
	{
		// Using 'Denman-Beavers' square root iteration. Should converge quadratically
		auto a = mat;                       // Converges to mat^0.5
		auto b = Mat2x2<S,A,B>::Identity(); // Converges to mat^-0.5
		for (int i = 0; i != 10; ++i)
		{
			auto a_next = S(0.5) * (a + Invert(b));
			auto b_next = S(0.5) * (b + Invert(a));
			a = a_next;
			b = b_next;
		}
		return a;
	}
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
namespace pr::maths
{
	PRUnitTestClass(Matrix2x2Tests)
	{
		std::default_random_engine rng;
		TestClass_Matrix2x2Tests()
			:rng(1u)
		{
		}

		PRUnitTestMethod(Create, float, double, int32_t, int64_t)
		{
			using S = T;
			using vec2_t = Vec2<S, void>;
			using mat2_t = Mat2x2<S, void, void>;

			auto V0 = mat2_t(1, 2, 3, 4);
			PR_EXPECT(V0.x == vec2_t(1, 2));
			PR_EXPECT(V0.y == vec2_t(3, 4));

			auto V1 = mat2_t(vec2_t(1, 2), vec2_t(3, 4));
			PR_EXPECT(V1.x == vec2_t(1, 2));
			PR_EXPECT(V1.y == vec2_t(3, 4));

			auto V2 = mat2_t({ 1,2,3,4 });
			PR_EXPECT(V2.x == vec2_t(1, 2));
			PR_EXPECT(V2.y == vec2_t(3, 4));

			auto V3 = mat2_t{ 4,5,6,7 };
			PR_EXPECT(V3.x == vec2_t(4, 5));
			PR_EXPECT(V3.y == vec2_t(6, 7));
		}
		PRUnitTestMethod(Operators, float, double, int32_t, int64_t)
		{
			using S = T;
			using vec2_t = Vec2<S, void>;
			using mat2_t = Mat2x2<S, void, void>;

			auto V0 = mat2_t(10, 20, 30, 40);
			auto V1 = mat2_t(20, 30, 40, 50);

			PR_EXPECT(V0 + V1 == mat2_t(30, 50, 70, 90));
			PR_EXPECT(V0 - V1 == mat2_t(-10, -10, -10, -10));

			// 1 3     2 4     2+9  4+15     11 19
			// 2 4  x  3 5  =  4+12 8+20  =  16 28
			PR_EXPECT(V0 * V1 == mat2_t(1100, 1600, 1900, 2800));

			PR_EXPECT(S(3) * V0 == mat2_t(30, 60, 90, 120));
			PR_EXPECT(V0 * S(3) == mat2_t(30, 60, 90, 120));
			PR_EXPECT(V0 / S(2) == mat2_t(5, 10, 15, 20));
			PR_EXPECT(V0 % S(20) == mat2_t(10, 0, 10, 0));

			PR_EXPECT(+V0 == mat2_t(+10, +20, +30, +40));
			PR_EXPECT(-V0 == mat2_t(-10, -20, -30, -40));

			PR_EXPECT(V0 == mat2_t(10, 20, 30, 40));
			PR_EXPECT(V0 != mat2_t(40, 30, 20, 10));
		}
		PRUnitTestMethod(MinMaxClamp, float, double, int32_t, int64_t)
		{
			using S = T;
			using vec2_t = Vec2<S, void>;
			using mat2_t = Mat2x2<S, void, void>;

			auto V0 = mat2_t(1, 2, 3, 4);
			auto V1 = mat2_t(-1, -2, -3, -4);
			auto V2 = mat2_t(2, 4, 6, 8);

			PR_EXPECT(Min(V0, V1, V2) == mat2_t(-1, -2, -3, -4));
			PR_EXPECT(Max(V0, V1, V2) == mat2_t(2, 4, 6, 8));
			PR_EXPECT(Clamp(V0, V1, V2) == mat2_t(1, 2, 3, 4));
			PR_EXPECT(Clamp(V0, S(0), S(1)) == mat2_t(1, 1, 1, 1));
		}
		PRUnitTestMethod(Determinant, float, double, int32_t, int64_t)
		{
			using S = T;
			using mat2_t = Mat2x2<S, void, void>;

			auto a2b = mat2_t{
				{2, 1},
				{3, 4},
			};
			PR_EXPECT(Determinant(a2b) == 2 * 4 - 3 * 1);

		}
		PRUnitTestMethod(GenerateInvert, float, double)
		{
			using S = T;
			using vec2_t = Vec2<S, void>;
			using mat2_t = Mat2x2<S, void, void>;

			auto a2b = mat2_t{
				{2, 1},
				{3, 4},
			};
			PR_EXPECT(IsInvertible(a2b));

			auto b2a = Invert(a2b);
			auto a2a = b2a * a2b;
			PR_EXPECT(FEql(mat2_t::Identity(), a2a));
		}
		PRUnitTestMethod(AffineInvert, float, double)
		{
			using S = T;
			using vec2_t = Vec2<S, void>;
			using mat2_t = Mat2x2<S, void, void>;

			auto a2b = mat2_t{
				{2, 1},
				{3, 4},
			};
			PR_EXPECT(IsAffine(a2b));

			auto b2a = Invert(a2b);
			auto a2a = b2a * a2b;
			PR_EXPECT(FEql(mat2_t::Identity(), a2a));

			auto b2a_fast = InvertAffine(a2b);
			PR_EXPECT(FEql(b2a_fast, b2a));
		}
		PRUnitTestMethod(OrthonormalInvert, float, double)
		{
			using S = T;
			using vec2_t = Vec2<S, void>;
			using mat2_t = Mat2x2<S, void, void>;

			auto a2b = mat2_t::Random(rng, -S(5), +S(5));
			PR_EXPECT(IsOrthogonal(a2b));

			auto b2a = Invert(a2b);
			auto a2a = b2a * a2b;
			PR_EXPECT(FEql(mat2_t::Identity(), a2a));

			auto b2a_fast = InvertOrthonormal(a2b);
			PR_EXPECT(FEql(b2a_fast, b2a));
		}
	};
}
#endif
