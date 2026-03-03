//*****************************************************************************
// Maths library
//  Copyright (c) Rylogic Ltd 2002
//*****************************************************************************
#pragma once
#include "pr/maths/forward.h"
#include "pr/maths/constants.h"
#include "pr/maths/maths_core.h"

namespace pr
{
	template <Scalar S, typename T>
	struct Vec2
	{
		// Notes:
		//  - Can't use __m64 because it has an alignment of 8.
		//    v2 is a member of the union in v3 which needs alignment of 4, or the size of v3 becomes 16.
		#pragma warning(push)
		#pragma warning(disable:4201) // nameless struct
		union
		{
			struct { S x, y; };
			struct { S arr[2]; };
		};
		#pragma warning(pop)

		// Construct
		Vec2() = default;
		constexpr explicit Vec2(S x_)
			:x(x_)
			,y(x_)
		{}
		constexpr Vec2(S x_, S y_)
			:x(x_)
			,y(y_)
		{}
		constexpr explicit Vec2(S const* v)
			:Vec2(v[0], v[1])
		{}
		template <maths::Vector2 V> constexpr explicit Vec2(V const& v)
			:Vec2(maths::comp<0>(v), maths::comp<1>(v))
		{}

		// Reinterpret as a different vector type
		template <typename U> explicit operator Vec2<S, U> const&() const
		{
			return reinterpret_cast<Vec2<S, U> const&>(*this);
		}
		template <typename U> explicit operator Vec2<S, U>&()
		{
			return reinterpret_cast<Vec2<S, U>&>(*this);
		}
		operator Vec2<S, void> const& () const
		{
			return reinterpret_cast<Vec2<S, void> const&>(*this);
		}
		operator Vec2<S, void>& ()
		{
			return reinterpret_cast<Vec2<S, void>&>(*this);
		}

		// Array access
		S const& operator [] (int i) const
		{
			pr_assert("index out of range" && i >= 0 && i < _countof(arr));
			return arr[i];
		}
		S& operator [] (int i)
		{
			pr_assert("index out of range" && i >= 0 && i < _countof(arr));
			return arr[i];
		}

		// Basic constants
		static constexpr Vec2 Zero()     { return Vec2(S(0),S(0)); }
		static constexpr Vec2 XAxis()    { return Vec2(S(1),S(0)); }
		static constexpr Vec2 YAxis()    { return Vec2(S(0),S(1)); }
		static constexpr Vec2 One()      { return Vec2(S(1), S(1)); }
		static constexpr Vec2 Tiny()     { return Vec2(maths::tiny<S>, maths::tiny<S>); }
		static constexpr Vec2 Min()      { return Vec2(limits<S>::min(), limits<S>::min()); }
		static constexpr Vec2 Max()      { return Vec2(limits<S>::max(), limits<S>::max()); }
		static constexpr Vec2 Lowest()   { return Vec2(limits<S>::lowest(), limits<S>::lowest()); }
		static constexpr Vec2 Epsilon()  { return Vec2(limits<S>::epsilon(), limits<S>::epsilon()); }
		static constexpr Vec2 Infinity() { return Vec2(limits<S>::infinity(), limits<S>::infinity()); }

		// Construct normalised
		static Vec2 Normal(S x, S y) requires std::is_floating_point_v<S>
		{
			return Normalise(Vec2(x, y));
		}

		// Create a random vector with unit length
		template <typename Rng = std::default_random_engine> requires std::is_floating_point_v<S>
		static Vec2 pr_vectorcall RandomN(Rng& rng)
		{
			std::uniform_real_distribution<S> dist(S(-1), S(1));
			for (;;)
			{
				auto x = dist(rng);
				auto y = dist(rng);
				auto v = Vec2(x, y);
				auto len = LengthSq(v);
				if (len > S(0.01) && len <= S(1))
					return v / Sqrt(len);
			}
		}
		
		// Create a random vector with components on interval ['vmin', 'vmax']
		template <typename Rng = std::default_random_engine> requires std::is_floating_point_v<S>
		static Vec2 pr_vectorcall Random(Rng& rng, Vec2 vmin, Vec2 vmax)
		{
			std::uniform_real_distribution<S> dist_x(vmin.x, vmax.x);
			std::uniform_real_distribution<S> dist_y(vmin.y, vmax.y);
			return Vec2(dist_x(rng), dist_y(rng));
		}

		// Create a random vector with length on interval [min_length, max_length]
		template <typename Rng = std::default_random_engine> requires std::is_floating_point_v<S>
		static Vec2 pr_vectorcall Random(Rng& rng, S min_length, S max_length)
		{
			std::uniform_real_distribution<S> dist(min_length, max_length);
			return dist(rng) * RandomN(rng);
		}

		// Create a random vector centred on 'centre' with radius 'radius'
		template <typename Rng = std::default_random_engine> requires std::is_floating_point_v<S>
		static Vec2 pr_vectorcall Random(Rng& rng, Vec2 centre, S radius)
		{
			return Random(rng, S(0), radius) + centre;
		}

		#pragma region Operators
		friend constexpr Vec2 operator + (Vec2 vec)
		{
			return vec;
		}
		friend constexpr Vec2 operator - (Vec2 vec)
		{
			return Vec2(-vec.x, -vec.y);
		}
		friend Vec2 operator * (S lhs, Vec2 rhs)
		{
			return rhs * lhs;
		}
		friend Vec2 operator * (Vec2 lhs, S rhs)
		{
			return Vec2(lhs.x * rhs, lhs.y * rhs);
		}
		friend Vec2 operator / (Vec2 lhs, S rhs)
		{
			// Don't check for divide by zero by default. For floats +inf/-inf are valid results
			return Vec2(lhs.x / rhs, lhs.y / rhs);
		}
		friend Vec2 operator % (Vec2 lhs, S rhs)
		{
			// Don't check for divide by zero by default. For floats +inf/-inf are valid results
			return Vec2{ Modulus(lhs.x, rhs), Modulus(lhs.y, rhs) };
		}
		friend Vec2 operator + (Vec2 lhs, Vec2 rhs)
		{
			return Vec2(lhs.x + rhs.x, lhs.y + rhs.y);
		}
		friend Vec2 operator - (Vec2 lhs, Vec2 rhs)
		{
			return Vec2(lhs.x - rhs.x, lhs.y - rhs.y);
		}
		friend Vec2 operator * (Vec2 lhs, Vec2 rhs)
		{
			return Vec2(lhs.x * rhs.x, lhs.y * rhs.y);
		}
		friend Vec2 operator / (Vec2 lhs, Vec2 rhs)
		{
			// Don't check for divide by zero by default. For floats +inf/-inf are valid results
			return Vec2(lhs.x / rhs.x, lhs.y / rhs.y);
		}
		friend Vec2 operator % (Vec2 lhs, Vec2 rhs)
		{
			// Don't check for divide by zero by default. For floats +inf/-inf are valid results
			return Vec2{ Modulus(lhs.x, rhs.x), Modulus(lhs.y, rhs.y) };
		}
		#pragma endregion
	};
	#define PR_VEC2_CHECKS(scalar)\
	static_assert(sizeof(Vec2<scalar, void>) == 2*sizeof(scalar), "Vector<"#scalar"> has the wrong size");\
	static_assert(maths::Vector2<Vec2<scalar, void>>, "Vector<"#scalar"> is not a Vector2");\
	static_assert(std::is_trivially_copyable_v<Vec2<scalar, void>>, "Must be a pod type");
	PR_VEC2_CHECKS(float);
	PR_VEC2_CHECKS(double);
	PR_VEC2_CHECKS(int32_t);
	PR_VEC2_CHECKS(int64_t);
	#undef PR_VEC2_CHECKS

	// Dot product: a.b
	template <Scalar S, typename T> constexpr S pr_vectorcall Dot(Vec2<S, T> lhs, Vec2<S, T> rhs)
	{
		return lhs.x * rhs.x + lhs.y * rhs.y;
	}

	// Cross product: Dot2(Rotate90CW(lhs), rhs)
	template <Scalar S, typename T> constexpr S pr_vectorcall Cross(Vec2<S, T> lhs, Vec2<S, T> rhs)
	{
		return lhs.y * rhs.x - lhs.x * rhs.y;
	}

	// Rotate a 2d vector by 90deg (when looking down the Z axis)
	template <Scalar S, typename T> constexpr Vec2<S, T> pr_vectorcall Rotate90CW(Vec2<S, T> v)
	{
		return Vec2<T>(-v.y, v.x);
	}

	// Rotate a 2d vector by -90def (when looking down the Z axis)
	template <Scalar S, typename T> constexpr Vec2<S, T> pr_vectorcall Rotate90CCW(Vec2<S, T> v)
	{
		return Vec2<S, T>(v.y, -v.x);
	}

	// Returns a vector with the 'xy' values permuted 'n' times. '0=xy, 1=yz'
	template <Scalar S, typename T> constexpr Vec2<S, T> pr_vectorcall Permute(Vec2<S, T> v, int n)
	{
		return (n%2) == 1 ? Vec2<S, T>(v.y, v.x) : v;
	}

	// Returns a 2-bit bitmask of the quadrant the vector is in. 0=(-x,-y), 1=(+x,-y), 2=(-x,+y), 3=(+x,+y)
	template <Scalar S, typename T> constexpr uint32_t pr_vectorcall Quadrant(Vec2<S, T> v)
	{
		return
			((v.x >= S(0)) << 0) |
			((v.y >= S(0)) << 1);
	}

	// Divide a circle into N sectors and return an index for the sector that 'vec' is in
	template <Scalar S, typename T> inline int pr_vectorcall Sector(Vec2<S, T> vec, int sectors)
	{
		return static_cast<int>(ATan2Positive<double>(vec.y, vec.x) * sectors / maths::tau);
	}
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
namespace pr::maths
{
	PRUnitTestClass(Vector2Tests)
	{
		PRUnitTestMethod(Create, float, double, int32_t, int64_t)
		{
			using S = T;
			using vec2_t = Vec2<S, void>;

			auto V0 = vec2_t(S(1));
			PR_EXPECT(V0.x == S(1));
			PR_EXPECT(V0.y == S(1));

			auto V1 = vec2_t(S(1), S(2));
			PR_EXPECT(V1.x == S(1));
			PR_EXPECT(V1.y == S(2));

			auto V2 = vec2_t({ S(3), S(4) });
			PR_EXPECT(V2.x == S(3));
			PR_EXPECT(V2.y == S(4));

			vec2_t V3 = { S(4), S(5) };
			PR_EXPECT(V3.x == S(4));
			PR_EXPECT(V3.y == S(5));
		}
		PRUnitTestMethod(Normal, float, double)
		{
			using S = T;
			using vec2_t = Vec2<S, void>;

			auto V4 = vec2_t::Normal(S(3), S(4));
			auto V4_expected = vec2_t(S(0.6), S(0.8));
			PR_EXPECT(FEql(V4, V4_expected));
			PR_EXPECT(FEql(V4[0], S(0.6)));
			PR_EXPECT(FEql(V4[1], S(0.8)));
		}
		PRUnitTestMethod(Operators, float, double, int32_t, int64_t)
		{
			using S = T;
			using vec2_t = Vec2<S, void>;

			auto V0 = vec2_t(S(10), S(8));
			auto V1 = vec2_t(S(2), S(12));
			auto eql = [](auto lhs, auto rhs)
			{
				if constexpr (std::floating_point<S>)
					return FEql(lhs, rhs);
				else
					return lhs == rhs;
			};
			PR_EXPECT(eql(V0 + V1, vec2_t(S(+12), S(+20))));
			PR_EXPECT(eql(V0 - V1, vec2_t(S(+8), S(-4))));
			PR_EXPECT(eql(V0 * V1, vec2_t(S(+20), S(+96))));
			PR_EXPECT(eql(V0 / V1, vec2_t(S(+5), S(8) / S(12))));
			PR_EXPECT(eql(V0 % V1, vec2_t(S(+0), S(8))));

			PR_EXPECT(eql(V0 * S(3), vec2_t(S(30), S(24))));
			PR_EXPECT(eql(V0 / S(2), vec2_t(S(5), S(4))));
			PR_EXPECT(eql(V0 % S(2), vec2_t(S(0), S(0))));

			PR_EXPECT(eql(S(3) * V0, vec2_t(S(30), S(24))));

			PR_EXPECT(eql(+V0, vec2_t(S(+10), S(+8))));
			PR_EXPECT(eql(-V0, vec2_t(S(-10), S(-8))));

			PR_EXPECT(V0 == vec2_t(S(10), S(8)));
			PR_EXPECT(V0 != vec2_t(S(2), S(1)));

			// Implicit conversion to T==void
			vec2_t V2 = Vec2<S, int>(S(1));

			// Explicit cast to T!=void
			Vec2<S, int> V3 = static_cast<Vec2<S, int>>(V2);
		}
		PRUnitTestMethod(MinMaxClamp, float, double, int32_t, int64_t)
		{
			using S = T;
			using vec2_t = Vec2<S, void>;

			auto V0 = vec2_t(S(+1), S(+2));
			auto V1 = vec2_t(S(-1), S(-2));
			auto V2 = vec2_t(S(+2), S(+4));

			PR_EXPECT(Min(V0, V1, V2) == vec2_t(S(-1), S(-2)));
			PR_EXPECT(Max(V0, V1, V2) == vec2_t(S(+2), S(+4)));
			PR_EXPECT(Clamp(V0, V1, V2) == vec2_t(S(1), S(2)));
			PR_EXPECT(Clamp(V0, S(0), S(1)) == vec2_t(S(1), S(1)));
		}
		PRUnitTestMethod(Normalise, float, double)
		{
			using S = T;
			using vec2_t = Vec2<S, void>;

			auto arr0 = vec2_t(S(1), S(2));
			auto len = Length(arr0);
			PR_EXPECT(FEql(Normalise(vec2_t::Zero(), arr0), arr0));
			PR_EXPECT(FEql(Normalise(arr0), vec2_t(S(1) / len, S(2) / len)));
			PR_EXPECT(IsNormal(Normalise(arr0)));
		}
		PRUnitTestMethod(CosAngle, float, double)
		{
			using S = T;
			using vec2_t = Vec2<S, void>;

			vec2_t arr0(S(1), S(0));
			vec2_t arr1(S(0), S(1));
			PR_EXPECT(FEql(CosAngle(arr0, arr1) - Cos(DegreesToRadians(S(90))), S(0)));
			PR_EXPECT(FEql(Angle(arr0, arr1), DegreesToRadians(S(90))));
		}
	};
}
#endif
