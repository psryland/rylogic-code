//*****************************************************************************
// Maths library
//  Copyright (c) Rylogic Ltd 2002
//*****************************************************************************
#pragma once
#include "pr/maths/forward.h"
#include "pr/maths/constants.h"
#include "pr/maths/maths_core.h"
#include "pr/maths/vector2.h"

namespace pr
{
	template <Scalar S, typename T>
	struct Vec3
	{
		#pragma warning(push)
		#pragma warning(disable:4201) // nameless struct
		union
		{
			struct { S x, y, z; };
			struct { Vec2<S, T> xy; };
			struct { S arr[3]; };
		};
		#pragma warning(pop)

		// Construct
		Vec3() = default;
		constexpr explicit Vec3(S x_)
			:x(x_)
			,y(x_)
			,z(x_)
		{}
		constexpr Vec3(S x_, S y_, S z_)
			:x(x_)
			,y(y_)
			,z(z_)
		{}
		constexpr explicit Vec3(S const* v)
			:Vec3(v[0], v[1], v[2])
		{}
		template <maths::Vector3 V> constexpr explicit Vec3(V const& v)
			:Vec3(maths::comp<0>(v), maths::comp<1>(v), maths::comp<2>(v))
		{}
		template <maths::Vector2 V> constexpr Vec3(V const& v, S z_)
			:Vec3(maths::comp<0>(v), maths::comp<1>(v), z_)
		{}

		// Reinterpret as a different vector type
		template <typename U> explicit operator Vec3<S, U> const& () const
		{
			return reinterpret_cast<Vec3<S, U> const&>(*this);
		}
		template <typename U> explicit operator Vec3<S, U>&()
		{
			return reinterpret_cast<Vec3<S, U>&>(*this);
		}
		operator Vec3<S, void> const& () const
		{
			return reinterpret_cast<Vec3<S, void> const&>(*this);
		}
		operator Vec3<S, void>& ()
		{
			return reinterpret_cast<Vec3<S, void>&>(*this);
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

		// Create other vector types
		Vec4<S, T> w0() const;
		Vec4<S, T> w1() const;
		Vec2<S, T> vec2(int i0, int i1) const
		{
			return Vec2<S, T>(arr[i0], arr[i1]);
		}

		// Basic constants
		static constexpr Vec3 Zero()     { return Vec3(S(0), S(0), S(0)); }
		static constexpr Vec3 XAxis()    { return Vec3(S(1), S(0), S(0)); }
		static constexpr Vec3 YAxis()    { return Vec3(S(0), S(1), S(0)); }
		static constexpr Vec3 ZAxis()    { return Vec3(S(0), S(0), S(1)); }
		static constexpr Vec3 One()      { return Vec3(S(1), S(1), S(1)); }
		static constexpr Vec3 TinyF()    { return Vec3(maths::tiny<S>, maths::tiny<S>, maths::tiny<S>); }
		static constexpr Vec3 Min()      { return Vec3(limits<S>::min(), limits<S>::min(), limits<S>::min()); }
		static constexpr Vec3 Max()      { return Vec3(limits<S>::max(), limits<S>::max(), limits<S>::max()); }
		static constexpr Vec3 Lowest()   { return Vec3(limits<S>::lowest(), limits<S>::lowest(), limits<S>::lowest()); }
		static constexpr Vec3 Epsilon()  { return Vec3(limits<S>::epsilon(), limits<S>::epsilon(), limits<S>::epsilon()); }
		static constexpr Vec3 Infinity() { return Vec3(limits<S>::infinity(), limits<S>::infinity(), limits<S>::infinity()); }

		// Construct normalised
		static Vec3 Normal(S x, S y, S z) requires std::is_floating_point_v<S>
		{
			return Normalise(Vec3(x, y, z));
		}

		// Create a random vector with unit length
		template <typename Rng = std::default_random_engine> requires std::is_floating_point_v<S>
		static Vec3 pr_vectorcall RandomN(Rng& rng)
		{
			std::uniform_real_distribution<S> dist(S(-1), S(1));
			for (;;)
			{
				auto x = dist(rng);
				auto y = dist(rng);
				auto z = dist(rng);
				auto v = Vec3(x, y, z);
				auto len = LengthSq(v);
				if (len > S(0.01) && len <= S(1))
					return v / Sqrt(len);
			}
		}

		// Create a random vector with components on interval ['vmin', 'vmax']
		template <typename Rng = std::default_random_engine> requires std::is_floating_point_v<S>
		static Vec3 pr_vectorcall Random(Rng& rng, Vec3 vmin, Vec3 vmax)
		{
			std::uniform_real_distribution<S> dist_x(vmin.x, vmax.x);
			std::uniform_real_distribution<S> dist_y(vmin.y, vmax.y);
			std::uniform_real_distribution<S> dist_z(vmin.z, vmax.z);
			return Vec3(dist_x(rng), dist_y(rng), dist_z(rng));
		}

		// Create a random vector with length on interval [min_length, max_length]
		template <typename Rng = std::default_random_engine> requires std::is_floating_point_v<S>
		static Vec3 pr_vectorcall Random(Rng& rng, S min_length, S max_length)
		{
			std::uniform_real_distribution<S> dist(min_length, max_length);
			return dist(rng) * RandomN(rng);
		}

		// Create a random vector centred on 'centre' with radius 'radius'
		template <typename Rng = std::default_random_engine> requires std::is_floating_point_v<S>
		static Vec3 pr_vectorcall Random(Rng& rng, Vec3 centre, S radius)
		{
			return Random(rng, S(0), radius) + centre;
		}

		#pragma region Operators
		friend constexpr Vec3 operator + (Vec3 vec)
		{
			return vec;
		}
		friend constexpr Vec3 operator - (Vec3 vec)
		{
			return Vec3(-vec.x, -vec.y, -vec.z);
		}
		friend Vec3 operator * (S lhs, Vec3 rhs)
		{
			return rhs * lhs;
		}
		friend Vec3 operator * (Vec3 lhs, S rhs)
		{
			return Vec3(lhs.x * rhs, lhs.y * rhs, lhs.z * rhs);
		}
		friend Vec3 operator / (Vec3 lhs, S rhs)
		{
			// Don't check for divide by zero by default. For Scalars +inf/-inf are valid results
			return Vec3(lhs.x / rhs, lhs.y / rhs, lhs.z / rhs);
		}
		friend Vec3 operator % (Vec3 lhs, S rhs)
		{
			// Don't check for divide by zero by default. For Scalars +inf/-inf are valid results
			if constexpr (std::floating_point<S>)
				return Vec3(Fmod(lhs.x, rhs), Fmod(lhs.y, rhs), Fmod(lhs.z, rhs));
			else
				return Vec3(lhs.x % rhs, lhs.y % rhs, lhs.z % rhs);
		}
		friend Vec3 operator + (Vec3 lhs, Vec3 rhs)
		{
			return Vec3(lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z);
		}
		friend Vec3 operator - (Vec3 lhs, Vec3 rhs)
		{
			return Vec3(lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z);
		}
		friend Vec3 operator * (Vec3 lhs, Vec3 rhs)
		{
			return Vec3(lhs.x * rhs.x, lhs.y * rhs.y, lhs.z * rhs.z);
		}
		friend Vec3 operator / (Vec3 lhs, Vec3 rhs)
		{
			// Don't check for divide by zero by default. For Scalars +inf/-inf are valid results
			return Vec3(lhs.x / rhs.x, lhs.y / rhs.y, lhs.z / rhs.z);
		}
		friend Vec3 operator % (Vec3 lhs, Vec3 rhs)
		{
			// Don't check for divide by zero by default. For Scalars +inf/-inf are valid results
			if constexpr (std::floating_point<S>)
				return Vec3(Fmod(lhs.x, rhs.x), Fmod(lhs.y, rhs.y), Fmod(lhs.z, rhs.z));
			else
				return Vec3(lhs.x % rhs.x, lhs.y % rhs.y, lhs.z % rhs.z);
		}
		#pragma endregion
	};
	#define PR_VEC3_CHECKS(scalar)\
	static_assert(sizeof(Vec3<scalar, void>) == 3 * sizeof(scalar), "Vector<"#scalar"> has the wrong size");\
	static_assert(maths::Vector3<Vec3<scalar, void>>, "Vector<"#scalar"> is not a Vector3");\
	static_assert(std::is_trivially_copyable_v<Vec3<scalar, void>>, "Must be a pod type");
	PR_VEC3_CHECKS(float);
	PR_VEC3_CHECKS(double);
	PR_VEC3_CHECKS(int32_t);
	PR_VEC3_CHECKS(int64_t);
	#undef PR_VEC3_CHECKS

	// Dot product: a . b
	template <Scalar S, typename T> constexpr S pr_vectorcall Dot(Vec3<S, T> lhs, Vec3<S, T> rhs)
	{
		return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
	}

	// Cross product: a x b
	template <Scalar S, typename T> constexpr Vec3<S, T> pr_vectorcall Cross(Vec3<S, T> lhs, Vec3<S, T> rhs)
	{
		return Vec3<S, T>{lhs.y*rhs.z - lhs.z*rhs.y, lhs.z*rhs.x - lhs.x*rhs.z, lhs.x*rhs.y - lhs.y*rhs.x};
	}

	// Triple product: a . b x c
	template <Scalar S, typename T> constexpr S pr_vectorcall Triple(Vec3<S, T> a, Vec3<S, T> b, Vec3<S, T> c)
	{
		return Dot(a, Cross(b, c));
	}

	// Returns a vector with the values permuted 'n' times. 0=xyz, 1=yzx, 2=zxy, etc
	template <Scalar S, typename T> constexpr Vec3<S, T> pr_vectorcall Permute(Vec3<S, T> v, int n)
	{
		switch (n % 3)
		{
			case 1:  return Vec3<S, T>{v.y, v.z, v.x};
			case 2:  return Vec3<S, T>{v.z, v.x, v.y};
			default: return v;
		}
	}

	// Returns a 3-bit bitmask of the octant the vector is in. 0=(-x,-y,-z), 1=(+x,-y,-z), 2=(-x,+y,-z), 3=(+x,+y,-z), 4=(-x,-y+z), 5=(+x,-y,+z), 6=(-x,+y,+z), 7=(+x,+y,+z)
	template <Scalar S, typename T> constexpr uint32_t Octant(Vec3<S, T> v)
	{
		return
			((v.x >= S(0)) << 0) |
			((v.y >= S(0)) << 1) |
			((v.z >= S(0)) << 2);
	}

}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
namespace pr::maths
{
	PRUnitTestClass(Vector3Tests)
	{
		PRUnitTestMethod(Create, float, double, int32_t, int64_t)
		{
			using S = T;
			using vec3_t = Vec3<S, void>;

			auto V0 = vec3_t(S(1));
			PR_EXPECT(V0.x == S(1));
			PR_EXPECT(V0.y == S(1));
			PR_EXPECT(V0.z == S(1));

			auto V1 = vec3_t(S(1), S(2), S(3));
			PR_EXPECT(V1.x == S(1));
			PR_EXPECT(V1.y == S(2));

			auto V2 = vec3_t({ S(3), S(4), S(5) });
			PR_EXPECT(V2.x == S(3));
			PR_EXPECT(V2.y == S(4));
			PR_EXPECT(V2.z == S(5));

			vec3_t V3 = { S(4), S(5), S(6) };
			PR_EXPECT(V3[0] == S(4));
			PR_EXPECT(V3[1] == S(5));
			PR_EXPECT(V3[2] == S(6));
		}
		PRUnitTestMethod(Normal, float, double)
		{
			using S = T;
			using vec3_t = Vec3<S, void>;

			auto V4 = vec3_t::Normal(S(3), S(4), S(5));
			auto V4_expected = vec3_t(S(0.42426406871192), S(0.56568542494923), S(0.70710678118654));
			PR_EXPECT(FEql(V4, V4_expected));
		}
	};
}
#endif
