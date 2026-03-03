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
#include "pr/maths/matrix4x4.h"

namespace pr
{
	template <Scalar S, typename T>
	struct Vec8
	{
		// Notes:
		//  - Spatial vectors describe a vector at a point plus the field of vectors around that point. 
		//  - Don't define component accessors because this is not a normal coordinate vector.

		#pragma warning(push)
		#pragma warning(disable:4201) // nameless struct
		union
		{
			struct { Vec4<S, void> ang, lin; };
			struct { S arr[8]; };
		};
		#pragma warning(pop)

		Vec8() = default;
		constexpr explicit Vec8(S x)
			:ang(x)
			,lin(x)
		{}
		constexpr Vec8(Vec3<S,void> ang_, Vec3<S,void> lin_)
			:ang(ang_, S(0))
			,lin(lin_, S(0))
		{}
		constexpr Vec8(Vec4<S,void> ang_, Vec4<S,void> lin_)
			:ang(ang_)
			,lin(lin_)
		{}
		constexpr Vec8(S wx, S wy, S wz, S vx, S vy, S vz)
			:ang(wx, wy, wz, S(0))
			,lin(vx, vy, vz, S(0))
		{}
		constexpr Vec8(S wx, S wy, S wz, S ww, S vx, S vy, S vz, S vw)
			:ang(wx, wy, wz, ww)
			,lin(vx, vy, vz, vw)
		{}

		// Reinterpret as a different vector type
		template <typename U> explicit operator Vec8<S, U> const& () const
		{
			return reinterpret_cast<Vec8<S, U> const&>(*this);
		}
		template <typename U> explicit operator Vec8<S, U>& ()
		{
			return reinterpret_cast<Vec8<S, U>&>(*this);
		}
		operator Vec8<S, void> const& () const
		{
			return reinterpret_cast<Vec8<S, void> const&>(*this);
		}
		operator Vec8<S, void>& ()
		{
			return reinterpret_cast<Vec8<S, void>&>(*this);
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
		static constexpr Vec8 Zero()
		{
			return Vec8{Vec4<S,void>::Zero(), Vec4<S,void>::Zero()};
		}

		// Sample the vector field at 'ofs'
		// Returns the direction and magnitude of the vector field at 'ofs'
		Vec4<S,void> LinAt(Vec4<S,void> ofs) const
		{
			return Vec4<S,void>{lin + Cross(ang, ofs)};
		}

		// Sample the vector field at 'ofs'
		// Not really sure what the physical interpretation of this is.
		// Returns 'ang - ofs x lin', which is the angular required at 'ofs' to ensure that the angular
		// is constant over the whole vector field, given that 'ofs x lin' contributes to the angular.
		Vec4<S,void> AngAt(Vec4<S,void> ofs) const
		{
			return Vec4<S,void>{ang - Cross(ofs, lin)};
		}

		#pragma region Operators
		friend constexpr Vec8 pr_vectorcall operator + (Vec8<S,T> lhs)
		{
			return lhs;
		}
		friend constexpr Vec8 pr_vectorcall operator - (Vec8<S,T> lhs)
		{
			return Vec8{-lhs.ang, -lhs.lin};
		}
		friend Vec8 pr_vectorcall operator * (S lhs, Vec8<S,T> rhs)
		{
			return rhs * lhs;
		}
		friend Vec8 pr_vectorcall operator * (Vec8<S,T> lhs, S rhs)
		{
			return Vec8{lhs.ang * rhs, lhs.lin * rhs};
		}
		friend Vec8 pr_vectorcall operator / (Vec8<S,T> lhs, S rhs)
		{
			return Vec8{lhs.ang / rhs, lhs.lin / rhs};
		}
		friend Vec8 pr_vectorcall operator % (Vec8<S,T> lhs, S rhs)
		{
			return Vec8{lhs.ang % rhs, lhs.lin % rhs};
		}
		friend Vec8 pr_vectorcall operator + (Vec8<S,T> lhs, Vec8<S,T> rhs)
		{
			return Vec8{lhs.ang + rhs.ang, lhs.lin + rhs.lin};
		}
		friend Vec8 pr_vectorcall operator - (Vec8<S,T> lhs, Vec8<S,T> rhs)
		{
			return Vec8{lhs.ang - rhs.ang, lhs.lin - rhs.lin};
		}
		friend Vec8 pr_vectorcall operator * (Vec8<S,T> lhs, Vec8<S,T> rhs)
		{
			return Vec8{lhs.ang * rhs.ang, lhs.lin * rhs.lin};
		}
		friend Vec8 pr_vectorcall operator / (Vec8<S,T> lhs, Vec8<S,T> rhs)
		{
			return Vec8{lhs.ang / rhs.ang, lhs.lin / rhs.lin};
		}
		friend Vec8 pr_vectorcall operator % (Vec8<S,T> lhs, Vec8<S,T> rhs)
		{
			return Vec8{lhs.ang % rhs.ang, lhs.lin % rhs.lin};
		}
		#pragma endregion
	};
	#define PR_VEC8_CHECKS(scalar)\
	static_assert(sizeof(Vec8<scalar,void>) == 2*4*sizeof(scalar), "Vec8<"#scalar"> has the wrong size");\
	static_assert(maths::VectorX<Vec8<scalar,void>>, "Vec8<"#scalar"> is not a vector");\
	static_assert(std::is_trivially_copyable_v<Vec8<scalar,void>>, "Vec8<"#scalar"> must be a pod type");\
	static_assert(std::alignment_of_v<Vec8<scalar,void>> == std::alignment_of_v<Vec4<scalar,void>>, "Vec8<"#scalar"> is not aligned correctly");
	PR_VEC8_CHECKS(float);
	PR_VEC8_CHECKS(double);
	PR_VEC8_CHECKS(int32_t);
	PR_VEC8_CHECKS(int64_t);
	#undef PR_VEC8_CHECKS

	// Compare for floating point equality
	template <Scalar S, typename T> inline bool FEql(Vec8<S,T> lhs, Vec8<S,T> rhs)
	{
		return
			FEql(lhs.ang, rhs.ang) &&
			FEql(lhs.lin, rhs.lin);
	}

	// Project a vector onto an axis. Loosely "dot(vec,axis)*axis"
	template <Scalar S, typename T> inline Vec8<S,T> Proj(Vec8<S,T> const& vec, Vec4<S,void> axis)
	{
		return Vec8<S,T>{
			Dot(vec.ang, axis) * axis,
			Dot(vec.lin, axis) * axis};
	}

	// Reflect a vector. Reverses the components of 'vec' in the direction of 'normal'
	template <Scalar S, typename T> inline Vec8<S,T> Reflect(Vec8<S,T> const& vec, Vec4<S,void> normal)
	{
		return vec - S(2) * Proj(vec, normal);
	}
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
namespace pr::maths
{
	PRUnitTest(Vector8Tests, float, double)
	{
		using S = T;
		using vec8_t = Vec8<S, void>;
		using vec4_t = Vec4<S, void>;

		std::default_random_engine rng;
		{// LinAt, AngAt
			auto v = vec8_t{vec4_t::Random(rng, vec4_t{}, S(10), S(0)), vec4_t::Random(rng, vec4_t{}, S(10), S(0))};
			auto lin = v.LinAt(vec4_t::Origin());
			auto ang = v.AngAt(vec4_t::Origin());
			auto V = vec8_t{ang, lin};
			PR_EXPECT(FEql(v, V));
		}
		{// LinAt, AngAt
			auto v = vec8_t{0, 0, 1, 0, 1, 0};

			auto lin0 = v.LinAt(vec4_t{-1,0,0,0});
			auto ang0 = v.AngAt(vec4_t{-1,0,0,0});
			PR_EXPECT(FEql(lin0, vec4_t{0,0,0,0}));
			PR_EXPECT(FEql(ang0, vec4_t{0,0,2,0}));

			auto lin1 = v.LinAt(vec4_t{0,0,0,0});
			auto ang1 = v.AngAt(vec4_t{0,0,0,0});
			PR_EXPECT(FEql(lin1, vec4_t{0,1,0,0}));
			PR_EXPECT(FEql(ang1, vec4_t{0,0,1,0}));

			auto lin2 = v.LinAt(vec4_t{+1,0,0,0});
			auto ang2 = v.AngAt(vec4_t{+1,0,0,0});
			PR_EXPECT(FEql(lin2, vec4_t{0,2,0,0}));
			PR_EXPECT(FEql(ang2, vec4_t{0,0,0,0}));

			auto lin3 = v.LinAt(vec4_t{+2,0,0,0});
			auto ang3 = v.AngAt(vec4_t{+2,0,0,0});
			PR_EXPECT(FEql(lin3, vec4_t{0,3,0,0}));
			PR_EXPECT(FEql(ang3, vec4_t{0,0,-1,0}));

			auto lin4 = v.LinAt(vec4_t{+3,0,0,0});
			auto ang4 = v.AngAt(vec4_t{+3,0,0,0});
			PR_EXPECT(FEql(lin4, vec4_t{0,4,0,0}));
			PR_EXPECT(FEql(ang4, vec4_t{0,0,-2,0}));
		}
		{// Projection
			auto v = vec8_t{1,-2,3,-3,2,-1};
			auto vn = Proj(v, vec4_t::ZAxis());
			auto vt = v - vn;
			auto r = vn + vt;
			PR_EXPECT(FEql(vn, vec8_t{0,0,3,0,0,-1}));
			PR_EXPECT(FEql(vt, vec8_t{1,-2,0,-3,2,0}));
			PR_EXPECT(FEql(r, v));
		}
		{// Projection/Reflect
			auto v = vec8_t{0, 0, 1, 0, 1, 0};
			auto n = vec4_t::Normal(-1,-1,-1,0);
			auto r = vec8_t{S(-0.6666666666666), S(-0.6666666666666), S(0.3333333333333), S(-0.6666666666666), S(0.33333333333333), S(-0.6666666666666)};
			auto R = Reflect(v, n);
			PR_EXPECT(FEql(r, R));
		}
	}
}
#endif