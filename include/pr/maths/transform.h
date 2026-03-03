//*****************************************************************************
// Maths library
//  Copyright (c) Rylogic Ltd 2002
//*****************************************************************************
#pragma once
#include "pr/maths/forward.h"
#include "pr/maths/constants.h"
#include "pr/maths/maths_core.h"
#include "pr/maths/vector2.h"
#include "pr/maths/vector4.h"
#include "pr/maths/quaternion.h"
#include "pr/maths/matrix3x4.h"
#include "pr/maths/matrix4x4.h"

namespace pr
{
	template <Scalar S, typename A, typename B>
	struct Xform
	{
		// Notes:
		//  - Xform cannot represent shear, but if 'scl' is non-uniform then mathematically, multiplication should result
		//    in a transform containing shear. The standard way to handle this is to silently discard shear, so:
		//     - Scale multiplies component-wise
		//     - Rotation multiplies normally
		//     - Position is scaled, then rotated
		// - This means that:
		//       Mat4x4 * Mat4x4 != Xform * Xform, if scale is not uniform

		Vec4<S, A> pos;
		Quat<S, A, B> rot;
		Vec4<S, void> scl;

		// Construct
		Xform() = default;
		constexpr Xform(Vec4<S, void> p, Quat<S,void,void> r, Vec4<S, void> s)
			: pos(p)
			, rot(r)
			, scl(s)
		{}
		constexpr Xform(Vec4<S, void> p, Quat<S,void,void> r)
			: pos(p)
			, rot(r)
			, scl(Vec4<S, void>::One())
		{}
		Xform(Vec4<S, A> p, Mat3x4<S,A,B> const& r)
		{
			auto [r_norm, scale] = Normalise(r);
			pos = p;
			rot = Quat<S, A, B>(r_norm);
			scl = scale.w1();
		}
		Xform(Mat4x4<S,A,B> const& m)
			:Xform(m.pos, m.rot)
		{}

		// Return the transform with scale set to one
		Xform s1() const
		{
			return Xform{ pos, rot, Vec4<S, void>::One() };
		}

		// Basic constants
		static constexpr Xform Identity()
		{
			return { Vec4<S, A>::Origin(), Quat<S, A, B>::Identity(), Vec4<S, void>::One() };
		}

		// Create a random transform
		template <typename Rng = std::default_random_engine> static Xform Random(Rng& rng, Vec4<S,void> centre, S radius, Vec2<S, void> scale_range)
		{
			return Xform(
				Vec4<S, void>::Random(rng, centre, radius, S(1)),
				Quat<S, void, void>::Random(rng),
				Vec4<S, void>::Random(rng, Vec4<S, void>(scale_range.x), Vec4<S, void>(scale_range.y), S(1))
			);
		}
		template <typename Rng = std::default_random_engine> static Xform Random(Rng& rng, Vec4<S,void> centre, S radius)
		{
			return Random(rng, centre, radius, { 1, 1 });
		}
		template <typename Rng = std::default_random_engine> static Xform Random(Rng& rng, Vec2<S, void> scale_range)
		{
			return Random(rng, Vec4<S, void>::Origin(), S(1), scale_range);
		}
		template <typename Rng = std::default_random_engine> static Xform Random(Rng& rng)
		{
			return Random(rng, Vec2<S, void>{ 1, 1 });
		}

		#pragma region Operators
		friend constexpr Xform pr_vectorcall operator + (Xform const& rhs)
		{
			return rhs;
		}
		friend constexpr Xform pr_vectorcall operator - (Xform const& rhs)
		{
			return Xform{ -rhs.pos, -rhs.rot, -rhs.scl };
		}
		friend Xform pr_vectorcall operator * (Xform const& lhs, Xform const& rhs)
		{
			return Xform{
				lhs.rot * (lhs.scl * rhs.pos.w0()) + lhs.pos,
				lhs.rot * rhs.rot,
				lhs.scl * rhs.scl
			};
		}
		friend Vec4<S,B> pr_vectorcall operator * (Xform const& lhs, Vec4<S, A> rhs)
		{
			return
				lhs.rot * (lhs.scl * rhs.w0()) +
				lhs.pos * rhs.w;
		}
		friend Quat<S,A,B> pr_vectorcall operator * (Xform const& lhs, Quat<S,A,B> rhs)
		{
			return lhs.rot * rhs;
		}
		#pragma endregion
	};
	#define PR_XFORM_CHECKS(scalar)\
	static_assert(sizeof(Xform<scalar,void,void>) == 12*sizeof(scalar), "Xform<"#scalar"> has the wrong size");\
	static_assert(std::is_trivially_copyable_v<Xform<scalar,void,void>>, "Xform<"#scalar"> must be a pod type");\
	static_assert(std::alignment_of_v<Xform<scalar,void,void>> == std::alignment_of_v<Vec4<scalar,void>>, "Xform<"#scalar"> is not aligned correctly");
	PR_XFORM_CHECKS(float);
	PR_XFORM_CHECKS(double);
	#undef PR_XFORM_CHECKS

	template <Scalar S, typename A, typename B> inline Mat4x4<S,A,B>::Mat4x4(Xform<S, void, void> const& xform) requires (std::floating_point<S>)
	{
		rot = Mat3x4<S, A, B>(xform.rot) * Mat3x4<S, A, B>::Scale(xform.scl.xyz);
		pos = xform.pos;
	}

	// FEql
	template <Scalar S, typename A, typename B> inline bool pr_vectorcall FEqlAbsolute(Xform<S, A, B> const& lhs, Xform<S, A, B> const& rhs, S tol)
	{
		return
			FEqlRelative(lhs.rot, rhs.rot, tol) &&
			FEqlAbsolute(lhs.pos, rhs.pos, tol) &&
			FEqlAbsolute(lhs.scl, rhs.scl, tol);
	}
	template <Scalar S, typename A, typename B> inline bool pr_vectorcall FEqlRelative(Xform<S, A, B> const& lhs, Xform<S, A, B> const& rhs, S tol)
	{
		return
			FEqlRelative(lhs.rot, rhs.rot, tol) &&
			FEqlRelative(lhs.pos, rhs.pos, tol) &&
			FEqlRelative(lhs.scl, rhs.scl, tol);
	}
	template <Scalar S, typename A, typename B> inline bool pr_vectorcall FEql(Xform<S, A, B> const& lhs, Xform<S, A, B> const& rhs)
	{
		return FEqlRelative(lhs, rhs, maths::tiny<S>);
	}

	/// <summary>Invert a transform</summary>
	template <Scalar S, typename A, typename B> inline Xform<S,B,A> Invert(Xform<S,A,B> const& xform)
	{
		auto inv_rot = ~xform.rot;
		auto inv_scl = 1.f / xform.scl;
		auto inv_pos = inv_rot * (inv_scl * -xform.pos.w0()).w1();
		return Xform<S, B, A>{ inv_pos, inv_rot, inv_scl };
	}
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
namespace pr::maths
{
	PRUnitTestClass(XformTests)
	{
		std::default_random_engine rng;
		TestClass_XformTests()
			:rng(1u)
		{
		}

		PRUnitTestMethod(ConstructionRoundTrip, float, double)
		{
			using xform_t = Xform<T, void, void>;
			using m4x4_t = Mat4x4<T, void, void>;
			using v4_t = Vec4<T, void>;
			using v2_t = Vec2<T, void>;

			auto xf1 = xform_t::Random(rng, v4_t(1,1,1,1), T(2), v2_t(T(0.2), T(1.5)));
			auto m = m4x4_t(xf1);
			auto xf2 = xform_t(m);

			PR_EXPECT(FEql(xf1, xf2));
		}
		PRUnitTestMethod(Multiply, float, double)
		{
			using xform_t = Xform<T, void, void>;
			using m4x4_t = Mat4x4<T, void, void>;

			// Scale must be uniform, or multiply would result in shear
			auto xf1 = xform_t::Random(rng, {T(2), T(2)});
			auto xf2 = xform_t::Random(rng, {T(3), T(3)});
			auto xf3 = xf1 * xf2;

			auto m1 = m4x4_t(xf1);
			auto m2 = m4x4_t(xf2);
			auto m3 = m1 * m2;

			auto xf3_from_m = xform_t(m3);
			auto m3_from_xf = m4x4_t(xf3);

			// Note: ** This is only true if scale is uniform **
			PR_EXPECT(FEql(xf3, xf3_from_m));
			PR_EXPECT(FEql(m3, m3_from_xf));
		}
		PRUnitTestMethod(MultiplyVector, float, double)
		{
			using xform_t = Xform<T, void, void>;
			using m4x4_t = Mat4x4<T, void, void>;
			using v4_t = Vec4<T, void>;

			auto v0 = v4_t(1, 2, 3, 1);
			auto v1 = v4_t(1, 2, 3, 0);

			auto xf = xform_t::Random(rng);
			auto m = m4x4_t(xf);

			v4_t r0 = xf * v0;
			v4_t r1 = xf * v1;

			v4_t R0 = m * v0;
			v4_t R1 = m * v1;

			PR_EXPECT(FEql(r0, R0));
			PR_EXPECT(FEql(r1, R1));
		}
		PRUnitTestMethod(Inversion, float, double)
		{
			using xform_t = Xform<T, void, void>;
			using m4x4_t = Mat4x4<T, void, void>;

			auto xf = xform_t::Random(rng);
			auto m = m4x4_t(xf);

			auto xf_inv = Invert(xf);
			auto m_inv = Invert(m);

			auto r = m4x4_t(xf_inv);
			auto R = m_inv;
			PR_EXPECT(FEql(r, R));

			auto xf2 = Invert(xf_inv);
			auto m2 = Invert(m_inv);
			PR_EXPECT(FEql(xf, xf2));
			PR_EXPECT(FEql(m, m2));
		}
	};
}
#endif