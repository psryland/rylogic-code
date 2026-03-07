//*****************************************************************************
// Maths library
//  Copyright (c) Rylogic Ltd 2002
//*****************************************************************************
#pragma once
#include "pr/math/core/forward.h"
#include "pr/math/core/traits.h"
#include "pr/math/types/vector2.h"
#include "pr/math/types/vector4.h"
#include "pr/math/types/quaternion.h"
#include "pr/math/types/matrix3x4.h"
#include "pr/math/types/matrix4x4.h"

namespace pr::math
{
	template <ScalarType S>
	struct Xform
	{
		static_assert(std::floating_point<S>, "Xform requires a floating-point scalar type");
		using element_t = S;

		// Notes:
		//  - Xform cannot represent shear, but if 'scl' is non-uniform then mathematically, multiplication should result
		//    in a transform containing shear. The standard way to handle this is to silently discard shear, so:
		//     - Scale multiplies component-wise
		//     - Rotation multiplies normally
		//     - Position is scaled, then rotated
		// - This means that:
		//       Mat4x4 * Mat4x4 != Xform * Xform, if scale is not uniform

		Vec4<S> pos;
		Quat<S> rot;
		Vec4<S> scl;

		// Construct
		Xform() = default;
		constexpr Xform(Vec4<S> p, Quat<S> r, Vec4<S> s) noexcept
			: pos(p)
			, rot(r)
			, scl(s)
		{}
		constexpr Xform(Vec4<S> p, Quat<S> r) noexcept
			: pos(p)
			, rot(r)
			, scl(Vec4<S>::One())
		{}
		Xform(Vec4<S> p, Mat3x4<S> const& r) noexcept
		{
			auto [r_norm, scale] = Normalise(r);
			pos = p;
			rot = Quat<S>(r_norm);
			scl = scale.w1();
		}
		explicit Xform(Mat4x4<S> const& m) noexcept
			: Xform(m.pos, m.rot)
		{}

		// Basic constants
		static constexpr Xform const& Identity() noexcept
		{
			static auto s_identity = Xform{ Vec4<S>::Origin(), Quat<S>::Identity(), Vec4<S>::One() };
			return s_identity;
		}

		// Return the transform with scale set to one
		constexpr Xform s1() const noexcept
		{
			return Xform{ pos, rot, Vec4<S>::One() };
		}

		#pragma region Operators
		friend Xform pr_vectorcall operator * (Xform const& lhs, Xform const& rhs) noexcept
		{
			return Xform{
				Rotate(lhs.rot, lhs.scl * rhs.pos.w0()) + lhs.pos,
				lhs.rot * rhs.rot,
				lhs.scl * rhs.scl,
			};
		}
		friend Vec4<S> pr_vectorcall operator * (Xform const& lhs, Vec4<S> rhs) noexcept
		{
			return
				Rotate(lhs.rot, lhs.scl * rhs.w0()) +
				lhs.pos * rhs.w;
		}
		friend Quat<S> pr_vectorcall operator * (Xform const& lhs, Quat<S> rhs) noexcept
		{
			return lhs.rot * rhs;
		}
		#pragma endregion
	};
	#define PR_XFORM_CHECKS(scalar)\
	static_assert(sizeof(Xform<scalar>) == 12 * sizeof(scalar), "Xform<" #scalar "> has the wrong size");\
	static_assert(std::is_trivially_copyable_v<Xform<scalar>>, "Xform<" #scalar "> must be a pod type");\
	static_assert(std::alignment_of_v<Xform<scalar>> == std::alignment_of_v<Vec4<scalar>>, "Xform<" #scalar "> is not aligned correctly");
	PR_XFORM_CHECKS(float);
	PR_XFORM_CHECKS(double);
	#undef PR_XFORM_CHECKS

	// Concept for detecting Xform types
	template <typename T> struct is_xform : std::false_type {};
	template <ScalarType S> struct is_xform<Xform<S>> : std::true_type {};
	template <typename T> concept XformType = is_xform<std::remove_cvref_t<T>>::value;

	// Deferred definition of Mat4x4(Xform) constructor
	template <ScalarType S>
	constexpr Mat4x4<S>::Mat4x4(Xform<S> const& xform) noexcept requires (std::floating_point<S>)
	{
		auto rotation = math::ToMatrix<Mat3x4<S>>(xform.rot);
		auto scale_mat = math::Scale<Mat3x4<S>>(xform.scl);
		rot = rotation * scale_mat;
		pos = xform.pos;
	}

	// Approximate equality (absolute tolerance)
	template <ScalarTypeFP S>
	inline bool pr_vectorcall FEqlAbsolute(Xform<S> const& lhs, Xform<S> const& rhs, S tol) noexcept
	{
		return
			FEqlOrientation(lhs.rot, rhs.rot, tol) &&
			FEqlAbsolute(lhs.pos, rhs.pos, tol) &&
			FEqlAbsolute(lhs.scl, rhs.scl, tol);
	}

	// Approximate equality (relative tolerance)
	template <ScalarTypeFP S>
	inline bool pr_vectorcall FEqlRelative(Xform<S> const& lhs, Xform<S> const& rhs, S tol) noexcept
	{
		return
			FEqlOrientation(lhs.rot, rhs.rot, tol) &&
			FEqlRelative(lhs.pos, rhs.pos, tol) &&
			FEqlRelative(lhs.scl, rhs.scl, tol);
	}

	// Approximate equality (default tolerance)
	template <ScalarTypeFP S>
	inline bool pr_vectorcall FEql(Xform<S> const& lhs, Xform<S> const& rhs) noexcept
	{
		return FEqlRelative(lhs, rhs, tiny<S>);
	}

	// Invert a transform
	template <ScalarTypeFP S>
	inline Xform<S> Invert(Xform<S> const& xform) noexcept
	{
		auto inv_rot = ~xform.rot;
		auto inv_scl = S(1) / xform.scl;
		auto inv_pos = Rotate(inv_rot, inv_scl * -xform.pos.w0()).w1();
		return Xform<S>{ inv_pos, inv_rot, inv_scl };
	}

	// Create a random transform with position, rotation, and scale
	template <XformType X, typename Rng = std::default_random_engine>
	inline X Random(Rng& rng, Vec4<typename X::element_t> centre, typename X::element_t radius, Vec2<typename X::element_t> scale_range) noexcept
	{
		using S = typename X::element_t;

		auto p = Random<Vec4<S>>(rng, centre, S(radius));
		p.w = S(1);

		auto axis = RandomN<Vec3<S>>(rng);
		std::uniform_real_distribution<S> angle_dist(S(0), constants<S>::tau);
		auto q = Quat<S>(axis, angle_dist(rng));

		auto s = Random<Vec4<S>>(rng, Vec4<S>(scale_range.x), Vec4<S>(scale_range.y));
		s.w = S(1);

		return X(p, q, s);
	}
	template <XformType X, typename Rng = std::default_random_engine>
	inline X Random(Rng& rng, Vec4<typename X::element_t> centre, typename X::element_t radius) noexcept
	{
		using S = typename X::element_t;
		return Random<X>(rng, centre, radius, Vec2<S>{ S(1), S(1) });
	}
	template <XformType X, typename Rng = std::default_random_engine>
	inline X Random(Rng& rng, Vec2<typename X::element_t> scale_range) noexcept
	{
		using S = typename X::element_t;
		return Random<X>(rng, Vec4<S>::Origin(), S(1), scale_range);
	}
	template <XformType X, typename Rng = std::default_random_engine>
	inline X Random(Rng& rng) noexcept
	{
		using S = typename X::element_t;
		return Random<X>(rng, Vec2<S>{ S(1), S(1) });
	}
}