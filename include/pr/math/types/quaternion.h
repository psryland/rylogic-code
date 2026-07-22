//*****************************************************************************
// Maths library
//  Copyright (c) Rylogic Ltd 2002
//*****************************************************************************
#pragma once
#include "pr/math/core/forward.h"
#include "pr/math/core/traits.h"
#include "pr/math/core/constants.h"
#include "pr/math/types/vector4.h"
#include "pr/math/types/vector3.h"
#include "pr/math/types/vector2.h"

namespace pr::math
{
	template <ScalarTypeFP S>
	struct Quat
	{
		enum
		{
			IntrinsicF = PR_MATHS_USE_INTRINSICS && std::is_same_v<S, float>,
			IntrinsicD = PR_MATHS_USE_INTRINSICS && std::is_same_v<S, double>,
			NoIntrinsic = PR_MATHS_USE_INTRINSICS == 0,
		};
		using intrinsic_t =
			std::conditional_t<IntrinsicF, __m128,
			std::conditional_t<IntrinsicD, __m256d,
			std::byte[4*sizeof(S)]
			>>;

		#pragma warning(push)
		#pragma warning(disable:4201) // nameless struct
		union alignas(4 * sizeof(S))
		{
			struct { S x, y, z, w; };
			struct { Vec2<S> xy, zw; };
			struct { Vec3<S> xyz; };
			struct { Vec4<S> xyzw; };
			struct { S arr[4]; };
			intrinsic_t vec;
		};
		#pragma warning(pop)

		// Construct
		Quat() = default;
		constexpr Quat(S x_, S y_, S z_, S w_) noexcept
			: x(x_)
			, y(y_)
			, z(z_)
			, w(w_)
		{}
		// Build from a random-access range. Caller guarantees at least 4 readable elements;
		// for sized ranges the precondition is checked with pr_assert.
		template <std::ranges::random_access_range R>
			requires (!VectorType<std::remove_cvref_t<R>>)
			      && std::convertible_to<std::ranges::range_reference_t<R>, S>
		constexpr Quat(R&& v) noexcept
		{
			if constexpr (std::ranges::sized_range<R>)
				pr_assert(std::ranges::size(v) >= 4 && "range must have at least 4 elements");
			auto it = std::ranges::begin(v);
			x = static_cast<S>(*it);
			y = static_cast<S>(*(it + 1));
			z = static_cast<S>(*(it + 2));
			w = static_cast<S>(*(it + 3));
		}

		// Build from a same-scalar external rank-1 vector of dimension 4.
		// Uses pr::math::vec() (fully qualified) because Quat has a data member
		// named 'vec' that would otherwise shadow the free function.
		constexpr Quat(Rank1VecSN<S, 4> auto v) noexcept
			:Quat(
				static_cast<S>(pr::math::vec(v).x),
				static_cast<S>(pr::math::vec(v).y),
				static_cast<S>(pr::math::vec(v).z),
				static_cast<S>(pr::math::vec(v).w)
			)
		{}
		constexpr Quat(intrinsic_t vec_) noexcept requires (!NoIntrinsic)
			:vec(vec_)
		{}

		// Create a quaternion from a rotation matrix
		explicit Quat(Mat3x3<S> const& m) noexcept;

		// Explicit cast to different Scalar type
		template <ScalarTypeFP S2> constexpr explicit operator Quat<S2>() const noexcept
		{
			return Quat<S2>(
				static_cast<S2>(x),
				static_cast<S2>(y),
				static_cast<S2>(z),
				static_cast<S2>(w)
			);
		}

		// Explicit cast to bool. True if any component is non-zero.
		constexpr explicit operator bool() const
		{
			return Any(*this);
		}

		// Create a quaternion from an axis and an angle
		Quat(Vec3<S> axis, S angle) noexcept
		{
			auto s = std::sin(S(0.5) * angle);
			x = axis.x * s;
			y = axis.y * s;
			z = axis.z * s;
			w = std::cos(S(0.5) * angle);
		}
		Quat(Vec4<S> axis, S angle) noexcept
			:Quat(axis.xyz, angle)
		{
			pr_assert(axis.w == 0);
		}
		
		// Create a quaternion from Euler angles. Order is roll, pitch, yaw
		Quat(S pitch, S yaw, S roll) noexcept
		{
			S cos_p = std::cos(pitch * S(0.5)), sin_p = std::sin(pitch * S(0.5));
			S cos_y = std::cos(yaw   * S(0.5)), sin_y = std::sin(yaw   * S(0.5));
			S cos_r = std::cos(roll  * S(0.5)), sin_r = std::sin(roll  * S(0.5));
			x = sin_p * cos_y * cos_r + cos_p * sin_y * sin_r;
			y = cos_p * sin_y * cos_r - sin_p * cos_y * sin_r;
			z = cos_p * cos_y * sin_r - sin_p * sin_y * cos_r;
			w = cos_p * cos_y * cos_r + sin_p * sin_y * sin_r;
		}

		// Construct a quaternion from two vectors representing start and end orientations
		Quat(Vec3<S> from, Vec3<S> to) noexcept
		{
			auto d = Dot(from, to);
			auto s = Sqrt(LengthSq(from) * LengthSq(to)) + d;
			auto axis = Cross(from, to);

			// Vectors are aligned, 180 degrees apart, or one is zero
			if (FEql(s, S(0)))
			{
				s = S(0);
				axis =
					LengthSq(from) > tiny<S> ? Perpendicular(from) :
					LengthSq(to) > tiny<S> ? Perpendicular(to) :
					Vec3<S>::ZAxis();
			}

			xyzw = Normalise(Vec4<S>{axis.x, axis.y, axis.z, s});
		}
		Quat(Vec4<S> from, Vec4<S> to) noexcept
			:Quat(from.xyz, to.xyz)
		{
			pr_assert(from.w == 0 && to.w == 0);
		}

		// Array access
		constexpr S operator [] (int i) const noexcept
		{
			pr_assert(i >= 0 && i < _countof(arr) && "index out of range");
			return arr[i];
		}
		constexpr S& operator [] (int i) noexcept
		{
			pr_assert(i >= 0 && i < _countof(arr) && "index out of range");
			return arr[i];
		}

		// Constants
		static constexpr Quat const& Zero() noexcept
		{
			static auto s_zero = Quat(S(0), S(0), S(0), S(0));
			return s_zero;
		}
		static constexpr Quat const& Identity() noexcept
		{
			static auto s_identity = math::Identity<Quat>();
			return s_identity;
		}

		// Get the canonical shortest-arc axis component (normalised).
		Vec4<S> Axis() const noexcept
		{
			// Canonicalize to the positive-w hemisphere (matches Angle()); identity axis is arbitrary
			auto sign = w >= S(0) ? S(1) : S(-1);
			return Normalise(Vec4<S>{sign * x, sign * y, sign * z, S(0)}, Vec4<S>{0, 0, 1, 0});
		}

		// Return the angle of rotation about 'Axis()'
		S Angle() const noexcept
		{
			return std::acos(CosAngle());
		}

		// Return the cosine of the angle of rotation about 'Axis()'
		S CosAngle() const noexcept
		{
			// Trig:
			//' w == cos(θ/2)
			//' cos²(θ/2) = 0.5 * (1 + cos(θ))
			//' w² == cos²(θ/2) == 0.5 * (1 + cos(θ))
			//' cos(θ) = 2w² - 1

			// This is always the smallest arc
			return Clamp(S(2) * Sqr(w) - LengthSq(xyzw), -S(1), +S(1));
		}

		// Return the sine of the angle of rotation about 'Axis()'
		S SinAngle() const noexcept
		{
			// Trig:
			//'  w == cos(θ/2)
			//'  sin(θ) = 2 * sin(θ/2) * cos(θ/2)

			// Use abs(w) to canonicalize to the positive-w hemisphere, matching Angle()/CosAngle()
			auto sin_half_angle = Length(xyz);
			return S(2) * sin_half_angle * std::abs(w);
		}
	};

	#define PR_MATH_DEFINE_TYPE(element)\
	template <> struct vector_traits<Quat<element>>\
		: quaternion_traits_base<element>\
		, vector_access_member<Quat<element>, element, 4>\
	{\
		template <ScalarType S> using rebind = Vec4<S>;\
		static Vec4<element> cast_vec4(Quat<element> q) noexcept { return q.xyzw; }\
	};\
	\
	static_assert(QuaternionType<Quat<element>>, "Quat<"#element"> is not a valid quaternion type");\
	static_assert(sizeof(Quat<element>) == 4*sizeof(element), "Quat<"#element"> has the wrong size");\
	static_assert(std::is_trivially_copyable_v<Quat<element>>, "Quat<"#element"> is not trivially copyable");

	PR_MATH_DEFINE_TYPE(float);
	PR_MATH_DEFINE_TYPE(double);
	#undef PR_MATH_DEFINE_TYPE

	// Decompose a quaternion into axis (normalised) and angle (radians).
	// The angle is always in [0, π] (the shortest arc). q and -q represent the same orientation;
	// when w < 0 the quaternion is canonicalized to positive-w form before extraction, which
	// flips the axis direction to preserve the correct rotation sense.
	template <QuaternionType Quat>
	inline auto pr_vectorcall AxisAngle(Quat q) noexcept
	{
		using S = typename vector_traits<Quat>::element_t;
		struct R { Vec4<S> axis; S angle; };
		pr_assert(IsNormalised(q.xyzw) && "quaternion isn't normalised");

		// Canonicalize to the positive-w hemisphere so the half-angle is in [0, π/2]
		// and the angle in [0, π].
		auto sign = q.w >= S(0) ? S(1) : S(-1);
		auto sin_half_angle = Length(q.xyz);

		// Use atan2 for the angle — well-conditioned everywhere, unlike acos
		auto angle = S(2) * std::atan2(sin_half_angle, sign * q.w);

		// Normalise the xyz part of the canonical quaternion directly (avoids sqrt(1-w²) cancellation)
		auto axis = sin_half_angle > tiny<S>
			? Vec4<S>(sign * q.x / sin_half_angle, sign * q.y / sin_half_angle, sign * q.z / sin_half_angle, S(0))
			: Vec4<S>(S(0), S(0), S(1), S(0)); // arbitrary axis for identity

		return R{ axis, angle };
	}

	// Test two quaternions for equivalence (i.e. do they represent the same orientation)
	template <QuaternionType Quat>
	inline bool FEqlOrientation(Quat lhs, Quat rhs, typename vector_traits<Quat>::element_t tol = tiny<typename vector_traits<Quat>::element_t>) noexcept
	{
		// Tolerance is on 'angle' rather than the quaternion components
		using S = typename vector_traits<Quat>::element_t;
		return FEqlAbsolute(AxisAngle(rhs * ~lhs).angle, S(0), tol);
	}

	// Returns the value of 'cos(theta / 2)', where 'theta' is the angle between 'a' and 'b'
	template <QuaternionType Quat>
	constexpr typename vector_traits<Quat>::element_t pr_vectorcall CosHalfAngle(Quat a, Quat b) noexcept
	{
		// The relative orientation between 'a' and 'b' is given by z = 'a * conj(b)'
		// where operator * is a quaternion multiply. The 'w' component of a quaternion
		// multiply is given by: q.w = a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z;
		// which is the same as q.w = Dot4(a,b) since conjugate negates the x,y,z
		// components of 'b'. Remember: q.w = Cos(theta/2)
		return Dot(a.xyzw, b.xyzw);
	}

	// Return possible Euler angles for the quaternion 'q'
	template <QuaternionType Quat>
	inline auto pr_vectorcall EulerAngles(Quat q) noexcept
	{
		using vt = vector_traits<Quat>;
		using S = typename vt::element_t;

		// From Wikipedia
		auto q0 = vec(q).w;
		auto q1 = vec(q).x;
		auto q2 = vec(q).y;
		auto q3 = vec(q).z;

		return {
			.pitch = s_cast<S>(std::atan2(2.0 * (q0*q1 + q2*q3), 1.0 - 2.0 * (q1*q1 + q2*q2))),
			.roll = s_cast<S>(std::asin(2.0 * (q0*q2 - q3*q1))),
			.yaw = s_cast<S>(atan2(2.0 * (q0*q3 + q1*q2), 1.0 - 2.0 * (q2*q2 + q3*q3))),
		};
	}

	// Rotate a vector by a quaternion
	template <QuaternionType Quat, VectorTypeFP Vec> requires (IsRank1<Vec> && SameS<Quat, Vec> && vector_traits<Vec>::dimension >= 3)
	constexpr Vec pr_vectorcall Rotate(Quat lhs, Vec rhs) noexcept
	{
		using vt = vector_traits<Vec>;

		// This is an optimised version of: 'r = q*v*conj(q) for when v.w == 0'
		auto xx = vec(lhs).x * vec(lhs).x, xy = vec(lhs).x * vec(lhs).y, xz = vec(lhs).x * vec(lhs).z, xw = vec(lhs).x * vec(lhs).w;
		auto yy = vec(lhs).y * vec(lhs).y, yz = vec(lhs).y * vec(lhs).z, yw = vec(lhs).y * vec(lhs).w;
		auto zz = vec(lhs).z * vec(lhs).z, zw = vec(lhs).z * vec(lhs).w;
		auto ww = vec(lhs).w * vec(lhs).w;

		Vec res = {};
		vec(res).x = ww * vec(rhs).x + 2 * yw * vec(rhs).z - 2 * zw * vec(rhs).y + xx * vec(rhs).x + 2 * xy * vec(rhs).y + 2 * xz * vec(rhs).z - zz * vec(rhs).x - yy * vec(rhs).x;
		vec(res).y = 2 * xy * vec(rhs).x + yy * vec(rhs).y + 2 * yz * vec(rhs).z + 2 * zw * vec(rhs).x - zz * vec(rhs).y + ww * vec(rhs).y - 2 * xw * vec(rhs).z - xx * vec(rhs).y;
		vec(res).z = 2 * xz * vec(rhs).x + 2 * yz * vec(rhs).y + zz * vec(rhs).z - 2 * yw * vec(rhs).x - yy * vec(rhs).z + 2 * xw * vec(rhs).y - xx * vec(rhs).z + ww * vec(rhs).z;
		if constexpr (vt::dimension > 3) vec(res).w = vec(rhs).w;
		return res;
	}

	// Scale the rotation 'q' by 'frac'. Returns a rotation about the same axis but with angle scaled by 'frac'.
	// Uses the shortest-arc (positive-w canonical) form so that Scale(q,0.5) always bisects the
	// minor arc rather than the reflex arc when w < 0.
	template <QuaternionType Quat>
	inline Quat pr_vectorcall Scale(Quat q, typename vector_traits<Quat>::element_t frac) noexcept
	{
		using S = typename vector_traits<Quat>::element_t;
		pr_assert("quaternion isn't normalised" && IsNormalised(q.xyzw));

		// Canonicalize to the positive-w hemisphere for shortest-arc scaling.
		auto sign = q.w >= S(0) ? S(1) : S(-1);
		auto sin_half_angle = Length(q.xyz);

		// Use atan2 for the half-angle — well-conditioned everywhere, unlike acos
		auto half_angle = std::atan2(sin_half_angle, sign * q.w);
		auto scaled_half_angle = frac * half_angle;
		auto sin_ha = std::sin(scaled_half_angle);
		auto cos_ha = std::cos(scaled_half_angle);

		// Normalise the xyz part of the canonical quaternion directly (avoids sqrt(1-w²) cancellation near identity)
		if (sin_half_angle > tiny<S>)
		{
			auto scale = sin_ha / sin_half_angle;
			return Quat{sign * q.x * scale, sign * q.y * scale, sign * q.z * scale, cos_ha};
		}
		else
		{
			// Identity quaternion — arbitrary axis
			return Quat{S(0), S(0), S(0), cos_ha};
		}
	}

	// Spherically interpolate between quaternions
	template <QuaternionType Quat>
	inline Quat pr_vectorcall Slerp(Quat a, Quat b, typename vector_traits<Quat>::element_t frac) noexcept
	{
		using vt = vector_traits<Quat>;
		using S = typename vt::element_t;

		if (frac == S(0)) return a;
		if (frac == S(1)) return b;

		// Flip 'b' so that both quaternions are in the same hemisphere (since: q == -q)
		auto cos_angle = CosHalfAngle(a, b);
		auto b_ = cos_angle >= 0 ? b : -b;
		cos_angle = std::abs(cos_angle);

		// Calculate coefficients
		if (cos_angle < S(0.95))
		{
			auto angle     = std::acos(cos_angle);
			auto scale0    = std::sin((S(1) - frac) * angle);
			auto scale1    = std::sin((frac       ) * angle);
			auto sin_angle = std::sin(angle);
			auto lerped    = (scale0 * a.xyzw + scale1 * b_.xyzw) / sin_angle;
			return Quat{ vec(lerped).x, vec(lerped).y, vec(lerped).z, vec(lerped).w };
		}
		else // "a" and "b" quaternions are very close
		{
			auto lerped = a.xyzw + frac * (b_.xyzw - a.xyzw);
			Quat q{ vec(lerped).x, vec(lerped).y, vec(lerped).z, vec(lerped).w };
			return Normalise(q);
		}
	}

	// Logarithm map of quaternion to tangent space at identity.
	// Maps q to v = axis * (angle/2), with |v| ∈ [0, π/2]. Uses positive-w canonical form
	// so that q and -q (the same orientation) map to the same principal tangent vector.
	// At exactly |v| = π/2 (a 180-degree rotation) the axis direction is inherently
	// ambiguous; use FEqlOrientation rather than comparing v directly near that boundary.
	template <VectorType Vec, QuaternionType Quat> requires (IsRank1<Vec> && SameS<Quat, Vec> && vector_traits<Vec>::dimension >= 3)
	inline Vec pr_vectorcall LogMap(Quat q) noexcept
	{
		using S = typename vector_traits<Quat>::element_t;
		auto xyz0 = Vec{ vec(q).x, vec(q).y, vec(q).z };

		// Quat = [u·sin(A/2), cos(A/2)].
		// Canonicalize to the positive-w hemisphere so the half-angle is in [0, π/2].
		// A quaternion with w < 0 encodes a rotation > 180 degrees; negating both halves
		// recovers the equivalent rotation in [0, π].
		auto sign = vec(q).w >= S(0) ? S(1) : S(-1);
		auto sin_half_ang = Length(xyz0);
		auto ang_by_2 = std::atan2(sin_half_ang, sign * vec(q).w); // well-conditioned everywhere

		// Scale canonical xyz by (half_angle / sin_half_angle) to get axis * half_angle
		return sin_half_ang > tiny<S>
			? xyz0 * static_cast<S>(sign * ang_by_2 / sin_half_ang)
			: xyz0;
	}
	
	// Exponential map of tangent space at identity to quaternion
	template <QuaternionType Quat, VectorType Vec> requires (IsRank1<Vec> && SameS<Quat, Vec> && vector_traits<Vec>::dimension >= 3)
	inline Quat pr_vectorcall ExpMap(Vec v) noexcept
	{
		using S = typename vector_traits<Vec>::element_t;

		// Vec = (+/-)A * (-/+)u.
		auto ang_by_2 = Length(v); // By convention, log space uses Length = A/2
		auto cos_half_ang = std::cos(ang_by_2);
		auto sin_half_ang = std::sin(ang_by_2); // != sqrt(1 - cos_half_ang²) when ang_by_2 > tau/2
		auto s = ang_by_2 > tiny<S> ? static_cast<S>(sin_half_ang / ang_by_2) : S(1);
		return { vec(v).x * s, vec(v).y * s, vec(v).z * s, static_cast<S>(cos_half_ang) };
	}

	// Evaluates 'ori' after 'time' for a constant angular velocity and angular acceleration
	template <QuaternionType Quat, VectorType Vec> requires (IsRank1<Vec> && SameS<Quat, Vec> && vector_traits<Vec>::dimension >= 3)
	inline Quat pr_vectorcall RotationAt(float time, Quat ori, Vec avel, Vec aacc) noexcept
	{
		using vt = vector_traits<Quat>;
		using S = typename vt::element_t;

		// Orientation can be computed analytically if angular velocity
		// and angular acceleration are parallel or angular acceleration is zero.
		if (LengthSq(Cross(avel, aacc)) < tiny<S>)
		{
			// Integrate w(t) = w0 + a*t exactly while the rotation axis is fixed.
			// Quaternion exp-map expects half-angle magnitude, so halve the angular displacement.
			auto const dt = static_cast<S>(time);
			auto const ang_disp = (avel + S(0.5) * aacc * dt) * dt;
			return ExpMap<Quat>(S(0.5) * ang_disp) * ori;
		}
		else
		{
			// Otherwise, use the SPIRAL(6) algorithm. 6th order accurate for moderate 'time_s'

			// 3-point Gauss-Legendre nodes for 6th order accuracy
			constexpr S root15f = S(3.87298334620741688518);
			constexpr S c1 = S(0.5) - root15f / S(10);
			constexpr S c2 = S(0.5);
			constexpr S c3 = S(0.5) + root15f / S(10);

			// Evaluate instantaneous angular velocity at nodes
			auto w0 = avel + aacc * c1 * time;
			auto w1 = avel + aacc * c2 * time;
			auto w2 = avel + aacc * c3 * time;

			auto u0 = ExpMap<Quat>(S(0.5) * w0 * time / S(3));
			auto u1 = ExpMap<Quat>(S(0.5) * w1 * time / S(3));
			auto u2 = ExpMap<Quat>(S(0.5) * w2 * time / S(3));

			return u2 * u1 * u0 * ori; // Compose the three Gauss-Legendre sub-steps in time order.
		}
	}

	// Create a quaternion from a rotation matrix
	template <QuaternionType Quat, VectorType Mat> requires (IsRank2<Mat> && SameS<Quat, Mat> && vector_traits<Mat>::dimension >= 3)
	constexpr Quat pr_vectorcall ToQuat(Mat const& mat) noexcept
	{
		using qt = vector_traits<Quat>;
		using mt = vector_traits<Mat>;
		using S = typename qt::element_t;
		pr_assert(IsOrthonormal(mat) && "Only orientation matrices can be converted into quaternions");

		S xx = vec(vec(mat).x).x, xy = vec(vec(mat).x).y, xz = vec(vec(mat).x).z;
		S yx = vec(vec(mat).y).x, yy = vec(vec(mat).y).y, yz = vec(vec(mat).y).z;
		S zx = vec(vec(mat).z).x, zy = vec(vec(mat).z).y, zz = vec(vec(mat).z).z;

		if (xx + yy + zz >= 0)
		{
			auto s = S(0.5) / Sqrt(S(1) + xx + yy + zz);
			return Quat{ (yz - zy) * s, (zx - xz) * s, (xy - yx) * s, (0.25f / s) };
		}
		if (xx > yy && xx > zz)
		{
			auto s = S(0.5) / Sqrt(S(1) + xx - yy - zz);
			return Quat{ (0.25f / s), (xy + yx) * s, (zx + xz) * s, (yz - zy) * s };
		}
		if (yy > zz)
		{
			auto s = S(0.5) / Sqrt(S(1) - xx + yy - zz);
			return Quat{ (xy + yx) * s, (0.25f / s), (yz + zy) * s, (zx - xz) * s };
		}
		{
			auto s = S(0.5) / Sqrt(S(1) - xx - yy + zz);
			return Quat{ (zx + xz) * s, (yz + zy) * s, (0.25f / s), (xy - yx) * s };
		}
	}

	// Create a rotation matrix from a quaternion
	template <VectorType Mat, QuaternionType Quat> requires (IsRank2<Mat> && SameS<Quat, Mat> && vector_traits<Mat>::dimension >= 3)
	constexpr Mat pr_vectorcall ToMatrix(Quat q) noexcept
	{
		using qt = vector_traits<Quat>;
		using mt = vector_traits<Mat>;
		using Vec = typename mt::component_t;
		using S = typename qt::element_t;

		// Stay on the proxy/view access path so the helper works with quaternions that only expose traits.
		auto const qv = vec(q);
		auto const norm_sq = qv.x * qv.x + qv.y * qv.y + qv.z * qv.z + qv.w * qv.w;
		pr_assert(norm_sq != S(0) && "'quat' is a zero quaternion");

		auto const s = S(2) / norm_sq;
		S xs = qv.x * s, ys = qv.y * s, zs = qv.z * s;
		S wx = qv.w * xs, wy = qv.w * ys, wz = qv.w * zs;
		S xx = qv.x * xs, xy = qv.x * ys, xz = qv.x * zs;
		S yy = qv.y * ys, yz = qv.y * zs, zz = qv.z * zs;
	
		Mat m = {};
		vec(m).x = Vec{S(1) - (yy + zz), xy + wz, xz - wy};
		vec(m).y = Vec{xy - wz, S(1) - (xx + zz), yz + wx};
		vec(m).z = Vec{xz + wy, yz - wx, S(1) - (xx + yy)};
		if constexpr (mt::dimension == 4)
			vec(m).w = Vec{0, 0, 0, S(1)};

		return m;
	}

	// Spherically interpolate between two rotations (using quat slerp)
	template <VectorTypeFP Mat> requires (IsRank2<Mat> && vector_traits<Mat>::dimension >= 3)
	inline Mat pr_vectorcall Slerp(Mat const& lhs, Mat const& rhs, typename vector_traits<Mat>::element_t frac) noexcept
	{
		using vt = vector_traits<Mat>;
		using S = typename vt::element_t;

		if (frac == S(0)) return lhs;
		if (frac == S(1)) return rhs;
		auto l = ToQuat<Quat<S>>(lhs);
		auto r = ToQuat<Quat<S>>(rhs);
		auto q = Slerp(l, r, frac);
		
		if constexpr (vt::dimension == 3)
		{
			return ToMatrix<Mat>(q);
		}
		if constexpr (vt::dimension == 4)
		{
			auto p = vec(lhs).w + frac * (vec(rhs).w - vec(lhs).w);
			return Mat{ ToMatrix<Mat3x3<S>>(q), p };
		}
	}

	// Deferred definition of Quat(Mat3x3) constructor
	template <ScalarTypeFP S>
	Quat<S>::Quat(Mat3x3<S> const& m) noexcept
		: Quat(ToQuat<Quat<S>>(m))
	{}
}








	#if 0

	namespace maths
	{
		// Specialise 'Avr' for quaternions
		// Finds the average rotation.
		template <ScalarType S, typename A, typename B> 
		struct Avr<Quat<S,A,B>, S>
		{
			Avr<Vec4<S, void>, S> m_avr;

			int Count() const noexcept
			{
				return m_avr.Count();
			}
			void Reset() noexcept
			{
				m_avr.Reset();
			}
			Quat<S,A,B> Mean() const noexcept
			{
				return Normalise(Quat<S,A,B>{m_avr.Mean()});
			}
			void Add(Quat<S,A,B> q) noexcept
			{
				// Nicked from Unity3D
				// Note: this only really works if all the quaternions are relatively close together.
				// For two quaternions, prefer 'Slerp'
				// This method is based on a simplified procedure described in this document:
				// http://ntrs.nasa.gov/archive/nasa/casi.ntrs.nasa.gov/20070017872_2007014421.pdf

				// Ensure the quaternions are in the same hemisphere (since q == -q)
				m_avr.Add(Dot4(q.xyzw, Mean().xyzw) >= 0 ? q.xyzw : -q.xyzw);
			}
		};
	}

	#endif
