//*****************************************************************************
// Maths library
//  Copyright (c) Rylogic Ltd 2002
//*****************************************************************************
#pragma once
#include "pr/math_new/core/forward.h"
#include "pr/math_new/core/traits.h"
#include "pr/math_new/core/constants.h"
#include "pr/math_new/types/vector4.h"
#include "pr/math_new/types/vector3.h"
#include "pr/math_new/types/vector2.h"

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
		constexpr explicit Quat(S x_) noexcept
			: x(x_)
			, y(x_)
			, z(x_)
			, w(x_)
		{
		}
		constexpr Quat(S x_, S y_, S z_, S w_) noexcept
			: x(x_)
			, y(y_)
			, z(z_)
			, w(w_)
		{}
		constexpr explicit Quat(std::ranges::random_access_range auto&& v) noexcept
			:Quat(v[0], v[1], v[2], v[3])
		{}
		constexpr explicit Quat(VectorTypeN<S, 4> auto v) noexcept
			:Quat(vec(v).x, vec(v).y, vec(v).z, vec(v).w)
		{}
		constexpr Quat(intrinsic_t vec_) noexcept requires (!NoIntrinsic)
			:vec(vec_)
		{}

		// Create a quaternion from a rotation matrix
		explicit Quat(Mat3x4<S> const& m) noexcept;

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

		// Get the axis component of the quaternion (normalised)
		Vec4<S> Axis() const noexcept
		{
			// The axis is arbitrary for identity rotations
			return Normalise(xyzw.w0(), Vec4<S>{0, 0, 1, 0});
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

			// The sign is determined by the sign of w (which represents cos(θ/2))
			auto sin_half_angle = Length(xyz);
			return S(2) * sin_half_angle * w;
		}
	};

	#define PR_MATH_DEFINE_TYPE(element)\
	template <> struct vector_traits<Quat<element>>\
		: quaternion_traits_base<element>\
		, vector_access_member<Quat<element>, element, 4>\
	{};\
	\
	static_assert(QuaternionType<Quat<element>>, "Quat<"#element"> is not a valid quaternion type");\
	static_assert(sizeof(Quat<element>) == 4*sizeof(element), "Quat<"#element"> has the wrong size");\
	static_assert(std::is_trivially_copyable_v<Quat<element>>, "Quat<"#element"> is not trivially copyable");

	PR_MATH_DEFINE_TYPE(float);
	PR_MATH_DEFINE_TYPE(double);
	#undef PR_MATH_DEFINE_TYPE

	// Normalise a quaternion to unit length
	template <QuaternionType Quat>
	inline Quat pr_vectorcall Normalise(Quat q) noexcept
	{
		using S = typename vector_traits<Quat>::element_t;
		auto len = Length(q.xyzw);
		return Quat{ q.x / len, q.y / len, q.z / len, q.w / len };
	}
	template <QuaternionType Quat>
	inline Quat pr_vectorcall Normalise(Quat q, Quat value_if_zero_length) noexcept
	{
		using S = typename vector_traits<Quat>::element_t;
		auto len = Length(q.xyzw);
		return len > tiny<S> ? Quat{ q.x / len, q.y / len, q.z / len, q.w / len } : value_if_zero_length;
	}

	// Decompose a quaternion into axis (normalised) and angle (radians)
	template <QuaternionType Quat>
	inline auto pr_vectorcall AxisAngle(Quat q) noexcept
	{
		using S = typename vector_traits<Quat>::element_t;
		struct R { Vec4<S> axis; S angle; };
		pr_assert(IsNormalised(q.xyzw) && "quaternion isn't normalised");

		// Use atan2 for the angle — well-conditioned everywhere, unlike acos
		auto sin_half_angle = Length(q.xyz);
		auto angle = S(2) * std::atan2(sin_half_angle, Abs(q.w));

		// Normalise the xyz part directly (avoids sqrt(1-w²) cancellation)
		auto axis = sin_half_angle > tiny<S>
			? Vec4<S>(q.x / sin_half_angle, q.y / sin_half_angle, q.z / sin_half_angle, S(0))
			: Vec4<S>(S(0), S(0), S(1), S(0)); // arbitrary axis for identity

		return R{ axis, angle };
	}

	// Test two quaternions for equivalence (i.e. do they represent the same orientation)
	template <QuaternionType Quat>
	inline bool FEqlOrientation(Quat lhs, Quat rhs, typename vector_traits<Quat>::element_t tol = tiny<typename vector_traits<Quat>::element_t>) noexcept
	{
		using S = typename vector_traits<Quat>::element_t;
		return FEqlAbsolute(AxisAngle(rhs * ~lhs).angle, S(0), tol);
	}

	// Quaternion FEql. Note: q == -q
	template <QuaternionType Quat>
	constexpr bool pr_vectorcall FEql(Quat lhs, Quat rhs) noexcept
	{
		using vt = vector_traits<Quat>;
		using S = typename vt::element_t;

		return
			FEqlRelative(lhs.xyzw, +rhs.xyzw, tiny<S>) ||
			FEqlRelative(lhs.xyzw, -rhs.xyzw, tiny<S>);
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

	// Scale the rotation 'q' by 'frac'. Returns a rotation about the same axis but with angle scaled by 'frac'
	template <QuaternionType Quat>
	inline Quat pr_vectorcall Scale(Quat q, typename vector_traits<Quat>::element_t frac) noexcept
	{
		using S = typename vector_traits<Quat>::element_t;
		pr_assert("quaternion isn't normalised" && IsNormalised(q.xyzw));

		// Use atan2 for the half-angle — well-conditioned everywhere, unlike acos
		auto sin_half_angle = Length(q.xyz);
		auto half_angle = std::atan2(sin_half_angle, Abs(q.w));
		auto scaled_half_angle = frac * half_angle;
		auto sin_ha = std::sin(scaled_half_angle);
		auto cos_ha = std::cos(scaled_half_angle);

		// Normalise the xyz part directly (avoids sqrt(1-w²) cancellation near identity)
		if (sin_half_angle > tiny<S>)
		{
			auto scale = sin_ha / sin_half_angle;
			return Quat{q.x * scale, q.y * scale, q.z * scale, cos_ha};
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

	// Logarithm map of quaternion to tangent space at identity
	template <VectorType Vec, QuaternionType Quat> requires (IsRank1<Vec> && SameS<Quat, Vec> && vector_traits<Vec>::dimension >= 3)
	inline Vec pr_vectorcall LogMap(Quat q) noexcept
	{
		using S = typename vector_traits<Quat>::element_t;
		auto xyz0 = Vec{ vec(q).x, vec(q).y, vec(q).z };

		// Quat = [u·sin(A/2), cos(A/2)]
		auto sin_half_ang = Length(xyz0);
		auto ang_by_2 = std::atan2(sin_half_ang, Abs(vec(q).w)); // well-conditioned everywhere

		// Scale xyz by (half_angle / sin_half_angle) to get axis * half_angle
		return sin_half_ang > tiny<S>
			? xyz0 * static_cast<S>(ang_by_2 / sin_half_ang)
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
			auto w = avel + aacc * time;
			return ExpMap(S(0.5) * w * time) * ori;
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

			auto u0 = ExpMap<Vec>(S(0.5) * w0 * time / S(3));
			auto u1 = ExpMap<Vec>(S(0.5) * w1 * time / S(3));
			auto u2 = ExpMap<Vec>(S(0.5) * w2 * time / S(3));

			return u2 * u1 * u0 * ori; // needs testing
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
	template <QuaternionType Quat, VectorType Mat> requires (IsRank2<Mat> && SameS<Quat, Mat> && vector_traits<Mat>::dimension >= 3)
	constexpr Mat pr_vectorcall ToMatrix(Quat q) noexcept
	{
		using qt = vector_traits<Quat>;
		using mt = vector_traits<Mat>;
		using Vec = typename mt::component_t;
		using S = typename qt::element_t;
		pr_assert(q != Quat{} && "'quat' is a zero quaternion");

		auto s = S(2) / LengthSq(q.xyzw);
		S xs = vec(q).x *  s, ys = vec(q).y *  s, zs = vec(q).z *  s;
		S wx = vec(q).w * xs, wy = vec(q).w * ys, wz = vec(q).w * zs;
		S xx = vec(q).x * xs, xy = vec(q).x * ys, xz = vec(q).x * zs;
		S yy = vec(q).y * ys, yz = vec(q).y * zs, zz = vec(q).z * zs;
	
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
		auto l = ToQuat<Quat<S>, Mat>(lhs);
		auto r = ToQuat<Quat<S>, Mat>(rhs);
		auto q = Slerp(l, r, frac);
		
		if constexpr (vt::dimension == 3)
		{
			return ToMatrix<Quat<S>, Mat>(q);
		}
		if constexpr (vt::dimension == 4)
		{
			auto p = vec(lhs).w + frac * (vec(rhs).w - vec(lhs).w);
			return Mat{ ToMatrix<Quat<S>, Mat3x4<S>>(q), p };
		}
	}

	// Deferred definition of Quat(Mat3x4) constructor
	template <ScalarTypeFP S>
	Quat<S>::Quat(Mat3x4<S> const& m) noexcept
		: Quat(ToQuat<Quat<S>, Mat3x4<S>>(m))
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
