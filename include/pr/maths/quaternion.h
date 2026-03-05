//*********************************************
// Maths Library
//  Copyright (c) Rylogic Ltd 2002
//*********************************************
#pragma once
#include "pr/maths/forward.h"
#include "pr/maths/constants.h"
#include "pr/maths/maths_core.h"
#include "pr/maths/vector2.h"
#include "pr/maths/vector3.h"
#include "pr/maths/vector4.h"
#include "pr/maths/stat.h"
//#include "pr/maths/matrix3x4.h"
//#include "pr/maths/matrix4x4.h" // make mat depend on quat

namespace pr
{
	template <Scalar S, typename A, typename B>
	struct Quat
	{
		static_assert(std::is_floating_point_v<S>);
		enum
		{
			IntrinsicF = PR_MATHS_USE_INTRINSICS && std::is_same_v<S, float>,
			IntrinsicD = PR_MATHS_USE_INTRINSICS && std::is_same_v<S, double>,
			NoIntrinsic = PR_MATHS_USE_INTRINSICS == 0,
		};
		#if PR_MATHS_USE_INTRINSICS
		using intrinsic_t =
			std::conditional_t<IntrinsicF, __m128,
			std::conditional_t<IntrinsicD, __m256d,
			void>>;
		#else
		using intrinsic_t = void;
		#endif

		#pragma warning(push)
		#pragma warning(disable:4201) // nameless struct
		union alignas(4 * sizeof(S))
		{
			struct { S x, y, z, w; };
			struct { Vec4<S, void> xyzw; };
			struct { Vec3<S, void> xyz; };
			struct { S arr[4]; };
			#if PR_MATHS_USE_INTRINSICS
			intrinsic_t vec;
			#endif
		};
		#pragma warning(pop)

		// Construct
		Quat() = default;
		constexpr Quat(S x_, S y_, S z_, S s_)
			:x(x_)
			,y(y_)
			,z(z_)
			,w(s_)
		{}
		constexpr explicit Quat(Vec4<S, void> vec)
			:xyzw(vec)
		{}
		constexpr explicit Quat(S const* v)
			:Quat(v[0], v[1], v[2], v[3])
		{}
		template <maths::Vector4 V> constexpr explicit Quat(V const& v)
			:Quat(maths::comp<0>(v), maths::comp<1>(v), maths::comp<2>(v), maths::comp<3>(v))
		{}
		#if PR_MATHS_USE_INTRINSICS
		Quat(intrinsic_t v)
			:vec(v)
		{
		}
		Quat& operator =(intrinsic_t v)
		{
			vec = v;
			return *this;
		}
		#endif

		// Reinterpret as a different quaternion type
		template <typename C, typename D> explicit operator Quat<S, C, D> const& () const
		{
			return reinterpret_cast<Quat<S, C, D> const&>(*this);
		}
		template <typename C, typename D> explicit operator Quat<S, C, D>& ()
		{
			return reinterpret_cast<Quat<S, C, D>&>(*this);
		}
		operator Quat<S, void, void> const& () const
		{
			return reinterpret_cast<Quat<S, void, void> const&>(*this);
		}
		operator Quat<S, void, void>& ()
		{
			return reinterpret_cast<Quat<S, void, void>&>(*this);
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

		// Create a quaternion from an axis and an angle
		Quat(Vec4<S, void> axis, S angle)
		{
			auto s = Sin(S(0.5) * angle);
			x = axis.x * s;
			y = axis.y * s;
			z = axis.z * s;
			w = Cos(S(0.5) * angle);
		}

		// Create a quaternion from Euler angles. Order is roll, pitch, yaw
		Quat(S pitch, S yaw, S roll)
		{
			S cos_p = Cos(pitch * S(0.5)), sin_p = Sin(pitch * S(0.5));
			S cos_y = Cos(yaw   * S(0.5)), sin_y = Sin(yaw   * S(0.5));
			S cos_r = Cos(roll  * S(0.5)), sin_r = Sin(roll  * S(0.5));
			x = sin_p * cos_y * cos_r + cos_p * sin_y * sin_r;
			y = cos_p * sin_y * cos_r - sin_p * cos_y * sin_r;
			z = cos_p * cos_y * sin_r - sin_p * sin_y * cos_r;
			w = cos_p * cos_y * cos_r + sin_p * sin_y * sin_r;
		}

		// Create a quaternion from a rotation matrix
		explicit Quat(Mat3x4<S,A,B> const& m);

		// Create a quaternion from a rotation matrix
		explicit Quat(Mat4x4<S,A,B> const& m)
			:Quat(m.rot)
		{}

		// Construct a quaternion from two vectors representing start and end orientations
		Quat(Vec4<S, void> from, Vec4<S, void> to)
		{
			pr_assert(from.w == 0 && to.w == 0);
			auto d = Dot(from, to);
			auto s = Sqrt(LengthSq(from) * LengthSq(to)) + d;
			auto axis = Cross(from, to);

			// Vectors are aligned, 180 degrees apart, or one is zero
			if (FEql(s, S(0)))
			{
				s = S(0);
				axis =
					LengthSq(from) > maths::tiny<float> ? Perpendicular(from) :
					LengthSq(to) > maths::tiny<float> ? Perpendicular(to) :
					v4::ZAxis();
			}

			xyzw = Normalise(Vec4<S, void>{axis.x, axis.y, axis.z, s});
		}

		// Get the axis component of the quaternion (normalised)
		Vec4<S, void> Axis() const
		{
			// The axis is arbitrary for identity rotations
			return Normalise(xyzw.w0(), Vec4<S, void>{0, 0, 1, 0});
		}

		// Return the angle of rotation about 'Axis()'
		S Angle() const
		{
			return Acos(CosAngle());
		}

		// Return the cosine of the angle of rotation about 'Axis()'
		S CosAngle() const
		{
			//pr_assert("quaternion isn't normalised" && IsNormal(*this));

			// Trig:
			//' w == cos(θ/2)
			//' cos²(θ/2) = 0.5 * (1 + cos(θ))
			//' w² == cos²(θ/2) == 0.5 * (1 + cos(θ))
			//' cos(θ) = 2w² - 1

			// This is always the smallest arc
			return Clamp(S(2) * Sqr(w) - LengthSq(xyzw), -S(1), +S(1));
		}

		// Return the sine of the angle of rotation about 'Axis()'
		S SinAngle() const
		{
			// Trig:
			//'  w == cos(θ/2)
			//'  sin(θ) = 2 * sin(θ/2) * cos(θ/2)

			// The sign is determined by the sign of w (which represents cos(θ/2))
			auto sin_half_angle = Length(xyz);
			return S(2) * sin_half_angle * w;
		}
		
		// Basic constants
		static constexpr Quat Zero()
		{
			return {0, 0, 0, 0};
		}
		static constexpr Quat Identity()
		{
			return {0, 0, 0, 1};
		}

		// Construct random
		template <typename Rng = std::default_random_engine> static Quat Random(Rng& rng, Vec4<S, void> axis, S min_angle, S max_angle)
		{
			// Create a random quaternion rotation
			std::uniform_real_distribution<S> dist(min_angle, max_angle);
			return Quat(axis, dist(rng));
		}
		template <typename Rng = std::default_random_engine> static Quat Random(Rng& rng, S min_angle, S max_angle)
		{
			// Create a random quaternion rotation
			std::uniform_real_distribution<S> dist(min_angle, max_angle);
			return Quat(Vec4<S,void>::RandomN(rng, 0), dist(rng));
		}
		template <typename Rng = std::default_random_engine> static Quat Random(Rng& rng)
		{
			// Create a random quaternion rotation
			std::uniform_real_distribution<S> dist(S(0), constants<S>::tau);
			return Quat(Vec4<S,void>::RandomN(rng, 0), dist(rng));
		}

		#pragma region Operators
		friend constexpr Quat pr_vectorcall operator +(Quat<S,A,B> q)
		{
			return q;
		}
		friend constexpr Quat  pr_vectorcall operator -(Quat<S,A,B> q)
		{
			return { -q.x, -q.y, -q.z, -q.w }; // Note: Not conjugate
		}
		friend Quat pr_vectorcall operator ~(Quat<S,A,B> q)
		{
			return { -q.x, -q.y, -q.z, q.w }; // This is conjugate
		}
		friend Quat pr_vectorcall operator + (Quat<S,A,B> lhs, Quat<S,A,B> rhs)
		{
			return Quat{ .xyzw = lhs.xyzw + rhs.xyzw };
		}
		friend Quat pr_vectorcall operator - (Quat<S,A,B> lhs, Quat<S,A,B> rhs)
		{
			return Quat{ .xyzw = lhs.xyzw - rhs.xyzw };
		}
		friend Quat pr_vectorcall operator *(Quat<S,A,B> lhs, float rhs)
		{
			return Quat{ .xyzw = lhs.xyzw * rhs };
		}
		friend Quat pr_vectorcall operator *(float lhs, Quat<S,A,B> rhs)
		{
			return Quat{ .xyzw = lhs * rhs.xyzw };
		}
		friend Quat pr_vectorcall operator /(Quat<S,A,B> lhs, float rhs)
		{
			return Quat{ .xyzw = lhs.xyzw / rhs };
		}

		// Quaternion multiply
		// Note about 'quat multiply' vs. 'r = q*v*conj(q)':
		// To rotate a vector or another quaternion, use the "sandwich product"
		// However, combining rotations is done using q1 * q2.
		// This is because:
		//'  r1 = a * v * conj(a)  - first rotation 
		//'  r2 = b * r1 * conj(b) - second rotation
		//'  r2 = b * a * v * conj(a) * conj(b)     
		//'  r2 = (b*a) * v * conj(b*a)             
		template <typename C> friend Quat<S,A,C> pr_vectorcall operator * (Quat<S,B,C> lhs, Quat<S,A,B> rhs)
		{
			auto q = Quat<S, A, C>{};
			q.x = lhs.w*rhs.x + lhs.x*rhs.w + lhs.y*rhs.z - lhs.z*rhs.y;
			q.y = lhs.w*rhs.y - lhs.x*rhs.z + lhs.y*rhs.w + lhs.z*rhs.x;
			q.z = lhs.w*rhs.z + lhs.x*rhs.y - lhs.y*rhs.x + lhs.z*rhs.w;
			q.w = lhs.w*rhs.w - lhs.x*rhs.x - lhs.y*rhs.y - lhs.z*rhs.z;
			return q;
		}

		// Quaternion rotate. i.e. 'r = q*v*conj(q)' the "sandwich product"
		// This is not really correct, since it's actually two multiplies
		// but it makes the code look nicer.
		friend Vec4<S, B> pr_vectorcall operator * (Quat<S,A,B> lhs, Vec4<S, A> rhs)
		{
			return Rotate(lhs, rhs);
		}

		#pragma endregion
	};
	#define PR_QUAT_CHECKS(scalar)\
	static_assert(sizeof(Quat<scalar, void, void>) == 4 * sizeof(scalar), "Quat<"#scalar"> has the wrong size");\
	static_assert(maths::Vector4<Quat<scalar, void, void>>, "Quat<"#scalar"> in not a Vector4");\
	static_assert(std::is_trivially_copyable_v<Quat<scalar, void, void>>, "Must be a pod type");\
	static_assert(std::alignment_of_v<Quat<scalar, void, void>> == 4 * sizeof(scalar), "Quat<"#scalar"> is not aligned correctly");
	PR_QUAT_CHECKS(float);
	PR_QUAT_CHECKS(double);
	#undef PR_QUAT_CHECKS

	// Quaternion FEql. Note: q == -q
	template <Scalar S, typename A, typename B> inline bool pr_vectorcall FEqlRelative(Quat<S,A,B> lhs, Quat<S,A,B> rhs, S tol)
	{
		return
			FEqlRelative(lhs.xyzw, +rhs.xyzw, tol) ||
			FEqlRelative(lhs.xyzw, -rhs.xyzw, tol);
	}
	template <Scalar S, typename A, typename B> inline bool pr_vectorcall FEql(Quat<S,A,B> lhs, Quat<S,A,B> rhs)
	{
		return FEqlRelative(lhs, rhs, maths::tiny<S>);
	}

	// Component add
	template <Scalar S, typename A, typename B> inline Quat<S,A,B> pr_vectorcall CompAdd(Quat<S,A,B> lhs, Quat<S,A,B> rhs)
	{
		return Quat<S,A,B>{lhs.xyzw + rhs.xyzw};
	}

	// Component multiply
	template <Scalar S, typename A, typename B> inline Quat<S,A,B> pr_vectorcall CompMul(Quat<S,A,B> lhs, S rhs)
	{
		return Quat<S,A,B>{lhs.xyzw * rhs};
	}
	template <Scalar S, typename A, typename B> inline Quat<S,A,B> pr_vectorcall CompMul(Quat<S,A,B> lhs, Quat<S,A,B> rhs)
	{
		return Quat<S,A,B>{lhs.xyzw * rhs.xyzw};
	}

	// Length squared
	template <Scalar S, typename A, typename B> inline S pr_vectorcall LengthSq(Quat<S,A,B> q)
	{
		return LengthSq(q.xyzw);
	}

	// Normalise the quaternion 'q'
	template <Scalar S, typename A, typename B> inline Quat<S,A,B> pr_vectorcall Normalise(Quat<S,A,B> q)
	{
		return Quat<S,A,B>{Normalise(q.xyzw)};
	}
	template <Scalar S, typename A, typename B> inline Quat<S,A,B> pr_vectorcall Normalise(Quat<S,A,B> q, Quat<S,A,B> def)
	{
		return Quat<S,A,B>{Normalise(q.xyzw, def.xyzw)};
	}

	// Returns the value of 'cos(theta / 2)', where 'theta' is the angle between 'a' and 'b'
	template <Scalar S, typename A, typename B> inline S pr_vectorcall CosHalfAngle(Quat<S,A,B> a, Quat<S,A,B> b)
	{
		// The relative orientation between 'a' and 'b' is given by z = 'a * conj(b)'
		// where operator * is a quaternion multiply. The 'w' component of a quaternion
		// multiply is given by: q.w = a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z;
		// which is the same as q.w = Dot4(a,b) since conjugate negates the x,y,z
		// components of 'b'. Remember: q.w = Cos(theta/2)
		return Dot4(a.xyzw, b.xyzw);
	}

	// Returns the smallest angle between two quaternions (in radians, [0, tau/2])
	template <Scalar S, typename A, typename B> inline S pr_vectorcall Angle(Quat<S,A,B> a, Quat<S,A,B> b)
	{
		// q.w = Cos(theta/2)
		// Note: cos(A) = 2 * cos²(A/2) - 1
		//  and: acos(A) = 0.5 * acos(2A² - 1), for A in [0, tau/2]
		// Using the 'acos(2A² - 1)' form always returns the smallest angle
		auto cos_half_ang = CosHalfAngle(a, b);
		return 
			cos_half_ang > 1.0f - maths::tiny<float> ? S(0) :
			cos_half_ang > 0 ? S(2) * Acos(Clamp(cos_half_ang, -S(1), +S(1))) : // better precision
			Acos(Clamp(S(2) * Sqr(cos_half_ang) - S(1), -S(1), +S(1)));
	}

	// Logarithm map of quaternion to tangent space at identity. Converts a quaternion into a length-scaled direction, where length is the angle of rotation
	template <Scalar S, typename A, typename B> inline Vec4<S, void> pr_vectorcall LogMap(Quat<S,A,B> q)
	{
		// Quat = [u.Sin(A/2), Cos(A/2)]
		auto cos_half_ang = Clamp<S>(q.w, -S(1), +S(1)); // [0, tau]
		auto sin_half_ang = Length(q.xyz); // Don't use 'sqrt(1 - w*w)', it's not float noise accurate enough when w ~= +/-1
		auto ang_by_2 = Acos(cos_half_ang); // By convention, log space uses Length = A/2
		return Abs(sin_half_ang) > maths::tiny<double>
			? q.xyzw.w0() * static_cast<S>(ang_by_2 / sin_half_ang)
			: q.xyzw.w0();
	}

	// Exponential map of tangent space at identity to quaternion. Converts a length-scaled direction to a quaternion.
	template <Scalar S> inline Quat<S, void, void> pr_vectorcall ExpMap(Vec4<S,void> v)
	{
		// Vec = (+/-)A * (-/+)u.
		auto ang_by_2 = Length(v); // By convention, log space uses Length = A/2
		auto cos_half_ang = Cos(ang_by_2);
		auto sin_half_ang = Sin(ang_by_2); // != Sqrt(1 - cos_half_ang * cos_half_ang) when ang_by_2 > tau/2
		auto s = ang_by_2 > maths::tiny<double> ? static_cast<S>(sin_half_ang / ang_by_2) : S(1);
		return { v.x * s, v.y * s, v.z * s, static_cast<S>(cos_half_ang) };
	}

	// Scale the rotation 'q' by 'frac'. Returns a rotation about the same axis but with angle scaled by 'frac'
	template <Scalar S, typename A, typename B> inline Quat<S,A,B> pr_vectorcall Scale(Quat<S,A,B> q, S frac)
	{
		pr_assert("quaternion isn't normalised" && IsNormal4(q));

		// Trig:
		//' sin^2(x) + cos^2(x) == 1
		//' s == sqrt(1 - w^2) == sqrt(1 - cos^2(x/2))
		//' s^2 == 1 - cos^2(x/2) == sin^2(x/2)
		//' s == sin(x/2)
		auto w = Clamp(q.w, S(-1.0), S(+1.0)); // = cos(x/2)
		auto s = Sqrt(S(+1.0) - Sqr(w));       // = sin(x/2)
		auto a = frac * Acos(w);               // = scaled half angle
		auto sin_ha = Sin(a);
		auto cos_ha = Cos(a);
		
		// Handle identity quaternion (s near zero) to avoid division by zero
		if (s < S(0.0001))
			return Quat<S,A,B>{S(0), S(0), S(0), cos_ha};
		
		return Quat<S,A,B>{
			q.x * sin_ha / s,
			q.y * sin_ha / s,
			q.z * sin_ha / s,
			cos_ha};
	}

	// Return the axis and angle from a quaternion
	template <Scalar S, typename A, typename B> inline std::tuple<Vec4<S,void>, S> pr_vectorcall AxisAngle(Quat<S,A,B> q)
	{
		pr_assert("quaternion isn't normalised" && IsNormal(q));

		// Trig:
		//' sin^2(x) + cos^2(x) == 1
		//' s == sqrt(1 - w^2) == sqrt(1 - cos^2(x/2))
		//' s^2 == 1 - cos^2(x/2) == sin^2(x/2)
		//' s == sin(x/2)
		auto w = Clamp(q.w, S(-1.0), S(+1.0));
		auto s = Sqrt(S(+1.0) - Sqr(w));
		auto angle = S(2.0) * Acos(w);
		auto axis = Abs(s) > maths::tiny<float>
			? Vec4<S,void>(q.x/s, q.y/s, q.z/s, S(0))
			: Vec4<S,void>{S(0), S(0), S(0), S(0)}; // axis is (0,0,0) when angle == 1
		return { axis, angle };
	}

	// Return possible Euler angles for the quaternion 'q'
	template <Scalar S, typename A, typename B> inline Vec4<S, void> EulerAngles(Quat<S,A,B> q)
	{
		// From Wikipedia
		double q0 = q.w, q1 = q.x, q2 = q.y, q3 = q.z;
		return Vec4<S,void>{
			s_cast<S>(atan2(2.0 * (q0*q1 + q2*q3), 1.0 - 2.0 * (q1*q1 + q2*q2))),
			s_cast<S>(asin (2.0 * (q0*q2 - q3*q1))),
			s_cast<S>(atan2(2.0 * (q0*q3 + q1*q2), 1.0 - 2.0 * (q2*q2 + q3*q3))),
			S(0)};
	}

	// Spherically interpolate between quaternions
	template <Scalar S, typename A, typename B> inline Quat<S,A,B> pr_vectorcall Slerp(Quat<S,A,B> a, Quat<S,A,B> b, S frac)
	{
		if (frac == S(0)) return a;
		if (frac == S(1)) return b;

		// Flip 'b' so that both quaternions are in the same hemisphere (since: q == -q)
		auto cos_angle = CosAngle(a,b);
		Quat<S,A,B> b_ = cos_angle >= 0 ? b : -b;
		cos_angle = Abs(cos_angle);

		// Calculate coefficients
		if (cos_angle < S(0.95))
		{
			auto angle     = Acos(cos_angle);
			auto scale0    = Sin((S(1) - frac) * angle);
			auto scale1    = Sin((frac       ) * angle);
			auto sin_angle = Sin(angle);
			return Quat<S,A,B>{(scale0*a.xyzw + scale1*b_.xyzw) / sin_angle};
		}
		else // "a" and "b" quaternions are very close
		{
			return Normalise(Quat<S,A,B>{Lerp(a.xyzw, b_.xyzw, frac)});
		}
	}

	// Rotate a vector by a quaternion
	template <Scalar S, typename A, typename B> inline Vec4<S,B> pr_vectorcall Rotate(Quat<S,A,B> lhs, Vec4<S,A> rhs)
	{
		// This is an optimised version of: 'r = q*v*conj(q) for when v.w == 0'
		S xx = lhs.x*lhs.x, xy = lhs.x*lhs.y, xz = lhs.x*lhs.z, xw = lhs.x*lhs.w;
		S                   yy = lhs.y*lhs.y, yz = lhs.y*lhs.z, yw = lhs.y*lhs.w;
		S                                     zz = lhs.z*lhs.z, zw = lhs.z*lhs.w;
		S                                                       ww = lhs.w*lhs.w;

		Vec4<S,B> r;
		r.x =   ww*rhs.x + 2*yw*rhs.z - 2*zw*rhs.y +   xx*rhs.x + 2*xy*rhs.y + 2*xz*rhs.z -   zz*rhs.x - yy*rhs.x;
		r.y = 2*xy*rhs.x +   yy*rhs.y + 2*yz*rhs.z + 2*zw*rhs.x -   zz*rhs.y +   ww*rhs.y - 2*xw*rhs.z - xx*rhs.y;
		r.z = 2*xz*rhs.x + 2*yz*rhs.y +   zz*rhs.z - 2*yw*rhs.x -   yy*rhs.z + 2*xw*rhs.y -   xx*rhs.z + ww*rhs.z;
		r.w = rhs.w;
		return r;
	}

	// Evaluates 'ori' after 'time' for a constant angular velocity and angular acceleration
	template <Scalar S, typename A, typename B> inline Quat<S,A,B> pr_vectorcall RotationAt(float time, Quat<S,A,B> ori, Vec4<S,A> avel, Vec4<S,A> aacc)
	{
		// Orientation can be computed analytically if angular velocity
		// and angular acceleration are parallel or angular acceleration is zero.
		if (LengthSq(Cross(avel, aacc)) < maths::tiny<S>)
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

			auto u0 = ExpMap(S(0.5) * w0 * time / S(3));
			auto u1 = ExpMap(S(0.5) * w1 * time / S(3));
			auto u2 = ExpMap(S(0.5) * w2 * time / S(3));

			return u2 * u1 * u0 * ori; // needs testing
		}
	}

	namespace maths
	{
		// Specialise 'Avr' for quaternions
		// Finds the average rotation.
		template <Scalar S, typename A, typename B> 
		struct Avr<Quat<S,A,B>, S>
		{
			Avr<Vec4<S, void>, S> m_avr;

			int Count() const
			{
				return m_avr.Count();
			}
			void Reset()
			{
				m_avr.Reset();
			}
			Quat<S,A,B> Mean() const
			{
				return Normalise(Quat<S,A,B>{m_avr.Mean()});
			}
			void Add(Quat<S,A,B> q)
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
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
namespace pr::maths
{
	PRUnitTestClass(QuaternionTests)
	{
		std::default_random_engine rng;
		TestClass_QuaternionTests()
			:rng(1u)
		{
		}

		PRUnitTestMethod(CreateFromMat3x4, float, double)
		{
			using S = T;
			using quat_t = Quat<S, void, void>;

			std::uniform_real_distribution<float> rng_angle(-constants<float>::tau, +constants<float>::tau);
			for (int i = 0; i != 100; ++i)
			{
				auto ang = rng_angle(rng);
				auto axis = Vec4<float,void>::RandomN(rng, 0);
				auto mat = m3x4::Rotation(axis, ang);
				auto q = Quat<float, void, void>(mat);
				auto v0 = Vec4<float, void>::RandomN(rng, 0);
				auto r0 = mat * v0;
				auto r1 = Rotate(q, v0);
				PR_EXPECT(FEql(r0, r1));
			}
		}
		PRUnitTestMethod(Angle, float, double)
		{
			using S = T;
			using quat_t = Quat<S, void, void>;

			float const angles[] = {
				-constants<float>::tau_by_2,
				-maths::tau_by_3f,
				-constants<float>::tau_by_4,
				-maths::tau_by_5f,
				-maths::tau_by_6f,
				-maths::tau_by_7f,
				0.0f,
				+maths::tau_by_7f,
				+maths::tau_by_6f,
				+maths::tau_by_5f,
				+constants<float>::tau_by_4,
				+maths::tau_by_3f,
				+constants<float>::tau_by_2,
			};

			auto axis = v4::Normal(1, 1, 1, 0);
			for (auto ANG0 : angles)
			{
				for (auto ANG1 : angles)
				{
					auto q0 = quat{ axis, ANG0 };
					auto q1 = quat{ axis, ANG1 };
					auto expected = Min(Abs(ANG1 - ANG0), Abs(constants<float>::tau - Abs(ANG1 - ANG0)));

					auto ang0 = Angle(q0, q1);
					auto ang1 = Angle(q1, q0);
					constexpr float angular_tolerance = 1e-3f;
					PR_EXPECT(FEqlAbsolute(ang0, expected, angular_tolerance));
					PR_EXPECT(FEqlAbsolute(ang1, expected, angular_tolerance));
				}
			}
		}
		PRUnitTestMethod(Average, float, double)
		{
			using S = T;
			using quat_t = Quat<S, void, void>;

			auto ideal_mean = quat_t(Normalise(Vec4<S,void>(1, 1, 1, 0)), S(0.5));

			std::uniform_int_distribution<int> rng_bool(0, 1);
			std::uniform_real_distribution<S> rng_angle(ideal_mean.Angle() - S(0.2), ideal_mean.Angle() + S(0.2));

			Avr<quat_t, S> avr;
			for (int i = 0; i != 1000; ++i)
			{
				auto axis = Normalise(ideal_mean.Axis() + Vec4<S, void>::Random(rng, S(0.0), S(0.2), S(0)));
				auto angle = rng_angle(rng);
				quat_t q(axis, angle);
				avr.Add(rng_bool(rng) ? q : -q);
			}

			auto actual_mean = avr.Mean();
			PR_EXPECT(FEqlRelative(ideal_mean, actual_mean, S(0.01)));
		}
		PRUnitTestMethod(LogMapExpMap, float, double)
		{
			using S = T;
			using quat_t = Quat<S, void, void>;

			// Degenerate cases
			{
				auto q0 = quat::Random(rng);
				auto angular_error = Angle(q0, q0);
				PR_EXPECT(angular_error == 0.0f);
			}

			// Round trip test
			auto max_angular_error = 0.0f;
			for (auto i = 0; i != 100; ++i)
			{
				auto q0 = quat::Random(rng);
				auto v0 = LogMap(q0);
				auto q1 = ExpMap(v0);
				auto angular_error = Angle(q0, q1);
				
				max_angular_error = Max(max_angular_error, angular_error);
				PR_EXPECT(Abs(angular_error) < 0.001f);
			}
			PR_EXPECT(max_angular_error < 0.001f);

			// Edge cases
			{
				auto q0 = quat{ -2.09713704e-08f, -0.00148352725f, -6.48572168e-11f, -0.999998927f };
				q0 = Normalise(q0);
				auto v0 = LogMap(q0);
				auto q1 = ExpMap(v0);
				auto angular_error = Angle(q0, q1);
				PR_EXPECT(Abs(angular_error) < 0.001f);
			}
			{
				auto q0 = quat{0.546808f, -0.448332f, 0.448332f, -0.546808f};
				auto v0 = LogMap(q0);
				auto q1 = ExpMap(v0);
				auto angular_error = Angle(q0, q1);
				PR_EXPECT(Abs(angular_error) < 0.001f);
			}
		}
		PRUnitTestMethod(RotationAt, float, double)
		{
			using S = T;
			using quat_t = Quat<S, void, void>;

			{// Analytic solution case
				auto ori = quat::Random(rng);
				auto avl = v4{ 0.6f, 0, 0.6f, 0 };
				auto aac = v4{ 0, 0, 0, 0 };

				auto rot = m3x4{ ori };
				for (float t = 0; t < 5.0f; t += 0.1f)
				{
					auto ORI = RotationAt(t, ori, avl, aac);
					auto ROT = RotationAt(t, rot, avl, aac);
					auto ROT2 = m3x4{ ORI };
					PR_EXPECT(FEql(ROT, ROT2));
				}
			}
			{// Non-analytic solution case
				auto ori = quat::Random(rng);
				auto avl = v4{ 1.2f, 0, 0, 0 };
				auto aac = v4{ 0, 0, 0.1f, 0 };

				auto rot = m3x4{ ori };
				for (float t = 0; t < 5.0f; t += 0.1f)
				{
					auto ORI = RotationAt(t, ori, avl, aac);
					auto ROT = RotationAt(t, rot, avl, aac);
					auto ROT2 = m3x4{ ORI };
					PR_EXPECT(FEql(ROT, ROT2));
				}
			}
			{// Wrapping
				for (int i = 0; i != 5; ++i)
				{
					auto time = 1.0f * i;
					auto avl = 2.4f * v4::ZAxis(); // radians
					auto q = RotationAt(time, quat::Identity(), avl, v4::Zero());
				}
			}
		}
	};
}
#endif
