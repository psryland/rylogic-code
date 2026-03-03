//*****************************************************************************
// Maths library
//  Copyright (c) Rylogic Ltd 2002
//*****************************************************************************
#pragma once
#include "pr/maths/forward.h"
#include "pr/maths/constants.h"
#include "pr/maths/maths_core.h"
#include "pr/maths/vector2.h"
#include "pr/maths/vector3.h"

namespace pr
{
	template <Scalar S, typename T>
	struct Vec4
	{
		enum
		{
			IntrinsicF = PR_MATHS_USE_INTRINSICS && std::is_same_v<S, float>,
			IntrinsicD = PR_MATHS_USE_INTRINSICS && std::is_same_v<S, double>,
			IntrinsicI = PR_MATHS_USE_INTRINSICS && std::is_same_v<S, int32_t>,
			IntrinsicL = PR_MATHS_USE_INTRINSICS && std::is_same_v<S, int64_t>,
			NoIntrinsic = PR_MATHS_USE_INTRINSICS == 0,
		};
		using intrinsic_t =
			std::conditional_t<IntrinsicF, __m128,
			std::conditional_t<IntrinsicD, __m256d,
			std::conditional_t<IntrinsicI, __m128i,
			std::conditional_t<IntrinsicL, __m256i,
			std::byte[4*sizeof(S)]
			>>>>;

		#pragma warning(push)
		#pragma warning(disable:4201) // nameless struct
		union alignas(4 * sizeof(S))
		{
			struct { S x, y, z, w; };
			struct { Vec2<S, T> xy, zw; };
			struct { Vec3<S, T> xyz; };
			struct { S arr[4]; };
			intrinsic_t vec;
		};
		#pragma warning(pop)

		// Construct
		Vec4() = default;
		constexpr explicit Vec4(S x_)
			: x(x_)
			, y(x_)
			, z(x_)
			, w(x_)
		{
		}
		constexpr Vec4(S x_, S y_, S z_, S w_)
			: x(x_)
			, y(y_)
			, z(z_)
			, w(w_)
		{
		}
		constexpr Vec4(Vec3<S, void> xyz, S w_)
			: x(xyz.x)
			, y(xyz.y)
			, z(xyz.z)
			, w(w_)
		{
		}
		constexpr explicit Vec4(S const* v)
			:Vec4(v[0], v[1], v[2], v[3])
		{
		}
		constexpr explicit Vec4(S const* v, S w_)
			:Vec4(v[0], v[1], v[2], w_)
		{
		}
		template <maths::Vector4 V> constexpr explicit Vec4(V const& v)
			:Vec4(maths::comp<0>(v), maths::comp<1>(v), maths::comp<2>(v), maths::comp<3>(v))
		{
		}
		template <maths::Vector3 V> constexpr Vec4(V const& v, S w_)
			: Vec4(maths::comp<0>(v), maths::comp<1>(v), maths::comp<2>(v), w_)
		{
		}
		template <maths::Vector2 V> constexpr Vec4(V const& v, S z_, S w_)
			: Vec4(maths::comp<0>(v), maths::comp<1>(v), z_, w_)
		{
		}
		template <maths::Vector2 V> constexpr Vec4(V const& xy, V const& zw)
			: Vec4(maths::comp<0>(xy), maths::comp<1>(xy), maths::comp<0>(zw), maths::comp<1>(zw))
		{
		}
		#if PR_MATHS_USE_INTRINSICS
		Vec4(intrinsic_t v)
			: vec(v)
		{
			pr_assert(is_aligned<Vec4>(this));
		}
		Vec4& operator =(intrinsic_t v)
		{
			pr_assert(is_aligned<Vec4>(this));
			vec = v;
			return *this;
		}
		#endif

		// Reinterpret as a different vector type
		template <typename U> explicit operator Vec4<S, U> const& () const
		{
			return reinterpret_cast<Vec4<S, U> const&>(*this);
		}
		template <typename U> explicit operator Vec4<S, U>& ()
		{
			return reinterpret_cast<Vec4<S, U>&>(*this);
		}
		operator Vec4<S, void> const& () const
		{
			return reinterpret_cast<Vec4<S, void> const&>(*this);
		}
		operator Vec4<S, void>& ()
		{
			return reinterpret_cast<Vec4<S, void>&>(*this);
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
		Vec4 w0() const
		{
			Vec4 r(x, y, z, S(0)); // LValue because of alignment
			return r;
		}
		Vec4 w1() const
		{
			Vec4 r(x, y, z, S(1)); // LValue because of alignment
			return r;
		}
		Vec2<S, T> vec2(int i0, int i1) const
		{
			return Vec2<S, T>{arr[i0], arr[i1]};
		}
		Vec3<S, T> vec3(int i0, int i1, int i2) const
		{
			return Vec3<S, T>{arr[i0], arr[i1], arr[i2]};
		}

		// Basic constants
		static constexpr Vec4 Zero() { return Vec4(S(0), S(0), S(0), S(0)); }
		static constexpr Vec4 XAxis() { return Vec4(S(1), S(0), S(0), S(0)); }
		static constexpr Vec4 YAxis() { return Vec4(S(0), S(1), S(0), S(0)); }
		static constexpr Vec4 ZAxis() { return Vec4(S(0), S(0), S(1), S(0)); }
		static constexpr Vec4 WAxis() { return Vec4(S(0), S(0), S(0), S(1)); }
		static constexpr Vec4 Origin() { return Vec4(S(0), S(0), S(0), S(1)); }
		static constexpr Vec4 One() { return Vec4(S(1), S(1), S(1), S(1)); }
		static constexpr Vec4 TinyF() { return Vec4(maths::tiny<S>, maths::tiny<S>, maths::tiny<S>, maths::tiny<S>); }
		static constexpr Vec4 Min() { return Vec4(limits<S>::min(), limits<S>::min(), limits<S>::min(), limits<S>::min()); }
		static constexpr Vec4 Max() { return Vec4(limits<S>::max(), limits<S>::max(), limits<S>::max(), limits<S>::max()); }
		static constexpr Vec4 Lowest() { return Vec4(limits<S>::lowest(), limits<S>::lowest(), limits<S>::lowest(), limits<S>::lowest()); }
		static constexpr Vec4 Epsilon() { return Vec4(limits<S>::epsilon(), limits<S>::epsilon(), limits<S>::epsilon(), limits<S>::epsilon()); }
		static constexpr Vec4 Infinity() { return Vec4(limits<S>::infinity(), limits<S>::infinity(), limits<S>::infinity(), limits<S>::infinity()); }

		// Construct normalised
		static Vec4 Normal(S x, S y, S z, S w) requires std::is_floating_point_v<S>
		{
			return Normalise(Vec4(x, y, z, w));
		}

		// Create a 4-vector containing random values, normalised to unit length
		template <typename Rng = std::default_random_engine> requires std::is_floating_point_v<S>
		static Vec4 pr_vectorcall RandomN(Rng& rng)
		{
			std::uniform_real_distribution<S> dist(S(-1), S(1));
			for (;;)
			{
				auto x = dist(rng);
				auto y = dist(rng);
				auto z = dist(rng);
				auto w = dist(rng);
				auto v = Vec4(x, y, z, w);
				auto len = LengthSq(v);
				if (len > S(0.01) && len <= S(1))
					return v / Sqrt(len);
			}
		}

		// Create a vector containing a random normalised 3-vector and 'w_'
		template <typename Rng = std::default_random_engine> requires std::is_floating_point_v<S>
		static Vec4 pr_vectorcall RandomN(Rng& rng, S w_)
		{
			return Vec4(Vec3<S, T>::RandomN(rng), w_);
		}

		// Create a random 4-vector with components on interval '[vmin, vmax]'
		template <typename Rng = std::default_random_engine> requires std::is_floating_point_v<S>
		static Vec4 pr_vectorcall Random(Rng& rng, Vec4 vmin, Vec4 vmax)
		{
			std::uniform_real_distribution<S> dist_x(vmin.x, vmax.x);
			std::uniform_real_distribution<S> dist_y(vmin.y, vmax.y);
			std::uniform_real_distribution<S> dist_z(vmin.z, vmax.z);
			std::uniform_real_distribution<S> dist_w(vmin.w, vmax.w);
			return Vec4(dist_x(rng), dist_y(rng), dist_z(rng), dist_w(rng));
		}

		// Create a random vector with xyz components on interval '[vmin, vmax]' and 'w_'
		template <typename Rng = std::default_random_engine> requires std::is_floating_point_v<S>
		static Vec4 pr_vectorcall Random(Rng& rng, Vec4 vmin, Vec4 vmax, S w_)
		{
			return Vec4(Vec3<S, T>::Random(rng, vmin.xyz, vmax.xyz), w_);
		}

		// Create a random 4-vector with length on interval [min_length, max_length]
		template <typename Rng = std::default_random_engine> requires std::is_floating_point_v<S>
		static Vec4 pr_vectorcall Random(Rng& rng, S min_length, S max_length)
		{
			std::uniform_real_distribution<S> dist(min_length, max_length);
			return dist(rng) * RandomN(rng);
		}

		// Create a vector with xyz of random length on interval [min_length, max_length], and 'w_'
		template <typename Rng = std::default_random_engine> requires std::is_floating_point_v<S>
		static Vec4 pr_vectorcall Random(Rng& rng, S min_length, S max_length, S w_)
		{
			return Vec4(Vec3<S, T>::Random(rng, min_length, max_length), w_);
		}

		// Create a random 4-vector with components on the interval [centre - radius, centre + radius].
		template <typename Rng = std::default_random_engine> requires std::is_floating_point_v<S>
		static Vec4 pr_vectorcall Random(Rng& rng, Vec4 centre, S radius)
		{
			return Random(rng, 0, radius) + centre;
		}

		// Create a vector with xyz randomly within the sphere [centre,radius], and 'w_'
		template <typename Rng = std::default_random_engine> requires std::is_floating_point_v<S>
		static Vec4 pr_vectorcall Random(Rng& rng, Vec4 centre, S radius, S w_)
		{
			return Vec4(Vec3<S, T>::Random(rng, centre.xyz, radius), w_);
		}

		#pragma region Operators
		friend constexpr Vec4 pr_vectorcall operator + (Vec4 vec)
		{
			return vec;
		}
		friend constexpr Vec4 pr_vectorcall operator - (Vec4 vec)
		{
			//#if PR_MATHS_USE_INTRINSICS
			//return Vec4{_mm_sub_ps(_mm_setzero_ps(), vec.vec)};
			//#else
			return Vec4{-vec.x, -vec.y, -vec.z, -vec.w};
			//#endif
		}
		friend Vec4 pr_vectorcall operator * (S lhs, Vec4 rhs)
		{
			return rhs * lhs;
		}
		friend Vec4 pr_vectorcall operator * (Vec4 lhs, S rhs)
		{
			if constexpr (IntrinsicF)
			{
				return Vec4{_mm_mul_ps(lhs.vec, _mm_set_ps1(rhs))};
			}
			else if constexpr (IntrinsicD)
			{
				return Vec4{_mm256_mul_pd(lhs.vec, _mm256_set1_pd(rhs))};
			}
			else
			{
				return Vec4{lhs.x * rhs, lhs.y * rhs, lhs.z * rhs, lhs.w * rhs};
			}
		}
		friend Vec4 pr_vectorcall operator / (Vec4 lhs, S rhs)
		{
			// Don't check for divide by zero by default. For floats +inf/-inf are valid results
			if constexpr (IntrinsicF)
			{
				return Vec4{_mm_div_ps(lhs.vec, _mm_set_ps1(rhs))};
			}
			else if constexpr (IntrinsicD)
			{
				return Vec4{_mm256_div_pd(lhs.vec, _mm256_set1_pd(rhs))};
			}
			else
			{
				return Vec4{lhs.x / rhs, lhs.y / rhs, lhs.z / rhs, lhs.w / rhs};
			}
		}
		friend Vec4 pr_vectorcall operator % (Vec4 lhs, S rhs)
		{
			// Don't check for divide by zero by default. For floats +inf/-inf are valid results
			return Vec4{ Modulus(lhs.x, rhs), Modulus(lhs.y, rhs), Modulus(lhs.z, rhs), Modulus(lhs.w, rhs) };
		}
		friend Vec4 pr_vectorcall operator / (S lhs, Vec4 rhs)
		{
			if constexpr (IntrinsicF)
			{
				return Vec4{_mm_div_ps(_mm_set_ps1(lhs), rhs.vec)};
			}
			else if constexpr (IntrinsicD)
			{
				return Vec4{_mm256_div_pd(_mm256_set1_pd(lhs), rhs.vec)};
			}
			else
			{
				return Vec4{lhs / rhs.x, lhs / rhs.y, lhs / rhs.z, lhs / rhs.w};
			}
		}
		friend Vec4 pr_vectorcall operator % (S lhs, Vec4 rhs)
		{
			if constexpr (std::floating_point<S>)
			{
				return Vec4{Fmod(lhs, rhs.x), Fmod(lhs, rhs.y), Fmod(lhs, rhs.z), Fmod(lhs, rhs.w)};
			}
			else
			{
				return Vec4{lhs % rhs.x, lhs % rhs.y, lhs % rhs.z, lhs % rhs.w};
			}
		}
		friend Vec4 pr_vectorcall operator + (Vec4 lhs, Vec4 rhs)
		{
			if constexpr (IntrinsicF)
			{
				return Vec4{_mm_add_ps(lhs.vec, rhs.vec)};
			}
			else if constexpr (IntrinsicD)
			{
				return Vec4{_mm256_add_pd(lhs.vec, rhs.vec)};
			}
			else
			{
				return Vec4{lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z, lhs.w + rhs.w};
			}
		}
		friend Vec4 pr_vectorcall operator - (Vec4 lhs, Vec4 rhs)
		{
			if constexpr (IntrinsicF)
			{
				return Vec4{_mm_sub_ps(lhs.vec, rhs.vec)};
			}
			else if constexpr (IntrinsicD)
			{
				return Vec4{_mm256_sub_pd(lhs.vec, rhs.vec)};
			}
			else
			{
				return Vec4{lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z, lhs.w - rhs.w};
			}
		}
		friend Vec4 pr_vectorcall operator * (Vec4 lhs, Vec4 rhs)
		{
			if constexpr (IntrinsicF)
			{
				return Vec4{_mm_mul_ps(lhs.vec, rhs.vec)};
			}
			else if constexpr (IntrinsicD)
			{
				return Vec4{_mm256_mul_pd(lhs.vec, rhs.vec)};
			}
			else
			{
				return Vec4{lhs.x * rhs.x, lhs.y * rhs.y, lhs.z * rhs.z, lhs.w * rhs.w};
			}
		}
		friend Vec4 pr_vectorcall operator / (Vec4 lhs, Vec4 rhs)
		{
			// Don't check for divide by zero by default. For floats +inf/-inf are valid results
			if constexpr (IntrinsicF)
			{
				return Vec4{_mm_div_ps(lhs.vec, rhs.vec)};
			}
			else if constexpr (IntrinsicD)
			{
				return Vec4{_mm256_div_pd(lhs.vec, rhs.vec)};
			}
			else
			{
				return Vec4{lhs.x / rhs.x, lhs.y / rhs.y, lhs.z / rhs.z, lhs.w / rhs.w};
			}
		}
		friend Vec4 pr_vectorcall operator % (Vec4 lhs, Vec4 rhs)
		{
			// Don't check for divide by zero by default. For floats +inf/-inf are valid results
			if constexpr (IntrinsicF)
			{
				auto div = _mm_div_ps(lhs.vec, rhs.vec);                                // a / b
				auto trunc = _mm_round_ps(div, _MM_FROUND_TO_ZERO | _MM_FROUND_NO_EXC); // trunc(a / b)
				auto prod = _mm_mul_ps(trunc, rhs.vec);                                 // trunc(a / b) * b
				auto rem = _mm_sub_ps(lhs.vec, prod);                                   // a - b * trunc(a / b) => fmod
				return Vec4{ rem };
			}
			else if constexpr (IntrinsicD)
			{
				auto div = _mm256_div_ps(lhs.vec, rhs.vec);                                // a / b
				auto trunc = _mm256_round_ps(div, _MM_FROUND_TO_ZERO | _MM_FROUND_NO_EXC); // trunc(a / b)
				auto prod = _mm256_mul_ps(trunc, rhs.vec);                                 // trunc(a / b) * b
				auto rem = _mm256_sub_ps(lhs.vec, prod);                                   // a - b * trunc(a / b) => fmod
				return Vec4{ rem };
			}
			else
			{
				return Vec4{ Modulus(lhs.x, rhs.x), Modulus(lhs.y, rhs.y), Modulus(lhs.z, rhs.z), Modulus(lhs.w, rhs.w) };
			}
		}
		friend bool pr_vectorcall operator == (Vec4 lhs, Vec4 rhs)
		{
			if constexpr (IntrinsicF)
			{
				return _mm_movemask_ps(_mm_cmpeq_ps(lhs.vec, rhs.vec)) == 0xF;
			}
			else if constexpr (IntrinsicD)
			{
				return _mm256_movemask_pd(_mm256_cmp_pd(lhs.vec, rhs.vec, _CMP_EQ_OQ)) == 0xF;
			}
			else
			{
				return lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z && lhs.w == rhs.w;
			}
		}
		friend bool pr_vectorcall operator != (Vec4 lhs, Vec4 rhs)
		{
			return !(lhs == rhs);
		}
		friend Vec4 pr_vectorcall operator ~ (Vec4 rhs) requires std::integral<S>
		{
			return Vec4{~rhs.x, ~rhs.y, ~rhs.z, ~rhs.w};
		}
		friend Vec4 pr_vectorcall operator ! (Vec4 rhs) requires std::integral<S>
		{
			return Vec4{!rhs.x, !rhs.y, !rhs.z, !rhs.w};
		}
		friend Vec4 pr_vectorcall operator | (Vec4 lhs, Vec4 rhs) requires std::integral<S>
		{
			return Vec4{lhs.x | rhs.x, lhs.y | rhs.y, lhs.z | rhs.z, lhs.w | rhs.w};
		}
		friend Vec4 pr_vectorcall operator & (Vec4 lhs, Vec4 rhs) requires std::integral<S>
		{
			return Vec4{lhs.x & rhs.x, lhs.y & rhs.y, lhs.z & rhs.z, lhs.w & rhs.w};
		}
		friend Vec4 pr_vectorcall operator ^ (Vec4 lhs, Vec4 rhs) requires std::integral<S>
		{
			return Vec4{lhs.x ^ rhs.x, lhs.y ^ rhs.y, lhs.z ^ rhs.z, lhs.w ^ rhs.w};
		}
		friend Vec4 pr_vectorcall operator << (Vec4 lhs, int rhs) requires std::integral<S>
		{
			return Vec4{lhs.x << rhs, lhs.y << rhs, lhs.z << rhs, lhs.w << rhs};
		}
		friend Vec4 pr_vectorcall operator << (Vec4 lhs, Vec4 rhs) requires std::integral<S>
		{
			return Vec4{lhs.x << rhs.x, lhs.y << rhs.y, lhs.z << rhs.z, lhs.w << rhs.w};
		}
		friend Vec4 pr_vectorcall operator >> (Vec4 lhs, int rhs) requires std::integral<S>
		{
			return Vec4{lhs.x >> rhs, lhs.y >> rhs, lhs.z >> rhs, lhs.w >> rhs};
		}
		friend Vec4 pr_vectorcall operator >> (Vec4 lhs, Vec4 rhs) requires std::integral<S>
		{
			return Vec4{lhs.x >> rhs.x, lhs.y >> rhs.y, lhs.z >> rhs.z, lhs.w >> rhs.w};
		}
		friend Vec4 pr_vectorcall operator || (Vec4 lhs, Vec4 rhs) requires std::integral<S>
		{
			return Vec4{lhs.x || rhs.x, lhs.y || rhs.y, lhs.z || rhs.z, lhs.w || rhs.w};
		}
		friend Vec4 pr_vectorcall operator && (Vec4 lhs, Vec4 rhs) requires std::integral<S>
		{
			return Vec4{lhs.x && rhs.x, lhs.y && rhs.y, lhs.z && rhs.z, lhs.w && rhs.w};
		}
		#pragma endregion
	};
	#define PR_VEC4_CHECKS(scalar)\
	static_assert(sizeof(Vec4<scalar, void>) == 4 * sizeof(scalar), "Vec4<"#scalar"> has the wrong size");\
	static_assert(maths::Vector4<Vec4<scalar, void>>, "Vector<"#scalar" is not a Vector4");\
	static_assert(std::is_trivially_copyable_v<Vec4<scalar, void>>, "Must be a pod type");\
	static_assert(std::alignment_of_v<Vec4<scalar, void>> == 4 * sizeof(scalar), "Vec4<"#scalar"> is not aligned correctly");
	PR_VEC4_CHECKS(float);
	PR_VEC4_CHECKS(double);
	PR_VEC4_CHECKS(int32_t);
	PR_VEC4_CHECKS(int64_t);
	#undef PR_VEC4_CHECKS
	
	//template <int S> class C; C<alignof(Vec4<double,void>)> c;

	// Implementation from Vec3
	template <Scalar S, typename T> Vec4<S, T> Vec3<S, T>::w0() const
	{
		return Vec4<S, T>(x, y, z, S(0));
	}
	template <Scalar S, typename T> Vec4<S, T> Vec3<S, T>::w1() const
	{
		return Vec4<S, T>(x, y, z, S(1));
	}

	// V4 FEql
	template <Scalar S, typename T> inline bool pr_vectorcall FEqlAbsolute(Vec4<S, T> a, Vec4<S, T> b, S tol)
	{
		static_assert(std::floating_point<S>, "FEql is only defined for floating point vectors");

		// abs(a - b) < tol
		if constexpr (Vec4<S, T>::IntrinsicF)
		{
			auto d = _mm_sub_ps(a.vec, b.vec);                                        // d = a - b;
			auto abs_d = _mm_and_ps(d, _mm_castsi128_ps(_mm_set1_epi32(0x7fffffff))); // d = abs(a - b);
			auto r = _mm_cmplt_ps(abs_d, _mm_set_ps1(tol));                           // r = abs(d) < tol
			return (_mm_movemask_ps(r) & 0x0f) == 0x0f;
		}
		else
		{
			return
				FEqlAbsolute(a.x, b.x, tol) &&
				FEqlAbsolute(a.y, b.y, tol) &&
				FEqlAbsolute(a.z, b.z, tol) &&
				FEqlAbsolute(a.w, b.w, tol);
		}
	}
	template <Scalar S, typename T> inline bool pr_vectorcall FEqlRelative(Vec4<S, T> a, Vec4<S, T> b, S tol)
	{
		static_assert(std::floating_point<S>, "FEql is only defined for floating point vectors");

		// Handles tests against zero where relative error is meaningless
		// Tests with 'b == 0' are the most common so do them first
		if (b == Vec4<S, T>{}) return MaxComponentAbs(a) < tol;
		if (a == Vec4<S, T>{}) return MaxComponentAbs(b) < tol;

		// Handle infinities and exact values
		if (a == b) return true;

		auto abs_max_element = Max(MaxComponentAbs(a), MaxComponentAbs(b));
		return FEqlAbsolute(a, b, tol * abs_max_element);
	}
	template <Scalar S, typename T> inline bool pr_vectorcall FEql(Vec4<S, T> a, Vec4<S, T> b)
	{
		if constexpr (std::floating_point<S>)
			return FEqlRelative(a, b, maths::tiny<S>);
		else
			return a == b;
	}

	// Abs
	template <Scalar S, typename T> inline Vec4<S, T> pr_vectorcall Abs(Vec4<S, T> v)
	{
		if constexpr (Vec4<S, T>::IntrinsicF)
		{
			return _mm_andnot_ps(_mm_set_ps1(-0.0f), v.vec);
		}
		else
		{
			return Vec4<S, T>{Abs(v.x), Abs(v.y), Abs(v.z), Abs(v.w)};
		}
	}

	// V4 length squared
	template <Scalar S, typename T> inline S pr_vectorcall LengthSq(Vec4<S, T> v)
	{
		if constexpr (Vec4<S, T>::IntrinsicF)
		{
			return _mm_dp_ps(v.vec, v.vec, 0xF1).m128_f32[0];
		}
		else
		{
			return LenSq(v.x, v.y, v.z, v.w);
		}
	}

	// Largest/Smallest element
	template <Scalar S, typename T> inline S pr_vectorcall MinComponent(Vec4<S, T> v)
	{
		if constexpr (Vec4<S, T>::IntrinsicF)
		{
			// min([x y z w], [y x w z]) = [x<y?x:y y<x?y:x z<w?z:w w<z?w:z] = [a a b b]
			// min([a a b b], [b b a a]) = [m m m m]
			auto abcd = v.vec;
			auto aabb = _mm_min_ps(abcd, _mm_shuffle_ps(abcd, abcd, _MM_SHUFFLE(1, 0, 3, 2)));
			auto mmmm = _mm_min_ps(aabb, _mm_shuffle_ps(aabb, aabb, _MM_SHUFFLE(2, 3, 0, 1)));
			return mmmm.m128_f32[0];
		}
		else
		{
			return MinComponent<Vec4<S, T>>(v);
		}
	}
	template <Scalar S, typename T> inline S pr_vectorcall MaxElement(Vec4<S, T> v)
	{
		if constexpr (Vec4<S, T>::IntrinsicF)
		{
			// max([x y z w], [y x w z]) = [x>y?x:y y>x?y:x z>w?z:w w>z?w:z] = [a a b b]
			// max([a a b b], [b b a a]) = [m m m m]
			auto abcd = v.vec;
			auto aabb = _mm_max_ps(abcd, _mm_shuffle_ps(abcd, abcd, _MM_SHUFFLE(1, 0, 3, 2)));
			auto mmmm = _mm_max_ps(aabb, _mm_shuffle_ps(aabb, aabb, _MM_SHUFFLE(2, 3, 0, 1)));
			return mmmm.m128_f32[0];
		}
		else
		{
			return MaxComponent<Vec4<S, T>>(v);
		}
	}

	// Normalise all components of 'v'
	template <Scalar S, typename T> requires std::floating_point<S>
	inline Vec4<S, T> pr_vectorcall Normalise(Vec4<S, T> v)
	{
		if constexpr (Vec4<S, T>::IntrinsicF)
		{
			auto r = _mm_div_ps(v.vec, _mm_sqrt_ps(_mm_dp_ps(v.vec, v.vec, 0xFF)));
			return r;
		}
		else
		{
			return v / Length(v);
		}
	}

	// Square: v * v
	template <Scalar S, typename T> inline Vec4<S, T> pr_vectorcall Sqr(Vec4<S, T> v)
	{
		if constexpr (Vec4<S, T>::IntrinsicF)
		{
			return _mm_mul_ps(v.vec, v.vec);
		}
		else
		{
			return Vec4<S, T>{Sqr(v.x), Sqr(v.y), Sqr(v.z), Sqr(v.w)};
		}
	}

	// Dot product: a . b
	template <Scalar S, typename T> inline S pr_vectorcall Dot3(Vec4<S, T> a, Vec4<S, T> b)
	{
		if constexpr (Vec4<S, T>::IntrinsicF)
		{
			auto r = _mm_dp_ps(a.vec, b.vec, 0x71);
			return r.m128_f32[0];
		}
		else
		{
			return a.x * b.x + a.y * b.y + a.z * b.z;
		}
	}
	template <Scalar S, typename T> inline S pr_vectorcall Dot4(Vec4<S, T> a, Vec4<S, T> b)
	{
		if constexpr (Vec4<S, T>::IntrinsicF)
		{
			return _mm_dp_ps(a.vec, b.vec, 0xF1).m128_f32[0];
		}
		else
		{
			return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
		}
	}
	template <Scalar S, typename T> inline S pr_vectorcall Dot(Vec4<S, T> a, Vec4<S, T> b)
	{
		return Dot4(a,b);
	}

	// Cross product: a x b
	template <Scalar S, typename T> inline Vec4<S, T> pr_vectorcall Cross3(Vec4<S, T> a, Vec4<S, T> b)
	{
		if constexpr (Vec4<S, T>::IntrinsicF)
		{
			return _mm_sub_ps(
				_mm_mul_ps(_mm_shuffle_ps(a.vec, a.vec, _MM_SHUFFLE(3, 0, 2, 1)), _mm_shuffle_ps(b.vec, b.vec, _MM_SHUFFLE(3, 1, 0, 2))),
				_mm_mul_ps(_mm_shuffle_ps(a.vec, a.vec, _MM_SHUFFLE(3, 1, 0, 2)), _mm_shuffle_ps(b.vec, b.vec, _MM_SHUFFLE(3, 0, 2, 1))));
		}
		else
		{
			return Vec4<S, T>{a.y* b.z - a.z * b.y, a.z* b.x - a.x * b.z, a.x* b.y - a.y * b.x, 0};
		}
	}
	template <Scalar S, typename T> inline Vec4<S, T> pr_vectorcall Cross(Vec4<S, T> a, Vec4<S, T> b)
	{
		return Cross3(a,b);
	}

	// Triple product: a . b x c
	template <Scalar S, typename T> inline S pr_vectorcall Triple(Vec4<S, T> a, Vec4<S, T> b, Vec4<S, T> c)
	{
		return Dot3(a, Cross3(b, c));
	}

	// Returns true if 'a' and 'b' parallel
	template <Scalar S, typename T> inline bool pr_vectorcall Parallel(Vec4<S, T> v0, Vec4<S, T> v1, S tol = maths::tiny<S>)
	{
		// '<=' to allow for 'tol' == 0.0
		return LengthSq(Cross3(v0, v1)) <= Sqr(tol);
	}

	// Returns a vector guaranteed not parallel to 'v'
	template <Scalar S, typename T> inline Vec4<S, T> pr_vectorcall CreateNotParallelTo(Vec4<S, T> v)
	{
		bool x_aligned = Abs(v.x) > Abs(v.y) && Abs(v.x) > Abs(v.z);
		return Vec4<S, T>{static_cast<S>(!x_aligned), S(0), static_cast<S>(x_aligned), v.w};
	}

	// Returns a vector perpendicular to 'v' with the same length of 'v'
	template <Scalar S, typename T> inline Vec4<S, T> pr_vectorcall Perpendicular(Vec4<S, T> v)
	{
		pr_assert("Cannot make a perpendicular to a zero vector" && (v != Vec4<S, T>::Zero()));
		auto vec = Cross3(v, CreateNotParallelTo(v));
		vec *= Sqrt(LengthSq(v) / LengthSq(vec));
		return vec;
	}

	// Returns a vector perpendicular to 'vec' favouring 'previous' as the preferred perpendicular.
	template <Scalar S, typename T> inline Vec4<S, T> pr_vectorcall Perpendicular(Vec4<S, T> vec, Vec4<S, T> previous)
	{
		// The length of the returned vector will be 'Length(vec)' or 'Length(previous)' (typically they'd be the same)
		// Either 'vec' or 'previous' can be zero, but not both.
		if (vec == Vec4<S, T>::Zero())
		{
			// Both 'vec' and 'previous' cannot be zero
			pr_assert("Cannot make a perpendicular to a zero vector" && (previous != Vec4<S, T>::Zero()));
			return previous;
		}
		if (Parallel(vec, previous)) // includes 'previous' == zero
		{
			// If 'previous' is parallel to 'vec', choose a new perpendicular
			return Perpendicular(vec);
		}

		// If 'previous' is still perpendicular, reuse it
		if (FEql(Dot3(vec, previous), S(0)))
			return previous;

		// Otherwise, make a perpendicular that is close to 'previous'
		return Normalise(Cross3(Cross3(vec, previous), vec));
	}

	// Returns a vector with the 'xyz' values permuted 'n' times. '0=xyzw, 1=yzxw, 2=zxyw'
	template <Scalar S, typename T> inline Vec4<S, T> pr_vectorcall Permute3(Vec4<S, T> v, int n)
	{
		switch (n % 3)
		{
			case 1:  return Vec4<S, T>{v.y, v.z, v.x, v.w};
			case 2:  return Vec4<S, T>{v.z, v.x, v.y, v.w};
			default: return v;
		}
	}

	// Returns a vector with the values permuted 'n' times. '0=xyzw, 1=yzwx, 2=zwxy, 3=wxyz'
	template <Scalar S, typename T> inline Vec4<S, T> pr_vectorcall Permute4(Vec4<S, T> v, int n)
	{
		switch (n % 4)
		{
			case 1:  return Vec4<S, T>{v.y, v.z, v.w, v.x};
			case 2:  return Vec4<S, T>{v.z, v.w, v.x, v.y};
			case 3:  return Vec4<S, T>{v.w, v.x, v.y, v.z};
			default: return v;
		}
	}

	// Returns a 3-bit bitmask of the octant the vector is in. 0=(-x,-y,-z), 1=(+x,-y,-z), 2=(-x,+y,-z), 3=(+x,+y,-z), 4=(-x,-y+z), 5=(+x,-y,+z), 6=(-x,+y,+z), 7=(+x,+y,+z)
	template <Scalar S, typename T> inline uint32_t pr_vectorcall Octant(Vec4<S, T> v)
	{
		return Octant(v.xyz);
	}

	// Return the component sum
	template <Scalar S, typename T> inline S pr_vectorcall ComponentSum(Vec4<S, T> v)
	{
		if constexpr (Vec4<S, T>::IntrinsicF)
		{
			auto sum = v.vec;
			sum = _mm_hadd_ps(sum, sum);
			sum = _mm_hadd_ps(sum, sum);
			S s; _mm_store_ss(&s, sum);
			return s;
		}
		else
		{
			return v.x + v.y + v.z + v.w;
		}
	}

	// Returns the most extreme point in the direction of 'separating_axis'
	template <Scalar S, typename T> inline Vec4<S, T> pr_vectorcall SupportPoint(Vec4<S, T> pt, Vec4<S, T> separating_axis)
	{
		// This overload allows other generic functions to work
		(void)separating_axis;
		return pt;
	}
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
namespace pr::maths
{
	PRUnitTestClass(Vector4Tests)
	{
		PRUnitTestMethod(Operators, float, double, int32_t, int64_t)
		{
			using S = T;
			using vec4_t = Vec4<S, void>;
			
			auto a = vec4_t{10, 20, 30, 40};
			auto b = vec4_t{-40, -30, -20, -10};

			PR_EXPECT(a + b == vec4_t(-30, -10, +10, +30));
			PR_EXPECT(a - b == vec4_t(+50, +50, +50, +50));
			PR_EXPECT(a * 3 == vec4_t(+30, +60, +90, +120));
			PR_EXPECT(3 * a == vec4_t(+30, +60, +90, +120));
			PR_EXPECT(a / 2 == vec4_t(5, 10, 15, 20));
			PR_EXPECT(a % 20 == vec4_t(+10, +0, +10, +0));
			PR_EXPECT(120 / a == vec4_t(12, 6, 4, 3));

			auto arr0 = vec4_t(+1, -2, +3, -4);
			auto arr1 = vec4_t(-1, +2, -3, +4);
			PR_EXPECT((arr0 == arr1) == !(arr0 != arr1));
			PR_EXPECT((arr0 != arr1) == !(arr0 == arr1));
			PR_EXPECT((arr0 < arr1) == !(arr0 >= arr1));
			PR_EXPECT((arr0 > arr1) == !(arr0 <= arr1));
			PR_EXPECT((arr0 <= arr1) == !(arr0 > arr1));
			PR_EXPECT((arr0 >= arr1) == !(arr0 < arr1));
		}
		PRUnitTestMethod(LargestSmallestElement, float, double, int32_t, int64_t)
		{
			using S = T;
			using vec4_t = Vec4<S, void>;
			
			auto v1 = vec4_t{1,-2,-3,4};
			PR_EXPECT(MinComponent(v1) == S(-3));
			PR_EXPECT(MaxComponent(v1) == S(+4));
			PR_EXPECT(MinElementIndex(v1) == 2);
			PR_EXPECT(MaxElementIndex(v1) == 3);
		}
		PRUnitTestMethod(FEql, float, double)
		{
			using S = T;
			using vec3_t = Vec3<S, void>;
			using vec4_t = Vec4<S, void>;

			// Equal if the relative difference is less than tiny compared to the maximum element in the matrix.
			{
				auto a = vec4_t{S(-2),0,0,0};
				auto b = vec4_t{S(-2),0,0,maths::tiny<S>};
				PR_EXPECT(FEql(MinComponent(a), S(-2)));
				PR_EXPECT(FEql(MinComponent(b), S(-2)));
				PR_EXPECT(FEql(MaxComponent(a), S(0)));
				PR_EXPECT(FEql(MaxComponent(b), maths::tiny<S>));
				PR_EXPECT(FEql(a, b));
			}

			// Not equal if the relative difference is less than tiny compared to the maximum element in the matrix.
			{
				auto a = vec4_t{S(-0.5), 0, 0, 0};
				auto b = vec4_t{S(-0.5), 0, 0, maths::tiny<S>};
				PR_EXPECT(FEql(MinComponent(a), S(-0.5)));
				PR_EXPECT(FEql(MinComponent(b), S(-0.5)));
				PR_EXPECT(FEql(MaxComponent(a), S(0)));
				PR_EXPECT(FEql(MaxComponent(b), maths::tiny<S>));
				PR_EXPECT(!FEql(a, b));
			}
			{
				vec4_t a(1, 1, -1, -1);
				auto t2 = S(maths::tiny<S> *2);
				auto ht = S(maths::tiny<S> *0.5);
				PR_EXPECT(FEql(a, vec4_t(1, 1, -1, -1)));
				PR_EXPECT(!FEql(a, vec4_t(1 + t2, 1, -1, -1)));
				PR_EXPECT(FEql(vec4_t(ht, 0, 0, 1).xyz, vec3_t::Zero()));
				PR_EXPECT(FEql(vec4_t(ht, 0, 0, ht), vec4_t::Zero()));
			}
		}
		PRUnitTestMethod(Abs, float, double, int32_t, int64_t)
		{
			using S = T;
			using vec4_t = Vec4<S, void>;
			
			vec4_t arr0 = {+1, -2, +3, -4};
			vec4_t arr1 = {-1, +2, -3, +4};
			vec4_t arr2 = {+1, +2, +3, +4};
			PR_EXPECT(Abs(arr0) == Abs(arr1));
			PR_EXPECT(Abs(arr0) == Abs(arr2));
			PR_EXPECT(Abs(arr1) == Abs(arr2));
		}
		PRUnitTestMethod(Finite, float, double)
		{
			using S = T;
			using vec4_t = Vec4<S, void>;
			
			volatile auto f0 = S(0);
			vec4_t arr0(0, 1, 10, 1);
			vec4_t arr1(0, 1, 1 / f0, 0 / f0);
			PR_EXPECT(IsFinite(arr0));
			PR_EXPECT(!IsFinite(arr1));
			PR_EXPECT(!All(arr0, [](S x) { return x < S(5); }));
			PR_EXPECT(Any(arr0, [](S x) { return x < S(5); }));
		}
		PRUnitTestMethod(MinMaxClamp, float, double, int32_t, int64_t)
		{
			using S = T;
			using vec4_t = Vec4<S, void>;
			
			vec4_t a(3,-1,2,-4);
			vec4_t b = {-2,-1,4,2};
			PR_EXPECT(Max(a,b) == vec4_t(3,-1,4,2));
			PR_EXPECT(Min(a,b) == vec4_t(-2,-1,2,-4));

			auto arr0 = vec4_t(+1, -2, +3, -4);
			auto arr1 = vec4_t(-1, +2, -3, +4);
			auto arr2 = vec4_t(+0, +0, +0, +0);
			auto arr3 = vec4_t(+2, +2, +2, +2);
			PR_EXPECT(Min(arr0, arr1, arr2, arr3) == vec4_t(-1, -2, -3, -4));
			PR_EXPECT(Max(arr0, arr1, arr2, arr3) == vec4_t(+2, +2, +3, +4));
			PR_EXPECT(Clamp(arr0, arr2, arr3) == vec4_t(+1, +0, +2, +0));
		}
		PRUnitTestMethod(MinMaxComponent, float, double, int32_t, int64_t)
		{
			using S = T;
			using vec4_t = Vec4<S, void>;
			
			vec4_t a(3,-1,2,-4);
			PR_EXPECT(MinComponent(a) == S(-4));
			PR_EXPECT(MaxComponent(a) == S(+3));
		}
		PRUnitTestMethod(Truncate, float, double)
		{
			using S = T;
			using vec4_t = Vec4<S, void>;
			using vec3_t = Vec3<S, void>;
			using len_t = decltype(Sqrt(S{}));

			vec4_t arr0 = {S(+1.1), S(-1.2), S(+2.8), S(-2.9)};
			vec4_t arr1 = {S(+1.0), S(-1.0), S(+2.0), S(-2.0)};
			vec4_t arr2 = {S(+1.0), S(-1.0), S(+3.0), S(-3.0)};
			vec4_t arr3 = {S(+0.1), S(-0.2), S(+0.8), S(-0.9)};
			PR_EXPECT(Trunc(arr0, ETruncType::TowardZero) == arr1);
			PR_EXPECT(Trunc(arr0, ETruncType::ToNearest) == arr2);
			PR_EXPECT(FEql(Frac(arr0), arr3));
		}
		PRUnitTestMethod(Length, float, double, int32_t, int64_t)
		{
			using S = T;
			using vec4_t = Vec4<S, void>;
			using len_t = decltype(Length(vec4_t{}));

			vec4_t a(3,-1,2,-4);
			PR_EXPECT(LengthSq(a) == S(a.x*a.x + a.y*a.y + a.z*a.z + a.w*a.w));
			PR_EXPECT(Length(a) == len_t(Sqrt(LengthSq(a))));

			auto b = vec4_t(3, 4, 5, 6);
			PR_EXPECT(FEql(Length(b.xy), len_t(5.0)));
			PR_EXPECT(FEql(Length(b.xyz), len_t(10.0/maths::root2)));
			PR_EXPECT(FEql(Length(b), len_t(9.273618495495704)));
		}
		PRUnitTestMethod(Normalise, float, double)
		{
			using S = T;
			using vec4_t = Vec4<S, void>;
			using len_t = decltype(Length(vec4_t{}));

			vec4_t a(3, -1, 2, -4);
			vec4_t b = Normalise(a.w0());
			vec4_t c = Normalise(a);
			PR_EXPECT(FEql(Length(b), len_t(1.0)));
			PR_EXPECT(FEql(Length(c), len_t(1.0)));
			PR_EXPECT(!IsNormal(a));
			PR_EXPECT(IsNormal(b));
			PR_EXPECT(IsNormal(c));

			auto arr0 = vec4_t(1, 2, 3, 4);
			auto len0 = Sqrt(S(1*1 + 2*2 + 3*3 + 4*4));
			PR_EXPECT(FEql(Normalise(vec4_t::Zero(), arr0), arr0));
			PR_EXPECT(FEql(Normalise(arr0), arr0/len0));
		}
		PRUnitTestMethod(Dot, float, double, int32_t, int64_t)
		{
			using S = T;
			using vec4_t = Vec4<S, void>;
			
			vec4_t a = {-2,  4,  2,  6};
			vec4_t b = { 3, -5,  2, -4};
			PR_EXPECT(Dot4(a,b) == S(-46));
			PR_EXPECT(Dot3(a,b) == S(-22));
		}
		PRUnitTestMethod(ComponentSum, float, double, int32_t, int64_t)
		{
			using S = T;
			using vec4_t = Vec4<S, void>;
			
			vec4_t a = {1, 2, 3, 4};
			PR_EXPECT(ComponentSum(a) == S(1+2+3+4));
		}
		PRUnitTestMethod(Alignment, float, double, int32_t, int64_t)
		{
			using S = T;
			using vec4_t = Vec4<S, void>;
			
			char c0;
			vec4_t const pt0[] =
			{
				{1, 2, 3, 4},
				{5, 6, 7, 8},
			};
			char c1;
			vec4_t const pt1[] =
			{
				{1, 2, 3, 4},
				{5, 6, 7, 8},
			};
			(void)c0,c1;
			PR_EXPECT(is_aligned(&pt0[0]));
			PR_EXPECT(is_aligned(&pt1[0]));
		}
		PRUnitTestMethod(LinearInterpolate, float, double)
		{
			using S = T;
			using vec4_t = Vec4<S, void>;
			
			vec4_t arr0(1, 10, 100, 1000);
			vec4_t arr1(2, 20, 200, 2000);
			auto lerp = Lerp(arr0, arr1, S(0.7));
			auto expected = vec4_t(S(1.7), 17, 170, 1700);
			PR_EXPECT(FEql(lerp, expected));
		}
		PRUnitTestMethod(SphericalLinearInterpolate, float, double)
		{
			using S = T;
			using vec4_t = Vec4<S, void>;
			
			auto a0 = vec4_t::XAxis();
			auto a1 = S(2) * vec4_t::YAxis();
			auto slerp = Slerp(a0, a1, S(0.5));
			auto expected = S(1.5) * vec4_t::Normal(S(0.5), S(0.5), 0, 0);
			PR_EXPECT(FEql(slerp, expected));
		}
		PRUnitTestMethod(Quantise, float, double)
		{
			using S = T;
			using vec4_t = Vec4<S, void>;
			
			vec4_t arr0(S(1.0 / 3.0), 0, S(2.0), constants<S>::tau);
			auto quantised = Quantise(arr0, 1024);
			auto expected = vec4_t(S(0.3330078125), 0, S(2), S(6.2822265625000000));
			PR_EXPECT(FEql(quantised, expected));
		}
		PRUnitTestMethod(Random, float, double)
		{
			using S = T;
			using vec4_t = Vec4<S, void>;
			
			auto radius = 10;
			auto centre = vec4_t{1, 1, 1, 1};
			auto prev = vec4_t{};
			for (int i = 0; i != 100; ++i)
			{
				auto v = vec4_t::Random(g_rng(), centre, S(10));
				PR_EXPECT(v != prev);
				PR_EXPECT(Length(v - centre) < radius);
			}
		}
	};
}
#endif