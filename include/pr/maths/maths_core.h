//*****************************************************************************
// Maths library
//  Copyright (c) Rylogic Ltd 2002
//*****************************************************************************
#pragma once
#include "pr/maths/forward.h"
#include "pr/maths/constants.h"

namespace pr
{
	#pragma region Operators
	template <maths::VectorX T> inline bool operator == (T const& lhs, T const& rhs)
	{
		return memcmp(&lhs, &rhs, sizeof(lhs)) == 0;
	}
	template <maths::VectorX T> inline bool operator != (T const& lhs, T const& rhs)
	{
		return memcmp(&lhs, &rhs, sizeof(lhs)) != 0;
	}
	template <maths::VectorX T> inline bool operator <  (T const& lhs, T const& rhs)
	{
		return memcmp(&lhs, &rhs, sizeof(lhs)) < 0;
	}
	template <maths::VectorX T> inline bool operator >  (T const& lhs, T const& rhs)
	{
		return memcmp(&lhs, &rhs, sizeof(lhs)) > 0;
	}
	template <maths::VectorX T> inline bool operator <= (T const& lhs, T const& rhs)
	{
		return memcmp(&lhs, &rhs, sizeof(lhs)) <= 0;
	}
	template <maths::VectorX T> inline bool operator >= (T const& lhs, T const& rhs)
	{
		return memcmp(&lhs, &rhs, sizeof(lhs)) >= 0;
	}
	template <maths::VectorX T> inline T& operator += (T& lhs, T const& rhs)
	{
		return lhs = lhs + rhs;
	}
	template <maths::VectorX T> inline T& operator -= (T& lhs, T const& rhs)
	{
		return lhs = lhs - rhs;
	}
	template <maths::VectorX T> inline T& operator *= (T& lhs, T const& rhs)
	{
		return lhs = lhs * rhs;
	}
	template <maths::VectorX T> inline T& operator /= (T& lhs, T const& rhs)
	{
		return lhs = lhs / rhs;
	}
	template <maths::VectorX T> inline T& operator %= (T& lhs, T const& rhs)
	{
		return lhs = lhs % rhs;
	}
	template <maths::VectorX T> inline T& operator *= (T& lhs, maths::vec_comp_t<T> rhs)
	{
		return lhs = lhs * rhs;
	}
	template <maths::VectorX T> inline T& operator /= (T& lhs, maths::vec_comp_t<T> rhs)
	{
		return lhs = lhs / rhs;
	}
	template <maths::VectorX T> inline T& operator %= (T& lhs, maths::vec_comp_t<T> rhs)
	{
		return lhs = lhs % rhs;
	}
	#pragma endregion

	// A global random generator. Non-deterministically seeded and not re-seed-able.
	inline std::default_random_engine& g_rng()
	{
		static std::random_device s_rd;
		static std::default_random_engine s_rng(s_rd());
		return s_rng;
	}

	// Compile time function for applying 'op' to each component of a vector
	template <maths::VectorX T, typename Op> constexpr T CompOp(T const& a, Op op)
	{
		T r;
		int i = 0, iend = maths::is_vec<T>::dim;
		for (; i != iend; ++i) r[i] = op(a[i]);
		return r;
	}
	template <maths::VectorX T, typename Op> constexpr T CompOp(T const& a, T const& b, Op op)
	{
		T r;
		int i = 0, iend = maths::is_vec<T>::dim;
		for (; i != iend; ++i) r[i] = op(a[i], b[i]);
		return r;
	}
	template <maths::VectorX T, typename Op> constexpr T CompOp(T const& a, T const& b, T const& c, Op op)
	{
		T r;
		int i = 0, iend = maths::is_vec<T>::dim;
		for (; i != iend; ++i) r[i] = op(a[i], b[i], c[i]);
		return r;
	}
	template <maths::VectorX T, typename Op> constexpr T CompOp(T const& a, T const& b, T const& c, T const& d, Op op)
	{
		T r;
		int i = 0, iend = maths::is_vec<T>::dim;
		for (; i != iend; ++i) r[i] = op(a[i], b[i], c[i], d[i]);
		return r;
	}

	// Return true if any element satisfies 'Pred'
	template <maths::VectorX T, typename Pred> constexpr bool Any(T const& v, Pred pred)
	{
		int i = 0, iend = maths::is_vec<T>::dim;
		for (; i != iend; ++i)
		{
			if constexpr (maths::VectorX<maths::vec_elem_t<T>>)
			{
				if (Any(v[i], pred))
					break;
			}
			else
			{
				if (pred(v[i]))
					break;
			}
		}
		return i != iend;
	}

	// Return true if all elements satisfy 'Pred'
	template <maths::VectorX T, typename Pred> constexpr bool All(T const& v, Pred pred)
	{
		int i = 0, iend = maths::is_vec<T>::dim;
		for (; i != iend; ++i)
		{
			if constexpr (maths::VectorX<maths::vec_elem_t<T>>)
			{
				if (!All(v[i], pred))
					break;
			}
			else
			{
				if (!pred(v[i]))
					break;
			}
		}
		return i == iend;
	}

	// Equality
	template <maths::VectorX T> constexpr bool Equal(T const& lhs, T const& rhs)
	{
		int i = 0, iend = maths::is_vec<T>::dim;
		for (; i != iend; ++i)
		{
			if constexpr (maths::VectorX<maths::vec_elem_t<T>>)
			{
				if (!Equal(lhs[i], rhs[i]))
					break;
			}
			else
			{
				if (lhs[i] != rhs[i])
					break;
			}
		}
		return i == iend;
	}

	// Absolute value
	template <std::floating_point T> constexpr T Abs(T x)
	{
		return x >= 0 ? x : -x;
	}
	template <std::integral T> constexpr T Abs(T x)
	{
		if constexpr (std::signed_integral<T>)
			return x >= 0 ? x : -x;
		else
			return x;
	}
	template <maths::VectorX T> constexpr T Abs(T const& v)
	{
		return CompOp(v, [](auto x){ return Abs(x); });
	}
	template <Scalar S, int N> constexpr std::array<std::decay_t<S>, N> Abs(S const(&v)[N])
	{
		std::array<std::decay_t<S>, N> r = {};
		for (int i = 0, iend = N; i != iend; ++i)
			r[i] = Abs(v[i]);
		return r;
	}

	// Min/Max/Clamp
	template <typename T> T Min(T x, T y) requires (requires (T a, T b) { a < b; } && !maths::VectorX<T>)
	{
		return (x < y) ? x : y;
	}
	template <typename T> constexpr T Max(T x, T y) requires (requires (T a, T b) { a < b; } && !maths::VectorX<T>)
	{
		return (x < y) ? y : x;
	}
	template <typename T> constexpr T Clamp(T x, T mn, T mx) requires (requires (T a, T b) { a < b; } && !maths::VectorX<T>)
	{
		pr_assert("[min,max] must be a positive range" && !(mx < mn));
		return (mx < x) ? mx : (x < mn) ? mn : x;
	}
	template <maths::VectorX T> constexpr T Min(T const& x, T const& y)
	{
		T r;
		int i = 0, iend = maths::is_vec<T>::dim;
		for (; i != iend; ++i)
			r[i] = Min(x[i], y[i]);
		
		return r;
	}
	template <maths::VectorX T> constexpr T Max(T const& x, T const& y)
	{
		T r;
		int i = 0, iend = maths::is_vec<T>::dim;
		for (; i != iend; ++i)
			r[i] = Max(x[i], y[i]);

		return r;
	}
	template <maths::VectorX T> constexpr T Clamp(T const& x, T const& mn, T const& mx)
	{
		T r;
		int i = 0, iend = maths::is_vec<T>::dim;
		for (; i != iend; ++i)
			r[i] = Clamp(x[i], mn[i], mx[i]);

		return r;
	}
	template <maths::VectorX T> constexpr T Clamp(T const& x, maths::vec_comp_t<T> mn, maths::vec_comp_t<T> mx)
	{
		T r;
		int i = 0, iend = maths::is_vec<T>::dim;
		for (; i != iend; ++i)
			r[i] = Clamp(x[i], mn, mx);

		return r;
	}
	template <typename T, typename... A> constexpr T Min(T const& x, T const& y, A&&... a)
	{
		return Min(Min(x, y), std::forward<A>(a)...);
	}
	template <typename T, typename... A> constexpr T Max(T const& x, T const& y, A&&... a)
	{
		return Max(Max(x, y), std::forward<A>(a)...);
	}

	// Min/Max element (i.e. nearest to -inf/+inf)
	template <maths::VectorX T> constexpr maths::vec_comp_t<T> MinComponent(T const& v)
	{
		if constexpr (maths::VectorX<maths::vec_elem_t<T>>)
		{
			auto min = MinComponent(v[0]);
			int i = 1, iend = maths::is_vec<T>::dim;
			for (; i != iend; ++i)
				min = Min(min, MinComponent(v[i]));

			return min;
		}
		else
		{
			auto min = v[0];
			int i = 1, iend = maths::is_vec<T>::dim;
			for (; i != iend; ++i)
				min = Min(min, v[i]);
			
			return min;
		}
	}
	template <maths::VectorX T> constexpr maths::vec_comp_t<T> MaxComponent(T const& v)
	{
		if constexpr (maths::VectorX<maths::vec_elem_t<T>>)
		{
			auto max = MaxComponent(v[0]);
			int i = 1, iend = maths::is_vec<T>::dim;
			for (; i != iend; ++i)
				max = Max(max, MaxComponent(v[i]));

			return max;
		}
		else
		{
			auto max = v[0];
			int i = 1, iend = maths::is_vec<T>::dim;
			for (; i != iend; ++i)
				max = Max(max, v[i]);
	
			return max;
		}
	}
	template <typename T> inline T MinComponent(std::span<T> const& a)
	{
		if (a.empty())
			throw std::runtime_error("minimum undefined on zero length span");

		auto min = MinComponent(a[0]);
		int i = 1, iend = static_cast<int>(a.size());
		for (; i != iend; ++i)
			min = Min(min, MinComponent(a[i]));

		return min;
	}
	template <typename T> inline T MaxComponent(std::span<T> const& a)
	{
		if (a.empty())
			throw std::runtime_error("maximum undefined on zero length span");

		auto max = MaxComponent(a[0]);
		int i = 1, iend = static_cast<int>(a.size());
		for (; i != iend; ++i)
			max = Max(max, MaxComponent(a[i]));

		return max;
	}

	// Min/Max absolute element (i.e. nearest to 0/+inf)
	template <maths::VectorX T> constexpr maths::vec_comp_t<T> MinComponentAbs(T const& v)
	{
		auto op = [](auto x)
		{
			if constexpr (maths::VectorX<maths::vec_elem_t<T>>)
				return MinComponentAbs(x);
			else
				return Abs(x);
		};
		auto min = op(v[0]);
		int i = 1, iend = maths::is_vec<T>::dim;
		for (; i != iend; ++i)
			min = Min(min, op(v[i]));
		
		return min;
	}
	template <maths::VectorX T> constexpr maths::vec_comp_t<T> MaxComponentAbs(T const& v)
	{
		auto op = [](auto x)
		{
			if constexpr (maths::VectorX<maths::vec_elem_t<T>>)
				return MaxComponentAbs(x);
			else
				return Abs(x);
		};
		auto max = op(v[0]);
		int i = 1, iend = maths::is_vec<T>::dim;
		for (; i != iend; ++i)
			max = Max(max, op(v[i]));

		return max;
	}
	template <typename T> inline T MinComponentAbs(std::span<T> a)
	{
		if (a.empty())
			throw std::runtime_error("minimum undefined on zero length span");
		
		auto op = [](auto x)
		{
			if constexpr (maths::VectorX<T>)
				return MinComponentAbs(x);
			else
				return x;
		};
		auto min = op(a[0]);
		int i = 1, iend = static_cast<int>(a.size());
		for (; i != iend; ++i)
			min = Min(min, op(a[i]));

		return min;
	}
	template <typename T> inline T MaxComponentAbs(std::span<T> a)
	{
		if (a.empty())
			throw std::runtime_error("maximum undefined on zero length span");

		auto op = [](auto x)
		{
			if constexpr (maths::VectorX<T>)
				return MaxComponentAbs(x);
			else
				return x;
		};
		auto max = op(a[0]);
		int i = 1, iend = static_cast<int>(a.size());
		for (; i != iend; ++i)
			max = Max(max, op(a[i]));

		return max;
	}

	// Smallest/Largest element index. Returns the index of the first min/max element if elements are equal.
	template <maths::VectorX T> constexpr int MinElementIndex(T const& v)
	{
		auto idx = 0;
		int i = 1, iend = maths::is_vec<T>::dim;
		for (; i != iend; ++i)
		{
			if (v[i] >= v[idx]) continue;
			idx = i;
		}
		return idx;
	}
	template <maths::VectorX T> constexpr int MaxElementIndex(T const& v)
	{
		auto idx = 0;
		int i = 1, iend = maths::is_vec<T>::dim;
		for (; i != iend; ++i)
		{
			if (v[i] <= v[idx]) continue;
			idx = i;
		}
		return idx;
	}

	#pragma warning (disable:4756) // Constant overflow in floating point arithmetic

	// Floating point comparisons. *WARNING* 'tol' is an absolute tolerance. Returns true if a is in the range (b-tol,b+tol)
	template <std::floating_point T> constexpr bool FEqlAbsolute(T a, T b, T tol)
	{
		// When float operations are performed at compile time, the compiler warnings about 'inf'
		pr_assert(tol >= 0 || !(tol == tol)); // NaN is not an error, comparisons with NaN are defined to always be false
		return Abs(a - b) < tol;
	}
	template <std::floating_point T> inline bool FEqlAbsolute(std::span<T> a, std::span<T> b, std::decay_t<T> tol)
	{
		if (a.size() != b.size())
			return false;

		int i = 0, iend = int(a.size());
		for (; i != iend && FEqlAbsolute(a[i], b[i], tol); ++i) {}
		return i == iend;
	}
	template <maths::VectorFP T> constexpr bool FEqlAbsolute(T const& a, T const& b, maths::vec_comp_t<T> tol)
	{
		int i = 0, iend = maths::is_vec<T>::dim;
		for (; i != iend && FEqlAbsolute(a[i], b[i], tol); ++i) {}
		return i == iend;
	}

	// *WARNING* 'tol' is a relative tolerance, relative to the largest of 'a' or 'b'
	template <std::floating_point T> constexpr bool FEqlRelative(T a, T b, T tol)
	{
		// Floating point compare is dangerous and subtle.
		// See: https://randomascii.wordpress.com/2012/02/25/comparing-floating-point-numbers-2012-edition/
		// and: http://floating-point-gui.de/errors/NearlyEqualsTest.java
		// Tests against zero treat 'tol' as an absolute difference threshold.
		// Tests between two non-zero values use 'tol' as a relative difference threshold.
		// i.e.
		//    FEql(2e-30, 1e-30) == false
		//    FEql(2e-30 - 1e-30, 0) == true

		// Handles tests against zero where relative error is meaningless
		// Tests with 'b == 0' are the most common so do them first
		if (b == 0) return Abs(a) < tol;
		if (a == 0) return Abs(b) < tol;

		// Handle infinities and exact values
		if (a == b) return true;

		// Test relative error as a fraction of the largest value
		auto abs_max_element = Max(Abs(a), Abs(b));
		return FEqlAbsolute(a, b, tol * abs_max_element);
	}
	template <std::floating_point T> inline bool FEqlRelative(std::span<T> a, std::span<T> b, std::decay_t<T> tol)
	{
		if (a.size() != b.size()) return false;
		auto max_a = MaxComponentAbs(a);
		auto max_b = MaxComponentAbs(b);
		if (max_b == 0) return max_a < tol;
		if (max_a == 0) return max_b < tol;
		auto abs_max_element = Max(max_a, max_b);
		return FEqlAbsolute<T>(a, b, tol * abs_max_element);
	}
	template <maths::VectorFP T> constexpr bool FEqlRelative(T const& a, T const& b, maths::vec_comp_t<T> tol)
	{
		auto max_a = MaxComponentAbs(a);
		auto max_b = MaxComponentAbs(b);
		if (max_b == 0) return max_a < tol;
		if (max_a == 0) return max_b < tol;
		auto abs_max_element = Max(max_a, max_b);
		return FEqlAbsolute(a, b, tol * abs_max_element);
	}

	// FEqlRelative using 'tiny'. Returns true if a in the range (b - max(a,b)*tiny, b + max(a,b)*tiny)
	template <std::floating_point T> constexpr bool FEql(T a, T b)
	{
		// Don't add a 'tol' parameter because it looks like the function should perform a == b +- tol, which isn't what it does.
		return FEqlRelative(a, b, maths::tiny<T>);
	}
	template <std::floating_point T> inline bool FEql(std::span<T> a, std::span<T> b)
	{
		return FEqlRelative(a, b, maths::tiny<T>);
	}
	template <maths::VectorFP T> constexpr bool FEql(T const& a, T const& b)
	{
		return FEqlRelative(a, b, maths::tiny<maths::vec_comp_t<T>>);
	}

	// Define FEql for integral types to make vector overloads simpler
	template <std::integral T> constexpr bool FEql(T a, T b)
	{
		return a == b;
	}
	template <std::integral T> inline bool FEql(std::span<T> a, std::span<T> b)
	{
		if (a.size() != b.size()) return false;
		size_t i = 0, iend = a.size();
		for (; i != iend && a[i] == b[i]; ++i) {}
		return i == iend;
	}
	template <maths::VectorIg T> constexpr bool FEql(T const& a, T const& b)
	{
		int i = 0, iend = maths::is_vec<T>::dim;
		for (; i != iend && a[i] == b[i]; ++i) {}
		return i == iend;
	}

	// NaN test
	inline bool IsNaN(float value)
	{
		return std::isnan(value);
	}
	inline bool IsNaN(double value)
	{
		return std::isnan(value);
	}
	template <std::integral T> inline bool IsNaN(T)
	{
		return false;
	}
	template <maths::VectorX T> inline bool IsNaN(T const& value, bool any = true) // false = all
	{
		int i = 0, iend = maths::is_vec<T>::dim;
		if (any)
		{
			for (; i != iend && !IsNaN(value[i]); ++i) {}
			return i != iend;
		}
		else
		{
			for (; i != iend && IsNaN(value[i]); ++i) {}
			return i == iend;
		}
	}

	// Finite test
	inline bool IsFinite(float value)
	{
		return std::isfinite(value);
	}
	inline bool IsFinite(double value)
	{
		return std::isfinite(value);
	}
	inline bool IsFinite(float value, float max_value)
	{
		return IsFinite(value) && std::fabs(value) < max_value;
	}
	inline bool IsFinite(double value, double max_value)
	{
		return IsFinite(value) && std::fabs(value) < max_value;
	}
	template <std::integral T> inline bool IsFinite(T value)
	{
		return value >= limits<T>::lowest() && value <= limits<T>::max();
	}
	template <std::integral T> inline bool IsFinite(T value, T max_value)
	{
		return IsFinite(value) && Abs(value) < max_value;
	}
	template <maths::VectorX T> inline bool IsFinite(T const& value, bool any = false)
	{
		return any
			? Any(value, [](auto x) { return IsFinite(x); })
			: All(value, [](auto x) { return IsFinite(x); });
	}

	// Ceil/Floor/Round/Modulus
	template <Scalar S> inline S Ceil(S x)
	{
		return static_cast<S>(std::ceil(x));
	}
	template <Scalar S> inline S Floor(S x)
	{
		return static_cast<S>(std::floor(x));
	}
	template <Scalar S> inline S Round(S x)
	{
		return static_cast<S>(std::round(x));
	}
	template <Scalar S> inline S RoundSD(S d, int significant_digits)
	{
		pr_assert(significant_digits >= 0 && "'significant_digits' value must be >= 0");

		// No significant digits is always zero
		if (d == 0 || significant_digits == 0)
			return 0;

		if constexpr (std::is_same_v<S, long long>) // int64_t is 19 digits
		{
			if (significant_digits > 19)
				return d;
		}
		if constexpr (std::is_same_v<S, float>) // float's mantissa is 7 digits
		{
			if (significant_digits > 7)
				return d;
		}
		if constexpr (std::is_same_v<S, double>) // double's mantissa is 17 digits
		{
			if (significant_digits > 17)
				return d;
		}

		auto pow = static_cast<int>(std::floor(std::log10(std::abs(d))));
		auto scale = std::pow(S(10), significant_digits - pow - 1);
		auto result = scale != 0 ? static_cast<S>(Round<double>(d * scale) / scale) : S{};
		return result;
	}
	template <Scalar S> inline S Modulus(S x, S y)
	{
		if constexpr (std::floating_point<S>)
			return std::fmod(x, y);
		else
			return x % y;
	}
	template <maths::VectorX T> inline T Ceil(T const& v)
	{
		// Note: arrays as vectors cannot use this function because arrays cannot be returned by value
		return CompOp(v, [](auto x) { return Ceil(x); });
	}
	template <maths::VectorX T> inline T Floor(T const& v)
	{
		// Note: arrays as vectors cannot use this function because arrays cannot be returned by value
		return CompOp(v, [](auto x) { return Floor(x); });
	}
	template <maths::VectorX T> inline T Round(T const& v)
	{
		// Note: arrays as vectors cannot use this function because arrays cannot be returned by value
		return CompOp(v, [](auto x) { return Round(x); });
	}
	template <maths::VectorX T> inline T RoundSD(T const& v, int significant_digits)
	{
		// Note: arrays as vectors cannot use this function because arrays cannot be returned by value
		return CompOp(v, [=](auto x) { return RoundSD(x, significant_digits); });
	}
	template <maths::VectorX T> inline T Modulus(T const& x, T const& y)
	{
		return CompOp(x, y, [](auto x, auto y) { return Modulus(x, y); });
	}

	// Wrap 'x' to range [mn, mx)
	template <Scalar S> constexpr S Wrap(S x, S mn, S mx)
	{
		// Given the range ['mn', 'mx') and 'x' somewhere on the number line.
		// Return 'x' wrapped into the range, allowing for 'x' < 'mn'.
		auto range = mx - mn;
		return mn + Modulus((Modulus(x - mn, range) + range), range);
	}

	// Converts bool to +1,-1 (note: no 0 value)
	constexpr int Bool2SignI(bool positive)
	{
		return positive ? +1 : -1;
	}
	constexpr float Bool2SignF(bool positive)
	{
		return positive ? +1.0f : -1.0f;
	}

	// Sign, returns +1 if x >= 0 otherwise -1. If 'zero_is_positive' is false, then 0 in gives 0 out.
	template <Scalar S> constexpr S Sign(S x, bool zero_is_positive = true)
	{
		if constexpr (std::is_unsigned_v<S>)
			return x > 0 ? +S(1) : static_cast<S>(zero_is_positive);
		else
			return x > 0 ? +S(1) : x < 0 ? -S(1) : static_cast<S>(zero_is_positive);
	}
	template <maths::VectorX T> constexpr T Sign(T const& v, bool zero_is_positive = true)
	{
		return CompOp(v, [=](auto x) { return Sign(x, zero_is_positive); });
	}

	// Divide 'a' by 'b' if 'b' is not equal to zero, otherwise return 'def'
	template <typename T> requires (requires (T x) { x / x; x != x; })
	constexpr T Div(T a, T b, T def = T())
	{
		return b != T() ? a / b : def;
	}

	// Truncate value
	enum class ETruncType { TowardZero, ToNearest };
	template <std::floating_point S> constexpr S Trunc(S x, ETruncType ty = ETruncType::TowardZero)
	{
		switch (ty)
		{
			case ETruncType::ToNearest:  return static_cast<S>(static_cast<long long>(x + Sign(x) * S(0.5)));
			case ETruncType::TowardZero: return static_cast<S>(static_cast<long long>(x));
			default: pr_assert("Unknown truncation type" && false); return x;
		}
	}
	template <maths::VectorFP T> constexpr T Trunc(T const& v, ETruncType ty = ETruncType::TowardZero)
	{
		return CompOp(v, [=](auto x) { return Trunc(x, ty); });
	}

	// Fractional part
	template <std::floating_point S> inline S Frac(S x)
	{
		S n;
		return std::modf(x, &n);
	}
	template <maths::VectorFP T> inline T Frac(T const& v)
	{
		return CompOp(v, [](auto x) { return Frac(x); });
	}

	// Square a value
	template <Scalar S> constexpr S Sqr(S x)
	{
		if constexpr (std::is_same_v<S, int8_t>)
			pr_assert("Overflow" && Abs(x) <= 0xB);
		if constexpr (std::is_same_v<S, uint8_t>)
			pr_assert("Overflow" && Abs(x) <= 0xF);
		if constexpr (std::is_same_v<S, int16_t>)
			pr_assert("Overflow" && Abs(x) <= 0xB5);
		if constexpr (std::is_same_v<S, uint16_t>)
			pr_assert("Overflow" && Abs(x) <= 0xFF);
		if constexpr (std::is_same_v<S, int32_t> || std::is_same_v<S, long>)
			pr_assert("Overflow" && Abs(x) <= 0xB504);
		if constexpr (std::is_same_v<S, uint32_t> || std::is_same_v<S, unsigned long>)
			pr_assert("Overflow" && Abs(x) <= 0xFFFFU);
		if constexpr (std::is_same_v<S, int64_t>)
			pr_assert("Overflow" && Abs(x) <= 0xB504F333LL);
		if constexpr (std::is_same_v<S, uint64_t>)
			pr_assert("Overflow" && Abs(x) <= 0xFFFFFFFFULL);

		return x * x;
	}
	template <maths::VectorX T> constexpr T Sqr(T const& v)
	{
		return CompOp(v, [](auto x) { return Sqr(x); });
	}

	// Cube a value
	template <Scalar S> constexpr S Cube(S x)
	{
		if constexpr (std::is_same_v<S, int8_t>)
			pr_assert("Overflow" && Abs(x) <= 0x5);
		if constexpr (std::is_same_v<S, uint8_t>)
			pr_assert("Overflow" && Abs(x) <= 0x6);
		if constexpr (std::is_same_v<S, int16_t>)
			pr_assert("Overflow" && Abs(x) <= 0x1F);
		if constexpr (std::is_same_v<S, uint16_t>)
			pr_assert("Overflow" && Abs(x) <= 0x28);
		if constexpr (std::is_same_v<S, int32_t> || std::is_same_v<S, long>)
			pr_assert("Overflow" && Abs(x) <= 0x50A);
		if constexpr (std::is_same_v<S, uint32_t> || std::is_same_v<S, unsigned long>)
			pr_assert("Overflow" && Abs(x) <= 0x659U);
		if constexpr (std::is_same_v<S, int64_t>)
			pr_assert("Overflow" && Abs(x) <= 0x1FFFFFLL);
		if constexpr (std::is_same_v<S, uint64_t>)
			pr_assert("Overflow" && Abs(x) <= 0x285145ULL);

		return x * x * x;
	}
	template <maths::VectorX T> constexpr T Cube(T const& v)
	{
		return CompOp(v, [](auto x) { return Cube(x); });
	}

	// Square root
	template <Scalar S> auto Sqrt(S x) -> decltype(std::sqrt(x))
	{
		if constexpr (std::floating_point<S>)
			pr_assert("Sqrt of undefined value" && IsFinite(x));
		if constexpr (std::is_signed_v<S>)
			pr_assert("Sqrt of negative value" && x >= S(0));

		return std::sqrt(x);
	}
	template <maths::VectorX T> inline T Sqrt(T const&)
	{
		// Sqrt is ill-defined for non-square matrices.
		// Matrices have an overload that finds the matrix whose product is 'x'.
		static_assert(maths::always_false<T>, "Sqrt is not defined for general vector types");
	}
	template <maths::VectorX T> inline T CompSqrt(T const& v) // Component Sqrt
	{
		return CompOp(v, [](auto x)
			{
				if constexpr (maths::VectorX<maths::vec_elem_t<T>>)
					return CompSqrt(x);
				else
					return Sqrt(x);
			});
	}
	constexpr double SqrtCT(double x)
	{
		// Compile time version of the square root.
		//   - For a finite and non-negative value of "x", returns an approximation for the square root of "x"
		//   - Otherwise, returns NaN
		struct L {
		constexpr static double NewtonRaphson(double x, double curr, double prev)
		{
			return curr == prev ? curr : NewtonRaphson(x, 0.5 * (curr + x / curr), curr);
		}};

		return x >= 0 && x < limits<double>::infinity()
			? L::NewtonRaphson(x, x, 0)
			: limits<double>::quiet_NaN();
	}

	// Integer square root
	template <std::integral T> constexpr T ISqrt(T x)
	{
		// Compile time version of the square root.
		//  - For a finite and non-negative value of "x", returns an approximation for the square root of "x"
		//  - This method always converges or oscillates about the answer with a difference of 1.
		//  - Otherwise, returns 0
		if (x < 0)
			return std::numeric_limits<T>::quiet_NaN();

		T curr = x, prev = 0, pprev = 0;
		for (;curr != prev && curr != pprev;)
		{
			pprev = prev;
			prev = curr;
			curr = (curr + x / curr) >> 1;
		}
		return Abs(x - curr * curr) < Abs(x - prev * prev) ? curr : prev;
	}

	// Signed Sqr
	template <Scalar S> constexpr S SignedSqr(S x)
	{
		return x >= S() ? Sqr(x) : -Sqr(x);
	}
	template <maths::VectorX T> constexpr T SignedSqr(T const& v)
	{
		return CompOp(v, [](auto x) { return SignedSqr(x); });
	}

	// Signed Sqrt
	template <Scalar S> inline auto SignedSqrt(S x) -> decltype(Sqrt(x))
	{
		return x >= S(0) ? Sqrt(x) : -Sqrt(-x);
	}
	template <maths::VectorX T> inline T CompSignedSqrt(T const& v)
	{
		return CompOp(v, [](auto x)
			{
				if constexpr (maths::VectorX<maths::vec_elem_t<T>>)
					return CompSignedSqrt(x);
				else
					return SignedSqrt(x);
			});
	}

	// Angles
	template <std::floating_point T> constexpr inline T DegreesToRadians(T degrees)
	{
		return static_cast<T>(degrees * constants<double>::tau_by_360);
	}
	template <std::floating_point T> constexpr inline T RadiansToDegrees(T radians)
	{
		return static_cast<T>(radians * constants<double>::E60_by_tau);
	}
	template <maths::VectorX T> constexpr inline T DegreesToRadians(T v)
	{
		return CompOp(v, [](auto x)
			{
				if constexpr (maths::VectorX<maths::vec_elem_t<T>>)
					return DegreesToRadians(x);
				else
					return DegreesToRadians(x);
			});
	}
	template <maths::VectorX T> constexpr inline T RadiansToDegrees(T v)
	{
		return CompOp(v, [](auto x)
			{
				if constexpr (maths::VectorX<maths::vec_elem_t<T>>)
					return RadiansToDegrees(x);
				else
					return RadiansToDegrees(x);
			});
	}

	// Trig functions
	template <Scalar S> inline S Sin(S x)
	{
		return std::sin(x);
	}
	template <Scalar S> inline S Cos(S x)
	{
		return std::cos(x);
	}
	template <Scalar S> inline S Tan(S x)
	{
		return std::tan(x);
	}
	template <Scalar S> inline S Asin(S x)
	{
		return std::asin(x);
	}
	template <Scalar S> inline S Acos(S x)
	{
		return std::acos(x);
	}
	template <Scalar S> inline S Atan(S x)
	{
		return std::atan(x);
	}
	template <Scalar S> inline S Atan2(S y, S x)
	{
		return std::atan2(y, x);
	}
	template <Scalar S> inline S Atan2Positive(S y, S x)
	{
		auto a = std::atan2(y, x);
		if (a < S(0)) a += constants<S>::tau;
		return a;
	}
	template <Scalar S> inline S Sinh(S x)
	{
		return std::sinh(x);
	}
	template <Scalar S> inline S Cosh(S x)
	{
		return std::cosh(x);
	}
	template <Scalar S> inline S Tanh(S x)
	{
		return std::tanh(x);
	}
	template <maths::VectorX T> inline T Sin(T const& v)
	{
		return CompOp(v, [](auto x) { return Sin(x); });
	}
	template <maths::VectorX T> inline T Cos(T const& v)
	{
		return CompOp(v, [](auto x) { return Cos(x); });
	}
	template <maths::VectorX T> inline T Tan(T const& v)
	{
		return CompOp(v, [](auto x) { return Tan(x); });
	}
	template <maths::VectorX T> inline T Asin(T const& v)
	{
		return CompOp(v, [](auto x) { return Asin(x); });
	}
	template <maths::VectorX T> inline T Acos(T const& v)
	{
		return CompOp(v, [](auto x) { return Acos(x); });
	}
	template <maths::VectorX T> inline T Atan(T const& v)
	{
		return CompOp(v, [](auto x) { return Atan(x); });
	}
	template <maths::VectorX T> inline T Atan2(T const& y, T const& x)
	{
		return CompOp(y, x, [](auto y, auto x) { return Atan2(y, x); });
	}
	template <maths::VectorX T> inline T Atan2Positive(T const& y, T const& x)
	{
		return CompOp(y, x, [](auto y, auto x) { return Atan2Positive(y, x); });
	}
	template <maths::VectorX T> inline T Sinh(T const& v)
	{
		return CompOp(v, [](auto x) { return Sinh(x); });
	}
	template <maths::VectorX T> inline T Cosh(T const& v)
	{
		return CompOp(v, [](auto x) { return Cosh(x); });
	}
	template <maths::VectorX T> inline T Tanh(T const& v)
	{
		return CompOp(v, [](auto x) { return Tanh(x); });
	}

	// Power/Exponent/Log
	constexpr int Pow2(int n)
	{
		return 1 << n;
	}
	template <Scalar S> inline S Pow(S x, S y)
	{
		return s_cast<S>(std::pow(x, y));
	}
	template <Scalar S> inline S Exp(S x)
	{
		return s_cast<S>(std::exp(x));
	}
	template <Scalar S> inline S Log10(S x)
	{
		return s_cast<S>(std::log10(x));
	}
	template <Scalar S> inline S Log(S x)
	{
		return s_cast<S>(std::log(x));
	}
	template <maths::VectorX T> inline T Pow(T const& x, T const& y)
	{
		return CompOp(x, y, [](auto x, auto y) { return Pow(x, y); });
	}
	template <maths::VectorX T> inline T Exp(T const& x)
	{
		return CompOp(x, [](auto x) { return Exp(x); });
	}
	template <maths::VectorX T> inline T Log10(T const& x)
	{
		return CompOp(x, [](auto x) { return Log10(x); });
	}
	template <maths::VectorX T> inline T Log(T const& x)
	{
		return CompOp(x, [](auto x) { return Log(x); });
	}

	// Lengths
	template <Scalar S> constexpr S LenSq(S x, S y)
	{
		return Sqr(x) + Sqr(y);
	}
	template <Scalar S> constexpr S LenSq(S x, S y, S z)
	{
		return Sqr(x) + Sqr(y) + Sqr(z);
	}
	template <Scalar S> constexpr S LenSq(S x, S y, S z, S w)
	{
		return Sqr(x) + Sqr(y) + Sqr(z) + Sqr(w);
	}
	template <Scalar S> inline auto Len(S x, S y) -> decltype(Sqrt(x))
	{
		return Sqrt(LenSq(x, y));
	}
	template <Scalar S> inline auto Len(S x, S y, S z) -> decltype(Sqrt(x))
	{
		return Sqrt(LenSq(x, y, z));
	}
	template <Scalar S> inline auto Len(S x, S y, S z, S w) -> decltype(Sqrt(x))
	{
		return Sqrt(LenSq(x, y, z, w));
	}
	template <maths::VectorX T> constexpr maths::vec_elem_t<T> LengthSq(T const& x)
	{
		auto r = Sqr(x[0]);
		int i = 1, iend = maths::is_vec<T>::dim;
		for (; i != iend; ++i)
			r += Sqr(x[i]);

		return r;
	}
	template <maths::VectorX T> constexpr auto Length(T const& x) -> decltype(Sqrt(maths::vec_elem_t<T>()))
	{
		if constexpr (maths::VectorX<maths::vec_elem_t<T>>)
			return CompSqrt(LengthSq(x));
		else
			return Sqrt(LengthSq(x));
	}
	template <typename T> inline T LengthSq(std::complex<T> const& x)
	{
		return Sqr(x.real()) + Sqr(x.imag());
	}
	template <typename T> inline T Length(std::complex<T> const& x)
	{
		return Sqrt(LengthSq(x));
	}

	// Normalise - 2,3,4 variants scale all elements in the vector (consistent with DirectX)
	// These are generic functions. Normalise is specialised for intrinsics.
	// Note, due to FP rounding, normalising non-zero vectors can create zero vectors
	template <maths::VectorFP T> inline T Normalise(T const& v)
	{
		return v / Length(v);
	}
	template <maths::VectorFP T> inline T Normalise(T const& v, T const& def)
	{
		auto r = Normalise(v);
		return IsFinite(r) ? r : def;
	}
	template <maths::VectorFP T, typename F> requires (requires (F f) { { f() } -> std::convertible_to<T>; })
	inline T Normalise(T const& v, F def_factory)
	{
		auto r = Normalise(v);
		return IsFinite(r) ? r : def_factory();
	}
	template <maths::VectorX T> inline bool IsNormal(T const& v)
	{
		return FEql(LengthSq(v), static_cast<maths::vec_comp_t<T>>(1));
	}
	template <maths::VectorX T> inline bool IsNormalised(T const& v)
	{
		return IsNormal(v);
	}

	// Sum the elements in a vector
	template <maths::VectorX T> inline maths::vec_elem_t<T> Sum(T const& v)
	{
		auto r = v[0];
		int i = 1, iend = maths::is_vec<T>::dim;
		for (; i != iend; ++i)
			r += v[i];

		return r;
	}

	// Sum the result of applying 'pred' to each element in a vector
	template <maths::VectorX T, typename Pred> inline auto Sum(T const& v, Pred pred) -> decltype(pred(v[0]))
	{
		auto r = pred(v[0]);
		int i = 1, iend = maths::is_vec<T>::dim;
		for (; i != iend; ++i)
			r += pred(v[i]);

		return r;
	}

	// Dot product
	template <maths::VectorX T> inline maths::vec_elem_t<T> Dot(T const& a, T const& b)
	{
		if constexpr (maths::VectorX<maths::vec_elem_t<T>>)
		{
			auto r = Dot(a[0], b[0]);
			int i = 1, iend = maths::is_vec<T>::dim;
			for (; i != iend; ++i)
				r += Dot(a[i], b[i]);
			
			return r;
		}
		else
		{
			auto r = a[0] * b[0];
			int i = 1, iend = maths::is_vec<T>::dim;
			for (; i != iend; ++i)
				r += a[i] * b[i];
			
			return r;
		}
	}
	
	// Return the normalised fraction that 'x' is, in the range ['min', 'max']
	template <std::floating_point S> inline S Frac(S min, S x, S max)
	{
		pr_assert("Positive definite interval required for 'Frac'" && Abs(max - min) > 0);
		return (x - min) / (max - min);
	}
	template <maths::VectorX T> inline T Frac(T const& min, T const& x, T const& max)
	{
		auto n = x - min;
		auto d = max - min;
		return n / d;
	}

	// Linearly interpolate from 'lhs' to 'rhs'
	template <Scalar S, std::floating_point F> inline S Lerp(S lhs, S rhs, F frac)
	{
		return s_cast<S>(lhs + frac * (rhs - lhs));
	}
	template <maths::VectorFP T> inline T Lerp(T const& lhs, T const& rhs, maths::vec_comp_t<T> frac)
	{
		// Don't implement this for integral vector types, callers can just cast from FP to intg.
		return lhs + frac * (rhs - lhs);
	}

	// Spherical linear interpolation from 'a' to 'b' for t=[0,1]
	template <maths::VectorFP T> inline T Slerp(T const& a, T const& b, maths::vec_comp_t<T> frac)
	{
		pr_assert("Cannot spherically interpolate to/from the zero vector" && a != T{} && b != T{});

		auto a_len = Length(a);
		auto b_len = Length(b);
		auto len = Lerp(a_len, b_len, frac);
		auto vec = Normalise(((1 - frac) / a_len) * a + (frac / b_len) * b);
		return len * vec;
	}

	// Quantise a value to a power of two. 'scale' should be a power of 2, i.e. 256, 1024, 2048, etc
	template <std::floating_point F, std::integral I> inline F Quantise(F x, I scale)
	{
		// The purpose of 'Quantise' is to round 'x' to the nearest representable floating number using
		// 'N' mantissa bits where '1 << N' == 'scale.
		return static_cast<I>(x * scale) / static_cast<F>(scale);
	}
	template <maths::VectorX T, std::integral I> inline T Quantise(T const& x, I scale)
	{
		T r;
		int i = 0, iend = maths::is_vec<T>::dim;
		for (; i != iend; ++i)
			r[i] = Quantise(x[i], scale);

		return r;
	}

	// Return the cosine of the angle of the triangle apex opposite 'opp'
	template <std::floating_point T> inline T CosAngle(T adj0, T adj1, T opp)
	{
		pr_assert("Angle undefined an when adjacent length is zero" && !FEql(adj0, T{}) && !FEql(adj1, T{}));
		return Clamp<T>((adj0*adj0 + adj1*adj1 - opp*opp) / (2 * adj0 * adj1), -1, 1);
	}

	// Return the cosine of the angle between two vectors
	template <maths::VectorX T> inline maths::vec_comp_t<T> CosAngle(T const& lhs, T const& rhs)
	{
		pr_assert("CosAngle undefined for zero vectors" && lhs != T{} && rhs != T{});
		auto const one = maths::vec_comp_t<T>{1};
		return Clamp(Dot(lhs, rhs) / Sqrt(LengthSq(lhs) * LengthSq(rhs)), -one, +one);
	}

	// Return the angle (in radians) of the triangle apex opposite 'opp'
	template <std::floating_point T> inline T Angle(T adj0, T adj1, T opp)
	{
		return Acos(CosAngle(adj0, adj1, opp));
	}

	// Return the angle between two vectors
	template <maths::VectorX T> inline maths::vec_comp_t<T> Angle(T const& lhs, T const& rhs)
	{
		return Acos(CosAngle(lhs, rhs));
	}

	// Return the length of a triangle side given by two adjacent side lengths and an angle between them
	template <std::floating_point T> inline T Length(T adj0, T adj1, T angle)
	{
		auto len_sq = adj0*adj0 + adj1*adj1 - 2 * adj0 * adj1 * Cos(angle);
		return len_sq > 0 ? Sqrt(len_sq) : 0;
	}

	// Returns 1 if 'hi' is > 'lo' otherwise 0
	template <typename T> T Step(T lo, T hi)
	{
		return lo <= hi ? static_cast<T>(0) : static_cast<T>(1);
	}

	// Returns the 'Hermite' interpolation (3t^2 - 2t^3) between 'lo' and 'hi' for t=[0,1]
	template <std::floating_point T> inline T SmoothStep(T lo, T hi, T t)
	{
		if (lo == hi) return lo;
		t = Clamp((t - lo) / (hi - lo), static_cast<T>(0), static_cast<T>(1));
		return t * t * (3 - 2 * t);
	}

	// Returns a fifth-order 'Perlin' interpolation (6t^5 - 15t^4 + 10t^3) between 'lo' and 'hi' for t=[0,1]
	template <std::floating_point T> inline T SmoothStep2(T lo, T hi, T t)
	{
		if (lo == hi) return lo;
		t = Clamp((t - lo) / (hi - lo), static_cast<T>(0), static_cast<T>(1));
		return static_cast<T>(t * t * t * (t * (t * 6 - 15) + 10));
	}

	// Scale a value on the range [-inf,+inf] to within the range [-1,+1].
	template <std::floating_point T> inline T Sigmoid(T x, T n = T(1))
	{
		// 'n' is a horizontal scaling factor.
		// If n = 1, [-1,+1] maps to [-0.5, +0.5]
		// If n = 10, [-10,+10] maps to [-0.5, +0.5], etc
		return static_cast<T>(ATan(x/n) / maths::tau_by_4);
	}

	// Scale a value on the range [0,1] such that:' f(0) = 0, f(1) = 1, and df(0.5) = 0'
	template <std::floating_point T> inline T UnitCubic(T x)
	{
		// This is used to weight values so that values near 0.5 are favoured
		constexpr T four = static_cast<T>(4);
		constexpr T half = static_cast<T>(0.5);
		return four * Cube(x - half) + half;
	}

	// Low precision reciprocal square root
	template <std::floating_point T> inline T Rsqrt0(T x)
	{
		T r;
		if constexpr (PR_MATHS_USE_INTRINSICS && std::is_same_v<T, double>)
		{
			// todo
			//__m128d r0;
			//r0 = _mm_load_sd(&x);
			//r0 = _mm_rsqrt_sd(r0);
			//_mm_store_sd(&r, r0);
			r = T(1) / Sqrt(x);
		}
		else if constexpr (PR_MATHS_USE_INTRINSICS && std::is_same_v<T, float>)
		{
			__m128 r0;
			r0 = _mm_load_ss(&x);
			r0 = _mm_rsqrt_ss(r0);
			_mm_store_ss(&r, r0);
		}
		else
		{
			r = T(1) / Sqrt(x);
		}
		return r;
	}

	// High(er) precision reciprocal square root
	template <std::floating_point T> inline T Rsqrt1(T x)
	{
		constexpr T c0 = +3.0;
		constexpr T c1 = -0.5;

		T r;
		if constexpr (PR_MATHS_USE_INTRINSICS && std::is_same_v<T, double>)
		{
			//todo
			//__m128d r0, r1;
			//r0 = _mm_load_sd(&x);
			//r1 = _mm_rsqrt_sd(r0);
			//r0 = _mm_mul_sd(r0, r1); // The general 'Newton-Raphson' reciprocal square root recurrence:
			//r0 = _mm_mul_sd(r0, r1); // (3 - b * X * X) * (X / 2)
			//r0 = _mm_sub_sd(r0, _mm_load_sd(&c0));
			//r1 = _mm_mul_sd(r1, _mm_load_sd(&c1));
			//r0 = _mm_mul_sd(r0, r1);
			//_mm_store_sd(&r, r0);
			r = T(1) / Sqrt(x);
		}
		else if constexpr (PR_MATHS_USE_INTRINSICS && std::is_same_v<T, float>)
		{
			__m128 r0, r1;
			r0 = _mm_load_ss(&x);
			r1 = _mm_rsqrt_ss(r0);
			r0 = _mm_mul_ss(r0, r1); // The general 'Newton-Raphson' reciprocal square root recurrence:
			r0 = _mm_mul_ss(r0, r1); // (3 - b * X * X) * (X / 2)
			r0 = _mm_sub_ss(r0, _mm_load_ss(&c0));
			r1 = _mm_mul_ss(r1, _mm_load_ss(&c1));
			r0 = _mm_mul_ss(r0, r1);
			_mm_store_ss(&r, r0);
		}
		else
		{
			r = T(1) / Sqrt(x);
		}
		return r;
	}

	// Cube root
	inline float Cubert(float x)
	{
		// This works because the integer interpretation of an IEEE 754 float
		// is approximately the log2(x) scaled by 2^23. The basic idea is to
		// use the log2(x) value as the initial guess then do some 'Newton-Raphson'
		// iterations to find the actual root.
		
		if (x == 0)
			return x;

		auto flip_sign = x < 0.0f;
		if (flip_sign) x = -x;
		
		union { float f; unsigned long i; } as;
		as.f = x;
		as.i = (as.i + 2U * 0x3f800000) / 3U;
		auto guess = as.f;

		x *= 1.0f / 3.0f;
		guess = (x / (guess*guess) + guess * (2.0f / 3.0f));
		guess = (x / (guess*guess) + guess * (2.0f / 3.0f));
		guess = (x / (guess*guess) + guess * (2.0f / 3.0f));
		return (flip_sign ? -guess : guess);
	}
	inline double Cubert(double x)
	{
		if (x == 0)
			return x;

		auto flip_sign = x < 0.0f;
		if (flip_sign)  x = -x;

		union { double f; unsigned long long i; } as;
		as.f = x;
		as.i = (as.i + 2ULL * 0x3FF0000000000000ULL) / 3ULL;
		auto guess = as.f;

		x *= 1.0 / 3.0;
		guess = (x / (guess*guess) + guess * (2.0 / 3.0));
		guess = (x / (guess*guess) + guess * (2.0 / 3.0));
		guess = (x / (guess*guess) + guess * (2.0 / 3.0));
		guess = (x / (guess*guess) + guess * (2.0 / 3.0));
		guess = (x / (guess*guess) + guess * (2.0 / 3.0));
		return (flip_sign ? -guess : guess);
	}

	// Fast hash
	inline uint32_t Hash(float value, uint32_t max_value)
	{
		constexpr uint32_t h = 0x8da6b343; // Arbitrary prime
		int n = static_cast<int>(h * value);
		n = n % max_value;
		if (n < 0) n += max_value;
		return static_cast<uint32_t>(n);
	}
	template <maths::VectorX T> inline uint32_t Hash(T const& value, uint32_t max_value)
	{
		constexpr uint32_t h[] = {0x8da6b343, 0xd8163841, 0xcb1ab31f}; // Arbitrary Primes

		int n = 0;
		for (int i = 0, iend = maths::is_vec<T>::dim; i != iend; ++i)
			n += static_cast<int>(h[i % _countof(h)] * value[i]);

		n = n % max_value;
		if (n < 0) n += max_value;
		return static_cast<uint32_t>(n);
	}

	// Return the greatest common factor between 'a' and 'b'
	template <std::integral T> inline T GreatestCommonFactor(T a, T b)
	{
		// Uses the Euclidean algorithm. If the greatest common factor is 1, then 'a' and 'b' are co-prime
		while (b) { auto t = b; b = a % b; a = t; }
		return a;
	}

	// Return the least common multiple between 'a' and 'b'
	template <std::integral T> inline T LeastCommonMultiple(T a, T b)
	{
		return (a * b) / GreatestCommonFactor(a, b);
	}

	// Convert a decimal back to a rational
	inline void DecimalToRational(double num, int& numer, int& denom)
	{
		(void)num,numer,denom;
		// Algorithm:
		//  c = the number of digits that form the repeating part of 'num'
		//  l = the offset from the decimal point to the start of the repeating part of 'num'
		//  let d = 9[c times]0[l times]
		//     e.g. 0.1435282828... has c = 2 (28), and l = 4 (0.1435), so d = 990000
		//          0.125           has c = 0, and l = 3, so d = 1000
		//  let n = the digits right of the decimal point with the repeating part removed
		//     e.g. 0.1435282828... so n = 1435
		//          0.125           so n = 125
		//  Find the GCF between n and d to get the rational: n/d
		//
		// Explanation:
		//   0.123(45)... (45 repeating) equals 0.123 + (45/10000 + 45/1000000 + 45/100000000 + ...)
		//   which is 123/1000 + the sum of the geometric series with ratio = 1/100
		//     = 123/1000 + (45/10000).(1/(1-(1/100)))
		//     = 123/1000 + (45/10000).(1/99)
		//     = 123/1000 + (45/990000)
		//     = (99*123 + 45)/990000
		//     = 12222/990000

		// todo: implement this
	}

	// Returns the number to add to pad 'size' up to 'alignment'
	template <std::integral T> constexpr T Pad(T size, int alignment)
	{
		pr_assert(((alignment - 1) & alignment) == 0 && "alignment should be a power of two");
		return static_cast<T>(~(size - 1) & (alignment - 1));
	}

	// Returns 'size' increased to a multiple of 'alignment'
	template <std::integral T> constexpr T PadTo(T size, int alignment)
	{
		return size + Pad<T>(size, alignment);
	}

	// Return 'ptr' aligned to 'alignment'
	template <typename T> constexpr T const* AlignTo(T const* ptr, int alignment)
	{
		auto ofs = reinterpret_cast<uint8_t const*>(ptr) - reinterpret_cast<uint8_t const*>(0);
		return reinterpret_cast<T*>(PadTo(ofs, alignment));
	}
	template <typename T> constexpr T* AlignTo(T* ptr, int alignment)
	{
		auto ofs = reinterpret_cast<uint8_t const*>(ptr) - reinterpret_cast<uint8_t const*>(0);
		return reinterpret_cast<T*>(PadTo(ofs, alignment));
	}

	// Function object for generating an arithmetic sequence
	template <typename Type> struct ArithmeticSequence
	{
		// A sequence defined by:
		//  an = a0 + n * step
		//  Sn = (n + 1) * (a0 + an) / 2
		Type a0, step, a;

		ArithmeticSequence(Type initial_value = 0, Type step = 0)
			:a0(initial_value)
			,step(step)
			,a(a0)
		{}
		Type operator()(int n) const
		{
			return static_cast<Type>(a0 + (n - 1) * step);
		}
		Type operator()()
		{
			auto v = a;
			a = static_cast<Type>(a + step);
			return v;
		}
		Type sum(int n) const
		{
			return ArithmeticSum(a0, step, n);
		}
	};
	template <typename Type> constexpr Type ArithmeticSum(Type a0, Type step, int n)
	{
		auto an = a0 + n * step;
		return static_cast<Type>((n + 1) * (a0 + an) / 2);
	}

	// Function object for generating an geometric sequence
	template <typename Type> struct GeometricSequence
	{
		// A sequence defined by:
		//  an = am * r = a0 * r^n, (where m = n - 1)
		//  Sn = a0.(1 - r^(n+1)) / (1 - r)
		Type a0, ratio, a;

		GeometricSequence(Type initial_value = 0, Type ratio = Type(1))
			:a0(initial_value)
			,ratio(ratio)
			,a(a0)
		{}
		Type operator()(int n) const
		{
			return static_cast<Type>(a0 * Pow<double>(ratio, n + 1));
		}
		Type operator()()
		{
			auto v = a;
			a = static_cast<Type>(a * ratio);
			return v;
		}
		Type sum(int n) const
		{
			return GeometricSum(a0, ratio, n);
		}
	};
	template <typename Type> constexpr Type GeometricSum(Type a0, Type ratio, int n)
	{
		auto rn = Pow<double>(ratio, n + 1);
		return static_cast<Type>(a0 * (1 - rn) / (1 - ratio));
	}

	// Permute the elements in 'arr'. Start with an ordered array; [0,N).
	// Repeated calls generates all permutations of elements in the array. Number of permutations == n!
	template <std::integral T> bool PermutationFirst(T* arr, int n)
	{
		std::sort(arr, arr+n);
		return n != 0;
	}
	template <std::integral T> bool PermutationNext(T* arr, int n)
	{
		// Algorithm:
		// - find the last pair of values that has increasing order.
		// - swap the first of the pair with the next greater value from the values in [i+1,n)
		// - sort the values in [i+1,n)
		// e.g.
		//   Given '524761', the last pair with increasing order is '47'.
		//   Swap '4' with '6' because its the next greater value to the right of '4' => '526741'
		//   Sort the values right of '6' => '526147'

		// Find the last pair
		int i = n - 1;
		for (; i-- > 0 && arr[i] > arr[i+1];) {}
		if (i == -1) return false;

		// Swap 'arr[i]' with the nearest value greater than 'arr[i]'
		// to the right of 'i' then sort the values in the range: [i+1, n)
		int j = i + 1;
		for (int k = j + 1; k < n; ++k)
		{
			if (arr[k] < arr[i]) continue;
			if (arr[k] > arr[j]) continue;
			j = k;
		}
		std::swap(arr[i], arr[j]);
		std::sort(arr + i + 1, arr + n);
		return true;
	}

	// Create a random value on interval ['vmin', 'vmax']
	template <typename Rng = std::default_random_engine> inline float Random(Rng& rng, float vmin, float vmax)
	{
		std::uniform_real_distribution<float> dist(vmin, vmax);
		return dist(rng);
	}

	// Create a random value centred on 'centre' with radius 'radius'
	template <typename Rng = std::default_random_engine> inline float RandomC(Rng& rng, float centre, float radius)
	{
		return Random(rng, centre - radius, centre + radius);
	}
}

// Specialise ::std
namespace std
{
	template <pr::maths::VectorX T> inline T min(T const& lhs, T const& rhs)
	{
		T v;
		int i = 0, iend = pr::maths::is_vec<T>::dim;
		for (; i != iend; ++i)
			v[i] = min(lhs[i], rhs[i]);

		return v;
	}
	template <pr::maths::VectorX T> inline T max(T const& lhs, T const& rhs)
	{
		T v;
		int i = 0, iend = pr::maths::is_vec<T>::dim;
		for (; i != iend; ++i)
			v[i] = max(lhs[i], rhs[i]);

		return v;
	}
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
namespace pr::maths
{
	PRUnitTestClass(MathsCoreTests)
	{
		PRUnitTestMethod(Permutations)
		{
			auto eql = [](int const* arr, int a, int b, int c, int d)
			{
				return arr[0] == a && arr[1] == b && arr[2] == c && arr[3] == d;
			};
			{// 4-sequential
				int arr1[] = {1, 2, 3, 4};
				PR_EXPECT(PermutationFirst(arr1, _countof(arr1)) && eql(arr1, 1, 2, 3, 4));//0
				PR_EXPECT(PermutationNext(arr1, _countof(arr1)) && eql(arr1, 1, 2, 4, 3));//1
				PR_EXPECT(PermutationNext(arr1, _countof(arr1)) && eql(arr1, 1, 3, 2, 4));//2
				PR_EXPECT(PermutationNext(arr1, _countof(arr1)) && eql(arr1, 1, 3, 4, 2));//3
				PR_EXPECT(PermutationNext(arr1, _countof(arr1)) && eql(arr1, 1, 4, 2, 3));//4
				PR_EXPECT(PermutationNext(arr1, _countof(arr1)) && eql(arr1, 1, 4, 3, 2));//5
				PR_EXPECT(PermutationNext(arr1, _countof(arr1)) && eql(arr1, 2, 1, 3, 4));//6
				PR_EXPECT(PermutationNext(arr1, _countof(arr1)) && eql(arr1, 2, 1, 4, 3));//7
				PR_EXPECT(PermutationNext(arr1, _countof(arr1)) && eql(arr1, 2, 3, 1, 4));//8
				PR_EXPECT(PermutationNext(arr1, _countof(arr1)) && eql(arr1, 2, 3, 4, 1));//9
				PR_EXPECT(PermutationNext(arr1, _countof(arr1)) && eql(arr1, 2, 4, 1, 3));//10
				PR_EXPECT(PermutationNext(arr1, _countof(arr1)) && eql(arr1, 2, 4, 3, 1));//11
				PR_EXPECT(PermutationNext(arr1, _countof(arr1)) && eql(arr1, 3, 1, 2, 4));//12
				PR_EXPECT(PermutationNext(arr1, _countof(arr1)) && eql(arr1, 3, 1, 4, 2));//13
				PR_EXPECT(PermutationNext(arr1, _countof(arr1)) && eql(arr1, 3, 2, 1, 4));//14
				PR_EXPECT(PermutationNext(arr1, _countof(arr1)) && eql(arr1, 3, 2, 4, 1));//15
				PR_EXPECT(PermutationNext(arr1, _countof(arr1)) && eql(arr1, 3, 4, 1, 2));//16
				PR_EXPECT(PermutationNext(arr1, _countof(arr1)) && eql(arr1, 3, 4, 2, 1));//17
				PR_EXPECT(PermutationNext(arr1, _countof(arr1)) && eql(arr1, 4, 1, 2, 3));//18
				PR_EXPECT(PermutationNext(arr1, _countof(arr1)) && eql(arr1, 4, 1, 3, 2));//19
				PR_EXPECT(PermutationNext(arr1, _countof(arr1)) && eql(arr1, 4, 2, 1, 3));//20
				PR_EXPECT(PermutationNext(arr1, _countof(arr1)) && eql(arr1, 4, 2, 3, 1));//21
				PR_EXPECT(PermutationNext(arr1, _countof(arr1)) && eql(arr1, 4, 3, 1, 2));//22
				PR_EXPECT(PermutationNext(arr1, _countof(arr1)) && eql(arr1, 4, 3, 2, 1));//23
				PR_EXPECT(!PermutationNext(arr1, _countof(arr1)));//24
			}
			{// non-sequential
				int i, arr2[] = {-1, 4, 11, 20};
				for (i = 1; i != 24; ++i)//== 4!
				{
					PR_EXPECT(PermutationNext(arr2, _countof(arr2)));
					if (i == 6) PR_EXPECT(eql(arr2, 4, -1, 11, 20));
					if (i == 13) PR_EXPECT(eql(arr2, 11, -1, 20, 4));
				}
				PR_EXPECT(!PermutationNext(arr2, _countof(arr2)));
			}
			{// large number of permutations
				int i, arr3[] = {-10, -9, -8, -1, 0, +1, +3, +6, +9};
				for (i = 1; PermutationNext(arr3, _countof(arr3)); ++i) {}
				PR_EXPECT(i == 362880); // == 9!
			}
		}
		PRUnitTestMethod(FloatingPointCompare)
		{
			float const _6dp = 1.000000111e-6f;

			// Regular large numbers - generally not problematic
			PR_EXPECT(FEqlRelative(1000000.0f, 1000001.0f, _6dp));
			PR_EXPECT(FEqlRelative(1000001.0f, 1000000.0f, _6dp));
			PR_EXPECT(!FEqlRelative(1000000.0f, 1000010.0f, _6dp));
			PR_EXPECT(!FEqlRelative(1000010.0f, 1000000.0f, _6dp));

			// Negative large numbers
			PR_EXPECT(FEqlRelative(-1000000.0f, -1000001.0f, _6dp));
			PR_EXPECT(FEqlRelative(-1000001.0f, -1000000.0f, _6dp));
			PR_EXPECT(!FEqlRelative(-1000000.0f, -1000010.0f, _6dp));
			PR_EXPECT(!FEqlRelative(-1000010.0f, -1000000.0f, _6dp));

			// Numbers around 1
			PR_EXPECT(FEqlRelative(1.0000001f, 1.0000002f, _6dp));
			PR_EXPECT(FEqlRelative(1.0000002f, 1.0000001f, _6dp));
			PR_EXPECT(!FEqlRelative(1.0000020f, 1.0000010f, _6dp));
			PR_EXPECT(!FEqlRelative(1.0000010f, 1.0000020f, _6dp));

			// Numbers around -1
			PR_EXPECT(FEqlRelative(-1.0000001f, -1.0000002f, _6dp));
			PR_EXPECT(FEqlRelative(-1.0000002f, -1.0000001f, _6dp));
			PR_EXPECT(!FEqlRelative(-1.0000010f, -1.0000020f, _6dp));
			PR_EXPECT(!FEqlRelative(-1.0000020f, -1.0000010f, _6dp));

			// Numbers between 1 and 0
			PR_EXPECT(FEqlRelative(0.000000001000001f, 0.000000001000002f, _6dp));
			PR_EXPECT(FEqlRelative(0.000000001000002f, 0.000000001000001f, _6dp));
			PR_EXPECT(!FEqlRelative(0.000000000100002f, 0.000000000100001f, _6dp));
			PR_EXPECT(!FEqlRelative(0.000000000100001f, 0.000000000100002f, _6dp));

			// Numbers between -1 and 0
			PR_EXPECT(FEqlRelative(-0.0000000010000001f, -0.0000000010000002f, _6dp));
			PR_EXPECT(FEqlRelative(-0.0000000010000002f, -0.0000000010000001f, _6dp));
			PR_EXPECT(!FEqlRelative(-0.0000000001000002f, -0.0000000001000001f, _6dp));
			PR_EXPECT(!FEqlRelative(-0.0000000001000001f, -0.0000000001000002f, _6dp));

			// Comparisons involving zero
			PR_EXPECT(FEqlRelative(+0.0f, +0.0f, _6dp));
			PR_EXPECT(FEqlRelative(+0.0f, -0.0f, _6dp));
			PR_EXPECT(FEqlRelative(-0.0f, -0.0f, _6dp));
			PR_EXPECT(FEqlRelative(+0.000001f, +0.0f, _6dp));
			PR_EXPECT(FEqlRelative(+0.0f, +0.000001f, _6dp));
			PR_EXPECT(FEqlRelative(-0.000001f, +0.0f, _6dp));
			PR_EXPECT(FEqlRelative(+0.0f, -0.000001f, _6dp));
			PR_EXPECT(!FEqlRelative(+0.00001f, +0.0f, _6dp));
			PR_EXPECT(!FEqlRelative(+0.0f, +0.00001f, _6dp));
			PR_EXPECT(!FEqlRelative(-0.00001f, +0.0f, _6dp));
			PR_EXPECT(!FEqlRelative(+0.0f, -0.00001f, _6dp));

			// Comparisons involving extreme values (overflow potential)
			auto float_hi = limits<float>::max();
			auto float_lo = maths::float_lowest;
			PR_EXPECT(FEqlRelative(float_hi, float_hi, _6dp));
			PR_EXPECT(!FEqlRelative(float_hi, float_lo, _6dp));
			PR_EXPECT(!FEqlRelative(float_lo, float_hi, _6dp));
			PR_EXPECT(FEqlRelative(float_lo, float_lo, _6dp));
			PR_EXPECT(!FEqlRelative(float_hi, float_hi / 2, _6dp));
			PR_EXPECT(!FEqlRelative(float_hi, float_lo / 2, _6dp));
			PR_EXPECT(!FEqlRelative(float_lo, float_hi / 2, _6dp));
			PR_EXPECT(!FEqlRelative(float_lo, float_lo / 2, _6dp));

			// Comparisons involving infinities
			PR_EXPECT(FEqlRelative(+limits<float>::infinity(), +limits<float>::infinity(), _6dp));
			PR_EXPECT(FEqlRelative(-limits<float>::infinity(), -limits<float>::infinity(), _6dp));
			PR_EXPECT(!FEqlRelative(-limits<float>::infinity(), +limits<float>::infinity(), _6dp));
			PR_EXPECT(!FEqlRelative(+limits<float>::infinity(), +limits<float>::max(), _6dp));
			PR_EXPECT(!FEqlRelative(-limits<float>::infinity(), -limits<float>::max(), _6dp));

			// Comparisons involving NaN values
			PR_EXPECT(!FEqlRelative(maths::float_nan, maths::float_nan, _6dp));
			PR_EXPECT(!FEqlRelative(maths::float_nan, +0.0f, _6dp));
			PR_EXPECT(!FEqlRelative(-0.0f, maths::float_nan, _6dp));
			PR_EXPECT(!FEqlRelative(maths::float_nan, -0.0f, _6dp));
			PR_EXPECT(!FEqlRelative(+0.0f, maths::float_nan, _6dp));
			PR_EXPECT(!FEqlRelative(maths::float_nan, +limits<float>::infinity(), _6dp));
			PR_EXPECT(!FEqlRelative(+limits<float>::infinity(), maths::float_nan, _6dp));
			PR_EXPECT(!FEqlRelative(maths::float_nan, -limits<float>::infinity(), _6dp));
			PR_EXPECT(!FEqlRelative(-limits<float>::infinity(), maths::float_nan, _6dp));
			PR_EXPECT(!FEqlRelative(maths::float_nan, +limits<float>::max(), _6dp));
			PR_EXPECT(!FEqlRelative(+limits<float>::max(), maths::float_nan, _6dp));
			PR_EXPECT(!FEqlRelative(maths::float_nan, -limits<float>::max(), _6dp));
			PR_EXPECT(!FEqlRelative(-limits<float>::max(), maths::float_nan, _6dp));
			PR_EXPECT(!FEqlRelative(maths::float_nan, +maths::float_min, _6dp));
			PR_EXPECT(!FEqlRelative(+maths::float_min, maths::float_nan, _6dp));
			PR_EXPECT(!FEqlRelative(maths::float_nan, -maths::float_min, _6dp));
			PR_EXPECT(!FEqlRelative(-maths::float_min, maths::float_nan, _6dp));

			// Comparisons of numbers on opposite sides of 0
			PR_EXPECT(!FEqlRelative(+1.0f, -1.0f, _6dp));
			PR_EXPECT(!FEqlRelative(-1.0f, +1.0f, _6dp));
			PR_EXPECT(!FEqlRelative(+1.000000001f, -1.0f, _6dp));
			PR_EXPECT(!FEqlRelative(-1.0f, +1.000000001f, _6dp));
			PR_EXPECT(!FEqlRelative(-1.000000001f, +1.0f, _6dp));
			PR_EXPECT(!FEqlRelative(+1.0f, -1.000000001f, _6dp));
			PR_EXPECT(FEqlRelative(2 * maths::float_min, 0.0f, _6dp));
			PR_EXPECT(!FEqlRelative(maths::float_min, -maths::float_min, _6dp));

			// The really tricky part - comparisons of numbers very close to zero.
			PR_EXPECT(FEqlRelative(+maths::float_min, +maths::float_min, _6dp));
			PR_EXPECT(!FEqlRelative(+maths::float_min, -maths::float_min, _6dp));
			PR_EXPECT(!FEqlRelative(-maths::float_min, +maths::float_min, _6dp));
			PR_EXPECT(FEqlRelative(+maths::float_min, 0.0f, _6dp));
			PR_EXPECT(FEqlRelative(-maths::float_min, 0.0f, _6dp));
			PR_EXPECT(FEqlRelative(0.0f, +maths::float_min, _6dp));
			PR_EXPECT(FEqlRelative(0.0f, -maths::float_min, _6dp));

			PR_EXPECT(!FEqlRelative(0.000000001f, -maths::float_min, _6dp));
			PR_EXPECT(!FEqlRelative(0.000000001f, +maths::float_min, _6dp));
			PR_EXPECT(!FEqlRelative(+maths::float_min, 0.000000001f, _6dp));
			PR_EXPECT(!FEqlRelative(-maths::float_min, 0.000000001f, _6dp));
		}
		PRUnitTestMethod(FloatingPointVectorCompare)
		{
			float arr0[] = {1, 2, 3, 4};
			float arr1[] = {1, 2, 3, 5};
			static_assert(maths::VectorX<decltype(arr0)>);
			static_assert(maths::VectorX<decltype(arr1)>);

			PR_EXPECT(!Equal(arr0, arr1));
		}
		PRUnitTestMethod(FEqlArrays)
		{
			auto t0 = 0.0f;
			auto t1 = tiny<float> * 0.5f;
			auto t2 = tiny<float> * 1.5f;
			float arr0[] = {t0, 0, tiny<float>, -1};
			float arr1[] = {t1, 0, tiny<float>, -1};
			float arr2[] = {t2, 0, tiny<float>, -1};

			PR_EXPECT(FEql(arr0, arr1)); // Different by 1.000005%
			PR_EXPECT(!FEql(arr0, arr2)); // Different by 1.000015%
		}
		PRUnitTestMethod(Finite)
		{
			volatile auto f0 = 0.0f;
			volatile auto d0 = 0.0;
			PR_EXPECT(IsFinite(1.0f));
			PR_EXPECT(IsFinite(limits<int>::max()));
			PR_EXPECT(!IsFinite(1.0f / f0));
			PR_EXPECT(!IsFinite(0.0 / d0));
			PR_EXPECT(!IsFinite(11, 10));

			#if 0 // move
			iv2 arr4(10, 1);
			PR_EXPECT(IsFinite(arr4));
			PR_EXPECT(!All(arr4, [](int x) { return x < 5; }));
			PR_EXPECT(Any(arr4, [](int x) { return x < 5; }));
			#endif
		}
		PRUnitTestMethod(Abs)
		{
			PR_EXPECT(Abs(-1.0f) == Abs(-1.0f));
			PR_EXPECT(Abs(-1.0f) == Abs(+1.0f));
			PR_EXPECT(Abs(+1.0f) == Abs(+1.0f));

			float arr3[] = {+1, -2, +3, -4};
			float arr4[] = {+1, +2, +3, +4};
			auto arr5 = Abs(arr3);
			std::span<float const> span0(arr5);
			std::span<float const> span1(arr4);
			PR_EXPECT(FEql(span0, span1));

			std::array<float, 5> const arr6 = {1, 2, 3, 4, 5};
			std::span<float const> span5(arr6);
		}
		PRUnitTestMethod(AnyAll)
		{
			float arr0[] = {1.0f, 2.0f, 0.0f, -4.0f};
			auto are_zero = [](float x) { return x == 0.0f; };
			auto not_zero = [](float x) { return x != 0.0f; };

			PR_EXPECT(!All(arr0, are_zero));
			PR_EXPECT(!All(arr0, not_zero));
			PR_EXPECT(Any(arr0, not_zero));
			PR_EXPECT(Any(arr0, are_zero));
		}
		PRUnitTestMethod(Lengths)
		{
			PR_EXPECT(LenSq(3, 4) == 25);
			PR_EXPECT(LenSq(3, 4, 5) == 50);
			PR_EXPECT(LenSq(3, 4, 5, 6) == 86);
			PR_EXPECT(FEql(Len<float>(3, 4), 5.0f));
			PR_EXPECT(FEql(Len<float>(3, 4, 5), 7.0710678f));
			PR_EXPECT(FEql(Len<float>(3, 4, 5, 6), 9.2736185f));
		}
		PRUnitTestMethod(MinMaxClamp)
		{
			PR_EXPECT(Min(1, 2, -3, 4, -5) == -5);
			PR_EXPECT(Max(1, 2, -3, 4, -5) == 4);
			PR_EXPECT(Clamp(-1, 0, 10) == 0);
			PR_EXPECT(Clamp(3, 0, 10) == 3);
			PR_EXPECT(Clamp(12, 0, 10) == 10);
		}
		PRUnitTestMethod(Wrap)
		{
			PR_EXPECT(Wrap(-1, 0, 3) == 2); // [0, 3)
			PR_EXPECT(Wrap(+0, 0, 3) == 0);
			PR_EXPECT(Wrap(+1, 0, 3) == 1);
			PR_EXPECT(Wrap(+2, 0, 3) == 2);
			PR_EXPECT(Wrap(+3, 0, 3) == 0);
			PR_EXPECT(Wrap(+4, 0, 3) == 1);

			PR_EXPECT(Wrap(-1.0, 0.0, 3.0) == 2.0); // [0, 3)
			PR_EXPECT(Wrap(+0.0, 0.0, 3.0) == 0.0);
			PR_EXPECT(Wrap(+1.0, 0.0, 3.0) == 1.0);
			PR_EXPECT(Wrap(+2.0, 0.0, 3.0) == 2.0);
			PR_EXPECT(Wrap(+3.0, 0.0, 3.0) == 0.0);
			PR_EXPECT(Wrap(+4.0, 0.0, 3.0) == 1.0);

			PR_EXPECT(Wrap(-3, -2, +3) == +2); // [-2,+2]
			PR_EXPECT(Wrap(-2, -2, +3) == -2);
			PR_EXPECT(Wrap(-1, -2, +3) == -1);
			PR_EXPECT(Wrap(+0, -2, +3) == +0);
			PR_EXPECT(Wrap(+1, -2, +3) == +1);
			PR_EXPECT(Wrap(+2, -2, +3) == +2);
			PR_EXPECT(Wrap(+3, -2, +3) == -2);

			PR_EXPECT(Wrap(-3.0, -2.0, +3.0) == +2.0); // [-2,+2]
			PR_EXPECT(Wrap(-2.0, -2.0, +3.0) == -2.0);
			PR_EXPECT(Wrap(-1.0, -2.0, +3.0) == -1.0);
			PR_EXPECT(Wrap(+0.0, -2.0, +3.0) == +0.0);
			PR_EXPECT(Wrap(+1.0, -2.0, +3.0) == +1.0);
			PR_EXPECT(Wrap(+2.0, -2.0, +3.0) == +2.0);
			PR_EXPECT(Wrap(+3.0, -2.0, +3.0) == -2.0);

			PR_EXPECT(Wrap(+1, +2, +5) == 4); // [+2,+5)
			PR_EXPECT(Wrap(+2, +2, +5) == 2);
			PR_EXPECT(Wrap(+3, +2, +5) == 3);
			PR_EXPECT(Wrap(+4, +2, +5) == 4);
			PR_EXPECT(Wrap(+5, +2, +5) == 2);
			PR_EXPECT(Wrap(+6, +2, +5) == 3);

			PR_EXPECT(Wrap(-3, 0, 1) == 0); // [0,1)
			PR_EXPECT(Wrap(-2, 0, 1) == 0);
			PR_EXPECT(Wrap(-1, 0, 1) == 0);
			PR_EXPECT(Wrap(+0, 0, 1) == 0);
			PR_EXPECT(Wrap(+1, 0, 1) == 0);
			PR_EXPECT(Wrap(+2, 0, 1) == 0);
			PR_EXPECT(Wrap(+3, 0, 1) == 0);

			PR_EXPECT(Wrap(-3, -1, 0) == -1); // [-1,0)
			PR_EXPECT(Wrap(-2, -1, 0) == -1);
			PR_EXPECT(Wrap(-1, -1, 0) == -1);
			PR_EXPECT(Wrap(+0, -1, 0) == -1);
			PR_EXPECT(Wrap(+1, -1, 0) == -1);
			PR_EXPECT(Wrap(+2, -1, 0) == -1);
		}
		PRUnitTestMethod(SmallestLargestElement)
		{
			int arr0[] = {1, 2, 3, 4, 5};
			int arr1[] = {2, 1, 3, 4, 5};
			int arr2[] = {2, 3, 1, 4, 5};
			int arr3[] = {2, 3, 4, 1, 5};
			int arr4[] = {2, 3, 4, 5, 1};

			PR_EXPECT(MinComponent(arr0) == 1);
			PR_EXPECT(MinComponent(arr1) == 1);
			PR_EXPECT(MinComponent(arr2) == 1);
			PR_EXPECT(MinComponent(arr3) == 1);
			PR_EXPECT(MinComponent(arr4) == 1);

			float arr5[] = {1, 2, 3, 4, 5};
			float arr6[] = {1, 2, 3, 5, 4};
			float arr7[] = {2, 3, 5, 1, 4};
			float arr8[] = {2, 5, 3, 4, 1};
			float arr9[] = {5, 2, 3, 4, 1};
			PR_EXPECT(MaxComponent(arr5) == 5);
			PR_EXPECT(MaxComponent(arr6) == 5);
			PR_EXPECT(MaxComponent(arr7) == 5);
			PR_EXPECT(MaxComponent(arr8) == 5);
			PR_EXPECT(MaxComponent(arr9) == 5);
		}
		PRUnitTestMethod(SmallestLargestElementIndex)
		{
			int arr0[] = {1, 2, 3, 4, 5};
			int arr1[] = {2, 1, 3, 4, 5};
			int arr2[] = {2, 3, 1, 4, 5};
			int arr3[] = {2, 3, 4, 1, 5};
			int arr4[] = {2, 3, 4, 5, 1};

			PR_EXPECT(MinElementIndex(arr0) == 0);
			PR_EXPECT(MinElementIndex(arr1) == 1);
			PR_EXPECT(MinElementIndex(arr2) == 2);
			PR_EXPECT(MinElementIndex(arr3) == 3);
			PR_EXPECT(MinElementIndex(arr4) == 4);

			float arr5[] = {1, 2, 3, 4, 5};
			float arr6[] = {1, 2, 3, 5, 4};
			float arr7[] = {2, 3, 5, 1, 4};
			float arr8[] = {2, 5, 3, 4, 1};
			float arr9[] = {5, 2, 3, 4, 1};
			PR_EXPECT(MaxElementIndex(arr5) == 4);
			PR_EXPECT(MaxElementIndex(arr6) == 3);
			PR_EXPECT(MaxElementIndex(arr7) == 2);
			PR_EXPECT(MaxElementIndex(arr8) == 1);
			PR_EXPECT(MaxElementIndex(arr9) == 0);
		}
		PRUnitTestMethod(Trunc)
		{
			PR_EXPECT(Trunc(1.9f) == 1.0f);
			PR_EXPECT(Trunc(1.9f, ETruncType::ToNearest) == 2.0f);
			PR_EXPECT(Trunc(10000000000000.9) == 10000000000000.0);
		}
		PRUnitTestMethod(Dot)
		{
			#if 0 // move
			v3 arr0(1, 2, 3);
			v3 arr1(2, 3, 4);
			iv2 arr2(1, 2);
			iv2 arr3(3, 4);
			quat arr4(4, 3, 2, 1);
			quat arr5(1, 2, 3, 4);
			PR_EXPECT(FEql(Dot(arr0, arr1), 20));
			PR_EXPECT(Dot(arr2, arr3) == 11);
			PR_EXPECT(Dot(arr4, arr5) == 20);
			#endif
		}
		PRUnitTestMethod(CosAngle)
		{
			PR_EXPECT(FEql(CosAngle(1.0, 1.0, maths::root2) - Cos(DegreesToRadians(90.0)), 0.0));
			PR_EXPECT(FEql(Angle(1.0, 1.0, maths::root2), DegreesToRadians(90.0)));
			PR_EXPECT(FEql(Length(1.0f, 1.0f, DegreesToRadians(90.0f)), maths::root2f));
		}
		PRUnitTestMethod(Fraction)
		{
			PR_EXPECT(FEql(Frac<float>(-5, 2, 5), 7.0f / 10.0f));
		}
		PRUnitTestMethod(CubeRoot)
		{
			{// 32bit
				auto a = 1.23456789123456789f;
				auto b = Cubert(a * a * a);
				PR_EXPECT(FEqlRelative(a, b, 0.000001f));
			}
			{// 64bit
				auto a = 1.23456789123456789;
				auto b = Cubert(a * a * a);
				PR_EXPECT(FEqlRelative(a, b, 0.000000000001));
			}
		}
		PRUnitTestMethod(SqrtRoot)
		{
			PR_EXPECT(Sqrt(64.0) == 8.0);
			static_assert(ISqrt(64) == 8);
			static_assert(ISqrt(4294836225) == 65535);
			static_assert(ISqrt(10000000000000000000LL) == 3162277660LL);
			static_assert(ISqrt(18446744065119617025LL) == 4294967295LL);
		}
		PRUnitTestMethod(ArithmeticSequence)
		{
			ArithmeticSequence a(2, 5);
			PR_EXPECT(a() == 2);
			PR_EXPECT(a() == 7);
			PR_EXPECT(a() == 12);
			PR_EXPECT(a() == 17);

			PR_EXPECT(ArithmeticSum(0, 2, 4) == 20);
			PR_EXPECT(ArithmeticSum(4, 2, 2) == 18);
			PR_EXPECT(ArithmeticSum(1, 2, 0) == 1);
			PR_EXPECT(ArithmeticSum(1, 2, 5) == 36);
		}
		PRUnitTestMethod(GeometricSequence)
		{
			GeometricSequence g(2, 5);
			PR_EXPECT(g() == 2);
			PR_EXPECT(g() == 10);
			PR_EXPECT(g() == 50);
			PR_EXPECT(g() == 250);

			PR_EXPECT(GeometricSum(1, 2, 4) == 31);
			PR_EXPECT(GeometricSum(4, 2, 2) == 28);
			PR_EXPECT(GeometricSum(1, 3, 0) == 1);
			PR_EXPECT(GeometricSum(1, 3, 5) == 364);
		}
	};
}
#endif
