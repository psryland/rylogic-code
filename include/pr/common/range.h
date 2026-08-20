//************************************************************************
// Range
//  Copyright (c) Rylogic Ltd 2011
//************************************************************************
#pragma once
#include <limits>
#include <concepts>
#include <type_traits>
#include <algorithm>
#include <cassert>

namespace pr
{
	template <typename T> concept Rangeable = requires(T x)
	{
		{ x + (x - x) / 2 };
		{ x < x || x > x || x <= x || x >= x || x == x || x != x };
	};

	// A range representation (intended for numeric types only, might partially work for iterators)
	// Assume integral by default (so that iterators etc work)
	template <Rangeable T = int>
	struct Range
	{
		using value_type = T;
		using difference_type = decltype(std::declval<T>() - std::declval<T>());
		static bool const is_integral = std::is_integral_v<difference_type>;
		inline static difference_type const integral_one = is_integral * difference_type{1};

		T m_beg; // The first in the range
		T m_end; // One past the last in the range

		// The default empty range
		static constexpr Range Zero()
		{
			return Range(T{}, T{});
		}

		// An invalid range. Used as an initialiser when finding a bounding range
		static constexpr Range Reset()
		{
			return Range{std::numeric_limits<T>::max(), std::numeric_limits<T>::lowest()};
		}

		// A range containing the maximum interval
		static constexpr Range Max()
		{
			return Range{std::numeric_limits<T>::lowest(), std::numeric_limits<T>::max()};
		}

		// Construct a range
		constexpr Range() = default;
		constexpr Range(T beg, T end)
			:m_beg(beg)
			,m_end(end)
		{}

		// True if this is an empty range
		constexpr bool empty() const
		{
			return m_beg == m_end;
		}

		// begin/end range support
		constexpr T begin() const
		{
			return m_beg;
		}
		T& begin()
		{
			return m_beg;
		}
		constexpr T end() const
		{
			return m_end;
		}
		T& end()
		{
			return m_end;
		}

		// The number of elements in or length of the range
		constexpr difference_type size() const
		{
			return m_end - m_beg;
		}

		// Set the range
		constexpr void set(T begin, T end)
		{
			m_beg = begin;
			m_end = end;
		}

		// Set the number of elements in or length of the range
		constexpr void resize(difference_type size)
		{
			m_end = m_beg + size;
		}

		// These are ambiguous, do they return a copy or modify themselves? Use the global functions
		//// Move the range
		//template <typename U> constexpr Range shift(U offset)
		//{
		//	m_beg = static_cast<T>(m_beg + offset);
		//	m_end = static_cast<T>(m_end + offset);
		//	return *this;
		//}

		//// Move the range
		//template <typename U> constexpr Range scale(U numer, U denom)
		//{
		//	m_beg = static_cast<T>((m_beg * numer) / denum);
		//	m_end = static_cast<T>((m_end * numer) / denum);
		//	return *this;
		//}

		// Return the midpoint of the range
		constexpr T mid() const
		{
			return m_beg + (m_end - m_beg)/2;
		}

		// Returns the last value to be considered within the range
		T last() const
		{
			assert(m_beg <= m_end && "range is invalid");
			assert(!is_integral || !empty() && "range is empty");
			return static_cast<T>(m_end - integral_one);
		}

		// True if 'value' is within this range
		template <Rangeable U> bool contains(U rhs) const
		{
			assert(size() >= 0 && "range is invalid");
			return is_integral
				? rhs >= m_beg && rhs < m_end
				: rhs >= m_beg && rhs <= m_end;
		}

		// True if 'rhs' is entirely within this range
		template <Rangeable U> bool contains(Range<U> rhs) const
		{
			assert(size() >= 0 && "range is invalid");
			assert(rhs.size() >= 0 && "range is invalid");
			return contains(rhs.m_beg) && rhs.m_end <= m_end;
		}

		// Returns true if this range and 'rhs' overlap
		template <Rangeable U> bool intersects(Range<U> rhs) const
		{
			assert(size() >= 0 && "range is invalid");
			assert(rhs.size() >= 0 && "rhs range is invalid");
			return m_beg < rhs.m_end && rhs.m_beg < m_end;
		}

		// Grows the range to include 'rhs' and passes 'rhs' through.
		template <Rangeable U> U grow(U rhs)
		{
			// Returning 'rhs' allows inline range measuring. e.g.  auto x = range_of_x.grow(get_x())
			if (rhs <  m_beg) { m_beg = static_cast<T>(rhs); }
			if (rhs >= m_end) { m_end = static_cast<T>(rhs + integral_one); }
			return rhs;
		}

		// Grows the range to include 'rhs' and passes 'rhs' through.
		template <Rangeable U> Range<U> grow(Range<U> rhs)
		{
			// Returning 'rhs' allows inline range measuring. e.g.  auto x = range_of_x.grow(get_x())
			// Don't treat !rhs.valid() as an error, it's the only way to grow with a no-op range
			if (rhs.size() < 0) return rhs;
			if (rhs.m_beg <  m_beg) { m_beg = rhs.m_beg; }
			if (rhs.m_end >= m_end) { m_end = rhs.m_end; }
			return rhs;
		}

		// Implicit conversion to a Range<U> if T is convertible to U
		template <Rangeable U, typename = std::enable_if_t<std::is_convertible_v<T,U>>>
		operator Range<U>()
		{
			return Range<U>(m_beg, m_end);
		}

		// Ranged for helper
		auto enumerate() const
		{
			struct Iter
			{
				T value;
				T operator*() const { return value; }
				Iter& operator++() { ++value; return *this; }
				bool operator == (Iter const& rhs) const { return value == rhs.value; }
			};
			struct R
			{
				Range const* m_this;
				Iter begin() const { return Iter{ m_this->m_beg }; }
				Iter end() const { return Iter{ m_this->m_end }; }
			};
			return R{ this };
		}
	};

	// Operators
	template <Rangeable T, Rangeable U> bool operator == (Range<T> const& lhs, Range<U> const& rhs)
	{
		return lhs.m_beg == rhs.m_beg && lhs.m_end == rhs.m_end;
	}
	template <Rangeable T, Rangeable U>  bool operator != (Range<T> const& lhs, Range<U> const& rhs)
	{
		return !(lhs == rhs);
	}

	// Returns true if 'rhs' is with 'range'
	template <Rangeable T, Rangeable U> inline [[nodiscard]] bool IsWithin(Range<T> const& range, U rhs)
	{
		return range.contains(rhs);
	}

	// Returns true if 'sub_range' is entirely within 'range'
	template <Rangeable T, Rangeable U> inline [[nodiscard]] bool IsWithin(Range<T> const& range, Range<U> const& sub_range)
	{
		return range.contains(sub_range);
	}

	// Returns true if the ranges 'lhs' and 'rhs' overlap
	template <Rangeable T, Rangeable U> inline [[nodiscard]] bool Intersects(Range<T> const& lhs, Range<U> const& rhs)
	{
		return lhs.intersects(rhs);
	}

	// Returns the intersection of 'lhs' with 'rhs'. If there is no intersection, returns [b,b) or [e,e) (from the lhs range). Note: this means Intersect(a,b) != Intersect(b,a)
	template <Rangeable T, Rangeable U> [[nodiscard]] Range<T> Intersect(Range<T> const& lhs, Range<U> const& rhs)
	{
		assert(lhs.size() >= 0 && "lhs range is invalid");
		assert(rhs.size() >= 0 && "rhs range is invalid");
		if (rhs.m_end <= lhs.m_beg) return Range<T>{lhs.m_beg, lhs.m_beg};
		if (rhs.m_beg >= lhs.m_end) return Range<T>{lhs.m_end, lhs.m_end};
		return Range<T>{std::max(lhs.m_beg, rhs.m_beg), std::min(lhs.m_end, rhs.m_end)};
	}

	// Expand 'range' if necessary to include 'rhs'
	template <Rangeable T, Rangeable U> [[nodiscard]] inline Range<T> Union(Range<T> const& range, U rhs)
	{
		auto r = range;
		r.grow(rhs);
		return r;
	}

	// Expand 'range' to include 'rhs' if necessary
	template <Rangeable T, Rangeable U> [[nodiscard]] inline Range<T> Union(Range<T> const& range, Range<U> const& rhs)
	{
		auto r = range;
		r.grow(rhs);
		return r;
	}

	// Expand 'range' if necessary to include 'rhs'. Returns 'rhs'
	template <Rangeable T, Rangeable U> inline U Grow(Range<T>& range, U rhs)
	{
		return range.grow(rhs);
	}

	// Expand 'range' if necessary to include 'rhs'. Returns 'rhs'
	// Returned by value because 'Range<U>::grow(Range<U>)' returns its by-value parameter,
	// so binding a reference to it here would dangle as soon as this function returns.
	template <Rangeable T, Rangeable U> inline Range<U> Grow(Range<T>& range, Range<U> const& rhs)
	{
		return range.grow(rhs);
	}

	// Clamp 'value' to within 'range'
	template <Rangeable T, Rangeable U> [[nodiscard]] inline T Clamp(T value, Range<U> const& range)
	{
		return std::clamp<T>(value, range.m_beg, range.m_end - Range<U>::integral_one);
	}

	// Move the range
	template <Rangeable T, Rangeable U> [[nodiscard]] constexpr Range<T> Shift(Range<T> range, U offset)
	{
		return Range<T>(
			static_cast<T>(range.m_beg + offset),
			static_cast<T>(range.m_end + offset));
	}

	// Scale the range
	template <Rangeable T, Rangeable U> [[nodiscard]] constexpr Range<T> Scale(Range<T> range, U numer, U denom)
	{
		return Range<T>(
			static_cast<T>((range.m_beg * numer) / denom),
			static_cast<T>((range.m_end * numer) / denom));
	}
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
namespace pr::common
{
	PRUnitTestClass(RangeTests)
	{
		static_assert(Range<int>::is_integral == true);
		static_assert(Range<float>::is_integral == false);
		static_assert(Range<char*>::is_integral == true);

		PRUnitTestMethod(General, Quick)
		{
			using IRange = pr::Range<int>;
			IRange r0(0,5);
			IRange r1(5,10);
			IRange r2(3,7);
			IRange r3(0,10);

			PR_EXPECT(!r0.empty());
			PR_EXPECT(r0.size() == 5);

			PR_EXPECT(!IsWithin(r0, -1));
			PR_EXPECT(IsWithin(r0, 0));
			PR_EXPECT(IsWithin(r0, 4));
			PR_EXPECT(!IsWithin(r0, 5));
			PR_EXPECT(!IsWithin(r0, 6));

			PR_EXPECT(IsWithin(r3, r0) == true);
			PR_EXPECT(IsWithin(r3, r1) == true);
			PR_EXPECT(IsWithin(r3, r2) == true);
			PR_EXPECT(IsWithin(r2, r0) == false);
			PR_EXPECT(IsWithin(r2, r1) == false);
			PR_EXPECT(IsWithin(r2, r3) == false);
			PR_EXPECT(IsWithin(r1, r0) == false);
			PR_EXPECT(IsWithin(r0, r1) == false);

			PR_EXPECT(Intersects(r3, r0) == true);
			PR_EXPECT(Intersects(r3, r1) == true);
			PR_EXPECT(Intersects(r3, r2) == true);
			PR_EXPECT(Intersects(r2, r0) == true);
			PR_EXPECT(Intersects(r2, r1) == true);
			PR_EXPECT(Intersects(r2, r3) == true);
			PR_EXPECT(!Intersects(r1, r0));
			PR_EXPECT(!Intersects(r0, r1));

			r0 = Shift(r0, +3);
			r1 = Shift(r1, -2);
			PR_EXPECT(r0 == r1);

			PR_EXPECT(r3.mid() == r2.mid());

			r0 = Shift(r0, -3);
			r0.resize(3);
			PR_EXPECT(r0.size() == 3);

			IRange r4 = IRange::Reset();
			Grow(r4, 4);
			PR_EXPECT(4 == r4.m_beg);
			PR_EXPECT(5 == r4.m_end);
			PR_EXPECT(1 == r4.size());
			PR_EXPECT(IsWithin(r4, 4));

			// 'Grow(Range<T>&, Range<U> const&)' returns a value, not a reference, because
			// 'Range<U>::grow(Range<U>)' returns its by-value parameter. Read the returned
			// range through a copy (not a reference) to confirm this overload is safe to use.
			IRange r5 = IRange::Reset();
			IRange r6(4, 8);
			IRange returned = Grow(r5, r6);
			PR_EXPECT(4 == r5.m_beg);
			PR_EXPECT(8 == r5.m_end);
			PR_EXPECT(returned == r6);
		}
		PRUnitTestMethod(IterRange, Quick)
		{
			using Vec = std::vector<int>;
			using IRange = pr::Range<Vec::const_iterator>;
			Vec vec; for (int i = 0; i != 10; ++i) vec.push_back(i);

			IRange r0(vec.begin(),vec.begin()+5);
			IRange r1(vec.begin()+5,vec.end());
			IRange r2(vec.begin()+3,vec.begin()+7);
			IRange r3(vec.begin(),vec.end());

			PR_EXPECT(r0.empty() == false);
			PR_EXPECT(r0.size() == 5);

			PR_EXPECT(IsWithin(r0, vec.begin()) == true);
			PR_EXPECT(IsWithin(r0, vec.begin() + 4) == true);
			PR_EXPECT(IsWithin(r0, vec.begin() + 5) == false);
			PR_EXPECT(IsWithin(r0, vec.end()) == false);

			PR_EXPECT(IsWithin(r3, r0) == true);
			PR_EXPECT(IsWithin(r3, r1) == true);
			PR_EXPECT(IsWithin(r3, r2) == true);
			PR_EXPECT(IsWithin(r2, r0) == false);
			PR_EXPECT(IsWithin(r2, r1) == false);
			PR_EXPECT(IsWithin(r2, r3) == false);
			PR_EXPECT(IsWithin(r1, r0) == false);
			PR_EXPECT(IsWithin(r0, r1) == false);

			PR_EXPECT(Intersects(r3, r0) == true);
			PR_EXPECT(Intersects(r3, r1) == true);
			PR_EXPECT(Intersects(r3, r2) == true);
			PR_EXPECT(Intersects(r2, r0) == true);
			PR_EXPECT(Intersects(r2, r1) == true);
			PR_EXPECT(Intersects(r2, r3) == true);
			PR_EXPECT(Intersects(r1, r0) == false);
			PR_EXPECT(Intersects(r0, r1) == false);

			r0 = Shift(r0, +3);
			r1 = Shift(r1, -2);
			PR_EXPECT(r0 == r1);

			PR_EXPECT(r3.mid() == r2.mid());

			r0 = Shift(r0, -3);
			r0.resize(3);
			PR_EXPECT(r0.size() == 3);

			IRange r4(vec.end(),vec.begin());
			Grow(r4, vec.begin() + 4);
			PR_EXPECT(vec.begin() + 4 == r4.m_beg);
			PR_EXPECT(vec.begin() + 5 == r4.m_end);
			PR_EXPECT(1 == r4.size());
			PR_EXPECT(IsWithin(r4, vec.begin() + 4));
		}
		PRUnitTestMethod(FloatingPointRange, Quick)
		{
			using FRange = pr::Range<float>;
			FRange r0(0.0f, 5.0f);
			FRange r1(5.0f, 10.0f);
			FRange r2(3.0f, 7.0f);
			FRange r3(0.0f, 10.0f);

			PR_EXPECT(!r0.empty());
			PR_EXPECT(r0.size() == 5.0f);

			PR_EXPECT(!IsWithin(r0, -1.0f));
			PR_EXPECT(IsWithin(r0, 0.0f));
			PR_EXPECT(IsWithin(r0, 4.0f));
			PR_EXPECT(IsWithin(r0, 5.0f));
			PR_EXPECT(!IsWithin(r0, 6.0f));

			PR_EXPECT(IsWithin(r3, r0) == true);
			PR_EXPECT(IsWithin(r3, r1) == true);
			PR_EXPECT(IsWithin(r3, r2) == true);
			PR_EXPECT(IsWithin(r2, r0) == false);
			PR_EXPECT(IsWithin(r2, r1) == false);
			PR_EXPECT(IsWithin(r2, r3) == false);
			PR_EXPECT(IsWithin(r1, r0) == false);
			PR_EXPECT(IsWithin(r0, r1) == false);

			PR_EXPECT(Intersects(r3, r0) == true);
			PR_EXPECT(Intersects(r3, r1) == true);
			PR_EXPECT(Intersects(r3, r2) == true);
			PR_EXPECT(Intersects(r2, r0) == true);
			PR_EXPECT(Intersects(r2, r1) == true);
			PR_EXPECT(Intersects(r2, r3) == true);
			PR_EXPECT(!Intersects(r1, r0));
			PR_EXPECT(!Intersects(r0, r1));

			r0 = Shift(r0, +3.0f);
			r1 = Shift(r1, -2.0f);
			PR_EXPECT(r0 == r1);

			PR_EXPECT(r3.mid() == r2.mid());

			r0 = Shift(r0, -3.0f);
			r0.resize(3.0f);
			PR_EXPECT(r0.size() == 3.0f);

			FRange r4 = FRange::Reset();
			Grow(r4, 4.0f);
			PR_EXPECT(4.0f == r4.m_beg);
			PR_EXPECT(4.0f == r4.m_end);
			PR_EXPECT(0.0f == r4.size());
			PR_EXPECT(IsWithin(r4, 4.0f));
		}
		PRUnitTestMethod(ImplicitConversion, Quick)
		{
			Range<uint16_t> r0(0, 65535);
			Range<uint32_t> r1;
			r1 = r0;
			PR_EXPECT(r1 == Range<uint32_t>(0, 65535));
		}
	};
}
#endif
