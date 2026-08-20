//*****************************************************************************************
// LinQ
//  Copyright (c) Rylogic Ltd 2016
//*****************************************************************************************
#pragma once

#include <iterator>

namespace pr
{
	struct AlwaysTrue
	{
		template <typename... Args> bool operator ()(Args&&...) const { return true; }
	};
	struct AlwaysFalse
	{
		template <typename... Args> bool operator ()(Args&&...) const { return false; }
	};
	struct Unchanged
	{
		template <typename T> T&& operator()(T&& x) const { return std::forward<T>(x); }
	};

	#define PR_LINQ_TODO 0
	#if PR_LINQ_TODO // Needs work... basically works

	// A helper for providing LinQ-type expressions
	template <typename IterType, typename Pred = AlwaysTrue, typename Adapt = Unchanged>
	struct Linq
	{
		using iter_type         = IterType;
		using value_type        = typename std::iterator_traits<IterType>::value_type;
		using iterator_category = typename std::input_iterator_tag;
		//using difference_type   = std::iterator_traits<iter_type>::difference_type;
		//using pointer           = std::iterator_traits<iter_type>::pointer;
		//using reference         = std::iterator_traits<iter_type>::reference;

		iter_type m_beg;
		iter_type m_end;
		Pred m_pred;
		Adapt m_adapt;

		Linq(iter_type beg, iter_type end, Pred pred = Pred(), Adapt adapt = Adapt())
			:m_beg(beg)
			,m_end(end)
			,m_pred(pred)
			,m_adapt(adapt)
		{
			seek();
		}

		// Seek to the first valid iterator position
		Linq& seek()
		{
			for (; !(m_beg == m_end) && !m_pred(*m_beg); ++m_beg) {}
			return *this;
		}
		bool empty() const
		{
			return m_beg == m_end;
		}

		// Iterator interface
		Linq begin() const
		{
			return *this;
		}
		Linq end() const
		{
			return Linq(m_end, m_end, m_pred, m_adapt);
		}
		value_type operator *() const
		{
			return m_adapt(*m_beg);
		}
		Linq& operator ++()
		{
			++m_beg;
			return seek();
		}
		Linq operator ++(int)
		{
			auto copy = *this;
			return ++*this, copy;
		}
		bool operator == (Linq const& rhs) const
		{
			return m_beg == rhs.m_beg && m_end == rhs.m_end;
		}
		bool operator != (Linq const& rhs) const
		{
			return !(*this == rhs);
		}

		// Return the range as a vector
		std::vector<value_type> to_vector() const
		{
			std::vector<value_type> vec;
			for (auto& v : *this)
				vec.push_back(v);
			return vec;
			//return std::vector<value_type>(begin(), end());
		}

		// Where: [](value_type const& v) { return true; }
		template <typename Pred2> Linq<Linq, Pred2, Adapt> where(Pred2 pred) const
		{
			return Linq<Linq, Pred2, Adapt>(begin(), end(), pred, m_adapt);
		}

		// Select: [](value_type const& v) { return v.other; }
		template <typename Adapt2> auto select(Adapt2 adapt) const -> Linq<Linq, AlwaysTrue, Adapt2>
		{
			return Linq<Linq, AlwaysTrue, Adapt2>(begin(), end(), AlwaysTrue(), adapt);
		}

		// True if 'any' in the linQ expression return true for 'pred'
		template <typename Pred2> bool any(Pred2 pred) const
		{
			// Seeks to the first value where 'pred' is true
			return !where(pred).empty();
		}
	};

	// Create a LinQ wrapper instance from an iterator range
	template <typename IterType>
	inline Linq<IterType> linq(IterType beg, IterType end)
	{
		return Linq<IterType>(beg, end);
	}

	// Create a LinQ wrapper instance from a container
	template <typename TCont>
	inline auto linq(TCont const& cont) -> decltype(linq(std::begin(cont), std::end(cont)))
	{
		return linq(std::begin(cont), std::end(cont));
	}

	#endif
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
#include <vector>
namespace pr::common
{
	PRUnitTest(LinqTests, Quick)
	{
		#if PR_LINQ_TODO
		{ // Simple array
			int cont[] = {0,1,2,3,4,5,6,7,8,9};
			auto expr = linq(cont)
				.where([](int x){ return (x%2) == 1; });

			PR_EXPECT(*expr++ == 1);
			PR_EXPECT(*expr++ == 3);
			PR_EXPECT(*expr++ == 5);
			PR_EXPECT(*expr++ == 7);
			PR_EXPECT(*expr++ == 9);
		}
		{ // Container type
			std::vector<int> cont = {0,1,2,3,4,5,6,7,8,9};
			auto result = linq(cont)
				.where([](int i){ return (i%3) == 0; })
				.to_vector();

			PR_CHECK(result.size(), 4U);
			PR_CHECK(result[0], 0);
			PR_CHECK(result[1], 3);
			PR_CHECK(result[2], 6);
			PR_CHECK(result[3], 9);
		}
		{ // Select
			int cont[] = {0,1,2,3,4,5};
			auto result = linq(cont)
				.where([](int i){ return (i%2) == 0;})
				.select([](int i){ return i + 0.5f; })
				.to_vector();

			PR_CHECK(result.size(), 3U);
			PR_CHECK(result[0], 0.5f);
			PR_CHECK(result[1], 2.5f);
			PR_CHECK(result[2], 4.5f);
		}
		{ // Any
			bool cont[] = {false, false, false, true, false};
			auto result = linq(cont).any([](bool b) { return b; });

			PR_EXPECT(result);
		}
		#endif
	}
}
#endif
