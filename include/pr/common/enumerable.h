//******************************************
// Enumerable
//  Copyright (c) Oct 2003 Paul Ryland
//******************************************
// A base class for creating C#-style Enumerables
// use:
//   using Things = Enumerable<std::vector<Thing>::iterator>;
//
//  struct Foo
//  {
//     using ThingCont = std::vector<Thing>;
//     ThingCont m_things;
//
//     Enumerable<ThingCont> Things() const { return MakeEnumerable(m_things); }
//  };
//
//  for (auto& i : foo.Things())
//    i.blah();
//

#pragma once
#include <iterator>

namespace pr
{
	// An iterator range
	template <typename TIter> struct Enumerable
	{
		TIter m_beg, m_end;
		Enumerable(TIter beg, TIter end)
			:m_beg(beg)
			,m_end(end)
		{
		}
		TIter const begin() const
		{
			return m_beg;
		}
		TIter begin()
		{
			return m_beg;
		}
		TIter const end() const
		{
			return m_end;
		}
		TIter end()
		{
			return m_end;
		}
	};

	// Helper for creating a filtering iterator
	template <typename TIter, typename TPred> struct FilterIter
	{
		using iterator_category = std::forward_iterator_tag;
		using value_type = typename std::iterator_traits<TIter>::value_type;

		TIter m_iter, m_end;
		TPred m_pred;

		FilterIter(TIter iter, TIter end, TPred pred)
			:m_iter(iter)
			, m_end(end)
			, m_pred(pred)
		{
			find_next_valid();
		}
		FilterIter& operator ++()
		{
			++m_iter;
			find_next_valid();
			return *this;
		}
		value_type& operator *() const
		{
			return *m_iter;
		}

		friend bool operator == (FilterIter const& lhs, FilterIter const& rhs)
		{
			return lhs.m_iter == rhs.m_iter;
		}
		friend bool operator != (FilterIter const& lhs, FilterIter const& rhs)
		{
			return lhs.m_iter != rhs.m_iter;
		}

	private:

		void find_next_valid()
		{
			for (; m_iter != m_end && !m_pred(*m_iter); ++m_iter) {}
		}
	};

	// Construct an enumerable from a predicate
	template <typename TCont, typename TPred, typename TFilter = FilterIter<typename TCont::iterator, TPred>>
	inline Enumerable<TFilter> MakeEnumerable(TCont& cont, TPred pred)
	{
		auto b = std::begin(cont);
		auto e = std::end(cont);
		return Enumerable<TFilter>
		{
			TFilter{b, e, pred},
			TFilter{e, e, pred}
		};
	}
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
#include <vector>
namespace pr::common
{
	namespace unittests::enumerable
	{
		struct Foo
		{
			using Cont = std::vector<int>;
			Cont m_int;

			Foo()
			{
				m_int.push_back(1);
				m_int.push_back(2);
				m_int.push_back(3);
			}
			Enumerable<FilterIter<Cont::iterator, bool(*)(int)>> OddInts()
			{
				bool (*pred)(int) = [](int item) { return (item % 2) == 1; };
				return MakeEnumerable(m_int, pred);
			}
		};
	}
	PRUnitTest(EnumerableTests, Quick)
	{
		using namespace unittests::enumerable;

		{
			Foo foo;
			for (auto& i : foo.OddInts())
				i *= 10;
			PR_EXPECT(foo.m_int[0] == 10);
			PR_EXPECT(foo.m_int[1] == 2);
			PR_EXPECT(foo.m_int[2] == 30);
		}
		{
			Foo foo;
			for (auto& i : MakeEnumerable(foo.m_int, [](int item){ return (item % 2) == 0; }))
				i *= -10;
				
			PR_EXPECT(foo.m_int[0] == 1);
			PR_EXPECT(foo.m_int[1] == -20);
			PR_EXPECT(foo.m_int[2] == 3);
		}
	}
}
#endif
