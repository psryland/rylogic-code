//*********************************************
// Vector Map
//  Copyright (c) Rylogic Ltd 2011
//*********************************************
// An stl-like map implemented using a contiguous container.
// Note: it is not a replacement for std::map because it has
// vector-like invalidation rules.
#pragma once
#include <algorithm>
#include <functional>
#include <initializer_list>
#include <stdexcept>
#include <utility>
#include <vector>

namespace pr
{
	// Implements a std::map like interface using a contiguous container
	// *Careful* invalidation rules are the same as for 'Vec', not std::map.
	template <typename Key, typename Type, typename Vec = std::vector<std::pair<Key, Type>>, typename Compare = std::less<Key>>
	struct vector_map
	{
		using this_type       = vector_map<Key, Type, Vec, Compare>;
		using container_type  = Vec;
		using cont_type       = Vec;
		using key_type        = Key;
		using mapped_type     = Type;
		using value_type      = typename Vec::value_type;
		using size_type       = typename Vec::size_type;
		using difference_type = typename Vec::difference_type;
		using iterator        = typename Vec::iterator;
		using const_iterator  = typename Vec::const_iterator;
		using miter           = iterator;
		using citer           = const_iterator;
		using key_compare     = Compare;

		Vec m_cont;
		Compare m_comp;

		struct value_compare
		{
			Compare m_comp;

			bool operator()(value_type const& lhs, value_type const& rhs) const
			{
				return m_comp(lhs.first, rhs.first);
			}
			bool operator()(value_type const& lhs, key_type const& rhs) const
			{
				return m_comp(lhs.first, rhs);
			}
			bool operator()(key_type const& lhs, value_type const& rhs) const
			{
				return m_comp(lhs, rhs.first);
			}
		};

		vector_map()
			: m_cont()
			, m_comp()
		{}
		explicit vector_map(Compare comp)
			: m_cont()
			, m_comp(std::move(comp))
		{}
		vector_map(std::initializer_list<value_type> init, Compare comp = Compare())
			: vector_map(std::move(comp))
		{
			insert(init.begin(), init.end());
		}
		template <typename Iter>
		vector_map(Iter first, Iter last, Compare comp = Compare())
			: vector_map(std::move(comp))
		{
			insert(first, last);
		}

		bool empty() const
		{
			return m_cont.empty();
		}
		size_type size() const
		{
			return m_cont.size();
		}
		const_iterator begin() const
		{
			return m_cont.begin();
		}
		iterator begin()
		{
			return m_cont.begin();
		}
		const_iterator end() const
		{
			return m_cont.end();
		}
		iterator end()
		{
			return m_cont.end();
		}
		const_iterator cbegin() const
		{
			return begin();
		}
		const_iterator cend() const
		{
			return end();
		}
		Compare key_comp() const
		{
			return m_comp;
		}
		value_compare value_comp() const
		{
			return value_compare{ m_comp };
		}
		void clear()
		{
			m_cont.clear();
		}

		// const reference to value at key
		Type const& at(Key const& key) const
		{
			auto iter = Iter(key);
			if (iter == end() || !Equivalent(iter->first, key))
				throw std::out_of_range("key not found");

			return iter->second;
		}

		// mutable reference to value at key
		Type& at(Key const& key)
		{
			auto iter = Iter(key);
			if (iter == end() || !Equivalent(iter->first, key))
				throw std::out_of_range("key not found");

			return iter->second;
		}

		// True if 'key' is in the map
		bool contains(Key const& key) const
		{
			return find(key) != end();
		}

		// const iterator to match for 'key' or end()
		const_iterator find(Key const& key) const
		{
			auto iter = Iter(key);
			if (iter == end() || !Equivalent(iter->first, key))
				return end();

			return iter;
		}

		// iterator to match for 'key' or end()
		iterator find(Key const& key)
		{
			auto iter = Iter(key);
			if (iter == end() || !Equivalent(iter->first, key))
				return end();

			return iter;
		}

		// const reference to value at 'key'. Careful with this reference, it may be invalidated by other inserts
		Type const& operator[](Key const& key) const
		{
			return at(key);
		}

		// mutable reference to value at 'key'. Careful with this reference, it may be invalidated by other inserts
		Type& operator[](Key const& key)
		{
			return try_emplace(key).first->second;
		}
		Type& operator[](Key&& key)
		{
			return try_emplace(std::move(key)).first->second;
		}

		std::pair<iterator, bool> insert(value_type const& value)
		{
			auto iter = Iter(value.first);
			if (iter != end() && Equivalent(iter->first, value.first))
				return { iter, false };

			return { m_cont.insert(iter, value), true };
		}
		std::pair<iterator, bool> insert(value_type&& value)
		{
			auto iter = Iter(value.first);
			if (iter != end() && Equivalent(iter->first, value.first))
				return { iter, false };

			return { m_cont.insert(iter, std::move(value)), true };
		}
		template <typename Iter>
		void insert(Iter first, Iter last)
		{
			for (; first != last; ++first)
				insert(*first);
		}
		template <typename... Args>
		std::pair<iterator, bool> emplace(Args&&... args)
		{
			return insert(value_type(std::forward<Args>(args)...));
		}
		template <typename K, typename... Args>
		std::pair<iterator, bool> try_emplace(K&& key, Args&&... args)
		{
			auto iter = Iter(key);
			if (iter != end() && Equivalent(iter->first, key))
				return { iter, false };

			auto value = value_type(std::forward<K>(key), mapped_type(std::forward<Args>(args)...));
			return { m_cont.insert(iter, std::move(value)), true };
		}
		iterator erase(const_iterator iter)
		{
			return m_cont.erase(iter);
		}
		size_type erase(Key const& key)
		{
			auto iter = find(key);
			if (iter == end())
				return 0;

			erase(iter);
			return 1;
		}

	private:
		// return an iterator to where 'key' would be inserted
		const_iterator Iter(Key const& key) const
		{
			return std::lower_bound(begin(), end(), key, value_comp());
		}
		iterator Iter(Key const& key)
		{
			return std::lower_bound(begin(), end(), key, value_comp());
		}
		bool Equivalent(Key const& lhs, Key const& rhs) const
		{
			return !m_comp(lhs, rhs) && !m_comp(rhs, lhs);
		}
	};
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
#include "pr/container/vector.h"
namespace pr::container
{
	PRUnitTestClass(VectorMapTests)
	{
		struct Thing
		{
			int m_id;
			Thing() :m_id() {}
			Thing(int id) :m_id(id) {}
		};

		using fixed_buffer = pr::vector<std::pair<int, Thing>, 10, true>;
		using Map = pr::vector_map<int, Thing, fixed_buffer>;

		static Map MakeMap()
		{
			Map map;
			map[3] = Thing(3);
			map[1] = Thing(1);
			map[9] = Thing(9);
			return map;
		}

		PRUnitTestMethod(LookupAndOrdering)
		{
			auto map = MakeMap();

			PR_EXPECT(!map.empty());
			PR_EXPECT(map.size() == 3U);
			PR_EXPECT(map[9].m_id == 9);
			PR_EXPECT(map[1].m_id == 1);
			PR_EXPECT(map[3].m_id == 3);
			PR_EXPECT(map.at(3).m_id == 3);
			PR_EXPECT(map.contains(1));
			PR_EXPECT(!map.contains(2));
			PR_EXPECT(map.find(2) == map.end());
			PR_EXPECT(map.begin()->first == 1);
			PR_EXPECT((map.begin() + 1)->first == 3);
			PR_EXPECT((map.begin() + 2)->first == 9);
		}

		PRUnitTestMethod(ConstLookupAndAt)
		{
			auto map = MakeMap();
			auto const& cmap = map;

			PR_EXPECT(cmap.at(1).m_id == 1);
			PR_EXPECT(cmap[3].m_id == 3);
			PR_EXPECT(cmap.find(9) != cmap.end());
			PR_EXPECT(cmap.find(4) == cmap.end());
			PR_THROWS(cmap.at(4), std::out_of_range);
			PR_THROWS(cmap[4], std::out_of_range);

			map.at(3) = Thing(30);
			PR_EXPECT(cmap.at(3).m_id == 30);
		}

		PRUnitTestMethod(InsertAndEmplace)
		{
			auto map = MakeMap();

			auto insert = map.insert({ 3, Thing(4) });
			PR_EXPECT(!insert.second);
			PR_EXPECT(insert.first->second.m_id == 3);

			insert = map.insert({ 2, Thing(2) });
			PR_EXPECT(insert.second);
			PR_EXPECT(map.size() == 4U);
			PR_EXPECT(map.begin()->first == 1);
			PR_EXPECT((map.begin() + 1)->first == 2);
			PR_EXPECT((map.begin() + 2)->first == 3);
			PR_EXPECT((map.begin() + 3)->first == 9);

			auto emplaced = map.try_emplace(4, 4);
			PR_EXPECT(emplaced.second);
			PR_EXPECT(emplaced.first->second.m_id == 4);

			emplaced = map.try_emplace(4, 40);
			PR_EXPECT(!emplaced.second);
			PR_EXPECT(emplaced.first->second.m_id == 4);

			auto inserted = map.emplace(5, Thing(5));
			PR_EXPECT(inserted.second);
			PR_EXPECT(inserted.first->second.m_id == 5);
		}

		PRUnitTestMethod(EraseAndClear)
		{
			auto map = MakeMap();

			PR_EXPECT(map.erase(3) == 1);
			PR_EXPECT(map.erase(3) == 0);
			PR_EXPECT(!map.contains(3));
			PR_EXPECT(map.size() == 2U);

			auto iter = map.find(1);
			PR_EXPECT(iter != map.end());
			iter = map.erase(iter);
			PR_EXPECT(iter == map.begin());
			PR_EXPECT(iter->first == 9);

			map.clear();
			PR_EXPECT(map.empty());
		}

		PRUnitTestMethod(InitializerAndRangeConstruction)
		{
			Map map = {
				{ 5, Thing(5) },
				{ 1, Thing(1) },
				{ 3, Thing(3) },
				{ 1, Thing(10) },
			};

			PR_EXPECT(map.size() == 3U);
			PR_EXPECT(map.begin()->first == 1);
			PR_EXPECT((map.begin() + 1)->first == 3);
			PR_EXPECT((map.begin() + 2)->first == 5);
			PR_EXPECT(map.at(1).m_id == 1);

			std::pair<int, Thing> values[] = {
				{ 4, Thing(4) },
				{ 2, Thing(2) },
				{ 4, Thing(40) },
			};
			Map map2(std::begin(values), std::end(values));
			PR_EXPECT(map2.size() == 2U);
			PR_EXPECT(map2.begin()->first == 2);
			PR_EXPECT((map2.begin() + 1)->first == 4);
			PR_EXPECT(map2.at(4).m_id == 4);
		}

		PRUnitTestMethod(CustomComparator)
		{
			using DescMap = pr::vector_map<int, Thing, fixed_buffer, std::greater<int>>;

			DescMap map;
			map[3] = Thing(3);
			map[1] = Thing(1);
			map[9] = Thing(9);

			PR_EXPECT(map.size() == 3U);
			PR_EXPECT(map.begin()->first == 9);
			PR_EXPECT((map.begin() + 1)->first == 3);
			PR_EXPECT((map.begin() + 2)->first == 1);
			PR_EXPECT(map.contains(3));
			PR_EXPECT(map.find(2) == map.end());

			auto insert = map.insert({ 3, Thing(30) });
			PR_EXPECT(!insert.second);
			PR_EXPECT(insert.first->second.m_id == 3);
		}
	};
}
#endif
