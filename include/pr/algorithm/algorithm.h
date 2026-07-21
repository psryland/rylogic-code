//********************************
// Algorithm
//  Copyright (c) Rylogic Ltd 2006
//********************************
// Helper wrappers to 'std::algorithm'
#pragma once
#include <algorithm>
#include <unordered_set>
#include <unordered_map>
#include <iterator>
#include <set>
#include <map>
#include <vector>
#include <queue>
#include <span>
#include <ranges>
#include <concepts>

namespace pr
{
	// Container traits
	#pragma region Container Traits
	namespace impl
	{
		template <typename T> struct std_container_traits
		{
			using value_type              = typename T::value_type;
			using const_iterator          = typename T::const_iterator;
			using iterator                = typename T::iterator;
			static bool const associative = false;
		};
		template <typename K, typename V> struct map_container_traits
		{
			using value_type = std::pair<K,V>;
			static bool const associative = true;
		};
		template <typename K> struct set_container_traits
		{
			using value_type = K;
			static bool const associative = true;
		};
	}

	template <typename T> struct container_traits
		:impl::std_container_traits<T>
	{};
	template <typename U> struct container_traits<U[]>
	{
		using value_type              = U;
		using const_iterator          = typename value_type const*;
		using iterator                = typename value_type*;
		static bool const associative = false;
	};
	template <typename K, typename V> struct container_traits<std::unordered_map<K,V>>
		:impl::map_container_traits<K,V>
	{};
	template <typename K, typename V> struct container_traits<std::map<K,V>>
		:impl::map_container_traits<K,V>
	{};
	template <typename K> struct container_traits<std::unordered_set<K>>
		:impl::set_container_traits<K>
	{};
	template <typename K> struct container_traits<std::set<K>>
		:impl::set_container_traits<K>
	{};

	static_assert(container_traits<std::vector<int>>::associative == false, "");
	static_assert(container_traits<std::unordered_set<int>>::associative == true, "");
	#pragma endregion

	// Container concept
	template <typename T>
	concept Container = requires(T t)
	{
		{std::begin(t)};
		{std::end(t)};
	};

	// Signed size of an array
	template <typename T, int S> requires (S <= 0x7FFFFFFF)
	constexpr int icountof(T const (&)[S])
	{
		return S;
	}
	template <typename T, int64_t S> requires (S >= 0x80000000LL && S <= 0x7FFFFFFFFFFFFFFFLL)
	constexpr int64_t icountof(T const (&)[S])
	{
		return S;
	}
	template <typename T>
	constexpr int64_t icountof(std::span<T> s)
	{
		return static_cast<int64_t>(s.size());
	}

	// Return the length of a container or array
	template <typename TCont> size_t length(TCont const& cont)
	{
		return cont.size();
	}
	template <typename Type, size_t N> size_t length(Type const (&cont)[N])
	{
		return N;
	}

	// Return the minimum of a set of values
	template <typename Type> inline Type min(Type const& a) { return a; }
	template <typename Type, typename... Args> inline Type min(Type const& a, Args&&... args)
	{
		return std::min(a, pr::min(std::forward<Args>(args)...));
	}

	// Return the maximum of a set of values
	template <typename Type> inline Type max(Type const& a) { return a; }
	template <typename Type, typename... Args> inline Type max(Type const& a, Args&&... args)
	{
		return std::max(a, pr::max(std::forward<Args>(args)...));
	}

	// Returns true if all in 'cont' pass 'pred'
	template <Container TCont, typename Pred> inline bool all(TCont const& cont, Pred pred)
	{
		return std::all_of(std::begin(cont), std::end(cont), pred);
	}

	// Returns true if any in 'cont' pass 'pred'
	template <Container TCont, typename Pred> inline bool any(TCont const& cont, Pred pred)
	{
		return std::any_of(std::begin(cont), std::end(cont), pred);
	}

	// True if 'func' returns true for any element in 'cont'
	template <Container TCont, typename TElem = typename container_traits<TCont>::value_type> inline bool contains(TCont const& cont, TElem const& item)
	{
		auto iter = std::find(std::begin(cont), std::end(cont), item);
		return iter != std::end(cont);
	}
	template <typename TIter, typename TElem = typename std::iterator_traits<TIter>::value_type> inline bool contains(TIter beg, TIter end, TElem const& item)
	{
		auto iter = std::find(beg, end, item);
		return iter != end;
	}
	template <typename TKey, typename TValue> inline bool contains(std::unordered_map<TKey, TValue> const& cont, TKey const& item)
	{
		return cont.count(item) != 0;
	}
	template <typename TKey, typename TValue> inline bool contains(std::map<TKey, TValue> const& cont, TKey const& item)
	{
		return cont.count(item) != 0;
	}
	template <typename TKey> inline bool contains(std::unordered_set<TKey> const& cont, TKey const& item)
	{
		return cont.count(item) != 0;
	}
	template <typename TKey> inline bool contains(std::set<TKey> const& cont, TKey const& item)
	{
		return cont.count(item) != 0;
	}

	// Return the lower bound
	template <Container TCont, typename TValue> inline auto lower_bound(TCont& cont, TValue const& val) -> decltype(std::begin(cont))
	{
		return std::lower_bound(std::begin(cont), std::end(cont), val);
	}
	template <Container TCont, typename TValue, typename TFunc> inline auto lower_bound(TCont& cont, TValue const& val, TFunc pred) -> decltype(std::begin(cont))
	{
		return std::lower_bound(std::begin(cont), std::end(cont), val, pred);
	}

	// Return the upper bound
	template <Container TCont, typename TValue> inline auto upper_bound(TCont& cont, TValue const& val) -> decltype(std::begin(cont))
	{
		return std::upper_bound(std::begin(cont), std::end(cont), val);
	}
	template <Container TCont, typename TValue, typename TFunc> inline auto upper_bound(TCont& cont, TValue const& val, TFunc pred) -> decltype(std::begin(cont))
	{
		return std::upper_bound(std::begin(cont), std::end(cont), val, pred);
	}

	// Returns a pair [lower, upper) for the range equal to 'val'
	template <Container TCont, typename TValue> inline auto equal_range(TCont& cont, TValue const& val) -> decltype(std::make_pair(std::begin(cont), std::begin(cont)))
	{
		return std::equal_range(std::begin(cont), std::end(cont), val);
	}
	template <Container TCont, typename TValue, typename TFunc> inline auto equal_range(TCont& cont, TValue const& val, TFunc pred) -> decltype(std::make_pair(std::begin(cont), std::begin(cont)))
	{
		return std::equal_range(std::begin(cont), std::end(cont), val, pred);
	}

	// Returns a pair [lower, upper) for the range from 'first' to 'last'
	template <Container TCont, typename TValue> inline auto find_bounds(TCont& cont, TValue const& first, TValue const& last) -> decltype(std::make_pair(std::begin(cont), std::begin(cont)))
	{
		assert(first <= last);
		auto lwr = std::lower_bound(std::begin(cont), std::end(cont), first);
		auto upr = std::upper_bound(lwr, std::end(cont), last);
		return std::make_pair(lwr, upr);
	}
	
	// True if 'func' returns true for any element in 'cont'
	template <Container TCont, typename TFunc> inline bool contains_if(TCont const& cont, TFunc pred)
	{
		auto iter = std::find_if(std::begin(cont), std::end(cont), pred);
		return iter != std::end(cont);
	}

	// Return the index of the first occurrence to satisfy 'pred'
	template <Container TCont, typename TFunc> inline int index_if(TCont const& cont, TFunc pred)
	{
		auto iter = std::find_if(std::begin(cont), std::end(cont), pred);
		return static_cast<int>(std::distance(std::begin(cont), iter));
	}

	// Return the index of 'val' in 'cont' or cont.size() if not found
	template <Container TCont, typename TValue> inline int index_of(TCont const& cont, TValue const& val)
	{
		auto iter = std::find(std::begin(cont), std::end(cont), val);
		return static_cast<int>(std::distance(std::begin(cont), iter));
	}

	// Return the first element in 'cont' that matches 'pred' or nullptr
	template <Container TCont, typename TValue> inline auto find(TCont& cont, TValue const& val) -> decltype(std::begin(cont))
	{
		return std::find(std::begin(cont), std::end(cont), val);
	}

	// Returns the first element in 'cont' that matches 'pred' or end iterator
	template <Container TCont, typename Pred> inline auto find_if(TCont& cont, Pred pred) -> decltype(std::begin(cont))
	{
		return std::find_if(std::begin(cont), std::end(cont), pred);
	}

	// Return the first element in 'cont' that matches 'pred' or throw
	template <Container TCont, typename Pred> inline auto get_if(TCont& cont, Pred pred) -> decltype(*std::begin(cont))
	{
		auto iter = find_if(cont, pred);
		return iter != std::end(cont) ? *iter : throw std::runtime_error("get_if() - no match found");
	}

	// Return the first pointer like argument that isn't null
	template <typename T, typename... Args> inline T first_not_null(T a, Args&&... args)
	{
		return a == nullptr ? first_not_null(std::forward<Args>(args)...) : a;
	}
	inline nullptr_t first_not_null()
	{
		return nullptr;
	}

	// Return the first element in 'cont' that matches 'pred' or return a default element instance
	template <Container TCont, typename Pred, typename Elem = typename container_traits<TCont>::value_type> inline Elem first_or_default(TCont& cont, Pred pred, Elem def = Elem())
	{
		auto iter = find_if(cont, pred);
		return iter != std::end(cont) ? *iter : def;
	}

	// Return the number of elements in 'cont' that match 'pred'
	template <Container TCont, typename Pred> inline TCont::difference_type count_if(TCont const& cont, Pred pred)
	{
		return std::count_if(std::begin(cont), std::end(cont), pred);
	}

	// Insert 'val' into 'cont' if there is no element in 'cont' equal to 'val'
	// 'cont' is assumed to be ordered. Returns true if 'val' was added to 'cont'
	template <Container TCont, typename TValue, typename OrderPred> inline bool insert_unique(TCont& cont, TValue const& val, OrderPred order_pred)
	{
		// '*iter' will be >= 'val'. So if 'val' is not < '*iter' it must be equal
		auto iter = std::lower_bound(std::begin(cont), std::end(cont), val, order_pred);
		if (iter != std::end(cont) && !order_pred(val,*iter)) return false;
		cont.insert(iter, val);
		return true;
	}
	template <Container TCont, typename TValue> inline bool insert_unique(TCont& cont, TValue const& val)
	{
		// '*iter' will be >= 'val'. So if 'val' is not < '*iter' it must be equal
		auto iter = std::lower_bound(std::begin(cont), std::end(cont), val);
		if (iter != std::end(cont) && *iter == val) return false;
		cont.insert(iter, val);
		return true;
	}

	// Insert 'val' into 'cont' in order
	template <Container TCont, typename Value, typename OrderPred> inline void insert_ordered(TCont& cont, Value const& val, OrderPred order_pred)
	{
		auto iter = std::lower_bound(std::begin(cont), std::end(cont), val, order_pred);
		cont.insert(iter, val);
	}
	template <Container TCont, typename Value> inline void insert_ordered(TCont& cont, Value const& val)
	{
		auto iter = std::lower_bound(std::begin(cont), std::end(cont), val);
		cont.insert(iter, val);
	}

	// Erase 'where' from 'cont'
	template <Container TCont> inline void erase_at(TCont& cont, typename container_traits<TCont>::const_iterator iter)
	{
		if (iter == std::end(cont)) return;
		cont.erase(iter);
	}
	template <Container TCont> inline void erase_unstable(TCont& cont, typename container_traits<TCont>::iterator where)
	{
		if (where == std::end(cont)) return;
		*where = std::move(cont.back());
		cont.pop_back();
	}

	// Erase the first instance of 'value' from 'cont'
	template <Container TCont> inline void erase_stable(TCont& cont, typename container_traits<TCont>::value_type const& value)
	{
		auto iter = std::find(std::begin(cont), std::end(cont), value);
		erase_at(cont, iter);
	}
	template <Container TCont> inline void erase_unstable(TCont& cont, typename container_traits<TCont>::value_type const& value)
	{
		auto iter = std::find(std::begin(cont), std::end(cont), value);
		erase_unstable(cont, iter);
	}

	// Erase the first match to 'pred' from 'cont'
	template <Container TCont, typename Pred> inline void erase_first(TCont& cont, Pred pred)
	{
		auto iter = std::find_if(std::begin(cont), std::end(cont), pred);
		erase_at(cont, iter);
	}
	template <Container TCont, typename Pred> inline void erase_first_unstable(TCont& cont, Pred pred)
	{
		auto iter = std::find_if(std::begin(cont), std::end(cont), pred);
		erase_unstable(cont, iter);
	}

	// Erase all elements from 'cont' that match 'pred'
	template <Container TCont, typename Pred> inline size_t erase_if(TCont& cont, Pred pred)
	{
		size_t num = 0;
		if constexpr (container_traits<TCont>::associative)
		{
			// 'std::remove_if' does not work on associative containers because they cannot be reordered.
			for (auto i = std::begin(cont); i != std::end(cont);)
			{
				if (!pred(*i))
				{
					++i;
				}
				else
				{
					i = cont.erase(i);
					++num;
				}
			}
		}
		else
		{
			auto end = std::remove_if(std::begin(cont), std::end(cont), pred);
			num = std::distance(end, std::end(cont));
			cont.erase(end, std::end(cont));
		}
		return num;
	}
	template <Container TCont, typename Pred> inline size_t erase_if_unstable(TCont& cont, Pred pred)
	{
		if constexpr (container_traits<TCont>::associative)
		{
			return erase_if(cont, pred);
		}
		else
		{
			auto beg = std::begin(cont);
			auto end = std::end(cont);
			for (;beg != end;)
			{
				if (!pred(*beg))
				{
					++beg;
				}
				else
				{
					*beg = std::move(*(--end));
				}
			}
			auto num = std::distance(end, std::end(cont));
			cont.erase(end, std::end(cont));
			return num;
		}
	}

	// Sort a container
	template <Container TCont> inline void sort(TCont& cont)
	{
		std::sort(std::begin(cont), std::end(cont));
	}
	template <Container TCont, typename Pred> inline void sort(TCont& cont, Pred pred)
	{
		std::sort(std::begin(cont), std::end(cont), pred);
	}

	// Transform
	template <Container TCont, typename Func> inline void transform(TCont& cont, Func func)
	{
		std::transform(std::begin(cont), std::end(cont), std::begin(cont), func);
	}
	template <Container TCont0, Container TCont1, typename Func>
	inline TCont0 transform(TCont1 const& src, Func func)
	{
		TCont0 out = {};
		out.reserve(std::distance(src.begin(), src.end()));
		for (auto const& item : src)
			out.push_back(func(item));
		return out;
	}

	// Assign with transform
	template <Container TCont0, Container TCont1, typename Func>
	inline void append(TCont0& dst, TCont1 const& src, Func func)
	{
		for (auto const& item : src)
			dst.push_back(func(item));
	}

	// Set intersection/union
	namespace impl
	{
		template <Container TContOut, typename TOrderedCont0, typename TOrderedCont1> TContOut set_intersection_ordered(TOrderedCont0 const& cont0, TOrderedCont1 const& cont1)
		{
			TContOut out;
			std::set_intersection(
				std::begin(cont0), std::end(cont0),
				std::begin(cont1), std::end(cont1),
				std::back_inserter(out));
			return std::move(out);
		}
		template <Container TContOut, typename TOrderedCont0, typename TOrderedCont1> TContOut set_union_ordered(TOrderedCont0 const& cont0, TOrderedCont1 const& cont1)
		{
			TContOut out;
			std::set_union(
				std::begin(cont0), std::end(cont0),
				std::begin(cont1), std::end(cont1),
				std::back_inserter(out));
			return std::move(out);
		}
		template <Container TContOut, typename TAssocCont0, Container TCont1> TContOut set_intersection_associative(TAssocCont0 const& cont0, TCont1 const& cont1)
		{
			TContOut out;
			auto write = std::back_inserter(out);
			for (auto& item : cont1)
			{
				if (!contains(cont0, item)) continue;
				*write++ = item;
			}
			return std::move(out);
		}
		template <Container TContOut, typename TAssocCont0, Container TCont1> TContOut set_union_associative(TAssocCont0 const& cont0, TCont1 const& cont1)
		{
			TContOut out(std::begin(cont0), std::end(cont0));
			auto write = std::back_inserter(out);
			for (auto& item : cont1)
			{
				if (contains(cont0, item)) continue;
				*write++ = item;
			}
			return std::move(out);
		}
	}
	template <Container TContOut, Container TCont0, Container TCont1> TContOut set_intersection(TCont0 const& cont0, TCont1 const& cont1)
	{
		if constexpr (container_traits<TCont0>::associative)
		{
			return impl::set_intersection_associative<TContOut>(cont0, cont1);
		}
		else if constexpr (container_traits<TCont1>::associative)
		{
			return impl::set_intersection_associative<TContOut>(cont1, cont0);
		}
		else
		{
			return impl::set_intersection_ordered<TContOut>(cont0, cont1);
		}
	}
	template <Container TContOut, Container TCont0, Container TCont1> TContOut set_union(TCont0 const& cont0, TCont1 const& cont1)
	{
		if constexpr (container_traits<TCont0>::associative)
		{
			return impl::set_union_associative<TContOut>(cont0, cont1);
		}
		else if constexpr (container_traits<TCont1>::associative)
		{
			return impl::set_union_associative<TContOut>(cont1, cont0);
		}
		else
		{
			return impl::set_union_ordered<TContOut>(cont0, cont1);
		}
	}

	// Zip behaviours
	enum class EZip { All, Unique, SetsBitmask, SetsFull };
	using ZipSet = struct { size_t src_index, elem_index; };

	// Concept for a range of 'T', e.g. T[3]
	template<typename R, typename T>
	concept RangeOf = std::ranges::range<R> && std::same_as<std::ranges::range_value_t<R>, T>;

	// Concept for a range of range of 'T', e.g. T[T[3], T[3], T[3]]
	template <typename R, typename T>
	concept RangeOfRangefOf = std::ranges::range<R> && RangeOf<std::ranges::range_value_t<R>, T>;

	// Zip sorted collections into one ordered collection
	template <EZip OutputItems, typename Type, typename TSources, typename TOut>
	requires (requires (TSources& srcs, TOut out)
	{
		size(srcs);
		srcs[0];
		size(srcs[0]);
		empty(srcs[0]);
		srcs[0][0];
		out(srcs[0][0], {});
	})
	void zip(TSources& srcs, TOut out)
	{
		// Notes:
		//  - 'All' -> returns all values. Second parameter is the 'src_index'
		//  - 'Unique' -> return unique values only. Second parameter is the 'src_index' of one of the sources with this value
		//  - 'SetsBitmask' -> return unique values only. Second parameter is a bitmask of the sources with this value.
		//  - 'SetsFull' -> return unique values only. Second parameter is the source index and element within that source that has this value.

		struct Elem
		{
			Type value;
			size_t src_idx;
			size_t elem_idx;
			bool operator < (Elem const& rhs) const { return rhs.value < value; }
		};

		std::priority_queue<Elem> min_heap;
		std::vector<ZipSet> set_items; set_items.reserve(size(srcs));
		uint64_t set_mask = 0;

		// Check the number of sources can be represented in the set mask
		if constexpr (OutputItems == EZip::SetsBitmask)
		{
			if (size(srcs) > sizeof(set_mask) * 8)
				throw std::runtime_error("SetsBitmask mode requires the number of sources to be <= mask bit count");
		}

		// Initialize heap with the first element from each source
		for (size_t i = 0, src_count = size(srcs); i != src_count; ++i)
		{
			if (empty(srcs[i])) continue;
			min_heap.emplace(srcs[i][0], i, 0);
		}

		// Output each item in order
		std::optional<Type> last_value = {};
		for (; !min_heap.empty(); )
		{
			auto [value, src_idx, elem_idx] = min_heap.top();
			min_heap.pop();

			if constexpr (OutputItems == EZip::All)
			{
				out(value, src_idx);
			}
			if constexpr (OutputItems == EZip::Unique)
			{
				if (!(value == last_value))
				{
					out(value, src_idx);
					last_value = value;
				}
			}
			if constexpr (OutputItems == EZip::SetsBitmask)
			{
				if (!(value == last_value))
				{
					if (set_mask != 0 && last_value)
						out(*last_value, set_mask);
					
					set_mask = 0;
				}
				set_mask |= 1ULL << src_idx;
				last_value = value;
			}
			if constexpr (OutputItems == EZip::SetsFull)
			{
				if (!(value == last_value))
				{
					if (!set_items.empty() && last_value)
						out(*last_value, set_items);
					
					set_items.resize(0);
				}
				set_items.push_back({ src_idx, elem_idx });
				last_value = value;
			}

			// Push next element from the same source
			if (elem_idx + 1 < size(srcs[src_idx]))
				min_heap.emplace(srcs[src_idx][elem_idx + 1], src_idx, elem_idx + 1);
		}

		// In sets mode, output the last set
		if constexpr (OutputItems == EZip::SetsBitmask)
		{
			if (set_mask != 0 && last_value)
				out(*last_value, set_mask);
		}
		if constexpr (OutputItems == EZip::SetsFull)
		{
			if (!set_items.empty() && last_value)
				out(*last_value, set_items);
		}
	}

	// Helper for ranged for to return a pair of the item and the iteration index
	template <std::ranges::input_range R>
	auto with_index(R&& range)
	{
		struct iterator
		{
			std::ranges::iterator_t<R> iter;
			size_t index;

			auto operator*() const { return std::pair{ *iter, index }; }
			auto operator++() { ++iter; ++index; return *this; }
			bool operator!=(const iterator& other) const { return iter != other.iter; }
		};

		struct iterable
		{
			R range;
			auto begin() { return iterator{ std::ranges::begin(range), 0 }; }
			auto end() { return iterator{ std::ranges::end(range), 0 }; }
		};

		return iterable{ std::forward<R>(range) };
	}

	// Iterate over a range, returning iterator pairs representing span of values with the same projection value. E.g. "000011222233344" returns 5 pairs: [0,4], [4,6], [6,10], [10, 13], [13, 15]
	template <std::ranges::input_range R, typename Proj>
	auto group_by(R& range, Proj proj)
	{
		struct iterator
		{
			std::ranges::iterator_t<R> first;
			std::ranges::iterator_t<R> last;
			std::ranges::iterator_t<R> end;
			Proj proj;

			auto operator*() const
			{
				return std::pair{ first, last };
			}
			auto operator++()
			{
				first = last;
				if (last != end)
				{
					auto p = proj(*last++);
					for (; last != end && proj(*last) == p; ++last) {}
				}
				return *this;
			}
			bool operator==(const iterator& rhs) const
			{
				return first == rhs.first && last == rhs.last;
			}
			bool operator!=(const iterator& rhs) const
			{
				return !operator ==(rhs);
			}
		};

		struct iterable
		{
			Proj proj;
			std::ranges::iterator_t<R> b, e;
			auto begin() { return ++iterator{ b, b, e, proj }; }
			auto end() { return iterator{ e, e, e, proj }; }
		};

		return iterable{ proj, std::ranges::begin(range), std::ranges::end(range) };
	}

	// Returns true if 'item' is in the "include" set implied by the ranges 'include' and 'exclude'
	template <typename T>
	bool IncludeFilter(T const& item, std::span<T const> include, std::span<T const> exclude, bool include_by_default)
	{
		auto idx_i = s_cast<size_t>(index_of(include, item));
		auto idx_e = s_cast<size_t>(index_of(exclude, item));

		// Include if in the include range
		if (!include.empty() && idx_i != include.size())
			return true;

		// Exclude if in the exclude range
		if (!exclude.empty() && idx_e != exclude.size())
			return false;

		// If only excludes have been given and not found in the exclude range, assume included
		if (include.empty() && !exclude.empty())
			return true;

		// If only includes have been given and not found in the include range, assume excluded
		if (exclude.empty() && !include.empty())
			return false;

		// If no includes or excludes, assume included
		if (include.empty() && exclude.empty())
			return include_by_default;

		// Includes and excludes have been given, but 'item' is not in either range.
		// Filtering is ambiguous, the caller should ensure this case doesn't occur.
		throw std::runtime_error("Unknown filter case");
	}
}


#if PR_UNITTESTS
#include "pr/common/unittests.h"
namespace pr::algorithm::astar::unittests
{
	PRUnitTestClass(AlgorithmTests)
	{
		PRUnitTestMethod(GroupBy)
		{
			char const src[] = "000011222233344";

			int i = 0;
			std::vector<int> boundaries = {0, 4, 6, 10, 13, 15, 16};
			std::vector<int> spans = {4, 2, 4, 3, 2, 1};
			for (auto [b, e] : group_by(src, [](char x) { return x; }))
			{
				PR_EXPECT(spans[i] == static_cast<int>(e - b));
				PR_EXPECT(boundaries[i] == static_cast<int>(b - &src[0]));
				PR_EXPECT(boundaries[i+1] == static_cast<int>(e - &src[0]));
				++i;
			}
			PR_EXPECT(i == ssize(spans));
		}

		// Exercise the stable value erase path with duplicates and each match position.
		PRUnitTestMethod(EraseStable)
		{
			// The first matching element is removed and later duplicates keep their order.
			{
				std::vector<int> cont = {1, 2, 1, 3};

				erase_stable(cont, 1);
				PR_EXPECT(cont == std::vector<int>({2, 1, 3}));

				erase_stable(cont, 1);
				PR_EXPECT(cont == std::vector<int>({2, 3}));
			}

			// Each remaining case isolates one position or the no-match path.
			{
				std::vector<int> cont = {1, 2, 3};

				erase_stable(cont, 1);
				PR_EXPECT(cont == std::vector<int>({2, 3}));

				cont = {1, 2, 3};
				erase_stable(cont, 2);
				PR_EXPECT(cont == std::vector<int>({1, 3}));

				cont = {1, 2, 3};
				erase_stable(cont, 3);
				PR_EXPECT(cont == std::vector<int>({1, 2}));

				cont = {1, 2, 3};
				erase_stable(cont, 4);
				PR_EXPECT(cont == std::vector<int>({1, 2, 3}));
			}
		}
	};
}
#endif
