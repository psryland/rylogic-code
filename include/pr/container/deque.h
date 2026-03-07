//***********************************************
// pr::deque<>
//  Copyright (c) Rylogic Ltd 2014
//***********************************************
// A version of std::deque with configurable block size
// and proper type alignment.
#pragma once

#include <memory>
#include <iterator>
#include <stdexcept>
#include <algorithm>
#include <type_traits>
#include <utility>
#include <cassert>
#include "pr/common/allocator.h"

#pragma intrinsic(memcmp, memcpy, memset, strcmp)

namespace pr
{
	namespace impl::deque
	{
		// Map of pointers to blocks
		template <typename T, size_t B, typename A>
		struct BlockPtrMap
		{
			using allocator_type   = A;
			using alloc_traits     = std::allocator_traits<std::remove_pointer_t<allocator_type>>;
			using value_type       = typename alloc_traits::value_type;
			using pointer          = typename alloc_traits::pointer;
			using const_pointer    = typename alloc_traits::const_pointer;
			using difference_type  = typename alloc_traits::difference_type;
			using size_type        = typename alloc_traits::size_type;
			using alloc_ptrs_t     = typename alloc_traits::template rebind_alloc<value_type*>;
			using alloc_ptr_traits = std::allocator_traits<alloc_ptrs_t>;
			using block            = value_type[B];
			using byte             = unsigned char;

			static constexpr size_t CountPerBlock = B;

			union {
			block**        m_blocks;     // The array of block pointers
			value_type**   m_ptrs;       // The array of block pointers, size always pow2
			};
			size_type      m_first;      // Index of the first in-use block
			size_type      m_last;       // Index of one past the last in-use block
			size_type      m_capacity;   // The length of 'm_ptrs', either 0 or pow2
			allocator_type m_alloc_type; // Allocator for allocating blocks (value_type[])
			alloc_ptrs_t   m_alloc_ptrs; // Allocator for allocating the block pointer array

			BlockPtrMap(allocator_type const& allocator)
				:m_ptrs()
				,m_first()
				,m_last()
				,m_capacity()
				,m_alloc_type(allocator)
				,m_alloc_ptrs(allocator)
			{}
			~BlockPtrMap()
			{
				free_all();
			}

			// The number of in-use blocks
			size_type count() const
			{
				return static_cast<size_type>(m_last - m_first);
			}

			// Move assign the block ptr array (only valid if allocators are compatible)
			template <size_type B> BlockPtrMap& operator = (BlockPtrMap<value_type,B, allocator_type>&& rhs)
			{
				assert(m_alloc_type == rhs.m_alloc_type && "this implementation only supports same allocators");
				std::swap(m_ptrs     , rhs.m_ptrs    );
				std::swap(m_first    , rhs.m_first   );
				std::swap(m_last     , rhs.m_last    );
				std::swap(m_capacity , rhs.m_capacity);
				return *this;
			}

			// Convert a signed element index into a block index relative to 'm_ptrs[m_first]'.
			// Note: assumes that m_ptr[m_first][0] == element index zero, remember to add 'm_first'
			difference_type block_index(difference_type element_index) const
			{
				auto const count = static_cast<difference_type>(CountPerBlock);
				return element_index >= 0
					? element_index / count
					: -~element_index / count - 1;
			}

			// Return the block for the given element index. Automatically grows the map.
			// Note: assumes that m_ptr[m_first][0] == element index zero, remember to add 'm_first'
			value_type* operator[](difference_type element_index)
			{
				// Convert the element index into a block index
				auto blk_idx = block_index(element_index);

				// Ensure the map is large enough to contain 'blk_idx'
				blk_idx = ensure_space(blk_idx);

				// Return the pointer to the block
				assert(size_type(blk_idx) < count() && "ensure space didn't work");
				return m_ptrs[m_first + blk_idx];
			}
			value_type const* operator[](difference_type element_index) const
			{
				assert(size_type(element_index) < count() * CountPerBlock && "element index out of range");

				// Convert the element index into a block index
				auto blk_idx = block_index(element_index);

				// Return the pointer to the block
				return m_ptrs[m_first + blk_idx];
			}

			// Release all allocated memory
			void free_all()
			{
				for (size_type i = 0; i != m_capacity; ++i)
					m_alloc_type.deallocate(m_ptrs[i], CountPerBlock);

				alloc_ptr_traits::deallocate(m_alloc_ptrs, m_ptrs, m_capacity);
				m_ptrs     = nullptr;
				m_first    = 0;
				m_last     = 0;
				m_capacity = 0;
			}

			// Release capacity. [first,last) are the in-use element range
			void shrink_to_fit(size_type& first, size_type& last)
			{
				// Free unused blocks at the front
				while (first >= CountPerBlock)
				{
					m_alloc_type.deallocate(m_ptrs[m_first], CountPerBlock);
					m_ptrs[m_first++] = nullptr;
					first -= CountPerBlock;
				}

				// Free unused blocks at the end
				while (count()* CountPerBlock - last >= CountPerBlock)
				{
					m_alloc_type.deallocate(m_ptrs[--m_last], CountPerBlock);
					m_ptrs[m_last] = nullptr;
				}
			
				// Reallocate the block ptr map if excessive
				size_type inuse = m_last - m_first;
				if (inuse < m_capacity/2)
				{
					auto new_capacity = m_capacity/2;
					for (; new_capacity/2 > inuse; new_capacity /= 2) {}
					value_type** mem = alloc_ptr_traits::allocate(m_alloc_ptrs, new_capacity);

					::memcpy(mem, m_ptrs + m_first, inuse * sizeof(value_type*));
					::memset(mem + inuse, 0, (new_capacity - inuse) * sizeof(value_type*));

					std::swap(m_ptrs, mem);
					m_first    = 0;
					m_last     = inuse;
					m_capacity = new_capacity;
				}
			}

			// Grow the block map to include 'blk_idx'.
			// If block index is outside the range [0, count()) then new blocks will be added.
			difference_type ensure_space(difference_type blk_idx)
			{
				if (size_type(blk_idx) < count())
					return blk_idx;

				// Add to the start
				if (blk_idx < 0)
				{
					size_type to_add = -blk_idx;

					// If there isn't space, reallocate the block ptr map
					if (to_add > m_first)
					{
						grow_map(to_add + count(), [](value_type** mem, size_type new_capacity, BlockPtrMap& map)
						{
							// Fill the front with nullptr and the end from the old map
							auto keep_count = map.m_capacity - map.m_first;
							auto fill_count = new_capacity - keep_count;
							::memset(mem, 0, fill_count * sizeof(value_type*));
							::memcpy(mem + fill_count, map.m_ptrs + map.m_first, keep_count * sizeof(value_type*));
							return fill_count;
						});
					}

					// Fill with new blocks
					for (; to_add-- != 0;)
						m_ptrs[--m_first] = m_alloc_type.allocate(CountPerBlock);

					blk_idx = 0;
				}
				else // add to the end
				{
					auto to_add = blk_idx - count() + 1;

					// If there isn't space, reallocate the block ptr map
					if (to_add > m_capacity - m_last)
					{
						grow_map(to_add + count(), [](value_type** mem, size_type new_capacity, BlockPtrMap& map)
						{
							// Fill the front from the old map, and the end with nullptr
							auto keep_count = map.m_last;
							auto fill_count = new_capacity - keep_count;
							::memset(mem + keep_count, 0, fill_count * sizeof(value_type*));
							::memcpy(mem, map.m_ptrs, keep_count * sizeof(value_type*));
							return map.m_first;
						});
					}

					// Fill with new blocks
					for (; to_add-- != 0;)
						m_ptrs[m_last++] = m_alloc_type.allocate(CountPerBlock);
				}
				return blk_idx;
			}

			// Grow the 'm_ptrs' buffer to contain at least 'min_count' items
			template <typename CopyFunc> void grow_map(size_type min_count, CopyFunc copy)
			{
				// Determine the required capacity
				auto new_capacity = m_capacity ? m_capacity * 2 : 1;
				for (; new_capacity < min_count; new_capacity *= 2) {}

				// Allocate a new pointer map
				value_type** mem = alloc_ptr_traits::allocate(m_alloc_ptrs, new_capacity);
				
				// Copy from the old map to the new
				auto new_count = count(); // same as old count
				auto new_first = copy(mem, new_capacity, *this);
				
				// Swap the pointers
				alloc_ptr_traits::deallocate(m_alloc_ptrs, m_ptrs, m_capacity);
				m_ptrs     = mem;
				m_first    = new_first;
				m_last     = new_first + new_count;
				m_capacity = new_capacity;
			}

			// Swap the block ptr map
			void swap(BlockPtrMap& rhs)
			{
				// same allocator, swap pointers
				auto same_alloc = m_alloc_type == rhs.m_alloc_type;
				if (same_alloc || alloc_traits::propagate_on_container_swap::value)
				{
					if (!same_alloc)
					{
						std::swap(m_alloc_type , rhs.m_alloc_type);
						std::swap(m_alloc_ptrs , rhs.m_alloc_ptrs);
					}
					std::swap(m_ptrs     , rhs.m_ptrs    );
					std::swap(m_first    , rhs.m_first   );
					std::swap(m_last     , rhs.m_last    );
					std::swap(m_capacity , rhs.m_capacity);
				}
			}
		};

		// Iterators
		template <typename T, size_t B, typename A> struct citer // const iterator
		{
			using allocator_type    = A;
			using alloc_traits      = std::allocator_traits<allocator_type>;
			using iterator_category = std::random_access_iterator_tag;
			using value_type        = typename alloc_traits::value_type;
			using pointer           = typename alloc_traits::pointer;
			using difference_type   = typename alloc_traits::difference_type;
			using size_type         = typename alloc_traits::size_type;
			using reference         = value_type const&;
			using map_ptr_t         = BlockPtrMap<T, B, A> const*;

			size_type m_idx; // element index + deque.m_first
			map_ptr_t m_map; // pointer to the block pointer array

			citer()
				:m_idx()
				, m_map()
			{}
			citer(size_type idx, map_ptr_t map)
				:m_idx(idx)
				,m_map(map)
			{}
			value_type const& operator [](difference_type ofs) const
			{
				return *(*this + ofs);
			}
			value_type const& operator * () const
			{
				return (*m_map)[m_idx][m_idx % B];
			}
			pointer operator ->() const
			{
				return std::pointer_traits<pointer>::pointer_to(**this);
			}
			citer& operator ++()
			{
				++m_idx;
				return *this;
			}
			citer operator ++(int)
			{
				auto tmp = *this;
				++*this;
				return tmp;
			}
			citer& operator --()
			{
				--m_idx;
				return *this;
			}
			citer operator --(int)
			{
				auto tmp = *this;
				--*this;
				return tmp;
			}

			friend bool operator == (citer lhs, citer rhs)
			{
				return lhs.m_idx == rhs.m_idx;
			}
			friend bool operator != (citer lhs, citer rhs)
			{
				return lhs.m_idx != rhs.m_idx;
			}
			friend bool operator <  (citer lhs, citer rhs)
			{
				return lhs.m_idx < rhs.m_idx;
			}
			friend bool operator >  (citer lhs, citer rhs)
			{
				return lhs.m_idx > rhs.m_idx;
			}
			friend bool operator <= (citer lhs, citer rhs)
			{
				return lhs.m_idx <= rhs.m_idx;
			}
			friend bool operator >= (citer lhs, citer rhs)
			{
				return lhs.m_idx >= rhs.m_idx;
			}

			friend citer& operator += (citer& lhs, difference_type ofs)
			{
				lhs.m_idx += ofs;
				return lhs;
			}
			friend citer& operator -= (citer& lhs, difference_type ofs)
			{
				lhs.m_idx -= ofs;
				return lhs;
			}
			friend citer operator +  (citer lhs, difference_type ofs)
			{
				auto tmp = lhs;
				return tmp += ofs;
			}
			friend citer operator -  (citer lhs, difference_type ofs)
			{
				auto tmp = lhs;
				return tmp -= ofs;
			}
			friend difference_type operator - (citer lhs, citer rhs)
			{
				return rhs.m_idx <= lhs.m_idx
					? lhs.m_idx - rhs.m_idx
					: -ptrdiff_t(rhs.m_idx - lhs.m_idx);
			}
		};
		template <typename T, size_t B, typename A> struct miter // mutable iterator
		{
			using allocator_type    = A;
			using alloc_traits      = std::allocator_traits<allocator_type>;
			using iterator_category = std::random_access_iterator_tag;
			using value_type        = typename alloc_traits::value_type;
			using pointer           = typename alloc_traits::pointer;
			using difference_type   = typename alloc_traits::difference_type;
			using size_type         = typename alloc_traits::size_type;
			using reference         = value_type&;
			using map_ptr_t         = BlockPtrMap<T, B, A>*;

			size_type m_idx; // element index + deque.m_first
			map_ptr_t m_map; // pointer to the block pointer array

			miter()
				:m_idx()
				,m_map()
			{}
			miter(size_type idx, map_ptr_t map)
				:m_idx(idx)
				,m_map(map)
			{}
			operator citer<T,B,A>() const
			{
				return citer<T,B,A>(m_idx, m_map);
			}
			value_type& operator [](difference_type ofs) const
			{
				return *(*this + ofs);
			}
			value_type& operator * () const
			{
				return (*m_map)[m_idx][m_idx % B];
			}
			pointer operator ->() const
			{
				return std::pointer_traits<pointer>::pointer_to(**this);
			}
			miter& operator ++()
			{
				++m_idx;
				return *this;
			}
			miter operator ++(int)
			{
				auto tmp = *this;
				++* this;
				return tmp;
			}
			miter& operator --()
			{
				--m_idx;
				return *this;
			}
			miter operator --(int)
			{
				auto tmp = *this;
				--* this;
				return tmp;
			}

			friend bool operator == (miter lhs, miter rhs)
			{
				return lhs.m_idx == rhs.m_idx;
			}
			friend bool operator != (miter lhs, miter rhs)
			{
				return lhs.m_idx != rhs.m_idx;
			}
			friend bool operator <  (miter lhs, miter rhs)
			{
				return lhs.m_idx < rhs.m_idx;
			}
			friend bool operator >  (miter lhs, miter rhs)
			{
				return lhs.m_idx > rhs.m_idx;
			}
			friend bool operator <= (miter lhs, miter rhs)
			{
				return lhs.m_idx <= rhs.m_idx;
			}
			friend bool operator >= (miter lhs, miter rhs)
			{
				return lhs.m_idx >= rhs.m_idx;
			}

			friend miter& operator += (miter& lhs, difference_type ofs)
			{
				lhs.m_idx += ofs;
				return lhs;
			}
			friend miter& operator -= (miter& lhs, difference_type ofs)
			{
				lhs.m_idx -= ofs;
				return lhs;
			}
			friend miter operator +  (miter lhs, difference_type ofs)
			{
				auto tmp = lhs;
				return tmp += ofs;
			}
			friend miter operator -  (miter lhs, difference_type ofs)
			{
				auto tmp = lhs;
				return tmp -= ofs;
			}
			friend difference_type operator - (miter lhs, miter rhs)
			{
				return rhs.m_idx <= lhs.m_idx
					? lhs.m_idx - rhs.m_idx
					: -ptrdiff_t(rhs.m_idx - lhs.m_idx);
			}
		};
		using test_citer = citer<int, 1, std::allocator<int>>;
		using test_miter = miter<int, 1, std::allocator<int>>;
		static_assert(std::is_convertible_v<test_miter, test_citer>);
		static_assert(std::is_same_v<test_miter, decltype(std::declval<test_miter>() + 3ULL)>);
	}

	// Not intended to be a complete replacement, just a 90% substitute
	template <typename Type, size_t BlockSize=16, typename Allocator=aligned_alloc<Type>>
	class deque
	{
		static_assert(((BlockSize - 1)& BlockSize) == 0, "BlockSize must be a power of two");

	public:

		using allocator_type = Allocator;
		using alloc_traits = std::allocator_traits<allocator_type>;
		using value_type = typename alloc_traits::value_type;
		using pointer = typename alloc_traits::pointer;
		using const_pointer = typename alloc_traits::const_pointer;
		using difference_type = typename alloc_traits::difference_type;
		using size_type = typename alloc_traits::size_type;
		using iterator = pr::impl::deque::miter<value_type, BlockSize, allocator_type>;
		using const_iterator = pr::impl::deque::citer<value_type, BlockSize, allocator_type>;
		using block_ptr_map_t = pr::impl::deque::BlockPtrMap<value_type, BlockSize, allocator_type>;

		static_assert(std::is_same_v<typename std::iterator_traits<iterator>::value_type, Type>);

		static constexpr bool type_is_pod_v = std::is_trivially_copyable_v<value_type>;
		static constexpr bool type_is_copyable_v = std::is_copy_constructible_v<value_type>;
		static constexpr int type_alignment_v = std::alignment_of_v<value_type>;
		static constexpr size_type CountPerBlock = BlockSize;
		static constexpr size_type BlockIndexMask = BlockSize - 1U;

	private:

		// Any combination of type, block size, and allocator is a friend
		template <class T, size_t B, class A> friend class deque;

		struct traits
		{
			static void destruct(allocator_type& alloc, value_type* first, size_type count)
			{
				if constexpr (type_is_pod_v)
				{
					assert((::memset(first, 0xdd, sizeof(value_type) * count), true));
					(void)alloc, first, count;
				}
				else
				{
					for (; count--;)
						alloc_traits::destroy(alloc, first++);
				}
			}
			static void construct(allocator_type& alloc, value_type* first, size_type count)
			{
				if constexpr (type_is_pod_v)
				{
					(void)alloc, first, count;
				}
				else
				{
					for (; count--;)
						alloc_traits::construct(alloc, first++);
				}
			}
			static void copy_constr(allocator_type& alloc, value_type* first, value_type const* src, size_type count)
			{
				if constexpr (type_is_pod_v)
				{
					::memcpy(first, src, sizeof(value_type) * count);
					(void)alloc;
				}
				else
				{
					for (; count--;)
						alloc_traits::construct(alloc, first++, *src++);
				}
			}
			static void move_constr(allocator_type& alloc, value_type* first, value_type* src, size_type count)
			{
				if constexpr (type_is_pod_v)
				{
					::memcpy(first, src, sizeof(value_type) * count);
					(void)alloc;
				}
				else
				{
					for (; count--;)
						alloc_traits::construct(alloc, first++, std::move(*src++));
				}
			}
			static void copy_assign(value_type* first, value_type const* src, size_type count)
			{
				if constexpr (type_is_pod_v)
				{
					::memcpy(first, src, sizeof(value_type) * count);
				}
				else
				{
					for (; count--; ++first, ++src)
						*first = *src;
				}
			}
			static void move_assign(value_type* first, value_type* src, size_type count)
			{
				if constexpr (type_is_pod_v)
				{
					::memcpy(first, src, sizeof(value_type) * count);
				}
				else
				{
					for (; count--; ++first, ++src)
						*first = std::move(*src);
				}
			}
			static void destruct_range(allocator_type& alloc, iterator first, size_type count)
			{
				for (; count--; ++first)
					destruct(alloc, &*first, 1);
			}
			static void fill_constr(allocator_type& alloc, value_type* first, size_type count, value_type const& val)
			{
				for (; count--;)
					copy_constr(alloc, first++, &val, 1);
			}
			static void fill_assign(value_type* first, size_type count, value_type const& val)
			{
				for (; count--;)
					copy_assign(first++, &val, 1);
			}
			template <class... Args> static void constr(allocator_type& alloc, value_type* first, Args&&... args)
			{
				alloc_traits::construct(alloc, first, std::forward<Args>(args)...);
			}
			static iterator move_left(const_iterator first, const_iterator last, iterator dest)
			{
				// move [first, last) backwards to [..., dest)
				for (; first != last; )
					*--dest = std::move(*--last);
				return dest;
			}
			static iterator move_right(const_iterator first, const_iterator last, iterator dest)
			{
				// move [first, last) to [dest, ...)
				for (; first != last; ++dest, ++first)
					*dest = std::move(*first);
				return dest;
			}
		};

		// Debugging view
		#ifndef NDEBUG
		template <typename T> struct DbgPtr { using type = T const (*)[CountPerBlock]; };
		template <> struct DbgPtr<char> { using type = char const*; };
		template <> struct DbgPtr<wchar_t> { using type = wchar_t const*; };
		typename DbgPtr<value_type>::type m_data;
		bool set_dbg_ptr()
		{
			auto ptr = &m_map[m_first][m_first & BlockIndexMask];
			m_data = reinterpret_cast<typename DbgPtr<value_type>::type>(ptr);
			return true;
		}
		#endif

		block_ptr_map_t m_map; // The array of block pointers
		size_type m_first; // Element index of the first element == The offset from 'm_map.m_ptr' to the first element
		size_type m_last;  // Element index of the last+1 element == The offset from 'm_map.m_ptr' to one passed the last element

	public:

		// construct empty
		deque()
			:m_map(allocator_type())
			, m_first()
			, m_last()
		{
			assert(set_dbg_ptr());
		}
		~deque()
		{
			clear();
		}

		// construct with custom allocator
		explicit deque(allocator_type const& allocator)
			:m_map(allocator)
			, m_first()
			, m_last()
		{
			assert(set_dbg_ptr());
		}

		// construct from count * value_type()
		explicit deque(size_type count)
			:m_map(allocator_type())
			, m_first()
			, m_last()
		{
			resize(count);
			assert(set_dbg_ptr());
		}

		// construct from count * val
		deque(size_type count, value_type const& val)
			:m_map(allocator_type())
			, m_first()
			, m_last()
		{
			for (; count-- != 0;)
				push_back(val);
			assert(set_dbg_ptr());
		}

		// copy construct (explicit copy constructor needed to prevent auto generated one even tho there's a template one that would work)
		deque(deque const& right)
			:m_map(right.alloc())
			, m_first()
			, m_last()
		{
			for (auto& r : right)
				push_back(r);
			assert(set_dbg_ptr());
		}

		// copy construct from any pr::deque type
		template <size_type B> deque(deque<value_type, B, allocator_type> const& right)
			:m_map(right.alloc())
			, m_first()
			, m_last()
		{
			for (auto& r : right)
				push_back(r);
			assert(set_dbg_ptr());
		}

		// construct from [first, last), with optional allocator
		template <class iter> deque(iter first, iter last, allocator_type const& allocator = allocator_type())
			:m_map(allocator)
			, m_first()
			, m_last()
		{
			for (; !(first == last); ++first)
				push_back(*first);
			assert(set_dbg_ptr());
		}

		// construct from initialiser list
		deque(std::initializer_list<value_type> list, allocator_type const& allocator = allocator_type())
			:m_map(allocator)
			, m_first()
			, m_last()
		{
			insert(begin(), list.begin(), list.end());
			assert(set_dbg_ptr());
		}

		// construct from another container-like object
		// 2nd parameter used to prevent overload issues with deque(count) (using SFINAE)
		template <class tcont> deque(tcont const& rhs, typename tcont::size_type = 0, allocator_type const& allocator = allocator_type())
			:m_map(allocator)
			, m_first()
			, m_last()
		{
			for (auto& elem : rhs)
				emplace_back(elem);
			assert(set_dbg_ptr());
		}

		// construct by moving right
		deque(deque&& rhs) noexcept
			:m_map(rhs.alloc())
			, m_first()
			, m_last()
		{
			*this = std::move(rhs);
			assert(set_dbg_ptr());
		}

		// construct by moving right with allocator
		deque(deque&& rhs, allocator_type const& allocator)
			:m_map(allocator)
			, m_first()
			, m_last()
		{
			*this = std::move(rhs);
			assert(set_dbg_ptr());
		}

		// The memory allocator
		allocator_type const& get_allocator() const
		{
			return alloc();
		}
		allocator_type& get_allocator()
		{
			return alloc();
		}

		// assign by moving rhs
		deque& operator = (deque&& rhs) noexcept
		{
			if (this != &rhs)
				impl_assign(std::move(rhs));
			assert(set_dbg_ptr());
			return *this;
		}

		// assign rhs
		template <size_type B> deque& operator = (deque<value_type, B, allocator_type>&& rhs)
		{
			impl_assign(std::move(rhs));
			assert(set_dbg_ptr());
			return *this;
		}

		// assign rhs
		deque& operator = (deque const& rhs)
		{
			if (this != &rhs)
				impl_assign(rhs);
			assert(set_dbg_ptr());
			return *this;
		}

		// assign rhs
		template <size_type B> deque& operator = (deque<value_type, B, allocator_type> const& rhs)
		{
			impl_assign(rhs);
			assert(set_dbg_ptr());
			return *this;
		}

		// assign initializer_list
		deque& operator = (std::initializer_list<value_type> list)
		{
			assert(set_dbg_ptr());
			assign(list.begin(), list.end());
			return *this;
		}

		// test if container is empty
		bool empty() const
		{
			return m_last == m_first;
		}

		// return the length of the container
		size_type size() const
		{
			return m_last - m_first;
		}

		// element at position
		value_type const& at(size_type idx) const
		{
			assert(idx < size() && "out of range");
			idx += m_first;
			return m_map[idx][idx & BlockIndexMask];
		}
		value_type& at(size_type idx)
		{
			assert(idx < size() && "out of range");
			idx += m_first;
			return m_map[idx][idx & BlockIndexMask];
		}

		// index operator
		value_type const& operator[](size_type idx) const
		{
			return at(idx);
		}
		value_type& operator[](size_type idx)
		{
			return at(idx);
		}

		// return iterator for beginning of mutable sequence
		iterator begin() noexcept
		{
			return iterator(m_first, &m_map);
		}

		// return iterator for beginning of non-mutable sequence
		const_iterator begin() const noexcept
		{
			return const_iterator(m_first, &m_map);;
		}

		// return iterator for end of mutable sequence
		iterator end() noexcept
		{
			return iterator(m_first + size(), &m_map);;
		}

		// return iterator for end of non-mutable sequence
		const_iterator end() const noexcept
		{
			return const_iterator(m_first + size(), &m_map);;
		}

		// return first element of mutable sequence
		value_type& front()
		{
			return *begin();
		}

		// return first element of non-mutable sequence
		value_type const& front() const
		{
			return *begin();
		}

		// return last element of mutable sequence
		value_type& back()
		{
			return *(end() - 1);
		}

		// return last element of non-mutable sequence
		value_type const& back() const
		{
			return *(end() - 1);
		}

		// erase all
		void clear()
		{
			resize(0);
			m_map.free_all();
		}

		// set new length, padding with value_type() as needed
		void resize(size_type count)
		{
			while (size() < count) emplace_back();
			while (size() > count) pop_back();
		}

		// set new length, padding with 'val' as needed
		void resize(size_type count, value_type const& val)
		{
			while (size() < count) push_back(val);
			while (size() > count) pop_back();
		}

		// reduce capacity
		void shrink_to_fit()
		{
			m_map.shrink_to_fit(m_first, m_last);
		}

		// number of push_front() calls before an allocation
		size_type capacity_front() const
		{
			return m_first;
		}

		// number of push_back() calls before an allocation
		size_type capacity_back() const
		{
			return m_map.count() * CountPerBlock - m_last;
		}

		// insert element at end
		void push_back(value_type&& value)
		{
			// insert by moving into element at end, provide strong guarantee
			emplace_back(std::move(value));
		}
		void push_back(value_type const& val)
		{
			auto block = m_map[m_last];
			auto idx = m_last & BlockIndexMask;
			traits::copy_constr(alloc(), block + idx, &val, 1);
			++m_last;
		}

		// insert element at end
		template<class... Args> void emplace_back(Args&&... args)
		{
			auto block = m_map[m_last];
			auto idx = m_last & BlockIndexMask;
			traits::constr(alloc(), block + idx, std::forward<Args>(args)...);
			++m_last;
		}

		// insert element at beginning
		void push_front(value_type&& value)
		{
			// insert by moving into element at front, provide strong guarantee
			emplace_front(std::move(value));
		}
		void push_front(value_type const& val)
		{
			auto block = m_map[m_first - 1];
			auto idx = (m_first - 1 + CountPerBlock) & BlockIndexMask;
			traits::copy_constr(alloc(), block + idx, &val, 1);
			if (m_first == 0) { m_first += CountPerBlock; m_last += CountPerBlock; }
			--m_first;
		}

		// insert element at beginning
		template<class... Args> void emplace_front(Args&&... args)
		{
			auto block = m_map[m_first - 1];
			auto idx = (m_first - 1 + CountPerBlock) & BlockIndexMask;
			traits::constr(alloc(), block + idx, std::forward<Args>(args)...);
			if (m_first == 0) { m_first += CountPerBlock; m_last += CountPerBlock; }
			--m_first;
		}

		// erase the element at the end
		void pop_back()
		{
			assert(!empty() && "pop_back() on empty deque");
			auto block = m_map[m_last - 1];
			auto idx = (m_last - 1 + CountPerBlock) & BlockIndexMask;
			traits::destruct(alloc(), block + idx, 1);
			--m_last; // last can't be 0 unless m_first > m_last
			if (m_first == m_last) m_first = m_last = 0;
		}

		// erase the element at the beginning
		void pop_front()
		{
			assert(!empty() && "pop_front() on empty deque");
			auto block = m_map[m_first];
			auto idx = m_first & BlockIndexMask;
			traits::destruct(alloc(), block + idx, 1);
			++m_first;
			if (m_first == m_last) m_first = m_last = 0;
		}

		// assign [first, last), input iterators
		template <typename iter> void assign(iter first, iter last)
		{
			clear();
			for (; first != last; ++first)
				emplace_back(*first);
		}

		// assign count * val
		void assign(size_type count, value_type const& val)
		{
			erase(begin(), end());
			impl_insert(begin(), count, val);
		}

		// insert val at 'at'
		iterator insert(const_iterator at, value_type const& val)
		{
			size_type ofs = at - begin();
			assert(ofs <= size() && "deque insert iterator outside range");

			// closer to front, push to front then copy
			if (ofs <= size() / 2)
			{
				push_front(val);
				if (ofs != 0) std::rotate(begin(), begin() + 1, begin() + 1 + ofs);
			}
			else // closer to back, push to back then copy
			{
				push_back(val);
				if (ofs != size() - 1) std::rotate(begin() + ofs, end() - 1, end());
			}
			return begin() + ofs;
		}

		// insert count * val at 'at'
		iterator insert(const_iterator at, size_type count, value_type const& val)
		{
			auto ofs = at - begin();
			impl_insert(at, count, val);
			return begin() + ofs;
		}

		// insert [first, last) at 'at', input iterators
		template <class Iter> iterator insert(const_iterator at, Iter first, Iter last)
		{
			size_type ofs = at - begin();
			assert(ofs <= size() && "deque insert iterator outside range");
			size_type old_size = size();

			if (first == last)
			{
			}
			else if (ofs <= size() / 2) // closer to front, push to front then rotate
			{
				try
				{
					for (; first != last; ++first)
						push_front(*first); // prepend flipped
				}
				catch (...)
				{
					for (; old_size < size(); )
						pop_front(); // restore old size, at least
					throw;
				}

				size_type num = size() - old_size;
				std::reverse(begin(), begin() + num); // flip new stuff in place
				std::rotate(begin(), begin() + num, begin() + num + ofs);
			}
			else // closer to back
			{
				try
				{
					for (; first != last; ++first)
						push_back(*first); // append
				}
				catch (...)
				{
					for (; old_size < size(); )
						pop_back(); // restore old size, at least
					throw;
				}
				std::rotate(begin() + ofs, begin() + old_size, end());
			}
			return begin() + ofs;
		}

		// erase element at 'at'
		iterator erase(const_iterator at)
		{
			return erase(at, at + 1);
		}

		// erase [first, last)
		iterator erase(const_iterator first, const_iterator last)
		{
			assert(first <= last && first >= begin() && last <= end() && "invalid iterator range");

			size_type b = first.m_idx - m_first;
			size_type e = m_last - last.m_idx;
			size_type c = last.m_idx - first.m_idx;

			if (b < e) // closer to front, move begin() forward
			{
				traits::move_left(begin(), first, make_iter(last)); // move [begin,first) to [..,last)
				traits::destruct_range(alloc(), begin(), c);
				m_first += c;
				if (m_first == m_last) m_first = m_last = 0;
			}
			else // closer to back, move end() backward
			{
				traits::move_right(last, end(), make_iter(first));
				traits::destruct_range(alloc(), end() - c, c);
				m_last -= c;
				if (m_first == m_last) m_first = m_last = 0;
			}
			return begin() + b;
		}

		// exchange contents with rhs
		void swap(deque& rhs)
		{
			if (this == &rhs) // same object, do nothing
			{
			}
			else if (alloc() == rhs.alloc() || alloc_traits::propagate_on_container_swap::value)
			{
				// same allocators or allocator allows swap
				std::swap(m_map, rhs.m_map);
				std::swap(m_first, rhs.m_first);
				std::swap(m_last, rhs.m_last);
			}
			else // containers are incompatible
			{
				assert(false && "deque containers incompatible for swap");
			}
		}

		// Conversion to a type that can be constructed from begin/end iterators
		// This allows cast to std::vector<> amoung others.
		// Note: converting to a std::vector<> when 'value_type' has an alignment greater than the
		// default alignment causes a compiler error because of std::vector.resize().
		template <typename TCont> requires (std::is_same_v<typename TCont::value_type, value_type>)
		explicit operator TCont() const
		{
			return TCont(begin(), end());
		}

	private:

		allocator_type const& alloc() const
		{
			return m_map.m_alloc_type;
		}
		allocator_type& alloc()
		{
			return m_map.m_alloc_type;
		}

		// copy assign rhs
		template <size_type B> void impl_assign(deque<value_type, B, allocator_type> const& rhs)
		{
			// change allocator before copying
			auto same_alloc = alloc() == rhs.alloc();
			if (!same_alloc && alloc_traits::propagate_on_container_copy_assignment::value)
			{
				clear();
				m_map.m_alloc_type = rhs.m_map.m_alloc_type;
				m_map.m_alloc_ptrs = rhs.m_map.m_alloc_ptrs;
			}

			if (rhs.empty())
			{
				clear();
			}
			else if (rhs.size() <= size()) // enough elements, copy new and destroy old
			{
				auto mid = std::copy(rhs.begin(), rhs.end(), begin());
				erase(mid, end());
			}
			else // new sequence longer, copy and construct new
			{
				auto mid = rhs.begin() + size();
				std::copy(rhs.begin(), mid, begin());
				insert(end(), mid, rhs.end());
			}
		}

		// move assign rhs
		template <size_type B> void impl_assign(deque<value_type, B, allocator_type>&& rhs)
		{
			clear();
			m_map = std::move(rhs.m_map);
			std::swap(m_first, rhs.m_first);
			std::swap(m_last, rhs.m_last);
		}

		// insert count * val at 'at'
		void impl_insert(const_iterator at, size_type count, value_type const& val)
		{
			size_type num;
			size_type ofs = at - begin();
			size_type rem = size() - ofs;
			size_type old_size = size();

			assert(ofs <= size() && "deque insert iterator outside range");

			if (ofs < rem) // closer to front
			{
				try
				{
					// insert longer than prefix
					if (ofs < count)
					{
						// push excess values
						for (num = count - ofs; 0 < num; --num)
							push_front(val);

						// push prefix
						for (num = ofs; 0 < num; --num)
							push_front(begin()[count - 1]);

						// fill in rest of values
						auto mid = begin() + count;
						std::fill<iterator>(mid, mid + ofs, val);
					}
					else // insert not longer than prefix
					{
						// push part of prefix
						for (num = count; 0 < num; --num)
							push_front(begin()[count - 1]);

						// in case val is in sequence
						auto mid = begin() + count;
						auto tmp = val;
						traits::move_right(mid + count, mid + ofs, mid); // copy rest of prefix
						std::fill<iterator>(begin() + ofs, mid + ofs, tmp); // fill in values
					}
				}
				catch (...)
				{
					// restore old size, at least
					for (; old_size < size();)
						pop_front();
					throw;
				}
			}
			else // closer to back
			{
				try
				{
					// insert longer than suffix
					if (rem < count)
					{
						// push excess values
						for (num = count - rem; 0 < num; --num)
							push_back(val);

						// push suffix
						for (num = 0; num < rem; ++num)
							push_back(begin()[ofs + num]);

						// fill in rest of values
						auto mid = begin() + ofs;
						std::fill<iterator>(mid, mid + rem, val);
					}
					else // insert not longer than prefix
					{
						// push part of prefix
						for (num = 0; num < count; ++num)
							push_back(begin()[ofs + rem - count + num]);

						// copy rest of prefix
						auto mid = begin() + ofs;
						auto tmp = val; // in case val is in sequence
						traits::move_left(mid, mid + rem - count, mid + rem);
						std::fill<iterator>(mid, mid + count, tmp); // fill in values
					}
				}
				catch (...)
				{
					// restore old size, at least
					for (; old_size < size();)
						pop_back();
					throw;
				}
			}
		}

		// make iterator from const_iterator
		iterator make_iter(const_iterator i)
		{
			assert(i.m_map == &m_map && "iterator not for this deque");
			return iterator(i.m_idx, &m_map);;
		}
	};

	// operators
	template <class T, size_t B, typename A> inline void swap(deque<T,B,A>& lhs, deque<T,B,A>& rhs)
	{
		lhs.swap(rhs);
	}
	template <class T1, size_t B1, typename A1, class T2, size_t B2, typename A2> inline bool operator == (deque<T1,B1,A1> const& lhs, deque<T2,B2,A2> const& rhs)
	{
		return lhs.size() == rhs.size() && std::equal(lhs.begin(), lhs.end(), rhs.begin());
	}
	template <class T1, size_t B1, typename A1, class T2, size_t B2, typename A2> inline bool operator != (deque<T1,B1,A1> const& lhs, deque<T2,B2,A2> const& rhs)
	{
		return !(lhs == rhs);
	}
	template <class T1, size_t B1, typename A1, class T2, size_t B2, typename A2> inline bool operator <  (deque<T1,B1,A1> const& lhs, deque<T2,B2,A2> const& rhs)
	{
		return std::lexicographical_compare(lhs.begin(), lhs.end(), rhs.begin(), rhs.end());
	}
	template <class T1, size_t B1, typename A1, class T2, size_t B2, typename A2> inline bool operator <= (deque<T1,B1,A1> const& lhs, deque<T2,B2,A2> const& rhs)
	{
		return !(rhs < lhs);
	}
	template <class T1, size_t B1, typename A1, class T2, size_t B2, typename A2> inline bool operator >  (deque<T1,B1,A1> const& lhs, deque<T2,B2,A2> const& rhs)
	{
		return rhs < lhs;
	}
	template <class T1, size_t B1, typename A1, class T2, size_t B2, typename A2> inline bool operator >= (deque<T1,B1,A1> const& lhs, deque<T2,B2,A2> const& rhs)
	{
		return !(lhs < rhs);
	}
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
namespace pr::container::tests
{
	PRUnitTestClass(DequeTests)
	{
		// The number of 'Type' instances in existence
		inline static int object_count = 0;

		// A singleton
		struct Single {};
		inline static std::shared_ptr<Single> const& ref()
		{
			static std::shared_ptr<Single> ptr(new Single);
			return ptr;
		};

		// Marker for moved values
		inline static constexpr int MovedValue = -1;

		// A copy constructable type
		struct Type
		{
			int val;
			std::shared_ptr<Single> ptr;

			Type()
				: Type(0)
			{
			}
			Type(int w)
				: val(w)
				, ptr(ref())
			{
				++object_count;
			}
			Type(Type&& rhs) noexcept
				: Type()
			{
				std::swap(val, rhs.val);
				std::swap(ptr, rhs.ptr);
				rhs.val = MovedValue;
			}
			Type(Type const& rhs)
				: val(rhs.val)
				, ptr(rhs.ptr)
			{
				++object_count;
			}
			Type& operator = (Type&& rhs) noexcept
			{
				if (this != &rhs)
				{
					std::swap(val, rhs.val);
					std::swap(ptr, rhs.ptr);
					rhs.val = MovedValue;
				}
				return *this;
			}
			Type& operator = (Type const& rhs)
			{
				if (this != &rhs)
				{
					val = rhs.val;
					ptr = rhs.ptr;
				}
				return *this;
			}
			virtual ~Type()
			{
				--object_count;
				PR_EXPECT(ptr.get() == ref().get()); // destructing an invalid Type
				val = 0xdddddddd;
			}
			friend bool operator == (Type const& lhs, Type const& rhs)
			{
				return lhs.val == rhs.val;
			}
			friend bool operator == (Type const& lhs, int rhs)
			{
				return lhs.val == rhs;
			}
		};
		static_assert(std::is_move_constructible<Type>::value);
		static_assert(std::is_copy_constructible<Type>::value);
		static_assert(std::is_move_assignable<Type>::value);
		static_assert(std::is_copy_assignable<Type>::value);

		// A move-only type
		struct NonCopyable :Type
		{
			NonCopyable()
				:Type()
			{
			}
			NonCopyable(int w)
				:Type(w)
			{
			}
			NonCopyable(NonCopyable&& rhs) noexcept
				:Type(std::move(rhs))
			{
			}
			NonCopyable& operator = (NonCopyable&& rhs) noexcept
			{
				Type::operator=(std::move(rhs));
				return *this;
			}
			NonCopyable(NonCopyable const&) = delete;
			NonCopyable& operator = (NonCopyable const&) = delete;
		};
		static_assert(std::is_move_constructible<NonCopyable>::value);
		static_assert(!std::is_copy_constructible<NonCopyable>::value);
		static_assert(std::is_move_assignable<NonCopyable>::value);
		static_assert(!std::is_copy_assignable<NonCopyable>::value);

		// Leaked objects checker
		struct Check
		{
			int m_count;
			long m_refs;

			Check()
				:m_count(object_count)
				, m_refs(ref().use_count())
			{
			}
			~Check()
			{
				PR_EXPECT(object_count == m_count);
				PR_EXPECT(ref().use_count() == m_refs);
			}
		};

		using Deque0 = pr::deque<Type, 8>;
		using Deque1 = pr::deque<Type, 16>;
		using Deque2 = pr::deque<NonCopyable, 4>;

		Check global_chk;
		std::vector<int> ints;
		std::vector<Type> types;

		TestClass_DequeTests()
		{
			for (int i = 0; i != 16; ++i)
			{
				ints.push_back(i);
				types.push_back(Type(i));
			}
		}

		PRUnitTestMethod(Constructors)
		{
			{
				Check chk;
				{
					// default constructor
					Deque0 deq;
					PR_EXPECT(deq.empty());
					PR_EXPECT(deq.size() == 0U);
				}
			} {
				Check chk;
				{
					// construct with allocator
					std::allocator<int> al;
					pr::deque<int, 16, std::allocator<int>> deq0(al);
					deq0.push_back(42);

					// copy to deque of different type
					pr::deque<int, 8, std::allocator<int>> deq1;
					deq1 = deq0;
					PR_EXPECT(deq0.size() == 1U);
					PR_EXPECT(deq0[0] == 42);
					PR_EXPECT(deq1.size() == 1U);
					PR_EXPECT(deq1[0] == 42);
				}
			} {
				Check chk;
				{
					// count constructor
					Deque1 deq(15);
					PR_EXPECT(!deq.empty());
					PR_EXPECT(deq.size() == 15U);
				}
			} {
				Check chk;
				{
					// count + instance constructor
					Deque0 deq(5U, 3);
					PR_EXPECT(deq.size() == 5U);
					for (size_t i = 0; i != 5; ++i)
						PR_EXPECT(deq[i] == 3);
				}
			} {
				Check chk;
				{
					// copy constructor
					Deque0 deq0(5U, 3);
					Deque1 deq1(deq0);
					PR_EXPECT(deq1.size() == deq0.size());
					for (size_t i = 0; i != deq0.size(); ++i)
						PR_EXPECT(deq1[i] == deq0[i]);
				}
			} {
				Check chk;
				{
					// Construct from a std::deque
					std::deque<int> deq0(4U, 6);
					Deque0 deq1(deq0);
					PR_EXPECT(deq1.size() == deq0.size());
					for (size_t i = 0; i != deq0.size(); ++i)
						PR_EXPECT(deq1[i] == deq0[i]);
				}
			} {
				Check chk;
				{
					// Construct from range
					int r[] = { 1, 2, 3, 4 };
					Deque0 deq1(std::begin(r), std::end(r));
					PR_EXPECT(deq1.size() == 4U);
					PR_EXPECT(deq1[0] == r[0]);
					PR_EXPECT(deq1[1] == r[1]);
					PR_EXPECT(deq1[2] == r[2]);
					PR_EXPECT(deq1[3] == r[3]);
				}
			} {
				Check chk;
				{
					// Move construct
					Deque0 deq0(4U, 6);
					Deque0 deq1 = std::move(deq0);
					PR_EXPECT(deq0.size() == 0U);
					PR_EXPECT(deq1.size() == 4U);
					for (size_t i = 0; i != deq1.size(); ++i)
						PR_EXPECT(deq1[i] == 6);
				}
			}
		}
		PRUnitTestMethod(Assign)
		{
			{
				Check chk;
				{
					// Copy assign
					Deque0 deq0(4U, 5);
					Deque1 deq1;
					deq1 = deq0;
					PR_EXPECT(deq0.size() == deq1.size());
					for (size_t i = 0; i != deq0.size(); ++i)
						PR_EXPECT(deq1[i] == deq0[i]);
				}
			} {
				Check chk;
				{
					// Move assign
					Deque0 deq0(4U, 5);
					Deque1 deq1;
					deq1 = std::move(deq0);
					PR_EXPECT(deq0.size() == 0U);
					PR_EXPECT(deq1.size() == 4U);
					for (auto& i : deq1)
						PR_EXPECT(i == 5);
				}
			} {
				Check chk;
				{
					// assign method
					Deque0 deq0;
					deq0.assign(3U, 5);
					PR_EXPECT(deq0.size() == 3U);
					for (size_t i = 0; i != 3; ++i)
						PR_EXPECT(deq0[i] == 5);

					// assign range method
					Deque1 deq1;
					deq1.assign(&types[0], &types[8]);
					PR_EXPECT(deq1.size() == 8U);
					for (size_t i = 0; i != 8; ++i)
						PR_EXPECT(deq1[i] == types[i]);
				}
			}
		}
		PRUnitTestMethod(Clear)
		{
			{
				Check chk;
				{
					pr::deque<int, 8> deq0({ 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 });
					PR_EXPECT(!deq0.empty());
					PR_EXPECT(deq0.front() == 0);
					PR_EXPECT(deq0.back() == 15);
					deq0.clear();
					PR_EXPECT(deq0.empty());
				}
			}
		}
		PRUnitTestMethod(Erase)
		{
			{
				Check chk;
				{
					// erase range non-pods
					Deque0 deq0(std::begin(types), std::end(types));
					auto b = deq0.begin();
					deq0.erase(b + 3, b + 13);
					PR_EXPECT(deq0.size() == 6U);
					for (size_t i = 0; i != 3; ++i) PR_EXPECT(deq0[i] == types[i]);
					for (size_t i = 3; i != 6; ++i) PR_EXPECT(deq0[i] == types[i + 10]);
				}
			}
			{
				Check chk;
				{
					// erase range pods
					pr::deque<int, 8> deq0(std::begin(ints), std::end(ints));
					auto b = deq0.begin();
					deq0.erase(b + 3, b + 13);
					PR_EXPECT(deq0.size() == 6U);
					for (size_t i = 0; i != 3; ++i) PR_EXPECT(deq0[i] == ints[i]);
					for (size_t i = 3; i != 6; ++i) PR_EXPECT(deq0[i] == ints[i + 10]);
				}
			}
			{
				Check chk;
				{
					// erase at
					Deque1 deq1(types.begin(), types.begin() + 4);
					deq1.erase(deq1.begin() + 2);
					PR_EXPECT(deq1.size() == 3U);
					for (size_t i = 0; i != 2; ++i) PR_EXPECT(deq1[i] == types[i]);
					for (size_t i = 2; i != 3; ++i) PR_EXPECT(deq1[i] == types[i + 1]);
				}
			}
		}
		PRUnitTestMethod(Insert)
		{
			{
				Check chk;
				{
					// insert count*val at 'at'
					Deque0 deq0;
					deq0.insert(deq0.end(), 4U, 9);
					PR_EXPECT(deq0.size() == 4U);
					for (size_t i = 0; i != 4; ++i)
						PR_EXPECT(deq0[i] == 9);
				}
			}
			{
				Check chk;
				{
					// insert range
					Deque1 deq1(4U, 6);
					deq1.insert(deq1.begin() + 2, &types[2], &types[7]);
					PR_EXPECT(deq1.size() == 9U);
					for (size_t i = 0; i != 2; ++i) PR_EXPECT(deq1[i] == 6);
					for (size_t i = 2; i != 7; ++i) PR_EXPECT(deq1[i] == types[i]);
					for (size_t i = 7; i != 9; ++i) PR_EXPECT(deq1[i] == 6);
				}
			}
		}
		PRUnitTestMethod(PushPop)
		{
			{
				Check chk;
				{
					// pop_back
					Deque0 deq;
					deq.insert(std::end(deq), std::begin(types), std::begin(types) + 3);
					auto addr = &deq[1];
					deq.insert(std::end(deq), std::begin(types) + 3, std::end(types));
					PR_EXPECT(deq.size() == 16U);
					PR_EXPECT(&deq[1] == addr);
					deq.pop_back();
					deq.pop_back();
					deq.pop_back();
					deq.pop_back();
					PR_EXPECT(deq.size() == 12U);
					PR_EXPECT(&deq[1] == addr);
					for (size_t i = 0; i != deq.size(); ++i)
						PR_EXPECT(deq[i] == types[i]);
				}
			}
			{
				Check chk;
				{
					// push_back
					Deque1 deq;
					deq.emplace_back(0);
					auto addr = &deq[0];
					PR_EXPECT(deq.size() == 1U);

					for (int i = 1; i != 4; ++i) deq.emplace_back(i);
					PR_EXPECT(deq.size() == 4U);
					PR_EXPECT(&deq[0] == addr);

					for (int i = 4; i != 9; ++i) deq.push_back(i);
					PR_EXPECT(deq.size() == 9U);
					PR_EXPECT(&deq[0] == addr);

					for (size_t i = 0; i != deq.size(); ++i)
						PR_EXPECT(deq[i] == types[i]);
				}
			}
			{
				Check chk;
				{
					// pop_front
					Deque0 deq;
					deq.insert(std::end(deq), std::begin(types), std::begin(types) + 8);
					auto addr = &deq[7];
					deq.insert(std::end(deq), std::begin(types) + 8, std::end(types));
					PR_EXPECT(deq.size() == 16U);
					PR_EXPECT(&deq[7] == addr);
					deq.pop_front();
					deq.pop_front();
					deq.pop_front();
					deq.pop_front();
					PR_EXPECT(deq.size() == 12U);
					PR_EXPECT(&deq[3] == addr);
					for (size_t i = 0; i != deq.size(); ++i)
						PR_EXPECT(deq[i] == types[i + 4]);
				}
			}
			{
				Check chk;
				{
					// push_front
					Deque1 deq;
					deq.emplace_front(0);
					auto addr = &deq[0];
					PR_EXPECT(deq.size() == 1U);

					for (int i = 1; i != 4; ++i) deq.emplace_front(i);
					PR_EXPECT(deq.size() == 4U);
					PR_EXPECT(&deq[3] == addr);

					for (int i = 4; i != 9; ++i) deq.push_front(i);
					PR_EXPECT(deq.size() == 9U);
					PR_EXPECT(&deq[8] == addr);

					for (size_t i = 0; i != deq.size(); ++i)
						PR_EXPECT(deq[i].val == 8 - types[i].val);
				}
			}
			{
				Check chk;
				{
					// resize
					Deque0 deq;
					deq.insert(std::begin(deq), types.data(), types.data() + 16);
					PR_EXPECT(deq.size() == 16U);
					deq.resize(7);
					PR_EXPECT(deq.size() == 7U);
					for (size_t i = 0; i != deq.size(); ++i)
						PR_EXPECT(deq[i] == types[i]);
					deq.resize(12);
					PR_EXPECT(deq.size() == 12U);
					for (size_t i = 0; i != 7U; ++i)
						PR_EXPECT(deq[i] == types[i]);
					for (size_t i = 7U; i != 12U; ++i)
						PR_EXPECT(deq[i] == 0);
				}
			}
		}
		PRUnitTestMethod(Operators)
		{
			{
				Check chk;
				{
					// assign and equality
					Deque0 deq0(4U, 1);
					Deque0 deq1(3U, 2);
					deq1 = deq0;
					PR_EXPECT(deq0 == deq1);
					PR_EXPECT(deq0.size() == 4U);
					PR_EXPECT(deq1.size() == 4U);
					for (size_t i = 0; i != 4; ++i)
						PR_EXPECT(deq1[i] == deq0[i]);
				}
			}
			{
				Check chk;
				{
					// inequality between different types
					Deque0 deq0(4U, 1);
					Deque1 deq1;
					deq1 = deq0;
					PR_EXPECT(!(deq0 != deq1));
				}
			}
			{
				Check chk;
				{
					// explicit conversion to std::deque
					Deque0 deq0(4U, 1);
					std::deque<Type> d(deq0.begin(), deq0.end());
					auto deq1 = static_cast<std::deque<Type>>(deq0);
					PR_EXPECT(deq1.size() == 4U);
					for (size_t i = 0; i != 4; ++i)
						PR_EXPECT(deq1[i] == deq0[i]);
				}
			}
		}
		PRUnitTestMethod(Mem)
		{
			Check chk;
			{
				pr::deque<int, 8> deq0;
				const int count = 20;
				for (int i = 0; i != count; ++i) deq0.push_back(i);
				PR_EXPECT(deq0.capacity_front() == 0U);
				PR_EXPECT(deq0.capacity_back() == 4U);

				deq0.push_front(-1);
				PR_EXPECT(deq0.capacity_front() == 7U);
				PR_EXPECT(deq0.capacity_back() == 4U);

				deq0.erase(std::begin(deq0) + 10, std::end(deq0));
				PR_EXPECT(deq0.capacity_front() == 7U);
				PR_EXPECT(deq0.capacity_back() == 15U);
				deq0.erase(std::begin(deq0) + 9, std::end(deq0));
				PR_EXPECT(deq0.capacity_front() == 7U);
				PR_EXPECT(deq0.capacity_back() == 16U);
				deq0.emplace_back(9);

				deq0.shrink_to_fit();
				PR_EXPECT(deq0.capacity_front() == 7U);
				PR_EXPECT(deq0.capacity_back() == 7U);

				deq0.pop_front();
				deq0.pop_back();
				deq0.shrink_to_fit();
				PR_EXPECT(deq0.capacity_front() == 0U);
				PR_EXPECT(deq0.capacity_back() == 0U);

				deq0.resize(0);
				deq0.shrink_to_fit();
				PR_EXPECT(deq0.capacity_front() == 0U);
				PR_EXPECT(deq0.capacity_front() == 0U);
			}
		}
	};
}
#endif
