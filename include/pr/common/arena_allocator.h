//******************************************
// Arena Allocator
//  Copyright (c) Oct 2025 Rylogic
//******************************************
#pragma once
#include <cassert>
#include <type_traits>
#include <memory>
#include <vector>

#ifndef PR_DBG_ARENA_ALLOCATOR
#define PR_DBG_ARENA_ALLOCATOR 0
#endif

namespace pr
{
	// An instantiable arena allocator
	template <size_t BlockSize, int Alignment, int ReallocGrowNumer = 1, int ReallocGrowDenom = 1>
	struct ArenaAllocator
	{
		// Notes:
		//  - Each allocation has a header indicating the size and capacity. This is needed for realloc support.
		//  - Allocations are never moved or freed. Freeing an allocation just marks it as unused.
		//  - The arena can be cleared or swept to remove unused blocks.
		//  - Large allocations (bigger than the block size) get their own block.

		struct AllocHeader
		{
			#pragma warning(push)
			#pragma warning(disable:4201) // nameless struct
			union {
				struct {
					size_t size; // The size of the allocation *excluding* this header
					size_t used; // The requested size of the allocation *excluding* this header
				};
				std::byte _[Alignment];
			};
			#pragma warning(pop)
		};
		struct Block
		{
			struct AlignedDeleter { void operator()(std::byte* p) { _aligned_free(p); } };
			using MemPtr = std::unique_ptr<std::byte, AlignedDeleter>;

			MemPtr mem;  // The allocating memory block
			size_t size; // The size of 'mem' (in bytes)
			size_t used; // The used space in 'mem' (in bytes)

			// Allocate a portion of this block
			AllocHeader* Alloc(size_t n)
			{
				assert(used + n <= size && "Allocation overflows the block");
				assert(n >= sizeof(AllocHeader) && "Each allocation from the block should have an AllocHeader");
				auto* alloc = reinterpret_cast<AllocHeader*>(mem.get() + used);
				used += n;
				return alloc;
			}

			// Enumerate the allocations in this block
			auto Enumerate() const
			{
				struct I
				{
					Block const* block;
					size_t offset;

					AllocHeader const& operator*() const
					{
						return *reinterpret_cast<AllocHeader const*>(block->mem.get() + offset);
					}
					I& operator++()
					{
						auto const* alloc = reinterpret_cast<AllocHeader const*>(block->mem.get() + offset);
						offset += sizeof(AllocHeader) + alloc->size;
						return *this;
					}
					bool operator!=(I const& rhs) const
					{
						return offset != rhs.offset;
					}
				};
				struct R
				{
					Block const* block;
					auto begin() const { return I{ block, 0 }; }
					auto end() const { return I{ block, block->used }; }
				};
				return R{ this };
			}
		};

		static_assert(sizeof(AllocHeader) % Alignment == 0);

		using Blocks = std::vector<Block>;
		Blocks m_blocks;
		Blocks m_large;

		ArenaAllocator()
			: m_blocks()
			, m_large()
		{}
		ArenaAllocator(ArenaAllocator&&) = default;
		ArenaAllocator(ArenaAllocator const&) = delete;
		ArenaAllocator& operator=(ArenaAllocator&&) = default;
		ArenaAllocator& operator=(ArenaAllocator const&) = delete;

		// Release the arena memory
		void Clear()
		{
			m_blocks.clear();
			m_large.clear();
		}

		// Remove blocks that contain only freed allocations
		void Sweep()
		{
			constexpr auto Unused = [](Block const& block)
			{
				for (auto const& alloc : block.Enumerate())
				{
					if (alloc.used != 0)
						return false;
				}
				return true;
			};
			m_blocks.erase(std::remove_if(std::begin(m_blocks), std::end(m_blocks), Unused), m_blocks.end());
			m_large.erase(std::remove_if(std::begin(m_large), std::end(m_large), Unused), m_large.end());
		}

		// Preallocate a block of size 'capacity'
		void Preallocate(size_t capacity)
		{
			m_blocks.push_back(NewBlock(capacity));
		}

		// Allocate n bytes
		void* Malloc(size_t n)
		{
			auto* alloc = Alloc(n);
			return alloc + 1;
		}

		// Allocate zeroed memory (n*sz bytes)
		void* Calloc(size_t n, size_t sz)
		{
			auto* alloc = Alloc(n * sz);
			std::memset(alloc + 1, 0, n * sz);
			return alloc + 1;
		}

		// Reallocate p to n bytes
		void* Realloc(void* p, size_t n)
		{
			if (p == nullptr)
			{
				return Malloc(n);
			}
			if (n == 0)
			{
				Free(p);
				return nullptr;
			}

			assert(!PR_DBG_ARENA_ALLOCATOR || IsValidBlock(p) && "Pointer was not allocated from this arena");
			auto* alloc = static_cast<AllocHeader*>(p) - 1;
			if (alloc->size >= n)
			{
				alloc->used = n;
				return alloc + 1;
			}

			auto* realloc = Alloc((n * ReallocGrowNumer) / ReallocGrowDenom);
			std::memcpy(realloc + 1, alloc + 1, alloc->used);
			Free(p);
			return realloc + 1;
		}

		// Free an allocation in the arena. This doesn't really do anything because it's an arena.
		void Free(void* p)
		{
			if (p == nullptr) return;
			assert(!PR_DBG_ARENA_ALLOCATOR || IsValidBlock(p) && "Pointer was not allocated from this arena");
			auto* alloc = static_cast<AllocHeader*>(p) - 1;
			alloc->used = 0;
		}

		// True if 'p' was allocated from this arena
		bool IsValidBlock(void const* p) const
		{
			// Find the block that 'p' is within
			constexpr auto find_block = [](void const* ptr, Blocks const& blocks) -> Block const*
			{
				for (auto const& b : blocks)
				{
					if (ptr >= b.mem.get() && ptr < b.mem.get() + b.size)
						return &b;
				}
				return nullptr;
			};
			Block const* block = nullptr;
			if (block == nullptr) block = find_block(p, m_blocks);
			if (block == nullptr) block = find_block(p, m_large);
			if (block == nullptr) return false;

			// Check that 'p' is the start of an allocation within the arena
			for (auto const& alloc : block->Enumerate())
			{
				if (p == &alloc + 1)
					return true;
			}

			return false;
		}

		// The number of blocks in the arena
		size_t BlockCount() const
		{
			return m_blocks.size() + m_large.size();
		}

		// The allocated size of the arena
		size_t SizeInBytes() const
		{
			size_t sz = 0;
			for (auto const& b : m_blocks) sz += b.size;
			for (auto const& b : m_large) sz += b.size;
			return sz;
		}

		// The used space in the arena.
		size_t Occupancy() const
		{
			size_t sz = 0;
			for (auto const& b : m_blocks)
				for (auto const& a : b.Enumerate())
					sz += a.used;
			for (auto const& b : m_large)
				for (auto const& a : b.Enumerate())
					sz += a.used;
			return sz;
		}

		// Pad 'n' out to a multiple of the alignment
		static constexpr size_t Pad(size_t n)
		{
			assert(n <= SIZE_MAX - Alignment);
			auto rem = n % Alignment;
			return n + (rem != 0 ? Alignment - rem : 0);
		}

	private:

		// Allocate an aligned block of size at least 'n'
		AllocHeader* Alloc(size_t n)
		{
			auto N = Pad(n); // Pad up to the alignment

			// Try to allocate from the current block
			static Block empty_block{};
			if (auto& block = !m_blocks.empty() ? m_blocks.back() : empty_block; sizeof(AllocHeader) + N <= block.size - block.used)
			{
				auto* alloc = block.Alloc(sizeof(AllocHeader) + N);
				alloc->size = N;
				alloc->used = n;
				return alloc;
			}

			// If this is a large allocation, create a large block
			else if (sizeof(AllocHeader) + N > BlockSize)
			{
				m_large.push_back(NewBlock(sizeof(AllocHeader) + N));
				auto* alloc = m_large.back().Alloc(sizeof(AllocHeader) + N);
				alloc->size = N;
				alloc->used = n;
				return alloc;
			}

			// Otherwise, add a new default size block
			else
			{
				m_blocks.push_back(NewBlock(BlockSize));
				auto* alloc = m_blocks.back().Alloc(sizeof(AllocHeader) + N);
				alloc->size = N;
				alloc->used = n;
				return alloc;
			}
		}

		// Allocate a block of memory
		Block NewBlock(size_t n)
		{
			return Block{
				.mem = typename Block::MemPtr{ static_cast<std::byte*>(_aligned_malloc(n, Alignment)) },
				.size = n,
				.used = 0,
			};
		}
	};
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
namespace pr::common
{
	PRUnitTest(ArenaAllocatorTests, Quick)
	{
		using Arena = ArenaAllocator<1024, 16>;
		Arena arena;

		// Simple alloc/free
		void* p1 = arena.Malloc(100);
		void* p2 = arena.Malloc(200);
		void* p3 = arena.Malloc(300);
		PR_EXPECT(arena.IsValidBlock(p1));
		PR_EXPECT(arena.IsValidBlock(p2));
		PR_EXPECT(arena.IsValidBlock(p3));
		PR_EXPECT(arena.Occupancy() == 600);

		arena.Free(p2);
		PR_EXPECT(arena.Occupancy() == 400);

		void* p4 = arena.Malloc(150);
		PR_EXPECT(arena.IsValidBlock(p4));
		PR_EXPECT(arena.Occupancy() == 550);

		arena.Free(p1);
		arena.Free(p3);
		arena.Free(p4);
		PR_EXPECT(arena.Occupancy() == 0);

		PR_EXPECT(arena.BlockCount() == 1);
		PR_EXPECT(arena.SizeInBytes() == 1024);

		// Realloc larger
		p1 = arena.Malloc(100);
		p2 = arena.Realloc(p1, 200);
		PR_EXPECT(p1 != p2);
		PR_EXPECT(arena.Occupancy() == 200);
		arena.Free(p2);
		PR_EXPECT(arena.Occupancy() == 0);

		// Realloc smaller
		p1 = arena.Malloc(100);
		p2 = arena.Realloc(p1, 50);
		PR_EXPECT(p1 == p2);
		PR_EXPECT(arena.Occupancy() == 50);
		arena.Free(p2);
		PR_EXPECT(arena.Occupancy() == 0);

		PR_EXPECT(arena.BlockCount() == 2);
		PR_EXPECT(arena.SizeInBytes() == 2048);

		// Alloc large block
		p1 = arena.Malloc(2000);
		PR_EXPECT(arena.IsValidBlock(p1));
		arena.Free(p1);
		PR_EXPECT(arena.Occupancy() == 0);

		PR_EXPECT(arena.BlockCount() == 3);
		PR_EXPECT(arena.SizeInBytes() == 2*1024 + 2000 + sizeof(typename Arena::AllocHeader));

		arena.Sweep();
		PR_EXPECT(arena.BlockCount() == 0);

		// Alloc after clear
		p1 = arena.Malloc(100);
		PR_EXPECT(arena.IsValidBlock(p1));
		arena.Clear();
		PR_EXPECT(!arena.IsValidBlock(p1));
		p2 = arena.Malloc(100);
		PR_EXPECT(arena.IsValidBlock(p2));
	}
}
#endif
