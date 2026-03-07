//***********************************************************************
// ObjectPool
//  Copyright (c) Rylogic Ltd 2005
//***********************************************************************
#pragma once
#ifndef PR_OBJECT_POOL_H
#define PR_OBJECT_POOL_H

#include <new>
#include "pr/meta/if.h"
#include "pr/meta/is_pod.h"
#include "pr/meta/alignment_of.h"
#include "pr/meta/aligned_storage.h"
#include "pr/common/assert.h"
#include "pr/common/cast.h"

#ifndef NDEBUG
#define PR_OP_CHK 1
#define PR_OP_INITMEM 1
#endif//NDEBUG

// Define this as 1 if you want checking done on the object pool
#ifndef PR_OP_CHK
#define PR_OP_CHK 0
#endif

// Define this as 1 if you're using this in a multi threaded environment
#ifndef PR_OP_MT
#define PR_OP_MT 0
#endif
#if PR_OP_MT
#	include <windows.h>
#endif

// Define this as 1 if you want objects to be initialised with 0xc0c0c0c0
// and overwritten with 0xd0d0d0d0 when returned
#ifndef PR_OP_INITMEM
#define PR_OP_INITMEM 0
#endif
#if PR_OP_INITMEM
#	include <memory.h>
#endif

namespace pr
{
	template <typename Type, std::size_t NumPerBlock>
	class ObjectPool
	{
		template <typename T> struct POD
		{
			static void DestructRange(T*, T*)                                                {}
			static void Destruct(T*)                                                         {}
			static void Construct(T*)                                                        {}
			template <typename P1> static void Construct(T*, P1)                             {}
			template <typename P1, typename P2> static void Construct(T*, P1, P2)            {}
		};
		template <typename T> struct NonPOD
		{
			static void DestructRange(T* first, T* last)                                      { while (first != last) Destruct(first++); }
			static void Destruct(T* target)                                                   { target->T::~T(); }
			static void Construct(T* target)                                                  { new (target) T; }
			template <typename P1> static void Construct(T* target, P1 p1)                    { new (target) T(p1); }
			template <typename P1, typename P2> static void Construct(T* target, P1 p1, P2 p2){ new (target) T(p1); }
		};
		using Constructor = typename meta::if_< meta::is_pod<Type>::value, POD<Type>, NonPOD<Type> >::type;

		// A block contains room for a 'NumPerBlock' array of 'Type'
		struct Block
		{
			enum { SizeInBytes = NumPerBlock * sizeof(Type) };
			typedef typename meta::aligned_storage<SizeInBytes, meta::alignment_of<Type>::value>::type TBuffer;
			byte* buffer() { return pr::byte_ptr(&m_block); }

			TBuffer m_block;
			Block* m_next;
			Block* m_prev;
			Block() :m_next(0) ,m_prev(0) {}
		};
		struct FreeObject
		{
			FreeObject* m_next;
		};

		#if PR_OP_MT
		struct ScopedLock
		{
			HANDLE m_semaphore;
			ScopedLock(HANDLE& semaphore) :m_semaphore(semaphore) { WaitForSingleObject(m_semaphore, INFINITE) == WAIT_OBJECT_0; }
			~ScopedLock()                                         { ReleaseSemaphore(m_semaphore, 1, 0); }
		};
		HANDLE m_semaphore;
		#endif

		enum { InitByte = 0xc0, DestByte = 0xd0 };

		static_assert(sizeof(Type) >= sizeof(FreeObject), "The pooled type must be large enough to contain a pointer");

		Block*      m_current_block;   // Points into a double linked list of Blocks
		uint8_t*    m_block_ptr;       // The pointer within the current block (always 'm_current_block')
		FreeObject* m_free_object;     // A single linked list of returned objects
		PR_EXPAND(PR_OP_CHK, unsigned int m_num_allocated);
		PR_EXPAND(PR_OP_CHK, unsigned int m_num_free);

		// Make 'm_current_block' point to an available block (new or otherwise)
		void GetOrCreateNextBlock()
		{
			// If the current block is in the middle of the list of blocks then move to the next one
			if (m_current_block && m_current_block->m_next != 0)
			{
				m_current_block = m_current_block->m_next;
			}
			// If there is no current block, or if the current block is at the end
			// of the list of blocks, time to allocate a new one
			else
			{
				Block* block = new Block();
				block->m_prev = m_current_block;
				if (m_current_block) m_current_block->m_next = block;
				m_current_block = block;

				PR_EXPAND(PR_OP_CHK, m_num_free      += NumPerBlock);
				PR_EXPAND(PR_OP_CHK, m_num_allocated += NumPerBlock);
			}

			// Point to the end of the current block, objects are allocated by decrementing this pointer
			m_block_ptr = m_current_block->buffer() + Block::SizeInBytes;
		}

		// Get an object from the object pool. If there are objects available in the free
		// list then use them. Otherwise use an object from the current block. If the current
		// block is used up, allocate another block.
		Type* GetInternal()
		{
			Type* object_to_return;
			{
				PR_EXPAND(PR_OP_MT, ScopedLock lock(m_semaphore));

				// If an object is available in the free list use it first
				if (m_free_object)
				{
					object_to_return = reinterpret_cast<Type*>(m_free_object);
					m_free_object = m_free_object->m_next;
				}
				else
				{
					// If the block is used up, create a new block
					if (m_block_ptr == m_current_block->buffer())
						GetOrCreateNextBlock();

					// If this fires then there isn't enough room for a whole 'Type' left in the current block
					// Somethings happened to make the block ptr not a multiple of the sizeof Type
					PR_ASSERT(PR_DBG, m_block_ptr - sizeof(Type) >= m_current_block->buffer(), "");

					// Use some more of the current block
					m_block_ptr -= sizeof(Type);
					object_to_return = reinterpret_cast<Type*>(m_block_ptr);
				}
			}
			PR_EXPAND(PR_OP_CHK, --m_num_free);
			PR_ASSERT(PR_OP_CHK, m_num_free < m_num_allocated, "");
			PR_EXPAND(PR_OP_INITMEM, memset(object_to_return, InitByte, sizeof(Type)));
			return object_to_return;
		}

		ObjectPool(ObjectPool const&);
		ObjectPool& operator = (ObjectPool const&);

	public:
		ObjectPool()
		:m_current_block(0)
		,m_block_ptr(0)
		,m_free_object(0)
		{
			PR_EXPAND(PR_OP_CHK, m_num_allocated = 0);
			PR_EXPAND(PR_OP_CHK, m_num_free      = 0);

			// Create a semaphore for protecting access to the object and block list
			PR_EXPAND(PR_OP_MT, m_semaphore = CreateSemaphore(0, 1/*initial*/, 1/*max*/, 0));
			PR_ASSERT(PR_OP_MT, m_semaphore != 0, "");
			PR_EXPAND(PR_OP_MT, ScopedLock lock(m_semaphore));
			GetOrCreateNextBlock();
		}

		~ObjectPool()
		{
			PR_ASSERT(PR_OP_CHK, m_num_allocated == m_num_free, "Some objects not returned to the pool");
			PR_EXPAND(PR_OP_MT, ScopedLock lock(m_semaphore));

			// Find the end of the block list
			Block* block = m_current_block;
			while (block->m_next) block = block->m_next;

			// Delete all of the blocks
			while (block)
			{
				Block* current = block;
				block = block->m_prev;
				delete current;
			}
		}

		Type* Get()
		{
			Type* object = GetInternal();
			Constructor::Construct(object);
			return object;
		}
		template <typename P1> Type* Get(P1 p1)
		{
			Type* object = GetInternal();
			Constructor::Construct(object, p1);
			return object;
		}
		template <typename P1, typename P2> Type* Get(P1 p1, P2 p2)
		{
			Type* object = GetInternal();
			Constructor::Construct(object, p1, p2);
			return object;
		}

		// Return an object to the pool.
		void Return(Type* object)
		{
			// Destruct the object
			Constructor::Destruct(object);
			PR_EXPAND(PR_OP_INITMEM, memset(object, DestByte, sizeof(Type)));
			PR_EXPAND(PR_OP_CHK, ++m_num_free);
			PR_ASSERT(PR_OP_CHK, m_num_free <= m_num_allocated, "");

			// Add the object to the list of free objects
			PR_EXPAND(PR_OP_MT, ScopedLock lock(m_semaphore));
			FreeObject& dead_object = *reinterpret_cast<FreeObject*>(object);
			dead_object.m_next = m_free_object;
			m_free_object = &dead_object;
		}

		// Assume all pooled objects are returned to the pool.
		// This can only be implemented for pod types as there is no way to tell which
		// objects in which block have already been destructed and are in the free list
		// For pod types destruction isn't necessary.
		// BE CAREFUL USING THIS. This method effectively pulls the objects back
		// from whereever they're being used and deletes them
		void ReclaimAll()
		{
			PR_ASSERT(PR_DBG, meta::is_pod<Type>::value, "This method can only be used for POD types");
			PR_EXPAND(PR_OP_MT, ScopedLock lock(m_semaphore));

			// Move 'm_current_block' to the first in the list
			PR_EXPAND(PR_OP_INITMEM, memset(m_current_block->buffer(), DestByte, sizeof(m_current_block->buffer())));
			while (m_current_block->m_prev)
			{
				m_current_block = m_current_block->m_prev;
				PR_EXPAND(PR_OP_INITMEM, memset(m_current_block->buffer(), DestByte, sizeof(m_current_block->buffer())));
			}

			// Point to the first position to allocate from (i.e. the end of the first block)
			m_block_ptr = m_current_block->buffer() + Block::SizeInBytes;
			m_free_object = 0;
			PR_EXPAND(PR_OP_CHK, m_num_free = m_num_allocated);
		}
	};
}

#undef PR_OP_CHK
#undef PR_OP_MT
#undef PR_OP_INITMEM

#endif