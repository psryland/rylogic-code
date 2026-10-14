//***********************************************************************
// Memory Pool
//  Copyright (c) Rylogic Ltd 2001
//***********************************************************************
//
// MemPool allows objects of any type to be "re-cycled" and avoid excessive
// new() and delete() calls.
//
// The pooled object must provide a Next() method that returns a reference
// to a pointer to another similar object. This is so the objects can be
// queued. Another good idea is to include new and delete operators for
// a mempooled class. This requires a globally defined MemPool object.
//
//	Optional defines:
//		INITIALISE_MEMORY	-	Pooled objects are initialise to 0xCDCDCDCD on Get()
//								and set to 0xDDDDDDDD on Return()
//
//	e.g.
//      const int OBJECTS_PER_BLOCK = 10;   // The number of objects to allocate at a time
//		class Thing;
//		MemPool<Thing> g_Thing_Pool(OBJECTS_PER_BLOCK);
//		class Thing
//		{
//		public:
//			Thing* m_next;
//			void* operator new(size_t)			{ return g_Thing_Pool.Get(false);			}
//			void  operator delete(void* obj)	{ g_Thing_Pool.Return((Thing*&)obj, false);	}
//		};
//
#ifndef MEMPOOL_H
#define MEMPOOL_H

#include <new>
#include <utility>
#include "pr/meta/if.h"
#include "pr/meta/is_pod.h"
#include "pr/common/assert.h"

#ifdef MEMPOOL_THREADSAFE
	#include <windows.h>
	#define MP_THREADSAFE(exp) exp
#else//MEMPOOL_THREADSAFE
	#define MP_THREADSAFE(exp)
#endif//MEMPOOL_THREADSAFE

#ifdef INITIALISE_MEMORY
	#include <memory.h>
	#undef  INITIALISE_MEMORY
	#define INITIALISE_MEMORY(exp) exp
#else//INITIALISE_MEMORY
	#define INITIALISE_MEMORY(exp)
#endif//INITIALISE_MEMORY

namespace pr
{
	// The memory pool
	template <typename T>
	class MemPool
	{
	public:
		MemPool(uint32_t estimated_size = 1);
		MemPool(const MemPool<T>& copy);
		~MemPool();
		MemPool<T>& operator = (const MemPool<T>& copy)
		{
			// Build the copy first so a failed allocation leaves this pool unchanged, then swap so the temporary frees the old blocks.
			// The semaphore is not swapped because it guards this object, not its contents.
			auto temp = MemPool<T>(copy);
			std::swap(m_block_list, temp.m_block_list);
			std::swap(m_object_list, temp.m_object_list);
			std::swap(m_objects_per_block, temp.m_objects_per_block);
			std::swap(m_allocated_objects, temp.m_allocated_objects);
			std::swap(m_free_objects, temp.m_free_objects);
			std::swap(m_block_ptr, temp.m_block_ptr);
			return *this;
		}

		T*	 Get();
		void Return(T* object);
		void ReclaimAll();
		void ReleaseMemory();
		void ForceReleaseMemory();

		uint32_t GetNumberOfObjectsPerBlock() const			{ return m_objects_per_block; }
		uint32_t GetNumberOfFreeObjects() const				{ return m_free_objects; }
		uint32_t GetNumberOfAllocatedObjects() const		{ return m_allocated_objects; }
		void SetNumberOfObjectsPerBlock(uint32_t number)	{ m_objects_per_block = number; }

		bool AllObjectsReturned() const				{ return m_allocated_objects == m_free_objects; }

	private:
		template <typename T>
		struct NonPOD
		{
			static void Construct(T* target)				{ new (target) T; }
			static void Destruct(T* target)					{ target->T::~T(); }
			static void DestructRange(T* first, T* last)	{ while( first < last ) {T* temp = first; ++first; temp->T::~T();} }
		};
		template <typename T>
		struct POD
		{
			static void Construct(T*)			{}
			static void Destruct(T*)			{}
			static void DestructRange(T*, T*)	{}
		};
		typedef typename meta::if_< meta::is_pod<T>::value, POD<T>, NonPOD<T> >::type Constructor;

		struct Block
		{
			Block(uint32_t number_of_objects) : m_number_of_objects(number_of_objects), m_next(0), m_prev(0)
			{	m_memory = new uint8_t[m_number_of_objects * sizeof(T)]; }
			~Block() { delete [] m_memory; }
			uint8_t*	m_memory;
			uint32_t	m_number_of_objects;
			Block*	m_next;
			Block*	m_prev;
		};

	private:
		#ifdef MEMPOOL_THREADSAFE
		bool Lock()		{ return WaitForSingleObject(m_semaphore, INFINITE) == WAIT_OBJECT_0; }
		void Unlock()	{ ReleaseSemaphore( m_semaphore, 1, 0 ); }
		HANDLE	m_semaphore;
		#else//MEMPOOL_THREADSAFE
		bool Lock()		{ return true; }
		void Unlock()	{}
		#endif//MEMPOOL_THREADSAFE

		bool GetOrCreateNextBlock();

	private:
		Block*	m_block_list;
		T*		m_object_list;
		uint32_t	m_objects_per_block;
		uint32_t	m_allocated_objects;
		uint32_t	m_free_objects;
		uint32_t	m_block_ptr;
	};

	//***********************************************************************//
	// Implementation
	//*****
	// Constructor
	template <typename T>
	MemPool<T>::MemPool(uint32_t estimated_size)
	:m_block_list(0)
	,m_object_list(0)
	,m_objects_per_block((estimated_size) ? (estimated_size) : (1))
	,m_allocated_objects(0)
	,m_free_objects(0)
	,m_block_ptr(0)
	{
		// Create a semaphore for protecting access to the object and block list
		MP_THREADSAFE(m_semaphore = CreateSemaphore(0, 1/*initial*/, 1/*max*/, 0);)
		MP_THREADSAFE(PR_ASSERT(PR_DBG, m_semaphore != 0, "");)

		Lock();
		GetOrCreateNextBlock();
		Unlock();
	}

	//*****
	// Copy constructor
	template <typename T>
	MemPool<T>::MemPool(const MemPool<T>& copy)
	:m_block_list(0)
	,m_object_list(0)
	,m_objects_per_block(copy.m_objects_per_block)
	,m_allocated_objects(0)
	,m_free_objects(0)
	,m_block_ptr(0)
	{
		PR_ASSERT(PR_DBG, copy.m_allocated_objects == copy.m_free_objects, "You are copying a mempool that has objects allocated from it");
		Lock();
		GetOrCreateNextBlock();
		Unlock();
	}

	//*****
	// Destructor
	template <typename T>
	inline MemPool<T>::~MemPool()
	{
		// Make sure all objects have been returned to the pool
		// To avoid apparent "Memory leaks" when using a global memory pool
		// Use ReleaseMemory() before the memory pool is destroyed
		PR_ASSERT(PR_DBG, AllObjectsReturned(), "");
		ReleaseMemory();
	}

	//*****
	// Get an object from the object pool. If there are objects available in the object
	// list then use them. Otherwise use an object from the current block. If the current
	// block is used up, allocate another block.
	template <typename T>
	T* MemPool<T>::Get()
	{
		Lock();
		T* object_to_return = 0;

		// If an object is available in the list use it first
		if( m_object_list )
		{
			PR_ASSERT(PR_DBG, m_free_objects > 0, "");
			object_to_return = m_object_list;
			m_object_list = m_object_list->m_next;
			--m_free_objects;
		}
		else
		{
			// If the block is used up, create a new block
			if( !m_block_list || m_block_ptr == m_block_list->m_number_of_objects )
			{
				if( !GetOrCreateNextBlock() ) { Unlock(); return 0; }
			}

			// Use some more of the current block
			PR_ASSERT(PR_DBG, m_block_ptr < m_block_list->m_number_of_objects, "");
			object_to_return = reinterpret_cast<T*>(&m_block_list->m_memory[m_block_ptr * sizeof(T)]);
			++m_block_ptr;
			--m_free_objects;
		}
		Unlock();

		INITIALISE_MEMORY(memset(object_to_return, 0xcd, sizeof(T));)
		object_to_return->m_next = 0;

		Constructor::Construct(reinterpret_cast<T*>(object_to_return));
		return object_to_return;
	}

	//*****
	// Return an object to the pool.
	template <typename T>
	void MemPool<T>::Return(T* object)
	{
		if( object == 0 ) return;

		Constructor::Destruct(object);

		PR_ASSERT(PR_DBG, object->m_next == 0, "");					// This object has already been deleted
		INITIALISE_MEMORY(memset(object, 0xDD, sizeof(T));)
		PR_ASSERT(PR_DBG, m_free_objects < m_allocated_objects, "");

		Lock();
		object->m_next = m_object_list;
		m_object_list = object;
		++m_free_objects;
		Unlock();
	}

	//*****
	// Assume all pooled objects are returned to the pool.
	// BE CAREFUL USING THIS. This method effectively pulls the objects back
	// from whereever they're being used and deletes them
	template <typename T>
	void MemPool<T>::ReclaimAll()
	{
		if( !m_block_list ) return;
		Lock();

		// Set the block pointer back to the start of the first block
		for(;;)
		{
			Constructor::DestructRange(
				reinterpret_cast<T*>(&m_block_list->m_memory[0]),
				reinterpret_cast<T*>(&m_block_list->m_memory[m_block_ptr * sizeof(T)]));

			// We're at the first block, done destructing
			if( !m_block_list->m_prev ) break;

			// Otherwise go to the previous block and destruct that
			m_block_list = m_block_list->m_prev;
			m_block_ptr	 = m_block_list->m_number_of_objects;
		}

		m_free_objects = m_allocated_objects;
		m_object_list = 0;
		m_block_ptr = 0;
		Unlock();
	}

	//*****
	// Release the pooled memory. This can only happen if all
	// objects have been returned to the pool
	template <typename T>
	inline void MemPool<T>::ReleaseMemory()
	{
		PR_ASSERT(PR_DBG, AllObjectsReturned(), "");	// This assert indicate leaked objects!
		ForceReleaseMemory();
	}

	//*****
	// Release the pooled memory even if it has not been returned to the pool.
	// This is useful if when objects are scattered around the place but we want
	// to delete all of them. E.g. destroying nodes in a list
	template <typename T>
	void MemPool<T>::ForceReleaseMemory()
	{
		Lock();

		// Navigate back to the first block
		while( m_block_list && m_block_list->m_prev )
			m_block_list = m_block_list->m_prev;

		while( m_block_list )
		{
			m_free_objects		-= m_block_list->m_number_of_objects;
			m_allocated_objects -= m_block_list->m_number_of_objects;
			Block* temp = m_block_list->m_next;
			delete m_block_list;
			m_block_list = temp;
		}
		m_object_list = 0;
		m_block_ptr = 0;
		Unlock();
	}

	//***********************************************************************************
	// Private MemPool methods
	//*****
	// Make 'm_block_list' point to an available block (new or otherwise)
	template <typename T>
	bool MemPool<T>::GetOrCreateNextBlock()
	{
		PR_ASSERT(PR_DBG, m_objects_per_block != 0, "");

		if( m_block_list == 0 )
		{
			m_block_list = new Block(m_objects_per_block);
			m_allocated_objects += m_objects_per_block;
			m_free_objects += m_objects_per_block;
		}
		else if( m_block_list->m_next == 0 )
		{
			PR_ASSERT(PR_DBG, m_block_ptr == m_block_list->m_number_of_objects, "");
			Block* block = new Block(m_objects_per_block);
			m_allocated_objects += m_objects_per_block;
			m_free_objects += m_objects_per_block;
			m_block_list->m_next = block;
			block->m_prev = m_block_list;
			m_block_list = block;
		}
		else
		{
			PR_ASSERT(PR_DBG, m_block_ptr == m_block_list->m_number_of_objects, "");
			m_block_list = m_block_list->m_next;
		}

		m_block_ptr = 0;
		return true;
	}
}//namespace pr

#undef MP_THREADSAFE
#undef INITIALISE_MEMORY

#endif//MEMPOOL_H
