//*********************************************
// Physics engine
//  Copyright (c) Rylogic Ltd 2006
//*********************************************

#include "physics/utility/stdafx.h"
#include "pr/physics/collision/collisioncache.h"
#include "physics/collision/collisioncouple.h"

using namespace pr;
using namespace pr::ph;
using namespace pr::ph::collision;

// Hash two shape pointers together to give a cache entry index
namespace pr { namespace ph { namespace collision
{
	inline std::size_t Hash(Shape const* shapeA, Shape const* shapeB)
	{
		std::size_t a = byte_ptr(shapeA) - byte_ptr(nullptr);
		std::size_t b = byte_ptr(shapeB) - byte_ptr(nullptr);
		return (a ^ b) % CollisionCache::MaxEntries;
	}
}}}

// Update the cache entry for a collision couple
void CacheData::Update(v4 sep_axis, std::size_t p_id, std::size_t q_id)
{
	m_separating_axis	= sep_axis;
	m_p_id				= p_id;
	m_q_id				= q_id;
}

// Collision cache *****************************************

CollisionCache::CollisionCache()
{
	memset(m_data, 0, sizeof(m_data));
	m_counter = 2;
}

// Look for cached information 
// This method returns one of three possibilities:
//	- true , cache_entry != 0 -> Cached data was found for this pair of primitives
//	- false, cache_entry != 0 -> Cached data was not found but a cache entry slot is available
//	- false, cache_entry == 0 -> Cached data was not found and the appropriate slot is unavailable
bool CollisionCache::Lookup(Shape const* shapeA, Shape const* shapeB, pr::ph::collision::CacheData*& cache_entry)
{
	//#if ASSERTS_PHYSICS == 1
	static bool enable_collision_cache = true;
	if( !enable_collision_cache ) { cache_entry = 0; return false; }
	//#endif//ASSERTS_PHYSICS == 1

	std::size_t hash = collision::Hash(shapeA, shapeB);
	collision::CacheData& data = m_data[hash];
	
	// If the slot contains information for this pair of shapes then use it
	if( data.m_shapeA == shapeA && data.m_shapeB == shapeB )
	{
		data.m_last_used = m_counter;
		cache_entry = &data;
		return true;
	}
	if( data.m_shapeA == shapeB && data.m_shapeB == shapeA )
	{
		data.Swap();
		data.m_last_used = m_counter;
		cache_entry = &data;
		return true;
	}

	// If the slot is available (i.e. it hasn't been used recently),
	// reserve it, and return a failed cache lookup
	if( m_counter - data.m_last_used > 1 )
	{
		data.m_last_used	= m_counter;
		data.m_shapeA		= shapeA;
		data.m_shapeB		= shapeB;
		cache_entry			= &data;
		return false;
	}

	// Otherwise the slot is currently in use by a different pair
	cache_entry = 0;
	return false;
}
