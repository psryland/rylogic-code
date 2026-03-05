//*********************************************
// Physics engine
//	(c)opyright Paul Ryland 2006
//*********************************************

#ifndef PR_PHYSICS_COLLISION_CACHE_H
#define PR_PHYSICS_COLLISION_CACHE_H
#pragma once

#include "pr/physics/types/forward.h"

namespace pr
{
	namespace ph
	{
		namespace collision
		{
			struct CacheData
			{
				const Shape*	m_shapeA;			// The address of the first object the cache entry is for
				const Shape*	m_shapeB;			// The address of the second object the cache entry is for
				v4				m_separating_axis;	// The best estimate of the separating axis from previous frames
				std::size_t		m_p_id;				// The vertex id last used on shapeA
				std::size_t		m_q_id;				// The vertex id last used on shapeB
				std::size_t		m_last_used;		// The counter value when this cache entry was last used

				void Update(v4 sep_axis, std::size_t p_id, std::size_t q_id);
				void Swap() { std::swap(m_shapeA, m_shapeB); std::swap(m_p_id, m_q_id); m_separating_axis = -m_separating_axis; }
			};
		}

		struct CollisionCache
		{
			enum { MaxEntries = pr::meta::prime_gtreq<1000>::value };
			collision::CacheData	m_data[MaxEntries];
			std::size_t				m_counter;				// A rolling counter used to identify cache slots that haven't been used recently

			CollisionCache();
			CollisionCache(CollisionCache const&);			// No copying
			CollisionCache& operator=(CollisionCache const&);	// No copying

			// Call this at the start of each frame to clear the 'used' flags
			void FrameStart() { ++m_counter; }

			// Look for cached information.
			// This method returns one of three possibilities:
			//	- true , cache_entry != 0 -> Cached data was found for this pair of primitives
			//	- false, cache_entry != 0 -> Cached data was not found but a cache entry slot is available
			//	- false, cache_entry == 0 -> Cached data was not found and the appropriate slot is unavailable
			bool Lookup(Shape const* shapeA, Shape const* shapeB, collision::CacheData*& cache_entry);
		};
	}
}

#endif
