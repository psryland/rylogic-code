//*********************************************
// Physics engine
//  Copyright (c) Rylogic Ltd 2006
//*********************************************

#pragma once
#ifndef PR_PHYSICS_COLLISION_COUPLE_H
#define PR_PHYSICS_COLLISION_COUPLE_H

#include "pr/physics/types/forward.h"
#include "pr/physics/shape/shape.h"
#include "pr/physics/collision/collisioncache.h"
#include "physics/collision/simplex.h"

namespace pr
{
	namespace ph
	{
		namespace mesh_vs_mesh
		{
			struct Couple
			{
				collision::CacheData* m_cache_data;		// A pointer to the cache entry for this pair
				Shape const&	m_shapeA;
				m4x4 const&		m_a2w;
				Shape const&	m_shapeB;
				m4x4 const&		m_b2w;
				m3x4			m_w2a;
				m3x4			m_w2b;
				Vert			m_vertex;				// The support vertex. Updated by calls to 'SupportVertex'
				Simplex			m_simplex;				// A polytope with upto 4 vertices within the Minkowski difference

				// Cached data
				v4				m_separating_axis;		// The current best estimate of the separating axis
				std::size_t		m_hint_id_p;			// A support vertex hint for object A
				std::size_t		m_hint_id_q;			// A support vertex hint for object B

				// Penetration members
				Vert			m_nearest;				// The vertex at the nearest point
				float			m_dist_sq_upper_bound;	// Upper bound on the distance to the nearest point

				Couple(Shape const& shapeA, m4x4 const& a2w, Shape const& shapeB, m4x4 const& b2w, CollisionCache* cache);
				~Couple();
				Couple(Couple const&);				// No copying
				Couple& operator=(Couple const&);	// No copying
				void CacheSeparatingAxis(Triangle const& tri);

				// Get the support vertices for 'shapeA' and 'shapeB' given 'diirection'
				void SupportVertex(v4 direction)
				{
					m_vertex.m_direction = direction;
					m_vertex.m_p = m_a2w * pr::ph::SupportVertex(m_shapeA, m_w2a *  direction, m_hint_id_p, m_vertex.m_id_p);
					m_vertex.m_q = m_b2w * pr::ph::SupportVertex(m_shapeB, m_w2b * -direction, m_hint_id_q, m_vertex.m_id_q);
					m_vertex.m_r = m_vertex.m_p - m_vertex.m_q + v4::Origin();
					m_hint_id_p  = m_vertex.m_id_p;
					m_hint_id_q  = m_vertex.m_id_q;
				}
			};
		}
	}
}

#endif
