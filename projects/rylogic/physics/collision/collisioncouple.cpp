//*********************************************
// Physics engine
//  Copyright (c) Rylogic Ltd 2006
//*********************************************

#include "physics/utility/stdafx.h"
#include "physics/collision/collisioncouple.h"
#include "pr/physics/collision/collisioncache.h"

using namespace pr;
using namespace pr::ph;
using namespace pr::ph::mesh_vs_mesh;

Couple::Couple(Shape const& shapeA, m4x4 const& a2w, Shape const& shapeB, m4x4 const& b2w, CollisionCache* cache)
:m_cache_data	(0)
,m_shapeA		(shapeA)
,m_a2w			(a2w)
,m_shapeB		(shapeB)
,m_b2w			(b2w)
,m_w2a			(InvertAffine(a2w.rot))
,m_w2b			(InvertAffine(b2w.rot))
,m_hint_id_p	(0)
,m_hint_id_q	(0)
,m_dist_sq_upper_bound(limits<float>::max())
{
	// Look in the cache for an entry for this pair of objects.
	if( !cache || !cache->Lookup(&shapeA, &shapeB, m_cache_data) )
	{
		// If these shapes are overlapping for the first time, get the initial
		// separating vector from the difference in positions.
		m_separating_axis = m_b2w.pos - m_a2w.pos;
		if(m_separating_axis == v4::Zero())	{ m_separating_axis = v4::XAxis(); }
		else								{ m_separating_axis = Normalise(m_separating_axis); }
	}
	else
	{
		m_separating_axis = m_cache_data->m_separating_axis;
		m_hint_id_p = m_cache_data->m_p_id;
		m_hint_id_q = m_cache_data->m_q_id;
	}
}

Couple::~Couple()
{
	// On destruction, update the cache entry with the results of the collision test
	if( m_cache_data )
	{
		m_cache_data->Update(m_separating_axis, m_hint_id_p, m_hint_id_q);
	}
}

// Save the support direction and hint ids from a vertex
void Couple::CacheSeparatingAxis(Triangle const& tri)
{
	m_separating_axis	= tri.m_direction;
	m_hint_id_p			= tri.m_vert[0].m_id_p;
	m_hint_id_q			= tri.m_vert[0].m_id_q;
}

