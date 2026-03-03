//*********************************************
// Physics engine
//  Copyright (c) Rylogic Ltd 2006
//*********************************************

#pragma once
#ifndef PR_PHYSICS_TERRAIN_PLANE_H
#define PR_PHYSICS_TERRAIN_PLANE_H

#include "pr/physics/types/forward.h"
#include "pr/physics/terrain/iterrain.h"

#ifndef PR_ASSERT
#	define PR_ASSERT_STR_DEFINED
#	define PR_ASSERT(grp, exp, str)
#endif

namespace pr
{
	namespace ph
	{
		// Create a flat plane terrain
		struct TerrainPlane :ITerrain
		{
			void CollideSpheres(terrain::Sample* points, std::size_t num_points, TerrainContact terrain_contact_cb, void* context)
			{
				for( terrain::Sample *p = points, *p_end = points + num_points; p != p_end; ++p )
				{
					// If the sphere penetrates the ground
					if( p->m_point.y - p->m_radius < 0.0f )
					{
						terrain::Result result;
						result.m_sample				= p;
						result.m_sample_index		= p - points;
						result.m_terrain_point		= p->m_point;
						result.m_terrain_point.y	= 0.0f;
						result.m_normal				= v4::YAxis();
						result.m_material_id		= 0;
						if( !terrain_contact_cb(result, context) ) return;
					}
				}			
			}
			void CollideShape(Shape const&, m4x4 const&, ContactManifold&)
			{
				PR_ASSERT(1, false, "TerrainPlane doesn't support this");
			}
		};
	}
}

#ifdef PR_ASSERT_STR_DEFINED
#	undef PR_ASSERT_STR_DEFINED
#	undef PR_ASSERT
#endif

#endif
