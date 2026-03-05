//*********************************************
// Physics engine
//  Copyright (c) Rylogic Ltd 2006
//*********************************************

#pragma once
#ifndef PR_PHYSICS_TERRAIN_IMPLICIT_SURFACE_H
#define PR_PHYSICS_TERRAIN_IMPLICIT_SURFACE_H

#include "pr/physics/types/forward.h"
#include "pr/physics/terrain/iterrain.h"

namespace pr
{
	namespace ph
	{
		// Create an implicit terrain surface
		// The implicit surface is a 2d continuous function in (x,z) with 'y' as the up direction
		struct TerrainImplicitSurf :ITerrain
		{
			float m_scale_x;
			float m_scale_z;
			
			TerrainImplicitSurf()
			:m_scale_x(0.01f)
			,m_scale_z(0.01f)
			{}
			
			// Returns the height of the terrain at the given x,z coordinate
			float Eval(float x, float z) const
			{
				return m_scale_x * Sqr(x) + m_scale_z * Sqr(z);
			}
			
			v4 EvalN(float x, float y, float z) const
			{
				v4 p0(x, y, z, 0.0f);
				v4 p1(x, 0.0f, z + 0.01f, 0.0f);
				v4 p2(x + 0.01f, 0.0f, z, 0.0f);
				p1.y = Eval(p1.x, p1.z);
				p2.y = Eval(p2.x, p2.z);
				return Normalise(Cross(p1 - p0, p2 - p0));
			}
			
			void CollideSpheres(terrain::Sample* points, std::size_t num_points, TerrainContact terrain_contact_cb, void* context)
			{
				for( terrain::Sample *p = points, *p_end = points + num_points; p != p_end; ++p )
				{
					// Evaluate the function at x, z
					float y = Eval(p->m_point.x, p->m_point.z);

					// If the sphere penetrates the ground
					if( p->m_point.y - p->m_radius < y )
					{
						terrain::Result result;
						result.m_sample				= p;
						result.m_sample_index		= p - points;
						result.m_terrain_point		= p->m_point;
						result.m_terrain_point.y	= y;
						result.m_normal				= EvalN(p->m_point.x, y, p->m_point.z);
						result.m_material_id		= 0;
						if( !terrain_contact_cb(result, context) ) return;
					}
				}			
			}
		};
	}
}

#endif
