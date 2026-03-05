//*********************************************
// Physics engine
//  Copyright (c) Rylogic Ltd 2006
//*********************************************

#pragma once
#ifndef PR_PHYSICS_TERRAIN_QUAD_TREE_H
#define PR_PHYSICS_TERRAIN_QUAD_TREE_H

#include "pr/physics/types/forward.h
#include "pr/physics/terrain/iterrain.h"
#include "pr/common/quadtree.h"

namespace pr
{
	namespace ph
	{
		// Create terrain from a triangle soup
		struct TerrainQuadTree : ITerrain
		{
			// Implementation, Create a region 1000x1000 say and modulus the positions
			// of each face by 1000.
			struct Face { v4 m_vert[3]; };

			pr::quad_tree::Tree<Face> m_tree;
			void Reset();
			void AddFace(v4 a, v4 b, v4 c);
			void CollideSpheres(terrain::Sample* points, std::size_t num_points, TerrainContact terrain_contact_cb, void* context);
		};
	}
}

#endif
