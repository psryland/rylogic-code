//*********************************************
// Physics engine
//  Copyright (c) Rylogic Ltd 2006
//*********************************************

#pragma once
#ifndef PR_PHYSICS_COLLISION_SIMPLEX_H
#define PR_PHYSICS_COLLISION_SIMPLEX_H

#include "pr/physics/types/forward.h"

namespace pr
{
	namespace ph
	{
		namespace mesh_vs_mesh
		{
			struct Vert
			{
				void set(v4 direction, std::size_t id_p, std::size_t id_q)	{ m_direction = direction; m_id_p = id_p; m_id_q = id_q; }
				v4			m_p, m_q, m_r;	// Points on objectA, objectB, and the minkowski difference of A and B
				v4			m_direction;	// The support direction used to calculate this vertex
				std::size_t	m_id_p, m_id_q;	// The id's of the vertices 'm_p' and 'm_q'
			};
			inline bool operator == (Vert const& lhs, Vert const& rhs)	{ return lhs.m_id_p == rhs.m_id_p && lhs.m_id_q == rhs.m_id_q; }
			inline bool operator != (Vert const& lhs, Vert const& rhs)	{ return !(lhs == rhs); }

			struct TrackVert
			{
				Vert*	m_vert;
				float	m_distance;		// Distance from the origin to the point on the sphere
				v4		m_offset;		// Offset from the point on the sphere to the vertex

				void set(Vert& vert)
				{
					PR_ASSERT(PR_DBG_PHYSICS, IsNormal(vert.m_direction), "");
					*m_vert				= vert;
					m_distance			= Dot3(m_vert->m_r, m_vert->m_direction);
					m_offset			= m_vert->m_r - m_distance * m_vert->m_direction;
				}
				TrackVert& operator = (TrackVert const& rhs)
				{
					*m_vert		= *rhs.m_vert;
					m_distance	= rhs.m_distance;
					m_offset	= rhs.m_offset;
					return *this;
				}
				Vert const& vert() const	{ return *m_vert; }
				v4 direction() const	{ return m_vert->m_direction; }
				v4 offset() const	{ return m_offset; }
				float distance() const		{ return m_distance; }
			};

			// A triangle used to represent the collision manifold on the minkowski difference
			struct Triangle
			{
				Vert		m_vert[3];		// A triangle describing a plane
				v4			m_direction;	// The direction of the normal of the triangle (not normalised)
				float		m_distance;		// The shortest distance from the plane to the origin
			};
		
			// This struct represents a simplex in 3D.
			// It is used by the GJK algorithm to determine collision
			struct Simplex
			{
				std::size_t	m_num_vertices;		// The number of vertices in the simplex (0 -> 3)
				Vert		m_vertex[4];		// The vertices of the simplex
				v4			m_bary_coords;		// The barycentric co-ordinates of 'm_nearest_point'
				v4			m_nearest_point;	// The nearest point on the convex hull of the simplex to the origin

				Simplex() :m_num_vertices(0) {}
				bool	AddVertex(Vert const& v);
				v4		FindNearestPoint(v4 point);
				v4		GetNearestPointOnA() const;	// Returns the nearest point on A in world space. Must call 'FindNearestPoint' first
				v4		GetNearestPointOnB() const;	// Returns the nearest point on B in world space. Must call 'FindNearestPoint' first
			};
		}
	}
}

#endif
