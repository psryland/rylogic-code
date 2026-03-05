//*********************************************
// Physics engine
//  Copyright (c) Rylogic Ltd 2006
//*********************************************

#include "physics/utility/stdafx.h"
#include "physics/collision/simplex.h"

using namespace pr;
using namespace pr::ph;
using namespace pr::ph::mesh_vs_mesh;
using namespace pr::geometry;

inline void Swap(float& lhs, float& rhs)                           { float t = lhs; lhs = rhs; rhs = t; }
inline void Swap(mesh_vs_mesh::Vert& lhs, mesh_vs_mesh::Vert& rhs) { mesh_vs_mesh::Vert t = lhs; lhs = rhs; rhs = t; }

// Add a vertex to the simplex. This method maintains the correct winding order 
// for the triangles. All triangles should have the origin on the negative side.
// The interior of a tetrahedron is on the negative side of all of it's triangles
bool Simplex::AddVertex(mesh_vs_mesh::Vert const& v)
{
	PR_ASSERT(PR_DBG_PHYSICS, m_num_vertices < 4, "");

	// All vertices must be unique
	for( std::size_t i = 0; i != m_num_vertices; ++i )
		if( v.m_id_p == m_vertex[i].m_id_p && v.m_id_q == m_vertex[i].m_id_q )
			return false;
	
	m_vertex[m_num_vertices++] = v;
	return true;
}

// This function finds the nearest point on the simplex to 'point'. It also reduces
// the simplex to the minimum number of vertices needed to describe that point.
v4 Simplex::FindNearestPoint(v4 point)
{
	switch( m_num_vertices )
	{
	case 1:	// Single point
		// When the simplex contains only one point, then it is
		// the nearest point and no vertices can be removed
		m_nearest_point  = m_vertex[0].m_r;
		m_bary_coords[0] = 1.0f;
		break;
	
	case 2: // Line
		// When the simplex contains two points, the nearest point
		// must be somewhere along the line seqment. 
		m_nearest_point = closest_point::PointToLine(point, m_vertex[0].m_r, m_vertex[1].m_r, m_bary_coords[0]);
		if( m_bary_coords[0] == 0.0f )
		{
			m_num_vertices		= 1;
			m_bary_coords[0]	= 1.0f;
		}
		else if( m_bary_coords[0] == 1.0f )
		{
			m_num_vertices		= 1;
			m_vertex[0]			= m_vertex[1];
		}
		else
		{
			m_bary_coords[1]	= m_bary_coords[0];
			m_bary_coords[0]	= 1.0f - m_bary_coords[0];
		}
		break;
	
	case 3: // Triangle
		// When the simplex contains three points, the nearest point
		// must be somewhere on face of the triangle
		m_nearest_point = geometry::closest_point::PointToTriangle(point, m_vertex[0].m_r, m_vertex[1].m_r, m_vertex[2].m_r, m_bary_coords);
		for( int i = 2; i >= 0; --i )
		{
			// If this vert is not needed in the simplex,
			// copy from the end over it.
			if( m_bary_coords[i] == 0.0f )
			{
				int last = (int)--m_num_vertices;
				m_vertex[i]      = m_vertex[last];
				m_bary_coords[i] = m_bary_coords[last];
			}
		}break;
	
	case 4: // Tetrahedron
		// When the simplex contains four points, the nearest point
		// must be somewhere on the faces of the tetrahedron or within it.
		// Since the simplex points are essentially random we need to ensure
		// the tetrahedron has positive volume
		if( PointInFrontOfPlane(m_vertex[0].m_r, m_vertex[1].m_r, m_vertex[2].m_r, m_vertex[3].m_r) )
		{
			m_nearest_point = closest_point::PointToTetrahedron(point,	m_vertex[0].m_r,
																		m_vertex[1].m_r,
																		m_vertex[2].m_r,
																		m_vertex[3].m_r,
																		m_bary_coords);
		}
		else
		{
			m_nearest_point = closest_point::PointToTetrahedron(point,	m_vertex[0].m_r,
																		m_vertex[1].m_r,
																		m_vertex[3].m_r,
																		m_vertex[2].m_r,
																		m_bary_coords);
			Swap(m_bary_coords[2], m_bary_coords[3]);
		}
		for( int i = 3; i >= 0; --i )
		{
			// If this vert is not needed in the simplex,
			// copy from the end over it.
			if( m_bary_coords[i] == 0.0f )
			{
				int last = (int)--m_num_vertices;
				m_vertex[i]      = m_vertex[last];
				m_bary_coords[i] = m_bary_coords[last];
			}
		}
		break;
	}
	PR_ASSERT(PR_DBG_PHYSICS, IsFinite(m_nearest_point), "");
	return m_nearest_point;
}

// Return the nearest point on object A
v4 Simplex::GetNearestPointOnA() const
{
	v4 point = v4::Zero();
	for( int i = 0; i != (int)m_num_vertices; ++i )
		point += m_bary_coords[i] * m_vertex[i].m_p;
	return point;
}

// Return the nearest point on object B
v4 Simplex::GetNearestPointOnB() const
{
	v4 point = v4::Zero();
	for (int i = 0; i != (int)m_num_vertices; ++i)
		point += m_bary_coords[i] * m_vertex[i].m_q;
	return point;
}
