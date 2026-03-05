//*********************************************
// Physics engine
//  Copyright (c) Rylogic Ltd 2006
//*********************************************

#include "physics/utility/stdafx.h"
#include "physics/collision/collision.h"

using namespace pr;
using namespace pr::ph;
using namespace pr::ph::collision;

// Projects a box onto 'axis'.
// 'box' is three radius vectors describing the box
// 'axis' is the axis to project the box onto.
// 'point' is a point maximal in the direction of 'axis'
// Returns the distance from the centre of the box to 'point' along 'axis'
float pr::ph::ProjectBox(m3x4 const& box, v4 axis, collision::Point& point)
{
	PR_ASSERT(PR_DBG_PHYSICS, FEql(Length(axis), 1.0f), "");

	point.m_type = EPointType_Point;
	float dist = 0.0f;
	for( int i = 0; i != 3; ++i )
	{
		float d = Dot3(axis, box[i]);
		if( FEqlRelative(d, 0.f, FaceToFaceTolerance) )
		{
			// Store the index of the axis that is a free direction
			PR_ASSERT(PR_DBG_PHYSICS, point.m_type < 2, "");
			point.m_dof_info[point.m_type++] = i;
		}
		else if( d > 0.0f )
		{
			point.m_point	+= box[i];
			dist			+= d;
		}
		else // d < 0.0f
		{
			point.m_point	-= box[i];
			dist			-= d;
		}
	}
	return dist;
}

// Project a triangle onto 'axis'
// 'tri' is the three vertices of the triangle
// 'axis' is the axis to project the triangle onto
// 'point' is a point maximal in the direction of 'axis'
// Returns the distance from the centre of the triangle to 'point' along 'axis'
float pr::ph::ProjectTri(m3x4 const& tri, v4 axis, collision::Point& point)
{
	// Store the indices of the two verts if the point type is 'edge'
	point.m_type		= EPointType_Point;
	point.m_dof_info[0] = 0;
	float dist			= Dot3(axis, tri[0]);
	v4    pt			= tri[0];
	float d;

	d = Dot3(axis, tri[1]);
	if( FEql(d, dist) )
	{
		pt += tri[1];
		point.m_type++;
		point.m_dof_info[1] = 1;
	}
	else if( d > dist )
	{
		pt = tri[1];
		point.m_type = EPointType_Point;
		point.m_dof_info[0] = 1;
		dist = d;
	}

	d = Dot3(axis, tri[2]);
	if( FEql(d, dist) )
	{
		pt += tri[2];
		point.m_type++;
		point.m_dof_info[1] = 2;
	}
	else if( d > dist )
	{
		pt = tri[2];
		point.m_type = EPointType_Point;
		point.m_dof_info[0] = 2;
		dist = d;
	}

	pt.w = 0.0f;
	pt /= (1.0f + point.m_type);
	point.m_point += pt;
	return dist;
}

