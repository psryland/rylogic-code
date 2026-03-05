//*********************************************
// Physics engine
//  Copyright (c) Rylogic Ltd 2006
//*********************************************

// This file is for general purpose collision functions and types

#pragma once
#ifndef PR_PHYSICS_COLLISION_H
#define PR_PHYSICS_COLLISION_H

namespace pr
{
	namespace ph
	{
		namespace collision
		{
			const float FaceToFaceTolerance = 0.001f;

			enum EPointType
			{
				EPointType_Point	= 0,
				EPointType_Edge		= 1,
				EPointType_Face		= 2,
				EPointType_NumberOf
			};
			struct Point
			{
				Point() {}
				Point(v4 point)
				:m_point(point)
				,m_type(EPointType_Point)
				{}
				void set(v4 point, int type, int dof0, int dof1)
				{
					m_point = point;
					m_type = type;
					m_dof_info[0] = dof0;
					m_dof_info[1] = dof1;
				}
				v4		m_point;		
				int		m_type;			// One of EPointType
				int		m_dof_info[2];	// Information about the degrees of freedom
			};
		}

		// Projects a box onto 'axis'.
		// 'box' is three radius vectors describing the box
		// 'axis' is the axis to project the box onto.
		// 'point' is a point maximal in the direction of 'axis'
		// Returns the distance from the centre of the box to 'point' along 'axis'
		float ProjectBox(m3x4 const& box, v4 axis, collision::Point& point);

		// Project a triangle onto 'axis'
		// 'tri' is the three vertices of the triangle
		// 'axis' is the axis to project the triangle onto
		// 'point' is a point maximal in the direction of 'axis'
		// Returns the distance from the centre of the triangle to 'point' along 'axis'
		float ProjectTri(m3x4 const& tri, v4 axis, collision::Point& point);
	}
}

#endif
