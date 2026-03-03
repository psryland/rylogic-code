//*********************************************
// Physics engine
//  Copyright (c) Rylogic Ltd 2006
//*********************************************

#include "physics/utility/stdafx.h"
#include "pr/physics/collision/collider.h"
#include "pr/physics/collision/contactmanifold.h"
#include "pr/physics/rigidbody/rigidbody.h"
#include "pr/physics/shape/shape.h"
#include "pr/physics/shape/shapecylinder.h"
#include "physics/utility/debug.h"
#include "physics/utility/profile.h"

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
				Point(v4 const& point)
				:m_point(point)
				,m_type(EPointType_Point)
				{}
				void set(v4 const& point, int type, int dof0, int dof1)
				{
					m_point = point;
					m_type = type;
					m_dof_axis[0] = dof0;
					m_dof_axis[1] = dof1;
				}
				v4		m_point;		
				int		m_type;			// One of EPointType
				int		m_dof_axis[2];	// The indices of the free axes
			};
			struct Overlap
			{
				Overlap(Overlap const&);			 // No copying
				Overlap& operator =(Overlap const&); // No copying
				Overlap(ShapeCylinder const& shapeA, m4x4 const& a2w, ShapeCylinder const& shapeB, m4x4 const& b2w)
				:m_shapeA(shapeA)
				,m_a2w(a2w)
				,m_shapeB(shapeB)
				,m_b2w(b2w)
				,m_penetration(maths::float_max)
				{
					m_heightA = m_a2w.y * m_shapeA.m_height;
					m_heightB = m_b2w.y * m_shapeB.m_height;
				}

				ShapeCylinder const&	m_shapeA;
				m4x4 const&				m_a2w;
				ShapeCylinder const&	m_shapeB;
				m4x4 const&				m_b2w;
				v4						m_axis;				// Always from m_pointA to m_pointB
				float					m_penetration;		// The deep of penetration. No contact if <= 0.0f
				Point					m_pointA;			// The point of contact on object A
				Point					m_pointB;			// The point of contact on object B
				v4						m_heightA;
				v4						m_heightB;
			};
		}//namespace collision
	}//namespace ph
}//namespace pr

using namespace pr;
using namespace pr::ph;
using namespace pr::ph::collision;

// Projects a cylinder onto 'axis'
// Returns the half width of the cylinder when projected.
// Also returns the point in the minimum direction of axis
float Project(v4 const& axis, v4 const& cyl_axis, float height, float radius, Point& point)
{
	float dist = 0.0f;
	float d = Dot3(axis, cyl_axis);
	
	// Project the long axis
	if( FEqlRelative(d, 0.f, FaceToFaceTolerance) )
	{
		//point.m_dof_axis[point.m_type++] = 0;
		//PR_ASSERT(PR_DBG_PHYSICS, point.m_type <= EPointType_NumberOf);
	}
	else if( d > 0.0f )
	{
		point.m_point -= cyl_axis * height;
		dist          += d * height;
	}
	else // d < 0.0f
	{
		point.m_point += cyl_axis * height;
		dist          -= d * height;
	}

	// Project the radius
	v4 radius_v4 = Cross3(cyl_axis, Cross3(cyl_axis, axis));
	d = Length(radius_v4);
	if( FEqlRelative(d, 0.f, FaceToFaceTolerance) )
	{
		//point.m_dof_axis[point.m_type++] = 0;
		//PR_ASSERT(PR_DBG_PHYSICS, point.m_type <= EPointType_NumberOf);
	}
	else // d > 0.0f
	{
		point.m_point -= radius_v4 * (radius / d);
		dist		  += (radius / d);
	}
	return dist;
}

// Test two cylinders for collision
//bool Collide(Overlap& data)
//{
//	data;
//	//v4 a_to_b = data.m_b2w.pos - data.m_a2w.pos;
//
//	//// Test the main axis of cylinder A
//	//{
//	//	Point pointB(data.m_b2w.pos);
//	//	v4    axis = data.m_a2w.y;
//	//	float sep  = Dot3(axis, a_to_b);
//	//	if( sep < 0.0f )	{ axis  = -axis; sep = -sep; } // Ensure axis points from A to B
//	//	float proj = Project(axis, data.m_b2w.y, data.m_shapeB.m_height, data.m_shapeB.m_radius, pointB); // Project cylinder B onto this axis
//	//	float overlap = -sep + data.m_shapeA.m_height + proj;
//	//	if( overlap < 0.0f )
//	//		return false;
//	//	if( overlap < data.m_penetration )
//	//	{
//	//		data.m_penetration	= overlap;
//	//		data.m_axis			= axis;
//	//		data.m_pointA		.set(data.m_a2w.pos + data.m_shapeA.m_height*axis, EPointType_Face, (i+1)%3, (i+2)%3);
//	//		data.m_pointB		= pointB;
//	//	}
//	//}
//
//	//// Test the main axis of cylinder B
//	//{
//	//}
//
//	//m4x4 a2b = b2w.InvertAffine() * a2w;
//	//v4 e0 =  cylA.m_height * v4::ZAxis();
//	//v4 s0 = -e0;
//	//v4 e1 =  cylB.m_height * a2b.z;
//	//v4 s1 = -e1;
//	//float t0, t1;
//	//ClosestPoint_LineSegmentToLineSegment(s0, e0, s1, e1, t0, t1);
//	//
//	//v4 nearestA = (1.0f-t0)*s0 + t0*e0;
//	//v4 nearestB = (1.0f-t1)*s1 + t1*e1;
//	//v4 nearest  = nearestB - nearestA;
//
//	//// Test the axis perpendicular to the main axis of cylinder A that is in the direction of the nearest vector
//	//{
//	//	//// Project B onto the z axis of A
//	//	//Dot3(b2w.z, a2w.z) * shapeB.m_height;
//
//	//	//// Test the axis of cylinder A
//	//	//float distA = Project(a2w.z, shapeA.m_height * a2w.z, 
//	//}
//
//	//// Test the axis perpendicular to the main axis of cylinder B that is in the direction of the -nearest vector
//	//{
//	//	//// Project B onto the z axis of A
//	//	//Dot3(b2w.z, a2w.z) * shapeB.m_height;
//
//	//	//// Test the axis of cylinder A
//	//	//float distA = Project(a2w.z, shapeA.m_height * a2w.z, 
//	//}
//	return false;
//}

// Detect collisions between cylinder shapes
void pr::ph::CylinderVsCylinder(Shape const& shapeA, m4x4 const& a2w, Shape const& shapeB, m4x4 const& b2w, ContactManifold&, CollisionCache*)
{
	PR_DECLARE_PROFILE(PR_PROFILE_CYL_COLLISION, phCylVsCyl);
	PR_PROFILE_SCOPE(PR_PROFILE_CYL_COLLISION, phCylVsCyl);
	
	ShapeCylinder const& cylA = shape_cast<ShapeCylinder>(shapeA);
	ShapeCylinder const& cylB = shape_cast<ShapeCylinder>(shapeB);
	Overlap min_overlap(cylA, a2w, cylB, b2w);
	//::Collide(min_overlap);
}
