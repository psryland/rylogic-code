//*********************************************
// Physics engine
//  Copyright (c) Rylogic Ltd 2006
//*********************************************

#include "physics/utility/stdafx.h"
#include "pr/physics/shape/shapebox.h"
#include "pr/physics/shape/shape.h"
#include "pr/physics/collision/contactmanifold.h"
#include "physics/utility/profile.h"

using namespace pr;
using namespace pr::ph;

// Construct a shape box
ShapeBox& ShapeBox::set(v4 dim, const m4x4& shape_to_model, MaterialId material_id, uint32_t flags)
{
	m_base.set(EShape_Box, sizeof(ShapeBox), shape_to_model, material_id, flags);
	m_radius = dim / 2.0f;
	m_radius.w = 0.0f;
	CalcBBox(*this, m_base.m_bbox);
	return *this;
}

// Return the bounding box for a box
BBox& pr::ph::CalcBBox(const ShapeBox& shape, BBox& bbox)
{
	bbox.m_centre = v4::Origin();
	bbox.m_radius = shape.m_radius;
	return bbox;
}

// Return the mass properties
MassProperties& pr::ph::CalcMassProperties(const ShapeBox& shape, float density, MassProperties& mp)
{
	float volume = 8.0f * shape.m_radius.x * shape.m_radius.y * shape.m_radius.z;

	mp.m_centre_of_mass = v4::Zero();
	mp.m_mass = volume * density;
	mp.m_os_inertia_tensor = m3x4::Identity();
	mp.m_os_inertia_tensor.x.x = (1.0f / 3.0f) * (shape.m_radius.y * shape.m_radius.y + shape.m_radius.z * shape.m_radius.z);	// (1/12)m(Y^2 + Z^2)
	mp.m_os_inertia_tensor.y.y = (1.0f / 3.0f) * (shape.m_radius.x * shape.m_radius.x + shape.m_radius.z * shape.m_radius.z);	// (1/12)m(X^2 + Z^2)
	mp.m_os_inertia_tensor.z.z = (1.0f / 3.0f) * (shape.m_radius.y * shape.m_radius.y + shape.m_radius.x * shape.m_radius.x);	// (1/12)m(Y^2 + Z^2)
	return mp;
}

// Shift the centre of a box
void pr::ph::ShiftCentre(ShapeBox&, v4& shift)
{
	PR_ASSERT(PR_DBG_PHYSICS, FEql(shift,pr::v4::Zero()), ""); (void)shift; //impossible to shift the centre of an implicit object
}

// Return a support vertex for a box
v4 pr::ph::SupportVertex(ShapeBox const& shape, v4 direction, std::size_t, std::size_t& sup_vert_id)
{
	PR_DECLARE_PROFILE(PR_PROFILE_SUPPORT_VERTS, phSupVertBox);
	PR_PROFILE_SCOPE(PR_PROFILE_SUPPORT_VERTS, phSupVertBox);

	int sign_x = (direction.x > 0.0f);
	int sign_y = (direction.y > 0.0f);
	int sign_z = (direction.z > 0.0f);

	sup_vert_id = (sign_z << 2) | (sign_y << 1) | (sign_x);
	return v4(
		(2.0f * sign_x - 1.0f) * shape.m_radius.x,
		(2.0f * sign_y - 1.0f) * shape.m_radius.y,
		(2.0f * sign_z - 1.0f) * shape.m_radius.z,
		1.0f);
}

//
//PHv4 phSupportVertex(const PHcylinder& prim, PHv4ref direction, PHuint, PHuint& support_vertex_id)
//{
//	PHv4 dir; dir.set(direction[0], 0.0f, direction[2], 0.0f);
//	dir.getNormal3(dir);
//
//	int sign_y = (direction[1] > 0.0f);
//	support_vertex_id =	(static_cast<PHuint>((dir[0] + 1.0f) * 0.5f * (1 << 4)) << 20) |
//																				sign_y |
//						(static_cast<PHuint>((dir[2] + 1.0f) * 0.5f * (1 << 4)) << 0 );
//	return PHv4::make(
//		dir[0] * prim.radius,
//		(2.0f * sign_y - 1.0f) * prim.height,
//		dir[2] * prim.radius,
//		1.0f);
//}
//PHv4 phSupportVertex(const PHsphere& prim, PHv4ref direction, PHuint, PHuint& support_vertex_id)
//{
//	PHv4 dir; dir.getNormal3(direction);
//
//	// Generate an id for the vertex in this direction
//	support_vertex_id =	(static_cast<PHuint>((dir[0] + 1.0f) * 0.5f * (1 << 4)) << 20) |
//						(static_cast<PHuint>((dir[1] + 1.0f) * 0.5f * (1 << 4)) << 10) |
//						(static_cast<PHuint>((dir[2] + 1.0f) * 0.5f * (1 << 4)) << 0 );
//	dir *= prim.radius;
//	dir.setW1();
//	return dir;
//}

// Find the nearest point and distance from a point to a shape
// 'shape' and 'point' are in the same space
void pr::ph::ClosestPoint(ShapeBox const& shape, v4 point, float& distance, v4& closest)
{
	closest  = point;
	distance = 0.0f; // Accumulate distance squared
	for( int i = 0; i != 3; ++i )
	{
		if( point[i] > shape.m_radius[i] )
		{
			distance  += Sqr(point[i] - shape.m_radius[i]);
			closest[i] = shape.m_radius[i];
		}
		else if( point[i] < -shape.m_radius[i] )
		{
			distance  += Sqr(point[i] + shape.m_radius[i]);
			closest[i] = -shape.m_radius[i];
		}
	}
	distance = Sqrt(distance);
}
