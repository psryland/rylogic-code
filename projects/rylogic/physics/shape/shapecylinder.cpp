//*********************************************
// Physics engine
//  Copyright (c) Rylogic Ltd 2006
//*********************************************

#include "physics/utility/stdafx.h"
#include "pr/physics/shape/shapecylinder.h"
#include "pr/physics/shape/shape.h"
#include "pr/physics/collision/contactmanifold.h"
#include "physics/utility/profile.h"

using namespace pr;
using namespace pr::ph;

// Construct the shape
ShapeCylinder& ShapeCylinder::set(float radius, float height, const m4x4& shape_to_model, MaterialId material_id, uint32_t flags)
{
	m_base.set(EShape_Cylinder, sizeof(ShapeCylinder), shape_to_model, material_id, flags);
	m_radius = radius;
	m_height = height / 2.0f;
	CalcBBox(*this, m_base.m_bbox);
	return *this;
}

// Return the bounding box for the shape
BBox& pr::ph::CalcBBox(ShapeCylinder const& shape, BBox& bbox)
{
	bbox.m_centre = v4::Origin();
	bbox.m_radius.x = shape.m_radius;
	bbox.m_radius.y = shape.m_height;
	bbox.m_radius.z = shape.m_radius;	// TODO, make the main axis of the cylinder the z axis
	bbox.m_radius.w = 0.0f;
	return bbox;
}

// Return the mass properties
MassProperties& pr::ph::CalcMassProperties(ShapeCylinder const& shape, float density, MassProperties& mp)
{
	auto volume = float(constants<double>::tau * shape.m_radius * shape.m_radius * shape.m_height);

	mp.m_centre_of_mass = v4::Zero();
	mp.m_mass = volume * density;
	mp.m_os_inertia_tensor		= m3x4::Identity();	// Note for shell, Ixx = Iyy = (1/2)mr^2 + (1/12)mL^2, Izz = mr^2
	mp.m_os_inertia_tensor.x.x	= (1.0f / 4.0f) * (shape.m_radius * shape.m_radius) + (1.0f / 3.0f) * (shape.m_height * shape.m_height);	// (1/4)mr^2 + (1/12)mL^2
	mp.m_os_inertia_tensor.y.y	= (1.0f / 2.0f) * (shape.m_radius * shape.m_radius);	// (1/2)mr^2
	mp.m_os_inertia_tensor.z.z	= mp.m_os_inertia_tensor.x.x;
	return mp;
}

// Shift the centre of a cylinder
void pr::ph::ShiftCentre(ShapeCylinder&, v4& shift)
{
	PR_ASSERT(PR_DBG_PHYSICS, FEql(shift,pr::v4::Zero()), ""); (void)shift; //impossible to shift the centre of an implicit object
}

// Return a support vertex for the shape
v4 pr::ph::SupportVertex(ShapeCylinder const& shape, v4 direction, std::size_t, std::size_t& sup_vert_id)
{
	PR_DECLARE_PROFILE(PR_PROFILE_SUPPORT_VERTS, phSupVertCyl);
	PR_PROFILE_SCOPE(PR_PROFILE_SUPPORT_VERTS, phSupVertCyl);

	bool xmajor = Abs(direction.x) >= Abs(direction.z);
	float c = xmajor ? Abs(direction.z/direction.x) : Abs(direction.x/direction.z);
	float a, b;	int id;
	if     ( c < 0.196350f )	{ a = 1.0f;      b = 0.0f;      id = 0; } // if     ( c < 1*pi/16 ) { a = 1, b = 0 }
	else if( c < 0.589049f )	{ a = 0.923880f; b = 0.382683f; id = 1; } // else if( c < 3*pi/16 ) { a = cos(pi/8), b = sin(pi/8) }
	else						{ a = 0.707107f; b = 0.707107f; id = 2; } // else                   { a = cos(pi/4), b = sin(pi/4) }

	v4 sup_vert;
	if( xmajor )
	{
		a *= (direction.x >= 0.0f) * 2.0f - 1.0f;
		b *= (direction.z >= 0.0f) * 2.0f - 1.0f;
		c  = (direction.y >= 0.0f) * 2.0f - 1.0f;
		sup_vert = v4(a * shape.m_radius, c * shape.m_height, b * shape.m_radius, 1.0f);
	}
	else
	{
		b *= (direction.x >= 0.0f) * 2.0f - 1.0f;
		a *= (direction.z >= 0.0f) * 2.0f - 1.0f;
		c  = (direction.y >= 0.0f) * 2.0f - 1.0f;
		sup_vert = v4(b * shape.m_radius, c * shape.m_height, a * shape.m_radius, 1.0f);
	}
	sup_vert_id = (sup_vert.z < 0.0f)*32 | (sup_vert.y < 0.0f)*16 | (sup_vert.x < 0.0f)*8 | xmajor*4 | id;
	return sup_vert;
}

// Find the nearest point and distance from a point to a shape
// 'shape' and 'point' are in the same space
void pr::ph::ClosestPoint(ShapeCylinder const& shape, v4 point, float& distance, v4& closest)
{
	closest  = point;
	distance = 0.0f; // Accumulate distance squared
	float dist_xz_sq = Sqr(point.x) + Sqr(point.z);
	if( point.y > shape.m_height )
	{
		distance += Sqr(point.y - shape.m_height);
		closest.y = shape.m_height;
	}
	else if( point.y < -shape.m_height )
	{
		distance  += Sqr(point.y + shape.m_height);
		closest.y = -shape.m_height;
	}
	if( dist_xz_sq > Sqr(shape.m_radius) )
	{
		float dist_xz		= Sqrt(dist_xz_sq);
		float r_div_dist_xz = shape.m_radius / dist_xz;
		distance += Sqr(dist_xz - shape.m_radius);
		closest.x = r_div_dist_xz * point.x;
		closest.z = r_div_dist_xz * point.z;
	}
	distance = Sqrt(distance);
}