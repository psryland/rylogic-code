//*********************************************
// Physics engine
//  Copyright (c) Rylogic Ltd 2006
//*********************************************

#include "physics/utility/stdafx.h"
#include "pr/physics/shape/shapetriangle.h"
#include "pr/physics/shape/shape.h"
#include "pr/physics/collision/contactmanifold.h"
#include "physics/utility/profile.h"

using namespace pr;
using namespace pr::ph;
using namespace pr::geometry;

// Construct a shape triangle
ShapeTriangle& ShapeTriangle::set(v4 a, v4 b, v4 c, const m4x4& shape_to_model, MaterialId material_id, uint32_t flags)
{
	PR_ASSERT(PR_DBG_PHYSICS, a.w == 0.0f && b.w == 0.0f && c.w == 0.0f, "");
	m_base.set(EShape_Triangle, sizeof(ShapeTriangle), shape_to_model, material_id, flags);
	m_v.x = a;
	m_v.y = b;
	m_v.z = c;
	m_v.w = Normalise(Cross(b-a,c-b));
	CalcBBox(*this, m_base.m_bbox);
	return *this;
}

// Return the bounding box for a triangle
BBox& pr::ph::CalcBBox(ShapeTriangle const& shape, BBox& bbox)
{
	bbox.reset();
	Grow(bbox, shape.m_v.x);
	Grow(bbox, shape.m_v.y);
	Grow(bbox, shape.m_v.z);
	return bbox;
}

// Return the inertia tensor for the triangle
m3x4 pr::ph::CalcInertiaTensor(ShapeTriangle const& shape)
{
	m3x4 inertia = m3x4::Zero();
	for( int i = 0; i != 3; ++i )
	{
		v4 vert = shape.m_v[i];
		inertia.x.x += Sqr(vert.y) + Sqr(vert.z);
		inertia.y.y += Sqr(vert.z) + Sqr(vert.x);
		inertia.z.z += Sqr(vert.x) + Sqr(vert.y);
		inertia.x.y += vert.x * vert.y;
		inertia.x.z += vert.x * vert.z;
		inertia.y.z += vert.y * vert.z;
	}
	inertia.x.y = -inertia.x.y;
	inertia.x.z = -inertia.x.y;
	inertia.y.z = -inertia.x.y;
	inertia.y.x = inertia.x.y;
	inertia.z.x = inertia.x.z;
	inertia.z.y = inertia.y.z;
	return inertia;
}

// Return the mass properties
MassProperties& pr::ph::CalcMassProperties(ShapeTriangle const& shape, float density, MassProperties& mp)
{
	mp.m_centre_of_mass = (shape.m_v.x + shape.m_v.y + shape.m_v.z) / 3.0f;
	mp.m_centre_of_mass.w = 0.0f;	// 'centre_of_mass' is an offset from the current model origin
	mp.m_mass = Length(Cross(shape.m_v.y-shape.m_v.x, shape.m_v.z-shape.m_v.y)) * 0.5f * density;
	mp.m_os_inertia_tensor = CalcInertiaTensor(shape);
	return mp;
}

// Shift the centre of a triangle
void pr::ph::ShiftCentre(ShapeTriangle& shape, v4& shift)
{
	PR_ASSERT(PR_DBG_PHYSICS, shift.w == 0.0f, "");
	if( FEql(shift,pr::v4::Zero()) ) return;
	shape.m_v.x -= shift;
	shape.m_v.y -= shift;
	shape.m_v.z -= shift;
	shape.m_base.m_shape_to_model.pos += shift;
	shift = pr::v4::Zero();
}

// Return a support vertex for a triangle
v4 pr::ph::SupportVertex(ShapeTriangle const& shape, v4 direction, std::size_t, std::size_t& sup_vert_id)
{
	PR_DECLARE_PROFILE(PR_PROFILE_SUPPORT_VERTS, phSupVertTri);
	PR_PROFILE_SCOPE(PR_PROFILE_SUPPORT_VERTS, phSupVertTri);

	v4 d;
	d.x = Dot3(direction, shape.m_v.x);
	d.y = Dot3(direction, shape.m_v.y);
	d.z = Dot3(direction, shape.m_v.z);
	d.w = 0.0f;
	sup_vert_id = MaxElementIndex(d.xyz);
	return shape.m_v[(int)sup_vert_id];
}

// Find the nearest point and distance from a point to a shape
// 'shape' and 'point' are in the same space
void pr::ph::ClosestPoint(ShapeTriangle const& shape, v4 point, float& distance, v4& closest)
{
	closest = closest_point::PointToTriangle(point, shape.m_v.x, shape.m_v.y, shape.m_v.z);
	distance = Length(point - closest);
}