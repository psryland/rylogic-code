//*********************************************
// Physics engine
//  Copyright (c) Rylogic Ltd 2006
//*********************************************

#include "physics/utility/stdafx.h"
#include "pr/physics/shape/shapesphere.h"
#include "pr/physics/shape/shape.h"
#include "pr/physics/collision/contactmanifold.h"
#include "physics/utility/profile.h"

namespace pr::ph
{
	// Construct a shape sphere
	ShapeSphere& ShapeSphere::set(float radius, const m4x4& shape_to_model, MaterialId material_id, uint32_t flags)
	{
		m_base.set(EShape_Sphere, sizeof(ShapeSphere), shape_to_model, material_id, flags);
		m_radius = radius;
		CalcBBox(*this, m_base.m_bbox);
		return *this;
	}

	// Return the bounding box for a sphere
	BBox& CalcBBox(ShapeSphere const& shape, BBox& bbox)
	{
		bbox.m_centre = v4::Origin();
		bbox.m_radius.x = shape.m_radius;
		bbox.m_radius.y = shape.m_radius;
		bbox.m_radius.z = shape.m_radius;
		bbox.m_radius.w = 0.0f;
		return bbox;
	}

	// Return the mass properties
	MassProperties& CalcMassProperties(ShapeSphere const& shape, float density, MassProperties& mp)
	{
		auto volume = float((2.0 / 3.0) * constants<double>::tau * shape.m_radius * shape.m_radius * shape.m_radius);

		mp.m_centre_of_mass = v4::Zero();
		mp.m_mass = volume * density;
		mp.m_os_inertia_tensor = m3x4::Identity();	// Note for a shell, Ixx = Iyy = Izz = 2/3mr^2
		mp.m_os_inertia_tensor.x.x = (2.0f / 5.0f) * (shape.m_radius * shape.m_radius);	// (2/5)mr^2
		mp.m_os_inertia_tensor.y.y = mp.m_os_inertia_tensor.x.x;
		mp.m_os_inertia_tensor.z.z = mp.m_os_inertia_tensor.x.x;
		return mp;
	}

	// Shift the centre of a sphere
	void ShiftCentre(ShapeSphere&, v4& shift)
	{
		PR_ASSERT(PR_DBG_PHYSICS, FEql(shift, pr::v4::Zero()), ""); shift; // Impossible to shift the centre of an implicit object
	}

	// Return a support vertex for a sphere
	v4 SupportVertex(ShapeSphere const& shape, v4 direction, std::size_t, std::size_t& sup_vert_id)
	{
		PR_DECLARE_PROFILE(PR_PROFILE_SUPPORT_VERTS, phSupVertSph);
		PR_PROFILE_SCOPE(PR_PROFILE_SUPPORT_VERTS, phSupVertSph);

		// We need to quantise the normal otherwise the iterative algorithms perform badly
		v4 dir = Normalise(direction);

		// Generate an id for the vertex in this direction
		sup_vert_id =
			static_cast<std::size_t>((dir.x + 1.0f) * 0.5f * (1 << 4)) << 20 |
			static_cast<std::size_t>((dir.y + 1.0f) * 0.5f * (1 << 4)) << 10 |
			static_cast<std::size_t>((dir.z + 1.0f) * 0.5f * (1 << 4)) << 0;
		return dir * shape.m_radius + v4::Origin();
	}

	// Find the nearest point and distance from a point to a shape
	// 'shape' and 'point' are in the same space
	void ClosestPoint(ShapeSphere const& shape, v4 point, float& distance, v4& closest)
	{
		distance = Length(point);
		closest = (shape.m_radius / distance) * point;
		closest.w = 1.0f;
		distance -= shape.m_radius;
	}
}