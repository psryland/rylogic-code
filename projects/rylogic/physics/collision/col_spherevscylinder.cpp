//*********************************************
// Physics engine
//  Copyright (c) Rylogic Ltd 2006
//*********************************************

#include "physics/utility/stdafx.h"
#include "pr/physics/collision/collider.h"
#include "pr/physics/collision/contactmanifold.h"
#include "pr/physics/shape/shapesphere.h"
#include "pr/physics/shape/shapecylinder.h"
#include "physics/utility/debug.h"

using namespace pr;
using namespace pr::ph;

// Detect collisions between a sphere and a cylinder object
void pr::ph::SphereVsCylinder(Shape const& sphere, m4x4 const& a2w, Shape const& cylinder, m4x4 const& b2w, ContactManifold& manifold, CollisionCache*)
{
	ShapeSphere   const& sph = shape_cast<ShapeSphere>  (sphere);
	ShapeCylinder const& cyl = shape_cast<ShapeCylinder>(cylinder);

	PR_EXPAND(PR_DBG_SPH_CYL_COLLISION, std::string str);
	PR_EXPAND(PR_DBG_SPH_CYL_COLLISION, ldr::Cylinder("Cylinder", "FF0000FF", b2w, cyl.m_radius, cyl.m_height * 2, str));
	PR_EXPAND(PR_DBG_SPH_CYL_COLLISION, ldr::Sphere("Sphere", "FFFF0000", a2w.pos, sph.m_radius, str));
	PR_EXPAND(PR_DBG_SPH_CYL_COLLISION, StringToFile(str, "C:/Deleteme/collision_spherecylinder.pr_script"); str.clear());

	// Get a transform for the sphere in cylinder space
	m4x4 a2b = InvertAffine(b2w) * a2w;
	v4 sphere_pos = a2b.pos; // sphere position in cylinder space
	float height = Abs(sphere_pos.y);

	// Test for penetration into the ends of the cylinder
	if (height > cyl.m_height + sph.m_radius)
		return; // Separate on the main axis of the cylinder

	// Test the lateral separation of the sphere and cylinder
	float dist_xz_sq = Sqr(sphere_pos.x) + Sqr(sphere_pos.z);
	if (dist_xz_sq > Sqr(cyl.m_radius + sph.m_radius))
		return; // Separate in the XZ plane of the cylinder

	float dist_xz = Sqrt(dist_xz_sq);
	float dxz_sq = 0.0f, dy_sq = 0.0f; // squared distance to the rim of the cylinder
	if (dist_xz > cyl.m_radius) dxz_sq = Sqr(dist_xz - cyl.m_radius);
	if (height > cyl.m_height) dy_sq = Sqr(height - cyl.m_height);
	if (dxz_sq + dy_sq > Sqr(sph.m_radius))
		return; // Separate on the axis from the centre of the sphere to the rim of the cylinder

	// If the centre of the sphere is inside the cylinder, adjust dxz_sq or dy_sq
	// so that we choose the nearest exit point on the cylinder
	if (dxz_sq == 0.0f && dy_sq == 0.0f)
	{
		// If the centre of the sphere is closer to a wall of the cylinder...
		if ((dist_xz - cyl.m_radius) > (height - cyl.m_height)) dxz_sq = maths::tiny<float>;
		// Otherwise closest to an end of the cylinder
		else dy_sq = maths::tiny<float>;
	}

	Contact contact;
	contact.m_material_idA = sph.m_base.m_material_id;
	contact.m_material_idB = cyl.m_base.m_material_id;

	// Case 1: the centre of the sphere is closest to one of the ends of the cylinder
	if (dxz_sq == 0.0f)
	{
		contact.m_depth = cyl.m_height + sph.m_radius - height;
		contact.m_normal = Sign(sphere_pos.y) * b2w.y;
		contact.m_pointA = a2w.pos - sph.m_radius * contact.m_normal;
		contact.m_pointB = a2w.pos - (height - cyl.m_height) * contact.m_normal;
	}

	// Case 2: the centre of the sphere is closest to the wall of the cylinder
	else if (dy_sq == 0.0f)
	{
		contact.m_depth = cyl.m_radius + sph.m_radius - dist_xz;
		if (dist_xz < maths::tiny<float>) contact.m_normal = a2w.x;	// choose arbitrarily
		else contact.m_normal = b2w * v4(sphere_pos.x / dist_xz, 0.0f, sphere_pos.z / dist_xz, 0.0f);
		contact.m_pointA = a2w.pos - sph.m_radius * contact.m_normal;
		contact.m_pointB = a2w.pos - (dist_xz - cyl.m_radius) * contact.m_normal;
	}

	// Case 3: the centre of the sphere is closest to the rim of the cylinder
	else
	{
		float scale = cyl.m_radius / dist_xz;
		auto rim = v4(sphere_pos.x * scale, Sign(sphere_pos.y) * cyl.m_height, sphere_pos.z * scale, 1.0f);
		float dist = Sqrt(dxz_sq + dy_sq); // distance to the rim of the cylinder

		contact.m_depth = sph.m_radius - dist;
		contact.m_normal = b2w * ((sphere_pos - rim) / dist);
		contact.m_pointA = a2w.pos - sph.m_radius * contact.m_normal;
		contact.m_pointB = b2w * rim;
	}

	PR_EXPAND(PR_DBG_SPH_CYL_COLLISION, ldr::Box("pointA", "FFFF0000", contact.m_pointA, 0.05f, str));
	PR_EXPAND(PR_DBG_SPH_CYL_COLLISION, ldr::Box("pointB", "FF0000FF", contact.m_pointB, 0.05f, str));
	PR_EXPAND(PR_DBG_SPH_CYL_COLLISION, ldr::LineD("norm", "FFFFFF00", contact.m_pointB, contact.m_normal, str));
	PR_EXPAND(PR_DBG_SPH_CYL_COLLISION, StringToFile(str, "C:/Deleteme/collision_spherecylinder.pr_script", true); str.clear());
	manifold.Add(contact);
}


