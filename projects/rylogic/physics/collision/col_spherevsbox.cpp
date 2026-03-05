//*********************************************
// Physics engine
//  Copyright (c) Rylogic Ltd 2006
//*********************************************

#include "physics/utility/stdafx.h"
#include "pr/physics/collision/collider.h"
#include "pr/physics/collision/contactmanifold.h"
#include "pr/physics/shape/shapesphere.h"
#include "pr/physics/shape/shapebox.h"
#include "physics/utility/debug.h"

using namespace pr;
using namespace pr::ph;

// Detect collisions between a sphere and a box object
void pr::ph::SphereVsBox(Shape const& sphere, m4x4 const& a2w, Shape const& box, m4x4 const& b2w, ContactManifold& manifold, CollisionCache*)
{
	ShapeSphere const& sphere_shape = shape_cast<ShapeSphere>(sphere);
	ShapeBox    const& box_shape    = shape_cast<ShapeBox   >(box);

	PR_EXPAND(PR_DBG_SPH_BOX_COLLISION, StartFile("C:/Deleteme/collision_boxsphere.pr_script");)
	PR_EXPAND(PR_DBG_SPH_BOX_COLLISION, ldr::GroupStart("BoxSpace");)
	PR_EXPAND(PR_DBG_SPH_BOX_COLLISION, ldr::g_output->Print(ldr::Txfm(b2w));)

	// Convert into box space
	v4 b2s = InvertAffine(b2w) * a2w.pos - v4::Origin(); // Box to sphere vector in box space
	PR_EXPAND(PR_DBG_SPH_BOX_COLLISION, ldr::Line("b2s", "FF00FF00", v4::Origin(), b2s);)
	
	// Get a vector from the sphere to the nearest point on the box
	v4 closest = v4::Zero();
	float dist_sq = 0.0f;
	for( int i = 0; i != 3; ++i )
	{
		if( b2s[i] > box_shape.m_radius[i] )
		{
			dist_sq    += Sqr(b2s[i] - box_shape.m_radius[i]);
			closest[i]  = box_shape.m_radius[i];
		}
		else if( b2s[i] < -box_shape.m_radius[i] )
		{
			dist_sq    += Sqr(b2s[i] + box_shape.m_radius[i]);
			closest[i]  = -box_shape.m_radius[i];
		}
		else
		{
			closest[i]  = b2s[i];
		}
	}
	PR_EXPAND(PR_DBG_SPH_BOX_COLLISION, ldr::Box("closest", "FFFFFF00", closest, 0.05f);)
	PR_EXPAND(PR_DBG_SPH_BOX_COLLISION, ldr::GroupEnd();)
	PR_EXPAND(PR_DBG_SPH_BOX_COLLISION, EndFile();)
	
	// If the separation is greater than the radius of the sphere then no collision
	if( dist_sq > Sqr(sphere_shape.m_radius) ) return;

	PR_EXPAND(PR_DBG_SPH_BOX_COLLISION, AppendFile("C:/Deleteme/collision_boxsphere.pr_script");)
	//PR_EXPAND(PR_DBG_SPH_BOX_COLLISION, ldr::GroupStart("BoxSpace");)
	//PR_EXPAND(PR_DBG_SPH_BOX_COLLISION, ldr::g_output->Print(ldr::Txfm(b2w));)
	//PR_EXPAND(PR_DBG_SPH_BOX_COLLISION, ldr::GroupEnd();)

	// Find the closest points
	Contact contact;
	contact.m_material_idA	= sphere_shape.m_base.m_material_id;
	contact.m_material_idB	= box_shape   .m_base.m_material_id;

	// If 'dist_sq' is zero then the centre of the sphere is inside the box
	if (dist_sq < maths::tiny<float>)
	{
		// Find the closest point on the box to the centre of the sphere
		int largest = MaxElementIndex(Abs(b2s));
		float sign = (b2s[largest] > 0.0f) * 2.0f - 1.0f;

		// Sphere contact point
		contact.m_pointA = b2s;
		contact.m_pointA[largest] -= sign * sphere_shape.m_radius;
		contact.m_pointA = b2w * contact.m_pointA + b2w.pos;

		// Box contact point
		contact.m_pointB = b2s;
		contact.m_pointB[largest] = sign * box_shape.m_radius[largest];
		contact.m_pointB = b2w * contact.m_pointB + b2w.pos;

		// Get the normal in world space
		contact.m_normal = v4::Zero();
		contact.m_normal[largest] = sign * 1.0f;
		contact.m_normal = b2w * contact.m_normal;
		contact.m_depth = sphere_shape.m_radius + box_shape.m_radius[largest] - Abs(b2s[largest]);
	}
	// Otherwise the centre of the sphere is outside of the box
	else
	{
		float dist = Sqrt(dist_sq);

		// Sphere contact point
		contact.m_pointA = b2w * ((closest - b2s) * sphere_shape.m_radius / dist) + a2w.pos;

		// Box contact point
		contact.m_pointB = b2w * closest + b2w.pos;

		// Get the normal in world space
		contact.m_normal = b2w * ((b2s - closest) / dist);
		contact.m_depth  = sphere_shape.m_radius - dist;
	}
	manifold.Add(contact);

	PR_EXPAND(PR_DBG_SPH_BOX_COLLISION, ldr::Box  ("ptA" , "FFFF0000", contact.m_pointA, 0.05f);)
	PR_EXPAND(PR_DBG_SPH_BOX_COLLISION, ldr::Box  ("ptB" , "FF0000FF", contact.m_pointB, 0.05f);)
	PR_EXPAND(PR_DBG_SPH_BOX_COLLISION, ldr::LineD("norm", "FFFFFF00", contact.m_pointB, contact.m_normal * contact.m_depth);)
	PR_EXPAND(PR_DBG_SPH_BOX_COLLISION, EndFile();)
}