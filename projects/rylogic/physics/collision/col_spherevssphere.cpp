//*********************************************
// Physics engine
//  Copyright (c) Rylogic Ltd 2006
//*********************************************

#include "physics/utility/stdafx.h"
#include "pr/physics/collision/collider.h"
#include "pr/physics/collision/contactmanifold.h"
#include "pr/physics/rigidbody/rigidbody.h"
#include "pr/physics/shape/shapesphere.h"
#include "physics/utility/debug.h"

using namespace pr;
using namespace pr::ph;

// Detect collisions between two spheres
void pr::ph::SphereVsSphere(Shape const& objA, m4x4 const& a2w, Shape const& objB, m4x4 const& b2w, ContactManifold& manifold, CollisionCache*)
{
	ShapeSphere const& shapeA = shape_cast<ShapeSphere>(objA);
	ShapeSphere const& shapeB = shape_cast<ShapeSphere>(objB);

	v4 b2a = a2w.pos - b2w.pos;
	float b2a_len = Length(b2a);
	if (b2a_len < maths::tiny<float>) { b2a_len = b2a.y = 0.001f; }
	float sep = b2a_len - shapeA.m_radius - shapeB.m_radius;
	if (sep > 0.0f) return;

	Contact contact;
	contact.m_normal = b2a / b2a_len;
	contact.m_pointA = b2a * (shapeA.m_radius / -b2a_len) + a2w.pos;
	contact.m_pointB = b2a * (shapeB.m_radius / b2a_len) + b2w.pos;
	contact.m_material_idA = shapeA.m_base.m_material_id;
	contact.m_material_idB = shapeB.m_base.m_material_id;
	contact.m_depth = -sep;
	manifold.Add(contact);
}
