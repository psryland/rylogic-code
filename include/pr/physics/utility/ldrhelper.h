//*********************************************
// Physics engine
//  Copyright (c) Rylogic Ltd 2006
//*********************************************

#pragma once
#ifndef PR_PHYSICS_LINEDRAWERHELPER_H
#define PR_PHYSICS_LINEDRAWERHELPER_H

#if PH_LDR_OUTPUT

#include "pr/linedrawer/ldr_helper.h"
#include "pr/geometry/geosphere.h"

#include "pr/physics/types/forward.h"
#include "pr/physics/shape/shapes.h"
#include "pr/physics/rigidbody/rigidbody.h"
#include "pr/physics/rigidbody/support.h"
#include "pr/physics/collision/contactmanifold.h"

// hack till I update this file to use ldr_helper2.h
#pragma warning(push)
//#pragma warning(disable:4100)

namespace pr
{
	namespace ldr
	{
		template <typename TStr> void PhShape(char const* name, unsigned int colour, ph::ShapeSphere const& shape, m4x4 const& o2w, TStr& str)
		{
			//Sphere(name, colour, o2w.pos, shape.m_radius, str);
		}
		template <typename TStr> void PhShape(char const* name, unsigned int colour, ph::ShapeBox const& shape, m4x4 const& o2w, TStr& str)
		{
			//Box(name, colour, o2w, shape.m_radius * 2.0f, str);
		}
		template <typename TStr> void PhShape(char const* name, unsigned int colour, ph::ShapeCylinder const& shape, m4x4 const& o2w, TStr& str)
		{
			//Cylinder(name, colour, o2w, shape.m_radius, shape.m_height * 2.0f, str);
		}
		template <typename TStr> void PhShape(char const* name, unsigned int colour, ph::ShapePolytope const& poly, m4x4 const& o2w, TStr& str)
		{
			//Polytope(name, colour, o2w, poly.vert_begin(), poly.vert_end(), str);
		}
		template <typename TStr> void PhShape(const char* name, const char* colour, ph::ShapeTriangle const& tri, m4x4 const& o2w, TStr& str)
		{
			//Triangle(name, colour, o2w, tri.m_v.x, tri.m_v.y, tri.m_v.z, str);
		}
		template <typename TStr> void PhShape(char const* name, unsigned int colour, ph::ShapeArray const& arr, m4x4 const& o2w, TStr& str)
		{
			//using namespace pr::ph;
			//GroupStart(name, str);
			//for( Shape const *s = arr.begin(), *s_end = arr.end(); s != s_end; s = Inc(s) )
			//	PhShape(GetShapeTypeStr(s->m_type), colour, *s, s->m_shape_to_model, str);
			//Transform(o2w, str);
			//GroupEnd(str);
		}
		template <typename TStr> void PhShape(char const* name, unsigned int colour, ph::Shape const& shape, m4x4 const& o2w, TStr& str)
		{
			//using namespace pr::ph;
			//switch( shape.m_type )
			//{
			//case EShape_Sphere:		PhShape(name, colour, shape_cast<ShapeSphere>  (shape), o2w, str); break;
			//case EShape_Box:		PhShape(name, colour, shape_cast<ShapeBox>     (shape), o2w, str); break;
			//case EShape_Cylinder:	PhShape(name, colour, shape_cast<ShapeCylinder>(shape), o2w, str); break;
			//case EShape_Polytope:	PhShape(name, colour, shape_cast<ShapePolytope>(shape), o2w, str); break;
			//case EShape_Triangle:	PhShape(name, colour, shape_cast<ShapeTriangle>(shape), o2w, str); break;
			//case EShape_Array:		PhShape(name, colour, shape_cast<ShapeArray>   (shape), o2w, str); break;
			//case EShape_Terrain:	break;
			//default:				PR_ASSERT(1, false, "Ldr: unsupport shape type");
			//}
		}
		template <typename TStr> void PhRigidbody(char const* name, unsigned int colour, ph::Rigidbody const& rb, TStr& str)
		{
			//PhShape(name, colour, *rb.GetShape(), rb.ObjectToWorld(), str);
		}
		template <typename TStr> void phTriangle(ph::mesh_vs_mesh::Triangle const& tri, TStr& str)
		{
			//GroupStart("ResultTriangle", str);
			//Triangle("tri", "FFFF00FF", tri.m_vert[0].m_r, tri.m_vert[1].m_r, tri.m_vert[2].m_r, str);
			//Line("line", "FFFF00FF", v4::Origin(), tri.m_direction * tri.m_distance * 1.02f, str);
			//GroupEnd(str);
		}
		template <typename TStr> void PhCollisionScene(char const* name, unsigned int colour, ph::Shape const& shapeA, m4x4 const& a2w, ph::Shape const& shapeB, m4x4 const& b2w, TStr& str)
		{
			//GroupStart(name, colour, str);
			//PhShape("ShapeA", "80FF0000", shapeA, a2w, str);
			//PhShape("ShapeB", "800000FF", shapeB, b2w, str);
			//GroupEnd(str);
		}
		template <typename TStr> void phContact(char const* name, unsigned int colour, ph::Contact const& contact, TStr& str)
		{
			//GroupStart(name, str);
			//Box  ("p"     , colour, contact.m_pointA, 0.02f, str);
			//Box  ("q"     , colour, contact.m_pointB, 0.02f, str);
			//LineD("normal", colour, contact.m_pointB, contact.m_normal *  contact.m_depth, str);
			//LineD("normal", colour, contact.m_pointA, contact.m_normal * -contact.m_depth, str);
			//GroupEnd(str);
		}
		template <typename TStr> void phContactManifold(char const* name, unsigned int colour, ph::ContactManifold const& manifold, TStr& str)
		{
			//for( uint32_t i = 0; i != manifold.Size(); ++i )
			//	phContact(name, colour, manifold[i], str);
		}
		template <typename TStr> void PhSupport(ph::Support const& support, TStr& str)
		{
			//// Output the rigidbody that owns this support
			//ph::Rigidbody const& rb = GetRBFromSupport(support);
			//PhRigidbody("Supported", "FF800000", rb, str);
			//
			//// Output the legs
			//for( uint32_t i = 0; i != support.m_num_supports; ++i )
			//{
			//	GroupStart("Leg", str);
			//	support::Leg const& leg = support.m_leg[i];
			//	
			//	// 'leg' should be part of a chain, find the owner of the chain
			//	for( support::Leg const* owner = pod_chain::Begin(leg); owner != pod_chain::End(leg); owner = owner->m_next )
			//	{
			//		if( owner->m_support_number != -1 ) continue;
			//		PhRigidbody("Support", "FF008000", GetRBFromSupport(GetSupportFromLeg(*owner)), str);
			//	}

			//	// Output the ray from the centre of the object to the support
			//	LineD("point", "FFFFFF00", rb.Position(), leg.m_point, str);
			//	GroupEnd(str);
			//}
		}
	}
}
#pragma warning(pop)
#endif

#endif
