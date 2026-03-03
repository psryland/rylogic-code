//*********************************************
// Physics engine
//  Copyright (c) Rylogic Ltd 2006
//*********************************************

#include "physics/utility/stdafx.h"
#include "pr/physics/shape/shapes.h"
#include "physics/utility/assert.h"

using namespace pr;
using namespace pr::ph;

// General shape functions *******************************************************

// Constructor
Shape& Shape::set(EShape type, std::size_t size, const m4x4& shape_to_model, MaterialId material_id, uint32_t flags)
{
	m_shape_to_model	= shape_to_model;
	m_type				= type;
	m_size				= size;
	m_material_id		= material_id;
	m_flags				= flags;
	m_bbox				.unit();		// Initialised by the shape builder
	return *this;
}

// Return a shape to use in place of a real shape for objects that don't need a shape really
Shape* pr::ph::GetDummyShape()
{
	static Shape dummy = Shape::make(EShape_NoShape, sizeof(Shape), m4x4::Identity(), 0, EShapeFlags_None);
	return &dummy;
}

// Return a string describing the shape type
const char* pr::ph::GetShapeTypeStr(EShape shape_type)
{
	switch( shape_type )
	{
	case EShape_Sphere:		return "sphere";
	case EShape_Capsule:	return "capsule";
	case EShape_Cylinder:	return "cylinder";
	case EShape_Box:		return "box";
	case EShape_Polytope:	return "polytope";
	case EShape_Triangle:	return "triangle";
	case EShape_Array:		return "array";
	case EShape_BVTree:		return "BVtree";
	case EShape_Terrain:	return "terrain";
	default: PR_ASSERT(PR_DBG_PHYSICS, false, "Unknown shape type"); return "unknown";
	}
}

// Calculate the bounding box for a shape
BBox& pr::ph::CalcBBox(Shape const& shape, BBox& bbox)
{
	switch( shape.m_type )
	{
	case EShape_Sphere:		return CalcBBox(shape_cast<ShapeSphere>		(shape), bbox);
	case EShape_Box:		return CalcBBox(shape_cast<ShapeBox>		(shape), bbox);
	case EShape_Cylinder:	return CalcBBox(shape_cast<ShapeCylinder>	(shape), bbox);
	case EShape_Polytope:	return CalcBBox(shape_cast<ShapePolytope>	(shape), bbox);
	default: PR_ASSERT(PR_DBG_PHYSICS, false, "Unknown primitive type"); return bbox;
	}
}

// Calculate the mass properties of a shape
MassProperties& pr::ph::CalcMassProperties(Shape const& shape, float density, MassProperties& mp)
{
	switch( shape.m_type )
	{
	case EShape_Sphere:		return CalcMassProperties(shape_cast<ShapeSphere>	(shape), density, mp);
	case EShape_Box:		return CalcMassProperties(shape_cast<ShapeBox>		(shape), density, mp);
	case EShape_Cylinder:	return CalcMassProperties(shape_cast<ShapeCylinder>	(shape), density, mp);
	default: PR_ASSERT(PR_DBG_PHYSICS, false, "Unknown primitive type"); return mp;
	}
}

// Shift the centre a shape. Updates 'shape.m_shape_to_model' and 'shift'
void pr::ph::ShiftCentre(Shape& shape, v4& shift)
{
	switch( shape.m_type )
	{
	case EShape_Sphere:		return ShiftCentre(shape_cast<ShapeSphere>  (shape), shift);
	case EShape_Box:		return ShiftCentre(shape_cast<ShapeBox>     (shape), shift);
	case EShape_Cylinder:	return ShiftCentre(shape_cast<ShapeCylinder>(shape), shift);
	case EShape_Polytope:	return ShiftCentre(shape_cast<ShapePolytope>(shape), shift);
	default: PR_ASSERT(PR_DBG_PHYSICS, false, "Unknown primitive type"); return;
	}
}

// Returns the support vertex for 'shape' in 'direction'. 'direction' is in shape space
v4 pr::ph::SupportVertex(Shape const& shape, v4 const& direction, std::size_t hint_vert_id, std::size_t& sup_vert_id)
{
	switch( shape.m_type )
	{
	case EShape_Sphere:		return SupportVertex(shape_cast<ShapeSphere>  (shape), direction, hint_vert_id, sup_vert_id);
	case EShape_Box:		return SupportVertex(shape_cast<ShapeBox>     (shape), direction, hint_vert_id, sup_vert_id);
	case EShape_Cylinder:	return SupportVertex(shape_cast<ShapeCylinder>(shape), direction, hint_vert_id, sup_vert_id);
	case EShape_Polytope:	return SupportVertex(shape_cast<ShapePolytope>(shape), direction, hint_vert_id, sup_vert_id);
	case EShape_Triangle:	return SupportVertex(shape_cast<ShapeTriangle>(shape), direction, hint_vert_id, sup_vert_id);
	default: PR_ASSERT(PR_DBG_PHYSICS, false, "Unknown primitive type"); return v4Zero;
	}
}

void pr::ph::ClosestPoint(Shape const& shape, v4 const& point, float& distance, v4& closest)
{
	switch( shape.m_type )
	{
	case EShape_Sphere:		return ClosestPoint(shape_cast<ShapeSphere>  (shape), point, distance, closest);
	case EShape_Box:		return ClosestPoint(shape_cast<ShapeBox>     (shape), point, distance, closest);
	case EShape_Cylinder:	return ClosestPoint(shape_cast<ShapeCylinder>(shape), point, distance, closest);
	case EShape_Polytope:	PR_ASSERT(PR_DBG_PHYSICS, false, "Not Implemented"); return;
	default:				PR_ASSERT(PR_DBG_PHYSICS, false, "Unknown primitive type"); return;
	}
}

//namespace pr
//{
//	namespace ph
//	{
//		namespace cylinder
//		{
//			BBox BBox(const Primitive& prim)
//			{
//				return BBox::make(v4Origin, prim.m_radius);
//			}
//			v4 CenterOfMass(const Primitive&)
//			{
//				return v4Origin;
//			}
//			m4x4 InertiaTensor(const Primitive& prim)
//			{
//				// Note for shell, Ixx = Iyy = (1/2)mr^2 + (1/12)mL^2, Izz = mr^2
//				m4x4 moi = m4x4::Identity();
//				moi.x.x = (1.0f / 4.0f) * (prim.m_radius.x * prim.m_radius.x) + (1.0f / 3.0f) * (prim.m_radius.z * prim.m_radius.z);	// (1/4)mr^2 + (1/12)mL^2
//				moi.y.y = moi.x.x;
//				moi.z.z = (1.0f / 2.0f) * (prim.m_radius.x * prim.m_radius.x);	// (1/2)mr^2
//				return moi;
//			}
//			v4 Support(const Primitive& prim, const v4& direction)
//			{
//				prim;
//				direction;
//				PR_ASSERT(PR_DBG_PHYSICS, false);
//				return v4Zero;
//			}
//			float Volume(const Primitive& prim)
//			{
//				return (2.0f * maths::pi) * prim.m_radius.x * prim.m_radius.x * prim.m_radius.z;
//			}
//		}//namespace cylinder
//
//		namespace model
//		{
//			//*****
//			// Unpack and resolve pointers for a range of models
//			void Unpack(ph::Model* first, ph::Model* last)
//			{
//				for( ph::Model* model = first; model != last; model = model->next() )
//				{
//					for( Primitive *p = model->prim_begin(), *p_end = model->prim_end(); p != p_end; ++p )
//					{
//						switch( p->m_type )
//						{
//						case EPrimitive_Sphere:
//							p->m_Support		= sphere::Support;
//							p->m_prim_data		= 0;
//							break;
//						case EPrimitive_Cylinder:
//							p->m_Support		= cylinder::Support;
//							p->m_prim_data		= 0;
//							break;
//						case EPrimitive_Box:
//							p->m_Support		= box::Support;
//							p->m_prim_data		= 0;
//							break;
//						case EPrimitive_Polytope:
//							p->m_Support		= polytope::Support;
//							p->m_prim_data		= (uint8*)model + (std::size_t)p->m_prim_data;
//							break;
//						}
//					}
//				}
//			}
//
//			//*****
//			// Pack and convert pointers to byte offsets for a range of models
//			void Pack(ph::Model* first, ph::Model* last)
//			{
//				for( ph::Model* model = first; model != last; model = model->next() )
//				{
//					for( Primitive *p = model->prim_begin(), *p_end = model->prim_end(); p != p_end; ++p )
//					{
//						switch( p->m_type )
//						{
//						case EPrimitive_Sphere:
//						case EPrimitive_Cylinder:
//						case EPrimitive_Box:
//							p->m_Support		= 0;
//							p->m_prim_data		= 0;
//							break;
//						case EPrimitive_Polytope:
//							p->m_Support		= 0;
//							p->m_prim_data		= reinterpret_cast<void*>((uint8*)p->m_prim_data - (uint8*)model);
//							break;
//						}
//					}
//				}
//			}
//		}//namespace model
//
//		//*****
//		// Return the centre of mass for a primitive
//		v4 pr::ph::CenterOfMass(const Primitive& prim)
//		{
//			switch( prim.m_type )
//			{
//			case EPrimitive_Sphere:		return sphere	::CenterOfMass(prim);
//			case EPrimitive_Cylinder:	return cylinder	::CenterOfMass(prim);
//			case EPrimitive_Box:		return box		::CenterOfMass(prim);
//			case EPrimitive_Polytope:	return polytope	::CenterOfMass(prim);
//			default: PR_ASSERT(PR_DBG_PHYSICS, false, "Unknown primitive type"); return v4Zero;
//			}
//		}
//
//		//*****
//		// Return the inertia tensor for a primitive
//		m4x4 pr::ph::InertiaTensor(const Primitive& prim)
//		{
//			switch( prim.m_type )
//			{
//			case EPrimitive_Sphere:		return sphere	::InertiaTensor(prim);
//			case EPrimitive_Cylinder:	return cylinder	::InertiaTensor(prim);
//			case EPrimitive_Box:		return box		::InertiaTensor(prim);
//			case EPrimitive_Polytope:	return polytope	::InertiaTensor(prim);
//			default: PR_ASSERT(PR_DBG_PHYSICS, false, "Unknown primitive type"); return m4x4::Identity();
//			}
//		}
//
//		//*****
//		// Return the volume of a primitive
//		float Volume(const Primitive& prim)
//		{
//			switch( prim.m_type )
//			{
//			case EPrimitive_Sphere:		return sphere	::Volume(prim);
//			case EPrimitive_Cylinder:	return cylinder	::Volume(prim);
//			case EPrimitive_Box:		return box		::Volume(prim);
//			case EPrimitive_Polytope:	return polytope	::Volume(prim);
//			default: PR_ASSERT(PR_DBG_PHYSICS, false, "Unknown primitive type"); return 0.0f;
//			}
//		}
//	}//namespace ph
//}//namespace pr
