//*********************************************
// Physics engine
//  Copyright (c) Rylogic Ltd 2006
//*********************************************

// A Shape is the basic type used for narrow phase collision.
// It may be a single shape or a collection of child shapes
// In collision detection, "CollisionPair"s returned from the broad phase
// are passed to the collision dispatcher which creates a collision agent
// containing the appropriate narrow phase collision detection function.
//
//  +------------------------------+
//  | Shape                        |
//  |  BBox, mass, mass tensor,    |
//  |  inv mass tensor, shape*     |
//  +------------------------------+
//  | transform                    |
//  +------------------------------+
//  | Shape Data                   |
//  +------------------------------+
//
// Notes:
// * Shapes MUST be memory location independent. (i.e. no pointers, byte offsets only)
//	 This is so that can be copied around/saved to file/appended to/etc
// * Shapes must have their origin within the shape. This is a requirement of collision
//	 detection which uses the relative positions of the centres as a starting point for
//	 finding the overlap between objects.
// * Shapes for rigidbodies should be in centre of mass frame

#pragma once
#ifndef PR_PHYSICS_SHAPE_H
#define PR_PHYSICS_SHAPE_H

#include "pr/physics/types/forward.h"

#ifndef PR_ASSERT
#	define PR_ASSERT_STR_DEFINED
#	define PR_ASSERT(grp, exp, str)
#endif

namespace pr
{
	namespace ph
	{
		enum EShapeFlags
		{
			EShapeFlags_None						= 0,
			EShapeFlags_WholeShapeTerrainCollision	= 1 << 0	// Pass the whole shape to the terrain collision function
		};

		// Shape base. All shapes must have this as their first member
		struct Shape
		{
			// Transform from shape space to physics model space (or parent shape space)
			m4x4 m_shape_to_model;

			// The type of shape this is. One of EShape
			EShape m_type;

			// The size in bytes of this shape and its data
			std::size_t m_size;

			// The physics material that this shape is made out of
			uint32_t m_material_id;

			// Flags for the shape. Bitwise OR of EShapeFlags
			uint32_t m_flags;

			// A bounding box for the shape (and its children if it's a composite shape)
			BBox m_bbox;

			static Shape make(EShape type, std::size_t size, const m4x4& shape_to_model, MaterialId material_id, uint32_t flags) { Shape s; s.set(type, size, shape_to_model, material_id, flags); return s; }
			Shape&		 set (EShape type, std::size_t size, const m4x4& shape_to_model, MaterialId material_id, uint32_t flags);
		};

		// Mass properties for an object
		struct MassProperties
		{
			m3x4	m_os_inertia_tensor;	// Object space inertia tensor
			v4		m_centre_of_mass;		// Offset to the object space centre of mass
			float	m_mass;					// Mass in kg

			MassProperties& set(m3x4 const& os_inertia_tensor, v4 centre_of_mass, float	mass) { m_os_inertia_tensor = os_inertia_tensor; m_centre_of_mass = centre_of_mass; m_mass = mass; return *this; }
		};

		// General shape functions
		Shape*			GetDummyShape		();
		const char*		GetShapeTypeStr		(EShape shape_type);
		BBox&	CalcBBox			(Shape const& shape, BBox& bbox);
		MassProperties& CalcMassProperties	(Shape const& shape, float density, MassProperties& mp);
		void			ShiftCentre			(Shape& shape, v4& shift);
		v4				SupportVertex		(Shape const& shape, v4 direction, std::size_t hint_vert_id, std::size_t& sup_vert_id);
		void			ClosestPoint		(Shape const& shape, v4 point, float& distance, v4& closest);

		// Shape casting helpers
		template <typename T> inline T const& shape_cast(Shape const& shape)			{ PR_ASSERT(1,           shape .m_type == (int)T::EShapeType, FmtS("Attempting to cast %s to %s", GetShapeTypeStr(shape .m_type), GetShapeTypeStr((pr::ph::EShape)T::EShapeType))); return reinterpret_cast<T const&>(shape); }
		template <typename T> inline T&       shape_cast(Shape&       shape)			{ PR_ASSERT(1,           shape .m_type == (int)T::EShapeType, FmtS("Attempting to cast %s to %s", GetShapeTypeStr(shape .m_type), GetShapeTypeStr((pr::ph::EShape)T::EShapeType))); return reinterpret_cast<T&      >(shape); }
		template <typename T> inline T const* shape_cast(Shape const* shape)			{ PR_ASSERT(1, !shape || shape->m_type == (int)T::EShapeType, FmtS("Attempting to cast %s to %s", GetShapeTypeStr(shape->m_type), GetShapeTypeStr((pr::ph::EShape)T::EShapeType))); return reinterpret_cast<T const*>(shape); }
		template <typename T> inline T*       shape_cast(Shape*       shape)			{ PR_ASSERT(1, !shape || shape->m_type == (int)T::EShapeType, FmtS("Attempting to cast %s to %s", GetShapeTypeStr(shape->m_type), GetShapeTypeStr((pr::ph::EShape)T::EShapeType))); return reinterpret_cast<T*      >(shape); }

		// Increment a shape pointer
		template <typename ShapeType> inline ShapeType const* Inc(ShapeType const* p) { return reinterpret_cast<ShapeType const*>(byte_ptr(p) + p->m_base.m_size); }
		template <typename ShapeType> inline ShapeType*       Inc(ShapeType*       p) { return reinterpret_cast<ShapeType*      >(byte_ptr(p) + p->m_base.m_size); }
		template <>                   inline Shape const*     Inc(Shape const*     p) { return reinterpret_cast<Shape const*    >(byte_ptr(p) + p->m_size); }
		template <>                   inline Shape*           Inc(Shape*           p) { return reinterpret_cast<Shape*          >(byte_ptr(p) + p->m_size); }
	}
}

#ifdef PR_ASSERT_STRDEFINED
#	undef PR_ASSERT_STRDEFINED
#	undef PR_ASSERT
#endif

#endif
