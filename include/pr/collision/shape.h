//*********************************************
// Collision
//  Copyright (C) Rylogic Ltd 2016
//*********************************************
// A Shape is the basic type used for narrow phase collision.
// It may be a single shape or a collection of child shapes.
// In collision detection, collision pairs that are returned from the broad
// phase are passed to the collision dispatcher which creates a collision agent
// containing the appropriate narrow phase collision detection function.
//
// Notes:
// * Shapes MUST be memory location independent. (i.e. no pointers, byte offsets only)
//	 This is so they can be copied around/saved to file/appended to/etc
// * Shapes must have their origin within the shape. This is a requirement of collision
//	 detection which uses the relative positions of the centres as a starting point for
//	 finding the overlap between objects.
// * Shapes for rigid bodies should be in centre of mass frame
#pragma once
#include "pr/collision/forward.h"

namespace pr::collision
{
	// New Shape CheckList:
	//  - add entry in 'PR_COLLISION_SHAPES'
	//  - add new shape_type.h file and implement
	//  - add include to shapes.h
	//  - add to support.h
	//  - update collision.h

	// Shape type enum
	// These are trivially copyable types that have 'Shape' as the first member.
	// Order is important because it affects the collision detection tri-table.
	enum class EShape :int
	{
		// Special value to indicate the shape is a dummy object
		NoShape = -1,

		// Primitive shape types: x(Name, Composite)
		#define PR_COLLISION_SHAPES(x)\
		x(Sphere  , false)\
		x(Box     , false)\
		x(Line    , false)\
		x(Triangle, false)\
		x(Polytope, false)\
		x(Array   , true )
		#define PR_ENUM(name, comp) name,
		PR_COLLISION_SHAPES(PR_ENUM)
		#undef PR_ENUM

		NumberOf,
	};
	inline char const* ToString(EShape shape)
	{
		switch (shape)
		{
			case EShape::NoShape: return "NoShape";
			#define PR_ENUM(name, comp) case EShape::name: return #name;
			PR_COLLISION_SHAPES(PR_ENUM)
			#undef PR_ENUM
			default: assert("Unknown shape type" && false); return "Unknown";
		}
	}

	// Physics material id
	using MaterialId = unsigned int;

	// Shape base. All shapes must have this as their first member.
	struct Shape
	{
		enum class EFlags
		{
			None                       = 0,
			WholeShapeTerrainCollision = 1 << 0, // Pass the whole shape to the terrain collision function
			_flags_enum = 0,
		};

		// Transform from shape space to parent shape space (or physics model space for root objects)
		m4x4 m_s2p;

		// A bounding box for the shape (and its children) (in shape space).
		BBox m_bbox;

		// The type of shape this is. One of EShape
		EShape m_type;

		// The physics material that this shape is made out of
		MaterialId m_material_id;

		// Flags for the shape. Bitwise OR of Shape::EFlags
		EFlags m_flags;

		// The size in bytes of this shape and its data
		size_t m_size;

		Shape() = default;
		Shape(EShape type, size_t size, m4x4 const& shape_to_parent = m4x4::Identity(), MaterialId material_id = 0, EFlags flags = EFlags::None)
			:m_s2p(shape_to_parent)
			,m_bbox(BBox::Reset())
			,m_type(type)
			,m_material_id(material_id)
			,m_flags(flags)
			,m_size(size)
		{}
	};

	// Traits/Concepts
	template <typename Shp> struct shape_traits
	{
		static constexpr EShape shape_type = EShape::NoShape;
		static constexpr bool composite = false;
		static constexpr bool is_shape_v = false;
	};
	template <> struct shape_traits<Shape>
	{
		static constexpr EShape shape_type = EShape::NoShape;
		static constexpr bool composite = false;
		static constexpr bool is_shape_v = true;
	};

	// Specializations for each concrete shape type, generated from the macro.
	// These MUST be defined before any code that might instantiate shape_cast
	// or check ShapeType, otherwise the primary template gets implicitly
	// instantiated with is_shape_v=false and the later explicit specialization
	// in the individual shape headers becomes illegal (C2766/C2908).
	#define PR_ENUM(name, comp)\
	template <> struct shape_traits<Shape##name>\
	{\
		static constexpr EShape shape_type = EShape::name;\
		static constexpr bool composite = comp;\
		static constexpr bool is_shape_v = true;\
	};
	PR_COLLISION_SHAPES(PR_ENUM)
	#undef PR_ENUM

	// Standard shape functions
	BBox pr_vectorcall CalcBBox(Shape const& shape);
	void pr_vectorcall ShiftCentre(Shape&, v4 shift);
	v4   pr_vectorcall SupportVertex(Shape const& shape, v4 direction, int hint_vert_id, int& sup_vert_id);
	void pr_vectorcall ClosestPoint(Shape const& shape, v4 point, float& distance, v4& closest);

	#define PR_ENUM(name, comp)\
	struct Shape##name; /*forward declaration*/ \
	BBox pr_vectorcall CalcBBox(Shape##name const& shape);\
	void pr_vectorcall ShiftCentre(Shape##name& shape, v4 shift);\
	v4   pr_vectorcall SupportVertex(Shape##name const& shape, v4 direction, int hnt_vert_id, int& sup_vert_id);\
	void pr_vectorcall ClosestPoint(Shape##name const& shape, v4 point, float& distance, v4& closest);
	PR_COLLISION_SHAPES(PR_ENUM)
	#undef PR_ENUM

	// Shape concepts
	template <typename T>
	concept ShapeType = shape_traits<std::remove_cv_t<T>>::is_shape_v;
	
	template <typename T>
	concept CompositeShapeType = ShapeType<T> && shape_traits<T>::composite;

	static_assert(ShapeType<Shape>);
	static_assert((sizeof(Shape) & 0xf) == 0);

	// Cast 'Shape' to a specific Shape type
	template <ShapeType Shp> inline Shp const& shape_cast(Shape const& shape)
	{
		assert("Invalid shape cast" && shape.m_type == shape_traits<Shp>::shape_type);
		return reinterpret_cast<Shp const&>(shape);
	}
	template <ShapeType Shp> inline Shp const* shape_cast(Shape const* shape)
	{
		assert("Invalid shape cast" && (shape == nullptr || shape->m_type == shape_traits<Shp>::shape_type));
		return reinterpret_cast<Shp const*>(shape);
	}
	template <ShapeType Shp> inline Shp& shape_cast(Shape& shape)
	{
		assert("Invalid shape cast" && shape.m_type == shape_traits<Shp>::shape_type);
		return reinterpret_cast<Shp&>(shape);
	}
	template <ShapeType Shp> inline Shp* shape_cast(Shape* shape)
	{
		assert("Invalid shape cast" && (shape == nullptr || shape->m_type == shape_traits<Shp>::shape_type));
		return reinterpret_cast<Shp*>(shape);
	}

	// Cast a Shape type to 'Shape'
	template <ShapeType Shp> inline Shape const& shape_cast(Shp const& shape)
	{
		return shape.m_base;
	}
	template <ShapeType Shp> inline Shape const* shape_cast(Shp const* shape)
	{
		return shape ? &shape->m_base : nullptr;
	}
	template <ShapeType Shp> inline Shape& shape_cast(Shp& shape)
	{
		return shape.m_base;
	}
	template <ShapeType Shp> inline Shape* shape_cast(Shp* shape)
	{
		return shape ? &shape->m_base : nullptr;
	}
	inline Shape const& shape_cast(Shape const& shape)
	{
		return shape;
	}
	inline Shape const* shape_cast(Shape const* shape)
	{
		return shape;
	}
	inline Shape& shape_cast(Shape& shape)
	{
		return shape;
	}
	inline Shape* shape_cast(Shape* shape)
	{
		return shape;
	}

	// Increment a shape pointer: for (Shape const *s = shape.begin(), *s_end = shape.end(); s != s_end; s = next(s)) {}
	template <ShapeType Shp> inline Shp const* next(Shp const* p)
	{
		return reinterpret_cast<Shp const*>(byte_ptr(p) + p->m_base.m_size);
	}
	template <ShapeType Shp> inline Shp* next(Shp* p)
	{
		return reinterpret_cast<Shp*>(byte_ptr(p) + p->m_base.m_size);
	}
	template <> inline Shape const* next(Shape const* p)
	{
		return reinterpret_cast<Shape const*>(byte_ptr(p) + p->m_size);
	}
	template <> inline Shape* next(Shape* p)
	{
		return reinterpret_cast<Shape*>(byte_ptr(p) + p->m_size);
	}

	// Return a shape to use in place of a real shape for objects that don't need a shape really
	inline Shape& NoShape()
	{
		static Shape s_no_shape(EShape::NoShape, sizeof(Shape));
		return s_no_shape;
	}

	// Calculate the bounding box for a shape (in parent space, i.e. includes m_s2p)
	inline BBox pr_vectorcall CalcBBox(Shape const& shape)
	{
		switch (shape.m_type)
		{
			#define PR_ENUM(name, comp) case EShape::name: return CalcBBox(shape_cast<Shape##name>(shape));
			PR_COLLISION_SHAPES(PR_ENUM)
			#undef PR_ENUM
			default: assert("Unknown primitive type" && false); return BBox::Reset();
		}
	}

	// Shift the centre a shape. Updates 'shape.m_shape_to_model' and 'shift'
	inline void pr_vectorcall ShiftCentre(Shape& shape, v4 shift)
	{
		switch (shape.m_type)
		{
			#define PR_ENUM(name, comp) case EShape::name: return ShiftCentre(shape_cast<Shape##name>(shape), shift);
			PR_COLLISION_SHAPES(PR_ENUM)
			#undef PR_ENUM
			default: assert("Unknown primitive type" && false); return;
		}
	}

	// Returns the support vertex for 'shape' in 'direction'. 'direction' is in shape space
	inline v4 pr_vectorcall SupportVertex(Shape const& shape, v4 direction, int hint_vert_id, int& sup_vert_id)
	{
		switch (shape.m_type)
		{
			#define PR_ENUM(name, comp) case EShape::name: return SupportVertex(shape_cast<Shape##name>(shape), direction, hint_vert_id, sup_vert_id);
			PR_COLLISION_SHAPES(PR_ENUM)
			#undef PR_ENUM
			default: assert("Unknown primitive type" && false); return v4::Zero();
		}
	}

	// Returns the closest point on 'shape' to 'point'. 'shape' and 'point' are in the same space
	inline void pr_vectorcall ClosestPoint(Shape const& shape, v4 point, float& distance, v4& closest)
	{
		switch (shape.m_type)
		{
			#define PR_ENUM(name, comp) case EShape::name: return ClosestPoint(shape_cast<Shape##name>(shape), point, distance, closest);
			PR_COLLISION_SHAPES(PR_ENUM)
			#undef PR_ENUM
			default: assert("Unknown primitive type" && false); return;
		}
	}
}
