//*********************************************
// Collision
//  Copyright (c) Rylogic Ltd 2016
//*********************************************
#pragma once

#include "pr/collision/shape.h"
#include "pr/collision/shape_sphere.h"
#include "pr/collision/shape_box.h"
#include "pr/collision/shape_line.h"
#include "pr/collision/shape_triangle.h"
#include "pr/collision/shape_polytope.h"
#include "pr/collision/shape_array.h"
#include "pr/collision/ray.h"
#include "pr/collision/ray_cast.h"

namespace pr::collision
{
	// Return a shape to use in place of a real shape for objects that don't need a shape really
	inline Shape* NoShape()
	{
		static Shape s_no_shape(EShape::NoShape, sizeof(Shape));
		return &s_no_shape;
	}

	// Calculate the bounding box for a shape (in parent space, i.e. includes m_s2p)
	template <typename>
	BBox pr_vectorcall CalcBBox(Shape const& shape)
	{
		switch (shape.m_type)
		{
			#define PR_COLLISION_SHAPE_CALCBBOX(name, comp) case EShape::name: return CalcBBox(shape_cast<Shape##name>(shape));
			PR_COLLISION_SHAPES(PR_COLLISION_SHAPE_CALCBBOX)
			#undef PR_COLLISION_SHAPE_CALCBBOX
			default: assert("Unknown primitive type" && false); return BBox::Reset();
		}
	}

	// Shift the centre a shape. Updates 'shape.m_shape_to_model' and 'shift'
	template <typename>
	void pr_vectorcall ShiftCentre(Shape& shape, v4 shift)
	{
		switch (shape.m_type)
		{
			#define PR_COLLISION_SHAPE_SHIFTCENTRE(name, comp) case EShape::name: return ShiftCentre(shape_cast<Shape##name>(shape), shift);
			PR_COLLISION_SHAPES(PR_COLLISION_SHAPE_SHIFTCENTRE)
			#undef PR_COLLISION_SHAPE_SHIFTCENTRE
			default: assert("Unknown primitive type" && false); return;
		}
	}

	// Returns the support vertex for 'shape' in 'direction'. 'direction' is in shape space
	template <typename>
	v4 pr_vectorcall SupportVertex(Shape const& shape, v4 direction, int hint_vert_id, int& sup_vert_id)
	{
		switch (shape.m_type)
		{
			#define PR_COLLISION_SHAPE_SUPPORTVERTEX(name, comp) case EShape::name: return SupportVertex(shape_cast<Shape##name>(shape), direction, hint_vert_id, sup_vert_id);
			PR_COLLISION_SHAPES(PR_COLLISION_SHAPE_SUPPORTVERTEX)
			#undef PR_COLLISION_SHAPE_SUPPORTVERTEX
			default: assert("Unknown primitive type" && false); return v4Zero;
		}
	}

	// Returns the closest point on 'shape' to 'point'. 'shape' and 'point' are in the same space
	template <typename>
	void pr_vectorcall ClosestPoint(Shape const& shape, v4 point, float& distance, v4& closest)
	{
		switch (shape.m_type)
		{
			#define PR_COLLISION_SHAPE_CLOSESTPOINT(name, comp) case EShape::name: return ClosestPoint(shape_cast<Shape##name>(shape), point, distance, closest);
			PR_COLLISION_SHAPES(PR_COLLISION_SHAPE_CLOSESTPOINT)
			#undef PR_COLLISION_SHAPE_CLOSESTPOINT
			default: assert("Unknown primitive type" && false); return;
		}
	}
}
