//*********************************************
// Collision
//  Copyright (C) Rylogic Ltd 2016
//*********************************************
#pragma once
#include "pr/collision/shape.h"
#include "pr/geometry/closest_point.h"

namespace pr::collision
{
	struct ShapeTriangle
	{
		Shape m_base;
		m4x4  m_v; // <x,y,z> = verts of the triangle, w = normal. Cross3(w, y-x) should point toward the interior of the triangle

		explicit ShapeTriangle(v4 a, v4 b, v4 c, m4x4 const& shape_to_parent = m4x4::Identity(), MaterialId material_id = 0, Shape::EFlags flags = Shape::EFlags::None)
			:m_base(EShape::Triangle, sizeof(ShapeTriangle), shape_to_parent, material_id, flags)
			,m_v(a, b, c, Normalise(Cross3(b-a,c-b)))
		{
			assert(a.w == 0.0f && b.w == 0.0f && c.w == 0.0f);
			m_base.m_bbox = CalcBBox(*this);
		}
		operator Shape const&() const
		{
			return m_base;
		}
		operator Shape&()
		{
			return m_base;
		}
		operator Shape const*() const
		{
			return &m_base;
		}
		operator Shape*()
		{
			return &m_base;
		}
	};
	static_assert(is_shape_v<ShapeTriangle>);

	// Return the bounding box for a triangle shape
	template <typename>
	BBox pr_vectorcall CalcBBox(ShapeTriangle const& shape)
	{
		auto bb = BBox::Reset();
		Grow(bb, shape.m_v.x);
		Grow(bb, shape.m_v.y);
		Grow(bb, shape.m_v.z);
		return bb;
	}

	// Shift the centre of a triangle
	template <typename>
	void pr_vectorcall ShiftCentre(ShapeTriangle& shape, v4 const& shift)
	{
		assert(shift.w == 0.0f);
		if (FEql(shift, v4Zero)) return;
		shape.m_v.x -= shift;
		shape.m_v.y -= shift;
		shape.m_v.z -= shift;
		shape.m_base.m_s2p.pos += shift;
	}

	// Return a support vertex for a triangle
	template <typename>
	v4 pr_vectorcall SupportVertex(ShapeTriangle const& shape, v4 direction, int, int& sup_vert_id)
	{
		auto d = v4{
			Dot3(direction, shape.m_v.x),
			Dot3(direction, shape.m_v.y),
			Dot3(direction, shape.m_v.z),
			0};

		sup_vert_id = MaxElementIndex(d.xyz);
		return shape.m_v[(int)sup_vert_id];
	}

	// Find the nearest point and distance from a point to a shape. 'shape' and 'point' are in the same space
	template <typename>
	void pr_vectorcall ClosestPoint(ShapeTriangle const& shape, v4 point, float& distance, v4& closest)
	{
		using namespace pr::geometry;
		closest = closest_point::PointToTriangle(point, shape.m_v.x, shape.m_v.y, shape.m_v.z);
		distance = Length(point - closest);
	}
}