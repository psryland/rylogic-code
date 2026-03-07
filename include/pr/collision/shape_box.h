//*********************************************
// Collision
//  Copyright (C) Rylogic Ltd 2016
//*********************************************
#pragma once
#include "pr/collision/forward.h"
#include "pr/collision/shape.h"

namespace pr::collision
{
	// A box collision shape
	struct ShapeBox
	{
		Shape m_base;
		v4    m_radius;

		ShapeBox() = default;
		explicit ShapeBox(v4 dim, m4x4 const& shape_to_parent = m4x4::Identity(), MaterialId material_id = 0, Shape::EFlags flags = Shape::EFlags::None)
			:m_base(EShape::Box, sizeof(ShapeBox), shape_to_parent, material_id, flags)
			,m_radius(dim * 0.5f)
		{
			assert(dim.x > 0 && dim.y > 0 && dim.z > 0 && dim.w == 0);
			m_base.m_bbox = CalcBBox(*this);
		}
		ShapeBox(BBox bbox, MaterialId material_id = 0, Shape::EFlags flags = Shape::EFlags::None)
			:ShapeBox(bbox.m_radius, m4x4::Translation(bbox.m_centre), material_id, flags)
		{}
		ShapeBox(OBox const& obox, MaterialId material_id = 0, Shape::EFlags flags = Shape::EFlags::None)
			:ShapeBox(obox.m_radius, obox.m_box_to_world, material_id, flags)
		{}
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
	static_assert(ShapeType<ShapeBox>);

	// Return the bounding box for a box shape
	inline BBox pr_vectorcall CalcBBox(ShapeBox const& shape)
	{
		return BBox(v4::Origin(), shape.m_radius);
	}

	// Shift the centre of a box shape
	inline void pr_vectorcall ShiftCentre(ShapeBox&, v4 shift)
	{
		assert("impossible to shift the centre of an implicit object" && FEql(shift, v4::Zero()));
		(void)shift; 
	}

	// Return a support vertex for a box shape
	inline v4 pr_vectorcall SupportVertex(ShapeBox const& shape, v4 direction, int, int& sup_vert_id)
	{
		int sign_x = (direction.x > 0.0f);
		int sign_y = (direction.y > 0.0f);
		int sign_z = (direction.z > 0.0f);

		sup_vert_id = (sign_z << 2) | (sign_y << 1) | (sign_x);
		return v4(
			(2.0f * sign_x - 1.0f) * shape.m_radius.x,
			(2.0f * sign_y - 1.0f) * shape.m_radius.y,
			(2.0f * sign_z - 1.0f) * shape.m_radius.z,
			1.0f);
	}

	// Returns the closest point on 'shape' to 'point'. 'shape' and 'point' are in the same space
	inline void pr_vectorcall ClosestPoint(ShapeBox const& shape, v4 point, float& distance, v4& closest)
	{
		closest = point;
		distance = 0.0f;
		for (int i = 0; i != 3; ++i)
		{
			if (point[i] > shape.m_radius[i])
			{
				distance += Sqr(point[i] - shape.m_radius[i]);
				closest[i] = +shape.m_radius[i];
			}
			else if (point[i] < -shape.m_radius[i])
			{
				distance += Sqr(point[i] + shape.m_radius[i]);
				closest[i] = -shape.m_radius[i];
			}
		}
		distance = Sqrt(distance);
	}
}