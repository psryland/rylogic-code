//*********************************************
// Collision
//  Copyright (c) Rylogic Ltd 2006
//*********************************************
#pragma once
#include "pr/collision/forward.h"
#include "pr/collision/shape.h"

namespace pr::collision
{
	// A Line segment shape with optional thickness.
	// The line is aligned to the Z axis, centred on the origin, with half-length = m_radius.
	// When m_thickness > 0, the line has a cylindrical collision envelope (without hemispherical end-caps).
	// The collision normal always comes from the OTHER shape's geometry, not from the line's end-caps.
	struct ShapeLine
	{
		Shape m_base;
		float m_radius;    // Half-length along the Z axis. Length = 2 * m_radius.
		float m_thickness; // Collision radius perpendicular to the Z axis. 0 = infinitely thin.
		int pad[2];

		ShapeLine() = default;
		explicit ShapeLine(float length, float thickness = 0, m4x4 const& shape_to_parent = m4x4::Identity(), MaterialId material_id = 0, Shape::EFlags flags = Shape::EFlags::None)
			:m_base(EShape::Line, sizeof(ShapeLine), shape_to_parent, material_id, flags)
			,m_radius(length * 0.5f)
			,m_thickness(thickness * 0.5f)
		{
			m_base.m_bbox = CalcBBox(*this);
		}
		explicit ShapeLine(v4 a, v4 b, float thickness = 0, m4x4 const& shape_to_parent = m4x4::Identity(), MaterialId material_id = 0, Shape::EFlags flags = Shape::EFlags::None)
			:ShapeLine(Length(b-a), thickness, shape_to_parent * m4x4::Transform(b-a, v4::ZAxis(), (a+b)/2.0f), material_id, flags)// checkme
		{}
		
		// Conversion
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
	static_assert(ShapeType<ShapeLine>);
	static_assert((sizeof(ShapeLine) & 0xf) == 0);

	// Return the bounding box for a line shape.
	// Includes the thickness envelope (hemispherical ends for bounding purposes).
	inline BBox pr_vectorcall CalcBBox(ShapeLine const& shape)
	{
		return BBox(v4::Origin(), v4(shape.m_thickness, shape.m_thickness, shape.m_radius + shape.m_thickness, 0.0f));
	}

	// Shift the centre of a line
	inline void pr_vectorcall ShiftCentre(ShapeLine&, v4 shift)
	{
		assert("impossible to shift the centre of an implicit object" && FEql(shift, v4::Zero()));
		(void)shift; 
	}

	// Return a support vertex for a line.
	// Uses hemispherical end-caps for accurate contact point calculation.
	inline v4 pr_vectorcall SupportVertex(ShapeLine const& shape, v4 direction, int, int& sup_vert_id)
	{
		// Choose the endpoint furthest in the given direction
		sup_vert_id = direction.z >= 0;
		auto vert = v4(0, 0, Sign(direction.z) * shape.m_radius, 1);

		// Add hemispherical end-cap offset for thick lines
		if (shape.m_thickness > 0)
		{
			auto len_sq = LengthSq(direction);
			if (len_sq > Sqr(math::tiny<float>))
				vert += shape.m_thickness * direction / Sqrt(len_sq);
		}
		return vert;
	}

	// Find the nearest point and distance from a point to a shape. 'shape' and 'point' are in the same space
	inline void pr_vectorcall ClosestPoint(ShapeLine const& shape, v4 point, float& distance, v4& closest)
	{
		if (Abs(point.z) < shape.m_radius)
		{
			closest = v4(0, 0, point.z, 1);
			distance = Len(point.x, point.y);
		}
		else
		{
			closest = v4(0, 0, Sign(point.z) * shape.m_radius, 1);
			distance = Length(point - closest);
		}

		// Reduce distance by thickness (the surface is m_thickness away from the axis)
		distance = std::max(0.0f, distance - shape.m_thickness);
	}
}
