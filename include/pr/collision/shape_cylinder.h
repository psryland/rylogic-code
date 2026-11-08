//*********************************************
// Collision
//  Copyright (c) Rylogic Ltd 2026
//*********************************************
#pragma once
#include "pr/collision/forward.h"
#include "pr/collision/shape.h"

namespace pr::collision
{
	// A solid cylinder with flat end caps.
	// The cylinder axis is the shape-space Z axis. The shape is centred on the origin, the caps lie at z = +/-m_hlength,
	// and the curved side is the set of points at distance m_radius from the Z axis.
	struct ShapeCylinder
	{
		Shape m_base;
		float m_hlength; // Half-length along the Z axis. Length = 2 * m_hlength.
		float m_radius;  // Radius of the curved side and the end caps.
		int pad[2];

		ShapeCylinder() = default;
		explicit ShapeCylinder(float length, float radius, m4x4 const& shape_to_root = m4x4::Identity(), MaterialId material_id = 0, Shape::EFlags flags = Shape::EFlags::None)
			: m_base(EShape::Cylinder, sizeof(ShapeCylinder), shape_to_root, BBox::Reset(), material_id, flags)
			, m_hlength(length * 0.5f)
			, m_radius(radius)
			, pad()
		{
			// A cylinder needs a real cross-section; a zero length cylinder is a disc and is allowed.
			assert(radius > 0 && length >= 0);
			m_base.m_bbox = CalcBBox(*this);
		}
		explicit ShapeCylinder(v4 a, v4 b, float radius, m4x4 const& shape_to_root = m4x4::Identity(), MaterialId material_id = 0, Shape::EFlags flags = Shape::EFlags::None)
			:ShapeCylinder(Length(b - a), radius, shape_to_root * m4x4::Transform(b - a, v4::ZAxis(), (a + b) / 2.0f), material_id, flags)
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
	static_assert(ShapeType<ShapeCylinder>);
	static_assert((sizeof(ShapeCylinder) & 0xf) == 0);

	// Number of sides of the regular polygon that represents a cap in contact manifolds.
	// The polygon is inscribed in the rim, so its corners are true rim points.
	static constexpr int CylinderCapSides = FeaturePolygonMaxSides;

	// Sine of the angle within which a direction is treated as parallel to a cap normal or a side normal.
	// Support directions come from iterative solvers and shape-space transforms, so exact ties are rare.
	// Without this tolerance a resting cylinder would return a single rim point instead of its full cap or side line.
	static constexpr float CylinderFeatureTol = 1e-3f;

	// Return the bounding box for a cylinder shape
	inline BBox pr_vectorcall CalcBBox(ShapeCylinder const& shape)
	{
		return BBox(v4::Origin(), v4(shape.m_radius, shape.m_radius, shape.m_hlength, 0.0f));
	}

	// Return a support vertex for a cylinder.
	// The support point is on the cap rim furthest in 'direction'. When 'direction' is parallel to the axis, every rim point
	// ties, so the cap centre is returned. That is still a valid support point because it has the same projection.
	inline v4 pr_vectorcall SupportVertex(ShapeCylinder const& shape, v4 direction, int, int& sup_vert_id)
	{
		// Select the cap facing 'direction'
		sup_vert_id = direction.z >= 0;
		auto vert = v4(0, 0, direction.z >= 0 ? +shape.m_hlength : -shape.m_hlength, 1);

		// Move out to the rim in the radial direction of 'direction'
		auto xy_len_sq = Sqr(direction.x) + Sqr(direction.y);
		if (xy_len_sq > Sqr(math::tiny<float>))
		{
			auto scale = shape.m_radius / Sqrt(xy_len_sq);
			vert.x = direction.x * scale;
			vert.y = direction.y * scale;
		}
		return vert;
	}

	// Find the nearest point and distance from a point to a solid cylinder. 'shape' and 'point' are in the same space.
	// Points inside the cylinder return zero distance and themselves as the closest point.
	inline void pr_vectorcall ClosestPoint(ShapeCylinder const& shape, v4 point, float& distance, v4& closest)
	{
		// Clamp the radial component to the side surface
		closest = point;
		auto rho = Len(point.x, point.y);
		if (rho > shape.m_radius)
		{
			closest.x = point.x * shape.m_radius / rho;
			closest.y = point.y * shape.m_radius / rho;
		}

		// Clamp the axial component to the caps
		closest.z = Clamp(point.z, -shape.m_hlength, +shape.m_hlength);
		closest.w = 1.0f;
		distance = Length(point - closest);
	}
}
