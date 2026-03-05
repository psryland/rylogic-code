//*********************************************
// Physics engine
//  Copyright (c) Rylogic Ltd 2006
//*********************************************

#pragma once
#ifndef PR_PHYSICS_SHAPE_TRIANGLE_H
#define PR_PHYSICS_SHAPE_TRIANGLE_H

#include "pr/physics/shape/shape.h"

namespace pr
{
	namespace ph
	{
		struct ShapeTriangle
		{
			Shape	m_base;
			m4x4	m_v;	// <x,y,z> = verts of the triangle, w = normal. Cross(w, y-x) should point toward the interior of the triangle

			enum { EShapeType = EShape_Triangle };
			static ShapeTriangle	make(v4 a, v4 b, v4 c, const m4x4& shape_to_model, MaterialId material_id, uint32_t flags) { ShapeTriangle s; s.set(a, b, c, shape_to_model, material_id, flags); return s; }
			ShapeTriangle&			set (v4 a, v4 b, v4 c, const m4x4& shape_to_model, MaterialId material_id, uint32_t flags);
			operator Shape const&() const	{ return m_base; }
			operator Shape& ()				{ return m_base; }
		};

		// Shape functions
		BBox&           CalcBBox			(ShapeTriangle const& shape, BBox& bbox);
		MassProperties& CalcMassProperties	(ShapeTriangle const& shape, float density, MassProperties& mp);
		m3x4			CalcInertiaTensor	(ShapeTriangle const& shape);
		void			ShiftCentre			(ShapeTriangle& shape, v4& shift);
		v4				SupportVertex		(ShapeTriangle const& shape, v4 direction, std::size_t hint_vert_id, std::size_t& sup_vert_id);
		void			ClosestPoint		(ShapeTriangle const& shape, v4 point, float& distance, v4& closest);
	}
}

#endif
