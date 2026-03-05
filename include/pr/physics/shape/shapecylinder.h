//*********************************************
// Physics engine
//  Copyright (c) Rylogic Ltd 2006
//*********************************************

#pragma once
#ifndef PR_PHYSICS_SHAPE_CYLINDER_H
#define PR_PHYSICS_SHAPE_CYLINDER_H

#include "pr/physics/shape/shape.h"

namespace pr
{
	namespace ph
	{
		struct ShapeCylinder
		{
			Shape	m_base;
			float	m_radius;
			float	m_height;	// Actually half height (y axis currently)

			enum { EShapeType = EShape_Cylinder };
			static ShapeCylinder	make(float radius, float height, const m4x4& shape_to_model, MaterialId material_id, uint32_t flags) { ShapeCylinder s; s.set(radius, height, shape_to_model, material_id, flags); return s; }
			ShapeCylinder&			set (float radius, float height, const m4x4& shape_to_model, MaterialId material_id, uint32_t flags);
			operator Shape const&() const	{ return m_base; }
			operator Shape& ()				{ return m_base; }
		};

		// Shape functions
		BBox&	CalcBBox			(ShapeCylinder const& shape, BBox& bbox);
		MassProperties& CalcMassProperties	(ShapeCylinder const& shape, float density, MassProperties& mp);
		void			ShiftCentre			(ShapeCylinder& shape, v4& shift);
		v4				SupportVertex		(ShapeCylinder const& shape, v4 direction, std::size_t hint_vert_id, std::size_t& sup_vert_id);
		void			ClosestPoint		(ShapeCylinder const& shape, v4 point, float& distance, v4& closest);
	}
}

#endif
