//*********************************************
// Collision
//  Copyright (c) Rylogic Ltd 2016
//*********************************************
#pragma once
#include "pr/math/math.h"
#include "pr/gfx/colour.h"
#include "pr/view3d-12/ldraw/ldraw_builder.h"
#include "pr/collision/shape.h"
#include "pr/collision/shape_sphere.h"
#include "pr/collision/shape_box.h"
#include "pr/collision/shape_line.h"
#include "pr/collision/shape_triangle.h"
#include "pr/collision/shape_array.h"
#include "pr/collision/penetration.h"
#include "pr/collision/support.h"

namespace pr::rdr12::ldraw
{
	struct LdrPhysicsShape : fluent::LdrBase<LdrPhysicsShape>
	{
		collision::Shape const* m_shape;

		LdrPhysicsShape()
			: m_shape()
		{}

		// Add a physics shape
		LdrPhysicsShape& shape(collision::Shape const& shape)
		{
			m_shape = &shape;
			return *this;
		}

		// Write to 'out'
		template <fluent::WriterType Writer, typename TOut>
		void WriteTo(TOut& out) const
		{
			struct L
			{
				TOut& m_out;
				LdrPhysicsShape const& m_me;
				L(LdrPhysicsShape const& me, TOut& out) :m_out(out), m_me(me) {}

				void Write(collision::ShapeSphere const& shape) const
				{
					fluent::LdrSphere().radius(shape.m_radius).o2w(shape.m_base.m_s2p).WriteTo<Writer>(m_out);
				}
				void Write(collision::ShapeBox const& shape) const
				{
					fluent::LdrBox().dim(shape.m_radius).o2w(shape.m_base.m_s2p).WriteTo<Writer>(m_out);
				}
				void Write(collision::ShapeTriangle const& shape) const
				{
					fluent::LdrTriangle().tri(shape.m_v.x, shape.m_v.y, shape.m_v.z).o2w(shape.m_base.m_s2p).WriteTo<Writer>(m_out);
				}
				void Write(collision::ShapeLine const& shape) const
				{
					auto const& s2p = shape.m_base.m_s2p;
					auto r = shape.m_radius * s2p.z;
					fluent::LdrLine().line(s2p.pos - r, s2p.pos + r).o2w(s2p).WriteTo<Writer>(m_out);
				}
				void Write(collision::ShapeArray const& shape) const
				{
					Writer::Write(m_out, EKeyword::Group, {}, {}, [&]
					{
						for (collision::Shape const* s = shape.begin(), *s_end = shape.end(); s != s_end; s = next(s))
							Write(*s);

						Writer::Append(m_out, O2W(shape.m_base.m_s2p));
					});
				}
				void Write(collision::Shape const& shape) const
				{
					using namespace collision;
					switch (shape.m_type)
					{
						case EShape::Sphere:     return Write(shape_cast<ShapeSphere>(shape));
						case EShape::Box:        return Write(shape_cast<ShapeBox>(shape));
						case EShape::Triangle:   return Write(shape_cast<ShapeTriangle>(shape));
						case EShape::Line:       return Write(shape_cast<ShapeLine>(shape));
						case EShape::Array:      return Write(shape_cast<ShapeArray>(shape));
						//case EShape::Polytope: return Shape(str, name, colour, shape_cast<ShapePolytope>(shape));
						//case EShape::Cylinder: return Shape(str, name, colour, shape_cast<ShapeCylinder>(shape));
						default: throw std::runtime_error("Unknown shape type");
					}
				}
			};

			L helper(*this, out);
			helper.Write(*m_shape);

			LdrBase::WriteTo<Writer>(out);
		}
	};
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
namespace pr::rdr12::ldraw
{
	PRUnitTest(LdrPhysicsShapeTests)
	{
		Builder L;
		auto& shape = L._<LdrPhysicsShape>();
		(void)shape;
	}
}
#endif
