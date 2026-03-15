//*********************************************
// Collision
//  Copyright (c) Rylogic Ltd 2016
//*********************************************
#pragma once
#include "pr/math/math.h"
#include "pr/gfx/colour.h"
#include "pr/common/ldraw.h"
#include "pr/collision/shape.h"
#include "pr/collision/shape_sphere.h"
#include "pr/collision/shape_box.h"
#include "pr/collision/shape_line.h"
#include "pr/collision/shape_triangle.h"
#include "pr/collision/shape_polytope.h"
#include "pr/collision/shape_array.h"
#include "pr/collision/penetration.h"
#include "pr/collision/support.h"

namespace pr::ldraw
{
	struct LdrCollisionShape : LdrGroup
	{
		LdrCollisionShape(seri::Name name = {}, seri::Colour colour = {})
			: LdrGroup(name, colour)
		{
		}

		LdrCollisionShape& shape(collision::Shape const& shape)
		{
			using namespace collision;
			switch (shape.m_type)
			{
				case EShape::Sphere:
				{
					auto& s = shape_cast<ShapeSphere>(shape);
					Sphere().radius(s.m_radius).o2w(s.m_base.m_s2p);
					break;
				}
				case EShape::Box:
				{
					auto& s = shape_cast<ShapeBox>(shape);
					Box().box(2 * s.m_radius).o2w(s.m_base.m_s2p);
					break;
				}
				case EShape::Triangle:
				{
					auto& s = shape_cast<ShapeTriangle>(shape);
					Triangle().tri(
						seri::Vec3{ s.m_v.x.x, s.m_v.x.y, s.m_v.x.z },
						seri::Vec3{ s.m_v.y.x, s.m_v.y.y, s.m_v.y.z },
						seri::Vec3{ s.m_v.z.x, s.m_v.z.y, s.m_v.z.z }
					).o2w(s.m_base.m_s2p);
					break;
				}
				case EShape::Line:
				{
					auto& s = shape_cast<ShapeLine>(shape);
					Cylinder().hr(2 * s.m_radius, s.m_thickness).o2w(s.m_base.m_s2p);
					break;
				}
				case EShape::Polytope:
				{
					auto& s = shape_cast<ShapePolytope>(shape);
					auto& p = Triangle();
					for (auto const& face : s.faces())
					{
						auto a = s.vertex(face.m_index[0]);
						auto b = s.vertex(face.m_index[1]);
						auto c = s.vertex(face.m_index[2]);
						p.tri(seri::Vec3{ a.x, a.y, a.z }, seri::Vec3{ b.x, b.y, b.z }, seri::Vec3{ c.x, c.y, c.z });
					}
					p.o2w(s.m_base.m_s2p);
					break;
				}
				case EShape::Array:
				{
					auto& s = shape_cast<ShapeArray>(shape);
					auto& grp = Group();
					for (auto const* sub = s.begin(), *end = s.end(); sub != end; sub = next(sub))
					{
						grp.Add<LdrCollisionShape>().shape(*sub);
					}
					grp.o2w(s.m_base.m_s2p);
					break;
				}
				default:
				{
					throw std::runtime_error("Unknown shape type");
				}
			}
			return *this;
		}
	};
}
