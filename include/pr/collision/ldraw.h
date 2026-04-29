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
			: LdrGroup(name, 0xFFFFFFFF)
		{
			group_colour(colour);
		}
		LdrCollisionShape& shape(collision::Shape const& shape)
		{
			using namespace collision;
			switch (shape.m_type)
			{
				case EShape::Sphere:
				{
					auto& s = shape_cast<ShapeSphere>(shape);
					Sphere().sphere(s.m_radius).facets(5).o2w(s.m_base.m_s2p);
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
					Cylinder().cylinder(2 * s.m_hlength, s.m_radius).facets(1, 50).end_caps().o2w(s.m_base.m_s2p);
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

	struct LdrCollisionContact : LdrGroup
	{
		LdrCollisionContact(seri::Name name = {}, seri::Colour colour = {})
			: LdrGroup(name, 0xFFFFFFFF)
		{
			group_colour(colour ? colour : 0xFFFFFF00);
		}
		LdrCollisionContact& contact(collision::Contact const& contact, float scale = 1.0f)
		{
			using namespace collision;
			switch (contact.m_feature)
			{
				case EFeature::None:
				{
					break;
				}
				case EFeature::Vert:
				{
					Sphere("Manifold").sphere(0.01f * scale).facets(2).pos(contact.m_manifold[0]);
					Box("Corners")
						.box(0.005f, contact.m_manifold[0]);
					break;
				}
				case EFeature::Edge:
				{
					Line("Manifold").line(contact.m_manifold[0], contact.m_manifold[1]);
					Box("Corners")
						.box(0.005f, contact.m_manifold[0])
						.box(0.005f, contact.m_manifold[1]);
					break;
				}
				case EFeature::Tri:
				{
					Triangle("Manifold").tri(contact.m_manifold[0], contact.m_manifold[1], contact.m_manifold[2]);
					Box("Corners")
						.box(0.005f, contact.m_manifold[0])
						.box(0.005f, contact.m_manifold[1])
						.box(0.005f, contact.m_manifold[2]);
					break;
				}
				case EFeature::Quad:
				{
					// LDraw expects quads in 'S' vert order
					Quad("Manifold").quad(contact.m_manifold[0], contact.m_manifold[1], contact.m_manifold[3], contact.m_manifold[2]);
					Box("Corners")
						.box(0.005f, contact.m_manifold[0])
						.box(0.005f, contact.m_manifold[1])
						.box(0.005f, contact.m_manifold[2])
						.box(0.005f, contact.m_manifold[3]);
					break;
				}
				default:
				{
					throw std::runtime_error("Unknown contact feature type");
				}
			}
			auto& norm = Line("Normal");
			for (auto const& pt : contact.Points())
			{
				norm.line(pt , pt + 0.5f * contact.m_axis * contact.m_depth);
				norm.line(pt , pt - 0.5f * contact.m_axis * contact.m_depth);
			}
			Sphere("Point").sphere(0.008f).pos(contact.Point());
			return *this;
		}
	};
}
