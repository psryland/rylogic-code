//*********************************************
// Physics engine
//  Copyright (c) Rylogic Ltd 2006
//*********************************************

#include "physics/utility/stdafx.h"
#include "pr/physics/collision/collider.h"
#include "pr/physics/collision/contactmanifold.h"
#include "pr/physics/rigidbody/rigidbody.h"
#include "pr/physics/shape/shapebox.h"
#include "pr/physics/shape/shapetriangle.h"
#include "physics/collision/collision.h"
#include "physics/utility/debug.h"
#include "physics/utility/profile.h"

using namespace pr;
using namespace pr::ph;
using namespace pr::ph::collision;
using namespace pr::geometry;

namespace pr
{
	namespace ph
	{
		namespace collision
		{
			struct Overlap
			{
				Overlap(Overlap const&);			 // No copying
				Overlap& operator =(Overlap const&); // No copying
				Overlap(ShapeBox const& box, m4x4 const& a2w, ShapeTriangle const& tri, m4x4 const& b2w)
				:m_box(box)
				,m_a2w(a2w)
				,m_tri(tri)
				,m_b2w(b2w)
				,m_penetration(limits<float>::max())
				{
					for( int i = 0; i != 3; ++ i )
					{
						m_box_radii[i] = m_a2w[i] * m_box.m_radius[i];
						m_tri_verts[i] = m_b2w    * m_tri.m_v[i];
					}
				}
				ShapeBox const&			m_box;
				m4x4 const&				m_a2w;
				ShapeTriangle const&	m_tri;
				m4x4 const&				m_b2w;
				v4						m_axis;				// Always from m_pointA to m_pointB
				float					m_penetration;		// The depth of penetration. No contact if <= 0.0f
				m3x4					m_box_radii;		// The radii of the box in world space
				m3x4					m_tri_verts;		// The verts of the triangle in world space
				Point					m_pointA;			// The point of contact on object A
				Point					m_pointB;			// The index of the vertex of the triangle, 
			};
		}//namespace collision
	}//namespace ph
}//namespace pr

// This function takes the points A and B in 'overlap' and adjusts them to
// the mostly likely point of contact based on the degrees of freedom in each point.
// 'pointA' and 'pointB' are in world space but relative to shapeA and shapeB
void GetPointOfContactBoxVsTri(v4& pointA, v4& pointB, Overlap const& overlap)
{
	switch( (overlap.m_pointA.m_type << 2) | overlap.m_pointB.m_type )
	{
	case (EPointType_Point<<2)|EPointType_Point:
		{
			pointA = overlap.m_pointA.m_point;
			pointB = overlap.m_pointB.m_point;
			break;
		}
	case (EPointType_Point<<2)|EPointType_Edge:
	case (EPointType_Point<<2)|EPointType_Face:
		{
			pointA = overlap.m_pointA.m_point;
			pointB = overlap.m_pointA.m_point - overlap.m_penetration * overlap.m_axis;
			break;
		}
	case (EPointType_Edge<<2)|EPointType_Point:
	case (EPointType_Face<<2)|EPointType_Point:
		{
			pointA = overlap.m_pointB.m_point + overlap.m_penetration * overlap.m_axis;
			pointB = overlap.m_pointB.m_point;
			break;
		}
	case (EPointType_Edge<<2)|EPointType_Edge:
		{
			// Find the closest point between the two edges and 
			v4 s0 = overlap.m_pointA.m_point + overlap.m_box_radii[overlap.m_pointA.m_dof_info[0]];
			v4 e0 = overlap.m_pointA.m_point - overlap.m_box_radii[overlap.m_pointA.m_dof_info[0]];
			v4 s1 = overlap.m_b2w.pos + overlap.m_tri_verts[overlap.m_pointB.m_dof_info[0]];
			v4 e1 = overlap.m_b2w.pos + overlap.m_tri_verts[overlap.m_pointB.m_dof_info[1]];
			closest_point::LineToLine(s0, e0, s1, e1, pointA, pointB);
			break;
		}
	case (EPointType_Edge<<2)|EPointType_Face:
		{
			// Clip the edge to the triangle
			v4 s = overlap.m_pointA.m_point + overlap.m_box_radii[overlap.m_pointA.m_dof_info[0]];
			v4 e = overlap.m_pointA.m_point - overlap.m_box_radii[overlap.m_pointA.m_dof_info[0]];
			v4 edge[3] =
			{
				overlap.m_tri_verts[1] - overlap.m_tri_verts[0],
				overlap.m_tri_verts[2] - overlap.m_tri_verts[1],
				overlap.m_tri_verts[0] - overlap.m_tri_verts[2]
			};
			Plane tri_plane[3];
			tri_plane[0] = plane::make(overlap.m_b2w.pos + overlap.m_tri_verts[0], Cross(Cross(edge[0], edge[1]), edge[0]));
			tri_plane[1] = plane::make(overlap.m_b2w.pos + overlap.m_tri_verts[1], Cross(Cross(edge[1], edge[2]), edge[1]));
			tri_plane[2] = plane::make(overlap.m_b2w.pos + overlap.m_tri_verts[2], Cross(Cross(edge[2], edge[0]), edge[2]));
			intersect::LineVsPlane(tri_plane[0], s, e, s, e);
			intersect::LineVsPlane(tri_plane[1], s, e, s, e);
			intersect::LineVsPlane(tri_plane[2], s, e, s, e);
			v4 avr = (s + e) / 2.0f;
			pointA = avr;
			pointB = avr - overlap.m_penetration * overlap.m_axis;
			break;
		}
	case (EPointType_Face<<2)|EPointType_Edge:
		{
			// Clip the edge to the planes of the box that are DoF's
			v4 s = overlap.m_b2w.pos + overlap.m_tri_verts[overlap.m_pointB.m_dof_info[0]];
			v4 e = overlap.m_b2w.pos + overlap.m_tri_verts[overlap.m_pointB.m_dof_info[1]];
			for (int i = 0; i != 2; ++i)
			{
				int const& axis = overlap.m_pointA.m_dof_info[i];
				float const& r  = overlap.m_box.m_radius[axis];
				float distA = Dot3(overlap.m_a2w[axis], overlap.m_a2w.pos);
				intersect::LineVsSlab(overlap.m_a2w[axis], distA - r, distA + r, s, e, s, e);
			}
			v4 avr = (s + e) / 2.0f;
			pointA = avr + overlap.m_penetration * overlap.m_axis;
			pointB = avr;
			break;
		}
	case (EPointType_Face<<2)|EPointType_Face:
		{
			v4 avr = v4::Zero();
			float count = 0;

			// Clip the three edges of the triangle to two slabs
			for (int i = 0; i != 3; ++i)
			{
				v4 s = overlap.m_b2w.pos + overlap.m_tri_verts[i];
				v4 e = overlap.m_b2w.pos + overlap.m_tri_verts[(i+1)%3];
				auto axis0 = overlap.m_pointA.m_dof_info[0];
				auto r0    = overlap.m_box.m_radius[axis0];
				auto dist0 = Dot3(overlap.m_a2w[axis0], overlap.m_a2w.pos);
				if (intersect::LineVsSlab(overlap.m_a2w[axis0], dist0 - r0, dist0 + r0, s, e, s, e))
				{
					auto axis1 = overlap.m_pointA.m_dof_info[1];
					auto r1    = overlap.m_box.m_radius[axis1];
					auto dist1 = Dot3(overlap.m_a2w[axis1], overlap.m_a2w.pos);
					if (intersect::LineVsSlab(overlap.m_a2w[axis1], dist1 - r1, dist1 + r1, s, e, s, e))
					{
						avr += s + e;
						count += 2.0f;
					}
				}
			}

			v4 edge[3] =
			{
				overlap.m_tri_verts[1] - overlap.m_tri_verts[0],
				overlap.m_tri_verts[2] - overlap.m_tri_verts[1],
				overlap.m_tri_verts[0] - overlap.m_tri_verts[2]
			};
			Plane tri_plane[3];
			tri_plane[0] = plane::make(overlap.m_b2w.pos + overlap.m_tri_verts[0], Cross(Cross(edge[0], edge[1]), edge[0]));
			tri_plane[1] = plane::make(overlap.m_b2w.pos + overlap.m_tri_verts[1], Cross(Cross(edge[1], edge[2]), edge[1]));
			tri_plane[2] = plane::make(overlap.m_b2w.pos + overlap.m_tri_verts[2], Cross(Cross(edge[2], edge[0]), edge[2]));

			// Clip the four edges of the box against the triangle
			v4 box_pts[4];
			int const& axisA0 = overlap.m_pointA.m_dof_info[0];
			int const& axisA1 = overlap.m_pointA.m_dof_info[1];
			box_pts[0] = overlap.m_pointA.m_point + overlap.m_box_radii[axisA0] + overlap.m_box_radii[axisA1];
			box_pts[1] = overlap.m_pointA.m_point + overlap.m_box_radii[axisA0] - overlap.m_box_radii[axisA1];
			box_pts[2] = overlap.m_pointA.m_point - overlap.m_box_radii[axisA0] - overlap.m_box_radii[axisA1];
			box_pts[3] = overlap.m_pointA.m_point - overlap.m_box_radii[axisA0] + overlap.m_box_radii[axisA1];
			for( int i = 0; i != 4; ++i )
			{
				v4 s = box_pts[i], e = box_pts[(i+1)%4];
				if( intersect::LineVsPlane(tri_plane[0], s, e, s, e) &&
					intersect::LineVsPlane(tri_plane[1], s, e, s, e) &&
					intersect::LineVsPlane(tri_plane[2], s, e, s, e) )
				{
					avr += s + e;
					count += 2.0f;
				}
			}
			PR_ASSERT(PR_DBG_PHYSICS, count, "");
			avr /= count;
			v4 half_pen = overlap.m_axis * overlap.m_penetration * 0.5f;
			pointA = avr + half_pen;
			pointB = avr - half_pen;
		}break;
	}
	PR_EXPAND(PR_DBG_BOX_TRI_COLLISION, StartFile("C:/DeleteMe/collision_boxtricontact.pr_script");)
	PR_EXPAND(PR_DBG_BOX_TRI_COLLISION, ldr::Box("pointA", "FFFF0000", pointA, 0.05f);)
	PR_EXPAND(PR_DBG_BOX_TRI_COLLISION, ldr::Box("pointB", "FF0000FF", pointB, 0.05f);)
	PR_EXPAND(PR_DBG_BOX_TRI_COLLISION, ldr::LineD("norm", "FFFFFF00", pointB, -overlap.m_axis);)
	PR_EXPAND(PR_DBG_BOX_TRI_COLLISION, EndFile();)
}

// Detect collisions between a box and a triangle
void pr::ph::BoxVsTriangle(Shape const& objA, m4x4 const& a2w, Shape const& objB, m4x4 const& b2w, ContactManifold& manifold, CollisionCache*)
{
	ShapeBox      const& box = shape_cast<ShapeBox>     (objA);
	ShapeTriangle const& tri = shape_cast<ShapeTriangle>(objB);
	PR_EXPAND(PR_DBG_BOX_TRI_COLLISION, StartFile("C:/Deleteme/collision_boxtri.pr_script");)
		PR_EXPAND(PR_DBG_BOX_TRI_COLLISION, ldr::Box("box", "FFFF0000", a2w, box.m_radius * 2.0f);)
		PR_EXPAND(PR_DBG_BOX_TRI_COLLISION, ldr::Triangle("tri", "FF0000FF", b2w, tri.m_v.x, tri.m_v.y, tri.m_v.z);)
		PR_EXPAND(PR_DBG_BOX_TRI_COLLISION, EndFile();)

		Overlap data(box, a2w, tri, b2w);
	v4 a_to_b = b2w.pos - a2w.pos;

	// Test the box against the plane of the triangle
	{
		v4 axis = data.m_b2w * tri.m_v.w;
		float sep = Dot3(axis, a_to_b);
		if (sep < 0.0f) { axis = -axis; sep = -sep; }
		PR_EXPAND(PR_DBG_BOX_TRI_COLLISION, StartFile("C:/Deleteme/collision_sepaxis.pr_script");)
			PR_EXPAND(PR_DBG_BOX_TRI_COLLISION, ldr::LineD("sep_axis", "FFFFFF00", b2w.pos, axis);)
			PR_EXPAND(PR_DBG_BOX_TRI_COLLISION, EndFile();)

			Point pointA(data.m_a2w.pos);
		float proj = ProjectBox(data.m_box_radii, axis, pointA);
		float overlap = -sep + proj;
		if (overlap < 0.0f)
			return; // No collision

		data.m_penetration = overlap;
		data.m_axis = axis;
		data.m_pointA = pointA;
		data.m_pointB.set(b2w.pos, EPointType_Face, 0, 0);
	}

	// Convert the triangle into box space
	m4x4 t2b = InvertAffine(a2w) * b2w; // Tri to Box space

	m4x4 tri_bs = t2b * tri.m_v;
	PR_EXPAND(PR_DBG_BOX_TRI_COLLISION, StartFile("C:/Deleteme/collision_boxtri2.pr_script"));
	PR_EXPAND(PR_DBG_BOX_TRI_COLLISION, ldr::Box("box", "FFFF0000", m4x4::Identity(), box.m_radius * 2.0f));
	PR_EXPAND(PR_DBG_BOX_TRI_COLLISION, ldr::Triangle("tri", "FF0000FF", tri_bs.x, tri_bs.y, tri_bs.z));
	PR_EXPAND(PR_DBG_BOX_TRI_COLLISION, EndFile();)
	tri_bs = Transpose3x3(tri_bs);

	// Test against the faces of the box
	for (int i = 0; i != 3; ++i)
	{
		PR_EXPAND(PR_DBG_BOX_TRI_COLLISION, StartFile("C:/Deleteme/collision_sepaxis.pr_script"));
		PR_EXPAND(PR_DBG_BOX_TRI_COLLISION, ldr::LineD("sep_axis", "FFFFFF00", a2w.pos, Sign(t2b.pos[i]) * a2w[i]));
		PR_EXPAND(PR_DBG_BOX_TRI_COLLISION, EndFile());

		int tri_vert_idx;
		float sep = t2b.pos[i];
		if (sep > 0.0f) { tri_vert_idx = MinElementIndex(tri_bs[i].xyz); }
		else { tri_vert_idx = MaxElementIndex(tri_bs[i].xyz); }
		float overlap = -Abs(sep) + box.m_radius[i] + Abs(tri_bs[i][tri_vert_idx]);
		if (overlap < 0.0f)
			return; // No collision

		if (overlap < data.m_penetration)
		{
			float sign = Sign(t2b.pos[i]);
			data.m_penetration = overlap;
			data.m_axis = sign * a2w[i];
			data.m_pointA.set(a2w.pos + sign * data.m_box_radii[i], EPointType_Face, (i + 1) % 3, (i + 2) % 3);
			data.m_pointB.set(b2w.pos + b2w * tri.m_v[tri_vert_idx], EPointType_Point, 0, 0);
		}
	}

	// Test against the cross products of the triangle edges and box axes
	// The penetration is the 'other' vertex of the triangle dot'ed with the edge cross product
	for (int j = 0; j != 3; ++j)
	{
		v4 edge = data.m_tri_verts[(j + 1) % 3] - data.m_tri_verts[j];
		for (int i = 0; i != 3; ++i)
		{
			v4 axis = Cross(a2w[i], edge);
			float len = Length(axis);
			if (len > maths::tiny<float>)
			{
				axis /= len;
				float sep = Dot3(axis, a_to_b);
				if (sep < 0.0f) { axis = -axis; sep = -sep; }
				PR_EXPAND(PR_DBG_BOX_TRI_COLLISION, StartFile("C:/Deleteme/collision_sepaxis.pr_script");)
					PR_EXPAND(PR_DBG_BOX_TRI_COLLISION, ldr::LineD("sep_axis", "FFFFFF00", a2w.pos, axis);)
					PR_EXPAND(PR_DBG_BOX_TRI_COLLISION, EndFile();)

					Point pointA(data.m_a2w.pos);
				Point pointB(data.m_b2w.pos);
				float projA = ProjectBox(data.m_box_radii, axis, pointA);
				float projB = ProjectTri(data.m_tri_verts, -axis, pointB);
				float overlap = -sep + projA + projB;
				if (overlap < 0.0f)
					return; // No collision

				if (overlap < data.m_penetration)
				{
					data.m_penetration = overlap;
					data.m_axis = axis;
					data.m_pointA = pointA;
					data.m_pointB = pointB;
				}
			}
		}
	}

	// If there was a collision fill in the collision manifold
	PR_ASSERT(PR_DBG_PHYSICS, data.m_penetration >= 0.0f, "Collision with no penetration?");
	Contact contact;
	contact.m_normal = -data.m_axis;
	contact.m_depth = data.m_penetration;
	contact.m_material_idA = box.m_base.m_material_id;
	contact.m_material_idB = tri.m_base.m_material_id;
	::GetPointOfContactBoxVsTri(contact.m_pointA, contact.m_pointB, data);
	manifold.Add(contact);
}