//*********************************************
// Physics engine
//  Copyright (c) Rylogic Ltd 2006
//*********************************************
// This file contains and implementation of box vs cylinder (not capsule)
// collision detection. As fair as I know this is an original algorithm.

#include "physics/utility/stdafx.h"
#include "pr/physics/collision/collider.h"
#include "pr/physics/collision/contactmanifold.h"
#include "pr/physics/rigidbody/rigidbody.h"
#include "pr/physics/shape/shape.h"
#include "pr/physics/shape/shapebox.h"
#include "pr/physics/shape/shapecylinder.h"
#include "physics/utility/debug.h"
#include "physics/utility/profile.h"

using namespace pr;
using namespace pr::ph;
using namespace pr::geometry;

namespace pr
{
	namespace ph
	{
		namespace box_vs_cyl
		{
			PR_EXPAND(PR_DBG_BOX_CYL_COLLISION, std::string str;);

			struct Overlap
			{
				Overlap(Overlap const&);				// No copying
				Overlap& operator =(Overlap const&);	// No copying
				Overlap(ShapeBox const& box, m4x4 const& a2w, ShapeCylinder const& cyl, m4x4 const& b2w)
				:m_box(box)
				,m_a2w(a2w)
				,m_cyl(cyl)
				,m_b2w(b2w)
				,m_box_pos(a2w.pos)
				,m_cyl_pos(b2w.pos)
				,m_cyl_axis(b2w.y)
				,m_diff(b2w.pos - a2w.pos)
				,m_penetration(limits<float>::max())
				{}

				ShapeBox const&		 m_box;
				m4x4 const&			 m_a2w;
				ShapeCylinder const& m_cyl;
				m4x4 const&			 m_b2w;
				v4			 m_box_pos;
				v4			 m_cyl_pos;
				v4			 m_cyl_axis;
				v4					 m_diff;
				v4					 m_axis;		// The collision normal
				v4					 m_pointA;		// The point of contact on object A (box)
				v4					 m_pointB;		// The point of contact on object B (cyl)
				float				 m_penetration;	// Depth of penetration. no contact if <= 0
			};

			// Check penetration of cylinder into a face of the box
			bool TestBoxAxes(Overlap& data, uint32_t i)
			{
				v4 sep_axis = data.m_a2w[i];
				float depth = Dot3(sep_axis, data.m_diff);
				if (depth < 0.0f)	sep_axis = -sep_axis;
				else				depth = -depth;
				float ratio = Dot3(sep_axis, data.m_cyl_axis);
				float cos_angle = Clamp(Abs(ratio), 0.0f, 1.0f);
				float sin_angle = Sqrt(1.0f - Sqr(cos_angle));
				depth += data.m_box.m_radius[i] + data.m_cyl.m_height * cos_angle + data.m_cyl.m_radius * sin_angle;

				// Separate on this axis
				if (depth < 0.0f)
					return false;

				// Can give up if the overlap is already greater than the current minimum
				if (depth >= data.m_penetration)
					return true;

				// Find the nearest point on the cylinder
				v4 cyl_point = data.m_cyl_pos - (Sign(ratio) * data.m_cyl.m_height) * data.m_cyl_axis;
				if (1.0f - cos_angle > maths::tiny<float>)
				{
					v4 radius = Normalise(Cross(data.m_cyl_axis, Cross(data.m_cyl_axis, sep_axis)));
					cyl_point += data.m_cyl.m_radius * radius;
					data.m_penetration = depth;
					data.m_axis = -sep_axis;
					data.m_pointA = cyl_point + depth * sep_axis;
					data.m_pointB = cyl_point;
				}
				// Otherwise find a common point on the contacting faces
				else
				{
					uint32_t j = (i + 1) % 3;
					uint32_t k = (i + 2) % 3;
					v4 box_point = data.m_box_pos + sep_axis * data.m_box.m_radius[i];

					// Check the centre point of the box for being within 
					// end of the cylinder and visa versa
					v4 diff = box_point - cyl_point;
					diff -= Dot3(diff, data.m_cyl_axis) * data.m_cyl_axis;
					if (LengthSq(diff) < Sqr(data.m_cyl.m_radius))
					{
						data.m_penetration = depth;
						data.m_axis = -sep_axis;
						data.m_pointA = box_point;
						data.m_pointB = box_point - depth * sep_axis;
					}
					else
					{
						float dj = Dot3(diff, data.m_a2w[j]);
						float dk = Dot3(diff, data.m_a2w[k]);
						if (Abs(dj) < data.m_box.m_radius[j] && Abs(dk) < data.m_box.m_radius[k])
						{
							data.m_penetration = depth;
							data.m_axis = -sep_axis;
							data.m_pointA = cyl_point + depth * sep_axis;
							data.m_pointB = cyl_point;
						}
						else
						{
							// In this case, just use the cylinder rim
							v4 pt = cyl_point + diff * data.m_cyl.m_radius;
							data.m_penetration = depth;
							data.m_axis = -sep_axis;
							data.m_pointA = pt + depth * sep_axis;
							data.m_pointB = pt;
						}
					}
				}

				PR_EXPAND(PR_DBG_BOX_CYL_COLLISION, StartFile("C:/DeleteMe/collision_points.pr_script"));
				PR_EXPAND(PR_DBG_BOX_CYL_COLLISION, ldr::Box("pointA", "FFFF0000", data.m_pointA, 0.02f));
				PR_EXPAND(PR_DBG_BOX_CYL_COLLISION, ldr::Box("pointB", "FF0000FF", data.m_pointB, 0.02f));
				PR_EXPAND(PR_DBG_BOX_CYL_COLLISION, EndFile());
				return true;
			}

			// Test 'box' against the main axis of the cylinder
			bool TestCylAxis(Overlap& data)
			{
				v4 sep_axis = data.m_cyl_axis;
				float depth = Dot3(sep_axis, data.m_diff);
				if (depth < 0.0f)	sep_axis = -sep_axis;
				else				depth = -depth;
				depth += data.m_cyl.m_height;
				PR_EXPAND(PR_DBG_BOX_CYL_COLLISION, StartFile("C:/DeleteMe/collision_sepaxis.pr_script"));
				PR_EXPAND(PR_DBG_BOX_CYL_COLLISION, ldr::LineD("sep_axis", "FFFFFF00", data.m_cyl_pos, sep_axis));
				PR_EXPAND(PR_DBG_BOX_CYL_COLLISION, EndFile());

				// Can give up if the overlap is already greater than the current minimum
				if (depth >= data.m_penetration)
					return true;

				v4 point = data.m_box_pos;
				for (int i = 0; i != 3; ++i)
				{
					float d = Dot3(data.m_a2w[i], sep_axis);
					float r = data.m_box.m_radius[i];
					if (d < -maths::tiny<float>)
					{
						depth -= d * r;
						point -= r * data.m_a2w[i];
					}
					else if (d > maths::tiny<float>)
					{
						depth += d * r;
						point += r * data.m_a2w[i];
					}
				}

				// Separate on this axis
				if (depth < 0.0f)
					return false;

				// Can give up if the overlap is already greater than the current minimum
				if (depth >= data.m_penetration)
					return true;

				data.m_penetration = depth;
				data.m_axis = -sep_axis;
				data.m_pointA = point;
				data.m_pointB = point - depth * sep_axis;

				PR_EXPAND(PR_DBG_BOX_CYL_COLLISION, StartFile("C:/DeleteMe/collision_points.pr_script"));
				PR_EXPAND(PR_DBG_BOX_CYL_COLLISION, ldr::Box("pointA", "FFFF0000", data.m_pointA, 0.02f));
				PR_EXPAND(PR_DBG_BOX_CYL_COLLISION, ldr::Box("pointB", "FF0000FF", data.m_pointB, 0.02f));
				PR_EXPAND(PR_DBG_BOX_CYL_COLLISION, EndFile());
				return true;
			}

			// Test the edges of the box against the wall of the cylinder
			bool TestCylWall(Overlap& data, uint32_t i)
			{
				v4 sep_axis = Cross(data.m_a2w[i], data.m_cyl_axis);
				if (FEql(sep_axis, v4::Zero())) sep_axis = data.m_diff - Dot3(data.m_diff, data.m_cyl_axis) * data.m_cyl_axis;
				if (FEql(sep_axis, v4::Zero())) return true;
				sep_axis = Normalise(sep_axis);

				uint32_t j = (i+1)%3;
				uint32_t k = (i+2)%3;

				float depth = Dot3(sep_axis, data.m_diff);
				if( depth < 0.0f )	sep_axis = -sep_axis;
				else				depth = -depth;
				depth += data.m_cyl.m_radius;
				PR_EXPAND(PR_DBG_BOX_CYL_COLLISION, StartFile("C:/DeleteMe/collision_sepaxis.pr_script"));
				PR_EXPAND(PR_DBG_BOX_CYL_COLLISION, ldr::LineD("sep_axis", "FFFFFF00", data.m_box_pos, sep_axis));
				PR_EXPAND(PR_DBG_BOX_CYL_COLLISION, EndFile());

				float ratio_j = Dot3(sep_axis, data.m_a2w[j]);
				float ratio_k = Dot3(sep_axis, data.m_a2w[k]);
				depth += Abs(ratio_j) * data.m_box.m_radius[j] + Abs(ratio_k) * data.m_box.m_radius[k];

				// Separate on this axis
				if( depth < 0.0f )
					return false; 

				// Can give up if the overlap is already greater than the current minimum
				if( depth >= data.m_penetration )
					return true;

				// Find a point on the nearest box edge
				v4 box_point = data.m_box_pos;
				box_point += Sign(ratio_j) * data.m_box.m_radius[j] * data.m_a2w[j];
				box_point += Sign(ratio_k) * data.m_box.m_radius[k] * data.m_a2w[k];
				PR_EXPAND(PR_DBG_BOX_CYL_COLLISION, StartFile("C:/DeleteMe/collision_sepaxis.pr_script"));
				PR_EXPAND(PR_DBG_BOX_CYL_COLLISION, ldr::LineD("sep_axis", "FFFFFF00", box_point, sep_axis));
				PR_EXPAND(PR_DBG_BOX_CYL_COLLISION, EndFile());

				// Find the nearest points between the nearest box edge and the main axis of the cylinder
				float t0, t1;
				v4 box_r = data.m_box.m_radius[i] * data.m_a2w[i];
				v4 cyl_r = data.m_cyl.m_height    * data.m_cyl_axis;
				v4 b0 = box_point - box_r;
				v4 b1 = box_point + box_r;
				v4 c0 = data.m_cyl_pos - cyl_r;
				v4 c1 = data.m_cyl_pos + cyl_r;
				closest_point::LineToLine(b0, b1, c0, c1, t0, t1);

				// Use the vector between the nearest points as the separating axis (but retain the direction)
				v4 b = b0 + t0 * (b1 - b0);
				v4 c = c0 + t1 * (c1 - c0);
				v4 saxis = c - b;
				if( Dot3(saxis, sep_axis) < 0.0f ) saxis = -saxis;
				if( FEql(saxis, v4::Zero()) ) saxis = sep_axis;
				sep_axis = Normalise(saxis);

				data.m_penetration	= depth;
				data.m_axis			= -sep_axis;
				data.m_pointA		= b;
				data.m_pointB		= b - depth * sep_axis;

				PR_EXPAND(PR_DBG_BOX_CYL_COLLISION, StartFile("C:/DeleteMe/collision_points.pr_script"));
				PR_EXPAND(PR_DBG_BOX_CYL_COLLISION, ldr::Box("pointA", "FFFF0000", data.m_pointA, 0.02f));
				PR_EXPAND(PR_DBG_BOX_CYL_COLLISION, ldr::Box("pointB", "FF0000FF", data.m_pointB, 0.02f));
				PR_EXPAND(PR_DBG_BOX_CYL_COLLISION, EndFile());
				return true;
			}

			// Test the edges of the box against the rims of the cylinder.
			bool TestCylRim(Overlap& data, uint32_t i)
			{
				uint32_t j = (i+1)%3;
				uint32_t k = (i+2)%3;

				v4 sep_axis = data.m_diff;
				v4 cyl_axis = data.m_cyl_axis;
				if( Dot3(cyl_axis, data.m_diff) < 0.0f )
					cyl_axis = -cyl_axis;

				float ratio_j = Dot3(sep_axis, data.m_a2w[j]);
				float ratio_k = Dot3(sep_axis, data.m_a2w[k]);

				// Find the point in the centre of the box edge that we want to test against the rim of the cylinder
				v4 box_point = data.m_box_pos;
				box_point += Sign(ratio_j) * data.m_box.m_radius[j] * data.m_a2w[j];
				box_point += Sign(ratio_k) * data.m_box.m_radius[k] * data.m_a2w[k];
				PR_EXPAND(PR_DBG_BOX_CYL_COLLISION, StartFile("C:/DeleteMe/collision_sepaxis.pr_script"));
				PR_EXPAND(PR_DBG_BOX_CYL_COLLISION, ldr::LineD("sep_axis", "FFFFFF00", box_point, sep_axis));
				PR_EXPAND(PR_DBG_BOX_CYL_COLLISION, EndFile());

				// Find the centre of the nearest end of the cylinder
				v4 cyl_point = data.m_cyl_pos - data.m_cyl.m_height * cyl_axis;
				
				// Project the nearest box edge into the plane of the nearest end of the cylinder.
				v4 r  = data.m_box.m_radius[i] * data.m_a2w[i];
				v4 p0 = box_point - r;	p0 -= Dot3(cyl_axis, p0 - cyl_point) * cyl_axis;
				v4 p1 = box_point + r;	p1 -= Dot3(cyl_axis, p1 - cyl_point) * cyl_axis;
				PR_EXPAND(PR_DBG_BOX_CYL_COLLISION, StartFile("C:/DeleteMe/collision_line.pr_script"));
				PR_EXPAND(PR_DBG_BOX_CYL_COLLISION, ldr::Line("p0-p1", "FF00FFFF", p0, p1));
				PR_EXPAND(PR_DBG_BOX_CYL_COLLISION, EndFile());

				// Find the intercepts on the cylinder rim with the infinite line passing through p0 and p1
				v4 d = p1 - p0;
				float d_len_sq = LengthSq(d);
				
				// If the box edge is parallel to the main axis of the cylinder then this edge
				// cannot penetrate the rim of the cylinder.
				if (FEql(d_len_sq, 0.f))
					return true;

				v4 nearest = p0 - (Dot3(d, p0 - cyl_point) / d_len_sq) * d;
				float nearest_dist_sq = LengthSq(nearest - cyl_point);
				float radius_sq = Sqr(data.m_cyl.m_radius);

				// If the nearest box edge does not clip the cylinder then this cannot be the separating axis
				if( nearest_dist_sq > radius_sq )
					return true;

				// Get the vector from the closest point to the intersection with the cylinder rim
				v4 x = Sqrt((radius_sq - nearest_dist_sq) / d_len_sq) * d;

				// Choose the point that is closest to the 'deepest' penetrating end of the box edge
				v4 point = Sign(Dot3(cyl_axis, data.m_a2w[i])) * x + nearest;
				PR_EXPAND(PR_DBG_BOX_CYL_COLLISION, AppendFile("C:/DeleteMe/collision_line.pr_script"));
				PR_EXPAND(PR_DBG_BOX_CYL_COLLISION, ldr::Box("p0-p1", "FF00FFFF", point, 0.02f));
				PR_EXPAND(PR_DBG_BOX_CYL_COLLISION, EndFile());

				// Refine the separating axis to the vector that is perpendicular to both the nearest
				// box edge and the tangent to the cylinder at 'point'
				sep_axis = Cross(data.m_a2w[i], Cross(point - data.m_cyl_pos, cyl_axis));
				if( FEql(sep_axis,pr::v4::Zero()) )	return true;
				sep_axis = Normalise(sep_axis);
				if( Dot3(sep_axis, data.m_cyl_pos - point) < 0.0f ) sep_axis = -sep_axis; // 'sep_axis' pointing from A to B
				PR_EXPAND(PR_DBG_BOX_CYL_COLLISION, StartFile("C:/DeleteMe/collision_sepaxis.pr_script"));
				PR_EXPAND(PR_DBG_BOX_CYL_COLLISION, ldr::LineD("sep_axis", "FFFFFF00", point, sep_axis));
				PR_EXPAND(PR_DBG_BOX_CYL_COLLISION, EndFile();)

				ratio_j = Dot3(sep_axis, data.m_a2w[j]);
				ratio_k = Dot3(sep_axis, data.m_a2w[k]);

				// Calculate the new depth of penetration
				float depth = Dot3(sep_axis, data.m_box_pos - point) + Abs(ratio_j) * data.m_box.m_radius[j] + Abs(ratio_k) * data.m_box.m_radius[k];

				// Separate on this axis
				if( depth < 0.0f )
					return false; 

				// Can give up if the overlap is already greater than the current minimum
				if( depth >= data.m_penetration )
					return true;

				data.m_penetration	= depth;
				data.m_axis			= -sep_axis;
				data.m_pointA		= point + depth * sep_axis;
				data.m_pointB		= point;

				PR_EXPAND(PR_DBG_BOX_CYL_COLLISION, StartFile("C:/DeleteMe/collision_points.pr_script"));
				PR_EXPAND(PR_DBG_BOX_CYL_COLLISION, ldr::Box("pointA", "FFFF0000", data.m_pointA, 0.02f));
				PR_EXPAND(PR_DBG_BOX_CYL_COLLISION, ldr::Box("pointB", "FF0000FF", data.m_pointB, 0.02f));
				PR_EXPAND(PR_DBG_BOX_CYL_COLLISION, EndFile());
				return true;
			}

			// Detect collisions between a box and a cylinder. Results returned in 'data'
			bool Collide(Overlap& data)
			{
				// Test the principle axes of the box as separating axes
				for( uint32_t i = 0; i != 3; ++i )
				{
					if( !TestBoxAxes(data, i) )
						return false;
				}

				// Test the main axis of the cylinder
				if( !TestCylAxis(data) )
					return false;

				// Test the cross products of the axes of the box with the main axis of the cylinder
				for( uint32_t i = 0; i != 3; ++i )
				{
					if( !TestCylWall(data, i) )
						return false;
				}

				// Test the cross products of the axes of the box with the tangents to the rim of the cylinder
				for( uint32_t i = 0; i != 3; ++i )
				{
					if( !TestCylRim(data, i) )
						return false;
				}
				return true;
			}
		}
	}
}

// Returns true if 'shapeA' and 'shapeB' are in collision
bool pr::ph::Collide(ShapeBox const& shapeA, m4x4 const& a2w, ShapeCylinder const& shapeB, m4x4 const& b2w)
{
	box_vs_cyl::Overlap min_overlap(shapeA, a2w, shapeB, b2w);
	return box_vs_cyl::Collide(min_overlap);
}

// Returns true if 'shapeA' and 'shapeB' are in collision with details in 'manifold'
bool pr::ph::Collide(ShapeBox const& shapeA, m4x4 const& a2w, ShapeCylinder const& shapeB, m4x4 const& b2w, ContactManifold& manifold, CollisionCache*)
{
	box_vs_cyl::Overlap min_overlap(shapeA, a2w, shapeB, b2w);
	if( !box_vs_cyl::Collide(min_overlap) ) return false;

	// If there was a collision fill in the collision manifold
	PR_ASSERT(PR_DBG_PHYSICS, min_overlap.m_penetration >= 0.0f, "Collision with no penetration?");
	Contact contact;
	contact.m_normal		= min_overlap.m_axis;
	contact.m_depth			= min_overlap.m_penetration;
	contact.m_material_idA	= shapeA.m_base.m_material_id;
	contact.m_material_idB	= shapeB.m_base.m_material_id;
	contact.m_pointA		= min_overlap.m_pointA;
	contact.m_pointB		= min_overlap.m_pointB;
	manifold.Add(contact);
	return true;
}

// Detect collisions between box and cylinder shapes
void pr::ph::BoxVsCylinder(Shape const& shapeA, m4x4 const& a2w, Shape const& shapeB, m4x4 const& b2w, ContactManifold& manifold, CollisionCache* cache)
{
	PR_DECLARE_PROFILE(PR_PROFILE_CYL_COLLISION, phBoxVsCyl);
	PR_PROFILE_SCOPE(PR_PROFILE_CYL_COLLISION, phBoxVsCyl);
	Collide(shape_cast<ShapeBox>(shapeA), a2w, shape_cast<ShapeCylinder>(shapeB), b2w, manifold, cache);
}
