//*********************************************
// Physics engine
//  Copyright (c) Rylogic Ltd 2006
//*********************************************

#include "physics/utility/stdafx.h"
#include "pr/physics/collision/collider.h"
#include "pr/physics/collision/contactmanifold.h"
#include "pr/physics/rigidbody/rigidbody.h"
#include "pr/physics/shape/shape.h"
#include "pr/physics/shape/shapebox.h"
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
		namespace box_vs_box
		{
			struct Overlap
			{
				Overlap(Overlap const&);			 // No copying
				Overlap& operator =(Overlap const&); // No copying
				Overlap(ShapeBox const& shapeA, m4x4 const& a2w, ShapeBox const& shapeB, m4x4 const& b2w)
				:m_shapeA(shapeA)
				,m_a2w(a2w)
				,m_shapeB(shapeB)
				,m_b2w(b2w)
				,m_penetration(limits<float>::max())
				{
					for( int i = 0; i != 3; ++ i )
					{
						m_boxA[i] = m_a2w[i] * m_shapeA.m_radius[i];
						m_boxB[i] = m_b2w[i] * m_shapeB.m_radius[i];
					}
				}

				ShapeBox const&	m_shapeA;
				m4x4     const&	m_a2w;
				ShapeBox const&	m_shapeB;
				m4x4     const&	m_b2w;
				v4				m_axis;			// Always from m_pointA to m_pointB
				Point			m_pointA;		// The point of contact on object A
				Point			m_pointB;		// The point of contact on object B
				m3x4			m_boxA;			// Radius vectors for box A
				m3x4			m_boxB;			// Radius vectors for box B
				float			m_penetration;	// The depth of penetration. No contact if <= 0.0f
			};

			// This function takes the points A and B in 'overlap' and adjusts them to
			// the mostly likely point of contact based on the degrees of freedom in each point.
			// 'pointA' and 'pointB' are in world space but relative to shapeA and shapeB
			void GetPointOfContactBoxVsBox(v4& pointA, v4& pointB, Overlap const& overlap)
			{
				switch( (overlap.m_pointA.m_type << 2) | overlap.m_pointB.m_type )
				{
				case (EPointType_Point<<2)|EPointType_Point:
					{
						pointA = overlap.m_pointA.m_point;
						pointB = overlap.m_pointB.m_point;
					}break;
				case (EPointType_Point<<2)|EPointType_Edge:
				case (EPointType_Point<<2)|EPointType_Face:
					{
						pointA = overlap.m_pointA.m_point;
						pointB = overlap.m_pointA.m_point - overlap.m_penetration * overlap.m_axis;
					}break;
				case (EPointType_Edge<<2)|EPointType_Point:
				case (EPointType_Face<<2)|EPointType_Point:
					{
						pointA = overlap.m_pointB.m_point + overlap.m_penetration * overlap.m_axis;
						pointB = overlap.m_pointB.m_point;
					}break;
				case (EPointType_Edge<<2)|EPointType_Edge:
					{
						// Find the closest point between the two edges and 
						v4 s0 = overlap.m_pointA.m_point + overlap.m_boxA[overlap.m_pointA.m_dof_info[0]];
						v4 e0 = overlap.m_pointA.m_point - overlap.m_boxA[overlap.m_pointA.m_dof_info[0]];
						v4 s1 = overlap.m_pointB.m_point + overlap.m_boxB[overlap.m_pointB.m_dof_info[0]];
						v4 e1 = overlap.m_pointB.m_point - overlap.m_boxB[overlap.m_pointB.m_dof_info[0]];
						closest_point::LineToLine(s0, e0, s1, e1, pointA, pointB);
					}break;
				case (EPointType_Edge<<2)|EPointType_Face:
					{
						// Clip the edge to the planes of boxB that are dof's
						v4 s = overlap.m_pointA.m_point + overlap.m_boxA[overlap.m_pointA.m_dof_info[0]];
						v4 e = overlap.m_pointA.m_point - overlap.m_boxA[overlap.m_pointA.m_dof_info[0]];
						for( int i = 0; i != 2; ++i )
						{
							int const& axis = overlap.m_pointB.m_dof_info[i];
							float const& r  = overlap.m_shapeB.m_radius[axis];
							float distB = Dot3(overlap.m_b2w[axis], overlap.m_b2w.pos);
							intersect::LineVsSlab(overlap.m_b2w[axis], distB - r, distB + r, s, e, s, e);
						}
						v4 avr = (s + e) / 2.0f;
						pointA = avr;
						pointB = avr - overlap.m_penetration * overlap.m_axis;
					}break;
				case (EPointType_Face<<2)|EPointType_Edge:
					{
						// Clip the edge to the planes of boxA that are dof's
						v4 s = overlap.m_pointB.m_point + overlap.m_boxB[overlap.m_pointB.m_dof_info[0]];
						v4 e = overlap.m_pointB.m_point - overlap.m_boxB[overlap.m_pointB.m_dof_info[0]];
						for( int i = 0; i != 2; ++i )
						{
							int const& axis = overlap.m_pointA.m_dof_info[i];
							float const& r  = overlap.m_shapeA.m_radius[axis];
							float distA = Dot3(overlap.m_a2w[axis], overlap.m_a2w.pos);
							intersect::LineVsSlab(overlap.m_a2w[axis], distA - r, distA + r, s, e, s, e);
						}
						v4 avr = (s + e) / 2.0f;
						pointA = avr + overlap.m_penetration * overlap.m_axis;
						pointB = avr;
					}break;	
				case (EPointType_Face<<2)|EPointType_Face:
					{
						int const& axisA0 = overlap.m_pointA.m_dof_info[0];
						int const& axisA1 = overlap.m_pointA.m_dof_info[1];
						int const& axisB0 = overlap.m_pointB.m_dof_info[0];
						int const& axisB1 = overlap.m_pointB.m_dof_info[1];

						v4 pts[2][4];
						pts[0][0] = overlap.m_pointA.m_point + overlap.m_boxA[axisA0] + overlap.m_boxA[axisA1];
						pts[0][1] = overlap.m_pointA.m_point + overlap.m_boxA[axisA0] - overlap.m_boxA[axisA1];
						pts[0][2] = overlap.m_pointA.m_point - overlap.m_boxA[axisA0] - overlap.m_boxA[axisA1];
						pts[0][3] = overlap.m_pointA.m_point - overlap.m_boxA[axisA0] + overlap.m_boxA[axisA1];
						pts[1][0] = overlap.m_pointB.m_point + overlap.m_boxB[axisB0] + overlap.m_boxB[axisB1];
						pts[1][1] = overlap.m_pointB.m_point + overlap.m_boxB[axisB0] - overlap.m_boxB[axisB1];
						pts[1][2] = overlap.m_pointB.m_point - overlap.m_boxB[axisB0] - overlap.m_boxB[axisB1];
						pts[1][3] = overlap.m_pointB.m_point - overlap.m_boxB[axisB0] + overlap.m_boxB[axisB1];

						// Clip each box against the other
						v4 avr = v4::Zero();
						float count = 0;
						for (int j = 0; j != 2; ++j)
						{
							auto& b2w    = (j == 0) ? overlap.m_b2w : overlap.m_a2w;
							auto& ptB    = (j == 0) ? overlap.m_pointB : overlap.m_pointA;
							auto& shapeB = (j == 0) ? overlap.m_shapeB : overlap.m_shapeA;
							for (int i = 0; i != 4; ++i)
							{
								// Implemented in terms of edges of A against boxB but symmetric
								// so when 'j' == 1 read A as B and B as A
								v4 s = pts[j][i], e = pts[j][(i + 1) % 4];
								auto& axis0  = ptB.m_dof_info[0];
								auto& rad0   = shapeB.m_radius[axis0];
								float distB0 = Dot3(b2w[axis0], b2w.pos);
								if (intersect::LineVsSlab(b2w[axis0], distB0 - rad0, distB0 + rad0, s, e, s, e))
								{
									auto& axis1  = ptB.m_dof_info[1];
									auto& rad1   = shapeB.m_radius[axis1];
									float distB1 = Dot3(b2w[axis1], b2w.pos);
									if (intersect::LineVsSlab(b2w[axis1], distB1 - rad1, distB1 + rad1, s, e, s, e))
									{
										avr += s + e;
										count += 2.0f;
									}
								}
							}
						}
						PR_ASSERT(PR_DBG_PHYSICS, count, "");
						avr /= count;
						v4 half_pen = overlap.m_axis * overlap.m_penetration * 0.5f;
						pointA = avr + half_pen;
						pointB = avr - half_pen;
					}break;
				}
				PR_EXPAND(PR_DBG_BOX_COLLISION, StartFile("C:/DeleteMe/collision_boxboxcontact.pr_script");)
				PR_EXPAND(PR_DBG_BOX_COLLISION, ldr::Box("pointA", "FFFF0000", pointA, 0.05f);)
				PR_EXPAND(PR_DBG_BOX_COLLISION, ldr::Box("pointB", "FF0000FF", pointB, 0.05f);)
				PR_EXPAND(PR_DBG_BOX_COLLISION, ldr::LineD("norm", "FFFFFF00", pointB, -overlap.m_axis);)
				PR_EXPAND(PR_DBG_BOX_COLLISION, EndFile();)
			}

			// Detect collisions between boxes. Results returned in 'data'
			bool Collide(Overlap& data)
			{
				v4 a_to_b = data.m_b2w.pos - data.m_a2w.pos;

				// Find the extent on each of the separating axes
				// Note: It's more efficient to test the axes in this order rather than combining the for loops
				for( int i = 0; i != 3; ++i )
				{
					Point pointB(data.m_b2w.pos);
					v4 axis = data.m_a2w[i];
					float sep = Dot3(axis, a_to_b);
					if( sep < 0.0f ) { axis = -axis; sep = -sep; }	// Ensure axis points from A to B
					float proj = ProjectBox(data.m_boxB, -axis, pointB);
					float overlap = -sep + data.m_shapeA.m_radius[i] + proj;
					if( overlap < 0.0f )
						return false; // No collision
					if( overlap < data.m_penetration )
					{
						data.m_penetration	= overlap;
						data.m_axis			= axis;
						data.m_pointA		.set(data.m_a2w.pos + data.m_shapeA.m_radius[i]*axis, EPointType_Face, (i+1)%3, (i+2)%3);
						data.m_pointB		= pointB;
					}
				}
				for( int i = 0; i != 3; ++i )
				{
					Point pointA(data.m_a2w.pos);
					v4 axis = data.m_b2w[i];
					float sep = Dot3(axis, a_to_b);
					if( sep < 0.0f ) { axis = -axis; sep = -sep; }	// Ensure axis points from A to B
					float proj = ProjectBox(data.m_boxA, axis, pointA);
					float overlap = -sep + data.m_shapeB.m_radius[i] + proj;
					if( overlap < 0.0f )
						return false; // No collision
					if( overlap < data.m_penetration )
					{
						data.m_penetration	= overlap;
						data.m_axis			= axis;
						data.m_pointA		= pointA;
						data.m_pointB		.set(data.m_b2w.pos - data.m_shapeB.m_radius[i]*axis, EPointType_Face, (i+1)%3, (i+2)%3);
					}
				}
				for( int i = 0; i != 3; ++i )
				{
					for( int j = 0; j != 3; ++j )
					{
						v4 axis = Cross(data.m_a2w[i], data.m_b2w[j]);
						if( !FEql(axis, pr::v4::Zero()) )
						{
							axis = Normalise(axis);
							Point pointA(data.m_a2w.pos);
							Point pointB(data.m_b2w.pos);
							float sep = Dot3(axis, a_to_b);
							if( sep < 0.0f ) { axis = -axis; sep = -sep; }	// Ensure axis points from A to B
							float projA = ProjectBox(data.m_boxA,  axis, pointA);
							float projB = ProjectBox(data.m_boxB, -axis, pointB);
							float overlap = -sep + projA + projB;
							if( overlap < 0.0f )
								return false; // No collision
							if( overlap < data.m_penetration )
							{
								data.m_penetration	= overlap;
								data.m_axis			= axis;
								data.m_pointA		= pointA;
								data.m_pointB		= pointB;
							}
						}
					}
				}
				return true;
			}

		}//namespace box_vs_box
	}//namespace ph
}//namespace pr

// Returns true if 'shapeA' and 'shapeB' are in collision
bool pr::ph::Collide(ShapeBox const& shapeA, m4x4 const& a2w, ShapeBox const& shapeB, m4x4 const& b2w)
{
	box_vs_box::Overlap min_overlap(shapeA, a2w, shapeB, b2w);
	return box_vs_box::Collide(min_overlap);
}

// Returns true if 'shapeA' and 'shapeB' are in collision with details in 'manifold'
bool pr::ph::Collide(ShapeBox const& shapeA, m4x4 const& a2w, ShapeBox const& shapeB, m4x4 const& b2w, ContactManifold& manifold, CollisionCache*)
{
	box_vs_box::Overlap min_overlap(shapeA, a2w, shapeB, b2w);
	if( !box_vs_box::Collide(min_overlap) ) return false;

	// If there was a collision fill in the collision manifold
	PR_ASSERT(PR_DBG_PHYSICS, min_overlap.m_penetration >= 0.0f, "Collision with no penetration?");
	Contact contact;
	contact.m_normal		= -min_overlap.m_axis;
	contact.m_depth			= min_overlap.m_penetration;
	contact.m_material_idA	= shapeA.m_base.m_material_id;
	contact.m_material_idB	= shapeB.m_base.m_material_id;
	box_vs_box::GetPointOfContactBoxVsBox(contact.m_pointA, contact.m_pointB, min_overlap);
	manifold.Add(contact);
	return true;
}

// Detect collisions between box shapes
void pr::ph::BoxVsBox(Shape const& shapeA, m4x4 const& a2w, Shape const& shapeB, m4x4 const& b2w, ContactManifold& manifold, CollisionCache* cache)
{
	PR_DECLARE_PROFILE(PR_PROFILE_BOX_COLLISION, phBoxVsBox);
	PR_PROFILE_SCOPE(PR_PROFILE_BOX_COLLISION, phBoxVsBox);
	Collide(shape_cast<ShapeBox>(shapeA), a2w, shape_cast<ShapeBox>(shapeB), b2w, manifold, cache);
}
