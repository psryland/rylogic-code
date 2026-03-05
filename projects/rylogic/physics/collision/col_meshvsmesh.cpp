//*********************************************
// Physics engine
//  Copyright (c) Rylogic Ltd 2006
//*********************************************

#include "physics/utility/stdafx.h"
#include "pr/physics/collision/collider.h"
#include "pr/physics/collision/contactmanifold.h"
#include "pr/physics/rigidbody/rigidbody.h"
#include "pr/physics/shape/shapepolytope.h"
#include "physics/collision/idpaircache.h"
#include "physics/collision/simplex.h"
#include "physics/collision/collisioncouple.h"
#include "physics/utility/debug.h"
#include "physics/utility/profile.h"

using namespace pr::geometry;

namespace pr::ph::mesh_vs_mesh
{
	float SeparationTolerance = 0.01f;
	float PenetrationTolerance = 0.01f;	// Fractional error it penetration distance
	PR_EXPAND(PR_DBG_MESH_COLLISION, int max_loop_count = 0;)
	PR_EXPAND(PR_DBG_MESH_COLLISION, int refine_edge_count = 0;)
	PR_EXPAND(PR_DBG_MESH_COLLISION, int get_opposing_edge_count = 0;)

	#if PR_DBG_PHYSICS == 1
	#include "pr/Physics/Shape/ShapePolytope.h"

	// Test the support vertex against the normal of every face of A and B and
	// against all of the cross product combinations. Return true for collision
	bool CollideBruteForce(Shape const& shapeA, m4x4 const& a2w, Shape const& shapeB, m4x4 const& b2w, v4& normal, bool test_collision_result)
	{
		if (shapeA.m_type != EShape_Polytope) return test_collision_result;
		if (shapeB.m_type != EShape_Polytope) return test_collision_result;

		ShapePolytope const& polyA = shape_cast<ShapePolytope>(shapeA);
		ShapePolytope const& polyB = shape_cast<ShapePolytope>(shapeB);
		auto w2a = InvertAffine(a2w.rot);
		auto w2b = InvertAffine(b2w.rot);

		//	if( a2w.pos == b2w.pos ) return false;

		float dp, shallowest = -limits<float>::max();
		std::size_t id = 0;
		for (std::size_t f = 0; f != polyA.m_face_count; ++f)
		{
			// ObjA
			v4 axis = a2w * Normalise(Cross(polyA.vertex(polyA.face(f).m_index[1]) - polyA.vertex(polyA.face(f).m_index[0]),
				polyA.vertex(polyA.face(f).m_index[2]) - polyA.vertex(polyA.face(f).m_index[0])));
			v4 p = a2w * SupportVertex(shapeA, w2a *  axis, id, id);
			v4 q = b2w * SupportVertex(shapeB, w2b * -axis, id, id);
			v4 r = Normalise(q - p);
			dp = Dot3(axis, r);
			if (dp >= 0.0f)
				return false; // no collision
			if (dp > shallowest)
				shallowest = dp; normal = axis;

			p = a2w * SupportVertex(shapeA, w2a * -axis, id, id);
			q = b2w * SupportVertex(shapeB, w2b *  axis, id, id);
			r = Normalise(q - p);
			dp = Dot3(-axis, r);
			if (dp >= 0.0f)
				return false; // no collision
			if (dp > shallowest)
			{
				shallowest = dp; normal = axis;
			}
		}

		for (std::size_t f = 0; f != polyB.m_face_count; ++f)
		{
			// ObjB
			v4 axis = b2w * Normalise(Cross(polyB.vertex(polyB.face(f).m_index[1]) - polyB.vertex(polyB.face(f).m_index[0]),
				polyB.vertex(polyB.face(f).m_index[2]) - polyB.vertex(polyB.face(f).m_index[0])));
			v4 p = a2w * SupportVertex(shapeA, w2a *  axis, id, id);
			v4 q = b2w * SupportVertex(shapeB, w2b * -axis, id, id);
			v4 r = Normalise(q - p);
			dp = Dot3(axis, r);
			if (dp >= 0.0f)
				return false; // no collision
			if (dp > shallowest)
			{
				shallowest = dp; normal = axis;
			}
			p = a2w * SupportVertex(shapeA, w2a * -axis, id, id);
			q = b2w * SupportVertex(shapeB, w2b *  axis, id, id);
			r = Normalise(q - p);
			dp = Dot3(-axis, r);
			if (dp >= 0.0f)
				return false; // no collision
			if (dp > shallowest)
				shallowest = dp; normal = axis;
		}
		for (std::size_t fj = 0; fj != polyB.m_face_count; ++fj)
		{
			for (std::size_t fi = 0; fi != polyA.m_face_count; ++fi)
			{
				for (std::size_t ej = 0; ej != 3; ++ej)
				{
					for (std::size_t ei = 0; ei != 3; ++ei)
					{
						v4 si = a2w *  polyA.vertex(polyA.face(fi).m_index[(ei + 0) % 3]); si;
						v4 sj = b2w *  polyB.vertex(polyB.face(fj).m_index[(ej + 0) % 3]); sj;
						v4 edgei = a2w * (polyA.vertex(polyA.face(fi).m_index[(ei + 1) % 3]) - polyA.vertex(polyA.face(fi).m_index[(ei + 0) % 3]));
						v4 edgej = b2w * (polyB.vertex(polyB.face(fj).m_index[(ej + 1) % 3]) - polyB.vertex(polyB.face(fj).m_index[(ej + 0) % 3]));
						v4 axis = Cross(edgei, edgej);
						if (axis == v4::Zero()) continue;

						axis = Normalise(axis);
						v4 p = a2w * SupportVertex(shapeA, w2a *  axis, id, id);
						v4 q = b2w * SupportVertex(shapeB, w2b * -axis, id, id);
						v4 r = Normalise(q - p);
						dp = Dot3(axis, r);
						if (dp >= 0.0f)
							return false; // no collision
						if (dp > shallowest)
						{
							shallowest = dp; normal = axis;
							PR_EXPAND(PR_DBG_MESH_COLLISION, ldr::Line("Edge0", "FFFF0000", si, si + edgei));
							PR_EXPAND(PR_DBG_MESH_COLLISION, ldr::Line("Edge1", "FFFF0000", sj, sj + edgej));
						}
						p = a2w * SupportVertex(shapeA, w2a * -axis, id, id);
						q = b2w * SupportVertex(shapeB, w2b *  axis, id, id);
						r = Normalise(q - p);
						dp = Dot3(-axis, r);
						if (dp >= 0.0f)
							return false; // no collision
						if (dp > shallowest)
						{
							shallowest = dp; normal = axis;
							PR_EXPAND(PR_DBG_MESH_COLLISION, ldr::Line("Edge0", "FFFF0000", si, si + edgei));
							PR_EXPAND(PR_DBG_MESH_COLLISION, ldr::Line("Edge1", "FF0000FF", sj, sj + edgej));
						}
					}
				}
			}
		}
		return true;
	}

	// If a half plane exists there should be at least 2 vectors that lie on it
	// and all other vectors have a positive dot product with the cross of those 2
	bool FindHalfPlaneBruteForce(v4 const* r, uint32_t r_size, bool should_exist)
	{
		for (uint32_t j = 0; j != r_size; ++j)
		{
			for (uint32_t i = 0; i != r_size; ++i)
			{
				if (i == j) continue;
				v4 half_space_normal = Cross(r[i], r[j]);
				if (!FEql(half_space_normal, v4::Zero()))
				{
					half_space_normal = Normalise(half_space_normal);
					bool all_positive = true;
					for (uint32_t k = 0; k != r_size; ++k)
					{
						if ((should_exist && Dot3(half_space_normal, r[k]) < -maths::tiny<float>) ||
							(!should_exist && Dot3(half_space_normal, r[k]) < maths::tiny<float>))
						{
							all_positive = false;
							break;
						}
					}
					if (all_positive)
					{
						return true;
					}
				}
			}
		}
		return false;
	}
	bool VerifyHalfSpace(v4 const* r, uint32_t r_size, v4 half_space_normal)
	{
		r; half_space_normal;
		PR_ASSERT(PR_DBG_PHYSICS, !FEql(half_space_normal,pr::v4::Zero()), "");
		for( uint32_t i = 0; i != r_size; ++i )
		{
			PR_ASSERT(PR_DBG_PHYSICS, Dot3(half_space_normal, r[i]) > -0.01f, "");
		}
		return true;
	}
	#endif//PR_DBG_PHYSICS
			
	// Fills out 'tri' by repeating 'vert' for all of its vertices
	inline void CreateTriangleFromVert(Triangle& tri, Vert const& vert)
	{
		tri.m_vert[2] = tri.m_vert[1] = tri.m_vert[0] = vert;
		tri.m_direction = vert.m_direction;
		tri.m_distance  = Length(vert.m_r);
		PR_EXPAND(PR_DBG_MESH_COLLISION, ldr::phTriangle(tri);)
	}

	// Clip an edge to a triangle
	inline void ClipEdgeToTriangle(v4 const (&tri)[3], v4 face_norm, v4& s, v4& e)
	{
		for( int i = 0; i != 3; ++i )
		{
			v4 norm = Cross(face_norm, tri[(i+1)%3] - tri[i]);
			if( FEql(norm,pr::v4::Zero()) ) continue;
			norm = Normalise(norm);
			if( Dot3(norm, tri[(i+2)%3] - tri[i]) < 0.0f ) norm = -norm;
			float d1 = Dot3(norm, s - tri[i]);
			float d2 = Dot3(norm, e - tri[i]);
			if	   ( d1 < 0.0f && d2 > 0.0f ) s = s + Clamp(d1/(d1-d2), 0.0f, 1.0f) * (e - s);
			else if( d2 < 0.0f && d1 > 0.0f ) e = s + Clamp(d1/(d1-d2), 0.0f, 1.0f) * (e - s);
		}
	}

	// This function interprets the collision manifold implied by 'nearest'
	// and uses it to fill in the contact points, and normal
	void GetContactManifold(Couple& col, Triangle const& nearest, ContactManifold& manifold)
	{
		Contact contact;
		contact.m_normal		= -nearest.m_direction;
		contact.m_depth			= nearest.m_distance;
		contact.m_material_idA	= col.m_shapeA.m_material_id;
		contact.m_material_idB	= col.m_shapeB.m_material_id;

		enum { Point = 0, Edge = 2, Face = 3 };
		int contact_typeA = (nearest.m_vert[0].m_id_p != nearest.m_vert[1].m_id_p) +
							(nearest.m_vert[0].m_id_p != nearest.m_vert[2].m_id_p) +
							(nearest.m_vert[1].m_id_p != nearest.m_vert[2].m_id_p);
		int contact_typeB = (nearest.m_vert[0].m_id_q != nearest.m_vert[1].m_id_q) +
							(nearest.m_vert[0].m_id_q != nearest.m_vert[2].m_id_q) +
							(nearest.m_vert[1].m_id_q != nearest.m_vert[2].m_id_q);

		// Careful here, remember the verts in nearest.m_vert can be in any order,
		// don't assume faces have the correct winding order
		switch( (contact_typeA << 2)|contact_typeB )
		{
		case (Point<<2)|Point:
			{
				contact.m_pointA	= nearest.m_vert[0].m_p;
				contact.m_pointB	= nearest.m_vert[0].m_q;
			}break;
		case (Point<<2)|Edge:
		case (Point<<2)|Face:
			{
				v4 pt = nearest.m_vert[0].m_p;
				contact.m_pointA	= pt;
				contact.m_pointB	= pt - nearest.m_direction * nearest.m_distance;
			}break;
		case (Edge<<2)|Point:
		case (Face<<2)|Point:
			{
				v4 pt = nearest.m_vert[0].m_q;
				contact.m_pointA	= pt + nearest.m_direction * nearest.m_distance;
				contact.m_pointB	= pt;
			}break;
		case (Edge<<2)|Edge:
			{
				v4 s0 = nearest.m_vert[0].m_p;
				v4 e0 = (nearest.m_vert[1].m_id_p != nearest.m_vert[0].m_id_p ? nearest.m_vert[1].m_p : nearest.m_vert[2].m_p);
				v4 s1 = nearest.m_vert[0].m_q;
				v4 e1 = (nearest.m_vert[1].m_id_q != nearest.m_vert[0].m_id_q ? nearest.m_vert[1].m_q : nearest.m_vert[2].m_q);
				closest_point::LineToLine(s0, e0, s1, e1, contact.m_pointA, contact.m_pointB);
			}break;
		case (Edge<<2)|Face:
			{
				v4 s =  nearest.m_vert[0].m_p;
				v4 e = (nearest.m_vert[1].m_id_p != nearest.m_vert[0].m_id_p ? nearest.m_vert[1].m_p : nearest.m_vert[2].m_p);
				v4 tri[3] = {nearest.m_vert[0].m_q, nearest.m_vert[1].m_q, nearest.m_vert[2].m_q};
				ClipEdgeToTriangle(tri, -nearest.m_direction, s, e);
				contact.m_pointA = (s + e) * 0.5f;
				contact.m_pointB = (s + e) * 0.5f - nearest.m_direction * nearest.m_distance;
			}break;
		case (Face<<2)|Edge:
			{
				v4 s =  nearest.m_vert[0].m_q;
				v4 e = (nearest.m_vert[1].m_id_q != nearest.m_vert[0].m_id_q ? nearest.m_vert[1].m_q : nearest.m_vert[2].m_q);
				v4 tri[3] = {nearest.m_vert[0].m_p, nearest.m_vert[1].m_p, nearest.m_vert[2].m_p};
				ClipEdgeToTriangle(tri, nearest.m_direction, s, e);
				contact.m_pointA = (s + e) * 0.5f + nearest.m_direction * nearest.m_distance;
				contact.m_pointB = (s + e) * 0.5f;
			}break;
		case (Face<<2)|Face:
			{
				// Clip the edges of one face against the other
				v4 avr = v4::Zero();
				v4 tri[3] = {nearest.m_vert[0].m_q, nearest.m_vert[1].m_q, nearest.m_vert[2].m_q};
				for( int i = 0; i != 3; ++i )
				{
					v4 s = nearest.m_vert[i].m_p;
					v4 e = nearest.m_vert[(i+1)%3].m_p;
					ClipEdgeToTriangle(tri, nearest.m_direction, s, e);
					avr += s + e;
				}
				avr /= 6.0f;
				contact.m_pointA = avr;
				contact.m_pointB = avr - nearest.m_direction * nearest.m_distance;
			}break;
		}

		PR_EXPAND(PR_DBG_MESH_COLLISION, ldr::phContact("Contact", "FFFFFF00", contact, "contact");)
		manifold.Add(contact);
	}


	// Return true if the last sampled point in 'col' is normal to the surface of the Minkowski difference
	inline bool PointAndNormalAreAligned(const Couple& col)
	{
		return Abs(LengthSq(col.m_nearest.m_r) - col.m_dist_sq_upper_bound) < PenetrationTolerance * PenetrationTolerance;
	}

	// Sets 'vert' to 'nearest' if it represents a tighter bound
	// on the nearest point than 'nearest'. Returns true if 'nearest'
	// intersects the Minkowski difference 
	bool SetNearestBound(Vert const& vert, Couple& col)
	{
		// Use the normal to the surface at 'vert' to bound the nearest distance
		// Remember the vert that last bounded the distance
		float dist_sq = Sqr(Dot3(vert.m_direction, vert.m_r)) / LengthSq(vert.m_direction);
		if( dist_sq < col.m_dist_sq_upper_bound )
		{
			col.m_dist_sq_upper_bound = dist_sq;
			col.m_nearest = vert;
				PR_EXPAND(PR_DBG_MESH_COLLISION, StartFile("C:/DeleteMe/collision_upperbound.pr_script");)
				PR_EXPAND(PR_DBG_MESH_COLLISION, ldr::Sphere("Upper_Bound", "80FF0000", v4::Origin(), Sqrt(col.m_dist_sq_upper_bound));)
				PR_EXPAND(PR_DBG_MESH_COLLISION, ldr::Line  ("TargetNorm" , "FFFFFF00", v4::Origin(), Normalise(vert.m_direction) * Sqrt(col.m_dist_sq_upper_bound));)
				PR_EXPAND(PR_DBG_MESH_COLLISION, EndFile();)
					
			// If the line from the origin to the vert is aligned with the normal then we've found our result
			return PointAndNormalAreAligned(col);
		}
		return false;
	}

	// Sample the Minkowski hull in the directions of the verts and face normals of the simplex.
	// Returns true if the nearest point is found during the sampling
	void SampleMinkowskiDiff(Couple& col)
	{
		switch( col.m_simplex.m_num_vertices )
		{
		case 0:
			PR_ASSERT(PR_DBG_PHYSICS, false, "This function should not be called with an empty simplex");
			break;
		case 1:
			SetNearestBound(col.m_simplex.m_vertex[0], col);
			break;
		case 2:
			{
				SetNearestBound(col.m_simplex.m_vertex[0], col);
				SetNearestBound(col.m_simplex.m_vertex[1], col);
				v4 x = col.m_simplex.m_vertex[1].m_r - col.m_simplex.m_vertex[0].m_r;
				v4 y = Perpendicular(x);
				v4 z = Cross(x, y);
				col.SupportVertex( y);	SetNearestBound(col.m_vertex, col);
				col.SupportVertex( z);	SetNearestBound(col.m_vertex, col);
				col.SupportVertex(-y);	SetNearestBound(col.m_vertex, col);
				col.SupportVertex(-z);	SetNearestBound(col.m_vertex, col);
			}break;
		case 3:
			{
				SetNearestBound(col.m_simplex.m_vertex[0], col);
				SetNearestBound(col.m_simplex.m_vertex[1], col);
				SetNearestBound(col.m_simplex.m_vertex[2], col);
						
				// Use the normal of the face to add a vertex on either side
				v4 norm = Cross(col.m_simplex.m_vertex[1].m_r - col.m_simplex.m_vertex[0].m_r, col.m_simplex.m_vertex[2].m_r - col.m_simplex.m_vertex[0].m_r);
				col.SupportVertex( norm);	SetNearestBound(col.m_vertex, col);
				col.SupportVertex(-norm);	SetNearestBound(col.m_vertex, col);
			}break;
		case 4:
			{
				// Add the triangles from the simplex
				const int tetra_tris[12] = {0, 1, 2,  0, 2, 3,  0, 3, 1,  3, 2, 1}, *tris = tetra_tris;
				for( int i = 0; i != 4; ++i, tris += 3 )
				{
					SetNearestBound(col.m_simplex.m_vertex[i], col);
					v4 norm = Cross(	col.m_simplex.m_vertex[tris[2]].m_r - col.m_simplex.m_vertex[tris[0]].m_r,
										col.m_simplex.m_vertex[tris[1]].m_r - col.m_simplex.m_vertex[tris[0]].m_r);
							
					// Ensure outward facing normals
					if( Dot3(norm, col.m_simplex.m_vertex[tris[0]].m_r) < 0.0f )
						norm = -norm;

					col.SupportVertex(norm);
					SetNearestBound(col.m_vertex, col);
				}
			}break;
		}
	}

	// Choose a vert with an offset that opposes 'a'
	bool GetOpposingVert(Couple& col, TrackVert& a, TrackVert& b, v4 refine_normal_direction)
	{
		PR_EXPAND(PR_DBG_MESH_COLLISION, int loop_count = 0;)
		PR_EXPAND(PR_DBG_MESH_COLLISION, get_opposing_edge_count = 0;)

		// Look in the direction of 'refine_normal_direction' for another (opposing) vertex
		// The test direction must not go passed 90 degrees to 'a.direction()'. It is possible
		// we won't find a direction that opposes 'refine_normal_direction' so limit the maximum
		// number of iterations
		for( int i = 0; i != 10; ++i )
		{
			PR_EXPAND(PR_DBG_MESH_COLLISION, if( loop_count++ > max_loop_count ) max_loop_count = loop_count;)
			PR_EXPAND(PR_DBG_MESH_COLLISION, ++get_opposing_edge_count;)

			v4 dir = Length(a.direction() + a.offset()) * refine_normal_direction;
			col.SupportVertex(Normalise(dir));
			b.set(col.m_vertex);
			if( SetNearestBound(b.vert(), col) ) return true;
			PR_EXPAND(PR_DBG_MESH_COLLISION, AppendFile("C:/DeleteMe/collision_nearestpoint.pr_script"); ldr::phTrackingVert(b.vert(), "FFFF0000"); EndFile();)

			// If this vert is on the same side of the origin as 'a' (in the direction of refine_normal_direction)
			// then it should be an improvement on 'a', but we still need to find an opposing vert.
			if( Dot3(b.offset(), refine_normal_direction) > 0.0f )
				break; // Break when an opposing vertex is found
			else
				a = b; // Refine trk[0] and try again
		}
		return false;
	}

	// Refine an edge by bringing the normals into alignment
	// 'refine_normal_direction' - This is the direction we bend the normals.
	bool RefineEdge(Couple& col, TrackVert& a, TrackVert& b, v4 refine_normal_direction)
	{
		PR_EXPAND(PR_DBG_MESH_COLLISION, int loop_count = 0;)

		Vert test_vert;
		TrackVert test = {&test_vert, 0.0f, v4::Zero()};
		for(;;)
		{
			PR_EXPAND(PR_DBG_MESH_COLLISION, ++refine_edge_count;)
			PR_EXPAND(PR_DBG_MESH_COLLISION, if( ++loop_count > max_loop_count ) max_loop_count = loop_count;)

			// Look in the average direction
			col.SupportVertex(Normalise(a.direction() + b.direction()));
			test.set(col.m_vertex);
			if( SetNearestBound(test.vert(), col) ) return true;
			PR_EXPAND(PR_DBG_MESH_COLLISION, AppendFile("C:/DeleteMe/collision_nearestpoint.pr_script"); ldr::phTrackingVert(test.vert(), "FFFF0000"); EndFile();)

			// Replace the vert that 'test' is on the same side as
			if( Dot3(test.offset(), refine_normal_direction) > 0.0f )	{ b = test; }
			else														{ a = test; }

			// If the directions are now equal, return
			if( FEqlRelative(a.direction(), b.direction(), PenetrationTolerance) )
				return false;
		}
	}

	// Find the depth of penetration and collision normal for two intersecting shapes.
	// Algorithm:
	// Have a guess in the direction of m_offset;
	// if the guess is too small this should just improve estimate[0]
	// else we'll get a new vert
	// when we get the second vert start bringing the normal back until we get the first vert again
	//	actually bring in the normal that isn't clipping the sphere
	// now use the perpendicular direction to the line connecting the first and second vert,
	// take this cross estimate[0].m_norm. = diff
	// Look in the direction of estimate[0].m_offset dot diff until a new vert is found
	// when the third vert is found bring the normal back until we get one of the first two verts
	// now use the normal of the triangle and use the vert in this direction as the nearest
	// If at any point we get a vert for which m_offset is zero then we're done
	void FindPenetration(Couple& col, Triangle& triangle)
	{
		PR_ASSERT(PR_DBG_PHYSICS, col.m_dist_sq_upper_bound == limits<float>::max(), "");
		PR_EXPAND(PR_DBG_MESH_COLLISION, StartFile("C:/DeleteMe/collision_nearestpoint.pr_script");EndFile();)
		PR_EXPAND(PR_DBG_MESH_COLLISION, refine_edge_count = 0; get_opposing_edge_count = 0;)

		// If the nearest point was found while sampling the Minkowski hull then
		// lucky us! Set all the verts of the triangle to the same point and return.
		SampleMinkowskiDiff(col);
		if( PointAndNormalAreAligned(col) )
		{
			col.m_nearest.m_direction = Normalise(col.m_nearest.m_direction);
			return CreateTriangleFromVert(triangle, col.m_nearest);
		}
				
		TrackVert trk[3] = {
			{&triangle.m_vert[0], 0.0f, v4::Zero()},
			{&triangle.m_vert[1], 0.0f, v4::Zero()},
			{&triangle.m_vert[2], 0.0f, v4::Zero()}};

		// Start with the best estimate from the simplex
		col.m_nearest.m_direction = Normalise(col.m_nearest.m_direction);
		trk[0].set(col.m_nearest);
		PR_EXPAND(PR_DBG_MESH_COLLISION, ldr::phTrackingVert(trk, 1);)

		v4 refine_normal_direction = Normalise(v4::Origin() - trk[0].offset());
		for( int i = 0; i != 3; ++i )
		{
			int j = (i + 1) % 3;
			int k = (i + 2) % 3;

			// Find a vert on the opposite site of the origin to trk[i]
			if( GetOpposingVert(col, trk[i], trk[j], refine_normal_direction) )
				return CreateTriangleFromVert(triangle, col.m_nearest);
			PR_EXPAND(PR_DBG_MESH_COLLISION, ldr::phTrackingVert(trk, i==0?2:3);)

			// Bring the normals of these verts into alignment
			if( RefineEdge(col, trk[i], trk[j], refine_normal_direction) )
				return CreateTriangleFromVert(triangle, col.m_nearest);
			PR_EXPAND(PR_DBG_MESH_COLLISION, ldr::phTrackingVert(trk, i==0?2:3);)
					
			// If this is the last iteration, or all three normals now agree
			// (note: trk[i] == trk[j] because of 'RefineEdge') then we're done
			if( i == 2 || (i > 0 && FEqlRelative(trk[i].direction(), trk[k].direction(), PenetrationTolerance)) )
				break;

			// If the origin does not project onto the edge formed between 'i' and 'i+1'
			// then find a new direction to refine the normal in
			v4 edge = trk[j].vert().m_r - trk[i].vert().m_r;
			if(edge== v4::Zero())	{ refine_normal_direction = v4::Origin() - trk[i].offset(); }
			else				{ refine_normal_direction = Normalise(Cross(trk[i].direction(), edge)); }

			// Make sure we choose a direction toward the origin
			float side = Dot3(refine_normal_direction, trk[i].offset());
						
			// If the origin projects onto the edge then use this as the closest point
			if( FEql(side, 0.0f) )
			{
				trk[k] = trk[i];
				break;
			}
			// Otherwise, flip the direction so we head towards the origin
			else if( side > 0.0f )
			{
				refine_normal_direction = -refine_normal_direction;
			}
		}
		PR_EXPAND(PR_DBG_MESH_COLLISION, ldr::phTrackingVert(trk, 3);)

		triangle.m_direction = triangle.m_vert[2].m_direction;
		triangle.m_distance  = Sqrt(col.m_dist_sq_upper_bound);
		PR_EXPAND(PR_DBG_MESH_COLLISION, ldr::phTriangle(triangle);)

		PR_EXPAND(PR_DBG_MESH_COLLISION, StartFile("C:/Deleteme/collision_features.pr_script");)
		PR_EXPAND(PR_DBG_MESH_COLLISION, ldr::GroupStart("FeatureA");)
		PR_EXPAND(PR_DBG_MESH_COLLISION, ldr::Sphere("s0", "FFFF0000", triangle.m_vert[0].m_p, 0.01f);)
		PR_EXPAND(PR_DBG_MESH_COLLISION, ldr::Sphere("s1", "FFFF0000", triangle.m_vert[1].m_p, 0.01f);)
		PR_EXPAND(PR_DBG_MESH_COLLISION, ldr::Sphere("s2", "FFFF0000", triangle.m_vert[2].m_p, 0.01f);)
		PR_EXPAND(PR_DBG_MESH_COLLISION, ldr::GroupEnd();)
		PR_EXPAND(PR_DBG_MESH_COLLISION, ldr::GroupStart("FeatureB");)
		PR_EXPAND(PR_DBG_MESH_COLLISION, ldr::Sphere("s0", "FF0000FF", triangle.m_vert[0].m_q, 0.01f);)
		PR_EXPAND(PR_DBG_MESH_COLLISION, ldr::Sphere("s1", "FF0000FF", triangle.m_vert[1].m_q, 0.01f);)
		PR_EXPAND(PR_DBG_MESH_COLLISION, ldr::Sphere("s2", "FF0000FF", triangle.m_vert[2].m_q, 0.01f);)
		PR_EXPAND(PR_DBG_MESH_COLLISION, ldr::GroupEnd();)
		PR_EXPAND(PR_DBG_MESH_COLLISION, EndFile();)
	}

	// This function attempts to find a half space in which all 'r' are on one side.
	// Returns true if such a half space exists
	// 'r'                 - an array of vectors
	// 'r_size'            - the length of the array
	// 'first_new_r'       - the index of the first 'r' that may not be in the current half space. Updated to r_size on returning true
	// 'half_space_normal' - the normal of the current half space. Updated if true is returned
	bool FindHalfPlane(v4 const* r, int r_size, int& first_new_r, v4& half_space_normal)
	{
		// 'r' must contain at least two entries before this method is called
		PR_ASSERT(PR_DBG_PHYSICS, r_size >= 2, "");

		// Initialise the half space normal if 'first_new_r' is position zero
		if (first_new_r == 0)
		{
			// Initialise the half space normal once we have two r's
			// If the product is zero, then any vector perpendicular to 'r[0]' should do
			half_space_normal = r[0] + r[1];
			if (FEql(half_space_normal, pr::v4::Zero())) half_space_normal = Perpendicular(r[0]);
			else                                     half_space_normal = Normalise(half_space_normal);
			first_new_r = 2;

			PR_ASSERT(PR_DBG_MESH_COLLISION, VerifyHalfSpace(r, 2, half_space_normal), "");
		}

		// Add each of the new vectors to the half space. (It doesn't work to find the depthest and use that)
		for (; first_new_r != r_size; ++first_new_r)
		{
			// Ignore vectors already above the half space
			if (Dot3(half_space_normal, r[first_new_r]) >= -maths::tiny<float>) continue;

			v4 new_r = r[first_new_r];

			// If 'new_r' lies outside the current half space then 'new_r' should lie in the plane
			// of a new half space (if it exists). This constrains the half space around one axis (new_r).
			// If we project all previous 'r' into the plane perpendicular to 'new_r' and there
			// is a line for which all other projected 'r's are on one side of, then this line is
			// the another constraint for the half space and a valid half space still exists

			// Local inline function for evaluating 'line' at 'pt'
			struct Line { static float Eqn(v2 const& line, v2 const& pt) { return pt.x * line.y - pt.y * line.x; } };

			// Construct a rotation matrix that transforms 'new_r' onto the z axis
			// This means all other 'r' can be projected into the XY place by rotating
			// them, then dropping their z value
			m3x4 M = RotationToZAxis(new_r);
			//m3x4 M;
			//{
			//	float const& x = new_r.x;
			//	float const& y = new_r.y;
			//	float const& z = new_r.z;
			//	float d = Sqrt(x*x + y*y);
			//	if( FEql(d, 0.0f) )
			//	{
			//		M = m3x4::Identity();	// Create an identity transform or a 180 degree rotation
			//		M.x.x = new_r.z;	// about Y depending on the sign of 'new_r.z'
			//		M.z.z = new_r.z;
			//	}
			//	else
			//	{
			//		M.x.Set(         x*z/d,  -y/d, x, 0.0f);
			//		M.y.Set(         y*z/d,   x/d, y, 0.0f);
			//		M.z.Set(-(x*x + y*y)/d,  0.0f, z, 0.0f);
			//	}
			//}

			// 'ra' and 'rb' are bounds for the line in the XY place
			int i = 0;
			v2 ra = v2Zero, rb = v2Zero;
			for (; i != first_new_r && FEql(ra, v2Zero);) { ra = (M * r[i++]).xy; }
			for (; i != first_new_r && FEql(rb, v2Zero);) { rb = (M * r[i++]).xy; }

			// We need to ensure 'rb' is on the positive side of 'ra'
			if (Line::Eqn(ra, rb) < 0.0f)
			{
				v2 tmp = ra; ra = rb; rb = tmp;	// swap
			}

			// Project the remaining 'r' into the XY plane
			for (i = 2; i != first_new_r + 1; ++i)
			{
				v2 t = (M * r[i]).xy;
				if (!FEql(t, v2Zero))
				{
					if (Line::Eqn(ra, t) >= 0.0f)
					{
						if (Line::Eqn(rb, t) > 0.0f)
						{
							rb = t;
						}
					}
					else
					{
						if (Line::Eqn(rb, t) > 0.0f)
						{
							PR_ASSERT(PR_DBG_MESH_COLLISION, !FindHalfPlaneBruteForce(r, r_size, false), "");
							return false;	// Cannot find a half space, there must be a collision
						}
						else
						{
							ra = t;
						}
					}
				}
			}
			// If we get here then a half space is possible. i.e. rb should still be on the positive side of 'ra'
			PR_ASSERT(PR_DBG_PHYSICS, Line::Eqn(ra, rb) >= -maths::tiny<float>, "");

			// Calculate a new half space normal. Use the perpendicular to 'ra' unless
			// that's zero in which case, use the perpendicular to 'rb'. If that's zero
			// as well then is doesn't matter what we use, might as well be the x axis
			v2 rn = v2(ra[1], -ra[0]);
			if (!FEql(rn, v2Zero)) { rn = Normalise(rn); }
			else
			{
				rn = v2(-rb[1], rb[0]);
				if (!FEql(rn, v2Zero)) { rn = Normalise(rn); }
				else { rn = v2XAxis; }
			}
			half_space_normal = Transpose(M) * v4(rn, 0.0f, 0.0f);

			PR_ASSERT(PR_DBG_MESH_COLLISION, VerifyHalfSpace(r, first_new_r + 1, half_space_normal), "");
			PR_ASSERT(PR_DBG_MESH_COLLISION, FindHalfPlaneBruteForce(r, first_new_r + 1, true), "");
		}
		return true;
	}

	// Collide using the GJK collision detection algorithm.
	// This code is based on the description given in "Real-time Collision Detection" by Christer Ericson
	// At the completion of the algorithm 'simplex' contains the closed features between
	// shapeA and shapeB or a simplex that contains the origin.
	// 'simplex' may be initialised with up to 4 vertices from the Minkowski difference of shapeA and shapeB
	bool CollideGJK(Couple& col)
	{
		// If the simplex is empty, initialise it using the separating axis
		if( col.m_simplex.m_num_vertices == 0 )
		{
			col.SupportVertex(col.m_separating_axis);
			col.m_simplex.AddVertex(col.m_vertex);
		}

		// Iteratively find the nearest point to the origin
		float last_nearest_distanceSq = limits<float>::max();
		for( int k = 0; ; ++k )
		{
			PR_EXPAND(PR_DBG_MESH_COLLISION, ldr::PhSimplex("Simplex", "FF00FF00", col.m_simplex, "simplex");)

			// Find the minimum normal distance from the convex hull of the simplex to the origin
			v4 nearest_point = col.m_simplex.FindNearestPoint(v4::Origin()) - v4::Origin();
			float nearest_distanceSq = LengthSq(nearest_point);
			if( nearest_distanceSq >= last_nearest_distanceSq )
			{
				PR_EXPAND(PR_DBG_MESH_COLLISION, DebugOutput(Fmt("GJK: Collision rejected after %d iterations\n", k).c_str());)
				return false;
			}
			else
			{
				last_nearest_distanceSq = nearest_distanceSq;
			}
			PR_EXPAND(PR_DBG_MESH_COLLISION, StartFile("C:/DeleteMe/collision_nearest.pr_script");)
			PR_EXPAND(PR_DBG_MESH_COLLISION, ldr::Sphere("Nearest", "FFFF0000", nearest_point, 0.02f); EndFile();)

			// If the closest point to the simplex is the origin then the
			// simplex surrounds the origin and the shapes are in collision
			//if( nearest_point.IsApproxZero3() )
			if( nearest_distanceSq < Sqr(maths::tiny<float>) )
			{
				PR_EXPAND(PR_DBG_MESH_COLLISION, DebugOutput(Fmt("GJK: COLLISION detected after %d iterations\n", k).c_str());)
				return true;
			}

			// Determine the new test separating axis from this nearest point
			col.m_separating_axis = nearest_point / -Sqrt(nearest_distanceSq); // Normalise
			//col.m_separating_axis = (v4::Origin() - nearest_point).GetNormal3();
			PR_EXPAND(PR_DBG_MESH_COLLISION, StartFile("C:/DeleteMe/collision_separatingaxis.pr_script");)
			PR_EXPAND(PR_DBG_MESH_COLLISION, ldr::LineD("Sep_axis", "FFFFFF00", nearest_point, col.m_separating_axis); EndFile();)

			// Get the support vertices for 'shapeA' and 'shapeB' using the test separating axis
			col.SupportVertex(col.m_separating_axis);
			PR_EXPAND(PR_DBG_MESH_COLLISION, StartFile("C:/DeleteMe/collision_support.pr_script");)
			PR_EXPAND(PR_DBG_MESH_COLLISION, ldr::Sphere("Support", "FF0000FF", col.m_vertex.m_r, 0.02f); EndFile();)

			// If the support vertex 'r' is no more extreme in the direction of the
			// separating axis than 'nearest_point' then the objects are not in
			// collision and the distance is nearest_point.Length()
			float r_dist = Dot3(col.m_separating_axis, col.m_vertex.m_r);
			float n_dist = Dot3(col.m_separating_axis, nearest_point) + SeparationTolerance;
			if( r_dist <= n_dist )
			{
				PR_EXPAND(PR_DBG_MESH_COLLISION, DebugOutput(Fmt("GJK: Collision rejected after %d iterations\n", k).c_str());)
				return false; // non-collision
			}
					
			// Otherwise, add 'p' and 'q' to the simplex and try again
			if( !col.m_simplex.AddVertex(col.m_vertex) )
			{
				PR_EXPAND(PR_DBG_MESH_COLLISION, DebugOutput(Fmt("GJK: Collision rejected after %d iterations\n", k).c_str());)
				return false;
			}
		}
	}
	bool CollideGJK(Shape const& shapeA, m4x4 const& a2w, Shape const& shapeB, m4x4 const& b2w, CollisionCache* cache)
	{
		Couple col(shapeA, a2w, shapeB, b2w, cache);
		return CollideGJK(col);
	}
	bool CollideGJK(Shape const& shapeA, m4x4 const& a2w, Shape const& shapeB, m4x4 const& b2w, ContactManifold& manifold, CollisionCache* cache)
	{
		Couple col(shapeA, a2w, shapeB, b2w, cache);
		if( !CollideGJK(col) ) return false;

		// Determine the penetration depth
		Triangle nearest;
		FindPenetration(col, nearest);

		// Convert the results of the penetration test into a collision manifold
		GetContactManifold(col, nearest, manifold);
		return true;
	}

	// Collide mesh vs mesh using the Chung Wang separating axis algorithm.
	// and the GJK collision algorithm. This code is based on the thesis:
	// "An efficient collision detection algorithm for polytopes in virtual
	// Environments" by Kelvin Chung Tat Leung.
	bool Collide(Couple& col)
	{
		bool	using_half_space_normal = false;	// True when we are using the half space normal as a test separating vector
		v4		r[MaxIterations];					// These are the world space vectors between the support vertices
		v4		half_space_normal;					// This is the normal of the half space plane for which all 'r' are above
		int		half_space_index = 0;

		IdPairCache id_cache;			// These are the ids of the support vertices visited

		for( int k = 0; k != MaxIterations; ++k )
		{
			// Get the support vertices for 'shapeA' and 'shapeB' using the test separating axis
			col.SupportVertex(col.m_separating_axis);
			id_cache.Add(col.m_vertex.m_id_p, col.m_vertex.m_id_q);
			PR_EXPAND(PR_DBG_MESH_COLLISION, (k==0)?StartFile("C:/DeleteMe/collision_chungwang.pr_script"):AppendFile("C:/DeleteMe/collision_chungwang.pr_script");)
			PR_EXPAND(PR_DBG_MESH_COLLISION, ldr::Sphere("supportA", "FFFF0000", col.m_vertex.m_p, 0.01f);)
			PR_EXPAND(PR_DBG_MESH_COLLISION, ldr::Sphere("supportB", "FF0000FF", col.m_vertex.m_q, 0.01f);)
			PR_EXPAND(PR_DBG_MESH_COLLISION, ldr::Line  ("a_to_b", "FFFF00FF"  , col.m_vertex.m_p,  col.m_vertex.m_q);)
			PR_EXPAND(PR_DBG_MESH_COLLISION, ldr::LineD ("Sep_axis", "FFFFFF00", col.m_vertex.m_p,  col.m_separating_axis);)
			PR_EXPAND(PR_DBG_MESH_COLLISION, ldr::LineD ("Sep_axis", "FFFFFF00", col.m_vertex.m_q, -col.m_separating_axis);)
			PR_EXPAND(PR_DBG_MESH_COLLISION, EndFile();)

			// Get the world space vector between these two vertices
			r[k] = v4::Origin() - col.m_vertex.m_r;
			float dp = Dot3(col.m_separating_axis, r[k]);
			if( dp >= -maths::tiny<float> )	// Lemma 1
			{
				PR_EXPAND(PR_DBG_MESH_COLLISION, DebugOutput(Fmt("CW: Collision rejected in %d iterations\n", k).c_str());)
				return false; // non-collision
			}
			float rk_length = Length(r[k]);
			r[k] /= rk_length;
			dp   /= rk_length;

			// Check whether the current pair of verts have occurred before
			// This could be hashed or something
			int dup_index;
			if( !id_cache.ReoccurringPair(dup_index) )
			{
				// Reflect the test separating axis about the "normal" of 'r[k]'
				col.m_separating_axis -= 2.0f * dp * r[k];	// Eqn 3.2 in the thesis
				using_half_space_normal = false;
			}
			else
			{
				// If the same support vertices are returned in two successive tests
				// then there is no collision between the objects.
				if( dup_index == k - 1 ) // Lemma 3
				{
					PR_EXPAND(PR_DBG_MESH_COLLISION, DebugOutput(Fmt("CW: Collision rejected in %d iterations\n", k).c_str());)
					return false; // non-collision
				}

				// If we are using the half space normal and the same pair of verts
				// occurs again then shapeA and shapeB do not collide
				if( using_half_space_normal )	// Lemma 8
				{
					PR_EXPAND(PR_DBG_MESH_COLLISION, DebugOutput(Fmt("CW: Collision rejected in %d iterations\n", k).c_str());)
					return false; // non-collision
				}
						
				// Look for a half space, if one cannot be found then we have a collision
				if( !FindHalfPlane(r, k + 1, half_space_index, half_space_normal) )	// section 4.2.1
				{
					PR_EXPAND(PR_DBG_MESH_COLLISION, DebugOutput(Fmt("CW: COLLISION detected in %d iterations\n", k).c_str());)
					return true; // Collision!
				}

				// This vertex is also a good candidate for the simplex
				if( col.m_simplex.m_num_vertices < 4 )
					col.m_simplex.AddVertex(col.m_vertex);

				// Use the half space normal. This guarantees that either the algorithm
				// will terminate or a new previously untested vertex pair will be found.
				col.m_separating_axis	= half_space_normal;
				using_half_space_normal = true;
			}
		}

		// If we get here we cannot easily tell whether there is a collision. Use the GJK algorithm for an exact result
		PR_EXPAND(PR_DBG_MESH_COLLISION, DebugOutput(Fmt("CW: Collision unknown after %d iterations\n", MaxIterations).c_str());)
		return CollideGJK(col);
	}
	bool Collide(Shape const& shapeA, m4x4 const& a2w, Shape const& shapeB, m4x4 const& b2w, CollisionCache* cache)
	{
		Couple col(shapeA, a2w, shapeB, b2w, cache);
		return Collide(col);
	}
	bool Collide(Shape const& shapeA, m4x4 const& a2w, Shape const& shapeB, m4x4 const& b2w, ContactManifold& manifold, CollisionCache* cache)
	{
		PR_EXPAND(PR_DBG_MESH_COLLISION, ldr::PhCollisionScene(shapeA, a2w, shapeB, b2w, "meshvsmesh");)
		PR_EXPAND(PR_DBG_MESH_COLLISION, ldr::PhMinkowski("minkowski", "800000FF", shapeA, a2w, shapeB, b2w, "minkowski");)

		Couple col(shapeA, a2w, shapeB, b2w, cache);
		if( !Collide(col) ) return false;
				
		// If the above collision test indicates a collision but the simplex does not contain
		// enough verts, use the GJK algorithm to complete the simplex. This should still be
		// faster than using the GJK algorithm directly in the average case
		if( col.m_simplex.m_num_vertices != 4 )	{ if( !CollideGJK(col) ) return false; }

		// Determine the penetration depth
		Triangle nearest;
		FindPenetration(col, nearest);
				
		// Convert the results of the penetration test into a collision manifold
		GetContactManifold(col, nearest, manifold);
				
		// Update the cache data using the contact triangle
		col.CacheSeparatingAxis(nearest);
		return true;
	}
}

using namespace pr;
using namespace pr::ph;

// Detect collisions between mesh shapes
void pr::ph::MeshVsMesh(Shape const& objA, m4x4 const& a2w, Shape const& objB, m4x4 const& b2w, ContactManifold& manifold, CollisionCache* cache)
{
	PR_DECLARE_PROFILE(PR_PROFILE_MESH_COLLISION, phMeshVsMesh);
	PR_PROFILE_SCOPE(PR_PROFILE_MESH_COLLISION, phMeshVsMesh);
	mesh_vs_mesh::Collide(objA, a2w, objB, b2w, manifold, cache);
}

// Calculate the nearest points between two primitives
bool pr::ph::GetNearestPoints(Shape const& shapeA, m4x4 const& a2w, Shape const& shapeB, m4x4 const& b2w, ContactManifold& manifold, CollisionCache* cache)
{
	mesh_vs_mesh::Couple col(shapeA, a2w, shapeB, b2w, cache);
	if( CollideGJK(col) )
	{
		// Determine the penetration depth
		mesh_vs_mesh::Triangle nearest;
		FindPenetration(col, nearest);

		// Convert the results of the penetration test into a collision manifold
		GetContactManifold(col, nearest, manifold);
		return true;
	}
	else
	{
		v4 nearest_point	= col.m_simplex.FindNearestPoint(v4::Origin()) - v4::Origin();
		Contact contact;
		contact.m_depth		= -Length(nearest_point);
		contact.m_normal	= nearest_point / contact.m_depth;
		contact.m_pointA	= col.m_simplex.GetNearestPointOnA() - col.m_a2w.pos;
		contact.m_pointB	= col.m_simplex.GetNearestPointOnB() - col.m_b2w.pos;
		manifold.Add(contact);
		return false;
	}	
}














