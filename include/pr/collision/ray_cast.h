//*********************************************
// Physics engine
//  Copyright (c) Rylogic Ltd 2006
//*********************************************
#pragma once
#include "pr/collision/shape.h"
#include "pr/collision/ray.h"
#include "pr/collision/ray_cast_result.h"
#include "pr/geometry/intersect.h"
#include "pr/geometry/point.h"

namespace pr::collision
{
	// Forward
	RayCastResult RayCast(Ray const& ray, Shape const& shape);

	// Implementation details
	namespace impl
	{
		// Shift 'ray' toward the centre of a shape to simulate the ray having a thickness.
		// The ray is in shape space so we're shifting it toward the origin.
		inline Ray ShiftTowardOrigin(Ray const& ray)
		{
			if (ray.m_thickness == 0.0f)
				return ray;

			auto direction_len  = Length(ray.m_direction);
			if (direction_len < maths::tiny<float>)
				return ray; // Zero-length ray, cannot shift
			
			auto forward        = ray.m_direction / direction_len;
			auto sideways       = (forward * Dot3(ray.m_point, forward) - ray.m_point).w0();
			auto sideways_len   = Length(sideways);
			sideways *= sideways_len > maths::tiny<float> ? (1.f / sideways_len) : 0.f; // If sideways length is zero, ray passes through origin - no shift needed
			return Ray(
				ray.m_point +
				forward  * pr::Min(direction_len, ray.m_thickness) +
				sideways * pr::Min(sideways_len , ray.m_thickness),
				ray.m_direction, 0.0f);
		}
	}

	// Ray vs. Sphere
	template <typename = void>
	RayCastResult RayCast(Ray const& ray, ShapeSphere const& shape)
	{
		RayCastResult result;

		// Check for zero-length ray direction
		auto direction_lenSq = LengthSq(ray.m_direction);
		if (direction_lenSq < Sqr(maths::tiny<float>))
			return result; // No valid ray direction
		
		// Find the closest point to the line
		auto closest_point   = ray.m_point - ray.m_direction * (Dot(ray.m_direction, ray.m_point) / direction_lenSq);
		auto closest_distSq  = LengthSq(closest_point);
		auto radius          = shape.m_radius + ray.m_thickness;
		auto radiusSq        = radius * radius;

		// If the closest point is not within the sphere then there is no intersection
		if (radiusSq >= closest_distSq)
		{
			// Get the distance from the closest point to the intersection with the boundary of the sphere
			auto x = Sqrt((radiusSq - closest_distSq) / direction_lenSq);

			// Get the parametric values and normal
			auto offset     = ray.m_direction * x;
			auto lstart     = closest_point - offset;
			auto lend       = closest_point + offset;
			result.m_t0     = Dot(ray.m_direction, lstart - ray.m_point) / direction_lenSq;
			result.m_t1     = Dot(ray.m_direction, lend - ray.m_point) / direction_lenSq;
			result.m_normal = (lstart / radius).w0();
			result.m_shape  = &shape.m_base;
		}
		return result;
	}

	// Ray vs. Box
	template <typename = void>
	RayCastResult RayCast(Ray const& ray, ShapeBox const& shape)
	{
		RayCastResult result;

		// For each side of the box
		result.m_t0 = 0.0f;
		result.m_t1 = 1.0f;
		result.m_shape = &shape.m_base;
		for (auto i = 0; i != 3; ++i)
		{
			if (Abs(ray.m_direction[i]) < maths::tiny<float>)
			{
				if (Abs(ray.m_point[i]) > shape.m_radius[i])
					return result;
			}
			else
			{
				// Compute the intersection 't' value of ray with near and far plane
				auto ta = (-(shape.m_radius[i] + ray.m_thickness) - ray.m_point[i]) / ray.m_direction[i];
				auto tb = (+(shape.m_radius[i] + ray.m_thickness) - ray.m_point[i]) / ray.m_direction[i];

				// Make ta the intersection with the near plane, tb the intersection with the far plane
				auto sign = -1.0f;
				if (ta > tb)
				{
					auto swp = ta;
					ta = tb;
					tb = swp;
					sign = 1.0f;
				}

				// Compute the intersection 
				if (ta > result.m_t0)
				{
					result.m_t0 = ta;
					result.m_normal = v4((i == 0) * sign, (i == 1) * sign, (i == 2) * sign, 0.0f);
				}
				if (tb < result.m_t1)
				{
					result.m_t1 = tb;
				}
				if (result.m_t0 > result.m_t1)
				{
					return result;
				}
			}
		}
		return result;
	}

	// Ray vs. Triangle
	template <typename = void>
	RayCastResult RayCast(Ray const& ray, ShapeTriangle const& shape)
	{
		RayCastResult result;

		// Adjust the ray to account for it's thickness
		auto r = impl::ShiftTowardOrigin(ray);

		v4 bary; float f2b;
		if (!geometry::intersect::RayVsTriangle(r.m_point, r.m_direction, 0, shape.m_v.x, shape.m_v.y, shape.m_v.z, f2b, bary))
			return result;

		auto intercept = geometry::BaryPoint(shape.m_v.x, shape.m_v.y, shape.m_v.z, bary);
		auto t = Sqrt(LengthSq(intercept - r.m_point) / LengthSq(r.m_direction));

		result.m_t0 = t;
		result.m_t1 = t;
		result.m_normal = f2b * shape.m_v.w;
		result.m_shape = &shape.m_base;
		return result;
	}

	// Ray vs. Polytope
	template <typename = void>
	RayCastResult RayCast(Ray const& ray, ShapePolytope const& shape)
	{
		(void)ray;
		(void)shape;
		#if 0 // Should be basically right, just needs testing
		#ifndef PR_COL_DBG
		#define PR_COL_DBG 0
		#endif

		// Local struct for debugging functions, optimiser should remove these calls
		struct Dbg
		{
			static void Scene()
			{
				#if PR_COL_DBG
				// StartFile("C:/Deleteme/raycast_scene.pr_script");)
				// ldr::Polytope("shape", "FF00A000", shape.vert_begin(), shape.vert_end());)
				// ldr::Vector("ray", "A0FF0000", ray.m_point, ray.m_direction, 0.03f);)
				// EndFile();)
				// ray_cast::Simplex().Dump("raycast_smplx");)
				// StartFile("C:/DeleteMe/raycast_nearest.pr_script");EndFile();)
				// StartFile("C:/Deleteme/raycast_dir.pr_script");EndFile();)
				// StartFile("C:/Deleteme/raycast_result.pr_script");EndFile();)
				#endif
			}
		};
		struct Simplex
		{
			struct Vert { v4 m_vert; std::size_t m_id; void set(v4 vert, std::size_t id) {m_vert = vert; m_id = id;} };
			Simplex() : m_num_verts(0), m_intersects(false) {}
	
			// Add a vertex to the simplex. Returns false if it's a duplicate
			bool AddVertex(v4 vert, std::size_t id, v4 lineS, v4 lineE)
			{
				PR_ASSERT(PR_DBG_PHYSICS, m_num_verts < 4, "");
				for (std::size_t i = 0; i != m_num_verts; ++i)
					if (m_vert[i].m_id == id) return false;

				if (m_num_verts < 3)
				{
					m_vert[m_num_verts++].set(vert, id);
				}
				else
				{
					// If the line pierces a triangle that is internal to the polytope then
					// it must intersect the polytope. We should not have 3 verts in the simplex
					// unless the line pierces the triangle formed by these three verts.
					PR_ASSERT(PR_DBG_PHYSICS, m_intersects, "");

					// If the line pierced the existing simplex triangle then it must pierce one
					// of the three triangles created between this fourth vertex and the edges of the triangle
					// Find the triangle that the line penetrates and replace the simplex vertex that isn't needed
					int a = 0, b = 1;
					if (!PointInFrontOfPlane(vert, m_vert[0].m_vert, m_vert[1].m_vert, m_vert[2].m_vert))
					{
						a = 1; b = 0;
					}
					v4 bary; float f2b;
					for (int v = 0; v != 3; ++v)
					{
						int j = (v + a) % 3;
						int k = (v + b) % 3;
						if (geometry::intersect::LineVsTriangle(lineS, lineE, vert, m_vert[j].m_vert, m_vert[k].m_vert, f2b, bary))
						{
							// Remove the vert that isn't needed
							int i = (v + 2) % 3;
							m_vert[i].set(vert, id);

							// Calculate the nearest and normal since we have the barycoords
							m_nearest = BaryPoint(m_vert[i].m_vert, m_vert[j].m_vert, m_vert[k].m_vert, bary);
							m_normal = Cross(m_vert[j].m_vert - m_vert[i].m_vert, m_vert[k].m_vert - m_vert[j].m_vert);
							return true;
						}
					}
					PR_ASSERT(PR_DBG_PHYSICS, false, "Line does not intercept any triangles");
				}
				return true;
			}

			// Find the next direction to search in.
			v4 FindNearest(v4 lineS, v4 lineE, Simplex& save_for_back_facing)
			{
				PR_ASSERT(PR_DBG_PHYSICS, m_num_verts > 0, "");
				float t0, t1;
				switch( m_num_verts )
				{
				case 1: 
					m_nearest = m_vert[0].m_vert;
					m_normal  = ClosestPoint_PointToInfiniteLine(m_nearest, lineS, lineE, t0) - m_nearest;
					return m_normal;
				case 2:
					{
						v4 line = lineE - lineS;
						ClosestPoint_LineSegmentToRay(m_vert[0].m_vert, m_vert[1].m_vert, lineS, line, t0, t1);
						if( t0 == 1.0f )	{ m_vert[0] = m_vert[1]; t0 = 0.0f; } // careful here, using t0 to pass into the next if
						if( t0 == 0.0f )	{ m_nearest = m_vert[0].m_vert; m_num_verts = 1; }
						else				{ m_nearest = (1.0f - t0)*m_vert[0].m_vert + t0*m_vert[1].m_vert; }
						m_normal = Cross(m_vert[1].m_vert - m_vert[0].m_vert, line);
						m_normal *= (Dot3(lineS - m_vert[0].m_vert, m_normal) >= 0.0f) * 2.0f - 1.0f;
						//m_normal = lineS + t0*line - m_nearest; don't use, not as robust
					}return m_normal;
				case 3:
					{
						// The nearest and normal have already been calculated in 'AddVertex'
						if( m_intersects ) return m_normal;

						// If the line intersects the triangle then the closest point is the
						// intercept and the normal is the triangle normal (opposing the line)
						v4 bary; float f2b;
						m_intersects = Intersect_LineToTriangle(lineS, lineE, m_vert[0].m_vert, m_vert[1].m_vert, m_vert[2].m_vert, f2b, bary);
						if( m_intersects )
						{
							m_nearest = BaryPoint(m_vert[0].m_vert, m_vert[1].m_vert, m_vert[2].m_vert, bary);
							m_normal  = f2b * Cross(m_vert[1].m_vert - m_vert[0].m_vert, m_vert[2].m_vert - m_vert[1].m_vert);

							// Save a copy of the simplex when we detect that the line intersects the
							// simplex. This is an optimisation to jump start the back facing search
							save_for_back_facing = *this;
							save_for_back_facing.m_normal *= -1;
							return m_normal;
						}
						// Otherwise, reduce the simplex to the minimum verts needed to represent the nearest point
						else
						{
							// Find the closest point on the triangle to the line
							v4 line = lineE - lineS;
							Vert  closest[2];
							float closest_t = 0.0f;
							float closest_dist_sq = limits<float>::max();
							for( int i = 0; i != 3; ++i )
							{
								Vert const& vert0 = m_vert[ i     ];
								Vert const& vert1 = m_vert[(i+1)%3];
								float dist_sq;
								ClosestPoint_LineSegmentToInfiniteLine(vert0.m_vert, vert1.m_vert, lineS, line, t0, t1, dist_sq);
								if( dist_sq < closest_dist_sq )
								{
									closest_dist_sq = dist_sq;
									closest[0] = vert0;
									closest[1] = vert1;
									closest_t = t0;
								}
							}
							if     ( closest_t == 1.0f )	{ m_num_verts = 1; m_vert[0] = closest[1];						   m_nearest = m_vert[0].m_vert; }
							else if( closest_t == 0.0f )	{ m_num_verts = 1; m_vert[0] = closest[0];						   m_nearest = m_vert[0].m_vert; }
							else							{ m_num_verts = 2; m_vert[0] = closest[0]; m_vert[1] = closest[1]; m_nearest = (1.0f - closest_t)*m_vert[0].m_vert + closest_t*m_vert[1].m_vert; }
							m_normal  = ClosestPoint_PointToInfiniteLine(m_nearest, lineS, lineE, t0) - m_nearest;
							return m_normal;
						}
					}break;
				default:
					PR_ASSERT(PR_DBG_PHYSICS, false, "");
				}
				return m_normal;
			}

			// Dump the simplex to line drawer
			#if PR_PH_DBG_RAY_CAST == 1
			void Dump(char const* name)
			{
				name;
				PR_EXPAND(1, StartFile(Fmt("C:/Deleteme/raycast_%s.pr_script", name).c_str());)
				PR_EXPAND(1, ldr::GroupStart(name);)
				for( std::size_t i = 0; i != m_num_verts; ++i )
				{
					PR_EXPAND(1, ldr::Box ("pt"  , "FF00FF00", m_vert[i].m_vert, 0.05f);)
					PR_EXPAND(1, ldr::Line("line", "FF00FF00", m_vert[i].m_vert, m_vert[(i+1)%m_num_verts].m_vert);)
				}
				PR_EXPAND(1, ldr::GroupEnd();)
				PR_EXPAND(1, EndFile();)
			}
			#endif//PR_PH_DBG_RAY_CAST

			std::size_t m_num_verts;
			Vert		m_vert[3];
			v4			m_nearest;
			v4			m_normal;
			bool		m_intersects;
		};

		RayCastResult result;

		Dbg::Scene();
		assert(!FEql3(ray.m_direction, v4::Zero()));

		auto ray_      = impl::ShiftTowardOrigin(ray);
		auto lineS     = ray_.m_point;
		auto lineE     = ray_.m_point + ray_.m_direction;
		result.m_t0    = -limits<float>::max();
		result.m_t1    =  limits<float>::max();
		result.m_shape = &shape.m_base;

		float t;
		std::size_t id = 0;
		v4 start_vert;

		// Attempt a quick out for the line vs. polytope test. If the distance to the most extreme vert
		// in the direction from the centre of 'shape' to the line is less that the distance to the line
		// then the line cannot penetrate the polytope
		auto dir = ClosestPoint_PointToInfiniteLine(v4::Origin(), lineS, lineE, t) - v4::Origin();
		if (!FEql3(dir, v4::Zero()))
		{
			start_vert = SupportVertex(shape, dir, id, id);
				PR_EXPAND(PR_PH_DBG_RAY_CAST, StartFile("C:/Deleteme/raycast_vert.pr_script");)
				PR_EXPAND(PR_PH_DBG_RAY_CAST, ldr::Box("vert", "FFFFFF00", start_vert, 0.02f);)
				PR_EXPAND(PR_PH_DBG_RAY_CAST, EndFile();)
			if( Dot3(start_vert, dir) < Length3Sq(dir) ) return false;
		}
		else
		{
				dir = Perpendicular3(ray.m_direction);
				start_vert = SupportVertex(shape, dir, id, id);
				PR_EXPAND(PR_PH_DBG_RAY_CAST, StartFile("C:/Deleteme/raycast_vert.pr_script");)
				PR_EXPAND(PR_PH_DBG_RAY_CAST, ldr::Box("vert", "FFFFFF00", start_vert, 0.02f);)
				PR_EXPAND(PR_PH_DBG_RAY_CAST, EndFile();)
		}

		std::size_t k;
		ray_cast::Simplex frnt, back;

		// Initialise the simplex with the start vert
		frnt.AddVertex(start_vert, id, lineS, lineE);

		// Clip each end of the line
		for( int side = 0; side != 2; ++side )
		{
			v4& line_s					= side == 0 ? lineS : lineE;
			v4& line_e					= side == 0 ? lineE : lineS;
			ray_cast::Simplex& smplx	= side == 0 ? frnt : back;
				PR_EXPAND(PR_PH_DBG_RAY_CAST, smplx.Dump("raycast_smplx");)

			// Iteratively move the simplex towards the start of the line
			for( k = 0; k != 2 * shape.m_vert_count; ++k ) // It should never take this long
			{
				dir = smplx.FindNearest(line_s, line_e, back);
					PR_EXPAND(PR_PH_DBG_RAY_CAST, smplx.Dump("raycast_smplx");)
				if( FEql3(dir,pr::v4::Zero()) ) dir = line_s - line_e;
					PR_EXPAND(PR_PH_DBG_RAY_CAST, StartFile("C:/DeleteMe/raycast_nearest.pr_script");)
					PR_EXPAND(PR_PH_DBG_RAY_CAST, ldr::Box("Nearest", "FFFF0000", smplx.m_nearest, 0.02f);)
					PR_EXPAND(PR_PH_DBG_RAY_CAST, ldr::LineD("Sep_axis", "FFFFFF00", smplx.m_nearest, Normalise3(dir));)
					PR_EXPAND(PR_PH_DBG_RAY_CAST, EndFile();)
		
				v4 vert = SupportVertex(shape, dir, id, id);
					PR_EXPAND(PR_PH_DBG_RAY_CAST, StartFile("C:/Deleteme/raycast_vert.pr_script");)
					PR_EXPAND(PR_PH_DBG_RAY_CAST, ldr::Box("vert", "FFFFFF00", vert, 0.02f);)
					PR_EXPAND(PR_PH_DBG_RAY_CAST, EndFile();)

				// If the support vertex 'vert' is no more extreme in the direction of the
				// separating axis 'dir' than 'nearest' then the line does not intersect the
				// polytope and the distance is the distance from the nearest point to the line
				float v_dist = Dot3(dir, vert);
				float n_dist = Dot3(dir, smplx.m_nearest) + maths::tiny<float>;
				if( v_dist <= n_dist )
				{
					if( smplx.m_intersects ) break;
					else return false;
				}			
				if( !smplx.AddVertex(vert, id, line_s, line_e) )
				{
					if( smplx.m_intersects ) break;
					else return false;
				}
					PR_EXPAND(PR_PH_DBG_RAY_CAST, smplx.Dump("raycast_smplx");)
			}
			PR_ASSERT(PR_DBG_PHYSICS, k != 2 * shape.m_vert_count, "Infinite loop averted"); // It should never take this long

			if( side == 0 )
			{
				result.m_t0		= Dot3(smplx.m_nearest - lineS, ray.m_direction) / Length3Sq(ray.m_direction);
				result.m_normal = Normalise3(dir); // Save the front face normal
			}
			else
			{
				result.m_t1		= Dot3(smplx.m_nearest - lineS, ray.m_direction) / Length3Sq(ray.m_direction);
			}		
		}

		PR_ASSERT(PR_DBG_PHYSICS, result.m_t0 <= result.m_t1, "");

		PR_EXPAND(PR_PH_DBG_RAY_CAST, StartFile("C:/Deleteme/raycast_result.pr_script");)
		PR_EXPAND(PR_PH_DBG_RAY_CAST, ldr::Line("ray", "FF0000FF", ray.m_point + result.m_t0*ray.m_direction, ray.m_point + result.m_t1*ray.m_direction);)
		PR_EXPAND(PR_PH_DBG_RAY_CAST, if( result.m_t0 != 0.0f ) ldr::LineD("norm", "FF00FFFF", ray.m_point + result.m_t0*ray.m_direction, result.m_normal);)
		PR_EXPAND(PR_PH_DBG_RAY_CAST, EndFile();)
		PR_EXPAND(PR_PH_DBG_RAY_CAST,	ray_cast::Simplex().Dump("raycast_smplx");)
		PR_EXPAND(PR_PH_DBG_RAY_CAST,	StartFile("C:/DeleteMe/raycast_nearest.pr_script");EndFile();)
		PR_EXPAND(PR_PH_DBG_RAY_CAST,	StartFile("C:/Deleteme/raycast_dir.pr_script");EndFile();)
		return result;
		#endif

		#define PR_COL_POLYTOPE_BRUTEFORCE 0
		#if PR_COL_POLYTOPE_BRUTEFORCE
		{
			v4	lineS = ray.m_point;
			v4			lineE = ray.m_point + ray.m_direction;
			result.m_t0 = -limits<float>::max(); result.m_t1 = limits<float>::max();

			// Attempt a quick out for the line vs. shape test
			float t; std::size_t id = 0;
			v4 nearest = ClosestPoint_PointToLineSegment(v4::Origin(), lineS, lineE, t) - v4::Origin();
			if( !FEql3(nearest,pr::v4::Zero()) )
			{
				v4 support = SupportVertex(shape, nearest, id, id);
				if( Dot3(support, nearest) < Length3Sq(nearest) ) return false;
			}

			// Clip the line segment against each face of the polytope
			for( ShapePolyFace const* f = shape.face_begin(), *f_end = shape.face_end(); f != f_end; ++f )
			{
				ShapePolyFace const& face = *f;

				v4 a = shape.vertex(face.m_index[0]);
				v4 b = shape.vertex(face.m_index[1]);
				v4 c = shape.vertex(face.m_index[2]);
				v4 plane = plane::make(a, c, b);	// Outward facing plane
				float t_min = result.m_t0;
				if( !Intersect_LineSegmentToPlane(plane, lineS, lineE, result.m_t0, result.m_t1) )
					return false;
		
				// Record the plane normal of the last plane to clip the line
				if( result.m_t0 > t_min )
				{
					result.m_normal = -plane;
					result.m_normal.w = 0.0f;
				}
			}
			return true;
		}
		#endif

		throw std::runtime_error("Not implemented");
	}

	// Ray vs. Array
	template <typename = void>
	RayCastResult RayCast(Ray const& ray, ShapeArray const& shape)
	{
		RayCastResult result;
		result.m_t0 = 1.0f;

		for (Shape const *s = shape.begin(), *s_end = shape.end(); s != s_end; s = next(s))
		{
			// Transform the ray into shape space and call recursively
			auto res = RayCast(InvertAffine(s->m_s2p) * ray, *s);
			if (res.m_shape != nullptr && res.m_t0 < result.m_t0)
			{
				// Record the nearest intersect
				result = res;
				result.m_normal = s->m_s2p * result.m_normal;
			}
		}
		return result;
	}

	// Return the intercept of a ray vs. a shape. The ray must be in shape space.
	inline RayCastResult RayCast(Ray const& ray, Shape const& shape)
	{
		switch (shape.m_type)
		{
			#define PR_COLLISION_SHAPE_RAYCAST(name, comp) case EShape::name: return RayCast(ray, shape_cast<Shape##name>(shape));
			PR_COLLISION_SHAPES(PR_COLLISION_SHAPE_RAYCAST)
			#undef PR_COLLISION_SHAPE_RAYCAST
			default: assert("Unknown primitive type" && false); return RayCastResult{};
		}
	}

	// Cast a world space ray
	template <ShapeType TShape>
	inline RayCastResult RayCastWS(Ray const& ray, TShape const& shape, m4x4 const& s2w)
	{
		// Transform the ray cast into shape space
		auto result = RayCast(InvertAffine(s2w) * ray, shape);

		// Transform the result back to world space
		if (result.m_shape != nullptr)
			result.m_normal = s2w * result.m_normal;
			
		return result;
	}
}
