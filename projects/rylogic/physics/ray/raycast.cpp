//*********************************************
// Physics engine
//  Copyright (c) Rylogic Ltd 2006
//*********************************************

#include "physics/utility/stdafx.h"
#include "pr/Physics/Ray/RayCast.h"
#include "pr/Physics/Shape/Shapes.h"

#define PR_PH_DBG_RAY_CAST 0
#if PR_PH_DBG_RAY_CAST == 1
#pragma message(__FILE__ "(10) : PR_PH_DBG_RAY_CAST is defined")
#endif//PR_PH_DBG_RAY_CAST 1

using namespace pr;
using namespace pr::ph;
using namespace pr::geometry;

// Shift the ray toward the centre of a shape to simulate the ray having a thickness
// The ray is in shape space so we're shifting it toward the origin
inline Ray ShiftRay(Ray const& ray)
{
	if( ray.m_thickness == 0.0f ) return ray;
	float direction_len = Length(ray.m_direction);
	v4 forward			= ray.m_direction / direction_len;
	v4 sideways			= forward * Dot3(ray.m_point, forward) - ray.m_point; sideways.w = 0.0f;
	float sideways_len  = Length(sideways);
	sideways			/= sideways_len;
	return Ray(ray.m_point +
		forward  * pr::Min(direction_len, ray.m_thickness) +
		sideways * pr::Min(sideways_len , ray.m_thickness),
		ray.m_direction);
}

// Return the intercept of a ray vs. a shape
bool pr::ph::RayCast(Ray const& ray, Shape const& shape, RayCastResult& result)
{
	switch( shape.m_type )
	{
	case EShape_Sphere:		return RayCast(ray, shape_cast<ShapeSphere  >(shape), result);
	case EShape_Box:		return RayCast(ray, shape_cast<ShapeBox     >(shape), result);
	case EShape_Cylinder:	return RayCast(ray, shape_cast<ShapeCylinder>(shape), result);
	case EShape_Polytope:	return RayCast(ray, shape_cast<ShapePolytope>(shape), result);
	case EShape_Triangle:	return RayCast(ray, shape_cast<ShapeTriangle>(shape), result);
	case EShape_Array:		return RayCast(ray, shape_cast<ShapeArray   >(shape), result);
	default: PR_ASSERT(PR_DBG_PHYSICS, false, "Unsupported shape type");
	}
	return false;
}

// Ray vs. Sphere
bool pr::ph::RayCast(Ray const& ray, ShapeSphere const& shape, RayCastResult& result)
{
	// Find the closest point to the line
	float	direction_lenSq = LengthSq(ray.m_direction);
	v4		closest_point	= ray.m_point - ray.m_direction * (Dot3(ray.m_direction, ray.m_point) / direction_lenSq);
	float	closest_distSq	= LengthSq(closest_point);
	float	radius			= shape.m_radius + ray.m_thickness;
	float	radiusSq		= radius * radius;

	// If the closest point is not within the sphere then there is no intersection
	if( radiusSq < closest_distSq ) return false;

	// Get the distance from the closest point to the intersection with the boundary of the sphere
	float x = Sqrt((radiusSq - closest_distSq) / direction_lenSq);

	// Get the parametric values and normal
	v4 offset		= ray.m_direction * x;
	v4 lstart		= closest_point - offset;
	v4 lend			= closest_point + offset;
	result.m_t0		= Dot3(ray.m_direction, lstart - ray.m_point) / direction_lenSq;
	result.m_t1		= Dot3(ray.m_direction, lend   - ray.m_point) / direction_lenSq;
	result.m_normal	= lstart / radius; result.m_normal.w = 0.0f;
	result.m_shape	= &shape.m_base;
	return true;
}

// Ray vs. Box
bool pr::ph::RayCast(Ray const& ray, ShapeBox const& shape, RayCastResult& result)
{
	// For each side of the box
	result.m_t0 = 0.0f;
	result.m_t1 = 1.0f;
	result.m_shape = &shape.m_base;
	for( int i = 0; i != 3; ++i )
	{
		if( Abs(ray.m_direction[i]) < maths::tiny<float> )
		{
			if( Abs(ray.m_point[i]) > shape.m_radius[i] ) return false;
		}
		else
		{
			// Compute the intersection 't' value of ray with near and far plane
			float ta = (-(shape.m_radius[i] + ray.m_thickness) - ray.m_point[i]) / ray.m_direction[i];
			float tb = ( (shape.m_radius[i] + ray.m_thickness) - ray.m_point[i]) / ray.m_direction[i];

			// Make ta the intersection with the near plane, tb the intersection with the far plane
			float sign = -1.0f;
			if( ta > tb ) { float swp = ta; ta = tb; tb = swp; sign = 1.0f; }

			// Compute the intersection 
			if( ta > result.m_t0 ) { result.m_t0 = ta; result.m_normal = v4((i == 0) * sign, (i == 1) * sign, (i == 2) * sign, 0.0f); }
			if( tb < result.m_t1 ) { result.m_t1 = tb; }
			if( result.m_t0 > result.m_t1 ) return false;
		}
	}
	return true;
}

// Ray vs. Cylinder
bool pr::ph::RayCast(Ray const& ray, ShapeCylinder const& shape, RayCastResult& result)
{ray;shape;result;
	// TODO
	return false;
}

// Ray vs. Polytope
namespace pr { namespace ph { namespace ray_cast {
struct Simplex
{
	struct Vert { v4 m_vert; std::size_t m_id; void set(v4 vert, std::size_t id) {m_vert = vert; m_id = id;} };
	Simplex() : m_num_verts(0), m_intersects(false) {}
	
	// Add a vertex to the simplex. Returns false if it's a duplicate
	bool AddVertex(v4 vert, std::size_t id, v4 lineS, v4 lineE)
	{
		PR_ASSERT(PR_DBG_PHYSICS, m_num_verts < 4, "");
		for( std::size_t i = 0; i != m_num_verts; ++i )
			if( m_vert[i].m_id == id ) return false;

		if( m_num_verts < 3 )
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
			if( !PointInFrontOfPlane(vert, m_vert[0].m_vert, m_vert[1].m_vert, m_vert[2].m_vert) )
			{	a = 1; b = 0; }
			v4 bary; float f2b;
			for( int v = 0; v != 3; ++v )
			{
				int j = (v + a) % 3;
				int k = (v + b) % 3;
				if( intersect::RayVsTriangle(lineS, lineE - lineS, 0, vert, m_vert[j].m_vert, m_vert[k].m_vert, f2b, bary) )
				{
					// Remove the vert that isn't needed
					int i = (v + 2) % 3;
					m_vert[i].set(vert, id);

					// Calculate the nearest and normal since we have the barycoords
					m_nearest = BaryPoint(m_vert[i].m_vert, m_vert[j].m_vert, m_vert[k].m_vert, bary);
					m_normal  = Cross(m_vert[j].m_vert - m_vert[i].m_vert, m_vert[k].m_vert - m_vert[j].m_vert);
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
		switch (m_num_verts)
		{
			case 1:
				m_nearest = m_vert[0].m_vert;
				m_normal = closest_point::PointToRay(m_nearest, lineS, lineE - lineS, 0, t0) - m_nearest;
				return m_normal;
			case 2:
			{
				v4 line = lineE - lineS;
				closest_point::LineToRay(m_vert[0].m_vert, m_vert[1].m_vert, lineS, line, t0, t1);
				if (t0 == 1.0f) { m_vert[0] = m_vert[1]; t0 = 0.0f; } // careful here, using t0 to pass into the next if
				if (t0 == 0.0f) { m_nearest = m_vert[0].m_vert; m_num_verts = 1; }
				else { m_nearest = (1.0f - t0) * m_vert[0].m_vert + t0 * m_vert[1].m_vert; }
				m_normal = Cross(m_vert[1].m_vert - m_vert[0].m_vert, line);
				m_normal *= (Dot3(lineS - m_vert[0].m_vert, m_normal) >= 0.0f) * 2.0f - 1.0f;
				//m_normal = lineS + t0*line - m_nearest; don't use, not as robust
			}return m_normal;
			case 3:
			{
				// The nearest and normal have already been calculated in 'AddVertex'
				if (m_intersects) return m_normal;

				// If the line intersects the triangle then the closest point is the
				// intercept and the normal is the triangle normal (opposing the line)
				v4 bary; float f2b;
				m_intersects = intersect::RayVsTriangle(lineS, lineE - lineS, 0, m_vert[0].m_vert, m_vert[1].m_vert, m_vert[2].m_vert, f2b, bary);
				if (m_intersects)
				{
					m_nearest = BaryPoint(m_vert[0].m_vert, m_vert[1].m_vert, m_vert[2].m_vert, bary);
					m_normal = f2b * Cross(m_vert[1].m_vert - m_vert[0].m_vert, m_vert[2].m_vert - m_vert[1].m_vert);

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
					for (int i = 0; i != 3; ++i)
					{
						Vert const& vert0 = m_vert[i];
						Vert const& vert1 = m_vert[(i + 1) % 3];
						float dist_sq;
						closest_point::LineToRay(vert0.m_vert, vert1.m_vert, lineS, line, t0, t1, dist_sq);
						if (dist_sq < closest_dist_sq)
						{
							closest_dist_sq = dist_sq;
							closest[0] = vert0;
							closest[1] = vert1;
							closest_t = t0;
						}
					}
					if (closest_t == 1.0f)
					{
						m_num_verts = 1;
						m_vert[0] = closest[1];
						m_nearest = m_vert[0].m_vert;
					}
					else if (closest_t == 0.0f)
					{
						m_num_verts = 1;
						m_vert[0] = closest[0];
						m_nearest = m_vert[0].m_vert;
					}
					else
					{
						m_num_verts = 2;
						m_vert[0] = closest[0];
						m_vert[1] = closest[1];
						m_nearest = (1.0f - closest_t) * m_vert[0].m_vert + closest_t * m_vert[1].m_vert;
					}
					m_normal = closest_point::PointToRay(m_nearest, lineS, lineE - lineS, 0, t0) - m_nearest;
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
}}}//namespace pr
bool pr::ph::RayCast(Ray const& ray, ShapePolytope const& shape, RayCastResult& result)
{
	PR_EXPAND(PR_PH_DBG_RAY_CAST, StartFile("C:/Deleteme/raycast_scene.pr_script");)
	PR_EXPAND(PR_PH_DBG_RAY_CAST, ldr::Polytope("shape", "FF00A000", shape.vert_begin(), shape.vert_end());)
	PR_EXPAND(PR_PH_DBG_RAY_CAST, ldr::Vector("ray", "A0FF0000", ray.m_point, ray.m_direction, 0.03f);)
	PR_EXPAND(PR_PH_DBG_RAY_CAST, EndFile();)
	PR_EXPAND(PR_PH_DBG_RAY_CAST,	ray_cast::Simplex().Dump("raycast_smplx");)
	PR_EXPAND(PR_PH_DBG_RAY_CAST,	StartFile("C:/DeleteMe/raycast_nearest.pr_script");EndFile();)
	PR_EXPAND(PR_PH_DBG_RAY_CAST,	StartFile("C:/Deleteme/raycast_dir.pr_script");EndFile();)
	PR_EXPAND(PR_PH_DBG_RAY_CAST,	StartFile("C:/Deleteme/raycast_result.pr_script");EndFile();)
	
	PR_ASSERT(PR_DBG_PHYSICS, !FEql(ray.m_direction,pr::v4::Zero()), "");

	Ray ray_ = ShiftRay(ray);
	v4 lineS = ray_.m_point;
	v4 lineE = ray_.m_point + ray_.m_direction;
	result.m_t0 = -limits<float>::max();
	result.m_t1 =  limits<float>::max();
	result.m_shape = &shape.m_base;

	float t;
	std::size_t id = 0;
	v4 start_vert;

	// Attempt a quick out for the line vs. polyyope test. If the distance to the most extreme vert
	// in the direction from the centre of 'shape' to the line is less that the distance to the line
	// then the line cannot penetrate the polytope
	v4 dir = closest_point::PointToRay(v4::Origin(), lineS, lineE - lineS, 0, t) - v4::Origin();
	if( !FEql(dir,pr::v4::Zero()) )
	{
		start_vert = SupportVertex(shape, dir, id, id);
			PR_EXPAND(PR_PH_DBG_RAY_CAST, StartFile("C:/Deleteme/raycast_vert.pr_script");)
			PR_EXPAND(PR_PH_DBG_RAY_CAST, ldr::Box("vert", "FFFFFF00", start_vert, 0.02f);)
			PR_EXPAND(PR_PH_DBG_RAY_CAST, EndFile();)
		if( Dot3(start_vert, dir) < LengthSq(dir) ) return false;
	}
	else
	{
		 dir = Perpendicular(ray.m_direction);
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
			if( FEql(dir,pr::v4::Zero()) ) dir = line_s - line_e;
				PR_EXPAND(PR_PH_DBG_RAY_CAST, StartFile("C:/DeleteMe/raycast_nearest.pr_script");)
				PR_EXPAND(PR_PH_DBG_RAY_CAST, ldr::Box("Nearest", "FFFF0000", smplx.m_nearest, 0.02f);)
				PR_EXPAND(PR_PH_DBG_RAY_CAST, ldr::LineD("Sep_axis", "FFFFFF00", smplx.m_nearest, Normalise(dir));)
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
			result.m_t0		= Dot3(smplx.m_nearest - lineS, ray.m_direction) / LengthSq(ray.m_direction);
			result.m_normal = Normalise(dir); // Save the front face normal
		}
		else
		{
			result.m_t1		= Dot3(smplx.m_nearest - lineS, ray.m_direction) / LengthSq(ray.m_direction);
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
	return true;
}

// Ray vs. Triangle
bool pr::ph::RayCast(Ray const& ray, ShapeTriangle const& shape, RayCastResult& result)
{
	Ray r = ShiftRay(ray);
	v4 bary; float f2b;
	if( !intersect::RayVsTriangle(r.m_point, r.m_direction, 0, shape.m_v.x, shape.m_v.y, shape.m_v.z, f2b, bary) )
		return false;

	v4 intercept = BaryPoint(shape.m_v.x, shape.m_v.y, shape.m_v.z, bary);
	float t = Sqrt(LengthSq(intercept - r.m_point) / LengthSq(r.m_direction));

	result.m_t0		= t;
	result.m_t1		= t;
	result.m_normal	= f2b * shape.m_v.w;
	result.m_shape	= &shape.m_base;
	return true;
}

// Ray vs. Array
bool pr::ph::RayCast(Ray const& ray, ShapeArray const& shape, RayCastResult& result)
{
	result.m_t0 = 1.0f;

	RayCastResult res;
	for( Shape const *s = shape.begin(), *s_end = shape.end(); s != s_end; s = Inc(s) )
	{
		if( RayCast(InvertAffine(s->m_shape_to_model) * ray, *s, res) && res.m_t0 < result.m_t0 )
		{
			result = res;
			result.m_normal = s->m_shape_to_model * result.m_normal;
		}
	}
	return result.m_t0 != 1.0f;
}

#if PR_DBG_PHYSICS == 1
bool pr::ph::RayCastBruteForce(Ray const& ray, ShapePolytope const& shape, RayCastResult& result)
{
	v4	lineS = ray.m_point;
	v4			lineE = ray.m_point + ray.m_direction;
	result.m_t0 = -limits<float>::max(); result.m_t1 = limits<float>::max();

	// Attempt a quick out for the line vs. shape test
	float t; std::size_t id = 0;
	v4 nearest = closest_point::PointToLine(v4::Origin(), lineS, lineE, t) - v4::Origin();
	if( !FEql(nearest,pr::v4::Zero()) )
	{
		v4 support = SupportVertex(shape, nearest, id, id);
		if( Dot3(support, nearest) < LengthSq(nearest) ) return false;
	}

	// Clip the line segment against each face of the polytope
	for (ShapePolyFace const* f = shape.face_begin(), *f_end = shape.face_end(); f != f_end; ++f)
	{
		ShapePolyFace const& face = *f;

		auto a = shape.vertex(face.m_index[0]);
		auto b = shape.vertex(face.m_index[1]);
		auto c = shape.vertex(face.m_index[2]);
		auto plane = plane::make(a, c, b);	// Outward facing plane
		float t_min = result.m_t0;
		if (!intersect::LineVsPlane(plane, lineS, lineE, result.m_t0, result.m_t1))
			return false;

		// Record the plane normal of the last plane to clip the line
		if (result.m_t0 > t_min)
			result.m_normal = -static_cast<v4>(plane.w0());
	}
	return true;
}
#endif//PR_DBG_PHYSICS == 1

