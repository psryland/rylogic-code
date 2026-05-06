//*********************************************
// Collision
//  Copyright (c) Rylogic Ltd 2026
//*********************************************
// GJK + EPA collision detection for arbitrary convex shape pairs.
//
// The Gilbert-Johnson-Keerthi (GJK) algorithm determines whether two convex shapes
// overlap by searching for the origin inside their Minkowski difference. If shapes
// overlap, the Expanding Polytope Algorithm (EPA) computes the penetration depth,
// contact normal, and contact point.
//
// This implementation works with any convex shape type through the generic
// SupportVertex(Shape const&, ...) dispatch in shape.h.
//
// Primary use: Polytope vs any shape (fills the tri-table Polytope entries).
// Can also serve as a generic fallback for any convex pair.
//
#pragma once
#include "pr/collision/forward.h"
#include "pr/collision/shape.h"
#include "pr/collision/support.h"
#include "pr/collision/penetration.h"

namespace pr::collision
{
	namespace gjk
	{
		constexpr int MaxIter = 64;        // GJK/EPA iteration limit
		constexpr int MaxEpaVerts = 64;    // EPA vertex buffer size
		constexpr int MaxEpaFaces = 128;   // EPA face buffer size
		constexpr float Eps = 1e-8f;       // Near-zero threshold for directions
		constexpr float EpaEps = 1e-6f;    // EPA convergence tolerance

		// A support point in the Minkowski difference, tracking the original shape vertices
		struct Sup
		{
			v4 w; // Minkowski difference point (a - b), w=0
			v4 a; // Support vertex on shape A (world space), w=1
			v4 b; // Support vertex on shape B (world space), w=1
		};

		// Compute the Minkowski difference support point: S(d) = sup_A(d) - sup_B(-d)
		// 'ha'/'hb' are hint vertex indices for hill-climbing (warm-started across iterations).
		inline Sup MkSupport(
			Shape const& sa, m4x4 const& a2w, m4x4 const& w2a,
			Shape const& sb, m4x4 const& b2w, m4x4 const& w2b,
			v4 dir, int& ha, int& hb)
		{
			int ia, ib;
			auto va = (a2w * SupportVertex(sa, w2a * dir, ha, ia)).w1();
			auto vb = (b2w * SupportVertex(sb, w2b * (-dir), hb, ib)).w1();
			ha = ia;
			hb = ib;
			return { (va - vb).w0(), va, vb };
		}

		// Specialised Minkowski support when shape B is a single point (e.g. sphere centre).
		// The support of a point in any direction is just the point itself, so no support
		// query is needed for B.
		inline Sup MkSupportPoint(
			Shape const& sa, m4x4 const& a2w, m4x4 const& w2a,
			v4 point_b, v4 dir, int& ha)
		{
			int ia;
			auto va = (a2w * SupportVertex(sa, w2a * dir, ha, ia)).w1();
			ha = ia;
			return { (va - point_b).w0(), va, point_b.w1() };
		}

		// Returns the centroid (in shape space) of the shape's support face
		// in the given direction. For face-on contact the support is a polygon (multiple
		// vertices tied for max dot); the centroid of that polygon is a stable, geometrically
		// meaningful contact-point candidate. For non-degenerate contact, only one vertex
		// ties and this reduces to the standard support point.
		inline v4 pr_vectorcall SupportFaceCentre(Shape const& shape, v4 direction)
		{
			// Use SupportFeature for shapes that have it (Sphere/Box/Line/Triangle): it returns
			// up to FeaturePolygonMaxSides vertices of the support face — average them.
			switch (shape.m_type)
			{
				case EShape::Sphere:
				case EShape::Box:
				case EShape::Line:
				case EShape::Triangle:
				{
					EFeature ft = EFeature::Vert;
					v4 pts[FeaturePolygonMaxSides];
					switch (shape.m_type)
					{
						case EShape::Sphere:   SupportFeature(shape_cast<ShapeSphere>(shape),   direction, ft, pts); break;
						case EShape::Box:      SupportFeature(shape_cast<ShapeBox>(shape),      direction, ft, pts); break;
						case EShape::Line:     SupportFeature(shape_cast<ShapeLine>(shape),     direction, ft, pts); break;
						case EShape::Triangle: SupportFeature(shape_cast<ShapeTriangle>(shape), direction, ft, pts); break;
						default: break;
					}
					int n = int(ft);
					if (n <= 0) return v4::Origin();
					v4 sum = v4::Zero();
					for (int i = 0; i != n; ++i) sum += pts[i];
					return (sum / static_cast<float>(n)).w1();
				}
				case EShape::Polytope:
				{
					constexpr float TieEps = 1e-4f;
					auto& poly = shape_cast<ShapePolytope>(shape);
					if (poly.m_vert_count == 0) return v4::Origin();
					float best_dot = Dot3(direction, poly.vertex(0));
					for (int i = 1; i != poly.m_vert_count; ++i)
					{
						auto d = Dot3(direction, poly.vertex(i));
						if (d > best_dot) best_dot = d;
					}
					v4 sum = v4::Zero();
					int count = 0;
					for (int i = 0; i != poly.m_vert_count; ++i)
					{
						auto d = Dot3(direction, poly.vertex(i));
						if (d >= best_dot - TieEps)
						{
							sum += poly.vertex(i);
							++count;
						}
					}
					return count > 0 ? (sum / static_cast<float>(count)).w1() : v4::Origin();
				}
				default:
				{
					int dummy_id;
					return SupportVertex(shape, direction, 0, dummy_id).w1();
				}
			}
		}

		// GJK simplex: up to 4 points (line, triangle, tetrahedron).
		// The newest point is always at index 0.
		struct Simplex
		{
			Sup s[4];
			int n = 0;

			void Push(Sup const& p)
			{
				for (int i = n; i > 0; --i)
					s[i] = s[i - 1];
				s[0] = p;
				++n;
			}
		};

		// ---- Simplex cases ----
		// Each function updates the simplex to the closest feature to the origin,
		// sets the new search direction, and returns true if the origin is enclosed.

		inline bool SimplexLine(Simplex& sx, v4& dir);
		inline bool SimplexTri(Simplex& sx, v4& dir);
		inline bool SimplexLine(Simplex& sx, v4& dir)
		{
			// A = newest (sx.s[0]), B = previous (sx.s[1])
			auto ab = sx.s[1].w - sx.s[0].w;
			auto ao = -sx.s[0].w;

			if (Dot3(ab, ao) > 0)
			{
				// Origin projects onto the segment interior.
				// Search perpendicular to AB toward the origin (triple cross product).
				dir = Cross(Cross(ab, ao), ab);
				if (LengthSq(dir) < Eps)
					dir = Perpendicular(ab);
			}
			else
			{
				// Origin is past A. Reduce to point A.
				sx.n = 1;
				dir = ao;
			}
			return false;
		}
		inline bool SimplexTri(Simplex& sx, v4& dir)
		{
			// A = newest (sx.s[0]), B = sx.s[1], C = sx.s[2]
			auto ab = sx.s[1].w - sx.s[0].w;
			auto ac = sx.s[2].w - sx.s[0].w;
			auto ao = -sx.s[0].w;
			auto n = Cross(ab, ac); // triangle normal

			// Test Voronoi regions of the triangle edges
			if (Dot3(Cross(n, ac), ao) > 0)
			{
				// Origin is outside edge AC
				if (Dot3(ac, ao) > 0)
				{
					// Closest to edge AC
					sx.s[1] = sx.s[2];
					sx.n = 2;
					dir = Cross(Cross(ac, ao), ac);
					if (LengthSq(dir) < Eps)
						dir = Perpendicular(ac);
				}
				else
				{
					// Fall through to AB edge check
					sx.n = 2;
					return SimplexLine(sx, dir);
				}
			}
			else if (Dot3(Cross(ab, n), ao) > 0)
			{
				// Origin is outside edge AB
				sx.n = 2;
				return SimplexLine(sx, dir);
			}
			else
			{
				// Origin is inside the triangle prism — pick the correct face
				if (Dot3(n, ao) > 0)
				{
					dir = n; // above the triangle
				}
				else
				{
					std::swap(sx.s[1], sx.s[2]); // flip winding
					dir = -n; // below the triangle
				}
			}
			return false;
		}
		inline bool SimplexTetra(Simplex& sx, v4& dir)
		{
			// A = newest (sx.s[0]), B = sx.s[1], C = sx.s[2], D = sx.s[3]
			auto ab = sx.s[1].w - sx.s[0].w;
			auto ac = sx.s[2].w - sx.s[0].w;
			auto ad = sx.s[3].w - sx.s[0].w;
			auto ao = -sx.s[0].w;

			// Face normals, oriented outward (away from the opposite vertex)
			auto abc = Cross(ab, ac); if (Dot3(abc, ad) > 0) abc = -abc;
			auto acd = Cross(ac, ad); if (Dot3(acd, ab) > 0) acd = -acd;
			auto adb = Cross(ad, ab); if (Dot3(adb, ac) > 0) adb = -adb;

			// Check if origin is outside any face
			if (Dot3(abc, ao) > 0)
			{
				sx.n = 3; // reduce to triangle ABC
				return SimplexTri(sx, dir);
			}
			if (Dot3(acd, ao) > 0)
			{
				sx.s[1] = sx.s[2]; sx.s[2] = sx.s[3]; sx.n = 3; // triangle ACD
				return SimplexTri(sx, dir);
			}
			if (Dot3(adb, ao) > 0)
			{
				auto tmp = sx.s[1]; sx.s[1] = sx.s[3]; sx.s[2] = tmp; sx.n = 3; // triangle ADB
				return SimplexTri(sx, dir);
			}

			// Origin is inside the tetrahedron
			return true;
		}
		inline bool DoSimplex(Simplex& sx, v4& dir)
		{
			switch (sx.n)
			{
				case 2: return SimplexLine(sx, dir);
				case 3: return SimplexTri(sx, dir);
				case 4: return SimplexTetra(sx, dir);
			}
			return false;
		}

		// Project the origin onto the closest point of a triangle in Minkowski space.
		// Uses Voronoi-region barycentric projection (Christer Ericke's "Real-Time Collision
		// Detection" §5.1.5). Robust to degenerate triangles — falls through to a vertex
		// or edge if the triangle is near-degenerate.
		inline v4 ClosestPointToTriangle(v4 p, v4 a, v4 b, v4 c)
		{
			auto ab = b - a, ac = c - a, ap = p - a;
			auto d1 = Dot3(ab, ap), d2 = Dot3(ac, ap);
			if (d1 <= 0 && d2 <= 0)
				return a;

			auto bp = p - b;
			auto d3 = Dot3(ab, bp), d4 = Dot3(ac, bp);
			if (d3 >= 0 && d4 <= d3)
				return b;

			auto cp = p - c;
			auto d5 = Dot3(ab, cp), d6 = Dot3(ac, cp);
			if (d6 >= 0 && d5 <= d6)
				return c;

			auto vc = d1 * d4 - d3 * d2;
			if (vc <= 0 && d1 >= 0 && d3 <= 0)
			{
				auto v = d1 / (d1 - d3);
				return a + v * ab;
			}

			auto vb = d5 * d2 - d1 * d6;
			if (vb <= 0 && d2 >= 0 && d6 <= 0)
			{
				auto w = d2 / (d2 - d6);
				return a + w * ac;
			}

			auto va = d3 * d6 - d5 * d4;
			if (va <= 0 && (d4 - d3) >= 0 && (d5 - d6) >= 0)
			{
				auto w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
				return b + w * (c - b);
			}

			auto denom = 1.0f / (va + vb + vc);
			auto v = vb * denom;
			auto w = vc * denom;
			return a + ab * v + ac * w;
		}

		// Project the origin onto the (Voronoi-reduced) simplex, returning the closest
		// point in Minkowski space. Caller must have just run DoSimplex so the simplex
		// has been pruned to the closest sub-feature; this means the origin's projection
		// onto that sub-feature lies inside it (up to FP noise). Robust to degenerate
		// simplices: line/triangle helpers fall back to a vertex on degeneracy.
		inline v4 SimplexClosestPoint(Simplex const& sx)
		{
			if (sx.n == 1)
				return sx.s[0].w;

			if (sx.n == 2)
			{
				auto a = sx.s[0].w, b = sx.s[1].w;
				auto ab = b - a;
				auto ab_sq = Dot3(ab, ab);
				if (ab_sq < Eps)
					return a;
				auto t = std::clamp(-Dot3(a, ab) / ab_sq, 0.0f, 1.0f);
				return a + t * ab;
			}

			if (sx.n == 3)
				return ClosestPointToTriangle(v4::Origin().w0(), sx.s[0].w, sx.s[1].w, sx.s[2].w).w0();

			// Tetrahedron — origin is enclosed if DoSimplex returned true.
			return v4::Zero();
		}

		// ---- EPA (Expanding Polytope Algorithm) ----

		struct Face
		{
			int i[3];    // Vertex indices
			v4 normal;   // Outward-facing unit normal
			float dist;  // Distance from origin to face plane
		};

		// Find penetration depth and contact info from a GJK simplex containing the origin.
		// Epa can fail to converge, in that case it's better to return no result than a bad contact.
		// Hopefully, next frame, a better contact will be found
		inline bool Epa(
			Shape const& sa, m4x4 const& a2w, m4x4 const& w2a,
			Shape const& sb, m4x4 const& b2w, m4x4 const& w2b,
			Simplex const& gjk_sx, int& ha, int& hb,
			v4& normal, float& depth)
		{
			if (gjk_sx.n < 4) return false;
			auto fail_on_exhaustion = sa.m_type != EShape::Sphere && sb.m_type != EShape::Sphere;

			Sup verts[MaxEpaVerts];
			Face faces[MaxEpaFaces];
			int nv = 4, nf = 0;
			for (int i = 0; i < 4; ++i)
				verts[i] = gjk_sx.s[i];

			// Ensure consistent tetrahedron winding.
			// If Cross(v1-v0, v2-v0) points toward v3, the winding is inverted.
			auto n012 = Cross(verts[1].w - verts[0].w, verts[2].w - verts[0].w);
			if (Dot3(n012, verts[3].w - verts[0].w) > 0)
				std::swap(verts[0], verts[1]);

			// Helper: add a face with validated outward normal
			auto add_face = [&](int a, int b, int c) -> bool
			{
				if (nf >= MaxEpaFaces) return false;
				auto ab = verts[b].w - verts[a].w;
				auto ac = verts[c].w - verts[a].w;
				auto fn = Cross(ab, ac);
				auto len = Length(fn);
				if (len < Eps) return false;
				fn /= len;
				auto d = Dot3(fn, verts[a].w);

				// Ensure normal points outward (away from origin)
				if (d < 0)
				{
					fn = -fn;
					d = -d;
					std::swap(b, c);
				}
				faces[nf++] = { {a, b, c}, fn, d };
				return true;
			};

			// Build initial tetrahedron faces
			add_face(0, 1, 2);
			add_face(0, 3, 1);
			add_face(0, 2, 3);
			add_face(1, 3, 2);

			struct Edge { int a, b; };

			for (int iter = 0; iter < MaxIter; ++iter)
			{
				// Find the face closest to the origin
				int ci = 0;
				for (int i = 1; i < nf; ++i)
					if (faces[i].dist < faces[ci].dist)
						ci = i;

				auto cf_normal = faces[ci].normal;
				auto cf_dist = faces[ci].dist;

				// Get new support in the closest face's normal direction
				auto sup = MkSupport(sa, a2w, w2a, sb, b2w, w2b, cf_normal, ha, hb);
				auto d = Dot3(sup.w, cf_normal);

				// Convergence: new support doesn't extend the polytope significantly
				if (d - cf_dist < EpaEps)
				{
					normal = cf_normal;
					depth = cf_dist;
					return true;
				}
				if (nv >= MaxEpaVerts)
				{
					if (fail_on_exhaustion)
						return false;

					normal = cf_normal;
					depth = cf_dist;
					return true;
				}

				// Add new vertex
				verts[nv] = sup;
				auto ni = nv++;

				// Remove faces visible from the new point, collecting horizon edges.
				// Shared edges (appearing in opposite directions) cancel — the remainder
				// forms the horizon boundary around the visible region.
				Edge edges[MaxEpaFaces * 3];
				int ne = 0;
				for (int i = nf - 1; i >= 0; --i)
				{
					if (Dot3(faces[i].normal, sup.w - verts[faces[i].i[0]].w) <= 0)
						continue;

					// Face is visible from the new point — collect its edges
					for (int j = 0; j < 3; ++j)
					{
						int ea = faces[i].i[j];
						int eb = faces[i].i[(j + 1) % 3];

						// Check if the reverse edge already exists (shared interior edge)
						bool shared = false;
						for (int k = ne - 1; k >= 0; --k)
						{
							if (edges[k].a == eb && edges[k].b == ea)
							{
								edges[k] = edges[--ne]; // cancel shared edge
								shared = true;
								break;
							}
						}
						if (!shared)
						{
							if (ne >= MaxEpaFaces * 3)
							{
								if (fail_on_exhaustion)
									return false;

								normal = cf_normal;
								depth = cf_dist;
								return true;
							}

							edges[ne++] = { ea, eb };
						}
					}

					// Remove the visible face (swap with last)
					faces[i] = faces[--nf];
				}

				// Create new faces from horizon edges to the new vertex
				for (int i = 0; i < ne; ++i)
				{
					if (!add_face(edges[i].a, edges[i].b, ni))
					{
						if (fail_on_exhaustion)
							return false;

						normal = cf_normal;
						depth = cf_dist;
						return true;
					}
				}
			}

			return false; // EPA did not converge
		}
	}

	// GJK + EPA collision detection for two arbitrary convex shapes.
	// Compatible with the tri-table function signature.
	inline bool pr_vectorcall GjkCollide(Shape const& lhs, m4x4 const& l2w, Shape const& rhs, m4x4 const& r2w, Contact& contact)
	{
		using namespace gjk;

		// Compute shape-to-world and world-to-shape transforms
		auto a2w = l2w * lhs.m_s2r;
		auto b2w = r2w * rhs.m_s2r;
		auto w2a = InvertOrthonormal(a2w);
		auto w2b = InvertOrthonormal(b2w);

		int ha = 0, hb = 0; // support vertex hints (warm-start for polytopes)

		// Initial search direction: from B's centre toward A's centre
		auto dir = (a2w.pos - b2w.pos).w0();
		if (LengthSq(dir) < Eps)
			dir = v4::XAxis();

		// Seed the simplex with the first support point
		Simplex sx;
		auto sup = MkSupport(lhs, a2w, w2a, rhs, b2w, w2b, dir, ha, hb);
		sx.Push(sup);
		dir = -sup.w;

		// GJK main loop: build a simplex that (hopefully) encloses the origin
		for (int iter = 0; iter < MaxIter; ++iter)
		{
			if (LengthSq(dir) < Eps)
				break;

			sup = MkSupport(lhs, a2w, w2a, rhs, b2w, w2b, dir, ha, hb);

			// If the new support point doesn't pass the origin, shapes are separated
			if (Dot3(sup.w, dir) < 0)
				return false;

			sx.Push(sup);

			if (DoSimplex(sx, dir))
			{
				// Origin enclosed — shapes overlap. Run EPA for penetration info.
				v4 normal;
				float depth;
				if (!Epa(lhs, a2w, w2a, rhs, b2w, w2b, sx, ha, hb, normal, depth))
					return false;

				// Orient axis from lhs toward rhs (convention: axis points A->B)
				auto projection_centre = [&](Shape const& shape, m4x4 const& s2w, m4x4 const& w2s)
				{
					auto axis = w2s * normal;
					EFeature feature_type;
					auto smax = s2w * SupportVertex(shape, +axis, feature_type);
					auto smin = s2w * SupportVertex(shape, -axis, feature_type);
					return 0.5f * (Dot3(normal, smin) + Dot3(normal, smax));
				};

				auto pa = projection_centre(lhs, a2w, w2a);
				auto pb = projection_centre(rhs, b2w, w2b);
				auto sep_axis = Bool2SignF(pa <= pb) * normal;
				auto [manifold, feature] = FindContactManifold(lhs, l2w, rhs, r2w, sep_axis, depth);

				contact.m_axis = sep_axis;
				contact.m_depth = depth;
				contact.m_manifold = manifold;
				contact.m_feature = feature;
				contact.m_mat_idA = lhs.m_material_id;
				contact.m_mat_idB = rhs.m_material_id;
				return true;
			}
		}
		return false; // GJK did not converge
	}

	// Closest-point GJK: find the closest point on the Minkowski difference
	// (shape_a - point_b) to the origin in world space. Equivalent to the closest
	// point on shape_a to point_b minus point_b. Returns false if point_b is
	// strictly inside shape_a (origin enclosed in the Minkowski difference) — the
	// caller is responsible for the deep-penetration fallback. On success,
	// |out_w_closest| equals the distance from point_b to shape_a.
	//
	// Uses the standard Voronoi-style simplex reduction (DoSimplex) which keeps
	// the most recent support vertex at index 0 — this guarantees progress.
	// ---- Generic closest-point GJK loop ----
	// Shared between GjkClosestPointToPoint and GjkClosestPointShapeToShape.
	// 'mk_support' is any callable with signature: gjk::Sup(v4 dir).
	// Returns false if the origin is enclosed in the Minkowski difference (i.e.
	// the two "shapes" overlap — caller is responsible for the deep-penetration
	// fallback). On success, 'out_w_closest' is the closest Minkowski-space
	// point to the origin (whose magnitude is the core-to-core distance).
	template <typename MkSupportFn>
	inline bool GjkClosestPointLoop(MkSupportFn mk_support, v4 init_dir, v4& out_w_closest)
	{
		using namespace gjk;

		out_w_closest = v4::Zero();
		Simplex sx;
		auto sup = mk_support(init_dir);
		sx.Push(sup);

		auto w_closest = sup.w;
		auto closest_dist_sq = Dot3(w_closest, w_closest);

		for (int iter = 0; iter < MaxIter; ++iter)
		{
			// Search toward the origin from the current closest Minkowski point.
			auto dir = -w_closest.w0();
			auto dir_sq = Dot3(dir, dir);
			if (dir_sq < Eps)
				break;

			sup = mk_support(dir);

			// Standard GJK termination: if the new support doesn't extend further
			// along the search direction than the current closest point, we've
			// converged. Equivalent to: dot(sup.w, dir) <= dot(w_closest, dir) + tol.
			// Since dot(w_closest, dir) = -closest_dist_sq, the test becomes:
			//   proj < 0 && proj^2 >= closest_dist_sq * |dir|^2 (modulo a small tol).
			auto proj = Dot3(sup.w, dir);
			if (proj < 0 && proj * proj > closest_dist_sq * dir_sq * 0.99999f)
				break;

			// Reject duplicate support vertices (GJK has converged but rounding
			// nudged the projection check). Without this the simplex can end up
			// with two coincident vertices and the next reduction degenerates.
			bool duplicate = false;
			for (int i = 0; i != sx.n; ++i)
			{
				if (LengthSq(sup.w - sx.s[i].w) < Eps)
				{
					duplicate = true;
					break;
				}
			}
			if (duplicate)
				break;

			sx.Push(sup);

			// Voronoi reduction. Returns true only if a tetrahedron encloses the
			// origin (the two "shapes" overlap). The dummy_dir output is unused
			// here because we recompute the closest point from the simplex directly.
			auto dummy_dir = dir;
			if (DoSimplex(sx, dummy_dir))
				return false;

			w_closest = SimplexClosestPoint(sx);
			closest_dist_sq = Dot3(w_closest, w_closest);
		}

		out_w_closest = w_closest;
		return true;
	}

	// Closest-point GJK: find the closest point on the Minkowski difference
	// (shape_a - point_b) to the origin in world space. Equivalent to the closest
	// point on shape_a to point_b minus point_b. Returns false if point_b is
	// strictly inside shape_a (origin enclosed in the Minkowski difference) — the
	// caller is responsible for the deep-penetration fallback. On success,
	// |out_w_closest| equals the distance from point_b to shape_a.
	inline bool pr_vectorcall GjkClosestPointToPoint(
		Shape const& shape_a, m4x4 const& a2w, m4x4 const& w2a,
		v4 point_b, v4& out_w_closest)
	{
		using namespace gjk;

		// Initial search direction: from point_b toward shape_a's centre.
		auto dir = (a2w.pos - point_b).w0();
		if (LengthSq(dir) < Eps)
			dir = v4::XAxis();

		int ha = 0;
		auto mk = [&](v4 d) { return MkSupportPoint(shape_a, a2w, w2a, point_b, d, ha); };
		return GjkClosestPointLoop(mk, dir, out_w_closest);
	}

	// Closest-point GJK between two arbitrary convex shapes. Returns false if
	// the shapes overlap (origin enclosed in the Minkowski difference) — the
	// caller is responsible for the deep-penetration fallback. On success,
	// |out_w_closest| equals the distance between the shapes.
	//
	// Note: this operates on the shapes as defined by their SupportVertex
	// functions, including any intrinsic margin (e.g. ShapeLine::m_radius).
	// To compute the distance between cores (excluding margins), pass shapes
	// with the margin stripped (e.g. ShapeLine with m_radius set to 0).
	inline bool pr_vectorcall GjkClosestPointShapeToShape(
		Shape const& shape_a, m4x4 const& a2w, m4x4 const& w2a,
		Shape const& shape_b, m4x4 const& b2w, m4x4 const& w2b,
		v4& out_w_closest)
	{
		using namespace gjk;

		auto dir = (a2w.pos - b2w.pos).w0();
		if (LengthSq(dir) < Eps)
			dir = v4::XAxis();

		int ha = 0, hb = 0;
		auto mk = [&](v4 d) { return MkSupport(shape_a, a2w, w2a, shape_b, b2w, w2b, d, ha, hb); };
		return GjkClosestPointLoop(mk, dir, out_w_closest);
	}

	// Convex-vs-Sphere collision via "GJK with margins": run closest-point GJK on
	// the convex shape vs the sphere centre (a single point), then check distance
	// < radius and add the margin analytically. This avoids EPA entirely and gives
	// exact contact normals — far more accurate than EPA against a curved Minkowski
	// boundary near a polytope edge.
	//
	// Compatible with the tri-table function signature. Expects lhs to be the
	// convex shape (Polytope) and rhs to be the sphere.
	inline bool pr_vectorcall ConvexVsSphere(Shape const& lhs, m4x4 const& l2w, Shape const& rhs, m4x4 const& r2w, Contact& contact)
	{
		using namespace gjk;
		assert(rhs.m_type == EShape::Sphere && "ConvexVsSphere requires rhs to be a Sphere");

		auto a2w = l2w * lhs.m_s2r;
		auto b2w = r2w * rhs.m_s2r;
		auto w2a = InvertOrthonormal(a2w);

		auto sphere_centre = b2w.pos.w1();
		auto radius = shape_cast<ShapeSphere>(rhs).m_radius;

		// Find the closest point on the convex to the sphere centre.
		v4 w_closest;
		if (!GjkClosestPointToPoint(lhs, a2w, w2a, sphere_centre, w_closest))
		{
			// Sphere centre is inside the convex (deep penetration). Use the convex
			// centre to sphere centre direction as a best-effort axis.
			auto axis = Normalise((sphere_centre - a2w.pos).w0(), v4::XAxis());
			contact.m_axis = axis;
			contact.m_depth = radius;
			contact.m_manifold = {};
			contact.m_manifold[0] = sphere_centre;
			contact.m_feature = EFeature::Vert;
			contact.m_mat_idA = lhs.m_material_id;
			contact.m_mat_idB = rhs.m_material_id;
			return true;
		}

		auto dist_sq = Dot3(w_closest, w_closest);
		auto dist = Sqrt(dist_sq);
		if (dist >= radius)
			return false;

		v4 axis;
		v4 mid;
		if (dist <= Eps)
		{
			// Sphere centre lies on the convex surface — degenerate normal direction.
			axis = Normalise((sphere_centre - a2w.pos).w0(), v4::XAxis());
			mid = sphere_centre;
		}
		else
		{
			// w_closest = closest_convex_pt - sphere_centre, so it points FROM the
			// sphere centre TOWARD the convex. Contact axis convention is "from A
			// (convex) to B (sphere)", i.e. opposite to w_closest. Midplane contact
			// point matches SphereVsSphere convention: midpoint between the two
			// penetrating surface points (closest convex pt and sphere surface in
			// the same direction).
			auto normal = w_closest / dist;
			auto depth = radius - dist;
			axis = -normal;
			mid = sphere_centre + normal * (radius - 0.5f * depth);
		}

		contact.m_axis = axis;
		contact.m_depth = radius - dist;
		contact.m_manifold = {};
		contact.m_manifold[0] = mid;
		contact.m_feature = EFeature::Vert;
		contact.m_mat_idA = lhs.m_material_id;
		contact.m_mat_idB = rhs.m_material_id;
		return true;
	}

	// Convex-vs-Line collision via "GJK with margins": run closest-point GJK on
	// the convex shape vs the line's CORE (line segment with radius stripped to 0),
	// then check distance < line.m_radius and add the margin analytically. This
	// avoids EPA against the curved Minkowski boundary near a polytope edge/vertex
	// (capsule lateral surface), giving exact contact normals.
	//
	// Compatible with the tri-table function signature. Expects lhs to be the
	// convex shape (Polytope) and rhs to be the line/capsule.
	//
	// The contact manifold is built via FindContactManifold using the GJK-derived
	// axis, which correctly handles the parallel-edge case (line parallel to a
	// polytope edge produces a 2-point edge manifold).
	inline bool pr_vectorcall ConvexVsLine(Shape const& lhs, m4x4 const& l2w, Shape const& rhs, m4x4 const& r2w, Contact& contact)
	{
		using namespace gjk;
		assert(rhs.m_type == EShape::Line && "ConvexVsLine requires rhs to be a Line");

		auto const& line_orig = shape_cast<ShapeLine>(rhs);
		auto line_radius = line_orig.m_radius;

		// 1D line with no margin: nothing to strip; defer to GjkCollide on the raw shapes.
		if (line_radius == 0.0f)
			return GjkCollide(lhs, l2w, rhs, r2w, contact);

		auto a2w = l2w * lhs.m_s2r;
		auto b2w = r2w * rhs.m_s2r;
		auto w2a = InvertOrthonormal(a2w);
		auto w2b = InvertOrthonormal(b2w);

		// Build a "core" copy of the line with the margin stripped — used only
		// for the GJK closest-point query. The original line (with radius) is
		// passed to FindContactManifold so capsule edge contacts are recognised.
		auto line_core = line_orig;
		line_core.m_radius = 0.0f;

		// Run closest-point GJK on (convex, line_core). Returns false if the
		// line core is fully inside the convex (deep penetration) — fall back to
		// full GJK+EPA on the inflated shapes for that case.
		v4 w_closest;
		if (!GjkClosestPointShapeToShape(lhs, a2w, w2a, line_core, b2w, w2b, w_closest))
			return GjkCollide(lhs, l2w, rhs, r2w, contact);

		auto dist_sq = Dot3(w_closest, w_closest);
		auto dist = Sqrt(dist_sq);
		if (dist >= line_radius)
			return false;

		v4 axis;
		float depth;
		if (dist <= Eps)
		{
			// Line core touches convex surface — degenerate normal direction.
			axis = Normalise((b2w.pos - a2w.pos).w0(), v4::XAxis());
			depth = line_radius;
		}
		else
		{
			// w_closest = sup_a - sup_b, points FROM the line core TOWARD the
			// convex. Contact axis convention is "from A (convex) to B (line)",
			// i.e. opposite to w_closest.
			auto normal = w_closest / dist;
			axis = -normal;
			depth = line_radius - dist;
		}

		// Construct the contact manifold from the GJK-derived axis. SupportFeature
		// on the line returns its full edge when the axis is perpendicular to the
		// line direction, so a parallel-edge contact correctly reduces to a 2-point
		// edge manifold via the existing FindContactManifold edge-edge case.
		auto [manifold, feature] = FindContactManifold(lhs, l2w, rhs, r2w, axis, depth);

		contact.m_axis = axis;
		contact.m_depth = depth;
		contact.m_manifold = manifold;
		contact.m_feature = feature;
		contact.m_mat_idA = lhs.m_material_id;
		contact.m_mat_idB = rhs.m_material_id;
		return true;
	}
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
#include "pr/collision/col_sphere_vs_sphere.h"

namespace pr::collision::tests
{
	PRUnitTestClass(GjkTests)
	{
		inline static constexpr bool CreateVisuals = false;

		// Two overlapping spheres: GJK should detect contact
		PRUnitTestMethod(OverlappingSpheres)
		{
			auto sa = ShapeSphere{1.0f};
			auto sb = ShapeSphere{1.0f};
			auto l2w = m4x4::Identity();
			auto r2w = m4x4::Translation(1.0f, 0, 0); // centres 1.0 apart, radii sum = 2

			Contact gjk_c;
			PR_EXPECT(GjkCollide(sa, l2w, sb, r2w, gjk_c));
			PR_EXPECT(gjk_c.m_depth > 0.0f);

			// Compare with specialised SphereVsSphere
			Contact ref_c;
			PR_EXPECT(SphereVsSphere(sa, l2w, sb, r2w, ref_c));
			PR_EXPECT(FEqlRelative(gjk_c.m_depth, ref_c.m_depth, 0.05f));
		}

		// Separated spheres: GJK should return false
		PRUnitTestMethod(SeparatedSpheres)
		{
			auto sa = ShapeSphere{0.5f};
			auto sb = ShapeSphere{0.5f};
			auto l2w = m4x4::Identity();
			auto r2w = m4x4::Translation(3, 0, 0);

			Contact c;
			PR_EXPECT(!GjkCollide(sa, l2w, sb, r2w, c));
		}

		// Two overlapping boxes
		PRUnitTestMethod(OverlappingBoxes)
		{
			auto ba = ShapeBox{v4{2, 2, 2, 0}}; // half-extent = 1
			auto bb = ShapeBox{v4{2, 2, 2, 0}};
			auto l2w = m4x4::Identity();
			auto r2w = m4x4::Translation(1.5f, 0, 0); // overlap of 0.5 in X

			Contact gjk_c;
			PR_EXPECT(GjkCollide(ba, l2w, bb, r2w, gjk_c));

			// Compare with specialised BoxVsBox
			Contact ref_c;
			PR_EXPECT(BoxVsBox(ba, l2w, bb, r2w, ref_c));
			PR_EXPECT(FEqlRelative(gjk_c.m_depth, ref_c.m_depth, 0.05f));
		}

		// Separated boxes
		PRUnitTestMethod(SeparatedBoxes)
		{
			auto ba = ShapeBox{v4{2, 2, 2, 0}};
			auto bb = ShapeBox{v4{2, 2, 2, 0}};
			auto l2w = m4x4::Identity();
			auto r2w = m4x4::Translation(5, 0, 0);

			Contact c;
			PR_EXPECT(!GjkCollide(ba, l2w, bb, r2w, c));
		}

		// Box vs Sphere
		PRUnitTestMethod(BoxVsSphereGjk)
		{
			auto box = ShapeBox{v4{2, 2, 2, 0}}; // half-extent = 1
			auto sph = ShapeSphere{0.5f};
			auto l2w = m4x4::Identity();
			auto r2w = m4x4::Translation(1.2f, 0, 0); // sphere centre 1.2 from box centre

			// Box surface at x=1, sphere surface at x=0.7..1.7 → overlap 0.3
			Contact gjk_c;
			PR_EXPECT(GjkCollide(box, l2w, sph, r2w, gjk_c));
			PR_EXPECT(gjk_c.m_depth > 0.0f);
		}

		// Contact axis direction: should point from lhs to rhs
		PRUnitTestMethod(AxisDirection)
		{
			auto sa = ShapeSphere{1.0f};
			auto sb = ShapeSphere{1.0f};
			auto l2w = m4x4::Identity();
			auto r2w = m4x4::Translation(1.0f, 0, 0);

			Contact c;
			PR_EXPECT(GjkCollide(sa, l2w, sb, r2w, c));

			// Axis should point from A to B, roughly in +X
			PR_EXPECT(c.m_axis.x > 0.5f);
		}

		// Nearly touching: barely overlapping spheres
		PRUnitTestMethod(BarelyOverlapping)
		{
			auto sa = ShapeSphere{1.0f};
			auto sb = ShapeSphere{1.0f};
			auto l2w = m4x4::Identity();
			auto r2w = m4x4::Translation(1.99f, 0, 0); // overlap = 0.01

			Contact c;
			PR_EXPECT(GjkCollide(sa, l2w, sb, r2w, c));
			PR_EXPECT(c.m_depth > 0.0f && c.m_depth < 0.05f);
		}

		// Coincident shapes: centres at same position
		PRUnitTestMethod(CoincidentCentres)
		{
			auto sa = ShapeSphere{1.0f};
			auto sb = ShapeSphere{1.0f};
			auto l2w = m4x4::Identity();
			auto r2w = m4x4::Identity();

			Contact c;
			PR_EXPECT(GjkCollide(sa, l2w, sb, r2w, c));
			PR_EXPECT(c.m_depth > 0.0f);
		}

		// Rotated box vs sphere
		PRUnitTestMethod(RotatedBoxVsSphere)
		{
			auto box = ShapeBox{v4{2, 1, 1, 0}}; // half-extents: (1, 0.5, 0.5)
			auto sph = ShapeSphere{0.3f};

			// Rotate box 45° about Z, place sphere near a corner
			auto l2w = m4x4::Transform(RotationRad<m3x3>(0, 0, constants<float>::tau_by_8), v4::Origin());
			auto r2w = m4x4::Translation(0.9f, 0.9f, 0);

			Contact gjk_c;
			auto gjk_result = GjkCollide(box, l2w, sph, r2w, gjk_c);

			// Also test with specialised function for comparison
			Contact ref_c;
			auto ref_result = BoxVsSphere(box, l2w, sph, r2w, ref_c);
			PR_EXPECT(gjk_result == ref_result);

			if (gjk_result && ref_result)
				PR_EXPECT(FEqlRelative(gjk_c.m_depth, ref_c.m_depth, 0.1f));
		}

		// GJK collision between polytopes built from point clouds via BuildPolytopeFromPoints
		PRUnitTestMethod(GjkBuiltPolytopes)
		{
			v4 pts_a[] = {
				v4{-1, -1, -1, 1}, v4{1, -1, -1, 1},
				v4{0, 1, -1, 1}, v4{0, 0, 1, 1},
			};
			v4 pts_b[] = {
				v4{-1, -1, -1, 1}, v4{1, -1, -1, 1},
				v4{0, 1, -1, 1}, v4{0, 0, 1, 1},
			};

			auto buf_a = BuildPolytopeFromPoints(pts_a);
			auto buf_b = BuildPolytopeFromPoints(pts_b);
			auto& pa = buf_a.as<ShapePolytope>();
			auto& pb = buf_b.as<ShapePolytope>();

			// Overlapping at origin — should collide
			Contact c;
			PR_EXPECT(GjkCollide(pa, m4x4::Identity(), pb, m4x4::Identity(), c));

			// Separated — should not collide
			PR_EXPECT(!GjkCollide(pa, m4x4::Identity(), pb, m4x4::Translation(10, 0, 0), c));
		}

		PRUnitTestMethod(PolytopeFaceManifold)
		{
			v4 cube_pts[] = {
				v4{-1, -1, -1, 1}, v4{+1, -1, -1, 1},
				v4{-1, +1, -1, 1}, v4{+1, +1, -1, 1},
				v4{-1, -1, +1, 1}, v4{+1, -1, +1, 1},
				v4{-1, +1, +1, 1}, v4{+1, +1, +1, 1},
			};

			auto buf_a = BuildPolytopeFromPoints(cube_pts);
			auto buf_b = BuildPolytopeFromPoints(cube_pts);
			auto& pa = buf_a.as<ShapePolytope>();
			auto& pb = buf_b.as<ShapePolytope>();

			Contact c;
			PR_EXPECT(GjkCollide(pa, m4x4::Identity(), pb, m4x4::Translation(1.5f, 0, 0), c));
			PR_EXPECT(FEqlRelative(c.m_depth, 0.5f, 0.05f));
			PR_EXPECT(c.Count() == 4);
			PR_EXPECT(c.m_feature == EFeature::Quad);
			PR_EXPECT(FEqlRelative(c.Point(), v4{0.75f, 0, 0, 1}, 0.01f));
			PR_EXPECT(Dot(c.m_axis, Cross(c.m_manifold[1] - c.m_manifold[0], c.m_manifold[2] - c.m_manifold[0])) > 0.0f);
		}
	};
}
#endif
