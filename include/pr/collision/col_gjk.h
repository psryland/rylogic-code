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

		// ---- EPA (Expanding Polytope Algorithm) ----

		struct Face
		{
			int i[3];    // Vertex indices
			v4 normal;   // Outward-facing unit normal
			float dist;  // Distance from origin to face plane
		};

		// Find penetration depth and contact info from a GJK simplex containing the origin.
		inline bool Epa(
			Shape const& sa, m4x4 const& a2w, m4x4 const& w2a,
			Shape const& sb, m4x4 const& b2w, m4x4 const& w2b,
			Simplex const& gjk_sx, int& ha, int& hb,
			v4& normal, float& depth)
		{
			if (gjk_sx.n < 4) return false;

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
				if (d - cf_dist < EpaEps || nv >= MaxEpaVerts)
				{
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
						if (!shared && ne < MaxEpaFaces * 3)
							edges[ne++] = { ea, eb };
					}

					// Remove the visible face (swap with last)
					faces[i] = faces[--nf];
				}

				// Create new faces from horizon edges to the new vertex
				for (int i = 0; i < ne; ++i)
					add_face(edges[i].a, edges[i].b, ni);
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
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
#include "pr/collision/col_sphere_vs_sphere.h"

namespace pr::collision::tests
{
	PRUnitTestClass(GjkTests)
	{
		inline static constexpr bool CreateVisuals = true;

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
