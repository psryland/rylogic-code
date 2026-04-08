//************************************
// Physics Engine
//  Copyright (c) Rylogic Ltd 2016
//************************************
// Unit tests that compare GPU collision functions (compiled as C++) against CPU collision functions.

#if PR_UNITTESTS
#include "pr/common/unittests.h"
#include "pr/physics/physics.h"
#include "src/compute/physics_types.h"
#include "src/compute/collision.hlsli"
#include "src/compute/gjk.hlsli"

namespace pr::physics::tests
{
	void ForceLink_GpuCollision() {}

	PRUnitTestClass(GpuCollisionTests)
	{
		// Tolerance check
		static bool Near(float a, float b, float tol = 1e-3f)
		{
			return std::abs(a - b) <= tol;
		}

		// Check axis directions match (allowing sign flip)
		static bool AxisMatch(pr::hlsl::float4 gpu, pr::v4 cpu, float tol = 1e-3f)
		{
			auto d = std::abs(gpu.x * cpu.x + gpu.y * cpu.y + gpu.z * cpu.z);
			return d > 1.0f - tol;
		}

		// ---- Sphere vs Sphere ----
		PRUnitTestMethod(SphereVsSphere)
		{
			using namespace pr;
			using namespace pr::hlsl;

			// Compare CPU and GPU results, and check expected depth (negative = no collision)
			auto BangTogether = [](collision::ShapeSphere const& a, m4x4 a2w, collision::ShapeSphere const& b, m4x4 b2w, float expected_depth)
			{
				bool should_collide = expected_depth >= 0;

				collision::Contact c;
				auto is_contact_cpu = collision::SphereVsSphere(a, a2w, b, b2w, c);

				float4 axis, point; float depth;
				auto is_contact_gpu = physics::SphereVsSphere(PackShape(a), a2w, PackShape(b), b2w, axis, point, depth);

				PR_EXPECT(should_collide == is_contact_gpu);
				PR_EXPECT(is_contact_gpu == is_contact_cpu);
				if (should_collide)
				{
					PR_EXPECT(Near(c.m_depth, depth));
					PR_EXPECT(Near(expected_depth, depth));
					PR_EXPECT(AxisMatch(axis, c.m_axis));
				}
			};

			// Overlapping along X: depth = 1.0 + 1.0 - 1.5 = 0.5
			{
				collision::ShapeSphere a(1.0f), b(1.0f);
				BangTogether(a, m4x4::Identity(), b, m4x4::Translation(1.5f, 0, 0), 0.5f);
			}

			// Separated
			{
				collision::ShapeSphere a(1.0f), b(1.0f);
				BangTogether(a, m4x4::Identity(), b, m4x4::Translation(5, 0, 0), -1);
			}

			// Diagonal overlap, different radii: depth = 2.0 + 1.0 - sqrt(3) ≈ 1.268
			{
				collision::ShapeSphere a(2.0f), b(1.0f);
				BangTogether(a, m4x4::Identity(), b, m4x4::Translation(1, 1, 1), 3.0f - Sqrt(3.0f));
			}

			// Both translated, overlapping along Y: depth = 0.5 + 0.3 - 0.6 = 0.2
			{
				collision::ShapeSphere a(0.5f), b(0.3f);
				BangTogether(a, m4x4::Translation(1, 2, 3), b, m4x4::Translation(1, 2.6f, 3), 0.2f);
			}
		}

		// ---- Sphere vs Box ----
		PRUnitTestMethod(SphereVsBox)
		{
			using namespace pr;
			using namespace pr::hlsl;

			// GPU takes (sphere, box), CPU takes (box, sphere) — argument order differs
			auto BangTogether = [](collision::ShapeSphere const& sph, m4x4 sph2w, collision::ShapeBox const& box, m4x4 box2w, float expected_depth)
			{
				bool should_collide = expected_depth >= 0;

				collision::Contact c;
				auto is_contact_cpu = collision::BoxVsSphere(box, box2w, sph, sph2w, c);

				float4 axis, point; float depth;
				auto is_contact_gpu = physics::SphereVsBox(PackShape(sph), sph2w, PackShape(box), box2w, axis, point, depth);

				PR_EXPECT(should_collide == is_contact_gpu);
				PR_EXPECT(is_contact_gpu == is_contact_cpu);
				if (should_collide)
				{
					PR_EXPECT(Near(c.m_depth, depth));
					PR_EXPECT(Near(expected_depth, depth));
				}
			};

			collision::ShapeBox unit_box(v4(1, 1, 1, 0)); // half-extent 0.5

			// Separated: sphere at x=2, radius=0.5, box face at x=0.5, gap=1.0
			BangTogether(collision::ShapeSphere(0.5f), m4x4::Translation(2.0f, 0, 0), unit_box, m4x4::Identity(), -1);

			// Face overlap: sphere surface at 0.8-0.5=0.3, box face at 0.5, depth=0.2
			BangTogether(collision::ShapeSphere(0.5f), m4x4::Translation(0.8f, 0, 0), unit_box, m4x4::Identity(), 0.2f);

			// Sphere near box corner: dist to corner (0.5,0.5,0) = sqrt(0.25+0.25)≈0.707, depth=1.0-0.707≈0.293
			BangTogether(collision::ShapeSphere(1.0f), m4x4::Translation(1.0f, 1.0f, 0), unit_box, m4x4::Identity(), 1.0f - Sqrt(0.5f));

			// Sphere centre inside box: depth = dist_to_nearest_face + radius = 0.5 + 0.1 = 0.6
			BangTogether(collision::ShapeSphere(0.1f), m4x4::Translation(0, 0, 0), collision::ShapeBox(v4(2, 2, 2, 0)), m4x4::Identity(), 1.1f);

			// Both transformed
			BangTogether(collision::ShapeSphere(0.3f), m4x4::Translation(1.5f, 2.0f, 0), unit_box, m4x4::Translation(1.2f, 2.0f, 0), 0.3f + 0.5f - 0.3f);

			// Sphere overlapping rotated box corner: depth ≈ sqrt(0.5) - 0.5 ≈ 0.207
			{
				auto box2w = m4x4::Transform(m3x3::RotationDeg(0, 0, 45.0f), v4(1, 0, 0, 1));
				BangTogether(collision::ShapeSphere(0.5f), m4x4::Identity(), unit_box, box2w, Sqrt(0.5f) - 0.5f);
			}
		}

		// ---- Sphere vs Line ----
		PRUnitTestMethod(SphereVsLine)
		{
			using namespace pr;
			using namespace pr::hlsl;

			// GPU takes (sphere, line), CPU takes (line, sphere) — argument order differs
			auto BangTogether = [](collision::ShapeSphere const& sph, m4x4 sph2w, collision::ShapeLine const& line, m4x4 line2w, float expected_depth)
			{
				bool should_collide = expected_depth >= 0;

				collision::Contact c;
				auto is_contact_cpu = collision::LineVsSphere(line, line2w, sph, sph2w, c);

				float4 axis, point; float depth;
				auto is_contact_gpu = physics::SphereVsLine(PackShape(sph), sph2w, PackShape(line), line2w, axis, point, depth);

				PR_EXPECT(should_collide == is_contact_gpu);
				PR_EXPECT(is_contact_gpu == is_contact_cpu);
				if (should_collide)
				{
					PR_EXPECT(Near(c.m_depth, depth));
					PR_EXPECT(Near(expected_depth, depth));
				}
			};

			// Sphere near midpoint of Z-aligned line
			// ShapeLine(2.0, 0.1) → half_len=1.0, thick_radius=0.05
			// depth = sphere_r + thick_r - distance = 0.5 + 0.05 - 0.4 = 0.15
			BangTogether(collision::ShapeSphere(0.5f), m4x4::Translation(0.4f, 0, 0), collision::ShapeLine(2.0f, 0.1f), m4x4::Identity(), 0.15f);

			// Sphere near endpoint (separated): dist_to_endpoint > radius + thickness
			BangTogether(collision::ShapeSphere(0.5f), m4x4::Translation(0.3f, 0, 1.3f), collision::ShapeLine(1.0f, 0.1f), m4x4::Identity(), -1);

			// Separated
			BangTogether(collision::ShapeSphere(0.5f), m4x4::Translation(3, 0, 0), collision::ShapeLine(1.0f, 0.1f), m4x4::Identity(), -1);

			// Both translated: depth = 0.5 + 0.05 - 0.4 = 0.15
			BangTogether(collision::ShapeSphere(0.5f), m4x4::Translation(2.4f, 3, 0), collision::ShapeLine(2.0f, 0.1f), m4x4::Translation(2, 3, 0), 0.15f);
		}

		// ---- Line vs Line ----
		PRUnitTestMethod(LineVsLine)
		{
			using namespace pr;
			using namespace pr::hlsl;

			auto BangTogether = [](collision::ShapeLine const& a, m4x4 a2w, collision::ShapeLine const& b, m4x4 b2w, float expected_depth)
			{
				bool should_collide = expected_depth >= 0;

				collision::Contact c;
				auto is_contact_cpu = collision::LineVsLine(a, a2w, b, b2w, c);

				float4 axis, point; float depth;
				auto is_contact_gpu = physics::LineVsLine(PackShape(a), a2w, PackShape(b), b2w, axis, point, depth);

				PR_EXPECT(should_collide == is_contact_gpu);
				PR_EXPECT(is_contact_gpu == is_contact_cpu);
				if (should_collide)
				{
					PR_EXPECT(Near(c.m_depth, depth));
					PR_EXPECT(Near(expected_depth, depth));
				}
			};

			// Two perpendicular lines crossing near origin, separated by 0.3 (thickness 0.1+0.1 < 0.3)
			{
				auto b2w = m4x4::Transform(v4(0, 1, 0, 0), float(math::constants<float>::tau_by_4), v4(0, 0.3f, 0, 1));
				BangTogether(collision::ShapeLine(2.0f, 0.2f), m4x4::Identity(), collision::ShapeLine(2.0f, 0.2f), b2w, -1);
			}

			// Perpendicular lines, close enough to collide: depth = 0.1 + 0.1 - 0.15 = 0.05
			{
				auto b2w = m4x4::Transform(v4(0, 1, 0, 0), float(math::constants<float>::tau_by_4), v4(0, 0.15f, 0, 1));
				BangTogether(collision::ShapeLine(2.0f, 0.2f), m4x4::Identity(), collision::ShapeLine(2.0f, 0.2f), b2w, 0.05f);
			}

			// Parallel lines, close but separated: gap = 0.5 - 0.15 - 0.15 = 0.2
			BangTogether(collision::ShapeLine(2.0f, 0.3f), m4x4::Identity(), collision::ShapeLine(2.0f, 0.3f), m4x4::Translation(0.5f, 0, 0), -1);

			// Parallel lines, overlapping: depth = 0.15 + 0.15 - 0.2 = 0.1
			BangTogether(collision::ShapeLine(2.0f, 0.3f), m4x4::Identity(), collision::ShapeLine(2.0f, 0.3f), m4x4::Translation(0.2f, 0, 0), 0.1f);

			// Separated (far apart)
			BangTogether(collision::ShapeLine(1.0f, 0.1f), m4x4::Identity(), collision::ShapeLine(1.0f, 0.1f), m4x4::Translation(5, 0, 0), -1);
		}

		// ---- Line vs Box ----
		PRUnitTestMethod(LineVsBox)
		{
			using namespace pr;
			using namespace pr::hlsl;

			auto BangTogether = [](collision::ShapeLine const& line, m4x4 line2w, collision::ShapeBox const& box, m4x4 box2w, float expected_depth)
			{
				bool should_collide = expected_depth >= 0;

				collision::Contact c;
				auto is_contact_cpu = collision::LineVsBox(line, line2w, box, box2w, c);

				float4 axis, point; float depth;
				auto is_contact_gpu = physics::LineVsBox(PackShape(line), line2w, PackShape(box), box2w, axis, point, depth);

				PR_EXPECT(should_collide == is_contact_gpu);
				PR_EXPECT(is_contact_gpu == is_contact_cpu);
				if (should_collide)
				{
					PR_EXPECT(Near(c.m_depth, depth, 0.05f));
					PR_EXPECT(Near(expected_depth, depth, 0.05f));
				}
			};

			collision::ShapeBox unit_box(v4(1, 1, 1, 0));

			// Line along Z approaching box face
			// ShapeLine(1.0, 0.2) → half_len=0.5, thick_radius=0.1
			// Line axis at x=0.55, box face at x=0.5, gap=0.05, depth = 0.1-0.05 = 0.05
			BangTogether(collision::ShapeLine(1.0f, 0.2f), m4x4::Translation(0.55f, 0, 0), unit_box, m4x4::Identity(), 0.05f);

			// Line far from box (separated)
			BangTogether(collision::ShapeLine(1.0f, 0.1f), m4x4::Translation(5, 0, 0), unit_box, m4x4::Identity(), -1);
		}

		// ---- Box vs Box ----
		PRUnitTestMethod(BoxVsBox)
		{
			using namespace pr;
			using namespace pr::hlsl;

			// Collide two boxes, and compare the CPU and GPU results
			auto BangTogether = [](collision::ShapeBox const& a, m4x4 a2w, collision::ShapeBox const& b, m4x4 b2w, bool should_collide)
			{
				collision::Contact c;
				auto is_contact_cpu = collision::BoxVsBox(a, a2w, b, b2w, c);

				auto sa = PackShape(a);
				auto sb = PackShape(b);
				float4 axis, point; float depth;
				auto is_contact_gpu = physics::BoxVsBox(sa, a2w, sb, b2w, axis, point, depth);

				PR_EXPECT(should_collide == is_contact_gpu);
				PR_EXPECT(is_contact_gpu == is_contact_cpu);
				if (should_collide)
				{
					PR_EXPECT(Near(c.m_depth, depth));
					PR_EXPECT(FEql(c.m_point, point));
					PR_EXPECT(AxisMatch(c.m_axis, axis));
				}
			};

			collision::ShapeBox a(v4(1, 1, 1, 0));
			collision::ShapeBox b(v4(1, 1, 1, 0));

			// Axis-aligned overlap along X
			{
				auto a2w = m4x4::Identity();
				auto b2w = m4x4::Translation(0.99f, 0, 0);
				BangTogether(a, a2w, b, b2w, true);
			}

			// Separated
			{
				auto a2w = m4x4::Identity();
				auto b2w = m4x4::Translation(1.1f, 0, 0);
				BangTogether(a, a2w, b, b2w, false);
			}

			// Rotated box overlap (45 degrees around Z)
			{
				auto a2w = m4x4::Identity();
				auto b2w = m4x4::Transform(m3x3::RotationDeg(0, 0, 45.0f), v4(1.15f, 0, 0, 1));
				BangTogether(a, a2w, b, b2w, true);
			}

			// Different sized boxes
			a = collision::ShapeBox(v4(2, 0.5f, 1, 0));
			b = collision::ShapeBox(v4(0.5f, 2, 1, 0));

			{
				auto a2w = m4x4::Identity();
				auto b2w = m4x4::Translation(1.24f, 0, 0);
				BangTogether(a, a2w, b, b2w, true);
			}

			// Corner contact
			{
				auto a2w = m4x4::Identity();
				auto b2w = m4x4::Transform(m3x3::RotationDeg(0, 45.0f, 45.0f), v4(1.95f, -0.5f, 0, 1));
				BangTogether(a, a2w, b, b2w, true);
			}

			// Edge contact
			{
				auto a2w = m4x4::Identity();
				auto b2w = m4x4::Transform(m3x3::RotationDeg(0, 45.0f, 45.0f), v4(1.75f, 0, 0, 1));
				BangTogether(a, a2w, b, b2w, true);
			}

			// ---- Both boxes transformed ----
			a = collision::ShapeBox(v4(1, 1, 1, 0));
			b = collision::ShapeBox(v4(1, 1, 1, 0));

			// Both rotated, face-face overlap
			{
				auto a2w = m4x4::Transform(m3x3::RotationDeg(0, 0, 20.0f), v4(-0.3f, 0.1f, 0, 1));
				auto b2w = m4x4::Transform(m3x3::RotationDeg(0, 0, -15.0f), v4(0.6f, -0.1f, 0, 1));
				BangTogether(a, a2w, b, b2w, true);
			}

			// Both rotated, separated
			{
				auto a2w = m4x4::Transform(m3x3::RotationDeg(0, 0, 20.0f), v4(-0.3f, 0.1f, 0, 1));
				auto b2w = m4x4::Transform(m3x3::RotationDeg(0, 0, -15.0f), v4(1.5f, 0, 0, 1));
				BangTogether(a, a2w, b, b2w, false);
			}

			// Both rotated around different axes, overlapping
			{
				auto a2w = m4x4::Transform(m3x3::RotationDeg(30.0f, 0, 0), v4(0, 0, 0, 1));
				auto b2w = m4x4::Transform(m3x3::RotationDeg(0, 0, 45.0f), v4(0.85f, 0, 0, 1));
				BangTogether(a, a2w, b, b2w, true);
			}

			// Both rotated around all three axes
			{
				auto a2w = m4x4::Transform(m3x3::RotationDeg(10.0f, 20.0f, 30.0f), v4(-0.2f, 0.1f, -0.1f, 1));
				auto b2w = m4x4::Transform(m3x3::RotationDeg(-15.0f, 25.0f, -10.0f), v4(0.7f, -0.1f, 0.2f, 1));
				BangTogether(a, a2w, b, b2w, true);
			}

			// ---- Edge-edge contact ----
			// Two elongated boxes crossing like an X, edges closest
			{
				auto thin = collision::ShapeBox(v4(0.2f, 0.2f, 3, 0));
				auto a2w = m4x4::Identity();
				auto b2w = m4x4::Transform(m3x3::RotationDeg(90.0f, 0, 0), v4(0.15f, 0, 0, 1));
				BangTogether(thin, a2w, thin, b2w, true);
			}

			// Two unit boxes positioned so an edge-edge cross-product axis is the minimum
			{
				auto a2w = m4x4::Transform(m3x3::RotationDeg(0, 0, 45.0f), v4(0, 0, 0, 1));
				auto b2w = m4x4::Transform(m3x3::RotationDeg(45.0f, 0, 0), v4(0, 0.95f, 0, 1));
				BangTogether(a, a2w, b, b2w, true);
			}

			// ---- Vertex-face contact ----
			// B's corner pokes into A's face (B rotated around two axes)
			{
				auto a2w = m4x4::Identity();
				auto b2w = m4x4::Transform(m3x3::RotationDeg(45.0f, 35.264f, 0), v4(0.95f, 0, 0, 1));
				BangTogether(a, a2w, b, b2w, true);
			}

			// A's corner pokes into B's face (A rotated, B at identity)
			{
				auto a2w = m4x4::Transform(m3x3::RotationDeg(45.0f, 35.264f, 0), v4(-0.95f, 0, 0, 1));
				auto b2w = m4x4::Identity();
				BangTogether(a, a2w, b, b2w, true);
			}

			// ---- Near-degenerate cases ----
			// Nearly touching (very small overlap)
			{
				auto a2w = m4x4::Identity();
				auto b2w = m4x4::Translation(0.999f, 0, 0);
				BangTogether(a, a2w, b, b2w, true);
			}

			// Deep overlap (one box mostly inside the other)
			{
				auto small_box = collision::ShapeBox(v4(0.3f, 0.3f, 0.3f, 0));
				auto big_box = collision::ShapeBox(v4(2, 2, 2, 0));
				auto a2w = m4x4::Identity();
				auto b2w = m4x4::Translation(0.1f, 0.05f, -0.02f);
				BangTogether(big_box, a2w, small_box, b2w, true);
			}

			// Boxes touching at an edge, both translated off-origin
			{
				auto a2w = m4x4::Transform(m3x3::RotationDeg(0, 0, 30.0f), v4(3, 5, 1, 1));
				auto b2w = m4x4::Transform(m3x3::RotationDeg(0, 0, -20.0f), v4(3.85f, 5, 1, 1));
				BangTogether(a, a2w, b, b2w, true);
			}

			// ---- Asymmetric shapes, both transformed ----
			{
				auto flat = collision::ShapeBox(v4(2, 2, 0.2f, 0));
				auto tall = collision::ShapeBox(v4(0.4f, 0.4f, 3, 0));
				auto a2w = m4x4::Transform(m3x3::RotationDeg(15.0f, 0, 0), v4(0, 0, 0, 1));
				auto b2w = m4x4::Transform(m3x3::RotationDeg(0, 30.0f, 0), v4(0.8f, 0, 0.5f, 1));
				BangTogether(flat, a2w, tall, b2w, true);
			}
		}

		// ---- Polytope vs Polytope ----
		PRUnitTestMethod(PolytopeVsPolytope)
		{
			using namespace pr;
			using namespace pr::hlsl;

			// Compare CPU GJK and GPU GJK results. GJK/EPA has wider tolerances than SAT.
			auto BangTogether = [](collision::ShapePolytope const& a, m4x4 a2w, collision::ShapePolytope const& b, m4x4 b2w, bool should_collide)
			{
				collision::Contact c;
				auto is_contact_cpu = collision::GjkCollide(a, a2w, b, b2w, c);

				// Build the vertex buffer for the GPU
				std::vector<v4> vertex_buffer;
				auto sa = PackShape(a, static_cast<int>(vertex_buffer.size()));
				for (auto const* v = a.vert_beg(); v != a.vert_end(); ++v)
					vertex_buffer.push_back(*v);

				auto sb = PackShape(b, static_cast<int>(vertex_buffer.size()));
				for (auto const* v = b.vert_beg(); v != b.vert_end(); ++v)
					vertex_buffer.push_back(*v);

				auto& verts = reinterpret_cast<StructuredBuffer<float4>&>(vertex_buffer);
				float4 axis, point; float depth;
				int gjk_iters, epa_iters;
				auto is_contact_gpu = physics::GjkCollide(sa, a2w, sb, b2w, verts, axis, point, depth, gjk_iters, epa_iters);

				PR_EXPECT(should_collide == is_contact_gpu);
				PR_EXPECT(is_contact_gpu == is_contact_cpu);
				if (should_collide)
				{
					PR_EXPECT(Near(c.m_depth, depth, 0.05f));
				}
			};

			// Two tetrahedra overlapping at origin
			{
				v4 tet_pts[] = {
					v4{-1, -1, -1, 1}, v4{1, -1, -1, 1},
					v4{0, 1, -1, 1}, v4{0, 0, 1, 1},
				};
				auto buf_a = collision::BuildPolytopeFromPoints(tet_pts);
				auto buf_b = collision::BuildPolytopeFromPoints(tet_pts);
				auto& pa = buf_a.as<collision::ShapePolytope>();
				auto& pb = buf_b.as<collision::ShapePolytope>();

				// Slight overlap along X
				BangTogether(pa, m4x4::Identity(), pb, m4x4::Translation(0.5f, 0, 0), true);
			}

			// Two tetrahedra separated
			{
				v4 tet_pts[] = {
					v4{-1, -1, -1, 1}, v4{1, -1, -1, 1},
					v4{0, 1, -1, 1}, v4{0, 0, 1, 1},
				};
				auto buf_a = collision::BuildPolytopeFromPoints(tet_pts);
				auto buf_b = collision::BuildPolytopeFromPoints(tet_pts);
				auto& pa = buf_a.as<collision::ShapePolytope>();
				auto& pb = buf_b.as<collision::ShapePolytope>();

				BangTogether(pa, m4x4::Identity(), pb, m4x4::Translation(10, 0, 0), false);
			}

			// Two cube polytopes overlapping along X
			{
				v4 cube_pts[] = {
					v4{-1, -1, -1, 1}, v4{1, -1, -1, 1},
					v4{-1, 1, -1, 1}, v4{1, 1, -1, 1},
					v4{-1, -1, 1, 1}, v4{1, -1, 1, 1},
					v4{-1, 1, 1, 1}, v4{1, 1, 1, 1},
				};
				auto buf_a = collision::BuildPolytopeFromPoints(cube_pts);
				auto buf_b = collision::BuildPolytopeFromPoints(cube_pts);
				auto& pa = buf_a.as<collision::ShapePolytope>();
				auto& pb = buf_b.as<collision::ShapePolytope>();

				// Overlap of 0.5 in X: cubes are ±1, so separation = 2*1 - 0.5 = 1.5
				BangTogether(pa, m4x4::Identity(), pb, m4x4::Translation(1.5f, 0, 0), true);
			}

			// Cube polytopes, both rotated
			{
				v4 cube_pts[] = {
					v4{-1, -1, -1, 1}, v4{1, -1, -1, 1},
					v4{-1, 1, -1, 1}, v4{1, 1, -1, 1},
					v4{-1, -1, 1, 1}, v4{1, -1, 1, 1},
					v4{-1, 1, 1, 1}, v4{1, 1, 1, 1},
				};
				auto buf_a = collision::BuildPolytopeFromPoints(cube_pts);
				auto buf_b = collision::BuildPolytopeFromPoints(cube_pts);
				auto& pa = buf_a.as<collision::ShapePolytope>();
				auto& pb = buf_b.as<collision::ShapePolytope>();

				auto a2w = m4x4::Transform(m3x3::RotationDeg(0, 0, 20.0f), v4(-0.5f, 0, 0, 1));
				auto b2w = m4x4::Transform(m3x3::RotationDeg(0, 0, -15.0f), v4(2.5f, 0, 0, 1));
				BangTogether(pa, a2w, pb, b2w, false);
			}

			// Tetrahedron vs cube, overlapping
			{
				v4 tet_pts[] = {
					v4{-1, -1, -1, 1}, v4{1, -1, -1, 1},
					v4{0, 1, -1, 1}, v4{0, 0, 1, 1},
				};
				v4 cube_pts[] = {
					v4{-0.5f, -0.5f, -0.5f, 1}, v4{0.5f, -0.5f, -0.5f, 1},
					v4{-0.5f, 0.5f, -0.5f, 1}, v4{0.5f, 0.5f, -0.5f, 1},
					v4{-0.5f, -0.5f, 0.5f, 1}, v4{0.5f, -0.5f, 0.5f, 1},
					v4{-0.5f, 0.5f, 0.5f, 1}, v4{0.5f, 0.5f, 0.5f, 1},
				};
				auto buf_a = collision::BuildPolytopeFromPoints(tet_pts);
				auto buf_b = collision::BuildPolytopeFromPoints(cube_pts);
				auto& pa = buf_a.as<collision::ShapePolytope>();
				auto& pb = buf_b.as<collision::ShapePolytope>();

				BangTogether(pa, m4x4::Identity(), pb, m4x4::Translation(0.5f, 0, 0), true);
			}
		}
	};
}
#endif