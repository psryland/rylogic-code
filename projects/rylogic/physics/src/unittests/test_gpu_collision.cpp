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
		// Expected contact for collision cases. Use NaN to skip checking a field. Axis sign is checked via AxisMatch (allows flip).
		// Pass nullptr for no-collision cases. Pass &Collides for collision cases where expected values are TBD.
		struct Expected
		{
			float depth;
			v4 axis;
			v4 point;
		};

		// Placeholder for "collides, but expected values TBD" — only CPU==GPU comparison is checked
		static inline Expected Collides = {NAN, v4(NAN, 0, 0, 0), v4(NAN, 0, 0, 0)};

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
			auto BangTogether = [](collision::ShapeSphere const& a, m4x4 a2w, collision::ShapeSphere const& b, m4x4 b2w, Expected const* expected)
			{
				bool should_collide = expected != nullptr;

				collision::Contact c;
				auto is_contact_cpu = collision::SphereVsSphere(a, a2w, b, b2w, c);

				float4 axis, point; float depth;
				auto is_contact_gpu = physics::SphereVsSphere(PackShape(a), a2w, PackShape(b), b2w, axis, point, depth);

				PR_EXPECT(should_collide == is_contact_gpu);
				PR_EXPECT(is_contact_gpu == is_contact_cpu);
				if (should_collide)
				{
					PR_EXPECT(Near(c.m_depth, depth));
					PR_EXPECT(FEql(c.m_point, point));
					PR_EXPECT(AxisMatch(c.m_axis, axis));

					// Check against analytically expected values
					PR_EXPECT(Near(expected->depth, depth));
					PR_EXPECT(AxisMatch(expected->axis, axis));
					PR_EXPECT(FEql(expected->point, point));
				}
			};

			// Overlapping along X: depth = 1+1-1.5 = 0.5
			// axis = (1,0,0), point = midpoint of surfaces = (0.75,0,0)
			{
				auto exp = Expected{0.5f, v4(1, 0, 0, 0), v4(0.75f, 0, 0, 1)};
				BangTogether(
					collision::ShapeSphere(1.0f), m4x4::Identity(),
					collision::ShapeSphere(1.0f), m4x4::Translation(1.5f, 0, 0),
					&exp);
			}

			// Separated
			{
				BangTogether(
					collision::ShapeSphere(1.0f), m4x4::Identity(),
					collision::ShapeSphere(1.0f), m4x4::Translation(5, 0, 0),
					nullptr);
			}

			// Diagonal overlap, different radii: depth = 2+1-sqrt(3) ≈ 1.268
			// axis = normalise(1,1,1), point at (radius_a - depth/2) from A along axis
			{
				auto dep = 3.0f - Sqrt(3.0f);
				auto n = Normalise(v4(1, 1, 1, 0));
				auto exp = Expected{dep, n, (n * (2.0f - dep * 0.5f)).w1()};
				BangTogether(
					collision::ShapeSphere(2.0f), m4x4::Identity(),
					collision::ShapeSphere(1.0f), m4x4::Translation(1, 1, 1),
					&exp);
			}

			// Both translated, overlapping along Y: depth = 0.5+0.3-0.6 = 0.2
			// axis = (0,1,0), point = (1, 2+0.5-0.1, 3) = (1, 2.4, 3)
			{
				auto exp = Expected{0.2f, v4(0, 1, 0, 0), v4(1, 2.4f, 3, 1)};
				BangTogether(
					collision::ShapeSphere(0.5f), m4x4::Translation(1, 2, 3),
					collision::ShapeSphere(0.3f), m4x4::Translation(1, 2.6f, 3),
					&exp);
			}
		}

		// ---- Sphere vs Box ----
		PRUnitTestMethod(SphereVsBox)
		{
			using namespace pr;
			using namespace pr::hlsl;

			// GPU takes (sphere, box), CPU takes (box, sphere) — argument order differs
			auto BangTogether = [](collision::ShapeSphere const& sph, m4x4 sph2w, collision::ShapeBox const& box, m4x4 box2w, Expected const* expected)
			{
				bool should_collide = expected != nullptr;

				collision::Contact c;
				auto is_contact_cpu = collision::BoxVsSphere(box, box2w, sph, sph2w, c);

				float4 axis, point; float depth;
				auto is_contact_gpu = physics::SphereVsBox(PackShape(sph), sph2w, PackShape(box), box2w, axis, point, depth);

				PR_EXPECT(should_collide == is_contact_gpu);
				PR_EXPECT(is_contact_gpu == is_contact_cpu);
				if (should_collide)
				{
					PR_EXPECT(Near(c.m_depth, depth));
					PR_EXPECT(FEql(c.m_point, point));
					PR_EXPECT(AxisMatch(c.m_axis, axis));

					// Check against analytically expected values
					if (!std::isnan(expected->depth)) PR_EXPECT(Near(expected->depth, depth));
					if (!std::isnan(expected->axis.x)) PR_EXPECT(AxisMatch(expected->axis, axis));
					if (!std::isnan(expected->point.x)) PR_EXPECT(FEql(expected->point, point));
				}
			};

			// Separated: sphere at x=2, radius=0.5, box face at x=0.5, gap=1.0
			{
				BangTogether(
					collision::ShapeSphere(0.5f), m4x4::Translation(2.0f, 0, 0),
					collision::ShapeBox(v4(1, 1, 1, 0)), m4x4::Identity(),
					nullptr);
			}

			// Face overlap along +X: closest_box=(0.5,0,0), dist=0.3, depth=0.2
			// axis=(1,0,0), midpoint=(0.5,0,0)-0.5*0.2*(1,0,0)=(0.4,0,0)
			{
				auto exp = Expected{0.2f, v4(1, 0, 0, 0), v4(0.4f, 0, 0, 1)};
				BangTogether(
					collision::ShapeSphere(0.5f), m4x4::Translation(0.8f, 0, 0),
					collision::ShapeBox(v4(1, 1, 1, 0)), m4x4::Identity(),
					&exp);
			}

			// Sphere near box corner: closest_box=(0.5,0.5,0), dist=sqrt(0.5), depth=1-sqrt(0.5)
			// axis=normalise(0.5,0.5,0), midpoint shifts half-depth inward from box corner
			{
				auto dist = Sqrt(0.5f);
				auto dep = 1.0f - dist;
				auto n = Normalise(v4(0.5f, 0.5f, 0, 0));
				auto exp = Expected{dep, n, v4(0.5f, 0.5f, 0, 1) - (0.5f * dep) * n};
				BangTogether(
					collision::ShapeSphere(1.0f), m4x4::Translation(1.0f, 1.0f, 0),
					collision::ShapeBox(v4(1, 1, 1, 0)), m4x4::Identity(),
					&exp);
			}

			// Sphere centre inside box (non-degenerate): sph(r=0.1) at (0.2,0,0), box(half=1)
			// Nearest face is +X at x=1, dist=0.8. depth=0.8+0.1=0.9
			// axis=(1,0,0), face_pt=(1,0,0), midpoint=(1,0,0)-0.5*0.9*(1,0,0)=(0.55,0,0)
			{
				auto exp = Expected{0.9f, v4(1, 0, 0, 0), v4(0.55f, 0, 0, 1)};
				BangTogether(
					collision::ShapeSphere(0.1f), m4x4::Translation(0.2f, 0, 0),
					collision::ShapeBox(v4(2, 2, 2, 0)), m4x4::Identity(),
					&exp);
			}

			// Both transformed: sph(r=0.3) at (1.5,2,0), box(half=0.5) at (1.2,2,0)
			// Sphere centre is inside box. Nearest face is +X at 1.7, face_dist=0.2. depth=0.3+0.2=0.5
			// axis=(1,0,0), face_pt=(1.7,2,0), midpoint=(1.7,2,0)-0.5*0.5*(1,0,0)=(1.45,2,0)
			{
				auto exp = Expected{0.5f, v4(1, 0, 0, 0), v4(1.45f, 2, 0, 1)};
				BangTogether(
					collision::ShapeSphere(0.3f), m4x4::Translation(1.5f, 2.0f, 0),
					collision::ShapeBox(v4(1, 1, 1, 0)), m4x4::Translation(1.2f, 2.0f, 0),
					&exp);
			}

			// Rotated box corner: sph(r=0.5) at origin, box(half=0.5) rotated 45°Z at (1,0,0)
			// In box local frame: sphere at (-cos45, sin45, 0). Clamped to (-0.5, 0.5, 0).
			// delta=(-cos45+0.5, sin45-0.5, 0), dist=sqrt(2)*|sin45-0.5|, depth=0.5-dist
			{
				auto c45 = Sqrt(0.5f);
				auto delta_l = v4(-c45 + 0.5f, c45 - 0.5f, 0, 0);
				auto dist = Length(delta_l);
				auto dep = 0.5f - dist;
				auto local_n = Normalise(delta_l);

				// Transform clamped point and normal to world space (rotation 45°Z at (1,0,0))
				auto box2w = m4x4::Transform(m3x3::RotationDeg(0, 0, 45.0f), v4(1, 0, 0, 1));
				auto clamped_w = box2w * v4(-0.5f, 0.5f, 0, 1);
				auto axis_w = Normalise((box2w * local_n.w0()).w0());
				auto midpoint = clamped_w - (0.5f * dep) * axis_w;

				auto exp = Expected{dep, axis_w, midpoint};
				BangTogether(
					collision::ShapeSphere(0.5f), m4x4::Identity(),
					collision::ShapeBox(v4(1, 1, 1, 0)), box2w,
					&exp);
			}

			// Face overlap matching detector test: box(half=1) at identity, sphere(r=0.8) at x=1.5
			// closest_box=(1,0,0), dist=0.5, depth=0.3, axis=(1,0,0), midpoint=(0.85,0,0)
			{
				auto exp = Expected{0.3f, v4(1, 0, 0, 0), v4(0.85f, 0, 0, 1)};
				BangTogether(
					collision::ShapeSphere(0.8f), m4x4::Translation(1.5f, 0, 0),
					collision::ShapeBox(v4(2, 2, 2, 0)), m4x4::Identity(),
					&exp);
			}

			// Rotated box vs sphere: box(half=1,2,1) rotated 45° around Z, sphere(r=0.5) at x=1.8
			// In box local: sphere at (1.8*c45, -1.8*c45, 0). Clamped x to 1. delta=(0.9√2-1, 0, 0). depth=0.5-(0.9√2-1)=1.5-0.9√2
			{
				auto c45 = Sqrt(0.5f);
				auto box2w = m4x4::Transform(m3x3::RotationDeg(0, 0, 45.0f), v4::Origin());
				auto local_sph = v4(1.8f * c45, -1.8f * c45, 0, 0);
				auto dist = 1.8f * c45 - 1.0f; // delta_x only (clamped at x=1)
				auto dep = 0.5f - dist;
				auto world_n = Normalise((box2w * v4(1, 0, 0, 0)).w0()); // local normal (1,0,0) → world
				auto clamped_w = box2w * v4(1.0f, local_sph.y, 0, 1);
				auto exp = Expected{dep, world_n, clamped_w - (0.5f * dep) * world_n};
				BangTogether(
					collision::ShapeSphere(0.5f), m4x4::Translation(1.8f, 0, 0),
					collision::ShapeBox(v4(2, 4, 2, 0)), box2w,
					&exp);
			}
		}

		// ---- Sphere vs Line ----
		PRUnitTestMethod(SphereVsLine)
		{
			using namespace pr;
			using namespace pr::hlsl;

			// GPU takes (sphere, line), CPU takes (line, sphere) — argument order differs
			auto BangTogether = [](collision::ShapeSphere const& sph, m4x4 sph2w, collision::ShapeLine const& line, m4x4 line2w, Expected const* expected)
			{
				bool should_collide = expected != nullptr;

				collision::Contact c;
				auto is_contact_cpu = collision::LineVsSphere(line, line2w, sph, sph2w, c);

				float4 axis, point; float depth;
				auto is_contact_gpu = physics::SphereVsLine(PackShape(sph), sph2w, PackShape(line), line2w, axis, point, depth);

				PR_EXPECT(should_collide == is_contact_gpu);
				PR_EXPECT(is_contact_gpu == is_contact_cpu);
				if (should_collide)
				{
					PR_EXPECT(Near(c.m_depth, depth));
					PR_EXPECT(FEql(c.m_point, point));
					PR_EXPECT(AxisMatch(axis, c.m_axis));

					PR_EXPECT(Near(expected->depth, depth));
					PR_EXPECT(AxisMatch(expected->axis, axis));
					PR_EXPECT(FEql(expected->point, point));
				}
			};

			// Sphere near midpointof Z-aligned line
			// ShapeLine(2.0, 0.1) → half_len=1.0, thick_radius=0.05
			// depth = 0.5+0.05-0.4 = 0.15, axis=(1,0,0), point = midpoint on line axis + half-depth shift
			{
				auto exp = Expected{0.15f, v4(1, 0, 0, 0), v4(0.225f, 0, 0, 1)};
				BangTogether(
					collision::ShapeSphere(0.5f), m4x4::Translation(0.4f, 0, 0),
					collision::ShapeLine(2.0f, 0.1f), m4x4::Identity(),
					&exp);
			}

			// Sphere near endpoint (separated)
			{
				BangTogether(
					collision::ShapeSphere(0.5f), m4x4::Translation(0.3f, 0, 1.3f),
					collision::ShapeLine(1.0f, 0.1f), m4x4::Identity(),
					nullptr);
			}

			// Separated
			{
				BangTogether(
					collision::ShapeSphere(0.5f), m4x4::Translation(3, 0, 0),
					collision::ShapeLine(1.0f, 0.1f), m4x4::Identity(),
					nullptr);
			}

			// Both translated: depth = 0.5+0.05-0.4 = 0.15, axis=(1,0,0)
			{
				auto exp = Expected{0.15f, v4(1, 0, 0, 0), v4(2.225f, 3, 0, 1)};
				BangTogether(
					collision::ShapeSphere(0.5f), m4x4::Translation(2.4f, 3, 0),
					collision::ShapeLine(2.0f, 0.1f), m4x4::Translation(2, 3, 0),
					&exp);
			}
		}

		// ---- Line vs Line ----
		PRUnitTestMethod(LineVsLine)
		{
			using namespace pr;
			using namespace pr::hlsl;

			auto BangTogether = [](collision::ShapeLine const& a, m4x4 a2w, collision::ShapeLine const& b, m4x4 b2w, Expected const* expected)
			{
				bool should_collide = expected != nullptr;

				collision::Contact c;
				auto is_contact_cpu = collision::LineVsLine(a, a2w, b, b2w, c);

				float4 axis, point; float depth;
				auto is_contact_gpu = physics::LineVsLine(PackShape(a), a2w, PackShape(b), b2w, axis, point, depth);

				PR_EXPECT(should_collide == is_contact_gpu);
				PR_EXPECT(is_contact_gpu == is_contact_cpu);
				if (should_collide)
				{
					PR_EXPECT(Near(c.m_depth, depth));
					PR_EXPECT(FEql(c.m_point, point));
					PR_EXPECT(AxisMatch(axis, c.m_axis));

					PR_EXPECT(Near(expected->depth, depth));
					PR_EXPECT(AxisMatch(expected->axis, axis));
					PR_EXPECT(FEql(expected->point, point));
				}
			};

			// Perpendicular lines, separatedby 0.3 (thickness 0.1+0.1 < 0.3)
			{
				BangTogether(
					collision::ShapeLine(2.0f, 0.2f), m4x4::Identity(),
					collision::ShapeLine(2.0f, 0.2f), m4x4::Transform(v4(0, 1, 0, 0), float(math::constants<float>::tau_by_4), v4(0, 0.3f, 0, 1)),
					nullptr);
			}

			// Perpendicular lines, colliding: depth = 0.1+0.1-0.15 = 0.05
			// Line A along Z at origin, line B along X at y=0.15. Closest points at origin and (0,0.15,0).
			// axis = (0,1,0), point = midpoint = (0, 0.075, 0)
			{
				auto exp = Expected{0.05f, v4(0, 1, 0, 0), v4(0, 0.075f, 0, 1)};
				BangTogether(
					collision::ShapeLine(2.0f, 0.2f), m4x4::Identity(),
					collision::ShapeLine(2.0f, 0.2f), m4x4::Transform(v4(0, 1, 0, 0), float(math::constants<float>::tau_by_4), v4(0, 0.15f, 0, 1)),
					&exp);
			}

			// Parallel lines, separated
			{
				BangTogether(
					collision::ShapeLine(2.0f, 0.3f), m4x4::Identity(),
					collision::ShapeLine(2.0f, 0.3f), m4x4::Translation(0.5f, 0, 0),
					nullptr);
			}

			// Parallel lines, overlapping: depth = 0.15+0.15-0.2 = 0.1
			// axis = (1,0,0), point = midpoint = (0.1, 0, 0)
			{
				auto exp = Expected{0.1f, v4(1, 0, 0, 0), v4(0.1f, 0, 0, 1)};
				BangTogether(
					collision::ShapeLine(2.0f, 0.3f), m4x4::Identity(),
					collision::ShapeLine(2.0f, 0.3f), m4x4::Translation(0.2f, 0, 0),
					&exp);
			}

			// Separated (far apart)
			{
				BangTogether(
					collision::ShapeLine(1.0f, 0.1f), m4x4::Identity(),
					collision::ShapeLine(1.0f, 0.1f), m4x4::Translation(5, 0, 0),
					nullptr);
			}
		}

		// ---- Line vs Box ----
		PRUnitTestMethod(LineVsBox)
		{
			using namespace pr;
			using namespace pr::hlsl;

			auto BangTogether = [](collision::ShapeLine const& line, m4x4 line2w, collision::ShapeBox const& box, m4x4 box2w, Expected const* expected)
			{
				bool should_collide = expected != nullptr;

				collision::Contact c;
				auto is_contact_cpu = collision::LineVsBox(line, line2w, box, box2w, c);

				float4 axis, point; float depth;
				auto is_contact_gpu = physics::LineVsBox(PackShape(line), line2w, PackShape(box), box2w, axis, point, depth);

				PR_EXPECT(should_collide == is_contact_gpu);
				PR_EXPECT(is_contact_gpu == is_contact_cpu);
				if (should_collide)
				{
					PR_EXPECT(Near(c.m_depth, depth, 0.05f));
					PR_EXPECT(FEql(c.m_point, point));
					PR_EXPECT(AxisMatch(axis, c.m_axis));

					PR_EXPECT(Near(expected->depth, depth, 0.05f));
					PR_EXPECT(AxisMatch(expected->axis, axis));
					PR_EXPECT(FEql(expected->point, point));
				}
			};

			// Line along Z approaching box face: depth = thick_r - gap = 0.1 - 0.05 = 0.05
			// axis=(1,0,0), line surface at 0.55-0.1=0.45, box face at 0.5, midpoint=(0.475,0,0)
			{
				auto exp = Expected{0.05f, v4(1, 0, 0, 0), v4(0.475f, 0, 0, 1)};
				BangTogether(
					collision::ShapeLine(1.0f, 0.2f), m4x4::Translation(0.55f, 0, 0),
					collision::ShapeBox(v4(1, 1, 1, 0)), m4x4::Identity(),
					&exp);
			}

			// Separated
			{
				BangTogether(
					collision::ShapeLine(1.0f, 0.1f), m4x4::Translation(5, 0, 0),
					collision::ShapeBox(v4(1, 1, 1, 0)), m4x4::Identity(),
					nullptr);
			}
		}

		// ---- Box vs Box ----
		PRUnitTestMethod(BoxVsBox)
		{
			using namespace pr;
			using namespace pr::hlsl;

			// Collide two boxes, and compare the CPU and GPU results
			auto BangTogether = [](collision::ShapeBox const& a, m4x4 a2w, collision::ShapeBox const& b, m4x4 b2w, Expected const* expected)
			{
				bool should_collide = expected != nullptr;

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

					PR_EXPECT(Near(expected->depth, depth));
					PR_EXPECT(AxisMatch(expected->axis, axis));
					PR_EXPECT(FEql(expected->point, point));
				}
			};

			// Axis-aligned overlap along X
			{
				auto exp = Expected{ 0.01f, v4(1, 0, 0, 0), v4(0.495f, 0, 0, 1) };
				BangTogether(
					collision::ShapeBox(v4(1, 1, 1, 0)), m4x4::Identity(),
					collision::ShapeBox(v4(1, 1, 1, 0)), m4x4::Translation(0.99f, 0, 0),
					&exp);
			}

			// Separated
			{
				BangTogether(
					collision::ShapeBox(v4(1, 1, 1, 0)), m4x4::Identity(),
					collision::ShapeBox(v4(1, 1, 1, 0)), m4x4::Translation(1.1f, 0, 0),
					nullptr);
			}

			// Rotated box overlap (45 degrees around Z)
			{
				auto exp = Expected{0.0571218729f, v4(1, 0, 0, 0), v4(0.471447f, 0, 0, 1)};
				BangTogether(
					collision::ShapeBox(v4(1, 1, 1, 0)), m4x4::Identity(),
					collision::ShapeBox(v4(1, 1, 1, 0)), m4x4::Transform(m3x3::RotationDeg(0, 0, 45.0f), v4(1.15f, 0, 0, 1)),
					&exp);
			}

			// Different sized boxes
			{
				auto exp = Expected{0.01f, v4(1, 0, 0, 0), v4(0.995f, 0, 0, 1)};
				BangTogether(
					collision::ShapeBox(v4(2, 0.5f, 1, 0)), m4x4::Identity(),
					collision::ShapeBox(v4(0.5f, 2, 1, 0)), m4x4::Translation(1.24f, 0, 0),
					&exp);
			}

			// Corner contact
			{
				auto exp = Expected{0.0285550356f, v4(1,0,0,0), v4(0.985723f, 0.0303301f, 0.271447f, 1)};
				BangTogether(
					collision::ShapeBox(v4(2, 0.5f, 1, 0)), m4x4::Identity(),
					collision::ShapeBox(v4(0.5f, 2, 1, 0)), m4x4::Transform(m3x3::RotationDeg(0, 45.0f, 45.0f), v4(1.95f, -0.5f, 0, 1)),
					&exp);
			}

			// Edge contact
			{
				auto exp = Expected{0.0247642994f, v4(0.816497f, 0.57735f, 0, 0), v4(0.98989f, 0.242851f, 0.0631133f, 1)};
				BangTogether(
					collision::ShapeBox(v4(2, 0.5f, 1, 0)), m4x4::Identity(),
					collision::ShapeBox(v4(0.5f, 2, 1, 0)), m4x4::Transform(m3x3::RotationDeg(0, 45.0f, 45.0f), v4(1.75f, 0, 0, 1)),
					&exp);
			}

			// Both rotated, face-edge overlap
			{
				auto exp = Expected{0.275268674f, v4(0.965926f,-0.258819f,0,0), v4(0.207912f,-0.163214f,0,1)};
				BangTogether(
					collision::ShapeBox(v4(1, 1, 1, 0)), m4x4::Transform(m3x3::RotationDeg(0, 0, 20.0f), v4(-0.3f, 0.1f, 0, 1)),
					collision::ShapeBox(v4(1, 1, 1, 0)), m4x4::Transform(m3x3::RotationDeg(0, 0, -15.0f), v4(0.6f, -0.1f, 0, 1)),
					&exp);
			}

			// Both rotated, separated
			{
				BangTogether(
					collision::ShapeBox(v4(1, 1, 1, 0)), m4x4::Transform(m3x3::RotationDeg(0, 0, 20.0f), v4(-0.3f, 0.1f, 0, 1)),
					collision::ShapeBox(v4(1, 1, 1, 0)), m4x4::Transform(m3x3::RotationDeg(0, 0, -15.0f), v4(1.5f, 0, 0, 1)),
					nullptr);
			}

			// Both rotated around different axes, overlapping
			{
				auto exp = Expected{0.0571082830f, v4(1,0,0,0), v4(0.471447f,0,0,1)};
				BangTogether(
					collision::ShapeBox(v4(1, 1, 1, 0)), m4x4::Transform(m3x3::RotationDeg(30.0f, 0, 0), v4(0, 0, 0, 1)),
					collision::ShapeBox(v4(1, 1, 1, 0)), m4x4::Transform(m3x3::RotationDeg(0, 0, 45.0f), v4(1.15f, 0, 0, 1)),
					&exp);
			}

			// Both rotated around all three axes
			{
				auto exp = Expected{0.0951071978f, v4(0.916575f,-0.164476f,-0.364469f,0), v4(0.285301f,-0.122901f,-0.0770606f,1)};
				BangTogether(
					collision::ShapeBox(v4(1, 1, 1, 0)), m4x4::Transform(m3x3::RotationDeg(10.0f, 20.0f, 30.0f), v4(-0.4f, 0.1f, -0.1f, 1)),
					collision::ShapeBox(v4(1, 1, 1, 0)), m4x4::Transform(m3x3::RotationDeg(-15.0f, 25.0f, -10.0f), v4(0.9f, -0.1f, 0.2f, 1)),
					&exp);
			}

			// ---- Edge-edge contact ----
			// Two elongated boxes crossing like an X, edges closest
			{
				auto exp = Expected{0.05f, v4(1,0,0,0), v4(0.075f,0,0,1)};
				BangTogether(
					collision::ShapeBox(v4(0.2f, 0.2f, 3, 0)), m4x4::Identity(),
					collision::ShapeBox(v4(0.2f, 0.2f, 3, 0)), m4x4::Transform(m3x3::RotationDeg(90.0f, 0, 0), v4(0.15f, 0, 0, 1)),
					&exp);
			}

			// Two unit boxes positioned so an edge-edge cross-product axis is the minimum
			{
				auto exp = Expected{0.464213550f, v4(0,1,0,0), v4(0,0.475f,0,1)};
				BangTogether(
					collision::ShapeBox(v4(1, 1, 1, 0)), m4x4::Transform(m3x3::RotationDeg(0, 0, 45.0f), v4(0, 0, 0, 1)),
					collision::ShapeBox(v4(1, 1, 1, 0)), m4x4::Transform(m3x3::RotationDeg(45.0f, 0, 0), v4(0, 0.95f, 0, 1)),
					&exp);
			}

			// ---- Vertex-face contact ----

			// B's corner pokes into A's face (B rotated around two axes)
			{
				auto exp = Expected{0.366496146f, v4(1,0,0,0), v4(0.316753f,0,-0.288681f,1)};
				BangTogether(
					collision::ShapeBox(v4(1, 1, 1, 0)), m4x4::Identity(),
					collision::ShapeBox(v4(1, 1, 1, 0)), m4x4::Transform(m3x3::RotationDeg(45.0f, 35.264f, 0), v4(0.95f, 0, 0, 1)),
					&exp);
			}

			// A's corner pokes into B's face (A rotated, B at identity)
			{
				auto exp = Expected{0.366496146f, v4(-1,0,0,0), v4(-0.316753f,0,0.288681f,1)};
				BangTogether(
					collision::ShapeBox(v4(1, 1, 1, 0)), m4x4::Transform(m3x3::RotationDeg(45.0f, 35.264f, 0), v4(-0.95f, 0, 0, 1)),
					collision::ShapeBox(v4(1, 1, 1, 0)), m4x4::Identity(),
					&exp);
			}

			// ---- Near-degenerate cases ----

			// Nearly touching (very small overlap)
			{
				auto exp = Expected{0.001f, v4(1,0,0,0), v4(0.4995f,0,0,1)};
				BangTogether(
					collision::ShapeBox(v4(1, 1, 1, 0)), m4x4::Identity(),
					collision::ShapeBox(v4(1, 1, 1, 0)), m4x4::Translation(0.999f, 0, 0),
					&exp);
			}

			// Deep overlap (one box mostly inside the other)
			{
				auto exp = Expected{1.05f, v4(1,0,0,0), v4(0.475f,0.025f,-0.01f,1)};
				BangTogether(
					collision::ShapeBox(v4(2, 2, 2, 0)), m4x4::Identity(),
					collision::ShapeBox(v4(0.3f, 0.3f, 0.3f, 0)), m4x4::Translation(0.1f, 0.05f, -0.02f),
					&exp);
			}

			// Boxes touching at an edge, both translated off-origin
			{
				auto exp = Expected{0.00153195858f, v4(0.866025f,0.5f,0,0), v4(3.68181f,-0.182453f,1,1)};
				BangTogether(
					collision::ShapeBox(v4(1, 1, 1, 0)), m4x4::Transform(m3x3::RotationDeg(0, 0, 30.0f), v4(3, 0, 1, 1)),
					collision::ShapeBox(v4(1, 1, 1, 0)), m4x4::Transform(m3x3::RotationDeg(0, 0, -20.0f), v4(4.322f, 0.116f, 1, 1)),
					&exp);
			}

			// ---- Asymmetric shapes, both transformed ----
			{
				auto exp = Expected{0.199977040f, v4(0.858356f,0.132788f,-0.495572f,0), v4(0.914174f,-0.186723f,0.899549f,1)};
				BangTogether(
					collision::ShapeBox(v4(2, 2, 0.2f, 0)), m4x4::Transform(m3x3::RotationDeg(15.0f, 0, 0), v4(0, 0, 1, 1)),
					collision::ShapeBox(v4(0.4f, 0.4f, 3, 0)), m4x4::Transform(m3x3::RotationDeg(0, 30.0f, 0), v4(0.8f, 0, 0.5f, 1)),
					&exp);
			}
		}

		// ---- Polytope vs Polytope ----
		PRUnitTestMethod(PolytopeVsPolytope)
		{
			using namespace pr;
			using namespace pr::hlsl;

			// Compare CPU GJK and GPU GJK results. GJK/EPA has wider tolerances than SAT.
			auto BangTogether = [](collision::ShapePolytope const& a, m4x4 a2w, collision::ShapePolytope const& b, m4x4 b2w, Expected const* expected)
			{
				bool should_collide = expected != nullptr;

				collision::Contact c;
				auto is_contact_cpu = collision::GjkCollide(a, a2w, b, b2w, c);

				// Build the vertex buffer for the GPU
				hlsl::StructuredBuffer<float4> verts;
				auto sa = PackShape(a, static_cast<int>(verts.size()));
				for (auto const* v = a.vert_beg(); v != a.vert_end(); ++v)
					verts.push_back(*v);

				auto sb = PackShape(b, static_cast<int>(verts.size()));
				for (auto const* v = b.vert_beg(); v != b.vert_end(); ++v)
					verts.push_back(*v);

				float4 axis, point; float depth; int gjk_iters, epa_iters;
				auto is_contact_gpu = physics::GjkCollide(sa, a2w, sb, b2w, verts, axis, point, depth, gjk_iters, epa_iters);

				PR_EXPECT(should_collide == is_contact_gpu);
				PR_EXPECT(is_contact_gpu == is_contact_cpu);
				if (should_collide)
				{
					PR_EXPECT(Near(c.m_depth, depth));
					PR_EXPECT(FEql(c.m_point, point));
					PR_EXPECT(AxisMatch(c.m_axis, axis));

					PR_EXPECT(Near(expected->depth, depth));
					PR_EXPECT(AxisMatch(expected->axis, axis));
					PR_EXPECT(FEql(expected->point, point));
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
				auto exp = Expected{0.0f, v4(0,0,0,0), v4(0,0,0,1)};
				BangTogether(pa, m4x4::Identity(), pb, m4x4::Translation(0.5f, 0, 0), &exp);
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

				BangTogether(pa, m4x4::Identity(), pb, m4x4::Translation(10, 0, 0), nullptr);
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
				auto exp = Expected{0.0f, v4(0,0,0,0), v4(0,0,0,1)};
				BangTogether(pa, m4x4::Identity(), pb, m4x4::Translation(1.5f, 0, 0), &exp);
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
				BangTogether(pa, a2w, pb, b2w, nullptr);
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

				auto exp = Expected{0.0f, v4(0,0,0,0), v4(0,0,0,1)};
				BangTogether(pa, m4x4::Identity(), pb, m4x4::Translation(0.5f, 0, 0), &exp);
			}
		}
	};
}
#endif