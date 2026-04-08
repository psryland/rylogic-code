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

namespace pr::physics::tests
{
	void ForceLink_GpuCollide() {}

	PRUnitTestClass(GpuCollideTests)
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

			// Overlapping along X
			{
				collision::ShapeSphere a(1.0f), b(1.0f);
				auto sa = PackShape(a);
				auto sb = PackShape(b);
				auto a2w = m4x4::Identity();
				auto b2w = m4x4::Translation(1.5f, 0, 0);

				float4 axis, point; float depth;
				auto gpu = physics::SphereVsSphere(sa, a2w, sb, b2w, axis, point, depth);

				collision::Contact c;
				auto cpu = collision::SphereVsSphere(a, a2w, b, b2w, c);
				PR_EXPECT(gpu && cpu);
				PR_EXPECT(Near(depth, c.m_depth));
				PR_EXPECT(AxisMatch(axis, c.m_axis));
			}

			// Separated
			{
				collision::ShapeSphere a(1.0f), b(1.0f);
				float4 axis, point; float depth;
				PR_EXPECT(!physics::SphereVsSphere(PackShape(a), m4x4::Identity(), PackShape(b), m4x4::Translation(5, 0, 0), axis, point, depth));
			}

			// Coincident centres (degenerate)
			{
				collision::ShapeSphere a(1.0f), b(1.0f);
				float4 axis, point; float depth;
				PR_EXPECT(!physics::SphereVsSphere(PackShape(a), m4x4::Identity(), PackShape(b), m4x4::Identity(), axis, point, depth));
			}

			// Diagonal overlap, different radii
			{
				collision::ShapeSphere a(2.0f), b(1.0f);
				auto sa = PackShape(a);
				auto sb = PackShape(b);
				auto a2w = m4x4::Identity();
				auto b2w = m4x4::Translation(1, 1, 1);

				float4 axis, point; float depth;
				auto gpu = physics::SphereVsSphere(sa, a2w, sb, b2w, axis, point, depth);

				collision::Contact c;
				auto cpu = collision::SphereVsSphere(a, a2w, b, b2w, c);
				PR_EXPECT(gpu && cpu);
				PR_EXPECT(Near(depth, c.m_depth));
				PR_EXPECT(AxisMatch(axis, c.m_axis));
			}
		}

		// ---- Sphere vs Box ----
		PRUnitTestMethod(SphereVsBox)
		{
			using namespace pr;
			using namespace pr::hlsl;

			// Separated: sphere at x=2, radius=0.5, box extends to x=1
			{
				collision::ShapeSphere sph(0.5f);
				collision::ShapeBox box(v4(1, 1, 1, 0));
				float4 axis, point; float depth;
				PR_EXPECT(!physics::SphereVsBox(PackShape(sph), m4x4::Translation(2.0f, 0, 0), PackShape(box), m4x4::Identity(), axis, point, depth));
			}

			// Overlapping face
			{
				collision::ShapeSphere sph(0.5f);
				collision::ShapeBox box(v4(1, 1, 1, 0));
				auto sph2w = m4x4::Translation(1.3f, 0, 0);
				auto box2w = m4x4::Identity();

				float4 axis, point; float depth;
				auto gpu = physics::SphereVsBox(PackShape(sph), sph2w, PackShape(box), box2w, axis, point, depth);

				collision::Contact c;
				auto cpu = collision::BoxVsSphere(box, box2w, sph, sph2w, c);
				PR_EXPECT(gpu && cpu);
				PR_EXPECT(Near(depth, c.m_depth));
			}

			// Sphere near box corner
			{
				collision::ShapeSphere sph(1.0f);
				collision::ShapeBox box(v4(1, 1, 1, 0));
				auto sph2w = m4x4::Translation(1.5f, 1.5f, 0);
				auto box2w = m4x4::Identity();

				float4 axis, point; float depth;
				auto gpu = physics::SphereVsBox(PackShape(sph), sph2w, PackShape(box), box2w, axis, point, depth);

				collision::Contact c;
				auto cpu = collision::BoxVsSphere(box, box2w, sph, sph2w, c);
				PR_EXPECT(gpu && cpu);
				PR_EXPECT(Near(depth, c.m_depth));
			}

			// Sphere centre inside box
			{
				collision::ShapeSphere sph(0.1f);
				collision::ShapeBox box(v4(2, 2, 2, 0));
				auto sph2w = m4x4::Translation(0.5f, 0, 0);
				auto box2w = m4x4::Identity();

				float4 axis, point; float depth;
				auto gpu = physics::SphereVsBox(PackShape(sph), sph2w, PackShape(box), box2w, axis, point, depth);

				collision::Contact c;
				auto cpu = collision::BoxVsSphere(box, box2w, sph, sph2w, c);
				PR_EXPECT(gpu && cpu);
				PR_EXPECT(Near(depth, c.m_depth, 0.05f));
			}
		}

		// ---- Sphere vs Line ----
		PRUnitTestMethod(SphereVsLine)
		{
			using namespace pr;
			using namespace pr::hlsl;

			// Sphere near midpoint of a Z-aligned line
			{
				collision::ShapeSphere sph(0.5f);
				collision::ShapeLine line(2.0f, 0.1f);
				auto sph2w = m4x4::Translation(0.4f, 0, 0);
				auto line2w = m4x4::Identity();

				float4 axis, point; float depth;
				auto gpu = physics::SphereVsLine(PackShape(sph), sph2w, PackShape(line), line2w, axis, point, depth);

				collision::Contact c;
				auto cpu = collision::LineVsSphere(line, line2w, sph, sph2w, c);
				PR_EXPECT(gpu && cpu);
				PR_EXPECT(Near(depth, c.m_depth));
			}

			// Sphere near endpoint
			{
				collision::ShapeSphere sph(0.5f);
				collision::ShapeLine line(1.0f, 0.1f);
				auto sph2w = m4x4::Translation(0.3f, 0, 1.3f);
				auto line2w = m4x4::Identity();

				float4 axis, point; float depth;
				auto gpu = physics::SphereVsLine(PackShape(sph), sph2w, PackShape(line), line2w, axis, point, depth);

				collision::Contact c;
				auto cpu = collision::LineVsSphere(line, line2w, sph, sph2w, c);
				PR_EXPECT(gpu == cpu);
				if (gpu) PR_EXPECT(Near(depth, c.m_depth));
			}

			// Separated
			{
				collision::ShapeSphere sph(0.5f);
				collision::ShapeLine line(1.0f, 0.1f);
				float4 axis, point; float depth;
				PR_EXPECT(!physics::SphereVsLine(PackShape(sph), m4x4::Translation(3, 0, 0), PackShape(line), m4x4::Identity(), axis, point, depth));
			}
		}

		// ---- Line vs Line ----
		PRUnitTestMethod(LineVsLine)
		{
			using namespace pr;
			using namespace pr::hlsl;

			// Two perpendicular lines crossing near origin
			{
				collision::ShapeLine a(2.0f, 0.2f), b(2.0f, 0.2f);
				auto a2w = m4x4::Identity();
				auto b2w = m4x4::Transform(v4(0, 1, 0, 0), float(math::constants<float>::tau_by_4), v4(0, 0.3f, 0, 1));

				float4 axis, point; float depth;
				auto gpu = physics::LineVsLine(PackShape(a), a2w, PackShape(b), b2w, axis, point, depth);

				collision::Contact c;
				auto cpu = collision::LineVsLine(a, a2w, b, b2w, c);
				PR_EXPECT(gpu == cpu);
				if (gpu) PR_EXPECT(Near(depth, c.m_depth));
			}

			// Parallel lines, close
			{
				collision::ShapeLine a(2.0f, 0.3f), b(2.0f, 0.3f);
				auto a2w = m4x4::Identity();
				auto b2w = m4x4::Translation(0.5f, 0, 0);

				float4 axis, point; float depth;
				auto gpu = physics::LineVsLine(PackShape(a), a2w, PackShape(b), b2w, axis, point, depth);

				collision::Contact c;
				auto cpu = collision::LineVsLine(a, a2w, b, b2w, c);
				PR_EXPECT(gpu == cpu);
				if (gpu) PR_EXPECT(Near(depth, c.m_depth));
			}

			// Separated
			{
				collision::ShapeLine a(1.0f, 0.1f), b(1.0f, 0.1f);
				float4 axis, point; float depth;
				PR_EXPECT(!physics::LineVsLine(PackShape(a), m4x4::Identity(), PackShape(b), m4x4::Translation(5, 0, 0), axis, point, depth));
			}
		}

		// ---- Line vs Box ----
		PRUnitTestMethod(LineVsBox)
		{
			using namespace pr;
			using namespace pr::hlsl;

			// Line along Z approaching box face
			{
				collision::ShapeLine line(1.0f, 0.2f);
				collision::ShapeBox box(v4(1, 1, 1, 0));
				auto line2w = m4x4::Translation(1.1f, 0, 0);
				auto box2w = m4x4::Identity();

				float4 axis, point; float depth;
				auto gpu = physics::LineVsBox(PackShape(line), line2w, PackShape(box), box2w, axis, point, depth);

				collision::Contact c;
				auto cpu = collision::LineVsBox(line, line2w, box, box2w, c);
				PR_EXPECT(gpu == cpu);
				if (gpu && cpu) PR_EXPECT(Near(depth, c.m_depth, 0.05f));
			}

			// Separated
			{
				collision::ShapeLine line(1.0f, 0.1f);
				collision::ShapeBox box(v4(1, 1, 1, 0));
				float4 axis, point; float depth;
				PR_EXPECT(!physics::LineVsBox(PackShape(line), m4x4::Translation(5, 0, 0), PackShape(box), m4x4::Identity(), axis, point, depth));
			}
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
		}
	};
}
#endif