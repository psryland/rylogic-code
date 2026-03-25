//************************************
// Physics Engine
//  Copyright (c) Rylogic Ltd 2016
//************************************
// Unit tests that compare GPU collision functions (compiled as C++) against CPU collision functions.
#pragma once

#if PR_UNITTESTS
#include "pr/common/unittests.h"
#include "pr/math/math.h"
#include "pr/hlsl/interop.h"
#include "pr/collision/col_sphere_vs_sphere.h"
#include "pr/collision/col_box_vs_sphere.h"
#include "pr/collision/col_line_vs_sphere.h"
#include "pr/collision/col_line_vs_line.h"
#include "pr/collision/col_line_vs_box.h"
#include "pr/collision/col_box_vs_box.h"
#include "src/compute/collision.hlsli"

namespace pr::physics::tests
{
	PRUnitTestClass(GpuCollideTests)
	{
		// Shape factory helpers
		static GpuShape MakeSphere(float radius)
		{
			GpuShape s = {};
			s.s2rb = pr::m4x4::Identity();
			s.type = SHAPE_SPHERE;
			s.data = pr::hlsl::float4(radius, 0, 0, 0);
			return s;
		}
		static GpuShape MakeBox(float hx, float hy, float hz)
		{
			GpuShape s = {};
			s.s2rb = pr::m4x4::Identity();
			s.type = SHAPE_BOX;
			s.data = pr::hlsl::float4(hx, hy, hz, 0);
			return s;
		}
		static GpuShape MakeLine(float half_length, float thickness)
		{
			GpuShape s = {};
			s.s2rb = pr::m4x4::Identity();
			s.type = SHAPE_LINE;
			s.data = pr::hlsl::float4(half_length, thickness, 0, 0);
			return s;
		}

		// Build a row-major float4x4 from position (identity rotation)
		static pr::hlsl::float4x4 W2W(float x, float y, float z)
		{
			using namespace pr::hlsl;
			return float4x4(float4(1, 0, 0, 0), float4(0, 1, 0, 0), float4(0, 0, 1, 0), float4(x, y, z, 1));
		}

		// Build a row-major float4x4 from a pr::m4x4
		static pr::hlsl::float4x4 ToHlsl(pr::m4x4 const& m)
		{
			return reinterpret_cast<pr::hlsl::float4x4 const&>(m);
		}

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
				float4 axis, point; float depth;
				auto gpu = physics::SphereVsSphere(MakeSphere(1.0f), W2W(0, 0, 0), MakeSphere(1.0f), W2W(1.5f, 0, 0), axis, point, depth);

				collision::ShapeSphere a(1.0f), b(1.0f);
				collision::Contact c;
				auto cpu = collision::SphereVsSphere(a, m4x4::Identity(), b, m4x4::Transform(m4x4::Identity().rot, v4(1.5f, 0, 0, 1)), c);
				PR_EXPECT(gpu && cpu);
				PR_EXPECT(Near(depth, c.m_depth));
				PR_EXPECT(AxisMatch(axis, c.m_axis));
			}

			// Separated
			{
				float4 axis, point; float depth;
				PR_EXPECT(!physics::SphereVsSphere(MakeSphere(1.0f), W2W(0, 0, 0), MakeSphere(1.0f), W2W(5, 0, 0), axis, point, depth));
			}

			// Coincident centres (degenerate)
			{
				float4 axis, point; float depth;
				PR_EXPECT(!physics::SphereVsSphere(MakeSphere(1.0f), W2W(0, 0, 0), MakeSphere(1.0f), W2W(0, 0, 0), axis, point, depth));
			}

			// Diagonal overlap, different radii
			{
				float4 axis, point; float depth;
				auto gpu = physics::SphereVsSphere(MakeSphere(2.0f), W2W(0, 0, 0), MakeSphere(1.0f), W2W(1, 1, 1), axis, point, depth);

				collision::ShapeSphere a(2.0f), b(1.0f);
				collision::Contact c;
				auto cpu = collision::SphereVsSphere(a, m4x4::Identity(), b, m4x4::Transform(m4x4::Identity().rot, v4(1, 1, 1, 1)), c);
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
				float4 axis, point; float depth;
				PR_EXPECT(!physics::SphereVsBox(MakeSphere(0.5f), W2W(2.0f, 0, 0), MakeBox(1, 1, 1), W2W(0, 0, 0), axis, point, depth));
			}

			// Overlapping face
			{
				float4 axis, point; float depth;
				auto gpu = physics::SphereVsBox(MakeSphere(0.5f), W2W(1.3f, 0, 0), MakeBox(1, 1, 1), W2W(0, 0, 0), axis, point, depth);

				collision::ShapeBox cpu_box(v4(1, 1, 1, 0));
				collision::ShapeSphere cpu_sphere(0.5f);
				collision::Contact c;
				auto cpu = collision::BoxVsSphere(cpu_box, m4x4::Identity(), cpu_sphere, m4x4::Transform(m4x4::Identity().rot, v4(1.3f, 0, 0, 1)), c);
				PR_EXPECT(gpu && cpu);
				PR_EXPECT(Near(depth, c.m_depth));
			}

			// Sphere near box corner
			{
				float4 axis, point; float depth;
				auto gpu = physics::SphereVsBox(MakeSphere(1.0f), W2W(1.5f, 1.5f, 0), MakeBox(1, 1, 1), W2W(0, 0, 0), axis, point, depth);

				collision::ShapeBox cpu_box(v4(1, 1, 1, 0));
				collision::ShapeSphere cpu_sphere(1.0f);
				collision::Contact c;
				auto cpu = collision::BoxVsSphere(cpu_box, m4x4::Identity(), cpu_sphere, m4x4::Transform(m4x4::Identity().rot, v4(1.5f, 1.5f, 0, 1)), c);
				PR_EXPECT(gpu && cpu);
				PR_EXPECT(Near(depth, c.m_depth));
			}

			// Sphere centre inside box
			{
				float4 axis, point; float depth;
				auto gpu = physics::SphereVsBox(MakeSphere(0.1f), W2W(0.5f, 0, 0), MakeBox(2, 2, 2), W2W(0, 0, 0), axis, point, depth);

				collision::ShapeBox cpu_box(v4(2, 2, 2, 0));
				collision::ShapeSphere cpu_sphere(0.1f);
				collision::Contact c;
				auto cpu = collision::BoxVsSphere(cpu_box, m4x4::Identity(), cpu_sphere, m4x4::Transform(m4x4::Identity().rot, v4(0.5f, 0, 0, 1)), c);
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
				float4 axis, point; float depth;
				auto gpu = physics::SphereVsLine(MakeSphere(0.5f), W2W(0.4f, 0, 0), MakeLine(2.0f, 0.1f), W2W(0, 0, 0), axis, point, depth);

				collision::ShapeLine cpu_line(2.0f, 0.1f);
				collision::ShapeSphere cpu_sphere(0.5f);
				collision::Contact c;
				auto cpu = collision::LineVsSphere(cpu_line, m4x4::Identity(), cpu_sphere, m4x4::Transform(m4x4::Identity().rot, v4(0.4f, 0, 0, 1)), c);
				PR_EXPECT(gpu && cpu);
				PR_EXPECT(Near(depth, c.m_depth));
			}

			// Sphere near endpoint
			{
				float4 axis, point; float depth;
				auto gpu = physics::SphereVsLine(MakeSphere(0.5f), W2W(0.3f, 0, 1.3f), MakeLine(1.0f, 0.1f), W2W(0, 0, 0), axis, point, depth);

				collision::ShapeLine cpu_line(1.0f, 0.1f);
				collision::ShapeSphere cpu_sphere(0.5f);
				collision::Contact c;
				auto cpu = collision::LineVsSphere(cpu_line, m4x4::Identity(), cpu_sphere, m4x4::Transform(m4x4::Identity().rot, v4(0.3f, 0, 1.3f, 1)), c);
				PR_EXPECT(gpu == cpu);
				if (gpu) PR_EXPECT(Near(depth, c.m_depth));
			}

			// Separated
			{
				float4 axis, point; float depth;
				PR_EXPECT(!physics::SphereVsLine(MakeSphere(0.5f), W2W(3, 0, 0), MakeLine(1.0f, 0.1f), W2W(0, 0, 0), axis, point, depth));
			}
		}

		// ---- Line vs Line ----
		PRUnitTestMethod(LineVsLine)
		{
			using namespace pr;
			using namespace pr::hlsl;

			// Two perpendicular lines crossing near origin
			{
				auto lb_o2w = m4x4::Transform(v4(0, 1, 0, 0), float(math::constants<float>::tau_by_4), v4(0, 0.3f, 0, 1));
				float4 axis, point; float depth;
				auto gpu = physics::LineVsLine(MakeLine(2.0f, 0.2f), W2W(0, 0, 0), MakeLine(2.0f, 0.2f), ToHlsl(lb_o2w), axis, point, depth);

				collision::ShapeLine a(2.0f, 0.2f), b(2.0f, 0.2f);
				collision::Contact c;
				auto cpu = collision::LineVsLine(a, m4x4::Identity(), b, lb_o2w, c);
				PR_EXPECT(gpu == cpu);
				if (gpu) PR_EXPECT(Near(depth, c.m_depth));
			}

			// Parallel lines, close
			{
				float4 axis, point; float depth;
				auto gpu = physics::LineVsLine(MakeLine(2.0f, 0.3f), W2W(0, 0, 0), MakeLine(2.0f, 0.3f), W2W(0.5f, 0, 0), axis, point, depth);

				collision::ShapeLine a(2.0f, 0.3f), b(2.0f, 0.3f);
				collision::Contact c;
				auto cpu = collision::LineVsLine(a, m4x4::Identity(), b, m4x4::Transform(m4x4::Identity().rot, v4(0.5f, 0, 0, 1)), c);
				PR_EXPECT(gpu == cpu);
				if (gpu) PR_EXPECT(Near(depth, c.m_depth));
			}

			// Separated
			{
				float4 axis, point; float depth;
				PR_EXPECT(!physics::LineVsLine(MakeLine(1.0f, 0.1f), W2W(0, 0, 0), MakeLine(1.0f, 0.1f), W2W(5, 0, 0), axis, point, depth));
			}
		}

		// ---- Line vs Box ----
		PRUnitTestMethod(LineVsBox)
		{
			using namespace pr;
			using namespace pr::hlsl;

			// Line along Z approaching box face
			{
				float4 axis, point; float depth;
				auto gpu = physics::LineVsBox(MakeLine(1.0f, 0.2f), W2W(1.1f, 0, 0), MakeBox(1, 1, 1), W2W(0, 0, 0), axis, point, depth);

				collision::ShapeLine cpu_line(1.0f, 0.2f);
				collision::ShapeBox cpu_box(v4(1, 1, 1, 0));
				collision::Contact c;
				auto cpu = collision::LineVsBox(cpu_line, m4x4::Transform(m4x4::Identity().rot, v4(1.1f, 0, 0, 1)), cpu_box, m4x4::Identity(), c);
				PR_EXPECT(gpu == cpu);
				if (gpu && cpu) PR_EXPECT(Near(depth, c.m_depth, 0.05f));
			}

			// Separated
			{
				float4 axis, point; float depth;
				PR_EXPECT(!physics::LineVsBox(MakeLine(1.0f, 0.1f), W2W(5, 0, 0), MakeBox(1, 1, 1), W2W(0, 0, 0), axis, point, depth));
			}
		}

		// ---- Box vs Box ----
		PRUnitTestMethod(BoxVsBox)
		{
			using namespace pr;
			using namespace pr::hlsl;

			// Axis-aligned overlap along X
			{
				float4 axis, point; float depth;
				auto gpu = physics::BoxVsBox(MakeBox(1, 1, 1), W2W(0, 0, 0), MakeBox(1, 1, 1), W2W(1.5f, 0, 0), axis, point, depth);

				collision::ShapeBox a(v4(1, 1, 1, 0)), b(v4(1, 1, 1, 0));
				collision::Contact c;
				auto cpu = collision::BoxVsBox(a, m4x4::Identity(), b, m4x4::Transform(m4x4::Identity().rot, v4(1.5f, 0, 0, 1)), c);
				PR_EXPECT(gpu && cpu);
				PR_EXPECT(Near(depth, c.m_depth, 0.01f));
			}

			// Separated
			{
				float4 axis, point; float depth;
				PR_EXPECT(!physics::BoxVsBox(MakeBox(1, 1, 1), W2W(0, 0, 0), MakeBox(1, 1, 1), W2W(5, 0, 0), axis, point, depth));
			}

			// Rotated box overlap (45 degrees around Z)
			{
				auto b_o2w = m4x4::Transform(v4(0, 0, 1, 0), float(math::constants<float>::tau_by_4 * 0.5), v4(1.8f, 0, 0, 1));
				float4 axis, point; float depth;
				auto gpu = physics::BoxVsBox(MakeBox(1, 1, 1), W2W(0, 0, 0), MakeBox(1, 1, 1), ToHlsl(b_o2w), axis, point, depth);

				collision::ShapeBox a(v4(1, 1, 1, 0)), b(v4(1, 1, 1, 0));
				collision::Contact c;
				auto cpu = collision::BoxVsBox(a, m4x4::Identity(), b, b_o2w, c);
				PR_EXPECT(gpu == cpu);
				if (gpu && cpu) PR_EXPECT(Near(depth, c.m_depth, 0.05f));
			}

			// Different sized boxes
			{
				float4 axis, point; float depth;
				auto gpu = physics::BoxVsBox(MakeBox(2, 0.5f, 1), W2W(0, 0, 0), MakeBox(0.5f, 2, 1), W2W(2, 0, 0), axis, point, depth);

				collision::ShapeBox a(v4(2, 0.5f, 1, 0)), b(v4(0.5f, 2, 1, 0));
				collision::Contact c;
				auto cpu = collision::BoxVsBox(a, m4x4::Identity(), b, m4x4::Transform(m4x4::Identity().rot, v4(2, 0, 0, 1)), c);
				PR_EXPECT(gpu == cpu);
				if (gpu && cpu) PR_EXPECT(Near(depth, c.m_depth, 0.01f));
			}
		}
	};
}

#endif