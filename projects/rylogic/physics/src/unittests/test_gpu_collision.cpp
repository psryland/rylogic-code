//************************************
// Physics Engine
//  Copyright (c) Rylogic Ltd 2016
//************************************
// Unit tests that compare GPU collision functions (compiled as C++) against CPU collision functions.

#if PR_UNITTESTS
#include "pr/common/unittests.h"
#include "pr/collision/ldraw.h"
#include "pr/collision/unittest_helpers.h"
#include "pr/physics/physics.h"
#include "src/compute/physics_types.h"
#include "src/compute/collision.hlsli"
#include "src/compute/gjk.hlsli"

namespace pr::physics::tests
{
	void ForceLink_GpuCollision() {}

	PRUnitTestClass(GpuCollisionTests)
	{
		static constexpr bool CreateVisuals = true;

		// Back-compat constructor for tests that only pin the contact centroid. Full manifold
		// validation is between the CPU contact and the GPU contact below.
		struct Expected :collision::Contact
		{
			Expected(float depth, v4 axis, v4 point)
			{
				m_axis = axis;
				SetPoint(point);
				m_depth = depth;
			}
		};

		// Tolerance check
		static bool Near(float a, float b, float tol = 1e-3f)
		{
			return FEqlAbsolute(a, b, tol);
		}

		// Check axis directions match (*Don't* allow sign flip)
		static bool AxisMatch(v4 expected, v4 actual, float tol = 1e-3f)
		{
			return FEqlRelative(expected, actual, tol) && expected.w == 0 && actual.w == 0;
		}

		// Check points match
		static bool PointMatch(v4 expected, v4 actual, float tol = 1e-3f)
		{
			return FEqlRelative(expected, actual, tol) && expected.w == 1 && actual.w == 1;
		}

		// Check 'contact' matches 'expected'.
		static void CheckExpected(collision::Contact const& contact, collision::Contact const* expected)
		{
			PR_EXPECT(contact.contact() == (expected != nullptr));
			if (expected == nullptr)
				return;

			PR_EXPECT(Near(expected->m_depth, contact.m_depth));
			PR_EXPECT(AxisMatch(expected->m_axis, contact.m_axis));
			if (expected->Count() > 1 && expected->Count() == contact.Count())
			{
				PR_EXPECT(PointMatch(expected->Point(), contact.Point()));
				PR_EXPECT(collision::tests::CheckContact(contact, *expected, 1e-3f));
			}
		}

		// Draw the scene
		void Visualise(collision::Shape const& a, m4x4 a2w, collision::Shape const& b, m4x4 b2w, collision::Contact const& c)
		{
			#if PR_UNITTESTS_VISUALISE
			if constexpr (CreateVisuals)
				collision::tests::VisualiseCollision(temp_dir() / L"LDraw/collision.ldr", a, a2w, b, b2w, c);
			#endif
			(void)a, a2w, b, b2w, c;
		}

		// Compare CPU and GPU results, and check expected depth (negative = no collision)
		void BangTogether(collision::Shape const& a, m4x4 a2w, collision::Shape const& b, m4x4 b2w, collision::Contact const* expected)
		{
			collision::Contact c0;
			collision::Collide(a, a2w, b, b2w, c0);
			Visualise(a, a2w, b, b2w, c0);
			CheckExpected(c0, expected);

			hlsl::StructuredBuffer<float4> verts;
			auto sa = PackShape(a, verts);
			auto sb = PackShape(b, verts);

			auto gpu_contact = GpuContact{};
			physics::CollideShapes(sa, a2w, sb, b2w, verts, gpu_contact);
			auto c1 = To<collision::Contact>(gpu_contact);
			Visualise(a, a2w, b, b2w, c1);
			CheckExpected(c1, expected);
		}

		// ---- Sphere vs Sphere ----
		PRUnitTestMethod(SphereVsSphere)
		{
			using namespace pr;
			using namespace pr::hlsl;

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
				constexpr auto dep = 3.0f - Sqrt(3.0f);
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

		// ---- Sphere vs Line ----
		PRUnitTestMethod(SphereVsLine)
		{
			using namespace pr;
			using namespace pr::hlsl;

			// Sphere near midpoint of Z-aligned line.
			// ShapeLine(2.0, 0.1) → half_len=1.0, thick_radius=0.1
			// depth = 0.5+0.1-0.4 = 0.2, axis points sphere→line = (-1,0,0); midpoint = (0,0,0)
			{
				auto exp = Expected{0.2f, v4(-1, 0, 0, 0), v4(0, 0, 0, 1)};
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

			// Both translated
			{
				auto exp = Expected{0.2f, v4(-1, 0, 0, 0), v4(2.0f, 3, 0, 1)};
				BangTogether(
					collision::ShapeSphere(0.5f), m4x4::Translation(2.4f, 3, 0),
					collision::ShapeLine(2.0f, 0.1f), m4x4::Translation(2, 3, 0),
					&exp);
			}

			// Sphere vs line end
			{
				auto exp = Expected{0.0406945869f, v4(-0.715183f,-0.583503f,0.384755f, 0), v4(0.0569633f,-0.179877f,0.184547f,1)};
				BangTogether(
					collision::ShapeSphere(0.5f), m4x4::Translation(0.4f, 0.1f, 0),
					collision::ShapeLine(2.0f, 0.1f), m4x4::TransformDeg(10, 0, 25, v4(0,-0.4f,1.2f,1)),
					&exp);
			}

		}

		// ---- Sphere vs Box ----
		PRUnitTestMethod(SphereVsBox)
		{
			using namespace pr;
			using namespace pr::hlsl;

			// Separated
			{
				BangTogether(
					collision::ShapeSphere(0.5f), m4x4::Translation(2.0f, 0, 0),
					collision::ShapeBox(v4(1, 1, 1, 0)), m4x4::Identity(),
					nullptr);
			}

			// Sphere to face
			{
				auto exp = Expected{0.2f, v4(-1, 0, 0, 0), v4(0.4f, 0, 0, 1)};
				BangTogether(
					collision::ShapeSphere(0.5f), m4x4::Translation(0.8f, 0, 0),
					collision::ShapeBox(v4(1, 1, 1, 0)), m4x4::Identity(),
					&exp);
			}

			// Sphere to edge
			{
				auto exp = Expected{0.292893261f, v4(-0.707107f,-0.707107f,0, 0), v4(0.396447f,0.396447f,0,1)};
				BangTogether(
					collision::ShapeSphere(1.0f), m4x4::Translation(1.0f, 1.0f, 0),
					collision::ShapeBox(v4(1, 1, 1, 0)), m4x4::Identity(),
					&exp);
			}

			// Sphere inside box
			{
				auto exp = Expected{0.9f, v4(-1, 0, 0, 0), v4(0.55f, 0, 0, 1)};
				BangTogether(
					collision::ShapeSphere(0.1f), m4x4::Translation(0.2f, 0, 0),
					collision::ShapeBox(v4(2, 2, 2, 0)), m4x4::Identity(),
					&exp);
			}

			// Sphere inside box, both transformed
			{
				auto exp = Expected{0.5f, v4(-1, 0, 0, 0), v4(1.45f, 2, 0, 1)};
				BangTogether(
					collision::ShapeSphere(0.3f), m4x4::Translation(1.5f, 2.0f, 0),
					collision::ShapeBox(v4(1, 1, 1, 0)), m4x4::Translation(1.2f, 2.0f, 0),
					&exp);
			}

			// Sphere vs box edge, box rotated
			{
				auto exp = Expected{0.207106814f,  v4(1, 0, 0, 0), v4(0.396447f,0,0,1)};
				BangTogether(
					collision::ShapeSphere(0.5f), m4x4::Identity(),
					collision::ShapeBox(v4(1, 1, 1, 0)), m4x4::TransformDeg(0, 0, 45.0f, v4(1, 0, 0, 1)),
					&exp);
			}

			// Sphere vs corner
			{
				auto exp = Expected{0.0742516518f, v4(-0.630542f,-0.17236f,0.756775f, 0), v4(1.14508f,0.402983f,-0.974031f, 1)};
				BangTogether(
					collision::ShapeSphere(0.6f), m4x4::Translation(1.5f, 0.5f, -1.4f),
					collision::ShapeBox(v4(2, 1, 0.5f, 0)), m4x4::TransformDeg(10, 20, 30, v4(0.2f, 0.3f, -0.4f, 1)),
					&exp);
			}

			// box rotated, sphere vs face
			{
				auto exp = Expected{0.227207914f, v4(-0.707107f,-0.707107f,0,0), v4(1.52678f,-0.273223f,0,1)};
				BangTogether(
					collision::ShapeSphere(0.5f), m4x4::Translation(1.8f, 0, 0),
					collision::ShapeBox(v4(2, 4, 2, 0)), m4x4::TransformDeg(0, 0, 45.0f, v4::Origin()),
					&exp);
			}
		}

		// ---- Line vs Line ----
		PRUnitTestMethod(LineVsLine)
		{
			using namespace pr;
			using namespace pr::hlsl;

			// Perpendicular lines, separatedby 0.3 (thickness 0.1+0.1 < 0.3)
			{
				BangTogether(
					collision::ShapeLine(2.0f, 0.1f), m4x4::Identity(),
					collision::ShapeLine(2.0f, 0.1f), m4x4::Transform(v4(0, 1, 0, 0), float(math::constants<float>::tau_by_4), v4(0, 0.3f, 0, 1)),
					nullptr);
			}

			// Perpendicular lines, colliding
			{
				auto exp = Expected{0.25f, v4(0, 1, 0, 0), v4(0,0.075f,0, 1)};
				BangTogether(
					collision::ShapeLine(2.0f, 0.2f), m4x4::Identity(),
					collision::ShapeLine(2.0f, 0.2f), m4x4::Transform(v4(0, 1, 0, 0), float(math::constants<float>::tau_by_4), v4(0, 0.15f, 0, 1)),
					&exp);
			}

			// Parallel lines, separated
			{
				BangTogether(
					collision::ShapeLine(2.0f, 0.1f), m4x4::Identity(),
					collision::ShapeLine(2.0f, 0.1f), m4x4::Translation(0.5f, 0, 0),
					nullptr);
			}

			// Parallel lines, not at the origin, overlapping: depth = 0.15+0.15-0.2 = 0.1
			// axis = (1,0,0), point = midpoint = (0.1, 0, 0)
			{
				auto exp = Expected{0.4f, v4(1, 0, 0, 0), v4(0.3f, 0.1f, 0.2f, 1)};
				BangTogether(
					collision::ShapeLine(2.0f, 0.3f), m4x4::Translation(0.2f, 0.1f, 0.2f),
					collision::ShapeLine(2.0f, 0.3f), m4x4::Translation(0.4f, 0.1f, 0.2f),
					&exp);
			}

			// Separated (far apart)
			{
				BangTogether(
					collision::ShapeLine(1.0f, 0.1f), m4x4::Identity(),
					collision::ShapeLine(1.0f, 0.1f), m4x4::Translation(5, 0, 0),
					nullptr);
			}

			// Arbitrary orientation, edge-to edge
			{
				auto exp = Expected{0.0498910546f, v4(0.597628f,0.720062f,0.352634f, 0), v4(0.610105f,-0.114319f,0.722602f, 1)};
				BangTogether(
					collision::ShapeLine(2.0f, 0.3f), m4x4::TransformDeg(40, 30, 20, v4(0.2f, 0.1f, 0.2f, 1)),
					collision::ShapeLine(2.0f, 0.3f), m4x4::TransformDeg(10,-20, 30, v4(1.0f, 0.2f, 0.2f, 1)),
					&exp);
			}

			// End-to-edge contact
			{
				auto exp = Expected{ 0.0792064667f, v4(-0.923698f,-0.259704f,0.281668f, 0), v4(0.132932f,-0.258726f,0.573786f, 1) };
				BangTogether(
					collision::ShapeLine(2.0f, 0.3f), m4x4::TransformDeg(40, 30, 20, v4(0.2f, 0.1f, 0.2f, 1)),
					collision::ShapeLine(2.0f, 0.3f), m4x4::TransformDeg(10, -30, 0, v4(-0.6f, -0.5f, 1.5f, 1)),
					&exp);
			}

			// End-to-end contact
			{
				auto exp = Expected{ 0.146075100f, v4(0.240969f, 0.47681f, 0.845332f, 0), v4(0.637713f,-0.43457f,1.05527f, 1) };
				BangTogether(
					collision::ShapeLine(2.0f, 0.3f), m4x4::TransformDeg(40, 30, 20, v4(0.2f, 0.1f, 0.2f, 1)),
					collision::ShapeLine(2.0f, 0.3f), m4x4::TransformDeg(10, -30, 30, v4(0.2f, -0.5f, 2.1f, 1)),
					&exp);
			}
		}

		// ---- Line vs Box ----
		PRUnitTestMethod(LineVsBox)
		{
			using namespace pr;
			using namespace pr::hlsl;

			// Separated
			{
				BangTogether(
					collision::ShapeLine(1.0f, 0.1f), m4x4::Translation(5, 0, 0),
					collision::ShapeBox(v4(1, 1, 1, 0)), m4x4::Identity(),
					nullptr);
			}

			// Line along Z approaching box face: depth = thick_r - gap = 0.1 - 0.05 = 0.05
			// axis=(1,0,0), line surface at 0.55-0.1=0.45, box face at 0.5, midpoint=(0.475,0,0)
			{
				auto exp = Expected{0.05f, v4(-1, 0, 0, 0), v4(0.475f, 0, 0, 1)};
				BangTogether(
					collision::ShapeLine(1.0f, 0.2f), m4x4::Translation(0.65f, 0, 0),
					collision::ShapeBox(v4(1, 1, 1, 0)), m4x4::Identity(),
					&exp);
			}

			// Line rotated about Y contacting box face
			{
				auto exp = Expected{0.05f, v4(-1, 0, 0, 0), v4(0.475f, 0, -0.433013f, 1)};
				BangTogether(
					collision::ShapeLine(1.0f, 0.2f), m4x4::Transform(m3x3::RotationDeg(0, 30, 0), v4(0.9f, 0, 0, 1)),
					collision::ShapeBox(v4(1, 1, 1, 0)), m4x4::Identity(),
					&exp);
			}

			// Line rotated contacting box edge
			{
				auto exp = Expected{0.0078418124f, v4(-0.960769f,-0.27735f,0, 0), v4(0.496231f, 0.498912f, 0.173353f, 1)};
				BangTogether(
					collision::ShapeLine(1.0f, 0.2f), m4x4::Transform(m3x3::RotationDeg(60, 30, 0), v4(0.7f, 0.5f, 0.2f, 1)),
					collision::ShapeBox(v4(1, 1, 1, 0)), m4x4::Identity(),
					&exp);
			}

			// Line rotated contacting box corner
			{
				auto exp = Expected{0.00974011f, v4(-0.629866f, -0.707071f, -0.321433f, 0), v4(0.4969f, 0.4966f, 0.4984f, 1)};
				BangTogether(
					collision::ShapeLine(1.0f, 0.2f), m4x4::Transform(m3x3::RotationDeg(40, 30, 0), v4(0.7f, 0.5f, 0.7f, 1)),
					collision::ShapeBox(v4(1, 1, 1, 0)), m4x4::Identity(),
					&exp);
			}

			// Line end contacting box corner
			{
				auto exp = Expected{0.0718057156f, v4(-0.846285f,0,-0.53273f, 0), v4(0.469616f, -0.478606f, 0.480873f, 1)};
				BangTogether(
					collision::ShapeLine(1.0f, 0.2f), m4x4::Transform(m3x3::RotationDeg(40, 30, 0), v4(0.8f,-0.8f,0.9f, 1)),
					collision::ShapeBox(v4(1, 1, 1, 0)), m4x4::Identity(),
					&exp);
			}

			// Slow convergence
			{
				auto exp = Expected{0.2f, v4(-0.999391f,-0.0348994f,0, 0), v4(0.400061f,0.49651f,0.2f, 1)};
				BangTogether(
					collision::ShapeLine(1.0f, 0.2f), m4x4::Transform(m3x3::RotationDeg(88, 90, 0), v4(0.5f,0.5f,0.2f, 1)),
					collision::ShapeBox(v4(1, 1, 1, 0)), m4x4::Identity(),
					&exp);
			}

			// Deep penetration
			{
				auto exp = Expected{0.2f, v4(-1, 0, 0, 0), v4(0.4f,0.250038f,0.204363f,1)};
				BangTogether(
					collision::ShapeLine(1.0f, 0.2f), m4x4::Transform(m3x3::RotationDeg(89, 0, 0), v4(0.5f,0.5f,0.2f, 1)),
					collision::ShapeBox(v4(1, 1, 1, 0)), m4x4::Identity(),
					&exp);
			}
		}

		// ---- Box vs Box ----
		PRUnitTestMethod(BoxVsBox)
		{
			using namespace pr;
			using namespace pr::hlsl;

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
				auto exp = Expected{0.366496146f, v4(1,0,0,0), v4(-0.316753f,0,0.288681f,1)};
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

			// Two tetrahedra overlapping at origin
			{
				v4 tet_pts[] = {
					v4{-1, -1, -1, 1}, v4{1, -1, -1, 1},
					v4{0, 1, -1, 1}, v4{0, 0, 1, 1},
				};
				auto buf_a = collision::BuildPolytopeFromPoints(tet_pts);
				auto buf_b = collision::BuildPolytopeFromPoints(tet_pts);
				
				// Slight overlap along X
				auto exp = Expected{1.11417198f, v4(0.742781f,-0.371391f,0.557086f,0), v4(0.25f,-0.25f,-0.5f,1)};
				BangTogether(
					buf_a.as<collision::ShapePolytope>(), m4x4::Identity(),
					buf_b.as<collision::ShapePolytope>(), m4x4::Translation(0.5f, 0, 0),
					&exp);
			}

			// Two tetrahedra separated
			{
				v4 tet_pts[] = {
					v4{-1, -1, -1, 1}, v4{1, -1, -1, 1},
					v4{0, 1, -1, 1}, v4{0, 0, 1, 1},
				};
				auto buf_a = collision::BuildPolytopeFromPoints(tet_pts);
				auto buf_b = collision::BuildPolytopeFromPoints(tet_pts);
				
				BangTogether(
					buf_a.as<collision::ShapePolytope>(), m4x4::Identity(),
					buf_b.as<collision::ShapePolytope>(), m4x4::Translation(10, 0, 0),
					nullptr);
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

				// Overlap of 0.5 in X: cubes are ±1, so separation = 2*1 - 0.5 = 1.5
				auto exp = Expected{0.5f, v4(1,0,0,0), v4(0.75f,0,0,1)};
				BangTogether(
					buf_a.as<collision::ShapePolytope>(), m4x4::Identity(),
					buf_b.as<collision::ShapePolytope>(), m4x4::Translation(1.5f, 0, 0),
					&exp);
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
				
				BangTogether(
					buf_a.as<collision::ShapePolytope>(), m4x4::Transform(m3x3::RotationDeg(0, 0, 20.0f), v4(-0.5f, 0, 0, 1)),
					buf_b.as<collision::ShapePolytope>(), m4x4::Transform(m3x3::RotationDeg(0, 0, -15.0f), v4(2.5f, 0, 0, 1)),
					nullptr);
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
				
				auto exp = Expected{0.545545f, v4(0.872872f,0.436436f,0.218218f,0), v4(0.166667f,-0.25f,-0.416667f,1)};
				BangTogether(
					buf_a.as<collision::ShapePolytope>(), m4x4::Identity(),
					buf_b.as<collision::ShapePolytope>(), m4x4::Translation(0.5f, 0, 0),
					&exp);
			}
		}
	};
}
#endif
