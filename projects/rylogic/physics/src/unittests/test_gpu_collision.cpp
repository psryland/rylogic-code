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
			hlsl::StructuredBuffer<float4> verts;
			hlsl::StructuredBuffer<GpuPolytopeFace> faces;
			hlsl::StructuredBuffer<GpuPolytopeEdge> edges;
			auto sa = PackShape(a, verts, &faces, &edges);
			auto sb = PackShape(b, verts, &faces, &edges);

			collision::Contact c0;
			collision::Collide(a, a2w, b, b2w, c0);
			Visualise(a, a2w, b, b2w, c0);

			auto gpu_contact = GpuContact{};
			physics::CollideShapes(sa, a2w, sb, b2w, verts, faces, edges, gpu_contact);
			auto c1 = To<collision::Contact>(gpu_contact);
			Visualise(a, a2w, b, b2w, c1);

			collision::tests::CheckContact(c0, expected);
			collision::tests::CheckContact(c1, expected);
		}

		// ---- Sphere vs Sphere ----
		PRUnitTestMethod(SphereVsSphere)
		{
			// Overlapping along X
			{
				auto exp = collision::Contact{
					.m_axis = v4{1, 0, 0, 0},
					.m_manifold = {
						v4{0.75f, 0, 0, 1},
					},
					.m_feature = EFeature::Vert,
					.m_depth = 0.5f,
				};
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

			// Diagonal overlap, different radii
			{
				auto exp = collision::Contact{
					.m_axis = v4{0.57735f,0.57735f,0.57735f,0},
					.m_manifold = {
						v4{0.788675f,0.788675f,0.788675f,1},
					},
					.m_feature = EFeature::Vert,
					.m_depth = 1.26794922f,
				};
				BangTogether(
					collision::ShapeSphere(2.0f), m4x4::Identity(),
					collision::ShapeSphere(1.0f), m4x4::Translation(1, 1, 1),
					&exp);
			}

			// Both translated, overlapping along Y
			{
				auto exp = collision::Contact{
					.m_axis = v4{0, 1, 0, 0},
					.m_manifold = {
						v4{1, 2.4f, 3, 1},
					},
					.m_feature = EFeature::Vert,
					.m_depth = 0.2f,
				};
				BangTogether(
					collision::ShapeSphere(0.5f), m4x4::Translation(1, 2, 3),
					collision::ShapeSphere(0.3f), m4x4::Translation(1, 2.6f, 3),
					&exp);
			}
		}

		// ---- Sphere vs Line ----
		PRUnitTestMethod(SphereVsLine)
		{
			// Sphere near midpoint of Z-aligned line.
			{
				auto exp = collision::Contact{
					.m_axis = v4{-1, 0, 0, 0},
					.m_manifold = {
						v4{0, 0, 0, 1},
					},
					.m_feature = EFeature::Vert,
					.m_depth = 0.2f,
				};
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
				auto exp = collision::Contact{
					.m_axis = v4{-1, 0, 0, 0},
					.m_manifold = {
						v4{2.0f, 3, 0, 1},
					},
					.m_feature = EFeature::Vert,
					.m_depth = 0.2f,
				};
				BangTogether(
					collision::ShapeSphere(0.5f), m4x4::Translation(2.4f, 3, 0),
					collision::ShapeLine(2.0f, 0.1f), m4x4::Translation(2, 3, 0),
					&exp);
			}

			// Sphere vs line end
			{
				auto exp = collision::Contact{
					.m_axis = v4{-0.715183f,-0.583503f,0.384755f, 0},
					.m_manifold = {
						v4{0.0569633f,-0.179877f,0.184547f,1},
					},
					.m_feature = EFeature::Vert,
					.m_depth = 0.0406945869f,
				};
				BangTogether(
					collision::ShapeSphere(0.5f), m4x4::Translation(0.4f, 0.1f, 0),
					collision::ShapeLine(2.0f, 0.1f), m4x4::TransformDeg(10, 0, 25, v4(0,-0.4f,1.2f,1)),
					&exp);
			}
		}

		// ---- Sphere vs Box ----
		PRUnitTestMethod(SphereVsBox)
		{
			// Separated
			{
				BangTogether(
					collision::ShapeSphere(0.5f), m4x4::Translation(2.0f, 0, 0),
					collision::ShapeBox(v4(1, 1, 1, 0)), m4x4::Identity(),
					nullptr);
			}

			// Sphere to face
			{
				auto exp = collision::Contact{
					.m_axis = v4{-1, 0, 0, 0},
					.m_manifold = {
						v4{0.4f, 0, 0, 1},
					},
					.m_feature = EFeature::Vert,
					.m_depth = 0.2f,
				};
				BangTogether(
					collision::ShapeSphere(0.5f), m4x4::Translation(0.8f, 0, 0),
					collision::ShapeBox(v4(1, 1, 1, 0)), m4x4::Identity(),
					&exp);
			}

			// Sphere to edge
			{
				auto exp = collision::Contact{
					.m_axis = v4{-0.707107f,-0.707107f,0, 0},
					.m_manifold = {
						v4{0.396447f,0.396447f,0,1},
					},
					.m_feature = EFeature::Vert,
					.m_depth = 0.292893261f,
				};
				BangTogether(
					collision::ShapeSphere(1.0f), m4x4::Translation(1.0f, 1.0f, 0),
					collision::ShapeBox(v4(1, 1, 1, 0)), m4x4::Identity(),
					&exp);
			}

			// Sphere inside box
			{
				auto exp = collision::Contact{
					.m_axis = v4{-1, 0, 0, 0},
					.m_manifold = {
						v4{0.55f, 0, 0, 1},
					},
					.m_feature = EFeature::Vert,
					.m_depth = 0.9f,
				};
				BangTogether(
					collision::ShapeSphere(0.1f), m4x4::Translation(0.2f, 0, 0),
					collision::ShapeBox(v4(2, 2, 2, 0)), m4x4::Identity(),
					&exp);
			}

			// Sphere inside box, both transformed
			{
				auto exp = collision::Contact{
					.m_axis = v4{-1, 0, 0, 0},
					.m_manifold = {
						v4{1.45f, 2, 0, 1},
					},
					.m_feature = EFeature::Vert,
					.m_depth = 0.5f,
				};
				BangTogether(
					collision::ShapeSphere(0.3f), m4x4::Translation(1.5f, 2.0f, 0),
					collision::ShapeBox(v4(1, 1, 1, 0)), m4x4::Translation(1.2f, 2.0f, 0),
					&exp);
			}

			// Sphere vs box edge, box rotated
			{
				auto exp = collision::Contact{
					.m_axis = v4{1, 0, 0, 0},
					.m_manifold = {
						v4{0.396447f,0,0,1},
					},
					.m_feature = EFeature::Vert,
					.m_depth = 0.207106814f,
				};
				BangTogether(
					collision::ShapeSphere(0.5f), m4x4::Identity(),
					collision::ShapeBox(v4(1, 1, 1, 0)), m4x4::TransformDeg(0, 0, 45.0f, v4(1, 0, 0, 1)),
					&exp);
			}

			// Sphere vs corner
			{
				auto exp = collision::Contact{
					.m_axis = v4{-0.630542f,-0.17236f,0.756775f, 0},
					.m_manifold = {
						v4{1.14508f,0.402983f,-0.974031f, 1},
					},
					.m_feature = EFeature::Vert,
					.m_depth = 0.0742516518f,
				};
				BangTogether(
					collision::ShapeSphere(0.6f), m4x4::Translation(1.5f, 0.5f, -1.4f),
					collision::ShapeBox(v4(2, 1, 0.5f, 0)), m4x4::TransformDeg(10, 20, 30, v4(0.2f, 0.3f, -0.4f, 1)),
					&exp);
			}

			// box rotated, sphere vs face
			{
				auto exp = collision::Contact{
					.m_axis = v4{-0.707107f,-0.707107f,0,0},
					.m_manifold = {
						v4{1.52678f,-0.273223f,0,1},
					},
					.m_feature = EFeature::Vert,
					.m_depth = 0.227207914f,
				};
				BangTogether(
					collision::ShapeSphere(0.5f), m4x4::Translation(1.8f, 0, 0),
					collision::ShapeBox(v4(2, 4, 2, 0)), m4x4::TransformDeg(0, 0, 45.0f, v4::Origin()),
					&exp);
			}
		}

		// ---- Line vs Line ----
		PRUnitTestMethod(LineVsLine)
		{
			// Perpendicular lines, separatedby 0.3 (thickness 0.1+0.1 < 0.3)
			{
				BangTogether(
					collision::ShapeLine(2.0f, 0.1f), m4x4::Identity(),
					collision::ShapeLine(2.0f, 0.1f), m4x4::Transform(v4(0, 1, 0, 0), float(math::constants<float>::tau_by_4), v4(0, 0.3f, 0, 1)),
					nullptr);
			}

			// Perpendicular lines, colliding
			{
				auto exp = collision::Contact{
					.m_axis = v4{0, 1, 0, 0},
					.m_manifold = {
						v4{0,0.075f,0, 1},
					},
					.m_feature = EFeature::Vert,
					.m_depth = 0.25f,
				};
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

			// Parallel lines, not at the origin, overlapping
			{
				auto exp = collision::Contact{
					.m_axis = v4{1, 0, 0, 0},
					.m_manifold = {
						v4(0.3f, 0.1f, -0.8f, 1),
						v4(0.3f, 0.1f, +1.2f, 1),

					},
					.m_feature = EFeature::Edge,
					.m_depth = 0.4f,
				};
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
				auto exp = collision::Contact{
					.m_axis = v4{0.597628f,0.720062f,0.352634f, 0},
					.m_manifold = {
						v4{0.610105f,-0.114319f,0.722602f, 1},
					},
					.m_feature = EFeature::Vert,
					.m_depth = 0.0498910546f,
				};
				BangTogether(
					collision::ShapeLine(2.0f, 0.3f), m4x4::TransformDeg(40, 30, 20, v4(0.2f, 0.1f, 0.2f, 1)),
					collision::ShapeLine(2.0f, 0.3f), m4x4::TransformDeg(10,-20, 30, v4(1.0f, 0.2f, 0.2f, 1)),
					&exp);
			}

			// End-to-edge contact
			{
				auto exp = collision::Contact{
					.m_axis = v4{-0.923698f,-0.259704f,0.281668f, 0},
					.m_manifold = {
						v4{0.132932f,-0.258726f,0.573786f, 1},
					},
					.m_feature = EFeature::Vert,
					.m_depth = 0.0792064667f,
				};
				BangTogether(
					collision::ShapeLine(2.0f, 0.3f), m4x4::TransformDeg(40, 30, 20, v4(0.2f, 0.1f, 0.2f, 1)),
					collision::ShapeLine(2.0f, 0.3f), m4x4::TransformDeg(10, -30, 0, v4(-0.6f, -0.5f, 1.5f, 1)),
					&exp);
			}

			// End-to-end contact
			{
				auto exp = collision::Contact{
					.m_axis = v4{0.240969f, 0.47681f, 0.845332f, 0},
					.m_manifold = {
						v4{0.637713f,-0.43457f,1.05527f, 1},
					},
					.m_feature = EFeature::Vert,
					.m_depth = 0.146075100f,
				};
				BangTogether(
					collision::ShapeLine(2.0f, 0.3f), m4x4::TransformDeg(40, 30, 20, v4(0.2f, 0.1f, 0.2f, 1)),
					collision::ShapeLine(2.0f, 0.3f), m4x4::TransformDeg(10, -30, 30, v4(0.2f, -0.5f, 2.1f, 1)),
					&exp);
			}
		}

		// ---- Line vs Box ----
		PRUnitTestMethod(LineVsBox)
		{
			// Separated
			{
				BangTogether(
					collision::ShapeLine(1.0f, 0.1f), m4x4::Translation(5, 0, 0),
					collision::ShapeBox(v4(1, 1, 1, 0)), m4x4::Identity(),
					nullptr);
			}

			// Line along Z approaching box face
			{
				auto exp = collision::Contact{
					.m_axis = v4{-1, 0, 0, 0},
					.m_manifold = {
						v4(0.475f, 0, -0.5f, 1),
						v4(0.475f, 0, +0.5f, 1),
					},
					.m_feature = EFeature::Edge,
					.m_depth = 0.05f,
				};
				BangTogether(
					collision::ShapeLine(1.0f, 0.2f), m4x4::Translation(0.65f, 0, 0),
					collision::ShapeBox(v4(1, 1, 1, 0)), m4x4::Identity(),
					&exp);
			}

			// Line rotated about Y contacting box face
			{
				auto exp = collision::Contact{
					.m_axis = v4{-1, 0, 0, 0},
					.m_manifold = {
						v4{0.475f, 0, -0.433013f, 1},
					},
					.m_feature = EFeature::Vert,
					.m_depth = 0.05f,
				};
				BangTogether(
					collision::ShapeLine(1.0f, 0.2f), m4x4::Transform(m3x3::RotationDeg(0, 30, 0), v4(0.9f, 0, 0, 1)),
					collision::ShapeBox(v4(1, 1, 1, 0)), m4x4::Identity(),
					&exp);
			}

			// Line rotated contacting box edge
			{
				auto exp = collision::Contact{
					.m_axis = v4{-0.960769f,-0.27735f,0, 0},
					.m_manifold = {
						v4{0.496231f, 0.498912f, 0.173353f, 1},
					},
					.m_feature = EFeature::Vert,
					.m_depth = 0.0078418124f,
				};
				BangTogether(
					collision::ShapeLine(1.0f, 0.2f), m4x4::Transform(m3x3::RotationDeg(60, 30, 0), v4(0.7f, 0.5f, 0.2f, 1)),
					collision::ShapeBox(v4(1, 1, 1, 0)), m4x4::Identity(),
					&exp);
			}

			// Line rotated contacting box corner
			{
				auto exp = collision::Contact{
					.m_axis = v4{-0.629866f, -0.707071f, -0.321433f, 0},
					.m_manifold = {
						v4{0.4969f, 0.4966f, 0.4984f, 1},
					},
					.m_feature = EFeature::Vert,
					.m_depth = 0.00974011f,
				};
				BangTogether(
					collision::ShapeLine(1.0f, 0.2f), m4x4::Transform(m3x3::RotationDeg(40, 30, 0), v4(0.7f, 0.5f, 0.7f, 1)),
					collision::ShapeBox(v4(1, 1, 1, 0)), m4x4::Identity(),
					&exp);
			}

			// Line end contacting box corner
			{
				auto exp = collision::Contact{
					.m_axis = v4{-0.846285f,0,-0.53273f, 0},
					.m_manifold = {
						v4{0.469616f, -0.478606f, 0.480873f, 1},
					},
					.m_feature = EFeature::Vert,
					.m_depth = 0.0718057156f,
				};
				BangTogether(
					collision::ShapeLine(1.0f, 0.2f), m4x4::Transform(m3x3::RotationDeg(40, 30, 0), v4(0.8f,-0.8f,0.9f, 1)),
					collision::ShapeBox(v4(1, 1, 1, 0)), m4x4::Identity(),
					&exp);
			}

			// Slow convergence
			{
				auto exp = collision::Contact{
					.m_axis = v4{-0.999391f,-0.0348994f,0, 0},
					.m_manifold = {
						v4{0.400061f,0.49651f,0.2f, 1},
					},
					.m_feature = EFeature::Vert,
					.m_depth = 0.2f,
				};
				BangTogether(
					collision::ShapeLine(1.0f, 0.2f), m4x4::Transform(m3x3::RotationDeg(88, 90, 0), v4(0.5f,0.5f,0.2f, 1)),
					collision::ShapeBox(v4(1, 1, 1, 0)), m4x4::Identity(),
					&exp);
			}

			// Deep penetration
			{
				auto exp = collision::Contact{
					.m_axis = v4{-1, 0, 0, 0},
					.m_manifold = {
						v4(0.4f, 0.5f, 0.200000f, 1),
						v4(0.4f, 0.0f, 0.208726f, 1),
					},
					.m_feature = EFeature::Edge,
					.m_depth = 0.2f,
				};
				BangTogether(
					collision::ShapeLine(1.0f, 0.2f), m4x4::Transform(m3x3::RotationDeg(89, 0, 0), v4(0.5f,0.5f,0.2f, 1)),
					collision::ShapeBox(v4(1, 1, 1, 0)), m4x4::Identity(),
					&exp);
			}
		}

		// ---- Box vs Box ----
		PRUnitTestMethod(BoxVsBox)
		{
			// Axis-aligned overlap along X
			{
				auto exp = collision::Contact{
					.m_axis = v4{1, 0, 0, 0},
					.m_manifold = {
						v4(0.494992f, -0.5f, -0.5f, 1),
						v4(0.494992f, +0.5f, -0.5f, 1),
						v4(0.494992f, +0.5f, +0.5f, 1),
						v4(0.494992f, -0.5f, +0.5f, 1),
					},
					.m_feature = EFeature::Quad,
					.m_depth = 0.0100150108f,
				};
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
				auto exp = collision::Contact{
					.m_axis = v4{1, 0, 0, 0},
					.m_manifold = {
						v4(0.471454f, 0, +0.5f, 1),
						v4(0.471454f, 0, -0.5f, 1),
					},
					.m_feature = EFeature::Edge,
					.m_depth = 0.0571218729f,
				};
				BangTogether(
					collision::ShapeBox(v4(1, 1, 1, 0)), m4x4::Identity(),
					collision::ShapeBox(v4(1, 1, 1, 0)), m4x4::Transform(m3x3::RotationDeg(0, 0, 45.0f), v4(1.15f, 0, 0, 1)),
					&exp);
			}

			// Different sized boxes
			{
				auto exp = collision::Contact{
					.m_axis = v4{1, 0, 0, 0},
					.m_manifold = {
						v4(0.994991f, -0.25f, -0.5f, 1),
						v4(0.994991f, +0.25f, -0.5f, 1),
						v4(0.994991f, +0.25f, +0.5f, 1),
						v4(0.994991f, -0.25f, +0.5f, 1),
					},
					.m_feature = EFeature::Quad,
					.m_depth = 0.0100175142f,
				};
				BangTogether(
					collision::ShapeBox(v4(2, 0.5f, 1, 0)), m4x4::Identity(),
					collision::ShapeBox(v4(0.5f, 2, 1, 0)), m4x4::Translation(1.24f, 0, 0),
					&exp);
			}

			// Corner contact
			{
				auto exp = collision::Contact{
					.m_axis = v4{1, 0, 0, 0},
					.m_manifold = {
						v4{0.985723f, 0.0303301f, 0.271447f, 1},
					},
					.m_feature = EFeature::Vert,
					.m_depth = 0.0285550356f,
				};
				BangTogether(
					collision::ShapeBox(v4(2, 0.5f, 1, 0)), m4x4::Identity(),
					collision::ShapeBox(v4(0.5f, 2, 1, 0)), m4x4::Transform(m3x3::RotationDeg(0, 45.0f, 45.0f), v4(1.95f, -0.5f, 0, 1)),
					&exp);
			}

			// Edge contact
			{
				auto exp = collision::Contact{
					.m_axis = v4{0.816497f, 0.57735f, 0, 0},
					.m_manifold = {
						v4{0.98989f, 0.242851f, 0.0631133f, 1},
					},
					.m_feature = EFeature::Vert,
					.m_depth = 0.0247642994f,
				};
				BangTogether(
					collision::ShapeBox(v4(2, 0.5f, 1, 0)), m4x4::Identity(),
					collision::ShapeBox(v4(0.5f, 2, 1, 0)), m4x4::Transform(m3x3::RotationDeg(0, 45.0f, 45.0f), v4(1.75f, 0, 0, 1)),
					&exp);
			}

			// Both rotated, face-edge overlap
			{
				auto exp = collision::Contact{
					.m_axis = v4{0.965926f,-0.258819f,0,0},
					.m_manifold = {
						v4(0.207905f, -0.163212f, +0.5f, 1),
						v4(0.207905f, -0.163212f, -0.5f, 1),
					},
					.m_feature = EFeature::Edge,
					.m_depth = 0.275282145f,
				};
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
				auto exp = collision::Contact{
					.m_axis = v4{1,0,0,0},
					.m_manifold = {
						v4(0.471454f, 0, +0.5f, 1),
						v4(0.471454f, 0, -0.5f, 1),
					},
					.m_feature = EFeature::Edge,
					.m_depth = 0.0571218729f,
				};
				BangTogether(
					collision::ShapeBox(v4(1, 1, 1, 0)), m4x4::Transform(m3x3::RotationDeg(30.0f, 0, 0), v4(0, 0, 0, 1)),
					collision::ShapeBox(v4(1, 1, 1, 0)), m4x4::Transform(m3x3::RotationDeg(0, 0, 45.0f), v4(1.15f, 0, 0, 1)),
					&exp);
			}

			// Both rotated around all three axes
			{
				auto exp = collision::Contact{
					.m_axis = v4{0.916575f,-0.164476f,-0.364469f,0},
					.m_manifold = {
						v4{0.285301f,-0.122901f,-0.0770606f,1},
					},
					.m_feature = EFeature::Vert,
					.m_depth = 0.0951071978f,
				};
				BangTogether(
					collision::ShapeBox(v4(1, 1, 1, 0)), m4x4::Transform(m3x3::RotationDeg(10.0f, 20.0f, 30.0f), v4(-0.4f, 0.1f, -0.1f, 1)),
					collision::ShapeBox(v4(1, 1, 1, 0)), m4x4::Transform(m3x3::RotationDeg(-15.0f, 25.0f, -10.0f), v4(0.9f, -0.1f, 0.2f, 1)),
					&exp);
			}

			// Two elongated boxes crossing like an X, edges closest
			{
				auto exp = collision::Contact{
					.m_axis = v4{1,0,0,0},
					.m_manifold = {
						v4(0.075002f, +0.1f, -0.1f, 1),
						v4(0.075002f, +0.1f, +0.1f, 1),
						v4(0.075002f, -0.1f, +0.1f, 1),
						v4(0.075002f, -0.1f, -0.1f, 1),
					},
					.m_feature = EFeature::Quad,
					.m_depth = 0.05f,
				};
				BangTogether(
					collision::ShapeBox(v4(0.2f, 0.2f, 3, 0)), m4x4::Identity(),
					collision::ShapeBox(v4(0.2f, 0.2f, 3, 0)), m4x4::Transform(m3x3::RotationDeg(90.0f, 0, 0), v4(0.15f, 0, 0, 1)),
					&exp);
			}

			// Two unit boxes positioned so an edge-edge cross-product axis is the minimum
			{
				auto exp = collision::Contact{
					.m_axis = v4{0,1,0,0},
					.m_manifold = {
						v4{0,0.475f,0,1},
					},
					.m_feature = EFeature::Vert,
					.m_depth = 0.464213550f,
				};
				BangTogether(
					collision::ShapeBox(v4(1, 1, 1, 0)), m4x4::Transform(m3x3::RotationDeg(0, 0, 45.0f), v4(0, 0, 0, 1)),
					collision::ShapeBox(v4(1, 1, 1, 0)), m4x4::Transform(m3x3::RotationDeg(45.0f, 0, 0), v4(0, 0.95f, 0, 1)),
					&exp);
			}

			// B's corner pokes into A's face (B rotated around two axes)
			{
				auto exp = collision::Contact{
					.m_axis = v4{1,0,0,0},
					.m_manifold = {
						v4{0.316753f,0,-0.288681f,1},
					},
					.m_feature = EFeature::Vert,
					.m_depth = 0.366496146f,
				};
				BangTogether(
					collision::ShapeBox(v4(1, 1, 1, 0)), m4x4::Identity(),
					collision::ShapeBox(v4(1, 1, 1, 0)), m4x4::Transform(m3x3::RotationDeg(45.0f, 35.264f, 0), v4(0.95f, 0, 0, 1)),
					&exp);
			}

			// A's corner pokes into B's face (A rotated, B at identity)
			{
				auto exp = collision::Contact{
					.m_axis = v4{1,0,0,0},
					.m_manifold = {
						v4{-0.316753f,0,0.288681f,1},
					},
					.m_feature = EFeature::Vert,
					.m_depth = 0.366496146f,
				};
				BangTogether(
					collision::ShapeBox(v4(1, 1, 1, 0)), m4x4::Transform(m3x3::RotationDeg(45.0f, 35.264f, 0), v4(-0.95f, 0, 0, 1)),
					collision::ShapeBox(v4(1, 1, 1, 0)), m4x4::Identity(),
					&exp);
			}

			// Nearly touching (very small overlap)
			{
				auto exp = collision::Contact{
					.m_axis = v4{1,0,0,0},
					.m_manifold = {
						v4(0.499492f, -0.5f, -0.5f, 1),
						v4(0.499492f, +0.5f, -0.5f, 1),
						v4(0.499492f, +0.5f, +0.5f, 1),
						v4(0.499492f, -0.5f, +0.5f, 1),
					},
					.m_feature = EFeature::Quad,
					.m_depth = 0.00101500750f,
				};
				BangTogether(
					collision::ShapeBox(v4(1, 1, 1, 0)), m4x4::Identity(),
					collision::ShapeBox(v4(1, 1, 1, 0)), m4x4::Translation(0.999f, 0, 0),
					&exp);
			}

			// Deep overlap (one box mostly inside the other)
			{
				auto exp = collision::Contact{
					.m_axis = v4{1,0,0,0},
					.m_manifold = {
						v4(0.475002f, -0.1f, -0.17f, 1),
						v4(0.475002f, +0.2f, -0.17f, 1),
						v4(0.475002f, +0.2f, +0.13f, 1),
						v4(0.475002f, -0.1f, +0.13f, 1),
					},
					.m_feature = EFeature::Quad,
					.m_depth = 1.05f,
				};
				BangTogether(
					collision::ShapeBox(v4(2, 2, 2, 0)), m4x4::Identity(),
					collision::ShapeBox(v4(0.3f, 0.3f, 0.3f, 0)), m4x4::Translation(0.1f, 0.05f, -0.02f),
					&exp);
			}

			// Boxes touching at an edge, both translated off-origin
			{
				auto exp = collision::Contact{
					.m_axis = v4{0.866025f,0.5f,0,0},
					.m_manifold = {
						v4(3.68181f, -0.18245f, 1.5f, 1),
						v4(3.68181f, -0.18245f, 0.5f, 1),
					},
					.m_feature = EFeature::Edge,
					.m_depth = 0.00154542923f,
				};
				BangTogether(
					collision::ShapeBox(v4(1, 1, 1, 0)), m4x4::Transform(m3x3::RotationDeg(0, 0, 30.0f), v4(3, 0, 1, 1)),
					collision::ShapeBox(v4(1, 1, 1, 0)), m4x4::Transform(m3x3::RotationDeg(0, 0, -20.0f), v4(4.322f, 0.116f, 1, 1)),
					&exp);
			}

			// ---- Asymmetric shapes, both transformed ----
			{
				auto exp = collision::Contact{
					.m_axis = v4{0.858356f,0.132788f,-0.495572f,0},
					.m_manifold = {
						v4{0.914174f,-0.186723f,0.899549f,1},
					},
					.m_feature = EFeature::Vert,
					.m_depth = 0.199977040f,
				};
				BangTogether(
					collision::ShapeBox(v4(2, 2, 0.2f, 0)), m4x4::Transform(m3x3::RotationDeg(15.0f, 0, 0), v4(0, 0, 1, 1)),
					collision::ShapeBox(v4(0.4f, 0.4f, 3, 0)), m4x4::Transform(m3x3::RotationDeg(0, 30.0f, 0), v4(0.8f, 0, 0.5f, 1)),
					&exp);
			}
		}
		
		// ---- Polytope vs Sphere ----
		PRUnitTestMethod(PolytopeVsSphere)
		{
			constexpr v4 tet_pts[] = {
				v4(-0.8f, -0.8f, -0.5f, 1),
				v4(+0.8f, -0.8f, -0.5f, 1),
				v4(+0.0f, +0.8f, -0.5f, 1),
				v4(+0.0f, +0.0f, +0.8f, 1),
			};
			auto buf = collision::BuildPolytopeFromPoints(tet_pts);
			auto& polytope = buf.as<collision::ShapePolytope>();

			// Sphere vs face
			{
				auto exp = collision::Contact{
					.m_axis = v4{0.862366f,0.431183f,0.265343f,0},
					.m_manifold = {
						v4{0.213439f,-0.0932803f,-0.0574033f,1},
					},
					.m_feature = EFeature::Vert,
					.m_depth = 0.167328164f,
				};
				BangTogether(
					polytope, m4x4::Identity(),
					collision::ShapeSphere{ 0.3f }, m4x4::Translation(0.4f, 0, 0),
					&exp);
			}

			// Sphere vs edge
			{
				auto exp = collision::Contact{
					.m_axis = v4{-0.380810f, 0.775708f, 0.503250f, 0},
					.m_manifold = {
						v4{0.005338f, 0.385428f, -0.139206f, 1},
					},
					.m_feature = EFeature::Vert,
					.m_depth = 0.046770f,
				};
				BangTogether(
					polytope, m4x4::TransformDeg(0, 0, 45, v4(0.4f, 0, 0, 1)),
					collision::ShapeSphere{ 0.3f }, m4x4::Translation(-0.1f, 0.6f, 0),
					&exp);
			}

			// Sphere vs corner
			{
				auto exp = collision::Contact{
					.m_axis = v4{0.565852f, 0.0f, -0.824507f, 0},
					.m_manifold = {
						v4{1.480808f, 0.0f, -0.426324f, 1},
					},
					.m_feature = EFeature::Vert,
					.m_depth = 0.178715f,
				};
				BangTogether(
					polytope, m4x4::TransformDeg(0, 0, 45, v4(0.4f, 0, 0, 1)),
					collision::ShapeSphere{ 0.3f }, m4x4::Translation(1.6f, 0, -0.6f),
					&exp);
			}
		}

		// ---- Triangle vs Box ----
		PRUnitTestMethod(TriangleVsBox)
		{
			// Triangle face-on to a box face
			{
				auto exp = collision::Contact{
					.m_axis = v4(0, 0, 1, 0),
					.m_manifold = {
						v4(0, 0.5f, -0.25f, 1),
						v4(-0.5f, -0.5f, -0.25f, 1),
						v4(+0.5f, -0.5f, -0.25f, 1),
					},
					.m_feature = EFeature::Tri,
					.m_depth = 0.5f,
				};
				BangTogether(
					collision::ShapeTriangle{v4{-0.5f, -0.5f, 0, 1}, v4{+0.5f, -0.5f, 0, 1}, v4{0, +0.5f, 0, 1}}, m4x4::Identity(),
					collision::ShapeBox{v4{1, 1, 1, 0}}, m4x4::Identity(),
					&exp);
			}

			// Separated
			{
				BangTogether(
					collision::ShapeTriangle{v4{-1, -1, 0, 1}, v4{+1, -1, 0, 1}, v4{0, +1, 0, 1}}, m4x4::Identity(),
					collision::ShapeBox{v4{0.5f, 0.5f, 0.5f, 0}}, m4x4::Translation(5, 0, 0),
					nullptr);
			}

			// Rotated triangle intersecting rotated box: exercises edge-cross axes
			{
				auto exp = collision::Contact{
					.m_axis = v4(0, 0.707106829f, -0.707106769f, 0),
					.m_manifold = {
						v4(0.300000012f, 0.0508883521f, 0.252665043f, 1),
					},
					.m_feature = EFeature::Vert,
					.m_depth = 0.285355419f,
				};
				BangTogether(
					collision::ShapeTriangle{v4{-1, 0, 0, 1}, v4{+1, 0, 0, 1}, v4{0, +1, 0, 1}}, m4x4::Transform(RotationRad<m3x3>(constants<float>::tau_by_8, 0, 0), v4{0.1f, 0, 0, 1}),
					collision::ShapeBox{v4{0.5f, 0.5f, 0.5f, 0}}, m4x4::Transform(RotationRad<m3x3>(0, constants<float>::tau_by_8, 0), v4{0.3f, 0.2f, 0, 1}),
					&exp);
			}
		}

		// ---- Polytope vs Box ----
		PRUnitTestMethod(PolytopeVsBox)
		{
			constexpr v4 tet_pts[] = {
				v4(-0.8f, -0.8f, -0.5f, 1),
				v4(+0.8f, -0.8f, -0.5f, 1),
				v4(+0.0f, +0.8f, -0.5f, 1),
				v4(+0.0f, +0.0f, +0.8f, 1),
			};
			auto buf = collision::BuildPolytopeFromPoints(tet_pts);
			auto& polytope = buf.as<collision::ShapePolytope>();

			// polytope vs ground box
			{
				auto exp = collision::Contact{
					.m_axis = v4{0, 0, -1, 0},
					.m_manifold = {
						v4{-0.369652f,-0.353485f,-0.0963602f,1},
					},
					.m_feature = EFeature::Vert,
					.m_depth = 0.192720473f,
				};
				BangTogether(
					polytope, m4x4::TransformDeg(-80, 25, 0, v4(0, 0, 0.6f, 1)),
					collision::ShapeBox(v4(100, 100, 1.0, 0)), m4x4::Translation(0, 0, -0.5f),
					&exp);
			}

			// Runtime narrow-phase works in body-A space. Near-axis box support queries should still return a face feature, otherwise the contact point
			// can snap to a far corner of the large ground box and give the resolver a bogus lever arm.
			{
				auto ground = collision::ShapeBox(v4(100, 100, 1.0, 0));
				auto a2w = m4x4::TransformDeg(-80, 25, 0, v4(0, 0, 0.6f, 1));
				auto b2w = m4x4::Translation(0, 0, -0.5f);
				auto b2a = InvertOrthonormal(a2w) * b2w;

				collision::Contact cpu_contact;
				PR_EXPECT(collision::Collide(polytope, a2w, ground, b2w, cpu_contact));

				hlsl::StructuredBuffer<float4> verts;
				hlsl::StructuredBuffer<GpuPolytopeFace> faces;
				hlsl::StructuredBuffer<GpuPolytopeEdge> edges;
				auto sa = PackShape(polytope, verts, &faces, &edges);
				auto sb = PackShape(ground, verts, &faces, &edges);

				GpuContact gpu_contact{};
				PR_EXPECT(physics::CollideShapes(sa, m4x4::Identity(), sb, b2a, verts, faces, edges, gpu_contact));

				auto gpu_axis_ws = a2w * gpu_contact.axis;
				auto gpu_point_ws = a2w * ContactCentroid(gpu_contact);
				PR_EXPECT(FEqlRelative(gpu_axis_ws, cpu_contact.m_axis, 1e-4f));
				PR_EXPECT(FEqlRelative(gpu_point_ws, cpu_contact.Point(), 1e-4f));
				PR_EXPECT(FEqlAbsolute(gpu_contact.depth, cpu_contact.m_depth, 1e-4f));
			}
		}

		PRUnitTestMethod(PolytopeVsGround_ShallowFastTumble)
		{
			v4 poly_pts[] = {
				v4{-1.0296705f,  0.2203998f, -0.1274743f, 1},
				v4{ 0.2744289f, -0.4075151f, -0.0194631f, 1},
				v4{ 0.2335063f, -0.0944359f, -0.4734315f, 1},
				v4{-0.1430905f,  0.3362911f,  0.0826810f, 1},
				v4{ 0.0412595f, -0.4096290f, -0.3432735f, 1},
				v4{ 0.3788538f,  0.3895851f,  0.5945120f, 1},
				v4{-0.1899699f, -0.1738422f, -0.2568449f, 1},
				v4{ 0.5233417f,  0.1291325f, -0.6528223f, 1},
				v4{-0.2173630f, -0.1193020f,  0.2580981f, 1},
			};
			auto poly_buf = collision::BuildPolytopeFromPoints(poly_pts);
			auto const& poly = poly_buf.as<collision::ShapePolytope>();

			auto ground = collision::ShapeBox{v4{790.38454f, 790.38454f, 10.0f, 0}};
			auto ground_l2w = m4x4::Translation(0, 0, -5);
			auto poly_o2w = m4x4{
				v4{-0.48897f, -0.03973f, -0.87140f, 0},
				v4{ 0.39218f,  0.88229f, -0.26030f, 0},
				v4{ 0.77917f, -0.46902f, -0.41583f, 0},
				v4{18.36216f,-60.80049f,  0.39107f, 1},
			};
			auto b2a = InvertOrthonormal(poly_o2w) * ground_l2w;

			collision::Contact cpu_contact;
			PR_EXPECT(collision::Collide(poly, poly_o2w, ground, ground_l2w, cpu_contact));

			hlsl::StructuredBuffer<float4> verts;
			hlsl::StructuredBuffer<GpuPolytopeFace> faces;
			hlsl::StructuredBuffer<GpuPolytopeEdge> edges;
			auto sa = PackShape(poly, verts, &faces, &edges);
			auto sb = PackShape(ground, verts, &faces, &edges);

			GpuContact gpu_contact{};
			auto hit = physics::CollideShapes(sa, m4x4::Identity(), sb, b2a, verts, faces, edges, gpu_contact);
			PR_EXPECT(hit);
			PR_EXPECT(gpu_contact.depth > 0.0f);
		}

		// ---- Polytope vs Line ----
		PRUnitTestMethod(PolytopeVsLine)
		{
			constexpr v4 tet_pts[] = {
				v4(-0.8f, -0.8f, -0.5f, 1),
				v4(+0.8f, -0.8f, -0.5f, 1),
				v4(+0.0f, +0.8f, -0.5f, 1),
				v4(+0.0f, +0.0f, +0.8f, 1),
			};
			auto buf = collision::BuildPolytopeFromPoints(tet_pts);
			auto& polytope = buf.as<collision::ShapePolytope>();

			// Line-edge vs face
			{
				auto exp = collision::Contact{
					.m_axis = v4{0.524098f,-0.851658f, 0, 0},
					.m_manifold = {
						v4(0.249307f, -0.217623f, -0.305579f, 1),
						v4(0.249307f, -0.217623f, +0.305580f, 1),
					},
					.m_feature = EFeature::Edge,
					.m_depth = 0.206549749f,
				};
				BangTogether(
					polytope, m4x4::TransformDeg(0, 90, 0, v4::Origin()),
					collision::ShapeLine(1.0f, 0.2f), m4x4::Translation(0.3f, -0.3f, 0),
					&exp);
			}

			// Line-edge vs edge
			{
				auto exp = collision::Contact{
					.m_axis = v4{-0.933257f,-0.311085f,-0.179605f,0},
					.m_manifold = {
						v4{0.316191f,-0.311744f,-0.467195f,1},
					},
					.m_feature = EFeature::Vert,
					.m_depth = 0.365301728f,
				};
				BangTogether(
					polytope, m4x4::TransformDeg(0, 0, 45, v4(0.4f, 0, 0, 1)),
					collision::ShapeLine(1.0f, 0.2f), m4x4::TransformDeg(30, 0, 0, v4(0.3f, -0.3f, -0.5f, 1)),
					&exp);
			}

			// Line-edge vs point
			{
				auto exp = collision::Contact{
					.m_axis = v4{1, 0, 0, 0},
					.m_manifold = {
						v4{0.79f, 0, 0, 1},
					},
					.m_feature = EFeature::Vert,
					.m_depth = 0.02f,
				};
				BangTogether(
					polytope, m4x4::TransformDeg(0, 90, 0, v4::Origin()),
					collision::ShapeLine(1.0f, 0.2f), m4x4::Translation(0.98f, 0, 0),
					&exp);
			}

			// Line-end vs face
			{
				auto exp = collision::Contact{
					.m_axis = v4{0.862364f,0.431187f,0.26534f,0},
					.m_manifold = {
						v4{0.0805324f,-0.0329575f,0.126166f,1},
					},
					.m_feature = EFeature::Vert,
					.m_depth = 0.247123376f,
				};
				BangTogether(
					polytope, m4x4::Identity(),
					collision::ShapeLine(1.0f, 0.2f), m4x4::TransformDeg(0, 45, 0, v4(0.5f, 0, 0.5f, 1)),
					&exp);
			}

			// Line-end vs edge
			{
				auto exp = collision::Contact{
					.m_axis = v4{-0.602213f,0.602213f,0.524097f, 0},
					.m_manifold = {
						v4{0.1106f,0.2894f,0.103746f,1},
					},
					.m_feature = EFeature::Vert,
					.m_depth = 0.0326877534f,
				};
				BangTogether(
					polytope, m4x4::TransformDeg(0, 0, 45, v4(0.4f, 0, 0, 1)),
					collision::ShapeLine(1.0f, 0.2f), m4x4::Translation(0, 0.4f, 0.7f),
					&exp);
			}

			// Line-edge vs point
			{
				auto exp = collision::Contact{
					.m_axis = v4{0, 0.627985f,0.778226f,0},
					.m_manifold = {
						v4{0.4f,-0.0193864f,0.775976f,1},
					},
					.m_feature = EFeature::Vert,
					.m_depth = 0.0617417544f,
				};
				BangTogether(
					polytope, m4x4::TransformDeg(0, 0, 45, v4(0.4f, 0, 0, 1)),
					collision::ShapeLine(1.0f, 0.2f), m4x4::TransformDeg(10, 0, 0, v4(0.4f, 0, 1.4f, 1)),
					&exp);
			}
		}

		// ---- Polytope vs Triangle ----
		PRUnitTestMethod(PolytopeVsTriangle)
		{
			constexpr v4 tet_pts[] = {
				v4(-0.8f, -0.8f, -0.5f, 1),
				v4(+0.8f, -0.8f, -0.5f, 1),
				v4(+0.0f, +0.8f, -0.5f, 1),
				v4(+0.0f, +0.0f, +0.8f, 1),
			};
			constexpr v4 tri_pts[] = {
				v4(-0.5f, -0.5f, 0, 1),
				v4(+0.5f, -0.5f, 0, 1),
				v4(0, +0.5f, 0, 1),
			};
			auto buf = collision::BuildPolytopeFromPoints(tet_pts);
			auto& polytope = buf.as<collision::ShapePolytope>();
			auto triangle = collision::ShapeTriangle(tri_pts[0], tri_pts[1], tri_pts[2]);

			// Tri-Face vs face
			{
				auto exp = collision::Contact{
					.m_axis = v4{0, 0, -1, 0},
					.m_manifold = {
						v4(+0.0f, +0.5f, -0.3f, 1),
						v4(+0.5f, -0.5f, -0.3f, 1),
						v4(-0.5f, -0.5f, -0.3f, 1),
					},
					.m_feature = EFeature::Tri,
					.m_depth = 0.4f,
				};
				BangTogether(
					polytope, m4x4::TransformDeg(0, 0, 0, v4(0, 0, 0, 1)),
					triangle, m4x4::TransformDeg(0, 0, 0, v4(0, 0, -0.1f, 1)),
					&exp);
			}
			
			// Tri-Face vs edge
			{
				auto exp = collision::Contact{
					.m_axis = v4{0, -1, 0, 0},
					.m_manifold = {
						v4(-0.4f, -0.79f, -0.5f, 1),
						v4(+0.4f, -0.79f, -0.5f, 1),
					},
					.m_feature = EFeature::Edge,
					.m_depth = 0.02f,
				};
				BangTogether(
					polytope, m4x4::TransformDeg(0, 0, 0, v4(0, 0, 0, 1)),
					triangle, m4x4::TransformDeg(90, 0, 0, v4(0, -0.78f, -0.2f, 1)),
					&exp);
			}

			// Tri-Face vs point
			{
				auto exp = collision::Contact{
					.m_axis = v4{0, 0, 1, 0},
					.m_manifold = {
						v4(0, 0, 0.775f, 1),
					},
					.m_feature = EFeature::Vert,
					.m_depth = 0.05f,
				};
				BangTogether(
					polytope, m4x4::TransformDeg(0, 0, 0, v4(0, 0, 0, 1)),
					triangle, m4x4::TransformDeg(0, 0, 0, v4(0, 0, 0.75f, 1)),
					&exp);
			}

			// Tri-Edge vs face
			{
				auto exp = collision::Contact{
					.m_axis = v4{0, 0, -1, 0},
					.m_manifold = {
						v4(-0.4f, 0, -0.4f, 1),
						v4(+0.4f, 0, -0.4f, 1),
					},
					.m_feature = EFeature::Edge,
					.m_depth = 0.2f,
				};
				BangTogether(
					polytope, m4x4::TransformDeg(0, 0, 0, v4(0, 0, 0, 1)),
					triangle, m4x4::TransformDeg(-90, 0, 0, v4(0, 0, -0.8f, 1)),
					&exp);
			}

			// Tri-Edge vs edge
			{
				auto exp = collision::Contact{
					.m_axis = v4{0, -1, 0, 0},
					.m_manifold = {
						v4(0.21f,-0.718223f,-0.5f,1),
					},
					.m_feature = EFeature::Vert,
					.m_depth = 0.163553417f,
				};
				BangTogether(
					polytope, m4x4::TransformDeg(0, 0, 0, v4(0, 0, 0, 1)),
					triangle, m4x4::TransformDeg(-135, 45, 0, v4(0, -0.99f, -0.79f, 1)),
					&exp);
			}

			// Tri-Vert vs face
			{
				auto exp = collision::Contact{
					.m_axis = v4{-0.862366f,0.431183f,0.265343f,0},
					.m_manifold = {
						v4(-0.121166f,0.0105831f,0.296513f,1),
					},
					.m_feature = EFeature::Vert,
					.m_depth = 0.0490884483f,
				};
				BangTogether(
					polytope, m4x4::TransformDeg(0, 0, 0, v4(0, 0, 0, 1)),
					triangle, m4x4::TransformDeg(90, 0, 0, v4(-0.6f, 0, 0.79f, 1)),
					&exp);
			}

			// Point/Edge vs point isn't possible
		}

		// ---- Polytope vs Polytope ----
		PRUnitTestMethod(PolytopeVsPolytope)
		{
			// Two tetrahedra overlapping at origin
			{
				v4 tet_pts[] = {
					v4{-1, -1, -1, 1}, v4{1, -1, -1, 1},
					v4{0, 1, -1, 1}, v4{0, 0, 1, 1},
				};
				auto buf_a = collision::BuildPolytopeFromPoints(tet_pts);
				auto buf_b = collision::BuildPolytopeFromPoints(tet_pts);
				
				// Slight overlap along X
				auto exp = collision::Contact{
					.m_axis = v4{0.742781f,0.371391f,-0.557086f, 0},
					.m_manifold = {
						v4{0.224136f,-0.48275f,-0.689656f,1},
					},
					.m_feature = EFeature::Vert,
					.m_depth = 1.11416829f,
				};
				BangTogether(
					buf_a.as<collision::ShapePolytope>(), m4x4::Identity(),
					buf_b.as<collision::ShapePolytope>(), m4x4::Translation(0.5f, 1e-5f, 0),
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

				auto exp = collision::Contact{
					.m_axis = v4{1,0,0,0},
					.m_manifold = {
						v4(0.75f, +1, +1, 1),
						v4(0.75f, -1, +1, 1),
						v4(0.75f, -1, -1, 1),
						v4(0.75f, +1, -1, 1),
					},
					.m_feature = EFeature::Quad,
					.m_depth = 0.5f,
				};
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
				
				auto exp = collision::Contact{
					.m_axis = v4{0.872872f,0.436436f,0.218218f,0},
					.m_manifold = {
						v4{0.238095f,-0.380952f,-0.440476f,1},
					},
					.m_feature = EFeature::Vert,
					.m_depth = 0.545544744f,
				};
				BangTogether(
					buf_a.as<collision::ShapePolytope>(), m4x4::Identity(),
					buf_b.as<collision::ShapePolytope>(), m4x4::Translation(0.5f, 0, 0),
					&exp);
			}
		}
	};

	PRUnitTestClass(GpuContactManifoldReducerTests)
	{
		static void CheckBoxDropContact(GpuContact const& contact)
		{
			constexpr auto Tol = 1e-5f;

			PR_EXPECT(contact.feature == FEATURE_QUAD);
			PR_EXPECT(FEqlRelative(contact.axis, v4(0, 0, -1, 0), Tol));
			PR_EXPECT(FEqlAbsolute(contact.depth, 0.01f, Tol));

			auto centroid = ContactCentroid(contact);
			PR_EXPECT(FEqlAbsolute(centroid.x, 0.0f, Tol));
			PR_EXPECT(FEqlAbsolute(centroid.y, 0.0f, Tol));
			PR_EXPECT(FEqlAbsolute(centroid.z, -0.005f, Tol));
			PR_EXPECT(FEqlAbsolute(centroid.w, 1.0f, Tol));

			for (int i = 0; i != contact.feature; ++i)
			{
				auto pt = contact.manifold[i];
				PR_EXPECT(pt.x >= -0.5f - Tol && pt.x <= +0.5f + Tol);
				PR_EXPECT(pt.y >= -0.5f - Tol && pt.y <= +0.5f + Tol);
				PR_EXPECT(FEqlAbsolute(pt.z, -0.005f, Tol));
				PR_EXPECT(FEqlAbsolute(pt.w, 1.0f, Tol));
			}
		}

		PRUnitTestMethod(BoxDropSupportFeatures)
		{
			auto box = collision::ShapeBox(v4(1, 1, 1, 0));
			auto grd = collision::ShapeBox(v4(100.0f, 100.0f, 1.0f, 0));
			auto box_to_world = m4x4::Translation(0, 0, 0.49f);
			auto grd_to_world = m4x4::Translation(0, 0, -0.5f);

			StructuredBuffer<float4> verts;
			auto sbox = PackShape(box, verts);
			auto sgrd = PackShape(grd, verts);
			auto axis = float4(0, 0, -1, 0);

			float3 pts_box[4], pts_grd[4];
			auto count_box = BoxSupportFeature(box_to_world[3].xyz, (float3x3)box_to_world, sbox.data.xyz, +axis.xyz, pts_box);
			auto count_grd = BoxSupportFeature(grd_to_world[3].xyz, (float3x3)grd_to_world, sgrd.data.xyz, -axis.xyz, pts_grd);
			PR_EXPECT(count_box == FEATURE_QUAD);
			PR_EXPECT(count_grd == FEATURE_QUAD);

			GpuFeature feat_box, feat_grd;
			FeatureClear(feat_box);
			FeatureClear(feat_grd);
			feat_box.count = count_box;
			feat_grd.count = count_grd;
			for (int i = 0; i != count_box; ++i)
				feat_box.points[i] = float4(pts_box[i], 1);
			for (int i = 0; i != count_grd; ++i)
				feat_grd.points[i] = float4(pts_grd[i], 1);

			GpuContact contact;
			FindContactManifold(feat_box, feat_grd, axis, 0.01f, contact);
			CheckBoxDropContact(contact);
		}

		PRUnitTestMethod(BoxDropBoxVsBox)
		{
			auto box = collision::ShapeBox(v4(1, 1, 1, 0));
			auto grd = collision::ShapeBox(v4(100.0f, 100.0f, 1.0f, 0));
			auto box_to_world = m4x4::Translation(0, 0, 0.49f);
			auto grd_to_world = m4x4::Translation(0, 0, -0.5f);

			StructuredBuffer<float4> verts;
			auto sbox = PackShape(box, verts);
			auto sgrd = PackShape(grd, verts);

			GpuContact contact;
			auto hit = BoxVsBox(sbox, box_to_world, sgrd, grd_to_world, contact);
			collision::tests::VisualiseCollision(temp_dir() / L"LDraw/collision.ldr", box, box_to_world, grd, grd_to_world, To<collision::Contact>(contact));
			PR_EXPECT(hit);
			CheckBoxDropContact(contact);
		}
	};
}
#endif
