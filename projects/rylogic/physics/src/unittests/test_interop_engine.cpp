//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// Diagnostic harness for the GPU pipeline: stitches together the C++ interop
// (i.e. CPU-emulated) versions of the compute shaders into a complete simulation
// step, so that scenarios which hang on the actual GPU can be run, debugged, and
// stepped through line-by-line in C++.
//
// The pipeline mirrors physics::Engine::Step():
//   1. Pack    — build GpuRigidBody + GpuShape buffers from the input bodies.
//   2. Integrate — physics::Evolve() (CPU) advances o2w/momentum by dt.
//   3. Broad-phase — brute-force AABB overlap (small body counts only).
//   4. Narrow-phase — physics::CollideShapes() (the C++-compiled HLSL) per pair.
//   5. Resolve  — ResolveInteropRunner runs the position+velocity solver passes.
//   6. Unpack   — write GpuRigidBody state back into the RigidBody objects.
//
// Skipping the actual GPU keeps the run fully synchronous and debuggable; tests
// can compare behaviour against the GPU-driven Engine to localise hangs/crashes.
#if PR_UNITTESTS
#include "pr/common/unittests.h"
#include "pr/physics/physics.h"
#include "src/compute/physics_types.h"
#include "src/compute/collision.hlsli"
#include "src/compute/gjk.hlsli"
#include "src/compute/interop/resolve_runner.h"

namespace pr::physics::tests
{
	void ForceLink_InteropEngine() {}

	// Lightweight CPU-only mirror of physics::Engine::Step().
	// Owns a ResolveInteropRunner and runs the same solver stages on the CPU.
	struct InteropEngine
	{
		EngineConfig m_config;
		std::vector<GpuMaterial> m_materials;
		ResolveInteropRunner m_resolver;

		struct StepStats
		{
			int contact_count = 0;
			int broadphase_pair_count = 0;
		};

		// Notes:
		//  - The resolver runner is re-used across steps; the contact/body buffers it
		//    owns are overwritten on each call to Run() so no per-step state leaks.
		//  - A single default GpuMaterial is installed at id 0 — callers can append more.
		explicit InteropEngine(EngineConfig const& config = {})
			: m_config(config)
			, m_materials{ GpuMaterial{ .friction_static = 0.0f, .elasticity_norm = 1.0f } }
			, m_resolver(config)
		{
		}

		// Run one simulation step.
		StepStats Step(float dt, std::span<RigidBody*> bodies)
		{
			StepStats stats;
			if (bodies.empty())
				return stats;

			// 1. Pack bodies and shapes into GPU-format buffers.
			hlsl::StructuredBuffer<float4> verts;
			std::vector<GpuShape> shapes;
			std::vector<GpuRigidBody> gpu_bodies;
			shapes.reserve(bodies.size());
			gpu_bodies.reserve(bodies.size());
			for (size_t i = 0; i != bodies.size(); ++i)
			{
				shapes.push_back(PackShape(bodies[i]->Shape(), verts));
				gpu_bodies.push_back(PackDynamics(*bodies[i], static_cast<int>(i)));
			}

			// 2. Integrate (CPU). Static/infinite-mass bodies don't move.
			for (auto* b : bodies)
			{
				if (b->InvMass() == 0.0f)
					continue;
				physics::Evolve(*b, dt);
			}

			// Refresh the GPU-format bodies after integration so the narrow-phase sees moved bodies.
			for (size_t i = 0; i != bodies.size(); ++i)
				gpu_bodies[i] = PackDynamics(*bodies[i], static_cast<int>(i));

			// 3 + 4. All-pairs brute-force broadphase + narrow-phase via CollideShapes.
			std::vector<GpuResolveContact> contacts;
			for (size_t a = 0; a != bodies.size(); ++a)
			{
				for (size_t b = a + 1; b != bodies.size(); ++b)
				{
					// Skip pairs of two static bodies — they never need contacts.
					if (bodies[a]->InvMass() == 0.0f && bodies[b]->InvMass() == 0.0f)
						continue;

					auto a2w = bodies[a]->O2W();
					auto b2w = bodies[b]->O2W();
					auto b2a = InvertOrthonormal(a2w) * b2w;
					++stats.broadphase_pair_count;

					// CollideShapes runs in body-A's local space (matches CSCollide semantics).
					GpuContact gc{};
					if (!physics::CollideShapes(shapes[a], m4x4::Identity(), shapes[b], b2a, verts, gc))
						continue;

					GpuResolveContact rc{};
					rc.axis = gc.axis;
					rc.contact_point = ContactCentroid(gc);
					for (int k = 0; k != GpuContactMaxPoints; ++k)
						rc.manifold[k] = gc.manifold[k];
					rc.b2a = b2a;
					rc.body_idx_a = static_cast<int>(a);
					rc.body_idx_b = static_cast<int>(b);
					rc.mat_id_a = shapes[a].material_id;
					rc.mat_id_b = shapes[b].material_id;
					rc.depth = gc.depth;
					rc.collision_time = 0.0f;
					rc.feature = gc.feature;
					rc.pad1 = 0;
					contacts.push_back(rc);
				}
			}
			stats.contact_count = static_cast<int>(contacts.size());

			// 5. Resolve. Skip when there are no contacts (matches the real engine's path).
			if (!contacts.empty())
			{
				m_resolver.Run(ResolveRunnerBuffers{
					.m_dt = dt,
					.m_bodies = gpu_bodies,
					.m_contacts = contacts,
					.m_materials = m_materials,
				});
			}

			// 6. Unpack solver results back into the input RigidBody objects.
			for (size_t i = 0; i != bodies.size(); ++i)
				UnpackDynamics(gpu_bodies[i], *bodies[i]);

			return stats;
		}
	};

	PRUnitTestClass(InteropEngineTests)
	{
		// Box-vs-box head-on collision driven entirely through the C++ interop
		// pipeline. Equivalent to CollisionPairTests.BoxVsBox but with no GPU
		// involvement, so it can be debugged step-by-step when the GPU version hangs.
		PRUnitTestMethod(BoxVsBoxHeadOn)
		{
			auto box_a = collision::ShapeBox(v4{0.5f, 0.5f, 0.5f, 0});
			auto box_b = collision::ShapeBox(v4{0.5f, 0.5f, 0.5f, 0});

			auto inertia = physics::Inertia::Box(v4{0.5f, 0.5f, 0.5f, 0}, 10.0f);

			// Two unit boxes 1.5m apart on the X axis, closing at 3 m/s each (head-on).
			physics::RigidBody bodies[2] = {
				physics::RigidBody{collision::shape_cast(&box_a), m4x4::Translation(-0.75f, 0, 0), inertia},
				physics::RigidBody{collision::shape_cast(&box_b), m4x4::Translation(+0.75f, 0, 0), inertia},
			};
			bodies[0].VelocityWS(v4::Zero(), v4{+3.0f, 0, 0, 0});
			bodies[1].VelocityWS(v4::Zero(), v4{-3.0f, 0, 0, 0});

			physics::RigidBody* body_ptrs[2] = {&bodies[0], &bodies[1]};

			InteropEngine engine{};

			bool collision_occurred = false;
			auto const dt = 1.0f / 60.0f;
			for (int step = 0; step != 100 && !collision_occurred; ++step)
			{
				bodies[0].ZeroForces();
				bodies[1].ZeroForces();
				auto stats = engine.Step(dt, body_ptrs);
				if (stats.contact_count > 0)
				{
					collision_occurred = true;
					// After an elastic head-on with equal masses the bodies should reverse direction.
					PR_EXPECT(bodies[0].VelocityWS().lin.x < -0.5f);
					PR_EXPECT(bodies[1].VelocityWS().lin.x > +0.5f);
				}
			}
			PR_EXPECT(collision_occurred);
		}
	};
}
#endif
