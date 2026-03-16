//************************************
// Physics-2 Engine
//  Copyright (c) Rylogic Ltd 2016
//************************************
// Unit tests for collision response (impulse resolution).
// These tests verify conservation of linear momentum, angular momentum, and kinetic energy
// across a range of collision configurations, from simple 1D head-on to oblique 3D impacts.
//
// Test approach:
//   1. Create two rigid bodies with known shapes, masses, positions, and velocities.
//   2. Add them to a brute-force broadphase engine with perfectly elastic, frictionless material.
//   3. Step the engine at a fixed timestep until a collision occurs.
//   4. Check that conservation laws hold and (where applicable) post-collision velocities
//      match the analytic solution for 1D elastic collisions.
//
// The analytic formulas for 1D elastic collisions:
//   v1' = ((m1-m2)*v1 + 2*m2*v2) / (m1+m2)
//   v2' = ((m2-m1)*v2 + 2*m1*v1) / (m1+m2)
//
#pragma once

#if PR_UNITTESTS
#include "pr/common/unittests.h"
#include "pr/collision/shape_box.h"
#include "pr/physics-2/collision/broadphase_brute.h"
#include "pr/physics-2/integrator/engine.h"
#include "pr/physics-2/rigid_body/rigid_body.h"
#include "pr/physics-2/shape/inertia.h"
#include "pr/physics-2/materials/material_map.h"

namespace pr::physics
{
	using collision::ShapeBox;
	PRUnitTestClass(CollisionTests)
	{
		// Snapshot of the system's conserved quantities at a moment in time.
		// Used to compare before/after collision for conservation checks.
		struct SystemState
		{
			v4 total_lin_momentum; // Sum of m*v for all bodies
			v4 total_ang_momentum; // Sum of (r × m*v + I*ω) about the world origin
			float total_ke;        // Sum of 0.5*m*v² + 0.5*ω·I·ω for all bodies

			// Capture the system state from a pair of bodies.
			// Angular momentum is computed about the world origin, which requires
			// the orbital term (r × p) in addition to the spin term (I*ω).
			static SystemState Capture(RigidBody const& a, RigidBody const& b)
			{
				auto s = SystemState{};
				s.total_lin_momentum = a.MomentumWS().lin + b.MomentumWS().lin;

				// Spin angular momentum (I*ω) is stored in MomentumWS().ang at each body's origin.
				// Orbital angular momentum (r × p) accounts for the body's position relative to
				// the measurement point (world origin).
				auto La = a.MomentumWS().ang + Cross(a.O2W().pos, a.MomentumWS().lin);
				auto Lb = b.MomentumWS().ang + Cross(b.O2W().pos, b.MomentumWS().lin);
				s.total_ang_momentum = La + Lb;

				s.total_ke = a.KineticEnergy() + b.KineticEnergy();
				return s;
			}
		};

		// Result of running a collision scenario, containing before/after state
		// and the post-collision velocities for analytic comparison.
		struct CollisionResult
		{
			SystemState before;
			SystemState after;
			v8motion vel_a; // Post-collision velocity of body A
			v8motion vel_b; // Post-collision velocity of body B
			bool collision_occurred;
		};

		// Run a two-body collision scenario to completion.
		// Sets up the engine with perfectly elastic frictionless material,
		// steps at a fixed dt until a collision is detected, then captures
		// the post-impulse state. Returns the before/after system state.
		static CollisionResult RunScenario(
			ShapeBox& box,
			v4 pos_a, v4 vel_a, float mass_a,
			v4 pos_b, v4 vel_b, float mass_b,
			v4 ang_vel_a = v4::Zero(), v4 ang_vel_b = v4::Zero())
		{
			auto result = CollisionResult{};
			result.collision_occurred = false;

			// Create rigid bodies using the template constructor which handles shape_cast internally.
			// We use the Inertia::Box overload that takes half-extents and mass.
			auto inertia_a = Inertia::Box(box.m_radius, mass_a);
			auto inertia_b = Inertia::Box(box.m_radius, mass_b);
			RigidBody bodies[2] = {
				RigidBody{&box, m4x4::Translation(pos_a), inertia_a},
				RigidBody{&box, m4x4::Translation(pos_b), inertia_b},
			};
			auto& body_a = bodies[0];
			auto& body_b = bodies[1];
			body_a.VelocityWS(ang_vel_a, vel_a);
			body_b.VelocityWS(ang_vel_b, vel_b);

			// Set up the engine with perfectly elastic, frictionless material
			MaterialMap mats;
			Engine engine(mats);

			auto& mat = mats(0);
			mat.m_elasticity_norm = 1.0f;  // Perfectly elastic
			mat.m_elasticity_tang = 0.0f;
			mat.m_elasticity_tors = 0.0f;
			mat.m_friction_static = 0.0f;  // No friction

			engine.Broadphase().Add(body_a);
			engine.Broadphase().Add(body_b);

			// Hook the PostCollisionDetection event to capture pre-impulse state.
			// This fires after Evolve and collision detection, but before impulse resolution.
			engine.PostCollisionDetection += [&](auto&, auto args)
			{
				if (args.m_contacts.empty())
					return;

				result.before = SystemState::Capture(body_a, body_b);
				result.collision_occurred = true;
			};

			// Step until collision or timeout.
			// Fixed 100 Hz timestep; 5000 steps = 50 seconds of simulation time.
			auto const dt = 1.0f / 100.0f;
			auto const max_steps = 5000;
			for (int step = 0; step < max_steps; ++step)
			{
				body_a.ZeroForces();
				body_b.ZeroForces();

				engine.Step(dt, bodies);

				if (result.collision_occurred)
				{
					// Capture post-impulse state immediately after the collision step
					result.after = SystemState::Capture(body_a, body_b);
					result.vel_a = body_a.VelocityWS();
					result.vel_b = body_b.VelocityWS();
					break;
				}
			}
			return result;
		}

		// Scenario 1: Two equal-mass boxes approaching each other head-on along X.
		// The classic "Newton's cradle" setup. For perfectly elastic collisions between
		// equal masses, velocities swap exactly: v1' = v2, v2' = v1.
		PRUnitTestMethod(HeadOnEqualMass)
		{
			auto box = ShapeBox(v4{2, 2, 2, 0});
			auto r = RunScenario(box,
				v4{-5, 0, 0, 1}, v4{+3, 0, 0, 0}, 10.0f,
				v4{+5, 0, 0, 1}, v4{-3, 0, 0, 0}, 10.0f);

			PR_EXPECT(r.collision_occurred);

			// Conservation laws
			PR_EXPECT(FEqlRelative(r.after.total_lin_momentum, r.before.total_lin_momentum, 0.001f));
			PR_EXPECT(FEqlRelative(r.after.total_ke, r.before.total_ke, 0.001f));

			// Analytic: equal masses swap velocities
			PR_EXPECT(FEqlRelative(r.vel_a.lin.x, -3.0f, 0.001f));
			PR_EXPECT(FEqlRelative(r.vel_b.lin.x, +3.0f, 0.001f));

			// No spurious Y/Z velocity or angular velocity
			PR_EXPECT(Abs(r.vel_a.lin.y) < 0.01f);
			PR_EXPECT(Abs(r.vel_a.lin.z) < 0.01f);
			PR_EXPECT(Abs(r.vel_b.lin.y) < 0.01f);
			PR_EXPECT(Abs(r.vel_b.lin.z) < 0.01f);
			PR_EXPECT(Length(r.vel_a.ang) < 0.01f);
			PR_EXPECT(Length(r.vel_b.ang) < 0.01f);
		}

		// Scenario 2: Unequal masses (10 vs 5) approaching head-on along X.
		// The heavier body slows down, the lighter one speeds up.
		// Analytic: v1' = ((m1-m2)*v1 + 2*m2*v2)/(m1+m2)
		//           v2' = ((m2-m1)*v2 + 2*m1*v1)/(m1+m2)
		PRUnitTestMethod(HeadOnDiffMass)
		{
			auto box = ShapeBox(v4{2, 2, 2, 0});
			auto r = RunScenario(box,
				v4{-5, 0, 0, 1}, v4{+3, 0, 0, 0}, 10.0f,
				v4{+5, 0, 0, 1}, v4{-3, 0, 0, 0}, 5.0f);

			PR_EXPECT(r.collision_occurred);

			// Conservation
			PR_EXPECT(FEqlRelative(r.after.total_lin_momentum, r.before.total_lin_momentum, 0.001f));
			PR_EXPECT(FEqlRelative(r.after.total_ke, r.before.total_ke, 0.001f));

			// Analytic: v1' = (10-5)*3 + 2*5*(-3)) / 15 = (15-30)/15 = -1.0
			//           v2' = (5-10)*(-3) + 2*10*3) / 15 = (15+60)/15 = 5.0
			PR_EXPECT(FEqlRelative(r.vel_a.lin.x, -1.0f, 0.001f));
			PR_EXPECT(FEqlRelative(r.vel_b.lin.x, +5.0f, 0.001f));

			// No angular velocity for centred 1D collision
			PR_EXPECT(Length(r.vel_a.ang) < 0.01f);
			PR_EXPECT(Length(r.vel_b.ang) < 0.01f);
		}

		// Scenario 3: A moving box hits a stationary box of equal mass.
		// The classic billiard ball scenario — the moving body stops and
		// the stationary body takes on the original velocity.
		PRUnitTestMethod(StationaryTarget)
		{
			auto box = ShapeBox(v4{2, 2, 2, 0});
			auto r = RunScenario(box,
				v4{-5, 0, 0, 1}, v4{+3, 0, 0, 0}, 10.0f,
				v4{+5, 0, 0, 1}, v4{ 0, 0, 0, 0}, 10.0f);

			PR_EXPECT(r.collision_occurred);

			// Conservation
			PR_EXPECT(FEqlRelative(r.after.total_lin_momentum, r.before.total_lin_momentum, 0.001f));
			PR_EXPECT(FEqlRelative(r.after.total_ke, r.before.total_ke, 0.001f));

			// Analytic: moving body stops, target takes the velocity
			PR_EXPECT(FEqlRelative(r.vel_a.lin.x, 0.0f, 0.05f));
			PR_EXPECT(FEqlRelative(r.vel_b.lin.x, 3.0f, 0.001f));

			// No angular velocity
			PR_EXPECT(Length(r.vel_a.ang) < 0.01f);
			PR_EXPECT(Length(r.vel_b.ang) < 0.01f);
		}

		// Scenario 4: Off-centre collision that induces rotation.
		// Body A is offset in Y so the contact point is not aligned with either CoM.
		// This produces angular impulse in addition to linear impulse, verifying
		// that the collision mass matrix correctly couples translation and rotation.
		PRUnitTestMethod(OffCentreRotation)
		{
			auto box = ShapeBox(v4{2, 2, 2, 0});
			auto r = RunScenario(box,
				v4{-5, +0.8f, 0, 1}, v4{+3, 0, 0, 0}, 10.0f,
				v4{+5,     0, 0, 1}, v4{ 0, 0, 0, 0}, 10.0f);

			PR_EXPECT(r.collision_occurred);

			// Linear momentum: must be exactly conserved
			PR_EXPECT(FEqlRelative(r.after.total_lin_momentum, r.before.total_lin_momentum, 0.001f));

			// KE: must be conserved for elastic collision
			PR_EXPECT(FEqlRelative(r.after.total_ke, r.before.total_ke, 0.001f));

			// Angular momentum: conserved within sub-step approximation tolerance.
			// The impulse is applied at the estimated collision time, but body positions
			// are at the Evolve time, so there's a small error proportional to penetration depth.
			auto dL = r.after.total_ang_momentum - r.before.total_ang_momentum;
			auto L_mag = Length(r.before.total_ang_momentum);
			auto ang_tol = L_mag > 0.01f ? L_mag * 0.05f : 0.01f;
			PR_EXPECT(Length(dL) < ang_tol);

			// Both bodies should now be spinning (non-zero angular velocity)
			PR_EXPECT(Length(r.vel_a.ang) > 0.1f);
			PR_EXPECT(Length(r.vel_b.ang) > 0.1f);

			// Body A should have slowed down in X (gave momentum to B)
			PR_EXPECT(r.vel_a.lin.x < 3.0f);
			PR_EXPECT(r.vel_b.lin.x > 0.0f);
		}

		// Scenario 5: Oblique collision — bodies approach at an angle.
		// Both bodies have Y-velocity components, so the collision is not
		// purely along the contact normal. Tests that the impulse correctly
		// resolves only the normal component (frictionless), leaving the
		// tangential velocity unchanged.
		PRUnitTestMethod(ObliqueCollision)
		{
			auto box = ShapeBox(v4{2, 2, 2, 0});
			auto r = RunScenario(box,
				v4{-5, -2, 0, 1}, v4{+3, +1, 0, 0}, 10.0f,
				v4{+5, +2, 0, 1}, v4{-3, -1, 0, 0}, 10.0f);

			PR_EXPECT(r.collision_occurred);

			// Linear momentum: exactly conserved
			PR_EXPECT(FEqlRelative(r.after.total_lin_momentum, r.before.total_lin_momentum, 0.001f));

			// KE: conserved for elastic collision
			PR_EXPECT(FEqlRelative(r.after.total_ke, r.before.total_ke, 0.001f));

			// Angular momentum: within sub-step tolerance
			auto dL = r.after.total_ang_momentum - r.before.total_ang_momentum;
			auto L_mag = Length(r.before.total_ang_momentum);
			auto ang_tol = L_mag > 0.01f ? L_mag * 0.05f : 0.01f;
			PR_EXPECT(Length(dL) < ang_tol);

			// Y velocity should be preserved (frictionless, normal is along X).
			// The normal impulse only affects the X component.
			PR_EXPECT(FEqlRelative(r.vel_a.lin.y, +1.0f, 0.05f));
			PR_EXPECT(FEqlRelative(r.vel_b.lin.y, -1.0f, 0.05f));

			// X velocity should have changed (reflected by the collision)
			PR_EXPECT(r.vel_a.lin.x < +3.0f);
			PR_EXPECT(r.vel_b.lin.x > -3.0f);
		}

		// Scenario 6: Heavy body hits a light stationary body.
		// The heavy body barely slows down, the light body flies away fast.
		// Tests the asymmetric mass case where one body is much heavier.
		PRUnitTestMethod(HeavyHitsLight)
		{
			auto box = ShapeBox(v4{2, 2, 2, 0});
			auto r = RunScenario(box,
				v4{-5, 0, 0, 1}, v4{+2, 0, 0, 0}, 50.0f,
				v4{+5, 0, 0, 1}, v4{ 0, 0, 0, 0}, 5.0f);

			PR_EXPECT(r.collision_occurred);

			// Conservation
			PR_EXPECT(FEqlRelative(r.after.total_lin_momentum, r.before.total_lin_momentum, 0.001f));
			PR_EXPECT(FEqlRelative(r.after.total_ke, r.before.total_ke, 0.001f));

			// Analytic: v1' = (50-5)*2/(50+5) = 90/55 ≈ 1.636
			//           v2' = 2*50*2/(50+5) = 200/55 ≈ 3.636
			auto v1_pred = ((50.0f - 5.0f) * 2.0f) / (50.0f + 5.0f);
			auto v2_pred = (2.0f * 50.0f * 2.0f) / (50.0f + 5.0f);
			PR_EXPECT(FEqlRelative(r.vel_a.lin.x, v1_pred, 0.001f));
			PR_EXPECT(FEqlRelative(r.vel_b.lin.x, v2_pred, 0.001f));
		}

		// Scenario 7: Light body hits a heavy stationary body.
		// The light body bounces back, the heavy body barely moves.
		// In the limit of infinite mass ratio, the light body perfectly reflects.
		PRUnitTestMethod(LightHitsHeavy)
		{
			auto box = ShapeBox(v4{2, 2, 2, 0});
			auto r = RunScenario(box,
				v4{-5, 0, 0, 1}, v4{+4, 0, 0, 0}, 5.0f,
				v4{+5, 0, 0, 1}, v4{ 0, 0, 0, 0}, 50.0f);

			PR_EXPECT(r.collision_occurred);

			// Conservation
			PR_EXPECT(FEqlRelative(r.after.total_lin_momentum, r.before.total_lin_momentum, 0.001f));
			PR_EXPECT(FEqlRelative(r.after.total_ke, r.before.total_ke, 0.001f));

			// Analytic: v1' = (5-50)*4/(5+50) = -180/55 ≈ -3.273 (bounces back)
			//           v2' = 2*5*4/(5+50) = 40/55 ≈ 0.727 (barely moves)
			auto v1_pred = ((5.0f - 50.0f) * 4.0f) / (5.0f + 50.0f);
			auto v2_pred = (2.0f * 5.0f * 4.0f) / (5.0f + 50.0f);
			PR_EXPECT(FEqlRelative(r.vel_a.lin.x, v1_pred, 0.001f));
			PR_EXPECT(FEqlRelative(r.vel_b.lin.x, v2_pred, 0.001f));

			// Light body should bounce back (negative X velocity)
			PR_EXPECT(r.vel_a.lin.x < 0.0f);
		}

		// Scenario 8: Symmetric oblique — bodies approach symmetrically about the X axis.
		// By symmetry, post-collision should also be symmetric. Both should gain equal
		// and opposite angular velocities and have mirrored linear velocities.
		PRUnitTestMethod(SymmetricOblique)
		{
			auto box = ShapeBox(v4{2, 2, 2, 0});
			auto r = RunScenario(box,
				v4{-5, -1, 0, 1}, v4{+3, +0.5f, 0, 0}, 10.0f,
				v4{+5, +1, 0, 1}, v4{-3, -0.5f, 0, 0}, 10.0f);

			PR_EXPECT(r.collision_occurred);

			// Conservation
			PR_EXPECT(FEqlRelative(r.after.total_lin_momentum, r.before.total_lin_momentum, 0.001f));
			PR_EXPECT(FEqlRelative(r.after.total_ke, r.before.total_ke, 0.001f));

			// By symmetry: velocities should be mirrored (vA.x ≈ -vB.x, vA.y ≈ -vB.y)
			PR_EXPECT(FEqlRelative(r.vel_a.lin.x, -r.vel_b.lin.x, 0.05f));
			PR_EXPECT(FEqlRelative(r.vel_a.lin.y, -r.vel_b.lin.y, 0.05f));
		}

		// Scenario 9: Glancing contact — bodies barely overlap in Y.
		// The contact area is small (edges just touching). This tests
		// that the collision detection and impulse work for near-miss geometry.
		PRUnitTestMethod(GlancingContact)
		{
			auto box = ShapeBox(v4{2, 2, 2, 0});

			// Offset by 1.9 in Y — just 0.1 overlap (box half-extent is 1.0)
			auto r = RunScenario(box,
				v4{-5, +1.9f, 0, 1}, v4{+3, 0, 0, 0}, 10.0f,
				v4{+5,     0, 0, 1}, v4{ 0, 0, 0, 0}, 10.0f);

			PR_EXPECT(r.collision_occurred);

			// Conservation laws must still hold
			PR_EXPECT(FEqlRelative(r.after.total_lin_momentum, r.before.total_lin_momentum, 0.001f));
			PR_EXPECT(FEqlRelative(r.after.total_ke, r.before.total_ke, 0.01f));

			// Should produce significant angular velocity due to the large lever arm
			PR_EXPECT(Length(r.vel_a.ang) > 0.1f);
			PR_EXPECT(Length(r.vel_b.ang) > 0.1f);
		}
	};
}
#endif
