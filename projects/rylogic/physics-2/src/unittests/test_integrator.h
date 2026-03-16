//************************************
// Physics-2 Engine
//  Copyright (c) Rylogic Ltd 2016
//************************************
#pragma once

#if PR_UNITTESTS
#include "pr/common/unittests.h"
#include "pr/physics-2/integrator/integrator.h"
#include "pr/physics-2/rigid_body/rigid_body_dynamics.h"
#include "pr/physics-2/shape/inertia.h"

namespace pr::physics
{
	PRUnitTestClass(IntegratorTests)
	{
		PRUnitTestMethod(IntegratorTests)
		{
			auto mass = 5.0f;
			auto force = v8force{1,1,1, 1,1,-1};

			// Set up a rigid body at rest
			auto rb = RigidBody{};
			rb.SetMassProperties(Inertia::Sphere(1, mass));

			// Initial KE should be zero
			auto ke0 = rb.KineticEnergy();
			PR_EXPECT(FEql(ke0, 0.f));

			// Get it moving by applying forces/torques
			auto dke = KineticEnergyChange(force, rb.MomentumWS(), rb.InertiaInvWS(), 1.0f);
			rb.ApplyForceWS(force);
			Evolve(rb, 1.0f);

			// KE Gained
			ke0 += dke;
			auto ke1 = rb.KineticEnergy();
			PR_EXPECT(FEql(ke0, ke1));

			// More force
			dke = KineticEnergyChange(force, rb.MomentumWS(), rb.InertiaInvWS(), 1.0f);
			rb.ApplyForceWS(force);
			Evolve(rb, 1.0f);

			// KE Gained again
			ke0 += dke;
			auto ke2 = rb.KineticEnergy();
			PR_EXPECT(FEql(ke0, ke2));

			// No force
			dke = 0;
			Evolve(rb, 1.0f);

			// KE unchanged
			ke0 += dke;
			auto ke3 = rb.KineticEnergy();
			PR_EXPECT(FEql(ke0, ke3));

			// Apply a force to stop the motion
			force = -rb.MomentumWS();
			dke = KineticEnergyChange(force, rb.MomentumWS(), rb.InertiaInvWS(), 1.0f);
			rb.ApplyForceWS(force);
			Evolve(rb, 1.0f);

			// KE lost
			ke0 += dke;
			auto ke4 = rb.KineticEnergy();
			PR_EXPECT(FEql(ke0, ke4));

			// KE back to zero
			PR_EXPECT(FEql(ke0, 0.f));
		}
	};

	// Compare Evolve() (reference) and EvolveCPU() (GPU-equivalent) for different body types.
	// Any mismatch here means the GPU compute shader will also produce wrong results.
	PRUnitTestClass(EvolveCPUComparisonTests)
	{
		// Run one integration step through both paths and compare results.
		// Returns the max absolute error across position, rotation, and momentum.
		static float CompareOneStep(RigidBody const& rb_template, float dt)
		{
			// Path A: Evolve() — the known-good reference
			auto rb_a = rb_template;
			Evolve(rb_a, dt);

			// Path B: Pack → EvolveCPU → compare (don't unpack, just compare raw)
			auto rb_b = rb_template;
			auto dyn = PackDynamics(rb_b);
			Evolve(dyn, dt);

			// Compare transform
			auto pos_err = Length(rb_a.O2W().pos - dyn.o2w.pos);
			auto rot_err_x = Length(rb_a.O2W().x - dyn.o2w.x);
			auto rot_err_y = Length(rb_a.O2W().y - dyn.o2w.y);
			auto rot_err_z = Length(rb_a.O2W().z - dyn.o2w.z);

			// Compare momentum
			auto mom_ang_err = Length(rb_a.MomentumWS().ang - dyn.momentum_ang);
			auto mom_lin_err = Length(rb_a.MomentumWS().lin - dyn.momentum_lin);

			return Max(pos_err, Max(rot_err_x, Max(rot_err_y, Max(rot_err_z, Max(mom_ang_err, mom_lin_err)))));
		}

		// Sphere at origin (CoM == 0, simplest case)
		PRUnitTestMethod(Sphere_NoForce)
		{
			auto rb = RigidBody{};
			rb.SetMassProperties(Inertia::Sphere(1.0f, 10.0f));
			rb.O2W(m4x4::Translation(v4{0, 0, 5, 1}));
			rb.VelocityWS(v4{0.5f, 0.3f, 0, 0}, v4{1.0f, 0, -2.0f, 0});

			for (int step = 0; step != 100; ++step)
			{
				auto err = CompareOneStep(rb, 1.0f / 60.0f);
				PR_EXPECT(err < 1e-5f);
				Evolve(rb, 1.0f / 60.0f);
			}
		}

		// Box at origin (CoM == 0, asymmetric inertia)
		PRUnitTestMethod(Box_WithForce)
		{
			auto rb = RigidBody{};
			rb.SetMassProperties(Inertia::Box(v4{1, 2, 0.5f, 0}, 10.0f));
			rb.O2W(m4x4::Translation(v4{0, 0, 5, 1}));
			rb.VelocityWS(v4{0.5f, -0.3f, 0.2f, 0}, v4{1.0f, 0, -2.0f, 0});

			for (int step = 0; step != 100; ++step)
			{
				// Apply gravity
				rb.ApplyForceWS(v8force{v4::Zero(), v4{0, 0, -9.81f * 10.0f, 0}});
				auto err = CompareOneStep(rb, 1.0f / 60.0f);
				PR_EXPECT(err < 1e-4f);
				Evolve(rb, 1.0f / 60.0f);
			}
		}

		// Box with angular velocity and torque (CoM == 0)
		PRUnitTestMethod(Box_AngularMotion)
		{
			auto rb = RigidBody{};
			rb.SetMassProperties(Inertia::Box(v4{1, 1, 1, 0}, 5.0f));
			rb.O2W(m4x4::Identity());
			rb.VelocityWS(v4{2.0f, 1.0f, -0.5f, 0}, v4::Zero());

			for (int step = 0; step != 200; ++step)
			{
				// Apply a torque
				rb.ApplyForceWS(v8force{v4{0.5f, -0.3f, 0.1f, 0}, v4::Zero()});
				auto err = CompareOneStep(rb, 1.0f / 60.0f);
				PR_EXPECT(err < 1e-4f);
				Evolve(rb, 1.0f / 60.0f);
			}
		}

		// Sphere with offset CoM (CoM != 0, tests the spatial coupling terms)
		// This is the critical test case — polytopes typically have non-zero CoM.
		PRUnitTestMethod(Sphere_OffsetCoM_NoForce)
		{
			// Create a sphere with CoM offset from the model origin.
			// This exercises the full 6x6 spatial inverse inertia multiply.
			auto com = v4{0.3f, -0.2f, 0.1f, 0};
			auto rb = RigidBody{};
			rb.SetMassProperties(Inertia::Sphere(1.0f, 10.0f, com), com);
			rb.O2W(m4x4::Translation(v4{0, 0, 5, 1}));
			rb.VelocityWS(v4{0.5f, 0.3f, 0, 0}, v4{1.0f, 0, -2.0f, 0});

			for (int step = 0; step != 100; ++step)
			{
				auto err = CompareOneStep(rb, 1.0f / 60.0f);
				PR_EXPECT(err < 1e-4f);
				Evolve(rb, 1.0f / 60.0f);
			}
		}

		// Box with offset CoM under gravity (CoM != 0, forces + coupling)
		PRUnitTestMethod(Box_OffsetCoM_WithForce)
		{
			auto com = v4{0.5f, 0.0f, -0.3f, 0};
			auto rb = RigidBody{};
			rb.SetMassProperties(Inertia::Box(v4{1, 2, 0.5f, 0}, 10.0f, com), com);
			rb.O2W(m4x4::Translation(v4{0, 0, 5, 1}));
			rb.VelocityWS(v4{0.5f, -0.3f, 0.2f, 0}, v4{1.0f, 0, -2.0f, 0});

			for (int step = 0; step != 100; ++step)
			{
				// Apply gravity
				rb.ApplyForceWS(v8force{v4::Zero(), v4{0, 0, -9.81f * 10.0f, 0}});
				auto err = CompareOneStep(rb, 1.0f / 60.0f);
				PR_EXPECT(err < 1e-4f);
				Evolve(rb, 1.0f / 60.0f);
			}
		}

		// Box with offset CoM and torque (CoM != 0, angular-linear coupling)
		PRUnitTestMethod(Box_OffsetCoM_Torque)
		{
			auto com = v4{-0.2f, 0.4f, 0.1f, 0};
			auto rb = RigidBody{};
			rb.SetMassProperties(Inertia::Box(v4{1, 1, 1, 0}, 5.0f, com), com);
			rb.O2W(m4x4::Identity());
			rb.VelocityWS(v4{2.0f, 1.0f, -0.5f, 0}, v4::Zero());

			for (int step = 0; step != 200; ++step)
			{
				// Apply a torque — with offset CoM, this should also affect linear velocity
				rb.ApplyForceWS(v8force{v4{0.5f, -0.3f, 0.1f, 0}, v4::Zero()});
				auto err = CompareOneStep(rb, 1.0f / 60.0f);
				PR_EXPECT(err < 1e-4f);
				Evolve(rb, 1.0f / 60.0f);
			}
		}
	};
}
#endif
