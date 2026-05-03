//************************************
// Physics Engine
//  Copyright (c) Rylogic Ltd 2016
//************************************

#if PR_UNITTESTS
#include "pr/common/unittests.h"
#include "pr/physics/physics.h"
#include "src/compute/physics_types.h"

namespace pr::physics::tests
{
	void ForceLink_RigidBody() {}

	PRUnitTestClass(RigidBodyTests)
	{
		PRUnitTestMethod(SimpleCase)
		{
			auto mass = 5.0f;
			auto rb = RigidBody{};
			rb.SetMassProperties(Inertia::Sphere(1, mass), v4{});

			// Apply a force and torque. The force at (0,1,0) cancels out the torque
			rb.ApplyForceWS(v4{1,0,0,0}, v4{0,0,1,0}, v4{0,1,0,0});

			// Check force applied
			auto ws_force = rb.ForceWS();
			auto os_force = rb.ForceOS();
			PR_EXPECT(FEql(ws_force, v8force{0,0,0, 1,0,0}));
			PR_EXPECT(FEql(os_force, v8force{0,0,0, 1,0,0}));

			// Integrate for 1 sec
			Evolve(rb, 1.0f);

			// Check position
			// Distance travelled: S = So + Vot + 0.5At²; So = 0, Vo = 0, t = 1, A = F/m, F = 1  =>  S = 0.5/mass
			auto o2w = rb.O2W();
			PR_EXPECT(FEql(o2w.rot, m3x3::Identity()));
			PR_EXPECT(FEql(o2w.pos, v4{0.5f / mass,0,0,1}));

			// Check the momentum
			auto ws_mom = rb.MomentumWS();
			auto os_mom = rb.MomentumOS();
			PR_EXPECT(FEql(ws_mom, v8force{0,0,0, 1,0,0}));
			PR_EXPECT(FEql(os_mom, v8force{0,0,0, 1,0,0}));

			// Check the velocity
			// Velocity: V = Vo + At; Vo = 0, t = 1, A = F/m, F = 1  =>  V = 1/mass
			auto ws_vel = rb.VelocityWS();
			auto os_vel = rb.VelocityOS();
			PR_EXPECT(FEql(ws_vel, v8motion{0,0,0, 1/mass,0,0}));
			PR_EXPECT(FEql(os_vel, v8motion{0,0,0, 1/mass,0,0}));
		}
		PRUnitTestMethod(SimpleCaseWithRotation)
		{
			auto mass = 5.0f;
			auto rb = RigidBody{};
			rb.SetMassProperties(Inertia::Sphere(1, mass), v4{});

			// Apply a force and torque. The force at (0,-1,0) doubles the torque
			rb.ApplyForceWS(v4{1,0,0,0}, v4{0,0,1,0}, v4{0,-1,0,0});

			// Check force applied
			auto ws_force = rb.ForceWS();
			auto os_force = rb.ForceOS();
			PR_EXPECT(FEql(ws_force, v8force{0,0,2, 1,0,0}));
			PR_EXPECT(FEql(os_force, v8force{0,0,2, 1,0,0}));

			// Integrate for 1 sec
			Evolve(rb, 1.0f);

			// Check position
			// Distance: S = So + Vot + 0.5At²; So = 0, Vo = 0, t = 1, A = F/m, F = 1  =>  S = 0.5/mass
			// Rotation: O = Oo + Wot + 0.5At²; Oo = 0, Wo = 0, t = 1, A = I^T, T = 2  =>  O = 0.5*I^(0,0,2)
			auto o2w = rb.O2W();
			auto pos = v4{0.5f / mass,0,0,1};
			auto rot = m3x3::Rotation(0.5f * (rb.InertiaInvWS() * v3{0,0,2}));
			auto invrot = InvertOrthonormal(rot);
			PR_EXPECT(FEql(o2w.pos, pos));
			PR_EXPECT(FEql(o2w.rot, rot));

			// Check the momentum
			auto ws_mom = rb.MomentumWS();
			auto os_mom = rb.MomentumOS();
			auto WS_MOM = v8force{0,0,2, 1,0,0};
			auto OS_MOM = invrot * WS_MOM;
			PR_EXPECT(FEql(ws_mom, WS_MOM));
			PR_EXPECT(FEql(os_mom, OS_MOM));

			// Check the velocity
			// Velocity: V = Vo + At; Vo = 0, t = 1, A = F/m, F = 1  =>  V = 1/mass
			// Rotation: W = Wo + At; Wo = 0, t = 1, A = I^T, T = 2  =>  W = I^(0,0,2)
			auto ws_vel = rb.VelocityWS();
			auto os_vel = rb.VelocityOS();
			auto WS_VEL = v8motion{(rb.InertiaInvWS() * v4{0,0,2,0}), v4{1/mass,0,0,0}};
			auto OS_VEL = invrot * WS_VEL;
			PR_EXPECT(FEql(ws_vel, WS_VEL));
			PR_EXPECT(FEql(os_vel, OS_VEL));
		}
		PRUnitTestMethod(OffCentreCoM)
		{
			auto mass = 5.0f;
			auto rb = RigidBody{};
			auto model_to_com = v4{0,1,0,0};
			rb.SetMassProperties(Inertia::Sphere(1, mass, model_to_com), model_to_com);

			// InertiaOS() returns the inertia about the CoM (block-diagonal storage —
			// see RigidBody::SetMassProperties). For a sphere of radius 1, this is
			// (2/5)*I regardless of the model_to_com offset.
			PR_EXPECT(FEql(rb.InertiaOS().To3x3(1), m3x3::Scale(0.4f, 0.4f, 0.4f)));

			// Translating the CoM-frame inertia to the model origin via the parallel-axis
			// theorem should give diag(1+0.4, 0.4, 1+0.4) for offset (0,1,0).
			auto inertia_at_origin = Translate(rb.InertiaOS(), model_to_com, ETranslateInertia::AwayFromCoM);
			PR_EXPECT(FEql(inertia_at_origin.To3x3(1), m3x3::Scale(1.4f, 0.4f, 1.4f)));

			// Apply a force at the CoM (no torque about CoM).
			rb.ApplyForceWS(v4{1,0,0,0}, v4{}, rb.CentreOfMassWS());

			// Check force applied.
			// Spatial force measured at the CoM. Since ws_at == CoM, zero moment arm — no torque.
			auto ws_force = rb.ForceWS();
			auto os_force = rb.ForceOS();
			PR_EXPECT(FEql(ws_force, v8force{0,0,0, 1,0,0}));
			PR_EXPECT(FEql(os_force, v8force{0,0,0, 1,0,0}));

			// Integrate for 1 sec
			Evolve(rb, 1.0f);

			// Check position
			// A force through the CoM produces pure translation — no rotation.
			auto o2w = rb.O2W();
			PR_EXPECT(FEql(o2w.rot, m3x3::Identity()));
			PR_EXPECT(FEql(o2w.pos, v4{0.5f / mass,0,0,1}));

			// Check the momentum (at CoM — no angular component)
			auto ws_mom = rb.MomentumWS();
			auto os_mom = rb.MomentumOS();
			PR_EXPECT(FEql(ws_mom, v8force{0,0,0, 1,0,0}));
			PR_EXPECT(FEql(os_mom, v8force{0,0,0, 1,0,0}));

			// Check the velocity
			// Zero angular velocity because force through CoM produces no torque.
			auto ws_vel = rb.VelocityWS();
			auto os_vel = rb.VelocityOS();
			PR_EXPECT(FEql(ws_vel, v8motion{0,0,0, 1/mass,0,0}));
			PR_EXPECT(FEql(os_vel, v8motion{0,0,0, 1/mass,0,0}));
		}
		PRUnitTestMethod(OffCentreCoMWithRotation)
		{
			auto mass = 5.0f;
			auto rb = RigidBody{};
			auto model_to_com = v4{0,1,0,0};
			rb.SetMassProperties(Inertia::Sphere(1, mass, model_to_com), model_to_com);

			// Apply a force and torque at the model origin (ws_at defaults to 0).
			rb.ApplyForceWS(v4{1,0,0,0}, v4{0,0,1,0});

			// Check force applied.
			// Spatial force shifted from model origin (ws_at = 0) to CoM (offset = (0,1,0)).
			// Shift adds Cross(force, offset) = Cross((1,0,0),(0,1,0)) = (0,0,1) to the torque.
			auto ws_force = rb.ForceWS();
			auto os_force = rb.ForceOS();
			PR_EXPECT(FEql(ws_force, v8force{0,0,2, 1,0,0}));
			PR_EXPECT(FEql(os_force, v8force{0,0,2, 1,0,0}));

			// Predict the Evolve result by replicating the integration logic.
			// Inertia is stored at the CoM (block-diagonal), so linear and angular are
			// decoupled in the multiply: v_lin = h_lin/m, omega = Ic_inv * h_ang.
			// The CoM travels in a straight line under linear momentum; the model origin
			// orbits the CoM as the body rotates.
			auto com_os = rb.CentreOfMassOS();
			auto com_ws_init = rb.O2W().pos + rb.O2W().rot * com_os;
			auto ws_iinv = rb.InertiaInvWS();
			auto ws_mom_mid = ws_force * 0.5f;

			// One refinement iteration (matching Evolve's midpoint predictor for rotation)
			auto ws_vel_est = ws_iinv * ws_mom_mid;
			auto do2w = m3x3::Rotation(ws_vel_est.ang.xyz * 0.5f);
			ws_iinv = Rotate(ws_iinv, do2w);

			auto ws_vel = ws_iinv * ws_mom_mid;
			auto rot = m3x3::Rotation(ws_vel.ang.xyz * 1.0f) * rb.O2W().rot;

			// CoM advances linearly; model origin = new_CoM - new_rot * com_os
			auto new_com_ws = com_ws_init + ws_vel.lin * 1.0f;
			auto pos = (new_com_ws - rot * com_os).w1();
			auto invrot = InvertOrthonormal(rot);

			// Integrate for 1 sec
			Evolve(rb, 1.0f);

			// Check position
			auto o2w = rb.O2W();
			PR_EXPECT(FEql(o2w.pos, pos));
			PR_EXPECT(FEqlRelative(o2w.rot, rot, 0.01f));

			// Check the momentum
			auto WS_MOM = ws_force * 1.0f;
			auto ws_mom = rb.MomentumWS();
			auto os_mom = rb.MomentumOS();
			auto OS_MOM = invrot * WS_MOM;
			PR_EXPECT(FEql(ws_mom, WS_MOM));
			PR_EXPECT(FEql(os_mom, OS_MOM));

			// Check the velocity
			auto ws_vel_final = rb.VelocityWS();
			auto os_vel_final = rb.VelocityOS();
			auto WS_VEL = rb.InertiaInvWS() * WS_MOM;
			auto OS_VEL = rb.InertiaInvOS() * os_mom;
			PR_EXPECT(FEqlRelative(ws_vel_final, WS_VEL, 0.01f));
			PR_EXPECT(FEqlRelative(os_vel_final, OS_VEL, 0.01f));
		}
		PRUnitTestMethod(OffCentreCoMWithComplexRotation)
		{
			auto mass = 5.0f;
			auto rb = RigidBody{};
			auto model_to_com = v4{0,1,0,0};
			rb.SetMassProperties(Inertia::Sphere(1, mass, model_to_com), model_to_com);

			// Apply forces and torques at various points.
			// Forces are shifted from ws_at to CoM (ws_com = (0,1,0)).
			rb.ApplyForceWS(v4{1,0,0,0}, v4{0,-1,0,0}, v4{0,1,1,0}); // +X push at (0,1,1) + -Y twist
			rb.ApplyForceWS(v4{0,-1,0,0}, v4{0,-1,0,0}, v4{1,1,0,0}); // -Y push at (1,1,0) + -Y twist

			// Check force applied (spatial force at CoM)
			auto ws_force = rb.ForceWS();
			auto os_force = rb.ForceOS();

			// Force 1: v8force{(0,-1,0), (1,0,0)} shifted by (ws_com - ws_at) = (0,0,-1)
			//   ang += Cross((1,0,0),(0,0,-1)) = (0,1,0)
			//   total: (0,-1+1,0) = (0,0,0), (1,0,0)
			// Force 2: v8force{(0,-1,0), (0,-1,0)} shifted by (ws_com - ws_at) = (-1,0,0)
			//   ang += Cross((0,-1,0),(-1,0,0)) = (0,0,-1)
			//   total: (0,-1+0,-1) = (0,-1,-1), (0,-1,0)
			// Combined: (0+0, 0-1, 0-1, 1+0, 0-1, 0+0) = (0,-1,-1, 1,-1,0)
			PR_EXPECT(FEql(ws_force, v8force{0,-1,-1, 1,-1,0}));
			PR_EXPECT(FEql(os_force, v8force{0,-1,-1, 1,-1,0}));

			// Predict Evolve result using the integration logic. The CoM travels linearly
			// while the model origin orbits the CoM as the body rotates.
			auto com_os = rb.CentreOfMassOS();
			auto com_ws_init = rb.O2W().pos + rb.O2W().rot * com_os;
			auto ws_iinv = rb.InertiaInvWS();
			auto ws_mom_mid = ws_force * 0.5f;

			auto ws_vel_est = ws_iinv * ws_mom_mid;
			auto do2w = m3x3::Rotation(ws_vel_est.ang.xyz * 0.5f);
			ws_iinv = Rotate(ws_iinv, do2w);

			auto ws_vel = ws_iinv * ws_mom_mid;
			auto rot = m3x3::Rotation(ws_vel.ang.xyz * 1.0f) * rb.O2W().rot;

			auto new_com_ws = com_ws_init + ws_vel.lin * 1.0f;
			auto pos = (new_com_ws - rot * com_os).w1();

			// Integrate for 1 sec
			Evolve(rb, 1.0f);

			// Check position
			auto o2w = rb.O2W();
			PR_EXPECT(FEql(o2w.pos, pos));
			PR_EXPECT(FEqlRelative(o2w.rot, rot, 0.01f));
		}
		PRUnitTestMethod(AsymmetricPolytopeRotatesAboutCentreOfMass)
		{
			constexpr v4 verts[] = {
				v4{-0.8f, -0.8f, -0.5f, 1},
				v4{+0.8f, -0.8f, -0.5f, 1},
				v4{+0.0f, +0.1f, -0.1f, 1},
				v4{+0.0f, +0.0f, +0.8f, 1},
			};
			auto true_com = (0.25f * (verts[0] + verts[1] + verts[2] + verts[3])).w0();
			auto buf = collision::BuildPolytopeFromPoints(verts);
			auto& polytope = buf.as<collision::ShapePolytope>();

			// BuildPolytopeFromPoints keeps the local vertices centred and stores the true object-space offset in m_s2r.
			PR_EXPECT(FEqlAbsolute(collision::CalcCentreOfMass(polytope), v4{}, 1e-5f));
			PR_EXPECT(FEqlAbsolute(polytope.m_base.m_s2r.pos.w0(), true_com, 1e-5f));

			auto mp = CalcMassProperties(polytope, 1.0f);
			PR_EXPECT(FEqlAbsolute(mp.m_centre_of_mass, true_com, 1e-5f));

			auto rb = RigidBody{};
			rb.O2W(m4x4::Translation(v4{0, 0, 2.5f, 1}));
			rb.Shape(&polytope.m_base, 10.0f);
			PR_EXPECT(FEqlAbsolute(rb.CentreOfMassOS(), true_com, 1e-5f));
			auto bbox_os = rb.Shape().m_s2r * rb.Shape().m_bbox;
			auto bbox_ws = rb.BBoxWS();
			auto dyn = PackDynamics(rb, 0);
			for (auto const& vert : verts)
			{
				PR_EXPECT(IsWithin(bbox_os, vert, 1e-5f));
				PR_EXPECT(IsWithin(bbox_ws, rb.O2W() * vert, 1e-5f));
				PR_EXPECT(IsWithin(dyn.os_bbox, vert, 1e-5f));
			}

			auto ang_vel = v4{-11.72f, -13.07f, +0.65f, 0};
			auto lin_vel = v4{0, 0, +5.60f, 0};
			rb.VelocityWS(v8motion{ang_vel, lin_vel});

			auto dt = 1.0f / 60.0f;
			auto com_ws0 = rb.O2W().pos + rb.O2W().rot * true_com;
			Evolve(rb, dt);

			auto com_ws1 = rb.O2W().pos + rb.O2W().rot * true_com;
			auto expected_com_ws1 = com_ws0 + lin_vel * dt;
			PR_EXPECT(FEqlAbsolute(com_ws1, expected_com_ws1, 1e-4f));
			PR_EXPECT(FEqlAbsolute(rb.CentreOfMassWS(), rb.O2W().rot * true_com, 1e-4f));
		}
		PRUnitTestMethod(Extrapolation)
		{
			auto mass = 5.0f;
			auto rb = RigidBody{};
			rb.SetMassProperties(Inertia::Sphere(1, mass), v4{});

			auto vel = v8motion{0,0,1, 0,1,0};
			rb.VelocityWS(vel);

			auto o2w0 = rb.O2W();
			auto O2W0 = m4x4::Identity();
			PR_EXPECT(FEql(o2w0, O2W0));

			auto o2w1 = rb.O2W(1.0f);
			auto O2W1 = m4x4::Transform(1*vel.ang, (1*vel.lin).w1());
			PR_EXPECT(FEql(o2w1, O2W1));

			auto o2w2 = rb.O2W(2.0f);
			auto O2W2 = m4x4::Transform(2*vel.ang, (2*vel.lin).w1());
			PR_EXPECT(FEql(o2w2, O2W2));

			auto o2w3 = rb.O2W(-2.0f);
			auto O2W3 = m4x4::Transform(-2*vel.ang, (-2*vel.lin).w1());
			PR_EXPECT(FEql(o2w3, O2W3));
		}
		PRUnitTestMethod(KineticEnergy)
		{
			auto mass = 5.0f;
			std::default_random_engine rng;

			// KE should be the same no matter what frame it's measured in
			auto rb = RigidBody{};
			rb.SetMassProperties(Inertia::Sphere(1, mass), v4{});
			rb.MomentumWS(v8force{0,0,1, 0,1,0});
			rb.O2W(m4x4::Random(rng, v4::Origin(), 5.0f));

			auto ws_ke = 0.5f * Dot(rb.VelocityWS(), rb.MomentumWS());
			auto os_ke = 0.5f * Dot(rb.VelocityOS(), rb.MomentumOS());
			PR_EXPECT(FEql(ws_ke, os_ke));
		}
		PRUnitTestMethod(VelocityOS_ShiftsToCoM)
		{
			// Verify that VelocityOS(ang, lin, os_at) shifts the velocity from os_at to the CoM.
			auto mass = 5.0f;
			auto rb = RigidBody{};
			rb.SetMassProperties(Inertia::Sphere(1, mass), v4{});

			// Set velocity at an offset point. With angular velocity present,
			// the shift to origin changes the linear component.
			auto os_ang = v4{0, 0, 1, 0};
			auto os_lin = v4{1, 0, 0, 0};
			auto os_at = v4{0, 1, 0, 0};
			rb.VelocityOS(os_ang, os_lin, os_at);

			// Shift from os_at to CoM (CoM = origin here): ofs = ws_com - ws_at = -os_at = (0,-1,0)
			// Shift(v8motion{ang, lin}, ofs) = {ang, lin + Cross(ang, ofs)}
			// Cross((0,0,1), (0,-1,0)) = (1, 0, 0)
			// shifted_lin = (1,0,0) + (1,0,0) = (2,0,0)
			auto ws_vel = rb.VelocityWS();
			PR_EXPECT(FEql(ws_vel, v8motion{0, 0, 1, 2, 0, 0}));
		}
		PRUnitTestMethod(DzhanibekovEffect)
		{
			// The Dzhanibekov effect (intermediate axis theorem / tennis racket theorem):
			// Rotation about the intermediate principal axis of inertia is unstable.
			// A small perturbation causes the body to periodically flip 180°.
			//
			// Setup: A box with three distinct principal moments Iz < Iy < Ix,
			// spinning about the intermediate axis (y), with a small perturbation.
			// For half-extents (1, 2, 4):
			//   Ix = (4+16)/3 ≈ 6.67, Iy = (1+16)/3 ≈ 5.67, Iz = (1+4)/3 ≈ 1.67
			// The instability growth rate σ = ω₀√((Iy-Iz)(Ix-Iy)/(Iz·Ix)) ≈ 0.6·ω₀

			auto mass = 1.0f;
			auto rb = RigidBody{};
			rb.SetMassProperties(Inertia::Box(v4{1, 2, 4, 0}, mass), v4{});

			// Initial angular velocity: mainly about the intermediate y-axis,
			// with a 10% perturbation to seed the instability.
			auto omega0 = 10.0f;
			auto perturbation = 0.1f * omega0;
			rb.VelocityWS(v8motion{perturbation, omega0, perturbation, 0, 0, 0});

			// Record initial conserved quantities
			auto h0 = rb.MomentumWS();
			auto ke0 = rb.KineticEnergy();

			// Simulate with small timesteps, no external forces.
			// With σ ≈ 6 rad/s and 10% perturbation, flips occur every ~0.8s.
			auto dt = 0.001f;
			auto total_time = 3.0f;
			auto steps = static_cast<int>(total_time / dt);

			auto flip_count = 0;
			auto prev_omega_y = rb.VelocityOS().ang.y;

			for (int i = 0; i < steps; ++i)
			{
				Evolve(rb, dt);

				// Get angular velocity in the body frame
				auto os_omega_y = rb.VelocityOS().ang.y;

				// Check for sign change of intermediate axis component (a "flip")
				if (prev_omega_y * os_omega_y < 0)
					++flip_count;

				prev_omega_y = os_omega_y;
			}

			// The Dzhanibekov effect: multiple flips should occur.
			PR_EXPECT(flip_count >= 2);

			// Angular momentum is exactly conserved (no forces → h_new = h_old each step)
			auto h_final = rb.MomentumWS();
			PR_EXPECT(FEql(h0, h_final));

			// Kinetic energy should be approximately conserved (drift from discrete rotation updates)
			auto ke_final = rb.KineticEnergy();
			PR_EXPECT(FEqlRelative(ke0, ke_final, 0.01f));
		}
	};
}
#endif
