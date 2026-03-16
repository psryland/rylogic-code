//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2016
//*********************************************
#include "pr/physics-2/integrator/integrator.h"
#include "pr/physics-2/rigid_body/rigid_body.h"
#include "pr/physics-2/rigid_body/rigid_body_dynamics.h"
#include "pr/physics-2/shape/inertia.h"

namespace pr::physics
{
	// Performs Störmer-Verlet kick-drift-kick on a RigidBodyDynamics.
	// This mirrors the GPU compute shader exactly, allowing A/B comparison for debugging.
	void Evolve(RigidBodyDynamics& dyn, float elapsed_seconds)
	{
		auto half_dt = elapsed_seconds * 0.5f;
		auto inv_mass = dyn.os_com_and_invmass.w;
		auto os_com = v4{dyn.os_com_and_invmass.x, dyn.os_com_and_invmass.y, dyn.os_com_and_invmass.z, 0};

		// ---- Step 1: Half-kick ----
		dyn.momentum_ang += dyn.force_ang * half_dt;
		dyn.momentum_lin += dyn.force_lin * half_dt;

		// ---- Step 2: Drift ----

		// Build the object-space unit inverse inertia 3x3 from compact storage
		auto const& dia = dyn.inertia_inv_diagonal;
		auto const& off = dyn.inertia_inv_products;
		auto os_iinv_unit = m3x4
		{
			v4{dia.x, off.x, off.y, 0},
			v4{off.x, dia.y, off.z, 0},
			v4{off.y, off.z, dia.z, 0},
		};

		// Extract the current rotation and position from the transform
		auto rot = dyn.o2w.rot;
		auto pos = dyn.o2w.pos;

		// Rotate the inverse inertia from object space to world space: R * I * R^T
		auto b2a = InvertOrthonormal(rot);
		auto ws_iinv_unit = rot * os_iinv_unit * b2a;

		// Symmetrize to counteract float drift
		ws_iinv_unit.x.y = ws_iinv_unit.y.x = 0.5f * (ws_iinv_unit.x.y + ws_iinv_unit.y.x);
		ws_iinv_unit.x.z = ws_iinv_unit.z.x = 0.5f * (ws_iinv_unit.x.z + ws_iinv_unit.z.x);
		ws_iinv_unit.y.z = ws_iinv_unit.z.y = 0.5f * (ws_iinv_unit.y.z + ws_iinv_unit.z.y);

		// Mass-scaled world-space inverse inertia
		auto ws_iinv = m3x4
		{
			ws_iinv_unit.x * inv_mass,
			ws_iinv_unit.y * inv_mass,
			ws_iinv_unit.z * inv_mass,
		};

		// Compute velocity from momentum (block-diagonal — no coupling terms).
		// Since momentum/forces are about the CoM, the inverse inertia at the CoM
		// gives a simple decoupled relationship: omega = Ic_inv * h_ang, v = h_lin / m.
		auto vel_ang = ws_iinv * dyn.momentum_ang;
		auto vel_lin = inv_mass * dyn.momentum_lin;

		// Midpoint predictor for the rotation step: estimate the angular velocity at
		// the midpoint rotation to account for precession of anisotropic bodies.
		// This gives second-order accuracy instead of first-order.
		auto half_dR = m3x4::Rotation(vel_ang * (elapsed_seconds * 0.5f));
		auto mid_rot = half_dR * rot;
		auto mid_b2a = InvertOrthonormal(mid_rot);
		auto ws_iinv_mid = mid_rot * os_iinv_unit * mid_b2a;
		ws_iinv_mid.x.y = ws_iinv_mid.y.x = 0.5f * (ws_iinv_mid.x.y + ws_iinv_mid.y.x);
		ws_iinv_mid.x.z = ws_iinv_mid.z.x = 0.5f * (ws_iinv_mid.x.z + ws_iinv_mid.z.x);
		ws_iinv_mid.y.z = ws_iinv_mid.z.y = 0.5f * (ws_iinv_mid.y.z + ws_iinv_mid.z.y);
		auto ws_iinv_mid_scaled = m3x4{
			ws_iinv_mid.x * inv_mass,
			ws_iinv_mid.y * inv_mass,
			ws_iinv_mid.z * inv_mass,
		};
		auto vel_ang_mid = ws_iinv_mid_scaled * dyn.momentum_ang;

		// CoM-based position update: translate CoM, derive model origin from new rotation.
		auto com_ws = rot * os_com;
		auto com_pos = pos + com_ws;

		auto dR = m3x4::Rotation(vel_ang_mid * elapsed_seconds);
		auto new_rot = dR * rot;
		auto new_com_pos = com_pos + vel_lin * elapsed_seconds;
		auto new_pos = new_com_pos - new_rot * os_com;

		// Orthonormalize the rotation and write back to the transform
		dyn.o2w = Orthonorm(m4x4{new_rot, new_pos});

		// ---- Step 3: Half-kick ----
		dyn.momentum_ang += dyn.force_ang * half_dt;
		dyn.momentum_lin += dyn.force_lin * half_dt;

		// Zero forces
		dyn.force_ang = v4{};
		dyn.force_lin = v4{};
	}

	// Evolve the rigid body forward in time by 'elapsed_seconds' using Störmer-Verlet integration.
	void Evolve(RigidBody& rb, float elapsed_seconds)
	{
		RigidBodyDynamics dyn = PackDynamics(rb);
		Evolve(dyn, elapsed_seconds);
		UnpackDynamics(dyn, rb);
	}

	#if 0
	// Half-kick: advance momentum by half a timestep using the current force.
	// This is one half of the Störmer-Verlet kick. Called before drift and after
	// collision resolution, so that collisions see the half-kicked state.
	void EvolveKick(RigidBody& rb, float half_dt)
	{
		auto ws_force = rb.ForceWS();
		auto half_impulse = ws_force * half_dt;
		rb.MomentumWS(rb.MomentumWS() + half_impulse);
	}

	// Drift: update position and orientation using the current (half-kicked) momentum.
	// The velocity is computed at the CoM where the inertia is block-diagonal, then
	// the CoM is translated and the model origin derived from the new rotation.
	void EvolveDrift(RigidBody& rb, float elapsed_seconds)
	{
		// Compute velocity from momentum. Since inertia is stored at CoM with CoM()==0,
		// the multiply is decoupled: omega = Ic_inv * h_ang, v_com = h_lin / m.
		auto ws_momentum = rb.MomentumWS();
		auto ws_inertia_inv = rb.InertiaInvWS();
		auto ws_velocity = ws_inertia_inv * ws_momentum;

		// Current CoM position in world space
		auto com_os = rb.CentreOfMassOS();
		auto com_ws = rb.O2W().pos + rb.O2W().rot * com_os;

		// Midpoint predictor: estimate the angular velocity at the half-step rotation
		// to account for precession of anisotropic bodies (see Evolve() for details).
		auto mid_rot = m3x4::Rotation(ws_velocity.ang * (elapsed_seconds * 0.5f)) * rb.O2W().rot;
		auto mid_iinv_ws = Rotate(rb.InertiaInvOS(), mid_rot);
		auto ws_velocity_mid = mid_iinv_ws * ws_momentum;

		// Apply rotation using the midpoint angular velocity
		auto drot = ws_velocity_mid.ang * elapsed_seconds;
		auto new_rot = m3x4::Rotation(drot) * rb.O2W().rot;

		// Translate the CoM by the linear velocity, then derive the model origin
		// from the new rotation. This ensures the model origin orbits around the
		// CoM correctly when the body rotates.
		auto new_com_ws = com_ws + ws_velocity.lin * elapsed_seconds;
		auto new_pos = new_com_ws - new_rot * com_os;

		auto o2w = Orthonorm(m4x4{new_rot, new_pos});
		rb.O2W(o2w);

		#if PR_DBG
		{
			auto h = rb.MomentumWS();
			auto rb_o2w = rb.O2W();
			if (IsNaN(h.ang) || IsNaN(h.lin) || IsNaN(rb_o2w))
			{
				auto f = fopen("dump\\evolve_crash.log", "a");
				if (f) {
					fprintf(f, "[NaN] h_ang=(%.4f,%.4f,%.4f) h_lin=(%.4f,%.4f,%.4f)\n",
						h.ang.x, h.ang.y, h.ang.z, h.lin.x, h.lin.y, h.lin.z);
					fprintf(f, "  pos=(%.4f,%.4f,%.4f) com=(%.4f,%.4f,%.4f)\n",
						rb_o2w.pos.x, rb_o2w.pos.y, rb_o2w.pos.z,
						rb.CentreOfMassOS().x, rb.CentreOfMassOS().y, rb.CentreOfMassOS().z);
					fclose(f);
				}
			}
			assert("EvolveDrift: NaN in momentum" && !IsNaN(h.ang) && !IsNaN(h.lin));
			assert("EvolveDrift: NaN in transform" && !IsNaN(rb_o2w));
			assert("EvolveDrift: orientation not orthonormal" && IsOrthonormal(rb_o2w.rot));
		}
		#endif
	}

	// Combined kick-drift-kick for backward compatibility.
	// Equivalent to EvolveKick(dt/2) + EvolveDrift(dt) + EvolveKick(dt/2).
	void Evolve(RigidBody& rb, float elapsed_seconds)
	{
		auto ws_force = rb.ForceWS();
		auto half_impulse = ws_force * (elapsed_seconds * 0.5f);

		// Step 1: Half-kick — advance momentum by half-step
		rb.MomentumWS(rb.MomentumWS() + half_impulse);

		// Step 2: Drift — advance position/orientation using the half-kicked momentum.
		// The velocity is at the CoM (inertia is block-diagonal, no coupling terms).
		// omega = Ic_inv * h_ang, v_com = h_lin / m.
		auto ws_momentum = rb.MomentumWS();
		auto ws_inertia_inv = rb.InertiaInvWS();
		auto ws_velocity = ws_inertia_inv * ws_momentum;

		// Midpoint predictor for the rotation step:
		// For anisotropic bodies, the angular velocity changes during the drift step
		// because the world-space inertia tensor changes with orientation (precession).
		// Using the angular velocity at the start of the step is only first-order accurate.
		// By estimating the rotation at the midpoint and recomputing omega there, we get
		// second-order accuracy, significantly reducing secular energy drift for bodies
		// whose angular velocity changes at collisions (polytopes, non-face contacts).
		auto mid_rot = m3x4::Rotation(ws_velocity.ang * (elapsed_seconds * 0.5f)) * rb.O2W().rot;
		auto mid_iinv_ws = Rotate(rb.InertiaInvOS(), mid_rot);
		auto ws_velocity_mid = mid_iinv_ws * ws_momentum;

		// Compute CoM position before the step.
		// The body's O2W transform positions the model origin; CoM is offset from it.
		auto com_os = rb.CentreOfMassOS();
		auto com_ws = rb.O2W().pos + rb.O2W().rot * com_os;

		// Apply rotation using the midpoint angular velocity
		auto drot = ws_velocity_mid.ang * elapsed_seconds;
		auto new_rot = m3x4::Rotation(drot) * rb.O2W().rot;

		// Translate the CoM by the CoM velocity, then derive the model origin position
		// from the new rotation. This ensures the model origin orbits around the CoM
		// correctly when the body rotates.
		auto new_com_ws = com_ws + ws_velocity.lin * elapsed_seconds;
		auto new_pos = new_com_ws - new_rot * com_os;

		auto o2w = Orthonorm(m4x4{new_rot, new_pos});
		rb.O2W(o2w);

		// Step 3: Half-kick — advance momentum by second half-step
		rb.MomentumWS(rb.MomentumWS() + half_impulse);
		rb.ZeroForces();

		#if PR_DBG
		{
			// Sanity checks on the post-integration state:
			// 1. Verify no NaN crept in during integration
			auto h = rb.MomentumWS();
			auto rb_o2w = rb.O2W();
			if (IsNaN(h.ang) || IsNaN(h.lin) || IsNaN(rb_o2w))
			{
				auto f = fopen("dump\\evolve_crash.log", "a");
				if (f) {
					fprintf(f, "[NaN] h_ang=(%.4f,%.4f,%.4f) h_lin=(%.4f,%.4f,%.4f)\n",
						h.ang.x, h.ang.y, h.ang.z, h.lin.x, h.lin.y, h.lin.z);
					fprintf(f, "  pos=(%.4f,%.4f,%.4f) com=(%.4f,%.4f,%.4f)\n",
						rb_o2w.pos.x, rb_o2w.pos.y, rb_o2w.pos.z,
						rb.CentreOfMassOS().x, rb.CentreOfMassOS().y, rb.CentreOfMassOS().z);
					fclose(f);
				}
			}
			assert("Evolve: NaN in momentum" && !IsNaN(h.ang) && !IsNaN(h.lin));
			assert("Evolve: NaN in transform" && !IsNaN(rb_o2w));

			// 2. Verify the orientation is still orthonormal (Orthonorm shouldn't need
			//    to make large corrections — if it does, the angular velocity is too high
			//    for the timestep or there's an integration bug)
			auto rot = rb_o2w.rot;
			if (!IsOrthonormal(rot))
			{
				auto f = fopen("dump\\evolve_crash.log", "a");
				if (f) {
					fprintf(f, "[NOT_ORTHONORMAL] rot:\n");
					fprintf(f, "  x=(%.6f,%.6f,%.6f) y=(%.6f,%.6f,%.6f) z=(%.6f,%.6f,%.6f)\n",
						rot.x.x,rot.x.y,rot.x.z, rot.y.x,rot.y.y,rot.y.z, rot.z.x,rot.z.y,rot.z.z);
					fprintf(f, "  vel_ang=(%.4f,%.4f,%.4f) dt=%.6f\n",
						ws_velocity.ang.x, ws_velocity.ang.y, ws_velocity.ang.z, elapsed_seconds);
					fclose(f);
				}
			}
			assert("Evolve: orientation not orthonormal" && IsOrthonormal(rot));

			// 3. Verify the inertia inverse is still valid after rotation
			auto ws_iinv = rb.InertiaInvWS();
			if (!ws_iinv.Check())
			{
				auto f = fopen("dump\\evolve_crash.log", "a");
				if (f) {
					auto iinv = ws_iinv.Ic3x3(1);
					fprintf(f, "[BAD_INERTIA] InvMass=%.6f CoM=(%.4f,%.4f,%.4f)\n",
						ws_iinv.InvMass(), ws_iinv.CoM().x, ws_iinv.CoM().y, ws_iinv.CoM().z);
					fprintf(f, "  diag=(%.6f,%.6f,%.6f) off=(%.6f,%.6f,%.6f)\n",
						iinv.x.x, iinv.y.y, iinv.z.z, iinv.x.y, iinv.x.z, iinv.y.z);
					fclose(f);
				}
			}
			assert("Evolve: invalid inertia after rotation" && ws_iinv.Check());
		}
		#endif
	}

	#endif

	// Calculate the signed change in kinetic energy caused by applying 'force' for 'time_s'.
	float KineticEnergyChange(v8force force, v8force momentum0, InertiaInv const& inertia_inv, float time_s)
	{
		// Kinetic energy change:
		//    0.5 * (v1*I*v1 - v0*I*v0)
		//  = 0.5 * (v1.h1 - v0.h0)

		// Initial velocity
		auto velocity0 = inertia_inv * momentum0;

		// 'force' causes a change in momentum
		auto dmomentum = force * time_s;
		auto momentum1 = momentum0 + dmomentum;

		// Which corresponds to a change in velocity
		auto dvelocity = inertia_inv * dmomentum;
		auto velocity1 = velocity0 + dvelocity;

		// Kinetic energy
		auto ke = 0.5f * (Dot(velocity1, momentum1) - Dot(velocity0, momentum0));
		return ke;
	}
}
