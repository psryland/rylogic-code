//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2016
//*********************************************
#include "pr/physics/integrator/integrator.h"
#include "pr/physics/rigid_body/rigid_body.h"
#include "pr/physics/shape/inertia.h"
#include "src/collision/shape_cache.h"
#include "src/compute/physics_types.h"

namespace pr::physics
{
	namespace
	{
		constexpr int AngularDriftSubstepMax = 32;
		constexpr float AngularDriftMaxRadians = 0.25f;
		constexpr int AngularDriftIterationCount = 8;

		m3x3 RotateInertiaInvUnit(m3x3 const& os_iinv_unit, m3x3 const& rot)
		{
			auto b2a = InvertOrthonormal(rot);
			auto ws_iinv_unit = rot * os_iinv_unit * b2a;
			ws_iinv_unit.x.y = ws_iinv_unit.y.x = 0.5f * (ws_iinv_unit.x.y + ws_iinv_unit.y.x);
			ws_iinv_unit.x.z = ws_iinv_unit.z.x = 0.5f * (ws_iinv_unit.x.z + ws_iinv_unit.z.x);
			ws_iinv_unit.y.z = ws_iinv_unit.z.y = 0.5f * (ws_iinv_unit.y.z + ws_iinv_unit.z.y);
			return ws_iinv_unit;
		}
		m3x3 ScaleInertiaInv(m3x3 const& iinv_unit, float inv_mass)
		{
			return m3x3
			{
				iinv_unit.x * inv_mass,
				iinv_unit.y * inv_mass,
				iinv_unit.z * inv_mass,
			};
		}
		v4 AngularVelocityWS(m3x3 const& os_iinv_unit, m3x3 const& rot, float inv_mass, v4 const& momentum_ang)
		{
			auto ws_iinv_unit = RotateInertiaInvUnit(os_iinv_unit, rot);
			auto ws_iinv = ScaleInertiaInv(ws_iinv_unit, inv_mass);
			return ws_iinv * momentum_ang;
		}

		// Return the component of 'vec' selected by a principal-axis index.
		float Component(v4 const& vec, int axis)
		{
			switch (axis)
			{
			case 0: return vec.x;
			case 1: return vec.y;
			case 2: return vec.z;
				default: throw std::runtime_error("Invalid angular drift axis");
			}
		}

		// Build an axis-angle vector for a principal-axis rotation.
		v4 AxisAngle(int axis, float angle)
		{
			switch (axis)
			{
			case 0: return v4{angle, 0, 0, 0};
			case 1: return v4{0, angle, 0, 0};
			case 2: return v4{0, 0, angle, 0};
				default: throw std::runtime_error("Invalid angular drift axis");
			}
		}

		// Apply the exact flow for one diagonal inertia principal-axis Hamiltonian term.
		void SymplecticAxisDrift(m3x3& rot, v4& momentum_os, v4 const& inertia_inv_diagonal, float inv_mass, int axis, float elapsed_seconds)
		{
			auto omega = inv_mass * Component(inertia_inv_diagonal, axis) * Component(momentum_os, axis);
			auto axis_angle = AxisAngle(axis, omega * elapsed_seconds);
			auto axis_rot = m3x3::Rotation(axis_angle.xyz);
			rot = rot * axis_rot;
			momentum_os = InvertOrthonormal(axis_rot) * momentum_os;
		}

		// Integrate a torque-free diagonal-inertia angular drift using symmetric principal-axis splitting.
		void SymplecticAngularDrift(m3x3& rot, v4 const& momentum_ang, v4 const& inertia_inv_diagonal, float inv_mass, float elapsed_seconds, int angular_steps)
		{
			auto momentum_os = InvertOrthonormal(rot) * momentum_ang;
			auto angular_dt = elapsed_seconds / static_cast<float>(angular_steps);
			for (int angular_step = 0; angular_step != angular_steps; ++angular_step)
			{
				// Strang-split the free-rigid-body Hamiltonian into exact principal-axis flows. Each axis flow rotates the orientation in the body frame
				// and counter-rotates body-space angular momentum, preserving the fixed world angular momentum without solving an implicit midpoint.
				SymplecticAxisDrift(rot, momentum_os, inertia_inv_diagonal, inv_mass, 0, angular_dt * 0.5f);
				SymplecticAxisDrift(rot, momentum_os, inertia_inv_diagonal, inv_mass, 1, angular_dt * 0.5f);
				SymplecticAxisDrift(rot, momentum_os, inertia_inv_diagonal, inv_mass, 2, angular_dt);
				SymplecticAxisDrift(rot, momentum_os, inertia_inv_diagonal, inv_mass, 1, angular_dt * 0.5f);
				SymplecticAxisDrift(rot, momentum_os, inertia_inv_diagonal, inv_mass, 0, angular_dt * 0.5f);

				rot = Orthonorm(rot);
				if (angular_step + 1 != angular_steps)
					momentum_os = InvertOrthonormal(rot) * momentum_ang;
			}
		}
	}

	// Performs Störmer-Verlet kick-drift-kick on a GpuRigidBody.
	// This mirrors the GPU compute shader exactly, allowing A/B comparison for debugging.
	void Evolve(GpuRigidBody& dyn, float elapsed_seconds)
	{
		auto half_dt = elapsed_seconds * 0.5f;
		auto inv_mass = dyn.os_com_and_invmass.w;
		auto os_com = v4{dyn.os_com_and_invmass.x, dyn.os_com_and_invmass.y, dyn.os_com_and_invmass.z, 0};

		// ---- Step 1: Half-kick ----
		dyn.momentum_ang += dyn.force_ang * half_dt;
		dyn.momentum_lin += dyn.force_lin * half_dt;

		// ---- Step 2: Drift ----

		auto const isotropic_inertia =
			dyn.inertia_inv_products.x == 0.0f &&
			dyn.inertia_inv_products.y == 0.0f &&
			dyn.inertia_inv_products.z == 0.0f &&
			dyn.inertia_inv_diagonal.x == dyn.inertia_inv_diagonal.y &&
			dyn.inertia_inv_diagonal.x == dyn.inertia_inv_diagonal.z;
		auto const diagonal_inertia =
			dyn.inertia_inv_products.x == 0.0f &&
			dyn.inertia_inv_products.y == 0.0f &&
			dyn.inertia_inv_products.z == 0.0f;

		// Build the object-space unit inverse inertia 3x3 from compact storage.
		auto const& dia = dyn.inertia_inv_diagonal;
		auto const& off = dyn.inertia_inv_products;
		auto os_iinv_unit = m3x3
		{
			v4{dia.x, off.x, off.y, 0},
			v4{off.x, dia.y, off.z, 0},
			v4{off.y, off.z, dia.z, 0},
		};

		// Extract the current rotation and position from the transform
		auto rot = dyn.o2w.rot;
		auto pos = dyn.o2w.pos;

		// Compute velocity from momentum (block-diagonal — no coupling terms).
		// Since momentum/forces are about the CoM, the inverse inertia at the CoM
		// gives a simple decoupled relationship: omega = Ic_inv * h_ang, v = h_lin / m.
		auto vel_ang = v4{};
		if (isotropic_inertia)
		{
			vel_ang = (inv_mass * dyn.inertia_inv_diagonal.x) * dyn.momentum_ang;
		}
		else
		{
			vel_ang = AngularVelocityWS(os_iinv_unit, rot, inv_mass, dyn.momentum_ang);
		}
		auto vel_lin = inv_mass * dyn.momentum_lin;

		// CoM-based position update: translate CoM, derive model origin from new rotation.
		auto com_ws = rot * os_com;
		auto com_pos = pos + com_ws;

		auto new_rot = rot;
		auto angular_speed = Length(vel_ang.xyz);
		if (angular_speed != 0.0f)
		{
			if (isotropic_inertia)
			{
				auto dR = m3x3::Rotation(vel_ang.xyz * elapsed_seconds);
				new_rot = Orthonorm(dR * new_rot);
			}
			else if (diagonal_inertia)
			{
				auto angular_steps = std::clamp(static_cast<int>(std::ceil(angular_speed * elapsed_seconds / AngularDriftMaxRadians)), 1, AngularDriftSubstepMax);
				SymplecticAngularDrift(new_rot, dyn.momentum_ang, dyn.inertia_inv_diagonal, inv_mass, elapsed_seconds, angular_steps);
			}
			else
			{
				// Solve the midpoint orientation implicitly rather than using a one-shot predictor. The drift is torque-free, so world angular momentum
				// remains fixed while the orientation-dependent inertia determines the midpoint angular velocity.
				auto angular_steps = std::clamp(static_cast<int>(std::ceil(angular_speed * elapsed_seconds / AngularDriftMaxRadians)), 1, AngularDriftSubstepMax);
				auto angular_dt = elapsed_seconds / static_cast<float>(angular_steps);
				auto step_vel_ang = vel_ang.xyz;
				for (int angular_step = 0; angular_step != angular_steps; ++angular_step)
				{
					auto mid_vel_ang = step_vel_ang;
					auto mid_rot = m3x3::Rotation(mid_vel_ang * (angular_dt * 0.5f)) * new_rot;
					for (int iteration = 0; iteration != AngularDriftIterationCount; ++iteration)
					{
						mid_vel_ang = AngularVelocityWS(os_iinv_unit, mid_rot, inv_mass, dyn.momentum_ang).xyz;
						mid_rot = m3x3::Rotation(mid_vel_ang * (angular_dt * 0.5f)) * new_rot;
					}

					auto dR = m3x3::Rotation(mid_vel_ang * angular_dt);
					new_rot = dR * new_rot;
					if (angular_step + 1 != angular_steps)
						step_vel_ang = AngularVelocityWS(os_iinv_unit, new_rot, inv_mass, dyn.momentum_ang).xyz;
				}
				new_rot = Orthonorm(new_rot);
			}
		}
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
		ShapeCache shape_cache;
		auto shape_id = shape_cache.GetOrAdd(rb.Shape());
		GpuRigidBody dyn = PackDynamics(rb, shape_id);
		Evolve(dyn, elapsed_seconds);
		UnpackDynamics(dyn, rb);
	}

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
		auto mid_rot = m3x3::Rotation(ws_velocity.ang * (elapsed_seconds * 0.5f)) * rb.O2W().rot;
		auto mid_iinv_ws = Rotate(rb.InertiaInvOS(), mid_rot);
		auto ws_velocity_mid = mid_iinv_ws * ws_momentum;

		// Apply rotation using the midpoint angular velocity
		auto drot = ws_velocity_mid.ang * elapsed_seconds;
		auto new_rot = m3x3::Rotation(drot) * rb.O2W().rot;

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
		auto mid_rot = m3x3::Rotation(ws_velocity.ang * (elapsed_seconds * 0.5f)) * rb.O2W().rot;
		auto mid_iinv_ws = Rotate(rb.InertiaInvOS(), mid_rot);
		auto ws_velocity_mid = mid_iinv_ws * ws_momentum;

		// Compute CoM position before the step.
		// The body's O2W transform positions the model origin; CoM is offset from it.
		auto com_os = rb.CentreOfMassOS();
		auto com_ws = rb.O2W().pos + rb.O2W().rot * com_os;

		// Apply rotation using the midpoint angular velocity
		auto drot = ws_velocity_mid.ang * elapsed_seconds;
		auto new_rot = m3x3::Rotation(drot) * rb.O2W().rot;

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
