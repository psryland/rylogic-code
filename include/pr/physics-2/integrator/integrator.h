//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2016
//*********************************************
#pragma once
#include "pr/physics-2/forward.h"

namespace pr::physics
{
	// Performs Störmer-Verlet kick-drift-kick on a RigidBody.
	// This mirrors the GPU compute shader exactly, allowing A/B comparison for debugging.
	void Evolve(RigidBodyDynamics& dyn, float elapsed_seconds);

	// Evolve the rigid body forward in time by 'elapsed_seconds' using Störmer-Verlet integration.
	void Evolve(RigidBody& rb, float elapsed_seconds);

	#if 0
	// Störmer-Verlet sub-steps for split integration.
	// The engine splits the kick-drift-kick so collision detection and resolution
	// occurs between the two half-kicks. This prevents phantom energy injection:
	// without the split, the second half-kick accelerates a body that has already
	// penetrated the ground, inflating its velocity before the collision reverses it.
	//
	// The integration pipeline in Engine::Step() is:
	//   1. EvolveKick(dt/2)         — first half-kick (advance momentum)
	//   2. EvolveDrift(dt)          — drift (update position/orientation)
	//   3. DetectAndResolve(dt)     — collision detection and impulse resolution
	//   4. EvolveKick(dt/2)         — second half-kick (advance momentum)
	//
	// With this ordering, elastic collisions that reverse the normal velocity
	// produce exact velocity reversal: v → -v. Without the split, the second
	// half-kick adds g·dt/2 extra velocity per bounce, accumulating ~m·v·g·dt
	// joules of phantom energy per collision.
	void EvolveKick(RigidBody& rb, float half_dt);
	void EvolveDrift(RigidBody& rb, float elapsed_seconds);

	// Combined kick-drift-kick for backward compatibility and testing.
	// Equivalent to EvolveKick(dt/2) + EvolveDrift(dt) + EvolveKick(dt/2).
	void Evolve(RigidBody& rb, float elapsed_seconds);

	// CPU fallback for GPU integration path.
	// Performs the same Störmer-Verlet kick-drift-kick on a RigidBodyDynamics buffer entry.
	// This mirrors the GPU compute shader exactly, allowing A/B comparison for debugging.
	void EvolveCPU(RigidBodyDynamics& dyn, float elapsed_seconds);
	#endif

	// Calculate the signed change in kinetic energy caused by applying 'force' for 'time_s'.
	// Assumes constant inertia over the timestep. Exact for symmetric bodies (sphere, etc.) at
	// the model origin. For the general case, KE change ≈ Dot(v_mid, f) * dt (power formula).
	float KineticEnergyChange(v8force force, v8force momentum0, InertiaInv const& inertia_inv, float time_s);
}
