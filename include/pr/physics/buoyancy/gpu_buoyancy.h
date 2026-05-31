//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#pragma once
#include "pr/physics/forward.h"
#include "pr/physics/integrator/engine.h"
#include "pr/physics/buoyancy/buoyancy_primitives.h"

namespace pr::physics
{
	// GPU buoyancy module that applies sampled-composite forces and records diagnostics through Engine::ExternalForces.
	struct GpuBuoyancy
	{
		static constexpr int MaxWaterWaveCount = 64;

		struct SineWave
		{
			v2 m_direction = v2::XAxis();
			float m_wavelength = 1.0f;
			float m_amplitude = 0.0f;
			float m_phase_speed = 0.0f;

			// Return a copy with the wave direction normalised.
			SineWave Normalised() const;
		};
		struct WaterSurface
		{
			float m_level = 0.0f;
			std::vector<SineWave> m_waves;

			// Return a copy with validated and normalised waves.
			WaterSurface Normalised() const;

			// Return true when the water height is spatially constant.
			bool IsFlat() const;

			// Evaluate the water height above the world-space XY position at a simulation time.
			float EvaluateHeight(v2 xy_ws, float time_s) const;

			// Evaluate the XY surface gradient (dh/dx, dh/dy) of the water height at a simulation time.
			// The gradient is the same one the GPU buoyancy pass uses to compute the lateral
			// component of the hydrostatic force, so callers (e.g. the sandbox water mesh)
			// can shade or visualise the surface consistently with the physics integration.
			v2 EvaluateGradient(v2 xy_ws, float time_s) const;

			// Evaluate the world-space water particle velocity (orbital flow) at a world-space
			// position and simulation time. Uses the linear deep-water (Airy) orbital-velocity
			// field consistent with the sine-wave height model: particles move in circular orbits
			// whose radius decays exponentially with depth below the still-water level. The drag
			// pass subtracts this from the body velocity to obtain the relative flow at each wetted
			// surface sample. The returned vector has w = 0.
			v4 EvaluateVelocity(v4 pos_ws, float time_s) const;
		};

		// Tunable parameters that govern how the GPU buoyancy pass converts a water surface and
		// a submerged hull into forces. Defaults match fresh water with mild viscous damping.
		struct Config
		{
			// Fluid density (kg/m^3). The static buoyancy force per unit submerged volume is
			// |gravity| * density. The default matches the analytic constant used by the
			// flat-water diagnostic comparison so unconfigured callers get matching values.
			float m_fluid_density = 1000.0f;

			// Linear viscous drag time-constant (seconds). The drag force per column is
			// -c_drag * V_submerged_col * v_body(centroid), where c_drag = density / tau_damp.
			// Tau is the e-folding time for a body whose density matches the fluid; lighter
			// bodies decay faster, heavier bodies slower. A small positive value adds stability
			// to wave-driven motion without dominating low-frequency dynamics. Set to <= 0 to
			// disable drag entirely (useful for purely-conservative validation cases).
			float m_drag_time_constant_s = 3.0f;

			// Quadratic (form) drag coefficient (dimensionless). Per-face drag is
			//   F_face = -0.5 * fluid_density * Cd * A_sub * max(0, v_n)^2 * n_ws
			// summed over a 2x2 sub-sample grid per face, where v_n is the outward-normal
			// component of the body's velocity at each sub-sample. Linear drag (above) and
			// quadratic drag operate independently: linear dominates near rest and stabilises
			// low-frequency motion, quadratic dominates at speed and provides realistic form
			// drag for translating/tumbling bodies. Typical values: 1.05 for a cube (default),
			// 0.47 for a sphere, 1.28 for a flat plate. Set to <= 0 to disable form drag.
			float m_quadratic_drag_coefficient = 1.05f;
		};

		struct BodyState
		{
			m4x4 m_o2w = m4x4::Identity();
			v4 m_centre_of_mass_os = v4::Zero();
			// World-space gravity vector for this body. The buoyancy pass uses the body's own
			// gravity sample (not a single global value) so each body responds to its local
			// gravity field. The host-side flat-water analytic comparison uses this same value
			// so reported errors reflect numerical/quantisation differences, not a gravity mismatch.
			v4 m_ws_gravity = v4::Zero();
			bool m_valid = false;
		};

		using StepIndexResolver = std::function<int(int stable_body_index)>;
		using BodyStateResolver = std::function<BodyState(int stable_body_index)>;

		struct Diagnostics
		{
			int m_body_index;
			int m_body_generation;
			float m_volume_m3;
			v4 m_force_ws;
			v4 m_centre_buoyancy_ws;
			v4 m_torque_ws;
			float m_analytic_volume_m3;
			v4 m_analytic_force_ws;
			v4 m_analytic_centre_buoyancy_ws;
			v4 m_analytic_torque_ws;
			float m_volume_error_m3;
			v4 m_force_error_ws;
			v4 m_centre_buoyancy_error_ws;
			v4 m_torque_error_ws;
			bool m_valid;
			bool m_analytic_valid;

			// Construct an invalid buoyancy diagnostic record.
			Diagnostics();
		};

		struct Registration
		{
			friend struct GpuBuoyancy;

		private:

			GpuBuoyancy* m_owner;
			int m_body_index;
			int m_body_generation;

			Registration(GpuBuoyancy& owner, int body_index, int body_generation);

		public:

			// Construct an empty registration handle.
			Registration();
			Registration(Registration const&) = delete;
			Registration& operator=(Registration const&) = delete;

			// Move ownership of a buoyancy hull registration.
			Registration(Registration&& rhs) noexcept;

			// Replace this handle with another registration, unregistering the current hull first.
			Registration& operator=(Registration&& rhs) noexcept;

			// Unregister the hull owned by this handle.
			~Registration();

			// Release the current hull registration, if any.
			void Reset() noexcept;

			// Return true when this handle owns a registered hull.
			explicit operator bool() const;
		};

	private:

		struct Impl;
		std::unique_ptr<Impl> m_impl;

		// Remove the buoyancy hull for a stable physics body index.
		void UnregisterHull(int body_index, int body_generation) noexcept;

	public:

		// Construct and subscribe the buoyancy pass to a physics engine. 'config' provides the
		// tunable fluid parameters (density, drag time constant) used for subsequent dispatches.
		// SetConfig may be called later to update these at runtime.
		GpuBuoyancy(ID3D12Device* device, Engine& engine, Config const& config, StepIndexResolver step_index_resolver, BodyStateResolver body_state_resolver);
		GpuBuoyancy(GpuBuoyancy const&) = delete;
		GpuBuoyancy& operator=(GpuBuoyancy const&) = delete;

		// Destroy the buoyancy module after all owned registrations have been released.
		~GpuBuoyancy();

		// Return the latest diagnostic record for a registered hull.
		Diagnostics LatestDiagnostics(int body_index, int body_generation) const;

		// Consume diagnostic readback data after the physics engine has completed its GPU step.
		void CompleteStep();

		// Set the water surface used by subsequent buoyancy force dispatches.
		void SetWaterSurface(WaterSurface const& water_surface);

		// Return the current water surface used by buoyancy force dispatches.
		WaterSurface const& GetWaterSurface() const;

		// Set the tunable buoyancy parameters used by subsequent dispatches.
		// The change takes effect on the next call to Engine::ExternalForces.
		void SetConfig(Config const& config);

		// Return the tunable buoyancy parameters currently in effect.
		Config const& GetConfig() const;

		// Register a composite convex-primitive buoyancy hull against a stable physics body index.
		// 'shape' is a collision::Shape that is either a single convex primitive (Box / Sphere /
		// Triangle / Polytope) or a ShapeArray of such primitives; it is flattened and copied into an
		// owned immutable descriptor at registration time, so the caller's shape may be modified or
		// destroyed afterwards. While the registration is alive, 'body' is marked NeverSleep so the
		// engine continues to call Engine::ExternalForces (and therefore this buoyancy pass) every
		// step regardless of the body's kinetic state. The original NeverSleep flag is restored on
		// unregister.
		Registration RegisterCompositeHull(RigidBody& body, int body_index, int body_generation, collision::Shape const& shape);
	};
}
