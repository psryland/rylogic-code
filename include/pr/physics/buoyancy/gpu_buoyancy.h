//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#pragma once
#include "pr/physics/forward.h"
#include "pr/physics/integrator/engine.h"
#include "pr/physics/atmosphere/air.h"
#include "pr/physics/buoyancy/buoyancy_primitives.h"
#include "pr/physics/surface/surface_sampling.h"
#include "pr/physics/terrain/water/water_field.h"

namespace pr::physics
{
	// GPU buoyancy module that applies sampled-composite forces and records diagnostics through Engine::ExternalForces.
	struct GpuBuoyancy
	{
		// Tunable parameters that govern how the GPU buoyancy pass converts a water surface and
		// a submerged hull into forces. Defaults match fresh water with mild viscous damping.
		struct Config
		{
			// Fluid density (kg/m^3). Buoyancy is the integral of the water pressure, |gravity| * density * depth, over the
			// wet surface samples, which equals |gravity| * density per unit submerged volume. The default matches the analytic
			// constant used by the flat-water diagnostic comparison so unconfigured callers get matching values.
			float m_fluid_density = 1000.0f;

			// Linear viscous drag time-constant (seconds). Each wet surface sample stands for the vertical water column between
			// it and the water surface, with signed volume dV (the columns sum to the submerged volume). Each column contributes
			//   dF = -(fluid_density / tau_linear) * dV * (v_linear - v_water)
			// Tau is the e-folding time for a fully submerged body whose density matches the fluid;
			// lighter bodies decay faster, heavier bodies slower. Set to <= 0 to disable linear drag.
			float m_linear_drag_time_constant_s = 5.0f;

			// Angular viscous drag time-constant (seconds). Each wet water column (see above) contributes
			//   dF = -(fluid_density / tau_angular) * dV * cross(omega, column_centroid - centre_of_mass)
			// Integrating the resulting torque over the submerged geometry damps rotation according to
			// its actual wet shape and lever arms without adding an orientation-specific heuristic.
			// Set to <= 0 to disable angular volume drag.
			float m_angular_drag_time_constant_s = 0.75f;

			// Quadratic normal form-drag coefficient (dimensionless). Each wet surface sample contributes
			//   dF_n = -0.5 * fluid_density * Cd * dA * max(0, v_n)^2 * n_ws
			// where v_n is the outward-normal relative velocity. Typical values are 1.05 for a cube
			// (default), 0.47 for a sphere, and 1.28 for a flat plate. Set to zero to disable form drag.
			float m_quadratic_drag_coefficient = 1.05f;

			// Tangential quadratic drag coefficient (dimensionless). This models unresolved surface
			// shear independently of normal form drag:
			//   dF_t = -0.5 * fluid_density * Ct * dA * |v_t| * v_t
			// The default is a typical skin-friction value for a smooth hull. Motion relative to the water is resisted mainly by
			// the separately modelled form drag, added mass, wave radiation, wave-making resistance, and air drag, so Ct does not
			// need to be raised to make up for them. Set to zero to disable tangential drag.
			float m_tangential_drag_coefficient = 0.01f;

			// Added-mass coefficient (dimensionless). A body cannot accelerate relative to the water without also accelerating the
			// water around it. Each wet water column adds Ca * fluid_density * dV of translational inertia at the centre of mass,
			// and the wave's lateral pressure gradient also drives that water, giving the Morison inertia force (1 + Ca) * rho * dV * a_fluid.
			// For light bodies the added mass is many times the body mass, which is why they follow the water instead of bouncing off it.
			// 0.5 is the value for a sphere. Set to zero to disable added mass.
			float m_added_mass_coefficient = 0.5f;

			// Scale of the wave-radiation damping (dimensionless; 1 is the physical estimate, 0 disables it).
			// A floating body that moves up and down relative to the water makes surface waves, and the energy they carry away
			// damps the motion. For light bodies this damping is very strong, which is why they settle onto the water instead of
			// bouncing on it. It is estimated from the waterline stiffness (see ApplyImplicitResponse in gpu_buoyancy.hlsl).
			float m_radiation_damping = 1.0f;

			// Scale of the steady wave-making resistance (dimensionless; 1 is the physical estimate, 0 disables it).
			// A floating body moving horizontally relative to the surface water leaves a wake of waves behind it, and the energy
			// they carry away resists the motion. The resistance is the linear-theory value for a pressure patch with the body's
			// displaced weight and waterplane shape (see pr/physics/buoyancy/wave_resistance.hlsli). It is negligible for slow
			// bodies, peaks when the body moves at about the speed of waves as long as its waterline, and falls at higher speeds.
			// It stops light bodies from surfing down wave faces. Blunt hulls make less wave resistance than linear theory
			// predicts, so values below 1 can be used for them.
			float m_wave_resistance = 1.0f;

			// Maximum surface-cell diameter in shape-local metres. Buoyancy and drag are both integrated over these cells.
			// Small primitives are sampled more finely so each gets at least buoyancy::MinSurfaceSamplesPerPrimitive samples
			// (see buoyancy::BuildSurfacePlan). Applies at registration/shape refresh; existing registrations retain their
			// immutable plans. No sample-count budget truncates coverage.
			float m_surface_spacing = buoyancy::DefaultSurfaceSpacing;

			// Publish per-hull force, torque, volume, and centre-of-buoyancy diagnostics through GPU
			// readback. Disabled by default so production stepping does not pay for validation data.
			bool m_enable_diagnostics = false;
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
			bool m_valid;

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

		// Construct an articulation-only buoyancy pass whose link targets are resolved directly by the engine.
		GpuBuoyancy(ID3D12Device* device, Engine& engine, Config const& config);
		GpuBuoyancy(GpuBuoyancy const&) = delete;
		GpuBuoyancy& operator=(GpuBuoyancy const&) = delete;

		// Destroy the buoyancy module after all owned registrations have been released.
		~GpuBuoyancy();

		// Return the latest diagnostic record for a registered hull.
		Diagnostics LatestDiagnostics(int body_index, int body_generation) const;

		// Consume diagnostic readback data after the physics engine has completed its GPU step.
		void CompleteStep();

		// Set the water field used by subsequent buoyancy force dispatches. The field is copied; the shader
		// evaluates the same elements as terrain::water::WaterField, and the dry cull uses its MaxHeight.
		// Wind seas are not supported; throws invalid_argument when the field has one.
		void SetWaterField(terrain::water::WaterField const& water_field);

		// Return the water field used by buoyancy force dispatches.
		terrain::water::WaterField const& GetWaterField() const;

		// Set the air above the water. The dry part of each surface cell of a body that touches the water feels the quadratic
		// form drag (Config::m_quadratic_drag_coefficient) relative to this air. This is small for dense bodies but significant
		// for light bodies with a large area, such as a beach ball. Primitives entirely above the highest water surface are not
		// evaluated, so bodies flying clear of the water feel no air drag from this module. Takes effect on the next call to
		// Engine::ExternalForces.
		void SetAir(Air const& air);

		// Return the air used by buoyancy force dispatches.
		Air const& GetAir() const;

		// Set the tunable buoyancy parameters. Fluid parameters take effect on the next call to
		// Engine::ExternalForces; surface spacing applies to later registrations and shape refreshes.
		void SetConfig(Config const& config);

		// Return the tunable buoyancy parameters currently in effect.
		Config const& GetConfig() const;

		// Register a rigid body's collision shape as its buoyancy hull against a stable physics body
		// index. The shape is flattened and copied into an owned descriptor at registration time.
		// A live registration refreshes this cached descriptor when RigidBody::ShapeChange reports a new
		// shape. Bodies that reference the same collision::Shape share the immutable descriptor and
		// its GPU upload. While registered, the body is marked NeverSleep so environmental forces
		// continue to run; the original flag is restored on unregister.
		Registration RegisterCompositeHull(RigidBody& body, int body_index, int body_generation);

		// Register one articulation link's immutable collision shape as a buoyancy hull. The complete tree stays awake while registered, and both objects must
		// outlive any pending step.
		Registration RegisterCompositeHull(Articulation& articulation, LinkHandle link, int body_index, int body_generation);
	};
}
