//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#pragma once
#include "pr/physics/forward.h"
#include "pr/physics/integrator/engine.h"

namespace pr::physics
{
	// Diagnostic GPU buoyancy module that records flat-water generated-box displacement results through Engine::ExternalForces.
	struct GpuBuoyancy
	{
		struct BodyState
		{
			m4x4 m_o2w = m4x4::Identity();
			v4 m_centre_of_mass_os = v4::Zero();
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

		// Construct and subscribe the diagnostic buoyancy pass to a physics engine.
		GpuBuoyancy(ID3D12Device* device, Engine& engine, StepIndexResolver step_index_resolver, BodyStateResolver body_state_resolver);
		GpuBuoyancy(GpuBuoyancy const&) = delete;
		GpuBuoyancy& operator=(GpuBuoyancy const&) = delete;

		// Destroy the buoyancy module after all owned registrations have been released.
		~GpuBuoyancy();

		// Return the latest diagnostic record for a registered hull.
		Diagnostics LatestDiagnostics(int body_index, int body_generation) const;

		// Consume diagnostic readback data after the physics engine has completed its GPU step.
		void CompleteStep();

		// Register a generated box buoyancy hull against a stable physics body index.
		Registration RegisterBoxHull(int body_index, int body_generation, v4 size);
	};
}
