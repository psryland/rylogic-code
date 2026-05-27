//************************************
// Lost at Sea
//  Copyright (c) Rylogic Ltd 2026
//************************************
#pragma once
#include "src/forward.h"

namespace las
{
	// Owns the LAS-side GPU buoyancy prototype and publishes readback diagnostics after each completed physics step.
	struct GpuBuoyancy
	{
		friend struct PhysicsSystem;

		struct BodyState
		{
			m4x4 m_o2w;
			v4 m_centre_of_mass_os;
			bool m_valid;

			// Construct an invalid analytic body-state snapshot.
			BodyState();
		};

		using StepIndexResolver = std::function<int(int body_slot_index)>;
		using BodyStateResolver = std::function<BodyState(int body_slot_index)>;

		struct Diagnostics
		{
			int m_body_slot_index;
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

	private:

		struct HullSlot
		{
			int m_generation;
			v4 m_half_extents;
			bool m_active;
		};
		struct DispatchHull
		{
			int m_body_slot_index;
			int m_body_generation;
			int m_body_step_index;
			v4 m_half_extents;
		};
		struct AnalyticResult
		{
			float m_volume_m3;
			v4 m_force_ws;
			v4 m_centre_buoyancy_ws;
			v4 m_torque_ws;
			bool m_valid;

			// Construct an invalid analytic buoyancy result.
			AnalyticResult();
		};

		ID3D12Device* m_device;
		StepIndexResolver m_step_index_resolver;
		BodyStateResolver m_body_state_resolver;
		::pr::compute::ComputeStep m_column_step;
		::pr::compute::ComputeStep m_reduce_step;
		multicast::AutoSub m_external_force_sub;

		std::vector<HullSlot> m_hulls;
		std::vector<DispatchHull> m_dispatch_hulls;
		std::vector<int> m_pending_body_slots;
		std::vector<int> m_pending_body_generations;
		std::vector<AnalyticResult> m_pending_analytic_results;
		::pr::compute::GpuReadbackBuffer::Allocation m_pending_readback;
		int m_pending_diagnostic_count;

		D3DPtr<ID3D12Resource> m_r_partials;
		D3DPtr<ID3D12Resource> m_r_diagnostics;
		int m_partial_capacity;
		int m_diagnostic_capacity;

		mutable std::mutex m_diagnostics_mutex;
		std::vector<Diagnostics> m_diagnostics;

	public:

		GpuBuoyancy(ID3D12Device* device, physics::Engine& engine, StepIndexResolver step_index_resolver, BodyStateResolver body_state_resolver);
		GpuBuoyancy(GpuBuoyancy const&) = delete;
		GpuBuoyancy& operator=(GpuBuoyancy const&) = delete;
		~GpuBuoyancy();

		// Return the latest diagnostic record for a registered hull.
		Diagnostics LatestDiagnostics(int body_slot_index, int body_generation) const;

		// Consume diagnostic readback data after the physics engine has completed its GPU step.
		void CompleteStep();

	private:

		// Register a generated box buoyancy hull against a LAS physics body slot.
		void RegisterBoxHull(int body_slot_index, int body_generation, v4 size);

		// Remove the buoyancy hull for a LAS physics body slot.
		void UnregisterHull(int body_slot_index, int body_generation) noexcept;

		// Record the diagnostic-only buoyancy compute work into the active physics GPU job.
		void Apply(physics::Engine& sender, physics::Engine::ExternalForceArgs const& args);

		// Calculate the exact flat-water analytic buoyancy result for a generated box hull.
		static AnalyticResult CalculateAnalyticBoxBuoyancy(BodyState const& body_state, v4 half_extents);

		// Resize GPU buffers used by the diagnostic dispatches.
		void EnsureGpuCapacity(physics::GpuJob& job, int hull_count);
	};
}
