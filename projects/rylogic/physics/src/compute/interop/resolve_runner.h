//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#pragma once
#include "pr/physics/integrator/engine_config.h"
#include "pr/physics/integrator/contact_priority.h"
#include "src/compute/physics_types.h"

namespace pr::physics
{
	struct ResolveRunnerBuffers
	{
		float m_dt;
		std::span<GpuRigidBody> m_bodies;
		std::span<GpuResolveContact> m_contacts;
		std::span<GpuMaterial const> m_materials;
	};

	struct IResolveRunner
	{
		virtual ~IResolveRunner() = default;

		virtual void Run(ResolveRunnerBuffers buffers) = 0;
	};

	struct ResolveInteropRunner final : IResolveRunner
	{
		explicit ResolveInteropRunner(EngineConfig const& config);

		void Run(ResolveRunnerBuffers buffers) override;

		// Load/Store expose the individual shader passes for replay debugging.
		void Load(ResolveRunnerBuffers buffers);
		void Store(ResolveRunnerBuffers buffers) const;

		void ComputeCollisionTimes();
		void ComputeShockRanks();
		void SortContacts();
		void AssignColours();
		void PositionSolve(int colour);
		void ResolveVelocity(int colour);

		std::span<uint32_t const> Colours() const;

		// Return true when the bounded graph-colour mask selected the coherent serial fallback.
		bool ColourOverflow() const;
		std::span<uint32_t const> ContactOrder() const;
		std::span<float const> ContactTimes() const;
		ContactPriorityResult ContactPriority(ContactPrioritySettings const& settings = {}) const;

	private:

		EngineConfig m_config;
		float m_dt;
		int m_body_count;
		int m_max_contacts;
		std::vector<GpuCollisionCounters> m_counters;
		std::vector<GpuRigidBody> m_bodies;
		std::vector<GpuResolveContact> m_contacts;
		std::vector<GpuMaterial> m_materials;
		std::vector<uint32_t> m_colours;
		std::vector<uint32_t> m_contact_order;
		std::vector<float> m_contact_times;
		std::vector<uint32_t> m_body_contact_head;
		std::vector<uint32_t> m_contact_next_a;
		std::vector<uint32_t> m_contact_next_b;
	};
}
