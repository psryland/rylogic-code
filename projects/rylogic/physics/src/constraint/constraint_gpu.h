//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#pragma once
#include "src/constraint/constraint_compiler.h"
#include "src/compute/physics_types.h"

namespace pr::physics
{
	// Open-addressed GPU hash table for enabled constrained body pairs whose mutual collisions are disabled.
	struct GpuCollisionExclusionTable
	{
		std::vector<GpuCollisionExclusion> m_slots;
		size_t m_count = 0;

		// The power-of-two table mask, or zero when the table is empty.
		uint32_t Mask() const
		{
			return m_slots.empty() ? 0U : s_cast<uint32_t>(m_slots.size() - 1);
		}
	};

	// Stable host-side endpoint identities used to invalidate only the warm-start rows whose connected bodies changed.
	struct ConstraintEndpointIdentity
	{
		BodyRef m_body_a;
		BodyRef m_body_b;

		// Compare the stable identities of both endpoints.
		friend bool operator==(ConstraintEndpointIdentity const&, ConstraintEndpointIdentity const&) = default;
	};

	// Sparse stable-slot streams used to upload persistent D6 descriptors and frame-local endpoint remaps.
	struct GpuConstraintUpload
	{
		ConstraintSet const* m_source = nullptr;
		std::vector<GpuConstraintEndpoint> m_endpoints;
		std::vector<ConstraintEndpointIdentity> m_endpoint_identities;
		std::vector<GpuD6ConstraintDesc> m_descriptors;
		std::vector<GpuCoupledConstraintEndpoint> m_coupled_endpoints;
		std::vector<int> m_coupled_articulation_indices;
		std::vector<int> m_coupled_articulation_islands;
		std::vector<GpuCoupledConstraintBlockTopology> m_coupled_block_topology;
		std::vector<GpuCoupledConstraintTarget> m_coupled_targets;
		std::vector<uint32_t> m_coupled_target_adjacency;
		std::vector<GpuCoupledConstraintIsland> m_coupled_islands;
		std::vector<uint32_t> m_coupled_island_blocks;
		GpuCollisionExclusionTable m_collision_exclusions;
		size_t m_rigid_active_count = 0;
		size_t m_coupled_active_count = 0;
		uint64_t m_topology_revision = 0;
		uint64_t m_parameter_revision = 0;
	};

	// True when an enabled descriptor with solver rows touches at least one articulation link.
	bool HasCoupledConstraintWork(ConstraintSet const& constraints);

	// Pack persistent descriptors and resolve enabled endpoints into current frame-local rigid-body indices.
	GpuConstraintUpload PackGpuConstraints(ConstraintSet const& constraints, BodyRemap const& remap);
}
