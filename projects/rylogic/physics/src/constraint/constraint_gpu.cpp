//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#include "pr/physics/rigid_body/rigid_body.h"
#include "src/constraint/constraint_gpu.h"

namespace pr::physics
{
	namespace
	{
		// Convert one public D6 axis descriptor without changing its canonical enum values.
		GpuConstraintAxisDesc PackAxis(ConstraintAxisDesc const& axis)
		{
			return GpuConstraintAxisDesc{
				.mode = static_cast<int>(axis.m_mode),
				.lower_limit = axis.m_limits.m_beg,
				.upper_limit = axis.m_limits.m_end,
				.target_position = axis.m_target_position,
				.target_velocity = axis.m_target_velocity,
				.stiffness = axis.m_stiffness,
				.damping = axis.m_damping,
				.max_force = axis.m_max_force,
			};
		}

		// Preserve linear XYZ followed by angular XYZ so runtime rows have stable canonical identities.
		GpuD6ConstraintDesc PackDescriptor(D6ConstraintDesc const& desc)
		{
			auto packed = GpuD6ConstraintDesc{
				.frame_a = PackGpuTransform(desc.m_frame_a.m_constraint_to_body),
				.frame_b = PackGpuTransform(desc.m_frame_b.m_constraint_to_body),
			};
			for (int axis = 0; axis != 3; ++axis)
			{
				packed.axes[axis] = PackAxis(desc.m_linear[axis]);
				packed.axes[3 + axis] = PackAxis(desc.m_angular[axis]);
			}
			return packed;
		}

		// Return true when an enabled descriptor contains at least one scalar solver row.
		bool HasSolverRows(D6ConstraintDesc const& desc)
		{
			for (auto const& axis : desc.m_linear)
				if (axis.m_mode != EConstraintAxisMode::Free)
					return true;
			for (auto const& axis : desc.m_angular)
				if (axis.m_mode != EConstraintAxisMode::Free)
					return true;
			return false;
		}

		// True when a descriptor needs a whole-tree response rather than independent rigid-body inverse inertia.
		bool RequiresCoupledSolver(D6ConstraintDesc const& desc)
		{
			return desc.m_frame_a.m_body.IsLink() || desc.m_frame_b.m_body.IsLink();
		}

		// Mix a canonical pair of encoded body indices for deterministic open addressing on both CPU and GPU.
		uint32_t CollisionExclusionHash(uint32_t body_idx_a_plus_one, uint32_t body_idx_b_plus_one)
		{
			auto hash = body_idx_a_plus_one * 0x9E3779B9U;
			hash ^= body_idx_b_plus_one + 0x85EBCA6BU + (hash << 6) + (hash >> 2);
			hash ^= hash >> 16;
			hash *= 0x7FEB352DU;
			hash ^= hash >> 15;
			return hash;
		}

		// Build a deterministic half-full hash table so broadphase lookups require expected constant work and no GPU construction pass.
		GpuCollisionExclusionTable BuildCollisionExclusions(std::span<GpuConstraintEndpoint const> endpoints)
		{
			auto candidates = std::vector<GpuCollisionExclusion>{};
			candidates.reserve(endpoints.size());
			for (auto const& endpoint : endpoints)
			{
				if (!AllSet(endpoint.flags, GpuConstraintEndpointFlags_Enabled) ||
					AllSet(endpoint.flags, GpuConstraintEndpointFlags_CollideConnected) ||
					endpoint.body_idx_a < 0 ||
					endpoint.body_idx_b < 0)
					continue;

				auto const body_idx_a = s_cast<uint32_t>(std::min(endpoint.body_idx_a, endpoint.body_idx_b)) + 1U;
				auto const body_idx_b = s_cast<uint32_t>(std::max(endpoint.body_idx_a, endpoint.body_idx_b)) + 1U;
				candidates.push_back(GpuCollisionExclusion{
					.body_idx_a_plus_one = body_idx_a,
					.body_idx_b_plus_one = body_idx_b,
				});
			}
			if (candidates.empty())
				return {};

			auto table_size = std::bit_ceil(std::max<size_t>(2, candidates.size() * 2));
			auto table = GpuCollisionExclusionTable{
				.m_slots = std::vector<GpuCollisionExclusion>(table_size),
			};
			auto const mask = table.Mask();
			for (auto const& candidate : candidates)
			{
				auto slot = CollisionExclusionHash(candidate.body_idx_a_plus_one, candidate.body_idx_b_plus_one) & mask;
				for (;; slot = (slot + 1U) & mask)
				{
					auto& entry = table.m_slots[slot];
					if (entry.body_idx_a_plus_one == candidate.body_idx_a_plus_one &&
						entry.body_idx_b_plus_one == candidate.body_idx_b_plus_one)
						break;
					if (entry.body_idx_a_plus_one != 0)
						continue;

					entry = candidate;
					++table.m_count;
					break;
				}
			}
			return table;
		}
	}

	// True when an enabled descriptor with solver rows touches at least one articulation link.
	bool HasCoupledConstraintWork(ConstraintSet const& constraints)
	{
		for (auto const& slot : constraints.m_slots)
		{
			if (slot.m_occupied && slot.m_desc.m_enabled && HasSolverRows(slot.m_desc) && RequiresCoupledSolver(slot.m_desc))
				return true;
		}
		return false;
	}

	// Pack persistent descriptors and resolve enabled endpoints into current frame-local rigid-body indices.
	GpuConstraintUpload PackGpuConstraints(ConstraintSet const& constraints, BodyRemap const& remap)
	{
		auto upload = GpuConstraintUpload{
			.m_source = &constraints,
			.m_topology_revision = constraints.m_topology_revision,
			.m_parameter_revision = constraints.m_parameter_revision,
		};
		upload.m_endpoints.resize(constraints.m_slots.size());
		upload.m_endpoint_identities.resize(constraints.m_slots.size());
		upload.m_descriptors.resize(constraints.m_slots.size());
		auto coupled_endpoints = std::vector<GpuCoupledConstraintEndpoint>{};
		auto participating_articulations = std::vector<uint8_t>(remap.ArticulationCount(), false);

		// Slot-preserving output lets removed descriptors become disabled tombstones without changing any surviving row or warm-start identity.
		for (uint32_t slot_index = 0; slot_index != constraints.m_slots.size(); ++slot_index)
		{
			auto const& slot = constraints.m_slots[slot_index];
			if (!slot.m_occupied)
			{
				upload.m_endpoints[slot_index].generation = slot.m_generation;
				continue;
			}

			auto const& desc = slot.m_desc;
			upload.m_endpoint_identities[slot_index] = ConstraintEndpointIdentity{
				.m_body_a = desc.m_frame_a.m_body,
				.m_body_b = desc.m_frame_b.m_body,
			};
			upload.m_descriptors[slot_index] = PackDescriptor(desc);
			auto flags = desc.m_collide_connected ? GpuConstraintEndpointFlags_CollideConnected : GpuConstraintEndpointFlags_None;
			auto const coupled = RequiresCoupledSolver(desc);
			if (coupled)
				flags |= GpuConstraintEndpointFlags_Coupled;
			if (desc.m_enabled)
			{
				flags |= GpuConstraintEndpointFlags_Enabled;
				if (HasSolverRows(desc))
				(coupled ? upload.m_coupled_active_count : upload.m_rigid_active_count) += 1;
			}

			// Disabled constraints deliberately do not require their bodies to be submitted until they are re-enabled.
			auto const endpoint_a = desc.m_enabled ? remap.ResolveEndpoint(desc.m_frame_a.m_body) : CompiledConstraintEndpoint{};
			auto const endpoint_b = desc.m_enabled ? remap.ResolveEndpoint(desc.m_frame_b.m_body) : CompiledConstraintEndpoint{};
			upload.m_endpoints[slot_index] = GpuConstraintEndpoint{
				.body_idx_a = endpoint_a.m_packed_body_index,
				.body_idx_b = endpoint_b.m_packed_body_index,
				.flags = flags,
				.generation = slot.m_generation,
				.break_force = desc.m_break_force,
				.break_torque = desc.m_break_torque,
			};

			// Coupled metadata remains absent on rigid-only frames and records global packed link indices without changing the rigid endpoint ABI.
			if (coupled && desc.m_enabled && HasSolverRows(desc))
			{
				if (coupled_endpoints.empty())
					coupled_endpoints.resize(constraints.m_slots.size(), GpuCoupledConstraintEndpoint{-1, -1, -1, -1});

				auto& coupled_endpoint = coupled_endpoints[slot_index];
				if (endpoint_a.IsLink())
				{
					coupled_endpoint.articulation_idx_a = endpoint_a.m_articulation_index;
					coupled_endpoint.link_idx_a = endpoint_a.m_packed_body_index - remap.BodyCount();
					participating_articulations[endpoint_a.m_articulation_index] = true;
				}
				if (endpoint_b.IsLink())
				{
					coupled_endpoint.articulation_idx_b = endpoint_b.m_articulation_index;
					coupled_endpoint.link_idx_b = endpoint_b.m_packed_body_index - remap.BodyCount();
					participating_articulations[endpoint_b.m_articulation_index] = true;
				}
			}
		}

		// Canonical articulation order makes compact mobility layout independent of constraint insertion order.
		upload.m_coupled_endpoints = std::move(coupled_endpoints);
		for (int articulation_index = 0; articulation_index != remap.ArticulationCount(); ++articulation_index)
			if (participating_articulations[articulation_index])
				upload.m_coupled_articulation_indices.push_back(articulation_index);

		upload.m_collision_exclusions = BuildCollisionExclusions(upload.m_endpoints);
		return upload;
	}
}
