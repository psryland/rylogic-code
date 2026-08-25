//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#include "pr/physics/rigid_body/rigid_body.h"
#include "pr/physics/articulation/articulation.h"
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

		// Return true when the packed descriptor contributes at least one runtime scalar row.
		bool HasSolverRows(GpuD6ConstraintDesc const& desc)
		{
			for (auto const& axis : desc.axes)
				if (axis.mode != GpuConstraintAxisMode_Free)
					return true;
			return false;
		}

		// Return true when a packed rigid endpoint can receive an impulse and therefore couples constraints dynamically.
		bool DynamicRigidEndpoint(BodyRemap const& remap, int body_idx)
		{
			if (body_idx < 0 || body_idx >= remap.BodyCount())
				return false;

			auto const& body = remap.Body(body_idx);
			return body.InvMass() > 0.0f && !AllSet(body.StateFlags(), ERigidBodyStateFlags::Static);
		}

		// Encode a disjoint-set or target key without conflating rigid-body and articulation/link index spaces.
		uint64_t CoupledKey(int type, int index)
		{
			return (static_cast<uint64_t>(static_cast<uint32_t>(type)) << 32) | static_cast<uint32_t>(index);
		}

		// Build stable independent islands and deterministic target adjacency for persistent coupled rows.
		void BuildCoupledTopology(GpuConstraintUpload& upload, BodyRemap const& remap)
		{
			if (upload.m_coupled_active_count == 0)
				return;

			upload.m_coupled_block_topology.resize(upload.m_endpoints.size(), GpuCoupledConstraintBlockTopology{
				.island_idx = -1,
				.target_idx_a = -1,
				.target_idx_b = -1,
			});

			auto const slot_active = [&](size_t slot_idx)
			{
				auto const& endpoint = upload.m_endpoints[slot_idx];
				return
					AllSet(endpoint.flags, GpuConstraintEndpointFlags_Enabled) &&
					AllSet(endpoint.flags, GpuConstraintEndpointFlags_Coupled) &&
					HasSolverRows(upload.m_descriptors[slot_idx]);
			};

			// Give dynamic rigid bodies and whole articulations distinct graph nodes; fixed endpoints deliberately do not connect islands.
			auto node_indices = std::unordered_map<uint64_t, int>{};
			auto parents = std::vector<int>{};
			auto node_index = [&](int type, int index)
			{
				auto const [iter, inserted] = node_indices.emplace(CoupledKey(type, index), isize(parents));
				if (inserted)
					parents.push_back(iter->second);
				return iter->second;
			};
			auto find_root = [&](int node)
			{
				auto root = node;
				while (parents[root] != root)
					root = parents[root];
				while (parents[node] != node)
				node = std::exchange(parents[node], root);
				return root;
			};
			auto union_nodes = [&](int lhs, int rhs)
			{
				if (lhs < 0 || rhs < 0)
					return;

				auto const lhs_root = find_root(lhs);
				auto const rhs_root = find_root(rhs);
				if (lhs_root != rhs_root)
					parents[rhs_root] = lhs_root;
			};
			auto endpoint_node = [&](size_t slot_idx, bool endpoint_b)
			{
				auto const& endpoint = upload.m_endpoints[slot_idx];
				auto const& coupled = upload.m_coupled_endpoints[slot_idx];
				auto const articulation_idx = endpoint_b ? coupled.articulation_idx_b : coupled.articulation_idx_a;
				if (articulation_idx >= 0)
					return node_index(GpuCoupledConstraintTargetType_Link, articulation_idx);

				auto const body_idx = endpoint_b ? endpoint.body_idx_b : endpoint.body_idx_a;
				return DynamicRigidEndpoint(remap, body_idx)
					? node_index(GpuCoupledConstraintTargetType_Rigid, body_idx)
					: -1;
			};

			// A constraint joins only owners that can exchange momentum; sharing world or a static body does not create physical coupling.
			for (size_t slot_idx = 0; slot_idx != upload.m_endpoints.size(); ++slot_idx)
			{
				if (!slot_active(slot_idx))
					continue;

				union_nodes(endpoint_node(slot_idx, false), endpoint_node(slot_idx, true));
			}

			// Assign compact islands by first stable-block occurrence so repeat packing produces byte-identical topology.
			auto island_by_root = std::unordered_map<int, int>{};
			auto island_counts = std::vector<int>{};
			for (size_t slot_idx = 0; slot_idx != upload.m_endpoints.size(); ++slot_idx)
			{
				if (!slot_active(slot_idx))
					continue;

				auto node = endpoint_node(slot_idx, false);
				if (node < 0)
					node = endpoint_node(slot_idx, true);
				if (node < 0)
					throw std::runtime_error("Coupled constraint has no dynamic or articulation owner");

				auto const root = find_root(node);
				auto const [iter, inserted] = island_by_root.emplace(root, isize(island_counts));
				if (inserted)
					island_counts.push_back(0);
				upload.m_coupled_block_topology[slot_idx].island_idx = iter->second;
				++island_counts[iter->second];
			}

			// Pack each island's stable block indices contiguously for deterministic reductions without floating-point atomics.
			upload.m_coupled_islands.resize(island_counts.size());
			auto island_cursors = std::vector<int>(island_counts.size());
			auto block_offset = 0;
			for (int island_idx = 0; island_idx != isize(island_counts); ++island_idx)
			{
				upload.m_coupled_islands[island_idx] = GpuCoupledConstraintIsland{
					.block_offset = block_offset,
					.block_count = island_counts[island_idx],
				};
				island_cursors[island_idx] = block_offset;
				block_offset += island_counts[island_idx];
			}
			upload.m_coupled_island_blocks.resize(block_offset);
			for (size_t slot_idx = 0; slot_idx != upload.m_endpoints.size(); ++slot_idx)
			{
				auto const island_idx = upload.m_coupled_block_topology[slot_idx].island_idx;
				if (island_idx >= 0)
					upload.m_coupled_island_blocks[island_cursors[island_idx]++] = s_cast<uint32_t>(slot_idx);
			}

			// Map canonical participating-articulation ranges to their owning islands for selective impulse evaluation and commit.
			upload.m_coupled_articulation_islands.reserve(upload.m_coupled_articulation_indices.size());
			for (auto const articulation_idx : upload.m_coupled_articulation_indices)
			{
				auto const node = node_indices.at(CoupledKey(GpuCoupledConstraintTargetType_Link, articulation_idx));
				upload.m_coupled_articulation_islands.push_back(island_by_root.at(find_root(node)));
			}

			// Create targets in stable block/endpoint order and count their fixed contribution ranges.
			auto target_by_key = std::unordered_map<uint64_t, int>{};
			auto target_for_endpoint = [&](size_t slot_idx, bool endpoint_b)
			{
				auto const& endpoint = upload.m_endpoints[slot_idx];
				auto const& coupled = upload.m_coupled_endpoints[slot_idx];
				auto const mobility_idx = endpoint_b ? coupled.mobility_idx_b : coupled.mobility_idx_a;
				auto const body_idx = endpoint_b ? endpoint.body_idx_b : endpoint.body_idx_a;
				auto const target_type = mobility_idx >= 0 ? GpuCoupledConstraintTargetType_Link : GpuCoupledConstraintTargetType_Rigid;
				auto const target_owner = mobility_idx >= 0 ? mobility_idx : body_idx;
				if (mobility_idx < 0 && !DynamicRigidEndpoint(remap, body_idx))
					return -1;

				auto const island_idx = upload.m_coupled_block_topology[slot_idx].island_idx;
				auto const [iter, inserted] = target_by_key.emplace(CoupledKey(target_type, target_owner), isize(upload.m_coupled_targets));
				if (inserted)
				upload.m_coupled_targets.push_back(GpuCoupledConstraintTarget{
					.target_type = target_type,
					.target_idx = target_owner,
					.island_idx = island_idx,
				});
				else if (upload.m_coupled_targets[iter->second].island_idx != island_idx)
					throw std::runtime_error("Coupled target belongs to more than one independent island");

				++upload.m_coupled_targets[iter->second].adjacency_count;
				return iter->second;
			};
			for (size_t slot_idx = 0; slot_idx != upload.m_endpoints.size(); ++slot_idx)
			{
				if (!slot_active(slot_idx))
					continue;

				auto& topology = upload.m_coupled_block_topology[slot_idx];
				topology.target_idx_a = target_for_endpoint(slot_idx, false);
				topology.target_idx_b = target_for_endpoint(slot_idx, true);
			}

			// Prefix ranges and replay the same stable scan to fill each target's contribution indices in deterministic order.
			auto adjacency_count = 0;
			auto target_cursors = std::vector<int>(upload.m_coupled_targets.size());
			for (size_t target_idx = 0; target_idx != upload.m_coupled_targets.size(); ++target_idx)
			{
				auto& target = upload.m_coupled_targets[target_idx];
				target.adjacency_offset = adjacency_count;
				target_cursors[target_idx] = adjacency_count;
				adjacency_count += target.adjacency_count;
			}
			upload.m_coupled_target_adjacency.resize(adjacency_count);
			for (size_t slot_idx = 0; slot_idx != upload.m_coupled_block_topology.size(); ++slot_idx)
			{
				auto const& topology = upload.m_coupled_block_topology[slot_idx];
				if (topology.island_idx < 0)
					continue;
				if (topology.target_idx_a >= 0)
					upload.m_coupled_target_adjacency[target_cursors[topology.target_idx_a]++] = s_cast<uint32_t>(2 * slot_idx);
				if (topology.target_idx_b >= 0)
					upload.m_coupled_target_adjacency[target_cursors[topology.target_idx_b]++] = s_cast<uint32_t>(2 * slot_idx + 1);
			}
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

	// Wake complete articulation trees referenced by active coupled rows before sleeping trees are omitted from frame packing.
	void WakeCoupledConstraintArticulations(ConstraintSet const& constraints, std::span<Articulation*> articulations)
	{
		// Stable-id lookup keeps wake discovery expected O(A + C) without retaining pointers in persistent descriptors.
		auto articulation_lookup = std::unordered_map<uint64_t, Articulation*>{};
		articulation_lookup.reserve(articulations.size());
		for (auto* articulation : articulations)
		{
			if (articulation == nullptr)
				throw std::invalid_argument("Engine articulation inputs cannot contain null pointers");

			articulation_lookup.emplace(articulation->Id().m_value, articulation);
		}

		auto wake_endpoint = [&](BodyRef const& endpoint)
		{
			if (!endpoint.IsLink())
				return;

			auto const iter = articulation_lookup.find(endpoint.m_articulation_id.m_value);
			if (iter != articulation_lookup.end())
				iter->second->Wake();
		};

		// Disabled, free-axis, and rigid-only descriptors do not make an articulation participate in this frame.
		for (auto const& slot : constraints.m_slots)
		{
			if (!slot.m_occupied || !slot.m_desc.m_enabled || !HasSolverRows(slot.m_desc) || !RequiresCoupledSolver(slot.m_desc))
				continue;

			wake_endpoint(slot.m_desc.m_frame_a.m_body);
			wake_endpoint(slot.m_desc.m_frame_b.m_body);
		}
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
					coupled_endpoints.resize(constraints.m_slots.size(), GpuCoupledConstraintEndpoint{
						.articulation_idx_a = -1,
						.link_idx_a = -1,
						.mobility_idx_a = -1,
						.root_link_idx_a = -1,
						.articulation_idx_b = -1,
						.link_idx_b = -1,
						.mobility_idx_b = -1,
						.root_link_idx_b = -1,
					});

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

		// Resolve global link indices into the canonical compact mobility stream once on the CPU so every GPU row remains constant-time.
		auto mobility_offsets = std::vector<int>(remap.ArticulationCount(), -1);
		auto mobility_count = 0;
		for (auto const articulation_index : upload.m_coupled_articulation_indices)
		{
			mobility_offsets[articulation_index] = mobility_count;
			mobility_count += remap.ArticulationBody(articulation_index).LinkCount();
		}
		for (auto& endpoint : upload.m_coupled_endpoints)
		{
			if (endpoint.articulation_idx_a >= 0)
			{
				endpoint.mobility_idx_a = mobility_offsets[endpoint.articulation_idx_a] + endpoint.link_idx_a - remap.ArticulationLinkOffset(endpoint.articulation_idx_a);
				endpoint.root_link_idx_a = remap.ArticulationLinkOffset(endpoint.articulation_idx_a);
			}
			if (endpoint.articulation_idx_b >= 0)
			{
				endpoint.mobility_idx_b = mobility_offsets[endpoint.articulation_idx_b] + endpoint.link_idx_b - remap.ArticulationLinkOffset(endpoint.articulation_idx_b);
				endpoint.root_link_idx_b = remap.ArticulationLinkOffset(endpoint.articulation_idx_b);
			}
		}

		// Persistent D6 rows can build their topology once on the CPU; transient contact rows will append GPU-built substep topology later.
		BuildCoupledTopology(upload, remap);
		upload.m_collision_exclusions = BuildCollisionExclusions(upload.m_endpoints);
		return upload;
	}
}
