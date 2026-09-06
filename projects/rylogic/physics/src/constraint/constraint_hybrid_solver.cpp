//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#include "pr/physics/articulation/articulation.h"
#include "src/constraint/constraint_solver_internal.h"

namespace pr::physics
{
	using namespace detail::constraint_solver;

	namespace
	{
		// Flat optional storage offsets for only the articulations participating in coupled blocks.
		struct HybridLayout
		{
			std::vector<int> m_link_offsets;
			std::vector<int> m_generalized_offsets;
			std::vector<int> m_participating_articulations;
			int m_link_count = 0;
			int m_generalized_count = 0;
			int m_max_articulation_links = 0;
		};

		// Reusable response and impulse buffers whose sizes are linear in participating articulation state.
		struct HybridScratch
		{
			std::vector<detail::SpatialMobility> m_link_mobilities;
			std::vector<v8force> m_link_impulses;
			std::vector<ArticulationImpulse> m_impulse_requests;
			std::vector<float> m_generalized_velocity;
			std::vector<float> m_generalized_delta;
			std::vector<v8motion> m_link_velocity;
			std::vector<v8motion> m_link_velocity_delta;
		};

		// Return whether a compiled block needs reduced-coordinate articulation response.
		bool IsCoupled(CompiledConstraintBlock const& block)
		{
			return block.m_endpoint_a.IsLink() || block.m_endpoint_b.IsLink();
		}

		// Mark one resolved link endpoint's articulation as participating in the hybrid lane.
		void MarkParticipating(CompiledConstraintEndpoint const& endpoint, std::span<uint8_t> participating)
		{
			if (!endpoint.IsLink())
				return;

			participating[endpoint.m_articulation_index] = true;
		}

		// Pack only participating articulation links and generalized coordinates into contiguous optional scratch.
		HybridLayout BuildHybridLayout(CompiledConstraintSet const& constraints, BodyRemap const& remap)
		{
			auto participating = std::vector<uint8_t>(remap.ArticulationCount(), false);
			for (auto const& block : constraints.m_blocks)
			{
				MarkParticipating(block.m_endpoint_a, participating);
				MarkParticipating(block.m_endpoint_b, participating);
			}

			auto layout = HybridLayout{
				.m_link_offsets = std::vector<int>(remap.ArticulationCount(), -1),
				.m_generalized_offsets = std::vector<int>(remap.ArticulationCount(), -1),
			};
			layout.m_participating_articulations.reserve(remap.ArticulationCount());
			for (int articulation_index = 0; articulation_index != remap.ArticulationCount(); ++articulation_index)
			{
				if (!participating[articulation_index])
					continue;

				auto const& articulation = remap.ArticulationBody(articulation_index);
				layout.m_link_offsets[articulation_index] = layout.m_link_count;
				layout.m_generalized_offsets[articulation_index] = layout.m_generalized_count;
				layout.m_participating_articulations.push_back(articulation_index);
				layout.m_link_count += articulation.LinkCount();
				layout.m_generalized_count += articulation.DofCount();
				layout.m_max_articulation_links = Max(layout.m_max_articulation_links, articulation.LinkCount());
			}
			return layout;
		}

		// Allocate all tree-sized hybrid buffers once for reuse across fixed solver iterations and bounded backtracking.
		HybridScratch MakeHybridScratch(HybridLayout const& layout)
		{
			auto scratch = HybridScratch{
				.m_link_mobilities = std::vector<detail::SpatialMobility>(layout.m_link_count),
				.m_link_impulses = std::vector<v8force>(layout.m_link_count),
				.m_impulse_requests = {},
				.m_generalized_velocity = std::vector<float>(layout.m_generalized_count),
				.m_generalized_delta = std::vector<float>(layout.m_generalized_count),
				.m_link_velocity = std::vector<v8motion>(layout.m_link_count),
				.m_link_velocity_delta = std::vector<v8motion>(layout.m_link_count),
			};
			scratch.m_impulse_requests.reserve(layout.m_max_articulation_links);
			return scratch;
		}

		// Compute every participating tree's exact self-link mobility with one inward/outward recurrence per tree.
		void PrepareLinkMobilities(BodyRemap const& remap, HybridLayout const& layout, std::span<detail::SpatialMobility> mobilities)
		{
			for (auto const articulation_index : layout.m_participating_articulations)
			{
				auto const link_offset = layout.m_link_offsets[articulation_index];
				auto const& articulation = remap.ArticulationBody(articulation_index);
				detail::ComputeArticulationLinkMobilities(
					articulation,
					mobilities.subspan(link_offset, articulation.LinkCount()));
			}
		}

		// Return one endpoint's physical velocity in the coordinate frame used by its compiled Jacobian.
		v8motion EndpointVelocity(CompiledConstraintEndpoint const& endpoint, BodyRemap const& remap, std::span<SolverBody const> bodies)
		{
			switch (endpoint.m_type)
			{
				case EConstraintBodyType::World:
				{
					return {};
				}
				case EConstraintBodyType::Rigid:
				{
					return bodies[endpoint.m_rigid_index].m_velocity;
				}
				case EConstraintBodyType::ArticulationLink:
				{
					return remap.ArticulationBody(endpoint.m_articulation_index).LinkVelocity(endpoint.m_link);
				}
				default:
				{
					throw std::invalid_argument("Unknown constraint endpoint type");
				}
			}
		}

		// Return one endpoint's detached pseudo velocity without reading or changing physical articulation motion.
		v8motion PseudoEndpointVelocity(CompiledConstraintEndpoint const& endpoint, HybridLayout const& layout, std::span<SolverBody const> bodies, std::span<v8motion const> link_velocities)
		{
			switch (endpoint.m_type)
			{
				case EConstraintBodyType::World:
				{
					return {};
				}
				case EConstraintBodyType::Rigid:
				{
					return bodies[endpoint.m_rigid_index].m_velocity;
				}
				case EConstraintBodyType::ArticulationLink:
				{
					return link_velocities[layout.m_link_offsets[endpoint.m_articulation_index] + endpoint.m_link_index];
				}
				default:
				{
					throw std::invalid_argument("Unknown constraint endpoint type");
				}
			}
		}

		// Return the physical relative velocity represented by one mixed-endpoint row.
		float HybridRowVelocity(CompiledConstraintRow const& row, CompiledConstraintBlock const& block, BodyRemap const& remap, std::span<SolverBody const> bodies)
		{
			return
				Dot(row.m_jacobian_a, EndpointVelocity(block.m_endpoint_a, remap, bodies)) +
				Dot(row.m_jacobian_b, EndpointVelocity(block.m_endpoint_b, remap, bodies));
		}

		// Return the detached pseudo relative velocity represented by one mixed-endpoint row.
		float HybridPseudoRowVelocity(CompiledConstraintRow const& row, CompiledConstraintBlock const& block, HybridLayout const& layout, std::span<SolverBody const> bodies, std::span<v8motion const> link_velocities)
		{
			return
				Dot(row.m_jacobian_a, PseudoEndpointVelocity(block.m_endpoint_a, layout, bodies, link_velocities)) +
				Dot(row.m_jacobian_b, PseudoEndpointVelocity(block.m_endpoint_b, layout, bodies, link_velocities));
		}

		// Return one endpoint's exact rigid or self-link response while deliberately omitting same-tree cross-link terms.
		float EndpointResponse(CompiledConstraintEndpoint const& endpoint, v8force lhs, v8force rhs, HybridLayout const& layout, std::span<SolverBody const> bodies, std::span<detail::SpatialMobility const> mobilities)
		{
			switch (endpoint.m_type)
			{
				case EConstraintBodyType::World:
				{
					return 0.0f;
				}
				case EConstraintBodyType::Rigid:
				{
					return Dot(lhs, bodies[endpoint.m_rigid_index].m_inertia_inv * rhs);
				}
				case EConstraintBodyType::ArticulationLink:
				{
					auto const mobility_index = layout.m_link_offsets[endpoint.m_articulation_index] + endpoint.m_link_index;
					return Dot(lhs, mobilities[mobility_index] * rhs);
				}
				default:
				{
					throw std::invalid_argument("Unknown constraint endpoint type");
				}
			}
		}

		// Build the exact-self local preconditioner used by simultaneous articulation-coupled updates.
		void PrepareHybridResponse(RuntimeBlock& runtime, CompiledConstraintSet const& constraints, HybridLayout const& layout, std::span<SolverBody const> bodies, std::span<detail::SpatialMobility const> mobilities, CpuConstraintSolverConfig const& config, CpuConstraintSolveMetrics& metrics)
		{
			auto const& block = constraints.m_blocks[runtime.m_compiled_index];
			auto response = std::array<float, MaxBlockRows * MaxBlockRows>{};
			for (int row = 0; row != runtime.m_row_count; ++row)
			{
				auto const& lhs = constraints.m_rows[runtime.m_rows[row].m_compiled_index];
				for (int column = 0; column != runtime.m_row_count; ++column)
				{
					auto const& rhs = constraints.m_rows[runtime.m_rows[column].m_compiled_index];
					response[row * MaxBlockRows + column] =
						EndpointResponse(block.m_endpoint_a, lhs.m_jacobian_a, rhs.m_jacobian_a, layout, bodies, mobilities) +
						EndpointResponse(block.m_endpoint_b, lhs.m_jacobian_b, rhs.m_jacobian_b, layout, bodies, mobilities);
				}
			}
			PrepareResponse(runtime, response, config, metrics);
		}

		// Return a physical coupled-row gradient including target, bias, and compliant impulse terms.
		float PhysicalResidual(RuntimeRow const& runtime_row, CompiledConstraintBlock const& block, CompiledConstraintSet const& constraints, BodyRemap const& remap, std::span<SolverBody const> bodies)
		{
			auto const& row = constraints.m_rows[runtime_row.m_compiled_index];
			return
				HybridRowVelocity(row, block, remap, bodies) -
				runtime_row.m_target_velocity +
				runtime_row.m_bias +
				runtime_row.m_gamma * runtime_row.m_impulse;
		}

		// Return a detached pseudo-row gradient for hard position correction.
		float PseudoResidual(RuntimeRow const& runtime_row, CompiledConstraintBlock const& block, CompiledConstraintSet const& constraints, HybridLayout const& layout, std::span<SolverBody const> bodies, std::span<v8motion const> link_velocities)
		{
			auto const& row = constraints.m_rows[runtime_row.m_compiled_index];
			return
				HybridPseudoRowVelocity(row, block, layout, bodies, link_velocities) -
				runtime_row.m_target_velocity +
				runtime_row.m_gamma * runtime_row.m_impulse;
		}

		// Form projected simultaneous block candidates and retain their pre-update residuals for the merit test.
		template <typename Residual>
		bool PrepareCoupledCandidates(std::span<RuntimeBlock> blocks, CompiledConstraintSet const& constraints, float relaxation, Residual&& residual)
		{
			auto has_delta = false;
			for (auto& runtime : blocks)
			{
				if (!runtime.m_solvable)
					continue;

				auto const& block = constraints.m_blocks[runtime.m_compiled_index];
				auto candidate = std::array<float, MaxBlockRows>{};
				for (int row = 0; row != runtime.m_row_count; ++row)
					runtime.m_rows[row].m_residual_before = residual(runtime.m_rows[row], block);

				// Each block uses its exact-self response, while every block remains frozen until the complete coupled batch is gathered.
				for (int row = 0; row != runtime.m_row_count; ++row)
				{
					auto correction = 0.0f;
					for (int column = 0; column != runtime.m_row_count; ++column)
						correction += runtime.m_inverse_response[row * MaxBlockRows + column] * runtime.m_rows[column].m_residual_before;
					candidate[row] = runtime.m_rows[row].m_impulse - relaxation * correction;
				}
				Project(candidate, runtime, block);

				for (int row = 0; row != runtime.m_row_count; ++row)
				{
					if (!std::isfinite(candidate[row]))
						throw std::runtime_error("Coupled constraint solve produced a non-finite impulse");

					auto& runtime_row = runtime.m_rows[row];
					runtime_row.m_pending_delta = candidate[row] - runtime_row.m_impulse;
					has_delta |= runtime_row.m_pending_delta != 0.0f;
				}
			}
			return has_delta;
		}

		// Accumulate one row's articulation endpoint contribution into flat link-wrench scratch.
		void GatherLinkImpulse(CompiledConstraintEndpoint const& endpoint, v8force jacobian, float impulse, HybridLayout const& layout, std::span<v8force> link_impulses)
		{
			if (!endpoint.IsLink())
				return;

			auto const impulse_index = layout.m_link_offsets[endpoint.m_articulation_index] + endpoint.m_link_index;
			link_impulses[impulse_index] += jacobian * impulse;
		}

		// Gather all pending mixed impulses and apply one complete-tree impulse ABA per participating articulation.
		void ApplyPhysicalCoupledDeltas(std::span<RuntimeBlock> blocks, CompiledConstraintSet const& constraints, BodyRemap const& remap, HybridLayout const& layout, std::span<SolverBody> bodies, HybridScratch& scratch)
		{
			std::ranges::fill(scratch.m_link_impulses, v8force{});
			for (auto& runtime : blocks)
			{
				auto const& block = constraints.m_blocks[runtime.m_compiled_index];
				for (int row = 0; row != runtime.m_row_count; ++row)
				{
					auto const delta = runtime.m_rows[row].m_pending_delta;
					if (delta == 0.0f)
						continue;

					auto const& compiled_row = constraints.m_rows[runtime.m_rows[row].m_compiled_index];
					ApplyRigidImpulse(compiled_row, block, delta, bodies, true);
					GatherLinkImpulse(block.m_endpoint_a, compiled_row.m_jacobian_a, delta, layout, scratch.m_link_impulses);
					GatherLinkImpulse(block.m_endpoint_b, compiled_row.m_jacobian_b, delta, layout, scratch.m_link_impulses);
				}
			}

			// Scanning participating trees is linear and guarantees all impulses for one tree share exactly one ABA solve.
			for (auto const articulation_index : layout.m_participating_articulations)
			{
				auto const link_offset = layout.m_link_offsets[articulation_index];
				auto& articulation = remap.MutableArticulation(articulation_index);
				scratch.m_impulse_requests.clear();
				for (int link_index = 0; link_index != articulation.LinkCount(); ++link_index)
				{
					auto const impulse = scratch.m_link_impulses[link_offset + link_index];
					if (impulse != v8force{})
						scratch.m_impulse_requests.push_back(ArticulationImpulse{.m_link = articulation.LinkAt(link_index), .m_impulse = impulse});
				}
				if (!scratch.m_impulse_requests.empty())
					articulation.ApplyImpulses(scratch.m_impulse_requests);
			}
		}

		// Apply pending mixed impulses to detached pseudo state using one non-committing ABA response per tree.
		void ApplyPseudoCoupledDeltas(std::span<RuntimeBlock> blocks, CompiledConstraintSet const& constraints, BodyRemap const& remap, HybridLayout const& layout, std::span<SolverBody> bodies, HybridScratch& scratch)
		{
			std::ranges::fill(scratch.m_link_impulses, v8force{});
			for (auto& runtime : blocks)
			{
				auto const& block = constraints.m_blocks[runtime.m_compiled_index];
				for (int row = 0; row != runtime.m_row_count; ++row)
				{
					auto const delta = runtime.m_rows[row].m_pending_delta;
					if (delta == 0.0f)
						continue;

					auto const& compiled_row = constraints.m_rows[runtime.m_rows[row].m_compiled_index];
					ApplyRigidImpulse(compiled_row, block, delta, bodies, false);
					GatherLinkImpulse(block.m_endpoint_a, compiled_row.m_jacobian_a, delta, layout, scratch.m_link_impulses);
					GatherLinkImpulse(block.m_endpoint_b, compiled_row.m_jacobian_b, delta, layout, scratch.m_link_impulses);
				}
			}

			// Detached response buffers accumulate pseudo generalized and link velocities without touching physical momentum.
			for (auto const articulation_index : layout.m_participating_articulations)
			{
				auto const link_offset = layout.m_link_offsets[articulation_index];
				auto& articulation = remap.MutableArticulation(articulation_index);
				auto const generalized_offset = layout.m_generalized_offsets[articulation_index];
				scratch.m_impulse_requests.clear();
				for (int link_index = 0; link_index != articulation.LinkCount(); ++link_index)
				{
					auto const impulse = scratch.m_link_impulses[link_offset + link_index];
					if (impulse != v8force{})
						scratch.m_impulse_requests.push_back(ArticulationImpulse{.m_link = articulation.LinkAt(link_index), .m_impulse = impulse});
				}
				if (scratch.m_impulse_requests.empty())
					continue;

				auto generalized_delta = std::span{scratch.m_generalized_delta}.subspan(generalized_offset, articulation.DofCount());
				auto link_delta = std::span{scratch.m_link_velocity_delta}.subspan(link_offset, articulation.LinkCount());
				detail::ComputeArticulationImpulseResponse(articulation, scratch.m_impulse_requests, generalized_delta, link_delta);
				for (int index = 0; index != articulation.DofCount(); ++index)
					scratch.m_generalized_velocity[generalized_offset + index] += generalized_delta[index];
				for (int index = 0; index != articulation.LinkCount(); ++index)
					scratch.m_link_velocity[link_offset + index] += link_delta[index];
			}
		}

		// Commit pending impulses to row accumulators after their physical or pseudo response has been accepted.
		void AcceptPendingDeltas(std::span<RuntimeBlock> blocks)
		{
			for (auto& runtime : blocks)
				for (int row = 0; row != runtime.m_row_count; ++row)
					runtime.m_rows[row].m_impulse += runtime.m_rows[row].m_pending_delta;
		}

		// Reverse pending impulses after a rejected merit step without changing the retained row accumulators.
		void NegatePendingDeltas(std::span<RuntimeBlock> blocks)
		{
			for (auto& runtime : blocks)
				for (int row = 0; row != runtime.m_row_count; ++row)
					runtime.m_rows[row].m_pending_delta = -runtime.m_rows[row].m_pending_delta;
		}

		// Return the exact quadratic-merit change using gradients before and after one simultaneous update.
		template <typename Residual>
		float CoupledMeritChange(std::span<RuntimeBlock> blocks, CompiledConstraintSet const& constraints, Residual&& residual)
		{
			auto merit_change = 0.0f;
			for (auto& runtime : blocks)
			{
				if (!runtime.m_solvable)
					continue;

				auto const& block = constraints.m_blocks[runtime.m_compiled_index];
				for (int row = 0; row != runtime.m_row_count; ++row)
				{
					auto const& runtime_row = runtime.m_rows[row];
					merit_change +=
						0.5f * runtime_row.m_pending_delta * runtime_row.m_residual_before +
						0.5f * runtime_row.m_pending_delta * residual(runtime_row, block);
				}
			}
			return merit_change;
		}

		// Execute one bounded monotone simultaneous coupled update against physical state.
		void SolvePhysicalCoupledSweep(std::span<RuntimeBlock> blocks, CompiledConstraintSet const& constraints, BodyRemap const& remap, HybridLayout const& layout, std::span<SolverBody> bodies, HybridScratch& scratch, CpuConstraintSolverConfig const& config, CpuConstraintSolveMetrics& metrics)
		{
			auto relaxation = config.m_coupled_relaxation;
			for (int attempt = 0; attempt != config.m_coupled_backtrack_limit + 1; ++attempt)
			{
				auto Residual = [&](RuntimeRow const& row, CompiledConstraintBlock const& block)
				{
					return PhysicalResidual(row, block, constraints, remap, bodies);
				};
				if (!PrepareCoupledCandidates(blocks, constraints, relaxation, Residual))
					return;

				ApplyPhysicalCoupledDeltas(blocks, constraints, remap, layout, bodies, scratch);
				AcceptPendingDeltas(blocks);
				auto const merit_change = CoupledMeritChange(blocks, constraints, Residual);
				auto const tolerance = 64.0f * std::numeric_limits<float>::epsilon() * (1.0f + std::abs(merit_change));
				if (std::isfinite(merit_change) && merit_change <= tolerance)
					return;

				// The ABA map is linear at fixed configuration, so applying the opposite batch restores the pre-attempt state.
				for (auto& runtime : blocks)
					for (int row = 0; row != runtime.m_row_count; ++row)
						runtime.m_rows[row].m_impulse -= runtime.m_rows[row].m_pending_delta;
				NegatePendingDeltas(blocks);
				ApplyPhysicalCoupledDeltas(blocks, constraints, remap, layout, bodies, scratch);
				++metrics.m_coupled_backtracks;
				relaxation *= 0.5f;
			}
			++metrics.m_rejected_coupled_sweeps;
		}

		// Execute one bounded monotone simultaneous coupled update against detached pseudo state.
		void SolvePseudoCoupledSweep(std::span<RuntimeBlock> blocks, CompiledConstraintSet const& constraints, BodyRemap const& remap, HybridLayout const& layout, std::span<SolverBody> bodies, HybridScratch& scratch, CpuConstraintSolverConfig const& config, CpuConstraintSolveMetrics& metrics)
		{
			auto relaxation = Min(config.m_position_relaxation, config.m_coupled_relaxation);
			for (int attempt = 0; attempt != config.m_coupled_backtrack_limit + 1; ++attempt)
			{
				auto Residual = [&](RuntimeRow const& row, CompiledConstraintBlock const& block)
				{
					return PseudoResidual(row, block, constraints, layout, bodies, scratch.m_link_velocity);
				};
				if (!PrepareCoupledCandidates(blocks, constraints, relaxation, Residual))
					return;

				ApplyPseudoCoupledDeltas(blocks, constraints, remap, layout, bodies, scratch);
				AcceptPendingDeltas(blocks);
				auto const merit_change = CoupledMeritChange(blocks, constraints, Residual);
				auto const tolerance = 64.0f * std::numeric_limits<float>::epsilon() * (1.0f + std::abs(merit_change));
				if (std::isfinite(merit_change) && merit_change <= tolerance)
					return;

				for (auto& runtime : blocks)
					for (int row = 0; row != runtime.m_row_count; ++row)
						runtime.m_rows[row].m_impulse -= runtime.m_rows[row].m_pending_delta;
				NegatePendingDeltas(blocks);
				ApplyPseudoCoupledDeltas(blocks, constraints, remap, layout, bodies, scratch);
				++metrics.m_coupled_backtracks;
				relaxation *= 0.5f;
			}
			++metrics.m_rejected_coupled_sweeps;
		}

		// Measure coupled fixed-point residuals and scalar-bound violations after the final physical sweep.
		void MeasureCoupledResidual(std::span<RuntimeBlock const> blocks, CompiledConstraintSet const& constraints, BodyRemap const& remap, std::span<SolverBody const> bodies, CpuConstraintSolveMetrics& metrics)
		{
			for (auto const& runtime : blocks)
			{
				if (!runtime.m_solvable)
					continue;

				auto const& block = constraints.m_blocks[runtime.m_compiled_index];
				auto candidate = std::array<float, MaxBlockRows>{};
				auto const gradient_step = 1.0f / runtime.m_response_scale;
				for (int row = 0; row != runtime.m_row_count; ++row)
				{
					auto const& runtime_row = runtime.m_rows[row];
					auto const residual = PhysicalResidual(runtime_row, block, constraints, remap, bodies);
					candidate[row] = runtime_row.m_impulse - gradient_step * residual;
					metrics.m_max_impulse_bound_violation = Max(
						metrics.m_max_impulse_bound_violation,
						Max(runtime_row.m_lower - runtime_row.m_impulse, runtime_row.m_impulse - runtime_row.m_upper, 0.0f));
				}
				Project(candidate, runtime, block);
				for (int row = 0; row != runtime.m_row_count; ++row)
					metrics.m_projected_velocity_residual = Max(metrics.m_projected_velocity_residual, std::abs(candidate[row] - runtime.m_rows[row].m_impulse) / gradient_step);
			}
		}

		// Return total physical kinetic energy across submitted rigid bodies and participating articulations.
		float HybridKineticEnergy(BodyRemap const& remap, HybridLayout const& layout)
		{
			auto energy = RigidKineticEnergy(remap);
			for (auto const articulation_index : layout.m_participating_articulations)
				energy += remap.ArticulationBody(articulation_index).KineticEnergy();
			return energy;
		}

		// Integrate detached pseudo twists once while preserving every physical momentum and generalized velocity.
		void CommitPositionCorrection(BodyRemap const& remap, HybridLayout const& layout, std::span<SolverBody const> pseudo_bodies, HybridScratch& scratch, float timestep)
		{
			for (int index = 0; index != remap.BodyCount(); ++index)
			{
				auto const& pseudo_velocity = pseudo_bodies[index].m_velocity;
				if (!IsFiniteMotion(pseudo_velocity))
					throw std::runtime_error("Position solve produced a non-finite rigid-body pseudo velocity");
				if (pseudo_velocity == v8motion{})
					continue;

				auto& body = remap.MutableBody(index);
				auto const com_os = body.CentreOfMassOS();
				auto const com_ws = body.CentreOfMassPositionWS();
				auto const new_rotation = m3x3::Rotation((pseudo_velocity.ang * timestep).xyz) * body.O2W().rot;
				auto const new_com_ws = com_ws + pseudo_velocity.lin * timestep;
				auto const new_position = new_com_ws - new_rotation * com_os;
				body.O2W(Orthonorm(m4x4{new_rotation, new_position}));
			}

			for (auto const articulation_index : layout.m_participating_articulations)
			{
				auto const generalized_offset = layout.m_generalized_offsets[articulation_index];
				auto& articulation = remap.MutableArticulation(articulation_index);
				detail::ApplyArticulationPositionCorrection(
					articulation,
					std::span<float const>{scratch.m_generalized_velocity}.subspan(generalized_offset, articulation.DofCount()),
					timestep);
			}
		}
	}

	// Solve a mixed rigid/articulation system with multiplicative rigid and simultaneous impulse-ABA sweeps.
	CpuConstraintSolveMetrics CpuConstraintSolver::SolveHybrid(CompiledConstraintSet const& constraints, BodyRemap const& remap, float timestep, CpuConstraintSolverConfig const& config)
	{
		auto metrics = CpuConstraintSolveMetrics{};
		auto const layout = BuildHybridLayout(constraints, remap);
		auto const energy_before = HybridKineticEnergy(remap, layout);
		auto scratch = MakeHybridScratch(layout);

		// Snapshot ordinary rigid state and prepare exact self-link mobilities once at the fixed constraint configuration.
		auto bodies = std::vector<SolverBody>(remap.BodyCount());
		for (int index = 0; index != remap.BodyCount(); ++index)
		{
			auto const& body = remap.Body(index);
			bodies[index].m_velocity = body.VelocityWS();
			bodies[index].m_inertia_inv = body.InertiaInvWS();
		}
		PrepareLinkMobilities(remap, layout, scratch.m_link_mobilities);

		// Compile active rows into rigid PGS blocks and articulation-coupled simultaneous blocks in stable source order.
		auto rigid_blocks = std::vector<RuntimeBlock>{};
		auto coupled_blocks = std::vector<RuntimeBlock>{};
		rigid_blocks.reserve(constraints.m_blocks.size());
		coupled_blocks.reserve(constraints.m_blocks.size());
		for (uint32_t block_index = 0; block_index != constraints.m_blocks.size(); ++block_index)
		{
			auto const& block = constraints.m_blocks[block_index];
			if (block.m_row_count > MaxBlockRows || block.m_row_begin + block.m_row_count > constraints.m_rows.size())
				throw std::invalid_argument("Compiled constraint block has an invalid row range");

			auto runtime = RuntimeBlock{.m_compiled_index = block_index};
			for (uint32_t local_index = 0; local_index != block.m_row_count; ++local_index)
			{
				auto const compiled_index = block.m_row_begin + local_index;
				auto runtime_row = RuntimeRow{};
				if (PreparePhysicalRow(constraints.m_rows[compiled_index], compiled_index, timestep, runtime_row))
					runtime.m_rows[runtime.m_row_count++] = runtime_row;
			}
			if (runtime.m_row_count == 0)
				continue;

			metrics.m_active_velocity_rows += runtime.m_row_count;
			if (IsCoupled(block))
			{
				metrics.m_coupled_velocity_rows += runtime.m_row_count;
				PrepareHybridResponse(runtime, constraints, layout, bodies, scratch.m_link_mobilities, config, metrics);
				coupled_blocks.push_back(runtime);
			}
			else
			{
				PrepareRigidResponse(runtime, constraints, bodies, config, metrics);
				rigid_blocks.push_back(runtime);
			}
		}

		// Restore projected cached impulses, applying coupled warm starts as one gathered impulse-ABA batch.
		auto RestoreWarmStart = [&](std::span<RuntimeBlock> blocks, bool coupled)
		{
			for (auto& runtime : blocks)
			{
				auto const& block = constraints.m_blocks[runtime.m_compiled_index];
				if (!runtime.m_solvable || !block.m_source || constraints.m_source == nullptr || config.m_warm_start_factor == 0.0f)
					continue;

				auto const found = m_warm_start.find(HandleKey(block.m_source));
				if (found == m_warm_start.end())
					continue;

				auto timestep_scale = found->second.m_timestep > 0.0f ? timestep / found->second.m_timestep : 0.0f;
				if (!std::isfinite(timestep_scale) || timestep_scale < 0.25f || timestep_scale > 4.0f)
					timestep_scale = 0.0f;

				auto candidate = std::array<float, MaxBlockRows>{};
				for (int row = 0; row != runtime.m_row_count; ++row)
				{
					auto const& compiled_row = constraints.m_rows[runtime.m_rows[row].m_compiled_index];
					candidate[row] = config.m_warm_start_factor * timestep_scale * found->second.m_impulses[CacheSlot(compiled_row)];
				}
				Project(candidate, runtime, block);
				for (int row = 0; row != runtime.m_row_count; ++row)
				{
					auto& runtime_row = runtime.m_rows[row];
					auto const& compiled_row = constraints.m_rows[runtime_row.m_compiled_index];
					runtime_row.m_impulse = candidate[row];
					if (coupled)
						runtime_row.m_pending_delta = candidate[row];
					else
						ApplyRigidImpulse(compiled_row, block, candidate[row], bodies, true);
				}
			}
		};
		RestoreWarmStart(rigid_blocks, false);
		RestoreWarmStart(coupled_blocks, true);
		ApplyPhysicalCoupledDeltas(coupled_blocks, constraints, remap, layout, bodies, scratch);

		// Each outer iteration applies one rigid PGS sweep followed by one gathered articulation-coupled sweep.
		for (int iteration = 0; iteration != config.m_velocity_iterations; ++iteration)
		{
			SolveRigidBlocks(rigid_blocks, constraints, bodies, 1, config.m_relaxation, true);
			SolvePhysicalCoupledSweep(coupled_blocks, constraints, remap, layout, bodies, scratch, config, metrics);
			++metrics.m_coupled_sweeps;
		}
		MeasureRigidResidual(rigid_blocks, constraints, bodies, metrics);
		MeasureCoupledResidual(coupled_blocks, constraints, remap, bodies, metrics);

		// Commit ordinary-body impulses after articulation velocities have already received their accepted batched responses.
		for (int index = 0; index != remap.BodyCount(); ++index)
		{
			if (bodies[index].m_momentum_delta != v8force{})
				remap.MutableBody(index).MomentumWS(remap.Body(index).MomentumWS() + bodies[index].m_momentum_delta);
		}
		metrics.m_physical_kinetic_energy_change = HybridKineticEnergy(remap, layout) - energy_before;

		// Replace retained impulses with exactly the active persistent rigid and coupled blocks.
		auto active_handles = std::unordered_set<uint64_t>{};
		if (constraints.m_source != nullptr)
		{
			active_handles.reserve(rigid_blocks.size() + coupled_blocks.size());
			auto StoreWarmStart = [&](std::span<RuntimeBlock const> blocks)
			{
				for (auto const& runtime : blocks)
				{
					auto const& block = constraints.m_blocks[runtime.m_compiled_index];
					if (!runtime.m_solvable || !block.m_source)
						continue;

					auto const key = HandleKey(block.m_source);
					active_handles.insert(key);
					auto& entry = m_warm_start[key];
					entry.m_impulses = {};
					entry.m_timestep = timestep;
					for (int row = 0; row != runtime.m_row_count; ++row)
					{
						auto const& compiled_row = constraints.m_rows[runtime.m_rows[row].m_compiled_index];
						entry.m_impulses[CacheSlot(compiled_row)] = runtime.m_rows[row].m_impulse;
					}
				}
			};
			StoreWarmStart(rigid_blocks);
			StoreWarmStart(coupled_blocks);
			for (auto iter = m_warm_start.begin(); iter != m_warm_start.end();)
				iter = active_handles.contains(iter->first) ? std::next(iter) : m_warm_start.erase(iter);
		}

		// Solve hard drift in detached pseudo rigid and generalized state, then integrate coordinates exactly once.
		if (config.m_position_iterations != 0 && config.m_position_beta != 0.0f && config.m_max_position_speed != 0.0f)
		{
			auto pseudo_bodies = bodies;
			for (auto& body : pseudo_bodies)
			{
				body.m_velocity = {};
				body.m_momentum_delta = {};
			}
			std::ranges::fill(scratch.m_generalized_velocity, 0.0f);
			std::ranges::fill(scratch.m_link_velocity, v8motion{});

			auto rigid_position_blocks = std::vector<RuntimeBlock>{};
			auto coupled_position_blocks = std::vector<RuntimeBlock>{};
			rigid_position_blocks.reserve(constraints.m_blocks.size());
			coupled_position_blocks.reserve(constraints.m_blocks.size());
			for (uint32_t block_index = 0; block_index != constraints.m_blocks.size(); ++block_index)
			{
				auto const& block = constraints.m_blocks[block_index];
				auto runtime = RuntimeBlock{.m_compiled_index = block_index};
				for (uint32_t local_index = 0; local_index != block.m_row_count; ++local_index)
				{
					auto const compiled_index = block.m_row_begin + local_index;
					auto runtime_row = RuntimeRow{};
					if (!PreparePositionRow(constraints.m_rows[compiled_index], compiled_index, timestep, config, runtime_row))
						continue;

					metrics.m_initial_position_error = Max(metrics.m_initial_position_error, std::abs(runtime_row.m_position_error));
					runtime.m_rows[runtime.m_row_count++] = runtime_row;
				}
				if (runtime.m_row_count == 0)
					continue;

				metrics.m_active_position_rows += runtime.m_row_count;
				if (IsCoupled(block))
				{
					PrepareHybridResponse(runtime, constraints, layout, pseudo_bodies, scratch.m_link_mobilities, config, metrics);
					coupled_position_blocks.push_back(runtime);
				}
				else
				{
					PrepareRigidResponse(runtime, constraints, pseudo_bodies, config, metrics);
					rigid_position_blocks.push_back(runtime);
				}
			}

			for (int iteration = 0; iteration != config.m_position_iterations; ++iteration)
			{
				SolveRigidBlocks(rigid_position_blocks, constraints, pseudo_bodies, 1, config.m_position_relaxation, false);
				SolvePseudoCoupledSweep(coupled_position_blocks, constraints, remap, layout, pseudo_bodies, scratch, config, metrics);
			}
			CommitPositionCorrection(remap, layout, pseudo_bodies, scratch, timestep);
		}

		return metrics;
	}
}
