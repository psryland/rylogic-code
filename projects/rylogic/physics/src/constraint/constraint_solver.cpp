//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#include "src/constraint/constraint_solver_internal.h"

namespace pr::physics
{
	namespace detail::constraint_solver
	{
		// Describe which side of a scalar limit is currently active.
		enum class ELimitState
		{
			Inactive,
			Lower,
			Upper,
			Bilateral,
		};

		// Return a finite-state check that includes every physically meaningful spatial component.
		bool IsFiniteMotion(v8motion const& value)
		{
			return
				std::isfinite(value.ang.x) &&
				std::isfinite(value.ang.y) &&
				std::isfinite(value.ang.z) &&
				std::isfinite(value.lin.x) &&
				std::isfinite(value.lin.y) &&
				std::isfinite(value.lin.z);
		}

		// Return a stable map key for one generational constraint handle.
		uint64_t HandleKey(ConstraintHandle handle)
		{
			return (static_cast<uint64_t>(handle.m_generation) << 32) | handle.m_index;
		}

		// Map a canonical D6 row to its persistent linear-then-angular cache slot.
		int CacheSlot(CompiledConstraintRow const& row)
		{
			switch (row.m_kind)
			{
				case EConstraintRowKind::Linear:
				{
					return row.m_axis;
				}
				case EConstraintRowKind::Angular:
				{
					return 3 + row.m_axis;
				}
				default:
				{
					throw std::invalid_argument("Unknown compiled constraint row kind");
				}
			}
		}

		// Return the current relative velocity represented by one compiled Jacobian row.
		float RigidRowVelocity(CompiledConstraintRow const& row, CompiledConstraintBlock const& block, std::span<SolverBody const> bodies)
		{
			auto velocity = 0.0f;
			if (block.m_endpoint_a.m_rigid_index >= 0)
				velocity += Dot(row.m_jacobian_a, bodies[block.m_endpoint_a.m_rigid_index].m_velocity);
			if (block.m_endpoint_b.m_rigid_index >= 0)
				velocity += Dot(row.m_jacobian_b, bodies[block.m_endpoint_b.m_rigid_index].m_velocity);
			return velocity;
		}

		// Apply one scalar row impulse immediately so later PGS blocks observe its response.
		void ApplyRigidImpulse(CompiledConstraintRow const& row, CompiledConstraintBlock const& block, float impulse, std::span<SolverBody> bodies, bool accumulate_momentum)
		{
			if (impulse == 0.0f)
				return;

			// Apply equal row-coordinate impulses through each endpoint's inverse spatial inertia.
			if (block.m_endpoint_a.m_rigid_index >= 0)
			{
				auto& body = bodies[block.m_endpoint_a.m_rigid_index];
				auto const momentum = row.m_jacobian_a * impulse;
				auto const velocity_delta = body.m_inertia_inv * momentum;
				body.m_velocity += velocity_delta;
				if (accumulate_momentum && velocity_delta != v8motion{})
					body.m_momentum_delta += momentum;
			}
			if (block.m_endpoint_b.m_rigid_index >= 0)
			{
				auto& body = bodies[block.m_endpoint_b.m_rigid_index];
				auto const momentum = row.m_jacobian_b * impulse;
				auto const velocity_delta = body.m_inertia_inv * momentum;
				body.m_velocity += velocity_delta;
				if (accumulate_momentum && velocity_delta != v8motion{})
					body.m_momentum_delta += momentum;
			}
		}

		// Return one entry of J M^-1 J^T for two rows belonging to the same block.
		float RigidResponse(CompiledConstraintRow const& lhs, CompiledConstraintRow const& rhs, CompiledConstraintBlock const& block, std::span<SolverBody const> bodies)
		{
			auto response = 0.0f;
			if (block.m_endpoint_a.m_rigid_index >= 0)
				response += Dot(lhs.m_jacobian_a, bodies[block.m_endpoint_a.m_rigid_index].m_inertia_inv * rhs.m_jacobian_a);
			if (block.m_endpoint_b.m_rigid_index >= 0)
				response += Dot(lhs.m_jacobian_b, bodies[block.m_endpoint_b.m_rigid_index].m_inertia_inv * rhs.m_jacobian_b);
			return response;
		}

		// Invert a dense matrix of at most six rows using deterministic partial-pivot elimination.
		bool Invert(std::array<float, MaxBlockRows * MaxBlockRows> const& matrix, int dimension, float pivot_tolerance, std::array<float, MaxBlockRows * MaxBlockRows>& inverse)
		{
			auto augmented = std::array<float, MaxBlockRows * MaxBlockRows * 2>{};
			for (int row = 0; row != dimension; ++row)
			{
				for (int column = 0; column != dimension; ++column)
					augmented[row * 2 * MaxBlockRows + column] = matrix[row * MaxBlockRows + column];
				augmented[row * 2 * MaxBlockRows + MaxBlockRows + row] = 1.0f;
			}

			// Pivot and eliminate in a fixed order, using row swaps only when numerically necessary.
			for (int pivot_column = 0; pivot_column != dimension; ++pivot_column)
			{
				auto pivot_row = pivot_column;
				auto pivot_size = std::abs(augmented[pivot_row * 2 * MaxBlockRows + pivot_column]);
				for (int row = pivot_column + 1; row != dimension; ++row)
				{
					auto const candidate = std::abs(augmented[row * 2 * MaxBlockRows + pivot_column]);
					if (candidate > pivot_size)
					{
						pivot_row = row;
						pivot_size = candidate;
					}
				}
				if (!(pivot_size > pivot_tolerance))
					return false;

				if (pivot_row != pivot_column)
				{
					for (int column = 0; column != 2 * MaxBlockRows; ++column)
						std::swap(augmented[pivot_column * 2 * MaxBlockRows + column], augmented[pivot_row * 2 * MaxBlockRows + column]);
				}

				auto const pivot = augmented[pivot_column * 2 * MaxBlockRows + pivot_column];
				for (int column = 0; column != 2 * MaxBlockRows; ++column)
					augmented[pivot_column * 2 * MaxBlockRows + column] /= pivot;

				for (int row = 0; row != dimension; ++row)
				{
					if (row == pivot_column)
						continue;

					auto const factor = augmented[row * 2 * MaxBlockRows + pivot_column];
					for (int column = 0; column != 2 * MaxBlockRows; ++column)
						augmented[row * 2 * MaxBlockRows + column] -= factor * augmented[pivot_column * 2 * MaxBlockRows + column];
				}
			}

			for (int row = 0; row != dimension; ++row)
				for (int column = 0; column != dimension; ++column)
					inverse[row * MaxBlockRows + column] = augmented[row * 2 * MaxBlockRows + MaxBlockRows + column];
			return true;
		}

		// Factor one local response, adding explicit diagonal regularization only for a singular nonzero block.
		void PrepareResponse(RuntimeBlock& runtime, std::array<float, MaxBlockRows * MaxBlockRows> response, CpuConstraintSolverConfig const& config, CpuConstraintSolveMetrics& metrics)
		{
			auto scale = 0.0f;
			for (int row = 0; row != runtime.m_row_count; ++row)
			{
				response[row * MaxBlockRows + row] += runtime.m_rows[row].m_gamma;
				scale = Max(scale, std::abs(response[row * MaxBlockRows + row]));
			}

			// A completely immovable block cannot change velocity and must remain bounded rather than manufacturing impulses.
			if (!(scale > math::tiny<float>))
			{
				++metrics.m_singular_blocks;
				return;
			}
			runtime.m_response_scale = scale;

			auto const tolerance = Max(1.0e-7f * scale, math::tiny<float>);
			if (Invert(response, runtime.m_row_count, tolerance, runtime.m_inverse_response))
			{
				runtime.m_solvable = true;
				return;
			}

			// Redundant rows receive scale-relative regularization so the selected solution stays finite and deterministic.
			auto const regularization = config.m_regularization * Max(scale, 1.0f);
			for (int row = 0; row != runtime.m_row_count; ++row)
				response[row * MaxBlockRows + row] += regularization;
			if (!Invert(response, runtime.m_row_count, tolerance, runtime.m_inverse_response))
			{
				++metrics.m_singular_blocks;
				return;
			}

			runtime.m_solvable = true;
			++metrics.m_regularized_blocks;
		}

		// Build and factor one local response using only ordinary rigid endpoints.
		void PrepareRigidResponse(RuntimeBlock& runtime, CompiledConstraintSet const& constraints, std::span<SolverBody const> bodies, CpuConstraintSolverConfig const& config, CpuConstraintSolveMetrics& metrics)
		{
			auto const& block = constraints.m_blocks[runtime.m_compiled_index];
			auto response = std::array<float, MaxBlockRows * MaxBlockRows>{};
			for (int row = 0; row != runtime.m_row_count; ++row)
			{
				auto const& lhs = constraints.m_rows[runtime.m_rows[row].m_compiled_index];
				for (int column = 0; column != runtime.m_row_count; ++column)
				{
					auto const& rhs = constraints.m_rows[runtime.m_rows[column].m_compiled_index];
					response[row * MaxBlockRows + column] = RigidResponse(lhs, rhs, block, bodies);
				}
			}
			PrepareResponse(runtime, response, config, metrics);
		}

		// Classify a limited row and return its signed error from the active boundary.
		ELimitState LimitState(CompiledConstraintRow const& row, float& position_error)
		{
			if (row.m_limits.m_beg == row.m_limits.m_end)
			{
				position_error = row.m_position - row.m_limits.m_beg;
				return ELimitState::Bilateral;
			}
			if (row.m_position <= row.m_limits.m_beg)
			{
				position_error = row.m_position - row.m_limits.m_beg;
				return ELimitState::Lower;
			}
			if (row.m_position >= row.m_limits.m_end)
			{
				position_error = row.m_position - row.m_limits.m_end;
				return ELimitState::Upper;
			}

			position_error = 0.0f;
			return ELimitState::Inactive;
		}

		// Intersect mode-specific impulse bounds with the descriptor's force cap.
		void SetBounds(RuntimeRow& runtime, ELimitState limit_state, float max_impulse)
		{
			switch (limit_state)
			{
				case ELimitState::Inactive:
					{
						runtime.m_lower = 0.0f;
						runtime.m_upper = 0.0f;
						break;
					}
				case ELimitState::Lower:
					{
						runtime.m_lower = 0.0f;
						runtime.m_upper = max_impulse;
						break;
					}
				case ELimitState::Upper:
					{
						runtime.m_lower = -max_impulse;
						runtime.m_upper = 0.0f;
						break;
					}
				case ELimitState::Bilateral:
					{
						runtime.m_lower = -max_impulse;
						runtime.m_upper = +max_impulse;
						break;
					}
				default:
					{
						throw std::invalid_argument("Unknown scalar limit state");
					}
			}
		}

		// Compile one descriptor row into physical-velocity targets, regularization, and impulse bounds.
		bool PreparePhysicalRow(CompiledConstraintRow const& row, uint32_t compiled_index, float timestep, RuntimeRow& runtime)
		{
			runtime = RuntimeRow{.m_compiled_index = compiled_index};
			auto const max_impulse = row.m_max_force * timestep;
			switch (row.m_mode)
			{
				case EConstraintAxisMode::Free:
					{
						return false;
					}
				case EConstraintAxisMode::Locked:
					{
						runtime.m_position_error = row.m_position - row.m_target_position;
						runtime.m_target_velocity = 0.0f;
						SetBounds(runtime, ELimitState::Bilateral, max_impulse);
						break;
					}
				case EConstraintAxisMode::Limited:
					{
						auto position_error = 0.0f;
						auto const state = LimitState(row, position_error);
						if (state == ELimitState::Inactive)
							return false;

						runtime.m_position_error = position_error;
						runtime.m_target_velocity = 0.0f;
						SetBounds(runtime, state, max_impulse);
						break;
					}
				case EConstraintAxisMode::Driven:
					{
						runtime.m_position_error = row.m_position - row.m_target_position;
						runtime.m_target_velocity = row.m_target_velocity;
						SetBounds(runtime, ELimitState::Bilateral, max_impulse);
						break;
					}
				default:
					{
						throw std::invalid_argument("Unknown constraint axis mode");
					}
			}

			// Implicit spring-damper regularization avoids injecting a large explicit positional velocity.
			auto const denominator = timestep * (row.m_damping + timestep * row.m_stiffness);
			if (denominator > math::tiny<float>)
			{
				runtime.m_gamma = 1.0f / denominator;
				runtime.m_bias = runtime.m_position_error * timestep * row.m_stiffness * runtime.m_gamma;
			}
			return true;
		}

		// Compile one hard passive row into a capped pseudo-velocity correction.
		bool PreparePositionRow(CompiledConstraintRow const& row, uint32_t compiled_index, float timestep, CpuConstraintSolverConfig const& config, RuntimeRow& runtime)
		{
			runtime = RuntimeRow{.m_compiled_index = compiled_index};
			if (row.m_stiffness != 0.0f || row.m_damping != 0.0f)
				return false;

			switch (row.m_mode)
			{
				case EConstraintAxisMode::Free:
				{
					return false;
				}
				case EConstraintAxisMode::Locked:
				{
					runtime.m_position_error = row.m_position - row.m_target_position;
					SetBounds(runtime, ELimitState::Bilateral, std::numeric_limits<float>::infinity());
					break;
				}
				case EConstraintAxisMode::Limited:
				{
					auto position_error = 0.0f;
					auto const state = LimitState(row, position_error);
					if (state == ELimitState::Inactive)
						return false;

					runtime.m_position_error = position_error;
					SetBounds(runtime, state, std::numeric_limits<float>::infinity());
					break;
				}
				case EConstraintAxisMode::Driven:
				{
					return false;
				}
				default:
				{
					throw std::invalid_argument("Unknown constraint axis mode");
				}
			}

			auto const correction = config.m_position_beta * runtime.m_position_error / timestep;
			runtime.m_target_velocity = -std::clamp(correction, -config.m_max_position_speed, +config.m_max_position_speed);
			return runtime.m_position_error != 0.0f;
		}

		// Apply the block's configured feasible-set projection to candidate impulses.
		void Project(std::array<float, MaxBlockRows>& candidate, RuntimeBlock const& runtime, CompiledConstraintBlock const& block)
		{
			switch (block.m_projection)
			{
				case EConstraintProjection::Independent:
					{
						for (int row = 0; row != runtime.m_row_count; ++row)
							candidate[row] = std::clamp(candidate[row], runtime.m_rows[row].m_lower, runtime.m_rows[row].m_upper);
						break;
					}
				case EConstraintProjection::FrictionCone:
					{
						if (runtime.m_row_count != 3)
							throw std::invalid_argument("A friction-cone block must contain exactly three active rows");

						auto const projected = ProjectFrictionCone({candidate[0], candidate[1], candidate[2]}, block.m_friction);
						candidate[0] = projected[0];
						candidate[1] = projected[1];
						candidate[2] = projected[2];
						break;
					}
				default:
					{
						throw std::invalid_argument("Unknown constraint block projection");
					}
			}
		}

		// Execute fixed-iteration block PGS over the supplied mutable row and body state.
		void SolveRigidBlocks(std::span<RuntimeBlock> runtime_blocks, CompiledConstraintSet const& constraints, std::span<SolverBody> bodies, int iterations, float relaxation, bool accumulate_momentum)
		{
			for (int iteration = 0; iteration != iterations; ++iteration)
			{
				// Stable compiled block order is the deterministic Gauss-Seidel update order.
				for (auto& runtime : runtime_blocks)
				{
					if (!runtime.m_solvable)
						continue;

					auto const& block = constraints.m_blocks[runtime.m_compiled_index];
					auto residual = std::array<float, MaxBlockRows>{};
					auto candidate = std::array<float, MaxBlockRows>{};
					for (int row = 0; row != runtime.m_row_count; ++row)
					{
						auto const& runtime_row = runtime.m_rows[row];
						auto const& compiled_row = constraints.m_rows[runtime_row.m_compiled_index];
						residual[row] =
							RigidRowVelocity(compiled_row, block, bodies) -
							runtime_row.m_target_velocity +
							runtime_row.m_bias +
							runtime_row.m_gamma * runtime_row.m_impulse;
					}

					// Solve the complete local block before projection so coupled joint axes share one effective mass.
					for (int row = 0; row != runtime.m_row_count; ++row)
					{
						auto correction = 0.0f;
						for (int column = 0; column != runtime.m_row_count; ++column)
							correction += runtime.m_inverse_response[row * MaxBlockRows + column] * residual[column];
						candidate[row] = runtime.m_rows[row].m_impulse - relaxation * correction;
					}
					Project(candidate, runtime, block);

					// Commit only finite projected candidates so corrupt state never propagates between blocks.
					for (int row = 0; row != runtime.m_row_count; ++row)
					{
						if (!std::isfinite(candidate[row]))
							throw std::runtime_error("Constraint solve produced a non-finite impulse");

						auto& runtime_row = runtime.m_rows[row];
						auto const delta = candidate[row] - runtime_row.m_impulse;
						auto const& compiled_row = constraints.m_rows[runtime_row.m_compiled_index];
						ApplyRigidImpulse(compiled_row, block, delta, bodies, accumulate_momentum);
						runtime_row.m_impulse = candidate[row];
					}
				}
			}
		}

		// Return the fixed-point projection residual and any scalar-bound violation after a solve.
		void MeasureRigidResidual(std::span<RuntimeBlock const> runtime_blocks, CompiledConstraintSet const& constraints, std::span<SolverBody const> bodies, CpuConstraintSolveMetrics& metrics)
		{
			for (auto const& runtime : runtime_blocks)
			{
				if (!runtime.m_solvable)
					continue;

				auto const& block = constraints.m_blocks[runtime.m_compiled_index];
				auto candidate = std::array<float, MaxBlockRows>{};
				auto const gradient_step = 1.0f / runtime.m_response_scale;
				for (int row = 0; row != runtime.m_row_count; ++row)
				{
					auto const& runtime_row = runtime.m_rows[row];
					auto const& compiled_row = constraints.m_rows[runtime_row.m_compiled_index];
					auto const residual =
						RigidRowVelocity(compiled_row, block, bodies) -
						runtime_row.m_target_velocity +
						runtime_row.m_bias +
						runtime_row.m_gamma * runtime_row.m_impulse;
					candidate[row] = runtime_row.m_impulse - gradient_step * residual;

					auto const lower_violation = runtime_row.m_lower - runtime_row.m_impulse;
					auto const upper_violation = runtime_row.m_impulse - runtime_row.m_upper;
					metrics.m_max_impulse_bound_violation = Max(metrics.m_max_impulse_bound_violation, Max(lower_violation, upper_violation, 0.0f));
				}
				Project(candidate, runtime, block);
				for (int row = 0; row != runtime.m_row_count; ++row)
					metrics.m_projected_velocity_residual = Max(metrics.m_projected_velocity_residual, std::abs(candidate[row] - runtime.m_rows[row].m_impulse) / gradient_step);
			}
		}

		// Validate finite fixed-work settings before mutating any body state.
		void ValidateSolverInputs(CpuConstraintSolverConfig const& config, float timestep)
		{
			if (!std::isfinite(timestep) || timestep <= 0.0f)
				throw std::invalid_argument("Constraint timestep must be finite and positive");
			if (config.m_velocity_iterations < 0 || config.m_position_iterations < 0)
				throw std::invalid_argument("Constraint iteration counts cannot be negative");
			if (!std::isfinite(config.m_relaxation) || config.m_relaxation <= 0.0f || config.m_relaxation > 2.0f)
				throw std::invalid_argument("Velocity relaxation must be finite and in (0,2]");
			if (!std::isfinite(config.m_coupled_relaxation) || config.m_coupled_relaxation <= 0.0f || config.m_coupled_relaxation > 1.0f)
				throw std::invalid_argument("Coupled relaxation must be finite and in (0,1]");
			if (!std::isfinite(config.m_position_relaxation) || config.m_position_relaxation <= 0.0f || config.m_position_relaxation > 2.0f)
				throw std::invalid_argument("Position relaxation must be finite and in (0,2]");
			if (!std::isfinite(config.m_position_beta) || config.m_position_beta < 0.0f || config.m_position_beta > 1.0f)
				throw std::invalid_argument("Position beta must be finite and in [0,1]");
			if (!std::isfinite(config.m_max_position_speed) || config.m_max_position_speed < 0.0f)
				throw std::invalid_argument("Maximum position-correction speed must be finite and nonnegative");
			if (!std::isfinite(config.m_regularization) || config.m_regularization < 0.0f)
				throw std::invalid_argument("Constraint regularization must be finite and nonnegative");
			if (!std::isfinite(config.m_warm_start_factor) || config.m_warm_start_factor < 0.0f || config.m_warm_start_factor > 1.0f)
				throw std::invalid_argument("Warm-start factor must be finite and in [0,1]");
			if (config.m_coupled_backtrack_limit < 0 || config.m_coupled_backtrack_limit > 16)
				throw std::invalid_argument("Coupled backtrack limit must be between zero and sixteen");
		}

		// Sum kinetic energy without allocating per-body diagnostic state.
		float RigidKineticEnergy(BodyRemap const& remap)
		{
			auto energy = 0.0f;
			for (int index = 0; index != remap.BodyCount(); ++index)
				energy += remap.Body(index).KineticEnergy();
			return energy;
		}
	}

	using namespace detail::constraint_solver;

	// Project a normal and two tangent impulses onto an exact circular Coulomb cone.
	std::array<float, 3> ProjectFrictionCone(std::array<float, 3> impulse, float friction)
	{
		if (!std::isfinite(friction) || friction < 0.0f)
			throw std::invalid_argument("Friction must be finite and nonnegative");
		for (auto const value : impulse)
			if (!std::isfinite(value))
				throw std::invalid_argument("Friction-cone input must be finite");

		auto const tangent_length = Sqrt(Sqr(impulse[1]) + Sqr(impulse[2]));
		if (impulse[0] >= 0.0f && tangent_length <= friction * impulse[0])
			return impulse;
		if (impulse[0] + friction * tangent_length <= 0.0f)
			return {};
		if (friction == 0.0f || tangent_length <= math::tiny<float>)
			return {Max(impulse[0], 0.0f), 0.0f, 0.0f};

		// The nearest boundary point preserves tangent direction and solves one scalar least-squares problem.
		auto const normal = (impulse[0] + friction * tangent_length) / (1.0f + Sqr(friction));
		auto const tangent_scale = friction * normal / tangent_length;
		return {normal, impulse[1] * tangent_scale, impulse[2] * tangent_scale};
	}

	// Construct an empty solver with no retained warm-start state.
	CpuConstraintSolver::CpuConstraintSolver()
		:m_warm_start()
		,m_source()
		,m_topology_revision()
		,m_parameter_revision()
	{}

	// Discard every retained impulse.
	void CpuConstraintSolver::ClearWarmStart()
	{
		m_warm_start.clear();
		m_source = nullptr;
		m_topology_revision = 0;
		m_parameter_revision = 0;
	}

	// Solve physical velocities and optional split position correction in deterministic block order.
	CpuConstraintSolveMetrics CpuConstraintSolver::Solve(CompiledConstraintSet const& constraints, BodyRemap const& remap, float timestep, CpuConstraintSolverConfig const& config)
	{
		ValidateSolverInputs(config, timestep);

		// Descriptor or topology changes invalidate cached impulses before any warm start can target stale row semantics.
		if (m_source != constraints.m_source || m_topology_revision != constraints.m_topology_revision || m_parameter_revision != constraints.m_parameter_revision)
			ClearWarmStart();
		m_source = constraints.m_source;
		m_topology_revision = constraints.m_topology_revision;
		m_parameter_revision = constraints.m_parameter_revision;

		// Preserve the minimal rigid-only path unless at least one active block requires complete-tree articulation response.
		auto const has_coupled_blocks = std::ranges::any_of(constraints.m_blocks, [](CompiledConstraintBlock const& block)
		{
			return block.m_endpoint_a.IsLink() || block.m_endpoint_b.IsLink();
		});
		if (has_coupled_blocks)
			return SolveHybrid(constraints, remap, timestep, config);

		auto metrics = CpuConstraintSolveMetrics{};
		auto const energy_before = RigidKineticEnergy(remap);

		// Snapshot mutable solver state while retaining physical momentum as the authoritative body quantity.
		auto bodies = std::vector<SolverBody>(remap.BodyCount());
		for (int index = 0; index != remap.BodyCount(); ++index)
		{
			auto const& body = remap.Body(index);
			bodies[index].m_velocity = body.VelocityWS();
			bodies[index].m_inertia_inv = body.InertiaInvWS();
		}

		// Compile active physical rows and their small block responses.
		auto velocity_blocks = std::vector<RuntimeBlock>{};
		velocity_blocks.reserve(constraints.m_blocks.size());
		for (uint32_t block_index = 0; block_index != constraints.m_blocks.size(); ++block_index)
		{
			auto const& block = constraints.m_blocks[block_index];
			if (block.m_row_count > MaxBlockRows || block.m_row_begin + block.m_row_count > constraints.m_rows.size())
				throw std::invalid_argument("Compiled constraint block has an invalid row range");

			auto runtime = RuntimeBlock{.m_compiled_index = block_index};
			for (uint32_t local_index = 0; local_index != block.m_row_count; ++local_index)
			{
				auto const compiled_index = block.m_row_begin + local_index;
				auto const& row = constraints.m_rows[compiled_index];
				auto runtime_row = RuntimeRow{};
				if (!PreparePhysicalRow(row, compiled_index, timestep, runtime_row))
					continue;

				runtime.m_rows[runtime.m_row_count++] = runtime_row;
			}
			if (runtime.m_row_count == 0)
				continue;

			metrics.m_active_velocity_rows += runtime.m_row_count;
			PrepareRigidResponse(runtime, constraints, bodies, config, metrics);
			velocity_blocks.push_back(runtime);
		}

		// Restore scaled cached impulses and project them against this frame's active bounds before applying them.
		for (auto& runtime : velocity_blocks)
		{
			auto const& block = constraints.m_blocks[runtime.m_compiled_index];
			if (!runtime.m_solvable || !block.m_source || constraints.m_source == nullptr || config.m_warm_start_factor == 0.0f)
				continue;

			auto const found = m_warm_start.find(HandleKey(block.m_source));
			if (found == m_warm_start.end())
				continue;

			// Discard cache entries across extreme timestep changes rather than amplifying a stale impulse.
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
				ApplyRigidImpulse(compiled_row, block, candidate[row], bodies, true);
			}
		}

		// Fixed iteration counts make runtime linear in active rows and reproducible for identical inputs.
		SolveRigidBlocks(velocity_blocks, constraints, bodies, config.m_velocity_iterations, config.m_relaxation, true);
		MeasureRigidResidual(velocity_blocks, constraints, bodies, metrics);

		// Commit physical impulses without reconstructing momentum from possibly singular static inertia.
		for (int index = 0; index != remap.BodyCount(); ++index)
		{
			if (bodies[index].m_momentum_delta != v8force{})
				remap.MutableBody(index).MomentumWS(remap.Body(index).MomentumWS() + bodies[index].m_momentum_delta);
		}
		metrics.m_physical_kinetic_energy_change = RigidKineticEnergy(remap) - energy_before;

		// Replace the cache with exactly the active persistent blocks so removed and inactive rows cannot reappear later.
		auto active_handles = std::unordered_set<uint64_t>{};
		if (constraints.m_source != nullptr)
		{
			active_handles.reserve(velocity_blocks.size());
			for (auto const& runtime : velocity_blocks)
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
			for (auto iter = m_warm_start.begin(); iter != m_warm_start.end();)
				iter = active_handles.contains(iter->first) ? std::next(iter) : m_warm_start.erase(iter);
		}

		// Solve hard positional drift in a separate pseudo-velocity state so physical momentum is unchanged.
		if (config.m_position_iterations != 0 && config.m_position_beta != 0.0f && config.m_max_position_speed != 0.0f)
		{
			auto pseudo_bodies = bodies;
			for (auto& body : pseudo_bodies)
			{
				body.m_velocity = {};
				body.m_momentum_delta = {};
			}

			auto position_blocks = std::vector<RuntimeBlock>{};
			position_blocks.reserve(constraints.m_blocks.size());
			for (uint32_t block_index = 0; block_index != constraints.m_blocks.size(); ++block_index)
			{
				auto const& block = constraints.m_blocks[block_index];
				auto runtime = RuntimeBlock{.m_compiled_index = block_index};
				for (uint32_t local_index = 0; local_index != block.m_row_count; ++local_index)
				{
					auto const compiled_index = block.m_row_begin + local_index;
					auto const& row = constraints.m_rows[compiled_index];
					auto runtime_row = RuntimeRow{};
					if (!PreparePositionRow(row, compiled_index, timestep, config, runtime_row))
						continue;

					metrics.m_initial_position_error = Max(metrics.m_initial_position_error, std::abs(runtime_row.m_position_error));
					runtime.m_rows[runtime.m_row_count++] = runtime_row;
				}
				if (runtime.m_row_count == 0)
					continue;

				metrics.m_active_position_rows += runtime.m_row_count;
				PrepareRigidResponse(runtime, constraints, pseudo_bodies, config, metrics);
				position_blocks.push_back(runtime);
			}

			SolveRigidBlocks(position_blocks, constraints, pseudo_bodies, config.m_position_iterations, config.m_position_relaxation, false);

			// Integrate pseudo twists once about each CoM and retain the original physical momentum.
			for (int index = 0; index != remap.BodyCount(); ++index)
			{
				auto const& pseudo_velocity = pseudo_bodies[index].m_velocity;
				if (!IsFiniteMotion(pseudo_velocity))
					throw std::runtime_error("Position solve produced a non-finite pseudo velocity");
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
		}

		return metrics;
	}
}
