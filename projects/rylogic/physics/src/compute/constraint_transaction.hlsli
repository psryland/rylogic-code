//*********************************************
// Physics Engine — Persistent Constraint Transactions
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#ifndef PR_PHYSICS_CONSTRAINT_TRANSACTION_HLSLI
#define PR_PHYSICS_CONSTRAINT_TRANSACTION_HLSLI
#include "physics_types.hlsli"
#include "body_twist.hlsli"
#include "constraint_algebra.hlsli"

#ifdef __cplusplus
namespace pr::physics {
using constraint_algebra::ProjectConstraintVelocityStep;
#endif

// One numerical substep with stable commit-unit membership. Epoch identifiers must not be reused while a cache remains alive.
struct GpuConstraintEpoch
{
	uint2 identity;
	float timestep;
	float warm_start_factor;
};

// Provenance of the last accepted response and coefficient cache, not the last attempted preparation.
struct GpuConstraintAcceptance
{
	uint2 epoch;
	float timestep;
	uint valid;
};

// Accepted coefficients and feature identities for one block. The containing commit unit owns their acceptance stamp.
struct GpuConstraintAcceptedRows
{
	float impulses[GpuConstraintRowsPerBlock];
	uint velocity_mask;
	uint row_states;
	uint row_identities;
	float linear_impulse_norm;
	float angular_impulse_norm;
	uint invalidation_receipt; // High 30 bits: input revision. Low two bits: accepted endpoint-body reset levels.
};

// One proposed primary-state component and its actual accumulated owned response. Admission does not publish either field.
struct GpuConstraintStagedComponent
{
	float value;
	float response;
};

// Return whether this record belongs to the current numerical substep, independently of its response magnitude.
inline bool ConstraintAcceptedInEpoch(GpuConstraintAcceptance accepted, GpuConstraintEpoch epoch)
{
	// A valid zero impulse still has temporal provenance; a prior-substep cache is never a current-substep response.
	return accepted.valid != 0u && all(accepted.epoch == epoch.identity);
}

// Stage cache scaling without changing its provenance. False indicates invalid context, including a timestep change within one epoch.
// With valid context, the derived scale may overflow; callers must reject that provisional seed before using it, not report an input failure.
inline bool ConstraintStageGuessScale(GpuConstraintAcceptance accepted, GpuConstraintEpoch epoch, out_(float) scale)
{
	// Context validation belongs before any prospective physical or cache publication.
	scale = 0.0f;
	if (!(epoch.timestep > 0.0f) || !isfinite(epoch.timestep) || !isfinite(epoch.warm_start_factor) || epoch.warm_start_factor < 0.0f)
		return false;

	if (accepted.valid == 0u)
		return true;

	if (!(accepted.timestep > 0.0f) || !isfinite(accepted.timestep))
		return false;

	if (ConstraintAcceptedInEpoch(accepted, epoch))
	{
		// Current-substep coefficients are unscaled guesses only when that substep actually accepted them.
		scale = 1.0f;
		return accepted.timestep == epoch.timestep;
	}

	// Preserve the existing cross-substep warm policy, measured from the last accepted timestep rather than a rejected attempt.
	float ratio = epoch.timestep / accepted.timestep;
	if (isfinite(ratio) && ratio >= 0.25f && ratio <= 4.0f)
		scale = epoch.warm_start_factor * ratio;

	return true;
}

// Return one compatible provisional coefficient. Callers supply a valid row index, finite scale, and ordered impulse bounds.
inline float ConstraintRowGuess(GpuConstraintAcceptedRows accepted, uint row_index, uint velocity_mask, uint row_states, uint row_identities, float2 bounds, float scale)
{
	// Row state and geometric feature must both match; an active row alone does not establish compatibility.
	uint active = 1u << row_index;
	uint state_shift = 2u * row_index;
	uint identity_shift = 4u * row_index;
	if ((accepted.velocity_mask & velocity_mask & active) == 0u ||
		((accepted.row_states ^ row_states) & (3u << state_shift)) != 0u ||
		((accepted.row_identities ^ row_identities) & (15u << identity_shift)) != 0u)
		return 0.0f;

	return clamp(accepted.impulses[row_index] * scale, bounds.x, bounds.y);
}

// Recover one free primary-state component without writing live state or interpreting old impulses through new mobility.
inline bool ConstraintStageFreeComponent(float live, float owned_response, GpuConstraintAcceptance accepted, GpuConstraintEpoch epoch, out_(float) free_value)
{
	// Previous-substep responses are physical history, even if their storage has not yet been overwritten.
	float previous = ConstraintAcceptedInEpoch(accepted, epoch) ? owned_response : 0.0f;
	free_value = live - previous;
	return isfinite(live) && isfinite(previous) && isfinite(free_value);
}

// Stage replacement of one owned response. All components of the commit unit must pass admission before any result is published.
inline bool ConstraintStageReplacement(float live, float owned_response, float new_response, GpuConstraintAcceptance accepted, GpuConstraintEpoch epoch, out_(GpuConstraintStagedComponent) staged)
{
	// Apply only the response difference, preserving exact no-effect replay and other writers' intervening physical changes.
	float previous = ConstraintAcceptedInEpoch(accepted, epoch) ? owned_response : 0.0f;
	float delta = new_response - previous;
	staged.value = live + delta;
	staged.response = previous + (staged.value - live);
	return isfinite(live) && isfinite(previous) && isfinite(new_response) && isfinite(delta) && isfinite(staged.value) && isfinite(staged.response);
}

// Stage one dynamic rigid endpoint's replacement. Response vectors are world-COM momentum increments, not torques or velocities.
inline bool ConstraintStageRigidEndpoint(GpuRigidBody live, GpuArticulationSpatialVector previous, GpuArticulationSpatialVector replacement, GpuConstraintAcceptance accepted, GpuConstraintEpoch epoch, out_(GpuRigidBody) staged, out_(GpuArticulationSpatialVector) response)
{
	// Every prospective component stays local; the caller admits both endpoints or the complete coupled island together.
	staged = live;
	response.ang = float4(0, 0, 0, 0);
	response.lin = float4(0, 0, 0, 0);
	for (int component = 0; component != 3; ++component)
	{
		// Record represented primary-state changes rather than assuming each computed impulse survived rounding.
		GpuConstraintStagedComponent angular;
		GpuConstraintStagedComponent translation;
		if (!ConstraintStageReplacement(live.momentum_ang[component], previous.ang[component], replacement.ang[component], accepted, epoch, angular) ||
			!ConstraintStageReplacement(live.momentum_lin[component], previous.lin[component], replacement.lin[component], accepted, epoch, translation))
			return false;

		staged.momentum_ang[component] = angular.value;
		staged.momentum_lin[component] = translation.value;
		response.ang[component] = angular.response;
		response.lin[component] = translation.response;
	}

	// Seed/undo/correction operations cannot wake a body. Only a represented accepted physical change invalidates sleep state.
	bool changed = any(staged.momentum_ang.xyz != live.momentum_ang.xyz) || any(staged.momentum_lin.xyz != live.momentum_lin.xyz);
	if (changed && (AllSet(live.state_flags, ERigidBodyStateFlags_Sleeping) || live.sleep.island_id >= 0))
	{
		// This remains prospective sleep state until the enclosing commit unit accepts.
		staged.sleep.generation++;
		staged.state_flags = SetFlag(staged.state_flags, ERigidBodyStateFlags_Sleeping, false);
		staged.sleep.timer_s = 0.0f;
		staged.sleep.island_id = -1;
		staged.sleep.flags = 0u;
	}
	return true;
}

// Stage coefficients and the detector's existing load definition from prepared rows. False is admission failure, not publication.
inline bool ConstraintStageAcceptedRows(GpuConstraintBlock block, GpuConstraintRow rows[GpuConstraintRowsPerBlock], out_(GpuConstraintAcceptedRows) accepted)
{
	// Clear all old obligations, including inactive/null-response support with nonzero accepted coefficients.
	accepted.velocity_mask = block.velocity_mask;
	accepted.row_states = block.row_states;
	accepted.row_identities = block.row_identities;
	accepted.linear_impulse_norm = 0.0f;
	accepted.angular_impulse_norm = 0.0f;
	accepted.invalidation_receipt = 0u;
	for (uint row_index = 0; row_index != GpuConstraintRowsPerBlock; ++row_index)
	{
		// Inactive working rows are not an accepted physical cache.
		accepted.impulses[row_index] = 0.0f;
	}

	// Linear-row moments about COM do not contribute to this detector's pure-angular-row torque metric.
	float linear_squared = 0.0f;
	float3 angular = float3(0, 0, 0);
	for (uint row_index = 0; row_index != GpuConstraintRowsPerBlock; ++row_index)
	{
		// Match the established canonical-row load reduction, not a norm of the response ledger.
		if ((block.velocity_mask & (1u << row_index)) == 0u)
			continue;

		GpuConstraintRow row = rows[row_index];
		float impulse = row.bounds.z;
		if (!isfinite(impulse) || !(impulse >= row.bounds.x && impulse <= row.bounds.y))
			return false;

		accepted.impulses[row_index] = impulse;
		if (any(row.jacobian_a_lin.xyz != float3(0, 0, 0)) || any(row.jacobian_b_lin.xyz != float3(0, 0, 0)))
		{
			// These scalar axes are orthonormal under the existing descriptor contract.
			linear_squared += impulse * impulse;
		}
		else
		{
			// Either non-world endpoint supplies the pure angular impulse in one consistent endpoint frame.
			float3 gradient = any(row.jacobian_a_ang.xyz != float3(0, 0, 0)) ? row.jacobian_a_ang.xyz : row.jacobian_b_ang.xyz;
			angular += impulse * gradient;
		}
	}
	accepted.linear_impulse_norm = sqrt(linear_squared);
	accepted.angular_impulse_norm = length(angular);
	return isfinite(accepted.linear_impulse_norm) && isfinite(accepted.angular_impulse_norm);
}

// Construct provenance to publish only with an accepted complete commit unit. The epoch context must already have passed validation.
inline GpuConstraintAcceptance ConstraintAcceptance(GpuConstraintEpoch epoch)
{
	// Empty-row acceptance also advances provenance and clears the accepted ownership at this epoch.
	GpuConstraintAcceptance accepted;
	accepted.epoch = epoch.identity;
	accepted.timestep = epoch.timestep;
	accepted.valid = 1u;
	return accepted;
}

// One ordinary block's accepted provenance, coefficients and actual endpoint momentum responses.
struct GpuConstraintRigidRecord
{
	GpuConstraintAcceptance accepted;
	GpuConstraintAcceptedRows rows;
	GpuArticulationSpatialVector response_a;
	GpuArticulationSpatialVector response_b;
};

// Invalid context/projection arithmetic, physical admission, and existing numerical rejection remain distinct outcomes.
static const uint GpuConstraintStage_Accepted = 0u;
static const uint GpuConstraintStage_AdmissionRejected = 1u;
static const uint GpuConstraintStage_NumericalFailure = 2u;
static const uint GpuConstraintStage_SolveRejected = 3u;

// Form the total endpoint responses of prepared rows with distinct endpoint identities.
inline void ConstraintRigidRowResponses(GpuConstraintBlock block, GpuConstraintRow rows[GpuConstraintRowsPerBlock], bool dynamic_a, bool dynamic_b, out_(GpuArticulationSpatialVector) response_a, out_(GpuArticulationSpatialVector) response_b)
{
	// Keep response construction separate from live momentum and its sleep state.
	response_a.ang = response_a.lin = float4(0, 0, 0, 0);
	response_b = response_a;
	for (uint axis = 0; axis != GpuConstraintRowsPerBlock; ++axis)
	{
		// Inactive rows never own a physical response, even when an old cache coefficient is nonzero.
		if ((block.velocity_mask & (1u << axis)) == 0u)
			continue;

		// The validated distinct endpoints retain independently attributable wrenches.
		GpuConstraintRow row = rows[axis];
		if (dynamic_a)
		{
			// World and kinematic endpoints do not receive an impulse.
			response_a.ang += row.bounds.z * row.jacobian_a_ang;
			response_a.lin += row.bounds.z * row.jacobian_a_lin;
		}
		if (dynamic_b)
		{
			// Distinct endpoints remain independently attributable within the block.
			response_b.ang += row.bounds.z * row.jacobian_b_ang;
			response_b.lin += row.bounds.z * row.jacobian_b_lin;
		}
	}
}

// Form a dynamic endpoint's provisional seed at current mobility without changing its physical input or wake state.
inline bool ConstraintStageRigidSeed(GpuRigidBody live, GpuArticulationSpatialVector previous, GpuArticulationSpatialVector seed, GpuConstraintAcceptance accepted, GpuConstraintEpoch epoch, out_(GpuRigidBody) staged)
{
	// Recover primary momentum, not velocity transformed through old or new inverse inertia.
	staged = live;
	for (int component = 0; component != 3; ++component)
	{
		// Seed admission is local; a failure leaves publication to the complete block owner.
		float free_angular;
		float free_translation;
		if (!ConstraintStageFreeComponent(live.momentum_ang[component], previous.ang[component], accepted, epoch, free_angular) ||
			!ConstraintStageFreeComponent(live.momentum_lin[component], previous.lin[component], accepted, epoch, free_translation))
			return false;

		// These working values have no sleep-state effects and never enter the response record on their own.
		staged.momentum_ang[component] = free_angular + seed.ang[component];
		staged.momentum_lin[component] = free_translation + seed.lin[component];
		if (!isfinite(staged.momentum_ang[component]) || !isfinite(staged.momentum_lin[component]))
			return false;
	}
	return true;
}

// Stage one ordinary block solve at prepared geometry and mobility. Inputs require a valid active-row inverse and distinct stable endpoints.
// The returned bodies, rows and record are provisional until Accepted.
inline uint ConstraintStageRigidBlock(GpuConstraintBlock block, GpuConstraintSolveCache inverse, GpuConstraintRow input_rows[GpuConstraintRowsPerBlock], GpuRigidBody live_a, GpuRigidBody live_b, GpuConstraintRigidRecord previous, GpuConstraintEpoch epoch, float relaxation, out_(GpuRigidBody) staged_a, out_(GpuRigidBody) staged_b, out_(GpuConstraintRigidRecord) record)
{
	// Invalid context is an input failure, not the non-terminal rejection of a physically unrepresentable replacement.
	staged_a = live_a;
	staged_b = live_b;
	record = previous;
	float scale;
	if (!ConstraintStageGuessScale(previous.accepted, epoch, scale))
		return GpuConstraintStage_NumericalFailure;

	// Valid finite inputs can overflow the derived provisional scale without invalidating the context or accepted cache.
	if (!isfinite(scale))
		return GpuConstraintStage_AdmissionRejected;

	// Guesses inherit only compatible accepted coefficients; prepared rows are not themselves an acceptance ledger.
	GpuConstraintRow rows[GpuConstraintRowsPerBlock];
	for (uint axis = 0; axis != GpuConstraintRowsPerBlock; ++axis)
	{
		// Never modify the caller's row storage while deriving a rejectable candidate.
		rows[axis] = input_rows[axis];
		rows[axis].bounds.z = ConstraintRowGuess(previous.rows, axis, block.velocity_mask, block.row_states, block.row_identities, rows[axis].bounds.xy, scale);
	}
	bool dynamic_a = block.body_idx_a >= 0 && live_a.os_com_and_invmass.w > 0.0f && !AllSet(live_a.state_flags, ERigidBodyStateFlags_Static);
	bool dynamic_b = block.body_idx_b >= 0 && live_b.os_com_and_invmass.w > 0.0f && !AllSet(live_b.state_flags, ERigidBodyStateFlags_Static);
	GpuArticulationSpatialVector response_a;
	GpuArticulationSpatialVector response_b;
	ConstraintRigidRowResponses(block, rows, dynamic_a, dynamic_b, response_a, response_b);

	// The previous response belongs to its recorded primary-state components. New J and mobility act only on provisional new impulses.
	GpuRigidBody seed_a = live_a;
	GpuRigidBody seed_b = live_b;
	if (dynamic_a && !ConstraintStageRigidSeed(live_a, previous.response_a, response_a, previous.accepted, epoch, seed_a))
		return GpuConstraintStage_AdmissionRejected;

	if (dynamic_b && !ConstraintStageRigidSeed(live_b, previous.response_b, response_b, previous.accepted, epoch, seed_b))
		return GpuConstraintStage_AdmissionRejected;

	// Projection consumes the same numerical block step as the production rigid solver, but reads the detached seed.
	BodyTwist twist_a = BodyTwistWS(seed_a);
	BodyTwist twist_b = BodyTwistWS(seed_b);
	for (int component = 0; component != 3; ++component)
	{
		// A finite provisional momentum can still have an unrepresentable physical speed at the current mobility.
		if (!isfinite(twist_a.ang[component]) || !isfinite(twist_a.lin[component]) || !isfinite(twist_b.ang[component]) || !isfinite(twist_b.lin[component]))
			return GpuConstraintStage_AdmissionRejected;
	}
	float residual[6];
	float impulses[6];
	float lower[6];
	float upper[6];
	float projected[6];
	for (int row_index = 0; row_index != inverse.row_count; ++row_index)
	{
		// Prepared canonical indices and inverse entries are owned and validated by the normal response compiler.
		uint axis = (inverse.active_axes >> (4u * (uint)row_index)) & 15u;
		GpuConstraintRow row = rows[axis];
		residual[row_index] =
			dot(row.jacobian_a_ang.xyz, twist_a.ang) + dot(row.jacobian_a_lin.xyz, twist_a.lin) +
			dot(row.jacobian_b_ang.xyz, twist_b.ang) + dot(row.jacobian_b_lin.xyz, twist_b.lin) -
			row.solve.y + row.solve.z + row.solve.w * row.bounds.z;
		impulses[row_index] = row.bounds.z;
		lower[row_index] = row.bounds.x;
		upper[row_index] = row.bounds.y;
	}
	if (inverse.row_count != 0 && !ProjectConstraintVelocityStep(inverse.row_count, inverse.inverse, residual, impulses, lower, upper, relaxation, projected))
		return GpuConstraintStage_NumericalFailure;

	// Zero active rows still accept an empty cache and release this owner's current-epoch response.
	for (int row_index = 0; row_index != inverse.row_count; ++row_index)
	{
		// Total projected coefficients define the new impulse, not an interpretation of the old response record.
		uint axis = (inverse.active_axes >> (4u * (uint)row_index)) & 15u;
		rows[axis].bounds.z = projected[row_index];
	}
	if (!ConstraintStageAcceptedRows(block, rows, record.rows))
		return GpuConstraintStage_AdmissionRejected;

	// Both endpoints, accepted rows and provenance stay provisional through the final complete-block gate.
	ConstraintRigidRowResponses(block, rows, dynamic_a, dynamic_b, response_a, response_b);
	record.response_a.ang = record.response_a.lin = float4(0, 0, 0, 0);
	record.response_b = record.response_a;
	if (dynamic_a && !ConstraintStageRigidEndpoint(live_a, previous.response_a, response_a, previous.accepted, epoch, staged_a, record.response_a))
		return GpuConstraintStage_AdmissionRejected;

	if (dynamic_b && !ConstraintStageRigidEndpoint(live_b, previous.response_b, response_b, previous.accepted, epoch, staged_b, record.response_b))
		return GpuConstraintStage_AdmissionRejected;

	// A finite proposed momentum is insufficient when current inverse inertia makes its physical velocity nonfinite.
	twist_a = BodyTwistWS(staged_a);
	twist_b = BodyTwistWS(staged_b);
	for (int component = 0; component != 3; ++component)
	{
		// Scalar checks are shared with the native HLSL replay.
		if (!isfinite(twist_a.ang[component]) || !isfinite(twist_a.lin[component]) || !isfinite(twist_b.ang[component]) || !isfinite(twist_b.lin[component]))
			return GpuConstraintStage_AdmissionRejected;
	}

	record.accepted = ConstraintAcceptance(epoch);
	return GpuConstraintStage_Accepted;
}

#ifdef __cplusplus
}
#endif
#endif
