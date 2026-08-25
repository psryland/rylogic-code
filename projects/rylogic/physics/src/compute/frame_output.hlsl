//*********************************************
// Physics Engine — Gathered GPU Frame Output
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#include "pr/hlsl/core.hlsli"
#include "pr/hlsl/interop.hlsli"
#include "physics/src/compute/physics_types.hlsli"

#ifdef __cplusplus
namespace pr::physics {
#endif

// Per-dispatch bounds and frame/substep identity.
struct cbFrameOutput
{
	int max_pairs;
	int max_contacts;
	int event_capacity;
	int collect_events;
	int substep_index;
	int substep_count;
	int body_count;
	int pad0;
	int articulation_count;
	int position_count;
	int velocity_count;
	int pad1;
};

ConstantBuffer<cbFrameOutput> resource(g, b0);
RWStructuredBuffer<GpuCollisionCounters> resource(g_counters, u0);
RWStructuredBuffer<GpuResolveContact> resource(g_contacts, u1);
RWStructuredBuffer<uint> resource(g_contact_order, u2);
RWStructuredBuffer<GpuFrameOutputHeader> resource(g_header, u3);
RWStructuredBuffer<GpuSubstepOutputState> resource(g_substep_state, u4);
RWStructuredBuffer<GpuCollisionEvent> resource(g_events, u5);
RWStructuredBuffer<GpuRigidBody> resource(g_bodies, u6);
RWStructuredBuffer<GpuRigidBody> resource(g_output_bodies, u7);
RWStructuredBuffer<GpuArticulation> resource(g_articulations, u8);
RWStructuredBuffer<GpuArticulationIntegrationState> resource(g_articulation_states, u9);
RWStructuredBuffer<float> resource(g_articulation_positions, u10);
RWStructuredBuffer<float> resource(g_articulation_velocities, u11);
RWStructuredBuffer<float> resource(g_articulation_accelerations, u12);
RWStructuredBuffer<GpuArticulationFrameOutput> resource(g_output_articulations, u13);
RWStructuredBuffer<float> resource(g_output_positions, u14);
RWStructuredBuffer<float> resource(g_output_velocities, u15);
RWStructuredBuffer<float> resource(g_output_accelerations, u16);

// Reserve a deterministic contiguous event range and retain capacity diagnostics before counters are reset.
numthreads(CSPrepareSubstepOutput, 1, 1, 1)
void CSPrepareSubstepOutput(int3 DTID(dtid))
{
	GpuCollisionCounters counters = g_counters[0];
	GpuFrameOutputHeader header = g_header[0];
	int pair_count = max(counters.pair_count, 0);
	int contact_count = max(counters.contact_count, 0);
	int retained_contacts = min(contact_count, g.max_contacts);

	header.final_counters = counters;
	header.max_pair_count = max(header.max_pair_count, pair_count);
	header.max_contact_count = max(header.max_contact_count, contact_count);
	header.substep_count = g.substep_count;
	if (header.pair_limit_substep < 0 && g.max_pairs > 0 && pair_count >= g.max_pairs)
		header.pair_limit_substep = g.substep_index;
	if (header.contact_limit_substep < 0 && g.max_contacts > 0 && contact_count >= g.max_contacts)
		header.contact_limit_substep = g.substep_index;

	int event_base = header.event_count;
	int available = max(header.event_capacity - event_base, 0);
	int event_count = g.collect_events != 0 ? min(retained_contacts, available) : 0;
	if (g.collect_events != 0 && retained_contacts > available)
	{
		header.event_overflow = 1;
		if (header.event_overflow_substep < 0)
			header.event_overflow_substep = g.substep_index;
	}
	header.event_count = event_base + event_count;

	g_header[0] = header;
	g_substep_state[0] = (GpuSubstepOutputState)0;
	g_substep_state[0].event_base = event_base;
	g_substep_state[0].event_count = event_count;
	g_substep_state[0].substep_index = g.substep_index;
}

// Copy resolved contacts in their deterministic solver order into the frame-wide event stream.
numthreads(CSAppendCollisionEvents, FrameOutputThreadCount, 1, 1)
void CSAppendCollisionEvents(int3 DTID(dtid))
{
	GpuSubstepOutputState state = g_substep_state[0];
	int local_index = dtid.x;
	if (local_index >= state.event_count)
		return;

	int contact_index = (int)g_contact_order[local_index];
	if (contact_index < 0 || contact_index >= g.max_contacts)
		return;

	GpuCollisionEvent collision_event = (GpuCollisionEvent)0;
	collision_event.contact = g_contacts[contact_index];
	collision_event.substep_index = state.substep_index;
	g_events[state.event_base + local_index] = collision_event;
}

// Gather final body state into the contiguous output allocation after all substeps complete.
numthreads(CSGatherFrameBodies, FrameOutputThreadCount, 1, 1)
void CSGatherFrameBodies(int3 DTID(dtid))
{
	int body_index = dtid.x;
	if (body_index >= g.body_count)
		return;

	g_output_bodies[body_index] = g_bodies[body_index];
}

// Gather final generalized state and one compact identity/root/status record per articulation.
numthreads(CSGatherFrameArticulations, FrameOutputThreadCount, 1, 1)
void CSGatherFrameArticulations(int3 DTID(dtid))
{
	int index = dtid.x;
	if (index < g.articulation_count)
	{
		GpuArticulation articulation = g_articulations[index];
		GpuArticulationIntegrationState state = g_articulation_states[index];
		GpuArticulationFrameOutput output = (GpuArticulationFrameOutput)0;
		output.root_to_world = articulation.root_to_world;
		output.identity_low = articulation.identity_low;
		output.identity_high = articulation.identity_high;
		output.status = state.status;
		output.iteration_count = state.iteration_count;
		output.residual = state.residual;
		g_output_articulations[index] = output;
	}
	if (index < g.position_count)
		g_output_positions[index] = g_articulation_positions[index];
	if (index < g.velocity_count)
	{
		g_output_velocities[index] = g_articulation_velocities[index];
		g_output_accelerations[index] = g_articulation_accelerations[index];
	}
}

#ifdef __cplusplus
}
#endif
