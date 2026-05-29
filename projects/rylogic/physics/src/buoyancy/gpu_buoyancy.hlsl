//************************************
// Physics Engine
//  Copyright (c) Rylogic Ltd 2026
//************************************
// Sine-wave water buoyancy pass for generated box hulls.

#include "pr/hlsl/core.hlsli"
#include "pr/hlsl/interop.hlsli"
#include "pr/hlsl/vector.hlsli"
#include "pr/hlsl/spatial_algebra.hlsli"
#include "physics/src/compute/physics_types.hlsli"

#define BUOYANCY_COLUMN_THREAD_COUNT 256
#define BUOYANCY_REDUCE_THREAD_COUNT 128

struct CBufGpuBuoyancy
{
	int hull_count;
	int groups_per_hull;
	int total_columns;
	int grid_x;
	int grid_y;
	int wave_count;
	float time_s;
	float water_level;
	float fluid_density;
	float drag_coefficient;
	float2 pad0;
};

struct GpuBuoyancyHull
{
	int body_index;
	float3 half_extents;
};

struct GpuBuoyancyWave
{
	float4 direction_wavelength_phase_speed;
	float4 amplitude;
};

struct GpuBuoyancyPartial
{
	float4 force_ws;
	float4 torque_ws;
	float4 moment_ws_volume;
};

struct GpuBuoyancyDiagnostic
{
	float4 force_ws;
	float4 torque_ws;
	float4 centre_buoyancy_ws;
	float4 moment_ws_volume;
	int body_index;
	int valid;
	float volume_m3;
	float pad0;
};

ConstantBuffer<CBufGpuBuoyancy> resource(g, b0);
RWStructuredBuffer<GpuRigidBody> resource(g_bodies, u0);
StructuredBuffer<GpuBuoyancyHull> resource(g_hulls, t0);
StructuredBuffer<GpuBuoyancyWave> resource(g_waves, t1);
RWStructuredBuffer<GpuBuoyancyPartial> resource(g_partials, u1);
RWStructuredBuffer<GpuBuoyancyDiagnostic> resource(g_diagnostics, u2);

groupshared float4 s_force_ws[BUOYANCY_COLUMN_THREAD_COUNT];
groupshared float4 s_torque_ws[BUOYANCY_COLUMN_THREAD_COUNT];
groupshared float4 s_moment_ws_volume[BUOYANCY_COLUMN_THREAD_COUNT];

// Expand the world XY bounds for the projected generated box hull.
void IncludeBoxCorner(GpuRigidBody body, float3 half_extents, float3 sign, inout float2 min_xy, inout float2 max_xy)
{
	float3 corner_ws = mul(float4(half_extents * sign, 1.0f), body.o2w).xyz;
	min_xy = min(min_xy, corner_ws.xy);
	max_xy = max(max_xy, corner_ws.xy);
}

// Evaluate the scene-described water height at a world-space XY position.
float EvaluateWaterHeight(float2 xy_ws)
{
	float height = g.water_level;
	for (int wave_index = 0; wave_index != g.wave_count; ++wave_index)
	{
		GpuBuoyancyWave wave = g_waves[wave_index];
		float2 direction = wave.direction_wavelength_phase_speed.xy;
		float wavelength = wave.direction_wavelength_phase_speed.z;
		float phase_speed = wave.direction_wavelength_phase_speed.w;
		float amplitude = wave.amplitude.x;
		float phase = dot(direction, xy_ws) * tau / wavelength + phase_speed * g.time_s;
		height += amplitude * sin(phase);
	}
	return height;
}

// Evaluate the scene-described water height and its XY gradient at a world-space XY position.
// Returns float3(height, dh/dx, dh/dy). Sharing the sin/cos work between the height and gradient
// keeps the per-column overhead small. The gradient is used by EvaluateColumn to compute the
// lateral (wave-slope) component of the hydrostatic force via the divergence theorem:
//   F_buoy = rho * |g| * V_submerged * (-dh/dx, -dh/dy, +1)
// which reduces to the purely vertical model on flat water.
float3 EvaluateWaterHeightAndGradient(float2 xy_ws)
{
	float height = g.water_level;
	float2 gradient = float2(0.0f, 0.0f);
	for (int wave_index = 0; wave_index != g.wave_count; ++wave_index)
	{
		GpuBuoyancyWave wave = g_waves[wave_index];
		float2 direction = wave.direction_wavelength_phase_speed.xy;
		float wavelength = wave.direction_wavelength_phase_speed.z;
		float phase_speed = wave.direction_wavelength_phase_speed.w;
		float amplitude = wave.amplitude.x;
		float k = tau / wavelength;
		float phase = dot(direction, xy_ws) * k + phase_speed * g.time_s;
		float s, c;
		sincos(phase, s, c);
		height += amplitude * s;
		gradient += direction * (amplitude * k * c);
	}
	return float3(height, gradient.x, gradient.y);
}

// Project the generated box hull onto the horizontal integration plane.
void ProjectBoxBoundsXY(GpuRigidBody body, float3 half_extents, out float2 min_xy, out float2 max_xy)
{
	min_xy = float2(1.0e20f, 1.0e20f);
	max_xy = float2(-1.0e20f, -1.0e20f);

	IncludeBoxCorner(body, half_extents, float3(-1.0f, -1.0f, -1.0f), min_xy, max_xy);
	IncludeBoxCorner(body, half_extents, float3(+1.0f, -1.0f, -1.0f), min_xy, max_xy);
	IncludeBoxCorner(body, half_extents, float3(-1.0f, +1.0f, -1.0f), min_xy, max_xy);
	IncludeBoxCorner(body, half_extents, float3(+1.0f, +1.0f, -1.0f), min_xy, max_xy);
	IncludeBoxCorner(body, half_extents, float3(-1.0f, -1.0f, +1.0f), min_xy, max_xy);
	IncludeBoxCorner(body, half_extents, float3(+1.0f, -1.0f, +1.0f), min_xy, max_xy);
	IncludeBoxCorner(body, half_extents, float3(-1.0f, +1.0f, +1.0f), min_xy, max_xy);
	IncludeBoxCorner(body, half_extents, float3(+1.0f, +1.0f, +1.0f), min_xy, max_xy);
}

// Clip a column ray against one object-space box slab.
bool IntersectBoxAxis(float origin_os, float direction_os, float half_extent, inout float t_min, inout float t_max)
{
	if (abs(direction_os) < 1.0e-6f)
	{
		return origin_os >= -half_extent && origin_os <= half_extent;
	}

	float t0 = (-half_extent - origin_os) / direction_os;
	float t1 = (+half_extent - origin_os) / direction_os;
	if (t0 > t1)
	{
		float tmp = t0;
		t0 = t1;
		t1 = tmp;
	}

	t_min = max(t_min, t0);
	t_max = min(t_max, t1);
	return t_min <= t_max;
}

// Intersect a vertical world-space integration column with the generated box hull.
bool IntersectBoxColumn(GpuRigidBody body, float3 half_extents, float2 xy_ws, out float t_min, out float t_max)
{
	float4x4 w2o = InvertOrthonormal(body.o2w);
	float3 origin_os = mul(float4(xy_ws, 0.0f, 1.0f), w2o).xyz;
	float3 direction_os = mul(float4(0.0f, 0.0f, 1.0f, 0.0f), w2o).xyz;

	t_min = -1.0e20f;
	t_max = +1.0e20f;
	if (!IntersectBoxAxis(origin_os.x, direction_os.x, half_extents.x, t_min, t_max))
	{
		return false;
	}
	if (!IntersectBoxAxis(origin_os.y, direction_os.y, half_extents.y, t_min, t_max))
	{
		return false;
	}
	if (!IntersectBoxAxis(origin_os.z, direction_os.z, half_extents.z, t_min, t_max))
	{
		return false;
	}

	return t_max > t_min;
}

// Evaluate one grid column and return its force, torque, and volume-weighted centroid contribution.
// The hydrostatic force per column uses the divergence theorem applied to the instantaneous water
// surface h(xy,t): F = rho * |g| * V_col * (-dh/dx, -dh/dy, +1). The vertical component reduces to
// the standard flat-water buoyancy; the horizontal components carry wave-slope (Froude-Krylov)
// forces that drive heave, drift and pitching from non-flat surfaces. A small linear viscous drag
// is added per column (proportional to submerged volume) so wave-driven motion does not resonate
// unboundedly. Both forces share the same centroid for torque accumulation.
void EvaluateColumn(GpuRigidBody body, GpuBuoyancyHull hull, int column_index, out float4 force_ws, out float4 torque_ws, out float4 moment_ws_volume)
{
	force_ws = float4(0.0f, 0.0f, 0.0f, 0.0f);
	torque_ws = float4(0.0f, 0.0f, 0.0f, 0.0f);
	moment_ws_volume = float4(0.0f, 0.0f, 0.0f, 0.0f);
	if ((body.state_flags & ERigidBodyStateFlags_Static) != 0 || body.os_com_and_invmass.w <= 0.0f)
	{
		return;
	}

	float2 min_xy;
	float2 max_xy;
	ProjectBoxBoundsXY(body, hull.half_extents, min_xy, max_xy);

	float2 span_xy = max_xy - min_xy;
	if (span_xy.x <= 1.0e-6f || span_xy.y <= 1.0e-6f)
	{
		return;
	}

	// Columns stay vertical for the incremental model; the surface height can vary per XY sample.
	int cell_x = column_index % g.grid_x;
	int cell_y = column_index / g.grid_x;
	float2 cell_size = span_xy / float2(g.grid_x, g.grid_y);
	float2 xy_ws = min_xy + (float2(cell_x, cell_y) + 0.5f) * cell_size;

	float t_min;
	float t_max;
	if (!IntersectBoxColumn(body, hull.half_extents, xy_ws, t_min, t_max))
	{
		return;
	}

	float3 height_and_grad = EvaluateWaterHeightAndGradient(xy_ws);
	float water_height = height_and_grad.x;
	float2 surface_grad = height_and_grad.yz;

	float submerged_t_min = t_min;
	float submerged_t_max = min(t_max, water_height);
	if (submerged_t_max <= submerged_t_min)
	{
		return;
	}

	float volume = (submerged_t_max - submerged_t_min) * cell_size.x * cell_size.y;
	float3 centroid_ws = float3(xy_ws, 0.5f * (submerged_t_min + submerged_t_max));
	float3 centre_of_mass_ws = mul(float4(body.os_com_and_invmass.xyz, 1.0f), body.o2w).xyz;

	// Hydrostatic force from the body's own local gravity sample (each rigid body carries a
	// world-space gravity vector that the engine treats as a sample of the local gravity field):
	//   F_buoy = rho * V * (-g_local) + rho * V * |g_local| * (-dh/dx, -dh/dy, 0)
	// The first term is Archimedes' principle in vector form (buoyancy directly opposes the
	// body's gravity); the second term is the wave-slope (Froude-Krylov) correction obtained
	// from the divergence theorem applied to the instantaneous water surface h(x,y,t). The
	// surface itself is parameterised in world XY with world Z as "up", so the slope correction
	// uses the world-frame gradient; the formulation is exact when the local gravity is along
	// -Z and remains a sensible first-order approximation for mild departures from that axis.
	// On flat water the slope term vanishes and the expression reduces to F = -rho * V * g_local.
	float3 gravity_ws = body.ws_gravity.xyz;
	float gravity_magnitude = length(gravity_ws);
	float rho_v = g.fluid_density * volume;
	float3 force_buoyancy = rho_v * (-gravity_ws + gravity_magnitude * float3(-surface_grad.x, -surface_grad.y, 0.0f));

	// Linear viscous drag at the column centroid. Computing the rigid-body velocity at the column
	// centroid means we damp linear and angular motion in one pass; the cross-product torque
	// accumulator picks up the angular contribution automatically.
	float inv_mass = body.os_com_and_invmass.w;
	float3x3 os_iinv = inv_mass * build_symmetric_3x3(body.inertia_inv_diagonal.xyz, body.inertia_inv_products.xyz);
	float3x3 ws_iinv = rotate_inertia_inv(os_iinv, (float3x3)body.o2w);
	float3 v_lin = inv_mass * body.momentum_lin.xyz;
	float3 omega_ws = mul(ws_iinv, body.momentum_ang.xyz);
	float3 v_at_centroid = v_lin + cross(omega_ws, centroid_ws - centre_of_mass_ws);
	float3 force_drag = -(g.drag_coefficient * volume) * v_at_centroid;

	float3 force = force_buoyancy + force_drag;
	float3 torque = cross(centroid_ws - centre_of_mass_ws, force);

	force_ws = float4(force, 0.0f);
	torque_ws = float4(torque, 0.0f);
	moment_ws_volume = float4(centroid_ws * volume, volume);
}

// Sum all entries in the shared reduction arrays.
void ReduceShared(uint thread_index, uint thread_count)
{
	for (uint stride = thread_count >> 1; stride != 0; stride >>= 1)
	{
		GroupMemoryBarrierWithGroupSync();
		if (thread_index < stride)
		{
			s_force_ws[thread_index] += s_force_ws[thread_index + stride];
			s_torque_ws[thread_index] += s_torque_ws[thread_index + stride];
			s_moment_ws_volume[thread_index] += s_moment_ws_volume[thread_index + stride];
		}
	}
	GroupMemoryBarrierWithGroupSync();
}

// Evaluate a block of buoyancy columns and write one partial record for the reducer.
numthreads(CSGpuBuoyancyColumns, BUOYANCY_COLUMN_THREAD_COUNT, 1, 1)
void CSGpuBuoyancyColumns(uint3 DTID(dispatch_thread_id), uint3 GID(group_id), uint3 GTID(group_thread_id))
{
	int global_group_index = int(group_id.x);
	int hull_index = global_group_index / g.groups_per_hull;
	int hull_group_index = global_group_index - hull_index * g.groups_per_hull;
	int thread_index = int(group_thread_id.x);
	int column_index = hull_group_index * BUOYANCY_COLUMN_THREAD_COUNT + thread_index;

	float4 force_ws = float4(0.0f, 0.0f, 0.0f, 0.0f);
	float4 torque_ws = float4(0.0f, 0.0f, 0.0f, 0.0f);
	float4 moment_ws_volume = float4(0.0f, 0.0f, 0.0f, 0.0f);
	if (hull_index < g.hull_count && column_index < g.total_columns)
	{
		GpuBuoyancyHull hull = g_hulls[hull_index];
		GpuRigidBody body = g_bodies[hull.body_index];
		EvaluateColumn(body, hull, column_index, force_ws, torque_ws, moment_ws_volume);
	}

	s_force_ws[thread_index] = force_ws;
	s_torque_ws[thread_index] = torque_ws;
	s_moment_ws_volume[thread_index] = moment_ws_volume;
	ReduceShared(uint(thread_index), BUOYANCY_COLUMN_THREAD_COUNT);

	if (thread_index == 0)
	{
		g_partials[global_group_index].force_ws = s_force_ws[0];
		g_partials[global_group_index].torque_ws = s_torque_ws[0];
		g_partials[global_group_index].moment_ws_volume = s_moment_ws_volume[0];
	}
}

// Reduce all partials for one hull, add the force/torque to the body accumulator, and write a diagnostic record.
numthreads(CSGpuBuoyancyReduce, BUOYANCY_REDUCE_THREAD_COUNT, 1, 1)
void CSGpuBuoyancyReduce(uint3 GID(group_id), uint3 GTID(group_thread_id))
{
	int hull_index = int(group_id.x);
	int thread_index = int(group_thread_id.x);
	int partial_index = hull_index * g.groups_per_hull + thread_index;

	float4 force_ws = float4(0.0f, 0.0f, 0.0f, 0.0f);
	float4 torque_ws = float4(0.0f, 0.0f, 0.0f, 0.0f);
	float4 moment_ws_volume = float4(0.0f, 0.0f, 0.0f, 0.0f);
	if (hull_index < g.hull_count && thread_index < g.groups_per_hull)
	{
		GpuBuoyancyPartial partial = g_partials[partial_index];
		force_ws = partial.force_ws;
		torque_ws = partial.torque_ws;
		moment_ws_volume = partial.moment_ws_volume;
	}

	s_force_ws[thread_index] = force_ws;
	s_torque_ws[thread_index] = torque_ws;
	s_moment_ws_volume[thread_index] = moment_ws_volume;
	ReduceShared(uint(thread_index), BUOYANCY_REDUCE_THREAD_COUNT);

	if (thread_index == 0 && hull_index < g.hull_count)
	{
		float volume = s_moment_ws_volume[0].w;
		float has_volume = volume > 1.0e-6f ? 1.0f : 0.0f;
		float3 centre_buoyancy_ws = has_volume != 0.0f ? s_moment_ws_volume[0].xyz / volume : float3(0.0f, 0.0f, 0.0f);

		GpuBuoyancyHull hull = g_hulls[hull_index];
		GpuRigidBody body = g_bodies[hull.body_index];
		body.force_lin += s_force_ws[0];
		body.force_ang += s_torque_ws[0];
		g_bodies[hull.body_index] = body;

		g_diagnostics[hull_index].force_ws = s_force_ws[0];
		g_diagnostics[hull_index].torque_ws = s_torque_ws[0];
		g_diagnostics[hull_index].centre_buoyancy_ws = float4(centre_buoyancy_ws, has_volume);
		g_diagnostics[hull_index].moment_ws_volume = s_moment_ws_volume[0];
		g_diagnostics[hull_index].body_index = hull.body_index;
		g_diagnostics[hull_index].valid = 1;
		g_diagnostics[hull_index].volume_m3 = volume;
		g_diagnostics[hull_index].pad0 = 0.0f;
	}
}
