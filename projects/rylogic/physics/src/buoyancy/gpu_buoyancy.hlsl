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
#include "physics/src/buoyancy/buoyancy_sampler.hlsli"

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
	float quadratic_drag_coefficient;
	float pad0;
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

// --- Sampled-composite backend (phase 10) volume-pass bindings ---
// Per-hull header for the composite volume pass: one record per registered composite hull. The
// header locates the hull's primitive block in g_prims / g_vol_prim_records and carries the stable
// per-hull sample seed and inside-test epsilon computed once at registration.
struct BuoyVolHeader
{
	int body_index;            // index into g_bodies
	int prim_base;             // start of this hull's primitives in g_prims / g_vol_prim_records
	int prim_count;            // number of convex primitives in this hull
	int total_volume_samples;  // actual emitted volume sample count (sum of per-primitive counts)
	uint hull_id;              // stable per-hull seed for the deterministic sample hash
	float eps;                 // scale-relative inside-test slack (sibling cull)
	int pad0;
	int pad1;
};

// Per-primitive volume-sample record, parallel to g_prims. Maps a flat sample index to its owning
// primitive (via the cumulative counts) and supplies that primitive's per-sample volume weight.
struct BuoyVolPrimRecord
{
	int count;    // volume samples allocated to this primitive
	float dvol;   // per-sample volume weight (= primitive_volume / count)
};

StructuredBuffer<BuoyVolHeader> resource(g_vol_headers, t2);
StructuredBuffer<BuoyPrimitive> resource(g_prims, t3);
StructuredBuffer<float4> resource(g_volume_verts, t4);
StructuredBuffer<int4> resource(g_tets, t5);
StructuredBuffer<float4> resource(g_face_planes, t6);
StructuredBuffer<BuoyVolPrimRecord> resource(g_vol_prim_records, t7);

// --- Sampled-composite backend (phase 12) surface-pass bindings ---
// Per-hull header for the composite surface (drag) pass: one record per registered composite hull.
// Layout-identical to BuoyVolHeader, but the sample-count field counts surface samples and the
// primitive block is indexed into g_surf_prim_records (parallel to g_prims) instead of the volume
// records. The body index, hull seed and inside-test epsilon are the same per-hull values.
struct BuoySurfHeader
{
	int body_index;             // index into g_bodies
	int prim_base;              // start of this hull's primitives in g_prims / g_surf_prim_records
	int prim_count;             // number of convex primitives in this hull
	int total_surface_samples;  // actual emitted surface sample count (sum of per-primitive counts)
	uint hull_id;               // stable per-hull seed for the deterministic sample hash
	float eps;                  // scale-relative inside-test slack (sibling cull)
	int pad0;
	int pad1;
};

// Per-primitive surface-sample record, parallel to g_prims. Maps a flat sample index to its owning
// primitive (via the cumulative counts) and supplies that primitive's per-sample area weight.
struct BuoySurfPrimRecord
{
	int count;    // surface samples allocated to this primitive
	float darea;  // per-sample area weight (= primitive_area / count)
};

StructuredBuffer<BuoySurfHeader> resource(g_surf_headers, t8);
StructuredBuffer<float4> resource(g_verts, t9);
StructuredBuffer<int4> resource(g_face_verts, t10);
StructuredBuffer<BuoySurfPrimRecord> resource(g_surf_prim_records, t11);

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

// Evaluate the scene-described water particle velocity (orbital flow) at a world-space position.
// Linear deep-water (Airy) theory, consistent with EvaluateWaterHeight:
//   u_along_d = -A*omega * e^(k*z) * sin(phase)   (horizontal, along the wave direction)
//   w_up      =  A*omega * e^(k*z) * cos(phase)   (vertical)
// where z is the signed height relative to the still-water level (clamped <= 0). The drag pass
// subtracts this from the body velocity to form the relative flow at each wetted surface sample.
float3 EvaluateWaterVelocity(float3 pos_ws)
{
	float depth = min(pos_ws.z - g.water_level, 0.0f);
	float3 velocity = float3(0.0f, 0.0f, 0.0f);
	for (int wave_index = 0; wave_index != g.wave_count; ++wave_index)
	{
		GpuBuoyancyWave wave = g_waves[wave_index];
		float2 direction = wave.direction_wavelength_phase_speed.xy;
		float wavelength = wave.direction_wavelength_phase_speed.z;
		float phase_speed = wave.direction_wavelength_phase_speed.w;
		float amplitude = wave.amplitude.x;
		float k = tau / wavelength;
		float phase = dot(direction, pos_ws.xy) * k + phase_speed * g.time_s;
		float s, c;
		sincos(phase, s, c);
		float speed = amplitude * phase_speed * exp(k * depth);
		velocity.xy += (-speed * s) * direction;
		velocity.z += speed * c;
	}
	return velocity;
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

// Per-face quadratic (form) drag for the registered box hull. Writes the total drag force and torque
// about the body's centre of mass in world space, integrated over a 2x2 sub-sample grid on each face.
// Sub-sampling (rather than evaluating only at face centroids) is necessary to capture rotational
// drag: for pure yaw, omega x r is purely tangent to a side face at its centre, so v_n = 0 and the
// centroid evaluation produces no torque. The 2x2 grid restores rotational damping cheaply (~24
// samples per body). Quadratic drag complements the per-column linear drag in EvaluateColumn:
// linear dominates near rest and stabilises low-frequency motion, quadratic dominates at speed and
// provides realistic form drag for translating/tumbling bodies.
//
// The model assumes stationary water (v_fluid = 0). The expression is structured as
// v_rel = v_point - v_fluid so wave-orbital velocity can be added later without restructuring.
// Drag only acts on faces whose outward normal has a positive velocity component into the water
// (the face is moving outward into the fluid); leeward faces contribute nothing. This matches the
// form-drag model for separated flow at moderate-to-high Reynolds numbers.
void EvaluatePerFaceQuadraticDrag(GpuBuoyancyHull hull, GpuRigidBody body, out float3 drag_force_ws, out float3 drag_torque_ws)
{
	drag_force_ws = float3(0.0f, 0.0f, 0.0f);
	drag_torque_ws = float3(0.0f, 0.0f, 0.0f);
	if (g.quadratic_drag_coefficient <= 0.0f)
	{
		return;
	}

	// Body kinematics at this instant. See the note in EvaluateColumn about the storage convention
	// for inertia_inv_* (mass-removed shape inertia, recombined here via the inv_mass scaling).
	float inv_mass = body.os_com_and_invmass.w;
	float3x3 os_iinv = inv_mass * build_symmetric_3x3(body.inertia_inv_diagonal.xyz, body.inertia_inv_products.xyz);
	float3x3 ws_iinv = rotate_inertia_inv(os_iinv, (float3x3)body.o2w);
	float3 v_lin = inv_mass * body.momentum_lin.xyz;
	float3 omega_ws = mul(ws_iinv, body.momentum_ang.xyz);
	float3 com_ws = mul(float4(body.os_com_and_invmass.xyz, 1.0f), body.o2w).xyz;

	// 0.5 * rho * Cd is the constant prefactor shared by every face sample.
	float coef = 0.5f * g.fluid_density * g.quadratic_drag_coefficient;

	// Iterate the 6 axis-aligned faces of the generated box hull. For each face the outward normal
	// is a unit axis (positive then negative along x, y, z), and the two in-face tangent axes are
	// the other two world axes; the half-widths along those tangents come from the matching
	// components of hull.half_extents. The hull geometry is centred on the body's model origin
	// (same assumption as ProjectBoxBoundsXY), so the face centres in object-space are simply
	// n_os * half_extent_along_normal.
	for (int f = 0; f != 6; ++f)
	{
		int axis = f >> 1;
		float sign_n = (f & 1) != 0 ? -1.0f : +1.0f;

		float3 n_os = float3(0.0f, 0.0f, 0.0f);
		float3 tu_os = float3(0.0f, 0.0f, 0.0f);
		float3 tv_os = float3(0.0f, 0.0f, 0.0f);
		float h_n = 0.0f;
		float hu = 0.0f;
		float hv = 0.0f;
		if (axis == 0)
		{
			n_os.x = sign_n; tu_os.y = 1.0f; tv_os.z = 1.0f;
			h_n = hull.half_extents.x; hu = hull.half_extents.y; hv = hull.half_extents.z;
		}
		else if (axis == 1)
		{
			n_os.y = sign_n; tu_os.x = 1.0f; tv_os.z = 1.0f;
			h_n = hull.half_extents.y; hu = hull.half_extents.x; hv = hull.half_extents.z;
		}
		else
		{
			n_os.z = sign_n; tu_os.x = 1.0f; tv_os.y = 1.0f;
			h_n = hull.half_extents.z; hu = hull.half_extents.x; hv = hull.half_extents.y;
		}

		// Quarter of the face area, shared by all four sub-samples.
		float sample_area = hu * hv;
		float3 face_centre_os = n_os * h_n;
		float3 n_ws = mul(float4(n_os, 0.0f), body.o2w).xyz;

		// 2x2 sub-sample grid: each sample is the centroid of one quadrant of the face,
		// at offsets +/-hu/2 along tangent u and +/-hv/2 along tangent v.
		[unroll] for (int si = 0; si != 2; ++si)
		[unroll] for (int sj = 0; sj != 2; ++sj)
		{
			float ofs_u = (si == 0 ? -0.5f : +0.5f) * hu;
			float ofs_v = (sj == 0 ? -0.5f : +0.5f) * hv;
			float3 sample_os = face_centre_os + tu_os * ofs_u + tv_os * ofs_v;
			float3 sample_ws = mul(float4(sample_os, 1.0f), body.o2w).xyz;

			// Submergence test: skip samples above the local water surface.
			float water_h = EvaluateWaterHeight(sample_ws.xy);
			if (sample_ws.z >= water_h)
			{
				continue;
			}

			// Form drag opposes the outward motion of the face. v_fluid = 0 for stationary water;
			// when wave-orbital velocity is added later it will be subtracted from v_at_sample.
			float3 v_at_sample = v_lin + cross(omega_ws, sample_ws - com_ws);
			float v_n = dot(v_at_sample, n_ws);
			if (v_n <= 0.0f)
			{
				continue;
			}

			float3 force = -(coef * sample_area * v_n * v_n) * n_ws;
			float3 torque = cross(sample_ws - com_ws, force);
			drag_force_ws += force;
			drag_torque_ws += torque;
		}
	}
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

		// Per-face quadratic (form) drag is added on top of the per-column buoyancy + linear drag
		// already reduced in s_force_ws[0] / s_torque_ws[0]. Doing the per-face evaluation here in
		// the reduce kernel (thread 0 only, once per body) keeps it cheap and avoids fan-out into
		// the columns dispatch where it would be replicated 128 times per body.
		float3 quadratic_drag_force_ws = float3(0.0f, 0.0f, 0.0f);
		float3 quadratic_drag_torque_ws = float3(0.0f, 0.0f, 0.0f);
		EvaluatePerFaceQuadraticDrag(hull, body, quadratic_drag_force_ws, quadratic_drag_torque_ws);

		float4 total_force_ws = s_force_ws[0] + float4(quadratic_drag_force_ws, 0.0f);
		float4 total_torque_ws = s_torque_ws[0] + float4(quadratic_drag_torque_ws, 0.0f);

		body.force_lin += total_force_ws;
		body.force_ang += total_torque_ws;
		g_bodies[hull.body_index] = body;

		g_diagnostics[hull_index].force_ws = total_force_ws;
		g_diagnostics[hull_index].torque_ws = total_torque_ws;
		g_diagnostics[hull_index].centre_buoyancy_ws = float4(centre_buoyancy_ws, has_volume);
		g_diagnostics[hull_index].moment_ws_volume = s_moment_ws_volume[0];
		g_diagnostics[hull_index].body_index = hull.body_index;
		g_diagnostics[hull_index].valid = 1;
		g_diagnostics[hull_index].volume_m3 = volume;
		g_diagnostics[hull_index].pad0 = 0.0f;
	}
}

// Evaluate a block of composite-hull volume samples (sampled-composite backend) and write one
// partial record for the reducer. Each in-fluid sample contributes a Froude-Krylov pressure-gradient
// force dF = rho*|g|*dV*(up - grad_ws) plus a per-sample torque about the body's centre of mass, and
// a weighted moment (sample_ws*dV, dV) used to recover wet volume + centre of buoyancy. Overlap
// regions are counted once via the lowest-index-sibling cull, so the union volume of arbitrary
// overlapping convex primitives is unbiased. This mirrors the volume pass of the CPU oracle
// SampleHull (include/pr/physics/buoyancy/buoyancy_sampler.h).
//
// Sample indexing: groups are laid out [hull 0 groups][hull 1 groups]..., g.groups_per_hull groups
// per hull. The flat sample index within a hull selects a primitive by walking the per-primitive
// cumulative counts (g_vol_prim_records, parallel to g_prims); the residual is the primitive-local
// sample ordinal fed to the deterministic hash. Threads beyond a hull's emitted sample count, and
// samples on static / zero-mass bodies or under (near-)zero gravity, contribute zero.
numthreads(CSBuoyancyVolumeSamples, BUOYANCY_COLUMN_THREAD_COUNT, 1, 1)
void CSBuoyancyVolumeSamples(uint3 GID(group_id), uint3 GTID(group_thread_id))
{
	int global_group_index = int(group_id.x);
	int hull_index = global_group_index / g.groups_per_hull;
	int hull_group_index = global_group_index - hull_index * g.groups_per_hull;
	int thread_index = int(group_thread_id.x);
	int sample_index = hull_group_index * BUOYANCY_COLUMN_THREAD_COUNT + thread_index;

	float4 force_ws = float4(0.0f, 0.0f, 0.0f, 0.0f);
	float4 torque_ws = float4(0.0f, 0.0f, 0.0f, 0.0f);
	float4 moment_ws_volume = float4(0.0f, 0.0f, 0.0f, 0.0f);

	if (hull_index < g.hull_count)
	{
		BuoyVolHeader header = g_vol_headers[hull_index];
		if (sample_index < header.total_volume_samples)
		{
			GpuRigidBody body = g_bodies[header.body_index];

			// Static / zero-mass bodies have no buoyant response (mirrors EvaluateColumn's guard).
			bool dynamic = (body.state_flags & ERigidBodyStateFlags_Static) == 0 && body.os_com_and_invmass.w > 0.0f;

			// Gravity defines the buoyancy "up" axis; guard the normalise against (near-)zero gravity.
			float3 gravity_ws = body.ws_gravity.xyz;
			float g_mag = length(gravity_ws);

			if (dynamic && g_mag > BUOY_TINY)
			{
				// Map the flat sample index to its owning primitive k and primitive-local ordinal by
				// walking the per-primitive cumulative counts (parallel to g_prims).
				int k = 0;
				int local_i = sample_index;
				float dvol = 0.0f;
				bool found = false;
				for (int pp = 0; pp != header.prim_count; ++pp)
				{
					BuoyVolPrimRecord rec = g_vol_prim_records[header.prim_base + pp];
					if (local_i < rec.count)
					{
						k = pp;
						dvol = rec.dvol;
						found = true;
						break;
					}
					local_i -= rec.count;
				}

				if (found)
				{
					BuoyPrimitive prim = g_prims[header.prim_base + k];

					// Emit a volume sample in shape-local space, then lift to COM-root and world.
					float4 pos_local;
					float weight;
					BuoyEmitVolumeSample(prim, BuoySampleIndex(header.hull_id, k, local_i), dvol, g_volume_verts, g_tets, pos_local, weight);
					float3 p_root = mul(float4(pos_local.xyz, 1.0f), prim.m_s2r).xyz;

					// Lowest-index-sibling cull: a lower-index primitive owns any shared volume, so the
					// union volume is counted exactly once without bias.
					if (!BuoyIsInsideAnyLowerSibling(g_prims, header.prim_base, k, p_root, header.eps, g_face_planes))
					{
						float3 up = -gravity_ws / g_mag;
						float3 sample_ws = mul(float4(p_root, 1.0f), body.o2w).xyz;

						// Wet test along -gravity. The flat-ocean water field is parameterised in world
						// XY with height measured along world Z; for gravity along -Z (the phase-11 box
						// parity gate) this matches signed-height-along-up exactly.
						float signed_height = dot(sample_ws, up);
						float3 hg = EvaluateWaterHeightAndGradient(sample_ws.xy);
						if (signed_height < hg.x)
						{
							// Froude-Krylov pressure-gradient force per unit volume: up minus the
							// lifted water-surface slope. Reduces to rho*|g|*dV*(0,0,1) for flat water.
							float3 grad_ws = hg.y * float3(1.0f, 0.0f, 0.0f) + hg.z * float3(0.0f, 1.0f, 0.0f);
							float3 dF = (g.fluid_density * g_mag * weight) * (up - grad_ws);
							float3 com_ws = mul(float4(body.os_com_and_invmass.xyz, 1.0f), body.o2w).xyz;

							force_ws = float4(dF, 0.0f);
							torque_ws = float4(cross(sample_ws - com_ws, dF), 0.0f);
							moment_ws_volume = float4(sample_ws * weight, weight);
						}
					}
				}
			}
		}
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

// Reduce all volume partials for one composite hull, add the buoyancy force/torque to the body
// accumulator, and write a diagnostic record. This mirrors CSGpuBuoyancyReduce but without the
// per-face quadratic-drag block (drag is a later phase) and reads the hull's body via the volume
// header rather than the legacy box-hull record.
numthreads(CSBuoyancyVolumeReduce, BUOYANCY_REDUCE_THREAD_COUNT, 1, 1)
void CSBuoyancyVolumeReduce(uint3 GID(group_id), uint3 GTID(group_thread_id))
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
		BuoyVolHeader header = g_vol_headers[hull_index];

		// Wet volume + centre of buoyancy recovered from the weighted moment sum.
		float volume = s_moment_ws_volume[0].w;
		float has_volume = volume > BUOY_TINY ? 1.0f : 0.0f;
		float3 centre_buoyancy_ws = has_volume != 0.0f ? s_moment_ws_volume[0].xyz / volume : float3(0.0f, 0.0f, 0.0f);

		GpuRigidBody body = g_bodies[header.body_index];
		bool dynamic = (body.state_flags & ERigidBodyStateFlags_Static) == 0 && body.os_com_and_invmass.w > 0.0f;
		if (dynamic)
		{
			body.force_lin += s_force_ws[0];
			body.force_ang += s_torque_ws[0];
			g_bodies[header.body_index] = body;
		}

		g_diagnostics[hull_index].force_ws = s_force_ws[0];
		g_diagnostics[hull_index].torque_ws = s_torque_ws[0];
		g_diagnostics[hull_index].centre_buoyancy_ws = float4(centre_buoyancy_ws, has_volume);
		g_diagnostics[hull_index].moment_ws_volume = s_moment_ws_volume[0];
		g_diagnostics[hull_index].body_index = header.body_index;
		g_diagnostics[hull_index].valid = 1;
		g_diagnostics[hull_index].volume_m3 = volume;
		g_diagnostics[hull_index].pad0 = 0.0f;
	}
}

// Sampled-composite surface (drag) sample pass. Mirrors CSBuoyancyVolumeSamples but emits SURFACE
// samples (point + outward normal + per-sample area) over the union boundary, deduplicated by the
// any-other-sibling exterior-side cull, and accumulates linear + quadratic drag instead of the
// Froude-Krylov pressure-gradient force. The drag math mirrors the CPU oracle's surface pass:
//   dF = -0.5*rho*Cd*dA*max(0,v_n)^2 * n   (quadratic, windward faces only)
//      + -c_lin*dA * v_rel                 (linear)
// where v_rel = v_point - v_water and v_n = dot(v_rel, n). Both terms are summed (the weight is the
// per-sample area dA). The moment-sum slot is written zero so it does not corrupt the volume pass's
// diagnostic COB when ReduceShared sums all three shared arrays.
numthreads(CSBuoyancyDragSurfaceSamples, BUOYANCY_COLUMN_THREAD_COUNT, 1, 1)
void CSBuoyancyDragSurfaceSamples(uint3 GID(group_id), uint3 GTID(group_thread_id))
{
	int global_group_index = int(group_id.x);
	int hull_index = global_group_index / g.groups_per_hull;
	int hull_group_index = global_group_index - hull_index * g.groups_per_hull;
	int thread_index = int(group_thread_id.x);
	int sample_index = hull_group_index * BUOYANCY_COLUMN_THREAD_COUNT + thread_index;

	float4 force_ws = float4(0.0f, 0.0f, 0.0f, 0.0f);
	float4 torque_ws = float4(0.0f, 0.0f, 0.0f, 0.0f);

	// Drag coefficients: c_lin is rho/tau (packed by the host as g.drag_coefficient) and c_quad is
	// the form-drag coefficient. Drag is active only if at least one is positive.
	float c_lin = g.drag_coefficient;
	float c_quad = g.quadratic_drag_coefficient;
	bool have_drag = c_lin > 0.0f || c_quad > 0.0f;

	if (have_drag && hull_index < g.hull_count)
	{
		BuoySurfHeader header = g_surf_headers[hull_index];
		if (sample_index < header.total_surface_samples)
		{
			GpuRigidBody body = g_bodies[header.body_index];

			// Static / zero-mass bodies have no drag response (mirrors the volume-pass guard).
			bool dynamic = (body.state_flags & ERigidBodyStateFlags_Static) == 0 && body.os_com_and_invmass.w > 0.0f;

			// Gravity defines the buoyancy "up" axis; guard the normalise against (near-)zero gravity.
			float3 gravity_ws = body.ws_gravity.xyz;
			float g_mag = length(gravity_ws);

			if (dynamic && g_mag > BUOY_TINY)
			{
				// Map the flat sample index to its owning primitive k and primitive-local ordinal by
				// walking the per-primitive cumulative counts (parallel to g_prims).
				int k = 0;
				int local_i = sample_index;
				float darea = 0.0f;
				bool found = false;
				for (int pp = 0; pp != header.prim_count; ++pp)
				{
					BuoySurfPrimRecord rec = g_surf_prim_records[header.prim_base + pp];
					if (local_i < rec.count)
					{
						k = pp;
						darea = rec.darea;
						found = true;
						break;
					}
					local_i -= rec.count;
				}

				if (found)
				{
					BuoyPrimitive prim = g_prims[header.prim_base + k];

					// Emit a surface sample (point + outward normal) in shape-local space, then lift to
					// COM-root and world. The normal is rotated by m_s2r (w=0) and renormalised.
					float4 pos_local;
					float4 normal_local;
					float weight;
					BuoyEmitSurfaceSample(prim, BuoySampleIndex(header.hull_id, k, local_i), darea, g_verts, g_face_planes, g_face_verts, pos_local, normal_local, weight);
					float3 p_root = mul(float4(pos_local.xyz, 1.0f), prim.m_s2r).xyz;
					float3 n_root = BuoyNormaliseOrZero(mul(float4(normal_local.xyz, 0.0f), prim.m_s2r).xyz);

					// A surface sample contributes to the union boundary only if it lies outside every
					// other sibling primitive (interior embedded surfaces are not wetted).
					if (!BuoyIsInsideAnyOtherSibling(g_prims, header.prim_base, header.prim_count, k, p_root, n_root, header.eps, g_face_planes))
					{
						float3 up = -gravity_ws / g_mag;
						float3 sample_ws = mul(float4(p_root, 1.0f), body.o2w).xyz;
						float3 normal_ws = BuoyNormaliseOrZero(mul(float4(n_root, 0.0f), body.o2w).xyz);

						// Wet test along -gravity (same flat-ocean idiom as the volume pass).
						float signed_height = dot(sample_ws, up);
						if (signed_height < EvaluateWaterHeight(sample_ws.xy))
						{
							// Body kinematics at this instant (same storage convention as
							// EvaluatePerFaceQuadraticDrag: inertia_inv_* is mass-removed shape inertia).
							float inv_mass = body.os_com_and_invmass.w;
							float3x3 os_iinv = inv_mass * build_symmetric_3x3(body.inertia_inv_diagonal.xyz, body.inertia_inv_products.xyz);
							float3x3 ws_iinv = rotate_inertia_inv(os_iinv, (float3x3)body.o2w);
							float3 v_lin = inv_mass * body.momentum_lin.xyz;
							float3 omega_ws = mul(ws_iinv, body.momentum_ang.xyz);
							float3 com_ws = mul(float4(body.os_com_and_invmass.xyz, 1.0f), body.o2w).xyz;

							// Relative flow at the sample (body velocity minus wave-orbital water flow).
							float3 v_point = v_lin + cross(omega_ws, sample_ws - com_ws);
							float3 v_rel = v_point - EvaluateWaterVelocity(sample_ws);
							float v_n = dot(v_rel, normal_ws);

							float3 dF = float3(0.0f, 0.0f, 0.0f);

							// Quadratic form drag acts only on faces moving outward into the fluid.
							if (c_quad > 0.0f && v_n > 0.0f)
								dF += (-0.5f * g.fluid_density * c_quad * weight * v_n * v_n) * normal_ws;

							// Linear drag acts isotropically against the relative flow.
							if (c_lin > 0.0f)
								dF += (-c_lin * weight) * v_rel;

							force_ws = float4(dF, 0.0f);
							torque_ws = float4(cross(sample_ws - com_ws, dF), 0.0f);
						}
					}
				}
			}
		}
	}

	s_force_ws[thread_index] = force_ws;
	s_torque_ws[thread_index] = torque_ws;
	s_moment_ws_volume[thread_index] = float4(0.0f, 0.0f, 0.0f, 0.0f);
	ReduceShared(uint(thread_index), BUOYANCY_COLUMN_THREAD_COUNT);

	if (thread_index == 0)
	{
		g_partials[global_group_index].force_ws = s_force_ws[0];
		g_partials[global_group_index].torque_ws = s_torque_ws[0];
		g_partials[global_group_index].moment_ws_volume = s_moment_ws_volume[0];
	}
}

// Reduce all surface partials for one composite hull and ADD the drag force/torque to the body
// accumulator and the diagnostic record. The diagnostic already holds the volume pass's buoyancy
// force/torque (this pass runs after the volume reduce), so the diagnostic force_ws/torque_ws become
// the combined buoyancy + drag result (matching the legacy convention). Volume, COB and the moment
// sum are produced by the volume pass and left untouched here.
numthreads(CSBuoyancyDragSurfaceReduce, BUOYANCY_REDUCE_THREAD_COUNT, 1, 1)
void CSBuoyancyDragSurfaceReduce(uint3 GID(group_id), uint3 GTID(group_thread_id))
{
	int hull_index = int(group_id.x);
	int thread_index = int(group_thread_id.x);
	int partial_index = hull_index * g.groups_per_hull + thread_index;

	float4 force_ws = float4(0.0f, 0.0f, 0.0f, 0.0f);
	float4 torque_ws = float4(0.0f, 0.0f, 0.0f, 0.0f);
	if (hull_index < g.hull_count && thread_index < g.groups_per_hull)
	{
		GpuBuoyancyPartial partial = g_partials[partial_index];
		force_ws = partial.force_ws;
		torque_ws = partial.torque_ws;
	}

	s_force_ws[thread_index] = force_ws;
	s_torque_ws[thread_index] = torque_ws;
	s_moment_ws_volume[thread_index] = float4(0.0f, 0.0f, 0.0f, 0.0f);
	ReduceShared(uint(thread_index), BUOYANCY_REDUCE_THREAD_COUNT);

	if (thread_index == 0 && hull_index < g.hull_count)
	{
		BuoySurfHeader header = g_surf_headers[hull_index];

		GpuRigidBody body = g_bodies[header.body_index];
		bool dynamic = (body.state_flags & ERigidBodyStateFlags_Static) == 0 && body.os_com_and_invmass.w > 0.0f;
		if (dynamic)
		{
			body.force_lin += s_force_ws[0];
			body.force_ang += s_torque_ws[0];
			g_bodies[header.body_index] = body;
		}

		g_diagnostics[hull_index].force_ws += s_force_ws[0];
		g_diagnostics[hull_index].torque_ws += s_torque_ws[0];
	}
}
