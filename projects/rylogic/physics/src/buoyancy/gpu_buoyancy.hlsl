//************************************
// Physics Engine
//  Copyright (c) Rylogic Ltd 2026
//************************************
// Sine-wave water buoyancy pass for composite collision-shape hulls.

#include "pr/hlsl/core.hlsli"
#include "pr/hlsl/interop.hlsli"
#include "pr/hlsl/vector.hlsli"
#include "pr/hlsl/spatial_algebra.hlsli"
#include "physics/src/compute/physics_types.hlsli"
#include "physics/src/buoyancy/buoyancy_sampler.hlsli"

#define BUOYANCY_SAMPLE_THREAD_COUNT 256
#define BUOYANCY_REDUCE_THREAD_COUNT 128

struct CBufGpuBuoyancy
{
	int hull_count;
	int groups_per_hull;
	int wave_count;
	float time_s;
	float water_level;
	float fluid_density;
	float linear_drag_coefficient;  // fluid_density / linear_drag_time_constant, applied per wet dV
	float angular_drag_coefficient; // fluid_density / angular_drag_time_constant, applied per wet dV
	float quadratic_drag_coefficient;
	float tangential_drag_coefficient;
	float time_step_s;
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
StructuredBuffer<GpuBuoyancyWave> resource(g_waves, t1);
RWStructuredBuffer<GpuBuoyancyPartial> resource(g_partials, u1);
RWStructuredBuffer<GpuBuoyancyDiagnostic> resource(g_diagnostics, u2);

// --- Sampled-composite volume-pass bindings ---
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
StructuredBuffer<float> resource(g_tet_cdf, t12);

// --- Sampled-composite surface-pass bindings ---
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

groupshared float4 s_force_ws[BUOYANCY_SAMPLE_THREAD_COUNT];
groupshared float4 s_torque_ws[BUOYANCY_SAMPLE_THREAD_COUNT];
groupshared float4 s_moment_ws_volume[BUOYANCY_SAMPLE_THREAD_COUNT];
groupshared float3 s_body_linear_velocity_ws;
groupshared float3 s_body_angular_velocity_ws;

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

// Evaluate the water height and lateral pressure gradient required by the volume pass. Orbital velocity is evaluated
// separately only for accepted wet samples, avoiding exponentials for the common fully-dry case.
// Each wave contributes A*omega^2/g*cos(phase), matching its configured orbital acceleration.
// For a dispersion-consistent gravity wave (omega^2 = g*k), this equals its geometric slope.
float3 EvaluateWaterHeightAndPressureGradient(float2 xy_ws, float gravity)
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
		gradient += direction * (amplitude * phase_speed * phase_speed * c / gravity);
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

// Limit a dissipative surface-drag wrench to the impulse that minimises the body's relative kinetic
// energy during this time step. Explicit integration may otherwise carry a fast or light body past
// zero relative velocity; quadratic drag then alternates sign and injects energy on later steps.
float SurfaceDragScale(GpuRigidBody body, float3 force_ws, float3 torque_ws)
{
	float inv_mass = body.os_com_and_invmass.w;
	if (inv_mass <= 0.0f || g.time_step_s <= 0.0f)
		return 1.0f;

	float3x3 os_iinv = inv_mass * build_symmetric_3x3(body.inertia_inv_diagonal.xyz, body.inertia_inv_products.xyz);
	float3x3 ws_iinv = rotate_inertia_inv(os_iinv, (float3x3)body.o2w);
	float3 com_ws = mul(float4(body.os_com_and_invmass.xyz, 1.0f), body.o2w).xyz;
	float3 linear_velocity_ws = inv_mass * body.momentum_lin.xyz;
	float3 angular_velocity_ws = mul(ws_iinv, body.momentum_ang.xyz);
	float3 relative_linear_velocity_ws = linear_velocity_ws - EvaluateWaterVelocity(com_ws);

	// A non-negative power means the spatially varying water field is driving the body rather than
	// damping its current generalized velocity, so that physically useful transfer is left unchanged.
	float power = dot(force_ws, relative_linear_velocity_ws) + dot(torque_ws, angular_velocity_ws);
	if (power >= 0.0f)
		return 1.0f;

	float response = inv_mass * dot(force_ws, force_ws) + dot(torque_ws, mul(ws_iinv, torque_ws));
	return response > BUOY_TINY
		? saturate(-power / (g.time_step_s * response))
		: 1.0f;
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

// Evaluate a block of composite-hull volume samples (sampled-composite backend) and write one
// partial record for the reducer. Each in-fluid sample contributes a Froude-Krylov pressure-gradient
// force dF = rho*|g|*dV*(up - grad_ws), volume-weighted linear damping, and their torque about the
// body's centre of mass. A weighted moment (sample_ws*dV, dV) recovers wet volume + centre of buoyancy. Overlap
// regions are counted once via the lowest-index-sibling cull, so the union volume of arbitrary
// overlapping convex primitives is unbiased. This mirrors the volume pass of the CPU oracle
// SampleHull (include/pr/physics/buoyancy/buoyancy_sampler.h).
//
// Sample indexing: groups are laid out [hull 0 groups][hull 1 groups]..., g.groups_per_hull groups
// per hull. The flat sample index within a hull selects a primitive by walking the per-primitive
// cumulative counts (g_vol_prim_records, parallel to g_prims); the residual is the primitive-local
// sample ordinal fed to the deterministic hash. Threads beyond a hull's emitted sample count, and
// samples on static / zero-mass bodies or under (near-)zero gravity, contribute zero.
numthreads(CSBuoyancyVolumeSamples, BUOYANCY_SAMPLE_THREAD_COUNT, 1, 1)
void CSBuoyancyVolumeSamples(uint3 GID(group_id), uint3 GTID(group_thread_id))
{
	int global_group_index = int(group_id.x);
	int hull_index = global_group_index / g.groups_per_hull;
	int hull_group_index = global_group_index - hull_index * g.groups_per_hull;
	int thread_index = int(group_thread_id.x);
	int sample_index = hull_group_index * BUOYANCY_SAMPLE_THREAD_COUNT + thread_index;

	float4 force_ws = float4(0.0f, 0.0f, 0.0f, 0.0f);
	float4 torque_ws = float4(0.0f, 0.0f, 0.0f, 0.0f);
	float4 moment_ws_volume = float4(0.0f, 0.0f, 0.0f, 0.0f);

	// Body velocity is invariant across all samples in a threadgroup. Reconstruct angular velocity
	// once instead of repeating the inverse-inertia rotation in every sample thread.
	if (thread_index == 0)
	{
		s_body_linear_velocity_ws = float3(0.0f, 0.0f, 0.0f);
		s_body_angular_velocity_ws = float3(0.0f, 0.0f, 0.0f);
		if ((g.linear_drag_coefficient > 0.0f || g.angular_drag_coefficient > 0.0f) && hull_index < g.hull_count)
		{
			BuoyVolHeader header = g_vol_headers[hull_index];
			GpuRigidBody body = g_bodies[header.body_index];
			float inv_mass = body.os_com_and_invmass.w;
			if ((body.state_flags & ERigidBodyStateFlags_Static) == 0 && inv_mass > 0.0f)
			{
				float3x3 os_iinv = inv_mass * build_symmetric_3x3(body.inertia_inv_diagonal.xyz, body.inertia_inv_products.xyz);
				float3x3 ws_iinv = rotate_inertia_inv(os_iinv, (float3x3)body.o2w);
				s_body_linear_velocity_ws = inv_mass * body.momentum_lin.xyz;
				s_body_angular_velocity_ws = mul(ws_iinv, body.momentum_ang.xyz);
			}
		}
	}
	GroupMemoryBarrierWithGroupSync();

	if (hull_index < g.hull_count)
	{
		BuoyVolHeader header = g_vol_headers[hull_index];
		if (sample_index < header.total_volume_samples)
		{
			GpuRigidBody body = g_bodies[header.body_index];

			// Static / zero-mass bodies have no buoyant response.
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

					// Flat-water fully-dry early-out. This is a per-sample skip, NOT an early return:
					// the group-wide ReduceShared below must be reached by every thread, so we only
					// suppress the per-sample work and leave force/torque/moment at zero. When
					// wave_count==0 the surface is a constant level along 'up', so a box/sphere whose
					// lowest support point is at or above water_level has every sample dry (the wet
					// test uses signed_height < water_level). Skipping it avoids the emit + sibling
					// cull + water eval and contributes exactly zero, so results are unchanged. Only
					// box/sphere have a cheap support test; other primitives fall through to sampling.
					bool fully_dry = false;
					if (g.wave_count == 0)
					{
						float3 up_dry = -gravity_ws / g_mag;
						float4x4 s2w = mul(prim.m_s2r, body.o2w);
						float lo, hi;
						fully_dry = BuoySupportAlongUp(prim, s2w, up_dry, lo, hi) && lo >= g.water_level;
					}

					if (!fully_dry)
					{
					// Emit a volume sample in shape-local space, then lift to COM-root and world.
					float4 pos_local;
					float weight;
					BuoyEmitVolumeSample(prim, BuoySampleIndex(header.hull_id, k, local_i), dvol, g_volume_verts, g_tets, g_tet_cdf, pos_local, weight);
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
						float3 hg = EvaluateWaterHeightAndPressureGradient(sample_ws.xy, g_mag);
						if (signed_height < hg.x)
						{
							// Froude-Krylov pressure-gradient force per unit volume. The lateral term
							// follows the configured wave acceleration; flat water remains purely vertical.
							float3 grad_ws = hg.y * float3(1.0f, 0.0f, 0.0f) + hg.z * float3(0.0f, 1.0f, 0.0f);
							float3 dF = (g.fluid_density * g_mag * weight) * (up - grad_ws);
							float3 com_ws = mul(float4(body.os_com_and_invmass.xyz, 1.0f), body.o2w).xyz;

							// Split translational and rotational point velocities so wave following and
							// roll damping remain independently tunable over the same wet-volume samples.
							if (g.linear_drag_coefficient > 0.0f || g.angular_drag_coefficient > 0.0f)
							{
								float3 r = sample_ws - com_ws;
								if (g.linear_drag_coefficient > 0.0f)
									dF -= (g.linear_drag_coefficient * weight) * (s_body_linear_velocity_ws - EvaluateWaterVelocity(sample_ws));
								if (g.angular_drag_coefficient > 0.0f)
									dF -= (g.angular_drag_coefficient * weight) * cross(s_body_angular_velocity_ws, r);
							}

							force_ws = float4(dF, 0.0f);
							torque_ws = float4(cross(sample_ws - com_ws, dF), 0.0f);
							moment_ws_volume = float4(sample_ws * weight, weight);
						}
					}
					}
				}
			}
		}
	}

	s_force_ws[thread_index] = force_ws;
	s_torque_ws[thread_index] = torque_ws;
	s_moment_ws_volume[thread_index] = moment_ws_volume;
	ReduceShared(uint(thread_index), BUOYANCY_SAMPLE_THREAD_COUNT);

	if (thread_index == 0)
	{
		g_partials[global_group_index].force_ws = s_force_ws[0];
		g_partials[global_group_index].torque_ws = s_torque_ws[0];
		g_partials[global_group_index].moment_ws_volume = s_moment_ws_volume[0];
	}
}

// Reduce all volume partials for one composite hull, add the buoyancy and volume-damping force/torque
// to the body accumulator, and write a diagnostic record. Quadratic drag is added by the later
// surface-sample pass. The hull's body is read via the volume header.
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
// any-other-sibling exterior-side cull, and accumulates quadratic drag. The math mirrors the CPU oracle:
//   dF = -0.5*rho*Cd*dA*max(0,v_n)^2 * n   (quadratic, windward faces only)
// where v_rel = v_point - v_water and v_n = dot(v_rel, n). The moment-sum slot is written zero so it
// does not corrupt the volume pass's diagnostic COB when ReduceShared sums all three shared arrays.
numthreads(CSBuoyancyDragSurfaceSamples, BUOYANCY_SAMPLE_THREAD_COUNT, 1, 1)
void CSBuoyancyDragSurfaceSamples(uint3 GID(group_id), uint3 GTID(group_thread_id))
{
	int global_group_index = int(group_id.x);
	int hull_index = global_group_index / g.groups_per_hull;
	int hull_group_index = global_group_index - hull_index * g.groups_per_hull;
	int thread_index = int(group_thread_id.x);
	int sample_index = hull_group_index * BUOYANCY_SAMPLE_THREAD_COUNT + thread_index;

	float4 force_ws = float4(0.0f, 0.0f, 0.0f, 0.0f);
	float4 torque_ws = float4(0.0f, 0.0f, 0.0f, 0.0f);

	// The surface pass evaluates independent dimensionless normal and tangential drag coefficients.
	float c_quad = g.quadratic_drag_coefficient;
	float c_tangent = g.tangential_drag_coefficient;
	bool have_drag = c_quad > 0.0f || c_tangent > 0.0f;

	// The same body state applies to all samples in this group, so reconstruct its velocities once.
	if (thread_index == 0)
	{
		s_body_linear_velocity_ws = float3(0.0f, 0.0f, 0.0f);
		s_body_angular_velocity_ws = float3(0.0f, 0.0f, 0.0f);
		if (have_drag && hull_index < g.hull_count)
		{
			BuoySurfHeader header = g_surf_headers[hull_index];
			GpuRigidBody body = g_bodies[header.body_index];
			float inv_mass = body.os_com_and_invmass.w;
			if ((body.state_flags & ERigidBodyStateFlags_Static) == 0 && inv_mass > 0.0f)
			{
				float3x3 os_iinv = inv_mass * build_symmetric_3x3(body.inertia_inv_diagonal.xyz, body.inertia_inv_products.xyz);
				float3x3 ws_iinv = rotate_inertia_inv(os_iinv, (float3x3)body.o2w);
				s_body_linear_velocity_ws = inv_mass * body.momentum_lin.xyz;
				s_body_angular_velocity_ws = mul(ws_iinv, body.momentum_ang.xyz);
			}
		}
	}
	GroupMemoryBarrierWithGroupSync();

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

					// Flat-water fully-dry early-out (mirror of the volume kernel). A per-sample skip,
					// not an early return, so every thread still reaches ReduceShared below. When
					// wave_count==0 a box/sphere whose lowest support point is at or above water_level
					// is entirely dry, so it generates no drag samples; skipping it is result-preserving.
					bool fully_dry = false;
					if (g.wave_count == 0)
					{
						float3 up_dry = -gravity_ws / g_mag;
						float4x4 s2w = mul(prim.m_s2r, body.o2w);
						float lo, hi;
						fully_dry = BuoySupportAlongUp(prim, s2w, up_dry, lo, hi) && lo >= g.water_level;
					}

					if (!fully_dry)
					{
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
							float3 com_ws = mul(float4(body.os_com_and_invmass.xyz, 1.0f), body.o2w).xyz;

							// Relative flow at the sample (body velocity minus wave-orbital water flow).
							float3 v_point = s_body_linear_velocity_ws + cross(s_body_angular_velocity_ws, sample_ws - com_ws);
							float3 v_rel = v_point - EvaluateWaterVelocity(sample_ws);
							float v_n = dot(v_rel, normal_ws);

							// Quadratic form drag acts only on faces moving outward into the fluid.
							float3 dF = float3(0.0f, 0.0f, 0.0f);
							if (c_quad > 0.0f && v_n > 0.0f)
								dF += (-0.5f * g.fluid_density * c_quad * weight * v_n * v_n) * normal_ws;

							// Tangential drag opposes the complete in-plane relative velocity. The
							// length(v_t) * v_t form is quadratic in speed and continuous through zero.
							if (c_tangent > 0.0f)
							{
								float3 v_t = v_rel - v_n * normal_ws;
								dF += (-0.5f * g.fluid_density * c_tangent * weight * length(v_t)) * v_t;
							}

							force_ws = float4(dF, 0.0f);
							torque_ws = float4(cross(sample_ws - com_ws, dF), 0.0f);
						}
					}
					}
				}
			}
		}
	}

	s_force_ws[thread_index] = force_ws;
	s_torque_ws[thread_index] = torque_ws;
	s_moment_ws_volume[thread_index] = float4(0.0f, 0.0f, 0.0f, 0.0f);
	ReduceShared(uint(thread_index), BUOYANCY_SAMPLE_THREAD_COUNT);

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
		float4 drag_force_ws = s_force_ws[0];
		float4 drag_torque_ws = s_torque_ws[0];
		if (dynamic)
		{
			// Preserve the sampled wrench direction while preventing an explicit drag impulse from
			// crossing the minimum-relative-energy point during this integration step.
			float scale = SurfaceDragScale(body, drag_force_ws.xyz, drag_torque_ws.xyz);
			drag_force_ws *= scale;
			drag_torque_ws *= scale;
			body.force_lin += drag_force_ws;
			body.force_ang += drag_torque_ws;
			g_bodies[header.body_index] = body;
		}

		g_diagnostics[hull_index].force_ws += drag_force_ws;
		g_diagnostics[hull_index].torque_ws += drag_torque_ws;
	}
}
