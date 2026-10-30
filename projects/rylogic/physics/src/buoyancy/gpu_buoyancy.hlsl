//************************************
// Physics Engine
//  Copyright (c) Rylogic Ltd 2026
//************************************
// Water-field buoyancy pass for composite collision-shape hulls.

#include "pr/hlsl/core.hlsli"
#include "pr/hlsl/interop.hlsli"
#include "pr/hlsl/vector.hlsli"
#include "pr/hlsl/spatial_algebra.hlsli"
#include "physics/src/compute/physics_types.hlsli"
#include "physics/src/buoyancy/buoyancy_sampler.hlsli"
#include "pr/physics/surface/surface_sampling.hlsli"

#define BUOYANCY_SAMPLE_THREAD_COUNT 256
#define BUOYANCY_REDUCE_THREAD_COUNT 128

struct CBufGpuBuoyancy
{
	int hull_count;
	int groups_per_hull;
	int water_field_count;
	float time_s;
	float water_level;
	float water_max_height;         // conservative upper bound of the water surface over all positions
	float fluid_density;
	float linear_drag_coefficient;  // fluid_density / linear_drag_time_constant, applied per wet dV
	float angular_drag_coefficient; // fluid_density / angular_drag_time_constant, applied per wet dV
	float quadratic_drag_coefficient;
	float tangential_drag_coefficient;
	float added_mass_coefficient;   // Ca: wet volume pass adds Ca*rho*dV of translational inertia per sample
	float radiation_damping;        // scale of the wave-radiation damping applied by the implicit solve
	float time_step_s;
	int enable_diagnostics;
	int jacobian_base;  // partial index of the first per-hull record holding the volume pass's implicit-solve terms
	int final_reduce;   // non-zero when this pass's reduce is the last buoyancy pass and applies the implicit solve
	int pad0;
	int pad1;
	int pad2;
};

#include "pr/physics/terrain/water/water_field.hlsli"

// One threadgroup's (or one hull's) sums over its samples. Each sample at offset r from the centre of mass contributes a
// damping rate d that resists the velocity of its point, and partly wet samples contribute a waterline stiffness k that
// resists the vertical displacement of their point, up.(dx + dtheta x r) = up.dx + s.dtheta with s = r x up. The 6x6
// Jacobians of these terms (linear and angular, including the coupling between them) are exactly described by the moments
// of d and k below, so the implicit solve does not need per-sample matrices.
//   force_ws, torque_ws        - hydrodynamic force and torque about the body's centre of mass.
//   moment_ws_volume           - (sum of sample_ws*dV, sum of dV) over wet volume, for diagnostics only.
//   predictor                  - change in the waterline wrench over the step, linearised in the current relative velocity:
//                                xyz = torque, w = force along up.
//   damping_lin                - sum of d over damping that resists the body's linear velocity (N.s/m).
//   damping_lin_moment         - sum of d*r over the same damping. It describes the torque caused by linear velocity.
//   damping_ang_moment         - sum of d*r over damping that resists the point velocity due to rotation, cross(w, r).
//                                It describes the force caused by angular velocity.
//   damping_ang_rr_diag/off    - sum of d*r*r^T over the same damping: (xx, yy, zz) and (xy, xz, yz).
//   stiffness                  - sum of waterline stiffness k (N/m).
//   stiffness_moment           - sum of k*s.
//   stiffness_ss_diag/off      - sum of k*s*s^T: (xx, yy, zz) and (xy, xz, yz).
//   added_mass                 - mass of water that must accelerate with the body, Ca*rho*(wet dV) (kg). It is
//                                treated as isotropic translational inertia at the centre of mass.
//   heave_rate                 - sum of k*(rate the waterline rises out of the water due to the body's linear velocity) (N/s).
//                                Radiation damping resists it.
struct BuoyancySums
{
	float3 force_ws;
	float3 torque_ws;
	float4 moment_ws_volume;
	float4 predictor;
	float damping_lin;
	float3 damping_lin_moment;
	float3 damping_ang_moment;
	float3 damping_ang_rr_diag;
	float3 damping_ang_rr_off;
	float stiffness;
	float3 stiffness_moment;
	float3 stiffness_ss_diag;
	float3 stiffness_ss_off;
	float added_mass;
	float heave_rate;
};

struct GpuBuoyancyDiagnostic
{
	float4 force_ws;
	float4 torque_ws;
	float4 centre_buoyancy_ws;
	float volume_m3;
	int valid;
	float pad0;
	float pad1;
};

ConstantBuffer<CBufGpuBuoyancy> resource(g, b0);
RWStructuredBuffer<GpuRigidBody> resource(g_bodies, u0);
StructuredBuffer<WaterFieldElement> resource(g_water_field, t1);
RWStructuredBuffer<BuoyancySums> resource(g_partials, u1);
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
	int count;          // volume samples allocated to this primitive
	float dvol;         // per-sample volume weight (= primitive_volume / count)
	float cell_height;  // height of the cube with volume dvol; the depth over which a sample goes from dry to fully wet
	float pad;
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

// Per-primitive counts and compact patch ranges, parallel to g_prims.
struct BuoySurfPrimRecord
{
	int count;
	int patch_begin;
	int patch_count;
	int pad;
};

StructuredBuffer<BuoySurfHeader> resource(g_surf_headers, t8);
StructuredBuffer<SurfacePatch> resource(g_surface_patches, t9);
StructuredBuffer<BuoySurfPrimRecord> resource(g_surf_prim_records, t11);

// One record per wave for the group reduction. Waves have at least 4 lanes.
groupshared BuoyancySums s_wave_sums[BUOYANCY_SAMPLE_THREAD_COUNT / 4];
groupshared float3 s_body_linear_velocity_ws;
groupshared float3 s_body_angular_velocity_ws;

// Evaluate the scene-described water height at a world-space XY position.
float EvaluateWaterHeight(float2 xy_ws)
{
	float height = g.water_level;
	for (int element_index = 0; element_index != g.water_field_count; ++element_index)
	{
		height += WaterFieldElementHeight(g_water_field[element_index], xy_ws, g.time_s);
	}
	return height;
}

// Evaluate the water height (x) and the dimensionless dynamic pressure gradient (yzw) required by the volume pass.
// Orbital velocity is evaluated separately only for accepted wet samples, avoiding exponentials for the common fully-dry case.
// Each wave contributes its orbital acceleration divided by -g (see WaterFieldElementHeightAndPressureGradient).
// For a dispersion-consistent gravity wave (omega^2 = g*k), the lateral part equals its geometric slope.
float4 EvaluateWaterHeightAndPressureGradient(float2 xy_ws, float gravity)
{
	float4 result = float4(g.water_level, 0.0f, 0.0f, 0.0f);
	for (int element_index = 0; element_index != g.water_field_count; ++element_index)
		result += WaterFieldElementHeightAndPressureGradient(g_water_field[element_index], xy_ws, g.time_s, gravity);

	return result;
}

// Evaluate the scene-described water particle velocity (orbital flow) at a world-space position.
// Linear deep-water (Airy) theory, consistent with EvaluateWaterHeight:
//   u_along_d = -A*omega * e^(k*z) * sin(phase)   (horizontal, along the wave direction)
//   w_up      =  A*omega * e^(k*z) * cos(phase)   (vertical)
// where z is the signed height relative to the still-water level (clamped <= 0). The drag pass
// subtracts this from the body velocity to form the relative flow at each wetted surface sample.
float3 EvaluateWaterVelocity(float3 pos_ws)
{
	float3 velocity = float3(0.0f, 0.0f, 0.0f);
	for (int element_index = 0; element_index != g.water_field_count; ++element_index)
	{
		velocity += WaterFieldElementVelocity(g_water_field[element_index], pos_ws, g.time_s, g.water_level);
	}
	return velocity;
}

// Evaluate the water surface slope and rise rate at a horizontal position: (dh/dx, dh/dy, dh/dt). See WaterFieldElementSurfaceRates.
float3 EvaluateWaterSurfaceRates(float2 xy_ws)
{
	float3 rates = float3(0.0f, 0.0f, 0.0f);
	for (int element_index = 0; element_index != g.water_field_count; ++element_index)
		rates += WaterFieldElementSurfaceRates(g_water_field[element_index], xy_ws, g.time_s);

	return rates;
}

// Matrix helpers for column vectors, mul(m, v).
float3x3 BuoySkew(float3 r)
{
	// mul(BuoySkew(r), v) = cross(r, v)
	return float3x3(float3(0.0f, -r.z, r.y), float3(r.z, 0.0f, -r.x), float3(-r.y, r.x, 0.0f));
}
float3x3 BuoyOuter(float3 a, float3 b)
{
	// mul(BuoyOuter(a, b), v) = a * dot(b, v)
	return float3x3(a.x * b, a.y * b, a.z * b);
}

// Replace the body's accumulated force and torque with the effective values of a linearly implicit step. The buoyancy wrench is
// evaluated once per step, but the waterline and drag terms change much faster than the step can resolve for light bodies, so an
// explicit step overshoots and launches the body off the waves. The solve treats the hydrodynamic wrench W = (force, torque) as
//   W(v + dv) ~= W + predictor - D*dv - (h/2)*K*dv
// where v = (linear, angular) velocity, D is the 6x6 damping Jacobian and K the 6x6 waterline stiffness Jacobian, and finds the
// effective wrench W_eff whose velocity change dv = h*M^-1*W_eff satisfies it:
//   (I + (h*(D + (h/2)*K + beta*K_heave) + M_a)*M^-1)*W_eff = W + predictor + F_rad
// where M_a is the added mass of the water that accelerates with the body (see BuoyancySums::added_mass), and beta*K_heave and
// F_rad are the wave-radiation damping of heave (see below).
// The (h^2/2) factor matches the integrator's position update, x' = x + h*v + (h/2)*dv. The predictor (-h*K*v_rel) is
// required for stability: it accounts for the waterline moving during the step. Every other force in the accumulator,
// including gravity, goes through the same solve, so the result is consistent at equilibrium (dv = 0 when the net force is 0).
// D and K include the coupling between linear and angular motion. Drag at points away from the centre of mass turns spin into
// force and translation into torque; for light bodies this coupling is as stiff as the direct terms, and leaving it explicit
// pumps energy between spin and translation. Each sample's damping is treated as isotropic at its point. This only changes the
// amount of numerical damping of motion the step cannot resolve; resolved motion converges to the explicit result.
void ApplyImplicitResponse(inout GpuRigidBody body, BuoyancySums terms)
{
	// The solve needs a step length and an up direction. Without either there is nothing to stabilise.
	float h = g.time_step_s;
	float g_mag = length(body.ws_gravity.xyz);
	if (h <= 0.0f || g_mag <= BUOY_TINY)
		return;

	float3 up = -body.ws_gravity.xyz / g_mag;
	float inv_mass = body.os_com_and_invmass.w;
	float3x3 os_iinv = inv_mass * build_symmetric_3x3(body.inertia_inv_diagonal.xyz, body.inertia_inv_products.xyz);
	float3x3 ws_iinv = rotate_inertia_inv(os_iinv, (float3x3)body.o2w);

	// Wave-radiation damping. A waterline moving up and down relative to the water makes surface waves that carry energy away.
	// The long-wave estimate for heave (Haskind relation, deep water) is B = rho*w^3*A^2/(2g), where A is the waterplane area and
	// w the heave frequency. The waterline stiffness is K = rho*g*A, so B = beta*K with beta = w^3*A/(2g^2). w is the heave
	// frequency sqrt(K/(m + M_a)). The estimate is only valid while the radiated wavelength is long compared with the waterplane,
	// so w is limited to sqrt(g/a), where a = sqrt(A/pi) is the waterplane radius; at higher frequencies the real damping falls,
	// but the motion is already overdamped there. Only heave is damped: the sampled waterline is not symmetric, so applying
	// beta to its roll and pitch terms turns vertical motion into false spin. The damping force is -beta*heave_rate, which
	// resists the waterline's mean rate of rising out of the water due to the body's linear velocity.
	float beta = 0.0f;
	float heave_damping_force = 0.0f;
	if (g.radiation_damping > 0.0f && terms.stiffness > 0.0f && g.fluid_density > 0.0f)
	{
		float area = terms.stiffness / (g.fluid_density * g_mag);
		float w2_heave = terms.stiffness * inv_mass / (1.0f + terms.added_mass * inv_mass);
		float w2_limit = g_mag / sqrt(2.0f * area / tau);
		float w2 = min(w2_heave, w2_limit);
		beta = g.radiation_damping * w2 * sqrt(w2) * area / (2.0f * g_mag * g_mag);
		heave_damping_force = -beta * terms.heave_rate;
	}

	// Assemble A = D + (h/2)*K + beta*K_heave in 3x3 blocks: a_fv is the force caused by linear velocity, a_fw the force caused by
	// angular velocity, a_tv the torque caused by linear velocity, and a_tw the torque caused by angular velocity. A point at r moving
	// with cross(w, r) has force -d*cross(w, r) = d*cross(r, w) and torque -d*cross(r, cross(w, r)) = -d*(|r|^2*I - r*r^T)*w.
	float hk = 0.5f * h;
	float3x3 identity = Identity3x3();
	float3x3 rr = build_symmetric_3x3(terms.damping_ang_rr_diag, terms.damping_ang_rr_off);
	float3x3 ss = build_symmetric_3x3(terms.stiffness_ss_diag, terms.stiffness_ss_off);
	float3x3 a_fv = terms.damping_lin * identity + ((hk + beta) * terms.stiffness) * BuoyOuter(up, up);
	float3x3 a_fw = -BuoySkew(terms.damping_ang_moment) + hk * BuoyOuter(up, terms.stiffness_moment);
	float3x3 a_tv = BuoySkew(terms.damping_lin_moment) + hk * BuoyOuter(terms.stiffness_moment, up);
	float3x3 a_tw = (rr._11 + rr._22 + rr._33) * identity - rr + hk * ss;

	// Form N = I + (h*A + M_a)*M^-1 in blocks, where M_a is the added mass. The body's momentum change is h*W_eff, but
	// the hydrodynamic wrench also contains -M_a*dv/h because the attached water must change velocity too, so the
	// effective inertia for the step is M + M_a. Locked rotation axes have zero rows and columns in ws_iinv, so the inertia is never inverted.
	float3x3 n_ff = (1.0f + terms.added_mass * inv_mass) * identity + (h * inv_mass) * a_fv;
	float3x3 n_ft = h * mul(a_fw, ws_iinv);
	float3x3 n_tf = (h * inv_mass) * a_tv;
	float3x3 n_tt = identity + h * mul(a_tw, ws_iinv);

	// Solve N*(force, torque) = (force', torque') by eliminating the force block. n_ff = I + (positive semi-definite) is
	// always invertible, and so is its Schur complement because N's eigenvalues are at least one for physical damping.
	// The radiation damping force is added along up.
	float3 force = body.force_lin.xyz + (terms.predictor.w + heave_damping_force) * up;
	float3 torque = body.force_ang.xyz + terms.predictor.xyz;
	float3x3 n_ff_inv = Invert(n_ff);
	float3x3 schur = n_tt - mul(n_tf, mul(n_ff_inv, n_ft));
	float3 torque_eff = mul(Invert(schur), torque - mul(n_tf, mul(n_ff_inv, force)));
	float3 force_eff = mul(n_ff_inv, force - mul(n_ft, torque_eff));
	body.force_lin.xyz = force_eff;
	body.force_ang.xyz = torque_eff;
}

// Add the damping of a sample at offset 'r' from the centre of mass. 'd_lin' resists the body's linear velocity at the point and
// 'd_ang' resists the point velocity due to rotation; drag on the full point velocity uses the same rate for both.
void AddPointDamping(inout BuoyancySums sums, float3 r, float d_lin, float d_ang)
{
	sums.damping_lin += d_lin;
	sums.damping_lin_moment += d_lin * r;
	sums.damping_ang_moment += d_ang * r;
	sums.damping_ang_rr_diag += d_ang * r * r;
	sums.damping_ang_rr_off += d_ang * float3(r.x * r.y, r.x * r.z, r.y * r.z);
}

// Add the waterline stiffness 'k' of a partly wet sample with s = cross(r, up).
void AddWaterlineStiffness(inout BuoyancySums sums, float3 s, float k)
{
	sums.stiffness += k;
	sums.stiffness_moment += k * s;
	sums.stiffness_ss_diag += k * s * s;
	sums.stiffness_ss_off += k * float3(s.x * s.y, s.x * s.z, s.y * s.z);
}

// Component-wise sum of two records.
BuoyancySums AddSums(BuoyancySums a, BuoyancySums b)
{
	a.force_ws += b.force_ws;
	a.torque_ws += b.torque_ws;
	a.moment_ws_volume += b.moment_ws_volume;
	a.predictor += b.predictor;
	a.damping_lin += b.damping_lin;
	a.damping_lin_moment += b.damping_lin_moment;
	a.damping_ang_moment += b.damping_ang_moment;
	a.damping_ang_rr_diag += b.damping_ang_rr_diag;
	a.damping_ang_rr_off += b.damping_ang_rr_off;
	a.stiffness += b.stiffness;
	a.stiffness_moment += b.stiffness_moment;
	a.stiffness_ss_diag += b.stiffness_ss_diag;
	a.stiffness_ss_off += b.stiffness_ss_off;
	a.added_mass += b.added_mass;
	a.heave_rate += b.heave_rate;
	return a;
}

// Sum a record over all threads of the group. Every thread must call this; the result is only valid in thread 0.
// Lanes are summed with wave intrinsics, so shared memory holds one record per wave instead of one per thread.
// The diagnostic volume moment is skipped when diagnostics are off because it is zero.
BuoyancySums ReduceGroup(BuoyancySums sums, uint thread_index, uint thread_count)
{
	// Sum within each wave and publish one record per wave.
	{
		sums.force_ws = WaveActiveSum(sums.force_ws);
		sums.torque_ws = WaveActiveSum(sums.torque_ws);
		if (g.enable_diagnostics != 0)
			sums.moment_ws_volume = WaveActiveSum(sums.moment_ws_volume);

		sums.predictor = WaveActiveSum(sums.predictor);
		sums.damping_lin = WaveActiveSum(sums.damping_lin);
		sums.damping_lin_moment = WaveActiveSum(sums.damping_lin_moment);
		sums.damping_ang_moment = WaveActiveSum(sums.damping_ang_moment);
		sums.damping_ang_rr_diag = WaveActiveSum(sums.damping_ang_rr_diag);
		sums.damping_ang_rr_off = WaveActiveSum(sums.damping_ang_rr_off);
		sums.stiffness = WaveActiveSum(sums.stiffness);
		sums.stiffness_moment = WaveActiveSum(sums.stiffness_moment);
		sums.stiffness_ss_diag = WaveActiveSum(sums.stiffness_ss_diag);
		sums.stiffness_ss_off = WaveActiveSum(sums.stiffness_ss_off);
		sums.added_mass = WaveActiveSum(sums.added_mass);
		sums.heave_rate = WaveActiveSum(sums.heave_rate);
	}
	uint lane_count = WaveGetLaneCount();
	if (WaveIsFirstLane())
		s_wave_sums[thread_index / lane_count] = sums;

	GroupMemoryBarrierWithGroupSync();

	// Thread 0 is the first lane of wave 0, so it already holds that wave's sum and adds the others.
	if (thread_index == 0)
	{
		uint wave_count = (thread_count + lane_count - 1) / lane_count;
		for (uint wave_index = 1; wave_index < wave_count; ++wave_index)
			sums = AddSums(sums, s_wave_sums[wave_index]);
	}
	return sums;
}

// Evaluate a block of composite-hull volume samples (sampled-composite backend) and write one
// partial record for the reducer. Each in-fluid sample contributes a Froude-Krylov pressure-gradient
// force dF = rho*|g|*dV*(up - grad_ws), volume-weighted linear damping, and their torque about the
// body's centre of mass. Diagnostic mode also accumulates (sample_ws*dV, dV) to recover wet volume
// and centre of buoyancy. Overlap regions are counted once via the lowest-index-sibling cull, so the
// union volume of arbitrary overlapping convex primitives is unbiased. This mirrors the volume pass
// of the CPU oracle SampleHull (include/pr/physics/buoyancy/buoyancy_sampler.h).
//
// Sample indexing: groups are laid out [hull 0 groups][hull 1 groups]..., g.groups_per_hull groups
// per hull. Each thread streams flat sample indices with a stride of one hull's thread population, so
// hulls with more samples than threads are fully covered. The flat sample index within a hull selects a primitive by walking the per-primitive
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

	BuoyancySums sums = (BuoyancySums)0;

	// Body velocity is invariant across all samples in a threadgroup. Reconstruct angular velocity
	// once instead of repeating the inverse-inertia rotation in every sample thread. Damping and the
	// waterline predictor both need it.
	if (thread_index == 0)
	{
		s_body_linear_velocity_ws = float3(0.0f, 0.0f, 0.0f);
		s_body_angular_velocity_ws = float3(0.0f, 0.0f, 0.0f);
		if (hull_index < g.hull_count)
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
		// Stream every ordinal while retaining a bounded number of reduction partials per hull.
		for (; sample_index < header.total_volume_samples; sample_index += g.groups_per_hull * BUOYANCY_SAMPLE_THREAD_COUNT)
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
				float cell_height = 0.0f;
				bool found = false;
				for (int pp = 0; pp != header.prim_count; ++pp)
				{
					BuoyVolPrimRecord rec = g_vol_prim_records[header.prim_base + pp];
					if (local_i < rec.count)
					{
						k = pp;
						dvol = rec.dvol;
						cell_height = rec.cell_height;
						found = true;
						break;
					}
					local_i -= rec.count;
				}

				if (found)
				{
					BuoyPrimitive prim = g_prims[header.prim_base + k];

					// Fully-dry early-out. This is a per-sample skip, NOT an early return:
					// the group-wide ReduceGroup below must be reached by every thread, so we only
					// suppress the per-sample work and leave force/torque/moment at zero. A sample is
					// partly wet while its centre is less than half a cell above the water, and the water
					// height never exceeds water_max_height, so a box/sphere whose lowest support point is
					// at least half a cell above that bound has every sample dry. Skipping it avoids the
					// emit + sibling cull + water eval and contributes exactly zero, so results are
					// unchanged. Only box/sphere have a cheap support test; other primitives fall through to sampling.
					bool fully_dry = false;
					{
						float3 up_dry = -gravity_ws / g_mag;
						float4x4 s2w = mul(prim.m_s2r, body.o2w);
						float lo, hi;
						fully_dry = BuoySupportAlongUp(prim, s2w, up_dry, lo, hi) && lo >= g.water_max_height + 0.5f * cell_height;
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
							// parity gate) this matches signed-height-along-up exactly. Each sample stands for a
							// cell of height cell_height, so it is wet in proportion to how much of that cell is
							// under the surface. This makes buoyancy a continuous function of depth.
							float signed_height = dot(sample_ws, up);
							float4 hg = EvaluateWaterHeightAndPressureGradient(sample_ws.xy, g_mag);
							float wet_fraction = BuoyWetFraction(signed_height, hg.x, cell_height);
							if (wet_fraction > 0.0f)
							{
								// Froude-Krylov pressure-gradient force per unit volume. Hydrostatic pressure gives 'up'; the wave's
								// orbital acceleration adds a dynamic gradient, so water at a crest supports less than still water and a
								// floating body accelerates with it instead of being left behind. The same acceleration also drives the
								// water attached to the body (added mass), so the dynamic term is scaled by the Morison coefficient (1 + Ca).
								float wet_dvol = weight * wet_fraction;
								float3 grad_ws = hg.yzw;
								float3 dF = (g.fluid_density * g_mag * wet_dvol) * (up - (1.0f + g.added_mass_coefficient) * grad_ws);
								sums.added_mass += g.added_mass_coefficient * g.fluid_density * wet_dvol;
								float3 com_ws = mul(float4(body.os_com_and_invmass.xyz, 1.0f), body.o2w).xyz;
								float3 r = sample_ws - com_ws;

								// The water-particle velocity is needed by linear damping only.
								bool partly_wet = wet_fraction < 1.0f;
								float3 water_velocity_ws = float3(0.0f, 0.0f, 0.0f);
								if (g.linear_drag_coefficient > 0.0f)
									water_velocity_ws = EvaluateWaterVelocity(sample_ws);

								// Split translational and rotational point velocities so wave following and
								// roll damping remain independently tunable over the same wet-volume samples.
								// Each damping term also adds its rate to the Jacobian for the implicit solve.
								if (g.linear_drag_coefficient > 0.0f)
								{
									float d_lin = g.linear_drag_coefficient * wet_dvol;
									dF -= d_lin * (s_body_linear_velocity_ws - water_velocity_ws);
									AddPointDamping(sums, r, d_lin, 0.0f);
								}
								if (g.angular_drag_coefficient > 0.0f)
								{
									float d_ang = g.angular_drag_coefficient * wet_dvol;
									dF -= d_ang * cross(s_body_angular_velocity_ws, r);
									AddPointDamping(sums, r, 0.0f, d_ang);
								}

								// A partly wet cell's buoyancy changes by rho*|g|*dvol for each cell height of immersion. Record that
								// stiffness, and the change in buoyancy that the current immersion rate will cause over the step. The
								// immersion rate follows the surface, not the water particles: a point moving with velocity v becomes more
								// immersed at dh/dt + dot(v.xy, slope) - v.z = -(dot(v, n) - dh/dt), where n = up - slope. So a body
								// riding the wave, including one sliding along its slope, is not resisted by it. 'heave_rate' records the
								// part due to the body's linear velocity, which the radiation damping resists.
								if (partly_wet)
								{
									float3 surface_rates = EvaluateWaterSurfaceRates(sample_ws.xy);
									float3 normal = up - float3(surface_rates.xy, 0.0f);
									float stiffness = g.fluid_density * g_mag * weight / cell_height;
									float3 r_up = cross(r, up);
									float3 v_point = s_body_linear_velocity_ws + cross(s_body_angular_velocity_ws, r);
									float delta_force = -g.time_step_s * stiffness * (dot(v_point, normal) - surface_rates.z);
									AddWaterlineStiffness(sums, r_up, stiffness);
									sums.predictor += float4(delta_force * r_up, delta_force);
									sums.heave_rate += stiffness * (dot(s_body_linear_velocity_ws, normal) - surface_rates.z);
								}

								sums.force_ws += dF;
								sums.torque_ws += cross(r, dF);
								if (g.enable_diagnostics != 0)
									sums.moment_ws_volume += float4(sample_ws * wet_dvol, wet_dvol);
							}
						}
					}
				}
			}
		}
	}

	sums = ReduceGroup(sums, uint(thread_index), BUOYANCY_SAMPLE_THREAD_COUNT);
	if (thread_index == 0)
		g_partials[global_group_index] = sums;
}

// Reduce all volume partials for one composite hull, add the buoyancy and volume-damping force/torque
// to the body accumulator, and write a diagnostic record. Quadratic drag is added by the later
// surface-sample pass. The hull's body is read via the volume header. When this is the last buoyancy
// pass, the implicit solve is applied here; otherwise the solve terms are stored for the surface reduce.
numthreads(CSBuoyancyVolumeReduce, BUOYANCY_REDUCE_THREAD_COUNT, 1, 1)
void CSBuoyancyVolumeReduce(uint3 GID(group_id), uint3 GTID(group_thread_id))
{
	int hull_index = int(group_id.x);
	int thread_index = int(group_thread_id.x);
	int partial_index = hull_index * g.groups_per_hull + thread_index;

	BuoyancySums sums = (BuoyancySums)0;
	if (hull_index < g.hull_count && thread_index < g.groups_per_hull)
		sums = g_partials[partial_index];

	sums = ReduceGroup(sums, uint(thread_index), BUOYANCY_REDUCE_THREAD_COUNT);
	if (thread_index == 0 && hull_index < g.hull_count)
	{
		BuoyVolHeader header = g_vol_headers[hull_index];

		GpuRigidBody body = g_bodies[header.body_index];
		bool dynamic = (body.state_flags & ERigidBodyStateFlags_Static) == 0 && body.os_com_and_invmass.w > 0.0f;
		if (dynamic)
		{
			body.force_lin.xyz += sums.force_ws;
			body.force_ang.xyz += sums.torque_ws;
			if (g.final_reduce != 0)
				ApplyImplicitResponse(body, sums);

			g_bodies[header.body_index] = body;
		}

		// The surface pass overwrites the per-group partials, so carry the solve terms in a per-hull record beyond them.
		// The force and torque are already in the body accumulator; the surface reduce uses only the solve terms.
		if (g.final_reduce == 0)
			g_partials[g.jacobian_base + hull_index] = sums;

		if (g.enable_diagnostics != 0)
		{
			// Wet volume + centre of buoyancy are validation outputs and do not participate in force application.
			float volume = sums.moment_ws_volume.w;
			float has_volume = volume > BUOY_TINY ? 1.0f : 0.0f;
			float3 centre_buoyancy_ws = has_volume != 0.0f ? sums.moment_ws_volume.xyz / volume : float3(0.0f, 0.0f, 0.0f);
			g_diagnostics[hull_index].force_ws = float4(sums.force_ws, 0.0f);
			g_diagnostics[hull_index].torque_ws = float4(sums.torque_ws, 0.0f);
			g_diagnostics[hull_index].centre_buoyancy_ws = float4(centre_buoyancy_ws, has_volume);
			g_diagnostics[hull_index].volume_m3 = volume;
			g_diagnostics[hull_index].valid = 1;
			g_diagnostics[hull_index].pad0 = 0.0f;
			g_diagnostics[hull_index].pad1 = 0.0f;
		}
	}
}

// Sampled-composite surface (drag) sample pass. Mirrors CSBuoyancyVolumeSamples but emits SURFACE
// samples (point + outward normal + per-sample area) over the union boundary, deduplicated by the
// any-other-sibling exterior-side cull, and accumulates quadratic drag. The math mirrors the CPU oracle:
//   dF = -0.5*rho*Cd*dA*max(0,v_n)^2 * n   (quadratic, windward faces only)
// where v_rel = v_point - v_water and v_n = dot(v_rel, n).
numthreads(CSBuoyancyDragSurfaceSamples, BUOYANCY_SAMPLE_THREAD_COUNT, 1, 1)
void CSBuoyancyDragSurfaceSamples(uint3 GID(group_id), uint3 GTID(group_thread_id))
{
	int hull_index = int(group_id.y);
	int hull_group_index = int(group_id.x);
	int global_group_index = hull_index * g.groups_per_hull + hull_group_index;
	int thread_index = int(group_thread_id.x);
	int sample_index = hull_group_index * BUOYANCY_SAMPLE_THREAD_COUNT + thread_index;

	BuoyancySums sums = (BuoyancySums)0;

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
		// Stream every ordinal while retaining a bounded number of reduction partials per hull.
		for (; sample_index < header.total_surface_samples; sample_index += g.groups_per_hull * BUOYANCY_SAMPLE_THREAD_COUNT)
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
				int patch_begin = 0;
				int patch_count = 0;
				bool found = false;
				for (int pp = 0; pp != header.prim_count; ++pp)
				{
					BuoySurfPrimRecord rec = g_surf_prim_records[header.prim_base + pp];
					if (local_i < rec.count)
					{
						k = pp;
						patch_begin = rec.patch_begin;
						patch_count = rec.patch_count;
						found = true;
						break;
					}
					local_i -= rec.count;
				}

				if (found)
				{
					BuoyPrimitive prim = g_prims[header.prim_base + k];

					// Fully-dry early-out (mirror of the volume kernel). A per-sample skip, not an
					// early return, so every thread still reaches ReduceGroup below. A box/sphere whose
					// lowest support point is at or above water_max_height is entirely dry. Its surface
					// cells lie on or above that point, so skipping it is consistent with the cell geometry.
					bool fully_dry = false;
					{
						float3 up_dry = -gravity_ws / g_mag;
						float4x4 s2w = mul(prim.m_s2r, body.o2w);
						float lo, hi;
						fully_dry = BuoySupportAlongUp(prim, s2w, up_dry, lo, hi) && lo >= g.water_max_height;
					}

					if (!fully_dry)
					{
						// Emit a surface sample (point + outward normal) in shape-local space, then lift to
						// COM-root and world. The normal is rotated by m_s2r (w=0) and renormalised.
						int lo_patch = 0;
						int hi_patch = patch_count;
						while (lo_patch < hi_patch)
						{
							int mid = lo_patch + (hi_patch - lo_patch) / 2;
							if ((uint) local_i < g_surface_patches[patch_begin + mid].m_sample_end)
								hi_patch = mid;
							else
								lo_patch = mid + 1;
						}
						uint ordinal_begin = lo_patch == 0 ? 0 : g_surface_patches[patch_begin + lo_patch - 1].m_sample_end;
						SurfaceSample sample = EmitSurfaceSample(g_surface_patches[patch_begin + lo_patch], (uint) local_i - ordinal_begin);
						float4 pos_local = sample.m_pos_local;
						float4 normal_local = sample.m_normal_local;
						float weight = sample.m_darea;
						float3 p_root = mul(float4(pos_local.xyz, 1.0f), prim.m_s2r).xyz;
						float3 n_root = BuoyNormaliseOrZero(mul(float4(normal_local.xyz, 0.0f), prim.m_s2r).xyz);

						// A surface sample contributes to the union boundary only if it lies outside every
						// other sibling primitive (interior embedded surfaces are not wetted).
						if (!BuoyIsInsideAnyOtherSibling(g_prims, header.prim_base, header.prim_count, k, p_root, n_root, header.eps, g_face_planes))
						{
							float3 up = -gravity_ws / g_mag;
							float3 sample_ws = mul(float4(p_root, 1.0f), body.o2w).xyz;
							float3 normal_ws = BuoyNormaliseOrZero(mul(float4(n_root, 0.0f), body.o2w).xyz);

							// Each sample stands for a surface cell, so drag acts on the cell's submerged area (see buoyancy::CellWetFraction).
							float signed_height = dot(sample_ws, up);
							float wet_weight = weight * BuoyWetFraction(signed_height, EvaluateWaterHeight(sample_ws.xy), BuoySurfaceCellHeight(weight, dot(normal_ws, up)));
							if (wet_weight > 0.0f)
							{
								float3 com_ws = mul(float4(body.os_com_and_invmass.xyz, 1.0f), body.o2w).xyz;

								// Relative flow at the sample (body velocity minus wave-orbital water flow).
								float3 v_point = s_body_linear_velocity_ws + cross(s_body_angular_velocity_ws, sample_ws - com_ws);
								float3 v_rel = v_point - EvaluateWaterVelocity(sample_ws);
								float v_n = dot(v_rel, normal_ws);

								// Quadratic form drag acts only on faces moving outward into the fluid. The implicit solve uses
								// its secant damping rate, 0.5*rho*Cd*dA*v_n (force / speed). For pure quadratic drag this makes
								// the step v' = v / (1 + h*c*v/m), which is the exact solution and never reverses the velocity.
								float3 dF = float3(0.0f, 0.0f, 0.0f);
								float damping = 0.0f;
								if (c_quad > 0.0f && v_n > 0.0f)
								{
									float d_quad = 0.5f * g.fluid_density * c_quad * wet_weight * v_n;
									dF -= (d_quad * v_n) * normal_ws;
									damping += d_quad;
								}

								// Tangential drag opposes the complete in-plane relative velocity. The
								// length(v_t) * v_t form is quadratic in speed and continuous through zero.
								if (c_tangent > 0.0f)
								{
									float3 v_t = v_rel - v_n * normal_ws;
									float d_tangent = 0.5f * g.fluid_density * c_tangent * wet_weight * length(v_t);
									dF -= d_tangent * v_t;
									damping += d_tangent;
								}

								// Each sample's damping rate resists the full velocity of its point, treated as isotropic.
								float3 r = sample_ws - com_ws;
								sums.force_ws += dF;
								sums.torque_ws += cross(r, dF);
								AddPointDamping(sums, r, damping, damping);
							}
						}
					}
				}
			}
		}
	}

	sums = ReduceGroup(sums, uint(thread_index), BUOYANCY_SAMPLE_THREAD_COUNT);
	if (thread_index == 0)
		g_partials[global_group_index] = sums;
}

// Reduce all surface partials for one composite hull, ADD the drag force/torque to the body
// accumulator, and apply the implicit solve using the combined volume and surface Jacobians. The
// diagnostic already holds the volume pass's buoyancy force/torque (this pass runs after the volume
// reduce), so the diagnostic force_ws/torque_ws become the combined buoyancy + drag result (matching
// the legacy convention). Diagnostics report the sampled wrench, before the implicit solve. Volume,
// COB and the moment sum are produced by the volume pass and left untouched here.
numthreads(CSBuoyancyDragSurfaceReduce, BUOYANCY_REDUCE_THREAD_COUNT, 1, 1)
void CSBuoyancyDragSurfaceReduce(uint3 GID(group_id), uint3 GTID(group_thread_id))
{
	int hull_index = int(group_id.x);
	int thread_index = int(group_thread_id.x);
	int partial_index = hull_index * g.groups_per_hull + thread_index;

	BuoyancySums sums = (BuoyancySums)0;
	if (hull_index < g.hull_count && thread_index < g.groups_per_hull)
		sums = g_partials[partial_index];

	sums = ReduceGroup(sums, uint(thread_index), BUOYANCY_REDUCE_THREAD_COUNT);
	if (thread_index == 0 && hull_index < g.hull_count)
	{
		BuoySurfHeader header = g_surf_headers[hull_index];

		GpuRigidBody body = g_bodies[header.body_index];
		bool dynamic = (body.state_flags & ERigidBodyStateFlags_Static) == 0 && body.os_com_and_invmass.w > 0.0f;
		float4 drag_force_ws = float4(sums.force_ws, 0.0f);
		float4 drag_torque_ws = float4(sums.torque_ws, 0.0f);
		if (dynamic)
		{
			// Combine the volume pass's solve terms with this pass's drag damping, then solve. The volume record's force and
			// torque are already in the body accumulator, so only its solve terms are combined.
			BuoyancySums terms = AddSums(g_partials[g.jacobian_base + hull_index], sums);
			body.force_lin += drag_force_ws;
			body.force_ang += drag_torque_ws;
			ApplyImplicitResponse(body, terms);
			g_bodies[header.body_index] = body;
		}

		if (g.enable_diagnostics != 0)
		{
			g_diagnostics[hull_index].force_ws += drag_force_ws;
			g_diagnostics[hull_index].torque_ws += drag_torque_ws;
		}
	}
}
