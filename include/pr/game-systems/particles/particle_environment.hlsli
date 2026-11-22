//************************************
// Game Systems
//  Copyright (c) Rylogic Ltd 2026
//************************************
// The environment that stepped particle types move in: gravity, wind, and sets of static collision planes. A particle system owns one environment,
// set by the caller in the space of its particles' positions. Particles never query physics; the caller describes the surfaces they may hit as planes.
//
// Stepped types integrate their motion in each tick instead of evaluating a closed form of their age (see ParticleIntegrate), so they can respond to
// wind and bounce off planes. Each tick depends only on the particle's state, the environment, and the tick length, so particles behave the same
// in slow motion and are independent of the camera. Trajectories depend slightly on the tick length where particles hit planes, because a tick's
// motion is tested for contacts as a straight segment.
//
// A type's compute shader includes this file before it defines its functions when they use the environment. particle_kernels.hlsli includes it too,
// so every kernel declares the environment's registers.
#ifndef PR_GAME_SYSTEMS_PARTICLE_ENVIRONMENT_HLSLI
#define PR_GAME_SYSTEMS_PARTICLE_ENVIRONMENT_HLSLI
#include "pr/game-systems/particles/particles.hlsli"

#ifdef __cplusplus
namespace pr::game_systems::particles
{
	using namespace pr::hlsl;
#endif

// The environment constants seen by every particle kernel at b2.
struct ParticleEnvironmentConstants
{
	float3 gravity;               // Acceleration applied to every stepped particle (m/s²).
	uint collision_iterations;    // Most plane contacts resolved per particle per tick. See ParticleCollidePlanes.
	float3 wind;                  // Uniform wind velocity (m/s), added to the wind grid.
	uint collision_set_count;     // Number of entries in the collision set table.
	float2 wind_origin;           // The minimum corner of the wind grid, in the XY plane.
	float wind_cell_size;         // The side length of a wind grid cell.
	uint pad0;
	uint2 wind_cells;             // The wind grid's columns in x and y. Zero when there is no grid.
	uint pad1;
	uint pad2;
};

// One static plane that particles collide with. Particles are allowed in the half-space dot(n, p) >= d, and their centres are kept 'radius' above it.
struct ParticleCollisionPlane
{
	float4 plane;       // xyz = Unit normal n, pointing into the allowed half-space. w = Offset d.
	float restitution;  // Fraction of the normal velocity kept after a contact. [0, 1].
	float damping;      // Fraction of the tangential velocity removed by a contact. [0, 1].
	float pad0;
	float pad1;
};

#ifndef __cplusplus

ConstantBuffer<ParticleEnvironmentConstants> g_environment : register(b2);
StructuredBuffer<ParticleCollisionPlane> g_planes : register(t2);
StructuredBuffer<uint2> g_collision_sets : register(t3); // (first plane, plane count) of each collision set.
StructuredBuffer<float2> g_wind : register(t4);          // Wind grid velocity of each column, row-major: index = y * wind_cells.x + x.

// Collision counts of the current thread, which the kernel framework adds to the type's counters once per wave.
static uint3 s_collision_counts = uint3(0, 0, 0); // x = contacts, y = initial overlaps, z = budget exhausted.

// Return the wind velocity at 'position': the uniform wind plus the wind grid, interpolated between column centres and clamped at the grid's edges.
float3 ParticleSampleWind(float3 position)
{
	// The grid has no vertical wind and no variation with height.
	float3 wind = g_environment.wind;
	uint2 cells = g_environment.wind_cells;
	if (cells.x != 0 && cells.y != 0)
	{
		// Column centres are at origin + (i + 0.5) * cell_size, so the sample point is measured from the first centre.
		float2 uv = clamp((position.xy - g_environment.wind_origin) / g_environment.wind_cell_size - 0.5f, 0.0f, float2(cells - 1));
		uint2 i0 = uint2(uv);
		uint2 i1 = min(i0 + 1, cells - 1);
		float2 f = uv - float2(i0);
		float2 w00 = g_wind[i0.y * cells.x + i0.x];
		float2 w10 = g_wind[i0.y * cells.x + i1.x];
		float2 w01 = g_wind[i1.y * cells.x + i0.x];
		float2 w11 = g_wind[i1.y * cells.x + i1.x];
		wind.xy += lerp(lerp(w00, w10, f.x), lerp(w01, w11, f.x), f.y);
	}
	return wind;
}

// Advance 'position' and 'velocity' by 't' seconds under constant acceleration 'accel' and drag towards the air velocity 'wind' at the rate 'drag' (1/s):
//   dv/dt = accel + drag * (wind - v)
// The solution is exact for any 't', so the result does not depend on how a time span is divided into ticks. 'drag' is non-negative.
void ParticleIntegrate(inout float3 position, inout float3 velocity, float3 accel, float drag, float3 wind, float t)
{
	// f = (1 - e^(-kt)) / k and h = (t - f) / k. Their exact forms lose precision as kt approaches zero, so short spans use their series instead.
	float x = drag * t;
	float f;
	float h;
	if (x < 1e-2f)
	{
		f = t * (1.0f - x * (0.5f - x / 6.0f));
		h = t * t * (0.5f - x * (1.0f / 6.0f - x / 24.0f));
	}
	else
	{
		f = (1.0f - exp(-x)) / drag;
		h = (t - f) / drag;
	}

	// The velocity relaxes towards the wind while the acceleration acts on it. The relaxation factor e^(-kt) equals 1 - k * f.
	float3 relative = velocity - wind;
	position += wind * t + relative * f + accel * h;
	velocity = wind + relative * (1.0f - drag * f) + accel * f;
}

// Return 'x' with its component into a plane of normal 'n' reflected and scaled by 'restitution', and its tangential component reduced by 'damping'.
// 'x' is unchanged when it does not point into the plane.
float3 ParticleReflect(float3 x, float3 n, float restitution, float damping)
{
	// Only motion into the plane is changed, so a particle moving away keeps its velocity.
	float xn = dot(x, n);
	if (xn >= 0.0f)
		return x;

	float3 tangential = x - xn * n;
	return tangential * (1.0f - damping) - restitution * xn * n;
}

// The distance below which a particle touches a plane rather than overlapping it. It covers the rounding of positions near 'position', so a
// particle resting on a plane is not reported as overlapping it.
float ParticleContactTolerance(float3 position)
{
	// Float rounding scales with the magnitude of the coordinates.
	float3 a = abs(position);
	return 1e-5f * max(1.0f, max(a.x, max(a.y, a.z)));
}

// Collide a particle of 'radius' moving from 'start' to 'end' this tick, with final velocity 'velocity', against the planes of collision set 'set'.
// On return, 'end' and 'velocity' are the resolved position and velocity. 'start' is moved out of any plane it began inside.
//  - A particle that starts inside a plane's allowed distance is pushed out along the plane's normal, and its velocity into the plane is reflected.
//  - The tick's motion is then a straight segment. Its earliest crossing of any plane moves the particle to the contact point, reflects the velocity
//    and the remaining motion, and the search repeats with the remaining motion.
//  - At most 'collision_iterations' contacts are resolved. A particle with further contacts stops at its last contact point for the rest of the tick.
// A 'set' outside the collision set table, such as -1, has no planes. Contacts, overlaps and exhausted budgets are counted for diagnostics.
void ParticleCollidePlanes(int set, float radius, inout float3 start, inout float3 end, inout float3 velocity)
{
	// Particles without a collision set move freely.
	if (set < 0 || uint(set) >= g_environment.collision_set_count)
		return;

	uint2 range = g_collision_sets[set];
	if (range.y == 0)
		return;

	// Push the particle out of planes it starts inside, carrying the tick's motion with it.
	float tolerance = ParticleContactTolerance(start);
	for (uint i = 0; i != range.y; ++i)
	{
		// The push resolves each plane in turn, so a particle inside a corner can end slightly inside an earlier plane. The sweep then corrects it.
		ParticleCollisionPlane plane = g_planes[range.x + i];
		float depth = plane.plane.w + radius - dot(plane.plane.xyz, start);
		if (depth > tolerance)
		{
			start += depth * plane.plane.xyz;
			end += depth * plane.plane.xyz;
			velocity = ParticleReflect(velocity, plane.plane.xyz, plane.restitution, plane.damping);
			s_collision_counts.y += 1;
		}
	}

	// Resolve the earliest crossing of the remaining motion until it crosses no plane or the budget is spent.
	for (uint contacts = 0;; ++contacts)
	{
		// Find the earliest plane crossed by the segment while moving into the plane.
		float3 motion = end - start;
		float earliest = 2.0f;
		uint hit = 0;
		for (uint j = 0; j != range.y; ++j)
		{
			// The distances are measured from the surface that the particle's centre is kept on.
			ParticleCollisionPlane plane = g_planes[range.x + j];
			float3 n = plane.plane.xyz;
			float d0 = dot(n, start) - plane.plane.w - radius;
			float d1 = dot(n, end) - plane.plane.w - radius;
			if (dot(motion, n) >= 0.0f || d1 >= -tolerance)
				continue;

			// A start already on or slightly inside the surface crosses at the start.
			float t = max(d0, 0.0f) / (max(d0, 0.0f) - d1);
			if (t < earliest)
			{
				earliest = t;
				hit = j;
			}
		}
		if (earliest > 1.0f)
			break;

		// With the budget spent, the particle stops at its last contact point rather than passing through the plane.
		if (contacts == g_environment.collision_iterations)
		{
			end = start;
			s_collision_counts.z += 1;
			break;
		}

		// Move to the contact, and bounce the remaining motion and the velocity off the plane.
		ParticleCollisionPlane plane = g_planes[range.x + hit];
		float3 contact = start + earliest * motion;
		float3 remaining = ParticleReflect(end - contact, plane.plane.xyz, plane.restitution, plane.damping);
		velocity = ParticleReflect(velocity, plane.plane.xyz, plane.restitution, plane.damping);
		start = contact;
		end = contact + remaining;
		s_collision_counts.x += 1;
	}
}

#endif

#ifdef __cplusplus
	static_assert(sizeof(ParticleEnvironmentConstants) == 64);
	static_assert(sizeof(ParticleCollisionPlane) == 32);
}
#endif
#endif
