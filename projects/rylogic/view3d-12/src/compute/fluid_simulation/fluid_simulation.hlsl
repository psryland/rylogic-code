//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#include "pr/hlsl/core.hlsli"
#include "pr/hlsl/geometry.hlsli"
#include "src/compute/particle_collision/particle.hlsli"
#include "src/compute/particle_collision/collision.hlsli"
#include "src/compute/spatial_partition/spatial_partition.hlsli"

#ifndef THREAD_GROUP_SIZE
#define THREAD_GROUP_SIZE 32
#endif
static const uint ThreadGroupSize = THREAD_GROUP_SIZE;

static const uint ColourScheme_None = 0;
static const uint ColourScheme_Velocity = 1;
static const uint ColourScheme_Accel = 2;
static const uint ColourScheme_Density = 3;
static const uint ColourScheme_Probe = 4;

static const uint MapType_None = 0;
static const uint MapType_Velocity = 1;
static const uint MapType_Accel = 2;
static const uint MapType_Density = 3;

// ** Note ** : alignment matters. float4 must be aligned to 16 bytes. [arrays] get aligned to 16 bytes

// Force profile controls
struct ForceControls
{
	float Scale;       // The force scaling factor
	float Range;       // Scales the repulsive force effectively making it act at greater range. [0,inf) (typically 1.0)
	float Balance;     // The position of the transition from the repulsive to the attractive force. [0,1] (typically ~0.8)
	float Dip;         // The depth of the attractive force [0,inf) (typically ~0.1)
};

// Colour spectrum
struct ColourSpectrum
{
	float4 Spectrum[4];     // The colour scale to use
	float2 Range;           // Scales [0,1] to the colour range
};

// Constant buffers
cbuffer cbFluidSim : register(b0)
{
	struct
	{
		float4 Gravity;         // The acceleration due to gravity

		int Dimensions;         // 2D or 3D simulation
		int NumParticles;       // The number of particles
		int CellCount;          // The number of grid cells in the spatial partition
		float GridScale;        // The scale factor for the spatial partition grid

		float ParticleRadius;   // The radius of influence for each particle
		float TimeStep;         // Leap-frog time step
		float ThermalDiffusion; // The thermal diffusion rate
		int RandomSeed;         // Seed value for the RNG

		ForceControls Force;

		float Viscosity;        // The viscosity scaler
		float3 pad;
	} Sim;
};
cbuffer cbProbeData : register(b1)
{
	struct
	{
		float4 Position;      // The position of the probe
		float Radius;         // The radius of the probe
		float Force;          // The force that the probe applies
		int NumParticles;     // The number of particles in the 'm_particles' buffer
	} Probe;
};
cbuffer cbCullData : register(b0)
{
	struct
	{
		int NumParticles;       // The number of particles
		int CellCount;          // The number of grid cells in the spatial partition
	} Cull;
};
cbuffer cbColourData : register(b0)
{
	struct
	{
		ColourSpectrum Colours; // Colouring data
		int NumParticles;       // The number of particles in the 'm_particles' buffer
		int Scheme;             // 0 = None, 1 = Velocity, 2 = Accel, 3 = Density
	} Col;
};
cbuffer cbMapData : register(b0)
{
	struct
	{
		float4x4 MapToWorld;    // Transform from map space to world space (including scale)
		ColourSpectrum Colours; // Colouring data
		int2 TexDim;            // The dimensions of the map texture

		int Type;              // 0 = Pressure
		int Dimensions;        // 2D or 3D simulation
		int CellCount;         // The number of grid cells in the spatial partition
		float GridScale;       // The scale factor for the spatial partition grid

		ForceControls Force;

		float ParticleRadius;  // The particle radius
		float3 pad;
	} Map;
};

struct OutputData
{
	int num_particles;
	uint warnings;
	float p0_energy;
};

// The positions of each particle
RWStructuredBuffer<PositionType> m_positions : register(u0);

// The dynamics for each particle
RWStructuredBuffer<DynamicsType> m_dynamics : register(u1);

// The indices of particle positions sorted spatially
RWStructuredBuffer<uint> m_spatial : register(u2);

// The lowest index (in m_spatial) for each cell hash (length CellCount)
RWStructuredBuffer<uint> m_idx_start : register(u3);

// The number of positions for each cell hash (length CellCount)
RWStructuredBuffer<uint> m_idx_count : register(u4);

// A buffer for passing back results
RWStructuredBuffer<OutputData> m_output : register(u5);

// A texture for writing the density map to
RWTexture2D<float4> m_tex_map : register(u6);

// Return a particle from the particle and dynamics buffers
inline Particle GetParticle(int i)
{
	return ToParticle(i, m_positions, m_dynamics);
}

// A random normal vector in 2 or 3 dimensions
inline float4 RandomNWithDim(uniform float2 seed, uniform int spatial_dimensions)
{
	return spatial_dimensions == 2 ? Random2N(seed) : Random3N(seed);
}

// Return a volume for a neighbour search
inline float4 SearchVolume(float radius, uniform int spatial_dimensions)
{
	return float4(radius, radius, select(spatial_dimensions == 3, radius, 0), 0);
}

// Return the colour interpolated on the colour scale
inline float4 LerpColour(float value, uniform ColourSpectrum col)
{
	float scale = saturate((value - col.Range.x) / (col.Range.y - col.Range.x));
	if (scale <= 0.0f)  return col.Spectrum[0];
	if (scale < 0.333f) return lerp(col.Spectrum[0], col.Spectrum[1], (scale - 0.000f) / 0.333f);
	if (scale < 0.666f) return lerp(col.Spectrum[1], col.Spectrum[2], (scale - 0.333f) / 0.333f);
	if (scale < 1.0f)   return lerp(col.Spectrum[2], col.Spectrum[3], (scale - 0.666f) / 0.333f);
	return col.Spectrum[3];
}

// Reflect a particle through a surface. 'dist_n' is the relative normal distance between the particle and the surface orign
inline Particle ReflectParticle(Particle particle, float dist_n, float4 surface)
{
	Particle reflection;
	dist_n += surface.w;
	dist_n = select(dist_n != 0, dist_n, TINY);
	reflection.pos = particle.pos - 2.0f * dist_n * float4(surface.xyz, 0);
	reflection.vel = float4(0, 0, 0, 0);
	return reflection;
}

// The influence at normalised 'distance' from a particle
inline float ViscosityKernel(float normalised_distance, uniform float viscosity_power)
{
	// -pow(x, viscosity_power) + 1
	float x = clamp(normalised_distance, 0.0f, 1.0f);
	float viscosity = -pow(x, sqr(viscosity_power)) + 1.0f;
	return select(viscosity_power != 0, viscosity, 0.0f);
}

// Contribution to density based on distance
inline float DensityKernel(float normalised_distance, uniform int spatial_dimensions)
{
	// The calculated density needs to be independent of particle radius.
	// Conceptually, we divide the sum of particle densities within the kernel
	// by the volume under the kernel.
	const float tau = 6.28318530717958647693f;
	float volume = 1.0f;// select(spatial_dimensions == 2,
		//(1.0f/6.0f) * tau * sqr(particle_radius),
		//(1.0f/6.0f) * tau * cube(particle_radius));

	float x = clamp(normalised_distance, 0.0f, 1.0f);
	return (1.0f - x) / volume;
}

// The density at normalized particle relative position 'pos_r' (assumes the particle is at the origin, and particle radius is 1)
inline float DensityAt(float4 pos_r, uniform int spatial_dimensions)
{
	float dist = select(any(pos_r), length(pos_r), TINY);
	float density = DensityKernel(dist, spatial_dimensions);
	return density;
}

// Find the density at 'pos' due to 'neighbour' (including its reflected ghost particles)
inline float DensityAt(float4 pos, Particle neighbour, uniform float particle_radius, uniform int spatial_dimensions)
{
	float density = 0;
	float4 pos_r = (pos - neighbour.pos) / particle_radius;
	
	// If 'neighbour' has no nearby boundary, then only the neighbour contributes
	if (WaveActiveOrThisThreadTrue(neighbour.surface.w >= particle_radius))
	{
		density += DensityAt(pos_r, spatial_dimensions);
	}

	// If 'target' is on the positive side of 'neighbour's surface, react with it and it's reflected ghost
	else if (dot(neighbour.surface, pos_r) > -neighbour.surface.w / particle_radius)
	{
		density += DensityAt(pos_r, spatial_dimensions);
		pos_r += (2 * neighbour.surface.w / particle_radius) * float4(neighbour.surface.xyz, 0);
		density += DensityAt(pos_r, spatial_dimensions);
	}

	return density;
}

// The force at normalised 'distance' from a particle. I.e. Force as a function of distance. Returns a value between [range, 0] (for x = [0, 1])
inline float ForceKernel(float normalised_distance, uniform ForceControls force)
{
	// The function is a piecewise function created from:
	//  Repulsive force: range * sqr(1 - x/balance), for x in [0,balance]
	//  Attractive force: -dip * sqr(sqr(Cx - C + 1) - 1), where C = 2 / (1 - balance), for x in [balance,1]
	float x = clamp(normalised_distance, 0.0001f, 1.0f);
	if (x <= force.Balance)
	{
		return force.Scale * (force.Range * force.Balance / x) * sqr(1.0f - x / force.Balance);
	}
	else
	{
		float C = 2.0f / (1.0f - force.Balance);
		return force.Scale * -force.Dip * sqr(sqr(C * x - C + 1) - 1);
	}
}
inline float dForceKernel(float normalised_distance, uniform ForceControls force)
{
	float x = clamp(normalised_distance, 0.0001f, 1.0f);
	if (x <= force.Balance)
	{
		return force.Scale * (force.Range / force.Balance) * (x - force.Balance) * (x + force.Balance) / sqr(x);
	}
	else
	{
		float C = 2.0f / (1.0f - force.Balance);
		float D = C * x - C + 1;
		return force.Scale * (4 * C * -force.Dip) * D * (sqr(D) - 1);
	}
}

// The pressure gradient at normalized particle relative position 'pos_r' (assumes the particle is at the origin, and particle radius is 1)
inline float4 PressureAt(float4 pos_r, uniform ForceControls force)
{
	float dist = select(any(pos_r), length(pos_r), TINY);
	float pressure = -dForceKernel(dist, force);
	return (pressure / dist) * pos_r;
}

// Find the pressure gradient at 'pos' due to 'neighbour' (including its reflected ghost particles)
inline float4 PressureAt(float4 pos, Particle neighbour, uniform float particle_radius, uniform ForceControls force)
{
	float4 grad = float4(0,0,0,0);
	float4 pos_r = (pos - neighbour.pos) / particle_radius;
	
	// If 'neighbour' has no nearby boundary, react with neighbour only
	if (WaveActiveOrThisThreadTrue(neighbour.surface.w >= particle_radius))
	{
		grad += PressureAt(pos_r, force);
	}

	// If 'target' is on the positive side of 'neighbour's surface, react with it and it's reflected ghost
	else if (dot(neighbour.surface, pos_r) > -neighbour.surface.w / particle_radius)
	{
		grad += PressureAt(pos_r, force);
		pos_r += (2 * neighbour.surface.w / particle_radius) * float4(neighbour.surface.xyz, 0);
		grad += PressureAt(pos_r, force);
	}

	return grad;
}


// The displacement at particle relative position 'pos_r' (assumes the particle is at the origin, and particle radius is 1)
inline float4 DisplacementAt(float4 pos_r, uniform ForceControls force, uniform float4 rand_dir)
{
	// Although particles should be 2 particle radius' apart, they don't see each
	// other until they are within 1 particle radius of each other. This means particles
	// should push apart to a separation of 1 particle radius.
	if (!WaveActiveOrThisThreadTrue(any(pos_r)))
		pos_r = TINY * rand_dir;

	float dist = length(pos_r);
	float shove = max(0, 1.0f - dist);
	return (force.Scale * shove / dist) * pos_r;
}

// Find the amount to displace 'pos' due to 'neighbour' (including its reflected ghost particles)
inline float4 DisplacementAt(float4 pos, Particle neighbour, uniform float time_step, uniform float particle_radius, uniform ForceControls force, uniform float4 rand_dir)
{
	float4 displacement = float4(0,0,0,0);
	float4 neighbour_pos = neighbour.pos + neighbour.vel * time_step * 0.5f;
	
	// Find the distance between 'pos' and 'neighbour' and push them apart by some fraction of that distance
	float4 pos_r = (pos - neighbour_pos) / particle_radius;
	
	// If 'neighbour' has no nearby boundary, react with neighbour only
	if (WaveActiveOrThisThreadTrue(neighbour.surface.w >= particle_radius))
	{
		// The amount needed to separate the particles (if applied to both particles)
		displacement += DisplacementAt(pos_r, force, rand_dir);
	}

	// If 'target' is on the positive side of 'neighbour's surface, react with it and it's reflected ghost
	else if (dot(neighbour.surface, pos_r) * particle_radius > -neighbour.surface.w)
	{
		displacement += DisplacementAt(pos_r, force, rand_dir);
		pos_r += (2 * neighbour.surface.w / particle_radius) * float4(neighbour.surface.xyz, 0);
	//	displacement += DisplacementAt(pos_r, force, rand_dir);
	}

	// Half because displacement should be applied to both particles
	return 0.5f * displacement / particle_radius;
}


// Calculates forces at each particle position
[numthreads(ThreadGroupSize, 1, 1)]
void ApplyForces(uint3 dtid : SV_DispatchThreadID, uint3 gtid : SV_GroupThreadID)
{
	if (dtid.x >= Sim.NumParticles)
		return;

	Particle target = GetParticle(dtid.x);
	float4 rand_dir = RandomNWithDim(float2(dtid.x, Sim.RandomSeed), Sim.Dimensions);

	// Fluids are incompressible so no matter what gravity force is applied, the distance between particles 
	// in equilibrium should be the same. Apply gravity first then inter-particle forces and restitution.

	// Apply gravity and thermal diffusion
	target.acc = float4(0, 0, 0, 0);
	target.acc += Sim.Gravity;
	target.acc += rand_dir * Sim.ThermalDiffusion;

	// Sample the environment for neighbours at the predicted particle position at TimeStep/2
	float half_time_step = Sim.TimeStep * 0.5f;
	float4 target_pos = target.pos + target.vel * half_time_step + 0.5f * target.acc * sqr(half_time_step);
	
	// The weighted group velocity
	float4 group_velocity = target.vel;
	group_velocity.w = 1; // .w is the weight

	// Borrowed idea from Verlet integration:
	// 1. Use velocity to get position previous.
	// 2. Have-at the position, moving away from the neighbours to some average position.
	// 3. New position - previous position = new velocity
	// 4. new velocity - old velocity =	acceleration
	float4 pos0 = target_pos;// - target.vel * Sim.TimeStep;
	float4 pos = target_pos; // .w is the weight
	//float4 grad = float4(0,0,0,0);
	
	//// Particles experience ghost pressure
	//if (target.surface.w < Sim.ParticleRadius)
	//{
	//	float4 pos_r = (2 * target.surface.w / Sim.ParticleRadius) * float4(target.surface.xyz, 0);
	////	pos += DisplacementAt(pos_r, Sim.Force);
	//	//grad += PressureAt(pos_r, Sim.Force);
	//}
	
	// Search the neighbours of 'target'
	FindIter find = Find(target_pos, SearchVolume(Sim.ParticleRadius, Sim.Dimensions), Sim.GridScale, Sim.CellCount);
	for (; DoFind(find, Sim.CellCount, m_idx_start, m_idx_count); )
	{
		for (int i = find.idx_range.x; i != find.idx_range.y; ++i)
		{
			// No self-interaction
			if (m_spatial[i] == dtid.x)
				continue;

			Particle neighbour = GetParticle(m_spatial[i]);
			float4 neighbour_pos = neighbour.pos + neighbour.vel * half_time_step;// + 0.5f * Sim.Gravity * sqr(half_time_step);
			float4 direction = neighbour_pos - target_pos;
			float separation = length(direction);

			// Viscosity
			if (true)
			{
				// Viscosity is inter-particle friction. When particles are close, their velocities should
				// tend toward the average of the group. Find the group average velocity (weighted by distance)
				float normalised_distance = separation / Sim.ParticleRadius;
				float visocity = ViscosityKernel(normalised_distance, Sim.Viscosity);
				group_velocity += visocity * neighbour.vel;
				group_velocity.w += visocity;
			}

			// Acceleration
			if (true)
			{
				pos += DisplacementAt(target_pos, neighbour, Sim.TimeStep, Sim.ParticleRadius, Sim.Force, rand_dir);
				//grad += PressureAt(target_pos, neighbour, Sim.ParticleRadius, Sim.Force);
			}
		}
	}

	// 'grad' is the net pressure gradient at 'target_pos'
	
	//pos += grad * 0.001f;

	// Find acceleration from the new position
	//pos /= pos.w;
	//float4 vel1 = (pos - pos0) / Sim.TimeStep;
	//target.acc += vel1 - target.vel;

	target.acc += (pos - pos0) / Sim.TimeStep;

	// Apply viscosity. Add an acceleration that makes the particles velocity closer to the group average
	if (true)
	{
		float4 target_vel = target.vel + 0.5f * target.acc * Sim.TimeStep;
		group_velocity /= group_velocity.w;
		target.acc += (group_velocity - target_vel) / Sim.TimeStep;
	}

	// Record the nett force
	m_dynamics[dtid.x].accel.xyz += target.acc.xyz;
}

// Apply an attractor to the particles
[numthreads(ThreadGroupSize, 1, 1)]
void ApplyProbe(uint3 dtid : SV_DispatchThreadID)
{
	if (dtid.x >= Probe.NumParticles)
		return;

	Particle target = GetParticle(dtid.x);

	// If the particle is within the probe radius apply a force toward the centre.
	// Scale by inverse distance so that the centre of the probe isn't high pressure.
	float4 r = target.pos - Probe.Position;
	float dist = length(r);
	if (WaveActiveOrThisThreadTrue(dist > Probe.Radius))
		return;

	float influence = saturate(dist / Probe.Radius); // 1 at range, 0 in the centre
	float4 direction = select(dist > 0.0001f, r / dist, float4(0, 0, 0, 0));
	float4 force = influence * (Probe.Force * direction - target.vel);
	m_dynamics[dtid.x].accel += force; // Probe ignores mass
}

// Remove NaN particles
[numthreads(ThreadGroupSize, 1, 1)]
void CullParticles(uint3 dtid : SV_DispatchThreadID)
{
	// Expect the spatial partitioning to have sorted the NaN particles to the end.
	// Rules:
	//  - If the src particle is NaN => done
	//  - If the src particle is not NaN
	//    -> If the dst index is output the dead range => Replace => done
	//    -> The dst index is in the dead range, look up the next dst index
	//       Repeat until outside the dead range.

	int src_idx = dtid.x;
	int dst_idx = m_spatial[dtid.x];
	int2 dead_range = IndexRange(Cull.CellCount - 1, m_idx_start, m_idx_count);

	// Return the new length of the particle buffer in 'm_output'
	if (dtid.x == 0)
		m_output[0].num_particles = Cull.NumParticles - (dead_range.y - dead_range.x);
	
	// If we're not in the dead range, nothing to do.
	if (src_idx < dead_range.x || src_idx >= dead_range.y)
		return;
	
	// If the source particle is dead then there is a dead particle in the dead range already. We're done
	if (any(isnan(m_positions[src_idx].pos)))
		return;

	// While 'dst_idx' is within the dead range, look up the next 'dst_idx'
	for (; dst_idx >= dead_range.x && dst_idx < dead_range.y; )
		dst_idx = m_spatial[dst_idx];
	
	// If the destination index is not within the dead range then replace and we're done
	if (dst_idx < dead_range.x || dst_idx >= dead_range.y)
	{
		m_positions[dst_idx] = m_positions[src_idx];
		m_dynamics[dst_idx] = m_dynamics[src_idx];
	}
}

// Apply colours to the particles
[numthreads(ThreadGroupSize, 1, 1)]
void ColourParticles(uint3 dtid : SV_DispatchThreadID)
{
	if (dtid.x >= Col.NumParticles)
		return;

	Particle target = GetParticle(dtid.x);
	m_positions[dtid.x].col = Col.Colours.Spectrum[0];

	// Colour the particles based on their velocity
	if (Col.Scheme == ColourScheme_Velocity)
	{
		float speed = length(target.vel);
		m_positions[dtid.x].col = LerpColour(speed, Col.Colours);
	}
	
	// Colour the particles based on their acceleration
	if (Col.Scheme == ColourScheme_Accel)
	{
		float accel = length(target.acc);
		m_positions[dtid.x].col = LerpColour(accel, Col.Colours);
	}

	//// Colour particles based on their density
	//if (Col.Scheme == ColourScheme_Density)
	//{
	//	float density = length(target.density);
	//	m_positions[dtid.x].col = LerpColour(density, Col.Colours);
	//}
}

// Populate a texture with each pixel representing the property at that point
[numthreads(32, 32, 1)]
void GenerateMap(uint3 dtid : SV_DispatchThreadID)
{
	if (dtid.x > Map.TexDim.x || dtid.y > Map.TexDim.y)
		return;
	
	// The position in world space of the texture coordinate
	float4 pos = mul(Map.MapToWorld, float4(dtid.x, dtid.y, 0, 1));// backwards? normally it's mul(vec, mat)
	float4 rand_dir = RandomNWithDim(float2(dtid.x, 0), Map.Dimensions);
	float4 value = float4(0,0,0,0);

	// Search the neighbours of 'pos'
	FindIter find = Find(pos, SearchVolume(Map.ParticleRadius, Map.Dimensions), Map.GridScale, Map.CellCount);
	while (DoFind(find, Map.CellCount, m_idx_start, m_idx_count))
	{
		for (int i = find.idx_range.x; i != find.idx_range.y; ++i)
		{
			//if (m_spatial[i] == 1)
			//	continue;
			
			Particle neighbour = GetParticle(m_spatial[i]);

			float4 direction = pos - neighbour.pos;
			float distance_sq = dot(direction, direction);
			if (distance_sq >= sqr(Map.ParticleRadius))
				continue;

			if (Map.Type == MapType_Velocity)
			{
				// Add up the weighted group velocity
				float distance = sqrt(distance_sq);
				direction = distance > 0.0001f ? direction / distance : rand_dir;
				value += lerp(neighbour.vel, float4(0,0,0,0), distance / Map.ParticleRadius) * direction;
			}
			if (Map.Type == MapType_Accel)
			{
				value += DisplacementAt(pos, neighbour, 0.01f, Map.ParticleRadius, Map.Force, rand_dir);
			}
			if (Map.Type == MapType_Density)
			{
				value.x += DensityAt(pos, neighbour, Map.ParticleRadius, Map.Dimensions);
			}
		}
	}
	
	// Write the density to the map
	m_tex_map[uint2(dtid.x, dtid.y)] = LerpColour(length(value), Map.Colours);
}

// Test function
[numthreads(1, 1, 1)]
void Debugging(uint3 dtid : SV_DispatchThreadID, uint3 gtid : SV_GroupThreadID)
{
	Particle p = GetParticle(0);
	
	float m = 0.51992292662535705707f;
	float nrg = 0.5f * m * length_sq(p.vel) + length(Sim.Gravity) * m * p.pos.y;
	m_output[0].p0_energy = nrg;
	
//	// Search the neighbours of the probe position
//	FindIter find = Find(Probe.Position, SearchVolume(Probe.Radius, Sim.Dimensions), Sim.GridScale, Sim.CellCount);
//	for (; DoFind(find, Sim.CellCount, m_idx_start, m_idx_count); )
//	{
//		for (int i = find.idx_range.x; i != find.idx_range.y; ++i)
//		{
//			Particle neighbour = GetParticle(m_spatial[i]);
//			float normalised_distance = length(neighbour.pos1 - Probe.Position) / Probe.Radius;
//			m_positions[m_spatial[i]].col = select(normalised_distance < 1, float4(0,1,0,1), float4(1,0,0,1));
//		}
//	}
}



#if 0 // not working

// Calculates a new position for 'target' as a result of interactive with 'neighbour'
inline float4 ParticleInteraction(uniform int idx, Particle target, Particle neighbour, float time_step)
{
	////// Rather than push particles apart, treat 'force' as a limit on the distance between particles.

	////// 1. Find the relative position of 'target' to 'neighbour'
	////// 2. Reduce acceleration toward the particle by the force limit at the distance
	////float4 pos_r = target.pos - neighbour.pos;
	////float4 acc_r = target.acc - neighbour.acc;
	
	////float distance = length(pos_r);
	////float force_limit = ForceKernel(distance / Sim.ParticleRadius, Sim.ForceRange, Sim.ForceBalance, Sim.ForceDip);
	
	////target.acc -= dot(acc_r, pos_r) * pos_r / sqr(distance);
	
	
	


	
	// Find the distance from 'neighbour' to 'target'
	float4 pos_r =
		target.pos + 0.5f * time_step * target.vel -
		neighbour.pos + 0.5f * time_step * neighbour.vel;
	
	float distance = length(pos_r);
	float4 direction = select(distance != 0, pos_r / distance, Random3WithDim(float2(idx, Sim.RandomSeed), Sim.Dimensions));
	
	// Push 'target' away from 'neighbour'. This is a 'force' in the star wars sense.
	float force = ForceKernel(distance / Sim.ParticleRadius, Sim.ForceRange, Sim.ForceBalance, Sim.ForceDip);
	float push = Sim.ForceScale * force * Sim.ParticleRadius;
	
	return target.pos + push * direction;
}

// Potential energy versus distance from a particle
inline float PotentialEnergy(float normalised_distance, uniform float range, uniform float balance, uniform float dip)
{
	// 'range' scales the repulsive force effectively making it act at greater range. [0,inf) (typically 1.0)
	// 'balance' is the position of the switch from the repulsive to the attractive force. [0,1] (typically ~0.8)
	// 'dip' is the depth of the attractive force [0,inf) (typically ~0.1
	// The function is a piecewise function created from:
	//  Repulsive force: (range * balance / x) * sqr(1 - x/balance), for x in [0,balance]
	//  Attractive force: -dip * sqr(sqr(Cx - C + 1) - 1), where C = 2 / (1 - balance), for x in [balance,1]
	float x = clamp(normalised_distance, 0.0001f, 1.0f);
	float C = 2.0f / (1.0f - balance);
	return select(x <= balance,
		+(range * balance / x) * sqr(1.0f - x / balance),
		-(dip) * sqr(sqr(C * x - C + 1) - 1));
}
inline float dPotentialEnergy(float normalised_distance, uniform float range, uniform float balance, uniform float dip)
{
	float x = clamp(normalised_distance, 0.0001f, 1.0f);
	float C = 2.0f / (1.0f - balance);
	float D = C * x - C + 1;
	return select(x <= range,
		+(balance / range) - (balance * range / sqr(x)),
		-(4 * C * dip) * D * (sqr(D) - 1));
}
inline float dForceKernal(float normalised_distance, uniform float range, uniform float balance, uniform float dip)
{
	// The slope of the Repulsive force is: (range/balance) - range * balance / sqr(x)
	// The slope of the attractive force is: -4 * dip * C * (C * x - C + 1) * (sqr(C * x - C + 1) - 1), where C = 2 / (1 - balance)
	float x = clamp(normalised_distance, 0.0001f, 1.0f);
	float C = 2.0f / (1.0f - balance);
	return select(x <= range,
		(range / balance) - (range * balance / sqr(x)),
		-4.0f * dip * C * (C * x - C + 1) * (sqr(C * x - C + 1) - 1));
}


// Calculates the acceleration experienced at 'target' as a result of the interaction between 'target' and 'neighbour'
inline float4 ParticleInteraction(uniform int idx, Particle target, Particle neighbour, float time_step)
#if 1
{
	// Concepts:
	//  - Using Lagrangian mechanics, so that energy is conserved.
	//    Total energy is the sum of kinetic and potential energy. E = KE(vel) + PE(pos)
	//    KE(vel) = 0.5.m.vel^2, PE(pos) = PotentialEnergy profile
	//    The force is the negative gradient of the potential energy. F = -dV(pos)/dx.
	//  - If we do the calculation relative to one of the particles, we only need to do half the calculations.
	//
	// Algorithm:
	//  1. Calculate the initial energy. Ei = KE(vel0) + PE(pos0)
	//  2. The energy at the end of the step, Ef = KE(vel1) + PE(pos1), should equal Ei
	//  3. Take a guess at the acceleration to apply
	//  4. Measure the new energy.
	//  5. Refine the guess based on energy error

	// As the particle goes from vel0 to vel1 (t = [0,1] there will be a time at which to apply the acceleration that
	// results in conserved energy. We just need to find 't'

	// Get the relative position and velocity. Treat 'target' as stationary.
	float4 pos0 = neighbour.pos - target.pos;
	float4 vel0 = neighbour.vel - target.vel;
	
	float nrg0 = 0.5f * length_sq(vel0) + PotentialEnergy(length(pos0) / Sim.ParticleRadius, Sim.ForceRange, Sim.ForceBalance, Sim.ForceDip);

	float tmax = 1.0f, tmin = 0.0f;
	for (int attempts = 10; ; --attempts)
	{
		float t = 0.5f * (tmax + tmin);
		float4 pos = pos0 + vel0 * time_step * t;
		float4 vel1 = vel0;
	
		// Apply the impulse at time 't' (mass = 1)
		float distance = length(pos);
		float accel = -dPotentialEnergy(distance / Sim.ParticleRadius, Sim.ForceRange, Sim.ForceBalance, Sim.ForceDip);
		vel1 += accel * select(distance != 0, pos / distance, float4(0,0,0,0));
		
		float4 pos1 = pos + vel1 * time_step * (1 - t);
		float nrg1 = 0.5f * length_sq(vel1) + PotentialEnergy(length(pos1) / Sim.ParticleRadius, Sim.ForceRange, Sim.ForceBalance, Sim.ForceDip);

		if (abs(nrg1 - nrg0) < 0.01f || attempts == 0)
		{
			// 'vel1' is the expected final relative velocity of 'neighbour'. 'vel1 - vel0' is an acceleration that gives this result.
			// The acceleration is shared between both particles, and we're returning the acceleration for 'target'.
			return select(attempts != 0, -0.5f * (vel1 - vel0), float4(0,0,0,0));
		}
		
		if (nrg1 > nrg0)
			tmax = t;
		else
			tmin = t;
	}
}
#else
{
	// Integration stability is an issue because the force kernel is asymptotic at x = 0.
	// Concepts:
	//  - If we had infinitesimal 'dt' steps, the acceleration would fall sharply as the particles moved apart.
	//  - If we do the calculation relative to one of the particles, we only need to do half the calculations.
	//  - For each change in distance between the particles, the relative velocity changes by the area under the
	//    ForceKernel curve, evaluated from dist0 to dist1 (direction is centre to centre).
	//  - We can stop when the particle velocity is moving away and the range is greater than the particle radius.
	//
	// Constrain the step size based on distance travelled, whichever is smaller min(vel * dt, ParticleRadius / N).
	// For each step, we want to integrate the ForceKernel based on the distance between the particles.
	
	const float SubSteps = 10.0f;
	const float MaxStepLength = Sim.ParticleRadius / SubSteps;
	const float MaxStepTime = time_step / SubSteps;
	
	// Get the relative position and velocity. Treat 'target' as stationary.
	float4 pos0 = neighbour.pos - target.pos;
	float4 vel0 = neighbour.vel - target.vel;
	float4 pos = pos0;
	float4 vel = vel0;
	float time_remaining = time_step;

	// Integrate the force kernel over the step
	for (int max_steps = 100; max_steps-- != 0 && time_remaining != 0;)
	{
		// If the particle is moving away and out of range, then there is no more influence from it
		float dist_sq = length_sq(pos);
		if (dot(pos, vel) >= 0 && dist_sq >= sqr(Sim.ParticleRadius))
			break;

		// Determine a step time / size
		float dt = min(MaxStepTime, time_remaining);
		dt = select(length_sq(vel * dt) > sqr(MaxStepLength), MaxStepLength / length(vel), dt);

		// Integrate the force kernel over the step
		{
			float4 hstep = pos + 0.5f * vel * dt;
			float hstep_dist = length(hstep);
			float force = ForceKernel(hstep_dist / Sim.ParticleRadius, Sim.ForceRange, Sim.ForceBalance, Sim.ForceDip) * Sim.ForceScale;
			float4 direction = select(hstep_dist != 0, hstep / hstep_dist, Random3WithDim(float2(idx, Sim.RandomSeed), Sim.Dimensions));
			vel += force * /*target.density * neighbour.density **/ direction;
		}

		// Advance the particles by the step
		pos += vel * dt;
		time_remaining -= dt;
	}

	// 'vel' is the expected final relative velocity of 'neighbour'. 'vel - vel0' is an acceleration that gives this result.
	// The acceleration is shared between both particles, and we're returning the acceleration for 'target'.
	return -0.5f * (vel - vel0);
}
#endif
// Slim representation of a neighbour
struct Neighbour
{
	float4 pos;
	float4 vel;
};

// Shared memory for buffering neighbours
static const int MaxNeighbours = 30;
groupshared Neighbour gs_neighbours[ThreadGroupSize][MaxNeighbours];


// Calculates the acceleration experienced at 'target' as a result of the interaction between 'target' and its neighbours
inline float4 ParticleInteractions(uniform int idx, uniform int gidx, int neighbour_count, Particle target, float time_step)
{
	// Integration stability is an issue because the force kernel is asymptotic at x = 0.
	// Concepts:
	//  - If we had infinitesimal 'dt' steps, the acceleration would fall sharply as the particles moved apart.
	//  - For each change in distance between the particles, the relative velocity changes by the area under the
	//    ForceKernel curve, evaluated from dist0 to dist1 (direction is centre to centre).
	//  - We can stop when the particle velocity is moving away and the range is greater than the particle radius.
	//  - For particles near a boundary, every particle interaction also has a virtual particle reflected through
	//    the boundary surface.
	//  - Particles on a boundary (i.e. distance zero) only experience force from other zero-distance particles.
	//
	// Constrain the step size based on distance travelled, whichever is smaller min(vel * dt, ParticleRadius / N).
	// For each step, we want to integrate the ForceKernel based on the distance between the particles.
	
	const float SubSteps = 10.0f;
	const float MaxStepLength = Sim.ParticleRadius / SubSteps;
	const float MaxStepTime = time_step / SubSteps;
	
	float4 pos0 = target.pos;
	float4 vel0 = target.vel;
	int i;

	// Advance the particle and its neighbours using sub-steps
	float time_remaining = time_step;
	for (int max_steps = 100; max_steps-- != 0 && time_remaining != 0;)
	{
		// Determine a step time / size
		float dt = min(MaxStepTime, time_remaining);
		for (i = 0; i != neighbour_count; ++i)
		{
			float4 pos = gs_neighbours[gidx][i].pos - pos0;
			float4 vel = gs_neighbours[gidx][i].vel - vel0;

			// If the particle is out of range, skip it 
			if (length_sq(pos) > sqr(Sim.ParticleRadius))
				continue;
			
			// Set the step size based on the largest relative velocity
			dt = select(length_sq(vel * dt) > sqr(MaxStepLength), MaxStepLength / length(vel), dt);
		}

		// Calculate the change of velocity experienced by 'target'
		float4 nett_vel = vel0;
		for (i = 0; i != neighbour_count; ++i)
		{
			float4 pos = gs_neighbours[gidx][i].pos - pos0;
			float4 vel = gs_neighbours[gidx][i].vel - vel0;
			
			// If the particle is out of range, skip it 
			if (length_sq(pos) >= sqr(Sim.ParticleRadius))
				continue;

			// Integrate the force kernel over the step
			float4 hstep = pos + 0.5f * vel * dt; // remember, 'pos' is relative to 'pos0' so it's also a direction vector
			float distance = length(hstep);
			
			// Measure the force between the particles
			float force = Sim.ForceScale * ForceKernel(distance / Sim.ParticleRadius, Sim.ForceRange, Sim.ForceBalance, Sim.ForceDip);
			float4 direction = select(distance != 0, hstep / distance, Random3WithDim(float2(idx, Sim.RandomSeed), Sim.Dimensions));

			// Apply half the acceleration to each particle
			float4 accel = (0.5f * force * dt) * direction;
			gs_neighbours[gidx][i].vel += accel;
			nett_vel -= accel;
		}

		// Advance the particles by the sub-step
		vel0 = nett_vel;
		pos0 += vel0 * dt;
		for (int i = 0; i != neighbour_count; ++i)
		{
			gs_neighbours[gidx][i].pos += gs_neighbours[gidx][i].vel * dt;
		}

		// Advance time
		time_remaining -= dt;
	}

	// 'vel0' is the velocity we predict 'target' to have, so 'vel0 - target.vel' is the acceleration that gives this result.
	return vel0 - target.vel;
}


	// Build a list of neighbours in the group shared memory
	int neighbour_count = 0;
	FindIter find = Find(target_pos, SearchVolume(Sim.ParticleRadius, Sim.Dimensions), Sim.GridScale, Sim.CellCount);
	while (DoFind(find, Sim.CellCount) && neighbour_count != MaxNeighbours)
	{
		for (int i = find.idx_range.x; i != find.idx_range.y; ++i)
		{
			// No self-interaction
			if (m_spatial[i] == dtid.x)
				continue;

			// Range check
			Particle neighbour = GetParticle(m_spatial[i]);
			float4 neighbour_pos = neighbour.pos + 0.5f * Sim.TimeStep * neighbour.vel;
			if (length_sq(target_pos - neighbour_pos) >= sqr(Sim.ParticleRadius))
				continue;

			// Add a neighbour to be considered
			gs_neighbours[gtid.x][neighbour_count].pos = neighbour.pos;
			gs_neighbours[gtid.x][neighbour_count].vel = neighbour.vel;
			if (++neighbour_count == MaxNeighbours)
			{
				nett_accel += ParticleInteractions(dtid.x, gtid.x, neighbour_count, target, Sim.TimeStep);
				neighbour_count = 0;
			}
		}
	}
	
	// Interact with each neighbour
	//for (int i = 0; i != neighbour_count; ++i)
	//	nett_accel += ParticleInteraction(dtid.x, gtid.x, neighbour_count, target, Sim.TimeStep);
	nett_accel += ParticleInteractions(dtid.x, gtid.x, neighbour_count, target, Sim.TimeStep);
	#endif





			#if 0 // Good version
			{
				// Acceleration integration sub-step 
				const int sub_steps = 5;
				const float sub_step = Sim.TimeStep / sub_steps;

				float4 t_pos = target.pos;
				float4 n_pos = neighbour.pos;
				float4 t_vel = target.vel;
				float4 n_vel = neighbour.vel;

				// Calculate sub-steps for a more accurate acceleration calculation
				[unroll] for (int k = 0; k != sub_steps; ++k)
				{
					// Advance the particles by a half step
					float4 t_pos_half_step = t_pos + 0.5f * sub_step * t_vel;
					float4 n_pos_half_step = n_pos + 0.5f * sub_step * n_vel;

					// Get the separating vector and distance
					float4 sep = t_pos_half_step - n_pos_half_step;
					float distance = length(sep);
					float4 direction = select(distance != 0, sep / distance, Random3WithDim(float2(dtid.x, Sim.RandomSeed), Sim.Dimensions)); // points at target

					// Calculate the acceleration due to particle forces
					float force = ForceKernel(distance / Sim.ParticleRadius, Sim.ForceRange, Sim.ForceBalance, Sim.ForceDip);
					float accel = Sim.ForceScale * target.density * neighbour.density * force;

					// TODO Accel based sub step size
					
					// If a particle is on the boundary, then it can't be accelerated into the boundary surface.
					// Instead the acceleration is reflected
					float4 t_accel = +accel * direction;
					float4 n_accel = -accel * direction;

					t_accel -= max(0, 2 * dot(t_accel, target.surface)) * target.surface;
					n_accel -= max(0, 2 * dot(n_accel, neighbour.surface)) * neighbour.surface;
					
					// Update the particle velocities by the acceleration
					t_vel += sub_step * t_accel;
					n_vel += sub_step * n_accel;

					// Advance the particles by a sub step
					t_pos += t_vel * sub_step;
					n_pos += n_vel * sub_step;
				}
			
				// 't_pos' and 't_vel' is where the particle should end up, calculate an acceleration that gives this result
				nett_accel += t_vel - target.vel;
			}
			#endif