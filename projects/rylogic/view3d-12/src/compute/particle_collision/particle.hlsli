//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
// Defines:
//   POSITION_TYPE - Define the element structure of the 'positions' buffer. The field 'pos' is
//      expected to be the position information. It can be float3 or float4.
//   DYNAMICS_TYPE - Define the element structure of the 'dynamics' buffer.
#ifndef PARTICLE_HLSLI
#define PARTICLE_HLSLI

// Custom type for position data
#ifndef POSITION_TYPE
#define POSITION_TYPE \
struct PositionType \
{ \
	float4 pos; \
	float4 col; \
}
#endif
POSITION_TYPE;

// Custom type for dynamics data
// 'vel' is the velocity of the particle
// 'accel' is the accumulated acceleration to apply to the particle
// 'surface' is a plane estimating any boundary the particle is next to
#ifndef DYNAMICS_TYPE
#define DYNAMICS_TYPE \
struct DynamicsType \
{ \
	float4 vel; \
	float4 accel; \
	float4 surface; \
}
#endif
DYNAMICS_TYPE;

// A struct that collects all the information about a particle
struct Particle
{
	float4 pos;     // Current position
	float4 vel;     // Current velocity
	float4 acc;     // Accumulated acceleration
	float4 surface; // This is a plane equation of a nearby surface
	float4 colour;  // Colour of the particle
};

// Return a particle from the particle and dynamics buffers
inline Particle ToParticle(int i, RWStructuredBuffer<PositionType> positions, RWStructuredBuffer<DynamicsType> dynamics)
{
	Particle part;
	part.pos = float4(positions[i].pos.xyz, 1);
	part.vel = float4(dynamics[i].vel.xyz, 0);
	part.acc = float4(dynamics[i].accel.xyz, 0);
	part.surface = dynamics[i].surface;
	part.colour = positions[i].col;
	return part;
}

#endif
