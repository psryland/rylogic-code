//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#include "src/compute/particle_collision/collision.hlsli"
#include "src/compute/particle_collision/particle.hlsli"

static const int ThreadGroupSize = 1024;
static const int MaxCollisionResolutionSteps = 10;

cbuffer cbCollision : register(b0)
{
	struct
	{
		// Note: Alignment matters. float2 must be aligned to 8 bytes.
		int NumParticles; // The number of particles
		int NumPrimitives; // The number of primitives
		int SpatialDimensions; // The number of spatial dimensions
		float TimeStep; // The time to advance each particle by
	
		float ParticleRadius; // The radius of volume that each particle represents
		float pad;

		float2 Restitution; // The coefficient of restitution (normal, tangential)
	} Sim;
};
cbuffer cbBoundary : register(b0)
{
	struct
	{
		int NumParticles; // The number of particles
		int NumPrimitives; // The number of primitives
		int SpatialDimensions; // The number of spatial dimensions
		float ParticleRadius; // The radius of volume that each particle represents
	} Bound;
};
cbuffer cbCull : register(b0)
{
	struct
	{
		float4 Geom[2];   // A plane, sphere, etc used to cull particles (set their positions to NaN)
		int Flags;        // [3:0] = 0: No culling, 1: Sphere, 2: Plane, 3: Box
		int NumParticles; // The number of particles
	} Cull;
};

static const int Cull_None = 0;         // Cull no used
static const int Cull_Plane = 1;        // Cull[0].xyz = normal, Cull[0].w = distance
static const int Cull_Sphere = 2;       // Cull[0].xyz = centre, Cull[0].w = radius
static const int Cull_SphereInside = 3; // Cull[0].xyz = centre, Cull[0].w = radius
static const int Cull_Box = 4;          // Cull[0].xyz = centre, Cull[1].xyz = half-extents
static const int Cull_BoxInside = 5;    // Cull[0].xyz = centre, Cull[1].xyz = half-extents
static const int Cull_Mask = 0x07;
static const float4 CulledPosition = float4(0,0,0,0)/0;

// The positions/colours of each particle
RWStructuredBuffer<PositionType> m_positions : register(u0);

// The dynamics for each particle
RWStructuredBuffer<DynamicsType> m_dynamics : register(u1);

// The primitives to collide against
StructuredBuffer<Prim> m_collision : register(t0);

// Return a particle from the particle and dynamics buffers
inline Particle GetParticle(int i)
{
	return ToParticle(i, m_positions, m_dynamics);
}

// Test a position for being culled
inline float4 CullCheck(float4 pos, uniform int cull_mode)
{
	// Note: the boundary counts as "not culled"
	switch (cull_mode)
	{
		case Cull_None:
		{
			return pos;
		}
		case Cull_Plane:
		{
			// Cull if below the plane
			return dot(pos, Cull.Geom[0]) >= 0 ? pos : CulledPosition;
		}
		case Cull_Sphere:
		{
			// Cull if outside the sphere
			float r_sq = sqr(Cull.Geom[0].w);
			float3 r = pos.xyz - Cull.Geom[0].xyz;
			return dot(r,r) <= r_sq ? pos : CulledPosition;
		}
		case Cull_SphereInside:
		{
			// Cull if inside the sphere
			float r_sq = sqr(Cull.Geom[0].w);
			float3 r = pos.xyz - Cull.Geom[0].xyz;
			return dot(r,r) >= r_sq ? pos : CulledPosition;
		}
		case Cull_Box:
		{
			// Cull if outside the box
			float3 r = abs(pos.xyz - Cull.Geom[0].xyz) - Cull.Geom[1].xyz;
			return all(r <= 0) ? pos : CulledPosition;
		}
		case Cull_BoxInside:
		{
			// Cull if inside the box
			float3 r = abs(pos.xyz - Cull.Geom[0].xyz) - Cull.Geom[1].xyz;
			return any(r >= 0) ? pos : CulledPosition;
		}
		default:
		{
			return pos;
		}
	}
}

// Evolves the particles forward in time while resolving collisions
numthreads(Integrate, ThreadGroupSize, 1, 1)
void Integrate(int3 dtid : SV_DispatchThreadID)
{
	if (dtid.x >= Sim.NumParticles)
		return;

	Particle target = GetParticle(dtid.x);

	// Assume the acceleration is constant over the time step.
	float4 pos1 = target.pos + target.vel * Sim.TimeStep + 0.5f * target.acc * sqr(Sim.TimeStep);
	float4 vel1 = target.vel + target.acc * Sim.TimeStep;
	float4 vel0 = target.vel;

	// The ray to the next position of the particle (updated by collisions)
	float4 ray = pos1 - target.pos;
	float4 acc = target.acc;
	float dt = Sim.TimeStep;

	// Integrate position and velocity and resolve collisions.
	// The ApplyForces function should calculate an acceleration that represents the average for the time step.
	// When collisions change the velocity direction, the acceleration shouldn't also change direction.
	for (int attempt = 0; ; ++attempt)
	{	
		// Find the nearest intercept
		float t = 1.0f;
		int intercept_found = 0;
		float4 normal = float4(0, 0, 0, 0);
		for (int i = 0; i != Sim.NumPrimitives; ++i)
			intercept_found |= Intercept_RayVsPrimitive(target.pos, ray, m_collision[i], normal, t);

		// Stop if no intercept found or too many collisions
		bool done = !intercept_found || attempt == MaxCollisionResolutionSteps;
		if (WaveActiveOrThisThreadTrue(done))
		{
			target.pos += ray * t;
			target.vel = attempt != MaxCollisionResolutionSteps ? vel1 : float4(0,0,0,0);
			
			// Use for highlighting stuck particles
			#if SHOW_STUCK_PARTICLES
			target.colour = attempt != MaxCollisionResolutionSteps ? target.colour : float4(1,0,1,1);
			#endif
			break;
		}
		
		// Constrain to 2D
		normal.z = Sim.SpatialDimensions == 3 ? normal.z : 0;
		
		// Advance the point to the intercept
		target.pos += ray * t;

		// Recalculate vel0 at the collision point
		vel0 = vel0 + acc * dt * t;
		
		// Calculate the remaining time
		dt = dt * (1 - t);

		// Reflect the velocity + apply restitution *before* applying acceleration to find 'vel1' and the reflected 'ray'.
		// Be careful not to add the acceleration first and then reflect it, accel should not be reflected.
		float4 vel_n = -dot(vel0, normal) * normal;
		float4 vel_t = vel0 + vel_n;
		vel0 = vel_n * Sim.Restitution.x + vel_t * Sim.Restitution.y;
		vel1 = vel0 + acc * dt;
		ray = vel1 * dt + 0.5f * acc * sqr(dt);

		// If the particle's reflected velocity plus acceleration has it still
		// moving into the surface, then the particle comes to rest on the surface.
		if (dot(vel1, normal) < 0)
		{
			// Remove the normal component of the velocity
			vel0 -= dot(vel0, normal) * normal;
			vel1 -= dot(vel1, normal) * normal;

			// Remove the normal component of the acceleration
			acc -= dot(acc, normal) * normal;

			// Use for testing boundaries
			#if SHOW_STUCK_PARTICLES
			target.colour = float4(0,1,0,1);
			#endif
		}
	}

	// Update the particle dynamics
	m_positions[dtid.x].pos.xyz = target.pos.xyz;
	m_dynamics[dtid.x].vel.xyz = target.vel.xyz;
	m_dynamics[dtid.x].accel.xyz = float4(0,0,0,0).xyz;
	#if SHOW_STUCK_PARTICLES
	m_positions[dtid.x].col = target.colour;
	#endif
}

// Find nearby surfaces for particles
numthreads(DetectBoundaries, ThreadGroupSize, 1, 1)
void DetectBoundaries(int3 dtid : SV_DispatchThreadID)
{
	if (dtid.x >= Bound.NumParticles)
		return;

	Particle target = GetParticle(dtid.x);

	//target.surface = float4(0,0,0,Bound.ParticleRadius);
	//for (int i = 0; i != Bound.NumPrimitives; ++i)
	//{
	//	float4 normal;
	//	float4 range = target.pos - ClosestPoint_PointToPrimitive(target.pos, m_collision[i], normal);
	//	float distance = length(range);
	//	target.surface = select(distance < target.surface.w, float4(normal.xyz, distance), target.surface);
	//}
	
	//// Constrain to 2D
	//target.surface.z = select(Bound.SpatialDimensions == 3, target.surface.z, 0);

	// Measure proximity to the nearest boundary
	float4 boundary_normal = float4(0,0,0,0);
	float boundary_distance = Bound.ParticleRadius;
	for (int i = 0; i != Bound.NumPrimitives; ++i)
	{
		float4 normal;
		float4 range = target.pos - ClosestPoint_PointToPrimitive(target.pos, m_collision[i], normal);
		float distance = length(range);
		
		float weight = clamp(1 - distance / Bound.ParticleRadius, 0, 1);
		boundary_normal += weight * normal;
		boundary_distance = min(boundary_distance, distance);
	}

	// Constrain to 2D
	target.surface = any(boundary_normal) ? float4(normalize(boundary_normal).xyz, boundary_distance) : float4(0,0,0,Bound.ParticleRadius);
	target.surface.z = Bound.SpatialDimensions == 3 ? target.surface.z : 0;

	m_dynamics[dtid.x].surface = target.surface;
}

// Mark culled particles with NaN positions
numthreads(CullDeadParticles, ThreadGroupSize, 1, 1)
void CullDeadParticles(int3 dtid : SV_DispatchThreadID)
{
	if (dtid.x >= Cull.NumParticles)
		return;	

	float4 pos = float4(m_positions[dtid.x].pos.xyz, 1);
	pos = CullCheck(pos, Cull.Flags & Cull_Mask);
	m_positions[dtid.x].pos.xyz = pos.xyz;
}