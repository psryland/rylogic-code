//************************************
// Physics Engine
//  Copyright (c) Rylogic Ltd 2026
//************************************
// Analytic water buoyancy and drag for rigid bodies. See gpu_water_forces.h.

#include "pr/hlsl/core.hlsli"
#include "pr/hlsl/interop.hlsli"
#include "pr/hlsl/vector.hlsli"
#include "physics/src/compute/physics_types.hlsli"
#include "pr/physics/terrain/water/water_field.hlsli"

#define WATER_FORCES_THREAD_COUNT 64
#define WATER_PROXY_SPHERE 0
#define WATER_PROXY_BOX 1
#define WATER_BOX_CELLS 4

// Must match 'CBufWaterForces' in gpu_water_forces.cpp.
struct CBufWaterForces
{
	int candidate_count;
	int element_count;
	float time_s;
	float dt;
	float water_level;
	float density;
	float linear_drag_rate;
	float quadratic_drag_coefficient;
	float angular_drag_rate;
};

// Must match 'GpuWaterForces::Candidate'. Positions and axes are in the body's model space.
struct GpuWaterCandidate
{
	int body_index;
	int proxy;
	float volume;
	float pad;
	float4 centre_os;
	float4 extent_os;
	float4 axis_x_os;
	float4 axis_y_os;
	float4 axis_z_os;
};

ConstantBuffer<CBufWaterForces> resource(g, b0);
RWStructuredBuffer<GpuRigidBody> resource(g_bodies, u0);
StructuredBuffer<GpuWaterCandidate> resource(g_candidates, t0);
StructuredBuffer<WaterFieldElement> resource(g_elements, t1);

// Submerged volume and the world-space centre of the submerged part.
struct Submerged
{
	float volume;
	float3 centre_ws;
};

// Return the exact submerged part of a sphere below the plane through 'surface_ws' with unit normal 'up'.
Submerged SubmergedSphere(float3 centre_ws, float radius, float3 surface_ws, float3 up)
{
	// The cap height is how far the sphere reaches below the plane. The cap centroid lies 3(2r-h)²/(4(3r-h)) below the sphere centre.
	Submerged result;
	float height = clamp(radius - dot(centre_ws - surface_ws, up), 0.0f, 2.0f * radius);
	result.volume = 0.5f * tau * height * height * (3.0f * radius - height) / 3.0f;
	float offset = height > 0.0f ? 3.0f * (2.0f * radius - height) * (2.0f * radius - height) / (4.0f * (3.0f * radius - height)) : radius;
	result.centre_ws = centre_ws - offset * up;
	return result;
}

// Return the submerged part of an oriented box below the plane through 'surface_ws' with unit normal 'up'.
// The box is split into cells, and each cell's filled fraction is linear in its depth across its thickness along 'up'.
// This is exact for a box that is fully wet or fully dry, and a smooth close approximation while the box crosses the surface.
Submerged SubmergedBox(float3 centre_ws, float3 half_ws[3], float3 surface_ws, float3 up)
{
	// Measure the thickness of one cell along the plane normal.
	float3 cell_half[3] = { half_ws[0] / WATER_BOX_CELLS, half_ws[1] / WATER_BOX_CELLS, half_ws[2] / WATER_BOX_CELLS };
	float cell_thickness = abs(dot(cell_half[0], up)) + abs(dot(cell_half[1], up)) + abs(dot(cell_half[2], up));
	float cell_volume = 8.0f * length(cell_half[0]) * length(cell_half[1]) * length(cell_half[2]);

	// Accumulate the wet volume and its first moment over all cells.
	Submerged result;
	result.volume = 0.0f;
	float3 moment = float3(0.0f, 0.0f, 0.0f);
	for (int k = 0; k != WATER_BOX_CELLS; ++k)
	{
		for (int j = 0; j != WATER_BOX_CELLS; ++j)
		{
			for (int i = 0; i != WATER_BOX_CELLS; ++i)
			{
				// Cells are centred at odd multiples of the cell half size, measured from the box centre.
				float3 cell_centre = centre_ws
					+ (2.0f * i + 1.0f - WATER_BOX_CELLS) * cell_half[0]
					+ (2.0f * j + 1.0f - WATER_BOX_CELLS) * cell_half[1]
					+ (2.0f * k + 1.0f - WATER_BOX_CELLS) * cell_half[2];
				float depth = dot(surface_ws - cell_centre, up);
				float fill = cell_thickness > 0.0f ? saturate((depth + cell_thickness) / (2.0f * cell_thickness)) : (depth > 0.0f ? 1.0f : 0.0f);
				float wet = fill * cell_volume;
				result.volume += wet;
				moment += wet * (cell_centre - (1.0f - fill) * cell_thickness * up);
			}
		}
	}
	result.centre_ws = result.volume > 0.0f ? moment / result.volume : centre_ws;
	return result;
}

// Apply buoyancy and drag to one candidate body for one substep.
numthreads(CSWaterForces, WATER_FORCES_THREAD_COUNT, 1, 1)
void CSWaterForces(uint3 dtid : SV_DispatchThreadID)
{
	// One thread per candidate; each candidate is a distinct body.
	if ((int)dtid.x >= g.candidate_count)
		return;

	GpuWaterCandidate candidate = g_candidates[dtid.x];
	GpuRigidBody body = g_bodies[candidate.body_index];
	float gravity = length(body.ws_gravity.xyz);

	// Place the proxy in world space. o2w rows are the model basis vectors and position.
	float3 centre_ws = mul(float4(candidate.centre_os.xyz, 1.0f), body.o2w).xyz;
	float3 axis_x_ws = mul(float4(candidate.axis_x_os.xyz, 0.0f), body.o2w).xyz;
	float3 axis_y_ws = mul(float4(candidate.axis_y_os.xyz, 0.0f), body.o2w).xyz;
	float3 axis_z_ws = mul(float4(candidate.axis_z_os.xyz, 0.0f), body.o2w).xyz;
	float3 com_ws = mul(float4(body.os_com_and_invmass.xyz, 1.0f), body.o2w).xyz;

	// Sample the water surface at the proxy centre and treat it locally as a plane.
	// Lateral pressure gradients from wave motion are sampled at the same point.
	float3 surface = float3(g.water_level, 0.0f, 0.0f);
	float2 pressure_gradient = float2(0.0f, 0.0f);
	for (int e = 0; e != g.element_count; ++e)
	{
		WaterFieldElement element = g_elements[e];
		surface += WaterFieldElementHeightAndGradient(element, centre_ws.xy, g.time_s);
		if (gravity > 0.0f)
			pressure_gradient += WaterFieldElementHeightAndPressureGradient(element, centre_ws.xy, g.time_s, gravity).yz;
	}
	float3 up = normalize(float3(-surface.y, -surface.z, 1.0f));
	float3 surface_ws = float3(centre_ws.xy, surface.x);

	// Find the submerged volume and its centre. Other shapes use a box proxy scaled to their true volume.
	Submerged wet;
	float proxy_volume;
	if (candidate.proxy == WATER_PROXY_SPHERE)
	{
		wet = SubmergedSphere(centre_ws, candidate.extent_os.x, surface_ws, up);
		proxy_volume = 2.0f / 3.0f * tau * candidate.extent_os.x * candidate.extent_os.x * candidate.extent_os.x;
	}
	else
	{
		float3 half_ws[3] = { axis_x_ws * candidate.extent_os.x, axis_y_ws * candidate.extent_os.y, axis_z_ws * candidate.extent_os.z };
		wet = SubmergedBox(centre_ws, half_ws, surface_ws, up);
		proxy_volume = 8.0f * candidate.extent_os.x * candidate.extent_os.y * candidate.extent_os.z;
	}
	if (wet.volume <= 0.0f || proxy_volume <= 0.0f)
		return;

	float fraction = saturate(wet.volume / proxy_volume);
	float volume = fraction * candidate.volume;

	// Buoyancy opposes gravity with the displaced water's weight, tilted by the wave pressure gradient.
	float3 force = g.density * gravity * volume * float3(-pressure_gradient.x, -pressure_gradient.y, 1.0f);

	// Linear drag acts on the velocity relative to the water at the centre of buoyancy.
	// The linear term is an exact exponential decay over the substep and the quadratic term is limited to the remaining relative momentum,
	// so drag can slow the body to the water's velocity but never reverse its relative motion.
	float inv_mass = body.os_com_and_invmass.w;
	float3 water_velocity = float3(0.0f, 0.0f, 0.0f);
	for (int v = 0; v != g.element_count; ++v)
		water_velocity += WaterFieldElementVelocity(g_elements[v], wet.centre_ws, g.time_s, g.water_level);

	float3 relative_momentum = body.momentum_lin.xyz - water_velocity / inv_mass;
	float linear_decay = 1.0f - exp(-g.linear_drag_rate * fraction * g.dt);
	float3 drag = -relative_momentum * linear_decay / g.dt;
	float3 remaining = relative_momentum + drag * g.dt;
	float speed = length(remaining) * inv_mass;
	float area = pow(volume, 2.0f / 3.0f);
	float quadratic = 0.5f * g.density * g.quadratic_drag_coefficient * area * speed * speed;
	float quadratic_limit = length(remaining) / g.dt;
	if (speed > 0.0f)
		drag -= normalize(remaining) * min(quadratic, quadratic_limit);

	// Angular drag is an exact exponential decay of the body's angular momentum over the substep.
	float angular_decay = 1.0f - exp(-g.angular_drag_rate * fraction * g.dt);
	float3 angular_drag = -body.momentum_ang.xyz * angular_decay / g.dt;

	// Buoyancy acts at the centre of buoyancy; drag acts through the centre of mass so it cannot inject spin.
	float3 torque = cross(wet.centre_ws - com_ws, force) + angular_drag;
	body.force_lin.xyz += force + drag;
	body.force_ang.xyz += torque;
	g_bodies[candidate.body_index].force_lin = body.force_lin;
	g_bodies[candidate.body_index].force_ang = body.force_ang;
}
