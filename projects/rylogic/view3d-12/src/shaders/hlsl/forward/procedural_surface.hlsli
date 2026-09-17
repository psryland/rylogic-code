//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2026
//*********************************************
#ifndef PR_VIEW3D_PROCEDURAL_SURFACE_HLSLI
#define PR_VIEW3D_PROCEDURAL_SURFACE_HLSLI

// Surface channels returned by the shared procedural evaluator.
struct ProceduralSurfaceSample
{
	// Channels share one noise evaluation so their visible structure remains coherent.

	float3 albedo;
	float height;
	float roughness;
};

// Mix one integer cell coordinate into a deterministic 32-bit value.
uint ProceduralHash(uint value)
{
	// Avalanche every input bit across the result with integer-only operations.
	value ^= value >> 16;
	value *= 0x7feb352du;
	value ^= value >> 15;
	value *= 0x846ca68bu;
	value ^= value >> 16;
	return value;
}

// Return a deterministic scalar for an integer lattice point and material seed.
float ProceduralLattice(int3 cell, uint seed)
{
	// Fold all three wrapped cell coordinates and the caller seed into one scalar.
	uint value = ProceduralHash(asuint(cell.x) ^ seed);
	value = ProceduralHash(value ^ asuint(cell.y));
	value = ProceduralHash(value ^ asuint(cell.z));
	return (value >> 8) * (1.0f / 16777215.0f);
}

// Evaluate smooth non-periodic value noise from a wrapped base cell and a cell-local coordinate.
float ProceduralValueNoise(int3 base_cell, float3 coordinate, uint seed)
{
	// Split the sample into its integer cell and smooth within-cell interpolation coordinate.
	float3 cell_floor = floor(coordinate);
	int3 cell = base_cell + int3(cell_floor);
	float3 fraction = coordinate - cell_floor;
	float3 blend = fraction * fraction * (3.0f - 2.0f * fraction);

	float n000 = ProceduralLattice(cell + int3(0, 0, 0), seed);
	float n100 = ProceduralLattice(cell + int3(1, 0, 0), seed);
	float n010 = ProceduralLattice(cell + int3(0, 1, 0), seed);
	float n110 = ProceduralLattice(cell + int3(1, 1, 0), seed);
	float n001 = ProceduralLattice(cell + int3(0, 0, 1), seed);
	float n101 = ProceduralLattice(cell + int3(1, 0, 1), seed);
	float n011 = ProceduralLattice(cell + int3(0, 1, 1), seed);
	float n111 = ProceduralLattice(cell + int3(1, 1, 1), seed);
	float nx00 = lerp(n000, n100, blend.x);
	float nx10 = lerp(n010, n110, blend.x);
	float nx01 = lerp(n001, n101, blend.x);
	float nx11 = lerp(n011, n111, blend.x);
	return lerp(lerp(nx00, nx10, blend.y), lerp(nx01, nx11, blend.y), blend.z);
}

// Evaluate a compact four-octave field without texture lookup or finite spatial extent.
float ProceduralFbm(int3 base_cell, float3 coordinate, uint seed, float detail)
{
	// Accumulate integer-scaled octaves so split-cell addressing remains translation invariant.
	float value = 0.0f;
	float weight = 0.5f;
	float weight_sum = 0.0f;
	for (int octave = 0; octave != 4; ++octave)
	{
		// Each octave doubles spatial frequency while the caller controls persistence.
		value += weight * ProceduralValueNoise(base_cell, coordinate, seed + uint(octave) * 0x9e3779b9u);
		weight_sum += weight;
		coordinate *= 2.0f;
		base_cell *= 2;
		weight *= saturate(detail);
	}
	return value / max(weight_sum, 0.00001f);
}

// Convert an interpolated world position into the split material coordinate supplied by the CPU.
float3 ProceduralCoordinate(float4 ws_vert)
{
	// Keep only cell-local fractions in translation constants while rows carry linear scaling.
	return float3(
		dot(ws_vert.xyz, g_pbr.procedural_coord_x.xyz) + g_pbr.procedural_coord_x.w,
		dot(ws_vert.xyz, g_pbr.procedural_coord_y.xyz) + g_pbr.procedural_coord_y.w,
		dot(ws_vert.xyz, g_pbr.procedural_coord_z.xyz) + g_pbr.procedural_coord_z.w);
}

// Evaluate albedo, height, and roughness from one shared parameterized field.
ProceduralSurfaceSample EvaluateProceduralSurface(float4 ws_vert)
{
	// Warp one shared broad field before deriving the correlated material channels.
	int3 base_cell = g_pbr.procedural_cell_seed.xyz;
	uint seed = asuint(g_pbr.procedural_cell_seed.w);
	float3 coordinate = ProceduralCoordinate(ws_vert);
	float warp_noise = ProceduralFbm(base_cell, coordinate + 19.17f, seed ^ 0x68bc21ebu, g_pbr.procedural_params1.x);
	coordinate += (warp_noise - 0.5f) * g_pbr.procedural_params1.y;
	float broad = ProceduralFbm(base_cell, coordinate, seed, g_pbr.procedural_params1.x);
	float fine = ProceduralValueNoise(base_cell * 8, coordinate * 8.0f, seed ^ 0xb5297a4du);
	float colour_value = saturate(0.82f * broad + 0.18f * fine);

	float palette_coordinate = colour_value * 3.0f;
	float3 colour =
		palette_coordinate < 1.0f ? lerp(g_pbr.procedural_colour0.rgb, g_pbr.procedural_colour1.rgb, palette_coordinate) :
		palette_coordinate < 2.0f ? lerp(g_pbr.procedural_colour1.rgb, g_pbr.procedural_colour2.rgb, palette_coordinate - 1.0f) :
		lerp(g_pbr.procedural_colour2.rgb, g_pbr.procedural_colour3.rgb, palette_coordinate - 2.0f);

	ProceduralSurfaceSample result = (ProceduralSurfaceSample)0;
	result.albedo = colour;
	result.height = broad;
	result.roughness = lerp(g_pbr.procedural_params0.z, g_pbr.procedural_params0.w, saturate(0.65f * broad + 0.35f * (1.0f - fine)));
	return result;
}

// Perturb a geometric normal from screen derivatives of the UV-free procedural height field.
float3 ProceduralWorldNormal(PSIn In, float3 normal, float height)
{
	// Convert screen-space height derivatives into a world-space surface gradient.
	float3 dpdx = ddx(In.ws_vert.xyz);
	float3 dpdy = ddy(In.ws_vert.xyz);
	float dhdx = ddx(height);
	float dhdy = ddy(height);
	float3 gradient = dhdx * cross(dpdy, normal) + dhdy * cross(normal, dpdx);
	float determinant = dot(dpdx, cross(dpdy, normal));
	if (abs(determinant) < 1.0e-8f)
		return normal;

	return normalize(normal - g_pbr.procedural_params0.y * gradient / determinant);
}

#endif
