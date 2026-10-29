//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2026
//*********************************************
// UV-free procedural surface material.
//
// The surface is band-limited to the pixel footprint so it does not alias at distance or at grazing angles: each noise layer fades to its mean
// value as its cells approach the pixel size, which acts like mipmapping for procedural noise. The bump gradient comes analytically from the same
// faded layers rather than from screen derivatives, so the perturbed normal has no 2x2 pixel-quad blockiness.
#ifndef PR_VIEW3D_PROCEDURAL_SURFACE_HLSLI
#define PR_VIEW3D_PROCEDURAL_SURFACE_HLSLI

// Surface channels returned by the shared procedural evaluator.
struct ProceduralSurfaceSample
{
	// Channels share one noise evaluation so their visible structure remains coherent.

	float3 albedo;
	float3 gradient; // World-space gradient of the band-limited height field, in height units per metre.
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
// Returns the noise value in 'x' and its gradient with respect to 'coordinate' in 'yzw'.
float4 ProceduralValueNoise(int3 base_cell, float3 coordinate, uint seed)
{
	// Split the sample into its integer cell and smooth within-cell interpolation coordinate.
	// The blend curve is 3t^2 - 2t^3, whose slope is 6t(1 - t).
	float3 cell_floor = floor(coordinate);
	int3 cell = base_cell + int3(cell_floor);
	float3 fraction = coordinate - cell_floor;
	float3 blend = fraction * fraction * (3.0f - 2.0f * fraction);
	float3 blend_slope = 6.0f * fraction * (1.0f - fraction);

	float n000 = ProceduralLattice(cell + int3(0, 0, 0), seed);
	float n100 = ProceduralLattice(cell + int3(1, 0, 0), seed);
	float n010 = ProceduralLattice(cell + int3(0, 1, 0), seed);
	float n110 = ProceduralLattice(cell + int3(1, 1, 0), seed);
	float n001 = ProceduralLattice(cell + int3(0, 0, 1), seed);
	float n101 = ProceduralLattice(cell + int3(1, 0, 1), seed);
	float n011 = ProceduralLattice(cell + int3(0, 1, 1), seed);
	float n111 = ProceduralLattice(cell + int3(1, 1, 1), seed);

	// Write the trilinear blend of the eight corners as a polynomial in the blend weights. It gives the same value as nested lerps, and each
	// partial derivative is the polynomial's derivative for that axis times that axis's blend slope.
	float k0 = n000;
	float k1 = n100 - n000;
	float k2 = n010 - n000;
	float k3 = n001 - n000;
	float k4 = n000 - n100 - n010 + n110;
	float k5 = n000 - n010 - n001 + n011;
	float k6 = n000 - n100 - n001 + n101;
	float k7 = -n000 + n100 + n010 - n110 + n001 - n101 - n011 + n111;
	float3 b = blend;
	float value = k0 + k1 * b.x + k2 * b.y + k3 * b.z + k4 * b.x * b.y + k5 * b.y * b.z + k6 * b.z * b.x + k7 * b.x * b.y * b.z;
	float3 gradient = blend_slope * float3(
		k1 + k4 * b.y + k6 * b.z + k7 * b.y * b.z,
		k2 + k5 * b.z + k4 * b.x + k7 * b.z * b.x,
		k3 + k6 * b.x + k5 * b.y + k7 * b.x * b.y);
	return float4(value, gradient);
}

// Return the strength, in [0,1], of a noise layer with 'frequency' cells per base cell, for a pixel 'footprint' base cells across.
float ProceduralOctaveFade(float footprint, float frequency)
{
	// The layer is at full strength while one of its cells covers at least two pixels, and has gone when a cell covers one pixel or less.
	// Between these the strength falls linearly with the number of the layer's cells per pixel.
	float cells_per_pixel = footprint * frequency;
	return saturate(2.0f - 2.0f * cells_per_pixel);
}

// Evaluate a compact four-octave field without texture lookup or finite spatial extent, band-limited to a pixel 'footprint' in base cells.
// Returns the value in 'x' and its gradient with respect to 'coordinate' in 'yzw'.
float4 ProceduralFbm(int3 base_cell, float3 coordinate, uint seed, float detail, float footprint)
{
	// Accumulate integer-scaled octaves so split-cell addressing remains translation invariant.
	// A fading octave is blended towards its mean (0.5), not towards zero, and keeps its share of 'weight_sum'. This keeps the average value,
	// and so the average colour and brightness, the same at every distance; only the detail around it is removed. Octaves that have faded
	// completely are not evaluated, so distant surfaces are also cheaper.
	float variation = 0.0f;
	float3 gradient = float3(0, 0, 0);
	float weight = 0.5f;
	float weight_sum = 0.0f;
	float frequency = 1.0f;
	for (int octave = 0; octave != 4; ++octave)
	{
		// Each octave doubles spatial frequency while the caller controls persistence.
		// Noise at 'coordinate * frequency' has a gradient with respect to 'coordinate' that is 'frequency' times the noise gradient.
		// The fade is treated as constant across the pixel, so the gradient ignores how the fade changes with distance.
		float fade = ProceduralOctaveFade(footprint, frequency);
		[branch] if (fade > 0.0f)
		{
			// Only the detail around the octave's mean is faded.
			float4 noise = ProceduralValueNoise(base_cell, coordinate, seed + uint(octave) * 0x9e3779b9u);
			variation += weight * fade * (noise.x - 0.5f);
			gradient += weight * fade * frequency * noise.yzw;
		}
		weight_sum += weight;
		coordinate *= 2.0f;
		base_cell *= 2;
		frequency *= 2.0f;
		weight *= saturate(detail);
	}
	return float4(0.5f + variation / weight_sum, gradient / weight_sum);
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

// Return the size of the pixel at 'ws_vert' in base noise cells, for EvaluateProceduralSurface.
// This uses screen derivatives, so call it from uniform control flow.
float ProceduralFootprint(float4 ws_vert)
{
	// Use the longer of the two screen-axis steps, so surfaces seen at grazing angles are filtered along their stretched direction. This blurs a
	// little more than needed across the short direction. The domain warp is ignored: it shifts the coordinate by a smooth, broad-scale offset,
	// which changes the local cell density only modestly.
	float3 coordinate = ProceduralCoordinate(ws_vert);
	return max(length(ddx(coordinate)), length(ddy(coordinate)));
}

// Evaluate albedo, height gradient, and roughness from one shared parameterized field.
// 'footprint' is the pixel size from ProceduralFootprint. Detail smaller than about two pixels fades to its mean value so the surface does not alias.
ProceduralSurfaceSample EvaluateProceduralSurface(float4 ws_vert, float footprint)
{
	// Warp one shared broad field before deriving the correlated material channels.
	int3 base_cell = g_pbr.procedural_cell_seed.xyz;
	uint seed = asuint(g_pbr.procedural_cell_seed.w);
	float detail = g_pbr.procedural_params1.x;
	float warp = g_pbr.procedural_params1.y;
	float3 coordinate = ProceduralCoordinate(ws_vert);
	float4 warp_noise = ProceduralFbm(base_cell, coordinate + 19.17f, seed ^ 0x68bc21ebu, detail, footprint);
	coordinate += (warp_noise.x - 0.5f) * warp;
	float4 broad = ProceduralFbm(base_cell, coordinate, seed, detail, footprint);

	// The fine layer has eight cells per base cell and fades to its mean like an fBm octave.
	float fine_fade = ProceduralOctaveFade(footprint, 8.0f);
	float fine = 0.5f;
	[branch] if (fine_fade > 0.0f)
		fine = lerp(0.5f, ProceduralValueNoise(base_cell * 8, coordinate * 8.0f, seed ^ 0xb5297a4du).x, fine_fade);

	float colour_value = saturate(0.82f * broad.x + 0.18f * fine);
	float palette_coordinate = colour_value * 3.0f;
	float3 colour =
		palette_coordinate < 1.0f ? lerp(g_pbr.procedural_colour0.rgb, g_pbr.procedural_colour1.rgb, palette_coordinate) :
		palette_coordinate < 2.0f ? lerp(g_pbr.procedural_colour1.rgb, g_pbr.procedural_colour2.rgb, palette_coordinate - 1.0f) :
		lerp(g_pbr.procedural_colour2.rgb, g_pbr.procedural_colour3.rgb, palette_coordinate - 2.0f);

	// The height is the broad field. The warp adds the same offset to all three axes, so by the chain rule the gradient with respect to the
	// unwarped coordinate is the broad gradient plus 'warp' times the warp-field gradient times the sum of the broad gradient's components.
	// The coordinate rows map world positions to coordinates, so the world gradient is the coordinate gradient multiplied by those rows.
	float3 coordinate_gradient = broad.yzw + warp * warp_noise.yzw * dot(broad.yzw, float3(1, 1, 1));
	float3 world_gradient =
		coordinate_gradient.x * g_pbr.procedural_coord_x.xyz +
		coordinate_gradient.y * g_pbr.procedural_coord_y.xyz +
		coordinate_gradient.z * g_pbr.procedural_coord_z.xyz;

	ProceduralSurfaceSample result = (ProceduralSurfaceSample)0;
	result.albedo = colour;
	result.gradient = world_gradient;
	result.roughness = lerp(g_pbr.procedural_params0.z, g_pbr.procedural_params0.w, saturate(0.65f * broad.x + 0.35f * (1.0f - fine)));
	return result;
}

// Perturb a unit world-space normal by the world-space height gradient from EvaluateProceduralSurface.
float3 ProceduralWorldNormal(float3 normal, float3 gradient)
{
	// Only the part of the gradient along the surface tilts the normal, so remove its component along the normal.
	float3 surface_gradient = gradient - dot(gradient, normal) * normal;
	return normalize(normal - g_pbr.procedural_params0.y * surface_gradient);
}

#endif