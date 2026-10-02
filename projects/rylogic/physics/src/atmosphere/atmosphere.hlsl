//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// Terrain-following atmosphere compute kernels for a staggered MAC grid.
//
// Grid layout:
//   The horizontal domain is a regular XY grid. U velocity lives on x-normal faces, V velocity lives on y-normal faces,
//   and W velocity lives on z-normal faces. Temperature and perturbation pressure live at cell centres.
//
// Terrain-following coordinates:
//   Each horizontal column has a caller-supplied floor height and a shared flat lid height. A sigma value in [0, 1]
//   maps to world Z by lerping from the local floor to the lid. The optional layer stretch raises sigma to a power so
//   the first layers can be thinner near the floor while the lid remains exact.
//
// One solver step records these passes in order:
//   1. CSAdvect traces velocity and temperature backward through the previous MAC field.
//   2. CSVorticity stores the swirl magnitude of the advected field (only when vorticity confinement is enabled).
//   3. CSForcesHeat applies buoyancy, vorticity confinement, floor exchange, lid relaxation, heat sources, and the open-edge sponge.
//   4. CSDivergence builds the metric-corrected divergence of the intermediate MAC velocity.
//   5. CSMgSmooth, CSMgResidual, CSMgRestrict, and CSMgProlongate run the pressure V-cycle.
//   6. CSNormalisePressure removes the closed-domain pressure null space.
//   7. CSProject subtracts the metric-corrected perturbation-pressure gradient from face velocities.
//
// Units are metres (m), seconds (s), Kelvin (K), and metres per second (m/s). The pressure field is a perturbation
// potential whose gradient has velocity units for projection. Outside faces are either solid, with zero normal flow,
// or open. On an open side, each boundary column has caller-supplied outside air. Where the outside wind blows into the
// domain, the face takes the outside wind and the boundary cell takes the outside temperature. Elsewhere the face copies
// the neighbouring interior face so air can leave freely.
//
// Terrain-following pressure gradients use the sigma metric. Horizontal gradients subtract dz/dx or dz/dy times a
// vertical pressure gradient average so that pressure differences along sloped layers represent a world-horizontal
// gradient rather than an upslope gradient. The multigrid solve uses the same metric operator as projection.
//
// The multigrid V-cycle semi-coarsens only the horizontal axes. All vertical layers are retained on every level, and
// each smoother pass solves one full vertical column with a tridiagonal relaxation. This keeps the stiff near-floor
// vertical coupling on the fine vertical grid while coarse levels remove long horizontal wavelengths.

#include "pr/hlsl/core.hlsli"
#include "pr/hlsl/interop.hlsli"
#include "pr/hlsl/vector.hlsli"

#define ATMOSPHERE_THREAD_X 8
#define ATMOSPHERE_THREAD_Y 8
#define ATMOSPHERE_THREAD_Z 4

static const float AtmosphereMinLayerPower = 0.05f;       // dimensionless lower bound that keeps stretched-layer powers finite
static const float AtmosphereMinCellHeight = 0.001f;      // metres; prevents zero-thickness metric denominators
static const float AtmosphereSmallWeight = 1.0e-6f;       // dimensionless; protects interpolation denominators at clipped boundaries
static const float AtmosphereSmallDiagonal = 1.0e-20f;    // operator units; avoids division by zero in degenerate local stencils
static const float AtmosphereMinHeatRadius = 0.0001f;     // metres; avoids division by zero for point-like heat sources
static const float AtmosphereMinSwirlGradient = 1.0e-8f;  // 1/(s m); below this the swirl has no clear centre and confinement adds no force
static const float AtmosphereOpenEdgeWindRate = 8.0f;     // 1/s; blends outside inflow wind into the open-edge sponge over a short solver step
#define ATMOSPHERE_TRACER_THREAD_X 64

// Root constants shared by every atmosphere kernel. Must match CBufAtmosphere in atmosphere.cpp.
// HLSL constant packing does not let a vector cross a 16-byte boundary, so the byte offset of each group is noted on the right.
struct CBufAtmosphere
{
	// Grid shape. The fine grid has 'cell_count.x * cell_count.y' columns, and each column has 'cell_count.z' layers.
	// Cell-centred arrays have this size. Face arrays have one extra entry along their normal axis.
	int3 cell_count;                // fine-grid cell counts in X, Y (columns) and Z (layers)                         @0
	int boundary_mask;              // one bit per side, set when that side is open: bit 0 = x-, 1 = x+, 2 = y-, 3 = y+   @12

	// Domain placement. Columns are square with side 'dx'. The vertical extent of each column runs from its floor
	// height (g_floor_height) up to the shared flat lid.
	float2 origin;                  // world-space XY of the low corner of cell (0,0), metres                          @16
	float lid_z;                    // world-space Z of the flat domain top, metres                                     @24
	float dx;                       // horizontal cell size in both X and Y, metres                                     @28

	// Step and layer shape.
	float dt;                       // duration of this solver step, seconds                                            @32
	float gravity;                  // positive gravitational acceleration used for buoyancy, m/s^2                    @36
	float first_layer_thickness;    // smallest allowed column height per layer; guards metric denominators, metres     @40
	float layer_power;              // sigma stretch exponent; values above 1 make the layers near the floor thinner    @44

	// Reference temperature profile. Buoyancy is driven by the difference between the cell temperature and this
	// profile at the same height: T_ref(z) = max(temp0 + lapse * z, min_temp).
	float temp0;                    // reference temperature at world Z = 0, K                                          @48
	float lapse;                    // change in reference temperature per metre of height (normally negative), K/m     @52
	float min_temp;                 // lower clamp for the reference temperature, K                                     @56

	// Floor and lid heat exchange (used by CSForcesHeat).
	float floor_exchange_rate;      // rate at which the lowest layer relaxes toward the floor temperature, 1/s         @60
	float lid_temperature;          // temperature that the top layer relaxes toward, K                                 @64
	float lid_relaxation_rate;      // rate of the top-layer relaxation; zero disables it, 1/s                          @68

	// Caller-supplied buffers.
	int source_count;               // number of valid entries in g_sources                                             @72
	int use_floor_temp_buffer;      // non-zero: g_floor_temperature has one value per column; zero: one uniform value  @76

	// Multigrid level selection (used by the CSMg* kernels and CSNormalisePressure). All levels are packed into the same
	// pressure buffers. A level keeps every vertical layer and has fewer columns, so 'mg_offset' is the index of the
	// level's first cell in the packed buffers. The child is the next coarser level.
	int2 mg_size;                   // column counts of the active level in X and Y                                     @80
	int2 mg_child_size;             // column counts of the child (coarser) level in X and Y; zero on the coarsest level @88
	int mg_offset;                  // index of the active level's first cell in the packed pressure buffers            @96
	int mg_child_offset;            // index of the child level's first cell in the packed pressure buffers             @100
	int mg_scale;                   // number of fine-grid columns per active-level column along X and Y                @104
	int mg_phase;                   // red-black colour (0/1) for CSMgSmooth, or the reduction pass for CSNormalisePressure @108

	// Open edges and swirl restoration (used by CSForcesHeat).
	int open_edge_band;             // columns inside each open side over which the inflow outside wind is blended in    @112
	float vorticity_confinement;    // swirl-restoring strength; the acceleration is this times dx times the swirl rate, 1/s @116
};

// Root constants shared by tracer kernels. Must match CBufAtmosphereTracers in atmosphere.cpp.
struct CBufAtmosphereTracers
{
	int tracer_count;               // number of tracer particles in g_tracers_in/g_tracers_out
	uint tracer_seed;               // deterministic seed mixed with particle index and frame counter
	uint tracer_frame;              // monotonically increasing tracer dispatch index
	float tracer_max_age;           // particle lifetime before deterministic respawn, seconds
};

// Spherical heat or relaxation source supplied by the caller. Must match GpuHeatSource in atmosphere.cpp.
struct HeatSource
{
	float4 centre;                  // world-space centre, metres
	float radius;                   // influence radius, metres
	float heating_rate;             // additive temperature rate at the centre, K/s
	float target_temperature;       // optional relaxation target, K
	float relaxation_rate;          // first-order relaxation rate at the centre, 1/s
};

// Outside air beside one boundary column. Must match GpuOutsideAir in atmosphere.cpp.
// The buffer always has one entry per boundary column, packed as: x- side by y, x+ side by y, y- side by x, y+ side by x.
// Entries for solid sides are unused.
struct OutsideAir
{
	float2 wind;                    // XY wind of the outside air, m/s
	float temperature_offset;       // outside air temperature relative to the reference profile, K
	float pad;                      // keeps the structure size a multiple of 16 bytes
};

RWStructuredBuffer<float> resource(g_u_out, u0);                  // output U face velocity, m/s
RWStructuredBuffer<float> resource(g_v_out, u1);                  // output V face velocity, m/s
RWStructuredBuffer<float> resource(g_w_out, u2);                  // output W face velocity, m/s
RWStructuredBuffer<float> resource(g_temperature_out, u3);        // output cell-centred temperature, K
RWStructuredBuffer<float> resource(g_pressure, u4);               // cell-centred perturbation pressure potential
RWStructuredBuffer<float> resource(g_divergence, u5);             // cell-centred divergence or multigrid right-hand side, 1/s
RWStructuredBuffer<float> resource(g_residual, u6);               // cell-centred residual or pressure normalisation scratch

StructuredBuffer<float> resource(g_u_in, t0);                     // input U face velocity, m/s
StructuredBuffer<float> resource(g_v_in, t1);                     // input V face velocity, m/s
StructuredBuffer<float> resource(g_w_in, t2);                     // input W face velocity, m/s
StructuredBuffer<float> resource(g_temperature_in, t3);           // input cell-centred temperature, K
StructuredBuffer<float> resource(g_floor_temperature, t4);        // one floor temperature per column, or one uniform value, K
StructuredBuffer<HeatSource> resource(g_sources, t5);             // active heat sources
StructuredBuffer<float> resource(g_floor_height, t6);             // one floor height per column, metres
StructuredBuffer<OutsideAir> resource(g_outside_air, t7);         // outside air, one entry per boundary column

// One advected tracer particle. Must match GpuTracerParticle in atmosphere.cpp.
struct TracerParticle
{
	float4 position;                // world-space position, metres
	float temperature;              // sampled cell-centred air temperature, K
	float age;                      // particle age, seconds
	float2 pad;                     // keeps the structure size a multiple of 16 bytes
};

RWStructuredBuffer<TracerParticle> resource(g_tracers_out, u7);     // output tracer particles
StructuredBuffer<TracerParticle> resource(g_tracers_in, t8);        // input tracer particles

ConstantBuffer<CBufAtmosphere> resource(g, b0);                   // per-dispatch constants
ConstantBuffer<CBufAtmosphereTracers> resource(gt, b1);             // per-dispatch tracer constants

static const int BoundarySolid = 0;
static const int BoundaryOpen = 1;

// Return true when the packed boundary bit marks an open side face.
bool BoundaryOpenBit(int bit)
{
	return (g.boundary_mask & (1u << (uint)bit)) != 0;
}

// Return the low-X boundary mode encoded for the shader.
int BoundaryXMin()
{
	return BoundaryOpenBit(0) ? BoundaryOpen : BoundarySolid;
}

// Return the high-X boundary mode encoded for the shader.
int BoundaryXMax()
{
	return BoundaryOpenBit(1) ? BoundaryOpen : BoundarySolid;
}

// Return the low-Y boundary mode encoded for the shader.
int BoundaryYMin()
{
	return BoundaryOpenBit(2) ? BoundaryOpen : BoundarySolid;
}

// Return the high-Y boundary mode encoded for the shader.
int BoundaryYMax()
{
	return BoundaryOpenBit(3) ? BoundaryOpen : BoundarySolid;
}


// Return the packed cell-centre index for a fine-grid cell.
int CellIndex(int3 c)
{
	return (c.z * g.cell_count.y + c.y) * g.cell_count.x + c.x;
}

// Return the packed column index for a fine-grid XY cell.
int ColumnIndex(int2 c)
{
	return c.y * g.cell_count.x + c.x;
}

// Return the packed U-face index for an x-normal face.
int UIndex(int3 f)
{
	return (f.z * g.cell_count.y + f.y) * (g.cell_count.x + 1) + f.x;
}

// Return the packed V-face index for a y-normal face.
int VIndex(int3 f)
{
	return (f.z * (g.cell_count.y + 1) + f.y) * g.cell_count.x + f.x;
}

// Return the packed W-face index for a z-normal face.
int WIndex(int3 f)
{
	return (f.z * g.cell_count.y + f.y) * g.cell_count.x + f.x;
}

// Return true when a fine-grid cell coordinate is inside the domain.
bool InCells(int3 c)
{
	return all(c >= int3(0, 0, 0)) && c.x < g.cell_count.x && c.y < g.cell_count.y && c.z < g.cell_count.z;
}

// Return true when a U-face coordinate is inside the stored face array.
bool InU(int3 f)
{
	return f.x >= 0 && f.x <= g.cell_count.x && f.y >= 0 && f.y < g.cell_count.y && f.z >= 0 && f.z < g.cell_count.z;
}

// Return true when a V-face coordinate is inside the stored face array.
bool InV(int3 f)
{
	return f.x >= 0 && f.x < g.cell_count.x && f.y >= 0 && f.y <= g.cell_count.y && f.z >= 0 && f.z < g.cell_count.z;
}

// Return true when a W-face coordinate is inside the stored face array.
bool InW(int3 f)
{
	return f.x >= 0 && f.x < g.cell_count.x && f.y >= 0 && f.y < g.cell_count.y && f.z >= 0 && f.z <= g.cell_count.z;
}

// Clamp an XY column coordinate to the nearest valid fine-grid column.
int2 ClampColumn(int2 c)
{
	return clamp(c, int2(0, 0), int2(g.cell_count.x - 1, g.cell_count.y - 1));
}

// Return the caller-supplied floor height for the nearest valid column.
float FloorHeight(int2 c)
{
	return g_floor_height[ColumnIndex(ClampColumn(c))];
}

// Return the stretched sigma value for a vertical face index.
float SigmaFace(int z)
{
	float raw = saturate((float)z / max((float)g.cell_count.z, 1.0f));
	float power = max(g.layer_power, AtmosphereMinLayerPower);
	return pow(raw, power);
}

// Return the stretched sigma value at a vertical cell centre.
float SigmaCentre(int z)
{
	return 0.5f * (SigmaFace(z) + SigmaFace(z + 1));
}

// Return the guarded floor-to-lid height of one column.
float ColumnHeight(int2 c)
{
	return max(g.lid_z - FloorHeight(c), g.first_layer_thickness * max((float)g.cell_count.z, 1.0f));
}

// Return the world-space Z of a vertical face in one fine-grid column.
float FaceZ(int2 c, int z)
{
	return FloorHeight(c) + SigmaFace(z) * ColumnHeight(c);
}

// Return the world-space Z of a cell centre in one fine-grid column.
float CellZ(int2 c, int z)
{
	return FloorHeight(c) + SigmaCentre(z) * ColumnHeight(c);
}

// Return the guarded physical height of one cell.
float CellDz(int2 c, int z)
{
	return max(FaceZ(c, z + 1) - FaceZ(c, z), AtmosphereMinCellHeight);
}

// Return the world-space centre of one fine-grid cell.
float3 CellCentre(int3 c)
{
	return float3(g.origin.x + (c.x + 0.5f) * g.dx, g.origin.y + (c.y + 0.5f) * g.dx, CellZ(c.xy, c.z));
}

// Return the world-space centre of one U face.
float3 UFaceCentre(int3 f)
{
	int2 c0 = ClampColumn(int2(f.x - 1, f.y));
	int2 c1 = ClampColumn(int2(f.x, f.y));
	float z = 0.5f * (CellZ(c0, f.z) + CellZ(c1, f.z));
	return float3(g.origin.x + f.x * g.dx, g.origin.y + (f.y + 0.5f) * g.dx, z);
}

// Return the world-space centre of one V face.
float3 VFaceCentre(int3 f)
{
	int2 c0 = ClampColumn(int2(f.x, f.y - 1));
	int2 c1 = ClampColumn(int2(f.x, f.y));
	float z = 0.5f * (CellZ(c0, f.z) + CellZ(c1, f.z));
	return float3(g.origin.x + (f.x + 0.5f) * g.dx, g.origin.y + f.y * g.dx, z);
}

// Return the world-space centre of one W face.
float3 WFaceCentre(int3 f)
{
	return float3(g.origin.x + (f.x + 0.5f) * g.dx, g.origin.y + (f.y + 0.5f) * g.dx, FaceZ(f.xy, f.z));
}

// Return the bounded reference temperature at a world-space height.
float ReferenceTemperature(float z)
{
	return max(g.min_temp, g.temp0 + g.lapse * z);
}

// Return true when a U face can carry normal flow.
bool UFaceActive(int3 f)
{
	if (!InU(f)) return false;
	if (f.x == 0) return BoundaryXMin() == BoundaryOpen;
	if (f.x == g.cell_count.x) return BoundaryXMax() == BoundaryOpen;
	return true;
}

// Return true when a V face can carry normal flow.
bool VFaceActive(int3 f)
{
	if (!InV(f)) return false;
	if (f.y == 0) return BoundaryYMin() == BoundaryOpen;
	if (f.y == g.cell_count.y) return BoundaryYMax() == BoundaryOpen;
	return true;
}

// Return true when a W face can carry normal flow.
bool WFaceActive(int3 f)
{
	if (!InW(f)) return false;
	if (f.z == 0) return false;
	if (f.z == g.cell_count.z) return false;
	return true;
}

// Return a U-face value or zero outside active flow faces.
float LoadU(int3 f)
{
	return UFaceActive(f) ? g_u_in[UIndex(f)] : 0.0f;
}

// Return a V-face value or zero outside active flow faces.
float LoadV(int3 f)
{
	return VFaceActive(f) ? g_v_in[VIndex(f)] : 0.0f;
}

// Return a W-face value or zero outside active flow faces.
float LoadW(int3 f)
{
	return WFaceActive(f) ? g_w_in[WIndex(f)] : 0.0f;
}

// Return a clamped cell-centred temperature sample.
float LoadTemperature(int3 c)
{
	return g_temperature_in[CellIndex(clamp(c, int3(0, 0, 0), int3(g.cell_count.x - 1, g.cell_count.y - 1, g.cell_count.z - 1)))];
}

// Return the fractional cell-centred vertical coordinate that contains a world-space height.
float CellCoordZ(int2 col, float z)
{
	float sigma = saturate((z - FloorHeight(col)) / ColumnHeight(col));
	float first = SigmaCentre(0);
	if (sigma <= first) return 0.0f;
	for (int iz = 0; iz != g.cell_count.z - 1; ++iz)
	{
		float lo = SigmaCentre(iz);
		float hi = SigmaCentre(iz + 1);
		if (sigma <= hi)
		{
			return (float)iz + saturate((sigma - lo) / max(hi - lo, AtmosphereSmallWeight));
		}
	}
	return (float)(g.cell_count.z - 1);
}


// Map a world-space position to fractional cell-centred grid coordinates.
float3 GridSample(float3 pos)
{
	float2 xy = float2((pos.x - g.origin.x) / g.dx - 0.5f, (pos.y - g.origin.y) / g.dx - 0.5f);
	int2 col = ClampColumn((int2)round(xy));
	return float3(xy, CellCoordZ(col, pos.z));
}

// Return a trilinear temperature interpolation from eight clamped cell samples.
float TrilinearTemperature(int3 c0, int3 c1, float3 f)
{
	float c00 = lerp(LoadTemperature(int3(c0.x, c0.y, c0.z)), LoadTemperature(int3(c1.x, c0.y, c0.z)), f.x);
	float c10 = lerp(LoadTemperature(int3(c0.x, c1.y, c0.z)), LoadTemperature(int3(c1.x, c1.y, c0.z)), f.x);
	float c01 = lerp(LoadTemperature(int3(c0.x, c0.y, c1.z)), LoadTemperature(int3(c1.x, c0.y, c1.z)), f.x);
	float c11 = lerp(LoadTemperature(int3(c0.x, c1.y, c1.z)), LoadTemperature(int3(c1.x, c1.y, c1.z)), f.x);
	return lerp(lerp(c00, c10, f.y), lerp(c01, c11, f.y), f.z);
}

// Sample temperature at a world-space position with clamped trilinear lookup.
float SampleTemperature(float3 pos)
{
	float3 grid = GridSample(pos);
	float3 base_f = floor(grid);
	float3 f = saturate(grid - base_f);
	int3 c0 = clamp((int3)base_f, int3(0, 0, 0), int3(g.cell_count.x - 1, g.cell_count.y - 1, g.cell_count.z - 1));
	int3 c1 = clamp(c0 + int3(1, 1, 1), int3(0, 0, 0), int3(g.cell_count.x - 1, g.cell_count.y - 1, g.cell_count.z - 1));
	return TrilinearTemperature(c0, c1, f);
}

// Return one staggered velocity component, selected by 'axis' (0 = U, 1 = V, 2 = W), or zero outside active flow faces.
float LoadFace(int axis, int3 f)
{
	switch (axis)
	{
		case 0: return LoadU(f);
		case 1: return LoadV(f);
		default: return LoadW(f);
	}
}

// Interpolate one staggered velocity component at fractional face coordinates 'grid'. 'max_index' is the largest face index on each axis.
// Coordinates outside the face range clamp to the nearest face, and inactive faces contribute zero.
float TrilinearFace(int axis, float3 grid, int3 max_index)
{
	// Blend the eight surrounding faces so motion of less than half a cell still moves the sampled value.
	float3 base_f = floor(grid);
	float3 f = saturate(grid - base_f);
	int3 c0 = clamp((int3)base_f, int3(0, 0, 0), max_index);
	int3 c1 = clamp((int3)base_f + int3(1, 1, 1), int3(0, 0, 0), max_index);
	float c00 = lerp(LoadFace(axis, int3(c0.x, c0.y, c0.z)), LoadFace(axis, int3(c1.x, c0.y, c0.z)), f.x);
	float c10 = lerp(LoadFace(axis, int3(c0.x, c1.y, c0.z)), LoadFace(axis, int3(c1.x, c1.y, c0.z)), f.x);
	float c01 = lerp(LoadFace(axis, int3(c0.x, c0.y, c1.z)), LoadFace(axis, int3(c1.x, c0.y, c1.z)), f.x);
	float c11 = lerp(LoadFace(axis, int3(c0.x, c1.y, c1.z)), LoadFace(axis, int3(c1.x, c1.y, c1.z)), f.x);
	return lerp(lerp(c00, c10, f.y), lerp(c01, c11, f.y), f.z);
}

// Sample U velocity at a world-space position by interpolating the surrounding U faces.
float SampleU(float3 pos)
{
	// U faces sit half a cell below the cell centres along X.
	float3 grid = GridSample(pos) + float3(0.5f, 0.0f, 0.0f);
	return TrilinearFace(0, grid, int3(g.cell_count.x, g.cell_count.y - 1, g.cell_count.z - 1));
}

// Sample V velocity at a world-space position by interpolating the surrounding V faces.
float SampleV(float3 pos)
{
	// V faces sit half a cell below the cell centres along Y.
	float3 grid = GridSample(pos) + float3(0.0f, 0.5f, 0.0f);
	return TrilinearFace(1, grid, int3(g.cell_count.x - 1, g.cell_count.y, g.cell_count.z - 1));
}

// Sample W velocity at a world-space position by interpolating the surrounding W faces.
float SampleW(float3 pos)
{
	// W faces sit approximately half a layer below the cell centres along Z.
	float3 grid = GridSample(pos) + float3(0.0f, 0.0f, 0.5f);
	return TrilinearFace(2, grid, int3(g.cell_count.x - 1, g.cell_count.y - 1, g.cell_count.z));
}

// Return a staggered velocity sample at a world-space position.
float3 SampleVelocity(float3 pos)
{
	return float3(SampleU(pos), SampleV(pos), SampleW(pos));
}

// Clamp a traced world-space position to the valid terrain-following domain.
float3 ClampToDomain(float3 pos)
{
	float x = clamp(pos.x, g.origin.x, g.origin.x + g.cell_count.x * g.dx);
	float y = clamp(pos.y, g.origin.y, g.origin.y + g.cell_count.y * g.dx);
	int2 col = ClampColumn((int2)floor(float2((x - g.origin.x) / g.dx, (y - g.origin.y) / g.dx)));
	float z = clamp(pos.z, FloorHeight(col), g.lid_z);
	return float3(x, y, z);
}


// Return the outside air beside one boundary column. See OutsideAir for the packed order.
OutsideAir OutsideAirXMin(int y)
{
	return g_outside_air[y];
}
OutsideAir OutsideAirXMax(int y)
{
	return g_outside_air[g.cell_count.y + y];
}
OutsideAir OutsideAirYMin(int x)
{
	return g_outside_air[2 * g.cell_count.y + x];
}
OutsideAir OutsideAirYMax(int x)
{
	return g_outside_air[2 * g.cell_count.y + g.cell_count.x + x];
}

// True when the outside air beside the given boundary column blows into the domain through an open side.
bool InflowXMin(int y)
{
	return BoundaryXMin() == BoundaryOpen && OutsideAirXMin(y).wind.x > 0.0f;
}
bool InflowXMax(int y)
{
	return BoundaryXMax() == BoundaryOpen && OutsideAirXMax(y).wind.x < 0.0f;
}
bool InflowYMin(int x)
{
	return BoundaryYMin() == BoundaryOpen && OutsideAirYMin(x).wind.y > 0.0f;
}
bool InflowYMax(int x)
{
	return BoundaryYMax() == BoundaryOpen && OutsideAirYMax(x).wind.y < 0.0f;
}

// True when U face 'p' lies on an open x side.
bool OpenBoundaryUFace(int3 p)
{
	return (p.x == 0 && BoundaryXMin() == BoundaryOpen) || (p.x == g.cell_count.x && BoundaryXMax() == BoundaryOpen);
}

// True when V face 'p' lies on an open y side.
bool OpenBoundaryVFace(int3 p)
{
	return (p.y == 0 && BoundaryYMin() == BoundaryOpen) || (p.y == g.cell_count.y && BoundaryYMax() == BoundaryOpen);
}

// Return the velocity of an open-boundary U face. Inflow faces take the outside wind. Outflow faces copy the
// neighbouring interior face from the input field, so air leaves without being pushed back. Requires OpenBoundaryUFace(p).
float OpenBoundaryU(int3 p)
{
	// Choose the side from the face index; the caller guarantees that this side is open.
	if (p.x == 0)
		return InflowXMin(p.y) ? OutsideAirXMin(p.y).wind.x : g_u_in[UIndex(int3(1, p.y, p.z))];

	return InflowXMax(p.y) ? OutsideAirXMax(p.y).wind.x : g_u_in[UIndex(int3(g.cell_count.x - 1, p.y, p.z))];
}

// Return the velocity of an open-boundary V face. See OpenBoundaryU. Requires OpenBoundaryVFace(p).
float OpenBoundaryV(int3 p)
{
	// Choose the side from the face index; the caller guarantees that this side is open.
	if (p.y == 0)
		return InflowYMin(p.x) ? OutsideAirYMin(p.x).wind.y : g_v_in[VIndex(int3(p.x, 1, p.z))];

	return InflowYMax(p.x) ? OutsideAirYMax(p.x).wind.y : g_v_in[VIndex(int3(p.x, g.cell_count.y - 1, p.z))];
}

// Find the outside air that flows into boundary cell 'p'. Returns false when no open side of the cell has inflow.
// A corner cell with inflow on two sides uses the x side.
bool InflowOutsideAir(int3 p, out OutsideAir air)
{
	// Test each side the cell touches, in packed-buffer order.
	air = (OutsideAir)0;
	if (p.x == 0 && InflowXMin(p.y))
		air = OutsideAirXMin(p.y);
	else if (p.x == g.cell_count.x - 1 && InflowXMax(p.y))
		air = OutsideAirXMax(p.y);
	else if (p.y == 0 && InflowYMin(p.x))
		air = OutsideAirYMin(p.x);
	else if (p.y == g.cell_count.y - 1 && InflowYMax(p.x))
		air = OutsideAirYMax(p.x);
	else
		return false;

	return true;
}

// Return the blend weight for nudging an open-edge column toward the inflow outside air, and that air's wind.
// The weight is 1 at an inflow side and falls to 0 across the sponge band. Where bands from two inflow sides overlap,
// the side with the larger weight provides the wind.
float OpenEdgeWindBlend(int2 col, out float2 wind)
{
	// A zero-width band leaves the outside wind on the boundary faces only.
	int band = g.open_edge_band;
	float edge = 0.0f;
	wind = float2(0.0f, 0.0f);
	if (band <= 0)
		return 0.0f;

	// Each side only affects the columns in its own row or column, so neighbouring rows can have opposing winds.
	if (InflowXMin(col.y))
	{
		float weight = saturate((float)(band - col.x) / (float)band);
		if (weight > edge)
		{
			edge = weight;
			wind = OutsideAirXMin(col.y).wind;
		}
	}
	if (InflowXMax(col.y))
	{
		float weight = saturate((float)(band - (g.cell_count.x - 1 - col.x)) / (float)band);
		if (weight > edge)
		{
			edge = weight;
			wind = OutsideAirXMax(col.y).wind;
		}
	}
	if (InflowYMin(col.x))
	{
		float weight = saturate((float)(band - col.y) / (float)band);
		if (weight > edge)
		{
			edge = weight;
			wind = OutsideAirYMin(col.x).wind;
		}
	}
	if (InflowYMax(col.x))
	{
		float weight = saturate((float)(band - (g.cell_count.y - 1 - col.y)) / (float)band);
		if (weight > edge)
		{
			edge = weight;
			wind = OutsideAirYMax(col.x).wind;
		}
	}
	return edge;
}

// Return the velocity at the centre of cell 'c', averaged from its two faces on each axis. 'c' is clamped into the grid.
float3 CellVelocity(int3 c)
{
	// Clamping lets neighbour lookups at the domain edges reuse the edge cell.
	c = clamp(c, int3(0, 0, 0), g.cell_count - 1);
	return 0.5f * float3(LoadU(c) + LoadU(c + int3(1, 0, 0)), LoadV(c) + LoadV(c + int3(0, 1, 0)), LoadW(c) + LoadW(c + int3(0, 0, 1)));
}

// Return the swirl (curl of the velocity) at the centre of cell 'c', 1/s.
// Derivatives are taken along the grid layers and ignore the terrain slope. This is accurate enough for the confinement force, which only restores small swirls.
float3 CellVorticity(int3 c)
{
	// Use central differences, or one-sided differences at the domain edges. An axis with one cell has no derivative.
	int3 lo = max(c - 1, int3(0, 0, 0));
	int3 hi = min(c + 1, g.cell_count - 1);
	float span_x = (float)max(hi.x - lo.x, 1) * g.dx;
	float span_y = (float)max(hi.y - lo.y, 1) * g.dx;
	float span_z = max(CellZ(c.xy, hi.z) - CellZ(c.xy, lo.z), AtmosphereMinCellHeight);
	float3 d_dx = (CellVelocity(int3(hi.x, c.y, c.z)) - CellVelocity(int3(lo.x, c.y, c.z))) / span_x;
	float3 d_dy = (CellVelocity(int3(c.x, hi.y, c.z)) - CellVelocity(int3(c.x, lo.y, c.z))) / span_y;
	float3 d_dz = (CellVelocity(int3(c.x, c.y, hi.z)) - CellVelocity(int3(c.x, c.y, lo.z))) / span_z;
	return float3(d_dy.z - d_dz.y, d_dz.x - d_dx.z, d_dx.y - d_dy.x);
}

// Return the swirl magnitude stored by CSVorticity for cell 'c', 1/s. 'c' is clamped into the grid.
float SwirlMagnitude(int3 c)
{
	return g_divergence[CellIndex(clamp(c, int3(0, 0, 0), g.cell_count - 1))];
}

// Return the vorticity confinement acceleration at the centre of cell 'c', m/s^2. Requires the swirl magnitudes from CSVorticity.
// The force pushes the flow around each local peak of swirl, which restores rotation that numerical smoothing removes.
float3 ConfinementAcceleration(int3 c)
{
	// Find the direction toward stronger swirl from the stored swirl magnitudes.
	c = clamp(c, int3(0, 0, 0), g.cell_count - 1);
	int3 lo = max(c - 1, int3(0, 0, 0));
	int3 hi = min(c + 1, g.cell_count - 1);
	float span_x = (float)max(hi.x - lo.x, 1) * g.dx;
	float span_y = (float)max(hi.y - lo.y, 1) * g.dx;
	float span_z = max(CellZ(c.xy, hi.z) - CellZ(c.xy, lo.z), AtmosphereMinCellHeight);
	float3 gradient = float3(
		(SwirlMagnitude(int3(hi.x, c.y, c.z)) - SwirlMagnitude(int3(lo.x, c.y, c.z))) / span_x,
		(SwirlMagnitude(int3(c.x, hi.y, c.z)) - SwirlMagnitude(int3(c.x, lo.y, c.z))) / span_y,
		(SwirlMagnitude(int3(c.x, c.y, hi.z)) - SwirlMagnitude(int3(c.x, c.y, lo.z))) / span_z);
	float len = length(gradient);
	if (len < AtmosphereMinSwirlGradient)
		return float3(0.0f, 0.0f, 0.0f);

	// Push at right angles to both that direction and the swirl axis. The cell size keeps the force consistent as the grid is refined.
	return g.vorticity_confinement * g.dx * cross(gradient / len, CellVorticity(c));
}


// Return pressure from one packed multigrid level, or zero outside it.
float PressureAtLevel(int3 c, int nx, int ny, int offset)
{
	if (c.x < 0 || c.x >= nx || c.y < 0 || c.y >= ny || c.z < 0 || c.z >= g.cell_count.z)
		return 0.0f;
	return g_pressure[offset + (c.z * ny + c.y) * nx + c.x];
}

// Return fine-level pressure for a cell coordinate.
float PressureAt(int3 c)
{
	return PressureAtLevel(c, g.cell_count.x, g.cell_count.y, 0);
}

// Return the packed index for one cell on a selected multigrid level.
int LevelIndex(int3 c, int nx, int ny, int offset)
{
	return offset + (c.z * ny + c.y) * nx + c.x;
}

// Map a multigrid column to the representative fine-grid column.
int2 FineColumnFromLevel(int2 c)
{
	return ClampColumn(int2(min(c.x * g.mg_scale + (g.mg_scale >> 1), g.cell_count.x - 1), min(c.y * g.mg_scale + (g.mg_scale >> 1), g.cell_count.y - 1)));
}

// Return floor height for a multigrid column through its representative fine column.
float FloorHeightLevel(int2 c)
{
	return FloorHeight(FineColumnFromLevel(c));
}

// Return world-space face height for a multigrid column.
float FaceZLevel(int2 c, int z)
{
	float floor_height = FloorHeightLevel(c);
	return floor_height + SigmaFace(z) * max(g.lid_z - floor_height, g.first_layer_thickness * max((float)g.cell_count.z, 1.0f));
}

// Return world-space centre height for a multigrid cell.
float CellZLevel(int2 c, int z)
{
	return 0.5f * (FaceZLevel(c, z) + FaceZLevel(c, z + 1));
}

// Return guarded physical height for a multigrid cell.
float CellDzLevel(int2 c, int z)
{
	return max(FaceZLevel(c, z + 1) - FaceZLevel(c, z), AtmosphereMinCellHeight);
}

// Return true when a coordinate is inside the active multigrid level.
bool InLevelCells(int3 c)
{
	return c.x >= 0 && c.x < g.mg_size.x && c.y >= 0 && c.y < g.mg_size.y && c.z >= 0 && c.z < g.cell_count.z;
}

// Return a one-sided or centred vertical pressure gradient on one level.
float VerticalPressureGradientLevel(int3 c, int nx, int ny, int offset)
{
	if (c.z <= 0)
	{
		return (PressureAtLevel(c + int3(0, 0, 1), nx, ny, offset) - PressureAtLevel(c, nx, ny, offset)) / max(CellZLevel(c.xy, 1) - CellZLevel(c.xy, 0), AtmosphereMinCellHeight);
	}
	if (c.z >= g.cell_count.z - 1)
	{
		return (PressureAtLevel(c, nx, ny, offset) - PressureAtLevel(c + int3(0, 0, -1), nx, ny, offset)) / max(CellZLevel(c.xy, g.cell_count.z - 1) - CellZLevel(c.xy, g.cell_count.z - 2), AtmosphereMinCellHeight);
	}
	return (PressureAtLevel(c + int3(0, 0, 1), nx, ny, offset) - PressureAtLevel(c + int3(0, 0, -1), nx, ny, offset)) / max(CellZLevel(c.xy, c.z + 1) - CellZLevel(c.xy, c.z - 1), AtmosphereMinCellHeight);
}

// Accumulate one pressure neighbour and its coefficient for the local column solve.
void AddPressureNeighbourLevel(int3 n, int boundary, float coeff, int nx, int ny, int offset, inout float sum, inout float denom)
{
	if (n.x >= 0 && n.x < nx && n.y >= 0 && n.y < ny && n.z >= 0 && n.z < g.cell_count.z)
	{
		sum += coeff * PressureAtLevel(n, nx, ny, offset);
		denom += coeff;
	}
	else if (boundary == BoundaryOpen)
	{
		denom += coeff;
	}
}

// Return the metric-corrected X pressure gradient at a level face.
float PressureGradientXLevel(int3 f, int nx, int ny, int offset)
{
	float h = g.dx * (float)g.mg_scale;
	if (f.x > 0 && f.x < nx)
	{
		int3 c0 = int3(f.x - 1, f.y, f.z);
		int3 c1 = int3(f.x, f.y, f.z);
		float dzdx = (CellZLevel(c1.xy, f.z) - CellZLevel(c0.xy, f.z)) / h;
		return (PressureAtLevel(c1, nx, ny, offset) - PressureAtLevel(c0, nx, ny, offset)) / h - dzdx * 0.5f * (VerticalPressureGradientLevel(c0, nx, ny, offset) + VerticalPressureGradientLevel(c1, nx, ny, offset));
	}
	if (f.x == 0 && BoundaryXMin() == BoundaryOpen)
		return PressureAtLevel(int3(0, f.y, f.z), nx, ny, offset) / h;
	if (f.x == nx && BoundaryXMax() == BoundaryOpen)
		return -PressureAtLevel(int3(nx - 1, f.y, f.z), nx, ny, offset) / h;
	return 0.0f;
}

// Return the metric-corrected Y pressure gradient at a level face.
float PressureGradientYLevel(int3 f, int nx, int ny, int offset)
{
	float h = g.dx * (float)g.mg_scale;
	if (f.y > 0 && f.y < ny)
	{
		int3 c0 = int3(f.x, f.y - 1, f.z);
		int3 c1 = int3(f.x, f.y, f.z);
		float dzdy = (CellZLevel(c1.xy, f.z) - CellZLevel(c0.xy, f.z)) / h;
		return (PressureAtLevel(c1, nx, ny, offset) - PressureAtLevel(c0, nx, ny, offset)) / h - dzdy * 0.5f * (VerticalPressureGradientLevel(c0, nx, ny, offset) + VerticalPressureGradientLevel(c1, nx, ny, offset));
	}
	if (f.y == 0 && BoundaryYMin() == BoundaryOpen)
		return PressureAtLevel(int3(f.x, 0, f.z), nx, ny, offset) / h;
	if (f.y == ny && BoundaryYMax() == BoundaryOpen)
		return -PressureAtLevel(int3(f.x, ny - 1, f.z), nx, ny, offset) / h;
	return 0.0f;
}

// Return the vertical pressure gradient at a level face.
float PressureGradientZLevel(int3 f, int nx, int ny, int offset)
{
	if (f.z > 0 && f.z < g.cell_count.z)
		return (PressureAtLevel(int3(f.x, f.y, f.z), nx, ny, offset) - PressureAtLevel(int3(f.x, f.y, f.z - 1), nx, ny, offset)) / CellDzLevel(f.xy, max(0, f.z - 1));
	return 0.0f;
}

// Return the terrain-following layer slope in X for one level cell.
float LevelTerrainSlopeX(int2 c, int z, int nx)
{
	int2 c0 = int2(max(0, c.x - 1), c.y);
	int2 c1 = int2(min(nx - 1, c.x + 1), c.y);
	return (CellZLevel(c1, z) - CellZLevel(c0, z)) / (g.dx * (float)g.mg_scale * (float)(abs(c1.x - c0.x) + AtmosphereSmallWeight));
}

// Return the terrain-following layer slope in Y for one level cell.
float LevelTerrainSlopeY(int2 c, int z, int ny)
{
	int2 c0 = int2(c.x, max(0, c.y - 1));
	int2 c1 = int2(c.x, min(ny - 1, c.y + 1));
	return (CellZLevel(c1, z) - CellZLevel(c0, z)) / (g.dx * (float)g.mg_scale * (float)(abs(c1.y - c0.y) + AtmosphereSmallWeight));
}

// Apply the metric pressure operator to one level cell.
float PressureOperatorLevel(int3 c, int nx, int ny, int offset)
{
	// The pressure matrix is the negative divergence of the same terrain-following pressure gradient used by projection. Matching these operators lets the V-cycle remove the divergence that projection will actually create on sloped sigma layers.
	float h = g.dx * (float)g.mg_scale;
	float dz = CellDzLevel(c.xy, c.z);
	float gx0 = PressureGradientXLevel(int3(c.x, c.y, c.z), nx, ny, offset);
	float gx1 = PressureGradientXLevel(int3(c.x + 1, c.y, c.z), nx, ny, offset);
	float gy0 = PressureGradientYLevel(int3(c.x, c.y, c.z), nx, ny, offset);
	float gy1 = PressureGradientYLevel(int3(c.x, c.y + 1, c.z), nx, ny, offset);
	float gz0 = PressureGradientZLevel(int3(c.x, c.y, c.z), nx, ny, offset);
	float gz1 = PressureGradientZLevel(int3(c.x, c.y, c.z + 1), nx, ny, offset);
	float base_div = (gx1 - gx0) / h + (gy1 - gy0) / h + (gz1 - gz0) / dz;
	float z_x = LevelTerrainSlopeX(c.xy, c.z, nx);
	float z_y = LevelTerrainSlopeY(c.xy, c.z, ny);
	int z0 = max(0, c.z - 1);
	int z1 = min(g.cell_count.z - 1, c.z + 1);
	float z_span = max(CellZLevel(c.xy, z1) - CellZLevel(c.xy, z0), AtmosphereMinCellHeight);
	float gx_lo = 0.5f * (PressureGradientXLevel(int3(c.x, c.y, z0), nx, ny, offset) + PressureGradientXLevel(int3(c.x + 1, c.y, z0), nx, ny, offset));
	float gx_hi = 0.5f * (PressureGradientXLevel(int3(c.x, c.y, z1), nx, ny, offset) + PressureGradientXLevel(int3(c.x + 1, c.y, z1), nx, ny, offset));
	float gy_lo = 0.5f * (PressureGradientYLevel(int3(c.x, c.y, z0), nx, ny, offset) + PressureGradientYLevel(int3(c.x, c.y + 1, z0), nx, ny, offset));
	float gy_hi = 0.5f * (PressureGradientYLevel(int3(c.x, c.y, z1), nx, ny, offset) + PressureGradientYLevel(int3(c.x, c.y + 1, z1), nx, ny, offset));
	return -(base_div - z_x * (gx_hi - gx_lo) / z_span - z_y * (gy_hi - gy_lo) / z_span);
}

// Return one entry of a unit pressure basis vector.
float PressureBasisAtLevel(int3 c, int3 target)
{
	return all(c == target) ? 1.0f : 0.0f;
}

// Return the vertical gradient of a unit pressure basis vector.
float VerticalPressureBasisGradientLevel(int3 c, int3 target)
{
	if (c.z <= 0)
	{
		return (PressureBasisAtLevel(c + int3(0, 0, 1), target) - PressureBasisAtLevel(c, target)) / max(CellZLevel(c.xy, 1) - CellZLevel(c.xy, 0), AtmosphereMinCellHeight);
	}
	if (c.z >= g.cell_count.z - 1)
	{
		return (PressureBasisAtLevel(c, target) - PressureBasisAtLevel(c + int3(0, 0, -1), target)) / max(CellZLevel(c.xy, g.cell_count.z - 1) - CellZLevel(c.xy, g.cell_count.z - 2), AtmosphereMinCellHeight);
	}
	return (PressureBasisAtLevel(c + int3(0, 0, 1), target) - PressureBasisAtLevel(c + int3(0, 0, -1), target)) / max(CellZLevel(c.xy, c.z + 1) - CellZLevel(c.xy, c.z - 1), AtmosphereMinCellHeight);
}

// Return the X gradient of a unit pressure basis vector.
float PressureBasisGradientXLevel(int3 f, int nx, int ny, int3 target)
{
	float h = g.dx * (float)g.mg_scale;
	if (f.x > 0 && f.x < nx)
	{
		int3 c0 = int3(f.x - 1, f.y, f.z);
		int3 c1 = int3(f.x, f.y, f.z);
		float dzdx = (CellZLevel(c1.xy, f.z) - CellZLevel(c0.xy, f.z)) / h;
		return (PressureBasisAtLevel(c1, target) - PressureBasisAtLevel(c0, target)) / h - dzdx * 0.5f * (VerticalPressureBasisGradientLevel(c0, target) + VerticalPressureBasisGradientLevel(c1, target));
	}
	if (f.x == 0 && BoundaryXMin() == BoundaryOpen)
		return PressureBasisAtLevel(int3(0, f.y, f.z), target) / h;
	if (f.x == nx && BoundaryXMax() == BoundaryOpen)
		return -PressureBasisAtLevel(int3(nx - 1, f.y, f.z), target) / h;
	return 0.0f;
}

// Return the Y gradient of a unit pressure basis vector.
float PressureBasisGradientYLevel(int3 f, int nx, int ny, int3 target)
{
	float h = g.dx * (float)g.mg_scale;
	if (f.y > 0 && f.y < ny)
	{
		int3 c0 = int3(f.x, f.y - 1, f.z);
		int3 c1 = int3(f.x, f.y, f.z);
		float dzdy = (CellZLevel(c1.xy, f.z) - CellZLevel(c0.xy, f.z)) / h;
		return (PressureBasisAtLevel(c1, target) - PressureBasisAtLevel(c0, target)) / h - dzdy * 0.5f * (VerticalPressureBasisGradientLevel(c0, target) + VerticalPressureBasisGradientLevel(c1, target));
	}
	if (f.y == 0 && BoundaryYMin() == BoundaryOpen)
		return PressureBasisAtLevel(int3(f.x, 0, f.z), target) / h;
	if (f.y == ny && BoundaryYMax() == BoundaryOpen)
		return -PressureBasisAtLevel(int3(f.x, ny - 1, f.z), target) / h;
	return 0.0f;
}

// Return the Z gradient of a unit pressure basis vector.
float PressureBasisGradientZLevel(int3 f, int3 target)
{
	if (f.z > 0 && f.z < g.cell_count.z)
		return (PressureBasisAtLevel(int3(f.x, f.y, f.z), target) - PressureBasisAtLevel(int3(f.x, f.y, f.z - 1), target)) / CellDzLevel(f.xy, max(0, f.z - 1));
	return 0.0f;
}

// Return the exact local diagonal of the metric pressure operator.
float PressureOperatorDiagonalLevel(int3 c, int nx, int ny)
{
	// The smoother uses the exact local diagonal of the metric operator instead of the simpler Cartesian stencil so sloped terrain corrections are relaxed instead of left for coarse-grid residuals only.
	float h = g.dx * (float)g.mg_scale;
	float dz = CellDzLevel(c.xy, c.z);
	float gx0 = PressureBasisGradientXLevel(int3(c.x, c.y, c.z), nx, ny, c);
	float gx1 = PressureBasisGradientXLevel(int3(c.x + 1, c.y, c.z), nx, ny, c);
	float gy0 = PressureBasisGradientYLevel(int3(c.x, c.y, c.z), nx, ny, c);
	float gy1 = PressureBasisGradientYLevel(int3(c.x, c.y + 1, c.z), nx, ny, c);
	float gz0 = PressureBasisGradientZLevel(int3(c.x, c.y, c.z), c);
	float gz1 = PressureBasisGradientZLevel(int3(c.x, c.y, c.z + 1), c);
	float base_div = (gx1 - gx0) / h + (gy1 - gy0) / h + (gz1 - gz0) / dz;
	float z_x = LevelTerrainSlopeX(c.xy, c.z, nx);
	float z_y = LevelTerrainSlopeY(c.xy, c.z, ny);
	int z0 = max(0, c.z - 1);
	int z1 = min(g.cell_count.z - 1, c.z + 1);
	float z_span = max(CellZLevel(c.xy, z1) - CellZLevel(c.xy, z0), AtmosphereMinCellHeight);
	float gx_lo = 0.5f * (PressureBasisGradientXLevel(int3(c.x, c.y, z0), nx, ny, c) + PressureBasisGradientXLevel(int3(c.x + 1, c.y, z0), nx, ny, c));
	float gx_hi = 0.5f * (PressureBasisGradientXLevel(int3(c.x, c.y, z1), nx, ny, c) + PressureBasisGradientXLevel(int3(c.x + 1, c.y, z1), nx, ny, c));
	float gy_lo = 0.5f * (PressureBasisGradientYLevel(int3(c.x, c.y, z0), nx, ny, c) + PressureBasisGradientYLevel(int3(c.x, c.y + 1, z0), nx, ny, c));
	float gy_hi = 0.5f * (PressureBasisGradientYLevel(int3(c.x, c.y, z1), nx, ny, c) + PressureBasisGradientYLevel(int3(c.x, c.y + 1, z1), nx, ny, c));
	return -(base_div - z_x * (gx_hi - gx_lo) / z_span - z_y * (gy_hi - gy_lo) / z_span);
}

// Return a residual correction for one pressure cell.
float PressureJacobiDeltaLevel(int3 c, int nx, int ny, int offset, float rhs)
{
	// Metric cross-terms couple nearby rows and layers, so the correction uses a local residual step instead of assuming a six-point Cartesian stencil.
	float diag = PressureOperatorDiagonalLevel(c, nx, ny);
	float op = PressureOperatorLevel(c, nx, ny, offset);
	return (rhs - op) / max(diag, AtmosphereSmallDiagonal);
}

// Return the neighbour-coefficient sum used by the column smoother.
float PressureDenomLevel(int3 c, int nx, int ny)
{
	float sum = 0.0f;
	float denom = 0.0f;
	float h = g.dx * (float)g.mg_scale;
	float idx2 = 1.0f / (h * h);
	float idz2 = 1.0f / (CellDzLevel(c.xy, c.z) * CellDzLevel(c.xy, c.z));
	AddPressureNeighbourLevel(c + int3(-1, 0, 0), BoundaryXMin(), idx2, nx, ny, g.mg_offset, sum, denom);
	AddPressureNeighbourLevel(c + int3( 1, 0, 0), BoundaryXMax(), idx2, nx, ny, g.mg_offset, sum, denom);
	AddPressureNeighbourLevel(c + int3(0, -1, 0), BoundaryYMin(), idx2, nx, ny, g.mg_offset, sum, denom);
	AddPressureNeighbourLevel(c + int3(0,  1, 0), BoundaryYMax(), idx2, nx, ny, g.mg_offset, sum, denom);
	AddPressureNeighbourLevel(c + int3(0, 0, -1), BoundarySolid, idz2, nx, ny, g.mg_offset, sum, denom);
	AddPressureNeighbourLevel(c + int3(0, 0,  1), BoundarySolid, idz2, nx, ny, g.mg_offset, sum, denom);
	return max(denom, AtmosphereSmallDiagonal);
}

// Return the mean pressure through one vertical column.
float AverageColumnPressureLevel(int2 c, int nx, int ny, int offset)
{
	float sum = 0.0f;
	for (int z = 0; z != g.cell_count.z; ++z)
		sum += PressureAtLevel(int3(c.x, c.y, z), nx, ny, offset);
	return sum / max((float)g.cell_count.z, 1.0f);
}

// Initialise both MAC velocity and scalar fields to a reference atmosphere at rest.
numthreads(CSInitialise, ATMOSPHERE_THREAD_X, ATMOSPHERE_THREAD_Y, ATMOSPHERE_THREAD_Z)
void CSInitialise(uint3 dtid : SV_DispatchThreadID)
{
	// Initialise every array element covered by this dispatch coordinate.
	int3 p = int3(dtid);
	if (InU(p)) g_u_out[UIndex(p)] = 0.0f;
	if (InV(p)) g_v_out[VIndex(p)] = 0.0f;
	if (InW(p)) g_w_out[WIndex(p)] = 0.0f;
	if (InCells(p))
	{
		g_temperature_out[CellIndex(p)] = ReferenceTemperature(CellCentre(p).z);
		g_pressure[CellIndex(p)] = 0.0f;
		g_divergence[CellIndex(p)] = 0.0f;
		g_residual[CellIndex(p)] = 0.0f;
	}
}

// Advect velocity and temperature through the previous MAC field using a clamped backtrace.
numthreads(CSAdvect, ATMOSPHERE_THREAD_X, ATMOSPHERE_THREAD_Y, ATMOSPHERE_THREAD_Z)
void CSAdvect(uint3 dtid : SV_DispatchThreadID)
{
	// Process every staggered sample type that exists at this dispatch coordinate.
	int3 p = int3(dtid);
	if (InU(p))
	{
		float3 pos = UFaceCentre(p);
		float3 prev = ClampToDomain(pos - SampleVelocity(pos) * g.dt);
		float value = SampleU(prev);
		if (OpenBoundaryUFace(p))
			value = OpenBoundaryU(p);

		g_u_out[UIndex(p)] = UFaceActive(p) ? value : 0.0f;
	}
	if (InV(p))
	{
		float3 pos = VFaceCentre(p);
		float3 prev = ClampToDomain(pos - SampleVelocity(pos) * g.dt);
		float value = SampleV(prev);
		if (OpenBoundaryVFace(p))
			value = OpenBoundaryV(p);

		g_v_out[VIndex(p)] = VFaceActive(p) ? value : 0.0f;
	}
	if (InW(p))
	{
		float3 pos = WFaceCentre(p);
		float3 prev = ClampToDomain(pos - SampleVelocity(pos) * g.dt);
		g_w_out[WIndex(p)] = WFaceActive(p) ? SampleW(prev) : 0.0f;
	}
	if (InCells(p))
	{
		float3 pos = CellCentre(p);
		float3 prev = ClampToDomain(pos - SampleVelocity(pos) * g.dt);
		float temp = SampleTemperature(prev);
		OutsideAir air;
		if (InflowOutsideAir(p, air))
		{
			// Inflow boundary cells take the temperature of the outside air that enters them.
			temp = ReferenceTemperature(pos.z) + air.temperature_offset;
		}
		g_temperature_out[CellIndex(p)] = temp;
	}
}

// Store the swirl magnitude of the advected field in the divergence scratch buffer for the confinement force in CSForcesHeat.
numthreads(CSVorticity, ATMOSPHERE_THREAD_X, ATMOSPHERE_THREAD_Y, ATMOSPHERE_THREAD_Z)
void CSVorticity(uint3 dtid : SV_DispatchThreadID)
{
	// Ignore coordinates outside the cell-centred scratch field.
	int3 c = int3(dtid);
	if (!InCells(c))
		return;

	g_divergence[CellIndex(c)] = length(CellVorticity(c));
}

// Apply forcing, buoyancy, floor exchange, lid relaxation, and heat sources.
numthreads(CSForcesHeat, ATMOSPHERE_THREAD_X, ATMOSPHERE_THREAD_Y, ATMOSPHERE_THREAD_Z)
void CSForcesHeat(uint3 dtid : SV_DispatchThreadID)
{
	// Process each velocity component and cell value that exists at this dispatch coordinate.
	int3 p = int3(dtid);
	bool confine = g.vorticity_confinement != 0.0f;
	if (InU(p))
	{
		// Restore lost swirl on interior faces using the average confinement force of the two cells that share the face.
		float value = g_u_in[UIndex(p)];
		if (confine && p.x > 0 && p.x < g.cell_count.x)
			value += 0.5f * (ConfinementAcceleration(p - int3(1, 0, 0)).x + ConfinementAcceleration(p).x) * g.dt;

		// Nudge the face toward the inflow outside wind inside the open-edge sponge.
		int2 col = ClampColumn(int2(min(p.x, g.cell_count.x - 1), p.y));
		float2 wind;
		float sponge = OpenEdgeWindBlend(col, wind);
		if (sponge > 0.0f)
			value = lerp(value, wind.x, saturate(AtmosphereOpenEdgeWindRate * sponge * g.dt));

		g_u_out[UIndex(p)] = UFaceActive(p) ? value : 0.0f;
	}
	if (InV(p))
	{
		// Restore lost swirl on interior faces using the average confinement force of the two cells that share the face.
		float value = g_v_in[VIndex(p)];
		if (confine && p.y > 0 && p.y < g.cell_count.y)
			value += 0.5f * (ConfinementAcceleration(p - int3(0, 1, 0)).y + ConfinementAcceleration(p).y) * g.dt;

		// Nudge the face toward the inflow outside wind inside the open-edge sponge.
		int2 col = ClampColumn(int2(p.x, min(p.y, g.cell_count.y - 1)));
		float2 wind;
		float sponge = OpenEdgeWindBlend(col, wind);
		if (sponge > 0.0f)
			value = lerp(value, wind.y, saturate(AtmosphereOpenEdgeWindRate * sponge * g.dt));

		g_v_out[VIndex(p)] = VFaceActive(p) ? value : 0.0f;
	}
	if (InW(p))
	{
		// Accelerate active faces by buoyancy from the temperature difference, plus the vertical confinement force.
		float accel = 0.0f;
		if (WFaceActive(p))
		{
			// Average the two cells that share the face; floor and lid faces clamp to their single neighbour.
			int z0 = max(0, p.z - 1);
			int z1 = min(g.cell_count.z - 1, p.z);
			float temp0 = g_temperature_in[CellIndex(int3(p.x, p.y, z0))];
			float temp1 = g_temperature_in[CellIndex(int3(p.x, p.y, z1))];
			float temp = 0.5f * (temp0 + temp1);
			float ref_temp = 0.5f * (ReferenceTemperature(CellZ(p.xy, z0)) + ReferenceTemperature(CellZ(p.xy, z1)));
			accel = g.gravity * (temp - ref_temp) / max(ref_temp, 1.0f);
			if (confine)
				accel += 0.5f * (ConfinementAcceleration(int3(p.x, p.y, z0)).z + ConfinementAcceleration(int3(p.x, p.y, z1)).z);
		}
		g_w_out[WIndex(p)] = WFaceActive(p) ? g_w_in[WIndex(p)] + accel * g.dt : 0.0f;
	}
	if (InCells(p))
	{
		float3 pos = CellCentre(p);
		float temp = g_temperature_in[CellIndex(p)];
		if (p.z == 0)
		{
			float floor_temp = g.use_floor_temp_buffer != 0 ? g_floor_temperature[p.y * g.cell_count.x + p.x] : g_floor_temperature[0];
			temp = lerp(temp, floor_temp, saturate(g.floor_exchange_rate * g.dt));
		}
		if (p.z == g.cell_count.z - 1)
		{
			temp = lerp(temp, g.lid_temperature, saturate(g.lid_relaxation_rate * g.dt));
		}
		for (int i = 0; i != g.source_count; ++i)
		{
			HeatSource src = g_sources[i];
			float dist = distance(pos, src.centre.xyz);
			if (dist <= src.radius)
			{
				float weight = 1.0f - dist / max(src.radius, AtmosphereMinHeatRadius);
				temp += src.heating_rate * weight * g.dt;
				if (src.relaxation_rate > 0.0f)
				{
					temp = lerp(temp, src.target_temperature, saturate(src.relaxation_rate * weight * g.dt));
				}
			}
		}
		g_temperature_out[CellIndex(p)] = temp;
	}
}

// Build the metric-corrected divergence of the intermediate MAC velocity.
numthreads(CSDivergence, ATMOSPHERE_THREAD_X, ATMOSPHERE_THREAD_Y, ATMOSPHERE_THREAD_Z)
void CSDivergence(uint3 dtid : SV_DispatchThreadID)
{
	// Ignore coordinates outside the cell-centred divergence field.
	int3 c = int3(dtid);
	if (!InCells(c))
		return;
	float dz = CellDz(c.xy, c.z);
	float base_div = (g_u_in[UIndex(int3(c.x + 1, c.y, c.z))] - g_u_in[UIndex(int3(c.x, c.y, c.z))]) / g.dx
		+ (g_v_in[VIndex(int3(c.x, c.y + 1, c.z))] - g_v_in[VIndex(int3(c.x, c.y, c.z))]) / g.dx
		+ (g_w_in[WIndex(int3(c.x, c.y, c.z + 1))] - g_w_in[WIndex(int3(c.x, c.y, c.z))]) / dz;
	float z_x = (CellZ(ClampColumn(c.xy + int2(1, 0)), c.z) - CellZ(ClampColumn(c.xy - int2(1, 0)), c.z)) / (g.dx * (float)(abs(ClampColumn(c.xy + int2(1, 0)).x - ClampColumn(c.xy - int2(1, 0)).x) + AtmosphereSmallWeight));
	float z_y = (CellZ(ClampColumn(c.xy + int2(0, 1)), c.z) - CellZ(ClampColumn(c.xy - int2(0, 1)), c.z)) / (g.dx * (float)(abs(ClampColumn(c.xy + int2(0, 1)).y - ClampColumn(c.xy - int2(0, 1)).y) + AtmosphereSmallWeight));
	int z0 = max(0, c.z - 1);
	int z1 = min(g.cell_count.z - 1, c.z + 1);
	float z_span = max(CellZ(c.xy, z1) - CellZ(c.xy, z0), AtmosphereMinCellHeight);
	float u_lo = 0.5f * (g_u_in[UIndex(int3(c.x, c.y, z0))] + g_u_in[UIndex(int3(c.x + 1, c.y, z0))]);
	float u_hi = 0.5f * (g_u_in[UIndex(int3(c.x, c.y, z1))] + g_u_in[UIndex(int3(c.x + 1, c.y, z1))]);
	float v_lo = 0.5f * (g_v_in[VIndex(int3(c.x, c.y, z0))] + g_v_in[VIndex(int3(c.x, c.y + 1, z0))]);
	float v_hi = 0.5f * (g_v_in[VIndex(int3(c.x, c.y, z1))] + g_v_in[VIndex(int3(c.x, c.y + 1, z1))]);
	g_divergence[CellIndex(c)] = base_div - z_x * (u_hi - u_lo) / z_span - z_y * (v_hi - v_lo) / z_span;
}

// Relax one red-black set of vertical columns on the active multigrid level.
numthreads(CSMgSmooth, ATMOSPHERE_THREAD_X, ATMOSPHERE_THREAD_Y, ATMOSPHERE_THREAD_Z)
void CSMgSmooth(uint3 dtid : SV_DispatchThreadID)
{
	// Select the first layer of one red-black column on the active multigrid level.
	int3 c = int3(dtid);
	if (!InLevelCells(c))
		return;
	if (c.z != 0)
		return;
	if (((c.x + c.y) & 1) != g.mg_phase)
		return;

	float h = g.dx * (float)g.mg_scale;
	float idx2 = 1.0f / (h * h);
	float lower[64];
	float diag[64];
	float upper[64];
	float rhs[64];
	float cp[64];
	float dp[64];
	for (int z = 0; z != g.cell_count.z; ++z)
	{
		int3 cz = int3(c.x, c.y, z);
		float sum = 0.0f;
		float denom = 0.0f;
		AddPressureNeighbourLevel(cz + int3(-1, 0, 0), BoundaryXMin(), idx2, g.mg_size.x, g.mg_size.y, g.mg_offset, sum, denom);
		AddPressureNeighbourLevel(cz + int3( 1, 0, 0), BoundaryXMax(), idx2, g.mg_size.x, g.mg_size.y, g.mg_offset, sum, denom);
		AddPressureNeighbourLevel(cz + int3(0, -1, 0), BoundaryYMin(), idx2, g.mg_size.x, g.mg_size.y, g.mg_offset, sum, denom);
		AddPressureNeighbourLevel(cz + int3(0,  1, 0), BoundaryYMax(), idx2, g.mg_size.x, g.mg_size.y, g.mg_offset, sum, denom);
		float dz = CellDzLevel(c.xy, z);
		float dz_lo = z != 0 ? CellDzLevel(c.xy, z - 1) : dz;
		lower[z] = z != 0 ? -1.0f / (dz * dz_lo) : 0.0f;
		upper[z] = z + 1 != g.cell_count.z ? -1.0f / (dz * dz) : 0.0f;
		diag[z] = denom - lower[z] - upper[z];
		rhs[z] = sum - g_divergence[LevelIndex(cz, g.mg_size.x, g.mg_size.y, g.mg_offset)];
	}
	cp[0] = upper[0] / max(diag[0], AtmosphereSmallDiagonal);
	dp[0] = rhs[0] / max(diag[0], AtmosphereSmallDiagonal);
	for (int z = 1; z != g.cell_count.z; ++z)
	{
		float m = 1.0f / max(diag[z] - lower[z] * cp[z - 1], AtmosphereSmallDiagonal);
		cp[z] = upper[z] * m;
		dp[z] = (rhs[z] - lower[z] * dp[z - 1]) * m;
	}
	float p = dp[g.cell_count.z - 1];
	g_pressure[LevelIndex(int3(c.x, c.y, g.cell_count.z - 1), g.mg_size.x, g.mg_size.y, g.mg_offset)] = p;
	for (int z = g.cell_count.z - 2; z >= 0; --z)
	{
		p = dp[z] - cp[z] * p;
		g_pressure[LevelIndex(int3(c.x, c.y, z), g.mg_size.x, g.mg_size.y, g.mg_offset)] = p;
	}
}

// Compute the multigrid residual on the active level.
numthreads(CSMgResidual, ATMOSPHERE_THREAD_X, ATMOSPHERE_THREAD_Y, ATMOSPHERE_THREAD_Z)
void CSMgResidual(uint3 dtid : SV_DispatchThreadID)
{
	// Ignore coordinates outside the active multigrid level.
	int3 c = int3(dtid);
	if (!InLevelCells(c))
		return;
	int idx = LevelIndex(c, g.mg_size.x, g.mg_size.y, g.mg_offset);
	g_residual[idx] = -g_divergence[idx] - PressureOperatorLevel(c, g.mg_size.x, g.mg_size.y, g.mg_offset);
}

// Restrict residuals from the active level into its horizontal child level.
numthreads(CSMgRestrict, ATMOSPHERE_THREAD_X, ATMOSPHERE_THREAD_Y, ATMOSPHERE_THREAD_Z)
void CSMgRestrict(uint3 dtid : SV_DispatchThreadID)
{
	// Average valid child residuals that overlap this coarse-grid column.
	int3 c = int3(dtid);
	if (c.x >= g.mg_child_size.x || c.y >= g.mg_child_size.y || c.z >= g.cell_count.z)
		return;
	float sum = 0.0f;
	float weight = 0.0f;
	for (int oy = 0; oy != 2; ++oy)
	{
		for (int ox = 0; ox != 2; ++ox)
		{
			int3 f = int3(c.x * 2 + ox, c.y * 2 + oy, c.z);
			if (f.x < g.mg_size.x && f.y < g.mg_size.y)
			{
				sum -= g_residual[LevelIndex(f, g.mg_size.x, g.mg_size.y, g.mg_offset)];
				weight += 1.0f;
			}
		}
	}
	int idx = LevelIndex(c, g.mg_child_size.x, g.mg_child_size.y, g.mg_child_offset);
	g_divergence[idx] = sum / max(weight, 1.0f);
	g_pressure[idx] = 0.0f;
}

// Prolongate child-level pressure corrections back into the active level.
numthreads(CSMgProlongate, ATMOSPHERE_THREAD_X, ATMOSPHERE_THREAD_Y, ATMOSPHERE_THREAD_Z)
void CSMgProlongate(uint3 dtid : SV_DispatchThreadID)
{
	// Add the nearest coarse correction to this fine pressure cell.
	int3 c = int3(dtid);
	if (!InLevelCells(c))
		return;
	int3 cc = int3(min(c.x / 2, g.mg_child_size.x - 1), min(c.y / 2, g.mg_child_size.y - 1), c.z);
	g_pressure[LevelIndex(c, g.mg_size.x, g.mg_size.y, g.mg_offset)] += g_pressure[LevelIndex(cc, g.mg_child_size.x, g.mg_child_size.y, g.mg_child_offset)];
}

// Remove the closed-domain pressure null space without changing pressure gradients.
numthreads(CSNormalisePressure, ATMOSPHERE_THREAD_X, ATMOSPHERE_THREAD_Y, ATMOSPHERE_THREAD_Z)
void CSNormalisePressure(uint3 dtid : SV_DispatchThreadID)
{
	// Use a stored reference pressure to remove the closed-domain null space.
	int3 c = int3(dtid);
	if (!InCells(c))
		return;
	bool all_solid = g.boundary_mask == 0;
	if (all_solid && g.mg_phase == 0)
	{
		if (c.x == 0 && c.y == 0)
			g_residual[CellIndex(c)] = PressureAt(c);
		return;
	}
	float reference = all_solid ? g_residual[CellIndex(int3(0, 0, c.z))] : 0.0f;
	g_pressure[CellIndex(c)] -= reference;
}

// Subtract the metric-corrected perturbation-pressure gradient from MAC face velocities.
numthreads(CSProject, ATMOSPHERE_THREAD_X, ATMOSPHERE_THREAD_Y, ATMOSPHERE_THREAD_Z)
void CSProject(uint3 dtid : SV_DispatchThreadID)
{
	// Project every staggered sample type that exists at this dispatch coordinate.
	int3 p = int3(dtid);
	if (InU(p))
	{
		float grad = 0.0f;
		if (p.x > 0 && p.x < g.cell_count.x)
		{
			int3 c0 = int3(p.x - 1, p.y, p.z);
			int3 c1 = int3(p.x, p.y, p.z);
			float dzdx = (CellZ(c1.xy, p.z) - CellZ(c0.xy, p.z)) / g.dx;
			grad = (PressureAt(c1) - PressureAt(c0)) / g.dx - dzdx * 0.5f * (VerticalPressureGradientLevel(c0, g.cell_count.x, g.cell_count.y, 0) + VerticalPressureGradientLevel(c1, g.cell_count.x, g.cell_count.y, 0));
		}
		else if (p.x == 0 && BoundaryXMin() == BoundaryOpen) grad = PressureAt(int3(0, p.y, p.z)) / g.dx;
		else if (p.x == g.cell_count.x && BoundaryXMax() == BoundaryOpen) grad = -PressureAt(int3(g.cell_count.x - 1, p.y, p.z)) / g.dx;
		float value = g_u_in[UIndex(p)] - grad;
		if (OpenBoundaryUFace(p))
			value = OpenBoundaryU(p);

		g_u_out[UIndex(p)] = UFaceActive(p) ? value : 0.0f;
	}
	if (InV(p))
	{
		float grad = 0.0f;
		if (p.y > 0 && p.y < g.cell_count.y)
		{
			int3 c0 = int3(p.x, p.y - 1, p.z);
			int3 c1 = int3(p.x, p.y, p.z);
			float dzdy = (CellZ(c1.xy, p.z) - CellZ(c0.xy, p.z)) / g.dx;
			grad = (PressureAt(c1) - PressureAt(c0)) / g.dx - dzdy * 0.5f * (VerticalPressureGradientLevel(c0, g.cell_count.x, g.cell_count.y, 0) + VerticalPressureGradientLevel(c1, g.cell_count.x, g.cell_count.y, 0));
		}
		else if (p.y == 0 && BoundaryYMin() == BoundaryOpen) grad = PressureAt(int3(p.x, 0, p.z)) / g.dx;
		else if (p.y == g.cell_count.y && BoundaryYMax() == BoundaryOpen) grad = -PressureAt(int3(p.x, g.cell_count.y - 1, p.z)) / g.dx;
		float value = g_v_in[VIndex(p)] - grad;
		if (OpenBoundaryVFace(p))
			value = OpenBoundaryV(p);

		g_v_out[VIndex(p)] = VFaceActive(p) ? value : 0.0f;
	}
	if (InW(p))
	{
		float grad = 0.0f;
		if (p.z > 0 && p.z < g.cell_count.z) grad = (PressureAt(int3(p.x, p.y, p.z)) - PressureAt(int3(p.x, p.y, p.z - 1))) / CellDz(p.xy, max(0, p.z - 1));
		g_w_out[WIndex(p)] = WFaceActive(p) ? g_w_in[WIndex(p)] - grad : 0.0f;
	}
	if (InCells(p))
	{
		g_temperature_out[CellIndex(p)] = g_temperature_in[CellIndex(p)];
	}
}

// Return a deterministic hash for tracer respawn sampling.
uint TracerHash(uint x)
{
	// Integer avalanching keeps neighbouring particle indexes visually decorrelated.
	x ^= x >> 16;
	x *= 0x7feb352du;
	x ^= x >> 15;
	x *= 0x846ca68bu;
	x ^= x >> 16;
	return x;
}

// Return a deterministic unit random value for one particle lane.
float TracerRand(uint particle_index, uint lane)
{
	// Mix the seed, particle index, frame, and lane so each coordinate has a stable independent sequence.
	uint h = TracerHash(gt.tracer_seed ^ (particle_index * 0x9e3779b9u) ^ (gt.tracer_frame * 0x85ebca6bu) ^ (lane * 0xc2b2ae35u));
	return ((float)(h & 0x00ffffffu) + 0.5f) / 16777216.0f;
}

// Return true when a world-space position is outside the terrain-following domain.
bool TracerOutside(float3 pos)
{
	// Tracers are visual probes and should respawn instead of clamping when they leave the valid domain.
	if (pos.x < g.origin.x || pos.x > g.origin.x + (float)g.cell_count.x * g.dx)
		return true;
	if (pos.y < g.origin.y || pos.y > g.origin.y + (float)g.cell_count.y * g.dx)
		return true;
	int2 col = ClampColumn((int2)floor(float2((pos.x - g.origin.x) / g.dx, (pos.y - g.origin.y) / g.dx)));
	return pos.z < FloorHeight(col) || pos.z > g.lid_z;
}

// Return a deterministic respawned tracer inside the terrain-following domain.
TracerParticle RespawnTracer(uint particle_index)
{
	// Sample XY uniformly by area, then sample sigma uniformly so flat and terrain-following grids both stay inside the local column.
	float rx = TracerRand(particle_index, 0u);
	float ry = TracerRand(particle_index, 1u);
	float rz = TracerRand(particle_index, 2u);
	float3 pos;
	pos.x = g.origin.x + rx * (float)g.cell_count.x * g.dx;
	pos.y = g.origin.y + ry * (float)g.cell_count.y * g.dx;
	int2 col = ClampColumn((int2)floor(float2((pos.x - g.origin.x) / g.dx, (pos.y - g.origin.y) / g.dx)));
	pos.z = FloorHeight(col) + rz * ColumnHeight(col);
	TracerParticle particle;
	particle.position = float4(pos, 1.0f);
	particle.temperature = SampleTemperature(pos);
	particle.age = 0.0f;
	particle.pad = float2(0.0f, 0.0f);
	return particle;
}

// Return a random point just inside one of the open side faces, chosen uniformly by side area.
// 'lane' selects the three random lanes used, so callers can draw several independent candidates.
float3 OpenSidePoint(uint particle_index, uint lane, out float3 inward)
{
	// Weight each open side by its length. Column height is ignored because the grid is a flat rectangle in XY.
	float lx = (float)g.cell_count.x * g.dx;
	float ly = (float)g.cell_count.y * g.dx;
	float w_xmin = (BoundaryXMin() == BoundaryOpen) ? ly : 0.0f;
	float w_xmax = (BoundaryXMax() == BoundaryOpen) ? ly : 0.0f;
	float w_ymin = (BoundaryYMin() == BoundaryOpen) ? lx : 0.0f;
	float w_ymax = (BoundaryYMax() == BoundaryOpen) ? lx : 0.0f;
	float total = w_xmin + w_xmax + w_ymin + w_ymax;

	// Pick a side, then a point along it. The point sits just inside the face so it is not immediately classed as outside.
	float pick = TracerRand(particle_index, lane + 0u) * total;
	float along = TracerRand(particle_index, lane + 1u);
	float rz = TracerRand(particle_index, lane + 2u);
	float inset = 1.0e-3f * g.dx;
	float3 pos;
	if (pick < w_xmin)
	{
		// West face.
		pos.x = g.origin.x + inset;
		pos.y = g.origin.y + along * ly;
		inward = float3(1.0f, 0.0f, 0.0f);
	}
	else if (pick < w_xmin + w_xmax)
	{
		// East face.
		pos.x = g.origin.x + lx - inset;
		pos.y = g.origin.y + along * ly;
		inward = float3(-1.0f, 0.0f, 0.0f);
	}
	else if (pick < w_xmin + w_xmax + w_ymin)
	{
		// South face.
		pos.x = g.origin.x + along * lx;
		pos.y = g.origin.y + inset;
		inward = float3(0.0f, 1.0f, 0.0f);
	}
	else
	{
		// North face.
		pos.x = g.origin.x + along * lx;
		pos.y = g.origin.y + ly - inset;
		inward = float3(0.0f, -1.0f, 0.0f);
	}

	// Sample height uniformly within the local column, as RespawnTracer does.
	int2 col = ClampColumn((int2)floor(float2((pos.x - g.origin.x) / g.dx, (pos.y - g.origin.y) / g.dx)));
	pos.z = FloorHeight(col) + rz * ColumnHeight(col);
	return pos;
}

// Return a deterministic tracer respawned on an open side face where the solved wind flows into the domain.
// Re-entry points are chosen roughly in proportion to the local inflow rate, so tracer density stays even along the flow.
// Falls back to RespawnTracer when none of the candidate points has inflow.
TracerParticle RespawnTracerAtInflow(uint particle_index)
{
	// Exact flux-weighted sampling would need the total inflow over every boundary face, which one thread cannot afford.
	// Instead, draw a few points uniformly over the open sides and keep one with probability proportional to its inflow speed.
	// Keeping a running total lets each candidate replace the current choice with probability 'its inflow / total inflow so far'.
	static const uint candidate_count = 8u;
	float3 chosen = float3(0.0f, 0.0f, 0.0f);
	float total = 0.0f;
	if (BoundaryXMin() == BoundaryOpen || BoundaryXMax() == BoundaryOpen || BoundaryYMin() == BoundaryOpen || BoundaryYMax() == BoundaryOpen)
	{
		for (uint i = 0u; i != candidate_count; ++i)
		{
			// Lanes 0-2 are used by RespawnTracer; each candidate uses three more lanes plus one for the keep decision.
			float3 inward;
			float3 pos = OpenSidePoint(particle_index, 3u + 4u * i, inward);
			float inflow = max(dot(SampleVelocity(pos), inward), 0.0f);
			if (inflow <= 0.0f)
				continue;

			total += inflow;
			if (TracerRand(particle_index, 6u + 4u * i) * total < inflow)
				chosen = pos;
		}
	}
	if (total <= 0.0f)
		return RespawnTracer(particle_index);

	TracerParticle particle;
	particle.position = float4(chosen, 1.0f);
	particle.temperature = SampleTemperature(chosen);
	particle.age = 0.0f;
	particle.pad = float2(0.0f, 0.0f);
	return particle;
}

// Initialise deterministic tracer particles inside the domain.
numthreads(CSInitialiseTracers, ATMOSPHERE_TRACER_THREAD_X, 1, 1)
void CSInitialiseTracers(uint3 dtid : SV_DispatchThreadID)
{
	// One thread owns one particle slot.
	uint particle_index = dtid.x;
	if (particle_index >= (uint)gt.tracer_count)
		return;

	g_tracers_out[particle_index] = RespawnTracer(particle_index);
}

// Advect tracer particles through the current MAC field.
numthreads(CSAdvectTracers, ATMOSPHERE_TRACER_THREAD_X, 1, 1)
void CSAdvectTracers(uint3 dtid : SV_DispatchThreadID)
{
	// Midpoint integration gives smoother tracks than a single Euler step while keeping this visual kernel cheap.
	uint particle_index = dtid.x;
	if (particle_index >= (uint)gt.tracer_count)
		return;

	TracerParticle particle = g_tracers_in[particle_index];
	float3 pos = particle.position.xyz;
	float3 v0 = SampleVelocity(pos);
	float3 mid = pos + 0.5f * g.dt * v0;
	float3 v1 = SampleVelocity(mid);
	pos += g.dt * v1;
	particle.age += g.dt;
	if (TracerOutside(pos))
	{
		// Tracers carried out of the domain re-enter with the inflow, keeping density even along the flow.
		particle = RespawnTracerAtInflow(particle_index);
	}
	else if (particle.age > gt.tracer_max_age)
	{
		// Expired tracers respawn anywhere in the volume. Uniform removal and uniform respawn keep the density unchanged.
		particle = RespawnTracer(particle_index);
	}
	else
	{
		particle.position = float4(pos, 1.0f);
		particle.temperature = SampleTemperature(pos);
	}
	g_tracers_out[particle_index] = particle;
}
