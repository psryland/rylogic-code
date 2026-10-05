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
// Solid columns:
//   A column whose floor is at or above the lid holds no air. All of its faces are walls with zero normal flow, it takes no
//   part in the pressure solve, and its pressure is zero. Neighbouring layer-slope terms use one-sided differences. Solid cells
//   keep the reference temperature of a flat-floor column. Trilinear sampling next to a wall still blends in those solid
//   values, so temperature and velocity samples within one cell of a wall are approximate. Coarse multigrid levels decide
//   solidity from one representative fine column; the fine-level smoother corrects the error this causes.
//
// One solver step records these passes in order:
//   1. CSAdvect traces velocity and temperature backward through the previous MAC field.
//   2. CSVorticity stores the swirl magnitude of the advected field (only when vorticity confinement is enabled).
//   3. CSForcesHeat applies buoyancy, vorticity confinement, floor exchange, lid relaxation, heat sources, the open-edge sponge, and wall drag.
//   4. CSDivergence builds the metric-corrected divergence of the intermediate MAC velocity.
//   5. CSMgSmooth, CSMgResidual, CSMgRestrict, and CSMgProlongate run the pressure V-cycle on the large levels; CSMgSmallLevels
//      runs the rest of the V-cycle, from the first level with at most ATMOSPHERE_FUSED_COLUMNS columns per axis, in one dispatch.
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
#define ATMOSPHERE_COARSE_COLUMNS 4 // the coarsest multigrid level has at most this many columns along X and Y
#define ATMOSPHERE_FUSED_COLUMNS 32 // levels with at most this many columns along X and Y run in one CSMgSmallLevels thread group
#define ATMOSPHERE_COLUMN_THREAD_X 8 // column kernels run one thread per column, so their groups are flat in Z
#define ATMOSPHERE_COLUMN_THREAD_Y 8
#define ATMOSPHERE_MAX_LAYERS 32 // largest supported 'cell_count.z'; sizes the per-thread vertical solve arrays. Must match MaxLayers in atmosphere.cpp

static const float AtmosphereMinLayerPower = 0.05f;       // dimensionless lower bound that keeps stretched-layer powers finite
static const float AtmosphereMinCellHeight = 0.001f;      // metres; prevents zero-thickness metric denominators
static const float AtmosphereSmallWeight = 1.0e-6f;       // dimensionless; protects interpolation denominators at clipped boundaries
static const float AtmosphereSmallDiagonal = 1.0e-20f;    // operator units; avoids division by zero in degenerate local stencils
static const float AtmosphereMinHeatRadius = 0.0001f;     // metres; avoids division by zero for point-like heat sources
static const float AtmosphereMinSwirlGradient = 1.0e-8f;  // 1/(s m); below this the swirl has no clear centre and confinement adds no force
static const float AtmosphereOpenEdgeWindRate = 8.0f;     // 1/s; blends outside inflow wind into the open-edge sponge over a short solver step
static const float AtmosphereSpecificHeat = 1004.5f;      // J/(kg K); dry air at constant pressure. Gravity divided by this is the dry adiabatic lapse rate
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
	// level's first cell in the packed buffers. The child (next coarser) level is derived from these; see MgChildLevel.
	int2 mg_size;                   // column counts of the active level in X and Y                                     @80
	int mg_offset;                  // index of the active level's first cell in the packed pressure buffers            @88
	int mg_scale;                   // number of fine-grid columns per active-level column along X and Y                @92
	int mg_phase;                   // red-black colour (0/1) for CSMgSmooth, or the reduction pass for CSNormalisePressure @96

	// Open edges and swirl restoration (used by CSForcesHeat).
	int open_edge_band;             // columns inside each open side over which the inflow outside wind is blended in    @100
	float vorticity_confinement;    // swirl-restoring strength; the acceleration is this times dx times the swirl rate, 1/s @104

	// Fused small-level V-cycle (used by CSMgSmallLevels).
	int mg_passes;                  // red-black smoothing passes on the coarsest level                                @108
	uint mg_smooth;                 // red-black smoothing passes on each level: before restriction in the low 16 bits, after the child correction is added in the high 16 bits @112

	// Vertical mixing of horizontal momentum (used by CSForcesHeat).
	float vertical_viscosity;       // vertical eddy viscosity of the horizontal wind; zero disables it, m^2/s          @116

	// Solid columns (see "Solid columns" above).
	float origin_z;                 // world-space Z of the flat floor used for the layer heights of solid columns, metres @120

	// Wall drag (used by CSForcesHeat). Dimensionless quadratic drag coefficient per outside face; zero is frictionless.
	// Each axis packs its min and max side as 16-bit floats (min in the low half) to keep the root signature within 64 DWORDs.
	uint drag_x;                    //                                                                                  @124
	uint drag_y;                    //                                                                                  @128
	uint drag_z;                    //                                                                                  @132
};

// Root constants shared by tracer kernels. Must match CBufAtmosphereTracers in atmosphere.cpp.
struct CBufAtmosphereTracers
{
	int tracer_count;               // number of tracer particles in g_tracers_in/g_tracers_out
	uint tracer_seed;               // deterministic seed mixed with particle index and frame counter
	uint tracer_frame;              // monotonically increasing tracer dispatch index
	float tracer_max_age;           // particle lifetime before deterministic respawn, seconds
	float tracer_ground_density;    // normalised tracer density at the floor. See TracerColumnFraction
	float tracer_break_density;     // normalised tracer density at the break height
	float tracer_upper_density;     // normalised tracer density from the break height to the lid
	float tracer_break_height;      // column fraction where the linear lower profile ends
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
	float speed;                    // magnitude of the air velocity that moved the particle in the last step, m/s
	float pad;                      // keeps the structure size a multiple of 16 bytes
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

// Return true when the nearest valid column is solid. A column whose floor reaches the lid holds no air, so all of its faces are walls.
bool ColumnSolid(int2 c)
{
	return FloorHeight(c) >= g.lid_z;
}

// Return 'n' when that neighbour column is air, otherwise 'c'. Layer-slope differences use this so a solid neighbour,
// whose floor is at the lid, does not create a false slope. The difference becomes one-sided next to a solid column.
int2 MetricNeighbour(int2 c, int2 n)
{
	n = ClampColumn(n);
	return ColumnSolid(n) ? c : n;
}

// Return the floor slope (dz/dx, dz/dy) under one fine-grid column. Solid neighbours are replaced by the column itself. See MetricNeighbour.
float2 FloorSlope(int2 c)
{
	// Use centred differences where both neighbours hold air, and one-sided differences at domain edges and beside solid columns.
	int2 cx0 = MetricNeighbour(c, c - int2(1, 0));
	int2 cx1 = MetricNeighbour(c, c + int2(1, 0));
	int2 cy0 = MetricNeighbour(c, c - int2(0, 1));
	int2 cy1 = MetricNeighbour(c, c + int2(0, 1));
	float sx = (FloorHeight(cx1) - FloorHeight(cx0)) / (g.dx * ((float)abs(cx1.x - cx0.x) + AtmosphereSmallWeight));
	float sy = (FloorHeight(cy1) - FloorHeight(cy0)) / (g.dx * ((float)abs(cy1.y - cy0.y) + AtmosphereSmallWeight));
	return float2(sx, sy);
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
	if (ColumnSolid(int2(f.x - 1, f.y)) || ColumnSolid(int2(f.x, f.y))) return false;
	if (f.x == 0) return BoundaryXMin() == BoundaryOpen;
	if (f.x == g.cell_count.x) return BoundaryXMax() == BoundaryOpen;
	return true;
}

// Return true when a V face can carry normal flow.
bool VFaceActive(int3 f)
{
	if (!InV(f)) return false;
	if (ColumnSolid(int2(f.x, f.y - 1)) || ColumnSolid(int2(f.x, f.y))) return false;
	if (f.y == 0) return BoundaryYMin() == BoundaryOpen;
	if (f.y == g.cell_count.y) return BoundaryYMax() == BoundaryOpen;
	return true;
}

// Return true when a W face can carry normal flow.
bool WFaceActive(int3 f)
{
	if (!InW(f)) return false;
	if (ColumnSolid(f.xy)) return false;
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

// Return the vertical velocity at the floor of an air column. Air cannot pass through the ground, so at the floor it moves along the
// slope: the vertical speed equals the bottom layer's horizontal velocity times the floor slope. On a flat floor this is zero.
float FloorW(int2 c)
{
	// Average the bottom-layer faces to the column centre, where the floor slope is measured.
	float u = 0.5f * (LoadU(int3(c.x, c.y, 0)) + LoadU(int3(c.x + 1, c.y, 0)));
	float v = 0.5f * (LoadV(int3(c.x, c.y, 0)) + LoadV(int3(c.x, c.y + 1, 0)));
	return dot(float2(u, v), FloorSlope(c));
}

// Return a W-face value or zero outside active flow faces. Floor faces of air columns follow the ground. See FloorW.
float LoadW(int3 f)
{
	if (f.z == 0 && InW(f) && !ColumnSolid(f.xy))
		return FloorW(f.xy);

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
	float floor_z = ColumnSolid(col) ? min(pos.z, g.lid_z) : FloorHeight(col);
	float z = clamp(pos.z, floor_z, g.lid_z);
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

// Return the wall drag of the bottom or top cell layer of column 'col' at layer 'z', 1/m. This is the drag coefficient divided by
// the layer thickness, summed over the floor and lid when the layer touches them. Zero when 'z' touches neither.
float FloorLidDrag(int2 col, int z)
{
	// The thickness of the touching layer sets how much air the wall must slow, so thin near-floor layers slow faster.
	float drag = 0.0f;
	if (z == 0)
		drag += f16tof32(g.drag_z & 0xFFFFu) / max(CellDz(col, 0), AtmosphereMinCellHeight);
	if (z == g.cell_count.z - 1)
		drag += f16tof32(g.drag_z >> 16) / max(CellDz(col, z), AtmosphereMinCellHeight);

	return drag;
}

// Return the wall drag on U face 'p', 1/m. U faces run along the y and z walls.
float WallDragU(int3 p)
{
	// Side walls are one column thick; the floor and lid use the layer thickness of the column on the low side of the face.
	float drag = FloorLidDrag(ClampColumn(int2(min(p.x, g.cell_count.x - 1), p.y)), p.z);
	if (p.y == 0)
		drag += f16tof32(g.drag_y & 0xFFFFu) / g.dx;
	if (p.y == g.cell_count.y - 1)
		drag += f16tof32(g.drag_y >> 16) / g.dx;

	return drag;
}

// Return the wall drag on V face 'p', 1/m. V faces run along the x and z walls.
float WallDragV(int3 p)
{
	// Side walls are one column thick; the floor and lid use the layer thickness of the column on the low side of the face.
	float drag = FloorLidDrag(ClampColumn(int2(p.x, min(p.y, g.cell_count.y - 1))), p.z);
	if (p.x == 0)
		drag += f16tof32(g.drag_x & 0xFFFFu) / g.dx;
	if (p.x == g.cell_count.x - 1)
		drag += f16tof32(g.drag_x >> 16) / g.dx;

	return drag;
}

// Return the wall drag on W face 'p', 1/m. W faces run along the x and y walls.
float WallDragW(int3 p)
{
	// Side walls are one column thick.
	float drag = 0.0f;
	if (p.x == 0)
		drag += f16tof32(g.drag_x & 0xFFFFu) / g.dx;
	if (p.x == g.cell_count.x - 1)
		drag += f16tof32(g.drag_x >> 16) / g.dx;
	if (p.y == 0)
		drag += f16tof32(g.drag_y & 0xFFFFu) / g.dx;
	if (p.y == g.cell_count.y - 1)
		drag += f16tof32(g.drag_y >> 16) / g.dx;

	return drag;
}

// Return 'value' slowed by quadratic wall drag over one step. 'drag' is the face's wall drag (1/m) and 'speed' is the local air speed (m/s).
// The deceleration rate is drag * speed. The implicit form slows the flow toward zero without reversing it, for any step size.
float ApplyWallDrag(float value, float drag, float speed)
{
	// Dividing by the growth factor is the implicit form of 'dv/dt = -drag * speed * v'.
	return value / (1.0f + g.dt * drag * speed);
}

// Return horizontal face value 'value' at layer 'z' after one step of vertical momentum mixing with the faces directly above and below.
// 'below' and 'above' are those neighbour values, 'col' is the column whose layer heights apply. Faces on the floor or lid mix only inward.
float MixVertical(float value, float below, float above, int2 col, int z)
{
	// The flux between two layers is the viscosity times the velocity difference over the distance between the layer centres.
	// Treating this face implicitly and its neighbours explicitly gives a weighted average of the three values, so the result
	// always lies between them and stays stable for any viscosity or step size.
	float dz = max(CellDz(col, z), AtmosphereMinCellHeight);
	float sum = value;
	float weight = 1.0f;
	if (z > 0)
	{
		// Mix with the layer below.
		float a = g.vertical_viscosity * g.dt / (dz * max(0.5f * (dz + CellDz(col, z - 1)), AtmosphereMinCellHeight));
		sum += a * below;
		weight += a;
	}
	if (z < g.cell_count.z - 1)
	{
		// Mix with the layer above.
		float a = g.vertical_viscosity * g.dt / (dz * max(0.5f * (dz + CellDz(col, z + 1)), AtmosphereMinCellHeight));
		sum += a * above;
		weight += a;
	}
	return sum / weight;
}


// One horizontally coarsened multigrid level. All levels are packed one after another into the pressure, divergence, and
// residual buffers. Every level keeps all 'cell_count.z' layers and has fewer columns than the level above it.
struct MgLevel
{
	int2 size;                      // column counts in X and Y
	int offset;                     // index of the level's first cell in the packed buffers
	int scale;                      // number of fine-grid columns per level column along X and Y
};

// Return the fine grid as a multigrid level.
MgLevel MgFineLevel()
{
	MgLevel level;
	level.size = g.cell_count.xy;
	level.offset = 0;
	level.scale = 1;
	return level;
}

// Return the level selected by the root constants.
MgLevel MgActiveLevel()
{
	MgLevel level;
	level.size = g.mg_size;
	level.offset = g.mg_offset;
	level.scale = g.mg_scale;
	return level;
}

// Return the next coarser level below 'level'. Must match BuildMultigridLevels in atmosphere.cpp.
MgLevel MgChildLevel(MgLevel level)
{
	// Each coarser level halves the column counts, rounding up, and is packed directly after its parent.
	MgLevel child;
	child.size = max(int2(1, 1), (level.size + 1) / 2);
	child.offset = level.offset + level.size.x * level.size.y * g.cell_count.z;
	child.scale = level.scale * 2;
	return child;
}

// Return the horizontal column spacing on 'level', metres.
float LevelDx(MgLevel level)
{
	return g.dx * (float)level.scale;
}

// Return true when a column is inside 'level'.
bool InLevelColumns(MgLevel level, int2 c)
{
	return all(c >= 0) && all(c < level.size);
}

// Return true when a cell is inside 'level'.
bool InLevelCells(MgLevel level, int3 c)
{
	return InLevelColumns(level, c.xy) && c.z >= 0 && c.z < g.cell_count.z;
}

// Return the packed index for one cell on 'level'.
int LevelIndex(MgLevel level, int3 c)
{
	return level.offset + (c.z * level.size.y + c.y) * level.size.x + c.x;
}

// Return pressure from one cell on 'level', or zero outside it.
float PressureAtLevel(MgLevel level, int3 c)
{
	return InLevelCells(level, c) ? g_pressure[LevelIndex(level, c)] : 0.0f;
}

// Return fine-level pressure for a cell coordinate.
float PressureAt(int3 c)
{
	return PressureAtLevel(MgFineLevel(), c);
}

// Map a column on 'level' to the representative fine-grid column.
int2 FineColumnFromLevel(MgLevel level, int2 c)
{
	return ClampColumn(min(c * level.scale + (level.scale >> 1), g.cell_count.xy - 1));
}

// Return true when the nearest valid column on 'level' is solid. Coarse columns take the state of their representative fine column.
bool LevelColumnSolid(MgLevel level, int2 c)
{
	return ColumnSolid(FineColumnFromLevel(level, clamp(c, int2(0, 0), level.size - 1)));
}

// Return 'n' when that neighbour column on 'level' is air, otherwise 'c'. See MetricNeighbour.
int2 LevelMetricNeighbour(MgLevel level, int2 c, int2 n)
{
	n = clamp(n, int2(0, 0), level.size - 1);
	return LevelColumnSolid(level, n) ? c : n;
}

// Return world-space face height for a column on 'level'.
float FaceZLevel(MgLevel level, int2 c, int z)
{
	// Coarse columns use the floor of their representative fine column.
	float floor_height = FloorHeight(FineColumnFromLevel(level, c));
	return floor_height + SigmaFace(z) * max(g.lid_z - floor_height, g.first_layer_thickness * max((float)g.cell_count.z, 1.0f));
}

// Return world-space centre height for a cell on 'level'.
float CellZLevel(MgLevel level, int2 c, int z)
{
	return 0.5f * (FaceZLevel(level, c, z) + FaceZLevel(level, c, z + 1));
}

// Return guarded physical height for a cell on 'level'.
float CellDzLevel(MgLevel level, int2 c, int z)
{
	return max(FaceZLevel(level, c, z + 1) - FaceZLevel(level, c, z), AtmosphereMinCellHeight);
}

// Return a one-sided or centred vertical pressure gradient on 'level'.
float VerticalPressureGradientLevel(MgLevel level, int3 c)
{
	if (c.z <= 0)
	{
		return (PressureAtLevel(level, c + int3(0, 0, 1)) - PressureAtLevel(level, c)) / max(CellZLevel(level, c.xy, 1) - CellZLevel(level, c.xy, 0), AtmosphereMinCellHeight);
	}
	if (c.z >= g.cell_count.z - 1)
	{
		return (PressureAtLevel(level, c) - PressureAtLevel(level, c + int3(0, 0, -1))) / max(CellZLevel(level, c.xy, g.cell_count.z - 1) - CellZLevel(level, c.xy, g.cell_count.z - 2), AtmosphereMinCellHeight);
	}
	return (PressureAtLevel(level, c + int3(0, 0, 1)) - PressureAtLevel(level, c + int3(0, 0, -1))) / max(CellZLevel(level, c.xy, c.z + 1) - CellZLevel(level, c.xy, c.z - 1), AtmosphereMinCellHeight);
}

// Accumulate one horizontal pressure neighbour and its coefficient for the column smoother.
// Open sides hold zero perturbation pressure outside the domain, so they add to the coefficient sum but not to the neighbour sum.
// Solid columns are walls with no flow through them, so they add to neither sum.
void AddPressureNeighbourLevel(MgLevel level, int3 n, int boundary, float coeff, inout float sum, inout float denom)
{
	if (InLevelCells(level, n))
	{
		if (LevelColumnSolid(level, n.xy))
			return;

		sum += coeff * g_pressure[LevelIndex(level, n)];
		denom += coeff;
	}
	else if (boundary == BoundaryOpen)
	{
		denom += coeff;
	}
}

// Return the metric-corrected X pressure gradient at a face on 'level'. Faces beside a solid column are walls with no gradient.
float PressureGradientXLevel(MgLevel level, int3 f)
{
	float h = LevelDx(level);
	if (LevelColumnSolid(level, int2(f.x - 1, f.y)) || LevelColumnSolid(level, int2(f.x, f.y)))
		return 0.0f;
	if (f.x > 0 && f.x < level.size.x)
	{
		int3 c0 = int3(f.x - 1, f.y, f.z);
		int3 c1 = int3(f.x, f.y, f.z);
		float dzdx = (CellZLevel(level, c1.xy, f.z) - CellZLevel(level, c0.xy, f.z)) / h;
		return (PressureAtLevel(level, c1) - PressureAtLevel(level, c0)) / h - dzdx * 0.5f * (VerticalPressureGradientLevel(level, c0) + VerticalPressureGradientLevel(level, c1));
	}
	if (f.x == 0 && BoundaryXMin() == BoundaryOpen)
		return PressureAtLevel(level, int3(0, f.y, f.z)) / h;
	if (f.x == level.size.x && BoundaryXMax() == BoundaryOpen)
		return -PressureAtLevel(level, int3(level.size.x - 1, f.y, f.z)) / h;
	return 0.0f;
}

// Return the metric-corrected Y pressure gradient at a face on 'level'. Faces beside a solid column are walls with no gradient.
float PressureGradientYLevel(MgLevel level, int3 f)
{
	float h = LevelDx(level);
	if (LevelColumnSolid(level, int2(f.x, f.y - 1)) || LevelColumnSolid(level, int2(f.x, f.y)))
		return 0.0f;
	if (f.y > 0 && f.y < level.size.y)
	{
		int3 c0 = int3(f.x, f.y - 1, f.z);
		int3 c1 = int3(f.x, f.y, f.z);
		float dzdy = (CellZLevel(level, c1.xy, f.z) - CellZLevel(level, c0.xy, f.z)) / h;
		return (PressureAtLevel(level, c1) - PressureAtLevel(level, c0)) / h - dzdy * 0.5f * (VerticalPressureGradientLevel(level, c0) + VerticalPressureGradientLevel(level, c1));
	}
	if (f.y == 0 && BoundaryYMin() == BoundaryOpen)
		return PressureAtLevel(level, int3(f.x, 0, f.z)) / h;
	if (f.y == level.size.y && BoundaryYMax() == BoundaryOpen)
		return -PressureAtLevel(level, int3(f.x, level.size.y - 1, f.z)) / h;
	return 0.0f;
}

// Return the vertical pressure gradient at a face on 'level'. Solid columns have no vertical flow.
float PressureGradientZLevel(MgLevel level, int3 f)
{
	if (f.z > 0 && f.z < g.cell_count.z && !LevelColumnSolid(level, f.xy))
		return (PressureAtLevel(level, f) - PressureAtLevel(level, int3(f.x, f.y, f.z - 1))) / CellDzLevel(level, f.xy, max(0, f.z - 1));
	return 0.0f;
}

// Return the terrain-following layer slope in X for one cell on 'level'. Solid neighbours are replaced by the cell's own column.
float LevelTerrainSlopeX(MgLevel level, int2 c, int z)
{
	int2 c0 = LevelMetricNeighbour(level, c, int2(c.x - 1, c.y));
	int2 c1 = LevelMetricNeighbour(level, c, int2(c.x + 1, c.y));
	return (CellZLevel(level, c1, z) - CellZLevel(level, c0, z)) / (LevelDx(level) * (float)(abs(c1.x - c0.x) + AtmosphereSmallWeight));
}

// Return the terrain-following layer slope in Y for one cell on 'level'. Solid neighbours are replaced by the cell's own column.
float LevelTerrainSlopeY(MgLevel level, int2 c, int z)
{
	int2 c0 = LevelMetricNeighbour(level, c, int2(c.x, c.y - 1));
	int2 c1 = LevelMetricNeighbour(level, c, int2(c.x, c.y + 1));
	return (CellZLevel(level, c1, z) - CellZLevel(level, c0, z)) / (LevelDx(level) * (float)(abs(c1.y - c0.y) + AtmosphereSmallWeight));
}

// Return the floor slope (dz/dx, dz/dy) under one column on 'level'. See FloorSlope.
float2 LevelFloorSlope(MgLevel level, int2 c)
{
	// Use the same neighbour rules as the layer slopes so the floor term follows the level's own geometry.
	int2 cx0 = LevelMetricNeighbour(level, c, int2(c.x - 1, c.y));
	int2 cx1 = LevelMetricNeighbour(level, c, int2(c.x + 1, c.y));
	int2 cy0 = LevelMetricNeighbour(level, c, int2(c.x, c.y - 1));
	int2 cy1 = LevelMetricNeighbour(level, c, int2(c.x, c.y + 1));
	float sx = (FaceZLevel(level, cx1, 0) - FaceZLevel(level, cx0, 0)) / (LevelDx(level) * ((float)abs(cx1.x - cx0.x) + AtmosphereSmallWeight));
	float sy = (FaceZLevel(level, cy1, 0) - FaceZLevel(level, cy0, 0)) / (LevelDx(level) * ((float)abs(cy1.y - cy0.y) + AtmosphereSmallWeight));
	return float2(sx, sy);
}

// Apply the metric pressure operator to one cell on 'level'. Solid cells hold zero pressure and have no equation.
float PressureOperatorLevel(MgLevel level, int3 c)
{
	// The pressure matrix is the negative divergence of the same terrain-following pressure gradient used by projection. Matching these operators lets the V-cycle remove the divergence that projection will actually create on sloped sigma layers.
	if (LevelColumnSolid(level, c.xy))
		return 0.0f;

	float h = LevelDx(level);
	float dz = CellDzLevel(level, c.xy, c.z);
	float gx0 = PressureGradientXLevel(level, int3(c.x, c.y, c.z));
	float gx1 = PressureGradientXLevel(level, int3(c.x + 1, c.y, c.z));
	float gy0 = PressureGradientYLevel(level, int3(c.x, c.y, c.z));
	float gy1 = PressureGradientYLevel(level, int3(c.x, c.y + 1, c.z));
	float gz0 = PressureGradientZLevel(level, int3(c.x, c.y, c.z));
	float gz1 = PressureGradientZLevel(level, int3(c.x, c.y, c.z + 1));

	// The floor face carries the horizontal gradient along the ground slope, matching the floor velocity used by the divergence. See FloorW.
	if (c.z == 0)
		gz0 = dot(float2(0.5f * (gx0 + gx1), 0.5f * (gy0 + gy1)), LevelFloorSlope(level, c.xy));

	float base_div = (gx1 - gx0) / h + (gy1 - gy0) / h + (gz1 - gz0) / dz;
	float z_x = LevelTerrainSlopeX(level, c.xy, c.z);
	float z_y = LevelTerrainSlopeY(level, c.xy, c.z);
	int z0 = max(0, c.z - 1);
	int z1 = min(g.cell_count.z - 1, c.z + 1);
	float z_span = max(CellZLevel(level, c.xy, z1) - CellZLevel(level, c.xy, z0), AtmosphereMinCellHeight);
	float gx_lo = 0.5f * (PressureGradientXLevel(level, int3(c.x, c.y, z0)) + PressureGradientXLevel(level, int3(c.x + 1, c.y, z0)));
	float gx_hi = 0.5f * (PressureGradientXLevel(level, int3(c.x, c.y, z1)) + PressureGradientXLevel(level, int3(c.x + 1, c.y, z1)));
	float gy_lo = 0.5f * (PressureGradientYLevel(level, int3(c.x, c.y, z0)) + PressureGradientYLevel(level, int3(c.x, c.y + 1, z0)));
	float gy_hi = 0.5f * (PressureGradientYLevel(level, int3(c.x, c.y, z1)) + PressureGradientYLevel(level, int3(c.x, c.y + 1, z1)));
	return -(base_div - z_x * (gx_hi - gx_lo) / z_span - z_y * (gy_hi - gy_lo) / z_span);
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
		// Solid columns have no layer heights, so their cells use the reference temperature of the same layer in a column standing on the grid origin.
		// Temperature samples beside a wall blend in these values, which keeps them close to the surrounding air.
		float z = ColumnSolid(p.xy) ? g.origin_z + SigmaCentre(p.z) * (g.lid_z - g.origin_z) : CellCentre(p).z;
		g_temperature_out[CellIndex(p)] = ReferenceTemperature(z);
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
		// Solid cells hold no air, so they keep their temperature.
		float3 pos = CellCentre(p);
		if (ColumnSolid(p.xy))
		{
			g_temperature_out[CellIndex(p)] = g_temperature_in[CellIndex(p)];
			return;
		}
		// Air that rises expands and cools at the dry adiabatic rate, and air that sinks warms. Without this, a lifted parcel would stay
		// warmer than the stable surrounding air and keep rising.
		float3 prev = ClampToDomain(pos - SampleVelocity(pos) * g.dt);
		float temp = SampleTemperature(prev) - g.gravity / AtmosphereSpecificHeat * (pos.z - prev.z);
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

// Apply forcing, buoyancy, wall drag, floor exchange, lid relaxation, and heat sources.
numthreads(CSForcesHeat, ATMOSPHERE_THREAD_X, ATMOSPHERE_THREAD_Y, ATMOSPHERE_THREAD_Z)
void CSForcesHeat(uint3 dtid : SV_DispatchThreadID)
{
	// Process each velocity component and cell value that exists at this dispatch coordinate.
	int3 p = int3(dtid);
	bool confine = g.vorticity_confinement != 0.0f;
	if (InU(p))
	{
		// Share momentum with the layers above and below, then restore lost swirl on interior faces using the average confinement force of the two cells that share the face.
		float value = g_u_in[UIndex(p)];
		int2 col = ClampColumn(int2(min(p.x, g.cell_count.x - 1), p.y));
		if (g.vertical_viscosity != 0.0f)
			value = MixVertical(value, g_u_in[UIndex(int3(p.xy, max(p.z - 1, 0)))], g_u_in[UIndex(int3(p.xy, min(p.z + 1, g.cell_count.z - 1)))], col, p.z);
		if (confine && p.x > 0 && p.x < g.cell_count.x)
			value += 0.5f * (ConfinementAcceleration(p - int3(1, 0, 0)).x + ConfinementAcceleration(p).x) * g.dt;

		// Nudge the face toward the outside wind inside the open-edge sponge.
		float2 wind;
		float sponge = OpenEdgeWindBlend(col, wind);
		if (sponge > 0.0f)
			value = lerp(value, wind.x, saturate(AtmosphereOpenEdgeWindRate * sponge * g.dt));

		// Slow faces that run along a dragged wall, using the air speed averaged over the two cells that share the face.
		float drag = WallDragU(p);
		if (drag > 0.0f)
			value = ApplyWallDrag(value, drag, 0.5f * length(CellVelocity(p - int3(1, 0, 0)) + CellVelocity(p)));

		g_u_out[UIndex(p)] = UFaceActive(p) ? value : 0.0f;
	}
	if (InV(p))
	{
		// Share momentum with the layers above and below, then restore lost swirl on interior faces using the average confinement force of the two cells that share the face.
		float value = g_v_in[VIndex(p)];
		int2 col = ClampColumn(int2(p.x, min(p.y, g.cell_count.y - 1)));
		if (g.vertical_viscosity != 0.0f)
			value = MixVertical(value, g_v_in[VIndex(int3(p.xy, max(p.z - 1, 0)))], g_v_in[VIndex(int3(p.xy, min(p.z + 1, g.cell_count.z - 1)))], col, p.z);
		if (confine && p.y > 0 && p.y < g.cell_count.y)
			value += 0.5f * (ConfinementAcceleration(p - int3(0, 1, 0)).y + ConfinementAcceleration(p).y) * g.dt;

		// Nudge the face toward the outside wind inside the open-edge sponge.
		float2 wind;
		float sponge = OpenEdgeWindBlend(col, wind);
		if (sponge > 0.0f)
			value = lerp(value, wind.y, saturate(AtmosphereOpenEdgeWindRate * sponge * g.dt));

		// Slow faces that run along a dragged wall, using the air speed averaged over the two cells that share the face.
		float drag = WallDragV(p);
		if (drag > 0.0f)
			value = ApplyWallDrag(value, drag, 0.5f * length(CellVelocity(p - int3(0, 1, 0)) + CellVelocity(p)));

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
		float value = g_w_in[WIndex(p)] + accel * g.dt;

		// Slow faces that run along a dragged side wall, using the air speed averaged over the two cells that share the face.
		float drag = WallDragW(p);
		if (drag > 0.0f)
			value = ApplyWallDrag(value, drag, 0.5f * length(CellVelocity(p - int3(0, 0, 1)) + CellVelocity(p)));

		g_w_out[WIndex(p)] = WFaceActive(p) ? value : 0.0f;
	}
	if (InCells(p))
	{
		// Solid cells hold no air, so floor, lid, and heat-source exchange do not apply to them.
		float3 pos = CellCentre(p);
		float temp = g_temperature_in[CellIndex(p)];
		if (ColumnSolid(p.xy))
		{
			g_temperature_out[CellIndex(p)] = temp;
			return;
		}
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

	// Solid cells have no pressure equation.
	if (ColumnSolid(c.xy))
	{
		g_divergence[CellIndex(c)] = 0.0f;
		return;
	}

	// Layer slopes use one-sided differences beside solid columns. See MetricNeighbour.
	float dz = CellDz(c.xy, c.z);
	float base_div = (g_u_in[UIndex(int3(c.x + 1, c.y, c.z))] - g_u_in[UIndex(int3(c.x, c.y, c.z))]) / g.dx
		+ (g_v_in[VIndex(int3(c.x, c.y + 1, c.z))] - g_v_in[VIndex(int3(c.x, c.y, c.z))]) / g.dx
		+ (g_w_in[WIndex(int3(c.x, c.y, c.z + 1))] - LoadW(int3(c.x, c.y, c.z))) / dz;
	int2 cx0 = MetricNeighbour(c.xy, c.xy - int2(1, 0));
	int2 cx1 = MetricNeighbour(c.xy, c.xy + int2(1, 0));
	int2 cy0 = MetricNeighbour(c.xy, c.xy - int2(0, 1));
	int2 cy1 = MetricNeighbour(c.xy, c.xy + int2(0, 1));
	float z_x = (CellZ(cx1, c.z) - CellZ(cx0, c.z)) / (g.dx * (float)(abs(cx1.x - cx0.x) + AtmosphereSmallWeight));
	float z_y = (CellZ(cy1, c.z) - CellZ(cy0, c.z)) / (g.dx * (float)(abs(cy1.y - cy0.y) + AtmosphereSmallWeight));
	int z0 = max(0, c.z - 1);
	int z1 = min(g.cell_count.z - 1, c.z + 1);
	float z_span = max(CellZ(c.xy, z1) - CellZ(c.xy, z0), AtmosphereMinCellHeight);
	float u_lo = 0.5f * (g_u_in[UIndex(int3(c.x, c.y, z0))] + g_u_in[UIndex(int3(c.x + 1, c.y, z0))]);
	float u_hi = 0.5f * (g_u_in[UIndex(int3(c.x, c.y, z1))] + g_u_in[UIndex(int3(c.x + 1, c.y, z1))]);
	float v_lo = 0.5f * (g_v_in[VIndex(int3(c.x, c.y, z0))] + g_v_in[VIndex(int3(c.x, c.y + 1, z0))]);
	float v_hi = 0.5f * (g_v_in[VIndex(int3(c.x, c.y, z1))] + g_v_in[VIndex(int3(c.x, c.y + 1, z1))]);
	g_divergence[CellIndex(c)] = base_div - z_x * (u_hi - u_lo) / z_span - z_y * (v_hi - v_lo) / z_span;
}

// Relax one vertical column of pressure on 'level'. Horizontal neighbours are read from 'g_pressure' and held fixed,
// and the column's own layers are solved together because the thin near-floor layers couple much more strongly in Z.
void RelaxColumn(MgLevel level, int2 xy)
{
	// The vertical system is tridiagonal. Forward elimination builds each row as it goes and keeps only the eliminated
	// coefficients 'cp' and 'dp', so each thread needs two small arrays instead of one array per matrix diagonal.
	if (LevelColumnSolid(level, xy))
	{
		// Solid columns hold no air and keep zero pressure.
		for (int sz = 0; sz != g.cell_count.z; ++sz)
			g_pressure[LevelIndex(level, int3(xy, sz))] = 0.0f;

		return;
	}

	float h = LevelDx(level);
	float idx2 = 1.0f / (h * h);
	float cp[ATMOSPHERE_MAX_LAYERS];
	float dp[ATMOSPHERE_MAX_LAYERS];
	float cp_lo = 0.0f;
	float dp_lo = 0.0f;
	float dz_lo = 0.0f;
	for (int z = 0; z != g.cell_count.z; ++z)
	{
		// Row 'z' couples the layer to the layers above and below; the horizontal neighbours move to the right-hand side.
		int3 cz = int3(xy, z);
		float sum = 0.0f;
		float denom = 0.0f;
		AddPressureNeighbourLevel(level, cz + int3(-1, 0, 0), BoundaryXMin(), idx2, sum, denom);
		AddPressureNeighbourLevel(level, cz + int3( 1, 0, 0), BoundaryXMax(), idx2, sum, denom);
		AddPressureNeighbourLevel(level, cz + int3(0, -1, 0), BoundaryYMin(), idx2, sum, denom);
		AddPressureNeighbourLevel(level, cz + int3(0,  1, 0), BoundaryYMax(), idx2, sum, denom);
		float dz = CellDzLevel(level, xy, z);
		float lower = z != 0 ? -1.0f / (dz * dz_lo) : 0.0f;
		float upper = z + 1 != g.cell_count.z ? -1.0f / (dz * dz) : 0.0f;
		float diag = denom - lower - upper;
		float rhs = sum - g_divergence[LevelIndex(level, cz)];

		// Eliminate the lower diagonal using the previous row.
		float m = 1.0f / max(diag - lower * cp_lo, AtmosphereSmallDiagonal);
		cp_lo = cp[z] = upper * m;
		dp_lo = dp[z] = (rhs - lower * dp_lo) * m;
		dz_lo = dz;
	}

	// Back substitution writes the relaxed pressures from the top layer down.
	float p = dp_lo;
	g_pressure[LevelIndex(level, int3(xy, g.cell_count.z - 1))] = p;
	for (int z = g.cell_count.z - 2; z >= 0; --z)
	{
		p = dp[z] - cp[z] * p;
		g_pressure[LevelIndex(level, int3(xy, z))] = p;
	}
}

// Relax one red-black colour of vertical columns on the active multigrid level.
// Each thread owns one column of the selected colour: thread x selects the x-th column of that colour within its row.
numthreads(CSMgSmooth, ATMOSPHERE_COLUMN_THREAD_X, ATMOSPHERE_COLUMN_THREAD_Y, 1)
void CSMgSmooth(uint3 dtid : SV_DispatchThreadID)
{
	// Map the thread to the column of colour 'mg_phase', skipping threads past the end of the row.
	MgLevel level = MgActiveLevel();
	int y = (int)dtid.y;
	int2 c = int2(2 * (int)dtid.x + ((y + g.mg_phase) & 1), y);
	if (!InLevelColumns(level, c))
		return;

	RelaxColumn(level, c);
}

// Compute the multigrid residual of cell 'c' on 'level'.
void ResidualCell(MgLevel level, int3 c)
{
	// The residual is the part of the pressure equation that the current pressure does not yet satisfy. Solid cells have no equation.
	int idx = LevelIndex(level, c);
	g_residual[idx] = LevelColumnSolid(level, c.xy) ? 0.0f : -g_divergence[idx] - PressureOperatorLevel(level, c);
}

// Restrict the residuals of 'level' into cell 'c' of its child level, and clear the child's pressure for a fresh correction.
void RestrictCell(MgLevel level, int3 c)
{
	// Average the valid residuals of the up-to-four columns that this child cell covers.
	MgLevel child = MgChildLevel(level);
	float sum = 0.0f;
	float weight = 0.0f;
	for (int oy = 0; oy != 2; ++oy)
	{
		for (int ox = 0; ox != 2; ++ox)
		{
			// Odd-sized levels have child columns that cover only one column along that axis.
			int3 f = int3(c.x * 2 + ox, c.y * 2 + oy, c.z);
			if (InLevelColumns(level, f.xy))
			{
				sum -= g_residual[LevelIndex(level, f)];
				weight += 1.0f;
			}
		}
	}
	int idx = LevelIndex(child, c);
	g_divergence[idx] = sum / max(weight, 1.0f);
	g_pressure[idx] = 0.0f;
}

// Add the child-level pressure correction to cell 'c' of 'level'.
void ProlongateCell(MgLevel level, int3 c)
{
	// Each cell takes the correction of the child cell that covers it.
	MgLevel child = MgChildLevel(level);
	int3 cc = int3(min(c.xy / 2, child.size - 1), c.z);
	g_pressure[LevelIndex(level, c)] += g_pressure[LevelIndex(child, cc)];
}

// Compute the multigrid residual on the active level.
numthreads(CSMgResidual, ATMOSPHERE_THREAD_X, ATMOSPHERE_THREAD_Y, ATMOSPHERE_THREAD_Z)
void CSMgResidual(uint3 dtid : SV_DispatchThreadID)
{
	// Ignore coordinates outside the active multigrid level.
	MgLevel level = MgActiveLevel();
	int3 c = int3(dtid);
	if (!InLevelCells(level, c))
		return;

	ResidualCell(level, c);
}

// Restrict residuals from the active level into its child level. Dispatched over the child level's cells.
numthreads(CSMgRestrict, ATMOSPHERE_THREAD_X, ATMOSPHERE_THREAD_Y, ATMOSPHERE_THREAD_Z)
void CSMgRestrict(uint3 dtid : SV_DispatchThreadID)
{
	// Ignore coordinates outside the child level.
	MgLevel level = MgActiveLevel();
	int3 c = int3(dtid);
	if (!InLevelCells(MgChildLevel(level), c))
		return;

	RestrictCell(level, c);
}

// Prolongate child-level pressure corrections back into the active level.
numthreads(CSMgProlongate, ATMOSPHERE_THREAD_X, ATMOSPHERE_THREAD_Y, ATMOSPHERE_THREAD_Z)
void CSMgProlongate(uint3 dtid : SV_DispatchThreadID)
{
	// Ignore coordinates outside the active multigrid level.
	MgLevel level = MgActiveLevel();
	int3 c = int3(dtid);
	if (!InLevelCells(level, c))
		return;

	ProlongateCell(level, c);
}

// Run 'passes' red-black smoothing passes on 'level' within one thread group, where thread 'xy' owns column 'xy'.
// Must be called by every thread in the group, because it contains group barriers.
void GroupSmooth(MgLevel level, int2 xy, int passes)
{
	// A group-wide device memory barrier after each colour makes that colour's writes visible before the other colour reads them.
	bool active = InLevelColumns(level, xy);
	int colour = (xy.x + xy.y) & 1;
	for (int pass = 0; pass != passes; ++pass)
	{
		// Each pass relaxes both colours in order, matching the dispatch-per-colour smoother.
		for (int phase = 0; phase != 2; ++phase)
		{
			// Only columns of the current colour write; the other colour's pressures are read as neighbours.
			if (active && colour == phase)
				RelaxColumn(level, xy);

			DeviceMemoryBarrierWithGroupSync();
		}
	}
}

// Run the remainder of a V-cycle, from the active level down to the coarsest level and back, in a single thread group.
// The active level has at most ATMOSPHERE_FUSED_COLUMNS columns along X and Y, so each thread owns one column on every level.
// Small levels have too few columns to fill the GPU, so running them in one group with group barriers replaces many short
// dispatches and UAV barriers. Uses the pre- and post-smoothing counts packed in 'mg_smooth' on each level above the coarsest, and
// 'mg_passes' passes on the coarsest level.
numthreads(CSMgSmallLevels, ATMOSPHERE_FUSED_COLUMNS, ATMOSPHERE_FUSED_COLUMNS, 1)
void CSMgSmallLevels(uint3 gtid : SV_GroupThreadID)
{
	// Count the levels between the active level and the coarsest level. Every thread must reach each barrier,
	// so threads outside a level skip its work instead of returning.
	MgLevel top = MgActiveLevel();
	int2 xy = int2(gtid.xy);
	int depth = 0;
	for (MgLevel l = top; any(l.size > ATMOSPHERE_COARSE_COLUMNS); l = MgChildLevel(l))
		++depth;

	// Down sweep: smooth each level, then pass its residual to the child level as the child's equation.
	MgLevel level = top;
	for (int d = 0; d != depth; ++d)
	{
		// Pre-smoothing damps cell-scale errors before the residual is restricted.
		GroupSmooth(level, xy, int(g.mg_smooth & 0xFFFFu));
		if (InLevelColumns(level, xy))
		{
			// Each thread computes the residual for its own column.
			for (int z = 0; z != g.cell_count.z; ++z)
				ResidualCell(level, int3(xy, z));
		}
		DeviceMemoryBarrierWithGroupSync();

		// Restriction reads the residuals of up to four columns, so it waits for the barrier above.
		MgLevel child = MgChildLevel(level);
		if (InLevelColumns(child, xy))
		{
			// Each thread restricts into its own column of the child level.
			for (int z = 0; z != g.cell_count.z; ++z)
				RestrictCell(level, int3(xy, z));
		}
		DeviceMemoryBarrierWithGroupSync();
		level = child;
	}

	// Smooth the coarsest level many times; it is small enough that this approximates an exact solve.
	GroupSmooth(level, xy, g.mg_passes);

	// Up sweep: add each child's correction to its parent level, then post-smooth the parent.
	for (int d = depth - 1; d >= 0; --d)
	{
		// Levels store no link to their parent, so walk down from the active level to level 'd'.
		MgLevel parent = top;
		for (int i = 0; i != d; ++i)
			parent = MgChildLevel(parent);

		if (InLevelColumns(parent, xy))
		{
			// Each thread corrects its own column.
			for (int z = 0; z != g.cell_count.z; ++z)
				ProlongateCell(parent, int3(xy, z));
		}
		DeviceMemoryBarrierWithGroupSync();
		GroupSmooth(parent, xy, int(g.mg_smooth >> 16));
	}
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
	// Project every staggered sample type that exists at this dispatch coordinate. The fine-level gradients match the operator
	// used by the pressure solve, including open sides and solid columns.
	int3 p = int3(dtid);
	if (InU(p))
	{
		float value = g_u_in[UIndex(p)] - PressureGradientXLevel(MgFineLevel(), p);
		if (OpenBoundaryUFace(p))
			value = OpenBoundaryU(p);

		g_u_out[UIndex(p)] = UFaceActive(p) ? value : 0.0f;
	}
	if (InV(p))
	{
		float value = g_v_in[VIndex(p)] - PressureGradientYLevel(MgFineLevel(), p);
		if (OpenBoundaryVFace(p))
			value = OpenBoundaryV(p);

		g_v_out[VIndex(p)] = VFaceActive(p) ? value : 0.0f;
	}
	if (InW(p))
	{
		g_w_out[WIndex(p)] = WFaceActive(p) ? g_w_in[WIndex(p)] - PressureGradientZLevel(MgFineLevel(), p) : 0.0f;
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

// Map a unit random value 'r' to a fraction of the column height (0 at the floor, 1 at the lid) for a new tracer.
// The density of new tracers falls linearly from 'tracer_ground_density' at the floor to 'tracer_break_density' at 'tracer_break_height',
// then is 'tracer_upper_density' up to the lid. The densities are normalised so the integral over the column is 1.
float TracerColumnFraction(float r)
{
	// Invert the cumulative density. Below the break it is 'ground * s + (brk - ground) * s^2 / (2 h)', which reaches 'lower_share' at s = h.
	float ground = gt.tracer_ground_density;
	float brk = gt.tracer_break_density;
	float upper = gt.tracer_upper_density;
	float h = gt.tracer_break_height;
	float lower_share = 0.5f * h * (ground + brk);
	if (r >= lower_share)
		return min(h + (r - lower_share) / max(upper, 1e-20f), 1.0f);

	// This form of the quadratic root avoids cancellation and stays finite when the lower density is even (ground == brk).
	return 2.0f * r / (ground + sqrt(max(ground * ground + 2.0f * (brk - ground) * r / h, 0.0f)));
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
	return ColumnSolid(col) || pos.z < FloorHeight(col) || pos.z > g.lid_z;
}

// Return a new tracer at rest at 'pos', sampling the local air.
TracerParticle MakeTracer(float3 pos)
{
	// Speed is sampled here so a respawned tracer is coloured correctly before its first advection step.
	TracerParticle particle;
	particle.position = float4(pos, 1.0f);
	particle.temperature = SampleTemperature(pos);
	particle.age = 0.0f;
	particle.speed = length(SampleVelocity(pos));
	particle.pad = 0.0f;
	return particle;
}

// Return a deterministic respawned tracer inside the terrain-following domain.
// Positions inside solid columns are redrawn a few times. If every draw is solid, the tracer respawns again on the next step.
TracerParticle RespawnTracer(uint particle_index)
{
	// Sample XY uniformly by area, then sample a fraction of the local column height so flat and terrain-following grids both stay inside the column. See TracerColumnFraction.
	// The first draw uses lanes 0-2. Redraws use lanes above those used by RespawnTracerAtInflow.
	static const uint attempt_count = 4u;
	float3 pos = float3(0.0f, 0.0f, 0.0f);
	for (uint attempt = 0u; attempt != attempt_count; ++attempt)
	{
		// Draw one candidate position from three random lanes.
		uint lane = attempt == 0u ? 0u : 64u + 3u * attempt;
		float rx = TracerRand(particle_index, lane + 0u);
		float ry = TracerRand(particle_index, lane + 1u);
		float rz = TracerRand(particle_index, lane + 2u);
		pos.x = g.origin.x + rx * (float)g.cell_count.x * g.dx;
		pos.y = g.origin.y + ry * (float)g.cell_count.y * g.dx;
		int2 col = ClampColumn((int2)floor(float2((pos.x - g.origin.x) / g.dx, (pos.y - g.origin.y) / g.dx)));
		pos.z = FloorHeight(col) + TracerColumnFraction(rz) * ColumnHeight(col);
		if (!ColumnSolid(col))
			break;
	}
	return MakeTracer(pos);
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

	// Sample height within the local column with the same profile as RespawnTracer.
	int2 col = ClampColumn((int2)floor(float2((pos.x - g.origin.x) / g.dx, (pos.y - g.origin.y) / g.dx)));
	pos.z = FloorHeight(col) + TracerColumnFraction(rz) * ColumnHeight(col);
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
			if (TracerOutside(pos))
				continue;

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

	return MakeTracer(chosen);
}

// Initialise deterministic tracer particles inside the domain.
numthreads(CSInitialiseTracers, ATMOSPHERE_TRACER_THREAD_X, 1, 1)
void CSInitialiseTracers(uint3 dtid : SV_DispatchThreadID)
{
	// One thread owns one particle slot.
	uint particle_index = dtid.x;
	if (particle_index >= (uint)gt.tracer_count)
		return;

	// Start each tracer part way through its lifetime. If all tracers started at age zero, they would all expire on the same frame
	// and respawn together, causing a visible jump in the density pattern. Lane 128 is above the lanes used by the respawn functions.
	TracerParticle particle = RespawnTracer(particle_index);
	particle.age = TracerRand(particle_index, 128u) * gt.tracer_max_age;
	g_tracers_out[particle_index] = particle;
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
		// Expired tracers respawn anywhere in the volume. Removal does not depend on position, so the density relaxes towards the respawn profile.
		particle = RespawnTracer(particle_index);
	}
	else
	{
		particle.position = float4(pos, 1.0f);
		particle.temperature = SampleTemperature(pos);
		particle.speed = length(v1);
	}
	g_tracers_out[particle_index] = particle;
}
