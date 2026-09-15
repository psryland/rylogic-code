# Baseline terrain evaluator

`BaselineSurface` prepares a bounded immutable recipe on the CPU. Ordinary CPU
queries, compute shaders, and vertex shaders use the same terrain composition,
noise hash, interpolation, analytic derivatives, and domain-warp Jacobian.
There is no GPU dispatch inside a CPU height query.

## CPU

```cpp
#include "pr/physics/terrain/landscape/baseline_surface.h"

using namespace pr::physics::terrain::landscape;
auto surface = BaselineSurface(BaselineSurfaceConfig{.m_seed = 42});
auto sample = surface.Sample(BaselineSurface::Position{-125.5, 300.0});
auto const& recipe = surface.Recipe();
```

Link `physics-landscape.lib` or `physics-static.lib`. The ordinary public header
does not include physics engine, device, queue, or GPU resource headers. The
existing `Config()`, scalar/batch `Sample()`, surface result, material identifiers,
seed expansion, and double-valued configuration remain the CPU interface.
Batch input/output spans must have equal lengths. They are borrowed only during
the call; order and scalar results are preserved. A failing element can leave an
already-written prefix, as with scalar iteration.

## Shader stages

```hlsl
#include "pr/physics/terrain/landscape/baseline_surface.hlsli"

// The consumer chooses all resources, register assignments, and lifetime policy.
StructuredBuffer<BaselineRecipe> terrain_recipes : register(t3);

// Inside either a vertex or compute entry point:
// BaselineResult sample = BaselineEvaluate(terrain_recipes[0], world_xy_double);
// Use sample only when sample.m_status == 0.
```

Upload the bytes of `surface.Recipe()` as **one StructuredBuffer element**, not a
legacy cbuffer. Bind the buffer as read-only shader data. The evaluator itself
declares no registers, buffers, UAVs, barriers, thread groups, or dispatch
intrinsics. It allocates no storage and performs no caching.
The C++ form of the shader entry is
`pr::physics::terrain::landscape::shared::BaselineEvaluate`.

### Precision and hardware requirements

* DXC, HLSL 2021, shader model **6.0 or newer**, compiled with **`-Gis`**.
  Both optimized and unoptimized strict compilation are tested.
* Require D3D12 `DoublePrecisionFloatShaderOps`, `Int64ShaderOps`, and a sufficient
  shader model. DXIL additionally declares **double-precision extensions for
  11.1**; pipeline creation must succeed on the target device.
* CPU evaluation remains FP64. HLSL field arithmetic defaults to FP32.
  Define `PR_TERRAIN_FP32=0` before inclusion to compile the same arithmetic
  with FP64 fields for a standalone shader consumer or precision comparison.
  Precision selection is compile-time, with no runtime fallback.
* World coordinates, frequency, lattice/seed phase, the additive height datum,
  and structured-buffer representations remain FP64. Seeds and signed lattice
  coordinates retain the original 64-bit hash; there is no replacement hash.
  Local lattice fractions are converted only after subtracting the double floor;
  seed-dependent slices near 65536 therefore retain their phase.
* HLSL's float-only `floor` and `sqrt` overloads are not used for double queries.
  Floor uses checked integer conversion. The square-root boundary uses native
  `std::sqrt` on the CPU. HLSL normalizes the double mantissa into `[1,4)`, uses a
  float reciprocal-root seed, then performs two double Newton refinements and
  restores the exponent. The float seed does not determine output precision;
  exponent-range tests include subnormals and the maximum finite double.
  FP32 rounded ridges use the native float square root. No terrain formula
  differs between the language front ends.
* Constants use the selected arithmetic type, with explicit double constants
  where phase requires them. Conversion to a renderer's float vertex position
  belongs **after** terrain evaluation.

These requirements exclude some devices and can be slow on consumer GPUs.
No cross-device bit identity or frame-time budget is claimed.

The easing polynomial is evaluated by midpoint symmetry, and its derivative as
`30*t*t*(1-t)*(1-t)`. This avoids cancellation near one in both languages without
changing the polynomial or the seed landscape. Original CPU goldens retain their
unchanged `1e-12` mixed tolerance. The final datum is added in double outside
FP32 field reduction, so a large altitude offset does not quantize local relief.
Corner gradients use a packed two-bit-per-component encoding of the original
16-entry table, shared by both languages and checked independently in CPU tests.

The measured acceptance gates are **absolute height error <= 0.001 m**, **absolute
error in each XY gradient component <= 1e-4**, and **normal-vector difference
length <= 1e-4**. No tolerance scales with reference height, including high datum
and near-zero/cancellation cases. These are gates on the representative matrix,
**not universal error bounds for arbitrary supported configurations**.

On an NVIDIA GeForce RTX 3080 Ti with DXC 1.8 (DLL 1.8.2407.7), both strict
`-Od` and `-O3` passed the same matrix. FP32 maxima were 0.307110 mm in height,
`5.139115e-5` in DX, `5.219188e-5` in DY, and `3.769469e-5` in normal-vector
difference length. Normals were formed on the GPU using the selected arithmetic
precision and read back, not reconstructed from GPU derivatives on the CPU.
FP64 GPU maxima were `9.095e-13` m, `3.553e-15` in either gradient component, and
`1.259e-15` in normal-vector difference length. Original CPU golden tolerances
remain unchanged.

### Input and failure contract

XY positions are metres; height is metres; derivatives are `dh/dx`, `dh/dy`.
The default recipe accepts the inclusive square `[-1e6, +1e6]` on each axis,
not just the four-kilometre disk used by the demonstration. A configured positive
finite coordinate limit replaces that bound.

Configuration validation retains the existing finite/positive field and
`[1, 8]` octave checks. Intermediate noise coordinates must remain finite and in
`[-2^63, 2^63)` so conversion to signed lattice coordinates is defined. The final
height and derivatives must be finite. Extreme finite parameter combinations
can exceed these arithmetic limits; a positive wavelength alone is not a promise
that all combinations are representable. FP32 fields have the usual narrower
finite range and underflow behaviour; nonfinite output remains status 2.
Finite output alone does not promise a fixed absolute error for ill-conditioned
recipes. The CPU configuration and double-query range are unchanged.

CPU queries retain `invalid_argument` for nonfinite coordinates/configuration
and unequal batch lengths, and `out_of_range` for coordinates beyond the configured
limit. Unsupported evaluation arithmetic throws `runtime_error`.

The shader result has status:

| Status | Meaning |
|---|---|
| 0 | Valid height, derivatives and material |
| 1 | Nonfinite or out-of-range input coordinate |
| 2 | Unsupported evaluation arithmetic or octave count |

Failures have zero height/derivatives and material `-1`. **They are not a terrain
plane or a successful contact.** Consumers must check status. Recipes are trusted
CPU-prepared inputs; arbitrary forged buffers are not a second configuration API.

### Layout and resource bounds

All layout fields are declared once in `baseline_types.hlsli`, with C++ size,
alignment, offset, and trivially-copyable assertions.

| Record | Bytes | Layout |
|---|---:|---|
| `BaselineBand` | 56 | Six doubles, `uint32` seed, `int32` octave count |
| `BaselineRecipe` | 488 | Eight bands, four doubles, material `int32`, reserved zero `uint32` |
| Query `double2` | 16 | X then Y |
| `BaselineResult` | 32 | Height, DX, DY doubles; material `int32`, status `uint32` |

Band order is regional base, region selector, region uplift, plains, hills,
mountains, warp X, warp Y. Recipe and band alignment is 8 bytes. Reserved bytes
are zero-initialized during preparation.

Each query evaluates at most 64 noise octaves, each with eight lattice corners.
Time is O(sum of octave counts); auxiliary storage is constant and contains no
input-sized arrays or recursion. CPU batches are O(N) with caller-owned storage.

The native hardware proof helper limits a synchronous batch to **4096** queries,
rejecting larger inputs before narrowing or allocation; empty batches allocate
nothing and dispatch nothing. It owns three default-heap resources
(`488 + 48*N` logical bytes), job-fence-owned upload/readback slices
(`488 + 48*N` bytes), two timestamp-query slots and 16 readback bytes, and a `32*N`-byte returned CPU vector. D3D12 heap alignment
and the shared fixture's bounded transfer-pool blocks add allocation overhead.
Results are copied before resources and slices are released; only one batch is
in flight. Production consumers own their own bounded scheduling policy.

## Validation

The component tests are imported by `projects\tests\unittests\unittests.vcxproj`.
`TerrainLandscapeTests` covers pre-refactor golden values, scalar/batch/error
behaviour, derivatives, normals, and concurrent CPU reads. `PerlinNoiseTests`
covers the shared noise generator, including periodic derivatives.
`TerrainBaselineGpuTests` compiles vertex and compute consumers through DXC and
executes/readbacks field and normal compute probes on D3D12. It also checks easing
analytically and double roots across the exponent range. The shared, bounded
test-only transport is `src/unittests/terrain_gpu_probe.h`, backed by the existing
`shared_gpu.h`; neither field test nor `baseline_probe.hlsl` includes sphere
contacts, Engine, or throughput definitions. Separately owned contact/throughput
fixtures are not part of this evaluator proof. No rendering, window, timing
campaign, or performance claim is involved.

Use VS2026 v145 Debug, suppress the runner's broad post-build Quick suite, then
pass class filters positionally:

```powershell
& $msbuild projects\rylogic\physics\physics.vcxproj /p:Configuration=Debug /p:Platform=x64 /p:RunUnitTests=false /p:RylogicGeneratePackageOnBuild=false
& $msbuild projects\tests\unittests\unittests.vcxproj /p:Configuration=Debug /p:Platform=x64 /p:RunUnitTests=false /p:RylogicGeneratePackageOnBuild=false
& projects\tests\unittests\obj\x64\Debug\unittests.exe TerrainLandscapeTests TerrainBaselineGpuTests PerlinNoiseTests
```

Each precision/compiler combination covers **62,125 query records**: five seeds
(`0`, `42`, `12648430`, `4294967295`, default `2772335137`) times five recipes
times 2485 positions. Both precisions and compiler modes give **248,500 field/normal
comparisons**; separate field/normal dispatches evaluate 497,000 terrain queries
before the small auxiliary/error tests.

Recipes are default; zero warp with eight octaves in all fields, negative hills
persistence and 0.001 ridge roundness; cancellation at `(731234.5, -642198.25)`;
a 7000 m datum shift; and changed warp amplitude/detail wavelengths/lacunarity/
ridge roundness. The exact fixed parameters are in `ProbeConfig`.

Every recipe samples a 33-by-33 grid over the full inclusive `[-1e6,+1e6]` square
and one deterministic jittered point in each of its 1024 cells. Another 240
positions bracket first/highest-octave lattice boundaries near five widely
separated coordinates (zero warp makes these detail-field boundaries).
132 positions check millimetre variation near zero, 4 km, the distant cancellation
point, and a distant domain corner. This is broad spatial sampling, not exhaustive
terrain coverage. The matrix is fixed independently of observed errors.

Negative controls require rejection of a 1.01 mm height error at a 7000 m datum,
independent DX/DY errors of `1.01e-4`, and a unit normal whose X/Y differences are
each `8e-5` but whose difference length is greater than `1e-4`. Inclusive threshold
acceptance and nonfinite rejection are checked too. Invalid coordinates,
unrepresentable lattice inputs, malformed octaves, FP32 overflow, empty batches,
and capacity errors remain explicit tests in both compiler modes.
Logs print the actual adapter, loaded DXC identity, compile flags and bytecode
sizes, recipe/seed/sample counts, failures, and all absolute maxima.

The focused runner passed 15 tests. A separate native Debug consumer was also
compiled directly with `baseline_surface.cpp` and the public CPU header, without
linking physics/compute libraries or including Engine/sphere-terrain headers.
Its source goldens and distant queries passed. The detailed native evidence and
evaluator-only file/hunk manifest are in the component's ignored
`obj/terrain-evaluator-final-report.json`. A selected-tree build is required when
assembling the independent evaluator commit from a mixed working tree.

Vertex compilation proves stage compatibility, not rendered vertex parity.
Erosion-derived features, physics contacts, and renderer integration are outside
this baseline evaluator.
