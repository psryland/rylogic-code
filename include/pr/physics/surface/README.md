# Shared surface quadrature

`surface_sampling.h` builds a primitive-local `surface::Plan` from a collision box, sphere,
line/capsule, triangle, or polytope. `surface_sampling.hlsli` emits its points, outward normals, and individual
area weights. **The emitter is the same source compiled as C++ and HLSL.** It has no water,
rigid-body, interior-tetrahedron, or terrain-collision dependency.

## Spacing and identity contract

`BuildPlan(shape, spacing)` requires finite positive spacing, measured in shape-local length units
(metres in physics). The shared default is **0.16 m**, used by terrain, CPU/GPU buoyancy and sandbox surface overlays.
Counts follow geometry and spacing; they are not a budget.

In exact arithmetic, every surface cell has diameter at most `spacing`, and every surface point is
within `spacing / 2` of an emitted node. Distances are planar on flat faces and **geodesic on spheres**.
Single-precision construction/emission adds ordinary coordinate-scale rounding error. Rigid
shape-to-root and root-to-world transforms preserve these bounds; non-rigid scaling is not supported.
The contract concerns primitive surfaces before application-specific wet/union filtering.

Every non-degenerate face includes its real vertices and covers its edges as well as its interior.
Coincident contributions from different faces are intentionally separate, with their own face normals
and represented areas. Do not average sharp normals before applying a nonlinear force. Artificial
triangle-strip boundaries and cube-map seams can also have coincident contributions; these represent
adjacent integration regions, not additional surface area. A future position-only contact consumer
may reduce positions separately; terrain contact selection belongs to the engine, not this emitter.

For unchanged geometry and spacing, primitive-local ordinal, position, normal, and weight are
deterministic. Surface identities do not depend on body registration IDs or frame order. Compound
callers enumerate primitive children in their existing order and apply each child's shape-to-root
transform once. Build plans directly from the primitive geometry, not from volume tetrahedra.

## Geometry and weighting

* **Boxes:** each face has a tensor-product rectangular grid. For edge lengths `a,b`,
  `nu = max(1, ceil(sqrt(2)*a/spacing))`, and likewise for `nv`. Hence the cell diagonal is at most
  spacing and rounding to the nearest grid node is within half that distance. A node receives one
  cell area in the interior, half on a boundary, and a quarter at a face corner. Per-face area and
  first moments are exact before floating-point rounding.
* **Triangles/polytope faces:** split a triangle at the altitude to its longest edge. Each resulting
  right triangle has perpendicular legs `L >= H`. Divide the long leg into
  `n = ceil(2*hypot(L,H)/spacing)` tapered strips. Strip `i` has initial height
  `hi = H*(n-i)/n` and `mi = ceil(2*hi/spacing)` subdivisions across its width. Its two columns use
  the same normal, including on all original edges. Along either grid parameter the displacement
  is at most half the requested spacing, so a cell's diameter is at most spacing and rounding both
  parameters gives the half-spacing coverage bound. Subdivision counts decrease towards the tip;
  an elongated thin face does not allocate a longest-edge-squared triangular lattice.
  Each strip integrates its bilinear nodal basis against its linearly varying width. With
  `t = 1/(n-i)` and `J = (L/n)*hi`, the column factors are `(3-t)/6` and `(3-2t)/6`, multiplied by
  the across-width trapezoid weights and `J/mi`. The collapsed tip's coincident nodes are combined
  into one weight `J/6`. This preserves area and first moments, including for slender triangles.
* **Spheres:** project the six faces of a unit cube radially onto the sphere. Each face uses
  `n = ceil(2*sqrt(2)*radius/spacing)` subdivisions in both directions. Radial projection from a
  cube face to a radius-`r` sphere has differential length at most `r` times the planar length,
  proving the geodesic bounds. The area-density variation is bounded by `3*sqrt(3)`, independent of
  resolution; there is no latitude/pole clustering. Each node owns a quarter of each incident
  spherical cell. Cell areas are exact spherical solid angles, computed from two spherical
  triangles using edge differences to avoid cancellation. Areas sum to `4*pi*r*r` and the
  construction has cube/antipodal symmetry. Non-constant integrands remain quadrature approximations.

* **Lines/capsules:** a positive radius uses two cube-mapped hemispheres and a periodic cylinder grid.
  Hemisphere patches inherit the sphere's conservative subdivision bounds. Cylinder axial and arc-length
  steps are at most `spacing/sqrt(2)`; trapezoidal axial weights and periodic angular weights sum to
  `2*pi*r*length`, and the hemispheres sum to `4*pi*r*r`. Equator contributions remain separate.
  A zero-radius line uses endpoint-inclusive subdivisions of length at most spacing, with zero area
  and zero normal. A zero-length capsule has only its hemispheres.

Zero-area triangles emit their longest segment (one point if collapsed); zero-radius spheres emit
one point. These samples have zero area and zero normal, so they add no surface force while preserving
geometric contact coverage. Boxes require strictly positive
extents, matching the collision shape's construction invariant; invalidated zero-extent boxes throw
at the sampler boundary instead of being treated as supported plates. Unsupported primitives,
negative/non-finite dimensions, invalid spacing, overflowing
geometry, indistinguishable float grid steps, and underflowing area weights throw `std::runtime_error`.
Nonzero area that cannot be represented must not be replaced with arbitrarily weighted corner samples.

## Cost, limits, and buoyancy integration

A patch is **96 bytes**, not a stored full sample cloud. Boxes and spheres retain six patches
(576 bytes); triangle faces retain `O(perimeter/spacing)` strips. Construction is linear in patch
count, with a count-validation pass before allocating a right triangle's strip records. Emission
is allocation-free and uses a binary search over primitive-local cumulative patch counts:
`O(log patch_count)` plus constant generation work. Sphere weights require trigonometric evaluation;
no speedup over another sampler is claimed.

Sample counts grow as `O(area/spacing^2 + perimeter/spacing + face_count)`. For the default:

| Primitive | Samples |
| --- | ---: |
| 1 x 1 x 1 m box | 1,536 |
| 2 x 2 x 1 m box | 3,720 |
| 2 x 2 x 2 m box | 5,400 |
| Radius 1 m sphere | 5,400 |
| 2 m cube or radius 1 m sphere at 0.035 m spacing | 40,344 |

No count is silently truncated. A grid dimension above 1,048,576, a primitive/hull population above
`0x7fff0000`, or patch-buffer bytes beyond signed 32-bit addressing is rejected explicitly.
Area-weight lower bounds must remain in the normal float range. These are representability/resource
limits, not quality controls; callers must choose a coarser spacing or smaller geometry after rejection.
Allocation/device resource failures propagate normally. Extremely large valid sample populations
can still be expensive and are not a real-time latency guarantee.

`GpuBuoyancy::Config::m_surface_spacing` is part of the immutable shape-cache key. Like polytope
derivation settings, it applies at registration/shape refresh; existing registrations retain their
plans. Both default to `surface::DefaultSpacing`, as does terrain; explicit caller overrides remain available.
`SamplerConfig::m_surface_spacing` controls each CPU oracle evaluation. `SampleHull` takes
only the volume sample count: there is no old surface-count overload or alternate Halton surface path.

GPU surface threads stream all ordinals using a grid-stride loop. At most 128 groups per hull produce
reduction partials, regardless of sample count; the limit is on **partial storage, not emission**.
Surface dispatch uses group X for the hull-local group and Y for the hull, avoiding a large flattened
X dimension. Hull-count, cumulative count, and patch-upload bounds are checked before allocation or
dispatch. Per-sample primitive selection and sibling containment retain the existing primitive walk.

Volume distribution, sample hash/indexing, tetrahedron selection, weights, lift, linear/angular
volume damping, gravity frame, wet tests, sibling ownership, and surface-drag impulse limiting are
unchanged. Surface forces still multiply each sample's `m_darea`.

Strict point wet/dry and sibling-cull tests approximate discontinuous boundaries. A waterline through
nodes or an overlapping/abutting primitive seam can therefore have spacing-dependent quadrature
error; exact partial-wet area or clipped CSG reconstruction is not promised. Refining spacing reduces
this error. Per-face area conservation alone is not a proof of integrated force or torque accuracy.

## Sampled cylindrical world boundary

`Engine::CylindricalBoundary(std::optional<CylindricalBoundaryConfig>)` installs an inward-facing cylinder centered at `m_centre_x,m_centre_y`, with
radius `m_radius` in metres and material `m_material_id`. `std::nullopt` removes only the boundary. It is unlimited in height and works with or without
`Engine::Terrain`. The managed equivalent is `Engine.SetCylindricalBoundary(CylindricalBoundaryConfiguration?)`.

`GpuWorldContacts` owns one shapeless infinite-mass endpoint and the common sampling, instance upload, contact reduction, status, and solver-append path.
All physical primitive types and compound leaves retain their existing shape-to-root transforms and material IDs; articulation proxies use the same path.
Cylinder contacts use radial inward normals and exact tangent-plane depths, not a finite box ring, sphere-only approximation, or position clamp. Contact
spread is retained in the cylinder's tangent/vertical plane. Unchanged sleeping support is skipped; disturbed islands and source replacement retain normal
wake semantics.

The shared GPU pipeline requires FP64 and `Int64ShaderOps`, including when only the boundary is configured. Shader sources are embedded in the native DLL.

Different terrain and boundary spacings have separately indexed ranges in one cached patch stream. Adding the default 0.05 m boundary does **not** change
the default **0.16 m terrain** sample count. Both sets are generated only when a shape first participates or packed shape indices are invalidated.
Interior boundary queries use a conservative XY-bound rejection before enumerating samples. Boundary history resources are absent without a boundary;
terrain timestamps measure only terrain work, not wall contacts or boundary validation.

### Discrete-motion and rejection envelope

The defaults are `m_surface_spacing=0.05 m`, `m_max_substep_motion=0.1 m`, and `m_max_penetration=0.25 m`. The last value is a **gross safety-rejection
cutoff**, not acceptable resting overlap or a spawn/restore allowance. This is the existing discrete, substepped solver, not swept/continuous collision.
Horizontal speed and pose-displacement bounds include child offsets and angular motion; vertical translation alone does not violate the motion limit.
With no rotation, 0.1 m at 1/240 s corresponds to a mathematical speed bound of 24 m/s;
leave rounding margin rather than operating exactly at that bound.

This does not exempt angular motion produced by wall friction. A 1 m, 1 kg sphere approaching at 20 m/s radial, 3 m/s tangential, and -100 m/s vertical
with material friction 0.3 and normal elasticity 0.05 exceeds the 0.1 m envelope at 240 Hz after its first wall impulse. At cardinal azimuths the resolved
angular speed is about 49.29 rad/s; an actual point on the sphere has 28.59 m/s horizontal velocity (0.11914 m per substep). At 45 degrees the corresponding
values are 45.97 rad/s and 24.66 m/s (0.10276 m per substep). These are real surface points, not bounding-box corners, so merely tightening the conservative
box estimate cannot admit this fixture. More substeps or different physical conditions need their own validation; do not raise the motion or overlap gates.
`CylindricalBoundaryFrictionTests` verifies this rejection at five azimuths and checks that caller-owned dynamics remain unchanged.

Motion failures report the first rejected body's packed index, zero-based substep, validation phase, world linear/angular velocities, conservative lever,
speed-motion and pose-displacement bounds, timestep, and configured limit. The reported bounds are conservative; they are not measurements of the maximum
physical surface-point speed.

GPU validation checks pre-integration, predicted, and resolved states. Every sampled point must have radial depth plus the configured spacing no greater
than `m_max_penetration`; the coverage margin bounds unsampled physical surface extent and float rounding. Motion/domain status and ordinary raw contact
capacity counters are read before publishing dynamics or collision events. Unsupported input is rejected; impulses and velocities are never silently clipped.
The shared sample-count and resource bounds above still apply, as does the 65,535 convex-instance dispatch limit. Eight distinct normal groups per leaf
are supported; additional groups fail explicitly.

Configuration requires finite center, radius, and positive envelope distances, a valid material ID, `2*spacing + max_substep_motion <= max_penetration`,
`max_penetration < radius*0.01`, and `max(abs(centre_x),abs(centre_y))+radius <= 1,000,000 m`. Spacing must be at least
`16*float_epsilon*(max(abs(centre_x),abs(centre_y))+radius)`. There is no Z-domain cutoff.

Startup/import placement belongs to the application and must contain the actual physical surface, not just its center. A conservative sufficient test is
that every transformed physical sample's radial distance plus spacing is at most radius. Exact shape-specific checks may admit additional valid placements.
Restoring previously completed simulation states may preserve their small, explicitly validated solver overlap; it must not substitute the gross cutoff
for an application acceptance tolerance. The DLL rejects native checkpoint capture and import whenever either world surface is configured: applications
must persist source configurations and body states explicitly.

## Validation

`CylindricalBoundaryTests` runs the actual GPU solver with a **1 cm maximum post-step physical-overlap gate**. At radius 4000 m, the Debug fixtures measured:

| Fixture | Observed maximum positive overlap |
| --- | ---: |
| 1 m sphere, 20 m/s radial approach, 1/240 s substeps | 6.104 mm |
| Transformed box/sphere compound under sustained radial acceleration | 1.758 mm |
| 0.6 m sphere, tangent sliding and initially -100 m/s vertical freefall | 1.017 mm |
| 1 m box with simultaneous terrain and wall support | None at recorded post-frame poses |

These are fixture measurements, not a universal bound for arbitrary geometry, force, solver settings, or initial overlap. The fast sphere is checked after
every substep; multi-substep fixtures are checked after each submitted frame. The independent extent oracle uses exact sphere radii and box corners, including
compound child transforms, rather than reusing collision samples. A 1 cm restored-state tolerance is supported for these tested settings; applications must
validate their actual body/force workload rather than adopt the 0.25 m gross rejection cutoff.

The suite also checks off-axis normals/depth and materials, contacts well above a visible rim, all primitive sample types, articulation links, unchanged
sleeping support and meaningful collision wakes, removal, invalid configuration, pending mutation, domain/motion rejection, and contact overflow.
Its density/cache regression retains **600 terrain samples at 0.16 m versus 5400 wall samples at 0.05 m** for a 1 m sphere, reuses unchanged plan storage,
and verifies that terrain-only operation allocates no boundary-history buffer.

`BuoyancySamplerTests` covers box corner normals, face areas, balance and cell diagonals; sphere
geodesic coverage/area; triangle vertices, edge/interior coverage and first moments (including
slender/degenerate inputs); surface-only polytopes; invalid plans; analytic full/half-wet force and
torque; and bit-identical volume/lift/damping results when only surface spacing changes.

`BuoyancyCompositeHostTests` executes actual GPU shaders. Its streaming parity test exceeds 32,768
samples and compares full/partial-wet, rotated boxes, spheres, polytopes, and a transformed mixed
box/sphere/triangle compound against the CPU oracle. Existing drag, impulse-limit, articulation,
overlap, wave-pressure, and volume regressions remain part of that class.
