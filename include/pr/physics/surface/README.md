# Shared surface quadrature

`surface_sampling.h` builds a primitive-local `surface::Plan` from a collision box, sphere,
triangle, or polytope. `surface_sampling.hlsli` emits its points, outward normals, and individual
area weights. **The emitter is the same source compiled as C++ and HLSL.** It has no water,
rigid-body, interior-tetrahedron, or terrain-collision dependency.

## Spacing and identity contract

`BuildPlan(shape, spacing)` requires finite positive spacing, measured in shape-local length units
(metres in physics). The default is **0.1 m**. Counts follow geometry and spacing; they are not a budget.

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
may deduplicate positions separately; this API does not implement terrain contacts.

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

Zero-area triangles and zero-radius spheres emit no weighted samples. Boxes require strictly positive
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
plans. `SamplerConfig::m_surface_spacing` controls each CPU oracle evaluation. `SampleHull` takes
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

## Validation

`BuoyancySamplerTests` covers box corner normals, face areas, balance and cell diagonals; sphere
geodesic coverage/area; triangle vertices, edge/interior coverage and first moments (including
slender/degenerate inputs); surface-only polytopes; invalid plans; analytic full/half-wet force and
torque; and bit-identical volume/lift/damping results when only surface spacing changes.

`BuoyancyCompositeHostTests` executes actual GPU shaders. Its streaming parity test exceeds 32,768
samples and compares full/partial-wet, rotated boxes, spheres, polytopes, and a transformed mixed
box/sphere/triangle compound against the CPU oracle. Existing drag, impulse-limit, articulation,
overlap, wave-pressure, and volume regressions remain part of that class.
