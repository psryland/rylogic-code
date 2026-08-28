# Physics Step Steady-State Allocation Plan

## Goal

The successful `Engine::BeginStep`/`Engine::CompleteStep` path should perform no CPU heap allocation after its active workload has reached a stable high-water
mark. Capacity growth, first activation of an optional feature, new collision geometry, and explicit cache trimming remain allowed allocation boundaries.

This is a static audit and implementation proposal. It does not claim an allocation reduction until the measurement gates below are implemented and pass.

## Steady-State Contract

For this work, steady state means:

- the submitted rigid bodies, articulation topology, constraint slots, and collision shapes fit capacities reached during warm-up;
- no optional feature is being activated for the first time;
- the GPU upload/readback rings and GPU resources have reached their required high-water capacities;
- callers do not mutate topology or add a previously unseen shape during the measured step;
- collision, break, and failure subscribers are either absent or independently pre-warmed; and
- the step succeeds, so exception formatting and diagnostic failure paths are outside the contract.

Allocation is permitted when one of those bounds grows. The first step after growth establishes a new high-water mark; later equivalent steps must again be
allocation-free.

## Workload Model

Let:

- `B` be submitted rigid bodies;
- `A` be active articulations;
- `L` be active articulation links;
- `D` be articulation degrees of freedom;
- `S` be persistent constraint slots, including tombstones;
- `K` be coupled graph nodes, targets, and adjacency entries;
- `X` be collision-exclusion pairs and hash slots; and
- `E` be published collision, break, and failure events.

Packing and retained scratch must remain expected `O(B + A + L + D + S + K + X + E)` time and `O(B + A + L + D + S + K + X + E)` memory. Open-addressed
lookups should remain at most half full so expected lookup is constant time. Their adversarial worst case remains linear per lookup, so identity hashing must use
a well-mixed deterministic hash and tests must include clustered identity values.

## Existing High-Water Storage

These paths already retain backing storage and normally allocate only when a bound grows:

| Owner | Reused storage |
|---|---|
| `EngineBufferCache` | packed rigid dynamics, CPU contacts, sleep islands, and GPU-to-CPU sleep IDs |
| `PendingStep` | body pointers, articulation pointers, sleep rollback pointers, and articulation output ranges |
| GPU feature owners | geometrically grown D3D12 resources for constraints, articulations, contacts, and frame output |
| `GpuTransferBuffer` | fenced upload/readback pages recycled after GPU completion |
| `ShapeCache` | packed geometry retained while shapes remain live |

Stable rigid-only scenes therefore already reuse most staging memory. Two exceptions are the node-based sleep lookup and any caller-owned event work.

## Recurring Allocation Sites

The following successful-path sites allocate backing storage or hash nodes again even when input sizes are unchanged.

### Step Setup and Generic Packing

| Site | Trigger | Cause |
|---|---|---|
| `EngineBufferCache::m_sleep_cpu_to_gpu_island_id` | sleeping rigid bodies | `unordered_map::clear()` destroys and deallocates nodes; each `try_emplace()` allocates them again |
| `PendingStep::m_articulation_range_lookup` | active articulations | the persistent `unordered_map` retains buckets but reallocates every cleared node |
| `WakeCoupledConstraintArticulations` | coupled constraints | a fresh local articulation-ID `unordered_map` allocates buckets and nodes |

`PendingStep`'s vectors retain their capacity. `std::vector` objects themselves do not allocate; only creation or growth of non-empty backing storage does.

### Articulation Packing

`PackGpuArticulations`, `PackGpuArticulationProxies`, and `ValidateGpuArticulationUpload` create fresh storage for:

- all non-empty vectors in `GpuArticulationUpload`;
- duplicate-identity validation;
- child and breadth-level cursors;
- per-level counts;
- schedule and adjacency validation;
- proxy shape IDs and packed proxy bodies; and
- adjacent-link collision exclusions.

Returning an upload object by value is not itself the issue: NRVO or a move transfers ownership without copying its buffers. The issue is that a fresh upload and
fresh helper containers acquire backing storage on every invocation, then discard it at the end of the step.

### Constraint Packing

`BodyRemap`, `PackGpuConstraints`, `BuildCoupledTopology`, and `BuildGpuCollisionExclusions` create fresh storage for:

- body and articulation pointer copies, packed link offsets, and two identity maps;
- all non-empty vectors in `GpuConstraintUpload`;
- participating-articulation flags and mobility offsets;
- disjoint-set parents, roots, island counts/cursors, target lookup, and target cursors;
- exclusion candidates and the final open-addressed exclusion table.

Most coupled graph keys are already dense packed indices. Hash maps at those sites add allocations and indirection without providing sparse-key semantics.

### GPU Upload Preparation

The GPU resources use high-water capacities, but CPU preparation still creates fresh vectors:

- `GpuConstraintSolver::Upload` copies all endpoints and creates `descriptor_dirty` and `reset_runtime`;
- `GpuArticulationMidpoint::Upload` builds integration-state initialization records;
- `BuildGpuArticulationMobilityRanges` builds canonical indices and output ranges, and also invokes allocation-using upload validation;
- `GpuCoupledContactSolver::Upload` builds the all-articulation index list.

The same articulation upload may consequently be validated and allocate validation scratch more than once in one frame.

`ConstraintSet::DirtyRanges()` also returns fresh storage, but production step code does not currently call it; it is not a current frame allocation.

### Readback and Publication

`Engine::Unpack` creates fresh vectors for:

- validated articulation output views;
- constraint-break events; and
- coupled-solver failure events.

Collision contacts already use `EngineBufferCache::m_contacts_cpu` as high-water storage. Event-handler implementations supplied by callers are outside the
engine's allocation guarantee and must be measured separately.

## Proposed Ownership and APIs

### 1. Add Lazy Feature Scratch Owners

Keep generic rigid-body scratch in `EngineBufferCache`, but lazily allocate constraint and articulation scratch only when each feature is first used:

```cpp
struct ArticulationStepScratch;
struct ConstraintStepScratch;

std::unique_ptr<ArticulationStepScratch> m_articulation_scratch;
std::unique_ptr<ConstraintStepScratch> m_constraint_scratch;
```

This preserves the optional-feature rule: an engine that never submits articulations or constraints does not retain their backing storage. Scratch lives for the
engine lifetime or until an explicit trim/reset boundary, never across engines.

`ArticulationStepScratch` should own the articulation upload, pack/validation cursors, proxy shape IDs, proxy bodies, collision exclusions, initialization states,
mobility indices/ranges, validated output views, and any flat lookup slots.

`ConstraintStepScratch` should own the constraint upload, reusable body remap, coupled-topology work arrays, endpoint upload copy, dirty/reset flags, exclusion
candidates/table, and break/failure publication arrays.

Scratch needed only by one GPU subsystem should remain on that subsystem rather than becoming a broad engine cache. For example, midpoint initialization records
belong to `GpuArticulationMidpoint`, and mobility range scratch belongs to `GpuArticulationMobility`.

### 2. Fill Caller-Owned Storage In Place

Replace fresh return objects with in-place pack operations:

```cpp
void PackGpuArticulations(
	std::span<Articulation* const> articulations,
	GpuArticulationUpload& upload,
	ArticulationPackScratch& scratch);

void PackGpuArticulationProxies(
	GpuArticulationUpload& upload,
	std::span<Articulation* const> articulations,
	std::span<int const> shape_ids,
	int first_body_index,
	std::vector<GpuRigidBody>& proxies);

void PackGpuConstraints(
	ConstraintSet const& constraints,
	BodyRemap const& remap,
	std::span<GpuCollisionExclusion const> additional_exclusions,
	GpuConstraintUpload& upload,
	ConstraintPackScratch& scratch);
```

Each entry point clears logical sizes but retains capacities. It must fully reset scalar counts, source pointers, and revision values so stale state cannot leak
between a large frame, a smaller frame, and an empty frame.

Unit-test convenience overloads may continue returning values by constructing local scratch, but production must use the in-place path.

### 3. Replace Transient Node Maps

Use dense arrays where keys are packed indices:

- coupled graph nodes: separate rigid-body and articulation index arrays;
- roots to islands: one array indexed by disjoint-set root;
- rigid and articulation targets: arrays indexed by packed body or mobility index; and
- articulation participation and mobility offsets: retained arrays reset to sentinel values.

For sparse stable identities, add one private reusable open-addressed `ScratchIndexMap<Key, Value>` with contiguous slots, a power-of-two capacity, a maximum
load factor of `0.5`, and generation stamps for constant-time logical clearing. Use it for body IDs, articulation IDs, pending articulation ranges, and persistent
sleep-island IDs.

Generation rollover must perform one explicit full slot clear outside ordinary operation. Insertion must detect duplicate keys and capacity overflow rather than
loop indefinitely.

### 4. Reuse Upload and Readback Vectors

- Let `GpuConstraintSolver` retain its mutable endpoint upload copy and byte-valued dirty/reset arrays. Avoid `vector<bool>` so storage and instrumentation remain
  straightforward.
- Let `GpuArticulationMidpoint` populate retained integration-state records.
- Let `GpuArticulationMobility` fill retained canonical-index and range vectors rather than move-assigning a newly allocated result.
- Let `GpuCoupledContactSolver` retain its canonical all-articulation indices.
- Let `Engine::Unpack` reuse articulation output views and event vectors, clearing them only after publication completes.

Event spans remain valid only for the synchronous callback. No caller may retain them after the callback returns.

### 5. Write Proxies Directly to Their Destination

Resize `m_rb_dynamics` once to the required rigid-plus-link count and write articulation proxies into its tail. This removes the proxy intermediate and its second
linear copy while preserving the existing packed-body ABI.

### 6. Defer Topology Caching Until Reuse Is Proven

The first implementation should still rebuild topology in linear work, but into retained storage. This is the lowest-risk way to remove allocations without
changing invalidation semantics.

After allocation gates pass, profile whether repeated packing remains material. If it does, split:

- articulation topology/inertia/axis/level streams from root transform, positions, velocities, forces, and accelerations; and
- constraint descriptors/coupled graph structure from current packed endpoints and runtime enable/break state.

Cache invariant streams only behind explicit topology and parameter revisions plus a packed-layout signature. Do not infer invariance from equal counts or object
pointers.

## Capacity and Release Policy

- Retain each backing allocation at its observed high-water capacity.
- Grow geometrically so gradual workload growth does not repeatedly allocate.
- Never shrink automatically inside `BeginStep`, `CompleteStep`, or a callback.
- Add an explicit idle-only `Engine::TrimCaches()` or equivalent scene-transition operation that releases CPU scratch and optional GPU resources.
- Reject trimming while a step is pending.
- `Reset` should define whether it preserves capacity for reuse or delegates to the trim operation; tests must lock down that contract.
- Report logical and allocated scratch bytes alongside existing feature-resource statistics so exceptional spikes are observable.

The retained CPU memory remains linear in peak workload size. GPU resource sizes, command submission count, wait count, and readback count do not change.

## Incremental Roadmap

1. **Instrumentation:** add a Debug-only, thread-scoped CRT allocation counter and a Release ETW heap-stack check. Establish per-scenario allocation baselines before
   changing ownership.
2. **Generic maps:** replace sleep and pending-articulation node maps with reusable flat lookups.
3. **Articulation pack:** introduce lazy articulation scratch, in-place upload packing/validation, and direct proxy output.
4. **Constraint pack:** introduce reusable `BodyRemap`, in-place constraint upload, dense coupled topology, and reusable exclusion construction.
5. **GPU preparation:** retain endpoint, flag, midpoint-state, mobility, and contact-participation CPU vectors in their owning GPU subsystems.
6. **Readback:** retain validated output and publication vectors while preserving the all-or-nothing commit boundary.
7. **Optional caching:** profile and add revision-keyed invariant stream caching only if pack time remains material.
8. **Retention controls:** expose scratch statistics and an explicit idle trim operation.

Each milestone should be committed independently and retain the established one-submission/one-wait/one-readback frame transaction.

## Acceptance Gates

### Allocation Correctness

Run each scenario for at least 120 warm-up frames, then measure at least 600 stable frames:

1. rigid bodies with no constraints;
2. sleeping rigid-body islands;
3. rigid persistent constraints;
4. pure Featherstone articulations;
5. rigid-to-articulation coupled constraints;
6. shaped articulation contacts; and
7. the comprehensive JSON feature showcase.

For each scenario:

- a Debug thread-scoped CRT hook reports zero allocations during successful physics steps after warm-up;
- a Release ETW heap trace reports no allocation stack owned by the physics step;
- capacities and allocated feature bytes remain constant throughout the measured window;
- increasing one bound may allocate on the growth frame, then returns to zero allocations;
- shrinking logical counts does not invalidate stale ranges or publish stale events; and
- explicit trimming releases retained optional storage and cannot run while a step is pending.

Measure event-free engine behavior separately from caller event subscribers. Add a second event-enabled run with pre-warmed non-allocating subscribers to verify
the engine-owned publication path.

### Correctness and Performance

- all native physics and constraint tests pass in Debug;
- conservation, Coriolis, Dzhanibekov, motor, limit, breakage, coupled-contact, rollback, and pathological-topology gates remain unchanged;
- every established Release sandbox benchmark remains within its existing regression tolerance;
- every new Release demo, including the comprehensive showcase, sustains at least 40 FPS; and
- every measured frame reports exactly one GPU submission, one wait, and one readback copy.

