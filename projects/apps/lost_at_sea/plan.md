# Lost at Sea

## Vision

A sailing game set in a procedurally generated archipelago. The core game loop is a physics simulation of a rigid body (ship) floating on a moving ocean. The player builds sailing vessels and tries to efficiently sail them through increasingly challenging conditions. Art style is low-poly/Phong (WoW-esque), not AAA.

### Inspiration
- Environment, Scenic Setting => Sid Meier's Pirates - age of pirates, wooden sailing ships
- Ship building => basic components, lego brick construction => Valheim base building, KSP rocket building
- Graphic styling => World of Warcraft => low-ish detail models, simple Phong shading

## Key Game Features
- Reasonably physically realistic sailing model. Sails catch air flow, apply forces to vessel
- Realistic buoyancy simulation. Ships displace water based on shape. Centre of buoyancy vs centre of mass affects righting moment
- Ocean water simulation (around the player). Wave amplitudes, wind direction and strength, gusts
- Varying weather causing calm and rough conditions

## Desired Gameplay Scenarios
- Build ship in "shipyard" (i.e. editor), click "Launch" to sail
- Capsize/Sink shipwreck => Ship disassembles into parts (with physics) => Restart back at last home island

---

## Design Decisions

### Platform & Rendering
- **Platform**: Win64, DirectX 12
- **Renderer**: view3d-12 as a **static library** (not DLL). No LdrObjects/ldraw script.
- **Asset pipeline**: Models loaded from FBX/glTF files or generated procedurally. Using existing `pr/geometry/gltf.h` and `pr/geometry/fbx.h` support.

### Scale
- **Real-world scale**: 1 unit = 1 metre. A human male ~1.8m tall.

### Coordinate System
- **Right-handed**: Z = up, X = forward, Y = right

### Camera Model
- Camera stays at (0, 0, cam_height) in render space
- All world objects render at `render_pos = true_pos - camera_world_pos`
- Physics runs in true world space (player doesn't travel astronomical distances)
- This keeps all rendered geometry near the origin for floating point precision

### World Model
- Infinite flat plane with Perlin noise height field
- `height(x, y)` = multi-octave Perlin/Simplex noise, seed-based for reproducibility
- Height > 0 = land, height < 0 = ocean floor, ocean surface at z = 0
- Rendered with curvature falloff to simulate a horizon - mountains peek over before full islands visible

### Ocean Surface
- Flat plane at z = 0 with animated vertex displacement (Gerstner/sine waves)
- Grid mesh centred on camera, tessellated more densely nearby
- CPU mirrors the same wave math for physics queries (`OceanHeightAt`)

### Physics
- Use the compute physics engine for the active ship rigid body.
- Buoyancy prototype is a no-readback GPU external-force pass, recorded after body upload and before integration.
- LAS owns the first buoyancy geometry and ocean data; the physics engine should stay LAS-agnostic.
- Use a simple watertight low-poly buoyancy mesh as the water-displacement proxy, separate from collision shapes.

### Technologies
- **Renderer**: `view3d-12` static lib (`projects/rylogic/view3d-12/`)
- **Collision**: `include/pr/collision/` - shapes, GJK, raycasting
- **Physics**: `include/pr/physics/` and `projects/rylogic/physics/` - GPU rigid body integrator and compute pipeline
- **Audio**: `projects/rylogic/audio/` - DirectSound, WAV, OGG streaming
- **Asset loading**: `include/pr/geometry/gltf.h`, `include/pr/geometry/fbx.h`

### Settings Format
- **JSON** (not ldraw script). Uses `pr/storage/json.h` for read/write.
- Simple struct with explicit `Load(path)` / `Save(path)` methods.
- Drop the old `SettingsBase<>` macro system and `pr::script::Reader` dependency.

### Header Inclusion Convention
- Libraries do **not** include files from outside the library, except via `forward.h`
- Each library has a `forward.h` that acts as a pseudo-precompiled header and includes all "foreign" dependencies
- All other headers within a library only include sibling headers from the same library
- This is a repo-wide convention and must be followed in all new code

---

## Phases

### Phase 0: Project Modernisation & Foundation *(Start Here)*
Get the project compiling cleanly with current libs before adding features.
Use AceInspaders as the reference for the modern app framework pattern.

- **0.1 Modernise `forward.h`**: Strip old includes (stdafx.h, d3d11.h, old event system, old script headers).
  Use modern view3d-12 headers. Follow the repo convention: forward.h is the only file that includes "foreign" headers.
- **0.2 Modernise `settings.h`**: Replace the old ldraw-script-based settings (`settings.cpp`) with JSON-based
  settings using `pr/storage/json.h`. Simple struct with `Load(path)`/`Save(path)` using JSON read/write.
  Drop `SettingsBase<>` and `pr::script::Reader` dependency.
- **0.3 Modernise `main.h`/`main.cpp`**: Follow AceInspaders pattern:
  - Use `pr::app::DefaultSetup` (or minimal custom Setup)
  - Use `pr::app::Main<Main, MainUI, Settings>` and `pr::app::MainUI<MainUI, Main, SimMsgLoop>`
  - Remove old commented-out audio/DirectSound experiments
  - Remove `skybox.h/cpp` from `src/world/` (old D3D9-era code). Use `pr::app::Skybox` from `pr/app/skybox.h` (already partially done in main.cpp)
- **0.4 Clean up stubs**: Remove old `cam.h/cpp`, `ship.h/cpp`, `terrain.h/cpp` stubs (will be rewritten for Phase 1+).
  Or keep empty placeholder files if preferred.
- **0.5 Remove menu.ldr**: Delete `data/menu/menu.ldr` (ldraw script). Menu system will be rebuilt later (or deferred).
- **0.6 Update vcxproj**: Remove references to old files (stdafx, ogg, vorbis, old camera/dinput).
  Ensure `view3d-12-static.lib` and `audio.lib` are linked.
- **0.7 Compile and run**: Verify the app launches, shows a skybox, and can be closed cleanly.

### Phase 1: Ocean & Terrain Rendering *(Current Focus)*
Render a basic world the player can look at.

- **1.1 Ocean mesh**: Grid mesh centred on camera (e.g. 256x256 vertices, ~500m x 500m). Vertex shader applies animated Gerstner wave displacement based on wind direction/speed.
- **1.2 Ocean shader**: Water material with environment map reflection (from skybox), Fresnel, foam at wave peaks. Depth-based colour (turquoise shallow, dark blue deep).
- **1.3 Height field**: Multi-octave Perlin/Simplex noise. Input = (world_x, world_y), output = height. Seed-based for reproducibility.
- **1.4 Terrain mesh**: Terrain mesh for visible land (height > 0). LOD by distance from camera. Simple height/slope-based texturing (sand, grass, rock).
- **1.5 Horizon curvature**: Vertex shader curves geometry downward with distance from camera. Mountains appear on horizon before full islands are visible.
- **1.6 Skybox integration**: Skybox matches ocean horizon line. Architecture extensible for future day/night cycle.

### Phase 2: Ocean Physics (Buoyancy)
The core technical challenge - make things float realistically.

Current design direction: prototype GPU buoyancy in LAS first, then move only the clean general pieces into the physics engine once the API shape is proven.

- **2.1 Wave consistency**: Align CPU and HLSL Gerstner phase conventions before trusting physics validation.
- **2.2 External force seam**: Add a pre-integrate `EventHandler` hook to the compute physics engine. The hook runs after body upload and before integration, receives `dt` plus absolute simulation time, and lets subscribers add to GPU body force/torque accumulators without readback.
- **2.3 LAS `GpuBuoyancy` module**: Own wave constants, body-to-buoyancy records, and a low-poly watertight buoyancy mesh. Start with a generated box mesh and a 128x128 integration grid.
- **2.4 Static buoyancy validation**: In flat water, compare GPU diagnostics for volume, force, centre of buoyancy, and torque against analytic box cases before applying damping.
- **2.5 No-readback force application**: Use a deterministic two-pass reduction so the final force and torque are written directly to `GpuRigidBody.force_*` before integration.
- **2.6 Gerstner water input**: Evaluate the same Gerstner waves in the buoyancy compute shader using the shared wave parameter layout.
- **2.7 Water damping**: Add heave/angular damping and later drag/slamming only after the static buoyancy force is trusted.
- **2.8 Test with primitives and proxy hulls**: Verify waterline, restoring torque, rocking on waves, settling to equilibrium, and capsizing.

### Phase 3: Sailing Model
Make a pre-built ship respond to wind.

- **3.1 Ship rigid body**: Simple hull shape (convex polytope or shape array). Derive mass/inertia. Register with physics engine.
- **3.2 Wind model**: Global wind direction + speed, with Perlin-based gust variation. `v4 WindAt(v4 pos, float time)` query.
- **3.3 Sail forces**: Apparent wind (true wind - ship velocity), sail angle to lift/drag coefficients (simplified aerofoil model), force at sail attachment point.
- **3.4 Rudder & steering**: Player input to rudder angle to lateral force at stern to turning moment. Keel provides lateral resistance.
- **3.5 Basic HUD**: Wind direction compass, ship speed, heading. Controls: A/D = rudder, W/S or mouse = sail trim.

### Phase 4: Player Experience & Polish
Turn the simulation into something playable.

- **4.1 Camera modes**: Chase cam (behind ship), deck cam (first person), free cam (debug). Smooth interpolation, wave-following.
- **4.2 Audio**: Ocean ambience (waves, wind), sailing sounds (creaking, splashing, flapping). Vary with speed/conditions. Use audio.dll.
- **4.3 Ship capsize/sink**: Roll past critical angle, break apart, physics debris, fade/sink, respawn at last island.
- **4.4 Island interaction**: Approach island, anchor/dock. Save point / home base.
- **4.5 Procedural island placement**: Use height field to identify islands. Place points of interest, docks.

### Phase 5: Weather & World
Environmental variety and challenge.

- **5.1 Weather system**: Time-varying wind strength and direction. Calm to breeze to gale cycle.
- **5.2 Wave conditions**: Amplitude/frequency driven by wind. Calm near islands (sheltered), rough in open sea.
- **5.3 Visual weather**: Rain particles, fog (reduced draw distance), storm lighting.
- **5.4 Difficulty gradient**: Further from start = stronger winds, bigger waves, more storms.

### Future Phases (Deferred)
- Ship building editor (Valheim/KSP style construction)
- Multiplayer
- NPC/rival ships (possibly Copilot SDK for AI reasoning)
- Day/night cycle & dynamic lighting
- Cannons/combat
- Trading/economy
- Save/load system
- Full fluid simulation (Navier-Stokes)

---

## Technical Notes

### GPU Buoyancy Algorithm (Phase 2 Detail)
Compute submerged volume with gravity-aligned columns over a LAS-owned buoyancy mesh.

1. Build a plane perpendicular to gravity over the projected buoyancy mesh bounds.
2. For each grid cell, cast a column ray through a closed low-poly triangle proxy mesh and sort the surface intersections into solid intervals.
3. Clamp each solid interval to the portion below the Gerstner water height at the column centre.
4. Accumulate displaced volume and per-column force along `-gravity`; sum torque from each column centroid relative to the body's centre of mass.
5. Reduce threadgroup partials into the GPU rigid-body force and torque accumulators before the normal physics integration pass.

### Ocean Mesh Strategy
- Grid mesh centred on camera, e.g. 256x256 verts covering ~500m x 500m
- Vertex shader displaces using sum of Gerstner waves
- CPU mirrors the same maths for physics queries
- Near camera: high tessellation. Far: low (LOD via mesh density or tessellation shader)
- Beyond mesh edge: skybox/fog hides boundary

### Height Field for Terrain
- `height(x, y) = Sum( amplitude_i * noise(frequency_i * x, frequency_i * y) )`
- Terrain generated in chunks around camera, LOD by distance
- Same height field used for collision detection (ship grounding)

---

## Other Crazy Ideas
- Integrate Copilot SDK to drive logic and reasoning for rival vessels