#pragma once
#include "src/forward.h"
#include "src/scene/body.h"
#include "src/scene/procedural_sky.h"
#include "src/diagnostics/diagnostics.h"
#include "src/utils/scene_loader.h"
#include "src/scene/water/water_visual.h"
#include "src/scene/scenario.h"

namespace physics_sandbox
{
	enum class EVisualMode
	{
		Normal,
		ContactPriority,
	};

	// The physics simulation scene. Owns the rigid bodies, physics engine, and all
	// simulation state. Deliberately UI-independent so it can be reused by both
	// the interactive sandbox and the headless unit test mode.
	struct Scene
	{
		struct StepProfile
		{
			double m_total_ms = 0;
			double m_gravity_ms = 0;
			double m_physics_ms = 0;
			double m_kill_zone_ms = 0;
			physics::Engine::StepProfile m_engine;
		};
		struct LoadProfile
		{
			double m_total_ms = 0;
			double m_prepare_ms = 0;
			double m_bbox_ms = 0;
			double m_shapes_ms = 0;
			double m_bodies_ms = 0;
			double m_ldraw_build_ms = 0;
			double m_ldraw_serialise_ms = 0;
			double m_ldraw_parse_ms = 0;
			double m_ldraw_assign_ms = 0;
			double m_logging_ms = 0;
			int m_body_count = 0;
			int m_shape_count = 0;
			int m_ldraw_object_count = 0;
			size_t m_ldraw_byte_count = 0;
			bool m_has_renderer = false;
		};

		rdr12::Renderer* m_rdr;

		// Runtime shader cache shared by the physics engine's GPU compute shaders.
		::pr::compute::shader_cache::ShaderCacheFS m_shader_cache;

		// Broadphase — either brute-force (CPU) or GPU sort-and-sweep.
		// Owned via unique_ptr to allow runtime selection based on GPU availability.
		physics::Engine m_physics;
		collision::ShapeBox m_box;

		// Bodies in the scene
		std::vector<Body> m_body;

		// Storage for shapes loaded in the scene file.
		byte_data<16> m_shape_buffer;

		// Optional diagnostic buoyancy module and the RAII hull registrations owned by the loaded scene.
		std::unique_ptr<physics::GpuBuoyancy> m_gpu_buoyancy;
		std::vector<physics::GpuBuoyancy::Registration> m_buoyancy_hulls;
		std::vector<int> m_buoyancy_body_indices;
		int m_buoyancy_generation;

		// When set, BuildBuoyancyDebugGfx() is run each frame and the resulting sample-cloud/force
		// overlay is drawn. Toggled by the sandbox UI ('B' key).
		bool m_show_buoyancy_debug;

		// LDraw overlay produced by BuildBuoyancyDebugGfx() (sample points coloured by classification,
		// surface normals, and per-primitive/total force + torque arrows). Rebuilt each frame while
		// m_show_buoyancy_debug is set; null otherwise.
		rdr12::ldraw::LdrObjectPtr m_buoyancy_debug_gfx;

		// Gravity acceleration vector (direction and magnitude, e.g. [0, -9.81, 0]).
		// Applied each step to all non-static bodies as F = m * g.
		v4 m_gravity;

		// Height below which bodies are frozen (zero velocity/momentum).
		// Prevents bodies that escape through the ground from falling to -infinity
		// and accumulating extreme float values that corrupt the simulation.
		float m_kill_zone_height;

		// Number of physics updates to run for each scene update.
		int m_physics_substeps;

		// Whether the engine is allowed to put low-energy bodies to sleep.
		bool m_allow_sleeping;

		// Ground plane visual. This is an LDraw object rendered as a large textured
		// quad. The physics ground is a static body in m_body[] with a thin box shape.
		rdr12::ldraw::LdrObjectPtr m_ground_gfx;

		// Water surface visual described by the loaded scene.
		std::optional<scene_loader::WaterDesc> m_water;
		std::unique_ptr<WaterVisual> m_water_gfx;

		// Procedural environment map cube + sky-dome model. Created alongside the water visual so
		// reflective surfaces have something to reflect; null when no water is present.
		rdr12::TextureCubePtr m_env_map;
		rdr12::ldraw::LdrObjectPtr m_sky_gfx;

		// Origin coordinate frame visual
		rdr12::ldraw::LdrObjectPtr m_origin_gfx;

		// Contact point and normal visualization overlay
		rdr12::ldraw::LdrObjectPtr m_contacts_gfx;

		// Active per-body visualisation mode.
		EVisualMode m_visual_mode;

		// Collision-readback subscription used by contact-priority visualisation and two-body diagnostics.
		pr::multicast::Sub m_collision_sub;

		// Whether to display contact points and collision normals
		bool m_show_contacts;

		// Simulation state
		double m_clock;
		bool m_step_pending;
		double m_pending_elapsed_seconds;
		int m_pending_substeps;
		StepProfile m_pending_step_profile;

		// The currently active scenario.
		EScenario m_current_scenario;

		// Diagnostics
		CollisionDiag m_diag;
		StepProfile m_last_step_profile;
		LoadProfile m_last_load_profile;
		int m_step_count;

		explicit Scene(rdr12::Renderer* rdr);

		// Reset the simulation to the current scenario's initial conditions
		void Reset();

		// Advance the simulation by one time step.
		// Returns true if a collision occurred during this step.
		bool Step(double elapsed_seconds);

		// Submit the first physics substep without waiting for its GPU results.
		void BeginStep(double elapsed_seconds);

		// Complete a submitted step, including any remaining dependent substeps.
		// Returns true if a collision occurred during this step.
		bool CompleteStep();

		// Return true while a step has been submitted but not completed.
		bool StepPending() const;

		// Configure bodies for the current scenario
		void SetupScenario(EScenario scenario);

		// Load a scene from a JSON file.
		// Replaces the current scenario with bodies defined in the file.
		void LoadScene(scene_loader::SceneDesc scene_desc);

		// Log comprehensive collision diagnostics and analytic comparisons
		void LogCollisionDiagnostics();

		// Compare post-collision velocities to the analytic solution for 1D elastic collision
		void LogAnalyticComparison();

		// Run all scenarios in sequence without rendering, log results for each
		void RunAllTests();

		// Export the scene as LDraw script
		void Dump();

		// Create/update the graphics objects for
		void UpdateCollisionGfx(std::span<physics::RbContact const> contacts);

		// Get/set the active visualisation mode.
		EVisualMode VisualMode() const;
		void VisualMode(EVisualMode mode);

		// Get/set whether automatic sleeping is enabled in the engine.
		bool AllowSleeping() const;
		void AllowSleeping(bool allow_sleeping);

		// Create/update the contact-priority visualisation.
		void UpdateContactPriorityGfx(std::span<physics::RbContact const> contacts);
		void SetContactPriorityFallbackGfx();
		void ClearContactPriorityGfx();
		void UpdateCollisionReadback();
		bool NeedsCollisionReadback() const;

		// Prepare visual state before collision readback updates it for a completed substep.
		void PrepareStepVisuals();

		// Apply gravity and submit one physics substep.
		void BeginPhysicsSubstep(float dt, double time_s, StepProfile& profile);

		// Wait for one submitted physics substep and collect its results.
		void CompletePhysicsSubstep(StepProfile& profile);

		// Calculate the bounding box for the scene (excluding terrain)
		BBox CalculateSceneBBox(scene_loader::SceneDesc const& scene_desc) const;

		// Release all scene-owned diagnostic buoyancy resources before replacing bodies or the module.
		void ClearBuoyancy();

		// Register every dynamic scene body for buoyancy when the scene contains water.
		void ConfigureBuoyancy(scene_loader::SceneDesc const& scene_desc);

		// Create the water mesh visual described by a loaded scene.
		void CreateWaterGfx(scene_loader::WaterDesc const& water, BBox const& scene_bbox);

		// Return the latest diagnostic buoyancy records for scene-registered hulls.
		std::vector<physics::GpuBuoyancy::Diagnostics> BuoyancyDiagnostics() const;

		// Rebuild m_buoyancy_debug_gfx by running the CPU buoyancy oracle (SampleHull) over each
		// registered hull and emitting an LDraw overlay of the per-sample wet/dry/culled classifications,
		// surface normals, and per-primitive + total force/torque arrows. A debug aid only; sampled with
		// the post-step body transform and current clock so it lags the GPU pass by approximately one frame.
		void BuildBuoyancyDebugGfx();
	};
}
