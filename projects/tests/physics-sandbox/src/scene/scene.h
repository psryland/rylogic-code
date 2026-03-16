#pragma once
#include "src/forward.h"
#include "src/scene/body.h"
#include "src/diagnostics/diagnostics.h"
#include "src/utils/scene_loader.h"
#include "src/scene/scenario.h"

namespace physics_sandbox
{
	// The physics simulation scene. Owns the rigid bodies, physics engine, and all
	// simulation state. Deliberately UI-independent so it can be reused by both
	// the interactive sandbox and the headless unit test mode.
	struct Scene
	{
		rdr12::Renderer* m_rdr;

		// Broadphase — either brute-force (CPU) or GPU sort-and-sweep.
		// Owned via unique_ptr to allow runtime selection based on GPU availability.
		physics::MaterialMap m_materials;
		physics::Engine m_physics;
		collision::ShapeBox m_box;

		// Bodies in the scene
		std::vector<Body> m_body;

		// Storage for shapes loaded in the scene file.
		byte_data<16> m_shape_buffer;

		// Gravity acceleration vector (direction and magnitude, e.g. [0, -9.81, 0]).
		// Applied each step to all non-static bodies as F = m * g.
		v4 m_gravity;

		// Height below which bodies are frozen (zero velocity/momentum).
		// Prevents bodies that escape through the ground from falling to -infinity
		// and accumulating extreme float values that corrupt the simulation.
		float m_kill_zone_height;

		// Ground plane visual. This is an LDraw object rendered as a large textured
		// quad. The physics ground is a static body in m_body[] with a thin box shape.
		rdr12::ldraw::LdrObjectPtr m_ground_gfx;

		// Origin coordinate frame visual
		rdr12::ldraw::LdrObjectPtr m_origin_gfx;

		// Simulation state
		double m_clock;

		// Diagnostics
		CollisionDiag m_diag;
		EScenario m_current_scenario;

		explicit Scene(rdr12::Renderer* rdr);

		// Reset the simulation to the current scenario's initial conditions
		void Reset();

		// Advance the simulation by one time step.
		// Returns true if a collision occurred during this step.
		bool Step(double elapsed_seconds);

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

		// Calculate the bounding box for the scene (excluding terrain)
		BBox CalculateSceneBBox(scene_loader::SceneDesc const& scene_desc) const;
	};
}
