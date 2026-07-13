#pragma once
#include "src/forward.h"

namespace physics_sandbox::scene_loader
{
	// Loads a physics scene from a JSON file.
	//
	// JSON schema:
	// {
	//     "scene": {
	//         "description": "Optional description of the scene",
	//         "gravity": [0, 0, -9.81],       // Optional, defaults to [0,0,0]
	//         "seed": 1547575334,             // Optional seed used for generated content
	//         "material": {                    // Optional global material properties
	//             "elasticity": 1.0,           // Normal restitution coefficient [0,1]
	//             "friction": 0.0              // Static friction coefficient
	//         },
	//         "physics": {                      // Optional physics settings
	//             "substeps": 4,                // Run this many physics steps per scene step
	//             "solver_iterations": 8,       // Override EngineConfig contact solver iterations
	//             "position_iterations": 4,     // Override EngineConfig split-position solver iterations
	//             "contact_slop_scale": 0.01,   // Slop fraction of minimum body thickness, capped by EngineConfig slop
	//             "support_contact_slop_scale": 0.005, // Slop fraction for load-bearing support contacts
	//             "warm_start_scale": 0.90,     // Previous-frame impulse fraction used to seed the solver
	//             "max_collision_pairs": 131072,// Override EngineConfig collision pair/contact capacity
	//             "selective_refresh_passes": 1, // Extra narrowphase/resolve passes over problematic contacts
	//             "selective_refresh_body_limit": 256, // Disable selective refresh above this body count; 0 = unlimited
	//             "selective_refresh_contact_limit": 512, // Disable selective refresh after dense previous-frame contact graphs; 0 = unlimited
	//             "selective_refresh_adaptive_solver_iterations": 48 // Small-scene residual solve stiffness
	//         },
	//         "buoyancy": {                    // Optional diagnostic buoyancy settings
	//             "hulls": [
	//                 { "body": "box1", "type": "box", "dimensions": [1, 1, 1] }
	//             ]
	//         },
	//         "water": {                       // Optional water surface used by buoyancy and the visual mesh
	//             "level": 0.0,                // Flat-water base height
	//             "size": [20, 20],            // Optional visual mesh size; omitted/zero means auto-size to scene
	//             "grid": [32, 32],            // Optional visual mesh cell count
	//             "colour": "0x602080FF",      // Optional ARGB water mesh colour
	//             "waves": [
	//                 { "direction": [1, 0], "wavelength": 8.0, "amplitude": 0.25, "phase_speed": 1.5 } // "period" is accepted as a wavelength alias
	//             ]
	//         },
	//         "ground_plane": {               // Optional ground plane
	//             "height": 0.0,              // Z height of the ground surface
	//             "texture": "#checker3"      // Stock texture name (optional)
	//         },
	//         "camera": {                     // Optional camera override
	//             "position": [0, -10, 5],
	//             "lookat": [0, 0, 0]
	//         },
	//         "shapes": {
	//             "shape0": { "name": "unit-box", "type": "box", "dimensions": [1, 1, 1] },
	//             "shape1": { "name": "ball", "type": "sphere", "radius": 1.0 }
	//         },
	//         "bodies": [
	//             {
	//                 "name": "box1",
	//                 "shape": "unit-box",                 // Shape name or inline shape object
	//                 "mass": 10.0, "position": [x, y, z],
	//                 "rotation": [rx, ry, rz],             // Optional, Euler angles in degrees (X, Y, Z order)
	//                 "velocity": [vx, vy, vz],             // Optional
	//                 "angular_velocity": [wx, wy, wz],     // Optional
	//                 "sleeping": true                      // Optional
	//             },
	//             { "name": "s1", "shape": { "type": "sphere", "radius": 1.0 }, ... },
	//             { "name": "l1", "shape": { "type": "line", "length": 2.0, "thickness": 0.1 }, ... },
	//             { "name": "t1", "shape": { "type": "triangle", "vertices": [[0,0,0],[1,0,0],[0,1,0]] }, ... },
	//             { "name": "p1", "shape": { "type": "polytope", "vertices": [[x,y,z], ...] }, ... }
	//         ],
	//         "body_generators": [
	//             {
	//                 "name": "heavy_box_#",                 // '#' is replaced by the generated body index
	//                 "selector": "random",                  // Optional: "random" or "linear", defaults to "random"
	//                 "instance_count": 20,                  // Optional, defaults to 1
	//                 "shape_palette_count": 8,              // Optional, defaults to min(instance_count, 16)
	//                 "colour": ["0xFF00AA00", "0xFF00FF00"],
	//                 "shape": "unit-box",                  // Shape name or inline generator shape object
	//                 "buoyancy_hull": {},                  // Optional: derive a matching hull for every generated body
	//                                                       // ("tessellation" optionally controls polytope interior tets)
	//                 "mass": [1.0, 10.0],
	//                 "position": [[-5, 0, 0], [+5, 0, 0]],
	//                 "velocity": [[0, 0, 0], [3, 3, 3]]
	//             }
	//         ]
	//     }
	// }

	// Parsed description of a single rigid body from JSON
	struct BodyDesc
	{
		// Shape: box, sphere, line, triangle, or polytope
		enum class EShape { Box, Sphere, Line, Triangle, Polytope };

		std::string name = "body";
		std::optional<Colour32> colour = {};

		EShape shape_type = EShape::Box;

		v4 box_dimensions = One<v4>();                                // Full dimensions (only valid when shape_type == Box)
		float sphere_radius = 1.0f;                                   // Radius (only valid when shape_type == Sphere)
		float line_length = 1.0f;                                     // Full length (only valid when shape_type == Line)
		float line_thickness = 0.0f;                                  // Full thickness, 0 = infinitely thin (only valid when shape_type == Line)
		v4 tri_verts[3] = { v4(1,0,0,1), v4(0,1,0,1), v4(-1,0,0,1) }; // Triangle vertices as offsets from origin (only valid when shape_type == Triangle)
		std::vector<v4> polytope_verts = {};                          // Convex hull vertices (only valid when shape_type == Polytope)

		float mass = 0;          // 0 = static (immovable) body with infinite mass
		v4 position = Origin<v4>();
		v4 rotation = Zero<v4>(); // Euler angles in degrees (X, Y, Z order = pitch, yaw, roll)
		v4 velocity = Zero<v4>();
		v4 angular_velocity = Zero<v4>();
		bool sleeping = false;
	};

	// Parsed description of a ground plane
	struct GroundPlaneDesc
	{
		v2 size = {};                            // the horizontal extent of the ground surface
		float height = 0.0;                      // Vertical height of the ground surface
		std::optional<Colour32> colour = {};     //
		std::optional<std::string> texture = {}; // Stock texture name (e.g. "#checker3")
	};

	// Parsed description of camera settings
	struct CameraDesc
	{
		v4 position = v4(0, 0, 1, 1);
		v4 lookat = Origin<v4>();
	};

	// Parsed description of a buoyancy hull. The hull geometry is specified independently of the
	// body's collision shape (mirroring the original box-only design), so the buoyancy mesh can be
	// a simplified, watertight, tessellated approximation of the visual/collision body.
	struct BuoyancyHullDesc
	{
		// Hull primitive type. The sampled-composite backend supports box, sphere, and polytope.
		enum class EShape { Box, Sphere, Polytope };

		std::string body_name;
		EShape shape_type = EShape::Box;

		v4 dimensions = One<v4>();          // Full dimensions (only valid when shape_type == Box)
		float radius = 1.0f;                // Radius (only valid when shape_type == Sphere)
		std::vector<v4> polytope_verts = {};// Convex hull vertices (only valid when shape_type == Polytope)
		int tessellation = 5;               // Interior tessellation resolution for polytope buoyancy tets (only valid when shape_type == Polytope)
	};

	// Parsed description of a sine-wave water surface and its sandbox visual mesh.
	struct WaterDesc
	{
		physics::GpuBuoyancy::WaterSurface surface;
		v2 size = v2::Zero();
		iv2 grid = iv2(32, 32);
		Colour32 colour = Colour32(0x602080FFU);
	};

	// Parsed scene description
	struct SceneDesc
	{
		std::filesystem::path filepath;
		std::string description;

		// Gravity acceleration vector (direction and magnitude)
		v4 gravity = Zero<v4>();

		// Material properties (applied to material slot 0)
		float elasticity = 1.0f;
		float friction = 0.0f;

		// Seed for generated scene content
		unsigned int seed = 0x5C3E2026u;

		// Physics settings
		int physics_substeps = 1;
		int physics_solver_iterations = physics::EngineConfig{}.solver_iterations;
		int physics_position_iterations = physics::EngineConfig{}.push_out_iterations;
		float physics_broadphase_aabb_margin = physics::EngineConfig{}.broadphase_aabb_margin;
		float physics_contact_sort_propagation_scale = physics::EngineConfig{}.contact_sort_propagation_scale;
		int physics_contact_sort_shock_iterations = physics::EngineConfig{}.contact_sort_shock_iterations;
		float physics_contact_slop_scale = physics::EngineConfig{}.contact_slop_scale;
		float physics_support_contact_slop_scale = physics::EngineConfig{}.support_contact_slop_scale;
		float physics_warm_start_scale = physics::EngineConfig{}.warm_start_scale;
		int physics_max_collision_pairs = physics::EngineConfig{}.max_collision_pairs;
		int physics_selective_refresh_passes = physics::EngineConfig{}.selective_refresh_passes;
		int physics_selective_refresh_max_pairs = physics::EngineConfig{}.selective_refresh_max_pairs;
		int physics_selective_refresh_body_limit = physics::EngineConfig{}.selective_refresh_body_limit;
		int physics_selective_refresh_contact_limit = physics::EngineConfig{}.selective_refresh_contact_limit;
		int physics_selective_refresh_solver_iterations = physics::EngineConfig{}.selective_refresh_solver_iterations;
		int physics_selective_refresh_position_iterations = physics::EngineConfig{}.selective_refresh_position_iterations;
		float physics_selective_refresh_bias_scale = physics::EngineConfig{}.selective_refresh_bias_scale;
		float physics_selective_refresh_restitution_scale = physics::EngineConfig{}.selective_refresh_restitution_scale;
		int physics_selective_refresh_adaptive_body_limit = physics::EngineConfig{}.selective_refresh_adaptive_body_limit;
		int physics_selective_refresh_adaptive_solver_iterations = physics::EngineConfig{}.selective_refresh_adaptive_solver_iterations;
		bool physics_selective_refresh_support_only = physics::EngineConfig{}.selective_refresh_support_only;
		bool physics_selective_refresh_resolve_support_only = physics::EngineConfig{}.selective_refresh_resolve_support_only;
		float physics_selective_refresh_depth_slop = physics::EngineConfig{}.selective_refresh_depth_slop;
		float physics_selective_refresh_support_depth_slop = physics::EngineConfig{}.selective_refresh_support_depth_slop;
		float physics_selective_refresh_closing_speed_slop = physics::EngineConfig{}.selective_refresh_closing_speed_slop;
		float physics_selective_refresh_support_alignment = physics::EngineConfig{}.selective_refresh_support_alignment;
		float physics_selective_refresh_aabb_margin = physics::EngineConfig{}.selective_refresh_aabb_margin;

		// Camera settings
		std::optional<CameraDesc> camera;

		// Ground plane
		std::optional<GroundPlaneDesc> ground;

		// Water surface
		std::optional<WaterDesc> water;

		// Bodies in the scene
		std::vector<BodyDesc> bodies;

		// Diagnostic generated-box buoyancy hulls.
		std::vector<BuoyancyHullDesc> buoyancy_hulls;
	};

	// Read a 3-element JSON array as a position vector (w=1) or direction vector (w=0).
	v4 ReadVec3(pr::json::Value const& arr, float w);

	// Parse a single body definition from a JSON object
	BodyDesc ReadBody(pr::json::Value const& body_json);

	// Parse a scene description from a JSON file
	SceneDesc LoadFromFile(std::filesystem::path const& filepath);
}
