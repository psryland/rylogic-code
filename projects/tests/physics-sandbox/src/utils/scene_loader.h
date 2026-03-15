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
	//         "material": {                    // Optional global material properties
	//             "elasticity": 1.0,           // Normal restitution coefficient [0,1]
	//             "friction": 0.0              // Static friction coefficient
	//         },
	//         "ground_plane": {               // Optional ground plane
	//             "height": 0.0,              // Z height of the ground surface
	//             "texture": "#checker3"      // Stock texture name (optional)
	//         },
	//         "bodies": [
	//             {
	//                 "name": "box1",
	//                 "shape": { "type": "box", "dimensions": [2, 2, 2] },
	//                 "mass": 10.0, "position": [x, y, z],
	//                 "velocity": [vx, vy, vz],        // Optional
	//                 "angular_velocity": [wx, wy, wz]  // Optional
	//             },
	//             { "name": "s1", "shape": { "type": "sphere", "radius": 1.0 }, ... },
	//             { "name": "l1", "shape": { "type": "line", "length": 2.0, "thickness": 0.1 }, ... },
	//             { "name": "t1", "shape": { "type": "triangle", "vertices": [[0,0,0],[1,0,0],[0,1,0]] }, ... },
	//             { "name": "p1", "shape": { "type": "polytope", "vertices": [[x,y,z], ...] }, ... }
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
		v4 velocity = Zero<v4>();
		v4 angular_velocity = Zero<v4>();
	};

	// Parsed description of a ground plane
	struct GroundPlaneDesc
	{
		float height = 0.0;                       // Y height of the ground surface
		std::optional<Colour32> colour = {};     //
		std::optional<std::string> texture = {}; // Stock texture name (e.g. "#checker3")
	};

	// Parsed description of camera settings
	struct CameraDesc
	{
		v4 position = v4(0, 0, 1, 1);
		v4 lookat = Origin<v4>();
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

		// Camera settings
		std::optional<CameraDesc> camera;

		// Ground plane
		std::optional<GroundPlaneDesc> ground;

		// Bodies in the scene
		std::vector<BodyDesc> bodies;
	};

	// Read a 3-element JSON array as a position vector (w=1) or direction vector (w=0).
	v4 ReadVec3(pr::json::Value const& arr, float w);

	// Parse a single body definition from a JSON object
	BodyDesc ReadBody(pr::json::Value const& body_json);

	// Parse a scene description from a JSON file
	SceneDesc LoadFromFile(std::filesystem::path const& filepath);
}
