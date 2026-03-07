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
	//                 "shape": {
	//                     "type": "box",
	//                     "dimensions": [2, 2, 2]     // Full width/height/depth
	//                 },
	//                 "mass": 10.0,              // Use 0 for static (immovable) bodies
	//                 "position": [x, y, z],
	//                 "velocity": [vx, vy, vz],       // Optional, defaults to zero
	//                 "angular_velocity": [wx, wy, wz] // Optional, defaults to zero
	//             },
	//             {
	//                 "name": "sphere1",
	//                 "shape": {
	//                     "type": "sphere",
	//                     "radius": 1.0
	//                 },
	//                 "mass": 5.0,
	//                 "position": [0, 0, 0],
	//                 "velocity": [1, 0, 0]
	//             }
	//         ]
	//     }
	// }

	// Parsed description of a single rigid body from JSON
	struct BodyDesc
	{
		std::string name;

		// Shape: either a box (dimensions) or a sphere (radius)
		enum class EShape { Box, Sphere } shape_type;
		v4 box_dimensions;   // Full dimensions (only valid when shape_type == Box)
		float sphere_radius; // Radius (only valid when shape_type == Sphere)

		float mass;          // 0 = static (immovable) body with infinite mass
		v4 position;
		v4 velocity;
		v4 angular_velocity;
	};

	// Parsed description of a ground plane
	struct GroundPlaneDesc
	{
		float height;           // Y height of the ground surface
		std::string texture;    // Stock texture name (e.g. "#checker3")

		GroundPlaneDesc()
			: height(0.0f)
			, texture("#checker3")
		{}
	};

	// Parsed scene description
	struct SceneDesc
	{
		std::string description;

		// Gravity acceleration vector (direction and magnitude)
		v4 gravity;

		// Material properties (applied to material slot 0)
		float elasticity;
		float friction;

		// Optional ground plane (nullptr-equivalent: has_ground = false)
		bool has_ground;
		GroundPlaneDesc ground;

		std::vector<BodyDesc> bodies;
	};

	// Read a 3-element JSON array as a position vector (w=1) or direction vector (w=0).
	inline v4 ReadVec3(pr::json::Value const& arr, float w)
	{
		auto const& a = arr.to_array();
		if (a.size() < 3)
			throw std::runtime_error("Expected a 3-element array for vector");

		return v4{
			a[0].to<float>(),
			a[1].to<float>(),
			a[2].to<float>(),
			w
		};
	}

	// Parse a single body definition from a JSON object
	inline BodyDesc ReadBody(pr::json::Value const& body_json)
	{
		auto const& obj = body_json.to_object();
		auto desc = BodyDesc{};

		// Name (required)
		desc.name = obj["name"].to<std::string>();

		// Shape (required)
		auto const& shape_obj = obj["shape"].to_object();
		auto shape_type = shape_obj["type"].to<std::string>();
		if (shape_type == "box")
		{
			desc.shape_type = BodyDesc::EShape::Box;
			desc.box_dimensions = ReadVec3(shape_obj["dimensions"], 0);
		}
		else if (shape_type == "sphere")
		{
			desc.shape_type = BodyDesc::EShape::Sphere;
			desc.sphere_radius = shape_obj["radius"].to<float>();
		}
		else
		{
			throw std::runtime_error(pr::FmtS("Unknown shape type: '%s'", shape_type.c_str()));
		}

		// Mass (required)
		desc.mass = obj["mass"].to<float>();

		// Position (required)
		desc.position = ReadVec3(obj["position"], 1.0f);

		// Velocity (optional, defaults to zero)
		if (auto* vel = obj.find("velocity"))
			desc.velocity = ReadVec3(*vel, 0.0f);
		else
			desc.velocity = v4::Zero();

		// Angular velocity (optional, defaults to zero)
		if (auto* avel = obj.find("angular_velocity"))
			desc.angular_velocity = ReadVec3(*avel, 0.0f);
		else
			desc.angular_velocity = v4::Zero();

		return desc;
	}

	// Parse a scene description from a JSON file
	inline SceneDesc LoadFromFile(std::filesystem::path const& filepath)
	{
		auto doc = pr::json::Read(filepath);
		auto const& root = doc.to_object();

		// The top-level object should contain a "scene" key
		auto const& scene_obj = root["scene"].to_object();

		auto desc = SceneDesc{};

		// Description (optional)
		if (auto* d = scene_obj.find("description"))
			desc.description = d->to<std::string>();

		// Gravity (optional, defaults to zero — no gravity)
		if (auto* g = scene_obj.find("gravity"))
			desc.gravity = ReadVec3(*g, 0.0f);
		else
			desc.gravity = v4::Zero();

		// Material properties (optional, defaults to perfectly elastic + frictionless)
		desc.elasticity = 1.0f;
		desc.friction = 0.0f;
		if (auto* mat = scene_obj.find("material"))
		{
			auto const& mat_obj = mat->to_object();
			if (auto* e = mat_obj.find("elasticity"))
				desc.elasticity = e->to<float>();
			if (auto* f = mat_obj.find("friction"))
				desc.friction = f->to<float>();
		}

		// Ground plane (optional)
		desc.has_ground = false;
		if (auto* gp = scene_obj.find("ground_plane"))
		{
			desc.has_ground = true;
			auto const& gp_obj = gp->to_object();
			if (auto* h = gp_obj.find("height"))
				desc.ground.height = h->to<float>();
			if (auto* t = gp_obj.find("texture"))
				desc.ground.texture = t->to<std::string>();
		}

		// Bodies (required, must be an array)
		auto const& bodies_arr = scene_obj["bodies"].to_array();
		for (auto const& body_val : bodies_arr)
			desc.bodies.push_back(ReadBody(body_val));

		return desc;
	}
}
