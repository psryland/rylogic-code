#include "src/forward.h"
#include "src/utils/scene_loader.h"

namespace physics_sandbox::scene_loader
{
	// Read a 3-element JSON array as a position vector (w=1) or direction vector (w=0).
	v4 ReadVec3(pr::json::Value const& arr, float w)
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
	BodyDesc ReadBody(pr::json::Value const& jv_body)
	{
		BodyDesc desc;
		auto const& jbody = jv_body.to_object();

		// Name
		if (auto* jname = jbody.find("name"))
			desc.name = jname->to<std::string>();
		
		// Colour
		if (auto* jcolour = jbody.find("colour"))
			desc.colour = To<Colour32>(jcolour->to<std::string>());

		// Shape
		if (auto* jshape = jbody.find("shape"))
		{
			auto const& jshape_obj = jshape->to_object();
			auto shape_type = jshape_obj["type"].to<std::string>();
			if (shape_type == "box")
			{
				desc.shape_type = BodyDesc::EShape::Box;
				desc.box_dimensions = ReadVec3(jshape_obj["dimensions"], 0);
			}
			else if (shape_type == "sphere")
			{
				desc.shape_type = BodyDesc::EShape::Sphere;
				desc.sphere_radius = jshape_obj["radius"].to<float>();
			}
			else if (shape_type == "line")
			{
				desc.shape_type = BodyDesc::EShape::Line;
				desc.line_length = jshape_obj["length"].to<float>();

				if (auto* t = jshape_obj.find("thickness"))
					desc.line_thickness = t->to<float>();
			}
			else if (shape_type == "triangle")
			{
				desc.shape_type = BodyDesc::EShape::Triangle;

				auto const& verts = jshape_obj["vertices"].to_array();
				if (verts.size() < 3)
					throw std::runtime_error("Triangle shape requires 3 vertices");

				desc.tri_verts[0] = ReadVec3(verts[0], 0);
				desc.tri_verts[1] = ReadVec3(verts[1], 0);
				desc.tri_verts[2] = ReadVec3(verts[2], 0);
			}
			else if (shape_type == "polytope")
			{
				desc.shape_type = BodyDesc::EShape::Polytope;

				auto const& verts = jshape_obj["vertices"].to_array();
				if (verts.size() < 4)
					throw std::runtime_error("Polytope shape requires at least 4 non-coplanar vertices");

				for (auto const& v : verts)
					desc.polytope_verts.push_back(ReadVec3(v, 1.0f));
			}
			else
			{
				throw std::runtime_error(pr::FmtS("Unknown shape type: '%s'", shape_type.c_str()));
			}
		}

		// Mass
		if (auto* jmass = jbody.find("mass"))
			desc.mass = jmass->to<float>();

		// Position
		if (auto* jpos = jbody.find("position"))
			desc.position = ReadVec3(*jpos, 1.0f);

		// Velocity (optional, defaults to zero)
		if (auto* jvel = jbody.find("velocity"))
			desc.velocity = ReadVec3(*jvel, 0.0f);

		// Angular velocity (optional, defaults to zero)
		if (auto* javl = jbody.find("angular_velocity"))
			desc.angular_velocity = ReadVec3(*javl, 0.0f);

		return desc;
	}

	// Parse a ground plane definition from a JSON object
	GroundPlaneDesc ReadGroundPlane(pr::json::Value const& jgp_)
	{
		GroundPlaneDesc ground;
		auto const& jgp = jgp_.to_object();

		if (auto* h = jgp.find("height"))
			ground.height = h->to<float>();

		if (auto* c = jgp.find("colour"))
			ground.colour = To<Colour32>(c->to<std::string>());

		if (auto* t = jgp.find("texture"))
			ground.texture = t->to<std::string>();

		return ground;
	}

	// Parse a ground plane definition from a JSON object
	CameraDesc ReadCamera(pr::json::Value const& jcam)
	{
		CameraDesc camera;
		auto const& jcam_obj = jcam.to_object();

		if (auto* jposition = jcam_obj.find("position"))
			camera.position = ReadVec3(*jposition, 1.0f);

		if (auto* jlookat = jcam_obj.find("lookat"))
			camera.lookat = ReadVec3(*jlookat, 1.0f);

		return camera;
	}

	// Parse a scene description from a JSON file
	SceneDesc LoadFromFile(std::filesystem::path const& filepath)
	{
		auto doc = pr::json::Read(filepath, json::Options{.AllowComments = true, .AllowTrailingCommas = true});
		auto const& jscene = doc.to_object()["scene"].to_object();

		SceneDesc desc;
		desc.filepath = filepath;

		// Description
		if (auto* jdesc = jscene.find("description"))
			desc.description = jdesc->to<std::string>();

		// Gravity
		if (auto* jgravity = jscene.find("gravity"))
			desc.gravity = ReadVec3(*jgravity, 0.0f);

		// Material properties
		if (auto* jmat = jscene.find("material"))
		{
			if (auto* jelasticity = jmat->to_object().find("elasticity"))
				desc.elasticity = jelasticity->to<float>();

			if (auto* jfriction = jmat->to_object().find("friction"))
				desc.friction = jfriction->to<float>();
		}

		// Ground plane
		if (auto* jground = jscene.find("ground_plane"))
			desc.ground = ReadGroundPlane(*jground);

		// Bodies
		if (auto* jbodies = jscene.find("bodies"))
		{
			for (auto const& jbody : jscene["bodies"].to_array())
				desc.bodies.push_back(ReadBody(jbody));
		}

		// Camera
		if (auto* jcamera = jscene.find("camera"))
		{
			desc.camera = ReadCamera(*jcamera);
		}

		return desc;
	}
}
