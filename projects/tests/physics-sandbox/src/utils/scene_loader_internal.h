#pragma once
#include "src/utils/scene_loader.h"

namespace physics_sandbox::scene_loader
{
	// Resolves an inline or named JSON shape into the common physical-body description used by links.
	using ShapeReader = std::function<BodyDesc(pr::json::Value const&)>;

	// Append articulation and persistent-constraint descriptions from the scene object.
	void AppendMultibodyDescriptions(SceneDesc& desc, pr::json::Object const& scene, ShapeReader const& read_shape);
}
