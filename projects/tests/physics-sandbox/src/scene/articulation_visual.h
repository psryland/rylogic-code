#pragma once
#include "src/forward.h"

namespace physics_sandbox
{
	// Owns the renderer object associated with one scene-owned articulation link.
	struct ArticulationVisual
	{
		int m_articulation_index;
		physics::LinkHandle m_link;
		collision::Shape const* m_shape;
		m4x4 m_shape_to_link;
		Colour32 m_colour;
		rdr12::ldraw::LdrObjectPtr m_gfx;

		// Create an unparsed visual record for one immutable link collision shape.
		ArticulationVisual(int articulation_index, physics::LinkHandle link, collision::Shape const& shape, m4x4 const& shape_to_link, Colour32 colour);

		// Synchronise the renderer object with the link's current world transform.
		void UpdateGfx(std::span<physics::Articulation const> articulations);

		// Add the link object when its transformed collision bounds intersect the camera frustum.
		void AddToScene(rdr12::Scene& scene, std::span<physics::Articulation const> articulations, m4x4 const& world_to_camera, Frustum const& frustum, v2 const& clip_planes) const;

		// Return the transformed collision-shape bounds used for camera framing.
		BBox Bounds(std::span<physics::Articulation const> articulations) const;
	};
}
