#include "src/scene/articulation_visual.h"

namespace physics_sandbox
{
	// Create an unparsed visual record for one immutable link collision shape.
	ArticulationVisual::ArticulationVisual(int articulation_index, physics::LinkHandle link, collision::Shape const& shape, m4x4 const& shape_to_link, Colour32 colour)
		: m_articulation_index(articulation_index)
		, m_link(link)
		, m_shape(&shape)
		, m_shape_to_link(shape_to_link)
		, m_colour(colour)
		, m_gfx()
	{
	}

	// Synchronise the renderer object with the link's current world transform.
	void ArticulationVisual::UpdateGfx(std::span<physics::Articulation const> articulations)
	{
		if (!m_gfx)
			return;

		auto const& articulation = articulations[m_articulation_index];
		m_gfx->O2W(articulation.LinkToWorld(m_link) * m_shape_to_link);
		auto colour = m_colour;
		if (articulation.Sleeping() && colour.a > 0x30)
			colour.a = 0x30;

		m_gfx->Colour(false, colour, "");
	}

	// Add the link object when its transformed collision bounds intersect the camera frustum.
	void ArticulationVisual::AddToScene(rdr12::Scene& scene, std::span<physics::Articulation const> articulations, m4x4 const& world_to_camera, Frustum const& frustum, v2 const& clip_planes) const
	{
		if (!m_gfx)
			return;

		auto const& articulation = articulations[m_articulation_index];
		auto const shape_to_world = articulation.LinkToWorld(m_link) * m_shape_to_link * m_shape->m_s2r;
		auto const bounds_camera_space = (world_to_camera * shape_to_world) * m_shape->m_bbox;
		if (!IsWithin(frustum, bounds_camera_space.m_centre, Length(bounds_camera_space.m_radius), clip_planes))
			return;

		m_gfx->AddToScene(scene);
	}

	// Return the transformed collision-shape bounds used for camera framing.
	BBox ArticulationVisual::Bounds(std::span<physics::Articulation const> articulations) const
	{
		auto const& articulation = articulations[m_articulation_index];
		auto const shape_to_world = articulation.LinkToWorld(m_link) * m_shape_to_link * m_shape->m_s2r;
		return shape_to_world * m_shape->m_bbox;
	}
}
