#pragma once
#include "src/forward.h"

namespace physics_sandbox
{
	// A rigid body with attached View3D graphics.
	// When the collision shape changes, the graphics object is automatically
	// rebuilt from LDraw. UpdateGfx() syncs the graphics transform to the physics transform.
	struct Body : physics::RigidBody
	{
		rdr12::ldraw::LdrObjectPtr m_gfx;
		m4x4 m_gfx_o2b;
		Colour32 m_colour = Colour32White;
		Colour32 m_priority_colour = Colour32White;
		Colour32 m_applied_colour = Colour32Black;
		bool m_priority_colour_enabled = false;

		Body() = default;
		Body(rdr12::Renderer* rdr, collision::Shape const* shape = nullptr, m4x4 const& o2w = m4x4::Identity(), physics::Inertia const& inertia = {});

		// Position the graphics at the rigid body location
		void UpdateGfx();

		// Set/clear the contact-priority visual override colour.
		void PriorityColour(Colour32 colour, bool enabled);

		// Add the body's graphics to a scene for rendering
		void AddToScene(rdr12::Scene& scene);
		void AddToScene(rdr12::Scene& scene, m4x4 const& w2c, Frustum const& frustum, v2 const& clip_planes);
	};
}
