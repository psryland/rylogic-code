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

		Body() = default;
		Body(rdr12::Renderer* rdr, collision::Shape const* shape = nullptr, m4x4 const& o2w = m4x4::Identity(), physics::Inertia const& inertia = {});

		// Position the graphics at the rigid body location
		void UpdateGfx();

		// Add the body's graphics to a scene for rendering
		void AddToScene(rdr12::Scene& scene);
	};
}