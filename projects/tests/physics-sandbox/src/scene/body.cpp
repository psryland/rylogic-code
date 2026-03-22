#include "pr/physics/utility/ldraw.h"
#include "src/scene/body.h"

namespace physics_sandbox
{
	Body::Body(rdr12::Renderer* rdr, collision::Shape const* shape, m4x4 const& o2w, physics::Inertia const& inertia)
		: physics::RigidBody(shape, o2w, inertia)
		, m_gfx()
	{
		// Rebuild graphics whenever the collision shape changes.
		ShapeChange += [rdr](RigidBody& sender, auto args)
		{
			// Note:
			//  - The handler uses the sender reference (not a captured 'this') so that
			//    it remains valid after the Body is moved by std::vector reallocation.
			//  - 'rdr' can be null when running in headless mode (i.e. unit tests)
			auto& self = static_cast<Body&>(sender);
			if (args.before())
			{
				// Release the old graphics object (ref-counted, so deletion is automatic)
				self.m_gfx = nullptr;
			}
			else
			{
				// Create new graphics from the physics shape using LDraw
				if (self.HasShape() && rdr != nullptr)
				{
					using namespace pr::ldraw;
					static std::default_random_engine rng;

					Builder builder;
					builder.Add<LdrRigidBody>("Body", RandomRGB(rng, 0.0f, 1.0f).argb).rigid_body(self);
					auto result = rdr12::ldraw::Parse(*rdr, builder.ToString());
					if (!result.m_objects.empty())
						self.m_gfx = result.m_objects.front();
				}
				self.UpdateGfx();
			}
		};
	}

	// Position the graphics at the rigid body location and update sleep visualisation
	void Body::UpdateGfx()
	{
		if (!m_gfx)
			return;

		m_gfx->O2W(m_o2w);

		// Make sleeping bodies semi-transparent for debugging
		bool sleeping = Sleeping();
		if (sleeping != m_was_sleeping)
		{
			m_was_sleeping = sleeping;
			if (sleeping)
			{
				// Store original colour, then apply 50% alpha
				m_original_colour = m_gfx->Colour(false);
				auto c = m_original_colour;
				c.a = 0x80;
				m_gfx->Colour(false, c);
			}
			else
			{
				m_gfx->Colour(false, m_original_colour);
			}
		}
	}

	// Add the body's graphics to a scene for rendering this frame
	void Body::AddToScene(rdr12::Scene& scene)
	{
		if (m_gfx)
			m_gfx->AddToScene(scene);
	}
}
