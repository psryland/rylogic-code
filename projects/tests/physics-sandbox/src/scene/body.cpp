#include "pr/physics/utility/ldraw.h"
#include "src/scene/body.h"

using namespace pr::ldraw;

namespace physics_sandbox
{
	namespace
	{
		Colour32 NextBodyColour()
		{
			static std::default_random_engine rng;
			return RandomRGB(rng, 0.7f, 1.0f);
		}
	}

	Body::Body(rdr12::Renderer* rdr, collision::Shape const* shape, m4x4 const& o2w, physics::Inertia const& inertia)
		: physics::RigidBody(shape, o2w, inertia)
		, m_gfx()
		, m_colour(NextBodyColour())
	{
		// Rebuild graphics whenever the collision shape changes.
		ShapeChange += [rdr](RigidBody& sender, auto args)
		{
			// Note:
			//  - The handler uses the sender reference (not a captured 'this') so that
			//    it remains valid after the Body is moved by std::vector reallocation.
			//  - 'rdr' can be null when running in headless mode (i.e. unit tests)
			auto& self = static_cast<Body&>(sender);
			if (args.after())
			{
				// Create new graphics from the physics shape
				self.m_gfx = nullptr;
				if (self.HasShape() && rdr != nullptr)
				{
					Builder builder;
					builder.Add<LdrRigidBody>("Body", self.m_colour.argb).rigid_body(self);
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
		if (m_gfx)
		{
			m_gfx->O2W(m_o2w);

			// Make sleeping bodies semi-transparent for debugging
			bool sleeping = Sleeping();
			if (sleeping != m_was_sleeping)
			{
				m_was_sleeping = sleeping;
				m_gfx->m_grp_colour = m_gfx->m_grp_colour.alpha(sleeping ? 0.5f : 1.0f);
				m_gfx->Colour(false, Colour32White, "", rdr12::ldraw::EColourOp::Multiply);
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
