//************************************
// Lost at Sea
//  Copyright (c) Rylogic Ltd 2025
//************************************
#include "src/forward.h"
#include "src/world/ship/ship.h"
#include "src/world/ocean/ocean.h"
#include "src/world/terrain/height_field.h"
#include "pr/physics-2/integrator/integrator.h"
#include "pr/physics-2/integrator/contact.h"
#include "pr/physics-2/integrator/impulse.h"

namespace las
{
	// Gravity acceleration (m/s²)
	constexpr float Gravity = -9.81f;

	Ship::Ship(Renderer& rdr, HeightField const&, v4 location)
		:m_col_shape(v4{1, 1, 1, 0})
		,m_body(&m_col_shape, m4x4::Identity(), Inertia::Box(v4{0.5f, 0.5f, 0.5f, 0}, 100.0f)) // Rigid body: mass = 100kg, box inertia with half-extents of 0.5
		,m_inst()
	{
		// Create a simple box model for visualisation
		ResourceFactory factory(rdr);
		auto opts = ModelGenerator::CreateOptions{}.bake(m4x4::Identity());
		m_inst.m_model = ModelGenerator::Box(factory, 0.5f, &opts);
		factory.FlushToGpu(EGpuFlush::Block);

		// Spawn above the ocean surface at the requested XY location
		location.z = 5.0f;
		m_body.O2W(m4x4::Translation(location));
		m_inst.m_i2w = m_body.O2W();
	}

	void Ship::Step(float dt, Ocean const& ocean, HeightField const& height_field, float sim_time)
	{
		auto mass = m_body.Mass();
		auto o2w = m_body.O2W();
		auto w2o = InvertAffine(o2w);

		// Apply gravity
		m_body.ApplyForceWS(v4{0, 0, Gravity * mass, 0}, v4::Zero());

		// Box corners in object space (1x1x1 box, half-extent 0.5)
		constexpr float H = 0.5f;
		constexpr v4 os_corners[] = {
			{-H, -H, -H, 1}, {+H, -H, -H, 1}, {-H, +H, -H, 1}, {+H, +H, -H, 1},
			{-H, -H, +H, 1}, {+H, -H, +H, 1}, {-H, +H, +H, 1}, {+H, +H, +H, 1},
		};

		// Static ocean body (infinite mass, zero velocity)
		auto ocean_body = RigidBody{nullptr, m4x4::Identity(), Inertia::Infinite()};

		// Ocean collision: test each corner against the ocean surface
		auto max_pen = 0.0f;
		auto max_correction = v4::Zero();
		auto submerged_count = 0;
		for (auto const& os_corner : os_corners)
		{
			auto ws_corner = o2w * os_corner;
			auto surface_z = ocean.HeightAt(ws_corner.x, ws_corner.y, sim_time);
			auto penetration = surface_z - ws_corner.z;
			if (penetration <= 0) continue;

			++submerged_count;

			// Ocean surface normal at this corner
			auto normal_ws = ocean.NormalAt(ws_corner.x, ws_corner.y, sim_time);

			// Build contact: ship (objA) vs ocean (objB)
			auto contact = physics::Contact{m_body, ocean_body};

			// Normal from ship to ocean, in ship object space
			auto os_normal = w2o * (-normal_ws);
			os_normal.w = 0;

			// Contact point is the corner as offset from ship origin (w=0)
			auto os_contact = v4{os_corner.x, os_corner.y, os_corner.z, 0};

			contact.m_axis = os_normal;
			contact.m_point = os_contact;
			contact.m_point_at_t = os_contact;
			contact.m_depth = penetration;
			contact.m_mat = physics::Material{
				physics::Material::DefaultID,
				/*friction_static=*/ 0.3f,
				/*elasticity_norm=*/ 0.3f,
				/*elasticity_tang=*/ 0.0f,
				/*elasticity_tors=*/ 0.0f,
				/*density=*/ 1025.0f,
			};

			// Apply impulse if approaching the surface
			auto rel_vel = contact.m_velocity.LinAt(contact.m_point_at_t);
			auto approach = Dot(rel_vel, contact.m_axis);
			if (approach < 0)
			{
				auto impulse = physics::RestitutionImpulse(contact);
				m_body.MomentumOS(m_body.MomentumOS() + impulse.m_os_impulse_objA);
			}

			// Track the deepest penetration for position correction
			if (penetration > max_pen)
			{
				max_pen = penetration;
				max_correction = normal_ws * penetration;
			}
		}

		// Position correction: push ship out of the ocean along the deepest normal
		if (max_pen > 0)
		{
			o2w = m_body.O2W();
			o2w.pos += max_correction;
			m_body.O2W(o2w);
		}

		// Water drag when submerged to damp oscillation
		if (submerged_count > 0)
		{
			auto submersion = static_cast<float>(submerged_count) / 8.0f;
			auto velocity = m_body.VelocityWS();
			m_body.ApplyForceWS(-150.0f * submersion * velocity.lin, -40.0f * submersion * velocity.ang);
		}

		// Terrain collision (safety net)
		{
			auto ws_pos = m_body.O2W().pos;
			auto terrain_z = height_field.HeightAt(ws_pos.x, ws_pos.y);
			auto bottom_z = ws_pos.z - H;
			auto penetration = terrain_z - bottom_z;
			if (penetration > 0)
			{
				auto terrain_normal = height_field.NormalAt(ws_pos.x, ws_pos.y);
				auto corrected = m_body.O2W();
				corrected.pos += terrain_normal * penetration;
				m_body.O2W(corrected);

				// Kill downward velocity on terrain contact
				auto vel = m_body.VelocityWS();
				if (vel.lin.z < 0)
				{
					vel.lin.z = 0;
					m_body.VelocityWS(vel);
				}
			}
		}

		// Integrate the rigid body forward in time
		pr::physics::Evolve(m_body, dt);
	}

	void Ship::PrepareRender(v4)
	{
		// The standard forward renderer transforms vertices via m_o2s (= c2s * w2c * o2w)
		// which already handles the camera position via w2c. No manual camera-relative
		// subtraction needed — that would cause double-subtraction.
		m_inst.m_i2w = m_body.O2W();
	}

	void Ship::AddToScene(Scene& scene)
	{
		scene.AddInstance(m_inst);
	}
}
