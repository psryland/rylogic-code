//************************************
// Lost at Sea
//  Copyright (c) Rylogic Ltd 2025
//************************************
#include "src/forward.h"
#include "src/world/ship/ship.h"

namespace las
{
	// Construct the ship body and matching render instance.
	Ship::Ship(Renderer& rdr, PhysicsSystem& physics, v4 location)
		:m_physics(physics)
		,m_body_handle(PhysicsSystem::BodyHandle::Invalid())
		,m_inst()
	{
		// Create a simple box model for visualisation
		ResourceFactory factory(rdr);
		auto opts = ModelGenerator::CreateOptions{}.bake(m4x4::Identity());
		m_inst.m_model = ModelGenerator::Box(factory, 0.5f, &opts);
		factory.FlushToGpu(EGpuFlush::Block);

		// Spawn above the ocean surface at the requested XY location
		location.z = 5.0f;
		auto const o2w = m4x4::Translation(location);
		m_inst.m_i2w = o2w;

		// The ship is fully registered with the physics system before construction completes.
		m_body_handle = m_physics.CreateBoxBody(PhysicsSystem::BoxBodyDesc{
			.m_size = v4{1, 1, 1, 0},
			.m_o2w = o2w,
			.m_mass_kg = 100.0f,
			.m_never_sleep = true,
		});
	}

	// Destroy the ship's physics registration while the rigid body is still alive.
	Ship::~Ship()
	{
		if (m_physics.IsValid(m_body_handle))
		{
			m_physics.DestroyBody(m_body_handle);
		}
	}

	// Return the latest published ship transform.
	m4x4 Ship::O2W() const
	{
		return m_physics.Snapshot(m_body_handle).m_o2w;
	}

	// Access the physics registration handle for this ship's body.
	PhysicsSystem::BodyHandle Ship::PhysicsHandle() const
	{
		return m_body_handle;
	}

	// Prepare shader constant buffers for rendering.
	void Ship::PrepareRender(v4)
	{
		// The standard forward renderer transforms vertices via m_o2s (= c2s * w2c * o2w)
		// which already handles the camera position via w2c. No manual camera-relative
		// subtraction needed — that would cause double-subtraction.
		m_inst.m_i2w = O2W();
	}

	// Add the ship render instance to the scene.
	void Ship::AddToScene(Scene& scene)
	{
		scene.AddInstance(m_inst);
	}
}
