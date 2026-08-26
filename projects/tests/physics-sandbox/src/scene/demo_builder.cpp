#include "pr/physics/utility/ldraw.h"
#include "src/scene/demo_builder.h"

namespace physics_sandbox
{
	namespace
	{
		// Use a visually quiet inelastic material so demonstrations expose constraint behaviour rather than contact bounce.
		constexpr auto DemoMaterial = physics::Material{
			.m_id = physics::Material::DefaultID,
			.m_friction_static = 0.6f,
			.m_elasticity_norm = 0.05f,
			.m_elasticity_tang = 0.0f,
			.m_elasticity_tors = 0.0f,
		};
	}

	// Return a constraint frame whose local X axis follows a selected world-space axis.
	m4x4 ConstraintAxisFrame(v4 axis, v4 position)
	{
		return m4x4::Transform(v4::XAxis(), axis, position.w1());
	}

	// Begin populating a scene that has already released its previous simulation objects.
	DemoBuilder::DemoBuilder(Scene& scene)
		: m_scene(scene)
		, m_buoyant_links()
	{
		m_scene.m_physics.Material(DemoMaterial);
	}

	// Return the scene being populated for deliberate initial-state adjustments.
	Scene& DemoBuilder::SceneState()
	{
		return m_scene;
	}

	// Store a box collision shape for the complete lifetime of the demonstration.
	collision::ShapeBox const& DemoBuilder::BoxShape(v4 dimensions)
	{
		return m_scene.m_owned_boxes.emplace_back(dimensions);
	}

	// Store a spherical collision shape for the complete lifetime of the demonstration.
	collision::ShapeSphere const& DemoBuilder::SphereShape(float radius)
	{
		return m_scene.m_owned_spheres.emplace_back(radius);
	}

	// Add a box-shaped rigid body and return its stable scene index.
	int DemoBuilder::AddBox(v4 dimensions, m4x4 const& object_to_world, float mass, Colour32 colour)
	{
		auto const& shape = BoxShape(dimensions);
		auto body = Body(nullptr);
		body.m_colour = colour;
		body.O2W(object_to_world);
		body.Shape(collision::shape_cast(&shape), mass < 0.0f ? physics::Inertia::Infinite() : physics::Inertia::Box(shape.m_radius, mass));
		m_scene.m_body.push_back(std::move(body));
		return isize(m_scene.m_body) - 1;
	}

	// Add a spherical rigid body and return its stable scene index.
	int DemoBuilder::AddSphere(float radius, m4x4 const& object_to_world, float mass, Colour32 colour)
	{
		auto const& shape = SphereShape(radius);
		auto body = Body(nullptr);
		body.m_colour = colour;
		body.O2W(object_to_world);
		body.Shape(collision::shape_cast(&shape), mass < 0.0f ? physics::Inertia::Infinite() : physics::Inertia::Sphere(radius, mass));
		m_scene.m_body.push_back(std::move(body));
		return isize(m_scene.m_body) - 1;
	}

	// Add a box aligned between two world-space points and return its stable scene index.
	int DemoBuilder::AddRod(v4 point_a, v4 point_b, float thickness, float mass, Colour32 colour)
	{
		auto const delta = (point_b - point_a).w0();
		auto const length = Length(delta);
		auto const centre = (0.5f * (point_a + point_b)).w1();
		auto const object_to_world = m4x4::Transform(v4::ZAxis(), delta, centre);
		return AddBox(v4{thickness, thickness, length, 0.0f}, object_to_world, mass, colour);
	}

	// Add a broad static floor without introducing a separate rendering path.
	int DemoBuilder::AddGround(float half_extent, float height)
	{
		auto const ground_index = AddBox(
			v4{2.0f * half_extent, 2.0f * half_extent, 1.0f, 0.0f},
			m4x4::Translation(0.0f, 0.0f, height - 0.5f),
			-1.0f,
			Colour32(0xFF606068U));
		m_scene.m_ground_body_index = ground_index;
		return ground_index;
	}

	// Return a shaped articulation link with matching box mass properties.
	physics::ArticulationLinkDesc DemoBuilder::BoxLink(v4 dimensions, float mass, bool collide_parent, bool collide_self)
	{
		auto const& shape = BoxShape(dimensions);
		return physics::ArticulationLinkDesc{
			.m_inertia = physics::Inertia::Box(shape.m_radius, mass),
			.m_shape = collision::shape_cast(&shape),
			.m_shape_to_link = m4x4::Identity(),
			.m_collide_parent = collide_parent,
			.m_collide_self = collide_self,
		};
	}

	// Return a shaped articulation link with matching spherical mass properties.
	physics::ArticulationLinkDesc DemoBuilder::SphereLink(float radius, float mass, bool collide_parent, bool collide_self)
	{
		auto const& shape = SphereShape(radius);
		return physics::ArticulationLinkDesc{
			.m_inertia = physics::Inertia::Sphere(radius, mass),
			.m_shape = collision::shape_cast(&shape),
			.m_shape_to_link = m4x4::Identity(),
			.m_collide_parent = collide_parent,
			.m_collide_self = collide_self,
		};
	}

	// Transfer one completed reduced-coordinate tree into scene ownership.
	int DemoBuilder::AddArticulation(physics::Articulation articulation)
	{
		m_scene.m_articulation.push_back(std::move(articulation));
		return isize(m_scene.m_articulation) - 1;
	}

	// Return one world endpoint frame expressed directly in world coordinates.
	physics::BodyFrame DemoBuilder::WorldFrame(m4x4 const& constraint_to_world) const
	{
		return physics::BodyFrame{
			.m_body = physics::BodyRef::World(),
			.m_constraint_to_body = constraint_to_world,
		};
	}

	// Return one rigid-body endpoint frame corresponding to a shared world frame.
	physics::BodyFrame DemoBuilder::BodyFrame(int body_index, m4x4 const& constraint_to_world) const
	{
		auto const& body = m_scene.m_body[body_index];
		return physics::BodyFrame{
			.m_body = physics::BodyRef::Rigid(body),
			.m_constraint_to_body = InvertOrthonormal(body.O2W()) * constraint_to_world,
		};
	}

	// Return one articulation-link endpoint frame corresponding to a shared world frame.
	physics::BodyFrame DemoBuilder::LinkFrame(int articulation_index, physics::LinkHandle link, m4x4 const& constraint_to_world) const
	{
		auto const& articulation = m_scene.m_articulation[articulation_index];
		return physics::BodyFrame{
			.m_body = physics::BodyRef::Link(articulation, link),
			.m_constraint_to_body = InvertOrthonormal(articulation.LinkToWorld(link)) * constraint_to_world,
		};
	}

	// Add a ball-and-socket constraint between two prepared endpoint frames.
	physics::ConstraintHandle DemoBuilder::AddBall(physics::BodyFrame frame_a, physics::BodyFrame frame_b, bool collide_connected)
	{
		return m_scene.m_constraints.Add(physics::BallSocketConstraintDesc{
			.m_frame_a = frame_a,
			.m_frame_b = frame_b,
			.m_collide_connected = collide_connected,
		});
	}

	// Add a hinge constraint whose free, limited, or driven axis is frame-local X.
	physics::ConstraintHandle DemoBuilder::AddHinge(physics::BodyFrame frame_a, physics::BodyFrame frame_b, physics::ConstraintAxisDesc const& axis, bool collide_connected)
	{
		return m_scene.m_constraints.Add(physics::HingeConstraintDesc{
			.m_frame_a = frame_a,
			.m_frame_b = frame_b,
			.m_axis = axis,
			.m_collide_connected = collide_connected,
		});
	}

	// Add a slider constraint whose free, limited, or driven axis is frame-local X.
	physics::ConstraintHandle DemoBuilder::AddSlider(physics::BodyFrame frame_a, physics::BodyFrame frame_b, physics::ConstraintAxisDesc const& axis, bool collide_connected)
	{
		return m_scene.m_constraints.Add(physics::SliderConstraintDesc{
			.m_frame_a = frame_a,
			.m_frame_b = frame_b,
			.m_axis = axis,
			.m_collide_connected = collide_connected,
		});
	}

	// Add a fully locked relative transform between two prepared endpoint frames.
	physics::ConstraintHandle DemoBuilder::AddWeld(physics::BodyFrame frame_a, physics::BodyFrame frame_b, float break_force)
	{
		return m_scene.m_constraints.Add(physics::WeldConstraintDesc{
			.m_frame_a = frame_a,
			.m_frame_b = frame_b,
			.m_break_force = break_force,
		});
	}

	// Configure one angular row without duplicating reduced-coordinate locked axes.
	physics::ConstraintHandle DemoBuilder::AddAngularControl(physics::BodyFrame frame_a, physics::BodyFrame frame_b, physics::ConstraintAxisDesc const& axis)
	{
		auto desc = physics::D6ConstraintDesc{
			.m_frame_a = frame_a,
			.m_frame_b = frame_b,
		};
		desc.m_angular[0] = axis;
		return m_scene.m_constraints.Add(desc);
	}

	// Configure one linear row without duplicating reduced-coordinate locked axes.
	physics::ConstraintHandle DemoBuilder::AddLinearControl(physics::BodyFrame frame_a, physics::BodyFrame frame_b, physics::ConstraintAxisDesc const& axis)
	{
		auto desc = physics::D6ConstraintDesc{
			.m_frame_a = frame_a,
			.m_frame_b = frame_b,
		};
		desc.m_linear[0] = axis;
		return m_scene.m_constraints.Add(desc);
	}

	// Set the world gravity sampled by every rigid body and articulation link.
	void DemoBuilder::Gravity(v4 gravity)
	{
		m_scene.m_gravity = gravity.w0();
	}

	// Set the number of GPU-resident substeps recorded into each single submitted frame.
	void DemoBuilder::Substeps(int substep_count)
	{
		if (substep_count < 1)
			throw std::runtime_error("A physics demonstration requires at least one substep");

		m_scene.m_physics_substeps = substep_count;
	}

	// Configure a visible water surface for deferred articulation-link buoyancy registration.
	void DemoBuilder::Water(float level, v2 size)
	{
		m_scene.m_water = scene_loader::WaterDesc{
			.surface = physics::GpuBuoyancy::WaterSurface{
				.m_level = level,
				.m_waves = {
					physics::GpuBuoyancy::SineWave{
						.m_direction = Normalise(v2{1.0f, 0.35f}),
						.m_wavelength = 8.0f,
						.m_amplitude = 0.18f,
						.m_phase_speed = 1.0f,
					},
				},
			},
			.size = size,
			.grid = iv2{48, 48},
			.colour = Colour32(0x602080FFU),
		};
	}

	// Defer registration of one shaped articulation link with the demonstration's buoyancy module.
	void DemoBuilder::MakeBuoyant(int articulation_index, physics::LinkHandle link)
	{
		m_buoyant_links.push_back(BuoyantLink{
			.m_articulation_index = articulation_index,
			.m_link = link,
		});
	}

	// Build all rigid-body and articulation-link renderer objects in one LDraw parse.
	void DemoBuilder::BuildGraphics()
	{
		if (m_scene.m_rdr == nullptr)
			return;

		// A single parse amortises renderer setup and avoids one parser/model transaction per demo object.
		auto builder = ldraw::Builder{};
		for (auto body_index = 0; body_index != isize(m_scene.m_body); ++body_index)
		{
			auto const& body = m_scene.m_body[body_index];
			if (!body.HasShape())
				continue;

			builder.Add<ldraw::LdrCollisionShape>(std::format("DemoBody{}", body_index), body.m_colour.argb).shape(body.Shape());
		}
		for (auto visual_index = 0; visual_index != isize(m_scene.m_articulation_visuals); ++visual_index)
		{
			auto const& visual = m_scene.m_articulation_visuals[visual_index];
			builder.Add<ldraw::LdrCollisionShape>(std::format("DemoLink{}", visual_index), visual.m_colour.argb).shape(*visual.m_shape);
		}

		auto result = rdr12::ldraw::Parse(*m_scene.m_rdr, builder.ToBinary());
		auto object_index = 0;
		for (auto& body : m_scene.m_body)
		{
			if (!body.HasShape())
				continue;

			if (object_index < isize(result.m_objects))
				body.m_gfx = result.m_objects[object_index++];
			body.UpdateGfx();
		}
		for (auto& visual : m_scene.m_articulation_visuals)
		{
			if (object_index < isize(result.m_objects))
				visual.m_gfx = result.m_objects[object_index++];
		}
		m_scene.UpdateArticulationGfx();
	}

	// Create the optional articulation-only buoyancy module after all articulation addresses are stable.
	void DemoBuilder::BuildBuoyancy()
	{
		if (m_buoyant_links.empty())
			return;
		if (!m_scene.m_water)
			throw std::runtime_error("Buoyant articulation links require a configured water surface");

		m_scene.m_gpu_buoyancy = std::make_unique<physics::GpuBuoyancy>(
			m_scene.m_physics.Device(),
			m_scene.m_physics,
			physics::GpuBuoyancy::Config{});
		m_scene.m_gpu_buoyancy->SetWaterSurface(m_scene.m_water->surface);

		// Registration is deferred until vector growth is complete because each RAII handle retains its articulation address.
		m_scene.m_buoyancy_hulls.reserve(m_buoyant_links.size());
		for (auto const& buoyant_link : m_buoyant_links)
		{
			auto& articulation = m_scene.m_articulation[buoyant_link.m_articulation_index];
			auto const stable_index = 1'000'000 + isize(m_scene.m_buoyancy_hulls);
			m_scene.m_buoyancy_hulls.push_back(m_scene.m_gpu_buoyancy->RegisterCompositeHull(
				articulation,
				buoyant_link.m_link,
				stable_index,
				m_scene.m_buoyancy_generation));
		}
	}

	// Complete pointer caches, renderer objects, water resources, and sleep-island preparation.
	void DemoBuilder::Finalise()
	{
		// Derived link transforms and scene visual records are created only after topology construction is complete.
		m_scene.m_articulation_visuals.clear();
		for (auto articulation_index = 0; articulation_index != isize(m_scene.m_articulation); ++articulation_index)
		{
			auto& articulation = m_scene.m_articulation[articulation_index];
			articulation.UpdateKinematics();
			for (auto link_index = 0; link_index != articulation.LinkCount(); ++link_index)
			{
				auto const link = articulation.LinkAt(link_index);
				auto const& link_desc = articulation.LinkDescription(link);
				if (link_desc.m_shape == nullptr)
					continue;

				auto const colour_seed = static_cast<uint32_t>(articulation_index * 37 + link_index * 53);
				auto const colour = Colour32(
					96 + static_cast<int>((colour_seed * 3) % 144),
					96 + static_cast<int>((colour_seed * 5 + 47) % 144),
					96 + static_cast<int>((colour_seed * 7 + 89) % 144),
					255);
				m_scene.m_articulation_visuals.emplace_back(articulation_index, link, *link_desc.m_shape, link_desc.m_shape_to_link, colour);
			}
		}

		m_scene.RebuildStepInputs();
		BuildBuoyancy();
		BuildGraphics();
		if (m_scene.m_water)
			m_scene.CreateWaterGfx(*m_scene.m_water, BBox::Reset());
		m_scene.UpdateCollisionReadback();
	}
}
