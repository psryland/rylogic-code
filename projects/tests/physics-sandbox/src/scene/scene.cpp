#include "pr/physics/utility/ldraw.h"
#include "src/scene/scene.h"
#include "src/utils/scene_loader.h"

namespace physics_sandbox
{
	namespace
	{
		using Clock = std::chrono::steady_clock;

		double ElapsedMs(Clock::time_point beg, Clock::time_point end)
		{
			return std::chrono::duration<double, std::milli>(end - beg).count();
		}
		void AddProfile(physics::Engine::StepProfile& lhs, physics::Engine::StepProfile const& rhs)
		{
			lhs.m_new_frame_ms += rhs.m_new_frame_ms;
			lhs.m_pack_ms += rhs.m_pack_ms;
			lhs.m_upload_ms += rhs.m_upload_ms;
			lhs.m_integrate_ms += rhs.m_integrate_ms;
			lhs.m_sleepwake_ms += rhs.m_sleepwake_ms;
			lhs.m_broadphase_ms += rhs.m_broadphase_ms;
			lhs.m_collide_ms += rhs.m_collide_ms;
			lhs.m_resolve_ms += rhs.m_resolve_ms;
			lhs.m_sleepupdate_ms += rhs.m_sleepupdate_ms;
			lhs.m_readback_ms += rhs.m_readback_ms;
			lhs.m_gpu_run_ms += rhs.m_gpu_run_ms;
			lhs.m_unpack_ms += rhs.m_unpack_ms;
		}
		bool SameVec(v4 const& lhs, v4 const& rhs)
		{
			return std::memcmp(&lhs, &rhs, sizeof(v4)) == 0;
		}
		bool SameShapeDesc(scene_loader::BodyDesc const& lhs, scene_loader::BodyDesc const& rhs)
		{
			if (lhs.shape_type != rhs.shape_type)
				return false;

			switch (lhs.shape_type)
			{
				case scene_loader::BodyDesc::EShape::Box:
				{
					return SameVec(lhs.box_dimensions, rhs.box_dimensions);
				}
				case scene_loader::BodyDesc::EShape::Sphere:
				{
					return lhs.sphere_radius == rhs.sphere_radius;
				}
				case scene_loader::BodyDesc::EShape::Line:
				{
					return lhs.line_length == rhs.line_length && lhs.line_thickness == rhs.line_thickness;
				}
				case scene_loader::BodyDesc::EShape::Triangle:
				{
					return SameVec(lhs.tri_verts[0], rhs.tri_verts[0]) && SameVec(lhs.tri_verts[1], rhs.tri_verts[1]) && SameVec(lhs.tri_verts[2], rhs.tri_verts[2]);
				}
				case scene_loader::BodyDesc::EShape::Polytope:
				{
					if (lhs.polytope_verts.size() != rhs.polytope_verts.size())
						return false;

					return std::memcmp(lhs.polytope_verts.data(), rhs.polytope_verts.data(), lhs.polytope_verts.size() * sizeof(v4)) == 0;
				}
				default:
				{
					throw std::runtime_error("Unknown shape type in scene description");
				}
			}
		}
		void AppendShape(byte_data<16>& shape_buffer, scene_loader::BodyDesc const& bd)
		{
			auto ofs = shape_buffer.size();
			switch (bd.shape_type)
			{
				case scene_loader::BodyDesc::EShape::Box:
				{
					shape_buffer.push_back(collision::ShapeBox(bd.box_dimensions));
					break;
				}
				case scene_loader::BodyDesc::EShape::Sphere:
				{
					shape_buffer.push_back(collision::ShapeSphere(bd.sphere_radius));
					break;
				}
				case scene_loader::BodyDesc::EShape::Line:
				{
					shape_buffer.push_back(collision::ShapeLine(bd.line_length, bd.line_thickness));
					break;
				}
				case scene_loader::BodyDesc::EShape::Triangle:
				{
					shape_buffer.push_back(collision::ShapeTriangle(bd.tri_verts[0], bd.tri_verts[1], bd.tri_verts[2]));
					break;
				}
				case scene_loader::BodyDesc::EShape::Polytope:
				{
					shape_buffer.push_back(collision::BuildPolytopeFromPoints(bd.polytope_verts));
					break;
				}
				default:
				{
					throw std::runtime_error("Unknown shape type in scene description");
				}
			}

			// Pad to 16-byte alignment and update the shape's m_size to include the padding.
			// collision::next() uses m_size to advance the shape pointer, so it must account
			// for any alignment padding between shapes.
			shape_buffer.pad_to(16);
			shape_buffer.at_byte_ofs<collision::Shape>(ofs).m_size = s_cast<int>(shape_buffer.size() - ofs);
		}
		m4x4 PrimitiveShapeToBody(Body const& body)
		{
			using namespace collision;

			auto& shape = body.Shape();
			switch (shape.m_type)
			{
				case EShape::Sphere:
				{
					auto& sphere = shape_cast<ShapeSphere>(shape);
					return sphere.m_base.m_s2r * m4x4::Scale(sphere.m_radius, v4::Origin());
				}
				case EShape::Box:
				{
					auto& box = shape_cast<ShapeBox>(shape);
					return box.m_base.m_s2r * m4x4::Scale(box.m_radius.x, box.m_radius.y, box.m_radius.z, v4::Origin());
				}
				case EShape::Line:
				{
					auto& line = shape_cast<ShapeLine>(shape);
					if (line.m_radius != 0)
						return line.m_base.m_s2r * m4x4::Scale(line.m_radius, line.m_radius, line.m_hlength, v4::Origin());

					return line.m_base.m_s2r * m4x4::Scale(1.0f, 1.0f, line.m_hlength, v4::Origin());
				}
				default:
				{
					throw std::runtime_error("Unsupported shared primitive shape type");
				}
			}
		}
	}

	physics::EngineConfig DefaultEngineConfig()
	{
		return physics::EngineConfig{};
	}

	Scene::Scene(rdr12::Renderer* rdr)
		: m_rdr(rdr)
		, m_physics(DefaultEngineConfig(), rdr ? rdr->d3d() : nullptr)
		, m_box(v4{ 2, 2, 2, 0 })
		, m_body()
		, m_shape_buffer()
		, m_gravity(v4::Zero())
		, m_kill_zone_height(-100.0f)
		, m_physics_substeps(1)
		, m_ground_gfx()
		, m_origin_gfx()
		, m_contacts_gfx()
		, m_show_contacts(true)
		, m_clock()
		, m_current_scenario()
		, m_diag()
		, m_step_count()
	{
		// Hook collision detection for detailed diagnostics only. The normal UI path uses the GPU contact counter
		// after Step(), avoiding a full contact-buffer readback when contact details are not needed.
		#ifdef PR_PHYSICS_DIAGNOSTICS
		m_physics.Collisions += [&](auto&, std::span<physics::RbContact const> contacts)
		{
			UpdateCollisionGfx(contacts);

			if (std::ssize(m_body) == 2 && !contacts.empty())
			{
				m_diag.before[0] = BodySnapshot::Capture(m_body[0]);
				m_diag.before[1] = BodySnapshot::Capture(m_body[1]);

				auto const& c = contacts.front();
				m_diag.contact_point_ws = c.m_objA->O2W() * c.m_point_at_t;
				m_diag.contact_normal_ws = (c.m_objA->O2W().rot * c.m_axis).w0();
				m_diag.depth = c.m_depth;
			}
		};
		#endif

		// Create a coordinate frame at the origin for visual reference
		if (m_rdr)
		{
			ldraw::Builder ldr;
			ldr.CoordFrame("origin").scale(3.0f).width(2.0f);
			auto result = rdr12::ldraw::Parse(*m_rdr, ldr.ToBinary());
			if (!result.m_objects.empty())
				m_origin_gfx = result.m_objects.front();
		}
	}

	// Reset the simulation to the current scenario's initial conditions
	void Scene::Reset()
	{
		m_clock = 0;
		m_step_count = 0;
		m_diag.Reset();
		m_gravity = v4::Zero();
		m_kill_zone_height = -100.0f;
		m_physics_substeps = 1;
		m_physics.Config(DefaultEngineConfig());

		// The engine caches caller-owned shapes/bodies by pointer. Drop those references before reusing scene storage.
		m_physics.ResetCaches();

		// Clean up the ground plane visual
		m_ground_gfx = nullptr;

		// Release any shapes owned by a previously loaded JSON scene.
		m_body.resize(0);
		m_shape_buffer.resize(0);

		// Set up perfectly elastic, frictionless material for clean collision tests
		m_physics.Material(physics::Material{
			.m_id = physics::Material::DefaultID,
			.m_friction_static = 0.0f,
			.m_elasticity_norm = 1.0f,
			.m_elasticity_tang = 0.0f,
			.m_elasticity_tors = 0.0f,
		});
	}

	// Advance the simulation by one time step. Returns true if a collision occurred during this step.
	bool Scene::Step(double elapsed_seconds)
	{
		auto const step_beg = Clock::now();
		m_clock += elapsed_seconds;
		auto const substeps = std::max(m_physics_substeps, 1);
		auto dt = static_cast<float>(elapsed_seconds / substeps);

		// Reset per-step collision flag
		m_diag.occurred = false;
		auto engine_profile = physics::Engine::StepProfile{};
		auto gravity_ms = 0.0;
		auto physics_ms = 0.0;

		for (int substep = 0; substep != substeps; ++substep)
		{
			// Apply gravity as an external force: F = m * g.
			// Static bodies (infinite mass) are skipped — they should not accelerate.
			// Forces are cleared by Evolve() at the end of each step, so we re-apply each substep.
			auto const gravity_beg = Clock::now();
			if (LengthSq(m_gravity) != 0)
			{
				for (auto& body : m_body)
					body.GravityWS(m_gravity);
			}
			auto const gravity_end = Clock::now();
			gravity_ms += ElapsedMs(gravity_beg, gravity_end);

			// Step physics (Evolve -> Broad Phase -> Narrow Phase -> PostCollisionDetection -> Resolve).
			auto const physics_beg = Clock::now();
			m_physics.Step(dt, std::span{ m_body });
			auto const physics_end = Clock::now();
			physics_ms += ElapsedMs(physics_beg, physics_end);
			AddProfile(engine_profile, m_physics.LastStepProfile());
			if (m_physics.LastCollisionStats().LastContactCount() != 0)
				m_diag.occurred = true;
		}
		if (m_diag.occurred)
			++m_diag.count;

		++m_step_count;

		#ifdef PR_PHYSICS_DIAGNOSTICS
		{
			// If a collision occurred this step, capture post-impulse snapshots.
			// Detailed logging is only done for the two-body test scenarios (not file-loaded scenes).
			if (m_diag.occurred && std::ssize(m_body) == 2)
			{
				m_diag.after[0] = BodySnapshot::Capture(m_body[0]);
				m_diag.after[1] = BodySnapshot::Capture(m_body[1]);
				LogCollisionDiagnostics();
			}
		}
		#endif

		// Kill zone: freeze bodies that have fallen below the threshold.
		// This prevents escaped bodies from accumulating extreme velocities
		// that corrupt float precision for the entire simulation.
		auto const kill_beg = Clock::now();
		for (int i = 0; i != std::ssize(m_body); ++i)
		{
			auto mass = m_body[i].Mass();
			if (mass >= physics::InfiniteMass * 0.5f)
				continue; // skip static bodies

			auto pos = m_body[i].O2W().pos;
			if (pos.z < m_kill_zone_height)
			{
				m_body[i].ZeroMomentum();
				m_body[i].ZeroForces();
			}
		}
		auto const kill_end = Clock::now();

		m_last_step_profile.m_total_ms = ElapsedMs(step_beg, kill_end);
		m_last_step_profile.m_gravity_ms = gravity_ms;
		m_last_step_profile.m_physics_ms = physics_ms;
		m_last_step_profile.m_kill_zone_ms = ElapsedMs(kill_beg, kill_end);
		m_last_step_profile.m_engine = engine_profile;

		return m_diag.occurred;
	}

	// Configure bodies for the current scenario. All test scenarios use no external
	// forces so that collisions can be validated against analytic predictions.
	void Scene::SetupScenario(EScenario scenario)
	{
		// The engine caches caller-owned shapes/bodies by pointer. Drop those references before reusing scene storage.
		m_physics.ResetCaches();

		m_body.resize(0);
		m_body.push_back(Body(m_rdr));
		m_body.push_back(Body(m_rdr));
		auto& objA = m_body[0];
		auto& objB = m_body[1];
		objA.m_colour = Colour32(0xFFFFA040U);
		objB.m_colour = Colour32(0xFF40A0FFU);

		// Common setup: zero forces/momentum
		for (int i = 0; i != std::ssize(m_body); ++i)
		{
			m_body[i].ZeroForces();
			m_body[i].ZeroMomentum();
		}

		// Load the scenario
		switch (scenario)
		{
			case EScenario::Sandbox:
			{
				// Default sandbox: two boxes approaching each other gently
				objA.Shape(m_box, physics::Inertia::Box(v4{ 1, 1, 1, 0 }, 10.0f));
				objB.Shape(m_box, physics::Inertia::Box(v4{ 1, 1, 1, 0 }, 10.0f));
				objA.O2W(m4x4::Translation( -5.0f, 0, 0));
				objB.O2W(m4x4::Translation( +5.0f, 0, 0));
				objA.VelocityWS(v4::Zero(), v4{ +2.0f, 0, 0, 0 });
				objB.VelocityWS(v4::Zero(), v4{ -2.0f, 0, 0, 0 });
				break;
			}
			case EScenario::HeadOnEqualMass:
			{
				// Two equal-mass boxes approaching each other along X.
				// Elastic collision should swap velocities exactly.
				objA.Shape(m_box, physics::Inertia::Box(v4{ 1, 1, 1, 0 }, 10.0f));
				objB.Shape(m_box, physics::Inertia::Box(v4{ 1, 1, 1, 0 }, 10.0f));
				objA.O2W(m4x4::Translation( -5.0f, 0, 0));
				objB.O2W(m4x4::Translation( +5.0f, 0, 0));
				objA.VelocityWS(v4::Zero(), v4{ +3.0f, 0, 0, 0 });
				objB.VelocityWS(v4::Zero(), v4{ -3.0f, 0, 0, 0 });
				break;
			}
			case EScenario::HeadOnDiffMass:
			{
				// Mass 10 hits mass 5 head-on along X.
				// v1' = (m1-m2)/(m1+m2)*v1 + 2*m2/(m1+m2)*v2
				// v2' = 2*m1/(m1+m2)*v1 + (m2-m1)/(m1+m2)*v2
				objA.Shape(m_box, physics::Inertia::Box(v4{ 1, 1, 1, 0 }, 10.0f));
				objB.Shape(m_box, physics::Inertia::Box(v4{ 1, 1, 1, 0 }, 5.0f));
				objA.O2W(m4x4::Translation( -5.0f, 0, 0));
				objB.O2W(m4x4::Translation( +5.0f, 0, 0));
				objA.VelocityWS(v4::Zero(), v4{ +3.0f, 0, 0, 0 });
				objB.VelocityWS(v4::Zero(), v4{ -3.0f, 0, 0, 0 });
				break;
			}
			case EScenario::StationaryTarget:
			{
				// Moving box hits a stationary box (classic billiard scenario)
				objA.Shape(m_box, physics::Inertia::Box(v4{ 1, 1, 1, 0 }, 10.0f));
				objB.Shape(m_box, physics::Inertia::Box(v4{ 1, 1, 1, 0 }, 10.0f));
				objA.O2W(m4x4::Translation( -5.0f, 0, 0));
				objB.O2W(m4x4::Translation( +5.0f, 0, 0));
				objA.VelocityWS(v4::Zero(), v4{ +3.0f, 0, 0, 0 });
				objB.VelocityWS(v4::Zero(), v4::Zero());
				break;
			}
			case EScenario::OffCenter:
			{
				// Off-center hit: boxes offset in Y, collision induces rotation.
				// Body A approaches along X but is offset in Y so the contact
				// point is not aligned with the centres of mass.
				objA.Shape(m_box, physics::Inertia::Box(v4{ 1, 1, 1, 0 }, 10.0f));
				objB.Shape(m_box, physics::Inertia::Box(v4{ 1, 1, 1, 0 }, 10.0f));
				objA.O2W(m4x4::Translation( -5.0f, +0.8f, 0));
				objB.O2W(m4x4::Translation( +5.0f, 0, 0));
				objA.VelocityWS(v4::Zero(), v4{ +3.0f, 0, 0, 0 });
				objB.VelocityWS(v4::Zero(), v4::Zero());
				break;
			}
			case EScenario::Oblique:
			{
				// Oblique collision: bodies approaching at an angle
				objA.Shape(m_box, physics::Inertia::Box(v4{ 1, 1, 1, 0 }, 10.0f));
				objB.Shape(m_box, physics::Inertia::Box(v4{ 1, 1, 1, 0 }, 10.0f));
				objA.O2W(m4x4::Translation( -5.0f, -2.0f, 0));
				objB.O2W(m4x4::Translation( +5.0f, +2.0f, 0));
				objA.VelocityWS(v4::Zero(), v4{ +3.0f, +1.0f, 0, 0 });
				objB.VelocityWS(v4::Zero(), v4{ -3.0f, -1.0f, 0, 0 });
				break;
			}
		}

		auto mat = m_physics.Material(0);

		DbgLog("\n--- Reset: Scenario %d [%s] ---\n", static_cast<int>(scenario), ScenarioName(scenario));
		DbgLog("  Material: elasticity_norm=%.2f friction=%.2f\n", mat.m_elasticity_norm, mat.m_friction_static);
		for (int i = 0; i != std::ssize(m_body); ++i)
		{
			auto snap = BodySnapshot::Capture(m_body[i]);
			snap.Log(FmtS("Body %d (initial)", i));
		}
		auto total_p = m_body[0].MomentumWS().lin + m_body[1].MomentumWS().lin;
		DbgLog("  Total lin momentum: (%.4f, %.4f, %.4f)\n", total_p.x, total_p.y, total_p.z);
		DbgLog("  Total KE: %.6f\n", m_body[0].KineticEnergy() + m_body[1].KineticEnergy());

		m_current_scenario = scenario;
	}

	// Load a scene from a JSON file.
	// Replaces the current scenario with bodies defined in the file.
	// Shapes are heap-allocated and owned by m_owned_shapes.
	void Scene::LoadScene(scene_loader::SceneDesc scene_desc)
	{
		auto const load_beg = Clock::now();
		auto mark = load_beg;
		m_last_load_profile = {};
		m_last_load_profile.m_has_renderer = m_rdr != nullptr;

		// Reset simulation state
		m_clock = 0;
		m_step_count = 0;
		m_diag.Reset();

		// The engine caches caller-owned shapes/bodies by pointer. Drop those references before reusing scene storage.
		m_physics.ResetCaches();

		// Clean up ground plane visual from previous scene
		m_ground_gfx = nullptr;

		// Clear existing bodies and owned shapes
		m_body.resize(0);
		m_shape_buffer.resize(0);

		// Apply gravity from the scene file
		m_gravity = scene_desc.gravity;
		m_physics_substeps = scene_desc.physics_substeps;
		auto engine_config = DefaultEngineConfig();
		engine_config.max_collision_pairs = scene_desc.physics_max_collision_pairs;
		engine_config.solver_iterations = scene_desc.physics_solver_iterations;
		engine_config.position_iterations = scene_desc.physics_position_iterations;
		engine_config.tgs_steps = scene_desc.physics_tgs_steps;
		engine_config.tgs_velocity_bias_max = scene_desc.physics_tgs_velocity_bias_max;
		m_physics.Config(engine_config);

		// Set the kill zone well below the ground plane. Bodies that fall below
		// this height are frozen to prevent them from corrupting the simulation.
		m_kill_zone_height = (scene_desc.ground ? scene_desc.ground->height : 0) - 50.0f;

		// Apply material properties from the scene file
		m_physics.Material(physics::Material{
			.m_id = physics::Material::DefaultID,
			.m_friction_static = scene_desc.friction,
			.m_elasticity_norm = scene_desc.elasticity,
			.m_elasticity_tang = 0.0f,
			.m_elasticity_tors = 0.0f,
		});
		auto const prepare_end = Clock::now();
		m_last_load_profile.m_prepare_ms = ElapsedMs(mark, prepare_end);
		mark = prepare_end;

		// Count total bodies: scene bodies + optional ground plane body
		auto num_scene_bodies = static_cast<int>(scene_desc.bodies.size());
		auto total_bodies = num_scene_bodies + (scene_desc.ground ? 1 : 0);
		m_last_load_profile.m_body_count = total_bodies;
		auto scene_bbox = CalculateSceneBBox(scene_desc);
		const auto ground_thickness = 10.0f;
		auto scene_rng = std::default_random_engine(scene_desc.seed);
		auto const bbox_end = Clock::now();
		m_last_load_profile.m_bbox_ms = ElapsedMs(mark, bbox_end);
		mark = bbox_end;

		// Shapes for the bodies in the scene. Generated bodies deliberately reuse a small shape palette, and this de-duplicates identical
		// descriptors across the whole scene so collision shapes are shared instead of rebuilt per body.
		auto shape_lookup = std::vector<int>(num_scene_bodies, -1);
		auto unique_shape_body_index = std::vector<int>{};
		unique_shape_body_index.reserve(num_scene_bodies);
		for (auto i = 0; i != num_scene_bodies; ++i)
		{
			auto const& bd = scene_desc.bodies[i];
			for (auto j = 0; j != isize(unique_shape_body_index); ++j)
			{
				if (SameShapeDesc(bd, scene_desc.bodies[unique_shape_body_index[j]]))
				{
					shape_lookup[i] = j;
					break;
				}
			}

			if (shape_lookup[i] == -1)
			{
				shape_lookup[i] = isize(unique_shape_body_index);
				unique_shape_body_index.push_back(i);
			}
		}

		m_last_load_profile.m_shape_count = isize(unique_shape_body_index) + (scene_desc.ground ? 1 : 0);
		{
			m_shape_buffer.reserve(m_last_load_profile.m_shape_count * 512);
			for (auto body_index : unique_shape_body_index)
				AppendShape(m_shape_buffer, scene_desc.bodies[body_index]);

			// Create a collision shape for the ground plane
			if (scene_desc.ground)
			{
				// Create the ground plane body as a large thin box with infinite mass.
				v2 extent = scene_desc.ground->size;
				if (LengthSq(extent) == 0) extent = v2(10.0f * Length(scene_bbox.Radius().xy));
				auto bd = scene_loader::BodyDesc{};
				bd.shape_type = scene_loader::BodyDesc::EShape::Box;
				bd.box_dimensions = v4{ extent.x, extent.y, ground_thickness, 0 };
				AppendShape(m_shape_buffer, bd);
			}
		}
		auto const shapes_end = Clock::now();
		m_last_load_profile.m_shapes_ms = ElapsedMs(mark, shapes_end);
		mark = shapes_end;

		// Bodies from the scene description.
		{
			auto shape_ptrs = std::vector<collision::Shape const*>{};
			shape_ptrs.reserve(m_last_load_profile.m_shape_count);
			for (auto shape_ptr = m_shape_buffer.data<collision::Shape>(); shape_ptr != nullptr && isize(shape_ptrs) != m_last_load_profile.m_shape_count; shape_ptr = collision::next(shape_ptr))
				shape_ptrs.push_back(shape_ptr);
			if (isize(shape_ptrs) != m_last_load_profile.m_shape_count)
				throw std::runtime_error("Scene shape buffer ended before all shapes were read");

			// Phase 1: Create bodies WITHOUT the renderer so the ShapeChange handler doesn't
			// try to create graphics yet. This avoids dangling pointer issues during the
			// construction loop (graphics creation calls AddShape which reads the shape data).
			m_body.reserve(total_bodies);
			for (auto body_index = 0; body_index != num_scene_bodies; ++body_index)
			{
				auto const& bd = scene_desc.bodies[body_index];
				Body body(nullptr);
				auto o2w = m4x4::TransformDeg(bd.rotation.x, bd.rotation.y, bd.rotation.z, bd.position);
				body.O2W(o2w);
				body.Shape(shape_ptrs[shape_lookup[body_index]], bd.mass);
				body.VelocityWS(bd.angular_velocity, bd.velocity);
				if (bd.sleeping)
					body.Sleep();
				body.m_colour = bd.colour ? *bd.colour : RandomRGB(scene_rng, 0.0f, 1.0f);
				m_body.push_back(std::move(body));
			}

			// Create the ground plane body as a large thin box with infinite mass.
			// The box is thin in Z (0.5 units) and wide in XY, centred at the ground height.
			if (scene_desc.ground)
			{
				Body ground(nullptr);
				ground.O2W(m4x4::Translation(0, 0, scene_desc.ground->height - 0.5f * ground_thickness));
				ground.Shape(shape_ptrs.back(), -1.0f);
				ground.m_colour = scene_desc.ground->colour ? *scene_desc.ground->colour : RandomRGB(scene_rng, 0.0f, 1.0f);
				m_body.push_back(std::move(ground));
			}
		}
		auto const bodies_end = Clock::now();
		m_last_load_profile.m_bodies_ms = ElapsedMs(mark, bodies_end);
		mark = bodies_end;

		// Create the graphics now that all bodies and shapes are stable in memory.
		// Build a single LDraw script containing all body shapes, then parse it in one
		// call. This is dramatically faster than parsing each body individually because
		// it amortises renderer overhead (shader cache lookups, resource pool, etc.).
		if (m_rdr)
		{
			using namespace pr::ldraw;

			// Share canonical renderer models for primitive shapes and keep exact prototypes only for geometry that cannot be represented by
			// a simple scale transform. Renderer model creation dominates large scene loads, so this avoids reparsing hundreds of boxes and spheres.
			auto use_box_prototype = false;
			auto use_sphere_prototype = false;
			auto use_thick_line_prototype = false;
			auto use_thin_line_prototype = false;
			auto exact_prototype_lookup = std::vector<int>(total_bodies, -1);
			auto exact_prototype_body_index = std::vector<int>{};
			for (int i = 0; i != total_bodies; ++i)
			{
				auto& body = m_body[i];
				if (!body.HasShape())
					continue;

				switch (body.Shape().m_type)
				{
					case collision::EShape::Box:
					{
						use_box_prototype = true;
						break;
					}
					case collision::EShape::Sphere:
					{
						use_sphere_prototype = true;
						break;
					}
					case collision::EShape::Line:
					{
						auto& line = collision::shape_cast<collision::ShapeLine>(body.Shape());
						if (line.m_radius != 0)
							use_thick_line_prototype = true;
						else
							use_thin_line_prototype = true;
						break;
					}
					default:
					{
						for (int j = 0; j != isize(exact_prototype_body_index); ++j)
						{
							auto const prototype_body = exact_prototype_body_index[j];
							if (i < num_scene_bodies && prototype_body < num_scene_bodies && SameShapeDesc(scene_desc.bodies[i], scene_desc.bodies[prototype_body]))
							{
								exact_prototype_lookup[i] = j;
								break;
							}
						}
						if (exact_prototype_lookup[i] == -1)
						{
							exact_prototype_lookup[i] = isize(exact_prototype_body_index);
							exact_prototype_body_index.push_back(i);
						}
						break;
					}
				}
			}

			Builder builder;
			auto prototype_count = 0;
			auto const box_prototype_name = std::string("ShapeBox");
			auto const sphere_prototype_name = std::string("ShapeSphere");
			auto const thick_line_prototype_name = std::string("ShapeThickLine");
			auto const thin_line_prototype_name = std::string("ShapeThinLine");
			if (use_box_prototype)
			{
				builder.Box(box_prototype_name).box(2, 2, 2).hide();
				++prototype_count;
			}
			if (use_sphere_prototype)
			{
				builder.Sphere(sphere_prototype_name).sphere(1).facets(5).hide();
				++prototype_count;
			}
			if (use_thick_line_prototype)
			{
				builder.Cylinder(thick_line_prototype_name).cylinder(2, 1).facets(1, 50).end_caps().hide();
				++prototype_count;
			}
			if (use_thin_line_prototype)
			{
				builder.Line(thin_line_prototype_name).line(v4(0, 0, -1, 1), v4(0, 0, +1, 1)).hide();
				++prototype_count;
			}

			auto exact_prototype_names = std::vector<std::string>{};
			exact_prototype_names.reserve(exact_prototype_body_index.size());
			for (int i = 0; i != isize(exact_prototype_body_index); ++i)
			{
				auto const body_index = exact_prototype_body_index[i];
				auto const prototype_name = std::format("ShapeExact{}", i);
				exact_prototype_names.push_back(prototype_name);
				builder.Add<LdrCollisionShape>(prototype_name).shape(m_body[body_index].Shape()).hide();
				++prototype_count;
			}

			for (int i = 0; i != total_bodies; ++i)
			{
				auto& body = m_body[i];
				if (!body.HasShape())
					continue;

				auto prototype_name = std::string_view{};
				body.m_gfx_o2b = m4x4::Identity();
				switch (body.Shape().m_type)
				{
					case collision::EShape::Box:
					{
						prototype_name = box_prototype_name;
						body.m_gfx_o2b = PrimitiveShapeToBody(body);
						break;
					}
					case collision::EShape::Sphere:
					{
						prototype_name = sphere_prototype_name;
						body.m_gfx_o2b = PrimitiveShapeToBody(body);
						break;
					}
					case collision::EShape::Line:
					{
						auto& line = collision::shape_cast<collision::ShapeLine>(body.Shape());
						prototype_name = line.m_radius != 0 ? std::string_view(thick_line_prototype_name) : std::string_view(thin_line_prototype_name);
						body.m_gfx_o2b = PrimitiveShapeToBody(body);
						break;
					}
					default:
					{
						prototype_name = exact_prototype_names[exact_prototype_lookup[i]];
						break;
					}
				}

				builder.Instance(std::format("Body{}", i), body.m_colour.argb)
					.address(prototype_name)
					.group_tint(body.m_colour.argb)
					.o2w(body.O2W() * body.m_gfx_o2b);
			}
			auto const ldraw_build_end = Clock::now();
			m_last_load_profile.m_ldraw_build_ms = ElapsedMs(mark, ldraw_build_end);
			mark = ldraw_build_end;

			// Parse all shapes in one batch
			auto ldr_script = builder.ToBinary();
			m_last_load_profile.m_ldraw_byte_count = ldr_script.size();
			auto const ldraw_serialise_end = Clock::now();
			m_last_load_profile.m_ldraw_serialise_ms = ElapsedMs(mark, ldraw_serialise_end);
			mark = ldraw_serialise_end;

			auto result = rdr12::ldraw::Parse(*m_rdr, ldr_script);
			m_last_load_profile.m_ldraw_object_count = static_cast<int>(result.m_objects.size());
			auto const ldraw_parse_end = Clock::now();
			m_last_load_profile.m_ldraw_parse_ms = ElapsedMs(mark, ldraw_parse_end);
			mark = ldraw_parse_end;

			// Assign each parsed instance object to its corresponding body. The prototype objects are first in the result list.
			for (int i = 0; i != total_bodies; ++i)
			{
				auto& body = m_body[i];
				if (!body.HasShape())
					continue;

				auto const obj_idx = prototype_count + i;
				if (obj_idx < static_cast<int>(result.m_objects.size()))
					body.m_gfx = result.m_objects[obj_idx];

				body.UpdateGfx();
			}
			auto const ldraw_assign_end = Clock::now();
			m_last_load_profile.m_ldraw_assign_ms = ElapsedMs(mark, ldraw_assign_end);
			mark = ldraw_assign_end;
		}
		// Logging
		{
			auto mat = m_physics.Material(0);

			DbgLog("\n--- Loaded scene from: %ls ---\n", scene_desc.filepath.c_str());
			if (!scene_desc.description.empty())
				DbgLog("  Description: %s\n", scene_desc.description.c_str());
			DbgLog("  Bodies: %d\n", static_cast<int>(m_body.size()));
			DbgLog("  Gravity: (%.2f, %.2f, %.2f)\n", m_gravity.x, m_gravity.y, m_gravity.z);
			DbgLog("  Ground: %s (height=%.2f)\n", scene_desc.ground ? "yes" : "no", scene_desc.ground ? scene_desc.ground->height : 0.0f);
			DbgLog("  Material: elasticity=%.2f friction=%.2f\n", mat.m_elasticity_norm, mat.m_friction_static);
			for (int i = 0; i != std::ssize(m_body); ++i)
			{
				auto snap = BodySnapshot::Capture(m_body[i]);
				auto name = (i < static_cast<int>(scene_desc.bodies.size())) ? scene_desc.bodies[i].name.c_str() : "ground";
				snap.Log(FmtS("Body %d '%s'", i, name));
			}
		}
		auto const logging_end = Clock::now();
		m_last_load_profile.m_logging_ms = ElapsedMs(mark, logging_end);
		m_last_load_profile.m_total_ms = ElapsedMs(load_beg, logging_end);
	}

	// Log comprehensive collision diagnostics and analytic comparisons
	void Scene::LogCollisionDiagnostics()
	{
		DbgLog("\n========================================\n");
		DbgLog("=== COLLISION #%d [%s] at t=%.4f ===\n", m_diag.count, ScenarioName(m_current_scenario), m_clock);
		DbgLog("========================================\n");

		// Contact info
		DbgLog("Contact:\n");
		DbgLog("  point_ws = (%.4f, %.4f, %.4f)\n",
			m_diag.contact_point_ws.x, m_diag.contact_point_ws.y, m_diag.contact_point_ws.z);
		DbgLog("  normal_ws = (%.4f, %.4f, %.4f)\n",
			m_diag.contact_normal_ws.x, m_diag.contact_normal_ws.y, m_diag.contact_normal_ws.z);
		DbgLog("  depth = %.6f\n", m_diag.depth);

		// Pre-impulse state
		DbgLog("Pre-impulse:\n");
		m_diag.before[0].Log("Body A");
		m_diag.before[1].Log("Body B");
		auto pre_total_p = m_diag.before[0].momentum.lin + m_diag.before[1].momentum.lin;
		auto pre_total_ke = m_diag.before[0].ke + m_diag.before[1].ke;
		DbgLog("  Total lin momentum: (%.4f, %.4f, %.4f)\n", pre_total_p.x, pre_total_p.y, pre_total_p.z);
		DbgLog("  Total KE: %.6f\n", pre_total_ke);

		// Angular momentum about world origin = spin (at CoM) + orbital (com × p).
		// MomentumWS().ang is the spin angular momentum at each body's centre of mass.
		// We must add the orbital term com × (m*v) for a correct system total.
		auto ang_mom_about_origin = [](BodySnapshot const& s)
		{
			auto orbital = Cross(s.com_pos, s.momentum.lin);
			return s.momentum.ang + orbital;
		};
		auto pre_total_L = ang_mom_about_origin(m_diag.before[0]) + ang_mom_about_origin(m_diag.before[1]);
		DbgLog("  Total ang momentum (about origin): (%.4f, %.4f, %.4f)\n", pre_total_L.x, pre_total_L.y, pre_total_L.z);

		// Post-impulse state
		DbgLog("Post-impulse:\n");
		m_diag.after[0].Log("Body A");
		m_diag.after[1].Log("Body B");
		auto post_total_p = m_diag.after[0].momentum.lin + m_diag.after[1].momentum.lin;
		auto post_total_ke = m_diag.after[0].ke + m_diag.after[1].ke;
		DbgLog("  Total lin momentum: (%.4f, %.4f, %.4f)\n", post_total_p.x, post_total_p.y, post_total_p.z);
		DbgLog("  Total KE: %.6f\n", post_total_ke);

		auto post_total_L = ang_mom_about_origin(m_diag.after[0]) + ang_mom_about_origin(m_diag.after[1]);
		DbgLog("  Total ang momentum (about origin): (%.4f, %.4f, %.4f)\n", post_total_L.x, post_total_L.y, post_total_L.z);

		// Conservation checks
		auto dp = post_total_p - pre_total_p;
		auto dL = post_total_L - pre_total_L;
		auto dke = post_total_ke - pre_total_ke;
		auto dL_pct = Length(pre_total_L) > 0.01f ? 100.0f * Length(dL) / Length(pre_total_L) : 0.0f;
		DbgLog("Conservation:\n");
		DbgLog("  Delta lin momentum: (%.6f, %.6f, %.6f) |dp|=%.6f\n", dp.x, dp.y, dp.z, Length(dp));
		DbgLog("  Delta ang momentum: (%.6f, %.6f, %.6f) |dL|=%.6f (%.2f%%)\n", dL.x, dL.y, dL.z, Length(dL), dL_pct);
		DbgLog("  Delta KE: %.6f (%.2f%%)\n", dke, pre_total_ke > 0 ? 100.0f * dke / pre_total_ke : 0.0f);

		// Pass/fail thresholds.
		// Angular momentum conservation is approximate due to sub-step time correction.
		bool momentum_ok = Length(dp) < 0.01f;
		auto ang_tol = Max(0.01f, Length(pre_total_L) * 0.05f);
		bool ang_momentum_ok = Length(dL) < ang_tol;
		bool ke_ok = Abs(dke) < 0.01f * pre_total_ke;
		DbgLog("  Lin Momentum conserved: %s\n", momentum_ok ? "PASS" : "*** FAIL ***");
		DbgLog("  Ang Momentum conserved: %s%s\n", ang_momentum_ok ? "PASS" : "*** FAIL ***", (Length(dL) > 0.01f && ang_momentum_ok) ? " (within sub-step tolerance)" : "");
		DbgLog("  KE conserved (elastic): %s\n", ke_ok ? "PASS" : "*** FAIL ***");

		// Analytic predictions for 1D head-on elastic collisions (scenarios 1-3)
		if (m_current_scenario == EScenario::HeadOnEqualMass ||
			m_current_scenario == EScenario::HeadOnDiffMass ||
			m_current_scenario == EScenario::StationaryTarget)
		{
			LogAnalyticComparison();
		}

		DbgLog("========================================\n\n");
	}

	// Compare post-collision velocities to the analytic solution for 1D elastic collision.
	// For perfectly elastic collision:
	//   v1' = ((m1-m2)*v1 + 2*m2*v2) / (m1+m2)
	//   v2' = ((m2-m1)*v2 + 2*m1*v1) / (m1+m2)
	void Scene::LogAnalyticComparison()
	{
		auto m1 = m_diag.before[0].mass;
		auto m2 = m_diag.before[1].mass;
		auto v1 = m_diag.before[0].lin_vel.x;
		auto v2 = m_diag.before[1].lin_vel.x;

		auto v1_pred = ((m1 - m2) * v1 + 2 * m2 * v2) / (m1 + m2);
		auto v2_pred = ((m2 - m1) * v2 + 2 * m1 * v1) / (m1 + m2);

		auto v1_actual = m_diag.after[0].lin_vel.x;
		auto v2_actual = m_diag.after[1].lin_vel.x;

		DbgLog("Analytic comparison (1D elastic, X component):\n");
		DbgLog("  v1': predicted=%.4f  actual=%.4f  error=%.6f\n", v1_pred, v1_actual, Abs(v1_pred - v1_actual));
		DbgLog("  v2': predicted=%.4f  actual=%.4f  error=%.6f\n", v2_pred, v2_actual, Abs(v2_pred - v2_actual));

		auto vy1 = m_diag.after[0].lin_vel.y;
		auto vz1 = m_diag.after[0].lin_vel.z;
		auto vy2 = m_diag.after[1].lin_vel.y;
		auto vz2 = m_diag.after[1].lin_vel.z;
		DbgLog("  v1_yz: (%.6f, %.6f)  v2_yz: (%.6f, %.6f)\n", vy1, vz1, vy2, vz2);

		auto w1 = m_diag.after[0].ang_vel;
		auto w2 = m_diag.after[1].ang_vel;
		DbgLog("  ang_vel1: (%.6f, %.6f, %.6f)  ang_vel2: (%.6f, %.6f, %.6f)\n",
			w1.x, w1.y, w1.z, w2.x, w2.y, w2.z);

		bool x_ok = Abs(v1_pred - v1_actual) < 0.05f && Abs(v2_pred - v2_actual) < 0.05f;
		bool yz_ok = Abs(vy1) < 0.05f && Abs(vz1) < 0.05f && Abs(vy2) < 0.05f && Abs(vz2) < 0.05f;
		bool no_spin = Length(w1) < 0.05f && Length(w2) < 0.05f;
		DbgLog("  X velocity match: %s\n", x_ok ? "PASS" : "*** FAIL ***");
		DbgLog("  Y/Z remain zero:  %s\n", yz_ok ? "PASS" : "*** FAIL ***");
		DbgLog("  No angular vel:   %s\n", no_spin ? "PASS" : "*** FAIL ***");
	}

	// Run all scenarios in sequence without rendering, log results for each.
	void Scene::RunAllTests()
	{
		DbgLog("\n################################################################\n");
		DbgLog("### AUTO-TEST: Running all scenarios\n");
		DbgLog("################################################################\n");

		auto const dt = 1.0f / 100.0f;
		auto const max_steps = 5000;

		for (int s = 1; s <= 5; ++s)
		{
			Reset();
			SetupScenario(static_cast<EScenario>(s));

			for (int step = 0; step < max_steps && m_diag.count == 0; ++step)
			{
				m_diag.occurred = false;

				auto bodies = std::span(m_body);
				m_physics.Step(dt, bodies);
				m_clock += dt;

				if (m_diag.occurred)
				{
					m_diag.after[0] = BodySnapshot::Capture(m_body[0]);
					m_diag.after[1] = BodySnapshot::Capture(m_body[1]);
					LogCollisionDiagnostics();
				}

				// NaN guard
				auto pos = m_body[0].O2W().pos;
				if (!std::isfinite(pos.x))
				{
					DbgLog("!!! NaN detected in scenario %d at step %d\n", s, step);
					break;
				}
			}

			if (m_diag.count == 0)
				DbgLog("!!! Scenario %d: No collision after %d steps!\n", s, max_steps);
		}

		DbgLog("\n################################################################\n");
		DbgLog("### AUTO-TEST COMPLETE\n");
		DbgLog("################################################################\n");
	}

	// Dump the current scene to an LDraw file for offline analysis. This is useful for
	void Scene::Dump()
	{
		using namespace pr::ldraw;

		ldraw::Builder builder;
		builder.Add<LdrRigidBody>("body0", 0x8000FF00).rigid_body(m_body[0]);
		builder.Add<LdrRigidBody>("body1", 0x10FF0000).rigid_body(m_body[1]);
		builder.Save(L"dump\\physics_dump.ldr");
	}

	// Create/update the graphics objects for
	void Scene::UpdateCollisionGfx(std::span<physics::RbContact const> contacts)
	{
		// Contact graphics are currently disabled. Keep this path opt-in because reading detailed
		// contacts back from the GPU has a measurable cost in large scenes.
		(void)contacts;
	}

	// Calculate the bounding box for the scene (excluding terrain)
	BBox Scene::CalculateSceneBBox(scene_loader::SceneDesc const& scene_desc) const
	{
		auto bbox = BBox::Reset();
		for (auto const& bd : scene_desc.bodies)
		{
			auto pos = bd.position;
			auto rad = v4::Zero();
			switch (bd.shape_type)
			{
				case scene_loader::BodyDesc::EShape::Box:      rad = bd.box_dimensions * 0.5f; break;
				case scene_loader::BodyDesc::EShape::Sphere:   rad = v4(bd.sphere_radius, bd.sphere_radius, bd.sphere_radius, 0); break;
				case scene_loader::BodyDesc::EShape::Line:     rad = v4(0, 0, bd.line_length * 0.5f, 0); break;
				case scene_loader::BodyDesc::EShape::Triangle: rad = v4(1, 1, 1, 0); break;
				case scene_loader::BodyDesc::EShape::Polytope:
				{
					for (auto const& v : bd.polytope_verts)
						Grow(bbox, (pos + v.w0()).w1());
					continue;
				}
				default: break;
			}
			Grow(bbox, BBox(pos, rad));
		}
		return bbox;
	}
}

#if PR_UNITTESTS
namespace physics_sandbox::tests
{
	namespace
	{
		struct ReloadBodyState
		{
			v4 m_pos;
			v8motion m_vel;
		};

		scene_loader::SceneDesc BoxScene(v4 const& dimensions, float z)
		{
			auto scene_desc = scene_loader::SceneDesc{};
			scene_desc.description = "Scene reload cache test";
			scene_desc.gravity = v4::Zero();
			scene_desc.ground = scene_loader::GroundPlaneDesc{
				.size = v2{ 10.0f, 10.0f },
				.height = 0.0f,
			};
			scene_desc.bodies.push_back(scene_loader::BodyDesc{
				.name = "box",
				.shape_type = scene_loader::BodyDesc::EShape::Box,
				.box_dimensions = dimensions,
				.mass = 1.0f,
				.position = v4{ 0.0f, 0.0f, z, 1.0f },
			});
			return scene_desc;
		}

		std::vector<ReloadBodyState> RunScene(Scene& scene, scene_loader::SceneDesc scene_desc, int step_count)
		{
			scene.LoadScene(std::move(scene_desc));

			auto const dt = 1.0 / 60.0;
			for (int step = 0; step != step_count; ++step)
				scene.Step(dt);

			auto state = std::vector<ReloadBodyState>();
			state.reserve(s_cast<size_t>(std::ssize(scene.m_body)));
			for (auto const& body : scene.m_body)
			{
				state.push_back(ReloadBodyState{
					.m_pos = body.O2W().pos,
					.m_vel = body.VelocityWS(),
				});
			}

			return state;
		}
		void ExpectSameState(char const* label, std::vector<ReloadBodyState> const& baseline, std::vector<ReloadBodyState> const& actual)
		{
			PR_EXPECT(baseline.size() == actual.size());
			for (int i = 0; i != std::ssize(baseline) && i != std::ssize(actual); ++i)
			{
				auto const pos_delta = Length(baseline[i].m_pos - actual[i].m_pos);
				auto const lin_vel_delta = Length(baseline[i].m_vel.lin - actual[i].m_vel.lin);
				auto const ang_vel_delta = Length(baseline[i].m_vel.ang - actual[i].m_vel.ang);
				if (pos_delta >= 0.001f || lin_vel_delta >= 0.001f || ang_vel_delta >= 0.001f)
				{
					DbgLog("Scene reload mismatch %s body %d: pos_delta=%g lin_vel_delta=%g ang_vel_delta=%g\n", label, i, pos_delta, lin_vel_delta, ang_vel_delta);
					PR_EXPECT(false);
				}
			}
		}
	}

	PRUnitTestClass(SceneReloadTests)
	{
		PRUnitTestMethod(LoadSceneClearsEngineCachedShapeState)
		{
			auto scene = Scene(nullptr);
			auto short_box = BoxScene(v4{ 1.0f, 1.0f, 1.0f, 0.0f }, 0.6f);
			auto tall_box = BoxScene(v4{ 1.0f, 1.0f, 4.0f, 0.0f }, 2.0f);

			auto const baseline = RunScene(scene, short_box, 1);
			scene.m_physics.ResetCaches();
			(void)RunScene(scene, tall_box, 1);
			auto const reloaded = RunScene(scene, short_box, 1);

			ExpectSameState("short_box_after_tall_box", baseline, reloaded);
		}
	};
}
#endif
