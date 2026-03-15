#include "pr/physics-2/utility/ldraw.h"
#include "src/scene/scene.h"
#include "src/utils/scene_loader.h"

namespace physics_sandbox
{
	Scene::Scene(rdr12::Renderer* rdr)
		: m_rdr(rdr)
		, m_materials()
		, m_physics(m_materials, rdr ? rdr->d3d() : nullptr)
		, m_box(v4{ 2, 2, 2, 0 })
		, m_body()
		, m_shape_buffer()
		, m_gravity(v4::Zero())
		, m_kill_zone_height(-100.0f)
		, m_ground_gfx()
		, m_origin_gfx()
		, m_clock()
		, m_diag()
		, m_current_scenario()
	{
		// Hook collision detection for diagnostics. This fires AFTER Evolve but BEFORE impulse resolution.
		m_physics.PostCollisionDetection += [&](auto&, auto args)
		{
			if (args.m_contacts.empty())
				return;

			// Lightweight: just track that a collision occurred (used by UI title bar)
			m_diag.occurred = true;
			m_diag.count++;

			#ifdef PR_PHYSICS_DIAGNOSTICS
			{
				// Capture pre-impulse state for first two bodies
				m_diag.before[0] = BodySnapshot::Capture(m_body[0]);
				m_diag.before[1] = BodySnapshot::Capture(m_body[1]);

				// Capture contact info (collision data is in objA space, transform to world)
				auto const& c = collisions[0];
				m_diag.contact_point_ws = m_body[0].O2W() * c.m_point_at_t;
				m_diag.contact_normal_ws = (m_body[0].O2W().rot * c.m_axis).w0();
				m_diag.depth = c.m_depth;
			}
			#endif
		};

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
		m_diag.Reset();
		m_gravity = v4::Zero();
		m_kill_zone_height = -100.0f;

		// Clean up the ground plane visual
		m_ground_gfx = nullptr;

		// Clear the broadphasebefore modifying bodies. The broadphase holds raw
		// pointers to RigidBody objects, which become invalid if the vector resizes.
		m_physics.Broadphase().Clear();

		// Release any shapes owned by a previously loaded JSON scene.
		m_body.resize(0);
		m_shape_buffer.resize(0);

		// Set up perfectly elastic, frictionless material for clean collision tests
		auto& mat = m_materials(0);
		mat.m_elasticity_norm = 1.0f;
		mat.m_elasticity_tang = 0.0f;
		mat.m_elasticity_tors = 0.0f;
		mat.m_friction_static = 0.0f;
	}

	// Advance the simulation by one time step.
	// Returns true if a collision occurred during this step.
	bool Scene::Step(double elapsed_seconds)
	{
		m_clock += elapsed_seconds;
		auto dt = float(elapsed_seconds);

		// Reset per-step collision flag
		m_diag.occurred = false;

		// Apply gravity as an external force: F = m * g.
		// Static bodies (infinite mass) are skipped — they should not accelerate.
		// Forces are cleared by Evolve() at the end of each step, so we re-apply each frame.
		if (m_gravity != v4::Zero())
		{
			for (int i = 0; i != std::ssize(m_body); ++i)
			{
				auto mass = m_body[i].Mass();
				if (mass < physics::InfiniteMass * 0.5f)
					m_body[i].ApplyForceWS(m_gravity * mass, v4::Zero(), m_body[i].O2W().rot * m_body[i].CentreOfMassOS());
			}
		}

		// Step physics (Evolve → Broad Phase → Narrow Phase → PostCollisionDetection → Resolve)
		auto bodies = std::span(m_body);
		m_physics.Step(dt, bodies);

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

		return m_diag.occurred;
	}

	// Configure bodies for the current scenario. All test scenarios use no external
	// forces so that collisions can be validated against analytic predictions.
	void Scene::SetupScenario(EScenario scenario)
	{
		m_body.resize(0);
		m_body.push_back(Body(m_rdr));
		m_body.push_back(Body(m_rdr));
		auto& objA = m_body[0];
		auto& objB = m_body[1];

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
				objA.O2W(m4x4::Translation(v4{ -5.0f, 0, 0, 1 }));
				objB.O2W(m4x4::Translation(v4{ +5.0f, 0, 0, 1 }));
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
				objA.O2W(m4x4::Translation(v4{ -5.0f, 0, 0, 1 }));
				objB.O2W(m4x4::Translation(v4{ +5.0f, 0, 0, 1 }));
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
				objA.O2W(m4x4::Translation(v4{ -5.0f, 0, 0, 1 }));
				objB.O2W(m4x4::Translation(v4{ +5.0f, 0, 0, 1 }));
				objA.VelocityWS(v4::Zero(), v4{ +3.0f, 0, 0, 0 });
				objB.VelocityWS(v4::Zero(), v4{ -3.0f, 0, 0, 0 });
				break;
			}
			case EScenario::StationaryTarget:
			{
				// Moving box hits a stationary box (classic billiard scenario)
				objA.Shape(m_box, physics::Inertia::Box(v4{ 1, 1, 1, 0 }, 10.0f));
				objB.Shape(m_box, physics::Inertia::Box(v4{ 1, 1, 1, 0 }, 10.0f));
				objA.O2W(m4x4::Translation(v4{ -5.0f, 0, 0, 1 }));
				objB.O2W(m4x4::Translation(v4{ +5.0f, 0, 0, 1 }));
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
				objA.O2W(m4x4::Translation(v4{ -5.0f, +0.8f, 0, 1 }));
				objB.O2W(m4x4::Translation(v4{ +5.0f, 0, 0, 1 }));
				objA.VelocityWS(v4::Zero(), v4{ +3.0f, 0, 0, 0 });
				objB.VelocityWS(v4::Zero(), v4::Zero());
				break;
			}
			case EScenario::Oblique:
			{
				// Oblique collision: bodies approaching at an angle
				objA.Shape(m_box, physics::Inertia::Box(v4{ 1, 1, 1, 0 }, 10.0f));
				objB.Shape(m_box, physics::Inertia::Box(v4{ 1, 1, 1, 0 }, 10.0f));
				objA.O2W(m4x4::Translation(v4{ -5.0f, -2.0f, 0, 1 }));
				objB.O2W(m4x4::Translation(v4{ +5.0f, +2.0f, 0, 1 }));
				objA.VelocityWS(v4::Zero(), v4{ +3.0f, +1.0f, 0, 0 });
				objB.VelocityWS(v4::Zero(), v4{ -3.0f, -1.0f, 0, 0 });
				break;
			}
		}

		// Rebuild the broadphase with the active bodies
		m_physics.Broadphase().Clear();
		for (int i = 0; i != std::ssize(m_body); ++i)
			m_physics.Broadphase().Add(m_body[i]);

		auto const& mat = m_materials(0);

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
		// Reset simulation state
		m_clock = 0;
		m_diag.Reset();

		// Clean up ground plane visual from previous scene
		m_ground_gfx = nullptr;

		// Clear the broadphase before modifying bodies. The broadphase holds raw
		// pointers to RigidBody objects, which become invalid if the vector resizes.
		m_physics.Broadphase().Clear();

		// Clear existing bodies and owned shapes
		m_body.resize(0);
		m_shape_buffer.resize(0);

		// Apply gravity from the scene file
		m_gravity = scene_desc.gravity;

		// Set the kill zone well below the ground plane. Bodies that fall below
		// this height are frozen to prevent them from corrupting the simulation.
		m_kill_zone_height = (scene_desc.ground ? scene_desc.ground->height : 0) - 50.0f;

		// Apply material properties from the scene file
		auto& mat = m_materials(0);
		mat.m_elasticity_norm = scene_desc.elasticity;
		mat.m_elasticity_tang = 0.0f;
		mat.m_elasticity_tors = 0.0f;
		mat.m_friction_static = scene_desc.friction;

		// Count total bodies: scene bodies + optional ground plane body
		auto num_scene_bodies = static_cast<int>(scene_desc.bodies.size());
		auto total_bodies = num_scene_bodies + (scene_desc.ground ? 1 : 0);
		auto scene_bbox = CalculateSceneBBox(scene_desc);
		const auto ground_thickness = 10.0f;

		// Shapes for the bodies in the scene.
		{
			m_shape_buffer.reserve(total_bodies * 512);
			for (auto const& bd : scene_desc.bodies)
			{
				auto ofs = m_shape_buffer.size();
				switch (bd.shape_type)
				{
					case scene_loader::BodyDesc::EShape::Box:
					{
						m_shape_buffer.push_back(collision::ShapeBox(bd.box_dimensions));
						break;
					}
					case scene_loader::BodyDesc::EShape::Sphere:
					{
						m_shape_buffer.push_back(collision::ShapeSphere(bd.sphere_radius));
						break;
					}
					case scene_loader::BodyDesc::EShape::Line:
					{
						m_shape_buffer.push_back(collision::ShapeLine(bd.line_length, bd.line_thickness));
						break;
					}
					case scene_loader::BodyDesc::EShape::Triangle:
					{
						m_shape_buffer.push_back(collision::ShapeTriangle(bd.tri_verts[0], bd.tri_verts[1], bd.tri_verts[2]));
						break;
					}
					case scene_loader::BodyDesc::EShape::Polytope:
					{
						m_shape_buffer.push_back(collision::BuildPolytopeFromPoints(bd.polytope_verts));
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
				m_shape_buffer.pad_to(16);
				m_shape_buffer.at_byte_ofs<collision::Shape>(ofs).m_size = m_shape_buffer.size() - ofs;
			}

			// Create a collision shape for the ground plane
			if (scene_desc.ground)
			{
				// Create the ground plane body as a large thin box with infinite mass.
				auto ground_half_extent = 10.0f * Length(scene_bbox.Radius().xy);
				m_shape_buffer.push_back(collision::ShapeBox(v4{ ground_half_extent, ground_half_extent, ground_thickness, 0 }));
			}
		}

		// Bodies from the scene description.
		{
			auto shape_ptr = m_shape_buffer.data<collision::Shape>();

			// Phase 1: Create bodies WITHOUT the renderer so the ShapeChange handler doesn't
			// try to create graphics yet. This avoids dangling pointer issues during the
			// construction loop (graphics creation calls AddShape which reads the shape data).
			m_body.reserve(total_bodies);
			for (auto const& bd : scene_desc.bodies)
			{
				Body body(nullptr);
				body.O2W(m4x4::Translation(bd.position));
				body.Shape(shape_ptr, bd.mass);
				body.VelocityWS(bd.angular_velocity, bd.velocity);
				m_body.push_back(std::move(body));

				shape_ptr = collision::next(shape_ptr);
			}

			// Create the ground plane body as a large thin box with infinite mass.
			// The box is thin in Z (0.5 units) and wide in XY, centred at the ground height.
			if (scene_desc.ground)
			{
				Body ground(nullptr);
				ground.O2W(m4x4::Translation(v4{ 0, 0, scene_desc.ground->height - 0.5f * ground_thickness, 1 }));
				ground.Shape(shape_ptr, -1.0f);
				m_body.push_back(std::move(ground));

				shape_ptr = collision::next(shape_ptr);
			}
		}

		// Create the graphics now that all bodies and shapes are stable in memory.
		// Build a single LDraw script containing all body shapes, then parse it in one
		// call. This is dramatically faster than parsing each body individually because
		// it amortises renderer overhead (shader cache lookups, resource pool, etc.).
		if (m_rdr)
		{
			using namespace pr::ldraw;
			static std::default_random_engine rng;

			// Build a single script with one root object per body
			Builder builder;
			for (auto const& [bd, i] : with_index(scene_desc.bodies))
			{
				auto& body = m_body[i];
				if (!body.HasShape())
					continue;

				auto colour = bd.colour ? *bd.colour : RandomRGB(rng, 0.0f, 1.0f);
				builder.Add<LdrRigidBody>("Body", colour.argb).rigid_body(body);
			}

			// Add ground plane shape
			if (scene_desc.ground)
			{
				auto& body = m_body.back();
				auto colour = scene_desc.ground->colour ? *scene_desc.ground->colour : RandomRGB(rng, 0.0f, 1.0f);
				builder.Add<LdrRigidBody>("Body", colour.argb).rigid_body(body);
			}

			// Parse all shapes in one batch
			auto result = rdr12::ldraw::Parse(*m_rdr, builder.ToBinary());

			// Assign each parsed object to its corresponding body
			int obj_idx = 0;
			for (auto const& [bd, i] : with_index(scene_desc.bodies))
			{
				auto& body = m_body[i];
				if (!body.HasShape())
					continue;

				if (obj_idx < static_cast<int>(result.m_objects.size()))
					body.m_gfx = result.m_objects[obj_idx++];

				body.UpdateGfx();
			}

			// Assign ground plane graphics
			if (scene_desc.ground && obj_idx < static_cast<int>(result.m_objects.size()))
			{
				m_body.back().m_gfx = result.m_objects[obj_idx++];
			}
		}

		// Rebuild the broadphase with the new bodies
		m_physics.Broadphase().Clear();
		for (int i = 0; i != std::ssize(m_body); ++i)
			m_physics.Broadphase().Add(m_body[i]);

		// Logging
		{
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
			//m_scenario = static_cast<EScenario>(s);
			Reset();

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

	// Export the scene as LDraw script
	void Scene::Dump()
	{
		using namespace pr::ldraw;

		ldraw::Builder builder;
		builder.Add<LdrRigidBody>("body0", 0x8000FF00).rigid_body(m_body[0]);
		builder.Add<LdrRigidBody>("body1", 0x10FF0000).rigid_body(m_body[1]);
		builder.Save(L"dump\\physics_dump.ldr");
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
