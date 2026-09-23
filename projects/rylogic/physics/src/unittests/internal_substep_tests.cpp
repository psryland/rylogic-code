//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// Acceptance tests for frame-wide GPU substep scheduling and gathered output.

#if PR_UNITTESTS
#include "pr/common/unittests.h"
#include "pr/physics/physics.h"
#include "src/unittests/shared_engine.h"

namespace pr::physics::tests
{
	namespace
	{
		// Construct a dynamic sphere with explicit world transform and velocity.
		RigidBody MakeSubstepSphere(collision::ShapeSphere const& shape, m4x4 const& o2w = m4x4::Identity(), v8motion const& velocity = {})
		{
			auto body = RigidBody{&shape, o2w, Inertia::Sphere(shape.m_radius, 1.0f)};
			body.VelocityWS(velocity);
			return body;
		}

		// Compare all externally visible dynamic state produced by two stepping schedules.
		void ExpectSubstepBodyState(RigidBody const& lhs, RigidBody const& rhs, float tolerance)
		{
			auto const lhs_o2w = lhs.O2W();
			auto const rhs_o2w = rhs.O2W();
			PR_EXPECT(FEqlAbsolute(lhs_o2w.pos, rhs_o2w.pos, tolerance));
			PR_EXPECT(FEqlAbsolute(lhs_o2w.x, rhs_o2w.x, tolerance));
			PR_EXPECT(FEqlAbsolute(lhs_o2w.y, rhs_o2w.y, tolerance));
			PR_EXPECT(FEqlAbsolute(lhs_o2w.z, rhs_o2w.z, tolerance));
			PR_EXPECT(FEqlAbsolute(lhs.MomentumWS().ang, rhs.MomentumWS().ang, tolerance));
			PR_EXPECT(FEqlAbsolute(lhs.MomentumWS().lin, rhs.MomentumWS().lin, tolerance));
			PR_EXPECT(lhs.Sleeping() == rhs.Sleeping());
		}

		// Return a hard locked constraint axis with a bounded corrective force.
		ConstraintAxisDesc MakeSubstepLockedAxis()
		{
			auto axis = ConstraintAxisDesc{};
			axis.m_mode = EConstraintAxisMode::Locked;
			axis.m_max_force = 10000.0f;
			return axis;
		}

		// Build isolated sphere pairs that first overlap on successive internal substeps.
		std::vector<RigidBody> MakeTimedCollisionPairs(collision::ShapeSphere const& shape)
		{
			auto bodies = std::vector<RigidBody>{};
			bodies.reserve(8);
			for (int pair_index = 0; pair_index != 4; ++pair_index)
			{
				auto const separation = 1.15f + 0.2f * pair_index;
				auto const y = 3.0f * pair_index;
				bodies.push_back(MakeSubstepSphere(shape, m4x4::Translation(-0.5f * separation, y, 0), v8motion{v4::Zero(), v4{+1, 0, 0, 0}}));
				bodies.push_back(MakeSubstepSphere(shape, m4x4::Translation(+0.5f * separation, y, 0), v8motion{v4::Zero(), v4{-1, 0, 0, 0}}));
			}
			return bodies;
		}

		// Convert a contiguous body collection into the pointer range accepted by Engine.
		std::vector<RigidBody*> SubstepBodyPointers(std::vector<RigidBody>& bodies)
		{
			auto pointers = std::vector<RigidBody*>{};
			pointers.reserve(bodies.size());
			for (auto& body : bodies)
				pointers.push_back(&body);
			return pointers;
		}
	}

	// Prove internal substeps preserve the one-submit, one-wait, one-readback frame contract.
	PRUnitTestClass(InternalSubstepTests)
	{
		// Report the detector's idle-frame overhead against the same Debug workload with refresh disabled.
		PRUnitTestMethod(SelectiveRefreshIdleCost, Extended)
		{
			// Keep one awake sphere in ground contact while toggling only the configured refresh passes.
			auto& engine = SharedEngine();
			auto sphere_shape = collision::ShapeSphere{0.5f};
			auto ground_shape = collision::ShapeBox{v4{10.0f, 10.0f, 1.0f, 0.0f}};
			auto wall_ms = std::array<double, 2>{};
			auto gpu_ms = std::array<double, 2>{};
			auto recorded_ms = std::array<double, 2>{};
			auto active_frames = std::array<int, 2>{};
			auto contact_frames = std::array<int, 2>{};
			for (int round = 0; round != 6; ++round)
			{
				// Alternate configurations to reduce run-order and device warm-up bias.
				auto passes = round % 2;
				ResetEngineForNextTest(engine);
				auto config = engine.Config();
				config.selective_refresh_passes = passes;
				config.selective_refresh_support_only = false;
				engine.Config(config);
				auto sphere = RigidBody{&sphere_shape, m4x4::Translation(0.0f, 0.0f, 0.499f), Inertia::Sphere(0.5f, 1.0f)};
				auto ground = RigidBody{&ground_shape, m4x4::Translation(0.0f, 0.0f, -0.5f), Inertia::Infinite()};
				sphere.NeverSleep(true);
				sphere.GravityWS(v4{0.0f, 0.0f, -9.81f, 0.0f});
				auto bodies = std::array<RigidBody*, 2>{&sphere, &ground};
				for (int frame = 0; frame != 64; ++frame)
				{
					// Replay a shallow corrected contact; exclude the first four frames while GPU resources settle.
					sphere.O2W(m4x4::Translation(0.0f, 0.0f, 0.499f));
					sphere.VelocityWS({});
					auto start = std::chrono::steady_clock::now();
					engine.Step(Engine::StepInput{.m_bodies = bodies, .m_elapsed_seconds = 1.0f / 60.0f});
					auto elapsed_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
					if (frame < 4)
						continue;

					wall_ms[passes] += elapsed_ms;
					gpu_ms[passes] += engine.LastStepProfile().m_gpu_run_ms;
					recorded_ms[passes] += engine.LastStepProfile().m_selective_ms;
					active_frames[passes] += engine.LastStepProfile().m_selective_refresh_pass_count != 0;
					contact_frames[passes] += engine.LastCollisionStats().m_contact_count != 0;
				}
			}
			for (int passes = 0; passes != 2; ++passes)
			{
				// Publish the comparative measurement without imposing a machine-dependent timing assertion.
				std::printf("selective_idle passes=%d mean_step_ms=%.3f mean_gpu_ms=%.3f mean_record_ms=%.3f active_frames=%d/180 contact_frames=%d/180\n",
					passes, wall_ms[passes] / 180, gpu_ms[passes] / 180, recorded_ms[passes] / 180, active_frames[passes], contact_frames[passes]);
				PR_EXPECT(active_frames[passes] == 0);
				PR_EXPECT(contact_frames[passes] > 0);
			}
		}

		// Admit support contacts between dynamic bodies without admitting equally shallow isolated ground contact.
		PRUnitTestMethod(SelectiveRefreshDistinguishesStackedSupport, Quick)
		{
			// Isolate the depth criterion from closing velocity and from the main position solve.
			auto& engine = SharedEngine();
			ResetEngineForNextTest(engine);
			auto config = engine.Config();
			config.max_collision_pairs = 16;
			config.push_out_iterations = 0;
			config.velocity_baumgarte = 0.0f;
			config.deep_penetration_baumgarte_min = 0.0f;
			config.deep_penetration_baumgarte_max = 0.0f;
			config.selective_refresh_closing_speed_slop = 100.0f;
			engine.Config(config);
			auto shape = collision::ShapeSphere{0.5f};
			auto ground_shape = collision::ShapeBox{v4{10.0f, 10.0f, 1.0f, 0.0f}};
			auto lower = RigidBody{&shape, m4x4::Translation(0.0f, 0.0f, 0.5f), Inertia::Sphere(0.5f, 1.0f)};
			auto upper = RigidBody{&shape, m4x4::Translation(0.0f, 0.0f, 1.4985f), Inertia::Sphere(0.5f, 1.0f)};
			auto ground = RigidBody{&ground_shape, m4x4::Translation(0.0f, 0.0f, -0.5f), Inertia::Infinite()};
			lower.GravityWS(v4{0.0f, 0.0f, -9.81f, 0.0f});
			upper.GravityWS(v4{0.0f, 0.0f, -9.81f, 0.0f});
			auto stack = std::array<RigidBody*, 3>{&lower, &upper, &ground};

			// A shallow dynamic-on-dynamic contact identifies a stack even below the in-pass 2 mm support slop.
			engine.Step(Engine::StepInput{.m_bodies = stack, .m_elapsed_seconds = 1.0f / 60.0f});
			PR_EXPECT(engine.LastStepProfile().m_selective_refresh_pass_count == 0);
			engine.Step(Engine::StepInput{.m_bodies = stack, .m_elapsed_seconds = 1.0f / 60.0f});
			PR_EXPECT(engine.LastStepProfile().m_selective_refresh_pass_count == 1);

			// An isolated support at the same depth must not activate the follow-up pass.
			ResetEngineForNextTest(engine);
			engine.Config(config);
			auto isolated = RigidBody{&shape, m4x4::Translation(0.0f, 0.0f, 0.4985f), Inertia::Sphere(0.5f, 1.0f)};
			isolated.GravityWS(v4{0.0f, 0.0f, -9.81f, 0.0f});
			auto pair = std::array<RigidBody*, 2>{&isolated, &ground};
			engine.Step(Engine::StepInput{.m_bodies = pair, .m_elapsed_seconds = 1.0f / 60.0f});
			engine.Step(Engine::StepInput{.m_bodies = pair, .m_elapsed_seconds = 1.0f / 60.0f});
			PR_EXPECT(engine.LastStepProfile().m_selective_refresh_pass_count == 0);

			// The current GPU contact count must enforce the configured limit even though host diagnostics reset before recording.
			ResetEngineForNextTest(engine);
			config.selective_refresh_contact_limit = 1;
			engine.Config(config);
			lower.O2W(m4x4::Translation(0.0f, 0.0f, 0.495f));
			upper.O2W(m4x4::Translation(0.0f, 0.0f, 1.49f));
			engine.Step(Engine::StepInput{.m_bodies = stack, .m_elapsed_seconds = 1.0f / 60.0f});
			PR_EXPECT(engine.LastCollisionStats().m_contact_count > 1);
			engine.Step(Engine::StepInput{.m_bodies = stack, .m_elapsed_seconds = 1.0f / 60.0f});
			PR_EXPECT(engine.LastStepProfile().m_selective_refresh_pass_count == 0);
		}

		// Verify the engine admits residual contacts one frame later, holds the gate across clean frames, and obeys reset and disable.
		PRUnitTestMethod(SelectiveRefreshAdmitsResidualAndExpires, Quick)
		{
			// Disable main push-out so an overlapping sphere provides a repeatable residual for the GPU detector.
			auto& engine = SharedEngine();
			ResetEngineForNextTest(engine);
			auto config = engine.Config();
			config.max_collision_pairs = 16;
			config.push_out_iterations = 0;
			config.position_baumgarte = 1.0f;
			config.velocity_baumgarte = 0.0f;
			config.deep_penetration_baumgarte_min = 0.0f;
			config.deep_penetration_baumgarte_max = 0.0f;
			config.warm_start_scale = 0.0f;
			config.selective_refresh_passes = 1;
			config.selective_refresh_position_iterations = 1;
			config.selective_refresh_support_only = false;
			engine.Config(config);
			auto sphere_shape = collision::ShapeSphere{0.5f};
			auto ground_shape = collision::ShapeBox{v4{10.0f, 10.0f, 1.0f, 0.0f}};
			auto sphere = RigidBody{&sphere_shape, m4x4::Translation(0.0f, 0.0f, 0.4f), Inertia::Sphere(0.5f, 1.0f)};
			auto ground = RigidBody{&ground_shape, m4x4::Translation(0.0f, 0.0f, -0.5f), Inertia::Infinite()};
			auto bodies = std::array<RigidBody*, 2>{&sphere, &ground};
			auto step = [&]()
			{
				// Keep the physical state fixed so only the detection gate changes the recorded work.
				sphere.O2W(m4x4::Translation(0.0f, 0.0f, 0.4f));
				sphere.VelocityWS({});
				engine.Step(Engine::StepInput{.m_bodies = bodies, .m_elapsed_seconds = 1.0f / 60.0f});
				PR_EXPECT(engine.LastStepProfile().m_submission_count == 1);
				PR_EXPECT(engine.LastStepProfile().m_readback_copy_count == 1);
			};

			// Detection uses the completed readback, not an extra GPU wait or first-frame refresh.
			step();
			PR_EXPECT(engine.LastStepProfile().m_selective_refresh_pass_count == 0);
			step();
			PR_EXPECT(engine.LastStepProfile().m_selective_refresh_pass_count == 1);

			// Four clean frames exhaust the hold; the next residual again waits until the following frame.
			for (int clean = 0; clean != 4; ++clean)
			{
				sphere.O2W(m4x4::Translation(0.0f, 0.0f, 2.0f));
				sphere.VelocityWS({});
				engine.Step(Engine::StepInput{.m_bodies = bodies, .m_elapsed_seconds = 1.0f / 60.0f});
				PR_EXPECT(engine.LastCollisionStats().m_contact_count == 0);
			}
			step();
			PR_EXPECT(engine.LastStepProfile().m_selective_refresh_pass_count == 0);

			// Reset clears pending admission even when the preceding frame found a residual.
			engine.ResetCaches();
			step();
			PR_EXPECT(engine.LastStepProfile().m_selective_refresh_pass_count == 0);

			// Configuration remains authoritative for both the detector and the follow-up pass.
			config.selective_refresh_passes = 0;
			engine.Config(config);
			step();
			step();
			PR_EXPECT(engine.LastStepProfile().m_selective_refresh_pass_count == 0);

			// A contact corrected by the main solve has no substantial residual and should not keep refresh alive.
			engine.ResetCaches();
			config.selective_refresh_passes = 1;
			config.push_out_iterations = 4;
			engine.Config(config);
			step();
			PR_EXPECT(sphere.O2W().pos.z > 0.49f);
			step();
			PR_EXPECT(engine.LastStepProfile().m_selective_refresh_pass_count == 0);
		}

		// Verify callback identity, time partitioning, and GPU boundary counts for the required substep counts.
		PRUnitTestMethod(OneSubmissionAndReadbackPerFrame, Quick)
		{
			auto shape = collision::ShapeSphere{0.25f};
			auto& engine = SharedEngine();
			for (auto const substep_count : std::array{1, 2, 4, 8})
			{
				ResetEngineForNextTest(engine);
				auto body = MakeSubstepSphere(shape, m4x4::Translation(0, 0, 1));
				auto bodies = std::array<RigidBody*, 1>{&body};
				auto callback_count = 0;
				auto const elapsed_seconds = 1.0f / 30.0f;
				auto const frame_time_s = 17.25;
				auto const substep_dt = elapsed_seconds / substep_count;

				// Each force callback must describe the precise interval whose commands it records.
				engine.ExternalForces += [&](Engine&, Engine::ExternalForceArgs const& args)
				{
					PR_EXPECT(args.m_body_count == 1);
					PR_EXPECT(args.m_bodies != nullptr);
					PR_EXPECT(args.m_substep_index == callback_count);
					PR_EXPECT(args.m_substep_count == substep_count);
					PR_EXPECT(args.m_dt == substep_dt);
					PR_EXPECT(std::abs(args.m_time_s - (frame_time_s + static_cast<double>(substep_dt) * callback_count)) < 1.0e-12);
					++callback_count;
				};

				engine.Step(Engine::StepInput{
					.m_bodies = bodies,
					.m_elapsed_seconds = elapsed_seconds,
					.m_substep_count = substep_count,
					.m_time_s = frame_time_s,
				});

				auto const& profile = engine.LastStepProfile();
				PR_EXPECT(callback_count == substep_count);
				PR_EXPECT(profile.m_substep_count == substep_count);
				PR_EXPECT(profile.m_submission_count == 1);
				PR_EXPECT(profile.m_wait_count == 1);
				PR_EXPECT(profile.m_readback_copy_count == 1);
				PR_EXPECT(engine.LastCollisionStats().m_event_capacity == 0);
			}
		}

		// Verify a pre-submit callback failure retires recorded GPU work and leaves the engine reusable.
		PRUnitTestMethod(RecordingFailureRetiresGpuWork, Extended)
		{
			auto shape = collision::ShapeSphere{0.25f};
			auto body = MakeSubstepSphere(shape, m4x4::Translation(0, 0, 1));
			auto bodies = std::array<RigidBody*, 1>{&body};
			auto& engine = SharedEngine();
			ResetEngineForNextTest(engine);
			auto callback_count = 0;
			engine.ExternalForces += [&](Engine&, Engine::ExternalForceArgs const& args)
			{
				++callback_count;
				if (args.m_substep_index == 1)
					throw std::runtime_error("Intentional later-substep recording failure");
			};

			// The failed frame must retire one complete substep before rejecting the next recorded pass.
			PR_THROWS(engine.Step(Engine::StepInput{
				.m_bodies = bodies,
				.m_elapsed_seconds = 1.0f / 60.0f,
				.m_substep_count = 2,
			}), std::runtime_error);
			PR_EXPECT(callback_count == 2);

			// A clean frame proves command-list, resource-state, and engine pending-state recovery.
			engine.ExternalForces.reset();
			engine.Step(Engine::StepInput{
				.m_bodies = bodies,
				.m_elapsed_seconds = 1.0f / 60.0f,
				.m_substep_count = 2,
			});
			PR_EXPECT(engine.LastStepProfile().m_submission_count == 1);
			PR_EXPECT(engine.LastStepProfile().m_wait_count == 1);
			PR_EXPECT(engine.LastStepProfile().m_readback_copy_count == 1);
		}

		// A collision observer that rejects publication must not leave its solved contact impulses available to the retry.
		PRUnitTestMethod(CompletionFailureInvalidatesContactWarmStart, Extended)
		{
			auto shape = collision::ShapeSphere{0.5f};
			auto const initial_a = MakeSubstepSphere(shape, m4x4::Translation(-0.4f, 0, 0), v8motion{v4::Zero(), +v4::XAxis()});
			auto const initial_b = MakeSubstepSphere(shape, m4x4::Translation(+0.4f, 0, 0), v8motion{v4::Zero(), -v4::XAxis()});
			auto retry_a = initial_a;
			auto retry_b = initial_b;
			auto retry_bodies = std::array<RigidBody*, 2>{&retry_a, &retry_b};
			auto& engine = SharedEngine();
			ResetEngineForNextTest(engine);
			engine.Collisions += [](Engine&, std::span<RbContact const>)
			{
				throw std::runtime_error("Intentional pre-publication collision rejection");
			};

			PR_THROWS(engine.Step(Engine::StepInput{
				.m_bodies = retry_bodies,
				.m_elapsed_seconds = 1.0f / 60.0f,
			}), std::runtime_error);
			engine.Collisions.reset();
			engine.Step(Engine::StepInput{
				.m_bodies = retry_bodies,
				.m_elapsed_seconds = 1.0f / 60.0f,
			});

			// A fresh cache and identical initial state define the retry result when no rejected warm start leaks through.
			auto reference_a = initial_a;
			auto reference_b = initial_b;
			auto reference_bodies = std::array<RigidBody*, 2>{&reference_a, &reference_b};
			ResetEngineForNextTest(engine);
			engine.Step(Engine::StepInput{
				.m_bodies = reference_bodies,
				.m_elapsed_seconds = 1.0f / 60.0f,
			});
			ExpectSubstepBodyState(retry_a, reference_a, 1.0e-6f);
			ExpectSubstepBodyState(retry_b, reference_b, 1.0e-6f);
		}

		// Preserve rigid-only contact warm starting when its cache lookup is fused into the colour-batched application pass.
		PRUnitTestMethod(RigidOnlyWarmStartLoadsOnApply, Extended)
		{
			auto shape = collision::ShapeSphere{0.5f};
			auto const initial_o2w_a = m4x4::Translation(-0.4f, 0, 0);
			auto const initial_o2w_b = m4x4::Translation(+0.4f, 0, 0);
			auto const initial_velocity_a = v8motion{v4::Zero(), +v4::XAxis()};
			auto const initial_velocity_b = v8motion{v4::Zero(), -v4::XAxis()};
			auto body_a = MakeSubstepSphere(shape, initial_o2w_a, initial_velocity_a);
			auto body_b = MakeSubstepSphere(shape, initial_o2w_b, initial_velocity_b);
			auto bodies = std::array<RigidBody*, 2>{&body_a, &body_b};
			auto& engine = SharedEngine();
			ResetEngineForNextTest(engine);
			auto config = EngineConfig{};
			config.sleeping_enabled = false;
			config.selective_refresh_passes = 0;
			config.push_out_iterations = 0;
			config.solver_iterations = 1;
			config.velocity_baumgarte = 0.0f;
			config.warm_start_scale = 1.0f;
			engine.Config(config);

			// Populate the previous-frame cache with the impulse needed to separate the overlapping equal-mass pair.
			engine.Step(Engine::StepInput{
				.m_bodies = bodies,
				.m_elapsed_seconds = 1.0f / 60.0f,
			});

			// Replay the same contact with iterative solving disabled so any response must come from the retained rigid-only warm start.
			body_a.O2W(initial_o2w_a);
			body_b.O2W(initial_o2w_b);
			body_a.VelocityWS(initial_velocity_a);
			body_b.VelocityWS(initial_velocity_b);
			config.solver_iterations = 0;
			engine.Config(config);
			engine.Step(Engine::StepInput{
				.m_bodies = bodies,
				.m_elapsed_seconds = 1.0f / 60.0f,
			});
			PR_EXPECT(body_a.VelocityWS().lin.x < 0.0f);
			PR_EXPECT(body_b.VelocityWS().lin.x > 0.0f);
		}

		// Verify immutable CPU-authored frame forces are restored before every internal integration pass.
		PRUnitTestMethod(FrameForcesMatchRepeatedExternalSteps, Quick)
		{
			auto shape = collision::ShapeSphere{0.35f};
			auto& engine = SharedEngine();
			auto const elapsed_seconds = 1.0f / 20.0f;
			auto const force = v4{3.0f, -2.0f, 5.0f, 0};
			auto const torque = v4{0.6f, -0.4f, 0.2f, 0};

			for (auto const substep_count : std::array{1, 2, 4, 8})
			{
				// Run the frame as one GPU-resident substep sequence.
				ResetEngineForNextTest(engine);
				auto internal = MakeSubstepSphere(shape, m4x4::Translation(0.2f, -0.3f, 0.7f), v8motion{v4{0.2f, -0.1f, 0.3f, 0}, v4{-0.4f, 0.5f, 0.1f, 0}});
				internal.ApplyForceWS(force, torque, internal.CentreOfMassOffsetWS());
				auto internal_bodies = std::array<RigidBody*, 1>{&internal};
				engine.Step(Engine::StepInput{
					.m_bodies = internal_bodies,
					.m_elapsed_seconds = elapsed_seconds,
					.m_substep_count = substep_count,
				});

				// Use separate submitted steps as the reference schedule, reapplying the same frame-constant force each time.
				ResetEngineForNextTest(engine);
				auto external = MakeSubstepSphere(shape, m4x4::Translation(0.2f, -0.3f, 0.7f), v8motion{v4{0.2f, -0.1f, 0.3f, 0}, v4{-0.4f, 0.5f, 0.1f, 0}});
				auto external_bodies = std::array<RigidBody*, 1>{&external};
				auto const substep_dt = elapsed_seconds / substep_count;
				for (int substep_index = 0; substep_index != substep_count; ++substep_index)
				{
					external.ApplyForceWS(force, torque, external.CentreOfMassOffsetWS());
					engine.Step(substep_dt, external_bodies);
				}

				ExpectSubstepBodyState(internal, external, 2.0e-5f);
			}
		}

		// Verify frame collision output is grouped monotonically by the substep that generated each contact.
		PRUnitTestMethod(CollisionEventsRetainSubstepOrder, Quick)
		{
			auto shape = collision::ShapeSphere{0.5f};
			auto bodies = MakeTimedCollisionPairs(shape);
			auto body_ptrs = SubstepBodyPointers(bodies);
			auto substep_indices = std::vector<int>{};
			auto retained_contacts = std::vector<RbContact>{};
			auto& engine = SharedEngine();
			ResetEngineForNextTest(engine);
			engine.Collisions += [&](Engine&, std::span<RbContact const> contacts)
			{
				for (auto const& contact : contacts)
				{
					substep_indices.push_back(contact.m_substep_index);
					retained_contacts.push_back(contact);
					auto const point = contact.Point();
					auto const expected_point_at_t = point + 0.5f * contact.m_time * contact.m_velocity.LinAt(point);
					PR_EXPECT(FEqlAbsolute(contact.m_point_at_t, expected_point_at_t, 1.0e-5f));
					PR_EXPECT(IsFinite(contact.m_b2a.x) && IsFinite(contact.m_b2a.y) && IsFinite(contact.m_b2a.z) && IsFinite(contact.m_b2a.pos));
					PR_EXPECT(IsFinite(contact.m_velocity.ang) && IsFinite(contact.m_velocity.lin));

					// Equal-radius spheres expose the generating frame independently of later solver pose corrections.
					PR_EXPECT(FEqlAbsolute(Length(contact.m_b2a.pos.w0()) + contact.m_depth, 1.0f, 1.0e-5f));
					PR_EXPECT(FEqlAbsolute(point, (0.5f * contact.m_b2a.pos).w1(), 1.0e-5f));
				}
			};

			engine.Step(Engine::StepInput{
				.m_bodies = body_ptrs,
				.m_elapsed_seconds = 0.4f,
				.m_substep_count = 4,
			});

			PR_EXPECT(!substep_indices.empty());
			PR_EXPECT(std::ranges::is_sorted(substep_indices));
			for (int substep_index = 0; substep_index != 4; ++substep_index)
				PR_EXPECT(std::ranges::find(substep_indices, substep_index) != substep_indices.end());

			// Velocity retains final-substep state; the generating geometric frame above must not be replaced by the corrected final pose.
			auto const& final_contact = retained_contacts.back();
			PR_EXPECT(final_contact.m_substep_index == 3);
			auto const final_b2a = InvertOrthonormal(final_contact.m_objA->O2W()) * final_contact.m_objB->O2W();
			auto const velocity_a = Shift(final_contact.m_objA->VelocityOS(), -final_contact.m_objA->CentreOfMassOS());
			auto const velocity_b = Shift(final_contact.m_objB->VelocityOS(), -final_contact.m_objB->CentreOfMassOS());
			auto const expected_velocity = final_b2a * velocity_b - velocity_a;
			PR_EXPECT(FEqlAbsolute(final_contact.m_velocity.ang, expected_velocity.ang, 1.0e-5f));
			PR_EXPECT(FEqlAbsolute(final_contact.m_velocity.lin, expected_velocity.lin, 1.0e-5f));
		}

		// Verify a bounded event queue reports the first overflowing substep instead of overwriting retained records.
		PRUnitTestMethod(CollisionEventOverflowIsBounded, Quick)
		{
			auto shape = collision::ShapeSphere{0.5f};
			auto bodies = MakeTimedCollisionPairs(shape);
			auto body_ptrs = SubstepBodyPointers(bodies);
			auto& engine = SharedEngine();
			ResetEngineForNextTest(engine);
			auto config = engine.Config();
			config.max_collision_events = 2;
			engine.Config(config);
			auto retained_event_count = 0;
			engine.Collisions += [&](Engine&, std::span<RbContact const> contacts)
			{
				retained_event_count += isize(contacts);
			};

			engine.Step(Engine::StepInput{
				.m_bodies = body_ptrs,
				.m_elapsed_seconds = 0.4f,
				.m_substep_count = 4,
			});

			auto const& stats = engine.LastCollisionStats();
			PR_EXPECT(retained_event_count == 2);
			PR_EXPECT(stats.m_event_count == 2);
			PR_EXPECT(stats.m_event_capacity == 2);
			PR_EXPECT(stats.EventLimitReached());
			PR_EXPECT(stats.m_event_overflow_substep >= 0);
			PR_EXPECT(stats.m_event_overflow_substep < 3);
		}

		// Verify persistent constraints compile and solve on every internal substep without injecting split-correction momentum.
		PRUnitTestMethod(ConstraintsPersistAcrossInternalSubsteps, Quick)
		{
			auto shape = collision::ShapeSphere{0.25f};
			auto body = MakeSubstepSphere(shape, m4x4::Translation(0.5f, 0, 0));
			auto desc = D6ConstraintDesc{};
			desc.m_frame_a.m_body = BodyRef::Rigid(body);
			desc.m_frame_b.m_body = BodyRef::World();
			desc.m_linear[0] = MakeSubstepLockedAxis();
			auto constraints = ConstraintSet{};
			constraints.Add(desc);
			auto bodies = std::array<RigidBody*, 1>{&body};
			auto const momentum_before = body.MomentumWS();
			auto& engine = SharedEngine();
			ResetEngineForNextTest(engine);

			engine.Step(Engine::StepInput{
				.m_bodies = bodies,
				.m_constraints = &constraints,
				.m_elapsed_seconds = 1.0f / 30.0f,
				.m_substep_count = 8,
			});

			PR_EXPECT(body.O2W().pos.x < 0.5f);
			PR_EXPECT(FEqlAbsolute(body.MomentumWS().ang, momentum_before.ang, 1.0e-6f));
			PR_EXPECT(FEqlAbsolute(body.MomentumWS().lin, momentum_before.lin, 1.0e-6f));
			PR_EXPECT(engine.LastStepProfile().m_submission_count == 1);
		}

		// Verify sleep timers advance once per internal substep while state remains GPU-resident.
		PRUnitTestMethod(SleepStateAdvancesAcrossInternalSubsteps, Quick)
		{
			auto shape = collision::ShapeSphere{0.25f};
			auto body = MakeSubstepSphere(shape, m4x4::Translation(0, 0, 1));
			auto bodies = std::array<RigidBody*, 1>{&body};
			auto& engine = SharedEngine();
			ResetEngineForNextTest(engine);

			engine.Step(Engine::StepInput{
				.m_bodies = bodies,
				.m_elapsed_seconds = 61.0f / 60.0f,
				.m_substep_count = 61,
			});

			PR_EXPECT(body.Sleeping());
			PR_EXPECT(engine.LastStepProfile().m_submission_count == 1);
			PR_EXPECT(engine.LastStepProfile().m_readback_copy_count == 1);
		}
	};
}
#endif
