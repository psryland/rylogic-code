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
			auto& engine = SharedEngine();
			ResetEngineForNextTest(engine);
			engine.Collisions += [&](Engine&, std::span<RbContact const> contacts)
			{
				for (auto const& contact : contacts)
					substep_indices.push_back(contact.m_substep_index);
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
