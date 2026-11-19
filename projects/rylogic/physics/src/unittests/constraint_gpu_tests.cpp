//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************

#if PR_UNITTESTS
#include "pr/common/unittests.h"
#include "pr/physics/physics.h"
#include "src/constraint/constraint_gpu.h"
#include "src/compute/constraint_transaction.hlsli"
#include "src/compute/constraint_solver_gpu.h"
#include "src/compute/coupled_constraint_work.hlsli"
#include "src/unittests/shared_gpu.h"

namespace pr::physics::tests
{
	namespace
	{
		// Build one translated child link for exercising the coupled constraint packing lane.
		std::pair<Articulation, LinkHandle> MakeConstraintGpuTree()
		{
			auto const link = ArticulationLinkDesc{
				.m_inertia = Inertia::Sphere(0.25f, 1.0f),
			};
			auto builder = ArticulationBuilder{};
			auto const root = builder.AddFloatingRoot(link);
			auto const child = builder.AddLink(root, ArticulationJointDesc::Revolute(v4::ZAxis(), m4x4::Translation(1.0f, 0.0f, 0.0f)), link);
			return {builder.Build(), child};
		}

		// Create one dynamic identity-pose endpoint for direct production transaction diagnostics.
		GpuRigidBody MakeTransactionBody(RigidBody& body, int index)
		{
			// Keep owner identity in the native body, while each fixture supplies its explicit GPU mobility and physical input.
			body.SetMassProperties(Inertia::Sphere(1.0f, 1.0f));
			body.O2W(m4x4::Identity());
			auto packed = PackDynamics(body, index);
			packed.state_flags = SetFlag(packed.state_flags, ERigidBodyStateFlags_ResetWarmStart, false);
			return packed;
		}
	}

	// Actual ordinary publication distinguishes provisional warm arithmetic from invalid accepted-epoch context.
	PRUnitTestClass(ConstraintRigidTransactionOutcomeTests)
	{
		// Finite valid context can have an unrepresentable provisional scale without creating a fatal solver error.
		PRUnitTestMethod(WarmScaleOverflowPreservesOwner, Extended)
		{
			// Establish a nonzero accepted cache through the real owner, with no body or descriptor reset on later uploads.
			auto body = RigidBody{};
			auto bodies = std::array{MakeTransactionBody(body, 0)};
			auto desc = ConstraintDesc{.m_frame_b = {.m_body = BodyRef::Rigid(body)}};
			desc.D6().m_linear[0] = ConstraintAxisDesc{.m_mode = EConstraintAxisMode::Driven, .m_target_velocity = 1};
			auto source = ConstraintSet{};
			source.Add(desc);
			auto native_bodies = std::array{&body};
			auto upload = PackGpuConstraints(source, BodyRemap(native_bodies));
			auto config = EngineConfig{};
			config.solver_iterations = 1;
			config.push_out_iterations = 0;
			auto& gpu = SharedTestGpu();
			auto solver = GpuConstraintSolver(gpu, config);
			auto epoch = GpuConstraintEpoch{.identity = uint2(1, 31), .timestep = 1.0f / 60.0f, .warm_start_factor = 1};
			auto records = std::array<GpuConstraintRigidRecord, 1>{};
			auto results = std::array<uint32_t, 1>{};
			solver.SolveTransactions(gpu.m_job, epoch, upload, bodies, records, results);
			PR_EXPECT(results[0] == GpuConstraintStage_Accepted);
			PR_EXPECT(bodies[0].momentum_lin.x == 1 && records[0].rows.impulses[0] == 1);
			auto accepted_body = bodies[0];
			auto accepted_record = records[0];

			// The ratio is within the existing warm window; only multiplying two finite values overflows.
			++epoch.identity.x;
			epoch.timestep *= 2;
			epoch.warm_start_factor = 2.0e38f;
			auto read_failure = [&]
			{
				// Read the actual public fatal-slot transport without another preparation clearing it.
				auto& job = gpu.m_job;
				auto data = job.m_readback.Alloc<GpuConstraintSolverState>(1);
				job.m_barriers.Transition(solver.FrameState(), D3D12_RESOURCE_STATE_COPY_SOURCE).Commit();
				job.m_cmd_list.CopyBufferRegion(data, solver.FrameState(), 0);
				job.Run();
				return data.ptr<GpuConstraintSolverState>()->failure_slot_plus_one;
			};
			solver.SolveTransactions(gpu.m_job, epoch, upload, bodies, records, results);
			auto failure = read_failure();
			log_stream() << "ordinary warm-scale overflow: outcome=" << results[0] << ", fatal_slot=" << failure << ", ratio=" << epoch.timestep / accepted_record.accepted.timestep << '\n';
			PR_EXPECT(results[0] == GpuConstraintStage_AdmissionRejected && failure == 0u);
			PR_EXPECT(memcmp(&accepted_body, &bodies[0], sizeof(GpuRigidBody)) == 0);
			PR_EXPECT(memcmp(&accepted_record, &records[0], sizeof(GpuConstraintRigidRecord)) == 0);

			// The same finite inputs with a reused accepted epoch now violate its fixed-dt contract and must remain fatal.
			epoch.identity = accepted_record.accepted.epoch;
			PR_THROWS(solver.SolveTransactions(gpu.m_job, epoch, upload, bodies, records, results), std::runtime_error);
			failure = read_failure();
			log_stream() << "ordinary changed accepted dt: outcome=" << results[0] << ", fatal_slot=" << failure << '\n';
			PR_EXPECT(results[0] == GpuConstraintStage_NumericalFailure && failure == 1u);
			PR_EXPECT(memcmp(&accepted_body, &bodies[0], sizeof(GpuRigidBody)) == 0);
			PR_EXPECT(memcmp(&accepted_record, &records[0], sizeof(GpuConstraintRigidRecord)) == 0);
		}
	};

	// Direct production owner buffers and dispatches, separate from the not-yet-migrated Engine sequence.
	PRUnitTestClass(ConstraintRigidTransactionGpuTests)
	{
		// An accepted iteration replaces a preparation-time reset hint with its own compatible cache.
		PRUnitTestMethod(AcceptedGuessesSurvivePreparationReset, Extended)
		{
			// A new source starts cold, but the second half-relaxed sweep must continue the first accepted half impulse.
			auto body = RigidBody{};
			auto bodies = std::array{MakeTransactionBody(body, 0)};
			auto desc = ConstraintDesc{.m_frame_b = {.m_body = BodyRef::Rigid(body)}};
			desc.D6().m_linear[0] = ConstraintAxisDesc{.m_mode = EConstraintAxisMode::Driven, .m_target_velocity = 1};
			auto set = ConstraintSet{};
			set.Add(desc);
			auto native_bodies = std::array{&body};
			auto upload = PackGpuConstraints(set, BodyRemap(native_bodies));
			auto config = EngineConfig{};
			config.solver_iterations = 2;
			config.constraint_relaxation = 0.5f;
			config.push_out_iterations = 0;
			auto& gpu = SharedTestGpu();
			auto solver = GpuConstraintSolver(gpu, config);
			auto epoch = GpuConstraintEpoch{.identity = uint2(1, 12), .timestep = 1.0f/60.0f, .warm_start_factor = 0.25f};
			auto records = std::array<GpuConstraintRigidRecord, 1>{};
			auto results = std::array<uint32_t, 1>{};
			solver.SolveTransactions(gpu.m_job, epoch, upload, bodies, records, results);
			PR_EXPECT(results[0] == GpuConstraintStage_Accepted);
			log_stream() << "Two half-relaxed owner iterations: momentum=" << bodies[0].momentum_lin.x << ", accepted coefficient=" << records[0].rows.impulses[0] << ", expected=.75\n";
			PR_EXPECT(bodies[0].momentum_lin.x == 0.75f);
			PR_EXPECT(records[0].rows.impulses[0] == 0.75f && records[0].response_b.lin.x == 0.75f);
		}

		// Recompilation without an upload must not reissue an invalidation already consumed by an accepted cache.
		PRUnitTestMethod(AcceptedGuessesSurviveRepeatedPreparation, Extended)
		{
			// Keep one uploaded source and one physical buffer through three preparations in the same epoch.
			auto body = RigidBody{};
			auto packed = MakeTransactionBody(body, 0);
			auto desc = ConstraintDesc{.m_frame_b = {.m_body = BodyRef::Rigid(body)}};
			desc.D6().m_linear[0] = ConstraintAxisDesc{.m_mode = EConstraintAxisMode::Driven, .m_target_velocity = 1};
			auto set = ConstraintSet{};
			set.Add(desc);
			auto native_bodies = std::array{&body};
			auto upload = PackGpuConstraints(set, BodyRemap(native_bodies));
			auto config = EngineConfig{};
			config.constraint_relaxation = 0.5f;
			config.push_out_iterations = 0;
			auto& gpu = SharedTestGpu();
			auto& job = gpu.m_job;
			auto solver = GpuConstraintSolver(gpu, config);
			auto epoch = GpuConstraintEpoch{.identity = uint2(1, 13), .timestep = 1.0f/60.0f, .warm_start_factor = 0.25f};
			auto r_bodies = D3DPtr<ID3D12Resource>{};
			auto body_readback = std::array<ReadbackAlloc, 3>{};
			auto record_readback = std::array<ReadbackAlloc, 3>{};
			auto result_readback = std::array<ReadbackAlloc, 3>{};
			try
			{
				// Only this initial upload establishes the endpoint reset input; later preparations see the same resource.
				r_bodies = gpu.CreateResource(compute::ResDesc::Buf<GpuRigidBody>(1, {}).usage(compute::EUsage::UnorderedAccess), job, "Physics:RepeatedPreparationBody");
				auto data = job.m_upload.Alloc<GpuRigidBody>(1);
				*data.ptr<GpuRigidBody>() = packed;
				job.m_barriers.Transition(r_bodies.get(), D3D12_RESOURCE_STATE_COPY_DEST).Commit();
				job.m_cmd_list.CopyBufferRegion(r_bodies.get(), 0, data);
				solver.Upload(job, upload);
				auto const iterations = std::array{2, 1, 1};
				for (int phase = 0; phase != 3; ++phase)
				{
					// Continue from the previously accepted total without an upload or a published warm operation.
					solver.PrepareVelocityTransactions(job, epoch);
					solver.CompileRows(job, epoch.timestep, 1, r_bodies);
					solver.PrepareVelocitySolve(job, epoch.timestep, 1, r_bodies);
					for (int iteration = 0; iteration != iterations[phase]; ++iteration)
					{
						// One accepted half-relaxed solve gives the next term of p = (p + 1)/2.
						solver.CommitVelocityTransactions(job, 1, r_bodies);
					}

					// Capture each phase before a later preparation can overwrite its result stream.
					auto output = solver.TransactionOutput();
					body_readback[phase] = job.m_readback.Alloc<GpuRigidBody>(1);
					record_readback[phase] = job.m_readback.Alloc<GpuConstraintRigidRecord>(1);
					result_readback[phase] = job.m_readback.Alloc<uint32_t>(1);
					job.m_barriers.Transition(r_bodies.get(), D3D12_RESOURCE_STATE_COPY_SOURCE);
					job.m_barriers.Transition(output.m_records, D3D12_RESOURCE_STATE_COPY_SOURCE);
					job.m_barriers.Transition(output.m_results, D3D12_RESOURCE_STATE_COPY_SOURCE);
					job.m_barriers.Commit();
					job.m_cmd_list.CopyBufferRegion(body_readback[phase], r_bodies.get(), 0);
					job.m_cmd_list.CopyBufferRegion(record_readback[phase], output.m_records, 0);
					job.m_cmd_list.CopyBufferRegion(result_readback[phase], output.m_results, 0);
				}
			}
			catch (...)
			{
				// Resource owners remain alive while a recording failure retires its valid prefix.
				auto error = std::current_exception();
				try
				{
					// Submission is outside this catch; only an interrupted recording reaches this retirement.
					job.RetireRecordedWork();
				}
				catch (std::exception const& cleanup_error)
				{
					// Preserve both the original failure and the cleanup diagnostic.
					std::cerr << "Repeated-preparation retirement failed: " << cleanup_error.what() << '\n';
				}
				std::rethrow_exception(error);
			}
			job.Run();

			// These exact binary fractions distinguish genuine continuation from both cold restart and historical warm scaling.
			auto const expected = std::array{0.75f, 0.875f, 0.9375f};
			for (int phase = 0; phase != 3; ++phase)
			{
				// Read actual publication, accepted coefficients, and accounted response, not prepared-row guesses.
				auto const& actual = *body_readback[phase].ptr<GpuRigidBody>();
				auto const& record = *record_readback[phase].ptr<GpuConstraintRigidRecord>();
				log_stream() << "No-Upload preparation " << phase << ": momentum=" << actual.momentum_lin.x << ", coefficient=" << record.rows.impulses[0] << ", expected=" << expected[phase] << '\n';
				PR_EXPECT(*result_readback[phase].ptr<uint32_t>() == GpuConstraintStage_Accepted);
				PR_EXPECT(actual.momentum_lin.x == expected[phase]);
				PR_EXPECT(record.rows.impulses[0] == expected[phase] && record.response_b.lin.x == expected[phase]);
			}
		}

		// Semantic invalidation stays pending through rejection and is acknowledged only by an accepted physical replacement.
		PRUnitTestMethod(InvalidationSurvivesRejectedUpload, Extended)
		{
			// Establish nonzero compatible support, so silently forgetting a later invalidation changes the under-relaxed result.
			auto body = RigidBody{};
			auto bodies = std::array{MakeTransactionBody(body, 0)};
			auto desc = ConstraintDesc{.m_frame_b = {.m_body = BodyRef::Rigid(body)}};
			desc.D6().m_linear[0] = ConstraintAxisDesc{.m_mode = EConstraintAxisMode::Driven, .m_target_velocity = 1};
			auto set = ConstraintSet{};
			auto handle = set.Add(desc);
			auto native_bodies = std::array{&body};
			auto upload = PackGpuConstraints(set, BodyRemap(native_bodies));
			auto config = EngineConfig{};
			config.solver_iterations = 2;
			config.constraint_relaxation = 0.5f;
			config.push_out_iterations = 0;
			auto& gpu = SharedTestGpu();
			auto solver = GpuConstraintSolver(gpu, config);
			auto epoch = GpuConstraintEpoch{.identity = uint2(1, 14), .timestep = 1.0f/60.0f, .warm_start_factor = 0.25f};
			auto records = std::array<GpuConstraintRigidRecord, 1>{};
			auto results = std::array<uint32_t, 1>{};
			solver.SolveTransactions(gpu.m_job, epoch, upload, bodies, records, results);
			PR_EXPECT(bodies[0].momentum_lin.x == 0.75f);
			auto accepted_body = bodies[0];
			auto accepted_record = records[0];

			// A real descriptor edit invalidates the old guess; finite large input makes the proposed load norm unrepresentable.
			desc.D6().m_linear[0].m_target_velocity = 2;
			set.Update(handle, desc);
			upload = PackGpuConstraints(set, BodyRemap(native_bodies));
			config.solver_iterations = 1;
			bodies[0].momentum_lin.x = 1.0e20f;
			auto rejected_input = bodies[0];
			solver.SolveTransactions(gpu.m_job, epoch, upload, bodies, records, results);
			PR_EXPECT(results[0] == GpuConstraintStage_AdmissionRejected);
			PR_EXPECT(std::memcmp(&rejected_input, &bodies[0], sizeof(rejected_input)) == 0);
			PR_EXPECT(std::memcmp(&accepted_record, &records[0], sizeof(accepted_record)) == 0);

			// The unchanged second upload must preserve the pending invalidation: cold half of target two is one, not 1.375.
			bodies[0] = accepted_body;
			solver.SolveTransactions(gpu.m_job, epoch, upload, bodies, records, results);
			PR_EXPECT(results[0] == GpuConstraintStage_Accepted);
			PR_EXPECT(bodies[0].momentum_lin.x == 1.0f && records[0].rows.impulses[0] == 1.0f);
			log_stream() << "Unchanged-upload retry after descriptor invalidation: momentum=" << bodies[0].momentum_lin.x << ", expected=1, stale guess would give=1.375\n";
			solver.SolveTransactions(gpu.m_job, epoch, upload, bodies, records, results);
			PR_EXPECT(bodies[0].momentum_lin.x == 1.5f);

			// A newly raised body reset invalidates once; the still-raised level is not a new event on the next preparation.
			bodies[0].state_flags |= ERigidBodyStateFlags_ResetWarmStart;
			solver.SolveTransactions(gpu.m_job, epoch, upload, bodies, records, results);
			PR_EXPECT(bodies[0].momentum_lin.x == 1.0f);
			solver.SolveTransactions(gpu.m_job, epoch, upload, bodies, records, results);
			PR_EXPECT(bodies[0].momentum_lin.x == 1.5f);
			bodies[0].state_flags = SetFlag(bodies[0].state_flags, ERigidBodyStateFlags_ResetWarmStart, false);
			solver.SolveTransactions(gpu.m_job, epoch, upload, bodies, records, results);
			PR_EXPECT(bodies[0].momentum_lin.x == 1.75f);

			// A raised reset still invalidates historical support at a fresh epoch: half of the remaining .25 speed error is .125.
			++epoch.identity.x;
			bodies[0].state_flags |= ERigidBodyStateFlags_ResetWarmStart;
			solver.SolveTransactions(gpu.m_job, epoch, upload, bodies, records, results);
			PR_EXPECT(bodies[0].momentum_lin.x == 1.875f && records[0].rows.impulses[0] == 0.125f);
			log_stream() << "Body reset/replay/clear/fresh-epoch controls reached 1, 1.5, 1.75, 1.875; fresh owned response=" << records[0].response_b.lin.x << '\n';
		}

		// Each valid block observes and acknowledges shared body-reset input independently, including both endpoint roles.
		PRUnitTestMethod(SharedBodyResetIsAcknowledgedPerBlock, Extended)
		{
			// Two valid blocks share a body in opposite endpoint slots; orthogonal rows keep each physical oracle independent.
			auto body = RigidBody{};
			auto bodies = std::array{MakeTransactionBody(body, 0)};
			auto desc_x = ConstraintDesc{.m_frame_b = {.m_body = BodyRef::Rigid(body)}};
			auto desc_y = ConstraintDesc{.m_frame_a = {.m_body = BodyRef::Rigid(body)}};
			desc_x.D6().m_linear[0] = ConstraintAxisDesc{.m_mode = EConstraintAxisMode::Driven, .m_target_velocity = 1};
			desc_y.D6().m_linear[1] = ConstraintAxisDesc{.m_mode = EConstraintAxisMode::Driven, .m_target_velocity = -2};
			auto set = ConstraintSet{};
			set.Add(desc_x);
			set.Add(desc_y);
			auto native_bodies = std::array{&body};
			auto upload = PackGpuConstraints(set, BodyRemap(native_bodies));
			auto config = EngineConfig{};
			config.solver_iterations = 2;
			config.constraint_relaxation = 0.5f;
			config.push_out_iterations = 0;
			auto& gpu = SharedTestGpu();
			auto solver = GpuConstraintSolver(gpu, config);
			auto epoch = GpuConstraintEpoch{.identity = uint2(1, 16), .timestep = 1.0f/60.0f, .warm_start_factor = 0.25f};
			auto records = std::array<GpuConstraintRigidRecord, 2>{};
			auto results = std::array<uint32_t, 2>{};
			solver.SolveTransactions(gpu.m_job, epoch, upload, bodies, records, results);
			PR_EXPECT(bodies[0].momentum_lin.x == 0.75f && bodies[0].momentum_lin.y == 1.5f);

			// Use the existing native motion-type event, returning to the same mobility without replacing GPU physical state.
			body.Motion(ERigidBodyMotion::Kinematic, {});
			body.Motion(ERigidBodyMotion::Dynamic, {});
			auto reset = PackDynamics(body, 0).state_flags & ERigidBodyStateFlags_ResetWarmStart;
			PR_EXPECT(reset != 0u);
			bodies[0].state_flags |= reset;
			config.solver_iterations = 1;
			solver.SolveTransactions(gpu.m_job, epoch, upload, bodies, records, results);
			log_stream() << "Shared-body reset: momentum=(" << bodies[0].momentum_lin.x << ',' << bodies[0].momentum_lin.y << "), expected=(.5,1); consuming the shared flag at block zero would leave y=1.75\n";
			PR_EXPECT(results[0] == GpuConstraintStage_Accepted && results[1] == GpuConstraintStage_Accepted);
			PR_EXPECT(bodies[0].momentum_lin.x == 0.5f && bodies[0].momentum_lin.y == 1.0f);
			PR_EXPECT((records[0].rows.invalidation_receipt & 3u) == 2u);
			PR_EXPECT((records[1].rows.invalidation_receipt & 3u) == 1u);
			PR_EXPECT(AllSet(bodies[0].state_flags, ERigidBodyStateFlags_ResetWarmStart));

			// Both receipts continue their own accepted guesses while the shared step-scoped input remains raised.
			solver.SolveTransactions(gpu.m_job, epoch, upload, bodies, records, results);
			PR_EXPECT(bodies[0].momentum_lin.x == 0.75f && bodies[0].momentum_lin.y == 1.5f);
			PR_EXPECT(records[0].rows.impulses[0] == 0.75f && records[1].rows.impulses[1] == -1.5f);
			PR_EXPECT(AllSet(bodies[0].state_flags, ERigidBodyStateFlags_ResetWarmStart));
		}

		// Healthy growth and source replacement initialize the complete resource set before it becomes usable.
		PRUnitTestMethod(StorageGrowthStartsWithEmptyAcceptedRecords, Extended)
		{
			// Start with one accepted owner while both physical endpoints are already available.
			auto a = RigidBody{};
			auto b = RigidBody{};
			auto bodies = std::array{MakeTransactionBody(a, 0), MakeTransactionBody(b, 1)};
			auto desc_a = ConstraintDesc{.m_frame_b = {.m_body = BodyRef::Rigid(a)}};
			auto desc_b = ConstraintDesc{.m_frame_b = {.m_body = BodyRef::Rigid(b)}};
			desc_a.D6().m_linear[0] = ConstraintAxisDesc{.m_mode = EConstraintAxisMode::Driven, .m_target_velocity = 1};
			desc_b.D6().m_linear[0] = ConstraintAxisDesc{.m_mode = EConstraintAxisMode::Driven, .m_target_velocity = 2};
			auto set = ConstraintSet{};
			set.Add(desc_a);
			auto native_bodies = std::array{&a, &b};
			auto config = EngineConfig{};
			config.solver_iterations = 1;
			config.constraint_relaxation = 0.5f;
			config.push_out_iterations = 0;
			auto& gpu = SharedTestGpu();
			auto solver = GpuConstraintSolver(gpu, config);
			auto epoch = GpuConstraintEpoch{.identity = uint2(1, 15), .timestep = 1.0f/60.0f, .warm_start_factor = 0.25f};
			auto records = std::array<GpuConstraintRigidRecord, 2>{};
			auto results = std::array<uint32_t, 2>{};
			auto upload = PackGpuConstraints(set, BodyRemap(native_bodies));
			solver.SolveTransactions(gpu.m_job, epoch, upload, bodies, std::span(records).first(1), std::span(results).first(1));
			PR_EXPECT(bodies[0].momentum_lin.x == 0.5f);

			// Growing one to two slots at a fresh epoch retains physical history but starts both new records cold.
			set.Add(desc_b);
			upload = PackGpuConstraints(set, BodyRemap(native_bodies));
			++epoch.identity.x;
			solver.SolveTransactions(gpu.m_job, epoch, upload, bodies, records, results);
			PR_EXPECT(results[0] == GpuConstraintStage_Accepted && results[1] == GpuConstraintStage_Accepted);
			PR_EXPECT(bodies[0].momentum_lin.x == 0.75f && bodies[1].momentum_lin.x == 1.0f);
			PR_EXPECT(records[0].response_b.lin.x == 0.25f && records[1].response_b.lin.x == 1.0f);
			auto output = solver.TransactionOutput();
			PR_EXPECT(output.m_slot_count == 2);
			PR_EXPECT(output.m_records->GetDesc().Width >= 2 * sizeof(GpuConstraintRigidRecord));
			PR_EXPECT(output.m_results->GetDesc().Width >= 2 * sizeof(uint32_t));

			// Replacing source at retained capacity also clears acceptance without an allocator-recovery mode.
			auto other = ConstraintSet{};
			other.Add(desc_a);
			other.Add(desc_b);
			upload = PackGpuConstraints(other, BodyRemap(native_bodies));
			++epoch.identity.x;
			bodies[0].momentum_lin.x = bodies[1].momentum_lin.x = 0;
			solver.SolveTransactions(gpu.m_job, epoch, upload, bodies, records, results);
			PR_EXPECT(bodies[0].momentum_lin.x == 0.5f && bodies[1].momentum_lin.x == 1.0f);
			output = solver.TransactionOutput();
			log_stream() << "Healthy capacity 1->2 and same-capacity source replacement: record bytes=" << output.m_records->GetDesc().Width << '\n';
		}

		// Input invalidation must not alias an accepted receipt when upload buffers grow inside spare accepted-record capacity.
		PRUnitTestMethod(UploadGrowthPreservesInvalidationIdentity, Extended)
		{
			// Enabled zero-row fillers grow storage without changing the one physical X-drive or its row identity.
			auto body = RigidBody{};
			auto bodies = std::array{MakeTransactionBody(body, 0)};
			auto desc = ConstraintDesc{.m_frame_b = {.m_body = BodyRef::Rigid(body)}};
			desc.D6().m_linear[0] = ConstraintAxisDesc{.m_mode = EConstraintAxisMode::Driven, .m_target_velocity = 1};
			auto filler = ConstraintDesc{.m_frame_b = {.m_body = BodyRef::Rigid(body)}};
			auto set = ConstraintSet{};
			auto handle = set.Add(desc);
			set.Add(filler);
			auto native_bodies = std::array{&body};
			auto config = EngineConfig{};
			config.solver_iterations = 1;
			config.constraint_relaxation = 0.5f;
			config.push_out_iterations = 0;
			auto& gpu = SharedTestGpu();
			auto solver = GpuConstraintSolver(gpu, config);
			auto epoch = GpuConstraintEpoch{.identity = uint2(1, 17), .timestep = 1.0f/60.0f, .warm_start_factor = 1};
			auto records = std::array<GpuConstraintRigidRecord, 4>{};
			auto results = std::array<uint32_t, 4>{};
			auto const expected_momenta = std::array{0.5f, 0.75f, 1.375f};
			auto const expected_capacities = std::array{2, 4, 4};
			auto previous = GpuConstraintRigidRecord{};
			auto generation = uint32_t{};
			for (int phase = 0; phase != 3; ++phase)
			{
				// Every new epoch preserves prior physical history; only the final growth changes the actual drive input.
				if (phase != 0)
					set.Add(filler);

				if (phase == 2)
				{
					// Identical generation, row mask, and geometric feature make the invalidation receipt the discriminating input.
					desc.D6().m_linear[0].m_target_velocity = 2;
					set.Update(handle, desc);
				}
				auto upload = PackGpuConstraints(set, BodyRemap(native_bodies));
				if (phase == 0)
					generation = upload.m_endpoints[0].generation;

				PR_EXPECT(upload.m_endpoints[0].generation == generation);
				PR_EXPECT(!AllSet(bodies[0].state_flags, ERigidBodyStateFlags_ResetWarmStart));
				solver.SolveTransactions(gpu.m_job, epoch, upload, bodies, std::span(records).first(phase + 2), std::span(results).first(phase + 2));
				auto output = solver.TransactionOutput();
				auto const record_capacity = output.m_records->GetDesc().Width / sizeof(GpuConstraintRigidRecord);
				auto const stats = solver.Stats();
				log_stream() << "Growth " << phase + 2 << ": upload capacity=" << stats.m_slot_capacity
					<< ", record capacity=" << record_capacity << ", generation=" << generation
					<< ", previous receipt=" << previous.rows.invalidation_receipt
					<< ", accepted receipt=" << records[0].rows.invalidation_receipt
					<< ", momentum=" << bodies[0].momentum_lin.x << ", owned=" << records[0].response_b.lin.x
					<< ", coefficient=" << records[0].rows.impulses[0] << ", expected=" << expected_momenta[phase] << '\n';
				PR_EXPECT(stats.m_slot_capacity == phase + 2);
				PR_EXPECT(record_capacity == expected_capacities[phase]);
				PR_EXPECT(results[0] == GpuConstraintStage_Accepted);
				PR_EXPECT(bodies[0].momentum_lin.x == expected_momenta[phase]);
				for (int slot = 1; slot != phase + 2; ++slot)
				{
					// These are real enabled public constraints, not fabricated lowered rows.
					PR_EXPECT(AllSet(upload.m_endpoints[slot].flags, GpuConstraintEndpointFlags_Enabled));
					PR_EXPECT(records[slot].rows.velocity_mask == 0u);
				}
				if (phase == 2)
				{
					// An old accepted revision cannot stand for a new descriptor, regardless of storage allocation boundaries.
					PR_EXPECT(records[0].rows.invalidation_receipt != previous.rows.invalidation_receipt);
					PR_EXPECT(records[0].rows.velocity_mask == previous.rows.velocity_mask);
					PR_EXPECT(records[0].rows.row_states == previous.rows.row_states);
					PR_EXPECT(records[0].rows.row_identities == previous.rows.row_identities);
					PR_EXPECT(records[0].response_b.lin.x == 0.625f);
				}
				previous = records[0];
				++epoch.identity.x;
			}
		}

		// Upload and preparation may replace resources while an earlier valid prefix is still recorded in the same job.
		PRUnitTestMethod(GrowingPreparationPreservesRecordedPrefix, Extended)
		{
			// The first preparation really has one logical body, then grows pseudo storage to two even with no position iterations.
			auto a = RigidBody{};
			auto b = RigidBody{};
			auto bodies = std::array{MakeTransactionBody(a, 0), MakeTransactionBody(b, 1)};
			auto desc_a = ConstraintDesc{.m_frame_b = {.m_body = BodyRef::Rigid(a)}};
			auto desc_b = ConstraintDesc{.m_frame_b = {.m_body = BodyRef::Rigid(b)}};
			desc_a.D6().m_linear[0] = ConstraintAxisDesc{.m_mode = EConstraintAxisMode::Driven, .m_target_velocity = 1};
			desc_b.D6().m_linear[0] = ConstraintAxisDesc{.m_mode = EConstraintAxisMode::Driven, .m_target_velocity = 2};
			desc_a.m_break_force = desc_b.m_break_force = 10;
			auto set = ConstraintSet{};
			auto handle_a = set.Add(desc_a);
			auto handle_b = ConstraintHandle{};
			auto native_bodies = std::array{&a, &b};
			auto config = EngineConfig{};
			config.constraint_relaxation = 0.5f;
			config.push_out_iterations = 0;
			auto& gpu = SharedTestGpu();
			auto& job = gpu.m_job;
			auto solver = GpuConstraintSolver(gpu, config);
			auto r_bodies = D3DPtr<ID3D12Resource>{};
			auto body_readbacks = std::array<ReadbackAlloc, 2>{};
			auto record_readbacks = std::array<ReadbackAlloc, 2>{};
			auto pseudo_readbacks = std::array<ReadbackAlloc, 2>{};
			auto break_readbacks = std::array<ReadbackAlloc, 2>{};
			try
			{
				// The fixture owns only physical input; displaced solver resources must be retained by their actual owner.
				r_bodies = gpu.CreateResource(compute::ResDesc::Buf<GpuRigidBody>(2, {}).usage(compute::EUsage::UnorderedAccess), job, "Physics:GrowingPreparationBodies");
				auto data = job.m_upload.Alloc<GpuRigidBody>(2);
				std::copy(bodies.begin(), bodies.end(), data.ptr<GpuRigidBody>());
				job.m_barriers.Transition(r_bodies.get(), D3D12_RESOURCE_STATE_COPY_DEST).Commit();
				job.m_cmd_list.CopyBufferRegion(r_bodies.get(), 0, data);
				for (int phase = 0; phase != 2; ++phase)
				{
					// Both preparations and their readbacks remain unsubmitted until after all replacements and optional-storage release.
					if (phase == 1)
						handle_b = set.Add(desc_b);

					auto upload = PackGpuConstraints(set, BodyRemap(std::span(native_bodies).first(phase + 1)));
					solver.Upload(job, upload);
					auto epoch = GpuConstraintEpoch{.identity = uint2(phase + 1, 18), .timestep = 1.0f/60.0f, .warm_start_factor = 1};
					solver.PrepareVelocityTransactions(job, epoch);
					solver.ClearPositionState(job, epoch.timestep, phase + 1, r_bodies);
					solver.CompileRows(job, epoch.timestep, phase + 1, r_bodies);
					solver.PrepareVelocitySolve(job, epoch.timestep, phase + 1, r_bodies);
					solver.CommitVelocityTransactions(job, phase + 1, r_bodies);
					auto const stats = solver.Stats();
					PR_EXPECT(stats.m_body_capacity == phase + 1);
					PR_EXPECT(stats.m_slot_capacity == phase + 1);
					PR_EXPECT(stats.m_break_capacity == phase + 1);

					// These temporary handles leave scope before the next growth; readback commands do not own their source resources.
					auto output = solver.TransactionOutput();
					auto pseudo = solver.PseudoVelocityStorage(job, phase + 1);
					auto breaks = solver.BreakOutput();
					body_readbacks[phase] = job.m_readback.Alloc<GpuRigidBody>(2);
					record_readbacks[phase] = job.m_readback.Alloc<GpuConstraintRigidRecord>(phase + 1);
					pseudo_readbacks[phase] = job.m_readback.Alloc<GpuConstraintPseudoVelocity>(phase + 1);
					break_readbacks[phase] = job.m_readback.Alloc<GpuConstraintBreakState>(phase + 1);
					job.m_barriers.Transition(r_bodies.get(), D3D12_RESOURCE_STATE_COPY_SOURCE);
					job.m_barriers.Transition(output.m_records, D3D12_RESOURCE_STATE_COPY_SOURCE);
					job.m_barriers.Transition(pseudo.get(), D3D12_RESOURCE_STATE_COPY_SOURCE);
					job.m_barriers.Transition(breaks.m_states, D3D12_RESOURCE_STATE_COPY_SOURCE);
					job.m_barriers.Commit();
					job.m_cmd_list.CopyBufferRegion(body_readbacks[phase], r_bodies.get(), 0);
					job.m_cmd_list.CopyBufferRegion(record_readbacks[phase], output.m_records, 0);
					job.m_cmd_list.CopyBufferRegion(pseudo_readbacks[phase], pseudo.get(), 0);
					job.m_cmd_list.CopyBufferRegion(break_readbacks[phase], breaks.m_states, 0);
					log_stream() << "Recorded phase " << phase << ": slots=" << stats.m_slot_capacity
						<< ", pseudo bodies=" << stats.m_body_capacity << ", break slots=" << stats.m_break_capacity << '\n';
				}

				// The optional latch can disappear before commands that used it are submitted.
				desc_a.m_break_force = desc_b.m_break_force = std::numeric_limits<float>::infinity();
				set.Update(handle_a, desc_a);
				set.Update(handle_b, desc_b);
				solver.Upload(job, PackGpuConstraints(set, BodyRemap(native_bodies)));
				PR_EXPECT(solver.BreakOutput().m_states == nullptr);
			}
			catch (...)
			{
				// Retire only an interrupted recording while every surviving owner is still alive.
				auto error = std::current_exception();
				try
				{
					// This does not turn the original failure into a successful diagnostic.
					job.RetireRecordedWork();
				}
				catch (std::exception const& cleanup_error)
				{
					// Preserve the original error and the distinct retirement failure.
					std::cerr << "Growing-preparation retirement failed: " << cleanup_error.what() << '\n';
				}
				std::rethrow_exception(error);
			}
			job.Run();

			// Both prefixes must retain their own physical result and initialized auxiliary state after the later replacements.
			for (int phase = 0; phase != 2; ++phase)
			{
				// No allocator failure was injected: this checks healthy growth and recorded-resource lifetime only.
				auto actual = body_readbacks[phase].ptr<GpuRigidBody>();
				auto records = record_readbacks[phase].ptr<GpuConstraintRigidRecord>();
				auto zero = GpuConstraintPseudoVelocity{};
				PR_EXPECT(actual[0].momentum_lin.x == (phase == 0 ? 0.5f : 0.75f));
				PR_EXPECT(actual[1].momentum_lin.x == (phase == 0 ? 0.0f : 1.0f));
				PR_EXPECT(records[0].response_b.lin.x == (phase == 0 ? 0.5f : 0.25f));
				for (int index = 0; index != phase + 1; ++index)
				{
					// Both clear/init dispatches used the capacity belonging to their own recorded preparation.
					PR_EXPECT(std::memcmp(&pseudo_readbacks[phase].ptr<GpuConstraintPseudoVelocity>()[index], &zero, sizeof(zero)) == 0);
					PR_EXPECT(break_readbacks[phase].ptr<GpuConstraintBreakState>()[index].flags == 0u);
				}
				log_stream() << "Completed prefix " << phase << ": momentum=(" << actual[0].momentum_lin.x << ',' << actual[1].momentum_lin.x << ")\n";
			}
		}

		// A new provisional twist may be unrepresentable while live input remains physically finite; this is not a projector failure.
		PRUnitTestMethod(SeedTwistAdmissionIsNotFatal, Extended)
		{
			// First commit a real angular support response that leaves exactly zero physical momentum.
			auto body = RigidBody{};
			auto bodies = std::array{MakeTransactionBody(body, 0)};
			bodies[0].inertia_inv_diagonal = float4(1, 1, 1, 0);
			bodies[0].inertia_inv_products = float4(0, 0, 0, 0);
			bodies[0].momentum_ang.y = 8.0f;
			auto desc = ConstraintDesc{.m_frame_b = {.m_body = BodyRef::Rigid(body)}};
			desc.D6().m_angular[1].m_mode = EConstraintAxisMode::Locked;
			auto set = ConstraintSet{};
			set.Add(desc);
			auto native_bodies = std::array{&body};
			auto upload = PackGpuConstraints(set, BodyRemap(native_bodies));
			auto config = EngineConfig{};
			config.solver_iterations = 1;
			config.push_out_iterations = 0;
			auto& gpu = SharedTestGpu();
			auto solver = GpuConstraintSolver(gpu, config);
			auto epoch = GpuConstraintEpoch{.identity = uint2(1, 10), .timestep = 1.0f/60.0f, .warm_start_factor = 0.25f};
			auto records = std::array<GpuConstraintRigidRecord, 1>{};
			auto results = std::array<uint32_t, 1>{};
			solver.SolveTransactions(gpu.m_job, epoch, upload, bodies, records, results);
			PR_EXPECT(results[0] == GpuConstraintStage_Accepted);
			PR_EXPECT(bodies[0].momentum_ang.y == 0.0f && records[0].response_b.ang.y == -8.0f);
			auto previous = records[0];

			// The rotated logarithm covector applies the old scalar seed partly in a newly very responsive direction.
			bodies[0].o2w = m4x4::Transform(v4::ZAxis(), 0.5f * std::acos(-1.0f), v4::Origin());
			bodies[0].inertia_inv_diagonal = float4(1, 1.0e38f, 1, 0);
			auto before = bodies[0];
			auto live_twist = BodyTwistWS(before);
			PR_EXPECT(All(live_twist.ang == float3(0, 0, 0)));
			solver.SolveTransactions(gpu.m_job, epoch, upload, bodies, records, results);
			PR_EXPECT(results[0] == GpuConstraintStage_AdmissionRejected);
			PR_EXPECT(std::memcmp(&before, &bodies[0], sizeof(before)) == 0);
			PR_EXPECT(std::memcmp(&previous, &records[0], sizeof(previous)) == 0);

			// A corrected mobility/pose retries against unchanged accepted ownership.
			bodies[0].o2w = m4x4::Identity();
			bodies[0].inertia_inv_diagonal = float4(1, 1, 1, 0);
			solver.SolveTransactions(gpu.m_job, epoch, upload, bodies, records, results);
			PR_EXPECT(results[0] == GpuConstraintStage_Accepted);
			PR_EXPECT(bodies[0].momentum_ang.y == 0.0f);

			// An accepted epoch cannot change dt: that invalid context must still reach the fatal numerical transport.
			epoch.timestep *= 0.5f;
			PR_THROWS(solver.SolveTransactions(gpu.m_job, epoch, upload, bodies, records, results), std::runtime_error);
			PR_EXPECT(results[0] == GpuConstraintStage_NumericalFailure);
			PR_EXPECT(std::memcmp(&previous, &records[0], sizeof(previous)) == 0);
		}

		// Measure actual committed momentum and accepted response after a noncoaxial pose/mobility change and external impulse.
		PRUnitTestMethod(TwoMovingEndpointsRebase, Extended)
		{
			// Both endpoints have unequal finite inertias and independent physical angular/linear state.
			auto a = RigidBody{};
			auto b = RigidBody{};
			auto bodies = std::array{MakeTransactionBody(a, 0), MakeTransactionBody(b, 1)};
			bodies[0].inertia_inv_diagonal = float4(1, 0.5f, 1.0f/3.0f, 0);
			bodies[1].inertia_inv_diagonal = float4(0.5f, 0.25f, 1.0f/3.0f, 0);
			bodies[0].inertia_inv_products = bodies[1].inertia_inv_products = float4(0, 0, 0, 0);
			bodies[0].momentum_ang = float4(1, -2, 0.5f, 0);
			bodies[1].momentum_ang = float4(-2, 3, 1, 0);
			bodies[0].momentum_lin = float4(1, 2, 3, 0);
			bodies[1].momentum_lin = float4(-2, 4, -1, 0);
			auto free_a = bodies[0].momentum_ang;
			auto free_b = bodies[1].momentum_ang;
			auto set = ConstraintSet{};
			set.Add(ConstraintDesc{
				.m_frame_a = {.m_body = BodyRef::Rigid(a)},
				.m_frame_b = {.m_body = BodyRef::Rigid(b)},
				.m_parameters = SphericalAngularConstraintParameters{.m_damping = 6},
			});
			auto native_bodies = std::array{&a, &b};
			auto upload = PackGpuConstraints(set, BodyRemap(native_bodies));
			auto config = EngineConfig{};
			config.solver_iterations = 1;
			config.push_out_iterations = 0;
			auto& gpu = SharedTestGpu();
			auto solver = GpuConstraintSolver(gpu, config);
			auto epoch = GpuConstraintEpoch{.identity = uint2(1, 3), .timestep = 1.0f/60.0f, .warm_start_factor = 0.25f};
			auto records = std::array<GpuConstraintRigidRecord, 1>{};
			auto results = std::array<uint32_t, 1>{};
			solver.SolveTransactions(gpu.m_job, epoch, upload, bodies, records, results);
			PR_EXPECT(results[0] == GpuConstraintStage_Accepted);
			for (int axis = 0; axis != 3; ++axis)
			{
				// The initial diagonal impulse solution is independent of the shader compiler and stored coefficients.
				auto wa = static_cast<double>(bodies[0].inertia_inv_diagonal[axis]);
				auto wb = static_cast<double>(bodies[1].inertia_inv_diagonal[axis]);
				auto expected = -(wb*free_b[axis]-wa*free_a[axis])/(10.0+wa+wb);
				PR_EXPECT(std::abs(bodies[1].momentum_ang[axis] - free_b[axis] - expected) < 1.0e-6);
				PR_EXPECT(std::abs(records[0].response_b.ang[axis] - expected) < 1.0e-6);
			}
			auto first = bodies;
			solver.SolveTransactions(gpu.m_job, epoch, upload, bodies, records, results);
			PR_EXPECT(results[0] == GpuConstraintStage_Accepted);
			PR_EXPECT(FEqlAbsolute(bodies[0].momentum_ang, first[0].momentum_ang, 5.0e-7f));
			PR_EXPECT(FEqlAbsolute(bodies[1].momentum_ang, first[1].momentum_ang, 5.0e-7f));

			// Apply an independent, known impulse directly to live input; only this block's previous response may be replaced.
			auto external_a = float4(0.125f, -0.25f, 0.5f, 0);
			auto external_b = float4(-0.375f, 0.125f, -0.25f, 0);
			free_a += external_a;
			free_b += external_b;
			bodies[0].momentum_ang += external_a;
			bodies[1].momentum_ang += external_b;
			bodies[0].momentum_lin += float4(0.25f, 0.5f, -0.125f, 0);
			auto before = bodies;
			auto old_record = records[0];
			bodies[0].o2w = m4x4::Transform(v4::ZAxis(), 0.6f, v4::Origin());
			bodies[1].o2w = m4x4::Transform(v4::ZAxis(), -0.4f, v4::Origin());
			bodies[1].inertia_inv_diagonal = float4(0.25f, 0.5f, 0.125f, 0);
			solver.SolveTransactions(gpu.m_job, epoch, upload, bodies, records, results);
			PR_EXPECT(results[0] == GpuConstraintStage_Accepted);
			auto impulse_b = bodies[1].momentum_ang - free_b;
			auto actual_delta = bodies[1].momentum_ang - before[1].momentum_ang;
			double inverse[2][3][3]{};
			for (int endpoint = 0; endpoint != 2; ++endpoint)
			{
				// Independent world inverse inertia from the principal moments and known pose rotation, not BodyTwistWS or compiled J.
				auto angle = endpoint == 0 ? 0.6 : -0.4;
				auto c = std::cos(angle);
				auto s = std::sin(angle);
				auto x = endpoint == 0 ? 1.0 : 0.25;
				auto y = 0.5;
				inverse[endpoint][0][0] = x*c*c+y*s*s;
				inverse[endpoint][1][1] = x*s*s+y*c*c;
				inverse[endpoint][0][1] = inverse[endpoint][1][0] = (x-y)*c*s;
				inverse[endpoint][2][2] = endpoint == 0 ? 1.0/3.0 : 0.125;
			}
			for (int axis = 0; axis != 3; ++axis)
			{
				// Reconstruct actual total impulse and incremental publication separately from old/new scalar coefficients.
				auto relative = 0.0;
				for (int column = 0; column != 3; ++column)
				{
					// Evaluate the two physical inertias independently.
					relative += inverse[1][axis][column]*bodies[1].momentum_ang[column] - inverse[0][axis][column]*bodies[0].momentum_ang[column];
				}
				PR_EXPECT(std::abs(impulse_b[axis] + 0.1*relative) < 3.0e-6);
				PR_EXPECT(std::abs(bodies[0].momentum_ang[axis] - free_a[axis] + impulse_b[axis]) < 3.0e-6);
				PR_EXPECT(std::abs(old_record.response_b.ang[axis] + actual_delta[axis] - records[0].response_b.ang[axis]) < 1.0e-6);
			}
			PR_EXPECT(All(bodies[0].momentum_lin == before[0].momentum_lin));
			PR_EXPECT(All(bodies[1].momentum_lin == before[1].momentum_lin));
			log_stream() << "actual owner publication: old response=" << old_record.response_b.ang << ", delta=" << actual_delta << ", total physical impulse=" << impulse_b << '\n';
		}

		// A current-epoch zero-row visit releases accepted support even though normal colouring no longer visits the slot.
		PRUnitTestMethod(ZeroRowsAndZeroIterations, Extended)
		{
			// One upper linear limit removes outward speed; pose-only relocation later makes that same descriptor inactive.
			auto body = RigidBody{};
			auto bodies = std::array{MakeTransactionBody(body, 0)};
			bodies[0].o2w.pos.y = 0.1f;
			bodies[0].momentum_lin = float4(0.25f, 1, -0.125f, 0);
			auto desc = ConstraintDesc{.m_frame_b = {.m_body = BodyRef::Rigid(body)}};
			desc.D6().m_linear[1] = ConstraintAxisDesc{.m_mode = EConstraintAxisMode::Limited, .m_limits = {-1, 0}};
			auto set = ConstraintSet{};
			set.Add(desc);
			auto native_bodies = std::array{&body};
			auto upload = PackGpuConstraints(set, BodyRemap(native_bodies));
			auto config = EngineConfig{};
			config.solver_iterations = 1;
			config.push_out_iterations = 0;
			auto& gpu = SharedTestGpu();
			auto solver = GpuConstraintSolver(gpu, config);
			auto epoch = GpuConstraintEpoch{.identity = uint2(1, 4), .timestep = 1.0f/60.0f, .warm_start_factor = 0.25f};
			auto records = std::array<GpuConstraintRigidRecord, 1>{};
			auto results = std::array<uint32_t, 1>{};
			solver.SolveTransactions(gpu.m_job, epoch, upload, bodies, records, results);
			PR_EXPECT(results[0] == GpuConstraintStage_Accepted);
			PR_EXPECT(bodies[0].momentum_lin.y == 0.0f);
			PR_EXPECT(records[0].rows.impulses[1] == -1.0f);

			// Bring the unconstrained directions to rest, then replay nonzero support without waking an exactly unchanged sleeping body.
			bodies[0].momentum_lin = float4(0, 0, 0, 0);
			bodies[0].state_flags |= ERigidBodyStateFlags_Sleeping;
			bodies[0].sleep.island_id = 19;
			bodies[0].sleep.generation = 7;
			auto resting = bodies[0];
			solver.SolveTransactions(gpu.m_job, epoch, upload, bodies, records, results);
			PR_EXPECT(results[0] == GpuConstraintStage_Accepted && records[0].rows.impulses[1] == -1.0f);
			PR_EXPECT(std::memcmp(&resting, &bodies[0], sizeof(resting)) == 0);

			// The same descriptor and epoch retain their owner while the boundary becomes inactive.
			bodies[0].o2w.pos.y = -0.1f;
			solver.SolveTransactions(gpu.m_job, epoch, upload, bodies, records, results);
			PR_EXPECT(results[0] == GpuConstraintStage_Accepted);
			PR_EXPECT(bodies[0].momentum_lin.y == 1.0f);
			PR_EXPECT(records[0].rows.velocity_mask == 0u && records[0].rows.impulses[1] == 0.0f);
			PR_EXPECT(records[0].rows.linear_impulse_norm == 0.0f && records[0].response_b.lin.y == 0.0f);

			// Restore observable support before starting a fresh zero-iteration epoch; an absent warm seed would not discriminate this contract.
			bodies[0].o2w.pos.y = 0.1f;
			solver.SolveTransactions(gpu.m_job, epoch, upload, bodies, records, results);
			PR_EXPECT(results[0] == GpuConstraintStage_Accepted && records[0].rows.impulses[1] == -1.0f);
			auto previous = records[0];
			auto before = bodies[0];
			++epoch.identity.x;
			config.solver_iterations = 0;
			solver.SolveTransactions(gpu.m_job, epoch, upload, bodies, records, results);
			PR_EXPECT(results[0] == 0xFFFFFFFFu);
			PR_EXPECT(std::memcmp(&before, &bodies[0], sizeof(before)) == 0);
			PR_EXPECT(std::memcmp(&previous, &records[0], sizeof(previous)) == 0);
		}

		// Nonzero accepted coefficients can produce zero represented response; an inactive visit must still clear their cache/load.
		PRUnitTestMethod(ZeroResponseStillRetiresCoefficients, Extended)
		{
			// A soft limit on a massive body has finite nonzero support too small to change its represented momentum.
			auto body = RigidBody{};
			auto bodies = std::array{MakeTransactionBody(body, 0)};
			bodies[0].os_com_and_invmass.w = 1.0e-30f;
			bodies[0].momentum_lin.x = 1.0e30f;
			bodies[0].o2w.pos.x = 0.1f;
			auto desc = ConstraintDesc{.m_frame_b = {.m_body = BodyRef::Rigid(body)}};
			desc.D6().m_linear[0] = ConstraintAxisDesc{.m_mode = EConstraintAxisMode::Limited, .m_limits = {-1, 0}, .m_damping = 10};
			auto set = ConstraintSet{};
			set.Add(desc);
			auto native_bodies = std::array{&body};
			auto upload = PackGpuConstraints(set, BodyRemap(native_bodies));
			auto config = EngineConfig{};
			config.solver_iterations = 1;
			config.push_out_iterations = 0;
			auto& gpu = SharedTestGpu();
			auto solver = GpuConstraintSolver(gpu, config);
			auto epoch = GpuConstraintEpoch{.identity = uint2(1, 11), .timestep = 1.0f/60.0f, .warm_start_factor = 0.25f};
			auto records = std::array<GpuConstraintRigidRecord, 1>{};
			auto results = std::array<uint32_t, 1>{};
			solver.SolveTransactions(gpu.m_job, epoch, upload, bodies, records, results);
			PR_EXPECT(results[0] == GpuConstraintStage_Accepted);
			PR_EXPECT(records[0].rows.impulses[0] < 0.0f && records[0].rows.linear_impulse_norm > 0.0f);
			PR_EXPECT(records[0].response_b.lin.x == 0.0f && bodies[0].momentum_lin.x == 1.0e30f);

			// The same owner becomes inactive through geometry only, not descriptor mutation or an explicit cache reset.
			bodies[0].o2w.pos.x = -0.1f;
			solver.SolveTransactions(gpu.m_job, epoch, upload, bodies, records, results);
			PR_EXPECT(results[0] == GpuConstraintStage_Accepted);
			PR_EXPECT(records[0].rows.velocity_mask == 0u && records[0].rows.impulses[0] == 0.0f);
			PR_EXPECT(records[0].rows.linear_impulse_norm == 0.0f && bodies[0].momentum_lin.x == 1.0e30f);
		}

		// A real accepted response plus an interleaved finite input makes free-state recovery unrepresentable; the whole owner rejects.
		PRUnitTestMethod(RejectedOwnerPreservesPublishedState, Extended)
		{
			// A large lever produces a large angular response from a representable linear-row load.
			auto body = RigidBody{};
			auto bodies = std::array{MakeTransactionBody(body, 0)};
			bodies[0].inertia_inv_diagonal = float4(1, 1, 1.0e-20f, 0);
			bodies[0].inertia_inv_products = float4(0, 0, 0, 0);
			bodies[0].momentum_ang.z = -1.0e38f;
			bodies[0].momentum_lin.y = -1.0e18f;
			auto desc = ConstraintDesc{
				.m_frame_a = {.m_constraint_to_body = m4x4::Translation(1.0e20f, 0, 0)},
				.m_frame_b = {.m_body = BodyRef::Rigid(body), .m_constraint_to_body = m4x4::Translation(1.0e20f, 0, 0)},
			};
			desc.D6().m_linear[1].m_mode = EConstraintAxisMode::Locked;
			auto set = ConstraintSet{};
			set.Add(desc);
			auto native_bodies = std::array{&body};
			auto upload = PackGpuConstraints(set, BodyRemap(native_bodies));
			auto config = EngineConfig{};
			config.solver_iterations = 1;
			config.push_out_iterations = 0;
			auto& gpu = SharedTestGpu();
			auto solver = GpuConstraintSolver(gpu, config);
			auto epoch = GpuConstraintEpoch{.identity = uint2(1, 6), .timestep = 1.0f/60.0f, .warm_start_factor = 0.25f};
			auto records = std::array<GpuConstraintRigidRecord, 1>{};
			auto results = std::array<uint32_t, 1>{};
			solver.SolveTransactions(gpu.m_job, epoch, upload, bodies, records, results);
			PR_EXPECT(results[0] == GpuConstraintStage_Accepted);
			PR_EXPECT(records[0].response_b.ang.z > 0.9e38f);
			PR_EXPECT(records[0].rows.linear_impulse_norm > 0.9e18f && records[0].rows.angular_impulse_norm == 0.0f);
			auto accepted_body = bodies[0];
			auto accepted_record = records[0];

			// This finite external input is preserved even though subtracting the old owned response would overflow.
			bodies[0].momentum_ang.z = -3.0e38f;
			bodies[0].state_flags |= ERigidBodyStateFlags_Sleeping;
			bodies[0].sleep.island_id = 31;
			bodies[0].sleep.generation = 17;
			auto rejected_input = bodies[0];
			solver.SolveTransactions(gpu.m_job, epoch, upload, bodies, records, results);
			PR_EXPECT(results[0] == GpuConstraintStage_AdmissionRejected);
			PR_EXPECT(std::memcmp(&rejected_input, &bodies[0], sizeof(rejected_input)) == 0);
			PR_EXPECT(std::memcmp(&accepted_record, &records[0], sizeof(accepted_record)) == 0);

			// Corrected input retries against the still-committed response record, without a physical reset or warm publication.
			bodies[0] = accepted_body;
			solver.SolveTransactions(gpu.m_job, epoch, upload, bodies, records, results);
			PR_EXPECT(results[0] == GpuConstraintStage_Accepted);
			PR_EXPECT(records[0].response_b.ang.z > 0.9e38f);
			log_stream() << "GPU rejection preserved complete body/cache/stamp/sleep bytes; retry accepted, linear load=" << records[0].rows.linear_impulse_norm << ", pure-angular load=" << records[0].rows.angular_impulse_norm << '\n';

			// Two rejected preparations in a fresh epoch cannot advance the accepted dt or reinterpret historical support as current response.
			accepted_record = records[0];
			++epoch.identity.x;
			bodies[0].momentum_ang.z = 3.35e38f;
			bodies[0].momentum_lin.y = 0.0f;
			rejected_input = bodies[0];
			for (int attempt = 0; attempt != 2; ++attempt)
			{
				// Finite live state overflows only when the positive historical seed is provisionally added.
				epoch.timestep *= 0.5f;
				solver.SolveTransactions(gpu.m_job, epoch, upload, bodies, records, results);
				PR_EXPECT(results[0] == GpuConstraintStage_AdmissionRejected);
				PR_EXPECT(std::memcmp(&rejected_input, &bodies[0], sizeof(rejected_input)) == 0);
				PR_EXPECT(std::memcmp(&accepted_record, &records[0], sizeof(accepted_record)) == 0);
			}

			// With zero free input, one under-relaxed iteration leaves exactly half the correctly scaled historical guess.
			bodies[0].momentum_ang.z = 0.0f;
			config.constraint_relaxation = 0.5f;
			auto expected = 0.5f * epoch.warm_start_factor * (epoch.timestep / accepted_record.accepted.timestep) * accepted_record.rows.impulses[1];
			solver.SolveTransactions(gpu.m_job, epoch, upload, bodies, records, results);
			PR_EXPECT(results[0] == GpuConstraintStage_Accepted);
			PR_EXPECT(std::abs(records[0].rows.impulses[1] / expected - 1.0f) < 1.0e-6f);
			PR_EXPECT(std::abs(bodies[0].momentum_ang.z / (expected * 1.0e20f) - 1.0f) < 1.0e-6f);
			log_stream() << "GPU accepted-cache dt provenance after h/2 and h/4 rejection: impulse=" << records[0].rows.impulses[1] << ", independent expected=" << expected << '\n';
		}
	};

	// Check the local physical-merit arithmetic on the device without adding a shipping diagnostic or state buffer.
	PRUnitTestClass(CoupledMeritArithmeticTests)
	{
		// Sharing a transform must preserve every value, uncertainty and exponent bit from separate motion transports.
		PRUnitTestMethod(PairedChildTransportPreservesEnclosures, Extended)
		{
			// Exercise separate inversions with uncertain, unavailable and widely scaled components; fixed host words also detect common-mode errors.
			auto const source = R"(
#include "physics/src/compute/coupled_constraint_work.hlsli"
RWStructuredBuffer<GpuCoupledConstraintWorkMotion> output : register(u0);

// Keep the separate-transport arithmetic as the equivalence reference, not as a shipping alternative.
CoupledWorkMotion ReferenceChildMotion(GpuConstraintFrame frame, CoupledWorkMotion parent)
{
	float4 inverse_q = float4(-frame.rotation.xyz, frame.rotation.w);
	CoupledWorkVector inverse_position = CoupledWorkScale3(CoupledWorkRotate(inverse_q, CoupledWorkInput3(frame.position.xyz)), CoupledMeritInput(-1.0f));
	CoupledWorkVector angular = CoupledWorkRotate(inverse_q, CoupledWorkPart(parent, true));
	CoupledWorkVector translation = CoupledWorkAdd3(CoupledWorkRotate(inverse_q, CoupledWorkPart(parent, false)), CoupledWorkCross3(inverse_position, angular));
	return CoupledWorkPack(angular, translation);
}

numthreads(Probe, 1, 1, 1)
void Probe()
{
	// Exact unit quaternions include identity, cyclic rotation and its opposite quaternion sign.
	for (int test = 0; test != 8; ++test)
	{
		GpuConstraintFrame frame = (GpuConstraintFrame)0;
		frame.rotation = test == 0 ? float4(0, 0, 0, 1) : (test % 2 == 0 ? float4(-0.5f, -0.5f, -0.5f, -0.5f) : float4(0.5f, 0.5f, 0.5f, 0.5f));
		frame.position = float4(1.25f, -0.75f, 0.5f, 0);
		GpuCoupledConstraintWorkMotion parent;
		parent.delta = CoupledWorkPack(CoupledWorkInput3(float3(0.25f, -0.5f, 1)), CoupledWorkInput3(float3(-2, 4, -8)));
		parent.midpoint = CoupledWorkPack(CoupledWorkInput3(float3(0.75f, 1.5f, -3)), CoupledWorkInput3(float3(5, -10, 20)));
		switch (test)
		{
			case 0: { frame.position = float4(0, 0, 0, 0); break; }
			case 1: { break; }
			case 2:
			{
				frame.position.xyz = float3(asfloat(247u << 23), -asfloat(7u << 23), asfloat(1u));
				parent.delta.angular = CoupledWorkInput3(float3(asfloat(27u << 23), -asfloat(227u << 23), asfloat(1u)));
				break;
			}
			case 3:
			{
				parent.delta.angular.components[1].error = 0.001f;
				parent.midpoint.translation.components[2].error = 0.125f;
				frame.position.xyz = float3(-asfloat(227u << 23), asfloat(157u << 23), -asfloat(27u << 23));
				break;
			}
			case 4:
			{
				frame.position.xyz = float3(asfloat(0x007fffffu), -asfloat(1u), 0);
				parent.midpoint.translation = CoupledWorkInput3(float3(asfloat(1u), -asfloat(2u), asfloat(0x007fffffu)));
				break;
			}
			case 5:
			{
				parent.delta.angular.components[0].exponent = uint2(0xfffffff0u, 1u);
				parent.midpoint.translation.components[1].exponent = uint2(16u, 0xffffffffu);
				break;
			}
			case 6:
			{
				parent.delta = CoupledWorkZeroMotion();
				parent.midpoint.angular.components[0] = CoupledMeritInput(asfloat(0x7fc00000u));
				break;
			}
			default:
			{
				parent.delta.angular.components[0].exponent = uint2(0xfffffffeu, 0x7fffffffu);
				frame.position.x = 16;
				break;
			}
		}
		GpuCoupledConstraintWorkMotion reference;
		reference.delta = ReferenceChildMotion(frame, parent.delta);
		reference.midpoint = ReferenceChildMotion(frame, parent.midpoint);
		output[2 * test] = reference;
		GpuCoupledConstraintWorkMotion actual;
		actual.delta = CoupledWorkChildMotion(frame, parent.delta);
		actual.midpoint = CoupledWorkChildMotion(frame, parent.midpoint);
		output[2 * test + 1] = actual;
	}
}
)";
			// Isolated-device control words, independently reproduced by scalar DXIL replay. Each scalar stores value, error, exponent low, exponent high.
			auto const control = std::array<uint32_t, 384>{
				0x3f000000u, 0x00000000u, 0xffffffffu, 0xffffffffu, 0xbf000000u, 0x00000000u, 0x00000000u, 0x00000000u,
				0x3f000000u, 0x00000000u, 0x00000001u, 0x00000000u, 0xbf000000u, 0x00000000u, 0x00000002u, 0x00000000u,
				0x3f000000u, 0x00000000u, 0x00000003u, 0x00000000u, 0xbf000000u, 0x00000000u, 0x00000004u, 0x00000000u,
				0x3f400000u, 0x00000000u, 0x00000000u, 0x00000000u, 0x3f400000u, 0x00000000u, 0x00000001u, 0x00000000u,
				0xbf400000u, 0x00000000u, 0x00000002u, 0x00000000u, 0x3f200000u, 0x00000000u, 0x00000003u, 0x00000000u,
				0xbf200000u, 0x00000000u, 0x00000004u, 0x00000000u, 0x3f200000u, 0x00000000u, 0x00000005u, 0x00000000u,
				0xbf000000u, 0x3538000fu, 0x00000000u, 0x00000000u, 0x3f000000u, 0x34dc000fu, 0x00000001u, 0x00000000u,
				0x3f000000u, 0x35e00011u, 0xffffffffu, 0xffffffffu, 0x3f240000u, 0x359a9014u, 0x00000003u, 0x00000000u,
				0xbf720000u, 0x35a00013u, 0x00000003u, 0x00000000u, 0xbf400000u, 0x36a25015u, 0x00000001u, 0x00000000u,
				0x3f400000u, 0x358a000au, 0x00000001u, 0x00000000u, 0xbf400000u, 0x350c000au, 0x00000002u, 0x00000000u,
				0x3f400000u, 0x361e000au, 0x00000000u, 0x00000000u, 0xbf620000u, 0x35b13018u, 0x00000004u, 0x00000000u,
				0x3f0c8000u, 0x353cf016u, 0x00000005u, 0x00000000u, 0x3f600000u, 0x36b7e019u, 0x00000002u, 0x00000000u,
				0xbf000000u, 0x35000008u, 0x00000065u, 0x00000000u, 0x00000000u, 0x3f00000au, 0x00000051u, 0x00000000u,
				0x00000000u, 0x3f600010u, 0x00000050u, 0x00000000u, 0x00000000u, 0x3f000027u, 0x000000c9u, 0x00000000u,
				0x3f000000u, 0x35b0001bu, 0x000000ddu, 0x00000000u, 0x00000000u, 0x3f60003fu, 0x000000c8u, 0x00000000u,
				0x3f400000u, 0x358a000au, 0x00000001u, 0x00000000u, 0xbf400000u, 0x350c000au, 0x00000002u, 0x00000000u,
				0x3f400000u, 0x361e000au, 0x00000000u, 0x00000000u, 0xbf400000u, 0x35e70027u, 0x0000007au, 0x00000000u,
				0xbf400000u, 0x36230021u, 0x00000079u, 0x00000000u, 0x00000000u, 0x3f0a0022u, 0x00000067u, 0x00000000u,
				0xbf000000u, 0x3a832984u, 0x00000000u, 0x00000000u, 0x3f000000u, 0x3a83203eu, 0x00000001u, 0x00000000u,
				0x3f000000u, 0x3b032e85u, 0xffffffffu, 0xffffffffu, 0xbf000000u, 0x3a833dd9u, 0x00000065u, 0x00000000u,
				0xbf000000u, 0x3a834ba2u, 0x00000064u, 0x00000000u, 0x00000000u, 0x3f385e61u, 0x00000051u, 0x00000000u,
				0x3f400000u, 0x358a000au, 0x00000001u, 0x00000000u, 0xbf400000u, 0x350c000au, 0x00000002u, 0x00000000u,
				0x3f400000u, 0x361e000au, 0x00000000u, 0x00000000u, 0x3f400000u, 0x35e70027u, 0x00000066u, 0x00000000u,
				0x3f400000u, 0x36230021u, 0x00000065u, 0x00000000u, 0x00000000u, 0x3f0a0022u, 0x00000053u, 0x00000000u,
				0xbf000000u, 0x3538000fu, 0x00000000u, 0x00000000u, 0x3f000000u, 0x34dc000fu, 0x00000001u, 0x00000000u,
				0x3f000000u, 0x35e00011u, 0xffffffffu, 0xffffffffu, 0x3f000000u, 0x35580013u, 0x00000003u, 0x00000000u,
				0xbf000000u, 0x350e000au, 0x00000004u, 0x00000000u, 0xbf000000u, 0x35f00015u, 0x00000002u, 0x00000000u,
				0x3f400000u, 0x358a000au, 0x00000001u, 0x00000000u, 0xbf400000u, 0x350c000au, 0x00000002u, 0x00000000u,
				0x3f400000u, 0x361e000au, 0x00000000u, 0x00000000u, 0xbf3fffffu, 0x35a28018u, 0xffffff84u, 0xffffffffu,
				0xbf000000u, 0x36800015u, 0xffffff82u, 0xffffffffu, 0xbda00000u, 0x3f200020u, 0xffffff70u, 0xffffffffu,
				0x00000000u, 0x3f00000au, 0xffffffdcu, 0x00000001u, 0x00000000u, 0x3f600010u, 0xffffffdbu, 0x00000001u,
				0x3f000000u, 0x35000008u, 0xfffffff0u, 0x00000001u, 0xbf000000u, 0x36480025u, 0xffffffefu, 0x00000001u,
				0xbf400000u, 0x365e002au, 0xffffffefu, 0x00000001u, 0x00000000u, 0x3f14002cu, 0xffffffdcu, 0x00000001u,
				0x3f400000u, 0x358a000au, 0x00000001u, 0x00000000u, 0xbf400000u, 0x350c000au, 0x00000002u, 0x00000000u,
				0x3f400000u, 0x361e000au, 0x00000000u, 0x00000000u, 0xbf040000u, 0x36213018u, 0x00000003u, 0x00000000u,
				0x3f0c8000u, 0x3536f015u, 0x00000005u, 0x00000000u, 0x3f600000u, 0x36b3e018u, 0x00000002u, 0x00000000u,
				0x80000000u, 0x00000000u, 0x00000000u, 0x00000000u, 0x80000000u, 0x00000000u, 0x00000000u, 0x00000000u,
				0x80000000u, 0x00000000u, 0x00000000u, 0x00000000u, 0x80000000u, 0x00000000u, 0x00000000u, 0x00000000u,
				0x80000000u, 0x00000000u, 0x00000000u, 0x00000000u, 0x80000000u, 0x00000000u, 0x00000000u, 0x00000000u,
				0x00000000u, 0x7f800000u, 0x00000000u, 0x00000000u, 0x00000000u, 0x7f800000u, 0x00000000u, 0x00000000u,
				0x00000000u, 0x7f800000u, 0x00000000u, 0x00000000u, 0x00000000u, 0x7f800000u, 0x00000000u, 0x00000000u,
				0x00000000u, 0x7f800000u, 0x00000000u, 0x00000000u, 0x00000000u, 0x7f800000u, 0x00000000u, 0x00000000u,
				0x00000000u, 0x3f00000au, 0xffffffeau, 0x7fffffffu, 0x00000000u, 0x3f600010u, 0xffffffe9u, 0x7fffffffu,
				0x3f000000u, 0x35000008u, 0xfffffffeu, 0x7fffffffu, 0xbf000000u, 0x37bd8027u, 0xfffffffdu, 0x7fffffffu,
				0xbf400000u, 0x37d7802du, 0xfffffffdu, 0x7fffffffu, 0x00000000u, 0x3f1400a6u, 0xffffffeau, 0x7fffffffu,
				0x3f400000u, 0x358a000au, 0x00000001u, 0x00000000u, 0xbf400000u, 0x350c000au, 0x00000002u, 0x00000000u,
				0x3f400000u, 0x361e000au, 0x00000000u, 0x00000000u, 0xbf698000u, 0x35ced022u, 0x00000006u, 0x00000000u,
				0xbf120000u, 0x37212c1du, 0x00000003u, 0x00000000u, 0x3f600000u, 0x3772802du, 0x00000002u, 0x00000000u,
			};

			// Equivalent entry shapes distinguish serial execution, reference-induced liveness, and per-invocation isolation.
			struct Probe
			{
				char const* m_name;
				std::string m_source;
				int m_dispatch_count;
				bool m_has_reference;
			};
			auto probes = std::array{
				Probe{.m_name = "serial-reference", .m_source = source, .m_dispatch_count = 1, .m_has_reference = true},
				Probe{.m_name = "serial-production", .m_source = source, .m_dispatch_count = 1, .m_has_reference = false},
				Probe{.m_name = "isolated-reference", .m_source = source, .m_dispatch_count = 8, .m_has_reference = true},
			};
			auto& two_calls = probes[1].m_source;
			auto const reference_begin = two_calls.find("\t\tGpuCoupledConstraintWorkMotion reference;");
			auto const reference_end = two_calls.find("\t\tGpuCoupledConstraintWorkMotion actual;");
			two_calls.erase(reference_begin, reference_end - reference_begin);
			auto& isolated = probes[2].m_source;
			isolated.replace(isolated.find("void Probe()"), std::string_view("void Probe()").size(), "void Probe(uint3 dispatch_id : SV_DispatchThreadID)");
			isolated.replace(isolated.find("for (int test = 0; test != 8; ++test)\n\t{"), std::string_view("for (int test = 0; test != 8; ++test)\n\t{").size(), "{\n\t\tint test = int(dispatch_id.x);");

			// Use the shipping Debug flags with the runtime compiler and normal fenced readback path.
			auto root = std::filesystem::path(__FILE__).parent_path().parent_path().parent_path().parent_path().parent_path().parent_path();
			auto resolver = compute::shader_cache::FileSourceResolver(std::vector<std::filesystem::path>{root / "include", root / "projects" / "rylogic"});
			auto& gpu = SharedTestGpu();
			for (auto const& probe : probes)
			{
				printf("child-work-shape: %s\n", probe.m_name);
				auto bytecode = compute::ShaderCompiler{}.Source(probe.m_source).Includes(resolver).EntryPoint(L"Probe").ShaderModel(L"cs_6_0").Optimise(false).DebugInfo().Arg(L"-Gfp").Arg(L"-Gis").Arg(L"-Ges").Arg(L"-WX").Arg(L"-Zpc").Compile();
				auto signature = compute::RootSig(compute::ERootSigFlags::ComputeOnly).UAV(EUAVReg::u0).Create(gpu, "ChildWork:Sig");
				auto pso = compute::ComputePSO(signature.get(), bytecode).Create(gpu, "ChildWork:PSO");
				auto resource = D3DPtr<ID3D12Resource>{};

				// Retire the job before its pipeline/resources and inspect output only after its normal completion fence.
				auto job = GpuJob{gpu.m_gpu, "PairedChildTransport", 0xFF00AAFF, 1};
				resource = gpu.CreateResource(compute::ResDesc::Buf<GpuCoupledConstraintWorkMotion>(16, {}).usage(compute::EUsage::UnorderedAccess), job, "ChildWork:Output");
				job.m_barriers.Transition(resource.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS).Commit();
				job.m_cmd_list.SetPipelineState(pso.get());
				job.m_cmd_list.SetComputeRootSignature(signature.get());
				job.m_cmd_list.AddComputeRootUnorderedAccessView(resource);
				job.m_cmd_list.Dispatch(probe.m_dispatch_count, 1, 1);
				job.m_barriers.Transition(resource.get(), D3D12_RESOURCE_STATE_COPY_SOURCE).Commit();
				auto readback = job.m_readback.Alloc<GpuCoupledConstraintWorkMotion>(16);
				job.m_cmd_list.CopyBufferRegion(readback, resource.get(), 0);
				job.Run();

				// Anchor every actual field to fixed host words, not to a second invocation of the changed primitives.
				auto output = readback.ptr<GpuCoupledConstraintWorkMotion>();
				for (int test = 0; test != 8; ++test)
				{
					auto expected_words = std::array<uint32_t, 48>{};
					std::copy_n(control.data() + 48 * test, 48, expected_words.data());
					auto const expected = std::bit_cast<GpuCoupledConstraintWorkMotion>(expected_words);
					auto const& actual = output[2 * test + 1];
					auto const& reference = output[2 * test];
					auto const aa = std::array{&actual.delta.angular, &actual.delta.translation, &actual.midpoint.angular, &actual.midpoint.translation};
					auto const bb = std::array{&expected.delta.angular, &expected.delta.translation, &expected.midpoint.angular, &expected.midpoint.translation};
					auto const rr = std::array{&reference.delta.angular, &reference.delta.translation, &reference.midpoint.angular, &reference.midpoint.translation};
					for (int part = 0; part != 4; ++part)
					{
						for (int component = 0; component != 3; ++component)
						{
							auto const& a = aa[part]->components[component];
							auto const& b = bb[part]->components[component];
							printf("child-work: case=%d part=%d component=%d value=%08x/%08x error=%08x/%08x exponent=%08x:%08x/%08x:%08x\n",
								test, part, component, std::bit_cast<uint32_t>(a.value), std::bit_cast<uint32_t>(b.value), std::bit_cast<uint32_t>(a.error), std::bit_cast<uint32_t>(b.error),
								a.exponent.y, a.exponent.x, b.exponent.y, b.exponent.x);
							PR_EXPECT(std::bit_cast<uint32_t>(a.value) == std::bit_cast<uint32_t>(b.value));
							PR_EXPECT(std::bit_cast<uint32_t>(a.error) == std::bit_cast<uint32_t>(b.error));
							PR_EXPECT(a.exponent.x == b.exponent.x && a.exponent.y == b.exponent.y);
							if (probe.m_has_reference)
							{
								// The serial-production shape leaves the even records unwritten; inspect only records emitted by this kernel.
								auto const& r = rr[part]->components[component];
								PR_EXPECT(std::bit_cast<uint32_t>(r.value) == std::bit_cast<uint32_t>(b.value));
								PR_EXPECT(std::bit_cast<uint32_t>(r.error) == std::bit_cast<uint32_t>(b.error));
								PR_EXPECT(r.exponent.x == b.exponent.x && r.exponent.y == b.exponent.y);
							}
						}
					}
				}

				// The cyclic rotation has exact dyadic midpoint values independent of either shader implementation and the captured word fixture.
				auto const exact = std::array{1.5, -3.0, 0.75, -14.125, 17.5625, 3.5};
				auto const parts = std::array{&output[3].midpoint.angular, &output[3].midpoint.translation};
				for (int component = 0; component != 6; ++component)
				{
					auto const& value = parts[component / 3]->components[component % 3];
					auto const exponent = std::bit_cast<int64_t>((uint64_t(value.exponent.y) << 32) | value.exponent.x);
					PR_EXPECT(exponent >= -10 && exponent <= 10);
					PR_EXPECT(std::isfinite(value.error) && value.error >= 0);
					PR_EXPECT(std::ldexp(double(value.value), int(exponent)) == exact[component]);
				}
			}
		}

		// Scaled complete work retains mixed magnitudes, unavailable inputs, exact identities and signed exponent arithmetic on the device.
		PRUnitTestMethod(CancellationCountsAndLocalDomain, Extended)
		{
			auto const source = R"(
#include "physics/src/compute/coupled_constraint_work.hlsli"
RWStructuredBuffer<CoupledMeritValue> merit_probe : register(u0);
numthreads(Probe, 1, 1, 1)
void Probe()
{
	GpuConstraintRow row = (GpuConstraintRow)0;
	for (int test = 0; test != 6; ++test)
	{
		// Only active visits contribute, despite the identical 132-slot capacity in all six reductions.
		int count = test % 3 == 0 ? 1 : test % 3 == 1 ? 3 : 99;
		float scale = test < 3 ? 1.0f : 0.00000095367431640625f;
		CoupledMeritValue merit = CoupledMeritInput(0.0f);
		for (int slot = 0; slot != 132; ++slot)
		{
			if (slot >= count)
				continue;

			float residual = (slot % 3 == 0 ? 16777216.0f : slot % 3 == 1 ? 1.0f : -16777216.0f) * scale;
			row.solve.y = -residual;
			merit = CoupledMeritAdd(merit, CoupledWorkRow(row, 0.0f, 1.0f, CoupledMeritInput(0.0f)));
		}
		merit_probe[test] = merit;
	}

	// Cancellation inside a metric contraction must retain uncertainty from the lost middle unit.
	CoupledMeritValue contraction = CoupledWorkDot3(CoupledWorkInput3(float3(1, 1, 1)), CoupledWorkInput3(float3(16777216.0f, 1.0f, -16777216.0f)));
	CoupledMeritValue value = CoupledMeritAdd(CoupledMeritInput(1.0f), CoupledMeritMul(CoupledMeritInput(0.5f), contraction));
	merit_probe[6] = value;

	// Input flushing, product flushing, and a subnormal cancellation result each have different operand-scaled losses.
	value = CoupledMeritMul(CoupledMeritInput(asfloat(1u)), CoupledMeritInput(asfloat(0x7f000000u)));
	merit_probe[7] = value;
	value = CoupledMeritMul(CoupledMeritInput(asfloat(27u << 23)), CoupledMeritInput(asfloat(87u << 23)));
	merit_probe[8] = value;
	value = CoupledMeritAdd(CoupledMeritInput(asfloat(0x00800001u)), CoupledMeritInput(-asfloat(0x00800000u)));
	merit_probe[9] = value;

	// This finite zero-work case is now evaluable rather than preserving the old unscaled error overflow.
	GpuRigidBody body = (GpuRigidBody)0;
	body.os_com_and_invmass.w = 1.0f;
	GpuArticulationSpatialVector a = (GpuArticulationSpatialVector)0;
	GpuArticulationSpatialVector b = a;
	a.lin.x = asfloat(0x7b800000u);
	b.lin.x = -a.lin.x;
	value = CoupledWorkRigid(body, a, b);
	merit_probe[10] = value;

	// Empty work remains exact zero; the bound does not install a universal minimum energy allowance.
	value = CoupledWorkRow(row, 0.0f, 0.0f, CoupledMeritInput(0.0f));
	merit_probe[11] = value;

	// Raw finite identities bypass polarization, including equal FLT_MAX and equal subnormal coefficients.
	a.lin.x = asfloat(0x7f7fffffu);
	value = CoupledWorkRigid(body, a, a);
	merit_probe[12] = value;
	row.solve.w = asfloat(0x7f7fffffu);
	value = CoupledWorkRow(row, asfloat(0x7f7fffffu), asfloat(0x7f7fffffu), CoupledMeritInput(0.0f));
	merit_probe[13] = value;
	value = CoupledWorkRow(row, 0.0f, -0.0f, CoupledMeritInput(0.0f));
	merit_probe[14] = value;
	value = CoupledWorkRow(row, asfloat(1u), asfloat(1u), CoupledMeritInput(0.0f));
	merit_probe[15] = value;
	row.solve = float4(0, -1, 0, 0);
	value = CoupledWorkRow(row, asfloat(1u), asfloat(2u), CoupledMeritInput(0.0f));
	merit_probe[16] = value;

	// Static endpoints must not form an overflowing inverse-inertia product before multiplying it by zero.
	body = (GpuRigidBody)0;
	body.o2w = float4x4(1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1);
	body.inertia_inv_diagonal.x = asfloat(227u << 23);
	body.momentum_ang.x = asfloat(227u << 23);
	CoupledWorkMotion motion = CoupledWorkRigidTwist(body);
	merit_probe[17] = CoupledWorkComponent(CoupledWorkPart(motion, true), 0);
	body.state_flags = ERigidBodyStateFlags_Kinematic;
	motion = CoupledWorkRigidTwist(body);
	merit_probe[18] = CoupledWorkComponent(CoupledWorkPart(motion, true), 0);

	// Current source and softness work includes the signed prescribed contribution, not a kinetic-energy clamp.
	row.solve = float4(0, 3, 0.5f, 2);
	value = CoupledWorkRow(row, 1, 2, CoupledMeritInput(0.75f));
	merit_probe[19] = value;

	// Already consumed invalid enclosures survive every arithmetic identity, in both operand orders.
	CoupledMeritValue invalid = CoupledMeritInput(asfloat(0x7fc00000u));
	CoupledMeritValue zero = CoupledMeritInput(0);
	merit_probe[20] = CoupledMeritMul(invalid, zero);
	merit_probe[21] = CoupledMeritMul(zero, invalid);
	merit_probe[22] = CoupledMeritAdd(invalid, zero);
	merit_probe[23] = CoupledMeritAdd(zero, invalid);
	merit_probe[24] = CoupledWorkNegate(invalid);
	merit_probe[25] = CoupledWorkRow(row, 1, 1, invalid);
	merit_probe[26] = CoupledMeritMul(CoupledMeritInput(asfloat(0x7f800000u)), zero);
	merit_probe[27] = CoupledMeritNormalize(invalid);

	// Scalar exponents retain a small changing coordinate beside a huge unchanged one.
	body = (GpuRigidBody)0;
	body.os_com_and_invmass.w = 1;
	a = (GpuArticulationSpatialVector)0;
	b = a;
	a.lin.xy = float2(asfloat(247u << 23), asfloat(27u << 23));
	b.lin.xy = float2(a.lin.x, asfloat(28u << 23));
	merit_probe[28] = CoupledWorkRigid(body, a, b);
	a.lin.xy = float2(0, asfloat(26u << 23));
	b.lin.xy = float2(asfloat(27u << 23), 0);
	merit_probe[29] = CoupledWorkRigid(body, a, b);

	// Check both sides of the precision-derived alignment branch with uncertain signed operands.
	for (int test = 0; test != 4; ++test)
	{
		CoupledMeritValue small = CoupledMeritInput(test < 2 ? 1.5f : -1.5f);
		small.error = asfloat(104u << 23);
		small.exponent = CoupledWorkExponent(-24 - test % 2);
		merit_probe[30 + test] = CoupledMeritAdd(CoupledMeritInput(1.0f), small);
	}

	// Decoding raw subnormal significands is exact and independent of input-operand FTZ.
	merit_probe[34] = CoupledMeritInput(asfloat(1u));
	merit_probe[35] = CoupledMeritInput(asfloat(0x807fffffu));
	value = CoupledMeritMul(CoupledMeritInput(asfloat(227u << 23)), CoupledMeritInput(asfloat(227u << 23)));
	merit_probe[36] = value;
	merit_probe[37] = CoupledMeritInput(CoupledMeritDiagnostic(value));
	value = CoupledMeritInput(asfloat(1u));
	merit_probe[38] = CoupledMeritInput(CoupledMeritDiagnostic(value));
	merit_probe[39] = CoupledMeritInput(CoupledMeritDiagnostic(invalid));

	// Carry, borrow, sign and overflow use raw two-word outputs, not a float comparison of large integers.
	uint2 exponents_a[8] = {uint2(0xffffffffu, 0u), uint2(0u, 1u), uint2(0xffffffffu, 0xffffffffu), uint2(0u, 0u),
		uint2(0xffffffffu, 0x7fffffffu), uint2(0u, 0x80000000u), uint2(0u, 0x80000000u), uint2(0xffffffffu, 0x7fffffffu)};
	uint2 exponents_b[8] = {uint2(1u, 0u), uint2(1u, 0u), uint2(1u, 0u), uint2(1u, 0u),
		uint2(1u, 0u), uint2(1u, 0u), uint2(0u, 0x80000000u), uint2(0u, 0x80000000u)};
	for (int test = 0; test != 8; ++test)
	{
		value = zero;
		bool valid = test % 2 == 0 ? CoupledWorkExponentAdd(exponents_a[test], exponents_b[test], value.exponent) :
			CoupledWorkExponentSubtract(exponents_a[test], exponents_b[test], value.exponent);
		value.value = valid ? 1 : 0;
		value.error = CoupledWorkExponentLess(exponents_a[test], exponents_b[test]) ? 1 : 0;
		merit_probe[40 + test] = value;
	}
}
)";
			auto root = std::filesystem::path(__FILE__).parent_path().parent_path().parent_path().parent_path().parent_path().parent_path();
			auto resolver = compute::shader_cache::FileSourceResolver(std::vector<std::filesystem::path>{
				std::filesystem::path(__FILE__).parent_path().parent_path() / "compute",
				root / "include",
				root / "projects" / "rylogic",
			});
			auto bytecode = compute::ShaderCompiler{}.Source(source).Includes(resolver).EntryPoint(L"Probe").ShaderModel(L"cs_6_0").Optimise(false).DebugInfo().Arg(L"-Gfp").Compile();
			auto& gpu = SharedTestGpu();
			auto signature = compute::RootSig(compute::ERootSigFlags::ComputeOnly).UAV(EUAVReg::u0).Create(gpu, "CoupledMerit:Sig");
			auto pso = compute::ComputePSO(signature.get(), bytecode).Create(gpu, "CoupledMerit:PSO");
			auto resource = D3DPtr<ID3D12Resource>{};
			// Terminal job retirement precedes local pipeline/resource destruction, including failed Run or failed recording.
			auto job = GpuJob{gpu.m_gpu, "CoupledMeritArithmeticTests", 0xFF00AAFF, 1};
			resource = gpu.CreateResource(compute::ResDesc::Buf<CoupledMeritValue>(48, {}).usage(compute::EUsage::UnorderedAccess), job, "CoupledMerit:Output");
			job.m_barriers.Transition(resource.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS).Commit();
			job.m_cmd_list.SetPipelineState(pso.get());
			job.m_cmd_list.SetComputeRootSignature(signature.get());
			job.m_cmd_list.AddComputeRootUnorderedAccessView(resource);
			job.m_cmd_list.Dispatch(1, 1, 1);
			job.m_barriers.Transition(resource.get(), D3D12_RESOURCE_STATE_COPY_SOURCE).Commit();
			auto readback = job.m_readback.Alloc<CoupledMeritValue>(48);
			job.m_cmd_list.CopyBufferRegion(readback, resource.get(), 0);
			job.Run();

			// Exact dyadic inputs make the independent double sums exact, including the terms binary32 cancellation loses.
			auto const output = readback.ptr<CoupledMeritValue>();
			auto values = std::array<double, 40>{};
			auto errors = std::array<double, 40>{};
			for (int test = 0; test != 48; ++test)
			{
				auto const& record = output[test];
				printf("range-merit: case=%d value=%.17g bound=%.17g exponent_low=%u exponent_high=%u\n",
					test, double(record.value), double(record.error), record.exponent.x, record.exponent.y);
				if (test >= 40)
					continue;

				auto const exponent = std::bit_cast<int64_t>((uint64_t(record.exponent.y) << 32) | record.exponent.x);
				PR_EXPECT(exponent > -1024 && exponent < 1024);
				values[test] = std::ldexp(double(record.value), int(exponent));
				errors[test] = std::ldexp(double(record.error), int(exponent));
			}

			for (int test = 0; test != 6; ++test)
			{
				auto const count = test % 3 == 0 ? 1 : test % 3 == 1 ? 3 : 99;
				auto const scale = test < 3 ? 1.0 : std::ldexp(1.0, -20);
				auto const exact = (count == 1 ? 16777216.0 : double(count / 3)) * scale;
				auto const magnitude = (count == 1 ? 16777216.0 : double(count / 3) * 33554433.0) * scale;
				PR_EXPECT(std::isfinite(errors[test]) && errors[test] > 0.0);
				PR_EXPECT(std::abs(values[test] - exact) <= errors[test]);
				PR_EXPECT(errors[test] < 0.001 * magnitude);
			}
			PR_EXPECT(values[1] == 0.0 && values[2] == 0.0);
			PR_EXPECT(errors[2] > errors[1] && errors[1] > errors[0]);
			PR_EXPECT(values[6] == 1.0 && std::abs(values[6] - 1.5) <= errors[6]);
			for (int test = 7; test != 10; ++test)
			{
				auto const exact = std::ldexp(1.0, test == 7 ? -22 : test == 8 ? -140 : -149);
				PR_EXPECT(values[test] == exact && std::isfinite(errors[test]));
				PR_EXPECT(errors[test] < exact * 1.0e-5);
			}
			PR_EXPECT(values[10] == 0.0 && std::isfinite(errors[10]));
			PR_EXPECT(values[11] == 0.0 && errors[11] == 0.0);
			for (int test = 12; test != 16; ++test)
				PR_EXPECT(values[test] == 0.0 && errors[test] == 0.0);

			PR_EXPECT(values[16] == std::ldexp(1.0, -149) && errors[16] < std::ldexp(1.0, -160));
			PR_EXPECT(values[17] == 0.0 && errors[17] == 0.0);
			PR_EXPECT(values[18] == std::ldexp(1.0, 100) && errors[18] == 0.0);
			PR_EXPECT(std::isfinite(errors[19]) && std::abs(values[19] - 1.25) <= errors[19]);
			for (int test = 20; test != 28; ++test)
				PR_EXPECT(std::isfinite(values[test]) && std::isinf(errors[test]));

			PR_EXPECT(values[28] > errors[28] && std::abs(values[28] - 3 * std::ldexp(1.0, -201)) <= errors[28]);
			PR_EXPECT(values[29] > errors[29] && std::abs(values[29] - 3 * std::ldexp(1.0, -203)) <= errors[29]);
			for (int test = 0; test != 4; ++test)
			{
				auto const scale = std::ldexp(1.0, -24 - test % 2);
				auto const exact = 1.0 + (test < 2 ? 0.75 : -0.75) * scale;
				PR_EXPECT(std::abs(values[30 + test] - exact) + std::ldexp(scale, -23) <= errors[30 + test]);
			}
			PR_EXPECT(values[34] == std::ldexp(1.0, -149) && errors[34] == 0);
			PR_EXPECT(values[35] == -(std::ldexp(1.0, -126) - std::ldexp(1.0, -149)) && errors[35] == 0);
			PR_EXPECT(values[36] == std::ldexp(1.0, 200) && std::isfinite(errors[36]));
			PR_EXPECT(values[37] == 0 && errors[37] == 0);
			PR_EXPECT(values[38] == std::ldexp(1.0, -149) && errors[38] == 0);
			PR_EXPECT(values[39] == 0 && errors[39] == 0);

			// Unsigned reference arithmetic independently checks wrapping bits and signed overflow/sign ordering.
			auto const aa = std::array<uint64_t, 8>{0xffffffffull, 0x100000000ull, UINT64_MAX, 0, INT64_MAX, 0x8000000000000000ull, 0x8000000000000000ull, INT64_MAX};
			auto const bb = std::array<uint64_t, 8>{1, 1, 1, 1, 1, 1, 0x8000000000000000ull, 0x8000000000000000ull};
			for (int test = 0; test != 8; ++test)
			{
				auto const expected = test % 2 == 0 ? aa[test] + bb[test] : aa[test] - bb[test];
				auto const& record = output[40 + test];
				PR_EXPECT(((uint64_t(record.exponent.y) << 32) | record.exponent.x) == expected);
				PR_EXPECT(record.value == (test < 4 ? 1.0f : 0.0f));
				PR_EXPECT(record.error == (std::bit_cast<int64_t>(aa[test]) < std::bit_cast<int64_t>(bb[test]) ? 1.0f : 0.0f));
			}
		}
	};

	// Exercise the shared shader transaction arithmetic without claiming a migrated solver or GPU resource-commit boundary.
	PRUnitTestClass(ConstraintTransactionCoreTests)
	{
		// Exercise epoch reset, no-effect wake, whole-block rejection/retry and zero-row retirement through the numerical stage.
		PRUnitTestMethod(RigidBlockFreshEpochAndRetirement, Quick)
		{
			// One linear damper has an exact dyadic solution: relative speed 4 -> 2 with gamma=2 and two unit masses.
			auto a = GpuRigidBody{};
			auto b = GpuRigidBody{};
			a.o2w = b.o2w = m4x4::Identity();
			a.os_com_and_invmass.w = b.os_com_and_invmass.w = 1.0f;
			b.momentum_lin.x = 4.0f;
			auto epoch = GpuConstraintEpoch{.identity = uint2(1, 8), .timestep = 0.25f, .warm_start_factor = 0.25f};
			auto block = GpuConstraintBlock{.body_idx_a = 0, .body_idx_b = 1, .velocity_mask = 1u, .row_states = 3u};
			auto inverse = GpuConstraintSolveCache{.row_count = 1};
			inverse.inverse[0] = 0.25f;
			GpuConstraintRow rows[6]{};
			rows[0].jacobian_a_lin.x = -1.0f;
			rows[0].jacobian_b_lin.x = 1.0f;
			rows[0].solve.w = 2.0f;
			rows[0].bounds = float4(-100, 100, 0, 0);
			auto accepted = GpuConstraintRigidRecord{};
			auto next = GpuConstraintRigidRecord{};
			auto next_a = GpuRigidBody{};
			auto next_b = GpuRigidBody{};
			PR_EXPECT(ConstraintStageRigidBlock(block, inverse, rows, a, b, accepted, epoch, 1.0f, next_a, next_b, next) == GpuConstraintStage_Accepted);
			PR_EXPECT(next_a.momentum_lin.x == 1.0f && next_b.momentum_lin.x == 3.0f);
			PR_EXPECT(next.rows.impulses[0] == -1.0f);
			a = next_a;
			b = next_b;
			accepted = next;
			a.state_flags = b.state_flags = ERigidBodyStateFlags_Sleeping;
			a.sleep.island_id = b.sleep.island_id = 17;
			a.sleep.generation = b.sleep.generation = 9;

			// A numerically exact replay leaves every body byte untouched despite a nonzero provisional warm seed.
			PR_EXPECT(ConstraintStageRigidBlock(block, inverse, rows, a, b, accepted, epoch, 1.0f, next_a, next_b, next) == GpuConstraintStage_Accepted);
			PR_EXPECT(std::memcmp(&a, &next_a, sizeof(a)) == 0);
			PR_EXPECT(std::memcmp(&b, &next_b, sizeof(b)) == 0);

			// A new substep keeps the first substep's physical history; its actual new response halves relative speed again.
			++epoch.identity.x;
			PR_EXPECT(ConstraintStageRigidBlock(block, inverse, rows, a, b, accepted, epoch, 1.0f, next_a, next_b, next) == GpuConstraintStage_Accepted);
			PR_EXPECT(next_a.momentum_lin.x == 1.5f && next_b.momentum_lin.x == 2.5f);
			PR_EXPECT(next.rows.impulses[0] == -0.5f);
			PR_EXPECT(next.response_a.lin.x == 0.5f && next.response_b.lin.x == -0.5f);
			PR_EXPECT(next_a.sleep.generation == 10 && next_b.sleep.generation == 10);
			a = next_a;
			b = next_b;
			accepted = next;

			// Overflow in endpoint B's free-state recovery rejects the entire block, not just that endpoint.
			auto invalid_b = b;
			invalid_b.momentum_lin.x = std::numeric_limits<float>::max();
			auto invalid_record = accepted;
			invalid_record.response_b.lin.x = -std::numeric_limits<float>::max();
			auto saved_a = a;
			auto saved_b = invalid_b;
			auto saved_record = invalid_record;
			PR_EXPECT(ConstraintStageRigidBlock(block, inverse, rows, a, invalid_b, invalid_record, epoch, 1.0f, next_a, next_b, next) == GpuConstraintStage_AdmissionRejected);
			PR_EXPECT(std::memcmp(&a, &saved_a, sizeof(a)) == 0);
			PR_EXPECT(std::memcmp(&invalid_b, &saved_b, sizeof(invalid_b)) == 0);
			PR_EXPECT(std::memcmp(&invalid_record, &saved_record, sizeof(invalid_record)) == 0);
			PR_EXPECT(rows[0].bounds.z == 0.0f);
			PR_EXPECT(ConstraintStageRigidBlock(block, inverse, rows, a, b, accepted, epoch, 1.0f, next_a, next_b, next) == GpuConstraintStage_Accepted);
			PR_EXPECT(next_a.momentum_lin.x == a.momentum_lin.x && next_b.momentum_lin.x == b.momentum_lin.x);

			// A zero-row visit retires the current epoch only, never the previous substep's already integrated history.
			block.velocity_mask = block.row_states = 0u;
			inverse.row_count = 0;
			PR_EXPECT(ConstraintStageRigidBlock(block, inverse, rows, a, b, accepted, epoch, 1.0f, next_a, next_b, next) == GpuConstraintStage_Accepted);
			PR_EXPECT(next_a.momentum_lin.x == 1.0f && next_b.momentum_lin.x == 3.0f);
			PR_EXPECT(next.rows.impulses[0] == 0.0f && next.rows.linear_impulse_norm == 0.0f);
			PR_EXPECT(next.response_a.lin.x == 0.0f && next.response_b.lin.x == 0.0f);
			PR_EXPECT(ConstraintAcceptedInEpoch(next.accepted, epoch));
		}

		// Solve two moving endpoints before and after changing basis/mobility, using physical impulse balances as the oracle.
		PRUnitTestMethod(RigidBlockRebasesActualSolve, Quick)
		{
			// The two finite inertias are unequal; neither endpoint is a world anchor.
			auto a = GpuRigidBody{};
			auto b = GpuRigidBody{};
			a.o2w = b.o2w = m4x4::Identity();
			a.os_com_and_invmass.w = b.os_com_and_invmass.w = 1.0f;
			a.inertia_inv_diagonal = float4(1.0f, 0.5f, 1.0f / 3.0f, 0);
			b.inertia_inv_diagonal = float4(0.5f, 0.25f, 1.0f / 3.0f, 0);
			a.momentum_ang = float4(1, -2, 0.5f, 0);
			b.momentum_ang = float4(-2, 3, 1, 0);
			a.momentum_lin = float4(0.25f, -0.5f, 0.75f, 0);
			b.momentum_lin = float4(-1, 2, -3, 0);
			auto epoch = GpuConstraintEpoch{.identity = uint2(1, 5), .timestep = 1.0f / 60.0f, .warm_start_factor = 0.25f};
			auto block = GpuConstraintBlock{.body_idx_a = 0, .body_idx_b = 1, .velocity_mask = 7u, .row_states = 63u};
			auto inverse = GpuConstraintSolveCache{.active_axes = 0x210u, .row_count = 3};
			GpuConstraintRow rows[6]{};
			for (int i = 0; i != 3; ++i)
			{
				// Three independent physical damper directions, with gamma = 1/(dt*c) = 10.
				rows[i].jacobian_a_ang[i] = -1.0f;
				rows[i].jacobian_b_ang[i] = 1.0f;
				rows[i].solve.w = 10.0f;
				rows[i].bounds = float4(-100, 100, 0, 0);
				inverse.inverse[6 * i + i] = 1.0f / (10.0f + a.inertia_inv_diagonal[i] + b.inertia_inv_diagonal[i]);
			}
			auto accepted = GpuConstraintRigidRecord{};
			auto next = GpuConstraintRigidRecord{};
			auto next_a = GpuRigidBody{};
			auto next_b = GpuRigidBody{};
			auto free_a = a.momentum_ang;
			auto free_b = b.momentum_ang;
			PR_EXPECT(ConstraintStageRigidBlock(block, inverse, rows, a, b, accepted, epoch, 1.0f, next_a, next_b, next) == GpuConstraintStage_Accepted);
			for (int i = 0; i != 3; ++i)
			{
				// This diagonal scalar solution is independent of the shared block projection.
				auto const wa = static_cast<double>(a.inertia_inv_diagonal[i]);
				auto const wb = static_cast<double>(b.inertia_inv_diagonal[i]);
				auto const impulse = -(wb * free_b[i] - wa * free_a[i]) / (10.0 + wa + wb);
				PR_EXPECT(std::abs(next_a.momentum_ang[i] - (free_a[i] - impulse)) < 5.0e-7);
				PR_EXPECT(std::abs(next_b.momentum_ang[i] - (free_b[i] + impulse)) < 5.0e-7);
			}
			a = next_a;
			b = next_b;
			accepted = next;

			// Replacing an unchanged accepted solve does not apply viscosity for a second dt.
			PR_EXPECT(ConstraintStageRigidBlock(block, inverse, rows, a, b, accepted, epoch, 1.0f, next_a, next_b, next) == GpuConstraintStage_Accepted);
			PR_EXPECT(FEqlAbsolute(next_a.momentum_ang, a.momentum_ang, 5.0e-7f));
			PR_EXPECT(FEqlAbsolute(next_b.momentum_ang, b.momentum_ang, 5.0e-7f));

			// Contact-like primary-state increments belong outside this block and survive the subsequent replacement.
			auto const external_a = float4(0.125f, -0.25f, 0.5f, 0);
			auto const external_b = float4(-0.375f, 0.125f, -0.25f, 0);
			a.momentum_ang += external_a;
			b.momentum_ang += external_b;
			free_a += external_a;
			free_b += external_b;
			a.momentum_lin += float4(1, 2, 3, 0);
			b.momentum_lin += float4(-3, -2, -1, 0);
			auto const linear_a = a.momentum_lin;
			auto const linear_b = b.momentum_lin;
			auto const c = std::cos(0.6);
			auto const s = std::sin(0.6);
			double wa[3][3] = {{1, 0, 0}, {0, 0.5, 0}, {0, 0, 1.0 / 3.0}};
			double wb[3][3] = {{0.25*c*c+0.5*s*s, -0.25*c*s, 0}, {-0.25*c*s, 0.25*s*s+0.5*c*c, 0}, {0, 0, 0.125}};
			b.o2w = m4x4::Transform(v4::ZAxis(), 0.6f, v4::Origin());
			b.inertia_inv_diagonal = float4(0.25f, 0.5f, 0.125f, 0);
			rows[0].jacobian_b_ang = float4(static_cast<float>(c), static_cast<float>(s), 0, 0);
			rows[1].jacobian_b_ang = float4(static_cast<float>(-s), static_cast<float>(c), 0, 0);
			float matrix[36]{};
			for (int i = 0; i != 3; ++i)
			{
				// Independent physical mobility is projected into the new orthonormal row basis only to prepare the normal inverse input.
				rows[i].jacobian_a_ang = -rows[i].jacobian_b_ang;
				for (int j = 0; j != 3; ++j)
				{
					// Preserve the world-vector operator rather than assuming the changed row basis diagonalizes both inertias.
					auto value = i == j ? 10.0 : 0.0;
					for (int x = 0; x != 3; ++x)
					{
						// Form J_i (W_a + W_b) J_j^T.
						for (int y = 0; y != 3; ++y)
						{
							// The physical matrices above are independent of BodyTwistWS.
							value += rows[i].jacobian_b_ang[x] * (wa[x][y] + wb[x][y]) * rows[j].jacobian_b_ang[y];
						}
					}
					matrix[6*i+j] = static_cast<float>(value);
				}
			}
			PR_EXPECT(constraint_algebra::InvertConstraintMatrix(matrix, 3, 1.0e-6f, inverse.inverse));
			auto before_a = a;
			auto before_b = b;
			auto before_record = accepted;
			auto prepared_rows = std::array<GpuConstraintRow, 6>{};
			std::copy_n(rows, 6, prepared_rows.data());
			PR_EXPECT(ConstraintStageRigidBlock(block, inverse, rows, a, b, accepted, epoch, 1.0f, next_a, next_b, next) == GpuConstraintStage_Accepted);
			auto impulse_b = next_b.momentum_ang - free_b;
			auto actual_delta_b = next_b.momentum_ang - before_b.momentum_ang;
			for (int x = 0; x != 3; ++x)
			{
				// The constitutive law uses actual physical impulse, not new Jacobians times the disputed cached scalars.
				auto relative_speed = 0.0;
				for (int y = 0; y != 3; ++y)
				{
					// Evaluate each body's world inverse inertia independently.
					relative_speed += wb[x][y] * next_b.momentum_ang[y] - wa[x][y] * next_a.momentum_ang[y];
				}
				PR_EXPECT(std::abs(impulse_b[x] + 0.1 * relative_speed) < 2.0e-6);
				PR_EXPECT(std::abs((next_a.momentum_ang[x] - free_a[x]) + impulse_b[x]) < 2.0e-6);
				PR_EXPECT(std::abs(accepted.response_b.ang[x] + actual_delta_b[x] - impulse_b[x]) < 2.0e-6);
			}
			PR_EXPECT(All(next_a.momentum_lin == linear_a));
			PR_EXPECT(All(next_b.momentum_lin == linear_b));
			PR_EXPECT(std::memcmp(rows, prepared_rows.data(), sizeof(rows)) == 0);
			PR_EXPECT(std::memcmp(&a, &before_a, sizeof(a)) == 0);
			PR_EXPECT(std::memcmp(&b, &before_b, sizeof(b)) == 0);
			PR_EXPECT(std::memcmp(&accepted, &before_record, sizeof(accepted)) == 0);
			log_stream() << "rigid numerical rebase: prior response=" << accepted.response_b.ang << ", actual delta=" << actual_delta_b << ", total physical impulse=" << impulse_b << '\n';
		}

		// Dispatch the shared core in unchanged Debug shader settings; this is not a production persistent-solver dispatch.
		PRUnitTestMethod(HardwareStagesWithoutPublishing, Extended)
		{
			// Compile the real helper, resolving its dependencies from this checkout instead of embedding a test copy.
			auto const source = R"(
#include "constraint_transaction.hlsli"
RWStructuredBuffer<float> output : register(u0);

// Probe temporal provenance, prospective state, and zero-row retirement through the shared shader core.
numthreads(Probe, 1, 1, 1)
void Probe()
{
	// Historical coefficients retain their accepted timestep through rejected preparation.
	GpuConstraintEpoch epoch;
	epoch.identity = uint2(7, 2);
	epoch.timestep = 0.08f;
	epoch.warm_start_factor = 0.25f;
	GpuConstraintAcceptance accepted = ConstraintAcceptance(epoch);
	epoch.identity.x++;
	epoch.timestep = 0.04f;
	float scale;
	output[0] = ConstraintStageGuessScale(accepted, epoch, scale) ? scale : -1.0f;
	epoch.timestep = 0.02f;
	output[1] = ConstraintStageGuessScale(accepted, epoch, scale) ? scale : -1.0f;
	float free_value;
	output[2] = ConstraintStageFreeComponent(3.0f, -0.5f, accepted, epoch, free_value) ? free_value : -1.0f;
	accepted = ConstraintAcceptance(epoch);
	output[3] = ConstraintStageGuessScale(accepted, epoch, scale) ? scale : -1.0f;

	// A nonzero seed replay has no physical effect and cannot wake a sleeping body.
	GpuRigidBody live = (GpuRigidBody)0;
	live.momentum_ang = float4(1, 2, 3, 0);
	live.state_flags = ERigidBodyStateFlags_Sleeping;
	live.sleep.island_id = 17;
	live.sleep.generation = 11u;
	GpuArticulationSpatialVector previous;
	previous.ang = float4(0.5f, -0.25f, 0.125f, 0);
	previous.lin = float4(0, 0, 0, 0);
	GpuRigidBody candidate;
	GpuArticulationSpatialVector response;
	output[4] = ConstraintStageRigidEndpoint(live, previous, previous, accepted, epoch, candidate, response) ? 1.0f : 0.0f;
	output[5] = (float)candidate.sleep.generation;
	output[6] = (float)candidate.sleep.island_id;
	previous.ang.x += 0.125f;
	GpuArticulationSpatialVector original = previous;
	original.ang.x -= 0.125f;
	output[7] = ConstraintStageRigidEndpoint(live, original, previous, accepted, epoch, candidate, response) ? candidate.momentum_ang.x : -1.0f;
	output[8] = (float)candidate.sleep.generation;
	output[9] = live.momentum_ang.x;

	// Candidate arithmetic failure does not alter the original stamp or live input.
	GpuConstraintStagedComponent component;
	float largest = asfloat(0x7f7fffffu);
	output[10] = ConstraintStageReplacement(1.0f, -largest, largest, accepted, epoch, component) ? 1.0f : 0.0f;
	output[11] = accepted.timestep;

	// Linear lever moments do not enter the pure-angular load, and inactive rows clear nonzero cached coefficients.
	GpuConstraintBlock block = (GpuConstraintBlock)0;
	block.velocity_mask = 7u;
	GpuConstraintRow rows[6];
	for (int i = 0; i != 6; ++i)
	{
		// Explicit initialization keeps padding and inactive slots deterministic.
		rows[i] = (GpuConstraintRow)0;
		rows[i].bounds = float4(-10, 10, 0, 0);
	}
	rows[0].jacobian_b_lin = float4(1, 0, 0, 0);
	rows[0].jacobian_b_ang = float4(0, 0, 100, 0);
	rows[0].bounds.z = 2.0f;
	rows[1].jacobian_b_ang = float4(0, 1, 0, 0);
	rows[1].bounds.z = 3.0f;
	rows[2].jacobian_b_ang = float4(0, 1, 0, 0);
	rows[2].bounds.z = -3.0f;
	GpuConstraintAcceptedRows cache;
	output[12] = ConstraintStageAcceptedRows(block, rows, cache) ? cache.linear_impulse_norm : -1.0f;
	output[13] = cache.angular_impulse_norm;
	block.velocity_mask = 0u;
	output[14] = ConstraintStageAcceptedRows(block, rows, cache) ? cache.impulses[1] : -1.0f;
	output[15] = cache.impulses[2];

	// Solve the actual shared numerical step for two moving endpoints with unequal inertia.
	GpuRigidBody a = (GpuRigidBody)0;
	GpuRigidBody b = (GpuRigidBody)0;
	a.o2w = b.o2w = float4x4(1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1);
	a.os_com_and_invmass.w = b.os_com_and_invmass.w = 1.0f;
	a.inertia_inv_diagonal = float4(1, 0.5f, 1.0f/3.0f, 0);
	b.inertia_inv_diagonal = float4(0.5f, 0.25f, 1.0f/3.0f, 0);
	a.momentum_ang = float4(1, -2, 0.5f, 0);
	b.momentum_ang = float4(-2, 3, 1, 0);
	block.body_idx_a = 0;
	block.body_idx_b = 1;
	block.velocity_mask = 7u;
	block.row_states = 63u;
	GpuConstraintSolveCache inverse = (GpuConstraintSolveCache)0;
	inverse.row_count = 3;
	inverse.active_axes = 0x210u;
	for (int axis = 0; axis != 6; ++axis)
	{
		// A three-row isotropic physical damper leaves the other canonical slots empty.
		rows[axis] = (GpuConstraintRow)0;
		if (axis < 3)
		{
			// gamma=10; independent diagonal initial mobility.
			rows[axis].jacobian_a_ang[axis] = -1.0f;
			rows[axis].jacobian_b_ang[axis] = 1.0f;
			rows[axis].solve.w = 10.0f;
			rows[axis].bounds = float4(-100, 100, 0, 0);
			inverse.inverse[6*axis+axis] = 1.0f/(10.0f+a.inertia_inv_diagonal[axis]+b.inertia_inv_diagonal[axis]);
		}
	}
	GpuConstraintRigidRecord old_record = (GpuConstraintRigidRecord)0;
	GpuConstraintRigidRecord new_record;
	GpuRigidBody next_a;
	GpuRigidBody next_b;
	output[16] = (float)ConstraintStageRigidBlock(block, inverse, rows, a, b, old_record, epoch, 1.0f, next_a, next_b, new_record);
	for (int axis = 0; axis != 3; ++axis)
	{
		// Read actual proposed physical momenta, not just row residuals or coefficients.
		output[17+axis] = next_a.momentum_ang[axis];
		output[20+axis] = next_b.momentum_ang[axis];
	}
	a = next_a;
	b = next_b;
	old_record = new_record;

	// Change the basis and physical mobility, preserving an interleaved external angular impulse.
	b.momentum_ang += float4(0.25f, 0.5f, -0.125f, 0);
	b.o2w = float4x4(0,1,0,0, -1,0,0,0, 0,0,1,0, 0,0,0,1);
	b.inertia_inv_diagonal = float4(0.125f, 0.25f, 0.5f, 0);
	rows[0].jacobian_b_ang = float4(0, 1, 0, 0);
	rows[1].jacobian_b_ang = float4(-1, 0, 0, 0);
	rows[0].jacobian_a_ang = -rows[0].jacobian_b_ang;
	rows[1].jacobian_a_ang = -rows[1].jacobian_b_ang;
	inverse.inverse[0] = 1.0f / 10.625f;
	inverse.inverse[7] = 1.0f / 11.25f;
	inverse.inverse[14] = 1.0f / (10.0f+1.0f/3.0f+0.5f);
	output[23] = (float)ConstraintStageRigidBlock(block, inverse, rows, a, b, old_record, epoch, 1.0f, next_a, next_b, new_record);
	for (int axis = 0; axis != 3; ++axis)
	{
		// Closed-form world-axis impulse equations provide the native-side expected values.
		output[24+axis] = next_a.momentum_ang[axis];
		output[27+axis] = next_b.momentum_ang[axis];
	}

	// New physical-admission rejection must not be confused with an existing projection arithmetic failure.
	GpuConstraintRigidRecord impossible = old_record;
	impossible.response_a.ang.x = -largest;
	GpuRigidBody extreme = a;
	extreme.momentum_ang.x = largest;
	output[30] = (float)ConstraintStageRigidBlock(block, inverse, rows, extreme, b, impossible, epoch, 1.0f, next_a, next_b, new_record);
	inverse.inverse[0] = asfloat(0x7fc00000u);
	output[31] = (float)ConstraintStageRigidBlock(block, inverse, rows, a, b, old_record, epoch, 1.0f, next_a, next_b, new_record);
}
)";
			auto root = std::filesystem::path(__FILE__).parent_path().parent_path().parent_path().parent_path().parent_path().parent_path();
			auto resolver = compute::shader_cache::FileSourceResolver(std::vector<std::filesystem::path>{
				std::filesystem::path(__FILE__).parent_path().parent_path() / "compute",
				root / "include",
				root / "projects" / "rylogic",
			});
			auto bytecode = compute::ShaderCompiler{}.Source(source).Includes(resolver).EntryPoint(L"Probe").ShaderModel(L"cs_6_0").Optimise(false).DebugInfo().Arg(L"-Gfp").Compile();
			auto& gpu = SharedTestGpu();
			auto& job = gpu.m_job;
			auto signature = compute::RootSig(compute::ERootSigFlags::ComputeOnly).UAV(EUAVReg::u0).Create(gpu, "ConstraintTransactionCore:Sig");
			auto pso = compute::ComputePSO(signature.get(), bytecode).Create(gpu, "ConstraintTransactionCore:PSO");
			auto resource = D3DPtr<ID3D12Resource>{};
			auto readback = ReadbackAlloc{};
			try
			{
				// Keep resource owners alive while retiring an interrupted recording prefix.
				resource = gpu.CreateResource(compute::ResDesc::Buf<float>(32, {}).usage(compute::EUsage::UnorderedAccess), job, "ConstraintTransactionCore:Output");
				job.m_barriers.Transition(resource.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS).Commit();
				job.m_cmd_list.SetPipelineState(pso.get());
				job.m_cmd_list.SetComputeRootSignature(signature.get());
				job.m_cmd_list.AddComputeRootUnorderedAccessView(resource);
				job.m_cmd_list.Dispatch(1, 1, 1);
				job.m_barriers.Transition(resource.get(), D3D12_RESOURCE_STATE_COPY_SOURCE).Commit();
				readback = job.m_readback.Alloc<float>(32);
				job.m_cmd_list.CopyBufferRegion(readback, resource.get(), 0);
			}
			catch (...)
			{
				// Cleanup cannot turn the original test failure into a different exception.
				auto error = std::current_exception();
				try
				{
					// Submission occurs only here for an interrupted recording, never after Run has taken ownership.
					job.RetireRecordedWork();
				}
				catch (std::exception const& cleanup_error)
				{
					// Preserve both errors without claiming that cleanup succeeded.
					log_stream() << "Recording retirement failed: " << cleanup_error.what() << '\n';
				}
				std::rethrow_exception(error);
			}

			// The normal submission sits outside the recording-only cleanup; assertions occur after completion.
			job.Run();
			auto expected = std::array{0.125f, 0.0625f, 3.0f, 1.0f, 1.0f, 11.0f, 17.0f, 1.125f, 12.0f, 1.0f, 0.0f, 0.02f, 2.0f, 0.0f, 0.0f, 0.0f};
			auto output = readback.ptr<float>();
			for (size_t i = 0; i != expected.size(); ++i)
			{
				// Compare actual device output to independent expected observations, not a CPU replay of the helper.
				PR_EXPECT(output[i] == expected[i]);
			}
			auto const physical_expected = std::array{
				0.0, 1.0-2.0/11.5, -2.0+1.75/10.75, 0.515625, -2.0+2.0/11.5, 3.0-1.75/10.75, 0.984375,
				0.0, 1.0-1.4375/11.25, -2.0+1.4375/10.625, 0.525, -1.75+1.4375/11.25, 3.5-1.4375/10.625, 0.85, 1.0, 2.0,
			};
			for (size_t i = 0; i != physical_expected.size(); ++i)
			{
				// Independent two-body impulse balance tests the real projection, basis/mobility rebase, and distinct failure classes.
				log_stream() << "D3D12 numerical output[" << (i + 16) << "]=" << output[i + 16] << ", expected=" << physical_expected[i] << '\n';
				PR_EXPECT(std::abs(output[i + 16] - physical_expected[i]) < 2.0e-6);
			}
			log_stream() << "D3D12 shared-core probe: 32 independent outputs match; cs_6_0 -Od -Zi -Gfp; production owner storage not dispatched\n";
		}

		// Rejected preparations cannot advance the accepted timestep or turn historical guesses into current-substep responses.
		PRUnitTestMethod(AcceptedTimeSurvivesRejectedAttempts, Quick)
		{
			// Accept at h, then attempt h/2 and h/4 in a fresh epoch without publishing either attempt.
			auto epoch = GpuConstraintEpoch{.identity = uint2(7, 2), .timestep = 0.08f, .warm_start_factor = 0.25f};
			auto accepted = ConstraintAcceptance(epoch);
			auto const original = accepted;
			auto scale = 0.0f;
			epoch.identity.x++;
			epoch.timestep = 0.04f;
			PR_EXPECT(ConstraintStageGuessScale(accepted, epoch, scale));
			PR_EXPECT(scale == 0.125f);
			epoch.timestep = 0.02f;
			PR_EXPECT(ConstraintStageGuessScale(accepted, epoch, scale));
			PR_EXPECT(scale == 0.0625f);
			PR_EXPECT(memcmp(&accepted, &original, sizeof(accepted)) == 0);
			PR_EXPECT(!ConstraintAcceptedInEpoch(accepted, epoch));

			// A selective attempt after rejected main attempts still reads the old acceptance; no current response may be subtracted.
			auto free_value = 0.0f;
			PR_EXPECT(ConstraintStageFreeComponent(3.0f, -0.5f, accepted, epoch, free_value));
			PR_EXPECT(free_value == 3.0f);
			auto cache = GpuConstraintAcceptedRows{};
			cache.velocity_mask = 1u;
			cache.row_states = 2u;
			cache.row_identities = 1u;
			cache.impulses[0] = 8.0f;
			PR_EXPECT(ConstraintRowGuess(cache, 0, 1u, 2u, 1u, float2(-10, 10), scale) == 0.5f);
			PR_EXPECT(ConstraintRowGuess(cache, 0, 1u, 2u, 3u, float2(-10, 10), scale) == 0.0f);

			// Only acceptance changes provenance. A zero response at that acceptance still establishes scale-one continuation.
			accepted = ConstraintAcceptance(epoch);
			PR_EXPECT(ConstraintStageGuessScale(accepted, epoch, scale));
			PR_EXPECT(scale == 1.0f);
			PR_EXPECT(ConstraintStageFreeComponent(3.0f, 0.0f, accepted, epoch, free_value));
			PR_EXPECT(free_value == 3.0f);
			epoch.timestep *= 0.5f;
			PR_EXPECT(!ConstraintStageGuessScale(accepted, epoch, scale));
			PR_EXPECT(ConstraintStageGuessScale(GpuConstraintAcceptance{}, epoch, scale));
			PR_EXPECT(scale == 0.0f);
			log_stream() << "accepted h=0.08; rejected h/2 then h/4: scales 0.125, 0.0625; selective cache remains historical until acceptance\n";
		}

		// Changed inertia and intervening momentum must not reinterpret or erase the response already written by this owner.
		PRUnitTestMethod(ChangedMobilityAndFreshEpoch, Quick)
		{
			// Independently solve diagonal implicit damping once, then record the actual represented momentum change.
			auto epoch = GpuConstraintEpoch{.identity = uint2(1, 0), .timestep = 1.0f / 60.0f, .warm_start_factor = 0.25f};
			auto accepted = GpuConstraintAcceptance{};
			auto body = GpuRigidBody{};
			body.momentum_ang = float4(1, 2, 3, 0);
			body.momentum_lin = float4(4, 5, 6, 0);
			body.sleep.island_id = -1;
			auto response = GpuArticulationSpatialVector{};
			auto replacement = GpuArticulationSpatialVector{};
			auto const alpha = double(epoch.timestep) * 6.0;
			for (int component = 0; component != 3; ++component)
			{
				// Inertia is diag(1,2,4); the expected solution is written without using shader row coefficients.
				auto inertia = double(1 << component);
				replacement.ang[component] = float(double(body.momentum_ang[component]) / (1.0 + alpha / inertia)) - body.momentum_ang[component];
			}
			auto staged_body = GpuRigidBody{};
			auto staged_response = GpuArticulationSpatialVector{};
			PR_EXPECT(ConstraintStageRigidEndpoint(body, response, replacement, accepted, epoch, staged_body, staged_response));
			body = staged_body;
			response = staged_response;
			accepted = ConstraintAcceptance(epoch);

			// Other writers contribute angular and linear impulse before a pose/inertia change. Their numeric increments must survive.
			auto extra = float4(0.125f, -0.25f, 0.5f, 0);
			body.momentum_ang += extra;
			body.momentum_lin += float4(-0.5f, 0.25f, 0.125f, 0);
			auto const before = body;
			auto const old_response = response;
			auto free_momentum = float4{};
			for (int component = 0; component != 3; ++component)
			{
				// Recover the original free impulse plus the independently supplied intervening impulse.
				PR_EXPECT(ConstraintStageFreeComponent(body.momentum_ang[component], response.ang[component], accepted, epoch, free_momentum[component]));
				PR_EXPECT(std::abs(free_momentum[component] - (float(component + 1) + extra[component])) < 2.0e-7f);
			}

			// New inertia is Rz(0.6)*diag(2,4,3)*Rz(0.6)^T. Solve its world-momentum viscous equation directly in double precision.
			auto c = std::cos(0.6);
			auto s = std::sin(0.6);
			auto wxx = 0.5*c*c + 0.25*s*s;
			auto wyy = 0.5*s*s + 0.25*c*c;
			auto wxy = 0.25*c*s;
			auto axx = 1.0 + alpha*wxx;
			auto ayy = 1.0 + alpha*wyy;
			auto axy = alpha*wxy;
			auto det = axx*ayy - axy*axy;
			auto expected = std::array{
				(ayy*free_momentum.x - axy*free_momentum.y) / det,
				(axx*free_momentum.y - axy*free_momentum.x) / det,
				double(free_momentum.z) / (1.0 + alpha/3.0),
			};
			for (int component = 0; component != 3; ++component)
			{
				// Only a newly computed response uses the new mobility; old_response remains the recorded world-momentum increment.
				replacement.ang[component] = float(expected[component]) - free_momentum[component];
			}
			PR_EXPECT(ConstraintStageRigidEndpoint(body, response, replacement, accepted, epoch, staged_body, staged_response));
			for (int component = 0; component != 3; ++component)
			{
				// Observe both the newly published physical state and the actual response difference independently.
				PR_EXPECT(std::abs(double(staged_body.momentum_ang[component]) - expected[component]) < 3.0e-7);
				PR_EXPECT(staged_response.ang[component] == old_response.ang[component] + (staged_body.momentum_ang[component] - before.momentum_ang[component]));
				PR_EXPECT(staged_body.momentum_lin[component] == before.momentum_lin[component]);
			}
			auto px = double(staged_body.momentum_ang.x);
			auto py = double(staged_body.momentum_ang.y);
			auto law_x = px + alpha*(wxx*px + wxy*py) - double(free_momentum.x);
			auto law_y = py + alpha*(wxy*px + wyy*py) - double(free_momentum.y);
			PR_EXPECT(std::abs(law_x) < 3.0e-7);
			PR_EXPECT(std::abs(law_y) < 3.0e-7);
			log_stream() << FmtS("changed-mobility accounting: p=(%g,%g,%g), viscous residual=(%g,%g); contact linear momentum preserved\n",
				staged_body.momentum_ang.x, staged_body.momentum_ang.y, staged_body.momentum_ang.z, law_x, law_y);

			// Fresh-substep ownership logically resets without subtracting physical history, even when old record storage remains populated.
			body = staged_body;
			response = staged_response;
			epoch.identity.x++;
			auto free_value = 0.0f;
			PR_EXPECT(ConstraintStageFreeComponent(body.momentum_ang.x, response.ang.x, accepted, epoch, free_value));
			PR_EXPECT(free_value == body.momentum_ang.x);
			auto staged = GpuConstraintStagedComponent{};
			PR_EXPECT(ConstraintStageReplacement(free_value, response.ang.x, -0.125f, accepted, epoch, staged));
			PR_EXPECT(staged.value == free_value - 0.125f);
			PR_EXPECT(staged.response == staged.value - free_value);
		}

		// Separate records retain an ordinary rigid bridge while coupled islands independently reject, accept, and retry.
		PRUnitTestMethod(MixedOwnersRejectAndRetry, Quick)
		{
			// C0 owns R0's Y response, R owns equal-and-opposite X response on R0/R1, and C1 owns R1's Z response.
			auto epoch = GpuConstraintEpoch{.identity = uint2(9, 0), .timestep = 0.02f, .warm_start_factor = 1.0f};
			auto accepted_c0 = ConstraintAcceptance(epoch);
			auto accepted_c1 = ConstraintAcceptance(epoch);
			auto accepted_r = ConstraintAcceptance(epoch);
			auto r0 = GpuRigidBody{};
			auto r1 = GpuRigidBody{};
			r0.momentum_ang = float4(10.5f, 11.0f, 10.0f, 0);
			r1.momentum_ang = float4(19.5f, 20.0f, 22.0f, 0);
			r0.momentum_lin = float4(0.25f, 0.5f, 0.75f, 0);
			r1.momentum_lin = float4(-0.75f, -0.5f, -0.25f, 0);
			r0.sleep.island_id = r1.sleep.island_id = -1;
			auto c0 = GpuArticulationSpatialVector{.ang = float4(0, 1, 0, 0)};
			auto c1 = GpuArticulationSpatialVector{.ang = float4(0, 0, 2, 0)};
			auto bridge0 = GpuArticulationSpatialVector{.ang = float4(0.5f, 0, 0, 0)};
			auto bridge1 = GpuArticulationSpatialVector{.ang = float4(-0.5f, 0, 0, 0)};
			auto const before_r0 = r0;
			auto const before_r1 = r1;
			auto const before_c0 = c0;
			auto const before_stamp = accepted_c0;

			// C0's rigid endpoint is valid but another component in its complete commit unit overflows. Nothing may publish.
			auto candidate_body = GpuRigidBody{};
			auto candidate_response = GpuArticulationSpatialVector{};
			auto component = GpuConstraintStagedComponent{};
			auto valid_rigid = ConstraintStageRigidEndpoint(r0, c0, GpuArticulationSpatialVector{.ang = float4(0, 0.5f, 0, 0)}, accepted_c0, epoch, candidate_body, candidate_response);
			auto max_value = std::numeric_limits<float>::max();
			auto valid_tree = ConstraintStageReplacement(1.0f, -max_value, max_value, accepted_c0, epoch, component);
			PR_EXPECT(valid_rigid && !valid_tree);
			if (valid_rigid && valid_tree)
			{
				// This is the fixture's explicit owner gate, not a claim that production GPU resources already implement it.
				r0 = candidate_body;
				c0 = candidate_response;
				accepted_c0 = ConstraintAcceptance(epoch);
			}
			PR_EXPECT(memcmp(&r0, &before_r0, sizeof(r0)) == 0);
			PR_EXPECT(memcmp(&c0, &before_c0, sizeof(c0)) == 0);
			PR_EXPECT(memcmp(&accepted_c0, &before_stamp, sizeof(accepted_c0)) == 0);

			// C1 independently accepts on R1; R subsequently replaces both its endpoints without releasing C0 or C1.
			PR_EXPECT(ConstraintStageRigidEndpoint(r1, c1, GpuArticulationSpatialVector{.ang = float4(0, 0, 3, 0)}, accepted_c1, epoch, candidate_body, candidate_response));
			r1 = candidate_body;
			c1 = candidate_response;
			auto candidate0 = GpuRigidBody{};
			auto candidate1 = GpuRigidBody{};
			auto record0 = GpuArticulationSpatialVector{};
			auto record1 = GpuArticulationSpatialVector{};
			PR_EXPECT(ConstraintStageRigidEndpoint(r0, bridge0, GpuArticulationSpatialVector{.ang = float4(0.75f, 0, 0, 0)}, accepted_r, epoch, candidate0, record0));
			PR_EXPECT(ConstraintStageRigidEndpoint(r1, bridge1, GpuArticulationSpatialVector{.ang = float4(-0.75f, 0, 0, 0)}, accepted_r, epoch, candidate1, record1));
			r0 = candidate0;
			r1 = candidate1;
			bridge0 = record0;
			bridge1 = record1;
			PR_EXPECT(all(r0.momentum_ang == float4(10.75f, 11, 10, 0)));
			PR_EXPECT(all(r1.momentum_ang == float4(19.25f, 20, 23, 0)));
			PR_EXPECT(bridge0.ang.x + bridge1.ang.x == 0.0f);

			// A contact write between failed attempt and retry remains outside C0 ownership.
			r0.momentum_ang.z += 0.125f;
			PR_EXPECT(ConstraintStageRigidEndpoint(r0, c0, GpuArticulationSpatialVector{.ang = float4(0, 0.5f, 0, 0)}, accepted_c0, epoch, candidate_body, candidate_response));
			PR_EXPECT(ConstraintStageReplacement(0.2f, -0.1f, -0.05f, accepted_c0, epoch, component));
			r0 = candidate_body;
			c0 = candidate_response;
			accepted_c0 = ConstraintAcceptance(epoch);
			PR_EXPECT(all(r0.momentum_ang == float4(10.75f, 10.5f, 10.125f, 0)));
			PR_EXPECT(component.value == 0.25f);
			PR_EXPECT(all(r0.momentum_lin == before_r0.momentum_lin));
			PR_EXPECT(all(r1.momentum_lin == before_r1.momentum_lin));
			log_stream() << "mixed R0--R1 bridge retained; C0 rejection unchanged; C1 accepted; C0 retry retained intervening contact and bridge writes\n";
		}

		// Nonzero seeds and unrepresentable tiny changes must not wake a body when accepted physical output is unchanged.
		PRUnitTestMethod(NoEffectReplayPreservesSleep, Quick)
		{
			// This body's nonzero recorded response is replayed exactly; a staging helper's internal operations are not physical activity.
			auto epoch = GpuConstraintEpoch{.identity = uint2(1, 0), .timestep = 0.01f, .warm_start_factor = 1.0f};
			auto stamp = ConstraintAcceptance(epoch);
			auto body = GpuRigidBody{};
			body.momentum_ang = float4(1, 2, 3, 0);
			body.momentum_lin = float4(4, 5, 6, 0);
			body.state_flags = ERigidBodyStateFlags_Sleeping;
			body.sleep.island_id = 17;
			body.sleep.timer_s = 3.5f;
			body.sleep.generation = 11;
			body.sleep.flags = 7;
			auto owned = GpuArticulationSpatialVector{.ang = float4(0.5f, -0.25f, 0.125f, 0), .lin = float4(-0.5f, 0.25f, -0.125f, 0)};
			auto staged = GpuRigidBody{};
			auto response = GpuArticulationSpatialVector{};
			PR_EXPECT(ConstraintStageRigidEndpoint(body, owned, owned, stamp, epoch, staged, response));
			PR_EXPECT(memcmp(&body, &staged, sizeof(body)) == 0);
			PR_EXPECT(memcmp(&owned, &response, sizeof(owned)) == 0);

			// One accepted represented delta wakes once; a sub-ULP proposed delta does not invent an owned physical response.
			auto replacement = owned;
			replacement.ang.x += 0.125f;
			PR_EXPECT(ConstraintStageRigidEndpoint(body, owned, replacement, stamp, epoch, staged, response));
			PR_EXPECT(staged.momentum_ang.x == 1.125f);
			PR_EXPECT(staged.sleep.generation == 12);
			PR_EXPECT(staged.sleep.island_id == -1);
			PR_EXPECT(!AllSet(staged.state_flags, ERigidBodyStateFlags_Sleeping));
			body.momentum_ang.x = 16777216.0f;
			owned = GpuArticulationSpatialVector{};
			replacement = GpuArticulationSpatialVector{.ang = float4(0.5f, 0, 0, 0)};
			PR_EXPECT(ConstraintStageRigidEndpoint(body, owned, replacement, stamp, epoch, staged, response));
			PR_EXPECT(memcmp(&body, &staged, sizeof(body)) == 0);
			PR_EXPECT(response.ang.x == 0.0f);
		}

		// Accepted loads use the established detector definition; zero rows must retire caches even when old response cancels.
		PRUnitTestMethod(LoadDefinitionAndNullResponseRetirement, Quick)
		{
			// A linear row has a large lever torque, while pure angular rows have canceling nonzero impulses.
			auto rows = std::array<GpuConstraintRow, GpuConstraintRowsPerBlock>{};
			rows[0].jacobian_b_lin = float4(1, 0, 0, 0);
			rows[0].jacobian_b_ang = float4(0, 0, 100, 0);
			rows[0].bounds = float4(-10, 10, 2, 0);
			rows[1].jacobian_b_ang = float4(0, 1, 0, 0);
			rows[1].bounds = float4(-10, 10, 3, 0);
			rows[2].jacobian_b_ang = float4(0, 1, 0, 0);
			rows[2].bounds = float4(-10, 10, -3, 0);
			auto block = GpuConstraintBlock{.velocity_mask = 7u, .row_states = 63u, .row_identities = 0x321u};
			auto cache = GpuConstraintAcceptedRows{};
			PR_EXPECT(ConstraintStageAcceptedRows(block, rows.data(), cache));
			PR_EXPECT(cache.linear_impulse_norm == 2.0f);
			PR_EXPECT(cache.angular_impulse_norm == 0.0f);
			PR_EXPECT(cache.impulses[1] == 3.0f && cache.impulses[2] == -3.0f);

			// The remaining pure-angular support has zero total response, but its accepted cache still makes the owner eligible.
			block.velocity_mask = 6u;
			PR_EXPECT(ConstraintStageAcceptedRows(block, rows.data(), cache));
			PR_EXPECT(cache.linear_impulse_norm == 0.0f && cache.angular_impulse_norm == 0.0f);
			PR_EXPECT(cache.velocity_mask != 0u);
			block.velocity_mask = 0u;
			auto retired = GpuConstraintAcceptedRows{};
			PR_EXPECT(ConstraintStageAcceptedRows(block, rows.data(), retired));
			PR_EXPECT(retired.velocity_mask == 0u);
			PR_EXPECT(std::ranges::all_of(retired.impulses, [](float value)
			{
				// All obsolete coefficient obligations are cleared, not only the observable net response.
				return value == 0.0f;
			}));
			PR_EXPECT(retired.linear_impulse_norm == 0.0f && retired.angular_impulse_norm == 0.0f);
			log_stream() << "linear-row COM torque=200 excluded from pure-angular load; nonzero null-response coefficients retired by zero-row acceptance\n";
		}
	};

	PRUnitTestClass(ConstraintGpuPackingTests)
	{
		// Preserve stable slots, canonical axis order, endpoint remapping, and compact transfer-layout contracts.
		PRUnitTestMethod(PacksSparsePersistentDescriptors, Quick)
		{
			PR_EXPECT(sizeof(GpuConstraintDesc) == 288);
			PR_EXPECT(sizeof(GpuConstraintEndpoint) == 32);
			PR_EXPECT(sizeof(GpuCoupledConstraintBlockTopology) == 16);
			PR_EXPECT(sizeof(GpuCoupledConstraintTarget) == 32);
			PR_EXPECT(sizeof(GpuCoupledConstraintIsland) == 16);
			PR_EXPECT(sizeof(GpuConstraintBlock) == 32);
			PR_EXPECT(sizeof(GpuConstraintRow) == 96);

			auto body_a = RigidBody{};
			auto body_b = RigidBody{};
			auto desc = ConstraintDesc{};
			desc.m_frame_a.m_body = BodyRef::Rigid(body_a);
			desc.m_frame_b.m_body = BodyRef::Rigid(body_b);
			desc.m_frame_a.m_constraint_to_body = m4x4::Transform(v4::ZAxis(), 0.25f, v4{1, 2, 3, 1});
			desc.D6().m_linear[0].m_mode = EConstraintAxisMode::Locked;
			desc.D6().m_linear[0].m_target_position = 0.5f;
			desc.D6().m_angular[2].m_mode = EConstraintAxisMode::Driven;
			desc.D6().m_angular[2].m_target_velocity = 2.0f;
			desc.D6().m_angular[2].m_max_force = 7.0f;
			desc.m_collide_connected = true;

			auto constraints = ConstraintSet{};
			auto const removed = constraints.Add(desc);
			auto const live = constraints.Add(desc);
			constraints.Remove(removed);
			auto body_ptrs = std::array<RigidBody*, 2>{&body_b, &body_a};
			auto const upload = PackGpuConstraints(constraints, BodyRemap(body_ptrs));

			PR_EXPECT(upload.m_rigid_active_count == 1);
			PR_EXPECT(upload.m_coupled_active_count == 0);
			PR_EXPECT(upload.m_breakable_count == 0);
			PR_EXPECT(upload.m_coupled_endpoints.empty());
			PR_EXPECT(upload.m_coupled_articulation_indices.empty());
			PR_EXPECT(upload.m_endpoints.size() == 2);
			PR_EXPECT(upload.m_descriptors.size() == 2);
			PR_EXPECT(upload.m_endpoints[removed.m_index].flags == GpuConstraintEndpointFlags_None);
			PR_EXPECT(upload.m_endpoints[removed.m_index].generation != removed.m_generation);
			PR_EXPECT(upload.m_endpoints[live.m_index].body_idx_a == 1);
			PR_EXPECT(upload.m_endpoints[live.m_index].body_idx_b == 0);
			PR_EXPECT(AllSet(upload.m_endpoints[live.m_index].flags, GpuConstraintEndpointFlags_Enabled));
			PR_EXPECT(AllSet(upload.m_endpoints[live.m_index].flags, GpuConstraintEndpointFlags_CollideConnected));
			PR_EXPECT(upload.m_descriptors[live.m_index].axes[0].mode == GpuConstraintAxisMode_Locked);
			PR_EXPECT(upload.m_descriptors[live.m_index].axes[0].target_position == 0.5f);
			PR_EXPECT(upload.m_descriptors[live.m_index].axes[5].mode == GpuConstraintAxisMode_Driven);
			PR_EXPECT(upload.m_descriptors[live.m_index].axes[5].target_velocity == 2.0f);
			PR_EXPECT(upload.m_descriptors[live.m_index].axes[5].max_force == 7.0f);
		}

		// Request overload storage only while at least one active constraint has a finite force or torque threshold.
		PRUnitTestMethod(PacksBreakableCountWithoutChargingOrdinaryConstraints, Quick)
		{
			auto body = RigidBody{};
			auto ordinary = ConstraintDesc{};
			ordinary.m_frame_a.m_body = BodyRef::Rigid(body);
			ordinary.m_frame_b.m_body = BodyRef::World();
			ordinary.D6().m_linear[0].m_mode = EConstraintAxisMode::Locked;
			auto breakable = ordinary;
			breakable.m_break_torque = 12.0f;
			auto disabled = breakable;
			disabled.m_enabled = false;
			auto constraints = ConstraintSet{};
			constraints.Add(ordinary);
			auto const breakable_handle = constraints.Add(breakable);
			constraints.Add(disabled);
			auto body_ptrs = std::array<RigidBody*, 1>{&body};

			auto upload = PackGpuConstraints(constraints, BodyRemap(body_ptrs));
			PR_EXPECT(upload.m_breakable_count == 1);

			constraints.SetEnabled(breakable_handle, false);
			upload = PackGpuConstraints(constraints, BodyRemap(body_ptrs));
			PR_EXPECT(upload.m_breakable_count == 0);
		}

		// Disabled descriptors retain their stable slot without requiring missing endpoint bodies in the submitted frame.
		PRUnitTestMethod(DisabledDescriptorsDoNotResolveEndpoints, Quick)
		{
			auto missing_body = RigidBody{};
			auto submitted_body = RigidBody{};
			auto desc = ConstraintDesc{};
			desc.m_frame_a.m_body = BodyRef::Rigid(missing_body);
			desc.m_frame_b.m_body = BodyRef::Rigid(submitted_body);
			desc.D6().m_linear[0].m_mode = EConstraintAxisMode::Locked;
			desc.m_enabled = false;

			auto constraints = ConstraintSet{};
			auto const handle = constraints.Add(desc);
			auto body_ptrs = std::array<RigidBody*, 1>{&submitted_body};
			auto const upload = PackGpuConstraints(constraints, BodyRemap(body_ptrs));

			PR_EXPECT(upload.m_rigid_active_count == 0);
			PR_EXPECT(upload.m_coupled_active_count == 0);
			PR_EXPECT(upload.m_endpoints[handle.m_index].body_idx_a == -1);
			PR_EXPECT(upload.m_endpoints[handle.m_index].body_idx_b == -1);
			PR_EXPECT(!AllSet(upload.m_endpoints[handle.m_index].flags, GpuConstraintEndpointFlags_Enabled));
		}

		// Enabled all-free descriptors preserve their slot but do not activate or allocate the solver lane by themselves.
		PRUnitTestMethod(AllFreeDescriptorsHaveNoSolverWork, Quick)
		{
			auto body_a = RigidBody{};
			auto body_b = RigidBody{};
			auto desc = ConstraintDesc{};
			desc.m_frame_a.m_body = BodyRef::Rigid(body_a);
			desc.m_frame_b.m_body = BodyRef::Rigid(body_b);

			auto constraints = ConstraintSet{};
			auto const handle = constraints.Add(desc);
			auto body_ptrs = std::array<RigidBody*, 2>{&body_a, &body_b};
			auto const upload = PackGpuConstraints(constraints, BodyRemap(body_ptrs));

			PR_EXPECT(upload.m_rigid_active_count == 0);
			PR_EXPECT(upload.m_coupled_active_count == 0);
			PR_EXPECT(AllSet(upload.m_endpoints[handle.m_index].flags, GpuConstraintEndpointFlags_Enabled));
		}

		// Classify link constraints separately so hidden proxies can never enter the independent rigid-body solver lane.
		PRUnitTestMethod(ClassifiesArticulationLinkConstraints, Quick)
		{
			auto [articulation, child] = MakeConstraintGpuTree();
			auto desc = ConstraintDesc{};
			desc.m_frame_a.m_body = BodyRef::Link(articulation, child);
			desc.m_frame_b.m_body = BodyRef::World();
			desc.D6().m_linear[0].m_mode = EConstraintAxisMode::Locked;
			auto constraints = ConstraintSet{};
			auto const handle = constraints.Add(desc);
			auto articulation_ptrs = std::array<Articulation*, 1>{&articulation};

			PR_EXPECT(HasCoupledConstraintWork(constraints));
			auto const upload = PackGpuConstraints(constraints, BodyRemap({}, articulation_ptrs));
			PR_EXPECT(upload.m_rigid_active_count == 0);
			PR_EXPECT(upload.m_coupled_active_count == 1);
			PR_EXPECT(upload.m_coupled_endpoints.size() == upload.m_endpoints.size());
			PR_EXPECT(upload.m_coupled_endpoints[handle.m_index].articulation_idx_a == 0);
			PR_EXPECT(upload.m_coupled_endpoints[handle.m_index].link_idx_a == 1);
			PR_EXPECT(upload.m_coupled_endpoints[handle.m_index].mobility_idx_a == 1);
			PR_EXPECT(upload.m_coupled_endpoints[handle.m_index].root_link_idx_a == 0);
			PR_EXPECT(upload.m_coupled_endpoints[handle.m_index].articulation_idx_b == -1);
			PR_EXPECT(upload.m_coupled_endpoints[handle.m_index].link_idx_b == -1);
			PR_EXPECT(upload.m_coupled_endpoints[handle.m_index].mobility_idx_b == -1);
			PR_EXPECT(upload.m_coupled_endpoints[handle.m_index].root_link_idx_b == -1);
			PR_EXPECT(upload.m_coupled_articulation_indices == std::vector<int>{0});
			PR_EXPECT(upload.m_endpoints[handle.m_index].body_idx_a == 1);
			PR_EXPECT(upload.m_endpoints[handle.m_index].body_idx_b == -1);
			PR_EXPECT(AllSet(upload.m_endpoints[handle.m_index].flags, GpuConstraintEndpointFlags_Enabled));
			PR_EXPECT(AllSet(upload.m_endpoints[handle.m_index].flags, GpuConstraintEndpointFlags_Coupled));

			constraints.SetEnabled(handle, false);
			PR_EXPECT(!HasCoupledConstraintWork(constraints));
		}

		// Pack stable target adjacency while keeping articulations attached through the same fixed body in independent islands.
		PRUnitTestMethod(PacksDeterministicCoupledTopology, Quick)
		{
			auto [articulation_a, child_a] = MakeConstraintGpuTree();
			auto [articulation_b, child_b] = MakeConstraintGpuTree();
			auto shape = collision::ShapeSphere{0.25f};
			auto dynamic_body = RigidBody{&shape, m4x4::Identity(), Inertia::Sphere(shape.m_radius, 1.0f)};
			auto fixed_body = RigidBody{&shape, m4x4::Identity(), Inertia::Infinite()};

			auto constraint = [](BodyRef body_a, BodyRef body_b)
			{
				auto desc = ConstraintDesc{};
				desc.m_frame_a.m_body = body_a;
				desc.m_frame_b.m_body = body_b;
				desc.D6().m_linear[0].m_mode = EConstraintAxisMode::Locked;
				return desc;
			};
			auto constraints = ConstraintSet{};
			auto const articulation_dynamic = constraints.Add(constraint(BodyRef::Link(articulation_a, child_a), BodyRef::Rigid(dynamic_body)));
			auto const articulation_fixed = constraints.Add(constraint(BodyRef::Link(articulation_b, child_b), BodyRef::Rigid(fixed_body)));
			auto const repeated_link = constraints.Add(constraint(BodyRef::Link(articulation_a, child_a), BodyRef::Rigid(fixed_body)));
			auto body_ptrs = std::array<RigidBody*, 2>{&dynamic_body, &fixed_body};
			auto articulation_ptrs = std::array<Articulation*, 2>{&articulation_a, &articulation_b};
			auto const upload = PackGpuConstraints(constraints, BodyRemap(body_ptrs, articulation_ptrs));

			PR_EXPECT(upload.m_coupled_islands.size() == 2);
			PR_EXPECT(upload.m_coupled_articulation_indices == std::vector<int>({0, 1}));
			PR_EXPECT(upload.m_coupled_articulation_islands == std::vector<int>({0, 1}));
			PR_EXPECT(upload.m_coupled_block_topology[articulation_dynamic.m_index].island_idx == 0);
			PR_EXPECT(upload.m_coupled_block_topology[articulation_fixed.m_index].island_idx == 1);
			PR_EXPECT(upload.m_coupled_block_topology[repeated_link.m_index].island_idx == 0);
			PR_EXPECT(upload.m_coupled_island_blocks == std::vector<uint32_t>({
				articulation_dynamic.m_index,
				repeated_link.m_index,
				articulation_fixed.m_index,
			}));

			PR_EXPECT(upload.m_coupled_targets.size() == 3);
			PR_EXPECT(upload.m_coupled_targets[0].target_type == GpuCoupledConstraintTargetType_Link);
			PR_EXPECT(upload.m_coupled_targets[0].target_idx == 1);
			PR_EXPECT(upload.m_coupled_targets[0].island_idx == 0);
			PR_EXPECT(upload.m_coupled_targets[1].target_type == GpuCoupledConstraintTargetType_Rigid);
			PR_EXPECT(upload.m_coupled_targets[1].target_idx == 0);
			PR_EXPECT(upload.m_coupled_targets[2].target_type == GpuCoupledConstraintTargetType_Link);
			PR_EXPECT(upload.m_coupled_targets[2].target_idx == 3);
			PR_EXPECT(upload.m_coupled_targets[2].island_idx == 1);
			PR_EXPECT(upload.m_coupled_target_adjacency == std::vector<uint32_t>({
				2U * articulation_dynamic.m_index,
				2U * repeated_link.m_index,
				2U * articulation_dynamic.m_index + 1U,
				2U * articulation_fixed.m_index,
			}));
			PR_EXPECT(upload.m_coupled_block_topology[articulation_fixed.m_index].target_idx_b == -1);
			PR_EXPECT(upload.m_coupled_block_topology[repeated_link.m_index].target_idx_b == -1);
		}

		// Connected-body collision suppression is deduplicated into a compact canonical hash table.
		PRUnitTestMethod(PacksCollisionExclusions, Quick)
		{
			auto body_a = RigidBody{};
			auto body_b = RigidBody{};
			auto desc = ConstraintDesc{};
			desc.m_frame_a.m_body = BodyRef::Rigid(body_a);
			desc.m_frame_b.m_body = BodyRef::Rigid(body_b);
			desc.D6().m_linear[0].m_mode = EConstraintAxisMode::Locked;

			auto constraints = ConstraintSet{};
			constraints.Add(desc);
			constraints.Add(desc);
			auto collide_desc = desc;
			collide_desc.m_collide_connected = true;
			constraints.Add(collide_desc);
			auto disabled_desc = desc;
			disabled_desc.m_enabled = false;
			constraints.Add(disabled_desc);

			auto body_ptrs = std::array<RigidBody*, 2>{&body_b, &body_a};
			auto const upload = PackGpuConstraints(constraints, BodyRemap(body_ptrs));
			auto const& exclusions = upload.m_collision_exclusions;

			PR_EXPECT(exclusions.m_count == 1);
			PR_EXPECT(exclusions.m_slots.size() == 4);
			PR_EXPECT(exclusions.Mask() == 3);
			PR_EXPECT(std::ranges::count_if(exclusions.m_slots, [](GpuCollisionExclusion const& entry)
			{
				return entry.body_idx_a_plus_one == 1 && entry.body_idx_b_plus_one == 2;
			}) == 1);
		}
	};
}
#endif
