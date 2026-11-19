//*********************************************
// Game Systems Atmosphere Record Tests
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// Tests of the atmosphere ABI owner: rejected requests leave the atmosphere usable, and any other failure makes it accept only destruction.
#if PR_UNITTESTS
#include "shared_gpu.h"
#include "game-systems/src/atmosphere/atmosphere_record.h"

namespace pr::game_systems::tests
{
	using namespace pr::game_systems::atmosphere;
	using AtmosphereStepDesc = AtmosphereRecord::AtmosphereStepDesc;
	using AtmosphereHeatSource = AtmosphereRecord::AtmosphereHeatSource;
	using AtmosphereProbeSample = AtmosphereRecord::AtmosphereProbeSample;
	using AtmosphereOutsideAir = AtmosphereRecord::AtmosphereOutsideAir;
	using AtmosphereTracerParticle = AtmosphereRecord::AtmosphereTracerParticle;
	using AtmosphereTracerSlot = AtmosphereRecord::AtmosphereTracerSlot;
	using AtmosphereCellState = AtmosphereRecord::AtmosphereCellState;
	using AtmosphereStats = AtmosphereRecord::AtmosphereStats;
	using Vector2 = AtmosphereRecord::Vector2;
	using Vector4 = AtmosphereRecord::Vector4;

	PRUnitTestClass(AtmosphereRecordTests)
	{
		// The number of columns in 'RecordConfig'.
		static constexpr int ColumnCount = 8 * 8;

		// The fixture explicitly waits for completion before releasing an atmosphere, including after an injected observer failure.
		struct CloseRecord
		{
			// Faults in these tests never block the real queue or its fence.
			void operator()(AtmosphereRecord* record) const
			{
				// Keep test cleanup separate from the production destructor's precondition.
				auto const result = record->TryClose(INFINITE);
				PR_EXPECT(result == S_OK);
				assert(result == S_OK);
				delete record;
			}
		};

		// Return a small flat-domain configuration with open sides, so outside air changes the field from the first step.
		static atmosphere::AtmosphereConfig RecordConfig()
		{
			// A uniform grid with cheap pressure solves keeps each record fast to create and step.
			using namespace atmosphere;
			return AtmosphereConfig{
				.m_grid = AtmosphereGrid{ .m_cell_count = iv3{ 8, 8, 4 }, .m_origin = v4::Zero(), .m_dx = 1.0f, .m_lid_z = 4.0f, .m_first_layer_thickness = 1.0f },
				.m_boundaries = AtmosphereBoundaries{ .m_x_min = atmosphere::EAtmosphereBoundary::Open, .m_x_max = atmosphere::EAtmosphereBoundary::Open, .m_y_min = atmosphere::EAtmosphereBoundary::Open, .m_y_max = atmosphere::EAtmosphereBoundary::Open },
				.m_reference = AtmosphereReferenceProfile{ .m_temperature_at_origin = 280.0f, .m_lapse_rate = 0.0f, .m_min_temperature = 200.0f },
				.m_pressure_vcycles = 1,
				.m_pressure_pre_smooth = 1,
				.m_pressure_post_smooth = 1,
				.m_pressure_coarse_smooth = 1,
			};
		}

		// Construction records initialization but cannot put resources on the GPU before an owner exists.
		PRUnitTestMethod(ConstructorsNeverSubmitInitialization, Quick)
		{
			// Each constructor shares this instrumented initialization job.
			auto& shared = SharedTestGpu();
			compute::tests::ForwardingQueue proxy(shared.m_gpu.queue());
			auto gpu = Gpu(shared.m_gpu.device(), &proxy, compute::EGpuOptions::None);
			auto solver = atmosphere::AtmosphereSolver(gpu, RecordConfig(), &SharedTestShaderCache());
			auto tracers = atmosphere::AtmosphereTracers(solver, gpu, atmosphere::AtmosphereTracerConfig{ .m_particle_count = 64 }, &SharedTestShaderCache());
			auto probes = atmosphere::AtmosphereProbes(solver, gpu, &SharedTestShaderCache());
			auto wind = atmosphere::AtmosphereSurfaceWind(solver, gpu, &SharedTestShaderCache());
			::pr::Scope<void> close([&]
			{
				// These fully constructed owners remain alive until their shared initialization job is closed.
				PR_EXPECT(gpu.m_job.TryClose(INFINITE) == S_OK);
			});
			PR_EXPECT(proxy.m_execute_calls == 0 && proxy.m_signal_attempts == 0);
			gpu.m_job.Run();
			PR_EXPECT(proxy.m_execute_calls == 1 && proxy.m_signal_attempts == 1);
		}

		// The unpublished owner survives initialization that executed but could not be fenced.
		PRUnitTestMethod(FailedOwnedInitializationCanBeClosedLater, Quick)
		{
			// The proxy outlives the record; its initialization job uses the same real queue with injectable Signal results.
			auto proxy = std::unique_ptr<compute::tests::ForwardingQueue>{};
			auto record = std::unique_ptr<AtmosphereRecord, CloseRecord>(new AtmosphereRecord(SharedTestGpu().m_gpu.device(), RecordConfig(), {}, &SharedTestShaderCache()));
			proxy = std::make_unique<compute::tests::ForwardingQueue>(record->m_gpu.m_job.m_queue.get());
			record->m_gpu.m_job.m_queue = D3DPtr<ID3D12CommandQueue>(proxy.get(), true);
			::pr::Scope<void> disarm([&]
			{
				// The fixture's close policy can retire the real work after the injected signal error is removed.
				proxy->m_fail_signals = false;
			});
			proxy->m_fail_signals = true;
			PR_THROWS(record->Initialise(), std::runtime_error);
			PR_EXPECT(record->m_failed);
			PR_EXPECT(RequiresRecreate([&] { record->BeginStep(AtmosphereStepDesc{ .m_dt = 0.1f }); }));
			PR_EXPECT(proxy->m_execute_calls == 1);
			PR_EXPECT(record->TryClose(0) == E_OUTOFMEMORY);
			PR_EXPECT(record->m_failed);
			proxy->m_fail_signals = false;
			PR_EXPECT(record->TryClose(INFINITE) == S_OK);
			PR_EXPECT(proxy->m_execute_calls == 1);
		}

		// Create an atmosphere record on the shared test device, with 'particle_count' tracers.
		static std::unique_ptr<AtmosphereRecord, CloseRecord> MakeRecord(int particle_count = 0)
		{
			// Each record creates its own queue on the shared device, as the ABI does.
			auto tracers = atmosphere::AtmosphereTracerConfig{ .m_particle_count = particle_count, .m_seed = 1 };
			auto record = std::unique_ptr<AtmosphereRecord, CloseRecord>(new AtmosphereRecord(SharedTestGpu().m_gpu.device(), RecordConfig(), tracers, &SharedTestShaderCache()));
			record->Initialise();
			return record;
		}

		// Return the status of the AtmosphereException thrown by 'call', or 'EStatus::Success' when it does not throw.
		// Any other exception escapes, so a call expected to be rejected cannot pass by failing in another way.
		template <typename Call> static EStatus StatusOf(Call&& call)
		{
			// Only rejections and failed-atmosphere errors are AtmosphereExceptions.
			try
			{
				call();
				return EStatus::Success;
			}
			catch (AtmosphereException const& ex)
			{
				return ex.m_status;
			}
		}

		// True if 'call' throws AtmosphereException with EStatus::InternalError and a message telling the caller to recreate the atmosphere.
		template <typename Call> static bool RequiresRecreate(Call&& call)
		{
			// A failed atmosphere reports the same error from every method.
			try
			{
				call();
				return false;
			}
			catch (AtmosphereException const& ex)
			{
				return ex.m_status == EStatus::InternalError && std::string_view(ex.what()).find("destroy it and create a new one") != std::string_view::npos;
			}
		}

		// Record, submit, and finish 'steps' steps with no heat sources or probes.
		static void Step(AtmosphereRecord& record, int steps)
		{
			// Each step is finished before the next, as the ABI requires.
			auto const desc = AtmosphereStepDesc{ .m_dt = 0.1f };
			for (int i = 0; i != steps; ++i)
			{
				record.BeginStep(desc);
				record.CompleteStep();
			}
		}

		// Read back the record's field as cell-centre samples.
		static std::vector<AtmosphereCellState> ReadCells(AtmosphereRecord& record)
		{
			// The capacity is exactly the cell count of 'RecordConfig'.
			auto cells = std::vector<AtmosphereCellState>(record.m_solver.Config().m_grid.CellCount());
			auto required = std::uint32_t{};
			record.CopyCellStates(cells.data(), static_cast<std::uint32_t>(cells.size()), required, nullptr);
			PR_EXPECT(required == cells.size());
			return cells;
		}

		// True if two fields are bitwise identical.
		static bool SameCells(std::vector<AtmosphereCellState> const& a, std::vector<AtmosphereCellState> const& b)
		{
			// The struct has no padding, so a byte comparison compares every value exactly.
			return a.size() == b.size() && std::memcmp(a.data(), b.data(), a.size() * sizeof(AtmosphereCellState)) == 0;
		}

		// Return outside air with the same wind and temperature offset for every column.
		static std::vector<AtmosphereOutsideAir> UniformAir(float wind_x, float wind_y, float temperature_offset)
		{
			// The ABI layout requires a zero reserved field.
			return std::vector<AtmosphereOutsideAir>(ColumnCount, AtmosphereOutsideAir{ .m_wind = { wind_x, wind_y }, .m_temperature_offset = temperature_offset, .m_reserved = 0.0f });
		}

		// An observer that throws after the step's fence was signalled leaves the step submitted but with no handle. The atmosphere must
		// keep the observer's exception as the error, then accept only destruction, without exposing its earlier outputs.
		PRUnitTestMethod(SubmitFailureAfterSignalFailsTheAtmosphere, Quick)
		{
			// Finish one ordinary step, so the atmosphere has published probe samples, surface wind, and tracers.
			auto record = MakeRecord(64);
			auto const probe = Vector4{ 4.0f, 4.0f, 1.0f, 1.0f };
			auto const desc = AtmosphereStepDesc{ .m_dt = 0.1f, .m_probe_count = 1, .m_surface_wind_height = 1.0f, .m_probes = &probe };
			record->BeginStep(desc);
			record->CompleteStep();
			auto required = std::uint32_t{};
			auto sample = AtmosphereProbeSample{};
			record->CopyProbes(&sample, 1, required);
			PR_EXPECT(required == 1);

			// Fail the next sync point notification. It is raised after the real queue signal, so the step is on the GPU.
			auto& gsync = record->m_gpu.m_job.m_gsync;
			auto fail_next = true;
			AutoSub sub = gsync.SyncPointAdded += [&](compute::GpuSync&, EmptyArgs const&)
			{
				// Fail only the first notification.
				if (std::exchange(fail_next, false))
					throw std::runtime_error("Test SyncPointAdded observer failure");
			};

			// The observer's exception is the call's error, not an ABI rejection, and the atmosphere is now failed.
			auto const last = gsync.LastAddedSyncPoint();
			auto primary = std::string{};
			auto rejected = false;
			try
			{
				record->BeginStep(desc);
			}
			catch (AtmosphereException const&)
			{
				rejected = true;
			}
			catch (std::runtime_error const& ex)
			{
				primary = ex.what();
			}
			PR_EXPECT(!rejected);
			PR_EXPECT(primary == "Test SyncPointAdded observer failure");
			PR_EXPECT(gsync.LastAddedSyncPoint() == last + 1);
			PR_EXPECT(record->m_failed);
			sub.reset();

			// Every method now reports the failure, including those that read outputs published before it and those that would wait.
			PR_EXPECT(RequiresRecreate([&] { record->PollStep(); }));
			PR_EXPECT(RequiresRecreate([&] { record->CompleteStep(); }));
			PR_EXPECT(RequiresRecreate([&] { record->CopyProbes(&sample, 1, required); }));
			PR_EXPECT(RequiresRecreate([&] { auto wind = std::vector<Vector2>(ColumnCount); record->CopySurfaceWind(wind.data(), ColumnCount, required); }));
			PR_EXPECT(RequiresRecreate([&] { auto slot = AtmosphereTracerSlot{}; record->AcquireTracers(slot); }));
			PR_EXPECT(RequiresRecreate([&] { auto air = UniformAir(1.0f, 0.0f, 0.0f); record->SetOutsideAir(air.data(), ColumnCount); }));
			PR_EXPECT(RequiresRecreate([&] { ReadCells(*record); }));
			PR_EXPECT(RequiresRecreate([&] { record->BeginStep(desc); }));
			PR_EXPECT(gsync.LastAddedSyncPoint() == last + 1);

			// Destruction waits for the submitted step through the terminal wait.
			record.reset();
		}

		// A completion observer that throws after the step's fence completed makes 'Complete' fail after it consumed the step's handle.
		// The step's outputs were not all published, so the atmosphere must fail rather than report the step as finished.
		PRUnitTestMethod(CompleteFailureFailsTheAtmosphere, Quick)
		{
			// Submit a step that samples a probe, then fail the completion notification.
			auto record = MakeRecord(64);
			auto const probe = Vector4{ 4.0f, 4.0f, 1.0f, 1.0f };
			auto const desc = AtmosphereStepDesc{ .m_dt = 0.1f, .m_probe_count = 1, .m_surface_wind_height = 1.0f, .m_probes = &probe };
			record->BeginStep(desc);
			auto fail_next = true;
			AutoSub sub = record->m_gpu.m_job.m_gsync.SyncPointCompleted += [&](compute::GpuSync&, EmptyArgs const&)
			{
				// Fail only the first notification.
				if (std::exchange(fail_next, false))
					throw std::runtime_error("Test SyncPointCompleted observer failure");
			};

			// The observer's exception is the call's error. The step's handle was consumed, so there is no step to finish again.
			auto primary = std::string{};
			auto rejected = false;
			try
			{
				record->CompleteStep();
			}
			catch (AtmosphereException const&)
			{
				rejected = true;
			}
			catch (std::runtime_error const& ex)
			{
				primary = ex.what();
			}
			PR_EXPECT(!rejected);
			PR_EXPECT(primary == "Test SyncPointCompleted observer failure");
			PR_EXPECT(record->m_failed);
			PR_EXPECT(!record->m_pending);
			sub.reset();

			// Outputs of the step are not available, and no further step can start.
			auto required = std::uint32_t{};
			auto sample = AtmosphereProbeSample{};
			PR_EXPECT(RequiresRecreate([&] { record->CopyProbes(&sample, 1, required); }));
			PR_EXPECT(RequiresRecreate([&] { record->CompleteStep(); }));
			PR_EXPECT(RequiresRecreate([&] { record->BeginStep(desc); }));
			record.reset();
		}

		// A rejected outside-air input must leave the earlier pending input unchanged, even when its invalid value is the last element.
		// Records 'a' and 'b' get the same non-default input; 'a' then gets a rejected input with a changed valid prefix, through both the ABI
		// owner and the native solver. Their next steps must match.
		PRUnitTestMethod(RejectedOutsideAirKeepsPendingInput, Quick)
		{
			// The earlier input, and a rejected input that changes every column but the last, which is not finite.
			auto const prior = UniformAir(1.5f, 0.5f, 2.0f);
			auto changed = UniformAir(-3.0f, 2.0f, -1.0f);
			changed.back().m_temperature_offset = std::numeric_limits<float>::quiet_NaN();

			// Apply the earlier input to both records, then the rejected input to 'a' only.
			auto a = MakeRecord();
			auto b = MakeRecord();
			a->SetOutsideAir(prior.data(), ColumnCount);
			b->SetOutsideAir(prior.data(), ColumnCount);
			PR_EXPECT(StatusOf([&] { a->SetOutsideAir(changed.data(), ColumnCount); }) == EStatus::InvalidArgument);
			PR_EXPECT(!a->m_failed);

			// The native setter owns the check, so call it directly too. It must return a rejection without writing its valid prefix.
			auto native = std::vector<atmosphere::AtmosphereOutsideAir>{};
			for (auto const& air : changed)
				native.push_back(atmosphere::AtmosphereOutsideAir{ .m_wind = v2{ air.m_wind.x, air.m_wind.y }, .m_temperature_offset = air.m_temperature_offset });

			PR_EXPECT(!a->m_solver.SetOutsideAir(native).Accepted());

			// The next steps of 'a' use the earlier input, as the untouched 'b' does.
			Step(*a, 3);
			Step(*b, 3);
			auto const cells_a = ReadCells(*a);
			auto const cells_b = ReadCells(*b);
			PR_EXPECT(SameCells(cells_a, cells_b));

			// The comparison can detect a lost or partly overwritten input: the changed prefix, or the default input, each give a different field.
			auto prefix = changed;
			prefix.back() = prior.back();
			auto c = MakeRecord();
			c->SetOutsideAir(prefix.data(), ColumnCount);
			Step(*c, 3);
			PR_EXPECT(!SameCells(ReadCells(*c), cells_b));
			auto d = MakeRecord();
			Step(*d, 3);
			PR_EXPECT(!SameCells(ReadCells(*d), cells_b));
		}

		// Accepted floors run the job to read back the field, which runs its sync observers. An observer that makes a rejected ABI call
		// throws the same AtmosphereException type as a rejection, but this call's work has already run. The atmosphere must report a failure, not the
		// observer's rejection status, keep the observer's error nested for diagnostics, and become failed.
		PRUnitTestMethod(ObserverRejectionDuringFloorRemapFailsTheAtmosphere, Quick)
		{
			// Valid floors that differ from the flat floors, so the remap is accepted and reads back the field.
			auto record = MakeRecord();
			auto& gsync = record->m_gpu.m_job.m_gsync;
			auto const floors = std::vector<float>(ColumnCount, 0.5f);

			// On the first sync point, after the real queue signal, make a rejected outside-air call on the same atmosphere.
			auto invalid = UniformAir(1.0f, 0.0f, 0.0f);
			invalid.back().m_temperature_offset = std::numeric_limits<float>::quiet_NaN();
			auto observer_status = EStatus::Success;
			auto observer_message = std::string{};
			auto fail_next = true;
			AutoSub sub = gsync.SyncPointAdded += [&](compute::GpuSync&, EmptyArgs const&)
			{
				// Only the first notification makes the call.
				if (!std::exchange(fail_next, false))
					return;

				// Keep the nested call's own rejection, then let its exception leave the observer.
				try
				{
					record->SetOutsideAir(invalid.data(), ColumnCount);
				}
				catch (AtmosphereException const& ex)
				{
					observer_status = ex.m_status;
					observer_message = ex.what();
					throw;
				}
			};

			// The nested call is itself a usable rejection. The floor call's read back was signalled, so it reports a failure with the same
			// message, and the observer's original exception nested inside.
			auto const last = gsync.LastAddedSyncPoint();
			auto status = EStatus::Success;
			auto message = std::string{};
			auto nested_status = EStatus::Success;
			auto nested_message = std::string{};
			try
			{
				record->SetFloors(floors.data(), ColumnCount);
			}
			catch (AtmosphereException const& ex)
			{
				// Keep the reported error, then unwrap the original it carries.
				status = ex.m_status;
				message = ex.what();
				try
				{
					std::rethrow_if_nested(ex);
				}
				catch (AtmosphereException const& inner)
				{
					nested_status = inner.m_status;
					nested_message = inner.what();
				}
			}
			PR_EXPECT(observer_status == EStatus::InvalidArgument);
			PR_EXPECT(status == EStatus::InternalError);
			PR_EXPECT(!observer_message.empty());
			PR_EXPECT(message == observer_message);
			PR_EXPECT(nested_status == EStatus::InvalidArgument);
			PR_EXPECT(nested_message == observer_message);
			PR_EXPECT(gsync.LastAddedSyncPoint() == last + 1);
			PR_EXPECT(record->m_failed);
			sub.reset();

			// Every method now reports the failure, and no further work is submitted.
			PR_EXPECT(RequiresRecreate([&] { record->SetFloors(floors.data(), ColumnCount); }));
			PR_EXPECT(RequiresRecreate([&] { ReadCells(*record); }));
			PR_EXPECT(RequiresRecreate([&] { record->BeginStep(AtmosphereStepDesc{ .m_dt = 0.1f }); }));
			PR_EXPECT(gsync.LastAddedSyncPoint() == last + 1);

			// Destruction waits for the signalled read back through the terminal wait.
			record.reset();
		}

		// A sync observer may make a nested call on the same atmosphere that fails it and then handle that call's exception, so the outer call's
		// work returns normally. The outer call must still fail. The observer sets the record's own failure flag, as the failed nested call would,
		// because a real nested failure would need GPU work recorded from inside the observer.
		PRUnitTestMethod(FailureDuringWorkFailsTheOuterCall, Quick)
		{
			// Valid floors that differ from the flat floors, so the remap is accepted and reads back the field.
			auto record = MakeRecord();
			auto& gsync = record->m_gpu.m_job.m_gsync;
			auto const floors = std::vector<float>(ColumnCount, 0.5f);

			// On the first sync point, after the real queue signal, fail the atmosphere without letting an exception leave the observer.
			auto fail_next = true;
			AutoSub sub = gsync.SyncPointAdded += [&](compute::GpuSync&, EmptyArgs const&)
			{
				// Only the first notification fails the atmosphere.
				if (std::exchange(fail_next, false))
					record->m_failed = true;
			};

			// The floor call's work returns normally, after signalling its read back and remap, but the call reports the failure.
			auto const last = gsync.LastAddedSyncPoint();
			PR_EXPECT(RequiresRecreate([&] { record->SetFloors(floors.data(), ColumnCount); }));
			PR_EXPECT(!fail_next);
			auto const signalled = gsync.LastAddedSyncPoint();
			PR_EXPECT(signalled > last);
			sub.reset();

			// Every method now reports the failure, and no further work is submitted.
			PR_EXPECT(RequiresRecreate([&] { record->SetFloors(floors.data(), ColumnCount); }));
			PR_EXPECT(gsync.LastAddedSyncPoint() == signalled);

			// Destruction waits for the signalled read back through the terminal wait.
			record.reset();
		}

		// Rejected floors must be reported before the field is read back, so no GPU work runs and the atmosphere stays usable.
		PRUnitTestMethod(RejectedFloorsRunNoWork, Quick)
		{
			// One floor too few, and a non-finite floor in an active column.
			auto record = MakeRecord();
			auto& gsync = record->m_gpu.m_job.m_gsync;
			auto const last = gsync.LastAddedSyncPoint();
			auto floors = std::vector<float>(ColumnCount, 0.0f);
			PR_EXPECT(StatusOf([&] { record->SetFloors(floors.data(), ColumnCount - 1); }) == EStatus::InvalidArgument);
			floors[ColumnCount / 2] = std::numeric_limits<float>::infinity();
			PR_EXPECT(StatusOf([&] { record->SetFloors(floors.data(), ColumnCount); }) == EStatus::InvalidArgument);
			PR_EXPECT(gsync.LastAddedSyncPoint() == last);
			PR_EXPECT(!record->m_failed);

			// The atmosphere still steps.
			Step(*record, 1);
		}

		// An invalid heat source is rejected by the solver's step checks, after the inputs are converted but before anything is recorded.
		PRUnitTestMethod(RejectedHeatSourceRecordsNoWork, Quick)
		{
			// A heat source with a non-finite radius.
			auto record = MakeRecord(64);
			auto& gsync = record->m_gpu.m_job.m_gsync;
			auto const last = gsync.LastAddedSyncPoint();
			auto const slot = record->m_tracers->CurrentSlot();
			auto const heat = AtmosphereHeatSource{ .m_centre = { 4.0f, 4.0f, 1.0f, 1.0f }, .m_radius = std::numeric_limits<float>::quiet_NaN(), .m_heating_rate = 1.0f, .m_target_temperature = 300.0f, .m_relaxation_rate = 0.1f };
			auto const desc = AtmosphereStepDesc{ .m_dt = 0.1f, .m_heat_source_count = 1, .m_heat_sources = &heat };
			PR_EXPECT(StatusOf([&] { record->BeginStep(desc); }) == EStatus::InvalidArgument);

			// Nothing was submitted and the tracers did not advance.
			PR_EXPECT(gsync.LastAddedSyncPoint() == last);
			PR_EXPECT(record->m_tracers->CurrentSlot() == slot);
			PR_EXPECT(!record->m_failed);
			PR_EXPECT(record->PollStep());

			// The atmosphere still steps.
			Step(*record, 1);
		}

		// When every tracer slot the next step could write is held, the step is rejected before anything is recorded.
		// Releasing a slot makes the atmosphere step again.
		PRUnitTestMethod(HeldTracerSlotsRejectStep, Quick)
		{
			// Hold the published slot after each step, until every slot is held. The last step's slot is current, so no slot is free.
			auto record = MakeRecord(64);
			auto slots = std::vector<AtmosphereTracerSlot>{};
			for (int i = 0; i != atmosphere::AtmosphereTracers::SlotCount; ++i)
			{
				// Each step writes a slot that is neither current nor held.
				if (i != 0)
					Step(*record, 1);

				auto& slot = slots.emplace_back();
				record->AcquireTracers(slot);
			}

			// The step is rejected without being submitted, and the atmosphere is still usable.
			auto& gsync = record->m_gpu.m_job.m_gsync;
			auto const last = gsync.LastAddedSyncPoint();
			PR_EXPECT(StatusOf([&] { record->BeginStep(AtmosphereStepDesc{ .m_dt = 0.1f }); }) == EStatus::InvalidArgument);
			PR_EXPECT(gsync.LastAddedSyncPoint() == last);
			PR_EXPECT(!record->m_failed);

			// Releasing the oldest slot lets the next step write it.
			record->ReleaseTracers(slots.front().m_slot, nullptr, 0);
			Step(*record, 1);
			PR_EXPECT(record->m_tracers->CurrentSlot() == slots.front().m_slot);

			// A release without a hold, or of a slot out of range, is rejected and does not fail the atmosphere.
			PR_EXPECT(StatusOf([&] { record->ReleaseTracers(slots.front().m_slot, nullptr, 0); }) == EStatus::InvalidArgument);
			PR_EXPECT(StatusOf([&] { record->ReleaseTracers(atmosphere::AtmosphereTracers::SlotCount, nullptr, 0); }) == EStatus::InvalidArgument);
			PR_EXPECT(!record->m_failed);

			// Release the remaining holds and the COM references the acquisitions returned.
			for (auto& slot : slots)
			{
				// The first slot's hold was released above.
				if (&slot != &slots.front())
					record->ReleaseTracers(slot.m_slot, nullptr, 0);

				static_cast<IUnknown*>(slot.m_resource)->Release();
			}
		}
	};
}
#endif
