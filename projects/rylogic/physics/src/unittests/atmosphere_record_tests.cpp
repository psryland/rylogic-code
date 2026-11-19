//*********************************************
// Atmosphere Ownership Tests
//  Copyright (c) Rylogic Ltd 2026
//*********************************************
#include "physics/src/dll/atmosphere_record.h"
#include "src/unittests/shared_gpu.h"
#include "src/unittests/failure_queue.h"

namespace pr::physics::tests
{
	// Exercise returned rejections, partial-work failures and terminal resource ownership.
	PRUnitTestClass(AtmosphereRecordTests)
	{
		static constexpr int ColumnCount = 64;

		// An open, uniform field exposes changes to outside-air inputs on the first step.
		static atmosphere::AtmosphereConfig Config()
		{
			using namespace atmosphere;
			return AtmosphereConfig{
				.m_grid = AtmosphereGrid{.m_cell_count = iv3{8, 8, 4}, .m_origin = v4::Zero(), .m_dx = 1.0f, .m_lid_z = 4.0f, .m_first_layer_thickness = 1.0f},
				.m_boundaries = AtmosphereBoundaries{.m_x_min = atmosphere::EAtmosphereBoundary::Open, .m_x_max = atmosphere::EAtmosphereBoundary::Open, .m_y_min = atmosphere::EAtmosphereBoundary::Open, .m_y_max = atmosphere::EAtmosphereBoundary::Open},
				.m_reference = AtmosphereReferenceProfile{.m_temperature_at_origin = 280.0f, .m_lapse_rate = 0.0f, .m_min_temperature = 200.0f},
				.m_pressure_vcycles = 1,
				.m_pressure_pre_smooth = 1,
				.m_pressure_post_smooth = 1,
				.m_pressure_coarse_smooth = 1,
			};
		}
		// Each owner has its own queue on the class's shared device, as at the ABI boundary.
		static std::unique_ptr<AtmosphereRecord> MakeRecord(int particles = 0)
		{
			return std::make_unique<AtmosphereRecord>(SharedTestGpu().m_gpu.device(), Config(), atmosphere::AtmosphereTracerConfig{.m_particle_count = particles, .m_seed = 1}, &SharedTestShaderCache());
		}
		// Only structured ABI errors become statuses; unrelated exceptions must still fail the test.
		template <typename Func> static EStatus StatusOf(Func&& func)
		{
			try
			{
				func();
				return EStatus::Success;
			}
			catch (ApiException const& ex)
			{
				return ex.m_status;
			}
		}
		// Advance one fully completed step.
		static void Step(AtmosphereRecord& record)
		{
			record.BeginStep(AtmosphereStepDesc{.dt = 0.1f});
			record.CompleteStep();
		}
		// Read the complete published field through the owner, not directly through the solver.
		static std::vector<AtmosphereCellState> ReadCells(AtmosphereRecord& record)
		{
			auto cells = std::vector<AtmosphereCellState>(record.m_solver.Config().m_grid.CellCount());
			auto required = std::uint32_t{};
			record.CopyCellStates(cells.data(), static_cast<std::uint32_t>(cells.size()), required, nullptr);
			PR_EXPECT(required == cells.size());
			return cells;
		}

		PRUnitTestMethod(InputRejectionsRunNoWork, Extended)
		{
			auto record = MakeRecord(64);
			auto& sync = record->m_gpu.m_job.m_gsync;
			auto const last = sync.LastAddedSyncPoint();
			auto const slot = record->m_tracers->CurrentSlot();
			auto floors = std::vector<float>(ColumnCount, 0.0f);
			PR_EXPECT(StatusOf([&] { record->SetFloors(floors.data(), ColumnCount - 1); }) == EStatus::InvalidArgument);
			floors.back() = std::numeric_limits<float>::infinity();
			PR_EXPECT(StatusOf([&] { record->SetFloors(floors.data(), ColumnCount); }) == EStatus::InvalidArgument);
			auto temperatures = std::vector<float>(ColumnCount, 290.0f);
			temperatures.back() = -1.0f;
			PR_EXPECT(StatusOf([&] { record->SetFloorTemperatures(temperatures.data(), ColumnCount); }) == EStatus::InvalidArgument);
			auto heat = AtmosphereHeatSource{.centre = {4, 4, 1, 1}, .radius = std::numeric_limits<float>::quiet_NaN()};
			PR_EXPECT(StatusOf([&] { record->BeginStep(AtmosphereStepDesc{.dt = 0.1f, .heat_source_count = 1, .heat_sources = &heat}); }) == EStatus::InvalidArgument);
			PR_EXPECT(sync.LastAddedSyncPoint() == last && record->m_tracers->CurrentSlot() == slot);
			PR_EXPECT(!record->m_failed && record->PollStep());
			Step(*record);
		}

		PRUnitTestMethod(RejectedOutsideAirPreservesPendingInput, Extended)
		{
			// An invalid final column must not replace the valid prefix of the preceding pending input.
			auto a = MakeRecord();
			auto b = MakeRecord();
			auto prior = std::vector<AtmosphereOutsideAir>(ColumnCount, AtmosphereOutsideAir{.wind = {1.5f, 0.5f}, .temperature_offset = 2.0f});
			auto changed = std::vector<AtmosphereOutsideAir>(ColumnCount, AtmosphereOutsideAir{.wind = {-3, 2}, .temperature_offset = -1});
			changed.back().temperature_offset = std::numeric_limits<float>::quiet_NaN();
			a->SetOutsideAir(prior.data(), ColumnCount);
			b->SetOutsideAir(prior.data(), ColumnCount);
			PR_EXPECT(StatusOf([&] { a->SetOutsideAir(changed.data(), ColumnCount); }) == EStatus::InvalidArgument);
			auto native = std::vector<atmosphere::AtmosphereOutsideAir>{};
			for (auto const& air : changed)
				native.push_back(atmosphere::AtmosphereOutsideAir{.m_wind = v2{air.wind.x, air.wind.y}, .m_temperature_offset = air.temperature_offset});

			PR_EXPECT(!a->m_solver.SetOutsideAir(native).Accepted() && !a->m_failed);
			for (int i = 0; i != 3; ++i)
			{
				Step(*a);
				Step(*b);
			}
			auto const left = ReadCells(*a);
			auto const right = ReadCells(*b);
			static_assert(sizeof(AtmosphereCellState) == 5 * sizeof(float));
			PR_EXPECT(std::memcmp(left.data(), right.data(), left.size() * sizeof(AtmosphereCellState)) == 0);
		}

		PRUnitTestMethod(SubmitFailureHidesPublishedOutputs, Extended)
		{
			// Seed every published output before failing after the next real queue signal.
			auto record = MakeRecord(64);
			auto const probe = Vector4{4, 4, 1, 1};
			auto const step = AtmosphereStepDesc{.dt = 0.1f, .probe_count = 1, .surface_wind_height = 1.0f, .probes = &probe};
			record->BeginStep(step);
			record->CompleteStep();
			auto required = std::uint32_t{};
			auto sample = AtmosphereProbeSample{};
			record->CopyProbes(&sample, 1, required);
			PR_EXPECT(required == 1);
			auto sub = multicast::AutoSub{record->m_gpu.m_job.m_gsync.SyncPointAdded += [](auto&, auto const&)
			{
				throw std::runtime_error("submitted atmosphere observer");
			}};
			auto message = std::string{};
			try
			{
				record->BeginStep(step);
			}
			catch (std::runtime_error const& ex)
			{
				message = ex.what();
			}
			PR_EXPECT(message == "submitted atmosphere observer" && record->m_failed);
			sub = {};
			auto const last = record->m_gpu.m_job.m_gsync.LastAddedSyncPoint();
			auto wind = std::vector<Vector2>(ColumnCount);
			auto slot = AtmosphereTracerSlot{};
			PR_EXPECT(StatusOf([&] { record->CopyProbes(&sample, 1, required); }) == EStatus::InternalError);
			PR_EXPECT(StatusOf([&] { record->CopySurfaceWind(wind.data(), ColumnCount, required); }) == EStatus::InternalError);
			PR_EXPECT(StatusOf([&] { record->AcquireTracers(slot); }) == EStatus::InternalError);
			PR_EXPECT(StatusOf([&] { record->PollStep(); }) == EStatus::InternalError);
			PR_EXPECT(StatusOf([&] { record->CompleteStep(); }) == EStatus::InternalError);
			PR_EXPECT(StatusOf([&] { record->BeginStep(step); }) == EStatus::InternalError);
			PR_EXPECT(StatusOf([&] { record->ReleaseTracers(0, nullptr, 0); }) == EStatus::InternalError);
			PR_EXPECT(StatusOf([&] { ReadCells(*record); }) == EStatus::InternalError);
			PR_EXPECT(record->m_gpu.m_job.m_gsync.LastAddedSyncPoint() == last);
		}

		PRUnitTestMethod(CompletionFailureClosesOwner, Extended)
		{
			auto record = MakeRecord();
			record->BeginStep(AtmosphereStepDesc{.dt = 0.1f});
			auto sub = multicast::AutoSub{record->m_gpu.m_job.m_gsync.SyncPointCompleted += [](auto&, auto const&)
			{
				throw std::runtime_error("completed atmosphere observer");
			}};
			auto message = std::string{};
			try
			{
				record->CompleteStep();
			}
			catch (std::runtime_error const& ex)
			{
				message = ex.what();
			}
			PR_EXPECT(message == "completed atmosphere observer" && record->m_failed && !record->m_pending);
			sub = {};
			PR_EXPECT(StatusOf([&] { record->PollStep(); }) == EStatus::InternalError);
			PR_EXPECT(StatusOf([&] { Step(*record); }) == EStatus::InternalError);
		}

		PRUnitTestMethod(EscapingNestedRejectionIsOuterFailure, Extended)
		{
			auto record = MakeRecord();
			auto floors = std::vector<float>(ColumnCount, 0.5f);
			auto nested_status = EStatus::Success;
			auto sub = multicast::AutoSub{record->m_gpu.m_job.m_gsync.SyncPointAdded += [&](auto&, auto const&)
			{
				// This nested request itself rejects cleanly, but the outer readback has already been submitted.
				try
				{
					record->SetOutsideAir(nullptr, 1);
				}
				catch (ApiException const& ex)
				{
					nested_status = ex.m_status;
					throw;
				}
			}};
			auto status = EStatus::Success;
			auto retained_status = EStatus::Success;
			auto message = std::string{};
			auto retained_message = std::string{};
			try
			{
				record->SetFloors(floors.data(), ColumnCount);
			}
			catch (ApiException const& ex)
			{
				status = ex.m_status;
				message = ex.what();
				try
				{
					std::rethrow_if_nested(ex);
				}
				catch (ApiException const& inner)
				{
					retained_status = inner.m_status;
					retained_message = inner.what();
				}
			}
			sub = {};
			PR_EXPECT(nested_status == EStatus::InvalidArgument && status == EStatus::InternalError);
			PR_EXPECT(retained_status == EStatus::InvalidArgument && !message.empty() && retained_message == message);
			PR_EXPECT(record->m_failed);
			PR_EXPECT(StatusOf([&] { record->SetFloors(floors.data(), ColumnCount); }) == EStatus::InternalError);
		}

		PRUnitTestMethod(LatchedFailureCannotReturnSuccess, Extended)
		{
			auto record = MakeRecord();
			auto floors = std::vector<float>(ColumnCount, 0.5f);
			auto sub = multicast::AutoSub{record->m_gpu.m_job.m_gsync.SyncPointAdded += [&](auto&, auto const&)
			{
				// Model a swallowed nested failure by setting its owner latch; this does not simulate nested allocation failure.
				record->m_failed = true;
			}};
			PR_EXPECT(StatusOf([&] { record->SetFloors(floors.data(), ColumnCount); }) == EStatus::InternalError);
			sub = {};
			PR_EXPECT(record->m_failed && record->m_gpu.m_job.IsRecording());
			PR_EXPECT(StatusOf([&] { ReadCells(*record); }) == EStatus::InternalError);
		}

		PRUnitTestMethod(HeldSlotsRejectBeforeRecording, Extended)
		{
			auto record = MakeRecord(64);
			auto slots = std::vector<AtmosphereTracerSlot>{};
			auto cleanup = Scope<void>([&]
			{
				for (auto& slot : slots)
				{
					if (slot.resource != nullptr)
						static_cast<IUnknown*>(slot.resource)->Release();
				}
			});
			for (int i = 0; i != atmosphere::AtmosphereTracers::SlotCount; ++i)
			{
				if (i != 0)
					Step(*record);

				slots.emplace_back();
				record->AcquireTracers(slots.back());
			}
			auto const last = record->m_gpu.m_job.m_gsync.LastAddedSyncPoint();
			PR_EXPECT(StatusOf([&] { Step(*record); }) == EStatus::InvalidArgument);
			PR_EXPECT(!record->m_failed && record->m_gpu.m_job.m_gsync.LastAddedSyncPoint() == last);
			record->ReleaseTracers(slots.front().slot, nullptr, 0);
			Step(*record);
			PR_EXPECT(record->m_tracers->CurrentSlot() == slots.front().slot);
			PR_EXPECT(StatusOf([&] { record->ReleaseTracers(slots.front().slot, nullptr, 0); }) == EStatus::InvalidArgument);
			PR_EXPECT(StatusOf([&] { record->ReleaseTracers(atmosphere::AtmosphereTracers::SlotCount, nullptr, 0); }) == EStatus::InvalidArgument);
			PR_EXPECT(!record->m_failed);
			for (auto const& slot : std::span{slots}.subspan(1))
				record->ReleaseTracers(slot.slot, nullptr, 0);
		}

		PRUnitTestMethod(FailedConstructorsFenceBeforeReturning, Extended)
		{
			// Each of the four constructors must issue its own terminal fence before its exception reaches the caller.
			for (int target = 0; target != 4; ++target)
			{
				auto& shared = SharedTestGpu();
				auto proxy = FailureQueue{shared.m_gpu.queue()};
				auto gpu = std::make_unique<Gpu>(shared.m_gpu.device(), &proxy, compute::EGpuOptions::None);
				auto solver = std::unique_ptr<atmosphere::AtmosphereSolver>{};
				if (target != 0)
					solver = std::make_unique<atmosphere::AtmosphereSolver>(*gpu, Config(), &SharedTestShaderCache());

				auto cleanup = Scope<void>([&]
				{
					proxy.m_failures = 0;
					if (gpu != nullptr)
						gpu->m_job.QuiesceForDestruction();
				});
				auto const executes = proxy.m_executes;
				auto const signals = proxy.m_signals;
				proxy.m_failures = 1;
				switch (target)
				{
					case 0: { PR_THROWS((void)atmosphere::AtmosphereSolver(*gpu, Config(), &SharedTestShaderCache()), std::runtime_error); break; }
					case 1: { PR_THROWS((void)atmosphere::AtmosphereTracers(*solver, *gpu, atmosphere::AtmosphereTracerConfig{.m_particle_count = 64}, &SharedTestShaderCache()), std::runtime_error); break; }
					case 2: { PR_THROWS((void)atmosphere::AtmosphereProbes(*solver, *gpu, &SharedTestShaderCache()), std::runtime_error); break; }
					case 3: { PR_THROWS((void)atmosphere::AtmosphereSurfaceWind(*solver, *gpu, &SharedTestShaderCache()), std::runtime_error); break; }
					default: { throw std::logic_error("Unknown construction control"); }
				}
				PR_EXPECT(proxy.m_executes == executes + 1 && proxy.m_signals == signals + 2);
				PR_EXPECT(!gpu->m_job.HasUnfencedSubmission());
				solver.reset();
				gpu.reset();
				PR_EXPECT(proxy.m_refs.load() == 1 && proxy.m_executes == executes + 1 && proxy.m_signals == signals + 2);
			}
		}
	};
}
