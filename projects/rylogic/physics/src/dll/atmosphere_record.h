//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2016
//*********************************************
#pragma once
#include "pr/physics/forward.h"
#include "pr/physics/physics.h"
#include "pr/physics/physics-dll.h"
#include "physics/src/dll/api_exception.h"
#include "physics/src/utility/gpu.h"

namespace pr::physics
{
	// The rejection returned by the work of one atmosphere ABI call, or none when the call accepted its request. A call that rejects its request
	// has not changed the atmosphere's state or recorded any GPU work.
	struct [[nodiscard]] ApiRejection
	{
		// The status to report, and the message, which is null when the request was accepted. The message must be a static string.
		EStatus m_status = EStatus::Success;
		char const* m_message = nullptr;

		// True when the call rejected its request.
		bool Rejected() const
		{
			return m_message != nullptr;
		}
	};

	// Owns one atmosphere solver, its optional tracers, and the compute job that steps them independently of the engine step.
	// The methods implement the atmosphere ABI calls after the caller has resolved the engine, owner thread and handle.
	// Each method throws ApiException for a rejected request, which has not changed the atmosphere's state or GPU work, so the atmosphere is
	// still usable. Any other failure means the work of the call may be partly done; it is reported as a failure (EStatus::InternalError, or a removed
	// device), never as a rejection, and the atmosphere is then failed: ordinary operations throw ApiException with EStatus::InternalError.
	// The caller must complete TryClose successfully before destroying the record. A call whose work runs sync observers that call methods on this
	// atmosphere also fails when one of those nested calls fails the atmosphere.
	struct AtmosphereRecord
	{
		// The solver and tracers reference 'm_gpu', so it is declared first and destroyed last.
		Gpu m_gpu;
		atmosphere::AtmosphereSolver m_solver;
		std::unique_ptr<atmosphere::AtmosphereTracers> m_tracers;
		atmosphere::AtmosphereProbes m_probes;
		atmosphere::AtmosphereSurfaceWind m_surface_wind;

		// The tracer slot holding the particles from the last finished step, or from creation. A step in flight writes a different slot.
		int m_published_slot;

		// The step in flight on the GPU, if any.
		GpuJob::RunHandle m_pending;

		// The probe samples of the last finished step.
		std::vector<atmosphere::AtmosphereProbeSample> m_probe_samples;

		// The surface wind of the last finished step, one per column, or empty when that step did not sample it.
		std::vector<v2> m_surface_wind_field;

		// True once work may be partly done; only explicit closure remains available.
		bool m_failed;

		// A failed creation has no caller handle, so its existing engine slot retains it for explicit cleanup.
		bool m_abandoned = false;

		// Create the solver and, when 'tracers' has particles, the tracers. Throws std::invalid_argument for an invalid configuration.
		AtmosphereRecord(ID3D12Device4* device, atmosphere::AtmosphereConfig config, atmosphere::AtmosphereTracerConfig const& tracers, IShaderCache* shader_cache);
		AtmosphereRecord(AtmosphereRecord&&) = delete;
		AtmosphereRecord(AtmosphereRecord const&) = delete;
		AtmosphereRecord& operator=(AtmosphereRecord&&) = delete;
		AtmosphereRecord& operator=(AtmosphereRecord const&) = delete;

		// Release an atmosphere after successful TryClose. Construction itself never submits work.
		~AtmosphereRecord() = default;

		// Submit initialization only after the engine slot owns this record.
		void Initialise();

		// Close without publishing results. The HRESULT and caller-owned timeout have the same contract as GpuJob::TryClose.
		HRESULT TryClose(DWORD timeout) noexcept;

		// Record and submit one step. Throws ApiException with EStatus::StepPending while a step is in flight.
		void BeginStep(AtmosphereStepDesc const& step);

		// Finish the step in flight if the GPU has executed it. Returns true when no step is in flight.
		bool PollStep();

		// Wait for and finish the step in flight. Throws ApiException with EStatus::NoStepPending when there is none.
		void CompleteStep();

		// Copy the probe samples of the last finished step. 'required' receives the sample count, even when 'capacity' is too small.
		void CopyProbes(AtmosphereProbeSample* samples, std::uint32_t capacity, std::uint32_t& required);

		// Copy the surface wind of the last finished step. 'required' receives the column count, even when 'capacity' is too small.
		void CopySurfaceWind(Vector2* wind, std::uint32_t capacity, std::uint32_t& required);

		// Change the column floors and remap the field. Runs GPU work to completion. Not allowed while a step is in flight.
		void SetFloors(float const* floor_heights, std::int32_t count);

		// Set the floor temperature under each column, used from the next step.
		void SetFloorTemperatures(float const* floor_temperatures, std::int32_t count);

		// Set the air outside the open faces, used from the next step.
		void SetOutsideAir(AtmosphereOutsideAir const* outside_air, std::int32_t count);

		// Read back all tracer particles. 'required' receives the particle count, even when 'capacity' is too small. Not allowed while a step is in flight.
		void CopyTracers(AtmosphereTracerParticle* particles, std::uint32_t capacity, std::uint32_t& required);

		// Hold the published tracer slot and return an owned COM reference to its buffer.
		void AcquireTracers(AtmosphereTracerSlot& slot);

		// Release one hold on tracer 'slot', recording when the consumer's GPU reads finish.
		void ReleaseTracers(std::int32_t slot, ID3D12Fence* fence, std::uint64_t value);

		// Read back the field as cell-centre samples, with optional stats. 'required' receives the cell count, even when 'capacity' is too small.
		void CopyCellStates(AtmosphereCellState* cells, std::uint32_t capacity, std::uint32_t& required, AtmosphereStats* stats);

	private:

		// True while a submitted step has not been finished.
		bool Pending() const;

		// Wait for the submitted step, then publish the tracer slot, probe samples, and surface wind it wrote.
		void FinishStep();

		// Run 'op' as one ABI call. 'op' returns its own rejection, which is thrown as an ApiException and leaves the atmosphere usable.
		// Any exception that 'op' throws marks the atmosphere failed. An ApiException is reported as EStatus::InternalError with the original
		// nested inside it; any other exception is rethrown unchanged, so the ABI call reports it as a failure (or as a removed device).
		template <typename Op> void Call(Op&& op)
		{
			// A failed atmosphere may hold a partly recorded or partly published step, so nothing it holds can be used again.
			if (m_failed)
				throw ApiException(EStatus::InternalError, "The atmosphere failed in an earlier call; destroy it and create a new one");

			// Perform the call's work, and build the exception for its rejection, inside the failure boundary. Only the returned value is a rejection.
			// Work that runs GPU jobs also runs their sync observers, which may throw any exception, including an ApiException from another ABI call;
			// the exception's type cannot show that no work was done. Building the rejection's exception can also throw, and then the boundary
			// cannot show that the rejection was reported, so that failure latches as well.
			auto rejection = std::optional<ApiException>{};
			try
			{
				if (auto const result = op(); result.Rejected())
					rejection.emplace(result.m_status, result.m_message);
			}
			catch (ApiException const& ex)
			{
				// The status of an escaping ApiException describes the nested call that threw it, not this call, whose work may be partly done.
				// Report this call as failed with a copy of the original, which cannot throw (see ApiException), so the original is never replaced.
				// The original stays nested inside for diagnostics.
				m_failed = true;
				auto failure = ex;
				failure.m_status = EStatus::InternalError;
				std::throw_with_nested(std::move(failure));
			}
			catch (...)
			{
				// The failure may have happened part way through the work.
				m_failed = true;
				throw;
			}

			// A sync observer run by the work may call methods on this atmosphere, and one of those calls may fail the atmosphere even though its
			// exception was handled before reaching here. That failure applies to this call too, and takes precedence over a rejection.
			if (m_failed)
				throw ApiException(EStatus::InternalError, "The atmosphere failed during this call; destroy it and create a new one");

			// Throw the rejection after leaving the boundary, so it is never mistaken for a failure. Copying the built exception cannot throw (see ApiException).
			if (rejection)
				throw *rejection;
		}
	};
}
