//*********************************************
// Physics Engine
//  Copyright (c) Rylogic Ltd 2026
//*********************************************
#pragma once
#include "pr/physics/forward.h"
#include "pr/physics/physics.h"
#include "physics/src/dll/api_exception.h"
#include "physics/src/utility/gpu.h"

namespace pr::physics
{
	// A returned input rejection changed no atmosphere state and recorded no work. Messages have static storage.
	struct [[nodiscard]] ApiRejection
	{
		EStatus m_status = EStatus::Success;
		char const* m_message = nullptr;

		// True when this call rejected its input.
		bool Rejected() const
		{
			return m_message != nullptr;
		}
	};

	// Owns one independent atmosphere queue and its solver, tracers and sampled outputs.
	// Returned input rejections preserve usability. Exceptions during possibly partial work permanently fail the owner:
	// later operations fail with InternalError, and only destruction/recreation is allowed.
	struct AtmosphereRecord
	{
		// The solver and tracers reference 'm_gpu', so it is declared first and destroyed last.
		Gpu m_gpu;
		atmosphere::AtmosphereSolver m_solver;
		std::unique_ptr<atmosphere::AtmosphereTracers> m_tracers;
		atmosphere::AtmosphereProbes m_probes;
		atmosphere::AtmosphereSurfaceWind m_surface_wind;

		// The last published tracer slot is not the slot written by an in-flight step.
		int m_published_slot;
		GpuJob::RunHandle m_pending;
		std::vector<atmosphere::AtmosphereProbeSample> m_probe_samples;
		std::vector<v2> m_surface_wind_field;
		bool m_failed;

		// Create the solver and optional tracers using the supplied device and shader cache.
		AtmosphereRecord(ID3D12Device4* device, atmosphere::AtmosphereConfig config, atmosphere::AtmosphereTracerConfig const& tracers, IShaderCache* shader_cache);
		AtmosphereRecord(AtmosphereRecord&&) = delete;
		AtmosphereRecord(AtmosphereRecord const&) = delete;
		AtmosphereRecord& operator=(AtmosphereRecord&&) = delete;
		AtmosphereRecord& operator=(AtmosphereRecord const&) = delete;

		// Quiesce submitted work before releasing resources, including after failure and on a finalizer thread.
		~AtmosphereRecord();

		// Record and submit a step; reject overlapping steps before recording.
		void BeginStep(AtmosphereStepDesc const& step);

		// Complete a submitted step if its GPU work has finished; return true when idle.
		bool PollStep();

		// Wait for and finish a pending step; reject an idle owner.
		void CompleteStep();

		// Copy the last published probes, reporting required capacity even for an undersized buffer.
		void CopyProbes(AtmosphereProbeSample* samples, std::uint32_t capacity, std::uint32_t& required);

		// Copy the last published surface wind, reporting required capacity.
		void CopySurfaceWind(Vector2* wind, std::uint32_t capacity, std::uint32_t& required);

		// Remap the column floors to completion while idle.
		void SetFloors(float const* floor_heights, std::int32_t count);

		// Set floor temperatures for the next step.
		void SetFloorTemperatures(float const* floor_temperatures, std::int32_t count);

		// Set outside air for the next step.
		void SetOutsideAir(AtmosphereOutsideAir const* outside_air, std::int32_t count);

		// Read tracer particles while idle, reporting required capacity before readback.
		void CopyTracers(AtmosphereTracerParticle* particles, std::uint32_t capacity, std::uint32_t& required);

		// Hold the published tracer slot and return an owned COM buffer reference.
		void AcquireTracers(AtmosphereTracerSlot& slot);

		// Release one hold after the supplied consumer fence reaches its value.
		void ReleaseTracers(std::int32_t slot, ID3D12Fence* fence, std::uint64_t value);

		// Read the idle field as cell-centre samples, with optional statistics.
		void CopyCellStates(AtmosphereCellState* cells, std::uint32_t capacity, std::uint32_t& required, AtmosphereStats* stats);

	private:
		// True until the submitted step's handle is consumed.
		bool Pending() const;

		// Complete GPU work and publish its sampled outputs.
		void FinishStep();

		// Only a returned rejection proves no work occurred. Escaping exceptions fail the owner, regardless of their type.
		template <typename Op> void Call(Op&& op)
		{
			if (m_failed)
				throw ApiException(EStatus::InternalError, "The atmosphere failed in an earlier call; destroy it and create a new one");

			// Construct the rejection inside the boundary: allocation failure must not masquerade as a delivered input rejection.
			auto rejection = std::optional<ApiException>{};
			try
			{
				if (auto const result = op(); result.Rejected())
					rejection.emplace(result.m_status, result.m_message);
			}
			catch (ApiException const& ex)
			{
				// Its status describes a nested call, not this potentially partial operation. Preserve the original as a nested exception.
				m_failed = true;
				auto failure = ex;
				failure.m_status = EStatus::InternalError;
				std::throw_with_nested(std::move(failure));
			}
			catch (...)
			{
				m_failed = true;
				throw;
			}

			// A sync observer may swallow a nested failure. That still invalidates this outer operation.
			if (m_failed)
				throw ApiException(EStatus::InternalError, "The atmosphere failed during this call; destroy it and create a new one");

			// The completed rejection copy cannot throw; see ApiException.
			if (rejection)
				throw *rejection;
		}
	};
}
