//*********************************************
// Audio Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// Internal helpers shared by engine.cpp and stream.cpp. Not part of the public API.
#pragma once
#include "pr/audio/forward.h"
#include "pr/audio/types.h"

namespace pr::audio
{
	// Own one explicit-lifetime XAudio2 voice.
	struct VoiceDeleter
	{
		void operator()(IXAudio2Voice* voice) const
		{
			if (voice != nullptr)
				voice->DestroyVoice();
		}
	};

	template <typename TVoice>
	using XAudioVoicePtr = std::unique_ptr<TVoice, VoiceDeleter>;

	// A scalar that moves linearly from its current value to a target over a fixed duration of steady-clock time.
	struct Ramp
	{
		using Clock = std::chrono::steady_clock;

		float m_start;
		float m_target;
		Clock::time_point m_begin;
		Clock::duration m_duration;

		// Create a ramp that already holds 'value'.
		explicit Ramp(float value)
			: m_start(value)
			, m_target(value)
			, m_begin()
			, m_duration(Clock::duration::zero())
		{}

		// Return the value at time 'now'. Times before the ramp began return the start value.
		float Value(Clock::time_point now) const
		{
			// A finished or zero-length ramp holds its target exactly
			if (m_duration <= Clock::duration::zero() || now >= m_begin + m_duration)
				return m_target;

			auto t = std::chrono::duration<double>(now - m_begin).count() / std::chrono::duration<double>(m_duration).count();
			t = std::clamp(t, 0.0, 1.0);
			return static_cast<float>(m_start + (m_target - m_start) * t);
		}

		// Start a new ramp toward 'target' from the value at 'now', lasting 'ramp_s' seconds. 'ramp_s' must be finite and non-negative.
		void Set(float target, float ramp_s, Clock::time_point now)
		{
			// Starting from the current value keeps the output continuous when a ramp is interrupted
			m_start = Value(now);
			m_target = target;
			m_begin = now;
			m_duration = std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(ramp_s));
		}

		// The value the ramp is moving toward.
		float Target() const
		{
			return m_target;
		}
	};

	// Retain a generation after destruction so stale handles remain distinguishable from invalid ones.
	template <typename T>
	struct Slot
	{
		std::uint32_t m_generation = 1;
		std::unique_ptr<T> m_object;
	};

	// Throw a diagnostic for a failed Windows audio operation.
	void CheckHR(HRESULT hr, char const* operation);

	// Encode a generation and slot index into a non-zero handle.
	std::uint64_t MakeHandle(std::size_t index, std::uint32_t generation);

	// Return the zero-based slot index encoded in a handle.
	std::size_t HandleIndex(std::uint64_t handle);

	// Return the generation encoded in a handle.
	std::uint32_t HandleGeneration(std::uint64_t handle);

	// Advance a slot generation without producing zero.
	void AdvanceGeneration(std::uint32_t& generation);
}
