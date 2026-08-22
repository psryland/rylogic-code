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
