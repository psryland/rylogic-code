//*********************************************
// Audio Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#pragma once
#include "pr/audio/forward.h"
#include "pr/audio/types.h"

namespace pr::audio
{
	// Carry a stable audio status through the native C++ layer.
	class Exception : public std::runtime_error
	{
		EStatus m_status;

	public:
		Exception(EStatus status, char const* message);

		// Return the stable category associated with this native failure.
		EStatus Status() const noexcept;
	};

	// Return the recommended engine configuration.
	Config DefaultConfig();

	// Return the default listener at the origin looking along negative Y with positive Z up.
	ListenerState DefaultListener();

	// Return default positional properties for a mono emitter.
	EmitterState DefaultEmitter();

	// Own an XAudio2 output graph and all clips and voices created within it.
	class Engine
	{
		struct Impl;
		std::unique_ptr<Impl> m_impl;

	public:
		explicit Engine(Config const& config = DefaultConfig());
		~Engine();
		Engine(Engine&&) noexcept;
		Engine& operator=(Engine&&) noexcept;
		Engine(Engine const&) = delete;
		Engine& operator=(Engine const&) = delete;

		// Create an immutable resident clip by copying a complete WAV file.
		ClipHandle ClipCreateWave(std::span<std::byte const> wave_file);

		// Release a resident clip that is not referenced by a live voice.
		void ClipDestroy(ClipHandle clip);

		// Create one logical playback voice referencing a resident clip.
		VoiceHandle VoiceCreate(VoiceDesc const& desc);

		// Stop and release one playback voice.
		void VoiceDestroy(VoiceHandle voice);

		// Start or restart a voice from its beginning.
		void VoicePlay(VoiceHandle voice);

		// Pause a playing voice without discarding its transport position.
		void VoicePause(VoiceHandle voice);

		// Stop a voice and reset its transport position.
		void VoiceStop(VoiceHandle voice);

		// Return the observable state of a voice.
		VoiceState VoiceStateGet(VoiceHandle voice) const;

		// Set the engine's single rendered listener.
		void ListenerSet(ListenerState const& listener);

		// Enable or update spatial properties for a mono voice.
		void VoiceEmitterSet(VoiceHandle voice, EmitterState const& emitter);

		// Disable spatial processing for a voice.
		void VoiceEmitterClear(VoiceHandle voice);

		// Set the linear gain for a mixer bus.
		void BusGainSet(EBus bus, float gain);

		// Create a resident stream by copying the complete bytes of an Ogg Vorbis file for background decoding.
		StreamHandle StreamCreateOgg(StreamDesc const& desc, std::span<std::byte const> ogg_file);

		// Stop and release a stream, discarding any buffered decode state.
		void StreamDestroy(StreamHandle stream);

		// Start or restart a stream from its beginning, or resume a paused stream from its retained position.
		void StreamPlay(StreamHandle stream);

		// Pause a playing stream without discarding its decode position or queued audio.
		void StreamPause(StreamHandle stream);

		// Stop a stream and reset its decode position to the beginning.
		void StreamStop(StreamHandle stream);

		// Reposition decoding to a PCM frame, discarding any buffered-but-unsubmitted audio.
		void StreamSeek(StreamHandle stream, std::uint64_t pcm_position);

		// Return the observable transport and decode state of a stream.
		StreamState StreamStateGet(StreamHandle stream) const;

		// Commit spatial state, process callback flags, and publish owner-thread events.
		void Update();

		// Copy and consume buffered runtime events.
		std::uint32_t EventsCopy(std::span<Event> events);

		// Return bounded runtime counters.
		Diagnostics DiagnosticsGet() const;
	};
}
