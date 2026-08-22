//*********************************************
// Audio Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// Dependency-minimal types shared by the native C++ and DLL APIs.
#pragma once
#include <cstdint>
#include <type_traits>

namespace pr::audio
{
	inline constexpr std::uint32_t AUDIO_API_VERSION = 0x00010000U;
	inline constexpr std::uint32_t AUDIO_STRUCT_VERSION = 1U;
	inline constexpr std::uint32_t AUDIO_INFINITE_LOOP = 0xFFFFFFFFU;

	using EngineHandle = std::uint64_t;
	using ClipHandle = std::uint64_t;
	using VoiceHandle = std::uint64_t;
	using StreamHandle = std::uint64_t;

	// Result of an audio API operation.
	enum class EStatus : std::int32_t
	{
		Success = 0,
		InvalidArgument = 1,
		InvalidStruct = 2,
		InvalidHandle = 3,
		StaleHandle = 4,
		WrongThread = 5,
		BufferTooSmall = 6,
		UnsupportedFormat = 7,
		ResourceLimit = 8,
		DeviceLost = 9,
		InternalError = 10,
	};

	// Public structures available through ABI size discovery.
	enum class EStructId : std::int32_t
	{
		Config = 1,
		ListenerState = 2,
		EmitterState = 3,
		VoiceDesc = 4,
		VoiceState = 5,
		Event = 6,
		Diagnostics = 7,
		StreamDesc = 8,
		StreamState = 9,
	};

	// Mixer bus used to group related voices.
	enum class EBus : std::int32_t
	{
		Effects = 0,
		Ambience = 1,
		Speech = 2,
		Music = 3,
		Interface = 4,
		Count = 5,
	};

	// Current transport state of a voice.
	enum class EPlaybackState : std::int32_t
	{
		Stopped = 0,
		Playing = 1,
		Paused = 2,
	};

	// Runtime event type returned through the owner-thread event queue.
	enum class EEvent : std::int32_t
	{
		VoiceEnded = 0,
		VoiceVirtualized = 1,
		VoiceRealized = 2,
		DeviceReset = 3,
		StreamStopped = 4,
		QueueOverflow = 5,
		StreamUnderrun = 6,
	};

	// Version marker at the start of every extensible public structure.
	struct StructHeader
	{
		std::uint32_t size;
		std::uint32_t version;
	};

	// Three-component wire vector with no dependency on the Rylogic maths library.
	struct Vector3
	{
		float x;
		float y;
		float z;
	};

	// Configurable engine capacities and physical constants.
	struct Config
	{
		StructHeader header;
		std::uint32_t max_logical_voices;
		std::uint32_t max_rendered_voices;
		std::uint32_t max_streams;
		std::uint32_t event_capacity;
		std::uint32_t sample_rate;
		std::uint32_t channel_count;
		float speed_of_sound;
		float max_engine_buffer_ms;
	};

	// Position, orientation, and motion of the engine's single rendered listener.
	struct ListenerState
	{
		StructHeader header;
		Vector3 position;
		Vector3 forward;
		Vector3 up;
		Vector3 velocity;
	};

	// Spatial properties applied to a mono positional voice.
	struct EmitterState
	{
		StructHeader header;
		Vector3 position;
		Vector3 forward;
		Vector3 up;
		Vector3 velocity;
		float min_distance;
		float max_distance;
		float cone_inner_angle;
		float cone_outer_angle;
		float cone_outer_gain;
		float doppler_scale;
		float obstruction;
		float occlusion;
		float reverb_send;
	};

	// Initial playback and routing properties for one voice.
	struct VoiceDesc
	{
		StructHeader header;
		ClipHandle clip;
		EBus bus;
		std::int32_t spatial;
		std::uint32_t loop_count;
		std::uint32_t priority;
		float volume;
		float pitch;
	};

	// Observable transport state for one voice.
	struct VoiceState
	{
		StructHeader header;
		EPlaybackState playback;
		std::int32_t spatial;
		std::int32_t virtualized;
		std::uint32_t reserved;
		std::uint64_t samples_played;
	};

	// Initial playback and routing properties for one decoded Ogg Vorbis stream.
	struct StreamDesc
	{
		StructHeader header;
		EBus bus;
		std::uint32_t loop_count;
		std::uint32_t priority;
		float volume;
		float pitch;
	};

	// Observable transport and decode state for one stream.
	struct StreamState
	{
		StructHeader header;
		EPlaybackState playback;
		std::uint32_t channel_count;
		std::uint32_t sample_rate;
		std::uint64_t pcm_position;
		std::uint64_t pcm_total;
		std::uint32_t underrun_count;
	};

	// Buffered runtime notification returned from the engine owner thread. The handle field holds
	// a VoiceHandle or StreamHandle depending on type; both are opaque uint64_t values.
	struct Event
	{
		StructHeader header;
		EEvent type;
		EStatus status;
		VoiceHandle voice;
		std::uint64_t sequence;
	};

	// Bounded runtime counts useful for diagnostics and stress tests.
	struct Diagnostics
	{
		StructHeader header;
		std::uint32_t logical_voice_count;
		std::uint32_t rendered_voice_count;
		std::uint32_t playing_voice_count;
		std::uint32_t virtualized_voice_count;
		std::uint32_t queued_event_count;
		std::uint32_t event_overflow_count;
		std::uint32_t device_reset_count;
		std::uint32_t output_channel_count;
		std::uint32_t output_sample_rate;
		float engine_buffer_ms;
	};

	static_assert(std::is_standard_layout_v<Config>);
	static_assert(std::is_standard_layout_v<ListenerState>);
	static_assert(std::is_standard_layout_v<EmitterState>);
	static_assert(std::is_standard_layout_v<VoiceDesc>);
	static_assert(std::is_standard_layout_v<VoiceState>);
	static_assert(std::is_standard_layout_v<StreamDesc>);
	static_assert(std::is_standard_layout_v<StreamState>);
	static_assert(std::is_standard_layout_v<Event>);
	static_assert(std::is_standard_layout_v<Diagnostics>);
	static_assert(sizeof(Vector3) == 12);
}
