//*********************************************
// Audio Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#include "pr/audio/forward.h"
#include "pr/audio/engine.h"
#include "audio/engine/handle_util.h"
#include "audio/engine/stream.h"

#pragma comment(lib, "xaudio2.lib")

namespace pr::audio
{
	namespace
	{
		using Clock = std::chrono::steady_clock;

		// Bound on one worker decode call, and how often the worker wakes to check for more work.
		inline constexpr std::size_t k_stream_decode_chunk_bytes = 4096;
		inline constexpr auto k_stream_worker_interval = std::chrono::milliseconds(5);

		// Convert an engine buffering bound in milliseconds to a frame count at a given sample rate.
		std::uint64_t FrameCountForMs(std::uint32_t sample_rate, float milliseconds)
		{
			return static_cast<std::uint64_t>(static_cast<double>(sample_rate) * static_cast<double>(milliseconds) / 1000.0);
		}

		// Validate a versioned structure supplied to the C++ API.
		template <typename T>
		void ValidateStruct(T const& value, char const* name)
		{
			if (value.header.size < sizeof(T) || value.header.version != AUDIO_STRUCT_VERSION)
				throw std::invalid_argument(FmtS("%s has an incompatible size or version", name));
		}

		// Return a finite normalized copy or reject an invalid direction.
		X3DAUDIO_VECTOR Direction(Vector3 value, char const* name)
		{
			auto length_sq = value.x * value.x + value.y * value.y + value.z * value.z;
			if (!std::isfinite(length_sq) || length_sq < 1.0e-8f)
				throw std::invalid_argument(FmtS("%s must be a finite non-zero vector", name));

			auto scale = 1.0f / std::sqrt(length_sq);
			return {value.x * scale, value.y * scale, -value.z * scale};
		}

		// Convert a public right-handed position or velocity to X3DAudio left-handed space.
		X3DAUDIO_VECTOR Position(Vector3 value)
		{
			if (!std::isfinite(value.x) || !std::isfinite(value.y) || !std::isfinite(value.z))
				throw std::invalid_argument("Audio vectors must contain finite values");

			return {value.x, value.y, -value.z};
		}

		// Validate that two normalized orientation vectors form a usable basis.
		void ValidateBasis(Vector3 forward, Vector3 up, char const* name)
		{
			auto normal_forward = Direction(forward, name);
			auto normal_up = Direction(up, name);
			auto dot = normal_forward.x * normal_up.x + normal_forward.y * normal_up.y + normal_forward.z * normal_up.z;
			if (std::abs(dot) > 0.01f)
				throw std::invalid_argument(FmtS("%s forward and up vectors must be orthogonal", name));
		}

		// Parse one little-endian integer from a bounded byte range.
		template <typename T>
		T Read(std::span<std::byte const> bytes, std::size_t offset)
		{
			if (offset > bytes.size() || bytes.size() - offset < sizeof(T))
				throw std::invalid_argument("WAV file contains a truncated field");

			auto value = T{};
			std::memcpy(&value, bytes.data() + offset, sizeof(T));
			return value;
		}

		// Immutable decoded metadata and resident sample storage.
		struct Clip
		{
			std::vector<std::byte> m_format;
			std::vector<std::byte> m_samples;
			std::uint32_t m_sample_count;
			std::uint16_t m_channel_count;
			std::uint32_t m_sample_rate;
		};

		// Parse the supported PCM/float subset of a complete RIFF/WAVE file.
		Clip ParseWave(std::span<std::byte const> wave_file)
		{
			constexpr auto RIFF = std::uint32_t{0x46464952};
			constexpr auto WAVE = std::uint32_t{0x45564157};
			constexpr auto FMT = std::uint32_t{0x20746D66};
			constexpr auto DATA = std::uint32_t{0x61746164};

			if (wave_file.size() < 12 || Read<std::uint32_t>(wave_file, 0) != RIFF || Read<std::uint32_t>(wave_file, 8) != WAVE)
				throw std::invalid_argument("Audio clip is not a RIFF/WAVE file");

			auto riff_size = static_cast<std::uint64_t>(Read<std::uint32_t>(wave_file, 4)) + 8;
			if (riff_size > wave_file.size())
				throw std::invalid_argument("WAV RIFF size exceeds the supplied buffer");

			auto fmt = std::span<std::byte const>{};
			auto data = std::span<std::byte const>{};
			for (auto offset = std::size_t{12}; offset + 8 <= riff_size;)
			{
				auto chunk_id = Read<std::uint32_t>(wave_file, offset + 0);
				auto chunk_size = static_cast<std::size_t>(Read<std::uint32_t>(wave_file, offset + 4));
				auto chunk_data = offset + 8;
				if (chunk_data > riff_size || chunk_size > riff_size - chunk_data)
					throw std::invalid_argument("WAV chunk exceeds the RIFF boundary");

				if (chunk_id == FMT && fmt.empty())
					fmt = wave_file.subspan(chunk_data, chunk_size);
				if (chunk_id == DATA && data.empty())
					data = wave_file.subspan(chunk_data, chunk_size);

				auto padded_size = chunk_size + (chunk_size & 1U);
				if (padded_size > riff_size - chunk_data)
					break;
				offset = chunk_data + padded_size;
			}

			if (fmt.size() < 16 || data.empty())
				throw std::invalid_argument("WAV file must contain non-empty fmt and data chunks");

			auto format_tag = Read<std::uint16_t>(fmt, 0);
			auto channels = Read<std::uint16_t>(fmt, 2);
			auto sample_rate = Read<std::uint32_t>(fmt, 4);
			auto block_align = Read<std::uint16_t>(fmt, 12);
			auto bits_per_sample = Read<std::uint16_t>(fmt, 14);
			if (channels == 0 || sample_rate == 0 || block_align == 0)
				throw std::invalid_argument("WAV format has invalid channels, sample rate, or block alignment");
			if (data.size() % block_align != 0)
				throw std::invalid_argument("WAV sample data is not block aligned");

			auto supported =
				format_tag == WAVE_FORMAT_PCM && (bits_per_sample == 8 || bits_per_sample == 16 || bits_per_sample == 24 || bits_per_sample == 32) ||
				format_tag == WAVE_FORMAT_IEEE_FLOAT && bits_per_sample == 32;
			if (!supported)
				throw Exception(EStatus::UnsupportedFormat, "WAV format is not supported; expected PCM8/16/24/32 or float32");

			auto format_size = fmt.size() >= sizeof(WAVEFORMATEX)
				? sizeof(WAVEFORMATEX) + Read<std::uint16_t>(fmt, 16)
				: sizeof(PCMWAVEFORMAT);
			if (format_size > fmt.size())
				throw std::invalid_argument("WAV format extension exceeds the fmt chunk");

			return Clip{
				.m_format = std::vector<std::byte>(fmt.begin(), fmt.begin() + format_size),
				.m_samples = std::vector<std::byte>(data.begin(), data.end()),
				.m_sample_count = static_cast<std::uint32_t>(data.size() / block_align),
				.m_channel_count = channels,
				.m_sample_rate = sample_rate,
			};
		}
	}

	// Throw a diagnostic for a failed Windows audio operation.
	void CheckHR(HRESULT hr, char const* operation)
	{
		if (FAILED(hr))
			throw std::runtime_error(FmtS("%s failed with HRESULT 0x%08X", operation, static_cast<unsigned>(hr)));
	}

	// Encode a generation and slot index into a non-zero handle.
	std::uint64_t MakeHandle(std::size_t index, std::uint32_t generation)
	{
		return (std::uint64_t{generation} << 32) | (std::uint64_t{index} + 1);
	}

	// Return the zero-based slot index encoded in a handle.
	std::size_t HandleIndex(std::uint64_t handle)
	{
		auto low = static_cast<std::uint32_t>(handle);
		if (low == 0)
			throw Exception(EStatus::InvalidHandle, "Audio handle is null");

		return static_cast<std::size_t>(low - 1);
	}

	// Return the generation encoded in a handle.
	std::uint32_t HandleGeneration(std::uint64_t handle)
	{
		return static_cast<std::uint32_t>(handle >> 32);
	}

	// Advance a slot generation without producing zero.
	void AdvanceGeneration(std::uint32_t& generation)
	{
		++generation;
		if (generation == 0)
			generation = 1;
	}


	Exception::Exception(EStatus status, char const* message)
		: std::runtime_error(message)
		, m_status(status)
	{}

	// Return the stable category associated with this native failure.
	EStatus Exception::Status() const noexcept
	{
		return m_status;
	}

	// Return the recommended engine configuration.
	Config DefaultConfig()
	{
		return {
			.header = {sizeof(Config), AUDIO_STRUCT_VERSION},
			.max_logical_voices = 1024,
			.max_rendered_voices = 128,
			.max_streams = 16,
			.event_capacity = 4096,
			.sample_rate = 0,
			.channel_count = 0,
			.speed_of_sound = 343.0f,
			.max_engine_buffer_ms = 40.0f,
		};
	}

	// Return the default listener at the origin looking along negative Y with positive Z up.
	ListenerState DefaultListener()
	{
		return {
			.header = {sizeof(ListenerState), AUDIO_STRUCT_VERSION},
			.position = {},
			.forward = {0, -1, 0},
			.up = {0, 0, 1},
			.velocity = {},
		};
	}

	// Return default positional properties for a mono emitter.
	EmitterState DefaultEmitter()
	{
		return {
			.header = {sizeof(EmitterState), AUDIO_STRUCT_VERSION},
			.position = {},
			.forward = {0, -1, 0},
			.up = {0, 0, 1},
			.velocity = {},
			.min_distance = 1.0f,
			.max_distance = 100.0f,
			.cone_inner_angle = X3DAUDIO_2PI,
			.cone_outer_angle = X3DAUDIO_2PI,
			.cone_outer_gain = 1.0f,
			.doppler_scale = 1.0f,
			.obstruction = 0.0f,
			.occlusion = 0.0f,
			.reverb_send = 0.0f,
		};
	}

	struct Engine::Impl : IXAudio2EngineCallback
	{
		// Record callback state without invoking user code from XAudio2 threads.
		struct VoiceCallback : IXAudio2VoiceCallback
		{
			std::atomic<bool> m_ended;

			VoiceCallback()
				: m_ended(false)
			{}

			void STDMETHODCALLTYPE OnVoiceProcessingPassStart(UINT32) override {}
			void STDMETHODCALLTYPE OnVoiceProcessingPassEnd() override {}
			void STDMETHODCALLTYPE OnStreamEnd() override
			{
				m_ended.store(true, std::memory_order_release);
			}
			void STDMETHODCALLTYPE OnBufferStart(void*) override {}
			void STDMETHODCALLTYPE OnBufferEnd(void*) override {}
			void STDMETHODCALLTYPE OnLoopEnd(void*) override {}
			void STDMETHODCALLTYPE OnVoiceError(void*, HRESULT) override {}
		};

		// Retain a generation after destruction so stale handles remain distinguishable.
		// (Slot<T> is defined once in handle_util.h and reused here via unqualified lookup.)

		// Runtime state for one logical playback voice.
		struct Voice
		{
			VoiceDesc m_desc;
			EmitterState m_emitter;
			bool m_has_emitter;
			bool m_virtualized;
			EPlaybackState m_playback;
			std::uint64_t m_samples_played;
			VoiceCallback m_callback;
			XAudioVoicePtr<IXAudio2SourceVoice> m_source;

			explicit Voice(VoiceDesc const& desc)
				: m_desc(desc)
				, m_emitter(DefaultEmitter())
				, m_has_emitter(desc.spatial != 0)
				, m_virtualized(false)
				, m_playback(EPlaybackState::Stopped)
				, m_samples_played(0)
				, m_callback()
				, m_source()
			{}
		};

		Config m_config;
		std::thread::id m_owner_thread;
		ListenerState m_listener;
		pr::RefPtr<IXAudio2> m_xaudio;
		XAudioVoicePtr<IXAudio2MasteringVoice> m_master;
		std::array<XAudioVoicePtr<IXAudio2SubmixVoice>, static_cast<std::size_t>(EBus::Count)> m_buses;
		XAudioVoicePtr<IXAudio2SubmixVoice> m_reverb_bus;
		std::vector<Slot<Clip>> m_clips;
		std::vector<Slot<Voice>> m_voices;
		std::deque<Event> m_events;
		std::uint64_t m_event_sequence;
		std::uint32_t m_event_overflow_count;
		std::uint32_t m_device_reset_count;
		std::atomic<HRESULT> m_critical_error;
		X3DAUDIO_HANDLE m_x3d;
		std::uint32_t m_output_channels;
		std::uint32_t m_output_sample_rate;
		bool m_com_initialized;

		// Streaming state. m_streams and every per-stream decode field are guarded by m_streams_mutex,
		// which is taken by the owner thread for the duration of each Stream* call and by the worker
		// thread for each decode pass; see stream.h for the full contract.
		std::vector<Slot<Stream>> m_streams;
		std::mutex m_streams_mutex;
		std::thread m_stream_worker;
		std::condition_variable m_stream_worker_wake;
		std::atomic<bool> m_stream_worker_stop;

		explicit Impl(Config const& config)
			: m_config(config)
			, m_owner_thread(std::this_thread::get_id())
			, m_listener(DefaultListener())
			, m_xaudio()
			, m_master()
			, m_buses()
			, m_reverb_bus()
			, m_clips()
			, m_voices()
			, m_events()
			, m_event_sequence(0)
			, m_event_overflow_count(0)
			, m_device_reset_count(0)
			, m_critical_error(S_OK)
			, m_x3d()
			, m_output_channels()
			, m_output_sample_rate()
			, m_com_initialized(false)
			, m_streams()
			, m_streams_mutex()
			, m_stream_worker()
			, m_stream_worker_wake()
			, m_stream_worker_stop(false)
		{
			ValidateStruct(config, "Config");
			if (config.max_logical_voices == 0 || config.max_rendered_voices == 0 || config.max_rendered_voices > config.max_logical_voices || config.event_capacity == 0)
				throw std::invalid_argument("Audio capacities must be non-zero and rendered voices cannot exceed logical voices");
			if (config.max_streams == 0)
				throw std::invalid_argument("Audio stream capacity must be non-zero");
			if (!(config.speed_of_sound > 0.0f) || !(config.max_engine_buffer_ms > 0.0f && config.max_engine_buffer_ms <= 40.0f))
				throw std::invalid_argument("Audio physical and buffering limits are invalid");

			auto com_result = CoInitializeEx(nullptr, COINIT_MULTITHREADED);
			if (com_result != RPC_E_CHANGED_MODE)
			{
				CheckHR(com_result, "CoInitializeEx");
				m_com_initialized = true;
			}

			CreateGraph();

			// Start the decode worker last: if anything above throws, no thread ever existed to join.
			m_stream_worker = std::thread(&Impl::StreamWorkerRun, this);
		}

		~Impl()
		{
			m_stream_worker_stop.store(true, std::memory_order_release);
			m_stream_worker_wake.notify_all();
			if (m_stream_worker.joinable())
				m_stream_worker.join();

			for (auto& slot : m_streams)
				slot.m_object.reset();
			for (auto& slot : m_voices)
				slot.m_object.reset();
			for (auto& bus : m_buses)
				bus.reset();
			m_reverb_bus.reset();
			m_master.reset();
			if (m_xaudio != nullptr)
				m_xaudio->UnregisterForCallbacks(this);
			m_xaudio = nullptr;
			// COM apartment counts can only be balanced by the initializing thread. Finalizer-only
			// abandonment releases all audio resources but must leave this count for process teardown.
			if (m_com_initialized && std::this_thread::get_id() == m_owner_thread)
				CoUninitialize();
		}

		// Enforce the single-owner mutation contract.
		void CheckThread() const
		{
			if (std::this_thread::get_id() != m_owner_thread)
				throw Exception(EStatus::WrongThread, "Mutable audio operation called from a non-owner OS thread");
		}

		// Create the XAudio2 mastering and category submix graph.
		void CreateGraph()
		{
			CheckHR(XAudio2Create(m_xaudio.address_of()), "XAudio2Create");
			CheckHR(m_xaudio->RegisterForCallbacks(this), "IXAudio2::RegisterForCallbacks");

			auto master = static_cast<IXAudio2MasteringVoice*>(nullptr);
			CheckHR(m_xaudio->CreateMasteringVoice(&master, m_config.channel_count, m_config.sample_rate, 0, nullptr, nullptr, AudioCategory_GameEffects), "IXAudio2::CreateMasteringVoice");
			m_master.reset(master);

			auto details = XAUDIO2_VOICE_DETAILS{};
			m_master->GetVoiceDetails(&details);
			m_output_channels = details.InputChannels;
			m_output_sample_rate = details.InputSampleRate;

			auto channel_mask = DWORD{};
			CheckHR(m_master->GetChannelMask(&channel_mask), "IXAudio2MasteringVoice::GetChannelMask");
			X3DAudioInitialize(channel_mask, m_config.speed_of_sound, m_x3d);

			for (auto& bus : m_buses)
			{
				auto submix = static_cast<IXAudio2SubmixVoice*>(nullptr);
				CheckHR(m_xaudio->CreateSubmixVoice(&submix, m_output_channels, m_output_sample_rate), "IXAudio2::CreateSubmixVoice");
				bus.reset(submix);
			}

			// Keep reverberation on a dedicated mono effect send so callers can control room contribution per spatial voice.
			auto reverb_effect = static_cast<IUnknown*>(nullptr);
			CheckHR(XAudio2CreateReverb(&reverb_effect), "XAudio2CreateReverb");
			auto effect_desc = XAUDIO2_EFFECT_DESCRIPTOR{
				.pEffect = reverb_effect,
				.InitialState = TRUE,
				.OutputChannels = 1,
			};
			auto effect_chain = XAUDIO2_EFFECT_CHAIN{
				.EffectCount = 1,
				.pEffectDescriptors = &effect_desc,
			};
			auto reverb_bus = static_cast<IXAudio2SubmixVoice*>(nullptr);
			auto reverb_result = m_xaudio->CreateSubmixVoice(&reverb_bus, 1, m_output_sample_rate, 0, 0, nullptr, &effect_chain);
			reverb_effect->Release();
			CheckHR(reverb_result, "IXAudio2::CreateSubmixVoice(reverb)");
			m_reverb_bus.reset(reverb_bus);
		}

		// Recreate the output graph while preserving engine and child handles.
		void ResetGraph()
		{
			auto streams_lock = std::lock_guard<std::mutex>(m_streams_mutex);

			for (auto& slot : m_voices)
			{
				if (!slot.m_object || !slot.m_object->m_source)
					continue;

				auto state = XAUDIO2_VOICE_STATE{};
				slot.m_object->m_source->GetState(&state);
				slot.m_object->m_samples_played = state.SamplesPlayed;
				slot.m_object->m_source.reset();
			}
			for (auto& slot : m_streams)
			{
				if (!slot.m_object || !slot.m_object->m_source_voice)
					continue;

				// The lost device stops delivering buffer-end callbacks, so recover the played position
				// directly from XAudio2 rather than trusting m_completed_count, then discard anything still
				// queued and reseek decode so the resumed stream starts exactly where playback left off.
				auto& stream = *slot.m_object;
				auto state = XAUDIO2_VOICE_STATE{};
				stream.m_source_voice->GetState(&state);
				auto queued_frames = stream.BufferedFrameCount();
				auto played = std::min<std::uint64_t>(state.SamplesPlayed, queued_frames);
				stream.m_source_voice.reset();
				stream.m_source_started = false;
				stream.m_callback.Reset();
				// Buffers already submitted to the destroyed voice will never complete now; discard them
				// so they cannot be mistaken for still-outstanding audio once a fresh voice is created.
				stream.m_inflight.clear();
				stream.Seek(stream.m_position_base + played);
			}
			for (auto& bus : m_buses)
				bus.reset();
			m_reverb_bus.reset();
			m_master.reset();
			if (m_xaudio != nullptr)
				m_xaudio->UnregisterForCallbacks(this);
			m_xaudio = nullptr;

			CreateGraph();

			// Restore the highest-priority active voices first so a device reset preserves virtualization policy.
			auto candidates = std::vector<std::size_t>{};
			candidates.reserve(m_voices.size());
			for (auto i = std::size_t{}; i != m_voices.size(); ++i)
			{
				auto& slot = m_voices[i];
				if (!slot.m_object || slot.m_object->m_playback == EPlaybackState::Stopped)
					continue;

				slot.m_object->m_virtualized = true;
				candidates.push_back(i);
			}
			std::stable_sort(candidates.begin(), candidates.end(), [&](std::size_t lhs, std::size_t rhs)
				{
					return m_voices[lhs].m_object->m_desc.priority > m_voices[rhs].m_object->m_desc.priority;
				});
			for (auto index : candidates)
			{
				if (RenderedVoiceCount() == m_config.max_rendered_voices)
					break;

				auto& slot = m_voices[index];
				auto handle = MakeHandle(index, slot.m_generation);
				Realize(handle, *slot.m_object);
				Submit(*slot.m_object);
				ApplySpatial(*slot.m_object);
				if (slot.m_object->m_playback == EPlaybackState::Playing)
					CheckHR(slot.m_object->m_source->Start(), "IXAudio2SourceVoice::Start");
			}
			for (auto& slot : m_streams)
			{
				if (!slot.m_object || slot.m_object->m_playback == EPlaybackState::Stopped)
					continue;

				RealizeStream(*slot.m_object);
			}

			++m_device_reset_count;
			AddEvent(EEvent::DeviceReset, 0);
		}

		// Report an XAudio2 critical failure for owner-thread recovery.
		void STDMETHODCALLTYPE OnCriticalError(HRESULT error) override
		{
			m_critical_error.store(error, std::memory_order_release);
		}

		void STDMETHODCALLTYPE OnProcessingPassStart() override {}
		void STDMETHODCALLTYPE OnProcessingPassEnd() override {}

		// Resolve a clip handle and distinguish invalid from stale references.
		Clip& ClipFromHandle(ClipHandle handle)
		{
			auto index = HandleIndex(handle);
			if (index >= m_clips.size())
				throw Exception(EStatus::InvalidHandle, "Audio clip handle is invalid");

			auto& slot = m_clips[index];
			if (slot.m_generation != HandleGeneration(handle) || !slot.m_object)
				throw Exception(EStatus::StaleHandle, "Audio clip handle is stale");

			return *slot.m_object;
		}

		// Resolve a voice handle and distinguish invalid from stale references.
		Voice& VoiceFromHandle(VoiceHandle handle)
		{
			auto index = HandleIndex(handle);
			if (index >= m_voices.size())
				throw Exception(EStatus::InvalidHandle, "Audio voice handle is invalid");

			auto& slot = m_voices[index];
			if (slot.m_generation != HandleGeneration(handle) || !slot.m_object)
				throw Exception(EStatus::StaleHandle, "Audio voice handle is stale");

			return *slot.m_object;
		}

		// Resolve a stream handle and distinguish invalid from stale references. Callers must hold m_streams_mutex.
		Stream& StreamFromHandle(StreamHandle handle)
		{
			auto index = HandleIndex(handle);
			if (index >= m_streams.size())
				throw Exception(EStatus::InvalidHandle, "Audio stream handle is invalid");

			auto& slot = m_streams[index];
			if (slot.m_generation != HandleGeneration(handle) || !slot.m_object)
				throw Exception(EStatus::StaleHandle, "Audio stream handle is stale");

			return *slot.m_object;
		}

		// Count currently materialized XAudio2 source voices.
		std::uint32_t RenderedVoiceCount() const
		{
			auto count = std::uint32_t{};
			for (auto const& slot : m_voices)
			{
				if (slot.m_object && slot.m_object->m_source)
					++count;
			}
			return count;
		}

		// Append one bounded owner-thread event.
		void AddEvent(EEvent type, VoiceHandle voice, EStatus status = EStatus::Success)
		{
			if (m_events.size() == m_config.event_capacity)
			{
				m_events.pop_front();
				++m_event_overflow_count;

				// Make event loss observable through the same queue while retaining the triggering event when capacity permits.
				m_events.push_back(Event{
					.header = {sizeof(Event), AUDIO_STRUCT_VERSION},
					.type = EEvent::QueueOverflow,
					.status = EStatus::BufferTooSmall,
					.voice = 0,
					.sequence = ++m_event_sequence,
				});
				if (type == EEvent::QueueOverflow || m_config.event_capacity == 1)
					return;

				m_events.pop_front();
			}

			m_events.push_back(Event{
				.header = {sizeof(Event), AUDIO_STRUCT_VERSION},
				.type = type,
				.status = status,
				.voice = voice,
				.sequence = ++m_event_sequence,
			});
		}

		// Materialize one logical voice in the XAudio2 graph.
		void Realize(VoiceHandle handle, Voice& voice)
		{
			if (voice.m_source)
				return;

			auto& clip = ClipFromHandle(voice.m_desc.clip);
			auto sends_desc = std::array{
				XAUDIO2_SEND_DESCRIPTOR{
					.Flags = 0,
					.pOutputVoice = m_buses.at(static_cast<std::size_t>(voice.m_desc.bus)).get(),
				},
				XAUDIO2_SEND_DESCRIPTOR{
					.Flags = 0,
					.pOutputVoice = m_reverb_bus.get(),
				},
			};
			auto sends = XAUDIO2_VOICE_SENDS{
				.SendCount = static_cast<UINT32>(sends_desc.size()),
				.pSends = sends_desc.data(),
			};
			auto source = static_cast<IXAudio2SourceVoice*>(nullptr);
			CheckHR(m_xaudio->CreateSourceVoice(&source, reinterpret_cast<WAVEFORMATEX const*>(clip.m_format.data()), XAUDIO2_VOICE_USEFILTER, XAUDIO2_DEFAULT_FREQ_RATIO, &voice.m_callback, &sends), "IXAudio2::CreateSourceVoice");
			voice.m_source.reset(source);
			voice.m_virtualized = false;
			AddEvent(EEvent::VoiceRealized, handle);
		}

		// Release one rendered voice while retaining logical transport state for later promotion.
		void Virtualize(VoiceHandle handle, Voice& voice)
		{
			if (!voice.m_source)
				return;

			auto state = XAUDIO2_VOICE_STATE{};
			voice.m_source->GetState(&state);
			voice.m_samples_played = state.SamplesPlayed;
			voice.m_source->Stop();
			voice.m_source->FlushSourceBuffers();
			voice.m_source.reset();
			voice.m_virtualized = voice.m_playback != EPlaybackState::Stopped;
			if (voice.m_virtualized)
				AddEvent(EEvent::VoiceVirtualized, handle);
		}

		// Queue the resident clip from the beginning.
		void Submit(Voice& voice)
		{
			auto& clip = ClipFromHandle(voice.m_desc.clip);
			auto buffer = XAUDIO2_BUFFER{
				.Flags = XAUDIO2_END_OF_STREAM,
				.AudioBytes = static_cast<UINT32>(clip.m_samples.size()),
				.pAudioData = reinterpret_cast<BYTE const*>(clip.m_samples.data()),
			};
			if (clip.m_sample_count != 0)
				buffer.PlayBegin = static_cast<UINT32>(voice.m_samples_played % clip.m_sample_count);
			buffer.LoopCount =
				voice.m_desc.loop_count == AUDIO_INFINITE_LOOP ? XAUDIO2_LOOP_INFINITE :
				static_cast<UINT32>(std::min<std::uint32_t>(voice.m_desc.loop_count, XAUDIO2_MAX_LOOP_COUNT));
			voice.m_callback.m_ended.store(false, std::memory_order_release);
			CheckHR(voice.m_source->SubmitSourceBuffer(&buffer), "IXAudio2SourceVoice::SubmitSourceBuffer");
		}

		// Apply current spatial state to one realized source voice.
		void ApplySpatial(Voice& voice)
		{
			if (!voice.m_source)
				return;

			auto& clip = ClipFromHandle(voice.m_desc.clip);
			auto pitch = std::clamp(voice.m_desc.pitch, XAUDIO2_MIN_FREQ_RATIO, XAUDIO2_DEFAULT_FREQ_RATIO);
			if (!voice.m_has_emitter)
			{
				auto reverb_matrix = std::vector<float>(clip.m_channel_count, 0.0f);
				CheckHR(voice.m_source->SetFrequencyRatio(pitch), "IXAudio2SourceVoice::SetFrequencyRatio");
				CheckHR(voice.m_source->SetVolume(std::max(0.0f, voice.m_desc.volume)), "IXAudio2Voice::SetVolume");
				CheckHR(voice.m_source->SetOutputMatrix(m_reverb_bus.get(), clip.m_channel_count, 1, reverb_matrix.data()), "IXAudio2Voice::SetOutputMatrix(reverb)");
				return;
			}
			if (clip.m_channel_count != 1)
				throw std::invalid_argument("Only mono clips can be spatialized");

			auto listener = X3DAUDIO_LISTENER{
				.OrientFront = Direction(m_listener.forward, "Listener forward"),
				.OrientTop = Direction(m_listener.up, "Listener up"),
				.Position = Position(m_listener.position),
				.Velocity = Position(m_listener.velocity),
			};
			auto& state = voice.m_emitter;
			if (!(state.min_distance > 0.0f) || !(state.max_distance > state.min_distance))
				throw std::invalid_argument("Emitter distance range is invalid");

			auto min_fraction = std::clamp(state.min_distance / state.max_distance, 0.0f, 0.999f);
			if (!(state.cone_inner_angle >= 0.0f && state.cone_inner_angle <= state.cone_outer_angle && state.cone_outer_angle <= X3DAUDIO_2PI))
				throw std::invalid_argument("Emitter cone angles are invalid");

			auto cone = X3DAUDIO_CONE{
				.InnerAngle = state.cone_inner_angle,
				.OuterAngle = state.cone_outer_angle,
				.InnerVolume = 1.0f,
				.OuterVolume = std::clamp(state.cone_outer_gain, 0.0f, 1.0f),
				.InnerLPF = 0.0f,
				.OuterLPF = 0.75f,
				.InnerReverb = 0.0f,
				.OuterReverb = 1.0f,
			};
			auto volume_points = std::array{
				X3DAUDIO_DISTANCE_CURVE_POINT{0.0f, 1.0f},
				X3DAUDIO_DISTANCE_CURVE_POINT{min_fraction, 1.0f},
				X3DAUDIO_DISTANCE_CURVE_POINT{1.0f, 0.0f},
			};
			auto volume_curve = X3DAUDIO_DISTANCE_CURVE{
				.pPoints = volume_points.data(),
				.PointCount = static_cast<UINT32>(volume_points.size()),
			};
			auto emitter = X3DAUDIO_EMITTER{
				.pCone = state.cone_outer_angle < X3DAUDIO_2PI ? &cone : nullptr,
				.OrientFront = Direction(state.forward, "Emitter forward"),
				.OrientTop = Direction(state.up, "Emitter up"),
				.Position = Position(state.position),
				.Velocity = Position(state.velocity),
				.ChannelCount = 1,
				.pVolumeCurve = &volume_curve,
				.CurveDistanceScaler = state.max_distance,
				.DopplerScaler = std::max(0.0f, state.doppler_scale),
			};
			auto matrix = std::vector<float>(m_output_channels);
			auto dsp = X3DAUDIO_DSP_SETTINGS{
				.pMatrixCoefficients = matrix.data(),
				.SrcChannelCount = 1,
				.DstChannelCount = m_output_channels,
			};
			X3DAudioCalculate(m_x3d, &listener, &emitter, X3DAUDIO_CALCULATE_MATRIX | X3DAUDIO_CALCULATE_DOPPLER | X3DAUDIO_CALCULATE_LPF_DIRECT | X3DAUDIO_CALCULATE_REVERB, &dsp);

			auto obstruction = std::clamp(state.obstruction, 0.0f, 1.0f);
			auto occlusion = std::clamp(state.occlusion, 0.0f, 1.0f);
			auto gain = std::max(0.0f, voice.m_desc.volume) * (1.0f - 0.35f * obstruction) * (1.0f - 0.65f * occlusion);
			auto filter = XAUDIO2_FILTER_PARAMETERS{
				.Type = LowPassFilter,
				.Frequency = std::clamp(dsp.LPFDirectCoefficient * (1.0f - 0.5f * obstruction - 0.75f * occlusion), 0.01f, 1.0f),
				.OneOverQ = 1.0f,
			};
			CheckHR(voice.m_source->SetOutputMatrix(m_buses.at(static_cast<std::size_t>(voice.m_desc.bus)).get(), 1, m_output_channels, matrix.data()), "IXAudio2Voice::SetOutputMatrix");
			auto reverb_send = std::clamp(state.reverb_send, 0.0f, 1.0f) * dsp.ReverbLevel;
			CheckHR(voice.m_source->SetOutputMatrix(m_reverb_bus.get(), 1, 1, &reverb_send), "IXAudio2Voice::SetOutputMatrix(reverb)");
			CheckHR(voice.m_source->SetFrequencyRatio(std::clamp(pitch * dsp.DopplerFactor, XAUDIO2_MIN_FREQ_RATIO, XAUDIO2_DEFAULT_FREQ_RATIO)), "IXAudio2SourceVoice::SetFrequencyRatio");
			CheckHR(voice.m_source->SetVolume(gain), "IXAudio2Voice::SetVolume");
			CheckHR(voice.m_source->SetFilterParameters(&filter), "IXAudio2Voice::SetFilterParameters");
		}

		// Materialize the XAudio2 source voice for a stream, bound to its configured bus. Callers must hold m_streams_mutex.
		void RealizeStream(Stream& stream)
		{
			if (stream.m_source_voice)
				return;

			auto format = WAVEFORMATEX{
				.wFormatTag = WAVE_FORMAT_PCM,
				.nChannels = static_cast<WORD>(stream.m_channel_count),
				.nSamplesPerSec = stream.m_sample_rate,
				.nAvgBytesPerSec = stream.m_sample_rate * stream.m_block_align,
				.nBlockAlign = static_cast<WORD>(stream.m_block_align),
				.wBitsPerSample = 16,
				.cbSize = 0,
			};
			auto send = XAUDIO2_SEND_DESCRIPTOR{
				.Flags = 0,
				.pOutputVoice = m_buses.at(static_cast<std::size_t>(stream.m_desc.bus)).get(),
			};
			auto sends = XAUDIO2_VOICE_SENDS{
				.SendCount = 1,
				.pSends = &send,
			};
			auto source = static_cast<IXAudio2SourceVoice*>(nullptr);
			CheckHR(m_xaudio->CreateSourceVoice(&source, &format, XAUDIO2_VOICE_USEFILTER, XAUDIO2_DEFAULT_FREQ_RATIO, &stream.m_callback, &sends), "IXAudio2::CreateSourceVoice");
			stream.m_source_voice.reset(source);
			stream.m_source_started = false;

			auto pitch = std::clamp(stream.m_desc.pitch, XAUDIO2_MIN_FREQ_RATIO, XAUDIO2_DEFAULT_FREQ_RATIO);
			CheckHR(stream.m_source_voice->SetFrequencyRatio(pitch), "IXAudio2SourceVoice::SetFrequencyRatio");
			CheckHR(stream.m_source_voice->SetVolume(std::max(0.0f, stream.m_desc.volume)), "IXAudio2Voice::SetVolume");
		}

		// Hand one decoded chunk to XAudio2 and track it in m_inflight until it completes. Callers must hold m_streams_mutex.
		void SubmitStreamChunk(Stream& stream, PcmChunk chunk)
		{
			if (!stream.m_source_voice || chunk.m_frame_count == 0)
				return;

			stream.m_inflight.push_back(std::move(chunk));
			auto& queued = stream.m_inflight.back();
			auto buffer = XAUDIO2_BUFFER{
				.AudioBytes = static_cast<UINT32>(queued.m_samples.size()),
				.pAudioData = reinterpret_cast<BYTE const*>(queued.m_samples.data()),
			};
			stream.m_callback.m_queued_count.fetch_add(1, std::memory_order_acq_rel);
			auto result = stream.m_source_voice->SubmitSourceBuffer(&buffer);
			if (FAILED(result))
				stream.m_callback.m_queued_count.fetch_sub(1, std::memory_order_acq_rel);
			CheckHR(result, "IXAudio2SourceVoice::SubmitSourceBuffer");
		}

		// Reconcile completed buffers, publish underrun/completion events, and forward newly decoded audio
		// to XAudio2. Called once per stream from Update(). Callers must hold m_streams_mutex.
		void PumpStream(StreamHandle handle, Stream& stream)
		{
			auto completed = stream.m_callback.m_completed_count.exchange(0, std::memory_order_acq_rel);
			for (auto i = std::uint32_t{}; i != completed && !stream.m_inflight.empty(); ++i)
			{
				stream.m_position_base += stream.m_inflight.front().m_frame_count;
				stream.m_inflight.pop_front();
			}

			auto underrun = stream.m_callback.m_underrun.exchange(false, std::memory_order_acq_rel);
			if (underrun && stream.m_playback == EPlaybackState::Playing && !(stream.m_decode_ended && stream.m_pending.empty() && stream.m_inflight.empty()))
			{
				++stream.m_underrun_count;
				AddEvent(EEvent::StreamUnderrun, handle);
			}

			if (stream.m_playback != EPlaybackState::Playing)
				return;

			for (auto chunk = stream.PopPendingChunk(); chunk; chunk = stream.PopPendingChunk())
				SubmitStreamChunk(stream, std::move(*chunk));
			if (!stream.m_source_started && !stream.m_inflight.empty())
			{
				CheckHR(stream.m_source_voice->Start(), "IXAudio2SourceVoice::Start");
				stream.m_source_started = true;
			}

			// Decode has permanently ended and every submitted chunk has finished playing: the stream is done.
			if (stream.m_decode_ended && stream.m_pending.empty() && stream.m_inflight.empty())
			{
				stream.m_playback = EPlaybackState::Stopped;
				stream.m_source_started = false;
				AddEvent(EEvent::StreamStopped, handle, stream.m_decode_error ? EStatus::UnsupportedFormat : EStatus::Success);
			}
		}

		// Background decode loop: the only place that calls into libvorbis, kept off the owner thread so
		// Update() and the Stream* API never block on codec work. Touches only decode state (never XAudio2).
		void StreamWorkerRun()
		{
			auto lock = std::unique_lock<std::mutex>(m_streams_mutex);
			while (!m_stream_worker_stop.load(std::memory_order_acquire))
			{
				for (auto& slot : m_streams)
				{
					if (!slot.m_object)
						continue;

					auto& stream = *slot.m_object;
					auto target_frames = FrameCountForMs(stream.m_sample_rate, m_config.max_engine_buffer_ms);
					while (!stream.m_decode_ended && stream.BufferedFrameCount() < target_frames)
					{
						if (!stream.DecodeOneChunk(k_stream_decode_chunk_bytes))
							break;
					}
				}
				m_stream_worker_wake.wait_for(lock, k_stream_worker_interval);
			}
		}
	};

	Engine::Engine(Config const& config)
		: m_impl(std::make_unique<Impl>(config))
	{}

	Engine::~Engine() = default;
	Engine::Engine(Engine&&) noexcept = default;
	Engine& Engine::operator=(Engine&&) noexcept = default;

	// Create an immutable resident clip by copying a complete WAV file.
	ClipHandle Engine::ClipCreateWave(std::span<std::byte const> wave_file)
	{
		m_impl->CheckThread();
		auto clip = std::make_unique<Clip>(ParseWave(wave_file));
		for (auto i = std::size_t{}; i != m_impl->m_clips.size(); ++i)
		{
			auto& slot = m_impl->m_clips[i];
			if (!slot.m_object)
			{
				slot.m_object = std::move(clip);
				return MakeHandle(i, slot.m_generation);
			}
		}

		m_impl->m_clips.push_back({});
		m_impl->m_clips.back().m_object = std::move(clip);
		return MakeHandle(m_impl->m_clips.size() - 1, m_impl->m_clips.back().m_generation);
	}

	// Release a resident clip that is not referenced by a live voice.
	void Engine::ClipDestroy(ClipHandle clip)
	{
		m_impl->CheckThread();
		for (auto const& slot : m_impl->m_voices)
		{
			if (slot.m_object && slot.m_object->m_desc.clip == clip)
				throw std::runtime_error("Cannot destroy an audio clip while a voice references it");
		}

		auto index = HandleIndex(clip);
		m_impl->ClipFromHandle(clip);
		m_impl->m_clips[index].m_object.reset();
		AdvanceGeneration(m_impl->m_clips[index].m_generation);
	}

	// Create one logical playback voice referencing a resident clip.
	VoiceHandle Engine::VoiceCreate(VoiceDesc const& desc)
	{
		m_impl->CheckThread();
		ValidateStruct(desc, "VoiceDesc");
		m_impl->ClipFromHandle(desc.clip);
		if (desc.bus < EBus::Effects || desc.bus >= EBus::Count || desc.volume < 0.0f || desc.pitch <= 0.0f)
			throw std::invalid_argument("Voice description contains invalid routing, gain, or pitch");

		auto live_count = std::uint32_t{};
		for (auto const& slot : m_impl->m_voices)
			live_count += slot.m_object != nullptr;
		if (live_count >= m_impl->m_config.max_logical_voices)
			throw Exception(EStatus::ResourceLimit, "Audio logical voice capacity reached");

		auto voice = std::make_unique<Impl::Voice>(desc);
		for (auto i = std::size_t{}; i != m_impl->m_voices.size(); ++i)
		{
			auto& slot = m_impl->m_voices[i];
			if (!slot.m_object)
			{
				slot.m_object = std::move(voice);
				return MakeHandle(i, slot.m_generation);
			}
		}

		m_impl->m_voices.push_back({});
		m_impl->m_voices.back().m_object = std::move(voice);
		return MakeHandle(m_impl->m_voices.size() - 1, m_impl->m_voices.back().m_generation);
	}

	// Stop and release one playback voice.
	void Engine::VoiceDestroy(VoiceHandle voice)
	{
		m_impl->CheckThread();
		auto index = HandleIndex(voice);
		m_impl->VoiceFromHandle(voice);
		m_impl->m_voices[index].m_object.reset();
		AdvanceGeneration(m_impl->m_voices[index].m_generation);
	}

	// Start or restart a voice from its beginning.
	void Engine::VoicePlay(VoiceHandle voice_handle)
	{
		m_impl->CheckThread();
		auto& voice = m_impl->VoiceFromHandle(voice_handle);
		if (!voice.m_source && m_impl->RenderedVoiceCount() == m_impl->m_config.max_rendered_voices)
		{
			// A higher-priority request may reclaim the lowest-priority physical slot without destroying its logical voice.
			auto victim_index = m_impl->m_voices.size();
			auto victim_priority = voice.m_desc.priority;
			for (auto i = std::size_t{}; i != m_impl->m_voices.size(); ++i)
			{
				auto const& slot = m_impl->m_voices[i];
				if (!slot.m_object || !slot.m_object->m_source || slot.m_object->m_desc.priority >= victim_priority)
					continue;

				victim_index = i;
				victim_priority = slot.m_object->m_desc.priority;
			}
			if (victim_index != m_impl->m_voices.size())
			{
				auto& victim = *m_impl->m_voices[victim_index].m_object;
				m_impl->Virtualize(MakeHandle(victim_index, m_impl->m_voices[victim_index].m_generation), victim);
			}
		}
		if (!voice.m_source && m_impl->RenderedVoiceCount() < m_impl->m_config.max_rendered_voices)
			m_impl->Realize(voice_handle, voice);
		if (!voice.m_source)
		{
			voice.m_virtualized = true;
			voice.m_playback = EPlaybackState::Playing;
			m_impl->AddEvent(EEvent::VoiceVirtualized, voice_handle);
			return;
		}

		voice.m_source->Stop();
		voice.m_source->FlushSourceBuffers();
		voice.m_samples_played = 0;
		m_impl->Submit(voice);
		m_impl->ApplySpatial(voice);
		CheckHR(voice.m_source->Start(), "IXAudio2SourceVoice::Start");
		voice.m_playback = EPlaybackState::Playing;
	}

	// Pause a playing voice without discarding its transport position.
	void Engine::VoicePause(VoiceHandle voice_handle)
	{
		m_impl->CheckThread();
		auto& voice = m_impl->VoiceFromHandle(voice_handle);
		if (voice.m_source)
		{
			auto state = XAUDIO2_VOICE_STATE{};
			voice.m_source->GetState(&state);
			voice.m_samples_played = state.SamplesPlayed;
			CheckHR(voice.m_source->Stop(), "IXAudio2SourceVoice::Stop");
		}
		voice.m_playback = EPlaybackState::Paused;
	}

	// Stop a voice and reset its transport position.
	void Engine::VoiceStop(VoiceHandle voice_handle)
	{
		m_impl->CheckThread();
		auto& voice = m_impl->VoiceFromHandle(voice_handle);
		if (voice.m_source)
		{
			voice.m_source->Stop();
			voice.m_source->FlushSourceBuffers();
			voice.m_source.reset();
		}
		voice.m_playback = EPlaybackState::Stopped;
		voice.m_virtualized = false;
		voice.m_samples_played = 0;
	}

	// Return the observable state of a voice.
	VoiceState Engine::VoiceStateGet(VoiceHandle voice_handle) const
	{
		m_impl->CheckThread();
		auto& voice = m_impl->VoiceFromHandle(voice_handle);
		auto samples_played = voice.m_samples_played;
		if (voice.m_source)
		{
			auto state = XAUDIO2_VOICE_STATE{};
			voice.m_source->GetState(&state);
			samples_played = state.SamplesPlayed;
		}

		return {
			.header = {sizeof(VoiceState), AUDIO_STRUCT_VERSION},
			.playback = voice.m_playback,
			.spatial = voice.m_has_emitter,
			.virtualized = voice.m_virtualized,
			.reserved = 0,
			.samples_played = samples_played,
		};
	}

	// Set the engine's single rendered listener.
	void Engine::ListenerSet(ListenerState const& listener)
	{
		m_impl->CheckThread();
		ValidateStruct(listener, "ListenerState");
		Position(listener.position);
		Position(listener.velocity);
		ValidateBasis(listener.forward, listener.up, "Listener orientation");
		m_impl->m_listener = listener;
	}

	// Enable or update spatial properties for a mono voice.
	void Engine::VoiceEmitterSet(VoiceHandle voice_handle, EmitterState const& emitter)
	{
		m_impl->CheckThread();
		ValidateStruct(emitter, "EmitterState");
		Position(emitter.position);
		Position(emitter.velocity);
		ValidateBasis(emitter.forward, emitter.up, "Emitter orientation");
		if (!std::isfinite(emitter.min_distance) || !std::isfinite(emitter.max_distance) || !(emitter.min_distance > 0.0f) || !(emitter.max_distance > emitter.min_distance))
			throw std::invalid_argument("Emitter distance range is invalid");
		if (!std::isfinite(emitter.cone_inner_angle) || !std::isfinite(emitter.cone_outer_angle) || !(emitter.cone_inner_angle >= 0.0f && emitter.cone_inner_angle <= emitter.cone_outer_angle && emitter.cone_outer_angle <= X3DAUDIO_2PI))
			throw std::invalid_argument("Emitter cone angles are invalid");
		if (!std::isfinite(emitter.cone_outer_gain) || !std::isfinite(emitter.doppler_scale) || !std::isfinite(emitter.obstruction) || !std::isfinite(emitter.occlusion) || !std::isfinite(emitter.reverb_send))
			throw std::invalid_argument("Emitter acoustic parameters must be finite");
		auto& voice = m_impl->VoiceFromHandle(voice_handle);
		voice.m_emitter = emitter;
		voice.m_has_emitter = true;
	}

	// Disable spatial processing for a voice.
	void Engine::VoiceEmitterClear(VoiceHandle voice_handle)
	{
		m_impl->CheckThread();
		auto& voice = m_impl->VoiceFromHandle(voice_handle);
		voice.m_has_emitter = false;
	}

	// Set the linear gain for a mixer bus.
	void Engine::BusGainSet(EBus bus, float gain)
	{
		m_impl->CheckThread();
		if (bus < EBus::Effects || bus >= EBus::Count || !std::isfinite(gain) || gain < 0.0f)
			throw std::invalid_argument("Audio bus or gain is invalid");

		CheckHR(m_impl->m_buses.at(static_cast<std::size_t>(bus))->SetVolume(gain), "IXAudio2Voice::SetVolume");
	}

	// Create a resident stream by copying the complete bytes of an Ogg Vorbis file for background decoding.
	StreamHandle Engine::StreamCreateOgg(StreamDesc const& desc, std::span<std::byte const> ogg_file)
	{
		m_impl->CheckThread();
		ValidateStruct(desc, "StreamDesc");
		if (desc.bus < EBus::Effects || desc.bus >= EBus::Count || desc.volume < 0.0f || desc.pitch <= 0.0f)
			throw std::invalid_argument("Stream description contains invalid routing, gain, or pitch");
		if (ogg_file.empty())
			throw std::invalid_argument("Ogg Vorbis file data is empty");

		auto lock = std::lock_guard<std::mutex>(m_impl->m_streams_mutex);
		auto live_count = std::uint32_t{};
		for (auto const& slot : m_impl->m_streams)
			live_count += slot.m_object != nullptr;
		if (live_count >= m_impl->m_config.max_streams)
			throw Exception(EStatus::ResourceLimit, "Audio stream capacity reached");

		// Copy the compressed file into engine ownership before parsing so the caller's buffer can be freed immediately.
		auto stream = std::make_unique<Stream>(desc, std::vector<std::byte>(ogg_file.begin(), ogg_file.end()));
		m_impl->RealizeStream(*stream);

		for (auto i = std::size_t{}; i != m_impl->m_streams.size(); ++i)
		{
			auto& slot = m_impl->m_streams[i];
			if (!slot.m_object)
			{
				slot.m_object = std::move(stream);
				return MakeHandle(i, slot.m_generation);
			}
		}

		m_impl->m_streams.push_back({});
		m_impl->m_streams.back().m_object = std::move(stream);
		return MakeHandle(m_impl->m_streams.size() - 1, m_impl->m_streams.back().m_generation);
	}

	// Stop and release a stream, discarding any buffered decode state.
	void Engine::StreamDestroy(StreamHandle stream_handle)
	{
		m_impl->CheckThread();
		auto lock = std::lock_guard<std::mutex>(m_impl->m_streams_mutex);
		auto index = HandleIndex(stream_handle);
		auto& stream = m_impl->StreamFromHandle(stream_handle);
		if (stream.m_source_voice)
		{
			if (stream.m_source_started)
				stream.m_source_voice->Stop();
			stream.m_source_voice.reset();
		}
		stream.m_source_started = false;
		stream.m_callback.Reset();
		m_impl->m_streams[index].m_object.reset();
		AdvanceGeneration(m_impl->m_streams[index].m_generation);
	}

	// Start or restart a stream from its beginning, or resume a paused stream from its retained position.
	void Engine::StreamPlay(StreamHandle stream_handle)
	{
		m_impl->CheckThread();
		auto lock = std::lock_guard<std::mutex>(m_impl->m_streams_mutex);
		auto& stream = m_impl->StreamFromHandle(stream_handle);
		if (stream.m_playback == EPlaybackState::Paused && stream.m_source_voice)
		{
			stream.m_playback = EPlaybackState::Playing;
			if (!stream.m_inflight.empty())
			{
				CheckHR(stream.m_source_voice->Start(), "IXAudio2SourceVoice::Start");
				stream.m_source_started = true;
			}
			return;
		}

		// (Re)start from the beginning: discard any queued audio and reseek decode to the start.
		if (stream.m_source_voice)
		{
			if (stream.m_source_started)
				stream.m_source_voice->Stop();
			stream.m_source_voice.reset();
		}
		stream.m_source_started = false;
		stream.m_callback.Reset();
		stream.m_inflight.clear();
		stream.m_loops_remaining = stream.m_desc.loop_count;
		stream.Seek(0);
		m_impl->RealizeStream(stream);
		stream.m_playback = EPlaybackState::Playing;
	}

	// Pause a playing stream without discarding its decode position or queued audio.
	void Engine::StreamPause(StreamHandle stream_handle)
	{
		m_impl->CheckThread();
		auto lock = std::lock_guard<std::mutex>(m_impl->m_streams_mutex);
		auto& stream = m_impl->StreamFromHandle(stream_handle);
		if (stream.m_source_voice && stream.m_source_started)
			CheckHR(stream.m_source_voice->Stop(), "IXAudio2SourceVoice::Stop");
		stream.m_source_started = false;
		stream.m_playback = EPlaybackState::Paused;
	}

	// Stop a stream and reset its decode position to the beginning.
	void Engine::StreamStop(StreamHandle stream_handle)
	{
		m_impl->CheckThread();
		auto lock = std::lock_guard<std::mutex>(m_impl->m_streams_mutex);
		auto& stream = m_impl->StreamFromHandle(stream_handle);
		if (stream.m_source_voice)
		{
			if (stream.m_source_started)
				stream.m_source_voice->Stop();
			stream.m_source_voice.reset();
		}
		stream.m_source_started = false;
		stream.m_callback.Reset();
		stream.m_inflight.clear();
		stream.m_loops_remaining = stream.m_desc.loop_count;
		stream.Seek(0);
		stream.m_playback = EPlaybackState::Stopped;
	}

	// Reposition decoding to a PCM frame, discarding any buffered-but-unsubmitted audio.
	void Engine::StreamSeek(StreamHandle stream_handle, std::uint64_t pcm_position)
	{
		m_impl->CheckThread();
		auto lock = std::lock_guard<std::mutex>(m_impl->m_streams_mutex);
		auto& stream = m_impl->StreamFromHandle(stream_handle);
		auto playback = stream.m_playback;
		if (stream.m_source_voice)
		{
			if (stream.m_source_started)
				stream.m_source_voice->Stop();
			stream.m_source_voice.reset();
		}
		stream.m_source_started = false;
		stream.m_callback.Reset();
		stream.m_inflight.clear();
		stream.Seek(pcm_position);
		if (playback != EPlaybackState::Stopped)
			m_impl->RealizeStream(stream);
	}

	// Return the observable transport and decode state of a stream.
	StreamState Engine::StreamStateGet(StreamHandle stream_handle) const
	{
		m_impl->CheckThread();
		auto lock = std::lock_guard<std::mutex>(m_impl->m_streams_mutex);
		auto& stream = m_impl->StreamFromHandle(stream_handle);
		return {
			.header = {sizeof(StreamState), AUDIO_STRUCT_VERSION},
			.playback = stream.m_playback,
			.channel_count = stream.m_channel_count,
			.sample_rate = stream.m_sample_rate,
			.pcm_position = stream.m_position_base,
			.pcm_total = stream.m_pcm_total,
			.underrun_count = stream.m_underrun_count,
		};
	}

	// Commit spatial state, process callback flags, and publish owner-thread events.
	void Engine::Update()
	{
		m_impl->CheckThread();
		if (FAILED(m_impl->m_critical_error.exchange(S_OK, std::memory_order_acq_rel)))
			m_impl->ResetGraph();

		for (auto i = std::size_t{}; i != m_impl->m_voices.size(); ++i)
		{
			auto& slot = m_impl->m_voices[i];
			if (!slot.m_object)
				continue;

			auto handle = MakeHandle(i, slot.m_generation);
			auto& voice = *slot.m_object;
			if (voice.m_callback.m_ended.exchange(false, std::memory_order_acq_rel))
			{
				voice.m_playback = EPlaybackState::Stopped;
				voice.m_virtualized = false;
				voice.m_samples_played = 0;
				voice.m_source.reset();
				m_impl->AddEvent(EEvent::VoiceEnded, handle);
			}
			if (voice.m_playback == EPlaybackState::Playing)
				m_impl->ApplySpatial(voice);
		}

		while (m_impl->RenderedVoiceCount() < m_impl->m_config.max_rendered_voices)
		{
			// Promote the highest-priority waiting logical voice whenever a physical slot becomes available.
			auto candidate_index = m_impl->m_voices.size();
			auto candidate_priority = std::uint32_t{};
			for (auto i = std::size_t{}; i != m_impl->m_voices.size(); ++i)
			{
				auto const& slot = m_impl->m_voices[i];
				if (!slot.m_object || !slot.m_object->m_virtualized || slot.m_object->m_playback != EPlaybackState::Playing)
					continue;
				if (candidate_index != m_impl->m_voices.size() && slot.m_object->m_desc.priority <= candidate_priority)
					continue;

				candidate_index = i;
				candidate_priority = slot.m_object->m_desc.priority;
			}
			if (candidate_index == m_impl->m_voices.size())
				break;

			auto& candidate = m_impl->m_voices[candidate_index];
			auto handle = MakeHandle(candidate_index, candidate.m_generation);
			m_impl->Realize(handle, *candidate.m_object);
			m_impl->Submit(*candidate.m_object);
			m_impl->ApplySpatial(*candidate.m_object);
			CheckHR(candidate.m_object->m_source->Start(), "IXAudio2SourceVoice::Start");
		}

		auto streams_lock = std::lock_guard<std::mutex>(m_impl->m_streams_mutex);
		for (auto i = std::size_t{}; i != m_impl->m_streams.size(); ++i)
		{
			auto& slot = m_impl->m_streams[i];
			if (!slot.m_object)
				continue;

			m_impl->PumpStream(MakeHandle(i, slot.m_generation), *slot.m_object);
		}
	}

	// Copy and consume buffered runtime events.
	std::uint32_t Engine::EventsCopy(std::span<Event> events)
	{
		m_impl->CheckThread();
		auto count = static_cast<std::uint32_t>(std::min(events.size(), m_impl->m_events.size()));
		for (auto i = std::uint32_t{}; i != count; ++i)
		{
			events[i] = m_impl->m_events.front();
			m_impl->m_events.pop_front();
		}
		return count;
	}

	// Return bounded runtime counters.
	Diagnostics Engine::DiagnosticsGet() const
	{
		m_impl->CheckThread();
		auto diagnostics = Diagnostics{
			.header = {sizeof(Diagnostics), AUDIO_STRUCT_VERSION},
			.output_channel_count = m_impl->m_output_channels,
			.output_sample_rate = m_impl->m_output_sample_rate,
			.engine_buffer_ms = m_impl->m_config.max_engine_buffer_ms,
		};
		for (auto const& slot : m_impl->m_voices)
		{
			if (!slot.m_object)
				continue;

			++diagnostics.logical_voice_count;
			diagnostics.rendered_voice_count += slot.m_object->m_source != nullptr;
			diagnostics.playing_voice_count += slot.m_object->m_playback == EPlaybackState::Playing;
			diagnostics.virtualized_voice_count += slot.m_object->m_virtualized;
		}
		diagnostics.queued_event_count = static_cast<std::uint32_t>(m_impl->m_events.size());
		diagnostics.event_overflow_count = m_impl->m_event_overflow_count;
		diagnostics.device_reset_count = m_impl->m_device_reset_count;
		return diagnostics;
	}
}
