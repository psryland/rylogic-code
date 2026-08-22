//*********************************************
// Audio Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// Thin C-linkage wrapper around the native C++ engine.
#include "pr/audio/forward.h"
#include "pr/audio/audio-dll.h"
#include "audio/dll/context.h"

using namespace pr::audio;

namespace
{
	thread_local std::string g_last_error;
	std::mutex g_context_mutex;
	std::shared_ptr<Context> g_ctx;
	std::atomic<std::uint32_t> g_next_engine_generation{1};

	// Encode a generation and slot index into a non-zero engine handle.
	EngineHandle MakeEngineHandle(std::size_t index, std::uint32_t generation)
	{
		return (std::uint64_t{generation} << 32) | (std::uint64_t{index} + 1);
	}

	// Return the engine slot index encoded in a public handle.
	std::size_t EngineIndex(EngineHandle handle)
	{
		auto low = static_cast<std::uint32_t>(handle);
		if (low == 0)
			throw std::invalid_argument("Audio engine handle is null");

		return static_cast<std::size_t>(low - 1);
	}

	// Advance an engine generation without producing zero.
	void AdvanceGeneration(std::uint32_t& generation)
	{
		++generation;
		if (generation == 0)
			generation = 1;
	}

	// Allocate a process-wide generation so handles can never alias across recreated DLL contexts.
	std::uint32_t NextEngineGeneration()
	{
		for (;;)
		{
			auto generation = g_next_engine_generation.fetch_add(1, std::memory_order_relaxed);
			if (generation != 0)
				return generation;
		}
	}

	// Store and report one synchronous ABI diagnostic.
	EStatus Fail(std::shared_ptr<Context> const& context, EStatus status, char const* message, char const* file, int line)
	{
		g_last_error = message;
		if (context && context->m_error_cb)
		{
			// A foreign callback must not be able to unwind through the stable C ABI.
			try
			{
				context->m_error_cb(message, file, line);
			}
			catch (...)
			{}
		}
		return status;
	}

	// Translate native failures into stable status values without crossing the ABI.
	template <typename Func>
	EStatus ApiCall(Func&& func, char const* file, int line)
	{
		auto context = TryPinDll();
		try
		{
			if (!context)
				throw std::runtime_error("Audio is not initialised");

			auto lock = LockGuard{context->m_mutex};
			func(*context);
			return EStatus::Success;
		}
		catch (Exception const& ex)
		{
			return Fail(context, ex.Status(), ex.what(), file, line);
		}
		catch (std::length_error const& ex)
		{
			return Fail(context, EStatus::BufferTooSmall, ex.what(), file, line);
		}
		catch (std::invalid_argument const& ex)
		{
			return Fail(context, EStatus::InvalidArgument, ex.what(), file, line);
		}
		catch (std::exception const& ex)
		{
			return Fail(context, EStatus::InternalError, ex.what(), file, line);
		}
		catch (...)
		{
			return Fail(context, EStatus::InternalError, "Unknown native audio failure", file, line);
		}
	}

	// Validate a required versioned structure.
	template <typename T>
	void Validate(T const* value, char const* name)
	{
		if (value == nullptr)
			throw std::invalid_argument(std::string(name) + " is null");
		if (value->header.size < sizeof(T) || value->header.version != AUDIO_STRUCT_VERSION)
			throw Exception(EStatus::InvalidStruct, (std::string(name) + " has an incompatible size or version").c_str());
	}

	// Resolve an engine and enforce its mutable owner-thread contract.
	Engine& EngineFromHandle(Context& context, EngineHandle handle)
	{
		auto index = EngineIndex(handle);
		if (index >= context.m_engines.size())
			throw std::invalid_argument("Audio engine handle is invalid");

		auto& slot = context.m_engines[index];
		if (slot.m_generation != static_cast<std::uint32_t>(handle >> 32) || !slot.m_engine)
			throw Exception(EStatus::StaleHandle, "Audio engine handle is stale");
		if (slot.m_owner_thread != std::this_thread::get_id())
			throw Exception(EStatus::WrongThread, "Mutable audio operation called from a non-owner OS thread");

		return *slot.m_engine;
	}
}

namespace pr::audio
{
	Context::Context(ReportErrorCB error_cb)
		: m_inits()
		, m_mutex()
		, m_error_cb(error_cb)
		, m_engines()
	{}

	std::shared_ptr<Context> PinDll()
	{
		auto lock = std::lock_guard{g_context_mutex};
		if (!g_ctx)
			throw std::runtime_error("Audio is not initialised");

		return g_ctx;
	}

	std::shared_ptr<Context> TryPinDll()
	{
		auto lock = std::lock_guard{g_context_mutex};
		return g_ctx;
	}
}

// DLL entry point.
HINSTANCE g_hInstance;
BOOL APIENTRY DllMain(HMODULE instance, DWORD reason, LPVOID)
{
	switch (reason)
	{
		case DLL_PROCESS_ATTACH:
		{
			g_hInstance = instance;
			break;
		}
		case DLL_PROCESS_DETACH:
		{
			g_hInstance = nullptr;
			break;
		}
		case DLL_THREAD_ATTACH:
		case DLL_THREAD_DETACH:
		{
			break;
		}
		default:
		{
			break;
		}
	}
	return TRUE;
}

// Register one process initialization token.
AUDIO_API DllHandle __stdcall Audio_Initialise(ReportErrorCB global_error_cb)
{
	try
	{
		auto context_lock = std::lock_guard{g_context_mutex};
		if (!g_ctx)
			g_ctx = std::make_shared<Context>(global_error_cb);

		static std::uintptr_t next_handle = 0;
		auto handle = reinterpret_cast<DllHandle>(++next_handle);
		auto state_lock = LockGuard{g_ctx->m_mutex};
		g_ctx->m_inits.insert(handle);
		return handle;
	}
	catch (std::exception const& ex)
	{
		g_last_error = ex.what();
		if (global_error_cb)
			global_error_cb(ex.what(), __FILE__, __LINE__);
		return nullptr;
	}
}

// Release one initialization token when no engines remain.
AUDIO_API void __stdcall Audio_Shutdown(DllHandle context_handle)
{
	auto context = std::shared_ptr<Context>{};
	auto error = static_cast<char const*>(nullptr);
	{
		auto context_lock = std::lock_guard{g_context_mutex};
		context = g_ctx;
		if (!context)
			return;

		auto state_lock = LockGuard{context->m_mutex};
		if (!context->m_inits.contains(context_handle))
		{
			error = "Audio_Shutdown received an invalid context handle";
		}
		else
		{
			auto live_engine = std::any_of(context->m_engines.begin(), context->m_engines.end(), [](EngineSlot const& slot)
				{
					return slot.m_engine != nullptr;
				});
			if (context->m_inits.size() == 1 && live_engine)
			{
				error = "Audio_Shutdown cannot release the final context while engines are alive";
			}
			else
			{
				context->m_inits.erase(context_handle);
				if (context->m_inits.empty())
					g_ctx.reset();
			}
		}
	}

	if (error != nullptr)
		Fail(context, EStatus::InvalidHandle, error, __FILE__, __LINE__);
}

// Release a forgotten context token and any final native context from any thread.
AUDIO_API void __stdcall Audio_ContextAbandon(DllHandle context_handle)
{
	try
	{
		auto abandoned = std::shared_ptr<Context>{};
		{
			auto context_lock = std::lock_guard{g_context_mutex};
			auto context = g_ctx;
			if (!context)
				return;

			auto state_lock = LockGuard{context->m_mutex};
			if (!context->m_inits.contains(context_handle))
				return;

			context->m_inits.erase(context_handle);
			if (context->m_inits.empty())
				abandoned = std::move(g_ctx);
		}

		// Destruction can join decoder workers, so keep it outside the process and context locks.
		abandoned.reset();
	}
	catch (...)
	{}
}

// Return the implemented ABI version.
AUDIO_API std::uint32_t __stdcall Audio_ApiVersion()
{
	return AUDIO_API_VERSION;
}

// Return the native size of a public structure.
AUDIO_API EStatus __stdcall Audio_StructSize(EStructId struct_id, std::uint32_t* size)
{
	return ApiCall([&](Context&)
		{
			if (size == nullptr)
				throw std::invalid_argument("Audio_StructSize output is null");

			switch (struct_id)
			{
				case EStructId::Config: { *size = sizeof(Config); break; }
				case EStructId::ListenerState: { *size = sizeof(ListenerState); break; }
				case EStructId::EmitterState: { *size = sizeof(EmitterState); break; }
				case EStructId::VoiceDesc: { *size = sizeof(VoiceDesc); break; }
				case EStructId::VoiceState: { *size = sizeof(VoiceState); break; }
				case EStructId::Event: { *size = sizeof(Event); break; }
				case EStructId::Diagnostics: { *size = sizeof(Diagnostics); break; }
				case EStructId::StreamDesc: { *size = sizeof(StreamDesc); break; }
				case EStructId::StreamState: { *size = sizeof(StreamState); break; }
				default: { throw std::invalid_argument("Unknown audio structure identifier"); }
			}
		}, __FILE__, __LINE__);
}

// Copy the calling thread's last synchronous audio diagnostic.
AUDIO_API EStatus __stdcall Audio_LastError(char* buffer, std::uint32_t capacity, std::uint32_t* required)
{
	if (required == nullptr)
		return EStatus::InvalidArgument;

	*required = static_cast<std::uint32_t>(g_last_error.size() + 1);
	if (buffer == nullptr || capacity < *required)
		return EStatus::BufferTooSmall;

	std::memcpy(buffer, g_last_error.c_str(), *required);
	return EStatus::Success;
}

// Create one owner-thread audio engine.
AUDIO_API EStatus __stdcall Audio_EngineCreate(DllHandle context_handle, Config const* config, EngineHandle* engine)
{
	return ApiCall([&](Context& context)
		{
			if (!context.m_inits.contains(context_handle))
				throw std::invalid_argument("Audio context handle is invalid");
			if (engine == nullptr)
				throw std::invalid_argument("Audio engine output is null");
			if (config != nullptr)
				Validate(config, "Config");

			auto instance = std::make_unique<Engine>(config != nullptr ? *config : DefaultConfig());
			for (auto i = std::size_t{}; i != context.m_engines.size(); ++i)
			{
				auto& slot = context.m_engines[i];
				if (!slot.m_engine)
				{
					slot.m_generation = NextEngineGeneration();
					slot.m_owner_thread = std::this_thread::get_id();
					slot.m_engine = std::move(instance);
					*engine = MakeEngineHandle(i, slot.m_generation);
					return;
				}
			}

			context.m_engines.push_back({});
			auto& slot = context.m_engines.back();
			slot.m_generation = NextEngineGeneration();
			slot.m_owner_thread = std::this_thread::get_id();
			slot.m_engine = std::move(instance);
			*engine = MakeEngineHandle(context.m_engines.size() - 1, slot.m_generation);
		}, __FILE__, __LINE__);
}

// Destroy one owner-thread audio engine.
AUDIO_API EStatus __stdcall Audio_EngineDestroy(EngineHandle engine)
{
	return ApiCall([&](Context& context)
		{
			EngineFromHandle(context, engine);
			auto& slot = context.m_engines[EngineIndex(engine)];
			slot.m_engine.reset();
			AdvanceGeneration(slot.m_generation);
		}, __FILE__, __LINE__);
}

// Release a forgotten engine from any thread without allowing finalizer failures to escape.
AUDIO_API void __stdcall Audio_EngineAbandon(EngineHandle engine)
{
	try
	{
		auto context = TryPinDll();
		if (!context)
			return;

		auto lock = LockGuard{context->m_mutex};
		auto index = EngineIndex(engine);
		if (index >= context->m_engines.size())
			return;

		auto& slot = context->m_engines[index];
		if (slot.m_generation != static_cast<std::uint32_t>(engine >> 32) || !slot.m_engine)
			return;

		slot.m_engine.reset();
		AdvanceGeneration(slot.m_generation);
	}
	catch (...)
	{}
}

#define AUDIO_ENGINE_CALL(name, parameters, expression) \
	AUDIO_API EStatus __stdcall name parameters \
	{ \
		return ApiCall([&](Context& context) { expression; }, __FILE__, __LINE__); \
	}

AUDIO_ENGINE_CALL(Audio_EngineUpdate, (EngineHandle engine), EngineFromHandle(context, engine).Update())
AUDIO_ENGINE_CALL(Audio_ListenerSet, (EngineHandle engine, ListenerState const* listener), Validate(listener, "ListenerState"); EngineFromHandle(context, engine).ListenerSet(*listener))
AUDIO_ENGINE_CALL(Audio_BusGainSet, (EngineHandle engine, EBus bus, float gain), EngineFromHandle(context, engine).BusGainSet(bus, gain))
AUDIO_ENGINE_CALL(Audio_ClipDestroy, (EngineHandle engine, ClipHandle clip), EngineFromHandle(context, engine).ClipDestroy(clip))
AUDIO_ENGINE_CALL(Audio_VoiceDestroy, (EngineHandle engine, VoiceHandle voice), EngineFromHandle(context, engine).VoiceDestroy(voice))
AUDIO_ENGINE_CALL(Audio_VoicePlay, (EngineHandle engine, VoiceHandle voice), EngineFromHandle(context, engine).VoicePlay(voice))
AUDIO_ENGINE_CALL(Audio_VoicePause, (EngineHandle engine, VoiceHandle voice), EngineFromHandle(context, engine).VoicePause(voice))
AUDIO_ENGINE_CALL(Audio_VoiceStop, (EngineHandle engine, VoiceHandle voice), EngineFromHandle(context, engine).VoiceStop(voice))
AUDIO_ENGINE_CALL(Audio_VoiceEmitterClear, (EngineHandle engine, VoiceHandle voice), EngineFromHandle(context, engine).VoiceEmitterClear(voice))
AUDIO_ENGINE_CALL(Audio_StreamDestroy, (EngineHandle engine, StreamHandle stream), EngineFromHandle(context, engine).StreamDestroy(stream))
AUDIO_ENGINE_CALL(Audio_StreamPlay, (EngineHandle engine, StreamHandle stream), EngineFromHandle(context, engine).StreamPlay(stream))
AUDIO_ENGINE_CALL(Audio_StreamPause, (EngineHandle engine, StreamHandle stream), EngineFromHandle(context, engine).StreamPause(stream))
AUDIO_ENGINE_CALL(Audio_StreamStop, (EngineHandle engine, StreamHandle stream), EngineFromHandle(context, engine).StreamStop(stream))
AUDIO_ENGINE_CALL(Audio_StreamSeek, (EngineHandle engine, StreamHandle stream, std::uint64_t pcm_position), EngineFromHandle(context, engine).StreamSeek(stream, pcm_position))

#undef AUDIO_ENGINE_CALL

// Create one resident WAV clip.
AUDIO_API EStatus __stdcall Audio_ClipCreateWave(EngineHandle engine, void const* wave_file, std::uint64_t wave_file_size, ClipHandle* clip)
{
	return ApiCall([&](Context& context)
		{
			if (wave_file == nullptr || wave_file_size == 0 || clip == nullptr || wave_file_size > std::numeric_limits<std::size_t>::max())
				throw std::invalid_argument("Audio WAV input or clip output is invalid");

			auto bytes = std::span{static_cast<std::byte const*>(wave_file), static_cast<std::size_t>(wave_file_size)};
			*clip = EngineFromHandle(context, engine).ClipCreateWave(bytes);
		}, __FILE__, __LINE__);
}

// Create one logical voice.
AUDIO_API EStatus __stdcall Audio_VoiceCreate(EngineHandle engine, VoiceDesc const* desc, VoiceHandle* voice)
{
	return ApiCall([&](Context& context)
		{
			Validate(desc, "VoiceDesc");
			if (voice == nullptr)
				throw std::invalid_argument("Audio voice output is null");

			*voice = EngineFromHandle(context, engine).VoiceCreate(*desc);
		}, __FILE__, __LINE__);
}

// Return one voice state.
AUDIO_API EStatus __stdcall Audio_VoiceStateGet(EngineHandle engine, VoiceHandle voice, VoiceState* state)
{
	return ApiCall([&](Context& context)
		{
			if (state == nullptr)
				throw std::invalid_argument("Audio voice state output is null");

			*state = EngineFromHandle(context, engine).VoiceStateGet(voice);
		}, __FILE__, __LINE__);
}

// Enable or update spatial properties for one voice.
AUDIO_API EStatus __stdcall Audio_VoiceEmitterSet(EngineHandle engine, VoiceHandle voice, EmitterState const* emitter)
{
	return ApiCall([&](Context& context)
		{
			Validate(emitter, "EmitterState");
			EngineFromHandle(context, engine).VoiceEmitterSet(voice, *emitter);
		}, __FILE__, __LINE__);
}

// Create one resident Ogg Vorbis stream by copying the complete compressed file into engine ownership.
AUDIO_API EStatus __stdcall Audio_StreamCreateOgg(EngineHandle engine, StreamDesc const* desc, void const* ogg_file, std::uint64_t ogg_file_size, StreamHandle* stream)
{
	return ApiCall([&](Context& context)
		{
			Validate(desc, "StreamDesc");
			if (ogg_file == nullptr || ogg_file_size == 0 || stream == nullptr || ogg_file_size > std::numeric_limits<std::size_t>::max())
				throw std::invalid_argument("Audio Ogg input or stream output is invalid");

			auto bytes = std::span{static_cast<std::byte const*>(ogg_file), static_cast<std::size_t>(ogg_file_size)};
			*stream = EngineFromHandle(context, engine).StreamCreateOgg(*desc, bytes);
		}, __FILE__, __LINE__);
}

// Return one stream's transport and decode state.
AUDIO_API EStatus __stdcall Audio_StreamStateGet(EngineHandle engine, StreamHandle stream, StreamState* state)
{
	return ApiCall([&](Context& context)
		{
			if (state == nullptr)
				throw std::invalid_argument("Audio stream state output is null");

			*state = EngineFromHandle(context, engine).StreamStateGet(stream);
		}, __FILE__, __LINE__);
}

// Copy and consume buffered owner-thread events.
AUDIO_API EStatus __stdcall Audio_EventsCopy(EngineHandle engine, Event* events, std::uint32_t capacity, std::uint32_t* required)
{
	return ApiCall([&](Context& context)
		{
			if (required == nullptr)
				throw std::invalid_argument("Audio event count output is null");

			auto& instance = EngineFromHandle(context, engine);
			auto diagnostics = instance.DiagnosticsGet();
			*required = diagnostics.queued_event_count;
			if (capacity < *required || events == nullptr && *required != 0)
				throw std::length_error("Audio event buffer is too small");

			instance.EventsCopy(std::span{events, events != nullptr ? capacity : 0U});
		}, __FILE__, __LINE__);
}

// Return engine diagnostics.
AUDIO_API EStatus __stdcall Audio_DiagnosticsGet(EngineHandle engine, Diagnostics* diagnostics)
{
	return ApiCall([&](Context& context)
		{
			if (diagnostics == nullptr)
				throw std::invalid_argument("Audio diagnostics output is null");

			*diagnostics = EngineFromHandle(context, engine).DiagnosticsGet();
		}, __FILE__, __LINE__);
}
