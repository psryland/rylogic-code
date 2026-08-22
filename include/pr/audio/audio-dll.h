//*********************************************
// Audio Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// Dependency-minimal, versioned C ABI for the native audio engine.
#pragma once

#ifdef AUDIO_EXPORTS
#define AUDIO_API __declspec(dllexport)
#else
#define AUDIO_API __declspec(dllimport)
#endif

#include <cstdint>
#include <utility>
#include <windows.h>
#include "pr/audio/types.h"

namespace pr::audio
{
	using DllHandle = unsigned char const*;

	// Pair a native callback with caller-owned context.
	template <typename FuncType>
	struct Callback
	{
		using FuncCB = FuncType;
		using CtxPtr = union { void const* cp; void* p; };

		CtxPtr m_ctx = {};
		FuncCB m_cb = {};

		template <typename... Args>
		auto operator()(Args&&... args) const
		{
			return m_cb(m_ctx.p, std::forward<Args>(args)...);
		}
		explicit operator bool() const
		{
			return m_cb != nullptr;
		}
	};

	using ReportErrorCB = Callback<void(__stdcall*)(void* ctx, char const* msg, char const* filepath, int line)>;
}

extern "C"
{
	AUDIO_API pr::audio::DllHandle __stdcall Audio_Initialise(pr::audio::ReportErrorCB global_error_cb);
	AUDIO_API void __stdcall Audio_Shutdown(pr::audio::DllHandle context);
	AUDIO_API void __stdcall Audio_ContextAbandon(pr::audio::DllHandle context);
	AUDIO_API std::uint32_t __stdcall Audio_ApiVersion();
	AUDIO_API pr::audio::EStatus __stdcall Audio_StructSize(pr::audio::EStructId struct_id, std::uint32_t* size);
	AUDIO_API pr::audio::EStatus __stdcall Audio_LastError(char* buffer, std::uint32_t capacity, std::uint32_t* required);

	AUDIO_API pr::audio::EStatus __stdcall Audio_EngineCreate(pr::audio::DllHandle context, pr::audio::Config const* config, pr::audio::EngineHandle* engine);
	AUDIO_API pr::audio::EStatus __stdcall Audio_EngineDestroy(pr::audio::EngineHandle engine);
	AUDIO_API void __stdcall Audio_EngineAbandon(pr::audio::EngineHandle engine);
	AUDIO_API pr::audio::EStatus __stdcall Audio_EngineUpdate(pr::audio::EngineHandle engine);
	AUDIO_API pr::audio::EStatus __stdcall Audio_ListenerSet(pr::audio::EngineHandle engine, pr::audio::ListenerState const* listener);
	AUDIO_API pr::audio::EStatus __stdcall Audio_BusGainSet(pr::audio::EngineHandle engine, pr::audio::EBus bus, float gain);

	AUDIO_API pr::audio::EStatus __stdcall Audio_ClipCreateWave(pr::audio::EngineHandle engine, void const* wave_file, std::uint64_t wave_file_size, pr::audio::ClipHandle* clip);
	AUDIO_API pr::audio::EStatus __stdcall Audio_ClipDestroy(pr::audio::EngineHandle engine, pr::audio::ClipHandle clip);

	AUDIO_API pr::audio::EStatus __stdcall Audio_VoiceCreate(pr::audio::EngineHandle engine, pr::audio::VoiceDesc const* desc, pr::audio::VoiceHandle* voice);
	AUDIO_API pr::audio::EStatus __stdcall Audio_VoiceDestroy(pr::audio::EngineHandle engine, pr::audio::VoiceHandle voice);
	AUDIO_API pr::audio::EStatus __stdcall Audio_VoicePlay(pr::audio::EngineHandle engine, pr::audio::VoiceHandle voice);
	AUDIO_API pr::audio::EStatus __stdcall Audio_VoicePause(pr::audio::EngineHandle engine, pr::audio::VoiceHandle voice);
	AUDIO_API pr::audio::EStatus __stdcall Audio_VoiceStop(pr::audio::EngineHandle engine, pr::audio::VoiceHandle voice);
	AUDIO_API pr::audio::EStatus __stdcall Audio_VoiceStateGet(pr::audio::EngineHandle engine, pr::audio::VoiceHandle voice, pr::audio::VoiceState* state);
	AUDIO_API pr::audio::EStatus __stdcall Audio_VoiceEmitterSet(pr::audio::EngineHandle engine, pr::audio::VoiceHandle voice, pr::audio::EmitterState const* emitter);
	AUDIO_API pr::audio::EStatus __stdcall Audio_VoiceEmitterClear(pr::audio::EngineHandle engine, pr::audio::VoiceHandle voice);

	AUDIO_API pr::audio::EStatus __stdcall Audio_StreamCreateOgg(pr::audio::EngineHandle engine, pr::audio::StreamDesc const* desc, void const* ogg_file, std::uint64_t ogg_file_size, pr::audio::StreamHandle* stream);
	AUDIO_API pr::audio::EStatus __stdcall Audio_StreamDestroy(pr::audio::EngineHandle engine, pr::audio::StreamHandle stream);
	AUDIO_API pr::audio::EStatus __stdcall Audio_StreamPlay(pr::audio::EngineHandle engine, pr::audio::StreamHandle stream);
	AUDIO_API pr::audio::EStatus __stdcall Audio_StreamPause(pr::audio::EngineHandle engine, pr::audio::StreamHandle stream);
	AUDIO_API pr::audio::EStatus __stdcall Audio_StreamStop(pr::audio::EngineHandle engine, pr::audio::StreamHandle stream);
	AUDIO_API pr::audio::EStatus __stdcall Audio_StreamSeek(pr::audio::EngineHandle engine, pr::audio::StreamHandle stream, std::uint64_t pcm_position);
	AUDIO_API pr::audio::EStatus __stdcall Audio_StreamStateGet(pr::audio::EngineHandle engine, pr::audio::StreamHandle stream, pr::audio::StreamState* state);

	AUDIO_API pr::audio::EStatus __stdcall Audio_EventsCopy(pr::audio::EngineHandle engine, pr::audio::Event* events, std::uint32_t capacity, std::uint32_t* required);
	AUDIO_API pr::audio::EStatus __stdcall Audio_DiagnosticsGet(pr::audio::EngineHandle engine, pr::audio::Diagnostics* diagnostics);
}
