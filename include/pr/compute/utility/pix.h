//*********************************************
// Compute
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#pragma once
#include <type_traits>
#include <filesystem>

// Defined in '/build/targets/WinPixEventRuntime.targets'
#ifndef PR_PIX_ENABLED
#define PR_PIX_ENABLED 0
#endif

#ifndef PR_COMPUTE_SHADER_DEBUG
#define PR_COMPUTE_SHADER_DEBUG 0
#endif

#if PR_PIX_ENABLED
#include <cassert>
#include <windows.h>
#include <pix3.h>
#endif

namespace pr::compute::pix
{
	// Load only the event runtime, returning null if unavailable. Capture engines are selected explicitly by the application.
	HMODULE LoadDll();

	// Mark CPU work on the calling thread without requiring a GPU command list.
	inline void BeginEvent(unsigned long colour, char const* name)
	{
		// The name is data, not a caller-supplied format string.
		#if PR_PIX_ENABLED
		PIXBeginEvent(colour, "%s", name);
		#else
		(void)colour, (void)name;
		#endif
	}

	// End the calling thread's current CPU event.
	inline void EndEvent()
	{
		// CPU PIX regions are paired on the same owner thread.
		#if PR_PIX_ENABLED
		PIXEndEvent();
		#endif
	}

	// Enable GPU capture before any D3D12 API calls, including adapter capability checks. Do not also load the timing capturer.
	inline void LoadLatestWinPixGpuCapturer()
	{
		// The capture layer must be installed before the application creates any D3D12 objects.
		#if PR_PIX_ENABLED
		auto h = PIXLoadLatestWinPixGpuCapturerLibrary();
		assert(h != nullptr && "Failed to load 'WinPixGpuCapturer.dll'");
		#endif
	}

	inline bool IsAttachedForGpuCapture()
	{
		#if PR_PIX_ENABLED
		return PIXIsAttachedForGpuCapture();
		#else
		return false;
		#endif
	}

	// Begin a GPU capture after loading the GPU capturer at application startup or launching through PIX for GPU capture.
	inline void BeginCapture(std::filesystem::path const& wpix_filepath)
	{
		// Starting capture here cannot install the capture layer on existing D3D12 objects.
		#if PR_PIX_ENABLED
		PIXCaptureParameters parameters = { .GpuCaptureParameters = {.FileName = wpix_filepath.c_str()} };
		auto r = PIXBeginCapture2(PIX_CAPTURE_GPU, &parameters);
		assert(r == S_OK && "PIX Capture error"); // Cannot load the WinPixGpuCapturer dll here. It needs to happen before the D3D device is created.
		#else
		(void)wpix_filepath;
		#endif
	}

	inline void EndCapture()
	{
		#if PR_PIX_ENABLED
		PIXEndCapture(FALSE);
		#endif
	}

	template<typename CONTEXT, typename... ARGS>
	inline void BeginEvent(CONTEXT* context, unsigned long colour, char const* format_string, ARGS... args)
	{
		#if PR_PIX_ENABLED
		PIXBeginEvent(context, colour, format_string, std::forward<ARGS>(args)...);
		#else
		(void)context, colour, format_string;
		#endif
	}

	template<typename CONTEXT>
	inline void EndEvent(CONTEXT* context)
	{
		#if PR_PIX_ENABLED
		PIXEndEvent(context);
		#else
		(void)context;
		#endif
	}

	// Detailed profiling is off by default. Set PR_PIX_DETAIL=1 before startup; the setting is cached on first use.
	inline bool DetailEnabled()
	{
		#if PR_PIX_ENABLED
		static bool const enabled = []
		{
			char value[2] = {};
			return GetEnvironmentVariableA("PR_PIX_DETAIL", value, sizeof(value)) == 1 && value[0] == '1';
		}();
		return enabled;
		#else
		return false;
		#endif
	}

	// Attach changing diagnostic values without splitting stable GPU region names used for aggregation.
	template<typename CONTEXT, typename... ARGS>
	inline void DetailMarker(CONTEXT* context, char const* format, ARGS... args)
	{
		#if PR_PIX_ENABLED
		if (DetailEnabled())
			PIXSetMarker(context, 0xFF90AA3F, format, args...);
		#else
		(void)context, (void)format;
		#endif
	}

	// Emit completed-readback diagnostics on the CPU without another GPU wait.
	template<typename... ARGS>
	inline void CompletedDetailMarker(char const* format, ARGS... args)
	{
		#if PR_PIX_ENABLED
		if (DetailEnabled())
			PIXSetMarker(0xFF90AA3F, format, args...);
		#else
		(void)format;
		#endif
	}

	// Opt-in nested GPU detail; names describe work, while markers carry variable identifiers.
	template<typename CONTEXT>
	struct DetailScope
	{
		CONTEXT* m_context;

		// Begin only when explicitly enabled before process startup.
		DetailScope(CONTEXT* context, char const* name)
			: m_context(DetailEnabled() ? context : nullptr)
		{
			if (m_context)
				BeginEvent(m_context, 0xFF90AA3F, "%s", name);
		}
		DetailScope(DetailScope const&) = delete;
		DetailScope& operator=(DetailScope const&) = delete;

		// End on the same command list as the matching begin.
		~DetailScope()
		{
			if (m_context)
				EndEvent(m_context);
		}
	};

	struct CaptureScope
	{
		bool m_active;

		CaptureScope(std::filesystem::path const& wpix_filepath, bool active)
			: m_active(active)
		{
			if (m_active) BeginCapture(wpix_filepath);
		}
		CaptureScope(CaptureScope&& rhs) noexcept
			: m_active(rhs.m_active)
		{
			rhs.m_active = false;
		}
		CaptureScope(CaptureScope const&) = delete;
		CaptureScope& operator=(CaptureScope&& rhs) noexcept
		{
			if (&rhs == this) return *this;
			m_active = rhs.m_active;
			rhs.m_active = false;
			return *this;
		}
		CaptureScope& operator=(CaptureScope const&) = delete;
		~CaptureScope()
		{
			if (m_active) EndCapture();
		}
	};

	template<typename CONTEXT>
	struct EventScope
	{
		CONTEXT* m_context;

		template<typename... ARGS>
		EventScope(CONTEXT* context, unsigned long colour, char const* format_string, ARGS... args)
			: m_context(context)
		{
			BeginEvent(context, colour, format_string, std::forward<ARGS>(args)...);
		}
		EventScope(EventScope&&) = delete;
		EventScope(EventScope const&) = delete;
		EventScope& operator=(EventScope&&) = delete;
		EventScope& operator=(EventScope const&) = delete;
		~EventScope()
		{
			EndEvent(m_context);
		}
	};
}
