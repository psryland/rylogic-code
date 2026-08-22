//*********************************************
// Audio Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#pragma once

#ifndef _WIN32_WINNT
#define _WIN32_WINNT _WIN32_WINNT_WIN10
#elif _WIN32_WINNT < _WIN32_WINNT_WIN10
#error "_WIN32_WINNT >= _WIN32_WINNT_WIN10 required"
#endif

// Standard and Windows dependencies shared by the audio module.
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <deque>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <mutex>
#include <numeric>
#include <optional>
#include <random>
#include <span>
#include <stdexcept>
#include <string>
#include <thread>
#include <type_traits>
#include <unordered_set>
#include <utility>
#include <vector>

#include <sdkddkver.h>
#include <windows.h>
#include <mmreg.h>
#include <xaudio2.h>
#include <xaudio2fx.h>
#include <x3daudio.h>

// The Ogg container and Vorbis codec sources are vendored under sdk\ogg and sdk\vorbis and
// compiled directly into the audio-static library; see audio.vcxproj.
#include <ogg/ogg.h>
#include <vorbis/codec.h>
#include <vorbis/vorbisfile.h>

// Rylogic dependencies are centralized here so sibling audio headers remain module-local.
#include "pr/common/assert.h"
#include "pr/common/cast.h"
#include "pr/common/fmt.h"
#include "pr/common/hresult.h"
#include "pr/common/refptr.h"
#include "pr/math/math.h"

#define PR_DBG_AUDIO PR_DBG

namespace pr::audio
{
	class Engine;

	// Sample rate value in samples per second.
	struct ESampleRate
	{
		enum type_t
		{
			_11025 = 11025,
			_22050 = 22050,
			_44100 = 44100,
			_48000 = 48000,
			_88200 = 88200,
			_96000 = 96000,
		};

		int value;

		constexpr ESampleRate(int rate)
			: value(rate)
		{}
		constexpr ESampleRate(type_t rate)
			: value(rate)
		{}
		constexpr operator int() const
		{
			return value;
		}
	};
}
