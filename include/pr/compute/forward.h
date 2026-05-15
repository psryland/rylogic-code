//*********************************************
// Compute
//  Copyright (c) Rylogic Ltd 2026
//*********************************************
#pragma once

#ifndef PR_DBG_RDR
#define PR_DBG_RDR PR_DBG
#endif

// Set this in the project settings, not here
#ifndef PR_RDR_RUNTIME_SHADERS
#define PR_RDR_RUNTIME_SHADERS 0
#endif

#include <vector>
#include <string>
#include <list>
#include <new>
#include <memory>
#include <sstream>
#include <fstream>
#include <unordered_map>
#include <algorithm>
#include <regex>
#include <optional>
#include <ranges>
#include <functional>
#include <execution>
#include <filesystem>
#include <chrono>
#include <limits>
#include <span>
#include <tuple>
#include <source_location>
#include <type_traits>
#include <variant>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <latch>
#include <future>
#include <cwctype>

#include <intrin.h>
#include <malloc.h>
#include <sdkddkver.h>
#include <winsock2.h>
#include <windows.h>
#include <dxgi.h>
#include <dxgidebug.h>
#include <d3d12.h>
#include <d3d12sdklayers.h>
#include <d3d11on12.h>
#include <dxgi1_4.h>
#include <dxgitype.h>
#include <dxcapi.h>

#include "pr/algorithm/algorithm.h"
#include "pr/common/allocator.h"
#include "pr/common/assert.h"
#include "pr/common/build_options.h"
#include "pr/common/cast.h"
#include "pr/common/coalesce.h"
#include "pr/common/d3dptr.h"
#include "pr/common/event_handler.h"
#include "pr/common/flags_enum.h"
#include "pr/common/fmt.h"
#include "pr/common/guid.h"
#include "pr/common/hash.h"
#include "pr/common/hresult.h"
#include "pr/common/min_max_fix.h"
#include "pr/common/range.h"
#include "pr/common/refcount.h"
#include "pr/common/refptr.h"
#include "pr/common/resource.h"
#include "pr/common/scope.h"
#include "pr/common/to.h"
#include "pr/container/byte_data.h"
#include "pr/container/deque.h"
#include "pr/container/ring.h"
#include "pr/container/vector.h"
#include "pr/container/vector_map.h"
#include "pr/geometry/geometry.h"
#include "pr/gfx/colour.h"
#include "pr/hlsl/interop.h"
#include "pr/hlsl/shader_registers.h"
#include "pr/macros/enum.h"
#include "pr/math/math.h"
#include "pr/math/conversion.h"
#include "pr/common/bit_fields.h"
#include "pr/meta/alignment_of.h"
#include "pr/str/char8.h"
#include "pr/str/string.h"
#include "pr/str/to_string.h"
#include "pr/win32/win32.h"

namespace pr::rdr12
{
	// Types
	using byte = unsigned char;
	using float4_t = float[4];
	using RdrId = std::uintptr_t;
	using Range = pr::Range<int64_t>;
	using Handle = win32::Handle;
	template <typename T> using Scope = pr::Scope<T>;
	template <typename T> using Allocator = pr::aligned_alloc<T>;
	template <typename T> using alloc_traits = std::allocator_traits<Allocator<T>>;
	template <typename T> using RefCounted = pr::RefCount<T>;
	template <typename T> using RefPtr = pr::RefPtr<T>;

	// Use the shader register types from pr::hlsl
	using hlsl::ECBufReg;
	using hlsl::ESRVReg;
	using hlsl::EUAVReg;
	using hlsl::ESamReg;
	using hlsl::ShaderReg;

	// Constants
	static constexpr Range RangeZero = Range::Zero();

	// Enumerations
	using EGeom = geometry::EGeom;
	using ETopo = geometry::ETopo;
	using ETopoGroup = geometry::ETopoGroup;

	// Compute resources
	struct Descriptor;
	struct DescriptorStore;
	struct FeatureSupport;
	struct GpuSync;
	struct Image;
	struct ImageWithData;
	struct ResDesc;
	struct RootSig;
	struct SamDesc;
	struct Vert;
}
