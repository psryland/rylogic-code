//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2016
//*********************************************
#pragma once
#include <concepts>
#include <type_traits>
#include <span>
#include <memory>
#include <vector>
#include <array>
#include <ranges>
#include <unordered_set>
#include <unordered_map>
#include <algorithm>
#include <numeric>
#include <fstream>
#include <format>
#include <chrono>
#include <cstdio>
#include <cstdint>
#include <cassert>
#include <stdexcept>

#include "pr/common/assert.h"
#include "pr/common/to.h"
#include "pr/common/cast.h"
#include "pr/common/flags_enum.h"
#include "pr/common/scope.h"
#include "pr/common/event_handler.h"
#include "pr/common/bit_fields.h"
#include "pr/algorithm/algorithm.h"
#include "pr/str/to_string.h"
#include "pr/container/vector.h"
#include "pr/container/byte_data.h"
#include "pr/math/math.h"
#include "pr/collision/collision.h"
#include "pr/geometry/closest_point.h"
#include "pr/geometry/intersect.h"
#include "pr/hlsl/interop.h"
#include "pr/compute/shaders/compiler/shader_cache.h"

// Physics diagnostics/profile code defaults to debug-only. Define these explicitly to opt in/out per build.
#ifndef PR_DBG_PHYSICS
#define PR_DBG_PHYSICS PR_DBG
#endif

#ifndef PR_PHYSICS_DIAGNOSTICS
#define PR_PHYSICS_DIAGNOSTICS PR_DBG_PHYSICS
#endif

#ifndef PR_PHYSICS_PROFILE
#define PR_PHYSICS_PROFILE PR_DBG_PHYSICS
#endif

// Forward declare D3D12 device (avoids including d3d12.h)
struct ID3D12Device4;

namespace pr::physics
{
	// Import types into this namespace
	using namespace math::spatial;
	using namespace collision;
	using namespace hlsl;

	using BBox = math::BoundingBox<float>;
	using IShaderCache = ::pr::compute::shader_cache::IShaderCache;

	// Custom deleter for smart pointers
	template <typename T> struct Deleter {
		void operator()(T* p) const; // Implemented where 'T' is fully defined
	};

	// Forwards
	struct Engine;
	struct EngineConfig;
	struct RigidBody;
	struct Inertia;
	struct InertiaInv;
	struct RbContact;
	struct MaterialMap;
	struct Material;

	struct Gpu;
	struct GpuIntegrator;
	struct GpuSleepManager;
	struct GpuSortAndSweep;
	struct GpuCollisionDetector;
	struct GpuSelectiveRefresher;
	struct GpuResolver;
	struct GpuRigidBody;
	struct GpuSleepIsland;
	struct GpuShape;
	struct GpuCollisionPair;
	struct GpuContact;
	struct GpuResolveContact;
	struct GpuCollisionCounters;
	struct GpuMaterial;
	struct GpuBuffers;
	struct EngineBufferCache;
	struct ShapeCache;

	using MaterialMapPtr = std::unique_ptr<MaterialMap, Deleter<MaterialMap>>;
	using GpuPtr = std::unique_ptr<Gpu, Deleter<Gpu>>;
	using GpuIntegratorPtr = std::unique_ptr<GpuIntegrator, Deleter<GpuIntegrator>>;
	using GpuSleepManagerPtr = std::unique_ptr<GpuSleepManager, Deleter<GpuSleepManager>>;
	using GpuSortAndSweepPtr = std::unique_ptr<GpuSortAndSweep, Deleter<GpuSortAndSweep>>;
	using GpuCollisionDetectorPtr = std::unique_ptr<GpuCollisionDetector, Deleter<GpuCollisionDetector>>;
	using GpuSelectiveRefresherPtr = std::unique_ptr<GpuSelectiveRefresher, Deleter<GpuSelectiveRefresher>>;
	using GpuResolverPtr = std::unique_ptr<GpuResolver, Deleter<GpuResolver>>;
	using CachePtr = std::unique_ptr<EngineBufferCache, Deleter<EngineBufferCache>>;

	// Traits
	template <typename T>
	concept RigidBodyType = std::derived_from<T, RigidBody>;

	template <typename T>
	concept RigidBodyRange = std::ranges::random_access_range<T> && RigidBodyType<std::ranges::range_value_t<T>>;

	// Literals
	constexpr float operator ""_kg(long double mass)
	{
		return float(mass);
	}
	constexpr float operator ""_m(long double dist)
	{
		return float(dist);
	}
}
