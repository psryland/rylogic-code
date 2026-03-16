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
#include <unordered_map>
#include <algorithm>
#include <cassert>

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

// Forward declare D3D12 device (avoids including d3d12.h)
struct ID3D12Device4;

namespace pr::physics
{
	// Import types into this namespace
	using namespace math::spatial;
	using namespace collision;

	// Custom deleter for smart pointers
	template <typename T> struct Deleter {
		void operator()(T* p) const; // Implemented where 'T' is fully defined
	};

	// Forwards
	struct Material;
	struct MaterialMap;
	struct RigidBody;
	struct IBroadphase;
	struct IMaterials;
	struct Engine;
	struct RigidBodyDynamics;
	struct IntegrateOutput;
	struct IntegrateAABB;
	struct Inertia;
	struct InertiaInv;
	struct RbContact;

	struct Gpu;
	struct GpuIntegrator;
	struct GpuCollisionDetector;
	struct GpuResolver;
	struct GpuResolveContact;
	struct GpuShape;
	struct GpuCollisionPair;
	struct GpuContact;
	struct EngineBufferCache;
	struct GpuSortAndSweep;

	using GpuPtr = std::unique_ptr<Gpu, Deleter<Gpu>>;
	using GpuIntegratorPtr = std::unique_ptr<GpuIntegrator, Deleter<GpuIntegrator>>;
	using GpuSortAndSweepPtr = std::unique_ptr<GpuSortAndSweep, Deleter<GpuSortAndSweep>>;
	using GpuCollisionDetectorPtr = std::unique_ptr<GpuCollisionDetector, Deleter<GpuCollisionDetector>>;
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
