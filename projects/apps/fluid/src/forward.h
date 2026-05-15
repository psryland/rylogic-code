// Fluid
#pragma once

#include <cstdint>
#include <span>
#include <vector>
#include <unordered_set>
#include <stdexcept>
#include <windows.h>

#include "pr/common/cast.h"
#include "pr/common/fmt.h"
#include "pr/common/coalesce.h"
#include "pr/common/tweakables.h"
#include "pr/common/static_callback.h"
#include "pr/common/resource.h"
#include "pr/common/bit_fields.h"
#include "pr/common/ldraw.h"
#include "pr/math/math.h"
#include "pr/container/vector.h"
#include "pr/algorithm/kdtree.h"
#include "pr/camera/camera.h"
#include "pr/gui/wingui.h"
#include "pr/hlsl/interop.h"
#include "pr/win32/windows_com.h"

#include "pr/compute/compute.h"
#include "pr/view3d-12/view3d.h"
#include "pr/view3d-12/compute/fluid_simulation/fluid_simulation.h"

namespace pr::fluid
{
	using namespace tweakables;

	struct FluidVisualisation;
	struct particle_t
	{
		v4 pos;
		v4 vel;
		v4 acc;
		v4 surface;
	};

	using particles_t = std::vector<particle_t>;
	using IndexSet = std::unordered_set<int64_t>;

	using ComputeStep = ::pr::compute::ComputeStep;
	using EUsage = ::pr::compute::EUsage;
	using Image = ::pr::compute::Image;
	using ResDesc = ::pr::compute::ResDesc;
	using UpdateSubresourceScope = ::pr::compute::GfxUpdateSubresourceScope;
	using FluidSimulation = rdr12::compute::fluid::FluidSimulation<>;
	using SpatialPartition = ::pr::compute::spatial_partition::SpatialPartition<>;
	using ParticleCollision = ::pr::compute::particle_collision::ParticleCollision<>;
	using CollisionBuilder = ::pr::compute::particle_collision::CollisionBuilder;
	using CollisionPrim = ::pr::compute::particle_collision::Prim;
	using Particle = rdr12::compute::fluid::Particle;
	using Dynamics = rdr12::compute::fluid::Dynamics;
	using Gpu = FluidSimulation::Gpu;
	using GpuJob = FluidSimulation::GpuJob;
}

