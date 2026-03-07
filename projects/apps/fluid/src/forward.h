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
#include "pr/math/math.h"
#include "pr/container/vector.h"
#include "pr/container/kdtree.h"
#include "pr/camera/camera.h"
#include "pr/gui/wingui.h"
#include "pr/win32/windows_com.h"

#include "pr/view3d-12/view3d.h"
#include "pr/view3d-12/ldraw/ldraw_object.h"
#include "pr/view3d-12/ldraw/ldraw_parsing.h"
#include "pr/view3d-12/ldraw/ldraw_builder.h"
#include "pr/view3d-12/compute/gpu_job.h"
#include "pr/view3d-12/compute/radix_sort/radix_sort.h"
#include "pr/view3d-12/compute/spatial_partition/spatial_partition.h"
#include "pr/view3d-12/compute/particle_collision/particle_collision.h"
#include "pr/view3d-12/compute/fluid_simulation/fluid_simulation.h"
#include "pr/view3d-12/utility/pix.h"

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

	using ComputeStep = rdr12::ComputeStep;
	using FluidSimulation = rdr12::compute::fluid::FluidSimulation<>;
	using SpatialPartition = rdr12::compute::spatial_partition::SpatialPartition;
	using ParticleCollision = rdr12::compute::particle_collision::ParticleCollision;
	using CollisionBuilder = rdr12::compute::particle_collision::CollisionBuilder;
	using CollisionPrim = rdr12::compute::particle_collision::Prim;
	using Particle = rdr12::compute::fluid::Particle;
	using Dynamics = rdr12::compute::fluid::Dynamics;
	using GpuJob = FluidSimulation::GpuJob;
}

