//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#pragma once
#include "src/constraint/constraint_compiler.h"
#include "src/compute/physics_types.h"

namespace pr::physics
{
	// Sparse stable-slot streams used to upload persistent D6 descriptors and frame-local endpoint remaps.
	struct GpuConstraintUpload
	{
		std::vector<GpuConstraintEndpoint> m_endpoints;
		std::vector<GpuD6ConstraintDesc> m_descriptors;
		size_t m_active_count = 0;
		uint64_t m_topology_revision = 0;
		uint64_t m_parameter_revision = 0;
	};

	// Pack persistent descriptors and resolve enabled endpoints into current frame-local rigid-body indices.
	GpuConstraintUpload PackGpuConstraints(ConstraintSet const& constraints, BodyRemap const& remap);
}
