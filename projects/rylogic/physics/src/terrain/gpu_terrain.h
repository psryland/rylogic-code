//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#pragma once
#include "pr/physics/terrain/landscape/baseline_surface.h"
#include "pr/physics/surface/surface_sampling.h"
#include "pr/physics/rigid_body/rigid_body.h"
#include "src/utility/gpu.h"
#include "src/compute/physics_types.h"

namespace pr::physics
{
	// Immutable terrain and cached primitive plans; contacts join the ordinary solver through an owned static endpoint.
	struct GpuTerrain
	{
		// Patch span and total sample count for one packed primitive shape.
		struct Range
		{
			uint32_t m_begin, m_patch_count, m_sample_count, m_pad;
		};

		// Packed body and primitive indices, with the child identity retained for contact matching.
		struct Instance
		{
			uint32_t m_body, m_shape, m_child, m_pad;
		};

		Gpu& m_gpu;
		terrain::landscape::BaselineSurface m_surface;
		float m_spacing;
		float m_height_upper;
		RigidBody m_endpoint;
		ComputeStep m_step;
		ComputeStep m_dispatch;
		std::vector<surface::SurfacePatch> m_patches;
		std::vector<Range> m_ranges;
		std::vector<Instance> m_instances;
		std::vector<Shape const*> m_sources;
		std::vector<bool> m_planned;
		size_t m_shape_count = 0;
		D3DPtr<ID3D12Resource> m_recipe, m_plans, m_patch_buffer, m_instance_buffer, m_status;
		D3DPtr<ID3D12QueryHeap> m_queries;
		D3DPtr<ID3D12Resource> m_query_readback;
		uint32_t m_query_capacity = 0, m_query_count = 0;
		uint64_t m_frequency = 0;

		// Validate and own a source, its conservative height bound, and the contact pipeline.
		GpuTerrain(Gpu& gpu, terrain::landscape::BaselineSurface surface, float spacing);

		// Invalidate shape-indexed plans after the engine has retired their GPU work.
		void Reset();

		// Prepare immutable plans on first dynamic use and upload this frame's participating instances.
		void Upload(GpuJob& job, ShapeCache const& shapes, std::span<GpuRigidBody const> bodies, int substeps);

		// Append terrain constraints to the ordinary contact stream and update its indirect dispatch.
		void Collide(GpuJob& job, int endpoint, int max_contacts, ID3D12Resource* bodies, ID3D12Resource* shapes, ID3D12Resource* contacts, ID3D12Resource* counters, ID3D12Resource* dispatch);

		// Record status and timestamp readbacks without submitting or waiting.
		ReadbackAlloc Readback(GpuJob& job);

		// Return terrain queue time only after this frame's submitted work has completed.
		double GpuTimeMs() const;
	};
}
