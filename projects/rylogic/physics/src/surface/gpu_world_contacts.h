//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#pragma once
#include "pr/physics/terrain/landscape/baseline_surface.h"
#include "pr/physics/surface/surface_sampling.h"
#include "pr/physics/surface/cylindrical_boundary.h"
#include "pr/physics/rigid_body/rigid_body.h"
#include "src/utility/gpu.h"
#include "src/compute/physics_types.h"

namespace pr::physics
{
	// Sampled world surfaces share primitive plans and one owned static solver endpoint.
	struct GpuWorldContacts
	{
		// Status flags followed by the first rejected motion's body, substep, phase, and scalar components.
		static constexpr int StatusWordCount = 20;
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
		std::optional<terrain::landscape::BaselineSurface> m_surface;
		std::optional<CylindricalBoundaryConfig> m_boundary;
		float m_terrain_spacing;
		float m_spacing;
		uint32_t m_boundary_plan_offset = 0;
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
		D3DPtr<ID3D12Resource> m_previous_bodies;
		uint32_t m_body_count = 0;
		float m_dt = 0;
		uint32_t m_substep = 0, m_phase = 0;

		// Validate and own independent world sources and their shared contact pipeline.
		GpuWorldContacts(Gpu& gpu, std::optional<terrain::landscape::BaselineSurface> surface, float spacing, std::optional<CylindricalBoundaryConfig> boundary = {});

		// Invalidate shape-indexed plans after the engine has retired their GPU work.
		void Reset();

		// Prepare immutable plans on first dynamic use and upload this frame's participating instances.
		void Upload(GpuJob& job, ShapeCache const& shapes, std::span<GpuRigidBody const> bodies, int substeps);

		// Append world constraints for awake or disturbed bodies and update the ordinary contact stream's indirect dispatch.
		void Collide(GpuJob& job, int endpoint, int max_contacts, bool sleeping_enabled, int island_count, ID3D12Resource* sleep_islands,
			ID3D12Resource* bodies, ID3D12Resource* shapes, ID3D12Resource* contacts, ID3D12Resource* counters, ID3D12Resource* dispatch);

		// Record status and timestamp readbacks without submitting or waiting.
		ReadbackAlloc Readback(GpuJob& job);

		// Return terrain queue time only after this frame's submitted work has completed.
		double GpuTimeMs() const;

		// Check the boundary envelope without emitting contacts; retain the completed pose for the next substep when requested.
		void ValidateBoundary(GpuJob& job, ID3D12Resource* bodies, ID3D12Resource* shapes, float dt, bool advance);

		// Record one surface or validation dispatch using the shared sampled streams.
		void Dispatch(GpuJob& job, int mode, int endpoint, int max_contacts, bool sleeping_enabled, int island_count, ID3D12Resource* sleep_islands,
			ID3D12Resource* bodies, ID3D12Resource* shapes, ID3D12Resource* contacts, ID3D12Resource* counters, ID3D12Resource* dispatch);
	};
}
