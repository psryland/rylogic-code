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
	// The compiled sampled world-contact kernels. They do not depend on the world sources, so one pipeline serves every set of sources on a device.
	struct GpuWorldContactPipeline
	{
		ComputeStep m_samples;
		ComputeStep m_step;
		ComputeStep m_dispatch;

		// Compile the kernels and create their pipelines. 'shader_cache' is optional; when given, compiled kernels are reused across runs.
		// Throws if the device lacks the FP64 and Int64 shader operations the kernels need.
		GpuWorldContactPipeline(Gpu& gpu, IShaderCache* shader_cache);
	};

	// Sampled world surfaces share primitive plans and one owned static solver endpoint.
	struct GpuWorldContacts
	{
		// Failure flags, first cause, body/leaf indices, sample ordinal, and source-specific detail.
		static constexpr int StatusWordCount = 6;

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

		// Stored sample result; matches WorldCandidate in gpu_world_contacts.hlsl.
		struct Candidate
		{
			float m_position[4];
			float m_normal_depth[4];
			uint32_t m_valid;
		};

		// A cached primitive plan and the last full replan that used it.
		struct CachedPlan
		{
			surface::Plan m_plan;
			uint64_t m_generation;
		};

		Gpu& m_gpu;
		GpuWorldContactPipeline& m_pipeline;
		std::optional<terrain::landscape::BaselineSurface> m_surface;
		std::optional<CylindricalBoundaryConfig> m_boundary;
		float m_terrain_spacing;
		float m_spacing;
		uint32_t m_boundary_plan_offset = 0;
		float m_height_upper;
		RigidBody m_endpoint;
		std::vector<surface::SurfacePatch> m_patches;
		std::vector<Range> m_ranges;
		std::vector<Instance> m_instances;
		std::vector<GpuDriftClampRange> m_clamp_ranges; // Instance range of every uploaded body, indexed by body. Instances are pushed in body order.
		std::vector<Shape const*> m_sources;
		std::vector<bool> m_planned;
		std::unordered_map<std::string, CachedPlan> m_plan_cache; // Primitive plans keyed by plan-relevant shape content and spacing (see PlanFor).
		uint64_t m_plan_generation = 0;                            // Incremented on every full replan; entries not used by the latest one are dropped.
		size_t m_shape_count = 0;
		uint32_t m_max_samples[2] = {}; // Largest per-instance sample count for terrain (0) and cylinder (1) plans in this frame.
		terrain::landscape::shared::BaselineLocalRecipe m_local_recipe_host; // Single-precision terrain recipe, prepared at construction and uploaded on first use.
		D3DPtr<ID3D12Resource> m_recipe, m_local_recipe, m_plans, m_patch_buffer, m_instance_buffer, m_status, m_candidates;
		D3DPtr<ID3D12Resource> m_clamp_range_buffer; // StructuredBuffer<GpuDriftClampRange>, one per uploaded body.
		D3DPtr<ID3D12Resource> m_clamps;             // RWStructuredBuffer<GpuDriftClampPlane>, 'DriftClampSlotsPerInstance' per instance.
		D3DPtr<ID3D12QueryHeap> m_queries;
		D3DPtr<ID3D12Resource> m_query_readback;
		uint32_t m_query_capacity = 0, m_query_count = 0;
		uint64_t m_frequency = 0;

		// Validate and own independent world sources. 'pipeline' must outlive this object.
		// Terrain is sampled in single precision over the surface's whole supported range, so construction throws out_of_range when a terrain octave's
		// lattice cells over that range do not fit in 30 bits (see BaselineSurface::LocalRecipe). Reduce the supported range or lengthen the shortest wavelength.
		GpuWorldContacts(Gpu& gpu, GpuWorldContactPipeline& pipeline, std::optional<terrain::landscape::BaselineSurface> surface, float spacing, std::optional<CylindricalBoundaryConfig> boundary = {});

		// Invalidate shape-indexed plans after the engine has retired their GPU work.
		void Reset();

		// Prepare immutable plans on first dynamic use and upload this frame's participating instances.
		// A body takes part only when its collision filter passes against the world's 'world_layer' and 'world_mask' (see CollisionFilterPasses).
		void Upload(GpuJob& job, ShapeCache const& shapes, std::span<GpuRigidBody const> bodies, int substeps, uint32_t world_layer, uint32_t world_mask);

		// Append world constraints for awake or disturbed bodies and update the ordinary contact stream's indirect dispatch.
		// Also replace every instance's drift clamps (see DriftClamps). 'dt' is the duration (seconds) of the next integration drift;
		// clear samples that can cross the surface within it produce drift clamps. Zero 'dt' clears every drift clamp.
		void Collide(GpuJob& job, float dt, int endpoint, int max_contacts, bool sleeping_enabled, int island_count, ID3D12Resource* sleep_islands,
			ID3D12Resource* bodies, ID3D12Resource* shapes, ID3D12Resource* contacts, ID3D12Resource* counters, ID3D12Resource* dispatch);

		// Replace every instance's drift clamps for an integration drift of 'dt' seconds from the current poses, without appending contacts.
		// Use this when the next integration is not preceded by Collide, such as the first substep of a frame.
		void RefreshDriftClamps(GpuJob& job, float dt, int endpoint, int max_contacts, bool sleeping_enabled, int island_count, ID3D12Resource* sleep_islands,
			ID3D12Resource* bodies, ID3D12Resource* shapes, ID3D12Resource* contacts, ID3D12Resource* counters);

		// Drift-clamp planes and per-body instance ranges for integration. Valid after Upload and a later Collide or RefreshDriftClamps
		// in the same frame; the ranges cover the first 'DriftClampBodyCount' bodies.
		ID3D12Resource* DriftClamps();
		ID3D12Resource* DriftClampRanges();
		int DriftClampBodyCount() const;

		// Bit 0 is set when the terrain writes drift clamps and bit 1 when the cylinder does. Integration must ignore the slots of other modes.
		uint32_t DriftClampModes() const;

		// Record status and timestamp readbacks without submitting or waiting.
		ReadbackAlloc Readback(GpuJob& job);

		// Return terrain queue time only after this frame's submitted work has completed.
		double GpuTimeMs() const;

		// Return the plan for a primitive leaf at 'spacing', building it only when no leaf with the same geometry has one.
		surface::Plan const& PlanFor(Shape const& shape, float spacing);

		// Record one surface-contact dispatch using the current poses and cached sampled streams. A null 'dispatch' refreshes only the drift clamps.
		void Dispatch(GpuJob& job, int mode, float dt, int endpoint, int max_contacts, bool sleeping_enabled, int island_count, ID3D12Resource* sleep_islands,
			ID3D12Resource* bodies, ID3D12Resource* shapes, ID3D12Resource* contacts, ID3D12Resource* counters, ID3D12Resource* dispatch);
	};
}
