//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2016
//*********************************************
#pragma once
#include "pr/physics/forward.h"
#include "src/utility/gpu.h"

namespace pr::physics
{
	struct GpuIntegrator
	{
		Gpu& m_gpu;                          // Lightweight D3D12 wrapper (device + command queue)
		EngineConfig const& m_config;        // Engine configuration parameters
		ComputeStep m_cs_seed_forces;        // Root signature + PSO for restoring frame-constant forces
		ComputeStep m_cs_integrate;          // Root signature + PSO for the integration shader
		D3DPtr<ID3D12Resource> m_r_counters; // GPU buffer: RWStructuredBuffer<GpuCollisionCounters> for storing the number of bodies, pairs, and contacts
		D3DPtr<ID3D12Resource> m_r_bodies;   // GPU buffer: RWStructuredBuffer<GpuRigidBody>
		D3DPtr<ID3D12Resource> m_r_frame_forces; // GPU buffer: StructuredBuffer<GpuFrameForce>
		D3DPtr<ID3D12Resource> m_r_aabb_sort;// GPU buffer: RWStructuredBuffer<float> expanded sort-axis bounds
		D3DPtr<ID3D12Resource> m_r_aabb_idx; // GPU buffer: RWStructuredBuffer<int> rigid body indices for the AABB bounds
		D3DPtr<ID3D12Resource> m_r_aabb_box; // GPU buffer: RWStructuredBuffer<BBox> exact world-space bounding boxes
		D3DPtr<ID3D12Resource> m_r_no_clamps;// GPU buffer: one-element placeholder bound when integration has no drift clamps
		int m_capacity;                      // Maximum number of bodies the buffers can hold
		int m_frame_force_capacity;          // Maximum number of immutable frame-force records retained for internal substeps

		// Optional position limits for one integration (see GpuDriftClampPlane). All members are null or zero when there are none.
		struct DriftClamps
		{
			ID3D12Resource* m_ranges = nullptr; // StructuredBuffer<GpuDriftClampRange>, indexed by body
			ID3D12Resource* m_planes = nullptr; // StructuredBuffer<GpuDriftClampPlane>, 'DriftClampSlotsPerInstance' per instance
			int m_body_count = 0;               // Bodies [0, m_body_count) have entries in 'm_ranges'
			uint32_t m_modes = 0;               // Bit 'm' set when the slots of world-contact mode 'm' are current
		};

		explicit GpuIntegrator(Gpu& gpu, EngineConfig const& config);

		// Upload staged body dynamics and optionally retain immutable forces for later internal substeps.
		void Upload(GpuJob& job, std::span<GpuRigidBody> bodies, bool retain_frame_forces = false);

		// Reset the collision counters without changing the body buffer.
		void ResetCounters(GpuJob& job);

		// Restore frame-constant forces before one internal substep.
		void SeedWorkingForces(GpuJob& job, int body_count);

		// Integrate bodies on GPU and write AABBs (but keep bodies GPU-resident for later readback).
		// 'clamps' limits how far the drift may move bodies; it never changes momentum.
		void Integrate(GpuJob& job, int body_count, float dt, int broadphase_sort_axis, DriftClamps const& clamps);

		// CPU-side testing: upload bodies, integrate on GPU, readback results. Calls job.Run() internally.
		void Integrate(GpuJob& job, std::span<GpuRigidBody> bodies, float dt, std::span<BBox> aabbs);

		// Readback data into the provided buffers. 0-length means "don't readback".
		void Readback(GpuJob& job, std::span<GpuRigidBody> bodies, std::span<BBox> aabbs);

		// Get the GPU resources
		D3DPtr<ID3D12Resource> Counters() { return m_r_counters; }
		D3DPtr<ID3D12Resource> Bodies() { return m_r_bodies; }
		D3DPtr<ID3D12Resource> AABBSortAxis() { return m_r_aabb_sort; }
		D3DPtr<ID3D12Resource> AABBBodyIndices() { return m_r_aabb_idx; }
		D3DPtr<ID3D12Resource> AABBBoxes() { return m_r_aabb_box; }

	private:

		// Create the optional force-restoration pipeline when a frame first requests multiple internal substeps.
		void EnsureSeedForcesPipeline();

		// Resize the core buffers and optional immutable force storage to hold 'capacity' bodies.
		void ResizeBuffers(GpuJob& job, int capacity, bool retain_frame_forces);
	};
}
