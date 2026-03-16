//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// Forward declaration for GpuCollisionDetector (pimpl pattern).
// The full definition lives in gpu_collision_detector.cpp and includes view3d-12 headers.
// This header is safe to include from any physics-2 public header.
#pragma once
#include "pr/physics-2/forward.h"
#include "src/utility/gpu.h"

namespace pr::physics
{
	struct GpuCollisionDetector
	{
		Gpu& m_gpu;                          // Lightweight D3D12 wrapper (device + command queue)
		ComputeStep m_cs_gjk;                // Root signature + PSO for the GJK shader
		D3DPtr<ID3D12Resource> m_r_shapes;   // GPU buffer: StructuredBuffer<GpuShape>
		D3DPtr<ID3D12Resource> m_r_verts;    // GPU buffer: StructuredBuffer<float4>
		D3DPtr<ID3D12Resource> m_r_pairs;    // GPU buffer: StructuredBuffer<GpuCollisionPair>
		D3DPtr<ID3D12Resource> m_r_contacts; // GPU buffer: RWStructuredBuffer<GpuContact>
		D3DPtr<ID3D12Resource> m_r_counters; // GPU buffer: RWStructuredBuffer<uint>
		int m_max_shapes;
		int m_max_verts;
		int m_max_pairs;

		explicit GpuCollisionDetector(Gpu& gpu);

		// Run collision detection on the GPU.
		// Uploads shapes, pairs, and vertices → dispatches GJK shader → reads back contacts.
		// When 'shapes_dirty' is false, the shapes and verts buffers are already on the GPU
		// from a previous frame and the upload is skipped.
		int DetectCollisions(
			GpuJob& job,
			std::span<GpuCollisionPair const> pairs,
			std::span<GpuShape const> shapes,
			std::span<v4 const> verts,
			std::vector<GpuContact>& out_contacts,
			bool shapes_dirty = true);

	private:

		// Compile the GJK compute shader from embedded resources.
		void CompileShader();

		// Create GPU buffers for collision pipeline.
		// Returns true if shape or vertex buffers were reallocated (requiring re-upload).
		bool ResizeBuffers(CmdList& cmd_list, int max_shapes, int max_verts, int max_pairs);
	};
}
