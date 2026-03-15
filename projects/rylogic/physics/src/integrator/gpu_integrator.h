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
		ComputeStep m_cs_integrate;          // Root signature + PSO for the integration shader
		D3DPtr<ID3D12Resource> m_r_bodies;   // GPU buffer: RWStructuredBuffer<RigidBodyDynamics>
		D3DPtr<ID3D12Resource> m_r_output;   // GPU buffer: RWStructuredBuffer<IntegrateOutput>
		D3DPtr<ID3D12Resource> m_r_aabb_x;   // GPU buffer: RWStructuredBuffer<float> bounding box x bounds
		D3DPtr<ID3D12Resource> m_r_aabb_y;   // GPU buffer: RWStructuredBuffer<float> bounding box y bounds
		D3DPtr<ID3D12Resource> m_r_aabb_z;   // GPU buffer: RWStructuredBuffer<float> bounding box z bounds
		D3DPtr<ID3D12Resource> m_r_aabb_idx; // GPU buffer: RWStructuredBuffer<int> rigid body indices for the AABB bounds
		int m_capacity;                      // Maximum number of bodies the buffers can hold

		explicit GpuIntegrator(Gpu& gpu);

		// Integrate bodies on GPU and readback AABBs (but keep bodies GPU-resident for later readback).
		void Integrate(GpuJob& job, std::span<RigidBodyDynamics> dynamics, float dt);
		
		// Readback data into the provided buffers. 0-length means "don't readback".
		void Readback(GpuJob& job, std::span<RigidBodyDynamics> dynamics, std::span<IntegrateDebugOutput> output, std::span<BBox> aabbs);

		// Get the GPU resource for the bodies buffer (for GJK to reference directly).
		D3DPtr<ID3D12Resource> BodiesResource() { return m_r_bodies; }

		// Get the GPU resources
		D3DPtr<ID3D12Resource> AABBAxisX() { return m_r_aabb_x; }
		D3DPtr<ID3D12Resource> AABBAxisY() { return m_r_aabb_y; }
		D3DPtr<ID3D12Resource> AABBAxisZ() { return m_r_aabb_z; }
		D3DPtr<ID3D12Resource> AABBBodyIndices() { return m_r_aabb_idx; }

	private:

		// Compile the integration compute shader from embedded resources and create the
		// root signature and pipeline state object.
		void CompileShaders();

		// Resize the buffers to hold 'capacity' bodies.
		void ResizeBuffers(CmdList& cmd_list, int capacity);
	};
}
