//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2025
//*********************************************
#pragma once
#include "pr/physics/forward.h"
#include "src/utility/gpu.h"

namespace pr::physics
{
	struct GpuResolver
	{
		Gpu& m_gpu;                               // Lightweight D3D12 wrapper (device + command queue)
		EngineConfig const& m_config;             // Engine configuration parameters
		BoundsSorter m_contact_sorter;            // Radix sort: key=collision_time (float), payload=contact_index (uint32)
		ComputeStep m_cs_compute_times;           // Parallel: compute collision times per contact
		ComputeStep m_cs_assign_colours;          // Serial: greedy graph colouring on sorted contacts
		ComputeStep m_cs_position_solve;          // Parallel: split position correction in colour batches
		ComputeStep m_cs_resolve;                 // Parallel: resolve contacts in colour batches
		D3DPtr<ID3D12CommandSignature> m_cmd_sig; // Command signature for indirect dispatch
		D3DPtr<ID3D12Resource> m_r_materials;     // GPU buffer: StructuredBuffer<GpuMaterial>
		D3DPtr<ID3D12Resource> m_r_colours;       // GPU buffer: RWStructuredBuffer<uint> per-contact colour assignment
		D3DPtr<ID3D12Resource> m_r_contact_times; // GPU buffer: float keys (collision_time) for radix sort
		D3DPtr<ID3D12Resource> m_r_contact_order; // GPU buffer: uint32 payloads (contact indices) for radix sort
		int m_max_materials;
		int m_max_contacts;

		explicit GpuResolver(Gpu& gpu, EngineConfig const& config);

		// Resolve collisions on the GPU using graph-coloured batches.
		void Resolve(GpuJob& job, float dt, int body_count, int max_contacts, D3DPtr<ID3D12Resource> dispatch, D3DPtr<ID3D12Resource> counters, D3DPtr<ID3D12Resource> contacts, D3DPtr<ID3D12Resource> bodies, std::span<GpuMaterial const> materials, float bias_scale = 1.0f, int solver_iterations = -1, int position_iterations = -1);

		// CPU-side testing: upload contacts and bodies, run graph colouring + resolve on GPU, readback bodies. Calls job.Run() internally.
		void Resolve(GpuJob& job, float dt, std::span<GpuResolveContact const> contacts, std::span<GpuRigidBody> bodies, std::span<GpuMaterial const> materials);

		// Readback bodies after GPU resolve (for CPU-side testing).
		void Readback(GpuJob& job, D3DPtr<ID3D12Resource> r_bodies, std::span<GpuRigidBody> out_bodies);

	private:

		// Compile the compute shaders
		void CompileShaders();

		// Resize the GPU buffers to support ???
		void ResizeBuffers(CmdList& cmd_list, int max_contacts, int max_materials);
	};
}
