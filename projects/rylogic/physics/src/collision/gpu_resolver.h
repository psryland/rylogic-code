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
		Gpu& m_gpu;
		ComputeStep m_cs_colouring;
		ComputeStep m_cs_resolve;
		D3DPtr<ID3D12CommandSignature> m_cmd_sig; // Command signature for indirect dispatch of the collision shader
		D3DPtr<ID3D12Resource> m_r_colours;
		int m_capacity;

		explicit GpuResolver(Gpu& gpu);

		// Resolve collisions on the GPU using graph-coloured batches.
		void Resolve(GpuJob& job, int body_count, D3DPtr<ID3D12Resource> dispatch, D3DPtr<ID3D12Resource> counters, D3DPtr<ID3D12Resource> contacts, D3DPtr<ID3D12Resource> bodies);

		// CPU-side testing: upload contacts and bodies, run graph colouring + resolve on GPU, readback bodies. Calls job.Run() internally.
		void Resolve(GpuJob& job, std::span<GpuResolveContact const> contacts, std::span<RigidBodyDynamics> bodies);

	private:

		// Compile the compute shaders
		void CompileShaders();

		// Resize the GPU buffers to support ???
		void ResizeBuffers(CmdList& cmd_list, int capacity);
	};
}
