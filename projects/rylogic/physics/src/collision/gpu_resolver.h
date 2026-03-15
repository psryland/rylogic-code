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
		// 'contacts' are the filtered/prepared contacts with body indices and materials.
		// 'colours' is the per-contact colour assignment.
		// 'max_colour' is the number of colour batches to dispatch.
		// 'bodies_resource' is the GPU-resident RigidBodyDynamics buffer from the integrator.
		void Resolve(GpuJob& job, int body_count, D3DPtr<ID3D12Resource> dispatch, D3DPtr<ID3D12Resource> counters, D3DPtr<ID3D12Resource> contacts, D3DPtr<ID3D12Resource> bodies);

	private:

		// Compile the compute shaders
		void CompileShaders();

		// Resize the GPU buffers to support ???
		void ResizeBuffers(CmdList& cmd_list, int capacity);
	};
}
