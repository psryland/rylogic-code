//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#pragma once
#include "pr/physics/forward.h"
#include "src/utility/gpu.h"

namespace pr::physics
{
	struct GpuSleepManager
	{
		Gpu& m_gpu;                              // Lightweight D3D12 wrapper (device + command queue)
		EngineConfig const& m_config;            // Engine configuration parameters
		ComputeStep m_cs_disturb_islands;        // Root signature + PSO for awake-body vs sleeping-island disturbance
		ComputeStep m_cs_wake_collided;          // Root signature + PSO for clearing Sleeping on collided bodies
		D3DPtr<ID3D12Resource> m_r_sleep_islands;// GPU buffer: RWStructuredBuffer<GpuSleepIsland>
		int m_capacity;                          // Maximum number of islands the buffer can hold

		explicit GpuSleepManager(Gpu& gpu, EngineConfig const& config);

		// Upload staged sleep islands to the GPU.
		void Upload(GpuJob& job, std::span<GpuSleepIsland const> islands);

		// Mark sleeping islands whose world-space bounds are overlapped by actually-awake bodies.
		void SleepWake(GpuJob& job, int body_count, int island_count, D3DPtr<ID3D12Resource> bodies);

		// Persist wake-ups for sleeping bodies that received resolver impulses.
		void SleepUpdate(GpuJob& job, int body_count, D3DPtr<ID3D12Resource> bodies);

		// Get the GPU resources
		D3DPtr<ID3D12Resource> SleepIslands() { return m_r_sleep_islands; }

	private:

		// Compile the sleep/wake compute shaders.
		void CompileShaders();

		// Resize the buffers to support 'capacity' sleep islands.
		void ResizeBuffers(CmdList& cmd_list, int capacity);
	};
}
