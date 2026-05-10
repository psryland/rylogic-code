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
		ComputeStep m_cs_init_sleep_state;       // Root signature + PSO for initialising body-parent and per-root stats scratch buffers
		ComputeStep m_cs_union_sleep_contacts;   // Root signature + PSO for unioning contacted dynamic bodies into components
		ComputeStep m_cs_canonicalise_roots;     // Root signature + PSO for compressing component parent links
		ComputeStep m_cs_mark_hit_islands;       // Root signature + PSO for marking persisted sleeping islands hit this frame
		ComputeStep m_cs_reduce_sleep_stats;     // Root signature + PSO for reducing per-body sleep state into component stats
		ComputeStep m_cs_apply_sleep_state;      // Root signature + PSO for applying component sleep/wake decisions to bodies
		D3DPtr<ID3D12CommandSignature> m_cmd_sig;// Command signature for indirect contact-count dispatches
		D3DPtr<ID3D12Resource> m_r_sleep_islands;// GPU buffer: RWStructuredBuffer<GpuSleepIsland>
		D3DPtr<ID3D12Resource> m_r_sleep_parents;// GPU buffer: RWStructuredBuffer<int> for union-find parent links
		D3DPtr<ID3D12Resource> m_r_sleep_stats;  // GPU buffer: RWStructuredBuffer<GpuSleepIslandStats>
		int m_island_capacity;                   // Maximum number of islands the island buffer can hold
		int m_body_capacity;                     // Maximum number of bodies the scratch buffers can hold

		explicit GpuSleepManager(Gpu& gpu, EngineConfig const& config);

		// Upload staged sleep islands to the GPU.
		void Upload(GpuJob& job, std::span<GpuSleepIsland const> islands);

		// Mark sleeping islands whose world-space bounds are overlapped by actually-awake bodies.
		void SleepWake(GpuJob& job, int body_count, int island_count, D3DPtr<ID3D12Resource> bodies);

		// Persist wake-ups and update automatic sleeping from the resolved contact graph.
		void SleepUpdate(GpuJob& job, float dt, int body_count, int island_count, int max_contacts, D3DPtr<ID3D12Resource> counters, D3DPtr<ID3D12Resource> contacts, D3DPtr<ID3D12Resource> contact_dispatch, D3DPtr<ID3D12Resource> bodies);

		// Get the GPU resources
		D3DPtr<ID3D12Resource> SleepIslands() { return m_r_sleep_islands; }

	private:

		// Compile the sleep/wake compute shaders.
		void CompileShaders();

		// Resize the buffers to support 'capacity' sleep islands.
		void ResizeIslandBuffers(CmdList& cmd_list, int capacity);

		// Resize scratch buffers to support 'capacity' bodies.
		void ResizeBodyBuffers(CmdList& cmd_list, int capacity);
	};
}
