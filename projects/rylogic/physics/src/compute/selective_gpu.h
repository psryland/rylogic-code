//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#pragma once
#include "pr/physics/forward.h"
#include "src/utility/gpu.h"

namespace pr::physics
{
	struct GpuSelectiveWorkSet
	{
		D3DPtr<ID3D12Resource> m_counters;
		D3DPtr<ID3D12Resource> m_pairs;
		D3DPtr<ID3D12Resource> m_cd_dispatch;
		D3DPtr<ID3D12Resource> m_contacts;
		D3DPtr<ID3D12Resource> m_resolve_dispatch;
		int m_max_pairs = 0;
		int m_max_contacts = 0;
	};

	struct GpuSelectiveRefresher
	{
		Gpu& m_gpu;                              // Lightweight D3D12 wrapper (device + command queue)
		EngineConfig const& m_config;            // Engine configuration parameters
		ComputeStep m_cs_prepare;                // Clears per-pass problem flags and destination counters
		ComputeStep m_cs_score_contacts;         // Scores resolved contacts and marks bodies that need refresh
		ComputeStep m_cs_compact_pairs;          // Compacts current broadphase pairs touching marked bodies
		ComputeStep m_cs_build_dispatch;         // Builds indirect collision-dispatch args for the compacted pair set
		D3DPtr<ID3D12CommandSignature> m_cmd_sig;
		D3DPtr<ID3D12Resource> m_r_problem_bodies;
		D3DPtr<ID3D12Resource> m_r_metrics;
		std::array<GpuSelectiveWorkSet, 2> m_work_sets;
		int m_body_capacity;
		int m_max_pairs;
		int m_max_contacts;

		explicit GpuSelectiveRefresher(Gpu& gpu, EngineConfig const& config);

		// Build the compact pair work set for one selective refresh pass.
		GpuSelectiveWorkSet& BuildWorkSet(
			GpuJob& job,
			int pass_index,
			int body_count,
			int max_pairs,
			int max_contacts,
			int source_max_contacts,
			int full_max_pairs,
			D3DPtr<ID3D12Resource> source_counters,
			D3DPtr<ID3D12Resource> source_contacts,
			D3DPtr<ID3D12Resource> source_contact_dispatch,
			D3DPtr<ID3D12Resource> full_pair_dispatch,
			D3DPtr<ID3D12Resource> full_counters,
			D3DPtr<ID3D12Resource> full_pairs,
			D3DPtr<ID3D12Resource> bodies);

	private:

		// Resize scratch buffers.
		void ResizeBuffers(CmdList& cmd_list, int body_count, int max_pairs, int max_contacts);
	};
}
