//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#pragma once
#include "pr/physics/integrator/engine_water.h"
#include "pr/physics/rigid_body/rigid_body.h"
#include "src/utility/gpu.h"

namespace pr::physics
{
	// Analytic buoyancy and drag for rigid bodies that may touch the water during a frame.
	// The host selects candidate bodies once per frame. Each internal substep then evaluates the candidates on the GPU from their current poses,
	// so the forces follow the body between substeps. Frames without candidates record no GPU work.
	// Each candidate samples the water depth once per frame at its proxy centre; wave amplitudes are corrected for that depth (see water_depth.hlsli).
	struct GpuWaterForces
	{
		// One candidate's volume proxy in model space. Must match 'GpuWaterCandidate' in gpu_water_forces.hlsl.
		struct Candidate
		{
			int m_body_index;
			int m_proxy;
			float m_volume;
			float m_depth;
			v4 m_centre_os;
			v4 m_extent_os;
			v4 m_axis_x_os;
			v4 m_axis_y_os;
			v4 m_axis_z_os;
		};

		// Values in 'Candidate::m_proxy'.
		static constexpr int ProxySphere = 0;
		static constexpr int ProxyBox = 1;

		Gpu& m_gpu;
		WaterConfig m_config;
		ComputeStep m_step;
		std::vector<Candidate> m_candidates;
		D3D12_GPU_VIRTUAL_ADDRESS m_candidates_va;
		D3D12_GPU_VIRTUAL_ADDRESS m_elements_va;

		// Compile the water-force pipeline for 'config'.
		GpuWaterForces(Gpu& gpu, WaterConfig config);

		// Select the bodies that may be wet during a frame of 'elapsed_s' seconds and stage their proxies for every substep.
		// 'bodies' are in GPU body order. Static and sleeping bodies are never candidates.
		void BeginFrame(GpuJob& job, std::span<RigidBody* const> bodies, float elapsed_s);

		// Wake sleeping bodies that cross the band of heights the moving surface can reach under them. Does nothing for a flat field.
		void WakeInSurfaceBand(std::span<RigidBody* const> bodies) const;

		// Record one substep's force evaluation. Does nothing when the frame has no candidates.
		void Apply(GpuJob& job, ID3D12Resource* bodies, float dt, double time_s);

		// Return the volume proxy for 'body', or false when its shape has no volume.
		static bool MakeCandidate(RigidBody const& body, int body_index, Candidate& candidate);

		// Return true when any point of 'body' might be below the surface of 'field' during the next 'elapsed_s' seconds.
		static bool MayBeWet(RigidBody const& body, terrain::water::WaterField const& field, float elapsed_s);
	};
}
