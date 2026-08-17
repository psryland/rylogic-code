//************************************
// Lost at Sea
//  Copyright (c) Rylogic Ltd 2024
//************************************
// Custom ocean shader override: VS for Gerstner wave displacement,
// PS for PBR water rendering (Fresnel, reflection, refraction, SSS, foam).
#pragma once
#include "src/forward.h"
#include "src/world/water/water_system.h"
#include "src/world/ocean/shaders/ocean_cbuf.hlsli"

namespace las
{
	struct OceanShader : rdr12::Shader
	{
		// Compiled shader bytecodes (populated at construction from runtime compilation).
		// The ByteCode wrappers in m_code point into these vectors, so they must outlive the shader.
		std::vector<uint8_t> m_vs_bytecode;
		std::vector<uint8_t> m_ps_bytecode;

		// Ocean constant buffer data, updated each frame
		alignas(16) CBufOcean m_cbuf;

		explicit OceanShader(Renderer& rdr);

		// Called per-nugget during forward rendering to bind the ocean constant buffer
		void SetupElement(ID3D12GraphicsCommandList* cmd_list, ::pr::compute::GpuUploadBuffer& upload, rdr12::Scene const& scene, rdr12::DrawListElement const* dle) override;

		// Copy one immutable water-field snapshot and update render-only frame parameters.
		void SetupFrame(water::Snapshot const& water_snapshot, v4 camera_world_pos, float outer_radius, int grid_vertex_count, float min_grid_spacing, float surface_warp_power, bool has_env_map, v4 sun_direction, v4 sun_colour);
	};
}
