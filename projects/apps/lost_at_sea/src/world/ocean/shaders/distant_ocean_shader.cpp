//************************************
// Lost at Sea
//  Copyright (c) Rylogic Ltd 2025
//************************************
#include "src/forward.h"
#include "src/world/ocean/shaders/distant_ocean_shader.h"
#include "src/world/ocean/shaders/distant_ocean_cbuf.hlsli"

namespace las
{
	enum class ERootParam : int
	{
		CBufScene = 0,
		CBufObject = 1,
		CBufFrame = 2,
		CBufDistantOcean = 3,
	};

	DistantOceanShader::DistantOceanShader(Renderer& rdr)
		: Shader(rdr)
		, m_vs_bytecode()
		, m_ps_bytecode()
		, m_cbuf()
	{
		static_assert((sizeof(CBufDistantOcean) % 16) == 0);
		static_assert(sizeof(CBufDistantOcean) == sizeof(m_cbuf), "CBufDistantOcean exceeds m_cbuf storage");
		auto resolver = ::pr::compute::shader_cache::ResourceSourceResolver{};

		auto compiler = ShaderCompiler{}
			.Source("src/world/ocean/shaders/distant_ocean.hlsl", resolver)
			.HlslVersion(EHlslVersion::Hlsl2021)
			.Define(L"SHADER_BUILD")
			.Optimise(true);

		m_vs_bytecode = compiler.ShaderModel(L"vs_6_0").EntryPoint(L"VSDistantOcean").Compile();
		m_ps_bytecode = compiler.ShaderModel(L"ps_6_0").EntryPoint(L"PSDistantOcean").Compile();
		m_code.VS = { m_vs_bytecode };
		m_code.PS = { m_ps_bytecode };

		// Default parameters
		auto& cbuf = storage_cast<CBufDistantOcean>(m_cbuf);
		cbuf = CBufDistantOcean{
			.camera_pos = v4::Zero(),
			.fog_params = v4(2000.0f, 5000.0f, 0, 0),
			.colour_shallow = v4(0.10f, 0.60f, 0.55f, 1.0f),
			.colour_deep = v4(0.02f, 0.08f, 0.20f, 1.0f),
			.fog_colour = v4(0.70f, 0.80f, 0.90f, 1.0f),
			.sun_direction = Normalise(v4(0.5f, 0.3f, 0.8f, 0.0f)),
			.sun_colour = v4(1.0f, 0.95f, 0.85f, 1.0f),
		};
	}

	void DistantOceanShader::SetupElement(ID3D12GraphicsCommandList* cmd_list, ::pr::compute::GpuUploadBuffer& upload, rdr12::Scene const&, rdr12::DrawListElement const* dle)
	{
		if (dle == nullptr)
			return;

		auto& cbuf = storage_cast<CBufDistantOcean>(m_cbuf);
		auto gpu_address = upload.Add(cbuf, D3D12_CONSTANT_BUFFER_DATA_PLACEMENT_ALIGNMENT, true);
		cmd_list->SetGraphicsRootConstantBufferView(static_cast<UINT>(ERootParam::CBufDistantOcean), gpu_address);
	}

	void DistantOceanShader::SetupFrame(v4 camera_world_pos, bool has_env_map, v4 sun_direction, v4 sun_colour)
	{
		auto& cbuf = storage_cast<CBufDistantOcean>(m_cbuf);
		cbuf.camera_pos = camera_world_pos;
		cbuf.has_env_map = has_env_map ? 1 : 0;
		cbuf.sun_direction = sun_direction;
		cbuf.sun_colour = sun_colour;
	}
}
