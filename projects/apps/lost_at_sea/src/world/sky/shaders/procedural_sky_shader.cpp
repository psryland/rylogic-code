//************************************
// Lost at Sea
//  Copyright (c) Rylogic Ltd 2025
//************************************
#include "src/forward.h"
#include "src/world/sky/shaders/procedural_sky_shader.h"

namespace las
{
	namespace shaders::procedural_sky
	{
		enum class ERootParam : int
		{
			CBufScene = 0,
			CBufObject = 1,
			CBufFrame = 2,
			CBufProceduralSky = 3,
		};

		using namespace pr::hlsl;
		#include "src/world/sky/shaders/procedural_sky_cbuf.hlsli"
		static_assert((sizeof(CBufProceduralSky) % 16) == 0);
	}

	ProceduralSkyShader::ProceduralSkyShader(Renderer& rdr)
		: Shader(rdr)
		, m_vs_bytecode()
		, m_ps_bytecode()
		, m_cbuf()
	{
		static_assert(sizeof(shaders::procedural_sky::CBufProceduralSky) == sizeof(m_cbuf),
			"CBufProceduralSky exceeds m_cbuf storage");

		auto compiler = ShaderCompiler{}
			.Source(resource::Read<char>(L"PROCEDURAL_SKY_HLSL", L"TEXT"))
			.Includes({ new rdr12::ResourceIncludeHandler, true })
			.Define(L"SHADER_BUILD")
			.Optimise(true);

		m_vs_bytecode = compiler.ShaderModel(L"vs_6_0").EntryPoint(L"VSProceduralSky").Compile();
		m_ps_bytecode = compiler.ShaderModel(L"ps_6_0").EntryPoint(L"PSProceduralSky").Compile();
		m_code.VS = { m_vs_bytecode };
		m_code.PS = { m_ps_bytecode };

		// Default: noon sun
		auto& cbuf = storage_cast<shaders::procedural_sky::CBufProceduralSky>(m_cbuf);
		cbuf = shaders::procedural_sky::CBufProceduralSky{
			.m_sun_direction = Normalise(v4(0.5f, 0.3f, 0.8f, 0.0f)),
			.m_sun_colour = v4(1.0f, 0.95f, 0.85f, 1.0f),
			.m_sun_intensity = 1.0f,
		};
	}

	void ProceduralSkyShader::SetupElement(ID3D12GraphicsCommandList* cmd_list, rdr12::GpuUploadBuffer& upload, rdr12::Scene const&, rdr12::DrawListElement const* dle)
	{
		if (dle == nullptr)
			return;

		auto& cbuf = storage_cast<shaders::procedural_sky::CBufProceduralSky>(m_cbuf);
		auto gpu_address = upload.Add(cbuf, D3D12_CONSTANT_BUFFER_DATA_PLACEMENT_ALIGNMENT, true);
		cmd_list->SetGraphicsRootConstantBufferView(
			static_cast<UINT>(shaders::procedural_sky::ERootParam::CBufProceduralSky), gpu_address);
	}

	void ProceduralSkyShader::SetupFrame(v4 sun_direction, v4 sun_colour, float sun_intensity)
	{
		auto& cbuf = storage_cast<shaders::procedural_sky::CBufProceduralSky>(m_cbuf);
		cbuf.m_sun_direction = sun_direction;
		cbuf.m_sun_colour = sun_colour;
		cbuf.m_sun_intensity = sun_intensity;
	}
}
