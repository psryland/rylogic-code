//************************************
// Physics Sandbox
//  Copyright (c) Rylogic Ltd 2026
//************************************
#include "src/scene/water/water_visual.h"
#include "src/scene/water/water_visual_cbuf.hlsli"

namespace physics_sandbox
{
	namespace
	{
		enum class ERootParam : int
		{
			CBufFrame = 0,
			CBufNugget = 1,
			CBufWater = 3,
		};

		// Vertex-shader overlay that keeps immutable wave parameters and takes time from the water instance.
		struct WaterShader : rdr12::Shader
		{
			std::vector<uint8_t> m_vs_bytecode;
			CBufWaterVisual m_cbuf;

			// Compile the sandbox-local shader and pack the normalised physics surface once.
			WaterShader(rdr12::Renderer& rdr, physics::GpuBuoyancy::WaterSurface const& surface)
				: Shader(rdr)
				, m_vs_bytecode()
				, m_cbuf()
			{
				static_assert(MaxWaterWaveCount == physics::GpuBuoyancy::MaxWaterWaveCount);
				static_assert((sizeof(CBufWaterVisual) % 16) == 0);

				m_cbuf.m_wave_count = isize(surface.m_waves);
				m_cbuf.m_water_level = surface.m_level;
				for (int wave_index = 0; wave_index != m_cbuf.m_wave_count; ++wave_index)
				{
					auto const& wave = surface.m_waves[wave_index];
					m_cbuf.m_waves[wave_index] = WaterVisualWave{
						.m_direction_wavelength_phase_speed = v4(wave.m_direction.x, wave.m_direction.y, wave.m_wavelength, wave.m_phase_speed),
						.m_amplitude = v4(wave.m_amplitude, 0.0f, 0.0f, 0.0f),
					};
				}

				auto resolver = ::pr::compute::shader_cache::ResourceSourceResolver{};
				auto compiler = ::pr::compute::ShaderCompiler{}
					.Source("src/scene/water/water_visual.hlsl", resolver)
					.HlslVersion(::pr::compute::EHlslVersion::Hlsl2021)
					.Define(L"SHADER_BUILD")
					.Optimise(true);

				m_vs_bytecode = compiler.ShaderModel(L"vs_6_0").EntryPoint(L"VSWater").Compile();
				m_code.VS = { m_vs_bytecode };
			}

			// Bind one per-draw constant buffer; only the simulation time changes after scene loading.
			void SetupElement(ID3D12GraphicsCommandList* cmd_list, GpuUploadBuffer& upload, rdr12::Scene const&, rdr12::DrawListElement const* dle) override
			{
				if (dle == nullptr || dle->m_instance == nullptr)
					throw std::runtime_error("Water shader requires a water visual instance");

				auto cbuf = m_cbuf;
				cbuf.m_time_s = dle->m_instance->get<float>(rdr12::EInstComp::Float1);
				auto gpu_address = upload.Add(cbuf, D3D12_CONSTANT_BUFFER_DATA_PLACEMENT_ALIGNMENT, true);
				cmd_list->SetGraphicsRootConstantBufferView(static_cast<UINT>(ERootParam::CBufWater), gpu_address);
			}
		};

		// Return the world-space XY position encoded by one static grid vertex.
		v2 WaterXY(v2 const& extent, iv2 const& grid, int ix, int iy)
		{
			auto const u = float(ix) / float(grid.x);
			auto const v = float(iy) / float(grid.y);
			return v2{
				(u - 0.5f) * extent.x,
				(v - 0.5f) * extent.y,
			};
		}
	}

	// Create immutable grid geometry and its sandbox-local water shader.
	WaterVisual::WaterVisual(rdr12::Renderer& rdr, scene_loader::WaterDesc const& water, v2 extent)
		: m_inst()
	{
		auto const vertex_count = (water.grid.x + 1) * (water.grid.y + 1);
		auto const index_of = [&](int ix, int iy)
		{
			return ix + iy * (water.grid.x + 1);
		};

		// Store only the flat XY grid. The vertex shader supplies height and normals every rendered frame.
		rdr12::ModelGenerator::Buffers<rdr12::Vert> buffers;
		auto const index_stride = static_cast<int>(vertex_count < 0xFFFF ? sizeof(uint16_t) : sizeof(uint32_t));
		buffers.Reset(vertex_count, 0, 0, index_stride);
		for (int iy = 0; iy != water.grid.y + 1; ++iy)
		{
			for (int ix = 0; ix != water.grid.x + 1; ++ix)
			{
				auto const index = index_of(ix, iy);
				auto const xy_ws = WaterXY(extent, water.grid, ix, iy);
				auto& vert = buffers.m_vcont[index];
				vert.m_vert = v4(xy_ws.x, xy_ws.y, water.surface.m_level, 1.0f);
				vert.m_diff = water.colour;
				vert.m_norm = v4::ZAxis();
				vert.m_tex0 = v2(float(ix) / float(water.grid.x), float(iy) / float(water.grid.y));
				vert.m_idx0 = iv2::Zero();
			}
		}

		// The index buffer remains immutable after this upload; larger diagnostic grids automatically use 32-bit indices.
		for (int iy = 0; iy != water.grid.y; ++iy)
		{
			for (int ix = 0; ix != water.grid.x; ++ix)
			{
				auto const i0 = index_of(ix + 0, iy + 0);
				auto const i1 = index_of(ix + 1, iy + 0);
				auto const i2 = index_of(ix + 0, iy + 1);
				auto const i3 = index_of(ix + 1, iy + 1);
				buffers.m_icont.push_back(i0);
				buffers.m_icont.push_back(i1);
				buffers.m_icont.push_back(i2);
				buffers.m_icont.push_back(i2);
				buffers.m_icont.push_back(i1);
				buffers.m_icont.push_back(i3);
			}
		}

		auto wave_height = 0.0f;
		for (auto const& wave : water.surface.m_waves)
			wave_height += std::abs(wave.m_amplitude);

		buffers.m_name = "Water";
		buffers.m_bbox = BBox(
			v4(0.0f, 0.0f, water.surface.m_level, 1.0f),
			v4(0.5f * extent.x, 0.5f * extent.y, wave_height, 0.0f));

		auto shader = rdr12::Shader::Create<WaterShader>(rdr, water.surface);
		buffers.m_ncont.push_back(
			rdr12::NuggetDesc(rdr12::ETopo::TriList, rdr12::EGeom::Vert | rdr12::EGeom::Colr | rdr12::EGeom::Norm | rdr12::EGeom::Tex0)
				.alpha_geom()
				.mat([&](rdr12::MaterialSimple& material)
				{
					material.use_shader_overlay(rdr12::ERenderStep::RenderForward, shader);
				}));

		rdr12::ResourceFactory factory(rdr);
		rdr12::ModelGenerator::Cache cache{ buffers };
		m_inst.m_model = rdr12::ModelGenerator::Create<rdr12::Vert>(factory, cache);
		m_inst.m_i2w = m4x4::Identity();
		m_inst.m_reflectivity = 0.5f;
		m_inst.m_time_s = 0.0f;
	}

	// Add the visual to a render scene using the current simulation time.
	void WaterVisual::AddToScene(rdr12::Scene& scene, float time_s)
	{
		m_inst.m_time_s = time_s;
		scene.AddInstance(m_inst);
	}
}
