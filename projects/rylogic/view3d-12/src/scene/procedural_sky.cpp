//************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2025
//************************************
#include "pr/view3d-12/forward.h"
#include "pr/view3d-12/scene/procedural_sky.h"
#include "pr/view3d-12/shaders/shader_forward.h"
#include "pr/view3d-12/model/model_generator.h"
#include "pr/view3d-12/model/vertex_layout.h"
#include "pr/view3d-12/material/material_simple.h"
#include "pr/view3d-12/resource/resource_factory.h"
#include "view3d-12/src/shaders/common.h"
#include "view3d-12/src/shaders/hlsl/sky/procedural_sky_cbuf.hlsli"

namespace pr::rdr12
{
	// Replaces the forward vertex/pixel stages while using its existing overlay constant-buffer slot.
	struct ProceduralSkyShader : Shader
	{
		sky::CBufProceduralSky m_cbuf;
		TextureCubePtr m_background;
		::pr::compute::Descriptor m_null_cube;

		// Use the renderer's build-time shader bytecode and default daylight parameters.
		explicit ProceduralSkyShader(Renderer& rdr)
			: Shader(rdr)
			, m_cbuf{
				.sun_direction = Normalise(v4(0.5f, 0.3f, 0.8f, 0)),
				.sun_colour = v4(1.0f, 0.95f, 0.85f, 1),
				.sun_intensity = 1.0f,
				.blend_weight = 1.0f,
				.world_to_sky = m4x4::Identity(),
				.world_to_cube = m4x4::Identity(),
			}
		{
			static_assert(sizeof(m_cbuf) == 176);
			m_code.VS = shader_code::procedural_sky_vs;
			m_code.PS = shader_code::procedural_sky_ps;

			// Bind a valid null descriptor even when the atmosphere has no source cubemap.
			ResourceStore::Access store(rdr);
			auto desc = D3D12_SHADER_RESOURCE_VIEW_DESC{
				.Format = DXGI_FORMAT_R8G8B8A8_UNORM,
				.ViewDimension = D3D12_SRV_DIMENSION_TEXTURECUBE,
				.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING,
				.TextureCube = { .MostDetailedMip = 0, .MipLevels = 1, .ResourceMinLODClamp = 0 },
			};
			m_null_cube = store.Descriptors().Create(nullptr, desc);
		}

		// Return the owned null view to the store; the texture member releases the retained source independently.
		~ProceduralSkyShader() override
		{
			ResourceStore::Access store(rdr());
			store.Descriptors().Release(m_null_cube);
		}

		// Bind sky constants and the background without replacing the shared material or reflection descriptors.
		void SetupElement(ID3D12GraphicsCommandList* cmd_list, GpuUploadBuffer& upload, Scene const& scene, DrawListElement const* dle) override
		{
			if (dle == nullptr)
				return;

			auto gpu_address = upload.Add(m_cbuf, D3D12_CONSTANT_BUFFER_DATA_PLACEMENT_ALIGNMENT, true);
			cmd_list->SetGraphicsRootConstantBufferView(static_cast<UINT>(shaders::fwd::ERootParam::CBufScreenSpace), gpu_address);
			auto cube = scene.wnd().m_heap_view.Add(m_background ? m_background->m_srv : m_null_cube);
			cmd_list->SetGraphicsRootDescriptorTable(static_cast<UINT>(shaders::fwd::ERootParam::SkyTexture), cube);
		}
	};

	// Build one persistent background triangle whose shader derives world directions from the camera.
	ProceduralSky::ProceduralSky(Renderer& rdr)
		: m_inst()
		, m_shader()
	{
		// A full-screen triangle covers perspective and orthographic views without cube seams or clip-distance dependence.
		rdr12::ModelGenerator::Buffers<Vert> buf;
		buf.Reset(3, 0, 0, sizeof(uint16_t));
		static v4 const verts[] = {
			v4(-1, -1, 1, 1),
			v4(-1, +3, 1, 1),
			v4(+3, -1, 1, 1),
		};
		for (int i = 0; i != 3; ++i)
		{
			auto& v = buf.m_vcont[i];
			v.m_vert = verts[i];
			v.m_diff = Colour(1.0f, 1.0f, 1.0f, 1.0f);
			v.m_norm = v4::Zero();
			v.m_tex0 = v2::Zero();
			v.m_idx0 = iv2::Zero();
			buf.m_icont.push_back(s_cast<uint16_t>(i));
		}
		buf.m_bbox = BBox(v4::Origin(), v4(1, 1, 1, 0));

		// The material retains the shader, so its pointer remains valid for the lifetime of the model.
		auto shdr = Shader::Create<ProceduralSkyShader>(rdr);
		m_shader = shdr.get();

		buf.m_ncont.push_back(
			NuggetDesc(ETopo::TriList, EGeom::Vert | EGeom::Colr)
				.flags(ENuggetFlag::ShadowCastExclude)
				.pso<EPipeState::CullMode>(D3D12_CULL_MODE_NONE)
				.pso<EPipeState::DepthWriteMask>(D3D12_DEPTH_WRITE_MASK_ZERO)
				.pso<EPipeState::DepthFunc>(D3D12_COMPARISON_FUNC_LESS_EQUAL)
				.mat([&](MaterialSimple& m) {
					m.use_shader_overlay(ERenderStep::RenderForward, shdr);
				})
			);

		// Upload geometry once; subsequent frames change only sky constants and the optional source binding.
		auto colour = Colour32White;
		auto opts = ModelGenerator::CreateOptions().colours({ &colour, 1 });
		ResourceFactory factory(rdr);
		ModelGenerator::Cache cache{buf};
		m_inst.m_model = ModelGenerator::Create<Vert>(factory, cache, &opts);
		m_inst.m_i2w = m4x4::Identity();
		m_inst.m_sko.Group(ESortGroup::Skybox);
		factory.FlushToGpu(EGpuFlush::Block);
	}

	// Reject invalid values before modifying the live shader so failed updates leave the previous sky intact.
	void ProceduralSky::Update(v4 sun_direction, v4 sun_colour, float sun_intensity)
	{
		sun_direction.w = 0;
		auto length_sq = LengthSq(sun_direction);
		if (!std::isfinite(length_sq) || length_sq <= 0 ||
			!std::isfinite(sun_colour.x) || !std::isfinite(sun_colour.y) || !std::isfinite(sun_colour.z) ||
			sun_colour.x < 0 || sun_colour.y < 0 || sun_colour.z < 0 ||
			!std::isfinite(sun_intensity) || sun_intensity < 0)
			throw std::invalid_argument("Procedural sky requires a finite nonzero sun direction and finite nonnegative colour/intensity");

		m_shader->m_cbuf.sun_direction = Normalise(sun_direction);
		m_shader->m_cbuf.sun_colour = sun_colour;
		m_shader->m_cbuf.sun_intensity = sun_intensity;
	}

	// The shader handles the far plane, while the instance remains at the camera for scene bookkeeping.
	void ProceduralSky::AddToScene(Scene& scene)
	{
		if (!m_inst.m_model)
			return;

		// Keep the model independent of the application's world scale and draw distance.
		m_inst.m_i2w = m4x4::Identity();
		m_inst.m_i2w.pos = scene.m_cam.CameraToWorld().pos;
		scene.AddInstance(m_inst);
	}

	// Validate first so a rejected update cannot partly replace the source or its direction mapping.
	void ProceduralSky::Blend(TextureCubePtr background, float weight, m4x4 const& world_to_sky, m4x4 const& world_to_background)
	{
		if (!std::isfinite(weight) || weight < 0 || weight > 1 || (!background && weight != 1))
			throw std::invalid_argument("Sky blend weight must be in [0,1], or 1 when no cubemap is supplied");

		// Only direction rotations are accepted; translation and scale belong to the caller's scene geometry.
		auto valid_rotation = [](m4x4 const& matrix)
		{
			for (auto column : {matrix.x, matrix.y, matrix.z, matrix.pos})
			{
				for (auto value : {column.x, column.y, column.z, column.w})
				{
					if (!std::isfinite(value))
						return false;
				}
			}
			return IsOrthonormal(matrix.rot, 0.0001f) && matrix.x.w == 0 && matrix.y.w == 0 && matrix.z.w == 0 &&
				matrix.pos.x == 0 && matrix.pos.y == 0 && matrix.pos.z == 0 && matrix.pos.w == 1;
		};
		if (!valid_rotation(world_to_sky) || !valid_rotation(world_to_background) ||
			(background && (!valid_rotation(background->m_cube2w) || &background->rdr() != &m_shader->rdr())))
			throw std::invalid_argument("Sky direction transforms must be finite rotations and the cubemap must belong to this renderer");

		// Retain the native texture independently of the caller's wrapper; update only constants during a fade.
		m_shader->m_cbuf.blend_weight = weight;
		m_shader->m_cbuf.world_to_sky = world_to_sky;
		m_shader->m_cbuf.world_to_cube = background ? InvertOrthonormal(background->m_cube2w) * world_to_background : m4x4::Identity();
		m_shader->m_background = std::move(background);
	}
}
