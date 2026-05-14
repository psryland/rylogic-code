//************************************
// Lost at Sea
//  Copyright (c) Rylogic Ltd 2025
//************************************
#include "src/forward.h"
#include "src/world/sky/procedural_sky.h"
#include "src/world/sky/shaders/procedural_sky_shader.h"

namespace las
{
	ProceduralSky::ProceduralSky(Renderer& rdr)
		: m_inst()
		, m_shader()
	{
		// Build a unit cube (8 vertices, 12 triangles).
		// Viewed from inside with front-face culling.
		rdr12::ModelGenerator::Buffers<Vert> buf;
		buf.Reset(8, 0, 0, sizeof(uint16_t));

		// Cube vertices at (±1, ±1, ±1)
		static const v4 verts[] = {
			v4(-1, -1, -1, 1), // 0
			v4(+1, -1, -1, 1), // 1
			v4(+1, +1, -1, 1), // 2
			v4(-1, +1, -1, 1), // 3
			v4(-1, -1, +1, 1), // 4
			v4(+1, -1, +1, 1), // 5
			v4(+1, +1, +1, 1), // 6
			v4(-1, +1, +1, 1), // 7
		};
		for (int i = 0; i < 8; ++i)
		{
			auto& v = buf.m_vcont[i];
			v.m_vert = verts[i];
			v.m_diff = Colour(1.0f, 1.0f, 1.0f, 1.0f);
			v.m_norm = v4::Zero();
			v.m_tex0 = v2::Zero();
			v.m_idx0 = iv2::Zero();
		}

		// CW winding from outside each face (front-cull reveals inside)
		static const uint16_t indices[] = {
			4, 5, 6,  4, 6, 7,  // +Z
			3, 2, 1,  3, 1, 0,  // -Z
			1, 2, 6,  1, 6, 5,  // +X
			4, 7, 3,  4, 3, 0,  // -X
			2, 3, 7,  2, 7, 6,  // +Y
			0, 1, 5,  0, 5, 4,  // -Y
		};
		for (auto idx : indices)
			buf.m_icont.push_back(idx);

		buf.m_bbox = BBox(v4::Origin(), v4(1, 1, 1, 0));

		auto shdr = Shader::Create<ProceduralSkyShader>(rdr);
		m_shader = shdr.get();

		buf.m_ncont.push_back(
			NuggetDesc(ETopo::TriList, EGeom::Vert | EGeom::Colr)
				.pso<EPipeState::CullMode>(D3D12_CULL_MODE_FRONT)
				.mat([&](MaterialSimple& m) {
					m.use_shader_overlay(ERenderStep::RenderForward, shdr);
				})
			);

		auto colour = Colour32White;
		auto opts = ModelGenerator::CreateOptions().colours({ &colour, 1 });

		ResourceFactory factory(rdr);
		ModelGenerator::Cache cache{buf};
		m_inst.m_model = ModelGenerator::Create<Vert>(factory, cache, &opts);
		m_inst.m_i2w = m4x4::Identity();
		m_inst.m_sko.Group(ESortGroup::Skybox);

		factory.FlushToGpu(EGpuFlush::Block);
	}

	void ProceduralSky::PrepareRender(v4 sun_direction, v4 sun_colour, float sun_intensity)
	{
		m_shader->SetupFrame(sun_direction, sun_colour, sun_intensity);
	}

	void ProceduralSky::AddToScene(Scene& scene)
	{
		if (!m_inst.m_model)
			return;

		// Centre on camera and scale to draw distance
		m_inst.m_i2w = m4x4::Scale(DomeScale, v4::Origin());
		m_inst.m_i2w.pos = scene.m_cam.CameraToWorld().pos;
		scene.AddInstance(m_inst);
	}
}
