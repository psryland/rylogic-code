// Fluid
#include "src/fluid_visualisation.h"

using namespace pr::rdr12;

namespace pr::fluid
{
	FluidVisualisation::FluidVisualisation(Renderer& rdr, Scene& scn)
		: m_rdr(&rdr)
		, m_scn(&scn)
		, m_gfx_scene()
		, m_tex_map()
		, m_gs_points(Shader::Create<shaders::PointSpriteGS>(rdr, v2(0.1f), true))
		, m_gfx_fluid()
		, m_gfx_vector_field()
		, m_gfx_map()
	{
		// Create a texture for displaying a fluid property
		{
			ResourceFactory factory(rdr);

			auto src = Image(4096, 4096, nullptr, DXGI_FORMAT_B8G8R8A8_UNORM);
			auto rdesc = ResDesc::Tex2D(src, 1).usage(EUsage::UnorderedAccess);
			auto tdesc = TextureDesc(AutoId, rdesc).name("Fluid:Map");
			m_tex_map = factory.CreateTexture2D(tdesc);

			auto opts = ModelGenerator::CreateOptions().bake(m4x4::Translation(0,0,-0.001f));
			m_gfx_map.m_model = ModelGenerator::Quad(factory, AxisId::PosZ, { 0, 0 }, 2, 2, iv2::Zero(), &opts);
			m_gfx_map.m_model->m_name = "Fluid:MapQuad";
			m_gfx_map.m_i2w = m4x4::Identity();
			
			auto& nug = *m_gfx_map.m_model->m_nuggets.get();
			nug.m_tex_diffuse = m_tex_map;
			nug.m_sam_diffuse = rdr.store().StockSampler(EStockSampler::PointClamp);
		}
	}
	FluidVisualisation::~FluidVisualisation()
	{
		// Remove instances from the scene before deleting them
		m_scn->ClearDrawlists();
	}

	// Reset the visualisation
	void FluidVisualisation::Init(int particle_capacity, std::string_view ldr, D3DPtr<ID3D12Resource> particle_buffer)
	{
		ResourceFactory factory(*m_rdr);

		// Create the visualisation scene
		m_gfx_scene = rdr12::ldraw::Parse(*m_rdr, ldr)[0];

		// Create a dynamic model for the fluid particles (using the particle buffer)
		{
			auto vb = ResDesc::VBuf<Vert>(particle_buffer.get()).usage(EUsage::UnorderedAccess);
			auto ib = ResDesc::IBuf<uint16_t>(0, {});
			auto mdesc = ModelDesc().vbuf(vb).ibuf(ib).name("Fluid:Particles");
			m_gfx_fluid.m_model = factory.CreateModel(mdesc, particle_buffer, nullptr);
			m_gfx_fluid.m_model->CreateNugget(factory, NuggetDesc(ETopo::PointList, EGeom::Vert | EGeom::Colr | EGeom::Tex0)
				.use_shader_overlay(ERenderStep::RenderForward, m_gs_points)
				.tex_diffuse(m_rdr->store().StockTexture(EStockTexture::WhiteDot))//WhiteSphere))
				.irange(0, 0));
			m_gfx_fluid.m_i2w = m4x4::Identity();
		}

		// Create a dynamic model for the pressure gradient lines
		{
			auto vb = ResDesc::VBuf<Vert>(3LL * particle_capacity, {});
			auto ib = ResDesc::IBuf<uint16_t>(0, {});
			auto mdesc = ModelDesc().vbuf(vb).ibuf(ib).name("Fluid:VectorField");
			m_gfx_vector_field.m_model = factory.CreateModel(mdesc);
			m_gfx_vector_field.m_model->CreateNugget(factory, NuggetDesc(ETopo::LineList, EGeom::Vert | EGeom::Colr).irange(0, 0));
			m_gfx_vector_field.m_i2w = m4x4::Identity();
		}

		// Make sure everything ready to go
		factory.FlushToGpu(EGpuFlush::Block);
	}

	// Populate the vector field with
	void FluidVisualisation::UpdateVectorField(std::span<particle_t const> particles, float particle_radius, float scale, int mode) const
	{
		ResourceFactory factory(*m_rdr);
		UpdateSubresourceScope update = m_gfx_vector_field.m_model->UpdateVertices(factory.CmdList(), factory.UploadBuffer());
		auto* beg = update.ptr<Vert>();
		auto* end = beg + m_gfx_vector_field.m_model->m_vcount;
		memset(beg, 0, (end - beg) * sizeof(Vert));

		Colour32 const col = 0xFF800000;

		switch (mode)
		{
			case 0:
			{
				break;
			}
			case 1:
			{
				auto* ptr = beg;
				m_gfx_vector_field.m_model->DeleteNuggets();
				for (auto const& particle : particles)
				{
					ptr->m_vert = particle.pos;
					ptr->m_diff = col;
					++ptr;

					ptr->m_vert = particle.pos + scale * particle.vel;
					ptr->m_diff = col;
					++ptr;
				}
				m_gfx_vector_field.m_model->CreateNugget(factory,
					NuggetDesc(ETopo::LineList, EGeom::Vert | EGeom::Colr)
					.vrange(0, 2 * particles.size())
					.irange(0, 0));
				break;
			}
			case 2:
			{
				auto* ptr = beg;
				m_gfx_vector_field.m_model->DeleteNuggets();
				for (auto const& particle : particles)
				{
					if (end - ptr < 2) break;

					ptr->m_vert = particle.pos;
					ptr->m_diff = col;
					++ptr;

					ptr->m_vert = particle.pos + scale * particle.acc;
					ptr->m_diff = col;
					++ptr;
				}
				m_gfx_vector_field.m_model->CreateNugget(factory,
					NuggetDesc(ETopo::LineList, EGeom::Vert | EGeom::Colr)
					.vrange(0, 2 * particles.size())
					.irange(0, 0));
				break;
			}
			case 3:
			{
				auto* ptr0 = beg;
				m_gfx_vector_field.m_model->DeleteNuggets();
				for (auto const& particle : particles)
				{
					if (particle.surface.w >= particle_radius)
						continue;

					if (end - ptr0 < 1) break;
					ptr0->m_vert = particle.pos - 2.0f * (particle.surface.w) * particle.surface.w0();
					ptr0->m_diff = col;
					++ptr0;
				}
				auto* ptr1 = ptr0;
				for (auto const& particle : particles)
				{
					if (particle.surface.w >= particle_radius)
						continue;

					if (end - ptr1 < 2) break;
					ptr1->m_vert = particle.pos;
					ptr1->m_diff = col;
					++ptr1;
					ptr1->m_vert = particle.pos - 2.0f * (particle.surface.w) * particle.surface.w0();
					ptr1->m_diff = col;
					++ptr1;
				}
				m_gfx_vector_field.m_model->CreateNugget(factory,
					NuggetDesc(ETopo::PointList, EGeom::Vert | EGeom::Colr | EGeom::Tex0)
					.use_shader_overlay(ERenderStep::RenderForward, m_gs_points)
					.tex_diffuse(m_rdr->store().StockTexture(EStockTexture::WhiteDot))//WhiteSphere))
					.vrange(0, ptr0 - beg)
					.irange(0, 0));
				m_gfx_vector_field.m_model->CreateNugget(factory,
					NuggetDesc(ETopo::LineList, EGeom::Vert | EGeom::Colr)
					.vrange(ptr0 - beg, ptr1 - beg)
					.irange(0, 0));

				break;
			}
		}
		update.Commit();
	}

	// Add the particles to the scene that renders them
	void FluidVisualisation::AddToScene(Scene& scene, EScene flags, int particle_count) const
	{
		// Add the static scene
		m_gfx_scene->AddToScene(scene);

		// The particles
		if (AllSet(flags, EScene::Particles))
		{
			auto& nug = *m_gfx_fluid.m_model->m_nuggets.get();
			nug.m_vrange = { 0, particle_count };
			scene.AddInstance(m_gfx_fluid);
		}

		// The vector field
		if (AllSet(flags, EScene::VectorField))
		{
			auto& nug = *m_gfx_fluid.m_model->m_nuggets.get();
			nug.m_vrange = { 0, particle_count };
			scene.AddInstance(m_gfx_vector_field);
		}

		// The map
		if (AllSet(flags, EScene::Map))
			scene.AddInstance(m_gfx_map);

		// Update the gradient
		#if 0
		if (Params.ShowGradients)
		{
			Tweakable<float, "GradientScale"> Scale = 0.0001f;
			Colour32 const col = Colour32Green;

			UpdateSubresourceScope update = m_gfx_gradient.m_model->UpdateVertices();
			auto* ptr = update.ptr<Vert>();
			for (auto& particle : ReadParticles())
			{
				ptr->m_vert = particle.pos;
				ptr->m_diff = col;
				ptr->m_norm = v4::Zero();
				ptr->m_tex0 = v2::Zero();
				ptr->pad = v2::Zero();
				++ptr;

				ptr->m_vert = particle.pos + Scale * m_sim->PressureAt(particle.m_pos, m_sim->m_particles.index(particle));
				ptr->m_diff = col;
				ptr->m_norm = v4::Zero();
				ptr->m_tex0 = v2::Zero();
				ptr->pad = v2::Zero();
				++ptr;
			}
			update.Commit(D3D12_RESOURCE_STATE_VERTEX_AND_CONSTANT_BUFFER);
			scene.AddInstance(m_gfx_gradient);
		}
		#endif

		#if 0 // Read back particles
		auto const& p = particles[0];
			
		if (m_time != m_last_frame_rendered)
			OutputDebugStringA(pr::FmtS("Pos: %3.3f %3.3f %3.3f - |Vel|: %3.3f\n"
				,p.pos.x, p.pos.y, p.pos.z
				,Length(p.vel)
		));
		#endif
	}

	// Handle input
	void FluidVisualisation::OnMouseButton(gui::MouseEventArgs&)
	{
	}
	void FluidVisualisation::OnMouseMove(gui::MouseEventArgs&)
	{
	}
	void FluidVisualisation::OnMouseWheel(gui::MouseWheelArgs&)
	{
	}
	void FluidVisualisation::OnKey(gui::KeyEventArgs&)
	{
	}
}
