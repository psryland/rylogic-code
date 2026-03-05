//*****************************************************************************************
// Application Framework
//  Copyright (c) Rylogic Ltd 2012
//*****************************************************************************************
#pragma once
#include "pr/app/forward.h"

namespace pr::app
{
	// A base class for a sky box
	struct Skybox
	{
		// Sky box styles - implies texture organisation as well
		enum class EStyle
		{
			// This is a geosphere with inward facing normals
			Geosphere,

			// This is a cubic dome, the texture should be a '+' shape with
			// the top portion from 0.25->0.75, and sides from 0->0.25, 0.75->1.0
			FiveSidedCube,

			// The is a full 6-sided cube, 'texpath' should be a filepath with
			// the format 'path\filename??.extn' where '??' will be replaced by
			// +X,-X,+Y,-Y,+Z,-Z to generate the six texture filepaths
			SixSidedCube,
		};

		// A renderer instance type for the sky box
		struct Instance
		{
			#define PR_RDR_INST(x)\
			x(m4x4              ,m_i2w   ,rdr12::EInstComp::I2WTransform   )\
			x(rdr12::ModelPtr   ,m_model ,rdr12::EInstComp::ModelPtr       )\
			x(rdr12::SKOverride ,m_sko   ,rdr12::EInstComp::SortkeyOverride)
			PR_RDR12_INSTANCE_MEMBERS(Instance, PR_RDR_INST);
			#undef PR_RDR_INST
		};

		using TexCont = pr::vector<rdr12::Texture2DPtr>;

		Instance m_inst;  // The sky box instance
		TexCont  m_tex;   // The textures used in the sky box
		float    m_scale; // Model scaler
		m4x4     m_i2w;   // The base orientation transform for the sky box

		// Constructs a sky box model and instance.
		// 'texpath' should be an unrolled cube texture
		Skybox(rdr12::Renderer& rdr, std::filesystem::path const& texpath, EStyle tex_style, float scale = 100.0f, m3x4 const& ori = m3x4::Identity())
			:m_inst()
			,m_tex()
			,m_scale(scale)
			,m_i2w(ori, v4::Origin())
		{
			switch (tex_style)
			{
				case EStyle::Geosphere:     InitGeosphere(rdr, texpath); break;
				case EStyle::FiveSidedCube: InitFiveSidedCube(rdr, texpath); break;
				case EStyle::SixSidedCube:  InitSixSidedCube(rdr, texpath); break;
				default: throw std::runtime_error("Unsupported texture style");
			}

			// Set the sort key so that the sky box draws last
			m_inst.m_sko.Group(rdr12::ESortGroup::Skybox);
			m_inst.m_model->m_name = "sky box";
		}

		// Add the sky box to a viewport
		void AddToScene(rdr12::Scene& scene)
		{
			m_inst.m_i2w = m_i2w * m4x4::Scale(m_scale, v4::Origin());
			m_inst.m_i2w.pos = scene.m_cam.CameraToWorld().pos;
			scene.AddInstance(m_inst);
		}

	private:

		// Create a model for a geosphere sky box
		void InitGeosphere(rdr12::Renderer& rdr, std::filesystem::path const& texpath)
		{
			using namespace pr::rdr12;
			ResourceFactory factory(rdr);

			ResDesc rdesc = ResDesc::Tex2D({});
			TextureDesc tdesc = TextureDesc(rdr12::AutoId, rdesc).name("skybox");
			auto skytex = factory.CreateTexture2D(texpath, tdesc);

			//ddata.m_geom = EGeom::Vert | EGeom::Tex0;
			//ddata.m_rsb = RSBlock::SolidCullFront();

			// Create the sky box model
			ModelGenerator::CreateOptions opts = ModelGenerator::CreateOptions().tex_diffuse(skytex, factory.CreateSampler(EStockSampler::LinearWrap));
			m_inst.m_model = ModelGenerator::Geosphere(factory, 1.0f, 3, &opts);
		}

		// Create a model for a 5-sided cubic dome
		void InitFiveSidedCube(rdr12::Renderer& rdr, std::filesystem::path const& texpath)
		{
			using namespace pr::rdr12;
			ResourceFactory factory(rdr);

			float const s = 0.5f;
			static Vert const verts[] =
			{
				{{-s,  s,  s, 1}, ColourWhite, v4::Zero(), { 0.25f, 0.25f}}, //0
				{{-s,  s, -s, 1}, ColourWhite, v4::Zero(), { 0.25f, 0.75f}}, //1
				{{ s,  s, -s, 1}, ColourWhite, v4::Zero(), { 0.75f, 0.75f}}, //2
				{{ s,  s,  s, 1}, ColourWhite, v4::Zero(), { 0.75f, 0.25f}}, //3
				{{-s, -s,  s, 1}, ColourWhite, v4::Zero(), {-0.25f, 0.25f}}, //4
				{{-s, -s, -s, 1}, ColourWhite, v4::Zero(), {-0.25f, 0.75f}}, //5
				{{-s, -s, -s, 1}, ColourWhite, v4::Zero(), { 0.25f, 1.25f}}, //6
				{{ s, -s, -s, 1}, ColourWhite, v4::Zero(), { 0.75f, 1.25f}}, //7
				{{ s, -s, -s, 1}, ColourWhite, v4::Zero(), { 1.25f, 0.75f}}, //8
				{{ s, -s,  s, 1}, ColourWhite, v4::Zero(), { 1.25f, 0.25f}}, //9
				{{ s, -s,  s, 1}, ColourWhite, v4::Zero(), { 0.75f,-0.25f}}, //10
				{{-s, -s,  s, 1}, ColourWhite, v4::Zero(), { 0.25f,-0.25f}}, //11
			};
			static uint16_t const indices[] =
			{
				0,  1,  2,  0,  2,  3,
				0,  4,  5,  0,  5,  1,
				1,  6,  7,  1,  7,  2,
				2,  8,  9,  2,  9,  3,
				3, 10, 11,  3, 11,  0,
			};

			// Create the sky box model
			auto vb = ResDesc::VBuf<Vert>(_countof(verts), verts);
			auto ib = ResDesc::IBuf<uint16_t>(_countof(indices), indices);
			ModelDesc mdesc = ModelDesc().vbuf(vb).ibuf(ib).name("sky box");
			m_inst.m_model = factory.CreateModel(mdesc);

			// Create a model nugget for the sky box
			ResDesc rdesc = ResDesc::Tex2D({});
			TextureDesc tdesc = TextureDesc(AutoId, rdesc);
			NuggetDesc ndesc = NuggetDesc(ETopo::TriList, EGeom::Vert | EGeom::Tex0).tex_diffuse(factory.CreateTexture2D(texpath, tdesc)).sam_diffuse(factory.CreateSampler(EStockSampler::LinearClamp));
			m_inst.m_model->CreateNugget(factory, ndesc);
		}

		// Create a model for a 6-sided cube
		void InitSixSidedCube(rdr12::Renderer& rdr, std::filesystem::path  const& texpath)
		{
			using namespace pr::rdr12;
			ResourceFactory factory(rdr);

			constexpr float s = 0.5f, t0 = 0.0f, t1 = 1.0f;
			static Vert const verts[] =
			{
				{{+s, +s, -s, 1}, ColourWhite, v4::Zero(), {t0, t0}}, //  0 // +X
				{{+s, -s, -s, 1}, ColourWhite, v4::Zero(), {t0, t1}}, //  1
				{{+s, -s, +s, 1}, ColourWhite, v4::Zero(), {t1, t1}}, //  2
				{{+s, +s, +s, 1}, ColourWhite, v4::Zero(), {t1, t0}}, //  3
				{{-s, +s, +s, 1}, ColourWhite, v4::Zero(), {t0, t0}}, //  4 // -X
				{{-s, -s, +s, 1}, ColourWhite, v4::Zero(), {t0, t1}}, //  5
				{{-s, -s, -s, 1}, ColourWhite, v4::Zero(), {t1, t1}}, //  6
				{{-s, +s, -s, 1}, ColourWhite, v4::Zero(), {t1, t0}}, //  7
				{{+s, +s, +s, 1}, ColourWhite, v4::Zero(), {t0, t0}}, //  8 // +Y
				{{-s, +s, +s, 1}, ColourWhite, v4::Zero(), {t0, t1}}, //  9
				{{-s, +s, -s, 1}, ColourWhite, v4::Zero(), {t1, t1}}, // 10
				{{+s, +s, -s, 1}, ColourWhite, v4::Zero(), {t1, t0}}, // 11
				{{+s, -s, -s, 1}, ColourWhite, v4::Zero(), {t0, t0}}, // 12 // -Y
				{{-s, -s, -s, 1}, ColourWhite, v4::Zero(), {t0, t1}}, // 13
				{{-s, -s, +s, 1}, ColourWhite, v4::Zero(), {t1, t1}}, // 14
				{{+s, -s, +s, 1}, ColourWhite, v4::Zero(), {t1, t0}}, // 15
				{{+s, +s, +s, 1}, ColourWhite, v4::Zero(), {t0, t0}}, // 16 // +Z
				{{+s, -s, +s, 1}, ColourWhite, v4::Zero(), {t0, t1}}, // 17
				{{-s, -s, +s, 1}, ColourWhite, v4::Zero(), {t1, t1}}, // 18
				{{-s, +s, +s, 1}, ColourWhite, v4::Zero(), {t1, t0}}, // 19
				{{-s, +s, -s, 1}, ColourWhite, v4::Zero(), {t0, t0}}, // 20 // -Z
				{{-s, -s, -s, 1}, ColourWhite, v4::Zero(), {t0, t1}}, // 21
				{{+s, -s, -s, 1}, ColourWhite, v4::Zero(), {t1, t1}}, // 22
				{{+s, +s, -s, 1}, ColourWhite, v4::Zero(), {t1, t0}}, // 23
			};
			static uint16_t const indices[] =
			{
				0, 1, 2,  0, 2, 3, // 0 - 6
				4, 5, 6,  4, 6, 7, // 6 - 12
				8, 9,10,  8,10,11, // 12 - 18
				12,13,14, 12,14,15, // 18 - 24
				16,17,18, 16,18,19, // 24 - 30
				20,21,22, 20,22,23, // 30 - 36
			};

			// Create the sky box model
			auto vb = ResDesc::VBuf<Vert>(_countof(verts), verts);
			auto ib = ResDesc::IBuf<uint16_t>(_countof(indices), indices);
			ModelDesc mdesc = ModelDesc().vbuf(vb).ibuf(ib).name("sky box");
			m_inst.m_model = factory.CreateModel(mdesc);

			// Create the model nuggets for the sky box
			NuggetDesc ndesc = NuggetDesc(ETopo::TriList, EGeom::Vert|EGeom::Tex0);

			// One texture per nugget
			auto tpath = texpath.wstring();
			size_t ofs = tpath.find(L"??", 0, 2);
			if (ofs == std::wstring::npos)
				throw std::runtime_error("Provided path does not include '??' characters");

			wchar_t const axes[6][3] = {{L"+X"},{L"-X"},{L"+Y"},{L"-Y"},{L"+Z"},{L"-Z"}};
			for (int i = 0; i != 6; ++i)
			{
				// Load the texture for this face of the sky box
				tpath[ofs+0] = axes[i][0];
				tpath[ofs+1] = axes[i][1];
				auto rdesc = ResDesc::Tex2D({});
				auto tdesc = TextureDesc(AutoId, rdesc).name("sky box");
				ndesc.tex_diffuse(factory.CreateTexture2D(tpath, tdesc)).sam_diffuse(factory.CreateSampler(EStockSampler::LinearClamp));

				// Create the render nugget for this face of the sky box
				ndesc.m_vrange = rdr12::Range(i*4, (i+1)*4);
				ndesc.m_irange = rdr12::Range(i*6, (i+1)*6);
				m_inst.m_model->CreateNugget(factory, ndesc);
			}
		}
	};
}
