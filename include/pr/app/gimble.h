//*****************************************************************************************
// Application Framework
//  Copyright (c) Rylogic Ltd 2012
//*****************************************************************************************
#pragma once
#include "pr/app/forward.h"

namespace pr::app
{
	// A gimble model
	struct Gimble
	{
		// A renderer instance type for the body
		#define PR_RDR_INST(x)\
			x(m4x4          ,m_i2w   ,rdr::EInstComp::I2WTransform)\
			x(rdr::ModelPtr ,m_model ,rdr::EInstComp::ModelPtr    )
		PR_RDR_DEFINE_INSTANCE(Instance, PR_RDR_INST);
		#undef PR_RDR_INST

		Instance m_inst;    // The gimble instance
		v4       m_ofs_pos; // The offset position from the camera focus point
		float    m_scale;   // A model size scaler.

		// Constructs a gimble model and instance.
		Gimble(Renderer& rdr)
			:m_inst()
			,m_ofs_pos(v4::Zero())
			,m_scale(1.0f)
		{
			InitModel(rdr);
		}

		// Handler for adding this object to the scene
		void HandleUpdateScene(rdr::Scene& scene, EmptyArgs const&)
		{
			auto const& view = scene.m_view;
			m_inst.m_i2w = m4x4::Scale(m_scale, view.FocusPoint() + view.CameraToWorld() * m_ofs_pos);
			scene.AddInstance(m_inst);
		}

	private:

		// Create a model for a 'gimble'
		void InitModel(Renderer& rdr)
		{
			using namespace pr::rdr;

			Vert const verts[] =
			{
				{{-0.1f,  0.0f,  0.0f, 1.0f}, {1.0f, 0.0f, 0.0f, 1.0f}, v4::Zero(), v2Zero},
				{{ 1.0f,  0.0f,  0.0f, 1.0f}, {1.0f, 0.0f, 0.0f, 1.0f}, v4::Zero(), v2Zero},
				{{ 0.0f, -0.1f,  0.0f, 1.0f}, {0.0f, 1.0f, 0.0f, 1.0f}, v4::Zero(), v2Zero},
				{{ 0.0f,  1.0f,  0.0f, 1.0f}, {0.0f, 1.0f, 0.0f, 1.0f}, v4::Zero(), v2Zero},
				{{ 0.0f,  0.0f, -0.1f, 1.0f}, {0.0f, 0.0f, 1.0f, 1.0f}, v4::Zero(), v2Zero},
				{{ 0.0f,  0.0f,  1.0f, 1.0f}, {0.0f, 0.0f, 1.0f, 1.0f}, v4::Zero(), v2Zero},
			};
			uint16_t const indices[] =
			{
				0, 1, 2, 3, 4, 5
			};

			// Create the gimble model
			m_inst.m_model = rdr.m_mdl_mgr.CreateModel(MdlSettings(verts, indices, BBox::Reset(), "gimble"));

			NuggetProps mat; // Get a suitable shader
			mat.m_topo = ETopo::LineList;
			mat.m_geom = EGeom::Vert | EGeom::Colr;

			// Create a render nugget
			m_inst.m_model->CreateNugget(mat);
		}
	};
}
