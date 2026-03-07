//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#include "pr/view3d-12/utility/diagnostics.h"
#include "pr/view3d-12/main/renderer.h"
#include "pr/view3d-12/main/window.h"
#include "pr/view3d-12/scene/scene.h"
#include "pr/view3d-12/resource/resource_factory.h"
#include "pr/view3d-12/shaders/shader_point_sprites.h"
#include "pr/view3d-12/shaders/shader_show_normals.h"
#include "pr/view3d-12/model/model.h"
#include "pr/view3d-12/model/nugget.h"

namespace pr::rdr12
{
	constexpr RdrId ShowNormalsId = hash::HashCT("ShowNormals");

	DiagState::DiagState(Window& wnd)
		:m_wnd(&wnd)
		,m_normal_lengths(0.1f)
		,m_normal_colour(Colour32Purple)
		,m_bboxes_visible(false)
		,m_gs_fillmode_points(Shader::Create<shaders::PointSpriteGS>(wnd.rdr(), v2(5.0f, 5.0f), false))
	{}
	Window& DiagState::wnd() const
	{
		return *m_wnd;
	}

	// Enable/Disable normals on 'model'
	void ShowNormals(Model* model, bool show)
	{
		// Notes:
		//  - The normals length property is controlled independently

		// Remove dependent nuggets used to show normals
		for (auto& nug : model->m_nuggets)
			nug.DeleteDependent([](Nugget& n) { return n.m_id == ShowNormalsId; });

		// If showing normals, add a dependent nugget for each nugget that has valid vertex normals
		if (show)
		{
			// Get or create an instance of the ShowNormals shader
			auto shdr = Shader::Create<shaders::ShowNormalsGS>(model->rdr());

			// Add a dependent nugget for each existing nugget that has vertex normals
			ResourceFactory factory(model->rdr());
			for (auto& nug : model->m_nuggets)
			{
				if (!AllSet(nug.m_geom, EGeom::Norm))
					continue;

				// Create a dependent nugget that draws the normals
				auto ndesc = NuggetDesc(ETopo::PointList, EGeom::Vert | EGeom::Colr)
					.irange(RangeZero)
					.id(ShowNormalsId)
					.use_shader_overlay(ERenderStep::RenderForward, shdr);

				auto& dep = *factory.CreateNugget(nug, model);
				nug.m_nuggets.push_back(dep);
			}
		}

		// Set a bit in the dbg flags
		model->m_dbg_flags = SetBits(model->m_dbg_flags, Model::EDbgFlags::NormalsVisible, show);
	}

	// Create a scale transform that positions a unit box at 'bbox'
	m4x4 BBoxTransform(BBox const& bbox)
	{
		return m4x4::Scale(
			bbox.SizeX() + math::tiny<float>,
			bbox.SizeY() + math::tiny<float>,
			bbox.SizeZ() + math::tiny<float>,
			bbox.Centre());
	}
}
