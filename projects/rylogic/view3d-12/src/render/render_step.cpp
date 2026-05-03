//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#include "pr/view3d-12/render/render_step.h"
#include "pr/view3d-12/scene/scene.h"
#include "pr/view3d-12/main/renderer.h"
#include "pr/view3d-12/main/window.h"
#include "pr/view3d-12/model/model.h"
#include "pr/view3d-12/model/nugget.h"
#include "pr/view3d-12/instance/instance.h"
#include "pr/view3d-12/resource/resource_store.h"
#include "pr/view3d-12/render/drawlist_element.h"

namespace pr::rdr12
{
	RenderStep::RenderStep(ERenderStep id, Scene& scene)
		: m_step_id(id)
		, m_scene(&scene)
		, m_drawlist()
		, m_boundaries()
		, m_sort_needed(true)
		, m_upload_buffer(wnd().m_gsync, 1ULL * 1024 * 1024)
		, m_default_pipe_state()
		, m_pipe_state_pool(wnd())
		, m_evt_model_delete(rdr().store().ModelDeleted += std::bind(&RenderStep::OnModelDeleted, this, _1, _2))
	{}

	// Access the renderer
	ID3D12Device4* RenderStep::d3d() const
	{
		return rdr().d3d();
	}
	Renderer& RenderStep::rdr() const
	{
		return wnd().rdr();
	}
	Window& RenderStep::wnd() const
	{
		return scn().wnd();
	}
	Scene& RenderStep::scn() const
	{
		return *m_scene;
	}

	// Reset/Populate the drawlist
	void RenderStep::ClearDrawlist()
	{
		if (auto drawlist = m_drawlist.lock())
			drawlist->resize(0);
	}

	// Sort the draw list based on sort key
	void RenderStep::Sort()
	{
		auto drawlist = m_drawlist.lock();

		// Sort by sort key
		std::sort(std::begin(*drawlist), std::end(*drawlist));

		// Find the AlphaFront and AlphaBack range, and sort them by distance from the camera
		auto alpha_back = std::lower_bound(drawlist->begin(), drawlist->end(), SortKey(ESortGroup::AlphaBack));
		auto alpha_front = std::lower_bound(drawlist->begin(), drawlist->end(), SortKey(ESortGroup::AlphaFront));
		auto alpha_end = std::lower_bound(drawlist->begin(), drawlist->end(), SortKey(ESortGroup::PostAlpha));

		auto cam_pos = scn().m_cam.CameraToWorld().pos;
		std::sort(alpha_back, alpha_front, [=](DrawListElement const& lhs, DrawListElement const& rhs)
		{
			auto dl = LengthSq(GetO2W(*lhs.m_instance).pos - cam_pos);
			auto dr = LengthSq(GetO2W(*rhs.m_instance).pos - cam_pos);
			return dl > dr; // back to front
		});
		std::sort(alpha_front, alpha_end, [=](DrawListElement const& lhs, DrawListElement const& rhs)
		{
			auto dl = LengthSq(GetO2W(*lhs.m_instance).pos - cam_pos);
			auto dr = LengthSq(GetO2W(*rhs.m_instance).pos - cam_pos);
			return dl > dr; // back to front
		});

		// Sorting done
		m_boundaries[ESortGroup::AlphaBack] = s_cast<int>(std::distance(drawlist->begin(), alpha_back));
		m_boundaries[ESortGroup::AlphaFront] = s_cast<int>(std::distance(drawlist->begin(), alpha_front));
		m_boundaries[ESortGroup::PostAlpha] = s_cast<int>(std::distance(drawlist->begin(), alpha_end));
		m_sort_needed = false;
	}
	void RenderStep::SortIfNeeded(dl_boundaries* alpha_start)
	{
		if (m_sort_needed)
			Sort();

		if (alpha_start)
			*alpha_start = m_boundaries;
	}

	// Add an instance. The instance, model, and nuggets must be resident for the entire time
	// that the instance is in the draw list, i.e. until 'RemoveInstance' or 'ClearDrawlist' is called.
	void RenderStep::AddInstance(BaseInstance const& inst)
	{
		// Ask the instance to provide the nuggets. This could be different to the nuggets owned by the model.
		// Also, if the instance uses an alpha tint, it will ask the model for it's "alpha" nuggets instead
		// of its "default" nuggets. This allows the same model to be used for both opaque and alpha tinted instances.
		auto nuggets = GetNuggets(inst);

		// Debug checks
		#if PR_DBG_RDR
		{
			// Note: Only print debug messages here. The debug behaviour needs to match the release behaviour
			auto const& model = GetModel(inst);

			// Check that nuggets have been created
			if (nuggets == nullptr && !AllSet(model->m_dbg_flags, Model::EDbgFlags::WarnedNoRenderNuggets))
			{
				PR_INFO(PR_DBG_RDR, std::format("This model ({}) has no nuggets, you need to call CreateNugget() on the model first\n", model->m_name));
				model->m_dbg_flags = SetBits(model->m_dbg_flags, Model::EDbgFlags::WarnedNoRenderNuggets, true);
			}

			// Check the instance transform is valid
			auto& o2w = GetO2W(inst);
			auto flags = GetFlags(inst);
			if (!IsFinite(o2w) && !AllSet(model->m_dbg_flags, Model::EDbgFlags::WarnedInvalidTransform))
			{
				PR_INFO(PR_DBG_RDR, std::format("This model (%s) has an invalid instance transform\n", model->m_name));
				model->m_dbg_flags = SetBits(model->m_dbg_flags, Model::EDbgFlags::WarnedInvalidTransform, true);
			}
			if (!AllSet(flags, EInstFlag::NonAffine) && !IsAffine(o2w) && !AllSet(model->m_dbg_flags, Model::EDbgFlags::WarnedInvalidTransform))
			{
				PR_INFO(PR_DBG_RDR, std::format("This model (%s) has a non-affine instance transform\n", model->m_name));
				model->m_dbg_flags = SetBits(model->m_dbg_flags, Model::EDbgFlags::WarnedInvalidTransform, true);
			}
		}
		#endif

		// Add the model nuggets to the draw list
		if (auto drawlist = m_drawlist.lock())
			AddNuggets(inst, nuggets, *drawlist);

		// Flag the draw list as changed
		m_sort_needed = true;
	}

	// Remove an instance from the scene
	void RenderStep::RemoveInstance(BaseInstance const& inst)
	{
		if (auto drawlist = m_drawlist.lock())
			erase_if(*drawlist, [&](DrawListElement const& dle){ return dle.m_instance == &inst; });
	}

	// Remove a batch of instances. Optimised by a single past through the draw list
	void RenderStep::RemoveInstances(BaseInstance const** inst, std::size_t count)
	{
		// Make a sorted list from the batch to remove
		BaseInstance const** doomed = PR_ALLOCA_POD(BaseInstance const* , count);
		BaseInstance const** doomed_end = doomed + count;
		std::copy(inst, inst + count, doomed);
		std::sort(doomed, doomed_end);

		// Remove instances
		if (auto drawlist = m_drawlist.lock())
		{
			erase_if(*drawlist, [&](DrawListElement const& dle)
			{
				auto iter = std::lower_bound(doomed, doomed_end, dle.m_instance);
				return iter != doomed_end && *iter == dle.m_instance;
			});
		}
	}

	// Notification of a model being destroyed
	void RenderStep::OnModelDeleted(Model& model, EmptyArgs const&) const // any thread
	{
		// Check the model is not current in a draw list
		if (auto drawlist = m_drawlist.lock())
		{
			for (auto& dle : *drawlist)
			{
				if (&model == dle.m_nugget->m_model)
					throw std::runtime_error("Model being deleted is still in the draw list");
			}
		}
	}
}
