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
		pr::sort(*drawlist);

		// Sorting done
		m_sort_needed = false;
	}
	void RenderStep::SortIfNeeded()
	{
		if (!m_sort_needed) return;
		Sort();
	}

	// Add an instance. The instance, model, and nuggets must be resident for the entire time
	// that the instance is in the draw list, i.e. until 'RemoveInstance' or 'ClearDrawlist' is called.
	void RenderStep::AddInstance(BaseInstance const& inst)
	{
		// Get the model associated with the instance
		auto const& model = GetModel(inst);
		if (model == nullptr)
			throw std::runtime_error("Null model pointer");

		// Get the nuggets for this render step
		auto& nuggets = model->m_nuggets;

		// Debug checks
		#if PR_DBG_RDR
		{
			// Note: Only print debug messages here. The debug behaviour needs to match the release behaviour

			// Check that nuggets have been created
			if (nuggets.empty() && !AllSet(model->m_dbg_flags, Model::EDbgFlags::WarnedNoRenderNuggets))
			{
				PR_INFO(PR_DBG_RDR, FmtS("This model (%s) has no nuggets, you need to call CreateNugget() on the model first\n", model->m_name.c_str()));
				model->m_dbg_flags = SetBits(model->m_dbg_flags, Model::EDbgFlags::WarnedNoRenderNuggets, true);
			}

			// Check the instance transform is valid
			auto& o2w = GetO2W(inst);
			auto flags = GetFlags(inst);
			if (!IsFinite(o2w) && !AllSet(model->m_dbg_flags, Model::EDbgFlags::WarnedInvalidTransform))
			{
				PR_INFO(PR_DBG_RDR, FmtS("This model (%s) has an invalid instance transform\n", model->m_name.c_str()));
				model->m_dbg_flags = SetBits(model->m_dbg_flags, Model::EDbgFlags::WarnedInvalidTransform, true);
			}
			if (!AllSet(flags, EInstFlag::NonAffine) && !IsAffine(o2w) && !AllSet(model->m_dbg_flags, Model::EDbgFlags::WarnedInvalidTransform))
			{
				PR_INFO(PR_DBG_RDR, FmtS("This model (%s) has a non-affine instance transform\n", model->m_name.c_str()));
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
