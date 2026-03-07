//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#include "pr/view3d-12/scene/scene.h"
#include "pr/view3d-12/main/window.h"
#include "pr/view3d-12/main/renderer.h"
#include "pr/view3d-12/render/render_step.h"
#include "pr/view3d-12/texture/texture_cube.h"
#include "pr/view3d-12/utility/eventargs.h"
#include "view3d-12/src/render/render_forward.h"
#include "view3d-12/src/render/render_smap.h"
#include "view3d-12/src/render/render_raycast.h"

namespace pr::rdr12
{
	// Make a scene
	Scene::Scene(Window& wnd, std::initializer_list<ERenderStep> rsteps, SceneCamera const& cam)
		: m_wnd(&wnd)
		, m_cam(cam)
		, m_viewport(wnd.BackBufferSize())
		, m_instances()
		, m_render_steps()
		, m_raycast_immed()
		, m_raycast_async()
		, m_global_light()
		, m_global_envmap()
		, m_global_fill_mode(EFillMode::Default)
		, m_eh_resize()
	{
		// Initialise the scene camera to match the full window
		auto bb_size = m_wnd->BackBufferSize();
		if (bb_size != iv2::Zero())
			m_cam.Aspect(1.0f * bb_size.x / bb_size.y);

		// Set the render steps for the scene
		SetRenderSteps({ rsteps.begin(), rsteps.size() });

		//// Set default scene render states
		//m_rsb = RSBlock::SolidCullBack();

		//// Use line antialiasing if multi-sampling is enabled
		//if (wnd.m_multisamp.Count != 1)
		//	m_rsb.Set(ERS::MultisampleEnable, TRUE);

		// Sign up for back buffer resize events
		m_eh_resize = wnd.m_rdr->BackBufferSizeChanged += std::bind(&Scene::HandleBackBufferSizeChanged, this, _1, _2);
	}
	Scene::~Scene()
	{
		SetRenderSteps({});
	}

	// Access the renderer
	ID3D12Device4* Scene::d3d() const
	{
		return rdr().d3d();
	}
	Renderer& Scene::rdr() const
	{
		return wnd().rdr();
	}
	Window& Scene::wnd() const
	{
		return *m_wnd;
	}

	// Reset the draw list for each render step
	void Scene::ClearDrawlists()
	{
		m_instances.clear();
		for (auto& rs : m_render_steps)
			rs->ClearDrawlist();
	}

	// Return a render step from this scene (if present)
	RenderStep const* Scene::FindRStep(ERenderStep id) const
	{
		for (auto& step : m_render_steps)
		{
			if (step->m_step_id != id) continue;
			return step.get();
		}
		return nullptr;
	}
	RenderStep* Scene::FindRStep(ERenderStep id)
	{
		for (auto& step : m_render_steps)
		{
			if (step->m_step_id != id) continue;
			return step.get();
		}
		return nullptr;
	}

	// Add an instance. The instance must be resident for the entire time that it is
	// in the draw list, i.e. until 'RemoveInstance' or 'ClearDrawlist' is called.
	// This method will add the instance to all render steps for which the model has appropriate nuggets.
	// Instances can be added to render steps directly if finer control is needed
	void Scene::AddInstance(BaseInstance const& inst)
	{
		m_instances.push_back(&inst);
		for (auto& rs : m_render_steps)
			rs->AddInstance(inst);
	}

	// Remove an instance from the scene
	void Scene::RemoveInstance(BaseInstance const& inst)
	{
		// Remove from our collection
		auto iter = pr::find(m_instances, &inst);
		if (iter != std::end(m_instances))
			m_instances.erase_fast(iter);

		// Remove from each render step
		for (auto& rs : m_render_steps)
			rs->RemoveInstance(inst);
	}

	// Set the render steps to use for rendering the scene
	void Scene::SetRenderSteps(std::span<ERenderStep const> rsteps)
	{
		m_render_steps.clear();

		for (auto rs : rsteps)
		{
			switch (rs)
			{
				case ERenderStep::RenderForward: m_render_steps.emplace_back(new RenderForward(*this)); break;
				case ERenderStep::ShadowMap:     m_render_steps.emplace_back(new RenderSmap(*this, m_global_light)); break;
				case ERenderStep::RayCast:       m_render_steps.emplace_back(new RenderRayCast(*this, std::bind(&Scene::HitTestAsyncResults, this, _1))); break;
				default: throw std::runtime_error("Unknown render step");
			}
		}
	}

	// Enable/Disable shadow casting
	void Scene::ShadowCasting(bool enable, int shadow_map_size)
	{
		if (enable && FindRStep<RenderSmap>() == nullptr)
		{
			m_render_steps.emplace(std::begin(m_render_steps), new RenderSmap(*this, m_global_light, shadow_map_size));
		}
		if (!enable && FindRStep<RenderSmap>() != nullptr)
		{
			pr::erase_if(m_render_steps, [](auto& rs) { return rs->m_step_id == ERenderStep::ShadowMap; });
		}
	}

	// Get/Set the scene-wide fill mode default.
	EFillMode Scene::FillMode() const
	{
		return m_global_fill_mode;
	}
	void Scene::FillMode(EFillMode fill_mode)
	{
		m_global_fill_mode = fill_mode;
		switch (m_global_fill_mode)
		{
			case EFillMode::Default:
			case EFillMode::Points:
			case EFillMode::SolidWire:
			{
				m_pso.Clear<EPipeState::FillMode>();
				break;
			}
			case EFillMode::Solid:
			{
				m_pso.Set<EPipeState::FillMode>(D3D12_FILL_MODE_SOLID);
				break;
			}
			case EFillMode::Wireframe:
			{
				m_pso.Set<EPipeState::FillMode>(D3D12_FILL_MODE_WIREFRAME);
				break;
			}
			default:
			{
				throw std::runtime_error("Unsupported fill mode");
			}
		}
	}

	// Get/Set the scene-wide cull mode default.
	ECullMode Scene::CullMode() const
	{
		auto cull_mode = m_pso.Find<EPipeState::CullMode>();
		return cull_mode != nullptr ? s_cast<ECullMode>(*cull_mode) : ECullMode::Default;
	}
	void Scene::CullMode(ECullMode cull_mode)
	{
		if (cull_mode != ECullMode::Default)
			m_pso.Set<EPipeState::CullMode>(s_cast<D3D12_CULL_MODE>(cull_mode));
		else
			m_pso.Clear<EPipeState::CullMode>();
	}

	// Perform an immediate hit test
	std::future<void> Scene::HitTest(std::span<HitTestRay const> rays, RayCastInstancesCB instances, RayCastResultsOut out)
	{
		// Notes:
		//  - The immediate ray cast should be completely separate from the continuous ray cast.
		//    It should be possible to have both used within a single frame.
		//  - 'snap_mode' defines the features that the ray can hit. It shouldn't be zero.
		if (rays.empty())
			return {};

		// Lazy create the ray cast render step
		if (m_raycast_immed == nullptr)
			m_raycast_immed.reset(new RenderRayCast(*this, {}));
			
		auto& rs = *m_raycast_immed.get();

		// Set the rays to cast
		rs.SetRays(rays, [=](auto) { return true; });

		// Populate the draw list with the provided instances, or the instances added to the scene
		if (instances != nullptr)
		{
			for (BaseInstance const* inst; (inst = instances()) != nullptr;)
				rs.AddInstance(*inst);
		}
		else
		{
			for (auto& inst : m_instances)
				rs.AddInstance(*inst);
		}

		// Run the hit test
		auto result = rs.ExecuteImmediate(out);

		// Reset ready for next time
		rs.ClearDrawlist();

		return result;
	}

	// Perform an asynchronous hit test. Submits GPU work and returns immediately.
	void Scene::HitTestAsync(std::span<HitTestRay const> rays)
	{
		if (rays.empty())
			return;

		// Lazy create the async ray cast render step
		if (m_raycast_async == nullptr)
			m_raycast_async.reset(new RenderRayCast(*this, {}));

		auto& rs = *m_raycast_async.get();

		// Set the rays to cast
		rs.SetRays(rays, [](auto) { return true; });

		// Populate the draw list with the instances that have been added to this render step.
		// Typically, only instances that are visible to hit tests should have been added.
		for (auto& inst : m_instances)
			rs.AddInstance(*inst);

		// Submit to GPU and return immediately
		rs.ExecuteAsync(std::bind(&Scene::HitTestAsyncResults, this, _1));

		// Reset the draw list ready for next time
		rs.ClearDrawlist();
	}

	// Render the scene, recording the command lists in 'frame'
	void Scene::Render(Frame& frame)
	{
		// Notes:
		//  - Start rendering 'scene'. Remember, this is only recording commands into command lists so "drawing" on a back buffer doesn't
		//    actually happen until 'Present' is called (which executes the command lists). This means a HUD scene can render to 'swap_chain_bb'
		//    at the same time as a main view scene renders to 'msaa_bb'. Present composites the scene by executing the msaa command lists,
		//    then resolving the msaa render target into the swap chain back buffer, then executing the swap chain command lists.
		//  - 'rs->Execute(frame)' could start a background thread and return immediately. It should add it's not-yet-closed command lists
		//    to the frame from the main thread before starting.

		// Make sure the scene is up to date
		OnUpdateScene(*this, { frame.m_prepare, frame.m_upload });

		// Invoke each render step in order
		for (auto& rs : m_render_steps)
			rs->Execute(frame);
	}

	// Resize the viewport on back buffer resize
	void Scene::HandleBackBufferSizeChanged(Window& wnd, BackBufferSizeChangedEventArgs const& args)
	{
		if (args.m_done && &wnd == m_wnd)
		{
			// Update the viewport to match the new back buffer area.
			// Use Set() rather than directly assigning Width/Height so that the
			// scissor rect and ScreenW/ScreenH are also updated. Without this,
			// D3D12's RSSetScissorRects clips rendering to the old bounds.
			m_viewport.Set(args.m_area);
		}
	}

	// Callback for hit test results
	void Scene::HitTestAsyncResults(std::span<HitTestResult const> results)
	{
		OnHitTestAsyncResults(*this, results);
	}
}