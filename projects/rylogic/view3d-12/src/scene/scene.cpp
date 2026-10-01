//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#include "pr/view3d-12/scene/scene.h"
#include "pr/view3d-12/main/window.h"
#include "pr/view3d-12/main/renderer.h"
#include "pr/view3d-12/instance/instance.h"
#include "pr/view3d-12/model/model.h"
#include "pr/view3d-12/render/render_step.h"
#include "pr/view3d-12/ray_tracing/ray_tracing_model.h"
#include "pr/view3d-12/ray_tracing/render_ray_tracing.h"
#include "pr/view3d-12/texture/texture_cube.h"
#include "pr/view3d-12/utility/eventargs.h"
#include "view3d-12/src/render/render_forward.h"
#include "view3d-12/src/render/render_smap.h"
#include "view3d-12/src/render/render_raycast.h"

namespace pr::rdr12
{
	// Reject incompatible resident geometry before changing a scene's render steps.
	static void ValidateRayTracingInstances(Scene::InstCont const& instances)
	{
		// Use the same source policy as instance admission and BLAS construction.
		for (auto const* inst : instances)
		{
			// Model-less instances do not contribute geometry.
			auto const& model = GetModel(*inst);
			if (model != nullptr)
				ValidateRayTracingGeometrySource(*model.get());
		}
	}

	// Make a scene
	Scene::Scene(Window& wnd, std::initializer_list<ERenderStep> rsteps, SceneCamera const& cam)
		: m_wnd(&wnd)
		, m_cam(cam)
		, m_viewport(wnd.BackBufferSize())
		, m_instances()
		, m_gsync_render_steps()
		, m_render_steps()
		, m_gsync_immed(wnd.d3d())
		, m_raycast_immed()
		, m_gsync_async(wnd.d3d())
		, m_raycast_async()
		, m_lights()
		, m_ambient(0xFF808080U)
		, m_global_envmap()
		, m_global_fill_mode(EFillMode::Default)
		, m_pso()
		, m_ray_tracing_props()
		, m_eh_resize()
		, m_far_clip_fade()
		, m_shadow_settings()
		, m_frame_lights()
		, m_resolved_lights()
		, m_dropped_lights()
		, m_post_effects(*wnd.m_rdr)
	{
		// Initialise the scene camera to match the full window
		auto bb_size = m_wnd->BackBufferSize();
		if (Any(bb_size != iv2::Zero()))
			m_cam.Aspect(1.0f * bb_size.x / bb_size.y);

		// Scenes start with a single default directional light
		m_lights.push_back(Light{});

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

	// Return the forward world fade settings for this scene.
	FarClipFadeProps Scene::FarClipFadeProperties() const
	{
		return m_far_clip_fade;
	}

	// Validate the complete range before replacing the current scene settings.
	void Scene::FarClipFadeProperties(FarClipFadeProps props)
	{
		props.Validate();
		if (props.m_enabled)
		{
			props.DepthRange(m_cam.ClipPlanes(false).y);
			if (auto* forward = FindRStep<RenderForward>())
				forward->ValidateFarClipFade();
		}

		// Failed material validation leaves the current option unchanged.
		m_far_clip_fade = props;
	}

	// Access the screen-space effects applied to this scene's output.
	PostProcessing const& Scene::PostEffects() const
	{
		return m_post_effects;
	}
	PostProcessing& Scene::PostEffects()
	{
		return m_post_effects;
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
		m_frame_lights.resize(0);
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
		// Reject incompatible geometry before publishing it to any drawlist. Rebuilt scenes perform this admission check each frame.
		if (FindRStep<RenderRayTracing>() != nullptr)
		{
			// Nested and shared instances must satisfy the receiving scene's active render contract.
			auto const& model = GetModel(inst);
			if (model != nullptr)
				ValidateRayTracingGeometrySource(*model.get());
		}

		// Debug checks of the instance transform. These depend only on the instance, so run them once here rather than in each render step.
		// Only print debug messages here, so that debug behaviour matches release behaviour. Checks stop after the first warning per model.
		#if PR_DBG_RDR
		if (auto const& model = GetModel(inst); model != nullptr && !AllSet(model->m_dbg_flags, Model::EDbgFlags::WarnedInvalidTransform))
		{
			auto const& o2w = GetO2W(inst);
			if (!IsFinite(o2w))
			{
				PR_INFO(PR_DBG_RDR, std::format("This model ({}) has an invalid instance transform\n", model->m_name));
				model->m_dbg_flags = SetBits(model->m_dbg_flags, Model::EDbgFlags::WarnedInvalidTransform, true);
			}
			else if (!AllSet(GetFlags(inst), EInstFlag::NonAffine) && !IsAffine(o2w))
			{
				PR_INFO(PR_DBG_RDR, std::format("This model ({}) has a non-affine instance transform\n", model->m_name));
				model->m_dbg_flags = SetBits(model->m_dbg_flags, Model::EDbgFlags::WarnedInvalidTransform, true);
			}
		}
		#endif

		// Publish only after source eligibility is established.
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
		// Reject unsupported geometry before discarding the current raster steps.
		if (std::find(rsteps.begin(), rsteps.end(), ERenderStep::RayTracing) != rsteps.end())
			ValidateRayTracingInstances(m_instances);

		// Replace only after validation succeeds. Finish and destroy old steps before releasing their fence storage.
		m_render_steps.clear();
		m_gsync_render_steps.clear();

		for (auto rs : rsteps)
		{
			switch (rs)
			{
				case ERenderStep::RenderForward: m_render_steps.emplace_back(new RenderForward(*this)); break;
				case ERenderStep::ShadowMap: throw std::runtime_error("The shadow map render step is managed by the scene's shadow settings and lights");
				case ERenderStep::RayCast:
				{
					// This step submits independently even when invoked during a frame. Duplicate entries need separate
					// reservation domains; nodes stay stable and survive complete step destruction on replacement/unwind.
					auto& gsync = m_gsync_render_steps.emplace_back(d3d());
					m_render_steps.push_back(std::make_unique<RenderRayCast>(*this, gsync, std::bind(&Scene::HitTestAsyncResults, this, _1)));
					break;
				}
				case ERenderStep::RayTracing:    m_render_steps.emplace_back(new RenderRayTracing(*this)); break;
				default: throw std::runtime_error("Unknown render step");
			}
		}
	}

	// Get/Set the scene-wide shadow settings
	ShadowSettings const& Scene::Shadows() const
	{
		return m_shadow_settings;
	}
	void Scene::Shadows(ShadowSettings const& settings)
	{
		// Reject settings the shadow atlas cannot represent
		auto is_pow2 = [](int x) { return x > 0 && (x & (x - 1)) == 0; };
		if (!is_pow2(settings.m_atlas_size) || settings.m_atlas_size > D3D12_REQ_TEXTURE2D_U_OR_V_DIMENSION)
			throw std::invalid_argument("Shadow atlas size must be a power of two no larger than the maximum texture size");
		if (settings.m_directional_resolution <= 0 || settings.m_spot_resolution <= 0 || settings.m_point_resolution <= 0)
			throw std::invalid_argument("Shadow view resolutions must be positive");
		if (settings.m_max_shadow_lights < 0 || settings.m_depth_bias < 0 || settings.m_slope_bias < 0 || settings.m_normal_bias < 0)
			throw std::invalid_argument("Shadow light count and biases must not be negative");
		if (settings.m_cascade_count < 1 || settings.m_cascade_count > MaxShadowCascades)
			throw std::invalid_argument(std::format("Shadow cascade count must be in [1, {}]", MaxShadowCascades));
		if (!(settings.m_shadow_distance >= 0) || !(settings.m_cascade_split_blend >= 0 && settings.m_cascade_split_blend <= 1))
			throw std::invalid_argument("Shadow distance must not be negative, and the cascade split blend must be in [0,1]");
		if (settings.m_filter_size != 5 && settings.m_filter_size != 7)
			throw std::invalid_argument("Shadow filter size must be 5 or 7");

		m_shadow_settings = settings;
	}

	// Add a world space light that shades the current frame only
	void Scene::AddFrameLight(Light const& light)
	{
		m_frame_lights.push_back(light);
	}

	// The world space lights that shade the current frame
	std::span<Light const> Scene::ResolvedLights() const
	{
		return m_resolved_lights;
	}

	// The number of on lights that did not shade the last frame
	int Scene::DroppedLightCount() const
	{
		return m_dropped_lights;
	}

	// Enable/disable ray tracing without rebuilding the existing raster render steps.
	void Scene::RayTracing(bool enable)
	{
		// Keep existing raster steps alive when toggling the ray-tracing pass.
		if (enable && FindRStep<RenderRayTracing>() == nullptr)
		{
			// A failed enable must leave the previous render-step configuration intact.
			ValidateRayTracingInstances(m_instances);
			m_render_steps.emplace_back(new RenderRayTracing(*this));
		}
		if (!enable && FindRStep<RenderRayTracing>() != nullptr)
		{
			// Remove only the ray-tracing pass.
			pr::erase_if(m_render_steps, [](auto& rs) { return rs->m_step_id == ERenderStep::RayTracing; });
		}
	}

	// Get/Set the ray tracing render settings for this scene.
	RayTracingProps Scene::RayTracingProperties() const
	{
		return m_ray_tracing_props;
	}
	void Scene::RayTracingProperties(RayTracingProps props)
	{
		switch (props.m_features)
		{
			case ERayTracingFeature::None:
			case ERayTracingFeature::Reflections:
			case ERayTracingFeature::Caustics:
			case ERayTracingFeature::All:
			{
				props.Clamp();
				m_ray_tracing_props = props;
				break;
			}
			default:
			{
				throw std::runtime_error("Unknown ray tracing feature flags");
			}
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
			m_raycast_immed.reset(new RenderRayCast(*this, m_gsync_immed, {}));
			
		auto& rs = *m_raycast_immed.get();
		// Drop this call's transient instances on both successful submission and failed recording.
		auto clear_drawlist = Scope<void>([&rs] { rs.ClearDrawlist(); });

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
		return rs.ExecuteImmediate(out);
	}

	// Perform an asynchronous hit test. Submits GPU work and returns immediately.
	void Scene::HitTestAsync(std::span<HitTestRay const> rays)
	{
		if (rays.empty())
			return;

		// Lazy create the async ray cast render step
		if (m_raycast_async == nullptr)
			m_raycast_async.reset(new RenderRayCast(*this, m_gsync_async, {}));

		auto& rs = *m_raycast_async.get();
		// Clearing transient instances does not discard submitted work or its pending readback handles.
		auto clear_drawlist = Scope<void>([&rs] { rs.ClearDrawlist(); });

		// Set the rays to cast
		rs.SetRays(rays, [](auto) { return true; });

		// Populate the draw list with the instances that have been added to this render step.
		// Typically, only instances that are visible to hit tests should have been added.
		for (auto& inst : m_instances)
			rs.AddInstance(*inst);

		// Submit to GPU and return immediately
		rs.ExecuteAsync(std::bind(&Scene::HitTestAsyncResults, this, _1));
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

		// Resolve the lights for this frame before any render step reads them
		m_dropped_lights = ResolveLights(m_lights, m_frame_lights, m_cam.CameraToWorld(), m_resolved_lights);

		// Add the shadow map step when a resolved light casts shadows, and remove it when none do. It runs first so the forward pass can read the atlas.
		{
			auto casts = std::any_of(m_resolved_lights.begin(), m_resolved_lights.end(), [](Light const& light) { return light.CastsShadow(); });
			auto want = casts && m_shadow_settings.m_max_shadow_lights > 0;
			auto* smap = FindRStep<RenderSmap>();
			if (want && smap == nullptr)
			{
				auto& step = *m_render_steps.emplace(std::begin(m_render_steps), new RenderSmap(*this));
				for (auto const* inst : m_instances)
					step->AddInstance(*inst);
			}
			if (!want && smap != nullptr)
			{
				// Earlier frames may still use the step's atlas and command allocators
				wnd().m_gsync.Wait();
				pr::erase_if(m_render_steps, [](auto& rs) { return rs->m_step_id == ERenderStep::ShadowMap; });
			}
		}

		// Allow render steps to do frame setup before any step starts recording its render commands.
		for (auto& rs : m_render_steps)
			rs->Prepare(frame);

		// Invoke each render step in order
		for (auto& rs : m_render_steps)
			rs->Execute(frame);

		// Post-process the composited scene. These commands run after all scene output, so recording order does not matter.
		m_post_effects.Render(frame, *this);
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
