//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#pragma once
#include "pr/view3d-12/forward.h"
#include "pr/view3d-12/scene/scene_camera.h"
#include "pr/view3d-12/instance/instance.h"
#include "pr/view3d-12/lighting/light.h"
#include "pr/view3d-12/texture/texture_cube.h"
#include "pr/view3d-12/utility/wrappers.h"
#include "pr/view3d-12/utility/cmd_list.h"
#include "pr/view3d-12/utility/ray_cast.h"
#include "pr/view3d-12/utility/eventargs.h"

namespace pr::rdr12
{
	struct alignas(16) Scene
	{
		// Notes:
		//  - A scene is a view into the 3D world, containing a camera and collection of instances.
		//  - A scene contains an ordered collection of render steps. Each render step has it's own drawlist.
		//  - Multiple scenes can contribute to the content of a window.
		//    e.g.
		//      A window could have separate scenes for; world geometry, player graphics, HUD, rear view mirror, etc
		//      Typically, most applications only have one scene.
		//	
		// Rendering multi-pass models:
		//    To render a model that needs to be done in multiple passes, add additional nuggets to
		//    the model that overlap with existing nuggets but have different render states/shaders
		//    e.g. To render back faces first, then front faces: Add a nugget for the whole model
		//    with front face culling, then another nugget for the whole model with back face culling.

		// Fixed container of render steps. Doesn't really need to be fixed,
		// but non-fixed means we need the Allocator to construct it.
		// Conceptually, 'InstCont' should be an unordered_set, but using an array is way
		// faster due to the lack of allocations. This means RemoveInstance is O(n) however.
		using RenderStepCont = pr::vector<std::unique_ptr<RenderStep>, 16, true>;
		using InstCont = pr::vector<BaseInstance const*, 1024, false>;
		using RenderRayCastPtr = std::unique_ptr<RenderRayCast>;
		
		Window*          m_wnd;              // The controlling window
		SceneCamera      m_cam;              // Represents the camera properties used to project onto the screen
		Viewport         m_viewport;         // Represents the rectangular area on the back buffer that this scene covers (modify this variable if you want. Use the methods tho. Remember clip regions)
		InstCont         m_instances;        // Instances added to this scene for rendering.
		RenderStepCont   m_render_steps;     // The stages of rendering the scene
		RenderRayCastPtr m_raycast_immed;    // A ray cast render step for performing immediate hit tests
		RenderRayCastPtr m_raycast_async;    // A ray cast render step for performing async hit tests
		Light            m_global_light;     // The global light settings
		TextureCubePtr   m_global_envmap;    // A global environment map
		EFillMode        m_global_fill_mode; // A scene-wide fill mode override. EFillMode::Default means "use the model's default"
		PipeStates       m_pso;              // Scene-wide pipe state overrides
		AutoSub          m_eh_resize;        // RT resize event handler subscription

		Scene(Window& wnd, std::initializer_list<ERenderStep> rsteps = {ERenderStep::RenderForward}, SceneCamera const& cam = SceneCamera{});
		~Scene();

		// Renderer access
		ID3D12Device4* d3d() const;
		Renderer& rdr() const;
		Window& wnd() const;

		// Clear/Populate the drawlists for each render step.
		// Draw lists can be used in two ways, one is to clear the draw sets with each frame
		// and rebuild them from scratch (useful for scenes that change frequently).
		// The other is to NOT clear the draw sets and add/remove instances between frames.
		void ClearDrawlists();

		// Add an instance. The instance must be resident for the entire time that it is
		// in the scene, i.e. until 'RemoveInstance' or 'ClearDrawlist' is called.
		// This method will add the instance to all render steps for which the model has appropriate nuggets.
		// Instances can be added to render steps directly if finer control is needed
		template <InstanceType Inst>
		void AddInstance(Inst const& inst)
		{
			AddInstance(inst.m_base);
		}

		// Remove an instance from the draw list
		template <InstanceType Inst>
		void RemoveInstance(Inst const& inst)
		{
			RemoveInstance(inst.m_base);
		}

		// Raised just before the drawlist is sorted. Handlers should add/remove
		// instances from the scene, or add/remove render steps as required.
		EventHandler<Scene&, UpdateSceneArgs const&> OnUpdateScene;

		// Set the render steps to use for rendering the scene
		void SetRenderSteps(std::span<ERenderStep const> rsteps);

		// Access the render step by type
		template <RenderStepType TRenderStep>
		TRenderStep const* FindRStep() const
		{
			return static_cast<TRenderStep const*>(FindRStep(TRenderStep::Id));
		}
		template <RenderStepType TRenderStep>
		TRenderStep* FindRStep()
		{
			return static_cast<TRenderStep*>(FindRStep(TRenderStep::Id));
		}

		// Enable/Disable shadow casting
		void ShadowCasting(bool enable, int shadow_map_size);

		// Enable/disable ray tracing for this scene.
		void RayTracing(bool enable);

		// Get/Set the scene-wide fill mode default.
		EFillMode FillMode() const;
		void FillMode(EFillMode fill_mode);

		// Get/Set the scene-wide cull mode default.
		ECullMode CullMode() const;
		void CullMode(ECullMode cull_mode);

		// Perform an immediate hit test on the instances provided by coroutine 'instances'.
		// Successive calls to 'instances' should return instances to be hit tested. Return nullptr when complete.
		// Snap distance depends on the snap mode. If 'snap_mode' is 'Perspective', then 'snap_distance' is actually the ratio proportional to depth.
		std::future<void> HitTest(std::span<HitTestRay const> rays, RayCastInstancesCB instances, RayCastResultsOut out);

		// Perform an asynchronous hit test. Submits GPU work and returns immediately.
		// Results are reported via 'out' when the GPU completes (may be on a background thread).
		void HitTestAsync(std::span<HitTestRay const> rays);

		// Raised when async hit test results are ready.
		EventHandler<Scene&, std::span<HitTestResult const>> OnHitTestAsyncResults;

		// Render the scene, recording the command lists in 'frame'
		void Render(Frame& frame);
	
	private:

		friend struct Window;

		// Return a render step from this scene (if present)
		RenderStep const* FindRStep(ERenderStep id) const;
		RenderStep* FindRStep(ERenderStep id);

		// Add/Remove an instance from this scene
		void AddInstance(BaseInstance const& inst);
		void RemoveInstance(BaseInstance const& inst);

		// Resize the viewport on back buffer resize
		void HandleBackBufferSizeChanged(Window& wnd, BackBufferSizeChangedEventArgs const& evt);

		// Callback for hit test results
		void HitTestAsyncResults(std::span<HitTestResult const> results);
	};
}

