//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#include "view3d-12/src/dll/v3d_window.h"
#include "view3d-12/src/dll/v3d_flight_camera.h"
#include "pr/view3d-12/ldraw/ldraw_object.h"
#include "pr/view3d-12/ldraw/ldraw_gizmo.h"
#include "pr/view3d-12/ldraw/ldraw_reader_text.h"
#include "pr/view3d-12/ldraw/ldraw_ui_script_editor.h"
#include "pr/view3d-12/ldraw/ldraw_ui_object_manager.h"
#include "pr/view3d-12/ldraw/ldraw_ui_measure_tool.h"
#include "pr/view3d-12/ldraw/ldraw_ui_angle_tool.h"
#include "pr/view3d-12/ldraw/ldraw.h"
#include "pr/view3d-12/main/renderer.h"
#include "pr/view3d-12/main/settings.h"
#include "pr/view3d-12/shaders/shader.h"
#include "pr/view3d-12/shaders/shader_point_sprites.h"
#include "pr/view3d-12/ray_tracing/render_ray_tracing.h"
#include "pr/view3d-12/resource/resource_factory.h"
#include "pr/view3d-12/texture/texture_2d.h"
#include "pr/view3d-12/texture/texture_cube.h"
#include "pr/view3d-12/texture/texture_desc.h"
#include "pr/view3d-12/utility/conversion.h"

namespace pr::rdr12
{
	using namespace ::pr::compute;

	// Default window construction settings
	WndSettings ToWndSettings(HWND hwnd, RdrSettings const& rsettings, view3d::WindowOptions const& opts)
	{
		return WndSettings(hwnd, true, rsettings)
			.DefaultOutput()
			.BackgroundColour(opts.m_background_colour)
			.AllowAltEnter(opts.m_allow_alt_enter != 0)
			.XrSupport(opts.m_xr_support != 0)
			.MutliSampling(opts.m_multisampling)
			.Name(opts.m_dbg_name ? std::string_view(opts.m_dbg_name) : std::string_view{});
	}

	// Validate a window pointer
	void Validate(V3dWindow const* window)
	{
		if (window == nullptr)
			throw std::runtime_error("Window pointer is null");
	}

	// View3d Window ****************************
	V3dWindow::V3dWindow(Renderer& rdr, HWND hwnd, view3d::WindowOptions const& opts)
		: m_rdr(&rdr)
		, m_hwnd(hwnd)
		, m_wnd(*m_rdr, ToWndSettings(hwnd, m_rdr->Settings(), opts))
		, m_scene(m_wnd)
		, m_objects()
		, m_gizmos()
		, m_guids()
		, m_focus_point()
		, m_origin_point()
		, m_bbox_model()
		, m_selection_box()
		, m_visible_objects()
		, m_settings()
		, m_anim_data()
		, m_ht_rays()
		, m_ht_results()
		, m_bbox_scene(BBox::Reset())
		, m_main_thread_id(std::this_thread::get_id())
		, m_ui_providers()
		, m_invalidated(false)
		, m_ht_invalidated(false)
		, m_ui_lighting()
		, m_ui_object_manager()
		, m_ui_script_editor()
		, m_ui_measure_tool()
		, m_ui_angle_tool()
		, ReportError()
		, OnSettingsChanged()
		, OnInvalidated()
		, OnRendering()
		, OnSceneChanged()
		, OnAnimationEvent()
	{
		try
		{
			// Notes:
			// - Don't observe the Context sources store for changes. The context handles this for us.
			ReportError += opts.m_error_cb;

			// Set the initial aspect ratio
			auto rt_area = m_wnd.BackBufferSize();
			if (LengthSq(rt_area) != 0)
				m_scene.m_cam.Aspect(rt_area.x / float(rt_area.y));

			// The scene starts with a single camera-relative directional light and grey ambient light
			auto& main_light = m_scene.m_lights[0];
			main_light.m_type = ELight::Directional;
			main_light.m_diffuse = Colour32(0xFFFFFFFFU);
			main_light.m_specular = Colour32(0xFF101010U);
			main_light.m_specular_power = 64.0f;
			main_light.m_intensity = 1.0f;
			main_light.m_direction = -v4::ZAxis();
			main_light.m_on = true;
			main_light.m_cam_relative = true;
			m_scene.m_ambient = Colour32(0xFF808080U);

			// Forward async hit test results
			m_eh_hittests = m_scene.OnHitTestAsyncResults += [this](Scene&, std::span<HitTestResult const> results)
			{
				// Buffer the type converted results
				m_ht_results.resize(0);
				m_ht_results.reserve(results.size());

				for (auto const& hit : results)
				{
					// Check that 'hit.m_instance' is a valid instance in this scene.
					auto ldr_obj = cast<ldraw::LdrObject>(hit.m_instance);

					// Not an object in this scene, keep looking
					if (!Has(ldr_obj, true))
						continue;

					// Not visible to hit tests, keep looking
					if (AllSet(ldr_obj->Flags(), ldraw::ELdrFlags::HitTestExclude))
						continue;

					// Build the result
					m_ht_results.push_back(To<view3d::HitTestResult>(hit));
				}

				// Notify subscribers with the first hit
				OnHitTestAsyncResults(this, m_ht_results.data(), isize(m_ht_results));
			};

			// Create the stock models
			CreateStockObjects();
		}
		catch (...)
		{
			this->~V3dWindow();
			throw;
		}
	}
	V3dWindow::~V3dWindow()
	{
		AnimControl(view3d::EAnimCommand::Stop);

		// Invalidate the copied providers before notifying satellites that outlived their documented detach point.
		auto providers = std::move(m_ui_providers);
		m_ui_providers.clear();
		for (auto const& provider : providers)
		{
			// Notify in attach order so each satellite releases its own window state.
			if (provider.m_detached != nullptr)
				provider.m_detached(provider.m_context);
		}

		// Disable the flight camera before tearing down the renderer / scene.
		// FlightCameraController owns a renderer poll-callback registration and
		// a Raw Input registration that must be released while everything is alive.
		m_flight_cam.reset();

		// Release the environment map probe and off-screen capture resources while the renderer is still alive
		m_envmap_probe.reset();
		m_envmap_capture.reset();

		m_hwnd = 0;
		m_scene.RemoveInstance(m_focus_point);
		m_scene.RemoveInstance(m_origin_point);
		m_scene.RemoveInstance(m_bbox_model);
		m_scene.RemoveInstance(m_selection_box);
	}

	// Renderer access
	Renderer& V3dWindow::rdr() const
	{
		return *m_rdr;
	}

	// Get/Set the settings
	std::string_view V3dWindow::Settings() const
	{
		// Ambient light, then one entry per scene light in order
		std::stringstream out;
		out << "*Ambient {" << std::hex << m_scene.m_ambient.argb << std::dec << "}\n";
		for (auto const& light : m_scene.m_lights)
			out << "*Light {\n" << light.Settings() << "}\n";

		m_settings = out.str();
		return m_settings.c_str();
	}
	void V3dWindow::Settings(std::string_view settings)
	{
		// Parse the settings. Saved lighting (which always includes the ambient light) replaces all scene lights.
		auto has_lighting = false;
		auto lights = LightList{};
		auto ambient = m_scene.m_ambient;
		mem_istream<char> src(settings);
		rdr12::ldraw::TextReader reader(src, {});
		for (int kw; reader.NextKeyword(kw);) switch (kw)
		{
			case rdr12::ldraw::HashI("Ambient"):
			{
				ambient = reader.Int<uint32_t>(16);
				has_lighting = true;
				break;
			}
			case rdr12::ldraw::HashI("Light"):
			{
				auto desc = reader.String<std::string>();
				lights.push_back(Light{});
				lights.back().Settings(desc);
				has_lighting = true;
				break;
			}
		}

		// Apply the parsed lighting
		if (!has_lighting)
			return;

		m_scene.m_lights = lights;
		m_scene.m_ambient = ambient;
		OnSettingsChanged(this, view3d::ESettings::Lighting_All);
		Invalidate();
	}

	// Get the current ray tracing capability and per-window enable state.
	view3d::RayTracingInfo V3dWindow::RayTracingInfo() const
	{
		auto const& support = rdr().RayTracing();
		return {
			.m_requested = support.Requested(),
			.m_hardware_supported = support.HardwareSupported(),
			.m_available = support.Available(),
			.m_enabled = RayTracingEnabled(),
			.m_tier = 
				support.m_tier == D3D12_RAYTRACING_TIER_NOT_SUPPORTED ? view3d::ERayTracingTier::NotSupported :
				support.m_tier == D3D12_RAYTRACING_TIER_1_0 ? view3d::ERayTracingTier::Tier1_0 :
				support.m_tier == D3D12_RAYTRACING_TIER_1_1 ? view3d::ERayTracingTier::Tier1_1 :
				view3d::ERayTracingTier::Unknown,
		};
	}

	// Return true when this window currently has the ray tracing render step.
	bool V3dWindow::RayTracingEnabled() const
	{
		return m_scene.FindRStep<RenderRayTracing>() != nullptr;
	}

	// Enable or disable the ray tracing render step for this window.
	void V3dWindow::RayTracingEnabled(bool enable)
	{
		if (RayTracingEnabled() == enable)
			return;

		if (enable && !rdr().RayTracing().Available())
			throw std::runtime_error(std::format("Ray tracing is not available on this renderer. Device tier: {}", rdr().RayTracing().TierName()));

		// Keep the existing raster steps alive when toggling RT. Rebuilding the list here destroys command-list owners
		// during settings application and also drops any per-step draw-list state that the raster path already owns.
		m_scene.RayTracing(enable);
		OnSettingsChanged(this, view3d::ESettings::Rendering_RayTracing);
		Invalidate();
	}

	// Get/Set the ray tracing render settings for this window.
	RayTracingProps V3dWindow::RayTracingProperties() const
	{
		return m_scene.RayTracingProperties();
	}
	void V3dWindow::RayTracingProperties(RayTracingProps props)
	{
		auto const before = RayTracingProperties();
		props.Clamp();
		if (props == before)
			return;

		m_scene.RayTracingProperties(props);
		OnSettingsChanged(this, view3d::ESettings::Rendering_RayTracing);
		Invalidate();
	}

	// Return the fade settings from their scene authority.
	FarClipFadeProps V3dWindow::FarClipFadeProperties() const
	{
		return m_scene.FarClipFadeProperties();
	}

	// Publish a validated rendering option change and request a new frame.
	void V3dWindow::FarClipFadeProperties(FarClipFadeProps props)
	{
		props.Validate();
		if (props == FarClipFadeProperties())
			return;

		// The scene setter validates camera-dependent constraints before changing its state.
		m_scene.FarClipFadeProperties(props);
		OnSettingsChanged(this, view3d::ESettings::Rendering_FarClipFade);
		Invalidate();
	}

	// Return the scene-owned underwater post effect settings.
	UnderwaterProps V3dWindow::PostEffectUnderwater() const
	{
		return m_scene.PostEffects().Underwater();
	}

	// Publish a validated post effect change and request a new frame.
	void V3dWindow::PostEffectUnderwater(UnderwaterProps const& props)
	{
		props.Validate();
		if (props == PostEffectUnderwater())
			return;

		m_scene.PostEffects().Underwater(props);
		OnSettingsChanged(this, view3d::ESettings::Rendering_PostEffects);
		Invalidate();
	}

	// The DPI of the monitor that this window is displayed on
	v2 V3dWindow::Dpi() const
	{
		return m_wnd.Dpi();
	}

	// Get/Set the back buffer size
	iv2 V3dWindow::BackBufferSize() const
	{
		return m_wnd.BackBufferSize();
	}
	void V3dWindow::BackBufferSize(iv2 sz, bool force_recreate)
	{
		if (sz.x < 0) sz.x = 0;
		if (sz.y < 0) sz.y = 0;

		// Before resize, the old aspect is: Aspect0 = scale * Width0 / Height0
		// After resize, the new aspect is: Aspect1 = scale * Width1 / Height1

		// Save the current camera aspect ratio
		auto old_size = m_wnd.BackBufferSize();
		auto old_aspect = m_scene.m_cam.Aspect();

		// Resize the render target
		m_wnd.BackBufferSize(sz, force_recreate);

		// Adjust the camera aspect ratio to preserve it
		auto new_size = m_wnd.BackBufferSize();
		auto new_aspect = (new_size.x == 0 || new_size.y == 0) ? 1.0f : new_size.x / float(new_size.y);
		auto scale = old_size.x * old_size.y != 0 ? old_aspect * old_size.y / float(old_size.x) : 1.0f;
		auto aspect = scale * new_aspect;
		m_scene.m_cam.Aspect(aspect);
	}

	// Get/Set whether the swap chain owns an output in DXGI exclusive fullscreen mode
	bool V3dWindow::FullScreen() const
	{
		return m_wnd.FullScreen();
	}
	void V3dWindow::FullScreen(bool fullscreen)
	{
		m_wnd.FullScreen(fullscreen);
	}

	// Get/Set the scene viewport
	view3d::Viewport V3dWindow::Viewport() const
	{
		auto& vp = m_scene.m_viewport;
		return view3d::Viewport{
			.m_x = vp.TopLeftX,
			.m_y = vp.TopLeftY,
			.m_width = vp.Width,
			.m_height = vp.Height,
			.m_min_depth = vp.MinDepth,
			.m_max_depth = vp.MaxDepth,
			.m_screen_w = vp.ScreenW,
			.m_screen_h = vp.ScreenH,
		};
	}
	void V3dWindow::Viewport(view3d::Viewport const& vp)
	{
		m_scene.m_viewport.Set(vp.m_x, vp.m_y, vp.m_width, vp.m_height, vp.m_screen_w, vp.m_screen_h, vp.m_min_depth, vp.m_max_depth);
		OnSettingsChanged(this, view3d::ESettings::Scene_Viewport);
	}

	// Enumerate the object collection GUIDs associated with this window
	void V3dWindow::EnumGuids(view3d::EnumGuidsCB enum_guids_cb)
	{
		assert(std::this_thread::get_id() == m_main_thread_id);
		for (auto& guid : m_guids)
		{
			if (enum_guids_cb(guid)) continue;
			break;
		}
	}

	// Enumerate the objects associated with this window
	void V3dWindow::EnumObjects(view3d::EnumObjectsCB enum_objects_cb)
	{
		assert(std::this_thread::get_id() == m_main_thread_id);
		for (auto& object : m_objects)
		{
			if (enum_objects_cb(object)) continue;
			break;
		}
	}
	void V3dWindow::EnumObjects(view3d::EnumObjectsCB enum_objects_cb, view3d::GuidPredCB pred)
	{
		assert(std::this_thread::get_id() == m_main_thread_id);
		for (auto& object : m_objects)
		{
			if (!pred(object->m_context_id)) continue;
			if (enum_objects_cb(object)) continue;
			break;
		}
	}

	// Return true if 'object' is part of this scene
	bool V3dWindow::Has(ldraw::LdrObject const* object, bool search_children) const
	{
		assert(std::this_thread::get_id() == m_main_thread_id);

		// Search (recursively) for a match for 'object'.
		auto name = search_children ? "" : nullptr;
		for (auto& obj : m_objects)
		{
			// 'Apply' returns false if a quick out occurred (i.e. 'object' was found)
			if (obj->Apply([=](auto* ob) { return ob != object; }, name)) continue;
			return true;
		}
		return false;
	}
	bool V3dWindow::Has(ldraw::LdrGizmo const* gizmo) const
	{
		assert(std::this_thread::get_id() == m_main_thread_id);
		for (auto& giz : m_gizmos)
		{
			if (giz != gizmo) continue;
			return true;
		}
		return false;
	}

	// Return the number of objects or object groups in this scene
	int V3dWindow::ObjectCount() const
	{
		assert(std::this_thread::get_id() == m_main_thread_id);
		return s_cast<int>(m_objects.size());
	}
	int V3dWindow::GizmoCount() const
	{
		return s_cast<int>(m_gizmos.size());
	}
	int V3dWindow::GuidCount() const
	{
		assert(std::this_thread::get_id() == m_main_thread_id);
		return s_cast<int>(m_guids.size());
	}

	// Return the bounding box of objects in this scene
	BBox V3dWindow::SceneBounds(view3d::ESceneBounds bounds, int except_count, GUID const* except) const
	{
		assert(std::this_thread::get_id() == m_main_thread_id);
		std::span<GUID const> except_arr(except, except_count);
		auto pred = [](ldraw::LdrObject const& ob)
		{
			return !AllSet(ob.Flags(), ldraw::ELdrFlags::SceneBoundsExclude);
		};

		BBox bbox;
		switch (bounds)
		{
			case view3d::ESceneBounds::All:
			{
				// Update the scene bounding box if out of date
				if (m_bbox_scene == BBox::Reset())
				{
					bbox = BBox::Reset();
					for (auto& obj : m_objects)
					{
						if (!pred(*obj)) continue;
						if (pr::contains(except_arr, obj->m_context_id)) continue;
						Grow(bbox, obj->BBoxWS(ldraw::EBBoxFlags::IncludeChildren, pred));
					}
					m_bbox_scene = bbox;
				}
				bbox = m_bbox_scene;
				break;
			}
			case view3d::ESceneBounds::Selected:
			{
				bbox = BBox::Reset();
				auto add_selected = [&](this auto&& self, ldraw::LdrObject const& o) -> void
				{
					if (!pred(o)) return;

					// A Selected node contributes its full subtree bbox, then we don't recurse —
					// nested selected descendants are already covered by the parent's bbox.
					if (AllSet(o.Flags(), ldraw::ELdrFlags::Selected))
					{
						Grow(bbox, o.BBoxWS(ldraw::EBBoxFlags::IncludeChildren, pred));
						return;
					}
					for (auto& child : o.m_child)
						self(*child.m_ptr);
				};
				for (auto& obj : m_objects)
				{
					if (pr::contains(except_arr, obj->m_context_id)) continue;
					add_selected(*obj);
				}
				break;
			}
			case view3d::ESceneBounds::Visible:
			{
				bbox = BBox::Reset();
				for (auto& obj : m_objects)
				{
					if (!pred(*obj)) continue;
					if (AllSet(obj->Flags(), ldraw::ELdrFlags::Hidden)) continue;
					if (pr::contains(except_arr, obj->m_context_id)) continue;
					Grow(bbox, obj->BBoxWS(ldraw::EBBoxFlags::IncludeChildren, pred));
				}
				break;
			}
			default:
			{
				assert(!"Unknown scene bounds type");
				bbox = BBox::Unit();
				break;
			}
		}
		return bbox.valid() ? bbox : BBox::Unit();
	}

	// Add/Remove an object to this window
	void V3dWindow::Add(ldraw::LdrObject* object)
	{
		assert(std::this_thread::get_id() == m_main_thread_id);
		auto iter = m_objects.find(object);
		if (iter == end(m_objects))
		{
			m_objects.insert(iter, object);
			m_guids.insert(object->m_context_id);
			ObjectContainerChanged(view3d::ESceneChanged::ObjectsAdded, { &object->m_context_id, 1 }, object);
		}
	}
	void V3dWindow::Remove(ldraw::LdrObject* object)
	{
		// 'm_guids' may be out of date now, but it doesn't really matter.
		// It's used to track the groups of objects added to the window.
		// A group with zero members is still a group.
		assert(std::this_thread::get_id() == m_main_thread_id);
		auto count = m_objects.size();

		// Remove the object
		m_objects.erase(object);

		// Notify if changed
		if (m_objects.size() != count)
			ObjectContainerChanged(view3d::ESceneChanged::ObjectsRemoved, { &object->m_context_id, 1 }, object);
	}

	// Add/Remove a gizmo to this window
	void V3dWindow::Add(ldraw::LdrGizmo* gizmo)
	{
		assert(std::this_thread::get_id() == m_main_thread_id);
		auto iter = m_gizmos.find(gizmo);
		if (iter == std::end(m_gizmos))
		{
			m_gizmos.insert(iter, gizmo);
			ObjectContainerChanged(view3d::ESceneChanged::GizmoAdded, {}, nullptr); // todo, overload and pass 'gizmo' out
		}
	}
	void V3dWindow::Remove(ldraw::LdrGizmo* gizmo)
	{
		m_gizmos.erase(gizmo);
		ObjectContainerChanged(view3d::ESceneChanged::GizmoRemoved, {}, nullptr);
	}

	// Add/Remove all objects to this window with the given context ids (or not with)
	void V3dWindow::Add(ldraw::SourceCont const& sources, view3d::GuidPredCB pred)
	{
		assert(std::this_thread::get_id() == m_main_thread_id);

		pr::vector<Guid> new_guids;
		for (auto& srcs : sources)
		{
			auto& src = srcs.second;
			if (!pred(src->m_context_id))
				continue;

			// Add objects from this source
			new_guids.push_back(src->m_context_id);
			for (auto& obj : src->m_output.m_objects)
				m_objects.insert(obj.get());

			// TODO: Camera settings are now applied via the command system (see ProcessCommand in ldraw_sources.cpp).
			// Camera commands need to be added to ECommand and processed there.
		}

		// Add the guids, even if no objects were added. Guids are used by refreshes to load objects belonging to the same context
		if (!new_guids.empty())
		{
			m_guids.insert(std::begin(new_guids), std::end(new_guids));
			ObjectContainerChanged(view3d::ESceneChanged::ObjectsAdded, new_guids, nullptr);
		}
	}
	void V3dWindow::Remove(view3d::GuidPredCB pred, bool keep_context_ids)
	{
		assert(std::this_thread::get_id() == m_main_thread_id);

		// Create a set of ids to remove
		GuidSet removed;
		for (auto& id : m_guids)
		{
			if (!pred(id)) continue;
			removed.insert(id);
		}

		if (!removed.empty())
		{
			// Remove objects in the 'remove' set
			auto old_count = m_objects.size();
			erase_if(m_objects, [&](auto* obj) { return removed.count(obj->m_context_id); });

			// Remove context ids
			if (!keep_context_ids)
			{
				for (auto& id : removed)
					m_guids.erase(id);
			}

			// Notify if changed
			if (m_objects.size() != old_count)
			{
				vector<Guid> guids(std::begin(removed), std::end(removed));
				ObjectContainerChanged(view3d::ESceneChanged::ObjectsRemoved, guids, nullptr);
			}

			// Refresh the window
			Invalidate();
		}
	}

	// Remove all objects from this scene
	void V3dWindow::RemoveAllObjects()
	{
		assert(std::this_thread::get_id() == m_main_thread_id);

		// Make a copy of the GUIDs
		vector<GUID> context_ids(std::begin(m_guids), std::end(m_guids));

		// Remove the objects and GUIDs
		m_objects.clear();
		m_guids.clear();

		// Notify that the scene has changed
		ObjectContainerChanged(view3d::ESceneChanged::ObjectsRemoved, context_ids, nullptr);
	}

	// Render this window into whatever render target is currently set
	void V3dWindow::Render()
	{
		// Notes:
		// - Don't be tempted to call 'Validate()' at the start of Render so that objects
		//   added to the scene during the render re-invalidate. Instead defer the invalidate
		//   to the next windows event.

		assert(std::this_thread::get_id() == m_main_thread_id);

		// Reset the draw list
		m_scene.ClearDrawlists();

		// If the viewport is empty, nothing to draw.
		// This is could be an error, but setting the viewport to empty could
		// also be a way to stop rendering when a window is minimised, etc..
		if (LengthSq(m_scene.m_viewport.AsIRect().Size()) == 0)
		{
			Validate();
			return;
		}

		// Notify of a render about to happen
		OnRendering(this);

		/*
		// Set the view and projection matrices. Do this before adding objects to the
		// scene as they do last minute transform adjustments based on the camera position.
		auto& cam = m_scene.m_cam;
		m_scene.SetView(cam);
		cam.m_moved = false;
		*/

		// Position and scale the focus point and origin point
		if (AnySet(m_visible_objects, EStockObject::FocusPoint | EStockObject::OriginPoint))
		{
			// Draw the point with perspective or orthographic projection based on the camera settings,
			// but with an aspect ratio matching the viewport regardless of the camera's aspect ratio.
			float const screen_fraction = 0.05f;
			auto aspect_v = float(m_scene.m_viewport.Width) / float(m_scene.m_viewport.Height);

			// Get the scene camera
			auto& scene_cam = m_scene.m_cam;
			auto fd = scene_cam.FocusDist();

			// Create a camera with the same aspect as the viewport
			auto v_camera = m_scene.m_cam;
			v_camera.Aspect(aspect_v);

			// Get the scaling factors from 'm_camera' to 'v_camera'
			auto viewarea_c = scene_cam.ViewRectAtDistance(fd);
			auto viewarea_v = v_camera.ViewRectAtDistance(fd);

			if (AllSet(m_visible_objects, EStockObject::FocusPoint))
			{
				// Scale the camera space X,Y coordinates
				// Note: this cannot be added as a matrix to 'i2w' or 'c2s' because we're
				// only scaling the instance position, not the whole instance geometry
				auto pt_cs = scene_cam.WorldToCamera() * scene_cam.FocusPoint();
				pt_cs.x *= viewarea_v.x / viewarea_c.x;
				pt_cs.y *= viewarea_v.y / viewarea_c.y;
				auto pt_ws = scene_cam.CameraToWorld() * pt_cs;

				auto sz = m_focus_point.m_size * screen_fraction * abs(pt_cs.z);
				m_focus_point.m_i2w = m4x4::Scale(sz, sz, sz, pt_ws);
				m_focus_point.m_c2s = v_camera.CameraToScreen();
				m_scene.AddInstance(m_focus_point);
			}
			if (AllSet(m_visible_objects, EStockObject::OriginPoint))
			{
				// Scale the camera space X,Y coordinates
				auto pt_cs = scene_cam.WorldToCamera() * v4::Origin();
				pt_cs.x *= viewarea_v.x / viewarea_c.x;
				pt_cs.y *= viewarea_v.y / viewarea_c.y;
				auto pt_ws = scene_cam.CameraToWorld() * pt_cs;

				auto sz = m_origin_point.m_size * screen_fraction * abs(pt_cs.z);
				m_origin_point.m_i2w = m4x4::Scale(sz, sz, sz, pt_ws);
				m_origin_point.m_c2s = v_camera.CameraToScreen();
				m_scene.AddInstance(m_origin_point);
			}
		}

		// Selection box
		if (AnySet(m_visible_objects, EStockObject::SelectionBox))
		{
			// Transform is updated by the user or by a call to SetSelectionBoxToSelected()
			// 'm_selection_box.m_i2w.pos.w' is zero when there is no selection.
			// Update the selection box if necessary
			SelectionBoxFitToSelected();
			if (m_selection_box.m_i2w.pos.w != 0)
				m_scene.AddInstance(m_selection_box);
		}

		// Get the animation clock time
		auto anim_time = (float)m_anim_data.m_clock.load().count();
		assert(IsFinite(anim_time)); (void)anim_time;

		// Add objects from the window to the scene
		for (auto& obj : m_objects)
		{
			obj->AddToScene(m_scene);

			// Only show bounding boxes for things that contribute to the scene bounds.
			if (m_wnd.m_diag.m_bboxes_visible && !AllSet(obj->Flags(), ldraw::ELdrFlags::SceneBoundsExclude))
				obj->AddBBoxToScene(m_scene);
		}

		// Add gizmos from the window to the scene
		for (auto& giz : m_gizmos)
		{
			giz->AddToScene(m_scene);
		}

		// Add the measure tool objects if the window is visible
		if (m_ui_measure_tool != nullptr && m_ui_measure_tool->Visible() && m_ui_measure_tool->Gfx())
			m_ui_measure_tool->Gfx()->AddToScene(m_scene);

		// Add the angle tool objects if the window is visible
		if (m_ui_angle_tool != nullptr && m_ui_angle_tool->Visible() && m_ui_angle_tool->Gfx())
			m_ui_angle_tool->Gfx()->AddToScene(m_scene);

		// Render the scene
		auto& frame = m_wnd.NewFrame();

		// Capture UI setup commands
		RecordUIProvider(view3d::ui::EPass::Prepare, frame);

		// Render the scene into the multi-sampled scene target
		m_scene.Render(frame);

		// Depth-tested world UI is scene-adjacent: it draws into the multi-sampled scene target
		// while the scene depth buffer is still bound and writable, so scene geometry occludes it.
		RecordUIProvider(view3d::ui::EPass::DepthTested, frame);

		// Composite is an independently recorded phase, so restore its documented boundary state
		// before a later command list begins the optional final overlay.
		if (frame.bb_post().m_render_target != nullptr)
		{
			BarrierBatch bb(frame.m_composite);
			bb.Transition(frame.bb_post().m_render_target.get(), D3D12_RESOURCE_STATE_PRESENT);
			bb.Commit();
		}

		// Produce the single-sample depth copy after every scene depth writer has run, then record
		// the two world overlay passes that consume it. Occlusion-faded work is recorded before
		// plain world overlays so unoccluded world UI always composites on top.
		if (!m_ui_providers.empty())
			m_wnd.RecordDepthResolve(frame.m_depth_resolve, frame.bb_main());

		// Draw world-anchored roots that fade per pixel when the resolved scene depth occludes them.
		RecordUIProvider(view3d::ui::EPass::OcclusionFaded, frame);

		// Draw unoccluded world-anchored roots above scene output but below screen-space UI.
		RecordUIProvider(view3d::ui::EPass::Overlay, frame);

		// Draw screen-space roots as the final back-buffer writer before presentation.
		RecordUIProvider(view3d::ui::EPass::FinalOverlay, frame);

		// Present the frame to the swap chain
		m_wnd.Present(frame);

		// No longer invalidated
		Validate();
	}

	// Append a provider so it draws after every provider attached before it.
	view3d::ui::EHostStatus V3dWindow::UIProviderAttach(view3d::ui::Provider const& provider)
	{
		using namespace view3d::ui;

		// Reject invalid calls, and reject a context that is already attached so detach by context stays unambiguous.
		if (std::this_thread::get_id() != m_main_thread_id)
			return EHostStatus::WrongThread;
		if (provider.m_header.m_size < sizeof(Provider) || provider.m_header.m_version != HostStructVersion)
			return EHostStatus::InvalidStruct;
		if (provider.m_context == nullptr || provider.m_record == nullptr)
			return EHostStatus::InvalidArgument;
		if (std::ranges::any_of(m_ui_providers, [&](Provider const& p) { return p.m_context == provider.m_context; }))
			return EHostStatus::AlreadyAttached;

		m_ui_providers.push_back(provider);
		return EHostStatus::Success;
	}

	// Detach the provider whose context matches the supplied identity, keeping the order of the others.
	view3d::ui::EHostStatus V3dWindow::UIProviderDetach(void* provider_context)
	{
		using namespace view3d::ui;

		// Find the provider by context identity.
		if (std::this_thread::get_id() != m_main_thread_id)
			return EHostStatus::WrongThread;
		if (provider_context == nullptr)
			return EHostStatus::InvalidArgument;

		auto iter = std::ranges::find_if(m_ui_providers, [&](Provider const& p) { return p.m_context == provider_context; });
		if (iter == m_ui_providers.end())
			return EHostStatus::NotAttached;

		m_ui_providers.erase(iter);
		return EHostStatus::Success;
	}

	// Record one pass for every attached provider without transferring command-list or target ownership.
	void V3dWindow::RecordUIProvider(view3d::ui::EPass pass_id, Frame& frame)
	{
		using namespace view3d::ui;

		if (m_ui_providers.empty())
			return;

		auto& bb_main = frame.bb_main();
		auto& bb_post = frame.bb_post();
		if (bb_post.m_render_target == nullptr)
			return;

		// Each pass names one host command list recorded at a fixed point in the frame. Depth-tested
		// world UI is scene-adjacent so it draws into the multi-sampled scene target while the scene
		// depth buffer is still live; the two world overlay passes draw into the resolved swap target
		// after alpha compositing; screen UI stays last in the true final overlay.
		auto& cmd_list = [&]() -> GfxCmdList&
		{
			switch (pass_id)
			{
				case EPass::Prepare: return frame.m_prepare;
				case EPass::DepthTested: return frame.m_world_depth;
				case EPass::OcclusionFaded: return frame.m_world_overlay;
				case EPass::Overlay: return frame.m_world_overlay;
				case EPass::FinalOverlay: return frame.m_final_overlay;
				default: throw std::runtime_error(std::format("Unsupported View3DUI provider pass {}", static_cast<std::uint32_t>(pass_id)));
			}
		}();

		// Depth-tested work targets the multi-sampled scene buffer; every other drawing pass targets
		// the resolved swap target.
		auto scene_target = pass_id == EPass::DepthTested;
		auto const& target_bb = scene_target ? bb_main : bb_post;

		// Only the passes that draw into the swap target need it made writable; the MSAA scene target
		// is already in its render-target boundary state when the scene-adjacent pass runs.
		auto brackets_swap_target = pass_id == EPass::OcclusionFaded || pass_id == EPass::Overlay || pass_id == EPass::FinalOverlay;
		if (brackets_swap_target)
		{
			// The host command list owns the transition that brackets a provider draw. The provider
			// receives the target and RTV but is never allowed to transition host resources.
			BarrierBatch bb(cmd_list);
			bb.Transition(bb_post.m_render_target.get(), D3D12_RESOURCE_STATE_RENDER_TARGET);
			bb.Commit();
		}

		// The single-sample depth copy is offered only to the pass that samples it, and only after
		// the host has recorded the resolve that fills it.
		auto resolved_depth = ResolvedDepth{ .m_resource = nullptr, .m_srv_format = DXGI_FORMAT_UNKNOWN, .m_width = 0, .m_height = 0 };
		if (pass_id == EPass::OcclusionFaded && bb_main.m_depth_stencil != nullptr)
		{
			auto size = bb_main.rt_size();
			resolved_depth = ResolvedDepth{
				.m_resource = m_wnd.ResolvedDepth(bb_main),
				.m_srv_format = m_wnd.ResolvedDepthSrvFormat(),
				.m_width = s_cast<std::uint32_t>(size.x),
				.m_height = s_cast<std::uint32_t>(size.y),
			};
		}

		auto size = target_bb.rt_size();
		auto dpi = Dpi();
		auto viewport = static_cast<D3D12_VIEWPORT const&>(m_scene.m_viewport);
		auto scissor = m_scene.m_viewport.m_clip[0];
		auto pass = Pass{
			.m_header = {sizeof(Pass), HostStructVersion},
			.m_pass = pass_id,
			.m_reserved = 0,
			.m_command_list = cmd_list.get(),
			.m_colour_target = target_bb.m_render_target.get(),
			.m_depth_target = bb_main.m_depth_stencil.get(),
			.m_rtv = target_bb.m_rtv,
			.m_dsv = scene_target ? bb_main.m_dsv : D3D12_CPU_DESCRIPTOR_HANDLE{},
			.m_width = s_cast<std::uint32_t>(size.x),
			.m_height = s_cast<std::uint32_t>(size.y),
			.m_colour_format = ::pr::compute::ToSRGB(m_wnd.m_rt_props.Format),
			.m_depth_format = m_wnd.m_ds_props.Format,
			.m_sample_count = target_bb.m_multisamp.Count,
			.m_sample_quality = target_bb.m_multisamp.Quality,
			.m_frame_number = s_cast<std::uint64_t>(m_wnd.FrameNumber()),
			.m_dpi_x = dpi.x,
			.m_dpi_y = dpi.y,
			.m_viewport = viewport,
			.m_scissor = scissor,
			.m_camera = UIProviderCamera(),
			.m_resolved_depth = resolved_depth,
		};
		// Record providers in attach order inside one target bracket. Copy the list so a provider cannot invalidate the iteration.
		// Each provider sets its own pipeline state and descriptor heaps, so no state is shared between providers.
		auto status = EHostStatus::Success;
		auto providers = m_ui_providers;
		for (auto const& provider : providers)
		{
			// Stop at the first failure so the error identifies one provider.
			status = provider.m_record(provider.m_context, &pass);
			if (status != EHostStatus::Success)
				break;
		}

		// Restore the swap target in the same command list that made it writable, so the bridge
		// phase remains self-contained even when no earlier composite writer ran this frame.
		if (brackets_swap_target)
		{
			BarrierBatch bb(cmd_list);
			bb.Transition(bb_post.m_render_target.get(), D3D12_RESOURCE_STATE_PRESENT);
			bb.Commit();
		}

		if (status != EHostStatus::Success)
			throw std::runtime_error(std::format("View3DUI provider failed with status {}", static_cast<std::int32_t>(status)));
	}

	// Snapshot the scene camera in the bridge's right-handed convention.
	view3d::ui::Camera V3dWindow::UIProviderCamera() const
	{
		auto const& cam = m_scene.m_cam;
		auto c2w = cam.CameraToWorld();

		// pr::Camera's camera-space z axis points backwards along the look direction, so the look
		// direction handed to the provider is -z. Orthographic height is derived from the same
		// focus distance and field of view the projection matrix is built from.
		auto forward = -c2w.z;
		auto fov_y = s_cast<float>(cam.FovY());
		auto ortho_height = s_cast<float>(2.0 * cam.FocusDist() * std::tan(cam.FovY() * 0.5));
		return view3d::ui::Camera{
			.m_position = { c2w.pos.x, c2w.pos.y, c2w.pos.z },
			.m_right = { c2w.x.x, c2w.x.y, c2w.x.z },
			.m_up = { c2w.y.x, c2w.y.y, c2w.y.z },
			.m_forward = { forward.x, forward.y, forward.z },
			.m_near_plane = s_cast<float>(cam.Near(false)),
			.m_far_plane = s_cast<float>(cam.Far(false)),
			.m_fov_y_rad = fov_y,
			.m_ortho_height = ortho_height,
			.m_orthographic = cam.Orthographic() ? 1U : 0U,
			.m_valid = 1U,
		};
	}
	
	// Wait for any previous frames to complete rendering within the GPU
	void V3dWindow::GSyncWait() const
	{
		m_wnd.m_gsync.Wait();
	}

	// Replace the swap chain buffers
	void V3dWindow::CustomSwapChain(std::span<BackBuffer> back_buffers)
	{
		m_wnd.CustomSwapChain(back_buffers);
	}
	void V3dWindow::CustomSwapChain(std::span<Texture2D*> back_buffers)
	{
		m_wnd.CustomSwapChain(back_buffers);
	}

	// Get/Set the render target for this window
	rdr12::BackBuffer const& V3dWindow::RenderTarget() const
	{
		return m_wnd.m_msaa_bb;
	}
	rdr12::BackBuffer& V3dWindow::RenderTarget()
	{
		return const_call(RenderTarget());
	}

	// Borrow the final colour target, including compositing and final overlays.
	rdr12::BackBuffer& V3dWindow::FrameOutput()
	{
		return m_wnd.FrameOutput();
	}

	// Call InvalidateRect on the HWND associated with this window
	void V3dWindow::InvalidateRect(RECT const* rect, bool erase)
	{
		if (m_hwnd != nullptr)
			::InvalidateRect(m_hwnd, rect, erase);

		if (!m_invalidated)
			OnInvalidated(this);

		// The window becomes validated again when 'Present()' or 'Validate()' is called.
		m_invalidated = true;
	}
	void V3dWindow::Invalidate(bool erase)
	{
		InvalidateRect(nullptr, erase);
	}

	// Clear the invalidated state for the window
	void V3dWindow::Validate()
	{
		m_invalidated = false;
	}
		
	// Reset the scene camera, using it's current forward and up directions, to view all objects in the scene
	void V3dWindow::ResetView()
	{
		auto c2w = m_scene.m_cam.CameraToWorld();
		ResetView(-c2w.z, c2w.y);
	}

	// Reset the scene camera to view all objects in the scene
	void V3dWindow::ResetView(v4 forward, v4 up, float dist, bool preserve_aspect, bool commit)
	{
		auto bbox = SceneBounds(view3d::ESceneBounds::All, 0, nullptr);
		ResetView(bbox, forward, up, dist, preserve_aspect, commit);
	}

	// Reset the camera to view a bbox
	void V3dWindow::ResetView(BBox const& bbox, v4 forward, v4 up, float dist, bool preserve_aspect, bool commit)
	{
		m_scene.m_cam.View(bbox, forward, up, dist, preserve_aspect, commit);

		auto settings = view3d::ESettings::Camera_Position;
		if (dist != 0) settings|= view3d::ESettings::Camera_FocusDist;
		if (!preserve_aspect) settings |= view3d::ESettings::Camera_Aspect;
		OnSettingsChanged(this, settings);
		Invalidate();
	}

	// General mouse navigation
	// 'ss_pos' is the mouse pointer position in 'window's screen space
	// 'nav_op' is the navigation type (typically Rotate=LButton, Translate=RButton)
	// 'nav_start_or_end' should be TRUE on mouse down/up events, FALSE for mouse move events
	// void OnMouseDown(UINT nFlags, CPoint point) { View3D_MouseNavigate(win, point, nav_op, TRUE); }
	// void OnMouseMove(UINT nFlags, CPoint point) { View3D_MouseNavigate(win, point, nav_op, FALSE); } if 'nav_op' is None, this will have no effect
	// void OnMouseUp  (UINT nFlags, CPoint point) { View3D_MouseNavigate(win, point, 0, TRUE); }
	// BOOL OnMouseWheel(UINT nFlags, short zDelta, CPoint) { if (nFlags == 0) View3D_MouseNavigateZ(win, 0, 0, zDelta / 120.0f); return TRUE; }
	bool V3dWindow::MouseNavigate(v2 ss_point, camera::ENavOp nav_op, bool nav_start_or_end)
	{
		auto nss_point = m_scene.m_viewport.SSPointToNSSPoint(ss_point);

		// This is true-ish. 'ss_pos' is allowed to be outside the window area which breaks this check
		//if (nss_point.x < -1.0 || nss_point.x > +1.0 || nss_point.y < -1.0 || nss_point.y > +1.0)
		//	throw std::runtime_error("Window viewport has not been set correctly. The ScreenW/H values should match the window size (not the viewport size)");

		auto refresh = false;
		auto gizmo_in_use = false;

		// Check any gizmos in the scene for interaction with the mouse
		for (auto& giz : m_gizmos)
		{
			refresh |= giz->MouseControl(m_scene.m_cam, nss_point, nav_op, nav_start_or_end);
			gizmo_in_use |= giz->m_manipulating;
			if (gizmo_in_use)
				break;
		}

		// If no gizmos are using the mouse, use standard mouse control
		if (!gizmo_in_use)
		{
			if (m_scene.m_cam.MouseControl(nss_point, nav_op, nav_start_or_end))
			{
				Invalidate();
				refresh |= true;
			}
		}

		return refresh;
	}
	bool V3dWindow::MouseNavigateZ(v2 ss_point, float delta, bool along_ray)
	{
		auto nss_point = m_scene.m_viewport.SSPointToNSSPoint(ss_point);

		auto refresh = false;
		auto gizmo_in_use = false;

		// Check any gizmos in the scene for interaction with the mouse
#if 0 // todo, gizmo mouse wheel behaviour
		for (auto& giz : m_gizmos)
		{
			refresh |= giz->MouseControlZ(m_scene.m_cam, nss_point, dist);
			gizmo_in_use |= giz->m_manipulating;
			if (gizmo_in_use)
				break;
		}
#endif

		// If no gizmos are using the mouse, use standard mouse control
		if (!gizmo_in_use)
		{
			if (m_scene.m_cam.MouseControlZ(nss_point, delta, along_ray))
			{
				Invalidate();
				refresh |= true;
			}
		}

		return refresh;
	}

	// Enable / disable the native flight-camera controller for this window.
	void V3dWindow::FlightCameraEnable(bool on)
	{
		assert(std::this_thread::get_id() == m_main_thread_id);

		if (on)
		{
			if (m_flight_cam == nullptr)
				m_flight_cam = std::make_unique<FlightCameraController>(*this);
			m_flight_cam->Enable(true);
		}
		else if (m_flight_cam != nullptr)
		{
			m_flight_cam->Enable(false);
		}
	}
	bool V3dWindow::FlightCameraIsEnabled() const
	{
		return m_flight_cam != nullptr && m_flight_cam->IsEnabled();
	}

	// Get/Set the window background colour
	Colour V3dWindow::BackgroundColour() const
	{
		return m_wnd.BkgdColour();
	}
	void V3dWindow::BackgroundColour(Colour colour)
	{
		if (BackgroundColour() == colour)
			return;

		m_wnd.BkgdColour(colour);
		OnSettingsChanged(this, view3d::ESettings::Scene_BackgroundColour);
		Invalidate();
	}
	
	// Get/Set the window fill mode
	EFillMode V3dWindow::FillMode() const
	{
		return m_scene.FillMode();
	}
	void V3dWindow::FillMode(EFillMode fill_mode)
	{
		if (FillMode() == fill_mode)
			return;

		m_scene.FillMode(fill_mode);
		OnSettingsChanged(this, view3d::ESettings::Scene_FillMode);
		Invalidate();
	}

	// Get/Set the window cull mode
	ECullMode V3dWindow::CullMode() const
	{
		return m_scene.CullMode();
	}
	void V3dWindow::CullMode(ECullMode cull_mode)
	{
		if (CullMode() == cull_mode)
			return;

		m_scene.CullMode(cull_mode);
		OnSettingsChanged(this, view3d::ESettings::Scene_CullMode);
		Invalidate();
	}

	// Enable/Disable orthographic projection
	bool V3dWindow::Orthographic() const
	{
		return m_scene.m_cam.Orthographic();
	}
	void V3dWindow::Orthographic(bool on)
	{
		if (Orthographic() == on)
			return;

		m_scene.m_cam.Orthographic(on);
		OnSettingsChanged(this, view3d::ESettings::Camera_Orthographic);
		Invalidate();
	}

	// Get/Set the distance to the camera focus point
	float V3dWindow::FocusDistance() const
	{
		return s_cast<float>(m_scene.m_cam.FocusDist());
	}
	void V3dWindow::FocusDistance(float dist)
	{
		if (FocusDistance() == dist)
			return;

		m_scene.m_cam.FocusDist(dist);
		m_scene.m_cam.Commit();

		OnSettingsChanged(this, view3d::ESettings::Camera_FocusDist);
		Invalidate();
	}

	// Get/Set the camera focus point position
	v4 V3dWindow::FocusPoint() const
	{
		return m_scene.m_cam.FocusPoint();
	}
	void V3dWindow::FocusPoint(v4 position)
	{
		if (All(FocusPoint() == position))
			return;

		m_scene.m_cam.FocusPoint(position);
		m_scene.m_cam.Commit();

		OnSettingsChanged(this, view3d::ESettings::Camera_FocusDist);
		Invalidate();
	}

	// Get/Set the camera focus point bounds
	BBox V3dWindow::FocusBounds() const
	{
		return m_scene.m_cam.FocusBounds();
	}
	void V3dWindow::FocusBounds(BBox bounds)
	{
		if (FocusBounds() == bounds)
			return;

		m_scene.m_cam.FocusBounds(bounds);
		m_scene.m_cam.Commit();

		OnSettingsChanged(this, view3d::ESettings::Camera_FocusDist);
		Invalidate();
	}

	// Get/Set the aspect ratio for the camera field of view
	float V3dWindow::Aspect() const
	{
		return s_cast<float>(m_scene.m_cam.Aspect());
	}
	void V3dWindow::Aspect(float aspect)
	{
		if (Aspect() == aspect)
			return;

		m_scene.m_cam.Aspect(aspect);

		OnSettingsChanged(this, view3d::ESettings::Camera_Aspect);
		Invalidate();
	}

	// Get/Set the camera field of view. null means don't change
	v2 V3dWindow::Fov() const
	{
		return v2(
			s_cast<float>(m_scene.m_cam.FovX()),
			s_cast<float>(m_scene.m_cam.FovY()));
	}
	void V3dWindow::Fov(v2 fov)
	{
		if (All(fov == Fov()))
			return;

		m_scene.m_cam.Fov(fov.x, fov.y);
		OnSettingsChanged(this, view3d::ESettings::Camera_Fov);
		Invalidate();
	}

	// Adjust the FocusDist, FovX, and FovY so that the average FOV equals 'fov'
	void V3dWindow::BalanceFov(float fov)
	{
		m_scene.m_cam.BalanceFov(fov);
		OnSettingsChanged(this, view3d::ESettings::Camera_FocusDist | view3d::ESettings::Camera_Fov);
		Invalidate();
	}

	// Get/Set (using fov and focus distance) the size of the perpendicular area visible to the camera at 'dist' (in world space). Use 'focus_dist != 0' to set a specific focus distance
	v2 V3dWindow::ViewRectAtDistance(float dist) const
	{
		return m_scene.m_cam.ViewRectAtDistance(dist);
	}
	void V3dWindow::ViewRectAtDistance(v2 rect, float focus_dist)
	{
		if (All(ViewRectAtDistance(focus_dist) == rect))
			return;

		m_scene.m_cam.ViewRectAtDistance(rect, focus_dist);

		OnSettingsChanged(this, view3d::ESettings::Camera_FocusDist | view3d::ESettings::Camera_Fov);
		Invalidate();
	}

	// Get/Set the near and far clip planes for the camera
	v2 V3dWindow::ClipPlanes(view3d::EClipPlanes flags) const
	{
		return m_scene.m_cam.ClipPlanes(AllSet(flags, view3d::EClipPlanes::CameraRelative));
	}
	void V3dWindow::ClipPlanes(float near_, float far_, view3d::EClipPlanes flags)
	{
		auto cp = ClipPlanes(flags);
		if (AllSet(flags, view3d::EClipPlanes::Near)) cp.x = near_;
		if (AllSet(flags, view3d::EClipPlanes::Far)) cp.y = far_;
		if (All(ClipPlanes(flags) == cp))
			return;

		m_scene.m_cam.ClipPlanes(cp.x, cp.y, AllSet(flags, view3d::EClipPlanes::CameraRelative));

		OnSettingsChanged(this, view3d::ESettings::Camera_ClipPlanes);
		Invalidate();
	}

	// Get/Set the scene camera lock mask
	camera::ELockMask V3dWindow::LockMask() const
	{
		return m_scene.m_cam.LockMask();
	}
	void V3dWindow::LockMask(camera::ELockMask mask)
	{
		if (LockMask() == mask)
			return;

		m_scene.m_cam.LockMask(mask);

		OnSettingsChanged(this, view3d::ESettings::Camera_LockMask);
		Invalidate();
	}

	// Get/Set the camera align axis
	v4 V3dWindow::AlignAxis() const
	{
		return m_scene.m_cam.Align();
	}
	void V3dWindow::AlignAxis(v4 axis)
	{
		if (All(AlignAxis() == axis))
			return;

		m_scene.m_cam.Align(axis);

		OnSettingsChanged(this, view3d::ESettings::Camera_AlignAxis);
		Invalidate();
	}
	
	// Reset to the default zoom
	void V3dWindow::ResetZoom()
	{
		auto z = Zoom();
		m_scene.m_cam.ResetZoom();
		if (Zoom() == z)
			return;

		OnSettingsChanged(this, view3d::ESettings::Camera_Fov);
		Invalidate();
	}
	
	// Get/Set the FOV zoom
	float V3dWindow::Zoom() const
	{
		return s_cast<float>(m_scene.m_cam.Zoom());
	}
	void V3dWindow::Zoom(float zoom)
	{
		if (Zoom() == zoom)
			return;

		m_scene.m_cam.Zoom(zoom, true);

		OnSettingsChanged(this, view3d::ESettings::Camera_Fov);
		Invalidate();
	}

	// The number of scene lights
	int V3dWindow::LightCount() const
	{
		return isize(m_scene.m_lights);
	}

	// Get/Set a scene light
	Light V3dWindow::SceneLight(int index) const
	{
		assert(index >= 0 && index < LightCount());
		return m_scene.m_lights[index];
	}
	void V3dWindow::SceneLight(int index, Light const& light)
	{
		// Report only the aspects of the light that changed
		assert(index >= 0 && index < LightCount());
		auto& prev = m_scene.m_lights[index];
		if (prev == light)
			return;

		auto settings = view3d::ESettings::Lighting;
		if (prev.m_type != light.m_type) settings |= view3d::ESettings::Lighting_Type;
		if (Any(prev.m_position != light.m_position)) settings |= view3d::ESettings::Lighting_Position;
		if (Any(prev.m_direction != light.m_direction)) settings |= view3d::ESettings::Lighting_Direction;
		if (prev.m_diffuse != light.m_diffuse) settings |= view3d::ESettings::Lighting_Colour;
		if (prev.m_specular != light.m_specular) settings |= view3d::ESettings::Lighting_Colour;
		if (prev.m_intensity != light.m_intensity) settings |= view3d::ESettings::Lighting_Colour;
		if (prev.m_specular_power != light.m_specular_power) settings |= view3d::ESettings::Lighting_Range;
		if (prev.m_range != light.m_range) settings |= view3d::ESettings::Lighting_Range;
		if (prev.m_falloff != light.m_falloff) settings |= view3d::ESettings::Lighting_Range;
		if (prev.m_inner_angle != light.m_inner_angle) settings |= view3d::ESettings::Lighting_Range;
		if (prev.m_outer_angle != light.m_outer_angle) settings |= view3d::ESettings::Lighting_Range;
		if (prev.m_cast_shadow != light.m_cast_shadow) settings |= view3d::ESettings::Lighting_Shadows;
		if (prev.m_cam_relative != light.m_cam_relative) settings |= view3d::ESettings::Lighting_Position | view3d::ESettings::Lighting_Direction;
		if (prev.m_on != light.m_on) settings |= view3d::ESettings::Lighting_All;

		prev = light;
		OnSettingsChanged(this, settings);
		Invalidate();
	}

	// Add/Remove scene lights
	int V3dWindow::AddLight(Light const& light)
	{
		m_scene.m_lights.push_back(light);
		OnSettingsChanged(this, view3d::ESettings::Lighting_All);
		Invalidate();
		return LightCount() - 1;
	}
	void V3dWindow::RemoveLight(int index)
	{
		assert(index >= 0 && index < LightCount());
		m_scene.m_lights.erase(m_scene.m_lights.begin() + index);
		OnSettingsChanged(this, view3d::ESettings::Lighting_All);
		Invalidate();
	}

	// Get/Set the scene-wide ambient light colour
	Colour32 V3dWindow::Ambient() const
	{
		return m_scene.m_ambient;
	}
	void V3dWindow::Ambient(Colour32 ambient)
	{
		if (m_scene.m_ambient == ambient)
			return;

		m_scene.m_ambient = ambient;
		OnSettingsChanged(this, view3d::ESettings::Lighting_Colour);
		Invalidate();
	}

	// Get/Set the scene-wide shadow settings
	ShadowSettings const& V3dWindow::Shadows() const
	{
		return m_scene.Shadows();
	}
	void V3dWindow::Shadows(ShadowSettings const& settings)
	{
		if (m_scene.Shadows() == settings)
			return;

		m_scene.Shadows(settings);
		OnSettingsChanged(this, view3d::ESettings::Lighting_Shadows);
		Invalidate();
	}

	// Get/Set the global environment map for this window
	TextureCube const* V3dWindow::EnvMap() const
	{
		return m_scene.m_global_envmap.get();
	}
	void V3dWindow::EnvMap(TextureCube* env_map)
	{
		// An explicit environment map replaces the probe's maps
		if (EnvMap() == env_map && m_envmap_probe == nullptr)
			return;

		m_envmap_probe = nullptr;
		m_scene.m_global_envmap = TextureCubePtr(env_map, true);
		m_scene.m_global_envmap_prev = nullptr;
		m_scene.m_global_envmap_blend = 1.0f;

		OnSettingsChanged(this, view3d::ESettings::Scene_EnvMap);
		Invalidate();
	}

	// Off-screen resources for rendering environment map faces. Creating these is far more expensive than a capture, so they are kept between captures.
	// Each face capture is submitted without waiting for the GPU. Work on the shared graphics queue runs in submission order, and every command list
	// starts from the resources' default states, so later submissions that reuse the target, the scratch texture, or a cube are correctly ordered.
	struct EnvMapCaptureResources
	{
		// The capture target is written through an sRGB view, so the stored bytes are sRGB encoded and can be copied directly into an sRGB cube
		static constexpr DXGI_FORMAT Format = DXGI_FORMAT_R8G8B8A8_UNORM;

		// The format of distance cubes. 16 bits avoid visible steps where reflections meet the captured surfaces.
		static constexpr DXGI_FORMAT DistanceFormat = DXGI_FORMAT_R16_UNORM;

		// Root parameters of the distance pass, see 'env_map_distance.hlsl'
		enum class EDistanceParam { Constants, Colour, Depth, Face, Distance };
		struct DistanceConstants
		{
			m4x4 s2c; // Screen to camera space for the face camera
			v4 info;  // x = face size in pixels, y = distance scale
		};

		ResourceFactory m_factory;          // Records the face copies and mip generation. It is kept so that captures do not wait for the GPU.
		int m_face_size;                    // Pixels per face edge
		Colour m_bkgd_colour;               // The clear colour, which is also the render target's optimised clear value
		Window m_wnd;                       // Single-sample off-screen window that renders the faces
		Texture2DPtr m_target;              // The window's render target
		D3DPtr<ID3D12Resource> m_scratch;   // Full-mip copy of the most recently rendered face
		D3DPtr<ID3D12Resource> m_scratch_distance; // Full-mip distances of the most recently rendered face
		UINT m_mips;                        // The number of mips in each face
		Scene m_scene;                      // The scene used to render the faces
		Window::GpuViewHeap m_view_heap;    // Shader-visible views for the distance pass. Retired by the capture window's frames.
		D3DPtr<ID3D12RootSignature> m_distance_sig; // Root signature of the distance pass
		D3DPtr<ID3D12PipelineState> m_distance_pso; // Pipeline state of the distance pass

		EnvMapCaptureResources(Renderer& rdr, int face_size, Colour bkgd_colour)
			: m_factory(rdr)
			, m_face_size(face_size)
			, m_bkgd_colour(bkgd_colour)
			, m_wnd(rdr, Settings(rdr, face_size, bkgd_colour))
			, m_target()
			, m_scratch()
			, m_scratch_distance()
			, m_mips()
			, m_scene(m_wnd)
			, m_view_heap(64, m_wnd.m_gsync)
			, m_distance_sig()
			, m_distance_pso()
		{
			// Create the pass that copies a face and stores each texel's distance in a separate texture
			auto device = rdr.D3DDevice();
			m_distance_sig = ::pr::compute::RootSig(::pr::compute::ERootSigFlags::ComputeOnly)
				.U32(hlsl::ECBufReg::b0, sizeof(DistanceConstants) / sizeof(uint32_t))
				.SRV(hlsl::ESRVReg::t0, 1, D3D12_SHADER_VISIBILITY_ALL, D3D12_DESCRIPTOR_RANGE_FLAG_DESCRIPTORS_VOLATILE)
				.SRV(hlsl::ESRVReg::t1, 1, D3D12_SHADER_VISIBILITY_ALL, D3D12_DESCRIPTOR_RANGE_FLAG_DESCRIPTORS_VOLATILE)
				.UAV(hlsl::EUAVReg::u0, 1, D3D12_SHADER_VISIBILITY_ALL, D3D12_DESCRIPTOR_RANGE_FLAG_DESCRIPTORS_VOLATILE)
				.UAV(hlsl::EUAVReg::u1, 1, D3D12_SHADER_VISIBILITY_ALL, D3D12_DESCRIPTOR_RANGE_FLAG_DESCRIPTORS_VOLATILE)
				.Create(device, "EnvMapDistanceSig");
			m_distance_pso = ::pr::compute::ComputePSO(m_distance_sig.get(), shader_code::env_map_distance_cs)
				.Create(device, "EnvMapDistancePSO");

			// Create the render target and make it the window's only back buffer
			auto target_desc = TextureDesc(AutoId, ResDesc::Tex2D(Image{face_size, face_size, nullptr, Format}, 1U, EUsage::RenderTarget).clear(m_wnd.m_rt_props));
			target_desc.rtv_format(ToSRGB(Format));
			m_target = m_factory.CreateTexture2D(target_desc);

			// Mip generation supports only single 2D textures, so each face is copied into a full-mip 2D texture before it is copied into the cube.
			// The texture allows unordered access so that mips are generated in place.
			m_scratch = m_factory.CreateResource(ResDesc::Tex2D(Image{face_size, face_size, nullptr, Format}, 0, EUsage::UnorderedAccess), "EnvMapCaptureFace");
			m_scratch_distance = m_factory.CreateResource(ResDesc::Tex2D(Image{face_size, face_size, nullptr, DistanceFormat}, 0, EUsage::UnorderedAccess), "EnvMapCaptureDistance");
			m_mips = m_scratch->GetDesc().MipLevels;
			m_factory.FlushToGpu(EGpuFlush::Block);

			auto target = BackBuffer(m_wnd, MultiSamp(1), m_target.get(), nullptr);
			m_wnd.CustomSwapChain(std::span{&target, 1});
		}
		EnvMapCaptureResources(EnvMapCaptureResources const&) = delete;
		EnvMapCaptureResources& operator=(EnvMapCaptureResources const&) = delete;

		// Window settings for a square, single-sample, off-screen target
		static WndSettings Settings(Renderer& rdr, int face_size, Colour bkgd_colour)
		{
			auto settings = WndSettings(nullptr, true, rdr.Settings()).Size(face_size, face_size);
			settings.m_mode.Format = Format;
			settings.m_multisamp = MultiSamp(1);
			settings.m_bkgd_colour = bkgd_colour;
			settings.m_name = "EnvMapCapture";
			return settings;
		}

		// The orientation of every captured cube. View3D cameras are right-handed, but the DX cube face layout is left-handed.
		// Faces are rendered in a world mirrored in Z, which makes the face cameras right-handed, and sampling applies the same mirror.
		static m4x4 CubeToWorld()
		{
			return m4x4{ v4::XAxis(), v4::YAxis(), -v4::ZAxis(), v4::Origin() };
		}

		// Return the capture resources for 'wnd', recreating them if the face size or the window's clear colour has changed
		static EnvMapCaptureResources& Get(V3dWindow& wnd, int face_size)
		{
			// Recreating waits for in-flight captures, because the factory's destructor flushes and blocks
			auto bkgd_colour = wnd.m_wnd.BkgdColour();
			auto& res = wnd.m_envmap_capture;
			if (res == nullptr || res->m_face_size != face_size || res->m_bkgd_colour != bkgd_colour)
			{
				res = nullptr;
				res = std::make_unique<EnvMapCaptureResources>(*wnd.m_rdr, face_size, bkgd_colour);
			}
			return *res;
		}

		// Throw unless 'cube' can receive captures, and return its face size
		static int ValidateCube(TextureCube const& cube)
		{
			// Captures copy sRGB encoded faces and every mip, so the cube must be a square RGBA8 sRGB cube with a full mip chain
			auto desc = cube.m_res->GetDesc();
			if (desc.Width != desc.Height || desc.DepthOrArraySize != 6 || desc.Format != ToSRGB(Format))
				throw std::runtime_error("Environment map capture requires a square RGBA8 sRGB cube map");

			auto face_size = s_cast<int>(desc.Width);
			auto mips = UINT(1);
			for (auto size = face_size; size > 1; size /= 2)
				++mips;

			if (desc.MipLevels != mips)
				throw std::runtime_error("Environment map capture requires a cube map with a full mip chain");

			return face_size;
		}

		// Render face 'face' of 'cube' from the cube's centre, using the lighting, render state, and objects of 'src'. The work is submitted to the GPU without waiting.
		// The caller sets the cube's orientation to 'CubeToWorld()', and its centre and distance scale. A positive distance scale stores distances in 'cube.m_distance',
		// which is created if needed.
		void CaptureFace(V3dWindow& src, TextureCube& cube, int face)
		{
			// Copy the lighting and render state from the source scene. The capture has no environment map, so it cannot reflect itself.
			// Objects flagged 'EnvMapCaptureExclude' are not rendered into the capture. The flag is per object; children do not inherit it.
			m_scene.m_inst_exclude = EInstFlag::EnvMapCaptureExclude;
			m_scene.m_sky_history = false; // Each face looks a different way, so the sky has no previous frame to blend with.
			m_scene.m_lights = src.m_scene.m_lights;
			m_scene.m_ambient = src.m_scene.m_ambient;
			m_scene.m_global_fill_mode = src.m_scene.m_global_fill_mode;
			m_scene.m_pso = src.m_scene.m_pso;
			m_scene.Shadows(src.m_scene.Shadows());
			m_scene.m_cam = src.m_scene.m_cam;
			m_scene.m_cam.Orthographic(false);
			m_scene.m_cam.Aspect(1.0);
			m_scene.m_cam.FovY(2.0 * std::atan(1.0));

			// Look along the mirrored major axis. Each face lists its DX major axis and the world directions of increasing U and V. Image up is the opposite of increasing V.
			struct FaceAxes { v4 major, u, v; };
			static FaceAxes const face_axes[] =
			{
				{ +v4::XAxis(), -v4::ZAxis(), -v4::YAxis() },
				{ -v4::XAxis(), +v4::ZAxis(), -v4::YAxis() },
				{ +v4::YAxis(), +v4::XAxis(), +v4::ZAxis() },
				{ -v4::YAxis(), +v4::XAxis(), -v4::ZAxis() },
				{ +v4::ZAxis(), +v4::XAxis(), -v4::YAxis() },
				{ -v4::ZAxis(), -v4::XAxis(), -v4::YAxis() },
			};
			auto const mirror = CubeToWorld();
			auto const& axes = face_axes[face];
			auto forward = mirror * axes.major;
			auto right = mirror * axes.u;
			auto up = -(mirror * axes.v);
			m_scene.m_cam.CameraToWorld(m4x4{ right, up, -forward, cube.m_centre });

			// Render the source window's objects from the face camera. The draw lists are cleared afterwards so that
			// captured objects can be freed between captures.
			m_scene.ClearDrawlists();
			for (auto& obj : src.m_objects)
				obj->AddToScene(m_scene);

			auto& frame = m_wnd.NewFrame();
			m_scene.Render(frame);
			m_wnd.Present(frame, EGpuFlush::Async);
			m_scene.ClearDrawlists();

			// Copy the rendered face into the scratch texture and generate its mips.
			// Mip generation averages the sRGB encoded values, which slightly darkens high-contrast detail in the blurrier mips.
			// Mips also average the stored distances, which blurs distances across silhouettes in the blurrier mips.
			auto* rendered = m_wnd.FrameOutput().m_render_target.get();
			auto& cmd_list = m_factory.CmdList();
			auto const store_distance = cube.m_distance_scale > 0.0f;
			if (store_distance)
			{
				// Give the cube a distance cube the first time it stores distances. It has the same size and mips as the colour cube.
				if (cube.m_distance == nullptr)
				{
					auto tdesc = TextureDesc(AutoId, ResDesc::TexCube(Image{m_face_size, m_face_size, nullptr, DistanceFormat}, 0)).name("EnvMapDistance");
					cube.m_distance = m_factory.CreateTextureCube(tdesc);
				}

				// Copy the colour and store distances with a compute pass. Its output views are untyped-load free, so any device can run it.
				auto* depth = m_wnd.m_msaa_bb.m_depth_stencil.get();
				BarrierBatch barriers(cmd_list);
				barriers.Transition(rendered, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
				barriers.Transition(depth, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
				barriers.Transition(m_scratch.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS, 0);
				barriers.Transition(m_scratch_distance.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS, 0);
				barriers.Commit();

				// Bind the pass. Each view gets its own table, because consecutive heap entries can wrap around the ring.
				cmd_list.SetComputeRootSignature(m_distance_sig.get());
				cmd_list.SetPipelineState(m_distance_pso.get());
				auto heaps = { m_view_heap.get() };
				cmd_list.SetDescriptorHeaps({ heaps.begin(), heaps.size() });

				auto cb = DistanceConstants{
					.s2c = Invert(m_scene.m_cam.CameraToScreen()),
					.info = v4(s_cast<float>(m_face_size), cube.m_distance_scale, 0, 0),
				};
				cmd_list.SetComputeRoot32BitConstants(EDistanceParam::Constants, sizeof(cb) / sizeof(uint32_t), &cb, 0);

				auto colour_srv = D3D12_SHADER_RESOURCE_VIEW_DESC{
					.Format = Format,
					.ViewDimension = D3D12_SRV_DIMENSION_TEXTURE2D,
					.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING,
					.Texture2D = { .MostDetailedMip = 0, .MipLevels = 1, .PlaneSlice = 0, .ResourceMinLODClamp = 0.0f },
				};
				cmd_list.SetComputeRootDescriptorTable(EDistanceParam::Colour, m_view_heap.Add(rendered, colour_srv));

				auto depth_srv = colour_srv;
				depth_srv.Format = m_wnd.ResolvedDepthSrvFormat();
				cmd_list.SetComputeRootDescriptorTable(EDistanceParam::Depth, m_view_heap.Add(depth, depth_srv));

				auto face_uav = D3D12_UNORDERED_ACCESS_VIEW_DESC{
					.Format = Format,
					.ViewDimension = D3D12_UAV_DIMENSION_TEXTURE2D,
					.Texture2D = { .MipSlice = 0, .PlaneSlice = 0 },
				};
				cmd_list.SetComputeRootDescriptorTable(EDistanceParam::Face, m_view_heap.Add(m_scratch.get(), face_uav));

				auto distance_uav = face_uav;
				distance_uav.Format = DistanceFormat;
				cmd_list.SetComputeRootDescriptorTable(EDistanceParam::Distance, m_view_heap.Add(m_scratch_distance.get(), distance_uav));

				// One thread per texel, in 8x8 groups
				auto groups = s_cast<UINT>((m_face_size + 7) / 8);
				cmd_list.Dispatch(groups, groups, 1);
				barriers.UAV(m_scratch.get());
				barriers.UAV(m_scratch_distance.get());
				barriers.Commit();
			}
			else
			{
				BarrierBatch barriers(cmd_list);
				barriers.Transition(rendered, D3D12_RESOURCE_STATE_COPY_SOURCE);
				barriers.Transition(m_scratch.get(), D3D12_RESOURCE_STATE_COPY_DEST);
				barriers.Commit();

				auto dst = D3D12_TEXTURE_COPY_LOCATION{ .pResource = m_scratch.get(), .Type = D3D12_TEXTURE_COPY_TYPE_SUBRESOURCE_INDEX, .SubresourceIndex = 0 };
				auto src_loc = D3D12_TEXTURE_COPY_LOCATION{ .pResource = rendered, .Type = D3D12_TEXTURE_COPY_TYPE_SUBRESOURCE_INDEX, .SubresourceIndex = 0 };
				cmd_list.CopyTextureRegion(&dst, 0, 0, 0, &src_loc, nullptr);
			}
			m_factory.GenerateMips(m_scratch.get());
			if (store_distance)
				m_factory.GenerateMips(m_scratch_distance.get());

			// Copy every mip of the face into the cube, and its distances into the distance cube. Cube subresources are ordered by face (array slice), then mip.
			{
				BarrierBatch barriers(cmd_list);
				barriers.Transition(m_scratch.get(), D3D12_RESOURCE_STATE_COPY_SOURCE);
				barriers.Transition(cube.m_res.get(), D3D12_RESOURCE_STATE_COPY_DEST);
				if (store_distance)
				{
					barriers.Transition(m_scratch_distance.get(), D3D12_RESOURCE_STATE_COPY_SOURCE);
					barriers.Transition(cube.m_distance->m_res.get(), D3D12_RESOURCE_STATE_COPY_DEST);
				}
				barriers.Commit();

				for (UINT mip = 0; mip != m_mips; ++mip)
				{
					auto dst = D3D12_TEXTURE_COPY_LOCATION{ .pResource = cube.m_res.get(), .Type = D3D12_TEXTURE_COPY_TYPE_SUBRESOURCE_INDEX, .SubresourceIndex = mip + s_cast<UINT>(face) * m_mips };
					auto src_loc = D3D12_TEXTURE_COPY_LOCATION{ .pResource = m_scratch.get(), .Type = D3D12_TEXTURE_COPY_TYPE_SUBRESOURCE_INDEX, .SubresourceIndex = mip };
					cmd_list.CopyTextureRegion(&dst, 0, 0, 0, &src_loc, nullptr);
					if (!store_distance)
						continue;

					dst.pResource = cube.m_distance->m_res.get();
					src_loc.pResource = m_scratch_distance.get();
					cmd_list.CopyTextureRegion(&dst, 0, 0, 0, &src_loc, nullptr);
				}
			}
			m_factory.FlushToGpu(EGpuFlush::Async);
		}
	};

	// Render this window's objects into 'env_map', centred at 'position'
	void V3dWindow::EnvMapCapture(TextureCube& env_map, v4 const& position)
	{
		// Capture uses the window's scene state and objects, so it has the same thread affinity as 'Render'
		assert(std::this_thread::get_id() == m_main_thread_id);

		// Render all six faces. GPU queue ordering makes frames that use 'env_map' after this call see the complete capture.
		// Distances are stored relative to the parallax bounds' size, so reflections that use this map can correct for parallax.
		auto& res = EnvMapCaptureResources::Get(*this, EnvMapCaptureResources::ValidateCube(env_map));
		env_map.m_cube2w = EnvMapCaptureResources::CubeToWorld();
		env_map.m_centre = position;
		env_map.m_distance_scale = EnvMapDistanceScale();
		for (int f = 0; f != 6; ++f)
			res.CaptureFace(*this, env_map, f);

		Invalidate();
	}

	// A time-sliced environment map. It renders one face per update into a capture cube. When the capture cube is complete,
	// it becomes the current cube, and the old current cube becomes the previous cube that the new one fades in over.
	struct EnvMapProbeResources
	{
		static constexpr int FadeSteps = 5; // Updates from a cube's completion until it fully replaces the previous cube

		int m_face_size;                      // Pixels per face edge
		std::array<TextureCubePtr, 3> m_cubes; // The previous, current, and capture cubes, in that order
		int m_face;                            // The next face to render into the capture cube
		int m_fade;                            // Updates since the current cube was completed, up to 'FadeSteps'
		int m_complete;                        // The number of complete cubes, up to 2

		EnvMapProbeResources(Renderer& rdr, int face_size)
			: m_face_size(face_size)
			, m_cubes()
			, m_face()
			, m_fade()
			, m_complete()
		{
			// Create the cubes in the format captures require, with the capture orientation
			ResourceFactory factory(rdr);
			for (auto& cube : m_cubes)
			{
				auto tdesc = TextureDesc(AutoId, ResDesc::TexCube(Image{face_size, face_size, nullptr, ToSRGB(EnvMapCaptureResources::Format)}, 0)).name("EnvMapProbe");
				cube = factory.CreateTextureCube(tdesc);
				cube->m_cube2w = EnvMapCaptureResources::CubeToWorld();
			}
			factory.FlushToGpu(EGpuFlush::Block);
		}
	};

	// Enable the time-sliced environment map probe with 'face_size' pixel faces, or disable it with 0
	void V3dWindow::EnvMapProbe(int face_size)
	{
		// Keep the probe's progress if nothing has changed
		if (face_size < 0)
			throw std::runtime_error("Environment map probe face size must not be negative");
		if (face_size == (m_envmap_probe != nullptr ? m_envmap_probe->m_face_size : 0))
			return;

		// The probe owns the global environment map, so changing the probe clears it until the new probe completes a cube
		m_envmap_probe = face_size != 0 ? std::make_unique<EnvMapProbeResources>(*m_rdr, face_size) : nullptr;
		m_scene.m_global_envmap = nullptr;
		m_scene.m_global_envmap_prev = nullptr;
		m_scene.m_global_envmap_blend = 1.0f;

		OnSettingsChanged(this, view3d::ESettings::Scene_EnvMap);
		Invalidate();
	}

	// Render the next face of the probe's environment map
	void V3dWindow::EnvMapProbeUpdate(v4 const& position)
	{
		// Capture uses the window's scene state and objects, so it has the same thread affinity as 'Render'
		assert(std::this_thread::get_id() == m_main_thread_id);
		if (m_envmap_probe == nullptr)
			throw std::runtime_error("Environment map probe is not enabled");

		// Render the next face. The position and distance scale are fixed for the whole cube so that its faces agree.
		auto& probe = *m_envmap_probe;
		auto& capture = *probe.m_cubes[2].get();
		if (probe.m_face == 0)
		{
			capture.m_centre = position;
			capture.m_distance_scale = EnvMapDistanceScale();
		}

		auto& res = EnvMapCaptureResources::Get(*this, probe.m_face_size);
		res.CaptureFace(*this, capture, probe.m_face);

		// When the capture cube is complete, it becomes current and fades in over the old current cube. The oldest cube is reused for the next capture.
		// The fade restarts at zero, which shows only the old current cube, so the image does not jump.
		if (++probe.m_face == 6)
		{
			std::rotate(probe.m_cubes.begin(), probe.m_cubes.begin() + 1, probe.m_cubes.end());
			probe.m_face = 0;
			probe.m_fade = 0;
			probe.m_complete = std::min(probe.m_complete + 1, 2);
		}
		else
		{
			probe.m_fade = std::min(probe.m_fade + 1, EnvMapProbeResources::FadeSteps);
		}

		// Bind the complete cubes. Until a second cube is complete, there is nothing to fade from.
		m_scene.m_global_envmap = probe.m_complete >= 1 ? probe.m_cubes[1] : nullptr;
		m_scene.m_global_envmap_prev = probe.m_complete >= 2 ? probe.m_cubes[0] : nullptr;
		m_scene.m_global_envmap_blend = s_cast<float>(probe.m_fade) / EnvMapProbeResources::FadeSteps;
		Invalidate();
	}

	// Get/Set the world-space bounds of the captured geometry that reflections correct for parallax
	BBox V3dWindow::EnvMapParallaxBounds() const
	{
		return m_scene.m_global_envmap_parallax_bounds;
	}
	void V3dWindow::EnvMapParallaxBounds(BBox const& bounds)
	{
		// An invalid box disables parallax correction. A valid box must be finite, because the march works between its faces.
		if (bounds.valid() && !IsFinite(bounds.m_radius))
			throw std::runtime_error("Environment map parallax bounds must be finite");
		if (bounds == m_scene.m_global_envmap_parallax_bounds)
			return;

		m_scene.m_global_envmap_parallax_bounds = bounds;
		Invalidate();
	}

	// The scale of the distances stored in captured cubes. It matches the size of the parallax bounds, so that the 8-bit 'd / (d + S)' encoding
	// spends most of its precision on distances within the bounds. 0 means the cube stores no distances.
	float V3dWindow::EnvMapDistanceScale() const
	{
		auto const& bounds = m_scene.m_global_envmap_parallax_bounds;
		return bounds.valid() ? Length(bounds.Radius().xyz) : 0.0f;
	}

	// Enable/Disable the depth buffer
	bool V3dWindow::DepthBufferEnabled() const
	{
		auto depth = m_scene.m_pso.Find<EPipeState::DepthEnable>();
		return depth != nullptr ? *depth : true;
	}
	void V3dWindow::DepthBufferEnabled(bool enabled)
	{
		m_scene.m_pso.Set<EPipeState::DepthEnable>(enabled ? TRUE : FALSE);
	}

	// Set the position and size of the selection box. If 'bbox' is 'BBox::Reset()' the selection box is not shown
	void V3dWindow::SetSelectionBox(BBox const& bbox, m3x3 const& ori)
	{
		if (bbox == BBox::Reset())
		{
			// Flag to not include the selection box
			m_selection_box.m_i2w.pos.w = 0;
		}
		else
		{
			m_selection_box.m_i2w =
				m4x4(ori, v4::Origin()) *
				m4x4::Scale(bbox.m_radius.x, bbox.m_radius.y, bbox.m_radius.z, bbox.m_centre);
		}
	}

	// Position the selection box to include the selected objects
	void V3dWindow::SelectionBoxFitToSelected()
	{
		// Find the bounds of the selected objects
		auto bbox = BBox::Reset();
		for (auto& obj : m_objects)
		{
			obj->Apply([&](ldraw::LdrObject const* c)
			{
				if (!AllSet(c->Flags(), ldraw::ELdrFlags::Selected) || AllSet(c->Flags(), ldraw::ELdrFlags::SceneBoundsExclude))
					return true;

				auto bb = c->BBoxWS(ldraw::EBBoxFlags::IncludeChildren);
				Grow(bbox, bb);
				return false;
			}, "");
		}
		SetSelectionBox(bbox);
	}

	// Get/Set the window background colour
	int V3dWindow::MultiSampling() const
	{
		return m_wnd.MultiSampling().Count;
	}
	void V3dWindow::MultiSampling(int multisampling)
	{
		if (MultiSampling() == multisampling)
			return;

		m_wnd.MultiSampling(MultiSamp(multisampling));

		OnSettingsChanged(this, view3d::ESettings::Scene_Multisampling);
		Invalidate();
	}

	// Get/Set the output dither noise amplitude, in 8-bit sRGB steps
	float V3dWindow::DitherAmount() const
	{
		return m_wnd.m_dither_amount;
	}
	void V3dWindow::DitherAmount(float amount)
	{
		// Ignore unchanged values so redundant sets do not trigger a redraw
		if (m_wnd.m_dither_amount == amount)
			return;

		m_wnd.m_dither_amount = amount;
		Invalidate();
	}

	// Control animation
	void V3dWindow::AnimControl(view3d::EAnimCommand command, seconds_t time)
	{
		using namespace std::chrono;
		static constexpr auto tick_size_s = seconds_t(0.01);

		// Callback function that is polled as fast as the message queue will allow
		auto const AnimTick = [](void* ctx)
		{
			auto& me = *reinterpret_cast<V3dWindow*>(ctx);
			me.AnimationStep(view3d::EAnimCommand::Step, me.m_anim_data.m_clock.load());
		};

		switch (command)
		{
			case view3d::EAnimCommand::Reset:
			{
				AnimControl(view3d::EAnimCommand::Stop);
				assert(IsFinite(time.count()));
				m_anim_data.m_clock.store(time);
				break;
			}
			case view3d::EAnimCommand::Play:
			{
				AnimControl(view3d::EAnimCommand::Stop);
				auto rate = time.count();
				auto issue = m_anim_data.m_issue.load();
				auto clock0 = m_anim_data.m_clock.load();
				m_anim_data.m_thread = std::jthread([this, issue, rate, clock0]
				{
					// 'rate' is the seconds/second step rate
					auto time0 = system_clock::now();
					auto increment = tick_size_s * rate;
					for (; ; std::this_thread::sleep_for(tick_size_s))
					{
						auto iss = m_anim_data.m_issue.load();
						if (iss != issue)
							break;

						// Every loop is a tick, and the step size is 'time'. 
						// If 'time' is zero, then stepping is real-time and the step size is 'elapsed'
						if (rate == 0.0)
							m_anim_data.m_clock.store(clock0 + (system_clock::now() - time0));
						else
							m_anim_data.m_clock.store(m_anim_data.m_clock.load() + increment);
					}
				});
				m_wnd.m_rdr->AddPollCB({ this, AnimTick }, seconds_t(0));
				break;
			}
			case view3d::EAnimCommand::Stop:
			{
				m_wnd.m_rdr->RemovePollCB({ this, AnimTick });
				++m_anim_data.m_issue;
				if (m_anim_data.m_thread.joinable())
					m_anim_data.m_thread.join();

				break;
			}
			case view3d::EAnimCommand::Step:
			{
				AnimControl(view3d::EAnimCommand::Stop);
				m_anim_data.m_clock = m_anim_data.m_clock.load() + time;
				break;
			}
			default:
			{
				throw std::runtime_error(FmtS("Unknown animation command: %d", command));
			}
		}

		// Notify of the animation event
		AnimationStep(command, m_anim_data.m_clock.load());
	}

	// True if animation is currently active
	bool V3dWindow::Animating() const
	{
		return m_anim_data.m_thread.joinable();
	}
	
	// Get/Set the value of the animation clock
	seconds_t V3dWindow::AnimTime() const
	{
		return m_anim_data.m_clock.load();
	}
	void V3dWindow::AnimTime(seconds_t clock)
	{
		assert(IsFinite(clock.count()) && clock.count() >= 0);
		m_anim_data.m_clock.store(clock);
	}

	// Called when the animation time has changed
	void V3dWindow::AnimationStep(view3d::EAnimCommand command, seconds_t anim_time)
	{
		// Update all animated objects in this window
		auto anim_time_s = static_cast<float>(anim_time.count());
		for (auto& obj : m_objects)
		{
			// Only animate children if the parent is animated
			if (AllSet(obj->RecursiveFlags(), ldraw::ELdrFlags::Animated))
				obj->AnimTime(anim_time_s, "");
		}

		Invalidate();
		OnAnimationEvent(this, command, anim_time.count());
	}

	// Cast rays into the scene, returning hit info for the nearest intercept for each ray
	void V3dWindow::HitTest(std::span<view3d::HitTestRay const> rays, std::span<view3d::HitTestResult> hits, RayCastInstancesCB instances)
	{
		if (rays.size() != hits.size())
			throw std::runtime_error("There should be a hit object for each ray");

		// Set up the ray cast
		vector<HitTestRay, 1> ray_casts = {};
		ray_casts.reserve(rays.size());
		for (auto& ray : rays)
			ray_casts.push_back(To<HitTestRay>(ray));

		// Initialise the results
		auto const invalid = view3d::HitTestResult{.m_distance = limits<float>::max()};
		for (auto& r : hits)
			r = invalid;

		// Do the ray casts into the scene and save the results
		m_scene.HitTest(ray_casts, instances, [=](std::span<HitTestResult const> results)
		{
			for (auto const& hit : results)
			{
				// Check that 'hit.m_instance' is a valid instance in this scene.
				// It could be a child instance, we need to search recursively for a match
				auto ldr_obj = cast<ldraw::LdrObject>(hit.m_instance);

				// Not an object in this scene, keep looking
				// This needs to come first in case 'ldr_obj' points to an object that has been deleted.
				if (!Has(ldr_obj, true))
					continue;

				// Not visible to hit tests, keep looking
				if (AllSet(ldr_obj->Flags(), ldraw::ELdrFlags::HitTestExclude))
					continue;

				// The intercepts are already sorted from nearest to furtherest.
				// So we can just accept the first intercept for each ray.
				if (hits[hit.m_ray_index].IsHit())
					continue;

				// Save the hit
				hits[hit.m_ray_index] = To<view3d::HitTestResult>(hit);
			}
		}).wait();
	}
	void V3dWindow::HitTest(std::span<view3d::HitTestRay const> rays, std::span<view3d::HitTestResult> hits, ldraw::LdrObject const* const* objects, int object_count)
	{
		// Create an instances function based on the given list of objects
		auto beg = &objects[0];
		auto end = beg + object_count;
		auto instances = [&]() -> BaseInstance const*
		{
			if (beg == end) return nullptr;
			auto* inst = *beg++;
			return &inst->m_base;
		};
		HitTest(rays, hits, instances);
	}
	void V3dWindow::HitTest(std::span<view3d::HitTestRay const> rays, std::span<view3d::HitTestResult> hits, view3d::GuidPredCB pred, int)
	{
		// Create an instances function based on the context ids
		auto beg = std::begin(m_scene.m_instances);
		auto end = std::end(m_scene.m_instances);
		auto instances = [&]() -> BaseInstance const*
		{
			for (; beg != end && pred && !pred(cast<ldraw::LdrObject>(*beg)->m_context_id); ++beg) {}
			return beg != end ? *beg++ : nullptr;
		};
		HitTest(rays, hits, instances);
	}

	// Trigger execution of the async hit test rays. Submits GPU work and returns immediately.
	void V3dWindow::HitTestAsync()
	{
		m_scene.HitTestAsync(m_ht_rays);
	}

	// Add/Update/Remove an async hit test ray.
	view3d::HitTestRayId V3dWindow::HitTestRayUpdate(view3d::HitTestRayId id, view3d::HitTestRay const* ray)
	{
		static int new_id = static_cast<int>(view3d::HitTestRayId::None);

		// Add a new ray
		if (id == view3d::HitTestRayId::None && ray != nullptr)
		{
			if (m_ht_rays.size() == rdr12::MaxRays)
				return view3d::HitTestRayId::None;

			auto ray_ = To<HitTestRay>(*ray);
			ray_.m_id = ++new_id;
			m_ht_rays.push_back(ray_);
			id = s_cast<view3d::HitTestRayId>(ray_.m_id);
		}

		// Remove a ray
		else if (ray == nullptr)
		{
			auto num = pr::erase_if(m_ht_rays, [ID = s_cast<int>(id)](HitTestRay const& r) { return r.m_id == ID; });
			id = num != 0 ? id : view3d::HitTestRayId::None;
		}

		// Update a ray
		else
		{
			auto it = std::ranges::find_if(m_ht_rays, [ID = s_cast<int>(id)](HitTestRay const& r) { return r.m_id == ID; });
			if (it == std::end(m_ht_rays))
				return view3d::HitTestRayId::None;

			*it = To<HitTestRay>(*ray);
			it->m_id = s_cast<int>(id);
		}

		return id;
	}

	// Move the focus point to the hit target
	void V3dWindow::CentreOnHitTarget(view3d::HitTestRay const& ray_)
	{
		HitTestRay ray = To<HitTestRay>(ray_);
		HitTestResult target = {};

		auto beg = std::begin(m_scene.m_instances);
		auto end = std::end(m_scene.m_instances);
		auto instances = [&]() -> BaseInstance const*
		{
			//for (; beg != end && beg->m_context_id != ray.m_hit_context_id; ++beg) {}
			return beg != end ? *beg++ : nullptr;
		};

		// Cast 'ray' into the scene
		m_scene.HitTest({ &ray, 1ULL }, instances, [&](std::span<HitTestResult const> results)
		{
			for (auto const& hit : results)
			{
				// Check that 'hit.m_instance' is a valid instance in this scene.
				// It could be a child instance, we need to search recursively for a match
				auto ldr_obj = cast<ldraw::LdrObject>(hit.m_instance);

				// Not an object in this scene, keep looking
				// This needs to come first in case 'ldr_obj' points to an object that has been deleted.
				if (!Has(ldr_obj, true))
					continue;

				// Not visible to hit tests, keep looking
				if (AllSet(ldr_obj->Flags(), ldraw::ELdrFlags::HitTestExclude))
					continue;

				// The intercepts are already sorted from nearest to furtherest.
				// So we can just accept the first intercept as the hit test.
				target = hit;
				break;
			}
		}).wait();

		// Move the focus point to the centre of the bbox of the hit object
		if (target.IsHit())
		{
			// If the shift key is held, focus on the exact hit point instead of the centre of the object
			if (KeyDown(VK_SHIFT)) 
			{
				FocusPoint(target.m_ws_intercept);
			}
			else
			{
				auto ldr_obj = cast<ldraw::LdrObject>(target.m_instance);
				auto bbox = ldr_obj->BBoxWS(ldraw::EBBoxFlags::IncludeChildren);
				FocusPoint(bbox.m_centre);
			}
		}
	}

	// Get/Set the visibility of one or more stock objects (focus point, origin, selection box, etc)
	bool V3dWindow::StockObjectVisible(view3d::EStockObject stock_objects) const
	{
		return AllSet(m_visible_objects, stock_objects);
	}
	void V3dWindow::StockObjectVisible(view3d::EStockObject stock_objects, bool vis)
	{
		if (StockObjectVisible(stock_objects) == vis)
			return;

		m_visible_objects = SetBits(m_visible_objects, stock_objects, vis);

		auto settings = view3d::ESettings::None;
		if (AllSet(stock_objects, view3d::EStockObject::FocusPoint)) settings |= view3d::ESettings::General_FocusPointVisible;
		if (AllSet(stock_objects, view3d::EStockObject::OriginPoint)) settings |= view3d::ESettings::General_OriginPointVisible;
		if (AllSet(stock_objects, view3d::EStockObject::SelectionBox)) settings |= view3d::ESettings::General_SelectionBoxVisible;
		OnSettingsChanged(this, settings);
		Invalidate();
	}

	// Get/Set the size of the focus point
	float V3dWindow::FocusPointSize() const
	{
		return m_focus_point.m_size;
	}
	void V3dWindow::FocusPointSize(float size)
	{
		if (FocusPointSize() == size)
			return;

		m_focus_point.m_size = size;

		OnSettingsChanged(this, view3d::ESettings::General_FocusPointSize);
		Invalidate();
	}

	// Get/Set the size of the origin point
	float V3dWindow::OriginPointSize() const
	{
		return m_origin_point.m_size;
	}
	void V3dWindow::OriginPointSize(float size)
	{
		if (OriginPointSize() == size)
			return;

		m_origin_point.m_size = size;

		OnSettingsChanged(this, view3d::ESettings::General_OriginPointSize);
		Invalidate();
	}

	// Get/Set the position and size of the selection box. If 'bbox' is 'BBox::Reset()' the selection box is not shown
	std::tuple<BBox, m3x3> V3dWindow::SelectionBox() const
	{
		if (m_selection_box.m_i2w.pos.w == 0)
			return { BBox::Reset(), m3x3::Identity() };

		auto const& i2w = m_selection_box.m_i2w;
		auto bbox = BBox(i2w.pos, v4(Length(i2w.x), Length(i2w.y), Length(i2w.z), 0));
		auto ori = m_selection_box.m_i2w.rot;
		return { bbox, ori };

	}
	void V3dWindow::SelectionBox(BBox const& bbox, m3x3 const& ori)
	{
		auto [b, o] = SelectionBox();
		if (b == bbox && All(o == ori))
			return;

		if (bbox == BBox::Reset())
		{
			// Flag to not include the selection box
			m_selection_box.m_i2w.pos.w = 0;
		}
		else
		{
			m_selection_box.m_i2w =
				m4x4(ori, v4::Origin()) *
				m4x4::Scale(bbox.m_radius.x, bbox.m_radius.y, bbox.m_radius.z, bbox.m_centre);
		}

		OnSettingsChanged(this, view3d::ESettings::General_SelectionBox);
		Invalidate();
	}

	// Show/Hide the bounding boxes
	bool V3dWindow::BBoxesVisible() const
	{
		return m_wnd.m_diag.m_bboxes_visible;
	}
	void V3dWindow::BBoxesVisible(bool vis)
	{
		if (BBoxesVisible() == vis)
			return;

		m_wnd.m_diag.m_bboxes_visible = vis;

		OnSettingsChanged(this, view3d::ESettings::Diagnostics_BBoxesVisible);
		Invalidate();
	}

	// Get/Set the length of the displayed vertex normals
	float V3dWindow::NormalsLength() const
	{
		return m_wnd.m_diag.m_normal_lengths;
	}
	void V3dWindow::NormalsLength(float length)
	{
		if (NormalsLength() == length)
			return;

		m_wnd.m_diag.m_normal_lengths = length;

		OnSettingsChanged(this, view3d::ESettings::Diagnostics_NormalsLength);
		Invalidate();
	}
	
	// Get/Set the colour of the displayed vertex normals
	Colour32 V3dWindow::NormalsColour() const
	{
		return m_wnd.m_diag.m_normal_colour;
	}
	void V3dWindow::NormalsColour(Colour32 colour)
	{
		if (NormalsColour() == colour)
			return;

		m_wnd.m_diag.m_normal_colour = colour;

		OnSettingsChanged(this, view3d::ESettings::Diagnostics_NormalsColour);
		Invalidate();
	}

	// Get/Set the colour of the displayed vertex normals
	v2 V3dWindow::FillModePointsSize() const
	{
		auto shdr = static_cast<shaders::PointSpriteGS const*>(m_wnd.m_diag.m_gs_fillmode_points.get());
		return shdr->m_size;
	}
	void V3dWindow::FillModePointsSize(v2 size)
	{
		if (All(FillModePointsSize() == size))
			return;
		
		auto shdr = static_cast<shaders::PointSpriteGS*>(m_wnd.m_diag.m_gs_fillmode_points.get());
		shdr->m_size = size;
		
		OnSettingsChanged(this, view3d::ESettings::Diagnostics_FillModePointsSize);
		Invalidate();
	}

	// Access the built-in script editor
	ldraw::ScriptEditorUI& V3dWindow::EditorUI()
	{
		if (!m_ui_script_editor)
			m_ui_script_editor.reset(new ldraw::ScriptEditorUI(m_hwnd));

		return *m_ui_script_editor;
	}

	// Access the built-in lighting controls UI
	LightingUI& V3dWindow::LightingUI()
	{
		if (!m_ui_lighting)
		{
			// The native lighting UI edits the main light (light 0) and the scene ambient light
			if (m_scene.m_lights.empty())
				m_scene.m_lights.push_back(Light{});

			m_ui_lighting.reset(new rdr12::LightingUI(m_hwnd, m_scene.m_lights[0], m_scene.m_ambient));
			m_ui_lighting->HideOnClose(true);
			m_ui_lighting->Commit += [&](rdr12::LightingUI& ui, Light const& light)
			{
				SceneLight(0, light);
				Ambient(ui.m_ambient);
			};
			m_ui_lighting->Preview += [&](rdr12::LightingUI& ui, Light const& light)
			{
				// Render once with the previewed lighting, then restore the committed lighting
				auto prev_light = m_scene.m_lights[0];
				auto prev_ambient = m_scene.m_ambient;
				m_scene.m_lights[0] = light;
				m_scene.m_ambient = ui.m_ambient;

				Render();

				m_scene.m_lights[0] = prev_light;
				m_scene.m_ambient = prev_ambient;
			};
		}

		return *m_ui_lighting;
	}

	// Return the focus point of the camera in this draw set
	static v4 __stdcall ReadPoint(void* ctx)
	{
		if (ctx == 0) return v4::Origin();
		return static_cast<V3dWindow const*>(ctx)->m_scene.m_cam.FocusPoint();
	}

	// Show/Hide the object manager tool
	bool V3dWindow::ObjectManagerVisible() const
	{
		return m_ui_object_manager != nullptr && m_ui_object_manager->Visible();

	}
	void V3dWindow::ObjectManagerVisible(bool show)
	{
		assert(std::this_thread::get_id() == m_main_thread_id);
		if (ObjectManagerVisible() == show)
			return;

		if (!m_ui_object_manager)
			m_ui_object_manager.reset(new ldraw::ObjectManagerUI(m_hwnd));
		
		m_ui_object_manager->Visible(show);
	}

	// Show/Hide the script editor tool
	bool V3dWindow::ScriptEditorVisible() const
	{
		return m_ui_script_editor != nullptr && m_ui_script_editor->Visible();

	}
	void V3dWindow::ScriptEditorVisible(bool show)
	{
		assert(std::this_thread::get_id() == m_main_thread_id);
		if (ScriptEditorVisible() == show)
			return;

		EditorUI().Visible(show);
	}

	// Show/Hide the measure tool
	bool V3dWindow::MeasureToolVisible() const
	{
		return m_ui_measure_tool != nullptr && m_ui_measure_tool->Visible();

	}
	void V3dWindow::MeasureToolVisible(bool show)
	{
		assert(std::this_thread::get_id() == m_main_thread_id);
		if (MeasureToolVisible() == show)
			return;

		if (!m_ui_measure_tool)
			m_ui_measure_tool.reset(new ldraw::MeasureUI(m_hwnd, &ReadPoint, this, rdr()));
		else
			m_ui_measure_tool->SetReadPoint(&ReadPoint, this);
		
		m_ui_measure_tool->Visible(show);
	}

	// Show/Hide the angle measure tool
	bool V3dWindow::AngleToolVisible() const
	{
		return m_ui_angle_tool != nullptr && m_ui_angle_tool->Visible();

	}
	void V3dWindow::AngleToolVisible(bool show)
	{
		assert(std::this_thread::get_id() == m_main_thread_id);
		if (AngleToolVisible() == show)
			return;

		if (!m_ui_angle_tool)
			m_ui_angle_tool.reset(new ldraw::AngleUI(m_hwnd, &ReadPoint, this, rdr()));
		else
			m_ui_angle_tool->SetReadPoint(&ReadPoint, this);
		
		m_ui_angle_tool->Visible(show);
	}

	// Implements standard key bindings. Returns true if handled
	bool V3dWindow::TranslateKey(EKeyCodes key, v2 ss_point)
	{
		// Notes:
		//  - This method is intended as a simple default for key bindings. Applications should
		//    probably not call this, but handled the keys bindings separately. This helps to show
		//    the expected behaviour of some common bindings though.
		if (FlightCameraIsEnabled())
			return false;

		auto code = key & EKeyCodes::KeyCode;
		auto modifiers = key & EKeyCodes::Modifiers;
		switch (code)
		{
			case EKeyCodes::F7:
			{
				auto up = LengthSq(m_scene.m_cam.Align()) > math::tiny<float> ? m_scene.m_cam.Align() : v4::YAxis();
				auto forward = up.z > up.y ? v4::YAxis() : -v4::ZAxis();

				auto bounds =
					AllSet(modifiers, EKeyCodes::Control) ? view3d::ESceneBounds::All :
					AllSet(modifiers, EKeyCodes::Shift) ? view3d::ESceneBounds::Selected :
					view3d::ESceneBounds::Visible;

				ResetView(SceneBounds(bounds, 0, nullptr), forward, up, 0, true, true);
				Invalidate();
				return true;
			}
			case EKeyCodes::Space:
			{
				ObjectManagerVisible(true);
				return true;
			}
			case EKeyCodes::W:
			{
				if (AllSet(modifiers, EKeyCodes::Control))
				{
					switch (FillMode())
					{
						case EFillMode::Default:
						case EFillMode::Solid:     FillMode(EFillMode::Wireframe); break;
						case EFillMode::Wireframe: FillMode(EFillMode::SolidWire); break;
						case EFillMode::SolidWire: FillMode(EFillMode::Default); break;
						default: throw std::runtime_error("Unknown fill mode");
					}
					Invalidate();
				}
				return true;
			}
			case EKeyCodes::Decimal:
			case EKeyCodes::OemPeriod:
			case EKeyCodes::MButton:
			{
				auto z = static_cast<float>(m_scene.m_cam.FocusDist());
				auto nss_pt = m_scene.m_viewport.SSPointToNSSPoint(ss_point);
				auto [pt, dir] = m_scene.m_cam.NSSPointToWSRay(v4{ nss_pt, z, 1 });
				CentreOnHitTarget(view3d::HitTestRay{
					.m_ws_origin = To<view3d::Vec4>(pt),
					.m_ws_direction = To<view3d::Vec4>(dir),
					.m_snap_mode = view3d::ESnapMode::All,
					.m_snap_distance = 0,
					.m_id = view3d::HitTestRayId::None,
				});
				return true;
			}
		}
		return false;
	}

	// Called when objects are added/removed from this window
	void V3dWindow::ObjectContainerChanged(view3d::ESceneChanged change_type, std::span<GUID const> context_ids, ldraw::LdrObject* object)
	{
		// Reset the draw lists so that removed objects are no longer in the draw list
		if (change_type == view3d::ESceneChanged::ObjectsRemoved)
		{
			// Objects are being removed, make sure they're not in the drawlist
			// for this window and that the graphics card is not still using them.
			m_scene.ClearDrawlists();
			m_wnd.m_gsync.Wait();
		}

		// Invalidate cached members
		m_bbox_scene = BBox::Reset();

		// Notify scene changed
		view3d::SceneChanged args = {change_type, context_ids.data(), s_cast<int>(context_ids.size()), object};
		OnSceneChanged(this, args);
	}

	// Create stock models such as the focus point, origin, etc
	void V3dWindow::CreateStockObjects()
	{
		ResourceFactory factory(rdr());

		// Create the focus point/origin models
		m_focus_point.m_model = factory.CreateModel(EStockModel::Basis);
		m_focus_point.m_tint = Colour32One;
		m_focus_point.m_i2w = m4x4::Identity();
		m_focus_point.m_size = 1.0f;
		m_origin_point.m_model = factory.CreateModel(EStockModel::Basis);
		m_origin_point.m_tint = Colour32Gray;
		m_origin_point.m_i2w = m4x4::Identity();
		m_origin_point.m_size = 1.0f;

		// Create the selection box model
		m_selection_box.m_model = factory.CreateModel(EStockModel::SelectionBox);
		m_selection_box.m_tint = Colour32White;
		m_selection_box.m_i2w = m4x4::Identity();
	}
}
