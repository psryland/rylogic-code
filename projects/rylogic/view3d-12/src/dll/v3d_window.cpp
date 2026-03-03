//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#include "view3d-12/src/dll/v3d_window.h"
#include "pr/view3d-12/ldraw/ldraw_object.h"
#include "pr/view3d-12/ldraw/ldraw_gizmo.h"
#include "pr/view3d-12/ldraw/ldraw_reader_text.h"
#include "pr/view3d-12/ldraw/ldraw.h"
#include "pr/view3d-12/shaders/shader_point_sprites.h"
#include "pr/view3d-12/resource/resource_factory.h"
#include "pr/view3d-12/utility/conversion.h"

namespace pr::rdr12
{
	// Default window construction settings
	WndSettings ToWndSettings(HWND hwnd, RdrSettings const& rsettings, view3d::WindowOptions const& opts)
	{
		return WndSettings(hwnd, true, rsettings)
			.DefaultOutput()
			.BackgroundColour(opts.m_background_colour)
			.AllowAltEnter(opts.m_allow_alt_enter != 0)
			.XrSupport(opts.m_xr_support != 0)
			.MutliSampling(opts.m_multisampling)
			.Name(opts.m_dbg_name ? std::string_view{ opts.m_dbg_name } : std::string_view{});
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
			if (rt_area != iv2Zero)
				m_scene.m_cam.Aspect(rt_area.x / float(rt_area.y));

			// The light for the scene
			m_scene.m_global_light.m_type = ELight::Directional;
			m_scene.m_global_light.m_ambient = Colour32(0xFF404040U);
			m_scene.m_global_light.m_diffuse = Colour32(0xFF404040U);
			m_scene.m_global_light.m_specular = Colour32(0xFF808080U);
			m_scene.m_global_light.m_specular_power = 1000.0f;
			m_scene.m_global_light.m_direction = -v4::ZAxis();
			m_scene.m_global_light.m_on = true;
			m_scene.m_global_light.m_cam_relative = true;

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
	char const* V3dWindow::Settings() const
	{
		std::stringstream out;
		out << "*Light {\n" << m_scene.m_global_light.Settings() << "}\n";
		m_settings = out.str();
		return m_settings.c_str();
	}
	void V3dWindow::Settings(char const* settings)
	{
		// Parse the settings
		mem_istream<char> src(settings);
		rdr12::ldraw::TextReader reader(src, {});
		for (int kw; reader.NextKeyword(kw);) switch (kw)
		{
			case rdr12::ldraw::HashI("Light"):
			{
				auto desc = reader.String<std::string>();
				m_scene.m_global_light.Settings(desc);
				OnSettingsChanged(this, view3d::ESettings::Lighting_All);
				break;
			}
		}
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
				for (auto& obj : m_objects)
				{
					if (!pred(*obj)) continue;
					if (!AllSet(obj->Flags(), ldraw::ELdrFlags::Selected)) continue;
					if (pr::contains(except_arr, obj->m_context_id)) continue;
					Grow(bbox, obj->BBoxWS(ldraw::EBBoxFlags::IncludeChildren, pred));
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
		auto old_count = m_objects.size();
		for (auto& srcs : sources)
		{
			auto& src = srcs.second;
			if (!pred(src->m_context_id))
				continue;

			// Add objects from this source
			new_guids.push_back(src->m_context_id);
			for (auto& obj : src->m_output.m_objects)
				m_objects.insert(obj.get());

			// Apply camera settings from this source
			if (src->m_output.m_cam_fields != ldraw::ECamField::None)
			{
				auto& cam = src->m_output.m_cam;
				auto changed = view3d::ESettings::Camera;
				if (AllSet(src->m_output.m_cam_fields, ldraw::ECamField::C2W))
				{
					m_scene.m_cam.CameraToWorld(cam.CameraToWorld());
					changed |= view3d::ESettings::Camera_Position;
				}
				if (AllSet(src->m_output.m_cam_fields, ldraw::ECamField::Focus))
				{
					m_scene.m_cam.LookAt(cam.CameraToWorld().pos, cam.FocusPoint(), cam.CameraToWorld().y);
					changed |= view3d::ESettings::Camera_Position;
					changed |= view3d::ESettings::Camera_FocusDist;
				}
				if (AllSet(src->m_output.m_cam_fields, ldraw::ECamField::Align))
				{
					m_scene.m_cam.Align(cam.Align());
					changed |= view3d::ESettings::Camera_AlignAxis;
				}
				if (AllSet(src->m_output.m_cam_fields, ldraw::ECamField::Aspect))
				{
					m_scene.m_cam.Aspect(cam.Aspect());
					changed |= view3d::ESettings::Camera_Aspect;
				}
				if (AllSet(src->m_output.m_cam_fields, ldraw::ECamField::FovY))
				{
					m_scene.m_cam.FovY(cam.FovY());
					changed |= view3d::ESettings::Camera_Fov;
				}
				if (AllSet(src->m_output.m_cam_fields, ldraw::ECamField::Near))
				{
					m_scene.m_cam.Near(cam.Near(true), true);
					changed |= view3d::ESettings::Camera_ClipPlanes;
				}
				if (AllSet(src->m_output.m_cam_fields, ldraw::ECamField::Far))
				{
					m_scene.m_cam.Far(cam.Far(true), true);
					changed |= view3d::ESettings::Camera_ClipPlanes;
				}
				if (AllSet(src->m_output.m_cam_fields, ldraw::ECamField::Ortho))
				{
					m_scene.m_cam.Orthographic(cam.Orthographic());
					changed |= view3d::ESettings::Camera_Orthographic;
				}

				// Notify if the camera was changed
				if (changed != view3d::ESettings::Camera)
					OnSettingsChanged(this, changed);
			}
		}
		if (m_objects.size() != old_count)
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
				pr::vector<Guid> guids(std::begin(removed), std::end(removed));
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
		pr::vector<GUID> context_ids(std::begin(m_guids), std::end(m_guids));

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
		if (m_scene.m_viewport.AsIRect().Size() == iv2::Zero())
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

		// Set the shadow casting light source
		m_scene.ShadowCasting(m_scene.m_global_light.m_cast_shadow != 0, 1024);

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
		m_scene.Render(frame);
		m_wnd.Present(frame);

		// No longer invalidated
		Validate();
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
	void V3dWindow::ResetView(v4 const& forward, v4 const& up, float dist, bool preserve_aspect, bool commit)
	{
		auto bbox = SceneBounds(view3d::ESceneBounds::All, 0, nullptr);
		ResetView(bbox, forward, up, dist, preserve_aspect, commit);
	}

	// Reset the camera to view a bbox
	void V3dWindow::ResetView(BBox const& bbox, v4 const& forward, v4 const& up, float dist, bool preserve_aspect, bool commit)
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
				refresh |= true;
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
				refresh |= true;
		}

		return refresh;
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
		if (FocusPoint() == position)
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
		if (fov == Fov())
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
		if (ViewRectAtDistance(focus_dist) == rect)
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
		if (ClipPlanes(flags) == cp)
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
		if (AlignAxis() == axis)
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

	// Get/Set the global scene light
	Light V3dWindow::GlobalLight() const
	{
		return m_scene.m_global_light;
	}
	void V3dWindow::GlobalLight(Light const& light)
	{
		if (GlobalLight() == light)
			return;

		auto settings = view3d::ESettings::Lighting;
		if (m_scene.m_global_light.m_type != light.m_type) settings |= view3d::ESettings::Lighting_Type;
		if (m_scene.m_global_light.m_position != light.m_position) settings |= view3d::ESettings::Lighting_Position;
		if (m_scene.m_global_light.m_direction != light.m_direction) settings |= view3d::ESettings::Lighting_Direction;
		if (m_scene.m_global_light.m_ambient != light.m_ambient) settings |= view3d::ESettings::Lighting_Colour;
		if (m_scene.m_global_light.m_diffuse != light.m_diffuse) settings |= view3d::ESettings::Lighting_Colour;
		if (m_scene.m_global_light.m_specular != light.m_specular) settings |= view3d::ESettings::Lighting_Colour;
		if (m_scene.m_global_light.m_specular_power != light.m_specular_power) settings |= view3d::ESettings::Lighting_Range;
		if (m_scene.m_global_light.m_range != light.m_range) settings |= view3d::ESettings::Lighting_Range;
		if (m_scene.m_global_light.m_falloff != light.m_falloff) settings |= view3d::ESettings::Lighting_Range;
		if (m_scene.m_global_light.m_inner_angle != light.m_inner_angle) settings |= view3d::ESettings::Lighting_Range;
		if (m_scene.m_global_light.m_outer_angle != light.m_outer_angle) settings |= view3d::ESettings::Lighting_Range;
		if (m_scene.m_global_light.m_cast_shadow != light.m_cast_shadow) settings |= view3d::ESettings::Lighting_Shadows;
		if (m_scene.m_global_light.m_cam_relative != light.m_cam_relative) settings |= view3d::ESettings::Lighting_Position | view3d::ESettings::Lighting_Direction;
		if (m_scene.m_global_light.m_on != light.m_on) settings |= view3d::ESettings::Lighting_All;

		m_scene.m_global_light = light;
		OnSettingsChanged(this, settings);
		Invalidate();
	}

	// Get/Set the global environment map for this window
	TextureCube const* V3dWindow::EnvMap() const
	{
		return m_scene.m_global_envmap.get();
	}
	void V3dWindow::EnvMap(TextureCube* env_map)
	{
		if (EnvMap() == env_map)
			return;

		m_scene.m_global_envmap = TextureCubePtr(env_map, true);

		OnSettingsChanged(this, view3d::ESettings::Scene_EnvMap);
		Invalidate();
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
	void V3dWindow::SetSelectionBox(BBox const& bbox, m3x4 const& ori)
	{
		if (bbox == BBox::Reset())
		{
			// Flag to not include the selection box
			m_selection_box.m_i2w.pos.w = 0;
		}
		else
		{
			m_selection_box.m_i2w =
				m4x4(ori, v4Origin) *
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
				m_anim_data.m_thread = std::jthread([this, issue, rate]
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
							m_anim_data.m_clock.store(system_clock::now() - time0);
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
		auto const invalid = view3d::HitTestResult{.m_distance = maths::float_max};
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
	view3d::HitTestRayId V3dWindow::HitTestRayUpdate(pr::view3d::HitTestRayId id, view3d::HitTestRay const* ray)
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
			auto ldr_obj = cast<ldraw::LdrObject>(target.m_instance);
			auto bbox = ldr_obj->BBoxWS(ldraw::EBBoxFlags::IncludeChildren);
			FocusPoint(bbox.m_centre);
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
	std::tuple<BBox, m3x4> V3dWindow::SelectionBox() const
	{
		if (m_selection_box.m_i2w.pos.w == 0)
			return { BBox::Reset(), m3x4::Identity() };

		auto const& i2w = m_selection_box.m_i2w;
		auto bbox = BBox(i2w.pos, v4(Length(i2w.x), Length(i2w.y), Length(i2w.z), 0));
		auto ori = m_selection_box.m_i2w.rot;
		return { bbox, ori };

	}
	void V3dWindow::SelectionBox(BBox const& bbox, m3x4 const& ori)
	{
		auto [b, o] = SelectionBox();
		if (b == bbox && o == ori)
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
		if (FillModePointsSize() == size)
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
			m_ui_lighting.reset(new rdr12::LightingUI(m_hwnd, m_scene.m_global_light));
			m_ui_lighting->HideOnClose(true);
			m_ui_lighting->Commit += [&](rdr12::LightingUI&, Light const& light)
			{
				GlobalLight(light);
			};
			m_ui_lighting->Preview += [&](rdr12::LightingUI&, Light const& light)
			{
				auto prev = m_scene.m_global_light;
				m_scene.m_global_light = light;

				Render();

				m_scene.m_global_light = prev;
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

		auto code = key & EKeyCodes::KeyCode;
		auto modifiers = key & EKeyCodes::Modifiers;
		switch (code)
		{
			case EKeyCodes::F7:
			{
				auto up = LengthSq(m_scene.m_cam.Align()) > maths::tinyf ? m_scene.m_cam.Align() : v4::YAxis();
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