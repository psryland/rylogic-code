//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#pragma once
#include "pr/view3d-12/forward.h"
#include "pr/view3d-12/main/window.h"
#include "pr/view3d-12/instance/instance.h"
#include "pr/view3d-12/scene/scene.h"
#include "pr/view3d-12/utility/ray_cast.h"
#include "pr/view3d-12/lighting/light_ui.h"
#include "view3d-12/src/dll/dll_forward.h"
#include "view3d-12/src/ldraw/sources/ldraw_sources.h"

namespace pr::rdr12::ldraw
{
	struct ScriptEditorUI;
	struct MeasureUI;
	struct AngleUI;
	class ObjectManagerUI;
}

namespace pr::rdr12
{
	struct V3dWindow
	{
		// Notes:
		//  - Combines a renderer Window with a collection of LdrObjects

		struct Instance
		{
			#define PR_RDR_INST(x)\
			x(m4x4     ,m_i2w   ,EInstComp::I2WTransform)\
			x(ModelPtr ,m_model ,EInstComp::ModelPtr)\
			x(Colour32 ,m_tint  ,EInstComp::TintColour32)
			PR_RDR12_INSTANCE_MEMBERS(Instance, PR_RDR_INST); // An instance type for other models used in LDraw
			#undef PR_RDR_INST
		};
		struct PointInstance
		{
			#define PR_RDR_INST(x)\
			x(m4x4     ,m_c2s   ,EInstComp::C2STransform)\
			x(m4x4     ,m_i2w   ,EInstComp::I2WTransform)\
			x(ModelPtr ,m_model ,EInstComp::ModelPtr)\
			x(Colour32 ,m_tint  ,EInstComp::TintColour32)\
			x(float    ,m_size  ,EInstComp::Float1)
			PR_RDR12_INSTANCE_MEMBERS(PointInstance, PR_RDR_INST) // An instance type for the focus point and origin point models
			#undef PR_RDR_INST
		};
		struct AnimData
		{
			std::jthread m_thread;
			std::atomic_int m_issue;
			std::atomic<seconds_t> m_clock;
			AnimData() :m_thread() ,m_issue() ,m_clock() {}
			explicit operator bool() const { return m_thread.joinable(); }
		};

		using LightingUIPtr = std::unique_ptr<LightingUI>;
		using LdrObjectManagerUIPtr = std::unique_ptr<ldraw::ObjectManagerUI>;
		using ScriptEditorUIPtr = std::unique_ptr<ldraw::ScriptEditorUI>;
		using LdrMeasureUIPtr = std::unique_ptr<ldraw::MeasureUI>;
		using LdrAngleUIPtr = std::unique_ptr<ldraw::AngleUI>;
		using HitTestRays = vector<rdr12::HitTestRay, 1>;
		using HitTestResults = vector<view3d::HitTestResult, 1>;

		// Renderer window/scene
		Renderer* m_rdr; // The main renderer
		HWND m_hwnd;     // The associated Win32 window handle
		Window m_wnd;    // The renderer window
		Scene m_scene;   // Use one scene for the window

		// Objects
		ObjectSet m_objects; // References to objects to draw (note: objects are owned by the context, not the window)
		GizmoSet m_gizmos;   // References to gizmos to draw (note: objects are owned by the context, not the window)
		GuidSet m_guids;     // The context ids added to this window

		// Stock objects
		PointInstance m_focus_point;     // Focus point graphics
		PointInstance m_origin_point;    // Origin point graphics
		Instance      m_bbox_model;      // Bounding box graphics
		Instance      m_selection_box;   // Selection box graphics
		EStockObject  m_visible_objects; // Visible stock objects

		// Misc
		mutable std::string m_settings;       // Window settings
		AnimData            m_anim_data;      // Animation time in seconds
		HitTestRays         m_ht_rays;        // The set of async hit test rays
		HitTestResults      m_ht_results;     // The results of async hit tests
		mutable pr::BBox    m_bbox_scene;     // Bounding box for all objects in the scene (Lazy updated)
		std::thread::id     m_main_thread_id; // The thread that created this window
		AutoSub             m_eh_hittests;    // Event handler for async hit test results
		bool                m_invalidated;    // True after Invalidate has been called but before Render has been called
		bool                m_ht_invalidated; // True if async hit tests need to be performed
		
		// UI Tools
		LightingUIPtr m_ui_lighting;               // A UI for controlling the lighting of the scene
		LdrObjectManagerUIPtr m_ui_object_manager; // A UI for managing objects within the window
		ScriptEditorUIPtr m_ui_script_editor;      // A UI for editing ldr scripts
		LdrMeasureUIPtr m_ui_measure_tool;         // A UI for measuring distances between points within the 3d environment
		LdrAngleUIPtr m_ui_angle_tool;             // A UI for measuring angles between points within the 3d environment

		V3dWindow(Renderer& rdr, HWND hwnd, view3d::WindowOptions const& opts);
		V3dWindow(V3dWindow&&) = default;
		V3dWindow(V3dWindow const&) = delete;
		V3dWindow& operator=(V3dWindow&&) = default;
		V3dWindow& operator=(V3dWindow const&) = delete;
		~V3dWindow();

		// Renderer access
		Renderer& rdr() const;

		// Error event. Can be called in a worker thread context
		MultiCast<view3d::ReportErrorCB, true> ReportError;

		// Settings changed event
		MultiCast<view3d::SettingsChangedCB, true> OnSettingsChanged;

		// Window invalidated
		MultiCast<view3d::InvalidatedCB, true> OnInvalidated;

		// Rendering event
		MultiCast<view3d::RenderingCB, true> OnRendering;

		// Scene changed event
		MultiCast<view3d::SceneChangedCB, true> OnSceneChanged;

		// Animation event
		MultiCast<view3d::AnimationCB, true> OnAnimationEvent;

		// Async hit test results
		MultiCast<view3d::HitTestAsyncCB, true> OnHitTestAsyncResults;

		// Get/Set the settings
		char const* Settings() const;
		void Settings(char const* settings);

		// The DPI of the monitor that this window is displayed on
		v2 Dpi() const;

		// Get/Set the back buffer size
		iv2 BackBufferSize() const;
		void BackBufferSize(iv2 sz, bool force_recreate);

		// Get/Set the window viewport
		view3d::Viewport Viewport() const;
		void Viewport(view3d::Viewport const& vp);

		// Enumerate the object collection GUIDs associated with this window
		void EnumGuids(view3d::EnumGuidsCB enum_guids_cb);

		// Enumerate the objects associated with this window
		void EnumObjects(view3d::EnumObjectsCB enum_objects_cb);
		void EnumObjects(view3d::EnumObjectsCB enum_objects_cb, view3d::GuidPredCB pred);

		// Return true if 'object' is part of this scene
		bool Has(ldraw::LdrObject const* object, bool search_children) const;
		bool Has(ldraw::LdrGizmo const* gizmo) const;

		// Return the number of objects or object groups in this scene
		int ObjectCount() const;
		int GizmoCount() const;
		int GuidCount() const;

		// Return the bounding box of objects in this scene
		BBox SceneBounds(view3d::ESceneBounds bounds, int except_count, GUID const* except) const;

		// Add/Remove an object to/from this window
		void Add(ldraw::LdrObject* object);
		void Remove(ldraw::LdrObject* object);

		// Add/Remove a gizmo to/from this window
		void Add(ldraw::LdrGizmo* gizmo);
		void Remove(ldraw::LdrGizmo* gizmo);

		// Add/Remove all objects to this window with the given context ids (or not with)
		void Add(ldraw::SourceCont const& sources, view3d::GuidPredCB pred);
		void Remove(view3d::GuidPredCB pred, bool keep_context_ids = false);

		// Remove all objects from this scene
		void RemoveAllObjects();

		// Render this window into whatever render target is currently set
		void Render();

		// Wait for any previous frames to complete rendering within the GPU
		void GSyncWait() const;

		// Replace the swap chain buffers
		void CustomSwapChain(std::span<BackBuffer> back_buffers);
		void CustomSwapChain(std::span<Texture2D*> back_buffers);

		// Get/Set the render target for this window
		rdr12::BackBuffer const& RenderTarget() const;
		rdr12::BackBuffer& RenderTarget();

		// Call InvalidateRect on the HWND associated with this window
		void InvalidateRect(RECT const* rect, bool erase = false);
		void Invalidate(bool erase = false);

		// Clear the invalidated state for the window
		void Validate();
		
		// Reset the scene camera, using it's current forward and up directions, to view all objects in the scene
		void ResetView();

		// Reset the scene camera to view all objects in the scene
		void ResetView(v4 forward, v4 up, float dist = 0.0f, bool preserve_aspect = true, bool commit = true);

		// Reset the camera to view a bbox
		void ResetView(BBox const& bbox, v4 forward, v4 up, float dist = 0.0f, bool preserve_aspect = true, bool commit = true);

		// General mouse navigation
		// 'ss_pos' is the mouse pointer position in 'window's screen space
		// 'nav_op' is the navigation type
		// 'nav_start_or_end' should be TRUE on mouse down/up events, FALSE for mouse move events
		// void OnMouseDown(UINT nFlags, CPoint point) { View3D_MouseNavigate(win, point, nav_op, TRUE); }
		// void OnMouseMove(UINT nFlags, CPoint point) { View3D_MouseNavigate(win, point, nav_op, FALSE); } if 'nav_op' is None, this will have no effect
		// void OnMouseUp  (UINT nFlags, CPoint point) { View3D_MouseNavigate(win, point, 0, TRUE); }
		// BOOL OnMouseWheel(UINT nFlags, short zDelta, CPoint) { if (nFlags == 0) View3D_MouseNavigateZ(win, 0, 0, zDelta / 120.0f); return TRUE; }
		bool MouseNavigate(v2 ss_point, camera::ENavOp nav_op, bool nav_start_or_end);
		bool MouseNavigateZ(v2 ss_point, float delta, bool along_ray);

		// Get/Set the window background colour
		Colour BackgroundColour() const;
		void BackgroundColour(Colour colour);
			
		// Get/Set the window fill mode
		EFillMode FillMode() const;
		void FillMode(EFillMode fill_mode);

		// Get/Set the window cull mode
		ECullMode CullMode() const;
		void CullMode(ECullMode cull_mode);

		// Enable/Disable orthographic projection
		bool Orthographic() const;
		void Orthographic(bool on);

		// Get/Set the distance to the camera focus point
		float FocusDistance() const;
		void FocusDistance(float dist);

		// Get/Set the camera focus point position
		v4 FocusPoint() const;
		void FocusPoint(v4 position);

		// Get/Set the camera focus point bounds
		BBox FocusBounds() const;
		void FocusBounds(BBox bounds);

		// Get/Set the aspect ratio for the camera field of view
		float Aspect() const;
		void Aspect(float aspect);

		// Get/Set the camera field of view. null means don't change
		v2 Fov() const;
		void Fov(v2 fov);

		// Adjust the FocusDist, FovX, and FovY so that the average FOV equals 'fov'
		void BalanceFov(float fov);

		// Get/Set (using fov and focus distance) the size of the perpendicular area visible to the camera at 'dist' (in world space). Use 'focus_dist != 0' to set a specific focus distance
		v2 ViewRectAtDistance(float dist) const;
		void ViewRectAtDistance(v2 rect, float focus_dist = 0);

		// Get/Set the near and far clip planes for the camera
		v2 ClipPlanes(view3d::EClipPlanes flags) const;
		void ClipPlanes(float near_, float far_, view3d::EClipPlanes flags);

		// Get/Set the scene camera lock mask
		camera::ELockMask LockMask() const;
		void LockMask(camera::ELockMask mask);

		// Get/Set the camera align axis
		v4 AlignAxis() const;
		void AlignAxis(v4 axis);

		// Reset to the default zoom
		void ResetZoom();

		// Get/Set the FOV zoom
		float Zoom() const;
		void Zoom(float zoom);

		// Get/Set the global scene light
		Light GlobalLight() const;
		void GlobalLight(Light const& light);

		// Get/Set the global environment map for this window
		TextureCube const* EnvMap() const;
		void EnvMap(TextureCube* env_map);

		// Enable/Disable the depth buffer
		bool DepthBufferEnabled() const;
		void DepthBufferEnabled(bool enabled);

		// Set the position and size of the selection box. If 'bbox' is 'BBoxReset' the selection box is not shown
		void SetSelectionBox(BBox const& bbox, m3x4 const& ori = m3x4::Identity());

		// Position the selection box to include the selected objects
		void SelectionBoxFitToSelected();
	
		// Get/Set the window background colour
		int MultiSampling() const;
		void MultiSampling(int multisampling);

		// Control animation
		void AnimControl(view3d::EAnimCommand command, seconds_t time = seconds_t::zero());
	
		// True if animation is currently active
		bool Animating() const;
	
		// Get/Set the value of the animation clock
		seconds_t AnimTime() const;
		void AnimTime(seconds_t clock);

		// Called when the animation time has changed
		void AnimationStep(view3d::EAnimCommand command, seconds_t anim_time);

		// Cast rays into the scene, returning hit info for the nearest intercept for each ray
		void HitTest(std::span<view3d::HitTestRay const> rays, std::span<view3d::HitTestResult> hits, RayCastInstancesCB instances);
		void HitTest(std::span<view3d::HitTestRay const> rays, std::span<view3d::HitTestResult> hits, ldraw::LdrObject const* const* objects, int object_count);
		void HitTest(std::span<view3d::HitTestRay const> rays, std::span<view3d::HitTestResult> hits, view3d::GuidPredCB pred, int);

		// Trigger execution of the async hit test. Submits GPU work and returns immediately.
		// Uses rays added with the 'HitTestRayUpdate' function. Results are returned through the 'OnAsyncHitTestResults' event when the GPU work is complete.
		void HitTestAsync();

		// Add/Update/Remove an async hit test ray.
		// Returns 'HitTestRayId::None' if no more rays can be added.
		// Returns 'id' if the ray was updated/removed successfully, otherwise returns HitTestRayId::None.
		// Use 'ray == nullptr' to remove a ray.
		view3d::HitTestRayId HitTestRayUpdate(view3d::HitTestRayId id, view3d::HitTestRay const* ray);

		// Move the focus point to the hit target
		void CentreOnHitTarget(view3d::HitTestRay const& ray);

		// Get/Set the visibility of one or more stock objects (focus point, origin, selection box, etc)
		bool StockObjectVisible(view3d::EStockObject stock_objects) const;
		void StockObjectVisible(view3d::EStockObject stock_objects, bool vis);

		// Get/Set the size of the focus point
		float FocusPointSize() const;
		void FocusPointSize(float size);

		// Get/Set the size of the origin point
		float OriginPointSize() const;
		void OriginPointSize(float size);

		// Get/Set the position and size of the selection box. If 'bbox' is 'BBox::Reset()' the selection box is not shown
		std::tuple<BBox, m3x4> SelectionBox() const;
		void SelectionBox(BBox const& bbox, m3x4 const& ori);

		// Show/Hide the bounding boxes
		bool BBoxesVisible() const;
		void BBoxesVisible(bool vis);

		// Get/Set the length of the displayed vertex normals
		float NormalsLength() const;
		void NormalsLength(float length);
		
		// Get/Set the colour of the displayed vertex normals
		Colour32 NormalsColour() const;
		void NormalsColour(Colour32 colour);

		// Get/Set the colour of the displayed vertex normals
		v2 FillModePointsSize() const;
		void FillModePointsSize(v2 size);

		// Access the built-in script editor
		ldraw::ScriptEditorUI& EditorUI();

		// Access the built-in lighting controls UI
		LightingUI& LightingUI();

		// Show/Hide the object manager tool
		bool ObjectManagerVisible() const;
		void ObjectManagerVisible(bool show);

		// Show/Hide the script editor tool
		bool ScriptEditorVisible() const;
		void ScriptEditorVisible(bool show);

		// Show/Hide the measurement tool
		bool MeasureToolVisible() const;
		void MeasureToolVisible(bool show);

		// Show/Hide the angle measurement tool
		bool AngleToolVisible() const;
		void AngleToolVisible(bool show);

		// Implements standard key bindings. 'ss_point' is the screen space mouse position (pixels). Returns true if handled.
		bool TranslateKey(EKeyCodes key, v2 ss_point);

	private:

		// Called when objects are added/removed from this window
		void ObjectContainerChanged(view3d::ESceneChanged change_type, std::span<GUID const> context_ids, ldraw::LdrObject* object);

		// Create stock models such as the focus point, origin, etc
		void CreateStockObjects();
	};

	// Validate a window pointer
	void Validate(V3dWindow const* window);
}