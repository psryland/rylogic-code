//#define PR_VIEW3D_CREATE_STACKTRACE
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Drawing;
using System.Linq;
using System.Runtime.InteropServices;
using Rylogic.Common;
using Rylogic.Interop.Win32;
using Rylogic.Maths;
using Rylogic.Utility;
using HWindow = System.IntPtr;
using HWND = System.IntPtr;

namespace Rylogic.Gfx
{
	public sealed partial class View3d
	{
		/// <summary>Binds a 3D scene to a window</summary>
		public sealed class Window :IDisposable
		{
			private readonly WindowOptions m_opts;              // The options used to create the window (contains references to the user provided error call back)
			private readonly SettingsChangedCB m_settings_cb;   // A local reference to prevent the callback being garbage collected
			private readonly InvalidatedCB m_invalidated_cb;    // A local reference to prevent the callback being garbage collected
			private readonly RenderingCB m_render_cb;           // A local reference to prevent the callback being garbage collected
			private readonly SceneChangedCB m_scene_changed_cb; // A local reference to prevent the callback being garbage collected
			private readonly AnimationCB m_animation_cb;        // A local reference to prevent the callback being garbage collected
			private readonly HitTestAsyncCB m_ht_async_cb;      // A local reference to prevent the callback being garbage collected

			public Window(View3d view, HWND hwnd, WindowOptions? opts = null)
			{
				View = view;
				Hwnd = hwnd;
				Diag = new Diagnostics(this);
				m_opts = opts ?? WindowOptions.New();
				m_opts.ErrorCB = new ReportErrorCB { m_cb = HandleError, m_ctx = IntPtr.Zero };

				// Create the window
				Handle = View3D_WindowCreate(hwnd, ref m_opts);
				if (Handle == IntPtr.Zero) throw new Exception("Failed to create View3D window");
				void HandleError(IntPtr ctx, string msg, string filepath, int line, long pos) => Error?.Invoke(this, new ErrorEventArgs(msg, filepath, line, pos));

				// Set up a callback for when settings are changed
				View3D_WindowSettingsChangedCB(Handle, m_settings_cb = new SettingsChangedCB { m_cb = HandleSettingsChanged }, true);
				void HandleSettingsChanged(IntPtr ctx, HWindow wnd, ESettings setting) => OnSettingsChanged?.Invoke(this, new SettingChangeEventArgs(setting));

				// Set up a callback for when the window is invalidated
				View3D_WindowInvalidatedCB(Handle, m_invalidated_cb = new InvalidatedCB { m_cb = HandleInvalidated }, true);
				void HandleInvalidated(IntPtr ctx, HWindow wnd) => OnInvalidated?.Invoke(this, EventArgs.Empty);

				// Set up a callback for when a render is about to happen
				View3D_WindowRenderingCB(Handle, m_render_cb = new RenderingCB { m_cb = HandleRendering }, true);
				void HandleRendering(IntPtr ctx, HWindow wnd) => OnRendering?.Invoke(this, EventArgs.Empty);

				// Set up a callback for when the object store for this window changes
				View3D_WindowSceneChangedCB(Handle, m_scene_changed_cb = new SceneChangedCB { m_cb = HandleSceneChanged }, true);
				void HandleSceneChanged(IntPtr ctx, HWindow wnd, ref SceneChanged args) => OnSceneChanged?.Invoke(this, new SceneChangedEventArgs(args));

				// Set up a callback for animation events
				View3D_WindowAnimEventCBSet(Handle, m_animation_cb = new AnimationCB { m_cb = HandleAnimationEvent }, true);
				void HandleAnimationEvent(IntPtr ctx, HWindow wnd, EAnimCommand command, double clock) => OnAnimationEvent?.Invoke(this, new AnimationEventArgs(command, clock));

				// Set up a callback for async hit tests
				View3D_HitTestAsyncCBSet(Handle, m_ht_async_cb = new HitTestAsyncCB { m_cb = HandleHitTestAsyncResult }, true);
				void HandleHitTestAsyncResult(IntPtr ctx, HWindow wnd, HitTestResult[] results, int count) => OnHitTestAsyncResult?.Invoke(this, new HitTestAsyncResultEventArgs(results));

				// Set up the light source
				SetLightSource(v4.Origin, -v4.ZAxis, true);

				// Display the focus point
				FocusPointVisible = true;

				// Position the camera
				Camera = new Camera(this);
				Camera.Lookat(new v4(0f, 0f, -2f, 1f), v4.Origin, v4.YAxis);
			}
			public void Dispose()
			{
				Util.BreakIf(Util.IsGCFinalizerThread, "Disposing in the GC finalizer thread");
				if (Handle == HWindow.Zero)
					return;

				View3D_WindowAnimControl(Handle, EAnimCommand.Stop, 0.0);
				View3D_WindowAnimEventCBSet(Handle, m_animation_cb, false);
				View3D_WindowSceneChangedCB(Handle, m_scene_changed_cb, false);
				View3D_WindowRenderingCB(Handle, m_render_cb, false);
				View3D_WindowInvalidatedCB(Handle, m_invalidated_cb, false);
				View3D_WindowSettingsChangedCB(Handle, m_settings_cb, false);
				View3D_WindowDestroy(Handle);
				Handle = HWindow.Zero;
			}

			/// <summary>Event call on errors. Note: can be called in a background thread context</summary>
			public event EventHandler<ErrorEventArgs>? Error;

			/// <summary>Event notifying whenever rendering settings have changed</summary>
			public event EventHandler<SettingChangeEventArgs>? OnSettingsChanged;

			/// <summary>Raised when Invalidate is called</summary>
			public event EventHandler? OnInvalidated;

			/// <summary>Event notifying of a render about to happen</summary>
			public event EventHandler? OnRendering;

			/// <summary>Event notifying whenever objects are added/removed from the scene (Not raised during Render however)</summary>
			public event EventHandler<SceneChangedEventArgs>? OnSceneChanged;

			/// <summary>Event notifying of state changes for animation</summary>
			public event EventHandler<AnimationEventArgs>? OnAnimationEvent;

			/// <summary>Respond to Win32 messages related to window resizing, minimising, etc</summary>
			public void WndProcFilter(object? _, WndProcEventArgs args)
			{
				if (args.Hwnd != Hwnd)
					throw new Exception("Window handle mismatch");
				if (args.Handled)
					return;

				switch (args.Message)
				{
					case Win32.WM_WINDOWPOSCHANGED:
					{
						var wp = Marshal.PtrToStructure<Win32.WINDOWPOS>(args.LParam);
						if (!wp.flags.HasFlag(Win32.EWindowPos.NoSize))
						{
							var dpi = DpiScale;
							var sz = new Size((int)(wp.cx * dpi.X), (int)(wp.cy * dpi.Y));
							BackBufferSize = sz;
							Viewport = new Viewport(0, 0, sz.Width, sz.Height, wp.cx, wp.cy, 0f, 1f);
							Camera.Aspect = Viewport.Aspect;
							Invalidate();

							args.Handled = true;
						}
						break;
					}
					case Win32.WM_LBUTTONDOWN:
					case Win32.WM_RBUTTONDOWN:
					case Win32.WM_LBUTTONUP:
					case Win32.WM_RBUTTONUP:
					{
						if (!DefaultNavigation)
							break;

						var pt = Win32.LParamToPoint(args.LParam);
						var mk = Win32.ToMouseKey(args.WParam);
						MouseNavigate(pt, mk, true);
						args.Handled = true;
						break;
					}
					case Win32.WM_MBUTTONUP:
					{
						if (!DefaultNavigation)
							break;

						// Middle-click moves the focus point to the object under the cursor
						var pt = Win32.LParamToPoint(args.LParam);
						TranslateKey(EKeyCodes.MButton, v2.From(pt));
						Invalidate();
						args.Handled = true;
						break;
					}
					case Win32.WM_MOUSEMOVE:
					{
						if (!DefaultNavigation)
							break;

						var pt = Win32.LParamToPoint(args.LParam);
						var mk = Win32.ToMouseKey(args.WParam);
						MouseNavigate(pt, mk, false);
						args.Handled = true;
						break;
					}
					case Win32.WM_MOUSEWHEEL:
					{
						if (!DefaultNavigation)
							break;

						var pt = Win32.POINT.FromPoint(Win32.LParamToPoint(args.LParam));
						User32.ScreenToClient(Hwnd, ref pt);
						var mk = Win32.ToMouseKey(args.WParam);
						var delta = Win32.ToMouseWheelDelta(args.WParam);
						
						MouseNavigateZ(pt.ToPoint(), mk, delta, true);
						args.Handled = true;
						break;
					}
				}
			}

			/// <summary>Triggers the 'OnInvalidated' event on the first call. Future calls are ignored until 'Present' or 'Validate' are called</summary>
			public void Invalidate()
			{
				View3D_WindowInvalidate(Handle, false);
			}
			public void Validate()
			{
				View3D_WindowValidate(Handle);
			}

			/// <summary>The associated view3d object</summary>
			public View3d View { get; }

			/// <summary>Get the view3d native handle for the window (Note: this is not the HWND)</summary>
			public HWindow Handle { get; private set; }

			/// <summary>Get the windows handle associated with this window (Note: Null if using off-screen rendering only)</summary>
			public HWND Hwnd { get; }

			/// <summary>Camera controls</summary>
			public Camera Camera { get; }

			/// <summary>Handle navigation in WndProcFilter</summary>
			public bool DefaultNavigation { get; set; }

			/// <summary>
			/// Mouse navigation and/or object manipulation.
			/// 'point' is a point in client space.
			/// 'btns' is the state of the mouse buttons (MK_LBUTTON etc)
			/// 'nav_op' is the logical navigation operation to perform.
			/// 'nav_beg_or_end' should be true on mouse button down or up, and false during mouse movement
			/// Returns true if the scene requires refreshing</summary>
			public bool MouseNavigate(Point point, EMouseBtns btns, ENavOp nav_op, bool nav_beg_or_end)
			{
				// This function is not in the CameraControls object because it is not solely used
				// for camera navigation. It can also be used to manipulate objects in the scene.
				if (m_in_mouse_navigate != 0) return false;
				using (Scope.Create(() => ++m_in_mouse_navigate, () => --m_in_mouse_navigate))
				{
					// Notify of navigating, allowing client code to make
					// changes or optionally handle the mouse event.
					var args = new MouseNavigateEventArgs(point, btns, nav_op, nav_beg_or_end);
					MouseNavigating?.Invoke(this, args);
					if (args.Handled)
						return false;

					// The mouse event wasn't handled, so forward to the window for navigation
					return View3D_MouseNavigate(Handle, v2.From(args.Point), args.NavOp, args.NavBegOrEnd);
				}
			}
			public bool MouseNavigate(Point point, EMouseBtns btns, bool nav_beg_or_end)
			{
				var op = Camera.MouseBtnToNavOp(btns);
				return MouseNavigate(point, btns, op, nav_beg_or_end);
			}
			private int m_in_mouse_navigate;

			/// <summary>
			/// Zoom using the mouse.
			/// 'point' is a point in client rect space.
			/// 'delta' is the mouse wheel scroll delta value (i.e. 120 = 1 click)
			/// 'along_ray' is true if the camera should move along a ray cast through 'point'
			/// Returns true if the scene requires refreshing</summary>
			public bool MouseNavigateZ(Point point, EMouseBtns btns, float delta, bool along_ray)
			{
				if (m_in_mouse_navigate != 0) return false;
				using (Scope.Create(() => ++m_in_mouse_navigate, () => --m_in_mouse_navigate))
				{
					// Notify of navigating, allowing client code to make
					// changes or optionally handle the mouse event.
					var args = new MouseNavigateEventArgs(point, btns, delta, along_ray);
					MouseNavigating?.Invoke(this, args);
					if (args.Handled)
						return false;

					// The mouse event wasn't handled, so forward to the window for navigation
					return View3D_MouseNavigateZ(Handle, v2.From(args.Point), args.Delta, args.AlongRay);
				}
			}

			/// <summary>
			/// Direct camera relative navigation or manipulation.
			/// Returns true if the scene requires refreshing</summary>
			public bool Navigate(float dx, float dy, float dz)
			{
				// This function is not in the CameraControls object because it is not solely used
				// for camera navigation. It can also be used to manipulate objects in the scene.
				return View3D_Navigate(Handle, dx, dy, dz);
			}

			/// <summary>Raised just before a mouse navigation happens</summary>
			public event EventHandler<MouseNavigateEventArgs>? MouseNavigating;

			/// <summary>Perform a hit test in the scene</summary>
			public HitTestResult HitTest(HitTestRay ray)
			{
				return HitTest(ray, x => true);
			}
			public HitTestResult HitTest(HitTestRay ray, Func<Guid, bool> context_pred)
			{
				var rays = new HitTestRay[1] { ray };
				var hits = new HitTestResult[1];

				using var rays_buf = Marshal_.Pin(rays, GCHandleType.Pinned);
				using var hits_buf = Marshal_.Pin(hits, GCHandleType.Pinned);

				bool CB(IntPtr ctx, ref Guid context_id) => context_pred(context_id);
				View3D_WindowHitTestByCtx(Handle, rays_buf.Pointer, hits_buf.Pointer, 1, new GuidPredCB { m_cb = CB });

				return hits[0];
			}
			public HitTestResult HitTest(HitTestRay ray, IEnumerable<Object> objects)
			{
				var rays = new HitTestRay[1] { ray };
				var hits = new HitTestResult[1];
				var insts = objects.Select(x => x.Handle).ToArray();

				using var rays_buf = Marshal_.Pin(rays, GCHandleType.Pinned);
				using var hits_buf = Marshal_.Pin(hits, GCHandleType.Pinned);
				using var insts_buf = Marshal_.Pin(insts, GCHandleType.Pinned);
				View3D_WindowHitTestObjects(Handle, rays_buf.Pointer, hits_buf.Pointer, 1, insts, insts.Length);

				return hits[0];
			}

			/// <summary>Event fired when async hit test results are available (may fire on a background thread)</summary>
			public event EventHandler<HitTestAsyncResultEventArgs>? OnHitTestAsyncResult;

			/// <summary>Add/Update/Remove an async hit test ray. Use 'id == none' to add a new ray. Use 'ray == null' to remove a ray</summary>
			public HitTestRayId HitTestAsyncRay(HitTestRayId id, HitTestRay? ray, int x, bool trigger_hit_test = true)
			{
				if (ray is HitTestRay r)
				{
					id = View3D_HitTestRayUpdate(Handle, id, ref r);
					if (id == HitTestRayId.None)
						return HitTestRayId.None;

					if (trigger_hit_test)
						View3D_HitTestAsync(Handle);
				}
				else
				{
					id = View3D_HitTestRayUpdate(Handle, id, IntPtr.Zero);
				}

				return id;
			}

			/// <summary>Get the render target texture</summary>
			public BackBuffer BackBuffer => View3D_WindowRenderTargetGet(Handle);

			/// <summary>The size of the render target (in pixels)</summary>
			public Size RenderTargetSize => BackBuffer.Dim;

			/// <summary>The DPI of the monitor this window is on</summary>
			public PointF DpiScale => View3D_WindowDpiScale(Handle).ToPointF();

			/// <summary>Import/Export a settings string</summary>
			public string Settings
			{
				get => View3D_WindowSettingsGetBStr(Handle) ?? string.Empty;
				set => View3D_WindowSettingsSet(Handle, value);
			}

			/// <summary>Enumerate the GUIDs associated with this window</summary>
			public void EnumGuids(Action<Guid> cb)
			{
				EnumGuids(guid => { cb(guid); return true; });
			}
			public void EnumGuids(Func<Guid, bool> cb)
			{
				bool CB(IntPtr ctx, ref Guid context_id) => cb(context_id);
				View3D_WindowEnumGuids(Handle, new EnumGuidsCB { m_cb = CB });
			}

			/// <summary>Enumerate the objects associated with this window</summary>
			public void EnumObjects(Action<Object> cb)
			{
				EnumObjects(obj => { cb(obj); return true; });
			}
			public void EnumObjects(Func<Object, bool> cb)
			{
				bool CB(IntPtr ctx, IntPtr obj) => cb(new Object(obj));
				View3D_WindowEnumObjects(Handle, new EnumObjectsCB { m_cb = CB });
			}
			public void EnumObjects(Action<Object> cb, Func<Guid, bool> context_pred)
			{
				EnumObjects(obj => { cb(obj); return true; }, context_pred);
			}
			public void EnumObjects(Func<Object, bool> cb, Func<Guid, bool> context_pred)
			{
				bool ObjCB(IntPtr c, IntPtr obj) => cb(new Object(obj));
				bool PredCB(IntPtr c, ref Guid guid) => context_pred(guid);
				View3D_WindowEnumObjectsById(Handle, new EnumObjectsCB { m_cb = ObjCB }, new GuidPredCB { m_cb = PredCB });
			}

			/// <summary>Return the objects associated with this window</summary>
			public Object[] Objects
			{
				get
				{
					var objs = new List<Object>();
					EnumObjects(x => objs.Add(x));
					return [.. objs];
				}
			}

			/// <summary>Add an object to the window</summary>
			public void AddObject(Object obj)
			{
				Util.BreakIf(!Math_.FEql(obj.O2P.w.w, 1f), "Invalid instance transform");
				Util.BreakIf(!Math_.IsFinite(obj.O2P), "Invalid instance transform");
				View3D_WindowAddObject(Handle, obj.Handle);
			}

			/// <summary>Add a gizmo to the window</summary>
			public void AddGizmo(Gizmo giz)
			{
				Util.BreakIf(!Math_.FEql(giz.O2W.w.w, 1f), "Invalid instance transform");
				Util.BreakIf(!Math_.IsFinite(giz.O2W), "Invalid instance transform");
				View3D_WindowAddGizmo(Handle, giz.m_handle);
			}

			/// <summary>Add multiple objects, filtered by 'context_ids</summary>
			public void AddObjects(Guid context_id)
			{
				AddObjects(x => x == context_id);
			}
			public void AddObjects(Func<Guid, bool> context_pred)
			{
				bool PredCB(IntPtr c, ref Guid guid) => context_pred(guid);
				View3D_WindowAddObjectsById(Handle, new GuidPredCB { m_cb = PredCB });
			}

			/// <summary>Add a collection of objects to the window</summary>
			public void AddObjects(IEnumerable<Object> objects)
			{
				foreach (var obj in objects)
					AddObject(obj);
			}

			/// <summary>Remove an object from the window</summary>
			public void RemoveObject(Object obj)
			{
				View3D_WindowRemoveObject(Handle, obj.Handle);
			}

			/// <summary>Remove a gizmo from the window</summary>
			public void RemoveGizmo(Gizmo giz)
			{
				View3D_WindowRemoveGizmo(Handle, giz.m_handle);
			}

			/// <summary>Remove a collection of objects from the window</summary>
			public void RemoveObjects(IEnumerable<Object> objects)
			{
				foreach (var obj in objects)
					RemoveObject(obj);
			}

			/// <summary>Remove multiple objects, filtered by 'context_ids'</summary>
			public void RemoveObjects(Func<Guid, bool> context_pred)
			{
				bool PredCB(IntPtr c, ref Guid guid) => context_pred(guid);
				View3D_WindowRemoveObjectsById(Handle, new GuidPredCB { m_cb = PredCB });
			}

			/// <summary>Remove all instances from the window</summary>
			public void RemoveAllObjects()
			{
				View3D_WindowRemoveAllObjects(Handle);
			}

			/// <summary>Return the number of objects in a window</summary>
			public int ObjectCount => View3D_WindowObjectCount(Handle);

			/// <summary>True if 'obj' is a member of this window</summary>
			public bool HasObject(Object obj, bool search_children)
			{
				return View3D_WindowHasObject(Handle, obj.Handle, search_children);
			}
			[Obsolete("Use HasObject")] public bool Contains(Object obj, bool search_children) => HasObject(obj, search_children);

			/// <summary>Return a bounding box of the objects in this window</summary>
			public BBox SceneBounds(ESceneBounds bounds, Guid[]? except = null)
			{
				return View3D_WindowSceneBounds(Handle, bounds, except?.Length ?? 0, except);
			}

			/// <summary>Show/Hide the focus point</summary>
			public bool FocusPointVisible
			{
				get => View3D_StockObjectVisibleGet(Handle, EStockObject.FocusPoint);
				set => View3D_StockObjectVisibleSet(Handle, EStockObject.FocusPoint, value);
			}

			/// <summary>Show/Hide the origin point</summary>
			public bool OriginPointVisible
			{
				get => View3D_StockObjectVisibleGet(Handle, EStockObject.OriginPoint);
				set => View3D_StockObjectVisibleSet(Handle, EStockObject.OriginPoint, value);
			}

			/// <summary>Get/Set whether the selection box is visible</summary>
			public bool SelectionBoxVisible
			{
				get => View3D_StockObjectVisibleGet(Handle, EStockObject.SelectionBox);
				set => View3D_StockObjectVisibleSet(Handle, EStockObject.SelectionBox, value);
			}

			/// <summary>Set the size of the focus point graphic</summary>
			public float FocusPointSize
			{
				get => View3D_FocusPointSizeGet(Handle);
				set => View3D_FocusPointSizeSet(Handle, value);
			}

			/// <summary>Set the size of the origin graphic</summary>
			public float OriginPointSize
			{
				get => View3D_OriginPointSizeGet(Handle);
				set => View3D_OriginPointSizeSet(Handle, value);
			}

			/// <summary>Set the position of the selection box</summary>
			public (BBox, m4x4) SelectionBox
			{
				get { View3D_SelectionBoxGet(Handle, out var box, out var o2w); return (box, o2w); }
				set => View3D_SelectionBoxSet(Handle, ref value.Item1, ref value.Item2);
			}

			/// <summary>Set the size and position of the selection box to bound the selected objects in this view</summary>
			public void SelectionBoxFitToSelected()
			{
				View3D_SelectionBoxFitToSelected(Handle);
			}

			/// <summary>Get/Set the render mode</summary>
			public EFillMode FillMode
			{
				get => View3D_WindowFillModeGet(Handle);
				set => View3D_WindowFillModeSet(Handle, value);
			}

			/// <summary>Get/Set the face culling mode</summary>
			public ECullMode CullMode
			{
				get => View3D_WindowCullModeGet(Handle);
				set => View3D_WindowCullModeSet(Handle, value);
			}

			/// <summary>Get/Set the multi-sampling level for the window</summary>
			public int MultiSampling
			{
				get => View3D_MultiSamplingGet(Handle);
				set => View3D_MultiSamplingSet(Handle, value);
			}

			/// <summary>Get/Set the light properties. Note returned value is a value type</summary>
			public LightInfo LightProperties
			{
				get => View3D_LightPropertiesGet(Handle);
				set => View3D_LightPropertiesSet(Handle, ref value);
			}

			/// <summary>Show the lighting dialog</summary>
			public void ShowLightingDlg()
			{
				throw new NotImplementedException();
				//todo View3D_LightShowDialog(Handle);
			}

			/// <summary>Set the single light source</summary>
			public void SetLightSource(v4 position, v4 direction, bool camera_relative)
			{
				View3D_LightSource(Handle, position, direction, camera_relative);
			}

			/// <summary>Show/Hide the object manager tool</summary>
			public bool ObjectManagerTool
			{
				get => View3D_ObjectManagerVisibleGet(Handle);
				set => View3D_ObjectManagerVisibleSet(Handle, value);
			}

			/// <summary>Show/Hide the scripr editor tool</summary>
			public bool ScriptTool
			{
				get => View3D_ScriptEditorVisibleGet(Handle);
				set => View3D_ScriptEditorVisibleSet(Handle, value);
			}

			/// <summary>Show/Hide the measuring tool</summary>
			public bool ShowMeasureTool
			{
				get => View3D_MeasureToolVisibleGet(Handle);
				set => View3D_MeasureToolVisibleSet(Handle, value);
			}

			/// <summary>Show/Hide the angle tool</summary>
			public bool ShowAngleTool
			{
				get => View3D_AngleToolVisibleGet(Handle);
				set => View3D_AngleToolVisibleSet(Handle, value);
			}

			/// <summary>The background colour for the window</summary>
			public Colour32 BackgroundColour
			{
				get => new(View3D_WindowBackgroundColourGet(Handle));
				set => View3D_WindowBackgroundColourSet(Handle, value.ARGB);
			}

			/// <summary>Get/Set the animation clock</summary>
			public double AnimTime
			{
				get => View3D_WindowAnimTimeGet(Handle);
				set => View3D_WindowAnimTimeSet(Handle, value);
			}

			/// <summary>Is animation currently running</summary>
			public bool Animating => View3D_WindowAnimating(Handle);

			/// <summary>Control animation</summary>
			public void AnimControl(EAnimCommand command, double time_s = 0.0)
			{
				View3D_WindowAnimControl(Handle, command, time_s);
			}

			/// <summary>Cause the window to be rendered. Remember to call Present when done</summary>
			public void Render()
			{
				View3D_WindowRender(Handle);
			}

			/// <summary>Wait for submitted commands to complete on the GPU</summary>
			public void GSyncWait()
			{
				View3D_WindowGSyncWait(Handle);
			}

			/// <summary>Get/Set the size of the swap chain back buffer (in pixels) (Note: *not* the MSAA back buffer)</summary>
			[Browsable(false)]
			public Size BackBufferSize
			{
				get => View3D_WindowBackBufferSizeGet(Handle);
				set => View3D_WindowBackBufferSizeSet(Handle, value, false);
			}

			/// <summary>Force recreate the back buffer with the given size</summary>
			public void RecreateBackBuffer(Size size)
			{
				View3D_WindowBackBufferSizeSet(Handle, size, true);
			}

			/// <summary>Replace the swap chain with 'swap_chain'</summary>
			public void CustomSwapChain(Texture[] swap_chain)
			{
				var handles = swap_chain.Select(x => x.Handle).ToArray();
				View3D_WindowCustomSwapChain(Handle, swap_chain.Length, handles);
			}

			/// <summary>Get/Set the size/position of the viewport within the render target</summary>
			public Viewport Viewport
			{
				get => View3D_WindowViewportGet(Handle);
				set
				{
					Util.BreakIf(value.Width == 0 || value.Height == 0, "Invalid viewport size");
					Util.BreakIf(value.ScreenW == 0 || value.ScreenH == 0, "Invalid viewport size");
					Util.BreakIf(!Math_.IsFinite(value.Width) || !Math_.IsFinite(value.Height), "Invalid viewport size");
					View3D_WindowViewportSet(Handle, ref value);
				}
			}

			/// <summary>Get/Set whether the depth buffer is enabled</summary>
			public bool DepthBufferEnabled
			{
				get => View3D_DepthBufferEnabledGet(Handle);
				set => View3D_DepthBufferEnabledSet(Handle, value);
			}

			/// <summary>Convert a length in pixels into a length in normalised screen space</summary>
			public v2 PixelsToNSS(v2 pixels)
			{
				return View3D_PixelsToNSS(Handle, pixels);
			}

			/// <summary>Convert a screen space point to a normalised point</summary>
			public v2 SSPointToNSSPoint(v2 screen)
			{
				return View3D_SSPointToNSSPoint(Handle, screen);
			}

			/// <summary>Convert a normalised point into a screen space point</summary>
			public v2 ScreenSpacePoint(v2 pt)
			{
				// Notes:
				//  - What does screen space mean if there is no back buffer?
				//    For WPF, or other window-less modes of use, there is no back buffer. I'm assuming here that the render target
				//    is the pseudo-back buffer. In the case were there is a back buffer but a temporary render target is in use, I'm
				//    using the back buffer still as I'm assuming that is still the screen size.
				//  - The render target dimension is in pixels however, and WPF uses device independent units (i.e. 96dpi).
				//    So to return points in device-independent screen space, the render target size needs to be adjusted
				//    by the DPI.
				var dpi_scale = DpiScale;
				var da = Hwnd != IntPtr.Zero ? BackBufferSize : RenderTargetSize;
				return new v2(
					(pt.x + 1f) * da.Width / (2f * dpi_scale.X),
					(1f - pt.y) * da.Height / (2f * dpi_scale.Y));
			}

			/// <summary>Standard keyboard shortcuts. 'key_code' corresponds to VK_KEY. 'ss_point' is the screen space mouse position (in pixels)</summary>
			public bool TranslateKey(EKeyCodes key_code, v2 ss_point)
			{
				return View3D_TranslateKey(Handle, key_code, ss_point);
			}

			/// <summary>Handy method for creating random objects</summary>
			public Guid CreateDemoScene()
			{
				return View3D_DemoSceneCreateText(Handle);

#if false
				{// Create an object using ldr script
				    HObject obj = ObjectCreate("*Box ldr_box FFFF00FF {1 2 3}");
				    DrawsetAddObject(obj);
				}

				{// Create a box model, an instance for it, and add it to the window
				    HModel model = CreateModelBox(new v4(0.3f, 0.2f, 0.4f, 0f), m4x4.Identity, 0xFFFF0000);
				    HInstance instance = CreateInstance(model, m4x4.Identity);
				    AddInstance(window, instance);
				}

				{// Create a mesh
				    // Mesh data
				    Vertex[] vert = new Vertex[]
				    {
				        new Vertex(new v4( 0f, 0f, 0f, 1f), v4.ZAxis, 0xFFFF0000, v2.Zero),
				        new Vertex(new v4( 0f, 1f, 0f, 1f), v4.ZAxis, 0xFF00FF00, v2.Zero),
				        new Vertex(new v4( 1f, 0f, 0f, 1f), v4.ZAxis, 0xFF0000FF, v2.Zero),
				    };
				    ushort[] face = new ushort[]
				    {
				        0, 1, 2
				    };

				    HModel model = CreateModel(vert.Length, face.Length, vert, face, EPrimType.D3DPT_TRIANGLELIST);
				    HInstance instance = CreateInstance(model, m4x4.Identity);
				    AddInstance(window, instance);
				}
#endif
			}

			/// <summary>Show a window containing and example ldr script file</summary>
			public void ShowExampleScript()
			{
				View3D_DemoScriptShow(Handle);
			}

			/// <summary>Diagnostic settings</summary>
			public Diagnostics Diag { get; }

			/// <summary>Namespace for diagnostic settings</summary>
			public class Diagnostics
			{
				private readonly Window m_window;
				internal Diagnostics(Window window)
				{
					m_window = window;
				}

				/// <summary>Get/Set whether object bounding boxes are visible</summary>
				public bool BBoxesVisible
				{
					get => View3D_DiagBBoxesVisibleGet(m_window.Handle);
					set => View3D_DiagBBoxesVisibleSet(m_window.Handle, value);
				}

				/// <summary>Get/Set the length of vertex normals (when visible)</summary>
				public float NormalsLength
				{
					get => View3D_DiagNormalsLengthGet(m_window.Handle);
					set => View3D_DiagNormalsLengthSet(m_window.Handle, value);
				}

				/// <summary>Get/Set the length of vertex normals (when visible)</summary>
				public Colour32 NormalsColour
				{
					get => View3D_DiagNormalsColourGet(m_window.Handle);
					set => View3D_DiagNormalsColourSet(m_window.Handle, value);
				}

				/// <summary>Get/Set the size of the points in EFillMode::Points mode</summary>
				public v2 FillModePointsSize
				{
					get => View3D_DiagFillModePointsSizeGet(m_window.Handle);
					set => View3D_DiagFillModePointsSizeSet(m_window.Handle, value);
				}
			}

			#region Equals
			public static bool operator == (Window? lhs, Window? rhs)
			{
				return ReferenceEquals(lhs,rhs) || Equals(lhs, rhs);
			}
			public static bool operator != (Window? lhs, Window? rhs)
			{
				return !(lhs == rhs);
			}
			public bool Equals(Window? rhs)
			{
				return rhs != null && Handle == rhs.Handle;
			}
			public override bool Equals(object? rhs)
			{
				return Equals(rhs as Window);
			}
			public override int GetHashCode()
			{
				return Handle.GetHashCode();
			}
			#endregion
		}
	}
}
