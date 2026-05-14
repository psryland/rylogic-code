using System;
using System.ComponentModel;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Input;
using System.Windows.Threading;
using Rylogic.Gfx;
using Rylogic.Utility;
using Rylogic.Windows.Extn;

namespace Rylogic.Gui.WPF
{
	public partial class ChartControl :IChartCMenu, IView3dCMenu, IChartCMenuContext, IView3dCMenuContext
	{
		// Notes:
		//  - Many commands are available on View3dControl but I want to make the Options the source
		//    of truth for the scene settings, so the scene commands are implemented again here, usually
		//    setting the Options value first then forwarding to the View3dControl command.
		//  - Boolean properties can't forward the ICommand to View3d because the property changed
		//    notification occurs on the wrong object.
		//  - Strictly, 'ChartControl' doesn't need to implement 'IChartCMenuContext, IView3dCMenuContext'
		//    because cmenus should bind to the ChartPanel or a UserControl, but it's a handy place to copy
		//    boilerplate from.
		//
		// Usage:
		//  There are a couple of ways to use cmenus.
		//  1) Use the default chart cmenus by putting this in your XAML:
		//      <gui:ChartControl
		//           SceneCMenu="{StaticResource ChartCMenu}"
		//           XAxisCMenu="{StaticResource ChartAxisCMenu}"
		//           YAxisCMenu="{StaticResource ChartAxisCMenu}"
		//     This will set the DataContext to the ChartPanel and AxisPanels appropriately because the cmenus match the default names.
		//  2) Assign the menus programmatically:
		//      private void InitCMenus()
		//      {
		//          SceneView.Scene.ContextMenu = this.FindCMenu("MyCustomCMenu", this);
		//          SceneView.XAxisPanel.ContextMenu = this.FindCMenu("MyCustomAxisCMenu", SceneView.GetAxisPanel(ChartControl.EAxis.XAxis));
		//          SceneView.YAxisPanel.ContextMenu = this.FindCMenu("MyCustomAxisCMenu", SceneView.GetAxisPanel(ChartControl.EAxis.YAxis));
		//      }
		//     In this case, you don't need anything in your XAML.
		//  3) Create a custom menu resource (that includes MenuItem StaticResources) and have the class containing the ChartControl (i.e.
		//     the DataContext for the UserControl or Window) implement 'IChartCMenuContext' and 'IView3dCMenuContext':
		//      <gui:ChartControl
		//           SceneCMenu="{StaticResource MyCustomCMenu}"
		//           XAxisCMenu="{StaticResource ChartAxisCMenu}"
		//           YAxisCMenu="{StaticResource ChartAxisCMenu}"
		//     The DataContext for 'MyCustomCMenu' will not be assigned (because it's Name isn't the default) so it should fall back to the
		//     DataContext of the containing window/control. Custom AxisCMenu's will need to be bound programmatically, because the CMenu
		//     doesn't know which axis it's attached to.

		/// <summary>Initialise the commands</summary>
		private void InitCommands()
		{
			// Tools Menu
			ToggleOriginPoint = Command.Create(this, ToggleOriginPointInternal);
			ToggleFocusPoint = Command.Create(this, ToggleFocusPointInternal);
			ToggleBBoxesVisible = Command.Create(this, ToggleBBoxesVisibleInternal);
			ToggleSelectionBox = Command.Create(this, ToggleSelectionBoxInternal);
			ToggleObjectInfo = Command.Create(this, ToggleObjectInfoInternal);
			ToggleShowCrossHair = Command.Create(this, ToggleShowCrossHairInternal);
			ToggleShowValueAtPointer = Command.Create(this, ToggleShowValueAtPointerInternal);
			ToggleGridLines = Command.Create(this, ToggleGridLinesInternal);
			ToggleAllowSelection = Command.Create(this, ToggleAllowSelectionInternal);
			ToggleAllowElementDragging = Command.Create(this, ToggleAllowElementDraggingInternal);
			ToggleAxes = Command.Create(this, ToggleAxesInternal);
			ToggleShowTapeMeasure = Command.Create(this, ToggleShowTapeMeasureInternal);

			// Camera
			AutoRangeView = Command.Create(this, AutoRangeViewInternal);
			DoAspect11 = Command.Create(this, DoAspect11Internal);
			ToggleLockAspect = Command.Create(this, ToggleLockAspectInternal);
			ToggleOrthographic = Command.Create(this, ToggleOrthographicInternal);
			ToggleFlightCamera = Command.Create(this, ToggleFlightCameraInternal);
			ToggleMouseCentredZoom = Command.Create(this, ToggleMouseCentredZoomInternal);

			// Rendering
			SetBackgroundColour = Command.Create(this, SetBackgroundColourInternal);
			ToggleAntialiasing = Command.Create(this, ToggleAntiAliasingInternal);
			ToggleShowNormals = Command.Create(this, ToggleShowNormalsInternal);

			// Don't show empty context menus
			ContextMenuOpening += (s, a) =>
			{
				if (a.OriginalSource is not FrameworkElement dp || dp.ContextMenu == null) return;
				a.Handled = dp.ContextMenu.Items.Count == 0;
			};
		}

		/// <summary>Accessors for setting the context menus in XAML</summary>
		public ContextMenu SceneCMenu
		{
			get => m_cmenu ?? new ContextMenu();
			set
			{
				if (m_cmenu == value) return;
		
				m_cmenu = value;
				if (m_cmenu != null && m_cmenu.Name == "ChartCMenu")
					m_cmenu.DataContext = Scene;

				// Set the cmenu on both the scene and the overlay.
				// The overlay may be hidden at times but the context menu should still work
				Scene.ContextMenu = value;
				Overlay.ContextMenu = value;
			}
		}
		public ContextMenu XAxisCMenu
		{
			get => XAxisPanel.ContextMenu;
			set
			{
				XAxisPanel.ContextMenu = value;
				if (XAxisPanel.ContextMenu != null && XAxisPanel.ContextMenu.Name == "ChartAxisCMenu")
					XAxisPanel.ContextMenu.DataContext = XAxisPanel;
			}
		}
		public ContextMenu YAxisCMenu
		{
			get => YAxisPanel.ContextMenu;
			set
			{
				YAxisPanel.ContextMenu = value;
				if (YAxisPanel.ContextMenu != null && YAxisPanel.ContextMenu.Name == "ChartAxisCMenu")
					YAxisPanel.ContextMenu.DataContext = YAxisPanel;
			}
		}
		private ContextMenu? m_cmenu;

		/// <inheritdoc/>
		public bool OriginPointVisible
		{
			get => Options.OriginPointVisible;
			set
			{
				if (OriginPointVisible == value) return;
				Options.OriginPointVisible = value;
				NotifyPropertyChanged(nameof(OriginPointVisible));
			}
		}
		public ICommand ToggleOriginPoint { get; private set; } = null!;
		private void ToggleOriginPointInternal()
		{
			OriginPointVisible = !OriginPointVisible;
			Invalidate();
		}

		/// <inheritdoc/>
		public bool FocusPointVisible
		{
			get => Options.FocusPointVisible;
			set
			{
				if (FocusPointVisible == value) return;
				Options.FocusPointVisible = value;
				NotifyPropertyChanged(nameof(FocusPointVisible));
			}
		}
		public ICommand ToggleFocusPoint { get; private set; } = null!;
		private void ToggleFocusPointInternal()
		{
			FocusPointVisible = !FocusPointVisible;
			Invalidate();
		}

		/// <summary>Toggle visibility of the focus point</summary>
		public bool BBoxesVisible
		{
			get => Scene.BBoxesVisible;
			set
			{
				if (BBoxesVisible == value) return;
				Scene.BBoxesVisible = value;
				NotifyPropertyChanged(nameof(BBoxesVisible));
			}
		}
		public ICommand ToggleBBoxesVisible { get; private set; } = null!;
		private void ToggleBBoxesVisibleInternal()
		{
			BBoxesVisible = !BBoxesVisible;
			Invalidate();
		}

		/// <summary>Toggle visibility of the focus point</summary>
		public bool SelectionBoxVisible
		{
			get => Window.SelectionBoxVisible;
			set
			{
				if (SelectionBoxVisible == value) return;
				Window.SelectionBoxVisible = value;
				NotifyPropertyChanged(nameof(SelectionBoxVisible));
			}
		}
		public ICommand ToggleSelectionBox { get; private set; } = null!;
		private void ToggleSelectionBoxInternal()
		{
			SelectionBoxVisible = !SelectionBoxVisible;
			Invalidate();
		}

		/// <summary>Text description of the object under the mouse pointer</summary>
		public string HoveredObjectInfo
		{
			get => field;
			set
			{
				if (field == value) return;
				field = value;
				NotifyPropertyChanged(nameof(HoveredObjectInfo));
			}
		}  = string.Empty;

		/// <summary>Enable/Disable object info</summary>
		public bool ObjectInfoEnabled
		{
			get => m_object_info_ray_id != View3d.HitTestRayId.None;
			set
			{
				if (ObjectInfoEnabled == value)
					return;

				if (value)
				{
					Window.OnHitTestAsyncResult += HandleAsyncHitTestResult;
					MouseMove += HandleObjectInfoMouseMove;
					MouseLeave += HandleObjectInfoMouseLeave;

					// Start a timer to throttle hit test updates
					m_object_info_timer = new DispatcherTimer(DispatcherPriority.Input) { Interval = TimeSpan.FromMilliseconds(33) };
					m_object_info_timer.Tick += HandleObjectInfoTimerTick;
					m_object_info_timer.Start();

					// Add a ray for the object info
					UpdateRay();
				}
				else
				{
					// Stop the throttle timer
					if (m_object_info_timer != null)
					{
						m_object_info_timer.Tick -= HandleObjectInfoTimerTick;
						m_object_info_timer.Stop();
						m_object_info_timer = null;
					}

					MouseLeave -= HandleObjectInfoMouseLeave;
					MouseMove -= HandleObjectInfoMouseMove;
					Window.OnHitTestAsyncResult -= HandleAsyncHitTestResult;

					// Remove the ray and clear the info text
					m_object_info_ray_id = Window.HitTestAsyncRay(m_object_info_ray_id, null, 0);
					m_object_info_dirty = false;
					HoveredObjectInfo = string.Empty;
				}

				NotifyPropertyChanged(nameof(ObjectInfoEnabled));

				// Handle async hit test results
				void UpdateRay()
				{
					// Update the async hit test ray on mouse move
					var pt = Mouse.GetPosition(Scene);
					var ray = Scene.Camera.RaySS(pt.ToV2());
					var ht_ray = View3d.HitTestRay.New(ray.m_ws_origin, ray.m_ws_direction);
					m_object_info_ray_id = Window.HitTestAsyncRay(m_object_info_ray_id, ht_ray, 0);
				}
				void HandleAsyncHitTestResult(object? sender, View3d.HitTestAsyncResultEventArgs e)
				{
					// Marshal to the UI thread since the callback may fire on a background thread
					if (!Dispatcher.CheckAccess())
					{
						Dispatcher.BeginInvoke(() => HandleAsyncHitTestResult(sender, e));
						return;
					}

					if (e.Results.Length > 0 && e.Results[0].IsHit)
					{
						var obj = e.Results[0].HitObject;
						HoveredObjectInfo = obj != null ? $"{obj.Name} [{obj.Type}]" : string.Empty;
					}
					else
					{
						HoveredObjectInfo = string.Empty;
					}
				}
				void HandleObjectInfoMouseMove(object sender, MouseEventArgs e)
				{
					m_object_info_dirty = true;
				}
				void HandleObjectInfoTimerTick(object? sender, EventArgs e)
				{
					if (!m_object_info_dirty) return;
					m_object_info_dirty = false;
					UpdateRay();
				}
				void HandleObjectInfoMouseLeave(object sender, MouseEventArgs e)
				{
					// Clear hovered object info when the mouse leaves the chart
					HoveredObjectInfo = string.Empty;
				}
			}
		}
		public ICommand ToggleObjectInfo { get; private set; } = null!;
		private void ToggleObjectInfoInternal()
		{
			ObjectInfoEnabled = !ObjectInfoEnabled;
		}
		private View3d.HitTestRayId m_object_info_ray_id = View3d.HitTestRayId.None;
		private DispatcherTimer? m_object_info_timer;
		private bool m_object_info_dirty;

		/// <inheritdoc/>
		public bool ShowCrossHair
		{
			get => m_xhair != null;
			set
			{
				if (ShowCrossHair == value) return;
				if (m_xhair != null)
				{
					MouseMove -= OnMouseMoveCrossHair;
					MouseWheel -= OnMouseWheelCrossHair;
					Cursor = Cursors.Arrow;
					Util.Dispose(ref m_xhair);
				}
				m_xhair = value ? new CrossHair(this) : null;
				if (m_xhair != null)
				{
					Cursor = Cursors.Cross;
					MouseWheel += OnMouseWheelCrossHair;
					MouseMove += OnMouseMoveCrossHair;
				}
				NotifyPropertyChanged(nameof(ShowCrossHair));

				// Handlers
				void OnMouseMoveCrossHair(object? sender, MouseEventArgs e)
				{
					var client_point = e.GetPosition(this);
					if (m_xhair != null && SceneBounds.Contains(client_point))
						m_xhair.PositionCrossHair(e.GetPosition(Scene).ToV2());
				}
				void OnMouseWheelCrossHair(object? sender, MouseEventArgs e)
				{
					if (m_xhair != null)
						m_xhair.PositionCrossHair(e.GetPosition(Scene).ToV2());
				}
			}
		}
		public ICommand ToggleShowCrossHair { get; private set; } = null!;
		private void ToggleShowCrossHairInternal()
		{
			ShowCrossHair = !ShowCrossHair;
		}
		private CrossHair? m_xhair;

		/// <inheritdoc/>
		public bool ShowValueAtPointer
		{
			get => field;
			set
			{
				if (field == value) return;
				if (field)
				{
					Scene.MouseMove -= HandleMouseMove;
					Overlay.MouseMove -= HandleMouseMove;
					m_popup_show_value.IsOpen = false;
				}
				field = value;
				if (field)
				{
					m_popup_show_value.IsOpen = true;
					Overlay.MouseMove += HandleMouseMove;
					Scene.MouseMove += HandleMouseMove;
				}
				NotifyPropertyChanged(nameof(ShowValueAtPointer));

				// Handlers
				void HandleMouseMove(object? sender, MouseEventArgs e)
				{
					if (sender is not FrameworkElement fe)
						return;

					var pos = e.GetPosition(fe);
					m_popup_show_value.HorizontalOffset = pos.X + 10;
					m_popup_show_value.VerticalOffset = pos.Y + 20;
				}
			}
		}
		public ICommand ToggleShowValueAtPointer { get; private set; } = null!;
		private void ToggleShowValueAtPointerInternal()
		{
			ShowValueAtPointer = !ShowValueAtPointer;
		}

		/// <inheritdoc/>
		public bool ShowAxes
		{
			get => Options.ShowAxes;
			set
			{
				if (ShowAxes == value) return;
				Options.ShowAxes = value;
				NotifyPropertyChanged(nameof(ShowAxes));
				Invalidate();
			}
		}
		public ICommand ToggleAxes { get; private set; } = null!;
		private void ToggleAxesInternal()
		{
			ShowAxes = !ShowAxes;
		}

		/// <inheritdoc/>
		public bool ShowGridLines
		{
			get => Options.ShowGridLines;
			set
			{
				if (ShowGridLines == value) return;
				Options.ShowGridLines = value;
				NotifyPropertyChanged(nameof(ShowGridLines));
				Invalidate();
			}
		}
		public ICommand ToggleGridLines { get; private set; } = null!;
		private void ToggleGridLinesInternal()
		{
			ShowGridLines = !ShowGridLines;
		}

		/// <inheritdoc/>
		public bool AllowSelection
		{
			get => Options.AllowSelection;
			set
			{
				if (AllowSelection == value) return;
				Options.AllowSelection = value;
				NotifyPropertyChanged(nameof(AllowSelection));
				Invalidate();
			}
		}
		public ICommand ToggleAllowSelection { get; private set; } = null!;
		private void ToggleAllowSelectionInternal()
		{
			AllowSelection = !AllowSelection;
		}

		/// <inheritdoc/>
		public bool AllowElementDragging
		{
			get => Options.AllowElementDragging;
			set
			{
				if (AllowElementDragging == value) return;
				Options.AllowElementDragging = value;
				NotifyPropertyChanged(nameof(AllowElementDragging));
				Invalidate();
			}
		}
		public ICommand ToggleAllowElementDragging { get; private set; } = null!;
		private void ToggleAllowElementDraggingInternal()
		{
			AllowElementDragging = !AllowElementDragging;
		}

		/// <summary>Tape measure tool</summary>
		public bool ShowTapeMeasure
		{
			get => m_tape != null;
			set
			{
				if (ShowTapeMeasure == value) return;
				if (ShowTapeMeasure)
				{
					Cursor = Cursors.Arrow;
					Util.Dispose(ref m_tape);
				}
				m_tape = value ? new TapeMeasure(this) : null;
				if (ShowTapeMeasure)
				{
					Cursor = Cursors.Cross;
				}
				NotifyPropertyChanged(nameof(ShowTapeMeasure));
			}
		}
		public ICommand ToggleShowTapeMeasure { get; private set; } = null!;
		private void ToggleShowTapeMeasureInternal()
		{
			ShowTapeMeasure = !ShowTapeMeasure;
		}
		private TapeMeasure? m_tape;

		/// <inheritdoc/>
		public View3d.ESceneBounds AutoRangeBounds
		{
			get;
			set
			{
				if (field == value) return;
				field = value;
				NotifyPropertyChanged(nameof(AutoRangeBounds));
			}
		}
		public ICommand AutoRangeView { get; private set; } = null!;
		private void AutoRangeViewInternal(object? parameter)
		{
			var bounds = parameter is View3d.ESceneBounds b ? b : View3d.ESceneBounds.All;
			AutoRange(who: bounds);
		}

		/// <inheritdoc/>
		public ICommand DoAspect11 { get; private set; } = null!;
		private void DoAspect11Internal()
		{
			Range.Aspect = Scene.Window.Viewport.Width / Scene.Window.Viewport.Height;
			SetCameraFromRange();
			Invalidate();
		}

		/// <inheritdoc/>
		public bool LockAspect
		{
			get => Options.LockAspect != null;
			set
			{
				if (LockAspect == value) return;
				Options.LockAspect = value ? (XAxis.Span * SceneBounds.Height) / (YAxis.Span * SceneBounds.Width) : (double?)null;
				NotifyPropertyChanged(nameof(LockAspect));
			}
		}
		public ICommand ToggleLockAspect { get; private set; } = null!;
		private void ToggleLockAspectInternal()
		{
			LockAspect = !LockAspect;
		}

		/// <inheritdoc/>
		public bool Orthographic
		{
			get => Options.Orthographic;
			set
			{
				if (Orthographic == value) return;
				Options.Orthographic = value;
				NotifyPropertyChanged(nameof(Orthographic));
			}
		}
		public ICommand ToggleOrthographic { get; private set; } = null!;
		private void ToggleOrthographicInternal()
		{
			Orthographic = !Orthographic;
		}

		/// <inheritdoc/>
		public bool FlightCamera
		{
			get;
			set
			{
				if (FlightCamera == value) return;

				if (FlightCamera)
				{
					if (MouseOperations.Active is MouseOp_FlightCamera op)
						MouseOperations.EndModalOp(op);
					else if (Scene.Window.FlightCameraEnabled)
						Scene.Window.FlightCameraEnabled = false;
				}

				field = value;

				if (FlightCamera)
					MouseOperations.BeginModalOp(new MouseOp_FlightCamera(this));

				NotifyPropertyChanged(nameof(FlightCamera));
				Invalidate();
			}
		}
		public ICommand ToggleFlightCamera { get; private set; } = null!;
		private void ToggleFlightCameraInternal()
		{
			FlightCamera = !FlightCamera;
		}

		/// <inheritdoc/>
		public bool MouseCentredZoom
		{
			get => Options.MouseCentredZoom;
			set
			{
				if (MouseCentredZoom == value) return;
				Options.MouseCentredZoom = value;
				NotifyPropertyChanged(nameof(MouseCentredZoom));
			}
		}
		public ICommand ToggleMouseCentredZoom { get; private set; } = null!;
		private void ToggleMouseCentredZoomInternal()
		{
			MouseCentredZoom = !MouseCentredZoom;
		}

		/// <inheritdoc/>
		public ENavMode NavigationMode
		{
			get => Options.NavigationMode;
			set
			{
				if (NavigationMode == value) return;
				Options.NavigationMode = value;
				NotifyPropertyChanged(nameof(NavigationMode));
			}
		}

		/// <inheritdoc/>
		public Colour32 BackgroundColour
		{
			get => Options.BackgroundColour;
			set
			{
				if (BackgroundColour == value) return;
				Options.BackgroundColour = value;
				NotifyPropertyChanged(nameof(BackgroundColour));
			}
		}
		public ICommand SetBackgroundColour { get; private set; } = null!;
		private void SetBackgroundColourInternal()
		{
			Scene.SetBackgroundColour.Execute(null);
			BackgroundColour = Window.BackgroundColour;
		}

		/// <inheritdoc/>
		public bool Antialiasing
		{
			get => Options.Antialiasing;
			set
			{
				if (Antialiasing == value) return;
				Options.Antialiasing = value;
				NotifyPropertyChanged(nameof(Antialiasing));
			}
		}
		public ICommand ToggleAntialiasing { get; private set; } = null!;
		private void ToggleAntiAliasingInternal()
		{
			Antialiasing = !Antialiasing;
		}

		/// <inheritdoc/>
		public double ShadowCastRange
		{
			get => Options.ShadowCastRange;
			set
			{
				if (ShadowCastRange == value) return;
				Options.ShadowCastRange = value;
				NotifyPropertyChanged(nameof(ShadowCastRange));
			}
		}

		/// <inheritdoc/>
		public EAlignDirection AlignDirection
		{
			get => Scene.AlignDirection;
			set
			{
				if (AlignDirection == value) return;
				Scene.AlignDirection = value;
				NotifyPropertyChanged(nameof(AlignDirection));
			}
		}

		/// <inheritdoc/>
		public EViewPreset ViewPreset
		{
			get => Scene.ViewPreset;
			set
			{
				if (ViewPreset == value) return;
				Scene.ViewPreset = value;
				NotifyPropertyChanged(nameof(ViewPreset));
			}
		}

		/// <summary>Saved views</summary>
		public ICollectionView SavedViews => Scene.SavedViews;
		public ICommand ApplySavedView => Scene.ApplySavedView;
		public ICommand SaveCurrentView => Scene.SaveCurrentView;
		public ICommand RemoveSavedView => Scene.RemoveSavedView;

		/// <inheritdoc/>
		public ICommand ShowCameraUI => Scene.ShowCameraUI;

		/// <inheritdoc/>
		public View3d.EFillMode FillMode
		{
			get => Options.FillMode;
			set
			{
				if (FillMode == value) return;
				Options.FillMode = value;
				NotifyPropertyChanged(nameof(FillMode));
			}
		}

		/// <inheritdoc/>
		public View3d.ECullMode CullMode
		{
			get => Options.CullMode;
			set
			{
				if (CullMode == value) return;
				Options.CullMode = value;
				NotifyPropertyChanged(nameof(CullMode));
			}
		}

		/// <inheritdoc/>
		public bool ShowNormals
		{
			get => Scene.ShowNormals;
			set
			{
				if (ShowNormals == value) return;
				Scene.ShowNormals = value;
				NotifyPropertyChanged(nameof(ShowNormals));
			}
		}
		public ICommand ToggleShowNormals { get; private set; } = null!;
		private void ToggleShowNormalsInternal()
		{
			ShowNormals = !ShowNormals;
			Invalidate();
		}

		/// <inheritdoc/>
		public float NormalsLength
		{
			get => Scene.NormalsLength;
			set
			{
				if (NormalsLength == value) return;
				Scene.NormalsLength = value;
				NotifyPropertyChanged(nameof(NormalsLength));
			}
		}

		/// <inheritdoc/>
		public Colour32 NormalsColour
		{
			get => Scene.NormalsColour;
			set
			{
				if (NormalsColour == value) return;
				Scene.NormalsColour = value;
				NotifyPropertyChanged(nameof(NormalsColour));
			}
		}
		public ICommand SetNormalsColour => Scene.SetNormalsColour;

		/// <inheritdoc/>
		public float FillModePointsSize
		{
			get => Scene.FillModePointsSize;
			set
			{
				if (FillModePointsSize == value) return;
				Scene.FillModePointsSize = value;
				NotifyPropertyChanged(nameof(FillModePointsSize));
			}
		}

		/// <inheritdoc/>
		public ICommand ShowAnimationUI => Scene.ShowAnimationUI;

		/// <inheritdoc/>
		public ICommand ShowLightingUI => Scene.ShowLightingUI;

		/// <inheritdoc/>
		public ICommand ShowRayTracingUI => Scene.ShowRayTracingUI;

		/// <inheritdoc/>
		public ICommand ShowMeasureToolUI => Scene.ShowMeasureToolUI;

		/// <inheritdoc/>
		public ICommand ShowObjectManagerUI => Scene.ShowObjectManagerUI;

		/// <inheritdoc/>
		public bool CanLinkCamera { get; set; } = false;
		public ICommand LinkCamera { get; set; } = Command.NoOp;

		/// <inheritdoc/>
		public IView3dCMenu View3dCMenuContext => Scene.View3dCMenuContext;

		/// <inheritdoc/>
		public IChartCMenu ChartCMenuContext => Scene.ChartCMenuContext;
	}
}
