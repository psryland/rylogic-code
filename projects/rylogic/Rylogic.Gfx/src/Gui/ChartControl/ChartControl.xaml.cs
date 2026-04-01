using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Diagnostics;
using System.Linq;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Input;
using System.Windows.Media;
using Rylogic.Common;
using Rylogic.Extn;
using Rylogic.Gfx;
using Rylogic.LDraw;
using Rylogic.Maths;
using Rylogic.Utility;
using Rylogic.Windows.Extn;

namespace Rylogic.Gui.WPF
{
	using ChartDetail;

	/// <summary>A view 3d based chart control</summary>
	public partial class ChartControl :UserControl, IDisposable, INotifyPropertyChanged
	{
		// Notes:
		// - "Chart Space" is camera aligned and at the focus plane, with the world origin
		//   projected onto the focus plane using the camera Z axis. So, (0,0,0) chart space
		//   is the point on the camera focus plane where a line from the world origin, along
		//   c2w.z intersects the camera focus plane. The Camera is at (w2c.pos.x, w2c.pos.y, FocusDist)
		//   in chart space.
		// - Naming convention:
		//     client_point = a Point relative to this control,
		//     scene_point = a v2 relative to 'Scene' (ChartPanel)
		//     world_point = a v4 in world space
		//     chart_point = a v4 in camera space, relative to the chart origin
		//     chart_origin = the world space origin projected onto the camera focus plane using c2w.z
		// - Two methods of camera control; 1. directly position the camera, or 2. set the
		//   axis range, then determine the camera position from that. Both systems are used.
		//   for mouse control, the camera is positioned and the axis range updated based on
		//   the camera position. For ranging, or fixed chart positioning, the camera position
		//   is moved to match the axis range.
		// - After setting the axis range, call SetCameraFromRange()
		// - After setting the camera position, call SetRangeFromCamera()
		// - Mouse wheel moves the camera in the camera z direction
		// - Mouse wheel on an axis (changing the aspect ratio) changes the FoV
		// - Client space is the control's coordinates. The View3D part typically has a non-zero
		//   offset from the control origin. e.g. SceneBounds = [30,10,100,100] say.
		//   Note: the overlay has a transform applied to it so that points in client space are
		//   correctly positioned.

		static ChartControl()
		{
			View3d.LoadDll(throw_if_missing: false);
		}
		public ChartControl()
			: this(string.Empty, new OptionsData())
		{ }
		public ChartControl(string title, OptionsData options)
		{
			InitializeComponent();

			Title = title;
			Options = options;
			Scene.Chart = this;
			Range = new RangeData(this);
			BaseRangeX = new RangeF(0.0, 1.0);
			BaseRangeY = new RangeF(0.0, 1.0);
			MouseOperations = new MouseOps();
			Elements = new ElementCollection(this);
			Selected = new SelectedCollection(this);
			Hovered = new HoveredCollection(this);

			DefaultMouseControl = true;
			DefaultKeyboardShortcuts = true;

			Scene.BuildScene += OnBuildScene;

			// Prevent the object manager seeing anything created in the 'CtxId' context
			View3d.ObjectManager.ExcludeCtxIds.Add(CtxId);
			if (DesignerProperties.GetIsInDesignMode(this))
				return;

			try
			{
				View3d = View3d.Create();

				InitCommands();
				InitNavigation();

				// Binding to 'this' is needed because the axis and scene parts of the control
				// need to be bound to the instances in this class.
				m_root.DataContext = this;
			}
			catch
			{
				Dispose();
				throw;
			}
		}
		public void Dispose()
		{
			Dispose(true);
			GC.SuppressFinalize(this);
		}
		protected virtual void Dispose(bool _)
		{
			// Note: if 'Gui_.DisposeChildren()' is used, the View3dControl will
			// be disposed before this control, because it is a child.
			Disposing?.Invoke(this, EventArgs.Empty);

			// Dispose the 'Elements'? Can argue either way. I'm saying no, because
			// where ever the caller is calling 'Dispose' on the chart, they can call
			// 'Util.DisposeRange(chart.Elements)' first if that's what they want.
			// That way the lifetime of the elements is independent of the chart.
			Elements.Clear();

			BindingOperations.ClearAllBindings(this);
			ShowHitTestRay = false;
			MouseOperations = null!;
			Range = null!;
			Options = null!;
			Scene.Dispose();
			View3d = null!;
		}

		/// <summary>Raised when the ChartControl is being disposed</summary>
		public event EventHandler? Disposing;

		/// <summary>Rendering options for the chart</summary>
		public OptionsData Options
		{
			get => (OptionsData)GetValue(OptionsProperty);
			set => SetValue(OptionsProperty, value);
		}
		private void Options_Changed(OptionsData new_value, OptionsData old_value)
		{
			if (old_value != null)
			{
				old_value.XAxis.PropertyChanged -= HandleAxisOptionsChanged;
				old_value.YAxis.PropertyChanged -= HandleAxisOptionsChanged;
				old_value.PropertyChanged -= HandleOptionsChanged;
			}
			if (new_value != null)
			{
				// Apply the options to the scene
				Camera.Orthographic = Options.Orthographic;
				Window.FillMode = Options.FillMode;
				Window.CullMode = Options.CullMode;
				Window.BackgroundColour = Options.BackgroundColour;
				Window.FocusPointVisible = Options.FocusPointVisible;
				Window.OriginPointVisible = Options.OriginPointVisible;
				Scene.MultiSampling = Options.Antialiasing ? 4 : 1;
				Scene.ShadowCastRange = Options.ShadowCastRange;
				PositionAxisPanels();

				new_value.PropertyChanged += HandleOptionsChanged;
				new_value.XAxis.PropertyChanged += HandleAxisOptionsChanged;
				new_value.YAxis.PropertyChanged += HandleAxisOptionsChanged;
			}

			void HandleOptionsChanged(object? sender, PropertyChangedEventArgs e)
			{
				var view3d_cmenu = Scene.ContextMenu?.DataContext as IView3dCMenu;
				var chart_cmenu = Scene.ContextMenu?.DataContext as IChartCMenu;
				switch (e.PropertyName)
				{
					case nameof(OptionsData.ShowGridLines):
					{
						chart_cmenu?.NotifyPropertyChanged(nameof(IChartCMenu.ShowGridLines));
						break;
					}
					case nameof(OptionsData.ShowAxes):
					{
						m_xaxis_panel.Invalidate();
						m_yaxis_panel.Invalidate();
						NotifyPropertyChanged(nameof(XAxisLabelVisibility));
						NotifyPropertyChanged(nameof(YAxisLabelVisibility));
						chart_cmenu?.NotifyPropertyChanged(nameof(IChartCMenu.ShowAxes));
						break;
					}
					case nameof(OptionsData.Antialiasing):
					{
						Scene.Antialiasing = Options.Antialiasing;
						view3d_cmenu?.NotifyPropertyChanged(nameof(IView3dCMenu.Antialiasing));
						Invalidate();
						break;
					}
					case nameof(OptionsData.ShadowCastRange):
					{
						Scene.ShadowCastRange = Options.ShadowCastRange;
						view3d_cmenu?.NotifyPropertyChanged(nameof(IView3dCMenu.ShadowCastRange));
						Invalidate();
						break;
					}
					case nameof(OptionsData.Orthographic):
					{
						Camera.Orthographic = Options.Orthographic;
						view3d_cmenu?.NotifyPropertyChanged(nameof(IView3dCMenu.Orthographic));
						Invalidate();
						break;
					}
					case nameof(OptionsData.FillMode):
					{
						Window.FillMode = Options.FillMode;
						view3d_cmenu?.NotifyPropertyChanged(nameof(IView3dCMenu.FillMode));
						Invalidate();
						break;
					}
					case nameof(OptionsData.CullMode):
					{
						Window.CullMode = Options.CullMode;
						view3d_cmenu?.NotifyPropertyChanged(nameof(IView3dCMenu.CullMode));
						Invalidate();
						break;
					}
					case nameof(OptionsData.BackgroundColour):
					{
						Window.BackgroundColour = Options.BackgroundColour;
						//NotifyPropertyChanged(nameof(ChartBackground));
						view3d_cmenu?.NotifyPropertyChanged(nameof(IView3dCMenu.BackgroundColour));

						// Invalidate grid lines and cross-hair because their colour depends on the background colour
						XAxis.Gfx.Invalidate();
						YAxis.Gfx.Invalidate();
						if (ShowCrossHair)
						{
							ShowCrossHair = false;
							ShowCrossHair = true;
						}
						Invalidate();
						break;
					}
					case nameof(OptionsData.FocusPointVisible):
					{
						Window.FocusPointVisible = Options.FocusPointVisible;
						view3d_cmenu?.NotifyPropertyChanged(nameof(IView3dCMenu.FocusPointVisible));
						Invalidate();
						break;
					}
					case nameof(OptionsData.OriginPointVisible):
					{
						Window.OriginPointVisible = Options.OriginPointVisible;
						view3d_cmenu?.NotifyPropertyChanged(nameof(IView3dCMenu.OriginPointVisible));
						Invalidate();
						break;
					}
					case nameof(OptionsData.NavigationMode):
					{
						chart_cmenu?.NotifyPropertyChanged(nameof(IChartCMenu.NavigationMode));
						break;
					}
					case nameof(OptionsData.LockAspect):
					{
						chart_cmenu?.NotifyPropertyChanged(nameof(IChartCMenu.LockAspect));
						break;
					}
					case nameof(OptionsData.MouseCentredZoom):
					{
						chart_cmenu?.NotifyPropertyChanged(nameof(IChartCMenu.MouseCentredZoom));
						break;
					}
				}
			}
			void HandleAxisOptionsChanged(object? sender, PropertyChangedEventArgs e)
			{
				switch (e.PropertyName)
				{
					case nameof(OptionsData.Axis.Side):
					{
						PositionAxisPanels();
						break;
					}
					case nameof(OptionsData.Axis.ShowGridLines):
					{
						Scene.Invalidate();
						break;
					}
				}
			}
		}
		public static readonly DependencyProperty OptionsProperty = Gui_.DPRegister<ChartControl>(nameof(Options), new OptionsData(), Gui_.EDPFlags.None);

		/// <summary>Scene background colour</summary>
		public SolidColorBrush SceneBackground
		{
			get => Options.BackgroundColour.ToMediaBrush();
			set => Options.BackgroundColour = value.Color.ToColour32();
		}

		/// <summary>The title of the chart</summary>
		public string Title
		{
			get => (string)GetValue(TitleProperty);
			set => SetValue(TitleProperty, value);
		}
		public static readonly DependencyProperty TitleProperty = Gui_.DPRegister<ChartControl>(nameof(Title), string.Empty, Gui_.EDPFlags.None);

		/// <summary>X-Axis label</summary>
		public string XLabel
		{
			get => (string)GetValue(XLabelProperty);
			set => SetValue(XLabelProperty, value);
		}
		private void XLabel_Changed()
		{
			XAxis.Label = XLabel;
		}
		public static readonly DependencyProperty XLabelProperty = Gui_.DPRegister<ChartControl>(nameof(XLabel), string.Empty, Gui_.EDPFlags.None);
		
		/// <summary>Y-Axis label</summary>
		public string YLabel
		{
			get => (string)GetValue(YLabelProperty);
			set => SetValue(YLabelProperty, value);
		}
		private void YLabel_Changed()
		{
			YAxis.Label = YLabel;
		}
		public static readonly DependencyProperty YLabelProperty = Gui_.DPRegister<ChartControl>(nameof(YLabel), string.Empty, Gui_.EDPFlags.None);

		/// <summary>All chart objects</summary>
		public ElementCollection Elements { get; }

		/// <summary>The set of selected chart elements</summary>
		public SelectedCollection Selected { get; }

		/// <summary>The set of hovered chart elements</summary>
		public HoveredCollection Hovered { get; }

		/// <summary>The current X/Y axis range of the chart</summary>
		public RangeData Range
		{
			get;
			private set
			{
				if (value == field) return;
				if (field != null)
				{
					field.XAxis.PropertyChanged -= HandleAxisPropertyChanged;
					field.YAxis.PropertyChanged -= HandleAxisPropertyChanged;
					Util.Dispose(ref field!);
				}
				field = value;
				if (field != null)
				{
					field.XAxis.PropertyChanged += HandleAxisPropertyChanged;
					field.YAxis.PropertyChanged += HandleAxisPropertyChanged;

					NotifyPropertyChanged(nameof(Range));
					NotifyPropertyChanged(nameof(XAxis));
					NotifyPropertyChanged(nameof(YAxis));
				}

				// Handlers
				void HandleAxisPropertyChanged(object? sender, PropertyChangedEventArgs e)
				{
					switch (e.PropertyName)
					{
						case nameof(RangeData.Axis.Label):
						{
							if (ReferenceEquals(sender, XAxis)) NotifyPropertyChanged(nameof(XLabel));
							if (ReferenceEquals(sender, YAxis)) NotifyPropertyChanged(nameof(YLabel));
							break;
						}
					}
				}
			}
		} = null!;

		/// <summary>Access to the current X axis</summary>
		public RangeData.Axis XAxis => Range.XAxis;
		public AxisPanel XAxisPanel => m_xaxis_panel;

		/// <summary>Access to the current Y axis</summary>
		public RangeData.Axis YAxis => Range.YAxis;
		public AxisPanel YAxisPanel => m_yaxis_panel;

		/// <summary>Access to the current Y axis</summary>
		public RangeData.Axis GetAxis(EAxis axis)
		{
			return
				axis == EAxis.XAxis ? XAxis :
				axis == EAxis.YAxis ? YAxis :
				throw new Exception($"Cannot return the axis associated with {axis}");
		}
		public AxisPanel GetAxisPanel(EAxis axis)
		{
			return
				axis == EAxis.XAxis ? XAxisPanel :
				axis == EAxis.YAxis ? YAxisPanel :
				throw new Exception($"Cannot return the axis panel associated with {axis}");
		}

		/// <summary>Default X axis range of the chart</summary>
		public RangeF BaseRangeX { get; set; }

		/// <summary>Default Y axis range of the chart</summary>
		public RangeF BaseRangeY { get; set; }

		/// <summary>The view3d part of the chart</summary>
		public ChartPanel Scene => m_chart_panel;

		/// <summary>A WPF canvas that overlays the 3d part of the chart</summary>
		public Canvas Overlay => m_chart_overlay;

		/// <summary>The camera view of the scene</summary>
		public View3d.Camera Camera => Scene.Camera;

		/// <summary>The view3d window associated with 'Scene'</summary>
		private View3d.Window Window => Scene.Window; // Keep private, clients should be using 'Scene'

		/// <summary>A control to use as the legend.</summary>
		public FrameworkElement? Legend
		{
			get;
			set
			{
				if (field == value) return;
				if (field != null)
				{
					Overlay.Children.Remove(field);
				}
				field = value;
				if (field != null)
				{
					Overlay.Children.Add(field);

					// If the legend is 'ChartLegend' (i.e. not some custom thing),
					// then default the ItemsSource to the chart elements.
					if (field is ChartLegend legend)
					{
						legend.ItemsSource ??= Elements;
						legend.PositionBasedOnAlignment();
					}
				}
			}
		} = null;

		/// <summary>A control to use as overlay graphics in client space</summary>
		public FrameworkElement? OverlayContent
		{
			// Notes:
			//  - Use:
			//      <view3d:ChartControl.OverlayContent>
			//          <StackPanel
			//              DataContext="{Binding DataContext, RelativeSource={RelativeSource Mode=FindAncestor, AncestorType={x:Type Window}}}"  -- Bind to the container of the chart control
			//              Canvas.Left="10"  -- Position on the chart
			//              Canvas.Top="10"   
			//              >
			//              <TextBlock Text="{Binding ., Converter={conv:ToString}}" /> -- Show the binding context
			//              <Line X1="0" Y1="{Binding LineY}" X2="100" Y2="0" Stroke="Red" /> -- Graphics in scene space
			//          </StackPanel>
			//      </view3d:ChartControl.OverlayContent>

			get;
			set
			{
				if (field == value) return;
				if (field != null)
				{
					Overlay.Children.Remove(field);
				}
				field = value;
				if (field != null)
				{
					Overlay.Children.Add(field);
				}
			}
		} = null;

		/// <summary>A control to use as overlay graphics in chart space</summary>
		public System.Windows.Shapes.Path? OverlayContentChartSpace
		{
			// Notes:
			//  - Use chart-space graphics:
			//    Using Geometry instead of 'Shapes' means the scaling doesn't affect the stroke thickness of lines etc
			//
			//     <view3d:ChartControl.OverlayContentChartSpace>
			//         <Path Stroke="Green" StrokeThickness="1" Data="M0,1 L5,3 L10,0">
			//     </view3d:ChartControl.OverlayContentChartSpace>
			//
			//  - Alternatively, use Geometry with bindings
			//     <view3d:ChartControl.OverlayContentChartSpace>
			//         <Path Stroke="Green" StrokeThickness="1" DataContext="{Binding DataContext, RelativeSource={RelativeSource Mode=FindAncestor, AncestorType={x:Type Window}}}">
			//             <Path.Data>
			//                 <GeometryGroup>
			//                     <LineGeometry StartPoint="{Binding StartPt}" EndPoint="{Binding EndPt" />
			//                 </GeometryGroup>
			//             </Path.Data>
			//         </Path>
			//     </view3d:ChartControl.OverlayContentChartSpace>

			get => m_overlay_content_chart_space;
			set
			{
				if (m_overlay_content_chart_space == value) return;
				if (m_overlay_content_chart_space != null)
				{
					Overlay.Children.Remove(m_overlay_content_chart_space);
				}
				m_overlay_content_chart_space = value;
				if (m_overlay_content_chart_space != null)
				{
					Overlay.Children.Add(m_overlay_content_chart_space);
					ApplyScaleToChartSpaceOverlay();
				}
			}
		}
		private System.Windows.Shapes.Path? m_overlay_content_chart_space = null;

		/// <summary>Apply the Chart-to-Overlay transform to all geometry</summary>
		private void ApplyScaleToChartSpaceOverlay()
		{
			if (m_overlay_content_chart_space?.Data is System.Windows.Media.Geometry geom)
			{
				if (geom.IsFrozen)
					geom = geom.Clone();
				
				geom.Transform = ChartToSceneSpace().ToTransform();
				m_overlay_content_chart_space.Data = geom.Freeze2();
			}
		}

		/// <summary>View3d context reference</summary>
		private View3d View3d
		{
			get;
			set
			{
				if (field == value) return;
				Util.Dispose(ref field!);
				field = value;
			}
		} = null!;

		/// <summary>Raised just before the chart renders, allowing users to add custom graphics</summary>
		public event EventHandler<View3dControl.BuildSceneEventArgs>? BuildScene;
		private void OnBuildScene(object? sender, View3dControl.BuildSceneEventArgs e)
		{
			// Notes:
			//  - 'Elements' manage their graphics directly in the View3d.Window. Those
			//    with overlay controls also have direct access to 'Chart.Overlay'
			//  - BuildScene is really just for graphics elements that aren't derived
			//    from 'Chart.Element'.

			// The offset in client space to the overlay origin
			var pt = TransformToDescendant(Scene).Transform(new Point());
			Canvas.SetLeft(Overlay, pt.X);
			Canvas.SetTop(Overlay, pt.Y);

			// Allow external additions to the scene
			BuildScene?.Invoke(this, e);
		}

		/// <summary>Invalidate the chart, triggering a redraw</summary>
		public void Invalidate()
		{
			Scene.Invalidate();
		}
		public void Invalidate(object sender, EventArgs args)
		{
			Invalidate();
		}

		/// <summary>Raised whenever the view3d area of the chart is scrolled or zoomed</summary>
		public event EventHandler<ChartMovedEventArgs>? ChartMoved;
		protected virtual void OnChartMoved(ChartMovedEventArgs args)
		{
			ApplyScaleToChartSpaceOverlay();
			ChartMoved?.Invoke(this, args);
		}

		/// <summary>Raised whenever elements in the chart have been edited or moved</summary>
		public event EventHandler<ChartChangedEventArgs>? ChartChanged;
		protected virtual void OnChartChanged(ChartChangedEventArgs args)
		{
			if (m_chart_changed_suspended != 0) return;
			ChartChanged?.Invoke(this, args);
		}
		public Scope SuspendChartChanged(bool raise_on_resume = true)
		{
			return Scope.Create(
				() => { return ++m_chart_changed_suspended; },
				cc => { if (--m_chart_changed_suspended == 0) OnChartChanged(new ChartChangedEventArgs(EChangeType.Edited)); });
		}
		private int m_chart_changed_suspended;

		/// <summary>Called during AutoRange to allow handlers to set the auto range</summary>
		public event EventHandler<AutoRangeEventArgs>? AutoRanging;
		protected virtual void OnAutoRanging(AutoRangeEventArgs args)
		{
			AutoRanging?.Invoke(this, args);
		}

		/// <summary>Returns a point in chart space from a point in scene space. Use to convert mouse (scene-space) locations to chart coordinates</summary>
		public v4 SceneToChart(v2 scene_point)
		{
			// Convert the scene space point to a world space point
			var world_point = Scene.Camera.SSPointToWSPoint(scene_point);

			// Convert to chart space (i.e. camera space, on the focus plane)
			// Shift relative to the chart origin (i.e. (0,0,0) world space, projected onto the focus plane)
			var w2c = Scene.Camera.W2O;
			var chart_point = w2c * world_point;
			chart_point.xy -= w2c.pos.xy;
			chart_point.z += Scene.Camera.FocusDist;
			return chart_point;
		}

		/// <summary>Returns a point in scene space from a point in chart space. Inverse of SceneToChart</summary>
		public v2 ChartToScene(v4 chart_point)
		{
			// Remember, chart space is camera aligned.
			
			// Shift relative to the camera
			chart_point.xy += Scene.Camera.W2O.pos.xy;
			chart_point.z -= Scene.Camera.FocusDist;

			// Convert from camera space to world space
			var world_point = Scene.Camera.O2W * chart_point;

			// Convert from world space to client scene space
			return WorldToScene(world_point);
		}

		/// <summary>Returns a point in scene space from a point in world space.</summary>
		public v2 WorldToScene(v4 world_point)
		{
			var scene_point = Scene.Camera.WSPointToSSPoint(world_point);
			return scene_point;
		}

		/// <summary>Convert a scene space rectangle to a chart space bounding box (with infinite Z)</summary>
		public BBox SceneToChart(Rect scene_rect)
		{
			var scene_pt0 = scene_rect.BottomLeft.ToV2();
			var scene_pt1 = scene_rect.TopRight.ToV2();

			var chart_pt0 = SceneToChart(scene_pt0); chart_pt0.z = -float.MaxValue/2f;
			var chart_pt1 = SceneToChart(scene_pt1); chart_pt1.z = +float.MaxValue/2f;

			return new BBox(
				(chart_pt1 + chart_pt0) / 2f,
				(chart_pt1 - chart_pt0) / 2f);
		}

		/// <summary>Returns a size in chart space from a size (and centre point) in scene space ('centre' defaults to the centre of the view)</summary>
		public Size SceneToChart(Size scene_size, Point? scene_point = null)
		{
			// Use the centre of the view. For Orthographic it makes no difference, but for perspective
			// sizes in the centre are smaller than sizes near the edges of the view.
			var scene_pt = scene_point ?? Scene.RenderArea().Centre();
			scene_pt.X -= scene_size.Width / 2;
			scene_pt.Y -= scene_size.Height / 2;

			// Convert the size to a chart space volume
			var bbox = SceneToChart(new Rect(scene_pt, scene_size));
			return new Size(bbox.SizeX, bbox.SizeY);
		}

		/// <summary>Convert a chart space bounding box to a client scene space rectangle. BBox's are chart/camera space aligned, so in scene space the bbox becomes a rectangle</summary>
		public Rect ChartToScene(BBox chart_bbox)
		{
			var chart_pt0 = chart_bbox.Min; chart_pt0.z = 0;
			var chart_pt1 = chart_bbox.Max; chart_pt1.z = 0;

			var scene_pt0 = ChartToScene(chart_pt0);
			var scene_pt1 = ChartToScene(chart_pt1);

			var pt = Math_.Min(scene_pt0, scene_pt1).ToPointD();
			var sz = Math_.Abs(scene_pt1 - scene_pt0).ToSizeD();
			return new Rect(pt, sz);
		}

		/// <summary>Convert a world space bounding box to a client scene space rectangle. Chart space and world space are not necessarily aligned, so the returned Rect may be inflated</summary>
		public Rect WorldToScene(BBox world_bbox)
		{
			var world_pt0 = world_bbox.Min;
			var world_pt1 = world_bbox.Max;

			var scene_pt0 = WorldToScene(world_pt0);
			var scene_pt1 = WorldToScene(world_pt1);

			return new Rect(scene_pt0.ToPointD(), scene_pt1.ToPointD());
		}

		/// <summary>Return a transform from scene space to chart space.</summary>
		public m4x4 SceneToChartSpace()
		{
			// Notes:
			//  - Chart space is aligned with the camera, so the transform from chart to
			//    scene is basically just a 2D scale and offset transform.
			// e.g.
			//   client2chart * plot_area.BottomLeft() = Point(x_min, y_min)
			//   client2chart * plot_area.TopRight()   = Point(x_max, y_max)

			var size = SceneBounds.Size;
			var scale_x = (float)+(XAxis.Span / size.Width);
			var scale_y = (float)-(YAxis.Span / size.Height);
			var offset_x = (float)+(XAxis.Min - 0 * scale_x);
			var offset_y = (float)+(YAxis.Min - size.Height * scale_y);

			// c = chart, s = scene
			var s2c = new m4x4(
				new v4(scale_x, 0, 0, 0),
				new v4(0, scale_y, 0, 0),
				new v4(0, 0, 1, 0),
				new v4(offset_x, offset_y, 0, 1));

			return s2c;
		}

		/// <summary>Return a transform from chart space to scene space.</summary>
		public m4x4 ChartToSceneSpace()
		{
			// Notes:
			//  - Chart space is aligned with the camera, so the transform from chart to
			//    scene is basically just a 2D scale and offset transform.
			// e.g.
			//   chart2scene * [x_min, y_min, 0, 1] = chart_area.BottomLeft()
			//   chart2scene * [x_max, y_max, 0, 1] = chart_area.TopRight()

			var size = SceneBounds.Size;
			var scale_x = (float)+(size.Width / XAxis.Span);
			var scale_y = (float)-(size.Height / YAxis.Span);
			var offset_x = (float)+(0 - XAxis.Min * scale_x);
			var offset_y = (float)+(size.Height - YAxis.Min * scale_y);

			// c = chart, s = scene
			var c2s = new m4x4(
				new v4(scale_x, 0, 0, 0),
				new v4(0, scale_y, 0, 0),
				new v4(0, 0, 1, 0),
				new v4(offset_x, offset_y, 0, 1));

			return c2s;
		}

		// Todo: these need to work in 3D
#if false

		/// <summary>Returns a point in chart space from a point in client space. Use to convert mouse (client-space) locations to chart coordinates</summary>
		//[Obsolete]
		public Point ClientToChart(Point client_point, bool scene_relative = false)
		{
			var bounds = SceneBounds;
			if (bounds.Width == 0 || bounds.Height == 0)
				throw new Exception("Chart size is zero, cannot convert from client space to chart space");

			// If 'client_point' is relative to the 3d scene then convert it to true client space.
			if (scene_relative)
			{
				client_point.X += bounds.X;
				client_point.Y += bounds.Y;
			}

			var pt = new Point(
				(XAxis.Min + (client_point.X - bounds.Left) * XAxis.Span / bounds.Width),
				(YAxis.Min - (client_point.Y - bounds.Bottom) * YAxis.Span / bounds.Height));

			return pt;
		}
		public Size ClientToChart(Size client_size)
		{
			var bounds = SceneBounds;
			if (bounds.Width == 0 || bounds.Height == 0)
				throw new Exception("Chart size is zero, cannot convert from client space to chart space");

			var pt = new Size(
				(client_size.Width * XAxis.Span / bounds.Width),
				(client_size.Height * YAxis.Span / bounds.Height));

			return pt;
		}
		public Vector ClientToChart(Vector client_vec)
		{
			var bounds = SceneBounds;
			if (bounds.Width == 0 || bounds.Height == 0)
				throw new Exception("Chart size is zero, cannot convert from client space to chart space");

			return new Vector(
				(+client_vec.X * XAxis.Span / bounds.Width),
				(-client_vec.Y * YAxis.Span / bounds.Height));
		}
		public Rect ClientToChart(Rect client_rect, bool scene_relative = false)
		{
			return new Rect(
				ClientToChart(client_rect.Location, scene_relative),
				ClientToChart(client_rect.Size));
		}

		/// <summary>Returns a point in client space from a point in chart space. Inverse of ClientToChart</summary>
		//[Obsolete]
		public Point ChartToClient(Point chart_point, bool scene_relative = false)
		{
			var bounds = SceneBounds;
			if (bounds.Width == 0 || bounds.Height == 0)
				throw new Exception("Chart size is zero, cannot convert from chart space to client space");

			var pt = new Point(
				(bounds.Left + (chart_point.X - XAxis.Min) * bounds.Width / XAxis.Span),
				(bounds.Bottom - (chart_point.Y - YAxis.Min) * bounds.Height / YAxis.Span));

			// If the caller wants the point relative to the 3d scene, add the offset
			if (scene_relative)
			{
				pt.X -= bounds.X;
				pt.Y -= bounds.Y;
			}

			return pt;
		}
		public Size ChartToClient(Size chart_size)
		{
			var bounds = SceneBounds;
			if (bounds.Width == 0 || bounds.Height == 0)
				throw new Exception("Chart size is zero, cannot convert from chart space to client space");

			return new Size(
				(chart_size.Width * bounds.Width / XAxis.Span),
				(chart_size.Height * bounds.Height / YAxis.Span));
		}
		public Vector ChartToClient(Vector chart_vec)
		{
			var bounds = SceneBounds;
			if (bounds.Width == 0 || bounds.Height == 0)
				throw new Exception("Chart size is zero, cannot convert from chart space to client space");

			return new Vector(
				(+chart_vec.X * bounds.Width / XAxis.Span),
				(-chart_vec.Y * bounds.Height / YAxis.Span));
		}
		public Rect ChartToClient(Rect chart_rect, bool scene_relative = false)
		{
			return new Rect(
				ChartToClient(chart_rect.Location, scene_relative),
				ChartToClient(chart_rect.Size));
		}

		/// <summary>Return a point in camera space from a point in chart space (Z = focus plane)</summary>
		public v4 ChartToCamera(Point chart_point)
		{
			// Remove the camera to origin offset
			var origin_cs = Camera.W2O.pos.xy;
			return new v4((float)(chart_point.X + origin_cs.x), (float)(chart_point.Y + origin_cs.y), -Camera.FocusDist, 1f);
		}

		/// <summary>Return a point in chart space from a point in camera space</summary>
		public Point CameraToChart(v4 camera_point)
		{
			Debug.Assert(camera_point.z != 0);

			// Project the camera space point onto the focus plane
			var proj = -Camera.FocusDist / camera_point.z;
			var pt = new v2(camera_point.x * proj, camera_point.y * proj);

			// Add the offset of the camera from the origin
			var origin_cs = Camera.W2O.pos.xy;
			return new Point(pt.x - origin_cs.x, pt.y - origin_cs.y);
		}

		/// <summary>Return a transform from chart space to client space.</summary>
		public m4x4 ChartToClientSpace(Rect plot_area, bool scene_relative = false)
		{
			// e.g.
			//   chart2client * Point(x_min, y_min) = plot_area.BottomLeft()
			//   chart2client * Point(x_max, y_max) = plot_area.TopRight()
			var scale_x = (float)+(plot_area.Width / XAxis.Span);
			var scale_y = (float)-(plot_area.Height / YAxis.Span);
			var offset_x = (float)+(plot_area.Left - XAxis.Min * scale_x);
			var offset_y = (float)+(plot_area.Bottom - YAxis.Min * scale_y);

			// C = chart, c = client
			var C2c = new m4x4(
				new v4(scale_x, 0, 0, 0),
				new v4(0, scale_y, 0, 0),
				new v4(0, 0, 1, 0),
				new v4(offset_x, offset_y, 0, 1));

#if false
			// Check the XAxis corners map to the expected client space locations
			var C_pt0 = new v4((float)XAxis.Min, (float)YAxis.Min, 0, 1);
			var C_pt1 = new v4((float)XAxis.Max, (float)YAxis.Max, 0, 1);
			var c_pt0 = C2c * C_pt0;
			var c_pt1 = C2c * C_pt1;
			Debug.Assert(Math.Abs(c_pt0.x - plot_area.Left  ) < 0.001);
			Debug.Assert(Math.Abs(c_pt0.y - plot_area.Bottom) < 0.001);
			Debug.Assert(Math.Abs(c_pt1.x - plot_area.Right ) < 0.001);
			Debug.Assert(Math.Abs(c_pt1.y - plot_area.Top   ) < 0.001);
#endif

			// If caller whats the transform to be 3d-scene relative, remove the offset
			if (scene_relative)
			{
				C2c.pos.x -= (float)plot_area.X;
				C2c.pos.y -= (float)plot_area.Y;
			}

			return C2c;
		}
		public m4x4 ChartToClientSpace(bool scene_relative = false)
		{
			return ChartToClientSpace(SceneBounds, scene_relative);
		}

		/// <summary>Return a transform from client space to chart space.</summary>
		public m4x4 ClientToChartSpace(Rect plot_area, bool scene_relative = false)
		{
			// e.g.
			//  client2chart * plot_area.BottomLeft() = Point(x_min, y_min)
			//  client2chart * plot_area.TopRight()   = Point(x_max, y_max)
			var scale_x = (float)+(XAxis.Span / plot_area.Width);
			var scale_y = (float)-(YAxis.Span / plot_area.Height);
			var offset_x = (float)+(XAxis.Min - plot_area.Left * scale_x);
			var offset_y = (float)+(YAxis.Min - plot_area.Bottom * scale_y);

			// C = chart, c = client
			var c2C = new m4x4(
				new v4(scale_x, 0, 0, 0),
				new v4(0, scale_y, 0, 0),
				new v4(0, 0, 1, 0),
				new v4(offset_x, offset_y, 0, 1));

#if false
			// Check the plot_area corners map to the expected graph space locations
			var c_pt0 = new v4((float)plot_area.Left, (float)plot_area.Bottom, 0, 1);
			var c_pt1 = new v4((float)plot_area.Right, (float)plot_area.Top, 0, 1);
			var C_pt0 = c2C * c_pt0;
			var C_pt1 = c2C * c_pt1;
			Debug.Assert(Math.Abs(C_pt0.x - XAxis.Min) < 0.001f);
			Debug.Assert(Math.Abs(C_pt0.y - YAxis.Min) < 0.001f);
			Debug.Assert(Math.Abs(C_pt1.x - XAxis.Max) < 0.001f);
			Debug.Assert(Math.Abs(C_pt1.y - YAxis.Max) < 0.001f);
#endif

			// If the caller wants the transform relative to the 3d-scene, add the offset
			if (scene_relative)
			{
				c2C.pos.x += (float)plot_area.Left * scale_x;
				c2C.pos.y += (float)plot_area.Bottom * scale_y;
			}

			return c2C;
		}
		public m4x4 ClientToChartSpace(bool scene_relative = false)
		{
			return ClientToChartSpace(SceneBounds, scene_relative);
		}

		/// <summary>Convert between client space and normalised screen space</summary>
		public Point ClientToNSS(Point client_point)
		{
			var bounds = SceneBounds;
			return new Point(
				((client_point.X - bounds.Left) / bounds.Width) * 2 - 1,
				((bounds.Bottom - client_point.Y) / bounds.Height) * 2 - 1);
		}
		public Size ClientToNSS(Size client_size)
		{
			var bounds = SceneBounds;
			return new Size(
				(client_size.Width / bounds.Width) * 2,
				(client_size.Height / bounds.Height) * 2);
		}
		public Rect ClientToNSS(Rect client_rect)
		{
			return new Rect(
				ClientToNSS(client_rect.Location),
				ClientToNSS(client_rect.Size));
		}
		public Point NSSToClient(Point nss_point)
		{
			var bounds = SceneBounds;
			return new Point(
				bounds.Left + 0.5 * (nss_point.X + 1) * bounds.Width,
				bounds.Bottom - 0.5 * (nss_point.Y + 1) * bounds.Height);
		}
		public Size NSSToClient(Size nss_size)
		{
			var bounds = SceneBounds;
			return new Size(
				0.5 * nss_size.Width * bounds.Width,
				0.5 * nss_size.Height * bounds.Height);
		}
		public Rect NSSToClient(Rect nss_rect)
		{
			return new Rect(
				NSSToClient(nss_rect.Location),
				NSSToClient(nss_rect.Size));
		}
#endif

		/// <summary>Perform a hit test on the chart but only test which zone is hit (faster)</summary>
		public HitTestResult HitTestZone(Point client_point, ModifierKeys modifier_keys, EMouseBtns mouse_btns)
		{
			// If the scene is not being displayed, we can't convert the point from client space to scene space
			if (PresentationSource.FromVisual(this) == null)
				return new HitTestResult(EZone.None, v2.Zero, v4.Zero, modifier_keys, mouse_btns, Array.Empty<HitTestResult.Hit>(), Scene.Camera);

			// Determine the hit zone of the control
			var zone = EZone.None;
			if (SceneBounds.Contains(client_point)) zone |= EZone.Chart;
			if (XAxisBounds.Contains(client_point)) zone |= EZone.XAxis;
			if (YAxisBounds.Contains(client_point)) zone |= EZone.YAxis;
			if (TitleBounds.Contains(client_point)) zone |= EZone.Title;

			// Convert the client control space point to a scene space point
			var scene_point = Gui_.MapPoint(this, Scene, client_point).ToV2();

			// Get the hit test point in scene space
			var chart_point = SceneToChart(scene_point);
			return new HitTestResult(zone, scene_point, chart_point, modifier_keys, mouse_btns, new List<HitTestResult.Hit>(), Scene.Camera);
		}

		/// <summary>Perform a hit test on the chart and all elements within the chart</summary>
		public HitTestResult HitTest(Point client_point, ModifierKeys modifier_keys, EMouseBtns mouse_btns)
		{
			return HitTest(client_point, modifier_keys, mouse_btns, x => x.Visible && x.Enabled);
		}

		/// <summary>Perform a hit test on the chart and all elements within the chart (filtering elements using 'pred')</summary>
		public HitTestResult HitTest(Point client_point, ModifierKeys modifier_keys, EMouseBtns mouse_btns, Func<Element, bool>? pred)
		{
			pred ??= x => x.Visible && x.Enabled;
			return HitTest(client_point, modifier_keys, mouse_btns, HitTestElements);
			void HitTestElements(IEnumerable<Element> elements, HitTestResult result)
			{
				foreach (var elem in elements.Where(pred))
				{
					// Hit test each element
					var hit = elem.HitTest(result.ChartPoint, result.ScenePoint, result.ModifierKeys, result.MouseBtns, Scene.Camera);
					if (hit != null)
						result.Hits.Add(hit);
				}
			}
		}

		/// <summary>Perform a hit test on the chart and all elements within the chart (using a custom function for testing elements)</summary>
		public HitTestResult HitTest(Point client_point, ModifierKeys modifier_keys, EMouseBtns mouse_btns, Action<IEnumerable<Element>, HitTestResult> element_hit_func)
		{
			// Hit test the zones of the chart
			var result = HitTestZone(client_point, modifier_keys, mouse_btns);

			// If the 'Scene' is hit, find elements that overlap 'client_point'.
			// 'element_hit_func' allows all elements to be hit tested in one go if wanted.
			if (result.Zone.HasFlag(EZone.Chart))
			{
				// Hit test the elements
				element_hit_func(Elements, result);

				// Sort the results by z order
				result.Hits.Sort((l, r) => -l.Element.PositionZ.CompareTo(r.Element.PositionZ));
			}
			return result;
		}

		/// <summary>Show/Hide a ray from the camera through the mouse pointer</summary>
		public bool ShowHitTestRay
		{
			get => m_hit_test_ray != null;
			set
			{
				if (ShowHitTestRay == value) return;
				if (ShowHitTestRay)
				{
					MouseMove -= UpdateHitTestRay;
					Util.Dispose(m_hit_test_ray);
				}
				m_hit_test_ray = value ? new View3d.Object(new LdrBuilder().Line("HitTestRay", Colour32.Green, v4.Origin, v4.Origin), false, null) : null;
				if (ShowHitTestRay)
				{
					MouseMove += UpdateHitTestRay;
				}
				NotifyPropertyChanged(nameof(ShowHitTestRay));

				// Handler
				void UpdateHitTestRay(object sender, MouseEventArgs e)
				{
					if (m_hit_test_ray == null)
						return;

					var pt = e.GetPosition(this);
					var zone = HitTestZone(pt, ModifierKeys.None, EMouseBtns.None);
					if (zone.Zone == EZone.Chart)
					{
						// Remember, this will only be a point on-screen when the camera is not orthographic
						pt = e.GetPosition(Scene);
						var cam = Scene.Camera;
						var ray = cam.RaySS(pt.ToV2());
						var pt0 = ray.m_ws_origin + ray.m_ws_direction * cam.NearPlane / -Math_.Dot(cam.O2W.z, ray.m_ws_direction);
						var pt1 = ray.m_ws_origin + ray.m_ws_direction * cam.FocusDist / -Math_.Dot(cam.O2W.z, ray.m_ws_direction);
						m_hit_test_ray.UpdateModel(new LdrBuilder().Line(pt0, pt1), View3d.EUpdateObject.Model);
						Scene.AddObject(m_hit_test_ray);
					}
					else
					{
						Scene.RemoveObject(m_hit_test_ray);
					}
					Scene.Invalidate();
				}
			}
		}
		private View3d.Object? m_hit_test_ray;

		/// <summary>Find the default range, then reset to the default range</summary>
		public void AutoRange(View3d.ESceneBounds who = View3d.ESceneBounds.All, EAxis axes = EAxis.Both, double inflate = 1.0)
		{
			// Notes:
			//  - AutoRange only considers graphics added to the scene.
			//  - ChartDataSeries' use a caching system and only add pieces of the series data that are witihn the visible X range.
			//    AutoRange should not show the full data series range because it is probably too big (that's why it's cached). You
			//    will need to manually set the range when using on-demand graphics like ChartDataSeries plots.

			// Allow the auto range to be handled by event
			var args = new AutoRangeEventArgs(who, axes);
			OnAutoRanging(args);

			// Get the bounding box, either from the event or from the scene bounds
			var bbox = args.Handled ? args.ViewBBox : Window.SceneBounds(who);

			// Check it's valid
			if (!bbox.IsValid)
				throw new Exception($"View bounding box is invalid: {bbox}");
			if (bbox.Radius == v4.Zero)
				bbox.Radius = 0.5f * v4.One;

			// Inflate the bounding box
			bbox.Radius *= inflate;

			// Position the camera to view the bounding box
			var up = Camera.AlignAxis != v4.Zero ? Camera.AlignAxis : Options.ResetUp;
			var fwd = !Math_.Parallel(Options.ResetForward, up) ? Options.ResetForward : Math_.Perpendicular(up);
			Camera.ResetView(bbox, fwd, up, dist: 0f, preserve_aspect: LockAspect, commit: true);

			// Set the desired pixel aspect ratio.
			if (Scene.RenderTarget != null)
			{
				// In WPF, controls don't get an 'owning window' until they are visible.
				// The view3d control uses a D3D11Image which sets it's back buffer once the window
				// handle is known. The size of the back buffer defines the size of the viewport.
				// So, if we don't have an owning window the viewport will have width/height = 0/0.
				var viewport = Scene.Window.Viewport;
				Scene.DesiredPixelAspect = Camera.Aspect * viewport.Height / viewport.Width;
			}

			// Set the axis range from the camera position
			SetRangeFromCamera();
			Invalidate();
		}

		/// <summary>Set the axis range based on the position of the camera and the field of view</summary>
		public void SetRangeFromCamera()
		{
			// The span of the X/Y axis is determined by the FoV and the focus point distance.
			// Set the projection mode before 'ViewArea' because it depends on the projection mode.
			Scene.Camera.Orthographic = Options.Orthographic;
			Scene.ActualPixelAspect = Scene.DesiredPixelAspect;
			var wh = Scene.Camera.ViewArea(Scene.Camera.FocusDist);

			// Set the axes range
			var w2c = Math_.InvertAffine(Scene.Camera.O2W);
			var xmin = -w2c.pos.x - wh.x * 0.5;
			var xmax = -w2c.pos.x + wh.x * 0.5;
			var ymin = -w2c.pos.y - wh.y * 0.5;
			var ymax = -w2c.pos.y + wh.y * 0.5;
			if (xmin < xmax) XAxis.Set(xmin, xmax);
			if (ymin < ymax) YAxis.Set(ymin, ymax);

			SaveNavCheckpoint();
		}

		/// <summary>Position the camera based on the axis range</summary>
		public void SetCameraFromRange()
		{
			// Notes:
			//  - The grid is always parallel to the focus plane of the camera.
			//  - The camera forward vector points at the centre of the grid.
			//  - The Grid (0,0) point is world origin projected onto the focus plane along the
			//    camera Z axis (*not* the ray from origin to camera) but should it??

			// Set the new desired pixel aspect based on the
			// axis ranges and the current size of the scene.
			var sz = SceneBounds;
			if (sz.Width != 0 && sz.Height != 0)
			{
				Scene.DesiredPixelAspect = Range.Aspect * sz.Height / sz.Width;
				Scene.ActualPixelAspect = Scene.DesiredPixelAspect;
			}

			// Find the world space position of the new focus point.
			// Move the focus point within the focus plane (parallel to the camera XY plane)
			// and adjust the focus distance so that the view has a width equal to XAxis.Span.
			// The camera aspect ratio should ensure the YAxis is correct.
			var c2w = Scene.Camera.O2W;
			var focus =
				c2w.x * (float)XAxis.Centre +
				c2w.y * (float)YAxis.Centre +
				c2w.z * Math_.Dot(Scene.Camera.FocusPoint - v4.Origin, c2w.z);

			// Move the camera in the camera Z axis direction so that the width/height at the
			// focus distance matches the XAxis/YAxis range. Use the Fov that is closest to 90deg
			//   Tan(FovX/2) = (XAxis.Span/2)/d  or  Tan(FovY/2) = (YAxis.Span/2)/d
			var use_xaxis = Math_.Abs(Scene.Camera.Fov.x - Math_.TauBy4F) < Math_.Abs(Scene.Camera.Fov.y - Math_.TauBy4F);
			var span = use_xaxis ? XAxis.Span : YAxis.Span;
			var fov = use_xaxis ? Scene.Camera.Fov.x : Scene.Camera.Fov.y;
			fov = Math_.Clamp(fov, float.Epsilon, Math_.TauBy2F - float.Epsilon);
			Scene.Camera.FocusDist = (float)(span / (2.0 * Math.Tan(fov / 2.0)));

			c2w.pos = focus.w1 + Scene.Camera.FocusDist * c2w.z;

			Scene.Camera.O2W = c2w;
			Scene.Camera.Commit();

			SaveNavCheckpoint();
		}

		/// <summary>The Z value of the highest element in the diagram</summary>
		private float HighestZ { get; set; }

		/// <summary>The Z value of the lowest element in the diagram</summary>
		private float LowestZ { get; set; }

		/// <summary>Set the Z value for all elements.</summary>
		private void UpdateElementZOrder()
		{
			LowestZ = 0f;
			HighestZ = 0f;

			foreach (var elem in Elements)
				elem.PositionZ = HighestZ += Camera.FocusDist * 0.001f;
		}

		/// <summary>Return a string representation of a location on the chart. 'location' is in ChartControl client space</summary>
		public string LocationText(Point client_point)
		{
			var bounds = SceneBounds;
			if (bounds.Width == 0 || bounds.Height == 0)
				return string.Empty;

			var scene_point = Gui_.MapPoint(this, Scene, client_point).ToV2();
			var chart_point = SceneToChart(scene_point);
			XAxis.GridLines(out var minx, out var maxx, out var stepx);
			YAxis.GridLines(out var miny, out var maxy, out var stepy);
			var xtick = XAxis.TickText(chart_point.x, stepx);
			var ytick = YAxis.TickText(chart_point.y, stepy);
			return $"{xtick} , {ytick}";
		}

		/// <summary>Return the current mouse pointer location as a string</summary>
		public string PointerLocationText => LocationText(Mouse.GetPosition(this));

		/// <summary>The client space rectangle that contains the View3d part of the control</summary>
		public Rect SceneBounds => Scene.RenderArea(this);

		/// <summary>The client space area of the XAxis area of the chart</summary>
		public Rect XAxisBounds => Gui_.FindVisualChild<AxisPanel>(this, x => x.Axis == XAxis)?.RenderArea(this) ?? Rect_.Zero;

		/// <summary>The client space area of the YAxis area of the chart</summary>
		public Rect YAxisBounds => Gui_.FindVisualChild<AxisPanel>(this, x => x.Axis == YAxis)?.RenderArea(this) ?? Rect_.Zero;

		/// <summary>The client space area of the chart title</summary>
		public Rect TitleBounds => m_title_label.RenderArea(this);

		/// <summary>Per button current mouse operation</summary>
		public MouseOps MouseOperations
		{
			get;
			private set
			{
				if (field == value) return;
				Util.Dispose(ref field!);
				field = value;
			}
		} = null!;

		/// <summary>
		/// Select elements that are wholly within 'chart_bbox'. (chart_bbox is axis aligned in chart space)
		/// If no modifier keys are down, elements not in 'chart_bbox' are deselected.
		/// If 'shift' is down, elements within 'chart_bbox' are selected in addition to the existing selection
		/// If 'ctrl' is down, elements within 'chart_bbox' are removed from the existing selection.</summary>
		public void SelectElements(BBox chart_bbox, ModifierKeys modifier_keys, EMouseBtns mouse_btns)
		{
			if (!Options.AllowSelection)
				return;

			// If the area of selection is less than the min drag distance, assume click selection
			var scene_bbox = ChartToScene(chart_bbox);
			var is_click = Math_.LengthSq(scene_bbox.Width, scene_bbox.Height) < Math_.Sqr(Options.MinDragPixelDistance);
			if (is_click)
			{
				// Hit test for elements to add to the selection
				var scene_point = scene_bbox.Centre();
				var client_point = Gui_.MapPoint(Scene, this, scene_point);
				var hits = HitTest(client_point, modifier_keys, mouse_btns);

				// If control is down, deselect the first selected element in the hit list
				if (modifier_keys.HasFlag(ModifierKeys.Control))
				{
					var first = hits.Hits.FirstOrDefault(x => x.Element.Selected);
					if (first != null)
						first.Element.Selected = false;
				}
				// If shift is down, select the first element not already selected in the hit list
				else if (modifier_keys.HasFlag(ModifierKeys.Shift))
				{
					var first = hits.Hits.FirstOrDefault(x => !x.Element.Selected);
					if (first != null)
						first.Element.Selected = true;
				}
				// Otherwise select the next element below the current selection or the top
				// element if nothing is selected. Clear all other selections
				else
				{
					// Find the element to select
					var to_select = hits.Hits.FirstOrDefault();
					for (int i = 0; i < hits.Hits.Count - 1; ++i)
					{
						if (!hits.Hits[i].Element.Selected) continue;
						to_select = hits.Hits[i + 1];
						break;
					}

					// Deselect all elements and select the next element
					Selected.Clear();
					if (to_select != null)
						to_select.Element.Selected = true;
				}
			}
			// Otherwise it's area selection
			else
			{
				using var sus = Selected.SuspendEvents(true);

				// If control is down, those in the selection area become deselected
				if (modifier_keys.HasFlag(ModifierKeys.Control))
				{
					// Only need to look in the selected list
					foreach (var elem in Selected.Where(x => chart_bbox.IsWithin(x.Bounds, 0f)).ToArray())
						elem.Selected = false;
				}
				// If shift is down, those in the selection area become selected
				else if (modifier_keys.HasFlag(ModifierKeys.Shift))
				{
					foreach (var elem in Elements.Where(x => !x.Selected && chart_bbox.IsWithin(x.Bounds, 0f)))
						elem.Selected = true;
				}
				// Otherwise, the existing selection is cleared, and those within the selection area become selected
				else
				{
					foreach (var elem in Elements)
						elem.Selected = chart_bbox.IsWithin(elem.Bounds, 0f);
				}
			}

			InvalidateArrange();
		}

		/// <summary>Position the axis panels based on their 'Side' values</summary>
		private void PositionAxisPanels()
		{
			{// XAxis
				Grid.SetRow(m_xaxis_label, Options.XAxis.Side == Dock.Top ? 1 : 5);
				Grid.SetRow(m_xaxis_panel, Options.XAxis.Side == Dock.Top ? 2 : 4);
			}
			{// YAxis
				Grid.SetColumn(m_yaxis_label, Options.YAxis.Side == Dock.Left ? 0 : 4);
				Grid.SetColumn(m_yaxis_panel, Options.YAxis.Side == Dock.Left ? 1 : 3);
			}
		}

		/// <summary>The chart coordinates at the current mouse pointer location</summary>
		public string ValueAtPointer => LocationText(Mouse.GetPosition(this));

		/// <summary>Callback for formatting the text display by the tape measure</summary>
		public Func<v4, v4, TapeMeasure.LabelText> TapeMeasureStringFormat { get; set; } = DefaultTapeMeasureStringFormat;
		public static TapeMeasure.LabelText DefaultTapeMeasureStringFormat(v4 beg, v4 end)
		{
			var dx = end.x - beg.x;
			var dy = end.y - beg.y;
			return new TapeMeasure.LabelText
			{
				LabelX = $"{dx:F3}",
				LabelY = $"{dy:F3}",
				LabelD = $"{Math_.Length0(dx, dy):F3}",
			};
		}

		/// <summary>Binding helpers</summary>
		public Visibility XAxisLabelVisibility => Options.ShowAxes && XAxis.Label.HasValue() ? Visibility.Visible : Visibility.Collapsed;
		public Visibility YAxisLabelVisibility => Options.ShowAxes && YAxis.Label.HasValue() ? Visibility.Visible : Visibility.Collapsed;

		/// <inheritdoc/>
		public event PropertyChangedEventHandler? PropertyChanged;
		public void NotifyPropertyChanged(string prop_name) => PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(prop_name));

		/// <summary>Chart control graphics context id</summary>
		public static readonly Guid CtxId = new("62D495BB-36D1-4B52-A067-1B7DB4011831");

		#region Self Consistency

		/// <summary>Check the self consistency of elements</summary>
		public bool CheckConsistency()
		{
			if (m_consistency_check_ref_count != 0) return true;
			try { CheckConsistencyInternal(); }
			catch (Exception ex)
			{
				Debug.WriteLine(ex.MessageFull());
				return false;
			}
			return true;
		}

		/// <summary>Check consistency of the diagram elements</summary>
		protected virtual void CheckConsistencyInternal()
		{
			// Take a copy to prevent events being raised
			var elements = Elements.ToArray();

			// The elements collection should be distinct
			elements.Sort(ByGuid);
			for (int i = 0; i < elements.Length - 1; ++i)
			{
				if (!Equals(elements[i].Id, elements[i + 1].Id)) continue;
				throw new Exception($"Element {elements[i]} is in the Elements collection more than once");
			}

			// All elements in the chart should have their Chart property set to this chart
			foreach (var elem in Elements)
			{
				if (elem.Chart == this) continue;
				throw new Exception($"Element {elem} is in the Elements collection but does not have its Chart property set correctly");
			}

			// The selected collection contains all that is selected and no more
			var selected0 = elements.Where(x => x.Selected).ToArray().QuickSort(ByGuid);
			var selected1 = Selected.ToArray().QuickSort(ByGuid);
			if (!selected0.SequenceEqual(selected1, ByGuid))
				throw new Exception("Selected elements collection is inconsistent with the selected state of the elements");

			// The 'ElemIds' should match the Elements collection
			if (Elements.Ids.Count != Elements.Count)
				throw new Exception("Elements collection and Element Id lookup table don't match");
			foreach (var elem in Elements)
				if (Elements.Ids.GetOrDefault(elem.Id) != elem)
					throw new Exception($"Element {elem} is not in the Element Id lookup table");

			// Check the consistency of all elements
			foreach (var elem in Elements)
				elem.CheckConsistency();
		}

		/// <summary>Temporarily disable the consistency check</summary>
		public Scope SuspendConsistencyCheck(bool check_on_exit = true)
		{
			return Scope.Create(
				() => ++m_consistency_check_ref_count,
				() =>
				{
					--m_consistency_check_ref_count;
					if (check_on_exit && m_consistency_check_ref_count == 0)
						Debug.Assert(CheckConsistency());
				});
		}
		private int m_consistency_check_ref_count;

		/// <summary>Element sorting predicates</summary>
		protected Cmp<Element> ByGuid = Cmp<Element>.From((l, r) => l.Id.CompareTo(r.Id));

		#endregion
	}
}
