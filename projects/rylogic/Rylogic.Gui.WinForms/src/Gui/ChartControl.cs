//***************************************************
// Chart Control
// Copyright (C) Rylogic Ltd 2016
//***************************************************
using System;
using System.Collections;
using System.Collections.Generic;
using System.ComponentModel;
using System.Diagnostics;
using System.Drawing;
using System.Linq;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading;
using System.Windows.Forms;
using System.Windows.Threading;
using System.Xml.Linq;
using Rylogic.Attrib;
using Rylogic.Common;
using Rylogic.Container;
using Rylogic.Extn;
using Rylogic.Gfx;
using Rylogic.LDraw;
using Rylogic.Maths;
using Rylogic.Utility;
using Rylogic.Interop.Win32;
using Matrix = System.Drawing.Drawing2D.Matrix;

namespace Rylogic.Gui.WinForms
{
	/// <summary>A view 3d based chart control</summary>
	public class ChartControl :UserControl
	{
		// Notes:
		// - Two methods of camera control; 1. directly position the camera, or 2. set the
		//   axis range, then determine the camera position from that. Both systems are used.
		//   for mouse control, the camera is positioned and the axis range updated based on
		//   the camera position. For ranging, or fixed chart positioning, the camera position
		//   is moved to match the axis range.
		// - After setting the axis range, call SetCameraFromRange()
		// - After setting the camera position, call SetRangeFromCamera()
		// - Mouse wheel moves the camera in the camera z direction
		// - Mouse wheel on an axis (changing the aspect ratio) changes the FoV

		/// <summary>Axis type</summary>
		[Flags] public enum EAxis
		{
			None = 0,
			XAxis = 1 << 0,
			YAxis = 1 << 1,
			Both = XAxis | YAxis,
		}

		public ChartControl()
			:this(string.Empty, new RdrOptions())
		{ }
		public ChartControl(string title, RdrOptions options)
		{
			SetStyle(ControlStyles.ResizeRedraw, true);
			DoubleBuffered = true;
			InitializeComponent();
			try
			{
				Options         = options;
				Title           = title;
				Range           = new RangeData(this);
				BaseRangeX      = new RangeF(0.0, 1.0);
				BaseRangeY      = new RangeF(0.0, 1.0);
				Scene           = new ChartPanel(this); // Must come after 'Range'
				MouseOperations = new MouseOps();
				Tools           = new ChartTools(this, Options);
				m_tt_show_value = new HintBalloon { AutoSizeMode = AutoSizeMode.GrowOnly };
				m_chart_frame   = new ChartFrame(this);

				Elements = new BindingListEx<Element> { PerItem = true, UseHashSet = true };
				Selected = new BindingListEx<Element> { PerItem = false };
				Hovered  = new BindingListEx<Element> { PerItem = false };
				ElemIds  = new Dictionary<Guid, Element>();

				Elements.ListChanging += (s,a) => OnElementListChanging(a);
				Selected.ListChanging += (s,a) => OnSelectionListChanging(a);
				Hovered .ListChanging += (s,a) => OnHoveredListChanging(a);

				AllowEditing = false;
				AllowSelection = false;
				DefaultMouseControl = true;
				DefaultKeyboardShortcuts = true;
				AreaSelectMode = EAreaSelectMode.Zoom;
			}
			catch
			{
				Dispose();
				throw;
			}
		}
		protected override void Dispose(bool disposing)
		{
			MouseOperations = null;
			Tools = null;
			Range = null;
			Scene = null;
			Options = null;
			Util.Dispose(ref components);
			base.Dispose(disposing);
		}
		protected override void OnLayout(LayoutEventArgs e)
		{
			if (Scene != null && !DesignMode)
			{
				using (this.SuspendLayout(false))
				{
					var dims = ChartDimensions;
					SceneBounds = dims.ChartArea;
					if (Options.LockAspect != null && dims.ChartArea.Area() != 0)
						Aspect = Options.LockAspect.Value;
				}
			}
			base.OnLayout(e);
		}
		protected override void OnPaint(PaintEventArgs e)
		{
			base.OnPaint(e);
			if (DesignMode) return;
			DoPaint(e.Graphics);
		}
		protected override void OnInvalidated(InvalidateEventArgs e)
		{
			Scene?.Invalidate();
			base.OnInvalidated(e);
		}

		/// <summary>Rendering options for the chart</summary>
		[Browsable(false), DesignerSerializationVisibility(DesignerSerializationVisibility.Hidden)]
		public RdrOptions Options
		{
			[DebuggerStepThrough] get { return m_rdr_options; }
			set
			{
				if (m_rdr_options == value) return;
				if (m_rdr_options != null) m_rdr_options.PropertyChanged -= HandleRdrOptionsChanged;
				m_rdr_options = value;
				if (m_rdr_options != null) m_rdr_options.PropertyChanged += HandleRdrOptionsChanged;
			}
		}
		private RdrOptions m_rdr_options;
		public event EventHandler OptionsChanged;
		protected virtual void OnRdrOptionsChanged()
		{
			OptionsChanged?.Invoke(this, EventArgs.Empty);
		}
		private void HandleRdrOptionsChanged(object sender, PropertyChangedEventArgs e)
		{
			OnRdrOptionsChanged();
		}

		/// <summary>The title of the chart</summary>
		[Browsable(false), DesignerSerializationVisibility(DesignerSerializationVisibility.Hidden)]
		public string Title
		{
			get { return m_title ?? string.Empty; }
			set
			{
				if (m_title == value) return;
				m_title = value;
				Invalidate();
			}
		}
		private string m_title;

		/// <summary>The current X/Y axis range of the chart</summary>
		[Browsable(false), DesignerSerializationVisibility(DesignerSerializationVisibility.Hidden)]
		public RangeData Range
		{
			[DebuggerStepThrough] get { return m_range; }
			private set
			{
				if (value == m_range) return;
				Util.Dispose(ref m_range);
				m_range = value;
			}
		}
		private RangeData m_range;

		/// <summary>Accessor to the current X axis</summary>
		[Browsable(false), DesignerSerializationVisibility(DesignerSerializationVisibility.Hidden)]
		public RangeData.Axis XAxis
		{
			[DebuggerStepThrough] get { return Range.XAxis; }
		}

		/// <summary>Accessor to the current Y axis</summary>
		[Browsable(false), DesignerSerializationVisibility(DesignerSerializationVisibility.Hidden)]
		public RangeData.Axis YAxis
		{
			[DebuggerStepThrough] get { return Range.YAxis; }
		}

		/// <summary>Default X axis range of the chart</summary>
		[Browsable(false), DesignerSerializationVisibility(DesignerSerializationVisibility.Hidden)]
		public RangeF BaseRangeX
		{
			[DebuggerStepThrough] get;
			set;
		}

		/// <summary>Default Y axis range of the chart</summary>
		[Browsable(false), DesignerSerializationVisibility(DesignerSerializationVisibility.Hidden)]
		public RangeF BaseRangeY
		{
			[DebuggerStepThrough] get;
			set;
		}

		/// <summary>Get/Set the aspect ratio of the chart area</summary>
		[Browsable(false), DesignerSerializationVisibility(DesignerSerializationVisibility.Hidden)]
		public double Aspect
		{
			get { return Camera.Aspect * SceneBounds.Height / SceneBounds.Width; }
			set
			{
				if (Aspect == value) return;
				if (Options.LockAspect != null) Options.LockAspect = value;
				Camera.Aspect = (float)(value * SceneBounds.Width / SceneBounds.Height);
				SetRangeFromCamera();
				Invalidate();
			}
		}

		/// <summary>The view3d part of the chart</summary>
		[Browsable(false), DesignerSerializationVisibility(DesignerSerializationVisibility.Hidden)]
		public ChartPanel Scene
		{
			[DebuggerStepThrough] get { return m_impl_scene; }
			private set
			{
				if (m_impl_scene == value) return;
				using (this.SuspendLayout(false))
				{
					if (m_impl_scene != null)
					{
						Controls.Remove(m_impl_scene);
						Util.Dispose(ref m_impl_scene);
					}
					m_impl_scene = value;
					if (m_impl_scene != null)
					{
						Controls.Add(m_impl_scene);
					}
				}
			}
		}
		private ChartPanel m_impl_scene;

		/// <summary>Renderer</summary>
		public View3d View3d
		{
			[DebuggerStepThrough] get { return Scene?.View3d; }
		}

		/// <summary>The view3d window for this control instance</summary>
		public View3d.Window Window
		{
			[DebuggerStepThrough] get { return Scene?.Window; }
		}

		/// <summary>The view of the chart</summary>
		public View3d.Camera Camera
		{
			[DebuggerStepThrough] get { return Scene?.Camera; }
		}

		/// <summary>All chart objects</summary>
		public BindingListEx<Element> Elements { get; private set; }

		/// <summary>The set of selected chart elements</summary>
		public BindingListEx<Element> Selected { get; private set; }

		/// <summary>The set of hovered chart elements</summary>
		public BindingListEx<Element> Hovered { get; private set; }

		/// <summary>Associative array from Element Ids to Elements</summary>
		public Dictionary<Guid, Element> ElemIds { get; private set; }

		/// <summary>Raised whenever the view3d area of the chart is scrolled or zoomed</summary>
		public event EventHandler<ChartMovedEventArgs> ChartMoved;
		protected virtual void OnChartMoved(ChartMovedEventArgs args)
		{
			ChartMoved?.Invoke(this, args);
		}
		//private void RaiseChartMoved(ChartMovedEventArgs args)
		//{
		//	if (!ChartMoved.IsSuspended())
		//		OnChartMoved(args);
		//}

		/// <summary>Raised whenever elements in the chart have been edited or moved</summary>
		public event EventHandler<ChartChangedEventArgs> ChartChanged;
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
		//private ChartChangedEventArgs RaiseChartChanged(ChartChangedEventArgs args)
		//{
		//	if (!ChartChanged.IsSuspended())
		//		OnChartChanged(args);
		//
		//	return args;
		//}

		/// <summary>Called just before the chart renders. Used to add View3d objects to the scene</summary>
		public event EventHandler<ChartRenderingEventArgs> ChartRendering;
		protected virtual void OnChartRendering(ChartRenderingEventArgs args)
		{
			ChartRendering?.Invoke(this, args);
		}
		//private void RaiseChartRendering(ChartRenderingEventArgs args)
		//{
		//	if (!ChartRendering.IsSuspended())
		//		OnChartRendering(args);
		//}

		/// <summary>Called after the chart has painted, allowing users to add graphics on top of the chart</summary>
		public event EventHandler<AddOverlaysOnPaintEventArgs> AddOverlaysOnPaint;
		protected virtual void OnAddOverlaysOnPaint(AddOverlaysOnPaintEventArgs args)
		{
			AddOverlaysOnPaint?.Invoke(this, args);
		}
		//private void RaiseAddOverlaysOnPaint(AddOverlaysOnPaintEventArgs args)
		//{
		//	if (!AddOverlaysOnPaint.IsSuspended())
		//		OnAddOverlaysOnPaint(args);
		//}

		/// <summary>Called during AutoRange to allow handlers to set the auto range</summary>
		public event EventHandler<AutoRangeEventArgs> AutoRanging;
		protected virtual void OnAutoRanging(AutoRangeEventArgs args)
		{
			AutoRanging?.Invoke(this, args);
		}
		//private void RaiseOnAutoRanging(AutoRangeEventArgs args)
		//{
		//	if (!AutoRanging.IsSuspended())
		//		OnAutoRanging(args);
		//}

		/// <summary>The client space rectangle that contains the View3d part of the control</summary>
		public Rectangle SceneBounds
		{
			get { return Scene.Bounds; }
			private set { Scene.Bounds = value; }
		}

		/// <summary>Returns a point in chart space from a point in client space. Use to convert mouse (client-space) locations to chart coordinates</summary>
		public PointF ClientToChart(PointF client_point)
		{
			return new PointF(
				(float)(XAxis.Min + (client_point.X - SceneBounds.Left  ) * XAxis.Span / SceneBounds.Width ),
				(float)(YAxis.Min - (client_point.Y - SceneBounds.Bottom) * YAxis.Span / SceneBounds.Height));
		}
		public SizeF ClientToChart(SizeF client_size)
		{
			return new SizeF(
				(float)(client_size.Width  * XAxis.Span / SceneBounds.Width ),
				(float)(client_size.Height * YAxis.Span / SceneBounds.Height));
		}
		public RectangleF ClientToChart(RectangleF client_rect)
		{
			return new RectangleF(
				ClientToChart(client_rect.Location),
				ClientToChart(client_rect.Size));
		}

		/// <summary>Returns a point in client space from a point in chart space. Inverse of ClientToChart</summary>
		public PointF ChartToClient(PointF chart_point)
		{
			return new PointF(
				(float)(SceneBounds.Left   + (chart_point.X - XAxis.Min) * SceneBounds.Width  / XAxis.Span),
				(float)(SceneBounds.Bottom - (chart_point.Y - YAxis.Min) * SceneBounds.Height / YAxis.Span));
		}
		public SizeF ChartToClient(SizeF chart_size)
		{
			return new SizeF(
				(float)(chart_size.Width  * SceneBounds.Width  / XAxis.Span),
				(float)(chart_size.Height * SceneBounds.Height / YAxis.Span));
		}
		public RectangleF ChartToClient(RectangleF chart_rect)
		{
			return new RectangleF(
				ChartToClient(chart_rect.Location),
				ChartToClient(chart_rect.Size));
		}

		/// <summary>Return a point in camera space from a point in chart space (Z = focus plane)</summary>
		public v4 ChartToCamera(PointF chart_point)
		{
			// Remove the camera to origin offset
			var origin_cs = Camera.W2O.pos.xy;
			var pt = new v2(chart_point) + origin_cs;
			return new v4(pt, -Camera.FocusDist, 1f);
		}

		/// <summary>Return a point in chart space from a point in camera space</summary>
		public PointF CameraToChart(v4 camera_point)
		{
			Debug.Assert(camera_point.z != 0);

			// Project the camera space point onto the focus plane
			var proj = -Camera.FocusDist / camera_point.z;
			var pt = new v2(camera_point.x * proj, camera_point.y * proj);

			// Add the offset of the camera from the origin
			var origin_cs = Camera.W2O.pos.xy;
			return pt - origin_cs;
		}

		/// <summary>
		/// Get the scale and translation transform from chart space to client space.
		/// e.g. chart2client * Point(x_min, y_min) = plot_area.BottomLeft()
		///      chart2client * Point(x_max, y_max) = plot_area.TopRight()</summary>
		public m4x4 ChartToClientSpace(Rectangle plot_area)
		{
			var scale_x  = (float)+(plot_area.Width  / XAxis.Span);
			var scale_y  = (float)-(plot_area.Height / YAxis.Span);
			var offset_x = (float)+(plot_area.Left   - XAxis.Min * scale_x);
			var offset_y = (float)+(plot_area.Bottom - YAxis.Min * scale_y);

			// C = chart, c = client
			var C2c = new m4x4(
				new v4(scale_x  , 0        , 0, 0),
				new v4(0        , scale_y  , 0, 0),
				new v4(0        , 0        , 1, 0),
				new v4(offset_x , offset_y , 0, 1));

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

			return C2c;
		}
		public m4x4 ChartToClientSpace()
		{
			return ChartToClientSpace(SceneBounds);
		}

		/// <summary>
		/// Get the scale and translation transform from client space to chart space.
		/// e.g. client2chart * plot_area.BottomLeft() = Point(x_min, y_min)
		///      client2chart * plot_area.TopRight()   = Point(x_max, y_max)</summary>
		public m4x4 ClientToChartSpace(Rectangle plot_area)
		{
			var scale_x  = (float)+(XAxis.Span / plot_area.Width );
			var scale_y  = (float)-(YAxis.Span / plot_area.Height);
			var offset_x = (float)+(XAxis.Min - plot_area.Left   * scale_x);
			var offset_y = (float)+(YAxis.Min - plot_area.Bottom * scale_y);

			// C = chart, c = client
			var c2C = new m4x4(
				new v4(scale_x  , 0        , 0, 0),
				new v4(0        , scale_y  , 0, 0),
				new v4(0        , 0        , 1, 0),
				new v4(offset_x , offset_y , 0, 1));

			#if true
			// Check the plot_area corners map to the expected graph space locations
			var c_pt0 = new v4((float)plot_area.Left , (float)plot_area.Bottom, 0, 1);
			var c_pt1 = new v4((float)plot_area.Right, (float)plot_area.Top   , 0, 1);
			var C_pt0 = c2C * c_pt0;
			var C_pt1 = c2C * c_pt1;
			Debug.Assert(Math.Abs(C_pt0.x - XAxis.Min) < 0.001f);
			Debug.Assert(Math.Abs(C_pt0.y - YAxis.Min) < 0.001f);
			Debug.Assert(Math.Abs(C_pt1.x - XAxis.Max) < 0.001f);
			Debug.Assert(Math.Abs(C_pt1.y - YAxis.Max) < 0.001f);
			#endif

			return c2C;
		}
		public m4x4 ClientToChartSpace()
		{
			return ClientToChartSpace(SceneBounds);
		}

		/// <summary>Convert between client space and normalised screen space</summary>
		public PointF ClientToNSS(PointF client_point)
		{
			return new PointF(
				((client_point.X - SceneBounds.Left) / SceneBounds.Width) * 2f - 1f,
				((SceneBounds.Bottom - client_point.Y) / SceneBounds.Height) * 2f - 1f);
		}
		public SizeF ClientToNSS(SizeF client_size)
		{
			return new SizeF(
				(client_size.Width  / SceneBounds.Width) * 2f,
				(client_size.Height / SceneBounds.Height) * 2f);
		}
		public RectangleF ClientToNSS(RectangleF client_rect)
		{
			return new RectangleF(
				ClientToNSS(client_rect.Location),
				ClientToNSS(client_rect.Size));
		}
		public PointF NSSToClient(PointF nss_point)
		{
			return new PointF(
				SceneBounds.Left + 0.5f * (nss_point.X + 1f) * SceneBounds.Width,
				SceneBounds.Bottom - 0.5f * (nss_point.Y + 1f) * SceneBounds.Height);
		}
		public SizeF NSSToClient(SizeF nss_size)
		{
			return new SizeF(
				0.5f * nss_size.Width  * SceneBounds.Width,
				0.5f * nss_size.Height * SceneBounds.Height);
		}
		public RectangleF NSSToClient(RectangleF nss_rect)
		{
			return new RectangleF(
				NSSToClient(nss_rect.Location),
				NSSToClient(nss_rect.Size));
		}

		/// <summary>Perform a hit test on the chart</summary>
		public HitTestResult HitTestCS(Point client_point, Keys modifier_keys, Func<Element, bool> pred)
		{
			// Determine the hit zone of the control
			var zone = EZone.None;
			if (SceneBounds.Contains(client_point)) zone |= EZone.Chart;
			if (XAxisBounds.Contains(client_point)) zone |= EZone.XAxis;
			if (YAxisBounds.Contains(client_point)) zone |= EZone.YAxis;
			if (TitleBounds.Contains(client_point)) zone |= EZone.Title;

			// The hit test point in chart space
			var chart_point = ClientToChart(client_point);

			// Find elements that overlap 'client_point'
			var hits = new List<HitTestResult.Hit>();
			if (zone.HasFlag(EZone.Chart))
			{
				var elements = pred != null ? Elements.Where(pred) : Elements;
				foreach (var elem in elements)
				{
					var hit = elem.HitTest(chart_point, client_point, modifier_keys, Scene.Camera);
					if (hit != null)
						hits.Add(hit);
				}

				// Sort the results by z order
				hits.Sort((l,r) => -l.Element.PositionZ.CompareTo(r.Element.PositionZ));
			}

			return new HitTestResult(zone, client_point, chart_point, modifier_keys, hits, Scene.Camera);
		}

		/// <summary>Find the default range, then reset to the default range</summary>
		public void AutoRange(View3d.ESceneBounds who = View3d.ESceneBounds.All)
		{
			// Allow the auto range to be handled by event
			var args = new AutoRangeEventArgs(who);
			OnAutoRanging(args);
			if (args.Handled && (!args.ViewBBox.IsValid || args.ViewBBox.Radius == v4.Zero))
				throw new Exception($"Caller provided view bounding box is invalid: {args.ViewBBox}");

			// Get the bounding box to fit into the view
			var bbox = args.Handled
				? args.ViewBBox
				: Window.SceneBounds(who, except:new[] { ChartTools.Id });

			// Position the camera to view the bounding box
			Camera.ResetView(bbox, Options.ResetForward, Options.ResetUp,
				dist: 0f,
				preserve_aspect: LockAspect,
				commit: true);

			// Set the axis range from the camera position
			SetRangeFromCamera();
			Invalidate();
		}

		/// <summary>Get/Set whether the aspect ratio is locked to the current value</summary>
		[Browsable(false), DesignerSerializationVisibility(DesignerSerializationVisibility.Hidden)]
		public bool LockAspect
		{
			get { return Options.LockAspect != null; }
			set
			{
				if (LockAspect == value) return;
				if (value)
					Options.LockAspect = (XAxis.Span * SceneBounds.Height) / (YAxis.Span * SceneBounds.Width);
				else
					Options.LockAspect = null;
			}
		}

		/// <summary>Set the axis range based on the position of the camera and the field of view</summary>
		public void SetRangeFromCamera()
		{
			// The grid is always parallel to the image plane of the camera.
			// The camera forward vector points at the centre of the grid.

			// Project the camera to world vector into camera space to determine the centre of the X/Y axis. 
			var w2c = Math_.InvertFast(Scene.Camera.O2W);

			// The span of the X/Y axis is determined by the FoV and the focus point distance.
			var wh = Scene.Camera.ViewArea(Scene.Camera.FocusDist);

			// Set the axes range
			var xmin = -w2c.pos.x - wh.x * 0.5;
			var xmax = -w2c.pos.x + wh.x * 0.5;
			var ymin = -w2c.pos.y - wh.y * 0.5;
			var ymax = -w2c.pos.y + wh.y * 0.5;
			if (xmin < xmax) XAxis.Set(xmin, xmax);
			if (ymin < ymax) YAxis.Set(ymin, ymax);
		}

		/// <summary>Position the camera based on the axis range</summary>
		public void SetCameraFromRange()
		{
			// Set the aspect ratio from the axis range and scene bounds
			Scene.Camera.Aspect = (float)(XAxis.Span / YAxis.Span);

			// Find the world space position of the new focus point.
			// Move the focus point within the focus plane (parallel to the camera XY plane)
			// and adjust the focus distance so that the view has a width equal to XAxis.Span.
			// The camera aspect ratio should ensure the YAxis is correct.
			var c2w = Scene.Camera.O2W;
			var focus =
				c2w.x * (float)XAxis.Centre +
				c2w.y * (float)YAxis.Centre +
				c2w.z * Math_.Dot(Scene.Camera.FocusPoint - v4.Origin, c2w.z);

			// Move the camera in the camera Z axis direction so that the width at the focus dist
			// matches the XAxis range. Tan(FovX/2) = (XAxis.Span/2)/d
			Scene.Camera.FocusDist = (float)(XAxis.Span / (2.0 * Math.Tan(Scene.Camera.Fov.x / 2.0)));
			c2w.pos = focus.w1 + Scene.Camera.FocusDist * c2w.z;

			Scene.Camera.O2W = c2w;
			Scene.Camera.Commit();
		}

		/// <summary>Paint the control</summary>
		private void DoPaint(Graphics gfx)
		{
			try
			{
				// Set the projection mode (this must come before 'SetRangeFromCamera' because
				// the 'ViewArea' function depends on the projection mode.
				Scene.Camera.Orthographic = Options.Orthographic;

				// Navigation moves the camera and axis ranges are derived from the camera position/view
				SetRangeFromCamera();

				// Get the areas to draw in
				var dims = ChartDimensions;

				// Draw the chart frame
				m_chart_frame.DoPaint(gfx, dims);

				// Ensure the scene is rendered
				SceneBounds = dims.ChartArea;
				Scene.Update();

				// Add user graphics over the chart
				OnAddOverlaysOnPaint(new AddOverlaysOnPaintEventArgs(gfx, dims, ChartToClientSpace(), ~EZone.None ^ EZone.Chart));
			}
			catch (OverflowException)
			{
				// There is a problem in the .NET graphics object that can cause these exceptions if the range is extreme
				using (var bsh = new SolidBrush(Color.FromArgb(0x80, Color.Black)))
					gfx.DrawString("Rendering error within .NET", Options.TitleFont, bsh, PointF.Empty);
			}
		}
		private ChartFrame m_chart_frame;

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
				elem.PositionZ = HighestZ += 0.001f;
		}

		/// <summary>Return the layout sizes of the chart</summary>
		public ChartDims ChartDimensions
		{
			get { return new ChartDims(this); }
		}

		/// <summary>True if users are allowed to add/remove/edit elements on the diagram</summary>
		public bool AllowEditing
		{
			get;
			set
			{
				if (field == value) return;
				field = value;
				//UpdateEditToolbar();
			}
		}

		/// <summary>True if users are allowed to select elements on the diagram</summary>
		public bool AllowSelection { get; set; }

		/// <summary>Return a string representation of a location on the chart</summary>
		public string LocationText(Point location)
		{
			var pt = ClientToChart(location);
			XAxis.GridLines(out var minx, out var maxx, out var stepx);
			YAxis.GridLines(out var miny, out var maxy, out var stepy);
			var xtick = XAxis.TickText(pt.X, stepx);
			var ytick = YAxis.TickText(pt.Y, stepy);
			return $"{xtick} , {ytick}";
		}

		/// <summary>Return the current mouse pointer location as a string</summary>
		public string PointerLocationText
		{
			get { return LocationText(PointToClient(MousePosition)); }
		}

		#region Chart Frame
		private class ChartFrame
		{
			private readonly ChartControl m_chart;
			private bool m_invalidated;
			private int m_xaxis_hash;
			private int m_yaxis_hash;
			private int m_xaxis_options;
			private int m_yaxis_options;
			private Bitmap m_image;

			public ChartFrame(ChartControl chart)
			{
				m_chart = chart;
				m_image = null;
			}

			/// <summary>Force a repaint of the frame</summary>
			public void Invalidate()
			{
				m_invalidated = true;
			}

			/// <summary>Draw the titles, axis labels, ticks, etc around the chart</summary>
			public void DoPaint(Graphics gfx, ChartDims dims)
			{
				// Drawing the frame can take a while, cache the frame image so if the chart
				// is not moved, we can just blit the cached copy to the screen
				var repaint = true;
				for (;;)
				{
					// Control size changed?
					if (m_image == null || m_image.Size != dims.Area.Size)
					{
						m_image = new Bitmap(dims.Area.Size.Width, dims.Area.Size.Height, gfx);
						break;
					}

					// Manually invalidated?
					if (m_invalidated)
						break;

					// Axes zoomed/scrolled?
					if (m_chart.XAxis.GetHashCode() != m_xaxis_hash)
						break;
					if (m_chart.YAxis.GetHashCode() != m_yaxis_hash)
						break;

					// Rendering options changed?
					if (m_chart.XAxis.Options.ChangeCounter != m_xaxis_options)
						break;
					if (m_chart.YAxis.Options.ChangeCounter != m_yaxis_options)
						break;

					// Cached copy is still good
					repaint = false;
					break;
				}

				// Repaint the frame if the cached version is out of date
				if (repaint)
				{
					var bmp = Graphics.FromImage(m_image);
					var opts = m_chart.Options;
					var xaxis = m_chart.XAxis;
					var yaxis = m_chart.YAxis;
					var title = m_chart.Title;
					var size = dims.Area.Size;
					var chart_area = dims.ChartArea.Shifted(-dims.Area.Left, -dims.Area.Top);

					// This is not enforced in the axis.Min/Max accessors because it's useful
					// to be able to change the min/max independently of each other, set them
					// to float max etc. It's only invalid to render a chart with a negative range
					Debug.Assert(xaxis.Span > 0, "Negative x range");
					Debug.Assert(yaxis.Span > 0, "Negative y range");

					// Clear to the background colour
					bmp.Clear(opts.BkColour);

					// Draw the chart title and labels
					if (title.HasValue())
					{
						using (var bsh = new SolidBrush(opts.TitleColour))
						{
							var r = dims.TitleSize;
							var x = (size.Width - r.Width) * 0.5f;
							var y = (0 + opts.Margin.Top) * 1f;
							bmp.TranslateTransform(x, y);
							bmp.MultiplyTransform(opts.TitleTransform);
							bmp.DrawString(title, opts.TitleFont, bsh, PointF.Empty);
							bmp.ResetTransform();
						}
					}
					if (xaxis.Label.HasValue() && opts.ShowAxes)
					{
						using (var bsh = new SolidBrush(xaxis.Options.LabelColour))
						{
							var r = dims.XLabelSize;
							var x = (size.Width - r.Width) * 0.5f;
							var y = (size.Height - opts.Margin.Bottom - r.Height) * 1f;
							bmp.TranslateTransform(x, y);
							bmp.MultiplyTransform(xaxis.Options.LabelTransform);
							bmp.DrawString(xaxis.Label, xaxis.Options.LabelFont, bsh, PointF.Empty);
							bmp.ResetTransform();
						}
					}
					if (yaxis.Label.HasValue() && opts.ShowAxes)
					{
						using (var bsh = new SolidBrush(yaxis.Options.LabelColour))
						{
							var r = dims.YLabelSize;
							var x = (0 + opts.Margin.Left) * 1f;
							var y = (size.Height + r.Width) * 0.5f;
							bmp.TranslateTransform(x, y);
							bmp.RotateTransform(-90.0f);
							bmp.MultiplyTransform(yaxis.Options.LabelTransform);
							bmp.DrawString(yaxis.Label, yaxis.Options.LabelFont, bsh, PointF.Empty);
							bmp.ResetTransform();
						}
					}

					// Tick marks and labels
					if (opts.ShowAxes)
					{
						if (xaxis.Options.DrawTickLabels || xaxis.Options.DrawTickMarks)
						{
							using (var pen = new Pen(xaxis.Options.TickColour))
							using (var bsh = new SolidBrush(xaxis.Options.TickColour))
							{
								var lbly = (float)(chart_area.Top + chart_area.Height + xaxis.Options.TickLength + 1);
								xaxis.GridLines(out var min, out var max, out var step);
								for (var x = min; x < max; x += step)
								{
									var X = (int)(chart_area.Left + x * chart_area.Width / xaxis.Span);
									if (xaxis.Options.DrawTickLabels)
									{
										var s = xaxis.TickText(x + xaxis.Min, step);
										bmp.DrawString(s, xaxis.Options.TickFont, bsh, new PointF(X, lbly), new StringFormat{Alignment = StringAlignment.Center});
									}
									if (xaxis.Options.DrawTickMarks)
									{
										bmp.DrawLine(pen, X, chart_area.Top + chart_area.Height, X, chart_area.Top + chart_area.Height + xaxis.Options.TickLength);
									}
								}
							}
						}
						if (yaxis.Options.DrawTickLabels || yaxis.Options.DrawTickMarks)
						{
							using (var pen = new Pen(yaxis.Options.TickColour))
							using (var bsh = new SolidBrush(yaxis.Options.TickColour))
							{
								var lblx = (float)(chart_area.Left - yaxis.Options.TickLength - 1);
								yaxis.GridLines(out var min, out var max, out var step);
								for (var y = min; y < max; y += step)
								{
									var Y = (int)(chart_area.Top + chart_area.Height - y * chart_area.Height / yaxis.Span);
									if (yaxis.Options.DrawTickLabels)
									{
										var s = yaxis.TickText(y + yaxis.Min, step);
										bmp.DrawString(s, yaxis.Options.TickFont, bsh,
											new RectangleF(lblx - dims.YTickLabelSize.Width, Y - dims.YTickLabelSize.Height*0.5f, dims.YTickLabelSize.Width, dims.YTickLabelSize.Height),
											new StringFormat{Alignment = StringAlignment.Far});
									}
									if (yaxis.Options.DrawTickMarks)
									{
										bmp.DrawLine(pen, chart_area.Left - yaxis.Options.TickLength, Y, chart_area.Left, Y);
									}
								}
							}
						}

						// Axes
						using (var pen = new Pen(xaxis.Options.AxisColour, xaxis.Options.AxisThickness))
						{
							var y = chart_area.Bottom;
							bmp.DrawLine(pen, new Point(chart_area.Left, y), new Point(chart_area.Right, y));
						}
						using (var pen = new Pen(yaxis.Options.AxisColour, yaxis.Options.AxisThickness))
						{
							var x = (int)(chart_area.Left - xaxis.Options.AxisThickness*0.5f);
							bmp.DrawLine(pen, new Point(x, chart_area.Top), new Point(x, chart_area.Bottom));
						}

						// Record cache invalidating values
						m_xaxis_hash = xaxis.GetHashCode();
						m_yaxis_hash = yaxis.GetHashCode();
						m_xaxis_options = xaxis.Options.ChangeCounter;
						m_yaxis_options = yaxis.Options.ChangeCounter;
						m_invalidated = false;
					}
				}

				// Blit the cached image to the screen
				gfx.DrawImageUnscaled(m_image, dims.Area.TopLeft());
			}
		}
		#endregion

		#region Chart Panel
		public class ChartPanel :Control
		{
			private ChartControl m_owner;         // The containing chart control
			private bool         m_render_needed; // True when the scene needs rendering again

			public ChartPanel(ChartControl owner)
			{
				SetStyle(ControlStyles.Selectable, false);
				if (this.IsInDesignMode())
					return;

				try
				{
					// No GDI compatible back buffer, cause we want anti aliasing...
					m_owner = owner;
					View3d = View3d.Create();
					Window = new View3d.Window(View3d, Handle, gdi_compatible_backbuffer:false, multi_sampling: owner.Options.AntiAliasing ? 4 : 1, dbg_name:"Chart")
					{
						LightProperties = View3d.LightInfo.Directional(-v4.ZAxis, Colour32.Zero, Colour32.Gray, Colour32.Zero, 0f, 0f),
						FocusPointVisible = false,
						OriginPointVisible = false,
					};
					Camera.Orthographic = true;
					Camera.Lookat(new v4(0, 0, 10, 1), v4.Origin, v4.YAxis);
					Camera.ClipPlanes(0.01f, 1000f, View3d.EClipPlanes.Both | View3d.EClipPlanes.CameraRelative);
				}
				catch
				{
					Dispose();
					throw;
				}
			}
			protected override void Dispose(bool disposing)
			{
				Util.Dispose(Window);
				Util.Dispose(View3d);
				base.Dispose(disposing);
			}
			protected override void OnResize(EventArgs e)
			{
				base.OnResize(e);
				if (Window != null)
					Window.BackBufferSize = ClientSize;
			}
			protected override void OnPaintBackground(PaintEventArgs e)
			{
				// Swallow
			}
			protected override void OnPaint(PaintEventArgs e)
			{
				base.OnPaint(e);
				if (DesignMode) return;
				if (m_render_needed) DoPaint();
				Present();

				// Add user graphics over the chart. Use a transform to make the
				// chart area appear in the same space as the non-chart area.
				using (e.Graphics.SaveState())
				{
					var dims = m_owner.ChartDimensions;
					e.Graphics.TranslateTransform(-dims.ChartArea.Left, -dims.ChartArea.Top);
					m_owner.OnAddOverlaysOnPaint(new AddOverlaysOnPaintEventArgs(e.Graphics, dims, m_owner.ChartToClientSpace(), EZone.Chart));
				}
			}
			protected override void OnInvalidated(InvalidateEventArgs e)
			{
				m_render_needed = true;
				Window?.Invalidate();
				base.OnInvalidated(e);
			}
			protected override void WndProc(ref Message m)
			{
				switch (m.Msg)
				{
				case Win32.WM_NCHITTEST:
					// Transparent to input events, let the owner control handle them
					m.Result = (IntPtr)Win32.HitTest.HTTRANSPARENT;
					return;
				}
				base.WndProc(ref m);
			}

			/// <summary>Renderer</summary>
			public View3d View3d { get; }

			/// <summary>The view3d window for this control instance</summary>
			public View3d.Window Window { get; }

			/// <summary>The view of the chart</summary>
			public View3d.Camera Camera => Window.Camera;

			/// <summary>Add an object to the scene</summary>
			public void AddObject(View3d.Object obj)
			{
				Window.AddObject(obj);
			}

			/// <summary>Remove an object from the scene</summary>
			public void RemoveObject(View3d.Object obj)
			{
				Window.RemoveObject(obj);
			}

			/// <summary>Render the chart 3d scene</summary>
			public void DoPaint()
			{
				if (Window == null || this.IsInDesignMode())
					return;

				// Update axis graphics
				m_owner.Range.AddToScene(Window);

				// Add/Remove all chart elements
				foreach (var elem in m_owner.Elements)
					elem.UpdateScene(Window);

				// Add/Remove user graphics
				m_owner.OnChartRendering(new ChartRenderingEventArgs(m_owner, Window));

				// Start the render
				Window.BackgroundColour = m_owner.Options.ChartBkColour;
				Window.FillMode = m_owner.Options.FillMode;
				Window.CullMode = m_owner.Options.CullMode;
				Window.Render();
			}
			public void Present()
			{
				//Window?.Present();
			}

			/// <summary>Returns a point in chart space from a point in ChartPanel-client space.</summary>
			private PointF ClientToChart(Point point)
			{
				return m_owner.ClientToChart(Control_.MapPoint(this, m_owner, point));
			}

			/// <summary>Returns a point in ChartPanel-client space from a point in chart space. Inverse of ClientToChart</summary>
			private Point ChartToClient(PointF point)
			{
				return Control_.MapPoint(m_owner, this, m_owner.ChartToClient(point).ToPoint());
			}
		}
		#endregion

		#region Chart Dims

		/// <summary>The calculated areas of the control</summary>
		[DebuggerDisplay("Area={Area}  ChartArea={ChartArea}")]
		public struct ChartDims
		{
			public ChartDims(ChartControl chart)
			{
				Chart = chart;
				using (var gfx = chart.CreateGraphics())
				{
					RectangleF rect = chart.ClientRectangle;
					var r = SizeF.Empty;

					Area = rect.ToRect();
					TitleSize = SizeF.Empty;
					XLabelSize = SizeF.Empty;
					YLabelSize = SizeF.Empty;
					XTickLabelSize = SizeF.Empty;
					YTickLabelSize = SizeF.Empty;

					// Add margins
					rect.X      += chart.Options.Margin.Left;
					rect.Y      += chart.Options.Margin.Top;
					rect.Width  -= chart.Options.Margin.Left + chart.Options.Margin.Right;
					rect.Height -= chart.Options.Margin.Top + chart.Options.Margin.Bottom;

					// Add space for the title
					if (chart.Title.HasValue())
					{
						r = gfx.MeasureString(chart.Title, chart.Options.TitleFont);
						rect.Y      += r.Height;
						rect.Height -= r.Height;
						TitleSize = r;
					}

					// Add space for the axes
					if (chart.Options.ShowAxes)
					{
						// Add space for tick marks
						if (chart.YAxis.Options.DrawTickMarks)
						{
							rect.X      += chart.YAxis.Options.TickLength;
							rect.Width  -= chart.YAxis.Options.TickLength;
						}
						if (chart.XAxis.Options.DrawTickMarks)
						{
							rect.Height -= chart.XAxis.Options.TickLength;
						}

						// Add space for the axis labels
						if (chart.XAxis.Label.HasValue())
						{
							r = gfx.MeasureString(chart.XAxis.Label, chart.XAxis.Options.LabelFont);
							rect.Height -= r.Height;
							XLabelSize = r;
						}
						if (chart.YAxis.Label.HasValue())
						{
							r = gfx.MeasureString(chart.YAxis.Label, chart.YAxis.Options.LabelFont);
							rect.X     += r.Height; // will be rotated by 90deg
							rect.Width -= r.Height;
							YLabelSize = r;
						}

						// Add space for the tick labels
						// Note: If you're having trouble with the axis jumping around
						// check the 'TickText' callback is returning fixed length strings
						if (chart.XAxis.Options.DrawTickLabels)
						{
							// Measure the height of the tick text
							r = chart.XAxis.MeasureTickText(gfx);
							rect.Height -= r.Height;
							XTickLabelSize = r;
						}
						if (chart.YAxis.Options.DrawTickLabels)
						{
							// Measure the width of the tick text
							r = chart.YAxis.MeasureTickText(gfx);
							rect.X     += r.Width;
							rect.Width -= r.Width;
							YTickLabelSize = r;
						}
					}

					if (rect.Width < 0) rect.Width = 0;
					if (rect.Height < 0) rect.Height = 0;
					ChartArea = rect.ToRect();
				}
			}

			/// <summary>The chart that these dimensions were calculated from</summary>
			public ChartControl Chart { get; private set; }

			/// <summary>The size of the control</summary>
			public Rectangle Area { get; private set; }

			/// <summary>The area of the view3d part of the chart</summary>
			public Rectangle ChartArea { get; private set; }

			/// <summary>The measured size of the chart title</summary>
			public SizeF TitleSize { get; private set; }

			/// <summary>The measured size of the X axis label</summary>
			public SizeF XLabelSize { get; private set; }

			/// <summary>The measured size of the Y axis label</summary>
			public SizeF YLabelSize { get; private set; }

			/// <summary>The measured size of the X axis tick labels</summary>
			public SizeF XTickLabelSize { get; private set; }

			/// <summary>The measured size of the Y axis tick labels</summary>
			public SizeF YTickLabelSize { get; private set; }
		}

		/// <summary>The client space area of the XAxis area of the chart</summary>
		private Rectangle XAxisBounds
		{
			get { return Rectangle.FromLTRB(SceneBounds.Left, SceneBounds.Bottom, SceneBounds.Right, ClientSize.Height); }
		}

		/// <summary>The client space area of the YAxis area of the chart</summary>
		private Rectangle YAxisBounds
		{
			get { return Rectangle.FromLTRB(0, SceneBounds.Top, SceneBounds.Left, SceneBounds.Bottom); }
		}

		/// <summary>The client space area of the chart title</summary>
		private Rectangle TitleBounds
		{
			get { return Rectangle.FromLTRB(SceneBounds.Left, 0, SceneBounds.Right, SceneBounds.Top); }
		}

		#endregion

		#region RdrOptions
		[TypeConverter(typeof(TyConv))]
		public class RdrOptions :INotifyPropertyChanged
		{
			private class TyConv :GenericTypeConverter<RdrOptions> {}

			public RdrOptions()
			{
				NavigationMode            = ENavMode.Chart2D;
				LockAspect                = null;
				BkColour                  = SystemColors.Control;
				ChartBkColour             = Color.White;
				TitleColour               = Color.Black;
				TitleFont                 = new Font("tahoma", 12, FontStyle.Bold);
				TitleTransform            = new Matrix(1f, 0f, 0f, 1f, 0f, 0f);
				Margin                    = new Padding(3);
				NoteFont                  = new Font("tahoma", 8, FontStyle.Regular);
				SelectionColour           = Color.FromArgb(0x80, Color.DarkGray);
				ShowGridLines             = true;
				GridZOffset               = 0.001f;
				ShowAxes                  = true;
				AntiAliasing              = true;
				FillMode                  = View3d.EFillMode.Solid;
				CullMode                  = View3d.ECullMode.Back;
				Orthographic              = false;
				MinSelectionDistance      = 10f;
				MinDragPixelDistance      = 5f;
				PerpendicularZTranslation = false;
				ResetForward              = -v4.ZAxis;
				ResetUp                   = +v4.YAxis;
				XAxis                     = new Axis();
				YAxis                     = new Axis();
				// Don't forget to add new members to the other constructors!
			}
			public RdrOptions(RdrOptions rhs)
			{
				NavigationMode            = rhs.NavigationMode;
				LockAspect                = rhs.LockAspect;
				BkColour                  = rhs.BkColour;
				ChartBkColour             = rhs.ChartBkColour;
				TitleColour               = rhs.TitleColour;
				TitleFont                 = (Font)rhs.TitleFont.Clone();
				TitleTransform            = rhs.TitleTransform;
				Margin                    = rhs.Margin;
				NoteFont                  = (Font)rhs.NoteFont.Clone();
				SelectionColour           = rhs.SelectionColour;
				ShowGridLines             = rhs.ShowGridLines;
				GridZOffset               = rhs.GridZOffset;
				ShowAxes                  = rhs.ShowAxes;
				AntiAliasing              = rhs.AntiAliasing;
				FillMode                  = rhs.FillMode;
				CullMode                  = rhs.CullMode;
				Orthographic              = rhs.Orthographic;
				MinSelectionDistance      = rhs.MinSelectionDistance;
				MinDragPixelDistance      = rhs.MinDragPixelDistance;
				PerpendicularZTranslation = rhs.PerpendicularZTranslation;
				ResetForward              = rhs.ResetForward;
				ResetUp                   = rhs.ResetUp;
				XAxis                     = new Axis(rhs.XAxis);
				YAxis                     = new Axis(rhs.YAxis);
			}
			public RdrOptions(XElement node) :this()
			{
				NavigationMode            = node.Element(nameof(NavigationMode           )).As<ENavMode>(NavigationMode       );
				LockAspect                = node.Element(nameof(LockAspect               )).As<double?>(LockAspect            );
				BkColour                  = node.Element(nameof(BkColour                 )).As<Color>(BkColour                );
				ChartBkColour             = node.Element(nameof(ChartBkColour            )).As<Color>(ChartBkColour           );
				TitleColour               = node.Element(nameof(TitleColour              )).As<Color>(TitleColour             );
				TitleTransform            = node.Element(nameof(TitleTransform           )).As<Matrix>(TitleTransform         );
				Margin                    = node.Element(nameof(Margin                   )).As<Padding>(Margin                );
				TitleFont                 = node.Element(nameof(TitleFont                )).As<Font>(TitleFont                );
				NoteFont                  = node.Element(nameof(NoteFont                 )).As<Font>(NoteFont                 );
				SelectionColour           = node.Element(nameof(SelectionColour          )).As<Color>(SelectionColour         );
				ShowAxes                  = node.Element(nameof(ShowAxes                 )).As<bool>(ShowAxes                 );
				ShowGridLines             = node.Element(nameof(ShowGridLines            )).As<bool>(ShowGridLines            );
				GridZOffset               = node.Element(nameof(GridZOffset              )).As<float>(GridZOffset             );
				AntiAliasing              = node.Element(nameof(AntiAliasing             )).As<bool>(AntiAliasing             );
				FillMode                  = node.Element(nameof(FillMode                 )).As<View3d.EFillMode>(FillMode     );
				CullMode                  = node.Element(nameof(CullMode                 )).As<View3d.ECullMode>(CullMode     );
				Orthographic              = node.Element(nameof(Orthographic             )).As<bool>(Orthographic             );
				MinSelectionDistance      = node.Element(nameof(MinSelectionDistance     )).As<float>(MinSelectionDistance    );
				MinDragPixelDistance      = node.Element(nameof(MinDragPixelDistance     )).As<float>(MinDragPixelDistance    );
				PerpendicularZTranslation = node.Element(nameof(PerpendicularZTranslation)).As<bool>(PerpendicularZTranslation);
				ResetForward              = node.Element(nameof(ResetForward             )).As<v4>(ResetForward               );
				ResetUp                   = node.Element(nameof(ResetUp                  )).As<v4>(ResetUp                    );
				XAxis                     = node.Element(nameof(XAxis                    )).As<Axis>(XAxis                    );
				YAxis                     = node.Element(nameof(YAxis                    )).As<Axis>(YAxis                    );
			}
			public XElement ToXml(XElement node)
			{
				node.Add2(nameof(NavigationMode           ) , NavigationMode            , false);
				node.Add2(nameof(LockAspect               ) , LockAspect                , false);
				node.Add2(nameof(BkColour                 ) , BkColour                  , false);
				node.Add2(nameof(ChartBkColour            ) , ChartBkColour             , false);
				node.Add2(nameof(TitleColour              ) , TitleColour               , false);
				node.Add2(nameof(TitleTransform           ) , TitleTransform            , false);
				node.Add2(nameof(Margin                   ) , Margin                    , false);
				node.Add2(nameof(TitleFont                ) , TitleFont                 , false);
				node.Add2(nameof(NoteFont                 ) , NoteFont                  , false);
				node.Add2(nameof(SelectionColour          ) , SelectionColour           , false);
				node.Add2(nameof(ShowGridLines            ) , ShowGridLines             , false);
				node.Add2(nameof(GridZOffset              ) , GridZOffset               , false);
				node.Add2(nameof(ShowAxes                 ) , ShowAxes                  , false);
				node.Add2(nameof(AntiAliasing             ) , AntiAliasing              , false);
				node.Add2(nameof(FillMode                 ) , FillMode                  , false);
				node.Add2(nameof(CullMode                 ) , CullMode                  , false);
				node.Add2(nameof(Orthographic             ) , Orthographic              , false);
				node.Add2(nameof(MinSelectionDistance     ) , MinSelectionDistance      , false);
				node.Add2(nameof(MinDragPixelDistance     ) , MinDragPixelDistance      , false);
				node.Add2(nameof(PerpendicularZTranslation) , PerpendicularZTranslation , false);
				node.Add2(nameof(ResetForward             ) , ResetForward              , false);
				node.Add2(nameof(ResetUp                  ) , ResetUp                   , false);
				node.Add2(nameof(XAxis                    ) , XAxis                     , false);
				node.Add2(nameof(YAxis                    ) , YAxis                     , false);
				return node;
			}
			public override string ToString()
			{
				return "Rendering Options";
			}

			/// <summary>Property changed</summary>
			public event PropertyChangedEventHandler PropertyChanged;
			private void SetProp<T>(ref T prop, T value, string name)
			{
				if (Equals(prop, value)) return;
				prop = value;
				PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(name));
			}

			/// <summary>The control method used for shifting the camera</summary>
			public ENavMode NavigationMode
			{
				get;
				set { SetProp(ref field, value, nameof(NavigationMode)); }
			}

			/// <summary>Lock the aspect ratio for the chart (null means unlocked)</summary>
			public double? LockAspect
			{
				get;
				set { SetProp(ref field, value, nameof(LockAspect)); }
			}

			/// <summary>The fill colour of the non-chart area of the control (e.g. behind the axis labels)</summary>
			public Color BkColour
			{
				get;
				set { SetProp(ref field, value, nameof(BkColour)); }
			}

			/// <summary>The fill colour of the chart plot area</summary>
			public Color ChartBkColour
			{
				get;
				set { SetProp(ref field, value, nameof(ChartBkColour)); }
			}

			/// <summary>The colour of the title text</summary>
			public Color TitleColour
			{
				get;
				set { SetProp(ref field, value, nameof(TitleColour)); }
			}

			/// <summary>Transform for position the chart title, offset from top centre</summary>
			public Matrix TitleTransform
			{
				get;
				set { SetProp(ref field, value, nameof(TitleTransform)); }
			}

			/// <summary>The distances from the edge of the control to the chart area</summary>
			public Padding Margin
			{
				get;
				set { SetProp(ref field, value, nameof(Margin)); }
			}

			/// <summary>Font to use for the title text</summary>
			public Font TitleFont
			{
				get;
				set { SetProp(ref field, value, nameof(TitleFont)); }
			}

			/// <summary>Font to use for chart notes</summary>
			public Font NoteFont
			{
				get;
				set { SetProp(ref field, value, nameof(NoteFont)); }
			}

			/// <summary>Area selection colour</summary>
			public Color SelectionColour
			{
				get;
				set { SetProp(ref field, value, nameof(SelectionColour)); }
			}

			/// <summary>Show grid lines (False overrides per-axis options)</summary>
			public bool ShowGridLines
			{
				get { return m_ShowGridLines; }
				set { SetProp(ref m_ShowGridLines, value, nameof(ShowGridLines)); }
			}
			private bool m_ShowGridLines;

			/// <summary>The offset from the origin for the grid, in the forward direction of the camera</summary>
			public float GridZOffset
			{
				get;
				set { SetProp(ref field, value, nameof(GridZOffset)); }
			}

			/// <summary>Show/Hide the chart axes</summary>
			public bool ShowAxes
			{
				get;
				set { SetProp(ref field, value, nameof(ShowAxes)); }
			}

			/// <summary>Enable/Disable multi-sampling in the view3d view. Can only be changed before the view is created</summary>
			public bool AntiAliasing
			{
				get;
				set { SetProp(ref field, value, nameof(AntiAliasing)); }
			}

			/// <summary>Fill mode, solid, wire, or both</summary>
			public View3d.EFillMode FillMode
			{
				get;
				set { SetProp(ref field, value, nameof(FillMode)); }
			}

			/// <summary>Fill mode, solid, wire, or both</summary>
			public View3d.ECullMode CullMode
			{
				get;
				set { SetProp(ref field, value, nameof(CullMode)); }
			}

			/// <summary>Get/Set orthographic camera projection</summary>
			public bool Orthographic
			{
				get;
				set { SetProp(ref field, value, nameof(Orthographic)); }
			}

			/// <summary>How close a click has to be for selection to occur (in client space)</summary>
			public float MinSelectionDistance
			{
				get;
				set { SetProp(ref field, value, nameof(MinSelectionDistance)); }
			}

			/// <summary>Minimum distance in pixels before the chart starts dragging</summary>
			public float MinDragPixelDistance
			{
				get;
				set { SetProp(ref field, value, nameof(MinDragPixelDistance)); }
			}

			/// <summary>True if the camera should move along a ray cast through the mouse point</summary>
			public bool PerpendicularZTranslation
			{
				get;
				set { SetProp(ref field, value, nameof(PerpendicularZTranslation)); }
			}

			/// <summary>The forward direction of the camera when reset</summary>
			public v4 ResetForward
			{
				get;
				set { SetProp(ref field, value, nameof(ResetForward)); }
			}

			/// <summary>The up direction of the camera when reset</summary>
			public v4 ResetUp
			{
				get;
				set { SetProp(ref field, value, nameof(ResetUp)); }
			}

			/// <summary>XAxis rendering options</summary>
			public Axis XAxis 
			{
				get;
				private set
				{
					if (field == value) return;
					if (field != null) field.PropertyChanged -= HandleXAxisPropertyChanged;
					field = value;
					if (field != null) field.PropertyChanged += HandleXAxisPropertyChanged;

					// Handler
					void HandleXAxisPropertyChanged(object sender, PropertyChangedEventArgs e)
					{
						PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(nameof(XAxis)));
					}
				}
			}

			/// <summary>YAxis rendering options</summary>
			public Axis YAxis
			{
				get;
				private set
				{
					if (field == value) return;
					if (field != null) field.PropertyChanged -= HandleYAxisPropertyChanged;
					field = value;
					if (field != null) field.PropertyChanged += HandleYAxisPropertyChanged;

					// Handler
					void HandleYAxisPropertyChanged(object sender, PropertyChangedEventArgs e)
					{
						PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(nameof(YAxis)));
					}
				}
			}

			[TypeConverter(typeof(TyConv))]
			public class Axis :INotifyPropertyChanged
			{
				private class TyConv :GenericTypeConverter<Axis> {}

				public Axis()
				{
					AxisColour     = Color.Black;
					LabelColour    = Color.Black;
					GridColour     = Color.WhiteSmoke;
					TickColour     = Color.Black;
					LabelFont      = new Font("tahoma", 10, FontStyle.Regular);
					TickFont       = new Font("tahoma", 8, FontStyle.Regular);
					DrawTickMarks  = true;
					DrawTickLabels = true;
					TickLength     = 5;
					MinTickSize    = 30;
					LabelTransform = new Matrix(1f, 0f, 0f, 1f, 0f, 0f);
					AxisThickness  = 1f;
					PixelsPerTick  = 30.0;
					ShowGridLines  = true;
				}
				public Axis(Axis rhs)
				{
					AxisColour     = rhs.AxisColour;
					LabelColour    = rhs.LabelColour;
					GridColour     = rhs.GridColour;
					TickColour     = rhs.TickColour;
					LabelFont      = (Font)rhs.LabelFont.Clone();
					TickFont       = (Font)rhs.TickFont.Clone();
					DrawTickMarks  = rhs.DrawTickMarks;
					DrawTickLabels = rhs.DrawTickLabels;
					TickLength     = rhs.TickLength;
					MinTickSize    = rhs.MinTickSize;
					LabelTransform = rhs.LabelTransform;
					AxisThickness  = rhs.AxisThickness;
					PixelsPerTick  = rhs.PixelsPerTick;
					ShowGridLines  = rhs.ShowGridLines;
				}
				public Axis(XElement node) :this()
				{
					AxisColour     = node.Element(nameof(AxisColour    )).As<Color>(AxisColour    );
					LabelColour    = node.Element(nameof(LabelColour   )).As<Color>(LabelColour   );
					GridColour     = node.Element(nameof(GridColour    )).As<Color>(GridColour    );
					TickColour     = node.Element(nameof(TickColour    )).As<Color>(TickColour    );
					LabelFont      = node.Element(nameof(LabelFont     )).As<Font>(LabelFont     );
					TickFont       = node.Element(nameof(TickFont      )).As<Font>(TickFont      );
					DrawTickMarks  = node.Element(nameof(DrawTickMarks )).As<bool>(DrawTickMarks );
					DrawTickLabels = node.Element(nameof(DrawTickLabels)).As<bool>(DrawTickLabels);
					TickLength     = node.Element(nameof(TickLength    )).As<int>(TickLength    );
					MinTickSize    = node.Element(nameof(MinTickSize   )).As<float>(MinTickSize   );
					LabelTransform = node.Element(nameof(LabelTransform)).As<Matrix>(LabelTransform);
					AxisThickness  = node.Element(nameof(AxisThickness )).As<float>(AxisThickness );
					PixelsPerTick  = node.Element(nameof(PixelsPerTick )).As<double>(PixelsPerTick );
					ShowGridLines  = node.Element(nameof(ShowGridLines )).As<bool>(ShowGridLines );
				}
				public XElement ToXml(XElement node)
				{
					node.Add2(nameof(AxisColour    ), AxisColour    , false);
					node.Add2(nameof(LabelColour   ), LabelColour   , false);
					node.Add2(nameof(GridColour    ), GridColour    , false);
					node.Add2(nameof(TickColour    ), TickColour    , false);
					node.Add2(nameof(LabelFont     ), LabelFont     , false);
					node.Add2(nameof(TickFont      ), TickFont      , false);
					node.Add2(nameof(DrawTickMarks ), DrawTickMarks , false);
					node.Add2(nameof(DrawTickLabels), DrawTickLabels, false);
					node.Add2(nameof(TickLength    ), TickLength    , false);
					node.Add2(nameof(MinTickSize   ), MinTickSize   , false);
					node.Add2(nameof(LabelTransform), LabelTransform, false);
					node.Add2(nameof(AxisThickness ), AxisThickness , false);
					node.Add2(nameof(PixelsPerTick ), PixelsPerTick , false);
					node.Add2(nameof(ShowGridLines ), ShowGridLines , false);
					return node;
				}
				public override string ToString()
				{
					return "Axis Options";
				}

				/// <summary>Property changed</summary>
				public event PropertyChangedEventHandler PropertyChanged;
				private void SetProp<T>(ref T prop, T value, string name)
				{
					if (Equals(prop, value)) return;
					prop = value;
					unchecked {++ChangeCounter;}
					PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(name));
				}
				public int ChangeCounter { get; private set; }

				/// <summary>The colour of the main axes</summary>
				public Color AxisColour
				{
					get;
					set { SetProp(ref field, value, nameof(AxisColour)); }
				}

				/// <summary>The colour of the label text</summary>
				public Color LabelColour
				{
					get;
					set { SetProp(ref field, value, nameof(LabelColour)); }
				}

				/// <summary>The colour of the grid lines</summary>
				public Color GridColour
				{
					get;
					set { SetProp(ref field, value, nameof(GridColour)); }
				}

				/// <summary>The colour of the tick text</summary>
				public Color TickColour
				{
					get;
					set { SetProp(ref field, value, nameof(TickColour)); }
				}

				/// <summary>The font to use for the axis label</summary>
				public Font LabelFont
				{
					get;
					set { SetProp(ref field, value, nameof(LabelFont)); }
				}

				/// <summary>The font to use for tick labels</summary>
				public Font TickFont
				{
					get;
					set { SetProp(ref field, value, nameof(TickFont)); }
				}

				/// <summary>True if tick marks should be drawn</summary>
				public bool DrawTickMarks
				{
					get;
					set { SetProp(ref field, value, nameof(DrawTickMarks)); }
				}

				/// <summary>True if tick labels should be drawn</summary>
				public bool DrawTickLabels
				{
					get;
					set { SetProp(ref field, value, nameof(DrawTickLabels)); }
				}

				/// <summary>The length of the tick marks</summary>
				public int TickLength
				{
					get;
					set { SetProp(ref field, value, nameof(TickLength)); }
				}

				/// <summary>The minimum space reserved for tick marks and labels</summary>
				public float MinTickSize
				{
					get;
					set { SetProp(ref field, value, nameof(MinTickSize)); }
				}

				/// <summary>Offset transform from default label position</summary>
				public Matrix LabelTransform
				{
					get;
					set { SetProp(ref field, value, nameof(LabelTransform)); }
				}

				/// <summary>The thickness of the axis line</summary>
				public float AxisThickness
				{
					get;
					set { SetProp(ref field, value, nameof(AxisThickness)); }
				}

				/// <summary>The preferred number of pixels between each grid line</summary>
				public double PixelsPerTick
				{
					get;
					set { SetProp(ref field, value, nameof(PixelsPerTick)); }
				}

				/// <summary>Show grid lines for this axis. (This settings is overruled by the main chart options)</summary>
				public bool ShowGridLines
				{
					get { return m_ShowGridLines; }
					set { SetProp(ref m_ShowGridLines, value, nameof(ShowGridLines)); }
				}
				private bool m_ShowGridLines;
			}
		}

		/// <summary>A UI for setting these rendering properties</summary>
		public class RdrOptionsUI :ToolForm
		{
			private readonly ChartControl m_chart;
			private readonly RdrOptions m_opts;

			public RdrOptionsUI(ChartControl chart, RdrOptions opts)
				:base(chart, EPin.Centre, Point.Empty, new Size(500,400), true)
			{
				m_chart  = chart;
				m_opts   = opts;
				ShowIcon = (chart.TopLevelControl as Form)?.ShowIcon ?? false;
				Icon     = (chart.TopLevelControl as Form)?.Icon;
				Text     = "Chart Properties";
				SetupUI();
			}
			private void SetupUI()
			{
				var pg = Controls.Add2(new PropertyGrid
				{
					SelectedObject = m_opts,
					Dock = DockStyle.Fill,
				});
				pg.PropertyValueChanged += (s,a) =>
				{
					m_chart.Invalidate();
					m_chart.XAxis.GridLineGfx = null;
					m_chart.YAxis.GridLineGfx = null;
				};
			}
		}
		#endregion

		#region Range

		/// <summary>The 2D size of the chart</summary>
		public sealed class RangeData :IDisposable
		{
			public RangeData(ChartControl owner)
			{
				Owner = owner;
				XAxis = new Axis(EAxis.XAxis, owner);
				YAxis = new Axis(EAxis.YAxis, owner);
			}
			public RangeData(RangeData rhs)
			{
				XAxis = new Axis(rhs.XAxis);
				YAxis = new Axis(rhs.YAxis);
			}
			public void Dispose()
			{
				XAxis = null;
				YAxis = null;
			}

			/// <summary>The chart that owns this axis</summary>
			public ChartControl Owner { [DebuggerStepThrough] get; private set; }

			/// <summary>The chart X axis</summary>
			public Axis XAxis
			{
				[DebuggerStepThrough] get { return m_xaxis; }
				internal set
				{
					if (m_xaxis == value) return;
					if (m_xaxis != null)
					{
						m_xaxis.Scroll -= HandleAxisScrolled;
						m_xaxis.Zoomed -= HandleAxisZoomed;
						Util.Dispose(ref m_xaxis);
					}
					m_xaxis = value;
					if (m_xaxis != null)
					{
						m_xaxis.Zoomed += HandleAxisZoomed;
						m_xaxis.Scroll += HandleAxisScrolled;
					}
				}
			}
			private Axis m_xaxis;

			/// <summary>The chart Y axis</summary>
			public Axis YAxis
			{
				[DebuggerStepThrough] get { return m_yaxis; }
				internal set
				{
					if (m_yaxis == value) return;
					if (m_yaxis != null)
					{
						m_yaxis.Scroll -= HandleAxisScrolled;
						m_yaxis.Zoomed -= HandleAxisZoomed;
						Util.Dispose(ref m_yaxis);
					}
					m_yaxis = value;
					if (m_yaxis != null)
					{
						m_yaxis.Zoomed += HandleAxisZoomed;
						m_yaxis.Scroll += HandleAxisScrolled;
					}
				}
			}
			private Axis m_yaxis;

			/// <summary>Add the graphics associated with the axes to the scene</summary>
			internal void AddToScene(View3d.Window window)
			{
				var cam = Owner.Scene.Camera;
				var wh = cam.ViewArea(Owner.Scene.Camera.FocusDist);
					
				// Position the grid lines so that they line up with the axis tick marks
				// Grid lines are modelled from the bottom left corner
				if (XAxis.GridLineGfx != null)
				{
					if (XAxis.Owner.Options.ShowGridLines && XAxis.Options.ShowGridLines)
					{
						XAxis.GridLines(out var min, out var max, out var step);

						var o2w = cam.O2W;
						o2w.pos = cam.FocusPoint - o2w * new v4((float)(wh.x/2 - min), wh.y/2, Owner.Options.GridZOffset, 0);

						XAxis.GridLineGfx.O2WSet(o2w);
						window.AddObject(XAxis.GridLineGfx);
					}
					else
					{
						window.RemoveObject(XAxis.GridLineGfx);
					}
				}
				if (YAxis.GridLineGfx != null)
				{
					if (YAxis.Owner.Options.ShowGridLines && YAxis.Options.ShowGridLines)
					{
						YAxis.GridLines(out var min, out var max, out var step);

						var o2w = cam.O2W;
						o2w.pos = cam.FocusPoint - o2w * new v4(wh.x/2, (float)(wh.y/2 - min), Owner.Options.GridZOffset, 0);

						YAxis.GridLineGfx.O2WSet(o2w);
						window.AddObject(YAxis.GridLineGfx);
					}
					else
					{
						window.RemoveObject(YAxis.GridLineGfx);
					}
				}
			}

			/// <summary>Notify of the axis zooming</summary>
			private void HandleAxisZoomed(object sender, EventArgs e)
			{
				// Invalidate the cached grid line graphics on zoom (for both axes), since the model will need to change size
				XAxis.GridLineGfx = null;
				YAxis.GridLineGfx = null;

				// If a moved event is pending, ensure zoomed is added to the args
				if (m_moved_args == null)
				{
					m_moved_args = new ChartMovedEventArgs(EMoveType.None);
					Dispatcher.CurrentDispatcher.BeginInvoke(new Action(NotifyMoved));
				}
				if (sender == XAxis) m_moved_args.MoveType |= EMoveType.XZoomed;
				if (sender == YAxis) m_moved_args.MoveType |= EMoveType.YZoomed;
			}

			/// <summary>Notify of the axis scrolled</summary>
			private void HandleAxisScrolled(object sender, EventArgs e)
			{
				if (m_moved_args == null)
				{
					m_moved_args = new ChartMovedEventArgs(EMoveType.None);
					Dispatcher.CurrentDispatcher.BeginInvoke(new Action(NotifyMoved));
				}
				if (sender == XAxis) m_moved_args.MoveType |= EMoveType.XScrolled;
				if (sender == YAxis) m_moved_args.MoveType |= EMoveType.YScrolled;
			}

			/// <summary>Raise the ChartMoved event on the chart</summary>
			private void NotifyMoved()
			{
				Owner.OnChartMoved(m_moved_args);
				m_moved_args = null;
			}
			private ChartMovedEventArgs m_moved_args;

			/// <summary>A axis on the chart (typically X or Y)</summary>
			public sealed class Axis :IDisposable
			{
				public Axis(EAxis axis, ChartControl owner)
					:this(axis, owner, 0f, 1f)
				{ }
				public Axis(EAxis axis, ChartControl owner, double min, double max)
				{
					Debug.Assert(axis == EAxis.XAxis || axis == EAxis.YAxis);
					Debug.Assert(owner != null);
					Set(min, max);
					AxisType        = axis;
					Owner           = owner;
					Label           = string.Empty;
					AllowScroll     = true;
					AllowZoom       = true;
					LockRange       = false;
					TickText        = DefaultTickText;
					MeasureTickText = DefaultMeasureTickText;
				}
				public Axis(Axis rhs)
				{
					Set(rhs.Min, rhs.Max);
					AxisType        = rhs.AxisType;
					Owner           = rhs.Owner;
					Label           = rhs.Label;
					AllowScroll     = rhs.AllowScroll;
					AllowZoom       = rhs.AllowZoom;
					LockRange       = rhs.LockRange;
					TickText        = rhs.TickText;
					MeasureTickText = rhs.MeasureTickText;
				}
				public void Dispose()
				{
					m_disposed = true;
					GridLineGfx = null;
				}
				private bool m_disposed;

				/// <summary>Render options for the axis</summary>
				public RdrOptions.Axis Options
				{
					get
					{
						if (Owner.XAxis == this) return Owner.Options.XAxis;
						if (Owner.YAxis == this) return Owner.Options.YAxis;
						throw new Exception("Owner is not the owner of this axis");
					}
				}

				/// <summary>Which axis this is</summary>
				public EAxis AxisType { get; private set; }

				/// <summary>The chart that owns this axis</summary>
				public ChartControl Owner { get; private set; }

				/// <summary>The axis label</summary>
				public string Label
				{
					[DebuggerStepThrough] get { return m_label ?? string.Empty; }
					set
					{
						if (m_label == value) return;
						m_label = value;
					}
				}
				private string m_label;

				/// <summary>The minimum axis value</summary>
				public double Min
				{
					[DebuggerStepThrough] get { return m_min; }
					set
					{
						if (m_min == value) return;
						Set(value, Max);
					}
				}
				private double m_min;

				/// <summary>The maximum axis value</summary>
				public double Max
				{
					[DebuggerStepThrough] get { return m_max; }
					set
					{
						if (m_max == value) return;
						Set(Min, value);
					}
				}
				private double m_max;

				/// <summary>The total range of this axis (max - min)</summary>
				public double Span
				{
					[DebuggerStepThrough] get { return Max - Min; }
					set
					{
						if (Span == value) return;
						if (value <= 0) throw new Exception ($"Invalid axis span: {value}");
						Set(Centre - 0.5*value, Centre + 0.5*value);
					}
				}

				/// <summary>The centre value of the range</summary>
				public double Centre
				{
					[DebuggerStepThrough] get { return (Min + Max) * 0.5; }
					set
					{
						if (Centre == value) return;
						Set(m_min + value - Centre, m_max + value - Centre);
					}
				}

				/// <summary>The min/max limits as a range</summary>
				public RangeF Range
				{
					[DebuggerStepThrough] get { return new RangeF(Min, Max); }
					set
					{
						if (Equals(Range, value)) return;
						Set(value.Beg, value.End);
					}
				}

				/// <summary>Allow scrolling on this axis</summary>
				public bool AllowScroll
				{
					get { return m_allow_scroll && !LockRange; }
					set
					{
						if (m_allow_scroll == value) return;
						m_allow_scroll = value;
					}
				}
				private bool m_allow_scroll;

				/// <summary>Allow zooming on this axis</summary>
				public bool AllowZoom
				{
					get { return m_allow_zoom && !LockRange; }
					set
					{
						if (m_allow_zoom == value) return;
						m_allow_zoom = value;
					}
				}
				private bool m_allow_zoom;

				/// <summary>Get/Set whether the range can be changed by user input</summary>
				public bool LockRange { get; set; }

				/// <summary>Convert the axis value to a string. "string TickText(double tick_value, double step_size)" </summary>
				public Func<double, double, string> TickText;

				/// <summary>Return the width/height to reserve for the tick text. "SizeF MeasureTickText(Graphics gfx)"</summary>
				public Func<Graphics, SizeF> MeasureTickText;

				/// <summary>Set the range without risk of an assert if 'min' is greater than 'Max' or visa versa</summary>
				public void Set(double min, double max)
				{
					Debug.Assert(min < max, "Range must be positive and non-zero");
					var zoomed = !Math_.FEql(max - min, m_max - m_min);
					var scroll = !Math_.FEql((max + min)*0.5, (m_max + m_min)*0.5);

					m_min = min;
					m_max = max;

					if (zoomed) OnZoomed();
					if (scroll) OnScroll();
				}
				public void Set(RangeF range)
				{
					Set(range.Beg, range.End);
				}

				/// <summary>Raised whenever the range scales</summary>
				public event EventHandler Zoomed;
				private void OnZoomed()
				{
					Zoomed?.Invoke(this, EventArgs.Empty);
				}

				/// <summary>Raised whenever the range shifts</summary>
				public event EventHandler Scroll;
				private void OnScroll()
				{
					Scroll?.Invoke(this, EventArgs.Empty);
				}

				/// <summary>Scroll the axis by 'delta'</summary>
				public void Shift(double delta)
				{
					if (!AllowScroll) return;
					Centre += delta;
				}

				/// <summary>Return the position of the grid lines for this axis</summary>
				public void GridLines(out double min, out double max, out double step)
				{
					var dims = Owner.ChartDimensions;
					var axis_length = Owner.XAxis == this ? dims.ChartArea.Width : Owner.YAxis == this ? dims.ChartArea.Height : 0.0;
					var max_ticks = axis_length / Options.PixelsPerTick;

					// Choose step sizes
					var span = Span;
					double step_base = Math.Pow(10.0, (int)Math.Log10(Span)); step = step_base;
					foreach (var s in new[] { 0.05f, 0.1f, 0.2f, 0.25f, 0.5f, 1.0f, 2.0f, 5.0f, 10.0f, 20.0f, 50.0f })
					{
						if (s * span > max_ticks * step_base) continue;
						step = step_base / s;
					}

					min = (Min - Math.IEEERemainder(Min, step)) - Min;
					max = Span * 1.0001;
					if (min < 0.0) min += step;

					// Protect against increments smaller than can be represented
					if (min + step == min)
						step = (max - min) * 0.01f;

					// Protect against too many ticks along the axis
					if (max - min > step*100)
						step = (max - min) * 0.01f;
				}
				public IEnumerable<double> EnumGridLines
				{
					get
					{
						GridLines(out var min, out var max, out var step);
						for (var x = min; x < max; x += step)
							yield return x;
					}
				}

				/// <summary>The graphics object used for grid lines</summary>
				internal View3d.Object GridLineGfx
				{
					get
					{
						if (m_disposed) throw new Exception();
						return m_gridlines ?? (m_gridlines = CreateGridLineGfx());
					}
					set
					{
						if (m_gridlines == value) return;
						Util.Dispose(ref m_gridlines);
					}
				}
				private View3d.Object m_gridlines;
				private View3d.Object CreateGridLineGfx()
				{
					// Create a model for the grid lines
					// Need to allow for one step in either direction because we only create the
					// grid lines model when scaling and we can translate by a max of one step in
					// either direction.
					GridLines(out var min, out var max, out var step);
					var num_lines = (int)(2 + (max - min) / step);

					// Create the grid lines at the origin, they get positioned as the camera moves
					var verts = new View3d.Vertex[num_lines * 2];
					var indices = new ushort[num_lines * 2];
					var nuggets = new View3d.Nugget[1];
					var name = string.Empty;
					var v = 0;
					var i = 0;

					// Grid verts
					if (this == Owner.XAxis)
					{
						name = "xaxis_grid";
						var x = 0f; var y0 = 0f; var y1 = (float)Owner.YAxis.Span;
						for (int l = 0; l != num_lines; ++l)
						{
							verts[v++] = new View3d.Vertex(new v4(x, y0, 0f, 1f), Options.GridColour.ToArgbU());
							verts[v++] = new View3d.Vertex(new v4(x, y1, 0f, 1f), Options.GridColour.ToArgbU());
							x += (float)step;
						}
					}
					if (this == Owner.YAxis)
					{
						name = "yaxis_grid";
						var y = 0f; var x0 = 0f; var x1 = (float)Owner.XAxis.Span;
						for (int l = 0; l != num_lines; ++l)
						{
							verts[v++] = new View3d.Vertex(new v4(x0, y, 0f, 1f), Options.GridColour.ToArgbU());
							verts[v++] = new View3d.Vertex(new v4(x1, y, 0f, 1f), Options.GridColour.ToArgbU());
							y += (float)step;
						}
					}

					// Grid indices
					for (int l = 0; l != num_lines; ++l)
					{
						indices[i] = (ushort)i++;
						indices[i] = (ushort)i++;
					}

					// Grid nugget
					nuggets[0] = new View3d.Nugget(View3d.ETopo.LineList, View3d.EGeom.Vert|View3d.EGeom.Colr);
					var gridlines = new View3d.Object(name, 0xFFFFFFFF, verts.Length, indices.Length, nuggets.Length, verts, indices, nuggets, ChartTools.Id);
					gridlines.FlagsSet(View3d.ELdrFlags.SceneBoundsExclude|View3d.ELdrFlags.NoZWrite, true);
					return gridlines;
				}

				/// <summary>Show the axis context menu</summary>
				public void ShowContextMenu(Point location, HitTestResult hit_result)
				{
					var cmenu = new ContextMenuStrip();
					using (cmenu.SuspendLayout(true))
					{
						{// Lock Range
							var opt = cmenu.Items.Add2(new ToolStripMenuItem("Lock") { Checked = LockRange, CheckOnClick = true });
							opt.CheckedChanged += (s,a) =>
							{
								LockRange = opt.Checked;
							};
						}

						// Customise the menu
						var type = AxisType == EAxis.XAxis ? AddUserMenuOptionsEventArgs.EType.XAxis : AddUserMenuOptionsEventArgs.EType.YAxis;
						Owner.OnAddUserMenuOptions(new AddUserMenuOptionsEventArgs(type, cmenu, hit_result));
					}
					cmenu.Items.TidySeparators();
					if (cmenu.Items.Count != 0)
						cmenu.Show(Owner, location);
				}

				/// <summary>Default value to text conversion</summary>
				public string DefaultTickText(double x, double step)
				{
					// This solves the rounding problem for values near zero when the axis span could be anything
					return !Math_.FEql(x / Span, 0.0) ? Math_.RoundSD(x, 5).ToString("G8") : "0";
				}

				/// <summary>Default tick text measurement</summary>
				public SizeF DefaultMeasureTickText(Graphics gfx)
				{
					// Can't use 'GridLines' here because creates an infinite recursion.
					// Using TickText(Min/Max, 0.0) causes the axes to jump around.
					// The best option is to use one fixed length string.
					return gfx.MeasureString("-9.99999", Options.TickFont);
				}

				/// <summary>Friendly string view</summary>
				public override string ToString()
				{
					return $"{Label} - [{Min}:{Max}]";
				}

				#region Equals
				public bool Equals(Axis rhs)
				{
					// Value equal if the axes represent the same range
					// but not necessarily the same options.
					return
						rhs      != null &&
						AxisType == rhs.AxisType &&
						Label    == rhs.Label &&
						Min      == rhs.Min &&
						Max      == rhs.Max;
				}
				public override bool Equals(object obj)
				{
					return Equals(obj as Axis);
				}
				public override int GetHashCode()
				{
					return new { AxisType, Label, Min, Max, Options }.GetHashCode();
				}
				#endregion
			}
		}

		#endregion

		#region Navigation

		// Prefer camera matrix operations for navigation, then call
		// SetRangeFromCamera to update the X/Y axis since these work
		// for 2D or 3D.

		/// <summary>Navigation methods for moving the camera</summary>
		public enum ENavMode
		{
			[Desc("2D Chart")]
			Chart2D,

			[Desc("3D Scene")]
			Scene3D,
		}

		/// <summary>Enable/Disable mouse navigation</summary>
		[Browsable(false)]
		public bool DefaultMouseControl
		{
			get;
			set
			{
				if (field == value) return;
				field = value;
			}
		}

		/// <summary>Enable/Disable keyboard shortcuts for navigation</summary>
		public bool DefaultKeyboardShortcuts
		{
			get;
			set
			{
				if (field == value) return;
				field = value;
			}
		}

		/// <summary>The default behaviour of area select mode</summary>
		public EAreaSelectMode AreaSelectMode { get; set; }
		public enum EAreaSelectMode { Disabled, SelectElements, Zoom }

		/// <summary>Mouse events on the chart</summary>
		protected override void OnMouseDown(MouseEventArgs e)
		{
			base.OnMouseDown(e);

			// If a mouse op is already active, ignore mouse down
			if (MouseOperations.Active != null)
				return;

			// Look for the mouse op to perform
			if (MouseOperations.Pending(e.Button) == null && DefaultMouseControl)
			{
				switch (e.Button)
				{
				default: return;
				case MouseButtons.Left:   MouseOperations.SetPending(e.Button, new MouseOpDefaultLButton(this)); break;
				case MouseButtons.Middle: MouseOperations.SetPending(e.Button, new MouseOpDefaultMButton(this)); break;
				case MouseButtons.Right:  MouseOperations.SetPending(e.Button, new MouseOpDefaultRButton(this)); break;
				}
			}

			// Start the next mouse op
			MouseOperations.BeginOp(e.Button);

			// Get the mouse op, save mouse location and hit test data, then call op.MouseDown()
			var op = MouseOperations.Active;
			if (op != null && !op.Cancelled)
			{
				op.m_btn_down    = true;
				op.m_grab_client = e.Location; // Note: in ChartControl space, not ChartPanel space
				op.m_grab_chart  = ClientToChart(op.m_grab_client);
				op.m_hit_result  = HitTestCS(op.m_grab_client, ModifierKeys, null);
				op.MouseDown(e);
				Capture = true;
			}
		}
		protected override void OnMouseMove(MouseEventArgs e)
		{
			base.OnMouseMove(e);

			// Look for the mouse op to perform
			var op = MouseOperations.Active;
			if (op != null)
			{
				if (!op.Cancelled)
					op.MouseMove(e);
			}
			// Otherwise, provide mouse hover detection
			else
			{
				var hit = HitTestCS(e.Location, ModifierKeys, null);
				var hovered = hit.Hits.Select(x => x.Element).ToHashSet(0);

				// Remove elements that are no longer hovered
				// and remove existing hovered's from the set.
				for (int i = Hovered.Count; i-- != 0;)
				{
					if (hovered.Contains(Hovered[i]))
						hovered.Remove(Hovered[i]);
					else
						Hovered.RemoveAt(i);
				}
				
				// Add elements that are now hovered
				Hovered.AddRange(hovered);
			}
		}
		protected override void OnMouseUp(MouseEventArgs e)
		{
			base.OnMouseUp(e);

			// Only release the mouse when all buttons are up
			if (MouseButtons == MouseButtons.None)
				Capture = false;

			// Look for the mouse op to perform
			var op = MouseOperations.Active;
			if (op != null && !op.Cancelled)
				op.MouseUp(e);
			
			MouseOperations.EndOp(e.Button);
		}
		protected override void OnMouseWheel(MouseEventArgs e)
		{
			base.OnMouseWheel(e);

			var scene_bounds = SceneBounds;
			var perp_z = Options.PerpendicularZTranslation != (ModifierKeys == Keys.Alt);
			var chart_pt = ClientToChart(e.Location);

			if (scene_bounds.Contains(e.Location))
			{
				// If there is a mouse op in progress, ignore the wheel
				var op = MouseOperations.Active;
				if (op == null || op.Cancelled)
				{
					// Translate the camera along a ray through 'point'
					var loc = Control_.MapPoint(this, Scene, e.Location);
					Scene.Window.MouseNavigateZ(loc, (EMouseBtns)e.Button, e.Delta, !perp_z);
					Invalidate();
				}
			}
			else if (Options.ShowAxes)
			{
				var scale = 0.001f;
				if (ModifierKeys.HasFlag(Keys.Shift)) scale *= 0.1f;
				if (ModifierKeys.HasFlag(Keys.Alt  )) scale *= 0.01f;
				var delta = Math_.Clamp(e.Delta * scale, -0.999f, 0.999f);

				var chg = false;

				// Change the aspect ratio by zooming on the XAxis
				var xaxis_bounds = Rectangle.FromLTRB(scene_bounds.Left, scene_bounds.Bottom, scene_bounds.Right, ClientSize.Height);
				if (xaxis_bounds.Contains(e.Location) && !XAxis.LockRange)
				{
					if (ModifierKeys == Keys.Control && XAxis.AllowScroll)
					{
						XAxis.Shift(XAxis.Span * delta);
						chg = true;
					}
					if (ModifierKeys != Keys.Control && XAxis.AllowZoom)
					{
						var x = perp_z ? XAxis.Centre : chart_pt.X;
						var left = (XAxis.Min - x) * (1f - delta);
						var rite = (XAxis.Max - x) * (1f - delta);
						XAxis.Set(x + left, x + rite);
						if (Options.LockAspect != null)
							YAxis.Span *= (1f - delta);

						chg = true;
					}
				}

				// Check the aspect ratio by zooming on the YAxis
				var yaxis_bounds = Rectangle.FromLTRB(0, scene_bounds.Top, scene_bounds.Left, scene_bounds.Bottom);
				if (yaxis_bounds.Contains(e.Location) && !YAxis.LockRange)
				{
					if (ModifierKeys == Keys.Control && YAxis.AllowScroll)
					{
						YAxis.Shift(YAxis.Span * delta);
						chg = true;;
					}
					if (ModifierKeys != Keys.Control && YAxis.AllowZoom)
					{
						var y = perp_z ? YAxis.Centre : chart_pt.Y;
						var left = (YAxis.Min - y) * (1f - delta);
						var rite = (YAxis.Max - y) * (1f - delta);
						YAxis.Set(y + left, y + rite);
						if (Options.LockAspect != null)
							XAxis.Span *= (1f - delta);

						chg = true;
					}
				}

				// Set the camera position from the Axis ranges
				if (chg)
				{
					SetCameraFromRange();
					Invalidate();
				}
			}
		}
		protected override void OnKeyDown(KeyEventArgs e)
		{
			SetCursor();

			var op = MouseOperations.Active;
			if (op != null && !e.Handled)
				op.OnKeyDown(e);

			// Allow derived classes to handle the key
			base.OnKeyDown(e);

			// If the current mouse operation doesn't use the key,
			// see if it's a default keyboard shortcut.
			if (!e.Handled && DefaultKeyboardShortcuts)
				TranslateKey(e);

		}
		protected override void OnKeyUp(KeyEventArgs e)
		{
			SetCursor();

			var op = MouseOperations.Active;
			if (op != null && !e.Handled)
				op.OnKeyUp(e);

			base.OnKeyUp(e);
		}

		/// <summary>Set the mouse cursor based on key state</summary>
		protected virtual void SetCursor()
		{
			// Sub class to do something like this:
			// if (ModifierKeys.HasFlag(Keys.Shift))   Cursor = Cursors.ArrowPlus;
			// if (ModifierKeys.HasFlag(Keys.Control)) Cursor = Cursors.ArrowMinus;
			Cursor = Cursors.Default;
		}

		/// <summary>Raised when the chart is clicked with the mouse</summary>
		public event EventHandler<ChartClickedEventArgs> ChartClicked;
		protected virtual void OnChartClicked(ChartClickedEventArgs args)
		{
			ChartClicked?.Invoke(this, args);
		}

		/// <summary>Raised when the chart is area selected</summary>
		public event EventHandler<ChartAreaSelectEventArgs> ChartAreaSelect;
		protected virtual void OnChartAreaSelect(ChartAreaSelectEventArgs args)
		{
			ChartAreaSelect?.Invoke(this, args);

			// Select chart elements by default
			if (!args.Handled)
			{
				switch (AreaSelectMode)
				{
				default: throw new Exception("Unknown area select mode");
				case EAreaSelectMode.Disabled:
					{
						break;
					}
				case EAreaSelectMode.SelectElements:
					{
						var rect = new RectangleF(args.SelectionArea.MinX, args.SelectionArea.MinY, args.SelectionArea.SizeX, args.SelectionArea.SizeY);
						SelectElements(rect, ModifierKeys);
						break;
					}
				case EAreaSelectMode.Zoom:
					{
						PositionChart(args.SelectionArea);
						break;
					}
				}
			}
		}

		/// <summary>Adjust the X/Y axis of the chart to cover the given area</summary>
		public void PositionChart(BBox area)
		{
			// Ignore if the given area is smaller than the minimum selection distance (in screen space)
			var bl = ChartToClient(new PointF(area.MinX, area.MinY));
			var tr = ChartToClient(new PointF(area.MaxX, area.MaxY));
			if (Math.Abs(bl.X - tr.X) < Options.MinSelectionDistance ||
				Math.Abs(bl.Y - tr.Y) < Options.MinSelectionDistance)
				return;

			XAxis.Set(area.MinX, area.MaxX);
			YAxis.Set(area.MinY, area.MaxY);
			SetCameraFromRange();
		}

		/// <summary>Shifts the X and Y range of the chart so that chart space position 'gs_point' is at client space position 'cs_point'</summary>
		public void PositionChart(Point cs_point, PointF gs_point)
		{
			var dst = ClientToChart(cs_point);
			var c2w = Scene.Camera.O2W;
			c2w.pos += c2w.x * (gs_point.X - dst.X) + c2w.y * (gs_point.Y - dst.Y);
			Scene.Camera.O2W = c2w;
			Invalidate();
		}

		/// <summary>Handle navigation keyboard shortcuts</summary>
		public virtual void TranslateKey(KeyEventArgs e)
		{
			if (e.Handled) return;
			switch (e.KeyCode)
			{
			case Keys.Escape:
				{
					if (AllowSelection)
					{
						Selected.Clear();
						Invalidate();
					}
					break;
				}
			case Keys.Delete:
				{
					if (AllowEditing)
					{
						//// Allow the caller to cancel the deletion or change the selection
						//var res = new DiagramChangedRemoveElementsEventArgs(Selected.ToArray());
						//if (!RaiseDiagramChanged(res).Cancel)
						//{
						//	foreach (var elem in res.Elements)
						//	{
						//		var node = elem as Node;
						//		if (node != null)
						//			node.DetachConnectors();
								
						//		var conn = elem as Connector;
						//		if (conn != null)
						//			conn.DetachNodes();

						//		Elements.Remove(elem);
						//	}
						//	Invalidate();
						//}
					}
					break;
				}
			case Keys.F5:
				{
					View3d.ReloadScriptSources();
					Invalidate();
					break;
				}
			case Keys.F7:
				{
					AutoRange();
					break;
				}
			case Keys.A:
				{
					if ((e.Modifiers & Keys.Control) != 0)
					{
						Selected.Clear();
						Selected.AddRange(Elements);
						Invalidate();
						Debug.Assert(CheckConsistency());
					}
					break;
				}
			}
		}

		#endregion

		#region Mouse Operations

		/// <summary>Per button current mouse operation</summary>
		public MouseOps MouseOperations
		{
			[DebuggerStepThrough] get;
			private set;
		}

		/// <summary>Manages per-button mouse operations</summary>
		public class MouseOps
		{
			/// <summary>The next mouse operation for each mouse button</summary>
			private readonly Dictionary<MouseButtons, MouseOp> m_pending;

			public MouseOps()
			{
				m_pending = Enum<MouseButtons>.Values.ToDictionary(k => k, k => (MouseOp)null);
			}

			/// <summary>The currently active mouse op</summary>
			public MouseOp Active
			{
				get { return m_active; }
				private set
				{
					if (m_active == value) return;
					if (m_active != null)
					{
						// Clean up the previous active mouse op
						if (m_active.Cancelled)
							m_active.NotifyCancelled();

						m_active.Disposed -= HandleMouseOpDisposed;
						m_active.Dispose();
					}
					m_active = value;
					if (m_active != null)
					{
						// Watch for external disposing
						m_active.Disposed -= HandleMouseOpDisposed;
						m_active.Disposed += HandleMouseOpDisposed;

						// If the op starts immediately without a mouse down, fake
						// a mouse down event as soon as it becomes active.
						if (!m_active.StartOnMouseDown)
							m_active.MouseDown(null);
					}
				}
			}
			private MouseOp m_active;

			/// <summary>Return the op pending for button 'idx'</summary>
			public MouseOp Pending(MouseButtons btn)
			{
				return m_pending[btn];
			}

			/// <summary>Add a mouse op to be started on the next mouse down event for button 'idx'</summary>
			public void SetPending(MouseButtons btn, MouseOp op)
			{
				if (m_pending[btn] == op) return;
				if (m_pending[btn] != null)
				{
					m_pending[btn].Disposed -= HandleMouseOpDisposed;
					m_pending[btn].Dispose();
				}
				m_pending[btn] = op;
				if (m_pending[btn] != null)
				{
					// Watch for external disposing
					m_pending[btn].Disposed -= HandleMouseOpDisposed;
					m_pending[btn].Disposed += HandleMouseOpDisposed;
				}
			}

			/// <summary>Start/End the next mouse op for button 'idx'</summary>
			public void BeginOp(MouseButtons btn)
			{
				Active = m_pending[btn];
				m_pending[btn] = null;
			}
			public void EndOp(MouseButtons btn)
			{
				Active = null;

				// If the next op starts immediately, begin it now
				if (m_pending[btn] != null && !m_pending[btn].StartOnMouseDown)
					BeginOp(btn);
			}

			/// <summary>Handle a mouse op being disposed externally</summary>
			private void HandleMouseOpDisposed(object sender, EventArgs args)
			{
				// This should never be called when 'MouseOps' is the one calling dispose
				// It's only to handle an external reference calling Dispose.
				var op = (MouseOp)sender;
				op.Disposed -= HandleMouseOpDisposed;

				if (m_active == op)
					m_active = null;

				foreach (var key in m_pending.Keys.ToList())
					if (m_pending[key] == op)
						m_pending[key] = null;
			}
		}

		/// <summary>Base class for a mouse operation performed with the mouse 'down -> [drag] -> up' sequence</summary>
		public abstract class MouseOp :IDisposable
		{
			// The general process goes:
			//  A mouse op is created and set as the pending operation in 'MouseOps'.
			//  MouseDown on the chart calls 'BeginOp' which moves the pending op to 'Active'.
			//  Mouse events on the chart are forwarded to the active op
			//  MouseUp ends the current Active op, if the pending op should start immediately
			//  then mouse up causes the next op to start (with a faked MouseDown event).
			//  If at any point a mouse op is cancelled, no further mouse events are forwarded
			//  to the op. When EndOp is called, a notification can be sent by the op to indicate cancelled.

			/// <summary>The owning chart</summary>
			protected ChartControl m_chart;
			protected Scope m_suspend_scope;

			// Selection data for a mouse button
			public bool          m_btn_down;    // True while the corresponding mouse button is down
			public Point         m_grab_client; // The client space location of where the chart was "grabbed" (note: ChartControl, not ChartPanel space)
			public PointF        m_grab_chart;  // The chart space location of where the chart was "grabbed"
			public HitTestResult m_hit_result;  // The hit test result on mouse down
			private bool         m_is_click;    // True until the mouse is dragged beyond the click threshold

			public MouseOp(ChartControl chart)
			{
				m_chart = chart;
				m_is_click = true;
				StartOnMouseDown = true;
				Cancelled = false;
			}
			public void Dispose()
			{
				Dispose(true);
				GC.SuppressFinalize(this);
			}
			protected virtual void Dispose(bool disposing)
			{
				Disposed?.Invoke(this, EventArgs.Empty);
				Util.Dispose(ref m_suspend_scope);
			}
			public event EventHandler Disposed;

			/// <summary>True if mouse down starts the op, false if the op should start as soon as possible</summary>
			public bool StartOnMouseDown { get; set; }

			/// <summary>True if the op was aborted</summary>
			public bool Cancelled { get; protected set; }

			/// <summary>True if the distance between 'location' and mouse down should be treated as a click</summary>
			public bool IsClick(Point location)
			{
				if (!m_is_click) return false;
				var grab = v2.From(m_grab_client);
				var diff = v2.From(location) - grab;
				return m_is_click = diff.LengthSq < Math_.Sqr(m_chart.Options.MinDragPixelDistance);
			}

			/// <summary>Called on mouse down</summary>
			public virtual void MouseDown(MouseEventArgs e) { }

			/// <summary>Called on mouse move</summary>
			public virtual void MouseMove(MouseEventArgs e) { }

			/// <summary>Called on mouse up</summary>
			public virtual void MouseUp(MouseEventArgs e) { }

			/// <summary>Called on key down</summary>
			public virtual void OnKeyDown(KeyEventArgs e) { }

			/// <summary>Called on key up</summary>
			public virtual void OnKeyUp(KeyEventArgs e) { }

			/// <summary>Called when the mouse operation is cancelled</summary>
			public virtual void NotifyCancelled() {}
		}

		/// <summary>A mouse operation for dragging selected elements around, area selecting, or left clicking (Left Button)</summary>
		public class MouseOpDefaultLButton :MouseOp
		{
			private HitTestResult.Hit m_hit_selected;
			private bool m_selection_graphic_added;
			private EAxis m_hit_axis;

			public MouseOpDefaultLButton(ChartControl chart) :base(chart)
			{
				m_selection_graphic_added = false;
			}
			public override void MouseDown(MouseEventArgs e)
			{
				// See where mouse down occurred
				if (m_chart.SceneBounds.Contains(e.Location)) m_hit_axis = EAxis.None;
				if (m_chart.XAxisBounds.Contains(e.Location)) m_hit_axis = EAxis.XAxis;
				if (m_chart.YAxisBounds.Contains(e.Location)) m_hit_axis = EAxis.YAxis;

				// Look for a selected object that the mouse operation starts on
				m_hit_selected = m_hit_result.Hits.FirstOrDefault(x => x.Element.Selected);

				// Record the drag start positions for selected objects
				foreach (var elem in m_chart.Selected)
					elem.DragStartPosition = elem.Position;

				// For 3D scenes, left mouse rotates if mouse down is within the chart bounds
				if (m_chart.Options.NavigationMode == ENavMode.Scene3D && m_hit_axis == EAxis.None)
					m_chart.Scene.Window.MouseNavigate(e.Location, (EMouseBtns)e.Button, View3d.ENavOp.Rotate, true);

				// Prevent events while dragging the elements around
				m_suspend_scope = m_chart.SuspendChartChanged(raise_on_resume:true);
			}
			public override void MouseMove(MouseEventArgs e)
			{
				// If we haven't dragged, treat it as a click instead (i.e. ignore till it's a drag operation)
				if (IsClick(e.Location) && !m_selection_graphic_added)
					return;

				var drag_selected = m_chart.AllowEditing && m_hit_selected != null;
				if (drag_selected)
				{
					// If the drag operation started on a selected element then drag the
					// selected elements within the diagram.
					var delta = Point_.Subtract(m_chart.ClientToChart(e.Location), m_grab_chart);
					m_chart.DragSelected(v2.From(delta), false);
				}
				else if (m_chart.Options.NavigationMode == ENavMode.Chart2D)
				{
					if (m_chart.AreaSelectMode != EAreaSelectMode.Disabled)
					{
						// Otherwise change the selection area
						if (!m_selection_graphic_added)
						{
							m_chart.Scene.AddObject(m_chart.Tools.AreaSelect);
							m_selection_graphic_added = true;
						}

						// Position the selection graphic
						var selection_area = BRect.FromBounds(m_grab_chart, m_chart.ClientToChart(e.Location));
						m_chart.Tools.AreaSelect.O2P = m4x4.Scale(
							selection_area.SizeX,
							selection_area.SizeY,
							1f,
							new v4(selection_area.Centre, m_chart.HighestZ, 1));
					}
				}
				else if (m_chart.Options.NavigationMode == ENavMode.Scene3D)
				{
					// MouseButtons.Right provides rotation, which we want for left mouse button.
					m_chart.Scene.Window.MouseNavigate(e.Location, (EMouseBtns)e.Button, View3d.ENavOp.Rotate, false);
				}
				m_chart.Invalidate();
				m_chart.Update();
			}
			public override void MouseUp(MouseEventArgs e)
			{
				Util.Dispose(ref m_suspend_scope);

				// If this is a single click...
				if (IsClick(e.Location))
				{
					// Pass the click event out to users first
					var args = new ChartClickedEventArgs(m_hit_result, e);
					m_chart.OnChartClicked(args);

					// If a selected element was hit on mouse down, see if it handles the click
					if (!args.Handled && m_hit_selected != null)
					{
						m_hit_selected.Element.HandleClicked(args);
					}

					// If no selected element was hit, try hovered elements
					if (!args.Handled && m_hit_result.Hits.Count != 0)
					{
						for (int i = 0; i != m_hit_result.Hits.Count && !args.Handled; ++i)
							m_hit_result.Hits[i].Element.HandleClicked(args);
					}

					// If the click is still unhandled, use the click to try to select something (if within the chart)
					if (!args.Handled && m_hit_result.Zone.HasFlag(EZone.Chart))
					{
						var selection_area = new BRect(m_grab_chart, v2.Zero);
						m_chart.SelectElements(selection_area, ModifierKeys);
					}
				}
				// Otherwise this is a drag action
				else
				{
					// If an element was selected, drag it around
					if (m_hit_selected != null && m_chart.AllowEditing)
					{
						var delta = Point_.Subtract(m_chart.ClientToChart(e.Location), m_grab_chart);
						m_chart.DragSelected(delta, true);
					}
					else if (m_chart.Options.NavigationMode == ENavMode.Chart2D)
					{
						// Otherwise create an area selection if the click started within the chart
						if (m_hit_result.Zone.HasFlag(EZone.Chart) && m_chart.AreaSelectMode != EAreaSelectMode.Disabled)
						{
							var selection_area = BBox.From(new v4(m_grab_chart, 0, 1f), new v4(m_chart.ClientToChart(e.Location), 0f, 1f));
							m_chart.OnChartAreaSelect(new ChartAreaSelectEventArgs(selection_area));
						}
					}
					else if (m_chart.Options.NavigationMode == ENavMode.Scene3D)
					{
						// For 3D scenes, left mouse rotates
						m_chart.Scene.Window.MouseNavigate(e.Location, (EMouseBtns)e.Button, View3d.ENavOp.Rotate, true);
					}
				}

				// Remove the area selection graphic
				if (m_selection_graphic_added)
					m_chart.Scene.RemoveObject(m_chart.Tools.AreaSelect);

				m_chart.Cursor = Cursors.Default;
				m_chart.Invalidate();
			}
		}

		/// <summary>A mouse operation for zooming (Middle Button)</summary>
		public class MouseOpDefaultMButton :MouseOp
		{
			private HintBalloon m_tape_measure_balloon;
			private bool m_tape_measure_graphic_added;

			public MouseOpDefaultMButton(ChartControl chart) :base(chart)
			{
				m_tape_measure_balloon = new HintBalloon
				{
					AutoSizeMode = AutoSizeMode.GrowOnly,
					Font = new Font(FontFamily.GenericMonospace, 8f),
					Owner = chart.TopLevelControl as Form,
				};
				m_tape_measure_graphic_added = false;
			}
			protected override void Dispose(bool disposing)
			{
				base.Dispose(disposing);
			}
			public override void MouseDown(MouseEventArgs e)
			{
				// If mouse down occurred within the chart, record it
				if (m_chart.SceneBounds.Contains(e.Location))
				{
					m_chart.Cursor = Cursors.CrossHair;
				}
			}
			public override void MouseMove(MouseEventArgs e)
			{
				// If we haven't dragged, treat it as a click instead (i.e. ignore till it's a drag operation)
				if (IsClick(e.Location) && !m_tape_measure_graphic_added)
					return;

				// Position the tape measure graphic
				var pt0 = m_chart.Camera.O2W * m_chart.ChartToCamera(m_grab_chart);
				var pt1 = m_chart.Camera.O2W * m_chart.ChartToCamera(m_chart.ClientToChart(e.Location));
				var delta = pt1 - pt0;
				m_chart.Tools.TapeMeasure.O2P = Math_.TxfmFromDir(EAxisId.PosZ, delta, pt0) * m4x4.Scale(1f, 1f, delta.Length, v4.Origin);
				m_tape_measure_balloon.Location = m_chart.PointToScreen(e.Location);
				m_tape_measure_balloon.Text =
					$"dX:  {delta.x}\r\n"+
					$"dY:  {delta.y}\r\n"+
					$"dZ:  {delta.z}\r\n"+
					$"Len: {delta.Length}\r\n";

				// Show the tape measure graphic (after the text has been initialised)
				if (!m_tape_measure_graphic_added)
				{
					m_chart.Scene.AddObject(m_chart.Tools.TapeMeasure);
					m_tape_measure_graphic_added = true;
					m_tape_measure_balloon.Visible = true;
				}

				m_chart.Invalidate();
				m_chart.Update();
			}
			public override void MouseUp(MouseEventArgs e)
			{
				Util.Dispose(ref m_suspend_scope);

				// If this is a single click...
				if (IsClick(e.Location))
				{
					// Pass the click event out to users first
					var args = new ChartClickedEventArgs(m_hit_result, e);
					m_chart.OnChartClicked(args);

					if (!args.Handled)
					{
						if (m_hit_result.Zone.HasFlag(EZone.Chart))
						{ }
						else if (m_hit_result.Zone.HasFlag(EZone.XAxis))
						{ }
						else if (m_hit_result.Zone.HasFlag(EZone.YAxis))
						{ }
					}
				}
				// Otherwise this is a drag action
				else
				{
				}

				// Remove the tape measure graphic
				if (m_tape_measure_graphic_added)
				{
					m_chart.Scene.RemoveObject(m_chart.Tools.TapeMeasure);
					m_tape_measure_balloon.Visible = false;
				}

				m_chart.Cursor = Cursors.Default;
				m_chart.Invalidate();
			}
		}

		/// <summary>A mouse operation for dragging the chart around or right clicking (Right Button)</summary>
		public class MouseOpDefaultRButton :MouseOp
		{
			/// <summary>The allowed motion based on where the chart was grabbed</summary>
			private EAxis m_drag_axis_allow;

			public MouseOpDefaultRButton(ChartControl chart)
				:base(chart)
			{}
			public override void MouseDown(MouseEventArgs e)
			{
				if (m_chart.SceneBounds.Contains(e.Location)) m_drag_axis_allow = EAxis.Both;
				if (m_chart.XAxisBounds.Contains(e.Location)) m_drag_axis_allow = EAxis.XAxis;
				if (m_chart.YAxisBounds.Contains(e.Location)) m_drag_axis_allow = EAxis.YAxis;

				// Right mouse translates for 2D and 3D scene
				var loc = Control_.MapPoint(m_chart, m_chart.Scene, e.Location);
				m_chart.Scene.Window.MouseNavigate(loc, (EMouseBtns)e.Button, View3d.ENavOp.Translate, true); 
			}
			public override void MouseMove(MouseEventArgs e)
			{
				// If we haven't dragged, treat it as a click instead (i.e. ignore till it's a drag operation)
				if (IsClick(e.Location))
					return;

				// Change the cursor once dragging
				m_chart.Cursor = Cursors.SizeAll;

				// Limit the drag direction
				var drop_loc = Control_.MapPoint(m_chart, m_chart.Scene, e.Location);
				var grab_loc = Control_.MapPoint(m_chart, m_chart.Scene, m_grab_client);
				if (!m_drag_axis_allow.HasFlag(EAxis.XAxis) || m_chart.XAxis.LockRange) drop_loc.X = grab_loc.X;
				if (!m_drag_axis_allow.HasFlag(EAxis.YAxis) || m_chart.YAxis.LockRange) drop_loc.Y = grab_loc.Y;

				m_chart.Scene.Window.MouseNavigate(drop_loc, (EMouseBtns)e.Button, View3d.ENavOp.Translate, false);
				m_chart.SetRangeFromCamera();
				m_chart.Invalidate();
				m_chart.Update();
			}
			public override void MouseUp(MouseEventArgs e)
			{
				m_chart.Cursor = Cursors.Default;

				// If we haven't dragged, treat it as a click instead
				if (IsClick(e.Location))
				{
					var args = new ChartClickedEventArgs(m_hit_result, e);
					m_chart.OnChartClicked(args);

					if (!args.Handled)
					{
						// Show the context menu on right click
						if (args.Button == MouseButtons.Right)
						{
							if (m_hit_result.Zone.HasFlag(EZone.Chart))
								m_chart.ShowContextMenu(args.Location, args.HitResult);
							else if (m_hit_result.Zone.HasFlag(EZone.XAxis))
								m_chart.XAxis.ShowContextMenu(args.Location, args.HitResult);
							else if (m_hit_result.Zone.HasFlag(EZone.YAxis))
								m_chart.YAxis.ShowContextMenu(args.Location, args.HitResult);
						}
					}
					m_chart.Invalidate();
				}
				else
				{
					// Limit the drag direction
					var drop_loc = Control_.MapPoint(m_chart, m_chart.Scene, e.Location);
					var grab_loc = Control_.MapPoint(m_chart, m_chart.Scene, m_grab_client);
					if (!m_drag_axis_allow.HasFlag(EAxis.XAxis) || m_chart.XAxis.LockRange) drop_loc.X = grab_loc.X;
					if (!m_drag_axis_allow.HasFlag(EAxis.YAxis) || m_chart.YAxis.LockRange) drop_loc.Y = grab_loc.Y;

					m_chart.Scene.Window.MouseNavigate(drop_loc, (EMouseBtns)e.Button, View3d.ENavOp.None, true);
					m_chart.SetRangeFromCamera();
					m_chart.Invalidate();
				}
			}
		}

		#endregion

		#region Elements

		/// <summary>Base class for anything on a chart</summary>
		[DebuggerDisplay("{Name} {Id} {GetType().Name}")]
		public abstract class Element :INotifyPropertyChanged ,IDisposable
		{
			/// <summary>Buffers for creating the chart graphics</summary>
			protected List<View3d.Vertex> m_vbuf;
			protected List<View3d.Nugget> m_nbuf;
			protected List<ushort> m_ibuf;

			protected Element(Guid id, m4x4 position, string name)
			{
				m_vbuf = new List<View3d.Vertex>();
				m_nbuf = new List<View3d.Nugget>();
				m_ibuf = new List<ushort>();

				Id                           = id;
				Name                         = name;
				m_impl_chart                 = null;
				m_impl_position              = position;
				m_impl_bounds                = new BBox(position.pos, v4.Zero);
				m_impl_selected              = false;
				m_impl_hovered               = false;
				m_impl_visible               = true;
				m_impl_visible_to_find_range = true;
				m_impl_screen_space          = false;
				m_impl_enabled               = true;
				IsInvalidated                = true;
				UserData                     = new Dictionary<Guid, object>();
			}
			protected Element(XElement node)
				:this(Guid.Empty, m4x4.Identity, string.Empty)
			{
				// Note: Bounds, size, etc are not stored, the implementation
				// of the element provides those (typically in UpdateGfx)
				Id       = node.Element(nameof(Id      )).As<Guid>(Id);
				Position = node.Element(nameof(Position)).As<m4x4>(Position);
				Name     = node.Element(nameof(Name    )).As<string>(Name);
			}
			public void Dispose()
			{
				Dispose(true); // Mimic the WinForms Control disposing pattern
			}
			protected bool Disposed { get; private set; }
			protected virtual void Dispose(bool disposing)
			{
				Utility.Util.BreakIf(!Disposed && Utility.Util.IsGCFinalizerThread);
				Chart = null;
				Invalidated = null;
				PositionChanged = null;
				DataChanged = null;
				SelectedChanged = null;
				Disposed = true;
			}

			/// <summary>Non-null when the element has been added to a chart. Not virtual, override 'SetChartCore' instead</summary>
			public ChartControl Chart
			{
				[DebuggerStepThrough] get { return m_impl_chart; }
				set { SetChartInternal(value, true); }
			}
			private ChartControl m_impl_chart;

			/// <summary>Assign the chart for this element</summary>
			internal void SetChartInternal(ChartControl chart, bool update)
			{
				// An Element can be added to a chart by assigning to the Chart property
				// or by adding it to the Elements collection. It can be removed by setting
				// this property to null, by removing it from the Elements collection, or
				// by Disposing the element. Note: the chart does not own the elements, 
				// elements should only be disposed by the caller.
				if (m_impl_chart == chart) return;

				// Detach from the old chart
				if (m_impl_chart != null && update)
				{
					m_impl_chart.Elements.Remove(this);
				}

				// Assign to the new chart
				SetChartCore(chart);

				// Attach to the new chart
				if (m_impl_chart != null && update)
				{
					Debug.Assert(!m_impl_chart.Elements.Contains(this), "Element already in the Chart's Elements collection");
					m_impl_chart.Elements.Add(this);
				}

				Debug.Assert(CheckConsistency());
				Invalidate();
			}

			/// <summary>Add or remove this element from 'chart'</summary>
			protected virtual void SetChartCore(ChartControl chart)
			{
				// Note: don't suspend events on Chart.Elements.
				// User code maybe watching for ListChanging events.
				Debug.Assert(!Disposed);

				// Remove this element from any selected collection on the outgoing chart
				// This also clears the 'selected' state for the element
				Selected = false;

				// Set the new chart
				m_impl_chart = chart;
			}

			/// <summary>Unique id for this element</summary>
			public Guid Id { get; private set; }

			/// <summary>Debugging name for the element</summary>
			public string Name { get; set; }

			/// <summary>Export to XML</summary>
			public virtual XElement ToXml(XElement node)
			{
				node.Add2(nameof(Id      ), Id      , false);
				node.Add2(nameof(Position), Position, false);
				node.Add2(nameof(Name    ), Name    , false);
				return node;
			}

			/// <summary>Import from XML. Used to update the state of this element without having to delete/recreate it</summary>
			protected virtual void FromXml(XElement node)
			{
				Position = node.Element(nameof(Position)).As<m4x4>(Position);
				Name     = node.Element(nameof(Name    )).As<string>(Name);
			}

			/// <summary>Replace the contents of this element with data from 'node'</summary>
			internal void Update(XElement node)
			{
				// Don't validate the TypeAttribute as users may have sub-classed the element
				using (SuspendEvents())
					FromXml(node);

				Invalidate();
			}

			/// <summary>RAII object for suspending events on this element</summary>
			public Scope SuspendEvents()
			{
				return Scope.Create(
					() =>
					{
						return new IDisposable[]
						{
							//Invalidated.Suspend(),
							//DataChanged.Suspend(),
							//SelectedChanged.Suspend(),
							//PositionChanged.Suspend(),
							//SizeChanged.Suspend(),
						};
					},
					arr =>
					{
						Util.DisposeAll(arr);
					});
			}

			/// <summary>Raised whenever a property of this Element changes</summary>
			public event PropertyChangedEventHandler PropertyChanged;
			protected virtual void OnPropertyChanged(PropertyChangedEventArgs args)
			{
				PropertyChanged?.Invoke(this, args);
			}
			protected void SetProp<T>(ref T prop, T value, string name, bool invalidate_graphics, bool invalidate_chart)
			{
				if (Equals(prop, value)) return;
				prop = value;
				OnPropertyChanged(new PropertyChangedEventArgs(name));
				if (invalidate_graphics) Invalidate();
				if (invalidate_chart) InvalidateChart();
			}

			/// <summary>Raised whenever the element needs to be redrawn</summary>
			public event EventHandler Invalidated;
			protected virtual void OnInvalidated()
			{
				IsInvalidated = true;
				InvalidateChart();
				Invalidated?.Invoke(this, EventArgs.Empty);
			}

			/// <summary>Raised whenever data associated with the element changes</summary>
			public event EventHandler DataChanged;
			protected virtual void OnDataChanged()
			{
				// Raise data changed on this element, and propagate
				// the event to the containing chart as well.
				DataChanged?.Invoke(this, EventArgs.Empty);
				RaiseChartChanged();
			}

			/// <summary>Raised whenever the element is moved</summary>
			public event EventHandler PositionChanged;
			protected void OnPositionChanged()
			{
				PositionChanged?.Invoke(this, EventArgs.Empty);
				RaiseChartChanged();
			}

			/// <summary>Raised whenever the element changes size</summary>
			public event EventHandler SizeChanged;
			protected void OnSizeChanged()
			{
				SizeChanged?.Invoke(this, EventArgs.Empty);
				RaiseChartChanged();
			}

			/// <summary>Raised whenever the element is selected or deselected</summary>
			public event EventHandler SelectedChanged;
			protected virtual void OnSelectedChanged()
			{
				SelectedChanged?.Invoke(this, EventArgs.Empty);
			}

			/// <summary>Raised whenever the element is hovered over with the mouse</summary>
			public event EventHandler HoveredChanged;
			protected virtual void OnHoveredChanged()
			{
				HoveredChanged?.Invoke(this, EventArgs.Empty);
			}

			/// <summary>Signal that the chart needs laying out</summary>
			protected virtual void RaiseChartChanged()
			{
				if (Chart == null) return;
				Chart.OnChartChanged(new ChartChangedEventArgs(EChangeType.Edited));
			}

			/// <summary>Call 'Invalidate' on the containing chart</summary>
			protected virtual void InvalidateChart()
			{
				if (Chart == null) return;
				Chart.Invalidate();
			}

			/// <summary>Indicate that the graphics for this element needs to be recreated or modified</summary>
			public void Invalidate(object sender = null, EventArgs args = null)
			{
				if (IsInvalidated) return;
				OnInvalidated();
			}

			/// <summary>True if this element has been invalidated. Call 'UpdateGfx' to clear the invalidated state</summary>
			public bool IsInvalidated { get; private set; }

			/// <summary>Get/Set the selected state</summary>
			public virtual bool Selected
			{
				get { return m_impl_selected; }
				set { SetSelectedInternal(value, true); }
			}
			private bool m_impl_selected;

			/// <summary>Set the selected state of this element</summary>
			internal void SetSelectedInternal(bool selected, bool update_selected_collection)
			{
				// This allows the chart to set the selected state without
				// adding/removing from the chart's 'Selected' collection.
				if (m_impl_selected == selected) return;
				if (m_impl_selected && update_selected_collection)
				{
					Chart?.Selected.Remove(this);
				}

				SetProp(ref m_impl_selected, selected, nameof(Selected), true, true);

				if (m_impl_selected && update_selected_collection)
				{
					// Selection state is changed by assigning to this property or by
					// addition/removal from the chart's 'Selected' collection.
					Chart?.Selected.Add(this);
				}

				// Notify observers about the selection change
				OnSelectedChanged();
				Invalidate();
			}

			/// <summary>Get/Set the hovered state</summary>
			public virtual bool Hovered
			{
				get { return m_impl_hovered; }
				internal set { SetHoveredInternal(value, true); }
			}
			private bool m_impl_hovered;

			/// <summary>Set the selected state of this element</summary>
			internal void SetHoveredInternal(bool hovered, bool update_hovered_collection)
			{
				// This allows the chart to set the hovered state without
				// adding/removing from the chart's 'Hovered' collection.
				if (m_impl_hovered == hovered) return;
				if (m_impl_hovered && update_hovered_collection)
				{
					Chart?.Hovered.Remove(this);
				}

				SetProp(ref m_impl_hovered, hovered, nameof(Hovered), true, true);

				if (m_impl_hovered && update_hovered_collection)
				{
					// Hovered state is changed by assigning to this property or by
					// addition/removal from the chart's 'Hovered' collection.
					Chart?.Hovered.Add(this);
				}

				// Notify observers about the hovered change
				OnHoveredChanged();
				Invalidate();
			}

			/// <summary>Get/Set whether this element is visible in the chart</summary>
			public bool Visible
			{
				get { return m_impl_visible; }
				set { SetProp(ref m_impl_visible, value, nameof(Visible), false, true); } // Make an element visible/invisible, doesn't invalidate it, only the chart that is displaying it.
			}
			private bool m_impl_visible;

			/// <summary>Get/Set whether this element is enabled</summary>
			public bool Enabled
			{
				get { return m_impl_enabled; }
				set { SetProp(ref m_impl_enabled, value, nameof(Enabled), true, true); } // Changing the enabled data of an element should result in the graphics changing in some way that implies 'Enabled/Disabled'
			}
			private bool m_impl_enabled;

			/// <summary>True if this element is included when finding the range of data in the chart</summary>
			public bool VisibleToFindRange
			{
				get { return m_impl_visible_to_find_range; }
				set { SetProp(ref m_impl_visible_to_find_range, value, nameof(VisibleToFindRange), false, true); } // Make an element visible/invisible, doesn't invalidate it, only the chart that is displaying it.
			}
			private bool m_impl_visible_to_find_range;

			/// <summary>True if this is a screen-space element (i.e. Position is in normalised screen space, not chart space)</summary>
			public bool ScreenSpace
			{
				get { return m_impl_screen_space; }
				protected set { SetProp(ref m_impl_screen_space, value, nameof(ScreenSpace), false, true); }
			}
			private bool m_impl_screen_space;

			/// <summary>Allow users to bind arbitrary data to the chart element</summary>
			public IDictionary<Guid, object> UserData
			{
				get;
				private set;
			}

			/// <summary>Send this element to the bottom of the Z-order</summary>
			public void SendToBack()
			{
				// Z order is determined by position in the Elements collection
				if (Chart == null)
					return;

				// Save the chart pointer because removing the element will remove
				// it from the chart causing 'Chart' to become null before Insert is called.
				var chart = Chart;
				using (chart.Elements.SuspendEvents())
				{
					chart.Elements.Remove(this);
					chart.Elements.Insert(0, this);
				}
				chart.UpdateElementZOrder();
				InvalidateChart();
			}

			/// <summary>Bring this element to top of the stack.</summary>
			public void BringToFront()
			{
				// Z order is determined by position in the Elements collection
				if (Chart == null)
					return;

				// Save the chart pointer because removing the element will remove
				// it from the Chart causing 'Chart' to become null before Insert is called
				var chart = Chart;
				using (chart.Elements.SuspendEvents())
				{
					chart.Elements.Remove(this);
					chart.Elements.Add(this);
				}
				chart.UpdateElementZOrder();
				InvalidateChart();
			}

			/// <summary>The element to chart transform</summary>
			public m4x4 Position
			{
				get { return m_impl_position; }
				set
				{
					if (Math_.FEql(m_impl_position, value)) return;
					SetPosition(value);
				}
			}
			private m4x4 m_impl_position;

			/// <summary>Internal set position and raise event</summary>
			protected virtual void SetPosition(m4x4 pos)
			{
				m_impl_position = pos;
				OnPositionChanged();
				OnPropertyChanged(new PropertyChangedEventArgs(nameof(Position)));
			}

			/// <summary>Get/Set the XY position of the element</summary>
			public v2 PositionXY
			{
				get { return Position.pos.xy; }
				set
				{
					var o2p = Position;
					Position = new m4x4(o2p.rot, new v4(value, o2p.pos.z, o2p.pos.w));
				}
			}

			/// <summary>Get/Set the z position of the element</summary>
			public float PositionZ
			{
				get { return Position.pos.z; }
				set
				{
					var o2p = Position;
					o2p.pos.z = value;
					Position = o2p;
				}
			}

			/// <summary>BBox for the element in chart space</summary>
			public virtual BBox Bounds
			{
				get { return m_impl_bounds; }
				protected set { SetProp(ref m_impl_bounds, value, nameof(Bounds), false, true); } // Doesn't invalidate graphics because bounds is usually set after graphics have been created
			}
			private BBox m_impl_bounds;

			/// <summary>Get/Set the centre point of the element (in chart space)</summary>
			public v4 Centre
			{
				get { return Bounds.Centre; }
				set { Position = new m4x4(Position.rot, value + (Position.pos - Centre)); }
			}

			/// <summary>True if this element can be resized</summary>
			public virtual bool Resizeable
			{
				get { return false; }
			}

			/// <summary>Perform a hit test on this object. Returns null for no hit. 'point' is in client space because typically hit testing uses pixel tolerances</summary>
			public virtual HitTestResult.Hit HitTest(PointF chart_point, Point client_point, Keys modifier_keys, View3d.Camera cam)
			{
				return null;
			}

			/// <summary>Handle a click event on this element</summary>
			public virtual void HandleClicked(ChartClickedEventArgs args)
			{}

			/// <summary>Drag the element 'delta' from the DragStartPosition</summary>
			public virtual void Drag(v2 delta, bool commit)
			{
				var p = DragStartPosition;
				p.pos.x += delta.x;
				p.pos.y += delta.y;
				Position = p;
				if (commit)
					DragStartPosition = Position;
			}

			/// <summary>Position recorded at the time dragging starts</summary>
			internal m4x4 DragStartPosition { get; set; }

			/// <summary>Update the graphics and object transforms associated with this element</summary>
			public void UpdateGfx(object sender = null, EventArgs args = null)
			{
				if (m_impl_updating_gfx != 0) return; // Protect against reentrancy
				using (Scope.Create(() => ++m_impl_updating_gfx, () => --m_impl_updating_gfx))
				{
					UpdateGfxCore();
					IsInvalidated = false;
				}
			}
			protected virtual void UpdateGfxCore() { }
			private int m_impl_updating_gfx;

			/// <summary>Add/Remove the graphics associated with this element to the window</summary>
			internal void UpdateScene(View3d.Window window)
			{
				if (IsInvalidated) UpdateGfx();
				UpdateSceneCore(window);
			}
			protected virtual void UpdateSceneCore(View3d.Window window)
			{
				// Add or remove associated graphics
				// if (Visible)
				// 	window.AddObject();
				// else
				// 	window.RemoveObject();
			}

			/// <summary>Check the self consistency of this element</summary>
			public virtual bool CheckConsistency()
			{
				return true;
			}
		}

		/// <summary>Handle elements added/removed from the elements list</summary>
		protected virtual void OnElementListChanging(ListChgEventArgs<Element> args)
		{
			var elem = args.Item;
			if (elem != null && (elem.Chart != null && elem.Chart != this))
				throw new ArgumentException("element belongs to another chart");

			switch (args.ChangeType)
			{
			case ListChg.Reset:
				{
					Elements.ForEach(x => x.SetChartInternal(this, false));
					ElemIds.Clear();
					Elements.ForEach(x => ElemIds.Add(x.Id, x));
					Debug.Assert(CheckConsistency());
					break;
				}
			case ListChg.ItemAdded:
				{
					elem.SetChartInternal(this, false);
					ElemIds.Add(elem.Id, elem);
					Debug.Assert(CheckConsistency());
					break;
				}
			case ListChg.ItemPreRemove:
				{
					elem.Selected = false;
					elem.Hovered = false;
					Debug.Assert(CheckConsistency());
					break;
				}
			case ListChg.ItemRemoved:
				{
					elem.SetChartInternal(null, false);
					ElemIds.Remove(elem.Id);
					Debug.Assert(CheckConsistency());
					break;
				}
			}
		}

		/// <summary>Handle elements added/removed from the selection list</summary>
		protected virtual void OnSelectionListChanging(ListChgEventArgs<Element> args)
		{
			var elem = args.Item;
			if (elem != null && (elem.Chart != null && elem.Chart != this))
				throw new ArgumentException("element belongs to another chart");

			switch (args.ChangeType)
			{
			case ListChg.Reset:
				{
					Elements.ForEach(x => x.SetSelectedInternal(false, false));
					Selected.ForEach(x => x.SetSelectedInternal(true, false));
					Debug.Assert(CheckConsistency());
					break;
				}
			case ListChg.ItemAdded:
				{
					elem.SetSelectedInternal(true, false);
					Debug.Assert(CheckConsistency());
					break;
				}
			case ListChg.ItemRemoved:
				{
					elem.SetSelectedInternal(false, false);
					Debug.Assert(CheckConsistency());
					break;
				}
			}
		}

		/// <summary>Handle elements added/removed from the hovered list</summary>
		protected virtual void OnHoveredListChanging(ListChgEventArgs<Element> args)
		{
			var elem = args.Item;
			if (elem != null && (elem.Chart != null && elem.Chart != this))
				throw new ArgumentException("element belongs to another chart");

			switch (args.ChangeType)
			{
			case ListChg.Reset:
				{
					Elements.ForEach(x => x.SetHoveredInternal(false, false));
					Hovered .ForEach(x => x.SetHoveredInternal(true, false));
					Debug.Assert(CheckConsistency());
					Invalidate();
					break;
				}
			case ListChg.ItemAdded:
				{
					elem.SetHoveredInternal(true, false);
					Debug.Assert(CheckConsistency());
					Invalidate();
					break;
				}
			case ListChg.ItemRemoved:
				{
					elem.SetHoveredInternal(false, false);
					Debug.Assert(CheckConsistency());
					Invalidate();
					break;
				}
			}
		}

		/// <summary>Move the selected elements by 'delta'</summary>
		private void DragSelected(v2 delta, bool commit)
		{
			if (!AllowEditing) return;
			foreach (var elem in Selected)
				elem.Drag(delta, commit);
		}

		/// <summary>
		/// Select elements that are wholly within 'rect'. (rect is in chart space)
		/// If no modifier keys are down, elements not in 'rect' are deselected.
		/// If 'shift' is down, elements within 'rect' are selected in addition to the existing selection
		/// If 'ctrl' is down, elements within 'rect' are removed from the existing selection.</summary>
		public void SelectElements(RectangleF rect, Keys modifier_keys)
		{
			if (!AllowSelection)
				return;

			// Normalise the selection
			var r = new BBox(new v4(rect.Centre(), 0f, 1f), new v4(Math_.Abs(v2.From(rect.Size))*0.5f, 1f, 0f));

			// If the area of selection is less than the min drag distance, assume click selection
			var is_click = r.DiametreSq < Math_.Sqr(Options.MinDragPixelDistance);
			if (is_click)
			{
				var pt = ChartToClient(rect.Location).ToPoint();
				var hits = HitTestCS(pt, modifier_keys, x => x.Enabled);

				// If control is down, deselect the first selected element in the hit list
				if (Bit.AllSet((int)modifier_keys, (int)Keys.Control))
				{
					var first = hits.Hits.FirstOrDefault(x => x.Element.Selected);
					if (first != null)
						first.Element.Selected = false;
				}
				// If shift is down, select the first element not already selected in the hit list
				else if (Bit.AllSet((int)modifier_keys, (int)Keys.Shift))
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
						to_select = hits.Hits[i+1];
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
				using (Selected.SuspendEvents(true))
				{
					// If control is down, those in the selection area become deselected
					if (Bit.AllSet((int)modifier_keys, (int)Keys.Control))
					{
						// Only need to look in the selected list
						foreach (var elem in Selected.Where(x => r.IsWithin(x.Bounds, 0f)).ToArray())
							elem.Selected = false;
					}
					// If shift is down, those in the selection area become selected
					else if (Bit.AllSet((int)modifier_keys, (int)Keys.Shift))
					{
						foreach (var elem in Elements.Where(x => !x.Selected && r.IsWithin(x.Bounds, 0f)))
							elem.Selected = true;
					}
					// Otherwise, the existing selection is cleared, and those within the selection area become selected
					else
					{
						foreach (var elem in Elements)
							elem.Selected = r.IsWithin(elem.Bounds, 0f);
					}
				}
			}

			Invalidate();
		}

		#endregion

		#region Context Menu

		/// <summary>Create and display a context menu</summary>
		public void ShowContextMenu(Point location, HitTestResult hit_result)
		{
			var cmenu = new ContextMenuStrip { Renderer = new ContextMenuRenderer() };

			using (this.ChangeCursor(Cursors.WaitCursor))
			using (cmenu.SuspendLayout(true))
			{
				#region Objects
				{
					var objects_menu = cmenu.Items.Add2(new ToolStripMenuItem("Objects") { Name = CMenu.Objects });
					{
						var opt = objects_menu.DropDownItems.Add2(new ToolStripMenuItem("Origin") { Name = CMenu.ObjectsMenu.Origin });
						opt.Checked = Scene.Window.OriginPointVisible;
						opt.Click += (s,a) =>
						{
							Scene.Window.OriginPointVisible = !Scene.Window.OriginPointVisible;
							Invalidate();
						};
					}
					{
						var opt = objects_menu.DropDownItems.Add2(new ToolStripMenuItem("Focus") { Name = CMenu.ObjectsMenu.Focus });
						opt.Checked = Scene.Window.FocusPointVisible;
						opt.Click += (s,a) =>
						{
							Scene.Window.FocusPointVisible = !Scene.Window.FocusPointVisible;
							Invalidate();
						};
					}
					{
						var opt = objects_menu.DropDownItems.Add2(new ToolStripMenuItem("Grid Lines") { Name = CMenu.ObjectsMenu.GridLines });
						opt.Checked = Options.ShowGridLines;
						opt.Click += (s,a) =>
						{
							Options.ShowGridLines = !Options.ShowGridLines;
							Invalidate();
						};
					}
					{
						var opt = objects_menu.DropDownItems.Add2(new ToolStripMenuItem("Axes") { Name = CMenu.ObjectsMenu.GridLines });
						opt.Checked = Options.ShowAxes;
						opt.Click += (s,a) =>
						{
							Options.ShowAxes = !Options.ShowAxes;
							Invalidate();
						};
					}
				}
				#endregion
				#region Tools
				{
					var tools_menu = cmenu.Items.Add2(new ToolStripMenuItem("Tools") { Name = CMenu.Tools });
					#region Show Value
					{
						var opt = tools_menu.DropDownItems.Add2(new ToolStripMenuItem("Show Value") { Name = CMenu.ToolsMenu.ShowValue });
						opt.Checked = m_show_value;
						opt.Click += (s, a) =>
						{
							if (m_show_value)
							{
								MouseMove -= OnMouseMoveTooltip;
								MouseWheel -= OnMouseWheelTooltip;
							}
							m_show_value = !m_show_value;
							m_tt_show_value.Size = m_tt_show_value.GetPreferredSize(Size.Empty);
							m_tt_show_value.Visible = m_show_value;
							if (m_show_value)
							{
								MouseMove += OnMouseMoveTooltip;
								MouseWheel += OnMouseWheelTooltip;
							}
						};
					}
					#endregion
					#region Show Cross Hair
					{
						var opt = tools_menu.DropDownItems.Add2(new ToolStripMenuItem("Show Cross Hair") { Name = CMenu.ToolsMenu.ShowXHair });
						opt.Checked = CrossHairVisible;
						opt.Click += (s, a) =>
						{
							CrossHairVisible = !CrossHairVisible;
							Invalidate();
						};
					}
					#endregion
				}
				#endregion
				#region Zoom Menu
				{
					var zoom_menu = cmenu.Items.Add2(new ToolStripMenuItem("Zoom") { Name = CMenu.Zoom });
					{
						var opt = zoom_menu.DropDownItems.Add2(new ToolStripMenuItem("Default") { Name = CMenu.ZoomMenu.Default });
						opt.Click += (s, a) =>
						{
							AutoRange();
						};
					}
					{
						var opt = zoom_menu.DropDownItems.Add2(new ToolStripMenuItem("Aspect 1:1") { Name = CMenu.ZoomMenu.Aspect1to1 });
						opt.Click += (s,a) =>
						{
							Aspect = 1.0f;
						};
					}
					{
						var opt = zoom_menu.DropDownItems.Add2(new ToolStripMenuItem("Lock Aspect") { Name = CMenu.ZoomMenu.LockAspect });
						opt.Checked = Options.LockAspect != null;
						opt.Click += (s,a) =>
						{
							LockAspect = !LockAspect;
						};
					}
					{
						var opt = zoom_menu.DropDownItems.Add2(new ToolStripMenuItem("Perpendicular Z Translation") { Name = CMenu.ZoomMenu.PerpZTrans });
						opt.Checked = Options.PerpendicularZTranslation;
						opt.Click += (s,a) =>
						{
							Options.PerpendicularZTranslation = !Options.PerpendicularZTranslation;
						};
					}
				}
				#endregion
				#region Rendering
				{
					var rendering_menu = cmenu.Items.Add2(new ToolStripMenuItem("Rendering") { Name = CMenu.Rendering });
					{
						var opt = rendering_menu.DropDownItems.Add2(new ToolStripComboBox("Navigation Mode") { Name = CMenu.RenderingMenu.NavigationMode });
						opt.DropDownStyle = ComboBoxStyle.DropDownList;
						opt.ComboBox.DataSource = Enum<ENavMode>.ValuesArray;
						opt.SelectedItem = Options.NavigationMode;
						opt.ComboBox.Format += (s,a) =>
						{
							a.Value = ((ENavMode)a.ListItem).Desc();
						};
						opt.SelectedIndexChanged += (s,a) =>
						{
							Options.NavigationMode = (ENavMode)opt.SelectedItem;
							Invalidate();
						};
					}
					{
						var opt = rendering_menu.DropDownItems.Add2(new ToolStripMenuItem("Orthographic") { Name = CMenu.RenderingMenu.Orthographic });
						opt.Checked = Options.Orthographic;
						opt.Click += (s,a) =>
						{
							Options.Orthographic = !Options.Orthographic;
							Invalidate();
						};
					}
					{
						var fillmode = Enum<View3d.EFillMode>.Cycle(Scene.Window.FillMode);
						var opt = rendering_menu.DropDownItems.Add2(new ToolStripMenuItem(fillmode.ToString()) { Name = CMenu.RenderingMenu.Wireframe });
						opt.Click += (s,a) =>
						{
							Options.FillMode = fillmode;
							Invalidate();
						};
					}
					{
						var opt = rendering_menu.DropDownItems.Add2(new ToolStripMenuItem("Anti-Aliasing") { Name = CMenu.RenderingMenu.AntiAliasing });
						opt.Checked = Options.AntiAliasing;
						opt.Click += (s,a) =>
						{
							Options.AntiAliasing = !Options.AntiAliasing;
							Scene.Window.MultiSampling = Options.AntiAliasing ? 4 : 1;
							Invalidate();
						};
					}
				}
				#endregion
				cmenu.Items.AddSeparator();
				#region Properties
				{
					var opt = cmenu.Items.Add2(new ToolStripMenuItem("Properties") { Name = CMenu.Properties });
					opt.Click += (s,a) =>
					{
						new RdrOptionsUI(this, Options).Show(this);
					};
				}
				#endregion

				// Allow users to add menu options
				OnAddUserMenuOptions(new AddUserMenuOptionsEventArgs(AddUserMenuOptionsEventArgs.EType.Chart, cmenu, hit_result));
			}
			cmenu.Items.TidySeparators();
			cmenu.Closed += (s, a) => Refresh();
			cmenu.Show(this, location);
		}

		/// <summary>Event allowing callers to add options to the context menu</summary>
		public event EventHandler<AddUserMenuOptionsEventArgs> AddUserMenuOptions;
		protected virtual void OnAddUserMenuOptions(AddUserMenuOptionsEventArgs args)
		{
			AddUserMenuOptions?.Invoke(this, args);
		}

		#endregion

		#region Show Value Tool tip

		/// <summary>A tool tip to display the mouse location value</summary>
		private HintBalloon m_tt_show_value;
		private bool m_show_value;

		/// <summary>Handle mouse move events while the tool tip is visible</summary>
		private void OnMouseMoveTooltip(object sender, MouseEventArgs e)
		{
			SetValueToolTip(e.Location);
		}
		private void OnMouseWheelTooltip(object sender, MouseEventArgs e)
		{
			SetValueToolTip(e.Location);
		}

		/// <summary>Update the text in the 'show value' hint balloon. 'location' is in client space</summary>
		private void SetValueToolTip(Point location)
		{
			if (SceneBounds.Contains(location))
			{
				m_tt_show_value.Text = LocationText(location);
				m_tt_show_value.Location = PointToScreen(location);
				m_tt_show_value.Owner = TopLevelControl as Form;
				m_tt_show_value.Visible = true;
			}
			else
			{
				m_tt_show_value.Visible = false;
			}
		}

		#endregion

		#region Show Cross Hair

		/// <summary>True while the cross hair is visible</summary>
		public bool CrossHairVisible
		{
			get { return m_cross_hair_visible; }
			set
			{
				if (m_cross_hair_visible == value) return;
				if (m_cross_hair_visible)
				{
					MouseMove -= OnMouseMoveCrossHair;
					MouseWheel -= OnMouseWheelCrossHair;
					Scene.RemoveObject(m_tools.CrossHairH);
					Scene.RemoveObject(m_tools.CrossHairV);
				}
				m_cross_hair_visible = value;
				if (m_cross_hair_visible)
				{
					Scene.AddObject(m_tools.CrossHairH);
					Scene.AddObject(m_tools.CrossHairV);
					MouseWheel += OnMouseWheelCrossHair;
					MouseMove += OnMouseMoveCrossHair;
				}
				Invalidate();
			}
		}
		private bool m_cross_hair_visible;

		/// <summary>The chart-space location of the cross hair</summary>
		public PointF CrossHairLocation
		{
			get { return m_cross_hair_location; }
			set
			{
				if (m_cross_hair_location == value) return;
				m_cross_hair_location = value;

				// Scale to fill the view
				var view = Camera.ViewArea(Camera.FocusDist);
				var pt_cs = ChartToCamera(m_cross_hair_location);
				if (m_tools.CrossHairH != null)
				{
					// Shift the position to the centre of the camera view
					var o2w = new m4x4(Camera.O2W.rot, Camera.O2W * new v4(0, pt_cs.y, pt_cs.z, 1f));
					m_tools.CrossHairH.O2P = o2w * m3x3.Scale(view.x, 1f, 1f).m4x4;
				}
				if (m_tools.CrossHairV != null)
				{
					// Shift the position to the centre of the camera view
					var o2w = new m4x4(Camera.O2W.rot, Camera.O2W * new v4(pt_cs.x, 0, pt_cs.z, 1f));
					m_tools.CrossHairV.O2P = o2w * m3x3.Scale(1f, view.y, 1f).m4x4;
				}

				// Notify of the new cross hair location
				CrossHairMoved?.Invoke(this, EventArgs.Empty);
				Invalidate();
				Update();
			}
		}
		private PointF m_cross_hair_location;

		/// <summary>Update the cross hair on mouse move</summary>
		private void OnMouseMoveCrossHair(object sender, MouseEventArgs e)
		{
			if (SceneBounds.Contains(e.Location))
			{
				CrossHairLocation = ClientToChart(e.Location);
				Invalidate();
			}
		}
		private void OnMouseWheelCrossHair(object sender, MouseEventArgs e)
		{
			if (SceneBounds.Contains(e.Location))
			{
				CrossHairLocation = ClientToChart(e.Location);
				Invalidate();
			}
		}

		/// <summary>Raised when the cross hair is visible and moved</summary>
		public event EventHandler CrossHairMoved;
		#endregion

		#region Tools

		/// <summary>Chart graphics</summary>
		private ChartTools Tools
		{
			[DebuggerStepThrough] get { return m_tools; }
			set
			{
				if (m_tools == value) return;
				Util.Dispose(ref m_tools);
				m_tools = value;
			}
		}
		private ChartTools m_tools;

		/// <summary>A collection of graphics used by the chart itself</summary>
		public sealed class ChartTools :IDisposable
		{
			public static readonly Guid Id = new Guid("62D495BB-36D1-4B52-A067-1B7DB4011831");

			private readonly ChartControl m_owner;
			internal ChartTools(ChartControl owner, RdrOptions opts)
			{
				m_owner = owner;
				if (owner.IsInDesignMode())
					return;

				Options      = opts;
				AreaSelect   = CreateAreaSelect();
				Resizer      = Array_.New(8, i => new ResizeGrabber(i));
				CrossHairH   = CreateCrossHair(true);
				CrossHairV   = CreateCrossHair(false);
				TapeMeasure  = CreateTapeMeasure();
			}
			public void Dispose()
			{
				Options      = null;
				AreaSelect   = null;
				Resizer      = null;
				CrossHairH   = null;
				CrossHairV   = null;
				TapeMeasure  = null;
			}

			/// <summary>Chart options</summary>
			private RdrOptions Options
			{
				[DebuggerStepThrough] get { return m_options; }
				set
				{
					if (m_options == value) return;
					if (m_options != null) m_options.PropertyChanged -= HandleOptionsChanged;
					m_options = value;
					if (m_options != null) m_options.PropertyChanged += HandleOptionsChanged;
				}
			}
			private RdrOptions m_options;

			/// <summary>Graphic for area selection</summary>
			public View3d.Object AreaSelect
			{
				get;
				private set
				{
					if (field == value) return;
					Util.Dispose(ref field);
					field = value;
				}
			}
			private View3d.Object CreateAreaSelect()
			{
				var ldr = new LdrBuilder().Rect("selection", Options.SelectionColour, EAxisId.PosZ, 1f, 1f, true, v4.Origin).ToString();
				var obj = new View3d.Object(ldr, false, Id, null);
				obj.FlagsSet(View3d.ELdrFlags.SceneBoundsExclude, true);
				return obj;
			}

			/// <summary>Graphics for the resizing grab zones</summary>
			public ResizeGrabber[] Resizer
			{
				get;
				private set
				{
					if (field == value) return;
					Util.DisposeAll(field);
					field = value;
				}
			}
			public class ResizeGrabber : View3d.Object
			{
				public ResizeGrabber(int corner) :base($"*Box resize_{corner} {{5}}", false, Id, null)
				{
					FlagsSet(View3d.ELdrFlags.SceneBoundsExclude, true);
					switch (corner)
					{
					case 0:
						Cursor = Cursors.SizeNESW;
						Direction = Math_.Normalise(new v2(-1,-1));
						Update = (b,z) => O2P = m4x4.Translation(b.Lower.x, b.Lower.y, z);
						break;
					case 1:
						Cursor = Cursors.SizeNESW;
						Direction = Math_.Normalise(new v2(+1,+1));
						Update = (b,z) => O2P = m4x4.Translation(b.Upper.x, b.Upper.y, z);
						break;
					case 2:
						Cursor = Cursors.SizeNWSE;
						Direction = Math_.Normalise(new v2(-1,+1));
						Update = (b,z) => O2P = m4x4.Translation(b.Lower.x, b.Upper.y, z);
						break;
					case 3:
						Cursor = Cursors.SizeNWSE;
						Direction = Math_.Normalise(new v2(+1,-1));
						Update = (b,z) => O2P = m4x4.Translation(b.Upper.x, b.Lower.y, z);
						break;
					case 4:
						Direction = new v2(-1,0);
						Cursor = Cursors.SizeWE;
						Update = (b,z) => O2P = m4x4.Translation(b.Lower.x, b.Centre.y, z);
						break;
					case 5:
						Direction = new v2(+1,0);
						Cursor = Cursors.SizeWE;
						Update = (b,z) => O2P = m4x4.Translation(b.Upper.x, b.Centre.y, z);
						break;
					case 6:
						Direction = new v2(0,-1);
						Cursor = Cursors.SizeNS;
						Update = (b,z) => O2P = m4x4.Translation(b.Centre.x, b.Lower.y, z);
						break;
					case 7:
						Direction = new v2(0,+1);
						Cursor = Cursors.SizeNS;
						Update = (b,z) => O2P = m4x4.Translation(b.Centre.x, b.Upper.y, z);
						break;
					}
				}

				/// <summary>The direction that this grabber can resize in </summary>
				public v2 Direction { get; private set; }

				/// <summary>The cursor to display when this grabber is used</summary>
				public Cursor Cursor { get; set; }

				/// <summary>Updates the position of the grabber</summary>
				public Action<BRect,float> Update;
			}

			/// <summary>A vertical and horizontal line</summary>
			public View3d.Object CrossHairH
			{
				get;
				private set
				{
					if (field == value) return;
					Util.Dispose(ref field);
					field = value;
				}
			}
			public View3d.Object CrossHairV
			{
				get;
				private set
				{
					if (field == value) return;
					Util.Dispose(ref field);
					field = value;
				}
			}
			private View3d.Object CreateCrossHair(bool horiz)
			{
				var col = Options.ChartBkColour.ToV4().xyz.Length > 0.5 ? 0xFFFFFFFF : 0xFF000000;
				var str = horiz
					? new LdrBuilder().Line("chart_cross_hair_h", col, new v4(-0.5f, 0, 0, 1f), new v4(+0.5f, 0, 0, 1f))
					: new LdrBuilder().Line("chart_cross_hair_v", col, new v4(0, -0.5f, 0, 1f), new v4(0, +0.5f, 0, 1f));
				var obj = new View3d.Object(str, false, Id, null);
				obj.FlagsSet(View3d.ELdrFlags.SceneBoundsExclude, true);
				return obj;
			}

			/// <summary>A line for measuring distances</summary>
			public View3d.Object TapeMeasure
			{
				get;
				private set
				{
					if (field == value) return;
					Util.Dispose(ref field);
					field = value;
				}
			}
			private View3d.Object CreateTapeMeasure()
			{
				var col = Options.ChartBkColour.ToV4().xyz.Length > 0.5 ? 0xFFFFFFFF : 0xFF000000;
				var str = new LdrBuilder().Line("tape_measure", col, new v4(0, 0, 0, 1f), new v4(0, 0, 1f, 1f));
				var obj = new View3d.Object(str, false, Id, null);
				obj.FlagsSet(View3d.ELdrFlags.SceneBoundsExclude, true);
				return obj;
			}

			/// <summary>Update the chart tools when the options change</summary>
			private void HandleOptionsChanged(object sender, PropertyChangedEventArgs e)
			{
				if (e.PropertyName == nameof(RdrOptions.SelectionColour))
				{
					AreaSelect.ColourSet(Options.SelectionColour);
				}
				if (e.PropertyName == nameof(RdrOptions.ChartBkColour))
				{
					CrossHairH = CreateCrossHair(true);
					CrossHairV = CreateCrossHair(false);
				}
			}
		}

		#endregion

		#region Nested Classes

		/// <summary>Named parts of the chart</summary>
		[Flags]
		public enum EZone
		{
			None,
			Chart = 1 << 0,
			XAxis = 1 << 1,
			YAxis = 1 << 2,
			Title = 1 << 3,
		}

		/// <summary>Chart change types</summary>
		public enum EChangeType
		{
			/// <summary>
			/// Raised after elements in the chart have been moved, resized, or had their content changed.
			/// This event will be raised in addition to the more detailed modification events below</summary>
			Edited,

			/// <summary>
			/// Elements are about to be deleted from the chart by the user.
			/// Setting 'Cancel' for this event will abort the deletion.</summary>
			RemovingElements,
		}

		/// <summary>Flags for how a chart was scrolled</summary>
		[Flags] public enum EMoveType
		{
			None = 0,
			XZoomed   = 1 << 0,
			YZoomed   = 1 << 1,
			XScrolled = 1 << 2,
			YScrolled = 1 << 3,
		}

		/// <summary>Results collection for a hit test</summary>
		public class HitTestResult
		{
			public HitTestResult(EZone zone, Point client_point, PointF chart_point, Keys modifier_keys, IEnumerable<Hit> hits, View3d.Camera cam)
			{
				Zone         = zone;
				ClientPoint  = client_point;
				ChartPoint   = chart_point;
				Hits         = hits.ToList();
				ModifierKeys = modifier_keys;
				Camera       = cam;
			}

			/// <summary>The zone on the chart that was hit</summary>
			public EZone Zone { get; private set; }

			/// <summary>The client space location of where the hit test was performed</summary>
			public Point ClientPoint { get; private set; }

			/// <summary>The chart space location of where the hit test was performed</summary>
			public PointF ChartPoint { get; private set; }

			/// <summary>The collection of hit objects</summary>
			public List<Hit> Hits { get; private set; }

			/// <summary>Keys held down during the hit test</summary>
			public Keys ModifierKeys { get; private set; }

			/// <summary>The camera position when the hit test was performed (needed for chart to screen space conversion)</summary>
			public View3d.Camera Camera { get; private set; }

			public class Hit
			{
				public Hit(Element elem, PointF elem_point, object context)
				{
					Element  = elem;
					Point    = elem_point;
					Location = elem.Position;
					Context  = context;
				}

				/// <summary>The element that was hit</summary>
				public Element Element { get; private set; }

				/// <summary>Where on the element it was hit (in element space)</summary>
				public PointF Point { get; private set; }

				/// <summary>The element's chart location at the time it was hit</summary>
				public m4x4 Location { get; private set; }

				/// <summary>Optional context information collected during the hit test</summary>
				public object Context { get; private set; }

				public override string ToString()
				{
					return Element.ToString();
				}
			}
		}

		/// <summary>Special menu item that doesn't draw highlighted</summary>
		private class NoHighlightToolStripMenuItem :ToolStripMenuItem
		{
			public NoHighlightToolStripMenuItem(string text) : base(text) { }
		}

		/// <summary>Custom button renderer because the office 'checked' state buttons look crap</summary>
		private class ContextMenuRenderer :ToolStripProfessionalRenderer
		{
			protected override void OnRenderMenuItemBackground(ToolStripItemRenderEventArgs e)
			{
				var item = e.Item as NoHighlightToolStripMenuItem;
				if (item == null) { base.OnRenderMenuItemBackground(e); return; }

				e.Graphics.FillRectangle(new SolidBrush(item.BackColor), e.Item.ContentRectangle);
				e.Graphics.DrawRectangle(Pens.Black, e.Item.ContentRectangle);
			}
		}

		/// <summary>Cursors for the chart</summary>
		public static class Cursors
		{
			public static readonly Cursor Default    = System.Windows.Forms.Cursors.Default;
			public static readonly Cursor WaitCursor = System.Windows.Forms.Cursors.WaitCursor;
			public static readonly Cursor SizeWE     = System.Windows.Forms.Cursors.SizeWE;
			public static readonly Cursor SizeNS     = System.Windows.Forms.Cursors.SizeNS;
			public static readonly Cursor SizeNESW   = System.Windows.Forms.Cursors.SizeNESW;
			public static readonly Cursor SizeNWSE   = System.Windows.Forms.Cursors.SizeNWSE;
			public static readonly Cursor SizeAll    = System.Windows.Forms.Cursors.SizeAll;
			public static readonly Cursor Arrow      = Resources.cursor_arrow.ToCursor(Point.Empty);
			public static readonly Cursor ArrowPlus  = Resources.cursor_arrow_plus.ToCursor(Point.Empty);
			public static readonly Cursor ArrowMinus = Resources.cursor_arrow_minus.ToCursor(Point.Empty);
			public static readonly Cursor CrossHair  = System.Windows.Forms.Cursors.Cross;
		}

		///// <summary>String constants used in XML export/import</summary>
		//public static class XmlTag
		//{
		//	public const string Root                 = "root";
		//	public const string TypeAttribute        = "ty";
		//	public const string Element              = "elem";
		//	public const string Id                   = "id";
		//	public const string Name                 = "name";
		//	public const string Position             = "pos";
		//	public const string XAxis                = "xaxis";
		//	public const string YAxis                = "yaxis";
		//	public const string BkColour             = "bk_colour";
		//	public const string ChartBkColour        = "chart_bk_colour";
		//	public const string TitleColour          = "title_colour";
		//	public const string TitleTransform       = "title_xform";
		//	public const string Margin               = "margin";
		//	public const string TitleFont            = "title_font";
		//	public const string NoteFont             = "note_font";
		//	public const string SelectionColour      = "selection_colour";
		//	public const string ShowGridLines        = "show_grid_lines";
		//	public const string AntiAliasing         = "antialiasing";
		//	public const string MinSelectionDistance = "min_selection_distance";
		//	public const string MinDragPixelDistance = "min_drag_pixel_distance";
		//	public const string AxisColour           = "axis_colour";
		//	public const string LabelColour          = "label_colour";
		//	public const string GridColour           = "grid_colour";
		//	public const string TickColour           = "tick_colour";
		//	public const string LabelFont            = "label_font";
		//	public const string TickFont             = "tick_font";
		//	public const string DrawTickMarks        = "draw_tick_marks";
		//	public const string DrawTickLabels       = "draw_tick_labels";
		//	public const string TickLength           = "tick_length";
		//	public const string MinTickSize          = "min_tick_size";
		//	public const string LabelTransform       = "label_xform";
		//	public const string AxisThickness        = "axis_thickness";
		//	public const string PixelsPerTick        = "pixels_per_tick";
		//}

		/// <summary>Names for context menu items to allow users to identify them</summary>
		public static class CMenu
		{
			public const string Tools = "tools";
			public static class ToolsMenu
			{
				public const string ShowValue = "show_value";
				public const string ShowXHair = "show_cross_hair";
			}
			public const string Zoom = "zoom";
			public static class ZoomMenu
			{
				public const string Default = "default";
				public const string Aspect1to1 = "aspect1:1";
				public const string LockAspect = "lock_aspect";
				public const string PerpZTrans = "perp_z_trans";
			}
			public const string Objects = "objects";
			public static class ObjectsMenu
			{
				public const string Origin = "origin";
				public const string Focus = "focus";
				public const string GridLines = "grid_lines";
				public const string Axes = "axes";
			}
			public const string Rendering = "rendering";
			public static class RenderingMenu
			{
				public const string NavigationMode = "nav_mode";
				public const string Orthographic = "orthographic";
				public const string Wireframe = "wireframe";
				public const string AntiAliasing = "anti_aliasing";
			}
			public const string Properties = "appearance";
			public static class AppearanceMenu
			{
				public const string BkColour = "bk_colour";
				public const string ChartBkColour = "chart_bk_colour";
			}
			public const string Axes = "axes";
			public static class AxesMenu
			{
				public const string XAxis = "x_axis";
				public static class XAxisMenu
				{
					public const string Min = "min";
					public const string Max = "max";
					public const string AllowScroll = "allow_scroll";
					public const string AllowZoom = "allow_zoom";
				}
				public const string YAxis = "y_axis";
				public static class YAxisMenu
				{
					public const string Min = "min";
					public const string Max = "max";
					public const string AllowScroll = "allow_scroll";
					public const string AllowZoom = "allow_zoom";
				}
			}
		}

		#endregion

		#region Event Args

		/// <summary>Event args for the chart changed event</summary>
		public class ChartChangedEventArgs :EventArgs
		{
			// Note: many events are available by attaching to the Elements binding list
			public ChartChangedEventArgs(EChangeType ty)
			{
				ChgType  = ty;
				Cancel   = false;
			}

			/// <summary>The type of change that occurred</summary>
			public EChangeType ChgType { get; private set; }

			/// <summary>A cancel property for "about to change" events</summary>
			public bool Cancel { get; set; }
		}

		/// <summary>Event args for the ChartMoved event</summary>
		public class ChartMovedEventArgs :EventArgs
		{
			public ChartMovedEventArgs(EMoveType move_type)
			{
				MoveType = move_type;
			}

			/// <summary>How the chart was moved</summary>
			public EMoveType MoveType { get; internal set; }
		}

		/// <summary>Event args for the ChartRendering event</summary>
		public class ChartRenderingEventArgs :EventArgs
		{
			public ChartRenderingEventArgs(ChartControl chart, View3d.Window window)
			{
				Chart = chart;
				Window = window;
			}

			/// <summary>The chart that is rendering</summary>
			public ChartControl Chart { get; private set; }

			/// <summary>The View3d window to add/remove objects to/from</summary>
			public View3d.Window Window { get; private set; }

			/// <summary>Add a view3d object to the chart scene</summary>
			public void AddToScene(View3d.Object obj)
			{
				Window.AddObject(obj);
			}

			/// <summary>Remove a single object from the chart scene</summary>
			public void RemoveFromScene(View3d.Object obj)
			{
				Window.RemoveObject(obj);
			}
		}

		/// <summary>Event args for the ChartClicked event</summary>
		public class ChartClickedEventArgs :MouseEventArgs
		{
			public ChartClickedEventArgs(HitTestResult hits, MouseEventArgs e)
				:base(e.Button, e.Clicks, e.X, e.Y, e.Delta)
			{
				HitResult = hits;
				Handled = false;
			}

			/// <summary>Results of a hit test performed at the click location</summary>
			public HitTestResult HitResult { get; private set; }

			/// <summary>Set to true to suppress default chart click behaviour</summary>
			public bool Handled { get; set; }
		}

		/// <summary>Event args for area selection</summary>
		public class ChartAreaSelectEventArgs :EventArgs
		{
			public ChartAreaSelectEventArgs(BBox selection_area)
			{
				SelectionArea = selection_area;
				Handled = false;
			}

			/// <summary>The area (actually volume if you include Z) of the selection</summary>
			public BBox SelectionArea { get; private set; }

			/// <summary>Set to true to suppress default chart click behaviour</summary>
			public bool Handled { get; set; }
		}

		/// <summary>Event args for FindingDefaultRange</summary>
		public class FindingDefaultRangeEventArgs :EventArgs
		{
			public FindingDefaultRangeEventArgs(EAxis axis, RangeF xrange, RangeF yrange)
			{
				Axis = axis;
				XRange = xrange;
				YRange = yrange;
			}

			/// <summary>The axis being ranged</summary>
			public EAxis Axis { get; private set; }

			/// <summary>The XAxis range calculated from known chart elements data</summary>
			public RangeF XRange { get; set; }

			/// <summary>The YAxis range calculated from known chart elements data</summary>
			public RangeF YRange { get; set; }
		}

		/// <summary>Customise context menu event args</summary>
		public class AddUserMenuOptionsEventArgs :EventArgs
		{
			public AddUserMenuOptionsEventArgs(EType type, ContextMenuStrip menu, HitTestResult hit_result)
			{
				Type      = type;
				Menu      = menu;
				HitResult = hit_result;
			}

			/// <summary>The context menu type</summary>
			public EType Type { get; private set; }
			public enum EType
			{
				Chart,
				XAxis,
				YAxis,
			}

			/// <summary>The menu to add menu items to</summary>
			public ContextMenuStrip Menu { get; private set; }

			/// <summary>The hit test result at the click location</summary>
			public HitTestResult HitResult { get; private set; }
		}

		/// <summary>Event args for the post-paint add overlays call</summary>
		public class AddOverlaysOnPaintEventArgs :EventArgs
		{
			public AddOverlaysOnPaintEventArgs(Graphics gfx, ChartDims dims, m4x4 chart_to_client, EZone zone)
			{
				Gfx           = gfx;
				Dims          = dims;
				ChartToClient = chart_to_client;
				Zone          = zone;
			}

			/// <summary>The device context to draw to</summary>
			public Graphics Gfx { get; private set; }

			/// <summary>Layout info about the chart</summary>
			public ChartDims Dims { get; private set; }

			/// <summary>The chart being drawn on</summary>
			public ChartControl Chart { get { return Dims.Chart; } }

			/// <summary>Transform from Chart space to client space</summary>
			public m4x4 ChartToClient { get; private set; }

			/// <summary>The part of the chart being painted</summary>
			public EZone Zone { get; private set; }
		}

		/// <summary>Event args for the auto range event</summary>
		public class AutoRangeEventArgs :EventArgs
		{
			public AutoRangeEventArgs(View3d.ESceneBounds who)
			{
				Who = who;
				ViewBBox = BBox.Reset;
				Handled = false;
			}

			/// <summary>The scene elements to be auto ranged</summary>
			public View3d.ESceneBounds Who { get; private set; }

			/// <summary>The bounding box of the range to view</summary>
			public BBox ViewBBox { get; set; }

			/// <summary>Set to true to indicate that the provided range should be used</summary>
			public bool Handled { get; set; }
		}

		#endregion

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
				if (!Equals(elements[i].Id, elements[i+1].Id)) continue;
				throw new Exception($"Element {elements[i]} is in the Elements collection more than once");
			}

			// All elements in the chart should have their Chart property set to this chart
			foreach (var elem in Elements)
			{
				if (elem.Chart == this) continue;
				throw new Exception($"Element {elem} is in the Elements collection but does not have its Chart property set correctly");
			}

			// The selected collection contains all that is selected and no more
			var selected0 = elements.Where(x => x.Selected).ToArray().Sort(ByGuid);
			var selected1 = Selected.ToArray().Sort(ByGuid);
			if (!selected0.SequenceEqual(selected1, ByGuid))
				throw new Exception("Selected elements collection is inconsistent with the selected state of the elements");

			// The 'ElemIds' should match the Elements collection
			if (ElemIds.Count != Elements.Count)
				throw new Exception("Elements collection and Element Id lookup table don't match");
			foreach (var elem in Elements)
				if (ElemIds.GetOrDefault(elem.Id) != elem)
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
		protected Cmp<Element> ByGuid = Cmp<Element>.From((l,r) => l.Id.CompareTo(r.Id));

		#endregion

		#region Component Designer generated code
		private System.ComponentModel.IContainer components = null;
		private void InitializeComponent()
		{
			components = new System.ComponentModel.Container();
			this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
		}
		#endregion
	}

	#region ChartDataSeries

	/// <summary>Represents a data source that can be added to a chart control</summary>
	public class ChartDataSeries :ChartControl.Element
	{
		private PointStyleTextures m_point_textures;

		public ChartDataSeries(string name, EFormat format, OptionsData options = null, int? capacity = null)
			:this(Guid.NewGuid(), name, format, options, capacity)
		{ }
		public ChartDataSeries(Guid id, string name, EFormat format, OptionsData options = null, int? capacity = null)
			:base(id, m4x4.Identity, name)
		{
			Init();
			Format = format;
			Options = options ?? new OptionsData();
		}
		public ChartDataSeries(XElement node)
			:base(node)
		{
			Init();
			Format = node.Element(nameof(Format)).As<EFormat>();
			Options = node.Element(nameof(Options)).As<OptionsData>();
		}
		private void Init()
		{
			m_data = new List<Pt>();
			Cache = new GfxCache(CreatePiece);
			m_point_textures = new PointStyleTextures();
			m_range_x = RangeF.Invalid;
		}
		protected override void Dispose(bool disposing)
		{
			Cache = null;
			PointSprite = null;
			ThickLineList = null;
			Util.Dispose(ref m_point_textures);
			base.Dispose(disposing);
		}
		public override XElement ToXml(XElement node)
		{
			node.Add2(nameof(Options), Options, false);
			return base.ToXml(node);
		}

		/// <summary>Options for rendering this series</summary>
		public OptionsData Options
		{
			get => m_options;
			set
			{
				if (m_options == value) return;
				if (m_options != null)
				{
					m_options.SettingChange -= HandleSettingChange;
				}
				m_options = value;
				if (m_options != null)
				{
					m_options.SettingChange += HandleSettingChange;
				}

				// Handler
				void HandleSettingChange(object sender, SettingChangeEventArgs e)
				{
					if (e.Before) return;
					switch (e.Key)
					{
						case nameof(Options.PointSize):
						{
							PointSprite = new View3d.Shader(View3d.EStockShader.PointSpritesGS, $"*PointSize {{{Options.PointSize} {Options.PointSize}}} *Depth {{{false}}}");
							break;
						}
						case nameof(Options.LineWidth):
						{
							ThickLineList = new View3d.Shader(View3d.EStockShader.ThickLineListGS, $"*LineWidth {{{Options.LineWidth}}}");
							break;
						}
					}
				}
			}
		}
		private OptionsData m_options = null!;

		/// <summary>Gain access to the underlying data</summary>
		public LockData Lock()
		{
			return new LockData(this);
		}
		private List<Pt> m_data;

		/// <summary>Read the number of data points in the series. (Not synchronised by 'Lock()')</summary>
		public int SampleCount
		{
			get { return m_data.Count; }
		}

		/// <summary>The format of the plot data</summary>
		public EFormat Format { get; set; }

		/// <summary>Return the X-Axis range of the data</summary>
		public RangeF RangeX
		{
			get
			{
				if (m_range_x == RangeF.Invalid)
				{
					using (Lock())
					{
						// 'Transform' only applies to the graphics
						if (m_data.Count == 0) return RangeF.Invalid;
						var beg = m_data.Front();
						var end = m_data.Back();
						m_range_x = new RangeF(beg.xf, end.xf);
					}
				}
				return m_range_x;
			}
		}
		private RangeF m_range_x;

		/// <summary>Cause all graphics models to be recreated</summary>
		public void FlushCachedGraphics()
		{
			Cache.Invalidate();
			m_range_x = RangeF.Invalid;
		}

		/// <summary>Cause graphics model that intersect 'x_range' to be recreated</summary>
		public void FlushCachedGraphics(RangeF x_range)
		{
			Cache.Invalidate(x_range);
			if (m_range_x != RangeF.Invalid)
				m_range_x.Grow(x_range);
		}
		
		/// <summary>Update the graphics for this indicator and add it to the scene</summary>
		protected override void UpdateSceneCore(View3d.Window window)
		{
			// Remove all series data graphics
			window.RemoveObjects(new[] { Id }, 1, 0);

			// Add each graphics piece over the range
			if (Visible)
			{
				// If there is no data, then there's no graphics
				var range_x = RangeX;
				if (range_x == RangeF.Invalid)
					return;

				// Get the range required for display
				var range = new RangeF(
					Math.Max(Chart.XAxis.Min, range_x.Beg),
					Math.Min(Chart.XAxis.Max, range_x.End));

				// Add each graphics piece over the range
				foreach (var piece in Cache.Get(range))
				{
					if (piece.Gfx == null) continue;
					window.AddObject(piece.Gfx);
				}
			}
		}

		/// <summary>Generate a piece of the graphics for 'x'</summary>
		private GfxPiece CreatePiece(double x, RangeF missing)
		{
			Debug.Assert(missing.Contains(x));
			using (Lock())
			{
				// Find the nearest point in the data to 'x'
				var idx = m_data.BinarySearch(pt => pt.xf.CompareTo(x), find_insert_position: true);

				// Convert 'missing' to an index range within the data
				var idx_missing = new RangeI(
					m_data.BinarySearch(pt => pt.xf.CompareTo(missing.Beg), find_insert_position:true),
					m_data.BinarySearch(pt => pt.xf.CompareTo(missing.End), find_insert_position:true));

				// Limit the size of 'idx_missing' to the block size
				const int PieceBlockSize = 4096;
				var idx_range = new RangeI(
					Math.Max(idx_missing.Beg, idx - PieceBlockSize),
					Math.Min(idx_missing.End, idx + PieceBlockSize));

				// Create graphics over the data range 'idx_range'
				//todo: this isn't right... need to handle this function returning 'failed to create piece'
				switch (Options.PlotType)
				{
				default: throw new Exception($"Unsupported plot type: {Options.PlotType}");
				case EPlotType.Point:
					{
						return idx_range.Size > 0
							? CreatePointPlot(idx_range)
							: new GfxPiece(null, missing);
					}
				case EPlotType.Line:
					{
						return idx_range.Size > 1
							? CreateLinePlot(idx_range)
							: new GfxPiece(null, missing);
					}
				case EPlotType.StepLine:
					{
						return idx_range.Size > 1
							? CreateStepLinePlot(idx_range)
							: new GfxPiece(null, missing);
					}
				case EPlotType.Bar:
					{
						return idx_range.Size > 1
							? CreateBarPlot(idx_range)
							: new GfxPiece(null, missing);
					}
				}
			}
		}

		/// <summary>Create a point cloud plot</summary>
		private GfxPiece CreatePointPlot(RangeI idx_range)
		{
			var n = idx_range.Sizei;

			// Resize the geometry buffers
			m_vbuf.Resize(n);
			m_ibuf.Resize(n);
			m_nbuf.Resize(1);

			// Create the vertex/index data
			var col = Options.Colour;
			var x_range = RangeF.Invalid;
			for (int i = 0, iend = n; i != iend; ++i)
			{
				var pt = m_data[i + idx_range.Begi];
				m_vbuf[i] = new View3d.Vertex(new v4((float)pt.xf, (float)pt.yf, 0f, 1f), col);
				m_ibuf[i] = (ushort)i;
				x_range.Grow(pt.xf);
			}

			// Create a nugget for the points using the sprite shader
			m_nbuf[0] = new View3d.Nugget(View3d.ETopo.PointList, View3d.EGeom.Vert|View3d.EGeom.Colr|View3d.EGeom.Tex0, 
				tex_diffuse: m_point_textures[Options.PointStyle],
				sam_diffuse: View3d.Sampler.FromStock(View3d.EStockSampler.LinearClamp),
				shaders: new[] { new View3d.Nugget.Shader(View3d.ERenderStep.ForwardRender, PointSprite) });

			// Create the graphics
			var gfx = new View3d.Object($"{Name}-[{idx_range.Beg},{idx_range.End})", 0xFFFFFFFF, m_vbuf.Count, m_ibuf.Count, m_nbuf.Count, m_vbuf.ToArray(), m_ibuf.ToArray(), m_nbuf.ToArray(), Id);
			return new GfxPiece(gfx, x_range);
		}

		/// <summary>Create a line plot</summary>
		private GfxPiece CreateLinePlot(RangeI idx_range)
		{
			var n = idx_range.Sizei;
			if (n == 0)
				throw new ArgumentException($"{nameof(ChartControl)}.{nameof(CreateLinePlot)} Index range must not be empty");

			// Resize the geometry buffers
			m_vbuf.Resize(n);
			m_ibuf.Resize(n);
			m_nbuf.Resize(1 + (Options.PointsOnLinePlot ? 1 : 0));

			// Create the vertex/index data
			int vert = 0, indx = 0;
			var col = Options.Colour;
			var x_range = RangeF.Invalid;
			for (int i = 0, iend = Math.Min(n, m_data.Count - idx_range.Begi); i != iend; ++i)
			{
				var j = i + idx_range.Begi;
				var pt = m_data[j];

				var v = vert;
				m_vbuf[vert++] = new View3d.Vertex(new v4((float)pt.xf, (float)pt.yf, 0f, 1f), col);
				m_ibuf[indx++] = (ushort)(v);

				x_range.Grow(pt.xf);
			}

			// Create a nugget for the list strip using the thick line shader
			m_nbuf[0] = new View3d.Nugget(View3d.ETopo.LineStrip, View3d.EGeom.Vert|View3d.EGeom.Colr, 
				shaders: new[] { new View3d.Nugget.Shader(View3d.ERenderStep.ForwardRender, ThickLineList) });

			// Create a nugget for the points (if visible)
			if (Options.PointsOnLinePlot)
			{
				m_nbuf[1] = new View3d.Nugget(View3d.ETopo.PointList, View3d.EGeom.Vert|View3d.EGeom.Colr|View3d.EGeom.Tex0, flags:View3d.ENuggetFlag.RangesCanOverlap, 
					tex_diffuse: m_point_textures[Options.PointStyle],
					sam_diffuse: View3d.Sampler.FromStock(View3d.EStockSampler.LinearClamp),
					shaders: new[] { new View3d.Nugget.Shader(View3d.ERenderStep.ForwardRender, PointSprite) });
			}

			// Create the graphics
			var gfx = new View3d.Object($"{Name}-[{idx_range.Beg},{idx_range.End})", 0xFFFFFFFF, m_vbuf.Count, m_ibuf.Count, m_nbuf.Count, m_vbuf.ToArray(), m_ibuf.ToArray(), m_nbuf.ToArray(), Id);
			return new GfxPiece(gfx, x_range);
		}

		/// <summary>Create a step line plot</summary>
		private GfxPiece CreateStepLinePlot(RangeI idx_range)
		{
			var n = idx_range.Sizei;

			// Resize the geometry buffers
			m_vbuf.Resize(2*n);
			m_ibuf.Resize(2*n + (Options.PointsOnLinePlot ? n : 0));
			m_nbuf.Resize(1 + (Options.PointsOnLinePlot ? 1 : 0));

			// Create the vertex/index data
			int vert = 0, indx = 0;
			var col = Options.Colour;
			var x_range = RangeF.Invalid;
			for (int i = 0, iend = Math.Min(n + 1, m_data.Count - idx_range.Begi); i != iend; ++i)
			{
				// Get the point and the next point
				var j = i + idx_range.Begi;
				var pt = m_data[j];
				var pt_r = j+1 != m_data.Count ? m_data[j+1] : pt;

				var v = vert;
				m_vbuf[vert++] = new View3d.Vertex(new v4((float)pt.xf  , (float)pt.yf, 0f, 1f), col);
				m_vbuf[vert++] = new View3d.Vertex(new v4((float)pt_r.xf, (float)pt.yf, 0f, 1f), col);
				m_ibuf[indx++] = (ushort)(v + 0);
				m_ibuf[indx++] = (ushort)(v + 1);

				x_range.Grow(pt.xf);
			}

			// Create a nugget for the list strip using the thick line shader
			m_nbuf[0] = new View3d.Nugget(View3d.ETopo.LineStrip, View3d.EGeom.Vert|View3d.EGeom.Colr, 0, vert, 0, indx, 
				shaders: new[] { new View3d.Nugget.Shader(View3d.ERenderStep.ForwardRender, ThickLineList) });

			// Create a nugget for the points (if visible)
			if (Options.PointsOnLinePlot)
			{
				// Add indices for the points
				var i0 = indx;
				for (int i = 0, iend = n; i != iend; ++i)
					m_ibuf[indx++] = (ushort)(i * 2);
			
				m_nbuf[1] = new View3d.Nugget(View3d.ETopo.PointList, View3d.EGeom.Vert|View3d.EGeom.Colr|View3d.EGeom.Tex0, 0, vert, i0, indx, 
					tex_diffuse: m_point_textures[Options.PointStyle],
					sam_diffuse: View3d.Sampler.FromStock(View3d.EStockSampler.LinearClamp),
					shaders: new[] { new View3d.Nugget.Shader(View3d.ERenderStep.ForwardRender, PointSprite) });
			}

			// Create the graphics
			var gfx = new View3d.Object($"{Name}-[{idx_range.Beg},{idx_range.End})", 0xFFFFFFFF, m_vbuf.Count, m_ibuf.Count, m_nbuf.Count, m_vbuf.ToArray(), m_ibuf.ToArray(), m_nbuf.ToArray(), Id);
			return new GfxPiece(gfx, x_range);
		}

		/// <summary>Create a bar graph</summary>
		private GfxPiece CreateBarPlot(RangeI idx_range)
		{
			var n = idx_range.Sizei;

			// Resize the geometry buffers
			m_vbuf.Resize(4 * n);
			m_ibuf.Resize(6 * n);
			m_nbuf.Resize(1);

			// Create the vertex/index data
			int vert = 0, indx = 0;
			var col = Options.Colour;
			var width = Options.BarWidth;
			var x_range = RangeF.Invalid;
			for (int i = 0, iend = n; i != iend; ++i)
			{
				// Get the points on either side of 'i'
				var j = i + idx_range.Begi;
				var pt_l = j != 0 ? m_data[j-1] : null;
				var pt   = m_data[j];
				var pt_r = j+1 != m_data.Count ? m_data[j+1] : null;

				// Get the distance to the left and right of 'pt.x'
				var l = pt_l != null ? 0.5f * width * (pt.xf - pt_l.xf) : 0f;
				var r = pt_r != null ? 0.5f * width * (pt_r.xf - pt.xf) : 0f;
				if (l == 0f) l = r;
				if (r == 0f) r = l;

				var v = vert;
				m_vbuf[vert++] = new View3d.Vertex(new v4((float)(pt.xf + r), (float)(pt.yf >= 0f ? pt.yf : 0f), 0f, 1f), col);
				m_vbuf[vert++] = new View3d.Vertex(new v4((float)(pt.xf - l), (float)(pt.yf >= 0f ? pt.yf : 0f), 0f, 1f), col);
				m_vbuf[vert++] = new View3d.Vertex(new v4((float)(pt.xf - l), (float)(pt.yf >= 0f ? 0f : pt.yf), 0f, 1f), col);
				m_vbuf[vert++] = new View3d.Vertex(new v4((float)(pt.xf + r), (float)(pt.yf >= 0f ? 0f : pt.yf), 0f, 1f), col);

				m_ibuf[indx++] = (ushort)(v + 0);
				m_ibuf[indx++] = (ushort)(v + 1);
				m_ibuf[indx++] = (ushort)(v + 2);
				m_ibuf[indx++] = (ushort)(v + 2);
				m_ibuf[indx++] = (ushort)(v + 3);
				m_ibuf[indx++] = (ushort)(v + 0);

				x_range.Grow(pt.xf);
			}

			// Create a nugget for the triangle list
			{
				m_nbuf[0] = new View3d.Nugget(View3d.ETopo.TriList, View3d.EGeom.Vert|View3d.EGeom.Colr);
			}

			// Create the graphics
			var gfx = new View3d.Object($"{Name}-[{idx_range.Beg},{idx_range.End})", 0xFFFFFFFF, m_vbuf.Count, m_ibuf.Count, m_nbuf.Count, m_vbuf.ToArray(), m_ibuf.ToArray(), m_nbuf.ToArray(), Id);
			return new GfxPiece(gfx, x_range);
		}

		/// <summary>A cache of graphics pieces for this data series</summary>
		private GfxCache Cache
		{
			get;
			set
			{
				if (field == value) return;
				Util.Dispose(ref field!);
				field = value;
			}
		} = null!;

		/// <summary>Point sprite shader</summary>
		private View3d.Shader PointSprite
		{
			get;
			set
			{
				if (field == value) return;
				Util.Dispose(ref field!);
				field = value;
			}
		} = null!;

		/// <summary>Thick Line-List shader</summary>
		private View3d.Shader ThickLineList
		{
			get;
			set
			{
				if (field == value) return;
				Util.Dispose(ref field!);
				field = value;
			}
		} = null!;

		/// <summary>ToString</summary>
		public override string ToString()
		{
			return $"{Name} count={m_data.Count}";
		}

		/// <summary>Supported plot types</summary>
		public enum EPlotType
		{
			Point,
			Line,
			StepLine,
			Bar,
		}

		/// <summary>Point styles</summary>
		public enum EPointStyle
		{
			Square,
			Circle,
			Triangle,
		}

		/// <summary>Formats for the plot data</summary>
		[Flags] public enum EFormat
		{
			XIntg = 1 << 0,
			XReal = 1 << 1,
			YIntg = 1 << 2,
			YReal = 1 << 3,

			XIntgYIntg = XIntg | YIntg,
			XRealYReal = XReal | YReal,
		}

		/// <summary>A cache of graphics objects that span the X-Axis</summary>
		public sealed class GfxCache :IDisposable
		{
			// Notes:
			// - This cache is intended to be used by other ChartDataSeries-like classes.
			//   If provides the functionality of breaking a data series up into pieces
			//   so that the limit of 64K indices is not exceeded.
			public GfxCache(CreatePieceHandler handler)
			{
				Pieces = new List<GfxPiece>();
				CreatePiece = handler;
			}
			public void Dispose()
			{
				Pieces = null;
			}

			/// <summary>Reset the cache</summary>
			public void Invalidate()
			{
				Util.DisposeRange(Pieces);
				Pieces.Clear();
			}
			public void Invalidate(RangeF x_range)
			{
				var beg = Pieces.BinarySearch(p => p.Range.CompareTo(x_range.Beg), find_insert_position:true);
				var end = Pieces.BinarySearch(p => p.Range.CompareTo(x_range.End), find_insert_position:true);
				Util.DisposeRange(Pieces, beg, end - beg);
				Pieces.RemoveRange(beg, end - beg);
			}

			/// <summary>The collection of cached graphics models</summary>
			private List<GfxPiece> Pieces
			{
				get;
				set
				{
					if (field == value) return;
					Util.DisposeRange(field);
					field = value;
				}
			}

			/// <summary>Get the series data graphics that spans the given x range</summary>
			public IEnumerable<GfxPiece> Get(RangeF range)
			{
				// Return each graphics piece over the range
				for (var x = range.Beg; x < range.End;)
				{
					var piece = CacheGet(x);
					yield return piece;
					Debug.Assert(piece.Range.End > x);
					x = piece.Range.End;
				}
			}

			/// <summary>Return the graphics piece that spans 'x'</summary>
			public GfxPiece CacheGet(double x)
			{
				// Search the cache for the model that spans 'x'
				var idx = Pieces.BinarySearch(p => p.Range.CompareTo(x));
				if (idx < 0)
				{
					idx = ~idx;

					// Get the X-range that is not cached
					var missing = new RangeF(
						idx != 0            ? Pieces[idx-1].Range.End : double.MinValue,
						idx != Pieces.Count ? Pieces[idx  ].Range.Beg : double.MaxValue);

					// There is no cached graphics for 'x', create it now
					var piece = CreatePiece(x, missing);
					Pieces.Insert(idx, piece);
				}
				return Pieces[idx];
			}

			/// <summary>The handler for providing pieces of the graphics</summary>
			public CreatePieceHandler CreatePiece { get; set; }

			/// <summary>
			/// Implementers of this handler should return a graphics object that spans
			/// some region about 'x', clipped by 'missing'. The returned graphics piece
			/// should contain the x-range that the graphics represents.</summary>
			public delegate GfxPiece CreatePieceHandler(double x, RangeF missing);
		}

		/// <summary>Graphics for a part of the series data</summary>
		public sealed class GfxPiece :IDisposable
		{
			public GfxPiece(View3d.Object gfx, RangeF range)
			{
				Gfx = gfx;
				Range = range;
			}
			public void Dispose()
			{
				Gfx = null;
			}

			/// <summary>The model for the piece of the series data graphics</summary>
			public View3d.Object Gfx
			{
				get { return m_gfx; }
				private set
				{
					if (m_gfx == value) return;
					Util.Dispose(ref m_gfx);
					m_gfx = value;
				}
			}
			private View3d.Object m_gfx;

			/// <summary>The X-Axis span covered by this piece</summary>
			public RangeF Range { get; private set; }
		}

		/// <summary>A single point in the data series. A class so that it can be sub-classed</summary>
		[DebuggerDisplay("{Description,nq}")]
		[StructLayout(LayoutKind.Explicit, Pack = 1)]
		public class Pt
		{
			// Notes:
			// - The format of this data is given by 'Format' in the 'ChartDataSeries' owner.
			// - Not IComparible because we don't know whether to compare 'xf' or 'xi'.
			//   Use the static 'CompareX?' functions below

			public Pt(double x_, double y_)
			{
				xf = x_;
				yf = y_;
			}
			public Pt(double x_, long y_)
			{
				xf = x_;
				yi = y_;
			}
			public Pt(long x_, double y_)
			{
				xi = x_;
				yf = y_;
			}
			public Pt(long x_, long y_)
			{
				xi = x_;
				yi = y_;
			}
			public Pt(Pt rhs)
			{
				xf = rhs.xf;
				yf = rhs.yf;
			}

			[FieldOffset(0)] public double xf;
			[FieldOffset(8)] public double yf;
			[FieldOffset(0)] public long xi;
			[FieldOffset(8)] public long yi;

			public static implicit operator Pt(v2 pt) { return new Pt(pt.x, pt.y); }
			public static implicit operator v2(Pt pt) { return new v2((float)pt.xf, (float)pt.yf); }
			public static implicit operator Pt(PointF pt) { return new Pt(pt.X, pt.Y); }
			public static implicit operator PointF(Pt pt) { return new PointF((float)pt.xf, (float)pt.yf); }

			/// <summary>Sorting predicate on X</summary>
			public static IComparer<Pt> CompareXf
			{
				get { return Cmp<Pt>.From((l,r) => l.xf.CompareTo(r.xf)); }
			}
			public static IComparer<Pt> CompareXi
			{
				get { return Cmp<Pt>.From((l,r) => l.xi.CompareTo(r.xi)); }
			}

			/// <summary>Guess at whether double or long values are used</summary>
			private string Description
			{
				get { return $"{(Math.Abs(xf) < 1e-200 ? xi.ToString() : xf.ToString())} {(Math.Abs(yf) < 1e-200 ? yi.ToString() : yf.ToString())}"; }
			}
		}

		/// <summary>RAII object for synchronising access to the underlying data</summary>
		public sealed class LockData : ICollection<Pt>, IDisposable
		{
			private RangeF m_changed_data_range;
			public LockData(ChartDataSeries owner)
			{
				Owner = owner;
				m_changed_data_range = RangeF.Invalid;
				Monitor.Enter(Data);
			}
			public void Dispose()
			{
				if (m_changed_data_range == RangeF.Max)
					Owner.FlushCachedGraphics();
				else if (m_changed_data_range != RangeF.Invalid)
					Owner.FlushCachedGraphics(m_changed_data_range);

				Monitor.Exit(Data);
			}

			/// <summary>The data series that this lock is on</summary>
			public ChartDataSeries Owner { get; private set; }

			/// <summary>The number of elements in the data series</summary>
			public int Count
			{
				get { return Data.Count; }
			}

			/// <summary>Reset the collection</summary>
			public void Clear()
			{
				m_changed_data_range = RangeF.Max;
				Data.Clear();
			}

			/// <summary>Add a datum point</summary>
			public Pt Add(Pt point)
			{
				m_changed_data_range.Grow(Format.HasFlag(EFormat.XIntg) ? point.xi : point.xf);
				Data.Add(point);
				return point;
			}

			/// <summary>Remove a datum point</summary>
			public bool Remove(Pt item)
			{
				m_changed_data_range = RangeF.Max;
				return Data.Remove(item);
			}

			/// <summary>Insert a datum point</summary>
			public Pt Insert(int index, Pt point)
			{
				m_changed_data_range.Grow(Format.HasFlag(EFormat.XIntg) ? point.xi : point.xf);
				Data.Insert(index, point);
				return point;
			}

			/// <summary>The data series</summary>
			public Pt this[int idx]
			{
				get { return Data[idx]; }
				set
				{
					m_changed_data_range.Grow(Format.HasFlag(EFormat.XIntg) ? value.xi : value.xf);
					Data[idx] = value;
				}
			}

			/// <summary>Search for a point</summary>
			public bool Contains(Pt point)
			{
				return Data.Contains(point);
			}

			/// <summary>Sort the data series on X values (F = doubles, I = longs)</summary>
			public void SortF()
			{
				m_changed_data_range = RangeF.Max;
				Data.Sort(Pt.CompareXf);
			}
			public void SortI()
			{
				m_changed_data_range = RangeF.Max;
				Data.Sort(Pt.CompareXi);
			}

			///<summary>
			/// Return the range of indices that need to be considered when plotting from 'xmin' to 'xmax'
			/// Note: in general, this range should include one point to the left of 'xmin' and one to the right
			/// of 'xmax' so that line graphs plot a line up to the border of the plot area</summary>
			public void IndexRange(double xmin, double xmax, out int imin, out int imax)
			{
				var lwr = new Pt(xmin, 0.0);
				var upr = new Pt(xmax, 0.0);

				imin = Data.BinarySearch(0, Count, lwr, Pt.CompareXf);
				if (imin < 0) imin = ~imin;
				if (imin != 0) --imin;

				imax = Data.BinarySearch(imin, Count - imin, upr, Pt.CompareXf);
				if (imax < 0) imax = ~imax;
				if (imax != Count) ++imax;
			}

			/// <summary>Enumerable access to the data given an index range</summary>
			public IEnumerable<Pt> Values(int i0 = 0, int i1 = int.MaxValue)
			{
				Debug.Assert(i0 >= 0 && i0 <= Count);
				Debug.Assert(i1 >= i0);
				for (int i = i0, iend = Math.Min(i1, Count); i != iend; ++i)
				{
					var pt = this[i];
					yield return pt;
				}
			}

			/// <summary>
			/// Returns the range of series data to consider when plotting from 'xmin' to 'xmax'
			/// Note: in general, this range should include one point to the left of 'xmin' and one to
			/// the right of 'xmax' so that line graphs plot a line up to the border of the plot area</summary>
			public IEnumerable<Pt> Values(double xmin, double xmax)
			{
				IndexRange(xmin, xmax, out var i0, out var i1);
				return Values(i0, i1);
			}

			/// <summary>
			/// Returns the range of indices to consider when plotting from 'xmin' to 'xmax'
			/// Note: in general, this range should include one point to the left of 'xmin' and one to
			/// the right of 'xmax' so that line graphs plot a line up to the border of the plot area</summary>
			public IEnumerable<int> Indices(double xmin, double xmax)
			{
				IndexRange(xmin, xmax, out var i0, out var i1);
				return Enumerable.Range(i0, i1 - i0);
			}

			/// <summary>The data format</summary>
			private EFormat Format => Owner.Format;

			/// <summary>Private so that all 'Add' are seen</summary>
			private List<Pt> Data => Owner.m_data;

			#region ICollection<Pt>
			void ICollection<Pt>.Add(Pt point) => Add(point);
			void ICollection<Pt>.CopyTo(Pt[] array, int arrayIndex) => Owner.m_data.CopyTo(array, arrayIndex);
			public IEnumerator<Pt> GetEnumerator() => Owner.m_data.GetEnumerator();
			IEnumerator IEnumerable.GetEnumerator() => Owner.m_data.GetEnumerator();
			bool ICollection<Pt>.IsReadOnly => false;
			#endregion
		}

		/// <summary>Plot colour generator</summary>
		public static Color GenerateColour(int i)
		{
			return m_colours[i % m_colours.Length];
		}
		private static Color[] m_colours =
		{
			Color.Black     ,
			Color.Blue      , Color.Red       , Color.Green      ,
			Color.DarkBlue  , Color.DarkRed   , Color.DarkGreen  ,
			Color.Purple    , Color.Turquoise , Color.Magenta    ,
			Color.Orange    , Color.Yellow    ,
			Color.LightBlue , Color.LightSalmon , Color.LightGreen ,
		};

		/// <summary></summary>
		private class PointStyleTextures :IDisposable
		{
			private readonly Dictionary<EPointStyle, View3d.Texture> m_map;
			public PointStyleTextures()
			{
				m_map = new Dictionary<EPointStyle, View3d.Texture>();
			}
			public void Dispose()
			{
				foreach (var tex in m_map.Values)
					tex.Dispose();

				m_map.Clear();
			}
			public View3d.Texture this[EPointStyle style]
			{
				get
				{
					switch (style)
					{
					default: throw new Exception($"Unknown point style: {style}");
					case EPointStyle.Square:
						{
							// No texture needed, squares a flat colour geometry
							return null;
						}
					case EPointStyle.Circle:
						{
							// A circle texture
							return m_map.GetOrAdd(style, s =>
							{
								var sz = 32;
								var tex = new View3d.Texture(sz,sz);
								using (var surf = tex.LockSurface(true))
								{
									surf.Gfx.Clear(Color.Transparent);
									surf.Gfx.FillEllipse(Brushes.White, new RectangleF(0,0,sz,sz));
								}
								return tex;
							});
						}
					case EPointStyle.Triangle:
						{
							// An equilateral triangle within a square texture
							return m_map.GetOrAdd(style, s =>
							{
								var sz = 128;
								var h0 = 0.5f * sz * (float)Math.Tan(Math_.DegreesToRadians(60));
								var h1 = 0.5f * (sz - h0);
								var tex = new View3d.Texture(sz,sz);
								using (var surf = tex.LockSurface(true))
								{
									surf.Gfx.Clear(Color.Transparent);
									surf.Gfx.FillPolygon(Brushes.White, new []{ new PointF(sz, h1), new PointF(0, h1), new PointF(0.5f*sz, h0+h1) });
								}
								return tex;
							});
						}
					}
				}
			}
		}

		/// <summary>Rendering options for this data series</summary>
		public class OptionsData :SettingsSet<OptionsData>
		{
			public OptionsData()
			{
				Colour           = Color.Black;
				PlotType         = EPlotType.Point;
				PointStyle       = EPointStyle.Square;
				PointSize        = 10f;
				LineWidth        = 5f;
				PointsOnLinePlot = true;
				BarWidth         = 0.8f;
				//Visible        = true;
				//DrawData       = true;
				//DrawErrorBars  = false;
				//PlotType       = EPlotType.Line;
				//PointColour    = Color.FromArgb(0xff, 0x80, 0, 0);
				//PointSize      = 5f;
				//LineColour     = Color.FromArgb(0xff, 0, 0, 0);
				//LineWidth      = 1f;
				//BarColour      = Color.FromArgb(0xff, 0x80, 0, 0);
				//ErrorBarColour = Color.FromArgb(0x80, 0xff, 0, 0);
				//DrawMovingAvr  = false;
				//MAWindowSize   = 10;
				//MALineColour   = Color.FromArgb(0xff, 0, 0, 0xFF);
				//MALineWidth    = 3f;
			}
			public OptionsData(XElement node)
				:base(node)
			{ }
			public OptionsData(OptionsData rhs)
			{
				Colour           = rhs.Colour;
				PlotType         = rhs.PlotType;
				PointStyle       = rhs.PointStyle;
				PointSize        = rhs.PointSize;
				LineWidth        = rhs.LineWidth;
				PointsOnLinePlot = rhs.PointsOnLinePlot;
				BarWidth         = rhs.BarWidth;
			}

			/// <summary>The base colour for this plot</summary>
			public Colour32 Colour
			{
				get { return get<Colour32>(nameof(Colour)); }
				set { set(nameof(Colour), value); }
			}

			/// <summary>The plot type for the series</summary>
			public EPlotType PlotType
			{
				get { return get<EPlotType>(nameof(PlotType)); }
				set { set(nameof(PlotType), value); }
			}

			/// <summary>The style of points to draw</summary>
			public EPointStyle PointStyle
			{
				get { return get<EPointStyle>(nameof(PointStyle)); }
				set { set(nameof(PointStyle), value); }
			}

			/// <summary>The size of point data graphics</summary>
			public float PointSize
			{
				get { return get<float>(nameof(PointSize)); }
				set { set(nameof(PointSize), value); }
			}

			/// <summary>The width of the line in line plots</summary>
			public float LineWidth
			{
				get { return get<float>(nameof(LineWidth)); }
				set { set(nameof(LineWidth), value); }
			}

			/// <summary>True if point graphics should be drawn on line plots</summary>
			public bool PointsOnLinePlot
			{
				get { return get<bool>(nameof(PointsOnLinePlot)); }
				set { set(nameof(PointsOnLinePlot), value); }
			}

			/// <summary>The normalised bar width in bar plots</summary>
			public float BarWidth
			{
				get { return get<float>(nameof(BarWidth)); }
				set { set(nameof(BarWidth), value); }
			}

			//public bool       Visible        { get; set; }
			//public bool       DrawData       { get; set; }
			//public bool       DrawErrorBars  { get; set; }
			//public EPlotType  PlotType       { get; set; }
			//public Color      BarColour      { get; set; }
			//public float      BarWidth       { get; set; }
			//public Color      ErrorBarColour { get; set; }
			//public bool       DrawMovingAvr  { get; set; }
			//public int        MAWindowSize   { get; set; }
			//public Color      MALineColour   { get; set; }
			//public float      MALineWidth    { get; set; }
		}
	}

	/// <summary>A Legend element for a collection of ChartDataSeries</summary>
	public class ChartDataLegend :ChartControl.Element
	{
		private List<ChartDataSeries> m_series;

		public ChartDataLegend()
			:this(Guid.NewGuid())
		{}
		public ChartDataLegend(Guid id)
			:base(id, m4x4.Identity, "Legend")
		{
			m_bk_colour = 0xFFFFFFFF;
			m_padding = new Padding(5);
			m_anchor = new v2(+1f, +1f);
			m_font = new Font("tahoma", 14f, FontStyle.Regular, GraphicsUnit.Point);
			m_series = new List<ChartDataSeries>();
			PositionXY = new v2(1f, 1f);
			ScreenSpace = true;
		}
		protected override void Dispose(bool disposing)
		{
			Gfx = null;
			base.Dispose(disposing);
		}
		protected override void SetChartCore(ChartControl chart)
		{
			m_series.Clear();
			base.SetChartCore(chart);
		}

		/// <summary>Legend graphics</summary>
		public View3d.Object Gfx
		{
			get { return m_gfx; }
			private set
			{
				if (m_gfx == value) return;
				Util.Dispose(ref m_gfx);
				m_gfx = value;
			}
		}
		private View3d.Object m_gfx;

		/// <summary>The background colour for the legend box</summary>
		public Colour32 BackColour
		{
			get { return m_bk_colour; }
			set { SetProp(ref m_bk_colour, value, nameof(BackColour), true, true); }
		}
		private Colour32 m_bk_colour;

		/// <summary>Padding between the legend text and the border</summary>
		public Padding Padding
		{
			get { return m_padding; }
			set { SetProp(ref m_padding, value, nameof(Padding), true, true); }
		}
		private Padding m_padding;

		/// <summary>The origin position of the legend graphics</summary>
		public v2 Anchor
		{
			get { return m_anchor; }
			set { SetProp(ref m_anchor, value, nameof(Anchor), true, true); }
		}
		private v2 m_anchor;

		/// <summary>Return the font to use for the legend</summary>
		public Font Font
		{
			get { return m_font; }
			set { SetProp(ref m_font, value, nameof(Font), true, false); }
		}
		private Font m_font;

		/// <summary>Generate the legend graphics</summary>
		protected override void UpdateGfxCore()
		{
			base.UpdateGfxCore();

			Gfx = null;
			if (Chart == null)
				return;

			// Get the ChartDataSeries objects on the chart
			m_series = Chart.Elements.OfType<ChartDataSeries>().ToList();
			if (m_series.Count != 0)
			{
				// Create the legend graphics object
				var sb = new StringBuilder();
				sb.Append(
					$"*Text legend {{\n"+
					$"*ScreenSpace\n"+
					$"*BackColour {{{BackColour}}}\n"+
					$"*Anchor {{{Anchor.x} {Anchor.y}}}\n"+
					$"*Padding{{{Padding.Left} {Padding.Top} {Padding.Right} {Padding.Bottom}}}\n"+
					$"*Font{{ *Name{{\"{Font.Name}\"}} *Size{{{Font.SizeInPoints}}} }}\n");

				bool newline = false;
				foreach (var s in m_series)
				{
					if (newline) sb.Append("*NewLine\n");
					sb.Append($"*Font {{*Colour {{{(s.Visible ? s.Options.Colour : Colour32.Gray)}}} }}\n");
					sb.Append($"\"{s.Name}\"\n");
					newline = true;
				}

				sb.Append("}");
				Gfx = new View3d.Object(sb.ToString(), false, Id);
			}
		}

		/// <summary>Add/Remove graphics from the scene</summary>
		protected override void UpdateSceneCore(View3d.Window window)
		{
			base.UpdateSceneCore(window);
			if (Gfx != null)
			{
				Gfx.O2P = Position;
				if (Visible)
					window.AddObject(Gfx);
				else
					window.RemoveObject(Gfx);
			}
		}

		/// <summary>Perform a hit test on this object. Returns null for no hit. 'point' is in client space because typically hit testing uses pixel tolerances</summary>
		public override ChartControl.HitTestResult.Hit HitTest(PointF chart_point, Point client_point, Keys modifier_keys, View3d.Camera cam)
		{
			if (Gfx == null || !Gfx.Visible)
				return null;
			
			return null;
		// todo:
		//	// Get the area covered by the legend in screen space
		//	var loc = Chart.NSSToClient(PositionXY.ToPointF());
		//	var area = Gfx.BBoxMS(false).ToRectXY();
		//
		//
		//
		//	area = area.Shifted(loc);
		//
		//
		//
		//
		//	// If this is a hit
		//	if (!area.Contains(client_point))
		//		return null;
		//
		//	// Get the hit point in legend space
		//	var elem_point = Drawing_.Subtract(client_point, area.TopLeft()).ToPointF();
		//
		//	// Determine which series was hit
		//	var idx = (int)Math_.Frac(Padding.Top, elem_point.Y, area.Height - Padding.Bottom) * m_series.Count;
		//	if (idx < 0 || idx >= m_series.Count)
		//		return null;
		//
		//	// Return hit data
		//	var hit = new ChartControl.HitTestResult.Hit(this, elem_point, m_series[idx]);
		//	return hit;
		}
	}

	#endregion
}
