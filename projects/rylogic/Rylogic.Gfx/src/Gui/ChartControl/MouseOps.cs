using System;
using System.Linq;
using System.Windows.Input;
using Rylogic.Extn;
using Rylogic.Gfx;
using Rylogic.Maths;
using Rylogic.Utility;
using Rylogic.Windows.Extn;

namespace Rylogic.Gui.WPF
{
	public partial class ChartControl
	{
		/// <summary>Manages per-button mouse operations</summary>
		public class MouseOps :IDisposable
		{
			public MouseOps()
			{
				Pending = new PendingOps(this);
			}
			public void Dispose()
			{
				Dispose(true);
				GC.SuppressFinalize(this);
			}
			protected virtual void Dispose(bool _)
			{
				Active = null;
				Pending.Dispose();
			}

			/// <summary>The currently active mouse op</summary>
			public MouseOp? Active
			{
				get => m_active;
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
			private MouseOp? m_active;

			/// <summary>The next mouse operation for each mouse button</summary>
			public PendingOps Pending { get; }

			/// <summary>Start/End the next mouse op for button 'idx'</summary>
			public void BeginOp(MouseButton btn)
			{
				Active = Pending[btn];
				Pending[btn] = null;
			}
			public void EndOp(MouseButton btn)
			{
				Active = null;

				// If the next op starts immediately, begin it now
				if (Pending[btn] is MouseOp op && !op.StartOnMouseDown)
					BeginOp(btn);
			}

			/// <summary>Handle a mouse op being disposed externally</summary>
			private void HandleMouseOpDisposed(object? sender, EventArgs args)
			{
				// This should never be called when 'MouseOps' is the one calling dispose
				// It's only to handle an external reference calling Dispose.
				var op = (MouseOp)sender!;
				op.Disposed -= HandleMouseOpDisposed;

				// Clear the active op if it is 'op'
				if (m_active == op)
					m_active = null;

				// Remove the op from the pending set
				foreach (var key in Enum<MouseButton>.Values)
					if (Pending[key] == op)
						Pending[key] = null;
			}

			/// <summary>Dictionary-like proxy for pending mouse operations</summary>
			public sealed class PendingOps :IDisposable
			{
				private readonly MouseOps m_owner;
				private readonly MouseOp?[] m_pending;

				internal PendingOps(MouseOps owner)
				{
					m_owner = owner;
					m_pending = new MouseOp[Enum<MouseButton>.Count];
				}
				public void Dispose()
				{
					Util.DisposeAll(m_pending);
				}
				public MouseOp? this[MouseButton btn]
				{
					get => m_pending[(int)btn];
					set
					{
						if (m_pending[(int)btn] == value) return;
						if (m_pending[(int)btn] is MouseOp old_pending_op)
						{
							old_pending_op.Disposed -= m_owner.HandleMouseOpDisposed;
							old_pending_op.Dispose();
						}
						m_pending[(int)btn] = value;
						if (m_pending[(int)btn] is MouseOp new_pending_op)
						{
							// Watch for external disposing
							new_pending_op.Disposed -= m_owner.HandleMouseOpDisposed;
							new_pending_op.Disposed += m_owner.HandleMouseOpDisposed;
						}
					}
				}
			}
		}

		/// <summary>Base class for a mouse operation performed with the mouse 'down -> [drag] -> up' sequence</summary>
		public abstract class MouseOp :IDisposable
		{
			// The general process goes:
			//  - A mouse op is created and set as the pending operation in 'MouseOps'.
			//  - MouseDown on the chart calls 'BeginOp' which moves the pending op to 'Active'.
			//  - Mouse events on the chart are forwarded to the active op.
			//  - MouseUp ends the current Active op, if the pending op should start immediately
			//    then mouse up causes the next op to start (with a faked MouseDown event).
			//  - If at any point a mouse op is cancelled, no further mouse events are forwarded
			//    to the op. When EndOp is called, a notification can be sent by the op to indicate cancelled.

			protected IDisposable? m_suspended_chart_changed;
			public MouseOp(ChartControl chart, bool allow_cancel = false)
			{
				Chart = chart;
				m_is_click = true;
				m_allow_cancel = allow_cancel;
				StartOnMouseDown = true;
				Cancelled = false;
			}
			public void Dispose()
			{
				Dispose(true);
				GC.SuppressFinalize(this);
			}
			protected virtual void Dispose(bool _)
			{
				Disposed?.Invoke(this, EventArgs.Empty);
				Util.Dispose(ref m_suspended_chart_changed);
			}
			public event EventHandler? Disposed;

			/// <summary>The owning chart</summary>
			protected ChartControl Chart { get; }

			/// <summary>The hit test result on mouse down</summary>
			public HitTestResult HitResult { get; internal set; } = null!;

			/// <summary>The chart space location of where the chart was "grabbed"</summary>
			public v4 GrabChart { get; set; }

			/// <summary>The client scene space location of where the chart was "grabbed" (note: "Scene" space, not "ChartControl" space)</summary>
			public v2 GrabScene { get; set; }

			/// <summary>The chart space location of the current mouse position over the chart</summary>
			public v4 DropChart { get; set; }

			/// <summary>The client scene space location of the current mouse position over the chart (note: "Scene" space, not "ChartControl" space)</summary>
			public v2 DropScene { get; set; }

			/// <summary>The displacement from the grab position</summary>
			public v4 DeltaChart => DropChart - GrabChart;

			/// <summary>The displacement from the grab position (note: "Scene" space, not "ChartControl" space)</summary>
			public v2 DeltaScene => DropScene - GrabScene;

			/// <summary>True if mouse down starts the op, false if the op should start as soon as possible (default is true)</summary>
			public bool StartOnMouseDown { get; set; }

			/// <summary>True if the op was aborted</summary>
			public bool Cancelled { get; protected set; }
			private readonly bool m_allow_cancel;

			/// <summary>True if the distance between 'scene_point' and mouse down should be treated as a click. Once false, then always false</summary>
			public bool IsClick(v2 scene_point)
			{
				// 'scene_point' is a point in 'Scene' space
				if (!m_is_click) return false;
				return m_is_click = (scene_point - GrabScene).LengthSq < Math_.Sqr(Chart.Options.MinDragPixelDistance);
			}
			private bool m_is_click; // True until the mouse is dragged beyond the click threshold

			// Handle events by default. Unhandled events fall back to default handling by the chart

			/// <summary>Called on mouse down</summary>
			public virtual void MouseDown(MouseButtonEventArgs? e)
			{
				// Note: 'e' can be null if the MouseOp starts immediately.
				// Using a dummy MouseButtonEventArgs object results in an InvalidOperationException
				// saying "Every RoutedEventArgs must have a non-null RoutedEvent associated with it".
				if (e == null) return;
				e.Handled = true;
			}

			/// <summary>Called on mouse move</summary>
			public virtual void MouseMove(MouseEventArgs e)
			{
				e.Handled = true;
			}

			/// <summary>Called on mouse up</summary>
			public virtual void MouseUp(MouseButtonEventArgs e)
			{
				e.Handled = true;
			}

			/// <summary>Called on mouse wheel</summary>
			public virtual void MouseWheel(MouseWheelEventArgs e)
			{
				e.Handled = true;
			}

			/// <summary>Called on key down</summary>
			public virtual void OnKeyDown(KeyEventArgs e)
			{
				if (!m_allow_cancel || e.Key != Key.Escape) return;
				Cancelled = true;
				e.Handled = true;
			}

			/// <summary>Called on key up</summary>
			public virtual void OnKeyUp(KeyEventArgs e)
			{
				e.Handled = true;
			}

			/// <summary>Called when the mouse operation is cancelled (as it is removed from 'Active')</summary>
			public virtual void NotifyCancelled()
			{
			}
		}

		/// <summary>A mouse operation for dragging selected elements around, area selecting, or left clicking (Left Button)</summary>
		public class MouseOpDefaultLButton : MouseOp
		{
			private HitTestResult.Hit? m_hit_selected;
			private AreaSelection? m_gfx_area_selection;
			private IDisposable? m_defer_nav_checkpoint;
			private IDisposable? m_mouse_capture;
			private Element? m_dragging_element;
			private EDragState m_drag_state;
			private int m_click_count;
			private EAxis m_hit_axis;

			public MouseOpDefaultLButton(ChartControl chart) 
				: base(chart, allow_cancel: true)
			{}
			protected override void Dispose(bool _)
			{
				Util.Dispose(ref m_gfx_area_selection);
				base.Dispose(_);
			}
			public override void MouseDown(MouseButtonEventArgs? e)
			{
				if (e == null) throw new Exception("This mouse op should start on mouse down");
				var client_point = e.GetPosition(Chart); // point in control space
				m_drag_state = EDragState.Start;
				m_click_count = e.ClickCount;

				// See where mouse down occurred
				if (Chart.SceneBounds.Contains(client_point)) m_hit_axis = EAxis.None;
				if (Chart.XAxisBounds.Contains(client_point)) m_hit_axis = EAxis.XAxis;
				if (Chart.YAxisBounds.Contains(client_point)) m_hit_axis = EAxis.YAxis;

				// Look for a selected object that the mouse operation starts on
				m_hit_selected = HitResult.Hits.FirstOrDefault(x => x.Element.Selected);

				// Record the drag start positions for selected objects
				foreach (var elem in Chart.Selected)
					elem.DragStartPosition = elem.O2W;

				// For 3D scenes, left mouse rotates if mouse down is within the chart bounds
				if (Chart.Options.NavigationMode == ENavMode.Scene3D && m_hit_axis == EAxis.None)
				{
					// Get the point in 'scene' space
					var scene_point = e.GetPosition(Chart.Scene).ToPointI();
					Chart.Scene.Window.MouseNavigate(scene_point, e.ToMouseBtns(Keyboard.Modifiers), View3d.ENavOp.Rotate, true);
				}

				// Prevent events while dragging the elements around
				m_suspended_chart_changed = Chart.SuspendChartChanged(raise_on_resume: true);
				m_defer_nav_checkpoint = Chart.DeferNavCheckpoints();

				// Don't swallow the event
				e.Handled = false;
			}
			public override void MouseMove(MouseEventArgs e)
			{
				var scene_point = e.GetPosition(Chart.Scene).ToV2();

				// If we haven't dragged, treat it as a click instead (i.e. ignore till it's a drag operation)
				if (IsClick(scene_point))
					return;

				// Capture the mouse if this is the start of a drag operation
				if (m_drag_state != EDragState.Dragging)
					m_mouse_capture = Chart.Scene.CaptureMouseScope();

				m_drag_state = EDragState.Dragging;

				// Pass the drag event out to users first
				var delta = Chart.SceneToChart(scene_point) - GrabChart;
				var args = new ChartDraggedEventArgs(HitResult, delta, m_drag_state);
				Chart.OnChartDragged(args);

				// See if the selected element handles dragging
				if (!args.Handled && m_hit_selected != null)
				{
					m_hit_selected.Element.HandleDraggedInternal(args);
					if (args.Handled)
						m_dragging_element = m_hit_selected.Element;
				}

				// See if selected element dragging is enabled
				if (!args.Handled && Chart.Options.AllowElementDragging && m_hit_selected != null)
				{
					// Drag elements in the focus plane of the camera
					var pt0 = Chart.Camera.SSPointToWSPoint(GrabScene);
					var pt1 = Chart.Camera.SSPointToWSPoint(scene_point);
					var translate = pt1 - pt0;
					foreach (var elem in Chart.Selected)
						elem.DragTranslate(translate, args.State);

					args.Handled = true;
				}

				// Otherwise, interpret drag as a navigation
				if (args.Handled) { }
				else if (Chart.Options.NavigationMode == ENavMode.Chart2D)
				{
					if (Chart.DoChartAreaSelect(HitResult.ModifierKeys))
					{
						// Position the selection graphic
						m_gfx_area_selection ??= new AreaSelection(Chart);
						m_gfx_area_selection.Selection = BBox.From(GrabChart, Chart.SceneToChart(scene_point));
					}
				}
				else if (Chart.Options.NavigationMode == ENavMode.Scene3D)
				{
					Chart.Scene.Window.MouseNavigate(scene_point.ToPoint(), e.ToMouseBtns(Keyboard.Modifiers), View3d.ENavOp.Rotate, false);
				}
				Chart.SetRangeFromCamera();
				Chart.Invalidate();

				e.Handled = args.Handled;
			}
			public override void MouseUp(MouseButtonEventArgs e)
			{
				Util.Dispose(ref m_suspended_chart_changed);
				Util.Dispose(ref m_defer_nav_checkpoint);
				Util.Dispose(ref m_mouse_capture);
				var scene_point = e.GetPosition(Chart.Scene).ToV2();

				// If this is a click...
				if (IsClick(scene_point))
				{
					// Pass the click event out to users first
					var args = new ChartClickedEventArgs(HitResult, e, m_click_count);
					Chart.OnChartClicked(args);

					// If a selected element was hit on mouse down, see if it handles the click
					if (!args.Handled && m_hit_selected != null)
					{
						m_hit_selected.Element.HandleClickedInternal(args);
					}

					// If no selected element was hit, try hovered elements
					if (!args.Handled && HitResult.Hits.Count != 0)
					{
						for (int i = 0; i != HitResult.Hits.Count && !args.Handled; ++i)
						{
							if (HitResult.Hits[i] == m_hit_selected) continue;
							HitResult.Hits[i].Element.HandleClickedInternal(args);
						}
					}

					// If the click is still unhandled, use the click to try to select something (if within the chart)
					if (!args.Handled && HitResult.Zone.HasFlag(EZone.Chart))
					{
						var selection = new BBox(GrabChart, v4.Zero);
						Chart.SelectElements(selection, Keyboard.Modifiers, e.ToMouseBtns());
					}

					e.Handled = args.Handled;
				}
				// Otherwise this is a drag action
				else
				{
					// Commit if dragging hasn't been cancelled
					if (m_drag_state == EDragState.Dragging)
						m_drag_state = EDragState.Commit;

					// Pass the drag event out to users first
					var delta = Chart.SceneToChart(scene_point) - GrabChart;
					var args = new ChartDraggedEventArgs(HitResult, delta, m_drag_state);
					Chart.OnChartDragged(args);

					// See if the selected element handles dragging
					if (!args.Handled && m_dragging_element != null)
					{
						m_dragging_element.HandleDraggedInternal(args);
						m_dragging_element = null;
						args.Handled = true;
					}

					// See if selected element dragging is enabled
					if (!args.Handled && Chart.Options.AllowElementDragging && m_hit_selected != null)
					{
						// Already in position
						args.Handled = true;
					}

					// Otherwise, interpret drag as a navigation
					if (args.Handled) {}
					else if (Chart.Options.NavigationMode == ENavMode.Chart2D)
					{
						// Otherwise create an area selection if the click started within the chart
						if (HitResult.Zone.HasFlag(EZone.Chart) && m_gfx_area_selection != null)
						{
							var chart_selection_bbox = BBox.From(GrabChart, Chart.SceneToChart(scene_point));
							Chart.OnChartAreaSelect(new ChartAreaSelectEventArgs(chart_selection_bbox, e.ToMouseBtns()));
						}
					}
					else if (Chart.Options.NavigationMode == ENavMode.Scene3D)
					{
						// For 3D scenes, left mouse rotates
						Chart.Scene.Window.MouseNavigate(scene_point.ToPoint(), e.ToMouseBtns(Keyboard.Modifiers), View3d.ENavOp.Rotate, true);
					}

					e.Handled = args.Handled;
				}

				Util.Dispose(ref m_gfx_area_selection);
				Chart.Cursor = Cursors.Arrow;
				Chart.Invalidate();
			}
			public override void OnKeyDown(KeyEventArgs e)
			{
				base.OnKeyDown(e);
				if (Cancelled)
				{
					m_drag_state = EDragState.Cancel;

					// Abort dragging
					if (m_dragging_element != null)
					{
						m_dragging_element.HandleDraggedInternal(new ChartDraggedEventArgs(HitResult, default, m_drag_state));
						m_dragging_element = null;
					}

					// Remove the selection graphics
					Util.Dispose(ref m_gfx_area_selection);

					// Refresh
					Chart.Invalidate();
				}
			}
		}

		/// <summary>A mouse operation for zooming (Middle Button)</summary>
		public class MouseOpDefaultMButton : MouseOp
		{
			private IDisposable? m_defer_nav_checkpoint;
			private IDisposable? m_mouse_capture;
			private EDragState m_drag_state;

			public MouseOpDefaultMButton(ChartControl chart)
				: base(chart)
			{}
			public override void MouseDown(MouseButtonEventArgs? e)
			{
				if (e == null) throw new Exception("This mouse op should start on mouse down");
				var location = e.GetPosition(Chart);

				// If mouse down occurred within the chart, record it
				if (Chart.SceneBounds.Contains(location))
				{
					Chart.Cursor = Cursors.Cross;
				}

				m_drag_state = EDragState.Start;
				m_defer_nav_checkpoint = Chart.DeferNavCheckpoints();
			}
			public override void MouseMove(MouseEventArgs e)
			{
				// If we haven't dragged, treat it as a click instead (i.e. ignore till it's a drag operation)
				var scene_point = e.GetPosition(Chart.Scene).ToV2();
				if (IsClick(scene_point))
					return;

				// Capture the mouse if this is the start of a drag operation
				if (m_drag_state != EDragState.Dragging)
					m_mouse_capture = Chart.Scene.CaptureMouseScope();

				m_drag_state = EDragState.Dragging;

				Chart.Invalidate();
			}
			public override void MouseUp(MouseButtonEventArgs e)
			{
				var scene_point = e.GetPosition(Chart.Scene).ToV2();
				Util.Dispose(ref m_suspended_chart_changed);
				Util.Dispose(ref m_defer_nav_checkpoint);
				Util.Dispose(ref m_mouse_capture);
				Chart.Cursor = Cursors.Arrow;

				// If this is a single click...
				if (IsClick(scene_point))
				{
					// Pass the click event out to users first
					var args = new ChartClickedEventArgs(HitResult, e);
					Chart.OnChartClicked(args);
					e.Handled = args.Handled;

					if (!e.Handled)
					{
						Chart.Scene.Window.TranslateKey(Interop.Win32.EKeyCodes.MButton, scene_point);
					}
				}
				// Otherwise this is a drag action
				else
				{
					// Commit if dragging hasn't been cancelled
					if (m_drag_state == EDragState.Dragging)
						m_drag_state = EDragState.Commit;

				}

				Chart.Invalidate();
			}
		}

		/// <summary>A mouse operation for dragging the chart around or right clicking (Right Button)</summary>
		public class MouseOpDefaultRButton : MouseOp
		{
			private EAxis m_drag_axis_allow; // The allowed motion based on where the chart was grabbed
			private EDragState m_drag_state;
			private IDisposable? m_defer_nav_checkpoint;
			private IDisposable? m_mouse_capture;

			public MouseOpDefaultRButton(ChartControl chart)
				: base(chart)
			{ }
			public override void MouseDown(MouseButtonEventArgs? e)
			{
				if (e == null) throw new Exception("This mouse op should start on mouse down");
				var location = e.GetPosition(Chart);

				if (Chart.SceneBounds.Contains(location)) m_drag_axis_allow = EAxis.Both;
				if (Chart.XAxisBounds.Contains(location)) m_drag_axis_allow = EAxis.XAxis;
				if (Chart.YAxisBounds.Contains(location)) m_drag_axis_allow = EAxis.YAxis;
				if (!Chart.XAxis.AllowScroll) m_drag_axis_allow &= ~EAxis.XAxis;
				if (!Chart.YAxis.AllowScroll) m_drag_axis_allow &= ~EAxis.YAxis;

				// Right mouse translates for 2D and 3D scene
				var point_ss = e.GetPosition(Chart.Scene).ToPointI();
				Chart.Scene.Window.MouseNavigate(point_ss, e.ToMouseBtns(Keyboard.Modifiers), View3d.ENavOp.Translate, true);

				m_drag_state = EDragState.Start;
				m_defer_nav_checkpoint = Chart.DeferNavCheckpoints();
			}
			public override void MouseMove(MouseEventArgs e)
			{
				var scene_point = e.GetPosition(Chart.Scene).ToV2();

				// If we haven't dragged, treat it as a click instead (i.e. ignore till it's a drag operation)
				if (IsClick(scene_point))
					return;

				// Capture the mouse if this is the start of a drag operation
				if (m_drag_state != EDragState.Dragging)
					m_mouse_capture = Chart.Scene.CaptureMouseScope();

				m_drag_state = EDragState.Dragging;

				// Limit the drag direction
				var grab_loc = GrabScene;
				var drop_loc = scene_point;
				if (!m_drag_axis_allow.HasFlag(EAxis.XAxis)) drop_loc.x = grab_loc.x;
				if (!m_drag_axis_allow.HasFlag(EAxis.YAxis)) drop_loc.y = grab_loc.y;

				// Change the cursor once dragging
				Chart.Scene.Cursor = Cursors.SizeAll;

				Chart.Scene.Window.MouseNavigate(drop_loc.ToPoint(), e.ToMouseBtns(Keyboard.Modifiers), View3d.ENavOp.Translate, false);
				Chart.SetRangeFromCamera();
				Chart.Invalidate();
			}
			public override void MouseUp(MouseButtonEventArgs e)
			{
				var scene_point = e.GetPosition(Chart.Scene).ToV2();
				Util.Dispose(ref m_defer_nav_checkpoint);
				Util.Dispose(ref m_mouse_capture);
				Chart.Scene.Cursor = Cursors.Arrow;

				// If we haven't dragged, treat it as a click instead
				if (IsClick(scene_point))
				{
					var args = new ChartClickedEventArgs(HitResult, e);
					Chart.OnChartClicked(args);
					Chart.Invalidate();

					e.Handled = args.Handled;
				}
				else
				{
					// Commit if dragging hasn't been cancelled
					if (m_drag_state == EDragState.Dragging)
						m_drag_state = EDragState.Commit;

					// Limit the drag direction
					var grab_loc = GrabScene;
					var drop_loc = scene_point;
					if (!m_drag_axis_allow.HasFlag(EAxis.XAxis)) drop_loc.x = grab_loc.x;
					if (!m_drag_axis_allow.HasFlag(EAxis.YAxis)) drop_loc.y = grab_loc.y;

					Chart.Scene.Window.MouseNavigate(drop_loc.ToPoint(), e.ToMouseBtns(Keyboard.Modifiers), View3d.ENavOp.None, true);
					Chart.SetRangeFromCamera();
					Chart.Invalidate();

					e.Handled = true;
				}
			}
			public override void OnKeyDown(KeyEventArgs e)
			{
				base.OnKeyDown(e);
				if (Cancelled)
				{
					m_drag_state = EDragState.Cancel;

					// Refresh
					Chart.Invalidate();
				}
			}
		}
	}
}
