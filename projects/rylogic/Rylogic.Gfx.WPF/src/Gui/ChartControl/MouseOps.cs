using System;
using System.Linq;
using System.Windows;
using System.Windows.Input;
using System.Windows.Interop;
using Rylogic.Common;
using Rylogic.Extn;
using Rylogic.Gfx;
using Rylogic.Maths;
using Rylogic.Utility;
using Rylogic.Windows.Extn;

namespace Rylogic.Gui.WPF
{
	public partial class ChartControl
	{
		// Note:
		//  - The chain of mouse events starts in 'Navigation.cs'
		//
		// The general process goes like this:
		//  - A mouse op is created and set as the pending operation in 'MouseOps'.
		//  - MouseDown on the chart calls 'BeginOp' which moves the pending op to 'Active'.
		//  - Mouse events on the chart are forwarded to the active op.
		//  - MouseUp ends the current Active op, if the pending op should start immediately
		//    then mouse up causes the next op to start (with a faked MouseDown event).
		//  - If at any point a mouse op is cancelled, no further mouse events are forwarded
		//    to the op. When EndOp is called, a notification can be sent by the op to indicate cancelled.

		/// <summary>Manages per-button mouse operations</summary>
		public class MouseOps :IDisposable
		{
			// Notes:
			//  - Ownership: Once a mouse op is added to the 'MouseOps' object, 'MouseOps' is responsible for disposing it.
			//  - The usage pattern is:
			//      - Client creates a mouse op and sets it as a pending op in 'MouseOps' for a specific button.
			//      - On MouseDown, the pending op for the appropriate button moves to 'Active'
			//      - On MouseUp, 'Active' is cleared and disposed
			//      - At any time, the mouse op can be Cancelled. Cancel is notified when the op is removed from 'Active'.
			private readonly MouseOp?[] m_pending = new MouseOp[Enum<MouseButton>.Count];

			public MouseOps()
			{
			}
			public void Dispose()
			{
				Active = null;
				Util.DisposeAll(m_pending);
				GC.SuppressFinalize(this);
			}
			
			/// <summary>The currently active mouse op</summary>
			public MouseOp? Active
			{
				get;
				private set
				{
					if (Active == value) return;
					if (field != null)
					{
						// Notify if the op was cancelled while active.
						if (field.Cancelled)
							field.NotifyCancelled();

						field.Dispose();
					}
					field = value;
					if (field != null)
					{
						// If the op starts immediately without a mouse down, fake
						// a mouse down event as soon as it becomes active.
						if (!field.StartOnMouseDown)
							field.MouseDown(null);
					}
				}
			}

			/// <summary>The pending mouse operations for each button</summary>
			public MouseOp? this[MouseButton btn]
			{
				get => m_pending[(int)btn];
				set 
				{
					// Notes:
					//  - 'value' might already be in the 'm_pending' list.
					//  - 'value' might be in the 'm_pending' list multiple times.
					//  - 'value' might be null when clearing a pending op.

					// Swap the pending op for 'btn' with 'value'
					var existing = m_pending[(int)btn];
					m_pending[(int)btn] = value;

					// If we no longer hold a reference to 'existing', dispose it.
					if (existing != null && Active != existing && !m_pending.Contains(existing))
						existing.Dispose();
				}
			}

			/// <summary>Start/End the next mouse op for button 'idx'</summary>
			public void BeginOp(MouseButton btn)
			{
				Active = this[btn];
				this[btn] = null;
			}
			public void BeginModalOp(MouseOp op)
			{
				Active = op;
			}
			public void EndOp(MouseButton btn)
			{
				if (Active != null && Active.IsModal)
					return;

				Active = null;

				// If the next op starts immediately, begin it now
				if (this[btn] is MouseOp op && !op.StartOnMouseDown)
					BeginOp(btn);
			}
			public void EndModalOp(MouseOp op)
			{
				if (Active == op)
					Active = null;
			}
		}

		/// <summary>Base class for a mouse operation performed with the mouse 'down -> [drag] -> up' sequence</summary>
		public abstract class MouseOp :IDisposable, RawInput.IInputSink
		{
			protected IDisposable? m_suspended_chart_changed;
			protected IDisposable? m_defer_nav_checkpoint;
			protected IDisposable? m_mouse_capture;
			protected EDragState m_drag_state;
			protected int m_click_count;

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
				Util.Dispose(ref m_defer_nav_checkpoint);
				Util.Dispose(ref m_mouse_capture);
			}
			public event EventHandler? Disposed;

			/// <summary>The owning chart</summary>
			protected ChartControl Chart { get; }

			/// <summary>The hit test result on mouse down</summary>
			public HitTestResult HitResult { get; internal set; } = null!;

			/// <summary>The chart space location of where the chart was "grabbed"</summary>
			public v4 GrabCS { get; set; }

			/// <summary>The client scene space location of where the chart was "grabbed" (note: "Scene" space, not "ChartControl" space)</summary>
			public v2 GrabSS { get; set; }

			/// <summary>The chart space location of the current mouse position over the chart</summary>
			public v4 DropCS { get; set; }

			/// <summary>The client scene space location of the current mouse position over the chart (note: "Scene" space, not "ChartControl" space)</summary>
			public v2 DropSS { get; set; }

			/// <summary>The displacement from the grab position</summary>
			public v4 DeltaCS => DropCS - GrabCS;

			/// <summary>The displacement from the grab position (note: "Scene" space, not "ChartControl" space)</summary>
			public v2 DeltaSS => DropSS - GrabSS;

			/// <summary>True if mouse down starts the op, false if the op should start as soon as possible (default is true)</summary>
			public bool StartOnMouseDown { get; set; }

			/// <summary>True for modal operations that remain active until explicitly ended</summary>
			public bool IsModal { get; set; }

			/// <summary>True if the op was aborted</summary>
			public bool Cancelled { get; protected set; }
			private readonly bool m_allow_cancel;

			/// <summary>True if the distance between 'scene_point' and mouse down should be treated as a click. Once false, then always false</summary>
			public bool IsClick(v2 scene_point)
			{
				// 'scene_point' is a point in 'Scene' space
				if (!m_is_click) return false;
				return m_is_click = (scene_point - GrabSS).LengthSq < Math_.Sqr(Chart.Options.MinDragPixelDistance);
			}
			private bool m_is_click; // True until the mouse is dragged beyond the click threshold

			/// <summary>Update the Drop location values</summary>
			private void UpdateDrop(MouseEventArgs e)
			{
				var client_point = e.GetPosition(Chart);
				DropSS = Gui_.MapPoint(Chart, Chart.Scene, client_point).ToV2();
				DropCS = Chart.SceneToChart(DropSS);
			}

			// Handle events by default. Unhandled events fall back to default handling by the chart

			/// <summary>Called on mouse down</summary>
			public virtual void MouseDown(MouseButtonEventArgs? e)
			{
				// Note: 'e' can be null if the MouseOp starts immediately.
				// Using a dummy MouseButtonEventArgs object results in an InvalidOperationException
				// saying "Every RoutedEventArgs must have a non-null RoutedEvent associated with it".
				if (e == null) return;
			}

			/// <summary>Called on mouse move</summary>
			public virtual void MouseMove(MouseEventArgs e)
			{
				UpdateDrop(e);
			}

			/// <summary>Called on mouse up</summary>
			public virtual void MouseUp(MouseButtonEventArgs e)
			{
				Util.Dispose(ref m_suspended_chart_changed);
				Util.Dispose(ref m_defer_nav_checkpoint);
				Util.Dispose(ref m_mouse_capture);
				UpdateDrop(e);
			}

			/// <summary>Called on mouse wheel</summary>
			public virtual void MouseWheel(MouseWheelEventArgs e)
			{
			}

			/// <summary>Called on key down</summary>
			public virtual void OnKeyDown(KeyEventArgs e)
			{
				if (!m_allow_cancel || e.Key != Key.Escape) return;
				Cancelled = true;
			}

			/// <summary>Called on key up</summary>
			public virtual void OnKeyUp(KeyEventArgs e)
			{
			}

			/// <summary>Called when the mouse operation is cancelled (as it is removed from 'Active')</summary>
			public virtual void NotifyCancelled()
			{
			}
		}

		/// <summary>Mouse left-button behaviour for 3D scenes</summary>
		public class MouseOp_LButton_3DScene : MouseOp
		{
			public MouseOp_LButton_3DScene(ChartControl chart) 
				: base(chart)
			{}
			public override void MouseDown(MouseButtonEventArgs? e)
			{
				if (e == null) throw new Exception("This mouse op should start on mouse down");
				m_drag_state = EDragState.Start;
				m_click_count = e.ClickCount;

				// Prevent events while dragging the elements around
				m_suspended_chart_changed = Chart.SuspendChartChanged(raise_on_resume: true);

				// Ignore nav checkpoints until the mouse is released
				m_defer_nav_checkpoint = Chart.DeferNavCheckpoints();

				// Capture the mouse
				m_mouse_capture = Chart.CaptureMouseScope();

				// For 3D scenes, left mouse rotates if mouse down is within the chart bounds
				if (HitResult.Zone.HasFlag(EZone.Chart))
				{
					Chart.Scene.Window.MouseNavigate(GrabSS.ToPoint(), e.ToMouseBtns(), View3d.ENavOp.Rotate, true);
				}

				// Don't swallow the event
				e.Handled = false;
			}
			public override void MouseMove(MouseEventArgs e)
			{
				base.MouseMove(e);

				// If we haven't dragged, treat it as a click instead (i.e. ignore till it's a drag operation)
				if (IsClick(DropSS))
					return;

				// Once the drag threshold is exceeded, switch to smooth navigation.
				// This suppresses WM_MOUSEMOVE/WM_NCHITTEST/WM_SETCURSOR flooding and
				// drives navigation at compositor frame rate via GetCursorPos instead.
				m_drag_state = EDragState.Dragging;

				// During drag, move events come from the compositor
				if (HitResult.Zone.HasFlag(EZone.Chart))
				{
					Chart.Scene.Window.MouseNavigate(DropSS.ToPoint(), e.ToMouseBtns(), View3d.ENavOp.Rotate, false);
				}

				Chart.SetRangeFromCamera();
				Chart.Invalidate();

				e.Handled = false;
			}
			public override void MouseUp(MouseButtonEventArgs e)
			{
				base.MouseUp(e);

				// If this is a click...
				if (IsClick(DropSS))
				{
					// Pass the click event out to users first
					var args = new ChartClickedEventArgs(HitResult, e, m_click_count);
					Chart.OnChartClicked(args);
					e.Handled = args.Handled;
				}

				// If dragging, commit if not cancelled
				if (m_drag_state == EDragState.Dragging)
				{
					m_drag_state = EDragState.Commit;

					// Pass the drag event out to users first
					//var delta = Chart.SceneToChart(DropSS) - GrabCS;
					var args = new ChartDraggedEventArgs(HitResult, DeltaCS, m_drag_state);
					Chart.OnChartDragged(args);
					e.Handled = args.Handled;
				}

				if (!e.Handled)
				{
					Chart.Scene.Window.MouseNavigate(DropSS.ToPoint(), e.ToMouseBtns(), View3d.ENavOp.Rotate, true);
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

		/// <summary>Mouse left-button behaviour for 2D charts</summary>
		public class MouseOp_LButton_2DChart : MouseOp
		{
			private HitTestResult.Hit? m_hit_selected;
			private AreaSelection? m_gfx_area_selection;
			private Element? m_dragging_element;

			public MouseOp_LButton_2DChart(ChartControl chart) 
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
				m_drag_state = EDragState.Start;
				m_click_count = e.ClickCount;

				// Look for a selected object that the mouse operation starts on
				m_hit_selected = HitResult.Hits.FirstOrDefault(x => x.Element.Selected);

				// Record the drag start positions for selected objects
				foreach (var elem in Chart.Selected)
					elem.DragStartPosition = elem.O2W;

				// Prevent events while dragging the elements around
				m_suspended_chart_changed = Chart.SuspendChartChanged(raise_on_resume: true);

				// Ignore nav checkpoints until the mouse is released
				m_defer_nav_checkpoint = Chart.DeferNavCheckpoints();

				// Capture the mouse
				m_mouse_capture = Chart.CaptureMouseScope();

				// Don't swallow the event
				e.Handled = false;
			}
			public override void MouseMove(MouseEventArgs e)
			{
				base.MouseMove(e);

				// If we haven't dragged, treat it as a click instead (i.e. ignore till it's a drag operation)
				if (IsClick(DropSS))
					return;

				m_drag_state = EDragState.Dragging;

				// Pass the drag event out to users first
				var args = new ChartDraggedEventArgs(HitResult, DeltaCS, m_drag_state);
				Chart.OnChartDragged(args);

				// See if the selected element handles dragging
				if (!args.Handled && m_hit_selected != null)
				{
					m_hit_selected.Element.HandleDraggedInternal(args);
					if (args.Handled)
						m_dragging_element = m_hit_selected.Element;
				}

				// See if selected element dragging is enabled
				if (!args.Handled && m_hit_selected != null && Chart.Options.AllowElementDragging)
				{
					// Drag elements in the focus plane of the camera
					var pt0 = Chart.Camera.SSPointToWSPoint(GrabSS);
					var pt1 = Chart.Camera.SSPointToWSPoint(DropSS);
					var translate = pt1 - pt0;
					foreach (var elem in Chart.Selected)
						elem.DragTranslate(translate, args.State);

					args.Handled = true;
				}

				// Otherwise, interpret the drag as a navigation
				if (!args.Handled)
				{
					if (Chart.DoChartAreaSelect(HitResult.ModifierKeys))
					{
						// Position the selection graphic
						m_gfx_area_selection ??= new AreaSelection(Chart);
						m_gfx_area_selection.Selection = BBox.From(GrabCS, Chart.SceneToChart(DropSS));
					}
				}

				Chart.SetRangeFromCamera();
				Chart.Invalidate();
				e.Handled = args.Handled;
			}
			public override void MouseUp(MouseButtonEventArgs e)
			{
				base.MouseUp(e);

				// If this is a click...
				if (IsClick(DropSS))
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
						var selection = new BBox(GrabCS, v4.Zero);
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
					var args = new ChartDraggedEventArgs(HitResult, DeltaCS, m_drag_state);
					Chart.OnChartDragged(args);

					// See if the selected element handles dragging
					if (!args.Handled && m_dragging_element != null)
					{
						m_dragging_element.HandleDraggedInternal(args);
						m_dragging_element = null;
						args.Handled = true;
					}

					// See if selected element dragging is enabled
					if (!args.Handled && m_hit_selected != null && Chart.Options.AllowElementDragging)
					{
						// Already in position
						args.Handled = true;
					}

					// Otherwise, interpret drag as a navigation
					if (!args.Handled)
					{
						// Perform an area selection if the click started within the chart
						if (HitResult.Zone.HasFlag(EZone.Chart) && m_gfx_area_selection != null)
						{
							var chart_selection_bbox = BBox.From(GrabCS, Chart.SceneToChart(DropSS));
							Chart.OnChartAreaSelect(new ChartAreaSelectEventArgs(chart_selection_bbox, e.ToMouseBtns()));
						}
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

		/// <summary>Mouse right-button behaviour for 3D scenes</summary>
		public class MouseOp_RButton_3DScene : MouseOp
		{
			// The allowed motion based on where the chart was grabbed
			private EAxis m_drag_axis_allow;

			public MouseOp_RButton_3DScene(ChartControl chart)
				: base(chart)
			{ }
			public override void MouseDown(MouseButtonEventArgs? e)
			{
				if (e == null) throw new Exception("This mouse op should start on mouse down");

				// Determine the allowed drag axes from the hit zone
				m_drag_axis_allow =
					HitResult.Zone.HasFlag(EZone.YAxis) ? EAxis.YAxis :
					HitResult.Zone.HasFlag(EZone.XAxis) ? EAxis.XAxis :
					EAxis.Both;

				if (!Chart.XAxis.AllowScroll) m_drag_axis_allow &= ~EAxis.XAxis;
				if (!Chart.YAxis.AllowScroll) m_drag_axis_allow &= ~EAxis.YAxis;

				// Right mouse translates for 3D scenes
				Chart.Scene.Window.MouseNavigate(GrabSS.ToPoint(), e.ToMouseBtns(), View3d.ENavOp.Translate, true);
				m_defer_nav_checkpoint = Chart.DeferNavCheckpoints();
				m_mouse_capture = Chart.CaptureMouseScope();
				m_drag_state = EDragState.Start;
			}
			public override void MouseMove(MouseEventArgs e)
			{
				base.MouseMove(e);

				// If we haven't dragged, treat it as a click instead (i.e. ignore till it's a drag operation)
				if (IsClick(DropSS))
					return;

				m_drag_state = EDragState.Dragging;

				// Change the cursor once dragging
				Chart.Scene.Cursor = Cursors.SizeAll;

				// Limit the drag direction
				var drop_loc = DropSS;
				if (!m_drag_axis_allow.HasFlag(EAxis.XAxis)) drop_loc.x = GrabSS.x;
				if (!m_drag_axis_allow.HasFlag(EAxis.YAxis)) drop_loc.y = GrabSS.y;

				Chart.Scene.Window.MouseNavigate(drop_loc.ToPoint(), e.ToMouseBtns(), View3d.ENavOp.Translate, false);
				Chart.SetRangeFromCamera();
				Chart.Invalidate();
			}
			public override void MouseUp(MouseButtonEventArgs e)
			{
				base.MouseUp(e);
				Chart.Scene.Cursor = Cursors.Arrow;

				// If we haven't dragged, treat it as a click instead
				if (IsClick(DropSS))
				{
					var args = new ChartClickedEventArgs(HitResult, e);
					Chart.OnChartClicked(args);
					Chart.Invalidate();
					e.Handled = args.Handled;
				}

				// If dragging, commit if not cancelled
				if (m_drag_state == EDragState.Dragging)
				{
					m_drag_state = EDragState.Commit;

					// Limit the drag direction
					var drop_loc = DropSS;
					if (!m_drag_axis_allow.HasFlag(EAxis.XAxis)) drop_loc.x = GrabSS.x;
					if (!m_drag_axis_allow.HasFlag(EAxis.YAxis)) drop_loc.y = GrabSS.y;

					Chart.Scene.Window.MouseNavigate(drop_loc.ToPoint(), e.ToMouseBtns(), View3d.ENavOp.None, true);
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

		/// <summary>Mouse right-button behaviour for 2D charts</summary>
		public class MouseOp_RButton_2DChart : MouseOp
		{
			private EAxis m_drag_axis_allow; // The allowed motion based on where the chart was grabbed

			public MouseOp_RButton_2DChart(ChartControl chart)
				: base(chart)
			{}
			public override void MouseDown(MouseButtonEventArgs? e)
			{
				if (e == null) throw new Exception("This mouse op should start on mouse down");

				// Determine the allowed drag axes from the hit zone
				m_drag_axis_allow =
					HitResult.Zone.HasFlag(EZone.YAxis) ? EAxis.YAxis :
					HitResult.Zone.HasFlag(EZone.XAxis) ? EAxis.XAxis :
					EAxis.Both;

				if (!Chart.XAxis.AllowScroll) m_drag_axis_allow &= ~EAxis.XAxis;
				if (!Chart.YAxis.AllowScroll) m_drag_axis_allow &= ~EAxis.YAxis;

				// Right mouse translates for 2D charts
				Chart.Scene.Window.MouseNavigate(GrabSS.ToPoint(), e.ToMouseBtns(), View3d.ENavOp.Translate, true);
				m_defer_nav_checkpoint = Chart.DeferNavCheckpoints();
				m_mouse_capture = Chart.CaptureMouseScope();
				m_drag_state = EDragState.Start;
			}
			public override void MouseMove(MouseEventArgs e)
			{
				base.MouseMove(e);

				// If we haven't dragged, treat it as a click instead (i.e. ignore till it's a drag operation)
				if (IsClick(DropSS))
					return;

				m_drag_state = EDragState.Dragging;

				// Change the cursor once dragging
				Chart.Scene.Cursor = Cursors.SizeAll;

				// Limit the drag direction
				var drop_loc = DropSS;
				if (!m_drag_axis_allow.HasFlag(EAxis.XAxis)) drop_loc.x = GrabSS.x;
				if (!m_drag_axis_allow.HasFlag(EAxis.YAxis)) drop_loc.y = GrabSS.y;

				Chart.Scene.Window.MouseNavigate(drop_loc.ToPoint(), e.ToMouseBtns(), View3d.ENavOp.Translate, false);
				Chart.SetRangeFromCamera();
				Chart.Invalidate();
			}
			public override void MouseUp(MouseButtonEventArgs e)
			{
				base.MouseUp(e);
				Chart.Scene.Cursor = Cursors.Arrow;

				// If we haven't dragged, treat it as a click instead
				if (IsClick(DropSS))
				{
					var args = new ChartClickedEventArgs(HitResult, e);
					Chart.OnChartClicked(args);
					Chart.Invalidate();
					e.Handled = args.Handled;
				}

				// If dragging, commit if not cancelled
				if (m_drag_state == EDragState.Dragging)
				{
					m_drag_state = EDragState.Commit;

					// Limit the drag direction
					var drop_loc = DropSS;
					if (!m_drag_axis_allow.HasFlag(EAxis.XAxis)) drop_loc.x = GrabSS.x;
					if (!m_drag_axis_allow.HasFlag(EAxis.YAxis)) drop_loc.y = GrabSS.y;

					Chart.Scene.Window.MouseNavigate(drop_loc.ToPoint(), e.ToMouseBtns(), View3d.ENavOp.None, true);
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

		/// <summary>A mouse operation for zooming (Middle Button)</summary>
		public class MouseOp_MButton : MouseOp
		{
			public MouseOp_MButton(ChartControl chart)
				: base(chart)
			{}
			public override void MouseDown(MouseButtonEventArgs? e)
			{
				if (e == null) throw new Exception("This mouse op should start on mouse down");

				// If mouse down occurred within the chart, record it
				if (HitResult.Zone.HasFlag(EZone.Chart))
				{
					Chart.Cursor = Cursors.Cross;
				}

				m_defer_nav_checkpoint = Chart.DeferNavCheckpoints();
				m_mouse_capture = Chart.CaptureMouseScope();
				m_drag_state = EDragState.Start;
			}
			public override void MouseMove(MouseEventArgs e)
			{
				base.MouseMove(e);

				// If we haven't dragged, treat it as a click instead (i.e. ignore till it's a drag operation)
				if (IsClick(DropSS))
					return;

				m_drag_state = EDragState.Dragging;
				Chart.Invalidate();
			}
			public override void MouseUp(MouseButtonEventArgs e)
			{
				base.MouseUp(e);
				Chart.Cursor = Cursors.Arrow;

				// If this is a single click...
				if (IsClick(DropSS))
				{
					// Pass the click event out to users first
					var args = new ChartClickedEventArgs(HitResult, e);
					Chart.OnChartClicked(args);

					if (!args.Handled)
					{
						Chart.Scene.Window.TranslateKey(Interop.Win32.EKeyCodes.MButton, DropSS);
					}

					e.Handled = args.Handled;
				}

				// Dragging
				else
				{
					// Commit if dragging hasn't been cancelled
					if (m_drag_state == EDragState.Dragging)
						m_drag_state = EDragState.Commit;
				}

				Chart.Invalidate();
			}
		}

		/// <summary>A mouse to set the measurement tool start/end points</summary>
		public class MouseOp_LButton_MeasureTool : MouseOp
		{
			// Don't actually need to do anything except allow the message to pass through
			// to the ChartPanel (Scene). This mouse op prevents the default navigation behaviour
			public MouseOp_LButton_MeasureTool(ChartControl chart)
				: base(chart)
			{}
			public override void MouseUp(MouseButtonEventArgs e)
			{
				base.MouseUp(e);
				Chart.Invalidate();
			}
		}

		/// <summary>Click-to-select on scene objects.</summary>
		public class MouseOp_LButton_SelectObject : MouseOp
		{
			public MouseOp_LButton_SelectObject(ChartControl chart)
				: base(chart)
			{}
			public override void MouseDown(MouseButtonEventArgs? e)
			{
				if (e == null) return;

				// Don't capture the mouse or start any navigation here.
				// This op only fires on mouse-up if the press behaved as a click.
				e.Handled = false;
			}
			public override void MouseMove(MouseEventArgs e)
			{
				base.MouseMove(e);

				// If the user starts dragging, we're done - selection only fires on a click.
				// Letting the op finish naturally on mouse-up keeps the state machine simple.
				if (!IsClick(DropSS))
					m_drag_state = EDragState.Cancel;
			}
			public override void MouseUp(MouseButtonEventArgs e)
			{
				base.MouseUp(e);

				// Only fire on a true click (not a drag)
				if (!IsClick(DropSS))
					return;

				// Cast a pixel-accurate ray through the click point and hit-test the scene
				var window = Chart.Scene.Window;
				var camera = Chart.Scene.Camera;
				var ray = camera.RaySS(GrabSS, View3d.ESnapMode.Faces, 0f);
				var hit = window.HitTest(ray);

				// Notify listeners (e.g. SceneUI -> Scene Manager)
				var args = new SceneObjectClickedEventArgs(hit, Keyboard.Modifiers, e.ChangedButton);
				Chart.OnSceneObjectClicked(args);
				e.Handled = args.Handled;
			}
		}

		/// <summary>Modal flight-camera operation for 3D scenes</summary>
		public class MouseOp_FlightCamera : MouseOp
		{
			private FlightCameraSession? m_session;

			public MouseOp_FlightCamera(ChartControl chart)
				: base(chart, allow_cancel: true)
			{
				StartOnMouseDown = false;
				IsModal = true;
			}
			protected override void Dispose(bool _)
			{
				Util.Dispose(ref m_session);
				base.Dispose(_);
			}
			public override void MouseDown(MouseButtonEventArgs? e)
			{
				m_session = new FlightCameraSession(Chart.Scene.Window, Chart.Scene, () => Chart.FlightCamera = false, () =>
				{
					var keyboard_shortcuts = Chart.DefaultKeyboardShortcuts;
					Chart.DefaultKeyboardShortcuts = false;
					return Scope.Create(null, () => Chart.DefaultKeyboardShortcuts = keyboard_shortcuts);
				});
				if (e != null)
					e.Handled = true;
			}
			public override void MouseMove(MouseEventArgs e)
			{
				e.Handled = true;
			}
			public override void MouseUp(MouseButtonEventArgs e)
			{
				e.Handled = true;
			}
			public override void MouseWheel(MouseWheelEventArgs e)
			{
				e.Handled = true;
			}
		}
	}
}
