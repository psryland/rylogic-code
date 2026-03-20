using System;
using System.Diagnostics;
using System.Linq;
using System.Windows.Input;
using Rylogic.Common;
using Rylogic.Extn;
using Rylogic.Maths;
using Rylogic.Utility;
using Rylogic.Windows.Extn;

namespace Rylogic.Gui.WPF
{
	public partial class ChartControl
	{
		private UndoHistory<NavHistoryRecord> m_nav_history = null!;

		/// <summary>Set up the chart for MouseOps</summary>
		private void InitNavigation()
		{
			m_nav_history = new UndoHistory<NavHistoryRecord>
			{
				IsDuplicate = NavHistoryRecord.ApproxEqual,
				ApplySnapshot = snapshot =>
				{
					XAxis.Range = snapshot.XRange;
					YAxis.Range = snapshot.YRange;
					SetCameraFromRange();
					Invalidate();
				},
			};
		}

		/// <summary>Enable/Disable mouse navigation</summary>
		public bool DefaultMouseControl
		{
			get;
			set
			{
				if (field == value) return;
				field = value;
				NotifyPropertyChanged(nameof(DefaultMouseControl));
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
				NotifyPropertyChanged(nameof(DefaultKeyboardShortcuts));
			}
		}

		/// <summary>Mouse/key events on the chart</summary>
		protected override void OnMouseDown(MouseButtonEventArgs args)
		{
			//
			//  *** Use PreviewMouseDown to set pending MouseOps ***
			//  *** Don't set e.Handled = true, SetPending() is enough ***
			//
			// Notes:
			//  - MouseButton events in WPF are routed events that behave differently to WinForms.
			//  - The order of handlers for MouseDown is:
			//     1) Registered class handlers
			//     2) OnMouseDown - base.OnMouseDown() does *not* raise the MouseDown event
			//     3) Handers attached to event MouseDown
			//  - MouseDown is a 'bubbling' event, it starts at the visual tree leaf element and
			//    bubbles up the tree, stopping when a handle sets 'e.Handled = true'
			//  - PreviewMouseDown is a tunnelling 'event', it starts at the window and drills
			//    down the tree to the leaves. If 'e.Handled = true' in a PreviewMouseDown handler
			//    then MouseDown is never raised, and override OnMouseDown isn't called.

			base.OnMouseDown(args);

			// If a mouse op is already active, ignore mouse down
			if (MouseOperations.Active != null)
				return;

			// Look for the mouse op to perform
			if (MouseOperations.Pending[args.ChangedButton] == null && DefaultMouseControl)
			{
				switch (args.ChangedButton)
				{
					case MouseButton.Left: MouseOperations.Pending[args.ChangedButton] = new MouseOpDefaultLButton(this); break;
					case MouseButton.Middle: MouseOperations.Pending[args.ChangedButton] = new MouseOpDefaultMButton(this); break;
					case MouseButton.Right: MouseOperations.Pending[args.ChangedButton] = new MouseOpDefaultRButton(this); break;
					case MouseButton.XButton1: UndoNavigation(); break;
					case MouseButton.XButton2: RedoNavigation(); break;
					default: return;
				}
			}

			// Start the next mouse op
			MouseOperations.BeginOp(args.ChangedButton);

			// Get the mouse op, save mouse location and hit test data, then call op.MouseDown()
			var op = MouseOperations.Active;
			if (op != null && !op.Cancelled)
			{
				// Don't capture the mouse here in mouse down because that prevents
				// mouse up messages getting to subscribers of the MouseUp event.
				// Only capture the mouse when we know it's a drag operation.
				var client_point = args.GetPosition(this);
				var scene_point = args.GetPosition(Scene).ToV2();

				op.GrabScene = op.DropScene = scene_point;
				op.GrabChart = op.DropChart = SceneToChart(scene_point);
				op.HitResult = HitTest(client_point, Keyboard.Modifiers, args.ToMouseBtns(), null);
				op.MouseDown(args);
			}
		}
		protected override void OnMouseMove(MouseEventArgs args)
		{
			base.OnMouseMove(args);
			var client_point = args.GetPosition(this);

			// Look for the mouse op to perform
			var op = MouseOperations.Active;
			if (op != null)
			{
				if (!op.Cancelled)
				{
					op.DropScene = Gui_.MapPoint(this, Scene, client_point).ToV2();
					op.DropChart = SceneToChart(op.DropScene);
					op.MouseMove(args);
				}
			}
			// Otherwise, provide mouse hover detection
			else if (SceneBounds != Rect_.Zero)
			{
				var hit = HitTest(client_point, Keyboard.Modifiers, args.ToMouseBtns());
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

				// Notify that the chart coordinate at the mouse pointer has changed
				if (hit.Zone == EZone.Chart && ShowValueAtPointer)
					NotifyPropertyChanged(nameof(ValueAtPointer));
			}
		}
		protected override void OnMouseUp(MouseButtonEventArgs args)
		{
			base.OnMouseUp(args);

			// Look for the mouse op to perform
			var op = MouseOperations.Active;
			if (op != null && !op.Cancelled)
				op.MouseUp(args);

			MouseOperations.EndOp(args.ChangedButton);
		}
		protected override void OnMouseWheel(MouseWheelEventArgs args)
		{
			base.OnMouseWheel(args);

			// If there is a mouse op in progress, forward the event
			var op = MouseOperations.Active;
			if (op != null && !op.Cancelled)
			{
				op.MouseWheel(args);
				if (args.Handled)
					return;
			}

			var client_point = args.GetPosition(this);
			var along_ray = Options.MouseCentredZoom || Keyboard.Modifiers.HasFlag(ModifierKeys.Alt);
			var chart_point = SceneToChart(Gui_.MapPoint(this, Scene, client_point).ToV2());
			var hit = HitTestZone(client_point, Keyboard.Modifiers, args.ToMouseBtns());

			// Batch mouse wheel events into 100ms groups
			var defer_nav_checkpoint = DeferNavCheckpoints();
			Dispatcher_.BeginInvokeDelayed(() =>
			{
				Util.Dispose(ref defer_nav_checkpoint!);
				SaveNavCheckpoint();
			}, TimeSpan.FromMilliseconds(100));

			var scale = 0.001f;
			if (Keyboard.Modifiers.HasFlag(ModifierKeys.Shift)) scale *= 0.1f;
			if (Keyboard.Modifiers.HasFlag(ModifierKeys.Alt)) scale *= 0.01f;
			var delta = Math_.Clamp(args.Delta * scale, -0.999f, 0.999f);
			var chg = (string?)null;

			// If zooming is allowed on both axes, translate the camera
			if (hit.Zone == EZone.Chart && XAxis.AllowZoom && YAxis.AllowZoom)
			{
				// Translate the camera along a ray through 'point'
				var scene_point = Gui_.MapPoint(this, Scene, client_point);
				Scene.Window.MouseNavigateZ(scene_point.ToPointI(), args.ToMouseBtns(Keyboard.Modifiers), args.Delta, along_ray);
				chg = nameof(SetRangeFromCamera);
			}
			
			// Otherwise, zoom on the allowed axis only
			else if (hit.Zone == EZone.XAxis || (hit.Zone == EZone.Chart && !YAxis.AllowZoom))
			{
				if (hit.ModifierKeys.HasFlag(ModifierKeys.Control) && XAxis.AllowScroll)
				{
					// Scroll the XAxis
					XAxis.Shift(XAxis.Span * delta);
					chg = nameof(SetCameraFromRange);
				}
				else if (!hit.ModifierKeys.HasFlag(ModifierKeys.Control) && XAxis.AllowZoom)
				{
					// Change the aspect ratio by zooming on the XAxis
					var x = along_ray ? chart_point.x : XAxis.Centre;
					var left = (XAxis.Min - x) * (1f - delta);
					var rite = (XAxis.Max - x) * (1f - delta);
					XAxis.Set(x + left, x + rite);
					if (Options.LockAspect != null)
						YAxis.Span *= (1f - delta);

					chg = nameof(SetCameraFromRange);
				}
			}
			else if (hit.Zone == EZone.YAxis || (hit.Zone == EZone.Chart && !XAxis.AllowZoom))
			{
				if (hit.ModifierKeys.HasFlag(ModifierKeys.Control) && YAxis.AllowScroll)
				{
					// Scroll the YAxis
					YAxis.Shift(YAxis.Span * delta);
					chg = nameof(SetCameraFromRange);
				}
				else if (!hit.ModifierKeys.HasFlag(ModifierKeys.Control) && YAxis.AllowZoom)
				{
					// Change the aspect ratio by zooming on the YAxis
					var y = along_ray ? chart_point.y : YAxis.Centre;
					var left = (YAxis.Min - y) * (1f - delta);
					var rite = (YAxis.Max - y) * (1f - delta);
					YAxis.Set(y + left, y + rite);
					if (Options.LockAspect != null)
						XAxis.Span *= (1f - delta);

					chg = nameof(SetCameraFromRange);
				}
			}

			switch (chg)
			{
				// Update the axes from the camera position
				case nameof(SetRangeFromCamera):
				{
					SetRangeFromCamera();
					NotifyPropertyChanged(nameof(ValueAtPointer));
					Invalidate();
					break;
				}
				// Set the camera position from the new axis ranges
				case nameof(SetCameraFromRange):
				{
					SetCameraFromRange();
					NotifyPropertyChanged(nameof(ValueAtPointer));
					Invalidate();
					break;
				}
			}
		}

		/// <summary>Handle key events</summary>
		protected override void OnKeyDown(KeyEventArgs args)
		{
			// *** Use PreviewKeyDown to add MouseOps ***
			SetCursor();

			var op = MouseOperations.Active;
			if (op != null && !args.Handled)
				op.OnKeyDown(args);

			// If the current mouse operation doesn't use the key,
			// see if it's a default keyboard shortcut.
			if (!args.Handled && DefaultKeyboardShortcuts)
				OnTranslateKey(args);

			base.OnKeyDown(args);
		}
		protected override void OnKeyUp(KeyEventArgs args)
		{
			SetCursor();

			var op = MouseOperations.Active;
			if (op != null && !args.Handled)
				op.OnKeyUp(args);

			base.OnKeyUp(args);
		}

		/// <summary>Set the mouse cursor based on key state</summary>
		protected virtual void SetCursor()
		{
			// Sub class to do something like this:
			// if (ModifierKeys.HasFlag(Keys.Shift))   Cursor = Cursors.ArrowPlus;
			// if (ModifierKeys.HasFlag(Keys.Control)) Cursor = Cursors.ArrowMinus;
			Cursor = Cursors.Arrow;
		}

		/// <summary>Raised when the chart is clicked with the mouse</summary>
		public event EventHandler<ChartClickedEventArgs>? ChartClicked;
		protected virtual void OnChartClicked(ChartClickedEventArgs args)
		{
			ChartClicked?.Invoke(this, args);
		}

		/// <summary>Raised when the chart is dragged with the mouse</summary>
		public event EventHandler<ChartDraggedEventArgs>? ChartDragged;
		protected virtual void OnChartDragged(ChartDraggedEventArgs args)
		{
			// This allows custom behaviour of a mouse drag on the chart
			ChartDragged?.Invoke(this, args);
		}

		/// <summary>Raised when the chart is area selected</summary>
		public event EventHandler<ChartAreaSelectEventArgs>? ChartAreaSelect;
		protected virtual void OnChartAreaSelect(ChartAreaSelectEventArgs args)
		{
			ChartAreaSelect?.Invoke(this, args);

			// Select chart elements by default
			if (!args.Handled)
			{
				switch (Options.AreaSelectMode)
				{
					case EAreaSelectMode.Disabled:
					{
						break;
					}
					case EAreaSelectMode.SelectElements:
					{
						SelectElements(args.SelectionVolume, Keyboard.Modifiers, args.MouseBtns);
						break;
					}
					case EAreaSelectMode.Zoom:
					{
						PositionChart(args.SelectionVolume);
						break;
					}
					case EAreaSelectMode.ZoomIfNoSelection:
					{
						if (Selected.Count == 0)
							PositionChart(args.SelectionVolume);
						break;
					}
					default:
					{
						throw new Exception("Unknown area select mode");
					}
				}
			}
		}
		private bool DoChartAreaSelect(ModifierKeys modifier_keys)
		{
			if (Options.AreaSelectRequiresShiftKey && !modifier_keys.HasFlag(ModifierKeys.Shift))
				return false;

			switch (Options.AreaSelectMode)
			{
				case EAreaSelectMode.Disabled: return false;
				case EAreaSelectMode.SelectElements: return true;
				case EAreaSelectMode.Zoom: return true;
				case EAreaSelectMode.ZoomIfNoSelection: return Selected.Count == 0;
				default: throw new Exception($"Unknown area selection mode: {Options.AreaSelectMode}");
			}
		}

		/// <summary>Adjust the X/Y axis of the chart to view the given volume</summary>
		public void PositionChart(BBox chart_bbox)
		{
			// Ensure the range is positive definite
			var rng_x = new RangeF(chart_bbox.MinX, chart_bbox.MaxX);
			var rng_y = new RangeF(chart_bbox.MinY, chart_bbox.MaxY);
			if (rng_x.Size < float.Epsilon) // float because chart_bbox is floats
			{
				rng_x.beg = chart_bbox.Centre.x * 0.99999;
				rng_x.end = chart_bbox.Centre.x * 1.00001;
			}
			if (rng_y.Size < float.Epsilon) // float because chart_bbox is floats
			{
				rng_y.beg = chart_bbox.Centre.y * 0.99999;
				rng_y.end = chart_bbox.Centre.y * 1.00001;
			}

			XAxis.Set(rng_x.beg, rng_x.end);
			YAxis.Set(rng_y.beg, rng_y.end);
			SetCameraFromRange();

			//// Ensure the axis range is >= 1 pixel for width/height
			//var chart_1px = SceneToChart(new Rect(new Size(1,1)));
			//XAxis.Set(chart_bbox.MinX, Math.Max(chart_bbox.MaxX, chart_bbox.MinX + chart_1px.SizeX));
			//YAxis.Set(chart_bbox.MinY, Math.Max(chart_bbox.MaxY, chart_bbox.MinY + chart_1px.SizeY));
			//SetCameraFromRange();
		}

		/// <summary>Shifts the X and Y range of the chart so that chart space position 'chart_point' is at client space position 'client_point'</summary>
		public void PositionChart(v4 chart_point, v2 scene_point)
		{
			var dst = SceneToChart(scene_point);
			var c2w = Scene.Camera.O2W;
			c2w.pos += c2w.x * (float)(chart_point.x - dst.x) + c2w.y * (float)(chart_point.y - dst.y);
			Scene.Camera.O2W = c2w;
			Invalidate();
		}

		/// <summary>Handle navigation keyboard shortcuts</summary>
		public virtual void OnTranslateKey(KeyEventArgs e)
		{
			// Notes:
			//  - Default chart key bindings. These are intended as default key bindings
			//    Applications should probably set DefaultKeyboardShortcuts to false and
			//    handle key bindings themselves.

			if (e.Handled)
				return;

			// Allow users to handle the key
			TranslateKey?.Invoke(this, e);
			if (e.Handled)
				return;

			// Fall back to default
			switch (e.Key)
			{
				case Key.Escape:
				{
					// Deselect all on escape
					if (Options.AllowSelection)
					{
						Selected.Clear();
						Invalidate();
						e.Handled = true;
					}
					break;
				}
				case Key.Delete:
				{
					//if (AllowEditing)
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
				case Key.F5:
				{
					View3d.ReloadScriptSources();
					Invalidate();
					e.Handled = true;
					break;
				}
				case Key.F7:
				{
					var bounds =
						Keyboard.Modifiers.HasFlag(ModifierKeys.Shift) ? Gfx.View3d.ESceneBounds.Selected :
						Keyboard.Modifiers.HasFlag(ModifierKeys.Control) ? Gfx.View3d.ESceneBounds.Visible :
						Gfx.View3d.ESceneBounds.All;

					AutoRange(who: bounds);
					Invalidate();
					e.Handled = true;
					break;
				}
				case Key.A:
				{
					if (Keyboard.Modifiers.HasFlag(ModifierKeys.Control))
					{
						Selected.Clear();
						Selected.AddRange(Elements);
						Invalidate();
						Debug.Assert(CheckConsistency());
						e.Handled = true;
					}
					break;
				}
			}
		}
		public event EventHandler<KeyEventArgs>? TranslateKey;

		/// <summary>Remove all nav checkpoint history</summary>
		public void ClearNavHistory()
		{
			m_nav_history.Clear();
		}

		/// <summary>Add an undo record to the navigation history</summary>
		public void SaveNavCheckpoint()
		{
			m_nav_history.Add(new NavHistoryRecord(this));
		}

		/// <summary>Prevent navigation check points from being added to the history</summary>
		public IDisposable DeferNavCheckpoints()
		{
			return m_nav_history.Defer();
		}

		/// <summary>Go back to the previous chart position</summary>
		public void UndoNavigation()
		{
			m_nav_history.Undo();
		}

		/// <summary>Go forward to the next chart position</summary>
		public void RedoNavigation()
		{
			m_nav_history.Redo();
		}

		/// <summary>A snapshot of the axis range</summary>
		private class NavHistoryRecord
		{
			public NavHistoryRecord(ChartControl chart)
			{
				XRange = chart.XAxis.Range;
				YRange = chart.YAxis.Range;
			}

			/// <summary>The chart range at the snapshot</summary>
			public RangeF XRange { get; }
			public RangeF YRange { get; }

			/// <summary>True if 'lhs' and 'rhs' are very similar</summary>
			public static bool ApproxEqual(NavHistoryRecord lhs, NavHistoryRecord rhs)
			{
				// Use a tolerance of 1% of the smallest range
				var tol = 0.01 * Math.Min(Math.Min(lhs.XRange.Size, lhs.YRange.Size), Math.Min(rhs.XRange.Size, rhs.YRange.Size));
				return
					Math_.FEqlAbsolute(lhs.XRange.Beg, rhs.XRange.Beg, tol) &&
					Math_.FEqlAbsolute(lhs.XRange.End, rhs.XRange.End, tol) &&
					Math_.FEqlAbsolute(lhs.YRange.Beg, rhs.YRange.Beg, tol) &&
					Math_.FEqlAbsolute(lhs.YRange.End, rhs.YRange.End, tol);
			}
		}

		/// <summary>Area selection mode</summary>
		public enum EAreaSelectMode
		{
			Disabled,

			/// <summary>Elements within the dragged area become selected</summary>
			SelectElements,

			/// <summary>The dragged out area becomes the new chart bounds</summary>
			Zoom,

			/// <summary>Only area-zoom if there are no elements in the selected state</summary>
			ZoomIfNoSelection,
		}
	}
}
