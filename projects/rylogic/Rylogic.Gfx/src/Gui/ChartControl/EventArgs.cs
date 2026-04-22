using System;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Input;
using System.Windows.Media;
using Rylogic.Common;
using Rylogic.Gfx;
using Rylogic.Maths;

namespace Rylogic.Gui.WPF
{
	public partial class ChartControl
	{
		/// <summary>Event args for the chart changed event</summary>
		public class ChartChangedEventArgs : EventArgs
		{
			// Note: many events are available by attaching to the Elements binding list
			public ChartChangedEventArgs(EChangeType ty)
			{
				ChgType = ty;
				Cancel = false;
			}

			/// <summary>The type of change that occurred</summary>
			public EChangeType ChgType { get; private set; }

			/// <summary>A cancel property for "about to change" events</summary>
			public bool Cancel { get; set; }
		}

		/// <summary>Event args for the ChartMoved event</summary>
		public class ChartMovedEventArgs : EventArgs
		{
			public ChartMovedEventArgs(EMoveType move_type)
			{
				MoveType = move_type;
			}

			/// <summary>How the chart was moved</summary>
			public EMoveType MoveType { get; internal set; }
		}

		/// <summary>Event args for the ChartClicked event</summary>
		public class ChartClickedEventArgs :EventArgs
		{
			// Notes:
			//  - MouseButtonEventArgs.ClickCount is always 1 for mouse up events in windows, so to 
			//    support multiple clicks on the chart I'm allowing the click count to be provided externally.

			private readonly MouseButtonEventArgs m_mouse_event;
			public ChartClickedEventArgs(HitTestResult hits, MouseButtonEventArgs mouse_event, int click_count = 0)
			{
				m_mouse_event = mouse_event;
				HitResult = hits;
				ClickCount = Math.Max(mouse_event.ClickCount, click_count);
				Handled = false;
			}

			/// <summary>Results of a hit test performed at the click location</summary>
			public HitTestResult HitResult { get; }

			/// <summary>The number of times the changed button was clicked.</summary>
			public int ClickCount { get; }

			/// <summary>The button that was clicked</summary>
			public MouseButton ChangedButton => m_mouse_event.ChangedButton;

			/// <summary>The state of the clicked button</summary>
			public MouseButtonState ButtonState => m_mouse_event.ButtonState;

			/// <summary>the current state of the left mouse button.</summary>
			public MouseButtonState LeftButton => m_mouse_event.LeftButton;

			/// <summary>The current state of the right mouse button.</summary>
			public MouseButtonState RightButton => m_mouse_event.RightButton;

			/// <summary>The current state of the middle mouse button.</summary>
			public MouseButtonState MiddleButton => m_mouse_event.MiddleButton;

			/// <summary>The current state of the first extended mouse button.</summary>
			public MouseButtonState XButton1 => m_mouse_event.XButton1;

			/// <summary>The current state of the second extended mouse button.</summary>
			public MouseButtonState XButton2 => m_mouse_event.XButton2;

			/// <summary>The position of the mouse pointer relative to the specified element.</summary>
			public Point GetPosition(IInputElement relativeTo) => m_mouse_event.GetPosition(relativeTo);

			/// <summary>Set to true to suppress default chart click behaviour</summary>
			public bool Handled { get; set; }
		}

		/// <summary>Event args raised when a 3D scene object is clicked (e.g. via Ctrl+Left click)</summary>
		public class SceneObjectClickedEventArgs :EventArgs
		{
			public SceneObjectClickedEventArgs(View3d.HitTestResult hit, ModifierKeys modifiers, MouseButton button)
			{
				Hit = hit;
				Modifiers = modifiers;
				Button = button;
				Handled = false;
			}

			/// <summary>The native hit test result (may have no hit)</summary>
			public View3d.HitTestResult Hit { get; }

			/// <summary>The hit object, or null if the ray missed every object</summary>
			public View3d.Object? HitObject => Hit.HitObject;

			/// <summary>The keyboard modifiers active at the time of the click</summary>
			public ModifierKeys Modifiers { get; }

			/// <summary>The mouse button that was clicked</summary>
			public MouseButton Button { get; }

			/// <summary>Set to true to suppress any default behaviour</summary>
			public bool Handled { get; set; }
		}

		/// <summary>Event args for when an element or elements is dragged within the chart</summary>
		public class ChartDraggedEventArgs :EventArgs
		{
			public ChartDraggedEventArgs(HitTestResult hits, v4 delta, EDragState state)
			{
				HitResult = hits;
				Delta = delta;
				State = state;
				Handled = false;
			}

			/// <summary>Results of a hit test performed at the click location</summary>
			public HitTestResult HitResult { get; }

			/// <summary>The drag vector from the start position (in chart space)</summary>
			public v4 Delta { get; }

			/// <summary>True when the drag is to be committed (i.e. at the end of a drag and escape not pressed)</summary>
			public EDragState State { get; }

			/// <summary>Set to true to suppress default chart click behaviour</summary>
			public bool Handled { get; set; }
		}

		/// <summary>Event args for area selection</summary>
		public class ChartAreaSelectEventArgs : EventArgs
		{
			public ChartAreaSelectEventArgs(BBox chart_selection_volume, EMouseBtns mouse_btns)
			{
				SelectionVolume = chart_selection_volume;
				MouseBtns = mouse_btns;
				Handled = false;
			}

			/// <summary>The volume of the selection (in chart space)</summary>
			public BBox SelectionVolume { get; }

			/// <summary>The current mouse button states</summary>
			public EMouseBtns MouseBtns { get; }

			/// <summary>Set to true to suppress default chart click behaviour</summary>
			public bool Handled { get; set; }
		}

		/// <summary>Event args for FindingDefaultRange</summary>
		public class FindingDefaultRangeEventArgs : EventArgs
		{
			public FindingDefaultRangeEventArgs(EAxis axis, RangeF xrange, RangeF yrange)
			{
				Axis = axis;
				XRange = xrange;
				YRange = yrange;
			}

			/// <summary>The axis being ranged</summary>
			public EAxis Axis { get; }

			/// <summary>The XAxis range calculated from known chart elements data</summary>
			public RangeF XRange { get; set; }

			/// <summary>The YAxis range calculated from known chart elements data</summary>
			public RangeF YRange { get; set; }
		}

		/// <summary>Customise context menu event args</summary>
		public class AddUserMenuOptionsEventArgs : EventArgs
		{
			public AddUserMenuOptionsEventArgs(ContextMenu menu, HitTestResult hit_result)
			{
				Menu = menu;
				HitResult = hit_result;
			}

			/// <summary>The zone on the chart where the context menu is to be displayed</summary>
			public EZone Zone => HitResult.Zone;

			/// <summary>The menu to add menu items to</summary>
			public ContextMenu Menu { get; }

			/// <summary>The hit test result at the click location</summary>
			public HitTestResult HitResult { get; }
		}

		/// <summary>Event args for the post-paint add overlays call</summary>
		public class AddOverlaysOnPaintEventArgs : EventArgs
		{
			public AddOverlaysOnPaintEventArgs(ChartControl chart, DrawingContext gfx, m4x4 chart_to_client, EZone zone)
			{
				Chart = chart;
				Gfx = gfx;
				ChartToClient = chart_to_client;
				Zone = zone;
			}

			/// <summary>The device context to draw to</summary>
			public DrawingContext Gfx { get; }

			/// <summary>The chart being drawn on</summary>
			public ChartControl Chart { get; }

			/// <summary>Transform from Chart space to client space</summary>
			public m4x4 ChartToClient { get; }

			/// <summary>The part of the chart being painted</summary>
			public EZone Zone { get; }
		}

		/// <summary>Event args for the auto range event</summary>
		public class AutoRangeEventArgs : EventArgs
		{
			public AutoRangeEventArgs(View3d.ESceneBounds who, EAxis axes)
			{
				Who = who;
				Axes = axes;
				ViewBBox = BBox.Reset;
				Handled = false;
			}

			/// <summary>The scene elements to be auto ranged</summary>
			public View3d.ESceneBounds Who { get; }

			/// <summary>The axes to auto range on</summary>
			public EAxis Axes { get; }

			/// <summary>The bounding box of the range to view</summary>
			public BBox ViewBBox { get; set; }

			/// <summary>Set to true to indicate that the provided range should be used</summary>
			public bool Handled { get; set; }
		}
	}
}