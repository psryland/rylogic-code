using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using System.Windows;
using System.Windows.Input;
using System.Windows.Interop;
using System.Windows.Media;
using Rylogic.Common;
using Rylogic.Interop.Win32;
using Rylogic.Utility;

namespace Rylogic.Gui.WPF
{
	public class RawInput : IDisposable
	{
		// Notes:
		//  - RawInput needs to be enabled on a per-window basis because it consumes all WM_INPUT messages for the window.
		//    To have multiple components handling raw input, they must share one instance for that window.
		//  - During mouse drags, high-frequency WM_MOUSEMOVE, WM_NCHITTEST, and WM_SETCURSOR messages flood the Win32
		//    message pump, completely starving the WPF compositor and causing multi-second rendering freezes.
		//    RIDEV_NOLEGACY suppresses all legacy mouse messages and we read mouse input from WM_INPUT instead.
		//  - Uses GetRawInputBuffer to drain all queued raw input in one call, avoiding missed events.
		//  - Button, wheel, and move events all fire immediately. Throttling is the caller's responsibility
		//    (e.g. via CompositionTarget.Rendering in the MouseOp).
		//  - Use GetPosition(Visual) for cursor position (uses GetCursorPos, always works regardless of RIDEV_NOLEGACY).
		//  - Use MouseBtns property for button state (tracked from WM_INPUT, reliable unlike WPF's Mouse.LeftButton).

		public interface IInputSink
		{
			void MouseDown(System.Windows.Input.MouseButtonEventArgs? e);
			void MouseMove(System.Windows.Input.MouseEventArgs e);
			void MouseUp(System.Windows.Input.MouseButtonEventArgs e);
			void MouseWheel(System.Windows.Input.MouseWheelEventArgs e);
			void OnKeyDown(System.Windows.Input.KeyEventArgs e);
			void OnKeyUp(System.Windows.Input.KeyEventArgs e);
		}

		private List<IInputSink> m_recv = [];
		private IntPtr m_buffer;
		private int m_buffer_size;
		private EMouseBtns m_button_state;

		/// <summary>Maps RI_MOUSE_*_DOWN flags to MouseButton and EMouseBtns values. UP flag is always DOWN &lt;&lt; 1.</summary>
		private static readonly (uint down_flag, MouseButton button, EMouseBtns mask)[] ButtonMap =
		[
			(Win32.RI_MOUSE_LEFT_BUTTON_DOWN, MouseButton.Left, EMouseBtns.Left),
			(Win32.RI_MOUSE_RIGHT_BUTTON_DOWN, MouseButton.Right, EMouseBtns.Right),
			(Win32.RI_MOUSE_MIDDLE_BUTTON_DOWN, MouseButton.Middle, EMouseBtns.Middle),
			(Win32.RI_MOUSE_BUTTON_4_DOWN, MouseButton.XButton1, EMouseBtns.XButton1),
			(Win32.RI_MOUSE_BUTTON_5_DOWN, MouseButton.XButton2, EMouseBtns.XButton2),
		];

		public RawInput(int buffer_count = 256)
		{
			// Pre-allocate buffer for GetRawInputBuffer — room for multiple QWORD-aligned records
			var record_size = Align8(Marshal.SizeOf<Win32.RAWINPUTHEADER>() + Marshal.SizeOf<Win32.RAWMOUSE>());
			m_buffer_size = buffer_count * record_size;
			m_buffer = Marshal.AllocHGlobal(m_buffer_size);
		}
		public void Dispose()
		{
			Enabled = false;

			if (m_buffer != IntPtr.Zero)
				Marshal.FreeHGlobal(m_buffer);

			m_buffer = IntPtr.Zero;
			m_buffer_size = 0;
		}

		/// <summary>The window to use RawInput on</summary>
		public HwndSource? Source { get; set; }

		/// <summary>Call to receive raw input</summary>
		public IDisposable Subscribe(IInputSink receiver)
		{
			return Scope.Create(
				() => { m_recv.Add(receiver); Enabled = m_recv.Count != 0; },
				() => { m_recv.Remove(receiver); Enabled = m_recv.Count != 0; }
			);
		}

		/// <summary>True if raw input is enabled and legacy input is disabled</summary>
		public bool Enabled
		{
			get;
			private set // Do *NOT* make this public, force use of Subscribe() so that multiple receivers can share one RawInput
			{
				if (Enabled == value)
					return;

				if (field) // currently enabled, now disabling
				{
					LegacyMouseMessages = true;
				}
				field = value;
				if (field) // currently disabled, now enabling
				{
					// Seed button state from hardware so we know which buttons are already held
					m_button_state = EMouseBtns.None;
					if (Win32.KeyDownAsync(EKeyCodes.LButton)) m_button_state |= EMouseBtns.Left;
					if (Win32.KeyDownAsync(EKeyCodes.RButton)) m_button_state |= EMouseBtns.Right;
					if (Win32.KeyDownAsync(EKeyCodes.MButton)) m_button_state |= EMouseBtns.Middle;
					if (Win32.KeyDownAsync(EKeyCodes.XButton1)) m_button_state |= EMouseBtns.XButton1;
					if (Win32.KeyDownAsync(EKeyCodes.XButton2)) m_button_state |= EMouseBtns.XButton2;

					LegacyMouseMessages = false;
				}
			}
		}

		/// <summary>Drain all queued raw input and dispatch events to receivers. Call this from your render/update loop.</summary>
		public void Poll()
		{
			var header_size = (uint)Marshal.SizeOf<Win32.RAWINPUTHEADER>();

			for (; ; )
			{
				var size = (uint)m_buffer_size;
				var count = (int)User32.GetRawInputBuffer(m_buffer, ref size, header_size);
				if (count <= 0)
					break;

				// Walk the variable-sized, QWORD-aligned records using header.dwSize (NEXTRAWINPUTBLOCK)
				var ptr = m_buffer;
				for (int i = 0; i != count; ++i)
				{
					var header = Marshal.PtrToStructure<Win32.RAWINPUTHEADER>(ptr);
					if (header.dwType == Win32.RIM_TYPEMOUSE)
					{
						var mouse = Win32.RAWINPUT.ReadData<Win32.RAWMOUSE>(ptr);
						HandleMouseInput(ref mouse);
					}

					ptr += Align8((int)header.dwSize);
				}
			}

			// Process a single RAWMOUSE record — update button state and fire events
			void HandleMouseInput(ref Win32.RAWMOUSE mouse)
			{
				var button_flags = mouse.usButtonFlags;

				// Update tracked button state and fire button events.
				// The RI_MOUSE flags are paired: DOWN at bit 2n, UP at bit 2n+1.
				foreach (var (down_flag, button, mask) in ButtonMap)
				{
					if (Bit.AllSet(button_flags, down_flag))
					{
						m_button_state |= mask;

						var args = new MouseButtonEventArgs(MouseBtns, button, UIElement.MouseDownEvent);
						foreach (var r in m_recv.ToArray())
							r.MouseDown(args);
					}
					if (Bit.AllSet(button_flags, down_flag << 1))
					{
						var args = new MouseButtonEventArgs(MouseBtns, button, UIElement.MouseUpEvent);
						foreach (var r in m_recv.ToArray())
							r.MouseUp(args);

						m_button_state &= ~mask;
					}
				}

				// Wheel events (cast to short for correct sign on scroll-down)
				if (Bit.AllSet(button_flags, Win32.RI_MOUSE_WHEEL))
				{
					var args = new MouseWheelEventArgs(MouseBtns, unchecked((short)mouse.usButtonData));
					foreach (var r in m_recv.ToArray())
						r.MouseWheel(args);
				}

				// Mouse movement
				if (mouse.lLastX != 0 || mouse.lLastY != 0)
				{
					var args = new MouseEventArgs(MouseBtns, UIElement.MouseMoveEvent);
					foreach (var r in m_recv.ToArray())
						r.MouseMove(args);
				}
			}
		}

		/// <summary>Enable or suppress legacy mouse messages</summary>
		private bool LegacyMouseMessages
		{
			get;
			set
			{
				if (LegacyMouseMessages == value || Source == null)
					return;

				if (LegacyMouseMessages)
				{
					// Legacy was enabled, now suppress with RIDEV_NOLEGACY
					var rid = new Win32.RAWINPUTDEVICE[]
					{
						new()
						{
							usUsagePage = 0x01,  // HID_USAGE_PAGE_GENERIC
							usUsage = 0x02,      // HID_USAGE_GENERIC_MOUSE
							dwFlags = Win32.RIDEV_NOLEGACY,
							hwndTarget = Source.Handle,
						}
					};
					User32.RegisterRawInputDevices(rid, 1, (uint)Marshal.SizeOf<Win32.RAWINPUTDEVICE>());
				}

				field = value;

				if (LegacyMouseMessages)
				{
					// Legacy now re-enabled, remove raw input registration
					var rid = new Win32.RAWINPUTDEVICE[]
					{
						new()
						{
							usUsagePage = 0x01,
							usUsage = 0x02,
							dwFlags = Win32.RIDEV_REMOVE,
							hwndTarget = IntPtr.Zero,
						}
					};
					User32.RegisterRawInputDevices(rid, 1, (uint)Marshal.SizeOf<Win32.RAWINPUTDEVICE>());
				}
			}
		} = true;

		/// <summary>Round up to QWORD (8-byte) alignment</summary>
		private static int Align8(int value) => (value + 7) & ~7;

		/// <summary>Get the current mouse position relative to a visual element. Uses GetCursorPos() which always works regardless of RIDEV_NOLEGACY.</summary>
		public static Point GetPosition(Visual visual)
		{
			var pt = User32.GetCursorPos();
			return visual.PointFromScreen(new Point(pt.X, pt.Y));
		}

		/// <summary>Get the current mouse button and modifier key state, tracked from WM_INPUT button events.</summary>
		public EMouseBtns MouseBtns
		{
			get
			{
				var res = m_button_state;
				if (Keyboard.Modifiers.HasFlag(ModifierKeys.Shift)) res |= EMouseBtns.Shift;
				if (Keyboard.Modifiers.HasFlag(ModifierKeys.Control)) res |= EMouseBtns.Ctrl;
				if (Keyboard.Modifiers.HasFlag(ModifierKeys.Alt)) res |= EMouseBtns.Alt;
				return res;
			}
		}

		/// <summary>MouseButtonEventArgs with reliable button state from RawInput tracking (not WPF's stale Mouse.LeftButton etc.)</summary>
		public class MouseButtonEventArgs : System.Windows.Input.MouseButtonEventArgs
		{
			public MouseButtonEventArgs(EMouseBtns btns, MouseButton button, RoutedEvent routed_event)
				: base(Mouse.PrimaryDevice, 0, button)
			{
				MouseBtns = btns;
				RoutedEvent = routed_event;
			}

			/// <summary>Reliable button+modifier state tracked from WM_INPUT</summary>
			public EMouseBtns MouseBtns { get; }
		}

		/// <summary>MouseEventArgs with reliable button state from RawInput tracking</summary>
		public class MouseEventArgs : System.Windows.Input.MouseEventArgs
		{
			public MouseEventArgs(EMouseBtns btns, RoutedEvent routed_event)
				: base(Mouse.PrimaryDevice, 0)
			{
				MouseBtns = btns;
				RoutedEvent = routed_event;
			}

			/// <summary>Reliable button+modifier state tracked from WM_INPUT</summary>
			public EMouseBtns MouseBtns { get; }
		}

		/// <summary>MouseWheelEventArgs with reliable button state from RawInput tracking</summary>
		public class MouseWheelEventArgs : System.Windows.Input.MouseWheelEventArgs
		{
			public MouseWheelEventArgs(EMouseBtns btns, int delta)
				: base(Mouse.PrimaryDevice, 0, delta)
			{
				MouseBtns = btns;
				RoutedEvent = UIElement.MouseWheelEvent;
			}

			/// <summary>Reliable button+modifier state tracked from WM_INPUT</summary>
			public EMouseBtns MouseBtns { get; }
		}
	}
}
