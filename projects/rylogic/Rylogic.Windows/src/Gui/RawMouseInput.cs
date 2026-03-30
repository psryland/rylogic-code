using System;
using System.Runtime.InteropServices;
using System.Windows;
using System.Windows.Input;
using System.Windows.Interop;
using Rylogic.Common;
using Rylogic.Interop.Win32;

namespace Rylogic.Gui
{
	public class RawMouseInput : IDisposable
	{
		// Notes:
		//  - During mouse drags, high-frequency WM_MOUSEMOVE, WM_NCHITTEST, and WM_SETCURSOR messages flood the Win32 message pump,
		//    completely starving the WPF compositor and causing multi-second rendering freezes. RIDEV_NOLEGACY suppresses all legacy mouse messages
		//    and we read mouse input from WM_INPUT instead. CompositionTarget.Rendering drives the rendering at compositor frame rate.
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

		/// <summary>Mouse events</summary>
		public EventHandler<MouseButtonEventArgs>? MouseDown;
		public EventHandler<MouseButtonEventArgs>? MouseUp;
		public EventHandler<MouseEventArgs>? MouseMove;
		public EventHandler<MouseWheelEventArgs>? MouseWheel;

		/// <summary>True if raw input is enable and legacy messages are suppressed</summary>
		public bool Enabled
		{
			get;
			set
			{
				if (Enabled == value || Source == null)
					return;

				if (Enabled)
				{
					LegacyMouseMessages = true;
					Source?.RemoveHook(HandleRawInput);
				}

				field = value;

				if (Enabled)
				{
					Source?.AddHook(HandleRawInput); // Install WM_INPUT handler and suppress legacy mouse messages
					LegacyMouseMessages = false;
				}

				/// <summary>WndProc hook for WM_INPUT messages during raw input mode.</summary>
				IntPtr HandleRawInput(IntPtr hwnd, int msg, IntPtr wParam, IntPtr lParam, ref bool handled)
				{
					if (msg != Win32.WM_INPUT)
						return IntPtr.Zero;

					uint RawInputHeaderSize = (uint)Marshal.SizeOf<Win32.RAWINPUTHEADER>();

					// Read raw input data size
					uint size = 0;
					if (User32.GetRawInputData(lParam, Win32.RID_INPUT, IntPtr.Zero, ref size, RawInputHeaderSize) != 0)
						throw new Exception("Failed to get raw input data size.");
					if (size == 0)
						return IntPtr.Zero;

					// Reqllocate the buffer if needed
					if (size > m_buffer_size)
					{
						if (m_buffer != IntPtr.Zero)
							Marshal.FreeHGlobal(m_buffer);

						m_buffer = Marshal.AllocHGlobal((int)size);
						m_buffer_size = (int)size;
					}

					// Read a whole RAWINPUT struct into the buffer.
					if ((int)User32.GetRawInputData(lParam, Win32.RID_INPUT, m_buffer, ref size, RawInputHeaderSize) == -1)
						throw new Exception("Failed to get raw input data.");

					// Read the RAWINPUTHEADER to determine the type of input
					var input = Marshal.PtrToStructure<Win32.RAWINPUT>(m_buffer);
					if (input.header.dwType != Win32.RIM_TYPEMOUSE)
						return IntPtr.Zero;

					var button_flags = input.mouse.usButtonFlags;

					// Detect button down
					if (Bit.AllSet(button_flags, Win32.RI_MOUSE_LEFT_BUTTON_DOWN))
						MouseDown?.Invoke(Source, new MouseButtonEventArgs(Mouse.PrimaryDevice, 0, MouseButton.Left) { RoutedEvent = UIElement.MouseDownEvent });
					if (Bit.AllSet(button_flags, Win32.RI_MOUSE_RIGHT_BUTTON_DOWN))
						MouseDown?.Invoke(Source, new MouseButtonEventArgs(Mouse.PrimaryDevice, 0, MouseButton.Right) { RoutedEvent = UIElement.MouseDownEvent });
					if (Bit.AllSet(button_flags, Win32.RI_MOUSE_MIDDLE_BUTTON_DOWN))
						MouseDown?.Invoke(Source, new MouseButtonEventArgs(Mouse.PrimaryDevice, 0, MouseButton.Middle) { RoutedEvent = UIElement.MouseDownEvent });

					// Detect button releases
					if (Bit.AllSet(button_flags, Win32.RI_MOUSE_LEFT_BUTTON_UP))
						MouseUp?.Invoke(Source, new MouseButtonEventArgs(Mouse.PrimaryDevice, 0, MouseButton.Left) { RoutedEvent = UIElement.MouseUpEvent });
					if (Bit.AllSet(button_flags, Win32.RI_MOUSE_RIGHT_BUTTON_UP))
						MouseUp?.Invoke(Source, new MouseButtonEventArgs(Mouse.PrimaryDevice, 0, MouseButton.Right) { RoutedEvent = UIElement.MouseUpEvent });
					if (Bit.AllSet(button_flags, Win32.RI_MOUSE_MIDDLE_BUTTON_UP))
						MouseUp?.Invoke(Source, new MouseButtonEventArgs(Mouse.PrimaryDevice, 0, MouseButton.Middle) { RoutedEvent = UIElement.MouseUpEvent });

					// Accumulate wheel delta
					if (Bit.AllSet(button_flags, Win32.RI_MOUSE_WHEEL))
					{
						var button_data = input.mouse.usButtonData;
						MouseWheel?.Invoke(Source, new MouseWheelEventArgs(Mouse.PrimaryDevice, 0, button_data) { RoutedEvent = UIElement.MouseWheelEvent });
					}

					// Mouse movement
					var flags = input.mouse.usFlags;

					Win32.RECT rect;
					if (Bit.AllSet(flags, Win32.MOUSE_VIRTUAL_DESKTOP))
					{
						rect.left = User32.GetSystemMetrics(Win32.ESystemMetrics.SM_XVIRTUALSCREEN);
						rect.top = User32.GetSystemMetrics(Win32.ESystemMetrics.SM_YVIRTUALSCREEN);
						rect.right = User32.GetSystemMetrics(Win32.ESystemMetrics.SM_CXVIRTUALSCREEN);
						rect.bottom = User32.GetSystemMetrics(Win32.ESystemMetrics.SM_CYVIRTUALSCREEN);
					}
					else
					{
						rect.left = 0;
						rect.top = 0;
						rect.right = User32.GetSystemMetrics(Win32.ESystemMetrics.SM_CXSCREEN);
						rect.bottom = User32.GetSystemMetrics(Win32.ESystemMetrics.SM_CYSCREEN);
					}

					int x = 0, y = 0;
					if (Bit.AllSet(flags, Win32.MOUSE_MOVE_ABSOLUTE))
					{
						x = (input.mouse.lLastX * rect.right / ushort.MaxValue) + rect.left;
						y = (input.mouse.lLastY * rect.bottom / ushort.MaxValue) + rect.top;
					}
					else // Win32.MOUSE_MOVE_RELATIVE
					{
						x = input.mouse.lLastX;
						y = input.mouse.lLastY;
					}
					if (x != 0 || y != 0)
					{
						MouseMove?.Invoke(Source, new MouseEventArgs(Mouse.PrimaryDevice, 0) { RoutedEvent = UIElement.MouseMoveEvent });
					}

					handled = false; // Let WPF also process WM_INPUT
					return IntPtr.Zero;
				}
			}
		}
		private IntPtr m_buffer;
		private int m_buffer_size;

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
					var rid = new Win32.RAWINPUTDEVICE[]
					{
						new()
						{
							usUsagePage = 0x01,  // HID_USAGE_PAGE_GENERIC
							usUsage = 0x02,     // HID_USAGE_GENERIC_MOUSE
							dwFlags = Win32.RIDEV_NOLEGACY,
							hwndTarget = Source.Handle,
						}
					};
					User32.RegisterRawInputDevices(rid, 1, (uint)Marshal.SizeOf<Win32.RAWINPUTDEVICE>());
				}

				field = value;

				if (LegacyMouseMessages)
				{
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
	}
}
