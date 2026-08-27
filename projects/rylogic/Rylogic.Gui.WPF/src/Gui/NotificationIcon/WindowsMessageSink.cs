using System;
using System.ComponentModel;
using System.Windows.Input;
using Rylogic.Interop.Win32;

namespace Rylogic.Gui.WPF.NotifyIcon
{
	public class WindowMessageSink :IDisposable
	{
		// Notes:
		//  - Receives messages from the task bar icon through window messages of an underlying helper window.

		private WindowMessageSink()
		{
			Version = Win32.ENotifyIconVersion.Win95;
			WindowId = "WPFTaskbarIcon_" + Guid.NewGuid();
			HWnd = IntPtr.Zero;
		}
		public WindowMessageSink(Win32.ENotifyIconVersion version = Win32.ENotifyIconVersion.Win95)
			:this()
		{
			Version = version;

			// Get the message used to indicate the task bar has been restarted
			// This is used to re-add icons when the task bar restarts
			TaskbarRestartMessageID = User32.RegisterWindowMessage("TaskbarCreated");

			// Create a hidden dummy window
			var wc = new Win32.WNDCLASS
			{
				style = 0,
				lpfnWndProc = m_message_handler = OnWindowMessageReceived,
				cbClsExtra = 0,
				cbWndExtra = 0,
				hInstance = IntPtr.Zero,
				hIcon = IntPtr.Zero,
				hCursor = IntPtr.Zero,
				hbrBackground = IntPtr.Zero,
				lpszMenuName = string.Empty,
				lpszClassName = WindowId,
			};
			IntPtr OnWindowMessageReceived(IntPtr hWnd, int message_id, IntPtr wParam, IntPtr lParam)
			{
				// Recreate the icon if the task bar was restarted (e.g. due to Win Explorer shutdown)
				if (message_id == TaskbarRestartMessageID)
					TaskbarCreated?.Invoke(this, EventArgs.Empty);

				// Forward the message
				ProcessWindowMessage(message_id, wParam, lParam);
				return User32.DefWindowProc(hWnd, message_id, wParam, lParam);
			}
			User32.RegisterClass(wc);

			// Create the message window
			HWnd = User32.CreateWindow(0, WindowId, "", 0, 0, 0, 1, 1, IntPtr.Zero, IntPtr.Zero, IntPtr.Zero, IntPtr.Zero);
			if (HWnd == IntPtr.Zero)
				throw new Win32Exception("Failed to create window for notification icon");
		}
		~WindowMessageSink()
		{
			Dispose(false);
		}
		public void Dispose()
		{
			Dispose(true);
			GC.SuppressFinalize(this);
		}
		protected virtual void Dispose(bool disposing)
		{
			if (IsDisposed) return;
			IsDisposed = true;

			// Always destroy the unmanaged handle (even if called from the GC)
			User32.DestroyWindow(HWnd);
			m_message_handler = null!;
		}
		public bool IsDisposed { get; private set; }

		/// <summary>The version of the underlying icon. Defines how incoming messages are interpreted.</summary>
		public Win32.ENotifyIconVersion Version { get; set; }

		/// <summary>Raised if the task bar was created or restarted. Requires the task bar icon to be reset.</summary>
		public event EventHandler? TaskbarCreated;

		/// <summary>Raised when the mouse interacts with the notification icon</summary>
		public event MouseEventHandler? MouseMove;
		public event MouseButtonEventHandler? MouseButton;
		public event MouseWheelEventHandler? MouseWheel;

		/// <summary>The custom tool tip should be closed or hidden.</summary>
		public event Action<bool>? ChangeToolTipStateRequest;

		/// <summary>Raised if a balloon ToolTip was either displayed or closed (indicated by the boolean flag).</summary>
		public event Action<bool>? BalloonToolTipChanged;

		/// <summary>Handle windows messages for the notification icon</summary>
		private void ProcessWindowMessage(int msg, IntPtr wParam, IntPtr lParam)
		{
			if (msg != CallbackMessageId)
				return;

			var message = lParam.ToInt32();
			switch (message)
			{
				case Win32.WM_CONTEXTMENU:
				{
					// TODO: Handle WM_CONTEXTMENU, see https://docs.microsoft.com/en-us/windows/win32/api/shellapi/nf-shellapi-shell_notifyiconw
					//Debug.WriteLine("Unhandled WM_CONTEXTMENU");
					break;
				}
				case Win32.NIN_BALLOONSHOW:
				{
					BalloonToolTipChanged?.Invoke(true);
					break;
				}
				case Win32.NIN_BALLOONHIDE:
				case Win32.NIN_BALLOONTIMEOUT:
				{
					BalloonToolTipChanged?.Invoke(false);
					break;
				}
				case Win32.NIN_BALLOONUSERCLICK:
				{
					//MouseEventReceived?.Invoke(MouseEvent.BalloonToolTipClicked);
					break;
				}
				case Win32.NIN_POPUPOPEN:
				{
					ChangeToolTipStateRequest?.Invoke(true);
					break;
				}
				case Win32.NIN_POPUPCLOSE:
				{
					ChangeToolTipStateRequest?.Invoke(false);
					break;
				}
				case Win32.NIN_SELECT:
				{
					// TODO: Handle NIN_SELECT see https://docs.microsoft.com/en-us/windows/win32/api/shellapi/nf-shellapi-shell_notifyiconw
					//Debug.WriteLine("Unhandled NIN_SELECT");
					break;
				}
				case Win32.NIN_KEYSELECT:
				{
					// TODO: Handle NIN_KEYSELECT see https://docs.microsoft.com/en-us/windows/win32/api/shellapi/nf-shellapi-shell_notifyiconw
					//Debug.WriteLine("Unhandled NIN_KEYSELECT");
					break;
				}
				case Win32.WM_MOUSEMOVE:
				{
					MouseMove?.Invoke(this, new MouseEventArgs(Mouse.PrimaryDevice, User32.GetMessageTime()));
					break;
				}
				case Win32.WM_LBUTTONDOWN:
				case Win32.WM_LBUTTONDBLCLK:
				{
					RaiseMouseButton(System.Windows.Input.MouseButton.Left, Mouse.MouseDownEvent);
					break;
				}
				case Win32.WM_LBUTTONUP:
				{
					RaiseMouseButton(System.Windows.Input.MouseButton.Left, Mouse.MouseUpEvent);
					break;
				}
				case Win32.WM_RBUTTONDOWN:
				case Win32.WM_RBUTTONDBLCLK:
				{
					RaiseMouseButton(System.Windows.Input.MouseButton.Right, Mouse.MouseDownEvent);
					break;
				}
				case Win32.WM_RBUTTONUP:
				{
					RaiseMouseButton(System.Windows.Input.MouseButton.Right, Mouse.MouseUpEvent);
					break;
				}
				case Win32.WM_MBUTTONDOWN:
				case Win32.WM_MBUTTONDBLCLK:
				{
					RaiseMouseButton(System.Windows.Input.MouseButton.Middle, Mouse.MouseDownEvent);
					break;
				}
				case Win32.WM_MBUTTONUP:
				{
					RaiseMouseButton(System.Windows.Input.MouseButton.Middle, Mouse.MouseUpEvent);
					break;
				}
				case Win32.WM_MOUSEWHEEL:
				{
					MouseWheel?.Invoke(this, new MouseWheelEventArgs(Mouse.PrimaryDevice, User32.GetMessageTime(), new Win32.WheelState(wParam).Delta));
					break;
				}
				default:
				{
					//Debug.WriteLine("Unhandled NotifyIcon message ID: " + lParam);
					break;
				}
			}

			// Preserve the Win32 transition explicitly because WPF does not track button state for the hidden message window.
			void RaiseMouseButton(System.Windows.Input.MouseButton button, System.Windows.RoutedEvent routed_event)
			{
				MouseButton?.Invoke(this, new MouseButtonEventArgs(Mouse.PrimaryDevice, User32.GetMessageTime(), button)
				{
					RoutedEvent = routed_event,
				});
			}
		}

		/// <summary>The ID of messages that are received from the task bar icon.</summary>
		public const int CallbackMessageId = 0x400;

		/// <summary>Window class ID.</summary>
		private string WindowId { get; }

		/// <summary>The ID of the message that is being received if the task bar is (re)started.</summary>
		private uint TaskbarRestartMessageID { get; }

		/// <summary>Window handle for the message window.</summary> 
		public IntPtr HWnd { get; }

		/// <summary>Reference to the wndproc so the GC doesn't eat it.</summary>
		private Win32.WNDPROC? m_message_handler;

		/// <summary>Creates a dummy instance. Used at design time.</summary>
		public static WindowMessageSink CreateEmpty() => new();
	}
}
