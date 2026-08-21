using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Runtime.ExceptionServices;
using System.Runtime.InteropServices;
using System.Threading;
using Rylogic.Common;
using Rylogic.Interop.Win32;

namespace Rylogic.Windows.Gui;

/// <summary>The presentation mode of a managed top-level Win32 window.</summary>
public enum EWindowMode
{
	Windowed,
	BorderlessFullscreen,
}

/// <summary>The size state reported by a top-level Win32 window.</summary>
public enum EWindowSizeState
{
	Restored,
	Minimized,
	Maximized,
}

/// <summary>The kind of standard pointer input reported by a top-level Win32 window.</summary>
public enum EPointerAction
{
	Move,
	ButtonDown,
	ButtonUp,
	Wheel,
}

/// <summary>Creation and lifecycle options for <see cref="Win32Application"/>.</summary>
public sealed class Win32ApplicationOptions
{
	/// <summary>The title shown by the desktop window manager.</summary>
	public string Title { get; set; } = string.Empty;

	/// <summary>The initial client-area width in physical pixels.</summary>
	public int ClientWidth { get; set; } = 1280;

	/// <summary>The initial client-area height in physical pixels.</summary>
	public int ClientHeight { get; set; } = 720;

	/// <summary>The initial screen-space X coordinate, or <see cref="Win32.CW_USEDEFAULT"/>.</summary>
	public int X { get; set; } = Win32.CW_USEDEFAULT;

	/// <summary>The initial screen-space Y coordinate, or <see cref="Win32.CW_USEDEFAULT"/>.</summary>
	public int Y { get; set; } = Win32.CW_USEDEFAULT;

	/// <summary>The native window style used while windowed.</summary>
	public int Style { get; set; } = Win32.WS_OVERLAPPEDWINDOW | Win32.WS_CLIPCHILDREN;

	/// <summary>The native extended window style used while windowed.</summary>
	public int StyleEx { get; set; } = Win32.WS_EX_APPWINDOW | Win32.WS_EX_WINDOWEDGE;

	/// <summary>An optional native menu handle.</summary>
	public IntPtr Menu { get; set; }

	/// <summary>The command used when the message pump first shows the window.</summary>
	public int ShowCommand { get; set; } = Win32.SW_SHOW;

	/// <summary>The initial desktop presentation mode.</summary>
	public EWindowMode WindowMode { get; set; } = EWindowMode.Windowed;

	/// <summary>The exit code used when cancellation requests closure.</summary>
	public int CancellationExitCode { get; set; }

	/// <summary>An optional message callback installed before native creation begins.</summary>
	public Action<WndProcEventArgs>? MessageHandler { get; set; }
}

/// <summary>Data reported when the client area changes size or minimize state.</summary>
public sealed class WindowSizeChangedEventArgs : EventArgs
{
	/// <summary>Create a size notification.</summary>
	public WindowSizeChangedEventArgs(int width, int height, EWindowSizeState state)
	{
		Width = width;
		Height = height;
		State = state;
	}

	/// <summary>The client width in physical pixels.</summary>
	public int Width { get; }

	/// <summary>The client height in physical pixels.</summary>
	public int Height { get; }

	/// <summary>The native restore, minimize, or maximize state.</summary>
	public EWindowSizeState State { get; }

	/// <summary>True when rendering should pause because the window is minimized.</summary>
	public bool IsMinimized => State == EWindowSizeState.Minimized;
}

/// <summary>Data reported when application activation or keyboard focus changes.</summary>
public sealed class WindowActivationChangedEventArgs : EventArgs
{
	/// <summary>Create an activation notification.</summary>
	public WindowActivationChangedEventArgs(bool active)
	{
		Active = active;
	}

	/// <summary>True when the window or application is active.</summary>
	public bool Active { get; }
}

/// <summary>Data reported when the window moves to a different desktop monitor.</summary>
public sealed class WindowMonitorChangedEventArgs : EventArgs
{
	/// <summary>Create a monitor transition notification.</summary>
	public WindowMonitorChangedEventArgs(IntPtr previous_monitor, IntPtr current_monitor)
	{
		PreviousMonitor = previous_monitor;
		CurrentMonitor = current_monitor;
	}

	/// <summary>The previous monitor handle, or zero during initial assignment.</summary>
	public IntPtr PreviousMonitor { get; }

	/// <summary>The current monitor handle.</summary>
	public IntPtr CurrentMonitor { get; }
}

/// <summary>Data reported when per-monitor DPI changes.</summary>
public sealed class WindowDpiChangedEventArgs : EventArgs
{
	/// <summary>Create a DPI transition notification.</summary>
	public WindowDpiChangedEventArgs(uint dpi_x, uint dpi_y, Win32.RECT suggested_bounds)
	{
		DpiX = dpi_x;
		DpiY = dpi_y;
		SuggestedBounds = suggested_bounds;
		ApplySuggestedBounds = true;
	}

	/// <summary>The new horizontal DPI.</summary>
	public uint DpiX { get; }

	/// <summary>The new vertical DPI.</summary>
	public uint DpiY { get; }

	/// <summary>The desktop bounds recommended by Windows for the new DPI.</summary>
	public Win32.RECT SuggestedBounds { get; }

	/// <summary>True to apply <see cref="SuggestedBounds"/> after notification.</summary>
	public bool ApplySuggestedBounds { get; set; }
}

/// <summary>Data reported when a standard keyboard message is received.</summary>
public sealed class WindowKeyChangedEventArgs : EventArgs
{
	/// <summary>Create a keyboard notification.</summary>
	public WindowKeyChangedEventArgs(EKeyCodes key, bool pressed, bool system_key, bool repeat, int repeat_count)
	{
		Key = key;
		Pressed = pressed;
		SystemKey = system_key;
		Repeat = repeat;
		RepeatCount = repeat_count;
	}

	/// <summary>The native virtual key.</summary>
	public EKeyCodes Key { get; }

	/// <summary>True for key-down and false for key-up.</summary>
	public bool Pressed { get; }

	/// <summary>True for the system-key message family used by Alt combinations.</summary>
	public bool SystemKey { get; }

	/// <summary>True when the key was already down.</summary>
	public bool Repeat { get; }

	/// <summary>The native repeat count carried by the message.</summary>
	public int RepeatCount { get; }
}

/// <summary>Data reported when translated character input is received.</summary>
public sealed class WindowTextInputEventArgs : EventArgs
{
	/// <summary>Create a text-input notification.</summary>
	public WindowTextInputEventArgs(char character, int repeat_count)
	{
		Character = character;
		RepeatCount = repeat_count;
	}

	/// <summary>The translated UTF-16 character.</summary>
	public char Character { get; }

	/// <summary>The native repeat count carried by the message.</summary>
	public int RepeatCount { get; }
}

/// <summary>Data reported for standard mouse messages in client coordinates.</summary>
public sealed class WindowPointerEventArgs : EventArgs
{
	/// <summary>Create a pointer notification.</summary>
	public WindowPointerEventArgs(EPointerAction action, int x, int y, EMouseBtns buttons, EMouseBtns changed_button, int wheel_delta)
	{
		Action = action;
		X = x;
		Y = y;
		Buttons = buttons;
		ChangedButton = changed_button;
		WheelDelta = wheel_delta;
	}

	/// <summary>The kind of pointer transition.</summary>
	public EPointerAction Action { get; }

	/// <summary>The client-space X coordinate.</summary>
	public int X { get; }

	/// <summary>The client-space Y coordinate.</summary>
	public int Y { get; }

	/// <summary>The native button and modifier state.</summary>
	public EMouseBtns Buttons { get; }

	/// <summary>The button changed by a button transition, otherwise none.</summary>
	public EMouseBtns ChangedButton { get; }

	/// <summary>The signed wheel delta for wheel input, otherwise zero.</summary>
	public int WheelDelta { get; }
}

/// <summary>Data reported before a native close request destroys the window.</summary>
public sealed class WindowClosingEventArgs : CancelEventArgs
{
	/// <summary>Create a cancellable close notification.</summary>
	public WindowClosingEventArgs(int exit_code)
	{
		ExitCode = exit_code;
	}

	/// <summary>The process-style exit code requested for the message pump.</summary>
	public int ExitCode { get; }
}

/// <summary>
/// Owns one managed top-level HWND and the message pump that runs it on the creating thread.
/// Borderless fullscreen covers the selected desktop monitor; true D3D exclusive fullscreen remains a graphics/swap-chain responsibility.
/// </summary>
public sealed class Win32Application : IDisposable
{
	private const string ClassName = "Rylogic.Windows.Win32Application";
	private const int WM_REQUEST_CLOSE = Win32.WM_APP + 0x321;
	private static readonly object s_class_lock = new();
	private static readonly Win32.WNDPROC s_wnd_proc = new(StaticWndProc);
	private static int s_class_atom;

	private readonly Win32ApplicationOptions m_options;
	private readonly int m_owner_thread_id;
	private readonly object m_lifetime_lock = new();
	private GCHandle m_self_handle;
	private ExceptionDispatchInfo? m_dispatch_exception;
	private WindowedState? m_windowed_state;
	private int? m_pending_exit_code;
	private IntPtr m_hwnd;
	private IntPtr m_monitor;
	private uint m_dpi;
	private int m_client_width;
	private int m_client_height;
	private int m_exit_code;
	private bool m_active;
	private bool m_has_focus;
	private bool m_running;
	private bool m_has_run;
	private bool m_disposed;
	private bool m_construction_complete;
	private bool m_destroying_raised;
	private bool m_destruction_in_progress;
	private EWindowMode m_window_mode;

	/// <summary>Create the native window on the current thread without showing it.</summary>
	public Win32Application(Win32ApplicationOptions? options = null)
	{
		m_options = options ?? new Win32ApplicationOptions();
		m_owner_thread_id = Environment.CurrentManagedThreadId;
		m_window_mode = EWindowMode.Windowed;

		if (m_options.ClientWidth <= 0)
			throw new ArgumentOutOfRangeException(nameof(options), m_options.ClientWidth, "ClientWidth must be positive.");
		if (m_options.ClientHeight <= 0)
			throw new ArgumentOutOfRangeException(nameof(options), m_options.ClientHeight, "ClientHeight must be positive.");

		// Calculate native outer dimensions while keeping the public size contract in client pixels.
		var outer = User32.AdjustWindowRect(
			Win32.RECT.FromLTRB(0, 0, m_options.ClientWidth, m_options.ClientHeight),
			m_options.Style,
			m_options.Menu != IntPtr.Zero,
			m_options.StyleEx);

		var hinstance = Kernel32.GetModuleHandle(null);
		EnsureWindowClass(hinstance);
		var created_hwnd = IntPtr.Zero;
		var associated = false;
		try
		{
			m_self_handle = GCHandle.Alloc(this, GCHandleType.Normal);

			// The GC handle is transferred to GWLP_USERDATA during WM_NCCREATE and released during WM_NCDESTROY.
			created_hwnd = User32.CreateWindow(
				m_options.StyleEx,
				ClassName,
				m_options.Title,
				m_options.Style,
				m_options.X,
				m_options.Y,
				outer.width,
				outer.height,
				IntPtr.Zero,
				m_options.Menu,
				hinstance,
				GCHandle.ToIntPtr(m_self_handle));

			if (created_hwnd == IntPtr.Zero)
			{
				var error = Marshal.GetLastWin32Error();
				ThrowDispatchException();
				throw error != Win32.ERROR_SUCCESS
					? new Win32Exception(error, "CreateWindowExW failed.")
					: new InvalidOperationException("CreateWindowExW rejected native window creation.");
			}

			if (m_hwnd != created_hwnd)
				throw new InvalidOperationException("The native window was not associated during WM_NCCREATE.");

			associated = true;
			m_dpi = User32.GetDpiForWindow(m_hwnd);
			if (m_dpi == 0)
				throw new Win32Exception("GetDpiForWindow failed.");

			// Correct the provisional system-DPI frame so the requested client size remains physical-pixel exact on the selected monitor.
			var dpi_outer = User32.AdjustWindowRectForDpi(
				Win32.RECT.FromLTRB(0, 0, m_options.ClientWidth, m_options.ClientHeight),
				m_options.Style,
				m_options.Menu != IntPtr.Zero,
				m_options.StyleEx,
				m_dpi);
			if (!User32.SetWindowPos(
				m_hwnd,
				IntPtr.Zero,
				0,
				0,
				dpi_outer.width,
				dpi_outer.height,
				Win32.SWP_NOMOVE | Win32.SWP_NOZORDER | Win32.SWP_NOACTIVATE))
				throw new Win32Exception("SetWindowPos failed while applying the initial DPI-aware client size.");

			var client = User32.GetClientRect(m_hwnd);
			m_client_width = client.width;
			m_client_height = client.height;
			UpdateMonitor();
			WindowMode = m_options.WindowMode;
			m_construction_complete = true;
		}
		catch (Exception creation_error)
		{
			// A constructor failure after native creation must release both the HWND and its managed GC root.
			var dispatch_error = TakeDispatchException();
			var primary_error = dispatch_error?.SourceException ?? creation_error;
			Exception? cleanup_error = null;
			var hwnd = m_hwnd != IntPtr.Zero
				? m_hwnd
				: !associated ? created_hwnd : IntPtr.Zero;
			if (hwnd != IntPtr.Zero)
			{
				try
				{
					DestroyOwnedWindow(false, hwnd);
				}
				catch (Win32Exception ex)
				{
					cleanup_error = ex;
				}
			}
			if (m_self_handle.IsAllocated && m_hwnd == IntPtr.Zero)
				m_self_handle.Free();

			if (cleanup_error != null)
				throw new AggregateException("Win32 window creation and cleanup both failed.", primary_error, cleanup_error);
			if (dispatch_error != null)
				dispatch_error.Throw();

			ExceptionDispatchInfo.Capture(creation_error).Throw();
			throw;
		}
	}

	/// <summary>Destroy the HWND deterministically on its owning thread.</summary>
	public void Dispose()
	{
		VerifyOwnerThread();
		if (m_disposed)
			return;

		var destroyed = DestroyOwnedWindow(true);
		if (!destroyed && m_hwnd != IntPtr.Zero)
			return;

		if (m_self_handle.IsAllocated)
			m_self_handle.Free();

		m_disposed = true;
		ThrowDispatchException();
	}

	/// <summary>The live top-level HWND exposed for renderer and swap-chain binding.</summary>
	public IntPtr Handle => m_hwnd;

	/// <summary>True after the native HWND has completed WM_NCDESTROY.</summary>
	public bool IsClosed => m_hwnd == IntPtr.Zero;

	/// <summary>The managed ID of the thread that owns the HWND and message pump.</summary>
	public int OwnerThreadId => m_owner_thread_id;

	/// <summary>The current client width in physical pixels.</summary>
	public int ClientWidth => m_client_width;

	/// <summary>The current client height in physical pixels.</summary>
	public int ClientHeight => m_client_height;

	/// <summary>True while the client area is minimized.</summary>
	public bool IsMinimized { get; private set; }

	/// <summary>True while Windows considers the application active.</summary>
	public bool IsActive => m_active;

	/// <summary>True while the HWND owns keyboard focus.</summary>
	public bool HasFocus => m_has_focus;

	/// <summary>The current per-monitor DPI.</summary>
	public uint Dpi => m_dpi;

	/// <summary>The monitor currently containing the largest part of the window.</summary>
	public IntPtr Monitor => m_monitor;

	/// <summary>The exit code requested by close, cancellation, or WM_QUIT.</summary>
	public int ExitCode => m_exit_code;

	/// <summary>
	/// The desktop presentation mode. Borderless fullscreen is reversible desktop composition, not D3D exclusive fullscreen.
	/// </summary>
	public EWindowMode WindowMode
	{
		get => m_window_mode;
		set
		{
			VerifyOwnerThread();
			VerifyUsable();
			if (m_window_mode == value)
				return;

			try
			{
				switch (value)
				{
					case EWindowMode.Windowed:
					{
						LeaveBorderlessFullscreen();
						break;
					}
					case EWindowMode.BorderlessFullscreen:
					{
						EnterBorderlessFullscreen();
						break;
					}
					default:
					{
						throw new ArgumentOutOfRangeException(nameof(value), value, null);
					}
				}
			}
			catch
			{
				ThrowDispatchException();
				throw;
			}

			ThrowDispatchException();
			VerifyUsable();
			m_window_mode = value;
		}
	}

	/// <summary>Raised for every message after HWND association, including WM_NCCREATE.</summary>
	public event EventHandler<WndProcEventArgs>? Message;

	/// <summary>Raised before a close request destroys the HWND.</summary>
	public event EventHandler<WindowClosingEventArgs>? Closing;

	/// <summary>
	/// Raised once on the owner thread immediately before post-construction destruction of the owned HWND.
	/// Subscriber failures cannot cancel destruction and are rethrown from the invoking managed boundary.
	/// </summary>
	public event EventHandler? Destroying;

	/// <summary>Raised after native association and managed routing have been released.</summary>
	public event EventHandler? Closed;

	/// <summary>Raised when client dimensions or minimize state change.</summary>
	public event EventHandler<WindowSizeChangedEventArgs>? SizeChanged;

	/// <summary>Raised when application activation changes.</summary>
	public event EventHandler<WindowActivationChangedEventArgs>? ActivationChanged;

	/// <summary>Raised when keyboard focus enters or leaves the HWND.</summary>
	public event EventHandler<WindowActivationChangedEventArgs>? FocusChanged;

	/// <summary>Raised after Windows assigns a new per-monitor DPI.</summary>
	public event EventHandler<WindowDpiChangedEventArgs>? DpiChanged;

	/// <summary>Raised when the HWND moves to a different monitor.</summary>
	public event EventHandler<WindowMonitorChangedEventArgs>? MonitorChanged;

	/// <summary>Raised for standard key-up and key-down messages.</summary>
	public event EventHandler<WindowKeyChangedEventArgs>? KeyChanged;

	/// <summary>Raised for translated WM_CHAR input.</summary>
	public event EventHandler<WindowTextInputEventArgs>? TextInput;

	/// <summary>Raised for standard mouse movement, buttons, and wheel messages.</summary>
	public event EventHandler<WindowPointerEventArgs>? PointerChanged;

	/// <summary>Show the HWND without starting the message pump.</summary>
	public void Show(int? show_command = null)
	{
		VerifyOwnerThread();
		VerifyUsable();
		try
		{
			User32.ShowWindow(m_hwnd, show_command ?? m_options.ShowCommand);
			User32.UpdateWindow(m_hwnd);
		}
		catch
		{
			ThrowDispatchException();
			throw;
		}

		ThrowDispatchException();
		VerifyUsable();
	}

	/// <summary>Run a message loop on the owning thread until the window closes or WM_QUIT arrives.</summary>
	public int Run(MessageLoop? message_loop = null, CancellationToken shutdown = default)
	{
		VerifyOwnerThread();
		if (m_disposed)
			throw new ObjectDisposedException(nameof(Win32Application));
		if (m_has_run)
			throw new InvalidOperationException("The application message pump can only run once.");

		m_has_run = true;
		if (m_hwnd == IntPtr.Zero)
			return m_exit_code;

		Show();
		ThrowDispatchException();
		if (m_hwnd == IntPtr.Zero)
			return m_exit_code;

		m_running = true;
		using var cancel_registration = shutdown.Register(() => RequestClose(m_options.CancellationExitCode));
		try
		{
			m_exit_code = (message_loop ?? new MessageLoop()).Run();
		}
		finally
		{
			// A foreign WM_QUIT must not leave the owned HWND alive after the pump exits.
			m_running = false;
			DestroyOwnedWindow(true);
		}

		ThrowDispatchException();
		return m_exit_code;
	}

	/// <summary>Synchronously request closure from the owning thread.</summary>
	public void Close(int exit_code = 0)
	{
		VerifyOwnerThread();
		VerifyUsable();
		SendClose(m_hwnd, exit_code);
		ThrowDispatchException();
	}

	/// <summary>Post a close request from any thread, returning false when the HWND has already closed.</summary>
	public bool RequestClose(int exit_code = 0)
	{
		lock (m_lifetime_lock)
		{
			var hwnd = m_hwnd;
			if (hwnd == IntPtr.Zero)
				return false;
			if (!User32.PostMessage(hwnd, WM_REQUEST_CLOSE, new IntPtr(exit_code), IntPtr.Zero))
				throw new Win32Exception("PostMessage failed while requesting window closure.");
		}

		return true;
	}

	/// <summary>Route one native message after managed association has been established.</summary>
	private IntPtr WndProc(IntPtr hwnd, int message, IntPtr wparam, IntPtr lparam)
	{
		if (Environment.CurrentManagedThreadId != m_owner_thread_id)
			throw new InvalidOperationException("The window procedure ran outside its owning thread.");

		// Raw handlers can implement renderer-specific policy and return an exact LRESULT.
		var previous_exit_code = m_exit_code;
		if (message == Win32.WM_CLOSE)
			m_exit_code = m_pending_exit_code ?? 0;

		var args = new WndProcEventArgs(hwnd, message, wparam, lparam);
		m_options.MessageHandler?.Invoke(args);
		Message?.Invoke(this, args);
		if (args.Handled && message != Win32.WM_DESTROY)
		{
			if (message == Win32.WM_CLOSE && m_hwnd == hwnd)
				m_exit_code = previous_exit_code;

			return args.Result;
		}

		switch (message)
		{
			case WM_REQUEST_CLOSE:
				{
					return SendClose(hwnd, wparam.ToInt32());
				}
			case Win32.WM_CLOSE:
				{
					if (m_hwnd != hwnd)
						return IntPtr.Zero;

					var exit_code = m_exit_code;
					var closing = new WindowClosingEventArgs(exit_code);
					Closing?.Invoke(this, closing);
					if (closing.Cancel && m_hwnd == hwnd)
					{
						m_exit_code = previous_exit_code;
						return IntPtr.Zero;
					}
					if (m_hwnd != hwnd)
						return IntPtr.Zero;

					DestroyOwnedWindow(true);

					return IntPtr.Zero;
				}
			case Win32.WM_DESTROY:
				{
					if (m_running)
						User32.PostQuitMessage(m_exit_code);

					return IntPtr.Zero;
				}
			case Win32.WM_SIZE:
				{
					var state = ToSizeState(wparam.ToInt32());
					m_client_width = Win32.LoWord(lparam);
					m_client_height = Win32.HiWord(lparam);
					IsMinimized = state == EWindowSizeState.Minimized;
					SizeChanged?.Invoke(this, new WindowSizeChangedEventArgs(m_client_width, m_client_height, state));
					UpdateMonitor();
					return IntPtr.Zero;
				}
			case Win32.WM_ACTIVATEAPP:
				{
					m_active = wparam != IntPtr.Zero;
					ActivationChanged?.Invoke(this, new WindowActivationChangedEventArgs(m_active));
					return IntPtr.Zero;
				}
			case Win32.WM_SETFOCUS:
				{
					m_has_focus = true;
					FocusChanged?.Invoke(this, new WindowActivationChangedEventArgs(true));
					return IntPtr.Zero;
				}
			case Win32.WM_KILLFOCUS:
				{
					m_has_focus = false;
					FocusChanged?.Invoke(this, new WindowActivationChangedEventArgs(false));
					return IntPtr.Zero;
				}
			case Win32.WM_DPICHANGED:
				{
					var suggested_bounds = Marshal.PtrToStructure<Win32.RECT>(lparam);
					var changed = new WindowDpiChangedEventArgs((uint)Win32.LoWord(wparam), (uint)Win32.HiWord(wparam), suggested_bounds);
					m_dpi = changed.DpiX;
					DpiChanged?.Invoke(this, changed);
					if (changed.ApplySuggestedBounds)
					{
						SetWindowBounds(suggested_bounds, Win32.SWP_NOZORDER | Win32.SWP_NOACTIVATE);
					}
					UpdateMonitor();
					return IntPtr.Zero;
				}
			case Win32.WM_MOVE:
			case Win32.WM_WINDOWPOSCHANGED:
				{
					UpdateMonitor();
					return User32.DefWindowProc(hwnd, message, wparam, lparam);
				}
			case Win32.WM_DISPLAYCHANGE:
				{
					if (m_window_mode == EWindowMode.BorderlessFullscreen)
						FitBorderlessToMonitor();

					UpdateMonitor();
					return User32.DefWindowProc(hwnd, message, wparam, lparam);
				}
			case Win32.WM_KEYDOWN:
			case Win32.WM_KEYUP:
			case Win32.WM_SYSKEYDOWN:
			case Win32.WM_SYSKEYUP:
				{
					var pressed = message == Win32.WM_KEYDOWN || message == Win32.WM_SYSKEYDOWN;
					var system_key = message == Win32.WM_SYSKEYDOWN || message == Win32.WM_SYSKEYUP;
					var repeat = (lparam.ToInt64() & (1L << 30)) != 0;
					KeyChanged?.Invoke(this, new WindowKeyChangedEventArgs((EKeyCodes)wparam.ToInt32(), pressed, system_key, repeat, Win32.LoWord(lparam)));
					return system_key
						? User32.DefWindowProc(hwnd, message, wparam, lparam)
						: IntPtr.Zero;
				}
			case Win32.WM_CHAR:
				{
					TextInput?.Invoke(this, new WindowTextInputEventArgs((char)wparam.ToInt32(), Win32.LoWord(lparam)));
					return IntPtr.Zero;
				}
			case Win32.WM_MOUSEMOVE:
			case Win32.WM_LBUTTONDOWN:
			case Win32.WM_LBUTTONDBLCLK:
			case Win32.WM_LBUTTONUP:
			case Win32.WM_RBUTTONDOWN:
			case Win32.WM_RBUTTONDBLCLK:
			case Win32.WM_RBUTTONUP:
			case Win32.WM_MBUTTONDOWN:
			case Win32.WM_MBUTTONDBLCLK:
			case Win32.WM_MBUTTONUP:
			case Win32.WM_XBUTTONDOWN:
			case Win32.WM_XBUTTONDBLCLK:
			case Win32.WM_XBUTTONUP:
			case Win32.WM_MOUSEWHEEL:
				{
					RaisePointerChanged(message, wparam, lparam);
					return IntPtr.Zero;
				}
			default:
				{
					return User32.DefWindowProc(hwnd, message, wparam, lparam);
				}
		}
	}

	/// <summary>Convert a native WM_SIZE state into the managed contract.</summary>
	private static EWindowSizeState ToSizeState(int state)
	{
		return state switch
		{
			Win32.SIZE_RESTORED => EWindowSizeState.Restored,
			Win32.SIZE_MINIMIZED => EWindowSizeState.Minimized,
			Win32.SIZE_MAXIMIZED => EWindowSizeState.Maximized,
			Win32.SIZE_MAXSHOW => EWindowSizeState.Restored,
			Win32.SIZE_MAXHIDE => EWindowSizeState.Restored,
			_ => throw new ArgumentOutOfRangeException(nameof(state), state, null),
		};
	}

	/// <summary>Decode a standard mouse message into client-space renderer input.</summary>
	private void RaisePointerChanged(int message, IntPtr wparam, IntPtr lparam)
	{
		var action = EPointerAction.Move;
		var changed_button = EMouseBtns.None;
		var wheel_delta = 0;
		var point = new Win32.POINT
		{
			X = unchecked((short)Win32.LoWord(lparam)),
			Y = unchecked((short)Win32.HiWord(lparam)),
		};

		switch (message)
		{
			case Win32.WM_MOUSEMOVE:
				{
					action = EPointerAction.Move;
					break;
				}
			case Win32.WM_LBUTTONDOWN:
			case Win32.WM_LBUTTONDBLCLK:
				{
					action = EPointerAction.ButtonDown;
					changed_button = EMouseBtns.Left;
					break;
				}
			case Win32.WM_LBUTTONUP:
				{
					action = EPointerAction.ButtonUp;
					changed_button = EMouseBtns.Left;
					break;
				}
			case Win32.WM_RBUTTONDOWN:
			case Win32.WM_RBUTTONDBLCLK:
				{
					action = EPointerAction.ButtonDown;
					changed_button = EMouseBtns.Right;
					break;
				}
			case Win32.WM_RBUTTONUP:
				{
					action = EPointerAction.ButtonUp;
					changed_button = EMouseBtns.Right;
					break;
				}
			case Win32.WM_MBUTTONDOWN:
			case Win32.WM_MBUTTONDBLCLK:
				{
					action = EPointerAction.ButtonDown;
					changed_button = EMouseBtns.Middle;
					break;
				}
			case Win32.WM_MBUTTONUP:
				{
					action = EPointerAction.ButtonUp;
					changed_button = EMouseBtns.Middle;
					break;
				}
			case Win32.WM_XBUTTONDOWN:
			case Win32.WM_XBUTTONDBLCLK:
			case Win32.WM_XBUTTONUP:
				{
					action = message != Win32.WM_XBUTTONUP ? EPointerAction.ButtonDown : EPointerAction.ButtonUp;
					changed_button = Win32.HiWord(wparam) switch
					{
						1 => EMouseBtns.XButton1,
						2 => EMouseBtns.XButton2,
						var value => throw new ArgumentOutOfRangeException(nameof(wparam), value, "Unknown XButton identifier."),
					};
					break;
				}
			case Win32.WM_MOUSEWHEEL:
				{
					action = EPointerAction.Wheel;
					wheel_delta = unchecked((short)Win32.HiWord(wparam));
					if (!User32.ScreenToClient(m_hwnd, ref point))
						throw new Win32Exception("ScreenToClient failed for mouse-wheel input.");

					break;
				}
			default:
				{
					throw new ArgumentOutOfRangeException(nameof(message), message, null);
				}
		}

		PointerChanged?.Invoke(this, new WindowPointerEventArgs(action, point.X, point.Y, Win32.ToMouseKey(wparam), changed_button, wheel_delta));
	}

	/// <summary>Capture native state and cover the current monitor without changing the display mode.</summary>
	private void EnterBorderlessFullscreen()
	{
		if (m_windowed_state != null)
			throw new InvalidOperationException("Windowed restore state is already captured.");

		var style = User32.GetWindowLongPtr(m_hwnd, Win32.GWL_STYLE);
		var style_ex = User32.GetWindowLongPtr(m_hwnd, Win32.GWL_EXSTYLE);
		var placement = User32.GetWindowPlacement(m_hwnd);
		m_windowed_state = new WindowedState(style, style_ex, placement);

		// Remove only desktop chrome while retaining unrelated behavior flags chosen by the caller.
		var borderless_style = (style.ToInt32() & ~Win32.WS_OVERLAPPEDWINDOW) | Win32.WS_POPUP;
		var borderless_style_ex = style_ex.ToInt32() & ~(Win32.WS_EX_WINDOWEDGE | Win32.WS_EX_CLIENTEDGE | Win32.WS_EX_STATICEDGE);
		User32.SetWindowLongPtr(m_hwnd, Win32.GWL_STYLE, new IntPtr(borderless_style));
		User32.SetWindowLongPtr(m_hwnd, Win32.GWL_EXSTYLE, new IntPtr(borderless_style_ex));

		if (User32.IsIconic(m_hwnd))
			User32.ShowWindow(m_hwnd, Win32.SW_RESTORE);

		FitBorderlessToMonitor(Win32.SWP_FRAMECHANGED);
	}

	/// <summary>Restore the exact styles, placement, and show state captured before borderless mode.</summary>
	private void LeaveBorderlessFullscreen()
	{
		var state = m_windowed_state ?? throw new InvalidOperationException("No windowed restore state has been captured.");
		User32.SetWindowPlacement(m_hwnd, state.Placement);
		User32.SetWindowLongPtr(m_hwnd, Win32.GWL_STYLE, state.Style);
		User32.SetWindowLongPtr(m_hwnd, Win32.GWL_EXSTYLE, state.StyleEx);

		if (!User32.SetWindowPos(
			m_hwnd,
			IntPtr.Zero,
			0,
			0,
			0,
			0,
			Win32.SWP_NOMOVE | Win32.SWP_NOSIZE | Win32.SWP_NOZORDER | Win32.SWP_NOOWNERZORDER | Win32.SWP_FRAMECHANGED))
			throw new Win32Exception("SetWindowPos failed while restoring window chrome.");

		m_windowed_state = null;
		UpdateMonitor();
	}

	/// <summary>Apply screen-space bounds and surface native positioning failures.</summary>
	private void SetWindowBounds(Win32.RECT bounds, int flags)
	{
		if (!User32.SetWindowPos(m_hwnd, IntPtr.Zero, bounds.left, bounds.top, bounds.width, bounds.height, (uint)flags))
			throw new Win32Exception("SetWindowPos failed.");
	}

	/// <summary>Cover the nearest monitor while preserving desktop-composited borderless semantics.</summary>
	private void FitBorderlessToMonitor(int flags = 0)
	{
		var monitor = User32.MonitorFromWindow(m_hwnd, Win32.EMonitorFromFlags.DEFAULT_TO_NEAREST);
		var bounds = User32.GetMonitorInfo(monitor).rcMonitor;
		SetWindowBounds(bounds, flags | Win32.SWP_NOOWNERZORDER | Win32.SWP_NOACTIVATE);
	}

	/// <summary>Raise a monitor transition only when the native monitor identity changes.</summary>
	private void UpdateMonitor()
	{
		if (m_hwnd == IntPtr.Zero)
			return;

		var monitor = User32.MonitorFromWindow(m_hwnd, Win32.EMonitorFromFlags.DEFAULT_TO_NEAREST);
		if (monitor == m_monitor)
			return;

		var previous = m_monitor;
		m_monitor = monitor;
		MonitorChanged?.Invoke(this, new WindowMonitorChangedEventArgs(previous, monitor));
	}

	/// <summary>Release the native-to-managed association exactly once during WM_NCDESTROY.</summary>
	private void ReleaseNativeAssociation()
	{
		lock (m_lifetime_lock)
		{
			var hwnd = m_hwnd;
			try
			{
				if (hwnd != IntPtr.Zero)
					User32.SetWindowLongPtr(hwnd, Win32.GWLP_USERDATA, IntPtr.Zero);
			}
			finally
			{
				m_hwnd = IntPtr.Zero;
				if (m_self_handle.IsAllocated)
					m_self_handle.Free();
			}
		}
	}

	/// <summary>Capture a managed dispatch failure so it can be rethrown outside the unmanaged callback.</summary>
	private void FailDispatch(Exception exception, int message)
	{
		CaptureDispatchFailure(exception);

		// Stop native dispatch without allowing an exception to cross the unmanaged WNDPROC boundary.
		var destroyed = false;
		var native_controls_lifetime =
			message == Win32.WM_NCCREATE ||
			message == Win32.WM_CREATE ||
			message == Win32.WM_DESTROY ||
			message == Win32.WM_NCDESTROY;
		if (!native_controls_lifetime)
		{
			try
			{
				destroyed = DestroyOwnedWindow(true);
			}
			catch (Exception destroy_error)
			{
				CaptureDispatchFailure(destroy_error);
			}
		}
		if (m_running && !destroyed && m_hwnd != IntPtr.Zero)
			User32.PostQuitMessage(m_exit_code);
	}

	/// <summary>Send WM_CLOSE with an exit code that is committed only when closure is accepted.</summary>
	private IntPtr SendClose(IntPtr hwnd, int exit_code)
	{
		var previous_exit_code = m_pending_exit_code;
		m_pending_exit_code = exit_code;
		try
		{
			return User32.SendMessage(hwnd, Win32.WM_CLOSE, IntPtr.Zero, IntPtr.Zero);
		}
		finally
		{
			m_pending_exit_code = previous_exit_code;
		}
	}

	/// <summary>Rethrow a captured callback failure at most once from managed code.</summary>
	private void ThrowDispatchException()
	{
		TakeDispatchException()?.Throw();
	}

	/// <summary>Record a callback failure for rethrow from the next managed lifecycle boundary.</summary>
	private void CaptureDispatchFailure(Exception exception)
	{
		lock (m_lifetime_lock)
		{
			m_dispatch_exception ??= ExceptionDispatchInfo.Capture(exception);
			m_exit_code = 1;
		}
	}

	/// <summary>Raise every pre-destruction subscriber without allowing one failure to cancel later subscribers or native destruction.</summary>
	private void RaiseDestroying()
	{
		var handlers = Destroying;
		if (handlers == null)
			return;

		// Every owner gets its cleanup boundary even when an earlier subscriber fails.
		foreach (EventHandler handler in handlers.GetInvocationList())
		{
			try
			{
				handler(this, EventArgs.Empty);
			}
			catch (Exception ex)
			{
				CaptureDispatchFailure(ex);
			}
		}
	}

	/// <summary>Destroy the owned HWND once while raising the pre-destruction notification on all post-construction teardown paths.</summary>
	private bool DestroyOwnedWindow(bool raise_destroying, IntPtr unassociated_hwnd = default)
	{
		VerifyOwnerThread();

		var hwnd = unassociated_hwnd;
		var invoke_destroying = false;
		lock (m_lifetime_lock)
		{
			if (m_destruction_in_progress)
				return false;
			if (m_hwnd != IntPtr.Zero)
				hwnd = m_hwnd;
			if (hwnd == IntPtr.Zero)
				return false;

			m_destruction_in_progress = true;
			if (raise_destroying && m_construction_complete && !m_destroying_raised)
			{
				m_destroying_raised = true;
				invoke_destroying = true;
			}
		}

		try
		{
			if (invoke_destroying)
				RaiseDestroying();

			if (!User32.DestroyWindow(hwnd))
				throw new Win32Exception(Marshal.GetLastWin32Error(), "DestroyWindow failed.");

			return true;
		}
		finally
		{
			lock (m_lifetime_lock)
			{
				m_destruction_in_progress = false;
			}
		}
	}

	/// <summary>Consume a captured callback failure so managed entry points rethrow it only once.</summary>
	private ExceptionDispatchInfo? TakeDispatchException()
	{
		return Interlocked.Exchange(ref m_dispatch_exception, null);
	}

	/// <summary>Associate WM_NCCREATE with its managed instance and route all later messages through GWLP_USERDATA.</summary>
	private static IntPtr StaticWndProc(IntPtr hwnd, int message, IntPtr wparam, IntPtr lparam)
	{
		Win32Application? application = null;
		try
		{
			if (message == Win32.WM_NCCREATE)
			{
				var create = Marshal.PtrToStructure<Win32.CREATESTRUCT>(lparam);
				var self_handle = GCHandle.FromIntPtr(create.lpCreateParams);
				application = self_handle.Target as Win32Application ?? throw new InvalidOperationException("WM_NCCREATE did not receive a Win32Application instance.");
				application.m_hwnd = hwnd;
				User32.SetWindowLongPtr(hwnd, Win32.GWLP_USERDATA, create.lpCreateParams);
			}
			else
			{
				var self = User32.GetWindowLongPtr(hwnd, Win32.GWLP_USERDATA);
				if (self != IntPtr.Zero)
					application = GCHandle.FromIntPtr(self).Target as Win32Application;
			}

			if (application == null)
				return User32.DefWindowProc(hwnd, message, wparam, lparam);

			if (message != Win32.WM_NCDESTROY)
				return application.WndProc(hwnd, message, wparam, lparam);

			// WM_NCDESTROY is the single native ownership boundary for handle and GC-root cleanup.
			try
			{
				return application.WndProc(hwnd, message, wparam, lparam);
			}
			finally
			{
				application.ReleaseNativeAssociation();
				application.Closed?.Invoke(application, EventArgs.Empty);
			}
		}
		catch (Exception ex)
		{
			application?.FailDispatch(ex, message);
			return message switch
			{
				Win32.WM_NCCREATE => IntPtr.Zero,
				Win32.WM_CREATE => new IntPtr(-1),
				_ => IntPtr.Zero,
			};
		}
	}

	/// <summary>Register the process-local class once with a permanently rooted delegate.</summary>
	private static int EnsureWindowClass(IntPtr hinstance)
	{
		lock (s_class_lock)
		{
			if (s_class_atom != 0)
				return s_class_atom;

			if (User32.GetClassInfo(hinstance, ClassName, out var atom) != null)
			{
				s_class_atom = atom;
				return s_class_atom;
			}

			var window_class = new Win32.WNDCLASSEX
			{
				cbSize = Marshal.SizeOf<Win32.WNDCLASSEX>(),
				style = Win32.CS_HREDRAW | Win32.CS_VREDRAW | Win32.CS_DBLCLKS,
				lpfnWndProc = s_wnd_proc,
				cbClsExtra = 0,
				cbWndExtra = 0,
				hInstance = hinstance,
				hIcon = IntPtr.Zero,
				hCursor = IntPtr.Zero,
				hbrBackground = IntPtr.Zero,
				lpszMenuName = null,
				lpszClassName = ClassName,
				hIconSm = IntPtr.Zero,
			};
			s_class_atom = User32.RegisterClass(window_class);
			return s_class_atom;
		}
	}

	/// <summary>Reject owner-thread operations after native or managed disposal.</summary>
	private void VerifyUsable()
	{
		if (m_disposed)
			throw new ObjectDisposedException(nameof(Win32Application));
		if (m_hwnd == IntPtr.Zero)
			throw new InvalidOperationException("The native window has closed.");
	}

	/// <summary>Enforce Win32 ownership rules for synchronous operations.</summary>
	private void VerifyOwnerThread()
	{
		if (Environment.CurrentManagedThreadId != m_owner_thread_id)
			throw new InvalidOperationException("This operation must run on the thread that created the Win32 application.");
	}

	/// <summary>The native state required to reverse a borderless transition exactly.</summary>
	private sealed class WindowedState
	{
		/// <summary>Capture native desktop state before entering borderless mode.</summary>
		public WindowedState(IntPtr style, IntPtr style_ex, Win32.WINDOWPLACEMENT placement)
		{
			Style = style;
			StyleEx = style_ex;
			Placement = placement;
		}

		/// <summary>The original window style.</summary>
		public IntPtr Style { get; }

		/// <summary>The original extended window style.</summary>
		public IntPtr StyleEx { get; }

		/// <summary>The original normal bounds and show state.</summary>
		public Win32.WINDOWPLACEMENT Placement { get; }
	}
}
