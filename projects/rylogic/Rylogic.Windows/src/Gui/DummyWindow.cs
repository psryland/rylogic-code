using System;
using System.ComponentModel;
using System.Runtime.ExceptionServices;
using System.Runtime.InteropServices;
using System.Threading;
using Rylogic.Common;
using Rylogic.Interop.Win32;

namespace Rylogic.Gui.Native;

/// <summary>An owning-thread message-only window used for native notifications.</summary>
public sealed class DummyWindow : IDisposable
{
	public const string ClassName = "Rylogic-DummyWindow";
	private static readonly object s_class_lock = new();
	private static readonly Win32.WNDPROC s_wnd_proc = new(StaticWndProc);
	private static int s_class_atom;

	private readonly int m_owner_thread_id;
	private readonly object m_lifetime_lock = new();
	private GCHandle m_self_handle;
	private ExceptionDispatchInfo? m_dispatch_exception;
	private IntPtr m_hwnd;
	private bool m_running;
	private bool m_disposed;

	/// <summary>Create a message-only HWND on the current thread.</summary>
	public DummyWindow(string? diag_name = null)
	{
		m_owner_thread_id = Environment.CurrentManagedThreadId;
		var hinstance = Kernel32.GetModuleHandle(null);
		EnsureWindowClass(hinstance);
		m_self_handle = GCHandle.Alloc(this, GCHandleType.Normal);

		m_hwnd = User32.CreateWindow(
			0,
			ClassName,
			diag_name ?? string.Empty,
			0,
			0,
			0,
			1,
			1,
			Win32.HWND_MESSAGE,
			IntPtr.Zero,
			hinstance,
			GCHandle.ToIntPtr(m_self_handle));
		if (m_hwnd == IntPtr.Zero)
		{
			var error = Marshal.GetLastWin32Error();
			if (m_self_handle.IsAllocated)
				m_self_handle.Free();
			if (m_dispatch_exception != null)
				m_dispatch_exception.Throw();

			throw error != Win32.ERROR_SUCCESS
				? new Win32Exception(error, "CreateWindowExW failed for the message-only window.")
				: new InvalidOperationException("CreateWindowExW rejected message-only window creation.");
		}
	}

	/// <summary>Destroy the message-only HWND on its owning thread.</summary>
	public void Dispose()
	{
		VerifyOwnerThread();
		if (m_disposed)
			return;

		lock (m_lifetime_lock)
		{
			if (m_hwnd != IntPtr.Zero && !User32.DestroyWindow(m_hwnd))
				throw new Win32Exception("DestroyWindow failed for the message-only window.");
		}
		if (m_self_handle.IsAllocated)
			m_self_handle.Free();

		m_disposed = true;
		ThrowDispatchException();
	}

	/// <summary>The live message-only HWND.</summary>
	public IntPtr Handle => m_hwnd;

	/// <summary>Raised for each routed native message.</summary>
	public event EventHandler<WndProcEventArgs>? Message;

	/// <summary>Dispatch all currently queued messages on the owning thread.</summary>
	public bool Pump()
	{
		VerifyOwnerThread();
		for (; User32.PeekMessage(out var msg, IntPtr.Zero, 0, 0, Win32.EPeekMessageFlags.Remove);)
		{
			if (msg.message == Win32.WM_QUIT)
				return false;

			User32.TranslateMessage(ref msg);
			User32.DispatchMessage(ref msg);
			ThrowDispatchException();
		}
		return true;
	}

	/// <summary>Run a synchronous owning-thread pump until close, cancellation, or WM_QUIT.</summary>
	public int Run(CancellationToken shutdown = default)
	{
		VerifyOwnerThread();
		if (m_running)
			throw new InvalidOperationException("The message-only window pump is already running.");
		if (m_hwnd == IntPtr.Zero)
			throw new ObjectDisposedException(nameof(DummyWindow));

		m_running = true;
		using var cancel_registration = shutdown.Register(() => RequestClose());
		try
		{
			for (Win32.MESSAGE msg; ;)
			{
				var result = User32.GetMessage(out msg, IntPtr.Zero, 0, 0);
				if (result == -1)
					throw new Win32Exception("GetMessage failed for the message-only window.");
				if (result == 0)
					return (int)msg.wparam;

				User32.TranslateMessage(ref msg);
				User32.DispatchMessage(ref msg);
				ThrowDispatchException();
			}
		}
		finally
		{
			m_running = false;
		}
	}

	/// <summary>Post a close request from any thread, returning false after native destruction.</summary>
	public bool RequestClose()
	{
		lock (m_lifetime_lock)
		{
			var hwnd = m_hwnd;
			if (hwnd == IntPtr.Zero)
				return false;
			if (!User32.PostMessage(hwnd, Win32.WM_CLOSE, IntPtr.Zero, IntPtr.Zero))
				throw new Win32Exception("PostMessage failed while closing the message-only window.");
		}

		return true;
	}

	/// <summary>Raise managed hooks and provide exact custom LRESULT support.</summary>
	private IntPtr WndProc(IntPtr hwnd, int message, IntPtr wparam, IntPtr lparam)
	{
		if (Environment.CurrentManagedThreadId != m_owner_thread_id)
			throw new InvalidOperationException("The message-only WNDPROC ran outside its owning thread.");

		var args = new WndProcEventArgs(hwnd, message, wparam, lparam);
		Message?.Invoke(this, args);
		if (args.Handled && message != Win32.WM_DESTROY)
			return args.Result;

		switch (message)
		{
			case Win32.WM_CLOSE:
				{
					if (m_hwnd != hwnd)
						return IntPtr.Zero;

					lock (m_lifetime_lock)
					{
						if (m_hwnd == hwnd && !User32.DestroyWindow(hwnd))
							throw new Win32Exception("DestroyWindow failed while closing the message-only window.");
					}

					return IntPtr.Zero;
				}
			case Win32.WM_DESTROY:
				{
					if (m_running)
						User32.PostQuitMessage(0);

					return IntPtr.Zero;
				}
			default:
				{
					return User32.DefWindowProc(hwnd, message, wparam, lparam);
				}
		}
	}

	/// <summary>Release the managed root and HWND association during WM_NCDESTROY.</summary>
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

	/// <summary>Capture callback failures so no managed exception crosses the native boundary.</summary>
	private void FailDispatch(Exception exception, int message)
	{
		m_dispatch_exception ??= ExceptionDispatchInfo.Capture(exception);
		var destroyed = false;
		lock (m_lifetime_lock)
		{
			var native_controls_lifetime =
				message == Win32.WM_NCCREATE ||
				message == Win32.WM_CREATE ||
				message == Win32.WM_DESTROY ||
				message == Win32.WM_NCDESTROY;
			if (m_hwnd != IntPtr.Zero && !native_controls_lifetime)
				destroyed = User32.DestroyWindow(m_hwnd);
		}
		if (m_running && !destroyed && m_hwnd != IntPtr.Zero)
			User32.PostQuitMessage(1);
	}

	/// <summary>Rethrow a captured callback failure at most once from managed code.</summary>
	private void ThrowDispatchException()
	{
		Interlocked.Exchange(ref m_dispatch_exception, null)?.Throw();
	}

	/// <summary>Associate WM_NCCREATE with this instance and route through GWLP_USERDATA.</summary>
	private static IntPtr StaticWndProc(IntPtr hwnd, int message, IntPtr wparam, IntPtr lparam)
	{
		DummyWindow? window = null;
		try
		{
			if (message == Win32.WM_NCCREATE)
			{
				var create = Marshal.PtrToStructure<Win32.CREATESTRUCT>(lparam);
				var self_handle = GCHandle.FromIntPtr(create.lpCreateParams);
				window = self_handle.Target as DummyWindow ?? throw new InvalidOperationException("WM_NCCREATE did not receive a DummyWindow instance.");
				window.m_hwnd = hwnd;
				User32.SetWindowLongPtr(hwnd, Win32.GWLP_USERDATA, create.lpCreateParams);
			}
			else
			{
				var self = User32.GetWindowLongPtr(hwnd, Win32.GWLP_USERDATA);
				if (self != IntPtr.Zero)
					window = GCHandle.FromIntPtr(self).Target as DummyWindow;
			}

			if (window == null)
				return User32.DefWindowProc(hwnd, message, wparam, lparam);
			if (message != Win32.WM_NCDESTROY)
				return window.WndProc(hwnd, message, wparam, lparam);

			try
			{
				return window.WndProc(hwnd, message, wparam, lparam);
			}
			finally
			{
				window.ReleaseNativeAssociation();
			}
		}
		catch (Exception ex)
		{
			window?.FailDispatch(ex, message);
			return message switch
			{
				Win32.WM_NCCREATE => IntPtr.Zero,
				Win32.WM_CREATE => new IntPtr(-1),
				_ => IntPtr.Zero,
			};
		}
	}

	/// <summary>Register the message-only class once with a permanently rooted delegate.</summary>
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
				style = 0,
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

	/// <summary>Enforce native thread ownership for synchronous operations.</summary>
	private void VerifyOwnerThread()
	{
		if (Environment.CurrentManagedThreadId != m_owner_thread_id)
			throw new InvalidOperationException("This operation must run on the thread that created the message-only window.");
	}
}
