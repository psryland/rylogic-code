#if PR_UNITTESTS
using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using System.Threading;
using Rylogic.Common;
using Rylogic.Interop.Win32;
using Rylogic.Windows.Gui;

namespace Rylogic.UnitTests;

/// <summary>Focused native lifecycle tests for the WPF-free managed application host.</summary>
[TestFixture]
public class TestWin32Application
{
	/// <summary>Prove creation-time routing, deterministic close, and exit-code propagation.</summary>
	[Test]
	public void LifecycleRoutesFromCreationAndReturnsExitCode()
	{
		var messages = new List<int>();
		var options = new Win32ApplicationOptions
		{
			Title = "Rylogic Win32 lifecycle test",
			MessageHandler = args => messages.Add(args.Message),
		};
		using var application = new Win32Application(options);

		Assert.Equal(true, messages.Contains(Win32.WM_NCCREATE));
		Assert.Equal(true, messages.Contains(Win32.WM_CREATE));
		Assert.Equal(false, application.IsClosed);

		Assert.Equal(true, application.RequestClose(37));
		Assert.Equal(37, application.Run());
		Assert.Equal(true, application.IsClosed);
		Assert.Equal(false, application.RequestClose());
	}

	/// <summary>Prove renderer setup can query the requested client dimensions before the window is shown.</summary>
	[Test]
	public void InitialClientSizeIsAvailableBeforeShow()
	{
		using var application = new Win32Application(new Win32ApplicationOptions
		{
			Title = "Rylogic Win32 initial size test",
			ClientWidth = 1234,
			ClientHeight = 567,
		});

		Assert.Equal(1234, application.ClientWidth);
		Assert.Equal(567, application.ClientHeight);
	}

	/// <summary>Prove a cancelled close request does not affect a later accepted exit code.</summary>
	[Test]
	public void CancelledCloseDoesNotCommitExitCode()
	{
		using var application = new Win32Application(new Win32ApplicationOptions { Title = "Rylogic Win32 cancelled close test" });
		var destroying_count = 0;
		application.Closing += (_, args) => args.Cancel = args.ExitCode == 37;
		application.Destroying += (_, _) => ++destroying_count;

		application.Close(37);
		Assert.Equal(false, application.IsClosed);
		Assert.Equal(0, application.ExitCode);
		Assert.Equal(0, destroying_count);

		Assert.Equal(true, application.RequestClose());
		Assert.Equal(0, application.Run());
	}

	/// <summary>Prove accepted close raises the one-shot destruction boundary before native ownership is released.</summary>
	[Test]
	public void CloseRaisesDestroyingBeforeClosed()
	{
		var events = new List<string>();
		using var application = new Win32Application(new Win32ApplicationOptions { Title = "Rylogic Win32 destroying close test" });
		TrackDestruction(application, events);

		application.Close(37);
		application.Dispose();

		AssertDestroyedOnce(application, events);
		Assert.Equal(37, application.ExitCode);
	}

	/// <summary>Prove cancellation closes through the same one-shot destruction boundary.</summary>
	[Test, TestFlags(EUnitTestFlags.Extended)]
	public void CancellationRaisesDestroyingBeforeClosed()
	{
		var events = new List<string>();
		using var shutdown = new CancellationTokenSource();
		using var application = new Win32Application(new Win32ApplicationOptions
		{
			Title = "Rylogic Win32 destroying cancellation test",
			CancellationExitCode = 23,
		});
		TrackDestruction(application, events);
		shutdown.Cancel();

		Assert.Equal(23, application.Run(shutdown: shutdown.Token));
		application.Dispose();

		AssertDestroyedOnce(application, events);
	}

	/// <summary>Prove direct disposal raises the destruction boundary once and remains idempotent.</summary>
	[Test]
	public void DisposeRaisesDestroyingBeforeClosed()
	{
		var events = new List<string>();
		var application = new Win32Application(new Win32ApplicationOptions { Title = "Rylogic Win32 destroying dispose test" });
		TrackDestruction(application, events);

		application.Dispose();
		application.Dispose();

		AssertDestroyedOnce(application, events);
	}

	/// <summary>Prove a WNDPROC callback failure still raises the destruction boundary and surfaces from Close.</summary>
	[Test]
	public void CallbackFailureRaisesDestroyingBeforeClosed()
	{
		var events = new List<string>();
		using var application = new Win32Application(new Win32ApplicationOptions { Title = "Rylogic Win32 destroying callback test" });
		TrackDestruction(application, events);
		application.Message += (_, args) =>
		{
			if (args.Message == Win32.WM_CLOSE)
				throw new InvalidOperationException("Expected close callback failure.");
		};

		Assert.Throws<InvalidOperationException>(() => application.Close());
		application.Dispose();

		AssertDestroyedOnce(application, events);
	}

	/// <summary>Prove destruction subscribers cannot cancel native teardown or prevent later subscribers from running.</summary>
	[Test]
	public void DestroyingFailureCannotCancelDestruction()
	{
		var events = new List<string>();
		var application = new Win32Application(new Win32ApplicationOptions { Title = "Rylogic Win32 destroying failure test" });
		application.Destroying += (_, _) => throw new InvalidOperationException("Expected destroying callback failure.");
		TrackDestruction(application, events);

		Assert.Throws<InvalidOperationException>(() => application.Dispose());
		application.Dispose();

		AssertDestroyedOnce(application, events);
	}

	/// <summary>Prove a show-time callback failure closes the HWND and returns through managed exception flow.</summary>
	[Test, TestFlags(EUnitTestFlags.Extended)]
	public void ShowCallbackFailureDoesNotEnterThePump()
	{
		using var application = new Win32Application(new Win32ApplicationOptions
		{
			Title = "Rylogic Win32 show failure test",
			MessageHandler = args =>
			{
				if (args.Message == Win32.WM_SHOWWINDOW)
					throw new InvalidOperationException("Expected show failure.");
			},
		});

		Assert.Throws<InvalidOperationException>(() => application.Run());
		Assert.Equal(true, application.IsClosed);
	}

	/// <summary>Prove direct show calls surface callback failures immediately rather than deferring them to disposal.</summary>
	[Test, TestFlags(EUnitTestFlags.Extended)]
	public void ShowSurfacesCallbackFailure()
	{
		using var application = new Win32Application(new Win32ApplicationOptions
		{
			Title = "Rylogic Win32 direct show failure test",
			MessageHandler = args =>
			{
				if (args.Message == Win32.WM_SHOWWINDOW)
					throw new InvalidOperationException("Expected direct show failure.");
			},
		});

		Assert.Throws<InvalidOperationException>(() => application.Show());
		Assert.Equal(true, application.IsClosed);
	}

	/// <summary>Prove mandatory teardown still terminates the pump when a raw handler claims WM_DESTROY.</summary>
	[Test]
	public void HandledDestroyStillTerminatesThePump()
	{
		using var application = new Win32Application(new Win32ApplicationOptions
		{
			Title = "Rylogic Win32 handled destroy test",
			MessageHandler = args =>
			{
				if (args.Message == Win32.WM_DESTROY)
					args.Handled = true;
			},
		});

		Assert.Equal(true, application.RequestClose(37));
		Assert.Equal(37, application.Run());
	}

	/// <summary>Prove reentrant destruction from the close callback preserves the accepted exit code.</summary>
	[Test]
	public void ClosingCanDestroyReentrantly()
	{
		using var application = new Win32Application(new Win32ApplicationOptions { Title = "Rylogic Win32 reentrant close test" });
		application.Closing += (_, _) => application.Dispose();

		application.Close(37);

		Assert.Equal(true, application.IsClosed);
		Assert.Equal(37, application.ExitCode);
	}

	/// <summary>Prove a mode-change callback failure is surfaced before the requested mode is committed.</summary>
	[Test]
	public void WindowModeSurfacesCallbackFailure()
	{
		using var application = new Win32Application(new Win32ApplicationOptions { Title = "Rylogic Win32 mode failure test" });
		application.Message += (_, args) =>
		{
			if (args.Message == Win32.WM_STYLECHANGED)
				throw new InvalidOperationException("Expected mode-change failure.");
		};

		Assert.Throws<InvalidOperationException>(() => application.WindowMode = EWindowMode.BorderlessFullscreen);
		Assert.Equal(EWindowMode.Windowed, application.WindowMode);
		Assert.Equal(true, application.IsClosed);
	}

	/// <summary>Prove constructor unwind preserves the managed callback failure that rejected initial fullscreen setup.</summary>
	[Test]
	public void ConstructorPreservesCreationCallbackFailure()
	{
		InvalidOperationException? observed = null;
		try
		{
			_ = new Win32Application(new Win32ApplicationOptions
			{
				Title = "Rylogic Win32 constructor failure test",
				WindowMode = EWindowMode.BorderlessFullscreen,
				MessageHandler = args =>
				{
					if (args.Message == Win32.WM_STYLECHANGED)
						throw new InvalidOperationException("Expected constructor callback failure.");
				},
			});
		}
		catch (InvalidOperationException ex)
		{
			observed = ex;
		}

		Assert.Equal(true, observed != null);
		Assert.Equal("Expected constructor callback failure.", observed!.Message);
	}

	/// <summary>Prove native double-click messages preserve the second button-down transition.</summary>
	[Test]
	public void DoubleClickRaisesButtonDown()
	{
		using var application = new Win32Application(new Win32ApplicationOptions { Title = "Rylogic Win32 double-click test" });
		WindowPointerEventArgs? observed = null;
		application.PointerChanged += (_, args) => observed = args;

		User32.SendMessage(application.Handle, Win32.WM_LBUTTONDBLCLK, new IntPtr(Win32.MK_LBUTTON), IntPtr.Zero);

		Assert.Equal(true, observed != null);
		Assert.Equal(EPointerAction.ButtonDown, observed!.Action);
		Assert.Equal(EMouseBtns.Left, observed.ChangedButton);
	}

	/// <summary>Prove borderless desktop fullscreen restores native window styles and placement.</summary>
	[Test]
	public void BorderlessFullscreenRestoresWindowedState()
	{
		using var application = new Win32Application(new Win32ApplicationOptions
		{
			Title = "Rylogic Win32 mode test",
			X = 120,
			Y = 140,
			ClientWidth = 640,
			ClientHeight = 480,
		});
		var style = User32.GetWindowLongPtr(application.Handle, Win32.GWL_STYLE);
		var style_ex = User32.GetWindowLongPtr(application.Handle, Win32.GWL_EXSTYLE);
		var placement = User32.GetWindowPlacement(application.Handle);

		application.WindowMode = EWindowMode.BorderlessFullscreen;
		Assert.Equal(EWindowMode.BorderlessFullscreen, application.WindowMode);
		Assert.Equal(false, User32.GetWindowLongPtr(application.Handle, Win32.GWL_STYLE) == style);

		application.WindowMode = EWindowMode.Windowed;
		var restored = User32.GetWindowPlacement(application.Handle);
		Assert.Equal(style, User32.GetWindowLongPtr(application.Handle, Win32.GWL_STYLE));
		Assert.Equal(style_ex, User32.GetWindowLongPtr(application.Handle, Win32.GWL_EXSTYLE));
		Assert.Equal(placement.showCmd, restored.showCmd);
		Assert.Equal(placement.rcNormalPosition.left, restored.rcNormalPosition.left);
		Assert.Equal(placement.rcNormalPosition.top, restored.rcNormalPosition.top);
		Assert.Equal(placement.rcNormalPosition.right, restored.rcNormalPosition.right);
		Assert.Equal(placement.rcNormalPosition.bottom, restored.rcNormalPosition.bottom);
	}

	/// <summary>Prove display topology changes refit a borderless window to its current monitor.</summary>
	[Test]
	public void BorderlessFullscreenRefitsAfterDisplayChange()
	{
		using var application = new Win32Application(new Win32ApplicationOptions { Title = "Rylogic Win32 display change test" });
		application.WindowMode = EWindowMode.BorderlessFullscreen;
		Assert.Equal(true, User32.SetWindowPos(application.Handle, IntPtr.Zero, 10, 10, 100, 100, Win32.SWP_NOZORDER));

		User32.SendMessage(application.Handle, Win32.WM_DISPLAYCHANGE, IntPtr.Zero, IntPtr.Zero);

		var monitor = User32.MonitorFromWindow(application.Handle, Win32.EMonitorFromFlags.DEFAULT_TO_NEAREST);
		var expected = User32.GetMonitorInfo(monitor).rcMonitor;
		var actual = User32.GetWindowRect(application.Handle);
		Assert.Equal(expected.left, actual.left);
		Assert.Equal(expected.top, actual.top);
		Assert.Equal(expected.right, actual.right);
		Assert.Equal(expected.bottom, actual.bottom);
	}

	/// <summary>Prove native DPI messages reach a typed, policy-controllable notification.</summary>
	[Test]
	public void DpiMessageRaisesTypedNotification()
	{
		using var application = new Win32Application(new Win32ApplicationOptions { Title = "Rylogic Win32 DPI test" });
		WindowDpiChangedEventArgs? observed = null;
		application.DpiChanged += (_, args) =>
		{
			args.ApplySuggestedBounds = false;
			observed = args;
		};

		var bounds = User32.GetWindowRect(application.Handle);
		var memory = Marshal.AllocHGlobal(Marshal.SizeOf<Win32.RECT>());
		try
		{
			Marshal.StructureToPtr(bounds, memory, false);
			var dpi = 144;
			var packed_dpi = new IntPtr(dpi | (dpi << 16));
			User32.SendMessage(application.Handle, Win32.WM_DPICHANGED, packed_dpi, memory);
		}
		finally
		{
			Marshal.FreeHGlobal(memory);
		}

		Assert.Equal(true, observed != null);
		Assert.Equal(144U, observed!.DpiX);
		Assert.Equal(144U, observed.DpiY);
	}

	/// <summary>Record the pre- and post-destruction boundaries while validating their thread and handle contracts.</summary>
	private static void TrackDestruction(Win32Application application, List<string> events)
	{
		application.Destroying += (_, _) =>
		{
			Assert.Equal(application.OwnerThreadId, Environment.CurrentManagedThreadId);
			Assert.Equal(false, application.IsClosed);
			Assert.Equal(false, application.Handle == IntPtr.Zero);
			events.Add("Destroying");
		};
		application.Closed += (_, _) =>
		{
			Assert.Equal(true, application.IsClosed);
			events.Add("Closed");
		};
	}

	/// <summary>Validate exactly one ordered destruction notification pair.</summary>
	private static void AssertDestroyedOnce(Win32Application application, List<string> events)
	{
		Assert.Equal(true, application.IsClosed);
		Assert.Equal(2, events.Count);
		Assert.Equal("Destroying", events[0]);
		Assert.Equal("Closed", events[1]);
	}
}
#endif
