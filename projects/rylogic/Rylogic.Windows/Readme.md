# Rylogic.Core.Windows

This assembly contains windows specific code.
It includes
- interop with WIN32
It excludes:
- WinForms
- WPF

## Win32Application cursors

The registered window class uses the shared system arrow. `ClientCursor` optionally borrows a native cursor for that window; zero restores the arrow.
Assign it on the HWND owner thread, keep owned cursors alive until the window has closed, then deselect any active owned cursor before destroying it.
The application never destroys a borrowed cursor. Assignment takes effect at the next unhandled `WM_SETCURSOR`, not by moving the pointer.

`Win32ApplicationOptions.MessageHandler` and `Message` subscribers run before the client fallback. A UI handler can call `User32.SetCursor(ibeam)`,
set `args.Result = new IntPtr(1)` and `args.Handled = true` for `WM_SETCURSOR` over its textbox. Other subscribers should respect `Handled`.
Leave other messages unhandled to retain the per-window default. The fallback handles only `HTCLIENT` for its own HWND;
non-client borders and child-window requests continue through `DefWindowProc`, preserving native sizing cursors.
