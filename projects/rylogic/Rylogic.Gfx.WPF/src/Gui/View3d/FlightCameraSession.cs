using System;
using System.Windows;
using System.Windows.Input;
using Rylogic.Extn;
using Rylogic.Gfx;
using Rylogic.Interop.Win32;
using Rylogic.Utility;

namespace Rylogic.Gui.WPF
{
	internal sealed class FlightCameraSession :IDisposable
	{
		private readonly View3d.Window m_window;
		private readonly UIElement m_key_scope;
		private readonly Action? m_cancel;
		private readonly Cursor? m_prev_override_cursor;
		private readonly IDisposable? m_state_scope;
		private readonly IDisposable? m_mouse_capture;
		private readonly IDisposable? m_cursor_clip;
		private bool m_disposed;

		public FlightCameraSession(View3d.Window window, UIElement key_scope, Action? cancel, Func<IDisposable?>? state_scope = null)
		{
			m_window = window;
			m_key_scope = key_scope;
			m_cancel = cancel;
			m_prev_override_cursor = Mouse.OverrideCursor;
			m_state_scope = state_scope?.Invoke();

			m_window.FlightCameraEnabled = true;
			Mouse.OverrideCursor = Cursors.None;
			if (m_cancel != null)
				m_key_scope.PreviewKeyDown += HandlePreviewKeyDown;
			m_key_scope.Focus();
			m_mouse_capture = m_key_scope.CaptureMouseScope();
			m_cursor_clip = CursorClipScope(m_key_scope);
		}

		public void Dispose()
		{
			if (m_disposed)
				return;

			m_disposed = true;
			if (m_cancel != null)
				m_key_scope.PreviewKeyDown -= HandlePreviewKeyDown;
			Util.Dispose(m_cursor_clip);
			Util.Dispose(m_mouse_capture);
			if (m_window.FlightCameraEnabled)
				m_window.FlightCameraEnabled = false;
			Mouse.OverrideCursor = m_prev_override_cursor;
			Util.Dispose(m_state_scope);
		}

		private void HandlePreviewKeyDown(object sender, KeyEventArgs args)
		{
			if (args.Key != Key.Escape)
				return;

			args.Handled = true;
			m_cancel?.Invoke();
		}

		private static IDisposable CursorClipScope(UIElement key_scope)
		{
			return Scope.Create(
				() => User32.ClipCursor(CursorClipRect(key_scope)),
				() => User32.ClipCursor(null));
		}
		private static Win32.RECT CursorClipRect(UIElement key_scope)
		{
			var top_left = key_scope.PointToScreen(new Point(0, 0));
			var bottom_right = key_scope.PointToScreen(new Point(key_scope.RenderSize.Width, key_scope.RenderSize.Height));
			return Win32.RECT.FromLTRB(
				(int)Math.Floor(top_left.X),
				(int)Math.Floor(top_left.Y),
				(int)Math.Ceiling(bottom_right.X),
				(int)Math.Ceiling(bottom_right.Y));
		}
	}
}
