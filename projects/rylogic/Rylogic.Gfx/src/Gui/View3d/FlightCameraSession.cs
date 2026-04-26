using System;
using System.Windows;
using System.Windows.Input;
using Rylogic.Gfx;
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
		}

		public void Dispose()
		{
			if (m_disposed)
				return;

			m_disposed = true;
			if (m_cancel != null)
				m_key_scope.PreviewKeyDown -= HandlePreviewKeyDown;
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
	}
}
