using System;
using System.Windows;
using System.Windows.Input;
using System.Windows.Media;
using Rylogic.Gui.WPF;
using Rylogic.Interop.Win32;

namespace LDraw.UI;

/// <summary>
/// A borderless window that presents a scene control over the full bounds of one monitor, including the taskbar area.
/// The window takes the scene control as its content and releases it when closed, so the caller can return the control to its normal host.
/// F11, or an Escape key press that the scene does not use, closes the window.
/// </summary>
public sealed class SceneFullscreenUI :Window
{
	public SceneFullscreenUI(Window owner, IntPtr monitor, ChartControl scene, string title)
	{
		// Present only the scene, without a frame, so the client area can cover the whole monitor
		Owner = owner;
		Icon = owner.Icon;
		Title = $"{title} - LDraw";
		WindowStyle = WindowStyle.None;
		ResizeMode = ResizeMode.NoResize;
		WindowStartupLocation = WindowStartupLocation.Manual;
		Background = Brushes.Black;
		m_monitor = monitor;
		m_scene = scene;
		Content = scene;
	}
	protected override void OnSourceInitialized(EventArgs e)
	{
		// Position the window before it is first shown so it never appears on the wrong monitor
		base.OnSourceInitialized(e);
		ApplyMonitorBounds();
	}
	protected override void OnDpiChanged(DpiScale old_dpi, DpiScale new_dpi)
	{
		// WPF applies the system's suggested rectangle after a DPI change. That rectangle keeps the size in DIPs, not the monitor bounds,
		// so re-apply the bounds once WPF has finished handling the change.
		base.OnDpiChanged(old_dpi, new_dpi);
		Dispatcher.BeginInvoke(ApplyMonitorBounds);
	}
	protected override void OnContentRendered(EventArgs e)
	{
		// Give the scene keyboard focus so that keyboard navigation works without a click first
		base.OnContentRendered(e);
		m_scene.Scene.Focus();
	}
	protected override void OnPreviewKeyDown(KeyEventArgs e)
	{
		// F11 always leaves fullscreen, even if the scene would otherwise use the key
		base.OnPreviewKeyDown(e);
		if (e.Handled || e.Key != Key.F11)
			return;

		e.Handled = true;
		Close();
	}
	protected override void OnKeyDown(KeyEventArgs e)
	{
		// Escape leaves fullscreen only when the scene has not used it (e.g. to cancel a mouse operation)
		base.OnKeyDown(e);
		if (e.Handled || e.Key != Key.Escape)
			return;

		e.Handled = true;
		Close();
	}
	protected override void OnClosed(EventArgs e)
	{
		// Release the scene before 'Closed' is raised so that handlers can return it to its normal host
		Content = null;
		base.OnClosed(e);
	}

	/// <summary>Cover the full bounds of the monitor. The coordinates are in the same space as 'SetWindowPos', which handles negative desktop positions and per-monitor DPI.</summary>
	private void ApplyMonitorBounds()
	{
		var bounds = User32.GetMonitorInfo(m_monitor).rcMonitor;
		User32.SetWindowPos(this.Hwnd(), IntPtr.Zero, bounds.left, bounds.top, bounds.width, bounds.height, (uint)(Win32.SWP_NOZORDER | Win32.SWP_NOACTIVATE));
	}

	/// <summary>The monitor that this window covers</summary>
	private readonly IntPtr m_monitor;

	/// <summary>The scene control presented by this window</summary>
	private readonly ChartControl m_scene;
}
