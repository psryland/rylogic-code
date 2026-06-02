using System;
using System.Diagnostics;
using System.Windows;
using System.Windows.Controls;
using Rylogic.Common;

namespace LDraw.MCP.Host.UI;

/// <summary>
/// A hidden window that hosts the system-tray notification icon and its context menu.
/// The window is never shown; it exists only so the tray icon has a WPF host and the
/// context-menu commands have a dispatcher to run on.</summary>
public partial class TrayWindow :Window, IDisposable
{
	private readonly HostSettings m_settings;
	private readonly McpServer m_server;
	private readonly MenuItem m_status_item;
	private HostConfigWindow? m_config_window;

	/// <summary>Create the tray window bound to 'settings' and 'server'</summary>
	public TrayWindow(HostSettings settings, McpServer server)
	{
		m_settings = settings;
		m_server = server;
		InitializeComponent();

		// The status header lives in the context menu held in Window.Resources, which is not part of the named
		// element tree, so resolve it from the resource dictionary rather than relying on a generated field.
		var menu = (ContextMenu)Resources["TrayMenu"];
		m_status_item = (MenuItem)menu.Items[0];

		// Use the main LDraw application icon for the tray entry. The icon is embedded in this exe via the
		// project's ApplicationIcon (set to LDraw's icon.ico), so extracting it from the running exe yields
		// the same icon as the main LDraw app without bundling a separate icon asset or duplicating the file.
		try
		{
			var exe = Environment.ProcessPath;
			if (exe != null)
				m_notify_icon.Icon = System.Drawing.Icon.ExtractAssociatedIcon(exe);
		}
		catch (Exception ex)
		{
			Trace.TraceWarning($"Could not load the LDraw MCP host tray icon: {ex.Message}");
		}

		m_server.StatusChanged += HandleStatusChanged;
		UpdateStatus();
	}

	/// <summary>Dispose the tray window and its icon</summary>
	public void Dispose()
	{
		m_server.StatusChanged -= HandleStatusChanged;
		m_notify_icon.Dispose();
	}

	/// <summary>Refresh the tooltip and status header when the server status changes</summary>
	private void HandleStatusChanged(object? sender, EventArgs e)
	{
		// Status changes are raised from the Kestrel start path, so marshal back to the UI thread.
		Dispatcher.BeginInvoke(new Action(UpdateStatus));
	}

	/// <summary>Mirror the current server status into the tray UI</summary>
	private void UpdateStatus()
	{
		m_notify_icon.ToolTipText = m_server.StatusMessage;
		m_status_item.Header = m_server.StatusMessage;
	}

	/// <summary>Copy the VS Code MCP configuration snippet to the clipboard</summary>
	private void HandleCopyConfig(object sender, RoutedEventArgs e)
	{
		try
		{
			Clipboard.SetText(m_settings.VSCodeConfiguration());
		}
		catch (Exception ex)
		{
			MessageBox.Show($"Could not copy the MCP configuration: {ex.Message}", "LDraw MCP Host", MessageBoxButton.OK, MessageBoxImage.Warning);
		}
	}

	/// <summary>Open the per-user MCP data folder in Explorer</summary>
	private void HandleOpenFolder(object sender, RoutedEventArgs e)
	{
		var folder = Path_.CombinePath(m_settings.UserDataDir, "MCP");
		Process.Start(new ProcessStartInfo { FileName = folder, UseShellExecute = true });
	}

	/// <summary>Open the host settings window (modal, single instance)</summary>
	private void HandleSettings(object sender, RoutedEventArgs e)
	{
		// Reuse an already-open settings window rather than stacking several; the tray menu can be clicked repeatedly.
		if (m_config_window != null)
		{
			m_config_window.Activate();
			return;
		}

		m_config_window = new HostConfigWindow(m_settings, m_server);
		m_config_window.Closed += (_, _) => m_config_window = null;
		m_config_window.Show();
	}

	/// <summary>Quit the host after confirmation; quitting drops the MCP endpoint</summary>
	private void HandleQuit(object sender, RoutedEventArgs e)
	{
		var result = MessageBox.Show("Quit the LDraw MCP host? The MCP endpoint will no longer be available to Copilot.", "LDraw MCP Host", MessageBoxButton.YesNo, MessageBoxImage.Question);
		if (result == MessageBoxResult.Yes)
			Application.Current.Shutdown();
	}
}
