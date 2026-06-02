using System;
using System.Threading;
using System.Threading.Tasks;
using System.Windows;
using LDraw.MCP.Host.UI;

namespace LDraw.MCP.Host;

/// <summary>The tray host application entry point</summary>
public partial class App :Application
{
	private Mutex? m_single_instance;
	private HostSettings m_settings = null!;
	private McpServer m_server = null!;
	private TrayWindow m_tray = null!;

	/// <summary>Application startup</summary>
	protected override void OnStartup(StartupEventArgs e)
	{
		base.OnStartup(e);

		// Only one host per user session may own the loopback port and tray icon.
		m_single_instance = new Mutex(initiallyOwned: true, name: @"Local\LDrawMcpHost.SingleInstance", out var is_new);
		if (!is_new)
		{
			MessageBox.Show("The LDraw MCP host is already running.", "LDraw MCP Host", MessageBoxButton.OK, MessageBoxImage.Information);
			Shutdown();
			return;
		}

		m_settings = HostSettings.Load();
		m_server = new McpServer(m_settings);
		m_tray = new TrayWindow(m_settings, m_server);

		// Start the endpoint without blocking the UI thread; status surfaces in the tray tooltip/menu.
		// Observe faults so a failed startup is logged rather than silently lost on the fire-and-forget task.
		m_server.StartAsync().ContinueWith(t =>
		{
			System.Diagnostics.Trace.TraceError($"LDraw MCP host failed to start: {t.Exception?.GetBaseException()}");
		}, TaskContinuationOptions.OnlyOnFaulted);
	}

	/// <summary>Application shutdown</summary>
	protected override void OnExit(ExitEventArgs e)
	{
		m_tray?.Dispose();
		m_server?.Dispose();
		m_single_instance?.Dispose();
		base.OnExit(e);
	}
}
