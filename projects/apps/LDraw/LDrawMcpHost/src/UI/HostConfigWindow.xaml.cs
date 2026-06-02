using System;
using System.Globalization;
using System.Threading.Tasks;
using System.Windows;
using Microsoft.Win32;

namespace LDraw.MCP.Host.UI;

/// <summary>
/// The host configuration window, opened from the tray menu.
/// Edits the single host-owned settings file and, on save, applies endpoint changes by
/// restarting the MCP server and updates the per-user "start at logon" registration.</summary>
public partial class HostConfigWindow :Window
{
	private readonly HostSettings m_settings;
	private readonly McpServer m_server;

	/// <summary>Create the settings window bound to 'settings' and 'server'</summary>
	public HostConfigWindow(HostSettings settings, McpServer server)
	{
		m_settings = settings;
		m_server = server;
		InitializeComponent();

		// Use the host/LDraw application icon so the settings window is visually consistent with the tray app.
		try
		{
			var exe = Environment.ProcessPath;
			if (exe != null)
			{
				using var icon = System.Drawing.Icon.ExtractAssociatedIcon(exe);
				if (icon != null)
					Icon = System.Windows.Interop.Imaging.CreateBitmapSourceFromHIcon(icon.Handle, Int32Rect.Empty, System.Windows.Media.Imaging.BitmapSizeOptions.FromEmptyOptions());
			}
		}
		catch (Exception ex)
		{
			System.Diagnostics.Trace.TraceWarning($"Could not load the settings window icon: {ex.Message}");
		}

		LoadFromSettings();
	}

	/// <summary>Populate the controls from the current settings</summary>
	private void LoadFromSettings()
	{
		m_tb_port.Text = m_settings.Port.ToString(CultureInfo.InvariantCulture);
		m_cb_listening.IsChecked = m_settings.Listening;
		m_cb_autolaunch.IsChecked = m_settings.AutoLaunch;
		m_tb_timeout.Text = m_settings.LaunchTimeoutSeconds.ToString(CultureInfo.InvariantCulture);
		m_tb_exepath.Text = m_settings.LDrawExePath;
		m_tb_token.Text = m_settings.EnsureAccessToken();
		m_cb_startup.IsChecked = StartupRegistration.IsRegistered();
	}

	/// <summary>Browse for an LDraw.exe override</summary>
	private void HandleBrowse(object sender, RoutedEventArgs e)
	{
		var dlg = new OpenFileDialog
		{
			Title = "Select LDraw.exe",
			Filter = "LDraw executable (LDraw.exe)|LDraw.exe|Executables (*.exe)|*.exe",
			CheckFileExists = true,
		};
		if (dlg.ShowDialog(this) == true)
			m_tb_exepath.Text = dlg.FileName;
	}

	/// <summary>Copy the access token to the clipboard</summary>
	private void HandleCopyToken(object sender, RoutedEventArgs e)
	{
		try
		{
			Clipboard.SetText(m_tb_token.Text);
		}
		catch (Exception ex)
		{
			MessageBox.Show(this, $"Could not copy the access token: {ex.Message}", "LDraw MCP Host", MessageBoxButton.OK, MessageBoxImage.Warning);
		}
	}

	/// <summary>Generate a new access token, shown immediately and persisted on save</summary>
	private void HandleRegenerate(object sender, RoutedEventArgs e)
	{
		var result = MessageBox.Show(this,
			"Regenerating the access token will break existing MCP client configurations until they are updated.\n\nContinue?",
			"LDraw MCP Host", MessageBoxButton.OKCancel, MessageBoxImage.Exclamation);
		if (result != MessageBoxResult.OK)
			return;

		// Show the new token now; it is committed to settings only when the user saves.
		m_tb_token.Text = HostSettings.GenerateAccessToken();
	}

	/// <summary>Validate, persist, and apply the settings</summary>
	private async void HandleSave(object sender, RoutedEventArgs e)
	{
		if (!int.TryParse(m_tb_port.Text, NumberStyles.Integer, CultureInfo.InvariantCulture, out var port) || port is < 1 or > 65535)
		{
			MessageBox.Show(this, "Enter a port number between 1 and 65535.", "LDraw MCP Host", MessageBoxButton.OK, MessageBoxImage.Warning);
			return;
		}
		if (!int.TryParse(m_tb_timeout.Text, NumberStyles.Integer, CultureInfo.InvariantCulture, out var timeout) || timeout < 1)
		{
			MessageBox.Show(this, "Enter a launch timeout of at least 1 second.", "LDraw MCP Host", MessageBoxButton.OK, MessageBoxImage.Warning);
			return;
		}

		// Detect endpoint-affecting changes before mutating settings so the server is only restarted when needed.
		var endpoint_changed = port != m_settings.Port || (m_cb_listening.IsChecked == true) != m_settings.Listening;

		m_settings.Port = port;
		m_settings.Listening = m_cb_listening.IsChecked == true;
		m_settings.AutoLaunch = m_cb_autolaunch.IsChecked == true;
		m_settings.LaunchTimeoutSeconds = timeout;
		m_settings.LDrawExePath = m_tb_exepath.Text.Trim();
		m_settings.AccessToken = m_tb_token.Text.Trim();
		m_settings.Save();

		// The Run-key registration is owned by the host, applied immediately so it matches the saved checkbox.
		try
		{
			StartupRegistration.SetRegistered(m_cb_startup.IsChecked == true);
		}
		catch (Exception ex)
		{
			MessageBox.Show(this, $"Settings saved, but the start-at-logon setting could not be updated: {ex.Message}", "LDraw MCP Host", MessageBoxButton.OK, MessageBoxImage.Warning);
		}

		// Apply endpoint changes without blocking the UI thread; the tray status reflects the result.
		if (endpoint_changed)
		{
			m_btn_save.IsEnabled = false;
			try
			{
				await Task.Run(m_server.RestartAsync).ConfigureAwait(true);
			}
			catch (Exception ex)
			{
				MessageBox.Show(this, $"The endpoint could not be restarted: {ex.GetBaseException().Message}", "LDraw MCP Host", MessageBoxButton.OK, MessageBoxImage.Warning);
			}
			finally
			{
				m_btn_save.IsEnabled = true;
			}
		}

		DialogResult = true;
		Close();
	}
}
