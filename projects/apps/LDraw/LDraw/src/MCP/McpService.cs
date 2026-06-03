using System;
using System.ComponentModel;
using System.Threading;
using System.Threading.Tasks;
using Rylogic.Common;
using Rylogic.Utility;

namespace LDraw.MCP;

/// <summary>
/// The LDraw-side owner of the per-process MCP control pipe.
/// The HTTP MCP endpoint, broker, and authentication now live in the standalone
/// LDrawMcpHost tray application. LDraw only hosts its named-pipe server and registry
/// entry so the host can discover and drive this process; there is no port binding,
/// broker election, or token handling here any more.</summary>
public sealed class McpService :IDisposable, INotifyPropertyChanged
{
	private readonly Model m_model;
	private readonly InstanceRegistry m_registry;
	private readonly SemaphoreSlim m_gate;
	private LDrawInstanceHost? m_instance_host;
	private bool m_started;

	/// <summary>Create the MCP control-pipe owner for 'model'</summary>
	public McpService(Model model)
	{
		m_model = model;
		m_registry = new InstanceRegistry(model.StartupOptions.UserDataDir);
		m_gate = new SemaphoreSlim(1, 1);
		StatusMessage = "MCP control disabled";

		model.Settings.SettingChange += HandleSettingChange;
	}

	/// <summary>A user-facing status string for the Options UI</summary>
	public string StatusMessage
	{
		get => field;
		private set
		{
			if (field == value) return;
			field = value;
			NotifyPropertyChanged(nameof(StatusMessage));
		}
	} = string.Empty;

	/// <summary>Start the MCP control pipe if enabled</summary>
	public async Task StartAsync()
	{
		await m_gate.WaitAsync().ConfigureAwait(false);
		try
		{
			m_started = true;
		}
		finally
		{
			m_gate.Release();
		}

		// Apply settings outside the lock because the settings snapshot is marshalled to the UI thread.
		await ApplyCurrentSettingsAsync().ConfigureAwait(false);
	}

	/// <summary>Stop the MCP control pipe</summary>
	public async Task StopAsync()
	{
		await m_gate.WaitAsync().ConfigureAwait(false);
		try
		{
			m_started = false;
			await StopInstanceUnlockedAsync().ConfigureAwait(false);
			StatusMessage = "MCP control stopped";
		}
		finally
		{
			m_gate.Release();
		}
	}

	/// <summary>Dispose of the MCP service</summary>
	public void Dispose()
	{
		m_model.Settings.SettingChange -= HandleSettingChange;
		StopAsync().GetAwaiter().GetResult();
		m_gate.Dispose();
	}

	/// <summary>Handle changes to MCP settings</summary>
	private void HandleSettingChange(object? sender, SettingChangeEventArgs e)
	{
		if (e.Before)
			return;

		// Nested settings bubble through the root settings set, so only react to changes from the MCP set itself or replacement of the MCP set.
		if (!ReferenceEquals(e.SettingSet, m_model.Settings.MCP) && e.Key != nameof(SettingsData.MCP))
			return;

		_ = ApplyCurrentSettingsAsync();
	}

	/// <summary>Apply the current in-memory MCP settings</summary>
	private async Task ApplyCurrentSettingsAsync()
	{
		try
		{
			// Read the mutable settings on the UI thread before taking the lock so the gate is never held while
			// waiting on the UI thread; that ordering also protects Dispose, which blocks the UI thread on the gate.
			var allow_control = await CaptureAllowControlAsync().ConfigureAwait(false);

			await m_gate.WaitAsync().ConfigureAwait(false);
			try
			{
				if (!m_started)
					return;

				// AllowControl gates whether this LDraw process exposes its control pipe to the MCP host.
				if (!allow_control)
				{
					await StopInstanceUnlockedAsync().ConfigureAwait(false);
					StatusMessage = "MCP control disabled";
					return;
				}

				StartInstanceUnlocked();
				StatusMessage = "MCP control pipe running";
			}
			finally
			{
				m_gate.Release();
			}
		}
		catch (Exception ex)
		{
			Log.Write(ELogLevel.Error, ex, "Failed to configure the LDraw MCP control pipe.");
			StatusMessage = $"MCP control failed: {ex.Message}";
		}
	}

	/// <summary>Read whether this process allows MCP control, on the UI thread</summary>
	private Task<bool> CaptureAllowControlAsync()
	{
		// A launch nonce means the host auto-launched this process specifically to drive it, so control is
		// forced on even if the persisted setting is off; otherwise honour the user's AllowControl choice.
		return m_model.InvokeAsync(
			() => m_model.Settings.MCP.AllowControl || m_model.StartupOptions.McpLaunchNonce.Length != 0,
			TimeSpan.FromSeconds(2));
	}

	/// <summary>Start the local instance host if needed</summary>
	private void StartInstanceUnlocked()
	{
		if (m_instance_host != null)
			return;

		m_instance_host = new LDrawInstanceHost(m_model, m_registry, m_model.StartupOptions.McpLaunchNonce);
		m_instance_host.Start();
	}

	/// <summary>Stop the local instance host if it is running</summary>
	private async Task StopInstanceUnlockedAsync()
	{
		if (m_instance_host == null)
			return;

		await m_instance_host.StopAsync().ConfigureAwait(false);
		Util.Dispose(ref m_instance_host);
	}

	/// <inheritdoc/>
	public event PropertyChangedEventHandler? PropertyChanged;

	/// <summary>Raise property changed for 'prop_name'</summary>
	private void NotifyPropertyChanged(string prop_name)
	{
		PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(prop_name));
	}
}
