using System;
using System.Diagnostics;
using Microsoft.Win32;

namespace LDraw.MCP.Host;

/// <summary>
/// Manages the per-user "start at logon" entry for the MCP host.
/// Uses the HKCU Run key (per-user, no elevation) rather than a per-machine mechanism so the
/// host's autostart state is owned by the host itself and is trivial to add, remove and migrate.</summary>
internal static class StartupRegistration
{
	// The HKCU Run key launches its values at interactive logon for the current user only.
	private const string RunKeyPath = @"Software\Microsoft\Windows\CurrentVersion\Run";
	private const string ValueName = "LDrawMcpHost";

	/// <summary>True if the host is registered to start at logon</summary>
	public static bool IsRegistered()
	{
		try
		{
			using var key = Registry.CurrentUser.OpenSubKey(RunKeyPath, writable: false);
			return key?.GetValue(ValueName) is string value && value.Length != 0;
		}
		catch (Exception ex)
		{
			Trace.TraceWarning($"Could not read the LDraw MCP host startup registration: {ex.Message}");
			return false;
		}
	}

	/// <summary>Register or unregister the host to start at the current user's logon</summary>
	public static void SetRegistered(bool registered)
	{
		try
		{
			using var key = Registry.CurrentUser.CreateSubKey(RunKeyPath, writable: true)
				?? throw new InvalidOperationException($"Could not open registry key HKCU\\{RunKeyPath}.");

			if (registered)
				key.SetValue(ValueName, QuotedExePath(), RegistryValueKind.String);
			else
				key.DeleteValue(ValueName, throwOnMissingValue: false);
		}
		catch (Exception ex)
		{
			Trace.TraceWarning($"Could not update the LDraw MCP host startup registration: {ex.Message}");
			throw;
		}
	}

	/// <summary>If registered, refresh the stored command so a moved/updated exe still autostarts correctly</summary>
	public static void SyncIfRegistered()
	{
		// The exe path can change across upgrades or installs; keep the Run value pointing at the running exe.
		if (!IsRegistered())
			return;

		try
		{
			using var key = Registry.CurrentUser.OpenSubKey(RunKeyPath, writable: true);
			if (key?.GetValue(ValueName) is string current && !string.Equals(current, QuotedExePath(), StringComparison.OrdinalIgnoreCase))
				key.SetValue(ValueName, QuotedExePath(), RegistryValueKind.String);
		}
		catch (Exception ex)
		{
			Trace.TraceWarning($"Could not synchronise the LDraw MCP host startup registration: {ex.Message}");
		}
	}

	/// <summary>The running executable path, quoted to survive spaces in the install folder</summary>
	private static string QuotedExePath()
	{
		var exe = Environment.ProcessPath ?? string.Empty;
		return $"\"{exe}\"";
	}
}
