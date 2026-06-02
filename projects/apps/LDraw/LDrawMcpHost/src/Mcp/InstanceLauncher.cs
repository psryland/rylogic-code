using System;
using System.Diagnostics;
using System.IO;

namespace LDraw.MCP.Host;

/// <summary>
/// Launches LDraw.exe on demand for the MCP host.
/// LDraw is normally deployed alongside the host, so the default exe path is the host's
/// sibling 'LDraw.exe'; an explicit override in host settings supports dev/non-standard layouts.</summary>
internal sealed class InstanceLauncher
{
	private readonly HostSettings m_settings;

	/// <summary>Create a launcher bound to the host settings that supply the LDraw path</summary>
	public InstanceLauncher(HostSettings settings)
	{
		m_settings = settings;
	}

	/// <summary>Resolve the LDraw.exe path: an explicit override when set, else the host's sibling executable</summary>
	public string ResolveExePath()
	{
		// An explicit path wins, but only when it actually exists; otherwise fall back to the install-folder sibling.
		if (m_settings.LDrawExePath.Length != 0 && File.Exists(m_settings.LDrawExePath))
			return m_settings.LDrawExePath;

		var host_dir = Path.GetDirectoryName(Environment.ProcessPath) ?? string.Empty;
		return Path.Combine(host_dir, "LDraw.exe");
	}

	/// <summary>Launch LDraw with a unique launch nonce and return it so the host can match the process it started</summary>
	public string Launch()
	{
		var exe = ResolveExePath();
		if (!File.Exists(exe))
			throw new FileNotFoundException($"LDraw executable not found at '{exe}'. Set the LDraw path in the MCP host settings.", exe);

		// The nonce is echoed back in the instance's registration; it is how the host distinguishes the process it
		// launched from any other LDraw that may register concurrently, and it forces MCP control on for that instance.
		var nonce = Guid.NewGuid().ToString("N");
		var psi = new ProcessStartInfo
		{
			FileName = exe,
			UseShellExecute = false,
			WorkingDirectory = Path.GetDirectoryName(exe),
		};
		psi.ArgumentList.Add($"{McpProtocol.LaunchNonceSwitch}={nonce}");
		Process.Start(psi);
		return nonce;
	}
}
