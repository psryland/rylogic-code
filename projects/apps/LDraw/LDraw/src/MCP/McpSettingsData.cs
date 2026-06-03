using Rylogic.Common;

namespace LDraw.MCP;

/// <summary>
/// Persistent per-process settings for LDraw's MCP control pipe.
/// The loopback port and access token are owned solely by the LDrawMcpHost tray app; LDraw no
/// longer stores them. The only LDraw-side setting is whether this process exposes its control
/// pipe so the host can discover and drive it.</summary>
public sealed class McpSettingsData :SettingsSet<McpSettingsData>
{
	public McpSettingsData()
	{
		// Default on so a normally launched LDraw is immediately controllable by the host, and so an
		// auto-launched instance is never dead on arrival. The host forces control on for instances it
		// launches itself (via the launch-nonce switch) regardless of this persisted value.
		AllowControl = true;
	}

	/// <summary>True if this LDraw process exposes its MCP control pipe to the host</summary>
	public bool AllowControl
	{
		get => get<bool>(nameof(AllowControl));
		set => set(nameof(AllowControl), value);
	}
}