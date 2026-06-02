namespace LDraw.MCP;

/// <summary>
/// Shared protocol constants for the LDraw MCP host and LDraw.exe.
/// These live in the contracts library so both the host (which owns the HTTP
/// endpoint) and LDraw use a single source of truth for the loopback port and
/// the access-token header name.</summary>
public static class McpProtocol
{
	/// <summary>The default localhost port for the LDraw MCP broker</summary>
	public const int DefaultPort = 38987;

	/// <summary>The HTTP header used to carry the local LDraw MCP access token</summary>
	public const string AccessTokenHeaderName = "X-LDraw-MCP-Token";

	/// <summary>
	/// The version of the host/LDraw control protocol (registry schema + pipe commands).
	/// The host pings an instance and compares this value so it can give a clear "update LDraw"
	/// error rather than failing obscurely when the two sides disagree on the pipe contract.</summary>
	public const int ProtocolVersion = 1;

	/// <summary>
	/// Command-line switch the host passes to an auto-launched LDraw process.
	/// Its presence both supplies the launch nonce (echoed back in the registration so the host can
	/// match the process it started) and forces this instance to allow MCP control regardless of the
	/// persisted AllowControl setting, so an auto-launched instance is never dead on arrival.</summary>
	public const string LaunchNonceSwitch = "--mcp-launch-nonce";
}
