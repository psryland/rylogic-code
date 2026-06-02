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
}
