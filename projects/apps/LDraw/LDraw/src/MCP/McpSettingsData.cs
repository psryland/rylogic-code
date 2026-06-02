using System;
using System.Security.Cryptography;
using Rylogic.Common;

namespace LDraw.MCP;

/// <summary>Persistent settings for the embedded MCP server</summary>
public sealed class McpSettingsData :SettingsSet<McpSettingsData>
{
	public McpSettingsData()
	{
		Enabled = false;
		Port = DefaultPort;
		AccessToken = string.Empty;
	}

	/// <summary>The default localhost port for the LDraw MCP broker</summary>
	public const int DefaultPort = McpProtocol.DefaultPort;

	/// <summary>The preferred HTTP header for local LDraw MCP access tokens</summary>
	public const string AccessTokenHeaderName = McpProtocol.AccessTokenHeaderName;

	/// <summary>True if the embedded MCP service should be active</summary>
	public bool Enabled
	{
		get => get<bool>(nameof(Enabled));
		set => set(nameof(Enabled), value);
	}

	/// <summary>The localhost port used by the MCP broker</summary>
	public int Port
	{
		get => get<int>(nameof(Port));
		set => set(nameof(Port), value);
	}

	/// <summary>The access token required by MCP clients</summary>
	public string AccessToken
	{
		get => get<string>(nameof(AccessToken));
		set => set(nameof(AccessToken), value ?? string.Empty);
	}

	/// <summary>The HTTP endpoint exposed by the broker</summary>
	public string Endpoint
	{
		get { return $"http://127.0.0.1:{Port}/mcp"; }
	}

	/// <summary>Ensure the settings contain an access token and return it</summary>
	public string EnsureAccessToken()
	{
		// Existing settings files won't have this value until the MCP feature is first used.
		if (string.IsNullOrWhiteSpace(AccessToken))
			AccessToken = GenerateAccessToken();

		return AccessToken;
	}

	/// <summary>Create a new URL-safe bearer token</summary>
	public static string GenerateAccessToken()
	{
		var bytes = RandomNumberGenerator.GetBytes(32);

		// Use the URL-safe base64 alphabet because the token is copied into JSON configuration and HTTP headers.
		return Convert.ToBase64String(bytes).TrimEnd('=').Replace('+', '-').Replace('/', '_');
	}

	/// <summary>Return a VS Code MCP configuration snippet for this broker</summary>
	public string VSCodeConfiguration()
	{
		// Generate the token lazily so copying the configuration always produces a usable snippet.
		var token = EnsureAccessToken();
		return $$"""
{
  "servers": {
    "ldraw": {
      "type": "http",
      "url": "{{Endpoint}}",
      "headers": {
        "{{AccessTokenHeaderName}}": "{{token}}"
      }
    }
  }
}
""";
	}
}
