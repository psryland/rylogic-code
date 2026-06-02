using System;
using System.IO;
using System.Security.Cryptography;
using System.Text.Json;
using System.Text.Json.Serialization;
using Rylogic.Common;
using Rylogic.Utility;

namespace LDraw.MCP.Host;

/// <summary>
/// Persistent, UI-free settings for the MCP tray host.
/// The host is the single authority for the loopback port and access token; LDraw
/// no longer owns these. Settings live in '%UserData%\MCP\host.json' next to the
/// instance registry so both the host and LDraw resolve the same per-user data dir.</summary>
public sealed class HostSettings
{
	/// <summary>The localhost port the MCP endpoint listens on</summary>
	public int Port { get; set; } = McpProtocol.DefaultPort;

	/// <summary>The bearer token MCP clients must present for 'tools/call'</summary>
	public string AccessToken { get; set; } = string.Empty;

	/// <summary>True if the host accepts MCP connections</summary>
	public bool Listening { get; set; } = true;

	/// <summary>The per-user data directory shared with LDraw (not serialised)</summary>
	[JsonIgnore]
	public string UserDataDir { get; private set; } = string.Empty;

	/// <summary>The host settings file path (not serialised)</summary>
	[JsonIgnore]
	public string FilePath { get; private set; } = string.Empty;

	/// <summary>The loopback MCP endpoint advertised to clients</summary>
	[JsonIgnore]
	public string Endpoint
	{
		get { return $"http://127.0.0.1:{Port}/mcp"; }
	}

	/// <summary>True if the configured port is a usable TCP port</summary>
	[JsonIgnore]
	public bool ValidPort
	{
		get { return Port is >= 1 and <= 65535; }
	}

	/// <summary>Load host settings, computing the LDraw-compatible user data dir</summary>
	public static HostSettings Load()
	{
		// Match LDraw's non-portable user data dir so the host and LDraw share one instance registry.
		var user_data_dir = Util.ResolveAppDataPath("Rylogic", "LDraw");
		Path_.CreateDirs(Path_.CombinePath(user_data_dir, "MCP"));
		var filepath = Path_.CombinePath(user_data_dir, "MCP", "host.json");

		var settings = ReadOrDefault(filepath);
		settings.UserDataDir = user_data_dir;
		settings.FilePath = filepath;
		settings.EnsureAccessToken();
		return settings;
	}

	/// <summary>Read 'filepath' or return defaults when it is missing or invalid</summary>
	private static HostSettings ReadOrDefault(string filepath)
	{
		try
		{
			if (Path_.FileExists(filepath))
				return JsonSerializer.Deserialize<HostSettings>(File.ReadAllText(filepath)) ?? new HostSettings();
		}
		catch (Exception ex)
		{
			// A corrupt settings file should not stop the host starting; fall back to defaults and rewrite on next save.
			System.Diagnostics.Trace.TraceWarning($"Invalid LDraw MCP host settings '{filepath}': {ex.Message}");
		}
		return new HostSettings();
	}

	/// <summary>Persist the current settings atomically</summary>
	public void Save()
	{
		if (FilePath.Length == 0)
			return;

		// Write to a temp file then move so a crash mid-write cannot leave a truncated settings file.
		var json = JsonSerializer.Serialize(this, new JsonSerializerOptions { WriteIndented = true });
		var temp = FilePath + ".tmp";
		File.WriteAllText(temp, json);
		File.Move(temp, FilePath, overwrite: true);
	}

	/// <summary>Ensure an access token exists and return it</summary>
	public string EnsureAccessToken()
	{
		if (string.IsNullOrWhiteSpace(AccessToken))
			AccessToken = GenerateAccessToken();

		return AccessToken;
	}

	/// <summary>Create a new URL-safe bearer token</summary>
	public static string GenerateAccessToken()
	{
		var bytes = RandomNumberGenerator.GetBytes(32);

		// URL-safe base64 because the token is copied into JSON configuration and HTTP headers.
		return Convert.ToBase64String(bytes).TrimEnd('=').Replace('+', '-').Replace('/', '_');
	}

	/// <summary>Return a VS Code MCP configuration snippet for this host</summary>
	public string VSCodeConfiguration()
	{
		var token = EnsureAccessToken();
		return $$"""
{
  "servers": {
    "ldraw": {
      "type": "http",
      "url": "{{Endpoint}}",
      "headers": {
        "{{McpProtocol.AccessTokenHeaderName}}": "{{token}}"
      }
    }
  }
}
""";
	}
}
