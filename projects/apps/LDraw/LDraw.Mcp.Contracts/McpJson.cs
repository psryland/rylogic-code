using System.Text.Json;

namespace LDraw.MCP;

/// <summary>Shared JSON configuration for MCP payloads and local IPC</summary>
public static class McpJson
{
	/// <summary>The serializer options used for MCP-related JSON payloads</summary>
	public static JsonSerializerOptions Options { get; } = new(JsonSerializerDefaults.Web)
	{
		// MCP schemas and JSON-RPC payloads conventionally use lower snake case rather than .NET property casing.
		DictionaryKeyPolicy = JsonNamingPolicy.SnakeCaseLower,
		PropertyNamingPolicy = JsonNamingPolicy.SnakeCaseLower,
		WriteIndented = true,
	};

	/// <summary>The serializer options used for one-line local IPC messages</summary>
	public static JsonSerializerOptions LineOptions { get; } = new(Options)
	{
		// The instance pipe protocol uses ReadLine/WriteLine, so payloads must not contain embedded newlines.
		WriteIndented = false,
	};
}
