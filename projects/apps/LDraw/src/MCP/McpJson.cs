using System.Text.Json;

namespace LDraw.MCP;

/// <summary>Shared JSON configuration for MCP payloads and local IPC</summary>
internal static class McpJson
{
	/// <summary>The serializer options used for MCP-related JSON payloads</summary>
	public static JsonSerializerOptions Options { get; } = new(JsonSerializerDefaults.Web)
	{
		// MCP schemas and JSON-RPC payloads conventionally use lower snake case rather than .NET property casing.
		DictionaryKeyPolicy = JsonNamingPolicy.SnakeCaseLower,
		PropertyNamingPolicy = JsonNamingPolicy.SnakeCaseLower,
		WriteIndented = true,
	};
}
