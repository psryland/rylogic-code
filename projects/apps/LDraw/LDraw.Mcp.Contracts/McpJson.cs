using System.Text.Json;
using System.Text.Json.Serialization.Metadata;

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

		// The MCP SDK marks these options read-only when building the tool catalogue, which requires an
		// explicit type info resolver. Use the reflection-based resolver so DTO (de)serialisation works
		// regardless of whether reflection-based serialisation is enabled by default in the hosting process.
		TypeInfoResolver = new DefaultJsonTypeInfoResolver(),
	};

	/// <summary>The serializer options used for one-line local IPC messages</summary>
	public static JsonSerializerOptions LineOptions { get; } = new(Options)
	{
		// The instance pipe protocol uses ReadLine/WriteLine, so payloads must not contain embedded newlines.
		WriteIndented = false,
	};
}
