using System;
using System.IO;
using System.Text.Json;
using System.Text.Json.Serialization;

namespace Rylogic.Audio.Authoring;

/// <summary>JSON settings and file helpers shared by every authoring file format.</summary>
public static class AuthoringJson
{
	/// <summary>
	/// Options for reading and writing authoring files. Names are snake_case, enums are snake_case strings, and output is tab-indented so the
	/// version-controlled files diff cleanly.
	/// </summary>
	public static JsonSerializerOptions Options { get; } = CreateOptions();

	/// <summary>Read a JSON file into 'T'. Throws if the file is missing or does not contain a 'T'.</summary>
	public static T Read<T>(string path)
	{
		// Name the file in the error so a malformed file can be found
		using var stream = File.OpenRead(path);
		try
		{
			return JsonSerializer.Deserialize<T>(stream, Options) ?? throw new InvalidDataException($"'{path}' is empty.");
		}
		catch (JsonException ex)
		{
			throw new InvalidDataException($"'{path}' is not valid: {ex.Message}", ex);
		}
	}

	/// <summary>Write 'value' to 'path' atomically, so a reader never sees a partly written file.</summary>
	public static void Write<T>(string path, T value)
	{
		// Write a temporary file next to the target, then replace the target in one rename
		var bytes = JsonSerializer.SerializeToUtf8Bytes(value, Options);
		WriteAtomic(path, bytes);
	}

	/// <summary>Serialise 'value' to a compact string. Used for hashing, so the result depends only on the value.</summary>
	public static string ToCanonicalString<T>(T value)
	{
		return JsonSerializer.Serialize(value, CanonicalOptions);
	}

	/// <summary>Write 'bytes' to 'path' atomically, creating the directory if needed.</summary>
	public static void WriteAtomic(string path, ReadOnlySpan<byte> bytes)
	{
		// A unique temporary name stops two writers from sharing a temporary file
		Directory.CreateDirectory(Path.GetDirectoryName(Path.GetFullPath(path))!);
		var temporary_path = $"{path}.{Guid.NewGuid():N}.tmp";
		try
		{
			File.WriteAllBytes(temporary_path, bytes.ToArray());
			File.Move(temporary_path, path, overwrite: true);
		}
		finally
		{
			File.Delete(temporary_path);
		}
	}

	/// <summary>Create the options used for files on disk.</summary>
	private static JsonSerializerOptions CreateOptions()
	{
		// Files are read leniently about trailing commas and comments because people edit them by hand
		var options = new JsonSerializerOptions
		{
			PropertyNamingPolicy = JsonNamingPolicy.SnakeCaseLower,
			DefaultIgnoreCondition = JsonIgnoreCondition.WhenWritingNull,
			WriteIndented = true,
			IndentCharacter = '\t',
			IndentSize = 1,
			ReadCommentHandling = JsonCommentHandling.Skip,
			AllowTrailingCommas = true,
			TypeInfoResolver = new System.Text.Json.Serialization.Metadata.DefaultJsonTypeInfoResolver(),
		};
		options.Converters.Add(new JsonStringEnumConverter(JsonNamingPolicy.SnakeCaseLower, allowIntegerValues: false));
		options.MakeReadOnly();
		return options;
	}

	/// <summary>Compact options used for hashing.</summary>
	private static readonly JsonSerializerOptions CanonicalOptions = new(Options) { WriteIndented = false };
}
