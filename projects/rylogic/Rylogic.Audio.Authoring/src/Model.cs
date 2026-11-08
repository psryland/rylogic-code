using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Text.Json;
using System.Text.Json.Nodes;
using System.Text.Json.Serialization;

namespace Rylogic.Audio.Authoring;

/// <summary>Where a source recording came from.</summary>
public enum EOriginKind
{
	/// <summary>Downloaded from Freesound. The origin reference is 'freesound:&lt;sound number&gt;'.</summary>
	Freesound,

	/// <summary>A file in one of the project's local folders. The origin reference is 'local:&lt;folder name&gt;/&lt;path relative to the folder&gt;'.</summary>
	Local,

	/// <summary>Made by hand with no separate original. The origin reference is 'hand_made:&lt;name&gt;'.</summary>
	HandMade,
}

/// <summary>The review state of one variant.</summary>
public enum EVariantStatus
{
	Unheard,
	Candidate,
	Accepted,
	Rejected,
}

/// <summary>The kinds of processing step a recipe can contain.</summary>
public enum ERecipeStep
{
	/// <summary>Keep only 'start' to 'end' seconds. A missing 'end' keeps to the end of the sound.</summary>
	Trim,

	/// <summary>Remove 'start' to 'end' seconds and join the remaining parts with a short crossfade.</summary>
	Cut,

	/// <summary>Fade in from silence over 'duration' seconds using 'curve'.</summary>
	FadeIn,

	/// <summary>Fade out to silence over the last 'duration' seconds using 'curve'.</summary>
	FadeOut,

	/// <summary>Scale by 'db' decibels.</summary>
	Gain,

	/// <summary>Measure the integrated loudness, then apply the static gain that makes it 'lufs'.</summary>
	Loudness,

	/// <summary>Remove content below 'frequency' Hz with a Butterworth slope of 'slope' dB per octave (12, 24, 36 or 48).</summary>
	HighPass,

	/// <summary>Remove content above 'frequency' Hz with a Butterworth slope of 'slope' dB per octave (12, 24, 36 or 48).</summary>
	LowPass,

	/// <summary>Boost or cut 'db' decibels around 'frequency' Hz with bandwidth 'q'.</summary>
	Eq,

	/// <summary>Crossfade the last 'duration' seconds into the start, so the result loops seamlessly. The result is 'duration' seconds shorter.</summary>
	Loop,
}

/// <summary>The shape of a fade.</summary>
public enum EFadeCurve
{
	/// <summary>Amplitude changes linearly with time.</summary>
	Linear,

	/// <summary>Amplitude follows a quarter sine, which sounds smoother at the ends.</summary>
	Sine,
}

/// <summary>One step of a processing recipe. Only the parameters of the step's kind are used; see <see cref="ERecipeStep"/>.</summary>
public sealed class RecipeStep
{
	/// <summary>The kind of processing.</summary>
	public ERecipeStep Step { get; set; }

	/// <summary>True to skip this step without removing it.</summary>
	[JsonIgnore(Condition = JsonIgnoreCondition.WhenWritingDefault)]
	public bool Bypass { get; set; }

	/// <summary>Region start, in seconds.</summary>
	public double? Start { get; set; }

	/// <summary>Region end, in seconds.</summary>
	public double? End { get; set; }

	/// <summary>Fade or loop crossfade length, in seconds.</summary>
	public double? Duration { get; set; }

	/// <summary>Fade shape.</summary>
	public EFadeCurve? Curve { get; set; }

	/// <summary>Gain, in decibels.</summary>
	public double? Db { get; set; }

	/// <summary>Target integrated loudness, in LUFS.</summary>
	public double? Lufs { get; set; }

	/// <summary>Filter frequency, in Hz.</summary>
	public double? Frequency { get; set; }

	/// <summary>Filter slope, in dB per octave.</summary>
	public int? Slope { get; set; }

	/// <summary>EQ bandwidth.</summary>
	public double? Q { get; set; }

	/// <summary>A new step of kind 'step' with typical starting parameters.</summary>
	public static RecipeStep CreateDefault(ERecipeStep step)
	{
		// Defaults are audible but gentle, so adding a step gives a sensible starting point to adjust
		return step switch
		{
			ERecipeStep.Trim => new RecipeStep { Step = step, Start = 0 },
			ERecipeStep.Cut => new RecipeStep { Step = step, Start = 0, End = 1 },
			ERecipeStep.FadeIn => new RecipeStep { Step = step, Duration = 0.5, Curve = EFadeCurve.Sine },
			ERecipeStep.FadeOut => new RecipeStep { Step = step, Duration = 0.5, Curve = EFadeCurve.Sine },
			ERecipeStep.Gain => new RecipeStep { Step = step, Db = 0 },
			ERecipeStep.Loudness => new RecipeStep { Step = step, Lufs = -23 },
			ERecipeStep.HighPass => new RecipeStep { Step = step, Frequency = 80, Slope = 12 },
			ERecipeStep.LowPass => new RecipeStep { Step = step, Frequency = 12000, Slope = 12 },
			ERecipeStep.Eq => new RecipeStep { Step = step, Frequency = 1000, Db = 0, Q = 1 },
			ERecipeStep.Loop => new RecipeStep { Step = step, Duration = 3 },
			_ => throw new ArgumentOutOfRangeException(nameof(step), step, "Unknown recipe step."),
		};
	}
}

/// <summary>
/// One candidate recording for a sound ID. The variant ID is stable and never reused within its preparation file, so the recipe stays bound to this
/// variant however the variants are reordered or reviewed.
/// </summary>
public sealed class Variant
{
	/// <summary>Stable variant ID, such as 'v3'.</summary>
	public string Id { get; set; } = string.Empty;

	/// <summary>Origin reference, such as 'freesound:483323'. See <see cref="EOriginKind"/>.</summary>
	public string Origin { get; set; } = string.Empty;

	/// <summary>The imported source ID, or null while the origin has only been browsed.</summary>
	public string? Source { get; set; }

	/// <summary>Review state.</summary>
	public EVariantStatus Status { get; set; }

	/// <summary>Free-text review note.</summary>
	public string? Note { get; set; }

	/// <summary>True when the variant has been listened to all the way through and is final.</summary>
	[JsonIgnore(Condition = JsonIgnoreCondition.WhenWritingDefault)]
	public bool Finalised { get; set; }

	/// <summary>
	/// A previously prepared file, relative to the configuration folder, that is used instead of rendering the recipe. It keeps an existing hand-made
	/// result until a recipe is confirmed to replace it.
	/// </summary>
	public string? ReferenceMaster { get; set; }

	/// <summary>The ordered processing steps.</summary>
	public List<RecipeStep> Recipe { get; set; } = [];
}

/// <summary>The version-controlled preparation file for one sound ID. Variant order is presentation order; the runtime uses the accepted ones in order.</summary>
public sealed class PrepFile
{
	/// <summary>File format version.</summary>
	public int Schema { get; set; } = 1;

	/// <summary>The sound ID.</summary>
	public string Id { get; set; } = string.Empty;

	/// <summary>The number used for the next new variant ID.</summary>
	public int NextVariant { get; set; } = 1;

	/// <summary>All variants, including rejected ones as rejection history.</summary>
	public List<Variant> Variants { get; set; } = [];

	/// <summary>The accepted variants, in runtime order.</summary>
	[JsonIgnore]
	public IEnumerable<Variant> Accepted => Variants.Where(x => x.Status == EVariantStatus.Accepted);

	/// <summary>Add a new variant for 'origin' at the end of the list and return it.</summary>
	public Variant AddVariant(string origin, string? source, EVariantStatus status)
	{
		// Take the next number so a removed variant's ID is never reused
		var variant = new Variant { Id = $"v{NextVariant++}", Origin = origin, Source = source, Status = status };
		Variants.Add(variant);
		return variant;
	}

	/// <summary>Find a variant by ID.</summary>
	public Variant? Find(string variant_id)
	{
		return Variants.FirstOrDefault(x => x.Id == variant_id);
	}

	/// <summary>Read the preparation file for 'sound_id' in 'prep_folder', or an empty one if there is none.</summary>
	public static PrepFile Load(string prep_folder, string sound_id)
	{
		// A sound with no file simply has no variants yet
		var path = PathFor(prep_folder, sound_id);
		if (!File.Exists(path))
			return new PrepFile { Id = sound_id };

		var file = AuthoringJson.Read<PrepFile>(path);
		if (file.Schema != 1 || file.Id != sound_id)
			throw new InvalidDataException($"'{path}' must be a schema 1 preparation file for '{sound_id}'.");

		return file;
	}

	/// <summary>Save this file into 'prep_folder'.</summary>
	public void Save(string prep_folder)
	{
		AuthoringJson.Write(PathFor(prep_folder, Id), this);
	}

	/// <summary>The path of the preparation file for 'sound_id'.</summary>
	public static string PathFor(string prep_folder, string sound_id)
	{
		return Path.Combine(prep_folder, $"{sound_id}.json");
	}
}

/// <summary>The provenance sidecar stored beside each original as '&lt;source-id&gt;.json'.</summary>
public sealed class SourceSidecar
{
	/// <summary>File format version.</summary>
	public int Schema { get; set; } = 1;

	/// <summary>The source ID: the first 16 hex digits of <see cref="DownloadSha256"/>.</summary>
	public string Id { get; set; } = string.Empty;

	/// <summary>The kind of origin.</summary>
	public EOriginKind Kind { get; set; }

	/// <summary>The origin reference, such as 'freesound:483323'.</summary>
	public string Origin { get; set; } = string.Empty;

	public string? Title { get; set; }
	public string? Author { get; set; }
	public string? Licence { get; set; }
	public string? Url { get; set; }

	/// <summary>The file name of the original download or generation.</summary>
	public string? DownloadName { get; set; }

	/// <summary>SHA-256 of the original download bytes.</summary>
	public string DownloadSha256 { get; set; } = string.Empty;

	/// <summary>SHA-256 of the stored audio file, used to prove the original is unchanged.</summary>
	public string OriginalSha256 { get; set; } = string.Empty;
}

/// <summary>What is known about one origin, whether or not it has been imported as a source.</summary>
public sealed class OriginRecord
{
	public string? Title { get; set; }
	public string? User { get; set; }
	public string? Licence { get; set; }
	public string? Url { get; set; }

	/// <summary>A browse-quality preview file name in the previews folder.</summary>
	public string? Preview { get; set; }

	/// <summary>True when the origin was judged not useful for any sound.</summary>
	[JsonIgnore(Condition = JsonIgnoreCondition.WhenWritingDefault)]
	public bool NotUseful { get; set; }

	/// <summary>The source ID once imported.</summary>
	public string? Source { get; set; }
}

/// <summary>The registry of every origin seen by the authoring tool, keyed by origin reference.</summary>
public sealed class OriginsFile
{
	public int Schema { get; set; } = 1;
	public SortedDictionary<string, OriginRecord> Origins { get; set; } = new(StringComparer.Ordinal);

	/// <summary>The kind of an origin reference.</summary>
	public static EOriginKind KindOf(string origin)
	{
		// The prefix before the first colon names the kind
		var colon = origin.IndexOf(':');
		return colon < 0 ? throw new InvalidDataException($"Origin '{origin}' has no kind prefix.") : origin[..colon] switch
		{
			"freesound" => EOriginKind.Freesound,
			"local" => EOriginKind.Local,
			"hand_made" => EOriginKind.HandMade,
			_ => throw new InvalidDataException($"Origin '{origin}' has an unknown kind."),
		};
	}

	/// <summary>The part of an origin reference after the kind prefix.</summary>
	public static string ReferenceOf(string origin)
	{
		return origin[(origin.IndexOf(':') + 1)..];
	}

	/// <summary>Read the registry, or an empty one if the file does not exist.</summary>
	public static OriginsFile Load(string path)
	{
		return File.Exists(path) ? AuthoringJson.Read<OriginsFile>(path) : new OriginsFile();
	}
}

/// <summary>One sound ID in the project's catalog. The project owns the meaning of the properties; the authoring tool only displays them.</summary>
public sealed record CatalogSound(string Id, string Section, string? Description, IReadOnlyDictionary<string, JsonNode?> Properties)
{
	/// <summary>The values a sound's 'load' may have: held decoded in memory, or decoded as it plays.</summary>
	public static readonly IReadOnlyList<string> Loads = ["memory", "stream"];

	/// <summary>True when the project says the sound loops, which enables loop-seam listening.</summary>
	public bool Loop => Properties.TryGetValue("loop", out var loop) && loop is JsonValue value && value.TryGetValue<bool>(out var flag) && flag;

	/// <summary>How the project loads the sound, one of <see cref="Loads"/>, or null when the catalog does not say.</summary>
	public string? Load => Properties.TryGetValue("load", out var load) && load is JsonValue value && value.TryGetValue<string>(out var text) ? text : null;
}

/// <summary>Reads the project's sound catalog: an object with 'sections', each with a 'name' and 'sounds', each sound having an 'id'.</summary>
public static class Catalog
{
	/// <summary>Read the catalog at 'path', in file order.</summary>
	public static IReadOnlyList<CatalogSound> Read(string path)
	{
		// Only the structure is interpreted here; every other property is passed through for display
		var root = JsonNode.Parse(File.ReadAllText(path), documentOptions: new JsonDocumentOptions { CommentHandling = JsonCommentHandling.Skip, AllowTrailingCommas = true }) as JsonObject
			?? throw new InvalidDataException($"'{path}' must be a JSON object.");

		var sounds = new List<CatalogSound>();
		foreach (var section in root["sections"] as JsonArray ?? throw new InvalidDataException($"'{path}' has no 'sections' array."))
		{
			// Each section groups sounds for people
			var section_name = section?["name"]?.GetValue<string>() ?? string.Empty;
			foreach (var sound in section?["sounds"] as JsonArray ?? throw new InvalidDataException($"'{path}' has a section without a 'sounds' array."))
			{
				var obj = sound as JsonObject ?? throw new InvalidDataException($"'{path}' has a sound that is not an object.");
				var id = obj["id"]?.GetValue<string>() ?? throw new InvalidDataException($"'{path}' has a sound without an 'id'.");
				var description = obj["description"]?.GetValue<string>();
				var properties = obj.Where(x => x.Key is not "id" and not "description").ToDictionary(x => x.Key, x => x.Value?.DeepClone());
				sounds.Add(new CatalogSound(id, section_name, description, properties));
			}
		}
		return sounds;
	}

	/// <summary>
	/// Set the property 'property' of sound 'id' in the catalog at 'path' to the scalar 'value'. Only that value changes in the file, so its hand-made
	/// layout is kept; a sound without the property gets it straight after its 'id'. The property must not be 'id', and an existing value must be a scalar.
	/// Throws <see cref="InvalidDataException"/> unless exactly one sound has the ID.
	/// </summary>
	public static void SetValue(string path, string id, string property, JsonValue value)
	{
		// The ID locates the sound, so it cannot be changed this way
		if (property == "id")
			throw new ArgumentException("A sound's 'id' cannot be set.", nameof(property));

		// Work on the raw bytes so token offsets index the file directly. A byte order mark is kept but not given to the reader.
		var bytes = File.ReadAllBytes(path);
		var bom = bytes.AsSpan().StartsWith(Encoding.UTF8.Preamble) ? Encoding.UTF8.Preamble.Length : 0;
		var (start, end, insert) = FindValue(bytes.AsSpan(bom), path, id, property);
		var json = value.ToJsonString();
		var text = Encoding.UTF8.GetBytes(insert ? $", {JsonSerializer.Serialize(property)}: {json}" : json);

		// Splice the new value in place of the old one, or insert it after the ID
		var result = new byte[bytes.Length - (end - start) + text.Length];
		bytes.AsSpan(0, bom + start).CopyTo(result);
		text.CopyTo(result.AsSpan(bom + start));
		bytes.AsSpan(bom + end).CopyTo(result.AsSpan(bom + start + text.Length));
		File.WriteAllBytes(path, result);
	}

	/// <summary>
	/// The byte range of the scalar value of 'property' in the one object whose 'id' is 'id'. When the object has no such property, the range is empty
	/// and placed just after the ID's value, and 'insert' is true.
	/// </summary>
	private static (int start, int end, bool insert) FindValue(ReadOnlySpan<byte> json, string path, string id, string property)
	{
		// Marks a frame whose wanted property holds an object or array
		const int StructuredValue = -2;

		// Walk the tokens keeping one frame per open object or array, recording the property each value belongs to
		var reader = new Utf8JsonReader(json, new JsonReaderOptions { CommentHandling = JsonCommentHandling.Skip, AllowTrailingCommas = true });
		var frames = new Stack<(string? property, bool matches, int id_end, int value_start, int value_end)>();
		var found = new List<(int start, int end, bool insert)>();
		while (reader.Read())
		{
			// The property a value belongs to is consumed by that value
			var owner = frames.Count != 0 ? frames.Peek().property : null;
			if (reader.TokenType != JsonTokenType.PropertyName && frames.Count != 0)
			{
				var frame = frames.Pop();
				frames.Push(frame with { property = null });
			}
			switch (reader.TokenType)
			{
				case JsonTokenType.PropertyName:
				{
					var frame = frames.Pop();
					frames.Push(frame with { property = reader.GetString() });
					break;
				}
				case JsonTokenType.StartObject:
				case JsonTokenType.StartArray:
				{
					// A structured value cannot be replaced by a scalar splice, so its owner is marked to reject it if it is the wanted sound
					if (owner == property)
					{
						var parent = frames.Pop();
						frames.Push(parent with { value_start = StructuredValue });
					}
					frames.Push((null, false, -1, -1, -1));
					break;
				}
				case JsonTokenType.EndObject:
				case JsonTokenType.EndArray:
				{
					// A finished object with the wanted ID is a match
					var frame = frames.Pop();
					if (frame.matches && frame.value_start == StructuredValue)
						throw new InvalidDataException($"'{path}' gives sound '{id}' a '{property}' that is not a single value.");
					if (frame.matches)
						found.Add(frame.value_start >= 0 ? (frame.value_start, frame.value_end, false) : (frame.id_end, frame.id_end, true));

					break;
				}
				case JsonTokenType.String when owner == "id":
				{
					if (reader.GetString() == id)
					{
						var frame = frames.Pop();
						frames.Push(frame with { matches = true, id_end = (int)reader.BytesConsumed });
					}
					break;
				}
				case JsonTokenType.String:
				case JsonTokenType.Number:
				case JsonTokenType.True:
				case JsonTokenType.False:
				case JsonTokenType.Null:
				{
					if (owner == property)
					{
						var frame = frames.Pop();
						frames.Push(frame with { value_start = (int)reader.TokenStartIndex, value_end = (int)reader.BytesConsumed });
					}
					break;
				}
			}
		}
		return found.Count == 1 ? found[0] : throw new InvalidDataException($"'{path}' defines sound '{id}' {found.Count} times, but must define it once.");
	}
}