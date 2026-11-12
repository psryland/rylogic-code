using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text;
using System.Text.Encodings.Web;
using System.Text.Json;
using System.Text.Json.Nodes;
using System.Text.Json.Serialization;
using System.Text.RegularExpressions;

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

	/// <summary>
	/// An external editor's change to another recording, kept as a new original. The origin reference is 'edited:&lt;source id&gt;', and its record names the
	/// recording it was made from (see <see cref="OriginRecord.EditOf"/>), whose credit it keeps.
	/// </summary>
	Edited,

	/// <summary>
	/// A file on disk added through the tool, with the credit given when it was added. The file is kept as an original because it exists nowhere else
	/// the tool can find. The origin reference is 'imported:&lt;source id&gt;'.
	/// </summary>
	Imported,
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

	/// <summary>
	/// Scale the audio by 'db' decibels and add each of 'inputs', which are other originals. The result is as long as the shortest part. A mono part
	/// is heard on both channels when another part is stereo.
	/// </summary>
	Mix,
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

	/// <summary>The other originals added by a mix.</summary>
	public List<MixInput>? Inputs { get; set; }

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
			ERecipeStep.Mix => new RecipeStep { Step = step, Db = 0, Inputs = [] },
			_ => throw new ArgumentOutOfRangeException(nameof(step), step, "Unknown recipe step."),
		};
	}
}

/// <summary>One original added by a mix step.</summary>
public sealed class MixInput
{
	/// <summary>Origin reference of the added recording, such as 'freesound:483323'. It is credited wherever the sound is used.</summary>
	public string Origin { get; set; } = string.Empty;

	/// <summary>The source ID once the origin's original is imported. A mix cannot render until every input has one.</summary>
	public string? Source { get; set; }

	/// <summary>Where in the added recording the mix starts, in seconds.</summary>
	public double Start { get; set; }

	/// <summary>The level of the added recording, in decibels.</summary>
	public double Db { get; set; }
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

	/// <summary>The originals added by the recipe's mix steps, including bypassed ones, because turning the bypass off uses them again.</summary>
	[JsonIgnore]
	public IEnumerable<MixInput> MixInputs => Recipe.SelectMany(x => x.Inputs ?? []);

	/// <summary>True if the variant plays 'origin', as its own recording or mixed in by its recipe.</summary>
	public bool Uses(string origin)
	{
		return Origin == origin || MixInputs.Any(x => x.Origin == origin);
	}
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

	/// <summary>For an edited recording, the origin of the recording it was made from.</summary>
	public string? EditOf { get; set; }

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

	/// <summary>
	/// For an edited recording, the origin of the first recording in its chain of edits. The edit is credited like that recording, and that recording
	/// counts as in use while the edit is.
	/// </summary>
	public string? EditOf { get; set; }

	/// <summary>The source ID once imported.</summary>
	public string? Source { get; set; }
}

/// <summary>The credit given to a recording added from a file. Only the title is required.</summary>
public sealed record FileCredit
{
	public required string Title { get; init; }
	public string? Author { get; init; }
	public string? Licence { get; init; }
	public string? Url { get; init; }
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
			"edited" => EOriginKind.Edited,
			"imported" => EOriginKind.Imported,
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

/// <summary>Whether a sound surrounds the listener or comes from a point in the world.</summary>
public enum ESoundPlacement
{
	/// <summary>Surrounds the listener. A bed has no position, so it has no range.</summary>
	Bed,

	/// <summary>Comes from a point in the world and fades out by its range. A positioned sound is mono, because its direction comes from its position.</summary>
	Positioned,
}

/// <summary>The channel layout of a sound's audio.</summary>
public enum ESoundChannels
{
	Mono,
	Stereo,
}

/// <summary>How the game loads a sound's audio.</summary>
public enum ESoundLoad
{
	/// <summary>Held decoded in memory, which suits short sounds that play often.</summary>
	Memory,

	/// <summary>Decoded as it plays, which suits anything long.</summary>
	Stream,
}

/// <summary>The shortest and longest length, in seconds, wanted for a sound's prepared audio.</summary>
public readonly record struct LengthRange(double Min, double Max);

/// <summary>One sound ID in the catalog. <see cref="Catalog"/> describes each value and the rules a sound must meet.</summary>
public sealed record CatalogSound
{
	/// <summary>The sound ID, a lower-case dotted name such as 'wind.plains.still'.</summary>
	public required string Id { get; init; }

	/// <summary>The name of the section that groups the sound for people.</summary>
	public required string Section { get; init; }

	/// <summary>Whether the sound surrounds the listener or comes from a point.</summary>
	public required ESoundPlacement Placement { get; init; }

	/// <summary>True when the sound repeats until stopped.</summary>
	public required bool Loop { get; init; }

	/// <summary>The channel layout of the sound's audio.</summary>
	public required ESoundChannels Channels { get; init; }

	/// <summary>The wanted length of the prepared audio, or null for no preference.</summary>
	public LengthRange? LengthS { get; init; }

	/// <summary>How the game loads the sound's audio.</summary>
	public required ESoundLoad Load { get; init; }

	/// <summary>The distance in metres at which a positioned sound becomes silent. Null for a bed.</summary>
	public double? RangeM { get; init; }

	/// <summary>The mixer group that the sound plays through, a lower-case dotted name.</summary>
	public required string Group { get; init; }

	/// <summary>The linear gain applied to the sound's audio.</summary>
	public required double Gain { get; init; }

	/// <summary>When the sound is active, as the JSON object written in the catalog. The authoring tool keeps it but does not interpret it.</summary>
	public JsonElement? Active { get; init; }

	/// <summary>A description of the sound for people.</summary>
	public string? Description { get; init; }
}

/// <summary>
/// The project's sound catalog: a JSON object with a 'schema' of 1 and 'sections', each of which has a 'name' and the 'sounds' it groups. The authoring
/// tool owns the file and writes it in one standard layout: tab indents, CRLF line ends, and each sound on one line with its properties in the order
/// below. Tools may rely on that layout, for example to change one value on a sound's line. A sound has these properties:
/// - 'id': a lower-case dotted name of letters, digits and underscores, starting with a letter, at most 128 characters, and unique in the catalog.
/// - 'placement': 'bed' or 'positioned'. 'channels': 'mono' or 'stereo'. A positioned sound must be mono.
/// - 'loop': true when the sound repeats. 'load': 'memory' or 'stream'.
/// - 'length_s': optional, '[min, max]' seconds wanted for the prepared audio.
/// - 'range_m': required for a positioned sound and positive; not allowed for a bed.
/// - 'group': a dotted name like an ID, at most 64 characters. 'gain': a finite number of at least 0.
/// - 'active': optional; a JSON object kept as it is. 'description': optional text for people.
/// Comments and trailing commas are accepted when reading but are not kept. Sections are written in the order of their first sound, so a section
/// with no sounds is not kept.
/// </summary>
public static partial class Catalog
{
	/// <summary>The properties a sound may have, in the order they are written.</summary>
	public static IReadOnlyList<string> Properties { get; } = ["id", "placement", "loop", "channels", "length_s", "load", "range_m", "group", "gain", "active", "description"];

	/// <summary>Read the catalog at 'path', in file order. Throws <see cref="InvalidDataException"/> when the catalog breaks the rules.</summary>
	public static IReadOnlyList<CatalogSound> Read(string path)
	{
		// Hand edits may include comments and trailing commas
		using var document = JsonDocument.Parse(File.ReadAllText(path), new JsonDocumentOptions { CommentHandling = JsonCommentHandling.Skip, AllowTrailingCommas = true });
		var root = document.RootElement;
		if (root.ValueKind != JsonValueKind.Object)
			throw new InvalidDataException($"'{path}' must be a JSON object.");

		// Only known properties are allowed, because the file is rewritten from what is read and anything else would be lost
		CheckProperties(root, ["schema", "sections"], $"'{path}'");
		if (!root.TryGetProperty("schema", out var schema) || schema.ValueKind != JsonValueKind.Number || !schema.TryGetInt32(out var version) || version != 1)
			throw new InvalidDataException($"'{path}' must have a 'schema' of 1.");

		// Each section groups its sounds under a unique name
		var sounds = new List<CatalogSound>();
		var names = new HashSet<string>(StringComparer.Ordinal);
		foreach (var section in ArrayOf(root, "sections", $"'{path}'"))
		{
			if (section.ValueKind != JsonValueKind.Object)
				throw new InvalidDataException($"'{path}' has a section that is not an object.");

			CheckProperties(section, ["name", "sounds"], $"A section in '{path}'");
			var name = StringOf(section, "name", $"A section in '{path}'");
			if (!names.Add(name))
				throw new InvalidDataException($"'{path}' has more than one section named '{name}'.");

			foreach (var sound in ArrayOf(section, "sounds", $"Section '{name}' in '{path}'"))
				sounds.Add(ParseSound(sound, name, $"'{path}'"));
		}

		// The rules that involve more than one value are checked once everything is read
		try
		{
			Check(sounds);
		}
		catch (InvalidDataException ex)
		{
			throw new InvalidDataException($"'{path}': {ex.Message}", ex);
		}
		return sounds;
	}

	/// <summary>
	/// Read one sound from 'sound', a JSON object of catalog properties, placing it in section 'section'. 'where' names the source for error messages.
	/// Only the type of each value is checked; see <see cref="Check"/> for the rules. Throws <see cref="InvalidDataException"/> for a missing, unknown
	/// or mistyped property.
	/// </summary>
	public static CatalogSound ParseSound(JsonElement sound, string section, string where)
	{
		// The ID comes first so that later errors can name the sound
		if (sound.ValueKind != JsonValueKind.Object)
			throw new InvalidDataException($"{where} has a sound that is not an object.");

		var id = StringOf(sound, "id", $"A sound in {where}");
		var owner = $"Sound '{id}' in {where}";
		CheckProperties(sound, Properties, owner);

		// Optional values are null when absent
		return new CatalogSound
		{
			Id = id,
			Section = section,
			Placement = EnumOf<ESoundPlacement>(sound, "placement", owner),
			Loop = sound.TryGetProperty("loop", out var loop) && loop.ValueKind is JsonValueKind.True or JsonValueKind.False ? loop.GetBoolean() : throw new InvalidDataException($"{owner} must have a boolean 'loop'."),
			Channels = EnumOf<ESoundChannels>(sound, "channels", owner),
			LengthS = sound.TryGetProperty("length_s", out var length) ? LengthOf(length, owner) : null,
			Load = EnumOf<ESoundLoad>(sound, "load", owner),
			RangeM = sound.TryGetProperty("range_m", out _) ? NumberOf(sound, "range_m", owner) : null,
			Group = StringOf(sound, "group", owner),
			Gain = NumberOf(sound, "gain", owner),
			Active = sound.TryGetProperty("active", out var active) ? active.ValueKind == JsonValueKind.Object ? active.Clone() : throw new InvalidDataException($"{owner} has an 'active' that is not an object.") : null,
			Description = sound.TryGetProperty("description", out _) ? StringOf(sound, "description", owner) : null,
		};
	}

	/// <summary>
	/// Check that every sound in 'sounds' meets the catalog's rules and that no two share an ID. Throws <see cref="InvalidDataException"/> naming the
	/// first sound that does not.
	/// </summary>
	public static void Check(IEnumerable<CatalogSound> sounds)
	{
		// Checked in order, so the first problem in the file is the one reported
		var ids = new HashSet<string>(StringComparer.Ordinal);
		foreach (var sound in sounds)
		{
			// The ID names the sound in later messages, so it is checked first
			var owner = $"Sound '{sound.Id}'";
			if (sound.Id.Length > 128 || !NamePattern().IsMatch(sound.Id))
				throw new InvalidDataException($"{owner} has an invalid ID; IDs are lower-case dotted names of letters, digits and underscores that start with a letter.");
			if (!ids.Add(sound.Id))
				throw new InvalidDataException($"{owner} is defined more than once.");
			if (string.IsNullOrWhiteSpace(sound.Section))
				throw new InvalidDataException($"{owner} needs a section name.");

			// A positioned sound's direction comes from its position, so it is mono and fades out by its range; a bed has no range
			switch (sound.Placement)
			{
				case ESoundPlacement.Bed:
				{
					if (sound.RangeM != null)
						throw new InvalidDataException($"{owner} is a bed and must not have a range.");

					break;
				}
				case ESoundPlacement.Positioned:
				{
					if (sound.Channels != ESoundChannels.Mono)
						throw new InvalidDataException($"{owner} is positioned and must be mono.");
					if (sound.RangeM is not { } range || !double.IsFinite(range) || range <= 0)
						throw new InvalidDataException($"{owner} is positioned and needs a finite, positive range.");

					break;
				}
				default:
				{
					throw new ArgumentOutOfRangeException(nameof(sounds), sound.Placement, "Unknown placement.");
				}
			}

			// The remaining values are independent of each other
			if (sound.Group.Length > 64 || !NamePattern().IsMatch(sound.Group))
				throw new InvalidDataException($"{owner} has an invalid group '{sound.Group}'; groups are dotted names like IDs.");
			if (!double.IsFinite(sound.Gain) || sound.Gain < 0)
				throw new InvalidDataException($"{owner} needs a finite gain of at least 0.");
			if (sound.LengthS is { } length && !(double.IsFinite(length.Min) && double.IsFinite(length.Max) && 0 <= length.Min && length.Min <= length.Max))
				throw new InvalidDataException($"{owner} has a length range that is not 0 <= min <= max.");
		}
	}

	/// <summary>
	/// Return 'sounds' with 'sound' added as the last sound of its section, or as the only sound of a new last section. Throws
	/// <see cref="InvalidDataException"/> when the result breaks the catalog's rules, for example when the ID is already used.
	/// </summary>
	public static IReadOnlyList<CatalogSound> Add(IReadOnlyList<CatalogSound> sounds, CatalogSound sound)
	{
		// Sections stay contiguous, so a new sound goes straight after its section's last sound
		var result = sounds.ToList();
		var last = result.FindLastIndex(x => x.Section == sound.Section);
		result.Insert(last >= 0 ? last + 1 : result.Count, sound);
		Check(result);
		return result;
	}

	/// <summary>
	/// Return 'sounds' with the sound 'id' replaced by 'sound', which may have a new ID or section. A sound that stays in its section keeps its place;
	/// one that moves becomes the last sound of its new section. Throws <see cref="InvalidDataException"/> when 'id' is not in the catalog or the
	/// result breaks the catalog's rules.
	/// </summary>
	public static IReadOnlyList<CatalogSound> Replace(IReadOnlyList<CatalogSound> sounds, string id, CatalogSound sound)
	{
		// Replacing in place keeps the order people chose within a section
		var index = IndexOf(sounds, id);
		var result = sounds.ToList();
		if (sounds[index].Section == sound.Section)
		{
			result[index] = sound;
			Check(result);
			return result;
		}

		// A sound moving section is added to its new one like a new sound
		result.RemoveAt(index);
		return Add(result, sound);
	}

	/// <summary>Return 'sounds' without the sound 'id'. Throws <see cref="InvalidDataException"/> when 'id' is not in the catalog.</summary>
	public static IReadOnlyList<CatalogSound> Remove(IReadOnlyList<CatalogSound> sounds, string id)
	{
		// An emptied section disappears, because sections exist only through their sounds
		var result = sounds.ToList();
		result.RemoveAt(IndexOf(sounds, id));
		return result;
	}

	/// <summary>Check 'sounds' and write them to 'path' in the standard layout. The file is replaced atomically.</summary>
	public static void Write(string path, IReadOnlyList<CatalogSound> sounds)
	{
		// A catalog that breaks the rules is never written, because the next read would fail
		Check(sounds);
		AuthoringJson.WriteAtomic(path, Encoding.UTF8.GetBytes(Format(sounds)));
	}

	/// <summary>The text of a catalog holding 'sounds', in the standard layout.</summary>
	public static string Format(IReadOnlyList<CatalogSound> sounds)
	{
		// Sections are written in the order of their first sound
		var text = new StringBuilder();
		text.Append("{\r\n\t\"schema\": 1,\r\n\t\"sections\": [\r\n");
		var sections = sounds.GroupBy(x => x.Section).ToList();
		for (int i = 0; i != sections.Count; ++i)
		{
			// One line per sound, separated by commas
			text.Append($"\t\t{{\r\n\t\t\t\"name\": {StringJson(sections[i].Key)},\r\n\t\t\t\"sounds\": [\r\n");
			var members = sections[i].ToList();
			for (int j = 0; j != members.Count; ++j)
				text.Append($"\t\t\t\t{FormatSound(members[j])}{(j != members.Count - 1 ? "," : "")}\r\n");

			text.Append($"\t\t\t]\r\n\t\t}}{(i != sections.Count - 1 ? "," : "")}\r\n");
		}
		text.Append("\t]\r\n}\r\n");
		return text.ToString();
	}

	/// <summary>
	/// The one-line JSON object of 'sound', with its properties in the standard order. Numbers use the fewest digits that keep them readable, and a gain
	/// always shows a decimal point, so '1.0' stays '1.0'.
	/// </summary>
	public static string FormatSound(CatalogSound sound)
	{
		// Optional properties are left out when they have no value
		var parts = new List<string>
		{
			Property("id", StringJson(sound.Id)),
			Property("placement", StringJson(NameOf(sound.Placement))),
			Property("loop", sound.Loop ? "true" : "false"),
			Property("channels", StringJson(NameOf(sound.Channels))),
		};
		if (sound.LengthS is { } length)
			parts.Add(Property("length_s", $"[{NumberJson(length.Min, "0.###")}, {NumberJson(length.Max, "0.###")}]"));

		parts.Add(Property("load", StringJson(NameOf(sound.Load))));
		if (sound.RangeM is { } range)
			parts.Add(Property("range_m", NumberJson(range, "0.##")));

		parts.Add(Property("group", StringJson(sound.Group)));
		parts.Add(Property("gain", NumberJson(sound.Gain, "0.0##")));
		if (sound.Active is { } active)
			parts.Add(Property("active", ElementJson(active)));
		if (sound.Description is { } description)
			parts.Add(Property("description", StringJson(description)));

		return $"{{{string.Join(", ", parts)}}}";

		static string Property(string name, string value)
		{
			return $"{StringJson(name)}: {value}";
		}
	}

	/// <summary>The lower-case catalog name of an enum value, such as 'bed' for <see cref="ESoundPlacement.Bed"/>.</summary>
	public static string NameOf<T>(T value) where T : struct, Enum
	{
		return value.ToString().ToLowerInvariant();
	}

	/// <summary>The index of the sound 'id' in 'sounds'.</summary>
	private static int IndexOf(IReadOnlyList<CatalogSound> sounds, string id)
	{
		var index = sounds.ToList().FindIndex(x => x.Id == id);
		return index >= 0 ? index : throw new InvalidDataException($"'{id}' is not a sound ID in the catalog.");
	}

	/// <summary>Throw unless every property of 'element' is one of 'allowed'. Unknown properties are rejected so that a misspelt one is not lost.</summary>
	private static void CheckProperties(JsonElement element, IReadOnlyList<string> allowed, string owner)
	{
		foreach (var property in element.EnumerateObject())
		{
			if (!allowed.Contains(property.Name))
				throw new InvalidDataException($"{owner} has unknown property '{property.Name}'.");
		}
	}

	/// <summary>The required array property 'name' of 'element'.</summary>
	private static JsonElement.ArrayEnumerator ArrayOf(JsonElement element, string name, string owner)
	{
		return element.TryGetProperty(name, out var value) && value.ValueKind == JsonValueKind.Array ? value.EnumerateArray() : throw new InvalidDataException($"{owner} must have a '{name}' array.");
	}

	/// <summary>The required, non-empty string property 'name' of 'element'.</summary>
	private static string StringOf(JsonElement element, string name, string owner)
	{
		return element.TryGetProperty(name, out var value) && value.ValueKind == JsonValueKind.String && value.GetString() is { Length: > 0 } text ? text : throw new InvalidDataException($"{owner} must have a non-empty string '{name}'.");
	}

	/// <summary>The required number property 'name' of 'element'.</summary>
	private static double NumberOf(JsonElement element, string name, string owner)
	{
		return element.TryGetProperty(name, out var value) && value.ValueKind == JsonValueKind.Number ? value.GetDouble() : throw new InvalidDataException($"{owner} must have a number '{name}'.");
	}

	/// <summary>The required enum property 'name' of 'element', written as its lower-case name.</summary>
	private static T EnumOf<T>(JsonElement element, string name, string owner) where T : struct, Enum
	{
		var text = StringOf(element, name, owner);
		var names = Enum.GetValues<T>().Select(NameOf).ToList();
		var index = names.IndexOf(text);
		return index >= 0 ? Enum.GetValues<T>()[index] : throw new InvalidDataException($"{owner} has '{name}' '{text}', but it must be one of: {string.Join(", ", names)}.");
	}

	/// <summary>A '[min, max]' length range.</summary>
	private static LengthRange LengthOf(JsonElement value, string owner)
	{
		if (value.ValueKind != JsonValueKind.Array || value.GetArrayLength() != 2 || value.EnumerateArray().Any(x => x.ValueKind != JsonValueKind.Number))
			throw new InvalidDataException($"{owner} must have a 'length_s' of two numbers, '[min, max]'.");

		return new LengthRange(value[0].GetDouble(), value[1].GetDouble());
	}

	/// <summary>'value' as JSON text on one line, with ', ' and ': ' separators.</summary>
	private static string ElementJson(JsonElement value)
	{
		// Containers are laid out here so that a value written over several lines still fits on its sound's line
		switch (value.ValueKind)
		{
			case JsonValueKind.Object:
			{
				return $"{{{string.Join(", ", value.EnumerateObject().Select(x => $"{StringJson(x.Name)}: {ElementJson(x.Value)}"))}}}";
			}
			case JsonValueKind.Array:
			{
				return $"[{string.Join(", ", value.EnumerateArray().Select(ElementJson))}]";
			}
			case JsonValueKind.String:
			{
				return StringJson(value.GetString()!);
			}
			case JsonValueKind.Number:
			case JsonValueKind.True:
			case JsonValueKind.False:
			case JsonValueKind.Null:
			{
				return value.GetRawText();
			}
			default:
			{
				throw new ArgumentOutOfRangeException(nameof(value), value.ValueKind, "Unexpected JSON value.");
			}
		}
	}

	/// <summary>'text' as a JSON string, escaping only what JSON requires so that punctuation stays readable.</summary>
	private static string StringJson(string text)
	{
		return JsonSerializer.Serialize(text, m_string_options);
	}

	/// <summary>'value' as a JSON number using the .NET number 'format'.</summary>
	private static string NumberJson(double value, string format)
	{
		return value.ToString(format, CultureInfo.InvariantCulture);
	}

	/// <summary>Options that escape only the characters JSON requires.</summary>
	private static readonly JsonSerializerOptions m_string_options = new() { Encoder = JavaScriptEncoder.UnsafeRelaxedJsonEscaping };

	/// <summary>A lower-case dotted name, as used by sound IDs and groups.</summary>
	[GeneratedRegex(@"^[a-z][a-z0-9_]*(?:\.[a-z0-9_]+)*$", RegexOptions.CultureInvariant)]
	private static partial Regex NamePattern();
}
