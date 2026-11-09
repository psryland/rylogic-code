using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;

namespace Rylogic.Audio.Authoring;

/// <summary>
/// Finds the prepared audio file of every accepted variant, in runtime order. Cookers and live previews use this so that every consumer plays exactly
/// what the authoring tool previews as the game's audio.
/// </summary>
public static class PreparedSounds
{
	/// <summary>
	/// Throw <see cref="InvalidDataException"/> if the preparation folder has a file for a sound that is not in 'defined_ids'.
	/// Such a file could never be played, so it is reported rather than ignored.
	/// </summary>
	public static void CheckDefined(AuthoringConfig config, IReadOnlyCollection<string> defined_ids)
	{
		// Every preparation file is named after its sound
		if (!Directory.Exists(config.PrepFolder))
			return;

		var known = defined_ids.ToHashSet(StringComparer.Ordinal);
		foreach (var path in Directory.EnumerateFiles(config.PrepFolder, "*.json"))
		{
			var sound_id = Path.GetFileNameWithoutExtension(path);
			if (!known.Contains(sound_id))
				throw new InvalidDataException($"Preparation file '{path}' is for sound '{sound_id}', which is not defined.");
		}
	}

	/// <summary>
	/// Return the prepared files of each sound in 'sound_ids' that has accepted variants, in runtime order. A sound with no preparation file or no
	/// accepted variant is absent, because it is intentionally unassigned. A variant uses its reference master if it has one, otherwise the cached render
	/// of its recipe from its original, rendering it if needed. A missing original is fetched again through 'fetcher'. Throws <see cref="InvalidDataException"/> for an
	/// accepted variant with nothing to prepare from or an original that cannot be fetched as recorded, and <see cref="FileNotFoundException"/> for a referenced file that is missing.
	/// Preparation files of other sounds are ignored; use <see cref="CheckDefined"/> to report files of undefined sounds.
	/// </summary>
	public static async Task<IReadOnlyDictionary<string, IReadOnlyList<string>>> ResolveAsync(AuthoringConfig config, RenderCache cache, OriginalFetcher fetcher, IReadOnlyCollection<string> sound_ids, CancellationToken cancel)
	{
		// The origins registry is read lazily, only when an original must be fetched
		OriginsFile? origins = null;
		var result = new Dictionary<string, IReadOnlyList<string>>(StringComparer.Ordinal);
		foreach (var sound_id in sound_ids)
		{
			// Each accepted variant becomes one prepared file, in the order the file lists them
			var prep = PrepFile.Load(config.PrepFolder, sound_id);
			var files = new List<string>();
			foreach (var variant in prep.Accepted)
			{
				// A reference master stands in for the recipe until the recipe is first edited
				if (variant.ReferenceMaster != null)
				{
					var reference = Path.GetFullPath(Path.Combine(config.Root, variant.ReferenceMaster));
					if (!File.Exists(reference))
						throw new FileNotFoundException($"Sound '{sound_id}' variant '{variant.Id}' uses reference master '{variant.ReferenceMaster}', which is missing.", reference);

					files.Add(reference);
					continue;
				}

				// Otherwise the recipe is rendered from the original, which is fetched again if it is missing; a changed original fails inside the render cache
				if (variant.Source == null)
					throw new InvalidDataException($"Sound '{sound_id}' variant '{variant.Id}' is accepted but its original has not been imported.");

				if (!fetcher.Sources.Contains(variant.Source))
				{
					// The origins registry says where the original came from, and is read only when something must be fetched
					origins ??= OriginsFile.Load(config.OriginsPath);
					var record = origins.Origins.GetValueOrDefault(variant.Origin) ?? throw new InvalidDataException($"Sound '{sound_id}' variant '{variant.Id}' has origin '{variant.Origin}', which is not in '{config.OriginsPath}'.");
					await fetcher.EnsureAsync(variant.Origin, record, variant.Source, cancel);
				}

				var (_, render_path) = await cache.GetOrRenderAsync(variant.Source, variant.Recipe, cancel);
				files.Add(render_path);
			}
			if (files.Count != 0)
				result.Add(sound_id, files);
		}
		return result;
	}
}
