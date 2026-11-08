using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Net.Http;
using System.Text.Json;
using System.Text.Json.Nodes;
using System.Text.RegularExpressions;
using System.Threading;
using System.Threading.Tasks;

namespace Rylogic.Audio.Authoring;

/// <summary>
/// One open authoring project: the catalog, the origins registry and the preparation files, plus the background jobs that import originals and render
/// recipes. Every method is thread-safe. Reading the state and applying review actions only take a short in-memory lock, so they never wait for
/// downloads, imports or renders; those run as <see cref="LatestJobs"/> and publish their results when they finish.
/// </summary>
public sealed partial class Workspace
{
	/// <summary>The latest render of one variant. A failed render keeps the key of the last successful one, so the last working audio stays playable.</summary>
	private sealed class RenderState
	{
		public string? Key;
		public string? Error;
	}

	// Lock order: a job's result is published while LatestJobs holds its own lock, and the publisher then takes 'm_lock'. Code holding 'm_lock' must
	// therefore never call into 'Jobs' directly; it queues jobs on the thread pool instead (see QueueJob).
	private readonly object m_lock = new();
	private readonly Dictionary<string, PrepFile> m_preps = new(StringComparer.Ordinal);
	private readonly Dictionary<string, RenderState> m_renders = new(StringComparer.Ordinal);
	private readonly Dictionary<string, string> m_import_errors = new(StringComparer.Ordinal);
	private readonly OriginsFile m_origins;
	private IReadOnlyList<CatalogSound> m_catalog;

	/// <summary>Open the project described by 'config'.</summary>
	public Workspace(AuthoringConfig config, Ffmpeg ffmpeg, HttpClient http)
	{
		// The catalog and origins are read now; preparation files are read when their sound is first used
		Config = config;
		Ffmpeg = ffmpeg;
		Sources = new SourceStore(config.OriginalsFolder, config.FetchedFolder, ffmpeg);
		Cache = new RenderCache(config.CacheFolder, Sources, ffmpeg);
		Freesound = new FreesoundClient(http, config.FreesoundCredentialsPath);
		Fetcher = new OriginalFetcher(config, Sources, Freesound);
		Jobs = new LatestJobs(Math.Max(1, Environment.ProcessorCount / 2));
		m_catalog = Catalog.Read(config.CatalogPath);
		m_origins = OriginsFile.Load(config.OriginsPath);
	}

	/// <summary>The project configuration.</summary>
	public AuthoringConfig Config { get; }

	/// <summary>The decoder and converter used for originals.</summary>
	public Ffmpeg Ffmpeg { get; }

	/// <summary>The originals store.</summary>
	public SourceStore Sources { get; }

	/// <summary>The render cache.</summary>
	public RenderCache Cache { get; }

	/// <summary>Freesound browsing and downloads.</summary>
	public FreesoundClient Freesound { get; }

	/// <summary>Fetches originals that are not stored yet.</summary>
	public OriginalFetcher Fetcher { get; }

	/// <summary>The background jobs. Slots are 'import:&lt;origin&gt;' and 'render:&lt;sound-id&gt;/&lt;variant-id&gt;'.</summary>
	public LatestJobs Jobs { get; }

	/// <summary>The file types accepted from local folders.</summary>
	public static IReadOnlyList<string> LocalTypes { get; } = [".wav", ".mp3", ".flac", ".ogg"];

	/// <summary>The review actions that change what the project's game plays for the action's sound 'id'. Other actions only change the review state.</summary>
	public static IReadOnlySet<string> GameActions { get; } = new HashSet<string>(["loop", "load", "status", "move", "recipe", "adopt"], StringComparer.Ordinal);

	/// <summary>The current state for display. Never waits for background work.</summary>
	public JsonObject Snapshot()
	{
		// Read the jobs first: taking the jobs lock while holding 'm_lock' would invert the lock order
		var pending = Jobs.Pending;
		var freesound = Freesound.Status;
		lock (m_lock)
		{
			// The sound IDs in catalog order, with the properties the project wants shown
			var ids = new JsonArray();
			foreach (var sound in m_catalog)
			{
				var properties = new JsonObject();
				foreach (var (name, value) in sound.Properties)
					properties[name] = value?.DeepClone();

				ids.Add(new JsonObject { ["id"] = sound.Id, ["section"] = sound.Section, ["description"] = sound.Description, ["loop"] = sound.Loop, ["load"] = sound.Load, ["properties"] = properties });
			}

			// Every known origin, with the address that plays it best: the original once imported, else the local file or the browse preview
			var origins = new JsonObject();
			foreach (var (origin, record) in m_origins.Origins)
			{
				var node = JsonSerializer.SerializeToNode(record, AuthoringJson.Options)!.AsObject();
				node["play"] = PlayUrl(origin, record);
				node["kind"] = JsonSerializer.SerializeToNode(OriginsFile.KindOf(origin), AuthoringJson.Options);
				node["import_pending"] = pending.ContainsKey(ImportSlot(origin));
				node["import_error"] = m_import_errors.GetValueOrDefault(origin);
				origins[origin] = node;
			}

			// The variants of each sound, each with the state of its latest render
			var preps = new JsonObject();
			foreach (var sound in m_catalog)
			{
				var variants = new JsonArray();
				foreach (var variant in PrepOf(sound.Id).Variants)
				{
					var node = JsonSerializer.SerializeToNode(variant, AuthoringJson.Options)!.AsObject();
					var slot = RenderSlot(sound.Id, variant.Id);
					var render = m_renders.GetValueOrDefault(slot);
					node["render_url"] = render?.Key != null ? $"/render/{render.Key}.wav" : null;
					node["render_error"] = render?.Error;
					node["render_pending"] = pending.ContainsKey(slot);
					node["reference_url"] = variant.ReferenceMaster != null ? $"/reference/{string.Join('/', variant.ReferenceMaster.Split('/').Select(Uri.EscapeDataString))}" : null;
					variants.Add(node);
				}
				preps[sound.Id] = variants;
			}

			return new JsonObject
			{
				["ids"] = ids,
				["origins"] = origins,
				["preps"] = preps,
				["pending"] = new JsonArray(pending.Values.Order(StringComparer.Ordinal).Select(x => (JsonNode)x).ToArray()),
				["freesound"] = new JsonObject { ["configured"] = freesound.Configured, ["connected"] = freesound.Connected },
				["apply"] = Config.Apply?.Display,
				["send"] = Config.Send?.Display,
			};
		}
	}

	/// <summary>
	/// Re-read the catalog and add local files that are new to the origins registry, crediting each to its folder and assigning it to the sound ID its
	/// path names. File-system reads happen before the lock is taken.
	/// </summary>
	public void Refresh()
	{
		// Read everything that touches the disk first
		var catalog = Catalog.Read(Config.CatalogPath);
		var local = new List<(LocalFolder folder, string rel)>();
		foreach (var folder in Config.LocalFolders.Where(x => Directory.Exists(x.Path)))
		{
			local.AddRange(Directory.EnumerateFiles(folder.Path, "*", SearchOption.AllDirectories)
				.Where(x => LocalTypes.Contains(Path.GetExtension(x).ToLowerInvariant()))
				.Select(x => (folder, Path.GetRelativePath(folder.Path, x).Replace('\\', '/')))
				.OrderBy(x => x.Item2, StringComparer.Ordinal));
		}

		lock (m_lock)
		{
			// The catalog may have been edited while the tool runs
			m_catalog = catalog;

			// A local file is identified by its path, so moving it makes a new origin
			var changed = false;
			foreach (var (folder, rel) in local)
			{
				var origin = $"local:{folder.Name}/{rel}";
				if (m_origins.Origins.ContainsKey(origin))
					continue;

				m_origins.Origins[origin] = new OriginRecord { Title = Path.ChangeExtension(rel, null), User = folder.Author, Licence = folder.Licence, Url = folder.Url };
				var guess = GuessSoundId(rel, m_catalog.Select(x => x.Id));
				if (guess != null)
				{
					var prep = PrepOf(guess);
					prep.AddVariant(origin, null, EVariantStatus.Unheard);
					prep.Save(Config.PrepFolder);
				}
				changed = true;
			}
			if (changed)
				SaveOrigins();
		}
	}

	/// <summary>
	/// Apply one review action and return a message for the user, or null. 'action' names the action; the other properties are its arguments.
	/// Throws <see cref="InvalidOperationException"/> for an action that cannot be applied.
	/// </summary>
	public string? Apply(JsonObject action)
	{
		var name = Text(action, "action");
		lock (m_lock)
		{
			switch (name)
			{
				case "useful":
				{
					// Marking an origin not useful only affects how the unassigned pool is shown
					var record = OriginRecordOf(Text(action, "origin"));
					record.NotUseful = !(action["useful"]?.GetValue<bool>() ?? true);
					SaveOrigins();
					return null;
				}
				case "assign":
				{
					return Assign(Text(action, "id"), Text(action, "origin"));
				}
				case "loop":
				{
					// The catalog belongs to the project, so only its 'loop' value is changed and the catalog is read back
					var loop = action["loop"]?.GetValue<bool>() ?? throw new InvalidOperationException("The 'loop' action needs a 'loop' value.");
					Catalog.SetValue(Config.CatalogPath, Text(action, "id"), "loop", JsonValue.Create(loop));
					m_catalog = Catalog.Read(Config.CatalogPath);
					return null;
				}
				case "load":
				{
					// Only the catalog's 'load' value changes, and only to a value the project understands
					var load = Text(action, "load");
					if (!CatalogSound.Loads.Contains(load))
						throw new InvalidOperationException($"The 'load' action needs one of: {string.Join(", ", CatalogSound.Loads)}.");

					Catalog.SetValue(Config.CatalogPath, Text(action, "id"), "load", JsonValue.Create(load));
					m_catalog = Catalog.Read(Config.CatalogPath);
					return null;
				}
				case "status":
				{
					// Accepting adds the variant at the end of the runtime order and fetches its original; any other change leaves its position alone
					var (prep, variant) = VariantOf(action);
					var status = JsonSerializer.Deserialize<EVariantStatus>(action["status"], AuthoringJson.Options);
					if (status == EVariantStatus.Accepted && variant.Status != EVariantStatus.Accepted)
					{
						prep.Variants.Remove(variant);
						var last_accepted = prep.Variants.FindLastIndex(x => x.Status == EVariantStatus.Accepted);
						prep.Variants.Insert(last_accepted + 1, variant);
					}
					variant.Status = status;
					prep.Save(Config.PrepFolder);
					if (status == EVariantStatus.Accepted)
						Prepare(prep.Id, variant);

					return null;
				}
				case "move":
				{
					// Swap with the neighbouring accepted variant, which is what runtime order means
					var (prep, variant) = VariantOf(action);
					var delta = action["delta"]?.GetValue<int>() ?? 0;
					var index = prep.Variants.IndexOf(variant);
					var other = delta < 0
						? prep.Variants.FindLastIndex(index, x => x.Status == EVariantStatus.Accepted && x != variant)
						: prep.Variants.FindIndex(index, x => x.Status == EVariantStatus.Accepted && x != variant);
					if (variant.Status != EVariantStatus.Accepted || delta == 0 || other < 0)
						throw new InvalidOperationException("Only an accepted variant can move, and only past another accepted variant.");

					(prep.Variants[index], prep.Variants[other]) = (prep.Variants[other], prep.Variants[index]);
					prep.Save(Config.PrepFolder);
					return null;
				}
				case "note":
				{
					var (prep, variant) = VariantOf(action);
					var note = Text(action, "note").Trim();
					variant.Note = note.Length != 0 ? note : null;
					prep.Save(Config.PrepFolder);
					return null;
				}
				case "finalised":
				{
					var (prep, variant) = VariantOf(action);
					variant.Finalised = action["finalised"]?.GetValue<bool>() ?? false;
					prep.Save(Config.PrepFolder);
					return null;
				}
				case "recipe":
				{
					// Save the edited recipe and render it; a newer edit supersedes a render still in progress
					var (prep, variant) = VariantOf(action);
					variant.Recipe = JsonSerializer.Deserialize<List<RecipeStep>>(action["recipe"], AuthoringJson.Options) ?? [];
					prep.Save(Config.PrepFolder);
					Prepare(prep.Id, variant);
					return null;
				}
				case "adopt":
				{
					// From now on the recipe's render is the variant's prepared audio; the reference file itself is left untouched
					var (prep, variant) = VariantOf(action);
					if (variant.Source == null)
						throw new InvalidOperationException("Import the original before adopting its recipe.");

					variant.ReferenceMaster = null;
					prep.Save(Config.PrepFolder);
					return "The recipe now prepares this variant. The previous reference file was not changed.";
				}
				case "prepare":
				{
					// Import or render every variant of a sound that has been accepted or is under review
					var prep = PrepOf(Text(action, "id"));
					foreach (var variant in prep.Variants.Where(x => x.Status != EVariantStatus.Rejected))
						Prepare(prep.Id, variant, import: variant.Status == EVariantStatus.Accepted || action["import"]?.GetValue<bool>() == true);

					return null;
				}
				default:
				{
					throw new InvalidOperationException($"Unknown action '{name}'.");
				}
			}
		}
	}

	/// <summary>
	/// Add Freesound sound 'number' to the origins registry, downloading its browse preview, and assign it to 'sound_id' if given. The page and preview
	/// are fetched without holding the state lock.
	/// </summary>
	public async Task<string?> AddFreesoundAsync(string? sound_id, string number, CancellationToken cancel)
	{
		// A known origin needs no download
		var origin = $"freesound:{number}";
		lock (m_lock)
		{
			if (m_origins.Origins.ContainsKey(origin))
				return sound_id != null ? Assign(sound_id, origin) : $"Sound {number} is already known.";
		}

		// Fetch the descriptive fields and the preview into a folder named after its first use
		var info = await Freesound.SoundInfoAsync(number, cancel);
		var previews = Config.PreviewsFolder ?? throw new InvalidOperationException("The project does not configure a previews folder.");
		var safe_title = SafeName().Replace(info.Title, "_");
		var preview = $"{sound_id ?? "unassigned"}/{number}-{safe_title[..Math.Min(60, safe_title.Length)]}.mp3";
		await Freesound.DownloadPreviewAsync(info.PreviewUrl, Path.Combine(previews, preview), cancel);

		lock (m_lock)
		{
			// Another request may have added it meanwhile; keep the first record
			if (!m_origins.Origins.ContainsKey(origin))
			{
				m_origins.Origins[origin] = new OriginRecord { Title = info.Title, User = info.User, Licence = info.Licence, Url = info.Url, Preview = preview };
				SaveOrigins();
			}
			return sound_id != null ? Assign(sound_id, origin) : "Added.";
		}
	}

	/// <summary>The origin whose browse preview is stored at 'preview' (relative to the previews folder), or null.</summary>
	public string? OriginOfPreview(string preview)
	{
		lock (m_lock)
			return m_origins.Origins.FirstOrDefault(x => x.Value.Preview == preview).Key;
	}

	/// <summary>Assign 'origin' to 'sound_id' and describe its other uses. Caller holds the lock.</summary>
	private string Assign(string sound_id, string origin)
	{
		// Assigning makes the origin useful again
		var record = OriginRecordOf(origin);
		record.NotUseful = false;
		SaveOrigins();

		// An earlier rejection is reconsidered rather than duplicated
		var prep = PrepOf(sound_id);
		var existing = prep.Variants.FirstOrDefault(x => x.Origin == origin);
		string notice;
		if (existing == null)
		{
			prep.AddVariant(origin, record.Source, EVariantStatus.Unheard);
			notice = $"Assigned to {sound_id}.";
		}
		else if (existing.Status == EVariantStatus.Rejected)
		{
			existing.Status = EVariantStatus.Candidate;
			notice = $"Rejected earlier for {sound_id}; it is a candidate again.";
		}
		else
		{
			notice = $"Already {JsonSerializer.Serialize(existing.Status, AuthoringJson.Options).Trim('"')} for {sound_id}.";
		}
		prep.Save(Config.PrepFolder);

		// Other live uses help avoid using one recording for unrelated sounds
		var others = m_catalog.Select(x => x.Id).Where(x => x != sound_id && PrepOf(x).Variants.Any(v => v.Origin == origin && v.Status != EVariantStatus.Rejected)).ToList();
		return others.Count != 0 ? $"{notice} Also used for {string.Join(", ", others)}." : notice;
	}

	/// <summary>Render 'variant' if its original is imported, otherwise import it first when 'import' is true. Caller holds the lock.</summary>
	private void Prepare(string sound_id, Variant variant, bool import = true)
	{
		// An imported variant renders straight away
		if (variant.Source != null)
		{
			QueueRender(sound_id, variant);
			return;
		}

		// Hand-made sources are added outside the tool, so there is nothing to fetch
		if (!import || OriginsFile.KindOf(variant.Origin) == EOriginKind.HandMade)
			return;

		QueueImport(variant.Origin);
	}

	/// <summary>Render the current recipe of 'variant' in the background. Caller holds the lock.</summary>
	private void QueueRender(string sound_id, Variant variant)
	{
		// Copy the inputs so later edits cannot change a render in progress
		var slot = RenderSlot(sound_id, variant.Id);
		var source = variant.Source!;
		var origin = variant.Origin;
		var record = CopyOf(OriginRecordOf(origin));
		var recipe = JsonSerializer.Deserialize<List<RecipeStep>>(AuthoringJson.ToCanonicalString(variant.Recipe), AuthoringJson.Options)!;
		QueueJob(slot, $"Render {sound_id} {variant.Id}", async cancel =>
		{
			// An original missing from the fetched folder is fetched again before rendering
			await Fetcher.EnsureAsync(origin, record, source, cancel);
			return await Cache.GetOrRenderAsync(source, recipe, cancel);
		}, result =>
		{
			// The newest render replaces the previous one
			lock (m_lock)
				m_renders[slot] = new RenderState { Key = result.key };
		}, error =>
		{
			// The last working render stays playable beside the error
			lock (m_lock)
			{
				var state = m_renders.GetValueOrDefault(slot) ?? (m_renders[slot] = new RenderState());
				state.Error = error.Message;
			}
		});
	}

	/// <summary>Import the original of 'origin' in the background, then bind it to every variant of that origin and render them. Caller holds the lock.</summary>
	private void QueueImport(string origin)
	{
		// Capture the provenance now; the record may be edited while the job runs
		var record = CopyOf(OriginRecordOf(origin));
		QueueJob(ImportSlot(origin), $"Import {origin}", cancel => Fetcher.ImportAsync(origin, record, cancel), sidecar =>
		{
			// Bind the source to the origin and to every variant still waiting for it, then render those variants
			lock (m_lock)
			{
				m_import_errors.Remove(origin);
				OriginRecordOf(origin).Source = sidecar.Id;
				SaveOrigins();
				foreach (var sound in m_catalog)
				{
					var prep = PrepOf(sound.Id);
					var waiting = prep.Variants.Where(x => x.Origin == origin && x.Source == null).ToList();
					if (waiting.Count == 0)
						continue;

					foreach (var variant in waiting)
					{
						variant.Source = sidecar.Id;
						QueueRender(sound.Id, variant);
					}
					prep.Save(Config.PrepFolder);
				}
			}
		}, error =>
		{
			// Report the failure on the origin; the variants keep playing the preview
			lock (m_lock)
				m_import_errors[origin] = error.Message;
		});
	}

	/// <summary>Run a job on the thread pool, so that no caller lock is held when <see cref="LatestJobs"/> takes its own.</summary>
	private void QueueJob<T>(string slot, string description, Func<CancellationToken, Task<T>> work, Action<T> on_completed, Action<Exception> on_failed)
	{
		_ = Task.Run(() => Jobs.Run(slot, description, work, on_completed, on_failed));
	}

	/// <summary>The address that plays 'origin' for review. Caller holds the lock.</summary>
	private string? PlayUrl(string origin, OriginRecord record)
	{
		// Prefer the original; otherwise the browse copy
		if (record.Source != null)
			return $"/original/{record.Source}";

		switch (OriginsFile.KindOf(origin))
		{
			case EOriginKind.Freesound:
			{
				return record.Preview != null ? $"/previews/{EscapePath(record.Preview)}" : null;
			}
			case EOriginKind.Local:
			{
				return $"/local/{EscapePath(OriginsFile.ReferenceOf(origin))}";
			}
			case EOriginKind.HandMade:
			{
				return null;
			}
			default:
			{
				throw new ArgumentOutOfRangeException(nameof(origin), origin, "Unknown origin kind.");
			}
		}
	}

	/// <summary>The preparation file of 'sound_id', loading it on first use. Caller holds the lock.</summary>
	private PrepFile PrepOf(string sound_id)
	{
		// Only catalog IDs have preparation files
		if (m_preps.TryGetValue(sound_id, out var prep))
			return prep;

		if (!m_catalog.Any(x => x.Id == sound_id))
			throw new InvalidOperationException($"'{sound_id}' is not a sound ID in the catalog.");

		return m_preps[sound_id] = PrepFile.Load(Config.PrepFolder, sound_id);
	}

	/// <summary>The preparation file and variant named by an action's 'id' and 'variant'. Caller holds the lock.</summary>
	private (PrepFile prep, Variant variant) VariantOf(JsonObject action)
	{
		var prep = PrepOf(Text(action, "id"));
		var variant = prep.Find(Text(action, "variant")) ?? throw new InvalidOperationException($"'{prep.Id}' has no variant '{Text(action, "variant")}'.");
		return (prep, variant);
	}

	/// <summary>The registry record of 'origin'. Caller holds the lock.</summary>
	private OriginRecord OriginRecordOf(string origin)
	{
		return m_origins.Origins.GetValueOrDefault(origin) ?? throw new InvalidOperationException($"Unknown origin '{origin}'.");
	}

	/// <summary>A copy of the provenance in 'record', for a background job to read while the registry may change. Caller holds the lock.</summary>
	private static OriginRecord CopyOf(OriginRecord record)
	{
		return new OriginRecord { Title = record.Title, User = record.User, Licence = record.Licence, Url = record.Url };
	}

	/// <summary>Save the origins registry. Caller holds the lock.</summary>
	private void SaveOrigins()
	{
		AuthoringJson.Write(Config.OriginsPath, m_origins);
	}

	/// <summary>The job slot that imports 'origin'.</summary>
	private static string ImportSlot(string origin)
	{
		return $"import:{origin}";
	}

	/// <summary>The job slot that renders a variant.</summary>
	private static string RenderSlot(string sound_id, string variant_id)
	{
		return $"render:{sound_id}/{variant_id}";
	}

	/// <summary>A required string argument of an action.</summary>
	private static string Text(JsonObject action, string name)
	{
		return action[name] is JsonValue value && value.TryGetValue<string>(out var text) ? text : throw new InvalidOperationException($"The action needs '{name}'.");
	}

	/// <summary>Escape each segment of a relative path for use in a URL.</summary>
	private static string EscapePath(string path)
	{
		return string.Join('/', path.Split('/').Select(Uri.EscapeDataString));
	}

	/// <summary>
	/// The sound ID a local file's path names, or null when nothing matches. A folder whose name is a sound ID, ignoring case and with '-', '_' or ' '
	/// in place of '.' (so 'lab-radio' names 'lab.radio'), assigns everything beneath it; the deepest such folder wins. Otherwise the result is the sound
	/// ID whose dot-separated parts best match the words of the folder and file names. A trailing 's' is ignored, so 'birds' matches 'bird'; between
	/// equal matches, the ID with fewer parts wins because more of it matched.
	/// </summary>
	public static string? GuessSoundId(string relative_path, IEnumerable<string> sound_ids)
	{
		// A folder that names a sound ID is an explicit assignment, so it takes precedence over words in track titles
		var ids = sound_ids.ToList();
		foreach (var folder in relative_path.Split('/').SkipLast(1).Reverse())
		{
			// Compare in one canonical spelling of the ID
			var named = ids.FirstOrDefault(x => string.Equals(x, FolderSeparators().Replace(folder, "."), StringComparison.OrdinalIgnoreCase));
			if (named != null)
				return named;
		}

		// Words are the lower-case letter and digit runs of the folder and file names
		var words = WordSplit().Split(Path.ChangeExtension(relative_path, null).ToLowerInvariant()).Where(x => x.Length != 0).Select(TrimPlural).ToHashSet();
		string? best = null;
		var best_score = (matched: 0, shortness: 0);
		foreach (var id in ids)
		{
			// Score by matched parts, then by fewer parts
			var parts = id.Split('.');
			var score = (matched: parts.Count(x => words.Contains(TrimPlural(x))), shortness: -parts.Length);
			if (score.matched != 0 && (best == null || score.CompareTo(best_score) > 0))
				(best, best_score) = (id, score);
		}
		return best;

		static string TrimPlural(string word)
		{
			return word.EndsWith('s') ? word[..^1] : word;
		}
	}

	[GeneratedRegex("[^a-z0-9]+")]
	private static partial Regex WordSplit();

	[GeneratedRegex("[-_ ]")]
	private static partial Regex FolderSeparators();

	[GeneratedRegex("[^A-Za-z0-9._-]+")]
	private static partial Regex SafeName();
}
