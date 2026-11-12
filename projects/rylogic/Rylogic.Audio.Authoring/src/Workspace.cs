using System;
using System.Collections.Generic;
using System.Diagnostics;
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
public sealed partial class Workspace : IDisposable
{
	/// <summary>The latest render of one variant. A failed render keeps the key of the last successful one, so the last working audio stays playable.</summary>
	private sealed class RenderState
	{
		public string? Key;
		public string? Error;
	}

	/// <summary>The size and write time of a file, which change whenever the file is written.</summary>
	private readonly record struct FileStamp(long Length, DateTime Modified);

	/// <summary>
	/// A variant's working copy, opened in the external editor. 'Seen' is the stamp of the last version imported or written by the tool, and 'Candidate'
	/// is the stamp found by the previous poll. A new version is imported once its stamp has stopped changing for one poll.
	/// </summary>
	private sealed class EditSession
	{
		public required string SoundId;
		public required string VariantId;
		public required string Path;
		public FileStamp? Seen;
		public FileStamp? Candidate;
		public string? Notice;
		public string? Error;
	}

	// Lock order: a job's result is published while LatestJobs holds its own lock, and the publisher then takes 'm_lock'. Code holding 'm_lock' must
	// therefore never call into 'Jobs' directly; it queues jobs on the thread pool instead (see QueueJob).
	private readonly object m_lock = new();
	private readonly Dictionary<string, PrepFile> m_preps = new(StringComparer.Ordinal);
	private readonly Dictionary<string, RenderState> m_renders = new(StringComparer.Ordinal);
	private readonly Dictionary<string, string> m_import_errors = new(StringComparer.Ordinal);
	private readonly Dictionary<string, (string? notice, string? error)> m_finds = new(StringComparer.Ordinal);
	private readonly Dictionary<string, EditSession> m_edits = new(StringComparer.Ordinal);
	private readonly CancellationTokenSource m_shutdown = new();
	private Task? m_edit_poll;
	private readonly OriginsFile m_origins;
	private IReadOnlyList<CatalogSound> m_catalog;

	/// <summary>Open the project described by 'config'. 'assistant' finds new candidates; without one, finding is unavailable.</summary>
	public Workspace(AuthoringConfig config, Ffmpeg ffmpeg, HttpClient http, IAssistant? assistant = null)
	{
		// The catalog, origins and preparation files are read now
		Config = config;
		Ffmpeg = ffmpeg;
		Assistant = assistant;
		Sources = new SourceStore(config.OriginalsFolder, config.FetchedFolder, ffmpeg);
		Cache = new RenderCache(config.CacheFolder, Sources, ffmpeg);
		Freesound = new FreesoundClient(http, config.FreesoundCredentialsPath);
		Fetcher = new OriginalFetcher(config, Sources, Freesound);
		Jobs = new LatestJobs(Math.Max(1, Environment.ProcessorCount / 2));
		m_catalog = Catalog.Read(config.CatalogPath);
		m_origins = OriginsFile.Load(config.OriginsPath);

		// Render every recipe the game uses, so playback matches the game from the start. Renders already in the cache finish at once.
		lock (m_lock)
		{
			foreach (var sound in m_catalog)
			{
				foreach (var variant in PrepOf(sound.Id).Variants.Where(x => x.Source != null && x.ReferenceMaster == null && x.Status != EVariantStatus.Rejected))
					QueueRender(sound.Id, variant);
			}
		}
	}

	/// <summary>Stop watching the editor working copies. Background jobs already running finish on their own.</summary>
	public void Dispose()
	{
		// The poll loop ends at its next tick
		m_shutdown.Cancel();
	}

	/// <summary>
	/// Raised with a sound ID when background work, rather than a review action, changes what the project's game plays for that sound. It is raised
	/// on a worker thread, without any workspace lock held.
	/// </summary>
	public event Action<string>? GameChanged;

	/// <summary>How often the editor working copies are checked for a new export.</summary>
	public static TimeSpan EditPollPeriod { get; } = TimeSpan.FromMilliseconds(500);

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

	/// <summary>The AI assistant that finds new candidates, or null if none is configured.</summary>
	public IAssistant? Assistant { get; }

	/// <summary>
	/// The background jobs. Slots are 'import:&lt;origin&gt;', 'render:&lt;sound-id&gt;/&lt;variant-id&gt;', 'find:&lt;sound-id&gt;' and
	/// 'edit:&lt;sound-id&gt;/&lt;variant-id&gt;'.
	/// </summary>
	public LatestJobs Jobs { get; }

	/// <summary>The file types accepted from local folders.</summary>
	public static IReadOnlyList<string> LocalTypes { get; } = [".wav", ".mp3", ".flac", ".ogg", ".opus"];

	/// <summary>The review actions that change what the project's game plays for the action's sound 'id'. Other actions only change the review state.</summary>
	public static IReadOnlySet<string> GameActions { get; } = new HashSet<string>(["add_sound", "edit_sound", "delete_sound", "status", "move", "recipe"], StringComparer.Ordinal);

	/// <summary>The sound ID an action changes: the new ID of an added or edited sound, otherwise the action's 'id', or null when it names none.</summary>
	public static string? SoundIdOf(JsonObject action)
	{
		return (string?)action["sound"]?["id"] ?? (string?)action["id"];
	}

	/// <summary>The current state for display. Never waits for background work.</summary>
	public JsonObject Snapshot()
	{
		// Read the jobs first: taking the jobs lock while holding 'm_lock' would invert the lock order
		var pending = Jobs.Pending;
		var freesound = Freesound.Status;
		lock (m_lock)
		{
			// The sound IDs in catalog order, each as its catalog properties plus its section, which is the form the edit actions take
			var ids = new JsonArray();
			foreach (var sound in m_catalog)
			{
				var node = JsonNode.Parse(Catalog.FormatSound(sound))!.AsObject();
				node["section"] = sound.Section;
				ids.Add(node);
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

					// The state of editing the variant in the external editor, once it has been opened there
					var edit_slot = EditSlot(sound.Id, variant.Id);
					var edit = m_edits.GetValueOrDefault(edit_slot);
					if (edit != null || pending.ContainsKey(edit_slot))
						node["edit"] = new JsonObject { ["pending"] = pending.ContainsKey(edit_slot), ["notice"] = edit?.Notice, ["error"] = edit?.Error, ["path"] = edit?.Path };

					variants.Add(node);
				}
				preps[sound.Id] = variants;
			}

			// The latest candidate search of each sound ID that has one
			var finds = new JsonObject();
			foreach (var sound in m_catalog)
			{
				var slot = FindSlot(sound.Id);
				var (notice, error) = m_finds.GetValueOrDefault(sound.Id);
				if (pending.ContainsKey(slot) || notice != null || error != null)
					finds[sound.Id] = new JsonObject { ["pending"] = pending.ContainsKey(slot), ["notice"] = notice, ["error"] = error };
			}

			var assistant = Assistant?.Status ?? new AssistantStatus(false, CopilotAssistant.EnableHint);
			return new JsonObject
			{
				["ids"] = ids,
				["origins"] = origins,
				["preps"] = preps,
				["pending"] = new JsonArray(pending.Values.Order(StringComparer.Ordinal).Select(x => (JsonNode)x).ToArray()),
				["freesound"] = new JsonObject { ["configured"] = freesound.Configured, ["connected"] = freesound.Connected },
				["assistant"] = new JsonObject { ["available"] = assistant.Available, ["description"] = assistant.Description },
				["finds"] = finds,
				["editor"] = new JsonObject
				{
					["available"] = Config.Editor != null,
					["name"] = Config.Editor?.ProgramName ?? "Audacity",
					["hint"] = Config.Editor == null ? $"Install Audacity, or set 'editor.command' in '{Config.ConfigPath}', then restart the tool." : null,
				},
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
				case "find_candidates":
				{
					return FindCandidates(Text(action, "id"));
				}
				case "open_editor":
				{
					var (prep, variant) = VariantOf(action);
					return OpenEditor(prep.Id, variant);
				}
				case "add_sound":
				{
					// A new sound has no variants yet; any leftover files for its ID would silently attach to it, so they are refused
					var sound = SoundOf(action);
					var catalog = Catalog.Add(m_catalog, sound);
					if (File.Exists(PrepFile.PathFor(Config.PrepFolder, sound.Id)))
						throw new InvalidOperationException($"A preparation file for '{sound.Id}' already exists. Remove it, or add the sound under another ID.");

					SaveCatalog(catalog);
					return $"Added {sound.Id}.";
				}
				case "edit_sound":
				{
					// Every catalog value may change. A new ID renames the sound's own files before the catalog is saved, so they always match it.
					var id = Text(action, "id");
					var sound = SoundOf(action);
					var catalog = Catalog.Replace(m_catalog, id, sound);
					if (sound.Id != id)
						RenameFiles(id, sound.Id);

					SaveCatalog(catalog);
					return sound.Id != id ? $"Renamed {id} to {sound.Id}." : null;
				}
				case "delete_sound":
				{
					// The sound's files go with it; version control is the way back
					var id = Text(action, "id");
					var catalog = Catalog.Remove(m_catalog, id);
					DeleteFiles(id);
					SaveCatalog(catalog);
					return $"Deleted {id}.";
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
					else if (status == EVariantStatus.Rejected)
						ForgetRejectedImport(variant.Origin);

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
					// Save the edited recipe and render it; a newer edit supersedes a render still in progress.
					// Editing a recipe makes it the variant's prepared audio, replacing any reference master; the reference file itself is left untouched.
					var (prep, variant) = VariantOf(action);
					if (variant.Source == null)
						throw new InvalidOperationException("Import the original before editing its recipe.");

					var mixed_before = variant.MixInputs.Select(x => x.Origin).ToHashSet(StringComparer.Ordinal);
					variant.Recipe = JsonSerializer.Deserialize<List<RecipeStep>>(action["recipe"], AuthoringJson.Options) ?? [];
					variant.ReferenceMaster = null;
					BindMixInputs(variant, mixed_before);
					prep.Save(Config.PrepFolder);
					Prepare(prep.Id, variant);
					return null;
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

	/// <summary>
	/// The credit to suggest for a file the user is about to add: its tags where it has them, and a title from its file name 'file_name' otherwise.
	/// 'file_path' is the file's contents, which may be a temporary copy.
	/// </summary>
	public async Task<FileCredit> SuggestCreditAsync(string file_path, string file_name, CancellationToken cancel)
	{
		// Tag names differ between formats and taggers, so the common spellings of each field are tried in turn
		var tags = await Ffmpeg.ReadTagsAsync(file_path, cancel);
		string? First(params string[] keys)
		{
			// The first non-empty tag among 'keys'
			return keys.Select(tags.GetValueOrDefault).FirstOrDefault(x => !string.IsNullOrWhiteSpace(x));
		}

		// A web address may have its own tag, or be written into a comment or the copyright notice
		var url = First("url", "www", "purl", "wxxx", "website", "woar", "woas");
		url ??= tags.Values.Select(x => WebAddress().Match(x)).FirstOrDefault(x => x.Success)?.Value;
		return new FileCredit
		{
			Title = First("title") ?? Path.GetFileNameWithoutExtension(file_name),
			Author = First("artist", "album_artist", "composer", "author", "performer"),
			Licence = First("license", "licence", "copyright"),
			Url = url,
		};
	}

	/// <summary>
	/// Add the audio file at 'file_path', named 'file_name' on the user's disk, as a new original credited with 'credit', and assign it to 'sound_id' if
	/// given. The file is copied into the originals folder, because it exists nowhere else the tool can find.
	/// </summary>
	public async Task<string> ImportFileAsync(string? sound_id, string file_path, string file_name, FileCredit credit, CancellationToken cancel)
	{
		// Only types the tool can decode are accepted, and every recording needs a name for the credit
		var type = Path.GetExtension(file_name).ToLowerInvariant();
		if (!LocalTypes.Contains(type))
			throw new InvalidOperationException($"'{file_name}' is not a supported audio file. Use one of: {string.Join(", ", LocalTypes)}.");
		if (string.IsNullOrWhiteSpace(credit.Title))
			throw new InvalidOperationException("Give the recording a title.");

		// The origin is named by the file's bytes, so adding the same file again finds the first copy
		var id = SourceStore.SourceIdOf(await SourceStore.Sha256Async(file_path, cancel));
		var origin = $"imported:{id}";
		lock (m_lock)
		{
			if (m_origins.Origins.ContainsKey(origin))
				return sound_id != null ? Assign(sound_id, origin) : $"'{file_name}' is already known.";
		}

		// Store the original without holding the lock; a wave file is converted to FLAC, which takes a moment
		var provenance = new SourceSidecar
		{
			Kind = EOriginKind.Imported,
			Origin = origin,
			Title = credit.Title.Trim(),
			Author = Blank(credit.Author),
			Licence = Blank(credit.Licence),
			Url = Blank(credit.Url),
			DownloadName = Path.GetFileName(file_name),
		};
		var sidecar = await Sources.ImportAsync(file_path, provenance, cancel);

		lock (m_lock)
		{
			// Another request may have added it meanwhile; keep the first record
			if (!m_origins.Origins.ContainsKey(origin))
			{
				m_origins.Origins[origin] = new OriginRecord { Title = provenance.Title, User = provenance.Author, Licence = provenance.Licence, Url = provenance.Url, Source = sidecar.Id };
				SaveOrigins();
			}
			return sound_id != null ? Assign(sound_id, origin) : $"Added '{file_name}' to the unassigned recordings.";
		}
	}

	/// <summary>
	/// Remove the file-added 'origin' once every variant using it is rejected: a rejected recording that exists nowhere else is not worth keeping.
	/// The variants go with it, and so does its original unless another origin shares it. Caller holds the lock.
	/// </summary>
	private void ForgetRejectedImport(string origin)
	{
		// Only file-added origins are forgotten, and only once nothing still considers them. A mix keeps its inputs even when rejected, because its recipe still names them.
		if (OriginsFile.KindOf(origin) != EOriginKind.Imported)
			return;
		if (m_catalog.Any(x => PrepOf(x.Id).Variants.Any(v => (v.Origin == origin && v.Status != EVariantStatus.Rejected) || v.MixInputs.Any(m => m.Origin == origin))))
			return;

		// Remove the rejected variants, and the renders shown for them, so no variant refers to a missing original
		foreach (var sound in m_catalog)
		{
			var prep = PrepOf(sound.Id);
			var gone = prep.Variants.Where(x => x.Origin == origin).ToList();
			if (gone.Count == 0)
				continue;

			foreach (var variant in gone)
			{
				prep.Variants.Remove(variant);
				m_renders.Remove(RenderSlot(sound.Id, variant.Id));
			}
			prep.Save(Config.PrepFolder);
		}

		// The same bytes may also have arrived another way, in which case the original belongs to that origin too
		var source = m_origins.Origins.GetValueOrDefault(origin)?.Source;
		m_origins.Origins.Remove(origin);
		SaveOrigins();
		if (source != null && !m_origins.Origins.Values.Any(x => x.Source == source) && Sources.Contains(source) && Sources.Sidecar(source).Kind == EOriginKind.Imported)
			Sources.Delete(source);
	}

	/// <summary>The origin whose browse preview is stored at 'preview' (relative to the previews folder), or null.</summary>
	public string? OriginOfPreview(string preview)
	{
		lock (m_lock)
			return m_origins.Origins.FirstOrDefault(x => x.Value.Preview == preview).Key;
	}

	/// <summary>
	/// Start a background search for up to <see cref="CandidateFinder.MaxPicks"/> new Freesound candidates for 'sound_id'. The assistant writes the
	/// queries and chooses among the results; each choice is added as an unheard variant. Only CC0 and CC-BY sounds new to the project are considered.
	/// Caller holds the lock.
	/// </summary>
	private string FindCandidates(string sound_id)
	{
		// Both the assistant and the Freesound credentials are needed before anything starts
		if (Assistant?.Status is not { Available: true })
			throw new InvalidOperationException(Assistant?.Status.Description ?? CopilotAssistant.EnableHint);
		if (!Freesound.Status.Configured)
			throw new InvalidOperationException("Enter the Freesound API credentials first.");

		// Capture the context now: the catalog entry, the author's accepted and rejected choices, and every Freesound sound the project already knows
		var sound = m_catalog.FirstOrDefault(x => x.Id == sound_id) ?? throw new InvalidOperationException($"'{sound_id}' is not a sound ID in the catalog.");
		var sound_json = Catalog.FormatSound(sound);
		var variants = PrepOf(sound_id).Variants;
		List<ReviewedSound> Reviewed(EVariantStatus status)
		{
			// The origin's title names the recording; the variant's note says what the author thought of it
			return [.. variants.Where(x => x.Status == status).Select(x => new ReviewedSound(m_origins.Origins.GetValueOrDefault(x.Origin)?.Title ?? x.Origin, x.Note))];
		}
		var liked = Reviewed(EVariantStatus.Accepted);
		var disliked = Reviewed(EVariantStatus.Rejected);
		var known = m_origins.Origins.Keys
			.Concat(m_catalog.SelectMany(x => PrepOf(x.Id).Variants.Select(v => v.Origin)))
			.Where(x => OriginsFile.KindOf(x) == EOriginKind.Freesound)
			.Select(OriginsFile.ReferenceOf)
			.ToHashSet(StringComparer.Ordinal);

		// Each step waits on the network or the model, so the whole search runs as one job; a new search for the same sound replaces it
		var assistant = Assistant!;
		m_finds.Remove(sound_id);
		QueueJob(FindSlot(sound_id), $"Find candidates for {sound_id}", async cancel =>
		{
			// Ask for queries, then search for each, keeping only sounds new to the project
			var queries = CandidateFinder.QueriesOf(await assistant.AskAsync(CandidateFinder.QueryPrompt(sound_json, liked, disliked), cancel));
			var results = new List<FreesoundSearchResult>();
			foreach (var query in queries)
				results.AddRange(await Freesound.SearchAsync(query, CandidateFinder.ResultsPerQuery, cancel));

			var choices = CandidateFinder.ChoicesOf(results, known);
			if (choices.Count == 0)
				return $"No new CC0 or CC-BY sounds were found for {sound_id} (searched: {string.Join(", ", queries)}).";

			// Let the assistant choose, then add each choice as an unheard variant
			var picks = CandidateFinder.PicksOf(await assistant.AskAsync(CandidateFinder.PickPrompt(sound_json, liked, disliked, choices), cancel), choices);
			if (picks.Count == 0)
				return $"None of the {choices.Count} new sounds found suit {sound_id} (searched: {string.Join(", ", queries)}).";

			foreach (var pick in picks)
				await AddFreesoundAsync(sound_id, pick.Number, cancel);

			return $"Added {picks.Count} candidate{(picks.Count == 1 ? "" : "s")} for {sound_id}: {string.Join(", ", picks.Select(x => x.Title))}.";
		}, notice =>
		{
			// The result stays visible until the next search of the sound
			lock (m_lock)
				m_finds[sound_id] = (notice, null);
		}, error =>
		{
			lock (m_lock)
				m_finds[sound_id] = (null, error.Message);
		});
		return $"Searching Freesound for {sound_id} candidates. This takes a minute or two.";
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
		var others = m_catalog.Select(x => x.Id).Where(x => x != sound_id && PrepOf(x).Variants.Any(v => v.Uses(origin) && v.Status != EVariantStatus.Rejected)).ToList();
		return others.Count != 0 ? $"{notice} Also used for {string.Join(", ", others)}." : notice;
	}

	/// <summary>
	/// Open a working WAV copy of the original of 'variant' in the external editor, and import each version the editor later saves over it. The
	/// imported version replaces the variant's original; its status, note and recipe are kept. Caller holds the lock.
	/// </summary>
	private string OpenEditor(string sound_id, Variant variant)
	{
		// The editor works on the unprocessed original, so the variant needs one
		var editor = Config.Editor ?? throw new InvalidOperationException($"No audio editor is available. Install Audacity, or set 'editor.command' in '{Config.ConfigPath}'.");
		if (variant.Source == null)
			throw new InvalidOperationException("Import the original before editing it.");

		// Capture the inputs; the working copy is named after the variant so the editor's title shows what is being edited
		var slot = EditSlot(sound_id, variant.Id);
		var origin = variant.Origin;
		var record = CopyOf(OriginRecordOf(origin));
		var source = variant.Source;
		var path = Path.Combine(Config.CacheFolder, "edits", $"{sound_id}.{variant.Id}.wav");
		var variant_id = variant.Id;
		QueueJob(slot, $"Open {sound_id} {variant_id} in {editor.ProgramName}", async cancel =>
		{
			// Write the original as 48 kHz float WAV, which every editor reads, then start the editor without waiting for it
			await Fetcher.EnsureAsync(origin, record, source, cancel);
			var audio = await Ffmpeg.DecodeAsync(Sources.OriginalPath(source), cancel);
			Directory.CreateDirectory(Path.GetDirectoryName(path)!);
			AuthoringJson.WriteAtomic(path, WavFile.Encode(audio));
			var stamp = StampOf(path);

			var command = editor.ForFile(path);
			var info = new ProcessStartInfo(command.Command) { UseShellExecute = false, WorkingDirectory = command.Directory };
			foreach (var argument in command.Arguments)
				info.ArgumentList.Add(argument);

			using var process = Process.Start(info) ?? throw new InvalidOperationException($"'{command.Command}' did not start.");
			return stamp;
		}, stamp =>
		{
			// The tool's own copy is not an edit; only a later version is imported
			lock (m_lock)
			{
				m_edits[slot] = new EditSession { SoundId = sound_id, VariantId = variant_id, Path = path, Seen = stamp, Candidate = stamp };
				m_edit_poll ??= Task.Run(() => PollEditsAsync(m_shutdown.Token));
			}
		}, error =>
		{
			// Keep any earlier session, so a failed reopen does not stop the watch on a copy that is still open
			lock (m_lock)
			{
				var session = m_edits.GetValueOrDefault(slot) ?? (m_edits[slot] = new EditSession { SoundId = sound_id, VariantId = variant_id, Path = path });
				session.Error = error.Message;
			}
		});
		return $"Opening {sound_id} {variant_id} in {editor.ProgramName}. Export the edit over the same file to use it.";
	}

	/// <summary>Watch every editor working copy, and import each new version once it has stopped changing.</summary>
	private async Task PollEditsAsync(CancellationToken cancel)
	{
		// Polling is used instead of change notifications, because editors replace files in several steps and a stable stamp shows that saving has finished
		using var timer = new PeriodicTimer(EditPollPeriod);
		try
		{
			while (await timer.WaitForNextTickAsync(cancel))
			{
				// Read the file stamps outside the lock, then compare and record them under it
				List<(string slot, EditSession session)> sessions;
				lock (m_lock)
					sessions = m_edits.Select(x => (x.Key, x.Value)).ToList();

				foreach (var (slot, session) in sessions)
				{
					// A version is imported when it differs from the last one seen and has not changed since the previous poll
					var stamp = StampOf(session.Path);
					lock (m_lock)
					{
						var ready = stamp != null && stamp != session.Seen && stamp == session.Candidate;
						session.Candidate = stamp;
						if (!ready)
							continue;

						session.Seen = stamp;
						QueueEditImport(slot, session);
					}
				}
			}
		}
		catch (OperationCanceledException)
		{
			// The workspace was disposed
		}
	}

	/// <summary>Import the current version of an editor working copy as the new original of its variant. Caller holds the lock.</summary>
	private void QueueEditImport(string slot, EditSession session)
	{
		// The variant may have been removed or renamed since it was opened
		var variant = m_catalog.Any(x => x.Id == session.SoundId) ? PrepOf(session.SoundId).Find(session.VariantId) : null;
		if (variant == null)
		{
			session.Error = $"{session.SoundId} {session.VariantId} no longer exists, so the edit was not imported.";
			return;
		}

		// An edit is credited like the recording it was made from, and names the first recording in its chain of edits
		var record = CopyOf(OriginRecordOf(variant.Origin));
		var edit_of = record.EditOf ?? variant.Origin;
		var path = session.Path;
		QueueJob(slot, $"Import the edit of {session.SoundId} {session.VariantId}", async cancel =>
		{
			// Work on a private copy, so the editor can save again while this version is imported
			var copy = Path.Combine(Path.GetDirectoryName(path)!, $"{Guid.NewGuid():N}.wav");
			try
			{
				File.Copy(path, copy);
				var id = SourceStore.SourceIdOf(await SourceStore.Sha256Async(copy, cancel));
				var provenance = new SourceSidecar
				{
					Kind = EOriginKind.Edited,
					Origin = $"edited:{id}",
					Title = record.Title,
					Author = record.User,
					Licence = record.Licence,
					Url = record.Url,
					EditOf = edit_of,
					DownloadName = Path.GetFileName(path),
				};
				return await Sources.ImportAsync(copy, provenance, cancel);
			}
			finally
			{
				File.Delete(copy);
			}
		}, sidecar =>
		{
			// The game only needs telling when the variant's original actually changed
			bool changed;
			lock (m_lock)
				changed = AdoptEdit(session, sidecar, record, edit_of);

			if (changed)
				GameChanged?.Invoke(session.SoundId);
		}, error =>
		{
			lock (m_lock)
				session.Error = error.Message;
		});
	}

	/// <summary>
	/// Make the imported edit 'sidecar' the original of the session's variant, and remove the edit it replaces if nothing else uses it. Returns false
	/// when the variant already uses it. Caller holds the lock.
	/// </summary>
	private bool AdoptEdit(EditSession session, SourceSidecar sidecar, OriginRecord record, string edit_of)
	{
		// The variant may have gone while the edit was imported, and an export of unchanged audio changes nothing
		session.Error = null;
		var variant = m_catalog.Any(x => x.Id == session.SoundId) ? PrepOf(session.SoundId).Find(session.VariantId) : null;
		if (variant == null)
		{
			session.Error = $"{session.SoundId} {session.VariantId} no longer exists, so the edit was not imported.";
			return false;
		}
		if (variant.Source == sidecar.Id)
			return false;

		// Register the edit as an origin and give it to the variant. Its review state and recipe still apply to the edited recording.
		var origin = sidecar.Origin;
		var previous = variant.Origin;
		m_origins.Origins[origin] = new OriginRecord { Title = record.Title, User = record.User, Licence = record.Licence, Url = record.Url, Source = sidecar.Id, EditOf = edit_of };
		variant.Origin = origin;
		variant.Source = sidecar.Id;
		variant.ReferenceMaster = null;
		var prep = PrepOf(session.SoundId);
		prep.Save(Config.PrepFolder);
		QueueRender(session.SoundId, variant);

		// Only the latest edit is kept: an earlier edit that no variant uses any more exists nowhere else worth keeping
		if (OriginsFile.KindOf(previous) == EOriginKind.Edited && !m_catalog.Any(x => PrepOf(x.Id).Variants.Any(v => v.Uses(previous))))
		{
			if (m_origins.Origins.GetValueOrDefault(previous)?.Source is string old_source)
				Sources.Delete(old_source);

			m_origins.Origins.Remove(previous);
		}
		SaveOrigins();
		session.Notice = $"Imported the edit at {DateTime.Now:HH:mm:ss}.";
		return true;
	}

	/// <summary>The stamp of the file at 'path', or null if it does not exist.</summary>
	private static FileStamp? StampOf(string path)
	{
		// A missing file has no stamp, so deleting the working copy is never mistaken for an edit
		var info = new FileInfo(path);
		return info.Exists ? new FileStamp(info.Length, info.LastWriteTimeUtc) : null;
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

		// Hand-made, edited and imported sources are added outside the tool, by the editor or from a file, so there is nothing to fetch
		if (!import || OriginsFile.KindOf(variant.Origin) is EOriginKind.HandMade or EOriginKind.Edited or EOriginKind.Imported)
			return;

		QueueImport(variant.Origin);
	}

	/// <summary>
	/// Bind each mix input of 'variant' to its origin's original. The import of a newly added input that has not been imported is started; inputs in
	/// 'mixed_before' already started theirs. The render fails with a reason until every input is imported. Caller holds the lock.
	/// </summary>
	private void BindMixInputs(Variant variant, IReadOnlySet<string> mixed_before)
	{
		// The registry is the only authority for an origin's source, so a source sent by the page is never trusted. An input not yet picked has no origin.
		foreach (var input in variant.MixInputs)
		{
			var record = m_origins.Origins.GetValueOrDefault(input.Origin);
			input.Source = record?.Source;
			if (record != null && input.Source == null && !mixed_before.Contains(input.Origin) && OriginsFile.KindOf(input.Origin) is EOriginKind.Freesound or EOriginKind.Local)
				QueueImport(input.Origin);
		}
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
		var mixed = recipe.Where(x => !x.Bypass).SelectMany(x => x.Inputs ?? []).Where(x => x.Source != null).Select(x => (x.Origin, record: CopyOf(OriginRecordOf(x.Origin)), source: x.Source!)).ToList();
		QueueJob(slot, $"Render {sound_id} {variant.Id}", async cancel =>
		{
			// Originals missing from the fetched folder are fetched again before rendering
			await Fetcher.EnsureAsync(origin, record, source, cancel);
			foreach (var input in mixed)
				await Fetcher.EnsureAsync(input.Origin, input.record, input.source, cancel);

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
					// Variants of the origin take it as their original; mixes waiting for it as an input render once it is bound
					var prep = PrepOf(sound.Id);
					var waiting = prep.Variants.Where(x => x.Origin == origin && x.Source == null).ToList();
					var mixing = prep.Variants.Where(x => x.MixInputs.Any(m => m.Origin == origin && m.Source == null)).ToList();
					if (waiting.Count == 0 && mixing.Count == 0)
						continue;

					foreach (var variant in waiting)
						variant.Source = sidecar.Id;
					foreach (var input in mixing.SelectMany(x => x.MixInputs).Where(x => x.Origin == origin))
						input.Source = sidecar.Id;
					foreach (var variant in waiting.Union(mixing).Where(x => x.Source != null))
						QueueRender(sound.Id, variant);

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
			case EOriginKind.Edited:
			case EOriginKind.Imported:
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

	/// <summary>The sound in an action's 'sound' object, in the section named by its 'section'.</summary>
	private static CatalogSound SoundOf(JsonObject action)
	{
		// The object has the catalog's form, so the catalog's reader parses it
		var sound = action["sound"] as JsonObject ?? throw new InvalidOperationException("The action needs a 'sound' object.");
		using var document = JsonDocument.Parse(sound.ToJsonString());
		return Catalog.ParseSound(document.RootElement, Text(action, "section").Trim(), "the request");
	}

	/// <summary>Write 'catalog' and read it back, so the state holds exactly what was saved. Caller holds the lock.</summary>
	private void SaveCatalog(IReadOnlyList<CatalogSound> catalog)
	{
		Catalog.Write(Config.CatalogPath, catalog);
		m_catalog = Catalog.Read(Config.CatalogPath);
	}

	/// <summary>
	/// Give the files of sound 'old_id' the ID 'new_id': its preparation file, and each reference master named '&lt;old_id&gt;.*' that no other sound
	/// uses. Every target is checked before anything moves, so a name clash changes nothing. Caller holds the lock.
	/// </summary>
	private void RenameFiles(string old_id, string new_id)
	{
		// Plan every move first. A master with another name, or one that another sound uses, keeps its name.
		var prep = PrepOf(old_id);
		var old_path = PrepFile.PathFor(Config.PrepFolder, old_id);
		if (File.Exists(PrepFile.PathFor(Config.PrepFolder, new_id)))
			throw new InvalidOperationException($"A preparation file for '{new_id}' already exists.");

		var shared = MastersOfOthers(old_id);
		var masters = new Dictionary<string, string>(StringComparer.Ordinal);
		foreach (var reference in prep.Variants.Select(x => x.ReferenceMaster).OfType<string>().Distinct().Where(x => !shared.Contains(x)))
		{
			// The ID part of the file name is replaced, keeping the folder and the rest of the name
			var name = reference[(reference.LastIndexOf('/') + 1)..];
			if (!name.StartsWith($"{old_id}.", StringComparison.Ordinal))
				continue;

			var renamed = $"{reference[..^name.Length]}{new_id}{name[old_id.Length..]}";
			if (File.Exists(MasterPath(renamed)))
				throw new InvalidOperationException($"Cannot rename reference master '{reference}', because '{renamed}' already exists.");

			masters[reference] = renamed;
		}

		// Move the masters and point the variants at their new names
		foreach (var (from, to) in masters.Where(x => File.Exists(MasterPath(x.Key))))
			File.Move(MasterPath(from), MasterPath(to));
		foreach (var variant in prep.Variants.Where(x => x.ReferenceMaster != null && masters.ContainsKey(x.ReferenceMaster)))
			variant.ReferenceMaster = masters[variant.ReferenceMaster!];

		// A sound that never had a preparation file still has none
		prep.Id = new_id;
		if (File.Exists(old_path))
		{
			prep.Save(Config.PrepFolder);
			File.Delete(old_path);
		}

		// Cached state follows the new ID
		m_preps.Remove(old_id);
		m_preps[new_id] = prep;
		foreach (var slot in RenderSlotsOf(old_id))
		{
			m_renders[$"{RenderSlot(new_id, string.Empty)}{slot[RenderSlot(old_id, string.Empty).Length..]}"] = m_renders[slot];
			m_renders.Remove(slot);
		}
	}

	/// <summary>Delete the preparation file of 'sound_id' and the reference masters that no other sound uses. Caller holds the lock.</summary>
	private void DeleteFiles(string sound_id)
	{
		// A master that another sound uses is kept for it
		var prep = PrepOf(sound_id);
		var shared = MastersOfOthers(sound_id);
		foreach (var reference in prep.Variants.Select(x => x.ReferenceMaster).OfType<string>().Distinct().Where(x => !shared.Contains(x)))
			File.Delete(MasterPath(reference));

		File.Delete(PrepFile.PathFor(Config.PrepFolder, sound_id));

		// Nothing cached may refer to the deleted sound
		m_preps.Remove(sound_id);
		foreach (var slot in RenderSlotsOf(sound_id))
			m_renders.Remove(slot);
	}

	/// <summary>The reference masters used by catalog sounds other than 'sound_id'. Caller holds the lock.</summary>
	private HashSet<string> MastersOfOthers(string sound_id)
	{
		return m_catalog.Where(x => x.Id != sound_id).SelectMany(x => PrepOf(x.Id).Variants).Select(x => x.ReferenceMaster).OfType<string>().ToHashSet(StringComparer.Ordinal);
	}

	/// <summary>The full path of a reference master given relative to the configuration folder.</summary>
	private string MasterPath(string reference)
	{
		return Path.GetFullPath(Path.Combine(Config.Root, reference));
	}

	/// <summary>The render slots of 'sound_id' that have a render state. Caller holds the lock.</summary>
	private List<string> RenderSlotsOf(string sound_id)
	{
		return m_renders.Keys.Where(x => x.StartsWith(RenderSlot(sound_id, string.Empty), StringComparison.Ordinal)).ToList();
	}

	/// <summary>The registry record of 'origin'. Caller holds the lock.</summary>
	private OriginRecord OriginRecordOf(string origin)
	{
		return m_origins.Origins.GetValueOrDefault(origin) ?? throw new InvalidOperationException($"Unknown origin '{origin}'.");
	}

	/// <summary>A copy of the provenance in 'record', for a background job to read while the registry may change. Caller holds the lock.</summary>
	private static OriginRecord CopyOf(OriginRecord record)
	{
		return new OriginRecord { Title = record.Title, User = record.User, Licence = record.Licence, Url = record.Url, EditOf = record.EditOf };
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

	/// <summary>The job slot that finds new candidates for a sound ID.</summary>
	private static string FindSlot(string sound_id)
	{
		return $"find:{sound_id}";
	}

	/// <summary>The job slot that opens a variant in the external editor and imports its edits. It also keys the variant's edit session.</summary>
	private static string EditSlot(string sound_id, string variant_id)
	{
		return $"edit:{sound_id}/{variant_id}";
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

	[GeneratedRegex(@"https?://[^\s""'<>]+", RegexOptions.IgnoreCase)]
	private static partial Regex WebAddress();

	/// <summary>'text' trimmed, or null when it is empty.</summary>
	private static string? Blank(string? text)
	{
		return string.IsNullOrWhiteSpace(text) ? null : text.Trim();
	}
}
