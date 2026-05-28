using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.IO;
using System.Linq;
using System.Threading.Tasks;
using LDraw.UI;
using ModelContextProtocol.Server;

namespace LDraw.MCP;

/// <summary>Source command broker routing</summary>
internal sealed partial class McpBroker
{
	/// <summary>List sources in 'instance_id'</summary>
	public async Task<LDrawSourceList> ListSourcesAsync(string? instance_id, LDrawListSourcesParams parameters)
	{
		var registration = ResolveInstance(instance_id);
		return await m_client.ListSourcesAsync(registration, parameters).ConfigureAwait(false);
	}

	/// <summary>Load a file source in 'instance_id'</summary>
	public async Task<LDrawSourceMutationResult> LoadSourceAsync(string? instance_id, LDrawLoadSourceParams parameters)
	{
		var registration = ResolveInstance(instance_id);
		return await m_client.LoadSourceAsync(registration, parameters).ConfigureAwait(false);
	}

	/// <summary>Reload sources in 'instance_id'</summary>
	public async Task<LDrawSourceMutationResult> ReloadSourceAsync(string? instance_id, LDrawSourceParams parameters)
	{
		var registration = ResolveInstance(instance_id);
		return await m_client.ReloadSourceAsync(registration, parameters).ConfigureAwait(false);
	}

	/// <summary>Remove sources from 'instance_id'</summary>
	public async Task<LDrawSourceMutationResult> RemoveSourceAsync(string? instance_id, LDrawSourceParams parameters)
	{
		var registration = ResolveInstance(instance_id);
		return await m_client.RemoveSourceAsync(registration, parameters).ConfigureAwait(false);
	}

	/// <summary>Set user-source scene membership in 'instance_id'</summary>
	public async Task<LDrawSourceMutationResult> SetSourceScenesAsync(string? instance_id, LDrawSetSourceScenesParams parameters)
	{
		var registration = ResolveInstance(instance_id);
		return await m_client.SetSourceScenesAsync(registration, parameters).ConfigureAwait(false);
	}
}

/// <summary>Source command pipe client calls</summary>
internal sealed partial class InstancePipeClient
{
	/// <summary>List sources in 'registration'</summary>
	public async Task<LDrawSourceList> ListSourcesAsync(InstanceRegistration registration, LDrawListSourcesParams parameters)
	{
		return await SendAsync<LDrawSourceList>(registration, InstancePipeCommands.ListSources, parameters, ReadTimeout).ConfigureAwait(false);
	}

	/// <summary>Load a file source in 'registration'</summary>
	public async Task<LDrawSourceMutationResult> LoadSourceAsync(InstanceRegistration registration, LDrawLoadSourceParams parameters)
	{
		return await SendAsync<LDrawSourceMutationResult>(registration, InstancePipeCommands.LoadSource, parameters, WriteTimeout).ConfigureAwait(false);
	}

	/// <summary>Reload sources in 'registration'</summary>
	public async Task<LDrawSourceMutationResult> ReloadSourceAsync(InstanceRegistration registration, LDrawSourceParams parameters)
	{
		return await SendAsync<LDrawSourceMutationResult>(registration, InstancePipeCommands.ReloadSource, parameters, WriteTimeout).ConfigureAwait(false);
	}

	/// <summary>Remove sources from 'registration'</summary>
	public async Task<LDrawSourceMutationResult> RemoveSourceAsync(InstanceRegistration registration, LDrawSourceParams parameters)
	{
		return await SendAsync<LDrawSourceMutationResult>(registration, InstancePipeCommands.RemoveSource, parameters, WriteTimeout).ConfigureAwait(false);
	}

	/// <summary>Set user-source scene membership in 'registration'</summary>
	public async Task<LDrawSourceMutationResult> SetSourceScenesAsync(InstanceRegistration registration, LDrawSetSourceScenesParams parameters)
	{
		return await SendAsync<LDrawSourceMutationResult>(registration, InstancePipeCommands.SetSourceScenes, parameters, WriteTimeout).ConfigureAwait(false);
	}
}

/// <summary>Source command instance implementation</summary>
internal sealed partial class LDrawInstanceHost
{
	/// <summary>List user-loaded sources, optionally filtered by scene membership</summary>
	private Task<LDrawSourceList> ListSourcesAsync(LDrawListSourcesParams parameters)
	{
		return m_model.InvokeAsync(() =>
		{
			var sources = m_model.Sources.AsEnumerable();
			if (!string.IsNullOrWhiteSpace(parameters.SceneName))
			{
				var scene = ResolveExplicitScene(parameters.SceneName, "ldraw_list_sources");
				sources = sources.Where(source => SourceSelectedInScene(source, scene));
			}

			return new LDrawSourceList
			{
				Sources = [..sources.Select(CreateSourceSummary)],
			};
		}, TimeSpan.FromSeconds(2));
	}

	/// <summary>Load a file-backed source and show it in the requested scenes</summary>
	private Task<LDrawSourceMutationResult> LoadSourceAsync(LDrawLoadSourceParams parameters)
	{
		return m_model.InvokeAsync(() =>
		{
			if (string.IsNullOrWhiteSpace(parameters.FilePath))
				throw new InvalidOperationException("ldraw_load_source requires file_path.");

			var filepath = Path.GetFullPath(parameters.FilePath);
			if (!File.Exists(filepath))
				throw new FileNotFoundException($"LDraw source file '{filepath}' does not exist.", filepath);

			var scenes = ResolveSourceTargetScenes(parameters.SceneNames, parameters.AllScenes);
			var source = m_model.AddFileSource(filepath, scenes);
			var source_info = CreateSourceSummary(source);
			return CreateSourceMutationResult("load_source", source_info, [source_info]);
		}, TimeSpan.FromSeconds(30));
	}

	/// <summary>Reload existing file-backed sources</summary>
	private Task<LDrawSourceMutationResult> ReloadSourceAsync(LDrawSourceParams parameters)
	{
		return m_model.InvokeAsync(() =>
		{
			var sources = ResolveSources(parameters, "ldraw_reload_source", allow_empty: false);
			foreach (var source in sources)
				source.Reload();

			var affected = sources.Select(CreateSourceSummary).ToList();
			return CreateSourceMutationResult("reload_source", affected.Count == 1 ? affected[0] : null, affected);
		}, TimeSpan.FromSeconds(30));
	}

	/// <summary>Remove user-loaded sources from the instance</summary>
	private Task<LDrawSourceMutationResult> RemoveSourceAsync(LDrawSourceParams parameters)
	{
		return m_model.InvokeAsync(() =>
		{
			var sources = ResolveSources(parameters, "ldraw_remove_source", allow_empty: false);
			var affected = sources.Select(CreateSourceSummary).ToList();
			foreach (var source in sources)
				source.Remove();

			return CreateSourceMutationResult("remove_source", affected.Count == 1 ? affected[0] : null, affected);
		}, TimeSpan.FromSeconds(30));
	}

	/// <summary>Change which user-loaded sources are visible in the requested scenes</summary>
	private Task<LDrawSourceMutationResult> SetSourceScenesAsync(LDrawSetSourceScenesParams parameters)
	{
		return m_model.InvokeAsync(() =>
		{
			var mode = ParseSceneSourceMode(parameters.Mode);
			var scenes = ResolveSourceTargetScenes(parameters.SceneNames, parameters.AllScenes);
			var sources = ResolveSources(parameters, "ldraw_set_source_scenes", allow_empty: mode == ESceneSourceMode.Clear);
			var changed = new Dictionary<Guid, Source>();

			foreach (var scene in scenes)
			{
				foreach (var source in ApplySourceMembership(scene, mode, sources, parameters.ResetView))
					changed[source.ContextId] = source;
			}

			var affected = changed.Values.Select(CreateSourceSummary).ToList();
			return CreateSourceMutationResult($"set_source_scenes_{mode.ToString().ToLowerInvariant()}", affected.Count == 1 ? affected[0] : null, affected);
		}, TimeSpan.FromSeconds(30));
	}

	/// <summary>Resolve the scene list for a source command</summary>
	private SceneUI[] ResolveSourceTargetScenes(IReadOnlyList<string> scene_names, bool all_scenes)
	{
		if (all_scenes)
			return [..m_model.Scenes];
		return ResolveScenes(scene_names);
	}

	/// <summary>Resolve user sources from context ids, display names, or file paths</summary>
	private Source[] ResolveSources(LDrawSourceParams parameters, string command_name, bool allow_empty)
	{
		if (parameters.AllSources)
			return [..m_model.Sources];
		if (parameters.SourceIds.Count == 0)
		{
			if (allow_empty)
				return [];
			throw new InvalidOperationException($"{command_name} requires source_ids unless all_sources is true.");
		}

		var sources = new List<Source>();
		var seen = new HashSet<Guid>();
		foreach (var source_id in parameters.SourceIds)
		{
			var source = ResolveSource(source_id, command_name);
			if (seen.Add(source.ContextId))
				sources.Add(source);
		}
		return [..sources];
	}

	/// <summary>Resolve a single user source from a context id, display name, or file path</summary>
	private Source ResolveSource(string source_id, string command_name)
	{
		if (string.IsNullOrWhiteSpace(source_id))
			throw new InvalidOperationException($"{command_name} source ids cannot be empty.");

		var id = source_id.Trim();
		Source[] matches;
		if (Guid.TryParse(id, out var context_id))
		{
			matches = [..m_model.Sources.Where(source => source.ContextId == context_id)];
		}
		else
		{
			var path_id = NormalisePathForCompare(id);
			matches =
			[
				..m_model.Sources.Where(source =>
					string.Equals(source.Name, id, StringComparison.OrdinalIgnoreCase) ||
					string.Equals(source.FilePath, id, StringComparison.OrdinalIgnoreCase) ||
					(source.FilePath.Length != 0 && string.Equals(NormalisePathForCompare(source.FilePath), path_id, StringComparison.OrdinalIgnoreCase)))
			];
		}

		if (matches.Length == 0)
			throw new InvalidOperationException($"{command_name} could not find source '{id}'.");
		if (matches.Length != 1)
			throw new InvalidOperationException($"{command_name} found {matches.Length} sources named or pathed '{id}'. Use a context id instead.");

		return matches[0];
	}

	/// <summary>Apply a source membership mode to one scene</summary>
	private Source[] ApplySourceMembership(SceneUI scene, ESceneSourceMode mode, Source[] requested, bool reset_view)
	{
		var current = m_model.Sources.Where(source => SourceSelectedInScene(source, scene)).ToArray();
		var current_ids = current.Select(source => source.ContextId).ToHashSet();
		var requested_ids = requested.Select(source => source.ContextId).ToHashSet();

		Source[] add;
		Source[] remove;
		switch (mode)
		{
			case ESceneSourceMode.Replace:
			{
				add = [..requested.Where(source => !current_ids.Contains(source.ContextId))];
				remove = [..current.Where(source => !requested_ids.Contains(source.ContextId))];
				break;
			}
			case ESceneSourceMode.Add:
			{
				add = [..requested.Where(source => !current_ids.Contains(source.ContextId))];
				remove = [];
				break;
			}
			case ESceneSourceMode.Remove:
			{
				add = [];
				remove = [..requested.Where(source => current_ids.Contains(source.ContextId))];
				break;
			}
			case ESceneSourceMode.Clear:
			{
				add = [];
				remove = current;
				break;
			}
			default:
			{
				throw new ArgumentOutOfRangeException(nameof(mode), mode, "Unknown scene source mode.");
			}
		}

		foreach (var source in remove)
			source.ShowInScenes([scene], show: false);
		foreach (var source in add)
			source.ShowInScenes([scene], show: true, reset_view: reset_view);

		if (add.Length != 0 || remove.Length != 0)
			scene.SceneView.Invalidate();

		return [..add.Concat(remove).DistinctBy(source => source.ContextId)];
	}

	/// <summary>Return true if 'source' is selected for 'scene'</summary>
	private static bool SourceSelectedInScene(Source source, SceneUI scene)
	{
		return source.SelectedScenes.Any(selected => string.Equals(selected.SceneName, scene.SceneName, StringComparison.OrdinalIgnoreCase));
	}

	/// <summary>Create a source mutation result from current model state</summary>
	private LDrawSourceMutationResult CreateSourceMutationResult(string action, LDrawSourceSummary? source, IEnumerable<LDrawSourceSummary> sources)
	{
		return new LDrawSourceMutationResult
		{
			Action = action,
			Source = source,
			Sources = [..sources],
			Scenes = [..m_model.Scenes.Select(CreateSceneInfo)],
		};
	}

	/// <summary>Normalise a file path for command-side source matching</summary>
	private static string NormalisePathForCompare(string filepath)
	{
		try
		{
			return Path.GetFullPath(filepath).TrimEnd(Path.DirectorySeparatorChar, Path.AltDirectorySeparatorChar);
		}
		catch (Exception ex) when (ex is ArgumentException or NotSupportedException or PathTooLongException)
		{
			return filepath.Trim();
		}
	}
}

/// <summary>Source command MCP tool surface</summary>
internal sealed partial class LDrawTools
{
	/// <summary>List user-loaded LDraw sources</summary>
	[McpServerTool(Name = "ldraw_list_sources", Title = "List LDraw sources", ReadOnly = true, Destructive = false, Idempotent = true, OpenWorld = false, UseStructuredContent = true)]
	[Description("Lists user-loaded LDraw sources, optionally filtered to sources selected for one scene.")]
	public Task<LDrawSourceList> ListSources(
		[Description("The scene name to filter by. Omit to list all user-loaded sources.")] string? scene_name = null,
		[Description("The instance id returned by ldraw_list_instances. Omit to target the broker instance.")] string? instance_id = null)
	{
		var parameters = new LDrawListSourcesParams
		{
			SceneName = scene_name,
		};
		return m_broker.ListSourcesAsync(instance_id, parameters);
	}

	/// <summary>Load a file-backed LDraw source</summary>
	[McpServerTool(Name = "ldraw_load_source", Title = "Load LDraw source", ReadOnly = false, Destructive = false, Idempotent = false, OpenWorld = true, UseStructuredContent = true)]
	[Description("Synchronously loads a file-backed LDraw source and shows it in requested scenes. Scene names default to the first scene.")]
	public Task<LDrawSourceMutationResult> LoadSource(
		[Description("Path to the .ldr/.bdr source file to load.")] string file_path,
		[Description("Scene names to show the source in. Omit to use the first scene.")] string[]? scene_names = null,
		[Description("True to show the source in every current scene.")] bool all_scenes = false,
		[Description("The instance id returned by ldraw_list_instances. Omit to target the broker instance.")] string? instance_id = null)
	{
		var parameters = new LDrawLoadSourceParams
		{
			FilePath = file_path,
			SceneNames = scene_names?.ToList() ?? [],
			AllScenes = all_scenes,
		};
		return m_broker.LoadSourceAsync(instance_id, parameters);
	}

	/// <summary>Reload user-loaded LDraw sources</summary>
	[McpServerTool(Name = "ldraw_reload_source", Title = "Reload LDraw source", ReadOnly = false, Destructive = false, Idempotent = false, OpenWorld = true, UseStructuredContent = true)]
	[Description("Reloads one or more existing user-loaded sources from their backing files.")]
	public Task<LDrawSourceMutationResult> ReloadSource(
		[Description("Source context ids, display names, or file paths to reload. Required unless all_sources is true.")] string[]? source_ids = null,
		[Description("True to reload every current user-loaded source.")] bool all_sources = false,
		[Description("The instance id returned by ldraw_list_instances. Omit to target the broker instance.")] string? instance_id = null)
	{
		var parameters = new LDrawSourceParams
		{
			SourceIds = source_ids?.ToList() ?? [],
			AllSources = all_sources,
		};
		return m_broker.ReloadSourceAsync(instance_id, parameters);
	}

	/// <summary>Remove user-loaded LDraw sources</summary>
	[McpServerTool(Name = "ldraw_remove_source", Title = "Remove LDraw source", ReadOnly = false, Destructive = true, Idempotent = false, OpenWorld = false, UseStructuredContent = true)]
	[Description("Removes one or more existing user-loaded sources from the instance.")]
	public Task<LDrawSourceMutationResult> RemoveSource(
		[Description("Source context ids, display names, or file paths to remove. Required unless all_sources is true.")] string[]? source_ids = null,
		[Description("True to remove every current user-loaded source.")] bool all_sources = false,
		[Description("The instance id returned by ldraw_list_instances. Omit to target the broker instance.")] string? instance_id = null)
	{
		var parameters = new LDrawSourceParams
		{
			SourceIds = source_ids?.ToList() ?? [],
			AllSources = all_sources,
		};
		return m_broker.RemoveSourceAsync(instance_id, parameters);
	}

	/// <summary>Set user-loaded source scene membership</summary>
	[McpServerTool(Name = "ldraw_set_source_scenes", Title = "Set LDraw source scenes", ReadOnly = false, Destructive = false, Idempotent = true, OpenWorld = false, UseStructuredContent = true)]
	[Description("Changes which user-loaded sources are visible in scene views. MCP-owned overlay sources are never added or removed.")]
	public Task<LDrawSourceMutationResult> SetSourceScenes(
		[Description("Membership operation: replace, add, remove, or clear. Replace means exactly the requested user sources are visible in the target scenes.")] string mode = "replace",
		[Description("Source context ids, display names, or file paths to use for replace/add/remove operations. Required unless all_sources is true or mode is clear.")] string[]? source_ids = null,
		[Description("Scene names to modify. Omit to use the first scene.")] string[]? scene_names = null,
		[Description("True to target every current user-loaded source.")] bool all_sources = false,
		[Description("True to target every current scene.")] bool all_scenes = false,
		[Description("True to frame each scene after adding source objects.")] bool reset_view = false,
		[Description("The instance id returned by ldraw_list_instances. Omit to target the broker instance.")] string? instance_id = null)
	{
		var parameters = new LDrawSetSourceScenesParams
		{
			Mode = mode,
			SourceIds = source_ids?.ToList() ?? [],
			SceneNames = scene_names?.ToList() ?? [],
			AllSources = all_sources,
			AllScenes = all_scenes,
			ResetView = reset_view,
		};
		return m_broker.SetSourceScenesAsync(instance_id, parameters);
	}
}
