using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.IO;
using System.Linq;
using System.Threading.Tasks;
using LDraw.UI;

namespace LDraw.MCP;

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
