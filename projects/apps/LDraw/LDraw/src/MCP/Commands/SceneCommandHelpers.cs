using System;
using System.Collections.Generic;
using System.Linq;
using LDraw.UI;

namespace LDraw.MCP;

/// <summary>Shared helpers for scene lifecycle MCP commands</summary>
internal sealed partial class LDrawInstanceHost
{
	/// <summary>MCP overlay membership operation for a scene</summary>
	private enum ESceneSourceMode
	{
		Replace,
		Add,
		Remove,
		Clear,
	}

	/// <summary>Resolve a scene that must be named explicitly</summary>
	private SceneUI ResolveExplicitScene(string? scene_name, string command_name)
	{
		if (string.IsNullOrWhiteSpace(scene_name))
			throw new InvalidOperationException($"{command_name} requires an explicit scene_name.");
		return ResolveScene(scene_name);
	}

	/// <summary>Return context ids currently present in 'scene'</summary>
	private static List<Guid> SceneContextIds(SceneUI scene)
	{
		var context_ids = new List<Guid>();
		scene.SceneView.Scene.Window.EnumGuids(context_id => context_ids.Add(context_id));
		return context_ids;
	}

	/// <summary>Return user-loaded sources currently present in 'scene'</summary>
	private List<Source> UserSourcesInScene(SceneUI scene)
	{
		var context_ids = SceneContextIds(scene).ToHashSet();
		return [..m_model.Sources.Where(source => context_ids.Contains(source.ContextId))];
	}

	/// <summary>Return overlay states currently present in 'scene'</summary>
	private OverlayState[] OverlayStatesInScene(SceneUI scene)
	{
		var context_ids = SceneContextIds(scene).ToHashSet();
		lock (m_overlay_lock)
			return [..m_overlays.Values.Where(state => context_ids.Contains(state.ContextId))];
	}

	/// <summary>Resolve overlay ids into overlay states</summary>
	private OverlayState[] ResolveOverlayStates(IReadOnlyList<string> overlay_ids, bool all_overlays, string command_name, bool allow_empty)
	{
		lock (m_overlay_lock)
		{
			if (all_overlays)
				return [..m_overlays.Values];
			if (overlay_ids.Count == 0)
			{
				if (allow_empty)
					return [];
				throw new InvalidOperationException($"{command_name} requires overlay_ids unless all_overlays is true.");
			}

			var states = new List<OverlayState>(overlay_ids.Count);
			var seen = new HashSet<string>(StringComparer.OrdinalIgnoreCase);
			foreach (var overlay_id in overlay_ids)
			{
				var id = NormaliseOverlayId(overlay_id);
				if (!m_overlays.TryGetValue(id, out var state))
					throw new InvalidOperationException($"No MCP overlay named '{id}' exists.");
				if (seen.Add(id))
					states.Add(state);
			}
			return [..states];
		}
	}

	/// <summary>Parse scene-source membership mode</summary>
	private static ESceneSourceMode ParseSceneSourceMode(string? mode)
	{
		return (mode ?? "replace").Trim().ToLowerInvariant() switch
		{
			"replace" => ESceneSourceMode.Replace,
			"add" => ESceneSourceMode.Add,
			"remove" => ESceneSourceMode.Remove,
			"clear" => ESceneSourceMode.Clear,
			_ => throw new ArgumentOutOfRangeException(nameof(mode), mode, "Scene source mode must be replace, add, remove, or clear."),
		};
	}

	/// <summary>Set whether 'state' is recorded as visible in 'scene_name'</summary>
	private static void SetOverlaySceneName(OverlayState state, string scene_name, bool visible)
	{
		state.SceneNames.RemoveAll(name => string.Equals(name, scene_name, StringComparison.OrdinalIgnoreCase));
		if (visible)
			state.SceneNames.Add(scene_name);
	}

	/// <summary>Apply overlay membership changes to 'scene'</summary>
	private List<OverlayState> ApplyOverlayMembership(SceneUI scene, ESceneSourceMode mode, OverlayState[] requested, bool reset_view)
	{
		var current = OverlayStatesInScene(scene);
		var current_ids = current.Select(state => state.ContextId).ToHashSet();
		var requested_ids = requested.Select(state => state.ContextId).ToHashSet();

		OverlayState[] add;
		OverlayState[] remove;
		switch (mode)
		{
			case ESceneSourceMode.Replace:
			{
				add = [..requested.Where(state => !current_ids.Contains(state.ContextId))];
				remove = [..current.Where(state => !requested_ids.Contains(state.ContextId))];
				break;
			}
			case ESceneSourceMode.Add:
			{
				add = [..requested.Where(state => !current_ids.Contains(state.ContextId))];
				remove = [];
				break;
			}
			case ESceneSourceMode.Remove:
			{
				add = [];
				remove = [..requested.Where(state => current_ids.Contains(state.ContextId))];
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

		foreach (var state in remove)
		{
			m_model.Clear(scene, state.ContextId);
			SetOverlaySceneName(state, scene.SceneName, visible: false);
		}
		foreach (var state in add)
		{
			m_model.AddObjects(scene, state.ContextId, reset_view);
			SetOverlaySceneName(state, scene.SceneName, visible: true);
		}
		if (add.Length != 0 || remove.Length != 0)
			scene.SceneView.Invalidate();

		return [..add.Concat(remove).DistinctBy(state => state.ContextId)];
	}

	/// <summary>Create a scene mutation result from current model state</summary>
	private LDrawSceneMutationResult CreateSceneMutationResult(string action, string scene_name, LDrawSceneInfo? scene, IEnumerable<OverlayState> overlays)
	{
		return new LDrawSceneMutationResult
		{
			Action = action,
			SceneName = scene_name,
			Scene = scene,
			Scenes = [..m_model.Scenes.Select(CreateSceneInfo)],
			Overlays = [..overlays.Select(state => state.ToInfo())],
		};
	}
}
