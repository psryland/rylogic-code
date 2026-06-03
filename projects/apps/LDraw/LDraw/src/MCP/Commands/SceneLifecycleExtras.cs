using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Threading.Tasks;

namespace LDraw.MCP;

/// <summary>Additional scene lifecycle command instance implementation</summary>
internal sealed partial class LDrawInstanceHost
{
	/// <summary>Remove all current objects from a scene without changing source membership state</summary>
	private Task<LDrawSceneMutationResult> ClearSceneAsync(LDrawSceneParams parameters)
	{
		return m_model.InvokeAsync(() =>
		{
			var scene = ResolveExplicitScene(parameters.SceneName, "ldraw_clear_scene");
			m_model.Clear(scene);
			return CreateSceneMutationResult("clear_scene", scene.SceneName, CreateSceneInfo(scene), []);
		}, TimeSpan.FromSeconds(10));
	}

	/// <summary>Rename a scene and keep user-source and MCP overlay memberships attached</summary>
	private async Task<LDrawSceneMutationResult> RenameSceneAsync(LDrawRenameSceneParams parameters)
	{
		await m_overlay_gate.WaitAsync().ConfigureAwait(false);
		try
		{
			return await m_model.InvokeAsync(() =>
			{
				var scene = ResolveExplicitScene(parameters.SceneName, "ldraw_rename_scene");
				var new_name = ResolveRenameTarget(parameters.NewSceneName, scene.SceneName);
				var old_name = scene.SceneName;

				foreach (var source in m_model.Sources)
					source.RenameSceneMembership(old_name, new_name);
				RenameOverlaySceneMembership(old_name, new_name);
				RenameChartSceneMembership(old_name, new_name);

				scene.SceneName = new_name;
				return CreateSceneMutationResult("rename_scene", new_name, CreateSceneInfo(scene), OverlayStatesInScene(scene));
			}, TimeSpan.FromSeconds(10)).ConfigureAwait(false);
		}
		finally
		{
			m_overlay_gate.Release();
		}
	}

	/// <summary>Resolve a unique new scene name for a rename command</summary>
	private string ResolveRenameTarget(string? requested_name, string current_name)
	{
		if (string.IsNullOrWhiteSpace(requested_name))
			throw new InvalidOperationException("ldraw_rename_scene requires new_scene_name.");

		var new_name = requested_name.Trim();
		if (string.Equals(new_name, current_name, StringComparison.OrdinalIgnoreCase))
			return new_name;
		if (m_model.Scenes.Any(scene => string.Equals(scene.SceneName, new_name, StringComparison.OrdinalIgnoreCase)))
			throw new InvalidOperationException($"A scene named '{new_name}' already exists.");

		return new_name;
	}

	/// <summary>Rename scene membership entries in MCP overlay state</summary>
	private void RenameOverlaySceneMembership(string old_name, string new_name)
	{
		lock (m_overlay_lock)
		{
			foreach (var state in m_overlays.Values)
				RenameSceneName(state.SceneNames, old_name, new_name);
		}
	}

	/// <summary>Rename scene membership entries in generated chart state</summary>
	private void RenameChartSceneMembership(string old_name, string new_name)
	{
		lock (m_chart_lock)
		{
			foreach (var state in m_charts.Values)
				RenameSceneName(state.SceneNames, old_name, new_name);
		}
	}

	/// <summary>Replace one scene name in a mutable scene-name list</summary>
	private static void RenameSceneName(List<string> scene_names, string old_name, string new_name)
	{
		for (int i = 0; i != scene_names.Count; ++i)
		{
			if (string.Equals(scene_names[i], old_name, StringComparison.OrdinalIgnoreCase))
				scene_names[i] = new_name;
		}
	}
}
