using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Threading.Tasks;
using ModelContextProtocol.Server;

namespace LDraw.MCP;

/// <summary>Additional scene lifecycle command broker routing</summary>
internal sealed partial class McpBroker
{
	/// <summary>Clear a scene in 'instance_id'</summary>
	public async Task<LDrawSceneMutationResult> ClearSceneAsync(string? instance_id, LDrawSceneParams parameters)
	{
		var registration = ResolveInstance(instance_id);
		return await m_client.ClearSceneAsync(registration, parameters).ConfigureAwait(false);
	}

	/// <summary>Rename a scene in 'instance_id'</summary>
	public async Task<LDrawSceneMutationResult> RenameSceneAsync(string? instance_id, LDrawRenameSceneParams parameters)
	{
		var registration = ResolveInstance(instance_id);
		return await m_client.RenameSceneAsync(registration, parameters).ConfigureAwait(false);
	}
}

/// <summary>Additional scene lifecycle command pipe client calls</summary>
internal sealed partial class InstancePipeClient
{
	/// <summary>Clear a scene in 'registration'</summary>
	public async Task<LDrawSceneMutationResult> ClearSceneAsync(InstanceRegistration registration, LDrawSceneParams parameters)
	{
		return await SendAsync<LDrawSceneMutationResult>(registration, InstancePipeCommands.ClearScene, parameters, WriteTimeout).ConfigureAwait(false);
	}

	/// <summary>Rename a scene in 'registration'</summary>
	public async Task<LDrawSceneMutationResult> RenameSceneAsync(InstanceRegistration registration, LDrawRenameSceneParams parameters)
	{
		return await SendAsync<LDrawSceneMutationResult>(registration, InstancePipeCommands.RenameScene, parameters, WriteTimeout).ConfigureAwait(false);
	}
}

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

/// <summary>Additional scene lifecycle command MCP tool surface</summary>
internal sealed partial class LDrawTools
{
	/// <summary>Clear all current objects from a scene</summary>
	[McpServerTool(Name = "ldraw_clear_scene", Title = "Clear LDraw scene", ReadOnly = false, Destructive = true, Idempotent = true, OpenWorld = false, UseStructuredContent = true)]
	[Description("Removes all currently rendered objects from a scene. Source membership is left unchanged, so later reloads or membership commands can add objects again.")]
	public Task<LDrawSceneMutationResult> ClearScene(
		[Description("The scene name to clear. Required.")] string scene_name,
		[Description("The instance id returned by ldraw_list_instances. Omit to target the broker instance.")] string? instance_id = null)
	{
		var parameters = new LDrawSceneParams
		{
			SceneName = scene_name,
		};
		return m_broker.ClearSceneAsync(instance_id, parameters);
	}

	/// <summary>Rename a scene view</summary>
	[McpServerTool(Name = "ldraw_rename_scene", Title = "Rename LDraw scene", ReadOnly = false, Destructive = false, Idempotent = true, OpenWorld = false, UseStructuredContent = true)]
	[Description("Renames a scene view, requiring a unique name and preserving user-source and MCP overlay memberships.")]
	public Task<LDrawSceneMutationResult> RenameScene(
		[Description("The existing scene name. Required.")] string scene_name,
		[Description("The new unique scene name.")] string new_scene_name,
		[Description("The instance id returned by ldraw_list_instances. Omit to target the broker instance.")] string? instance_id = null)
	{
		var parameters = new LDrawRenameSceneParams
		{
			SceneName = scene_name,
			NewSceneName = new_scene_name,
		};
		return m_broker.RenameSceneAsync(instance_id, parameters);
	}
}
