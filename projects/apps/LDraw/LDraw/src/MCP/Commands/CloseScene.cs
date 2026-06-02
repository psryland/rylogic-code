using System;
using System.ComponentModel;
using System.Linq;
using System.Threading.Tasks;
using ModelContextProtocol.Server;

namespace LDraw.MCP;

/// <summary>Close-scene command broker routing</summary>
internal sealed partial class McpBroker
{
	/// <summary>Close a scene in 'instance_id'</summary>
	public async Task<LDrawSceneMutationResult> CloseSceneAsync(string? instance_id, LDrawSceneParams parameters)
	{
		var registration = ResolveInstance(instance_id);
		return await m_client.CloseSceneAsync(registration, parameters).ConfigureAwait(false);
	}
}

/// <summary>Close-scene command pipe client call</summary>
internal sealed partial class InstancePipeClient
{
	/// <summary>Close a scene in 'registration'</summary>
	public async Task<LDrawSceneMutationResult> CloseSceneAsync(InstanceRegistration registration, LDrawSceneParams parameters)
	{
		return await SendAsync<LDrawSceneMutationResult>(registration, InstancePipeCommands.CloseScene, parameters, WriteTimeout).ConfigureAwait(false);
	}
}

/// <summary>Close-scene command instance implementation</summary>
internal sealed partial class LDrawInstanceHost
{
	/// <summary>Close a scene when it contains no user-loaded sources</summary>
	private async Task<LDrawSceneMutationResult> CloseSceneAsync(LDrawSceneParams parameters)
	{
		await m_overlay_gate.WaitAsync().ConfigureAwait(false);
		try
		{
			return await m_model.InvokeAsync(() =>
			{
				var scene = ResolveExplicitScene(parameters.SceneName, "ldraw_close_scene");
				if (m_model.Scenes.Count <= 1)
					throw new InvalidOperationException("ldraw_close_scene cannot close the last remaining scene.");

				var user_sources = UserSourcesInScene(scene);
				if (user_sources.Count != 0)
				{
					var names = string.Join(", ", user_sources.Select(source => $"{source.Name} ({source.ContextId:D})"));
					throw new InvalidOperationException($"ldraw_close_scene cannot close scene '{scene.SceneName}' because it contains {user_sources.Count} user source(s): {names}.");
				}

				var scene_name = scene.SceneName;
				var scene_info = CreateSceneInfo(scene);
				var overlays = OverlayStatesInScene(scene);
				foreach (var state in overlays)
					SetOverlaySceneName(state, scene_name, visible: false);

				m_model.CloseScene(scene);
				return CreateSceneMutationResult("close_scene", scene_name, scene_info, overlays);
			}, TimeSpan.FromSeconds(10)).ConfigureAwait(false);
		}
		finally
		{
			m_overlay_gate.Release();
		}
	}
}

/// <summary>Close-scene command MCP tool surface</summary>
internal sealed partial class LDrawTools
{
	/// <summary>Close a scene that contains only MCP-owned overlay sources</summary>
	[McpServerTool(Name = "ldraw_close_scene", Title = "Close LDraw scene", ReadOnly = false, Destructive = false, Idempotent = false, OpenWorld = false, UseStructuredContent = true)]
	[Description("Closes a scene only when it contains no user-loaded sources. The last remaining scene cannot be closed.")]
	public Task<LDrawSceneMutationResult> CloseScene(
		[Description("The scene name to close. Required.")] string scene_name,
		[Description("The instance id returned by ldraw_list_instances. Omit to target the broker instance.")] string? instance_id = null)
	{
		var parameters = new LDrawSceneParams
		{
			SceneName = scene_name,
		};
		return m_broker.CloseSceneAsync(instance_id, parameters);
	}
}
