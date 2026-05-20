using System;
using System.ComponentModel;
using System.Linq;
using System.Threading.Tasks;
using ModelContextProtocol.Server;

namespace LDraw.MCP;

/// <summary>Set-scene-sources command broker routing</summary>
internal sealed partial class McpBroker
{
	/// <summary>Set MCP overlay membership for a scene in 'instance_id'</summary>
	public async Task<LDrawSceneMutationResult> SetSceneSourcesAsync(string? instance_id, LDrawSetSceneSourcesParams parameters)
	{
		var registration = ResolveInstance(instance_id);
		return await m_client.SetSceneSourcesAsync(registration, parameters).ConfigureAwait(false);
	}
}

/// <summary>Set-scene-sources command pipe client call</summary>
internal sealed partial class InstancePipeClient
{
	/// <summary>Set MCP overlay membership for a scene in 'registration'</summary>
	public async Task<LDrawSceneMutationResult> SetSceneSourcesAsync(InstanceRegistration registration, LDrawSetSceneSourcesParams parameters)
	{
		return await SendAsync<LDrawSceneMutationResult>(registration, InstancePipeCommands.SetSceneSources, parameters, WriteTimeout).ConfigureAwait(false);
	}
}

/// <summary>Set-scene-sources command instance implementation</summary>
internal sealed partial class LDrawInstanceHost
{
	/// <summary>Change the MCP overlay sources visible in a scene</summary>
	private async Task<LDrawSceneMutationResult> SetSceneSourcesAsync(LDrawSetSceneSourcesParams parameters)
	{
		await m_overlay_gate.WaitAsync().ConfigureAwait(false);
		try
		{
			return await m_model.InvokeAsync(() =>
			{
				var scene = ResolveExplicitScene(parameters.SceneName, "ldraw_set_scene_sources");
				var mode = ParseSceneSourceMode(parameters.Mode);
				var overlay_states = ResolveOverlayStates(parameters.OverlayIds, parameters.AllOverlays, "ldraw_set_scene_sources", allow_empty: mode == ESceneSourceMode.Clear);
				var changed = ApplyOverlayMembership(scene, mode, overlay_states, parameters.ResetView);
				return CreateSceneMutationResult($"set_scene_sources_{mode.ToString().ToLowerInvariant()}", scene.SceneName, CreateSceneInfo(scene), changed);
			}, TimeSpan.FromSeconds(10)).ConfigureAwait(false);
		}
		finally
		{
			m_overlay_gate.Release();
		}
	}
}

/// <summary>Set-scene-sources command MCP tool surface</summary>
internal sealed partial class LDrawTools
{
	/// <summary>Set MCP overlay source membership for a scene</summary>
	[McpServerTool(Name = "ldraw_set_scene_sources", Title = "Set LDraw scene sources", ReadOnly = false, Destructive = false, Idempotent = true, OpenWorld = false, UseStructuredContent = true)]
	[Description("Changes which MCP-owned overlay sources are visible in a scene. User-loaded sources are never added or removed.")]
	public Task<LDrawSceneMutationResult> SetSceneSources(
		[Description("The scene name to modify. Required.")] string scene_name,
		[Description("Membership operation: replace, add, remove, or clear. Replace means exactly the requested MCP overlays are visible.")] string mode = "replace",
		[Description("MCP overlay ids to use for replace/add/remove operations. Required unless all_overlays is true or mode is clear.")] string[]? overlay_ids = null,
		[Description("True to target all current MCP overlays.")] bool all_overlays = false,
		[Description("True to frame the scene after adding overlay sources.")] bool reset_view = false,
		[Description("The instance id returned by ldraw_list_instances. Omit to target the broker instance.")] string? instance_id = null)
	{
		var parameters = new LDrawSetSceneSourcesParams
		{
			SceneName = scene_name,
			Mode = mode,
			OverlayIds = overlay_ids?.ToList() ?? [],
			AllOverlays = all_overlays,
			ResetView = reset_view,
		};
		return m_broker.SetSceneSourcesAsync(instance_id, parameters);
	}
}
