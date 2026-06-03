using System;
using System.Collections.Generic;
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
		var registration = await ResolveInstanceAsync(instance_id).ConfigureAwait(false);
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
		[Description("The id of a running LDraw instance from ldraw_list_instances. Omit to target the default instance (most-recently-used, or auto-launched when none are running).")] string? instance_id = null)
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
