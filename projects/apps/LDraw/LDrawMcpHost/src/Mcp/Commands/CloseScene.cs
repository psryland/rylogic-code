using System;
using System.Collections.Generic;
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
		var registration = await ResolveInstanceAsync(instance_id).ConfigureAwait(false);
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

/// <summary>Close-scene command MCP tool surface</summary>
internal sealed partial class LDrawTools
{
	/// <summary>Close a scene that contains only MCP-owned overlay sources</summary>
	[McpServerTool(Name = "ldraw_close_scene", Title = "Close LDraw scene", ReadOnly = false, Destructive = false, Idempotent = false, OpenWorld = false, UseStructuredContent = true)]
	[Description("Closes a scene only when it contains no user-loaded sources. Closing the last scene creates a new empty replacement scene.")]
	public Task<LDrawSceneMutationResult> CloseScene(
		[Description("The scene name to close. Required.")] string scene_name,
		[Description("The id of a running LDraw instance from ldraw_list_instances. Omit to target the default instance (most-recently-used, or auto-launched when none are running).")] string? instance_id = null)
	{
		var parameters = new LDrawSceneParams
		{
			SceneName = scene_name,
		};
		return m_broker.CloseSceneAsync(instance_id, parameters);
	}
}
