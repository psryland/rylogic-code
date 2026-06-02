using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Threading.Tasks;
using ModelContextProtocol.Server;

namespace LDraw.MCP;

/// <summary>Switch-scene command broker routing</summary>
internal sealed partial class McpBroker
{
	/// <summary>Activate a scene in 'instance_id'</summary>
	public async Task<LDrawSceneMutationResult> SwitchSceneAsync(string? instance_id, LDrawSceneParams parameters)
	{
		var registration = await ResolveInstanceAsync(instance_id).ConfigureAwait(false);
		return await m_client.SwitchSceneAsync(registration, parameters).ConfigureAwait(false);
	}
}

/// <summary>Switch-scene command pipe client call</summary>
internal sealed partial class InstancePipeClient
{
	/// <summary>Activate a scene in 'registration'</summary>
	public async Task<LDrawSceneMutationResult> SwitchSceneAsync(InstanceRegistration registration, LDrawSceneParams parameters)
	{
		return await SendAsync<LDrawSceneMutationResult>(registration, InstancePipeCommands.SwitchScene, parameters, WriteTimeout).ConfigureAwait(false);
	}
}

/// <summary>Switch-scene command MCP tool surface</summary>
internal sealed partial class LDrawTools
{
	/// <summary>Make a scene active and visible</summary>
	[McpServerTool(Name = "ldraw_switch_scene", Title = "Switch LDraw scene", ReadOnly = false, Destructive = false, Idempotent = true, OpenWorld = false, UseStructuredContent = true)]
	[Description("Makes a named scene the active visible scene in LDraw.")]
	public Task<LDrawSceneMutationResult> SwitchScene(
		[Description("The scene name to activate. Required.")] string scene_name,
		[Description("The id of a running LDraw instance from ldraw_list_instances. Omit to target the default instance (most-recently-used, or auto-launched when none are running).")] string? instance_id = null)
	{
		var parameters = new LDrawSceneParams
		{
			SceneName = scene_name,
		};
		return m_broker.SwitchSceneAsync(instance_id, parameters);
	}
}
