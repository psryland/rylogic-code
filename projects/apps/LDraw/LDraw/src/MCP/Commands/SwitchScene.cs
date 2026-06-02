using System;
using System.ComponentModel;
using System.Threading.Tasks;
using ModelContextProtocol.Server;

namespace LDraw.MCP;

/// <summary>Switch-scene command broker routing</summary>
internal sealed partial class McpBroker
{
	/// <summary>Activate a scene in 'instance_id'</summary>
	public async Task<LDrawSceneMutationResult> SwitchSceneAsync(string? instance_id, LDrawSceneParams parameters)
	{
		var registration = ResolveInstance(instance_id);
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

/// <summary>Switch-scene command instance implementation</summary>
internal sealed partial class LDrawInstanceHost
{
	/// <summary>Activate a named scene</summary>
	private Task<LDrawSceneMutationResult> SwitchSceneAsync(LDrawSceneParams parameters)
	{
		return m_model.InvokeAsync(() =>
		{
			var scene = ResolveExplicitScene(parameters.SceneName, "ldraw_switch_scene");
			m_model.ActivateScene(scene);
			return CreateSceneMutationResult("switch_scene", scene.SceneName, CreateSceneInfo(scene), []);
		}, TimeSpan.FromSeconds(5));
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
		[Description("The instance id returned by ldraw_list_instances. Omit to target the broker instance.")] string? instance_id = null)
	{
		var parameters = new LDrawSceneParams
		{
			SceneName = scene_name,
		};
		return m_broker.SwitchSceneAsync(instance_id, parameters);
	}
}
