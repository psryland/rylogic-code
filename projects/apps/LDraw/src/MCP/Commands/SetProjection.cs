using System;
using System.ComponentModel;
using System.Threading.Tasks;
using ModelContextProtocol.Server;

namespace LDraw.MCP;

/// <summary>Set-projection command broker routing</summary>
internal sealed partial class McpBroker
{
	/// <summary>Set projection mode for a scene in 'instance_id'</summary>
	public async Task<LDrawViewMutationResult> SetProjectionAsync(string? instance_id, LDrawSetProjectionParams parameters)
	{
		var registration = ResolveInstance(instance_id);
		return await m_client.SetProjectionAsync(registration, parameters).ConfigureAwait(false);
	}
}

/// <summary>Set-projection command pipe client call</summary>
internal sealed partial class InstancePipeClient
{
	/// <summary>Set projection mode for a scene in 'registration'</summary>
	public async Task<LDrawViewMutationResult> SetProjectionAsync(InstanceRegistration registration, LDrawSetProjectionParams parameters)
	{
		return await SendAsync<LDrawViewMutationResult>(registration, InstancePipeCommands.SetProjection, parameters, WriteTimeout).ConfigureAwait(false);
	}
}

/// <summary>Set-projection command instance implementation</summary>
internal sealed partial class LDrawInstanceHost
{
	/// <summary>Set perspective or orthographic projection for the requested scene</summary>
	private Task<LDrawViewMutationResult> SetProjectionAsync(LDrawSetProjectionParams parameters)
	{
		return m_model.InvokeAsync(() =>
		{
			var scene = ResolveScene(parameters.SceneName);
			scene.SceneState.Chart.Orthographic = parameters.Orthographic;
			scene.SceneView.Invalidate();
			return CreateViewMutationResult(parameters.Orthographic ? "set_projection_orthographic" : "set_projection_perspective", scene);
		}, TimeSpan.FromSeconds(5));
	}
}

/// <summary>Set-projection command MCP tool surface</summary>
internal sealed partial class LDrawTools
{
	/// <summary>Set a scene to perspective or orthographic projection</summary>
	[McpServerTool(Name = "ldraw_set_projection", Title = "Set LDraw projection", ReadOnly = false, Destructive = false, Idempotent = true, OpenWorld = false, UseStructuredContent = true)]
	[Description("Sets a scene to orthographic or perspective projection and persists the normal scene setting.")]
	public Task<LDrawViewMutationResult> SetProjection(
		[Description("True for orthographic projection; false for perspective projection.")] bool orthographic,
		[Description("The instance id returned by ldraw_list_instances. Omit to target the broker instance.")] string? instance_id = null,
		[Description("The scene name to modify. Omit to use the first scene.")] string? scene_name = null)
	{
		var parameters = new LDrawSetProjectionParams
		{
			SceneName = scene_name,
			Orthographic = orthographic,
		};
		return m_broker.SetProjectionAsync(instance_id, parameters);
	}
}
