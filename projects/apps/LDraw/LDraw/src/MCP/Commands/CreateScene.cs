using System;
using System.ComponentModel;
using System.Linq;
using System.Threading.Tasks;
using ModelContextProtocol.Server;

namespace LDraw.MCP;

/// <summary>Create-scene command broker routing</summary>
internal sealed partial class McpBroker
{
	/// <summary>Create a scene in 'instance_id'</summary>
	public async Task<LDrawSceneMutationResult> CreateSceneAsync(string? instance_id, LDrawCreateSceneParams parameters)
	{
		var registration = ResolveInstance(instance_id);
		return await m_client.CreateSceneAsync(registration, parameters).ConfigureAwait(false);
	}
}

/// <summary>Create-scene command pipe client call</summary>
internal sealed partial class InstancePipeClient
{
	/// <summary>Create a scene in 'registration'</summary>
	public async Task<LDrawSceneMutationResult> CreateSceneAsync(InstanceRegistration registration, LDrawCreateSceneParams parameters)
	{
		return await SendAsync<LDrawSceneMutationResult>(registration, InstancePipeCommands.CreateScene, parameters, WriteTimeout).ConfigureAwait(false);
	}
}

/// <summary>Create-scene command instance implementation</summary>
internal sealed partial class LDrawInstanceHost
{
	/// <summary>Create a scene and optionally show MCP overlays in it</summary>
	private async Task<LDrawSceneMutationResult> CreateSceneAsync(LDrawCreateSceneParams parameters)
	{
		await m_overlay_gate.WaitAsync().ConfigureAwait(false);
		try
		{
			return await m_model.InvokeAsync(() =>
			{
				var overlay_states = ResolveOverlayStates(parameters.OverlayIds, parameters.AllOverlays, "ldraw_create_scene", allow_empty: true);
				var scene = m_model.CreateScene(parameters.SceneName, parameters.Activate);
				var changed = ApplyOverlayMembership(scene, ESceneSourceMode.Add, overlay_states, parameters.ResetView);
				return CreateSceneMutationResult("create_scene", scene.SceneName, CreateSceneInfo(scene), changed);
			}, TimeSpan.FromSeconds(10)).ConfigureAwait(false);
		}
		finally
		{
			m_overlay_gate.Release();
		}
	}
}

/// <summary>Create-scene command MCP tool surface</summary>
internal sealed partial class LDrawTools
{
	/// <summary>Create a new LDraw scene view</summary>
	[McpServerTool(Name = "ldraw_create_scene", Title = "Create LDraw scene", ReadOnly = false, Destructive = false, Idempotent = false, OpenWorld = false, UseStructuredContent = true)]
	[Description("Creates a new scene view. Optional overlay_ids can show existing MCP overlays in the new scene; user sources are not added automatically.")]
	public Task<LDrawSceneMutationResult> CreateScene(
		[Description("The instance id returned by ldraw_list_instances. Omit to target the broker instance.")] string? instance_id = null,
		[Description("Requested scene name. Omit to generate a unique scene name. Explicit duplicate names fail.")] string? scene_name = null,
		[Description("True to make the created scene active.")] bool activate = true,
		[Description("Existing MCP overlay ids to show in the new scene.")] string[]? overlay_ids = null,
		[Description("True to show all current MCP overlays in the new scene.")] bool all_overlays = false,
		[Description("True to frame the scene after adding overlay sources.")] bool reset_view = false)
	{
		var parameters = new LDrawCreateSceneParams
		{
			SceneName = scene_name,
			Activate = activate,
			OverlayIds = overlay_ids?.ToList() ?? [],
			AllOverlays = all_overlays,
			ResetView = reset_view,
		};
		return m_broker.CreateSceneAsync(instance_id, parameters);
	}
}
