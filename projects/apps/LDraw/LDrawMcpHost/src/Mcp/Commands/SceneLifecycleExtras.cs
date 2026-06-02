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
		var registration = await ResolveInstanceAsync(instance_id).ConfigureAwait(false);
		return await m_client.ClearSceneAsync(registration, parameters).ConfigureAwait(false);
	}

	/// <summary>Rename a scene in 'instance_id'</summary>
	public async Task<LDrawSceneMutationResult> RenameSceneAsync(string? instance_id, LDrawRenameSceneParams parameters)
	{
		var registration = await ResolveInstanceAsync(instance_id).ConfigureAwait(false);
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

/// <summary>Additional scene lifecycle command MCP tool surface</summary>
internal sealed partial class LDrawTools
{
	/// <summary>Clear all current objects from a scene</summary>
	[McpServerTool(Name = "ldraw_clear_scene", Title = "Clear LDraw scene", ReadOnly = false, Destructive = true, Idempotent = true, OpenWorld = false, UseStructuredContent = true)]
	[Description("Removes all currently rendered objects from a scene. Source membership is left unchanged, so later reloads or membership commands can add objects again.")]
	public Task<LDrawSceneMutationResult> ClearScene(
		[Description("The scene name to clear. Required.")] string scene_name,
		[Description("The id of a running LDraw instance from ldraw_list_instances. Omit to target the default instance (most-recently-used, or auto-launched when none are running).")] string? instance_id = null)
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
		[Description("The id of a running LDraw instance from ldraw_list_instances. Omit to target the default instance (most-recently-used, or auto-launched when none are running).")] string? instance_id = null)
	{
		var parameters = new LDrawRenameSceneParams
		{
			SceneName = scene_name,
			NewSceneName = new_scene_name,
		};
		return m_broker.RenameSceneAsync(instance_id, parameters);
	}
}
