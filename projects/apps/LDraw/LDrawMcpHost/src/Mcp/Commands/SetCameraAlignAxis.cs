using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Threading.Tasks;
using ModelContextProtocol.Server;

namespace LDraw.MCP;

/// <summary>Set-camera-align-axis command broker routing</summary>
internal sealed partial class McpBroker
{
	/// <summary>Set camera align axis for a scene in 'instance_id'</summary>
	public async Task<LDrawViewMutationResult> SetCameraAlignAxisAsync(string? instance_id, LDrawSetCameraAlignAxisParams parameters)
	{
		var registration = await ResolveInstanceAsync(instance_id).ConfigureAwait(false);
		return await m_client.SetCameraAlignAxisAsync(registration, parameters).ConfigureAwait(false);
	}
}

/// <summary>Set-camera-align-axis command pipe client call</summary>
internal sealed partial class InstancePipeClient
{
	/// <summary>Set camera align axis for a scene in 'registration'</summary>
	public async Task<LDrawViewMutationResult> SetCameraAlignAxisAsync(InstanceRegistration registration, LDrawSetCameraAlignAxisParams parameters)
	{
		return await SendAsync<LDrawViewMutationResult>(registration, InstancePipeCommands.SetCameraAlignAxis, parameters, WriteTimeout).ConfigureAwait(false);
	}
}

/// <summary>Set-camera-align-axis command MCP tool surface</summary>
internal sealed partial class LDrawTools
{
	/// <summary>Set a scene camera up-axis alignment</summary>
	[McpServerTool(Name = "ldraw_set_camera_align_axis", Title = "Set LDraw camera align axis", ReadOnly = false, Destructive = false, Idempotent = true, OpenWorld = false, UseStructuredContent = true)]
	[Description("Sets the named camera up-axis alignment and persists the normal scene setting. Valid values: None, PosX, NegX, PosY, NegY, PosZ, NegZ.")]
	public Task<LDrawViewMutationResult> SetCameraAlignAxis(
		[Description("Named align direction: None, PosX, NegX, PosY, NegY, PosZ, or NegZ.")] string align_direction,
		[Description("The id of a running LDraw instance from ldraw_list_instances. Omit to target the default instance (most-recently-used, or auto-launched when none are running).")] string? instance_id = null,
		[Description("The scene name to modify. Omit to use the first scene.")] string? scene_name = null)
	{
		var parameters = new LDrawSetCameraAlignAxisParams
		{
			SceneName = scene_name,
			AlignDirection = align_direction,
		};
		return m_broker.SetCameraAlignAxisAsync(instance_id, parameters);
	}
}
