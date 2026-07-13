using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Threading.Tasks;
using ModelContextProtocol.Server;

namespace LDraw.MCP;

/// <summary>Frame-selection command broker routing</summary>
internal sealed partial class McpBroker
{
	/// <summary>Frame selected objects in 'instance_id'</summary>
	public async Task<LDrawFrameResult> FrameSelectionAsync(string? instance_id, LDrawFrameSelectionParams parameters)
	{
		var registration = await ResolveInstanceAsync(instance_id).ConfigureAwait(false);
		return await m_client.FrameSelectionAsync(registration, parameters).ConfigureAwait(false);
	}
}

/// <summary>Frame-selection command pipe client call</summary>
internal sealed partial class InstancePipeClient
{
	/// <summary>Frame selected objects in 'registration'</summary>
	public async Task<LDrawFrameResult> FrameSelectionAsync(InstanceRegistration registration, LDrawFrameSelectionParams parameters)
	{
		return await SendAsync<LDrawFrameResult>(registration, InstancePipeCommands.FrameSelection, parameters, WriteTimeout).ConfigureAwait(false);
	}
}

/// <summary>Frame-selection command MCP tool surface</summary>
internal sealed partial class LDrawTools
{
	/// <summary>Frame the currently selected objects in a running LDraw scene</summary>
	[McpServerTool(Name = "ldraw_frame_selection", Title = "Frame LDraw selection", ReadOnly = false, Destructive = false, Idempotent = true, OpenWorld = false, UseStructuredContent = true)]
	[Description("Frames the current selected objects. Fails if the selection has no valid bounds.")]
	public Task<LDrawFrameResult> FrameSelection(
		[Description("The id of a running LDraw instance from ldraw_list_instances. Omit to target the default instance (most-recently-used, or auto-launched when none are running).")] string? instance_id = null,
		[Description("The scene name to modify. Omit to use the first scene.")] string? scene_name = null,
		[Description("Maximum number of selected objects to include in the response, clamped to 1..1000.")] int max_count = 200)
	{
		var parameters = new LDrawFrameSelectionParams
		{
			SceneName = scene_name,
			MaxCount = max_count,
		};
		return m_broker.FrameSelectionAsync(instance_id, parameters);
	}
}
