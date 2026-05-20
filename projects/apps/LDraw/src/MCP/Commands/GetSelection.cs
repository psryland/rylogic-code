using System;
using System.ComponentModel;
using System.Threading.Tasks;
using ModelContextProtocol.Server;

namespace LDraw.MCP;

/// <summary>Get-selection command broker routing</summary>
internal sealed partial class McpBroker
{
	/// <summary>Return selected objects in 'instance_id'</summary>
	public async Task<LDrawSelectionResult> GetSelectionAsync(string? instance_id, LDrawGetSelectionParams parameters)
	{
		var registration = ResolveInstance(instance_id);
		return await m_client.GetSelectionAsync(registration, parameters).ConfigureAwait(false);
	}
}

/// <summary>Get-selection command pipe client call</summary>
internal sealed partial class InstancePipeClient
{
	/// <summary>Read selected objects from 'registration'</summary>
	public async Task<LDrawSelectionResult> GetSelectionAsync(InstanceRegistration registration, LDrawGetSelectionParams parameters)
	{
		return await SendAsync<LDrawSelectionResult>(registration, InstancePipeCommands.GetSelection, parameters, ReadTimeout).ConfigureAwait(false);
	}
}

/// <summary>Get-selection command instance implementation</summary>
internal sealed partial class LDrawInstanceHost
{
	/// <summary>Return selected objects and selected bounds from the requested scene</summary>
	private Task<LDrawSelectionResult> GetSelectionAsync(LDrawGetSelectionParams parameters)
	{
		return m_model.InvokeAsync(() =>
		{
			var scene = ResolveScene(parameters.SceneName);

			// Selection is represented by existing View3D selected flags so this read does not need Object Manager UI coupling.
			return CreateSelectionResult(scene, "get", parameters.IncludeChildren, parameters.MaxCount);
		}, TimeSpan.FromSeconds(5));
	}
}

/// <summary>Get-selection command MCP tool surface</summary>
internal sealed partial class LDrawTools
{
	/// <summary>Return selected objects in a running LDraw scene</summary>
	[McpServerTool(Name = "ldraw_get_selection", Title = "Get LDraw selection", ReadOnly = true, Destructive = false, Idempotent = true, OpenWorld = false, UseStructuredContent = true)]
	[Description("Returns selected objects and selected bounds. Omit scene_name to use the first scene.")]
	public Task<LDrawSelectionResult> GetSelection(
		[Description("The instance id returned by ldraw_list_instances. Omit to target the broker instance.")] string? instance_id = null,
		[Description("The scene name to query. Omit to use the first scene.")] string? scene_name = null,
		[Description("True to recursively include child objects when enumerating selected descendants.")] bool include_children = true,
		[Description("Maximum number of selected objects to return, clamped to 1..1000.")] int max_count = 200)
	{
		var parameters = new LDrawGetSelectionParams
		{
			SceneName = scene_name,
			IncludeChildren = include_children,
			MaxCount = max_count,
		};
		return m_broker.GetSelectionAsync(instance_id, parameters);
	}
}
