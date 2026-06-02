using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Threading.Tasks;
using ModelContextProtocol.Server;

namespace LDraw.MCP;

/// <summary>List-scenes command broker routing</summary>
internal sealed partial class McpBroker
{
	/// <summary>Return scene summaries in 'instance_id'</summary>
	public async Task<LDrawSceneList> ListScenesAsync(string? instance_id, LDrawListScenesParams parameters)
	{
		var registration = ResolveInstance(instance_id);
		return await m_client.ListScenesAsync(registration, parameters).ConfigureAwait(false);
	}
}

/// <summary>List-scenes command MCP tool surface</summary>
internal sealed partial class LDrawTools
{
	/// <summary>List scene views in a running LDraw instance</summary>
	[McpServerTool(Name = "ldraw_list_scenes", Title = "List LDraw scenes", ReadOnly = true, Destructive = false, Idempotent = true, OpenWorld = false, UseStructuredContent = true)]
	[Description("Lists scene views, source memberships, visible bounds, active/visible state, and camera data. Omit scene_name to return all scenes.")]
	public Task<LDrawSceneList> ListScenes(
		[Description("The instance id returned by ldraw_list_instances. Omit to target the broker instance.")] string? instance_id = null,
		[Description("Optional scene name filter. Omit to list all scenes.")] string? scene_name = null)
	{
		var parameters = new LDrawListScenesParams
		{
			SceneName = scene_name,
		};
		return m_broker.ListScenesAsync(instance_id, parameters);
	}
}
