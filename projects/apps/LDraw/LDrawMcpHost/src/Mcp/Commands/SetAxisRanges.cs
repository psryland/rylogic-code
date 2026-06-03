using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Threading.Tasks;
using ModelContextProtocol.Server;

namespace LDraw.MCP;

/// <summary>Set-axis-ranges command broker routing</summary>
internal sealed partial class McpBroker
{
	/// <summary>Set chart axis ranges for a scene in 'instance_id'</summary>
	public async Task<LDrawAxisRangeResult> SetAxisRangesAsync(string? instance_id, LDrawSetAxisRangesParams parameters)
	{
		var registration = await ResolveInstanceAsync(instance_id).ConfigureAwait(false);
		return await m_client.SetAxisRangesAsync(registration, parameters).ConfigureAwait(false);
	}
}

/// <summary>Set-axis-ranges command MCP tool surface</summary>
internal sealed partial class LDrawTools
{
	/// <summary>Set the visible chart X/Y axis ranges for a scene</summary>
	[McpServerTool(Name = "ldraw_set_axis_ranges", Title = "Set LDraw axis ranges", ReadOnly = false, Destructive = false, Idempotent = true, OpenWorld = false, UseStructuredContent = true)]
	[Description("Sets one or both visible chart X/Y axis ranges for a scene, then updates the camera from those ranges. This is for chart-like views, not object bounds framing.")]
	public Task<LDrawAxisRangeResult> SetAxisRanges(
		[Description("The id of a running LDraw instance from ldraw_list_instances. Omit to target the default instance (most-recently-used, or auto-launched when none are running).")] string? instance_id = null,
		[Description("The scene name to modify. Omit to use the first scene.")] string? scene_name = null,
		[Description("Minimum X axis value. Supply with x_max.")] double? x_min = null,
		[Description("Maximum X axis value. Supply with x_min.")] double? x_max = null,
		[Description("Minimum Y axis value. Supply with y_max.")] double? y_min = null,
		[Description("Maximum Y axis value. Supply with y_min.")] double? y_max = null)
	{
		return m_broker.SetAxisRangesAsync(instance_id, new LDrawSetAxisRangesParams
		{
			SceneName = scene_name,
			XMin = x_min,
			XMax = x_max,
			YMin = y_min,
			YMax = y_max,
		});
	}
}
