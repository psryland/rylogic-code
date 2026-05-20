using System.ComponentModel;
using System.Linq;
using System.Threading.Tasks;
using ModelContextProtocol.Server;

namespace LDraw.MCP;

/// <summary>Chart-set-axis-ranges command broker routing</summary>
internal sealed partial class McpBroker
{
	/// <summary>Set chart axis ranges in 'instance_id'</summary>
	public async Task<LDrawAxisRangeResult> ChartSetAxisRangesAsync(string? instance_id, LDrawChartSetAxisRangesParams parameters)
	{
		var registration = ResolveInstance(instance_id);
		return await m_client.ChartSetAxisRangesAsync(registration, parameters).ConfigureAwait(false);
	}
}

/// <summary>Chart-set-axis-ranges command instance implementation</summary>
internal sealed partial class LDrawInstanceHost
{
	/// <summary>Set one or both visible axis ranges for the scene containing an existing generated chart</summary>
	private Task<LDrawAxisRangeResult> ChartSetAxisRangesAsync(LDrawChartSetAxisRangesParams parameters)
	{
		var state = CloneChartState(parameters.ChartId, "ldraw_chart_set_axis_ranges");
		parameters.SceneName ??= state.SceneNames.FirstOrDefault();
		return SetAxisRangesAsync(parameters, "chart_set_axis_ranges");
	}
}

/// <summary>Chart-set-axis-ranges command MCP tool surface</summary>
internal sealed partial class LDrawTools
{
	/// <summary>Set visible axis ranges for a generated chart scene</summary>
	[McpServerTool(Name = "ldraw_chart_set_axis_ranges", Title = "Set LDraw chart axis ranges", ReadOnly = false, Destructive = false, Idempotent = true, OpenWorld = false, UseStructuredContent = true)]
	[Description("Sets one or both visible X/Y axis ranges for the scene containing an MCP-owned chart. The raw chart script is unchanged.")]
	public Task<LDrawAxisRangeResult> ChartSetAxisRanges(
		[Description("The instance id returned by ldraw_list_instances. Omit to target the broker instance.")] string? instance_id = null,
		[Description("Chart id returned by ldraw_chart_create. Omit to use 'default'.")] string? chart_id = null,
		[Description("The scene name to modify. Omit to use the first scene that contains the chart.")] string? scene_name = null,
		[Description("Minimum X axis value. Supply with x_max.")] double? x_min = null,
		[Description("Maximum X axis value. Supply with x_min.")] double? x_max = null,
		[Description("Minimum Y axis value. Supply with y_max.")] double? y_min = null,
		[Description("Maximum Y axis value. Supply with y_min.")] double? y_max = null)
	{
		return m_broker.ChartSetAxisRangesAsync(instance_id, new LDrawChartSetAxisRangesParams
		{
			ChartId = chart_id,
			SceneName = scene_name,
			XMin = x_min,
			XMax = x_max,
			YMin = y_min,
			YMax = y_max,
		});
	}
}
