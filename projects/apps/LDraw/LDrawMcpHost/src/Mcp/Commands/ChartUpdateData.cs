using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Threading.Tasks;
using ModelContextProtocol.Server;

namespace LDraw.MCP;

/// <summary>Chart-update-data command broker routing</summary>
internal sealed partial class McpBroker
{
	/// <summary>Replace generated chart data in 'instance_id'</summary>
	public async Task<LDrawChartResult> ChartUpdateDataAsync(string? instance_id, LDrawChartUpdateDataParams parameters)
	{
		var registration = ResolveInstance(instance_id);
		return await m_client.ChartUpdateDataAsync(registration, parameters).ConfigureAwait(false);
	}
}

/// <summary>Chart-update-data command MCP tool surface</summary>
internal sealed partial class LDrawTools
{
	/// <summary>Replace the inline data rows for an existing generated chart</summary>
	[McpServerTool(Name = "ldraw_chart_update_data", Title = "Update LDraw chart data", ReadOnly = false, Destructive = false, Idempotent = true, OpenWorld = false, UseStructuredContent = true)]
	[Description("Replaces inline data rows for an MCP-owned chart and reloads the same overlay. The chart definition and series are preserved.")]
	public Task<LDrawChartResult> ChartUpdateData(
		[Description("Replacement inline numeric data rows. Each row must have the same number of finite values.")] double[][] data_rows,
		[Description("The instance id returned by ldraw_list_instances. Omit to target the broker instance.")] string? instance_id = null,
		[Description("Chart id returned by ldraw_chart_create. Omit to use 'default'.")] string? chart_id = null,
		[Description("True to frame the target scene after reloading the chart overlay.")] bool reset_view = false)
	{
		return m_broker.ChartUpdateDataAsync(instance_id, new LDrawChartUpdateDataParams
		{
			ChartId = chart_id,
			DataRows = ChartDataRows(data_rows),
			ResetView = reset_view,
		});
	}
}
