using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Threading.Tasks;
using ModelContextProtocol.Server;

namespace LDraw.MCP;

/// <summary>Chart-clear command broker routing</summary>
internal sealed partial class McpBroker
{
	/// <summary>Clear one or all generated charts in 'instance_id'</summary>
	public async Task<LDrawChartResult> ChartClearAsync(string? instance_id, LDrawChartClearParams parameters)
	{
		var registration = await ResolveInstanceAsync(instance_id).ConfigureAwait(false);
		return await m_client.ChartClearAsync(registration, parameters).ConfigureAwait(false);
	}
}

/// <summary>Chart-clear command MCP tool surface</summary>
internal sealed partial class LDrawTools
{
	/// <summary>Clear one generated chart overlay, or all generated chart overlays</summary>
	[McpServerTool(Name = "ldraw_chart_clear", Title = "Clear LDraw chart", ReadOnly = false, Destructive = false, Idempotent = false, OpenWorld = false, UseStructuredContent = true)]
	[Description("Clears one MCP-owned generated chart overlay, or all generated chart overlays when chart_id is omitted. User source documents are not modified.")]
	public Task<LDrawChartResult> ChartClear(
		[Description("The id of a running LDraw instance from ldraw_list_instances. Omit to target the default instance (most-recently-used, or auto-launched when none are running).")] string? instance_id = null,
		[Description("Chart id returned by ldraw_chart_create. Omit to clear all generated MCP charts.")] string? chart_id = null)
	{
		return m_broker.ChartClearAsync(instance_id, new LDrawChartClearParams
		{
			ChartId = chart_id,
		});
	}
}
