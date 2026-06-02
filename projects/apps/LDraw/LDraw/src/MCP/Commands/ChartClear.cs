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
		var registration = ResolveInstance(instance_id);
		return await m_client.ChartClearAsync(registration, parameters).ConfigureAwait(false);
	}
}

/// <summary>Chart-clear command instance implementation</summary>
internal sealed partial class LDrawInstanceHost
{
	/// <summary>Clear one generated chart overlay, or all generated chart overlays</summary>
	private async Task<LDrawChartResult> ChartClearAsync(LDrawChartClearParams parameters)
	{
		var states = ChartStatesForClear(parameters.ChartId);
		if (states.Length == 0)
		{
			return new LDrawChartResult
			{
				Action = "chart_clear",
			};
		}

		var charts = new List<LDrawChartInfo>(states.Length);
		var overlays = new List<LDrawOverlayInfo>(states.Length);
		foreach (var state in states)
		{
			var results = await ClearOverlayAsync(new LDrawOverlayClearParams { OverlayId = state.OverlayId }).ConfigureAwait(false);
			var overlay = results.Single().Overlay;
			overlays.Add(overlay);
			charts.Add(CreateChartInfo(state, overlay));
		}
		RemoveChartStates(states);

		return new LDrawChartResult
		{
			Action = "chart_clear",
			Chart = charts.Count == 1 ? charts[0] : null,
			Charts = charts,
			Overlay = overlays.Count == 1 ? overlays[0] : null,
			Overlays = overlays,
		};
	}
}

/// <summary>Chart-clear command MCP tool surface</summary>
internal sealed partial class LDrawTools
{
	/// <summary>Clear one generated chart overlay, or all generated chart overlays</summary>
	[McpServerTool(Name = "ldraw_chart_clear", Title = "Clear LDraw chart", ReadOnly = false, Destructive = false, Idempotent = false, OpenWorld = false, UseStructuredContent = true)]
	[Description("Clears one MCP-owned generated chart overlay, or all generated chart overlays when chart_id is omitted. User source documents are not modified.")]
	public Task<LDrawChartResult> ChartClear(
		[Description("The instance id returned by ldraw_list_instances. Omit to target the broker instance.")] string? instance_id = null,
		[Description("Chart id returned by ldraw_chart_create. Omit to clear all generated MCP charts.")] string? chart_id = null)
	{
		return m_broker.ChartClearAsync(instance_id, new LDrawChartClearParams
		{
			ChartId = chart_id,
		});
	}
}
