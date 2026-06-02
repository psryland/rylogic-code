using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Threading.Tasks;
using ModelContextProtocol.Server;

namespace LDraw.MCP;

/// <summary>Chart-add-series command broker routing</summary>
internal sealed partial class McpBroker
{
	/// <summary>Add a series to a generated chart in 'instance_id'</summary>
	public async Task<LDrawChartResult> ChartAddSeriesAsync(string? instance_id, LDrawChartAddSeriesParams parameters)
	{
		var registration = await ResolveInstanceAsync(instance_id).ConfigureAwait(false);
		return await m_client.ChartAddSeriesAsync(registration, parameters).ConfigureAwait(false);
	}
}

/// <summary>Chart-add-series command MCP tool surface</summary>
internal sealed partial class LDrawTools
{
	/// <summary>Add a series to an existing generated chart</summary>
	[McpServerTool(Name = "ldraw_chart_add_series", Title = "Add LDraw chart series", ReadOnly = false, Destructive = false, Idempotent = false, OpenWorld = false, UseStructuredContent = true)]
	[Description("Adds a series expression to an existing MCP-owned chart and reloads the same overlay.")]
	public Task<LDrawChartResult> ChartAddSeries(
		[Description("Y expression for the new series, e.g. C1 or abs(C2 - C1).")] string y_axis,
		[Description("The id of a running LDraw instance from ldraw_list_instances. Omit to target the default instance (most-recently-used, or auto-launched when none are running).")] string? instance_id = null,
		[Description("Chart id returned by ldraw_chart_create. Omit to use 'default'.")] string? chart_id = null,
		[Description("Series display name. Omit for seriesN.")] string? name = null,
		[Description("Series colour, e.g. FF00A0E0, #00A0E0, or Cyan.")] string? colour = null,
		[Description("X expression for the new series, e.g. C0 or CI. Omit to use C0.")] string? x_axis = null,
		[Description("Optional line width.")] double? width = null,
		[Description("True to smooth the line.")] bool smooth = false,
		[Description("True to frame the target scene after reloading the chart overlay.")] bool reset_view = false)
	{
		if (string.IsNullOrWhiteSpace(y_axis))
			throw new InvalidOperationException("y_axis is required.");

		return m_broker.ChartAddSeriesAsync(instance_id, new LDrawChartAddSeriesParams
		{
			ChartId = chart_id,
			Series = new LDrawChartSeriesInput
			{
				Name = name,
				Colour = colour,
				XAxis = x_axis,
				YAxis = y_axis,
				Width = width,
				Smooth = smooth,
			},
			ResetView = reset_view,
		});
	}
}
