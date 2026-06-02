using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Threading.Tasks;
using ModelContextProtocol.Server;

namespace LDraw.MCP;

/// <summary>Chart-create command broker routing</summary>
internal sealed partial class McpBroker
{
	/// <summary>Create or replace a generated chart in 'instance_id'</summary>
	public async Task<LDrawChartResult> ChartCreateAsync(string? instance_id, LDrawChartCreateParams parameters)
	{
		var registration = ResolveInstance(instance_id);
		return await m_client.ChartCreateAsync(registration, parameters).ConfigureAwait(false);
	}
}

/// <summary>Chart-create command MCP tool surface</summary>
internal sealed partial class LDrawTools
{
	/// <summary>Create or replace a generated chart overlay from inline rows</summary>
	[McpServerTool(Name = "ldraw_chart_create", Title = "Create LDraw chart", ReadOnly = false, Destructive = false, Idempotent = true, OpenWorld = false, UseStructuredContent = true)]
	[Description("Creates or replaces an MCP-owned chart overlay from inline numeric rows. User source documents are not modified. Omit series to plot C0 vs C1, or CI vs C0 for one-column data.")]
	public Task<LDrawChartResult> ChartCreate(
		[Description("Inline numeric data rows. Each row must have the same number of finite values.")] double[][] data_rows,
		[Description("The instance id returned by ldraw_list_instances. Omit to target the broker instance.")] string? instance_id = null,
		[Description("Chart id to create or replace. Omit to use 'default'. Reusing an id replaces the prior generated chart overlay.")] string? chart_id = null,
		[Description("Display name for the chart and generated overlay source.")] string? name = null,
		[Description("Chart colour, e.g. FFFFFFFF, #FFFFFF, or White.")] string? colour = null,
		[Description("Scene names to show the chart in. Omit to use the first scene. Use '*' as the only value for all scenes.")] string[]? scene_names = null,
		[Description("Optional full series definitions. Prefer ldraw_chart_add_series for additional series after creation.")] LDrawChartSeriesInput[]? series = null,
		[Description("Optional initial series name when not passing series definitions.")] string? series_name = null,
		[Description("Optional initial series colour when not passing series definitions.")] string? series_colour = null,
		[Description("Optional initial X expression, e.g. C0 or CI, when not passing series definitions.")] string? x_axis = null,
		[Description("Optional initial Y expression, e.g. C1, when not passing series definitions.")] string? y_axis = null,
		[Description("Optional initial series line width.")] double? width = null,
		[Description("True to smooth the initial series line.")] bool smooth = false,
		[Description("True to frame the target scene after loading the chart overlay.")] bool reset_view = true)
	{
		var series_list = series?.ToList() ?? [];
		var initial_series = InitialSeries(series_name, series_colour, x_axis, y_axis, width, smooth);
		if (series_list.Count != 0 && initial_series != null)
			throw new InvalidOperationException("Use either series definitions or scalar initial series arguments, not both.");
		if (initial_series != null)
			series_list.Add(initial_series);

		return m_broker.ChartCreateAsync(instance_id, new LDrawChartCreateParams
		{
			ChartId = chart_id,
			Name = name,
			Colour = colour,
			DataRows = ChartDataRows(data_rows),
			Series = series_list,
			SceneNames = scene_names?.ToList() ?? [],
			ResetView = reset_view,
		});
	}
}
