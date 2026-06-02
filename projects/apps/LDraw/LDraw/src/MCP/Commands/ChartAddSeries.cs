using System;
using System.ComponentModel;
using System.Threading.Tasks;

namespace LDraw.MCP;

/// <summary>Chart-add-series command instance implementation</summary>
internal sealed partial class LDrawInstanceHost
{
	/// <summary>Add a series to an existing generated chart</summary>
	private async Task<LDrawChartResult> ChartAddSeriesAsync(LDrawChartAddSeriesParams parameters)
	{
		var state = CloneChartState(parameters.ChartId, "ldraw_chart_add_series");
		state.Series.Add(CreateChartSeries(parameters.Series, state.Series.Count));

		var overlay = await LoadChartStateAsync(state, parameters.ResetView, "chart_add_series").ConfigureAwait(false);
		StoreChartState(state);

		var chart = CreateChartInfo(state, overlay.Overlay);
		return new LDrawChartResult
		{
			Action = "chart_add_series",
			Chart = chart,
			Charts = [chart],
			Overlay = overlay.Overlay,
			Overlays = [overlay.Overlay],
		};
	}
}
