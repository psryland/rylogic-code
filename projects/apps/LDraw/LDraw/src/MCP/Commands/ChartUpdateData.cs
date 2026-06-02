using System.ComponentModel;
using System.Threading.Tasks;

namespace LDraw.MCP;

/// <summary>Chart-update-data command instance implementation</summary>
internal sealed partial class LDrawInstanceHost
{
	/// <summary>Replace the inline data rows for an existing generated chart</summary>
	private async Task<LDrawChartResult> ChartUpdateDataAsync(LDrawChartUpdateDataParams parameters)
	{
		var state = CloneChartState(parameters.ChartId, "ldraw_chart_update_data");
		var (data_rows, columns) = ValidateDataRows(parameters.DataRows, "data_rows");
		state.DataRows = data_rows;
		state.Columns = columns;

		var overlay = await LoadChartStateAsync(state, parameters.ResetView, "chart_update_data").ConfigureAwait(false);
		StoreChartState(state);

		var chart = CreateChartInfo(state, overlay.Overlay);
		return new LDrawChartResult
		{
			Action = "chart_update_data",
			Chart = chart,
			Charts = [chart],
			Overlay = overlay.Overlay,
			Overlays = [overlay.Overlay],
		};
	}
}
