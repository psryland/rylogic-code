using System;
using System.ComponentModel;
using System.Linq;
using System.Threading.Tasks;

namespace LDraw.MCP;

/// <summary>Chart-create command instance implementation</summary>
internal sealed partial class LDrawInstanceHost
{
	/// <summary>Create or replace a generated chart overlay from inline data</summary>
	private async Task<LDrawChartResult> ChartCreateAsync(LDrawChartCreateParams parameters)
	{
		var state = CreateChartState(parameters);
		var overlay = await LoadChartStateAsync(state, parameters.ResetView, "chart_create").ConfigureAwait(false);
		StoreChartState(state);

		var chart = CreateChartInfo(state, overlay.Overlay);
		return new LDrawChartResult
		{
			Action = "chart_create",
			Chart = chart,
			Charts = [chart],
			Overlay = overlay.Overlay,
			Overlays = [overlay.Overlay],
		};
	}
}
