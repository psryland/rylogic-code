using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Threading.Tasks;

namespace LDraw.MCP;

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
