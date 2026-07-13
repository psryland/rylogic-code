using System.ComponentModel;
using System.Linq;
using System.Threading.Tasks;

namespace LDraw.MCP;

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
