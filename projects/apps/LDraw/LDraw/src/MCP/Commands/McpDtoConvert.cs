using Rylogic.Gui.WPF;

namespace LDraw.MCP;

/// <summary>
/// LDraw-side converters that build MCP contract DTOs from WPF/View3D runtime types.
/// These live in the LDraw assembly (not in LDraw.Mcp.Contracts) so the shared
/// contracts library stays free of WPF and View3D dependencies.</summary>
internal static class McpDtoConvert
{
	/// <summary>Create MCP axis range data from a ChartControl axis</summary>
	public static LDrawAxisRangeInfo AxisRange(ChartControl.RangeData.Axis axis)
	{
		return new LDrawAxisRangeInfo
		{
			Min = axis.Min,
			Max = axis.Max,
			Span = axis.Span,
		};
	}
}
