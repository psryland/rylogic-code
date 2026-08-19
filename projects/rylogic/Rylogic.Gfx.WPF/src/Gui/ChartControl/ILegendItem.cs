using Rylogic.Gfx;

namespace Rylogic.Gui.WPF
{
	public interface IChartLegendItem
	{
		/// <summary>The name to display in the legend</summary>
		string Name { get; }

		/// <summary>Colour for the legend item</summary>
		Colour32 Colour { get; }

		/// <summary>True when the item is visible</summary>
		bool Visible { get; set; }
	}
}
