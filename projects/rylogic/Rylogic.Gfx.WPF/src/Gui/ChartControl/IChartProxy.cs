namespace Rylogic.Gui.WPF
{
	public interface IChartProxy
	{
		/// <summary>A string identifier for the chart</summary>
		string Name { get; }

		/// <summary>Access to the wrapped chart instance</summary>
		ChartControl? Chart { get; }
	}
}
