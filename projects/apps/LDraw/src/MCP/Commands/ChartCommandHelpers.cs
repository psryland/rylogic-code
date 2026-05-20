using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Rylogic.Gfx;

namespace LDraw.MCP;

/// <summary>Shared helpers for generated chart MCP commands</summary>
internal sealed partial class LDrawInstanceHost
{
	/// <summary>Create chart state from an MCP create request</summary>
	private static ChartState CreateChartState(LDrawChartCreateParams parameters)
	{
		var chart_id = NormaliseChartId(parameters.ChartId);
		var (data_rows, columns) = ValidateDataRows(parameters.DataRows, "data_rows");
		var name = string.IsNullOrWhiteSpace(parameters.Name) ? chart_id : parameters.Name.Trim();
		var colour = ParseColour(string.IsNullOrWhiteSpace(parameters.Colour) ? "FFFFFFFF" : parameters.Colour);
		var series = parameters.Series.Count != 0
			? parameters.Series.Select((x, i) => CreateChartSeries(x, i)).ToList()
			: [CreateDefaultChartSeries(columns)];

		return new ChartState(chart_id, name, colour, data_rows, columns, series, [..parameters.SceneNames]);
	}

	/// <summary>Create one chart series from MCP input</summary>
	private static ChartSeriesState CreateChartSeries(LDrawChartSeriesInput input, int index)
	{
		var name = string.IsNullOrWhiteSpace(input.Name) ? $"series{index}" : input.Name.Trim();
		var colour = ParseColour(string.IsNullOrWhiteSpace(input.Colour) ? DefaultSeriesColour(index) : input.Colour);
		var x_axis = string.IsNullOrWhiteSpace(input.XAxis) ? "C0" : input.XAxis.Trim();
		var y_axis = string.IsNullOrWhiteSpace(input.YAxis) ? "C1" : input.YAxis.Trim();
		if (input.Width is { } width && (!double.IsFinite(width) || width <= 0.0))
			throw new InvalidOperationException("Series width must be a positive finite value.");

		return new ChartSeriesState(name, colour, x_axis, y_axis, input.Width, input.Smooth);
	}

	/// <summary>Create a default series suitable for the number of data columns</summary>
	private static ChartSeriesState CreateDefaultChartSeries(int columns)
	{
		return columns == 1
			? new ChartSeriesState("series0", ParseColour(DefaultSeriesColour(0)), "CI", "C0", null, false)
			: new ChartSeriesState("series0", ParseColour(DefaultSeriesColour(0)), "C0", "C1", null, false);
	}

	/// <summary>Return a stable default colour for a generated series</summary>
	private static string DefaultSeriesColour(int index)
	{
		var colours = new[]
		{
			"FF00A0E0",
			"FFA000E0",
			"FF00C060",
			"FFE0A000",
			"FFE04040",
			"FF8080FF",
		};
		return colours[Math.Abs(index) % colours.Length];
	}

	/// <summary>Validate inline chart data and return a deep copy plus the column count</summary>
	private static (List<List<double>> Rows, int Columns) ValidateDataRows(IReadOnlyList<List<double>> data_rows, string parameter_name)
	{
		if (data_rows.Count == 0)
			throw new InvalidOperationException($"{parameter_name} must contain at least one row.");
		if (data_rows.Count > 100000)
			throw new InvalidOperationException($"{parameter_name} cannot contain more than 100000 rows.");

		var columns = data_rows[0].Count;
		if (columns == 0)
			throw new InvalidOperationException($"{parameter_name} rows must contain at least one column.");
		if (columns > 256)
			throw new InvalidOperationException($"{parameter_name} rows cannot contain more than 256 columns.");

		var rows = new List<List<double>>(data_rows.Count);
		for (int row = 0; row != data_rows.Count; ++row)
		{
			if (data_rows[row].Count != columns)
				throw new InvalidOperationException($"{parameter_name} row {row} has {data_rows[row].Count} columns; expected {columns}.");

			var values = new List<double>(columns);
			for (int col = 0; col != columns; ++col)
			{
				var value = data_rows[row][col];
				if (!double.IsFinite(value))
					throw new InvalidOperationException($"{parameter_name} row {row}, column {col} must be finite.");
				values.Add(value);
			}
			rows.Add(values);
		}
		return (rows, columns);
	}

	/// <summary>Load the current chart state through the MCP overlay path</summary>
	private async Task<LDrawOverlayResult> LoadChartStateAsync(ChartState state, bool reset_view, string action)
	{
		var overlay = await SetOverlayScriptAsync(new LDrawOverlayScriptParams
		{
			OverlayId = state.OverlayId,
			Name = $"MCP Chart - {state.Name}",
			Script = BuildChartScript(state),
			SceneNames = state.SceneNames,
			ResetView = reset_view,
		}, append: false).ConfigureAwait(false);

		state.SceneNames = overlay.Overlay.SceneNames;
		overlay.Action = action;
		return overlay;
	}

	/// <summary>Build raw LDraw chart script from stored MCP chart state</summary>
	private static string BuildChartScript(ChartState state)
	{
		var script = new StringBuilder();
		script.AppendLine($"*Chart {SafeLdrIdentifier(state.ChartId)} {state.Colour}");
		script.AppendLine("{");
		script.AppendLine($"	*Dim {{{state.Columns}}}");
		script.AppendLine("	*Data");
		script.AppendLine("	{");
		foreach (var row in state.DataRows)
			script.AppendLine($"		{string.Join(" ", row.Select(FormatReal))}");
		script.AppendLine("	}");
		foreach (var series in state.Series)
		{
			script.AppendLine($"	*Series {SafeLdrIdentifier(series.Name)} {series.Colour}");
			script.AppendLine("	{");
			script.AppendLine($"		*XAxis {{{QuoteLdrString(series.XAxis)}}}");
			script.AppendLine($"		*YAxis {{{QuoteLdrString(series.YAxis)}}}");
			if (series.Width is { } width)
				script.AppendLine($"		*Width {{{FormatReal(width)}}}");
			if (series.Smooth)
				script.AppendLine("		*Smooth {}");
			script.AppendLine("	}");
		}
		script.AppendLine("}");
		return script.ToString();
	}

	/// <summary>Create public MCP chart info from stored state</summary>
	private static LDrawChartInfo CreateChartInfo(ChartState state, LDrawOverlayInfo overlay)
	{
		return new LDrawChartInfo
		{
			ChartId = state.ChartId,
			Name = state.Name,
			Colour = state.Colour.ToString(),
			Columns = state.Columns,
			Rows = state.DataRows.Count,
			Series = [..state.Series.Select(x => x.ToInfo())],
			SceneNames = [..state.SceneNames],
			Overlay = overlay,
		};
	}

	/// <summary>Get an existing chart state as a mutable clone</summary>
	private ChartState CloneChartState(string? chart_id, string command_name)
	{
		var id = NormaliseChartId(chart_id);
		lock (m_chart_lock)
		{
			if (!m_charts.TryGetValue(id, out var state))
				throw new InvalidOperationException($"{command_name} could not find generated chart '{id}'.");
			return state.Clone();
		}
	}

	/// <summary>Store successfully loaded chart state</summary>
	private void StoreChartState(ChartState state)
	{
		lock (m_chart_lock)
			m_charts[state.ChartId] = state.Clone();
	}

	/// <summary>Return chart states targeted by a clear command</summary>
	private ChartState[] ChartStatesForClear(string? chart_id)
	{
		lock (m_chart_lock)
		{
			if (string.IsNullOrWhiteSpace(chart_id))
				return [..m_charts.Values.Select(x => x.Clone())];

			var id = NormaliseChartId(chart_id);
			if (!m_charts.TryGetValue(id, out var state))
				throw new InvalidOperationException($"No generated MCP chart named '{id}' exists.");
			return [state.Clone()];
		}
	}

	/// <summary>Remove chart states after their overlays have been cleared</summary>
	private void RemoveChartStates(IEnumerable<ChartState> states)
	{
		lock (m_chart_lock)
		{
			foreach (var state in states)
				m_charts.Remove(state.ChartId);
		}
	}

	/// <summary>Remove chart states whose backing overlays were cleared through the generic overlay command path</summary>
	private void RemoveChartStatesForOverlayIds(IEnumerable<string> overlay_ids)
	{
		var ids = overlay_ids.ToHashSet(StringComparer.OrdinalIgnoreCase);
		lock (m_chart_lock)
		{
			foreach (var state in m_charts.Values.Where(state => ids.Contains(state.OverlayId)).ToArray())
				m_charts.Remove(state.ChartId);
		}
	}

	/// <summary>Normalise and validate a caller-visible chart id</summary>
	private static string NormaliseChartId(string? chart_id)
	{
		var id = string.IsNullOrWhiteSpace(chart_id) ? "default" : chart_id.Trim();
		if (id.Length > 48 || id.Any(char.IsControl))
			throw new InvalidOperationException("Chart id must be 1-48 non-control characters.");
		return id;
	}

	/// <summary>Return the MCP overlay id used for a chart id</summary>
	private static string ChartOverlayId(string chart_id)
	{
		return $"chart_{chart_id}";
	}

	/// <summary>Convert a display name into an LDraw identifier token</summary>
	private static string SafeLdrIdentifier(string name)
	{
		var id = new StringBuilder(name.Length + 1);
		foreach (var ch in name)
			id.Append(char.IsLetterOrDigit(ch) || ch == '_' ? ch : '_');
		if (id.Length == 0)
			id.Append("chart");
		if (char.IsDigit(id[0]))
			id.Insert(0, '_');
		return id.ToString();
	}

	/// <summary>Quote a string for raw LDraw script</summary>
	private static string QuoteLdrString(string value)
	{
		return $"\"{value.Replace("\\", "\\\\").Replace("\"", "\\\"")}\"";
	}

	/// <summary>Format a floating point value for generated LDraw script</summary>
	private static string FormatReal(double value)
	{
		return value.ToString("R", CultureInfo.InvariantCulture);
	}

	/// <summary>State for one generated MCP chart</summary>
	private sealed class ChartState
	{
		/// <summary>Create state for a generated MCP chart</summary>
		public ChartState(string chart_id, string name, Colour32 colour, List<List<double>> data_rows, int columns, List<ChartSeriesState> series, List<string> scene_names)
		{
			ChartId = chart_id;
			Name = name;
			Colour = colour;
			DataRows = data_rows;
			Columns = columns;
			Series = series;
			SceneNames = scene_names;
		}

		/// <summary>Caller-visible chart id</summary>
		public string ChartId { get; }

		/// <summary>The backing MCP overlay id</summary>
		public string OverlayId => ChartOverlayId(ChartId);

		/// <summary>Chart display name</summary>
		public string Name { get; set; }

		/// <summary>Chart object colour</summary>
		public Colour32 Colour { get; set; }

		/// <summary>Inline data rows</summary>
		public List<List<double>> DataRows { get; set; }

		/// <summary>Number of data columns</summary>
		public int Columns { get; set; }

		/// <summary>Series definitions</summary>
		public List<ChartSeriesState> Series { get; set; }

		/// <summary>Scene names currently showing the chart overlay</summary>
		public List<string> SceneNames { get; set; }

		/// <summary>Create a deep clone of this state</summary>
		public ChartState Clone()
		{
			return new ChartState(
				ChartId,
				Name,
				Colour,
				[..DataRows.Select(row => row.ToList())],
				Columns,
				[..Series.Select(series => series.Clone())],
				[..SceneNames]);
		}
	}

	/// <summary>State for one generated chart series</summary>
	private sealed class ChartSeriesState
	{
		/// <summary>Create state for one generated chart series</summary>
		public ChartSeriesState(string name, Colour32 colour, string x_axis, string y_axis, double? width, bool smooth)
		{
			Name = name;
			Colour = colour;
			XAxis = x_axis;
			YAxis = y_axis;
			Width = width;
			Smooth = smooth;
		}

		/// <summary>Series display name</summary>
		public string Name { get; set; }

		/// <summary>Series colour</summary>
		public Colour32 Colour { get; set; }

		/// <summary>X-axis expression</summary>
		public string XAxis { get; set; }

		/// <summary>Y-axis expression</summary>
		public string YAxis { get; set; }

		/// <summary>Optional line width</summary>
		public double? Width { get; set; }

		/// <summary>True to smooth the line</summary>
		public bool Smooth { get; set; }

		/// <summary>Create a deep clone of this series state</summary>
		public ChartSeriesState Clone()
		{
			return new ChartSeriesState(Name, Colour, XAxis, YAxis, Width, Smooth);
		}

		/// <summary>Create public MCP series info</summary>
		public LDrawChartSeriesInfo ToInfo()
		{
			return new LDrawChartSeriesInfo
			{
				Name = Name,
				Colour = Colour.ToString(),
				XAxis = XAxis,
				YAxis = YAxis,
				Width = Width,
				Smooth = Smooth,
			};
		}
	}
}

/// <summary>Shared argument helpers for chart MCP tools</summary>
internal sealed partial class LDrawTools
{
	/// <summary>Convert tool data rows into mutable DTO data</summary>
	private static List<List<double>> ChartDataRows(double[][] data_rows)
	{
		if (data_rows == null)
			throw new InvalidOperationException("data_rows is required.");
		return [..data_rows.Select(row => row?.ToList() ?? throw new InvalidOperationException("data_rows cannot contain null rows."))];
	}

	/// <summary>Create an optional first series definition from scalar tool arguments</summary>
	private static LDrawChartSeriesInput? InitialSeries(string? series_name, string? series_colour, string? x_axis, string? y_axis, double? width, bool smooth)
	{
		if (string.IsNullOrWhiteSpace(series_name) &&
			string.IsNullOrWhiteSpace(series_colour) &&
			string.IsNullOrWhiteSpace(x_axis) &&
			string.IsNullOrWhiteSpace(y_axis) &&
			width == null &&
			!smooth)
			return null;

		return new LDrawChartSeriesInput
		{
			Name = series_name,
			Colour = series_colour,
			XAxis = x_axis,
			YAxis = y_axis,
			Width = width,
			Smooth = smooth,
		};
	}
}
