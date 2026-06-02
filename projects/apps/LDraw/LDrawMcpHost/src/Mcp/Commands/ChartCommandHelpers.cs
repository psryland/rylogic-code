using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Threading.Tasks;
using ModelContextProtocol.Server;

namespace LDraw.MCP;

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
