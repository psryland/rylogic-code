using System;
using System.ComponentModel;
using System.Threading.Tasks;
using LDraw.UI;

namespace LDraw.MCP;

/// <summary>Set-axis-ranges command instance implementation</summary>
internal sealed partial class LDrawInstanceHost
{
	/// <summary>Set one or both visible chart axis ranges for the requested scene</summary>
	private Task<LDrawAxisRangeResult> SetAxisRangesAsync(LDrawSetAxisRangesParams parameters, string action)
	{
		return m_model.InvokeAsync(() =>
		{
			var scene = ResolveScene(parameters.SceneName);
			return ApplyAxisRanges(scene, parameters, action);
		}, TimeSpan.FromSeconds(5));
	}

	/// <summary>Apply one or both visible chart axis ranges to 'scene'</summary>
	private static LDrawAxisRangeResult ApplyAxisRanges(SceneUI scene, LDrawSetAxisRangesParams parameters, string action)
	{
		var set_x = HasAxisRange(parameters.XMin, parameters.XMax, "x_min", "x_max");
		var set_y = HasAxisRange(parameters.YMin, parameters.YMax, "y_min", "y_max");
		if (!set_x && !set_y)
			throw new InvalidOperationException("At least one complete axis range is required.");

		if (set_x)
		{
			ValidateAxisRange(parameters.XMin!.Value, parameters.XMax!.Value, "X");
			scene.SceneView.XAxis.Set(parameters.XMin.Value, parameters.XMax.Value);
		}
		if (set_y)
		{
			ValidateAxisRange(parameters.YMin!.Value, parameters.YMax!.Value, "Y");
			scene.SceneView.YAxis.Set(parameters.YMin.Value, parameters.YMax.Value);
		}

		scene.SceneView.SetCameraFromRange();
		scene.SceneView.Invalidate();
		return new LDrawAxisRangeResult
		{
			Action = action,
			SceneName = scene.SceneName,
			XAxisRange = McpDtoConvert.AxisRange(scene.SceneView.XAxis),
			YAxisRange = McpDtoConvert.AxisRange(scene.SceneView.YAxis),
			Camera = CreateCameraInfo(scene),
		};
	}

	/// <summary>Return true if a complete axis range is supplied, throwing for partial ranges</summary>
	private static bool HasAxisRange(double? min, double? max, string min_name, string max_name)
	{
		if (min.HasValue != max.HasValue)
			throw new InvalidOperationException($"{min_name} and {max_name} must be supplied together.");
		return min.HasValue;
	}

	/// <summary>Validate an explicit axis range</summary>
	private static void ValidateAxisRange(double min, double max, string axis_name)
	{
		if (!double.IsFinite(min) || !double.IsFinite(max))
			throw new InvalidOperationException($"{axis_name} axis range values must be finite.");
		if (min >= max)
			throw new InvalidOperationException($"{axis_name} axis range minimum must be less than the maximum.");
	}
}
