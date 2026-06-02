using System;
using System.ComponentModel;
using System.Threading.Tasks;
using LDraw.UI;
using ModelContextProtocol.Server;

namespace LDraw.MCP;

/// <summary>Set-axis-ranges command broker routing</summary>
internal sealed partial class McpBroker
{
	/// <summary>Set chart axis ranges for a scene in 'instance_id'</summary>
	public async Task<LDrawAxisRangeResult> SetAxisRangesAsync(string? instance_id, LDrawSetAxisRangesParams parameters)
	{
		var registration = ResolveInstance(instance_id);
		return await m_client.SetAxisRangesAsync(registration, parameters).ConfigureAwait(false);
	}
}

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

/// <summary>Set-axis-ranges command MCP tool surface</summary>
internal sealed partial class LDrawTools
{
	/// <summary>Set the visible chart X/Y axis ranges for a scene</summary>
	[McpServerTool(Name = "ldraw_set_axis_ranges", Title = "Set LDraw axis ranges", ReadOnly = false, Destructive = false, Idempotent = true, OpenWorld = false, UseStructuredContent = true)]
	[Description("Sets one or both visible chart X/Y axis ranges for a scene, then updates the camera from those ranges. This is for chart-like views, not object bounds framing.")]
	public Task<LDrawAxisRangeResult> SetAxisRanges(
		[Description("The instance id returned by ldraw_list_instances. Omit to target the broker instance.")] string? instance_id = null,
		[Description("The scene name to modify. Omit to use the first scene.")] string? scene_name = null,
		[Description("Minimum X axis value. Supply with x_max.")] double? x_min = null,
		[Description("Maximum X axis value. Supply with x_min.")] double? x_max = null,
		[Description("Minimum Y axis value. Supply with y_max.")] double? y_min = null,
		[Description("Maximum Y axis value. Supply with y_min.")] double? y_max = null)
	{
		return m_broker.SetAxisRangesAsync(instance_id, new LDrawSetAxisRangesParams
		{
			SceneName = scene_name,
			XMin = x_min,
			XMax = x_max,
			YMin = y_min,
			YMax = y_max,
		});
	}
}
