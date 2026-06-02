using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Threading.Tasks;
using ModelContextProtocol.Server;

namespace LDraw.MCP;

/// <summary>Render and display command broker routing</summary>
internal sealed partial class McpBroker
{
	/// <summary>Set runtime render settings in 'instance_id'</summary>
	public async Task<LDrawViewMutationResult> SetRenderSettingsAsync(string? instance_id, LDrawSetRenderSettingsParams parameters)
	{
		var registration = ResolveInstance(instance_id);
		return await m_client.SetRenderSettingsAsync(registration, parameters).ConfigureAwait(false);
	}

	/// <summary>Capture a scene image in 'instance_id'</summary>
	public async Task<LDrawSceneCaptureResult> CaptureSceneAsync(string? instance_id, LDrawCaptureSceneParams parameters)
	{
		var registration = ResolveInstance(instance_id);
		return await m_client.CaptureSceneAsync(registration, parameters).ConfigureAwait(false);
	}

	/// <summary>Return chart display options in 'instance_id'</summary>
	public async Task<LDrawChartDisplayOptionsInfo> GetChartDisplayOptionsAsync(string? instance_id, LDrawChartDisplayOptionsParams parameters)
	{
		var registration = ResolveInstance(instance_id);
		return await m_client.GetChartDisplayOptionsAsync(registration, parameters).ConfigureAwait(false);
	}

	/// <summary>Set chart display options in 'instance_id'</summary>
	public async Task<LDrawChartDisplayOptionsResult> SetChartDisplayOptionsAsync(string? instance_id, LDrawSetChartDisplayOptionsParams parameters)
	{
		var registration = ResolveInstance(instance_id);
		return await m_client.SetChartDisplayOptionsAsync(registration, parameters).ConfigureAwait(false);
	}
}

/// <summary>Render and display command pipe client calls</summary>
internal sealed partial class InstancePipeClient
{
	/// <summary>Set runtime render settings in 'registration'</summary>
	public async Task<LDrawViewMutationResult> SetRenderSettingsAsync(InstanceRegistration registration, LDrawSetRenderSettingsParams parameters)
	{
		return await SendAsync<LDrawViewMutationResult>(registration, InstancePipeCommands.SetRenderSettings, parameters, WriteTimeout).ConfigureAwait(false);
	}

	/// <summary>Capture a scene image in 'registration'</summary>
	public async Task<LDrawSceneCaptureResult> CaptureSceneAsync(InstanceRegistration registration, LDrawCaptureSceneParams parameters)
	{
		return await SendAsync<LDrawSceneCaptureResult>(registration, InstancePipeCommands.CaptureScene, parameters, WriteTimeout).ConfigureAwait(false);
	}

	/// <summary>Return chart display options in 'registration'</summary>
	public async Task<LDrawChartDisplayOptionsInfo> GetChartDisplayOptionsAsync(InstanceRegistration registration, LDrawChartDisplayOptionsParams parameters)
	{
		return await SendAsync<LDrawChartDisplayOptionsInfo>(registration, InstancePipeCommands.GetChartDisplayOptions, parameters, WriteTimeout).ConfigureAwait(false);
	}

	/// <summary>Set chart display options in 'registration'</summary>
	public async Task<LDrawChartDisplayOptionsResult> SetChartDisplayOptionsAsync(InstanceRegistration registration, LDrawSetChartDisplayOptionsParams parameters)
	{
		return await SendAsync<LDrawChartDisplayOptionsResult>(registration, InstancePipeCommands.SetChartDisplayOptions, parameters, WriteTimeout).ConfigureAwait(false);
	}
}

/// <summary>Render and display command MCP tool surface</summary>
internal sealed partial class LDrawTools
{
	/// <summary>Set runtime render settings for a running LDraw scene</summary>
	[McpServerTool(Name = "ldraw_set_render_settings", Title = "Set LDraw render settings", ReadOnly = false, Destructive = false, Idempotent = true, OpenWorld = false, UseStructuredContent = true)]
	[Description("Sets runtime render settings such as antialiasing, shadow cast range, and ray tracing. These changes do not edit user source files.")]
	public Task<LDrawViewMutationResult> SetRenderSettings(
		[Description("Optional multi-sample anti-aliasing state.")] bool? antialiasing = null,
		[Description("Optional shadow cast range. Zero disables shadow casting.")] double? shadow_cast_range = null,
		[Description("Optional ray tracing enabled state. Requires ray tracing support when true.")] bool? ray_tracing_enabled = null,
		[Description("The instance id returned by ldraw_list_instances. Omit to target the broker instance.")] string? instance_id = null,
		[Description("The scene name to modify. Omit to use the first scene.")] string? scene_name = null)
	{
		var parameters = new LDrawSetRenderSettingsParams
		{
			SceneName = scene_name,
			Antialiasing = antialiasing,
			ShadowCastRange = shadow_cast_range,
			RayTracingEnabled = ray_tracing_enabled,
		};
		return m_broker.SetRenderSettingsAsync(instance_id, parameters);
	}

	/// <summary>Capture a running LDraw scene to an image file</summary>
	[McpServerTool(Name = "ldraw_capture_scene", Title = "Capture LDraw scene", ReadOnly = false, Destructive = false, Idempotent = false, OpenWorld = true, UseStructuredContent = true)]
	[Description("Renders a running scene and writes a PNG, JPG, or BMP image. Omit output_path to write a timestamped PNG under the temp directory.")]
	public Task<LDrawSceneCaptureResult> CaptureScene(
		[Description("Output image path. The extension must be .png, .jpg, or .bmp. Omit for a temp PNG.")] string? output_path = null,
		[Description("True to overwrite an existing output file.")] bool overwrite = false,
		[Description("The instance id returned by ldraw_list_instances. Omit to target the broker instance.")] string? instance_id = null,
		[Description("The scene name to capture. Omit to use the first scene.")] string? scene_name = null)
	{
		var parameters = new LDrawCaptureSceneParams
		{
			SceneName = scene_name,
			OutputPath = output_path,
			Overwrite = overwrite,
		};
		return m_broker.CaptureSceneAsync(instance_id, parameters);
	}

	/// <summary>Return chart display options for a running LDraw scene</summary>
	[McpServerTool(Name = "ldraw_get_chart_display_options", Title = "Get LDraw chart display options", ReadOnly = true, Destructive = false, Idempotent = true, OpenWorld = false, UseStructuredContent = true)]
	[Description("Returns runtime chart display options for axes, grid lines, tick marks, and tick labels.")]
	public Task<LDrawChartDisplayOptionsInfo> GetChartDisplayOptions(
		[Description("The instance id returned by ldraw_list_instances. Omit to target the broker instance.")] string? instance_id = null,
		[Description("The scene name to query. Omit to use the first scene.")] string? scene_name = null)
	{
		var parameters = new LDrawChartDisplayOptionsParams
		{
			SceneName = scene_name,
		};
		return m_broker.GetChartDisplayOptionsAsync(instance_id, parameters);
	}

	/// <summary>Set chart display options for a running LDraw scene</summary>
	[McpServerTool(Name = "ldraw_set_chart_display_options", Title = "Set LDraw chart display options", ReadOnly = false, Destructive = false, Idempotent = true, OpenWorld = false, UseStructuredContent = true)]
	[Description("Sets runtime chart display options for axes, grid lines, tick marks, and tick labels. Profile-backed chart options are copied before mutation.")]
	public Task<LDrawChartDisplayOptionsResult> SetChartDisplayOptions(
		[Description("Optional axes visibility state.")] bool? show_axes = null,
		[Description("Optional grid-line visibility state applied to X and Y axes.")] bool? show_grid_lines = null,
		[Description("Optional tick-mark visibility state applied to X and Y axes.")] bool? draw_tick_marks = null,
		[Description("Optional tick-label visibility state applied to X and Y axes.")] bool? draw_tick_labels = null,
		[Description("Optional preferred tick spacing in pixels, applied to X and Y axes.")] double? pixels_per_tick = null,
		[Description("Optional axis line colour, applied to X and Y axes.")] string? axis_colour = null,
		[Description("Optional tick text colour, applied to X and Y axes.")] string? tick_colour = null,
		[Description("The instance id returned by ldraw_list_instances. Omit to target the broker instance.")] string? instance_id = null,
		[Description("The scene name to modify. Omit to use the first scene.")] string? scene_name = null)
	{
		var parameters = new LDrawSetChartDisplayOptionsParams
		{
			SceneName = scene_name,
			ShowAxes = show_axes,
			ShowGridLines = show_grid_lines,
			DrawTickMarks = draw_tick_marks,
			DrawTickLabels = draw_tick_labels,
			PixelsPerTick = pixels_per_tick,
			AxisColour = axis_colour,
			TickColour = tick_colour,
		};
		return m_broker.SetChartDisplayOptionsAsync(instance_id, parameters);
	}
}
