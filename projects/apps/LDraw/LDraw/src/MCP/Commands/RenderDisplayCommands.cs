using System;
using System.ComponentModel;
using System.IO;
using System.Linq;
using System.Threading.Tasks;
using LDraw.UI;
using ModelContextProtocol.Server;
using Rylogic.Gui.WPF;

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

/// <summary>Render and display command instance implementation</summary>
internal sealed partial class LDrawInstanceHost
{
	/// <summary>Set runtime-only render settings for the requested scene</summary>
	private Task<LDrawViewMutationResult> SetRenderSettingsAsync(LDrawSetRenderSettingsParams parameters)
	{
		return m_model.InvokeAsync(() =>
		{
			if (parameters.Antialiasing == null && parameters.ShadowCastRange == null && parameters.RayTracingEnabled == null)
				throw new InvalidOperationException("ldraw_set_render_settings requires at least one render setting.");

			var scene = ResolveScene(parameters.SceneName);
			if (parameters.Antialiasing != null || parameters.ShadowCastRange != null)
			{
				var options = RuntimeChartOptions(scene);
				if (parameters.Antialiasing != null)
					options.Antialiasing = parameters.Antialiasing.Value;
				if (parameters.ShadowCastRange != null)
				{
					if (parameters.ShadowCastRange.Value < 0.0)
						throw new InvalidOperationException("shadow_cast_range must be greater than or equal to zero.");

					options.ShadowCastRange = parameters.ShadowCastRange.Value;
				}
			}
			if (parameters.RayTracingEnabled != null)
			{
				var window = scene.SceneView.Scene.Window;
				if (parameters.RayTracingEnabled.Value && !window.RayTracingAvailable)
					throw new InvalidOperationException("Ray tracing is not available for this View3D window.");

				window.RayTracingEnabled = parameters.RayTracingEnabled.Value;
			}

			scene.SceneView.Invalidate();
			return new LDrawViewMutationResult
			{
				SceneName = scene.SceneName,
				Action = "set_render_settings",
				ViewSettings = CreateViewSettingsInfo(scene),
			};
		}, TimeSpan.FromSeconds(10));
	}

	/// <summary>Capture the requested scene to an image file</summary>
	private Task<LDrawSceneCaptureResult> CaptureSceneAsync(LDrawCaptureSceneParams parameters)
	{
		return m_model.InvokeAsync(() =>
		{
			var scene = ResolveScene(parameters.SceneName);
			var output_path = CaptureOutputPath(scene.SceneName, parameters.OutputPath);
			if (!parameters.Overwrite && File.Exists(output_path))
				throw new InvalidOperationException($"Capture output already exists: {output_path}");

			var directory = Path.GetDirectoryName(output_path);
			if (!string.IsNullOrWhiteSpace(directory))
				Directory.CreateDirectory(directory);

			var size = scene.SceneView.Scene.SaveImage(output_path);
			var file_info = new FileInfo(output_path);
			return new LDrawSceneCaptureResult
			{
				SceneName = scene.SceneName,
				OutputPath = output_path,
				Format = Path.GetExtension(output_path).TrimStart('.').ToLowerInvariant(),
				Width = size.Width,
				Height = size.Height,
				ByteCount = file_info.Length,
			};
		}, TimeSpan.FromSeconds(10));
	}

	/// <summary>Return chart display options for the requested scene</summary>
	private Task<LDrawChartDisplayOptionsInfo> GetChartDisplayOptionsAsync(LDrawChartDisplayOptionsParams parameters)
	{
		return m_model.InvokeAsync(() =>
		{
			var scene = ResolveScene(parameters.SceneName);
			return CreateChartDisplayOptionsInfo(scene);
		}, TimeSpan.FromSeconds(5));
	}

	/// <summary>Set runtime-only chart display options for the requested scene</summary>
	private Task<LDrawChartDisplayOptionsResult> SetChartDisplayOptionsAsync(LDrawSetChartDisplayOptionsParams parameters)
	{
		return m_model.InvokeAsync(() =>
		{
			if (!HasChartDisplayMutation(parameters))
				throw new InvalidOperationException("ldraw_set_chart_display_options requires at least one chart display option.");

			var scene = ResolveScene(parameters.SceneName);
			var options = RuntimeChartOptions(scene);
			if (parameters.ShowAxes != null)
				options.ShowAxes = parameters.ShowAxes.Value;
			if (parameters.ShowGridLines != null)
				options.ShowGridLines = parameters.ShowGridLines.Value;
			if (parameters.DrawTickMarks != null)
			{
				options.XAxis.DrawTickMarks = parameters.DrawTickMarks.Value;
				options.YAxis.DrawTickMarks = parameters.DrawTickMarks.Value;
			}
			if (parameters.DrawTickLabels != null)
			{
				options.XAxis.DrawTickLabels = parameters.DrawTickLabels.Value;
				options.YAxis.DrawTickLabels = parameters.DrawTickLabels.Value;
			}
			if (parameters.PixelsPerTick != null)
			{
				if (parameters.PixelsPerTick.Value <= 0.0)
					throw new InvalidOperationException("pixels_per_tick must be greater than zero.");

				options.XAxis.PixelsPerTick = parameters.PixelsPerTick.Value;
				options.YAxis.PixelsPerTick = parameters.PixelsPerTick.Value;
			}
			if (!string.IsNullOrWhiteSpace(parameters.AxisColour))
			{
				var colour = ParseColour(parameters.AxisColour);
				options.XAxis.AxisColour = colour;
				options.YAxis.AxisColour = colour;
			}
			if (!string.IsNullOrWhiteSpace(parameters.TickColour))
			{
				var colour = ParseColour(parameters.TickColour);
				options.XAxis.TickColour = colour;
				options.YAxis.TickColour = colour;
			}

			scene.SceneView.Invalidate();
			return new LDrawChartDisplayOptionsResult
			{
				Action = "set_chart_display_options",
				Options = CreateChartDisplayOptionsInfo(scene),
			};
		}, TimeSpan.FromSeconds(10));
	}

	/// <summary>Create a runtime-only options copy so MCP display changes do not mutate profile-backed scene state</summary>
	private static ChartControl.OptionsData RuntimeChartOptions(SceneUI scene)
	{
		var options = new ChartControl.OptionsData(scene.SceneView.Options);
		scene.SceneView.Options = options;
		return options;
	}

	/// <summary>Return true if any chart display option mutation parameter was supplied</summary>
	private static bool HasChartDisplayMutation(LDrawSetChartDisplayOptionsParams parameters)
	{
		return
			parameters.ShowAxes != null ||
			parameters.ShowGridLines != null ||
			parameters.DrawTickMarks != null ||
			parameters.DrawTickLabels != null ||
			parameters.PixelsPerTick != null ||
			!string.IsNullOrWhiteSpace(parameters.AxisColour) ||
			!string.IsNullOrWhiteSpace(parameters.TickColour);
	}

	/// <summary>Create a serialisable chart display options DTO for 'scene'</summary>
	private static LDrawChartDisplayOptionsInfo CreateChartDisplayOptionsInfo(SceneUI scene)
	{
		var options = scene.SceneView.Options;
		return new LDrawChartDisplayOptionsInfo
		{
			SceneName = scene.SceneName,
			ShowAxes = options.ShowAxes,
			ShowGridLines = options.ShowGridLines,
			XDrawTickMarks = options.XAxis.DrawTickMarks,
			YDrawTickMarks = options.YAxis.DrawTickMarks,
			XDrawTickLabels = options.XAxis.DrawTickLabels,
			YDrawTickLabels = options.YAxis.DrawTickLabels,
			XPixelsPerTick = options.XAxis.PixelsPerTick,
			YPixelsPerTick = options.YAxis.PixelsPerTick,
			XAxisColour = options.XAxis.AxisColour.ToString(),
			YAxisColour = options.YAxis.AxisColour.ToString(),
			XTickColour = options.XAxis.TickColour.ToString(),
			YTickColour = options.YAxis.TickColour.ToString(),
		};
	}

	/// <summary>Resolve and validate a capture output path</summary>
	private static string CaptureOutputPath(string scene_name, string? output_path)
	{
		var path = string.IsNullOrWhiteSpace(output_path)
			? Path.Combine(Path.GetTempPath(), $"ldraw_{SafeFileName(scene_name)}_{DateTime.Now:yyyyMMdd_HHmmss_fff}.png")
			: Path.GetFullPath(output_path);

		if (string.IsNullOrWhiteSpace(Path.GetExtension(path)))
			path = Path.ChangeExtension(path, ".png");

		var extension = Path.GetExtension(path).ToLowerInvariant();
		return extension switch
		{
			".png" or ".jpg" or ".bmp" => path,
			_ => throw new InvalidOperationException("Capture output extension must be .png, .jpg, or .bmp."),
		};
	}

	/// <summary>Replace invalid file-name characters with underscores</summary>
	private static string SafeFileName(string name)
	{
		var invalid = Path.GetInvalidFileNameChars();
		return new string(name.Select(x => invalid.Contains(x) ? '_' : x).ToArray());
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
