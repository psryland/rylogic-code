using System;
using System.ComponentModel;
using System.IO;
using System.Linq;
using System.Threading.Tasks;
using LDraw.UI;
using Rylogic.Gui.WPF;

namespace LDraw.MCP;

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
