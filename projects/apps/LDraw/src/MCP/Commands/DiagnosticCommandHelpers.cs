using System;
using System.Globalization;
using System.Linq;
using LDraw.UI;
using Rylogic.Gfx;

namespace LDraw.MCP;

/// <summary>Shared helpers for diagnostic MCP commands</summary>
internal sealed partial class LDrawInstanceHost
{
	/// <summary>Create diagnostic mode DTO data from current scene state</summary>
	private static LDrawDiagnosticModesInfo CreateDiagnosticModesInfo(SceneUI scene)
	{
		return new LDrawDiagnosticModesInfo
		{
			SceneName = scene.SceneName,
			ViewSettings = CreateViewSettingsInfo(scene),
			AvailableFillModes = [..Enum.GetNames<View3d.EFillMode>()],
			AvailableCullModes = [..Enum.GetNames<View3d.ECullMode>()],
			SessionLocalModes =
			[
				nameof(LDrawSetDiagnosticModesParams.BBoxesVisible),
				nameof(LDrawSetDiagnosticModesParams.ShowNormals),
				nameof(LDrawSetDiagnosticModesParams.NormalsLength),
				nameof(LDrawSetDiagnosticModesParams.NormalsColour),
				nameof(LDrawSetDiagnosticModesParams.FillModePointsSize),
				nameof(LDrawSetDiagnosticModesParams.SelectionBoxVisible),
				nameof(LDrawSetDiagnosticModesParams.ObjectInfoEnabled),
			],
		};
	}

	/// <summary>Parse a named enum value used by View3D diagnostic settings</summary>
	private static TEnum ParseDiagnosticEnum<TEnum>(string? value, string parameter_name)
		where TEnum : struct, Enum
	{
		if (string.IsNullOrWhiteSpace(value))
			throw new InvalidOperationException($"{parameter_name} is required.");
		if (!Enum.TryParse<TEnum>(value.Trim(), ignoreCase: true, out var parsed))
		{
			var values = string.Join(", ", Enum.GetNames<TEnum>());
			throw new InvalidOperationException($"{parameter_name} must be one of: {values}.");
		}
		return parsed;
	}

	/// <summary>Convert a positive finite MCP numeric value to a float</summary>
	private static float PositiveFloat(double value, string parameter_name)
	{
		if (!double.IsFinite(value) || value <= 0.0 || value > float.MaxValue)
			throw new InvalidOperationException($"{parameter_name} must be a positive finite value.");
		return (float)value;
	}

	/// <summary>Format a floating point value for generated LDraw script</summary>
	private static string FormatReal(float value)
	{
		return value.ToString("R", CultureInfo.InvariantCulture);
	}
}
