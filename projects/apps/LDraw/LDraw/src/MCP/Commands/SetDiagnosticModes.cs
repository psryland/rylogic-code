using System;
using System.ComponentModel;
using System.Threading.Tasks;
using Rylogic.Gfx;

namespace LDraw.MCP;

/// <summary>Set-diagnostic-modes command instance implementation</summary>
internal sealed partial class LDrawInstanceHost
{
	/// <summary>Set one or more diagnostic and rendering modes for the requested scene</summary>
	private Task<LDrawViewMutationResult> SetDiagnosticModesAsync(LDrawSetDiagnosticModesParams parameters)
	{
		return m_model.InvokeAsync(() =>
		{
			var scene = ResolveScene(parameters.SceneName);
			var scene_view = scene.SceneView;
			if (parameters.FillMode != null)
				scene.SceneState.Chart.FillMode = ParseDiagnosticEnum<View3d.EFillMode>(parameters.FillMode, "fill_mode");
			if (parameters.CullMode != null)
				scene.SceneState.Chart.CullMode = ParseDiagnosticEnum<View3d.ECullMode>(parameters.CullMode, "cull_mode");
			if (parameters.FocusPointVisible != null)
				scene.SceneState.Chart.FocusPointVisible = parameters.FocusPointVisible.Value;
			if (parameters.OriginPointVisible != null)
				scene.SceneState.Chart.OriginPointVisible = parameters.OriginPointVisible.Value;
			if (parameters.SelectionBoxVisible != null)
				scene_view.SelectionBoxVisible = parameters.SelectionBoxVisible.Value;
			if (parameters.ObjectInfoEnabled != null)
				scene_view.ObjectInfoEnabled = parameters.ObjectInfoEnabled.Value;
			if (parameters.BBoxesVisible != null)
				scene_view.BBoxesVisible = parameters.BBoxesVisible.Value;
			if (parameters.ShowNormals != null)
				scene_view.ShowNormals = parameters.ShowNormals.Value;
			if (parameters.NormalsLength != null)
				scene_view.NormalsLength = PositiveFloat(parameters.NormalsLength.Value, "normals_length");
			if (parameters.NormalsColour != null)
				scene_view.NormalsColour = ParseColour(parameters.NormalsColour);
			if (parameters.FillModePointsSize != null)
				scene_view.FillModePointsSize = PositiveFloat(parameters.FillModePointsSize.Value, "fill_mode_points_size");

			scene_view.Invalidate();
			return CreateViewMutationResult("set_diagnostic_modes", scene);
		}, TimeSpan.FromSeconds(10));
	}
}
