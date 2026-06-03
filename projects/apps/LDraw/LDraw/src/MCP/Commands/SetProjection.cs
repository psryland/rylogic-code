using System;
using System.ComponentModel;
using System.Threading.Tasks;

namespace LDraw.MCP;

/// <summary>Set-projection command instance implementation</summary>
internal sealed partial class LDrawInstanceHost
{
	/// <summary>Set perspective or orthographic projection for the requested scene</summary>
	private Task<LDrawViewMutationResult> SetProjectionAsync(LDrawSetProjectionParams parameters)
	{
		return m_model.InvokeAsync(() =>
		{
			var scene = ResolveScene(parameters.SceneName);
			scene.SceneState.Chart.Orthographic = parameters.Orthographic;
			scene.SceneView.Invalidate();
			return CreateViewMutationResult(parameters.Orthographic ? "set_projection_orthographic" : "set_projection_perspective", scene);
		}, TimeSpan.FromSeconds(5));
	}
}
