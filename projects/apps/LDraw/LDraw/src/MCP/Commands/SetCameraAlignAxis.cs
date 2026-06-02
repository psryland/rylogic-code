using System;
using System.ComponentModel;
using System.Threading.Tasks;

namespace LDraw.MCP;

/// <summary>Set-camera-align-axis command instance implementation</summary>
internal sealed partial class LDrawInstanceHost
{
	/// <summary>Set the named camera up-axis alignment for the requested scene</summary>
	private Task<LDrawViewMutationResult> SetCameraAlignAxisAsync(LDrawSetCameraAlignAxisParams parameters)
	{
		return m_model.InvokeAsync(() =>
		{
			var scene = ResolveScene(parameters.SceneName);
			scene.SceneState.AlignDirection = ParseAlignDirection(parameters.AlignDirection);
			scene.SceneView.Invalidate();
			return CreateViewMutationResult("set_camera_align_axis", scene);
		}, TimeSpan.FromSeconds(5));
	}
}
