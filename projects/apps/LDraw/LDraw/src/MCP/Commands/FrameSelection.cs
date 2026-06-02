using System;
using System.ComponentModel;
using System.Threading.Tasks;

namespace LDraw.MCP;

/// <summary>Frame-selection command instance implementation</summary>
internal sealed partial class LDrawInstanceHost
{
	/// <summary>Frame the currently selected bounds in the requested scene</summary>
	private Task<LDrawFrameResult> FrameSelectionAsync(LDrawFrameSelectionParams parameters)
	{
		return m_model.InvokeAsync(() =>
		{
			var scene = ResolveScene(parameters.SceneName);
			var bounds = SelectedBounds(scene);
			if (!bounds.IsValid)
				throw new InvalidOperationException("Cannot frame selection because no selected object bounds are valid.");

			// The native selected-bounds path matches the existing user-facing Object Manager centre-view behaviour.
			var window = scene.SceneView.Scene.Window;
			window.Camera.ResetView(bounds);
			scene.SceneView.Invalidate();

			return new LDrawFrameResult
			{
				SceneName = scene.SceneName,
				Action = "frame_selection",
				Objects = CreateObjectInfos(SelectedObjects(scene, include_children: true, parameters.MaxCount)),
				Bounds = LDrawBoundsInfo.From(bounds),
				Camera = CreateCameraInfo(scene),
			};
		}, TimeSpan.FromSeconds(10));
	}
}
