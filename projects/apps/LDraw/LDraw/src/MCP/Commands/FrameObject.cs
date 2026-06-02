using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Threading.Tasks;

namespace LDraw.MCP;

/// <summary>Frame-object command instance implementation</summary>
internal sealed partial class LDrawInstanceHost
{
	/// <summary>Frame one or more matching objects in the requested scene</summary>
	private Task<LDrawFrameResult> FrameObjectAsync(LDrawFrameObjectParams parameters)
	{
		return m_model.InvokeAsync(() =>
		{
			var scene = ResolveScene(parameters.SceneName);
			var query = QueryObjects(scene, parameters);
			var targets = parameters.AllowMultiple ? query.Matches : new List<ObjectEntry> { ResolveSingleObject(query, "ldraw_frame_object") };
			if (targets.Count == 0)
				throw new InvalidOperationException("ldraw_frame_object did not match any objects.");
			if (parameters.AllowMultiple && query.Truncated)
				throw new InvalidOperationException("ldraw_frame_object matched more objects than max_count. Increase max_count or refine the query.");

			// Use the native selected-bounds calculation without permanently changing selection unless the caller explicitly asks for it.
			var original_selection = CaptureSelectionIds(scene);
			var action = parameters.Select ? "select_and_frame_object" : "frame_object";
			try
			{
				ApplySelection(scene, targets, "replace");
				var bounds = SelectedBounds(scene);
				if (!bounds.IsValid)
					throw new InvalidOperationException("Cannot frame object because the matched object bounds are not valid.");

				var window = scene.SceneView.Scene.Window;
				window.Camera.ResetView(bounds);
				if (!parameters.Select)
					RestoreSelection(scene, original_selection);

				scene.SceneView.Invalidate();
				return new LDrawFrameResult
				{
					SceneName = scene.SceneName,
					Action = action,
					Objects = CreateObjectInfos(targets),
					Bounds = LDrawBoundsInfo.From(bounds),
					Camera = CreateCameraInfo(scene),
				};
			}
			finally
			{
				if (!parameters.Select)
					RestoreSelection(scene, original_selection);
			}
		}, TimeSpan.FromSeconds(10));
	}
}
