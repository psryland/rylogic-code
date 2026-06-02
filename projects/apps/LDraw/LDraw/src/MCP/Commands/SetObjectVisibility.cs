using System;
using System.ComponentModel;
using System.Threading.Tasks;

namespace LDraw.MCP;

/// <summary>Set-object-visibility command instance implementation</summary>
internal sealed partial class LDrawInstanceHost
{
	/// <summary>Set visibility for matching objects in the requested scene</summary>
	private Task<LDrawObjectMutationResult> SetObjectVisibilityAsync(LDrawSetObjectVisibilityParams parameters)
	{
		return m_model.InvokeAsync(() =>
		{
			if (parameters.SetVisible == null)
				throw new InvalidOperationException("ldraw_set_object_visibility requires the visible parameter.");

			var scene = ResolveScene(parameters.SceneName);
			var query = QueryObjects(scene, parameters);
			var targets = ResolveMutationTargets(query, "ldraw_set_object_visibility");
			foreach (var entry in targets)
				entry.Object.Visible = parameters.SetVisible.Value;

			scene.SceneView.Invalidate();
			return new LDrawObjectMutationResult
			{
				SceneName = scene.SceneName,
				Action = parameters.SetVisible.Value ? "show_objects" : "hide_objects",
				Objects = CreateObjectInfos(targets),
			};
		}, TimeSpan.FromSeconds(10));
	}
}
