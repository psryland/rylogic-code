using System;
using System.ComponentModel;
using System.Threading.Tasks;

namespace LDraw.MCP;

/// <summary>Show-normals command instance implementation</summary>
internal sealed partial class LDrawInstanceHost
{
	/// <summary>Set normal visibility for matching objects in the requested scene</summary>
	private Task<LDrawObjectMutationResult> ShowNormalsAsync(LDrawShowNormalsParams parameters)
	{
		return m_model.InvokeAsync(() =>
		{
			var scene = ResolveScene(parameters.SceneName);
			var query = QueryObjects(scene, parameters);
			var targets = ResolveMutationTargets(query, "ldraw_show_normals");
			foreach (var entry in targets)
			{
				// View3D uses null for this object only and an empty name for this object plus all children.
				entry.Object.ShowNormalsSet(parameters.SetShowNormals, parameters.Recursive ? string.Empty : null);
			}

			scene.SceneView.Invalidate();
			return new LDrawObjectMutationResult
			{
				SceneName = scene.SceneName,
				Action = parameters.SetShowNormals ? "show_normals" : "hide_normals",
				Objects = CreateObjectInfos(targets),
			};
		}, TimeSpan.FromSeconds(10));
	}
}
