using System;
using System.ComponentModel;
using System.Threading.Tasks;
using Rylogic.Gfx;

namespace LDraw.MCP;

/// <summary>Object render-state command instance implementation</summary>
internal sealed partial class LDrawInstanceHost
{
	/// <summary>Return render-state data for one object in the requested scene</summary>
	private Task<LDrawObjectInfo> GetObjectRenderStateAsync(LDrawGetObjectRenderStateParams parameters)
	{
		return m_model.InvokeAsync(() =>
		{
			var scene = ResolveScene(parameters.SceneName);
			var query = QueryObjects(scene, parameters);
			var target = ResolveSingleObject(query, "ldraw_get_object_render_state");
			return CreateObjectInfo(target);
		}, TimeSpan.FromSeconds(10));
	}

	/// <summary>Set render-state data for matching objects in the requested scene</summary>
	private Task<LDrawObjectMutationResult> SetObjectRenderStateAsync(LDrawSetObjectRenderStateParams parameters)
	{
		return m_model.InvokeAsync(() =>
		{
			if (!HasRenderStateMutation(parameters))
				throw new InvalidOperationException("ldraw_set_object_render_state requires at least one render-state parameter.");

			var scene = ResolveScene(parameters.SceneName);
			var query = QueryObjects(scene, parameters);
			var targets = ResolveMutationTargets(query, "ldraw_set_object_render_state");
			var name = parameters.Recursive ? string.Empty : null;
			foreach (var entry in targets)
			{
				if (parameters.VisibleState != null)
					entry.Object.VisibleSet(parameters.VisibleState.Value, name);
				if (parameters.Wireframe != null)
					entry.Object.WireframeSet(parameters.Wireframe.Value, name);
				if (parameters.ShowNormals != null)
					entry.Object.ShowNormalsSet(parameters.ShowNormals.Value, name);
				if (parameters.Reflectivity != null)
					entry.Object.ReflectivitySet((float)parameters.Reflectivity.Value, name);
				if (!string.IsNullOrWhiteSpace(parameters.SortGroup))
					entry.Object.SortGroupSet(ParseSortGroup(parameters.SortGroup), name);
				if (!string.IsNullOrWhiteSpace(parameters.NuggetTint))
					entry.Object.NuggetTintSet(ParseColour(parameters.NuggetTint), name);

				ApplyFlag(entry.Object, View3d.ELdrFlags.SceneBoundsExclude, parameters.SceneBoundsExcluded, name);
				ApplyFlag(entry.Object, View3d.ELdrFlags.HitTestExclude, parameters.HitTestExcluded, name);
				ApplyFlag(entry.Object, View3d.ELdrFlags.ShadowCastExclude, parameters.ShadowCastExcluded, name);
				ApplyFlag(entry.Object, View3d.ELdrFlags.NoZTest, parameters.NoZTest, name);
				ApplyFlag(entry.Object, View3d.ELdrFlags.NoZWrite, parameters.NoZWrite, name);
				ApplyNuggetFlag(entry.Object, View3d.ENuggetFlag.Hidden, parameters.NuggetHidden, name);
				ApplyNuggetFlag(entry.Object, View3d.ENuggetFlag.AlphaBlend, parameters.NuggetAlphaBlend, name);
			}

			scene.SceneView.Invalidate();
			return new LDrawObjectMutationResult
			{
				SceneName = scene.SceneName,
				Action = "set_object_render_state",
				Objects = CreateObjectInfos(targets),
			};
		}, TimeSpan.FromSeconds(10));
	}

	/// <summary>Return true if any render-state mutation parameter was supplied</summary>
	private static bool HasRenderStateMutation(LDrawSetObjectRenderStateParams parameters)
	{
		return
			parameters.VisibleState != null ||
			parameters.Wireframe != null ||
			parameters.ShowNormals != null ||
			parameters.Reflectivity != null ||
			!string.IsNullOrWhiteSpace(parameters.SortGroup) ||
			!string.IsNullOrWhiteSpace(parameters.NuggetTint) ||
			parameters.SceneBoundsExcluded != null ||
			parameters.HitTestExcluded != null ||
			parameters.ShadowCastExcluded != null ||
			parameters.NoZTest != null ||
			parameters.NoZWrite != null ||
			parameters.NuggetHidden != null ||
			parameters.NuggetAlphaBlend != null;
	}

	/// <summary>Parse a View3D object sort group name</summary>
	private static View3d.ESortGroup ParseSortGroup(string value)
	{
		if (!Enum.TryParse<View3d.ESortGroup>(value.Trim(), ignoreCase: true, out var sort_group))
			throw new InvalidOperationException($"Unknown sort group '{value}'.");

		return sort_group;
	}

	/// <summary>Apply an LDraw object flag when 'state' has a value</summary>
	private static void ApplyFlag(View3d.Object obj, View3d.ELdrFlags flag, bool? state, string? name)
	{
		if (state != null)
			obj.FlagsSet(flag, state.Value, name);
	}

	/// <summary>Apply a first-nugget flag when 'state' has a value</summary>
	private static void ApplyNuggetFlag(View3d.Object obj, View3d.ENuggetFlag flag, bool? state, string? name)
	{
		if (state != null)
			obj.NuggetFlagsSet(flag, state.Value, name);
	}
}
