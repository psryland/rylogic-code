using System;
using System.ComponentModel;
using System.Linq;
using System.Threading.Tasks;
using Rylogic.Gfx;
using Rylogic.Maths;

namespace LDraw.MCP;

/// <summary>Hit-test command instance implementation</summary>
internal sealed partial class LDrawInstanceHost
{
	/// <summary>Run a screen-space ray hit test in the requested scene</summary>
	private Task<LDrawHitTestInfo> HitTestAsync(LDrawHitTestParams parameters)
	{
		return m_model.InvokeAsync(() =>
		{
			var scene = ResolveScene(parameters.SceneName);
			var window = scene.SceneView.Scene.Window;
			var point_ss = HitTestPoint(window, parameters);
			var ray = window.Camera.RaySS(point_ss, ParseSnapMode(parameters.SnapMode), ParseSnapDistance(parameters.SnapDistance));
			var hit = HasObjectHitTestScope(parameters)
				? window.HitTest(ray, HitTestObjects(scene, parameters))
				: window.HitTest(ray);

			return CreateHitTestInfo(scene.SceneName, hit);
		}, TimeSpan.FromSeconds(10));
	}

	/// <summary>Create a screen-space hit-test point, defaulting to the viewport centre</summary>
	private static v2 HitTestPoint(View3d.Window window, LDrawHitTestParams parameters)
	{
		var viewport = window.Viewport;
		var width = viewport.ScreenW != 0 ? viewport.ScreenW : viewport.Width;
		var height = viewport.ScreenH != 0 ? viewport.ScreenH : viewport.Height;
		var x = parameters.X ?? width * 0.5;
		var y = parameters.Y ?? height * 0.5;
		return new v2((float)x, (float)y);
	}

	/// <summary>Return the object subset to use for object-scoped hit tests</summary>
	private View3d.Object[] HitTestObjects(LDraw.UI.SceneUI scene, LDrawHitTestParams parameters)
	{
		var query = QueryObjects(scene, parameters);
		var targets = ResolveMutationTargets(query, "ldraw_hit_test");
		return [..targets.Select(x => x.Object)];
	}

	/// <summary>Return true when object query filters should restrict the hit test</summary>
	private static bool HasObjectHitTestScope(LDrawHitTestParams parameters)
	{
		return
			!string.IsNullOrWhiteSpace(parameters.ObjectId) ||
			!string.IsNullOrWhiteSpace(parameters.Name) ||
			!string.IsNullOrWhiteSpace(parameters.Type) ||
			!string.IsNullOrWhiteSpace(parameters.ContextId) ||
			parameters.Selected != null ||
			parameters.Visible != null;
	}

	/// <summary>Parse a hit-test snap mode name or flags expression</summary>
	private static View3d.ESnapMode ParseSnapMode(string? snap_mode)
	{
		if (string.IsNullOrWhiteSpace(snap_mode))
			return View3d.ESnapMode.AllPerspective;

		var value = snap_mode.Replace("|", ",", StringComparison.Ordinal).Trim();
		if (!Enum.TryParse<View3d.ESnapMode>(value, ignoreCase: true, out var mode))
			throw new InvalidOperationException($"Unknown snap mode '{snap_mode}'.");

		return mode;
	}

	/// <summary>Validate the screen-space snap distance</summary>
	private static float ParseSnapDistance(double snap_distance)
	{
		if (snap_distance < 0.0)
			throw new InvalidOperationException("Snap distance must be greater than or equal to zero.");

		return (float)snap_distance;
	}

	/// <summary>Create a serialisable DTO from a View3D hit-test result</summary>
	private LDrawHitTestInfo CreateHitTestInfo(string scene_name, View3d.HitTestResult hit)
	{
		var hit_object = hit.HitObject;
		return new LDrawHitTestInfo
		{
			SceneName = scene_name,
			IsHit = hit.IsHit,
			RayOrigin = LDrawVector3.From(hit.m_ws_ray_origin),
			RayDirection = LDrawVector3.From(hit.m_ws_ray_direction),
			Intercept = LDrawVector3.From(hit.m_ws_intercept),
			Normal = LDrawVector3.From(hit.m_ws_normal),
			Distance = hit.m_distance,
			SnapType = hit.m_snap_type.ToString(),
			RayIndex = hit.m_ray_index,
			RayId = hit.m_ray_id,
			Object = hit_object != null ? CreateObjectInfo(new ObjectEntry(hit_object, 0, hit_object.Name, null, hit_object)) : null,
		};
	}
}
