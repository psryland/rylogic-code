using System;
using System.ComponentModel;
using System.Linq;
using System.Threading.Tasks;
using ModelContextProtocol.Server;
using Rylogic.Gfx;
using Rylogic.Maths;

namespace LDraw.MCP;

/// <summary>Hit-test command broker routing</summary>
internal sealed partial class McpBroker
{
	/// <summary>Run a scene hit test in 'instance_id'</summary>
	public async Task<LDrawHitTestInfo> HitTestAsync(string? instance_id, LDrawHitTestParams parameters)
	{
		var registration = ResolveInstance(instance_id);
		return await m_client.HitTestAsync(registration, parameters).ConfigureAwait(false);
	}
}

/// <summary>Hit-test command pipe client calls</summary>
internal sealed partial class InstancePipeClient
{
	/// <summary>Run a scene hit test in 'registration'</summary>
	public async Task<LDrawHitTestInfo> HitTestAsync(InstanceRegistration registration, LDrawHitTestParams parameters)
	{
		return await SendAsync<LDrawHitTestInfo>(registration, InstancePipeCommands.HitTest, parameters, WriteTimeout).ConfigureAwait(false);
	}
}

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

/// <summary>Hit-test command MCP tool surface</summary>
internal sealed partial class LDrawTools
{
	/// <summary>Ray-cast from a screen-space point into a running LDraw scene</summary>
	[McpServerTool(Name = "ldraw_hit_test", Title = "Hit test LDraw scene", ReadOnly = true, Destructive = false, Idempotent = true, OpenWorld = false, UseStructuredContent = true)]
	[Description("Ray-casts from a screen-space point in a running LDraw scene and returns serialised hit data plus the hit object DTO. Omit x/y to cast through the viewport centre.")]
	public Task<LDrawHitTestInfo> HitTest(
		[Description("Screen-space X coordinate relative to the scene viewport. Omit for viewport centre.")] double? x = null,
		[Description("Screen-space Y coordinate relative to the scene viewport. Omit for viewport centre.")] double? y = null,
		[Description("Snap mode: NoSnap, Verts, Edges, Faces, All, AllPerspective, or a comma/pipe-separated flags expression.")] string? snap_mode = null,
		[Description("Snap distance in screen-space pixels.")] double snap_distance = 0.0,
		[Description("The instance id returned by ldraw_list_instances. Omit to target the broker instance.")] string? instance_id = null,
		[Description("The scene name to hit test. Omit to use the first scene.")] string? scene_name = null,
		[Description("Optional object id to restrict hit testing to one object.")] string? object_id = null,
		[Description("Optional object name filter to restrict hit testing.")] string? name = null,
		[Description("Optional object type filter to restrict hit testing.")] string? type = null,
		[Description("Optional source context id filter to restrict hit testing.")] string? context_id = null,
		[Description("Optional selected-state filter to restrict hit testing.")] bool? selected = null,
		[Description("Optional visible-state filter to restrict hit testing.")] bool? visible = null,
		[Description("True to include child objects in object-scoped hit tests.")] bool include_children = true,
		[Description("Name/type matching mode for object-scoped hit tests: exact, contains, or regex.")] string match_mode = "contains",
		[Description("True for case-sensitive name/type matching in object-scoped hit tests.")] bool case_sensitive = false,
		[Description("Maximum number of object-scoped candidates, clamped to 1..1000. Throws if more matches exist.")] int max_count = 200)
	{
		var parameters = ObjectQueryParameters<LDrawHitTestParams>(scene_name, object_id, name, type, context_id, selected, visible, include_children, match_mode, case_sensitive, max_count);
		parameters.X = x;
		parameters.Y = y;
		parameters.SnapMode = snap_mode;
		parameters.SnapDistance = snap_distance;
		return m_broker.HitTestAsync(instance_id, parameters);
	}
}
