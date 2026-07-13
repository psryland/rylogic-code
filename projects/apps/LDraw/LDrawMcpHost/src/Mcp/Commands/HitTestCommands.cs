using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Threading.Tasks;
using ModelContextProtocol.Server;

namespace LDraw.MCP;

/// <summary>Hit-test command broker routing</summary>
internal sealed partial class McpBroker
{
	/// <summary>Run a scene hit test in 'instance_id'</summary>
	public async Task<LDrawHitTestInfo> HitTestAsync(string? instance_id, LDrawHitTestParams parameters)
	{
		var registration = await ResolveInstanceAsync(instance_id).ConfigureAwait(false);
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
		[Description("The id of a running LDraw instance from ldraw_list_instances. Omit to target the default instance (most-recently-used, or auto-launched when none are running).")] string? instance_id = null,
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
