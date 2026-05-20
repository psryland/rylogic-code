using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using ModelContextProtocol.Server;
using Rylogic.Gfx;
using Rylogic.Maths;

namespace LDraw.MCP;

/// <summary>Show-object-bounds command broker routing</summary>
internal sealed partial class McpBroker
{
	/// <summary>Draw bounds for objects matching a query in 'instance_id'</summary>
	public async Task<LDrawBoundsOverlayResult> ShowObjectBoundsAsync(string? instance_id, LDrawShowObjectBoundsParams parameters)
	{
		var registration = ResolveInstance(instance_id);
		return await m_client.ShowObjectBoundsAsync(registration, parameters).ConfigureAwait(false);
	}
}

/// <summary>Show-object-bounds command pipe client call</summary>
internal sealed partial class InstancePipeClient
{
	/// <summary>Draw bounds for objects in 'registration'</summary>
	public async Task<LDrawBoundsOverlayResult> ShowObjectBoundsAsync(InstanceRegistration registration, LDrawShowObjectBoundsParams parameters)
	{
		return await SendAsync<LDrawBoundsOverlayResult>(registration, InstancePipeCommands.ShowObjectBounds, parameters, WriteTimeout).ConfigureAwait(false);
	}
}

/// <summary>Show-object-bounds command instance implementation</summary>
internal sealed partial class LDrawInstanceHost
{
	/// <summary>Draw world-space AABBs for matching objects using a generated MCP overlay</summary>
	private async Task<LDrawBoundsOverlayResult> ShowObjectBoundsAsync(LDrawShowObjectBoundsParams parameters)
	{
		var setup = await m_model.InvokeAsync(() =>
		{
			var scene = ResolveScene(parameters.SceneName);
			var query = QueryObjects(scene, parameters);
			var targets = ResolveMutationTargets(query, "ldraw_show_object_bounds");
			var line_width = PositiveFloat(parameters.LineWidth, "line_width");
			var colour = ParseColour(string.IsNullOrWhiteSpace(parameters.Colour) ? "FF00FF00" : parameters.Colour);
			var bounds = new List<BBox>(targets.Count);

			foreach (var entry in targets)
			{
				var model_bounds = entry.Object.BBoxMS(View3d.EBBoxFlags.IncludeChildren);
				if (!model_bounds.IsValid)
					continue;

				var world_bounds = entry.Object.O2WGet(null) * model_bounds;
				if (world_bounds.IsValid)
					bounds.Add(world_bounds);
			}
			if (bounds.Count == 0)
				throw new InvalidOperationException("ldraw_show_object_bounds did not find any valid bounds for the matched objects.");

			return new BoundsOverlaySetup(
				scene.SceneName,
				CreateObjectInfos(targets),
				[..bounds.Select(LDrawBoundsInfo.From)],
				BuildBoundsOverlayScript(bounds, colour.ToString(), line_width));
		}, TimeSpan.FromSeconds(10)).ConfigureAwait(false);

		var overlay_id = string.IsNullOrWhiteSpace(parameters.OverlayId) ? "diagnostic_bounds" : parameters.OverlayId;
		var overlay_name = string.IsNullOrWhiteSpace(parameters.Name) ? "MCP Diagnostic Bounds" : parameters.Name;
		var overlay = await SetOverlayScriptAsync(new LDrawOverlayScriptParams
		{
			OverlayId = overlay_id,
			Name = overlay_name,
			Script = setup.Script,
			SceneNames = [setup.SceneName],
			ResetView = false,
		}, append: false).ConfigureAwait(false);

		return new LDrawBoundsOverlayResult
		{
			SceneName = setup.SceneName,
			Action = "show_object_bounds",
			Overlay = overlay.Overlay,
			Objects = setup.Objects,
			Bounds = setup.Bounds,
		};
	}

	/// <summary>Build the LDraw script that displays the requested bounds</summary>
	private static string BuildBoundsOverlayScript(IReadOnlyList<BBox> bounds, string colour, float line_width)
	{
		var script = new StringBuilder();
		script.AppendLine("*Group mcp_diagnostic_bounds FFFFFFFF");
		script.AppendLine("{");
		for (int i = 0; i != bounds.Count; ++i)
		{
			var bbox = bounds[i];
			script.AppendLine($"	*LineBox bounds_{i} {colour}");
			script.AppendLine("	{");
			script.AppendLine($"		*Data {{{FormatReal(bbox.SizeX)} {FormatReal(bbox.SizeY)} {FormatReal(bbox.SizeZ)}}}");
			script.AppendLine($"		*Width {{{FormatReal(line_width)}}}");
			script.AppendLine($"		*o2w {{ *pos {{{FormatReal(bbox.Centre.x)} {FormatReal(bbox.Centre.y)} {FormatReal(bbox.Centre.z)}}} }}");
			script.AppendLine("	}");
		}
		script.AppendLine("}");
		return script.ToString();
	}

	/// <summary>Generated bounds overlay data captured from the UI thread</summary>
	private sealed record BoundsOverlaySetup(string SceneName, List<LDrawObjectInfo> Objects, List<LDrawBoundsInfo> Bounds, string Script);
}

/// <summary>Show-object-bounds command MCP tool surface</summary>
internal sealed partial class LDrawTools
{
	/// <summary>Draw bounds for objects in a running LDraw scene</summary>
	[McpServerTool(Name = "ldraw_show_object_bounds", Title = "Show LDraw object bounds", ReadOnly = false, Destructive = false, Idempotent = true, OpenWorld = false, UseStructuredContent = true)]
	[Description("Draws world-space AABBs for matching objects using a replaceable MCP overlay. Use ldraw_set_diagnostic_modes bboxes_visible for cheaper all-object native bounding boxes.")]
	public Task<LDrawBoundsOverlayResult> ShowObjectBounds(
		[Description("The instance id returned by ldraw_list_instances. Omit to target the broker instance.")] string? instance_id = null,
		[Description("The scene name to modify. Omit to use the first scene.")] string? scene_name = null,
		[Description("Opaque object id returned by ldraw_find_objects or ldraw_list_objects.")] string? object_id = null,
		[Description("Object name filter.")] string? name = null,
		[Description("Object type filter, for example Model, Group, Line, or Box.")] string? type = null,
		[Description("Optional source context id returned by ldraw_get_scene_summary or ldraw_list_scenes.")] string? context_id = null,
		[Description("Optional selected-state filter.")] bool? selected = null,
		[Description("Optional visible-state filter.")] bool? visible = null,
		[Description("True to recursively include child objects when matching.")] bool include_children = true,
		[Description("Name/type matching mode: exact, contains, or regex.")] string match_mode = "contains",
		[Description("True for case-sensitive name/type matching.")] bool case_sensitive = false,
		[Description("Maximum number of matching objects to draw, clamped to 1..1000. Throws if more matches exist.")] int max_count = 200,
		[Description("Overlay id to replace. Omit to use 'diagnostic_bounds'. Reusing the id replaces previous bounds instead of stacking overlays.")] string? overlay_id = null,
		[Description("Display name for the generated overlay source.")] string? overlay_name = null,
		[Description("Bounds colour, e.g. FF00FF00, #00FF00, or Green.")] string? colour = null,
		[Description("Line width in pixels. Must be positive.")] double line_width = 2.0)
	{
		var parameters = ObjectQueryParameters<LDrawShowObjectBoundsParams>(scene_name, object_id, name, type, context_id, selected, visible, include_children, match_mode, case_sensitive, max_count);
		parameters.OverlayId = overlay_id;
		parameters.Name = overlay_name;
		parameters.Colour = colour;
		parameters.LineWidth = line_width;
		return m_broker.ShowObjectBoundsAsync(instance_id, parameters);
	}
}
