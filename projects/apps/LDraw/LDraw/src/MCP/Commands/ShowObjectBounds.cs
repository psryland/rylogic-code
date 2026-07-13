using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Rylogic.Gfx;
using Rylogic.Maths;

namespace LDraw.MCP;

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
		var overlay_name = string.IsNullOrWhiteSpace(parameters.OverlayName) ? "MCP Diagnostic Bounds" : parameters.OverlayName;
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
