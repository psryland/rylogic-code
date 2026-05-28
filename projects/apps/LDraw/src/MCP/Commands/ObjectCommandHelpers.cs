using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using System.Text.RegularExpressions;
using LDraw.UI;
using Rylogic.Gfx;
using Rylogic.Maths;

namespace LDraw.MCP;

/// <summary>Shared helpers for object-centric MCP commands</summary>
internal sealed partial class LDrawInstanceHost
{
	/// <summary>Object plus traversal metadata captured from a scene enumeration</summary>
	private sealed record ObjectEntry(View3d.Object Object, int Depth, string Path, View3d.Object? Parent, View3d.Object Root);

	/// <summary>Query result with the scene and truncation state preserved</summary>
	private sealed record ObjectQueryResult(SceneUI Scene, List<ObjectEntry> Matches, bool Truncated);

	/// <summary>Enumerate objects in 'scene' with optional source filtering and child traversal</summary>
	private static List<ObjectEntry> EnumerateObjects(SceneUI scene, Guid? context_id, bool include_children)
	{
		var objects = new List<ObjectEntry>();
		var window = scene.SceneView.Scene.Window;
		var root_index = 0;

		// Root enumeration can be filtered natively by context id; child traversal happens in managed code so each entry gets path metadata.
		bool AddRoot(View3d.Object obj)
		{
			var path = root_index++.ToString(CultureInfo.InvariantCulture);
			AddObject(objects, obj, 0, path, null, obj, include_children);
			return true;
		}

		if (context_id != null)
			window.EnumObjects(AddRoot, id => id == context_id.Value);
		else
			window.EnumObjects(AddRoot);

		return objects;
	}

	/// <summary>Add 'obj' and optionally its descendants to an enumeration list</summary>
	private static void AddObject(List<ObjectEntry> objects, View3d.Object obj, int depth, string path, View3d.Object? parent, View3d.Object root, bool include_children)
	{
		objects.Add(new ObjectEntry(obj, depth, path, parent, root));
		if (!include_children)
			return;

		// Child indices are included in the path only for diagnostics; object ids remain the authoritative opaque identity.
		var children = obj.Children;
		for (int i = 0; i != children.Count; ++i)
		{
			AddObject(objects, children[i], depth + 1, $"{path}/{i}", obj, root, include_children);
		}
	}

	/// <summary>Return objects that match the supplied query parameters</summary>
	private ObjectQueryResult QueryObjects(SceneUI scene, LDrawObjectQueryParams parameters)
	{
		var max_count = ClampObjectCount(parameters.MaxCount);
		var context_id = ParseContextId(parameters.ContextId);
		var matches = new List<ObjectEntry>(max_count);
		var truncated = false;

		// Enumeration is separated from filtering so every command shares the same id/path generation rules.
		foreach (var entry in EnumerateObjects(scene, context_id, parameters.IncludeChildren))
		{
			if (!MatchesObject(entry, parameters))
				continue;

			if (matches.Count == max_count)
			{
				truncated = true;
				break;
			}

			matches.Add(entry);
		}

		return new ObjectQueryResult(scene, matches, truncated);
	}

	/// <summary>Resolve a query that must identify exactly one object</summary>
	private ObjectEntry ResolveSingleObject(ObjectQueryResult query, string command_name)
	{
		if (query.Matches.Count == 0)
			throw new InvalidOperationException($"{command_name} did not match any objects.");
		if (query.Matches.Count != 1)
			throw new InvalidOperationException($"{command_name} matched {query.Matches.Count} objects. Refine the query or use object_id. Candidates: {CandidateSummary(query.Matches)}");

		return query.Matches[0];
	}

	/// <summary>Resolve mutation targets and reject ambiguous truncation</summary>
	private List<ObjectEntry> ResolveMutationTargets(ObjectQueryResult query, string command_name)
	{
		if (query.Matches.Count == 0)
			throw new InvalidOperationException($"{command_name} did not match any objects.");
		if (query.Truncated)
			throw new InvalidOperationException($"{command_name} matched more objects than max_count. Increase max_count or refine the query.");

		return query.Matches;
	}

	/// <summary>Return true if 'entry' satisfies all query filters</summary>
	private static bool MatchesObject(ObjectEntry entry, LDrawObjectQueryParams parameters)
	{
		var obj = entry.Object;
		if (!string.IsNullOrWhiteSpace(parameters.ObjectId) && !string.Equals(ObjectId(obj), NormaliseObjectId(parameters.ObjectId), StringComparison.OrdinalIgnoreCase))
			return false;
		if (!string.IsNullOrWhiteSpace(parameters.Name) && !MatchesText(obj.Name, parameters.Name, parameters.MatchMode, parameters.CaseSensitive))
			return false;
		if (!string.IsNullOrWhiteSpace(parameters.Type) && !MatchesText(obj.Type, parameters.Type, parameters.MatchMode, parameters.CaseSensitive))
			return false;
		if (parameters.Selected != null && IsSelected(obj) != parameters.Selected.Value)
			return false;
		if (parameters.Visible != null && obj.Visible != parameters.Visible.Value)
			return false;

		return true;
	}

	/// <summary>Return true if 'value' matches 'pattern' using the requested mode</summary>
	private static bool MatchesText(string value, string pattern, string match_mode, bool case_sensitive)
	{
		var comparison = case_sensitive ? StringComparison.Ordinal : StringComparison.OrdinalIgnoreCase;
		switch (NormaliseMatchMode(match_mode))
		{
			case "exact":
			{
				return string.Equals(value, pattern, comparison);
			}
			case "contains":
			{
				return value.Contains(pattern, comparison);
			}
			case "regex":
			{
				var options = case_sensitive ? RegexOptions.None : RegexOptions.IgnoreCase;
				return Regex.IsMatch(value, pattern, options);
			}
			default:
			{
				throw new ArgumentOutOfRangeException(nameof(match_mode), match_mode, "Unknown object match mode.");
			}
		}
	}

	/// <summary>Normalise a match mode string and validate it</summary>
	private static string NormaliseMatchMode(string? match_mode)
	{
		var mode = string.IsNullOrWhiteSpace(match_mode) ? "contains" : match_mode.Trim().ToLowerInvariant();
		switch (mode)
		{
			case "exact":
			case "contains":
			case "regex":
			{
				return mode;
			}
			default:
			{
				throw new ArgumentOutOfRangeException(nameof(match_mode), match_mode, "Object match mode must be exact, contains, or regex.");
			}
		}
	}

	/// <summary>Return the current selected-state flag for an object</summary>
	private static bool IsSelected(View3d.Object obj)
	{
		return obj.Flags.HasFlag(View3d.ELdrFlags.Selected);
	}

	/// <summary>Set the selected-state flag for an object if it needs changing</summary>
	private static void SetSelected(View3d.Object obj, bool selected)
	{
		if (IsSelected(obj) == selected)
			return;

		// Use the existing View3D selected flag only; MCP remains isolated and does not depend on LDraw UI internals.
		obj.FlagsSet(View3d.ELdrFlags.Selected, selected, null);
	}

	/// <summary>Capture selected object ids from a scene</summary>
	private static HashSet<string> CaptureSelectionIds(SceneUI scene)
	{
		return new HashSet<string>(
			EnumerateObjects(scene, null, include_children: true).Where(x => IsSelected(x.Object)).Select(x => ObjectId(x.Object)),
			StringComparer.OrdinalIgnoreCase);
	}

	/// <summary>Replace scene selection with 'selected_ids'</summary>
	private static void RestoreSelection(SceneUI scene, HashSet<string> selected_ids)
	{
		foreach (var entry in EnumerateObjects(scene, null, include_children: true))
			SetSelected(entry.Object, selected_ids.Contains(ObjectId(entry.Object)));
	}

	/// <summary>Return selected objects for a scene with a response count limit</summary>
	private static List<ObjectEntry> SelectedObjects(SceneUI scene, bool include_children, int max_count)
	{
		var objects = EnumerateObjects(scene, null, include_children);
		return [..objects.Where(x => IsSelected(x.Object)).Take(ClampObjectCount(max_count))];
	}

	/// <summary>Return the current selected-object bounds for a scene</summary>
	private static BBox SelectedBounds(SceneUI scene)
	{
		return scene.SceneView.Scene.Window.SceneBounds(View3d.ESceneBounds.Selected);
	}

	/// <summary>Build public DTO objects from traversal entries</summary>
	private static List<LDrawObjectInfo> CreateObjectInfos(IEnumerable<ObjectEntry> entries)
	{
		return [..entries.Select(CreateObjectInfo)];
	}

	/// <summary>Create object DTO data for a traversed View3D object</summary>
	private static LDrawObjectInfo CreateObjectInfo(ObjectEntry entry)
	{
		var obj = entry.Object;
		var o2p = obj.O2P;
		var o2w = obj.O2WGet(null);
		return new LDrawObjectInfo
		{
			ObjectId = ObjectId(obj),
			Name = obj.Name,
			Type = obj.Type,
			ContextId = obj.ContextId.ToString("D"),
			ParentObjectId = entry.Parent != null ? ObjectId(entry.Parent) : string.Empty,
			RootObjectId = ObjectId(entry.Root),
			Path = entry.Path,
			Depth = entry.Depth,
			Visible = obj.Visible,
			Selected = IsSelected(obj),
			Colour = obj.Colour.ToString(),
			BaseColour = obj.BaseColour.ToString(),
			ChildCount = obj.ChildCount,
			ModelBounds = LDrawBoundsInfo.From(obj.BBoxMS(View3d.EBBoxFlags.IncludeChildren)),
			ObjectToParent = [..o2p.ToArray().Select(x => (double)x)],
			ObjectToWorld = [..o2w.ToArray().Select(x => (double)x)],
			Reflectivity = obj.Reflectivity,
			Wireframe = obj.Wireframe,
			ShowNormals = obj.ShowNormals,
			SortGroup = obj.SortGroup.ToString(),
			Flags = obj.Flags.ToString(),
			NuggetFlags = obj.NuggetFlags.ToString(),
			NuggetTint = obj.NuggetTint.ToString(),
		};
	}

	/// <summary>Create a process-lifetime opaque object id from a View3D object wrapper</summary>
	private static string ObjectId(View3d.Object obj)
	{
		return obj.Handle.ToInt64().ToString("x16", CultureInfo.InvariantCulture);
	}

	/// <summary>Normalise an incoming object id for comparison</summary>
	private static string NormaliseObjectId(string object_id)
	{
		var id = object_id.Trim();
		if (id.StartsWith("0x", StringComparison.OrdinalIgnoreCase))
			id = id[2..];
		return id;
	}

	/// <summary>Clamp object result counts to a bounded range</summary>
	private static int ClampObjectCount(int max_count)
	{
		return Math.Clamp(max_count <= 0 ? 200 : max_count, 1, 1000);
	}

	/// <summary>Return a compact description of query candidates for an ambiguity error</summary>
	private static string CandidateSummary(IEnumerable<ObjectEntry> entries)
	{
		return string.Join("; ", entries.Take(5).Select(x => $"{x.Object.Name} ({x.Object.Type}, id={ObjectId(x.Object)}, path={x.Path})"));
	}

	/// <summary>Parse a user-supplied colour value</summary>
	private static Colour32 ParseColour(string colour)
	{
		var value = colour.Trim();
		if (value.StartsWith("0x", StringComparison.OrdinalIgnoreCase))
			value = value[2..];
		return Colour32.Parse(value);
	}
}

/// <summary>Shared argument helpers for object-centric MCP tool methods</summary>
internal sealed partial class LDrawTools
{
	/// <summary>Create typed object query parameters from common MCP tool arguments</summary>
	private static T ObjectQueryParameters<T>(
		string? scene_name,
		string? object_id,
		string? name,
		string? type,
		string? context_id,
		bool? selected,
		bool? visible,
		bool include_children,
		string match_mode,
		bool case_sensitive,
		int max_count)
		where T : LDrawObjectQueryParams, new()
	{
		// All object-centric tools accept the same query surface so an object found by one command can be refined by another.
		return new T
		{
			SceneName = scene_name,
			ObjectId = object_id,
			Name = name,
			Type = type,
			ContextId = context_id,
			Selected = selected,
			Visible = visible,
			IncludeChildren = include_children,
			MatchMode = match_mode,
			CaseSensitive = case_sensitive,
			MaxCount = max_count,
		};
	}
}
