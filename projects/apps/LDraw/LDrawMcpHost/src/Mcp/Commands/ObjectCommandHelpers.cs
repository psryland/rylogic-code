using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Threading.Tasks;
using ModelContextProtocol.Server;

namespace LDraw.MCP;

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
