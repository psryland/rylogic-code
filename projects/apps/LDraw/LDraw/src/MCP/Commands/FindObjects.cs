using System;
using System.ComponentModel;
using System.Threading.Tasks;

namespace LDraw.MCP;

/// <summary>Find-object command instance implementation</summary>
internal sealed partial class LDrawInstanceHost
{
	/// <summary>Return objects matching 'parameters' from the requested scene</summary>
	private Task<LDrawObjectList> FindObjectsAsync(LDrawObjectQueryParams parameters)
	{
		return m_model.InvokeAsync(() =>
		{
			var scene = ResolveScene(parameters.SceneName);
			var query = QueryObjects(scene, parameters);

			// Query results are returned with the same object DTO shape as list_objects so clients can use object_id for follow-up calls.
			return new LDrawObjectList
			{
				SceneName = query.Scene.SceneName,
				Objects = CreateObjectInfos(query.Matches),
				Truncated = query.Truncated,
			};
		}, TimeSpan.FromSeconds(5));
	}
}
