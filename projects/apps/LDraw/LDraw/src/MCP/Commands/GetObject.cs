using System;
using System.ComponentModel;
using System.Threading.Tasks;

namespace LDraw.MCP;

/// <summary>Get-object command instance implementation</summary>
internal sealed partial class LDrawInstanceHost
{
	/// <summary>Return exactly one object matching 'parameters'</summary>
	private Task<LDrawObjectInfo> GetObjectAsync(LDrawGetObjectParams parameters)
	{
		return m_model.InvokeAsync(() =>
		{
			var scene = ResolveScene(parameters.SceneName);
			var query = QueryObjects(scene, parameters);
			var entry = ResolveSingleObject(query, "ldraw_get_object");
			return CreateObjectInfo(entry);
		}, TimeSpan.FromSeconds(5));
	}
}
