using System;
using System.ComponentModel;
using System.Linq;
using System.Threading.Tasks;

namespace LDraw.MCP;

/// <summary>List-scenes command instance implementation</summary>
internal sealed partial class LDrawInstanceHost
{
	/// <summary>Return scene summaries for the requested instance</summary>
	private Task<LDrawSceneList> ListScenesAsync(LDrawListScenesParams parameters)
	{
		return m_model.InvokeAsync(() =>
		{
			var scenes = string.IsNullOrWhiteSpace(parameters.SceneName)
				? m_model.Scenes
				: [ResolveScene(parameters.SceneName)];
			return new LDrawSceneList
			{
				Scenes = [..scenes.Select(CreateSceneInfo)],
			};
		}, TimeSpan.FromSeconds(5));
	}
}
