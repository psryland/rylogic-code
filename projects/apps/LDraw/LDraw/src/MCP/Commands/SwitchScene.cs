using System;
using System.ComponentModel;
using System.Threading.Tasks;

namespace LDraw.MCP;

/// <summary>Switch-scene command instance implementation</summary>
internal sealed partial class LDrawInstanceHost
{
	/// <summary>Activate a named scene</summary>
	private Task<LDrawSceneMutationResult> SwitchSceneAsync(LDrawSceneParams parameters)
	{
		return m_model.InvokeAsync(() =>
		{
			var scene = ResolveExplicitScene(parameters.SceneName, "ldraw_switch_scene");
			m_model.ActivateScene(scene);
			return CreateSceneMutationResult("switch_scene", scene.SceneName, CreateSceneInfo(scene), []);
		}, TimeSpan.FromSeconds(5));
	}
}
