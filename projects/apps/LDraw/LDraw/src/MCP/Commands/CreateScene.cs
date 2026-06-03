using System;
using System.ComponentModel;
using System.Linq;
using System.Threading.Tasks;

namespace LDraw.MCP;

/// <summary>Create-scene command instance implementation</summary>
internal sealed partial class LDrawInstanceHost
{
	/// <summary>Create a scene and optionally show MCP overlays in it</summary>
	private async Task<LDrawSceneMutationResult> CreateSceneAsync(LDrawCreateSceneParams parameters)
	{
		await m_overlay_gate.WaitAsync().ConfigureAwait(false);
		try
		{
			return await m_model.InvokeAsync(() =>
			{
				var overlay_states = ResolveOverlayStates(parameters.OverlayIds, parameters.AllOverlays, "ldraw_create_scene", allow_empty: true);
				var scene = m_model.CreateScene(parameters.SceneName, parameters.Activate);
				var changed = ApplyOverlayMembership(scene, ESceneSourceMode.Add, overlay_states, parameters.ResetView);
				return CreateSceneMutationResult("create_scene", scene.SceneName, CreateSceneInfo(scene), changed);
			}, TimeSpan.FromSeconds(10)).ConfigureAwait(false);
		}
		finally
		{
			m_overlay_gate.Release();
		}
	}
}
