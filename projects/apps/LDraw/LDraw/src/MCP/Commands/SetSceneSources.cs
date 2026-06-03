using System;
using System.ComponentModel;
using System.Linq;
using System.Threading.Tasks;

namespace LDraw.MCP;

/// <summary>Set-scene-sources command instance implementation</summary>
internal sealed partial class LDrawInstanceHost
{
	/// <summary>Change the MCP overlay sources visible in a scene</summary>
	private async Task<LDrawSceneMutationResult> SetSceneSourcesAsync(LDrawSetSceneSourcesParams parameters)
	{
		await m_overlay_gate.WaitAsync().ConfigureAwait(false);
		try
		{
			return await m_model.InvokeAsync(() =>
			{
				var scene = ResolveExplicitScene(parameters.SceneName, "ldraw_set_scene_sources");
				var mode = ParseSceneSourceMode(parameters.Mode);
				var overlay_states = ResolveOverlayStates(parameters.OverlayIds, parameters.AllOverlays, "ldraw_set_scene_sources", allow_empty: mode == ESceneSourceMode.Clear);
				var changed = ApplyOverlayMembership(scene, mode, overlay_states, parameters.ResetView);
				return CreateSceneMutationResult($"set_scene_sources_{mode.ToString().ToLowerInvariant()}", scene.SceneName, CreateSceneInfo(scene), changed);
			}, TimeSpan.FromSeconds(10)).ConfigureAwait(false);
		}
		finally
		{
			m_overlay_gate.Release();
		}
	}
}
