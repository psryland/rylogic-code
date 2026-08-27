using System;
using System.ComponentModel;
using System.Linq;
using System.Threading.Tasks;

namespace LDraw.MCP;

/// <summary>Close-scene command instance implementation</summary>
internal sealed partial class LDrawInstanceHost
{
	/// <summary>Close a scene when it contains no user-loaded sources</summary>
	private async Task<LDrawSceneMutationResult> CloseSceneAsync(LDrawSceneParams parameters)
	{
		await m_overlay_gate.WaitAsync().ConfigureAwait(false);
		try
		{
			return await m_model.InvokeAsync(() =>
			{
				var scene = ResolveExplicitScene(parameters.SceneName, "ldraw_close_scene");
				var user_sources = UserSourcesInScene(scene);
				if (user_sources.Count != 0)
				{
					var names = string.Join(", ", user_sources.Select(source => $"{source.Name} ({source.ContextId:D})"));
					throw new InvalidOperationException($"ldraw_close_scene cannot close scene '{scene.SceneName}' because it contains {user_sources.Count} user source(s): {names}.");
				}

				var scene_name = scene.SceneName;
				var scene_info = CreateSceneInfo(scene);
				var overlays = OverlayStatesInScene(scene);
				foreach (var state in overlays)
					SetOverlaySceneName(state, scene_name, visible: false);

				m_model.CloseScene(scene);
				return CreateSceneMutationResult("close_scene", scene_name, scene_info, overlays);
			}, TimeSpan.FromSeconds(10)).ConfigureAwait(false);
		}
		finally
		{
			m_overlay_gate.Release();
		}
	}
}
