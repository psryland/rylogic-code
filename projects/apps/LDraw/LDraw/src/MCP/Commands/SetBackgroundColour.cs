using System;
using System.ComponentModel;
using System.Threading.Tasks;

namespace LDraw.MCP;

/// <summary>Set-background-colour command instance implementation</summary>
internal sealed partial class LDrawInstanceHost
{
	/// <summary>Set the background colour for the requested scene</summary>
	private Task<LDrawViewMutationResult> SetBackgroundColourAsync(LDrawSetBackgroundColourParams parameters)
	{
		return m_model.InvokeAsync(() =>
		{
			if (string.IsNullOrWhiteSpace(parameters.Colour))
				throw new InvalidOperationException("ldraw_set_background_colour requires colour.");

			var scene = ResolveScene(parameters.SceneName);
			scene.SceneState.Chart.BackgroundColour = ParseColour(parameters.Colour);
			scene.SceneView.Invalidate();
			return CreateViewMutationResult("set_background_colour", scene);
		}, TimeSpan.FromSeconds(5));
	}
}
