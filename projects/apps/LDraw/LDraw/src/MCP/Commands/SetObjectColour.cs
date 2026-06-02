using System;
using System.ComponentModel;
using System.Threading.Tasks;

namespace LDraw.MCP;

/// <summary>Set-object-colour command instance implementation</summary>
internal sealed partial class LDrawInstanceHost
{
	/// <summary>Set or reset colour for matching objects in the requested scene</summary>
	private Task<LDrawObjectMutationResult> SetObjectColourAsync(LDrawSetObjectColourParams parameters)
	{
		return m_model.InvokeAsync(() =>
		{
			if (!parameters.Reset && string.IsNullOrWhiteSpace(parameters.Colour))
				throw new InvalidOperationException("ldraw_set_object_colour requires colour unless reset is true.");

			var scene = ResolveScene(parameters.SceneName);
			var query = QueryObjects(scene, parameters);
			var targets = ResolveMutationTargets(query, "ldraw_set_object_colour");
			var name = parameters.Recursive ? string.Empty : null;
			if (parameters.Reset)
			{
				foreach (var entry in targets)
					entry.Object.ResetColour(name);
			}
			else
			{
				var colour = ParseColour(parameters.Colour!);
				foreach (var entry in targets)
					entry.Object.ColourSet(parameters.BaseColour, colour, name);
			}

			scene.SceneView.Invalidate();
			return new LDrawObjectMutationResult
			{
				SceneName = scene.SceneName,
				Action = parameters.Reset ? "reset_object_colour" : parameters.BaseColour ? "set_object_base_colour" : "set_object_colour",
				Objects = CreateObjectInfos(targets),
			};
		}, TimeSpan.FromSeconds(10));
	}
}
