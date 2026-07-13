using System;
using System.ComponentModel;
using System.Threading.Tasks;

namespace LDraw.MCP;

/// <summary>Get-selection command instance implementation</summary>
internal sealed partial class LDrawInstanceHost
{
	/// <summary>Return selected objects and selected bounds from the requested scene</summary>
	private Task<LDrawSelectionResult> GetSelectionAsync(LDrawGetSelectionParams parameters)
	{
		return m_model.InvokeAsync(() =>
		{
			var scene = ResolveScene(parameters.SceneName);

			// Selection is represented by existing View3D selected flags so this read does not need Object Manager UI coupling.
			return CreateSelectionResult(scene, "get", parameters.IncludeChildren, parameters.MaxCount);
		}, TimeSpan.FromSeconds(5));
	}
}
