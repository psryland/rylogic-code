using System;
using System.ComponentModel;
using System.Threading.Tasks;

namespace LDraw.MCP;

/// <summary>Get-diagnostic-modes command instance implementation</summary>
internal sealed partial class LDrawInstanceHost
{
	/// <summary>Return diagnostic and rendering modes for the requested scene</summary>
	private Task<LDrawDiagnosticModesInfo> GetDiagnosticModesAsync(LDrawDiagnosticModesParams parameters)
	{
		return m_model.InvokeAsync(() =>
		{
			var scene = ResolveScene(parameters.SceneName);
			return CreateDiagnosticModesInfo(scene);
		}, TimeSpan.FromSeconds(5));
	}
}
