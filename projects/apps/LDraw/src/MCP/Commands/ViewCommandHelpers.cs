using System;
using Rylogic.Gfx;

namespace LDraw.MCP;

/// <summary>Shared helpers for view-setting MCP commands</summary>
internal sealed partial class LDrawInstanceHost
{
	/// <summary>Parse a named camera align direction</summary>
	private static EAlignDirection ParseAlignDirection(string? align_direction)
	{
		if (string.IsNullOrWhiteSpace(align_direction))
			throw new InvalidOperationException("Camera align direction is required.");
		if (!Enum.TryParse<EAlignDirection>(align_direction.Trim(), ignoreCase: true, out var direction))
			throw new InvalidOperationException("Camera align direction must be one of: None, PosX, NegX, PosY, NegY, PosZ, or NegZ.");
		return direction;
	}

	/// <summary>Create a view mutation result from current scene state</summary>
	private static LDrawViewMutationResult CreateViewMutationResult(string action, UI.SceneUI scene)
	{
		return new LDrawViewMutationResult
		{
			Action = action,
			SceneName = scene.SceneName,
			ViewSettings = CreateViewSettingsInfo(scene),
		};
	}
}
