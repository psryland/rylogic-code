using LDraw.UI;

namespace LDraw;

/// <summary>Host-owned scene operations that require UI docking support</summary>
public interface ISceneHost
{
	/// <summary>Create a scene and optionally make it active</summary>
	SceneUI CreateScene(string? name, bool activate);

	/// <summary>Make 'scene' the active visible scene</summary>
	void ActivateScene(SceneUI scene);

	/// <summary>Close and remove 'scene' using the normal UI lifecycle</summary>
	void CloseScene(SceneUI scene);
}
