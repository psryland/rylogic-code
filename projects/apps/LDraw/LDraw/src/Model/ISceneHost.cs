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

	/// <summary>Show the animation controls where they were last shown. If 'activate' is false, keyboard focus is not taken from the current window</summary>
	void ShowAnimationControls(bool activate);
}
