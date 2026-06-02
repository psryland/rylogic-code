using System;
using System.Linq;
using LDraw.UI;
using Rylogic.Gui.WPF;

namespace LDraw;

public partial class MainWindow :ISceneHost
{
	/// <summary>Create a scene and add it to the dock container</summary>
	SceneUI ISceneHost.CreateScene(string? name, bool activate)
	{
		var scene_name = ResolveNewSceneName(name);
		var scene = new SceneUI(Model, scene_name);
		Model.Scenes.Add(scene);
		m_dc.Add(scene, EDockSite.Centre);
		if (activate)
			m_dc.FindAndShow(scene);
		return scene;
	}

	/// <summary>Make 'scene' the active dock content</summary>
	void ISceneHost.ActivateScene(SceneUI scene)
	{
		if (!Model.Scenes.Contains(scene))
			throw new InvalidOperationException($"Scene '{scene.SceneName}' is not owned by this LDraw instance.");
		if (scene.DockControl.DockContainer != m_dc)
			m_dc.Add(scene, EDockSite.Centre);

		m_dc.FindAndShow(scene);
	}

	/// <summary>Close 'scene' through its existing command path</summary>
	void ISceneHost.CloseScene(SceneUI scene)
	{
		if (!Model.Scenes.Contains(scene))
			throw new InvalidOperationException($"Scene '{scene.SceneName}' is not owned by this LDraw instance.");

		scene.CloseScene.Execute(null);
	}

	/// <summary>Resolve a caller-provided scene name or generate a unique name</summary>
	private string ResolveNewSceneName(string? name)
	{
		var scene_name = string.IsNullOrWhiteSpace(name) ? Model.GenerateSceneName() : name.Trim();
		if (Model.Scenes.Any(scene => string.Equals(scene.SceneName, scene_name, StringComparison.OrdinalIgnoreCase)))
			throw new InvalidOperationException($"A scene named '{scene_name}' already exists.");
		return scene_name;
	}
}
