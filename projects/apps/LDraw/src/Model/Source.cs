using System;
using System.Collections.Generic;
using System.Collections.Specialized;
using System.ComponentModel;
using System.Diagnostics;
using System.Linq;
using LDraw.UI;
using Rylogic.Common;
using Rylogic.Gfx;
using Rylogic.Utility;

namespace LDraw
{
	[DebuggerDisplay("{Name,nq}")]
	public class Source : IDisposable, INotifyPropertyChanged
	{
		// Notes:
		//  - Model for an LDraw source
		//  - View component is the 'SourceItemUI'

		public Source(Model model, View3d.Source source)
		{
			Model = model;
			View3dSource = source;
		}
		public void Dispose()
		{
			// Remove objects from all scenes
			m_selected_scene_names.Clear();
			PopulateScenes();

			// Detach all external subscribers before nulling members.
			// Without this, a WPF binding that still references this Source
			// could fire a PropertyChanged handler that accesses Model after disposal.
			PropertyChanged = null;

			View3dSource = null!;
			Model = null!;
			GC.SuppressFinalize(this);
		}

		/// <summary>Access to the app data</summary>
		public Model Model
		{
			get;
			private set
			{
				if (field == value) return;
				if (field != null)
				{
					field.Scenes.CollectionChanged -= HandleScenesCollectionChanged;
					field.PropertyChanged -= HandlePropertyChanged;
				}
				field = value;
				if (field != null)
				{
					field.PropertyChanged += HandlePropertyChanged;
					field.Scenes.CollectionChanged += HandleScenesCollectionChanged;
				}

				void HandlePropertyChanged(object? sender, PropertyChangedEventArgs e)
				{
					switch (e.PropertyName)
					{
						case nameof(Model.Profile):
						{
							PopulateScenes();
							NotifyPropertyChanged(nameof(AvailableScenes));
							break;
						}
					}
				}
				void HandleScenesCollectionChanged(object? sender, NotifyCollectionChangedEventArgs e)
				{
					NotifyPropertyChanged(nameof(AvailableScenes));
				}
			}
		} = null!;

		/// <summary>The native LDraw source</summary>
		private View3d.Source View3dSource
		{
			get;
			set
			{
				if (field == value) return;
				if (field != null)
				{
					field.PropertyChanged -= HandleSourcePropertyChanged;
					Util.Dispose(ref field!);
				}
				field = value;
				if (field != null)
				{
					field.PropertyChanged += HandleSourcePropertyChanged;
				}
			
				void HandleSourcePropertyChanged(object? sender, PropertyChangedEventArgs e)
				{
					if (e.PropertyName != null)
						NotifyPropertyChanged(e.PropertyName);
				}
			}
		} = null!;

		/// <summary>The context id associated with the source</summary>
		public Guid ContextId => View3dSource.ContextId;

		/// <summary>The name of the source</summary>
		public string Name
		{
			get => View3dSource?.Name ?? string.Empty;
			set => View3dSource.Name = value;
		}

		/// <summary>The filepath associated with the source</summary>
		public string FilePath => View3dSource?.Info.FilePath ?? string.Empty;

		/// <summary>The number of objects in this source</summary>
		public int ObjectCount => View3dSource?.Info.ObjectCount ?? 0;

		/// <summary>True if there is a scene to render to</summary>
		public bool CanRender => SelectedScenes.Any();

		/// <summary>True if this source has a file that can be edited</summary>
		public bool CanEdit => View3dSource?.Info is View3d.SourceInfo info && Path_.FileExists(info.FilePath) && info.TextFormat;

		/// <summary>True if the source exists</summary>
		public bool Exists => FilePath.Length == 0 || Path_.FileExists(FilePath); // Has no associated file, or it does and the file exists

		/// <summary>The names of the scenes to add this source to</summary>
		public void ShowInScenes(IEnumerable<SceneUI> scenes, bool show)
		{
			// Notes:
			//  - The selected scene names may contain names for scenes that don't exist if the profile has changed.
			//    This is deliberate because I want switching back to the original profile to re-add to those scenes.
			foreach (var scene in scenes)
			{
				if (show)
					m_selected_scene_names.Add(scene.SceneName);
				else
					m_selected_scene_names.Remove(scene.SceneName);
			}

			PopulateScenes();
			NotifyPropertyChanged(nameof(SelectedScenes));
		}
		private HashSet<string> m_selected_scene_names = [];

		/// <summary>Add objects from this source to all selected scenes</summary>
		private void PopulateScenes()
		{
			foreach (var scene in AvailableScenes)
			{
				if (m_selected_scene_names.Contains(scene.SceneName))
					Model.AddObjects(scene, ContextId);
				else
					Model.Clear(scene, ContextId);
			}
		}

		/// <summary>The scenes to draw this source in</summary>
		public IEnumerable<SceneUI> SelectedScenes => Model.Scenes.Where(x => m_selected_scene_names.Contains(x.SceneName));

		/// <summary>The available scenes to render this source in</summary>
		public IEnumerable<SceneUI> AvailableScenes => Model.Scenes;

		/// <summary>Remove this source</summary>
		public void Remove()
		{
			View3dSource.Remove();
		}

		/// <summary>Reload the objects from this source</summary>
		public void Reload()
		{
			View3dSource.Reload();
		}

		/// <summary>Raised when the source has changed</summary>
		public event EventHandler SourceChanged
		{
			add { View3dSource.SourceChanged += value; }
			remove { View3dSource.SourceChanged -= value; }
		}

		/// <inheritdoc/>
		public event PropertyChangedEventHandler? PropertyChanged;
		private void NotifyPropertyChanged(string prop_name) => PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(prop_name));
	}
}
