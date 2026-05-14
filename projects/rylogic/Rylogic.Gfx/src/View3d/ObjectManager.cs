using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using Rylogic.Extn;

namespace Rylogic.Gfx
{
	public sealed partial class View3d
	{
		public sealed class ObjectManager :IDisposable, INotifyPropertyChanged
		{
			// Notes:
			//  - This class is the View-Model for the object manager.
			//  - The purpose of this class is to manage the collection of objects and handle deleted objects.
			//  - UI frameworks need to provide binding wrappers.
			public ObjectManager(Window window, IEnumerable<Guid> excluded)
			{
				Window = window;
				Exclude = [.. Enumerable.Concat(ExcludeCtxIds, excluded)];
				Objects = [];
				SelectedObjects = [];

				SyncObjectsWithScene();
			}
			public void Dispose()
			{
				Window = null!;
			}

			/// <summary>Context IDs of objects to always exclude from user-facing management</summary>
			public static HashSet<Guid> ExcludeCtxIds { get; } = [];

			/// <summary>The scene that this manager is associated with</summary>
			public Window Window
			{
				get;
				private set
				{
					if (field == value) return;
					if (field != null)
					{
						field.OnSceneChanged -= HandleSceneChanged;
					}
					field = value;
					if (field != null)
					{
						field.OnSceneChanged += HandleSceneChanged;
					}

					// Handlers
					void HandleSceneChanged(object? sender, SceneChangedEventArgs args)
					{
						// Ignore if only excluded context ids have changed
						if (args.ContextIds.All(x => Exclude.Contains(x)))
							return;

						SyncObjectsWithScene();
					}
				}
			} = null!;

			/// <summary>Context Ids of objects not to show in the object manager</summary>
			public HashSet<Guid> Exclude { get; }

			/// <summary>The collection of objects in this scene (top level tree nodes)</summary>
			public List<Object> Objects { get; }

			/// <summary>Selected objects</summary>
			public HashSet<Object> SelectedObjects { get; }

			/// <summary>Update the 'Objects' collection to match the objects in the scene</summary>
			private void SyncObjectsWithScene()
			{
				// Read the objects from the scene
				var objects = new HashSet<Object>();
				Window.EnumObjects(objects.Add, x => !Exclude.Contains(x));
				Objects.Sync(objects);
				SelectedObjects.RemoveWhere(x => !objects.Contains(x));
				Objects.Sort((l,r) => l.Name.CompareTo(r.Name));
				NotifyPropertyChanged(nameof(Objects));
			}

			/// <inheritdoc/>
			public event PropertyChangedEventHandler? PropertyChanged;
			private void NotifyPropertyChanged(string prop_name) => PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(prop_name));
		}
	}
}
