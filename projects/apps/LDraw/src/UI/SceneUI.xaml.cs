using System;
using System.Collections.Generic;
using System.Collections.Specialized;
using System.ComponentModel;
using System.Diagnostics;
using System.Linq;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Input;
using LDraw.Dialogs;
using Rylogic.Common;
using Rylogic.Extn;
using Rylogic.Gfx;
using Rylogic.Gui.WPF;
using Rylogic.Maths;
using Rylogic.Utility;

namespace LDraw.UI
{
	[DebuggerDisplay("{SceneName,nq}")]
	public sealed partial class SceneUI :UserControl, IDockable, IDisposable, INotifyPropertyChanged, IView3dCMenuContext, IChartCMenuContext
	{
		public SceneUI(Model model, string name)
		{
			InitializeComponent();
			DockControl = new DockControl(this, name)
			{
				ShowTitle = false,
				TabText = name,
				TabCMenu = this.FindCMenu("TabCMenu", this),
				DestroyOnClose = true,
			};
			OtherScenesView = new ListCollectionView(new List<SceneWrapper>());
			Model = model;
			SceneView = m_scene;
			SceneName = name;
			SceneState = Model.Profile.SceneState.get(name);
			SceneView.Background = Colour32.LightSteelBlue.ToMediaBrush();
			SceneView.Scene.Window.SetLightSource(v4.Origin, Math_.Normalise(new v4(0, 0, -1, 0)), true);
			SceneView.Scene.Window.OnRendering += HandleSceneRendering;
			SceneView.Scene.ContextMenu.DataContext = this;
			SceneView.Scene.DefaultKeyboardShortcuts = true;
			Links = new List<ChartLink>();

			ClearScene = Command.Create(this, ClearSceneInternal);
			RenameScene = Command.Create(this, RenameSceneInternal);
			CloseScene = Command.Create(this, CloseSceneInternal);
			LinkCameras = Command.Create(this, LinkCamerasInternal);

			PopulateOtherScenes();

			SceneView.LinkCamera = LinkCameras;
			SceneView.CanLinkCamera = OtherScenes.Count > 1;

			DataContext = this;
		}
		public void Dispose()
		{
			OtherScenes.Clear();
			SceneState = null!;
			SceneView = null!;
			Model = null!;
			DockControl = null!;
			m_scene.Dispose();
		}
		protected override void OnPreviewKeyDown(KeyEventArgs e)
		{
			base.OnPreviewKeyDown(e);
			if (e.Handled)
				return;

			switch (e.Key)
			{
				case Key.Space:
				{
					// Show the dockable scene manager, floating it if currently hidden
					if (DockControl.DockContainer is DockContainer dc)
					{
						var mgr = dc.FindContent<ObjectManagerUI>().FirstOrDefault();
						if (mgr != null)
						{
							mgr.SetScene(this);

							// If it's tucked away in auto-hide, float it so it's visible
							if (mgr.DockControl.IsAutoHide)
								mgr.DockControl.IsFloating = true;

							mgr.DockControl.IsActiveContent = true;
						}
					}
					e.Handled = true;
					break;
				}
				default:
				{
					// Let other keys fall through to the default Key handler
					break;
				}
			}
		}

		/// <summary>Provides support for the DockContainer</summary>
		public DockControl DockControl
		{
			get;
			private set
			{
				if (field == value) return;
				if (field != null)
				{
					field.ActiveChanged -= HandleSceneActive;
					field.SavingLayout -= HandleSavingLayout;
					field.LoadingLayout -= HandleLoadingLayout;
					Util.Dispose(ref field!);
				}
				field = value;
				if (field != null)
				{
					field.LoadingLayout += HandleLoadingLayout;
					field.SavingLayout += HandleSavingLayout;
					field.ActiveChanged += HandleSceneActive;
				}

				// Handlers
				void HandleSceneActive(object? sender, ActiveContentChangedEventArgs args)
				{
				}
				void HandleLoadingLayout(object? sender, DockContainerLoadingLayoutEventArgs e)
				{
				}
				void HandleSavingLayout(object? sender, DockContainerSavingLayoutEventArgs e)
				{
				}
			}
		} = null!;

		/// <summary>App logic</summary>
		public Model Model
		{
			get;
			private set
			{
				if (field == value) return;
				if (field != null)
				{
					field.PropertyChanged -= HandlePropertyChanged;
					field.Scenes.CollectionChanged -= HandleScenesCollectionChanged;
					field.Scenes.Remove(this);
				}
				field = value;
				if (field != null)
				{
					// Don't add to field.Scenes, that's the caller's choice
					field.Scenes.CollectionChanged += HandleScenesCollectionChanged;
					field.PropertyChanged += HandlePropertyChanged;
				}

				// Handlers
				void HandleScenesCollectionChanged(object? sender, NotifyCollectionChangedEventArgs e)
				{
					// Update the collection of 'other' scenes
					PopulateOtherScenes();
					CleanupLinks();

					// Enable/Disable the Link Cameras option in the scene context menu
					SceneView.CanLinkCamera = OtherScenes.Count > 1;
				}
				void HandlePropertyChanged(object? sender, PropertyChangedEventArgs e)
				{
					switch (e.PropertyName)
					{
						case nameof(Model.Profile):
						{
							// Nothing to do, the scenes will be deleted and recreated
							break;
						}
					}
				}
			}
		} = null!;
		
		/// <summary>Scene state settings</summary>
		public SceneStateData SceneState
		{
			get;
			private set
			{
				if (field == value) return;
				if (field != null)
				{
					SceneView.Scene.Window.OnSettingsChanged -= HandleSceneSettingChanged;
					field.SettingChange -= HandleSettingChange;
					SceneView.Options = new ChartControl.OptionsData();
				}
				field = value;
				if (field != null)
				{
					SceneView.Options = field.Chart;
					field.SettingChange += HandleSettingChange;
					SceneView.Scene.Window.OnSettingsChanged += HandleSceneSettingChanged;
					field.NotifyAllSettingsChanged();
				}

				// Handlers
				void HandleSettingChange(object? sender, SettingChangeEventArgs e)
				{
					// Handle notification that a SceneState value has changed
					if (e.Before)
						return;

					switch (e.Key)
					{
						case nameof(SceneStateData.Chart):
						{
							SceneView.Options = SceneState.Chart;
							break;
						}
						case nameof(SceneStateData.AlignDirection):
						{
							SceneView.Scene.AlignDirection = SceneState.AlignDirection;
							break;
						}
						case nameof(SceneStateData.ViewPreset):
						{
							SceneView.Scene.ViewPreset = SceneState.ViewPreset;
							break;
						}
					}
				}
				void HandleSceneSettingChanged(object? sender, View3d.SettingChangeEventArgs e)
				{
					// Handle notification that a View3d.Window property has changed
					var cmenu = SceneView.Scene.ContextMenu?.DataContext as IView3dCMenu;

					// Persist settings
					if (Bit.AllSet(e.Setting, View3d.ESettings.Camera))
					{
						if (Bit.AllSet(e.Setting, View3d.ESettings.Camera_Orthographic))
						{
							SceneState.Chart.Orthographic = SceneView.Camera.Orthographic;
							cmenu?.NotifyPropertyChanged(nameof(IView3dCMenu.Orthographic));
						}
						if (Bit.AllSet(e.Setting, View3d.ESettings.Camera_AlignAxis))
						{
							SceneState.AlignDirection = AlignDirection_.FromAxis(SceneView.Camera.AlignAxis);
							cmenu?.NotifyPropertyChanged(nameof(IView3dCMenu.AlignDirection));
						}
					}
				}
			}
		} = null!;

		/// <summary>The 3d part of the scene (i.e. the chart control)</summary>
		public ChartControl SceneView
		{
			get;
			private set
			{
				if (field == value) return;
				if (field != null)
				{
					field.PropertyChanged -= HandlePropertyChanged;
				}
				field = value;
				if (field != null)
				{
					field.PropertyChanged += HandlePropertyChanged;
				}

				// Handler
				void HandlePropertyChanged(object? sender, PropertyChangedEventArgs e)
				{
					// Forward property changed from the ChartControl.
					// Probably should only forward properties of IChartCMenu and IView3dCMenu.
					if (e.PropertyName is string prop_name)
						NotifyPropertyChanged(prop_name);
				}
			}
		} = null!;

		/// <summary>The camera focus point</summary>
		public v4 FocusPoint
		{
			get => SceneView.Camera.FocusPoint;
			set
			{
				if (FocusPoint == value) return;
				SceneView.Camera.FocusPoint = value;
				NotifyPropertyChanged(nameof(FocusPoint));
				SceneView.Invalidate();
			}
		}

		/// <summary>Text description of the object under the mouse pointer</summary>
		public string HoveredObjectInfo => SceneView.HoveredObjectInfo;

		/// <summary>True if object info display is enabled</summary>
		public bool ObjectInfoEnabled => SceneView.ObjectInfoEnabled;

		/// <summary>The name assigned to this scene UI</summary>
		public string SceneName
		{
			get;
			set
			{
				if (field == value) return;
				field = value;
				DockControl.TabText = field;
				NotifyPropertyChanged(nameof(SceneName));
			}
		} = null!;

		/// <summary>Other available scenes</summary>
		public ICollectionView OtherScenesView { get; }
		private List<SceneWrapper> OtherScenes => (List<SceneWrapper>)OtherScenesView.SourceCollection;
		private void PopulateOtherScenes()
		{
			var other_scenes = Model.Scenes.Except(this)
				.Select(x => new SceneWrapper(x))
				.Prepend(SceneWrapper.NullScene);

			// Update the collection
			OtherScenes.Sync(other_scenes);
			OtherScenesView.Refresh();
			if (OtherScenesView.CurrentItem == null)
				OtherScenesView.MoveCurrentToFirst();

			// Notify changed
			NotifyPropertyChanged(nameof(OtherScenes));
		}

		/// <summary>The links to other scenes</summary>
		public List<ChartLink> Links { get; }
		public void CleanupLinks()
		{
			Links.RemoveAll(x => x.Source == null || x.Target == null || (x.CamLink == ELinkCameras.None && x.AxisLink == ELinkAxes.None));
		}

		/// <summary>True if the animation UI is visible</summary>
		public bool AnimationUI
		{
			get;
			set
			{
				if (field == value) return;
				field = value;
				NotifyPropertyChanged(nameof(AnimationUI));
			}
		}

		/// <summary>Remove all objects from the scene</summary>
		public Command ClearScene { get; }
		private void ClearSceneInternal()
		{
			Model.Clear(this);
		}

		/// <summary>Rename the scene tab</summary>
		public Command RenameScene { get; }
		private void RenameSceneInternal()
		{
			var prompt = new PromptUI(Window.GetWindow(this))
			{
				Title = "Rename Scene",
				Prompt = "Set the label for this scene",
				Value = SceneName,
				ShowWrapCheckbox = false,
			};
			if (prompt.ShowDialog() == true)
			{
				SceneName = prompt.Value;
			}
		}

		/// <summary>Close and remove this scene</summary>
		public Command CloseScene { get; }
		private void CloseSceneInternal()
		{
			Model.Scenes.Remove(this);
			Dispose();
		}

		/// <summary>Show the link cameras UI</summary>
		public Command LinkCameras { get; }
		private void LinkCamerasInternal()
		{
			if (m_link_cameras_ui == null)
			{
				m_link_cameras_ui = new LinkCamerasUI(Window.GetWindow(this), Model);
				m_link_cameras_ui.Closed += delegate { m_link_cameras_ui = null; };
				m_link_cameras_ui.Show();
			}
			m_link_cameras_ui.Focus();
		}
		private LinkCamerasUI? m_link_cameras_ui;

		/// <summary>Event from the view3d window during rendering</summary>
		private void HandleSceneRendering(object? sender, EventArgs e)
		{
			NotifyPropertyChanged(nameof(FocusPoint));
		}

		/// <inheritdoc/>
		public IChartCMenu ChartCMenuContext => SceneView;

		/// <inheritdoc/>
		public IView3dCMenu View3dCMenuContext => SceneView;

		/// <inheritdoc/>
		public event PropertyChangedEventHandler? PropertyChanged;
		public void NotifyPropertyChanged(string prop_name) => PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(prop_name));
	}
}
