using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.IO;
using System.Linq;
using System.Windows;
using System.Windows.Data;
using System.Windows.Input;
using LDraw.Dialogs;
using LDraw.UI;
using Microsoft.Win32;
using Rylogic.Common;
using Rylogic.Extn;
using Rylogic.Gfx;
using Rylogic.Gui.WPF;
using Rylogic.Utility;

namespace LDraw
{
	public partial class MainWindow :Window, INotifyPropertyChanged
	{
		public MainWindow(Model model)
		{
			InitializeComponent();
			Model = model;
			StatusMessage = "Idle";

			Profiles = CollectionViewSource.GetDefaultView(Model.Settings.Profiles);
			Profiles.MoveCurrentTo(Model.Profile);

			// Commands
			NewScript = Command.Create(this, NewScriptInternal);
			NewScene = Command.Create(this, NewSceneInternal);
			OpenFile = Command.Create(this, OpenFileInternal);
			SaveFile = Command.Create(this, SaveFileInternal);
			SaveFileAs = Command.Create(this, SaveFileAsInternal);
			ToggleStreaming = Command.Create(this, ToggleStreamingInternal);
			ShowPreferences = Command.Create(this, ShowPreferencesInternal);
			ShowExampleScript = Command.Create(this, ShowExampleScriptInternal);
			ShowAbout = Command.Create(this, ShowAboutInternal);
			Exit = Command.Create(this, ExitInternal);

			m_recent_files.Import(Model.Settings.RecentFiles);
			m_recent_files.RecentFileSelected = fp => AddFileSourceAsync(fp);

			InitDockContainer();
			LoadProfile();

			DataContext = this;
		}
		protected override void OnClosing(CancelEventArgs e)
		{
			m_dc.LayoutChanged -= SaveLayout;
			base.OnClosing(e);
		}
		protected override void OnClosed(EventArgs e)
		{
			Util.DisposeRange(m_dc.AllContent.OfType<IDisposable>());
			m_dc.Dispose();
			Model = null!;
			base.OnClosed(e);
		}
		protected override void OnPreviewKeyDown(KeyEventArgs e)
		{
			// Handle keyboard shortcuts that may be swallowed by child HWNDs (e.g. the 3D viewport)
			if (Keyboard.Modifiers == ModifierKeys.Control)
			{
				switch (e.Key)
				{
				    case Key.N:
					{
					    NewScript.Execute(null);
					    e.Handled = true;
					    break;
					}
				    case Key.O:
					{
						OpenFile.Execute(null);
					    e.Handled = true;
					    break;
					}
				    case Key.S:
					{
						SaveFile.Execute(null);
					    e.Handled = true;
					    break;
					}
				    case Key.Tab:
					{
						if (m_dc.ActivePane != null)
					    {
							var steps = Keyboard.Modifiers.HasFlag(ModifierKeys.Shift) ? -1 : +1;
							m_dc.ActivePane.CycleVisibleContent(steps);
							e.Handled = true;
					    }
					    break;
					}
				}
			}
			base.OnPreviewKeyDown(e);
		}
		protected override void OnPreviewDrop(DragEventArgs e)
		{
			if (e.Data.GetDataPresent(DataFormats.FileDrop) &&
				e.Data.GetData(DataFormats.FileDrop) is string[] files)
			{
				// If dropped on a specific scene, make that the selected scene for the source
				var scenes = e.Source is SceneUI scene
					? new[] { scene }
					: null;

				// Add each file as a source
				foreach (var file in files)
					AddFileSourceAsync(file, scenes);

				e.Handled = true;
			}

			base.OnPreviewDrop(e);
		}

		private void PreviewMouseDoubleClick_ShowObjectInfo(object sender, MouseButtonEventArgs e)
		{
			// Todo: Show a UI containing information about the object under the mouse pointer
		}

		/// <summary>App logic</summary>
		public Model Model
		{
			get;
			private set
			{
				if (field == value) return;
				if (field != null)
				{
					field.FileOpening -= HandleFileOpened;
					field.ParsingProgressChanged -= HandleParsingProgressChanged;
					field.SourcesChanged -= HandleSourcesChanged;
					field.PropertyChanged -= HandlePropertyChanged;
					Util.Dispose(ref field!);
				}
				field = value;
				if (field != null)
				{
					field.PropertyChanged += HandlePropertyChanged;
					field.SourcesChanged += HandleSourcesChanged;
					field.ParsingProgressChanged += HandleParsingProgressChanged;
					field.FileOpening += HandleFileOpened;
				}

				// Handlers
				void HandlePropertyChanged(object? sender, PropertyChangedEventArgs e)
				{
					switch (e.PropertyName)
					{
						case nameof(Model.Profile):
						{
							Profiles.MoveCurrentTo(Model.Profile);
							LoadProfile();
							break;
						}
						case nameof(SettingsData.Profiles):
						{
							Profiles.Refresh();
							break;
						}
					}
				}
				void HandleFileOpened(object? sender, ValueEventArgs<string> e)
				{
					m_recent_files.Add(e.Value);
					Model.Settings.RecentFiles = m_recent_files.Export();
				}
				void HandleParsingProgressChanged(object? sender, EventArgs args)
				{
					NotifyPropertyChanged(nameof(ParsingProgress));
				}
				void HandleSourcesChanged(object? sender, EventArgs e)
				{
					NotifyPropertyChanged(nameof(StreamingState));
				}
			}
		} = null!;

		/// <summary>The current active content in the dock container</summary>
		public IDockable? ActiveContent
		{
			get;
			set
			{
				if (field == value) return;
				if (field != null)
				{
					if (field is ScriptUI script)
						script.PropertyChanged -= HandleScriptPropertyChanged;
					if (field is SceneUI prev_scene)
						prev_scene.PropertyChanged -= HandleScenePropertyChanged;
				}
				field = value;
				if (field != null)
				{
					if (field is ScriptUI script)
						script.PropertyChanged -= HandleScriptPropertyChanged;
					if (field is SceneUI new_scene)
						new_scene.PropertyChanged += HandleScenePropertyChanged;
				}

				NotifyPropertyChanged(nameof(ActiveScene));
				NotifyPropertyChanged(nameof(ActiveScript));
				NotifyPropertyChanged(nameof(ScriptHasFocus));
				NotifyPropertyChanged(nameof(ScriptHasFocusAndNeedsSave));
				NotifyPropertyChanged(nameof(ActiveContent));
				NotifyPropertyChanged(nameof(HoveredObjectInfo));
				NotifyPropertyChanged(nameof(ObjectInfoEnabled));

				void HandleScriptPropertyChanged(object? sender, PropertyChangedEventArgs e)
				{
					if (sender is not ScriptUI script) return;
					if (e.PropertyName == nameof(ScriptUI.SaveNeeded))
						NotifyPropertyChanged(nameof(ScriptHasFocusAndNeedsSave));
				}
				void HandleScenePropertyChanged(object? sender, PropertyChangedEventArgs e)
				{
					if (e.PropertyName == nameof(SceneUI.HoveredObjectInfo))
						NotifyPropertyChanged(nameof(HoveredObjectInfo));
					else if (e.PropertyName == nameof(SceneUI.ObjectInfoEnabled))
						NotifyPropertyChanged(nameof(ObjectInfoEnabled));
				}
			}
		}

		/// <summary>The active content cast to specific types for binding</summary>
		public SceneUI? ActiveScene => ActiveContent as SceneUI;
		public ScriptUI? ActiveScript => ActiveContent as ScriptUI;

		/// <summary>True if the active content in the dock container is a script ui</summary>
		public bool ScriptHasFocus => m_dc.ActiveDockable is ScriptUI;

		/// <summary>True if the active content in the dock container is a script ui and its content needs saving</summary>
		public bool ScriptHasFocusAndNeedsSave => m_dc.ActiveDockable is ScriptUI script && script.SaveNeeded;

		/// <summary>Status bar text</summary>
		public string StatusMessage
		{
			get => field ?? string.Empty;
			set
			{
				if (field == value) return;
				field = value;
				NotifyPropertyChanged(nameof(StatusMessage));
			}
		}

		/// <summary>The text description of the streaming state</summary>
		public View3d.EStreamingState StreamingState => Model.View3d.StreamingState;

		/// <summary>Non-null while script parsing is in progress</summary>
		public ParsingProgressData? ParsingProgress => Model.ParsingProgress;

		/// <summary>A text description of the object under the mouse pointer</summary>
		public string HoveredObjectInfo => ActiveScene?.HoveredObjectInfo ?? string.Empty;

		/// <summary>True if object info display is enabled on the active scene</summary>
		public bool ObjectInfoEnabled => ActiveScene?.ObjectInfoEnabled ?? false;

		/// <summary>The available profiles</summary>
		public ICollectionView Profiles
		{
			get => field;
			set
			{
				if (field == value) return;
				if (field != null)
				{
					field.CurrentChanged -= HandleCurrentChanged;
				}
				field = value;
				if (field != null)
				{
					field.CurrentChanged += HandleCurrentChanged;
				}

				// Handlers
				void HandleCurrentChanged(object? sender, EventArgs e)
				{
					if (Profiles.CurrentItem is SettingsProfile profile)
					{
						Model.Profile = profile;
					}
				}
			}
		} = null!;

		/// <summary>Set the initial state of the dock container</summary>
		private void InitDockContainer()
		{
			// Set options
			m_dc.Options.AlwaysShowTabs = true;

			// Add a log window
			var log = new LogUI(Model);
			m_dc.Add(log, EDockSite.Right).IsAutoHide = true;

			// Add the sources list window
			var sources = new SourcesListUI(Model);
			m_dc.Add(sources, EDockSite.Left).IsAutoHide = true;

			// Update our reference to the active content whenever it changes in the dock container
			m_dc.ActiveContentChanged += delegate { ActiveContent = m_dc.ActiveDockable; };
			m_dc.LayoutChanged += SaveLayout;

			// Add the menu for dock container windows
			m_menu.Items.Insert(1, m_dc.WindowsMenu());
		}

		/// <summary>Update the UI layout to match the current profile</summary>
		private void LoadProfile()
		{
			// Add all scenes to the dock container
			foreach (var scene in Model.Scenes)
				m_dc.Add(scene, EDockSite.Centre);

			// Restore the layout
			m_dc.LoadLayout(Model.Profile.UILayout);

			// Make the first scene the active content
			if (Model.Scenes.Count != 0)
				Model.Scenes[0].DockControl.IsActiveContent = true;
		}

		/// <summary>Persist the current layout to settings</summary>
		private void SaveLayout(object? sender = null, EventArgs? args = null)
		{
			try
			{
				Model.Profile.UILayout = m_dc.SaveLayout();
			}
			catch (Exception ex)
			{
				Log.Write(ELogLevel.Error, ex, "Saving the UI layout failed.");
				MsgBox.Show(this, $"Saving the UI layout failed.\n{ex.Message}", Util.AppProductName, MsgBox.EButtons.OK, MsgBox.EIcon.Information);
			}
		}

		/// <summary>Open 'source' in a script editor</summary>
		public void OpenInEditor(Source source)
		{
			try
			{
				// Open 'source' in an editor window
				var ui = Model.OpenInEditor(source) ?? throw new Exception("Failed to open the editor for the new script file");

				// Display the editor
				m_dc.Add(ui, EDockSite.Left).IsFloating = true;
			}
			catch (Exception ex)
			{
				Log.Write(ELogLevel.Info, ex, "Opening source in editor failed.");
				MsgBox.Show(this, $"Opening source in editor failed.\n{ex.Message}", Util.AppProductName, MsgBox.EButtons.OK, MsgBox.EIcon.Information);
			}
		}

		/// <summary>Add a file source (synchronous — for NewScript which needs immediate Source ref)</summary>
		public Source? AddFileSource(string? filepath, IList<SceneUI>? scenes = null)
		{
			try
			{
				// Prompt for a filepath if none provided
				if (filepath == null || filepath.Length == 0)
				{
					var dlg = new OpenFileDialog { Title = "Open an Ldraw supported file", Filter = Model.SupportedFilesFilter };
					if (dlg.ShowDialog(App.Current.MainWindow) != true)
						return null;
					
					filepath = dlg.FileName ?? throw new FileNotFoundException($"A invalid filepath was selected");
				}

				// If the file doesn't exist reject it
				if (!Path_.FileExists(filepath))
					throw new FileNotFoundException($"File '{filepath}' does not exist");

				// Add the file as a new source
				return Model.AddFileSource(filepath, scenes ?? Model.Scenes.Take(1));
			}
			catch (Exception ex)
			{
				Log.Write(ELogLevel.Info, ex, "Add file source failed.", filepath ?? string.Empty, 0, 0);
				MsgBox.Show(Owner, $"Add file source failed.\n{ex.Message}", Util.AppProductName, MsgBox.EButtons.OK, MsgBox.EIcon.Information);
				return null;
			}
		}

		/// <summary>Add a file source asynchronously (non-blocking)</summary>
		public void AddFileSourceAsync(string? filepath, IList<SceneUI>? scenes = null)
		{
			try
			{
				// Prompt for a filepath if none provided
				if (filepath == null || filepath.Length == 0)
				{
					var dlg = new OpenFileDialog { Title = "Open an Ldraw supported file", Filter = Model.SupportedFilesFilter };
					if (dlg.ShowDialog(App.Current.MainWindow) != true)
						return;
					
					filepath = dlg.FileName ?? throw new FileNotFoundException($"A invalid filepath was selected");
				}

				// If the file doesn't exist reject it
				if (!Path_.FileExists(filepath))
					throw new FileNotFoundException($"File '{filepath}' does not exist");

				// Add the file as a new source asynchronously
				Model.AddFileSourceAsync(filepath, scenes ?? Model.Scenes.Take(1));
			}
			catch (Exception ex)
			{
				Log.Write(ELogLevel.Info, ex, "Add file source failed.", filepath ?? string.Empty, 0, 0);
				MsgBox.Show(Owner, $"Add file source failed.\n{ex.Message}", Util.AppProductName, MsgBox.EButtons.OK, MsgBox.EIcon.Information);
			}
		}

		/// <summary>Add a new Script to the view</summary>
		public Command NewScript { get; }
		private void NewScriptInternal()
		{
			try
			{
				// Create a new script file
				var filepath = Model.CreateNewScriptFile();
				var src = AddFileSource(filepath) ?? throw new Exception("Failed to an empty script file");

				// Open it in an editor
				var ui = Model.OpenInEditor(src) ?? throw new Exception("Failed to open the editor for the new script file");

				// Display the editor
				m_dc.Add(ui, EDockSite.Left).IsFloating = true;
			}
			catch (Exception ex)
			{
				Log.Write(ELogLevel.Info, ex, "Create new script failed.");
				MsgBox.Show(this, $"Create new script failed.\n{ex.Message}", Util.AppProductName, MsgBox.EButtons.OK, MsgBox.EIcon.Information);
			}
		}

		/// <summary>Add a new Scene to the view</summary>
		public Command NewScene { get; }
		private void NewSceneInternal()
		{
			try
			{
				var scene = Model.Scenes.Add2(new SceneUI(Model, Model.GenerateSceneName()));
				m_dc.Add(scene, EDockSite.Centre);
			}
			catch (Exception ex)
			{
				Log.Write(ELogLevel.Info, ex, "Create new scene failed.");
				MsgBox.Show(this, $"Create new scene failed.\n{ex.Message}", Util.AppProductName, MsgBox.EButtons.OK, MsgBox.EIcon.Information);
			}
		}

		/// <summary>Open a file</summary>
		public Command OpenFile { get; }
		private void OpenFileInternal()
		{
			AddFileSourceAsync(null);
		}

		/// <summary>Save the currently focused script</summary>
		public Command SaveFile { get; }
		private void SaveFileInternal()
		{
			if (m_dc.ActiveDockable is not ScriptUI script)
				return;

			script.SaveScript.Execute();
		}

		/// <summary>Save the currently focused script</summary>
		public Command SaveFileAs { get; }
		private void SaveFileAsInternal()
		{
			if (m_dc.ActiveDockable is not ScriptUI script)
				return;

			script.SaveScript.Execute();
			throw new NotImplementedException("Save as not supported");
		}

		/// <summary>Enable/Disable listening for streaming connections</summary>
		public Command ToggleStreaming { get; }
		private void ToggleStreamingInternal()
		{
			var currently_active = StreamingState != View3d.EStreamingState.Disconnected;
			Model.View3d.Streaming(!currently_active, Model.Profile.StreamingPort);
			NotifyPropertyChanged(nameof(StreamingState));
		}

		/// <summary>Show application preferences</summary>
		public Command ShowPreferences { get; }
		private void ShowPreferencesInternal()
		{
			if (m_settings_ui == null)
			{
				m_settings_ui = new SettingsUI(this, Model);
				m_settings_ui.Closed += delegate { m_settings_ui = null; };
				m_settings_ui.Show();
			}
			m_settings_ui.Focus();
		}
		private SettingsUI? m_settings_ui;

		/// <summary>Show a dialog containing the example script</summary>
		public Command ShowExampleScript { get; }
		private void ShowExampleScriptInternal()
		{
			try
			{
				// Create a new script file
				var filepath = Model.CreateNewScriptFile("ExampleScript");
				File.WriteAllText(filepath, View3d.ExampleScript);
				var src = AddFileSource(filepath) ?? throw new Exception("Failed to an empty script file");

				// Open it in an editor
				var ui = Model.OpenInEditor(src) ?? throw new Exception("Failed to open the editor for the new script file");

				// Display the editor
				m_dc.Add(ui, EDockSite.Left).IsFloating = true;
			}
			catch (Exception ex)
			{
				Log.Write(ELogLevel.Info, ex, "Create example script failed.");
				MsgBox.Show(this, $"Create example script failed.\n{ex.Message}", Util.AppProductName, MsgBox.EButtons.OK, MsgBox.EIcon.Information);
			}
		}

		/// <summary>Show the about box</summary>
		public Command ShowAbout { get; }
		private void ShowAboutInternal()
		{
			new AboutUI(this).ShowDialog();
		}

		/// <summary>Shutdown the application</summary>
		public Command Exit { get; }
		private void ExitInternal()
		{
			Close();
		}

		/// <inheritdoc/>
		public event PropertyChangedEventHandler? PropertyChanged;
		private void NotifyPropertyChanged(string prop_name) => PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(prop_name));
	}
}
