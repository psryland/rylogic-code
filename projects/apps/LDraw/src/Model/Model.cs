using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.ComponentModel;
using System.IO;
using System.Linq;
using System.Text.RegularExpressions;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Controls;
using System.Windows.Threading;
using LDraw.MCP;
using LDraw.UI;
using Rylogic.Common;
using Rylogic.Extn;
using Rylogic.Gfx;
using Rylogic.Gui.WPF;
using Rylogic.Utility;

namespace LDraw
{
	public sealed class Model :IDisposable, INotifyPropertyChanged
	{
		public Model(StartupOptions options, SettingsData settings)
		{
			StartupOptions = options;
			Sync = SynchronizationContext.Current ?? throw new Exception("No synchronisation context available");
			FileWatchTimer = new DispatcherTimer(DispatcherPriority.ApplicationIdle);
			View3d = View3d.Create();
			Sources = [];
			Scenes = [];
			Scripts = [];
			Settings = settings;
			Mcp = new McpService(this);

			// Ensure the temporary script directory exists
			Path_.CreateDirs(TempScriptDirectory);
		}
		public void Dispose()
		{
			Util.Dispose(ref m_mcp!);
			FileWatchTimer = null!;
			View3d = null!;
			GC.SuppressFinalize(this);
		}

		/// <summary>Parsed command line options</summary>
		public StartupOptions StartupOptions { get; }

		/// <summary>The main thread synchronisation context</summary>
		private SynchronizationContext Sync { get; }

		/// <summary>Run 'action' on the main thread and return the result</summary>
		public async Task<T> InvokeAsync<T>(Func<T> action, TimeSpan? timeout = null)
		{
			if (SynchronizationContext.Current == Sync)
				return action();

			var tcs = new TaskCompletionSource<T>(TaskCreationOptions.RunContinuationsAsynchronously);
			using var cts = timeout.HasValue ? new CancellationTokenSource(timeout.Value) : null;
			var token = cts?.Token ?? CancellationToken.None;
			using var reg = token.Register(() => tcs.TrySetException(new TimeoutException("Timed out waiting for the LDraw UI thread.")));

			Sync.Post(_ =>
			{
				if (token.IsCancellationRequested)
					return;

				try
				{
					tcs.TrySetResult(action());
				}
				catch (Exception ex)
				{
					tcs.TrySetException(ex);
				}
			}, null);

			return await tcs.Task.ConfigureAwait(false);
		}

		/// <summary>The embedded MCP service for exposing LDraw to AI clients</summary>
		public McpService Mcp
		{
			get => m_mcp;
			private set => m_mcp = value;
		}
		private McpService m_mcp = null!;

		/// <summary>The view3d DLL context </summary>
		public View3d View3d
		{
			get;
			set
			{
				if (field == value) return;
				if (field != null)
				{
					field.ParsingProgress -= HandleParsingProgress;
					field.OnStoreChange -= HandleStoreChanged;
					field.Error -= ReportError;
					Util.Dispose(ref field!);
				}
				field = value;
				if (field != null)
				{
					field.Error += ReportError;
					field.OnStoreChange += HandleStoreChanged;
					field.ParsingProgress += HandleParsingProgress;
				}

				// Handlers
				void ReportError(object? sender, View3d.ErrorEventArgs e)
				{
					Log.Write(ELogLevel.Error, e.Message, e.Filepath, e.FileLine, e.FileOffset);
				}
				void HandleStoreChanged(object? sender, View3d.StoreChangedEventArgs e)
				{
					// Refresh the collection of sources if sources were added or removed.
					// 'SourcesChanged' can mean a source was added/removed, or it can mean
					// a source changed its data (i.e. Load was called).
					if (e.After && (
						e.ChangeFlags.HasFlag(View3d.EStoreChangeFlags.ContextIdAdded) ||
						e.ChangeFlags.HasFlag(View3d.EStoreChangeFlags.ContextIdRemoved)))
					{
						// Assume all sources will be removed to start with
						var old = Sources.ToDictionary(x => x.ContextId, x => x);

						// Get the native code to tell us what sources exist
						Dictionary<Guid, Source> nue = [];
						field.EnumSources(src =>
						{
							// Ignore tool-owned contexts that should not appear as user sources.
							if (View3d.ObjectManager.ExcludeCtxIds.Contains(src.ContextId))
								return;

							// Remove 'src.ContextId' from the old list if it's still in use
							if (old.ContainsKey(src.ContextId))
							{
								old.Remove(src.ContextId);
								return;
							}

							// Add the new source
							nue.Add(src.ContextId, new Source(this, src));
						});

						// Update the 'Sources' list
						Sources.RemoveAll(x => old.ContainsKey(x.ContextId));

						// Automatically add new sources to the first scene
						var reset_view = Profile.ResetOnLoad && e.ChangeFlags.HasFlag(View3d.EStoreChangeFlags.ObjectsAdded);
						foreach (var s in nue.Values)
						{
							if (!s.SelectedScenes.Any())
								s.ShowInScenes(s.AvailableScenes.Take(1), true, reset_view);
						}

						// Add the new scene to the sources list
						Sources.AddRange(nue.Values);

						// Notify of sources changed/deleted
						SourcesChanged?.Invoke(this, EventArgs.Empty);

						// Disposing any old sources (after notifying so they remain valid in event handlers)
						Util.DisposeRange(old.Values);
					}

					// Clean up progress tracking for sources that have finished loading/reloading
					if (e.After && e.ChangeFlags.HasFlag(View3d.EStoreChangeFlags.ObjectsAdded))
					{
						foreach (var ctx_id in e.ContextIds)
						{
							m_progress_map.Remove(ctx_id);
							var src = Sources.FirstOrDefault(s => s.ContextId == ctx_id);
							if (src != null)
							{
								src.IsLoading = false;
								src.LoadFraction = 0;
							}
						}
						ParsingProgress = m_progress_map.Count > 0
							? m_progress_map.Values.MinBy(p => p.Percentage)
							: null;
					}

					// Reloads update objects already present in scenes. Reset the view here, where the store-change initiator says this was a reload,
					// rather than in AddObjects where callers may only be changing scene membership.
					if (e.After && Profile.ResetOnLoad && e.ChangeFlags.HasFlag(View3d.EStoreChangeFlags.ObjectsAdded))
					{
						if (e.Initiator == View3d.EStoreChangeInitiator.Reload)
							AutoRangeScenesContaining(e.ContextIds);
					}

					// Just prior to reloading sources
					if (e.Before && Profile.ClearErrorLogOnReload)
						Log.Clear();

					// This implements auto range on load... but sources can change for reasons that don't require
					// an auto range (e.g. measure tool graphics).
					//	// After a source change, reset
					//	if (e.After && Settings.ResetOnLoad)
					//		foreach (var scene in Scenes)
					//			scene.SceneView.AutoRange();
				}
				void HandleParsingProgress(object? sender, View3d.ParsingProgressEventArgs e) // worker thread context
				{
					// Marshal to the main thread and update progress
					Sync.Post(x =>
					{
						var args = (View3d.ParsingProgressEventArgs)x!;

						if (args.Complete)
						{
							m_progress_map.Remove(args.ContextId);

							// Reset loading state on the source
							var src = Sources.FirstOrDefault(s => s.ContextId == args.ContextId);
							if (src != null)
							{
								src.IsLoading = false;
								src.LoadFraction = 0;
							}
						}
						else
						{
							// Find the progress record and update it with the latest data
							if (!m_progress_map.TryGetValue(args.ContextId, out var progress))
							{
								progress = new ParsingProgressData(args.ContextId, () => View3d.CancelLoad(args.ContextId));
								m_progress_map[args.ContextId] = progress;
							}

							progress.DataSourceName = args.Filepath;
							progress.DataLength = args.FileSize;
							progress.DataOffset = args.FileOffset;

							// Update loading state on the source
							var src = Sources.FirstOrDefault(s => s.ContextId == args.ContextId);
							if (src != null)
							{
								src.IsLoading = true;
								src.LoadFraction = args.FileSize > 0 ? (double)args.FileOffset / args.FileSize : 0;
							}
						}

						// Update the status bar with the lowest-progress entry
						ParsingProgress = m_progress_map.Count > 0
							? m_progress_map.Values.MinBy(p => p.Percentage)
							: null;
					}, e);
				}
			}
		} = null!;

		/// <summary>Application settings</summary>
		public SettingsData Settings
		{
			get;
			set
			{
				if (field == value) return;
				if (field != null)
				{
					field.SettingChange -= HandleSettingChange;
				}
				field = value;
				if (field != null)
				{
					field.SettingChange += HandleSettingChange;

					// Ensure there is a default profile
					if (field.Profiles.Count == 0)
						field.Profiles.Add(new SettingsProfile { Name = SettingsProfile.DefaultProfileName });

					// Start with the first profile at startup
					Profile = field.Profiles[0];
				}

				// Handlers
				void HandleSettingChange(object? sender, SettingChangeEventArgs e)
				{
					if (e.Before) return;
					switch (e.Key)
					{
						case nameof(SettingsData.Profiles):
						{
							NotifyPropertyChanged(nameof(Settings.Profiles));
							break;
						}
						case nameof(SettingsProfile.CheckForChangesPollPeriodS) when e.SettingSet == Profile:
						{
							FileWatchTimer.Interval = TimeSpan.FromSeconds(Profile.CheckForChangesPollPeriodS);
							break;
						}
					}
				}
			}
		} = null!;

		/// <summary>The active profile</summary>
		public SettingsProfile Profile
		{
			get;
			set
			{
				if (Profile == value) return;
				if (field != null)
				{
					Settings.Save();
				}
				field = value;
				if (field != null)
				{
					// When the profile changes, delete and recreate the scenes
					Util.DisposeRange(Scenes);
					Scenes.Clear();

					// Create scenes
					foreach (var ss in field.SceneState)
					{
						Scenes.Add(new SceneUI(this, ss.Name));
					}
					if (Scenes.Count == 0)
					{
						Scenes.Add(new SceneUI(this, GenerateSceneName()));
					}
				}
				NotifyPropertyChanged(nameof(Profile));
			}
		} = null!;

		/// <summary>The collection of current Ldraw object sources</summary>
		public List<Source> Sources { get; }
		public event EventHandler? SourcesChanged;

		/// <summary>The scene instances</summary>
		public ObservableCollection<SceneUI> Scenes { get; }

		/// <summary>The script instances</summary>
		public ObservableCollection<ScriptUI> Scripts { get; }

		/// <summary>Notify of a file about to be opened</summary>
		public event EventHandler<ValueEventArgs<string>>? FileOpening;
		public void NotifyFileOpening(string filepath) => FileOpening?.Invoke(this, new ValueEventArgs<string>(filepath));

		/// <summary>Progress updates for parsing. Returns the lowest-progress entry, or null while not parsing</summary>
		public ParsingProgressData? ParsingProgress
		{
			get;
			private set
			{
				if (field == value) return;
				field = value;
				NotifyPropertyChanged(nameof(ParsingProgress));
				ParsingProgressChanged?.Invoke(this, EventArgs.Empty);
			}
		}
		private readonly Dictionary<Guid, ParsingProgressData> m_progress_map = [];

		/// <summary>Raised when a progress update is received</summary>
		public event EventHandler? ParsingProgressChanged;

		/// <summary>Timer used to watch for file changes</summary>
		public DispatcherTimer FileWatchTimer
		{
			get => m_file_watch_timer;
			set
			{
				if (m_file_watch_timer == value) return;
				if (m_file_watch_timer != null)
				{
					m_file_watch_timer.Stop();
					m_file_watch_timer.Tick -= HandleCheckForChangedFiles;
				}
				m_file_watch_timer = value;
				if (m_file_watch_timer != null)
				{
					m_file_watch_timer.Tick += HandleCheckForChangedFiles;
					m_file_watch_timer.Start();
				}

				// Handlers
				void HandleCheckForChangedFiles(object? sender, EventArgs e)
				{
					try
					{
						if (Profile.AutoRefresh)
							View3d.CheckForChangedSources();
					}
					catch (Exception ex)
					{
						Log.Write(ELogLevel.Error, ex, "Error during CheckForChangedFiles");
					}
				}
			}
		}
		private DispatcherTimer m_file_watch_timer = null!;

		/// <summary>Add a file ldraw source (synchronous, blocks the calling thread)</summary>
		public Source AddFileSource(string filepath, IEnumerable<SceneUI> scenes)
		{
			NotifyFileOpening(filepath);
			var ctx_id = View3d.LoadScriptFromFile(filepath).ContextId;
			var src = Sources.First(x => x.ContextId == ctx_id);
			src.ShowInScenes(scenes, true, reset_view: true);
			return src;
		}

		/// <summary>Add a file ldraw source asynchronously (non-blocking)</summary>
		public void AddFileSourceAsync(string filepath, IEnumerable<SceneUI> scenes)
		{
			NotifyFileOpening(filepath);

			// Capture the scenes list before entering the background thread
			var target_scenes = scenes.ToArray();

			// Pre-compute the context id so it's available immediately
			var ctx_id = View3d.ContextIdFromFilepath(filepath);

			Task.Run(() =>
			{
				// This blocks the worker thread during parsing, but not the GUI thread.
				// Progress events are fired from this thread and marshaled to the UI thread.
				View3d.LoadScriptFromFile(filepath, context_id: ctx_id, add_completed: (id, before) =>
				{
					if (before)
						return;

					// Queue behind the store-change event that updates 'Sources'.
					Sync.Post(_ =>
					{
						var src = Sources.FirstOrDefault(x => x.ContextId == id);
						src?.ShowInScenes(target_scenes, true, reset_view: true);
					}, null);
				});
			});
		}

		/// <summary>Cancel the current loading operation (the one with lowest progress)</summary>
		public void CancelCurrentLoad()
		{
			var progress = ParsingProgress;
			if (progress != null)
				View3d.CancelLoad(progress.ContextId);
		}

		/// <summary>Add a string ldraw source</summary>
		public Source AddStringSource(string text, IEnumerable<SceneUI> scenes)
		{
			var ctx_id = View3d.LoadScriptFromString(text).ContextId;
			var src = Sources.First(x => x.ContextId == ctx_id);
			src.ShowInScenes(scenes, true, reset_view: true);
			return src;
		}

		/// <summary>Return a generated name for a new scene UI</summary>
		public string GenerateSceneName()
		{
			for (; ; )
			{
				var name = $"{UITag.Scene}-{++m_scene_number}";
				if (!Scenes.Any(x => string.Compare(x.SceneName, name, true) == 0))
					return name;
			}
		}
		private int m_scene_number;

		/// <summary>Return a generated name for a new script UI</summary>
		public string GenerateScriptName()
		{
			for (; ; )
			{
				var name = $"{UITag.Script}-{++m_script_number}";
				if (!Scripts.Any(x => string.Compare(x.ScriptName, name, true) == 0))
					return name;
			}
		}
		private int m_script_number;

		/// <summary>Create an empty script and return its filepath</summary>
		public string CreateNewScriptFile(string? script_name = null)
		{
			script_name ??= GenerateScriptName();
			var filepath = Path_.CombinePath(TempScriptDirectory, $"{script_name}.ldr");
			File.AppendText(filepath).Dispose();
			return filepath;
		}

		/// <summary>Open a ldr script text file in a script window</summary>
		public ScriptUI? OpenInEditor(Source src)
		{
			if (src.FilePath.Length == 0)
				return null;

			// See if there is already a script with this source
			foreach (var script in Scripts)
			{
				if (script.Source.ContextId == src.ContextId)
					return script;
			}

			// Otherwise, create a new one
			return Scripts.Add2(new ScriptUI(src));
		}

		/// <summary>Clear all instances from all scenes</summary>
		public void Clear()
		{
			foreach (var scene in Scenes)
			{
				var view = scene.SceneView;
				view.Scene.RemoveAllObjects();
				view.Scene.Invalidate();
			}
		}

		/// <summary>Clear all instances from one or more scenes</summary>
		public void Clear(IEnumerable<SceneUI> scenes)
		{
			Clear(scenes, x => true);
		}

		/// <summary>Clear instances from one or more scenes</summary>
		public void Clear(SceneUI scene) => Clear(new[] { scene });
		public void Clear(SceneUI scene, Guid context_id) => Clear(new[] { scene }, x => x == context_id);
		public void Clear(IEnumerable<SceneUI> scenes, Guid context_id) => Clear(scenes, x => x == context_id);
		public void Clear(IEnumerable<SceneUI> scenes, Func<Guid,bool> context_pred)
		{
			// Remove objects from the scenes
			foreach (var scene in scenes)
			{
				var view = scene.SceneView;
				view.Scene.RemoveObjects(context_pred);
				view.Scene.Invalidate();
			}
		}

		/// <summary>Add objects associated with 'context_id' to 'scene'.</summary>
		public void AddObjects(SceneUI scene, Guid context_id, bool reset_view = false)
		{
			AddObjects(new[] { scene }, context_id, reset_view);
		}

		/// <summary>Add objects associated with 'context_id' to 'scenes'.</summary>
		public void AddObjects(IEnumerable<SceneUI> scenes, Guid context_id, bool reset_view = false)
		{
			AddObjects(scenes, x => x == context_id, reset_view);
		}

		/// <summary>Add objects matching 'context_pred' to 'scenes'.</summary>
		public void AddObjects(IEnumerable<SceneUI> scenes, Func<Guid, bool> context_pred, bool reset_view = false)
		{
			// Add the matching source objects to each scene. Auto-range is reserved for real load/reload operations, not ordinary scene membership changes.
			foreach (var scene in scenes)
			{
				var view = scene.SceneView;
				view.Scene.AddObjects(context_pred);

				// Auto range the view
				if (reset_view && Profile.ResetOnLoad)
					view.AutoRange();
				else
					view.Invalidate();
			}
		}

		/// <summary>Auto-range scenes that contain any of the reloaded source contexts.</summary>
		private void AutoRangeScenesContaining(Guid[] context_ids)
		{
			var context_set = new HashSet<Guid>(context_ids);
			foreach (var scene in Scenes)
			{
				var contains_context = false;
				scene.SceneView.Scene.Window.EnumGuids(context_id =>
				{
					contains_context = context_set.Contains(context_id);
					return !contains_context;
				});
				if (contains_context)
					scene.SceneView.AutoRange();
			}
		}

		/// <summary>The file paths of existing temporary scripts</summary>
		public IEnumerable<FileSystemInfo> TemporaryScripts()
		{
			// Treat anything in the temporary script directory as a temporary script
			foreach (var file in Path_.EnumFileSystem(TempScriptDirectory, SearchOption.TopDirectoryOnly, exclude: FileAttributes.Directory))
			{
				// Filter out files that aren't ldr script
				var temp_script_pattern = $@"^{UITag.Script}[-].*\.ldr$";
				if (!Regex.IsMatch(file.FullName, temp_script_pattern))
					continue;

				yield return file;
			}
		}

		/// <summary>Return a collection of scenes to add objects to</summary>
		public IList<SceneUI> ChooseScenes(string prompt_text)
		{
			if (Scenes.Count == 0)
				throw new Exception($"No 3D scenes available");

			// If there's only one option, no need to prompt
			if (Scenes.Count == 1)
				return new[] { Scenes[0] };

			// Allow the objects to be added to the selected scenes
			var dlg = new ListUI(App.Current.MainWindow)
			{
				Title = "Select Scenes",
				Prompt = prompt_text,
				SelectionMode = SelectionMode.Extended,
				DisplayMember = nameof(SceneUI.SceneName),
				AllowCancel = true,
			};
			dlg.Items.AddRange(Scenes);

			// Prompt for the scenes to use
			if (dlg.ShowDialog() != true || dlg.SelectedItems.Count == 0)
				return Array.Empty<SceneUI>();

			return dlg.SelectedItems.Cast<SceneUI>().ToArray();
		}

		/// <inheritdoc/>
		public event PropertyChangedEventHandler? PropertyChanged;
		private void NotifyPropertyChanged(string prop_name) => PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(prop_name));

		/// <summary>The directory to contain temporary scripts in</summary>
		private string TempScriptDirectory => Path_.CombinePath(StartupOptions.UserDataDir, "Temporary Scripts");

		/// <summary></summary>
		public static readonly string SupportedFilesFilter = Util.FileDialogFilter(
			"Supported Files", "*.ldr", "*.bdr", "*.p3d", "*.3ds", "*.stl", "*.fbx", "*.gltf", "*.glb", "*.csv",
			"Ldr Script", "*.ldr", "*.bdr",
			"Binary Model File", "*.p3d",
			"3D Studio Max Model File", "*.3ds",
			"STL CAD Model File", "*.stl",
			"Filmbox Model File", "*.fbx",
			"glTF Model File", "*.gltf", "*.glb",
			"Comma Separated Values", "*.csv",
			"All Files", "*.*");

		/// <summary>Text file types that can be edited in the script UI</summary>
		public static readonly string EditableFilesFilter = Util.FileDialogFilter(
			"Script Files", "*.ldr",
			"Text Files", "*.txt",
			"Comma Separated Values", "*.csv",
			"All Files", "*.*");

		/// <summary>Text file types that can be edited in the script UI</summary>
		public static readonly string AssetFilesFilter = Util.FileDialogFilter(
			"Binary Model File", "*.p3d",
			"3D Studio Max Model File", "*.3ds",
			"STL CAD Model File", "*.stl",
			"Filmbox Model File", "*.fbx",
			"glTF Model File", "*.gltf", "*.glb",
			"Comma Separated Values", "*.csv",
			"All Files", "*.*");

		/// <summary></summary>
		private static class UITag
		{
			public const string Scene = "Scene";
			public const string Script = "Script";
		}
	}
}
