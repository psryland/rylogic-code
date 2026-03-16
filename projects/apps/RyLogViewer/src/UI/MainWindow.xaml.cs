using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.IO;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Input;
using Microsoft.Win32;
using Rylogic.Common;
using Rylogic.Gui.WPF;
using RyLogViewer.Options;

namespace RyLogViewer
{
	public partial class MainWindow : Window, INotifyPropertyChanged
	{
		private readonly LogGrid m_grid;
		private readonly HighlightsPanel m_highlights_panel;
		private readonly FiltersPanel m_filters_panel;
		private readonly FindPanel m_find_panel;
		private readonly BookmarksPanel m_bookmarks_panel;
		private readonly TransformsPanel m_transforms_panel;
		private readonly ActionsPanel m_actions_panel;

		public MainWindow(Main main, Settings settings, IReport report)
		{
			InitializeComponent();

			Main = main;
			Settings = settings;
			Report = report;

			// Create the log grid and add it to the dock container as centre content
			m_grid = new LogGrid
			{
				Style = (Style)FindResource("LogDataGridStyle"),
			};
			m_grid.Settings = settings;
			m_grid.Highlights = main.Highlights;
			m_grid.SetBinding(DataGrid.ItemsSourceProperty, new System.Windows.Data.Binding(nameof(LogData)) { Source = this });

			// Add the grid to the dock container as centre content
			m_dock.Add(m_grid, EDockSite.Centre);

			// Create dockable tool panels
			m_highlights_panel = new HighlightsPanel(Main);
			m_filters_panel = new FiltersPanel(Main);
			m_find_panel = new FindPanel(Main);
			m_bookmarks_panel = new BookmarksPanel(Main);
			m_transforms_panel = new TransformsPanel(Main);
			m_actions_panel = new ActionsPanel(Main);
			m_dock.Add(m_highlights_panel, EDockSite.Right);
			m_dock.Add(m_filters_panel, EDockSite.Right);
			m_dock.Add(m_transforms_panel, EDockSite.Right);
			m_dock.Add(m_actions_panel, EDockSite.Right);
			m_dock.Add(m_find_panel, EDockSite.Bottom);
			m_dock.Add(m_bookmarks_panel, EDockSite.Bottom);

			// Wire up navigation events from find/bookmarks
			m_find_panel.FoundMatch += (s, e) => ScrollToLine(e.LineIndex);
			m_bookmarks_panel.NavigateToBookmark += (s, e) => ScrollToLine(e.LineIndex);

			// Initialise from settings
			m_recent_files.Import(Settings.RecentFiles);
			m_recent_files.RecentFileSelected += fp =>
			{
				m_recent_files.Add(fp);
				try
				{
					Main.LogDataSource = new SingleFileSource(fp, Settings);
				}
				catch
				{
					Main.LogDataSource = null;
				}
			};

			// Commands
			OpenSingleLogFile = Command.Create(this, OpenSingleLogFileInternal);
			OpenProgramOutput = Command.Create(this, OpenProgramOutputInternal);
			ExportLog = Command.Create(this, ExportLogInternal);
			ShowOptionsUI = Command.Create(this, ShowOptionsUIInternal);
			ToggleHighlights = Command.Create(this, ToggleHighlightsInternal);
			ToggleFilters = Command.Create(this, ToggleFiltersInternal);
			ToggleFind = Command.Create(this, ToggleFindInternal);
			ToggleBookmarks = Command.Create(this, ToggleBookmarksInternal);
			ToggleTransforms = Command.Create(this, ToggleTransformsInternal);
			ToggleActions = Command.Create(this, ToggleActionsInternal);
			ShowAbout = Command.Create(this, ShowAboutInternal);
			Shutdown = Command.Create(this, ShutdownInternal);

			Loaded += (s, a) =>
			{
				// Assign data contexts after properties have been set
				DataContext = this;
				m_panel.DataContext = this;

				// Track selection changes
				m_grid.SelectionChanged += HandleGridSelectionChanged;

				// Wire bookmark toggle from grid context menu
				m_grid.ToggleBookmarkRequested += (gs, ge) =>
				{
					var idx = m_grid.SelectedIndex;
					if (idx >= 0 && idx < Main.Count)
					{
						var line = Main[idx];
						Main.Bookmarks.Toggle(idx, line.FileByteRange.Beg, line.Value(0));
					}
				};

				// Wire click actions — double-click triggers matching actions
				m_grid.MouseDoubleClick += (gs, ge) =>
				{
					var idx = m_grid.SelectedIndex;
					if (idx < 0 || idx >= Main.Count) return;
					var text = Main[idx].Value(0);
					foreach (var action in Main.Actions)
						action.Execute(text);
				};
			};

			// Keyboard shortcuts
			InputBindings.Add(new KeyBinding(OpenSingleLogFile, Key.O, ModifierKeys.Control));
			InputBindings.Add(new KeyBinding(ToggleFind, Key.F, ModifierKeys.Control));
			InputBindings.Add(new KeyBinding(Shutdown, Key.Q, ModifierKeys.Control));
		}

		/// <summary>Notify property changes</summary>
		public event PropertyChangedEventHandler? PropertyChanged;

		/// <summary>Application logic</summary>
		private Main Main
		{
			get;
			set
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

				// Handlers
				void HandlePropertyChanged(object? sender, PropertyChangedEventArgs e)
				{
					PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(nameof(WindowTitle)));
					PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(nameof(StatusLogDataSource)));
				}
			}
		} = null!;

		/// <summary>Settings</summary>
		private Settings Settings { get; }

		/// <summary></summary>
		private IReport Report { get; }

		/// <summary>Open a single log file</summary>
		public ICommand OpenSingleLogFile { get; }
		private void OpenSingleLogFileInternal()
		{
			// Prompt for a log file
			var fd = new OpenFileDialog
			{
				Title = "Open a Log File",
				Filter = Constants.LogFileFilter,
				Multiselect = false,
				CheckFileExists = true,
			};
			if (fd.ShowDialog(this) != true) return;
			var filepath = fd.FileName;

			try
			{
				// Validate
				if (!Path_.FileExists(filepath))
					throw new FileNotFoundException($"File '{filepath}' does not exist");

				// Add the file to the recent files
				AddToRecentFiles(filepath);

				// Create a log data source from the log file
				Main.LogDataSource = new SingleFileSource(filepath, Settings);
			}
			catch (Exception ex)
			{
				Report.ErrorPopup($"Failed to open file {filepath} due to an error.", ex);
				Main.LogDataSource = null;
			}
		}

		/// <summary>Open a program output data source</summary>
		public ICommand OpenProgramOutput { get; }
		private void OpenProgramOutputInternal()
		{
			var dlg = new ProgramOutputUI(Settings.OutputFilepathHistory.Length > 0 ? Array.Empty<LaunchApp>() : Array.Empty<LaunchApp>())
			{
				Owner = this,
			};
			if (dlg.ShowDialog() != true || dlg.Result == null) return;

			try
			{
				Main.LogDataSource = new ProgramOutputSource(dlg.Result);
			}
			catch (Exception ex)
			{
				Report.ErrorPopup($"Failed to launch program '{dlg.Result.Executable}'.", ex);
				Main.LogDataSource = null;
			}
		}

		/// <summary>Export the current log data</summary>
		public ICommand ExportLog { get; }
		private void ExportLogInternal()
		{
			if (Main.Count == 0)
			{
				MessageBox.Show(this, "No log data to export.", "Export", MessageBoxButton.OK, MessageBoxImage.Information);
				return;
			}
			var dlg = new ExportUI(Main, Settings) { Owner = this };
			dlg.ShowDialog();
		}

		/// <summary>Open the Option UI</summary>
		public ICommand ShowOptionsUI { get; }
		private void ShowOptionsUIInternal(object? value)
		{
			if (m_options_ui == null)
			{
				m_options_ui = new OptionsUI(Main, Settings, Report) { Owner = this };
				m_options_ui.SelectedPage = value is EOptionsPage page ? page : EOptionsPage.General;
				m_options_ui.Closed += delegate { m_options_ui = null; };
				m_options_ui.Show();
			}
			m_options_ui.Focus();
		}
		private OptionsUI? m_options_ui;

		/// <summary>Shutdown the application</summary>
		public ICommand Shutdown { get; }
		private void ShutdownInternal()
		{
			Application.Current.Shutdown(0);
		}

		/// <summary>Show the About dialog</summary>
		public ICommand ShowAbout { get; }
		private void ShowAboutInternal()
		{
			var version = System.Reflection.Assembly.GetExecutingAssembly().GetName().Version;
			MessageBox.Show(this,
				$"RyLogViewer\n" +
				$"Version {version}\n\n" +
				$"A log file viewer for large log files.\n" +
				$"© Rylogic Ltd",
				"About RyLogViewer",
				MessageBoxButton.OK,
				MessageBoxImage.Information);
		}

		/// <summary>Toggle the highlights panel visibility</summary>
		public ICommand ToggleHighlights { get; }
		private void ToggleHighlightsInternal()
		{
			var dc = m_highlights_panel.DockControl;
			if (dc.DockPane != null)
				dc.Close();
			else
				m_dock.Add(m_highlights_panel, EDockSite.Right);
		}

		/// <summary>Toggle the filters panel visibility</summary>
		public ICommand ToggleFilters { get; }

		/// <summary>Toggle the find panel visibility</summary>
		public ICommand ToggleFind { get; }

		/// <summary>Toggle the bookmarks panel visibility</summary>
		public ICommand ToggleBookmarks { get; }

		/// <summary>Toggle the transforms panel visibility</summary>
		public ICommand ToggleTransforms { get; }

		/// <summary>Toggle the actions panel visibility</summary>
		public ICommand ToggleActions { get; }
		private void ToggleFiltersInternal()
		{
			var dc = m_filters_panel.DockControl;
			if (dc.DockPane != null)
				dc.Close();
			else
				m_dock.Add(m_filters_panel, EDockSite.Right);
		}
		private void ToggleFindInternal()
		{
			var dc = m_find_panel.DockControl;
			if (dc.DockPane != null)
				dc.Close();
			else
				m_dock.Add(m_find_panel, EDockSite.Bottom);
		}
		private void ToggleBookmarksInternal()
		{
			var dc = m_bookmarks_panel.DockControl;
			if (dc.DockPane != null)
				dc.Close();
			else
				m_dock.Add(m_bookmarks_panel, EDockSite.Bottom);
		}
		private void ToggleTransformsInternal()
		{
			var dc = m_transforms_panel.DockControl;
			if (dc.DockPane != null)
				dc.Close();
			else
				m_dock.Add(m_transforms_panel, EDockSite.Right);
		}
		private void ToggleActionsInternal()
		{
			var dc = m_actions_panel.DockControl;
			if (dc.DockPane != null)
				dc.Close();
			else
				m_dock.Add(m_actions_panel, EDockSite.Right);
		}

		/// <summary>Scroll the grid to the given line index and select it</summary>
		private void ScrollToLine(int line_index)
		{
			if (line_index < 0 || line_index >= Main.Count) return;
			m_grid.SelectedIndex = line_index;
			m_grid.ScrollIntoView(m_grid.Items[line_index]);
		}

		/// <summary>Handle grid selection changes</summary>
		private void HandleGridSelectionChanged(object sender, SelectionChangedEventArgs e)
		{
			PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(nameof(StatusFilePosition)));
			PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(nameof(StatusSelection)));
		}

		/// <summary>The index of the currently selected row, or -1 if none</summary>
		private int SelectedRowIndex => m_grid.SelectedIndex;

		/// <summary>The data to display in the grid</summary>
		public IReadOnlyCollection<ILine> LogData => Main;

		/// <summary>Whether tail mode is enabled</summary>
		public bool TailEnabled
		{
			get => Main.TailEnabled;
			set
			{
				Main.TailEnabled = value;
				PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(nameof(TailEnabled)));
			}
		}

		/// <summary>Whether watch mode is enabled</summary>
		public bool WatchEnabled
		{
			get => Main.WatchEnabled;
			set
			{
				Main.WatchEnabled = value;
				PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(nameof(WatchEnabled)));
			}
		}

		/// <summary>Application title</summary>
		public string WindowTitle => Main.LogDataSource != null ? $"RyLogViewer - {Main.LogDataSource.Path}" : $"RyLogViewer";

		/// <summary>Status for the current log data source</summary>
		public string StatusLogDataSource => Main.LogDataSource == null ? "No Log Data Source" : string.Empty;

		/// <summary>Status for the current file position</summary>
		public string StatusFilePosition
		{
			get
			{
				var r = SelectedRowIndex;
				var pos = (r >= 0 && r < Main.Count) ? Main[r].FileByteRange.Beg : 0;
				var end = Main.FileByteRange.End;
				return $"Position: {pos:N0} / {end:N0} bytes";
			}
		}

		/// <summary>Status for the current selection</summary>
		public string StatusSelection
		{
			get
			{
				var items = m_grid.SelectedItems;
				if (items.Count == 0)
					return "Selection: (none)";

				// Get the byte range spanning the selected rows
				var first = items[0] as ILine;
				var last = items[items.Count - 1] as ILine;
				if (first == null || last == null)
					return "Selection: (none)";

				var beg = first.FileByteRange.Beg;
				var end = last.FileByteRange.End;
				var size = end - beg;
				return $"Selection: [{beg:N0} - {end:N0}] ({size:N0} bytes)";
			}
		}

		/// <summary>Status for the current encoding</summary>
		public string StatusEncoding
		{
			get
			{
				var s = Main.EncodingName;
				return string.IsNullOrEmpty(s) ? "Encoding" : s;
			}
		}

		/// <summary>Status for the current line ending mode</summary>
		public string StatusLineEnding
		{
			get
			{
				var s = Main.LineEnding;
				return string.IsNullOrEmpty(s) ? "Line Ending" : s;
			}
		}

		/// <summary>Add a file to the recent file paths</summary>
		public void AddToRecentFiles(string filepath)
		{
			m_recent_files.Add(filepath);

			// Update the settings
			Settings.General.LastLoadedFile = filepath;
			Settings.RecentFiles = m_recent_files.Export();
		}
	}
}
