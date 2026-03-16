using System;
using System.Collections;
using System.Collections.Generic;
using System.Collections.Specialized;
using System.ComponentModel;
using System.IO;
using System.Text;
using System.Windows.Threading;
using Rylogic.Common;
using Rylogic.Plugin;
using Rylogic.Utility;
using RyLogViewer.Options;

namespace RyLogViewer
{
	public class Main : IReadOnlyList<ILine>, INotifyCollectionChanged, INotifyPropertyChanged, IDisposable
	{
		private readonly IReport m_report;
		private DispatcherTimer? m_tail_timer;
		private FileSystemWatcher? m_file_watcher;
		private long m_last_known_file_length;

		public Main(StartupOptions su, Settings settings, IReport report)
		{
			m_report = report;
			Settings = settings;

			Highlights = new HighLightContainer();
			Filters = new FilterContainer();
			Bookmarks = new BookmarkContainer();
			Transforms = new TransformContainer();
			Actions = new ClickActionContainer();

			// Rebuild the index when filters change
			Filters.FiltersChanged += (s, e) => RebuildIndex();
			Highlights.HighlightsChanged += (s, e) =>
			{
				// In quick-filter mode, highlights act as filters
				if (Settings.QuickFilterEnabled)
					RebuildIndex();

				// Always notify so the grid repaints with new highlight colours
				CollectionChanged?.Invoke(this, new NotifyCollectionChangedEventArgs(NotifyCollectionChangedAction.Reset));
			};

			// Tail mode timer — polls the file for new content
			m_tail_timer = new DispatcherTimer { Interval = TimeSpan.FromMilliseconds(500) };
			m_tail_timer.Tick += HandleTailTimerTick;

			LoadPlugins();
		}

		/// <summary>Clean up resources</summary>
		public void Dispose()
		{
			if (m_tail_timer != null)
			{
				m_tail_timer.Stop();
				m_tail_timer.Tick -= HandleTailTimerTick;
				m_tail_timer = null;
			}
			StopFileWatcher();
			LogDataSource = null;
		}

		/// <summary>Notify property changes</summary>
		public event PropertyChangedEventHandler? PropertyChanged;

		/// <summary>Raised when the collection of log data lines changes</summary>
		public event NotifyCollectionChangedEventHandler? CollectionChanged;

		/// <summary>Application settings</summary>
		private Settings Settings
		{
			get => m_settings;
			set
			{
				if (m_settings == value) return;
				if (m_settings != null)
				{
					m_settings.SettingChange -= HandleSettingsChange;
				}
				m_settings = value;
				if (m_settings != null)
				{
					m_settings.SettingChange += HandleSettingsChange;
				}

				// Handler
				void HandleSettingsChange(object? sender, SettingChangeEventArgs e)
				{
					if (e.Before) return;
					if (e.Key == nameof(Options.Settings.TailEnabled))
						UpdateTailMode();
					if (e.Key == nameof(Options.Settings.WatchEnabled))
					{
						if (Settings.WatchEnabled) StartFileWatcher();
						else StopFileWatcher();
					}
				}
			}
		}
		private Settings m_settings = null!;

		/// <summary>The highlight patterns</summary>
		public HighLightContainer Highlights { get; }

		/// <summary>The filter patterns</summary>
		public FilterContainer Filters { get; }

		/// <summary>The bookmarks</summary>
		public BookmarkContainer Bookmarks { get; }

		/// <summary>The text transforms</summary>
		public TransformContainer Transforms { get; }

		/// <summary>The click actions</summary>
		public ClickActionContainer Actions { get; }

		/// <summary>Lists the names of the available formatters</summary>
		public IEnumerable<string> AvailableFormatters
		{
			get
			{
				yield return SingleLineFormatter.Name;
				yield return DelimitedLineFormatter.Name;
				yield return JsonLineFormatter.Name;
				foreach (var fmt in m_plugin_formatters.Instances)
					yield return fmt.Name;
			}
		}

		/// <summary>The name of the encoding currently in use</summary>
		public string EncodingName
		{
			get
			{
				if (Lines != null)
					return Tools.Humanise(Lines.LineIndex.Encoding.EncodingName);
				if (!string.IsNullOrEmpty(Settings.Format.Encoding))
					return Settings.Format.Encoding;
				return string.Empty;
			}
		}

		/// <summary>The line ending sequence currently in use</summary>
		public string LineEnding
		{
			get
			{
				if (Lines != null)
					return Tools.Humanise(Encoding.UTF8.GetString(Lines.LineIndex.LineEnd));
				if (!string.IsNullOrEmpty(Settings.Format.LineEnding))
					return Settings.Format.LineEnding;
				return string.Empty;
			}
		}

		/// <summary>The byte range of the current log data source</summary>
		public RangeI FileByteRange => Lines?.LineIndex.FileByteRange ?? RangeI.Zero;

		/// <summary>Get/Set the log data to be displayed</summary>
		public ILogDataSource? LogDataSource
		{
			get => m_log_data_source;
			set
			{
				if (m_log_data_source == value) return;
				if (m_log_data_source != null)
				{
					// Clear any references to the current log data
					StopFileWatcher();
					m_tail_timer?.Stop();
					Bookmarks.Clear();

					Lines = null!;
					Util.Dispose(ref m_log_data_source!);
				}
				m_log_data_source = value;
				if (m_log_data_source != null)
				{
					// Determine the data source encoding
					var encoding = ChooseEncoding(m_log_data_source);
					var line_end = ChooseLineEnding(m_log_data_source, encoding);

					// Create an index for identifying lines
					var line_index = new LineIndex(m_log_data_source, encoding, line_end, m_settings);

					// Create a formatter for the lines
					var formatter = CreateFormatter(m_log_data_source, encoding);

					// Create a cache of lines to reduce formatting load
					Lines = new LineCache(line_index, formatter, m_report);

					// Track file length for tail mode
					using (var stream = m_log_data_source.OpenStream())
						m_last_known_file_length = stream.Length;

					// Build the log index
					Build(LogOpenPosition, true);

					// Start tail and watch modes
					UpdateTailMode();
					StartFileWatcher();
				}

				PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(nameof(LogDataSource)));
				CollectionChanged?.Invoke(this, new NotifyCollectionChangedEventArgs(NotifyCollectionChangedAction.Reset));
			}
		}
		private ILogDataSource? m_log_data_source;

		/// <summary>The cache of lines for the current log data source</summary>
		private LineCache Lines
		{
			get;
			set
			{
				if (field == value) return;
				if (field != null)
				{
					field.CollectionChanged -= HandleCollectionChanged;
					Util.Dispose(ref field!);
				}
				field = value;
				if (field != null)
				{
					field.CollectionChanged += HandleCollectionChanged;
				}

				// Handlers
				void HandleCollectionChanged(object? sender, NotifyCollectionChangedEventArgs e)
				{
					CollectionChanged?.Invoke(this, e);
				}
			}
		} = null!;

		/// <summary>Trigger an update of the line index</summary>
		private void Build(long filepos, bool reload)
		{
			// Compile a list of filters
			var filters = new List<IFilter>(Filters);
			if (Settings.QuickFilterEnabled)
			{
				// Add a RejectAll so that non-highlighted means discard
				filters.AddRange(Highlights);
				filters.Add(Filter.RejectAll);
			}

			// Build the line index
			Lines.LineIndex.Build(filepos, reload, filters);
		}

		/// <summary>Rebuild the line index at the current position</summary>
		private void RebuildIndex()
		{
			if (Lines == null) return;
			Build(Lines.LineIndex.FileByteRange.End, true);
		}

		/// <summary>Handle tail timer tick — check if file has grown</summary>
		private void HandleTailTimerTick(object? sender, EventArgs e)
		{
			if (LogDataSource == null || Lines == null) return;
			try
			{
				using var stream = LogDataSource.OpenStream();
				var length = stream.Length;
				if (length != m_last_known_file_length)
				{
					m_last_known_file_length = length;

					// Rebuild at the end of the file to show new content
					Build(length, false);
				}
			}
			catch
			{
				// File may be temporarily locked — ignore and try again next tick
			}
		}

		/// <summary>Handle file system change — the file has been modified externally</summary>
		private void HandleFileChanged(object sender, FileSystemEventArgs e)
		{
			// FileSystemWatcher fires on a thread pool thread, so marshal to UI
			m_tail_timer?.Dispatcher.BeginInvoke(new Action(() =>
			{
				if (LogDataSource == null || Lines == null) return;
				try
				{
					using var stream = LogDataSource.OpenStream();
					m_last_known_file_length = stream.Length;

					// Rebuild at the current position
					Build(Lines.LineIndex.FileByteRange.End, true);
				}
				catch
				{
					// File may be temporarily locked
				}
			}));
		}

		/// <summary>Start or stop the tail timer based on settings</summary>
		private void UpdateTailMode()
		{
			if (m_tail_timer == null) return;
			if (Settings.TailEnabled && LogDataSource != null)
				m_tail_timer.Start();
			else
				m_tail_timer.Stop();
		}

		/// <summary>Start watching the current file for external changes</summary>
		private void StartFileWatcher()
		{
			StopFileWatcher();
			if (!Settings.WatchEnabled || LogDataSource == null) return;

			// Only watch file-based sources
			var filepath = LogDataSource.Path;
			if (string.IsNullOrEmpty(filepath) || !File.Exists(filepath)) return;

			var dir = System.IO.Path.GetDirectoryName(filepath);
			var name = System.IO.Path.GetFileName(filepath);
			if (dir == null || name == null) return;

			m_file_watcher = new FileSystemWatcher(dir, name)
			{
				NotifyFilter = NotifyFilters.LastWrite | NotifyFilters.Size,
				EnableRaisingEvents = true,
			};
			m_file_watcher.Changed += HandleFileChanged;
		}

		/// <summary>Stop watching the file</summary>
		private void StopFileWatcher()
		{
			if (m_file_watcher != null)
			{
				m_file_watcher.EnableRaisingEvents = false;
				m_file_watcher.Changed -= HandleFileChanged;
				m_file_watcher.Dispose();
				m_file_watcher = null;
			}
		}

		/// <summary>Raised when the user scrolls to the end of the file (for tail mode)</summary>
		public bool TailEnabled
		{
			get => Settings.TailEnabled;
			set
			{
				if (Settings.TailEnabled == value) return;
				Settings.TailEnabled = value;
				UpdateTailMode();
				PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(nameof(TailEnabled)));
			}
		}

		/// <summary>Whether file watching is enabled</summary>
		public bool WatchEnabled
		{
			get => Settings.WatchEnabled;
			set
			{
				if (Settings.WatchEnabled == value) return;
				Settings.WatchEnabled = value;
				if (value) StartFileWatcher();
				else StopFileWatcher();
				PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(nameof(WatchEnabled)));
			}
		}

		/// <summary>Create a new instance of a formatter of the currently selected type</summary>
		private ILineFormatter CreateFormatter(ILogDataSource src, Encoding encoding)
		{
			switch (Settings.Format.Formatter)
			{
				case SingleLineFormatter.Name:
				{
					return new SingleLineFormatter(encoding);
				}
				case DelimitedLineFormatter.Name:
				{
					var delimiter = Settings.Format.ColDelimiter;
					if (string.IsNullOrEmpty(delimiter)) delimiter = "\t";
					return new DelimitedLineFormatter(encoding, delimiter);
				}
				case JsonLineFormatter.Name:
				{
					return new JsonLineFormatter(encoding);
				}
				default:
				{
					foreach (var fmt in m_plugin_formatters.Instances)
					{
						if (fmt.Name != Settings.Format.Formatter) continue;
						return fmt;
					}
					return new SingleLineFormatter(encoding);
				}
			}
		}

		/// <summary>Choose an encoding to apply to 'src'</summary>
		private Encoding ChooseEncoding(ILogDataSource src)
		{
			var auto_detect = string.IsNullOrEmpty(Settings.Format.Encoding);
			if (!auto_detect)
				return Tools.GetEncoding(Settings.Format.Encoding);

			// Auto detect the encoding
			using (var stream = src.OpenStream())
				return Tools.GuessEncoding(stream, out var _);
		}

		/// <summary>Choose the byte sequence to mark line endings</summary>
		private byte[] ChooseLineEnding(ILogDataSource src, Encoding encoding)
		{
			var auto_detect = string.IsNullOrEmpty(Settings.Format.LineEnding);
			if (!auto_detect)
				return Tools.GetLineEnding(Settings.Format.LineEnding);

			// Auto detect the line ending
			using (var stream = src.OpenStream())
				return Tools.GuessLineEnding(stream, encoding, Settings.LogData.MaxLineLength, out var _);
		}

		/// <summary>Load plugin assemblies</summary>
		private void LoadPlugins()
		{
			var plugin_dir = Util.ResolveAppPath("plugins");
			Path_.CreateDirs(plugin_dir);

			// Load line formatters
			m_plugin_formatters.Load(plugin_dir);

			// Report failures
			if (m_plugin_formatters.Failures.Count != 0)
			{
				var sb = new StringBuilder();
				sb.AppendLine("Plugin Load Failures:");
				foreach (var fail in m_plugin_formatters.Failures)
					sb.AppendLine(fail.Name).AppendLine(fail.Error.Message).AppendLine();

				// This will need delaying until the window is visible
				m_report.ErrorPopup(sb.ToString());
			}
		}
		private Plugins<ILineFormatter> m_plugin_formatters = new();

		/// <summary>The position in the log to open</summary>
		private long LogOpenPosition
		{
			get
			{
				if (LogDataSource == null)
					throw new NullReferenceException($"{nameof(LogDataSource)} is null");

				if (Settings.LogData.OpenAtEnd)
				{
					using var src = LogDataSource.OpenStream();
					return src.Length;
				}
				return 0L;
			}
		}

		#region IReadOnlyList<ILine>
		public int Count => Lines?.Count ?? 0;
		public ILine this[int index] => Lines?[index] ?? new ErrorLine("Line Index out of range", RangeI.Zero);
		#endregion
		#region IEnumerable<ILine>
		public IEnumerator<ILine> GetEnumerator()
		{
			var lines = (IReadOnlyList<ILine>)Lines;
			return (lines ?? new ILine[0]).GetEnumerator();
		}
		IEnumerator IEnumerable.GetEnumerator()
		{
			return GetEnumerator();
		}
		#endregion
	}
}
