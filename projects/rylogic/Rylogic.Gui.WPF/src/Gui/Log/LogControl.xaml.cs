using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Collections.Specialized;
using System.ComponentModel;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Text;
using System.Text.RegularExpressions;
using System.Threading;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Threading;
using System.Xml.Linq;
using Microsoft.Win32;
using Rylogic.Common;
using Rylogic.Extn;
using Rylogic.Interop.Win32;
using Rylogic.Utility;

namespace Rylogic.Gui.WPF
{
	public partial class LogControl : UserControl, IDockable, IDisposable, INotifyPropertyChanged, LogControl.ILogEntryPatternProvider
	{
		// Usage:
		//   This component can be used in two ways, one is as a simple text
		//   panel for displaying log messages, the second is as a view of a
		//   log file.
		//   1)
		//    - Call AddMessage()
		//   2)
		//    - Add an instance of the control
		//    - Set the log filepath
		// Notes:
		//  - The EntryDelimiter must be a single byte UTF8 character because the log control
		//    reads bytes in blocks from the log file, and doesn't support delimiters spanning
		//    blocks (for performance).
		//  - The EntryDelimiter is *NOT* needed in the LogEntryPattern.

		private const int FilePollPeriodMS = 100;

		public LogControl()
		{
			InitializeComponent();
			m_view.MouseRightButtonUp += DataGrid_.ColumnVisibility;
			m_view.SelectionChanged += HandleSelectionChanged;
			m_view.ContextMenuOpening += HandleContextMenuOpening;

			// Support for dock container controls
			DockControl = new DockControl(this, "log")
			{
				TabText = "Log",
				DefaultDockLocation = new DockContainer.DockLocation(auto_hide: EDockSite.Right),
			};

			// When docked in an auto-hide panel, pop out on new messages
			PopOutOnNewMessages = true;

			// Show log filepath related UI by default
			ShowLogFilepath = true;

			// Line wrap default
			LineWrap = true;

			// Hide diagnostics by default
			FilterLevel = ELogLevel.Info;

			// A buffer of the log entries.
			// This is populated by calls to AddMessage or from the log file.
			LogEntries = new ObservableCollection<LogEntry>();

			// Guess the log starts now
			Epoch = DateTimeOffset.Now;

			// Define a line in the log
			LogEntryPattern = null;
			// Examples:
			//   Use the 'ColumnNames' for tags so the columns become visible
			//   new Regex(@"^(?<File>.*?)\|(?<Level>.*?)\|(?<Timestamp>.*?)\|(?<Elapsed>.*?)\|(?<Message>.*)\|\n",RegexOptions.Singleline|RegexOptions.Multiline|RegexOptions.CultureInvariant|RegexOptions.Compiled);
			//   The log entry pattern should not typically contain the line delimiter character

			// Highlighting patterns
			Highlighting = new ObservableCollection<HLPattern>();
			// Examples:
			//   Highlighting.Add(new HLPattern(Color_.From("#FCF"), Color_.From("#C0C"), EPattern.RegularExpression, ".*fatal.*") { IgnoreCase = true });
			//   Highlighting.Add(new HLPattern(Color_.From("#FDD"), Color_.From("#F00"), EPattern.RegularExpression, ".*error.*") { IgnoreCase = true });
			//   Highlighting.Add(new HLPattern(Colors.Transparent, Color_.From("#E70"), EPattern.RegularExpression, ".*warn.*") { IgnoreCase = true });
			//   Highlighting.Add(new HLPattern(Colors.Transparent, Color_.From("#888"), EPattern.RegularExpression, ".*debug.*") { IgnoreCase = true });

			// The log entry delimiter
			EntryDelimiter = Log_.EntryDelimiter;

			// Limit the number of log entries to display
			MaxLines = int.MaxValue;
			MaxFileBytes = 100 * 1024 * 1024;
			FileBaseOffset = 0;

			// Commands
			BrowseForLogFile = Command.Create(this, BrowseForLogFileInternal);
			ClearLog = Command.Create(this, ClearLogInternal);
			ToggleTailScroll = Command.Create(this, ToggleTailScrollInternal);
			ToggleLineWrap = Command.Create(this, ToggleLineWrapInternal);
			CopySelectedRowsToClipboard = Command.Create(this, CopySelectedRowsToClipboardInternal);
			CopySelectedMessagesToClipboard = Command.Create(this, CopySelectedMessagesToClipboardInternal);

			// Don't set DataContext, it must be inherited
			UpdateColumnVisibility();
		}
		public void Dispose()
		{
			Dispose(true);
			GC.SuppressFinalize(this);
		}
		protected virtual void Dispose(bool _)
		{
			m_view.ContextMenuOpening -= HandleContextMenuOpening;
			m_view.SelectionChanged -= HandleSelectionChanged;
			m_view.MouseRightButtonUp -= DataGrid_.ColumnVisibility;
			BindingOperations.ClearAllBindings(this);
			LogEntries = null!;
			LogFilepath = null;
			Highlighting = null!;
			DockControl = null!;
		}
		protected override void OnPreviewKeyDown(KeyEventArgs e)
		{
			base.OnPreviewKeyDown(e);
			var key = e.Key.ToKeyCode(include_modifier_keys: false);
			if (key == EKeyCodes.End && (Keyboard.IsKeyDown(Key.LeftCtrl) || Keyboard.IsKeyDown(Key.RightCtrl)))
			{
				TailScroll = true;
				e.Handled = true;
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
					field.ActiveChanged -= HandleActiveChanged;
					field.DockContainerChanged -= HandleDockContainerChanged;
					field.SavingLayout -= HandleSavingLayout;
					field.LoadingLayout -= HandleLoadingLayout;
					Util.Dispose(ref field!);
				}
				field = value;
				if (field != null)
				{
					field.LoadingLayout += HandleLoadingLayout;
					field.SavingLayout += HandleSavingLayout;
					field.DockContainerChanged += HandleDockContainerChanged;
					field.ActiveChanged += HandleActiveChanged;
				}

				// Handlers
				void HandleLoadingLayout(object? sender, DockContainerLoadingLayoutEventArgs e)
				{
					LoadSettings(e.UserData);
				}
				void HandleSavingLayout(object? sender, DockContainerSavingLayoutEventArgs e)
				{
					SaveSettings(e.Node);
				}
				void HandleActiveChanged(object? sender, EventArgs e)
				{
					if (DockControl.IsActiveContent)
					{
						//DockControl.TabColoursActive.Text = Color.Black;
						//DockControl.InvalidateTab();
					}
				}
				void HandleDockContainerChanged(object? sender, DockContainerChangedEventArgs e)
				{
					OnDockContainerChanged(e);
				}
			}
		} = null!;
		protected virtual void OnDockContainerChanged(DockContainerChangedEventArgs args)
		{ }

		/// <summary>Get/Set the log file to display. Setting a filepath sets the control to the 'file tail' mode</summary>
		public string? LogFilepath
		{
			get => (string?)GetValue(LogFilepathProperty);
			set => SetValue(LogFilepathProperty, value);
		}
		private void LogFilepath_Changed(string? new_value, string? old_value)
		{
			// Allow setting to the same value
			if (old_value != null)
			{
				m_watch!.Remove(old_value);
				Util.Dispose(ref m_watch);
			}
			FileBaseOffset = 0;
			if (new_value != null)
			{
				m_watch = new FileWatch { PollPeriod = TimeSpan.FromMilliseconds(FilePollPeriodMS) };
				m_watch.Add(new_value, HandleFileChanged);
				HandleFileChanged(new_value, null);
			}
			NotifyPropertyChanged(nameof(LogFilepath));

			// Handlers
			bool HandleFileChanged(string filepath, object? ctx)
			{
				try
				{
					// Open the file
					using var file = new FileStream(filepath, FileMode.Open, FileAccess.Read, FileShare.ReadWrite | FileShare.Delete);

					// Get the file length (remember this is potentially changing all the time)
					var fend = file.Seek(0, SeekOrigin.End);

					// Determine the file offset to start reading from
					var fpos = Math.Max(FileBaseOffset, fend - MaxFileBytes);
					var le = LogEntries.LastOrDefault(x => x.FromFile);
					if (le != null) fpos = Math.Min(le.FPos, fend);

					// Read forwards from 'pos' to the end of the file
					for (; fpos != fend;)
					{
						// Read a chunk from the file
						var len = (int)Math.Min(fend - fpos, m_buf.Length);
						file.Seek(fpos, SeekOrigin.Begin);
						if (file.Read(m_buf, 0, len) != len)
							throw new Exception("Failed to read file");

						// Extract log entries from the file data
						var fstart = fpos;
						for (int s = 0, e = 0; fpos != fend; s = e)
						{
							// Log lines start with the ESC character '\u001b'
							for (; s != len && m_buf[s] != EntryDelimiter; ++s) { }
							fpos = fstart + s;

							// No log lines found
							if (s == len)
								break;

							// Find the start of the next log entry
							for (e = s + 1; e != len && m_buf[e] != EntryDelimiter; ++e) { }

							// [s,e) is a log entry. If 'm_buf[e]' is not the delimiter then it may be a partial entry
							var entry = new LogEntry(this, fstart + s, Encoding.UTF8.GetString(m_buf, s + 1, e - s - 1), true);

							// Add to the log entries collection. Replace any partial log entry
							int i = LogEntries.Count;
							for (; i-- != 0 && (!LogEntries[i].FromFile || LogEntries[i].FPos > fpos);) { }
							if (i >= 0 && LogEntries[i].FPos == entry.FPos)
								LogEntries[i] = entry;
							else
								LogEntries.Add(entry);

							// If we haven't read to the end of the file, re-read from 's'.
							// If we have read to the end of the file, exit the loop.
							if (e == len && len != m_buf.Length)
								fpos = fend;
						}
					}
				}
				catch (Exception ex)
				{
					AddMessage($"Log Error: {ex.Message}");
				}
				return true;
			}
		}
		public static readonly DependencyProperty LogFilepathProperty = Gui_.DPRegister<LogControl>(nameof(LogFilepath), null, Gui_.EDPFlags.TwoWay);
		private byte[] m_buf = new byte[8192];
		private FileWatch? m_watch;

		/// <summary>A regex expression for extracting lines from the log file</summary>
		public Regex? LogEntryPattern
		{
			get => (Regex?)GetValue(LogEntryPatternProperty);
			set => SetValue(LogEntryPatternProperty, value);
		}
		private void LogEntryPattern_Changed()
		{
			// Examples:
			//   Use the 'ColumnNames' for tags so the columns become visible
			//   new Regex(@"^(?<File>.*?)\|(?<Tag>.*?)\|(?<Level>.*?)\|(?<Timestamp>.*?)\|(?<Message>.*)\n"
			//       ,RegexOptions.Singleline|RegexOptions.Multiline|RegexOptions.CultureInvariant|RegexOptions.Compiled);
			// Notes:
			//   - The pattern doesn't need to include the entry delimiter.
			//     The pattern is only applied to each row read from the log file.
			//     The entry delimiter is used to determine a "row" and is therefore not part of each row.
			UpdateColumnVisibility();
			SignalRefresh();
			NotifyPropertyChanged(nameof(LogEntryPattern));
		}
		public static readonly DependencyProperty LogEntryPatternProperty = Gui_.DPRegister<LogControl>(nameof(LogEntryPattern), null, Gui_.EDPFlags.TwoWay);

		/// <summary>The time of the first log entry</summary>
		public DateTimeOffset Epoch
		{
			get => (DateTimeOffset)GetValue(EpochProperty);
			set => SetValue(EpochProperty, value);
		}
		private void Epoch_Changed() => SignalRefresh();
		public static readonly DependencyProperty EpochProperty = Gui_.DPRegister<LogControl>(nameof(Epoch), DateTimeOffset.Now, Gui_.EDPFlags.None);

		/// <summary>If docked in a doc container, pop-out when new messages are added to the log</summary>
		public bool PopOutOnNewMessages
		{
			get => (bool)GetValue(PopOutOnNewMessagesProperty);
			set => SetValue(PopOutOnNewMessagesProperty, value);
		}
		public static readonly DependencyProperty PopOutOnNewMessagesProperty = Gui_.DPRegister<LogControl>(nameof(PopOutOnNewMessages), Boxed.True, Gui_.EDPFlags.None);

		/// <summary>Show UI parts related to log files</summary>
		public bool ShowLogFilepath
		{
			get => (bool)GetValue(ShowLogFilepathProperty);
			set => SetValue(ShowLogFilepathProperty, value);
		}
		public static readonly DependencyProperty ShowLogFilepathProperty = Gui_.DPRegister<LogControl>(nameof(ShowLogFilepath), Boxed.True, Gui_.EDPFlags.None);

		/// <summary>Line wrap mode</summary>
		public bool LineWrap
		{
			get => (bool)GetValue(LineWrapProperty);
			set => SetValue(LineWrapProperty, value);
		}
		private void LineWrap_Changed(bool new_value)
		{
			m_view.HorizontalScrollBarVisibility = new_value ? ScrollBarVisibility.Disabled : ScrollBarVisibility.Auto;
		}
		public static readonly DependencyProperty LineWrapProperty = Gui_.DPRegister<LogControl>(nameof(LineWrap), Boxed.False, Gui_.EDPFlags.TwoWay);

		/// <summary>Command to invoke for navigating to the selected log entry's source location</summary>
		public Command? GoToLogEntry
		{
			get => (Command?)GetValue(GoToLogEntryProperty);
			set => SetValue(GoToLogEntryProperty, value);
		}
		private void GoToLogEntry_Changed()
		{
			GoToLogEntry?.NotifyCanExecuteChanged();
			NotifyPropertyChanged(nameof(GoToLogEntry));
		}
		public static readonly DependencyProperty GoToLogEntryProperty = Gui_.DPRegister<LogControl>(nameof(GoToLogEntry), null, Gui_.EDPFlags.None);

		/// <summary>Scroll the view to the last entry</summary>
		public bool TailScroll
		{
			get => (bool)GetValue(TailScrollProperty);
			set => SetValue(TailScrollProperty, value);
		}
		private void TailScroll_Changed(bool new_value)
		{
			if (new_value)
				ScrollToEnd();
		}
		public static readonly DependencyProperty TailScrollProperty = Gui_.DPRegister<LogControl>(nameof(TailScroll), Boxed.False, Gui_.EDPFlags.TwoWay);

		/// <summary>The minimum log level to display</summary>
		public ELogLevel FilterLevel
		{
			get => (ELogLevel)GetValue(FilterLevelProperty);
			set => SetValue(FilterLevelProperty, value);
		}
		private void FilterLevel_Changed() => LogEntriesView?.Refresh();
		public static readonly DependencyProperty FilterLevelProperty = Gui_.DPRegister<LogControl>(nameof(FilterLevel), ELogLevel.Info, Gui_.EDPFlags.TwoWay);

		/// <summary>The column names that are hidden by default (Delimited by spaces, commas, or semicolons)</summary>
		public string HiddenColumns
		{
			get => (string)GetValue(HiddenColumnsProperty);
			set => SetValue(HiddenColumnsProperty, value);
		}
		private void HiddenColumns_Changed() => UpdateColumnVisibility();
		public static readonly DependencyProperty HiddenColumnsProperty = Gui_.DPRegister<LogControl>(nameof(HiddenColumns), string.Empty, Gui_.EDPFlags.None);

		/// <summary>String format for the timestamp</summary>
		public string TimestampFormat
		{
			get => (string)GetValue(TimestampFormatProperty);
			set => SetValue(TimestampFormatProperty, value);
		}
		private void TimestampFormat_Changed() => SignalRefresh();
		public static readonly DependencyProperty TimestampFormatProperty = Gui_.DPRegister<LogControl>(nameof(TimestampFormat), "yyyy-MM-dd HH:mm:ss.fff", Gui_.EDPFlags.None);

		/// <summary>String format for the elapsed time. Optional parts allowed, E.g. "[d\\.][hh\\:][mm\\:]ss\\.fff"</summary>
		public string ElapsedFormat
		{
			get => (string)GetValue(ElapsedFormatProperty);
			set => SetValue(ElapsedFormatProperty, value);
		}
		private void ElapsedFormat_Changed() => SignalRefresh();
		public static readonly DependencyProperty ElapsedFormatProperty = Gui_.DPRegister<LogControl>(nameof(ElapsedFormat), "c", Gui_.EDPFlags.None);

		/// <summary>A special character used to mark the start of a log entry. Must be a 1-byte UTF8 character</summary>
		public char EntryDelimiter { get; set; }

		/// <summary>The maximum number of lines to show in the log</summary>
		public int MaxLines { get; set; }

		/// <summary>The maximum number of bytes to read from the log file</summary>
		public int MaxFileBytes { get; set; }
		
		/// <summary>The minimum position in the log file to read from. Set when 'Clear' is called</summary>
		public long FileBaseOffset { get; set; }

		/// <summary>Get/Set watching of the log file for changes</summary>
		public bool Freeze
		{
			get => m_freeze;
			set
			{
				if (m_freeze == value) return;
				if (m_freeze)
				{
					if (m_watch != null)
						m_watch.PollPeriod = TimeSpan.FromMilliseconds(FilePollPeriodMS);
				}
				m_freeze = value;
				if (m_freeze)
				{
					if (m_watch != null)
						m_watch.PollPeriod = TimeSpan.Zero;
				}
				NotifyPropertyChanged(nameof(Freeze));
			}
		}
		private bool m_freeze;

		/// <summary>A buffer of the log entries</summary>
		public ObservableCollection<LogEntry> LogEntries
		{
			get;
			set
			{
				// Notes:
				//  - Allow public set so that the observable collection can be provided externally.
				//  - Log entry collections can be made thread safe using: 'BindingOperations.EnableCollectionSynchronization(Entries, new object())';

				if (field == value) return;
				if (field != null)
				{
					field.CollectionChanged -= HandleLogEntriesChanged;
					LogEntriesView = new ListCollectionView(Array.Empty<LogEntry>());
				}
				field = value;
				if (field != null)
				{
					LogEntriesView = new ListCollectionView(field);
					field.CollectionChanged += HandleLogEntriesChanged;
				}

				// Notify properties changed
				NotifyPropertyChanged(nameof(LogEntries));
				NotifyPropertyChanged(nameof(LogEntriesView));

				// Handlers
				void HandleLogEntriesChanged(object? sender, NotifyCollectionChangedEventArgs e)
				{
					if (e.Action != NotifyCollectionChangedAction.Add)
						return;

					// Handle the entries collection being changed from a different thread
					if (Thread.CurrentThread.ManagedThreadId != Dispatcher.Thread.ManagedThreadId)
					{
						Dispatcher.BeginInvoke(new Action(() => HandleLogEntriesChanged(sender, e)));
						return;
					}

					// If a log entry was added, pop-out.
					if (DockControl?.DockContainer != null)
					{
						if (PopOutOnNewMessages)
							DockControl.DockContainer.FindAndShow(this);
						else if (DockControl.TabState != ETabState.Active)
							DockControl.TabState = ETabState.Flashing;
					}

					// Prevent the LogEntries collection getting too big.
					// Defer because we can't edit the collection in this handler.
					if (LogEntries.Count > MaxLines)
						Dispatcher.BeginInvoke(new Action(() =>
						{
							if (LogEntries == null) return;
							LogEntries.RemoveRange(0, LogEntries.Count - MaxLines);
						}));

					// Auto scroll to the last row
					// Have to do this outside of the event handler or we get an exception
					// about the "ItemsSource being inconsistent with the ItemsControl"
					if (TailScroll)
						Dispatcher.BeginInvoke(new Action(ScrollToEnd));
				}
			}
		} = null!;

		/// <summary>Binding view of the log entries</summary>
		public ICollectionView LogEntriesView
		{
			get;
			private set
			{
				if (field == value) return;
				if (field != null)
				{
					field.CurrentChanged -= HandleLogEntriesViewCurrentChanged;
				}
				field = value;
				if (field != null)
				{
					field.Filter = obj => obj is LogEntry le && le.Level >= FilterLevel;
					field.CurrentChanged += HandleLogEntriesViewCurrentChanged;
				}
			}
		} = null!;
		private void HandleLogEntriesViewCurrentChanged(object? sender, EventArgs e)
		{
			NotifyPropertyChanged(nameof(CurrentLogEntry));
			GoToLogEntry?.NotifyCanExecuteChanged();
		}

		/// <summary>Trigger a refresh</summary>
		private void SignalRefresh()
		{
			if (m_log_entry_view_refresh_pending) return;
			m_log_entry_view_refresh_pending = true;
			Dispatcher.BeginInvoke(new Action(() =>
			{
				LogEntriesView?.Refresh();
				m_log_entry_view_refresh_pending = false;
			}));
		}
		private bool m_log_entry_view_refresh_pending;

		/// <summary>Highlighting patterns (in priority order)</summary>
		public ObservableCollection<HLPattern> Highlighting
		{
			get;
			set
			{
				if (field == value) return;
				if (field != null)
				{
					field.CollectionChanged -= HandleCollectionChanged;
				}
				field = value;
				if (field != null)
				{
					field.CollectionChanged += HandleCollectionChanged;
				}

				// Handler
				void HandleCollectionChanged(object? sender, NotifyCollectionChangedEventArgs e)
				{
					// Invalidate all the highlighting patterns
					foreach (var le in LogEntries)
						le.Highlight = null;

					SignalRefresh();
				}
			}
		} = null!;
		IEnumerable<HLPattern> ILogEntryPatternProvider.Highlighting => Highlighting;

		/// <summary>Access to the columns of the log grid</summary>
		public IEnumerable<DataGridColumn> Columns => m_view.Columns;

		/// <summary>The currently visible columns of the log grid</summary>
		public IEnumerable<DataGridColumn> VisibleColumns => m_view.Columns.Where(x => x.Visibility == Visibility.Visible);
		public IEnumerable<string> VisibleColumnNames => VisibleColumns.Select(x => x.Header as string).NotNull();

		/// <summary>The currently selected log entry</summary>
		public LogEntry? CurrentLogEntry =>
			m_view.SelectedItem as LogEntry ??
			m_view.CurrentItem as LogEntry ??
			LogEntriesView.CurrentItem as LogEntry;

		/// <summary>Use the log entry pattern to create columns</summary>
		private void UpdateColumnVisibility()
		{
			// The visible column names, taken from the pattern.
			// Group names should match the values in 'ColumnNames' to support non-string values
			var shown = LogEntryPattern?.GetGroupNames().Skip(1).ToHashSet(x => x) ?? new HashSet<string>();
			if (shown.Count == 0)
			{
				// Only show the 'Text' column, and hide the column headers
				m_view.HeadersVisibility = DataGridHeadersVisibility.None;
				m_view.HorizontalScrollBarVisibility = LineWrap ? ScrollBarVisibility.Disabled : ScrollBarVisibility.Auto;
				foreach (var column in m_view.Columns)
				{
					var name = DataGrid_.GetColumnName(column);
					column.Visibility = name != ColumnNames.Text ? Visibility.Visible : Visibility.Collapsed;
				}
				return;
			}

			// The column names that are hidden by default
			var hidden = HiddenColumns.Split(new[] { ' ', ',', ';' }, StringSplitOptions.RemoveEmptyEntries).ToHashSet(x => x);
		
			// Show each column that has a matching capture group in the pattern
			m_view.HeadersVisibility = DataGridHeadersVisibility.Column;
			m_view.HorizontalScrollBarVisibility = LineWrap ? ScrollBarVisibility.Disabled : ScrollBarVisibility.Auto;
			foreach (var column in m_view.Columns)
			{
				var name = DataGrid_.GetColumnName(column);

				Visibility state;
				state = shown.Contains(name) ? Visibility.Visible : Visibility.Collapsed;
				state = !hidden.Contains(name) ? state : Visibility.Collapsed;
				column.Visibility = state;
			}

			NotifyPropertyChanged(nameof(VisibleColumns));
			NotifyPropertyChanged(nameof(VisibleColumnNames));
		}

		/// <summary>Update the log file to display</summary>
		private void OpenLogFile(string? filepath)
		{
			// Prompt for a log file if not given
			if (filepath == null)
			{
				var dlg = new OpenFileDialog { Title = "Select a log file to display" };
				if (dlg.ShowDialog(Window.GetWindow(this)) != true)
					return;

				filepath = dlg.FileName;
			}

			// Set the log file to display
			LogFilepath = filepath;
		}

		/// <summary>Reset the error log to empty</summary>
		public void Clear()
		{
			// Determine the file position of the last 'FromFile' log entry.
			if (LogFilepath != null && LogEntries.LastOrDefault(x => x.FromFile) is LogEntry last)
				FileBaseOffset = last.FPos + Encoding.UTF8.GetBytes(last.Text).Length;
			else
				FileBaseOffset = 0;

			LogEntries.Clear();
		}

		/// <summary>Add text to the log. Can be called from any thread</summary>
		public void AddMessage(string text)
		{
			if (Thread.CurrentThread.ManagedThreadId != Dispatcher.Thread.ManagedThreadId)
			{
				Dispatcher.BeginInvoke(new Action(() => AddMessage(text)));
				return;
			}

			// Use the FPos value of the last entry so that if 'AddMessage' is mixed with
			// entries read from a log file, the read position in the log file is preserved.
			var fpos = !LogEntries.Empty() ? LogEntries.Back().FPos : 0;
			LogEntries.Add(new LogEntry(this, fpos, text, false));
		}

		/// <summary>Make the last log entry the selected one</summary>
		public void ScrollToEnd()
		{
			LogEntriesView.MoveCurrentToLast();

			var last = LogEntriesView.CurrentItem;
			if (last != null)
				m_view.ScrollIntoView(last);
		}

		/// <summary>Load/Save settings</summary>
		public void LoadSettings(XElement? node)
		{
			// Allow null to make first-run scenarios easier
			if (node == null)
				return;

			FilterLevel = node.Element(nameof(FilterLevel)).As<ELogLevel>(FilterLevel);

			// Build a settings map from the column settings
			var column_settings = node.Elements("Columns", "Column")
				.Select(x => x.As<string>().Split(','))
				.Where(x => x.Length == 4)
				.Select(x => new
				{
					Header = x[0],
					Idx = int.Parse(x[1]),
					Width = DataGrid_.ParseDataGridLength(x[2]),
					Visibility = Enum<Visibility>.Parse(x[3]),
				})
				.ToList();

			// Apply the settings to the columns
			foreach (var col in Columns)
			{
				if (!(col.Header is string header))
					continue;

				var settings = column_settings.FirstOrDefault(x => x.Header == header);
				if (settings == null)
					continue;

				if (settings.Idx >= m_view.Columns.Count)
					continue;

				col.DisplayIndex = settings.Idx;
				col.Visibility = settings.Visibility;
				col.Width = settings.Width;
			}
		}
		public XElement SaveSettings(XElement? node = null)
		{
			node ??= new XElement("Log");
			node.Add2(nameof(FilterLevel), FilterLevel, false);

			// Only save column info if the layout is valid
			var columns_node = node.Add2(new XElement("Columns"));
			if (IsArrangeValid)
			{
				foreach (var col in Columns)
				{
					if (col.Header is not string header) continue;
					columns_node.Add2("Column", $"{header},{col.DisplayIndex},{col.Width},{col.Visibility}", false);
				}
			}
			return node;
		}

		/// <summary>Browse for a log file to display</summary>
		public Command BrowseForLogFile { get; }
		private void BrowseForLogFileInternal()
		{
			OpenLogFile(null);
		}

		/// <summary>Clear the error log display (not the file)</summary>
		public Command ClearLog { get; }
		private void ClearLogInternal()
		{
			Clear();
		}

		/// <summary>Toggle scrolling to the last log entry</summary>
		public Command ToggleTailScroll { get; }
		private void ToggleTailScrollInternal()
		{
			TailScroll = !TailScroll;
		}

		/// <summary>Toggle line wrapping mode</summary>
		public Command ToggleLineWrap { get; }
		private void ToggleLineWrapInternal()
		{
			LineWrap = !LineWrap;
		}

		/// <summary>Copy the selected rows to the clipboard</summary>
		public Command CopySelectedRowsToClipboard { get; }
		private void CopySelectedRowsToClipboardInternal()
		{
			var sb = new StringBuilder();
			foreach (var le in m_view.SelectedItems.Cast<LogEntry>()) sb.Append(le.Text);
			try { Clipboard.SetData(DataFormats.Text, sb.ToString()); } catch { }
		}

		/// <summary></summary>
		public Command CopySelectedMessagesToClipboard { get; }
		private void CopySelectedMessagesToClipboardInternal()
		{
			var sb = new StringBuilder();
			foreach (var le in m_view.SelectedItems.Cast<LogEntry>()) sb.Append(le.Message);
			try { Clipboard.SetData(DataFormats.Text, sb.ToString()); } catch { }
		}

		/// <summary>Switch out of tail scroll mode when a row other than the last is selected</summary>
		private void HandleMouseDown(object sender, MouseButtonEventArgs e)
		{
			if (e.ChangedButton == MouseButton.Right &&
				e.OriginalSource is DependencyObject hit &&
				Gui_.FindVisualParent<DataGridRow>(hit, root: m_view) is DataGridRow row)
			{
				if (!row.IsSelected)
				{
					m_view.SelectedItems.Clear();
					m_view.SelectedItem = row.Item;
				}
				m_view.CurrentItem = row.Item;
				LogEntriesView.MoveCurrentTo(row.Item);
			}

			Dispatcher.BeginInvoke(new Action(() =>
			{
				// Have to dispatcher this because the current position isn't updated until
				// after the preview mouse down event.
				if (TailScroll && LogEntriesView.CurrentPosition != LogEntriesView.Count() - 1)
					TailScroll = false;
			}));
		}

		/// <summary>Update context menu command availability before the menu is shown</summary>
		private void HandleContextMenuOpening(object sender, ContextMenuEventArgs e)
		{
			GoToLogEntry?.NotifyCanExecuteChanged();
		}

		/// <summary>Update commands that depend on the selected log entry</summary>
		private void HandleSelectionChanged(object sender, SelectionChangedEventArgs e)
		{
			NotifyPropertyChanged(nameof(CurrentLogEntry));
			GoToLogEntry?.NotifyCanExecuteChanged();
		}

		/// <summary>Allow double click on a log entry to do something</summary>
		private void HandleMouseDoubleClick(object sender, MouseButtonEventArgs e)
		{
			if (CurrentLogEntry is LogEntry le)
				LogEntryDoubleClick?.Invoke(this, new LogEntryDoubleClickEventArgs(le));
		}

		/// <summary>Raised when a log entry is double clicked</summary>
		public event EventHandler<LogEntryDoubleClickEventArgs>? LogEntryDoubleClick;
		public class LogEntryDoubleClickEventArgs :EventArgs
		{
			public LogEntryDoubleClickEventArgs(LogEntry le)
			{
				Entry = le;
			}
			public LogEntry Entry { get; }
		}		

		/// <summary></summary>
		public event PropertyChangedEventHandler? PropertyChanged;
		public void NotifyPropertyChanged(string prop_name) => PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(prop_name));

		/// <summary>Provides a regex that describes the format of the log entry</summary>
		public interface ILogEntryPatternProvider
		{
			/// <summary>A regex that describes the format of the log entry</summary>
			Regex? LogEntryPattern { get; }

			/// <summary>The highlighting patterns</summary>
			IEnumerable<HLPattern> Highlighting { get; }

			/// <summary>Format string for timestamps</summary>
			string? TimestampFormat { get; }

			/// <summary>Format string for elapsed time</summary>
			string? ElapsedFormat { get; }
		}

		/// <summary>A single log entry</summary>
		[DebuggerDisplay("[{FPos}] {Text}")]
		public class LogEntry
		{
			private readonly ILogEntryPatternProvider m_provider;
			public LogEntry(ILogEntryPatternProvider provider, long fpos, string text, bool from_file)
			{
				m_provider = provider;
				FPos = fpos;
				Text = text;
				FromFile = from_file;
			}

			/// <summary>Log file offset (byte index of the log entry delimiter). Used for sorting</summary>
			public long FPos { get; }

			/// <summary>The text for the log entry</summary>
			public string Text { get; }

			/// <summary>True if this log entry was read from a log file</summary>
			public bool FromFile { get; }

			/// <summary>Map fields to matches in 'Text'</summary>
			public string Tag => Read<string?>(nameof(Tag), x => x) ?? string.Empty;

			/// <summary>The log entry importance level</summary>
			public ELogLevel Level => Read(nameof(Level), x => Enum<ELogLevel>.Parse(x));

			/// <summary>The time since the log was started</summary>
			public TimeSpan Elapsed => Read(nameof(Elapsed), x => TimeSpan_.TryParse(x)) ?? TimeSpan.Zero;
			public string ElapsedFormatted => Elapsed.ToStringOptional(m_provider.ElapsedFormat ?? @"[d\.\ ][hh\:][mm\:]ss\.fff");

			/// <summary>The time point of the log entry</summary>
			public DateTimeOffset Timestamp => Read(nameof(Timestamp), x => DateTimeOffset_.TryParse(x)) ?? DateTimeOffset.MinValue;
			public string TimestampFormatted => Timestamp.ToString(m_provider.TimestampFormat ?? "yyyy-MM-dd HH:mm:ss.fff");

			/// <summary>The log entry message</summary>
			public string Message => Read(nameof(Message), x => x) ?? string.Empty;

			/// <summary>The file that was the source of the log entry</summary>
			public string File => Read(nameof(File), x => x) ?? string.Empty;

			/// <summary>The line in the file that was the source of the log entry</summary>
			public int Line => Read(nameof(Line), int.Parse);

			/// <summary>The byte offset into the file that was the source of the log entry</summary>
			public int Offset => Read(nameof(Offset), int.Parse);

			/// <summary>The number of repeat log entries of the same type</summary>
			public int Occurrences => Read(nameof(Occurrences), int.Parse);

			/// <summary>Lazy regex pattern match</summary>
			private T Read<T>(string grp, Func<string, T> parse)
			{
				m_match ??= m_provider.LogEntryPattern?.Match(Text);
				if (m_match != null && m_match.Success)
				{
					try
					{
						var value = m_match.Groups[grp]?.Value ?? string.Empty;
						return value.Length != 0 ? parse(value) : default!;
					}
					catch { }
				}
				return default!;
			}
			private Match? m_match;

			/// <summary>The highlighting pattern suitable for this log entry (or null)</summary>
			public HLPattern? Highlight
			{
				get
				{
					m_highlight ??= m_provider.Highlighting.FirstOrDefault(x => x.IsMatch(Text));
					m_highlight ??= Level switch
					{
						ELogLevel.Fatal => new HLPattern(Color_.From("#FCF"), Color_.From("#C0C")),
						ELogLevel.Error => new HLPattern(Color_.From("#FDD"), Color_.From("#F00")),
						ELogLevel.Warn => new HLPattern(Colors.Transparent, Color_.From("#E70")),
						ELogLevel.Info => new HLPattern(Colors.Transparent, Colors.Black),
						ELogLevel.Debug => new HLPattern(Colors.Transparent, Color_.From("#888")),
						ELogLevel.NoLevel => new HLPattern(Colors.Transparent, Colors.Black),
						_ => throw new Exception($"Unknown log level for highlighting"),
					};
					return m_highlight;
				}
				set
				{
					m_highlight = value;
				}
			}
			private HLPattern? m_highlight;
		}

		/// <summary>Patterns for highlighting rows in the log</summary>
		[DebuggerDisplay("{Expr}")]
		public class HLPattern : Pattern
		{
			public HLPattern(Color bk, Color fr)
				: base()
			{
				BackColour = bk;
				ForeColour = fr;
			}
			public HLPattern(Color bk, Color fr, EPattern patn_type, string expr)
				: base(patn_type, expr)
			{
				BackColour = bk;
				ForeColour = fr;
			}
			public HLPattern(HLPattern rhs)
				: base(rhs)
			{
				BackColour = rhs.BackColour;
				ForeColour = rhs.ForeColour;
			}
			public HLPattern(XElement node)
				: base(node)
			{
				BackColour = node.Element(nameof(BackColour)).As<Color>();
				ForeColour = node.Element(nameof(ForeColour)).As<Color>();
			}
			public override XElement ToXml(XElement node)
			{
				base.ToXml(node);
				node.Add2(nameof(BackColour), BackColour, false);
				node.Add2(nameof(ForeColour), ForeColour, false);
				return node;
			}

			/// <summary>Background colour when the pattern is a match</summary>
			public Color BackColour { get; set; }

			/// <summary>Foreground colour when the pattern is a match</summary>
			public Color ForeColour { get; set; }
		}

		/// <summary>Typical column names</summary>
		public static class ColumnNames
		{
			public const string Tag         = nameof(LogEntry.Tag);
			public const string Level       = nameof(LogEntry.Level);
			public const string Timestamp   = nameof(LogEntry.Timestamp);
			public const string Elapsed     = nameof(LogEntry.Elapsed);
			public const string Message     = nameof(LogEntry.Message);
			public const string File        = nameof(LogEntry.File);
			public const string Line        = nameof(LogEntry.Line);
			public const string Offset      = nameof(LogEntry.Offset);
			public const string Occurrences = nameof(LogEntry.Occurrences);
			public const string Text        = nameof(LogEntry.Text);
		}
	}
}
