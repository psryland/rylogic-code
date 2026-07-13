using System;
using System.IO;
using System.Linq;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using LDraw.UI;
using Rylogic.Common;
using Rylogic.Gui.WPF;
using Rylogic.Utility;

namespace LDraw
{
	public sealed partial class LogUI :Grid, IDockable, IDisposable
	{
		public LogUI(Model model)
		{
			InitializeComponent();
			DockControl.Owner = this;
			Model = model;
			GoToError = Command.Create(this, GoToErrorInternal, GoToErrorAvailable);
			m_log.LogEntries = Log.Entries;
			m_log.LogEntryPattern = Log.PatternRegex;
			m_log.GoToLogEntry = GoToError;
			m_log.PopOutOnNewMessages = false;
			m_log.FilterLevel = ELogLevel.Debug;
			m_log.LogEntryDoubleClick += HandleLogEntryDoubleClick;

			// Hide all columns except the Message column initially
			foreach (var col in m_log.Columns)
				col.Visibility = (col.Header is string header && header == LogControl.ColumnNames.Message) ? Visibility.Visible : Visibility.Collapsed;
		}
		public void Dispose()
		{
			Model = null!;
			m_log.LogEntryDoubleClick -= HandleLogEntryDoubleClick;
			m_log.GoToLogEntry = null;
			m_log.Dispose();
		}

		/// <summary></summary>
		public DockControl DockControl => m_log.DockControl;

		/// <summary>App logic</summary>
		public Model Model
		{
			get;
			private set
			{
				if (field == value) return;
				if (field != null)
				{
				}
				field = value;
				if (field != null)
				{
				}
			}
		} = null!;

		/// <summary>Handle a log entry being double clicked in the log view</summary>
		private async void HandleLogEntryDoubleClick(object? sender, LogControl.LogEntryDoubleClickEventArgs e)
		{
			await NavigateToErrorAsync(e.Entry);
		}

		/// <summary>Navigate to the selected log entry's source location</summary>
		public Command GoToError { get; private set; } = null!;
		private bool GoToErrorAvailable()
		{
			return CanGoToError(m_log.CurrentLogEntry);
		}
		private async void GoToErrorInternal()
		{
			if (m_log.CurrentLogEntry is LogControl.LogEntry entry)
				await NavigateToErrorAsync(entry);
		}
		private bool CanGoToError(LogControl.LogEntry? entry)
		{
			if (entry == null || entry.File.Length == 0 || entry.Line <= 0 || !Path_.FileExists(entry.File))
				return false;

			if (!string.IsNullOrEmpty(Model.Profile.TextEditorPath))
				return true;

			return
				Model.Scripts.Any(x => Path_.Compare(entry.File, x.FilePath) == 0) ||
				Model.Sources.Any(x => Path_.Compare(entry.File, x.FilePath) == 0 && x.CanEdit);
		}
		private async Task NavigateToErrorAsync(LogControl.LogEntry entry)
		{
			try
			{
				if (!CanGoToError(entry))
					return;

				// If an external text editor is configured, use it
				var file = entry.File;
				var line = Math.Max(1, entry.Line);
				var editor_path = Model.Profile.TextEditorPath;
				if (!string.IsNullOrEmpty(editor_path))
				{
					ExternalTextEditor.Launch(editor_path, Model.Profile.TextEditorArguments, file, line);
					return;
				}

				// Otherwise, find/show the script in the built-in editor and navigate to it
				var script = OpenInBuiltInEditor(file);
				if (script == null)
					return;

				await script.ScrollToAsync(line, 1, true);
				script.Editor.Focus();
			}
			catch (Exception ex)
			{
				Log.Write(ELogLevel.Error, ex, "Failed to go to error");
			}
		}
		private ScriptUI? OpenInBuiltInEditor(string file)
		{
			var script = Model.Scripts.FirstOrDefault(x => Path_.Compare(file, x.FilePath) == 0);
			var source = script?.Source ?? Model.Sources.FirstOrDefault(x => Path_.Compare(file, x.FilePath) == 0 && x.CanEdit);
			if (source != null)
				script = Model.OpenInEditor(source);

			if (script == null)
				return null;

			var dock_container = DockControl.DockContainer;
			if (dock_container == null)
			{
				script.DockControl.IsActiveContent = true;
				return script;
			}

			if (script.DockControl.DockContainer != dock_container)
				dock_container.Add(script, EDockSite.Left).IsFloating = true;

			dock_container.FindAndShow(script.DockControl);
			return script;
		}
	}
}
