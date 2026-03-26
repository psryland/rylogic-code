using System;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Windows;
using System.Windows.Controls;
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
			m_log.LogEntries = Log.Entries;
			m_log.LogEntryPattern = Log.PatternRegex;
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
		private void HandleLogEntryDoubleClick(object? sender, LogControl.LogEntryDoubleClickEventArgs e)
		{
			var file = e.Entry.File;
			var line = e.Entry.Line;

			// No source file associated with this entry
			if (string.IsNullOrEmpty(file))
				return;

			// If an external text editor is configured, use it
			var editor_path = Model.Profile.TextEditorPath;
			if (!string.IsNullOrEmpty(editor_path))
			{
				OpenInExternalEditor(editor_path, Model.Profile.TextEditorArguments, file, line);
				return;
			}

			// Otherwise, find the script in the built-in editor and navigate to it
			var script = Model.Scripts.FirstOrDefault(x => Path_.Compare(file, x.FilePath) == 0);
			if (script == null)
				return;

			script.DockControl.IsActiveContent = true;
			script.ScrollTo(line, 0, true);
		}

		/// <summary>Launch an external text editor at the given file and line</summary>
		private static void OpenInExternalEditor(string editor_path, string arguments_pattern, string file, int line)
		{
			try
			{
				var arguments = arguments_pattern
					.Replace("{file}", file)
					.Replace("{line}", line.ToString());

				Process.Start(new ProcessStartInfo
				{
					FileName = editor_path,
					Arguments = arguments,
					UseShellExecute = false,
				});
			}
			catch (Exception ex)
			{
				Log.Write(ELogLevel.Error, ex, "Failed to launch text editor");
			}
		}
	}
}
