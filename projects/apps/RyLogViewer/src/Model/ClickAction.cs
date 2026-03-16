using System;
using System.Collections.ObjectModel;
using System.Collections.Specialized;
using System.Diagnostics;
using System.Text.RegularExpressions;
using Rylogic.Common;

namespace RyLogViewer
{
	/// <summary>An action triggered by clicking a log line that matches a pattern</summary>
	public class ClickAction : Pattern
	{
		public ClickAction()
			: this(EPattern.Substring, string.Empty)
		{ }
		public ClickAction(EPattern patn_type, string expr)
			: base(patn_type, expr)
		{
			Executable = string.Empty;
			Arguments = string.Empty;
			WorkingDirectory = string.Empty;
		}
		public ClickAction(ClickAction rhs)
			: base(rhs)
		{
			Executable = rhs.Executable;
			Arguments = rhs.Arguments;
			WorkingDirectory = rhs.WorkingDirectory;
		}

		/// <summary>Path to the executable to launch</summary>
		public string Executable { get; set; }

		/// <summary>Command line arguments (may contain {0}, {1}, etc. for capture groups)</summary>
		public string Arguments { get; set; }

		/// <summary>Working directory for the process</summary>
		public string WorkingDirectory { get; set; }

		/// <summary>Execute this action against a matched line</summary>
		public void Execute(string line_text)
		{
			if (string.IsNullOrEmpty(Executable)) return;
			if (!Active || !IsValid) return;
			if (!IsMatch(line_text)) return;

			// Expand capture groups in arguments
			var args = Arguments;
			try
			{
				var match = Regex.Match(line_text);
				if (match.Success)
				{
					for (var i = 0; i < match.Groups.Count; ++i)
						args = args.Replace($"{{{i}}}", match.Groups[i].Value);
				}
			}
			catch { }

			try
			{
				Process.Start(new ProcessStartInfo
				{
					FileName = Executable,
					Arguments = args,
					WorkingDirectory = WorkingDirectory,
					UseShellExecute = true,
				});
			}
			catch
			{
				// Swallow launch failures
			}
		}

		public override string ToString()
		{
			return $"{Expr} → {Executable}";
		}
	}

	/// <summary>Observable collection of click actions with change notification</summary>
	public class ClickActionContainer : ObservableCollection<ClickAction>
	{
		/// <summary>Raised when actions are added, removed, or modified</summary>
		public event EventHandler? ActionsChanged;

		/// <inheritdoc/>
		protected override void OnCollectionChanged(NotifyCollectionChangedEventArgs e)
		{
			base.OnCollectionChanged(e);
			ActionsChanged?.Invoke(this, EventArgs.Empty);
		}

		/// <summary>Notify that an action was edited in-place</summary>
		public void NotifyItemChanged()
		{
			ActionsChanged?.Invoke(this, EventArgs.Empty);
		}
	}
}
