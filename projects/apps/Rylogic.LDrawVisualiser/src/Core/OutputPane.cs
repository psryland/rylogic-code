namespace Rylogic.LDrawVisualiser.Core
{
	using System;
	using Microsoft.VisualStudio;
	using Microsoft.VisualStudio.Shell;
	using Microsoft.VisualStudio.Shell.Interop;

	/// <summary>
	/// Lazily creates and writes to a dedicated "LDraw Visualiser" pane in the VS Output window.
	/// All access must be on the UI thread (callers are responsible — DebugProxy already
	/// enforces this via ThreadHelper.ThrowIfNotOnUIThread).
	/// </summary>
	internal static class OutputPane
	{
		// A stable GUID identifies our pane so VS reuses the same pane across sessions
		// instead of creating a new one each time the package loads.
		private static readonly Guid s_pane_guid = new("F2A1C7B8-3D4E-4A5F-9C8B-7E6D5F4A3B2C");
		private const string PaneTitle = "LDraw Visualiser";

		private static IVsOutputWindowPane? s_pane;

		/// <summary>Append text to the pane (no trailing newline)</summary>
		public static void Write(string text)
		{
			ThreadHelper.ThrowIfNotOnUIThread();
			var pane = EnsurePane();
			pane?.OutputStringThreadSafe(text);
		}

		/// <summary>Append text followed by a newline</summary>
		public static void WriteLine(string text)
		{
			Write(text + Environment.NewLine);
		}

		/// <summary>Bring the pane to the front (called on first write so users see output appear)</summary>
		public static void Activate()
		{
			ThreadHelper.ThrowIfNotOnUIThread();
			EnsurePane()?.Activate();
		}

		// Acquire the pane, creating it on first use. Returns null if VS isn't reachable.
		private static IVsOutputWindowPane? EnsurePane()
		{
			ThreadHelper.ThrowIfNotOnUIThread();
			if (s_pane != null)
				return s_pane;

			if (Package.GetGlobalService(typeof(SVsOutputWindow)) is not IVsOutputWindow output)
				return null;

			// Try to fetch an existing pane (survives extension reload during a VS session).
			var pane_guid = s_pane_guid;
			if (output.GetPane(ref pane_guid, out var existing) == VSConstants.S_OK && existing != null)
			{
				s_pane = existing;
				return s_pane;
			}

			// Otherwise create it. visible=1 + clearWithSolution=0 keeps the pane around
			// across solution switches so logs aren't lost.
			if (output.CreatePane(ref pane_guid, PaneTitle, 1, 0) != VSConstants.S_OK)
				return null;

			output.GetPane(ref pane_guid, out s_pane);
			s_pane?.Activate();
			return s_pane;
		}
	}
}
