using System;
using System.ComponentModel.Design;
using System.Runtime.InteropServices;
using System.Threading;
using System.Threading.Tasks;
using EnvDTE;
using EnvDTE80;
using Microsoft.VisualStudio.Shell;
using Microsoft.VisualStudio.Shell.Interop;

[assembly: ComVisible(false)]

namespace Rylogic.LDrawVisualiser
{
	[Guid(PackageGuidString)]
	[PackageRegistration(UseManagedResourcesOnly = true, AllowsBackgroundLoading = true)]
	[InstalledProductRegistration("Rylogic LDraw Visualiser", "LDraw debug visualiser for Visual Studio", "1.0.0")]
	[ProvideMenuResource("Menus.ctmenu", 1)]
	[ProvideToolWindow(typeof(VisualizerToolWindow), Style = VsDockStyle.Tabbed, Window = "3ae79031-e1bc-11d0-8f78-00a0c9110057")]
	[ProvideOptionPage(typeof(LDrawVisualiserOptions), "Rylogic", "LDraw Visualiser Options", 0, 0, true)]
	[ProvideBindingPath]
	public sealed class LDrawVisualiserPackage : AsyncPackage
	{
		public const string PackageGuidString = "B8E3A1F2-7D4C-4A5B-9E6F-1C2D3E4F5A6B";

		/// <summary>The VS automation object</summary>
		public DTE2? Dte { get; private set; }

		/// <summary>Extension options</summary>
		public LDrawVisualiserOptions Options => (LDrawVisualiserOptions)GetDialogPage(typeof(LDrawVisualiserOptions));

		/// <summary>Debug event hooks</summary>
		private DebuggerEvents? m_debugger_events;

		/// <summary>The tool window instance (if open)</summary>
		internal VisualizerToolWindow? ToolWindow => FindToolWindow(typeof(VisualizerToolWindow), 0, false) as VisualizerToolWindow;

		protected override async Task InitializeAsync(CancellationToken cancellation_token, IProgress<ServiceProgressData> progress)
		{
			await base.InitializeAsync(cancellation_token, progress);
			await JoinableTaskFactory.SwitchToMainThreadAsync(cancellation_token);

			// Get the DTE automation object
			Dte = await GetServiceAsync(typeof(DTE)) as DTE2;

			// Subscribe to debugger break events
			if (Dte != null)
			{
				m_debugger_events = Dte.Events.DebuggerEvents;
				m_debugger_events.OnEnterBreakMode += OnEnterBreakMode;
			}

			// Register the menu command for showing the tool window
			if (await GetServiceAsync(typeof(IMenuCommandService)) is OleMenuCommandService mcs)
			{
				var cmd_id = new CommandID(GuidList.CommandSet, CommandIds.ShowVisualiser);
				mcs.AddCommand(new MenuCommand(ShowToolWindow, cmd_id));
			}
		}

		protected override void Dispose(bool disposing)
		{
			if (disposing && m_debugger_events != null)
			{
				Microsoft.VisualStudio.Shell.ThreadHelper.ThrowIfNotOnUIThread();
				m_debugger_events.OnEnterBreakMode -= OnEnterBreakMode;
			}
			base.Dispose(disposing);
		}

		private void ShowToolWindow(object? sender, EventArgs e)
		{
			JoinableTaskFactory.RunAsync(async () =>
			{
				var window = await ShowToolWindowAsync(typeof(VisualizerToolWindow), 0, true, DisposalToken);
				if (window?.Frame is IVsWindowFrame frame)
					Microsoft.VisualStudio.ErrorHandler.ThrowOnFailure(frame.Show());
			});
		}

		private void OnEnterBreakMode(dbgEventReason reason, ref dbgExecutionAction action)
		{
			ToolWindow?.OnEnterBreakMode();
		}
	}

	static class CommandIds
	{
		public const int ShowVisualiser = 0x0100;
	}

	static class GuidList
	{
		private const string GuidLDrawVisualiserCmdSetString = "A1B2C3D4-E5F6-7890-ABCD-EF1234567890";
		public static readonly Guid CommandSet = new(GuidLDrawVisualiserCmdSetString);
	}
}
