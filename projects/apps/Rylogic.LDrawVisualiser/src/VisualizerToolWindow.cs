using System;
using System.Runtime.InteropServices;
using Microsoft.VisualStudio.Shell;

namespace Rylogic.LDrawVisualiser
{
	[Guid("C3D4E5F6-A1B2-7890-CDEF-123456789ABC")]
	public class VisualizerToolWindow : ToolWindowPane
	{
		private readonly VisualizerToolWindowControl m_control;

		public VisualizerToolWindow() : base(null)
		{
			Caption = "LDraw Visualiser";
			m_control = new VisualizerToolWindowControl();
			Content = m_control;
		}

		/// <summary>Called by the package when the debugger enters break mode</summary>
		internal void OnEnterBreakMode()
		{
			m_control.OnEnterBreakMode();
		}

		/// <summary>Provide the package to the control once sited</summary>
		protected override void OnCreate()
		{
			base.OnCreate();
			if (Package is LDrawVisualiserPackage pkg)
			{
				m_control.Initialize(pkg);
			}
		}

		protected override void Dispose(bool disposing)
		{
			if (disposing)
			{
				m_control.Shutdown();
			}
			base.Dispose(disposing);
		}
	}
}
