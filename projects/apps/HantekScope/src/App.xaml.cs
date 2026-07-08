using System;
using System.Windows;
using Rylogic.Gfx;

namespace HantekScope
{
	/// <summary>Application entry point and global setup.</summary>
	public partial class App :Application
	{
		static App()
		{
			// Disable WPF's real-time stylus thread which posts synthetic WM_MOUSEMOVE
			// messages that bypass Windows' normal mouse-move coalescing (matches LDraw).
			AppContext.SetSwitch("Switch.System.Windows.Input.Stylus.DisableStylusAndTouchSupport", true);
		}

		protected override void OnStartup(StartupEventArgs e)
		{
			base.OnStartup(e);

			// The View3D native engine backs the chart; load it before any chart is created.
			View3d.LoadDll();

			var main = new UI.MainWindow();
			main.Show();
		}
	}
}
