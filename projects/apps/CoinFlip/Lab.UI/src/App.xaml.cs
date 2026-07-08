using System;
using System.Windows;
using Rylogic.DB;
using Rylogic.Gfx;

namespace Lab.UI
{
	/// <summary>Interaction logic for App.xaml</summary>
	public partial class App : Application
	{
		public App()
		{
			// Allow view3d to fall back to the software (WARP) rasterizer when no suitable hardware
			// DX12 adapter is present (e.g. when running inside a VM). Must be set before View3d is
			// initialised. WARP is slow but has full shader-model support so the lab still renders.
			Environment.SetEnvironmentVariable("VIEW3D_ALLOW_SOFTWARE_ADAPTER", "1");

			// Load the native libraries before any window that uses them is created.
			View3d.LoadDll();
			Sqlite.LoadDll();
		}

		protected override void OnStartup(StartupEventArgs e)
		{
			base.OnStartup(e);
			var wnd = new MainWindow();
			wnd.Show();
		}
	}
}
