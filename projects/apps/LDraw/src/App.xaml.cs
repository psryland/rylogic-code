using System;
using System.Diagnostics;
using System.Windows;
using Rylogic.Extn;
using Rylogic.Gfx;
using Rylogic.Gui.WPF;
using Rylogic.Maths;
using Rylogic.Utility;
using Rylogic.Windows.Extn;

namespace LDraw
{
	public partial class App :Application
	{
		static App()
		{
			//WPFUtil.WaitForDebugger();
		}
		protected override void OnStartup(StartupEventArgs e)
		{
			base.OnStartup(e);
			ShutdownMode = ShutdownMode.OnExplicitShutdown;

			try
			{
				View3d.LoadDll();
				Xml_.Config
					.SupportRylogicMathsTypes()
					.SupportRylogicGraphicsTypes()
					.SupportSystemDrawingPrimitiveTypes()
					.SupportSystemDrawingCommonTypes()
					.SupportWPFTypes()
					;

				var options = new StartupOptions(e.Args);

				// If the settings fail to load, the caller might want to hand tweak them
				var settings = LoadSettings(options.SettingsPath);
				if (settings == null)
				{
					Shutdown();
					return;
				}

				ShutdownMode = ShutdownMode.OnMainWindowClose;
				MainWindow = new MainWindow(new Model(options, settings));
				MainWindow.Show();
			}
			catch (Exception ex)
			{
				if (Debugger.IsAttached) throw;
				Log.Write(ELogLevel.Fatal, ex, Util.AppProductName);
				MsgBox.Show(null, $"Application startup failure: {ex.MessageFull()}", Util.AppProductName, MsgBox.EButtons.OK, MsgBox.EIcon.Error);
				Shutdown();
			}
		}

		/// <summary>Load settings with fall back to defaults on error</summary>
		private SettingsData? LoadSettings(string settings_path)
		{
			try
			{
				return new SettingsData(settings_path);
			}
			catch (Exception ex)
			{
				Log.Write(ELogLevel.Warn, ex, "Failed to load application settings. Defaults used");
				var dlg = new MsgBox(null, $"Failed to load application settings.\n{ex.MessageFull()}.", Util.AppProductName, MsgBox.EButtons.OKCancel, MsgBox.EIcon.Information);
				dlg.ShowInTaskbar = true;
				dlg.WindowStartupLocation = WindowStartupLocation.CenterScreen;
				dlg.OkText = new LocaleString("UseDefaultSettings", "Use Default Settings");
				if (dlg.ShowDialog() != true)
					return null;

				return new SettingsData() { Filepath = settings_path };
			}
		}
	}
}
