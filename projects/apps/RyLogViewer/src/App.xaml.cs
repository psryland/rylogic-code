using System;
using System.Diagnostics;
using System.IO;
using System.Net.Mail;
using System.Reflection;
using System.Text;
using System.Windows;
using Microsoft.Win32;
using Rylogic.Common;
using Rylogic.Extn;
using Rylogic.Gui.WPF;
using Rylogic.Utility;
using RyLogViewer.Options;

[assembly: ThemeInfo(ResourceDictionaryLocation.SourceAssembly, ResourceDictionaryLocation.SourceAssembly)]

namespace RyLogViewer
{
	/// <summary>Interaction logic for App.xaml</summary>
	public partial class App : Application
	{
		void ApplicationMain(object sender, StartupEventArgs args)
		{
			var unhandled = (Exception?)null;
			var startup_options = (StartupOptions?)null;
			#if !DEBUG || TRAP_UNHANDLED_EXCEPTIONS
			try
			#endif
			{
				Xml_.Config.SupportWPFTypes();
				var report = new ErrorReporter();

				// Parse command line arguments into startup options
				var err = (Exception?)null;
				try { startup_options = new StartupOptions(args.Args); }
				catch (Exception ex) { err = ex; }

				// If there was an error, display the error message
				if (err != null)
				{
					report.ErrorPopup("There is an error in the startup options provided.", err);
					//todo HelpUI.ShowDialog(null, HelpUI.EContent.Html, Application.ProductName, Resources.command_line_ref);
					Shutdown(1);
					return;
				}

				// Shouldn't be possible, but the compiler doesn't know that
				if (startup_options == null)
				{
					Shutdown(1);
					return;
				}

				// If they just want help displayed...
				if (startup_options.ShowHelp)
				{
					//todo HelpUI.ShowDialog(null, HelpUI.EContent.Html, Application.ProductName, Resources.command_line_ref);
					Shutdown(0);
					return;
				}

				// If an export path is given, run as a command line tool doing an export
				if (startup_options.ExportPath != null)
				{
					//todo Main.ExportToFile(StartupOptions);
					Shutdown(0);
					return;
				}

				// Otherwise show the app
				var settings = new Settings(startup_options.SettingsPath);
				var main = new Main(startup_options, settings, report);
				var window = new MainWindow(main, settings, report);
				window.Show();

				// Open file from command line if specified
				if (startup_options.FileToLoad != null)
				{
					try { main.LogDataSource = new SingleFileSource(startup_options.FileToLoad, settings); }
					catch (Exception ex) { report.ErrorPopup("Failed to open file.", ex); }
				}
			}
			#if !DEBUG || TRAP_UNHANDLED_EXCEPTIONS
			catch (Exception e) { unhandled = e; }
			#endif

			// Report unhandled exceptions
			if (unhandled != null)
				HandleTheUnhandled(unhandled, startup_options);
		}

		/// <summary>Handle unhandled exceptions</summary>
		private void HandleTheUnhandled(Exception ex, StartupOptions? su)
		{
			var res = MsgBox.Show(null,
				$"{Util.AppProductName} has shutdown with the following error.\r\n" +
				$"Error: {ex.GetType().Name}\r\n" +
				$"\r\n" +
				$"Deleting the applications settings file, '{su?.SettingsPath ?? Path_.FileName(Settings.Default.Filepath)}', might prevent this problem.\r\n" +
				$"\r\n" +
				$"Generating a report and sending it to {Util.GetAssemblyAttribute<AssemblyCompanyAttribute>().Company} will aid in the resolution of this issue. " +
				$"The generated report is a plain text file that you can review before sending.\r\n" +
				$"\r\n" +
				$"Would you like to generate the report?\r\n" +
				$"\r\n" +
				$"Alternatively, please contact {Constants.SupportEmail} with information about this error so that it can be fixed.\r\n" +
				$"\r\n" +
				$"Apologies for any inconvenience caused.\r\n",
				"Unexpected Termination", MsgBox.EButtons.YesNo, MsgBox.EIcon.Error);
			if (res == true)
			{
				// Prompt for a location to save the crash report
				var dg = new SaveFileDialog
				{
					Title = "Save Crash Report",
					FileName = $"{Util.AppProductName}_CrashReport",
					Filter = Util.FileDialogFilter("Crash Report Files", "*.txt", "All files", "*.*"),
					DefaultExt = "txt",
					CheckPathExists = true
				};
				if (dg.ShowDialog() == true)
				{
					// Read the contents of the settings file
					var settings = "Settings filepath unknown";
					if (su != null && Path_.FileExists(su.SettingsPath))
						settings = File.ReadAllText(su.SettingsPath);

					// Generate a string containing the report
					var sb = new StringBuilder()
						.Append(Util.AppProductName).Append(" - Crash Report - ").Append(DateTime.UtcNow).AppendLine()
						.AppendLine("---------------------------------------------------------------")
						.AppendLine("[Unhandled Exception Type]")
						.AppendLine(ex.MessageFull())
						.AppendLine()
						.AppendLine("[Settings File Contents]")
						.AppendLine(settings)
						.AppendLine()
						.AppendLine("[General]")
						.AppendLine($"Application Version: {Util.AssemblyVersion()}")
						.AppendLine(Environment.OSVersion.VersionString)
						.AppendLine(su?.Dump() ?? string.Empty)
						.AppendLine()
						.AppendLine("[Additional Comments]")
						.AppendLine("Any additional information about what you were doing when this crash occurred would be extremely helpful and appreciated");

					// Write the report
					File.WriteAllText(dg.FileName, sb.ToString());

					// Launch the text file in the default editor
					if (MsgBox.Show(null, "Preview the report before sending?", "Review Report", MsgBox.EButtons.YesNo, MsgBox.EIcon.Question) == true)
						try { Process.Start(dg.FileName); } catch { }

					try
					{
						// Try to create an email with the attachment ready to go
						var email = new MailMessage();
						email.From = new MailAddress(Constants.SupportEmail);
						email.To.Add(Constants.SupportEmail);
						email.Subject = $"{Util.AppProductName} crash report";
						email.Priority = MailPriority.Normal;
						email.IsBodyHtml = false;
						email.Body =
							$"To {Util.AppCompany},\r\n" +
							$"\r\n" +
							$"Attached is a crash report generated on {DateTime.Now}.\r\n" +
							$"A brief description of how the application was being used at the time follows:\r\n" +
							$"\r\n\r\n\r\n\r\n" +
							$"Regards,\r\n" +
							$"A Helpful User";
						email.Attachments.Add(new Attachment(dg.FileName));

						// Try to send it
						var smtp = new SmtpClient();
						smtp.Send(email);
					}
					catch { }
				}
			}
			Shutdown(1);
		}
	}
}
