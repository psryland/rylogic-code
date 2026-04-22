using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Input;
using System.Windows.Media;
using EnvDTE;
using Microsoft.VisualStudio.Shell;
using Rylogic.LDrawVisualiser.Core;

namespace Rylogic.LDrawVisualiser
{
	public partial class VisualizerToolWindowControl : UserControl
	{
		private LDrawVisualiserPackage? m_package;
		private ScriptProjectManager? m_project_manager;
		private readonly ScriptCompiler m_compiler = new();
		private readonly LDrawStreamer m_streamer = new();
		private ScriptInfo? m_active_script;
		private DateTime m_last_send_time;

		public VisualizerToolWindowControl()
		{
			InitializeComponent();
		}

		/// <summary>Initialize with the VS package (provides DTE access)</summary>
		internal void Initialize(LDrawVisualiserPackage package)
		{
			m_package = package;
			m_project_manager = new ScriptProjectManager(package.Options);
			m_auto_refresh_check.IsChecked = package.Options.DefaultAutoRefresh;

			// Ensure the scripts directory exists and populate the list
			m_project_manager.EnsureDirectoryExists();
			RefreshScriptsList();

			// Select the last used script
			var last = package.Options.LastSelectedScript;
			var match = m_scripts_list.Items.Cast<ScriptInfo>().FirstOrDefault(s => s.Name == last);
			if (match != null)
				m_scripts_list.SelectedItem = match;
			else if (m_scripts_list.Items.Count > 0)
				m_scripts_list.SelectedIndex = 0;
		}

		/// <summary>Clean up resources</summary>
		internal void Shutdown()
		{
			PersistState();
			m_streamer.Dispose();
		}

		/// <summary>Called when the debugger enters break mode</summary>
		internal void OnEnterBreakMode()
		{
			if (m_auto_refresh_check.IsChecked == true && m_active_script != null)
			{
				_ = ThreadHelper.JoinableTaskFactory.RunAsync(async () =>
				{
					await ThreadHelper.JoinableTaskFactory.SwitchToMainThreadAsync();
					RunActiveScript();
				});
			}
		}

		/// <summary>Called when a script .csx file is saved in VS. Auto-compiles and runs if it's the active script.</summary>
		internal void OnScriptSaved(string filepath)
		{
			if (m_active_script == null) return;
			if (!string.Equals(filepath, m_active_script.FilePath, StringComparison.OrdinalIgnoreCase)) return;

			RunActiveScript();
		}

		// -- Script management ----------------------------------------------------

		private void RefreshScriptsList()
		{
			if (m_project_manager == null) return;

			var selected_name = m_active_script?.Name;
			var scripts = m_project_manager.GetScripts();

			m_scripts_list.Items.Clear();
			foreach (var script in scripts)
				m_scripts_list.Items.Add(script);

			// Restore selection
			if (selected_name != null)
			{
				var match = m_scripts_list.Items.Cast<ScriptInfo>().FirstOrDefault(s => s.Name == selected_name);
				if (match != null)
					m_scripts_list.SelectedItem = match;
			}
		}

		private void OnNewScript(object sender, RoutedEventArgs e)
		{
			if (m_project_manager == null) return;

			// Generate a unique name
			var base_name = "Script";
			var name = base_name;
			var counter = 1;
			var existing = m_project_manager.GetScripts().Select(s => s.Name).ToHashSet(StringComparer.OrdinalIgnoreCase);
			while (existing.Contains(name))
				name = $"{base_name}{++counter}";

			var script = m_project_manager.CreateScript(name);
			RefreshScriptsList();

			// Select and open the new script
			var match = m_scripts_list.Items.Cast<ScriptInfo>().FirstOrDefault(s => s.Name == script.Name);
			if (match != null)
			{
				m_scripts_list.SelectedItem = match;
				OpenScriptInEditor(match);
			}
		}

		private void OnScriptDoubleClick(object sender, MouseButtonEventArgs e)
		{
			if (m_scripts_list.SelectedItem is ScriptInfo script)
				OpenScriptInEditor(script);
		}

		private void OnScriptSelectionChanged(object sender, SelectionChangedEventArgs e)
		{
			if (m_scripts_list.SelectedItem is ScriptInfo script)
			{
				m_active_script = script;
				SetStatus($"Selected: {script.Name}");
			}
		}

		private void OnRunScript(object sender, RoutedEventArgs e)
		{
			if (sender is Button btn && btn.Tag is ScriptInfo script)
			{
				m_scripts_list.SelectedItem = script;
				m_active_script = script;
				RunActiveScript();
			}
		}

		private void OnRenameScript(object sender, RoutedEventArgs e)
		{
			if (sender is not Button btn || btn.Tag is not ScriptInfo script) return;
			if (m_project_manager == null) return;

			var dlg = new RenameScriptDialog(script.Name);
			if (dlg.ShowDialog() != true) return;

			var new_name = dlg.NewName;
			if (string.IsNullOrWhiteSpace(new_name) || new_name == script.Name) return;

			// Validate the name contains no invalid filename characters
			if (new_name.IndexOfAny(Path.GetInvalidFileNameChars()) >= 0)
			{
				MessageBox.Show("The name contains invalid characters.", "Rename Script", MessageBoxButton.OK, MessageBoxImage.Warning);
				return;
			}

			try
			{
				var renamed = m_project_manager.RenameScript(script, new_name);
				if (m_active_script?.Name == script.Name)
					m_active_script = renamed;

				RefreshScriptsList();
				SetStatus($"Renamed to: {renamed.Name}");
			}
			catch (IOException ex)
			{
				MessageBox.Show(ex.Message, "Rename Script", MessageBoxButton.OK, MessageBoxImage.Warning);
			}
		}

		private void OnRevealScript(object sender, RoutedEventArgs e)
		{
			if (sender is not Button btn || btn.Tag is not ScriptInfo script) return;

			try
			{
				System.Diagnostics.Process.Start("explorer.exe", $"/select,\"{script.FilePath}\"");
			}
			catch (Exception ex)
			{
				SetStatus($"Failed to reveal: {ex.Message}");
			}
		}

		private void OnDeleteScript(object sender, RoutedEventArgs e)
		{
			if (sender is not Button btn || btn.Tag is not ScriptInfo script) return;
			if (m_project_manager == null) return;

			var result = MessageBox.Show($"Delete script ''{script.Name}''?", "Confirm Delete", MessageBoxButton.YesNo, MessageBoxImage.Question);
			if (result != MessageBoxResult.Yes) return;

			m_project_manager.DeleteScript(script);
			if (m_active_script?.Name == script.Name)
				m_active_script = null;

			RefreshScriptsList();
		}

		/// <summary>Open a script file in the VS editor for editing</summary>
		private void OpenScriptInEditor(ScriptInfo script)
		{
			ThreadHelper.ThrowIfNotOnUIThread();
			if (m_package?.Dte == null) return;

			try
			{
				m_package.Dte.ItemOperations.OpenFile(script.FilePath, Constants.vsViewKindTextView);
				SetStatus($"Opened: {script.Name}");
			}
			catch (Exception ex)
			{
				SetStatus($"Failed to open: {ex.Message}");
			}
		}

		private void PersistState()
		{
			if (m_package == null || m_active_script == null) return;
			m_package.Options.LastSelectedScript = m_active_script.Name;
			m_package.Options.SaveSettingsToStorage();
		}

		// -- Connection -----------------------------------------------------------

		/// <summary>Save any open .csx documents in the scripts directory before running</summary>
		private void SaveOpenScriptDocuments()
		{
			ThreadHelper.ThrowIfNotOnUIThread();
			if (m_package?.Dte == null) return;

			var scripts_dir = m_package.Options.ScriptsDirectory;
			foreach (Document doc in m_package.Dte.Documents)
			{
				if (doc.Saved) continue;
				if (doc.FullName == null) continue;
				if (!doc.FullName.EndsWith(".csx", StringComparison.OrdinalIgnoreCase)) continue;
				if (!doc.FullName.StartsWith(scripts_dir, StringComparison.OrdinalIgnoreCase)) continue;

				try { doc.Save(); }
				catch { /* Ignore save failures for individual documents */ }
			}
		}

		private void OnConnectClick(object sender, RoutedEventArgs e)
		{
			if (m_streamer.State == EConnectionState.Connected)
			{
				m_streamer.Disconnect();
			}
			else
			{
				var address = m_package?.Options.DefaultAddress ?? "localhost:1976";
				var (host, port) = LDrawStreamer.ParseAddress(address);
				m_streamer.Host = host;
				m_streamer.Port = port;
				m_streamer.Connect();
			}
			UpdateConnectionUI();
			SaveOpenScriptDocuments();
			RunActiveScript();
		}

		private void OnRefreshClick(object sender, RoutedEventArgs e)
		{
			SaveOpenScriptDocuments();
			RunActiveScript();
		}

		// -- Compile & Run --------------------------------------------------------

		/// <summary>Read the active script from disk, compile, evaluate, and send to LDraw</summary>
		private void RunActiveScript()
		{
			ThreadHelper.ThrowIfNotOnUIThread();

			if (m_active_script == null)
			{
				SetStatus("No script selected");
				return;
			}

			if (m_project_manager == null || m_package == null)
			{
				SetStatus("Error: not initialized");
				return;
			}

			// Read the script body and using directives from the .csx file
			var script_body = m_project_manager.ReadScriptBody(m_active_script);
			var extra_usings = m_project_manager.ReadUsingDirectives(m_active_script);

			// Compile
			var compiled = m_compiler.Compile(script_body, extra_usings);
			UpdateErrorPanel();
			if (!compiled)
			{
				SetStatus("Compilation failed");
				return;
			}

			if (m_compiler.CompiledScript == null)
			{
				SetStatus("No compiled script");
				return;
			}

			// Evaluate — use DebugProxy in break mode, NullProxy otherwise
			string ldraw_script;
			try
			{
				var debugger = m_package.Dte?.Debugger;
				var in_break_mode = debugger?.CurrentMode == dbgDebugMode.dbgBreakMode;
				dynamic vars = in_break_mode && debugger != null
					? new DebugProxy(debugger)
					: NullProxy.Instance;

				ldraw_script = m_compiler.CompiledScript(vars);
			}
			catch (Exception ex)
			{
				var script_path = m_active_script?.FilePath ?? "<unknown>";
				ShowErrors(new[] { $"{script_path}(0,0): error: {ex.Message}" });
				SetStatus("Runtime error");
				return;
			}

			// Clear errors on success
			ShowErrors(Array.Empty<string>());

			// Send
			if (m_streamer.State != EConnectionState.Connected)
			{
				SetStatus("Not connected to LDraw");
				return;
			}

			if (m_streamer.Send(ldraw_script))
			{
				m_last_send_time = DateTime.Now;
				SetStatus($"Sent {ldraw_script.Length} chars at {m_last_send_time:HH:mm:ss}");
			}
			else
			{
				SetStatus($"Send failed: {m_streamer.LastError}");
				UpdateConnectionUI();
			}
		}

		// -- UI helpers -----------------------------------------------------------

		private void UpdateErrorPanel()
		{
			var script_path = m_active_script?.FilePath ?? "<unknown>";
			var errors = new List<string>();
			foreach (var diag in m_compiler.Diagnostics)
			{
				if (diag.Severity >= Microsoft.CodeAnalysis.DiagnosticSeverity.Warning)
					errors.Add($"{script_path}{diag.GetMessage()}");
			}

			ShowErrors(errors);
		}

		/// <summary>Show error messages in the error panel, or hide it when empty</summary>
		private void ShowErrors(IReadOnlyList<string> errors)
		{
			if (errors.Count > 0)
			{
				m_error_text.Text = string.Join("\n", errors);
				m_error_panel.Visibility = Visibility.Visible;
			}
			else
			{
				m_error_text.Text = string.Empty;
				m_error_panel.Visibility = Visibility.Collapsed;
			}
		}

		private void UpdateConnectionUI()
		{
			switch (m_streamer.State)
			{
				case EConnectionState.Connected:
					m_status_indicator.Fill = new SolidColorBrush(Colors.Green);
					m_status_indicator.ToolTip = "Connected";
					m_connect_btn.Content = "Disconnect";
					break;
				case EConnectionState.Error:
					m_status_indicator.Fill = new SolidColorBrush(Colors.Red);
					m_status_indicator.ToolTip = m_streamer.LastError ?? "Error";
					m_connect_btn.Content = "Connect";
					break;
				default:
					m_status_indicator.Fill = new SolidColorBrush(Colors.Gray);
					m_status_indicator.ToolTip = "Disconnected";
					m_connect_btn.Content = "Connect";
					break;
			}
		}

		private void SetStatus(string message)
		{
			m_status_text.Text = message;
		}
	}
}
