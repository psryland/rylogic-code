using System;
using System.Collections.Generic;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using Microsoft.VisualStudio.Shell;
using Rylogic.LDrawVisualiser.Core;

namespace Rylogic.LDrawVisualiser
{
	public partial class VisualizerToolWindowControl : UserControl
	{
		private LDrawVisualiserPackage? m_package;
		private readonly ScriptCompiler m_compiler = new();
		private readonly LDrawStreamer m_streamer = new();
		private string? m_last_script_text;
		private DateTime m_last_send_time;

		public VisualizerToolWindowControl()
		{
			InitializeComponent();

			// Set default script text
			m_script_editor.Text = @"var b = new Builder();
// Access debug variables via 'vars', e.g.:
// b.Box(""obj1"").box(1,2,3).o2w(vars.my_object.m_o2w);
return b.ToString();";
		}

		/// <summary>Initialize with the VS package (provides DTE access)</summary>
		internal void Initialize(LDrawVisualiserPackage package)
		{
			m_package = package;
		}

		/// <summary>Clean up resources</summary>
		internal void Shutdown()
		{
			m_streamer.Dispose();
		}

		/// <summary>Called when the debugger enters break mode</summary>
		internal void OnEnterBreakMode()
		{
			if (m_auto_refresh_check.IsChecked == true)
			{
				ThreadHelper.JoinableTaskFactory.RunAsync(async () =>
				{
					await ThreadHelper.JoinableTaskFactory.SwitchToMainThreadAsync();
					EvaluateAndSend();
				});
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
				var (host, port) = LDrawStreamer.ParseAddress(m_address_box.Text);
				m_streamer.Host = host;
				m_streamer.Port = port;
				m_streamer.Connect();
			}
			UpdateConnectionUI();
		}

		private void OnRefreshClick(object sender, RoutedEventArgs e)
		{
			EvaluateAndSend();
		}

		/// <summary>Compile (if needed), evaluate the script, and send the result to LDraw</summary>
		private void EvaluateAndSend()
		{
			ThreadHelper.ThrowIfNotOnUIThread();

			if (m_package?.Dte == null)
			{
				SetStatus("Error: DTE not available");
				return;
			}

			var debugger = m_package.Dte.Debugger;
			if (debugger.CurrentMode != EnvDTE.dbgDebugMode.dbgBreakMode)
			{
				SetStatus("Not in break mode");
				return;
			}

			var script_text = m_script_editor.Text;

			// Compile only if script text has changed
			if (script_text != m_last_script_text)
			{
				var compiled = m_compiler.Compile(script_text);
				UpdateErrorPanel();

				if (!compiled)
				{
					SetStatus("Compilation failed");
					return;
				}
				m_last_script_text = script_text;
			}

			if (m_compiler.CompiledScript == null)
			{
				SetStatus("No compiled script");
				return;
			}

			// Evaluate with fresh debug proxy
			string ldraw_script;
			try
			{
				var vars = new DebugProxy(debugger);
				ldraw_script = m_compiler.CompiledScript(vars);
			}
			catch (Exception ex)
			{
				SetStatus($"Runtime error: {ex.Message}");
				return;
			}

			// Send to LDraw
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

		private void UpdateErrorPanel()
		{
			var errors = new List<string>();
			foreach (var diag in m_compiler.Diagnostics)
			{
				if (diag.Severity >= Microsoft.CodeAnalysis.DiagnosticSeverity.Warning)
					errors.Add(diag.GetMessage());
			}

			if (errors.Count > 0)
			{
				m_error_list.ItemsSource = errors;
				m_error_panel.Visibility = Visibility.Visible;
			}
			else
			{
				m_error_list.ItemsSource = null;
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
