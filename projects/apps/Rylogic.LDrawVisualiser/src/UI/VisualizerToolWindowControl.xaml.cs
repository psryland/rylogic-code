using System;
using System.Collections.Generic;
using System.Linq;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using ICSharpCode.AvalonEdit.CodeCompletion;
using Microsoft.VisualStudio.PlatformUI;
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
		private CompletionWindow? m_completion_window;

		public VisualizerToolWindowControl()
		{
			InitializeComponent();

			// Set default script text
			m_script_editor.Text = "";

			// Apply VS theme to AvalonEdit
			ApplyThemeToEditor();
			VSColorTheme.ThemeChanged += _ => ApplyThemeToEditor();

			// Hook up autocomplete
			m_script_editor.TextArea.TextEntered += OnTextEntered;

			// Hook up hover tooltips for debug variable evaluation
			m_script_editor.TextArea.TextView.MouseHover += OnMouseHover;
			m_script_editor.TextArea.TextView.MouseHoverStopped += OnMouseHoverStopped;
		}

		private ToolTip? m_hover_tooltip;

		/// <summary>Apply VS theme colours to the AvalonEdit editor (which doesn't use dynamic resources)</summary>
		private void ApplyThemeToEditor()
		{
			var bg = VSColorTheme.GetThemedColor(EnvironmentColors.ToolWindowBackgroundColorKey);
			var fg = VSColorTheme.GetThemedColor(EnvironmentColors.ToolWindowTextColorKey);
			m_script_editor.Background = new SolidColorBrush(Color.FromRgb(bg.R, bg.G, bg.B));
			m_script_editor.Foreground = new SolidColorBrush(Color.FromRgb(fg.R, fg.G, fg.B));

			// Line number colour
			m_script_editor.LineNumbersForeground = new SolidColorBrush(Color.FromArgb(128, fg.R, fg.G, fg.B));

			// Detect dark theme (luminance < 0.5)
			var is_dark = (0.299 * bg.R + 0.587 * bg.G + 0.114 * bg.B) / 255.0 < 0.5;

			// Apply theme-appropriate syntax highlighting colours
			var highlighting = m_script_editor.SyntaxHighlighting;
			if (highlighting != null)
			{
				foreach (var colour in highlighting.NamedHighlightingColors)
				{
					switch (colour.Name)
					{
						case "Comment":
							colour.Foreground = new ICSharpCode.AvalonEdit.Highlighting.SimpleHighlightingBrush(
								is_dark ? Color.FromRgb(0x6A, 0x99, 0x55) : Color.FromRgb(0x00, 0x80, 0x00)); // green
							break;
						case "String":
							colour.Foreground = new ICSharpCode.AvalonEdit.Highlighting.SimpleHighlightingBrush(
								is_dark ? Color.FromRgb(0xCE, 0x91, 0x78) : Color.FromRgb(0xA3, 0x15, 0x15)); // orange / dark red
							break;
						case "Char":
							colour.Foreground = new ICSharpCode.AvalonEdit.Highlighting.SimpleHighlightingBrush(
								is_dark ? Color.FromRgb(0xCE, 0x91, 0x78) : Color.FromRgb(0xA3, 0x15, 0x15));
							break;
						case "Preprocessor":
							colour.Foreground = new ICSharpCode.AvalonEdit.Highlighting.SimpleHighlightingBrush(
								is_dark ? Color.FromRgb(0xBD, 0x63, 0xC5) : Color.FromRgb(0x80, 0x00, 0x80)); // purple
							break;
						case "Punctuation":
							colour.Foreground = new ICSharpCode.AvalonEdit.Highlighting.SimpleHighlightingBrush(
								is_dark ? Color.FromRgb(0xDC, 0xDC, 0xDC) : Color.FromRgb(0x00, 0x00, 0x00)); // light grey / black
							break;
						case "ValueTypeKeywords":
						case "ReferenceTypeKeywords":
						case "ThisOrBaseReference":
						case "NullOrValueKeywords":
						case "Keywords":
						case "GotoKeywords":
						case "ContextKeywords":
						case "ExceptionKeywords":
						case "CheckedKeyword":
						case "UnsafeKeywords":
						case "OperatorKeywords":
						case "ParameterModifiers":
						case "Modifiers":
						case "Visibility":
						case "NamespaceKeywords":
						case "GetSetAddRemove":
						case "TrueFalse":
						case "TypeKeywords":
							colour.Foreground = new ICSharpCode.AvalonEdit.Highlighting.SimpleHighlightingBrush(
								is_dark ? Color.FromRgb(0x56, 0x9C, 0xD6) : Color.FromRgb(0x00, 0x00, 0xFF)); // blue
							break;
						case "NumberLiteral":
							colour.Foreground = new ICSharpCode.AvalonEdit.Highlighting.SimpleHighlightingBrush(
								is_dark ? Color.FromRgb(0xB5, 0xCE, 0xA8) : Color.FromRgb(0x09, 0x86, 0x58)); // light green / teal
							break;
						case "MethodCall":
							colour.Foreground = new ICSharpCode.AvalonEdit.Highlighting.SimpleHighlightingBrush(
								is_dark ? Color.FromRgb(0xDC, 0xDC, 0xAA) : Color.FromRgb(0x74, 0x53, 0x1F)); // yellow / brown
							break;
					}
				}

				// Force redraw with new colours
				m_script_editor.SyntaxHighlighting = null;
				m_script_editor.SyntaxHighlighting = highlighting;
			}
		}

		/// <summary>Show a tooltip with the debug value when hovering over 'vars.xxx.yyy' expressions</summary>
		private void OnMouseHover(object sender, System.Windows.Input.MouseEventArgs e)
		{
			ThreadHelper.ThrowIfNotOnUIThread();

			var pos = m_script_editor.TextArea.TextView.GetPositionFloor(e.GetPosition(m_script_editor.TextArea.TextView) + m_script_editor.TextArea.TextView.ScrollOffset);
			if (pos == null)
				return;

			var offset = m_script_editor.Document.GetOffset(pos.Value.Location);
			var expression = ExtractVarsExpression(m_script_editor.Document, offset);
			if (expression == null)
				return;

			// Evaluate the expression via the debugger
			if (m_package?.Dte?.Debugger == null || m_package.Dte.Debugger.CurrentMode != EnvDTE.dbgDebugMode.dbgBreakMode)
			{
				ShowHoverTooltip($"{expression}\n(not in break mode)");
				return;
			}

			var dbg_expr = m_package.Dte.Debugger.GetExpression(expression);
			if (dbg_expr.IsValidValue)
				ShowHoverTooltip($"{expression} = {dbg_expr.Value}");
			else
				ShowHoverTooltip($"{expression}\n(could not evaluate)");
		}

		private void OnMouseHoverStopped(object sender, System.Windows.Input.MouseEventArgs e)
		{
			if (m_hover_tooltip != null)
				m_hover_tooltip.IsOpen = false;
		}

		private void ShowHoverTooltip(string text)
		{
			if (m_hover_tooltip == null)
			{
				m_hover_tooltip = new ToolTip();
			}
			m_hover_tooltip.Content = new TextBlock
			{
				Text = text,
				FontFamily = new FontFamily("Consolas"),
				FontSize = 12,
			};
			m_hover_tooltip.PlacementTarget = m_script_editor;
			m_hover_tooltip.IsOpen = true;
		}

		/// <summary>
		/// Extract a 'vars.xxx.yyy.zzz' expression at the given document offset.
		/// Strips the leading 'vars.' so the result can be passed to GetExpression().
		/// Returns null if the cursor is not over a vars expression.
		/// </summary>
		private static string? ExtractVarsExpression(ICSharpCode.AvalonEdit.Document.TextDocument doc, int offset)
		{
			if (offset < 0 || offset >= doc.TextLength)
				return null;

			// Expand to find the full dotted identifier at this position
			var start = offset;
			var end = offset;

			// Walk backward over identifier chars and dots
			while (start > 0)
			{
				var ch = doc.GetCharAt(start - 1);
				if (char.IsLetterOrDigit(ch) || ch == '_' || ch == '.')
					start--;
				else
					break;
			}

			// Walk forward over identifier chars and dots
			while (end < doc.TextLength)
			{
				var ch = doc.GetCharAt(end);
				if (char.IsLetterOrDigit(ch) || ch == '_' || ch == '.')
					end++;
				else
					break;
			}

			var full_text = doc.GetText(start, end - start).Trim('.');
			if (!full_text.StartsWith("vars.", StringComparison.Ordinal))
				return null;

			// Strip the 'vars.' prefix — the remainder is the debug expression
			return full_text.Substring(5);
		}

		/// <summary>Initialize with the VS package (provides DTE access)</summary>
		internal void Initialize(LDrawVisualiserPackage package)
		{
			m_package = package;

			// Apply options defaults
			var options = package.Options;
			if (!string.IsNullOrEmpty(options.DefaultScriptText))
				m_script_editor.Text = options.DefaultScriptText;
			if (!string.IsNullOrEmpty(options.DefaultAddress))
				m_address_box.Text = options.DefaultAddress;

			m_auto_refresh_check.IsChecked = options.DefaultAutoRefresh;
		}

		/// <summary>Clean up resources</summary>
		internal void Shutdown()
		{
			m_streamer.Dispose();
			VSColorTheme.ThemeChanged -= _ => ApplyThemeToEditor();
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

		/// <summary>Show autocomplete suggestions after typing a dot or starting a word</summary>
		private void OnTextEntered(object sender, System.Windows.Input.TextCompositionEventArgs e)
		{
			if (e.Text == ".")
			{
				ShowCompletionWindow(CompletionProvider.GetMemberCompletions(m_script_editor, m_script_editor.CaretOffset));
			}
			else if (e.Text.Length == 1 && char.IsLetter(e.Text[0]))
			{
				// Check if this is the start of a word (no preceding letter)
				var offset = m_script_editor.CaretOffset;
				if (offset >= 2)
				{
					var prev = m_script_editor.Document.GetCharAt(offset - 2);
					if (!char.IsLetterOrDigit(prev) && prev != '_' && prev != '.')
					{
						ShowCompletionWindow(CompletionProvider.GetKeywordCompletions(m_script_editor, offset));
					}
				}
				else
				{
					ShowCompletionWindow(CompletionProvider.GetKeywordCompletions(m_script_editor, offset));
				}
			}
		}

		private void ShowCompletionWindow(IEnumerable<CSharpCompletionData> items)
		{
			var item_list = items.ToList();
			if (item_list.Count == 0)
				return;

			m_completion_window = new CompletionWindow(m_script_editor.TextArea);
			foreach (var item in item_list)
				m_completion_window.CompletionList.CompletionData.Add(item);

			m_completion_window.Show();
			m_completion_window.Closed += (_, _) => m_completion_window = null;
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
