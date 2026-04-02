using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Input;
using System.Windows.Media;
using ICSharpCode.AvalonEdit.CodeCompletion;
using Microsoft.VisualStudio.PlatformUI;
using Microsoft.VisualStudio.Shell;
using Rylogic.LDrawVisualiser.Core;
using Rylogic.Utility;

namespace Rylogic.LDrawVisualiser
{
	public static class VisualizerCommands
	{
		public static readonly RoutedUICommand SaveAndRefresh = new("Save and Refresh", "SaveAndRefresh", typeof(VisualizerCommands), new InputGestureCollection
		{
			new KeyGesture(Key.S, ModifierKeys.Control)
		});
	}

	public partial class VisualizerToolWindowControl : UserControl
	{
		private LDrawVisualiserPackage? m_package;
		private readonly ScriptCompiler m_compiler = new();
		private readonly LDrawStreamer m_streamer = new();
		private string? m_last_script_text;
		private DateTime m_last_send_time;
		private CompletionWindow? m_completion_window;
		private bool m_suppress_selection_changed;

		public VisualizerToolWindowControl()
		{
			InitializeComponent();

			m_script_editor.Text = "";

			// Ctrl+S saves and refreshes
			CommandBindings.Add(new CommandBinding(VisualizerCommands.SaveAndRefresh, OnSaveAndRefresh));

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

		/// <summary>The scripts directory from options</summary>
		private string ScriptsDirectory => m_package?.Options.ScriptsDirectory ?? Util.ResolveAppDataPath("Rylogic", "VSExtension", "LDrawVisualiserScripts");

		/// <summary>The currently selected script name</summary>
		private string CurrentScriptName => m_script_name_combo.Text?.Trim() is { Length: > 0 } name ? name : "Script";

		/// <summary>Get the file path for a given script name</summary>
		private string ScriptFilePath(string name) => Path.Combine(ScriptsDirectory, $"{name}.cs");

		/// <summary>Apply VS theme colours to the AvalonEdit editor (which doesn't use dynamic resources)</summary>
		private void ApplyThemeToEditor()
		{
			var bg = VSColorTheme.GetThemedColor(EnvironmentColors.ToolWindowBackgroundColorKey);
			var fg = VSColorTheme.GetThemedColor(EnvironmentColors.ToolWindowTextColorKey);
			m_script_editor.Background = new SolidColorBrush(Color.FromRgb(bg.R, bg.G, bg.B));
			m_script_editor.Foreground = new SolidColorBrush(Color.FromRgb(fg.R, fg.G, fg.B));

			// Line number colour
			m_script_editor.LineNumbersForeground = new SolidColorBrush(Color.FromArgb(128, fg.R, fg.G, fg.B));

			// Theme the combo box (editable ComboBox has an internal TextBox that needs styling)
			var bg_brush = new SolidColorBrush(Color.FromRgb(bg.R, bg.G, bg.B));
			var fg_brush = new SolidColorBrush(Color.FromRgb(fg.R, fg.G, fg.B));
			m_script_name_combo.Background = bg_brush;
			m_script_name_combo.Foreground = fg_brush;

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
								is_dark ? Color.FromRgb(0x6A, 0x99, 0x55) : Color.FromRgb(0x00, 0x80, 0x00));
							break;
						case "String":
						case "Char":
							colour.Foreground = new ICSharpCode.AvalonEdit.Highlighting.SimpleHighlightingBrush(
								is_dark ? Color.FromRgb(0xCE, 0x91, 0x78) : Color.FromRgb(0xA3, 0x15, 0x15));
							break;
						case "Preprocessor":
							colour.Foreground = new ICSharpCode.AvalonEdit.Highlighting.SimpleHighlightingBrush(
								is_dark ? Color.FromRgb(0xBD, 0x63, 0xC5) : Color.FromRgb(0x80, 0x00, 0x80));
							break;
						case "Punctuation":
							colour.Foreground = new ICSharpCode.AvalonEdit.Highlighting.SimpleHighlightingBrush(
								is_dark ? Color.FromRgb(0xDC, 0xDC, 0xDC) : Color.FromRgb(0x00, 0x00, 0x00));
							break;
						case "ValueTypeKeywords": case "ReferenceTypeKeywords": case "ThisOrBaseReference":
						case "NullOrValueKeywords": case "Keywords": case "GotoKeywords": case "ContextKeywords":
						case "ExceptionKeywords": case "CheckedKeyword": case "UnsafeKeywords":
						case "OperatorKeywords": case "ParameterModifiers": case "Modifiers":
						case "Visibility": case "NamespaceKeywords": case "GetSetAddRemove":
						case "TrueFalse": case "TypeKeywords":
							colour.Foreground = new ICSharpCode.AvalonEdit.Highlighting.SimpleHighlightingBrush(
								is_dark ? Color.FromRgb(0x56, 0x9C, 0xD6) : Color.FromRgb(0x00, 0x00, 0xFF));
							break;
						case "NumberLiteral":
							colour.Foreground = new ICSharpCode.AvalonEdit.Highlighting.SimpleHighlightingBrush(
								is_dark ? Color.FromRgb(0xB5, 0xCE, 0xA8) : Color.FromRgb(0x09, 0x86, 0x58));
							break;
						case "MethodCall":
							colour.Foreground = new ICSharpCode.AvalonEdit.Highlighting.SimpleHighlightingBrush(
								is_dark ? Color.FromRgb(0xDC, 0xDC, 0xAA) : Color.FromRgb(0x74, 0x53, 0x1F));
							break;
					}
				}
				m_script_editor.SyntaxHighlighting = null;
				m_script_editor.SyntaxHighlighting = highlighting;
			}
		}

		/// <summary>Show a tooltip with the debug value when hovering over 'vars.xxx.yyy' expressions</summary>
		private void OnMouseHover(object sender, MouseEventArgs e)
		{
			ThreadHelper.ThrowIfNotOnUIThread();

			var pos = m_script_editor.TextArea.TextView.GetPositionFloor(e.GetPosition(m_script_editor.TextArea.TextView) + m_script_editor.TextArea.TextView.ScrollOffset);
			if (pos == null)
				return;

			var offset = m_script_editor.Document.GetOffset(pos.Value.Location);
			var expression = ExtractVarsExpression(m_script_editor.Document, offset);
			if (expression == null)
				return;

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

		private void OnMouseHoverStopped(object sender, MouseEventArgs e)
		{
			if (m_hover_tooltip != null)
				m_hover_tooltip.IsOpen = false;
		}

		private void ShowHoverTooltip(string text)
		{
			m_hover_tooltip ??= new ToolTip();
			m_hover_tooltip.Content = new TextBlock
			{
				Text = text,
				FontFamily = new FontFamily("Consolas"),
				FontSize = 12,
			};
			m_hover_tooltip.PlacementTarget = m_script_editor;
			m_hover_tooltip.IsOpen = true;
		}

		private static string? ExtractVarsExpression(ICSharpCode.AvalonEdit.Document.TextDocument doc, int offset)
		{
			if (offset < 0 || offset >= doc.TextLength)
				return null;

			var start = offset;
			var end = offset;

			while (start > 0)
			{
				var ch = doc.GetCharAt(start - 1);
				if (char.IsLetterOrDigit(ch) || ch == '_' || ch == '.')
					start--;
				else
					break;
			}

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

			return full_text.Substring(5);
		}

		/// <summary>Initialize with the VS package (provides DTE access)</summary>
		internal void Initialize(LDrawVisualiserPackage package)
		{
			m_package = package;
			var options = package.Options;

			m_auto_refresh_check.IsChecked = options.DefaultAutoRefresh;

			// Populate the script name combo from existing files
			PopulateScriptNames();

			// Select the last used script (this triggers loading the script text)
			var last_script = options.LastSelectedScript;
			SelectScript(last_script);
		}

		/// <summary>Clean up resources</summary>
		internal void Shutdown()
		{
			// Save the current script before shutting down
			SaveCurrentScript();
			PersistLastSelectedScript();

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

		// -- Script persistence ---------------------------------------------------

		/// <summary>Populate the combo box with script names from the scripts directory</summary>
		private void PopulateScriptNames()
		{
			m_suppress_selection_changed = true;
			try
			{
				var items = new List<string>();
				var dir = ScriptsDirectory;
				if (Directory.Exists(dir))
				{
					foreach (var file in Directory.GetFiles(dir, "*.cs"))
						items.Add(Path.GetFileNameWithoutExtension(file));
				}

				if (items.Count == 0)
					items.Add("Script");

				m_script_name_combo.Items.Clear();
				foreach (var name in items)
					m_script_name_combo.Items.Add(name);
			}
			finally
			{
				m_suppress_selection_changed = false;
			}
		}

		/// <summary>Select a script by name, loading its content into the editor</summary>
		private void SelectScript(string name)
		{
			m_suppress_selection_changed = true;
			try
			{
				// Move the selected name to the top of the combo box
				MoveToTop(name);

				// Load the script text from file
				var filepath = ScriptFilePath(name);
				if (File.Exists(filepath))
				{
					try
					{
						m_script_editor.Text = File.ReadAllText(filepath);
					}
					catch
					{
						m_script_editor.Text = m_package?.Options.DefaultScriptText ?? "";
					}
				}
				else
				{
					m_script_editor.Text = m_package?.Options.DefaultScriptText ?? "";
				}

				// Reset compilation state for the new script
				m_last_script_text = null;
			}
			finally
			{
				m_suppress_selection_changed = false;
			}
		}

		/// <summary>Move a name to the top of the combo box items</summary>
		private void MoveToTop(string name)
		{
			// Remove if already present
			for (var i = m_script_name_combo.Items.Count - 1; i >= 0; --i)
			{
				if (string.Equals(m_script_name_combo.Items[i] as string, name, StringComparison.OrdinalIgnoreCase))
					m_script_name_combo.Items.RemoveAt(i);
			}

			m_script_name_combo.Items.Insert(0, name);
			m_script_name_combo.Text = name;
		}

		/// <summary>Save the current script text to a file matching the combo box name</summary>
		private void SaveCurrentScript()
		{
			try
			{
				var name = CurrentScriptName;
				var dir = ScriptsDirectory;
				Directory.CreateDirectory(dir);
				File.WriteAllText(ScriptFilePath(name), m_script_editor.Text);
			}
			catch
			{
				// Silently ignore save failures
			}
		}

		/// <summary>Persist the currently selected script name to options</summary>
		private void PersistLastSelectedScript()
		{
			if (m_package == null) return;
			m_package.Options.LastSelectedScript = CurrentScriptName;
			m_package.Options.SaveSettingsToStorage();
		}

		private void OnScriptSelectionChanged(object sender, SelectionChangedEventArgs e)
		{
			if (m_suppress_selection_changed) return;
			if (m_script_name_combo.SelectedItem is not string name) return;

			// Save the current script before switching
			SaveCurrentScript();

			// Load the newly selected script
			SelectScript(name);
			PersistLastSelectedScript();
		}

		/// <summary>Handle Enter in the combo box to commit a typed name</summary>
		private void OnScriptNameKeyDown(object sender, KeyEventArgs e)
		{
			if (e.Key != Key.Return) return;

			var name = CurrentScriptName;

			// Save the current script under the (possibly new) name
			SaveCurrentScript();

			// Refresh the combo box and select the new name
			PopulateScriptNames();
			SelectScript(name);
			PersistLastSelectedScript();

			e.Handled = true;
		}

		// -- Connection -----------------------------------------------------------

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
		}

		private void OnRefreshClick(object sender, RoutedEventArgs e)
		{
			EvaluateAndSend();
		}

		private void OnSaveAndRefresh(object sender, ExecutedRoutedEventArgs e)
		{
			SaveCurrentScript();
			EvaluateAndSend();
		}

		// -- Autocomplete ---------------------------------------------------------

		private void OnTextEntered(object sender, TextCompositionEventArgs e)
		{
			if (e.Text == ".")
			{
				ShowCompletionWindow(CompletionProvider.GetMemberCompletions(m_script_editor, m_script_editor.CaretOffset));
			}
			else if (e.Text.Length == 1 && char.IsLetter(e.Text[0]))
			{
				var offset = m_script_editor.CaretOffset;
				if (offset >= 2)
				{
					var prev = m_script_editor.Document.GetCharAt(offset - 2);
					if (!char.IsLetterOrDigit(prev) && prev != '_' && prev != '.')
						ShowCompletionWindow(CompletionProvider.GetKeywordCompletions(m_script_editor, offset));
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
			if (item_list.Count == 0) return;

			m_completion_window = new CompletionWindow(m_script_editor.TextArea);
			foreach (var item in item_list)
				m_completion_window.CompletionList.CompletionData.Add(item);

			m_completion_window.Show();
			m_completion_window.Closed += (_, _) => m_completion_window = null;
		}

		// -- Evaluate & Send ------------------------------------------------------

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

				// Save script to file after each successful compile
				SaveCurrentScript();
			}

			if (m_compiler.CompiledScript == null)
			{
				SetStatus("No compiled script");
				return;
			}

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
