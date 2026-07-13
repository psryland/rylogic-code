using System;
using System.Collections.Generic;
using System.Collections.Specialized;
using System.ComponentModel;
using System.ComponentModel.Design;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Controls.Primitives;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Threading;
using System.Xml;
using ICSharpCode.AvalonEdit;
using ICSharpCode.AvalonEdit.AddIn;
using ICSharpCode.AvalonEdit.CodeCompletion;
using ICSharpCode.AvalonEdit.Document;
using ICSharpCode.AvalonEdit.Editing;
using ICSharpCode.AvalonEdit.Folding;
using ICSharpCode.AvalonEdit.Highlighting;
using ICSharpCode.AvalonEdit.Highlighting.Xshd;
using ICSharpCode.AvalonEdit.Indentation;
using ICSharpCode.AvalonEdit.Search;
using Rylogic.Extn;
using Rylogic.Gfx;
using Rylogic.Gui.WPF;
using Rylogic.Script;
using Rylogic.Str;
using Rylogic.Utility;

namespace LDraw.UI
{
	[DebuggerDisplay("{FilePath,nq}")]
	public sealed partial class ScriptUI :UserControl, IDockable, IDisposable, INotifyPropertyChanged
	{
		// Notes:
		//  - A script editor is used to edit existing script files.
		private CancellationTokenSource m_cancel_load = new();
		private TaskCompletionSource<object?> m_initial_load = new(TaskCreationOptions.RunContinuationsAsynchronously);

		static ScriptUI()
		{
			// Register the LdrScript syntax definitions in AvalonEdit
			var stream = typeof(ScriptUI).Assembly.GetManifestResourceStream("LDraw.res.LdrSyntaxRules.xshd") ?? throw new Exception("LdrSyntaxRules.xshd embedded resource not found");
			using var reader = new XmlTextReader(stream);
			var syntax_rules = HighlightingLoader.Load(reader, HighlightingManager.Instance);
			HighlightingManager.Instance.RegisterHighlighting("Ldr", new[] { ".ldr" }, syntax_rules);
		}
		public ScriptUI(Source source)
		{
			InitializeComponent();

			// A ScriptUI represents the script *file*, which is longer-lived than any particular
			// native Source. Capture the document identity up-front so the editor stays valid even
			// if the Source is later removed from the store. The live Source is resolved dynamically
			// (see 'Source') and re-created on demand when rendering.
			Model = source.Model;
			FilePath = source.FilePath;
			ContextId = View3d.ContextIdFromFilepath(FilePath);

			DockControl = new DockControl(this, $"Script-{ContextId}")
			{
				ShowTitle = false,
				TabText = ScriptName,
				TabToolTip = FilePath,
				TabCMenu = TabCMenu(),
				DestroyOnClose = true,
			};
			LdrAutoComplete = new View3d.AutoComplete();
			Editor = m_editor;

			Render = Command.Create(this, RenderInternal);
			SaveScript = Command.Create(this, SaveScriptInternal);
			RemoveObjects = Command.Create(this, RemoveObjectsInternal);
			CloseScript = Command.Create(this, CloseScriptInternal);
			IndentSelection = Command.Create(this, IndentSelectionInternal);
			CommentOutSelection = Command.Create(this, CommentOutSelectionInternal);
			UncommentSelection = Command.Create(this, UncommentSelectionInternal);

			// Track the live source for this script. It may not currently exist in the store
			// (e.g. it was removed from the Sources list), in which case 'Source' is null until
			// the script is rendered again.
			Model.SourcesChanged += HandleSourcesChanged;
			Model.Scenes.CollectionChanged += HandleScenesCollectionChanged;
			Log.EntriesChanged += HandleLogEntriesChanged;
			RefreshCurrentSource();

			DataContext = this;
			Loaded += async delegate
			{
				Editor.Text = $"Loading '{FilePath}'...";

				try
				{
					Editor.Text = await File.ReadAllTextAsync(FilePath, m_cancel_load.Token);
				}
				catch (OperationCanceledException) { }
				catch (Exception ex)
				{
					Editor.Text = $"Error loading file: {ex.Message}";
				}
				finally
				{
					m_initial_load.TrySetResult(null);
				}
			};
		}
		protected override void OnPreviewGotKeyboardFocus(KeyboardFocusChangedEventArgs e)
		{
			base.OnPreviewGotKeyboardFocus(e);
			CheckForChangedScript();
		}
		public void Dispose()
		{
			m_cancel_load.Cancel();

			// Detach from the model and the live source. The Source is owned by the model,
			// so we only unsubscribe; we never dispose it here.
			Model.SourcesChanged -= HandleSourcesChanged;
			Model.Scenes.CollectionChanged -= HandleScenesCollectionChanged;
			Log.EntriesChanged -= HandleLogEntriesChanged;
			Source = null;

			// Remove this script UI from the model
			Model.Scripts.Remove(this);

			Editor = null!;
			DockControl = null!;
			GC.SuppressFinalize(this);
		}

		/// <summary>Provides support for the DockContainer</summary>
		public DockControl DockControl
		{
			get;
			private set
			{
				if (field == value) return;
				if (field != null)
				{
					field.ActiveChanged -= HandleActiveChanged;
					field.SavingLayout -= HandleSavingLayout;
					Util.Dispose(ref field!);
				}
				field = value;
				if (field != null)
				{
					field.SavingLayout += HandleSavingLayout;
					field.ActiveChanged += HandleActiveChanged;
				}

				// Handlers
				void HandleActiveChanged(object? sender, ActiveContentChangedEventArgs e)
				{
					// When activated, restore focus to the editor
					if (DockControl.IsActiveContent)
						Editor.Focus();

					//	Options.BkColour = args.ContentNew == this ? Color.LightSteelBlue : Color.LightGray;
					//	Invalidate();
				}
				void HandleSavingLayout(object? sender, DockContainerSavingLayoutEventArgs e)
				{
					//if (!Model.IsTempScriptFilepath(Filepath))
					//	e.Node.Add2(nameof(Filepath), Filepath, false);
				}
			}
		} = null!;

		/// <summary>The live LDraw source for this script, or null if it is not currently in the store</summary>
		public Source? Source
		{
			get;
			private set
			{
				if (field == value) return;
				if (field != null)
				{
					field.PropertyChanged -= HandleSourcePropertyChanged;
					field.SourceChanged -= HandleSourceChanged;
					// Don't dispose Source, we don't own it.
				}
				field = value;
				if (field != null)
				{
					field.SourceChanged += HandleSourceChanged;
					field.PropertyChanged += HandleSourcePropertyChanged;
				}

				// The available source changed, so dependent UI state may have changed too.
				if (DockControl != null)
					DockControl.TabText = ScriptName;
				NotifyPropertyChanged(nameof(Source));
				NotifyPropertyChanged(nameof(ScriptName));
				NotifyPropertyChanged(nameof(CanRender));
				NotifyPropertyChanged(nameof(CanRemove));

				void HandleSourceChanged(object? sender, EventArgs e)
				{
					// ?
				}
				void HandleSourcePropertyChanged(object? sender, PropertyChangedEventArgs e)
				{
					switch (e.PropertyName)
					{
						case nameof(LDraw.Source.Name):
						{
							// Update the tab text
							if (DockControl != null)
								DockControl.TabText = ScriptName;

							NotifyPropertyChanged(nameof(ScriptName));
							break;
						}
						case nameof(LDraw.Source.SelectedScenes):
						{
							// Scene membership affects whether render/remove are meaningful
							NotifyPropertyChanged(nameof(CanRender));
							NotifyPropertyChanged(nameof(CanRemove));
							break;
						}
					}
				}
			}
		}

		/// <summary>Resolve the live source for this script's context id (may be null if not loaded)</summary>
		private void RefreshCurrentSource()
		{
			Source = Model.Sources.FirstOrDefault(x => x.ContextId == ContextId);
		}
		private void HandleSourcesChanged(object? sender, EventArgs e)
		{
			RefreshCurrentSource();
		}
		private void HandleScenesCollectionChanged(object? sender, NotifyCollectionChangedEventArgs e)
		{
			NotifyPropertyChanged(nameof(CanRender));
		}
		private void HandleLogEntriesChanged(object? sender, EventArgs e)
		{
			RefreshErrorMarkers();
		}

		/// <summary>Auto complete provider for LDraw script</summary>
		private View3d.AutoComplete LdrAutoComplete { get; }

		/// <summary>App logic</summary>
		public Model Model { get; }

		/// <summary>The filepath of the script this editor represents</summary>
		public string FilePath { get; }

		/// <summary>Context id for objects created by this script (stable; derived from the filepath)</summary>
		public Guid ContextId { get; }

		/// <summary>The name displayed for this script UI</summary>
		public string ScriptName => Source?.Name is string name && name.Length != 0 ? name : System.IO.Path.GetFileNameWithoutExtension(FilePath);

		/// <summary>True if the script can be rendered (it is already shown, or there is a scene to add it to)</summary>
		public bool CanRender => Source is Source src ? src.CanRender : Model.Scenes.Count != 0;

		/// <summary>True if the script currently has objects in scenes that can be removed</summary>
		public bool CanRemove => Source?.CanRender ?? false;

		/// <summary>The text editor control</summary>
		public TextEditor Editor
		{
			get => m_text_editor;
			private set
			{
				if (m_text_editor == value) return;
				if (m_text_editor != null)
				{
					// Unhook handlers
					m_text_editor.MouseHoverStopped += HandleMouseHoverStopped;
					m_text_editor.MouseHover += HandleMouseHover;
					m_text_editor.TextArea.Caret.PositionChanged -= HandleCaretPositionChanged;
					m_text_editor.TextArea.SelectionChanged -= HandleSelectionChanged;
					m_text_editor.Document.TextChanged -= HandleTextChanged;
					m_text_editor.PreviewKeyDown -= HandleKeyDown;

					// Drop the text marker service
					m_text_marker_service = null!;

					// Release the fields collection
					m_fields = null;

					// Uninstall folding support
					m_folding_timer.Stop();
					FoldingManager.Uninstall(m_folding_mgr);
					m_folding_mgr = null!;

					// Uninstall search support
					m_search_ui.Uninstall();
					m_search_ui = null!;
				}
				m_text_editor = value;
				if (m_text_editor != null)
				{
					// Set general properties
					m_text_editor.TextArea.SelectionCornerRadius = 1.0;

					// Add support for searching
					m_search_ui = SearchPanel.Install(m_text_editor);

					// Add support for folding
					m_folding_mgr = FoldingManager.Install(m_text_editor.TextArea);
					m_folding_timer = new DispatcherTimer { Interval = TimeSpan.FromSeconds(2) };
					m_folding_timer.Tick += UpdateFoldings;
					m_folding_timer.Start();

					// Add indentation support
					m_text_editor.TextArea.IndentationStrategy = new IndentStrategy(m_text_editor);

					// Add underline errors support
					m_text_marker_service = new TextMarkerService(m_text_editor.Document);
					m_text_editor.TextArea.TextView.BackgroundRenderers.Add(m_text_marker_service);
					m_text_editor.TextArea.TextView.LineTransformers.Add(m_text_marker_service);
					var services = (IServiceContainer?)m_text_editor.Document.ServiceProvider.GetService(typeof(IServiceContainer)) ?? throw new NullReferenceException("IServiceContainer service not available");
					services.AddService(typeof(ITextMarkerService), m_text_marker_service);

					// Adjust key bindings
					AvalonEditCommands.DeleteLine.InputGestures.Clear();
					AvalonEditCommands.IndentSelection.InputGestures.Clear();

					// Hook up handlers
					m_text_editor.PreviewKeyDown += HandleKeyDown;
					m_text_editor.Document.TextChanged += HandleTextChanged;
					m_text_editor.TextArea.SelectionChanged += HandleSelectionChanged;
					m_text_editor.TextArea.Caret.PositionChanged += HandleCaretPositionChanged;
					m_text_editor.MouseHover += HandleMouseHover;
					m_text_editor.MouseHoverStopped += HandleMouseHoverStopped;
				}

				// Handlers
				TextSegmentCollection<TextSegment> FindFields()
				{
					var doc = Editor.Document;
					const string field_pattern = @"(<\w+>)|((?:\w+\s*\|\s*)+\w+)";
					var search = SearchStrategyFactory.Create(field_pattern, true, false, SearchMode.RegEx);
					var fields = new TextSegmentCollection<TextSegment>();
					fields.AddRange(search.FindAll(doc, 0, doc.TextLength).Cast<TextSegment>());
					return fields;
				}
				void HandleKeyDown(object? sender, KeyEventArgs e)
				{
					switch (e.Key)
					{
					case Key.Space:
						{
							// Display the auto complete or call tip
							if (Keyboard.Modifiers.HasFlag(ModifierKeys.Control))
							{
								if (Keyboard.Modifiers.HasFlag(ModifierKeys.Shift))
									ShowCallTip();
								else
									ShowAutoComplete();
								e.Handled = true;
							}
							break;
						}
					case Key.Up:
					case Key.Down:
						{
							// Quickly navigate to "<field>" or "Se|ect" fields
							if (Keyboard.Modifiers.HasFlag(ModifierKeys.Control))
							{
								var doc = Editor.Document;
								var text_length = doc.TextLength;

								// Clear the selection so that find isn't limited to the selected text
								if (e.Key == Key.Up)
									Editor.Select(Editor.SelectionStart, 0);
								else
									Editor.Select(Editor.SelectionStart + Editor.SelectionLength, 0);

								// Update the cache of field text segments
								m_fields ??= FindFields();

								// Search for a field identifier
								var seg = m_fields.FindFirstSegmentWithStartAfter(Editor.CaretOffset);
								if (e.Key == Key.Down)
								{
									seg ??= m_fields.FirstSegment;
								}
								if (e.Key == Key.Up)
								{
									if (seg != null) seg = m_fields.GetPreviousSegment(seg);
									seg ??= m_fields.LastSegment;
								}
								if (seg == null)
									break;

								// Expand the range to include '[' and ']' characters
								int beg = seg.StartOffset, end = seg.EndOffset;
								for (; beg > 0 && doc.GetCharAt(beg - 1) == '['; --beg) { }
								for (; end < text_length && doc.GetCharAt(end) == ']'; ++end) { }
								Editor.Select(beg, end - beg);
								e.Handled = true;
							}
							break;
						}
					case Key.D1:
					case Key.D2:
					case Key.D3:
					case Key.D4:
					case Key.D5:
					case Key.D6:
					case Key.D7:
					case Key.D8:
						{
							if (Keyboard.Modifiers.HasFlag(ModifierKeys.Control))
							{
								var fold = !Keyboard.Modifiers.HasFlag(ModifierKeys.Shift);
								var level = (int)e.Key - (int)Key.D1;

								var stack = new Stack<FoldingSection>();
								foreach (var folding in m_folding_mgr.AllFoldings)
								{
									for (; stack.Count != 0 && !stack.Peek().Contains(folding);)
										stack.Pop();

									// Fold/Unfold everything deeper than 'level'
									if (stack.Count >= level)
										folding.IsFolded = fold;

									stack.Push(folding);
								}

								e.Handled = true;
							}
							break;
						}
					}
				}
				void HandleTextChanged(object? sender, EventArgs e)
				{
					// Invalidate cached field matches
					m_fields = null;
				}
				void HandleSelectionChanged(object? sender, EventArgs e)
				{
					NotifyPropertyChanged(nameof(CaretPositionDescription));
				}
				void HandleCaretPositionChanged(object? sender, EventArgs e)
				{
					NotifyPropertyChanged(nameof(CaretPositionDescription));
					NotifyPropertyChanged(nameof(Location));
				}
				void HandleMouseHover(object? sender, MouseEventArgs e)
				{
					var view = m_text_editor.TextArea.TextView;
					var pt = e.GetPosition(view);

					// If the mouse is hovering over an error marker, display a tooltip with the error message
					if (view.GetPositionFloor(pt + view.ScrollOffset) is TextViewPosition pos)
					{
						// Look for error markers at the mouse position
						var ofs = Editor.Document.GetOffset(pos.Line, pos.Column);
						if (m_text_marker_service.GetMarkersAtOffset(ofs).FirstOrDefault() is ITextMarker marker)
						{
							m_tt_errors ??= new ToolTip();
							m_tt_errors.Closed += delegate { m_tt_errors = null; };
							m_tt_errors.PlacementTarget = view;
							m_tt_errors.Placement = PlacementMode.Mouse;
							m_tt_errors.StaysOpen = true;
							m_tt_errors.Content = new TextBlock
							{
								Text = (string?)marker.ToolTip,
								TextWrapping = TextWrapping.Wrap,
							};
							m_tt_errors.IsOpen = true;
							e.Handled = true;
						}
					}
				}
				void HandleMouseHoverStopped(object? sender, MouseEventArgs e)
				{
					if (m_tt_errors != null)
						m_tt_errors.IsOpen = false;
				}
				void ShowAutoComplete()
				{
					// Find the start of the word
					var pos = Editor.CaretOffset;
					var line = Editor.Document.GetLineByOffset(pos);
					var text = Editor.Document.GetText(line.Offset, pos - line.Offset);
					var b = text.LastIndexOf(x => !IsLdrObjectTypeChar(x));
					var partial = text.Substring(b + 1);

					// Determine the ldr script object parent
					var start = pos - partial.Length;
					var address = View3d.AddressAt(Editor.Document.GetText(0, start));

					// Get the completion possibilities
					var templates = LdrAutoComplete.Lookup(partial, true, LdrAutoComplete[address]);

					// Display the auto completion matches
					var completion_ui = new CompletionWindow(Editor.TextArea);
					completion_ui.CompletionList.CompletionData.AddRange(templates.Select(x => new CompletionItem(x, address, start, DoAutoComplete)));
					if (completion_ui.CompletionList.CompletionData.Count != 0)
						completion_ui.Show();
				}
				void ShowCallTip()
				{
					// Determine the ldr script object parent and lookup the template
					// Note: text.Length != Editor.CurrentPos because of multi-byte characters
					var address = View3d.AddressAt(Editor.Document.GetText(0, Editor.CaretOffset));
					var template = LdrAutoComplete[address];
					if (template == null)
						return;

					var flags = View3d.AutoComplete.EExpandFlags.OptionalChildTemplates | View3d.AutoComplete.EExpandFlags.Optionals;
					var insight = new InsightWindow(Editor.TextArea)
					{
						Content = View3d.AutoComplete.ExpandTemplate(template, flags, 0, "  "),
						SizeToContent = SizeToContent.WidthAndHeight
					};
					insight.Show();
				}
				void DoAutoComplete(CompletionItem completion)
				{
					var doc = Editor.Document;
					var text_length = doc.TextLength;

					// Determine the range of the text to replace
					var end = completion.StartOffset;
					for (; end != text_length && IsLdrObjectTypeChar(doc.GetCharAt(end)); ++end) { }

					// Determine the current indent level
					var indent_level = 0;
					var current_indent = doc.GetText(TextUtilities.GetLeadingWhitespace(doc, doc.GetLineByOffset(completion.StartOffset)));
					for (; current_indent.StartsWith(Editor.Options.IndentationString); ++indent_level)
						current_indent = current_indent.Substring(Editor.Options.IndentationString.Length);

					// Replace with the expanded template
					var text = View3d.AutoComplete.ExpandTemplate(completion.Template, View3d.AutoComplete.EExpandFlags.Optionals, indent_level, Editor.Options.IndentationString);
					doc.Replace(completion.StartOffset, end - completion.StartOffset, text);
				}
				void UpdateFoldings(object? sender, EventArgs e)
				{
					var foldings = new List<NewFolding>();
					using var reader = Editor.Document.CreateReader();
					using var src = (Src)new TextReaderSrc(reader);
					var com = new InComment();

					var last_newline_offset = 0;
					var start_offsets = new Stack<int>();
					var embedded_offset = int.MaxValue;
					foreach (var (ch, i) in src.WithIndex())
					{
						if (com.WithinComment(src, 0))
							continue;

						if (ch == '\n')
						{
							last_newline_offset = i + 1;
						}
						else if (ch == '{')
						{
							start_offsets.Push(i);
						}
						else if (ch == '}' && start_offsets.Count != 0)
						{
							var start = start_offsets.Pop();
							if (start < last_newline_offset)
								foldings.Add(new NewFolding(start, i + 1));
						}
						else if (src.Match("#embedded"))
						{
							embedded_offset = i + "#embedded".Length;
						}
						else if (src.Match("#end"))
						{
							if (embedded_offset < last_newline_offset)
								foldings.Add(new NewFolding(embedded_offset, i + "#end".Length));
							embedded_offset = int.MaxValue;
						}
					}
					foldings.Sort((a, b) => a.StartOffset.CompareTo(b.StartOffset));
					m_folding_mgr.UpdateFoldings(foldings, -1);
				}
			}
		}
		private TextEditor m_text_editor = null!;
		private SearchPanel m_search_ui = null!;
		private FoldingManager m_folding_mgr = null!;
		private DispatcherTimer m_folding_timer = null!;
		private TextSegmentCollection<TextSegment>? m_fields = null;
		private TextMarkerService m_text_marker_service = null!;
		private ToolTip? m_tt_errors;

		/// <summary>The current position in the editor</summary>
		public TextLocation Location
		{
			get => Editor.Document.GetLocation(Editor.CaretOffset);
			set => Editor.CaretOffset = Editor.Document.GetOffset(value);
		}
		public void ScrollTo(int line, int column, bool move_caret)
		{
			line = Math.Clamp(line, 1, Editor.Document.LineCount);
			column = Math.Clamp(column, 1, Editor.Document.GetLineByNumber(line).Length + 1);

			if (move_caret)
				Editor.CaretOffset = Editor.Document.GetOffset(line, column);

			Editor.ScrollTo(line, column);
		}
		public async Task ScrollToAsync(int line, int column, bool move_caret)
		{
			await m_initial_load.Task;
			ScrollTo(line, column, move_caret);
		}

		/// <summary>The range of selected characters</summary>
		public string CaretPositionDescription
		{
			get
			{
				var ss = Editor.SelectionStart;
				var sl = Editor.SelectionLength;
				return sl == 0 ? $"Pos: {Editor.CaretOffset}" : $"Sel: {ss}..{ss + sl} ({sl})";
			}
		}

		///// <summary>Update the objects associated with 'ContextId' within View3D's object store</summary>
		//private void UpdateObjects()
		//{
		//	LastUpdateTime = Log.Elapsed;
		//	Log.Clear(x => Path_.Compare(x.File, Filepath) == 0);

		//	var scenes = Source.SelectedScenes.ToArray();
		//	var include_paths = Model.Settings.IncludePaths;
		//	var selection = Editor.SelectionLength != 0 ? Editor.TextArea.Selection.GetText() : null;

		//	// Load the script file in a background thread
		//	ThreadPool.QueueUserWorkItem(x =>
		//	{
		//		try
		//		{
		//			if (selection == null)
		//				Model.View3d.LoadScriptFromFile(Filepath, ContextId, include_paths, OnAdd);
		//			else
		//				Model.View3d.LoadScriptFromString(selection, ContextId, include_paths, OnAdd);

		//			void OnAdd(Guid id, bool before)
		//			{
		//				if (before)
		//					Model.Clear(scenes, id);
		//				else
		//					Model.AddObjects(scenes, id);

		//				RefreshErrorMarkers();
		//			}
		//		}
		//		catch (Exception)
		//		{
		//			RefreshErrorMarkers();
		//		}
		//	});
		//}
		//private TimeSpan LastUpdateTime;

		/// <summary>Return the tab context menu</summary>
		private ContextMenu TabCMenu()
		{
			var cmenu = (ContextMenu)FindResource("TabCMenu");
			cmenu.DataContext = this;
			return cmenu;
		}

		///// <summary>Load script from a file</summary>
		//public void LoadFile(string? filepath = null)
		//{
		//	// Prompt for a filepath if not given
		//	if (filepath == null || filepath.Length == 0)
		//	{
		//		var dlg = new OpenFileDialog { Title = "Load Script", Filter = Model.EditableFilesFilter };
		//		if (dlg.ShowDialog(App.Current.MainWindow) != true) return;
		//		filepath = dlg.FileName ?? throw new Exception("Invalid filepath selected");
		//	}
		//	if (!File.Exists(filepath))
		//	{
		//		throw new FileNotFoundException("Load script file failed", filepath);
		//	}

		//	// Load the file into the editor
		//	Editor.Text = File.ReadAllText(filepath);

		//	// Save the filepath
		//	Filepath = filepath;

		//	// Record the file state when last loaded
		//	m_last_fileinfo = FileInfo;

		//	// Sync'd with disk
		//	SaveNeeded = false;
		//}
		//public void LoadFile()
		//{
		//	LoadFile(Filepath);
		//}

		/// <summary>Test the script file for external changes</summary>
		public void CheckForChangedScript()
		{
		//	// If this script has never been saved, no changes
		//	if (!File.Exists(Filepath))
		//		return;

		//	// Look for changes
		//	var fileinfo = FileInfo;
		//	if (m_last_fileinfo != null && fileinfo != null &&
		//		((m_last_fileinfo.LastWriteTimeUtc == fileinfo.LastWriteTimeUtc) ||
		//		(m_last_fileinfo.Length == fileinfo.Length && Editor.Text == File.ReadAllText(Filepath))))
		//	{
		//		m_last_fileinfo = fileinfo;
		//		return;
		//	}

		//	// The file has changed. Prompt if needed.
		//	if (SaveNeeded)
		//	{
		//		var dlg = new MsgBox(Window.GetWindow(this),
		//			$"{Path_.FileName(Filepath)} has changed on disk.\n" +
		//			$"\n" +
		//			$"'Overwrite' = save and replace the version on disk.\n" +
		//			$"'Discard' = discard local changes and reload from disk.\n" +
		//			$"'Ignore' = continue with the local version without saving.\n",
		//			"Script Changed on Disk", MsgBox.EButtons.OverwriteDiscardIgnore);
		//		dlg.ShowDialog();
		//		switch (dlg.Result)
		//		{
		//		case MsgBox.EResult.Overwrite:
		//			SaveFile(Filepath);
		//			break;
		//		case MsgBox.EResult.Discard:
		//			LoadFile(Filepath);
		//			break;
		//		case MsgBox.EResult.Ignore:
		//			m_last_fileinfo = fileinfo;
		//			SaveNeeded = true;
		//			break;
		//		default:
		//			throw new Exception($"{dlg.Result} unexpected");
		//		}
		//	}
		//	else if (!(Model.Settings.ReloadChangedScripts is bool reload))
		//	{
		//		var dlg = new MsgBox(Window.GetWindow(this),
		//			$"'{Filepath}' has changed on disk.\n" +
		//			$"\n" +
		//			$"'Reload' = reload the script from disk.\n" +
		//			$"'Ignore' = continue with the local version.\n",
		//			"Script Changed on Disk", MsgBox.EButtons.ReloadIgnore)
		//		{ ShowAlwaysCheckbox = true };
		//		dlg.ShowDialog();
		//		switch (dlg.Result)
		//		{
		//		case MsgBox.EResult.Reload:
		//			LoadFile(Filepath);
		//			if (dlg.Always) Model.Settings.ReloadChangedScripts = true;
		//			break;
		//		case MsgBox.EResult.Ignore:
		//			if (dlg.Always) Model.Settings.ReloadChangedScripts = false;
		//			m_last_fileinfo = fileinfo;
		//			SaveNeeded = true;
		//			break;
		//		default:
		//			throw new Exception($"{dlg.Result} unexpected");
		//		}
		//	}
		//	else if (reload)
		//	{
		//		LoadFile(Filepath);
		//	}
		}

		/// <summary>Update the error markers into this script</summary>
		private void RefreshErrorMarkers()
		{
			// TODO: Build in background and update error markers in real time
			//m_text_marker_service.RemoveAll(_ => true);
			//foreach (var le in Log.Entries)
			//{
			//	if (Path_.Compare(le.File, FilePath) != 0) continue;
			//	if (le.Elapsed < LastUpdateTime) continue;
			//
			//	var line = Editor.Document.GetLineByNumber(le.Line);
			//	var marker = m_text_marker_service.Create(line.Offset, line.Length);
			//	marker.MarkerTypes = ETextMarkerTypes.SquigglyUnderline | ETextMarkerTypes.LineInScrollBar;
			//	marker.MarkerColor = Colors.Red;
			//	marker.ToolTip = le.Message;
			//}
		}

		/// <summary>True if the script has been edited without being saved</summary>
		public bool SaveNeeded
		{
			get;
			set
			{
				if (field == value) return;
				field = value;

				// Add/Remove a '*' from the tab
				DockControl.TabText = DockControl.TabText.TrimEnd('*') + (field ? "*" : string.Empty);

				// Notify
				NotifyPropertyChanged(nameof(SaveNeeded));
			}
		}

		/// <summary>Render the contents of this script file in the selected scene</summary>
		public Command Render { get; }
		private void RenderInternal()
		{
			if (SaveNeeded)
				SaveScriptInternal();

			// If the source isn't currently in the store (e.g. it was removed from the Sources
			// list), re-add it from the script file. The context id is derived from the filepath,
			// so re-adding re-establishes the link to this editor. This makes the editor behave
			// like an external editor on the file: rendering (re)loads it.
			if (Source is not Source source)
			{
				Model.AddFileSource(FilePath, Model.Scenes.Take(1).ToArray());
				return;
			}

			// Already in the store; reload to pick up the latest file content
			if (!source.SelectedScenes.Any())
				return;

			source.Reload();
			//UpdateObjects();
		}

		/// <summary>Save the contents of the script to file</summary>
		public Command SaveScript { get; }
		private void SaveScriptInternal()
		{
			try
			{
				// Save the file from the editor
				File.WriteAllText(FilePath, Editor.Text);
				SaveNeeded = false;
			}
			catch (Exception ex)
			{
				Log.Write(ELogLevel.Info, ex, "Saving script file failed.");
				MsgBox.Show(Window.GetWindow(this), $"Saving script file failed.\n{ex.Message}", Util.AppProductName, MsgBox.EButtons.OK, MsgBox.EIcon.Information);
			}
		}

		/// <summary>Remove objects associated with this script from the selected scenes</summary>
		public Command RemoveObjects { get; }
		private void RemoveObjectsInternal()
		{
			if (Source is not Source source)
				return;

			var scenes = source.SelectedScenes.ToArray();
			if (scenes.Length == 0) return;
			Model.Clear(scenes, ContextId);
		}

		/// <summary>Close and remove this script</summary>
		public Command CloseScript { get; }
		private void CloseScriptInternal()
		{
			Dispose();
		}

		/// <summary>Indent the selected text</summary>
		public Command IndentSelection { get; }
		private void IndentSelectionInternal()
		{
			AvalonEditCommands.IndentSelection.Execute(null, null);
		}

		/// <summary>Comment out the selected lines using line comments</summary>
		public Command CommentOutSelection { get; }
		private void CommentOutSelectionInternal()
		{
			var doc = Editor.Document;
			var beg = doc.GetLineByOffset(Editor.SelectionStart);
			var end = doc.GetLineByOffset(Editor.SelectionStart + Editor.SelectionLength);

			// Find the smallest indent whitespace
			int len = int.MaxValue;
			for (var line = beg; line != null && line.Offset <= end.Offset; line = line.NextLine)
			{
				var segment = TextUtilities.GetLeadingWhitespace(doc, line);
				if (segment.Length > len) continue;
				len = segment.Length;
			}
			for (var line = beg; line != null && line.Offset <= end.Offset; line = line.NextLine)
			{
				var segment = doc.GetText(line.Offset, len);
				doc.Replace(line.Offset, len, segment + "//");
			}
		}

		/// <summary>Remove one layer of line comments from the selected lines</summary>
		public Command UncommentSelection  { get; }
		private void UncommentSelectionInternal()
		{
			var doc = Editor.Document;
			var beg = doc.GetLineByOffset(Editor.SelectionStart);
			var end = doc.GetLineByOffset(Editor.SelectionStart + Editor.SelectionLength);
			for (var line = beg; line != null && line.Offset <= end.Offset; line = line.NextLine)
			{
				var segment = TextUtilities.GetLeadingWhitespace(doc, line);
				if (line.EndOffset - segment.EndOffset >= 2 && doc.GetText(segment.EndOffset, 2) == "//")
					doc.Replace(segment.EndOffset, 2, string.Empty);
			}
		}

		/// <summary></summary>
		public event PropertyChangedEventHandler? PropertyChanged;
		private void NotifyPropertyChanged(string prop_name) => PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(prop_name));

		/// <summary>Character class for Ldr Object types</summary>
		private static bool IsLdrObjectTypeChar(char ch)
		{
			return char.IsLetterOrDigit(ch) || ch == '*' || ch == '_';
		}

		/// <summary>Auto completion item</summary>
		private class CompletionItem :ICompletionData
		{
			private readonly Action<CompletionItem> m_on_complete;
			public CompletionItem(View3d.AutoComplete.Template template, string parent_address, int start_offset, Action<CompletionItem> on_complete)
			{
				Template = template;
				LdrParentAddress = parent_address;
				StartOffset = start_offset;
				m_on_complete = on_complete;
			}
			public View3d.AutoComplete.Template Template { get; }
			public int StartOffset { get; }
			public string LdrParentAddress { get; }
			public ImageSource? Image => null;
			public string Text => $"*{Template.Keyword}";
			public object Content => Text;
			public object? Description => null;
			public double Priority => 1.0;
			public void Complete(TextArea text_area, ISegment completionSegment, EventArgs e) => m_on_complete(this);
		}

		/// <summary>Indentation strategy for the script editor</summary>
		private class IndentStrategy :IIndentationStrategy
		{
			private readonly TextEditor m_editor;
			public IndentStrategy(TextEditor editor)
			{
				m_editor = editor;
			}
			public void IndentLine(TextDocument document, DocumentLine line)
			{
				if (line.PreviousLine != null)
				{
					// Read the indent from the previous line
					var prev_indent_seg = TextUtilities.GetLeadingWhitespace(document, line.PreviousLine);

					// Get the current indentation from 'line'
					var curr_indent_seg = TextUtilities.GetLeadingWhitespace(document, line);
					
					// The indent text from the previous line
					var indent = document.GetText(prev_indent_seg);

					//// If the previous line starts with '{', add one indent
					//if (prev_indent_seg.EndOffset < document.TextLength &&
					//	document.GetText(prev_indent_seg.EndOffset, 1) == "{")
					//	indent += m_editor.Options.IndentationString;
					//
					//// If the current line starts with '}', remove one indent
					//if (curr_indent_seg.EndOffset < document.TextLength && 
					//	document.GetText(curr_indent_seg.EndOffset, 1) == "}" &&
					//	indent.EndsWith(m_editor.Options.IndentationString))
					//	indent = indent.Substring(0, indent.Length - m_editor.Options.IndentationString.Length);

					// Copy the indentation to 'line'. 'OffsetChangeMappingType.RemoveAndInsert' guarantees the caret moves behind the new indentation.
					document.Replace(curr_indent_seg.Offset, curr_indent_seg.Length, indent, OffsetChangeMappingType.RemoveAndInsert);
				}
			}
			public void IndentLines(TextDocument document, int line_beg, int line_end)
			{
				for (var i = line_beg; i != line_end; ++i)
				{
					var line = document.GetLineByNumber(i);
					IndentLine(document, line);
				}
			}
		}
	}
}
