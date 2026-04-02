using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Diagnostics;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using Rylogic.Common;
using Rylogic.Extn;
using Rylogic.Gui.WPF;
using Rylogic.Utility;

namespace LDraw.UI
{
	public partial class SourcesListUI : UserControl, IDockable, IDisposable, INotifyPropertyChanged
	{
		public SourcesListUI(Model model)
		{
			InitializeComponent();
			DockControl = new DockControl(this, $"SourcesList")
			{
				ShowTitle = false,
				TabText = "Sources",
				DestroyOnClose = false,
			};
			Model = model;

			AddSource = Command.Create(this, AddSourceInternal);
			OpenInEditor = Command.Create(this, OpenInEditorInternal, OpenInEditorAvailable);
			OpenInExternalEditor = Command.Create(this, OpenInExternalEditorInternal, OpenInExternalEditorAvailable);
			DataContext = this;
		}
		public void Dispose()
		{
			Sources = null!;
			Model = null!;
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
				Util.Dispose(ref field!);
				field = value;
			}
		} = null!;

		/// <summary>App logic</summary>
		private Model Model
		{
			get;
			set
			{
				if (field == value) return;
				if (field != null)
				{
					field.SourcesChanged -= HandleSourcesChanged;
				}
				field = value;
				if (field != null)
				{
					field.SourcesChanged += HandleSourcesChanged;
				}

				void HandleSourcesChanged(object? sender, EventArgs args)
				{
					var source_list = (List<SourceItemUI>)Sources.SourceCollection;
					source_list.SyncStable(Model.Sources, (l,r) => l.ContextId == r.Source.ContextId, (s,i) => new SourceItemUI(s));
					NotifyPropertyChanged(nameof(Sources));
					Sources.Refresh();
				}
			}
		} = null!;

		/// <summary>The loaded sources</summary>
		public ICollectionView Sources
		{
			get;
			set
			{
				if (field == value) return;
				if (field != null)
				{
					field.CurrentChanged -= HandleCurrentSelectionChanged;
				}
				field = value;
				if (field != null)
				{
					field.CurrentChanged += HandleCurrentSelectionChanged;
				}
				
				void HandleCurrentSelectionChanged(object? sender, EventArgs e)
				{
					OpenInEditor.NotifyCanExecuteChanged();
					OpenInExternalEditor.NotifyCanExecuteChanged();
				}
			}
		} = new ListCollectionView(new List<SourceItemUI>());

		/// <summary></summary>
		public Command AddSource { get; }
		private void AddSourceInternal()
		{
			if (Window.GetWindow(this) is MainWindow main_window)
				main_window.AddFileSource(null);
		}

		/// <summary>Open the selected source in an editor (if possible)</summary>
		public Command OpenInEditor { get; }
		private bool OpenInEditorAvailable()
		{
			return
				Sources.CurrentItem is SourceItemUI item &&
				item.Source.CanEdit;
		}
		private void OpenInEditorInternal()
		{
			try
			{
				if (Window.GetWindow(this) is MainWindow main_window &&
					Sources.CurrentItem is SourceItemUI item &&
					item.Source.CanEdit)
				{
					main_window.OpenInEditor(item.Source);
				}
			}
			catch (Exception ex)
			{
				Log.Write(ELogLevel.Info, ex, "Open-in-editor for this file source failed.");
				MsgBox.Show(Window.GetWindow(this), $"Open-in-editor for this file source failed.\n{ex.Message}", Util.AppProductName, MsgBox.EButtons.OK, MsgBox.EIcon.Information);
			}
		}

		/// <summary>Open the selected source in an external text editor</summary>
		public Command OpenInExternalEditor { get; }
		private bool OpenInExternalEditorAvailable()
		{
			return
				Sources.CurrentItem is SourceItemUI item &&
				item.Source.FilePath.Length != 0 &&
				Path_.FileExists(item.Source.FilePath);
		}
		private void OpenInExternalEditorInternal()
		{
			try
			{
				if (Sources.CurrentItem is not SourceItemUI item || item.Source.FilePath.Length == 0)
					return;

				var editor_path = Model.Profile.TextEditorPath;
				if (!string.IsNullOrEmpty(editor_path))
				{
					var arguments = Model.Profile.TextEditorArguments
						.Replace("{file}", item.Source.FilePath)
						.Replace("{line}", "1");

					Process.Start(new ProcessStartInfo
					{
						FileName = editor_path,
						Arguments = arguments,
						UseShellExecute = false,
					});
				}
				else
				{
					// Fall back to the OS default application for this file type
					Process.Start(new ProcessStartInfo
					{
						FileName = item.Source.FilePath,
						UseShellExecute = true,
					});
				}
			}
			catch (Exception ex)
			{
				Log.Write(ELogLevel.Error, ex, "Failed to launch text editor");
			}
		}

		/// <summary>Show a copy cursor when dragging files over the sources list</summary>
		private void HandleDragOver(object sender, DragEventArgs e)
		{
			e.Effects = e.Data.GetDataPresent(DataFormats.FileDrop)
				? DragDropEffects.Copy
				: DragDropEffects.None;
			e.Handled = true;
		}

		/// <summary>Add dropped files as sources</summary>
		private void HandleDrop(object sender, DragEventArgs e)
		{
			if (e.Data.GetDataPresent(DataFormats.FileDrop) &&
				e.Data.GetData(DataFormats.FileDrop) is string[] files &&
				Window.GetWindow(this) is MainWindow main_window)
			{
				foreach (var file in files)
					main_window.AddFileSource(file);

				e.Handled = true;
			}
		}

		/// <inheritdoc/>
		public event PropertyChangedEventHandler? PropertyChanged;
		private void NotifyPropertyChanged(string prop_name) => PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(prop_name));
	}
}