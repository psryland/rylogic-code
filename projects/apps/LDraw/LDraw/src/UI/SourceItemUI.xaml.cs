using System;
using System.ComponentModel;
using System.Windows;
using System.Windows.Controls;
using Rylogic.Gui.WPF;
using Rylogic.Utility;

namespace LDraw.UI
{
	public partial class SourceItemUI : UserControl, IDisposable, INotifyPropertyChanged
	{
		public SourceItemUI(Source source)
		{
			InitializeComponent();
			Source = source;

			RemoveSource = Command.Create(this, RemoveSourceInternal);
			DataContext = this;
		}
		public void Dispose()
		{
			GC.SuppressFinalize(this);
		}

		/// <summary>The source this item represents</summary>
		public Source Source { get; }

		/// <summary>Remove this source from the Model</summary>
		public Command RemoveSource { get; }
		private void RemoveSourceInternal()
		{
			try
			{
				Source.Remove();
			}
			catch (Exception ex)
			{
				Log.Write(ELogLevel.Info, ex, "Error removing source");
				MsgBox.Show(Window.GetWindow(this), $"Error removing source.\n{ex.Message}", Util.AppProductName, MsgBox.EButtons.OK, MsgBox.EIcon.Information);
			}
		}

		/// <inheritdoc/>
		public event PropertyChangedEventHandler? PropertyChanged;
		private void NotifyPropertyChanged(string prop_name) => PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(prop_name));
	}
}
