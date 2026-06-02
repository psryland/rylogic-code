using System;
using System.ComponentModel;
using System.IO;
using System.Windows.Input;
using Rylogic.Gui.WPF;

namespace LDraw
{
	public class ParsingProgressData :INotifyPropertyChanged
	{
		public ParsingProgressData(Guid context_id, Action? cancel_action = null)
		{
			ContextId = context_id;
			CancelAction = cancel_action;
			CancelCommand = Command.Create(this, () => CancelAction?.Invoke(), () => CanCancel);
		}

		/// <summary>The context id associated with the parsing progress</summary>
		public Guid ContextId { get; }

		/// <summary>Action to invoke to cancel the current load</summary>
		public Action? CancelAction { get; }

		/// <summary>True if cancellation is supported</summary>
		public bool CanCancel => CancelAction != null;

		/// <summary>Command to cancel the current loading operation</summary>
		public Command CancelCommand { get; }

		/// <summary>The name of the file/script currently being parsed</summary>
		public string DataSourceName
		{
			get => m_data_source_name ?? "Script";
			set
			{
				if (m_data_source_name == value) return;
				m_data_source_name = value;
				NotifyPropertyChanged(nameof(DataSourceName));
				NotifyPropertyChanged(nameof(DisplayName));
			}
		}
		private string? m_data_source_name;

		/// <summary>Display name for the status bar (includes "Loading:" prefix and percentage)</summary>
		public string DisplayName => DataLength != 0
			? $"Loading: {Path.GetFileName(DataSourceName)}... ({Percentage:F0}%)"
			: $"Loading: {Path.GetFileName(DataSourceName)}...";

		/// <summary>The length of the data being parsed</summary>
		public long DataLength
		{
			get;
			set
			{
				if (field == value) return;
				field = value;
				NotifyPropertyChanged(nameof(Percentage));
				NotifyPropertyChanged(nameof(IsIndeterminate));
				NotifyPropertyChanged(nameof(DisplayName));
			}
		}

		/// <summary>Current position within the file</summary>
		public long DataOffset
		{
			get;
			set
			{
				if (field == value) return;
				field = value;
				NotifyPropertyChanged(nameof(Percentage));
				NotifyPropertyChanged(nameof(DisplayName));
			}
		}

		/// <summary>Percentage progress through the file</summary>
		public double Percentage => DataLength != 0 ? 100.0 * DataOffset / DataLength : 100.0 * (DataOffset % 0x1000) / 0x1000;

		/// <summary>True if we don't know the upper bound</summary>
		public bool IsIndeterminate => DataLength == 0;

		/// <inheritdoc />
		public event PropertyChangedEventHandler? PropertyChanged;
		private void NotifyPropertyChanged(string prop_name) => PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(prop_name));
	}
}
