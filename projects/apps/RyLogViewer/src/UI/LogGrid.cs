using System;
using System.Globalization;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Media;
using Rylogic.Common;
using Rylogic.Gui.WPF;
using RyLogViewer.Options;

namespace RyLogViewer
{
	public class LogGrid : DataGrid, IDockable
	{
		private static readonly LineToColumnConverter s_conv = new();

		public LogGrid()
		{
			DockControl = new DockControl(this, "Log View")
			{
				TabText = "Log View",
				ShowTitle = false,
			};
			AutoGenerateColumns = false;
			EnableRowVirtualization = true;
			EnableColumnVirtualization = true;
			GridLinesVisibility = DataGridGridLinesVisibility.None;
			HeadersVisibility = DataGridHeadersVisibility.None;
			IsReadOnly = true;
			SelectionUnit = DataGridSelectionUnit.FullRow;

			// Context menu
			ContextMenu = new ContextMenu();
			var copy_item = new MenuItem { Header = "Copy", InputGestureText = "Ctrl+C" };
			copy_item.Click += (s, e) => CopySelectedToClipboard();
			ContextMenu.Items.Add(copy_item);

			var copy_line_item = new MenuItem { Header = "Copy Full Line" };
			copy_line_item.Click += (s, e) => CopySelectedLinesToClipboard();
			ContextMenu.Items.Add(copy_line_item);

			ContextMenu.Items.Add(new Separator());

			var bookmark_item = new MenuItem { Header = "Toggle Bookmark" };
			bookmark_item.Click += (s, e) => ToggleBookmarkRequested?.Invoke(this, EventArgs.Empty);
			ContextMenu.Items.Add(bookmark_item);
		}

		/// <summary>IDockable implementation</summary>
		public DockControl DockControl { get; }

		/// <summary>Raised when the user wants to toggle a bookmark on the selected row</summary>
		public event EventHandler? ToggleBookmarkRequested;

		/// <summary>Copy selected cell text to clipboard</summary>
		private void CopySelectedToClipboard()
		{
			if (SelectedItem is ILine line)
				Clipboard.SetText(line.Value(0));
		}

		/// <summary>Copy all selected lines to clipboard</summary>
		private void CopySelectedLinesToClipboard()
		{
			var sb = new System.Text.StringBuilder();
			foreach (var item in SelectedItems)
			{
				if (item is ILine line)
					sb.AppendLine(line.Value(0));
			}
			if (sb.Length > 0)
				Clipboard.SetText(sb.ToString());
		}

		/// <summary>The highlight patterns to apply to rows</summary>
		public HighLightContainer? Highlights { get; set; }

		/// <summary>Apply highlight colours to rows as they are loaded</summary>
		protected override void OnLoadingRow(DataGridRowEventArgs e)
		{
			base.OnLoadingRow(e);
			if (e.Row.Item is not ILine line || Highlights == null || Highlights.Count == 0)
			{
				e.Row.ClearValue(DataGridRow.BackgroundProperty);
				e.Row.ClearValue(DataGridRow.ForegroundProperty);
				return;
			}

			// Get the text of the line (column 0)
			var text = line.Value(0);

			// Test each highlight pattern
			foreach (var hl in Highlights)
			{
				if (!hl.Active || !hl.IsValid) continue;
				if (!hl.IsMatch(text)) continue;

				e.Row.Background = new SolidColorBrush(hl.BackColour);
				e.Row.Foreground = new SolidColorBrush(hl.ForeColour);
				return;
			}

			// No highlight match — use default colours from settings
			if (Settings != null && Settings.Format.AlternateLineColours)
			{
				var idx = e.Row.GetIndex();
				e.Row.Background = new SolidColorBrush(idx % 2 == 0 ? Settings.Format.LineBackColour1 : Settings.Format.LineBackColour2);
				e.Row.Foreground = new SolidColorBrush(idx % 2 == 0 ? Settings.Format.LineForeColour1 : Settings.Format.LineForeColour2);
			}
			else
			{
				e.Row.ClearValue(DataGridRow.BackgroundProperty);
				e.Row.ClearValue(DataGridRow.ForegroundProperty);
			}
		}

		/// <summary>Application settings</summary>
		public Settings Settings
		{
			get;
			set
			{
				if (field == value) return;
				if (field != null)
				{
					field.SettingChange -= HandleSettingChange;
				}
				field = value;
				if (field != null)
				{
					field.SettingChange += HandleSettingChange;

					// Apply settings
					ColumnCount = field.Format.ColumnCount;
				}

				// Handlers
				void HandleSettingChange(object? sender, SettingChangeEventArgs e)
				{
					if (e.Before) return;
					if (e.Key == nameof(Format.ColumnCount) && e.Value is int column_count)
						ColumnCount = column_count;
				}
			}
		} = null!;

		/// <summary>Get/Set the number of displayed columns</summary>
		public int ColumnCount
		{
			get { return Columns.Count; }
			set
			{
				Columns.Clear();
				for (var i = 0; i != value; ++i)
				{
					// Bind to the row item (ILine) directly. The converter extracts the column value.
					Columns.Add(new DataGridTextColumn
					{
						Header = $"Column {i}",
						Binding = new Binding(".")
						{
							Converter = s_conv,
							ConverterParameter = i,
							Mode = BindingMode.OneWay,
						},
						Width = i == 0 ? new DataGridLength(1, DataGridLengthUnitType.Star) : DataGridLength.Auto,
					});
				}
				HeadersVisibility = Columns.Count > 1
					? DataGridHeadersVisibility.Column
					: DataGridHeadersVisibility.None;
			}
		}

		/// <summary>Converter that extracts a column value from an ILine row item</summary>
		private class LineToColumnConverter : IValueConverter
		{
			public object Convert(object value, Type targetType, object parameter, CultureInfo culture)
			{
				if (value is ILine line && parameter is int column)
					return line.Value(column);

				return string.Empty;
			}

			public object ConvertBack(object value, Type targetType, object parameter, CultureInfo culture)
			{
				throw new NotImplementedException();
			}
		}
	}
}
