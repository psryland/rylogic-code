using System;
using System.Collections;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Reflection;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Controls.Primitives;
using System.Windows.Data;
using System.Windows.Input;
using System.Windows.Media;
using Rylogic.Common;
using Rylogic.Container;
using Rylogic.Extn;
using Rylogic.Maths;
using Rylogic.Utility;

namespace Rylogic.Gui.WPF
{
	public static class DataGrid_
	{
		// Notes:

		// HowTos:
		//  - Double click a row:
		//    Create a row style like this and set it as the RowStyle on the grid.
		//       <Style x:Key="StyleRow" TargetType="DataGridRow">
		//            <EventSetter Event = "MouseDoubleClick" Handler="HandleDoubleClick" />
		//       </Style>

		/// <summary>Grid address</summary>
		[DebuggerDisplay("[{RowIndex}, {ColIndex}]")]
		public struct Address
		{
			public int RowIndex;
			public int ColIndex;
			public Address(int r, int c)
			{
				RowIndex = r;
				ColIndex = c;
			}
		}

		/// <summary>Return the grid that owns this column</summary>
		public static DataGrid? Grid(this DataGridColumn col)
		{
			m_pi_column_owner ??= typeof(DataGridColumn).GetProperty("DataGridOwner", BindingFlags.Instance | BindingFlags.NonPublic) ?? throw new Exception("DataGridOwner property not found");
			return (DataGrid?)m_pi_column_owner.GetValue(col, null);
		}
		private static PropertyInfo? m_pi_column_owner;

		/// <summary>Return the grid that owns this cell info</summary>
		public static DataGrid? Grid(this DataGridCellInfo ci)
		{
			m_pi_cellinfo_owner ??= typeof(DataGridCellInfo).GetProperty("Owner", BindingFlags.Instance | BindingFlags.NonPublic) ?? throw new Exception("Owner property not found");
			return (DataGrid?)m_pi_cellinfo_owner.GetValue(ci, null);
		}
		private static PropertyInfo? m_pi_cellinfo_owner;

		/// <summary>Return the cell associated with a 'cell info' (i.e. SelectedCell)</summary>
		public static DataGridCell? Cell(this DataGridCellInfo ci)
		{
			var cell_content = ci.Column.GetCellContent(ci.Item);
			return (DataGridCell?)cell_content?.Parent ?? null;
		}
		public static DataGridCell? Cell(this DataGridRow row, int index)
		{
			m_mi_TryGetCell ??= typeof(DataGridRow).GetMethod("TryGetCell", BindingFlags.Instance | BindingFlags.NonPublic) ?? throw new Exception("TryGetCell method not found");
			return (DataGridCell?)m_mi_TryGetCell.Invoke(row, new object[] { index });
		}
		private static MethodInfo? m_mi_TryGetCell;

		/// <summary>Return cell info for this cell</summary>
		public static DataGridCellInfo Info(this DataGridCell cell)
		{
			return new DataGridCellInfo(cell);
		}

		/// <summary>Returns the cell that contains 'dp' (or null)</summary>
		public static DataGridCell? FindCell(DependencyObject dp)
		{
			return Gui_.FindVisualParent<DataGridCell>(dp);
		}

		/// <summary>Returns the row that contains this cell</summary>
		public static DataGridRow GetRow(this DataGridCell cell)
		{
			return Gui_.FindVisualParent<DataGridRow>(cell) ?? throw new Exception("Cell does not have an associated row");
		}
		public static DataGridRow GetRow(this DataGridCellInfo ci)
		{
			return ci.Column.Grid()?.ItemContainerGenerator.ContainerFromItem(ci.Item) is DataGridRow row ? row : throw new Exception("Cell info does not correspond to a row in the grid");
		}

		/// <summary>Get the grid row from a cell info</summary>
		public static int GetRowIndex(this DataGridCellInfo ci)
		{
			m_pi_cellinfo_iteminfo ??= typeof(DataGridCellInfo).GetProperty("ItemInfo", BindingFlags.Instance | BindingFlags.NonPublic) ?? throw new Exception("ItemInfo property not found");
			m_pi_iteminfo_indx ??= m_pi_cellinfo_iteminfo.PropertyType.GetProperty("Index", BindingFlags.Instance | BindingFlags.NonPublic) ?? throw new Exception("Index property not found");
			var ii = m_pi_cellinfo_iteminfo.GetValue(ci, null) ?? throw new Exception("ItemInfo property not found");
			var idx = m_pi_iteminfo_indx.GetValue(ii, null) ?? throw new Exception("Index property not found");
			return (int)idx;
		}
		private static PropertyInfo? m_pi_cellinfo_iteminfo;
		private static PropertyInfo? m_pi_iteminfo_indx;

		/// <summary>Returns the cell and column index of this cell</summary>
		public static Address GridAddress(this DataGridCell cell)
		{
			return new Address
			{
				ColIndex = cell.Column.DisplayIndex,
				RowIndex = cell.GetRow().GetIndex(),
			};
		}
		public static Address GridAddress(this DataGridCellInfo ci)
		{
			return new Address
			{
				ColIndex = ci.Column.DisplayIndex,
				RowIndex = ci.GetRowIndex(),
			};
		}

		/// <summary>Bounding ranges for the selected cells</summary>
		public static (RangeI, RangeI) SelectedBounds(this DataGrid grid)
		{
			var parameters = new object[4];
			m_mi_GetSelectionRange ??= grid.SelectedCells.GetType().GetMethod("GetSelectionRange", BindingFlags.Instance | BindingFlags.NonPublic) ?? throw new Exception("GetSelectionRange method not found");
			m_mi_GetSelectionRange.Invoke(grid.SelectedCells, parameters);
			var col_bounds = new RangeI((int)parameters[0], (int)parameters[1] + 1); // Exclusive range
			var row_bounds = new RangeI((int)parameters[2], (int)parameters[3] + 1); // Exclusive range
			return (row_bounds, col_bounds);
		}
		private static MethodInfo? m_mi_GetSelectionRange;

		/// <summary>True if the given row/col address is within the grid</summary>
		public static bool IsWithin(this DataGrid grid, int r, int c)
		{
			return 
				r >= 0 && r < grid.Items.Count &&
				c >= 0 && c < grid.Columns.Count;
		}
		public static bool IsWithin(this DataGrid grid, Address addr) => IsWithin(grid, addr.RowIndex, addr.ColIndex);

		/// <summary>Attempt to set the value of a cell</summary>
		public static void SetCellValue(DataGrid grid, int r, int c, object? value)
		{
			// Note:
			// - There is no 'UpdateBinding' call after setting the value because
			//   it is expected that the bound property will notify of property changed.

			switch (grid.Columns[c])
			{
				// If the column is bound, attempt to set the value via the binding
				case DataGridBoundColumn col0:
				{
					if (col0.Binding is Binding binding &&
						grid.Items[r] is object item &&
						binding.Mode.HasFlag(BindingMode.TwoWay) &&
						item.GetType().GetProperty(binding.Path.Path) is PropertyInfo pi)
					{
						value = Util.ConvertTo(value, pi.PropertyType);
						pi.SetValue(item, value);
					}
					break;
				}
				case DataGridBoundTemplateColumn col1:
				{
					if (col1.Binding is Binding binding &&
						grid.Items[r] is object item &&
						binding.Mode.HasFlag(BindingMode.TwoWay) &&
						item.GetType().GetProperty(binding.Path.Path) is PropertyInfo pi)
					{
						value = Util.ConvertTo(value, pi.PropertyType);
						pi.SetValue(item, value);
					}
					break;
				}
				default:
				{
					throw new Exception("Unknown DataGrid column type");
				}
			}
		}
		public static void SetCellValue(DataGrid grid, Address addr, object? value) => SetCellValue(grid, addr.RowIndex, addr.ColIndex, value);

		/// <summary>Return the value of this cell</summary>
		public static object? GetCellValue(this DataGridCell cell)
		{
			var item = cell.Info().Item;
			var content = cell.Column.GetCellContent(item);
			if (content is FrameworkElement fe && content.DataContext is DataRowView view)
				return view.Row.ItemArray[0];
			
			return content;
		}

		/// <summary>Parse a string representation of a DataGridLength</summary>
		public static DataGridLength ParseDataGridLength(string length)
		{
			if (length.ToLower() == "auto")
				return new DataGridLength(1.0, DataGridLengthUnitType.Auto);
			if (length.ToLower() == "sizetocells")
				return new DataGridLength(1.0, DataGridLengthUnitType.SizeToCells);
			if (length.ToLower() == "sizetoheader")
				return new DataGridLength(1.0, DataGridLengthUnitType.SizeToHeader);
			if (length.ToLower() == "*")
				return new DataGridLength(1.0, DataGridLengthUnitType.Star);
			if (length.EndsWith("*"))
				return new DataGridLength(double.Parse(length.TrimEnd('*')), DataGridLengthUnitType.Star);
			if (double.TryParse(length, out var pixels))
				return new DataGridLength(pixels, DataGridLengthUnitType.Pixel);
			throw new Exception($"Failed to parse DataGridLength string: {length}");
		}

		#region Cut Copy Paste
		
		// Usage:
		//   Add 'gui:DataGrid_.CopyPasteSupport="True"' to your DataGrid.

		private const int CopyPasteSupport = 0;
		public static readonly DependencyProperty CopyPasteSupportProperty = Gui_.DPRegisterAttached(typeof(DataGrid_), nameof(CopyPasteSupport), Boxed.False, Gui_.EDPFlags.None);
		public static bool GetCopyPasteSupport(DependencyObject obj) => (bool)obj.GetValue(CopyPasteSupportProperty);
		public static void SetCopyPasteSupport(DependencyObject obj, bool value) => obj.SetValue(CopyPasteSupportProperty, value);
		private static void CopyPasteSupport_Changed(DependencyObject obj)
		{
			if (obj is not DataGrid grid)
				return;

			if (GetCopyPasteSupport(obj))
			{
				var copy = new RoutedCommand(nameof(CopyPasteSupport) + "_DoCopy", typeof(DataGrid));
				copy.InputGestures.Add(new KeyGesture(Key.C, ModifierKeys.Control));
				grid.CommandBindings.Add(new CommandBinding(copy, DoCopyInternal));

				var paste = new RoutedCommand(nameof(CopyPasteSupport) + "_DoPaste", typeof(DataGrid));
				paste.InputGestures.Add(new KeyGesture(Key.V, ModifierKeys.Control));
				grid.CommandBindings.Add(new CommandBinding(paste, DoPasteInternal));
			}
			else
			{
				// Remove the bindings added by this extension
				var bindings = grid.CommandBindings.OfType<CommandBinding>().Where(x => x.Command is RoutedCommand rcmd && rcmd.Name.StartsWith(nameof(CopyPasteSupport))).ToList();
				foreach (var b in bindings)
					grid.CommandBindings.Remove(b);
			}

			const uint CLIPBRD_E_CANT_OPEN = 0x800401D0;
			const uint CLIPBRD_E_BAD_DATA = 0x800401d3;

			// Implement the clipboard operations
			static void DoCopyInternal(object? sender, ExecutedRoutedEventArgs e)
			{
				if (sender is not DataGrid grid)
					return;

				// Copy the selected cells
				for (int attempt = 0; ; ++attempt, Thread.Yield())
				{
					try
					{
						var csv = SelectedToCSV(grid);

						var obj = new DataObject();
						obj.SetText(csv.Export(false));
						Clipboard.SetDataObject(obj, true);
						break;
					}
					catch (COMException ex)
					{
						var code = unchecked((uint)ex.ErrorCode);
						if (code != CLIPBRD_E_CANT_OPEN) throw;
						if (attempt == 3)
						{
							MsgBox.Show(Window.GetWindow(grid), "Clipboard is unavailable. Another process has it locked.", "Clipboard Error", MsgBox.EButtons.OK, MsgBox.EIcon.Error);
							return;
						}
					}
				}

				e.Handled = true;
			}
			static void DoPasteInternal(object? sender, ExecutedRoutedEventArgs e)
			{
				if (sender is not DataGrid grid || grid.IsReadOnly)
					return;

				// Extract a table of CSV data from the clipboard
				var csv = (CSVData?)null;
				for (var attempt = 0; ; ++attempt, Thread.Yield())
				{
					try
					{
						var data = Clipboard.GetDataObject();
						if (data != null && data.GetDataPresent(typeof(string)) && data.GetData(typeof(string)) is string str)
						{
							var ms = new MemoryStream(Encoding.UTF8.GetBytes(str), false);
							csv = CSVData.Load(ms, true);
							break;
						}
						break;
					}
					catch (COMException ex)
					{
						var code = unchecked((uint)ex.ErrorCode);
						if (code == CLIPBRD_E_BAD_DATA) return;
						if (code != CLIPBRD_E_CANT_OPEN) throw;
						if (attempt == 3)
						{
							MsgBox.Show(Window.GetWindow(grid), "Clipboard is unavailable. Another process has it locked.", "Clipboard Error", MsgBox.EButtons.OK, MsgBox.EIcon.Error);
							return;
						}
					}
				}
				if (csv == null)
					return;

				e.Handled = true;

				// Map the data to the selected cells
				var (rows, cols) = grid.SelectedBounds();

				// Automatically transpose the clipboard data
				var transpose =
					(cols.Count == 1 && rows.Count > 1 && csv.RowCount == 1 && csv.ColCount > 1) ||
					(cols.Count > 1 && rows.Count == 1 && csv.RowCount > 1 && csv.ColCount == 1);

				var errors = new List<Exception>();

				// If a single cell is selected, paste the given layout over the writeable cells,
				// ignoring layout, and clip to the bounds of the grid.
				if (cols.Count == 1 && rows.Count == 1)
				{
					// Iterate over the clipboard data
					for (var r = 0; r != csv.Rows.Count; ++r)
					{
						for (var c = 0; c != csv.Rows[r].Count; ++c)
						{
							var addr = new Address(rows.Begi + r, cols.Begi + c);
							if (grid.IsWithin(addr))
							{
								try { SetCellValue(grid, addr, csv[r, c]); }
								catch (Exception ex) { errors.Add(ex); }
							}
						}
					}
				}
				// Otherwise, only write over the selected writeable cells
				else
				{
					// Iterate over the selected cells
					foreach (var ci in grid.SelectedCells)
					{
						var addr = ci.GridAddress();
						var r = addr.RowIndex - rows.Begi;
						var c = addr.ColIndex - cols.Begi;
						if (transpose) Math_.Swap(ref r, ref c);
						if (csv.IsWithin(r, c))
						{
							try { SetCellValue(grid, addr, csv[r, c]); }
							catch (Exception ex) { errors.Add(ex); }
						}
					}
				}

				// Report if errors occurred
				if (errors.Count != 0)
				{
					MsgBox.Show(Window.GetWindow(grid),
						$"Some values could not be pasted due to errors",
						"Clipboard Errors", MsgBox.EButtons.OK, MsgBox.EIcon.Error);
				}
			}
			static CSVData SelectedToCSV(DataGrid grid)
			{
				// Find the cell range of selected cells
				var (rows, cols) = grid.SelectedBounds();

				// Create a mapping from column index to visible column index
				var idx = 0;
				var col_index_map = new int[cols.Sizei];
				for (var i = 0; i != col_index_map.Length; ++i)
					col_index_map[i] = grid.Columns[i].Visibility == Visibility.Visible ? idx++ : -1;

				// Create a table of CSV data, populated from the visible selected cells
				var csv = new CSVData { AutoSize = true };
				foreach (var ci in grid.SelectedCells.Where(x => x.Column.Visibility == Visibility.Visible))
				{
					Binding? binding =
						ci.Column is DataGridBoundColumn col0 ? (Binding?)col0.Binding :
						ci.Column is DataGridBoundTemplateColumn col1 ? col1.Binding :
						ci.Column is TreeGridColumn col2 ? (Binding?)col2.Binding :
						null;

					if (binding != null)
					{
						var addr = ci.GridAddress();
						csv[addr.RowIndex - rows.Begi, col_index_map[addr.ColIndex]] = binding.Eval(ci.Item)?.ToString() ?? string.Empty;
					}
				}
				return csv;
			}
		}

		#endregion

		#region Auto Column Size

		// Usage:
		//   In your xaml set 'DataGrid_.ColumnResizeMode' and 'NotifyOnTargetUpdated=True'
		//   in the column text binding on all columns that can change.
		//    e.g.
		//      <DataGrid
		//          local:DataGrid_.ColumnResizeMode="FitToDisplayWidth"
		//          ... />
		//          <DataGridTextColumn
		//              Header = "MyColumn"
		//              Binding="{Binding TextToShow, NotifyOnTargetUpdated=True}"
		//              />

		/// <summary>Column resizing schemes</summary>
		public enum EColumnResizeMode
		{
			None,
			FitToDisplayWidth,
		}

		/// <summary>Column resizing attached property</summary>
		private const int ColumnResizeMode = 0;
		public static readonly DependencyProperty ColumnResizeModeProperty = Gui_.DPRegisterAttached(typeof(DataGrid_), nameof(ColumnResizeMode), EColumnResizeMode.None, Gui_.EDPFlags.None);
		public static EColumnResizeMode GetColumnResizeMode(DependencyObject obj) => (EColumnResizeMode)obj.GetValue(ColumnResizeModeProperty);
		public static void SetColumnResizeMode(DependencyObject obj, EColumnResizeMode value) => obj.SetValue(ColumnResizeModeProperty, value);
		private static void ColumnResizeMode_Changed(DependencyObject obj)
		{
			if (!(obj is DataGrid grid))
				return;

			grid.SizeChanged -= ResizeColumns;
			grid.TargetUpdated -= ResizeColumns;
			if (GetColumnResizeMode(obj) != EColumnResizeMode.None)
			{
				grid.SizeChanged += ResizeColumns;
				grid.TargetUpdated += ResizeColumns;
			}

			// Handler
			static void ResizeColumns(object? sender, EventArgs? e)
			{
				if (!(sender is DataGrid grid)) return;
				var widths = grid.Columns.Select(x => x.Width).ToList();

				var total_width = grid.ActualWidth;
				if (double.IsNaN(total_width))
					return;

				foreach (var (col, i) in grid.Columns.WithIndex())
				{
					if (widths[i].UnitType == DataGridLengthUnitType.Pixel)
						col.Width = widths[i];
					else
						col.Width = new DataGridLength(widths[i].Value, DataGridLengthUnitType.Star);
				}
			}
		}

		#endregion

		#region Column Visibility

		// Usage:
		//  Add 'gui:DataGrid_.ColumnVisibilitySupport="True"' to your DataGrid.

		/// <summary>Add column visibility to a DataGrid attached property</summary>
		private const int ColumnVisibilitySupport = 0;
		public static readonly DependencyProperty ColumnVisibilitySupportProperty = Gui_.DPRegisterAttached(typeof(DataGrid_), nameof(ColumnVisibilitySupport), Boxed.False, Gui_.EDPFlags.None);
		public static bool GetColumnVisibilitySupport(DependencyObject obj) => (bool)obj.GetValue(ColumnVisibilitySupportProperty);
		public static void SetColumnVisibilitySupport(DependencyObject obj, bool value) => obj.SetValue(ColumnVisibilitySupportProperty, value);
		private static void ColumnVisibilitySupport_Changed(DependencyObject obj)
		{
			if (obj is not DataGrid grid)
				return;

			grid.MouseRightButtonUp -= ColumnVisibility;
			if (GetColumnVisibilitySupport(obj))
				grid.MouseRightButtonUp += ColumnVisibility;
		}

		/// <summary>Display a context menu for showing/hiding columns in the grid. Attach to 'MouseRightButtonUp'</summary>
		public static void ColumnVisibility(object? sender, MouseButtonEventArgs args)
		{
			// Unfortunately, binding like this 'MouseRightButtonUp="{Binding DataGrid_.ColumnVisibility}"'
			// doesn't work. You'd need to provide an instance method that forwards to this call, so you
			// might as well just sign up the event handler in the constructor:
			//  m_grid.MouseRightButtonUp += DataGrid_.ColumnVisibility;
			if (sender is not DataGrid grid ||
				args.OriginalSource is not DependencyObject hit ||
				args.ChangedButton != MouseButton.Right ||
				args.RightButton != MouseButtonState.Released)
				return;

			// Right mouse on a column header displays a context menu for hiding/showing columns
			var hit_header = Gui_.FindVisualParent<DataGridColumnHeader>(hit, root: grid) != null;
			if (!hit_header)
			{
				// The 'filler' header is not clickable (IsHitTestVisible is false)
				var filler = Gui_.FindVisualChild<DataGridColumnHeader>(grid, x => x.Name == "PART_FillerColumnHeader");
				hit_header = filler?.RenderArea().Contains(args.GetPosition(filler)) ?? false;
			}
			if (hit_header)
			{
				grid.ColumnVisibilityCMenu();
				args.Handled = true;
			}
		}

		/// <summary>Display a context menu for showing/hiding columns in the grid (at 'location' relative to the grid).</summary>
		public static void ColumnVisibilityCMenu(this DataGrid grid, Action<DataGridColumn>? on_vis_changed = null)
		{
			var cmenu = new ContextMenu();
			foreach (var col in grid.Columns.Cast<DataGridColumn>())
			{
				var item = cmenu.Items.Add2(new MenuItem
				{
					Header = CopyHeader(col.Header),
					IsChecked = col.Visibility == Visibility.Visible,
					Tag = col
				});
				item.Click += (s, a) =>
				{
					var c = (DataGridColumn)item.Tag;
					c.Visibility = item.IsChecked ? Visibility.Collapsed : Visibility.Visible;
					on_vis_changed?.Invoke(c);
				};
			}
			cmenu.PlacementTarget = grid;
			cmenu.StaysOpen = true;
			cmenu.IsOpen = true;

			// Make a copy of the header to use in the context menu
			static object CopyHeader(object? hdr)
			{
				if (hdr is string str)
					return str;
				if (hdr is TextBlock tb)
					return tb.Text;
				if (hdr is Image img)
					return new Image { Source = img.Source, MaxHeight = 18 };
				return "<no heading>";
			}
		}

		#endregion

		#region Arrows Commit Edits

		// Usage:
		//  Add 'gui:DataGrid_.ArrowsCommitEdits="True"'

		private const int ArrowsCommitEdits = 0;
		public static readonly DependencyProperty ArrowsCommitEditsProperty = Gui_.DPRegisterAttached(typeof(DataGrid_), nameof(ArrowsCommitEdits), Boxed.False, Gui_.EDPFlags.None);
		public static bool GetArrowsCommitEdits(DependencyObject obj) => (bool)obj.GetValue(ArrowsCommitEditsProperty);
		public static void SetArrowsCommitEdits(DependencyObject obj, bool value) => obj.SetValue(ArrowsCommitEditsProperty, value);
		private static void ArrowsCommitEdits_Changed(DependencyObject obj)
		{
			if (obj is not DataGrid grid)
				return;

			grid.BeginningEdit -= HandleCellEditBeg;
			grid.CellEditEnding -= HandleCellEditEnd;
			if (GetArrowsCommitEdits(obj))
			{
				grid.BeginningEdit += HandleCellEditBeg;
				grid.CellEditEnding += HandleCellEditEnd;
			}

			// Handlers
			void HandleCellEditBeg(object? sender, DataGridBeginningEditEventArgs e)
			{
				grid.PreviewKeyDown += HandleKeyDown;
			}
			void HandleCellEditEnd(object? sender, DataGridCellEditEndingEventArgs e)
			{
				grid.PreviewKeyDown -= HandleKeyDown;
			}
			void HandleKeyDown(object? sender, KeyEventArgs e)
			{
				if (e.Key == Key.Up || e.Key == Key.Down || e.Key == Key.PageUp || e.Key == Key.PageDown)
					grid.CommitEdit();
			}
		}

		#endregion

		#region AutoScrollToCurrent

		// Usage:
		//  Add 'gui:DataGrid_.AutoScrollToCurrent="True" to your DataGrid.

		private const int AutoScrollToCurrent = 0;
		public static readonly DependencyProperty AutoScrollToCurrentProperty = Gui_.DPRegisterAttached(typeof(DataGrid_), nameof(AutoScrollToCurrent), Boxed.False, Gui_.EDPFlags.None);
		public static bool GetAutoScrollToCurrent(DependencyObject obj) => (bool)obj.GetValue(AutoScrollToCurrentProperty);
		public static void SetAutoScrollToCurrent(DependencyObject obj, bool value) => obj.SetValue(AutoScrollToCurrentProperty, value);
		private static void AutoScrollToCurrent_Changed(DependencyObject obj)
		{
			var enabled = GetAutoScrollToCurrent(obj);
			if (obj is not DataGrid data_grid)
				return;

			// This doesn't work using DataGrid's CurrentItem. For some reason only
			// grids with focus generate the CurrentCellChanged event. Using the underlying
			// ItemsSource avoids this problem.

			// The ItemsSource may not be assigned yet. Get the ItemSource property so we can see when its value changes.
			if (DependencyPropertyDescriptor.FromProperty(ItemsControl.ItemsSourceProperty, typeof(DataGrid)) is not DependencyPropertyDescriptor items_source_dp)
				return;

			// Flag to batch multiple current changed events
			object scroll_pending = false;

			// Wait for changes to the item source
			items_source_dp.RemoveValueChanged(data_grid, HandleItemsSourceChanged);
			items_source_dp.AddValueChanged(data_grid, HandleItemsSourceChanged);
			
			// Handlers
			void HandleItemsSourceChanged(object? sender, EventArgs e)
			{
				if (sender is not DataGrid data_grid) return;
				if (data_grid.ItemsSource is ICollectionView cv)
				{
					// The old items source will still have the handler subscribed but it will
					// remove itself as soon as it doesn't match the data grid's item source.
					cv.CurrentChanged -= HandleCurrentChanged;
					cv.CurrentChanged += HandleCurrentChanged;
				}
			}
			void HandleCurrentChanged(object? sender, EventArgs e)
			{
				if (sender is not ICollectionView cv) return;
				if (!enabled || data_grid.ItemsSource != cv)
				{
					cv.CurrentChanged -= HandleCurrentChanged;
					return;
				}
				if (scroll_pending is bool sp && sp)
					return;

				// Invoke the scroll on the message queue
				scroll_pending = true;
				data_grid.Dispatcher.BeginInvoke(new Action(() =>
				{
					scroll_pending = false;
					if (cv.CurrentItem != null)
						data_grid.ScrollIntoView(cv.CurrentItem);
				}));
			}
		}

		#endregion

		#region Row Drag Drop

		// Usage:
		//  Add 'gui:DataGrid_.ReorderRowsWithDragDrop="True"' to your DataGrid.
		//   You may want 'CanUserSortColumns="False"' as well because sorting prevents
		//   the default behaviour of reordering the underlying collection.
		//  Alternatively, you can add a handler:
		//   gui:DataGrid_.ReorderRowDrop="HandleReorderRowDrop"
		//   private void HandleReorderRowDrop(object sender, DataGrid_.ReorderRowDropEventArgs e)
		//  You can then handle reordering with custom code.
		//  Use the 'RowsReordered' to get notification after a reorder has happened

		/// <summary>Reorder rows with drag/drop attached property</summary>
		private const int ReorderRowsWithDragDrop = 0;
		public static readonly DependencyProperty ReorderRowsWithDragDropProperty = Gui_.DPRegisterAttached(typeof(DataGrid_), nameof(ReorderRowsWithDragDrop), Boxed.False, Gui_.EDPFlags.None);
		public static bool GetReorderRowsWithDragDrop(DependencyObject obj) => (bool)obj.GetValue(ReorderRowsWithDragDropProperty);
		public static void SetReorderRowsWithDragDrop(DependencyObject obj, bool value) => obj.SetValue(ReorderRowsWithDragDropProperty, value);
		private static void ReorderRowsWithDragDrop_Changed(DependencyObject obj)
		{
			if (!(obj is DataGrid data_grid))
				return;

			var enable = GetReorderRowsWithDragDrop(obj);
			if (enable)
			{
				data_grid.AllowDrop = true;
				data_grid.PreviewMouseLeftButtonDown += HandleMouseDown;
				data_grid.DragEnter += HandleDragging;
				data_grid.DragOver += HandleDragging;
				data_grid.Drop += HandleDrop;
			}
			else
			{
				data_grid.Drop -= HandleDrop;
				data_grid.DragOver += HandleDragging;
				data_grid.DragEnter += HandleDragging;
				data_grid.PreviewMouseLeftButtonDown -= HandleMouseDown;
			}

			// Handlers
			static void HandleDrop(object? sender, DragEventArgs e)
			{
				// Must allow move
				if (!e.AllowedEffects.HasFlag(DragDropEffects.Move))
					return;

				// Ignore if not dropping a row
				var data = (ReorderRowsWithDragDropData)e.Data.GetData(typeof(ReorderRowsWithDragDropData));
				if (data == null)
					return;

				// Ignore if the mouse hasn't moved enough to start dragging
				if (!data.Dragging)
					return;

				// Don't allow drop if not over a row header
				var row_header = e.OriginalSource is DependencyObject src ? Gui_.FindVisualParent<DataGridRowHeader>(src, root: data.Grid) : null;
				if (row_header == null)
					return;

				// Must drop over a row
				var row = Gui_.FindVisualParent<DataGridRow>(row_header, root: data.Grid);
				if (row == null)
					return;

				// Get the index of the insert position
				var row_pt = e.GetPosition(row_header);
				var grab_index = data.Row.GetIndex();
				var drop_index = row.GetIndex() + (2 * row_pt.Y < row_header.ActualHeight ? 0 : 1);

				// Nothing to do if not reordering
				if (grab_index != drop_index)
				{
					// Reorder the items source.
					var args = new ReorderRowDropEventArgs(data.Grid, data.Row, drop_index);
					
					// Call the event to allow user code to handle it
					data.Grid.RaiseEvent(args);

					// Fallback to the default handling
					if (!args.Handled)
						ReorderRowsDropDefaultHandler(data.Grid, args);

					// Notify if reordering happened
					if (args.Handled)
						data.Grid.RaiseEvent(new RoutedEventArgs(RowsReorderedEvent, data.Grid));
				}

				e.Handled = true;
			}
			static void HandleDragging(object? sender, DragEventArgs e)
			{
				e.Effects = DragDropEffects.None;
				e.Handled = true;

				// Must allow move
				if (!e.AllowedEffects.HasFlag(DragDropEffects.Move))
					return;

				// Ignore if not dropping a row
				var data = (ReorderRowsWithDragDropData)e.Data.GetData(typeof(ReorderRowsWithDragDropData));
				if (data == null)
					return;

				// Don't allow drop if not over a row header
				var row_header = e.OriginalSource is DependencyObject src ? Gui_.FindVisualParent<DataGridRowHeader>(src, root: data.Grid) : null;
				if (row_header == null)
					return;

				// The current position in grid space
				var pt = e.GetPosition(data.Grid);

				// Check the mouse has moved enough to start dragging
				if (!data.Dragging)
				{
					// Not moved enough
					if ((pt - data.Grab).LengthSquared < 25) return;
					data.Dragging = true;
				}

				// Set 'Effect' to indicate whether drop is allowed
				e.Effects = DragDropEffects.Move;

				// Update the indicator position
				var row_pt = e.GetPosition(row_header); // The mouse position relative to the hovered row
				var ins_pt = 2 * row_pt.Y < row_header.ActualHeight ? new Point(0, 0) : new Point(0, row_header.ActualHeight); // The insert point relative to the hovered row
				var scn_pt = row_header.PointToScreen(ins_pt);

				data.PositionIndicator(scn_pt);
			}
			static void HandleMouseDown(object? sender, MouseButtonEventArgs e)
			{
				if (!(e.OriginalSource is DependencyObject src)) return;
				if (!(sender is DataGrid grid)) return;

				var row_header = Gui_.FindVisualParent<DataGridRowHeader>(src, root: grid);
				if (row_header != null && grid.Cursor != Cursors.SizeNS)
				{
					grid.Dispatcher.BeginInvoke(new Action(() =>
					{
						using var data = new ReorderRowsWithDragDropData(grid, e.GetPosition(grid));
						DragDrop.DoDragDrop(grid, data, DragDropEffects.Move | DragDropEffects.Copy | DragDropEffects.Link);
					}));
					e.Handled = true;
				}
			}
		}
		private sealed class ReorderRowsWithDragDropData :IDisposable
		{
			public ReorderRowsWithDragDropData(DataGrid grid, Point grab)
			{
				Grid = grid;
				Grab = grab;
				Row = (grid.InputHitTest(grab) as DependencyObject)?.FindVisualParent<DataGridRow>(root: grid) ?? throw new Exception("Dragged row not found");
				Indicator = new Popup
				{
					Placement = PlacementMode.Relative,
					PlacementTarget = grid,
					IsHitTestVisible = false,
					AllowsTransparency = true,
					StaysOpen = true,
				};
				Indicator.Child = new System.Windows.Shapes.Path
				{
					Data = Geometry_.MakePolygon(true, new Point(0, 0), new Point(10, 4), new Point(0, 8)),
					Stroke = Brushes.DeepSkyBlue,
					Fill = Brushes.SkyBlue,
					StrokeThickness = 1,
					IsHitTestVisible = false,
				};
			}
			public void Dispose()
			{
				Indicator.IsOpen = false;
			}

			/// <summary>The source of the drag</summary>
			public DataGrid Grid { get; }

			/// <summary>The row being dragged</summary>
			public DataGridRow Row { get; }

			/// <summary>Dragging only starts if the mouse moves by more than a minimum distance</summary>
			public bool Dragging
			{
				get => Indicator.IsOpen;
				set => Indicator.IsOpen = value;
			}

			/// <summary>Grid relative mouse location at the start of the drag</summary>
			public Point Grab { get; }

			/// <summary>Graphics to show where the row will be inserted</summary>
			private Popup Indicator { get; }

			/// <summary>Position the insert indicator. 'pt' is in screen space</summary>
			public void PositionIndicator(Point scn_pt)
			{
				var sz = Indicator.Child.DesiredSize;
				var pt = Grid.PointFromScreen(scn_pt);
				Indicator.HorizontalOffset = pt.X - sz.Width;
				Indicator.VerticalOffset = pt.Y - sz.Height / 2;
			}
		}
		public class ReorderRowDropEventArgs :RoutedEventArgs
		{
			public ReorderRowDropEventArgs(object sender, DataGridRow row, int insert_index)
				:base(ReorderRowDropEvent, sender)
			{
				Row = row;
				DropIndex = insert_index;
			}

			/// <summary>The row being dragged</summary>
			public DataGridRow Row { get; }

			/// <summary>The index of the row that was grabbed</summary>
			public int GrabIndex => Row.GetIndex();

			/// <summary>The index of where to insert the row</summary>
			public int DropIndex { get; }
		}

		/// <summary>Default implementation for the 'ReorderRowDrop' event when not handled</summary>
		public static void ReorderRowsDropDefaultHandler(object sender, ReorderRowDropEventArgs args)
		{
			// Guess at the item source and reorder:
			// If the source is an unsorted/unfiltered/ungrouped ICollectionView then the source collection
			// order should match the view order. Modify the source collection and refresh the view.
			var grid = (DataGrid)sender;
			if (grid.ItemsSource is ICollectionView view && view.SortDescriptions.Count == 0 && view.Filter == null && view.GroupDescriptions.Count == 0)
			{
				// Drag and drop is only supported on mutable lists
				if (view.SourceCollection is IList list && !list.IsReadOnly)
				{
					// Move to before the grab position
					if (args.DropIndex < args.GrabIndex)
					{
						var tmp = list[args.GrabIndex];
						for (int i = args.GrabIndex; i != args.DropIndex; --i)
							list[i] = list[i - 1];

						list[args.DropIndex] = tmp;
					}
					// Move to after the grab position
					else
					{
						var tmp = list[args.GrabIndex];
						for (int i = args.GrabIndex; i != args.DropIndex - 1; ++i)
							list[i] = list[i + 1];

						list[args.DropIndex - 1] = tmp;
					}
					view.Refresh();
					args.Handled = true;
				}
			}
		}

		/// <summary>Event for reordering the underlying collection on row drop</summary>
		private const int ReorderRowDrop = 0;
		public delegate void ReorderRowDropEventHandler(object sender, ReorderRowDropEventArgs args);
		public static readonly RoutedEvent ReorderRowDropEvent = EventManager.RegisterRoutedEvent(nameof(ReorderRowDrop), RoutingStrategy.Bubble, typeof(ReorderRowDropEventHandler), typeof(DataGrid_));
		public static void AddReorderRowDropHandler(DependencyObject d, ReorderRowDropEventHandler handler)
		{
			if (d is UIElement uie)
				uie.AddHandler(ReorderRowDropEvent, handler);
		}
		public static void RemoveReorderRowDropHandler(DependencyObject d, ReorderRowDropEventHandler handler)
		{
			if (d is UIElement uie)
				uie.RemoveHandler(ReorderRowDropEvent, handler);
		}

		/// <summary>Event to signal that rows have been reordered</summary>
		private const int RowsReordered = 0;
		public delegate void RowsReorderedEventHandler(object sender, RoutedEventArgs args);
		public static readonly RoutedEvent RowsReorderedEvent = EventManager.RegisterRoutedEvent(nameof(RowsReordered), RoutingStrategy.Bubble, typeof(RowsReorderedEventHandler), typeof(DataGrid_));

		public static void AddRowsReorderedHandler(DependencyObject d, RowsReorderedEventHandler handler)
		{
			if (d is UIElement uie)
				uie.AddHandler(RowsReorderedEvent, handler);
		}
		public static void RemoveRowsReorderedHandler(DependencyObject d, RowsReorderedEventHandler handler)
		{
			if (d is UIElement uie)
				uie.RemoveHandler(RowsReorderedEvent, handler);
		}

		#endregion

		#region Column Name
		private const int ColumnName = 0;
		public static readonly DependencyProperty ColumnNameProperty = Gui_.DPRegisterAttached(typeof(DataGrid_), nameof(ColumnName), string.Empty, Gui_.EDPFlags.None);
		public static string GetColumnName(DependencyObject obj) => (string)obj.GetValue(ColumnNameProperty);
		public static void SetColumnName(DependencyObject obj, string value) => obj.SetValue(ColumnNameProperty, value);
		private static void ColumnName_Changed(DependencyObject obj)
		{
		}
		#endregion
	}
}
