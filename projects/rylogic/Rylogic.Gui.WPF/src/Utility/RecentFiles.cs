using System;
using System.Collections.Generic;
using System.Text;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Input;
using Rylogic.Windows.Extn;

namespace Rylogic.Gui.WPF
{
	public class RecentFilesMenuItem : MenuItem
	{
		// Notes:
		//  - This is a replacement for a menu item that handles a list of recent file paths
		// Usage:
		// In your XAML file:
		//  'xmlns:gui="clr-namespace:Rylogic.Gui.WPF;assembly=Rylogic.Gui"'
		//  ...
		//  <Menu>
		//    <gui:RecentFilesMenuItem Header= "_Recent Files" Name="m_recent_files">
		//  </Menu>

		public RecentFilesMenuItem()
		{
			Filepaths = new List<string>();
			MaxCount = 10;
			ResetListText = "<Clear Recent Files>";
			UpdateMenu();
		}

		/// <summary>The collection of recent files</summary>
		public List<string> Filepaths { get; private set; }

		/// <summary>The handler for when a recent file is selected</summary>
		public Action<string>? RecentFileSelected { get; set; }

		/// <summary>Raised whenever the recent file list changes</summary>
		public event EventHandler? RecentFilesListChanged;

		/// <summary>Get/Set the limit for how many files appear in the recent files list</summary>
		public int MaxCount { get; set; }
		public static readonly DependencyProperty MaxCountProperty = Gui_.DPRegister<RecentFilesMenuItem>(nameof(MaxCount), 10, Gui_.EDPFlags.None);

		/// <summary>The text to display as the last item allowing the user to clear the list</summary>
		public string ResetListText { get; set; }
		public static readonly DependencyProperty ResetListTextProperty = Gui_.DPRegister<RecentFilesMenuItem>(nameof(ResetListText), "<Clear Recent Files>", Gui_.EDPFlags.None);

		/// <summary>Returns true if the given filepath is in the recent files list</summary>
		public bool IsInRecents(string file)
		{
			return Filepaths.FindIndex(x => string.Equals(x, file, StringComparison.CurrentCultureIgnoreCase)) != -1;
		}
		
		/// <summary>Reset the recent files list</summary>
		public void Clear()
		{
			Filepaths.Clear();
			UpdateMenu();

			// Notify recent files changed
			RecentFilesListChanged?.Invoke(this, EventArgs.Empty);
		}

		/// <summary>Remove a filepath from the recent files list</summary>
		public void Remove(string file, bool update_menu)
		{
			Filepaths.RemoveAll(x => string.Equals(x, file, StringComparison.CurrentCultureIgnoreCase));

			if (update_menu)
				UpdateMenu();

			// Notify recent files changed
			RecentFilesListChanged?.Invoke(this, EventArgs.Empty);
		}

		/// <summary>Add a file to the recent files list</summary>
		public void Add(string file)
		{
			Add(file, true);
		}
		public void Add(string file, bool update_menu)
		{
			// Validate the filepath
			if (string.IsNullOrEmpty(file))
				throw new Exception("Invalid recent file path");

			// Move the item matching 'file' to the front of the list
			Filepaths.RemoveAll(x => string.Equals(x, file, StringComparison.CurrentCultureIgnoreCase));
			Filepaths.Insert(0, file);

			// Cap the maximum number of recent files
			for (; Filepaths.Count > MaxCount;)
				Filepaths.RemoveAt(Filepaths.Count - 1);

			// Update the menu
			if (update_menu)
				UpdateMenu();

			// Notify recent files changed
			RecentFilesListChanged?.Invoke(this, EventArgs.Empty);
		}

		/// <summary>Update the recent files menu</summary>
		public void UpdateMenu()
		{
			// Clear any menu items
			Items.Clear();

			// Add a menu item for each filepath
			foreach (string f in Filepaths)
			{
				var item = new MenuItem()
				{
					Header = DisplayText(f),
					Tag = f,
				};
				item.Click += (s, a) =>
				{
					var menu = (MenuItem)s;
					var filepath = (string)menu.Tag;
					Add(filepath, true);
					RecentFileSelected?.Invoke(filepath);
				};
				item.PreviewMouseDown += (s, a) =>
				{
					var menu = (MenuItem)s;
					if (a.ChangedButton == MouseButton.Right && a.ButtonState == MouseButtonState.Pressed)
					{
						var loc = a.GetPosition(menu);
						ShowMenuItemContextMenu(item, loc, () => IsSubmenuOpen = false);
						a.Handled = true;
					}
				};
				Items.Add(item);
			}

			// Only add the clear all option if there is something to clear
			if (Items.Count != 0)
			{
				Items.Add(new Separator());

				// Add a menu item for clearing the recent files list
				var clear_all = new MenuItem
				{
					Header = ResetListText,
				};
				clear_all.Click += (s, a) =>
				{
					Clear();
				};
				Items.Add(clear_all);
			}
		}

		/// <summary>Show a context menu with more options for individual menu items</summary>
		private void ShowMenuItemContextMenu(MenuItem menu_item, Point pt, Action on_close)
		{
			var cmenu = new ContextMenu
			{
				PlacementTarget = menu_item,
				PlacementRectangle = new Rect(pt, Vector_.Infinity),
			};
			{
				// Remove
				var item = new MenuItem { Header = "Remove" };
				item.Click += (s, e) => Remove((string)menu_item.Tag, true);
				cmenu.Items.Add(item);
			}
			{
				// Copy to clipboard
				var item = new MenuItem { Header = "Copy to Clipboard" };
				item.Click += (s, e) =>
				{
					try { Clipboard.SetText((string)menu_item.Tag); }
					catch { }
				};
				cmenu.Items.Add(item);
			}
			cmenu.Closed += (s, a) => on_close?.Invoke();
			cmenu.IsOpen = true;
		}

		/// <summary>Escape access-key markers in display text</summary>
		private static string DisplayText(string text)
		{
			return text.Replace("_", "__");
		}

		/// <summary>Export recent files to a single string</summary>
		public string Export()
		{
			var str = new StringBuilder();
			foreach (var s in Filepaths)
				str.Append(s).Append(",");
			if (str.Length != 0)
				str.Remove(str.Length - 1, 1);
			return str.ToString();
		}

		/// <summary>Import recent files from a single string</summary>
		public void Import(string str)
		{
			if (!string.IsNullOrEmpty(str))
			{
				Filepaths.Clear();
				foreach (var s in str.Split(new[] { "," }, StringSplitOptions.RemoveEmptyEntries))
					Filepaths.Add(s);
			}
			UpdateMenu();
		}
	}
}
