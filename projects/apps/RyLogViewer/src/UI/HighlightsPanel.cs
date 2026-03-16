using System.Collections.Specialized;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Media;
using Rylogic.Common;
using Rylogic.Gui.WPF;

namespace RyLogViewer
{
	/// <summary>Dockable panel for managing highlight patterns</summary>
	public class HighlightsPanel : DockPanel, IDockable
	{
		private readonly Main m_main;
		private readonly ListBox m_list;

		public HighlightsPanel(Main main)
		{
			m_main = main;

			DockControl = new DockControl(this, "Highlights")
			{
				TabText = "Highlights",
			};

			// Toolbar with add/remove buttons
			var toolbar = new System.Windows.Controls.ToolBar();
			SetDock(toolbar, Dock.Top);
			Children.Add(toolbar);

			var add_btn = new Button { Content = "Add", Padding = new Thickness(8, 2, 8, 2) };
			add_btn.Click += (s, e) => AddHighlight();
			toolbar.Items.Add(add_btn);

			var remove_btn = new Button { Content = "Remove", Padding = new Thickness(8, 2, 8, 2) };
			remove_btn.Click += (s, e) => RemoveHighlight();
			toolbar.Items.Add(remove_btn);

			var edit_btn = new Button { Content = "Edit", Padding = new Thickness(8, 2, 8, 2) };
			edit_btn.Click += (s, e) => EditHighlight();
			toolbar.Items.Add(edit_btn);

			// List of highlights
			m_list = new ListBox
			{
				ItemsSource = m_main.Highlights,
				DisplayMemberPath = nameof(Highlight.Expr),
			};
			Children.Add(m_list);

			// Style the list items with highlight colours
			m_list.ItemContainerStyle = CreateHighlightItemStyle();
		}

		/// <summary>IDockable implementation</summary>
		public DockControl DockControl { get; }

		/// <summary>Add a new highlight pattern</summary>
		private void AddHighlight()
		{
			var hl = new Highlight
			{
				Expr = "",
				PatnType = EPattern.Substring,
				Active = true,
			};

			// Open editor dialog
			var dlg = new PatternEditorUI { Owner = Window.GetWindow(this) };
			dlg.Editor.NewPattern(hl);
			if (dlg.ShowDialog() == true)
			{
				// Copy the edited pattern properties back
				var pat = dlg.Editor.Pattern;
				hl.Expr = pat.Expr;
				hl.PatnType = pat.PatnType;
				hl.IgnoreCase = pat.IgnoreCase;
				hl.WholeLine = pat.WholeLine;
				hl.Invert = pat.Invert;
				m_main.Highlights.Add(hl);
			}
		}

		/// <summary>Edit the selected highlight</summary>
		private void EditHighlight()
		{
			if (m_list.SelectedItem is not Highlight hl) return;

			var dlg = new PatternEditorUI { Owner = Window.GetWindow(this) };
			dlg.Editor.EditPattern(hl, clone: true);
			if (dlg.ShowDialog() == true)
			{
				var pat = dlg.Editor.Pattern;
				hl.Expr = pat.Expr;
				hl.PatnType = pat.PatnType;
				hl.IgnoreCase = pat.IgnoreCase;
				hl.WholeLine = pat.WholeLine;
				hl.Invert = pat.Invert;

				// Notify the collection changed to trigger UI refresh
				m_main.Highlights.NotifyItemChanged();
			}
		}

		/// <summary>Remove the selected highlight</summary>
		private void RemoveHighlight()
		{
			if (m_list.SelectedItem is not Highlight hl) return;
			m_main.Highlights.Remove(hl);
		}

		/// <summary>Create a style for list items that shows highlight colours</summary>
		private static Style CreateHighlightItemStyle()
		{
			var style = new Style(typeof(ListBoxItem));

			// Bind Background to the highlight's BackColour
			var bg_setter = new Setter(ListBoxItem.BackgroundProperty, new Binding(nameof(Highlight.BackColour))
			{
				Converter = new ColorToBrushConverter(),
			});
			style.Setters.Add(bg_setter);

			// Bind Foreground to the highlight's ForeColour
			var fg_setter = new Setter(ListBoxItem.ForegroundProperty, new Binding(nameof(Highlight.ForeColour))
			{
				Converter = new ColorToBrushConverter(),
			});
			style.Setters.Add(fg_setter);

			return style;
		}

		/// <summary>Converter from Color to SolidColorBrush</summary>
		private class ColorToBrushConverter : IValueConverter
		{
			public object Convert(object value, System.Type targetType, object parameter, System.Globalization.CultureInfo culture)
			{
				if (value is Color color)
					return new SolidColorBrush(color);
				return Brushes.Transparent;
			}

			public object ConvertBack(object value, System.Type targetType, object parameter, System.Globalization.CultureInfo culture)
			{
				throw new System.NotImplementedException();
			}
		}
	}
}
