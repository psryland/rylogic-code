using System.Windows;
using System.Windows.Controls;
using Rylogic.Common;
using Rylogic.Gui.WPF;

namespace RyLogViewer
{
	/// <summary>Dockable panel for managing text transforms</summary>
	public class TransformsPanel : DockPanel, IDockable
	{
		private readonly Main m_main;
		private readonly ListBox m_list;

		public TransformsPanel(Main main)
		{
			m_main = main;

			DockControl = new DockControl(this, "Transforms")
			{
				TabText = "Transforms",
			};

			// Toolbar
			var toolbar = new System.Windows.Controls.ToolBar();
			SetDock(toolbar, Dock.Top);
			Children.Add(toolbar);

			var add_btn = new Button { Content = "Add", Padding = new Thickness(8, 2, 8, 2) };
			add_btn.Click += (s, e) => AddTransform();
			toolbar.Items.Add(add_btn);

			var edit_btn = new Button { Content = "Edit", Padding = new Thickness(8, 2, 8, 2) };
			edit_btn.Click += (s, e) => EditTransform();
			toolbar.Items.Add(edit_btn);

			var remove_btn = new Button { Content = "Remove", Padding = new Thickness(8, 2, 8, 2) };
			remove_btn.Click += (s, e) => RemoveTransform();
			toolbar.Items.Add(remove_btn);

			// Transform list
			m_list = new ListBox
			{
				ItemsSource = m_main.Transforms,
			};
			Children.Add(m_list);
		}

		/// <summary>IDockable implementation</summary>
		public DockControl DockControl { get; }

		/// <summary>Add a new transform</summary>
		private void AddTransform()
		{
			var dlg = new TransformEditUI { Owner = Window.GetWindow(this) };
			if (dlg.ShowDialog() == true && dlg.Result != null)
				m_main.Transforms.Add(dlg.Result);
		}

		/// <summary>Edit the selected transform</summary>
		private void EditTransform()
		{
			if (m_list.SelectedItem is not Transform tx) return;
			var dlg = new TransformEditUI(tx) { Owner = Window.GetWindow(this) };
			if (dlg.ShowDialog() == true && dlg.Result != null)
			{
				var idx = m_list.SelectedIndex;
				m_main.Transforms[idx] = dlg.Result;
			}
		}

		/// <summary>Remove the selected transform</summary>
		private void RemoveTransform()
		{
			if (m_list.SelectedItem is not Transform tx) return;
			m_main.Transforms.Remove(tx);
		}
	}
}
