using System;
using System.ComponentModel;
using System.Windows.Controls;
using Rylogic.Gfx;
using Rylogic.Gui.WPF;
using Rylogic.Utility;

namespace LDraw.UI
{
	public partial class ObjectManagerUI : UserControl, IDockable, IDisposable, INotifyPropertyChanged
	{
		public ObjectManagerUI()
		{
			InitializeComponent();
			DockControl = new DockControl(this, "ObjectManager")
			{
				ShowTitle = false,
				TabText = "Scene Manager",
				DestroyOnClose = false,
			};
			DataContext = this;
		}
		public void Dispose()
		{
			ManagerUI = null;
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

		/// <summary>The current View3dObjectManagerUI instance</summary>
		private View3dObjectManagerUI? ManagerUI
		{
			get;
			set
			{
				if (field == value) return;
				if (field != null)
				{
					m_root.Children.Remove(field);
					field.Dispose();
				}
				field = value;
				if (field != null)
				{
					m_root.Children.Add(field);
				}
			}
		}

		/// <summary>Update the object manager content for the given scene</summary>
		public void SetScene(SceneUI? scene)
		{
			var window = scene?.SceneView?.Scene?.Window;

			// No-op if we're already showing this window. Recreating ManagerUI
			// throws away the tree's expansion/selection state, which makes the
			// scene manager appear to "collapse itself" whenever the active
			// dockable changes (e.g. clicking a scene to interact with it).
			if (ReferenceEquals(window, m_current_window))
				return;

			m_current_window = window;
			ManagerUI = window != null ? new View3dObjectManagerUI(window) : null;
		}
		private View3d.Window? m_current_window;

		/// <inheritdoc/>
		public event PropertyChangedEventHandler? PropertyChanged;
		private void NotifyPropertyChanged(string prop_name) => PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(prop_name));
	}
}
