using System;
using System.ComponentModel;
using System.Windows;
using Rylogic.Gfx;

namespace Rylogic.Gui.WPF
{
	public partial class View3dStereoUI :Window, INotifyPropertyChanged
	{
		private readonly View3d.Window m_window;

		/// <summary>Create the side-by-side stereo configuration dialog for 'window'</summary>
		public View3dStereoUI(Window owner, View3d.Window window)
		{
			// Bind the dialog to the native window's stereo settings
			InitializeComponent();
			Owner = owner;
			Icon = owner?.Icon;
			m_window = window;

			m_window.OnSettingsChanged += HandleSettingsChanged;
			Accept = Command.Create(this, AcceptInternal);
			PinState = new PinData(this, EPin.Centre, pinned: false);

			DataContext = this;
		}

		/// <summary>Remove native window event subscriptions when the dialog closes</summary>
		protected override void OnClosed(EventArgs e)
		{
			// Stop listening to the native window
			m_window.OnSettingsChanged -= HandleSettingsChanged;
			base.OnClosed(e);
		}

		/// <summary>Support pinning this window</summary>
		private PinData PinState { get; }

		/// <summary>Enable/disable side-by-side stereo rendering</summary>
		public bool Enabled
		{
			get
			{
				// Read from the native window so the dialog always shows the current state
				return m_window.Stereo.Enabled;
			}
			set
			{
				// Apply the change to the native window
				var props = m_window.Stereo;
				if (props.Enabled == value)
					return;

				props.Enabled = value;
				m_window.Stereo = props;
			}
		}

		/// <summary>Distance between the eyes in millimetres (0 = automatic)</summary>
		public float EyeSeparationMm
		{
			get
			{
				// Read from the native window so the dialog always shows the current state
				return m_window.Stereo.EyeSeparationMm;
			}
			set
			{
				// The native window rejects negative distances
				if (value < 0)
					throw new ArgumentOutOfRangeException(nameof(EyeSeparationMm), "Eye separation cannot be negative");

				var props = m_window.Stereo;
				if (props.EyeSeparationMm == value)
					return;

				props.EyeSeparationMm = value;
				m_window.Stereo = props;
			}
		}

		/// <summary>Length of one scene unit in millimetres (0 = metres)</summary>
		public float SceneUnitMm
		{
			get
			{
				// Read from the native window so the dialog always shows the current state
				return m_window.Stereo.SceneUnitMm;
			}
			set
			{
				// The native window rejects negative lengths
				if (value < 0)
					throw new ArgumentOutOfRangeException(nameof(SceneUnitMm), "Scene unit length cannot be negative");

				var props = m_window.Stereo;
				if (props.SceneUnitMm == value)
					return;

				props.SceneUnitMm = value;
				m_window.Stereo = props;
			}
		}

		/// <summary>Distance to the zero-disparity plane in scene units (0 = automatic)</summary>
		public float Convergence
		{
			get
			{
				// Read from the native window so the dialog always shows the current state
				return m_window.Stereo.Convergence;
			}
			set
			{
				// The native window rejects negative distances
				if (value < 0)
					throw new ArgumentOutOfRangeException(nameof(Convergence), "Convergence distance cannot be negative");

				var props = m_window.Stereo;
				if (props.Convergence == value)
					return;

				props.Convergence = value;
				m_window.Stereo = props;
			}
		}

		/// <summary>Close the stereo configuration dialog</summary>
		public Command Accept { get; }
		private void AcceptInternal()
		{
			// Close the dialog, reporting success when shown modally
			if (this.IsModal())
				DialogResult = true;

			Close();
		}

		/// <summary>Refresh bindings when the native window reports a stereo setting change</summary>
		private void HandleSettingsChanged(object? sender, View3d.SettingChangeEventArgs e)
		{
			// Ignore unrelated setting changes
			if (!e.Setting.HasFlag(View3d.ESettings.Rendering_Stereo))
				return;

			NotifyPropertyChanged(nameof(Enabled));
			NotifyPropertyChanged(nameof(EyeSeparationMm));
			NotifyPropertyChanged(nameof(SceneUnitMm));
			NotifyPropertyChanged(nameof(Convergence));
		}

		/// <inheritdoc/>
		public event PropertyChangedEventHandler? PropertyChanged;

		/// <summary>Notify data bindings of a property change</summary>
		private void NotifyPropertyChanged(string prop_name)
		{
			// Forward to WPF bindings
			PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(prop_name));
		}
	}
}
