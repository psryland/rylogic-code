using System;
using System.ComponentModel;
using System.Windows;
using Rylogic.Gfx;

namespace Rylogic.Gui.WPF
{
	public partial class View3dLightingUI :Window, INotifyPropertyChanged
	{
		private readonly View3d.Window m_window;
		public View3dLightingUI(Window owner, View3d.Window window)
		{
			InitializeComponent();
			Owner = owner;
			Icon = owner?.Icon;
			m_window = window;

			Light = new View3d.Light(m_window.LightProperties);
			PinState = new PinData(this, EPin.Centre, pinned: false);

			SetAmbientColour = Command.Create(this, SetAmbientColourInternal);
			SetDiffuseColour = Command.Create(this, SetDiffuseColourInternal);
			SetSpecularColour = Command.Create(this, SetSpecularColourInternal);
			Accept = Command.Create(this, AcceptInternal);

			DataContext = this;
		}
		protected override void OnClosed(EventArgs e)
		{
			Light = null!;
			base.OnClosed(e);
		}

		/// <summary>The light being modified</summary>
		public View3d.Light Light
		{
			get => m_light;
			set
			{
				if (m_light == value) return;
				if (m_light != null)
				{
					m_light.PropertyChanged -= HandlePropertyChanged;
				}
				m_light = value;
				if (m_light != null)
				{
					m_light.PropertyChanged += HandlePropertyChanged;
				}

				// Handlers
				void HandlePropertyChanged(object? sender, PropertyChangedEventArgs e)
				{
					switch (e.PropertyName)
					{
					case nameof(Light.Type):
						{
							NotifyPropertyChanged(nameof(IsCasting));
							NotifyPropertyChanged(nameof(HasPosition));
							NotifyPropertyChanged(nameof(HasDirection));
							NotifyPropertyChanged(nameof(HasSpot));
							break;
						}
					}
					m_window.LightProperties = Light.Data;
					m_window.Invalidate();
				}
			}
		}
		private View3d.Light m_light = null!;

		/// <summary>Support pinning this window</summary>
		private PinData PinState { get; }

		/// <summary>True if the light casts rays/shadows</summary>
		public bool IsCasting => Light.Type != View3d.ELight.Ambient;

		/// <summary>True if position is valid for the light type</summary>
		public bool HasPosition => Light.Type == View3d.ELight.Point || Light.Type == View3d.ELight.Spot;

		/// <summary>True if direction is valid for the light type</summary>
		public bool HasDirection => Light.Type == View3d.ELight.Directional || Light.Type == View3d.ELight.Spot;

		/// <summary>True if the light is spotlight cones</summary>
		public bool HasSpot => Light.Type == View3d.ELight.Spot;

		/// <summary>Assign the ambient colour</summary>
		public Command SetAmbientColour { get; }
		private void SetAmbientColourInternal()
		{
			var ui = new ColourPickerUI(this, Light.Ambient);
			ui.ColorChanged += delegate { Light.Ambient = ui.Colour; };
			Light.Ambient = ui.ShowDialog() == true ? ui.Colour : ui.InitialColour;
		}

		/// <summary>Assign the diffuse colour</summary>
		public Command SetDiffuseColour { get; }
		private void SetDiffuseColourInternal()
		{
			var ui = new ColourPickerUI(this, Light.Diffuse);
			ui.ColorChanged += delegate { Light.Diffuse = ui.Colour; };
			Light.Diffuse = ui.ShowDialog() == true ? ui.Colour : ui.InitialColour;
		}

		/// <summary>Assign the specular colour</summary>
		public Command SetSpecularColour { get; }
		private void SetSpecularColourInternal()
		{
			var ui = new ColourPickerUI(this, Light.Specular);
			ui.ColorChanged += delegate { Light.Specular = ui.Colour; };
			Light.Specular = ui.ShowDialog() == true ? ui.Colour : ui.InitialColour;
		}

		/// <summary>Accept button</summary>
		public Command Accept { get; }
		private void AcceptInternal()
		{
			if (this.IsModal())
				DialogResult = true;

			Close();
		}

		/// <inheritdoc/>
		public event PropertyChangedEventHandler? PropertyChanged;
		private void NotifyPropertyChanged(string prop_name) => PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(prop_name));
	}
}
