using System;
using System.ComponentModel;
using System.Windows;
using Rylogic.Gfx;
using Rylogic.Maths;
using Rylogic.Utility;

namespace Rylogic.Gui.WPF
{
	public partial class View3dCameraUI : Window, INotifyPropertyChanged
	{
		private readonly View3d.Window m_window;
		public View3dCameraUI(Window owner, View3d.Window window)
		{
			InitializeComponent();
			Owner = owner;
			Icon = owner?.Icon;

			m_window = window;
			m_window.OnRendering += RefreshCamera;
			PinState = new PinData(this, EPin.Centre, pinned: false);

			ConstrainFocus = Command.Create(this, ConstrainFocusInternal);
			Accept = Command.Create(this, AcceptInternal);

			DataContext = this;
		}
		protected override void OnClosed(EventArgs e)
		{
			base.OnClosed(e);
			m_window.OnRendering -= RefreshCamera;
		}
		private void RefreshCamera(object? sender, EventArgs e)
		{
			NotifyPropertyChanged("");
		}

		/// <summary>Support pinning this window</summary>
		private PinData PinState { get; }

		/// <summary>The camera to display</summary>
		public View3d.Camera Camera
		{
			get => m_window.Camera;
		}

		/// <summary>Camera position</summary>
		public v4 Position
		{
			get => Camera.O2W.pos;
			set
			{
				if (Position == value) return;
				Camera.SetPosition(value);
				NotifyPropertyChanged(nameof(Position));
			}
		}

		/// <summary>What the camera is looking at</summary>
		public v4 FocusPoint
		{
			get => Camera.FocusPoint;
			set
			{
				if (FocusPoint == value) return;
				Camera.FocusPoint = value;
				NotifyPropertyChanged(nameof(FocusPoint));
			}
		}

		/// <summary>Distance to the near plane</summary>
		public float NearPlane
		{
			get => Camera.NearPlane;
			set
			{
				if (NearPlane == value) return;
				Camera.NearPlane = value;
				NotifyPropertyChanged(nameof(NearPlane));
			}
		}

		/// <summary>Distance to the far plane</summary>
		public float FarPlane
		{
			get => Camera.FarPlane;
			set
			{
				if (FarPlane == value) return;
				Camera.FarPlane = value;
				NotifyPropertyChanged(nameof(FarPlane));
			}
		}

		/// <summary>The bounding box that limits the focus position</summary>
		public string FocusBoundsCentre
		{
			get => Camera.FocusBounds.Centre.xyz.ToString();
			set
			{
				if (FocusBoundsCentre == value) return;
				if (v3.TryParse(value) is v3 centre)
				{
					var bounds = Camera.FocusBounds;
					bounds.Centre = centre.w1;
					Camera.FocusBounds = bounds;
					NotifyPropertyChanged(nameof(FocusBoundsCentre));
				}
			}
		}
		public string FocusBoundsRadius
		{
			get => Camera.FocusBounds.Radius.xyz.ToString();
			set
			{
				if (FocusBoundsRadius == value) return;
				if (v3.TryParse(value) is v3 radius)
				{
					var bounds = Camera.FocusBounds;
					bounds.Radius = radius.w0;
					Camera.FocusBounds = bounds;
					NotifyPropertyChanged(nameof(FocusBoundsRadius));
				}
			}
		}

		/// <summary>Constrain the focus point</summary>
		public Command ConstrainFocus { get; }
		private void ConstrainFocusInternal(object? parameter)
		{
			if (parameter is not string code)
				return;

			try
			{
				var bounds = Camera.FocusBounds;
				var radius = code[0] == '0' ? 0 : float.PositiveInfinity;
				switch (code[1])
				{
					case 'X': bounds.Radius.x = radius; break;
					case 'Y': bounds.Radius.y = radius; break;
					case 'Z': bounds.Radius.z = radius; break;
					default: throw new Exception("Unknown focus constraint axis");
				}
				Camera.FocusBounds = bounds;
				NotifyPropertyChanged(nameof(FocusBoundsRadius));
			}
			catch (Exception ex)
			{
				MsgBox.Show(Owner, $"Failed to constrain focus point.\n{ex.Message}", Util.AppProductName, MsgBox.EButtons.OK, MsgBox.EIcon.Information);
			}
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
