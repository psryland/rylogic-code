using System;
using System.ComponentModel;
using System.Windows;
using Rylogic.Gfx;

namespace Rylogic.Gui.WPF
{
	public partial class View3dRayTracingUI :Window, INotifyPropertyChanged
	{
		private readonly View3d.Window m_window;

		/// <summary>Create the ray tracing configuration dialog for 'window'</summary>
		public View3dRayTracingUI(Window owner, View3d.Window window)
		{
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
			m_window.OnSettingsChanged -= HandleSettingsChanged;
			base.OnClosed(e);
		}

		/// <summary>Support pinning this window</summary>
		private PinData PinState { get; }

		/// <summary>True if this window can enable ray tracing</summary>
		public bool RayTracingAvailable
		{
			get
			{
				return m_window.RayTracingInfo.Available;
			}
		}

		/// <summary>Enable/disable ray tracing for this window</summary>
		public bool RayTracingEnabled
		{
			get
			{
				return m_window.RayTracingEnabled;
			}
			set
			{
				if (RayTracingEnabled == value)
					return;

				m_window.RayTracingEnabled = value;
				NotifyRayTracingChanged();
			}
		}

		/// <summary>Status text describing whether ray tracing can be enabled</summary>
		public string StatusText
		{
			get
			{
				var info = m_window.RayTracingInfo;
				if (info.Available)
					return info.Enabled
						? "Ray tracing is enabled. The current implementation renders diagnostic-coloured reflections for opaque static geometry."
						: "Ray tracing is available but disabled. Enable it to add the ray tracing render step to this view.";

				if (!info.Requested)
					return "Ray tracing capability was not requested when View3D was initialised.";

				if (!info.HardwareSupported)
					return $"Ray tracing is not supported by this device ({TierName(info.Tier)}).";

				return $"Ray tracing is unavailable ({TierName(info.Tier)}).";
			}
		}

		/// <summary>Close the ray tracing configuration dialog</summary>
		public Command Accept { get; }
		private void AcceptInternal()
		{
			if (this.IsModal())
				DialogResult = true;

			Close();
		}

		/// <summary>Refresh bindings when the native window reports a ray tracing setting change</summary>
		private void HandleSettingsChanged(object? sender, View3d.SettingChangeEventArgs e)
		{
			if (!e.Setting.HasFlag(View3d.ESettings.Rendering_RayTracing))
				return;

			NotifyRayTracingChanged();
		}

		/// <summary>Notify all ray tracing UI bindings that the native state may have changed</summary>
		private void NotifyRayTracingChanged()
		{
			NotifyPropertyChanged(nameof(RayTracingAvailable));
			NotifyPropertyChanged(nameof(RayTracingEnabled));
			NotifyPropertyChanged(nameof(StatusText));
		}

		/// <summary>Convert the public ray tracing tier enum to display text</summary>
		private static string TierName(View3d.ERayTracingTier tier)
		{
			switch (tier)
			{
			case View3d.ERayTracingTier.NotSupported:
				{
					return "not supported";
				}
			case View3d.ERayTracingTier.Tier1_0:
				{
					return "tier 1.0";
				}
			case View3d.ERayTracingTier.Tier1_1:
				{
					return "tier 1.1";
				}
			case View3d.ERayTracingTier.Unknown:
				{
					return "unknown tier";
				}
			default:
				{
					throw new ArgumentOutOfRangeException(nameof(tier), tier, "Unknown ray tracing tier");
				}
			}
		}

		/// <inheritdoc/>
		public event PropertyChangedEventHandler? PropertyChanged;

		/// <summary>Notify data bindings of a property change</summary>
		private void NotifyPropertyChanged(string prop_name)
		{
			PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(prop_name));
		}
	}
}
