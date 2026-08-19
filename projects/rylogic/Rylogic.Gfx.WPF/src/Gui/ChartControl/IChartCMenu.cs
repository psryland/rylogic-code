using System.ComponentModel;
using System.Windows.Input;

namespace Rylogic.Gui.WPF
{
	public interface IChartCMenu :INotifyPropertyChanged
	{
		/// <summary>Show/hide grid lines</summary>
		bool ShowGridLines { get; set; }
		ICommand ToggleGridLines { get; }

		/// <summary>Show/hide axes</summary>
		bool ShowAxes { get; set; }
		ICommand ToggleAxes { get; }

		/// <summary>Allow elements on the chart to be selected</summary>
		bool AllowSelection { get; set; }
		ICommand ToggleAllowSelection { get; }

		/// <summary>Allow elements on the chart to be dragged around</summary>
		bool AllowElementDragging { get; set; }
		ICommand ToggleAllowElementDragging { get; }

		/// <summary>Show/hide the popup containing the pointer value</summary>
		bool ShowValueAtPointer { get; set; }
		ICommand ToggleShowValueAtPointer { get; }

		/// <summary>Show/hide the cross-hair</summary>
		bool ShowCrossHair { get; set; }
		ICommand ToggleShowCrossHair { get; }

		/// <summary>Navigation control mode</summary>
		ChartControl.ENavMode NavigationMode { get; set; }

		/// <summary>Reset the aspect ratio to 1:1</summary>
		ICommand DoAspect11 { get; }

		/// <summary>Get/Set whether the aspect ratio can change</summary>
		bool LockAspect { get; set; }
		ICommand ToggleLockAspect { get; }

		/// <summary>Get/Set whether zoom zooms in toward the mouse</summary>
		bool MouseCentredZoom { get; set; }
		ICommand ToggleMouseCentredZoom { get; }

		/// <summary>Link camera to another scene camera</summary>
		bool CanLinkCamera { get; }
		ICommand LinkCamera { get; }

		/// <summary>Allow property changed to be triggered externally</summary>
		void NotifyPropertyChanged(string prop_name);
	}
	public interface IChartCMenuContext
	{
		/// <summary>The data context for Chart menu items</summary>
		IChartCMenu ChartCMenuContext { get; }
	}
	public interface IChartAxisCMenu :INotifyPropertyChanged
	{
		/// <summary>Allow scrolling</summary>
		bool AllowScroll { get; set; }
		ICommand ToggleScrollLock { get; }

		/// <summary>Allow zooming</summary>
		bool AllowZoom { get; set; }
		ICommand ToggleZoomLock { get; }

		/// <summary>Allow property changed to be triggered externally</summary>
		void NotifyPropertyChanged(string prop_name);
	}
	public interface IChartAxisCMenuContext
	{
		/// <summary>The data context for Chart axis menu items</summary>
		IChartAxisCMenu ChartAxisCMenuContext { get; }
	}
}
