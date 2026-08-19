using System;
using System.ComponentModel;
using System.Windows.Input;

namespace Rylogic.Gui.WPF.ChartDiagram
{
	/// <summary>Marker interface for element styles</summary>
	public interface IStyle :INotifyPropertyChanged
	{
	}

	/// <summary>For elements that have an associated IStyle object</summary>
	public interface IHasStyle
	{
		IStyle Style { get; }
	}

	/// <summary>Says "I have a GUID"</summary>
	internal interface IHasId
	{
		Guid Id { get; }
	}

	/// <summary>Diagram menu item</summary>
	public interface IDiagramCMenu :INotifyPropertyChanged
	{
		/// <summary>Enable/Disable continuous scattering of nodes</summary>
		bool Scattering { get; }
		ICommand ToggleScattering { get; }

		/// <summary>The scattering coulomb charge value</summary>
		double ScatterCharge { get; set; }

		/// <summary>The scattering spring constant value</summary>
		double ScatterSpring { get; set; }

		/// <summary>The scattering friction constant value</summary>
		double ScatterFriction { get; set; }

		/// <summary>Enable/Disabled continuous optimisation of connector connections between nodes</summary>
		bool Relinking { get; }
		ICommand ToggleRelinking { get; }

		/// <summary>Perform one iteration of relinking</summary>
		ICommand DoRelink { get; }
	}
	public interface IDiagramCMenuContext
	{
		/// <summary>The data context for diagram context menu items</summary>
		IDiagramCMenu DiagramCMenuContext { get; }
	}
}