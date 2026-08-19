using System;
using System.ComponentModel;
using System.Windows.Input;
using Rylogic.Utility;

namespace Rylogic.Gui.WPF.ChartDiagram
{
	public static partial class Diagram_
	{
		/// <summary>Implementation of Diagram context menu commands</summary>
		public class CMenu :IDiagramCMenu
		{
			public CMenu(ChartControl diagram)
			{
				Diagram = diagram;
				ToggleScattering = Command.Create(diagram, ToggleScatteringInternal);
				ToggleRelinking = Command.Create(diagram, ToggleRelinkingInternal);
				DoRelink = Command.Create(diagram, DoRelinkInternal);
				Diagram.Scene.Disposing += CleanUp;

				// Handlers
				void CleanUp(object? sender, EventArgs args)
				{
					Scattering = false;
					Relinking = false;
				}
			}

			/// <summary>The chart that hosts the diagram</summary>
			private ChartControl Diagram { get; }

			/// <summary>The diagram options</summary>
			private Options Options => (Options)Diagram.Options;

			/// <inheritdoc/>
			public bool Scattering
			{
				get => m_scatterer != null;
				set
				{
					if (Scattering == value) return;
					Util.Dispose(ref m_scatterer);
					m_scatterer = value ? new NodeScatterer(Diagram, Options) : null;
					NotifyPropertyChanged(nameof(Scattering));
					Diagram.Invalidate();
				}
			}
			public ICommand ToggleScattering { get; }
			private void ToggleScatteringInternal()
			{
				Scattering = !Scattering;
			}
			private NodeScatterer? m_scatterer;

			/// <inheritdoc/>
			public double ScatterCharge
			{
				get => Options.Scatter.CoulombConstant;
				set => Options.Scatter.CoulombConstant = value;
			}

			/// <inheritdoc/>
			public double ScatterSpring
			{
				get => Options.Scatter.SpringConstant;
				set => Options.Scatter.SpringConstant = value;
			}

			/// <inheritdoc/>
			public double ScatterFriction
			{
				get => Options.Scatter.FrictionConstant;
				set => Options.Scatter.FrictionConstant = value;
			}

			/// <inheritdoc/>
			public bool Relinking
			{
				get => m_link_optimiser != null;
				set
				{
					if (Relinking == value) return;
					Util.Dispose(ref m_link_optimiser);
					m_link_optimiser = value ? new LinkOptimiser(Diagram, Options) : null;
					NotifyPropertyChanged(nameof(Relinking));
					Diagram.Invalidate();
				}
			}
			public ICommand ToggleRelinking { get; }
			private void ToggleRelinkingInternal()
			{
				Relinking = !Relinking;
			}
			private LinkOptimiser? m_link_optimiser;

			/// <inheritdoc/>
			public ICommand DoRelink { get; }
			private void DoRelinkInternal()
			{
				using var lo = new LinkOptimiser(Diagram, Options);
				lo.Step();
			}

			/// <inheritdoc/>
			public event PropertyChangedEventHandler? PropertyChanged;
			private void NotifyPropertyChanged(string prop_name) => PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(prop_name));
		}
	}
}
