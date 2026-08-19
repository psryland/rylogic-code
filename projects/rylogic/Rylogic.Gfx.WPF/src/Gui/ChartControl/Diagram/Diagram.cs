using System;
using System.ComponentModel;
using System.Linq;
using System.Windows.Input;
using Rylogic.Common;
using Rylogic.Gfx;
using Rylogic.Maths;
using Rylogic.Utility;

namespace Rylogic.Gui.WPF.ChartDiagram
{
	public static partial class Diagram_
	{
		// Notes:
		//  - Put a ChartControl instance in your XAML.
		//  - Call Diagram_.Init() to set it up to be used as a diagram.


		/// <summary>Initialise a chart control to be used for diagrams</summary>
		//public static void Init(ChartControl diagram, Options options)
		//{
		//	diagram.Scene.Window.LightProperties = View3d.LightInfo.Ambient(0xFF808080);
		//	diagram.Options.Orthographic = true;
		//	diagram.Options.LockAspect = 1.0;
		//	diagram.Options.AllowElementDragging = options.AllowArrange;
		//
		//	// Add support for resizing nodes
		//	//chart.MouseDown -= HandleMouseDown;
		//	//chart.MouseDown += HandleMouseDown;
		//	//void HandleMouseDown(object? sender, MouseButtonEventArgs e)
		//	//{
		//	//	if (sender is not ChartControl chart) return;
		//	//	if (!chart.Selected.OfType<Node>().Any(x => x.Selected && x.Style.Resizeable))
		//	//		return;
		//	//	
		//	//	chart.MouseOperations.Pending[MouseButton.Left] = new MouseOpResize(chart);
		//	//	e.Handled = true;
		//	//}
		//}

		/// <summary>Diagram Options</summary>
		public class Options :ChartControl.OptionsData
		{
			public Options()
			{
				NavigationMode = ChartControl.ENavMode.Chart2D;
				BackgroundColour = Colour32.Gray;
				AllowElementDragging = true;
				AllowSelection = true;
				ShowAxes = false;
				ShowGridLines = false;
				Orthographic = true;
				LockAspect = 1.0;
				Scatter = new ScatterOptions();
				Relink = new RelinkOptions();
			}

			/// <summary>Allow nodes in the diagram to be moved</summary>
			public bool AllowArrange
			{
				get => get<bool>(nameof(AllowArrange));
				set => set(nameof(AllowArrange), value);
			}

			/// <summary>Node scattering parameters</summary>
			public ScatterOptions Scatter { get; }
			public class ScatterOptions :SettingsSet<ScatterOptions>
			{
				public ScatterOptions()
				{
					SpringConstant = 0.1f;
					CoulombConstant = 1f;
					FrictionConstant = 0.5f;
					ConnectorScale = 1f;
					Equilibrium = 0.01f;
				}

				/// <summary>Tuning constant for the attractive force between connected nodes</summary>
				[Description("Tuning constant for the attractive force between connected nodes")]
				public double SpringConstant
				{
					get => get<float>(nameof(SpringConstant));
					set => set(nameof(SpringConstant), value);
				}

				/// <summary>Tuning constant for the repulsive force between nodes</summary>
				[Description("Tuning constant for the repulsive force between nodes")]
				public double CoulombConstant
				{
					get => get<float>(nameof(CoulombConstant));
					set => set(nameof(CoulombConstant), value);
				}

				/// <summary>Tuning constant for the friction force that slows nodes motion during scattering</summary>
				[Description("Tuning constant for the friction force that slows nodes motion during scattering")]
				public double FrictionConstant
				{
					get => get<float>(nameof(FrictionConstant));
					set => set(nameof(FrictionConstant), value);
				}

				/// <summary>Tuning constant that multiples the coulomb force proportionally to the number of connections a node has</summary>
				[Description("Tuning constant that multiples the coulomb force proportionally to the number of connections a node has")]
				public float ConnectorScale
				{
					get => get<float>(nameof(ConnectorScale));
					set => set(nameof(ConnectorScale), value);
				}

				/// <summary>Threshold for node movements that indicates equilibrium has been reached</summary>
				[Description("Threshold for node movements that indicates equilibrium has been reached")]
				public float Equilibrium
				{
					get => get<float>(nameof(Equilibrium));
					set => set(nameof(Equilibrium), value);
				}
			}

			/// <summary>Connector</summary>
			public RelinkOptions Relink { get; }
			public class RelinkOptions :SettingsSet<RelinkOptions>
			{
				public RelinkOptions()
				{
					AnchorSharingMode = Node.EAnchorSharing.ShareAll;
				}

				/// <summary>The anchor sharing mode</summary>
				[Description("The anchor sharing mode")]
				public Node.EAnchorSharing AnchorSharingMode
				{
					get => get<Node.EAnchorSharing>(nameof(AnchorSharingMode));
					set => set(nameof(AnchorSharingMode), value);
				}
			}
		}

		/// <summary>A mouse operation for resizing selected elements</summary>
		public class MouseOpResize :ChartControl.MouseOp
		{
			// The objects being resized
			private readonly Resizee[] m_resizees;
			private class Resizee
			{
				public Resizee(Node node)
				{
					Node = node;
					InitialPosition = node.O2W;
					InitialSize = node.Size;
				}
				public Node Node;
				public m4x4 InitialPosition;
				public v4 InitialSize;
			}

			//private readonly Tools.ResizeGrabber m_grabber; // The grabber to use for resizing

			public MouseOpResize(ChartControl chart)//, Tools.ResizeGrabber grabber)
				: base(chart, allow_cancel: true)
			{
				//m_grabber = grabber;

				// Make a collection of the resizeable selected elements and their initial size
				m_resizees = Chart.Selected
					.OfType<Node>()
					.Where(x => x.Style.Resizeable)
					.Select(x => new Resizee(x))
					.ToArray();
			}
			public override void MouseDown(MouseButtonEventArgs? e)
			{
				//m_diag.Cursor = m_grabber.Cursor;
				//foreach (var node in m_resizees)
				//	node.Node.Resizing = true;
			}
			public override void MouseMove(MouseEventArgs e)
			{
				//	// The resize delta in diagram space
				//	var pt = e.GetPosition(Chart.Scene);
				//	var vec_ds = Chart.Camera.SSVecToWSVec(GrabClient, pt);
				//	//var delta = Math_.Dot(vec_ds.xy, m_grabber.Direction);
				//
				//	// Scale all of the resizeable selected elements
				//	foreach (var elem in m_resizees)
				//	{
				//		elem.Elem.PositionXY = elem.InitialPosition + (delta / 2) * m_grabber.Direction;
				//		elem.Elem.Size = new v2(
				//			Math_.Max(10, elem.InitialSize.x + delta * Math.Abs(m_grabber.Direction.x)),
				//			Math_.Max(10, elem.InitialSize.y + delta * Math.Abs(m_grabber.Direction.y)));
				//	}
				//
				//	Chart.Invalidate();
			}
			public override void MouseUp(MouseButtonEventArgs e)
			{
				Chart.Cursor = Cursors.Arrow;
				//	m_resizees.ForEach(x => x.Elem.Resizing = false);
				Chart.Invalidate();
			}
		}
	}
}
