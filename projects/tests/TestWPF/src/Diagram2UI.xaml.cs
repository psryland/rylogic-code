using System;
using System.Windows;
using Rylogic.Gfx;
using Rylogic.Gui.WPF;
using Rylogic.Gui.WPF.ChartDiagram;
using Rylogic.Maths;
using Rylogic.Utility;

namespace TestWPF
{
	public partial class Diagram2UI :Window, IChartCMenuContext, IView3dCMenuContext, IDiagramCMenuContext
	{
		public Diagram2UI()
		{
			InitializeComponent();

			var opts = new Diagram_.Options
			{
				Relink =
				{
					AnchorSharingMode = Node.EAnchorSharing.ShareSameOnly,
				}
			};
			var bs = new NodeStyle
			{
				AutoSize = true,
				Resizeable = true,
				Border = Colour32.Black,
				Fill = Colour32.LightSteelBlue,
				Hovered = Colour32.LightGreen,
				Selected = Colour32.SteelBlue,
				Text = Colour32.Black,
				TextDisabled = Colour32.Gray,
			};
			var cs = new ConnectorStyle
			{
				Line = Colour32.DarkGreen,
				Dangling = Colour32.Red,
				Selected = Colour32.SteelBlue,
				Hovered = Colour32.LightGreen,
				EdgeStyle = Connector.EEdgeStyle.Smooth,
				Width = 10.0,
			};

			// Setup the chart for diagram mode
			m_diag.Options = opts;
			var light = m_diag.Scene.Window.MainLight;
			light.On = false;
			m_diag.Scene.Window.MainLight = light;
			m_diag.Scene.Window.Ambient = Colour32.Gray;
			DiagramCMenuContext = new Diagram_.CMenu(m_diag);

			var node0 = new QuadNode("Node0\nIS A BIGGGG\n Node! \n Oh Yeaaahhh!!", position: m4x4.Translation(-10, -5, -2), style: bs) { Chart = m_diag };
			var node1 = new QuadNode("Node1", position: m4x4.Translation(10, 10, 0), style: bs) { Chart = m_diag };
			var node2 = new QuadNode("Node2", position: m4x4.Translation(20, -15, 0), style: bs) { Chart = m_diag };
			var node3 = new QuadNode("Node3", position: m4x4.Translation(-20, 20, 0), style: bs) { Chart = m_diag };
			var node4 = new QuadNode("Node4", position: m4x4.Translation(5, -20, 0), style: bs) { Chart = m_diag };
			var node5 = new QuadNode("Node5", position: m4x4.Translation(5, 5, 1), style: bs) { Chart = m_diag };

			var conn0 = new Connector(node0, node1, type:Connector.EType.BiDir, style:cs) { Chart = m_diag };
			var conn1 = new Connector(node0, node2, type: Connector.EType.BiDir, style:cs) { Chart = m_diag };
			var conn2 = new Connector(node1, node2, type: Connector.EType.BiDir, style: cs) { Chart = m_diag };
			var conn3 = new Connector(node1, node3, type: Connector.EType.Forward, style: cs) { Chart = m_diag };
			var conn4 = new Connector(node4, node2, type: Connector.EType.Back, style: cs) { Chart = m_diag };
			var conn5 = new Connector(node1, node5, type: Connector.EType.Forward, style: cs) { Chart = m_diag };
			var conn6 = new Connector(node2, node5, type: Connector.EType.Forward, style: cs) { Chart = m_diag };
			var conn7 = new Connector(node0, node5, type: Connector.EType.Forward, style: cs) { Chart = m_diag };
			var conn8 = new Connector(node3, node0, type: Connector.EType.Forward, style: cs) { Chart = m_diag };
			var conn9 = new Connector(node0, node4, type: Connector.EType.Forward, style: cs) { Chart = m_diag };
			var connA = new Connector(node3, node4, type: Connector.EType.Line, style: cs) { Chart = m_diag };

			m_diag.Scene.ContextMenu.DataContext = this;
			DataContext = this;
		}
		protected override void OnClosed(EventArgs e)
		{
			Util.DisposeRange(m_diag.Elements);
			Gui_.DisposeChildren(this, EventArgs.Empty);
			base.OnClosed(e);
		}

		/// <inheritdoc/>
		public IView3dCMenu View3dCMenuContext => m_diag.Scene.View3dCMenuContext;

		/// <inheritdoc/>
		public IChartCMenu ChartCMenuContext => m_diag.Scene.ChartCMenuContext;

		/// <inheritdoc/>
		public IDiagramCMenu DiagramCMenuContext { get; }
	}
}
