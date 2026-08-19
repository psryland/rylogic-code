using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Diagnostics;
using System.Linq;
using System.Windows;
using System.Windows.Input;
using System.Xml.Linq;
using Rylogic.Common;
using Rylogic.Extn;
using Rylogic.Gfx;
using Rylogic.Gui.WPF;
using Rylogic.LDraw;
using Rylogic.Maths;
using Rylogic.Utility;

namespace Rylogic.Gui.WPF.ChartDiagram
{
	/// <summary>
	/// A base class for connectors between elements.
	/// Connectors have two ends (nodes), either of which is allowed to be null.
	/// AnchorPoints are never null however.</summary>
	[DebuggerDisplay("{Description,nq}")]
	public class Connector :ChartControl.Element, IHasStyle
	{
		public Connector()
			: this(null, null)
		{ }
		public Connector(Node? node0, Node? node1, EType? type = null, Guid? id = null, ConnectorStyle? style = null, string? text = null)
			: base(id ?? Guid.NewGuid(), text, m4x4.Identity)
		{
			m_anc0 = new AnchorPoint();
			m_anc1 = new AnchorPoint();
			CentreOffset = v4.Zero;
			Type = type ?? EType.Line;
			Style = style ?? new ConnectorStyle();
			Gfx = new View3d.Object("*Group{}", false, Id);
			Node0 = node0;
			Node1 = node1;
		}
		public Connector(XElement node)
			: base(node)
		{
			m_anc0 = new AnchorPoint();
			m_anc1 = new AnchorPoint();
			CentreOffset = node.Element(XmlTag.CentreOffset).As<v4>(v4.Zero);
			Anc0 = node.Element(XmlTag.Anchor0).As<AnchorPoint>();
			Anc1 = node.Element(XmlTag.Anchor1).As<AnchorPoint>();
			Type = node.Element(XmlTag.Type).As<EType>();
			Style = new ConnectorStyle(node.Element(XmlTag.Style).As<Guid>());
			Gfx = new View3d.Object("*Group{}", false, Id);
		}
		protected override void Dispose(bool disposing)
		{
			DetachNodes();
			Style = null!;
			Gfx = null!;
			base.Dispose(disposing);
		}

		/// <summary>Graphics for the connector line</summary>
		private View3d.Object Gfx
		{
			get;
			set
			{
				if (field == value) return;
				Util.Dispose(ref field!);
				field = value;
			}
		} = null!;

		/// <summary>The position of the mid-point relative to the average of Anc0.LocationWS and Anc1.LocationWS</summary>
		private v4 CentreOffset { get; set; }

		/// <summary>Export to XML</summary>
		public override XElement ToXml(XElement node)
		{
			base.ToXml(node);
			node.Add2(XmlTag.Anchor0, Anc0, false);
			node.Add2(XmlTag.Anchor1, Anc1, false);
			node.Add2(XmlTag.Type, Type, false);
			node.Add2(XmlTag.Style, Style.Id, false);
			return node;
		}
		protected override void FromXml(XElement node)
		{
			Style = new ConnectorStyle(node.Element(XmlTag.Style).As<Guid>(Style.Id));
			Anc0 = node.Element(XmlTag.Anchor0).As<AnchorPoint>(Anc0);
			Anc1 = node.Element(XmlTag.Anchor1).As<AnchorPoint>(Anc1);
			base.FromXml(node);
		}

		/// <summary>The connector type</summary>
		public EType Type
		{
			get;
			set
			{
				if (field == value) return;
				field = value;
				Invalidate();
			}
		}

		/// <summary>The connector end type at Anc0 (on Node0)</summary>
		public EEnd End0 => Type.HasFlag(EType.Back) ? EEnd.Arrow : EEnd.Line;

		/// <summary>The connector end type at Anc1 (on Node1)</summary>
		public EEnd End1 => Type.HasFlag(EType.Forward) ? EEnd.Arrow : EEnd.Line;

		/// <summary>The 'from' anchor. Note: assigning to this anchor changes it's content, not the instance</summary>
		public AnchorPoint Anc0
		{
			get => m_anc0;
			set
			{
				Node0 = value.Node;
				m_anc0.Location = value.Location;
				m_anc0.Normal = value.Normal;
				m_anc0.UID = value.UID;
				NotifyPropertyChanged(nameof(Anc0));
			}
		}
		private readonly AnchorPoint m_anc0;

		/// <summary>The 'to' anchor. Note: assigning to this anchor changes it's content, not the instance</summary>
		public AnchorPoint Anc1
		{
			get => m_anc1;
			set
			{
				Node1 = value.Node;
				m_anc1.Location = value.Location;
				m_anc1.Normal = value.Normal;
				m_anc1.UID = value.UID;
				NotifyPropertyChanged(nameof(Anc1));
			}
		}
		private readonly AnchorPoint m_anc1;

		/// <summary>The 'from' then 'to' anchor points as a sequence</summary>
		public IEnumerable<AnchorPoint> Ancs
		{
			get
			{
				yield return Anc0;
				yield return Anc1;
			}
		}

		/// <summary>Get the anchor point associated with 'node'</summary>
		public AnchorPoint? Anc(Node node)
		{
			return
				Node0 == node ? Anc0 :
				Node1 == node ? Anc1 :
				null;
		}

		/// <summary>Get the anchor point not associated with 'node'</summary>
		public AnchorPoint? OtherAnc(Node node)
		{
			return
				Node0 == node ? Anc1 :
				Node1 == node ? Anc0 :
				null;
		}

		/// <summary>The 'from' node. Nodes can be null, implying the connector is dangling</summary>
		public Node? Node0
		{
			get => Anc0.Node;
			set
			{
				if (Node0 == value) return;
				if (Node0 != null)
				{
					Node0.PositionChanged -= HandlePositionChanged;
					Node0.Connectors.Remove(this);
				}
				Anc0.Node = value;
				if (Node0 != null)
				{
					Node0.Connectors.Add(this);
					Node0.PositionChanged += HandlePositionChanged;

					// Attach to the nearest anchor point on 'Node0'
					var pt = Node1?.O2W.pos ?? v4.Origin;
					var anc0 = Node0?.NearestAnchor(pt, pt_in_node_space: false) ?? AnchorPoint.Origin;
					var anc1 = Node1?.NearestAnchor(anc0.LocationWS, pt_in_node_space: false) ?? AnchorPoint.Origin;
					Anc0 = Node0?.NearestAnchor(anc1.LocationWS, pt_in_node_space: false) ?? AnchorPoint.Origin;
				}
				NotifyPropertyChanged(nameof(Node0));

				// Handlers
				void HandlePositionChanged(object? sender, EventArgs e)
				{
					Invalidate();
				}
			}
		}

		/// <summary>The 'to' node. Nodes can be null, implying the connector is connected to the diagram</summary>
		public Node? Node1
		{
			get => Anc1.Node;
			set
			{
				if (Node1 == value) return;
				if (Node1 != null)
				{
					Node1.PositionChanged -= HandlePositionChanged;
					Node1.Connectors.Remove(this);
				}
				Anc1.Node = value;
				if (Node1 != null)
				{
					Node1.Connectors.Add(this);
					Node1.PositionChanged += HandlePositionChanged;

					// Attach to the nearest anchor point on 'Node1'
					var pt = Node0?.O2W.pos ?? v4.Origin;
					var anc1 = Node1?.NearestAnchor(pt, pt_in_node_space: false) ?? AnchorPoint.Origin;
					var anc0 = Node0?.NearestAnchor(anc1.LocationWS, pt_in_node_space: false) ?? AnchorPoint.Origin;
					Anc1 = Node1?.NearestAnchor(anc0.LocationWS, pt_in_node_space: false) ?? AnchorPoint.Origin;
				}
				NotifyPropertyChanged(nameof(Node1));

				// Handlers
				void HandlePositionChanged(object? sender, EventArgs e)
				{
					Invalidate();
				}
			}
		}

		/// <summary>The non-null 'from' then 'to' nodes, as a sequence</summary>
		public IEnumerable<Node> Nodes
		{
			get
			{
				if (Node0 != null) yield return Node0;
				if (Node1 != null) yield return Node1;
			}
		}

		/// <summary>Returns the other connected node</summary>
		public Node? OtherNode(Node node)
		{
			if (Node0 == node) return Node1;
			if (Node1 == node) return Node0;
			throw new Exception($"{Description} is not connected to {node}");
		}

		/// <summary>True if the connector is not attached at both ends</summary>
		public bool Dangling => Node0 == null || Node1 == null;

		/// <summary>True if at least one end of the connector is attached to a node</summary>
		public bool Attached => Node0 != null || Node1 != null;

		/// <summary>True if this connector is attached at both ends to the same node</summary>
		public bool Loop => Node0 != null && Node0 == Node1;

		/// <summary>Detach from the nodes at each end of the connector</summary>
		public void DetachNodes()
		{
			Node0 = null;
			Node1 = null;
		}

		/// <summary>Style attributes for the connector</summary>
		public ConnectorStyle Style
		{
			get;
			set
			{
				if (Style == value) return;
				if (field != null)
				{
					field.PropertyChanged -= HandleStyleChanged;
				}
				field = value ?? new ConnectorStyle();
				if (field != null)
				{
					field.PropertyChanged += HandleStyleChanged;
				}
				Invalidate();

				// Handlers
				void HandleStyleChanged(object? sender, PropertyChangedEventArgs e)
				{
					Invalidate();
				}
			}
		} = new();
		IStyle IHasStyle.Style => Style;

		/// <inheritdoc/>
		public override BBox Bounds
		{
			get
			{
				var bounds = BBox.Reset;
				foreach (var pt in Points(true))
					BBox.Grow(ref bounds, pt);
				return bounds;
			}
		}

		/// <inheritdoc/>
		public override ChartControl.HitTestResult.Hit? HitTest(v4 chart_point, v2 scene_point, ModifierKeys modifier_keys, EMouseBtns mouse_btns, View3d.Camera cam)
		{
			if (Chart is not ChartControl chart)
				return null;

			// Convert the screen space width into a distance on the focus plane
			var sz = new v2((float)Style.Width, (float)Style.Width);
			var snap = (chart.SceneToChart(scene_point) - chart.SceneToChart(scene_point + sz)).xy.Length;

			// Hit test the element
			var ray = cam.RaySS(scene_point, View3d.ESnapMode.Edges, snap);
			var results = chart.Scene.Window.HitTest(ray, new[] { Gfx });
			if (!results.IsHit)
				return null;

			// Convert the hit point to connector space
			var pt = Math_.InvertAffine(O2W) * results.m_ws_intercept;
			var hit = new ChartControl.HitTestResult.Hit(this, pt, null);
			return hit;
		}

		/// <inheritdoc/>
		protected override void HandleClicked(ChartControl.ChartClickedEventArgs args)
		{
		//	// Only respond if selected and editing is allowed
		//	if (Selected && Diagram.AllowChanges)
		//	{
		//		// Hit point in screen space
		//		var hit_point_ds = Position.pos + new v4(hit.Point, 0, 0);
		//		var hit_point_cs = v2.From(cam.WSPointToSSPoint(hit_point_ds));
		//		var anc0_cs = v2.From(cam.WSPointToSSPoint(Anc0.LocationWS));
		//		var anc1_cs = v2.From(cam.WSPointToSSPoint(Anc1.LocationWS));
		//
		//		// If the click was at the ends of the connector and diagram editing
		//		// is allowed, detach the connector and start a move link mouse op
		//		if ((hit_point_cs - anc0_cs).LengthSq < MinCSSelectionDistanceSq)
		//		{
		//			Diagram.m_mouse_op.SetPending(1, new MouseOpMoveLink(Diagram, this, false) { StartOnMouseDown = false });
		//			return true;
		//		}
		//		if ((hit_point_cs - anc1_cs).LengthSq < MinCSSelectionDistanceSq)
		//		{
		//			Diagram.m_mouse_op.SetPending(1, new MouseOpMoveLink(Diagram, this, true) { StartOnMouseDown = false });
		//			return true;
		//		}
		//	}
		//	return false;
		}

		/// <inheritdoc/>
		protected override void HandleDragged(ChartControl.ChartDraggedEventArgs args)
		{
			Invalidate();
		}

		/// <summary>The location of the centre of the connector (in world space)</summary>
		public v4 CentreWS => PositionAt(0.5);

		/// <summary>A position on the connector at parametric time, t (in world space)</summary>
		public v4 PositionAt(double t)
		{
			return Style.EdgeStyle switch
			{
				EEdgeStyle.Straight => Math_.Lerp(t, Points(true).ToArray()),
				EEdgeStyle.Rectangular => Math_.Lerp(t, Points(true).ToArray()),
				EEdgeStyle.Smooth => Math_.Lerp(t, Spline.CreateSplines(Points(true), Spline.ETopo.Disjoint4).ToArray()),
				_ => throw new Exception($"Unknown edge style: {Style.EdgeStyle}"),
			};
		}

		/// <inheritdoc/>
		protected override void UpdateGfxCore()
		{
			// Ensure Nodes are updated before connectors
			Node0?.UpdateGfx();
			Node1?.UpdateGfx();

			// Update the transform
			var centre = 0.5f * (Anc0.LocationWS + Anc1.LocationWS);
			O2W = m4x4.Translation(centre);

			// Get the points of the connector
			var pts = Points(false);
			var ty = Type switch
			{
				EType.Line => EArrowType.Line,
				EType.Forward => EArrowType.Fwd,
				EType.Back => EArrowType.Back,
				EType.BiDir => EArrowType.FwdBack,
				_ => throw new Exception($"Unknown connector type: {Type}"),
			};
			var col =
				Selected ? Style.Selected :
				Hovered ? Style.Hovered :
				Dangling ? Style.Dangling :
				Style.Line;

			// Update the connector line graphics
			var ldr = new Builder();
			switch (Style.EdgeStyle)
			{
				case EEdgeStyle.Straight:
				{
					ldr.Line("Connector", col).arrow(ty).width(Style.Width).lines(pts);
					break;
				}
				case EEdgeStyle.Rectangular:
				{
					ldr.Line("Connector", col).arrow(ty).width(Style.Width).lines(pts);
					break;
				}
				case EEdgeStyle.Smooth:
				{
					ldr.Line("Connector", col).arrow(ty).width(Style.Width).smooth(true).lines(pts);
					break;
				}
				default:
				{
					throw new Exception($"Unknown edge style: {Style.EdgeStyle}");
				}
			}
			Gfx.UpdateModel(ldr.ToString());
		}

		/// <inheritdoc/>
		protected override void UpdateSceneCore(View3d.Window window, View3d.Camera camera)
		{
			base.UpdateSceneCore(window, camera);
			if (Gfx == null) return;

			// Bias the connectors toward the camera by a fraction of the Focus Distance
			var bias = camera.FocusDist * 0.01f;
			if (Hovered) bias *= 1.1f;
			if (Selected) bias *= 1.1f;
			
			var o2w = O2W;
			o2w.pos += camera.Orthographic
				? camera.O2W.z * bias
				: Math_.Normalise(camera.O2W.pos - o2w.pos) * bias;
			Gfx.O2P = o2w;

			// Add to the scene
			window.AddObject(Gfx);
		}

		protected override void RemoveFromSceneCore(View3d.Window window)
		{
			base.RemoveFromSceneCore(window);
			if (Gfx == null) return;
			window.RemoveObject(Gfx);
		}

		/// <summary>Detach from 'node'</summary>
		public void Remove(Node node)
		{
			if (node == null) throw new ArgumentNullException("node");
			if (Node0 == node) { Node0 = null; Invalidate(); return; }
			if (Node1 == node) { Node1 = null; Invalidate(); return; }
			throw new Exception($"Connector '{ToString()}' is not connected to node {node}");
		}

		/// <summary>Returns the corner points of the connector from Anc0 to Anc1</summary>
		private IEnumerable<v4> Points(bool world_space)
		{
			// A connector is constructed from: Anc0.LocationWS, CentreOffset, Anc1.LocationWS.
			// A line from Anc0.LocationWS is extended in the direction of Anc0.NormalWS till it
			// is perpendicular to CentreOffset. The same is done from Anc1.LocationWS
			// A connecting line between these points is then formed.

			// A connector is a line from Anc0 to Anc1, via CentreOffset.
			var origin = world_space ? v4.Zero : O2W.pos.w0;
			var beg = Anc0.LocationWS - origin;
			var end = Anc1.LocationWS - origin;
			var ctr = 0.5f * (beg + end) + CentreOffset;

			// Choose directions for the end points
			var dir0 = Anc0.NormalWS;
			var dir1 = Anc1.NormalWS;
			if (dir0 == v4.Zero) dir0 = -dir1;
			if (dir1 == v4.Zero) dir1 = -dir0;
			if (dir0 == v4.Zero) dir0 = Math_.Normalise(end - beg, v4.Zero);
			if (dir1 == v4.Zero) dir1 = Math_.Normalise(beg - end, v4.Zero);

			// Return the edge points
			switch (Style.EdgeStyle)
			{
				case EEdgeStyle.Straight:
				{
					yield return beg;
					yield return end;
					break;
				}
				case EEdgeStyle.Rectangular:
				{
					// Grow the ends towards each other
					var pts0 = new List<v4> { beg };
					var pts1 = new List<v4> { end };
					var min_length = Style.MinLength;
					for (; ; )
					{
						if (Math_.FEql(dir0, v4.Zero) || Math_.FEql(dir1, v4.Zero))
							break;

						// Go in the direction of 'dir', at least the minimum length, toward 'ctr'
						beg += Math_.Max(min_length, Math_.Dot(dir0, ctr - beg)) * dir0;
						end += Math_.Max(min_length, Math_.Dot(dir1, ctr - end)) * dir1;
						if (Math_.FEql(beg, end))
							break;

						pts0.Add(beg);
						pts1.Add(end);
						dir0 = Math_.Normalise(ProjectToAxis(ctr - beg), v4.Zero);
						dir1 = Math_.Normalise(ProjectToAxis(ctr - end), v4.Zero);
						min_length = 0;
					}
					foreach (var pt in pts0)
						yield return pt;
					foreach (var pt in pts1.Reversed())
						yield return pt;

					// Return a vector with all but the largest magnitude component set to zero
					static v4 ProjectToAxis(v4 vec)
					{
						var i = Math_.MaxElementIndex(Math_.Abs(vec));
						vec.x = i == 0 ? vec.x : 0;
						vec.y = i == 1 ? vec.y : 0;
						vec.z = i == 2 ? vec.z : 0;
						return vec;
					}
					break;
				}
				case EEdgeStyle.Smooth:
				{
					yield return beg;
					yield return beg + dir0 * Style.MinLength;
					yield return end + dir1 * Style.MinLength;
					yield return end;
					break;
				}
				default:
				{
					throw new Exception($"Unknown edge style: {Style.EdgeStyle}");
				}
			}
		}

		/// <summary>Check the self consistency of this element</summary>
		public override bool CheckConsistency()
		{
		//	// Connections to the same node are allowed
		//	// Null nodes are allowed.
		//
		//	// The connected nodes should contain a reference to this connector
		//	if (Node0 != null && Node0.Connectors.FirstOrDefault(x => x.Id == Id) == null)
		//		throw new Exception($"Connector {ToString()} is connected to node {Node0}, but that node does not contain a reference to this connector");
		//	if (Node1 != null && Node1.Connectors.FirstOrDefault(x => x.Id == Id) == null)
		//		throw new Exception($"Connector {ToString()} is connected to node {Node1}, but that node does not contain a reference to this connector");
		//
		//	// The style should be known to the diagram
		//	if (Diagram != null)
		//	{
		//		if (!Diagram.m_connector_styles.ContainsKey(Style.Id))
		//			throw new Exception($"Connector {ToString()} style is not in the diagram's style cache");
		//	}
		//
		//	return base.CheckConsistency();
		return true;
		}

		/// <summary>Debugging description</summary>
		public string Description => $"Connector[{Anc0.Description} -> {Anc1.Description}]";

		/// <summary>Between same nodes ignoring direction, equality comparer</summary>
		public static bool HasSameNodes(Connector lhs, Connector rhs)
		{
			if (lhs.Node0 == null || lhs.Node1 == null) return false;
			if (rhs.Node0 == null || rhs.Node1 == null) return false;
			return
				(lhs.Node0 == rhs.Node0 && lhs.Node1 == rhs.Node1) ||
				(lhs.Node0 == rhs.Node1 && lhs.Node1 == rhs.Node0);
		}

		/// <summary>Between same nodes not ignoring direction, equality comparer</summary>
		public static bool HasSameNodesAndDirection(Connector lhs, Connector rhs)
		{
			if (lhs.Node0 == null || lhs.Node1 == null) return false;
			if (rhs.Node0 == null || rhs.Node1 == null) return false;

			// Omnidirectional connectors
			if ((lhs.Type == EType.Line && rhs.Type == EType.Line) ||
				(lhs.Type == EType.BiDir && rhs.Type == EType.BiDir))
				return
					(lhs.Node0 == rhs.Node0 && lhs.Node1 == rhs.Node1) ||
					(lhs.Node0 == rhs.Node1 && lhs.Node1 == rhs.Node0);

			// Unidirectional connectors
			if (lhs.Type == EType.Forward)
				return
					(rhs.Type == EType.Forward && lhs.Node0 == rhs.Node0 && lhs.Node1 == rhs.Node1) ||
					(rhs.Type == EType.Back    && lhs.Node0 == rhs.Node1 && lhs.Node1 == rhs.Node0);

			if (lhs.Type == EType.Back)
				return
					(rhs.Type == EType.Back && lhs.Node0 == rhs.Node0 && lhs.Node1 == rhs.Node1) ||
					(rhs.Type == EType.Forward && lhs.Node0 == rhs.Node1 && lhs.Node1 == rhs.Node0);

			throw new Exception($"Unknown connector type: {lhs.Type}");
		}

		/// <summary>Connector end types</summary>
		public enum EEnd
		{
			/// <summary>Used for AnchorPoints that aren't connected to connectors</summary>
			None,

			/// <summary>Simple line</summary>
			Line,

			/// <summary>An arrow pointing into the node</summary>
			Arrow,

			/// <summary>Special value used by AnchorPoints when a mixture of connector types are connected</summary>
			Mixed,
		}

		/// <summary>Connector types</summary>
		[Flags]
		public enum EType
		{
			Line = 0,
			Forward = 1 << 0,
			Back = 1 << 1,
			BiDir = Forward | Back,
		}

		/// <summary>Styles for connectors</summary>
		public enum EEdgeStyle
		{
			/// <summary>Straight lines directly from anchor to anchor</summary>
			Straight,

			/// <summary>Straight horizontal and vertical lines from anchor to anchor</summary>
			Rectangular,

			/// <summary>A spline curve from from anchor to anchor</summary>
			Smooth,
		}
	}
}
