using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Xml.Linq;
using Rylogic.Extn;
using Rylogic.Gui.WPF;
using Rylogic.Maths;

namespace Rylogic.Gui.WPF.ChartDiagram
{
	/// <summary>A point that a connector can connect to</summary>
	[DebuggerDisplay("{Description,nq}")]
	public class AnchorPoint
	{
		// Notes:
		//  - An anchor point can be orphaned, allowing connectors to be dangling.
		public const int NoUID = -1;

		public AnchorPoint()
			: this(null, v4.Origin, v4.YAxis, NoUID)
		{ }
		public AnchorPoint(AnchorPoint rhs)
			: this(rhs.m_node, rhs.m_location, rhs.m_normal, rhs.UID)
		{ }
		public AnchorPoint(Node? node, v4 loc, v4 norm, int uid)
		{
			m_node = node;
			m_location = loc;
			m_normal = norm;
			UID = uid;
		}
		public AnchorPoint(IDictionary<Guid, Node> nodes, XElement node)
			: this()
		{
			var id = node.Element(XmlTag.ElementId).As<Guid>();
			m_node = nodes.TryGetValue(id, out m_node) ? m_node : null;
			m_location = node.Element(XmlTag.Location).As<v4>(v4.Origin);
		}

		/// <summary>A default anchor at the chart origin</summary>
		public static readonly AnchorPoint Origin = new(null, v4.Origin, v4.Zero, NoUID);

		/// <summary>Export to XML</summary>
		public XElement ToXml(XElement node)
		{
			node.Add2(XmlTag.ElementId, Node != null ? Node.Id : Guid.Empty, false);
			node.Add2(XmlTag.Location, Location, false);
			return node;
		}

		/// <summary>The node this anchor point is attached to. Null when the anchor point is orphaned</summary>
		public Node? Node
		{
			get => m_node;
			set
			{
				if (m_node == value) return;

				// When set to null, record the last diagram space position so that
				// anything connected to this anchor point doesn't snap to the origin.
				if (value == null)
				{
					Location = LocationWS;
					Normal = NormalWS;
				}

				m_node = value;
			}
		}
		private Node? m_node;

		/// <summary>Get/Set the node-relative position of the anchor (in node space)</summary>
		public v4 Location
		{
			get => m_location;
			set
			{
				if (m_location == value) return;
				m_location = value;
			}
		}
		private v4 m_location;

		/// <summary>Get/Set the node-relative anchor normal (can be zero)</summary>
		public v4 Normal
		{
			get => m_normal;
			set
			{
				if (m_normal == value) return;
				m_normal = value;
			}
		}
		private v4 m_normal;

		/// <summary>An id assigned by the creator of the Anchor point for easy identification</summary>
		public int UID;

		/// <summary>Get the diagram space position of the anchor</summary>
		public v4 LocationWS => Node?.O2W * Location ?? Location;

		/// <summary>Get the diagram space anchor normal (can be zero)</summary>
		public v4 NormalWS => Node?.O2W * Normal ?? Normal;

		/// <summary>Get the connectors attached to this anchor point</summary>
		public IEnumerable<Connector> Connectors => Node?.Connectors.Where(x => x.Anc0 == this || x.Anc1 == this) ?? Array.Empty<Connector>();

		/// <summary>The connector end type based on the connectors linked to this anchor point</summary>
		public Connector.EEnd Type
		{
			get
			{
				// Find the union of connector types for all connectors attached to this node
				var type = Connector.EEnd.None;
				foreach (var conn in Connectors)
				{
					if (conn.Anc0 == this)
					{
						if (type == Connector.EEnd.None) type = conn.End0;
						if (type != conn.End0) return Connector.EEnd.Mixed;
					}
					if (conn.Anc1 == this)
					{
						if (type == Connector.EEnd.None) type = conn.End1;
						if (type != conn.End1) return Connector.EEnd.Mixed;
					}
				}
				return type;
			}
		}

		/// <summary></summary>
		public string Description => $"{UID} '{Node?.Description ?? "dangling"}' Type={Type}";

		/// <summary></summary>
		public static bool operator ==(AnchorPoint? left, AnchorPoint? right) => Equals(left, right);
		public static bool operator !=(AnchorPoint? left, AnchorPoint? right) => !Equals(left, right);
		public bool Equals(AnchorPoint ap)
		{
			// Anchor points are equal if they're on the same object and at the same location
			return
				ap != null &&
				Node == ap.Node &&
				Location == ap.Location &&
				Normal == ap.Normal;
		}
		public override bool Equals(object? obj)
		{
			return obj is AnchorPoint ap && Equals(ap);
		}
		public override int GetHashCode()
		{
			return new { Node, Location, Normal }.GetHashCode();
		}
	}
}
