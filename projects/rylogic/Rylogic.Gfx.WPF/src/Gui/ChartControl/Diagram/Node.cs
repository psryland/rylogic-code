using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Diagnostics;
using System.Drawing;
using System.Drawing.Imaging;
using System.Linq;
using System.Xml.Linq;
using Rylogic.Container;
using Rylogic.Extn;
using Rylogic.Gui.WPF;
using Rylogic.Maths;

namespace Rylogic.Gui.WPF.ChartDiagram;

/// <summary>Base class for all diagram node types</summary>
[DebuggerDisplay("{Description,nq}")]
public abstract class Node :ChartControl.Element, IHasStyle
{
	// Notes:
	//  - A node is a chart element set up for connecting to other node elements via connectors.
	//  - 'Node' has support for text and size, but does not imply a particular shape (2d or 3d).
	// - The texel density is controlled by Style.TexelDensity where a '1 x 1' node contains
	//   'TexelDensity x TexelDensity' pixels.

	/// <summary>Base node constructor</summary>
	/// <param name="id">Globally unique id for the element</param>
	/// <param name="size">The initial dimensions of the node</param>
	/// <param name="text">The text of the node</param>
	/// <param name="position">The position of the node on the diagram</param>
	/// <param name="style">Style properties for the node</param>
	protected Node(v4 size, Guid id, string text, m4x4? position = null, NodeStyle? style = null)
		: base(id, text, position)
	{
		Text = text;
		Style = style ?? new NodeStyle();
		SizeMax = v4.MaxValue;
		SizeMin = v4.Zero;
		Size = size;
		Connectors = new BindingListEx<Connector>();
		TextFormat = new StringFormat(0);
	}
	protected Node(XElement node)
		: base(node)
	{
		Text = node.Element(XmlTag.Text).As<string>(string.Empty);
		Style = new NodeStyle(node.Element(XmlTag.Style).As<Guid>());
		SizeMax = node.Element(XmlTag.SizeMax).As<v4>(v4.MaxValue);
		Size = node.Element(XmlTag.Size).As<v4>(v4.Zero);
		Connectors = new BindingListEx<Connector>();
		TextFormat = new StringFormat(0);

		// Be careful using Style in here, it's only a place-holder
		// instance until the element has been added to a diagram.
	}
	protected override void Dispose(bool disposing)
	{
		DetachConnectors();
		Style = null!;
		base.Dispose(disposing);
	}

	/// <summary>Export to XML</summary>
	public override XElement ToXml(XElement node)
	{
		base.ToXml(node);
		node.Add2(XmlTag.Text, Text, false);
		node.Add2(XmlTag.Style, Style.Id, false);
		node.Add2(XmlTag.SizeMax, SizeMax, false);
		node.Add2(XmlTag.Size, Size, false);
		return node;
	}
	protected override void FromXml(XElement node)
	{
		Style = new NodeStyle(node.Element(XmlTag.Style).As<Guid>());
		Text = node.Element(XmlTag.Text).As<string>(string.Empty);
		SizeMax = node.Element(XmlTag.SizeMax).As<v4>(v4.MaxValue);
		Size = node.Element(XmlTag.Size).As<v4>(v4.Zero);
		base.FromXml(node);
	}

	/// <summary>The connectors linked to this node</summary>
	public BindingListEx<Connector> Connectors { get; }

	/// <summary>Text to display in this node</summary>
	public virtual string Text
	{
		get;
		set
		{
			if (Text == value) return;
			field = value;

			if (Style.AutoSize) PerformAutoSize();
			NotifyDataChanged();
			Invalidate();
		}
	} = string.Empty;

	/// <summary>How to layout the text in the node</summary>
	public StringFormat TextFormat
	{
		get;
		set
		{
			if (field == value) return;
			field = value;

			if (Style.AutoSize) PerformAutoSize();
			Invalidate();
		}
	} = null!;

	/// <summary>The diagram space width/height/depth of the element</summary>
	public v4 Size
	{
		get;
		set
		{
			// Clamp 'value' to the size limits
			value = Math_.Clamp(value, SizeMin, SizeMax);
			if (Size == value)
				return;

			field = value;
			NotifySizeChanged();
			NotifyPropertyChanged(nameof(Size));
			Invalidate();
		}
	}

	/// <summary>Get/Set minimum size limit for auto size</summary>
	public v4 SizeMin
	{
		get;
		set
		{
			if (SizeMin == value) return;
			field = value;
			SizeMax = Math_.Max(value, SizeMax);
			Size = Size;
			NotifyPropertyChanged(nameof(SizeMin));
		}
	}

	/// <summary>Get/Set maximum size limit for auto size</summary>
	public v4 SizeMax
	{
		get;
		set
		{
			if (SizeMax == value) return;
			field = value;
			SizeMin = Math_.Min(value, SizeMin);
			Size = Size;
			NotifyPropertyChanged(nameof(SizeMax));
		}
	}

	/// <summary>Resize the node if AutoSize is enabled, otherwise no-op</summary>
	public void PerformAutoSize()
	{
		//base.Size = PreferredSize(SizeMax);
	}

	/// <summary>Return the preferred node size given the current text and size limits</summary>
	public virtual v4 PreferredSize(SizeF layout)
	{
		if (!Text.HasValue())
			return Size;

		using var img = new Bitmap(1, 1, PixelFormat.Format32bppArgb);
		using var gfx = Graphics.FromImage(img);
		v2 texel_size = gfx.MeasureString(Text, Style.Font, layout, TextFormat);

		// Convert from texels to node units and add padding
		var node_size = texel_size / Style.TexelDensity;
		node_size.x += Style.Padding.Left + Style.Padding.Right;
		node_size.y += Style.Padding.Top + Style.Padding.Bottom;

		return new v4(node_size, 0, 0);
	}
	public v4 PreferredSize(v4 size) => PreferredSize(size.xy.ToSizeF());
	public v4 PreferredSize() => PreferredSize(SizeMax);

	/// <summary>Style attributes for the node</summary>
	public NodeStyle Style
	{
		get;
		set
		{
			if (Style == value) return;
			if (field != null)
			{
				field.PropertyChanged -= HandleStyleChanged;
			}
			field = value ?? new();
			if (field != null)
			{
				field.PropertyChanged += HandleStyleChanged;
			}
			HandleStyleChanged(null, new PropertyChangedEventArgs(string.Empty));

			// Handlers
			void HandleStyleChanged(object? sender, PropertyChangedEventArgs e)
			{
				// Reassign the size to cause an auto size (if necessary)
				Size = Size;
				Invalidate();
			}
		}
	} = new();
	IStyle IHasStyle.Style => Style;

	/// <inheritdoc/>
	public override BBox Bounds => new(O2W.pos, Size / 2);

	/// <summary>Returns the position to draw text given the available 'rect', padding, and text alignment</summary>
	protected RectangleF TextLocation(Graphics gfx, RectangleF rect)
	{
		var pad = Style.Padding;
		var bnd = rect.Inflated(-pad.Left, -pad.Top, -pad.Right, -pad.Bottom);
		var tx = gfx.MeasureString(Text, Style.Font, bnd.Size, TextFormat);
		return Style.TextAlign switch
		{
			ContentAlignment.TopLeft => new RectangleF(bnd.Left, bnd.Top, tx.Width, tx.Height),
			ContentAlignment.TopCenter => new RectangleF(bnd.Left + (bnd.Width - tx.Width) * 0.5f, bnd.Top, tx.Width, tx.Height),
			ContentAlignment.TopRight => new RectangleF(bnd.Left + (bnd.Width - tx.Width), bnd.Top, tx.Width, tx.Height),
			ContentAlignment.MiddleLeft => new RectangleF(bnd.Left, bnd.Top + (bnd.Height - tx.Height) * 0.5f, tx.Width, tx.Height),
			ContentAlignment.MiddleCenter => new RectangleF(bnd.Left + (bnd.Width - tx.Width) * 0.5f, bnd.Top + (bnd.Height - tx.Height) * 0.5f, tx.Width, tx.Height),
			ContentAlignment.MiddleRight => new RectangleF(bnd.Left + (bnd.Width - tx.Width), bnd.Top + (bnd.Height - tx.Height) * 0.5f, tx.Width, tx.Height),
			ContentAlignment.BottomLeft => new RectangleF(bnd.Left, bnd.Top + (bnd.Height - tx.Height), tx.Width, tx.Height),
			ContentAlignment.BottomCenter => new RectangleF(bnd.Left + (bnd.Width - tx.Width) * 0.5f, bnd.Top + (bnd.Height - tx.Height), tx.Width, tx.Height),
			ContentAlignment.BottomRight => new RectangleF(bnd.Left + (bnd.Width - tx.Width), bnd.Top + (bnd.Height - tx.Height), tx.Width, tx.Height),
			_ => throw new ArgumentException("unknown text alignment"),
		};
	}

	/// <summary>Remove this node from the connectors that reference it</summary>
	public void DetachConnectors()
	{
		while (!Connectors.Empty())
			Connectors[0].Remove(this);
	}

	/// <summary>The locations (in node space) that connectors can attach to</summary>
	public virtual IEnumerable<AnchorPoint> AnchorPoints()
	{
		yield return new AnchorPoint(this, v4.Origin, v4.Zero, 0);
	}

	/// <summary>The anchor points available for connecting to</summary>
	public IEnumerable<AnchorPoint> AnchorPointsAvailable(EAnchorSharing sharing, Connector.EEnd end_type)
	{
		// Even though 'AnchorPoints' returns new instances of anchor points, the attached connectors
		// are accessed through the node, which means any anchor point on the same node at a given location
		// will have the same connections.
		switch (sharing)
		{
			case EAnchorSharing.ShareAll:
			{
				return AnchorPoints();
			}
			case EAnchorSharing.ShareSameOnly:
			{
				return AnchorPoints().Where(x => x.Type == Connector.EEnd.None || x.Type == end_type);
			}
			case EAnchorSharing.NoSharing:
			{
				return AnchorPoints().Where(x => x.Type == Connector.EEnd.None);
			}
			default:
			{
				throw new Exception($"Unknown sharing mode: {sharing}");
			}
		}
	}

	/// <summary>Return the attachment location and normal nearest to 'pt'.</summary>
	public virtual AnchorPoint? NearestAnchor(v4 pt, bool pt_in_node_space)
	{
		if (!pt_in_node_space)
			pt = Math_.InvertAffine(O2W) * pt;

		return AnchorPoints().MinBy(x => (x.Location - pt).LengthSq);
	}

	/// <inheritdoc/>
	public override bool CheckConsistency()
	{
		// The Connectors collection in the node should contain a connector that references this node exactly once
		foreach (var conn in Connectors)
		{
			if (conn.Node0 != this && conn.Node1 != this)
				throw new Exception($"Node {ToString()} contains connector {conn} but is not referenced by the connector");
		}

		return base.CheckConsistency();
	}

	/// <summary></summary>
	public string Description => $"Text={Text.Summary(20)}";

	/// <summary>Behaviours of anchor sharing</summary>
	public enum EAnchorSharing
	{
		/// <summary>Connectors can use anchors already in use by other connectors</summary>
		ShareAll,

		/// <summary>Connectors can use anchors already in use by other connectors if the connector direction is the same</summary>
		ShareSameOnly,

		/// <summary>Connectors don't share anchors unless there is no choice</summary>
		NoSharing,
	}
}
