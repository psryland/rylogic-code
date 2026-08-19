using System;
using System.ComponentModel;
using System.Diagnostics;
using System.Drawing;
using Rylogic.Common;
using Rylogic.Gfx;

namespace Rylogic.Gui.WPF.ChartDiagram
{
	[DebuggerDisplay("{Description,nq}")]
	public class NodeStyle :SettingsSet<NodeStyle>, IHasId, IStyle
	{
		public NodeStyle()
			: this(Guid.NewGuid())
		{ }
		public NodeStyle(Guid id)
		{
			Id            = id;
			AutoSize      = true;
			Resizeable    = true;
			Border        = Colour32.Black;
			Fill          = Colour32.WhiteSmoke;
			Hovered       = Colour32.LightGreen;
			Selected      = Colour32.Blue;
			Disabled      = Colour32.LightGray;
			Text          = Colour32.Black;
			TextDisabled  = Colour32.DarkGray;
			CornerRadius  = 0.1;
			AnchorSpacing = 0.2;
			TexelDensity  = 256.0;
			TextAlign     = ContentAlignment.MiddleCenter;
			Font          = new Font(FontFamily.GenericSansSerif, 36f, GraphicsUnit.Point);
			Padding       = RectangleF.FromLTRB(0.1f, 0.1f, 0.1f, 0.1f);
		}

		/// <summary>Unique id for the style</summary>
		public Guid Id { get; }

		/// <summary>True if the node's size is determined automatically from its content</summary>
		[Description("True if the node resizes automatically to fit the content")]
		public bool AutoSize
		{
			get => get<bool>(nameof(AutoSize));
			set => set(nameof(AutoSize), value);
		}

		/// <summary>True if the node can be resized by the user</summary>
		[Description("True if the node can be resized by the user")]
		public bool Resizeable
		{
			get => get<bool>(nameof(Resizeable));
			set => set(nameof(Resizeable), value);
		}

		/// <summary>The colour of the node border</summary>
		[Description("The colour of a node border")]
		public Colour32 Border
		{
			get => get<Colour32>(nameof(Border));
			set => set(nameof(Border), value);
		}

		/// <summary>The node background colour</summary>
		[Description("The normal background colour of a node")]
		public Colour32 Fill
		{
			get => get<Colour32>(nameof(Fill));
			set => set(nameof(Fill), value);
		}

		/// <summary>The colour of the node when hovered</summary>
		[Description("The colour of a node when hovered")]
		public Colour32 Hovered
		{
			get => get<Colour32>(nameof(Hovered));
			set => set(nameof(Hovered), value);
		}

		/// <summary>The colour of the node when selected</summary>
		[Description("The colour of a node when selected")]
		public Colour32 Selected
		{
			get => get<Colour32>(nameof(Selected));
			set => set(nameof(Selected), value);
		}

		/// <summary>The colour of the node when disabled</summary>
		[Description("The colour of a node when disabled")]
		public Colour32 Disabled
		{
			get => get<Colour32>(nameof(Disabled));
			set => set(nameof(Disabled), value);
		}

		/// <summary>The node text colour</summary>
		[Description("The normal colour of text")]
		public Colour32 Text
		{
			get => get<Colour32>(nameof(Text));
			set => set(nameof(Text), value);
		}

		/// <summary>The node text colour when disabled</summary>
		[Description("The colour of text when the node is disabled")]
		public Colour32 TextDisabled
		{
			get => get<Colour32>(nameof(TextDisabled));
			set => set(nameof(TextDisabled), value);
		}

		/// <summary>Corner radius. Interpretation depends on node type</summary>
		[Description("The rounding of corners on a node")]
		public double CornerRadius
		{
			get => get<double>(nameof(CornerRadius));
			set => set(nameof(CornerRadius), value);
		}

		/// <summary>This distance between snap-to points on a node</summary>
		[Description("This distance between snap-to points on a node")]
		public double AnchorSpacing
		{
			get => get<double>(nameof(AnchorSpacing));
			set => set(nameof(AnchorSpacing), value);
		}

		/// <summary>The texel density on the node, i.e. a '1 x 1' node contains 'TexelDensity x TexelDensity' pixels</summary>
		[Description("The texel density on the node, i.e. a '1 x 1' node contains 'TexelDensity x TexelDensity' pixels")]
		public double TexelDensity
		{
			get => get<double>(nameof(TexelDensity));
			set => set(nameof(TexelDensity), value);
		}

		/// <summary>The alignment of the text within the node</summary>
		[Description("The alignment of text within the node")]
		public ContentAlignment TextAlign
		{
			get => get<ContentAlignment>(nameof(TextAlign));
			set => set(nameof(TextAlign), value);
		}

		/// <summary>The font to use for the node text</summary>
		[Description("The font to use for the node text")]
		public Font Font
		{
			get => get<Font>(nameof(Font));
			set => set(nameof(Font), value);
		}

		/// <summary>The padding surrounding the text in the node (in node dimensions, not texels)</summary>
		[Description("The padding between the node edge and the contained text (in node dimensions, not texels)")]
		public RectangleF Padding
		{
			get => get<RectangleF>(nameof(Padding));
			set => set(nameof(Padding), value);
		}

		/// <summary></summary>
		public string Description => $"NS: {Id}";
	}
}
