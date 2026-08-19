using System;
using System.Diagnostics;
using Rylogic.Common;
using Rylogic.Gfx;

namespace Rylogic.Gui.WPF.ChartDiagram
{
	[DebuggerDisplay("{Description,nq}")]
	public class ConnectorStyle :SettingsSet<ConnectorStyle>, IHasId, IStyle
	{
		public ConnectorStyle()
			: this(Guid.NewGuid())
		{ }
		public ConnectorStyle(Guid id)
		{
			Id        = id;
			Line      = Colour32.Black;
			Hovered   = Colour32.LightGreen;
			Selected  = Colour32.Blue;
			Dangling  = Colour32.DarkRed;
			Width     = 3.0;
			MinLength = 0.3;
			EdgeStyle = Connector.EEdgeStyle.Straight;
		}

		/// <summary>Unique id for the style</summary>
		public Guid Id { get; }

		/// <summary>The colour of the line portion of the connector</summary>
		public Colour32 Line
		{
			get => get<Colour32>(nameof(Line));
			set => set(nameof(Line), value);
		}

		/// <summary>The colour of the line when selected</summary>
		public Colour32 Selected
		{
			get => get<Colour32>(nameof(Selected));
			set => set(nameof(Selected), value);
		}

		/// <summary>The colour of the line when hovered</summary>
		public Colour32 Hovered
		{
			get => get<Colour32>(nameof(Hovered));
			set => set(nameof(Hovered), value);
		}

		/// <summary>The colour of dangling connectors</summary>
		public Colour32 Dangling
		{
			get => get<Colour32>(nameof(Dangling));
			set => set(nameof(Dangling), value);
		}

		/// <summary>The (screen-space) width of the connector line</summary>
		public double Width
		{
			get => get<double>(nameof(Width));
			set => set(nameof(Width), value);
		}

		/// <summary>The minimum distance a connector sticks out from a node</summary>
		public double MinLength
		{
			get => get<double>(nameof(MinLength));
			set => set(nameof(MinLength), value);
		}

		/// <summary>True for a smooth connector, false for a straight edged connector</summary>
		public Connector.EEdgeStyle EdgeStyle
		{
			get => get<Connector.EEdgeStyle>(nameof(EdgeStyle));
			set => set(nameof(EdgeStyle), value);
		}

		/// <summary>Description</summary>
		public string Description => $"CS: {Id}";
	}
}
