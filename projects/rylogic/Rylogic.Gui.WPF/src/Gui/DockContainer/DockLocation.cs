using System.Linq;
using System.Windows;
using System.Xml.Linq;
using Rylogic.Extn;

namespace Rylogic.Gui.WPF
{
	using DockContainerDetail;

	public partial class DockContainer
	{
		/// <summary>A record of a dock location within the dock container</summary>
		public class DockLocation
		{
			public DockLocation(EDockSite[]? address = null, int index = int.MaxValue, EDockSite? auto_hide = null, int? float_window_id = null, Rect? float_bounds = null)
			{
				Address = address ?? new[] { EDockSite.Centre };
				Index = index;
				AutoHide = auto_hide;
				FloatingWindowId = float_window_id;
				FloatingBounds = float_bounds;
			}
			public DockLocation(XElement node)
			{
				Address = node.Element(XmlTag.Address).As<string>().Split(',').Select(x => Enum<EDockSite>.Parse(x)).ToArray();
				Index = node.Element(XmlTag.Index).As<int>(int.MaxValue);
				AutoHide = node.Element(XmlTag.AutoHide).As<EDockSite?>(def: null);
				FloatingWindowId = node.Element(XmlTag.FloatingWindow).As<int?>(def: null);
				FloatingBounds = node.Element(XmlTag.Bounds)?.As<Rect>();
			}
			public XElement ToXml(XElement node)
			{
				// Add info about the host of this content
				if (FloatingWindowId != null)
				{
					// If hosted in a floating window, record the window id
					node.Add2(XmlTag.FloatingWindow, FloatingWindowId.Value, false);
				}
				else if (FloatingBounds != null)
				{
					// If hosted in a new floating window, record its screen bounds
					node.Add2(XmlTag.Bounds, FloatingBounds.Value, false);
				}
				else if (AutoHide != null)
				{
					// If hosted in an auto hide panel, record the panel id
					node.Add2(XmlTag.AutoHide, AutoHide.Value, false);
				}
				else
				{
					// Otherwise we're hosted in the main dock container by default
				}

				// Add the location of where in the host this content is stored
				node.Add2(XmlTag.Address, string.Join(",", Address), false);
				node.Add2(XmlTag.Index, Index, false);
				return node;
			}

			/// <summary>The location within the host's tree</summary>
			public EDockSite[] Address { get; set; }

			/// <summary>The index within the content of the dock pace at 'Address'</summary>
			public int Index { get; set; }

			/// <summary>The auto hide site (or null if not in an auto site location)</summary>
			public EDockSite? AutoHide { get; set; }

			/// <summary>The Id of a floating window to dock to (or null if not in a floating window)</summary>
			public int? FloatingWindowId { get; set; }

			/// <summary>The screen bounds of a new floating window to dock to. Used when 'FloatingWindowId' is null (null if not floating)</summary>
			public Rect? FloatingBounds { get; set; }

			/// <summary></summary>
			public override string ToString()
			{
				var addr = string.Join(",", Address);
				if (FloatingWindowId != null) return $"Floating Window {FloatingWindowId.Value}: {addr} (Index:{Index})";
				if (FloatingBounds != null) return $"New Floating Window {FloatingBounds.Value}: {addr} (Index:{Index})";
				if (AutoHide != null) return $"Auto Hide {AutoHide.Value}: {addr} (Index:{Index})";
				return $"Dock Container: {addr} (Index:{Index})";
			}

			/// <summary>Implicit conversion from array of dock sites to a doc location</summary>
			public static implicit operator DockLocation(EDockSite[] site)
			{
				return new DockLocation(address: site);
			}
		}
	}
}
