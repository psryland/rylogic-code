using System;
using System.Diagnostics;
using Rylogic.Container;

namespace Rylogic.Gui.WPF
{
	public partial class ChartControl
	{
		public class HoveredCollection : BindingListEx<Element>
		{
			private readonly ChartControl m_chart;
			public HoveredCollection(ChartControl chart)
			{
				m_chart = chart;
				PerItem = false;
			}
			protected override void OnListChanging(ListChgEventArgs<Element> args)
			{
				// Handle elements added/removed from the hovered list
				var elem = args.Item;
				if (elem != null && (elem.Chart != null && elem.Chart != m_chart))
					throw new ArgumentException("element belongs to another chart");

				switch (args.ChangeType)
				{
				case ListChg.Reset:
					{
						// Clear the hovered state from all elements
						foreach (var e in m_chart.Elements)
							e.SetHoveredInternal(false, false);

						// Set the hovered state for all hovered elements
						foreach (var h in this)
							h.SetHoveredInternal(true, false);

						Debug.Assert(m_chart.CheckConsistency());
						break;
					}
				case ListChg.ItemAdded:
					{
						// Set the hovered state for the added element
						if (elem == null) throw new Exception("Item should be provided in ItemAdded events");
						elem.SetHoveredInternal(true, false);

						Debug.Assert(m_chart.CheckConsistency());
						break;
					}
				case ListChg.ItemRemoved:
					{
						// Clear the hovered state for the removed element
						if (elem == null) throw new Exception("Item should be provided in ItemRemoved events");
						elem.SetHoveredInternal(false, false);

						Debug.Assert(m_chart.CheckConsistency());
						break;
					}
				}
			}
		}
	}
}
