using System;
using System.Collections.Generic;
using System.Diagnostics;
using Rylogic.Container;

namespace Rylogic.Gui.WPF
{
	public partial class ChartControl
	{
		public class ElementCollection : BindingListEx<Element>
		{
			private readonly ChartControl m_chart;
			public ElementCollection(ChartControl chart)
			{
				m_chart = chart;
				PerItem = true;
				UseHashSet = true;
				Ids = new Dictionary<Guid, Element>();
			}
			protected override void OnListChanging(ListChgEventArgs<Element> args)
			{
				var elem = args.Item;
				if (elem != null && (elem.Chart != null && elem.Chart != m_chart))
					throw new ArgumentException("element belongs to another chart");

				switch (args.ChangeType)
				{
					case ListChg.Reset:
					{
						// Remove all elements from the chart
						foreach (var e in this)
							e.SetChartInternal(m_chart, false);

						// Repopulate the IDs collection
						Ids.Clear();
						foreach (var e in this)
							Ids.Add(e.Id, e);

						Debug.Assert(m_chart.CheckConsistency());
						break;
					}
					case ListChg.ItemAdded:
					{
						// Remove from any previous chart
						if (elem == null) throw new Exception("ItemAdded should provide the added element");
						elem.SetChartInternal(m_chart, false);

						// Track the ID
						Ids.Add(elem.Id, elem);

						Debug.Assert(m_chart.CheckConsistency());
						break;
					}
					case ListChg.ItemPreRemove:
					{
						// Remove from selected or hovered sets before removing form the collection
						if (elem == null) throw new Exception("ItemPreRemove should provide the element to be removed");
						elem.Selected = false;
						elem.Hovered = false;

						Debug.Assert(m_chart.CheckConsistency());
						break;
					}
					case ListChg.ItemRemoved:
					{
						// Remove from the chart
						if (elem == null) throw new Exception("ItemRemoved should provide the removed element");
						elem.SetChartInternal(null, false);

						// Forget the ID
						Ids.Remove(elem.Id);

						Debug.Assert(m_chart.CheckConsistency());
						break;
					}
				}
				base.OnListChanging(args);
			}

			/// <summary>A map from unique id to element</summary>
			public Dictionary<Guid, Element> Ids { get; }
		}
	}
}
