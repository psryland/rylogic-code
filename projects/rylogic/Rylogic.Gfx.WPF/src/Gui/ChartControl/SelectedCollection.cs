using System;
using System.Diagnostics;
using Rylogic.Container;

namespace Rylogic.Gui.WPF
{
	public partial class ChartControl
	{
		public class SelectedCollection : BindingListEx<Element>
		{
			private readonly ChartControl m_chart;
			public SelectedCollection(ChartControl chart)
			{
				m_chart = chart;
				PerItem = false;
			}
			protected override void OnListChanging(ListChgEventArgs<Element> args)
			{
				// Handle elements added/removed from the selection list
				var elem = args.Item;
				if (elem != null && (elem.Chart != null && elem.Chart != m_chart))
					throw new ArgumentException("element belongs to another chart");

				switch (args.ChangeType)
				{
					case ListChg.Reset:
					{
						// Clear the selected state from all elements
						foreach (var e in m_chart.Elements)
							e.SetSelectedInternal(false, false);

						// Set the selected state on all selected elements
						foreach (var e in this)
							e.SetSelectedInternal(true, false);

						Debug.Assert(m_chart.CheckConsistency());
						break;
					}
					case ListChg.ItemAdded:
					{
						// Set the selected state on the added element
						if (elem == null) throw new Exception("ItemAdded should provide the added element");
						elem.SetSelectedInternal(true, false);

						Debug.Assert(m_chart.CheckConsistency());
						break;
					}
					case ListChg.ItemRemoved:
					{
						// Clear the selected state from the removed element
						if (elem == null) throw new Exception("ItemRemoved should provide the removed element");
						elem.SetSelectedInternal(false, false);

						Debug.Assert(m_chart.CheckConsistency());
						break;
					}
				}
			
				base.OnListChanging(args);
			}
		}
	}
}
