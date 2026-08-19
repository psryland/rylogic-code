using System;
using System.Collections.Generic;
using System.Collections.Specialized;
using System.Xml.Linq;
using Rylogic.Common;
using Rylogic.Extn;
using Rylogic.Utility;

namespace CoinFlip
{
	public class IndicatorContainer
	{
		// Notes:
		//  - This collection maps trade pair names to a list of indicator instances.
		//  - The indicator instances contain the data but not the graphics. Charts that
		//    are displaying a trade pair create 'views' of the indicator on the chart.
		//    These views should update when the underlying indicator instance data changes.
		//    This means an indicator instance should update on all charts that display the
		//    same trade pair.
		//  - An indicator instance should just contain the core data needed to create the
		//    graphics on a chart, such as time/price positions, width, colour, etc.

		public IndicatorContainer()
		{
			Indicators = new Dictionary<string, List<IIndicator>>();
			Load();
		}

		/// <summary>Remove and dispose all indicators</summary>
		public void Clear()
		{
			// Don't save, the caller should do that if needed
			foreach (var kv in Indicators)
				Util.DisposeRange(kv.Value);

			Indicators.Clear();
			NotifyIndicatorsChanged(NotifyCollectionChangedAction.Reset, null, null);
		}

		/// <summary>Indicators per trade pair</summary>
		private Dictionary<string, List<IIndicator>> Indicators { get; }

		/// <summary>Add data for a new indicator associated with 'pair'</summary>
		public TIndicator Add<TIndicator>(string pair_name, TIndicator indy) where TIndicator : IIndicator
		{
			// Adding 'data' for an indicator should result in a new indicator appearing on
			// any chart that displays 'pair'.
			var indicators = Indicators.GetOrAdd(pair_name, x => new List<IIndicator>());
			indicators.Add(indy);
			indy.SettingChange += HandleIndicatorSettingChange;
			Save();

			// Notify new indicator added
			NotifyIndicatorsChanged(NotifyCollectionChangedAction.Add, pair_name, indy);

			return indy;
		}

		/// <summary>Remove an indicator by id</summary>
		public void Remove(string pair_name, Guid id)
		{
			if (Indicators.TryGetValue(pair_name, out var list))
			{
				var idx = list.IndexOf(x => x.Id == id);
				if (idx != -1)
				{
					var indy = list[idx];
					list.RemoveAt(idx);
					indy.SettingChange -= HandleIndicatorSettingChange;
					Save();

					// Notify about an indicator removed
					NotifyIndicatorsChanged(NotifyCollectionChangedAction.Remove, pair_name, indy);
				}
			}
		}

		/// <summary>Get the indicators associated with 'pair'</summary>
		public IReadOnlyList<IIndicator> this[TradePair pair]
		{
			get => Indicators.TryGetValue(pair.Name, out var indy) ? indy : (IReadOnlyList<IIndicator>)new IIndicator[0];
		}

		/// <summary>Settings filepath for stored indicator configurations</summary>
		private string Filepath => Model.BackTesting 
			? Misc.ResolveUserPath("Sim", "indicators.xml")
			: Misc.ResolveUserPath("indicators.xml");

		/// <summary>Save indicator data to a settings file</summary>
		public void Save()
		{
			var root = new XElement(XmlTag.Root);
			foreach (var kv in Indicators)
			{
				var pair = root.Add2(new XElement(XmlTag.Pair)).AttrValueSet(XmlTag.Name, kv.Key);
				kv.Value.Sort(x => x.DisplayOrder);
				foreach (var indy in kv.Value)
					pair.Add2(XmlTag.Indy, indy, true);
			}
			root.Save(Filepath);
			m_save_pending = false;
		}

		/// <summary>Load indicator data from a settings file</summary>
		public void Load()
		{
			Clear();
			if (!Path_.FileExists(Filepath))
				return;

			var root = XElement.Load(Filepath);
			foreach (var elem_pair in root.Elements(XmlTag.Pair))
			{
				var pair = elem_pair.AttrValue(XmlTag.Name) ?? throw new NullReferenceException("Pair name cannot be null");
				var list = Indicators.GetOrAdd(pair, x => new List<IIndicator>());
				foreach (var elem_indy in elem_pair.Elements(XmlTag.Indy))
				{
					var indy = (IIndicator?)elem_indy.ToObject() ?? throw new NullReferenceException("Element is not an indicator");
					indy.SettingChange += HandleIndicatorSettingChange;
					list.Add(indy);
				}
			}

			NotifyIndicatorsChanged(NotifyCollectionChangedAction.Reset, null, null);
		}

		/// <summary></summary>
		public event EventHandler<IndicatorEventArgs>? CollectionChanged;
		private void NotifyIndicatorsChanged(NotifyCollectionChangedAction action, string? pair_name, IIndicator? indy) => CollectionChanged?.Invoke(this, new IndicatorEventArgs(action, pair_name, indy));

		/// <summary>When an indicator setting changes, trigger a save</summary>
		private void HandleIndicatorSettingChange(object? sender, SettingChangeEventArgs e)
		{
			if (m_save_pending) return;
			SynchronizationContext_.BeginInvokeDelayed(Save, TimeSpan.FromSeconds(1));
			m_save_pending = true;
		}
		private bool m_save_pending;

		/// <summary></summary>
		private static class XmlTag
		{
			public const string Root = "root";
			public const string Name = "name";
			public const string Pair = "pair";
			public const string Indy = "indicator";
		}
	}

	#region EventArgs
	public class IndicatorEventArgs :EventArgs
	{
		public IndicatorEventArgs(NotifyCollectionChangedAction action)
		{
			Action = action;
			PairName = string.Empty;
			Indicator = null;
		}
		public IndicatorEventArgs(NotifyCollectionChangedAction action, string? pair_name, IIndicator? indy)
		{
			Action = action;
			PairName = pair_name;
			Indicator = indy;
		}

		/// <summary></summary>
		public NotifyCollectionChangedAction Action { get; }

		/// <summary></summary>
		public string? PairName { get; }
		
		/// <summary></summary>
		public IIndicator? Indicator { get; }
	}
	#endregion
}
