using System.Collections.ObjectModel;
using System.Collections.Specialized;

namespace RyLogViewer
{
	public class HighLightContainer : ObservableCollection<Highlight>
	{
		/// <summary>Raised when highlights change and the display needs updating</summary>
		public event NotifyCollectionChangedEventHandler? HighlightsChanged;

		/// <summary>Notify that the highlights have changed</summary>
		protected override void OnCollectionChanged(NotifyCollectionChangedEventArgs e)
		{
			base.OnCollectionChanged(e);
			HighlightsChanged?.Invoke(this, e);
		}

		/// <summary>Notify that an item was modified in place</summary>
		public void NotifyItemChanged()
		{
			OnCollectionChanged(new NotifyCollectionChangedEventArgs(NotifyCollectionChangedAction.Reset));
		}
	}
}
