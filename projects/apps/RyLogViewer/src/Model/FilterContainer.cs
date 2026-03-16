using System.Collections.ObjectModel;
using System.Collections.Specialized;

namespace RyLogViewer
{
	public class FilterContainer : ObservableCollection<Filter>
	{
		/// <summary>Raised when filters change and the index needs rebuilding</summary>
		public event NotifyCollectionChangedEventHandler? FiltersChanged;

		/// <summary>Notify that the filters have changed</summary>
		protected override void OnCollectionChanged(NotifyCollectionChangedEventArgs e)
		{
			base.OnCollectionChanged(e);
			FiltersChanged?.Invoke(this, e);
		}

		/// <summary>Notify that an item was modified in place</summary>
		public void NotifyItemChanged()
		{
			OnCollectionChanged(new NotifyCollectionChangedEventArgs(NotifyCollectionChangedAction.Reset));
		}
	}
}

