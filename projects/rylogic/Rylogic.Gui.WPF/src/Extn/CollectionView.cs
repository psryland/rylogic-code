using System;
using System.ComponentModel;

namespace Rylogic.Gui.WPF
{
	public static class CollectionView_
	{
		/// <summary>Return the source collection as a given type</summary>
		public static T Source<T>(this ICollectionView collection)
		{
			return collection.SourceCollection is T src ? src : throw new InvalidCastException($"Source collection is not an instance of {typeof(T).Name}");
		}

		/// <summary>Fluent MoveCurrentToFirst</summary>
		public static ICollectionView MoveCurrentToFirst2(this ICollectionView collection)
		{
			collection.MoveCurrentToFirst();
			return collection;
		}

		/// <summary>Move the current position to 'item' or the first item if 'item' is not found</summary>
		public static bool MoveCurrentToOrFirst(this ICollectionView collection, object? item)
		{
			return item != null
				? collection.MoveCurrentTo(item)
				: collection.MoveCurrentToFirst();
		}

		/// <summary>Access the current item (or null)</summary>
		public static T CurrentAs<T>(this ICollectionView collection)
		{
			return (T)collection.CurrentItem;
		}
	}
}
