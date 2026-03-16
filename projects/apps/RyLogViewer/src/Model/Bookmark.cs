using System.Collections.ObjectModel;

namespace RyLogViewer
{
	/// <summary>A bookmark at a specific line position in the log</summary>
	public class Bookmark
	{
		public Bookmark(int line_index, long file_position, string text)
		{
			LineIndex = line_index;
			FilePosition = file_position;
			Text = text;
		}

		/// <summary>The line index in the current view</summary>
		public int LineIndex { get; set; }

		/// <summary>The byte position in the file</summary>
		public long FilePosition { get; }

		/// <summary>Preview text from the bookmarked line</summary>
		public string Text { get; }

		/// <summary>Display text for the bookmark list</summary>
		public override string ToString()
		{
			return $"Line {LineIndex}: {(Text.Length > 80 ? Text[..80] + "..." : Text)}";
		}
	}

	/// <summary>Collection of bookmarks</summary>
	public class BookmarkContainer : ObservableCollection<Bookmark>
	{
		/// <summary>Add a bookmark for a line if not already bookmarked</summary>
		public void Toggle(int line_index, long file_position, string text)
		{
			// Check if already bookmarked at this position
			for (var i = Count - 1; i >= 0; --i)
			{
				if (Items[i].FilePosition == file_position)
				{
					RemoveAt(i);
					return;
				}
			}
			Add(new Bookmark(line_index, file_position, text));
		}

		/// <summary>Find the next bookmark after the given line index</summary>
		public Bookmark? FindNext(int current_line)
		{
			Bookmark? best = null;
			Bookmark? first = null;
			foreach (var bm in this)
			{
				if (first == null || bm.LineIndex < first.LineIndex)
					first = bm;
				if (bm.LineIndex > current_line && (best == null || bm.LineIndex < best.LineIndex))
					best = bm;
			}

			// Wrap around
			return best ?? first;
		}

		/// <summary>Find the previous bookmark before the given line index</summary>
		public Bookmark? FindPrev(int current_line)
		{
			Bookmark? best = null;
			Bookmark? last = null;
			foreach (var bm in this)
			{
				if (last == null || bm.LineIndex > last.LineIndex)
					last = bm;
				if (bm.LineIndex < current_line && (best == null || bm.LineIndex > best.LineIndex))
					best = bm;
			}

			// Wrap around
			return best ?? last;
		}
	}
}
