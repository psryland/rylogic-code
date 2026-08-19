using System;
using System.Collections.Generic;
using System.Diagnostics;
using Rylogic.Common;
using Rylogic.Extn;
using Rylogic.Gfx;
using Rylogic.Utility;

namespace Rylogic.Gui.WPF
{
	/// <summary>Interface of a graphics element representing an X-Axis range in a chart</summary>
	public interface IChartGfxPiece :IDisposable
	{
		// Notes:
		//  - See the notes for the CreatePieceHandler.
		//  - 'Range' should represent an X-axis region in the cache, *not* the size of the returned graphics.
		//    If graphics can't be created for a region, then return a piece containing null graphics and a range
		//    that covers the 'no-data' range.

		/// <summary>The X-Axis span covered by this piece</summary>
		RangeF Range { get; }
	}

	/// <summary>A cache of graphics objects that span the X-Axis</summary>
	public sealed class ChartGfxCache : IDisposable
	{
		// Notes:
		// - This cache is intended to be used by ChartDataSeries-like classes.
		//   If provides the functionality of breaking a data series up into pieces
		//   so that the limit of 64K indices is not exceeded.
		// - The cache only really works in Chart2D mode, where the camera is aligned
		//   to world space.

		public ChartGfxCache(CreatePieceHandler handler)
		{
			Pieces = new List<IChartGfxPiece>();
			CreatePiece = handler;
		}
		public void Dispose()
		{
			Pieces = null!;
		}

		/// <summary>Reset the cache</summary>
		public void Invalidate()
		{
			Util.DisposeRange(Pieces);
			Pieces.Clear();
		}
		public void Invalidate(double x)
		{
			if (Pieces.Count == 0)
				return;

			// Invalidate the piece that contains 'x'
			var idx = Pieces.BinarySearch(p => p.Range.CompareTo(x), find_insert_position: true);
			if (idx != Pieces.Count && Pieces[idx].Range.Contains(x))
			{
				Util.Dispose(Pieces[idx]);
				Pieces.RemoveAt(idx);
			}
		}
		public void Invalidate(RangeF x_range)
		{
			if (Pieces.Count == 0)
				return;

			if (x_range == RangeF.Invalid)
			{
				Invalidate();
			}
			else
			{
				// Invalidate all pieces that overlap the given range
				var beg = Pieces.BinarySearch(p => p.Range.CompareTo(x_range.Beg), find_insert_position: true);
				var end = Math.Min(Pieces.Count, Pieces.BinarySearch(p => p.Range.CompareTo(x_range.End), find_insert_position: true) + 1);
				Util.DisposeRange(Pieces, beg, end - beg);
				Pieces.RemoveRange(beg, end - beg);
			}
		}

		/// <summary>The collection of cached graphics models</summary>
		public List<IChartGfxPiece> Pieces
		{
			get;
			private set
			{
				if (field == value) return;
				Util.DisposeRange(field);
				field = value;
			}
		} = null!;

		/// <summary>
		/// Get the series data graphics that spans the given x range.
		/// Note: if there is no data for parts of 'range' the returned pieces will have 'null' graphics.</summary>
		public IEnumerable<IChartGfxPiece> Get(RangeF range)
		{
			// Return each graphics piece over the range
			for (var x = range.Beg; x < range.End;)
			{
				// If a piece cannot be created for 'x' then stop iterating because we cannot advance the range.
				// If there is no data at 'x', the returned piece should have null graphics but with a range that
				// covers the no-data range. Returning null indicates an error, not no data.
				var piece = CacheGet(x);
				if (piece == null)
					break;

				// The returned piece must span 'x' or we cannot advance
				Debug.Assert(piece.Range.End > x);
				yield return piece;
				x = piece.Range.End;
			}
		}

		/// <summary>Return the graphics piece that spans 'x'</summary>
		public IChartGfxPiece? CacheGet(double x)
		{
			// Search the cache for the model that spans 'x'
			var idx = Pieces.BinarySearch(p => p.Range.CompareTo(x));
			if (idx < 0)
			{
				idx = ~idx;

				// Get the X-range that is not cached
				var missing = new RangeF(
					idx != 0 ? Pieces[idx - 1].Range.End : double.MinValue,
					idx != Pieces.Count ? Pieces[idx].Range.Beg : double.MaxValue);

				// There is no cached graphics for 'x', create it now. If no graphics can
				// be created (because there's no data at 'x') then return null.
				Debug.Assert(missing.Contains(x));
				var piece = CreatePiece(x, missing);
				if (piece == null)
					return null;

				// Pieces should not overlap on 'Range', their graphics
				// might overlap but not the range each piece represents.
				Debug.Assert(Pieces.TrueForAll(p => p.Range.Intersect(piece.Range).Empty));
				Pieces.Insert(idx, piece);
			}
			return Pieces[idx];
		}

		/// <summary>
		/// The handler for providing pieces of the graphics.
		/// Implementers of this handler should return a graphics object that spans some region about 'x',
		/// clipped by 'missing'. The returned graphics piece should contain the x-range that the graphics
		/// represents. If there is no data at 'x' the returned piece should contain null graphics with a
		/// range covering the 'no-data' range. Note: missing.Contains(x) == true</summary>
		public CreatePieceHandler CreatePiece { get; set; }
		public delegate IChartGfxPiece CreatePieceHandler(double x, RangeF missing);
	}

	/// <summary>View3d graphics for a part of the series data</summary>
	public sealed class ChartGfxPiece :IChartGfxPiece
	{
		public ChartGfxPiece(View3d.Object? gfx, RangeF range)
		{
			Gfx = gfx;
			Range = range;
		}
		public void Dispose()
		{
			Gfx = null;
		}

		/// <summary>The model for the piece of the series data graphics</summary>
		public View3d.Object? Gfx
		{
			get;
			private set
			{
				if (field == value) return;
				Util.Dispose(ref field);
				field = value;
			}
		}

		/// <summary>The X-Axis span covered by this piece</summary>
		public RangeF Range { get; }
	}
}

