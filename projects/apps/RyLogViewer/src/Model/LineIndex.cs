using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Text;
using System.Threading;
using System.Windows.Threading;
using Rylogic.Common;
using Rylogic.Extn;
using Rylogic.Maths;
using RyLogViewer.Options;

namespace RyLogViewer
{
	public class LineIndex
	{
		// Notes:
		//  - This class is a cache of text line data to byte range within the log.

		private readonly List<RangeI> m_line_index; // The byte ranges of lines in a file
		private readonly Dispatcher m_dispatcher;  // Message dispatcher
		private State m_state;                     // State data for the line index
		private int m_issue;                       // Issue number for builds

		public LineIndex(ILogDataSource src, Encoding encoding, byte[] line_end, Settings settings)
		{
			Src = src;
			m_line_index = new List<RangeI>(settings.LogData.LineCacheCount);
			m_dispatcher = Dispatcher.CurrentDispatcher;

			m_state = new State
			{
				m_encoding = encoding,
				m_line_end = line_end,
				m_file_buffer_size = settings.LogData.FileBufSize,
				m_line_cache_count = settings.LogData.LineCacheCount,
				m_max_line_length = settings.LogData.MaxLineLength,
				m_ignore_blanks = settings.LogData.IgnoreBlankLines,
			};
		}

		/// <summary>The log data source</summary>
		public ILogDataSource Src { get; private set; }

		/// <summary>The encoding format being used</summary>
		public Encoding Encoding => m_state.m_encoding;

		/// <summary>The line end byte sequence being used</summary>
		public byte[] LineEnd => m_state.m_line_end;

		/// <summary>The number of lines in the line index</summary>
		public int Count => m_line_index.Count;

		/// <summary>Return the byte range of a line. Negative values mean 'Count - index'</summary>
		public RangeI this[int index]
		{
			get
			{
				if (index < -Count || index >= Count) throw new IndexOutOfRangeException($"Line index ({index}) outside range [{-Count},{Count})");
				return index >= 0 ? m_line_index[index] : m_line_index[Count + index];
			}
		}

		/// <summary>Returns the byte range of the complete file, last time we checked its length. Begin is always 0</summary>
		public RangeI FileByteRange => new(0, m_state.m_fileend);

		/// <summary>
		/// Returns the byte range of the file currently covered by this cache.
		/// Note: the range is between starts of lines, not the full range. This is 
		/// the only range we know is complete and doesn't contain partial lines</summary>
		public RangeI LineStartRange => m_state.m_line_start_range;

		/// <summary>Notify of a build progress</summary>
		public event EventHandler<BuildProgressEventArgs>? BuildProgress;

		/// <summary>Notify of a build completing</summary>
		/// todo Watch.CheckForChangedFiles(); // On completion, check if the file has changed again and rerun if it has
		public event EventHandler<BuildCompleteEventArgs>? BuildComplete;

		/// <summary>Notify of a build error</summary>
		public event EventHandler<BuildErrorEventArgs>? BuildError;

		/// <summary>Returns true if the build is cancelled</summary>
		public bool IsBuildCancelled(int issue) // Worker thread context
		{
			// This only writes to 'm_issue' when 'issue' == 'm_issue'
			return Interlocked.CompareExchange(ref m_issue, issue, issue) != issue;
		}

		/// <summary>
		/// Updates the line index centred around 'filepos'.
		/// If 'filepos' is within the current byte range of 'm_line_index' then an incremental search
		/// for lines is done in the direction needed to re-centre the line list around 'filepos'.
		/// If 'reload' is true a full rebuild of the cache is done</summary>
		public void Build(long filepos, bool reload, IList<IFilter> filters)
		{
			// Incremental updates cannot interrupt full reloads
			if (m_reload_in_progress && reload == false) return;
			m_reload_in_progress = reload;

			// Increment the build issue number to cancel any previous builds
			Interlocked.Increment(ref m_issue);
			var issue = m_issue;

			// Make a copy of the current state
			var state = m_state;

			// The index that 'filepos' would have in 'm_line_index'.
			// -1 if 'filepos' is before the current cached range,
			// or 'm_line_index.Count' if after
			state.m_index_centre = FindIndex(m_line_index, filepos);
			state.m_index_count = m_line_index.Count;

			// Make a copy of the filters
			state.m_filters = new List<IFilter>(filters);

			// Start a build in a worker thread
			ThreadPool.QueueUserWorkItem(BuildWorker);
			void BuildWorker(object? _)
			{
				try
				{
					// Open a new stream to the log data
					using var src = Src.OpenStream();

					// Determine new state properties of the file
					state.m_fileend = src.Length;
					state.m_filepos = Math_.Clamp(filepos, 0, state.m_fileend);

					// Do the update
					var lines = BuildAsync(ref state, src, reload, Progress);

					// Add the lines to the line index
					m_dispatcher.BeginInvoke(new Action(() =>
					{
						MergeResults(state, lines, reload, Progress);
					}));
				}
				catch (Exception ex)
				{
					m_dispatcher.BeginInvoke(new Action(() =>
					{
						BuildError?.Invoke(this, new BuildErrorEventArgs(ex));
					}));
				}
				finally
				{
					m_dispatcher.BeginInvoke(new Action(() =>
					{
						if (!Progress(1, 1)) return;
						m_reload_in_progress = false;
					}));
				}
			}
			bool Progress(long scanned, long length)
			{
				m_dispatcher.BeginInvoke(new Action(() =>
				{
					if (IsBuildCancelled(issue)) return;
					BuildProgress?.Invoke(this, new BuildProgressEventArgs(scanned, length));
				}));
				return !IsBuildCancelled(issue);
			}
		}
		private bool m_reload_in_progress;

		/// <summary>Called when building the line index completes (success or failure)</summary>
		private void MergeResults(State d, List<RangeI> n_line_index, bool reload, ProgressFunc progress)
		{
			// This method runs in the main thread, so if the build issue is the same at
			// the start of this method it can't be changed until after this function returns.
			if (!progress(0, n_line_index.Count))
				return;

			// Both old and new ranges are expected to be contiguous
			var old_rng = Tools.GetRange(m_line_index);
			var new_rng = Tools.GetRange(n_line_index);
			int row_delta = 0;

			// Replace 'm_line_index' with 'line_index'
			if (reload)
			{
				// Try to determine the row delta
				// Use any range overlap to work out the row delta.
				// If the ranges overlap, we can search for the start address of the intersection in both
				// ranges to get the row delta. If the don't overlap, the best we can do is say the direction.
				var intersect = old_rng.Intersect(new_rng);
				if (!intersect.Empty)
				{
					var old_idx = FindIndex(m_line_index, intersect.Beg);
					var new_idx = FindIndex(n_line_index, intersect.Beg);
					row_delta = new_idx - old_idx;
				}
				else
				{
					row_delta = intersect.Beg == old_rng.End ? -n_line_index.Count : n_line_index.Count;
				}

				// Replace the line index
				m_line_index.Assign(n_line_index);
			}

			// The new range is to the left of the old range
			else if (new_rng.Beg < old_rng.Beg && new_rng.End <= old_rng.End)
			{
				// Make sure there's no overlap by removing data from 'm_line_index'
				var trim = 0;
				for (; trim != m_line_index.Count && m_line_index[trim].Beg < new_rng.End; ++trim) { }
				m_line_index.RemoveRange(0, trim);
				row_delta -= trim;

				// Insert the new lines
				m_line_index.InsertRange(0, n_line_index);
				row_delta += n_line_index.Count;

				// Trim the tail
				if (m_line_index.Count > d.m_line_cache_count)
					m_line_index.RemoveRange(d.m_line_cache_count, m_line_index.Count - d.m_line_cache_count);
			}

			// The new range is to the right of the old range
			else if (new_rng.Beg >= old_rng.Beg && new_rng.End > old_rng.End)
			{
				// Make sure there's no overlap by removing data from 'm_line_index'
				var trim = 0;
				for (; trim != m_line_index.Count && m_line_index.Back(trim).End > new_rng.Beg; ++trim) { }
				m_line_index.RemoveRange(m_line_index.Count - trim, trim);

				// Insert the new lines
				m_line_index.InsertRange(m_line_index.Count, n_line_index);

				// Trim the head
				if (m_line_index.Count > d.m_line_cache_count)
				{
					row_delta -= m_line_index.Count - d.m_line_cache_count;
					m_line_index.RemoveRange(0, m_line_index.Count - d.m_line_cache_count);
				}
			}

			// Update the line start range to reflect the cached lines
			if (m_line_index.Count > 0)
				d.m_line_start_range = new RangeI(m_line_index[0].Beg, m_line_index[m_line_index.Count - 1].Beg);
			else
				d.m_line_start_range = RangeI.Zero;

			// Save the new index state
			m_state = d;

			// Notify of update complete
			BuildComplete?.Invoke(this, new BuildCompleteEventArgs(new_rng, row_delta));
		}

		/// <summary>Update the line index</summary>
		private static List<RangeI> BuildAsync(ref State d, Stream src, bool reload, ProgressFunc progress) // Worker thread context
		{
			// This method runs in a background thread
			// All we're doing here is loading data around 'd.filepos' so that there are an equal number
			// of lines on either side. This can be optimised however because the existing range of
			// cached data probably overlaps the range we want loaded.
			if (!progress(0, d.m_fileend))
				return new List<RangeI>();

			// A temporary buffer for reading sections of the file
			var buf = new byte[d.m_max_line_length];

			// Seek to the first line that starts immediately before 'filepos'
			d.m_filepos = FindLineStart(src, d.m_filepos, d.m_fileend, d.m_line_end, d.m_encoding, buf);

			// Determine the range to scan and the number of lines in each direction
			var bwd_first = (d.m_fileend - d.m_filepos) > (d.m_filepos - 0); // scan in the most bound direction first
			var scan_range = CalcBufferRange(d.m_filepos, d.m_fileend, d.m_file_buffer_size);
			var line_range = CalcLineRange(d.m_line_cache_count);
			var bwd_lines = line_range.Begi;
			var fwd_lines = line_range.Endi;

			// Incremental loading - only load what isn't already cached.
			// If the 'filepos' is left of the cache centre, try to extent in left direction first.
			// If the scan range in that direction is empty, try extending at the other end. The
			// aim is to try to get d.line_index_count as close to d.line_cache_count as possible
			// without loading data that is already cached.
			if (!reload && !d.m_line_start_range.Empty)
				AdjustForIncrementalBuild(d, ref bwd_first, ref scan_range, ref bwd_lines, ref fwd_lines);

			// Check the number of lines does not exceed the settings value
			Debug.Assert(bwd_lines + fwd_lines <= d.m_line_cache_count);

			// Build the collection of line byte ranges to add to the cache
			var line_index = ScanForLines(src, d, bwd_first, scan_range, bwd_lines, fwd_lines, buf, progress);
			return line_index;
		}

		/// <summary>Scan over the given range for lines</summary>
		private static List<RangeI> ScanForLines(Stream src, State d, bool bwd_first, RangeI scan_range, int bwd_lines, int fwd_lines, byte[] buf, ProgressFunc progress)
		{
			var line_index = new List<RangeI>();
			if (bwd_lines == 0 && fwd_lines == 0)
				return line_index;

			// Line index buffers for collecting the results
			var fwd_line_buf = new List<RangeI>(fwd_lines);
			var bwd_line_buf = new List<RangeI>(bwd_lines);

			// Data used in the 'add_line' callback. Updated for forward and backward passes
			var lbd = new LineBufferData
			{
				line_buf = null!, // pointer to either 'fwd_line_buf' or 'bwd_line_buf'
				line_limit = 0,  // Caps the number of lines read for each of the forward and backward searches
			};

			// Callback for adding line byte ranges to a line buffer
			bool AddLine(RangeI line, long baddr, long fend, byte[] bf, Encoding enc)
			{
				if (line.Empty && d.m_ignore_blanks)
					return true;

				// Test 'text' against each filter to see if it's included
				// Note: not caching this string because we want to read immediate data
				// from the file to pick up file changes.
				string text = d.m_encoding.GetString(buf, (int)line.Beg, (int)line.Size);
				if (!PassesFilters(text, d.m_filters))
					return true;

				// Convert the byte range to a file range
				line = line.Shift(baddr);
				Debug.Assert(new RangeI(0, d.m_fileend).Contains(line));
				lbd.line_buf.Add(line);
				Debug.Assert(lbd.line_buf.Count <= lbd.line_limit);
				return (fwd_line_buf.Count + bwd_line_buf.Count) < lbd.line_limit;
			}

			// Callback for updating progress
			bool Progress(long _, long __)
			{
				var scanned = fwd_line_buf.Count + bwd_line_buf.Count;
				return progress(scanned, lbd.line_limit);
			}

			// Scan twice, starting in the direction of the smallest range so that any
			// unused cache space is used by the search in the other direction.
			var scan_from = Math_.Clamp(d.m_filepos, scan_range.Beg, scan_range.End);
			for (int a = 0; a != 2; ++a, bwd_first = !bwd_first)
			{
				lbd.line_buf = bwd_first ? bwd_line_buf : fwd_line_buf;
				lbd.line_limit += bwd_first ? bwd_lines : fwd_lines;
				if ((bwd_line_buf.Count + fwd_line_buf.Count) < lbd.line_limit)
				{
					var length = bwd_first ? scan_from - scan_range.Beg : scan_range.End - scan_from;
					if (!FindLines(src, scan_from, d.m_fileend, bwd_first, length, AddLine, d.m_encoding, d.m_line_end, buf, Progress))
						break;
				}
			}

			// Scanning backward adds lines to the line index in reverse order.
			bwd_line_buf.Reverse();

			// 'line_index' should be a contiguous block of byte offset ranges for
			// the lines around 'd.m_filepos'. If 'reload' is false, then the line
			// index will only contain byte offset ranges that are not currently cached.
			line_index.Capacity = bwd_line_buf.Count + fwd_line_buf.Count;
			line_index.AddRange(bwd_line_buf);
			line_index.AddRange(fwd_line_buf);

			// Return the found line ranges
			return line_index;
		}

		/// <summary>Adjust the scan parameters when doing an incremental update based on the currently loaded data</summary>
		private static void AdjustForIncrementalBuild(State d, ref bool scan_backward, ref RangeI scan_range, ref int bwd_lines, ref int fwd_lines)
		{
			// Determine the direction the cached range is moving based on where 'filepos' is relative
			// to the current cache centre and which range contains an valid area to be scanned.
			// With incremental scans we can only update one side of the cache because the returned line index has to
			// be a contiguous block of lines. This means one of 'bwd_lines' or 'fwd_lines' must be zero.
			var Lrange = new RangeI(scan_range.Beg, d.m_line_start_range.Beg);
			var Rrange = new RangeI(d.m_line_start_range.End, scan_range.End);
			var dir =
				(!Lrange.Empty && !Rrange.Empty) ? Math.Sign(2 * d.m_index_centre - d.m_line_cache_count) :
				(!Lrange.Empty) ? -1 :
				(!Rrange.Empty) ? +1 :
				0;

			// Determine the number of lines to scan, based on direction
			if (dir < 0)
			{
				scan_backward = true;
				scan_range = Lrange;
				bwd_lines -= Math_.Clamp(d.m_index_centre - 0, 0, bwd_lines);
				fwd_lines = 0;
			}
			else if (dir > 0)
			{
				scan_backward = false;
				scan_range = Rrange;
				bwd_lines = 0;
				fwd_lines -= Math_.Clamp(d.m_index_count - d.m_index_centre - 1, 0, fwd_lines);
			}
			else if (dir == 0)
			{
				bwd_lines = 0;
				fwd_lines = 0;
				scan_range = RangeI.Zero;
			}
		}

		/// <summary>Scan the file from 'filepos' adding whole lines to 'line_index' until 'length' bytes have been read or 'add_line' returns false. Returns true if not interrupted</summary>
		private static bool FindLines(Stream src, long filepos, long fileend, bool backward, long length, AddLineFunc add_line, Encoding encoding, byte[] line_end, byte[] buf, ProgressFunc progress)
		{
			long scanned = 0, read_addr = filepos;
			for (; ; )
			{
				// Progress update
				if (!progress(scanned, length))
					return false;

				// Seek to the start position
				src.Seek(read_addr, SeekOrigin.Begin);

				// Buffer the contents of the file in 'buf'.
				var remaining = length - scanned; 
				var read = Buffer(src, remaining, fileend, encoding, backward, buf, out var eof);
				if (read == 0) break;

				// Set iterator limits.
				// 'i' is where to start scanning from
				// 'iend' is the end of the range to scan
				// 'ilast' is the start of the last line found
				// 'base_addr' is the file offset from which buf was read
				var i = backward ? read - 1 : 0;
				var iend = backward ? -1 : read;
				var lasti = backward ? read : 0;
				var base_addr = backward ? src.Position : src.Position - read;

				// If we're searching backwards and 'i' is at the end of a line,
				// we don't want to count that as the first found line so adjust 'i'.
				// If not however, then 'i' is partway through a line or at the end
				// of a file without a 'line_end' at the end and we want to include
				// this (possibly partial) line.
				if (backward && IsLineEnd(buf, read - line_end.Length, line_end))
					i -= line_end.Length;

				// Scan the buffer for lines
				for (i = Tools.FindNextDelim(buf, i, read, line_end, backward); i != iend; i = Tools.FindNextDelim(buf, i, read, line_end, backward))
				{
					// 'i' points to the start of a line,
					// 'lasti' points to the start of the last line we found
					// Get the range in buf containing the line
					var line = backward
						? new RangeI(i, lasti - line_end.Length)
						: new RangeI(lasti, i - line_end.Length);

					// Pass the detected line to the callback
					if (!add_line(line, base_addr, fileend, buf, encoding))
						return false;

					lasti = i;
					if (backward) i -= line_end.Length + 1;
				}

				// From 'lasti' to the end (or start in the backwards case) of the buffer represents
				// a (possibly partial) line. If we read a full buffer load last time, then we'll go
				// round again trying to read another buffer load, starting from 'lasti'.
				if (read == buf.Length)
				{
					// Make sure we're always making progress
					var scan_increment = backward ? (read - lasti) : lasti;
					if (scan_increment == 0) // No lines detected in this block
						throw new NoLinesException(read);

					scanned += scan_increment;
					read_addr = filepos + (backward ? -scanned : +scanned);
				}
				// Otherwise, we've read to the end (or start) of the file, or to the limit 'length'.
				// What's left in the buffer may be a partial line.
				else
				{
					// 'i' points to 'iend',
					// 'lasti' points to the start of the last line we found
					// Get the range in buf containing the line
					var line = backward
						? new RangeI(i + 1, lasti - line_end.Length)
						: new RangeI(lasti, i - (IsLineEnd(buf, i - line_end.Length, line_end) ? line_end.Length : 0));

					// Pass the detected line to the callback
					if (!add_line(line, base_addr, fileend, buf, encoding))
						return false;

					break;
				}
			}
			return true;
		}

		/// <summary>Seek to the first line that starts immediately before 'filepos'</summary>
		private static long FindLineStart(Stream src, long filepos, long fileend, byte[] line_end, Encoding encoding, byte[] buf)
		{
			// Set the file position
			src.Seek(filepos, SeekOrigin.Begin);

			// Read a block into 'buf'
			var read = Buffer(src, buf.Length, fileend, encoding, true, buf, out var eof);
			if (read == 0) return 0; // assume the first character in the file is the start of a line

			// Scan for a line start
			var idx = Tools.FindNextDelim(buf, read - 1, read, line_end, true);
			if (idx != -1) return src.Position + idx; // found
			if (filepos == read) return 0; // assume the first character in the file is the start of a line
			throw new NoLinesException(read);
		}

		/// <summary>
		/// Returns the index in 'line_index' for the line that contains 'filepos'.
		/// Returns '-1' if 'filepos' is before the first range and 'line_index.Count'
		/// if 'filepos' is after the last range</summary>
		private static int FindIndex(List<RangeI> line_index, long filepos)
		{
			// Careful, comparing 'filepos' to line starts, if idx == line_index.Count
			// it could be in the last line, or after the last line.
			if (line_index.Count == 0) return 0;
			if (filepos >= line_index.Back().End) return line_index.Count;
			int idx = line_index.BinarySearch(line => line.Beg.CompareTo(filepos));
			return idx >= 0 ? idx : ~idx - 1;
		}

		/// <summary>
		/// Buffer a maximum of 'count' bytes from 'src' into 'buf' (Note: automatically capped
		/// at buf.Length). If 'backward' is true the stream is 'seek'ed backward from the current
		/// position before reading and then 'seek'ed backward again after reading so that conceptually
		/// the file position moves in the direction of the read.
		/// Returns the number of bytes buffered in 'buf'</summary>
		private static int Buffer(Stream src, long count, long fileend, Encoding encoding, bool backward, byte[] buf, out bool eof)
		{
			Debug.Assert(count >= 0);
			var pos = src.Position;
			eof = backward ? pos == 0 : pos == fileend;

			// The number of bytes to buffer
			count = Math.Min(count, buf.Length);
			count = Math.Min(count, backward ? src.Position : fileend - src.Position);
			if (count == 0) return 0;
			Debug.Assert(count > 0);

			// Set the file position to the location to read from
			if (backward) src.Seek(-count, SeekOrigin.Current);
			pos = src.Position;
			eof = backward ? pos == 0 : pos + count == fileend;

			// Move to the start of a character
			if (encoding.Equals(Encoding.ASCII))
			{}
			else if (encoding.Equals(Encoding.Unicode) || encoding.Equals(Encoding.BigEndianUnicode))
			{
				// Skip the byte order mark (BOM)
				if (pos == 0 && src.ReadByte() != -1 && src.ReadByte() != -1) { }

				// Ensure a 16-bit word boundary
				if ((src.Position % 2) == 1)
					src.ReadByte();
			}
			else if (encoding.Equals(Encoding.UTF8))
			{
				// Some programs store a byte order mark (BOM) at the start of UTF-8 text files.
				// These bytes are 0xEF, 0xBB, 0xBF, so ignore them if present
				if (pos == 0 && src.ReadByte() == 0xEF && src.ReadByte() == 0xBB && src.ReadByte() == 0xBF) { }
				else src.Seek(pos, SeekOrigin.Begin);

				// UTF-8 encoding:
				// Bits Last code point  Byte 1    Byte 2    Byte 3    Byte 4    Byte 5   Byte 6
				//   7     U+007F        0xxxxxxx
				//  11     U+07FF        110xxxxx  10xxxxxx
				//  16     U+FFFF        1110xxxx  10xxxxxx  10xxxxxx
				//  21     U+1FFFFF      11110xxx  10xxxxxx  10xxxxxx  10xxxxxx
				//  26     U+3FFFFFF     111110xx  10xxxxxx  10xxxxxx  10xxxxxx  10xxxxxx
				//  31     U+7FFFFFFF    1111110x  10xxxxxx  10xxxxxx  10xxxxxx  10xxxxxx  10xxxxxx
				const int encode_mask = 0xC0; // 11000000
				const int encode_char = 0x80; // 10000000
				for (; ; )
				{
					var b = src.ReadByte();
					if (b == -1) return 0; // End of file
					if ((b & encode_mask) == encode_char) continue; // If 'b' is a byte halfway through an encoded char, keep reading

					// Otherwise it's an ASCII char or the start of a multi-byte char
					// save the byte we read, and read the remainder of 'count'
					src.Seek(-1, SeekOrigin.Current);
					break;
				}
			}

			// Buffer file data
			// We don't add the BOM to the buffer because the purpose of this function is to return lines of text from the file source.
			// If we've been asked to read from 'filepos' == 0 and  'count' is less than the BOM.Length then return 0 bytes read. This
			// means that if a UTF-8 BOM is present, reading 1 byte => read=0, reading 2 bytes => read=0, reading 3 bytes =>read=0,
			// reading 4 bytes => read = 4, reading 5 bytes => read=5, etc
			if (count < src.Position - pos)
				return 0;

			count -= src.Position - pos;
			var read = src.Read(buf, 0, (int)Math.Min(count, buf.Length));
			if (read != count) throw new IOException($"Failed to read file over range [{pos},{pos + count}) ({count} bytes). Read {read}/{count} bytes.");
			if (backward) src.Seek(-read, SeekOrigin.Current);
			return read;
		}

		/// <summary>Returns the byte range that would be buffered given a file position, size, and buffer size</summary>
		private static RangeI CalcBufferRange(long filepos, long fileend, long buf_size)
		{
			var rng = new RangeI();
			long ovr, hbuf = buf_size / 2;

			// Start with the range that has 'filepos' in the middle
			rng.Beg = filepos - hbuf;
			rng.End = filepos + hbuf;

			// Any overflow, add to the other range
			if ((ovr = 0 - rng.Beg) > 0) { rng.Beg += ovr; rng.End += ovr; }
			if ((ovr = rng.End - fileend) > 0) { rng.End -= ovr; rng.Beg -= ovr; }
			if ((ovr = 0 - rng.Beg) > 0) { rng.Beg += ovr; }

			Debug.Assert(rng.Beg >= 0 && rng.End <= fileend && rng.Beg <= rng.End);
			return rng;
		}

		/// <summary>Returns the number of lines that would be each side of the cache centre for 'num_lines'</summary>
		private static RangeI CalcLineRange(long num_lines)
		{
			var bwd_lines = Math.Max(1, num_lines / 2);
			var fwd_lines = Math.Max(2, num_lines - bwd_lines);
			return new RangeI(bwd_lines, fwd_lines);
		}

		/// <summary>Tests 'text' against each of the filters in 'filters'</summary>
		private static bool PassesFilters(string text, IEnumerable<IFilter> filters)
		{
			foreach (var ft in filters)
			{
				if (!ft.IsMatch(text)) continue;
				return ft.IfMatch == EIfMatch.Keep;
			}
			return true;
		}

		/// <summary>Returns true if buf[index] matches 'line_end'. Handles out of bounds</summary>
		private static bool IsLineEnd(byte[] buf, int index, byte[] line_end)
		{
			if (index < 0 || index + line_end.Length > buf.Length)
				return false;

			for (int i = 0; i != line_end.Length; ++i)
				if (buf[index + i] != line_end[i])
					return false;

			return true;
		}

		/// <summary>Callback function called by FindLines that is called with each line found</summary>
		/// <param name="rng">The byte range of the line within 'buf' not including the line ending</param>
		/// <param name="base_addr">The base address in the file that buf is relative to</param>
		/// <param name="fileend">The current known end position of the file</param>
		/// <param name="buf">Buffer containing the buffered file data</param>
		/// <param name="encoding">The text encoding used</param>
		/// <returns>Return true to continue adding lines, false to stop</returns>
		private delegate bool AddLineFunc(RangeI rng, long base_addr, long fileend, byte[] buf, Encoding encoding);

		/// <summary>Callback function called periodically while finding lines</summary>
		/// <returns>Return true to continue finding, false to abort</returns>
		private delegate bool ProgressFunc(long scanned, long length);

		/// <summary>Helper tuple</summary>
		private struct LineBufferData
		{
			public List<RangeI> line_buf;
			public int line_limit;
		}

		/// <summary>State variables for the line index. In a struct so they are copy-able</summary>
		private struct State
		{
			public Encoding m_encoding;      // Text encoding format of the source data
			public byte[] m_line_end;        // The line ending characters
			public long m_fileend;           // The last known length of the log file
			public long m_filepos;           // The last load position for the line index, nominally the centre of the data 
			public int m_line_cache_count;   // The number of lines to cache
			public long m_max_line_length;   // The maximum length of any one line (in bytes)
			public long m_file_buffer_size;  // The maximum amount of file data to load
			public bool m_ignore_blanks;     // True to ignore blank lines;
			public RangeI m_line_start_range; // The byte range of known complete lines
			public int m_index_centre;       // The index in the cache (pre-build) of 'm_filepos' 
			public int m_index_count;        // The number of entries in the line index (pre-build)

			public List<IFilter> m_filters;         // Line filters
		}
	}

	#region EventArgs
	public class BuildProgressEventArgs : EventArgs
	{
		public BuildProgressEventArgs(long done, long total)
		{
			Done = done;
			Total = total;
		}

		/// <summary>The amount done so far</summary>
		public long Done { get; set; }

		/// <summary>The total amount to do</summary>
		public long Total { get; set; }
	}
	public class BuildCompleteEventArgs : EventArgs
	{
		public BuildCompleteEventArgs(RangeI new_range, int row_delta)
		{
			NewRange = new_range;
			RowDelta = row_delta;
		}

		/// <summary>The range of new data added to the cache.</summary>
		public RangeI NewRange { get; private set; }

		/// <summary>The number of rows that the line index has shifted by</summary>
		public int RowDelta { get; private set; }
	}
	public class BuildErrorEventArgs : EventArgs
	{
		public BuildErrorEventArgs(Exception error)
		{
			Error = error;
		}
		public Exception Error { get; private set; }
	}
	#endregion
}
