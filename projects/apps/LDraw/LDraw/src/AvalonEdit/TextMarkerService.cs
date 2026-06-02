// Copyright (c) 2014 AlphaSierraPapa for the SharpDevelop Team
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy of this
// software and associated documentation files (the "Software"), to deal in the Software
// without restriction, including without limitation the rights to use, copy, modify, merge,
// publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons
// to whom the Software is furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in all copies or
// substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
// INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
// PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
// FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
// OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
// DEALINGS IN THE SOFTWARE.

using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Windows;
using System.Windows.Media;
using System.Windows.Threading;
using ICSharpCode.AvalonEdit.Document;
using ICSharpCode.AvalonEdit.Rendering;

namespace ICSharpCode.AvalonEdit.AddIn
{
	/// <summary>Handles the text markers for a code editor.</summary>
	public sealed class TextMarkerService :DocumentColorizingTransformer, IBackgroundRenderer, ITextMarkerService, ITextViewConnect
	{
		private readonly TextDocument m_document;
		private readonly TextSegmentCollection<TextMarker> m_markers;
		private readonly List<TextView> m_text_views;

		public TextMarkerService(TextDocument document)
		{
			if (document == null)
				throw new ArgumentNullException("document");

			m_document = document;
			m_markers = new TextSegmentCollection<TextMarker>(document);
			m_text_views = new List<TextView>();
		}
		protected override void ColorizeLine(DocumentLine line)
		{
			if (m_markers == null)
				return;

			var lineStart = line.Offset;
			var lineEnd = lineStart + line.Length;
			foreach (var marker in m_markers.FindOverlappingSegments(lineStart, line.Length))
			{
				var foregroundBrush = (Brush?)null;
				if (marker.ForegroundColor != null)
				{
					foregroundBrush = new SolidColorBrush(marker.ForegroundColor.Value);
					foregroundBrush.Freeze();
				}

				ChangeLinePart(
					Math.Max(marker.StartOffset, lineStart),
					Math.Min(marker.EndOffset, lineEnd),
					element => {
						if (foregroundBrush != null)
						{
							element.TextRunProperties.SetForegroundBrush(foregroundBrush);
						}
						Typeface tf = element.TextRunProperties.Typeface;
						element.TextRunProperties.SetTypeface(new Typeface(
							tf.FontFamily,
							marker.FontStyle ?? tf.Style,
							marker.FontWeight ?? tf.Weight,
							tf.Stretch
						));
					}
				);
			}
		}

		/// <summary>Create a text marker for the given text range</summary>
		public ITextMarker Create(int start_offset, int length)
		{
			var text_lenth = m_document.TextLength;
			if (start_offset < 0 || start_offset > text_lenth)
				throw new ArgumentOutOfRangeException("startOffset", start_offset, "Value must be between 0 and " + text_lenth);
			if (length < 0 || start_offset + length > text_lenth)
				throw new ArgumentOutOfRangeException("length", length, "length must not be negative and startOffset+length must not be after the end of the document");

			// No need to mark segment for redraw: the text marker is invisible until a property is set
			var m = new TextMarker(this, start_offset, length);
			m_markers.Add(m);
			return m;
		}

		/// <summary>Enum all markers that span the position at 'offset'</summary>
		public IEnumerable<ITextMarker> GetMarkersAtOffset(int offset) => m_markers.FindSegmentsContaining(offset);

		/// <summary>Enum all contained markers</summary>
		public IEnumerable<ITextMarker> TextMarkers => m_markers ?? Enumerable.Empty<ITextMarker>();

		/// <summary>Remove all contained text markers that pass 'predicate'</summary>
		public void RemoveAll(Predicate<ITextMarker> predicate)
		{
			if (predicate == null)
				throw new ArgumentNullException("predicate");

			foreach (var m in m_markers.ToArray())
			{
				if (!predicate(m)) continue;
				Remove(m);
			}
		}

		/// <summary>Remove a specific marker</summary>
		public void Remove(ITextMarker marker)
		{
			if (marker == null)
				throw new ArgumentNullException("marker");

			if (marker is TextMarker m && m_markers.Remove(m))
			{
				Redraw(m);
				m.OnDeleted();
			}
		}

		/// <summary>Redraws the specified text segment.</summary>
		internal void Redraw(ISegment segment)
		{
			foreach (var view in m_text_views)
				view.Redraw(segment, DispatcherPriority.Normal);
		
			RedrawRequested?.Invoke(this, EventArgs.Empty);
		}

		/// <summary></summary>
		public event EventHandler? RedrawRequested;

		/// <summary>The render layer</summary>
		public KnownLayer Layer => KnownLayer.Selection; // draw behind selection

		/// <summary></summary>
		public void Draw(TextView text_view, DrawingContext drawing_context)
		{
			if (text_view == null)
				throw new ArgumentNullException("textView");
			if (drawing_context == null)
				throw new ArgumentNullException("drawingContext");
			if (!text_view.VisualLinesValid)
				return;

			var visual_lines = text_view.VisualLines;
			if (visual_lines.Count == 0)
				return;

			var view_beg = visual_lines.First().FirstDocumentLine.Offset;
			var view_end = visual_lines.Last().LastDocumentLine.EndOffset;
			foreach (var marker in m_markers.FindOverlappingSegments(view_beg, view_end - view_beg))
			{
				if (marker.BackgroundColor != null)
				{
					var geo_builder = new BackgroundGeometryBuilder();
					geo_builder.AlignToWholePixels = true;
					geo_builder.CornerRadius = 3;
					geo_builder.AddSegment(text_view, marker);
					var geometry = geo_builder.CreateGeometry();
					if (geometry != null)
					{
						var color = marker.BackgroundColor.Value;
						var brush = new SolidColorBrush(color);
						brush.Freeze();
						drawing_context.DrawGeometry(brush, null, geometry);
					}
				}
				var underlineMarkerTypes = ETextMarkerTypes.SquigglyUnderline | ETextMarkerTypes.NormalUnderline | ETextMarkerTypes.DottedUnderline;
				if ((marker.MarkerTypes & underlineMarkerTypes) != 0)
				{
					foreach (var r in BackgroundGeometryBuilder.GetRectsForSegment(text_view, marker))
					{
						var point_beg = r.BottomLeft;
						var point_end = r.BottomRight;

						var used_brush = new SolidColorBrush(marker.MarkerColor);
						used_brush.Freeze();
						if ((marker.MarkerTypes & ETextMarkerTypes.SquigglyUnderline) != 0)
						{
							double offset = 2.5;
							var count = Math.Max((int)((point_end.X - point_beg.X) / offset) + 1, 4);

							var geometry = new StreamGeometry();
							using (var ctx = geometry.Open())
							{
								ctx.BeginFigure(point_beg, false, false);
								ctx.PolyLineTo(CreatePoints(point_beg, point_end, offset, count).ToArray(), true, false);
							}
							geometry.Freeze();

							var used_pen = new Pen(used_brush, 1);
							used_pen.Freeze();
							drawing_context.DrawGeometry(Brushes.Transparent, used_pen, geometry);
						}
						if ((marker.MarkerTypes & ETextMarkerTypes.NormalUnderline) != 0)
						{
							var used_pen = new Pen(used_brush, 1);
							used_pen.Freeze();
							drawing_context.DrawLine(used_pen, point_beg, point_end);
						}
						if ((marker.MarkerTypes & ETextMarkerTypes.DottedUnderline) != 0)
						{
							var used_pen = new Pen(used_brush, 1);
							used_pen.DashStyle = DashStyles.Dot;
							used_pen.Freeze();
							drawing_context.DrawLine(used_pen, point_beg, point_end);
						}
					}
				}
			}
		}

		/// <summary></summary>
		IEnumerable<Point> CreatePoints(Point start, Point end, double offset, int count)
		{
			for (int i = 0; i < count; i++)
				yield return new Point(start.X + i * offset, start.Y - ((i + 1) % 2 == 0 ? offset : 0));
		}

		/// <summary></summary>
		void ITextViewConnect.AddToTextView(TextView textView)
		{
			if (textView != null && !m_text_views.Contains(textView))
			{
				Debug.Assert(textView.Document == m_document);
				m_text_views.Add(textView);
			}
		}

		/// <summary></summary>
		void ITextViewConnect.RemoveFromTextView(TextView textView)
		{
			if (textView != null)
			{
				Debug.Assert(textView.Document == m_document);
				m_text_views.Remove(textView);
			}
		}
	}
}