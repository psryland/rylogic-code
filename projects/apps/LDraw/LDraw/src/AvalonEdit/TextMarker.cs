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
using System.Windows;
using System.Windows.Media;
using ICSharpCode.AvalonEdit.Document;

namespace ICSharpCode.AvalonEdit.AddIn
{
	public sealed class TextMarker :TextSegment, ITextMarker
	{
		private readonly TextMarkerService m_service;
		public TextMarker(TextMarkerService service, int startOffset, int length)
		{
			m_service = service ?? throw new ArgumentNullException("service");
			m_marker_types = ETextMarkerTypes.None;
			StartOffset = startOffset;
			Length = length;
		}

		/// <summary></summary>
		public event EventHandler? Deleted;

		/// <summary></summary>
		public bool IsDeleted => !IsConnectedToCollection;

		/// <summary></summary>
		public void Delete()
		{
			m_service.Remove(this);
		}

		/// <summary></summary>
		internal void OnDeleted()
		{
			Deleted?.Invoke(this, EventArgs.Empty);
		}

		/// <summary></summary>
		void Redraw()
		{
			m_service.Redraw(this);
		}

		/// <summary></summary>
		public Color? BackgroundColor
		{
			get;
			set
			{
				if (field == value) return;
				field = value;
				Redraw();
			}
		}

		/// <summary></summary>
		public Color? ForegroundColor
		{
			get;
			set
			{
				if (field == value) return;
				field = value;
				Redraw();
			}
		}

		/// <summary></summary>
		public FontWeight? FontWeight
		{
			get;
			set
			{
				if (field == value) return;
				field = value;
				Redraw();
			}
		}

		/// <summary></summary>
		public FontStyle? FontStyle
		{
			get;
			set
			{
				if (field == value) return;
				field = value;
				Redraw();
			}
		}

		/// <summary></summary>
		public ETextMarkerTypes MarkerTypes
		{
			get => m_marker_types;
			set
			{
				if (m_marker_types == value) return;
				m_marker_types = value;
				Redraw();
			}
		}
		private ETextMarkerTypes m_marker_types;

		/// <summary></summary>
		public Color MarkerColor
		{
			get;
			set
			{
				if (field == value) return;
				field = value;
				Redraw();
			}
		}

		/// <summary></summary>
		public object? ToolTip { get; set; }

		/// <summary></summary>
		public object? Tag { get; set; }
	}
}