using System;
using Rylogic.Common;

namespace RyLogViewer
{
	public class DelimitedLine : ILine
	{
		private readonly string[] m_columns;

		public DelimitedLine(string text, RangeI file_byte_range, string delimiter)
		{
			FileByteRange = file_byte_range;
			m_columns = text.Split(new[] { delimiter }, StringSplitOptions.None);
		}

		/// <summary>The log data byte range</summary>
		public RangeI FileByteRange { get; }

		/// <summary>Return the value for the requested column</summary>
		public string Value(int column)
		{
			return column >= 0 && column < m_columns.Length ? m_columns[column] : string.Empty;
		}
	}
}
