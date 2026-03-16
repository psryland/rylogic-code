using System;
using System.Collections.Generic;
using System.Text;
using System.Text.Json;
using Rylogic.Common;

namespace RyLogViewer
{
	public class JsonLineFormatter : ILineFormatter
	{
		private readonly Encoding m_encoding;

		public JsonLineFormatter()
			: this(Encoding.UTF8)
		{ }
		public JsonLineFormatter(Encoding encoding)
		{
			m_encoding = encoding;
		}

		/// <summary>A name for this formatter, for displaying in the UI</summary>
		public const string Name = "JSON Lines";
		string ILineFormatter.Name => Name;

		/// <summary>Return a line instance from a buffer of log data</summary>
		public ILine CreateLine(byte[] line_buf, int start, int count, RangeI file_byte_range)
		{
			var text = m_encoding.GetString(line_buf, start, count).Trim();
			try
			{
				// Try to parse as JSON and extract values as columns
				using var doc = JsonDocument.Parse(text);
				var values = new List<string>();
				foreach (var prop in doc.RootElement.EnumerateObject())
					values.Add(prop.Value.ToString());

				return new JsonLine(values.ToArray(), file_byte_range, text);
			}
			catch
			{
				// If JSON parsing fails, treat as a single-column line
				return new Line(text, file_byte_range);
			}
		}
	}

	/// <summary>A line whose columns are JSON property values</summary>
	public class JsonLine : ILine
	{
		private readonly string[] m_values;
		private readonly string m_raw;

		public JsonLine(string[] values, RangeI file_byte_range, string raw_text)
		{
			m_values = values;
			m_raw = raw_text;
			FileByteRange = file_byte_range;
		}

		/// <summary>The log data byte range</summary>
		public RangeI FileByteRange { get; }

		/// <summary>Return the value for the requested column</summary>
		public string Value(int column)
		{
			if (column == 0 && m_values.Length == 0) return m_raw;
			return column >= 0 && column < m_values.Length ? m_values[column] : string.Empty;
		}
	}
}
