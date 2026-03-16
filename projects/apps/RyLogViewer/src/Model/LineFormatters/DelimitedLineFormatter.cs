using System.Text;
using Rylogic.Common;

namespace RyLogViewer
{
	public class DelimitedLineFormatter : ILineFormatter
	{
		private readonly Encoding m_encoding;
		private readonly string m_delimiter;

		public DelimitedLineFormatter(Encoding encoding, string delimiter)
		{
			m_encoding = encoding;
			m_delimiter = delimiter;
		}

		/// <summary>A name for this formatter, for displaying in the UI</summary>
		public const string Name = "Delimited Text";
		string ILineFormatter.Name => Name;

		/// <summary>Return a line instance from a buffer of log data</summary>
		public ILine CreateLine(byte[] line_buf, int start, int count, RangeI file_byte_range)
		{
			var text = m_encoding.GetString(line_buf, start, count);
			return new DelimitedLine(text, file_byte_range, m_delimiter);
		}
	}
}
