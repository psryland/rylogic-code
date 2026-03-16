using System.IO;
using Rylogic.Common;
using RyLogViewer.Options;

namespace RyLogViewer
{
	public class SingleFileSource : ILogDataSource
	{
		private readonly string m_filepath;
		private readonly Settings m_settings;
		public SingleFileSource(string filepath, Settings settings)
		{
			m_filepath = filepath;
			m_settings = settings;
		}
		public void Dispose()
		{}

		/// <summary>The name of this file source</summary>
		public string Path
		{
			get { return m_settings.General.FullPathInTitle ? m_filepath : Path_.FileName(m_filepath); }
		}

		/// <summary>Create a new stream with access to the log data</summary>
		public Stream OpenStream()
		{
			return new FileStream(m_filepath, FileMode.Open, FileAccess.Read, FileShare.ReadWrite);
		}
	}
}
