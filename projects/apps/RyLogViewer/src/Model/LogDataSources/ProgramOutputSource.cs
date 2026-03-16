using System;
using System.Diagnostics;
using System.IO;
using System.Text;
using System.Threading;
using Rylogic.Utility;

namespace RyLogViewer
{
	/// <summary>Configuration for launching a program and capturing its output</summary>
	public class LaunchApp : ICloneable
	{
		/// <summary>Path to the executable</summary>
		public string Executable { get; set; } = string.Empty;

		/// <summary>Command line arguments</summary>
		public string Arguments { get; set; } = string.Empty;

		/// <summary>Working directory for the process</summary>
		public string WorkingDirectory { get; set; } = string.Empty;

		/// <summary>File to write captured output to. If empty, a temp file is used.</summary>
		public string OutputFilepath { get; set; } = string.Empty;

		/// <summary>Whether to show the process window</summary>
		public bool ShowWindow { get; set; }

		/// <summary>Whether to append to the output file (vs overwrite)</summary>
		public bool AppendOutputFile { get; set; } = true;

		/// <summary>Whether to capture stdout</summary>
		public bool CaptureStdout { get; set; } = true;

		/// <summary>Whether to capture stderr</summary>
		public bool CaptureStderr { get; set; } = true;

		public LaunchApp()
		{ }
		public LaunchApp(LaunchApp rhs)
		{
			Executable = rhs.Executable;
			Arguments = rhs.Arguments;
			WorkingDirectory = rhs.WorkingDirectory;
			OutputFilepath = rhs.OutputFilepath;
			ShowWindow = rhs.ShowWindow;
			AppendOutputFile = rhs.AppendOutputFile;
			CaptureStdout = rhs.CaptureStdout;
			CaptureStderr = rhs.CaptureStderr;
		}
		public object Clone()
		{
			return new LaunchApp(this);
		}
		public override string ToString()
		{
			return Executable;
		}
	}

	/// <summary>Data source that launches a process and captures stdout/stderr to a file</summary>
	public class ProgramOutputSource : ILogDataSource
	{
		private readonly string m_filepath;
		private readonly bool m_is_temp_file;
		private readonly Process m_process;
		private readonly FileStream m_output_stream;
		private readonly object m_lock = new();

		public ProgramOutputSource(LaunchApp launch)
		{
			// Determine output file
			m_filepath = launch.OutputFilepath;
			m_is_temp_file = string.IsNullOrEmpty(m_filepath);
			if (m_is_temp_file)
				m_filepath = System.IO.Path.Combine(System.IO.Path.GetTempPath(), $"rylogviewer_{Guid.NewGuid():N}.tmp");

			// Open the output file
			var mode = launch.AppendOutputFile && !m_is_temp_file ? FileMode.Append : FileMode.Create;
			m_output_stream = new FileStream(m_filepath, mode, FileAccess.Write, FileShare.Read, 4096, FileOptions.Asynchronous);

			// Configure the process
			var info = new ProcessStartInfo
			{
				FileName = launch.Executable,
				Arguments = launch.Arguments,
				WorkingDirectory = launch.WorkingDirectory,
				UseShellExecute = false,
				CreateNoWindow = !launch.ShowWindow,
				RedirectStandardOutput = launch.CaptureStdout,
				RedirectStandardError = launch.CaptureStderr,
			};
			m_process = new Process { StartInfo = info, EnableRaisingEvents = true };
			m_process.Exited += (s, e) => ProcessExited?.Invoke(this, EventArgs.Empty);

			// Start the process and begin async reads
			m_process.Start();
			if (launch.CaptureStdout)
				BeginReadStream(m_process.StandardOutput.BaseStream);
			if (launch.CaptureStderr)
				BeginReadStream(m_process.StandardError.BaseStream);
		}

		/// <summary>Raised when the child process exits</summary>
		public event EventHandler? ProcessExited;

		/// <summary>A filepath or description of the data source</summary>
		public string Path => m_filepath;

		/// <summary>Create a new stream with access to the log data</summary>
		public Stream OpenStream()
		{
			return new FileStream(m_filepath, FileMode.Open, FileAccess.Read, FileShare.ReadWrite);
		}

		/// <summary>Clean up the process and output file</summary>
		public void Dispose()
		{
			// Kill the process if still running
			try
			{
				if (!m_process.HasExited)
					m_process.Kill();
			}
			catch { }
			m_process.Dispose();

			lock (m_lock)
			{
				m_output_stream.Dispose();
			}

			// Delete temp file
			if (m_is_temp_file)
			{
				try { File.Delete(m_filepath); }
				catch { }
			}
		}

		/// <summary>Begin async reading from a stream, forwarding data to the output file</summary>
		private void BeginReadStream(Stream stream)
		{
			var buf = new byte[4096];
			ReadLoop(stream, buf);
		}
		private async void ReadLoop(Stream stream, byte[] buf)
		{
			try
			{
				for (;;)
				{
					var read = await stream.ReadAsync(buf, 0, buf.Length);
					if (read == 0) break;
					lock (m_lock)
					{
						m_output_stream.Write(buf, 0, read);
						m_output_stream.Flush();
					}
				}
			}
			catch
			{
				// Stream closed or process exited
			}
		}
	}
}
