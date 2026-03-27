using System;
using System.IO;
using System.Net.Sockets;
using System.Text;

namespace Rylogic.LDrawVisualiser.Core
{
	/// <summary>Connection state for the LDraw streamer</summary>
	public enum EConnectionState
	{
		Disconnected,
		Connected,
		Error,
	}

	/// <summary>
	/// TCP client that streams LDraw script strings to the LDraw application.
	/// LDraw listens on a configurable port (default 1976) when streaming is enabled.
	/// </summary>
	public class LDrawStreamer : IDisposable
	{
		private TcpClient? m_client;
		private NetworkStream? m_stream;

		/// <summary>The host to connect to</summary>
		public string Host { get; set; } = "localhost";

		/// <summary>The port to connect to</summary>
		public int Port { get; set; } = 1976;

		/// <summary>Current connection state</summary>
		public EConnectionState State { get; private set; } = EConnectionState.Disconnected;

		/// <summary>Last error message, if any</summary>
		public string? LastError { get; private set; }

		/// <summary>Connect to the LDraw streaming port</summary>
		public bool Connect()
		{
			Disconnect();

			try
			{
				m_client = new TcpClient();
				m_client.Connect(Host, Port);
				m_stream = m_client.GetStream();
				State = EConnectionState.Connected;
				LastError = null;
				return true;
			}
			catch (Exception ex)
			{
				State = EConnectionState.Error;
				LastError = ex.Message;
				CleanupConnection();
				return false;
			}
		}

		/// <summary>Disconnect from LDraw</summary>
		public void Disconnect()
		{
			CleanupConnection();
			State = EConnectionState.Disconnected;
			LastError = null;
		}

		/// <summary>Send an LDraw script string to the connected LDraw instance</summary>
		public bool Send(string ldraw_script)
		{
			if (State != EConnectionState.Connected || m_stream == null)
			{
				LastError = "Not connected";
				return false;
			}

			try
			{
				var bytes = Encoding.UTF8.GetBytes(ldraw_script);
				m_stream.Write(bytes, 0, bytes.Length);
				m_stream.Flush();
				LastError = null;
				return true;
			}
			catch (Exception ex)
			{
				State = EConnectionState.Error;
				LastError = ex.Message;
				CleanupConnection();
				return false;
			}
		}

		/// <summary>Parse an "host:port" address string</summary>
		public static (string host, int port) ParseAddress(string address)
		{
			var parts = address.Split(':');
			var host = parts.Length > 0 ? parts[0].Trim() : "localhost";
			var port = parts.Length > 1 && int.TryParse(parts[1].Trim(), out var p) ? p : 1976;
			if (string.IsNullOrEmpty(host)) host = "localhost";
			return (host, port);
		}

		private void CleanupConnection()
		{
			try { m_stream?.Close(); } catch { }
			try { m_client?.Close(); } catch { }
			m_stream = null;
			m_client = null;
		}

		public void Dispose()
		{
			Disconnect();
		}
	}
}
