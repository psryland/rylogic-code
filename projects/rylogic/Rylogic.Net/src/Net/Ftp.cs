using System;
using System.IO;
using System.Net;
using System.Net.Security;
using System.Net.Sockets;
using System.Security.Authentication;
using System.Security.Cryptography.X509Certificates;
using System.Text;
using Rylogic.Common;
using Rylogic.Streams;
using Rylogic.Utility;

namespace Rylogic.Net
{
	/// <summary>
	/// A class for creating FTP/FTPS connections.
	/// FTPS = ftp explicit SSL. Note FTPS != SFTP (ssh ftp) </summary>
	public sealed class FTPConnection :IDisposable
	{
		// Example Sessions:
		// Active Data Transfer Example:
		//    Client: USER anonymous
		//    Server: 331 Guest login ok, send your e-mail address as password.
		//    Client: PASS NcFTP@
		//    Server: 230 Logged in anonymously.
		//    Client: PORT 192,168,1,2,7,138                           (The client wants the server to send to port number 1930 on IP address 192.168.1.2.)
		//    Server: 200 PORT command successful.
		//    Client: LIST
		//    Server: 150 Opening ASCII mode data connection for /bin/ls.    (The server now connects out from port 21 to port 1930 on 192.168.1.2.)
		//    Server: 226 Listing completed.                           (That succeeded, so the data is now sent over the established data connection.)
		//    Client: QUIT
		//    Server: 221 Goodbye.
		//
		// Passive Data Transfer Example:
		//    Client: USER anonymous
		//    Server: 331 Guest login ok, send your e-mail address as password.
		//    Client: PASS NcFTP@
		//    Server: 230 Logged in anonymously.
		//    Client: PASV                                             (The client is asking where he should connect.)
		//    Server: 227 Entering Passive Mode (172,16,3,4,204,173)   (The server replies with port 52397 on IP address 172.16.3.4.)
		//    Client: LIST
		//    Server: 150 Data connection accepted from 172.16.3.4:52397; transfer starting.  (The client has now connected to the server at port 52397 on IP address 172.16.3.4.)
		//    Server: 226 Listing completed.                           (That succeeded, so the data is now sent over the established data connection.)
		//    Client: QUIT
		//    Server: 221 Goodbye.

		/// <summary>An event that is called to trace the behaviour of the ftp connection. Mainly for debugging</summary>
		public event TraceEvent? TraceMessage;
		public delegate void TraceEvent(string message, params object[] args);
		private void Trace(Utility.Lazy<string> message, params object[] args)
		{
			TraceMessage?.Invoke("[FTPConnection] " + message, args);
		}

		/// <summary>The stream over which ftp commands are sent</summary>
		private Stream? m_cmd;

		/// <summary>Creates an FTP connection</summary>
		public FTPConnection()
		{
			Settings = new SettingsData();
		}
		public void Dispose()
		{
			Close();
		}

		/// <summary></summary>
		private SettingsData Settings { get; set; }

		/// <summary>Returns true if this object is connected to an ftp host</summary>
		public bool IsConnected
		{
			get; private set;
		}

		/// <summary>Set Ascii or Binary mode for downloads</summary>
		public bool AsciiMode
		{
			set
			{
				if (!IsConnected) throw new InvalidOperationException("AsciiMode can only be set once connected");
				Reply reply = SendCommand(value ? "TYPE A" : "TYPE I");
				if (reply.Code != Status.CommandOkay) throw new IOException(reply.Message);
			}
		}

		/// <summary>Close the FTP connection.</summary>
		public void Close(bool send_quit = true)
		{
			Trace("Closing...");
			if (m_cmd != null)
			{
				if (send_quit) SendCommand("QUIT");
				m_cmd.Dispose();
				m_cmd = null;
			}
			IsConnected = false;
		}

		/// <summary>Connect to the remote host</summary>
		public void Login(SettingsData settings)
		{
			// Disconnect first
			Close();

			Settings = settings;

			// If 'UseSSL' is true, we connect to the socket, ask for SSL by sending
			// the 'AUTH SSL' command, create the SSL stream, then log on with the username/pw

			// Attempt to connect. (Throws on failure)
			Socket socket = new Socket(AddressFamily.InterNetwork, SocketType.Stream, ProtocolType.Tcp);
			socket.Connect(new IPEndPoint(Dns.GetHostEntry(Settings.RemoteHost).AddressList[0], Settings.RemotePort));
			m_cmd = new NetworkStream(socket, true);

			// Check the socket is connected
			Reply reply = ReadReply();
			if (reply.Code != Status.ServiceReady)
				throw new IOException(reply.Message);

			// If we're using SSL wrap the command stream in an SSLStream
			if (Settings.UseSSL)
			{
				// Ask the socket to use SSL
				SendCommand("AUTH SSL");

				// Create SSL Stream
				m_cmd = WrapInSSLStream(m_cmd);
			}

			// Login with username/password
			reply = SendCommand("USER " + Settings.UserName);
			switch (reply.Code)
			{
			default: Close(false); throw new IOException(reply.Message);
			case Status.UserLoggedIn: break;
			case Status.UserNameOkay:
				reply = SendCommand("PASS " + Settings.Password);
				if (reply.Code != Status.UserLoggedIn && reply.Code != Status.CommandSuperfluous)
				{
					Close(false);
					throw new IOException(reply.Message);
				}
				break;
			}

			IsConnected = true;
			Trace("Connected to " + Settings.RemoteHost);

			// Navigate to the remote path
			ChangeRemoteDirectory(Settings.RemotePath);
		}

		/// <summary>Send a command string over the ftp connection</summary>
		public Reply SendCommand(string command)
		{
			if (m_cmd == null)
				throw new Exception("No connection stream");

			// Ensure the command line is complete
			if (!command.EndsWith("\r\n")) command += "\r\n";
			Trace(Lazy.New(()=>command.TrimEnd('\r','\n')));

			// Send the command over the command stream
			byte[] cmd = Encoding.ASCII.GetBytes(command);
			m_cmd.Write(cmd, 0, cmd.Length);
			return ReadReply();
		}

		/// <summary>Return the response message from the connection</summary>
		private Reply ReadReply()
		{
			if (m_cmd == null)
				throw new Exception("No connection stream");

			using var sr = new StreamReader(new UncloseableStream(m_cmd));
			var reply = sr.ReadLine() ?? string.Empty;
			Trace("Reply: " + reply);
			return new Reply(reply);
		}

		/// <summary>FTP uses two ports: 21 for commands, and 20 for sending data. This method returns a stream over which data can be sent.</summary>
		private Stream ConnectDataStream()
		{
			// Putting the command connection in passive mode causes a respose by the server
			// indicating where the data connection should be made. It has the form:
			// 227 Entering Passive Mode (172,16,3,4,204,173)
			Reply reply = SendCommand("PASV");
			if (reply.Code != Status.EnteringPassiveMode)
				throw new IOException(reply.Message);
			
			try
			{
				// Extract the ip data from the reply
				int s = reply.Message.IndexOf('(') + 1;
				int e = reply.Message.IndexOf(')');
				string[] addr = reply.Message.Substring(s, e - s).Split(',');
				
				// Create the IP and port values
				string address = string.Format("{0}.{1}.{2}.{3}",addr[0],addr[1],addr[2],addr[3]);
				int port       = (int.Parse(addr[4]) << 8) | (int.Parse(addr[5]));
				
				// Create the data socket
				Socket socket = new Socket(AddressFamily.InterNetwork, SocketType.Stream, ProtocolType.Tcp);
				socket.Connect(new IPEndPoint(Dns.GetHostEntry(address).AddressList[0], port));
				Stream data = new NetworkStream(socket, true);
				
				if (Settings.UseSSL)
				{
					// Ask the socket to use SSL
					SendCommand("AUTH SSL");

					// Create SSL Stream
					data = WrapInSSLStream(data);
				}
				
				return data;
			}
			catch (Exception ex)
			{
				throw new IOException("Malformed PASV reply: " + reply.Message, ex);
			}
		}
		
		/// <summary>Wraps a stream in an SSL stream and check authentication</summary>
		private Stream WrapInSSLStream(Stream src)
		{
			var stream = new SslStream(src);
			stream.AuthenticateAsClient(Settings.RemoteHost, null, Settings.SSLProtocols, true);
			if (!stream.IsAuthenticated) throw new AuthenticationException("Failed to create a secure data connection");
			return stream;
		}

		/// <summary>Return a string array containing the remote directory's file list.</summary>
		public string[] RemoteFileList(string mask)
		{
			// Ask for a directory listing
			Reply reply = SendCommand("NLST " + mask);
			if (reply.Code != Status.DataConnectionOpenning && reply.Code != Status.DataConnectionAlreadyOpen)
				throw new IOException(reply.Message);
			
			// Read the response on a data channel
			string response;
			using(var sr = new StreamReader(ConnectDataStream(), Encoding.ASCII))
				response = sr.ReadToEnd();
			
			// Read the connection closing message on the command channel
			reply = ReadReply();
			if (reply.Code != Status.DataConnectionClosing)
				throw new IOException(reply.Message);
			
			// Return the listing
			string[] lines = response.Split(new[]{"\r\n"}, StringSplitOptions.RemoveEmptyEntries);
			return lines;
		}

		/// <summary>Return the size of a remote file.</summary>
		public long RemoteFileSize(string filename)
		{
			Reply reply = SendCommand("SIZE " + filename);
			if (reply.Code != Status.FileStatus)
				throw new IOException(reply.Message);
			
			return long.Parse(reply.Message);
		}

		/// <summary>Download a remote file to a local filepath. The local file will be created, appended, or overwritten, but the path must exist.</summary>
		public void Download(string remote_filename, string local_filepath, ResumeBehaviour resume = ResumeBehaviour.DontResume)
		{
			Trace(string.Format("Downloading: {0} from {1}/{2} to {3}" ,remote_filename ,Settings.RemoteHost ,Settings.RemotePath ,local_filepath));
			Reply reply;
			
			// If the local filepath is a directory, then append the remote filename as the local filename
			if (Directory.Exists(local_filepath))
				local_filepath = Path.Combine(local_filepath, remote_filename);
			
			// Download the file
			using (FileStream output = new FileStream(local_filepath, FileMode.OpenOrCreate, FileAccess.Write))
			{
				long offset = output.Length;
				if (resume != ResumeBehaviour.DontResume && offset != 0)
				{
					// Some servers may not support resuming.
					if (SendCommand("REST " + offset).Code == Status.PendingFurtherInfo)
					{
						Trace("Resuming download at offset " + offset);
						output.Seek(offset, SeekOrigin.Begin);
					}
					else if (resume == ResumeBehaviour.Resume)
					{
						// The caller specifically wanted resuming but it's
						// not supported, throw so they can choose what to do.
						Trace("Resuming not supported. Aborting.");
						throw new NotSupportedException("Resumed download not supported by server");
					}
					else
					{
						Trace("Resuming not supported. Restarting.");
						output.Seek(0, SeekOrigin.Begin);
					}
				}
				
				// Retrieve the file
				reply = SendCommand("RETR " + remote_filename);
				if (reply.Code != Status.DataConnectionOpenning && reply.Code != Status.DataConnectionAlreadyOpen)
					throw new IOException(reply.Message);
				
				// Read the file data from the data channel
				using (BinaryReader r = new BinaryReader(ConnectDataStream()))
					r.BaseStream.CopyTo(output);
			}
			
			// Confirm done
			reply = ReadReply();
			if (reply.Code != Status.DataConnectionClosing && reply.Code != Status.RequestedActionOkay)
				throw new IOException(reply.Message);
		}

		/// <summary>Download a remote file to the local directory, keeping the same file name.</summary>
		public void Download(string remote_filename, ResumeBehaviour resume = ResumeBehaviour.DontResume)
		{
			Download(remote_filename, @".\"+remote_filename, resume);
		}

		/// <summary>Upload a local file to the host. The remote file will be named 'remote_filename' on the host</summary>
		public void Upload(string local_filepath, string remote_filename, ResumeBehaviour resume = ResumeBehaviour.DontResume)
		{
			Trace(string.Format("Uploading: {0} to {1}/{2}/{3}" ,local_filepath ,Settings.RemoteHost ,Settings.RemotePath ,remote_filename));
			Reply reply;
			
			// Check the local filepath exists first
			if (!Path_.FileExists(local_filepath))
				throw new FileNotFoundException("Upload failed", local_filepath);
			
			// Upload the file
			using (var input = new FileStream(local_filepath, FileMode.Open, FileAccess.Read))
			{
				if (resume != ResumeBehaviour.DontResume)
				{
					// Some servers may not support resumed uploading.
					long offset = RemoteFileSize(remote_filename);
					if (SendCommand("REST " + offset).Code == Status.PendingFurtherInfo)
					{
						Trace("Resuming file upload from offset " + offset);
						input.Seek(offset, SeekOrigin.Begin);
					}
					else if (resume == ResumeBehaviour.Resume)
					{
						// The caller specifically wanted resuming but it's
						// not supported, throw so they can choose what to do.
						Trace("Resuming not supported. Aborting.");
						throw new NotSupportedException("Resumed upload not supported by server");
					}
					else
					{
						Trace("Resuming not supported. Restarting.");
						input.Seek(0, SeekOrigin.Begin);
					}
				}
				
				// Store the file
				reply = SendCommand("STOR " + remote_filename);
				if (reply.Code != Status.DataConnectionAlreadyOpen && reply.Code != Status.DataConnectionOpenning)
					throw new IOException(reply.Message);
				
				// Write the file to the data channel
				using (BinaryWriter w = new BinaryWriter(ConnectDataStream()))
					input.CopyTo(w.BaseStream);
			}
			
			// Confirm done
			reply = ReadReply();
			if (reply.Code != Status.DataConnectionClosing && reply.Code != Status.RequestedActionOkay)
				throw new IOException(reply.Message);
		}

		/// <summary>Upload a local file to the host. The remote file will have the same name on the host as the local file.</summary>
		public void Upload(string local_filepath, ResumeBehaviour resume = ResumeBehaviour.DontResume)
		{
			Upload(local_filepath, Path.GetFileName(local_filepath), resume);
		}

		/// <summary>Delete a file from the remote FTP server.</summary>
		public void DeleteFile(string filename)
		{
			Reply reply = SendCommand("DELE " + filename);
			if (reply.Code != Status.RequestedActionOkay)
				throw new IOException(reply.Message);
		}

		/// <summary>Rename a file on the remote FTP server.</summary>
		public void RenameFile(string old_filename, string new_filename)
		{
			Reply reply = SendCommand("RNFR " + old_filename);
			if (reply.Code != Status.PendingFurtherInfo)
				throw new IOException(reply.Message);

			// Known problem: RNTO will overwrite 'new_filename' if it exists
			reply = SendCommand("RNTO " + new_filename);
			if (reply.Code != Status.RequestedActionOkay)
				throw new IOException(reply.Message);
		}

		/// <summary>Create a directory on the remote FTP server.</summary>
		public void CreateDirectory(string remote_directory)
		{
			Reply reply = SendCommand("MKD " + remote_directory);
			if (reply.Code != Status.RequestedActionOkay)
				throw new IOException(reply.Message);
		}

		/// <summary>Delete a directory on the remote FTP server.</summary>
		public void RemoveDirectory(string remote_directory)
		{
			Reply reply = SendCommand("RMD " + remote_directory);
			if (reply.Code != Status.RequestedActionOkay)
				throw new IOException(reply.Message);
			
			Trace("Deleted directory '" + remote_directory + "'");
		}

		/// <summary>Change the current working directory on the remote FTP server.</summary>
		public void ChangeRemoteDirectory(string remote_directory)
		{
			if (remote_directory == ".") return;
			
			// Send the change directory command
			Reply reply = SendCommand("CWD " + remote_directory);
			if (reply.Code != Status.RequestedActionOkay)
				throw new IOException(reply.Message);
			
			Trace("Remote directory is now '" + remote_directory + "'");
		}

		///<summary>FTP command reply codes</summary>
		public enum Status
		{
			Unknown = -1,

			// 100 Series - The requested action was initiated; expect another reply before proceeding with a new command.
			RestartMarkerReply = 110, // Restart marker reply. The text is exact and not left to the particular implementation; it must read "MARK yyyy = mmmm" where yyyy is User-process data stream marker, and mmmm server's equivalent marker (note the spaces between markers and "=").
			ServiceReadyIn = 120, // Service ready in nn minutes.
			DataConnectionAlreadyOpen = 125, // Data Connection already open; transfer starting.
			DataConnectionOpenning = 150, // File status okay; about to open data connection. FTP uses two ports: 21 for sending commands, and 20 for sending data. A status code of 150 indicates that the server is about to open a new connection on port 20 to send some data.

			// 200 Series - The requested action has been successfully completed.
			CommandOkay = 200, // Command okay.
			CommandSuperfluous = 202, // Command not implemented, superfluous at this site.
			SystemStatus = 211, // System status, or system help reply.
			DirectoryStatus = 212, // Directory status.
			FileStatus = 213, // File status.
			HelpMessage = 214, // Help message.
			NameSystemType = 215, // NAME system type. Where NAME is an official system name from the list in the Assigned Numbers document.
			ServiceReady = 220, // Service ready for new user.
			ServiceClosing = 221, // Service closing control connection. Logged out if appropriate.
			DataConnectionOpen = 225, // Data connection open; no transfer in progress.
			DataConnectionClosing = 226, // Closing data connection. Requested file action successful (for example; file transfer or file abort). The command opens a data connection on port 20 to perform an action, such as transferring a file. This action successfully completes, and the data connection is closed.
			EnteringPassiveMode = 227, // Entering Passive Mode. (h1,h2,h3,h4,p1,p2)
			UserLoggedIn = 230, // User logged in, proceed. This status code appears after the client sends the correct password. It indicates that the user has successfully logged on.
			RequestedActionOkay = 250, // Requested file action okay, completed.
			PathnameCreated = 257, // "PATHNAME" created.

			// 300 Series - The command has been accepted, but the requested action is on hold, pending receipt of further information.
			UserNameOkay = 331, // User name okay, need password. You see this status code after the client sends a user name, regardless of whether the user name that is provided is a valid account on the system.
			NeedAccountForLogin = 332, // Need account for login. Provide login credentials
			PendingFurtherInfo = 350, // Requested file action pending further information.

			// 400 Series - The command was not accepted and the requested action did not take place, but the error condition is temporary and the action may be requested again.
			ServiceNotAvailable = 421, // Error 421 Service not available, closing control connection. Error 421 User limit reached Error 421 You are not authorized to make the connection Error 421 Max connections reached Error 421 Max connections exceeded   This can be a reply to any command if the service knows it must shut down. Try logging in later.
			CannotOpenDataConnection = 425, // Cannot open data connection. Change from PASV to PORT mode, check your firewall settings, or try to connect via HTTP.
			ConnectionClosed = 426, // Connection closed; transfer aborted. The command opens a data connection to perform an action, but that action is canceled, and the data connection is closed. Try logging back in; contact your hosting provider to check if you need to increase your hosting account; try disabling the firewall on your PC to see if that solves the problem. If not, contact your hosting provider or ISP.
			FileNotAvailable = 450, // Requested file action not taken. File unavailable (e.g., file busy). Try again later.
			RequestedActionAborted = 451, // Requested action aborted: local error in processing. Ensure command and parameters were typed correctly.
			InsufficientStorage = 452, // Requested action not taken. Insufficient storage space in system. Ask FTP administrator to increase allotted storage space, or archive/delete remote files.

			// 500 Series - The command was not accepted and the requested action did not take place.
			SyntaxError = 500, // Syntax error, command unrecognized, command line too long. Try switching to passive mode.
			SyntaxErrorInArgs = 501, // Syntax error in parameters or arguments. Verify your input; e.g., make sure there are no erroneous characters, spaces, etc.
			CommandNotSupported = 502, // Command not implemented. The server does not support this command.
			BadSequenceOfCommands = 503, // Bad sequence of commands. Verify command sequence.
			ParameterNotSupported = 504, // Command not implemented for that parameter. Ensure entered parameters are correct.
			UserNotLoggedIn = 530, // User not logged in. Ensure that you typed the correct user name and password combination. Some servers use this code instead of 421 when the user limit is reached
			NeedAccountToStoreFiles = 532, // Need account for storing files. Logged in user does not have permission to store files on remote server.
			FileNotFound = 550, // Requested action not taken. File unavailable, not found, not accessible. Verify that you are attempting to connect to the correct server/location. The administrator of the remote server must provide you with permission to connect via FTP.
			ExceededStorageAllocation = 552, // Requested file action aborted. Exceeded storage allocation. More disk space is needed. Archive files on the remote server that you no longer need.
			FileNameNotAllowed = 553, // Requested action not taken. File name not allowed. Change the file name or delete spaces/special characters in the file name.

			// 10,000 Series - Common Winsock Error Codes (complete list of Winsock error codes)
			ConnectionResetByPeer = 10054, // Connection reset by peer. The connection was forcibly closed by the remote host.
			ConnectionFailedTimeout = 10060, // Cannot connect to remote server. Generally a time-out error. Try switching from PASV to PORT mode, or try increasing the time-out value.
			ConnectionRefused = 10061, // Cannot connect to remote server. The connection is actively refused by the server. Try switching the connection port.
			DirectoryNotEmpty = 10066, // Directory not empty. The server will not delete this directory while there are files/folders in it. If you want to remove the directory, first archive or delete the files in it.
			TooManyUsers = 10068, // Too many users, server is full. Try logging in at another time.
		}

		/// <summary>Download/Upload resume behaviour</summary>
		public enum ResumeBehaviour
		{
			/// <summary>Overwrites any existing file and downloads/uploads the whole file</summary>
			DontResume,

			/// <summary>Resume downloading if the local file exists, or resume uploading if the remote file exists. Throws if resuming is not supported</summary>
			Resume,

			/// <summary>Use 'Resume' behaviour if the server supports it, otherwise use the 'DontResume' behaviour.</summary>
			ResumeIfSupported
		}

		/// <summary>Connection settings</summary>
		public class SettingsData
		{
			/// <summary>The address to connect to</summary>
			public string RemoteHost = string.Empty;

			/// <summary>The port to connect to. 990 is the SSL port</summary>
			public int RemotePort = 990;

			/// <summary>The path on the remote server</summary>
			public string RemotePath = ".";

			/// <summary>The user name to log on with</summary>
			public string UserName = string.Empty;

			/// <summary>The password to log on with</summary>
			public string Password = string.Empty;

			/// <summary>Set to true to login using SSL</summary>
			public bool UseSSL = true;

			/// <summary>The SSL protocols to support</summary>
			public SslProtocols SSLProtocols = SslProtocols.None;
		}

		/// <summary>The response message that acknowledges a command</summary>
		public class Reply
		{
			public Reply(string reply)
			{
				bool valid = reply.Length >= 3;
				Message = valid ? reply.Substring(4) : string.Empty;
				Code = valid ? (Status)int.Parse(reply.Substring(0, 3)) : Status.Unknown;
			}

			/// <summary>The full reply message</summary>
			public string Message { get; }

			/// <summary>The reply code parsed from 'Message'</summary>
			public Status Code { get; }
		}
	}

	/// <summary>Utilities for ftp-related global methods</summary>
	public static class FTPUtil
	{
		/// <summary>Returns a string containing info about the certificate</summary>
		public static string DumpInfo(this X509Certificate? cert, bool verbose)
		{
			if (cert is null)
				return "No certificate";

			var s = new StringBuilder().AppendFormat(
				"Certficate Information for: {0}\n"+
				"Certificate Format: {1}\n"+
				"Issuer Name: {2}\n"+
				" Valid From: {3}\n"+
				"   Valid To: {4}\n"
				,cert.Subject
				,cert.GetFormat()
				,cert.Issuer
				,cert.GetEffectiveDateString()
				,cert.GetExpirationDateString());
			if (verbose)
			{
				s.AppendFormat(
					"Serial Number: {0}\n"+
					"Hash: {1}\n"+
					"Key Algorithm: {2}\n"+
					"Key Algorithm Parameters: {3}\n"+
					"Public Key:\n{4}\n"
					,cert.GetSerialNumberString()
					,cert.GetCertHashString()
					,cert.GetKeyAlgorithm()
					,cert.GetKeyAlgorithmParametersString()
					,cert.GetPublicKeyString());
			}
			return s.ToString();
		}

		/// <summary>Dump info about an ssl stream</summary>
		public static string DumpInfo(this SslStream ssl_stream, string server_name, bool verbose)
		{
			return new StringBuilder(ssl_stream.RemoteCertificate.DumpInfo(verbose)).AppendFormat(
				"\n\nSSL Connect Report for : {0}\n"+
				"Is Authenticated: {1}\n"+
				"Is Encrypted: {2}\n"+
				"Is Signed: {3}\n"+
				"Is Mutually Authenticated: {4}\n\n"+
#if NETFRAMEWORK
				"Hash Algorithm: {5}\n"+
				"Hash Strength: {6}\n"+
				"Cipher Algorithm: {7}\n"+
				"Cipher Strength: {8}\n"+
				"Key Exchange Algorithm: {9}\n"+
				"Key Exchange Strength: {10}\n"+
				"SSL Protocol: {11}"
				,server_name
				,ssl_stream.IsAuthenticated
				,ssl_stream.IsEncrypted
				,ssl_stream.IsSigned
				,ssl_stream.IsMutuallyAuthenticated
#pragma warning disable SYSLIB0058
				,ssl_stream.HashAlgorithm
				,ssl_stream.HashStrength
				,ssl_stream.CipherAlgorithm
				,ssl_stream.CipherStrength
				,ssl_stream.KeyExchangeAlgorithm
				,ssl_stream.KeyExchangeStrength
#pragma warning restore SYSLIB0058
				,ssl_stream.SslProtocol
#else
				"Negotiated Cipher Suite: {5}\n"+
				"SSL Protocol: {6}"
				,server_name
				,ssl_stream.IsAuthenticated
				,ssl_stream.IsEncrypted
				,ssl_stream.IsSigned
				,ssl_stream.IsMutuallyAuthenticated
				,ssl_stream.NegotiatedCipherSuite
				,ssl_stream.SslProtocol
#endif
				).ToString();
		}
	}
}

#if PR_UNITTESTS
namespace Rylogic.UnitTests
{
	using Net;

	[TestFixture] public class TestFtp
	{
		//hack[Test]
		public void FTPS()
		{
			try
			{
				var settings = new FTPConnection.SettingsData
				{
					RemoteHost = "192.168.0.150",
					RemotePort = 21,
					UserName = "Friend",
					Password = "37awatea",
					UseSSL = false
				};
				using(var ftp = new FTPConnection())
				{
					ftp.TraceMessage += (s,a)=>{ System.Diagnostics.Debug.Write(string.Format(s,a)); };
					ftp.Login(settings);
					ftp.AsciiMode = true;
					ftp.Upload(@"D:\deleteme\test.html");
					ftp.Close();
				}
			}
			catch (Exception e)
			{
				System.Diagnostics.Debug.WriteLine("Caught Error :" + e.Message);
			}
		}
	}
}
#endif
