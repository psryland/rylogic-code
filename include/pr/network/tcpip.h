#pragma once
#include <string>
#include <sstream>
#include <vector>
#include <span>
#include <algorithm>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <atomic>
#include <cassert>
#include <exception>
#include "pr/network/sockets.h"

namespace pr::network
{
	// A TCP socket with server behaviour
	class TcpServer :private ServerSocket
	{
		// Create m_listen_socket to use for listening on
		virtual Socket CreateListenSocket(int port) override
		{
			// Create the listen socket
			auto socket = Socket(::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP));
			if (socket == INVALID_SOCKET)
				Throw(WSAGetLastError());

			// Allow address reuse to avoid WSAEADDRINUSE when restarting quickly
			int reuse = 1;
			if (::setsockopt(socket, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<char const*>(&reuse), sizeof(reuse)) == SOCKET_ERROR)
				Throw(WSAGetLastError());

			// Bind the local address to the socket
			sockaddr_in my_address = {};
			my_address.sin_family = AF_INET;
			my_address.sin_addr.S_un.S_addr = INADDR_ANY;
			my_address.sin_port = htons(static_cast<u_short>(port));
			auto result = ::bind(socket, (sockaddr const*)&my_address, sizeof(my_address));
			if (result == SOCKET_ERROR)
				Throw(WSAGetLastError());

			return socket;
		}

	public:

		explicit TcpServer(Winsock const& winsock)
			:ServerSocket(winsock)
		{}
		TcpServer(TcpServer const&) = delete;
		TcpServer& operator =(TcpServer const&) = delete;

		// True if the server is listening for connections
		bool Listening() const
		{
			return ServerSocket::Listening();
		}

		// The port we're listening on
		uint16_t ListenPort() const
		{
			return ServerSocket::ListenPort();
		}

		// Turn on/off the server
		// 'listen_port' is a port number of your choosing
		// 'connect_cb' should have the signature: void ConnectionCB(void* ctx, SOCKET socket, sockaddr_in const* client_addr);
		// 'client_addr' will be non-null for connections, null for disconnections
		void AllowConnections(uint16_t listen_port, ConnectionCB connect_cb, int max_connections = SOMAXCONN)
		{
			ServerSocket::AllowConnections(listen_port, connect_cb, max_connections);
		}
		void AllowConnections(uint16_t listen_port, int max_connections = SOMAXCONN)
		{
			AllowConnections(listen_port, [](SOCKET, sockaddr_in const*){}, max_connections);
		}

		// Block until 'client_count' connections have been made
		bool WaitForClients(size_t client_count, int timeout_ms = ~0)
		{
			return ServerSocket::WaitForClients(client_count, timeout_ms);
		}

		// Stop accepting incoming connections
		void StopConnections()
		{
			ServerSocket::StopConnections();
		}

		// Return the number of connected clients
		size_t ClientCount() const
		{
			return ServerSocket::ClientCount();
		}

		// Send data to all clients
		bool Send(std::span<uint8_t const> data, int timeout_ms = ~0)
		{
			return ServerSocket::SendStream(data, timeout_ms);
		}

		// Receive data from any client
		// Returns true when data is read from a client, 'out_client' is the client that was read from
		// Returns false if no data was read from any client.
		// Throws if a connection was aborted, or had a problem
		bool Recv(std::span<uint8_t> data, size_t& bytes_read, int timeout_ms = 0, SOCKET* out_client = nullptr)
		{
			return ServerSocket::RecvStream(data, bytes_read, timeout_ms, out_client);
		}
		bool Recv(std::span<uint8_t> data, int timeout_ms = ~0, SOCKET* out_client = nullptr)
		{
			size_t bytes_read;
			return Recv(data, bytes_read, timeout_ms, out_client);
		}
	};

	// A TCP socket with client behaviour
	class TcpClient :private ClientSocket
	{
	public:

		explicit TcpClient(Winsock const& winsock)
			:ClientSocket(winsock)
		{}
		TcpClient(TcpClient&&) = default;
		TcpClient(TcpClient const&) = delete;
		TcpClient& operator =(TcpClient&&) = default;
		TcpClient& operator =(TcpClient const&) = delete;

		// (Re)create 'm_socket'
		// Typically applications can just call 'Connect', however some socket options
		// need to be set after creating the socket but before connecting, in this case
		// applications can call 'CreateSocket' first, then connect.
		void CreateSocket()
		{
			Close();

			// Create the socket
			m_socket = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
			if (m_socket == INVALID_SOCKET)
				Throw(WSAGetLastError());
		}

		// For a TCP connections, use IPPROTO_TCP, the ip address and port
		// For a UDP connection with a default ip/port, use IPPROTO_UDP, ip, port
		//  Send/Recv can be used with this type of connection, the UDP packets go
		//  to the specified ip/port. I.e connect sets them as the default ip/port.
		// For a UDP conection without any default ip/port, use 'nullptr, 0, IPPROTO_UDP'
		//  Send/Recv return errors for this connection type, however, SendTo and RecvFrom work.
		// Returns true if the connection is established, false on timeout. Throws on failure
		bool Connect(char const* ip, uint16_t port, int timeout_ms = ~0)
		{
			if (m_socket == INVALID_SOCKET)
				CreateSocket();

			// If an ip address is given, attempt to connect to it
			// It won't be given for UDP connections
			if (ip != nullptr)
			{
				sockaddr_in host_addr = GetAddress(ip, port);
				int result = ::connect(m_socket, (sockaddr*)&host_addr, sizeof(host_addr));
				if (result == SOCKET_ERROR)
					Throw(WSAGetLastError());

				// Wait for the socket to say it's writable (meaning it's connected)
				fd_set set = {};
				FD_SET(m_socket, &set);
				auto timeout = TimeVal(timeout_ms);
				result = ::select(0, 0, &set, 0, timeout_ms == ~0 ? nullptr : &timeout);
				if (result == 0)
					return false;
				if (result == SOCKET_ERROR)
					Throw(WSAGetLastError());
			}
			return true;
		}

		// Close the socket
		void Close()
		{
			ClientSocket::Close();
		}

		// True if a the socket is connected to a host
		bool IsConnected() const
		{
			return ClientSocket::IsValid();
		}

		// Send/Recv data to/from the host
		// Returns true if all data was sent/received
		// Returns false if the connection to the client was closed gracefully
		// Throws if the connection was aborted, or had a problem
		bool Send(std::span<uint8_t const> data, int timeout_ms = ~0)
		{
			return ClientSocket::SendStream(data, timeout_ms);
		}
		bool Recv(std::span<uint8_t> data, size_t& bytes_read, int timeout_ms = ~0)
		{
			return ClientSocket::RecvStream(data, bytes_read, timeout_ms);
		}

		// Get/Set a socket option
		// 'level' - The level at which the option is defined. Example: SOL_SOCKET.
		// 'optname' - The socket option for which the value is to be retrieved. Example: SO_ACCEPTCONN.
		//  The 'optname' value must be a socket option defined within the specified level, or behaviour is undefined.
		// 'optval' [out] - A pointer to the buffer in which the value for the requested option is to be returned.
		// 'optlen' [in, out] - The size, in bytes, of the 'optval' buffer.
		template <typename OptType>
		OptType SocketOption(int level, int optname) const
		{
			OptType opt; int len = sizeof(opt);
			ClientSocket::GetSocketOption(level, optname, (char*)&opt, len);
			return opt;
		}
		template <typename OptType>
		void SocketOption(int level, int optname, OptType opt)
		{
			ClientSocket::SetSocketOption(level, optname, (char const*)&opt, sizeof(opt));
		}
	};

	// A UDP socket (client or server)
	class UdpClient
	{
		// Notes:
		//  - Modeled on the C# UdpClient class
		//  - UDP is connectionless so does not need to be based on ServerSocket.
		//  - UDP is intended to work like this:
		//      Server listens on the listen port for a message.
		//      The received message contains the end-point of the sender.
		//      Server replies with a message using the end-point from the received message

		Winsock const& m_winsock;         // The winsock instance we're bound to
		SOCKET m_socket;
		int m_listen_port;

	public:

		explicit UdpClient(Winsock const& winsock)
			:m_winsock(winsock)
			,m_socket(INVALID_SOCKET)
			,m_listen_port()
		{
		}
		UdpClient(Winsock const& winsock, int listen_port)
			:UdpClient(winsock)
		{
			Connect(listen_port);
		}
		UdpClient(UdpClient const&) = delete;
		UdpClient& operator =(UdpClient const&) = delete;
		~UdpClient()
		{
			Close();
		}

		// Create the socket, and bind it to a port
		void Connect(int listen_port)
		{
			Close();

			// Create the listen socket
			auto socket = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
			if (socket == INVALID_SOCKET)
				Throw(WSAGetLastError());

			// Bind the socket to an end-point
			SOCKADDR_IN ep = {};
			ep.sin_family = AF_INET;
			ep.sin_addr.S_un.S_addr = INADDR_ANY;
			ep.sin_port = htons(static_cast<u_short>(listen_port));
			int result = ::bind(socket, (sockaddr const*)&ep, sizeof(ep));
			if (result == SOCKET_ERROR)
				Throw(WSAGetLastError());

			m_socket = socket;
			m_listen_port = listen_port;
		}

		// Close the socket
		void Close()
		{
			if (m_socket == INVALID_SOCKET)
				return;
			
			::shutdown(m_socket, SD_BOTH);
			::closesocket(m_socket);
			m_socket = INVALID_SOCKET;
		}

		// Send data.
		// Returns true if all data was sent
		bool Send(std::span<uint8_t const> data, SOCKADDR_IN ep, int timeout_ms = ~0)
		{
			return network::SendDatagram(m_socket, data, timeout_ms, ep);
		}

		// Receive data from any client
		// Returns true when data is read from a client
		// Returns false if no data was read from any client.
		// Throws if a connection was aborted, or had a problem
		bool Recv(std::span<uint8_t> data, size_t& bytes_read, SOCKADDR_IN* ep = nullptr, int timeout_ms = 0)
		{
			SOCKADDR_IN sender;
			ep = ep ? ep : &sender;
			return network::RecvDatagram(m_socket, data, bytes_read, timeout_ms, *ep);
		}
	};
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"

namespace pr::network
{
	PRUnitTestFamily(TcpIpTests, Slow)
	{
		uint16_t TestPort = 54321;
		if constexpr (sizeof(void*) == 8)
			TestPort += 2;
		if constexpr (PR_DBG)
			TestPort += 1;

		Winsock wsa;
		{
			volatile bool connected = false;

			// If this fails with WSAEACCESS, it's probably because the firewall is blocking unittest.exe/.dll
			TcpServer svr(wsa);
			svr.AllowConnections(TestPort, [&](SOCKET, sockaddr_in const*)
			{
				connected = true;
			});

			TcpClient client(wsa);
			client.Connect("127.0.0.1", TestPort);

			PR_EXPECT(svr.WaitForClients(1));
			PR_EXPECT(connected);

			uint8_t const data[] = "Test data";
			PR_EXPECT(svr.Send(data));

			size_t bytes_read;
			uint8_t result[sizeof(data)] = {};
			PR_EXPECT(client.Recv(result, bytes_read));

			PR_EXPECT(memcmp(&data[0], &result[0], sizeof(data)) == 0);
			svr.StopConnections();
		}
		{
			TcpServer svr(wsa);
			svr.AllowConnections(TestPort, 10);

			TcpClient client(wsa);
			client.Connect("127.0.0.1", TestPort);

			PR_EXPECT(svr.WaitForClients(1));

			uint8_t const data[] = "Test data";
			PR_EXPECT(client.Send(data, sizeof(data)));

			uint8_t result[sizeof(data)] = {};
			PR_EXPECT(svr.Recv(result, sizeof(result)));

			PR_EXPECT(memcmp(&data[0], &result[0], sizeof(data)) == 0);

			client.Close();
			svr.StopConnections();
		}
	}
}
#endif
