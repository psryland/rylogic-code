//*****************************************
// WinSock
//	Copyright (c) Rylogic 2019
//*****************************************
#pragma once

#if defined(_INC_WINDOWS) && !defined(_WINSOCK2API_)
#error "winsock2.h must be included before windows.h"
#endif

#include <string>
#include <string_view>
#include <format>
#include <stdexcept>
#include <ws2tcpip.h>
#include <winsock2.h>

#pragma comment(lib, "ws2_32.lib")

namespace pr::network
{
	// This is a wrapper of the 'winsock' dll. An instance of this object should have the scope of all network activity
	struct Winsock :WSADATA
	{
		Winsock(int major = 2, int minor = 2) :WSADATA()
		{
			auto version = MAKEWORD(major, minor);
			if (WSAStartup(version, this) != 0)
				throw std::runtime_error("WSAStartup failed");
			if (wVersion != version)
				throw std::runtime_error("WSAStartup - incorrect version");
		}
		~Winsock()
		{
			WSACleanup();
		}
	};

	// Scoped SOCKET
	struct Socket
	{
		SOCKET m_socket;

		Socket()
			:m_socket(INVALID_SOCKET)
		{}
		Socket(SOCKET s)
			:m_socket(s)
		{}
		Socket(int af, int type, int protocol)
			:m_socket(::socket(af, type, protocol))
		{}
		Socket(Socket&& rhs) noexcept
			:m_socket(INVALID_SOCKET)
		{
			std::swap(m_socket, rhs.m_socket);
		}
		Socket(Socket const&) = delete;
		Socket& operator =(Socket&& rhs) noexcept
		{
			if (this != &rhs)
				std::swap(m_socket, rhs.m_socket);

			return *this;
		}
		Socket& operator = (Socket const&) = delete;
		Socket& operator=(nullptr_t)
		{
			// Swap in an empty socket so the temporary closes the current one
			return *this = Socket{};
		}
		~Socket()
		{
			if (m_socket != INVALID_SOCKET)
			{
				::shutdown(m_socket, SD_BOTH);
				::closesocket(m_socket);
			}
		}	
		operator SOCKET() const
		{
			return m_socket;
		}
		explicit operator bool() const
		{
			return m_socket != INVALID_SOCKET;
		}
		friend bool operator == (Socket const& lhs, nullptr_t)
		{
			return lhs.m_socket == INVALID_SOCKET;
		}
		friend bool operator != (Socket const& lhs, nullptr_t)
		{
			return !(lhs == nullptr);
		}
	};

	// Return the string representation of a socket error
	constexpr char const* SocketErrorToMsg(DWORD code)
	{
		switch (code)
		{
			default:                         return "Unknown socket error";
			case WSA_INVALID_PARAMETER:      return "One or more parameters are invalid.\r\nAn application used a Windows Sockets function which directly maps to a Windows function. The Windows function is indicating a problem with one or more parameters. Note that this error is returned by the operating system, so the error number may change in future releases of Windows.";
			case WSA_OPERATION_ABORTED:      return "Overlapped operation aborted.\r\nAn overlapped operation was canceled due to the closure of the socket, or the execution of the SIO_FLUSH command in WSAIoctl. Note that this error is returned by the operating system, so the error number may change in future releases of Windows.";
			case WSA_IO_INCOMPLETE:          return "Overlapped I/O event object not in signaled state.\r\nThe application has tried to determine the status of an overlapped operation which is not yet completed. Applications that use WSAGetOverlappedResult (with the fWait flag set to FALSE) in a polling mode to determine when an overlapped operation has completed, get this error code until the operation is complete. Note that this error is returned by the operating system, so the error number may change in future releases of Windows.";
			case WSA_IO_PENDING:             return "Overlapped operations will complete later.\r\nThe application has initiated an overlapped operation that cannot be completed immediately. A completion indication will be given later when the operation has been completed. Note that this error is returned by the operating system, so the error number may change in future releases of Windows.";
			case WSAEINTR:                   return "Interrupted function call.\r\nA blocking operation was interrupted by a call to WSACancelBlockingCall.";
			case WSAEBADF:                   return "File handle is not valid.\r\nThe file handle supplied is not valid.";
			case WSAEACCES:                  return "Access to the socket is denied. It is either in-use, or being denied by firewall or antivirus software.\r\nAn attempt was made to access a socket in a way forbidden by its access permissions. An example is using a broadcast address for sendto without broadcast permission being set using setsockopt(SO_BROADCAST).\r\nAnother possible reason for the WSAEACCES error is that when the bind function is called (on Windows NT 4.0 with SP4 and later), another application, service, or kernel mode driver is bound to the same address with exclusive access. Such exclusive access is a new feature of Windows NT 4.0 with SP4 and later, and is implemented by using the SO_EXCLUSIVEADDRUSE option.";
			case WSAEFAULT:                  return "Bad address.\r\nThe system detected an invalid pointer address in attempting to use a pointer argument of a call. This error occurs if an application passes an invalid pointer value, or if the length of the buffer is too small. For instance, if the length of an argument, which is a sockaddr structure, is smaller than the sizeof(sockaddr).";
			case WSAEINVAL:                  return "Invalid argument.\r\nSome invalid argument was supplied (for example, specifying an invalid level to the setsockopt function). In some instances, it also refers to the current state of the socket. For instance, calling accept on a socket that is not listening.";
			case WSAEMFILE:                  return "Too many open files.\r\nToo many open sockets. Each implementation may have a maximum number of socket handles available, either globally, per process, or per thread.";
			case WSAEWOULDBLOCK:             return "Resource temporarily unavailable.\r\nThis error is returned from operations on nonblocking sockets that cannot be completed immediately, for example recv when no data is queued to be read from the socket. It is a nonfatal error, and the operation should be retried later. It is normal for WSAEWOULDBLOCK to be reported as the result from calling connect on a nonblocking SOCK_STREAM socket, since some time must elapse for the connection to be established.";
			case WSAEINPROGRESS:             return "Operation now in progress.\r\nA blocking operation is currently executing. Windows Sockets only allows a single blocking operation-per- task or thread-to be outstanding, and if any other function call is made (whether or not it references that or any other socket) the function fails with the WSAEINPROGRESS error.";
			case WSAEALREADY:                return "Operation already in progress.\r\nAn operation was attempted on a nonblocking socket with an operation already in progress-that is, calling connect a second time on a nonblocking socket that is already connecting, or canceling an asynchronous request (WSAAsyncGetXbyY) that has already been canceled or completed.";
			case WSAENOTSOCK:                return "Socket operation on nonsocket.\r\nAn operation was attempted on something that is not a socket. Either the socket handle parameter did not reference a valid socket, or for select, a member of an fd_set was not valid.";
			case WSAEDESTADDRREQ:            return "Destination address required.\r\nA required address was omitted from an operation on a socket. For example, this error is returned if sendto is called with the remote address of ADDR_ANY.";
			case WSAEMSGSIZE:                return "Message too long.\r\nA message sent on a datagram socket was larger than the internal message buffer or some other network limit, or the buffer used to receive a datagram was smaller than the datagram itself.";
			case WSAEPROTOTYPE:              return "Protocol wrong type for socket.\r\nA protocol was specified in the socket function call that does not support the semantics of the socket type requested. For example, the ARPA Internet UDP protocol cannot be specified with a socket type of SOCK_STREAM.";
			case WSAENOPROTOOPT:             return "Bad protocol option.\r\nAn unknown, invalid or unsupported option or level was specified in a getsockopt or setsockopt call.";
			case WSAEPROTONOSUPPORT:         return "Protocol not supported.\r\nThe requested protocol has not been configured into the system, or no implementation for it exists. For example, a socket call requests a SOCK_DGRAM socket, but specifies a stream protocol.";
			case WSAESOCKTNOSUPPORT:         return "Socket type not supported.\r\nThe support for the specified socket type does not exist in this address family. For example, the optional type SOCK_RAW might be selected in a socket call, and the implementation does not support SOCK_RAW sockets at all.";
			case WSAEOPNOTSUPP:              return "Operation not supported.\r\nThe attempted operation is not supported for the type of object referenced. Usually this occurs when a socket descriptor to a socket that cannot support this operation is trying to accept a connection on a datagram socket.";
			case WSAEPFNOSUPPORT:            return "Protocol family not supported.\r\nThe protocol family has not been configured into the system or no implementation for it exists. This message has a slightly different meaning from WSAEAFNOSUPPORT. However, it is interchangeable in most cases, and all Windows Sockets functions that return one of these messages also specify WSAEAFNOSUPPORT.";
			case WSAEAFNOSUPPORT:            return "Address family not supported by protocol family.\r\nAn address incompatible with the requested protocol was used. All sockets are created with an associated address family (that is, AF_INET for Internet Protocols) and a generic protocol type (that is, SOCK_STREAM). This error is returned if an incorrect protocol is explicitly requested in the socket call, or if an address of the wrong family is used for a socket, for example, in sendto.";
			case WSAEADDRINUSE:              return "Address already in use.\r\nTypically, only one usage of each socket address (protocol/IP address/port) is permitted. This error occurs if an application attempts to bind a socket to an IP address/port that has already been used for an existing socket, or a socket that was not closed properly, or one that is still in the process of closing. For server applications that need to bind multiple sockets to the same port number, consider using setsockopt (SO_REUSEADDR). Client applications usually need not call bind at all-connect chooses an unused port automatically. When bind is called with a wildcard address (involving ADDR_ANY), a WSAEADDRINUSE error could be delayed until the specific address is committed. This could happen with a call to another function later, including connect, listen, WSAConnect, or WSAJoinLeaf.";
			case WSAEADDRNOTAVAIL:           return "Cannot assign requested address.\r\nThe requested address is not valid in its context. This normally results from an attempt to bind to an address that is not valid for the local computer. This can also result from connect, sendto, WSAConnect, WSAJoinLeaf, or WSASendTo when the remote address or port is not valid for a remote computer (for example, address or port 0).";
			case WSAENETDOWN:                return "Network is down.\r\nA socket operation encountered a dead network. This could indicate a serious failure of the network system (that is, the protocol stack that the Windows Sockets DLL runs over), the network interface, or the local network itself.";
			case WSAENETUNREACH:             return "Network is unreachable.\r\nA socket operation was attempted to an unreachable network. This usually means the local software knows no route to reach the remote host.";
			case WSAENETRESET:               return "Network dropped connection on reset.\r\nThe connection has been broken due to keep-alive activity detecting a failure while the operation was in progress. It can also be returned by setsockopt if an attempt is made to set SO_KEEPALIVE on a connection that has already failed.";
			case WSAECONNABORTED:            return "Software caused connection abort.\r\nAn established connection was aborted by the software in your host computer, possibly due to a data transmission time-out or protocol error.";
			case WSAECONNRESET:              return "Connection reset by peer.\r\nAn existing connection was forcibly closed by the remote host. This normally results if the peer application on the remote host is suddenly stopped, the host is rebooted, the host or remote network interface is disabled, or the remote host uses a hard close (see setsockopt for more information on the SO_LINGER option on the remote socket). This error may also result if a connection was broken due to keep-alive activity detecting a failure while one or more operations are in progress. Operations that were in progress fail with WSAENETRESET. Subsequent operations fail with WSAECONNRESET.";
			case WSAENOBUFS:                 return "No buffer space available.\r\nAn operation on a socket could not be performed because the system lacked sufficient buffer space or because a queue was full.";
			case WSAEISCONN:                 return "Socket is already connected.\r\nA connect request was made on an already-connected socket. Some implementations also return this error if sendto is called on a connected SOCK_DGRAM socket (for SOCK_STREAM sockets, the to parameter in sendto is ignored) although other implementations treat this as a legal occurrence.";
			case WSAENOTCONN:                return "Socket is not connected.\r\nA request to send or receive data was disallowed because the socket is not connected and (when sending on a datagram socket using sendto) no address was supplied. Any other type of operation might also return this error-for example, setsockopt setting SO_KEEPALIVE if the connection has been reset.";
			case WSAESHUTDOWN:               return "Cannot send after socket shutdown.\r\nA request to send or receive data was disallowed because the socket had already been shut down in that direction with a previous shutdown call. By calling shutdown a partial close of a socket is requested, which is a signal that sending or receiving, or both have been discontinued.";
			case WSAETOOMANYREFS:            return "Too many references.\r\nToo many references to some kernel object.";
			case WSAETIMEDOUT:               return "Connection timed out.\r\nA connection attempt failed because the connected party did not properly respond after a period of time, or the established connection failed because the connected host has failed to respond.";
			case WSAECONNREFUSED:            return "Connection refused.\r\nNo connection could be made because the target computer actively refused it. This usually results from trying to connect to a service that is inactive on the foreign host-that is, one with no server application running.";
			case WSAELOOP:                   return "Cannot translate name.\r\nCannot translate a name.";
			case WSAENAMETOOLONG:            return "Name too long.\r\nA name component or a name was too long.";
			case WSAEHOSTDOWN:               return "Host is down.\r\nA socket operation failed because the destination host is down. A socket operation encountered a dead host. Networking activity on the local host has not been initiated. These conditions are more likely to be indicated by the error WSAETIMEDOUT.";
			case WSAEHOSTUNREACH:            return "No route to host.\r\nA socket operation was attempted to an unreachable host. See WSAENETUNREACH.";
			case WSAENOTEMPTY:               return "Directory not empty.\r\nCannot remove a directory that is not empty.";
			case WSAEPROCLIM:                return "Too many processes.\r\nA Windows Sockets implementation may have a limit on the number of applications that can use it simultaneously. WSAStartup may fail with this error if the limit has been reached.";
			case WSAEUSERS:                  return "User quota exceeded.\r\nRan out of user quota.";
			case WSAEDQUOT:                  return "Disk quota exceeded.\r\nRan out of disk quota.";
			case WSAESTALE:                  return "Stale file handle reference.\r\nThe file handle reference is no longer available.";
			case WSAEREMOTE:                 return "Item is remote.\r\nThe item is not available locally.";
			case WSASYSNOTREADY:             return "Network subsystem is unavailable.\r\nThis error is returned by WSAStartup if the Windows Sockets implementation cannot function at this time because the underlying system it uses to provide network services is currently unavailable. Users should check:\r\nThat the appropriate Windows Sockets DLL file is in the current path.\r\nThat they are not trying to use more than one Windows Sockets implementation simultaneously. If there is more than one Winsock DLL on your system, be sure the first one in the path is appropriate for the network subsystem currently loaded.\r\nThe Windows Sockets implementation documentation to be sure all necessary components are currently installed and configured correctly.";
			case WSAVERNOTSUPPORTED:         return "Winsock.dll version out of range.\r\nThe current Windows Sockets implementation does not support the Windows Sockets specification version requested by the application. Check that no old Windows Sockets DLL files are being accessed.";
			case WSANOTINITIALISED:          return "Successful WSAStartup not yet performed.\r\nEither the application has not called WSAStartup or WSAStartup failed. The application may be accessing a socket that the current active task does not own (that is, trying to share a socket between tasks), or WSACleanup has been called too many times.";
			case WSAEDISCON:                 return "Graceful shutdown in progress.\r\nReturned by WSARecv and WSARecvFrom to indicate that the remote party has initiated a graceful shutdown sequence.";
			case WSAENOMORE:                 return "No more results.\r\nNo more results can be returned by the WSALookupServiceNext function.";
			case WSAECANCELLED:              return "Call has been canceled.\r\nA call to the WSALookupServiceEnd function was made while this call was still processing. The call has been canceled.";
			case WSAEINVALIDPROCTABLE:       return "Procedure call table is invalid.\r\nThe service provider procedure call table is invalid. A service provider returned a bogus procedure table to Ws2_32.dll. This is usually caused by one or more of the function pointers being NULL.";
			case WSAEINVALIDPROVIDER:        return "Service provider is invalid.\r\nThe requested service provider is invalid. This error is returned by the WSCGetProviderInfo and WSCGetProviderInfo32 functions if the protocol entry specified could not be found. This error is also returned if the service provider returned a version number other than 2.0.";
			case WSAEPROVIDERFAILEDINIT:     return "Service provider failed to initialize.\r\nThe requested service provider could not be loaded or initialized. This error is returned if either a service provider's DLL could not be loaded (LoadLibrary failed) or the provider's WSPStartup or NSPStartup function failed.";
			case WSASYSCALLFAILURE:          return "System call failure.\r\nA system call that should never fail has failed. This is a generic error code, returned under various conditions.\r\nReturned when a system call that should never fail does fail. For example, if a call to WaitForMultipleEvents fails or one of the registry functions fails trying to manipulate the protocol/namespace catalogs.\r\nReturned when a provider does not return SUCCESS and does not provide an extended error code. Can indicate a service provider implementation error.";
			case WSASERVICE_NOT_FOUND:       return "Service not found.\r\nNo such service is known. The service cannot be found in the specified name space.";
			case WSATYPE_NOT_FOUND:          return "Class type not found.\r\nThe specified class was not found.";
			case WSA_E_NO_MORE:              return "No more results.\r\nNo more results can be returned by the WSALookupServiceNext function.";
			case WSA_E_CANCELLED:            return "Call was canceled.\r\nA call to the WSALookupServiceEnd function was made while this call was still processing. The call has been canceled.";
			case WSAEREFUSED:                return "Database query was refused.\r\nA database query failed because it was actively refused.";
			case WSAHOST_NOT_FOUND:          return "Host not found.\r\nNo such host is known. The name is not an official host name or alias, or it cannot be found in the database(s) being queried. This error may also be returned for protocol and service queries, and means that the specified name could not be found in the relevant database.";
			case WSATRY_AGAIN:               return "Nonauthoritative host not found.\r\nThis is usually a temporary error during host name resolution and means that the local server did not receive a response from an authoritative server. A retry at some time later may be successful.";
			case WSANO_RECOVERY:             return "This is a nonrecoverable error.\r\nThis indicates that some sort of nonrecoverable error occurred during a database lookup. This may be because the database files (for example, BSD-compatible HOSTS, SERVICES, or PROTOCOLS files) could not be found, or a DNS request was returned by the server with a severe error.";
			case WSANO_DATA:                 return "Valid name, no data record of requested type.\r\nThe requested name is valid and was found in the database, but it does not have the correct associated data being resolved for. The usual example for this is a host name-to-address translation attempt (using gethostbyname or WSAAsyncGetHostByName) which uses the DNS (Domain Name Server). An MX record is returned but no A record-indicating the host itself exists, but is not directly reachable.";
			case WSA_QOS_RECEIVERS:          return "QoS receivers.\r\nAt least one QoS reserve has arrived.";
			case WSA_QOS_SENDERS:            return "QoS senders.\r\nAt least one QoS send path has arrived.";
			case WSA_QOS_NO_SENDERS:         return "No QoS senders.\r\nThere are no QoS senders.";
			case WSA_QOS_NO_RECEIVERS:       return "QoS no receivers.\r\nThere are no QoS receivers.";
			case WSA_QOS_REQUEST_CONFIRMED:  return "QoS request confirmed.\r\nThe QoS reserve request has been confirmed.";
			case WSA_QOS_ADMISSION_FAILURE:  return "QoS admission error.\r\nA QoS error occurred due to lack of resources.";
			case WSA_QOS_POLICY_FAILURE:     return "QoS policy failure.\r\nThe QoS request was rejected because the policy system couldn't allocate the requested resource within the existing policy.";
			case WSA_QOS_BAD_STYLE:          return "QoS bad style.\r\nAn unknown or conflicting QoS style was encountered.";
			case WSA_QOS_BAD_OBJECT:         return "QoS bad object.\r\nA problem was encountered with some part of the filterspec or the provider-specific buffer in general.";
			case WSA_QOS_TRAFFIC_CTRL_ERROR: return "QoS traffic control error.\r\nAn error with the underlying traffic control (TC) API as the generic QoS request was converted for local enforcement by the TC API. This could be due to an out of memory error or to an internal QoS provider error.";
			case WSA_QOS_GENERIC_ERROR:      return "QoS generic error.\r\nA general QoS error.";
			case WSA_QOS_ESERVICETYPE:       return "QoS service type error.\r\nAn invalid or unrecognized service type was found in the QoS flowspec.";
			case WSA_QOS_EFLOWSPEC:          return "QoS flowspec error.\r\nAn invalid or inconsistent flowspec was found in the QOS structure.";
			case WSA_QOS_EPROVSPECBUF:       return "Invalid QoS provider buffer.\r\nAn invalid QoS provider-specific buffer.";
			case WSA_QOS_EFILTERSTYLE:       return "Invalid QoS filter style.\r\nAn invalid QoS filter style was used.";
			case WSA_QOS_EFILTERTYPE:        return "Invalid QoS filter type.\r\nAn invalid QoS filter type was used.";
			case WSA_QOS_EFILTERCOUNT:       return "Incorrect QoS filter count.\r\nAn incorrect number of QoS FILTERSPECs were specified in the FLOWDESCRIPTOR.";
			case WSA_QOS_EOBJLENGTH:         return "Invalid QoS object length.\r\nAn object with an invalid ObjectLength field was specified in the QoS provider-specific buffer.";
			case WSA_QOS_EFLOWCOUNT:         return "Incorrect QoS flow count.\r\nAn incorrect number of flow descriptors was specified in the QoS structure.";
			case WSA_QOS_EUNKOWNPSOBJ:       return "Unrecognized QoS object.\r\nAn unrecognized object was found in the QoS provider-specific buffer.";
			case WSA_QOS_EPOLICYOBJ:         return "Invalid QoS policy object.\r\nAn invalid policy object was found in the QoS provider-specific buffer.";
			case WSA_QOS_EFLOWDESC:          return "Invalid QoS flow descriptor.\r\nAn invalid QoS flow descriptor was found in the flow descriptor list.";
			case WSA_QOS_EPSFLOWSPEC:        return "Invalid QoS provider-specific flowspec.\r\nAn invalid or inconsistent flowspec was found in the QoS provider-specific buffer.";
			case WSA_QOS_EPSFILTERSPEC:      return "Invalid QoS provider-specific filterspec.\r\nAn invalid FILTERSPEC was found in the QoS provider-specific buffer.";
			case WSA_QOS_ESDMODEOBJ:         return "Invalid QoS shape discard mode object.\r\nAn invalid shape discard mode object was found in the QoS provider-specific buffer.";
			case WSA_QOS_ESHAPERATEOBJ:      return "Invalid QoS shaping rate object.\r\nAn invalid shaping rate object was found in the QoS provider-specific buffer.";
			case WSA_QOS_RESERVED_PETYPE:    return "Reserved policy QoS element type.\r\nA reserved policy element was found in the QoS provider-specific buffer.";
		}
	}

	// Throw a socket error
	inline void Throw(DWORD code, std::string_view message = "")
	{
		auto err = SocketErrorToMsg(code);
		throw std::runtime_error(std::format("{} [{}] {}", message, code, err));
	}

	// Error checking helper
	inline void Check(bool success, std::string_view message = "")
	{
		if (success) return;
		Throw(WSAGetLastError(), message);
	}
	inline void Check(int socket_result, std::string_view message = "")
	{
		if (socket_result != SOCKET_ERROR) return;
		Throw(WSAGetLastError(), message);
	}

	// IP:Port address iterator (Use the 'GetAddress' functions)
	struct AddrInfo
	{
		// Wraps a call to 'getaddrinfo' and 'freeaddrinfo'
		// returning a collection of address infos
		struct AddrIter
		{
			ADDRINFOA* m_ptr;

			AddrIter()
				:m_ptr()
			{}

			ADDRINFOA* operator ->() const
			{
				return m_ptr;
			}
			ADDRINFOA& operator *() const
			{
				return *m_ptr;
			}
			AddrIter& operator ++()
			{
				m_ptr = m_ptr->ai_next;
				return *this;
			}
			ADDRINFOA** address_of()
			{
				return &m_ptr;
			}
			
			friend bool operator == (AddrIter lhs, AddrIter rhs)
			{
				return lhs.m_ptr == rhs.m_ptr;
			}
			friend bool operator != (AddrIter lhs, AddrIter rhs)
			{
				return lhs.m_ptr != rhs.m_ptr;
			}
		};

		AddrIter m_first;

		// Convert an IP and service (aka port) into a socket address.
		// 'ip' can be an IPv4 or IPv6 address.
		// 'service' can be a string representation of a port number or
		//  a service name like 'http', 'https', or something listed in
		//  %WINDIR%\system32\drivers\etc\services
		AddrInfo(std::string_view ip, std::string_view service, int addr_family = AF_UNSPEC, int socket_type = SOCK_STREAM, int proto = IPPROTO_TCP)
		{
			ADDRINFO hints = {};
			hints.ai_family = addr_family;
			hints.ai_socktype = socket_type;
			hints.ai_protocol = proto;
			auto r = ::getaddrinfo(std::string(ip).c_str(), std::string(service).c_str(), &hints, m_first.address_of());
			if (r != 0)
				Throw(WSAGetLastError(), std::format("Failed to resolve address: {}:{}", ip, service));
		}
		~AddrInfo()
		{
			::freeaddrinfo(m_first.m_ptr);
		}
		AddrIter begin() const
		{
			return m_first;
		}
		AddrIter end() const
		{
			return AddrIter();
		}
	};

	// Convert an IP and service (aka port) into a socket address
	// 'ip' can be an IPv4 or IPv6 address
	// 'service' can be a string representation of a port number or
	// a service name like 'http', 'https', or something listed in
	// %WINDIR%\system32\drivers\etc\services
	inline sockaddr_in GetAddress(std::string_view ip, std::string_view service)
	{
		for (auto& i : AddrInfo(ip, service))
		{
			switch (i.ai_family)
			{
				case AF_INET:
				case AF_INET6:
					return *reinterpret_cast<sockaddr_in const*>(i.ai_addr);
				default:
					break;
			}
		}
		throw std::runtime_error(std::format("Failed to resolve address: {}:{}", ip, service));
	}

	// Convert an ip and port to a socket address
	inline sockaddr_in GetAddress(std::string_view ip, uint16_t port)
	{
		return GetAddress(ip, std::to_string(port));
	}

	// Get the address bound to 'socket'. This can be used when 'connect' is called without
	// calling 'bind' to retrieve the local address assigned by the system.
	template <typename SockAddrType = sockaddr_in>
	inline SockAddrType GetSockName(SOCKET socket)
	{
		SockAddrType addr;
		int addr_size = sizeof(addr);
		Check(::getsockname(socket, reinterpret_cast<sockaddr*>(&addr), &addr_size));
		Check(addr_size == sizeof(addr));
		return addr;
	}

	// Return the IP address associated with a socket address
	inline std::string GetIPAddress(sockaddr_in const& addr)
	{
		char ip[INET_ADDRSTRLEN];
		return ::inet_ntop(AF_INET, &addr.sin_addr, ip, _countof(ip));
	}

	// Return the Port associated with a socket address
	inline uint16_t GetPort(sockaddr_in const& addr)
	{
		return ntohs(addr.sin_port);
	}

}
