#pragma once

#include <string>
#include <sstream>
#include <vector>
#include <algorithm>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <atomic>
#include <cassert>
#include <exception>

#include "pr/network/sockets.h"

#include <cguid.h>
#include <ws2bth.h>
#include <bthsdpdef.h>
#include <bluetoothapis.h>
#pragma comment(lib, "bthprops.lib")

namespace pr::network
{
	// Helper for querying the available bluetooth services
	struct BluetoothServices
	{
		// A query set is a header followed by a blob of memory
		std::vector<unsigned char> m_buf;
		HANDLE m_handle;
		bool m_more; // true if there are more services available

		// Use LUP_FLUSHCACHE to refresh BT devices
		BluetoothServices(ULONG flags = LUP_CONTAINERS)
			:m_buf()
			,m_handle()
			,m_more(true)
		{
			m_buf.resize(sizeof(WSAQUERYSETW));
			auto& qs = query_set();
			qs.dwSize = DWORD(m_buf.size());
			qs.dwNameSpace = NS_BTH;
			auto r = WSALookupServiceBeginW(&qs, flags, &m_handle);
			if (r == NO_ERROR && m_handle != nullptr)
				return;

			r = WSAGetLastError();
			if (r == WSANO_DATA) { m_more = false; return; }
			Check(r);
		}
		~BluetoothServices()
		{
			if (m_handle == nullptr) return;
			WSALookupServiceEnd(m_handle);
		}

		// Get information about next bluetooth device
		bool next(ULONG flags = LUP_RETURN_NAME | LUP_RETURN_ADDR |LUP_RETURN_TYPE)
		{
			for (;m_more;)
			{
				auto& qs = query_set();
				qs.dwSize = DWORD(m_buf.size());
				qs.dwNameSpace = NS_BTH;
				DWORD size = qs.dwSize;
					
				auto r = WSALookupServiceNextW(m_handle, flags, &size, &qs);
				if (r == NO_ERROR)
					break;

				r = WSAGetLastError();
				if (r == WSA_E_NO_MORE) { m_more = false; break; }
				if (r == WSAEFAULT) { m_buf.resize(size); continue; }
				Check(r);
			}
			return m_more;
		}

		// Find the next service associated with the given device
		bool next(wchar_t const* device_name, ULONG flags = LUP_RETURN_NAME | LUP_RETURN_ADDR |LUP_RETURN_TYPE)
		{
			for (;next(flags);)
			{
				// Compare the name to see if this is the device we are looking for.
				auto const& qs = query_set();
				if (qs.lpszServiceInstanceName != nullptr && _wcsicmp(qs.lpszServiceInstanceName, device_name) == 0)
					return true;
			}
			return false;
		}
		WSAQUERYSETW const& query_set() const { return *reinterpret_cast<WSAQUERYSETW const*>(m_buf.data()); }
		WSAQUERYSETW& query_set()             { return *reinterpret_cast<WSAQUERYSETW*>(m_buf.data()); }
	};

	// A specific bluetooth device
	struct BluetoothDeviceInfo :BLUETOOTH_DEVICE_INFO_STRUCT
	{
		BluetoothDeviceInfo(BLUETOOTH_DEVICE_INFO_STRUCT const& info)
			:BLUETOOTH_DEVICE_INFO_STRUCT(info)
		{}
	};

	// Wrapper for displaying a 'choose bluetooth device' GUI
	struct BluetoothDeviceUI
	{
		BLUETOOTH_SELECT_DEVICE_PARAMS m_params;
		bool m_valid;

		static BLUETOOTH_SELECT_DEVICE_PARAMS DefaultParams()
		{
			auto params = BLUETOOTH_SELECT_DEVICE_PARAMS{sizeof(BLUETOOTH_SELECT_DEVICE_PARAMS)};
			params.fShowRemembered = TRUE;
			params.fShowUnknown = TRUE;
			params.fShowAuthenticated = TRUE;
			return params;
		}
		BluetoothDeviceUI(BLUETOOTH_SELECT_DEVICE_PARAMS params = DefaultParams())
			:m_params(params)
			,m_valid(false)
		{}
		~BluetoothDeviceUI()
		{
			if (!m_valid) return;
			::BluetoothSelectDevicesFree(&m_params);
		}
		int ShowDialog()
		{
			m_valid = ::BluetoothSelectDevices(&m_params) != 0;
			return m_valid ? IDOK : IDCANCEL;
		}
		BluetoothDeviceInfo device() const
		{
			return BluetoothDeviceInfo(m_params.pDevices[0]);
		}
	};

	// Launch the system control panel for bluetooth devices
	inline bool ShowBTControlPanel()
	{
		// Get the windows directory on the local system
		wchar_t windir[32767];
		::GetWindowsDirectoryW(windir, _countof(windir));

		// Create a working path using the alias 'sysnative'. This is needed because the control panel
		// must be launched as an x86 or x64 process regardless of whether this is called from a 32 or 64
		// bit process.
		std::wstring path = windir;
		path.append(L"\\sysnative");
	
		// Create the control panel process to display the bluetooth settings
		auto pi = PROCESS_INFORMATION{};
		auto si = STARTUPINFOW{sizeof(STARTUPINFOW)};
		std::wstring cmdline = path + L"\\control bthprops.cpl";
		return CreateProcessW(nullptr, &cmdline[0], nullptr, nullptr, FALSE, 0, nullptr, path.c_str(), &si, &pi) != 0;
	}

	// Convert a bluetooth device name to a bluetooth address by performing inquiry with remote name requests.
	// Note: this method can fail (return false) because remote name requests arrive after a device inquiry has completed.
	// Without a window to receive IN_RANGE notifications, we don't have a direct mechanism to determine when remote name
	// requests have completed.
	// Use the LUP_FLUSHCACHE flag to cause the lookup service to do a fresh lookup instead of pulling the information from device cache.
	inline bool DeviceNameToBluetoothAddr(wchar_t const* name, SOCKADDR_BTH& addr, int flags_ = 0)
	{
		addr = SOCKADDR_BTH{};

		// WSALookupService is used for both service search and device inquiry
		ULONG flags =
			LUP_CONTAINERS  | // LUP_CONTAINERS is the flag which signals that we're doing a device inquiry.
			LUP_RETURN_NAME | // Friendly device name (if available) will be returned in lpszServiceInstanceName
			LUP_RETURN_ALL  | 
			LUP_RETURN_ADDR | // BTH_ADDR will be returned in lpcsaBuffer member of WSAQUERYSET;
			flags_;

		// Lookup services
		BluetoothServices svc(flags);
		for (;svc.next(name);)
		{
			// Found a remote bluetooth device with matching name.
			// Get the address of the device and exit the lookup.
			auto const& qs = svc.query_set();
			addr = *reinterpret_cast<SOCKADDR_BTH*>(qs.lpcsaBuffer->RemoteAddr.lpSockaddr);
			return true;
		}

		return false;
	}

	// Get address from formatted address-string of the remote device
	inline bool DeviceAddrToBluetoothAddr(wchar_t const* addr_string, SOCKADDR_BTH& addr)
	{
		std::wstring addr_str = addr_string;
		addr_str.append(1, L'\0');
			
		int len = sizeof(addr);
		auto r = WSAStringToAddressW(&addr_str[0], AF_BTH, nullptr, (SOCKADDR*)&addr, &len);
		if (r != NO_ERROR)
			Check(WSAGetLastError());
	}

	// A bluetooth socket with server behaviour
	class BTServer :public ServerSocket
	{
		typedef std::vector<GUID> TServices;
		TServices m_services;

		// Create m_listen_socket to use for listening on
		virtual Socket CreateListenSocket(int) override
		{
			// Create the listen socket using the RFCOMM protocol
			auto socket = Socket(::socket(AF_BTH, SOCK_STREAM, BTHPROTO_RFCOMM));
			if (socket == INVALID_SOCKET)
				Check(WSAGetLastError());

			// Bind the local address to the socket
			SOCKADDR_BTH my_address = {};
			my_address.addressFamily = AF_BTH;
			my_address.port = BT_PORT_ANY;
			auto r = ::bind(socket, (sockaddr const*)&my_address, sizeof(my_address));
			if (r == SOCKET_ERROR)
				Check(WSAGetLastError());
			
			return socket;
		}

	public:

		explicit BTServer(Winsock const& winsock)
			:ServerSocket(winsock)
			,m_services()
		{}
		BTServer(BTServer const&) = delete;
		BTServer& operator =(BTServer const&) = delete;
		~BTServer()
		{
			for (;!m_services.empty();)
				WithdrawService(m_services.back());
		}

		// Bluetooth servers register services with the OS which clients query in order
		// to establish the port to connect to (since only ports 1-31 are valid in RFCOMM).
		// Services are typically registered just before the server 'accepts' incoming connections
		void PublishService(GUID const& guid, wchar_t const* service_name)
		{
			if (std::find(std::begin(m_services), std::end(m_services), guid) != std::end(m_services))
				throw std::exception("Bluetooth service already registered");

			SOCKADDR_BTH addr;
			addr.addressFamily  = AF_BTH;
			addr.btAddr         = 0;
			addr.serviceClassId = GUID_NULL;
			addr.port           = BT_PORT_ANY;

			CSADDR_INFO csa;
			csa.LocalAddr.iSockaddrLength = sizeof(SOCKADDR_BTH);
			csa.LocalAddr.lpSockaddr      = (SOCKADDR*)&addr;
			csa.iSocketType               = SOCK_STREAM;
			csa.iProtocol                 = BTHPROTO_RFCOMM;

			WSAQUERYSETW reg_info            = {sizeof(WSAQUERYSETW)};
			reg_info.lpszServiceInstanceName = const_cast<wchar_t*>(service_name);
			reg_info.lpServiceClassId        = const_cast<GUID*>(&guid);
			reg_info.lpVersion               = nullptr;
			reg_info.lpszComment             = nullptr;
			reg_info.dwNameSpace             = NS_BTH;
			reg_info.lpNSProviderId          = nullptr;
			reg_info.dwNumberOfCsAddrs       = 1;
			reg_info.lpcsaBuffer             = &csa;

			auto r = WSASetServiceW(&reg_info, RNRSERVICE_REGISTER, 0);
			if (r == SOCKET_ERROR)
				Check(WSAGetLastError());

			m_services.push_back(guid);
		}
		void WithdrawService(GUID const& guid)
		{
			WSAQUERYSETW reg_info = {sizeof(WSAQUERYSETW)};
			reg_info.lpServiceClassId = const_cast<GUID*>(&guid);
			auto r = WSASetServiceW(&reg_info, RNRSERVICE_DELETE, 0);
			if (r == SOCKET_ERROR)
				Check(WSAGetLastError());

			m_services.erase(std::remove(std::begin(m_services), std::end(m_services), guid), std::end(m_services));
		}
	};

	// A bluetooth socket with client behaviour
	class BTClient :public ClientSocket
	{
		BTClient(BTClient const&) = delete; // no copying
		BTClient& operator =(BTClient const&) = delete;

		// Connect 'm_socket' to 'addr'
		bool ConnectToHost(SOCKADDR_BTH const& addr, int timeout_ms)
		{
			// Connect to the remote socket
			auto r = ::connect(m_socket, (struct sockaddr *)&addr, sizeof(addr));
			if (r == SOCKET_ERROR)
				Check(WSAGetLastError());

			// Wait for the socket to say it's writable (meaning it's connected)
			fd_set set = {};
			FD_SET(m_socket, &set);
			auto timeout = network::TimeVal(timeout_ms);
			r = ::select(0, 0, &set, 0, timeout_ms == ~0 ? nullptr : &timeout);
			if (r == NO_ERROR)
				return false;
			if (r == SOCKET_ERROR)
				Check(WSAGetLastError());

			return true;
		}

	public:

		struct ServiceOrPort
		{
			GUID m_service;
			int m_port;
			ServiceOrPort(GUID const& service) :m_service(service), m_port(0) {}
			ServiceOrPort(int port) :m_service(GUID_NULL), m_port(port) {}
		};

		explicit BTClient(Winsock const& winsock)
			:ClientSocket(winsock)
		{}

		// (Re)create 'm_socket'
		// Typically applications can just call 'Connect', however some socket options
		// need to be set after creating the socket but before connecting, in this case
		// applications can call 'CreateSocket' first, then connect.
		void CreateSocket()
		{
			Close();

			// Create a bluetooth socket using the RFCOMM protocol
			m_socket = ::socket(AF_BTH, SOCK_STREAM, BTHPROTO_RFCOMM);
			if (m_socket == INVALID_SOCKET)
				Check(WSAGetLastError());
		}

		// Connect to a BT device
		bool Connect(BluetoothDeviceInfo const& device, ServiceOrPort service_or_port, int timeout_ms = ~0)
		{
			if (m_socket == INVALID_SOCKET)
				CreateSocket();

			SOCKADDR_BTH addr = {};
			addr.addressFamily = AF_BTH;
			addr.btAddr = device.Address.ullLong;
			addr.serviceClassId = service_or_port.m_service;
			addr.port = service_or_port.m_port;
			return ConnectToHost(addr, timeout_ms);
		}

		// Connect to a device by name
		bool Connect(wchar_t const* device_name, ServiceOrPort service_or_port, int timeout_ms = ~0)
		{
			if (m_socket == INVALID_SOCKET)
				CreateSocket();

			SOCKADDR_BTH addr = {};
			if (!network::DeviceNameToBluetoothAddr(device_name, addr))
				return false;

			addr.serviceClassId = service_or_port.m_service;
			addr.port = service_or_port.m_port;
			return ConnectToHost(addr, timeout_ms);
		}
	};
}