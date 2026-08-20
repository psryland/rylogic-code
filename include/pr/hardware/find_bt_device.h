//**************************************************************
// FindBTDevices
//  Copyright (c) Rylogic 2015
//**************************************************************
// Scoped wrapper around the BluetoothFindFirstRadio/BluetoothFindNextRadio  API
// Usage:
//	for (pr::FindBTDevices f; !f.done(); f.next())
//	{
//		OutputDebugStringW(f.szName);
//	}

#pragma once

#include <string>
#include <windows.h>
#include <winerror.h>
#include <ws2bth.h>
#include <bthsdpdef.h>
#include <bluetoothapis.h>
#pragma comment(lib, "bthprops.lib")

namespace pr
{
	namespace impl::bth
	{
		// Throw an exception for a win32 error
		inline void ThrowWin32Exception(char const* msg, int err)
		{
			char desc[4096], err_msg[4096];
			auto result = HRESULT_FROM_WIN32(err);
			if (FormatMessageA(FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS, NULL, result, MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT), err_msg, DWORD(sizeof(err_msg)), NULL))
				sprintf_s(desc, "%s: %s", msg, err_msg);
			else
				sprintf_s(desc, "%s: 0x%80X", msg, result);

			throw std::exception(desc);
		}
	}

	// Helper for enumerating BT radios
	struct FindBTRadios :BLUETOOTH_FIND_RADIO_PARAMS
	{
	private:
		HBLUETOOTH_RADIO_FIND m_find;
		HANDLE m_radio;
		bool m_more;
	
	public:
		FindBTRadios()
			:BLUETOOTH_FIND_RADIO_PARAMS({ sizeof(BLUETOOTH_FIND_RADIO_PARAMS) })
			,m_find()
			,m_radio()
			,m_more(true)
		{
			// Find the first radio
			m_find = ::BluetoothFindFirstRadio(this, &m_radio);
			if (m_find == nullptr)
			{
				auto err = ::GetLastError();
				switch (err)
				{
					case ERROR_NO_MORE_ITEMS:
						m_more = false;
						break;
					default:
						impl::bth::ThrowWin32Exception("Error while enumerating bluetooth radio devices", err);
						break;
				}
			}
		}
		~FindBTRadios()
		{
			if (m_find != nullptr)
				::BluetoothFindRadioClose(m_find);
		}

		// True once enumeration is complete
		bool done() const
		{
			return !m_more;
		}

		// The handle of the current radio device being enumerated
		HANDLE handle() const
		{
			return m_radio;
		}

		// Move to the next device
		void next()
		{
			if (::BluetoothFindNextRadio(m_find, &m_radio))
				return;

			auto err = ::GetLastError();
			switch (err)
			{
				case ERROR_NO_MORE_ITEMS:
				case RPC_S_SERVER_UNAVAILABLE:
					m_more = false;
					break;
				default:
					impl::bth::ThrowWin32Exception("Error while enumerating bluetooth radio devices", err);
					break;
			}
		}
	
		// Return information about the current radio
		BLUETOOTH_RADIO_INFO info() const
		{
			BLUETOOTH_RADIO_INFO info = {sizeof(BLUETOOTH_RADIO_INFO)};
			BluetoothGetRadioInfo(handle(), &info);
			return info;
		}
	};

	// Helper for enumerating BT devices
	struct FindBTDevices :BLUETOOTH_DEVICE_INFO_STRUCT
	{
		struct Params :BLUETOOTH_DEVICE_SEARCH_PARAMS
		{
			Params() :BLUETOOTH_DEVICE_SEARCH_PARAMS()
			{
				dwSize = sizeof(BLUETOOTH_DEVICE_SEARCH_PARAMS);
				fReturnAuthenticated = TRUE;
				fReturnRemembered = TRUE;
				fReturnUnknown = TRUE;
				fReturnConnected = TRUE;

				// Use this to start a query for devices
				fIssueInquiry = FALSE;
				cTimeoutMultiplier = 0;

				// The specific radio to search, use nullptr for all
				hRadio = nullptr;
			}
		};

	private:
		Params m_search_params;
		HBLUETOOTH_DEVICE_FIND m_find;
		bool m_more;

	public:
		FindBTDevices(Params const& p = Params())
			:BLUETOOTH_DEVICE_INFO_STRUCT()
			,m_search_params(p)
			,m_find()
			,m_more(true)
		{
			dwSize = sizeof(BLUETOOTH_DEVICE_INFO_STRUCT);

			// Find the first device on this radio
			m_find = ::BluetoothFindFirstDevice(&m_search_params, static_cast<BLUETOOTH_DEVICE_INFO_STRUCT*>(this));
			if (m_find == nullptr)
			{
				auto err = ::GetLastError();
				switch (err)
				{
					case ERROR_INVALID_HANDLE:
					case ERROR_NO_MORE_ITEMS:
					case RPC_S_SERVER_UNAVAILABLE:
						m_more = false;
						break;
					default:
						impl::bth::ThrowWin32Exception("Failed to enumerate devices on bluetooth radio", err);
						break;
				}
			}
		}
		~FindBTDevices()
		{
			if (m_find != nullptr)
				::BluetoothFindDeviceClose(m_find);
		}

		// True once enumeration is complete
		bool done() const
		{
			return !m_more;
		}

		// Move to the next device
		void next()
		{
			if (::BluetoothFindNextDevice(m_find, this))
				return;

			auto err = ::GetLastError();
			switch (err)
			{
				case ERROR_NO_MORE_ITEMS:
					m_more = false;
					break;
				default:
					impl::bth::ThrowWin32Exception("Failed to enumerate devices on bluetooth radio", err);
					break;
			}
		}
			
		// Return information about the current radio
		BLUETOOTH_DEVICE_INFO_STRUCT info() const
		{
			BLUETOOTH_DEVICE_INFO_STRUCT info = {sizeof(BLUETOOTH_DEVICE_INFO_STRUCT)};
			BluetoothGetDeviceInfo(m_search_params.hRadio, &info);
			return info;
		}
	};
}

#if PR_UNITTESTS
#include <dbt.h>
#include <bthdef.h>
#include "pr/common/unittests.h"
#include "pr/filesys/filesys.h"
namespace pr::hardware
{
	PRUnitTest(FindBTRadiosTests, Quick)
	{
		for (FindBTRadios r; !r.done(); r.next())
		{
			OutputDebugStringW(L"Found BT Radio\n");
		}
	}
	PRUnitTest(FindBTDevicesTests, Quick)
	{
		for (FindBTDevices f; !f.done(); f.next())
		{
			OutputDebugStringW(f.szName);
			OutputDebugStringW(L"\n");
		}
	}
}
#endif
