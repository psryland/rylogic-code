//******************************************
// Direct Input
//  Copyright (c) Rylogic Ltd 2010
//******************************************
#pragma once

#ifndef DIRECTINPUT_VERSION
#define DIRECTINPUT_VERSION 0x0800
#endif

#include <vector>
#include <dinput.h>
#include <cassert>
#include "pr/common/hresult.h"
#include "pr/common/d3dptr.h"

#ifndef PR_DBG_DINPUT
#define PR_DBG_DINPUT PR_DBG
#endif

namespace pr::dinput
{
	enum class EDeviceClass
	{
		All      = DI8DEVCLASS_ALL,
		Device   = DI8DEVCLASS_DEVICE,
		Mouse    = DI8DEVCLASS_POINTER,
		Keyboard = DI8DEVCLASS_KEYBOARD,
		Joystick = DI8DEVCLASS_GAMECTRL
	};
	enum class EDeviceType
	{
		Device        = DI8DEVTYPE_DEVICE,
		Mouse         = DI8DEVTYPE_MOUSE,
		Keyboard      = DI8DEVTYPE_KEYBOARD,
		Joystick      = DI8DEVTYPE_JOYSTICK,
		GamePad       = DI8DEVTYPE_GAMEPAD,
		Driving       = DI8DEVTYPE_DRIVING,
		Flight        = DI8DEVTYPE_FLIGHT,
		FirstPerson   = DI8DEVTYPE_1STPERSON,
		DeviceControl = DI8DEVTYPE_DEVICECTRL,
		ScreenPointer = DI8DEVTYPE_SCREENPOINTER,
		Remote        = DI8DEVTYPE_REMOTE,
		Supplemental  = DI8DEVTYPE_SUPPLEMENTAL,
	};
	enum class EEnumDevicesFlags
	{
		// Enumerate Devices flags
		AllDevices = DIEDFL_ALLDEVICES,
		AttachedOnly = DIEDFL_ATTACHEDONLY,
		ForceFeedback = DIEDFL_FORCEFEEDBACK,
		IncludeAliases = DIEDFL_INCLUDEALIASES,
		IncludeHidden = DIEDFL_INCLUDEHIDDEN,
		IncludePhantoms = DIEDFL_INCLUDEPHANTOMS,
		_flags_enum = 0,
	};
	enum class EEnumObjectsFlags
	{
		// Flags that specify the types of object to be enumerated. 
		// Each of the following values restricts the enumeration to objects of the described type

		// All objects.
		All = DIDFT_ALL,

		// A relative axis.
		RelAxis = DIDFT_RELAXIS,

		// An absolute axis
		AbsAxis = DIDFT_ABSAXIS,

		// An axis, either absolute or relative.
		Axis = DIDFT_AXIS,

		// A push button. A push button is reported as down when the user presses it, and as up when the user releases it.
		PushButton = DIDFT_PSHBUTTON,

		// A toggle button. A toggle button is reported as down when the user presses itand remains so until the user presses the button a second time.
		ToggleButton = DIDFT_TGLBUTTON,

		// A push button or a toggle button.
		Button = DIDFT_BUTTON,

		// A point-of-view controller.
		PoV = DIDFT_POV,

		// A HID link collection. HID link collections do not generate data of their own.
		Collection = DIDFT_COLLECTION,

		// An object that does not generate data.
		NoData = DIDFT_NODATA,

		// An object that belongs to HID link collection number n.
		EnumCollection0 = DIDFT_ENUMCOLLECTION(0),
		EnumCollection1 = DIDFT_ENUMCOLLECTION(1),// etc

		// An object that does not belong to any HID link collection. In other words, an object for which the wCollectionNumber member of the DIDEVICEOBJECTINSTANCE structure is 0.
		NoCollection = DIDFT_NOCOLLECTION,

		// An object that contains a force-feedback actuator. In other words, forces can be applied to this object.
		FFActuator = DIDFT_FFACTUATOR,
		
		// An object that can be used to trigger force-feedback effects.
		FFEffectTrigger = DIDFT_FFEFFECTTRIGGER,

		// An object that supports output. For details, see Remarks in IDirectInputDevice8::SendDeviceData.
		Output = DIDFT_OUTPUT,

		// An object of a type defined by the manufacturer.
		VendorDefined = DIDFT_VENDORDEFINED,

		// Controls identified by a Human Interface Device(HID) usage alias. This flag applies only to HID-compliant USB devices.
		Alias = DIDFT_ALIAS,

		_flags_enum = 0,
	};
	enum class EReadDataFlags
	{
		Read = 0,
		Peek = DIGDD_PEEK,
	};
	enum class EDataFormatFlags
	{
		AbsAxis = DIDF_ABSAXIS,
		RelAxis = DIDF_RELAXIS,
	};

	static constexpr int BufferedBlockReadSize = 64;

	// An input device as reported from dx
	struct DeviceInstance
	{
		EDeviceType m_device_type;
		GUID        m_instance_guid;
		GUID        m_product_guid;
		std::string m_instance_name;
		std::string m_product_name;

		DeviceInstance()
			:m_device_type()
			,m_instance_guid(GUID_NULL)
			,m_product_guid(GUID_NULL)
			,m_instance_name()
			,m_product_name()
		{}
		bool valid() const
		{
			return IsEqualGUID(m_instance_guid, GUID_NULL) == 0;
		}
	};

	// Return a pointer to the direct input interface
	inline D3DPtr<IDirectInput8> GetDInput(HINSTANCE app_inst)
	{
		D3DPtr<IDirectInput8> ptr;
		Check(::DirectInput8Create(app_inst, DIRECTINPUT_VERSION, IID_IDirectInput8, (void**)ptr.address_of(), 0));
		return ptr;
	}

	// When constructed, enumerates all devices on the system
	template <typename Cont = std::vector<DeviceInstance>>
	struct DeviceEnum
	{
		Cont m_devices;

		DeviceEnum(IDirectInput8* dinput, EDeviceClass device_class, EEnumDevicesFlags device_flags, Cont const& cont = Cont())
			:m_devices(cont)
		{
			dinput->EnumDevices(static_cast<DWORD>(device_class), EnumDevicesCB, this, static_cast<DWORD>(device_flags));
		}
		static BOOL CALLBACK EnumDevicesCB(LPCDIDEVICEINSTANCE lpddi, LPVOID ctx)
		{
			auto& me = *static_cast<DeviceEnum*>(ctx);

			DeviceInstance inst;
			inst.m_device_type = static_cast<EDeviceType>(lpddi->dwDevType);
			inst.m_instance_guid = lpddi->guidInstance;
			inst.m_product_guid = lpddi->guidProduct;
			inst.m_instance_name = To<std::string>(lpddi->tszInstanceName);
			inst.m_product_name = To<std::string>(lpddi->tszProductName);
			me.m_devices.push_back(inst);
			return (me.m_devices.size() != me.m_devices.capacity()) ? DIENUM_CONTINUE : DIENUM_STOP;
		}
	};

	// A type that can be used with DeviceEnum to select a single device
	struct SelectDevice
	{
		DeviceInstance m_instance;
		std::string    m_product_name;
		GUID           m_product_guid;
		bool           m_found;

		SelectDevice()
			:m_instance()
			,m_product_name()
			,m_product_guid()
			,m_found(false)
		{}
		SelectDevice(std::string const& product_name, GUID product_guid)
			:m_product_name(product_name)
			,m_product_guid(product_guid)
			,m_found(false)
		{}
		size_t size() const
		{
			return m_found;
		}
		size_t capacity() const
		{
			return 1U;
		}
		void push_back(DeviceInstance const& inst)
		{
			if (!m_product_name.empty() && m_product_name != inst.m_product_name) return;
			if (m_product_guid != GUID_NULL && m_product_guid != inst.m_product_guid) return;
			m_instance = inst;
			m_found = true;
		}
	};

	// Find an instance of a device
	inline DeviceInstance FindDeviceInstance(IDirectInput8* dinput, std::string product_name, GUID product_guid, EDeviceClass device_class, EEnumDevicesFlags device_flags = EEnumDevicesFlags::AllDevices)
	{
		DeviceEnum<SelectDevice> em(dinput, device_class, device_flags, SelectDevice(product_name, product_guid));
		return em.m_devices.m_instance;
	}
	inline DeviceInstance FindDeviceInstance(IDirectInput8* dinput, std::string product_name, EDeviceClass device_class, EEnumDevicesFlags device_flags = EEnumDevicesFlags::AllDevices)
	{
		return FindDeviceInstance(dinput, product_name, GUID_NULL, device_class, device_flags);
	}
	inline DeviceInstance FindDeviceInstance(IDirectInput8* dinput, GUID product_guid, EDeviceClass device_class, EEnumDevicesFlags device_flags = EEnumDevicesFlags::AllDevices)
	{
		return FindDeviceInstance(dinput, "", product_guid, device_class, device_flags);
	}
	inline DeviceInstance FindDeviceInstance(IDirectInput8* dinput, EDeviceClass device_class, EEnumDevicesFlags device_flags = EEnumDevicesFlags::AllDevices)
	{
		return FindDeviceInstance(dinput, "", GUID_NULL, device_class, device_flags);
	}

	// When constructed, enumerates all objects on a device, e.g. the buttons on a joystick etc
	template <typename FmtCont = std::vector<DIOBJECTDATAFORMAT>, typename InstCont = std::vector<DIDEVICEOBJECTINSTANCE>>
	struct DeviceObjectEnum
	{
		FmtCont  m_data_format;
		InstCont m_obj_inst;

		DeviceObjectEnum(IDirectInputDevice8* device, EEnumObjectsFlags flags = EEnumObjectsFlags::All, FmtCont const& fmt_cont = FmtCont(), InstCont const& inst_cont = InstCont())
			:m_data_format(fmt_cont)
			,m_obj_inst(inst_cont)
		{
			Check(device->EnumObjects(EnumDeviceObjectsCB, this, static_cast<DWORD>(flags)));
		}
		static BOOL CALLBACK EnumDeviceObjectsCB(DIDEVICEOBJECTINSTANCE const* lpddoi, LPVOID ctx)
		{
			auto& me = *static_cast<DeviceObjectEnum*>(ctx);

			DIOBJECTDATAFORMAT data_fmt;
			if (false) {}
			else if (IsEqualGUID(lpddoi->guidType, GUID_XAxis )) { PR_INFO(PR_DBG_DINPUT, "GUID_XAxis "); data_fmt.pguid = &GUID_XAxis; }
			else if (IsEqualGUID(lpddoi->guidType, GUID_YAxis )) { PR_INFO(PR_DBG_DINPUT, "GUID_YAxis "); data_fmt.pguid = &GUID_YAxis; }
			else if (IsEqualGUID(lpddoi->guidType, GUID_ZAxis )) { PR_INFO(PR_DBG_DINPUT, "GUID_ZAxis "); data_fmt.pguid = &GUID_ZAxis; }
			else if (IsEqualGUID(lpddoi->guidType, GUID_RxAxis)) { PR_INFO(PR_DBG_DINPUT, "GUID_RxAxis"); data_fmt.pguid = &GUID_RxAxis; }
			else if (IsEqualGUID(lpddoi->guidType, GUID_RyAxis)) { PR_INFO(PR_DBG_DINPUT, "GUID_RyAxis"); data_fmt.pguid = &GUID_RyAxis; }
			else if (IsEqualGUID(lpddoi->guidType, GUID_RzAxis)) { PR_INFO(PR_DBG_DINPUT, "GUID_RzAxis"); data_fmt.pguid = &GUID_RzAxis; }
			else if (IsEqualGUID(lpddoi->guidType, GUID_Slider)) { PR_INFO(PR_DBG_DINPUT, "GUID_Slider"); data_fmt.pguid = &GUID_Slider; }
			else if (IsEqualGUID(lpddoi->guidType, GUID_Button)) { PR_INFO(PR_DBG_DINPUT, "GUID_Button"); data_fmt.pguid = &GUID_Button; }
			else if (IsEqualGUID(lpddoi->guidType, GUID_Key   )) { PR_INFO(PR_DBG_DINPUT, "GUID_Key   "); data_fmt.pguid = &GUID_Key; }
			else if (IsEqualGUID(lpddoi->guidType, GUID_POV   )) { PR_INFO(PR_DBG_DINPUT, "GUID_POV   "); data_fmt.pguid = &GUID_POV; }
			else { PR_INFO(PR_DBG_DINPUT, "Unknown device data format type"); return TRUE; }
			
			// Each device object produces 4 bytes of data. This is a byte offset
			data_fmt.dwOfs = DWORD(me.m_data_format.size() * sizeof(long));
			data_fmt.dwType = lpddoi->dwType;
			data_fmt.dwFlags = lpddoi->dwFlags;
			me.m_data_format.push_back(data_fmt);
			me.m_obj_inst.push_back(*lpddoi);
			return TRUE;
		}
	};

	// Configuration for setting up a dinput device
	struct DeviceSettings
	{
		D3DPtr<IDirectInput8> m_dinput;       // The dinput interface pointer
		DeviceInstance        m_instance;     // The device instance to use
		HWND                  m_hwnd;         // The window that the device is associated with
		int                   m_buffer_size;  // Number of events to buffer
		bool                  m_buffered;     // Whether to use buffered data
		bool                  m_events;       // Whether to use events

		DeviceSettings()
			:m_dinput()
			,m_instance()
			,m_hwnd()
			,m_buffer_size()
			,m_buffered()
			,m_events()
		{}

		// Default settings for a device class
		DeviceSettings(HINSTANCE app_inst, HWND hwnd, EDeviceClass dev_class, int buf_size = 0, bool buffered = false, bool events = false)
			:m_dinput(dinput::GetDInput(app_inst))
			,m_instance(FindDeviceInstance(m_dinput.get(), dev_class))
			,m_hwnd(hwnd)
			,m_buffer_size(buf_size)
			,m_buffered(buffered)
			,m_events(events)
		{}
	};

	// Base class for a dinput device
	class Device
	{
	protected:

		DeviceSettings              m_settings;  // The device config
		D3DPtr<IDirectInputDevice8> m_device;    // The device

	public:

		// The event handle that signals when input is available
		HANDLE m_event;

		explicit Device(DeviceSettings const& settings)
			:m_settings(settings)
			,m_device()
			,m_event()
		{
			// Check the device instance guid is valid
			Check(settings.m_instance.valid(), "direct input device instance invalid");

			// Create the device
			Check(settings.m_dinput->CreateDevice(settings.m_instance.m_instance_guid, m_device.address_of(), 0));

			// Cooperate with windows
			Check(m_device->SetCooperativeLevel(m_settings.m_hwnd, DISCL_FOREGROUND | DISCL_NONEXCLUSIVE));

			// Setup buffered data
			if (m_settings.m_buffered)
			{
				DIPROPDWORD prop_data;
				prop_data.diph.dwSize = sizeof(DIPROPDWORD);
				prop_data.diph.dwHeaderSize = sizeof(DIPROPHEADER);
				prop_data.diph.dwObj = 0;
				prop_data.diph.dwHow = DIPH_DEVICE;
				prop_data.dwData = m_settings.m_buffer_size;
				Check(m_device->SetProperty(DIPROP_BUFFERSIZE, &prop_data.diph));
			}

			// Setup event notification
			if (m_settings.m_events)
			{
				m_event = CreateEvent(0, FALSE, FALSE, 0);
				Check(m_event != nullptr, "Failed to create a system event for dinput events");
				Check(m_device->SetEventNotification(m_event));
			}
		}
		Device(Device const&) = delete;
		void operator = (Device const&) = delete;
		virtual ~Device()
		{
			UnAcquire();
		}

		// Acquire the device. Returns true if the device was acquired, false if it was lost to another app.
		bool Acquire()
		{
			auto res = m_device->Acquire();
			if (res == DIERR_OTHERAPPHASPRIO) return false;
			Check(res);
			return true;
		}

		// Release the acquired device
		void UnAcquire()
		{
			Check(m_device->Unacquire());
		}

		// Flush the data from the buffer
		void FlushBuffer()
		{
			DWORD count = INFINITE;
			Check(m_device->GetDeviceData(sizeof(DIDEVICEOBJECTDATA), 0, &count, 0));
		}

	protected:

		// Read the state of the device
		// Returns DI_OK if the status was read successfully, false if another app has the device.
		HRESULT ReadDeviceState(void* buffer, DWORD buffer_size)
		{
			for (;;)
			{
				auto res = m_device->GetDeviceState(buffer_size, buffer);
				if (res == DI_OK) return res;
				if (res == DIERR_NOTACQUIRED || res == DIERR_INPUTLOST)
				{
					if (Acquire()) continue;
					return res;
				}
				Check(res);
			}
		}

		// Read up to 'count' data items from the device.
		// On return 'count' contains the number actually read.
		// 'flags' is one of 0 or DIGDD_PEEK
		// Returns "Success" if the status was read successfully, "InputLost" if another app has the device
		// or "DataPending" if the data isn't ready yet.
		HRESULT ReadDeviceData(DIDEVICEOBJECTDATA* buf, DWORD& count, EReadDataFlags flags = EReadDataFlags::Read)
		{
			for (;;)
			{
				auto res = m_device->GetDeviceData(sizeof(DIDEVICEOBJECTDATA), buf, &count, static_cast<DWORD>(flags));
				if (res == DI_OK) return res;
				if (res == DIERR_NOTACQUIRED) { if (Acquire()) continue; return res; }
				if (res == DIERR_INPUTLOST) { if (Acquire()) continue; return res; }
				if (res == DI_BUFFEROVERFLOW) return res; // This indicates some data was lost
				Check(res);
			}
		}
	};

	// Keyboard
	class Keyboard :public Device
	{
		enum { MaxkeyStates = 256 };
		uint8_t m_key_state[MaxkeyStates];

	public:

		struct KeyData
		{
			uint32_t m_key;       // Which key
			uint8_t  m_state;     // The state of the key
			uint32_t m_timestamp; // The time at which the key changed state in milliseconds
			bool KeyDown() const { return (m_state & 0x80) != 0; }
		};

		explicit Keyboard(DeviceSettings const& settings)
			:Device(settings)
			,m_key_state()
		{
			Check(m_device->SetDataFormat(&c_dfDIKeyboard));
		}

		// Non buffered data
		bool KeyDown(int key) const
		{
			return (m_key_state[key] & 0x80) == 0x80;
		}

		// Sample the state of the keyboard at this point in time
		// Returns DI_OK, DIERR_NOTACQUIRED, or DIERR_INPUTLOST
		HRESULT Sample()
		{
			memset(&m_key_state[0], 0, sizeof(m_key_state));
			return ReadDeviceState(&m_key_state, sizeof(m_key_state));
		}

		// Buffered data - Reads from the dinput buffered keyboard data into 'm_key_state'
		// This method can also be used to read key event data into 'events'.
		// If given, 'events' must point to a buffer of at least 'max_to_read' KeyData structs.
		// Returns the number of buffered events read.
		int ReadBuffer(int max_to_read = 1, KeyData* events = 0)
		{
			DIDEVICEOBJECTDATA buf[BufferedBlockReadSize];

			int read = 0;
			for (; read != max_to_read;)
			{
				// Read the buffer
				auto count = static_cast<DWORD>(std::min(max_to_read - read, BufferedBlockReadSize)); // 'count' is modified in ReadDeviceData
				if (Failed(ReadDeviceData(buf, count))) break;
				if (count == 0) break;
				read += static_cast<int>(count);

				// Copy the data into the key buffer
				for (auto i = 0; i != static_cast<int>(count); ++i)
				{
					auto& data = buf[i];
					assert(data.dwOfs < 256);

					// Copy into the key state buffer
					m_key_state[data.dwOfs] = static_cast<uint8_t>(data.dwData);
					if (events)
					{
						events->m_key = data.dwOfs;
						events->m_state = static_cast<uint8_t>(data.dwData);
						events->m_timestamp = static_cast<uint32_t>(data.dwTimeStamp);
						++events;
					}
				}
			}
			return read;
		}
	};

	// Mouse
	class Mouse :public Device
	{
		DIMOUSESTATE2 m_state[2];
		DIMOUSESTATE2* m_prev;
		DIMOUSESTATE2* m_curr;

	public:
		enum EButton { Left = 0, Right = 1, Middle = 2, LeftLeft = 3, RightRight = 4, NumberOf };
		struct MouseData
		{
			long m_x, m_y, m_z;
			uint8_t m_btn[8];
		};

		Mouse(DeviceSettings const& settings)
			:Device(settings)
			,m_state()
			,m_prev(&m_state[0])
			,m_curr(&m_state[1])
		{
			Check(m_device->SetDataFormat(&c_dfDIMouse2));
		}

		// Non buffered data
		bool btn(int i) const
		{
			assert(0 <= i && i < 8);
			return (m_curr->rgbButtons[i] & 0x80) == 0x80;
		}
		bool btn_down(int i) const
		{
			return  btn(i) && (m_prev->rgbButtons[i] & 0x80) == 0x00;
		}
		bool btn_up(int i) const
		{
			return !btn(i) && (m_prev->rgbButtons[i] & 0x80) == 0x80;
		}
		uint8_t btn_mask() const
		{
			uint8_t btns = 0;
			for (int i = 0; i != 8; ++i)
				btns |= static_cast<uint8_t>(int(btn(i)) << i);

			return btns;
		}

		// These are pixel distances since last sampled (Not absolute!)
		// Accumulate these by:
		//  if (btn_down() || btn_up) accum = zero;
		//  if (btn()) accum += dxyz();
		long dx() const
		{
			return m_curr->lX;
		}
		long dy() const
		{
			return m_curr->lY;
		}
		long dz() const
		{
			return m_curr->lZ;
		}
		v2 dxy() const
		{
			return v2(static_cast<float>(dx()), static_cast<float>(dy()));
		}
		v4 dxyz() const
		{
			return v4(static_cast<float>(dx()), static_cast<float>(dy()), static_cast<float>(dz()), 0);
		}
		long daxis(int i) const
		{
			assert(0 <= i && i < 3);
			return (&m_curr->lX)[i];
		}

		// Sample the state of the mouse at this point in time
		// Returns DI_OK, DIERR_NOTACQUIRED, or DIERR_INPUTLOST
		HRESULT Sample()
		{
			std::swap(m_prev, m_curr);
			memset(m_curr, 0, sizeof(*m_curr));
			return ReadDeviceState(m_curr, sizeof(*m_curr));
		}

		// Buffered data - Reads from the dinput buffered mouse data into 'm_curr'
		// This method can also be used to read mouse event data into 'events'.
		// If given, 'events' must point to a buffer of at least 'max_to_read' MouseData structs.
		// Returns the number of buffered events read.
		int ReadBuffer(int max_to_read = 1, MouseData* events = 0)
		{
			DIDEVICEOBJECTDATA buf;

			int read = 0;
			for (; read != max_to_read;)
			{
				// Read the buffer
				DWORD count = 1;
				if (Failed(ReadDeviceData(&buf, count))) break;
				if (count == 0) break;
				read += static_cast<int>(count);

				// Copy the data into the buffer
				std::swap(m_prev, m_curr);
				switch (buf.dwOfs)
				{
				case offsetof(DIMOUSESTATE, lX): m_curr->lX = buf.dwData; break;
				case offsetof(DIMOUSESTATE, lY): m_curr->lY = buf.dwData; break;
				case offsetof(DIMOUSESTATE, lZ): m_curr->lZ = buf.dwData; break;
				case DIMOFS_BUTTON0: m_curr->rgbButtons[0] = (BYTE)buf.dwData; break;
				case DIMOFS_BUTTON1: m_curr->rgbButtons[1] = (BYTE)buf.dwData; break;
				case DIMOFS_BUTTON2: m_curr->rgbButtons[2] = (BYTE)buf.dwData; break;
				case DIMOFS_BUTTON3: m_curr->rgbButtons[3] = (BYTE)buf.dwData; break;
				case DIMOFS_BUTTON4: m_curr->rgbButtons[4] = (BYTE)buf.dwData; break;
				case DIMOFS_BUTTON5: m_curr->rgbButtons[5] = (BYTE)buf.dwData; break;
				case DIMOFS_BUTTON6: m_curr->rgbButtons[6] = (BYTE)buf.dwData; break;
				case DIMOFS_BUTTON7: m_curr->rgbButtons[7] = (BYTE)buf.dwData; break;
				default: assert(false); break;
				}
				if (events)
				{
					events->m_x = m_curr->lX;
					events->m_y = m_curr->lY;
					events->m_z = m_curr->lZ;
					for (int i = 0; i != 8; ++i)
						events->m_btn[i] = m_curr->rgbButtons[i];

					++events;
				}
			}
			return read;
		}
	};

	// Joystick
	class Joystick :public Device
	{
		std::vector<long> m_state;

	public:

		struct JoyData
		{
			int  m_offset;
			long m_state;
		};

		Joystick(DeviceSettings const& settings)
			:Device(settings)
			,m_state()
		{
			// Enumerate the buttons, axes, etc for this device
			DeviceObjectEnum<> em(m_device.get());
			Check(!em.m_data_format.empty(), "Can't enumerate any buttons, axes, etc, for this joystick");

			// Construct a data format
			DIDATAFORMAT format = {};
			format.dwSize = sizeof(DIDATAFORMAT);
			format.dwObjSize = sizeof(DIOBJECTDATAFORMAT);
			format.dwFlags = static_cast<DWORD>(EDataFormatFlags::AbsAxis);
			format.dwDataSize = static_cast<DWORD>(em.m_data_format.size() * sizeof(long));
			format.dwNumObjs = static_cast<DWORD>(em.m_data_format.size());
			format.rgodf = &em.m_data_format[0];
			m_state.resize(em.m_data_format.size());
			Check(m_device->SetDataFormat(&format));
		}

		// Non buffered data
		// hmm, need a way to identity what each index position actually is...
		long axis(int i) const
		{
			assert(0 <= i && i < static_cast<int>(m_state.size()));
			return m_state[i];
		}
		bool btn(int i) const
		{
			assert(0 <= i && i < static_cast<int>(m_state.size()));
			return (m_state[i] & 0x80) == 0x80;
		}

		// Sample the state of the joystick at this point in time
		// Returns DI_OK, DIERR_NOTACQUIRED, or DIERR_INPUTLOST
		HRESULT Sample()
		{
			auto buffer_size = static_cast<DWORD>(m_state.size() * sizeof(long));
			memset(&m_state[0], 0, buffer_size);
			m_device->Poll();
			return ReadDeviceState(&m_state[0], buffer_size);
		}

		// Buffered data - Reads from the dinput buffered joystick data into 'm_state'
		// This method can also be used to read joystick event data into 'events'.
		// If given, 'events' must point to a buffer of at least 'max_to_read' JoyData structs.
		// Returns the number of buffered events read.
		int ReadBuffer(int max_to_read = 1, JoyData* events = 0)
		{
			DIDEVICEOBJECTDATA buf[BufferedBlockReadSize];

			int read = 0;
			for (; read != max_to_read;)
			{
				// Read the buffer
				auto count = static_cast<DWORD>(std::min(max_to_read - read, BufferedBlockReadSize)); // count is modified in ReadDeviceData
				if (Failed(ReadDeviceData(buf, count))) break;
				if (count == 0) break;
				read += static_cast<int>(count);

				// Copy the data into the state buffer
				for (int i = 0; i != static_cast<int>(count); ++i)
				{
					auto& data = buf[i];
					assert(data.dwOfs < m_state.size());

					// Copy into the state buffer
					m_state[data.dwOfs] = data.dwData;
					if (events)
					{
						events->m_offset = static_cast<int>(data.dwOfs);
						events->m_state = static_cast<long>(data.dwData);
						++events;
					}
				}
			}
			return read;
		}
	};

	// Xbox controller
	class XboxCtrller :public Joystick
	{
	public:

		enum EAxis { LeftX, LeftY, RightX, RightY };
		enum EBtn { A, B, X, Y, White, Black, StickBtnLeft, StickBtnRight, TrigLeft, TrigRight, Start, Back };

		XboxCtrller(DeviceSettings const& settings)
			:Joystick(settings)
		{}

		// Non buffered data
		void LStick(long& x, long& y) const
		{
			x = axis(LeftX);
			y = axis(LeftY);
		}
		void RStick(long& x, long& y) const
		{
			x = axis(RightX);
			y = axis(RightY);
		}
		uint32_t DPad() const
		{
			return 0;
		}

		// Helper for finding the xbox controller device
		static DeviceInstance Find(IDirectInput8* dinput)
		{
			return FindDeviceInstance(dinput, "Microsoft Xbox Controller", EDeviceClass::Joystick);
		}
	};
}