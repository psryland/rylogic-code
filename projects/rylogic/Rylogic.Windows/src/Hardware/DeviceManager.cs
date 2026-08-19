using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Diagnostics;
using System.Linq;
using System.Runtime.ExceptionServices;
using System.Runtime.InteropServices;
using System.Security;
using System.Security.AccessControl;
using System.Text;
using System.Threading;
using Microsoft.Win32;
using Microsoft.Win32.SafeHandles;
using Rylogic.Common;
using Rylogic.Extn;
using Rylogic.Gui.Native;
using Rylogic.Interop.Win32;
using Rylogic.Windows.Extn;

namespace Rylogic.Hardware
{
	/// <summary>Class for accessing hardware devices</summary>
	public static class DeviceManager
	{
		static DeviceManager()
		{
			using var ready = new ManualResetEventSlim();
			ExceptionDispatchInfo? startup_error = null;
			EventHandler<WndProcEventArgs> message_handler = HandleMessage;
			var filter = new Win32.DEV_BROADCAST_DEVICEINTERFACE
			{
				hdr = new Win32.DEV_BROADCAST_HDR
				{
					dbch_size = Marshal.SizeOf<Win32.DEV_BROADCAST_DEVICEINTERFACE>(),
					dbch_devicetype = Win32.EDeviceBroadcaseType.DeviceInterface,
				},
				class_guid = Guid.Empty,
				Name = string.Empty,
			};

			// Device broadcasts need a permanently pumped owning thread even when callers do not provide an application message loop.
			s_notify_thread = new Thread(() =>
			{
				try
				{
					using var notify_wnd = new DummyWindow("Rylogic.DeviceManager.NotifyWnd");
					notify_wnd.Message += message_handler;
					using var notification_handle = User32.RegisterDeviceNotification(
						notify_wnd.Handle,
						filter,
						Win32.EDeviceNotifyFlags.WindowHandle | Win32.EDeviceNotifyFlags.AllInterface_Classes);
					if (notification_handle.IsInvalid)
						throw new Win32Exception("RegisterDeviceNotification failed.");

					ready.Set();
					notify_wnd.Run();
				}
				catch (Exception ex)
				{
					if (ready.IsSet)
						throw;

					startup_error = ExceptionDispatchInfo.Capture(ex);
					ready.Set();
				}
			})
			{
				IsBackground = true,
				Name = "Rylogic device notifications",
			};
			s_notify_thread.Start();
			ready.Wait();
			startup_error?.Throw();
		}

		/// <summary>Translate native device-interface broadcasts into managed hardware notifications.</summary>
		private static void HandleMessage(object? sender, WndProcEventArgs args)
		{
			if (args.Message != Win32.WM_DEVICECHANGE)
				return;

			var event_type = (Win32.EDeviceChangedEventType)args.WParam.ToInt32();
			if (event_type != Win32.EDeviceChangedEventType.DeviceArrival &&
				event_type != Win32.EDeviceChangedEventType.DeviceRemoveComplete)
				return;

			var header = Marshal.PtrToStructure<Win32.DEV_BROADCAST_HDR>(args.LParam);
			if (header.dbch_devicetype != Win32.EDeviceBroadcaseType.DeviceInterface)
				return;

			var data = Marshal.PtrToStructure<Win32.DEV_BROADCAST_DEVICEINTERFACE>(args.LParam);
			var dev_class = DevClass.From(data.class_guid);
			var changed = new HardwareChangedEventArgs(dev_class, data.Name, event_type == Win32.EDeviceChangedEventType.DeviceArrival);
			HardwareChanged?.Invoke(null, changed);
		}

		/// <summary>The background thread owning the device-notification HWND and message pump.</summary>
		private static readonly Thread s_notify_thread;

		/// <summary>Enumerate all devices in the given device class</summary>
		public static IEnumerable<Device> EnumDevices(DevClass dev_class, ESetupDiGetClassDevsFlags flags = ESetupDiGetClassDevsFlags.Present)
		{
			var dev_class_handle = GetClassDevs(dev_class.Id, null, IntPtr.Zero, ESetupDiGetClassDevsFlags.Present);
			foreach (var did in EnumDeviceInfo(dev_class_handle))
				yield return new Device(dev_class_handle, did);
		}

		/// <summary>Raised on the device-notification thread when a device is enabled or disabled.</summary>
		public static event EventHandler<HardwareChangedEventArgs>? HardwareChanged;

		[DebuggerDisplay("{DevicePath,nq}")]
		public class HardwareChangedEventArgs :EventArgs
		{
			public HardwareChangedEventArgs(DevClass device, string device_path, bool arrived)
			{
				DevClass = device;
				DevicePath = device_path;
				Arrived = arrived;
			}

			/// <summary>Lazy loaded device info for the changed device (arrived only)</summary>
			public Device? Device
			{
				get
				{
					if (Removed)
						return null;

					// Lazy create the 'Device'
					if (m_device == null)
					{
						// Find the device info
						var dev_class_handle = GetClassDevs(DevClass.Id, null, IntPtr.Zero, ESetupDiGetClassDevsFlags.Present);
						foreach (var device in EnumDeviceInfo(dev_class_handle).Select(x => new Device(dev_class_handle, x)))
						{
							if (!device.HardwareIDs.Contains(DevicePath)) continue;
							m_device = device;
							break;
						}
					}
					return m_device;
				}
			}
			private Device? m_device;

			/// <summary>The type of device that was changed</summary>
			public DevClass DevClass { get; }

			/// <summary></summary>
			public string DevicePath { get; }

			/// <summary>The device has just become available</summary>
			public bool Arrived { get; }

			/// <summary>The device was removed</summary>
			public bool Removed => !Arrived;
		}

		/// <summary>Device information</summary>
		[DebuggerDisplay("{DeviceDescription,nq}")]
		public class Device
		{
			private readonly DevClassSafeHandle m_handle;
			private readonly DeviceInfoData m_did;
			internal Device(DevClassSafeHandle handle, DeviceInfoData did)
			{
				m_handle = handle;
				m_did = did;
			}

			/// <summary>Device description</summary>
			public string DeviceDescription => (string?)RegProp(EDeviceRegistryProperty.DEVICEDESC) ?? string.Empty;

			/// <summary>Device hardware ID</summary>
			public IList<string> HardwareIDs => (IList<string>?)RegProp(EDeviceRegistryProperty.HARDWAREID) ?? Array.Empty<string>();

			/// <summary>Return a registry property</summary>
			public object? RegProp(EDeviceRegistryProperty prop)
			{
				var (ty, data) = GetDeviceRegistryProperty(m_handle, m_did, prop);
				return RegistryKey_.Decode(ty, data);
			}
			public object? RegProp(string key_name)
			{
				using var key = OpenDevRegKey(m_handle, m_did, EScope.Global, 0, EDevKeyKind.Device, RegistryRights.ReadKey);
				return key.GetValue(key_name);
			}

			/// <summary>Return a device property as a binary blob</summary>
			public object? DevProp(DEVPROPKEY prop)
			{
				var (ty, data) = GetDeviceProperty(m_handle, m_did, prop);
				switch (ty)
				{
					case EDevPropType.EMPTY: return null;
					case EDevPropType.NULL: return null;
					case EDevPropType.SBYTE: return (sbyte)data[0];
					case EDevPropType.BYTE: return data[0];
					case EDevPropType.INT16: return BitConverter.ToInt16(data, 0);
					case EDevPropType.UINT16: return BitConverter.ToUInt16(data, 0);
					case EDevPropType.INT32: return BitConverter.ToInt32(data, 0);
					case EDevPropType.UINT32: return BitConverter.ToUInt32(data, 0);
					case EDevPropType.INT64: return BitConverter.ToInt64(data, 0);
					case EDevPropType.UINT64: return BitConverter.ToUInt64(data, 0);
					case EDevPropType.FLOAT: return BitConverter.ToSingle(data, 0);
					case EDevPropType.DOUBLE: return BitConverter.ToDouble(data, 0);
					//case EDevPropType.DECIMAL:
					case EDevPropType.GUID: return new Guid(data);
					case EDevPropType.CURRENCY: return BitConverter.ToInt64(data, 0);
					//case EDevPropType.DATE:
					//case EDevPropType.FILETIME:
					case EDevPropType.BOOLEAN: return BitConverter.ToBoolean(data, 0);
					case EDevPropType.STRING: return Encoding.Unicode.GetString(data, 0, data.Length).TrimEnd('\0');
					case EDevPropType.STRING_LIST: return Str_.DecodeStringArray(Encoding.Unicode.GetString(data, 0, data.Length));
					//case EDevPropType.SECURITY_DESCRIPTOR:
					//case EDevPropType.SECURITY_DESCRIPTOR_STRING:
					//case EDevPropType.DEVPROPKEY:
					//case EDevPropType.DEVPROPTYPE:
					case EDevPropType.BINARY: return data;
					case EDevPropType.ERROR: return BitConverter.ToUInt32(data, 0); // HRESULT
					case EDevPropType.NTSTATUS: return BitConverter.ToUInt32(data, 0); // NTSTATUS
					//case EDevPropType.STRING_INDIRECT:
					default:
					{
						throw new NotImplementedException($"Device property type {ty} has not been implemented");
					}
				}
			}

			/// <summary>Enable/Disable the device. Will throw an exception if the device is not Disable-able.</summary>
			public bool Enabled
			{
				get
				{
					throw new NotImplementedException();
				}
				set
				{
					// The size is just the size of the header, but we've flattened the structure.
					// The header comprises the first two fields, both integer.
					var parms = new PropertyChangeParameters
					{
						hdr = new SP_CLASSINSTALL_HEADER { Size = Marshal.SizeOf<SP_CLASSINSTALL_HEADER>(), DiFunction = (uint)EDiFunction.PropertyChange },
						Scope = EScope.Global,
						StateChange = value ? EStateChangeAction.Enable : EStateChangeAction.Disable,
					};

					if (!SetClassInstallParams(m_handle, m_did, parms))
						throw new Win32Exception();

					if (!CallClassInstaller(EDiFunction.PropertyChange, m_handle, m_did))
					{
						var err = Marshal.GetLastWin32Error();
						if (err == (int)ESetupApiError.NotDisableable)
							throw new Exception("Device can't be disabled (programmatically or in Device Manager).");
						if (err >= (int)ESetupApiError.NoAssociatedClass && err <= (int)ESetupApiError.OnlyValidateViaAuthenticode)
							throw new Win32Exception($"SetupAPI error: {(ESetupApiError)err}");
						throw new Win32Exception(err);
					}
				}
			}
		}

		#region Enumerations
		/// <summary></summary>
		[Flags]
		public enum ESetupDiGetClassDevsFlags
		{
			Default         = 1 << 0,
			Present         = 1 << 1,
			AllClasses      = 1 << 2,
			Profile         = 1 << 3,
			DeviceInterface = 1 << 4,
		}

		/// <summary></summary>
		[Flags]
		public enum EScope
		{
			/// <summary>DICS_FLAG_GLOBAL</summary>
			Global = 1,

			/// <summary>DICS_FLAG_CONFIGSPECIFIC</summary>
			ConfigSpecific = 2,

			/// <summary>DICS_FLAG_CONFIGGENERAL</summary>
			ConfigGeneral = 4
		}

		/// <summary>Registry key types</summary>
		[Flags]
		public enum EDevKeyKind :uint
		{
			/// <summary>DIREG_DEV - Open/Create/Delete device hardware key</summary>
			Device = 0x00000001,

			/// <summary>DIREG_DRV - Open/Create/Delete driver software key</summary>
			Driver = 0x00000002,

			/// <summary>DIREG_BOTH - Delete both driver and Device key</summary>
			Both = 0x00000004,
		}

		/// <summary></summary>
		public enum EDiFunction
		{
			SelectDevice                  = 1,
			InstallDevice                 = 2,
			AssignResources               = 3,
			Properties                    = 4,
			Remove                        = 5,
			FirstTimeSetup                = 6,
			FoundDevice                   = 7,
			SelectClassDrivers            = 8,
			ValidateClassDrivers          = 9,
			InstallClassDrivers           = (int)0xa,
			CalcDiskSpace                 = (int)0xb,
			DestroyPrivateData            = (int)0xc,
			ValidateDriver                = (int)0xd,
			Detect                        = (int)0xf,
			InstallWizard                 = (int)0x10,
			DestroyWizardData             = (int)0x11,
			PropertyChange                = (int)0x12,
			EnableClass                   = (int)0x13,
			DetectVerify                  = (int)0x14,
			InstallDeviceFiles            = (int)0x15,
			UnRemove                      = (int)0x16,
			SelectBestCompatDrv           = (int)0x17,
			AllowInstall                  = (int)0x18,
			RegisterDevice                = (int)0x19,
			NewDeviceWizardPreSelect      = (int)0x1a,
			NewDeviceWizardSelect         = (int)0x1b,
			NewDeviceWizardPreAnalyze     = (int)0x1c,
			NewDeviceWizardPostAnalyze    = (int)0x1d,
			NewDeviceWizardFinishInstall  = (int)0x1e,
			Unused1                       = (int)0x1f,
			InstallInterfaces             = (int)0x20,
			DetectCancel                  = (int)0x21,
			RegisterCoInstallers          = (int)0x22,
			AddPropertyPageAdvanced       = (int)0x23,
			AddPropertyPageBasic          = (int)0x24,
			Reserved1                     = (int)0x25,
			Troubleshooter                = (int)0x26,
			PowerMessageWake              = (int)0x27,
			AddRemotePropertyPageAdvanced = (int)0x28,
			UpdateDriverUI                = (int)0x29,
			Reserved2                     = (int)0x30
		}

		/// <summary></summary>
		public enum EStateChangeAction
		{
			Enable     = 1,
			Disable    = 2,
			PropChange = 3,
			Start      = 4,
			Stop       = 5
		}

		/// <summary>Device registry property codes (SPDRP)</summary>
		public enum EDeviceRegistryProperty :uint
		{
			/// <summary>DeviceDesc (R/W)</summary>
			DEVICEDESC = 0x00000000,

			/// <summary>HardwareID (R/W)</summary>
			HARDWAREID = 0x00000001,

			/// <summary>CompatibleIDs (R/W)</summary>
			COMPATIBLEIDS = 0x00000002,

			/// <summary></summary>
			UNUSED0 = 0x00000003,

			/// <summary>Service (R/W)</summary>
			SERVICE = 0x00000004,

			/// <summary></summary>
			UNUSED1 = 0x00000005,

			/// <summary></summary>
			UNUSED2 = 0x00000006,

			/// <summary>Class (R--tied to ClassGUID)</summary>
			CLASS = 0x00000007,

			/// <summary>ClassGUID (R/W)</summary>
			CLASSGUID = 0x00000008,

			/// <summary>Driver (R/W)</summary>
			DRIVER = 0x00000009,

			/// <summary>ConfigFlags (R/W)</summary>
			CONFIGFLAGS = 0x0000000A,

			/// <summary>Mfg (R/W)</summary>
			MFG = 0x0000000B,

			/// <summary>FriendlyName (R/W)</summary>
			FRIENDLYNAME = 0x0000000C,

			/// <summary>LocationInformation (R/W)</summary>
			LOCATION_INFORMATION = 0x0000000D,

			/// <summary>PhysicalDeviceObjectName (R)</summary>
			PHYSICAL_DEVICE_OBJECT_NAME = 0x0000000E,

			/// <summary>Capabilities (R)</summary>
			CAPABILITIES = 0x0000000F,

			/// <summary>UiNumber (R)</summary>
			UI_NUMBER = 0x00000010,

			/// <summary>UpperFilters (R/W)</summary>
			UPPERFILTERS = 0x00000011,

			/// <summary>LowerFilters (R/W)</summary>
			LOWERFILTERS = 0x00000012,

			/// <summary>BusTypeGUID (R)</summary>
			BUSTYPEGUID = 0x00000013,

			/// <summary>LegacyBusType (R)</summary>
			LEGACYBUSTYPE = 0x00000014,

			/// <summary>BusNumber (R)</summary>
			BUSNUMBER = 0x00000015,

			/// <summary>Enumerator Name (R)</summary>
			ENUMERATOR_NAME = 0x00000016,

			/// <summary>Security (R/W, binary form)</summary>
			SECURITY = 0x00000017,

			/// <summary>Security (W, SDS form)</summary>
			SECURITY_SDS = 0x00000018,

			/// <summary>Device Type (R/W)</summary>
			DEVTYPE = 0x00000019,

			/// <summary>Device is exclusive-access (R/W)</summary>
			EXCLUSIVE = 0x0000001A,

			/// <summary>Device Characteristics (R/W)</summary>
			CHARACTERISTICS = 0x0000001B,

			/// <summary>Device Address (R)</summary>
			ADDRESS = 0x0000001C,

			/// <summary>UiNumberDescFormat (R/W)</summary>
			UI_NUMBER_DESC_FORMAT = 0X0000001D,

			/// <summary>Device Power Data (R)</summary>
			DEVICE_POWER_DATA = 0x0000001E,

			/// <summary>Removal Policy (R)</summary>
			REMOVAL_POLICY = 0x0000001F,

			/// <summary>Hardware Removal Policy (R)</summary>
			REMOVAL_POLICY_HW_DEFAULT = 0x00000020,

			/// <summary>Removal Policy Override (RW)</summary>
			REMOVAL_POLICY_OVERRIDE = 0x00000021,

			/// <summary>Device Install State (R)</summary>
			INSTALL_STATE = 0x00000022,

			/// <summary>Device Location Paths (R)</summary>
			LOCATION_PATHS = 0x00000023,
		}

		/// <summary></summary>
		public enum ESetupApiError
		{
			NoAssociatedClass               = unchecked((int)0xe0000200),
			ClassMismatch                   = unchecked((int)0xe0000201),
			DuplicateFound                  = unchecked((int)0xe0000202),
			NoDriverSelected                = unchecked((int)0xe0000203),
			KeyDoesNotExist                 = unchecked((int)0xe0000204),
			InvalidDevinstName              = unchecked((int)0xe0000205),
			InvalidClass                    = unchecked((int)0xe0000206),
			DevinstAlreadyExists            = unchecked((int)0xe0000207),
			DevinfoNotRegistered            = unchecked((int)0xe0000208),
			InvalidRegProperty              = unchecked((int)0xe0000209),
			NoInf                           = unchecked((int)0xe000020a),
			NoSuchHDevinst                  = unchecked((int)0xe000020b),
			CantLoadClassIcon               = unchecked((int)0xe000020c),
			InvalidClassInstaller           = unchecked((int)0xe000020d),
			DiDoDefault                     = unchecked((int)0xe000020e),
			DiNoFileCopy                    = unchecked((int)0xe000020f),
			InvalidHwProfile                = unchecked((int)0xe0000210),
			NoDeviceSelected                = unchecked((int)0xe0000211),
			DevinfolistLocked               = unchecked((int)0xe0000212),
			DevinfodataLocked               = unchecked((int)0xe0000213),
			DiBadPath                       = unchecked((int)0xe0000214),
			NoClassInstallParams            = unchecked((int)0xe0000215),
			FileQueueLocked                 = unchecked((int)0xe0000216),
			BadServiceInstallSect           = unchecked((int)0xe0000217),
			NoClassDriverList               = unchecked((int)0xe0000218),
			NoAssociatedService             = unchecked((int)0xe0000219),
			NoDefaultDeviceInterface        = unchecked((int)0xe000021a),
			DeviceInterfaceActive           = unchecked((int)0xe000021b),
			DeviceInterfaceRemoved          = unchecked((int)0xe000021c),
			BadInterfaceInstallSect         = unchecked((int)0xe000021d),
			NoSuchInterfaceClass            = unchecked((int)0xe000021e),
			InvalidReferenceString          = unchecked((int)0xe000021f),
			InvalidMachineName              = unchecked((int)0xe0000220),
			RemoteCommFailure               = unchecked((int)0xe0000221),
			MachineUnavailable              = unchecked((int)0xe0000222),
			NoConfigMgrServices             = unchecked((int)0xe0000223),
			InvalidPropPageProvider         = unchecked((int)0xe0000224),
			NoSuchDeviceInterface           = unchecked((int)0xe0000225),
			DiPostProcessingRequired        = unchecked((int)0xe0000226),
			InvalidCOInstaller              = unchecked((int)0xe0000227),
			NoCompatDrivers                 = unchecked((int)0xe0000228),
			NoDeviceIcon                    = unchecked((int)0xe0000229),
			InvalidInfLogConfig             = unchecked((int)0xe000022a),
			DiDontInstall                   = unchecked((int)0xe000022b),
			InvalidFilterDriver             = unchecked((int)0xe000022c),
			NonWindowsNTDriver              = unchecked((int)0xe000022d),
			NonWindowsDriver                = unchecked((int)0xe000022e),
			NoCatalogForOemInf              = unchecked((int)0xe000022f),
			DevInstallQueueNonNative        = unchecked((int)0xe0000230),
			NotDisableable                  = unchecked((int)0xe0000231),
			CantRemoveDevinst               = unchecked((int)0xe0000232),
			InvalidTarget                   = unchecked((int)0xe0000233),
			DriverNonNative                 = unchecked((int)0xe0000234),
			InWow64                         = unchecked((int)0xe0000235),
			SetSystemRestorePoint           = unchecked((int)0xe0000236),
			IncorrectlyCopiedInf            = unchecked((int)0xe0000237),
			SceDisabled                     = unchecked((int)0xe0000238),
			UnknownException                = unchecked((int)0xe0000239),
			PnpRegistryError                = unchecked((int)0xe000023a),
			RemoteRequestUnsupported        = unchecked((int)0xe000023b),
			NotAnInstalledOemInf            = unchecked((int)0xe000023c),
			InfInUseByDevices               = unchecked((int)0xe000023d),
			DiFunctionObsolete              = unchecked((int)0xe000023e),
			NoAuthenticodeCatalog           = unchecked((int)0xe000023f),
			AuthenticodeDisallowed          = unchecked((int)0xe0000240),
			AuthenticodeTrustedPublisher    = unchecked((int)0xe0000241),
			AuthenticodeTrustNotEstablished = unchecked((int)0xe0000242),
			AuthenticodePublisherNotTrusted = unchecked((int)0xe0000243),
			SignatureOSAttributeMismatch    = unchecked((int)0xe0000244),
			OnlyValidateViaAuthenticode     = unchecked((int)0xe0000245)
		}

		/// <summary>Device property data types</summary>
		public enum EDevPropType
		{
			// Property data types.
			EMPTY                      = 0x00000000,          // nothing, no property data
			NULL                       = 0x00000001,          // null property data
			SBYTE                      = 0x00000002,          // 8-bit signed int (SBYTE)
			BYTE                       = 0x00000003,          // 8-bit unsigned int (BYTE)
			INT16                      = 0x00000004,          // 16-bit signed int (SHORT)
			UINT16                     = 0x00000005,          // 16-bit unsigned int (USHORT)
			INT32                      = 0x00000006,          // 32-bit signed int (LONG)
			UINT32                     = 0x00000007,          // 32-bit unsigned int (ULONG)
			INT64                      = 0x00000008,          // 64-bit signed int (LONG64)
			UINT64                     = 0x00000009,          // 64-bit unsigned int (ULONG64)
			FLOAT                      = 0x0000000A,          // 32-bit floating-point (FLOAT)
			DOUBLE                     = 0x0000000B,          // 64-bit floating-point (DOUBLE)
			DECIMAL                    = 0x0000000C,          // 128-bit data (DECIMAL)
			GUID                       = 0x0000000D,          // 128-bit unique identifier (GUID)
			CURRENCY                   = 0x0000000E,          // 64 bit signed int currency value (CURRENCY)
			DATE                       = 0x0000000F,          // date (DATE)
			FILETIME                   = 0x00000010,          // file time (FILETIME)
			BOOLEAN                    = 0x00000011,          // 8-bit boolean (DEVPROP_BOOLEAN)
			STRING                     = 0x00000012,          // null-terminated string
			STRING_LIST                = STRING|TYPEMOD_LIST, // multi-sz string list
			SECURITY_DESCRIPTOR        = 0x00000013,          // self-relative binary SECURITY_DESCRIPTOR
			SECURITY_DESCRIPTOR_STRING = 0x00000014,          // security descriptor string (SDDL format)
			DEVPROPKEY                 = 0x00000015,          // device property key (DEVPROPKEY)
			DEVPROPTYPE                = 0x00000016,          // device property type (DEVPROPTYPE)
			BINARY                     = BYTE|TYPEMOD_ARRAY,  // custom binary data
			ERROR                      = 0x00000017,          // 32-bit Win32 system error code
			NTSTATUS                   = 0x00000018,          // 32-bit NTSTATUS code
			STRING_INDIRECT            = 0x00000019,          // string resource (@[path\]<dllname>,-<strId>)

			// Property type modifiers.  Used to modify base DEVPROP_TYPE_ values, as
			// appropriate.  Not valid as standalone DEVPROPTYPE values.
			TYPEMOD_ARRAY = 0x00001000, // array of fixed-sized data elements
			TYPEMOD_LIST  = 0x00002000, // list of variable-sized data elements
		}
		#endregion

		#region Interop

		[StructLayout(LayoutKind.Sequential)]
		public struct DEVPROPKEY
		{
			public Guid fmtid;
			public uint pid;

			#region Known Keys

			// DEVPKEY_NAME
			// Common DEVPKEY used to retrieve the display name for an object.
			public static DEVPROPKEY NAME => new() { fmtid = new Guid(0xb725f130, 0x47ef, 0x101a, 0xa5, 0xf1, 0x02, 0x60, 0x8c, 0x9e, 0xeb, 0xac), pid =  10 };    // DEVPROP_TYPE_STRING

			// Device properties
			// These DEVPKEYs correspond to the SetupAPI SPDRP_XXX device properties.
			public static DEVPROPKEY Device_DeviceDesc => new() { fmtid = new Guid(0xa45c254e, 0xdf1c, 0x4efd, 0x80, 0x20, 0x67, 0xd1, 0x46, 0xa8, 0x50, 0xe0), pid =  2 };     // DEVPROP_TYPE_STRING
			public static DEVPROPKEY Device_HardwareIds => new() { fmtid = new Guid(0xa45c254e, 0xdf1c, 0x4efd, 0x80, 0x20, 0x67, 0xd1, 0x46, 0xa8, 0x50, 0xe0), pid =  3 };     // DEVPROP_TYPE_STRING_LIST
			public static DEVPROPKEY Device_CompatibleIds => new() { fmtid = new Guid(0xa45c254e, 0xdf1c, 0x4efd, 0x80, 0x20, 0x67, 0xd1, 0x46, 0xa8, 0x50, 0xe0), pid =  4 };     // DEVPROP_TYPE_STRING_LIST
			public static DEVPROPKEY Device_Service => new() { fmtid = new Guid(0xa45c254e, 0xdf1c, 0x4efd, 0x80, 0x20, 0x67, 0xd1, 0x46, 0xa8, 0x50, 0xe0), pid =  6 };     // DEVPROP_TYPE_STRING
			public static DEVPROPKEY Device_Class => new() { fmtid = new Guid(0xa45c254e, 0xdf1c, 0x4efd, 0x80, 0x20, 0x67, 0xd1, 0x46, 0xa8, 0x50, 0xe0), pid =  9 };     // DEVPROP_TYPE_STRING
			public static DEVPROPKEY Device_ClassGuid => new() { fmtid = new Guid(0xa45c254e, 0xdf1c, 0x4efd, 0x80, 0x20, 0x67, 0xd1, 0x46, 0xa8, 0x50, 0xe0), pid =  10 };    // DEVPROP_TYPE_GUID
			public static DEVPROPKEY Device_Driver => new() { fmtid = new Guid(0xa45c254e, 0xdf1c, 0x4efd, 0x80, 0x20, 0x67, 0xd1, 0x46, 0xa8, 0x50, 0xe0), pid =  11 };    // DEVPROP_TYPE_STRING
			public static DEVPROPKEY Device_ConfigFlags => new() { fmtid = new Guid(0xa45c254e, 0xdf1c, 0x4efd, 0x80, 0x20, 0x67, 0xd1, 0x46, 0xa8, 0x50, 0xe0), pid =  12 };    // DEVPROP_TYPE_UINT32
			public static DEVPROPKEY Device_Manufacturer => new() { fmtid = new Guid(0xa45c254e, 0xdf1c, 0x4efd, 0x80, 0x20, 0x67, 0xd1, 0x46, 0xa8, 0x50, 0xe0), pid =  13 };    // DEVPROP_TYPE_STRING
			public static DEVPROPKEY Device_FriendlyName => new() { fmtid = new Guid(0xa45c254e, 0xdf1c, 0x4efd, 0x80, 0x20, 0x67, 0xd1, 0x46, 0xa8, 0x50, 0xe0), pid =  14 };    // DEVPROP_TYPE_STRING
			public static DEVPROPKEY Device_LocationInfo => new() { fmtid = new Guid(0xa45c254e, 0xdf1c, 0x4efd, 0x80, 0x20, 0x67, 0xd1, 0x46, 0xa8, 0x50, 0xe0), pid =  15 };    // DEVPROP_TYPE_STRING
			public static DEVPROPKEY Device_PDOName => new() { fmtid = new Guid(0xa45c254e, 0xdf1c, 0x4efd, 0x80, 0x20, 0x67, 0xd1, 0x46, 0xa8, 0x50, 0xe0), pid =  16 };    // DEVPROP_TYPE_STRING
			public static DEVPROPKEY Device_Capabilities => new() { fmtid = new Guid(0xa45c254e, 0xdf1c, 0x4efd, 0x80, 0x20, 0x67, 0xd1, 0x46, 0xa8, 0x50, 0xe0), pid =  17 };    // DEVPROP_TYPE_UINT32
			public static DEVPROPKEY Device_UINumber => new() { fmtid = new Guid(0xa45c254e, 0xdf1c, 0x4efd, 0x80, 0x20, 0x67, 0xd1, 0x46, 0xa8, 0x50, 0xe0), pid =  18 };    // DEVPROP_TYPE_UINT32
			public static DEVPROPKEY Device_UpperFilters => new() { fmtid = new Guid(0xa45c254e, 0xdf1c, 0x4efd, 0x80, 0x20, 0x67, 0xd1, 0x46, 0xa8, 0x50, 0xe0), pid =  19 };    // DEVPROP_TYPE_STRING_LIST
			public static DEVPROPKEY Device_LowerFilters => new() { fmtid = new Guid(0xa45c254e, 0xdf1c, 0x4efd, 0x80, 0x20, 0x67, 0xd1, 0x46, 0xa8, 0x50, 0xe0), pid =  20 };    // DEVPROP_TYPE_STRING_LIST
			public static DEVPROPKEY Device_BusTypeGuid => new() { fmtid = new Guid(0xa45c254e, 0xdf1c, 0x4efd, 0x80, 0x20, 0x67, 0xd1, 0x46, 0xa8, 0x50, 0xe0), pid =  21 };    // DEVPROP_TYPE_GUID
			public static DEVPROPKEY Device_LegacyBusType => new() { fmtid = new Guid(0xa45c254e, 0xdf1c, 0x4efd, 0x80, 0x20, 0x67, 0xd1, 0x46, 0xa8, 0x50, 0xe0), pid =  22 };    // DEVPROP_TYPE_UINT32
			public static DEVPROPKEY Device_BusNumber => new() { fmtid = new Guid(0xa45c254e, 0xdf1c, 0x4efd, 0x80, 0x20, 0x67, 0xd1, 0x46, 0xa8, 0x50, 0xe0), pid =  23 };    // DEVPROP_TYPE_UINT32
			public static DEVPROPKEY Device_EnumeratorName => new() { fmtid = new Guid(0xa45c254e, 0xdf1c, 0x4efd, 0x80, 0x20, 0x67, 0xd1, 0x46, 0xa8, 0x50, 0xe0), pid =  24 };    // DEVPROP_TYPE_STRING
			public static DEVPROPKEY Device_Security => new() { fmtid = new Guid(0xa45c254e, 0xdf1c, 0x4efd, 0x80, 0x20, 0x67, 0xd1, 0x46, 0xa8, 0x50, 0xe0), pid =  25 };    // DEVPROP_TYPE_SECURITY_DESCRIPTOR
			public static DEVPROPKEY Device_SecuritySDS => new() { fmtid = new Guid(0xa45c254e, 0xdf1c, 0x4efd, 0x80, 0x20, 0x67, 0xd1, 0x46, 0xa8, 0x50, 0xe0), pid =  26 };    // DEVPROP_TYPE_SECURITY_DESCRIPTOR_STRING
			public static DEVPROPKEY Device_DevType => new() { fmtid = new Guid(0xa45c254e, 0xdf1c, 0x4efd, 0x80, 0x20, 0x67, 0xd1, 0x46, 0xa8, 0x50, 0xe0), pid =  27 };    // DEVPROP_TYPE_UINT32
			public static DEVPROPKEY Device_Exclusive => new() { fmtid = new Guid(0xa45c254e, 0xdf1c, 0x4efd, 0x80, 0x20, 0x67, 0xd1, 0x46, 0xa8, 0x50, 0xe0), pid =  28 };    // DEVPROP_TYPE_BOOLEAN
			public static DEVPROPKEY Device_Characteristics => new() { fmtid = new Guid(0xa45c254e, 0xdf1c, 0x4efd, 0x80, 0x20, 0x67, 0xd1, 0x46, 0xa8, 0x50, 0xe0), pid =  29 };    // DEVPROP_TYPE_UINT32
			public static DEVPROPKEY Device_Address => new() { fmtid = new Guid(0xa45c254e, 0xdf1c, 0x4efd, 0x80, 0x20, 0x67, 0xd1, 0x46, 0xa8, 0x50, 0xe0), pid =  30 };    // DEVPROP_TYPE_UINT32
			public static DEVPROPKEY Device_UINumberDescFormat => new() { fmtid = new Guid(0xa45c254e, 0xdf1c, 0x4efd, 0x80, 0x20, 0x67, 0xd1, 0x46, 0xa8, 0x50, 0xe0), pid =  31 };    // DEVPROP_TYPE_STRING
			public static DEVPROPKEY Device_PowerData => new() { fmtid = new Guid(0xa45c254e, 0xdf1c, 0x4efd, 0x80, 0x20, 0x67, 0xd1, 0x46, 0xa8, 0x50, 0xe0), pid =  32 };    // DEVPROP_TYPE_BINARY
			public static DEVPROPKEY Device_RemovalPolicy => new() { fmtid = new Guid(0xa45c254e, 0xdf1c, 0x4efd, 0x80, 0x20, 0x67, 0xd1, 0x46, 0xa8, 0x50, 0xe0), pid =  33 };    // DEVPROP_TYPE_UINT32
			public static DEVPROPKEY Device_RemovalPolicyDefault => new() { fmtid = new Guid(0xa45c254e, 0xdf1c, 0x4efd, 0x80, 0x20, 0x67, 0xd1, 0x46, 0xa8, 0x50, 0xe0), pid =  34 };    // DEVPROP_TYPE_UINT32
			public static DEVPROPKEY Device_RemovalPolicyOverride => new() { fmtid = new Guid(0xa45c254e, 0xdf1c, 0x4efd, 0x80, 0x20, 0x67, 0xd1, 0x46, 0xa8, 0x50, 0xe0), pid =  35 };    // DEVPROP_TYPE_UINT32
			public static DEVPROPKEY Device_InstallState => new() { fmtid = new Guid(0xa45c254e, 0xdf1c, 0x4efd, 0x80, 0x20, 0x67, 0xd1, 0x46, 0xa8, 0x50, 0xe0), pid =  36 };    // DEVPROP_TYPE_UINT32
			public static DEVPROPKEY Device_LocationPaths => new() { fmtid = new Guid(0xa45c254e, 0xdf1c, 0x4efd, 0x80, 0x20, 0x67, 0xd1, 0x46, 0xa8, 0x50, 0xe0), pid =  37 };    // DEVPROP_TYPE_STRING_LIST
			public static DEVPROPKEY Device_BaseContainerId => new() { fmtid = new Guid(0xa45c254e, 0xdf1c, 0x4efd, 0x80, 0x20, 0x67, 0xd1, 0x46, 0xa8, 0x50, 0xe0), pid =  38 };    // DEVPROP_TYPE_GUID

			// Device and Device Interface property
			// Common DEVPKEY used to retrieve the device instance id associated with devices and device interfaces.
			public static DEVPROPKEY Device_InstanceId => new() { fmtid = new Guid(0x78c34fc8, 0x104a, 0x4aca, 0x9e, 0xa4, 0x52, 0x4d, 0x52, 0x99, 0x6e, 0x57), pid =  256 };   // DEVPROP_TYPE_STRING

			// Device properties
			// These DEVPKEYs correspond to a device's status and problem code.
			public static DEVPROPKEY Device_DevNodeStatus => new() { fmtid = new Guid(0x4340a6c5, 0x93fa, 0x4706, 0x97, 0x2c, 0x7b, 0x64, 0x80, 0x08, 0xa5, 0xa7), pid =  2 };     // DEVPROP_TYPE_UINT32
			public static DEVPROPKEY Device_ProblemCode => new() { fmtid = new Guid(0x4340a6c5, 0x93fa, 0x4706, 0x97, 0x2c, 0x7b, 0x64, 0x80, 0x08, 0xa5, 0xa7), pid =  3 };     // DEVPROP_TYPE_UINT32

			// Device properties
			// These DEVPKEYs correspond to a device's relations.
			public static DEVPROPKEY Device_EjectionRelations => new() { fmtid = new Guid(0x4340a6c5, 0x93fa, 0x4706, 0x97, 0x2c, 0x7b, 0x64, 0x80, 0x08, 0xa5, 0xa7), pid =  4 };     // DEVPROP_TYPE_STRING_LIST
			public static DEVPROPKEY Device_RemovalRelations => new() { fmtid = new Guid(0x4340a6c5, 0x93fa, 0x4706, 0x97, 0x2c, 0x7b, 0x64, 0x80, 0x08, 0xa5, 0xa7), pid =  5 };     // DEVPROP_TYPE_STRING_LIST
			public static DEVPROPKEY Device_PowerRelations => new() { fmtid = new Guid(0x4340a6c5, 0x93fa, 0x4706, 0x97, 0x2c, 0x7b, 0x64, 0x80, 0x08, 0xa5, 0xa7), pid =  6 };     // DEVPROP_TYPE_STRING_LIST
			public static DEVPROPKEY Device_BusRelations => new() { fmtid = new Guid(0x4340a6c5, 0x93fa, 0x4706, 0x97, 0x2c, 0x7b, 0x64, 0x80, 0x08, 0xa5, 0xa7), pid =  7 };     // DEVPROP_TYPE_STRING_LIST
			public static DEVPROPKEY Device_Parent => new() { fmtid = new Guid(0x4340a6c5, 0x93fa, 0x4706, 0x97, 0x2c, 0x7b, 0x64, 0x80, 0x08, 0xa5, 0xa7), pid =  8 };     // DEVPROP_TYPE_STRING
			public static DEVPROPKEY Device_Children => new() { fmtid = new Guid(0x4340a6c5, 0x93fa, 0x4706, 0x97, 0x2c, 0x7b, 0x64, 0x80, 0x08, 0xa5, 0xa7), pid =  9 };     // DEVPROP_TYPE_STRING_LIST
			public static DEVPROPKEY Device_Siblings => new() { fmtid = new Guid(0x4340a6c5, 0x93fa, 0x4706, 0x97, 0x2c, 0x7b, 0x64, 0x80, 0x08, 0xa5, 0xa7), pid =  10 };    // DEVPROP_TYPE_STRING_LIST
			public static DEVPROPKEY Device_TransportRelations => new() { fmtid = new Guid(0x4340a6c5, 0x93fa, 0x4706, 0x97, 0x2c, 0x7b, 0x64, 0x80, 0x08, 0xa5, 0xa7), pid =  11 };    // DEVPROP_TYPE_STRING_LIST

			// Device property
			// This DEVPKEY corresponds to a the status code that resulted in a device to be in a problem state.
			public static DEVPROPKEY Device_ProblemStatus => new() { fmtid = new Guid(0x4340a6c5, 0x93fa, 0x4706, 0x97, 0x2c, 0x7b, 0x64, 0x80, 0x08, 0xa5, 0xa7), pid =  12 };     // DEVPROP_TYPE_NTSTATUS

			// Device properties
			// These DEVPKEYs are set for the corresponding types of root-enumerated devices.
			public static DEVPROPKEY Device_Reported => new() { fmtid = new Guid(0x80497100, 0x8c73, 0x48b9, 0xaa, 0xd9, 0xce, 0x38, 0x7e, 0x19, 0xc5, 0x6e), pid =  2 };  // DEVPROP_TYPE_BOOLEAN
			public static DEVPROPKEY Device_Legacy => new() { fmtid = new Guid(0x80497100, 0x8c73, 0x48b9, 0xaa, 0xd9, 0xce, 0x38, 0x7e, 0x19, 0xc5, 0x6e), pid =  3 };  // DEVPROP_TYPE_BOOLEAN

			// Device Container Id
			public static DEVPROPKEY Device_ContainerId => new() { fmtid = new Guid(0x8c7ed206, 0x3f8a, 0x4827, 0xb3, 0xab, 0xae, 0x9e, 0x1f, 0xae, 0xfc, 0x6c), pid =  2 };     // DEVPROP_TYPE_GUID
			public static DEVPROPKEY Device_InLocalMachineContainer => new() { fmtid = new Guid(0x8c7ed206, 0x3f8a, 0x4827, 0xb3, 0xab, 0xae, 0x9e, 0x1f, 0xae, 0xfc, 0x6c), pid =  4 };     // DEVPROP_TYPE_BOOLEAN

			// Device property
			// This DEVPKEY correspond to a device's model.
			public static DEVPROPKEY Device_Model => new() { fmtid = new Guid(0x78c34fc8, 0x104a, 0x4aca, 0x9e, 0xa4, 0x52, 0x4d, 0x52, 0x99, 0x6e, 0x57), pid =  39 };    // DEVPROP_TYPE_STRING

			// Device Experience related Keys
			public static DEVPROPKEY Device_ModelId => new() { fmtid = new Guid(0x80d81ea6, 0x7473, 0x4b0c, 0x82, 0x16, 0xef, 0xc1, 0x1a, 0x2c, 0x4c, 0x8b), pid =  2 }; // DEVPROP_TYPE_GUID
			public static DEVPROPKEY Device_FriendlyNameAttributes => new() { fmtid = new Guid(0x80d81ea6, 0x7473, 0x4b0c, 0x82, 0x16, 0xef, 0xc1, 0x1a, 0x2c, 0x4c, 0x8b), pid =  3 }; // DEVPROP_TYPE_UINT32
			public static DEVPROPKEY Device_ManufacturerAttributes => new() { fmtid = new Guid(0x80d81ea6, 0x7473, 0x4b0c, 0x82, 0x16, 0xef, 0xc1, 0x1a, 0x2c, 0x4c, 0x8b), pid =  4 }; // DEVPROP_TYPE_UINT32
			public static DEVPROPKEY Device_PresenceNotForDevice => new() { fmtid = new Guid(0x80d81ea6, 0x7473, 0x4b0c, 0x82, 0x16, 0xef, 0xc1, 0x1a, 0x2c, 0x4c, 0x8b), pid =  5 }; // DEVPROP_TYPE_BOOLEAN
			public static DEVPROPKEY Device_SignalStrength => new() { fmtid = new Guid(0x80d81ea6, 0x7473, 0x4b0c, 0x82, 0x16, 0xef, 0xc1, 0x1a, 0x2c, 0x4c, 0x8b), pid =  6 }; // DEVPROP_TYPE_INT32
			public static DEVPROPKEY Device_IsAssociateableByUserAction => new() { fmtid = new Guid(0x80d81ea6, 0x7473, 0x4b0c, 0x82, 0x16, 0xef, 0xc1, 0x1a, 0x2c, 0x4c, 0x8b), pid =  7 }; // DEVPROP_TYPE_BOOLEAN
			public static DEVPROPKEY Device_ShowInUninstallUI => new() { fmtid = new Guid(0x80d81ea6, 0x7473, 0x4b0c, 0x82, 0x16, 0xef, 0xc1, 0x1a, 0x2c, 0x4c, 0x8b), pid =  8 }; // DEVPROP_TYPE_BOOLEAN

			// Other Device properties
			public static DEVPROPKEY Device_Numa_Proximity_Domain => new() { fmtid = new Guid(0x540b947e, 0x8b40, 0x45bc, 0xa8, 0xa2, 0x6a, 0x0b, 0x89, 0x4c, 0xbd, 0xa2), pid =  1 };    // DEVPROP_TYPE_UINT32
			public static DEVPROPKEY Device_DHP_Rebalance_Policy => new() { fmtid = new Guid(0x540b947e, 0x8b40, 0x45bc, 0xa8, 0xa2, 0x6a, 0x0b, 0x89, 0x4c, 0xbd, 0xa2), pid =  2 };    // DEVPROP_TYPE_UINT32
			public static DEVPROPKEY Device_Numa_Node => new() { fmtid = new Guid(0x540b947e, 0x8b40, 0x45bc, 0xa8, 0xa2, 0x6a, 0x0b, 0x89, 0x4c, 0xbd, 0xa2), pid =  3 };    // DEVPROP_TYPE_UINT32
			public static DEVPROPKEY Device_BusReportedDeviceDesc => new() { fmtid = new Guid(0x540b947e, 0x8b40, 0x45bc, 0xa8, 0xa2, 0x6a, 0x0b, 0x89, 0x4c, 0xbd, 0xa2), pid =  4 };    // DEVPROP_TYPE_STRING
			public static DEVPROPKEY Device_IsPresent => new() { fmtid = new Guid(0x540b947e, 0x8b40, 0x45bc, 0xa8, 0xa2, 0x6a, 0x0b, 0x89, 0x4c, 0xbd, 0xa2), pid =  5 };    // DEVPROP_TYPE_BOOL
			public static DEVPROPKEY Device_HasProblem => new() { fmtid = new Guid(0x540b947e, 0x8b40, 0x45bc, 0xa8, 0xa2, 0x6a, 0x0b, 0x89, 0x4c, 0xbd, 0xa2), pid =  6 };    // DEVPROP_TYPE_BOOL
			public static DEVPROPKEY Device_ConfigurationId => new() { fmtid = new Guid(0x540b947e, 0x8b40, 0x45bc, 0xa8, 0xa2, 0x6a, 0x0b, 0x89, 0x4c, 0xbd, 0xa2), pid =  7 };    // DEVPROP_TYPE_STRING
			public static DEVPROPKEY Device_ReportedDeviceIdsHash => new() { fmtid = new Guid(0x540b947e, 0x8b40, 0x45bc, 0xa8, 0xa2, 0x6a, 0x0b, 0x89, 0x4c, 0xbd, 0xa2), pid =  8 };    // DEVPROP_TYPE_UINT32
			public static DEVPROPKEY Device_PhysicalDeviceLocation => new() { fmtid = new Guid(0x540b947e, 0x8b40, 0x45bc, 0xa8, 0xa2, 0x6a, 0x0b, 0x89, 0x4c, 0xbd, 0xa2), pid =  9 };    // DEVPROP_TYPE_BINARY
			public static DEVPROPKEY Device_BiosDeviceName => new() { fmtid = new Guid(0x540b947e, 0x8b40, 0x45bc, 0xa8, 0xa2, 0x6a, 0x0b, 0x89, 0x4c, 0xbd, 0xa2), pid =  10 };   // DEVPROP_TYPE_STRING
			public static DEVPROPKEY Device_DriverProblemDesc => new() { fmtid = new Guid(0x540b947e, 0x8b40, 0x45bc, 0xa8, 0xa2, 0x6a, 0x0b, 0x89, 0x4c, 0xbd, 0xa2), pid =  11 };   // DEVPROP_TYPE_STRING
			public static DEVPROPKEY Device_DebuggerSafe => new() { fmtid = new Guid(0x540b947e, 0x8b40, 0x45bc, 0xa8, 0xa2, 0x6a, 0x0b, 0x89, 0x4c, 0xbd, 0xa2), pid =  12 };   // DEVPROP_TYPE_UINT32
			public static DEVPROPKEY Device_PostInstallInProgress => new() { fmtid = new Guid(0x540b947e, 0x8b40, 0x45bc, 0xa8, 0xa2, 0x6a, 0x0b, 0x89, 0x4c, 0xbd, 0xa2), pid =  13 };   // DEVPROP_TYPE_BOOLEAN
			public static DEVPROPKEY Device_Stack => new() { fmtid = new Guid(0x540b947e, 0x8b40, 0x45bc, 0xa8, 0xa2, 0x6a, 0x0b, 0x89, 0x4c, 0xbd, 0xa2), pid =  14 };   // DEVPROP_TYPE_STRING_LIST
			public static DEVPROPKEY Device_ExtendedConfigurationIds => new() { fmtid = new Guid(0x540b947e, 0x8b40, 0x45bc, 0xa8, 0xa2, 0x6a, 0x0b, 0x89, 0x4c, 0xbd, 0xa2), pid =  15 };   // DEVPROP_TYPE_STRING_LIST
			public static DEVPROPKEY Device_IsRebootRequired => new() { fmtid = new Guid(0x540b947e, 0x8b40, 0x45bc, 0xa8, 0xa2, 0x6a, 0x0b, 0x89, 0x4c, 0xbd, 0xa2), pid =  16 };   // DEVPROP_TYPE_BOOLEAN
			public static DEVPROPKEY Device_FirmwareDate => new() { fmtid = new Guid(0x540b947e, 0x8b40, 0x45bc, 0xa8, 0xa2, 0x6a, 0x0b, 0x89, 0x4c, 0xbd, 0xa2), pid =  17 };   // DEVPROP_TYPE_FILETIME
			public static DEVPROPKEY Device_FirmwareVersion => new() { fmtid = new Guid(0x540b947e, 0x8b40, 0x45bc, 0xa8, 0xa2, 0x6a, 0x0b, 0x89, 0x4c, 0xbd, 0xa2), pid =  18 };   // DEVPROP_TYPE_STRING
			public static DEVPROPKEY Device_FirmwareRevision => new() { fmtid = new Guid(0x540b947e, 0x8b40, 0x45bc, 0xa8, 0xa2, 0x6a, 0x0b, 0x89, 0x4c, 0xbd, 0xa2), pid =  19 };   // DEVPROP_TYPE_STRING
			public static DEVPROPKEY Device_DependencyProviders => new() { fmtid = new Guid(0x540b947e, 0x8b40, 0x45bc, 0xa8, 0xa2, 0x6a, 0x0b, 0x89, 0x4c, 0xbd, 0xa2), pid =  20 };   // DEVPROP_TYPE_STRING_LIST
			public static DEVPROPKEY Device_DependencyDependents => new() { fmtid = new Guid(0x540b947e, 0x8b40, 0x45bc, 0xa8, 0xa2, 0x6a, 0x0b, 0x89, 0x4c, 0xbd, 0xa2), pid =  21 };   // DEVPROP_TYPE_STRING_LIST
			public static DEVPROPKEY Device_SoftRestartSupported => new() { fmtid = new Guid(0x540b947e, 0x8b40, 0x45bc, 0xa8, 0xa2, 0x6a, 0x0b, 0x89, 0x4c, 0xbd, 0xa2), pid =  22 };   // DEVPROP_TYPE_BOOLEAN
			public static DEVPROPKEY Device_ExtendedAddress => new() { fmtid = new Guid(0x540b947e, 0x8b40, 0x45bc, 0xa8, 0xa2, 0x6a, 0x0b, 0x89, 0x4c, 0xbd, 0xa2), pid =  23 };   // DEVPROP_TYPE_UINT64
			public static DEVPROPKEY Device_AssignedToGuest => new() { fmtid = new Guid(0x540b947e, 0x8b40, 0x45bc, 0xa8, 0xa2, 0x6a, 0x0b, 0x89, 0x4c, 0xbd, 0xa2), pid =  24 };   // DEVPROP_TYPE_BOOLEAN
			public static DEVPROPKEY Device_SessionId => new() { fmtid = new Guid(0x83da6326, 0x97a6, 0x4088, 0x94, 0x53, 0xa1, 0x92, 0x3f, 0x57, 0x3b, 0x29), pid =  6 };     // DEVPROP_TYPE_UINT32

			// Device activity timestamp properties
			public static DEVPROPKEY Device_InstallDate => new() { fmtid = new Guid(0x83da6326, 0x97a6, 0x4088, 0x94, 0x53, 0xa1, 0x92, 0x3f, 0x57, 0x3b, 0x29), pid =  100 };   // DEVPROP_TYPE_FILETIME
			public static DEVPROPKEY Device_FirstInstallDate => new() { fmtid = new Guid(0x83da6326, 0x97a6, 0x4088, 0x94, 0x53, 0xa1, 0x92, 0x3f, 0x57, 0x3b, 0x29), pid =  101 };   // DEVPROP_TYPE_FILETIME
			public static DEVPROPKEY Device_LastArrivalDate => new() { fmtid = new Guid(0x83da6326, 0x97a6, 0x4088, 0x94, 0x53, 0xa1, 0x92, 0x3f, 0x57, 0x3b, 0x29), pid =  102 };   // DEVPROP_TYPE_FILETIME
			public static DEVPROPKEY Device_LastRemovalDate => new() { fmtid = new Guid(0x83da6326, 0x97a6, 0x4088, 0x94, 0x53, 0xa1, 0x92, 0x3f, 0x57, 0x3b, 0x29), pid =  103 };   // DEVPROP_TYPE_FILETIME

			// Device driver properties
			public static DEVPROPKEY Device_DriverDate => new() { fmtid = new Guid(0xa8b865dd, 0x2e3d, 0x4094, 0xad, 0x97, 0xe5, 0x93, 0xa7, 0xc, 0x75, 0xd6), pid =  2 };     // DEVPROP_TYPE_FILETIME
			public static DEVPROPKEY Device_DriverVersion => new() { fmtid = new Guid(0xa8b865dd, 0x2e3d, 0x4094, 0xad, 0x97, 0xe5, 0x93, 0xa7, 0xc, 0x75, 0xd6), pid =  3 };     // DEVPROP_TYPE_STRING
			public static DEVPROPKEY Device_DriverDesc => new() { fmtid = new Guid(0xa8b865dd, 0x2e3d, 0x4094, 0xad, 0x97, 0xe5, 0x93, 0xa7, 0xc, 0x75, 0xd6), pid =  4 };     // DEVPROP_TYPE_STRING
			public static DEVPROPKEY Device_DriverInfPath => new() { fmtid = new Guid(0xa8b865dd, 0x2e3d, 0x4094, 0xad, 0x97, 0xe5, 0x93, 0xa7, 0xc, 0x75, 0xd6), pid =  5 };     // DEVPROP_TYPE_STRING
			public static DEVPROPKEY Device_DriverInfSection => new() { fmtid = new Guid(0xa8b865dd, 0x2e3d, 0x4094, 0xad, 0x97, 0xe5, 0x93, 0xa7, 0xc, 0x75, 0xd6), pid =  6 };     // DEVPROP_TYPE_STRING
			public static DEVPROPKEY Device_DriverInfSectionExt => new() { fmtid = new Guid(0xa8b865dd, 0x2e3d, 0x4094, 0xad, 0x97, 0xe5, 0x93, 0xa7, 0xc, 0x75, 0xd6), pid =  7 };     // DEVPROP_TYPE_STRING
			public static DEVPROPKEY Device_MatchingDeviceId => new() { fmtid = new Guid(0xa8b865dd, 0x2e3d, 0x4094, 0xad, 0x97, 0xe5, 0x93, 0xa7, 0xc, 0x75, 0xd6), pid =  8 };     // DEVPROP_TYPE_STRING
			public static DEVPROPKEY Device_DriverProvider => new() { fmtid = new Guid(0xa8b865dd, 0x2e3d, 0x4094, 0xad, 0x97, 0xe5, 0x93, 0xa7, 0xc, 0x75, 0xd6), pid =  9 };     // DEVPROP_TYPE_STRING
			public static DEVPROPKEY Device_DriverPropPageProvider => new() { fmtid = new Guid(0xa8b865dd, 0x2e3d, 0x4094, 0xad, 0x97, 0xe5, 0x93, 0xa7, 0xc, 0x75, 0xd6), pid =  10 };    // DEVPROP_TYPE_STRING
			public static DEVPROPKEY Device_DriverCoInstallers => new() { fmtid = new Guid(0xa8b865dd, 0x2e3d, 0x4094, 0xad, 0x97, 0xe5, 0x93, 0xa7, 0xc, 0x75, 0xd6), pid =  11 };    // DEVPROP_TYPE_STRING_LIST
			public static DEVPROPKEY Device_ResourcePickerTags => new() { fmtid = new Guid(0xa8b865dd, 0x2e3d, 0x4094, 0xad, 0x97, 0xe5, 0x93, 0xa7, 0xc, 0x75, 0xd6), pid =  12 };    // DEVPROP_TYPE_STRING
			public static DEVPROPKEY Device_ResourcePickerExceptions => new() { fmtid = new Guid(0xa8b865dd, 0x2e3d, 0x4094, 0xad, 0x97, 0xe5, 0x93, 0xa7, 0xc, 0x75, 0xd6), pid =  13 };    // DEVPROP_TYPE_STRING
			public static DEVPROPKEY Device_DriverRank => new() { fmtid = new Guid(0xa8b865dd, 0x2e3d, 0x4094, 0xad, 0x97, 0xe5, 0x93, 0xa7, 0xc, 0x75, 0xd6), pid =  14 };    // DEVPROP_TYPE_UINT32
			public static DEVPROPKEY Device_DriverLogoLevel => new() { fmtid = new Guid(0xa8b865dd, 0x2e3d, 0x4094, 0xad, 0x97, 0xe5, 0x93, 0xa7, 0xc, 0x75, 0xd6), pid =  15 };    // DEVPROP_TYPE_UINT32

			// Device properties
			// These DEVPKEYs may be set by the driver package installed for a device.
			public static DEVPROPKEY Device_NoConnectSound => new() { fmtid = new Guid(0xa8b865dd, 0x2e3d, 0x4094, 0xad, 0x97, 0xe5, 0x93, 0xa7, 0xc, 0x75, 0xd6), pid =  17 }; // DEVPROP_TYPE_BOOLEAN
			public static DEVPROPKEY Device_GenericDriverInstalled => new() { fmtid = new Guid(0xa8b865dd, 0x2e3d, 0x4094, 0xad, 0x97, 0xe5, 0x93, 0xa7, 0xc, 0x75, 0xd6), pid =  18 }; // DEVPROP_TYPE_BOOLEAN
			public static DEVPROPKEY Device_AdditionalSoftwareRequested => new() { fmtid = new Guid(0xa8b865dd, 0x2e3d, 0x4094, 0xad, 0x97, 0xe5, 0x93, 0xa7, 0xc, 0x75, 0xd6), pid =  19 }; // DEVPROP_TYPE_BOOLEAN

			// Device safe-removal properties
			public static DEVPROPKEY Device_SafeRemovalRequired => new() { fmtid = new Guid(0xafd97640, 0x86a3, 0x4210, 0xb6, 0x7c, 0x28, 0x9c, 0x41, 0xaa, 0xbe, 0x55), pid =  2 }; // DEVPROP_TYPE_BOOLEAN
			public static DEVPROPKEY Device_SafeRemovalRequiredOverride => new() { fmtid = new Guid(0xafd97640, 0x86a3, 0x4210, 0xb6, 0x7c, 0x28, 0x9c, 0x41, 0xaa, 0xbe, 0x55), pid =  3 }; // DEVPROP_TYPE_BOOLEAN

			// Device properties
			// These DEVPKEYs may be set by the driver package installed for a device.
			public static DEVPROPKEY DrvPkg_Model => new() { fmtid = new Guid(0xcf73bb51, 0x3abf, 0x44a2, 0x85, 0xe0, 0x9a, 0x3d, 0xc7, 0xa1, 0x21, 0x32), pid =  2 };     // DEVPROP_TYPE_STRING
			public static DEVPROPKEY DrvPkg_VendorWebSite => new() { fmtid = new Guid(0xcf73bb51, 0x3abf, 0x44a2, 0x85, 0xe0, 0x9a, 0x3d, 0xc7, 0xa1, 0x21, 0x32), pid =  3 };     // DEVPROP_TYPE_STRING
			public static DEVPROPKEY DrvPkg_DetailedDescription => new() { fmtid = new Guid(0xcf73bb51, 0x3abf, 0x44a2, 0x85, 0xe0, 0x9a, 0x3d, 0xc7, 0xa1, 0x21, 0x32), pid =  4 };     // DEVPROP_TYPE_STRING
			public static DEVPROPKEY DrvPkg_DocumentationLink => new() { fmtid = new Guid(0xcf73bb51, 0x3abf, 0x44a2, 0x85, 0xe0, 0x9a, 0x3d, 0xc7, 0xa1, 0x21, 0x32), pid =  5 };     // DEVPROP_TYPE_STRING
			public static DEVPROPKEY DrvPkg_Icon => new() { fmtid = new Guid(0xcf73bb51, 0x3abf, 0x44a2, 0x85, 0xe0, 0x9a, 0x3d, 0xc7, 0xa1, 0x21, 0x32), pid =  6 };     // DEVPROP_TYPE_STRING_LIST
			public static DEVPROPKEY DrvPkg_BrandingIcon => new() { fmtid = new Guid(0xcf73bb51, 0x3abf, 0x44a2, 0x85, 0xe0, 0x9a, 0x3d, 0xc7, 0xa1, 0x21, 0x32), pid =  7 };     // DEVPROP_TYPE_STRING_LIST

			// Device setup class properties
			// These DEVPKEYs correspond to the SetupAPI SPCRP_XXX setup class properties.
			public static DEVPROPKEY DeviceClass_UpperFilters => new() { fmtid = new Guid(0x4321918b, 0xf69e, 0x470d, 0xa5, 0xde, 0x4d, 0x88, 0xc7, 0x5a, 0xd2, 0x4b), pid =  19 };    // DEVPROP_TYPE_STRING_LIST
			public static DEVPROPKEY DeviceClass_LowerFilters => new() { fmtid = new Guid(0x4321918b, 0xf69e, 0x470d, 0xa5, 0xde, 0x4d, 0x88, 0xc7, 0x5a, 0xd2, 0x4b), pid =  20 };    // DEVPROP_TYPE_STRING_LIST
			public static DEVPROPKEY DeviceClass_Security => new() { fmtid = new Guid(0x4321918b, 0xf69e, 0x470d, 0xa5, 0xde, 0x4d, 0x88, 0xc7, 0x5a, 0xd2, 0x4b), pid =  25 };    // DEVPROP_TYPE_SECURITY_DESCRIPTOR
			public static DEVPROPKEY DeviceClass_SecuritySDS => new() { fmtid = new Guid(0x4321918b, 0xf69e, 0x470d, 0xa5, 0xde, 0x4d, 0x88, 0xc7, 0x5a, 0xd2, 0x4b), pid =  26 };    // DEVPROP_TYPE_SECURITY_DESCRIPTOR_STRING
			public static DEVPROPKEY DeviceClass_DevType => new() { fmtid = new Guid(0x4321918b, 0xf69e, 0x470d, 0xa5, 0xde, 0x4d, 0x88, 0xc7, 0x5a, 0xd2, 0x4b), pid =  27 };    // DEVPROP_TYPE_UINT32
			public static DEVPROPKEY DeviceClass_Exclusive => new() { fmtid = new Guid(0x4321918b, 0xf69e, 0x470d, 0xa5, 0xde, 0x4d, 0x88, 0xc7, 0x5a, 0xd2, 0x4b), pid =  28 };    // DEVPROP_TYPE_BOOLEAN
			public static DEVPROPKEY DeviceClass_Characteristics => new() { fmtid = new Guid(0x4321918b, 0xf69e, 0x470d, 0xa5, 0xde, 0x4d, 0x88, 0xc7, 0x5a, 0xd2, 0x4b), pid =  29 };    // DEVPROP_TYPE_UINT32

			// Device setup class properties
			public static DEVPROPKEY DeviceClass_Name => new() { fmtid = new Guid(0x259abffc, 0x50a7, 0x47ce, 0xaf, 0x8, 0x68, 0xc9, 0xa7, 0xd7, 0x33, 0x66), pid =  2 };      // DEVPROP_TYPE_STRING
			public static DEVPROPKEY DeviceClass_ClassName => new() { fmtid = new Guid(0x259abffc, 0x50a7, 0x47ce, 0xaf, 0x8, 0x68, 0xc9, 0xa7, 0xd7, 0x33, 0x66), pid =  3 };      // DEVPROP_TYPE_STRING
			public static DEVPROPKEY DeviceClass_Icon => new() { fmtid = new Guid(0x259abffc, 0x50a7, 0x47ce, 0xaf, 0x8, 0x68, 0xc9, 0xa7, 0xd7, 0x33, 0x66), pid =  4 };      // DEVPROP_TYPE_STRING
			public static DEVPROPKEY DeviceClass_ClassInstaller => new() { fmtid = new Guid(0x259abffc, 0x50a7, 0x47ce, 0xaf, 0x8, 0x68, 0xc9, 0xa7, 0xd7, 0x33, 0x66), pid =  5 };      // DEVPROP_TYPE_STRING
			public static DEVPROPKEY DeviceClass_PropPageProvider => new() { fmtid = new Guid(0x259abffc, 0x50a7, 0x47ce, 0xaf, 0x8, 0x68, 0xc9, 0xa7, 0xd7, 0x33, 0x66), pid =  6 };      // DEVPROP_TYPE_STRING
			public static DEVPROPKEY DeviceClass_NoInstallClass => new() { fmtid = new Guid(0x259abffc, 0x50a7, 0x47ce, 0xaf, 0x8, 0x68, 0xc9, 0xa7, 0xd7, 0x33, 0x66), pid =  7 };      // DEVPROP_TYPE_BOOLEAN
			public static DEVPROPKEY DeviceClass_NoDisplayClass => new() { fmtid = new Guid(0x259abffc, 0x50a7, 0x47ce, 0xaf, 0x8, 0x68, 0xc9, 0xa7, 0xd7, 0x33, 0x66), pid =  8 };      // DEVPROP_TYPE_BOOLEAN
			public static DEVPROPKEY DeviceClass_SilentInstall => new() { fmtid = new Guid(0x259abffc, 0x50a7, 0x47ce, 0xaf, 0x8, 0x68, 0xc9, 0xa7, 0xd7, 0x33, 0x66), pid =  9 };      // DEVPROP_TYPE_BOOLEAN
			public static DEVPROPKEY DeviceClass_NoUseClass => new() { fmtid = new Guid(0x259abffc, 0x50a7, 0x47ce, 0xaf, 0x8, 0x68, 0xc9, 0xa7, 0xd7, 0x33, 0x66), pid =  10 };     // DEVPROP_TYPE_BOOLEAN
			public static DEVPROPKEY DeviceClass_DefaultService => new() { fmtid = new Guid(0x259abffc, 0x50a7, 0x47ce, 0xaf, 0x8, 0x68, 0xc9, 0xa7, 0xd7, 0x33, 0x66), pid =  11 };     // DEVPROP_TYPE_STRING
			public static DEVPROPKEY DeviceClass_IconPath => new() { fmtid = new Guid(0x259abffc, 0x50a7, 0x47ce, 0xaf, 0x8, 0x68, 0xc9, 0xa7, 0xd7, 0x33, 0x66), pid =  12 };     // DEVPROP_TYPE_STRING_LIST

			// Other Device setup class properties
			public static DEVPROPKEY DeviceClass_DHPRebalanceOptOut => new() { fmtid = new Guid(0xd14d3ef3, 0x66cf, 0x4ba2, 0x9d, 0x38, 0x0d, 0xdb, 0x37, 0xab, 0x47, 0x01), pid =  2 };    // DEVPROP_TYPE_BOOLEAN
			public static DEVPROPKEY DeviceClass_ClassCoInstallers => new() { fmtid = new Guid(0x713d1703, 0xa2e2, 0x49f5, 0x92, 0x14, 0x56, 0x47, 0x2e, 0xf3, 0xda, 0x5c), pid =  2 };     // DEVPROP_TYPE_STRING_LIST

			// Device interface properties
			public static DEVPROPKEY DeviceInterface_FriendlyName => new() { fmtid = new Guid(0x026e516e, 0xb814, 0x414b, 0x83, 0xcd, 0x85, 0x6d, 0x6f, 0xef, 0x48, 0x22), pid =  2 };   // DEVPROP_TYPE_STRING
			public static DEVPROPKEY DeviceInterface_Enabled => new() { fmtid = new Guid(0x026e516e, 0xb814, 0x414b, 0x83, 0xcd, 0x85, 0x6d, 0x6f, 0xef, 0x48, 0x22), pid =  3 };   // DEVPROP_TYPE_BOOLEAN
			public static DEVPROPKEY DeviceInterface_ClassGuid => new() { fmtid = new Guid(0x026e516e, 0xb814, 0x414b, 0x83, 0xcd, 0x85, 0x6d, 0x6f, 0xef, 0x48, 0x22), pid =  4 };   // DEVPROP_TYPE_GUID
			public static DEVPROPKEY DeviceInterface_ReferenceString => new() { fmtid = new Guid(0x026e516e, 0xb814, 0x414b, 0x83, 0xcd, 0x85, 0x6d, 0x6f, 0xef, 0x48, 0x22), pid =  5 };   // DEVPROP_TYPE_STRING
			public static DEVPROPKEY DeviceInterface_Restricted => new() { fmtid = new Guid(0x026e516e, 0xb814, 0x414b, 0x83, 0xcd, 0x85, 0x6d, 0x6f, 0xef, 0x48, 0x22), pid =  6 };   // DEVPROP_TYPE_BOOLEAN
			public static DEVPROPKEY DeviceInterface_UnrestrictedAppCapabilities => new() { fmtid = new Guid(0x026e516e, 0xb814, 0x414b, 0x83, 0xcd, 0x85, 0x6d, 0x6f, 0xef, 0x48, 0x22), pid =  8 }; // DEVPROP_TYPE_STRING_LIST
			public static DEVPROPKEY DeviceInterface_SchematicName => new() { fmtid = new Guid(0x026e516e, 0xb814, 0x414b, 0x83, 0xcd, 0x85, 0x6d, 0x6f, 0xef, 0x48, 0x22), pid =  9 };   // DEVPROP_TYPE_STRING

			// Device interface class properties
			public static DEVPROPKEY DeviceInterfaceClass_DefaultInterface => new() { fmtid = new Guid(0x14c83a99, 0x0b3f, 0x44b7, 0xbe, 0x4c, 0xa1, 0x78, 0xd3, 0x99, 0x05, 0x64), pid =  2 }; // DEVPROP_TYPE_STRING
			public static DEVPROPKEY DeviceInterfaceClass_Name => new() { fmtid = new Guid(0x14c83a99, 0x0b3f, 0x44b7, 0xbe, 0x4c, 0xa1, 0x78, 0xd3, 0x99, 0x05, 0x64), pid =  3 }; // DEVPROP_TYPE_STRING

			// Device Container Properties
			public static DEVPROPKEY DeviceContainer_Address => new() { fmtid = new Guid(0x78c34fc8, 0x104a, 0x4aca, 0x9e, 0xa4, 0x52, 0x4d, 0x52, 0x99, 0x6e, 0x57), pid =  51 };    // DEVPROP_TYPE_STRING | DEVPROP_TYPE_STRING_LIST
			public static DEVPROPKEY DeviceContainer_DiscoveryMethod => new() { fmtid = new Guid(0x78c34fc8, 0x104a, 0x4aca, 0x9e, 0xa4, 0x52, 0x4d, 0x52, 0x99, 0x6e, 0x57), pid =  52 };    // DEVPROP_TYPE_STRING_LIST
			public static DEVPROPKEY DeviceContainer_IsEncrypted => new() { fmtid = new Guid(0x78c34fc8, 0x104a, 0x4aca, 0x9e, 0xa4, 0x52, 0x4d, 0x52, 0x99, 0x6e, 0x57), pid =  53 };    // DEVPROP_TYPE_BOOLEAN
			public static DEVPROPKEY DeviceContainer_IsAuthenticated => new() { fmtid = new Guid(0x78c34fc8, 0x104a, 0x4aca, 0x9e, 0xa4, 0x52, 0x4d, 0x52, 0x99, 0x6e, 0x57), pid =  54 };    // DEVPROP_TYPE_BOOLEAN
			public static DEVPROPKEY DeviceContainer_IsConnected => new() { fmtid = new Guid(0x78c34fc8, 0x104a, 0x4aca, 0x9e, 0xa4, 0x52, 0x4d, 0x52, 0x99, 0x6e, 0x57), pid =  55 };    // DEVPROP_TYPE_BOOLEAN
			public static DEVPROPKEY DeviceContainer_IsPaired => new() { fmtid = new Guid(0x78c34fc8, 0x104a, 0x4aca, 0x9e, 0xa4, 0x52, 0x4d, 0x52, 0x99, 0x6e, 0x57), pid =  56 };    // DEVPROP_TYPE_BOOLEAN
			public static DEVPROPKEY DeviceContainer_Icon => new() { fmtid = new Guid(0x78c34fc8, 0x104a, 0x4aca, 0x9e, 0xa4, 0x52, 0x4d, 0x52, 0x99, 0x6e, 0x57), pid =  57 };    // DEVPROP_TYPE_STRING
			public static DEVPROPKEY DeviceContainer_Version => new() { fmtid = new Guid(0x78c34fc8, 0x104a, 0x4aca, 0x9e, 0xa4, 0x52, 0x4d, 0x52, 0x99, 0x6e, 0x57), pid =  65 };    // DEVPROP_TYPE_STRING
			public static DEVPROPKEY DeviceContainer_Last_Seen => new() { fmtid = new Guid(0x78c34fc8, 0x104a, 0x4aca, 0x9e, 0xa4, 0x52, 0x4d, 0x52, 0x99, 0x6e, 0x57), pid =  66 };    // DEVPROP_TYPE_FILETIME
			public static DEVPROPKEY DeviceContainer_Last_Connected => new() { fmtid = new Guid(0x78c34fc8, 0x104a, 0x4aca, 0x9e, 0xa4, 0x52, 0x4d, 0x52, 0x99, 0x6e, 0x57), pid =  67 };    // DEVPROP_TYPE_FILETIME
			public static DEVPROPKEY DeviceContainer_IsShowInDisconnectedState => new() { fmtid = new Guid(0x78c34fc8, 0x104a, 0x4aca, 0x9e, 0xa4, 0x52, 0x4d, 0x52, 0x99, 0x6e, 0x57), pid =  68 };   // DEVPROP_TYPE_BOOLEAN
			public static DEVPROPKEY DeviceContainer_IsLocalMachine => new() { fmtid = new Guid(0x78c34fc8, 0x104a, 0x4aca, 0x9e, 0xa4, 0x52, 0x4d, 0x52, 0x99, 0x6e, 0x57), pid =  70 };    // DEVPROP_TYPE_BOOLEAN
			public static DEVPROPKEY DeviceContainer_MetadataPath => new() { fmtid = new Guid(0x78c34fc8, 0x104a, 0x4aca, 0x9e, 0xa4, 0x52, 0x4d, 0x52, 0x99, 0x6e, 0x57), pid =  71 };    // DEVPROP_TYPE_STRING
			public static DEVPROPKEY DeviceContainer_IsMetadataSearchInProgress => new() { fmtid = new Guid(0x78c34fc8, 0x104a, 0x4aca, 0x9e, 0xa4, 0x52, 0x4d, 0x52, 0x99, 0x6e, 0x57), pid =  72 };          // DEVPROP_TYPE_BOOLEAN
			public static DEVPROPKEY DeviceContainer_MetadataChecksum => new() { fmtid = new Guid(0x78c34fc8, 0x104a, 0x4aca, 0x9e, 0xa4, 0x52, 0x4d, 0x52, 0x99, 0x6e, 0x57), pid =  73 };            // DEVPROP_TYPE_BINARY
			public static DEVPROPKEY DeviceContainer_IsNotInterestingForDisplay => new() { fmtid = new Guid(0x78c34fc8, 0x104a, 0x4aca, 0x9e, 0xa4, 0x52, 0x4d, 0x52, 0x99, 0x6e, 0x57), pid =  74 };          // DEVPROP_TYPE_BOOLEAN
			public static DEVPROPKEY DeviceContainer_LaunchDeviceStageOnDeviceConnect => new() { fmtid = new Guid(0x78c34fc8, 0x104a, 0x4aca, 0x9e, 0xa4, 0x52, 0x4d, 0x52, 0x99, 0x6e, 0x57), pid =  76 };    // DEVPROP_TYPE_BOOLEAN
			public static DEVPROPKEY DeviceContainer_LaunchDeviceStageFromExplorer => new() { fmtid = new Guid(0x78c34fc8, 0x104a, 0x4aca, 0x9e, 0xa4, 0x52, 0x4d, 0x52, 0x99, 0x6e, 0x57), pid =  77 };       // DEVPROP_TYPE_BOOLEAN
			public static DEVPROPKEY DeviceContainer_BaselineExperienceId => new() { fmtid = new Guid(0x78c34fc8, 0x104a, 0x4aca, 0x9e, 0xa4, 0x52, 0x4d, 0x52, 0x99, 0x6e, 0x57), pid =  78 };    // DEVPROP_TYPE_GUID
			public static DEVPROPKEY DeviceContainer_IsDeviceUniquelyIdentifiable => new() { fmtid = new Guid(0x78c34fc8, 0x104a, 0x4aca, 0x9e, 0xa4, 0x52, 0x4d, 0x52, 0x99, 0x6e, 0x57), pid =  79 };        // DEVPROP_TYPE_BOOLEAN
			public static DEVPROPKEY DeviceContainer_AssociationArray => new() { fmtid = new Guid(0x78c34fc8, 0x104a, 0x4aca, 0x9e, 0xa4, 0x52, 0x4d, 0x52, 0x99, 0x6e, 0x57), pid =  80 };    // DEVPROP_TYPE_STRING_LIST
			public static DEVPROPKEY DeviceContainer_DeviceDescription1 => new() { fmtid = new Guid(0x78c34fc8, 0x104a, 0x4aca, 0x9e, 0xa4, 0x52, 0x4d, 0x52, 0x99, 0x6e, 0x57), pid =  81 };    // DEVPROP_TYPE_STRING
			public static DEVPROPKEY DeviceContainer_DeviceDescription2 => new() { fmtid = new Guid(0x78c34fc8, 0x104a, 0x4aca, 0x9e, 0xa4, 0x52, 0x4d, 0x52, 0x99, 0x6e, 0x57), pid =  82 };    // DEVPROP_TYPE_STRING
			public static DEVPROPKEY DeviceContainer_HasProblem => new() { fmtid = new Guid(0x78c34fc8, 0x104a, 0x4aca, 0x9e, 0xa4, 0x52, 0x4d, 0x52, 0x99, 0x6e, 0x57), pid =  83 };    // DEVPROP_TYPE_BOOLEAN
			public static DEVPROPKEY DeviceContainer_IsSharedDevice => new() { fmtid = new Guid(0x78c34fc8, 0x104a, 0x4aca, 0x9e, 0xa4, 0x52, 0x4d, 0x52, 0x99, 0x6e, 0x57), pid =  84 };    // DEVPROP_TYPE_BOOLEAN
			public static DEVPROPKEY DeviceContainer_IsNetworkDevice => new() { fmtid = new Guid(0x78c34fc8, 0x104a, 0x4aca, 0x9e, 0xa4, 0x52, 0x4d, 0x52, 0x99, 0x6e, 0x57), pid =  85 };    // DEVPROP_TYPE_BOOLEAN
			public static DEVPROPKEY DeviceContainer_IsDefaultDevice => new() { fmtid = new Guid(0x78c34fc8, 0x104a, 0x4aca, 0x9e, 0xa4, 0x52, 0x4d, 0x52, 0x99, 0x6e, 0x57), pid =  86 };    // DEVPROP_TYPE_BOOLEAN
			public static DEVPROPKEY DeviceContainer_MetadataCabinet => new() { fmtid = new Guid(0x78c34fc8, 0x104a, 0x4aca, 0x9e, 0xa4, 0x52, 0x4d, 0x52, 0x99, 0x6e, 0x57), pid =  87 };    // DEVPROP_TYPE_STRING
			public static DEVPROPKEY DeviceContainer_RequiresPairingElevation => new() { fmtid = new Guid(0x78c34fc8, 0x104a, 0x4aca, 0x9e, 0xa4, 0x52, 0x4d, 0x52, 0x99, 0x6e, 0x57), pid =  88 };    // DEVPROP_TYPE_BOOLEAN
			public static DEVPROPKEY DeviceContainer_ExperienceId => new() { fmtid = new Guid(0x78c34fc8, 0x104a, 0x4aca, 0x9e, 0xa4, 0x52, 0x4d, 0x52, 0x99, 0x6e, 0x57), pid =  89 };    // DEVPROP_TYPE_GUID
			public static DEVPROPKEY DeviceContainer_Category => new() { fmtid = new Guid(0x78c34fc8, 0x104a, 0x4aca, 0x9e, 0xa4, 0x52, 0x4d, 0x52, 0x99, 0x6e, 0x57), pid =  90 };    // DEVPROP_TYPE_STRING_LIST
			public static DEVPROPKEY DeviceContainer_Category_Desc_Singular => new() { fmtid = new Guid(0x78c34fc8, 0x104a, 0x4aca, 0x9e, 0xa4, 0x52, 0x4d, 0x52, 0x99, 0x6e, 0x57), pid =  91 };    // DEVPROP_TYPE_STRING_LIST
			public static DEVPROPKEY DeviceContainer_Category_Desc_Plural => new() { fmtid = new Guid(0x78c34fc8, 0x104a, 0x4aca, 0x9e, 0xa4, 0x52, 0x4d, 0x52, 0x99, 0x6e, 0x57), pid =  92 };    // DEVPROP_TYPE_STRING_LIST
			public static DEVPROPKEY DeviceContainer_Category_Icon => new() { fmtid = new Guid(0x78c34fc8, 0x104a, 0x4aca, 0x9e, 0xa4, 0x52, 0x4d, 0x52, 0x99, 0x6e, 0x57), pid =  93 };    // DEVPROP_TYPE_STRING
			public static DEVPROPKEY DeviceContainer_CategoryGroup_Desc => new() { fmtid = new Guid(0x78c34fc8, 0x104a, 0x4aca, 0x9e, 0xa4, 0x52, 0x4d, 0x52, 0x99, 0x6e, 0x57), pid =  94 };    // DEVPROP_TYPE_STRING_LIST
			public static DEVPROPKEY DeviceContainer_CategoryGroup_Icon => new() { fmtid = new Guid(0x78c34fc8, 0x104a, 0x4aca, 0x9e, 0xa4, 0x52, 0x4d, 0x52, 0x99, 0x6e, 0x57), pid =  95 };    // DEVPROP_TYPE_STRING
			public static DEVPROPKEY DeviceContainer_PrimaryCategory => new() { fmtid = new Guid(0x78c34fc8, 0x104a, 0x4aca, 0x9e, 0xa4, 0x52, 0x4d, 0x52, 0x99, 0x6e, 0x57), pid =  97 };    // DEVPROP_TYPE_STRING
			public static DEVPROPKEY DeviceContainer_UnpairUninstall => new() { fmtid = new Guid(0x78c34fc8, 0x104a, 0x4aca, 0x9e, 0xa4, 0x52, 0x4d, 0x52, 0x99, 0x6e, 0x57), pid =  98 };    // DEVPROP_TYPE_BOOLEAN
			public static DEVPROPKEY DeviceContainer_RequiresUninstallElevation => new() { fmtid = new Guid(0x78c34fc8, 0x104a, 0x4aca, 0x9e, 0xa4, 0x52, 0x4d, 0x52, 0x99, 0x6e, 0x57), pid =  99 };  // DEVPROP_TYPE_BOOLEAN
			public static DEVPROPKEY DeviceContainer_DeviceFunctionSubRank => new() { fmtid = new Guid(0x78c34fc8, 0x104a, 0x4aca, 0x9e, 0xa4, 0x52, 0x4d, 0x52, 0x99, 0x6e, 0x57), pid =  100 };   // DEVPROP_TYPE_UINT32
			public static DEVPROPKEY DeviceContainer_AlwaysShowDeviceAsConnected => new() { fmtid = new Guid(0x78c34fc8, 0x104a, 0x4aca, 0x9e, 0xa4, 0x52, 0x4d, 0x52, 0x99, 0x6e, 0x57), pid =  101 };    // DEVPROP_TYPE_BOOLEAN
			public static DEVPROPKEY DeviceContainer_ConfigFlags => new() { fmtid = new Guid(0x78c34fc8, 0x104a, 0x4aca, 0x9e, 0xa4, 0x52, 0x4d, 0x52, 0x99, 0x6e, 0x57), pid =  105 };   // DEVPROP_TYPE_UINT32
			public static DEVPROPKEY DeviceContainer_PrivilegedPackageFamilyNames => new() { fmtid = new Guid(0x78c34fc8, 0x104a, 0x4aca, 0x9e, 0xa4, 0x52, 0x4d, 0x52, 0x99, 0x6e, 0x57), pid =  106 };   // DEVPROP_TYPE_STRING_LIST
			public static DEVPROPKEY DeviceContainer_CustomPrivilegedPackageFamilyNames => new() { fmtid = new Guid(0x78c34fc8, 0x104a, 0x4aca, 0x9e, 0xa4, 0x52, 0x4d, 0x52, 0x99, 0x6e, 0x57), pid =  107 };   // DEVPROP_TYPE_STRING_LIST
			public static DEVPROPKEY DeviceContainer_IsRebootRequired => new() { fmtid = new Guid(0x78c34fc8, 0x104a, 0x4aca, 0x9e, 0xa4, 0x52, 0x4d, 0x52, 0x99, 0x6e, 0x57), pid =  108 };   // DEVPROP_TYPE_BOOLEAN
			public static DEVPROPKEY DeviceContainer_FriendlyName => new() { fmtid = new Guid(0x656A3BB3, 0xECC0, 0x43FD, 0x84, 0x77, 0x4A, 0xE0, 0x40, 0x4A, 0x96, 0xCD), pid =  12288 }; // DEVPROP_TYPE_STRING
			public static DEVPROPKEY DeviceContainer_Manufacturer => new() { fmtid = new Guid(0x656A3BB3, 0xECC0, 0x43FD, 0x84, 0x77, 0x4A, 0xE0, 0x40, 0x4A, 0x96, 0xCD), pid =  8192 };  // DEVPROP_TYPE_STRING
			public static DEVPROPKEY DeviceContainer_ModelName => new() { fmtid = new Guid(0x656A3BB3, 0xECC0, 0x43FD, 0x84, 0x77, 0x4A, 0xE0, 0x40, 0x4A, 0x96, 0xCD), pid =  8194 };  // DEVPROP_TYPE_STRING (localizable)
			public static DEVPROPKEY DeviceContainer_ModelNumber => new() { fmtid = new Guid(0x656A3BB3, 0xECC0, 0x43FD, 0x84, 0x77, 0x4A, 0xE0, 0x40, 0x4A, 0x96, 0xCD), pid =  8195 };  // DEVPROP_TYPE_STRING
			public static DEVPROPKEY DeviceContainer_InstallInProgress => new() { fmtid = new Guid(0x83da6326, 0x97a6, 0x4088, 0x94, 0x53, 0xa1, 0x92, 0x3f, 0x57, 0x3b, 0x29), pid =  9 };     // DEVPROP_TYPE_BOOLEAN
			#endregion
		}

		[StructLayout(LayoutKind.Sequential)]
		public struct DeviceInfoData
		{
			public int Size;
			public Guid ClassGuid;
			public int DevInst;
			public IntPtr Reserved;
		}

		[StructLayout(LayoutKind.Sequential)]
		public struct SP_CLASSINSTALL_HEADER
		{
			public int Size;
			public uint DiFunction;
		}

		[StructLayout(LayoutKind.Sequential)]
		public struct PropertyChangeParameters
		{
			public SP_CLASSINSTALL_HEADER hdr;
			public EStateChangeAction StateChange;
			public EScope Scope;
			public int HwProfile;
		}

		/// <summary></summary>
		public static bool CallClassInstaller(EDiFunction install_function, DevClassSafeHandle handle, DeviceInfoData did)
		{
			return SetupDiCallClassInstaller_(install_function, handle, ref did);
		}
		[DllImport("setupapi.dll", EntryPoint = "SetupDiCallClassInstaller", SetLastError = true)]
		private static extern bool SetupDiCallClassInstaller_(EDiFunction installFunction, DevClassSafeHandle deviceInfoSet, [In] ref DeviceInfoData deviceInfoData);

		/// <summary></summary>
		public static bool SetClassInstallParams(DevClassSafeHandle handle, DeviceInfoData did, PropertyChangeParameters parms)
		{
			return SetupDiSetClassInstallParams_(handle, ref did, ref parms, Marshal.SizeOf(parms));

		}
		[DllImport("setupapi.dll", EntryPoint = "SetupDiSetClassInstallParamsW", CharSet = CharSet.Unicode, SetLastError = true)]
		private static extern bool SetupDiSetClassInstallParams_(DevClassSafeHandle deviceInfoSet, [In] ref DeviceInfoData deviceInfoData, [In] ref PropertyChangeParameters classInstallParams, int classInstallParamsSize);

		/// <summary>Returns info for each device in a device information set.</summary>
		public static IEnumerable<DeviceInfoData> EnumDeviceInfo(DevClassSafeHandle handle)
		{
			var did = new DeviceInfoData { Size = Marshal.SizeOf<DeviceInfoData>() };
			for (int i = 0; SetupDiEnumDeviceInfo_(handle, i, ref did); ++i)
				yield return did;
			
			if (Marshal.GetLastWin32Error() is int err && err != Win32.ERROR_NO_MORE_ITEMS)
				throw new Win32Exception(err);
		}
		[DllImport("setupapi.dll", EntryPoint = "SetupDiEnumDeviceInfo", SetLastError = true)]
		private static extern bool SetupDiEnumDeviceInfo_(DevClassSafeHandle deviceInfoSet, int memberIndex, ref DeviceInfoData did);

		/// <summary>Opens a handle to a device information set that contains requested device information elements for a local computer.</summary>
		public static DevClassSafeHandle GetClassDevs(Guid class_dev, string? enumerator, IntPtr hwndParent, ESetupDiGetClassDevsFlags flags)
		{
			var handle = SetupDiGetClassDevs_(ref class_dev, enumerator, hwndParent, flags);
			if (handle == Win32.INVALID_HANDLE_VALUE) throw new Win32Exception();
			return new DevClassSafeHandle(handle, true);
		}
		[DllImport("setupapi.dll", EntryPoint = "SetupDiGetClassDevsW", CharSet = CharSet.Unicode, SetLastError = true)]
		private static extern IntPtr SetupDiGetClassDevs_(ref Guid classGuid, [MarshalAs(UnmanagedType.LPWStr)] string? enumerator, IntPtr hwndParent, ESetupDiGetClassDevsFlags flags);
		[DllImport("setupapi.dll", EntryPoint = "SetupDiGetClassDevsW", CharSet = CharSet.Unicode, SetLastError = true)]
		private static extern IntPtr SetupDiGetClassDevs_(IntPtr classGuid, [MarshalAs(UnmanagedType.LPWStr)] string? enumerator, IntPtr hwndParent, ESetupDiGetClassDevsFlags flags);

		/// <summary>Get the class name from a dev class guid</summary>
		public static string ClassNameFromGuid(Guid dev_class_guid)
		{
			var result = SetupDiClassNameFromGuid_(ref dev_class_guid, IntPtr.Zero, 0, out var required_size);
			if (result)
				return string.Empty;
			if (Marshal.GetLastWin32Error() != Win32.ERROR_INSUFFICIENT_BUFFER)
				throw new Win32Exception($"Failed to get device class from Guid: {dev_class_guid}");

			var sb = new StringBuilder(required_size);
			if (!SetupDiClassNameFromGuid_(ref dev_class_guid, sb, sb.Length, out required_size))
				throw new Win32Exception($"Failed to get device class from Guid: {dev_class_guid}");

			return sb.TrimEnd('\0').ToString();
		}
		[DllImport("setupapi.dll", EntryPoint= "SetupDiClassNameFromGuidW", CharSet = CharSet.Unicode, SetLastError = true)]
		private static extern bool SetupDiClassNameFromGuid_(ref Guid ClassGuid, StringBuilder className, int ClassNameSize, out int RequiredSize);
		[DllImport("setupapi.dll", EntryPoint = "SetupDiClassNameFromGuidW", SetLastError = true)]
		private static extern bool SetupDiClassNameFromGuid_(ref Guid ClassGuid, IntPtr className, int ClassNameSize, out int RequiredSize);

		/// <summary>Retrieves the GUID(s) associated with the specified class name. This list is built based on the classes currently installed on the system.</summary>
		public static Guid[] ClassGuidsFromName(string dev_class_name)
		{
			var result = SetupDiClassGuidsFromName_(dev_class_name, IntPtr.Zero, 0, out var required_size);
			if (result)
				return Array.Empty<Guid>();
			if (Marshal.GetLastWin32Error() != Win32.ERROR_INSUFFICIENT_BUFFER)
				throw new Win32Exception($"Failed to get device class Guids for: {dev_class_name}");

			var guids = new Guid[required_size];
			if (!SetupDiClassGuidsFromName_(dev_class_name, guids, guids.Length, out _))
				throw new Win32Exception($"Failed to get device class Guids for: {dev_class_name}");

			return guids;
		}
		[DllImport("setupapi.dll", EntryPoint = "ClassGuidsFromNameW", CharSet = CharSet.Unicode, SetLastError = true)]
		private static extern bool SetupDiClassGuidsFromName_([MarshalAs(UnmanagedType.LPWStr)] string ClassName, Guid[] ClassGuidArray1stItem, int ClassGuidArraySize, out int RequiredSize);
		[DllImport("setupapi.dll", EntryPoint = "ClassGuidsFromNameW", CharSet = CharSet.Unicode, SetLastError = true)]
		private static extern bool SetupDiClassGuidsFromName_([MarshalAs(UnmanagedType.LPWStr)] string ClassName, IntPtr ClassGuidArray1stItem, int ClassGuidArraySize, out int RequiredSize);

		/// <summary></summary>
		public static string GetDeviceInstanceId(DevClassSafeHandle handle, DeviceInfoData did)
		{
			var result = SetupDiGetDeviceInstanceId_(handle, ref did, IntPtr.Zero, 0, out var required_size);
			var err = Marshal.GetLastWin32Error();
			if (result || err == Win32.ERROR_NOT_FOUND)
				return string.Empty;
			if (err != Win32.ERROR_INSUFFICIENT_BUFFER)
				throw new Win32Exception($"Failed to get device instance id for: {did.ClassGuid}");

			var sb = new StringBuilder(required_size);
			if (!SetupDiGetDeviceInstanceId_(handle, ref did, sb, sb.Length, out required_size))
				throw new Win32Exception($"Failed to get device instance id for: {did.ClassGuid}");

			return sb.TrimEnd('\0').ToString();
		}
		[DllImport("setupapi.dll", EntryPoint = "SetupDiGetDeviceInstanceIdW", CharSet = CharSet.Unicode, SetLastError = true)]
		private static extern bool SetupDiGetDeviceInstanceId_(DevClassSafeHandle deviceInfoSet, ref DeviceInfoData did, [MarshalAs(UnmanagedType.LPWStr)] StringBuilder DeviceInstanceId, int DeviceInstanceIdSize, out int RequiredSize);
		[DllImport("setupapi.dll", EntryPoint = "SetupDiGetDeviceInstanceIdW", CharSet = CharSet.Unicode, SetLastError = true)]
		private static extern bool SetupDiGetDeviceInstanceId_(DevClassSafeHandle deviceInfoSet, ref DeviceInfoData did, IntPtr DeviceInstanceId, int DeviceInstanceIdSize, out int RequiredSize);

		/// <summary></summary>
		private static (EDevPropType type, byte[] prop) GetDeviceProperty(DevClassSafeHandle deviceInfoSet, DeviceInfoData did, DEVPROPKEY propertyKey)
		{
			var result = SetupDiGetDeviceProperty_(deviceInfoSet, ref did, ref propertyKey, out var type, IntPtr.Zero, 0, out var required_size, 0);
			var err = Marshal.GetLastWin32Error();
			if (result || err == Win32.ERROR_NOT_FOUND)
				return ((EDevPropType)type, Array.Empty<byte>());
			if (err != Win32.ERROR_INSUFFICIENT_BUFFER)
				throw new Win32Exception(err);

			var buf = new byte[required_size];
			if (!SetupDiGetDeviceProperty_(deviceInfoSet, ref did, ref propertyKey, out type, buf, buf.Length, out _, 0))
				throw new Win32Exception();

			return ((EDevPropType)type, buf);
		}
		[DllImport("setupapi.dll", EntryPoint = "SetupDiGetDevicePropertyW", SetLastError = true)]
		private static extern bool SetupDiGetDeviceProperty_(DevClassSafeHandle deviceInfoSet, [In] ref DeviceInfoData did, [In] ref DEVPROPKEY propertyKey, [Out] out uint propertyType, byte[] propertyBuffer, int propertyBufferSize, out int requiredSize, uint flags);
		[DllImport("setupapi.dll", EntryPoint = "SetupDiGetDevicePropertyW", SetLastError = true)]
		private static extern bool SetupDiGetDeviceProperty_(DevClassSafeHandle deviceInfoSet, [In] ref DeviceInfoData did, [In] ref DEVPROPKEY propertyKey, [Out] out uint propertyType, IntPtr propertyBuffer, int propertyBufferSize, out int requiredSize, uint flags);

		/// <summary>
		/// The SetupDiGetDeviceRegistryProperty function retrieves the specified device property.
		/// This handle is typically returned by the SetupDiGetClassDevs or SetupDiGetClassDevsEx function.</summary>
		/// <param Name="DeviceInfoSet">Handle to the device information set that contains the interface and its underlying device.</param>
		/// <param Name="DeviceInfoData">Pointer to an SP_DEVINFO_DATA structure that defines the device instance.</param>
		/// <param Name="Property">Device property to be retrieved. SEE MSDN</param>
		/// <param Name="PropertyRegDataType">Pointer to a variable that receives the registry data Type. This parameter can be NULL.</param>
		/// <param Name="PropertyBuffer">Pointer to a buffer that receives the requested device property.</param>
		/// <param Name="PropertyBufferSize">Size of the buffer, in bytes.</param>
		/// <param Name="RequiredSize">Pointer to a variable that receives the required buffer size, in bytes. This parameter can be NULL.</param>
		/// <returns>If the function succeeds, the return value is nonzero.</returns>
		public static (RegistryValueKind type, byte[] prop) GetDeviceRegistryProperty(DevClassSafeHandle handle, DeviceInfoData did, EDeviceRegistryProperty property)
		{
			var result = SetupDiGetDeviceRegistryProperty_(handle, ref did, property, out var type, IntPtr.Zero, 0, out var required_size);
			var err = Marshal.GetLastWin32Error();
			if (result || err == Win32.ERROR_NOT_FOUND)
				return ((RegistryValueKind)type, Array.Empty<byte>());
			if (err != Win32.ERROR_INSUFFICIENT_BUFFER)
				throw new Win32Exception(err);

			var buf = new byte[required_size];
			if (!SetupDiGetDeviceRegistryProperty_(handle, ref did, property, out type, buf, buf.Length, out _))
				throw new Win32Exception();

			return ((RegistryValueKind)type, buf);
		}
		[DllImport("setupapi.dll", EntryPoint = "SetupDiGetDeviceRegistryPropertyW", CharSet = CharSet.Unicode, SetLastError = true)]
		private static extern bool SetupDiGetDeviceRegistryProperty_(DevClassSafeHandle hDeviceInfoSet, ref DeviceInfoData DeviceInfoData, EDeviceRegistryProperty Property, out uint PropertyRegDataType, byte[] PropertyBuffer, int PropertyBufferSize, out int RequiredSize);
		[DllImport("setupapi.dll", EntryPoint = "SetupDiGetDeviceRegistryPropertyW", CharSet = CharSet.Unicode, SetLastError = true)]
		private static extern bool SetupDiGetDeviceRegistryProperty_(DevClassSafeHandle hDeviceInfoSet, ref DeviceInfoData DeviceInfoData, EDeviceRegistryProperty Property, out uint PropertyRegDataType, IntPtr PropertyBuffer, int PropertyBufferSize, out int RequiredSize);

		/// <summary></summary>
		public static RegistryKey OpenDevRegKey(DevClassSafeHandle handle, DeviceInfoData did, EScope scope, uint hwProfile, EDevKeyKind kind, RegistryRights access)
		{
			var key = SetupDiOpenDevRegKey_(handle, ref did, scope, hwProfile, kind, access);
			return RegistryKey.FromHandle(key);
		}
		[DllImport("setupapi.dll", EntryPoint = "SetupDiOpenDevRegKey", CharSet = CharSet.Unicode, SetLastError = true)]
		private static extern SafeRegistryHandle SetupDiOpenDevRegKey_(DevClassSafeHandle hDeviceInfoSet, ref DeviceInfoData did, EScope scope, uint hwProfile, EDevKeyKind kind, RegistryRights access);

		/// <summary></summary>
		[SuppressUnmanagedCodeSecurity()]
		[DllImport("setupapi.dll", EntryPoint = "SetupDiDestroyDeviceInfoList", SetLastError = true)]
		private static extern bool SetupDiDestroyDeviceInfoList(IntPtr deviceInfoSet);
		#endregion

		public class DevClassSafeHandle :SafeHandleZeroOrMinusOneIsInvalid
		{
			public DevClassSafeHandle(IntPtr handle, bool owns_handle) : base(owns_handle) => SetHandle(handle);
			protected override bool ReleaseHandle() => SetupDiDestroyDeviceInfoList(DangerousGetHandle());
		}
	}
}

#if PR_UNITTESTS
namespace Rylogic.UnitTests
{
	[TestFixture]
	public class TestDeviceManager
	{
		// For some reason, the window class for the dummywindow isn't registered during unit tests...
		[Test]
		public void EnumDevices()
		{
		//	var devices = DeviceManager.EnumDevices(DevClass.NetworkAdapter).ToList();
		//	foreach (var dev in devices)
		//		Debug.WriteLine(dev.StringDevProp(DeviceManager.DEVPROPKEY.Device_FriendlyName));
		}

		[Test]
		public void FindCommPort()
		{
		//	var devices = DeviceManager.EnumDevices(DevClass.Ports);
		//	foreach (var dev in devices)
		//	{
		//		var port = dev.StringRegProp("PortName");
		//		Debug.WriteLine(port);
		//	}
		}

		[Test]
		public void WatchForDevices()
		{
			//DeviceManager.HardwareChanged += HandleHWChanged;
			//void HandleHWChanged(object? sender, DeviceManager.HardwareChangedEventArgs e)
			//{
			//	Debug.WriteLine(e.Dump());
			//}
			//for (int i = 0; i != 100; ++i)
			//	Thread.Sleep(100);
		}
	}
}
#endif