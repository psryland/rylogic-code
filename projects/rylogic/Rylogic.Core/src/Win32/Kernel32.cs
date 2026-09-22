using System;
using System.Runtime.InteropServices;
using System.Threading;
using Microsoft.Win32.SafeHandles;

namespace Rylogic.Interop.Win32
{
	public static partial class Win32
	{
		public const int ATTACH_PARENT_PROCESS = -1;

		#region Access Constants
		public const uint DELETE = 0x00010000U;
		public const uint READ_CONTROL = 0x00020000U;
		public const uint WRITE_DAC = 0x00040000U;
		public const uint WRITE_OWNER = 0x00080000U;
		public const uint SYNCHRONIZE = 0x00100000U;

		public const uint STANDARD_RIGHTS_REQUIRED = 0x000F0000U;
		public const uint STANDARD_RIGHTS_READ = READ_CONTROL;
		public const uint STANDARD_RIGHTS_WRITE = READ_CONTROL;
		public const uint STANDARD_RIGHTS_EXECUTE = READ_CONTROL;
		public const uint STANDARD_RIGHTS_ALL = 0x001F0000U;
		public const uint SPECIFIC_RIGHTS_ALL = 0x0000FFFFU;
		#endregion

		#region CreateFile Constants
		[Flags]
		public enum EFileAccess :uint
		{
			NEITHER = 0,
			READ_DATA = 0x0001,    // file & pipe
			LIST_DIRECTORY = 0x0001,    // directory
			WRITE_DATA = 0x0002,    // file & pipe
			ADD_FILE = 0x0002,    // directory
			APPEND_DATA = 0x0004,    // file
			ADD_SUBDIRECTORY = 0x0004,    // directory
			CREATE_PIPE_INSTANCE = 0x0004,    // named pipe
			READ_EA = 0x0008,    // file & directory
			WRITE_EA = 0x0010,    // file & directory
			EXECUTE = 0x0020,    // file
			TRAVERSE = 0x0020,    // directory
			DELETE_CHILD = 0x0040,    // directory
			READ_ATTRIBUTES = 0x0080,    // all
			WRITE_ATTRIBUTES = 0x0100,    // all
			ALL_ACCESS = STANDARD_RIGHTS_REQUIRED | SYNCHRONIZE | 0x1FF,
			GENERIC_READ = (STANDARD_RIGHTS_READ | READ_DATA | READ_ATTRIBUTES | READ_EA | SYNCHRONIZE),
			GENERIC_WRITE = (STANDARD_RIGHTS_WRITE | WRITE_DATA | WRITE_ATTRIBUTES | WRITE_EA | APPEND_DATA | SYNCHRONIZE),
			GENERIC_EXECUTE = (STANDARD_RIGHTS_EXECUTE | READ_ATTRIBUTES | EXECUTE | SYNCHRONIZE),
		}
		[Flags]
		public enum EFileShare :uint
		{
			NONE = 0,
			READ = 0x00000001,
			WRITE = 0x00000002,
			DELETE = 0x00000004,
		}
		[Flags]
		public enum EFileFlag :uint
		{
			NONE = 0,
			READONLY = 0x00000001,
			HIDDEN = 0x00000002,
			SYSTEM = 0x00000004,
			DIRECTORY = 0x00000010,
			ARCHIVE = 0x00000020,
			DEVICE = 0x00000040,
			NORMAL = 0x00000080,
			TEMPORARY = 0x00000100,
			SPARSE_FILE = 0x00000200,
			REPARSE_POINT = 0x00000400,
			COMPRESSED = 0x00000800,
			OFFLINE = 0x00001000,
			NOT_CONTENT_INDEXED = 0x00002000,
			ENCRYPTED = 0x00004000,
			INTEGRITY_STREAM = 0x00008000,
			VIRTUAL = 0x00010000,
			NO_SCRUB_DATA = 0x00020000,
			EA = 0x00040000,
			RECALL_ON_OPEN = 0x00040000,
			PINNED = 0x00080000,
			FIRST_PIPE_INSTANCE = 0x00080000,
			UNPINNED = 0x00100000,
			OPEN_NO_RECALL = 0x00100000,
			OPEN_REPARSE_POINT = 0x00200000,
			RECALL_ON_DATA_ACCESS = 0x00400000,
			SESSION_AWARE = 0x00800000,
			POSIX_SEMANTICS = 0x01000000,
			BACKUP_SEMANTICS = 0x02000000,
			DELETE_ON_CLOSE = 0x04000000,
			SEQUENTIAL_SCAN = 0x08000000,
			RANDOM_ACCESS = 0x10000000,
			NO_BUFFERING = 0x20000000,
			OVERLAPPED = 0x40000000,
			WRITE_THROUGH = 0x80000000,
		}
		[Flags]
		public enum EFileFlagOther :uint
		{
			TREE_CONNECT_ATTRIBUTE_PRIVACY = 0x00004000,
			TREE_CONNECT_ATTRIBUTE_INTEGRITY = 0x00008000,
			TREE_CONNECT_ATTRIBUTE_GLOBAL = 0x00000004,
			TREE_CONNECT_ATTRIBUTE_PINNED = 0x00000002,
			FILE_ATTRIBUTE_STRICTLY_SEQUENTIAL = 0x20000000,
			FILE_NOTIFY_CHANGE_FILE_NAME = 0x00000001,
			FILE_NOTIFY_CHANGE_DIR_NAME = 0x00000002,
			FILE_NOTIFY_CHANGE_ATTRIBUTES = 0x00000004,
			FILE_NOTIFY_CHANGE_SIZE = 0x00000008,
			FILE_NOTIFY_CHANGE_LAST_WRITE = 0x00000010,
			FILE_NOTIFY_CHANGE_LAST_ACCESS = 0x00000020,
			FILE_NOTIFY_CHANGE_CREATION = 0x00000040,
			FILE_NOTIFY_CHANGE_SECURITY = 0x00000100,
			FILE_ACTION_ADDED = 0x00000001,
			FILE_ACTION_REMOVED = 0x00000002,
			FILE_ACTION_MODIFIED = 0x00000003,
			FILE_ACTION_RENAMED_OLD_NAME = 0x00000004,
			FILE_ACTION_RENAMED_NEW_NAME = 0x00000005,
			MAILSLOT_NO_MESSAGE = ~0x000000U,
			MAILSLOT_WAIT_FOREVER = ~0x000000U,
			FILE_CASE_SENSITIVE_SEARCH = 0x00000001,
			FILE_CASE_PRESERVED_NAMES = 0x00000002,
			FILE_UNICODE_ON_DISK = 0x00000004,
			FILE_PERSISTENT_ACLS = 0x00000008,
			FILE_FILE_COMPRESSION = 0x00000010,
			FILE_VOLUME_QUOTAS = 0x00000020,
			FILE_SUPPORTS_SPARSE_FILES = 0x00000040,
			FILE_SUPPORTS_REPARSE_POINTS = 0x00000080,
			FILE_SUPPORTS_REMOTE_STORAGE = 0x00000100,
			FILE_RETURNS_CLEANUP_RESULT_INFO = 0x00000200,
			FILE_SUPPORTS_POSIX_UNLINK_RENAME = 0x00000400,
			FILE_VOLUME_IS_COMPRESSED = 0x00008000,
			FILE_SUPPORTS_OBJECT_IDS = 0x00010000,
			FILE_SUPPORTS_ENCRYPTION = 0x00020000,
			FILE_NAMED_STREAMS = 0x00040000,
			FILE_READ_ONLY_VOLUME = 0x00080000,
			FILE_SEQUENTIAL_WRITE_ONCE = 0x00100000,
			FILE_SUPPORTS_TRANSACTIONS = 0x00200000,
			FILE_SUPPORTS_HARD_LINKS = 0x00400000,
			FILE_SUPPORTS_EXTENDED_ATTRIBUTES = 0x00800000,
			FILE_SUPPORTS_OPEN_BY_FILE_ID = 0x01000000,
			FILE_SUPPORTS_USN_JOURNAL = 0x02000000,
			FILE_SUPPORTS_INTEGRITY_STREAMS = 0x04000000,
			FILE_SUPPORTS_BLOCK_REFCOUNTING = 0x08000000,
			FILE_SUPPORTS_SPARSE_VDL = 0x10000000,
			FILE_DAX_VOLUME = 0x20000000,
			FILE_SUPPORTS_GHOSTING = 0x40000000,
		}
		public enum EFileCreation
		{
			CREATE_NEW = 1,
			CREATE_ALWAYS = 2,
			OPEN_EXISTING = 3,
			OPEN_ALWAYS = 4,
			TRUNCATE_EXISTING = 5,
		}
		#endregion

		#region Device IO
		public enum EDeviceType :int
		{
			// These are the 
			BEEP = 0x00000001,
			CD_ROM = 0x00000002,
			CD_ROM_FILE_SYSTEM = 0x00000003,
			CONTROLLER = 0x00000004,
			DATALINK = 0x00000005,
			DFS = 0x00000006,
			DISK = 0x00000007,
			DISK_FILE_SYSTEM = 0x00000008,
			FILE_SYSTEM = 0x00000009,
			INPORT_PORT = 0x0000000a,
			KEYBOARD = 0x0000000b,
			MAILSLOT = 0x0000000c,
			MIDI_IN = 0x0000000d,
			MIDI_OUT = 0x0000000e,
			MOUSE = 0x0000000f,
			MULTI_UNC_PROVIDER = 0x00000010,
			NAMED_PIPE = 0x00000011,
			NETWORK = 0x00000012,
			NETWORK_BROWSER = 0x00000013,
			NETWORK_FILE_SYSTEM = 0x00000014,
			NULL = 0x00000015,
			PARALLEL_PORT = 0x00000016,
			PHYSICAL_NETCARD = 0x00000017,
			PRINTER = 0x00000018,
			SCANNER = 0x00000019,
			SERIAL_MOUSE_PORT = 0x0000001a,
			SERIAL_PORT = 0x0000001b,
			SCREEN = 0x0000001c,
			SOUND = 0x0000001d,
			STREAMS = 0x0000001e,
			TAPE = 0x0000001f,
			TAPE_FILE_SYSTEM = 0x00000020,
			TRANSPORT = 0x00000021,
			UNKNOWN = 0x00000022,
			VIDEO = 0x00000023,
			VIRTUAL_DISK = 0x00000024,
			WAVE_IN = 0x00000025,
			WAVE_OUT = 0x00000026,
			FIRE_8042_PORT = 0x00000027,
			NETWORK_REDIRECTOR = 0x00000028,
			BATTERY = 0x00000029,
			BUS_EXTENDER = 0x0000002a,
			MODEM = 0x0000002b,
			VDM = 0x0000002c,
			MASS_STORAGE = 0x0000002d,
			SMB = 0x0000002e,
			KS = 0x0000002f,
			CHANGER = 0x00000030,
			SMARTCARD = 0x00000031,
			ACPI = 0x00000032,
			DVD = 0x00000033,
			FULLSCREEN_VIDEO = 0x00000034,
			DFS_FILE_SYSTEM = 0x00000035,
			DFS_VOLUME = 0x00000036,
			SERENUM = 0x00000037,
			TERMSRV = 0x00000038,
			KSEC = 0x00000039,
			FIPS = 0x0000003A,
			INFINIBAND = 0x0000003B,
			VMBUS = 0x0000003E,
			CRYPT_PROVIDER = 0x0000003F,
			WPD = 0x00000040,
			BLUETOOTH = 0x00000041,
			MT_COMPOSITE = 0x00000042,
			MT_TRANSPORT = 0x00000043,
			BIOMETRIC = 0x00000044,
			PMI = 0x00000045,
			EHSTOR = 0x00000046,
			DEVAPI = 0x00000047,
			GPIO = 0x00000048,
			USBEX = 0x00000049,
			CONSOLE = 0x00000050,
			NFP = 0x00000051,
			SYSENV = 0x00000052,
			VIRTUAL_BLOCK = 0x00000053,
			POINT_OF_SERVICE = 0x00000054,
			STORAGE_REPLICATION = 0x00000055,
			TRUST_ENV = 0x00000056,
			UCM = 0x00000057,
			UCMTCPCI = 0x00000058,
			PERSISTENT_MEMORY = 0x00000059,
			NVDIMM = 0x0000005a,
			HOLOGRAPHIC = 0x0000005b,
			SDFXHCI = 0x0000005c,
			UCMUCSI = 0x0000005d,
		}
		public enum EDeviceIoMethod
		{
			BUFFERED = 0,
			IN_DIRECT = 1,
			OUT_DIRECT = 2,
			NEITHER = 3,
			DIRECT_TO_HARDWARE = IN_DIRECT,
			DIRECT_FROM_HARDWARE = OUT_DIRECT,
		}
		public enum EDeviceAccess
		{
			ANY = 0,
			SPECIAL = ANY,
			READ = 0x0001,
			WRITE = 0x0002,
		}
		#endregion

		#region LoadLibrary Flags
		[Flags] public enum ELoadLibraryFlags :uint
		{
			/// <summary>
			/// If this value is used, and the executable module is a DLL, the system does not call DllMain for process and thread initialization and termination.
			/// Also, the system does not load additional executable modules that are referenced by the specified module. Note  Do not use this value; it is provided
			/// only for backward compatibility. If you are planning to access only data or resources in the DLL, use LOAD_LIBRARY_AS_DATAFILE_EXCLUSIVE or
			/// LOAD_LIBRARY_AS_IMAGE_RESOURCE or both. Otherwise, load the library as a DLL or executable module using the LoadLibrary function.</summary>
			DONT_RESOLVE_DLL_REFERENCES = 0x00000001,

			/// <summary>
			/// If this value is used, the system maps the file into the calling process's virtual address space as if it were a data file. Nothing is done to execute or
			/// prepare to execute the mapped file. Therefore, you cannot call functions like GetModuleFileName, GetModuleHandle or GetProcAddress with this DLL. Using this
			/// value causes writes to read-only memory to raise an access violation. Use this flag when you want to load a DLL only to extract messages or resources from it.
			/// This value can be used with LOAD_LIBRARY_AS_IMAGE_RESOURCE.For more information, see Remarks.</summary>
			LOAD_LIBRARY_AS_DATAFILE = 0x00000002,

			/// <summary>
			/// If this value is used and lpFileName specifies an absolute path, the system uses the alternate file search strategy discussed in the Remarks section to find associated executable
			/// modules that the specified module causes to be loaded. If this value is used and lpFileName specifies a relative path, the behavior is undefined. If this value is not used, or if
			/// lpFileName does not specify a path, the system uses the standard search strategy discussed in the Remarks section to find associated executable modules that the specified module
			/// causes to be loaded. This value cannot be combined with any LOAD_LIBRARY_SEARCH flag.</summary>
			LOAD_WITH_ALTERED_SEARCH_PATH = 0x00000008,

			/// <summary>
			/// If this value is used, the system does not check AppLocker rules or apply Software Restriction Policies for the DLL. This action applies only to the DLL
			/// being loaded and not to its dependencies. This value is recommended for use in setup programs that must run extracted DLLs during installation.
			/// Windows Server 2008 R2 and Windows 7:  On systems with KB2532445 installed, the caller must be running as "LocalSystem" or "TrustedInstaller"; otherwise
			/// the system ignores this flag.For more information, see "You can circumvent AppLocker rules by using an Office macro on a computer that is running Windows 7
			/// or Windows Server 2008 R2" in the Help and Support Knowledge Base at https://support.microsoft.com/kb/2532445.
			/// Windows Server 2008, Windows Vista, Windows Server 2003 and Windows XP:  AppLocker was introduced in Windows 7 and Windows Server 2008 R2.</summary>
			LOAD_IGNORE_CODE_AUTHZ_LEVEL = 0x00000010,

			/// <summary>
			/// If this value is used, the system maps the file into the process's virtual address space as an image file. However, the loader does not load the static imports or
			/// perform the other usual initialization steps. Use this flag when you want to load a DLL only to extract messages or resources from it. Unless the application depends
			/// on the file having the in-memory layout of an image, this value should be used with either LOAD_LIBRARY_AS_DATAFILE_EXCLUSIVE or LOAD_LIBRARY_AS_DATAFILE.
			/// For more information, see the Remarks section.
			/// Windows Server 2003 and Windows XP:  This value is not supported until Windows Vista.</summary>
			LOAD_LIBRARY_AS_IMAGE_RESOURCE = 0x00000020,

			/// <summary>
			/// Similar to LOAD_LIBRARY_AS_DATAFILE, except that the DLL file is opened with exclusive write access for the calling process. Other processes cannot open the DLL
			/// file for write access while it is in use.However, the DLL can still be opened by other processes. This value can be used with LOAD_LIBRARY_AS_IMAGE_RESOURCE.
			/// For more information, see Remarks.
			/// Windows Server 2003 and Windows XP:  This value is not supported until Windows Vista.</summary>
			LOAD_LIBRARY_AS_DATAFILE_EXCLUSIVE = 0x00000040,

			/// <summary>
			/// Specifies that the digital signature of the binary image must be checked at load time. This value requires Windows 8.1, Windows 10 or later.</summary>
			LOAD_LIBRARY_REQUIRE_SIGNED_TARGET = 0x00000080,

			/// <summary>
			/// If this value is used, the directory that contains the DLL is temporarily added to the beginning of the list of directories that are searched for the DLL's dependencies.
			/// Directories in the standard search path are not searched. The lpFileName parameter must specify a fully qualified path. This value cannot be combined with LOAD_WITH_ALTERED_SEARCH_PATH.
			/// For example, if Lib2.dll is a dependency of C:\Dir1\Lib1.dll, loading Lib1.dll with this value causes the system to search for Lib2.dll only in C:\Dir1. To search for Lib2.dll in
			/// C:\Dir1 and all of the directories in the DLL search path, combine this value with LOAD_LIBRARY_DEFAULT_DIRS.
			/// Windows 7, Windows Server 2008 R2, Windows Vista and Windows Server 2008:  This value requires KB2533623 to be installed.
			/// Windows Server 2003 and Windows XP:  This value is not supported.</summary>
			LOAD_LIBRARY_SEARCH_DLL_LOAD_DIR = 0x00000100,

			/// <summary>
			/// If this value is used, the application's installation directory is searched for the DLL and its dependencies. Directories in the standard search path are not searched.
			/// This value cannot be combined with LOAD_WITH_ALTERED_SEARCH_PATH.
			/// Windows 7, Windows Server 2008 R2, Windows Vista and Windows Server 2008:  This value requires KB2533623 to be installed.
			/// Windows Server 2003 and Windows XP:  This value is not supported.</summary>
			LOAD_LIBRARY_SEARCH_APPLICATION_DIR = 0x00000200,

			/// <summary>
			/// If this value is used, directories added using the AddDllDirectory or the SetDllDirectory function are searched for the DLL and its dependencies. If more than one directory
			/// has been added, the order in which the directories are searched is unspecified. Directories in the standard search path are not searched.This value cannot be combined with
			/// LOAD_WITH_ALTERED_SEARCH_PATH.
			/// Windows 7, Windows Server 2008 R2, Windows Vista and Windows Server 2008:  This value requires KB2533623 to be installed.
			/// Windows Server 2003 and Windows XP:  This value is not supported.</summary>
			LOAD_LIBRARY_SEARCH_USER_DIRS = 0x00000400,

			/// <summary>
			/// If this value is used, %windows%\system32 is searched for the DLL and its dependencies. Directories in the standard search path are not searched. This value cannot be combined
			/// with LOAD_WITH_ALTERED_SEARCH_PATH.
			/// Windows 7, Windows Server 2008 R2, Windows Vista and Windows Server 2008:  This value requires KB2533623 to be installed.
			/// Windows Server 2003 and Windows XP:  This value is not supported.</summary>
			LOAD_LIBRARY_SEARCH_SYSTEM32 = 0x00000800,

			/// <summary>
			/// This value is a combination of LOAD_LIBRARY_SEARCH_APPLICATION_DIR, LOAD_LIBRARY_SEARCH_SYSTEM32, and LOAD_LIBRARY_SEARCH_USER_DIRS. Directories in the standard search
			/// path are not searched. This value cannot be combined with LOAD_WITH_ALTERED_SEARCH_PATH. This value represents the recommended maximum number of directories an application
			/// should include in its DLL search path.
			/// Windows 7, Windows Server 2008 R2, Windows Vista and Windows Server 2008:  This value requires KB2533623 to be installed.
			/// Windows Server 2003 and Windows XP:  This value is not supported.</summary>
			LOAD_LIBRARY_SEARCH_DEFAULT_DIRS = 0x00001000,

			/// <summary>
			/// If this value is used, loading a DLL for execution from the current directory is only allowed if it is under a directory in the Safe load list.</summary>
			LOAD_LIBRARY_SAFE_CURRENT_DIRS = 0x00002000,
		}
		#endregion

		[StructLayout(LayoutKind.Sequential)]
		public struct LE_SCAN_REQUEST
		{
			public uint reserved0;

			/// <summary>0 = Passive, 1 = Active</summary>
			public int scanType;

			public uint reserved1;

			/// <summary>Interval in 0.625 ms units</summary>
			public ushort scanInterval;

			/// <summary>Interval in 0.625 ms units</summary>
			public ushort scanWindow;

			public uint reserved2;
			public uint reserved3;
		}
	}

	public static class Kernel32
	{
		/// <summary>Add a directory to the DLL search path</summary>
		public static IntPtr AddDllDirectory(string new_directory) => AddDllDirectory_(new_directory);
		[DllImport("kernel32.dll", EntryPoint = "AddDllDirectory", SetLastError = true)]
		private static extern IntPtr AddDllDirectory_([MarshalAs(UnmanagedType.LPWStr)] string new_directory);

		/// <summary>The native macros (CTL_CODE) used to encode commands passed to DeviceIoControl (see winioctl.h, bthioctl.h)</summary>
		public static uint CtlCodeEncode(Win32.EDeviceType device_type, uint function, Win32.EDeviceIoMethod method, Win32.EDeviceAccess access)
		{
			return ((uint)device_type << 16) | ((uint)access << 14) | (function << 2) | ((uint)method);
		}
		public static void CtlCodeDecode(uint code, out Win32.EDeviceType device_type, out uint function, out Win32.EDeviceIoMethod method, out Win32.EDeviceAccess access)
		{
			device_type = (Win32.EDeviceType)((code >> 16) & 0xFFFF);
			access = (Win32.EDeviceAccess)((code >> 14) & 0x3);
			function = (code >> 2) & 0xFFF;
			method = (Win32.EDeviceIoMethod)((code >> 0) & 0x3);
		}

		/// <summary></summary>
		public static bool AllocConsole() => AllocConsole_();
		[DllImport("kernel32.dll", EntryPoint = "AllocConsole", SetLastError = true)]
		private static extern bool AllocConsole_();

		/// <summary></summary>
		public static bool AttachConsole(int dwProcessId) => AttachConsole_(dwProcessId);
		[DllImport("kernel32.dll", EntryPoint = "AttachConsole", SetLastError = true)]
		private static extern bool AttachConsole_(int dwProcessId);

		/// <summary></summary>
		public static bool CancelIo(SafeFileHandle hDevice) => CancelIo_(hDevice);
		[DllImport("kernel32.dll", EntryPoint = "CancelIo", SetLastError = true)] 
		private static extern bool CancelIo_(SafeFileHandle hDevice);

		/// <summary></summary>
		public static SafeFileHandle CreateFile(string file_name, Win32.EFileAccess dwDesiredAccess, Win32.EFileShare dwShareMode, IntPtr SecurityAttributes, Win32.EFileCreation dwCreationDisposition, Win32.EFileFlag dwFlagsAndAttributes, IntPtr hTemplateFile)
		{
			return CreateFile_(file_name, (uint)dwDesiredAccess, (uint)dwShareMode, SecurityAttributes, (uint)dwCreationDisposition, (uint)dwFlagsAndAttributes, hTemplateFile);
		}
		[DllImport("kernel32.dll", EntryPoint = "CreateFileW", CharSet = CharSet.Unicode, SetLastError = true)]
		private static extern SafeFileHandle CreateFile_(string lpFileName, uint dwDesiredAccess, uint dwShareMode, IntPtr SecurityAttributes, uint dwCreationDisposition, uint dwFlagsAndAttributes, IntPtr hTemplateFile);

		/// <summary></summary>
		public static SafeFileHandle CreateEvent(IntPtr lpEventAttributes, bool bManualReset, bool bInitialState, string? lpName) => CreateEvent_(lpEventAttributes, bManualReset, bInitialState, lpName);
		[DllImport("kernel32.dll", EntryPoint = "CreateEventW", SetLastError = true, CharSet = CharSet.Auto)]
		private static extern SafeFileHandle CreateEvent_(IntPtr lpEventAttributes, bool bManualReset, bool bInitialState, [MarshalAs(UnmanagedType.LPWStr)] string? lpName);

		/// <summary>Create an unnamed waitable timer with the requested timing behaviour and access rights.</summary>
		public static SafeWaitHandle CreateWaitableTimerEx(IntPtr timer_attributes, string? timer_name, Win32.EWaitableTimerCreateFlags flags, Win32.ETimerAccess desired_access)
		{
			// Preserve SafeHandle lifetime management for every timer created through this wrapper.
			return CreateWaitableTimerEx_(timer_attributes, timer_name, flags, desired_access);
		}
		/// <summary>Native waitable-timer creation entry point.</summary>
		[DllImport("kernel32.dll", EntryPoint = "CreateWaitableTimerExW", SetLastError = true, CharSet = CharSet.Unicode)]
		private static extern SafeWaitHandle CreateWaitableTimerEx_(IntPtr lpTimerAttributes, string? lpTimerName, Win32.EWaitableTimerCreateFlags dwFlags, Win32.ETimerAccess dwDesiredAccess);

		/// <summary></summary>
		public static bool DeviceIoControl(SafeFileHandle hDevice, uint dwIoControlCode, ref Win32.LE_SCAN_REQUEST ble_scan_request, IntPtr lpOutBuffer, uint nOutBufferSize, out uint lpBytesReturned, NativeOverlapped? overlapped)
		{
			var ovr = overlapped ?? new NativeOverlapped();
			return DeviceIoControl_(hDevice, dwIoControlCode, ref ble_scan_request, (uint)Marshal.SizeOf<Win32.LE_SCAN_REQUEST>(), lpOutBuffer, nOutBufferSize, out lpBytesReturned, ref ovr);
		}
		[DllImport("kernel32.dll", EntryPoint = "DeviceIoControl", SetLastError = true, CharSet = CharSet.Auto)]
		private static extern bool DeviceIoControl_(SafeFileHandle hDevice, uint dwIoControlCode, ref Win32.LE_SCAN_REQUEST lpInBuffer, uint nInBufferSize, IntPtr lpOutBuffer, uint nOutBufferSize, out uint lpBytesReturned, ref NativeOverlapped lpOverlapped);
		[DllImport("kernel32.dll", EntryPoint = "DeviceIoControl", SetLastError = true, CharSet = CharSet.Auto)]
		private static extern bool DeviceIoControl_(SafeFileHandle hDevice, uint dwIoControlCode, IntPtr lpInBuffer, uint nInBufferSize, IntPtr lpOutBuffer, uint nOutBufferSize, out uint lpBytesReturned, ref NativeOverlapped lpOverlapped);

		/// <summary></summary>
		public static bool FileTimeToSystemTime(ref Win32.FILETIME ft, out Win32.SYSTEMTIME st) => FileTimeToSystemTime(ref ft, out st);
		[DllImport("kernel32.dll", EntryPoint = "FileTimeToSystemTime", SetLastError = true)]
		private static extern bool FileTimeToSystemTime_([In] ref Win32.FILETIME ft, [Out] out Win32.SYSTEMTIME st);

		/// <summary></summary>
		public static SafeFindHandle FindFirstFile(string fileName, ref Win32.WIN32_FIND_DATA data) => FindFirstFile_(fileName, ref data);
		[DllImport("kernel32.dll", EntryPoint = "FindFirstFile", CharSet = CharSet.Unicode, SetLastError = true)]
		private static extern SafeFindHandle FindFirstFile_(string fileName, ref Win32.WIN32_FIND_DATA data);

		/// <summary></summary>
		public static bool FindNextFile(SafeFindHandle hndFindFile, ref Win32.WIN32_FIND_DATA lpFindFileData) => FindNextFile_(hndFindFile, ref lpFindFileData);
		[DllImport("kernel32.dll", EntryPoint = "FindNextFile", CharSet = CharSet.Auto, SetLastError = true)]
		private static extern bool FindNextFile_(SafeFindHandle hndFindFile, ref Win32.WIN32_FIND_DATA lpFindFileData);

		/// <summary></summary>
		public static bool FindClose(IntPtr handle) => FindClose_(handle);
		[DllImport("kernel32.dll", EntryPoint = "FindClose")]
		private static extern bool FindClose_(IntPtr handle);

		/// <summary></summary>
		public static uint FormatMessage(uint dwFlags, IntPtr lpSource, uint dwMessageId, uint dwLanguageId, ref IntPtr lpBuffer, uint nSize, IntPtr pArguments) => FormatMessage_(dwFlags, lpSource, dwMessageId, dwLanguageId, ref lpBuffer, nSize, pArguments);
		[DllImport("kernel32.dll", EntryPoint = "FormatMessage", SetLastError = true)]
		private static extern uint FormatMessage_(uint dwFlags, IntPtr lpSource, uint dwMessageId, uint dwLanguageId, ref IntPtr lpBuffer, uint nSize, IntPtr pArguments);

		/// <summary></summary>
		public static bool FreeConsole() => FreeConsole_();
		[DllImport("kernel32.dll", EntryPoint = "FreeConsole", SetLastError = true)]
		private static extern bool FreeConsole_();

		/// <summary></summary>
		public static bool FreeLibrary(IntPtr module) => FreeLibrary_(module);
		[DllImport("Kernel32.dll", EntryPoint = "FreeLibrary", SetLastError = true)]
		private static extern bool FreeLibrary_(IntPtr module);

		/// <summary></summary>
		public static bool GetFileInformationByHandle(IntPtr hFile, out Win32.BY_HANDLE_FILE_INFORMATION lpFileInformation) => GetFileInformationByHandle_(hFile, out lpFileInformation);
		[DllImport("kernel32.dll", EntryPoint = "GetFileInformationByHandle", SetLastError = true)]
		private static extern bool GetFileInformationByHandle_(IntPtr hFile, out Win32.BY_HANDLE_FILE_INFORMATION lpFileInformation);
		
		/// <summary></summary>
		public static IntPtr GetModuleHandle(string? lpModuleName) => GetModuleHandle_(lpModuleName);
		[DllImport("Kernel32.dll", EntryPoint = "GetModuleHandleW", SetLastError = true)]
		private static extern IntPtr GetModuleHandle_([MarshalAs(UnmanagedType.LPWStr)] string? lpModuleName);

		/// <summary></summary>
		public static IntPtr LoadLibrary(string path) => LoadLibrary_(path);
		[DllImport("Kernel32.dll", EntryPoint = "LoadLibraryW", SetLastError = true)]
		private static extern IntPtr LoadLibrary_([MarshalAs(UnmanagedType.LPWStr)]string path);

		/// <summary></summary>
		public static IntPtr LoadLibraryEx(string path, IntPtr hFile, Win32.ELoadLibraryFlags flags) => LoadLibraryEx_(path, hFile, flags);
		[DllImport("Kernel32.dll", EntryPoint = "LoadLibraryExW", SetLastError = true)]
		private static extern IntPtr LoadLibraryEx_([MarshalAs(UnmanagedType.LPWStr)] string path, IntPtr hFile, Win32.ELoadLibraryFlags flags);

		/// <summary></summary>
		public static IntPtr GetProcAddress(IntPtr module, string proc_name) => GetProcAddress_(module, proc_name);
		[DllImport("kernel32", EntryPoint = "GetProcAddress", SetLastError = true, CharSet = CharSet.Ansi)]
		private static extern IntPtr GetProcAddress_(IntPtr hModule, string lpProcName);

		/// <summary>Remove a DLL directory from the search path that was added using 'AddDllDirectory'</summary>
		public static bool RemoveDllDirectory(IntPtr directory_cookie) => RemoveDllDirectory_(directory_cookie);
		[DllImport("kernel32.dll", EntryPoint = "RemoveDllDirectory", SetLastError = true)]
		private static extern bool RemoveDllDirectory_(IntPtr directory_cookie);

		/// <summary></summary>
		public static SafeFileHandle ReOpenFile(SafeFileHandle hOriginalFile, Win32.EFileAccess dwDesiredAccess, Win32.EFileShare dwShareMode, Win32.EFileFlag dwFlagsAndAttributes) => ReOpenFile_(hOriginalFile, (uint)dwDesiredAccess, (uint)dwShareMode, (uint)dwFlagsAndAttributes);
		[DllImport("kernel32.dll", EntryPoint = "ReOpenFile", SetLastError = true)]
		private static extern SafeFileHandle ReOpenFile_(SafeFileHandle hOriginalFile, uint dwDesiredAccess, uint dwShareMode, uint dwFlagsAndAttributes);

		/// <summary></summary>
		public static void SetLastError(int err) => SetLastError_(err);
		[DllImport("kernel32.dll", EntryPoint = "SetLastError", SetLastError = true)]
		private static extern void SetLastError_(int err);

		/// <summary>Arm a waitable timer using an absolute positive or relative negative 100-nanosecond due time.</summary>
		public static bool SetWaitableTimer(SafeWaitHandle timer, ref long due_time, int period_milliseconds, IntPtr completion_routine, IntPtr completion_argument, bool resume)
		{
			// Pass the due-time storage by reference because Win32 reads the signed LARGE_INTEGER value synchronously.
			return SetWaitableTimer_(timer, ref due_time, period_milliseconds, completion_routine, completion_argument, resume);
		}
		/// <summary>Native waitable-timer arming entry point.</summary>
		[DllImport("kernel32.dll", EntryPoint = "SetWaitableTimer", SetLastError = true)]
		private static extern bool SetWaitableTimer_(SafeWaitHandle hTimer, ref long lpDueTime, int lPeriod, IntPtr pfnCompletionRoutine, IntPtr lpArgToCompletionRoutine, bool fResume);

		/// <summary></summary>
		public static Win32.ExecutionState SetThreadExecutionState(Win32.ExecutionState esFlags) => SetThreadExecutionState_(esFlags);
		[DllImport("kernel32.dll", EntryPoint = "SetThreadExecutionState", SetLastError = true)]
		private static extern Win32.ExecutionState SetThreadExecutionState_(Win32.ExecutionState esFlags);

		/// <summary></summary>
		public static bool SystemTimeToFileTime([In] ref Win32.SYSTEMTIME st, [Out] out Win32.FILETIME ft) => SystemTimeToFileTime_(ref st, out ft);
		[DllImport("kernel32.dll", EntryPoint = "SystemTimeToFileTime", SetLastError = true)]
		private static extern bool SystemTimeToFileTime_([In] ref Win32.SYSTEMTIME st, [Out] out Win32.FILETIME ft);

		/// <summary></summary>
		public static bool WriteConsole(IntPtr hConsoleOutput, [MarshalAs(UnmanagedType.LPWStr)] string lpBuffer, uint nNumberOfCharsToWrite, out uint lpNumberOfCharsWritten, IntPtr lpReserved) => WriteConsole_(hConsoleOutput, lpBuffer, nNumberOfCharsToWrite, out lpNumberOfCharsWritten, lpReserved);
		[DllImport("kernel32.dll", EntryPoint = "WriteConsoleW", SetLastError = true)]
		private static extern bool WriteConsole_(IntPtr hConsoleOutput, [MarshalAs(UnmanagedType.LPWStr)] string lpBuffer, uint nNumberOfCharsToWrite, out uint lpNumberOfCharsWritten, IntPtr lpReserved);
	}

	/// <summary>Wraps a FindFirstFile handle.</summary>
	public sealed class SafeFindHandle : SafeHandleZeroOrMinusOneIsInvalid
	{
		public SafeFindHandle() : base(true) { }
		protected override bool ReleaseHandle() { return Kernel32.FindClose(handle); }
	}
}
