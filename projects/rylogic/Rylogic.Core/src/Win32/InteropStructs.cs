using System;
using System.Drawing;
using System.IO;
using System.Runtime.InteropServices;
using HWND     = System.IntPtr;
using HCURSOR  = System.IntPtr;
using UINT     = System.UInt32;
using WORD     = System.UInt16;
using DWORD    = System.UInt32;
using COLORREF = System.UInt32;
using WPARAM   = System.IntPtr;
using LPARAM   = System.IntPtr;

namespace Rylogic.Interop.Win32
{
	public static partial class Win32
	{
		#pragma warning disable IDE1006 // Naming Styles

		[StructLayout(LayoutKind.Sequential)]
		public struct BY_HANDLE_FILE_INFORMATION
		{
			public uint FileAttributes;
			public System.Runtime.InteropServices.ComTypes.FILETIME CreationTime;
			public System.Runtime.InteropServices.ComTypes.FILETIME LastAccessTime;
			public System.Runtime.InteropServices.ComTypes.FILETIME LastWriteTime;
			public uint VolumeSerialNumber;
			public uint FileSizeHigh;
			public uint FileSizeLow;
			public uint NumberOfLinks;
			public uint FileIndexHigh;
			public uint FileIndexLow;
		};

		[StructLayout(LayoutKind.Sequential, CharSet = CharSet.Auto, Pack = 4)]
		public struct CHARRANGE(int mn, int mx)
		{
			/// <summary>First character of range (0 for start of doc)</summary>
			public int min = mn;

			/// <summary>Last character of range (-1 for end of doc)</summary>
			public int max = mx;
		}

		[StructLayout(LayoutKind.Sequential, CharSet = CharSet.Auto, Pack = 4)]
		public struct COLORSCHEME
		{
			public DWORD    dwSize;
			public COLORREF clrBtnHighlight;       // highlight color
			public COLORREF clrBtnShadow;          // shadow color
		}

		[StructLayout(LayoutKind.Sequential, CharSet=CharSet.Auto, Pack=1)] 
		public struct COMBOBOXINFO
		{
			public Int32 cbSize;
			public RECT rcItem;
			public RECT rcButton;
			public int buttonState;
			public IntPtr hwndCombo;
			public IntPtr hwndEdit;
			public IntPtr hwndList;
		}

		[StructLayout(LayoutKind.Sequential, CharSet = CharSet.Auto, Pack = 4)]
		public struct COMDLG_FILTERSPEC
		{
			[MarshalAs(UnmanagedType.LPWStr)] public string pszName;
			[MarshalAs(UnmanagedType.LPWStr)] public string pszSpec;
		}

		[StructLayout(LayoutKind.Sequential, CharSet = CharSet.Unicode)]
		public struct CREATESTRUCT
		{
			public IntPtr lpCreateParams;
			public IntPtr hInstance;
			public IntPtr hMenu;
			public HWND hwndParent;
			public int cy;
			public int cx;
			public int y;
			public int x;
			public int style;
			[MarshalAs(UnmanagedType.LPWStr)] public string lpszName;
			[MarshalAs(UnmanagedType.LPWStr)] public string lpszClass;
			public uint dwExStyle;
		}
		
		[StructLayout(LayoutKind.Sequential)]
		public struct CURSORINFO
		{
			public DWORD cbSize;
			public DWORD flags;
			public HCURSOR hCursor;
			public POINT ptScreenPos;

			public static CURSORINFO Default => new() { cbSize = (uint)Marshal.SizeOf(typeof(CURSORINFO)) };
		}

		[StructLayout(LayoutKind.Sequential)]
		public struct DEV_BROADCAST_HDR
		{
			/// <summary>
			/// The size of this structure, in bytes. If this is a user-defined event, this member must be the size of
			/// this header, plus the size of the variable-length data in the _DEV_BROADCAST_USERDEFINED structure.</summary>
			public int dbch_size;
			public EDeviceBroadcaseType dbch_devicetype;
			public int dbch_reserved;
		}

		[StructLayout(LayoutKind.Sequential, CharSet = CharSet.Unicode)]
		public struct DEV_BROADCAST_DEVICEINTERFACE
		{
			private const int NameLen = 255;

			public DEV_BROADCAST_HDR hdr;
			public Guid class_guid;

			/// <summary></summary>
			public string Name
			{
				readonly get => m_name != null && Array.IndexOf(m_name, '\0') is int end && end != -1 ? new string(m_name, 0, end) : string.Empty;
				set
				{
					m_name = new char[NameLen];
					var len = Math.Min(value.Length, 255);
					Array.Copy(value.ToCharArray(), m_name, len);
				}
			}
			[MarshalAs(UnmanagedType.ByValArray, SizeConst = NameLen)] private char[]? m_name;
		}

		[StructLayout(LayoutKind.Explicit)]
		public struct FILETIME
		{
			[FieldOffset(0)] public DWORD dwLowDateTime;
			[FieldOffset(4)] public DWORD dwHighDateTime;
			[FieldOffset(0)] public long value;
		}

		[StructLayout(LayoutKind.Sequential)]
		public struct FLASHWINFO
		{
			/// <summary>The size of the structure in bytes.</summary>
			public uint cbSize;
			/// <summary>A Handle to the Window to be Flashed. The window can be either opened or minimized.</summary>
			public IntPtr hwnd;
			/// <summary>The Flash Status.</summary>
			public uint dwFlags;
			/// <summary>The number of times to Flash the window.</summary>
			public uint uCount;
			/// <summary>The rate at which the Window is to be flashed, in milliseconds. If Zero, the function uses the default cursor blink rate.</summary>
			public uint dwTimeout;
		}

		[StructLayout(LayoutKind.Sequential, CharSet = CharSet.Auto, Pack = 4)]
		public struct FORMATRANGE
		{
			/// <summary>DC to draw on</summary>
			public IntPtr hdc;

			/// <summary>Target DC for determining text formatting</summary>
			public IntPtr hdcTarget;

			/// <summary>Region of the DC to draw to (in twips)</summary>
			public RECT rc;

			/// <summary>Region of the whole DC (page size) (in twips)</summary>
			public RECT rcPage;

			/// <summary>Range of text to draw (see earlier declaration)</summary>
			public CHARRANGE char_range;
		}

		[StructLayout(LayoutKind.Sequential, CharSet = CharSet.Unicode, Pack = 4)]
		public struct ICONINFO
		{
			public bool fIcon;
			public int xHotspot;
			public int yHotspot;
			public IntPtr hbmMask;
			public IntPtr hbmColor;
		}

		[StructLayout(LayoutKind.Sequential)]
		public struct INITCOMMONCONTROLSEX
		{
			public DWORD dwSize; // size of this structure
			public DWORD dwICC; // flags indicating which classes to be initialized
		}

		[StructLayout(LayoutKind.Sequential, CharSet = CharSet.Auto, Pack = 4)]
		public struct KNOWNFOLDER_DEFINITION
		{
			public KF_CATEGORY category;
			[MarshalAs(UnmanagedType.LPWStr)] public string pszName;
			[MarshalAs(UnmanagedType.LPWStr)] public string pszCreator;
			[MarshalAs(UnmanagedType.LPWStr)] public string pszDescription;
			public Guid fidParent;
			[MarshalAs(UnmanagedType.LPWStr)] public string pszRelativePath;
			[MarshalAs(UnmanagedType.LPWStr)] public string pszParsingName;
			[MarshalAs(UnmanagedType.LPWStr)] public string pszToolTip;
			[MarshalAs(UnmanagedType.LPWStr)] public string pszLocalizedName;
			[MarshalAs(UnmanagedType.LPWStr)] public string pszIcon;
			[MarshalAs(UnmanagedType.LPWStr)] public string pszSecurity;
			public uint dwAttributes;
			public KF_DEFINITION_FLAGS kfdFlags;
			public Guid ftidType;
		}

		[StructLayout(LayoutKind.Sequential)]
		public struct MESSAGE
		{
			public HWND hwnd;
			public UINT message;
			public WPARAM wparam;
			public LPARAM lparam;
			public DWORD time;
			public POINT pt;
			public DWORD lPrivate;
		}

		[StructLayout(LayoutKind.Sequential)]
		public struct MINMAXINFO
		{
			// Structure pointed to by WM_GETMINMAXINFO lParam
			public POINT ptReserved;
			public POINT ptMaxSize;
			public POINT ptMaxPosition;
			public POINT ptMinTrackSize;
			public POINT ptMaxTrackSize;
		}

		[StructLayout(LayoutKind.Sequential, CharSet = CharSet.Unicode)]
		public struct MONITORINFOEX
		{
			private const int CCHDEVICENAME = 32;
			public DWORD cbSize; // = sizeof(MONITORINFOEX)
			public RECT rcMonitor;
			public RECT rcWork;
			public DWORD dwFlags;
			[MarshalAs(UnmanagedType.ByValTStr, SizeConst = CCHDEVICENAME)] public string szDevice;
		}

		[StructLayout(LayoutKind.Sequential, CharSet = CharSet.Unicode)]
		public struct NOTIFYICONDATA
		{
			/// <summary>Size of this structure, in bytes.</summary>
			public uint cbSize;

			/// <summary>
			/// Handle to the window that receives notification messages associated with an icon in the
			/// task bar status area. The Shell uses hWnd and uID to identify which icon to operate on
			/// when Shell_NotifyIcon is invoked.</summary>
			public IntPtr hWnd; // WindowHandle

			/// <summary>
			/// Application-defined identifier of the task bar icon. The Shell uses hWnd and uID to identify
			/// which icon to operate on when Shell_NotifyIcon is invoked. You can have multiple icons
			/// associated with a single hWnd by assigning each a different uID. This feature, however
			/// is currently not used.</summary>
			public uint uID; // TaskbarIconId

			/// <summary>
			/// Flags that indicate which of the other members contain valid data.
			/// This member can be a combination of the NIF_XXX constants.</summary>
			public ENotifyIconDataMembers uFlags; // ValidMembers

			/// <summary>
			/// Application-defined message identifier. The system uses this identifier to send
			/// notifications to the window identified in hWnd.</summary>
			public uint uCallbackMessage;

			/// <summary>A handle to the icon that should be displayed.</summary>
			public IntPtr hIcon; // IconHandle

			/// <summary>
			/// String with the text for a standard ToolTip. It can have a maximum of 64 characters including
			/// the terminating NULL. For Version 5.0 and later, szTip can have a maximum of 128 characters, including the terminating NULL.</summary>
			[MarshalAs(UnmanagedType.ByValTStr, SizeConst = 128)]
			public string szTip; // ToolTipText

			/// <summary>State of the icon. Remember to also set the 'StateMask'</summary>
			public EIconState IconState;

			/// <summary>
			/// A value that specifies which bits of the state member are retrieved or modified.
			/// For example, setting this member to 'Hidden' causes only the item's hidden state to be retrieved.</summary>
			public EIconState StateMask;

			/// <summary>
			/// String with the text for a balloon ToolTip. It can have a maximum of 255 characters.
			/// To remove the ToolTip, set the NIF_INFO flag in uFlags and set szInfo to an empty string.</summary>
			[MarshalAs(UnmanagedType.ByValTStr, SizeConst = 256)]
			public string szInfo; // BalloonText

			/// <summary>
			/// Mainly used to set the version when 'Shell_NotifyIcon' is invoked with 'NotifyCommand.SetVersion'.
			/// However, for legacy operations, the same member is also used to set timeouts for balloon ToolTips.</summary>
			public uint VersionOrTimeout;

			/// <summary>
			/// String containing a title for a balloon ToolTip.
			/// This title appears in boldface above the text. It can have a maximum of 63 characters.</summary>
			[MarshalAs(UnmanagedType.ByValTStr, SizeConst = 64)]
			public string szInfoTitle; // BalloonTitle

			/// <summary>
			/// Adds an icon to a balloon ToolTip, which is placed to the left of the title.
			/// If the 'szInfoTitle' member is zero-length, the icon is not shown.</summary>
			public ENotifyIconBalloonFlags dwInfoFlags; // BalloonFlags

			/// <summary>
			/// Windows XP (Shell32.dll version 6.0) and later.<br/>
			/// - Windows 7 and later: A registered GUID that identifies the icon.
			///   This value overrides uID and is the recommended method of identifying the icon.<br/>
			/// - Windows XP through Windows Vista: Reserved.</summary>
			public Guid guidItem; // TaskbarIconGuid

			/// <summary>
			/// Windows Vista (Shell32.dll version 6.0.6) and later. The handle of a customized balloon icon provided
			/// by the application that should be used independently of the tray icon. If this member is non-NULL and
			/// the 'BalloonFlags.User' flag is set, this icon is used as the balloon icon.<br/>
			/// If this member is NULL, the legacy behavior is carried out.</summary>
			public IntPtr hBalloonIcon; // CustomBalloonIconHandle

			/// <summary>Creates a default data structure that provides a hidden task bar icon without the icon being set.</summary>
			public static NOTIFYICONDATA New(IntPtr handle, uint callback_message, ENotifyIconVersion version)
			{
				var data = new NOTIFYICONDATA();

				// Need to set another size on xp/2003- otherwise certain features (e.g. balloon tooltips) don't work.
				data.cbSize = Environment.OSVersion.Version.Major >= 6
					? (uint)Marshal.SizeOf(data) // Use the current size
					: 952; // NOTIFYICONDATAW_V3_SIZE

				// Set to fixed timeout for old versions
				if (Environment.OSVersion.Version.Major < 6)
					data.VersionOrTimeout = 10;

				data.hWnd = handle;
				data.uID = 0x0;
				data.uCallbackMessage = callback_message;
				data.VersionOrTimeout = (uint)version;
				data.hIcon = IntPtr.Zero;
				data.IconState = EIconState.Hidden;
				data.StateMask = EIconState.Hidden;
				data.uFlags = ENotifyIconDataMembers.Message | ENotifyIconDataMembers.Icon | ENotifyIconDataMembers.Tip;
				data.szTip = string.Empty;
				data.szInfo = string.Empty;
				data.szInfoTitle = string.Empty;
				return data;
			}
		}

		[StructLayout(LayoutKind.Sequential)]
		public struct NMHDR
		{
			// Notification message (WM_NOTIFY) header
			public HWND hwndFrom;
			public UINT idFrom;
			public UINT code; // NM_code
		}

		[StructLayout(LayoutKind.Sequential, CharSet = CharSet.Auto, Pack = 4)]
		public struct PBRANGE
		{
			public int iLow;
			public int iHigh;
		}

		[StructLayout(LayoutKind.Sequential)]
		public struct POINT
		{
			public int X;
			public int Y;
			public readonly Point ToPoint() => new(X, Y);
			public static POINT FromPoint(Point pt) => new() { X = pt.X, Y = pt.Y };
			public static implicit operator Point(POINT pt) => pt.ToPoint();
		}

		[StructLayout(LayoutKind.Sequential, Pack = 4)]
		public struct PROPERTYKEY
		{
			public Guid fmtid;
			public uint pid;
		}

		[StructLayout(LayoutKind.Sequential)]
		public struct RAWINPUTDEVICE
		{
			public ushort usUsagePage;
			public ushort usUsage;
			public uint dwFlags;
			public IntPtr hwndTarget;
		}

		[StructLayout(LayoutKind.Sequential)]
		public struct RAWINPUTHEADER
		{
			public uint dwType;
			public uint dwSize;
			public IntPtr hDevice;
			public IntPtr wParam;
		}

		[StructLayout(LayoutKind.Explicit)]
		public struct RAWMOUSE
		{
			// The native RAWMOUSE has a union { ULONG ulButtons; struct { USHORT usButtonFlags; USHORT usButtonData; }; }
			// which starts at offset 4 (2-byte padding after usFlags for ULONG alignment).
			[FieldOffset(0)] public ushort usFlags;
			[FieldOffset(4)] public uint ulButtons;
			[FieldOffset(4)] public ushort usButtonFlags;
			[FieldOffset(6)] public ushort usButtonData;
			[FieldOffset(8)] public uint ulRawButtons;
			[FieldOffset(12)] public int lLastX;
			[FieldOffset(16)] public int lLastY;
			[FieldOffset(20)] public uint ulExtraInformation;
		}
		
		[StructLayout(LayoutKind.Sequential)]
		public struct RAWKEYBOARD
		{
			public ushort MakeCode;
			public ushort Flags;
			public ushort Reserved;
			public ushort VKey;
			public uint Message;
			public uint ExtraInformation;
		}

		[StructLayout(LayoutKind.Sequential)]
		public struct RAWHID
		{
			public uint dwSizeHid;    // byte size of each report
			public uint dwCount;      // number of input packed
			public byte bRawData;
		}

		[StructLayout(LayoutKind.Sequential)]
		public struct RAWINPUT
		{
			public RAWINPUTHEADER header;

			// The union (mouse/keyboard/hid) follows the header immediately.
			// Can't use FieldOffset because RAWINPUTHEADER size differs between x86 (16) and x64 (24).
			// Use ReadData<T>(IntPtr) to read the union member at the correct offset.
			public static T ReadData<T>(IntPtr pRawInput) where T : struct
			{
				var offset = Marshal.SizeOf<RAWINPUTHEADER>();
				return Marshal.PtrToStructure<T>(pRawInput + offset);
			}
		}

		[StructLayout(LayoutKind.Sequential)]
		public struct RECT
		{
			public int left;
			public int top;
			public int right;
			public int bottom;

			public readonly int width => right - left;
			public readonly int height => bottom - top;

			public readonly Size ToSize()                           => new(right - left, bottom - top);
			public readonly Rectangle ToRectangle()                 => new(left, top, width, height);
			public static RECT FromRectangle(Rectangle rect)        => new() { left = rect.Left, top = rect.Top, right = rect.Right, bottom = rect.Bottom };
			public static RECT FromSize(Size size)                  => new() { left = 0, top = 0, right = size.Width, bottom = size.Height };
			public static RECT FromLTRB(int l, int t, int r, int b) => new() { left = l, top = t, right = r, bottom = b };
		};

		[StructLayout(LayoutKind.Sequential)]
		public struct SCROLLINFO
		{
			public uint cbSize;
			public uint fMask;
			public int nMin;
			public int nMax;
			public uint nPage;
			public int nPos;
			public int nTrackPos;

			public static SCROLLINFO Default => new() { cbSize = (uint)Marshal.SizeOf(typeof(SCROLLINFO)) };
		}

		[StructLayout(LayoutKind.Sequential, CharSet = CharSet.Unicode, Pack = 4)]
		public struct SHFILEOPSTRUCTW32
		{
			public IntPtr hwnd;
			public UInt32 wFunc;
			public IntPtr pFrom; // Must be a double null terminated string, can't use strings because interop drops the double null
			public IntPtr pTo;   // Must be a double null terminated string, can't use strings because interop drops the double null
			public UInt16 fFlags;
			public Int32 fAnyOperationsAborted;
			public IntPtr hNameMappings;
			[MarshalAs(UnmanagedType.LPWStr)] public string lpszProgressTitle;
		}

		[StructLayout(LayoutKind.Sequential, CharSet = CharSet.Unicode, Pack = 8)]
		public struct SHFILEOPSTRUCTW64
		{
			public IntPtr hwnd;
			public UInt32 wFunc;
			public IntPtr pFrom; // Must be a double null terminated string, can't use strings because interop drops the double null
			public IntPtr pTo;   // Must be a double null terminated string, can't use strings because interop drops the double null
			public UInt16 fFlags;
			public Int32 fAnyOperationsAborted;
			public IntPtr hNameMappings;
			[MarshalAs(UnmanagedType.LPWStr)] public string lpszProgressTitle;
		}

		[StructLayout(LayoutKind.Sequential)]
		public struct SYSTEMTIME
		{
			public WORD wYear;
			public WORD wMonth;
			public WORD wDayOfWeek;
			public WORD wDay;
			public WORD wHour;
			public WORD wMinute;
			public WORD wSecond;
			public WORD wMilliseconds;
		}

		[StructLayout(LayoutKind.Sequential, CharSet = CharSet.Unicode, Pack = 4)]
		public struct TEXTRANGE(int min, int max)
		{
			/// <summary>The range of text to retrieve</summary>
			public CHARRANGE char_range = new(min, max);

			/// <summary>The buffer to receive the text</summary>
			[MarshalAs(UnmanagedType.LPWStr)]
			public string text = new('\0', max - min); // Allocated by caller, zero terminated by RichEdit
		}

		[StructLayout(LayoutKind.Sequential)]
		public struct WINDOWPOS
		{
			public HWND hwnd;
			public HWND hwndInsertAfter;
			public int  x;
			public int  y;
			public int  cx;
			public int  cy;
			public EWindowPos flags;
		}

		[StructLayout(LayoutKind.Sequential, CharSet = CharSet.Auto), BestFitMapping(false)]
		public struct WIN32_FIND_DATA
		{
			// Used by the FindFirstFile or FindNextFile functions.
			public FileAttributes dwFileAttributes;
			public uint ftCreationTime_dwLowDateTime;
			public uint ftCreationTime_dwHighDateTime;
			public uint ftLastAccessTime_dwLowDateTime;
			public uint ftLastAccessTime_dwHighDateTime;
			public uint ftLastWriteTime_dwLowDateTime;
			public uint ftLastWriteTime_dwHighDateTime;
			public uint nFileSizeHigh;
			public uint nFileSizeLow;
			public int dwReserved0;
			public int dwReserved1;
			[MarshalAs(UnmanagedType.ByValTStr, SizeConst = 260)] public string cFileName;
			[MarshalAs(UnmanagedType.ByValTStr, SizeConst = 14)] public string cAlternateFileName;

			/// <summary>The filename.extn of the file</summary>
			public readonly string FileName => cFileName;

			/// <summary>Attributes of the file.</summary>
			public readonly FileAttributes Attributes => dwFileAttributes;

			/// <summary>The file size</summary>
			public readonly long FileSize => ToLong(nFileSizeHigh, nFileSizeLow);

			/// <summary>File creation time in local time</summary>
			public readonly DateTime CreationTime => CreationTimeUtc.ToLocalTime();

			/// <summary>File creation time in UTC</summary>
			public readonly DateTime CreationTimeUtc => ToDateTime(ftCreationTime_dwHighDateTime, ftCreationTime_dwLowDateTime);

			/// <summary>Gets the last access time in local time.</summary>
			public readonly DateTime LastAccessTime => LastAccessTimeUtc.ToLocalTime();

			/// <summary>File last access time in UTC</summary>
			public readonly DateTime LastAccessTimeUtc => ToDateTime(ftLastAccessTime_dwHighDateTime, ftLastAccessTime_dwLowDateTime);

			/// <summary>Gets the last access time in local time.</summary>
			public readonly DateTime LastWriteTime => LastWriteTimeUtc.ToLocalTime();

			/// <summary>File last write time in UTC</summary>
			public readonly DateTime LastWriteTimeUtc => ToDateTime(ftLastWriteTime_dwHighDateTime, ftLastWriteTime_dwLowDateTime);

			public override readonly string ToString() => cFileName;
			private static DateTime ToDateTime(uint high, uint low) => DateTime.FromFileTimeUtc(ToLong(high, low));
			private static long ToLong(uint high, uint low) => ((long)high << 0x20) | low;
		}

		[StructLayout(LayoutKind.Sequential, CharSet = CharSet.Unicode)]
		public struct WNDCLASS
		{
			public uint style;
			public WNDPROC lpfnWndProc;
			public int cbClsExtra;
			public int cbWndExtra;
			public IntPtr hInstance;
			public IntPtr hIcon;
			public IntPtr hCursor;
			public IntPtr hbrBackground;
			[MarshalAs(UnmanagedType.LPWStr)] public string lpszMenuName;
			[MarshalAs(UnmanagedType.LPWStr)] public string lpszClassName;
		}

		[StructLayout(LayoutKind.Sequential, CharSet = CharSet.Unicode)]
		public struct WNDCLASSEX
		{
			public int cbSize; // = sizeof(WNDCLASSEX)
			public uint style;
			public WNDPROC lpfnWndProc;
			public int cbClsExtra;
			public int cbWndExtra;
			public IntPtr hInstance;
			public IntPtr hIcon;
			public IntPtr hCursor;
			public IntPtr hbrBackground;
			[MarshalAs(UnmanagedType.LPWStr)] public string lpszMenuName;
			[MarshalAs(UnmanagedType.LPWStr)] public string lpszClassName;
			public IntPtr hIconSm;
		}

		[StructLayout(LayoutKind.Sequential)]
		public struct XFORM
		{
			public float eM11;
			public float eM12;
			public float eM21;
			public float eM22;
			public float eDx;
			public float eDy;
		}

		#pragma warning restore IDE1006 // Naming Styles
	}
}
