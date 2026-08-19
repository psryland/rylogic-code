//***************************************************
// Win32 wrapper
//  Copyright (c) Rylogic Ltd 2008
//***************************************************

using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Diagnostics;
using System.Drawing;
using System.Linq;
using System.Reflection;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading;
using Rylogic.Common;
using Rylogic.Extn;
using Rylogic.Maths;
using Rylogic.Utility;
using HWND = System.IntPtr;

namespace Rylogic.Interop.Win32
{
	/// <summary>Win32 wrapper</summary>
	[System.Security.SuppressUnmanagedCodeSecurity] // We won't use this maliciously
	public static partial class Win32
	{
		// Style Guidance:
		// - Windows header constants should be in their own regions, grouped by the constant prefix.

		#region Windows Versions
		public static bool IsWindowsVistaOrLater
		{
			get { return Environment.OSVersion.Platform == PlatformID.Win32NT && Environment.OSVersion.Version >= new Version(6, 0, 6000); }
		}
		public static bool IsWindowsXPOrLater
		{
			get { return Environment.OSVersion.Platform == PlatformID.Win32NT && Environment.OSVersion.Version >= new Version(5, 1, 2600); }
		}
		#endregion

		#region Windows header constants

		#region Child Window From Point
		public const int CWP_ALL             = 0x0000; // Does not skip any child windows
		public const int CWP_SKIPINVISIBLE   = 0x0001; // Skips invisible child windows
		public const int CWP_SKIPDISABLED    = 0x0002; // Skips disabled child windows
		public const int CWP_SKIPTRANSPARENT = 0x0004; // Skips transparent child windows
		#endregion

		#region Class styles CS_
		public const int CS_BYTEALIGNCLIENT               = 0x00001000;
		public const int CS_BYTEALIGNWINDOW               = 0x00002000;
		public const int CS_CLASSDC                       = 0x00000040;
		public const int CS_DBLCLKS                       = 0x00000008;
		public const int CS_DROPSHADOW                    = 0x00020000;
		public const int CS_GLOBALCLASS                   = 0x00004000;
		public const int CS_HREDRAW                       = 0x00000002;
		public const int CS_NOCLOSE                       = 0x00000200;
		public const int CS_OWNDC                         = 0x00000020;
		public const int CS_PARENTDC                      = 0x00000080;
		public const int CS_SAVEBITS                      = 0x00000800;
		public const int CS_VREDRAW                       = 0x00000001;
		#endregion

		#region Common Controls
		public const int ICC_LISTVIEW_CLASSES   = 0x00000001; // listview, header
		public const int ICC_TREEVIEW_CLASSES   = 0x00000002; // treeview, tooltips
		public const int ICC_BAR_CLASSES        = 0x00000004; // toolbar, statusbar, trackbar, tooltips
		public const int ICC_TAB_CLASSES        = 0x00000008; // tab, tooltips
		public const int ICC_UPDOWN_CLASS       = 0x00000010; // updown
		public const int ICC_PROGRESS_CLASS     = 0x00000020; // progress
		public const int ICC_HOTKEY_CLASS       = 0x00000040; // hotkey
		public const int ICC_ANIMATE_CLASS      = 0x00000080; // animate
		public const int ICC_WIN95_CLASSES      = 0x000000FF;
		public const int ICC_DATE_CLASSES       = 0x00000100; // month picker, date picker, time picker, updown
		public const int ICC_USEREX_CLASSES     = 0x00000200; // comboex
		public const int ICC_COOL_CLASSES       = 0x00000400; // rebar (coolbar) control
		public const int ICC_INTERNET_CLASSES   = 0x00000800;
		public const int ICC_PAGESCROLLER_CLASS = 0x00001000; // page scroller
		public const int ICC_NATIVEFNTCTL_CLASS = 0x00002000; // native font control
		public const int ICC_STANDARD_CLASSES   = 0x00004000;
		public const int ICC_LINK_CLASS         = 0x00008000;
		#endregion

		#region Common Control Messages CCM_
		public const uint CCM_FIRST               = 0x2000;    // Common control shared messages
		public const uint CCM_LAST                = (CCM_FIRST + 0x200);
		public const uint CCM_SETBKCOLOR          = (CCM_FIRST + 0x1); // lParam is bkColor
		public const uint CCM_SETCOLORSCHEME      = (CCM_FIRST + 0x2); // lParam is color scheme
		public const uint CCM_GETCOLORSCHEME      = (CCM_FIRST + 0x3); // fills in COLORSCHEME pointed to by lParam
		public const uint CCM_GETDROPTARGET       = (CCM_FIRST + 0x4);
		public const uint CCM_SETUNICODEFORMAT    = (CCM_FIRST + 0x5);
		public const uint CCM_GETUNICODEFORMAT    = (CCM_FIRST + 0x6);
		public const uint CCM_SETVERSION          = (CCM_FIRST + 0x7);
		public const uint CCM_GETVERSION          = (CCM_FIRST + 0x8);
		public const uint CCM_SETNOTIFYWINDOW     = (CCM_FIRST + 0x9); // wParam == hwndParent.
		public const uint CCM_SETWINDOWTHEME      = (CCM_FIRST + 0xb);
		public const uint CCM_DPISCALE            = (CCM_FIRST + 0xc); // wParam == Awareness
		#endregion

		#region ComboBox Messages
		public const uint CB_GETEDITSEL            = 0x0140;
		public const uint CB_LIMITTEXT             = 0x0141;
		public const uint CB_SETEDITSEL            = 0x0142;
		public const uint CB_ADDSTRING             = 0x0143;
		public const uint CB_DELETESTRING          = 0x0144;
		public const uint CB_DIR                   = 0x0145;
		public const uint CB_GETCOUNT              = 0x0146;
		public const uint CB_GETCURSEL             = 0x0147;
		public const uint CB_GETLBTEXT             = 0x0148;
		public const uint CB_GETLBTEXTLEN          = 0x0149;
		public const uint CB_INSERTSTRING          = 0x014a;
		public const uint CB_RESETCONTENT          = 0x014b;
		public const uint CB_FINDSTRING            = 0x014c;
		public const uint CB_SELECTSTRING          = 0x014d;
		public const uint CB_SETCURSEL             = 0x014e;
		public const uint CB_SHOWDROPDOWN          = 0x014f;
		public const uint CB_GETITEMDATA           = 0x0150;
		public const uint CB_SETITEMDATA           = 0x0151;
		public const uint CB_GETDROPPEDCONTROLRECT = 0x0152;
		public const uint CB_SETITEMHEIGHT         = 0x0153;
		public const uint CB_GETITEMHEIGHT         = 0x0154;
		public const uint CB_SETEXTENDEDUI         = 0x0155;
		public const uint CB_GETEXTENDEDUI         = 0x0156;
		public const uint CB_GETDROPPEDSTATE       = 0x0157;
		public const uint CB_FINDSTRINGEXACT       = 0x0158;
		public const uint CB_SETLOCALE             = 0x0159;
		public const uint CB_GETLOCALE             = 0x015a;
		public const uint CB_GETTOPINDEX           = 0x015b;
		public const uint CB_SETTOPINDEX           = 0x015c;
		public const uint CB_GETHORIZONTALEXTENT   = 0x015d;
		public const uint CB_SETHORIZONTALEXTENT   = 0x015e;
		public const uint CB_GETDROPPEDWIDTH       = 0x015f;
		public const uint CB_SETDROPPEDWIDTH       = 0x0160;
		public const uint CB_INITSTORAGE           = 0x0161;
		public const uint CB_MSGMAX_OLD            = 0x0162;
		public const uint CB_MULTIPLEADDSTRING     = 0x0163;
		public const uint CB_GETCOMBOBOXINFO       = 0x0164;
		public const uint CB_MSGMAX                = 0x0165;
		#endregion
		
		#region Create Window
		public const int CW_USEDEFAULT = unchecked((int)0x80000000);
		#endregion

		#region Device Changed event types DBT_
		public enum EDeviceChangedEventType
		{
			/// <summary>DBT_DEVNODES_CHANGED - A device has been added to or removed from the system.</summary>
			DevNodesChanged = 0x0007,

			/// <summary>DBT_QUERYCHANGECONFIG - Permission is requested to change the current configuration(dock or undock).</summary>
			QueryChangeConfig = 0x0017,

			/// <summary>DBT_CONFIGCHANGED - The current configuration has changed, due to a dock or undock.</summary>
			ConfigChanged = 0x0018,

			/// <summary>DBT_CONFIGCHANGECANCELED - A request to change the current configuration(dock or undock) has been canceled.</summary>
			ConfigChangeCanceled = 0x0019,

			/// <summary>DBT_DEVICEARRIVAL - A device or piece of media has been inserted and is now available.</summary>
			DeviceArrival = 0x8000,

			/// <summary>DBT_DEVICEQUERYREMOVE - Permission is requested to remove a device or piece of media.Any application can deny this request and cancel the removal.</summary>
			DeviceQueryRemove = 0x8001,

			/// <summary>DBT_DEVICEQUERYREMOVEFAILED - A request to remove a device or piece of media has been canceled.</summary>
			DeviceQueryRemoveFailed = 0x8002,

			/// <summary>DBT_DEVICEREMOVEPENDING - A device or piece of media is about to be removed. Cannot be denied.</summary>
			DeviceRemovePending = 0x8003,

			/// <summary>DBT_DEVICEREMOVECOMPLETE - A device or piece of media has been removed.</summary>
			DeviceRemoveComplete = 0x8004,

			/// <summary>DBT_DEVICETYPESPECIFIC - A device-specific event has occurred.</summary>
			DeviceTypeSpecific = 0x8005,

			/// <summary>DBT_CUSTOMEVENT - A custom event has occurred.</summary>
			CustomEvent = 0x8006,

			/// <summary>DBT_USERDEFINED - The meaning of this message is user-defined.</summary>
			UserDefined = 0xFFFF,
		}
		#endregion

		#region Device Broadcast flags
		public enum EDeviceBroadcaseType : uint
		{
			/// <summary>DBT_DEVTYP_OEM - OEM- or IHV-defined device type. This structure is a DEV_BROADCAST_OEM structure.</summary>
			OEM = 0x00000000,

			/// <summary>DBT_DEVTYP_VOLUME - Logical volume.This structure is a DEV_BROADCAST_VOLUME structure.</summary>
			Volume = 0x00000002,

			/// <summary>DBT_DEVTYP_PORT - Port device (serial or parallel). This structure is a DEV_BROADCAST_PORT structure.</summary>
			Port = 0x00000003,

			/// <summary>DBT_DEVTYP_DEVICEINTERFACE - Class of devices. This structure is a DEV_BROADCAST_DEVICEINTERFACE structure.</summary>
			DeviceInterface = 0x00000005,

			/// <summary>DBT_DEVTYP_HANDLE - File system handle. This structure is a DEV_BROADCAST_HANDLE structure.</summary>
			Handle = 0x00000006,
		}
		#endregion

		#region Device Notify flags
		[Flags] public enum EDeviceNotifyFlags
		{
			/// <summary>DEVICE_NOTIFY_WINDOW_HANDLE - The hRecipient parameter is a window handle.</summary>
			WindowHandle = 0x00000000,

			/// <summary>DEVICE_NOTIFY_SERVICE_HANDLE - The hRecipient parameter is a service status handle.</summary>
			ServiceHandle = 0x00000001,

			/// <summary>
			/// DEVICE_NOTIFY_ALL_INTERFACE_CLASSES - Notifies the recipient of device interface events for all
			/// device interface classes (the 'dbcc_classguid' member is ignored). This value can be used only if the 
			/// 'dbch_devicetype' member is 'DBT_DEVTYP_DEVICEINTERFACE'.</summary>
			AllInterface_Classes = 0x00000004,
		}
		#endregion

		#region Dialog Box Command IDs
		public const int IDOK       = 1;
		public const int IDCANCEL   = 2;
		public const int IDABORT    = 3;
		public const int IDRETRY    = 4;
		public const int IDIGNORE   = 5;
		public const int IDYES      = 6;
		public const int IDNO       = 7;
		public const int IDCLOSE    = 8;
		public const int IDHELP     = 9;
		public const int IDTRYAGAIN = 10;
		public const int IDCONTINUE = 11;
		public const int IDTIMEOUT  = 32000;
		#endregion

		#region Dialog get codes
		public const int DLGC_WANTARROWS      = 0x0001; // Direction keys.
		public const int DLGC_WANTTAB         = 0x0002; // TAB key.
		public const int DLGC_WANTALLKEYS     = 0x0004; // All keyboard input.
		public const int DLGC_WANTMESSAGE     = 0x0004; // All keyboard input (the application passes this message in the MSG structure to the control).
		public const int DLGC_HASSETSEL       = 0x0008; // EM_SETSEL messages.
		public const int DLGC_DEFPUSHBUTTON   = 0x0010; // Default push button.
		public const int DLGC_UNDEFPUSHBUTTON = 0x0020; // Non-default push button.
		public const int DLGC_RADIOBUTTON     = 0x0040; // Radio button.
		public const int DLGC_WANTCHARS       = 0x0080; // WM_CHAR messages.
		public const int DLGC_STATIC          = 0x0100; //Static control.
		public const int DLGC_BUTTON          = 0x2000; // Button.
		#endregion

		#region Dialog Styles DS_
		public const int DS_ABSALIGN      = 0x01;
		public const int DS_SYSMODAL      = 0x02;
		public const int DS_LOCALEDIT     = 0x20;/* 16-bit: Edit items get Local storage. 32-bit and up: meaningless. */
		public const int DS_SETFONT       = 0x40;/* User specified font for Dlg controls */
		public const int DS_MODALFRAME    = 0x80;/* Can be combined with WS_CAPTION  */
		public const int DS_NOIDLEMSG     = 0x100;/* WM_ENTERIDLE message will not be sent */
		public const int DS_SETFOREGROUND = 0x200;/* not in win3.1 */
		public const int DS_3DLOOK        = 0x0004;
		public const int DS_FIXEDSYS      = 0x0008;
		public const int DS_NOFAILCREATE  = 0x0010;
		public const int DS_CONTROL       = 0x0400;
		public const int DS_CENTER        = 0x0800;
		public const int DS_CENTERMOUSE   = 0x1000;
		public const int DS_CONTEXTHELP   = 0x2000;
		public const int DS_SHELLFONT     = DS_SETFONT | DS_FIXEDSYS;
		public const int DS_USEPIXELS     = 0x8000;
		#endregion

		#region Edit Control
		// Edit Control Styles
		public const int ES_LEFT        = 0x0000;
		public const int ES_CENTER      = 0x0001;
		public const int ES_RIGHT       = 0x0002;
		public const int ES_MULTILINE   = 0x0004;
		public const int ES_UPPERCASE   = 0x0008;
		public const int ES_LOWERCASE   = 0x0010;
		public const int ES_PASSWORD    = 0x0020;
		public const int ES_AUTOVSCROLL = 0x0040;
		public const int ES_AUTOHSCROLL = 0x0080;
		public const int ES_NOHIDESEL   = 0x0100;
		public const int ES_OEMCONVERT  = 0x0400;
		public const int ES_READONLY    = 0x0800;
		public const int ES_WANTRETURN  = 0x1000;
		public const int ES_NUMBER      = 0x2000;

		// Edit Control Notification Codes
		public const int EN_SETFOCUS     = 0x0100;
		public const int EN_KILLFOCUS    = 0x0200;
		public const int EN_CHANGE       = 0x0300;
		public const int EN_UPDATE       = 0x0400;
		public const int EN_ERRSPACE     = 0x0500;
		public const int EN_MAXTEXT      = 0x0501;
		public const int EN_HSCROLL      = 0x0601;
		public const int EN_VSCROLL      = 0x0602;
		public const int EN_ALIGN_LTR_EC = 0x0700;
		public const int EN_ALIGN_RTL_EC = 0x0701;

		// Edit control EM_SETMARGIN parameters
		public const int EC_LEFTMARGIN  = 0x0001;
		public const int EC_RIGHTMARGIN = 0x0002;
		public const int EC_USEFONTINFO = 0xffff;

		// Edit Control Messages
		public const int EM_GETSEL               = 0x00B0;
		public const int EM_SETSEL               = 0x00B1;
		public const int EM_GETRECT              = 0x00B2;
		public const int EM_SETRECT              = 0x00B3;
		public const int EM_SETRECTNP            = 0x00B4;
		public const int EM_SCROLL               = 0x00B5;
		public const int EM_LINESCROLL           = 0x00B6;
		public const int EM_SCROLLCARET          = 0x00B7;
		public const int EM_GETMODIFY            = 0x00B8;
		public const int EM_SETMODIFY            = 0x00B9;
		public const int EM_GETLINECOUNT         = 0x00BA;
		public const int EM_LINEINDEX            = 0x00BB;
		public const int EM_SETHANDLE            = 0x00BC;
		public const int EM_GETHANDLE            = 0x00BD;
		public const int EM_GETTHUMB             = 0x00BE;
		public const int EM_LINELENGTH           = 0x00C1;
		public const int EM_REPLACESEL           = 0x00C2;
		public const int EM_GETLINE              = 0x00C4;
		public const int EM_LIMITTEXT            = 0x00C5;
		public const int EM_CANUNDO              = 0x00C6;
		public const int EM_UNDO                 = 0x00C7;
		public const int EM_FMTLINES             = 0x00C8;
		public const int EM_LINEFROMCHAR         = 0x00C9;
		public const int EM_SETTABSTOPS          = 0x00CB;
		public const int EM_SETPASSWORDCHAR      = 0x00CC;
		public const int EM_EMPTYUNDOBUFFER      = 0x00CD;
		public const int EM_GETFIRSTVISIBLELINE  = 0x00CE;
		public const int EM_SETREADONLY          = 0x00CF;
		public const int EM_SETWORDBREAKPROC     = 0x00D0;
		public const int EM_GETWORDBREAKPROC     = 0x00D1;
		public const int EM_GETPASSWORDCHAR      = 0x00D2;
		public const int EM_SETMARGINS           = 0x00D3;
		public const int EM_GETMARGINS           = 0x00D4;
		public const int EM_SETLIMITTEXT         = EM_LIMITTEXT;//;win40 Name change 
		public const int EM_GETLIMITTEXT         = 0x00D5;
		public const int EM_POSFROMCHAR          = 0x00D6;
		public const int EM_CHARFROMPOS          = 0x00D7;
		public const int EM_SETIMESTATUS         = 0x00D8;
		public const int EM_GETIMESTATUS         = 0x00D9;
		public const int EM_FORMATRANGE          = (int)WM_USER + 57;

		// EDITWORDBREAKPROC code values
		public const int WB_LEFT        = 0;
		public const int WB_RIGHT       = 1;
		public const int WB_ISDELIMITER = 2;
		#endregion

		#region Execution State
		[Flags] public enum ExecutionState :uint
		{
			EsAwaymodeRequired = 0x00000040,
			EsContinuous = 0x80000000,
			EsDisplayRequired = 0x00000002,
			EsSystemRequired = 0x00000001
		}
		#endregion

		#region Flash Window flags
		[Flags] public enum EFlashWindowFlags : uint
		{
			/// <summary>Stop flashing. The system restores the window to its original state.</summary>
			FLASHW_STOP = 0,

			/// <summary>Flash the window caption.</summary>
			FLASHW_CAPTION = 1,

			/// <summary>Flash the task bar button.</summary>
			FLASHW_TRAY = 2,

			/// <summary>Flash both the window caption and task bar button. This is equivalent to setting the FLASHW_CAPTION | FLASHW_TRAY flags.</summary>
			FLASHW_ALL = 3,

			/// <summary>Flash continuously, until the FLASHW_STOP flag is set.</summary>
			FLASHW_TIMER = 4,

			/// <summary>Flash continuously until the window comes to the foreground.</summary>
			FLASHW_TIMERNOFG = 12,
		}
		#endregion

		#region GetAncestor GA_
		public const int GA_PARENT    = 1;
		public const int GA_ROOT      = 2;
		public const int GA_ROOTOWNER = 3;
		#endregion

		#region Get Class Long Pointer GCLP_, GCL_
		public const int GCW_ATOM           = -32; // Retrieves an ATOM value that uniquely identifies the window class. This is the same atom that the RegisterClassEx function returns.
		public const int GCL_CBCLSEXTRA     = -20; // Retrieves the size, in bytes, of the extra memory associated with the class.
		public const int GCL_CBWNDEXTRA     = -18; // Retrieves the size, in bytes, of the extra window memory associated with each window in the class. For information on how to access this memory, see GetWindowLongPtr.
		public const int GCLP_HBRBACKGROUND = -10; // Retrieves a handle to the background brush associated with the class.
		public const int GCLP_HCURSOR       = -12; // Retrieves a handle to the cursor associated with the class.
		public const int GCLP_HICON         = -14; // Retrieves a handle to the icon associated with the class.
		public const int GCLP_HICONSM       = -34; // Retrieves a handle to the small icon associated with the class.
		public const int GCLP_HMODULE       = -16; // Retrieves a handle to the module that registered the class.
		public const int GCLP_MENUNAME      =  -8; // Retrieves the pointer to the menu name string. The string identifies the menu resource associated with the class.
		public const int GCL_STYLE          = -26; // Retrieves the window-class style bits.
		public const int GCLP_WNDPROC       = -24; // Retrieves the address of the window procedure, or a handle representing the address of the window procedure. You must use the CallWindowProc function to call the window procedure.
		#endregion

		#region Get Window Long Pointer GWL_, GWLP_
		public const int GWL_WNDPROC   = -4;
		public const int GWL_HINSTANCE = -6;
		public const int GWL_ID        = -12;
		public const int GWL_STYLE     = -16;
		public const int GWL_EXSTYLE   = -20;
		public const int GWL_USERDATA  = -21;
		public const int GWLP_WNDPROC    = -4;
		public const int GWLP_HINSTANCE  = -6;
		public const int GWLP_HWNDPARENT = -8;
		public const int GWLP_USERDATA   = -21;
		public const int GWLP_ID         = -12;
		#endregion

		#region GetWindow uCmd GW_
		public const uint GW_OWNER = 4;
		#endregion

		#region Graphics modes GM_
		public const int GM_COMPATIBLE                    = 1;
		public const int GM_ADVANCED                      = 2;
		#endregion

		#region HitTest
		public enum HitTest :int
		{
			HTBORDER      = 18, // In the border of a window that does not have a sizing border.
			HTBOTTOM      = 15, // In the lower-horizontal border of a resizeable window (the user can click the mouse to resize the window vertically).
			HTBOTTOMLEFT  = 16, // In the lower-left corner of a border of a resizeable window (the user can click the mouse to resize the window diagonally).
			HTBOTTOMRIGHT = 17, // In the lower-right corner of a border of a resizeable window (the user can click the mouse to resize the window diagonally).
			HTCAPTION     = 2,  // In a title bar.
			HTCLIENT      = 1,  // In a client area.
			HTCLOSE       = 20, // In a Close button.
			HTERROR       = -2, // On the screen background or on a dividing line between windows (same as HTNOWHERE, except that the DefWindowProc function produces a system beep to indicate an error).
			HTGROWBOX     = 4,  // In a size box (same as HTSIZE).
			HTHELP        = 21, // In a Help button.
			HTHSCROLL     = 6,  // In a horizontal scroll bar.
			HTLEFT        = 10, // In the left border of a resizeable window (the user can click the mouse to resize the window horizontally).
			HTMENU        = 5,  // In a menu.
			HTMAXBUTTON   = 9,  // In a Maximize button.
			HTMINBUTTON   = 8,  // In a Minimize button.
			HTNOWHERE     = 0,  // On the screen background or on a dividing line between windows.
			HTREDUCE      = 8,  // In a Minimize button.
			HTRIGHT       = 11, // In the right border of a resizeable window (the user can click the mouse to resize the window horizontally).
			HTSIZE        = 4,  // In a size box (same as HTGROWBOX).
			HTSYSMENU     = 3,  // In a window menu or in a Close button in a child window.
			HTTOP         = 12, // In the upper-horizontal border of a window.
			HTTOPLEFT     = 13, // In the upper-left corner of a window border.
			HTTOPRIGHT    = 14, // In the upper-right corner of a window border.
			HTTRANSPARENT = -1, // In a window currently covered by another window in the same thread (the message will be sent to underlying windows in the same thread until one of them returns a code that is not HTTRANSPARENT).
			HTVSCROLL     = 7,  // In the vertical scroll bar.
			HTZOOM        = 9,  // In a Maximize button.
		}
		#endregion

		#region HWND constants HWND_
		public static readonly HWND HWND_TOP = new(0);
		public static readonly HWND HWND_BOTTOM = new(1);
		public static readonly HWND HWND_TOPMOST = new(-1);
		public static readonly HWND HWND_NOTOPMOST = new(-2);
		#endregion

		#region IME
		// bit field for conversion mode
		[Flags] public enum EIME_CMODE
		{
			ALPHANUMERIC = 0x0000,
			NATIVE = 0x0001,
			CHINESE = NATIVE,
			HANGUL = NATIVE,
			JAPANESE = NATIVE,
			KATAKANA = 0x0002,// only effect under IME_CMODE_NATIVE
			LANGUAGE = 0x0003,
			FULLSHAPE = 0x0008,
			ROMAN = 0x0010,
			CHARCODE = 0x0020,
			HANJACONVERT = 0x0040,
			NATIVESYMBOL = 0x0080,
			HANGEUL = NATIVE, // IME_CMODE_HANGEUL is old name of IME_CMODE_HANGUL. It will be gone eventually.
			SOFTKBD = 0x0080,
			NOCONVERSION = 0x0100,
			EUDC = 0x0200,
			SYMBOL = 0x0400,
			FIXED = 0x0800,
		}

		// bit field for sentence mode
		[Flags] public enum EIME_SMODE
		{
			NONE = 0x0000,
			PLAURALCLAUSE = 0x0001,
			SINGLECONVERT = 0x0002,
			AUTOMATIC = 0x0004,
			PHRASEPREDICT = 0x0008,
			CONVERSATION = 0x0010,
			RESERVED = 0x0000F000,
		}

		// style of candidate
		public enum EIME_CAND
		{
			UNKNOWN = 0x0000,
			READ = 0x0001,
			CODE = 0x0002,
			MEANING = 0x0003,
			RADICAL = 0x0004,
			STROKE = 0x0005,
		}

		// wParam of report message WM_IME_NOTIFY
		public enum EIME_Notification
		{
			IMN_CLOSESTATUSWINDOW = 0x0001,
			IMN_OPENSTATUSWINDOW = 0x0002,
			IMN_CHANGECANDIDATE = 0x0003,
			IMN_CLOSECANDIDATE = 0x0004,
			IMN_OPENCANDIDATE = 0x0005,
			IMN_SETCONVERSIONMODE = 0x0006,
			IMN_SETSENTENCEMODE = 0x0007,
			IMN_SETOPENSTATUS = 0x0008,
			IMN_SETCANDIDATEPOS = 0x0009,
			IMN_SETCOMPOSITIONFONT = 0x000A,
			IMN_SETCOMPOSITIONWINDOW = 0x000B,
			IMN_SETSTATUSWINDOWPOS = 0x000C,
			IMN_GUIDELINE = 0x000D,
			IMN_PRIVATE = 0x000E,
		}

		// wParam of report message WM_IME_REQUEST
		public enum EIME_Request
		{
			COMPOSITIONWINDOW = 0x0001,
			CANDIDATEWINDOW = 0x0002,
			COMPOSITIONFONT = 0x0003,
			RECONVERTSTRING = 0x0004,
			CONFIRMRECONVERTSTRING = 0x0005,
			QUERYCHARPOSITION = 0x0006,
			DOCUMENTFEED = 0x0007,
		}

		// wParam for WM_IME_CONTROL
		public enum EIME_Control
		{
			GETCANDIDATEPOS             = 0x0007,
			SETCANDIDATEPOS             = 0x0008,
			GETCOMPOSITIONFONT          = 0x0009,
			SETCOMPOSITIONFONT          = 0x000A,
			GETCOMPOSITIONWINDOW        = 0x000B,
			SETCOMPOSITIONWINDOW        = 0x000C,
			GETSTATUSWINDOWPOS          = 0x000F,
			SETSTATUSWINDOWPOS          = 0x0010,
			CLOSESTATUSWINDOW           = 0x0021,
			OPENSTATUSWINDOW            = 0x0022,
		}

		// dwAction for ImmNotifyIME
		public enum EIMM_Notification
		{
			OPENCANDIDATE                = 0x0010,
			CLOSECANDIDATE               = 0x0011,
			SELECTCANDIDATESTR           = 0x0012,
			CHANGECANDIDATELIST          = 0x0013,
			FINALIZECONVERSIONRESULT     = 0x0014,
			COMPOSITIONSTR               = 0x0015,
			SETCANDIDATE_PAGESTART       = 0x0016,
			SETCANDIDATE_PAGESIZE        = 0x0017,
			IMEMENUSELECTED              = 0x0018,
		}

		// lParam for WM_IME_SETCONTEXT
		[Flags] public enum EIME_SetContextFlags :ulong
		{
			SHOWUICANDIDATEWINDOW       = 0x00000001,
			SHOWUICOMPOSITIONWINDOW     = 0x80000000,
			SHOWUIGUIDELINE             = 0x40000000,
			SHOWUIALLCANDIDATEWINDOW    = 0x0000000F,
			SHOWUIALL                   = 0xC000000F,
		}
		#endregion

		#region Known Folders
		public enum FFFP_MODE
		{
			FFFP_EXACTMATCH,
			FFFP_NEARESTPARENTMATCH
		}

		public enum KF_CATEGORY
		{
			VIRTUAL = 0x00000001,
			FIXED   = 0x00000002,
			COMMON  = 0x00000003,
			PERUSER = 0x00000004
		}

		[Flags]
		public enum KF_DEFINITION_FLAGS
		{
			KFDF_PERSONALIZE = 0x00000001,
			KFDF_LOCAL_REDIRECT_ONLY = 0x00000002,
			KFDF_ROAMABLE = 0x00000004,
		}
		#endregion

		#region ListBox Messages
		public const uint LB_ADDSTRING            = 0x0180;
		public const uint LB_INSERTSTRING         = 0x0181;
		public const uint LB_DELETESTRING         = 0x0182;
		public const uint LB_SELITEMRANGEEX       = 0x0183;
		public const uint LB_RESETCONTENT         = 0x0184;
		public const uint LB_SETSEL               = 0x0185;
		public const uint LB_SETCURSEL            = 0x0186;
		public const uint LB_GETSEL               = 0x0187;
		public const uint LB_GETCURSEL            = 0x0188;
		public const uint LB_GETTEXT              = 0x0189;
		public const uint LB_GETTEXTLEN           = 0x018A;
		public const uint LB_GETCOUNT             = 0x018B;
		public const uint LB_SELECTSTRING         = 0x018C;
		public const uint LB_DIR                  = 0x018D;
		public const uint LB_GETTOPINDEX          = 0x018E;
		public const uint LB_FINDSTRING           = 0x018F;
		public const uint LB_GETSELCOUNT          = 0x0190;
		public const uint LB_GETSELITEMS          = 0x0191;
		public const uint LB_SETTABSTOPS          = 0x0192;
		public const uint LB_GETHORIZONTALEXTENT  = 0x0193;
		public const uint LB_SETHORIZONTALEXTENT  = 0x0194;
		public const uint LB_SETCOLUMNWIDTH       = 0x0195;
		public const uint LB_ADDFILE              = 0x0196;
		public const uint LB_SETTOPINDEX          = 0x0197;
		public const uint LB_GETITEMRECT          = 0x0198;
		public const uint LB_GETITEMDATA          = 0x0199;
		public const uint LB_SETITEMDATA          = 0x019A;
		public const uint LB_SELITEMRANGE         = 0x019B;
		public const uint LB_SETANCHORINDEX       = 0x019C;
		public const uint LB_GETANCHORINDEX       = 0x019D;
		public const uint LB_SETCARETINDEX        = 0x019E;
		public const uint LB_GETCARETINDEX        = 0x019F;
		public const uint LB_SETITEMHEIGHT        = 0x01A0;
		public const uint LB_GETITEMHEIGHT        = 0x01A1;
		public const uint LB_FINDSTRINGEXACT      = 0x01A2;
		public const uint LB_SETLOCALE            = 0x01A5;
		public const uint LB_GETLOCALE            = 0x01A6;
		public const uint LB_SETCOUNT             = 0x01A7;
		public const uint LB_INITSTORAGE          = 0x01A8;
		public const uint LB_ITEMFROMPOINT        = 0x01A9;
		public const uint LB_GETLISTBOXINFO       = 0x01B2;
		#endregion

		#region ListView Control
		public const uint LVN_ITEMCHANGING    = (LVN_FIRST-0 );
		public const uint LVN_ITEMCHANGED     = (LVN_FIRST-1 );
		public const uint LVN_INSERTITEM      = (LVN_FIRST-2 );
		public const uint LVN_DELETEITEM      = (LVN_FIRST-3 );
		public const uint LVN_DELETEALLITEMS  = (LVN_FIRST-4 );
		public const uint LVN_BEGINLABELEDITA = (LVN_FIRST-5 );
		public const uint LVN_BEGINLABELEDITW = (LVN_FIRST-75);
		public const uint LVN_ENDLABELEDITA   = (LVN_FIRST-6 );
		public const uint LVN_ENDLABELEDITW   = (LVN_FIRST-76);
		public const uint LVN_COLUMNCLICK     = (LVN_FIRST-8 );
		public const uint LVN_BEGINDRAG       = (LVN_FIRST-9 );
		public const uint LVN_BEGINRDRAG      = (LVN_FIRST-11);
		public const uint LVN_ODCACHEHINT     = (LVN_FIRST-13);
		public const uint LVN_ODFINDITEMA     = (LVN_FIRST-52);
		public const uint LVN_ODFINDITEMW     = (LVN_FIRST-79);
		public const uint LVN_ITEMACTIVATE    = (LVN_FIRST-14);
		public const uint LVN_ODSTATECHANGED  = (LVN_FIRST-15);
		public const uint LVN_HOTTRACK        = (LVN_FIRST-21);
		public const uint LVN_GETDISPINFOA    = (LVN_FIRST-50);
		public const uint LVN_GETDISPINFOW    = (LVN_FIRST-77);
		public const uint LVN_SETDISPINFOA    = (LVN_FIRST-51);
		public const uint LVN_SETDISPINFOW    = (LVN_FIRST-78);
		public const uint LVN_BEGINLABELEDIT  = LVN_BEGINLABELEDITW;
		public const uint LVN_ENDLABELEDIT    = LVN_ENDLABELEDITW;
		public const uint LVN_GETDISPINFO     = LVN_GETDISPINFOW;
		public const uint LVN_SETDISPINFO     = LVN_SETDISPINFOW;
		public const uint LVN_ODFINDITEM      = LVN_ODFINDITEMW;
		#endregion

		#region Mapping Modes MM_
		public const int MM_TEXT                          = 1;
		public const int MM_LOMETRIC                      = 2;
		public const int MM_HIMETRIC                      = 3;
		public const int MM_LOENGLISH                     = 4;
		public const int MM_HIENGLISH                     = 5;
		public const int MM_TWIPS                         = 6;
		public const int MM_ISOTROPIC                     = 7;
		public const int MM_ANISOTROPIC                   = 8;
		#endregion

		#region MAPKV - map virtual key
		public const byte MAPVK_VK_TO_VSC = 0x0;
		public const byte MAPVK_VSC_TO_VK = 0x1;
		public const byte MAPVK_VK_TO_CHAR = 0x2;
		public const byte MAPVK_VSC_TO_VK_EX = 0x3;
		#endregion

		#region Menu Flags MF_, MFT_, MFS_
		public const int MF_INSERT           = 0x00000000;
		public const int MF_CHANGE           = 0x00000080;
		public const int MF_APPEND           = 0x00000100;
		public const int MF_DELETE           = 0x00000200;
		public const int MF_REMOVE           = 0x00001000;
		public const int MF_BYCOMMAND        = 0x00000000;
		public const int MF_BYPOSITION       = 0x00000400;
		public const int MF_SEPARATOR        = 0x00000800;
		public const int MF_ENABLED          = 0x00000000;
		public const int MF_GRAYED           = 0x00000001;
		public const int MF_DISABLED         = 0x00000002;
		public const int MF_UNCHECKED        = 0x00000000;
		public const int MF_CHECKED          = 0x00000008;
		public const int MF_USECHECKBITMAPS  = 0x00000200;
		public const int MF_STRING           = 0x00000000;
		public const int MF_BITMAP           = 0x00000004;
		public const int MF_OWNERDRAW        = 0x00000100;
		public const int MF_POPUP            = 0x00000010;
		public const int MF_MENUBARBREAK     = 0x00000020;
		public const int MF_MENUBREAK        = 0x00000040;
		public const int MF_UNHILITE         = 0x00000000;
		public const int MF_HILITE           = 0x00000080;
		public const int MF_DEFAULT          = 0x00001000;
		public const int MF_SYSMENU          = 0x00002000;
		public const int MF_HELP             = 0x00004000;
		public const int MF_RIGHTJUSTIFY     = 0x00004000;
		public const int MF_MOUSESELECT      = 0x00008000;
		public const int MF_END              = 0x00000080;  /* Obsolete -- only used by old RES files */

		public const int MFT_STRING          = MF_STRING;
		public const int MFT_BITMAP          = MF_BITMAP;
		public const int MFT_MENUBARBREAK    = MF_MENUBARBREAK;
		public const int MFT_MENUBREAK       = MF_MENUBREAK;
		public const int MFT_OWNERDRAW       = MF_OWNERDRAW;
		public const int MFT_RADIOCHECK      = 0x00000200;
		public const int MFT_SEPARATOR       = MF_SEPARATOR;
		public const int MFT_RIGHTORDER      = 0x00002000;
		public const int MFT_RIGHTJUSTIFY    = MF_RIGHTJUSTIFY;

		public const int MFS_GRAYED          = 0x00000003;
		public const int MFS_DISABLED        = MFS_GRAYED;
		public const int MFS_CHECKED         = MF_CHECKED;
		public const int MFS_HILITE          = MF_HILITE;
		public const int MFS_ENABLED         = MF_ENABLED;
		public const int MFS_UNCHECKED       = MF_UNCHECKED;
		public const int MFS_UNHILITE        = MF_UNHILITE;
		public const int MFS_DEFAULT         = MF_DEFAULT;
		#endregion

		#region Monitor From
		/// <summary>Flags for the MonitorFromPoint function</summary>
		public enum EMonitorFromFlags
		{
			DEFAULT_TO_NULL = 0x00000000,
			DEFAULT_TO_PRIMARY = 0x00000001,
			DEFAULT_TO_NEAREST = 0x00000002,
		}
		#endregion

		#region Mouse key MK_
		// Key State Masks for Mouse Messages
		public const int MK_LBUTTON = 0x0001;
		public const int MK_RBUTTON = 0x0002;
		public const int MK_SHIFT = 0x0004;
		public const int MK_CONTROL = 0x0008;
		public const int MK_MBUTTON = 0x0010;
		public const int MK_XBUTTON1 = 0x0020;
		public const int MK_XBUTTON2 = 0x0040;
		#endregion

		#region MOUSE_MOVE_
		public const uint MOUSE_MOVE_RELATIVE = 0;
		public const uint MOUSE_MOVE_ABSOLUTE = 1;
		public const uint MOUSE_VIRTUAL_DESKTOP = 2;  // the coordinates are mapped to the virtual desktop
		public const uint MOUSE_ATTRIBUTES_CHANGED = 4;  // requery for mouse attributes
		public const uint MOUSE_MOVE_NOCOALESCE = 8;
		#endregion

		#region NIN Notification Icon
		/// <summary>
		/// This message is only send when using NOTIFYICON_VERSION_4, the Shell now sends the associated application an NIN_SELECT notification.
		/// Send when a notify icon is activated with mouse or ENTER key. Earlier versions send WM_RBUTTONDOWN and WM_RBUTTONUP messages.</summary>
		public const int NIN_SELECT = WM_USER;

		/// <summary>
		/// This message is only send when using NOTIFYICON_VERSION_4, the Shell now sends the associated application an NIN_SELECT notification.
		/// Send when a notify icon is activated with SPACEBAR or ENTER key.
		/// Earlier versions send WM_RBUTTONDOWN and WM_RBUTTONUP messages.</summary>
		public const int NIN_KEYSELECT = WM_USER + 1;

		/// <summary>Sent when the balloon is shown (balloons are queued).</summary>
		public const int NIN_BALLOONSHOW = WM_USER + 2;

		/// <summary>
		/// Sent when the balloon disappears. For example, when the icon is deleted.
		/// This message is not sent if the balloon is dismissed because of a timeout or if the user clicks the mouse.
		/// As of Windows 7, NIN_BALLOONHIDE is also sent when a notification with the NIIF_RESPECT_QUIET_TIME flag set attempts
		/// to display during quiet time (a user's first hour on a new computer).In that case, the balloon is never displayed at all.</summary>
		public const int NIN_BALLOONHIDE = WM_USER + 3;

		/// <summary>Sent when the balloon is dismissed because of a timeout.</summary>
		public const int NIN_BALLOONTIMEOUT = WM_USER + 4;

		/// <summary>Sent when the balloon is dismissed because the user clicked the mouse.</summary>
		public const int NIN_BALLOONUSERCLICK = WM_USER + 5;

		/// <summary>Sent when the user hovers the cursor over an icon to indicate that the richer pop-up UI should be used in place of a standard textual tooltip.</summary>
		public const int NIN_POPUPOPEN = WM_USER + 6;

		/// <summary>Sent when a cursor no longer hovers over an icon to indicate that the rich pop-up UI should be closed.</summary>
		public const int NIN_POPUPCLOSE = WM_USER + 7;
		#endregion

		#region Notification NF_,NFR_
		public const int NFR_ANSI = 1;
		public const int NFR_UNICODE = 2;
		public const int NF_QUERY = 3;
		public const int NF_REQUERY = 4;
		#endregion

		#region Notification Icon
		public enum ENotifyIconVersion
		{
			/// <summary>Default behavior (legacy Win95). Expects a 'NotifyIconData' size of 488.</summary>
			Win95 = 0x0,

			/// <summary>Behavior representing Win2000 an higher. Expects a 'NotifyIconData' size of 504.</summary>
			Win2000 = 0x3,

			/// <summary>
			/// Extended tooltip support, which is available for Vista and later.
			/// Detailed information about what the different versions do, can be found
			/// <a href="https://docs.microsoft.com/en-us/windows/win32/api/shellapi/nf-shellapi-shell_notifyicona">here</a>
			/// </summary>
			Vista = 0x4,
		}
		public enum ENotifyCommand
		{
			/// <summary>The taskbar icon is being created.</summary>
			Add = 0x00,

			/// <summary>The settings of the taskbar icon are being updated.</summary>
			Modify = 0x01,

			/// <summary>The taskbar icon is deleted.</summary>
			Delete = 0x02,

			/// <summary>Focus is returned to the taskbar icon. Currently not in use.</summary>
			SetFocus = 0x03,

			/// <summary>
			/// Shell32.dll version 5.0 and later only. Instructs the taskbar to behave according to the version number specified in the 
			/// uVersion member of the structure pointed to by lpdata. This message allows you to specify whether you want the version
			/// 5.0 behavior found on Microsoft Windows 2000 systems, or the behavior found on earlier Shell versions.
			/// The default value for uVersion is zero, indicating that the original Windows 95 notify icon behavior should be used.</summary>
			SetVersion = 0x04,
		}
		[Flags]
		public enum ENotifyIconDataMembers :uint
		{
			/// <summary>The message ID is set.</summary>
			Message = 0x01,

			/// <summary>The notification icon is set.</summary>
			Icon = 0x02,

			/// <summary>The tooltip is set.</summary>
			Tip = 0x04,

			/// <summary>State information 'IconState' is set. This applies to both 'IconState' and 'StateMask'.</summary>
			State = 0x08,

			/// <summary>The balloon ToolTip is set. Accordingly, the following members are set: 'szInfo', 'szInfoTitle', 'BalloonFlags', 'VersionOrTimeout'</summary>
			Info = 0x10,

			/// <summary>Internal identifier is set. Reserved.</summary>
			Guid = 0x20,

			/// <summary>
			/// Windows Vista (Shell32.dll version 6.0.6) and later. If the ToolTip cannot be displayed immediately, discard it.<br/>
			/// Use this flag for ToolTips that represent real-time information which would be meaningless or misleading if displayed at a later time.
			/// For example, a message that states "Your telephone is ringing."<br/> This modifies and must be combined with the <see cref="Info"/> flag.</summary>
			Realtime = 0x40,

			/// <summary>
			/// Windows Vista (Shell32.dll version 6.0.6) and later. Use the standard ToolTip. Normally, when uVersion is set
			/// to NOTIFYICON_VERSION_4, the standard ToolTip is replaced by the application-drawn pop-up user interface (UI).
			/// If the application wants to show the standard tooltip in that case, regardless of whether the on-hover UI is showing,
			/// it can specify NIF_SHOWTIP to indicate the standard tooltip should still be shown.<br/>
			/// Note that the NIF_SHOWTIP flag is effective until the next call to Shell_NotifyIcon.</summary>
			UseLegacyToolTips = 0x80
		}
		[Flags]
		public enum EIconState :uint
		{
			/// <summary>The icon is visible.</summary>
			Visible = 0x00,

			/// <summary>Hide the icon.</summary>
			Hidden = 0x01,

			/// <summary>The icon is shared</summary>
			Shared = 0x02, // currently not supported
		}
		public enum ENotifyIconBalloonFlags :uint
		{
			/// <summary>No icon is displayed.</summary>
			None = 0x00,

			/// <summary>An information icon is displayed.</summary>
			Info = 0x01,

			/// <summary>A warning icon is displayed.</summary>
			Warning = 0x02,

			/// <summary>An error icon is displayed.</summary>
			Error = 0x03,

			/// <summary>Windows XP Service Pack 2 (SP2) and later. Use a custom icon as the title icon.</summary>
			User = 0x04,

			/// <summary>Windows XP (Shell32.dll version 6.0) and later. Do not play the associated sound. Applies only to balloon ToolTips.</summary>
			NoSound = 0x10,

			/// <summary>
			/// Windows Vista (Shell32.dll version 6.0.6) and later. The large version of the icon should be used as the balloon icon.
			/// This corresponds to the icon with dimensions SM_CXICON x SM_CYICON. If this flag is not set, the icon with dimensions
			/// XM_CXSMICON x SM_CYSMICON is used.<br/>
			/// - This flag can be used with all stock icons.<br/>
			/// - Applications that use older customized icons (NIIF_USER with hIcon) must
			///   provide a new SM_CXICON x SM_CYICON version in the tray icon (hIcon). These
			///   icons are scaled down when they are displayed in the System Tray or
			///   System Control Area (SCA).<br/>
			/// - New customized icons (NIIF_USER with hBalloonIcon) must supply an
			///   SM_CXICON x SM_CYICON version in the supplied icon (hBalloonIcon).</summary>
			LargeIcon = 0x20,

			/// <summary>Windows 7 and later.</summary>
			RespectQuietTime = 0x80
		}
		#endregion

		#region Query Status QS_
		public const int QS_KEY              = 0x0001;
		public const int QS_MOUSEMOVE        = 0x0002;
		public const int QS_MOUSEBUTTON      = 0x0004;
		public const int QS_POSTMESSAGE      = 0x0008;
		public const int QS_TIMER            = 0x0010;
		public const int QS_PAINT            = 0x0020;
		public const int QS_SENDMESSAGE      = 0x0040;
		public const int QS_HOTKEY           = 0x0080;
		public const int QS_ALLPOSTMESSAGE   = 0x0100;
		public const int QS_RAWINPUT         = 0x0400;
		public const int QS_TOUCH            = 0x0800;
		public const int QS_POINTER          = 0x1000;
		public const int QS_MOUSE            = QS_MOUSEMOVE | QS_MOUSEBUTTON;
		public const int QS_INPUT            = QS_MOUSE | QS_KEY | QS_RAWINPUT | QS_TOUCH | QS_POINTER;
		public const int QS_ALLEVENTS        = QS_INPUT | QS_POSTMESSAGE | QS_TIMER | QS_PAINT | QS_HOTKEY;
		public const int QS_ALLINPUT         = QS_INPUT | QS_POSTMESSAGE | QS_TIMER | QS_PAINT | QS_HOTKEY | QS_SENDMESSAGE;
		#endregion

		#region Peek Message PM_
		public const int PM_NOREMOVE         = 0x0000;
		public const int PM_REMOVE           = 0x0001;
		public const int PM_NOYIELD          = 0x0002;
		public const int PM_QS_INPUT         = QS_INPUT << 16;
		public const int PM_QS_POSTMESSAGE   = (QS_POSTMESSAGE | QS_HOTKEY | QS_TIMER) << 16;
		public const int PM_QS_PAINT         = QS_PAINT << 16;
		public const int PM_QS_SENDMESSAGE   = QS_SENDMESSAGE << 16;
		[Flags] public enum EPeekMessageFlags
		{
			/// <summary>PM_NOREMOVE - Messages are not removed from the queue after processing by PeekMessage.</summary>
			NoRemove = PM_NOREMOVE,

			/// <summary>PM_REMOVE - Messages are removed from the queue after processing by PeekMessage.</summary>
			Remove = PM_REMOVE,

			/// <summary>PM_NOYIELD - Prevents the system from releasing any thread that is waiting for the caller to go idle (see WaitForInputIdle). Combine this value with either PM_NOREMOVE or PM_REMOVE.</summary>
			NoYield = PM_NOYIELD,
		}
		#endregion

		#region Progress Bar
		public const uint PBS_SMOOTH              = 0x01;
		public const uint PBS_VERTICAL            = 0x04;
		public const uint PBS_MARQUEE             = 0x08;
		public const uint PBS_SMOOTHREVERSE       = 0x10;

		public const uint PBM_SETRANGE            = (WM_USER+1);
		public const uint PBM_SETPOS              = (WM_USER+2);
		public const uint PBM_DELTAPOS            = (WM_USER+3);
		public const uint PBM_SETSTEP             = (WM_USER+4);
		public const uint PBM_STEPIT              = (WM_USER+5);
		public const uint PBM_SETRANGE32          = (WM_USER+6);  // lParam = high, wParam = low

		public const uint PBM_GETRANGE            = (WM_USER+7);    // wParam = return (TRUE ? low : high). lParam = PPBRANGE or NULL
		public const uint PBM_GETPOS              = (WM_USER+8);
		public const uint PBM_SETBARCOLOR         = (WM_USER+9);    // lParam = bar color
		public const uint PBM_SETMARQUEE          = (WM_USER+10);
		public const uint PBM_GETSTEP             = (WM_USER+13);
		public const uint PBM_GETBKCOLOR          = (WM_USER+14);
		public const uint PBM_GETBARCOLOR         = (WM_USER+15);
		public const uint PBM_SETSTATE            = (WM_USER+16); // wParam = PBST_[State] (NORMAL, ERROR, PAUSED)
		public const uint PBM_GETSTATE            = (WM_USER+17);
		public const uint PBM_SETBKCOLOR          = CCM_SETBKCOLOR; // lParam = bkColor

		public const int PBST_NORMAL = 0x0001;
		public const int PBST_ERROR  = 0x0002;
		public const int PBST_PAUSED = 0x0003;
		#endregion

		#region Redraw Window RDW_
		public const int RDW_INVALIDATE = 0x0001;
		public const int RDW_INTERNALPAINT = 0x0002;
		public const int RDW_ERASE = 0x0004;
		public const int RDW_VALIDATE = 0x0008;
		public const int RDW_NOINTERNALPAINT = 0x0010;
		public const int RDW_NOERASE = 0x0020;
		public const int RDW_NOCHILDREN = 0x0040;
		public const int RDW_ALLCHILDREN = 0x0080;
		public const int RDW_UPDATENOW = 0x0100;
		public const int RDW_ERASENOW = 0x0200;
		public const int RDW_FRAME = 0x0400;
		public const int RDW_NOFRAME = 0x0800;
		#endregion

		#region Rich Edit Control
		//public const uint EM_GETLIMITTEXT                 = (WM_USER + 37);
		//public const uint EM_POSFROMCHAR                  = (WM_USER + 38);
		//public const uint EM_CHARFROMPOS                  = (WM_USER + 39);
		//public const uint EM_SCROLLCARET                  = (WM_USER + 49);
		public const uint EM_CANPASTE                     = (WM_USER + 50);
		public const uint EM_DISPLAYBAND                  = (WM_USER + 51);
		public const uint EM_EXGETSEL                     = (WM_USER + 52);
		public const uint EM_EXLIMITTEXT                  = (WM_USER + 53);
		public const uint EM_EXLINEFROMCHAR               = (WM_USER + 54);
		public const uint EM_EXSETSEL                     = (WM_USER + 55);
		public const uint EM_FINDTEXT                     = (WM_USER + 56);
		//public const uint EM_FORMATRANGE                  = (WM_USER + 57);
		public const uint EM_GETCHARFORMAT                = (WM_USER + 58);
		public const uint EM_GETEVENTMASK                 = (WM_USER + 59);
		public const uint EM_GETOLEINTERFACE              = (WM_USER + 60);
		public const uint EM_GETPARAFORMAT                = (WM_USER + 61);
		public const uint EM_GETSELTEXT                   = (WM_USER + 62);
		public const uint EM_HIDESELECTION                = (WM_USER + 63);
		public const uint EM_PASTESPECIAL                 = (WM_USER + 64);
		public const uint EM_REQUESTRESIZE                = (WM_USER + 65);
		public const uint EM_SELECTIONTYPE                = (WM_USER + 66);
		public const uint EM_SETBKGNDCOLOR                = (WM_USER + 67);
		public const uint EM_SETCHARFORMAT                = (WM_USER + 68);
		public const uint EM_SETEVENTMASK                 = (WM_USER + 69);
		public const uint EM_SETOLECALLBACK               = (WM_USER + 70);
		public const uint EM_SETPARAFORMAT                = (WM_USER + 71);
		public const uint EM_SETTARGETDEVICE              = (WM_USER + 72);
		public const uint EM_STREAMIN                     = (WM_USER + 73);
		public const uint EM_STREAMOUT                    = (WM_USER + 74);
		public const uint EM_GETTEXTRANGE                 = (WM_USER + 75);
		public const uint EM_FINDWORDBREAK                = (WM_USER + 76);
		public const uint EM_SETOPTIONS                   = (WM_USER + 77);
		public const uint EM_GETOPTIONS                   = (WM_USER + 78);
		public const uint EM_FINDTEXTEX                   = (WM_USER + 79);
		public const uint EM_GETWORDBREAKPROCEX           = (WM_USER + 80);
		public const uint EM_SETWORDBREAKPROCEX           = (WM_USER + 81);

		// RichEdit 2.0 messages
		public const uint EM_SETUNDOLIMIT                 = (WM_USER + 82);
		public const uint EM_REDO                         = (WM_USER + 84);
		public const uint EM_CANREDO                      = (WM_USER + 85);
		public const uint EM_GETUNDONAME                  = (WM_USER + 86);
		public const uint EM_GETREDONAME                  = (WM_USER + 87);
		public const uint EM_STOPGROUPTYPING              = (WM_USER + 88);
		public const uint EM_SETTEXTMODE                  = (WM_USER + 89);
		public const uint EM_GETTEXTMODE                  = (WM_USER + 90);

		public const uint EM_GETSCROLLPOS                 = (WM_USER + 221);
		public const uint EM_SETSCROLLPOS                 = (WM_USER + 222);
		#endregion

		#region RI_MOUSE_
		public const uint RI_MOUSE_LEFT_BUTTON_DOWN   = 0x0001; // Left Button changed to down.
		public const uint RI_MOUSE_LEFT_BUTTON_UP     = 0x0002; // Left Button changed to up.
		public const uint RI_MOUSE_RIGHT_BUTTON_DOWN  = 0x0004; // Right Button changed to down.
		public const uint RI_MOUSE_RIGHT_BUTTON_UP    = 0x0008; // Right Button changed to up.
		public const uint RI_MOUSE_MIDDLE_BUTTON_DOWN = 0x0010; // Middle Button changed to down.
		public const uint RI_MOUSE_MIDDLE_BUTTON_UP   = 0x0020; // Middle Button changed to up.

		public const uint RI_MOUSE_BUTTON_1_DOWN      = RI_MOUSE_LEFT_BUTTON_DOWN;
		public const uint RI_MOUSE_BUTTON_1_UP        = RI_MOUSE_LEFT_BUTTON_UP;
		public const uint RI_MOUSE_BUTTON_2_DOWN      = RI_MOUSE_RIGHT_BUTTON_DOWN;
		public const uint RI_MOUSE_BUTTON_2_UP        = RI_MOUSE_RIGHT_BUTTON_UP;
		public const uint RI_MOUSE_BUTTON_3_DOWN      = RI_MOUSE_MIDDLE_BUTTON_DOWN;
		public const uint RI_MOUSE_BUTTON_3_UP        = RI_MOUSE_MIDDLE_BUTTON_UP;

		public const uint RI_MOUSE_BUTTON_4_DOWN      = 0x0040;
		public const uint RI_MOUSE_BUTTON_4_UP        = 0x0080;
		public const uint RI_MOUSE_BUTTON_5_DOWN      = 0x0100;
		public const uint RI_MOUSE_BUTTON_5_UP        = 0x0200;
		public const uint RI_MOUSE_WHEEL              = 0x0400;
		public const uint RI_MOUSE_HWHEEL             = 0x0800;
		#endregion

		#region RID_
		public const uint RID_INPUT = 0x10000003;
		public const uint RID_HEADER = 0x10000005;
		#endregion

		#region RIM_
		public const uint RIM_TYPEMOUSE = 0;
		public const uint RIM_TYPEKEYBOARD = 1;
		public const uint RIM_TYPEHID = 2;
		public const uint RIM_TYPEMAX = 2;
		#endregion

		#region RIDEV_
		public const uint RIDEV_REMOVE            = 0x00000001;
		public const uint RIDEV_EXCLUDE           = 0x00000010;
		public const uint RIDEV_PAGEONLY          = 0x00000020;
		public const uint RIDEV_NOLEGACY          = 0x00000030;
		public const uint RIDEV_INPUTSINK         = 0x00000100;
		public const uint RIDEV_CAPTUREMOUSE      = 0x00000200; // effective when mouse nolegacy is specified, otherwise it would be an error
		public const uint RIDEV_NOHOTKEYS         = 0x00000200; // effective for keyboard.
		public const uint RIDEV_APPKEYS           = 0x00000400; // effective for keyboard.
		public const uint RIDEV_EXINPUTSINK       = 0x00001000;
		public const uint RIDEV_DEVNOTIFY         = 0x00002000;
		public const uint RIDEV_EXMODEMASK        = 0x000000F0;
		#endregion

		#region Scroll Bar SB_
		public enum ScrollBarDirection
		{
			SB_HORZ = 0,
			SB_VERT = 1,
			SB_CTL = 2,
			SB_BOTH = 3
		}

		// These are the same as System.Windows.Forms.ScrollEventType
		public const int SB_LINEUP           = 0;
		public const int SB_LINELEFT         = 0;
		public const int SB_LINEDOWN         = 1;
		public const int SB_LINERIGHT        = 1;
		public const int SB_PAGEUP           = 2;
		public const int SB_PAGELEFT         = 2;
		public const int SB_PAGEDOWN         = 3;
		public const int SB_PAGERIGHT        = 3;
		public const int SB_THUMBPOSITION    = 4;
		public const int SB_THUMBTRACK       = 5;
		public const int SB_TOP              = 6;
		public const int SB_LEFT             = 6;
		public const int SB_BOTTOM           = 7;
		public const int SB_RIGHT            = 7;
		public const int SB_ENDSCROLL        = 8;
		#endregion

		#region Scroll info flags SIF_
		public enum ScrollInfoMask :uint
		{
			SIF_RANGE = 0x1,
			SIF_PAGE = 0x2,
			SIF_POS = 0x4,
			SIF_DISABLENOSCROLL = 0x8,
			SIF_TRACKPOS = 0x10,
			SIF_ALL = SIF_RANGE + SIF_PAGE + SIF_POS + SIF_TRACKPOS
		}
		#endregion

		#region Set Window Position SWP_
		public const int SWP_NOSIZE = 0x0001;
		public const int SWP_NOMOVE = 0x0002;
		public const int SWP_NOZORDER = 0x0004;
		public const int SWP_NOREDRAW = 0x0008;
		public const int SWP_NOACTIVATE = 0x0010;
		public const int SWP_FRAMECHANGED = 0x0020;
		public const int SWP_SHOWWINDOW = 0x0040;
		public const int SWP_HIDEWINDOW = 0x0080;
		public const int SWP_NOCOPYBITS = 0x0100;
		public const int SWP_NOOWNERZORDER = 0x0200;
		public const int SWP_NOSENDCHANGING = 0x0400;
		public const int SWP_DRAWFRAME = SWP_FRAMECHANGED;
		public const int SWP_NOREPOSITION = SWP_NOOWNERZORDER;
		public const int SWP_DEFERERASE = 0x2000;
		public const int SWP_ASYNCWINDOWPOS = 0x4000;
		[Flags] public enum EWindowPos :int
		{
			None = 0,
			NoSize = SWP_NOSIZE,
			NoMove = SWP_NOMOVE,
			NoZorder = SWP_NOZORDER,
			NoRedraw = SWP_NOREDRAW, // Note: this means "the system isn't going to redraw, so you have to"
			NoActivate = SWP_NOACTIVATE,
			FrameChanged = SWP_FRAMECHANGED,
			ShowWindow = SWP_SHOWWINDOW,
			HideWindow = SWP_HIDEWINDOW,
			NoCopyBits = SWP_NOCOPYBITS,
			NoOwnerZOrder = SWP_NOOWNERZORDER,
			NoSendChanging = SWP_NOSENDCHANGING,
			DrawFrame = SWP_DRAWFRAME,
			NoReposition = SWP_NOREPOSITION,
			DeferErase = SWP_DEFERERASE,
			AsyncWindowpos = SWP_ASYNCWINDOWPOS,
			NoClientSize = 0x0800, // SWP_NOCLIENTSIZE (don't send WM_SIZE)
			NoClientMove = 0x1000, // SWP_NOCLIENTMOVE (don't send WM_MOVE)
			StateChange = 0x8000, // SWP_STATECHANGED (minimized, maximised, etc)
			allow_bitops = 0,
		}
		#endregion

		#region Shell File Operations
		// Shell File Operations
		public const int FO_MOVE   = 0x0001;
		public const int FO_COPY   = 0x0002;
		public const int FO_DELETE = 0x0003;
		public const int FO_RENAME = 0x0004;

		// SHFILEOPSTRUCT.fFlags and IFileOperation::SetOperationFlags() flag values
		public const int FOF_MULTIDESTFILES        = 0x0001;
		public const int FOF_CONFIRMMOUSE          = 0x0002;
		public const int FOF_SILENT                = 0x0004;  // don't display progress UI (confirm prompts may be displayed still)
		public const int FOF_RENAMEONCOLLISION     = 0x0008;  // automatically rename the source files to avoid the collisions
		public const int FOF_NOCONFIRMATION        = 0x0010;  // don't display confirmation UI, assume "yes" for cases that can be bypassed, "no" for those that can not
		public const int FOF_WANTMAPPINGHANDLE     = 0x0020;  // Fill in SHFILEOPSTRUCT.hNameMappings.  Must be freed using SHFreeNameMappings
		public const int FOF_ALLOWUNDO             = 0x0040;  // enable undo including Recycle behavior for IFileOperation::Delete()
		public const int FOF_FILESONLY             = 0x0080;  // only operate on the files (non folders), both files and folders are assumed without this
		public const int FOF_SIMPLEPROGRESS        = 0x0100;  // means don't show names of files
		public const int FOF_NOCONFIRMMKDIR        = 0x0200;  // don't dispplay confirmatino UI before making any needed directories, assume "Yes" in these cases
		public const int FOF_NOERRORUI             = 0x0400;  // don't put up error UI, other UI may be displayed, progress, confirmations
		public const int FOF_NOCOPYSECURITYATTRIBS = 0x0800;  // dont copy file security attributes (ACLs)
		public const int FOF_NORECURSION           = 0x1000;  // don't recurse into directories for operations that would recurse
		public const int FOF_NO_CONNECTED_ELEMENTS = 0x2000;  // don't operate on connected elements ("xxx_files" folders that go with .htm files)
		public const int FOF_WANTNUKEWARNING       = 0x4000;  // during delete operation, warn if object is being permanently destroyed instead of recycling (partially overrides FOF_NOCONFIRMATION)
		public const int FOF_NORECURSEREPARSE      = 0x8000; // deprecated; the operations engine always does the right thing on FolderLink objects (symlinks, reparse points, folder shortcuts)
		public const int FOF_NO_UI                 = (FOF_SILENT | FOF_NOCONFIRMATION | FOF_NOERRORUI | FOF_NOCONFIRMMKDIR); // don't display any UI at all

		public enum FDAP
		{
			BOTTOM = 0x00000000,
			TOP    = 0x00000001,
		}
		public enum FDE_SHAREVIOLATION_RESPONSE
		{
			FDESVR_DEFAULT = 0x00000000,
			FDESVR_ACCEPT  = 0x00000001,
			FDESVR_REFUSE  = 0x00000002
		}
		public enum FDE_OVERWRITE_RESPONSE
		{
			FDEOR_DEFAULT = 0x00000000,
			FDEOR_ACCEPT  = 0x00000001,
			FDEOR_REFUSE  = 0x00000002
		}
		public enum SIATTRIBFLAGS
		{
			AND       = 0x00000001 , // if multiple items and the attributes together.
			OR        = 0x00000002 , // if multiple items or the attributes together.
			APPCOMPAT = 0x00000003 , // Call GetAttributes directly on the ShellFolder for multiple attributes
		}
		public enum SIGDN :uint
		{
			NORMALDISPLAY               = 0x00000000, // SHGDN_NORMAL
			PARENTRELATIVEPARSING       = 0x80018001, // SHGDN_INFOLDER | SHGDN_FORPARSING
			DESKTOPABSOLUTEPARSING      = 0x80028000, // SHGDN_FORPARSING
			PARENTRELATIVEEDITING       = 0x80031001, // SHGDN_INFOLDER | SHGDN_FOREDITING
			DESKTOPABSOLUTEEDITING      = 0x8004c000, // SHGDN_FORPARSING | SHGDN_FORADDRESSBAR
			FILESYSPATH                 = 0x80058000, // SHGDN_FORPARSING
			URL                         = 0x80068000, // SHGDN_FORPARSING
			PARENTRELATIVEFORADDRESSBAR = 0x8007c001, // SHGDN_INFOLDER | SHGDN_FORPARSING | SHGDN_FORADDRESSBAR
			PARENTRELATIVE              = 0x80080001  // SHGDN_INFOLDER
		}
		[Flags] public enum FOS :uint
		{
			OVERWRITEPROMPT    = 0x00000002,
			STRICTFILETYPES    = 0x00000004,
			NOCHANGEDIR        = 0x00000008,
			PICKFOLDERS        = 0x00000020,
			FORCEFILESYSTEM    = 0x00000040, // Ensure that items returned are filesystem items.
			ALLNONSTORAGEITEMS = 0x00000080, // Allow choosing items that have no storage.
			NOVALIDATE         = 0x00000100,
			ALLOWMULTISELECT   = 0x00000200,
			PATHMUSTEXIST      = 0x00000800,
			FILEMUSTEXIST      = 0x00001000,
			CREATEPROMPT       = 0x00002000,
			SHAREAWARE         = 0x00004000,
			NOREADONLYRETURN   = 0x00008000,
			NOTESTFILECREATE   = 0x00010000,
			HIDEMRUPLACES      = 0x00020000,
			HIDEPINNEDPLACES   = 0x00040000,
			NODEREFERENCELINKS = 0x00100000,
			DONTADDTORECENT    = 0x02000000,
			FORCESHOWHIDDEN    = 0x10000000,
			DEFAULTNOMINIMODE  = 0x20000000
		}
		public enum CDCONTROLSTATE
		{
			CDCS_INACTIVE = 0x00000000,
			CDCS_ENABLED  = 0x00000001,
			CDCS_VISIBLE  = 0x00000002
		}
		#endregion

		#region Show Window SW_
		public const int SW_HIDE = 0;
		public const int SW_SHOWNORMAL = 1;
		public const int SW_NORMAL = 1;
		public const int SW_SHOWMINIMIZED = 2;
		public const int SW_SHOWMAXIMIZED = 3;
		public const int SW_MAXIMIZE = 3;
		public const int SW_SHOWNOACTIVATE = 4;
		public const int SW_SHOW = 5;
		public const int SW_MINIMIZE = 6;
		public const int SW_SHOWMINNOACTIVE = 7;
		public const int SW_SHOWNA = 8;
		public const int SW_RESTORE = 9;
		public const int SW_SHOWDEFAULT = 10;
		public const int SW_FORCEMINIMIZE = 11;
		public const int SW_MAX = 11;

		// WM_SHOWWINDOW message constants
		public const int SW_PARENTCLOSING = 1;
		public const int SW_OTHERZOOM = 2;
		public const int SW_PARENTOPENING = 3;
		public const int SW_OTHERUNZOOM = 4;
		#endregion

		#region System Metrics
		public enum ESystemMetrics :int
		{
			SM_CXSCREEN = 0,      // The width of the screen of the primary display monitor, in pixels.This is the same value obtained by calling GetDeviceCaps as follows: GetDeviceCaps(hdcPrimaryMonitor, HORZRES).
			SM_ARRANGE = 56,     // The flags that specify how the system arranged minimized windows. For more information, see the Remarks section in this topic.
			SM_CLEANBOOT                   = 67,     // The value that specifies how the system is started: 0 Normal boot, 1 Fail-safe boot, 2 Fail-safe with network boot, A fail-safe boot (also called SafeBoot, Safe Mode, or Clean Boot) bypasses the user startup files.
			SM_CMONITORS                   = 80,     // The number of display monitors on a desktop.For more information, see the Remarks section in this topic.
			SM_CMOUSEBUTTONS               = 43,     // The number of buttons on a mouse, or zero if no mouse is installed.
			SM_CONVERTIBLESLATEMODE        = 0x2003, // Reflects the state of the laptop or slate mode, 0 for Slate Mode and non-zero otherwise. When this system metric changes, the system sends a broadcast message via WM_SETTINGCHANGE with "ConvertibleSlateMode" in the LPARAM. Note that this system metric doesn't apply to desktop PCs. In that case, use GetAutoRotationState.
			SM_CXBORDER                    = 5,      // The width of a window border, in pixels.This is equivalent to the SM_CXEDGE value for windows with the 3-D look.
			SM_CXCURSOR                    = 13,     // The width of a cursor, in pixels.The system cannot create cursors of other sizes.
			SM_CXDLGFRAME                  = 7,      // This value is the same as SM_CXFIXEDFRAME.
			SM_CXDOUBLECLK                 = 36,     // The width of the rectangle around the location of a first click in a double-click sequence, in pixels.The second click must occur within the rectangle that is defined by SM_CXDOUBLECLK and SM_CYDOUBLECLK for the system to consider the two clicks a double-click.The two clicks must also occur within a specified time. To set the width of the double-click rectangle, call SystemParametersInfo with SPI_SETDOUBLECLKWIDTH.
			SM_CXDRAG                      = 68,     // The number of pixels on either side of a mouse-down point that the mouse pointer can move before a drag operation begins.This allows the user to click and release the mouse button easily without unintentionally starting a drag operation. If this value is negative, it is subtracted from the left of the mouse-down point and added to the right of it.
			SM_CXEDGE                      = 45,     // The width of a 3-D border, in pixels.This metric is the 3-D counterpart of SM_CXBORDER.
			SM_CXFIXEDFRAME                = 7,      // The thickness of the frame around the perimeter of a window that has a caption but is not sizable, in pixels.SM_CXFIXEDFRAME is the height of the horizontal border, and SM_CYFIXEDFRAME is the width of the vertical border. This value is the same as SM_CXDLGFRAME.
			SM_CXFOCUSBORDER               = 83,     // The width of the left and right edges of the focus rectangle that the DrawFocusRect draws. This value is in pixels. Windows 2000:  This value is not supported.
			SM_CXFRAME                     = 32,     // This value is the same as SM_CXSIZEFRAME.
			SM_CXFULLSCREEN                = 16,     // The width of the client area for a full-screen window on the primary display monitor, in pixels.To get the coordinates of the portion of the screen that is not obscured by the system taskbar or by application desktop toolbars, call the SystemParametersInfo function with the SPI_GETWORKAREA value.
			SM_CXHSCROLL                   = 21,     // The width of the arrow bitmap on a horizontal scroll bar, in pixels.
			SM_CXHTHUMB                    = 10,     // The width of the thumb box in a horizontal scroll bar, in pixels.
			SM_CXICON                      = 11,     // The default width of an icon, in pixels.The LoadIcon function can load only icons with the dimensions that SM_CXICON and SM_CYICON specifies.
			SM_CXICONSPACING               = 38,     // The width of a grid cell for items in large icon view, in pixels.Each item fits into a rectangle of size SM_CXICONSPACING by SM_CYICONSPACING when arranged.This value is always greater than or equal to SM_CXICON.
			SM_CXMAXIMIZED                 = 61,     // The default width, in pixels, of a maximized top-level window on the primary display monitor.
			SM_CXMAXTRACK                  = 59,     // The default maximum width of a window that has a caption and sizing borders, in pixels.This metric refers to the entire desktop.The user cannot drag the window frame to a size larger than these dimensions. A window can override this value by processing the WM_GETMINMAXINFO message.
			SM_CXMENUCHECK                 = 71,     // The width of the default menu check-mark bitmap, in pixels.
			SM_CXMENUSIZE                  = 54,     // The width of menu bar buttons, such as the child window close button that is used in the multiple document interface, in pixels.
			SM_CXMIN                       = 28,     // The minimum width of a window, in pixels.
			SM_CXMINIMIZED                 = 57,     // The width of a minimized window, in pixels.
			SM_CXMINSPACING                = 47,     // The width of a grid cell for a minimized window, in pixels.Each minimized window fits into a rectangle this size when arranged.This value is always greater than or equal to SM_CXMINIMIZED.
			SM_CXMINTRACK                  = 34,     // The minimum tracking width of a window, in pixels.The user cannot drag the window frame to a size smaller than these dimensions. A window can override this value by processing the WM_GETMINMAXINFO message.
			SM_CXPADDEDBORDER              = 92,     // The amount of border padding for captioned windows, in pixels. Windows XP/2000:  This value is not supported.
			SM_CXSIZE                      = 30,     // The width of a button in a window caption or title bar, in pixels.
			SM_CXSIZEFRAME                 = 32,     // The thickness of the sizing border around the perimeter of a window that can be resized, in pixels.SM_CXSIZEFRAME is the width of the horizontal border, and SM_CYSIZEFRAME is the height of the vertical border. This value is the same as SM_CXFRAME.
			SM_CXSMICON                    = 49,     // The recommended width of a small icon, in pixels.Small icons typically appear in window captions and in small icon view.
			SM_CXSMSIZE                    = 52,     // The width of small caption buttons, in pixels.
			SM_CXVIRTUALSCREEN             = 78,     // The width of the virtual screen, in pixels.The virtual screen is the bounding rectangle of all display monitors.The SM_XVIRTUALSCREEN metric is the coordinates for the left side of the virtual screen.
			SM_CXVSCROLL                   = 2,      // The width of a vertical scroll bar, in pixels.
			SM_CYBORDER                    = 6,      // The height of a window border, in pixels.This is equivalent to the SM_CYEDGE value for windows with the 3-D look.
			SM_CYCAPTION                   = 4,      // The height of a caption area, in pixels.
			SM_CYCURSOR                    = 14,     // The height of a cursor, in pixels.The system cannot create cursors of other sizes.
			SM_CYDLGFRAME                  = 8,      // This value is the same as SM_CYFIXEDFRAME.
			SM_CYDOUBLECLK                 = 37,     // The height of the rectangle around the location of a first click in a double-click sequence, in pixels.The second click must occur within the rectangle defined by SM_CXDOUBLECLK and SM_CYDOUBLECLK for the system to consider the two clicks a double-click.The two clicks must also occur within a specified time. To set the height of the double-click rectangle, call SystemParametersInfo with SPI_SETDOUBLECLKHEIGHT.
			SM_CYDRAG                      = 69,     // The number of pixels above and below a mouse-down point that the mouse pointer can move before a drag operation begins.This allows the user to click and release the mouse button easily without unintentionally starting a drag operation. If this value is negative, it is subtracted from above the mouse-down point and added below it.
			SM_CYEDGE                      = 46,     // The height of a 3-D border, in pixels.This is the 3-D counterpart of SM_CYBORDER.
			SM_CYFIXEDFRAME                = 8,      // The thickness of the frame around the perimeter of a window that has a caption but is not sizable, in pixels.SM_CXFIXEDFRAME is the height of the horizontal border, and SM_CYFIXEDFRAME is the width of the vertical border. This value is the same as SM_CYDLGFRAME.
			SM_CYFOCUSBORDER               = 84,     // The height of the top and bottom edges of the focus rectangle drawn by DrawFocusRect.This value is in pixels. Windows 2000:  This value is not supported.
			SM_CYFRAME                     = 33,     // This value is the same as SM_CYSIZEFRAME.
			SM_CYFULLSCREEN                = 17,     // The height of the client area for a full-screen window on the primary display monitor, in pixels.To get the coordinates of the portion of the screen not obscured by the system taskbar or by application desktop toolbars, call the SystemParametersInfo function with the SPI_GETWORKAREA value.
			SM_CYHSCROLL                   = 3,      // The height of a horizontal scroll bar, in pixels.
			SM_CYICON                      = 12,     // The default height of an icon, in pixels.The LoadIcon function can load only icons with the dimensions SM_CXICON and SM_CYICON.
			SM_CYICONSPACING               = 39,     // The height of a grid cell for items in large icon view, in pixels.Each item fits into a rectangle of size SM_CXICONSPACING by SM_CYICONSPACING when arranged.This value is always greater than or equal to SM_CYICON.
			SM_CYKANJIWINDOW               = 18,     // For double byte character set versions of the system, this is the height of the Kanji window at the bottom of the screen, in pixels.
			SM_CYMAXIMIZED                 = 62,     // The default height, in pixels, of a maximized top-level window on the primary display monitor.
			SM_CYMAXTRACK                  = 60,     // The default maximum height of a window that has a caption and sizing borders, in pixels.This metric refers to the entire desktop.The user cannot drag the window frame to a size larger than these dimensions. A window can override this value by processing the WM_GETMINMAXINFO message.
			SM_CYMENU                      = 15,     // The height of a single-line menu bar, in pixels.
			SM_CYMENUCHECK                 = 72,     // The height of the default menu check-mark bitmap, in pixels.
			SM_CYMENUSIZE                  = 55,     // The height of menu bar buttons, such as the child window close button that is used in the multiple document interface, in pixels.
			SM_CYMIN                       = 29,     // The minimum height of a window, in pixels.
			SM_CYMINIMIZED                 = 58,     // The height of a minimized window, in pixels.
			SM_CYMINSPACING                = 48,     // The height of a grid cell for a minimized window, in pixels.Each minimized window fits into a rectangle this size when arranged.This value is always greater than or equal to SM_CYMINIMIZED.
			SM_CYMINTRACK                  = 35,     // The minimum tracking height of a window, in pixels.The user cannot drag the window frame to a size smaller than these dimensions. A window can override this value by processing the WM_GETMINMAXINFO message.
			SM_CYSCREEN                    = 1,      // The height of the screen of the primary display monitor, in pixels.This is the same value obtained by calling GetDeviceCaps as follows: GetDeviceCaps(hdcPrimaryMonitor, VERTRES).
			SM_CYSIZE                      = 31,     // The height of a button in a window caption or title bar, in pixels.
			SM_CYSIZEFRAME                 = 33,     // The thickness of the sizing border around the perimeter of a window that can be resized, in pixels.SM_CXSIZEFRAME is the width of the horizontal border, and SM_CYSIZEFRAME is the height of the vertical border. This value is the same as SM_CYFRAME.
			SM_CYSMCAPTION                 = 51,     // The height of a small caption, in pixels.
			SM_CYSMICON                    = 50,     // The recommended height of a small icon, in pixels.Small icons typically appear in window captions and in small icon view.
			SM_CYSMSIZE                    = 53,     // The height of small caption buttons, in pixels.
			SM_CYVIRTUALSCREEN             = 79,     // The height of the virtual screen, in pixels.The virtual screen is the bounding rectangle of all display monitors.The SM_YVIRTUALSCREEN metric is the coordinates for the top of the virtual screen.
			SM_CYVSCROLL                   = 20,     // The height of the arrow bitmap on a vertical scroll bar, in pixels.
			SM_CYVTHUMB                    = 9,      // The height of the thumb box in a vertical scroll bar, in pixels.
			SM_DBCSENABLED                 = 42,     // Nonzero if User32.dll supports DBCS; otherwise, 0.
			SM_DEBUG                       = 22,     // Nonzero if the debug version of User.exe is installed; otherwise, 0.
			SM_DIGITIZER                   = 94,     // Nonzero if the current operating system is Windows 7 or Windows Server 2008 R2 and the Tablet PC Input service is started; otherwise, 0. The return value is a bitmask that specifies the type of digitizer input supported by the device.For more information, see Remarks. Windows Server 2008, Windows Vista and Windows XP/2000:  This value is not supported.
			SM_IMMENABLED                  = 82,     // Nonzero if Input Method Manager/Input Method Editor features are enabled; otherwise, 0. SM_IMMENABLED indicates whether the system is ready to use a Unicode-based IME on a Unicode application.To ensure that a language-dependent IME works, check SM_DBCSENABLED and the system ANSI code page. Otherwise the ANSI-to-Unicode conversion may not be performed correctly, or some components like fonts or registry settings may not be present.
			SM_MAXIMUMTOUCHES              = 95,     // Nonzero if there are digitizers in the system; otherwise, 0. SM_MAXIMUMTOUCHES returns the aggregate maximum of the maximum number of contacts supported by every digitizer in the system.If the system has only single-touch digitizers, the return value is 1. If the system has multi-touch digitizers, the return value is the number of simultaneous contacts the hardware can provide. Windows Server 2008, Windows Vista and Windows XP/2000:  This value is not supported.
			SM_MEDIACENTER                 = 87,     // Nonzero if the current operating system is the Windows XP, Media Center Edition, 0 if not.
			SM_MENUDROPALIGNMENT           = 40,     // Nonzero if drop-down menus are right-aligned with the corresponding menu-bar item; 0 if the menus are left-aligned.
			SM_MIDEASTENABLED              = 74,     // Nonzero if the system is enabled for Hebrew and Arabic languages, 0 if not.
			SM_MOUSEPRESENT                = 19,     // Nonzero if a mouse is installed; otherwise, 0. This value is rarely zero, because of support for virtual mice and because some systems detect the presence of the port instead of the presence of a mouse.
			SM_MOUSEHORIZONTALWHEELPRESENT = 91,     // Nonzero if a mouse with a horizontal scroll wheel is installed; otherwise 0.
			SM_MOUSEWHEELPRESENT           = 75,     // Nonzero if a mouse with a vertical scroll wheel is installed; otherwise 0.
			SM_NETWORK                     = 63,     // The least significant bit is set if a network is present; otherwise, it is cleared.The other bits are reserved for future use.
			SM_PENWINDOWS                  = 41,     // Nonzero if the Microsoft Windows for Pen computing extensions are installed; zero otherwise.
			SM_REMOTECONTROL               = 0x2001, // This system metric is used in a Terminal Services environment to determine if the current Terminal Server session is being remotely controlled.Its value is nonzero if the current session is remotely controlled; otherwise, 0. You can use terminal services management tools such as Terminal Services Manager(tsadmin.msc) and shadow.exe to control a remote session.When a session is being remotely controlled, another user can view the contents of that session and potentially interact with it.
			SM_REMOTESESSION               = 0x1000, // This system metric is used in a Terminal Services environment. If the calling process is associated with a Terminal Services client session, the return value is nonzero.If the calling process is associated with the Terminal Services console session, the return value is 0. Windows Server 2003 and Windows XP:  The console session is not necessarily the physical console.For more information, see WTSGetActiveConsoleSessionId.
			SM_SAMEDISPLAYFORMAT           = 81,     // Nonzero if all the display monitors have the same color format, otherwise, 0. Two displays can have the same bit depth, but different color formats. For example, the red, green, and blue pixels can be encoded with different numbers of bits, or those bits can be located in different places in a pixel color value.
			SM_SECURE                      = 44,     // This system metric should be ignored; it always returns 0.
			SM_SERVERR2                    = 89,     // The build number if the system is Windows Server 2003 R2; otherwise, 0.
			SM_SHOWSOUNDS                  = 70,     // Nonzero if the user requires an application to present information visually in situations where it would otherwise present the information only in audible form; otherwise, 0.
			SM_SHUTTINGDOWN                = 0x2000, // Nonzero if the current session is shutting down; otherwise, 0. Windows 2000:  This value is not supported.
			SM_SLOWMACHINE                 = 73,     // Nonzero if the computer has a low-end (slow) processor; otherwise, 0.
			SM_STARTER                     = 88,     // Nonzero if the current operating system is Windows 7 Starter Edition, Windows Vista Starter, or Windows XP Starter Edition; otherwise, 0.
			SM_SWAPBUTTON                  = 23,     // Nonzero if the meanings of the left and right mouse buttons are swapped; otherwise, 0.
			SM_SYSTEMDOCKED                = 0x2004, // Reflects the state of the docking mode, 0 for Undocked Mode and non-zero otherwise.When this system metric changes, the system sends a broadcast message via WM_SETTINGCHANGE with "SystemDockMode" in the LPARAM.
			SM_TABLETPC                    = 86,     // Nonzero if the current operating system is the Windows XP Tablet PC edition or if the current operating system is Windows Vista or Windows 7 and the Tablet PC Input service is started; otherwise, 0. The SM_DIGITIZER setting indicates the type of digitizer input supported by a device running Windows 7 or Windows Server 2008 R2.For more information, see Remarks.
			SM_XVIRTUALSCREEN              = 76,     // The coordinates for the left side of the virtual screen.The virtual screen is the bounding rectangle of all display monitors.The SM_CXVIRTUALSCREEN metric is the width of the virtual screen.
			SM_YVIRTUALSCREEN              = 77,     // The coordinates for the top of the virtual screen.The virtual screen is the bounding rectangle of all display monitors.The SM_CYVIRTUALSCREEN metric is the height of the virtual screen.
		}
		#endregion

		#region Text Mode TM_
		public const int TM_PLAINTEXT       = 1;
		public const int TM_RICHTEXT        = 2; // default behaviour
		public const int TM_SINGLELEVELUNDO = 4;
		public const int TM_MULTILEVELUNDO  = 8; // default behaviour
		public const int TM_SINGLECODEPAGE  = 16;
		public const int TM_MULTICODEPAGE   = 32; // default behaviour
		#endregion

		#region Virtual key VK_
		// VK_0 - VK_9 are the same as ASCII '0' - '9' (0x30 - 0x39)
		// 0x40 : unassigned
		// VK_A - VK_Z are the same as ASCII 'A' - 'Z' (0x41 - 0x5A)
		public const byte VK_LBUTTON = 0x01;
		public const byte VK_RBUTTON = 0x02;
		public const byte VK_CANCEL = 0x03;
		public const byte VK_MBUTTON = 0x04;    /* NOT contiguous with L & RBUTTON */
		public const byte VK_XBUTTON1 = 0x05;    /* NOT contiguous with L & RBUTTON */
		public const byte VK_XBUTTON2 = 0x06;    /* NOT contiguous with L & RBUTTON */
		public const byte VK_BACK = 0x08;
		public const byte VK_TAB = 0x09;
		public const byte VK_CLEAR = 0x0C;
		public const byte VK_RETURN = 0x0D;
		public const byte VK_SHIFT = 0x10;
		public const byte VK_CONTROL = 0x11;
		public const byte VK_MENU = 0x12;
		public const byte VK_PAUSE = 0x13;
		public const byte VK_CAPITAL = 0x14;
		public const byte VK_KANA = 0x15;
		public const byte VK_HANGEUL = 0x15;  /* old name - should be here for compatibility */
		public const byte VK_HANGUL = 0x15;
		public const byte VK_JUNJA = 0x17;
		public const byte VK_FINAL = 0x18;
		public const byte VK_HANJA = 0x19;
		public const byte VK_KANJI = 0x19;
		public const byte VK_ESCAPE = 0x1B;
		public const byte VK_CONVERT = 0x1C;
		public const byte VK_NONCONVERT = 0x1D;
		public const byte VK_ACCEPT = 0x1E;
		public const byte VK_MODECHANGE = 0x1F;
		public const byte VK_SPACE = 0x20;
		public const byte VK_PRIOR = 0x21;
		public const byte VK_NEXT = 0x22;
		public const byte VK_END = 0x23;
		public const byte VK_HOME = 0x24;
		public const byte VK_LEFT = 0x25;
		public const byte VK_UP = 0x26;
		public const byte VK_RIGHT = 0x27;
		public const byte VK_DOWN = 0x28;
		public const byte VK_SELECT = 0x29;
		public const byte VK_PRINT = 0x2A;
		public const byte VK_EXECUTE = 0x2B;
		public const byte VK_SNAPSHOT = 0x2C;
		public const byte VK_INSERT = 0x2D;
		public const byte VK_DELETE = 0x2E;
		public const byte VK_HELP = 0x2F;
		public const byte VK_LWIN = 0x5B;
		public const byte VK_RWIN = 0x5C;
		public const byte VK_APPS = 0x5D;
		public const byte VK_SLEEP = 0x5F;
		public const byte VK_NUMPAD0 = 0x60;
		public const byte VK_NUMPAD1 = 0x61;
		public const byte VK_NUMPAD2 = 0x62;
		public const byte VK_NUMPAD3 = 0x63;
		public const byte VK_NUMPAD4 = 0x64;
		public const byte VK_NUMPAD5 = 0x65;
		public const byte VK_NUMPAD6 = 0x66;
		public const byte VK_NUMPAD7 = 0x67;
		public const byte VK_NUMPAD8 = 0x68;
		public const byte VK_NUMPAD9 = 0x69;
		public const byte VK_MULTIPLY = 0x6A;
		public const byte VK_ADD = 0x6B;
		public const byte VK_SEPARATOR = 0x6C;
		public const byte VK_SUBTRACT = 0x6D;
		public const byte VK_DECIMAL = 0x6E;
		public const byte VK_DIVIDE = 0x6F;
		public const byte VK_F1 = 0x70;
		public const byte VK_F2 = 0x71;
		public const byte VK_F3 = 0x72;
		public const byte VK_F4 = 0x73;
		public const byte VK_F5 = 0x74;
		public const byte VK_F6 = 0x75;
		public const byte VK_F7 = 0x76;
		public const byte VK_F8 = 0x77;
		public const byte VK_F9 = 0x78;
		public const byte VK_F10 = 0x79;
		public const byte VK_F11 = 0x7A;
		public const byte VK_F12 = 0x7B;
		public const byte VK_F13 = 0x7C;
		public const byte VK_F14 = 0x7D;
		public const byte VK_F15 = 0x7E;
		public const byte VK_F16 = 0x7F;
		public const byte VK_F17 = 0x80;
		public const byte VK_F18 = 0x81;
		public const byte VK_F19 = 0x82;
		public const byte VK_F20 = 0x83;
		public const byte VK_F21 = 0x84;
		public const byte VK_F22 = 0x85;
		public const byte VK_F23 = 0x86;
		public const byte VK_F24 = 0x87;
		public const byte VK_NUMLOCK = 0x90;
		public const byte VK_SCROLL = 0x91;
		public const byte VK_OEM_NEC_EQUAL = 0x92;   // '=' key on numpad
		public const byte VK_OEM_FJ_JISHO = 0x92;   // 'Dictionary' key
		public const byte VK_OEM_FJ_MASSHOU = 0x93;   // 'Unregister word' key
		public const byte VK_OEM_FJ_TOUROKU = 0x94;   // 'Register word' key
		public const byte VK_OEM_FJ_LOYA = 0x95;   // 'Left OYAYUBI' key
		public const byte VK_OEM_FJ_ROYA = 0x96;   // 'Right OYAYUBI' key
		public const byte VK_LSHIFT = 0xA0;
		public const byte VK_RSHIFT = 0xA1;
		public const byte VK_LCONTROL = 0xA2;
		public const byte VK_RCONTROL = 0xA3;
		public const byte VK_LMENU = 0xA4;
		public const byte VK_RMENU = 0xA5;
		public const byte VK_BROWSER_BACK = 0xA6;
		public const byte VK_BROWSER_FORWARD = 0xA7;
		public const byte VK_BROWSER_REFRESH = 0xA8;
		public const byte VK_BROWSER_STOP = 0xA9;
		public const byte VK_BROWSER_SEARCH = 0xAA;
		public const byte VK_BROWSER_FAVORITES = 0xAB;
		public const byte VK_BROWSER_HOME = 0xAC;
		public const byte VK_VOLUME_MUTE = 0xAD;
		public const byte VK_VOLUME_DOWN = 0xAE;
		public const byte VK_VOLUME_UP = 0xAF;
		public const byte VK_MEDIA_NEXT_TRACK = 0xB0;
		public const byte VK_MEDIA_PREV_TRACK = 0xB1;
		public const byte VK_MEDIA_STOP = 0xB2;
		public const byte VK_MEDIA_PLAY_PAUSE = 0xB3;
		public const byte VK_LAUNCH_MAIL = 0xB4;
		public const byte VK_LAUNCH_MEDIA_SELECT = 0xB5;
		public const byte VK_LAUNCH_APP1 = 0xB6;
		public const byte VK_LAUNCH_APP2 = 0xB7;
		public const byte VK_OEM_1 = 0xBA;   // ';:' for US
		public const byte VK_OEM_PLUS = 0xBB;   // '+' any country
		public const byte VK_OEM_COMMA = 0xBC;   // ',' any country
		public const byte VK_OEM_MINUS = 0xBD;   // '-' any country
		public const byte VK_OEM_PERIOD = 0xBE;   // '.' any country
		public const byte VK_OEM_2 = 0xBF;   // '/?' for US
		public const byte VK_OEM_3 = 0xC0;   // '`~' for US
		public const byte VK_OEM_4 = 0xDB;  //  '[{' for US
		public const byte VK_OEM_5 = 0xDC;  //  '\|' for US
		public const byte VK_OEM_6 = 0xDD;  //  ']}' for US
		public const byte VK_OEM_7 = 0xDE;  //  ''"' for US
		public const byte VK_OEM_8 = 0xDF;
		public const byte VK_OEM_AX = 0xE1;  //  'AX' key on Japanese AX kbd
		public const byte VK_OEM_102 = 0xE2;  //  "<>" or "\|" on RT 102-key kbd.
		public const byte VK_ICO_HELP = 0xE3;  //  Help key on ICO
		public const byte VK_ICO_00 = 0xE4;  //  00 key on ICO
		public const byte VK_PROCESSKEY = 0xE5;
		public const byte VK_ICO_CLEAR = 0xE6;
		public const byte VK_PACKET = 0xE7;
		public const byte VK_OEM_RESET = 0xE9;
		public const byte VK_OEM_JUMP = 0xEA;
		public const byte VK_OEM_PA1 = 0xEB;
		public const byte VK_OEM_PA2 = 0xEC;
		public const byte VK_OEM_PA3 = 0xED;
		public const byte VK_OEM_WSCTRL = 0xEE;
		public const byte VK_OEM_CUSEL = 0xEF;
		public const byte VK_OEM_ATTN = 0xF0;
		public const byte VK_OEM_FINISH = 0xF1;
		public const byte VK_OEM_COPY = 0xF2;
		public const byte VK_OEM_AUTO = 0xF3;
		public const byte VK_OEM_ENLW = 0xF4;
		public const byte VK_OEM_BACKTAB = 0xF5;
		public const byte VK_ATTN = 0xF6;
		public const byte VK_CRSEL = 0xF7;
		public const byte VK_EXSEL = 0xF8;
		public const byte VK_EREOF = 0xF9;
		public const byte VK_PLAY = 0xFA;
		public const byte VK_ZOOM = 0xFB;
		public const byte VK_NONAME = 0xFC;
		public const byte VK_PA1 = 0xFD;
		public const byte VK_OEM_CLEAR = 0xFE;
		#endregion

		#region Windows Messages
		public const int WM_NULL = 0x0000;
		public const int WM_CREATE = 0x0001;
		public const int WM_DESTROY = 0x0002;
		public const int WM_MOVE = 0x0003;
		public const int WM_SIZE = 0x0005;
		public const int WM_ACTIVATE = 0x0006;
		public const int WM_SETFOCUS = 0x0007;
		public const int WM_KILLFOCUS = 0x0008;
		public const int WM_ENABLE = 0x000A;
		public const int WM_SETREDRAW = 0x000B;
		public const int WM_SETTEXT = 0x000C;
		public const int WM_GETTEXT = 0x000D;
		public const int WM_GETTEXTLENGTH = 0x000E;
		public const int WM_PAINT = 0x000F;
		public const int WM_CLOSE = 0x0010;
		public const int WM_QUERYENDSESSION = 0x0011;
		public const int WM_QUERYOPEN = 0x0013;
		public const int WM_ENDSESSION = 0x0016;
		public const int WM_QUIT = 0x0012;
		public const int WM_ERASEBKGND = 0x0014;
		public const int WM_SYSCOLORCHANGE = 0x0015;
		public const int WM_SHOWWINDOW = 0x0018;
		public const int WM_WININICHANGE = 0x001A;
		public const int WM_SETTINGCHANGE = WM_WININICHANGE;
		public const int WM_DEVMODECHANGE = 0x001B;
		public const int WM_ACTIVATEAPP = 0x001C;
		public const int WM_FONTCHANGE = 0x001D;
		public const int WM_TIMECHANGE = 0x001E;
		public const int WM_CANCELMODE = 0x001F;
		public const int WM_SETCURSOR = 0x0020;
		public const int WM_MOUSEACTIVATE = 0x0021;
		public const int WM_CHILDACTIVATE = 0x0022;
		public const int WM_QUEUESYNC = 0x0023;
		public const int WM_GETMINMAXINFO = 0x0024;
		public const int WM_PAINTICON = 0x0026;
		public const int WM_ICONERASEBKGND = 0x0027;
		public const int WM_NEXTDLGCTL = 0x0028;
		public const int WM_SPOOLERSTATUS = 0x002A;
		public const int WM_DRAWITEM = 0x002B;
		public const int WM_MEASUREITEM = 0x002C;
		public const int WM_DELETEITEM = 0x002D;
		public const int WM_VKEYTOITEM = 0x002E;
		public const int WM_CHARTOITEM = 0x002F;
		public const int WM_SETFONT = 0x0030;
		public const int WM_GETFONT = 0x0031;
		public const int WM_SETHOTKEY = 0x0032;
		public const int WM_GETHOTKEY = 0x0033;
		public const int WM_QUERYDRAGICON = 0x0037;
		public const int WM_COMPAREITEM = 0x0039;
		public const int WM_GETOBJECT = 0x003D;
		public const int WM_COMPACTING = 0x0041;
		public const int WM_COMMNOTIFY = 0x0044;  /* no longer suported */
		public const int WM_WINDOWPOSCHANGING = 0x0046;
		public const int WM_WINDOWPOSCHANGED = 0x0047;
		public const int WM_POWER = 0x0048;
		public const int WM_COPYDATA = 0x004A;
		public const int WM_CANCELJOURNAL = 0x004B;
		public const int WM_NOTIFY = 0x004E;
		public const int WM_INPUTLANGCHANGEREQUEST = 0x0050;
		public const int WM_INPUTLANGCHANGE = 0x0051;
		public const int WM_TCARD = 0x0052;
		public const int WM_HELP = 0x0053;
		public const int WM_USERCHANGED = 0x0054;
		public const int WM_NOTIFYFORMAT = 0x0055;
		public const int WM_CONTEXTMENU = 0x007B;
		public const int WM_STYLECHANGING = 0x007C;
		public const int WM_STYLECHANGED = 0x007D;
		public const int WM_DISPLAYCHANGE = 0x007E;
		public const int WM_GETICON = 0x007F;
		public const int WM_SETICON = 0x0080;
		public const int WM_NCCREATE = 0x0081;
		public const int WM_NCDESTROY = 0x0082;
		public const int WM_NCCALCSIZE = 0x0083;
		public const int WM_NCHITTEST = 0x0084;
		public const int WM_NCPAINT = 0x0085;
		public const int WM_NCACTIVATE = 0x0086;
		public const int WM_GETDLGCODE = 0x0087;
		public const int WM_SYNCPAINT = 0x0088;
		public const int WM_NCMOUSEMOVE = 0x00A0;
		public const int WM_NCLBUTTONDOWN = 0x00A1;
		public const int WM_NCLBUTTONUP = 0x00A2;
		public const int WM_NCLBUTTONDBLCLK = 0x00A3;
		public const int WM_NCRBUTTONDOWN = 0x00A4;
		public const int WM_NCRBUTTONUP = 0x00A5;
		public const int WM_NCRBUTTONDBLCLK = 0x00A6;
		public const int WM_NCMBUTTONDOWN = 0x00A7;
		public const int WM_NCMBUTTONUP = 0x00A8;
		public const int WM_NCMBUTTONDBLCLK = 0x00A9;
		public const int WM_NCXBUTTONDOWN = 0x00AB;
		public const int WM_NCXBUTTONUP = 0x00AC;
		public const int WM_NCXBUTTONDBLCLK = 0x00AD;
		public const int WM_INPUT_DEVICE_CHANGE = 0x00FE;
		public const int WM_INPUT = 0x00FF;
		public const int WM_KEYDOWN = 0x0100;
		public const int WM_KEYUP = 0x0101;
		public const int WM_CHAR = 0x0102;
		public const int WM_DEADCHAR = 0x0103;
		public const int WM_SYSKEYDOWN = 0x0104;
		public const int WM_SYSKEYUP = 0x0105;
		public const int WM_SYSCHAR = 0x0106;
		public const int WM_SYSDEADCHAR = 0x0107;
		public const int WM_CONVERTREQUESTEX = 0x0108;
		public const int WM_UNICHAR = 0x0109;
		public const int WM_IME_STARTCOMPOSITION = 0x010D;
		public const int WM_IME_ENDCOMPOSITION = 0x010E;
		public const int WM_IME_COMPOSITION = 0x010F;
		public const int WM_IME_KEYLAST = 0x010F;
		public const int WM_INITDIALOG = 0x0110;
		public const int WM_COMMAND = 0x0111;
		public const int WM_SYSCOMMAND = 0x0112;
		public const int WM_TIMER = 0x0113;
		public const int WM_HSCROLL = 0x0114;
		public const int WM_VSCROLL = 0x0115;
		public const int WM_INITMENU = 0x0116;
		public const int WM_INITMENUPOPUP = 0x0117;
		public const int WM_MENUSELECT = 0x011F;
		public const int WM_MENUCHAR = 0x0120;
		public const int WM_ENTERIDLE = 0x0121;
		public const int WM_MENURBUTTONUP = 0x0122;
		public const int WM_MENUDRAG = 0x0123;
		public const int WM_MENUGETOBJECT = 0x0124;
		public const int WM_UNINITMENUPOPUP = 0x0125;
		public const int WM_MENUCOMMAND = 0x0126;
		public const int WM_CHANGEUISTATE = 0x0127;
		public const int WM_UPDATEUISTATE = 0x0128;
		public const int WM_QUERYUISTATE = 0x0129;
		public const int WM_CTLCOLORMSGBOX = 0x0132;
		public const int WM_CTLCOLOREDIT = 0x0133;
		public const int WM_CTLCOLORLISTBOX = 0x0134;
		public const int WM_CTLCOLORBTN = 0x0135;
		public const int WM_CTLCOLORDLG = 0x0136;
		public const int WM_CTLCOLORSCROLLBAR = 0x0137;
		public const int WM_CTLCOLORSTATIC = 0x0138;
		public const int MN_GETHMENU = 0x01E1;
		public const int WM_MOUSEMOVE = 0x0200;
		public const int WM_LBUTTONDOWN = 0x0201;
		public const int WM_LBUTTONUP = 0x0202;
		public const int WM_LBUTTONDBLCLK = 0x0203;
		public const int WM_RBUTTONDOWN = 0x0204;
		public const int WM_RBUTTONUP = 0x0205;
		public const int WM_RBUTTONDBLCLK = 0x0206;
		public const int WM_MBUTTONDOWN = 0x0207;
		public const int WM_MBUTTONUP = 0x0208;
		public const int WM_MBUTTONDBLCLK = 0x0209;
		public const int WM_MOUSEWHEEL = 0x020A;
		public const int WM_XBUTTONDOWN = 0x020B;
		public const int WM_XBUTTONUP = 0x020C;
		public const int WM_XBUTTONDBLCLK = 0x020D;
		public const int WM_DPICHANGED = 0x02E0;
		public const int WM_MOUSEHWHEEL = 0x020E;
		public const int WM_PARENTNOTIFY = 0x0210;
		public const int WM_ENTERMENULOOP = 0x0211;
		public const int WM_EXITMENULOOP = 0x0212;
		public const int WM_NEXTMENU = 0x0213;
		public const int WM_SIZING = 0x0214;
		public const int WM_CAPTURECHANGED = 0x0215;
		public const int WM_MOVING = 0x0216;
		public const int WM_DEVICECHANGE = 0x0219;
		public const int WM_MDICREATE = 0x0220;
		public const int WM_MDIDESTROY = 0x0221;
		public const int WM_MDIACTIVATE = 0x0222;
		public const int WM_MDIRESTORE = 0x0223;
		public const int WM_MDINEXT = 0x0224;
		public const int WM_MDIMAXIMIZE = 0x0225;
		public const int WM_MDITILE = 0x0226;
		public const int WM_MDICASCADE = 0x0227;
		public const int WM_MDIICONARRANGE = 0x0228;
		public const int WM_MDIGETACTIVE = 0x0229;
		public const int WM_MDISETMENU = 0x0230;
		public const int WM_ENTERSIZEMOVE = 0x0231;
		public const int WM_EXITSIZEMOVE = 0x0232;
		public const int WM_DROPFILES = 0x0233;
		public const int WM_MDIREFRESHMENU = 0x0234;
		public const int WM_POINTERDEVICECHANGE = 0x0238;
		public const int WM_POINTERDEVICEINRANGE = 0x0239;
		public const int WM_POINTERDEVICEOUTOFRANGE = 0x023A;
		public const int WM_TOUCH = 0x0240;
		public const int WM_NCPOINTERUPDATE = 0x0241;
		public const int WM_NCPOINTERDOWN = 0x0242;
		public const int WM_NCPOINTERUP = 0x0243;
		public const int WM_POINTERUPDATE = 0x0245;
		public const int WM_POINTERDOWN = 0x0246;
		public const int WM_POINTERUP = 0x0247;
		public const int WM_POINTERENTER = 0x0249;
		public const int WM_POINTERLEAVE = 0x024A;
		public const int WM_POINTERACTIVATE = 0x024B;
		public const int WM_POINTERCAPTURECHANGED = 0x024C;
		public const int WM_TOUCHHITTESTING = 0x024D;
		public const int WM_POINTERWHEEL = 0x024E;
		public const int DM_POINTERHITTEST = 0x0250;
		public const int WM_POINTERHWHEEL = 0x024F;
		public const int WM_IME_SETCONTEXT = 0x0281;
		public const int WM_IME_NOTIFY = 0x0282;
		public const int WM_IME_CONTROL = 0x0283;
		public const int WM_IME_COMPOSITIONFULL = 0x0284;
		public const int WM_IME_SELECT = 0x0285;
		public const int WM_IME_CHAR = 0x0286;
		public const int WM_IME_REQUEST = 0x0288;
		public const int WM_IME_KEYDOWN = 0x0290;
		public const int WM_IME_KEYUP = 0x0291;
		public const int WM_MOUSEHOVER = 0x02A1;
		public const int WM_MOUSELEAVE = 0x02A3;
		public const int WM_NCMOUSEHOVER = 0x02A0;
		public const int WM_NCMOUSELEAVE = 0x02A2;
		public const int WM_WTSSESSION_CHANGE = 0x02B1;
		public const int WM_TABLET_FIRST = 0x02c0;
		public const int WM_TABLET_LAST = 0x02df;
		public const int WM_CUT = 0x0300;
		public const int WM_COPY = 0x0301;
		public const int WM_PASTE = 0x0302;
		public const int WM_CLEAR = 0x0303;
		public const int WM_UNDO = 0x0304;
		public const int WM_RENDERFORMAT = 0x0305;
		public const int WM_RENDERALLFORMATS = 0x0306;
		public const int WM_DESTROYCLIPBOARD = 0x0307;
		public const int WM_DRAWCLIPBOARD = 0x0308;
		public const int WM_PAINTCLIPBOARD = 0x0309;
		public const int WM_VSCROLLCLIPBOARD = 0x030A;
		public const int WM_SIZECLIPBOARD = 0x030B;
		public const int WM_ASKCBFORMATNAME = 0x030C;
		public const int WM_CHANGECBCHAIN = 0x030D;
		public const int WM_HSCROLLCLIPBOARD = 0x030E;
		public const int WM_QUERYNEWPALETTE = 0x030F;
		public const int WM_PALETTEISCHANGING = 0x0310;
		public const int WM_PALETTECHANGED = 0x0311;
		public const int WM_HOTKEY = 0x0312;
		public const int WM_PRINT = 0x0317;
		public const int WM_PRINTCLIENT = 0x0318;
		public const int WM_APPCOMMAND = 0x0319;
		public const int WM_THEMECHANGED = 0x031A;
		public const int WM_CLIPBOARDUPDATE = 0x031D;
		public const int WM_DWMCOMPOSITIONCHANGED = 0x031E;
		public const int WM_DWMNCRENDERINGCHANGED = 0x031F;
		public const int WM_DWMCOLORIZATIONCOLORCHANGED = 0x0320;
		public const int WM_DWMWINDOWMAXIMIZEDCHANGE = 0x0321;
		public const int WM_GETTITLEBARINFOEX = 0x033F;
		public const int WM_HANDHELDFIRST = 0x0358;
		public const int WM_HANDHELDLAST = 0x035F;
		public const int WM_AFXFIRST = 0x0360;
		public const int WM_AFXLAST = 0x037F;
		public const int WM_PENWINFIRST = 0x0380;
		public const int WM_PENWINLAST = 0x038F;
		public const int WM_REFLECT = 0x2000;
		public const int WM_APP = 0x8000;
		public const int WM_USER = 0x0400;
		#endregion

		#region windows message high WH_
		public const int WH_MOUSE_LL = 14;
		public const int WH_KEYBOARD_LL = 13;
		public const int WH_MOUSE = 7;
		public const int WH_KEYBOARD = 2;
		#endregion

		#region WinEvent constants
		public const uint EVENT_SYSTEM_FOREGROUND = 0x0003;
		public const uint WINEVENT_OUTOFCONTEXT = 0x0000;
		public const uint WINEVENT_SKIPOWNPROCESS = 0x0002;
		#endregion

		#region WM_SIZE state values SIZE_
		public const int SIZE_RESTORED  = 0;
		public const int SIZE_MINIMIZED = 1;
		public const int SIZE_MAXIMIZED = 2;
		public const int SIZE_MAXSHOW   = 3;
		public const int SIZE_MAXHIDE   = 4;
		#endregion

		#region Window Styles WS_
		public const int WS_POPUP                         = unchecked((int)0x80000000); // The windows is a pop-up window. This style cannot be used with the WS_CHILD style.
		public const int WS_CHILD                         = unchecked(0x40000000); // The window is a child window. A window with this style cannot have a menu bar. This style cannot be used with the WS_POPUP style.
		public const int WS_CHILDWINDOW                   = unchecked(0x40000000); // Same as the WS_CHILD style.
		public const int WS_ICONIC                        = unchecked(0x20000000); // The window is initially minimized. Same as the WS_MINIMIZE style.
		public const int WS_MINIMIZE                      = unchecked(0x20000000); // The window is initially minimized. Same as the WS_ICONIC style.
		public const int WS_VISIBLE                       = unchecked(0x10000000); // The window is initially visible. This style can be turned on and off by using the ShowWindow or SetWindowPos function.
		public const int WS_DISABLED                      = unchecked(0x08000000); // The window is initially disabled. A disabled window cannot receive input from the user. To change this after a window has been created, use the EnableWindow function.
		public const int WS_CLIPSIBLINGS                  = unchecked(0x04000000); // Clips child windows relative to each other; that is, when a particular child window receives a WM_PAINT message, the WS_CLIPSIBLINGS style clips all other overlapping child windows out of the region of the child window to be updated. If WS_CLIPSIBLINGS is not specified and child windows overlap, it is possible, when drawing within the client area of a child window, to draw within the client area of a neighboring child window.
		public const int WS_CLIPCHILDREN                  = unchecked(0x02000000); // Excludes the area occupied by child windows when drawing occurs within the parent window. This style is used when creating the parent window.
		public const int WS_MAXIMIZE                      = unchecked(0x01000000); // The window is initially maximized.
		public const int WS_CAPTION                       = unchecked(0x00C00000); // The window has a title bar (includes the WS_BORDER style).
		public const int WS_BORDER                        = unchecked(0x00800000); // The window has a thin-line border.
		public const int WS_DLGFRAME                      = unchecked(0x00400000); // The window has a border of a style typically used with dialog boxes. A window with this style cannot have a title bar.
		public const int WS_VSCROLL                       = unchecked(0x00200000); // The window has a vertical scroll bar.
		public const int WS_HSCROLL                       = unchecked(0x00100000); // The window has a horizontal scroll bar.
		public const int WS_SYSMENU                       = unchecked(0x00080000); // The window has a window menu on its title bar. The WS_CAPTION style must also be specified.
		public const int WS_SIZEBOX                       = unchecked(0x00040000); // The window has a sizing border. Same as the WS_THICKFRAME style.
		public const int WS_THICKFRAME                    = unchecked(0x00040000); // The window has a sizing border. Same as the WS_SIZEBOX style.
		public const int WS_GROUP                         = unchecked(0x00020000); // The window is the first control of a group of controls. The group consists of this first control and all controls defined after it, up to the next control with the WS_GROUP style. The first control in each group usually has the WS_TABSTOP style so that the user can move from group to group. The user can subsequently change the keyboard focus from one control in the group to the next control in the group by using the direction keys. You can turn this style on and off to change dialog box navigation. To change this style after a window has been created, use the SetWindowLong function.
		public const int WS_MINIMIZEBOX                   = unchecked(0x00020000); // The window has a minimize button. Cannot be combined with the WS_EX_CONTEXTHELP style. The WS_SYSMENU style must also be specified.
		public const int WS_MAXIMIZEBOX                   = unchecked(0x00010000); // The window has a maximize button. Cannot be combined with the WS_EX_CONTEXTHELP style. The WS_SYSMENU style must also be specified.
		public const int WS_TABSTOP                       = unchecked(0x00010000); // The window is a control that can receive the keyboard focus when the user presses the TAB key. Pressing the TAB key changes the keyboard focus to the next control with the WS_TABSTOP style. You can turn this style on and off to change dialog box navigation. To change this style after a window has been created, use the SetWindowLong function. For user-created windows and modeless dialogs to work with tab stops, alter the message loop to call the IsDialogMessage function.
		public const int WS_TILED                         = unchecked(0x00000000); // The window is an overlapped window. An overlapped window has a title bar and a border. Same as the WS_OVERLAPPED style.
		public const int WS_OVERLAPPED                    = unchecked(0x00000000); // The window is an overlapped window. An overlapped window has a title bar and a border. Same as the WS_TILED style.
		public const int WS_POPUPWINDOW                   = unchecked(WS_POPUP | WS_BORDER | WS_SYSMENU); // The window is a pop-up window. The WS_CAPTION and WS_POPUPWINDOW styles must be combined to make the window menu visible.
		public const int WS_TILEDWINDOW                   = unchecked(WS_OVERLAPPED | WS_CAPTION | WS_SYSMENU | WS_THICKFRAME | WS_MINIMIZEBOX | WS_MAXIMIZEBOX); // The window is an overlapped window. Same as the WS_OVERLAPPEDWINDOW style.
		public const int WS_OVERLAPPEDWINDOW              = unchecked(WS_OVERLAPPED | WS_CAPTION | WS_SYSMENU | WS_THICKFRAME | WS_MINIMIZEBOX | WS_MAXIMIZEBOX); // The window is an overlapped window. Same as the WS_TILEDWINDOW style.
		#endregion

		#region Window Styles Extended WS_EX_ 
		public const int WS_EX_DLGMODALFRAME              = 0x00000001;
		public const int WS_EX_NOPARENTNOTIFY             = 0x00000004;
		public const int WS_EX_TOPMOST                    = 0x00000008;
		public const int WS_EX_ACCEPTFILES                = 0x00000010;
		public const int WS_EX_TRANSPARENT                = 0x00000020;
		public const int WS_EX_MDICHILD                   = 0x00000040;
		public const int WS_EX_TOOLWINDOW                 = 0x00000080;
		public const int WS_EX_WINDOWEDGE                 = 0x00000100;
		public const int WS_EX_CLIENTEDGE                 = 0x00000200;
		public const int WS_EX_CONTEXTHELP                = 0x00000400;
		public const int WS_EX_RIGHT                      = 0x00001000;
		public const int WS_EX_LEFT                       = 0x00000000;
		public const int WS_EX_RTLREADING                 = 0x00002000;
		public const int WS_EX_LTRREADING                 = 0x00000000;
		public const int WS_EX_LEFTSCROLLBAR              = 0x00004000;
		public const int WS_EX_RIGHTSCROLLBAR             = 0x00000000;
		public const int WS_EX_CONTROLPARENT              = 0x00010000;
		public const int WS_EX_STATICEDGE                 = 0x00020000;
		public const int WS_EX_APPWINDOW                  = 0x00040000;
		public const int WS_EX_OVERLAPPEDWINDOW           = WS_EX_WINDOWEDGE | WS_EX_CLIENTEDGE;
		public const int WS_EX_PALETTEWINDOW              = WS_EX_WINDOWEDGE | WS_EX_TOOLWINDOW | WS_EX_TOPMOST;
		public const int WS_EX_LAYERED                    = 0x00080000;
		public const int WS_EX_NOINHERITLAYOUT            = 0x00100000; // Disable inheritence of mirroring by children
		public const int WS_EX_NOREDIRECTIONBITMAP        = 0x00200000;
		public const int WS_EX_COMPOSITED                 = 0x02000000;
		public const int WS_EX_LAYOUTRTL                  = 0x00400000;
		public const int WS_EX_NOACTIVATE                 = 0x08000000;
		#endregion

		#region WM_NOTIFY codes
		public const uint NM_FIRST = unchecked(0U - 0U);       // generic to all controls
		public const uint NM_LAST = unchecked(0U - 99U);
		public const uint LVN_FIRST = unchecked(0U - 100U);       // listview
		public const uint LVN_LAST = unchecked(0U - 199U);
		public const uint HDN_FIRST = unchecked(0U - 300U);       // header
		public const uint HDN_LAST = unchecked(0U - 399U);
		public const uint TVN_FIRST = unchecked(0U - 400U);       // treeview
		public const uint TVN_LAST = unchecked(0U - 499U);
		public const uint TTN_FIRST = unchecked(0U - 520U);       // tooltips
		public const uint TTN_LAST = unchecked(0U - 549U);
		public const uint TCN_FIRST = unchecked(0U - 550U);       // tab control
		public const uint TCN_LAST = unchecked(0U - 580U);
		public const uint CDN_FIRST = unchecked(0U - 601U);       // common dialog (new)
		public const uint CDN_LAST = unchecked(0U - 699U);
		public const uint TBN_FIRST = unchecked(0U - 700U);       // toolbar
		public const uint TBN_LAST = unchecked(0U - 720U);
		public const uint UDN_FIRST = unchecked(0U - 721U);        // updown
		public const uint UDN_LAST = unchecked(0U - 729U);
		public const uint DTN_FIRST = unchecked(0U - 740U);       // datetimepick
		public const uint DTN_LAST = unchecked(0U - 745U);       // DTN_FIRST - 5
		public const uint MCN_FIRST = unchecked(0U - 746U);       // monthcal
		public const uint MCN_LAST = unchecked(0U - 752U);       // MCN_FIRST - 6
		public const uint DTN_FIRST2 = unchecked(0U - 753U);       // datetimepick2
		public const uint DTN_LAST2 = unchecked(0U - 799U);
		public const uint CBEN_FIRST = unchecked(0U - 800U);       // combo box ex
		public const uint CBEN_LAST = unchecked(0U - 830U);
		public const uint RBN_FIRST = unchecked(0U - 831U);       // rebar
		public const uint RBN_LAST = unchecked(0U - 859U);
		public const uint IPN_FIRST = unchecked(0U - 860U);       // internet address
		public const uint IPN_LAST = unchecked(0U - 879U);       // internet address
		public const uint SBN_FIRST = unchecked(0U - 880U);       // status bar
		public const uint SBN_LAST = unchecked(0U - 899U);
		public const uint PGN_FIRST = unchecked(0U - 900U);       // Pager Control
		public const uint PGN_LAST = unchecked(0U - 950U);
		public const uint WMN_FIRST = unchecked(0U - 1000U);
		public const uint WMN_LAST = unchecked(0U - 1200U);
		public const uint BCN_FIRST = unchecked(0U - 1250U);
		public const uint BCN_LAST = unchecked(0U - 1350U);
		public const uint TRBN_FIRST = unchecked(0U - 1501U);       // trackbar
		public const uint TRBN_LAST = unchecked(0U - 1519U);
		#endregion

		#region WM_SYSCOMMAND values
		public const uint SC_WPARAM_MASK = 0xFFF0; // In C# you need to mask the wparam with this as the lower bits contain magic stuff
		public const uint SC_SIZE = 0xF000; // Sizes the window.
		public const uint SC_MOVE = 0xF010; // Moves the window.
		public const uint SC_MINIMIZE = 0xF020; // Minimizes the window.
		public const uint SC_MAXIMIZE = 0xF030; // Maximizes the window.
		public const uint SC_NEXTWINDOW = 0xF040; // Moves to the next window.
		public const uint SC_PREVWINDOW = 0xF050; // Moves to the previous window.
		public const uint SC_CLOSE = 0xF060; // Closes the window.
		public const uint SC_VSCROLL = 0xF070; // Scrolls vertically.
		public const uint SC_HSCROLL = 0xF080; // Scrolls horizontally.
		public const uint SC_MOUSEMENU = 0xF090; // Retrieves the window menu as a result of a mouse click.
		public const uint SC_KEYMENU = 0xF100; // Retrieves the window menu as a result of a keystroke. For more information, see the Remarks section.
		public const uint SC_ARRANGE = 0xF110; //
		public const uint SC_RESTORE = 0xF120; // Restores the window to its normal position and size.
		public const uint SC_TASKLIST = 0xF130; // Activates the Start menu.
		public const uint SC_SCREENSAVE = 0xF140; // Executes the screen saver application specified in the [boot] section of the System.ini file.
		public const uint SC_HOTKEY = 0xF150; // Activates the window associated with the application-specified hot key. The lParam parameter identifies the window to activate.
		public const uint SC_DEFAULT = 0xF160; // Selects the default item; the user double-clicked the window menu.
		public const uint SC_MONITORPOWER = 0xF170; // Sets the state of the display. This command supports devices that have power-saving features, such as a battery-powered personal computer.
		public const uint SC_CONTEXTHELP = 0xF180; // Changes the cursor to a question mark with a pointer. If the user then clicks a control in the dialog box, the control receives a WM_HELP message.
		public const uint SC_SEPARATOR = 0xF00F; //
		#endregion

		#region WM_ACTIVATE state values WA_
		public const uint WA_INACTIVE = 0;
		public const uint WA_ACTIVE = 1;
		public const uint WA_CLICKACTIVE = 2;
		#endregion

		#endregion

		#region Windows Callback Functions
		[UnmanagedFunctionPointer(CallingConvention.StdCall)]
		public delegate IntPtr WNDPROC(HWND hwnd, int code, IntPtr wparam, IntPtr lparam);

		[UnmanagedFunctionPointer(CallingConvention.StdCall)]
		public delegate bool EnumWindowsProc(HWND hwnd, int lParam);
		#endregion

		/// <summary>Magic value used for message queue only windows</summary>
		public static readonly IntPtr HWND_MESSAGE = new(-3);

		/// <summary>INVALID_HANDLE_VALUE constant</summary>
		public static readonly IntPtr INVALID_HANDLE_VALUE = new(-1);

		/// <summary>A lazy created HWND for a STATIC window</summary>
		public static IntPtr ProxyParentHwnd
		{
			get
			{
				// This window is used to allow child windows to be created before the actual parent HWND is available.
				// See ScintillaControl as an example.
				if (m_proxy_parent_hwnd == IntPtr.Zero)
					m_proxy_parent_hwnd = User32.CreateWindow(0, "STATIC", string.Empty, 0, 0, 0, 1, 1, IntPtr.Zero, IntPtr.Zero, IntPtr.Zero, IntPtr.Zero);
				return m_proxy_parent_hwnd;
			}
		}
		private static IntPtr m_proxy_parent_hwnd;

		/// <summary>Returns the upper 16bits of a 32bit DWORD such as LPARAM or WPARAM</summary>
		public static int HiWord(long dword)
		{
			return (int)((dword >> 16) & 0xFFFF);
		}
		public static uint HiWord(ulong dword)
		{
			return (uint)((dword >> 16) & 0xFFFF);
		}
		public static int HiWord(IntPtr dword)
		{
			return (int)HiWord(dword.ToInt64());
		}

		/// <summary>Returns the lower 16bits of a 32bit DWORD such as LPARAM or WPARAM</summary>
		public static int LoWord(long dword)
		{
			return (int)(dword & 0xFFFF);
		}
		public static uint LoWord(ulong dword)
		{
			return (uint)(dword & 0xFFFF);
		}
		public static int LoWord(IntPtr dword)
		{
			return (int)LoWord(dword.ToInt64());
		}

		/// <summary>Convert a win32 system time to a date time offset</summary>
		public static DateTimeOffset ToDateTimeOffset(SYSTEMTIME st)
		{
			var ft = new FILETIME();
			if (!Kernel32.SystemTimeToFileTime(ref st, out ft))
				throw new Exception("Failed to convert system time to file time");

			return DateTimeOffset.FromFileTime(ft.value);
		}

		/// <summary>Convert a wparam into a mouse key (from the WM_LBUTTONDOWN event)</summary>
		public static EMouseBtns ToMouseKey(IntPtr wparam)
		{
			var mk = unchecked((EMouseBtns)LoWord(wparam));
			if (User32.GetKeyState(EKeyCodes.Menu) < 0) mk |= EMouseBtns.Alt;
			return mk;
		}

		/// <summary>Convert WinForms MouseButtons to win32 MK_ macro values</summary>
		public static int ToMKey(EMouseBtns btns)
		{
			// System.Windows.Forms.EKeys is the same as the win32 VK_ macros
			// System.Windows.Forms.MouseButtons *isn't* the same as MK_ macros however
			// nor is System.Windows.Input.MouseButton

			var mk = 0;
			if (btns.HasFlag(EMouseBtns.Left    )) mk |= MK_LBUTTON;
			if (btns.HasFlag(EMouseBtns.Right   )) mk |= MK_RBUTTON;
			if (btns.HasFlag(EMouseBtns.Middle  )) mk |= MK_MBUTTON;
			if (btns.HasFlag(EMouseBtns.XButton1)) mk |= MK_XBUTTON1;
			if (btns.HasFlag(EMouseBtns.XButton2)) mk |= MK_XBUTTON2;
			return mk;
		}

		/// <summary>Convert WParam to EKeyCodes</summary>
		public static EKeyCodes ToVKey(IntPtr wparam)
		{
			// System.Windows.Forms.EKeys is the same as the win32 VK_ macros
			// System.Windows.Forms.MouseButtons *isn't* the same as MK_ macros however
			// nor is System.Windows.Input.MouseButton

			var vk = (EKeyCodes)wparam;
			if (KeyDown(EKeyCodes.ShiftKey)) vk |= EKeyCodes.Shift;
			if (KeyDown(EKeyCodes.ControlKey)) vk |= EKeyCodes.Control;
			return vk;
		}

		/// <summary>Convert a wparam to a mouse wheel delta (WM_MOUSEWHEEL event)</summary>
		public static short ToMouseWheelDelta(IntPtr wparam)
		{
			return unchecked((short)HiWord(wparam));
		}

		/// <summary>Test the async state of a key</summary>
		public static bool KeyDownAsync(EKeyCodes vkey)
		{
			return (User32.GetAsyncKeyState(vkey) & 0x8000) != 0;
		}

		/// <summary>Test for key down using the key state for current message</summary>
		public static bool KeyDown(EKeyCodes vkey)
		{
			return (User32.GetKeyState(vkey) & 0x8000) != 0;
		}

		/// <summary>Detect a key press using its async state</summary>
		public static bool KeyPressAsync(EKeyCodes vkey)
		{
			if (!KeyDownAsync(vkey)) return false;
			while (KeyDownAsync(vkey)) Thread.Sleep(10);
			return true;
		}

		/// <summary>Convert a 'Keys' key value to a UNICODE char using the keyboard state of the last message. Returns true if 'vkey' can be converted and 'ch' is valid, false if not</summary>
		public static bool CharFromVKey(EKeyCodes vkey, out char ch)
		{
			// This can be called from WM_KEYDOWN to provide the actual character, instead of waiting for WM_CHAR

			ch = '\0';
			var keyboardState = new byte[256];
			User32.GetKeyboardState(keyboardState);

			var scan_code = User32.MapVirtualKey((uint)vkey, MAPVK_VK_TO_VSC);
			var sb = new StringBuilder(2);
			var r = User32.ToUnicode((uint)vkey, scan_code, keyboardState, sb, sb.Capacity, 0);
			if (r == 1) ch = sb[0];
			return r == 1;
		}

		/// <summary>True if this key is an array key</summary>
		public static bool IsArrowKey(this EKeyCodes vk)
		{
			return vk == EKeyCodes.Left || vk == EKeyCodes.Right || vk == EKeyCodes.Up || vk == EKeyCodes.Down;
		}

		/// <summary>Convert the LParam from WM_KEYDOWN, WM_KEYUP, WM_CHAR to usable data</summary>
		public struct KeyState
		{
			public uint Repeats;    // Number of skipped key repeats
			public uint ScanCode;   // OWM scan code
			public bool Extended;   // e.g. 1 if right-hand ALT, CTRL, etc
			public bool Alt;        // ALT is pressed
			public int  Transition; // 0(b00) = low-low, 1(b01) = low-hi, 2(b10) = hi-low, 3(b11) = hi-hi
			public KeyState(uint lparam)
			{
				Repeats    = ((lparam >>  0) & 0xffffU);
				ScanCode   = ((lparam >> 16) & 0x7fU);
				Extended   = ((lparam >> 24) & 1) != 0;
				Alt        = ((lparam >> 29) & 1) != 0;
				Transition =
					(((lparam >> 30) & 1) != 0 ? 2 : 0) |
					(((lparam >> 31) & 1) == 0 ? 1 : 0);
				//Transition = ((lparam & (1U << 30)) != 0 ? 2 : 0) | ((lparam & (1U << 31)) == 0 ? 1 : 0);
			}
			public KeyState(int lparam)
				:this((uint)lparam)
			{}
			public KeyState(IntPtr lparam)
				:this((uint)lparam)
			{}
		}

		/// <summary>Convert the WParam from WM_MOUSEWHEEL to usable data</summary>
		/// <remarks>Read the mouse wheel delta from a WParam</remarks>
		public struct WheelState(uint wparam)
		{
			public short Delta = (short)HiWord(wparam);
			public int MouseKey = LoWord(wparam); // An MK_ value

			public WheelState(int wparam)
				: this((uint)wparam)
			{ }
			public WheelState(IntPtr wparam)
				: this((uint)wparam)
			{}
		}

		/// <summary>Pack/Unpack a Point from an LPARAM</summary>
		public static IntPtr PointToLParam(Point pt)
		{
			return new IntPtr((((pt.Y & 0xffff) << 16) | (pt.X & 0xffff)));
		}
		public static Point LParamToPoint(IntPtr lparam)
		{
			return new Point(lparam.ToInt32() & 0xffff, lparam.ToInt32() >> 16);
		}
		public static Point LParamToPoint(uint lparam)
		{
			return LParamToPoint((IntPtr)lparam);
		}

		/// <summary>Return the window under a screen space point</summary>
		public static HWND WindowFromPoint(Point pt)
		{
			return WindowFromPoint(POINT.FromPoint(pt));
		}

		/// <summary>Return the control under the screen space point</summary>
		public static HWND ChildWindowFromPoint(HWND parent, Point pt, int cwp_flags)
		{
			return User32.ChildWindowFromPointEx(parent, POINT.FromPoint(pt), cwp_flags);
		}

		/// <summary>Centre a window to it's parent</summary>
		public static bool CenterWindow(HWND hwnd, Rectangle scn)
		{
			var parent = User32.GetParent(hwnd);
			if (parent == IntPtr.Zero)
				return false;

			var wnd_rect = User32.GetWindowRect(hwnd);
			var parent_rect = User32.GetWindowRect(parent);

			var w = wnd_rect.width;
			var h = wnd_rect.height;
			var x = parent_rect.left + (parent_rect.width - w) / 2;
			var y = parent_rect.top + (parent_rect.height - h) / 2;

			// Make sure that the dialog box never moves outside of the screen
			// Note: 'scn' = SystemInformation.VirtualScreen typically
			if (x < scn.Left) x = scn.Left;
			if (y < scn.Top) y = scn.Top;
			if (x + w > scn.Right) x = scn.Right - w;
			if (y + h > scn.Bottom) y = scn.Bottom - h;

			return User32.MoveWindow(hwnd, x, y, w, h, false);
		}

		/// <summary>Set or clear window styles</summary>
		public static void SetStyle(HWND hwnd, uint style, bool enabled)
		{
			var s = User32.GetWindowLong(hwnd, Win32.GWL_STYLE);
			var r = User32.SetWindowLong(hwnd, Win32.GWL_STYLE, Bit.SetBits(s, style, enabled));
			if (r == 0) throw new Win32Exception();
		}
		public static void SetStyleEx(HWND hwnd, uint style, bool enabled)
		{
			var s = User32.GetWindowLong(hwnd, Win32.GWL_EXSTYLE);
			User32.SetWindowLong(hwnd, Win32.GWL_EXSTYLE, Bit.SetBits(s, style, enabled));
		}

		/// <summary>Get/Set scroll bar position</summary>
		public static int GetScrollBarPos(IntPtr hWnd, int nBar)
		{
			var info = SCROLLINFO.Default;
			info.fMask = (int)ScrollInfoMask.SIF_POS;
			User32.GetScrollInfo(hWnd, nBar, ref info);
			return info.nPos;
		}
		public static void SetScrollBarPos(IntPtr hWnd, int nBar, int nPos, bool bRedraw)
		{
			var info = SCROLLINFO.Default;
			info.fMask = (int)ScrollInfoMask.SIF_POS;
			info.nPos = nPos;
			User32.SetScrollInfo(hWnd, nBar, ref info, bRedraw);
		}

		public delegate int HookProc(int nCode, int wParam, IntPtr lParam);

		/// <summary>Output a trace of wndproc messages to the debug output window</summary>
		public static void WndProcDebug(IntPtr hwnd, int msg, IntPtr wparam, IntPtr lparam, string name)
		{
			#if true
			using (Scope.Create(() => ++m_wnd_proc_nest, () => --m_wnd_proc_nest))
			{
				if (msg < Win32.WM_USER)
				{
					++m_msg_idx;
					var msg_str = DebugMessage(hwnd, msg, wparam, lparam);
					if (msg_str.HasValue())
					{
						for (var i = 1; i != m_wnd_proc_nest; ++i) Debug.Write("\t");
						Debug.WriteLine($"{m_msg_idx:d5}|{name}|{msg_str}");
					}
					if (m_msg_idx == 0)
						Debugger.Break();
				}
			}
			#endif
		}
		public static void WndProcDebug(ref MESSAGE m, string name)
		{
			WndProcDebug(m.hwnd, (int)m.message, m.wparam, m.lparam, name);
		}
		private static int m_wnd_proc_nest = 0; // Tracks how often WndProc has been called recursively
		private static int m_msg_idx = 0;

		/// <summary>Convert a wndproc message to a string</summary>
		public static string DebugMessage(ref MESSAGE msg)
		{
			return DebugMessage(msg.hwnd, (int)msg.message, msg.wparam, msg.lparam);
		}
		public static string DebugMessage(IntPtr hwnd, int msg, IntPtr wparam, IntPtr lparam)
		{
			var wparam_lo = LoWord(wparam);
			var wparam_hi = HiWord(wparam);
			var lparam_lo = LoWord(lparam);
			var lparam_hi = HiWord(lparam);

			var hdr = $"{MsgIdToString(msg)}(0x{msg:X4}):";
			var wnd = $" hwnd: 0x{hwnd:X8}";
			var wp = $"wparam: {wparam:X8}({wparam_hi:X4},{wparam_lo:X4})";
			var lp = $"lparam: {lparam:X8}({lparam_hi:X4},{lparam_lo:X4})";

			switch (msg)
			{
				default:
				{
					return $"{hdr} {wnd} {wp} {lp}";
				}
				case WM_KEYDOWN:
				case WM_KEYUP:
				case WM_CHAR:
				{
					return
						$"{hdr} " +
						$"'{(msg == WM_CHAR ? new string((char)wparam_lo, 1) : Enum<EKeyCodes>.ToString(wparam.ToInt64()))}'({wparam})  " +
						$"repeats:{lparam_lo}  " +
						$"state:{((lparam_hi & (1 << 13)) != 0 ? "ALT " : string.Empty)}  " +
						$"transition:{((lparam_hi & (1 << 14)) != 0 ? "1" : "0")}{((lparam_hi & (1 << 15)) != 0 ? "0" : "1")}";
				}
				case WM_LBUTTONDOWN:
				{
					return
						$"{hdr} button state = " +
						$"{((wparam_lo & MK_CONTROL) != 0 ? "|Ctrl" : "")}" +
						$"{((wparam_lo & MK_LBUTTON) != 0 ? "|LBtn" : "")}" +
						$"{((wparam_lo & MK_MBUTTON) != 0 ? "|MBtn" : "")}" +
						$"{((wparam_lo & MK_RBUTTON) != 0 ? "|RBtn" : "")}" +
						$"{((wparam_lo & MK_SHIFT) != 0 ? "|Shift" : "")}" +
						$"{((wparam_lo & MK_XBUTTON1) != 0 ? "|XBtn1" : "")}" +
						$"{((wparam_lo & MK_XBUTTON2) != 0 ? "|XBtn2" : "")}  " +
						$"x,y=({lparam_lo},{lparam_hi})";
				}
				case WM_ACTIVATEAPP:
				{
					return $"{hdr} {((int)wparam != 0 ? "ACTIVE" : "INACTIVE")} Other Thread: {lparam}";
				}
				case WM_ACTIVATE:
				{
					return $"{hdr} {(LoWord(wparam) == WA_ACTIVE ? "ACTIVE" : LoWord(wparam) == WA_INACTIVE ? "INACTIVE" : "Click ACTIVE")} Other Window: {lparam}";
				}
				case WM_NCACTIVATE:
				{
					return $"{hdr} {((int)wparam != 0 ? "ACTIVE" : "INACTIVE")} {lp}";
				}
				case WM_MOUSEACTIVATE:
				{
					return $"{hdr} top level parent window = {wparam}  {lp}";
				}
				case WM_SHOWWINDOW:
				{
					var reason = (int)lparam switch
					{
						SW_OTHERUNZOOM => "OtherUnzoom",
						SW_PARENTCLOSING => "ParentClosing",
						SW_OTHERZOOM => "OtherZoom",
						SW_PARENTOPENING => "ParentOpening",
						_ => "ShowWindow called",
					};
					return $"{hdr} {((int)wparam != 0 ? "VISIBLE" : "HIDDEN")} {reason}";
				}
				case WM_WINDOWPOSCHANGING:
				case WM_WINDOWPOSCHANGED:
				{
					var wpos = Marshal.PtrToStructure<WINDOWPOS>(lparam);
					return string.Format("{0} x,y=({1},{2}) size=({3},{4}) after={5} flags={6}{7}{8}{9}{10}{11}{12}{13}{14}{15}{16}{17}{18}"
						, hdr
						, wpos.x, wpos.y
						, wpos.cx, wpos.cy
						, wpos.hwndInsertAfter
						, wpos.flags.HasFlag(EWindowPos.DrawFrame) ? "|SWP_DRAWFRAME" : ""
						, wpos.flags.HasFlag(EWindowPos.FrameChanged) ? "|SWP_FRAMECHANGED" : ""
						, wpos.flags.HasFlag(EWindowPos.HideWindow) ? "|SWP_HIDEWINDOW" : ""
						, wpos.flags.HasFlag(EWindowPos.NoActivate) ? "|SWP_NOACTIVATE" : ""
						, wpos.flags.HasFlag(EWindowPos.NoCopyBits) ? "|SWP_NOCOPYBITS" : ""
						, wpos.flags.HasFlag(EWindowPos.NoMove) ? "|SWP_NOMOVE" : ""
						, wpos.flags.HasFlag(EWindowPos.NoOwnerZOrder) ? "|SWP_NOOWNERZORDER" : ""
						, wpos.flags.HasFlag(EWindowPos.NoRedraw) ? "|SWP_NOREDRAW" : ""
						, wpos.flags.HasFlag(EWindowPos.NoReposition) ? "|SWP_NOREPOSITION" : ""
						, wpos.flags.HasFlag(EWindowPos.NoSendChanging) ? "|SWP_NOSENDCHANGING" : ""
						, wpos.flags.HasFlag(EWindowPos.NoSize) ? "|SWP_NOSIZE" : ""
						, wpos.flags.HasFlag(EWindowPos.NoZorder) ? "|SWP_NOZORDER" : ""
						, wpos.flags.HasFlag(EWindowPos.ShowWindow) ? "|SWP_SHOWWINDOW" : "");
				}
				case WM_KILLFOCUS:
				{
					return $"{hdr} Focused Window: {wparam}";
				}
				case WM_NOTIFY:
				{
					var notify_type = "unknown";
					var nmhdr = Marshal.PtrToStructure<NMHDR>(lparam);
					if (NM_LAST <= nmhdr.code) notify_type = "NM";
					else if (LVN_LAST <= nmhdr.code) notify_type = "LVN";
					else if (HDN_LAST <= nmhdr.code) notify_type = "HDN";
					else if (TVN_LAST <= nmhdr.code) notify_type = "TVN";
					else if (TTN_LAST <= nmhdr.code) notify_type = "TTN";
					else if (TCN_LAST <= nmhdr.code) notify_type = "TCN";
					else if (CDN_LAST <= nmhdr.code) notify_type = "CDN";
					else if (TBN_LAST <= nmhdr.code) notify_type = "TBN";
					else if (UDN_LAST <= nmhdr.code) notify_type = "UDN";
					else if (DTN_LAST <= nmhdr.code) notify_type = "DTN";
					else if (MCN_LAST <= nmhdr.code) notify_type = "MCN";
					else if (DTN_LAST2 <= nmhdr.code) notify_type = "DTN";
					else if (CBEN_LAST <= nmhdr.code) notify_type = "CBEN";
					else if (RBN_LAST <= nmhdr.code) notify_type = "RBN";
					else if (IPN_LAST <= nmhdr.code) notify_type = "IPN";
					else if (SBN_LAST <= nmhdr.code) notify_type = "SBN";
					else if (PGN_LAST <= nmhdr.code) notify_type = "PGN";
					else if (WMN_LAST <= nmhdr.code) notify_type = "WMN";
					else if (BCN_LAST <= nmhdr.code) notify_type = "BCN";
					else if (TRBN_LAST <= nmhdr.code) notify_type = "TRBN";

					// Ignore
					if (nmhdr.code == LVN_HOTTRACK)
						return "";

					return string.Format("{0} SourceCtrlId = {1}  from_hWnd: {2}  from_id: {3}  code: {4}:{5}"
						, hdr
						, wparam
						, nmhdr.hwndFrom
						, nmhdr.idFrom
						, nmhdr.code
						, notify_type);
				}
				case WM_PARENTNOTIFY:
				{
					var details = LoWord(wparam) switch
					{
						WM_CREATE => $"Child Id:{HiWord(wparam)} hwnd:{LParamToPoint(lparam)}",
						WM_DESTROY => $"Child Id:{HiWord(wparam)} hwnd:{LParamToPoint(lparam)}",
						WM_LBUTTONDOWN => $"LButton {LParamToPoint(lparam)}",
						WM_MBUTTONDOWN => $"MButton {LParamToPoint(lparam)}",
						WM_RBUTTONDOWN => $"RButton {LParamToPoint(lparam)}",
						WM_XBUTTONDOWN => $"XButton btn:{HiWord(wparam)} {LParamToPoint(lparam)}",
						WM_POINTERDOWN => $"Pointer Down ptr:{HiWord(wparam)}",
						_ => $"Unexpected event. {HiWord(wparam)}",
					};
					return $"{hdr} evt: {MsgIdToString((int)LoWord(wparam))} {details}";
				}
				case WM_SYSKEYDOWN:
				{
					return $"{hdr} vk_key = {wparam} ({Enum<EKeyCodes>.ToString((int)wparam)})  Repeats: {lparam_lo}  lParam: {lparam}";
				}
				case WM_PAINT:
				{
					//	RECT r;
					//	::GetUpdateRect(hWnd, &r, FALSE);
					//	return "{0} update=({1},{2}) size=({3},{4})  HDC: {5}{6}"
					//		.Fmt(hdr
					//		,r.left ,r.top ,r.right - r.left ,r.bottom - r.top
					//		,wParam
					//		,newline);
					goto case WM_NULL;
				}
				case WM_TIMER:
				{
					goto case WM_NULL;
				}
				case WM_GETOBJECT:
				{
					goto case WM_NULL;
				}
				case WM_IME_REQUEST:
				{
					return $"{hdr} IME Request {(EIME_Request)wparam}  lParam: {lparam}";
				}
				case WM_IME_NOTIFY:
				{
					return $"{hdr} IME Notify {(EIME_Notification)wparam}  lParam: {lparam}";
				}
				case WM_IME_SETCONTEXT:
				{
					var active = (int)wparam != 0 ? "Active" : "Inactive";
					return $"{hdr} IME SetContext  Window: {active} Options: {(EIME_SetContextFlags)lparam}";
				}
				case WM_NCHITTEST:
				case WM_SETCURSOR:
				case WM_NCMOUSEMOVE:
				case WM_NCMOUSELEAVE:
				case WM_MOUSEMOVE:
				case WM_GETICON:
				//case EWinMsg::wm_UAHDRAWMENUITEM:
				//case EWinMsg::wm_UAHDRAWMENU:
				//case EWinMsg::wm_UAHINITMENU:
				//case EWinMsg::wm_DWMCOLORIZATIONCOLORCHANGED:
				//case EWinMsg::wm_UAHMEASUREMENUITEM:
				//case WM_CTLCOLORDLG:
				//case WM_AFXLAST:
				//case WM_ENTERIDLE:
				//case WM_ERASEBKGND:
				//case WM_PAINT:
				case WM_NULL:
					return "";//ignore
			}
		}

		/// <summary>Convert a message constant to a string</summary>
		public static string MsgIdToString(int msg_id)
		{
			if (!m_wm_name.TryGetValue(msg_id, out var name))
			{
				var fi = typeof(Win32).GetFields(BindingFlags.Public | BindingFlags.Static)
					.Where(x => x.IsLiteral)
					.Where(x => x.Name.StartsWith("WM_") || x.Name.StartsWith("EM_"))
					.FirstOrDefault(x =>
						{
							var val = x.GetValue(null);
							if (val is uint v0) return v0 == (uint)msg_id;
							if (val is int v1) return v1 == msg_id;
							return false;
						});

				name = fi != null ? fi.Name : string.Empty;
				m_wm_name.Add(msg_id, name);
			}
			return name;
		}
		private static Dictionary<int, string> m_wm_name = [];
	}
}
