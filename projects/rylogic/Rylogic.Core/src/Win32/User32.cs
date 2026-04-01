using System;
using System.ComponentModel;
using System.Runtime.InteropServices;
using System.Text;
using Microsoft.Win32.SafeHandles;
using Rylogic.Extn;
using Rylogic.Utility;
using HDC = System.IntPtr;
using HICON = System.IntPtr;
using HRGN = System.IntPtr;
using HWND = System.IntPtr;
using LPARAM = System.IntPtr;
using WPARAM = System.IntPtr;

namespace Rylogic.Interop.Win32
{
	public static class User32
	{
		/// <summary></summary>
		public static bool AppendMenu(IntPtr hMenu, uint uFlags, int uIDNewItem, string lpNewItem) => AppendMenu_(hMenu, uFlags, uIDNewItem, lpNewItem);
		public static bool AppendMenu(IntPtr hMenu, uint uFlags, IntPtr uIDNewItem, string lpNewItem) => AppendMenu_(hMenu, uFlags, uIDNewItem, lpNewItem);
		[DllImport("user32.dll", EntryPoint = "AppendMenuW", CharSet = CharSet.Unicode)]
		private static extern bool AppendMenu_(IntPtr hMenu, uint uFlags, int uIDNewItem, [MarshalAs(UnmanagedType.LPWStr)] string lpNewItem);
		[DllImport("user32.dll", EntryPoint = "AppendMenuW", CharSet = CharSet.Unicode)]
		private static extern bool AppendMenu_(IntPtr hMenu, uint uFlags, IntPtr uIDNewItem, [MarshalAs(UnmanagedType.LPWStr)] string lpNewItem);

		/// <summary></summary>
		public static IntPtr AttachThreadInput(IntPtr idAttach, IntPtr idAttachTo, int fAttach) => AttachThreadInput_(idAttach, idAttachTo, fAttach);
		[DllImport("user32.dll", EntryPoint = "AttachThreadInput")]
		public static extern IntPtr AttachThreadInput_(IntPtr idAttach, IntPtr idAttachTo, int fAttach);

		/// <summary></summary>
		[DllImport("user32.dll")]
		public static extern int CallNextHookEx(int idHook, int nCode, int wParam, IntPtr lParam);

		[DllImport("user32.dll", EntryPoint = "CheckMenuItem")]
		public static extern int CheckMenuItem(IntPtr hMenu, int uIDCheckItem, int uCheck);

		[DllImport("user32.dll")]
		public static extern HWND ChildWindowFromPointEx(HWND parent, Win32.POINT point, int flags);

		[DllImport("user32.dll")]
		public static extern bool ClientToScreen(HWND hwnd, ref Win32.POINT pt);

		[DllImport("user32.dll")]
		public static extern IntPtr CreateIconIndirect(ref Win32.ICONINFO icon);

		[DllImport("user32.dll")]
		public static extern IntPtr CreatePopupMenu();

		/// <summary>Create a new window instance</summary>
		public static HWND CreateWindow(int dwExStyle, string lpClassName, string lpWindowName, int dwStyle, int x, int y, int nWidth, int nHeight, IntPtr hWndParent, IntPtr hMenu, IntPtr hInstance, IntPtr lpParam)
		{
			return CreateWindow_(dwExStyle, lpClassName, lpWindowName, dwStyle, x, y, nWidth, nHeight, hWndParent, hMenu, hInstance, lpParam);
		}
		public static HWND CreateWindow(int dwExStyle, int lpClassAtom, string lpWindowName, int dwStyle, int x, int y, int nWidth, int nHeight, IntPtr hWndParent, IntPtr hMenu, IntPtr hInstance, IntPtr lpParam)
		{
			return CreateWindow_(dwExStyle, (IntPtr)lpClassAtom, lpWindowName, dwStyle, x, y, nWidth, nHeight, hWndParent, hMenu, hInstance, lpParam);
		}
		[DllImport("user32.dll", EntryPoint = "CreateWindowExW", CharSet = CharSet.Unicode, SetLastError = true)]
		private static extern HWND CreateWindow_(int dwExStyle, [MarshalAs(UnmanagedType.LPWStr)] string lpClassName, [MarshalAs(UnmanagedType.LPWStr)] string lpWindowName, int dwStyle, int x, int y, int nWidth, int nHeight, IntPtr hWndParent, IntPtr hMenu, IntPtr hInstance, IntPtr lpParam);
		[DllImport("user32.dll", EntryPoint = "CreateWindowExW", CharSet = CharSet.Unicode, SetLastError = true)]
		private static extern HWND CreateWindow_(int dwExStyle, IntPtr lpClassAtom, [MarshalAs(UnmanagedType.LPWStr)] string lpWindowName, int dwStyle, int x, int y, int nWidth, int nHeight, IntPtr hWndParent, IntPtr hMenu, IntPtr hInstance, IntPtr lpParam);

		[DllImport("user32.dll", SetLastError = true, CharSet = CharSet.Unicode)]
		public static extern IntPtr CallWindowProc(IntPtr lpPrevWndFunc, HWND hWnd, int Msg, WPARAM wParam, LPARAM lParam);

		[DllImport("user32.dll", SetLastError = true, CharSet = CharSet.Unicode)]
		public static extern IntPtr DefWindowProc(HWND hWnd, int Msg, WPARAM wParam, LPARAM lParam);

		[DllImport("user32.dll", CharSet = CharSet.Auto)]
		public static extern bool DestroyIcon(HICON hicon);

		/// <summary></summary>
		public static bool DestroyWindow(HWND hwnd) => DestroyWindow_(hwnd);
		[DllImport("user32.dll", EntryPoint = "DestroyWindow", CharSet = CharSet.Unicode)]
		private static extern bool DestroyWindow_(HWND hwnd);

		/// <summary></summary>
		public static int DispatchMessage(ref Win32.MESSAGE msg) => DispatchMessage_(ref msg);
		[DllImport("user32.dll", EntryPoint = "DispatchMessageW", CharSet = CharSet.Unicode)]
		private static extern int DispatchMessage_(ref Win32.MESSAGE lpMsg);

		/// <summary>Draw an icon into an HDC</summary>
		public static void DrawIcon(HDC hDC, int X, int Y, HICON hIcon)
		{
			if (!DrawIcon_(hDC, X, Y, hIcon))
				throw new Win32Exception("DrawIcon failed");
		}
		[DllImport("user32.dll", EntryPoint = "DrawIcon", CharSet = CharSet.Unicode, SetLastError = true)]
		private static extern bool DrawIcon_(HDC hDC, int X, int Y, HICON hIcon);

		[DllImport("user32.dll")]
		public static extern int EnumWindows(Win32.EnumWindowsProc ewp, int lParam);

		/// <summary>Flash an application window</summary>
		public static bool FlashWindow(HWND hwnd, Win32.EFlashWindowFlags flags, uint count = uint.MaxValue, uint flash_rate = 0)
		{
			var info = new Win32.FLASHWINFO
			{
				cbSize = (uint)Marshal.SizeOf<Win32.FLASHWINFO>(),
				hwnd = hwnd,
				dwFlags = (uint)flags,
				uCount = count,
				dwTimeout = flash_rate,
			};
			return FlashWindowEx_(ref info);
		}
		[DllImport("user32.dll", EntryPoint = "FlashWindowEx")]
		private static extern bool FlashWindowEx_(ref Win32.FLASHWINFO pwfi);

		[DllImport("user32.dll")]
		public static extern HWND GetAncestor(HWND hwnd, uint flags);

		[DllImport("user32.dll")]
		public static extern short GetAsyncKeyState(EKeyCodes vKey);

		[DllImport("user32.dll", SetLastError = true)]
		public static extern bool GetCaretPos(ref Win32.POINT point);

		/// <summary>Get the window class description for a window class name</summary>
		/// <param name="hinstance">A handle to the instance of the application that created the class. To retrieve information about classes
		/// defined by the system (such as buttons or list boxes),set this parameter to NULL.</param>
		/// <param name="class_name">The name of the window class to look up</param>
		/// <returns>Returns the window class data if the window class is registered</returns>
		public static Win32.WNDCLASSEX? GetClassInfo(IntPtr hinstance, string class_name, out int atom)
		{
			var wc = new Win32.WNDCLASSEX { cbSize = Marshal.SizeOf<Win32.WNDCLASSEX>() };
			atom = GetClassInfoEx_(hinstance, class_name, ref wc);
			return atom != 0 ? wc : (Win32.WNDCLASSEX?)null;
		}
		[DllImport("user32.dll", EntryPoint = "GetClassInfoExW", CharSet = CharSet.Unicode, SetLastError = true)]
		private static extern int GetClassInfoEx_(IntPtr hinstance, [MarshalAs(UnmanagedType.LPWStr)] string class_name, ref Win32.WNDCLASSEX lpwcx);

		[DllImport("user32.dll", EntryPoint = "GetClassLongPtrW", SetLastError = true)]
		public static extern IntPtr GetClassLongPtr(HWND hwnd, int index);

		/// <summary></summary>
		public static Win32.RECT GetClientRect(HWND hwnd)
		{
			return GetClientRect_(hwnd, out var rect) ? rect : throw new Win32Exception("GetClientRect failed"); ;
		}
		[DllImport("user32.dll", EntryPoint = "GetClientRect")]
		private static extern bool GetClientRect_(HWND hwnd, out Win32.RECT rect);

		/// <summary>Return the mouse position in screen coordinates</summary>
		public static Win32.POINT GetCursorPos()
		{
			return GetCursorPos_(out var pt) ? pt : throw new Win32Exception("GetCursorPos failed");
		}
		[DllImport("user32.dll", EntryPoint = "GetCursorPos")]
		private static extern bool GetCursorPos_(out Win32.POINT lpPoint);

		/// <summary>Return info about the current mouse cursor</summary>
		public static Win32.CURSORINFO GetCursorInfo()
		{
			var info = Win32.CURSORINFO.Default;
			GetCursorInfo_(ref info);
			return info;
		}
		[DllImport("user32.dll", EntryPoint = "GetCursorInfo", CharSet = CharSet.Unicode, SetLastError = true)]
		private static extern bool GetCursorInfo_(ref Win32.CURSORINFO pci);

		[DllImport("user32.dll")]
		public static extern IntPtr GetDC(HWND hwnd);

		/// <summary></summary>
		public static IntPtr GetDesktopWindow() => GetDesktopWindow_();
		[DllImport("user32", EntryPoint = "GetDesktopWindow")]
		private static extern IntPtr GetDesktopWindow_();

		[DllImport("user32.dll")]
		public static extern int GetDoubleClickTime();

		[DllImport("user32.dll")]
		public static extern HWND GetFocus();

		[DllImport("user32.dll")]
		public static extern HWND GetForegroundWindow();

		[DllImport("user32.dll")]
		public static extern bool GetIconInfo(IntPtr hIcon, ref Win32.ICONINFO pIconInfo);

		[DllImport("user32.dll")]
		public static extern int GetKeyboardState(byte[] pbKeyState);

		[DllImport("user32.dll")]
		public static extern short GetKeyState(EKeyCodes vKey);

		/// <summary></summary>
		public static int GetMessage(out Win32.MESSAGE msg, IntPtr hWnd, uint messageFilterMin, uint messageFilterMax) => GetMessage_(out msg, hWnd, messageFilterMin, messageFilterMax);
		[DllImport("user32.dll", EntryPoint = "GetMessageW", CharSet = CharSet.Unicode)]
		private static extern int GetMessage_(out Win32.MESSAGE msg, IntPtr hWnd, uint messageFilterMin, uint messageFilterMax);

		/// <summary></summary>
		public static int GetMessageTime() => GetMessageTime_();
		[DllImport("user32.dll", EntryPoint = "GetMessageTime")]
		public static extern int GetMessageTime_();

		/// <summary>Return info about the given monitor handle</summary>
		public static Win32.MONITORINFOEX GetMonitorInfo(IntPtr hMonitor)
		{
			var info = new Win32.MONITORINFOEX { cbSize = (uint)Marshal.SizeOf<Win32.MONITORINFOEX>() };
			return GetMonitorInfoW_(hMonitor, ref info) ? info : throw new Win32Exception("Monitor info not available");
		}
		[DllImport("user32.dll", EntryPoint = "GetMonitorInfoW")]
		private static extern bool GetMonitorInfoW_(IntPtr hMonitor, ref Win32.MONITORINFOEX info);

		/// <summary></summary>
		[DllImport("user32.dll", ExactSpelling = true)]
		public static extern IntPtr GetNextDlgTabItem(IntPtr hDlg, IntPtr hCtl, [MarshalAs(UnmanagedType.Bool)] bool bPrevious);

		[DllImport("user32.dll")]
		public static extern HWND GetParent(HWND hwnd);

		[DllImport("user32.dll")]
		public static extern uint GetRawInputBuffer(IntPtr pRawInput, ref uint pcbSize, uint cbSizeHeader);
		
		[DllImport("user32.dll")]
		public static extern uint GetRawInputData(IntPtr hRawInput, uint uiCommand, IntPtr pData, ref uint pcbSize, uint cbSizeHeader);

		[DllImport("user32.dll")]
		public static extern bool GetScrollInfo(HWND hwnd, int BarType, ref Win32.SCROLLINFO lpsi);

		[DllImport("user32.dll")]
		public static extern int GetScrollPos(HWND hWnd, int nBar);

		/// <summary></summary>
		public static IntPtr GetShellWindow() => GetShellWindow_();
		[DllImport("user32", EntryPoint = "GetShellWindow")]
		private static extern IntPtr GetShellWindow_();

		[DllImport("user32.dll")]
		public static extern IntPtr GetSystemMenu(HWND hwnd, bool bRevert);

		/// <summary>System metrics</summary>
		public static int GetSystemMetrics(Win32.ESystemMetrics metric)
		{
			return GetSystemMetrics_((int)metric);
		}
		[DllImport("user32.dll", EntryPoint = "GetSystemMetrics")]
		private static extern int GetSystemMetrics_(int nIndex);

		[DllImport("user32.dll")]
		public static extern bool GetUpdateRect(HWND hwnd, out Win32.RECT rect, bool erase);

		[DllImport("user32.dll", SetLastError = true)]
		public static extern uint GetWindowLong(HWND hWnd, int nIndex);

		[DllImport("user32.dll", EntryPoint = "GetWindowLongPtrW", CharSet = CharSet.Unicode, SetLastError = true)]
		public static extern long GetWindowLongPtr(HWND hWnd, int nIndex); // This is only defined in 64bit builds, otherwise it's a #define to GetWindowLong

		/// <summary>Retrieves a handle to a window that has the specified relationship to the specified window.</summary>
		[DllImport("user32.dll", SetLastError = true)]
		public static extern HWND GetWindow(HWND hWnd, uint uCmd);

		[DllImport("user32.dll", CharSet = CharSet.Unicode)]
		public static extern int GetWindowModuleFileName(HWND hwnd, StringBuilder title, int size);

		/// <summary>Return the area of a window in screen space</summary>
		public static Win32.RECT GetWindowRect(HWND hwnd)
		{
			return GetWindowRect_(hwnd, out var rect) ? rect : throw new Win32Exception("GetWindowRect failed");
		}
		[DllImport("user32.dll", EntryPoint = "GetWindowRect")]
		private static extern bool GetWindowRect_(HWND hwnd, out Win32.RECT rect);

		[DllImport("user32.dll", CharSet = CharSet.Unicode)]
		public static extern int GetWindowText(HWND hwnd, StringBuilder title, int size);

		[DllImport("user32.dll", SetLastError = true)]
		public static extern IntPtr GetWindowThreadProcessId(HWND hWnd, ref IntPtr lpdwProcessId);

		[DllImport("user32.dll")]
		public static extern int HideCaret(HWND hwnd);

		[DllImport("user32.dll", EntryPoint = "InsertMenu", CharSet = CharSet.Unicode)]
		public static extern bool InsertMenu(IntPtr hMenu, int wPosition, int wFlags, int wIDNewItem, string lpNewItem);

		[DllImport("user32.dll", EntryPoint = "InsertMenu", CharSet = CharSet.Unicode)]
		public static extern bool InsertMenu(IntPtr hMenu, int wPosition, int wFlags, IntPtr wIDNewItem, string lpNewItem);

		[DllImport("user32.dll")]
		public static extern bool InvalidateRect(HWND hwnd, IntPtr lpRect, bool bErase);

		[DllImport("user32.dll")]
		public static extern bool InvalidateRect(HWND hwnd, ref Win32.RECT lpRect, bool bErase);

		[DllImport("user32.dll")]
		public static extern bool IsChild(HWND parent, HWND hwnd);

		[DllImport("user32.dll")]
		public static extern bool IsIconic(HWND hwnd);

		[DllImport("user32.dll")]
		public static extern bool IsWindow(HWND hwnd);

		[DllImport("user32.dll")]
		public static extern bool IsWindowVisible(HWND hwnd);

		[DllImport("user32.dll")]
		public static extern bool IsZoomed(HWND hwnd);

		[DllImport("user32.dll")]
		public static extern bool LockWindowUpdate(HWND hWndLock);

		[DllImport("user32.dll")]
		public static extern IntPtr LoadCursorFromFile([MarshalAs(UnmanagedType.LPWStr)] string lpFileName);

		[DllImport("user32.dll")]
		public static extern uint MapVirtualKey(uint uCode, uint uMapType);

		/// <summary>Return the monitor that contains the given point</summary>
		public static IntPtr MonitorFromPoint(int x, int y, Win32.EMonitorFromFlags flags = Win32.EMonitorFromFlags.DEFAULT_TO_NEAREST) => MonitorFromPoint(new Win32.POINT { X = x, Y = y }, flags);
		public static IntPtr MonitorFromPoint(Win32.POINT pt, Win32.EMonitorFromFlags flags = Win32.EMonitorFromFlags.DEFAULT_TO_NEAREST) => MonitorFromPoint_(pt, (int)flags);
		[DllImport("user32", EntryPoint = "MonitorFromPoint")]
		private static extern IntPtr MonitorFromPoint_(Win32.POINT pt, int flags);

		/// <summary>Return the monitor that contains the given rectangle</summary>
		public static IntPtr MonitorFromRect(Win32.RECT rc, Win32.EMonitorFromFlags flags = Win32.EMonitorFromFlags.DEFAULT_TO_NEAREST) => MonitorFromRect_(rc, (int)flags);
		[DllImport("user32", EntryPoint = "MonitorFromRect")]
		private static extern IntPtr MonitorFromRect_(Win32.RECT rc, int flags);

		/// <summary>Return the monitor that contains the given window handle</summary>
		public static IntPtr MonitorFromWindow(IntPtr hwnd, Win32.EMonitorFromFlags flags = Win32.EMonitorFromFlags.DEFAULT_TO_NEAREST) => MonitorFromWindow_(hwnd, (int)flags);
		[DllImport("user32", EntryPoint = "MonitorFromWindow")]
		private static extern IntPtr MonitorFromWindow_(IntPtr hwnd, int flags);

		[DllImport("user32.dll")]
		public static extern bool MoveWindow(HWND hWnd, int X, int Y, int nWidth, int nHeight, bool repaint);

		/// <summary></summary>
		public static int MsgWaitForMultipleObjects(int nCount, [MarshalAs(UnmanagedType.LPArray)] IntPtr []? pHandles, bool fWaitAll, int dwMilliseconds, int dwWakeMask) => MsgWaitForMultipleObjects_(nCount, pHandles, fWaitAll, dwMilliseconds, dwWakeMask);
		[DllImport("user32", EntryPoint = "MsgWaitForMultipleObjects")]
		private static extern int MsgWaitForMultipleObjects_(int nCount, [MarshalAs(UnmanagedType.LPArray)] IntPtr []? pHandles, bool fWaitAll, int dwMilliseconds, int dwWakeMask);

		/// <summary></summary>
		public static bool PeekMessage(out Win32.MESSAGE msg, IntPtr hWnd, uint messageFilterMin, uint messageFilterMax, Win32.EPeekMessageFlags flags)
		{
			return PeekMessage_(out msg, hWnd, messageFilterMin, messageFilterMax, (int)flags);
		}
		[DllImport("user32.dll", EntryPoint = "PeekMessageW", CharSet = CharSet.Unicode)]
		private static extern bool PeekMessage_(out Win32.MESSAGE msg, IntPtr hWnd, uint messageFilterMin, uint messageFilterMax, int flags);

		/// <summary></summary>
		public static bool PostMessage(IntPtr hwnd, uint message, IntPtr wparam, IntPtr lparam) => PostMessage_(hwnd, message, wparam, lparam);
		[DllImport("user32.dll", EntryPoint = "PostMessageW", CharSet = CharSet.Unicode)]
		private static extern bool PostMessage_(IntPtr hwnd, uint message, IntPtr wparam, IntPtr lparam);

		/// <summary></summary>
		public static int PostQuitMessage(int nExitCode) => PostQuitMessage_(nExitCode);
		[DllImport("user32.dll", EntryPoint = "PostQuitMessage")]
		private static extern int PostQuitMessage_(int nExitCode);

		[DllImport("user32.dll", EntryPoint = "PostThreadMessage")]
		public static extern int PostThreadMessage(int idThread, uint msg, int wParam, int lParam);

		[DllImport("user32.dll")]
		public static extern bool RedrawWindow(HWND hWnd, ref Win32.RECT lprcUpdate, HRGN hrgnUpdate, uint flags);
		
		[DllImport("user32.dll")]
		public static extern bool RedrawWindow(HWND hWnd, IntPtr lprcUpdate, HRGN hrgnUpdate, uint flags);

		/// <summary>Register a window class</summary>
		public static ushort RegisterClass(Win32.WNDCLASS wnd_class)
		{
			// RegisterClass only sets last error if there is an error
			Kernel32.SetLastError(Win32.ERROR_SUCCESS);
			var atom = RegisterClass_(ref wnd_class);
			var err = Marshal.GetLastWin32Error();
			if (err != Win32.ERROR_SUCCESS) throw new Win32Exception();
			return atom;
		}
		public static ushort RegisterClass(Win32.WNDCLASSEX wnd_class)
		{
			// RegisterClass only sets last error if there is an error
			Kernel32.SetLastError(Win32.ERROR_SUCCESS);
			var atom = RegisterClassEx_(ref wnd_class);
			var err = Marshal.GetLastWin32Error();
			if (err != Win32.ERROR_SUCCESS) throw new Win32Exception(); 
			return atom;
		}
		[DllImport("user32.dll", EntryPoint = "RegisterClassW", CharSet = CharSet.Unicode, SetLastError = true)]
		private static extern ushort RegisterClass_(ref Win32.WNDCLASS lpWndClass);
		[DllImport("user32.dll", EntryPoint = "RegisterClassExW", CharSet = CharSet.Unicode, SetLastError = true)]
		private static extern ushort RegisterClassEx_(ref Win32.WNDCLASSEX lpWndClass);

		/// <summary>Registry for notifications about devices being adding/removed</summary>
		public static SafeDevNotifyHandle RegisterDeviceNotification(IntPtr recipient, Win32.DEV_BROADCAST_HDR notificationFilter, Win32.EDeviceNotifyFlags flags)
		{
			// This function expects a contiguous struct with a 'DEV_BROADCAST_HDR'
			// as the header. Create overloads for any new struct types.
			var handle = RegisterDeviceNotification_(recipient, ref notificationFilter, (int)flags);
			return new SafeDevNotifyHandle(handle, true);
		}
		public static SafeDevNotifyHandle RegisterDeviceNotification(IntPtr recipient, Win32.DEV_BROADCAST_DEVICEINTERFACE notificationFilter, Win32.EDeviceNotifyFlags flags)
		{
			var handle = RegisterDeviceNotification_(recipient, ref notificationFilter, (int)flags);
			return new SafeDevNotifyHandle(handle, true);
		}
		[DllImport("user32.dll", EntryPoint = "RegisterDeviceNotificationW", CharSet = CharSet.Unicode, SetLastError = true)]
		private static extern IntPtr RegisterDeviceNotification_(IntPtr recipient, ref Win32.DEV_BROADCAST_HDR filter, int flags);
		[DllImport("user32.dll", EntryPoint = "RegisterDeviceNotificationW", CharSet = CharSet.Unicode, SetLastError = true)]
		private static extern IntPtr RegisterDeviceNotification_(IntPtr recipient, ref Win32.DEV_BROADCAST_DEVICEINTERFACE filter, int flags);
		
		[DllImport("user32.dll", SetLastError = true)]
		public static extern bool RegisterRawInputDevices(Win32.RAWINPUTDEVICE[] pRawInputDevices, uint uiNumDevices, uint cbSize);

		[DllImport("user32.dll", EntryPoint = "RegisterWindowMessageW")]
		public static extern uint RegisterWindowMessage([MarshalAs(UnmanagedType.LPWStr)] string lpString);

		[DllImport("user32.dll")]
		public static extern bool ReleaseDC(HWND hWnd, IntPtr hDC);

		[DllImport("user32.dll")]
		public static extern bool ScreenToClient(HWND hwnd, ref Win32.POINT pt);

		[DllImport("user32.dll", EntryPoint = "SendMessage", SetLastError = true)]
		public static extern IntPtr SendMessage(HWND hwnd, uint msg, int wparam, int lparam); // Don't return int, it truncates on 64bit

		[DllImport("user32.dll", EntryPoint = "SendMessage", SetLastError = true)]
		public static extern IntPtr SendMessage(HWND hwnd, int msg, IntPtr wparam, IntPtr lparam);

		[DllImport("user32.dll", EntryPoint = "SendMessage", SetLastError = true)]
		public static extern IntPtr SendMessage(HWND hwnd, uint msg, IntPtr wparam, IntPtr lparam);

		[DllImport("user32.dll", EntryPoint = "SendMessage", SetLastError = true)]
		public static extern IntPtr SendMessage(HWND hwnd, uint msg, IntPtr wparam, ref Win32.POINT lparam);

		[DllImport("user32.dll", EntryPoint = "SendMessage", SetLastError = true)]
		public static extern IntPtr SendMessage(HWND hwnd, uint msg, IntPtr wparam, ref Win32.RECT lparam);

		[DllImport("user32.dll", EntryPoint = "SendMessage", SetLastError = true)]
		public static extern IntPtr SendMessage(HWND hwnd, uint msg, IntPtr wparam, out Win32.COMBOBOXINFO lparam);

		[DllImport("user32.dll")]
		public static extern HWND SetActiveWindow(HWND hwnd);

		[DllImport("user32.dll")]
		public static extern IntPtr SetCursor(IntPtr cursor);

		[DllImport("user32.dll")]
		public static extern HWND SetFocus(HWND hwnd);

		[DllImport("user32.dll")]
		public static extern bool SetForegroundWindow(HWND hwnd);

		[DllImport("user32.dll")]
		public static extern IntPtr SetParent(HWND hWndChild, HWND hWndNewParent);

		[DllImport("user32.dll")]
		public static extern bool SetProcessDPIAware();

		[DllImport("user32.dll")]
		public static extern int SetScrollInfo(HWND hwnd, int fnBar, ref Win32.SCROLLINFO lpsi, bool fRedraw);

		[DllImport("user32.dll")]
		public static extern int SetScrollPos(HWND hWnd, int nBar, int nPos, bool bRedraw);

		/// <summary>Installs an event hook to receive notifications for a range of events.</summary>
		public delegate void WinEventDelegate(IntPtr hWinEventHook, uint eventType, IntPtr hwnd, int idObject, int idChild, uint dwEventThread, uint dwmsEventTime);
		[DllImport("user32.dll")]
		public static extern IntPtr SetWinEventHook(uint eventMin, uint eventMax, IntPtr hmodWinEventProc, WinEventDelegate lpfnWinEventProc, uint idProcess, uint idThread, uint dwFlags);

		/// <summary>Removes an event hook installed by SetWinEventHook.</summary>
		[DllImport("user32.dll")]
		public static extern bool UnhookWinEvent(IntPtr hWinEventHook);

		[DllImport("user32.dll")]
		public static extern int SetWindowsHookEx(int idHook, Win32.HookProc lpfn, IntPtr hMod, int dwThreadId);

		[DllImport("user32.dll")]
		public static extern int SetWindowLong(HWND hWnd, int nIndex, uint dwNewLong);

		[DllImport("user32.dll")]
		public static extern bool SetWindowPos(HWND hWnd, HWND hWndInsertAfter, int X, int Y, int cx, int cy, uint uFlags);

		[DllImport("user32.dll")]
		public static extern int ShowCaret(IntPtr hwnd);

		[DllImport("user32.dll")]
		public static extern bool ShowWindow(IntPtr hWnd, int nCmdShow);

		[DllImport("user32.dll")]
		public static extern bool ShowWindowAsync(HWND hwnd, int nCmdShow);

		[DllImport("user32.dll")]
		public static extern int ToAscii(int uVirtKey, int uScanCode, byte[] lpbKeyState, byte[] lpwTransKey, int fuState);

		[DllImport("user32.dll")]
		public static extern int ToUnicode(uint wVirtKey, uint wScanCode, byte[] lpKeyState, [Out, MarshalAs(UnmanagedType.LPWStr, SizeParamIndex = 4)] StringBuilder pwszBuff, int cchBuff, uint wFlags);

		[DllImport("user32.dll")]
		public static extern bool TranslateMessage(ref Win32.MESSAGE lpMsg);

		[DllImport("user32.dll")]
		public static extern bool UpdateWindow(HWND hWnd);

		[DllImport("user32.dll")]
		public static extern int UnhookWindowsHookEx(int idHook);

		/// <summary>Release a device notification handle</summary>
		public static bool UnregisterDeviceNotification(IntPtr handle) => UnregisterDeviceNotification_(handle);
		[DllImport("user32.dll", EntryPoint = "UnregisterDeviceNotification", SetLastError = true)]
		private static extern bool UnregisterDeviceNotification_(IntPtr handle);

		[DllImport("user32.dll")]
		public static extern HWND WindowFromPoint(Win32.POINT Point);

		[DllImport("user32.dll")]
		public static extern bool ValidateRect(HWND hwnd, IntPtr lpRect);

		[DllImport("user32.dll")]
		public static extern bool ValidateRect(HWND hwnd, ref Win32.RECT lpRect);

		// This static method is required because legacy OSes do not support SetWindowLongPtr 
		public static IntPtr SetWindowLongPtr(HWND hWnd, int nIndex, IntPtr dwNewLong)
		{
			return IntPtr.Size == 8
				? SetWindowLongPtr64(hWnd, nIndex, dwNewLong)
				: new IntPtr(SetWindowLong32(hWnd, nIndex, dwNewLong.ToInt32()));
		}
		[DllImport("user32.dll", EntryPoint = "SetWindowLong")]
		private static extern int SetWindowLong32(HWND hWnd, int nIndex, int dwNewLong);
		[DllImport("user32.dll", EntryPoint = "SetWindowLongPtr")]
		private static extern IntPtr SetWindowLongPtr64(HWND hWnd, int nIndex, IntPtr dwNewLong);

		[DllImport("uxtheme.dll")]
		public static extern int SetWindowTheme(IntPtr hWnd, [MarshalAs(UnmanagedType.LPWStr)] string appname, [MarshalAs(UnmanagedType.LPWStr)] string idlist);

		
		/// <summary>Return the DC for a window</summary>
		public static Scope WindowDC(HWND hwnd)
		{
			return Scope.Create(() => GetDC(hwnd), dc => ReleaseDC(hwnd, dc));
		}
	}

	public sealed class SafeDevNotifyHandle :SafeHandleZeroOrMinusOneIsInvalid
	{
		public SafeDevNotifyHandle(IntPtr handle, bool owns_handle) : base(owns_handle) => SetHandle(handle);
		protected override bool ReleaseHandle() => User32.UnregisterDeviceNotification(DangerousGetHandle());
	}
}
