//*****************************************************************************************
// Win32 API 
//  Copyright (c) Rylogic Ltd 2014
//*****************************************************************************************
// A collection of structs that wrap the win32 API and expose an
// interface similar to C# .NET win forms. Inspired by ATL\WTL.
// Specs:
//   - As fast as ATL\WTL. Uses ATL thunks for WNDPROC
//   - Doesn't use macros. Much easier to debug/read
//   - Single file with minimal dependencies. Depends on standard
//     C++ headers, atlbase.h, and other standard headers (no external
//     libraries needed)
//   - Automatic support for resizing
//   - C#.NET style event handlers
/*
Example Use:

	struct MyWindow :Form
	{
		enum { IDC_SPLIT = 100, IDC_LEFT, IDC_RITE,};
		
		Panel    m_panel;
		Splitter m_split;

		MyWindow()
			:Form(Params<>()
				.name("test")
				.title(L"My Window")
				.xy(2000,100)
				.wh(800,600)
				.menu({{L"&File", Menu(Menu::EKind::Popup, {MenuItem(L"E&xit", IDCLOSE)})}})
				.main_wnd(true)
				.wndclass(RegisterWndClass<MyWindow>()))
			,m_panel(Panel   ::Params<>().parent(this_   ).xy(50,50).wh(Fill,Fill).anchor(EAnchor::All))
			,m_split(Splitter::Params<>().parent(&m_panel).dock(EDock::Fill))
		{
			m_split.Pane0.Style('+', WS_BORDER);
			m_split.Pane1.Style('+', WS_BORDER);
		}
	};

	// Entry point
	int __stdcall WinMain(HINSTANCE hInstance, HINSTANCE, LPTSTR, int)
	{
		InitCtrls();
		try
		{
			(void)hInstance;
			MyWindow main;
			main.Show();

			MessageLoop loop;
			loop.AddMessageFilter(main);
			return loop.Run();
		}
		catch (std::exception const& ex)
		{
			OutputDebugStringA("Died: ");
			OutputDebugStringA(ex.what());
			OutputDebugStringA("\n");
			return -1;
		}
	}
*/
#pragma once

#ifdef NOGDI
#error The GDI is required for wingui.h
#endif

#include <vector>
#include <string>
#include <string_view>
#include <span>
#include <fstream>
#include <iterator>
#include <unordered_map>
#include <algorithm>
#include <new>
#include <memory>
#include <functional>
#include <atomic>
#include <mutex>
#include <thread>
#include <format>
#include <type_traits>
#include <cassert>

#include <winsdkver.h>
#include <sdkddkver.h>
#include <winuser.h>
#include <commctrl.h>
#include <commdlg.h>
#include <richedit.h>
#include <shellapi.h>
#include <shobjidl.h>
#include <shlguid.h>
#include <shlwapi.h>
#include <uxtheme.h>
#include <gdiplus.h>
#include <tchar.h>

inline static constexpr bool PR_WNDPROCDEBUG = false;
#if PR_WNDPROCDEBUG
#pragma message("WARNING: **************************** WndProcDebug Enabled.")
#include "pr/gui/messagemap_dbg.h"
#endif

#pragma comment(lib, "comctl32.lib")
#pragma comment(lib, "uxtheme.lib")
#pragma comment(lib, "gdiplus.lib")
#pragma comment(lib, "shlwapi.lib")

// Disable warnings
#pragma warning(push)
#pragma warning(disable: 4351) // C4351: new behaviour: elements of array will be default initialized

// Implementation
namespace pr
{
	// Import the 'Gdiplus' namespace into 'pr::gdi'
	namespace gdi = Gdiplus;
	namespace gui
	{
		static_assert(_WIN32_WINNT >= _WIN32_WINNT_WIN6, "Windows version not >= win6");

		// Forwards
		struct DlgTemplate;
		struct Control;
		struct Form;

		#pragma region Constants
		// Control Id range
		inline constexpr int ID_BEG = 0x00000001;
		inline constexpr int ID_END = 0x00010000;

		// Special id for controls that don't need an id.
		// Id's should be in the range [0,0xffff] because they are sometimes
		// represented by a 'ushort'. Auto size/positioning also packs a int32
		// with flags and the control id.
		inline constexpr int ID_UNUSED = 0x0000FFFF;

		// Addition message ids
		enum
		{
			// A user windows message that returns the Control* associated with a given hwnd
			WM_GETCTRLPTR = WM_USER,

			// The first windows message not reserved by pr::gui
			WM_USER_BASE,
		};
		#pragma endregion

		#pragma region Enumerations

		// The common control classes
		enum class ECommonControl
		{
			None = 0,
			ListViewClasses = ICC_LISTVIEW_CLASSES, // listview, header
			TreeViewClasses = ICC_TREEVIEW_CLASSES, // treeview, tooltips
			BarClasses = ICC_BAR_CLASSES, // toolbar, statusbar, trackbar, tooltips
			TabClasses = ICC_TAB_CLASSES, // tab, tooltips
			UpDown = ICC_UPDOWN_CLASS, // updown
			Progress = ICC_PROGRESS_CLASS, // progress
			Hotkey = ICC_HOTKEY_CLASS, // hotkey
			Animate = ICC_ANIMATE_CLASS, // animate
			Win95Classes = ICC_WIN95_CLASSES, //
			DateClasses = ICC_DATE_CLASSES, // month picker, date picker, time picker, updown
			ComboEx = ICC_USEREX_CLASSES, // comboex
			Rebar = ICC_COOL_CLASSES, // rebar (coolbar) control
			Internet = ICC_INTERNET_CLASSES, //
			PageScroller = ICC_PAGESCROLLER_CLASS, // page scroller
			NativeFontCtrl = ICC_NATIVEFNTCTL_CLASS, // native font control
			StandardClasses = ICC_STANDARD_CLASSES,
			LinkClass = ICC_LINK_CLASS,
			All = ~None,
			allow_bitops = 0,
		};

		//todo: unused at the mo
		enum class EUnits
		{
			// X,Y,W,H in pixels
			Pixels,

			// Units are relative to the average size of the window font
			DialogUnits,
		};

		// Auto size anchors
		enum class EAnchor
		{
			None = 0,
			Left = 1 << 0,
			Top = 1 << 1,
			Right = 1 << 2,
			Bottom = 1 << 3,
			TopLeft = Left | Top,
			TopRight = Right | Top,
			BottomLeft = Left | Bottom,
			BottomRight = Right | Bottom,
			LeftTopRight = Left | Top | Right,
			LeftBottomRight = Left | Bottom | Right,
			LeftTopBottom = Left | Top | Bottom,
			RightTopBottom = Right | Top | Bottom,
			All = Left | Top | Right | Bottom,
			allow_bitops = 0,
		};

		// Window docking
		enum class EDock
		{
			None = 0,
			Fill = 1,
			Top = 2,
			Bottom = 3,
			Left = 4,
			Right = 5
		};

		// Dialog result
		enum class EDialogResult
		{
			None = 0,
			Ok = IDOK,
			Cancel = IDCANCEL,
			Abort = IDABORT,
			Retry = IDRETRY,
			Ignore = IDIGNORE,
			Yes = IDYES,
			No = IDNO,
			Close = IDCLOSE,
			Help = IDHELP,
			TryAgain = IDTRYAGAIN,
			Continue = IDCONTINUE,
			Timeout = IDTIMEOUT,
		};

		// Window start position
		enum class EStartPosition
		{
			Default,
			CentreParent,
			Manual,
		};

		// Set window position flags
		enum class EWindowPos :UINT
		{
			None = 0,
			NoSize = SWP_NOSIZE,
			NoMove = SWP_NOMOVE,
			NoZorder = SWP_NOZORDER,
			NoRedraw = SWP_NOREDRAW,
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
		};

		// Control key state
		enum class EModifierKey
		{
			None = 0,
			LShift = 1 << 0,
			RShift = 1 << 1,
			Shift = LShift | RShift,
			LCtrl = 1 << 2,
			RCtrl = 1 << 3,
			Ctrl = LCtrl | RCtrl,
			LAlt = 1 << 4,
			RAlt = 1 << 5,
			Alt = LAlt | RAlt,
			allow_bitops = 0,
		};

		// Mouse key state, used in mouse down/up events
		enum class EMouseKey
		{
			None = 0,
			Left = MK_LBUTTON, // 0x0001
			Right = MK_RBUTTON, // 0x0002
			Shift = MK_SHIFT,   // 0x0004
			Ctrl = MK_CONTROL, // 0x0008
			Middle = MK_MBUTTON, // 0x0010
			XButton1 = MK_XBUTTON1,// 0x0020
			XButton2 = MK_XBUTTON2,// 0x0040
			Alt = 0x0080,     // There is no MK_ defined for alt, this is tested using GetKeyState
			allow_bitops = 0,
		};

		enum :DWORD { DefaultControlStyle = WS_CHILD | WS_VISIBLE | WS_CLIPCHILDREN | WS_CLIPSIBLINGS };
		enum :DWORD { DefaultControlStyleEx = 0 };

		// Don't add WS_VISIBLE to the default style. Derived forms should choose when to be visible at the end of their constructors
		// WS_OVERLAPPEDWINDOW = (WS_OVERLAPPED|WS_CAPTION|WS_SYSMENU|WS_THICKFRAME|WS_MINIMIZEBOX|WS_MAXIMIZEBOX)
		// WS_POPUPWINDOW = (WS_POPUP|WS_BORDER|WS_SYSMENU)
		// WS_EX_COMPOSITED adds automatic double buffering, which doesn't work for directX apps
		enum :DWORD { DefaultFormStyle = DS_SETFONT | DS_FIXEDSYS | WS_OVERLAPPEDWINDOW | WS_CLIPCHILDREN };
		enum :DWORD { DefaultFormStyleEx = WS_EX_APPWINDOW | WS_EX_WINDOWEDGE };

		enum :DWORD { DefaultDialogStyle = (DefaultFormStyle | DS_MODALFRAME | WS_POPUPWINDOW) & ~(WS_OVERLAPPED) };
		enum :DWORD { DefaultDialogStyleEx = (DefaultFormStyleEx) & ~(WS_EX_APPWINDOW) };
		#pragma endregion

		#pragma region Unicode conversion
		// A static instance of the locale, because this thing takes ages to construct
		inline std::locale const& locale()
		{
			static std::locale s_locale("");
			return s_locale;
		}

		// Narrow
		inline std::string Narrow(char const* from, std::size_t len = 0)
		{
			if (!from) return std::string();
			if (len == 0) len = strlen(from);
			return std::string(from, from + len);
		}
		inline std::string Narrow(wchar_t const* from, std::size_t len = 0)
		{
			if (!from) return std::string();
			if (len == 0) len = wcslen(from);
			std::vector<char> buffer(len + 1);
			std::use_facet<std::ctype<wchar_t>>(locale()).narrow(from, from + len, '_', &buffer[0]);
			return std::string(&buffer[0], &buffer[len]);
		}
		inline std::string Narrow(std::string const& from) { return from; }
		inline std::string Narrow(std::wstring const& from) { return Narrow(from.c_str(), from.size()); }

		// Widen
		inline std::wstring Widen(wchar_t const* from, std::size_t len = 0)
		{
			if (!from) return std::wstring();
			if (len == 0) len = wcslen(from);
			return std::wstring(from, from + len);
		}
		inline std::wstring Widen(char const* from, std::size_t len = 0)
		{
			if (!from) return std::wstring();
			if (len == 0) len = strlen(from);
			std::vector<wchar_t> buffer(len + 1);
			std::use_facet<std::ctype<wchar_t>>(locale()).widen(from, from + len, &buffer[0]);
			return std::wstring(&buffer[0], &buffer[len]);
		}
		inline std::wstring Widen(std::wstring const& from) { return from; }
		inline std::wstring Widen(std::string const& from) { return Widen(from.c_str(), from.size()); }

		// Template specialised versions of the win32 API functions
		template <typename Char> struct Win32;
		template <> struct Win32<char>
		{
			static int WindowText(HWND hwnd, LPSTR lpString, int nMaxCount)
			{
				return ::GetWindowTextA(hwnd, lpString, nMaxCount);
			}
			static int WindowTextLength(HWND hwnd)
			{
				return ::GetWindowTextLengthA(hwnd);
			}
			static int MenuString(HMENU hMenu, UINT uIDItem, LPSTR lpString, int cchMax, UINT flags)
			{
				return ::GetMenuStringA(hMenu, uIDItem, lpString, cchMax, flags);
			}
		};
		template <> struct Win32<wchar_t>
		{
			static int WindowText(HWND hwnd, LPWSTR lpString, int nMaxCount)
			{
				return ::GetWindowTextW(hwnd, lpString, nMaxCount);
			}
			static int WindowTextLength(HWND hwnd)
			{
				return ::GetWindowTextLengthW(hwnd);
			}
			static int MenuString(HMENU hMenu, UINT uIDItem, LPWSTR lpString, int cchMax, UINT flags)
			{
				return ::GetMenuStringW(hMenu, uIDItem, lpString, cchMax, flags);
			}
		};
		#pragma endregion

		#pragma region Support Functions / Structures

		// Cast with overflow check
		template <typename TTo, typename TFrom> inline TTo cast(TFrom from)
		{
			assert("Overflow or underflow in cast" && static_cast<TFrom>(static_cast<TTo>(from)) == from);
			return static_cast<TTo>(from);
		}
		template <typename TTo, typename TFrom> inline TTo rcast(TFrom from)
		{
			return reinterpret_cast<TTo>(from);
		}

		// Convert to byte pointer
		template <typename T> byte const* bptr(T const* t) { return reinterpret_cast<byte const*>(t); }
		template <typename T> byte* bptr(T* t) { return reinterpret_cast<byte*>(t); }

		// Append bytes
		template <typename TCont> void append(TCont& cont, void const* x, size_t byte_count)
		{
			cont.insert(end(cont), bptr(x), bptr(x) + byte_count);
		}

		// Raw string copy
		template <int N> void StrCopy(char(&dest)[N], char    const* src) { strncpy(dest, src, N); dest[N - 1] = 0; }
		template <int N> void StrCopy(wchar_t(&dest)[N], wchar_t const* src) { wcsncpy(dest, src, N); dest[N - 1] = 0; }

		// All in container
		template <typename TCont, typename Pred> void for_all(TCont& cont, Pred pred)
		{
			for (auto& item : cont)
				pred(item);
		}

		// Set bits.
		template <typename T, typename U> inline bool AllSet(T value, U mask)
		{
			return static_cast<T>(value & mask) == static_cast<T>(mask);
		}
		template <typename T, typename U> inline bool AnySet(T value, U mask)
		{
			return static_cast<T>(value & mask) != 0;
		}
		template <typename T, typename U> inline T SetBits(T value, U mask, bool state)
		{
			// If 'state' is true, returns 'value | mask'. If false, returns 'value & ~mask'
			return state ? static_cast<T>(value | mask) : static_cast<T>(value & ~mask);
		}

		// FmtS
		template <typename TChar> inline TChar const* FmtS(TChar const* format, ...)
		{
			static thread_local TChar buf[1024];
			int const buf_count = _countof(buf);

			struct L {
				static int Format(char* dst, size_t max_count, char const* fmt, va_list arg_list) { return _vsprintf_p(dst, max_count, fmt, arg_list); }
				static int Format(wchar_t* dst, size_t max_count, wchar_t const* fmt, va_list arg_list) { return _vswprintf_p(dst, max_count, fmt, arg_list); }
			};

			va_list arg_list;
			va_start(arg_list, format);
			auto n = L::Format(buf, buf_count, format, arg_list);
			buf[n >= 0 ? std::min(buf_count - 1, n) : 0] = 0;
			va_end(arg_list);

			return buf;
		}

		// Enum class operator | and &
		template <typename U> static std::true_type  has_allow_bitops(decltype(U::allow_bitops)*) {};
		template <typename>   static std::false_type has_allow_bitops(...) {};
		template <typename T> constexpr bool has_allow_bitops_v = decltype(has_allow_bitops<T>(nullptr))::value;
		template <typename T, typename = std::enable_if_t<has_allow_bitops_v<T>>> inline T operator ~ (T rhs)
		{
			return static_cast<T>(~static_cast<std::underlying_type_t<T>>(rhs));
		}
		template <typename T, typename = std::enable_if_t<has_allow_bitops_v<T>>> inline T operator | (T lhs, T rhs)
		{
			auto r = static_cast<std::underlying_type_t<T>>(lhs) | static_cast<std::underlying_type_t<T>>(rhs);
			return static_cast<T>(r);
		}
		template <typename T, typename = std::enable_if_t<has_allow_bitops_v<T>>> inline T operator & (T lhs, T rhs)
		{
			auto r = static_cast<std::underlying_type_t<T>>(lhs) & static_cast<std::underlying_type_t<T>>(rhs);
			return static_cast<T>(r);
		}
		template <typename T, typename = std::enable_if_t<has_allow_bitops_v<T>>> inline T& operator |= (T& lhs, T rhs)
		{
			return lhs = lhs | rhs;
		}
		template <typename T, typename = std::enable_if_t<has_allow_bitops_v<T>>> inline T& operator &= (T& lhs, T rhs)
		{
			return lhs = lhs & rhs;
		}

		// Convert an error code into an error message
		inline std::string ErrorMessage(HRESULT result)
		{
			char msg[8192];
			DWORD length(_countof(msg));
			if (!FormatMessageA(FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS, NULL, result, MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT), msg, length, NULL))
				sprintf_s(msg, "Unknown error code: 0x%80X", result);
			return msg;
		}

		// Test an HRESULT and throw on error
		inline void Check(HRESULT result, std::string message)
		{
			if (SUCCEEDED(result)) [[likely]] return;
			throw std::runtime_error(message.append("\n").append(ErrorMessage(GetLastError())).c_str());
		}
		inline void Check(BOOL result, std::string message)
		{
			if (result != 0) [[likely]] return;
			auto hr = HRESULT(GetLastError());
			Check(SUCCEEDED(hr) ? E_FAIL : hr, message);
		}
		inline void Check(gdi::Status result, std::string message)
		{
			if (result == gdi::Status::Ok) [[likely]] return;
			throw std::runtime_error(message.c_str());
		}

		// Return the window class name that 'hwnd' is an instance of
		inline std::wstring WndClassName(HWND hwnd)
		{
			assert(::IsWindow(hwnd));

			std::wstring cn(64, 0);
			for (int len = 0; len == 0;)
			{
				len = ::GetClassNameW(hwnd, &cn[0], int(cn.size()));
				cn.resize(len == 0 ? cn.size() * 2 : len);
			}
			return cn;
		}

		// Replace macros from windowsx.h
		constexpr WORD MakeWord(DWORD_PTR lo, DWORD_PTR hi)
		{
			return WORD((lo & 0xff) | ((hi & 0xff) << 8));
		}
		constexpr LONG MakeLong(DWORD_PTR lo, DWORD_PTR hi)
		{
			return LONG((lo & 0xffff) | ((hi & 0xffff) << 16));
		}
		constexpr WORD HiWord(DWORD_PTR l)
		{
			return WORD((l >> 16) & 0xffff);
		}
		constexpr BYTE HiByte(DWORD_PTR w)
		{
			return BYTE((w >> 8) & 0xff);
		}
		constexpr WORD LoWord(DWORD_PTR l)
		{
			return WORD(l & 0xffff);
		}
		constexpr BYTE LoByte(DWORD_PTR w)
		{
			return BYTE(w & 0xff);
		}
		constexpr int GetXLParam(LPARAM lparam) // GET_X_LPARAM
		{
			return int(short(LoWord(lparam)));
		}
		constexpr int GetYLParam(LPARAM lparam) // GET_Y_LPARAM
		{
			return int(short(HiWord(lparam)));
		}
		constexpr WPARAM MakeWParam(int lo, int hi)
		{
			return WPARAM(MakeLong(lo, hi));
		}
		constexpr LPARAM MakeLParam(int lo, int hi)
		{
			return LPARAM(MakeLong(lo, hi));
		}

		// Replacements for win32 macros
		inline LPCWSTR MakeIntResourceW(int i) // MAKEINTRESOURCEW
		{
			return rcast<LPCWSTR>(cast<WORD>(i) + (LPCSTR)0);
		}
		inline LPCWSTR MakeIntAtomW(ATOM atom) // MAKEINTATOM
		{
			return rcast<LPCWSTR>(cast<WORD>(atom) + (LPCSTR)0);
		}
		inline bool IsIntResource(LPCWSTR res_id) // IS_INTRESOURCE
		{
			return res_id != nullptr && HiWord(rcast<LPCSTR>(res_id) - (LPCSTR)0) == 0;
		}
		inline bool IsIntResource(LPCSTR res_id) // IS_INTRESOURCE
		{
			return res_id != nullptr && HiWord(rcast<LPCSTR>(res_id) - (LPCSTR)0) == 0;
		}
		inline WORD ResourceInt(LPCWSTR res)
		{
			assert(IsIntResource(res));
			return LoWord(rcast<LPCSTR>(res) - (LPCSTR)0);
		}
		inline WORD ResourceInt(LPCSTR res)
		{
			assert(IsIntResource(res));
			return LoWord(rcast<LPCSTR>(res) - (LPCSTR)0);
		}

		// Select 'Lhs' if not void, otherwise 'Rhs'
		template <typename Lhs, typename Rhs>
		using not_void_t = std::conditional_t<std::is_void_v<Lhs>, Rhs, Lhs>;

		// Helper for changing the state of a variable, restoring it on destruction
		template <typename T> struct RAII
		{
			T* m_var;
			T old_value;
			RAII(T& var, T new_value) :m_var(&var), old_value(var) { *m_var = new_value; }
			~RAII() { *m_var = old_value; }
			RAII(RAII const&) = delete;
			RAII& operator =(RAII const&) = delete;
		};

		// An RAII object that calls a lambda at scope exit
		template <typename Func> struct ScopeExit
		{
			Func m_func;
			bool m_doit;

			~ScopeExit()
			{
				if (!m_doit) return;
				m_func();
			}
			ScopeExit(Func func)
				:m_func(func)
				, m_doit(true)
			{}
			ScopeExit(ScopeExit&& rhs)
				:m_func(rhs.m_func)
				, m_doit(rhs.m_doit)
			{
				rhs.m_doit = false;
			}
			ScopeExit& operator = (ScopeExit&& rhs)
			{
				if (this == &rhs) return *this;
				std::swap(m_func, rhs.m_func);
				std::swap(m_doit, rhs.m_doit);
				return *this;
			}
			ScopeExit(ScopeExit const&) = delete;
			ScopeExit& operator = (ScopeExit const&) = delete;
		};
		template <typename Func> ScopeExit<Func> OnScopeExit(Func func)
		{
			return std::move(ScopeExit<Func>(func));
		}

		// Represent a handle or id of a resource (e.g. HMENU, HACCEL, etc).
		template <typename HandleType = void*> struct ResId
		{
			wchar_t const* m_res_id;
			HandleType m_handle;

			ResId() :m_res_id(), m_handle() {}
			ResId(wchar_t const* res) :m_res_id(res), m_handle() {}
			ResId(HandleType handle) :m_res_id(), m_handle(handle) {}
			ResId(int id) :m_res_id(id != ID_UNUSED ? MakeIntResourceW(id) : nullptr), m_handle() {}
			ResId(char const*) = delete; // Prevent accidental use of narrow resource name strings

			bool operator == (nullptr_t) const { return m_handle == nullptr && m_res_id == nullptr; }
			bool operator != (nullptr_t) const { return !(*this == nullptr); }
			WORD id() const { return IsIntResource(m_res_id) ? ResourceInt(m_res_id) : ID_UNUSED; }
		};

		// Send message casting helper
		template <typename TRet, typename WP, typename LP> struct SendMsg
		{
			static TRet Send(HWND hwnd, UINT msg, WP wparam, LP lparam) { return TRet(::SendMessageW(hwnd, msg, WPARAM(wparam), LPARAM(lparam))); }
		};
		template <typename WP, typename LP> struct SendMsg<bool, WP, LP>
		{
			static bool Send(HWND hwnd, UINT msg, WP wparam, LP lparam) { return ::SendMessageW(hwnd, msg, WPARAM(wparam), LPARAM(lparam)) != 0; }
		};

		// Select an object into an 'hdc'
		struct SelectObject
		{
			HDC m_hdc; HGDIOBJ m_old;
			~SelectObject() {
				if (m_old) ::SelectObject(m_hdc, m_old);
			}
			SelectObject(HDC hdc, HGDIOBJ obj)
				:m_hdc(hdc)
				, m_old(::SelectObject(hdc, obj))
			{}
			SelectObject(SelectObject&& rhs) noexcept
				:m_hdc(rhs.m_hdc)
				, m_old()
			{
				std::swap(m_old, rhs.m_old);
			}
		};

		// Basically CComPtr with needing to include atlbase.h
		template <typename T> struct CComPtr
		{
			T* p;
			CComPtr(T* ptr = nullptr) :p(ptr) { if (p != nullptr) p->AddRef(); }
			CComPtr(CComPtr&& rhs) :p() { std::swap(p, rhs.p); }
			CComPtr(CComPtr const&) = delete;
			CComPtr& operator = (CComPtr&& rhs) noexcept { std::swap(p, rhs.p); return *this; }
			CComPtr& operator = (CComPtr const&) = delete;
			~CComPtr() { if (p != nullptr) p->Release(); }
			explicit operator bool() const { return p != nullptr; }
			bool operator == (nullptr_t) const { return p == nullptr; }
			T* operator -> () const { return p; }
		};

		// Forward
		CComPtr<IStream> StreamFromResource(HINSTANCE inst, wchar_t const* resource, wchar_t const* res_type);

		#pragma pack(push,8)
		struct AtlThunkData_t; // opaque
		struct StdCallThunk
		{
			inline static HMODULE atlthunk_dll = LoadLibraryW(L"atlthunk.dll");
			using AllocFn = AtlThunkData_t* (__stdcall *)();
			using FreeFn = void(__stdcall *)(AtlThunkData_t*);
			using InitFn = void(__stdcall *)(AtlThunkData_t*, void*, size_t);
			using CodeFn = void*(__stdcall *)(AtlThunkData_t*);

			AtlThunkData_t* m_thunk;
			StdCallThunk() :m_thunk(reinterpret_cast<AllocFn>(GetProcAddress(atlthunk_dll, "AtlThunk_AllocateData"))()) {}
			StdCallThunk(StdCallThunk&&) = delete;
			StdCallThunk(StdCallThunk const&) = delete;
			StdCallThunk& operator = (StdCallThunk&&) = delete;
			StdCallThunk& operator = (StdCallThunk const&) = delete;
			~StdCallThunk()
			{
				if (!m_thunk) return;
				reinterpret_cast<FreeFn>(GetProcAddress(atlthunk_dll, "AtlThunk_FreeData"))(m_thunk);
			}
			void Init(_In_ DWORD_PTR proc, _In_opt_ void* this_)
			{
				Check(m_thunk != nullptr, "Failed to allocate thunk data");
				reinterpret_cast<InitFn>(GetProcAddress(atlthunk_dll, "AtlThunk_InitData"))(m_thunk, (void*)proc, (size_t)this_);
			}
			void* GetCodeAddress() const
			{
				return reinterpret_cast<CodeFn>(GetProcAddress(atlthunk_dll, "AtlThunk_DataToCode"))(m_thunk);
			}
		};
		#pragma pack(pop)

		#pragma endregion

		#pragma region Win32 Structure Wrappers

		struct Point;
		struct Rect;
		struct Size;

		// String
		using string = std::wstring;
		using string_view = std::wstring_view;

		// Point
		struct Point :POINT
		{
			Point() :POINT() {}
			Point(long x_, long y_) { x = x_; y = y_; }
			explicit Point(SIZE sz) { x = sz.cx; y = sz.cy; }
			explicit Point(LPARAM lparam) :Point(GetXLParam(lparam), GetYLParam(lparam)) {}
			long operator[](int axis) const { return axis == 0 ? x : y; }
		};

		// Size
		struct Size :SIZE
		{
			Size() :SIZE() {}
			Size(long sx, long sy) { cx = sx; cy = sy; }
			explicit Size(POINT pt) { cx = pt.x; cy = pt.y; }
			operator Rect() const;
			long operator[](int axis) const { return axis == 0 ? cx : cy; }
			float aspect() const { return float(cx) / cy; }
		};

		// Rect
		struct Rect :RECT
		{
			Rect() :RECT()                                {}
			Rect(RECT const& r) :RECT(r)                  {}
			Rect(POINT const& pt, SIZE const& sz) :RECT() { left = pt.x; top = pt.y; right = pt.x + sz.cx; bottom = pt.y + sz.cy; }
			Rect(int l, int t, int r, int b)              { left = l; top = t; right = r; bottom = b; }
			explicit Rect(Size s)                         { left = top = 0; right = s.cx; bottom = s.cy; }
			bool empty() const                            { return left == right && top == bottom; }
			long area() const                             { return width() * height(); }
			long width() const                            { return right - left; }
			void width(long w)                            { right = left + w; }
			long height() const                           { return bottom - top; }
			void height(long h)                           { bottom = top + h; }
			Size size() const                             { return Size{width(), height()}; }
			void size(Size sz)                            { right = left + sz.cx; bottom = top + sz.cy; }
			int  size(int axis) const                     { return axis == 0 ? width() : height(); }
			float aspect() const                          { return float(width()) / height(); }
			Point centre() const                          { return Point((left + right) / 2, (top + bottom) / 2); }
			void centre(Point pt)                         { long w = width(), h = height(); left = pt.x-w/2; right = left+w; top = pt.y-h/2; bottom = top+h; }
			Point const* points() const                   { return reinterpret_cast<Point const*>(&left); }
			Point const& topleft() const                  { return points()[0]; }
			Point const& bottomright() const              { return points()[1]; }
			Point* points()                               { return reinterpret_cast<Point*>(&left); }
			Point& topleft()                              { return points()[0]; }
			Point& bottomright()                          { return points()[1]; }

			// This function returns false if the result is a zero rect (that's why I'm not using Check())
			// The returned rect is the bounding box of the geometric operation (note how that effects subtract)
			bool Contains(Point const& pt, bool incl = false) const
			{
				return incl
					? pt.x >= left && pt.x <= right && pt.y >= top && pt.y <= bottom
					: pt.x >= left && pt.x <  right && pt.y >= top && pt.y <  bottom;
			}
			Rect Shifted(int dx, int dy) const
			{
				auto r = *this;
				::OffsetRect(&r, dx, dy);
				return r;
			}
			Rect Shifted(Size const& dxy) const
			{
				auto r = *this;
				::OffsetRect(&r, dxy.cx, dxy.cy);
				return r;
			}
			Rect Inflate(int dx, int dy) const
			{
				auto r = *this;
				::InflateRect(&r, dx, dy);
				return r;
			}
			Rect Adjust(int dl, int dt, int dr, int db) const
			{
				auto r = *this;
				r.left += dl;
				r.top += dt;
				r.right += dr;
				r.bottom += db;
				return r;
			}
			Rect Adjust(Rect const& adj) const
			{
				auto r = *this;
				r.left += adj.left;
				r.top += adj.top;
				r.right += adj.right;
				r.bottom += adj.bottom;
				return r;
			}
			Rect Intersect(Rect const& rhs) const
			{
				auto r = *this;
				::IntersectRect(&r, this, &rhs);
				return r;
			}
			Rect Union(Rect const& rhs) const
			{
				auto r = *this;
				if (*this != rhs) // UnionRect has a bug if these are equal, (returns [0x0])
					::UnionRect(&r, this, &rhs);
				return r;
			}
			Rect Subtract(Rect const& rhs) const
			{
				// Reduces the size of this rectangle by excluding the area 'rhs'.
				// The result must be a rectangle or an exception is thrown.
				// Note: "::SubtractRect" doesn't work how I'd expect
				auto lhs = *this;

				// If 'x' has no area, then subtraction is identity
				if (rhs.empty())
					return lhs;

				// If the rectangles do not overlap. Right/Bottom is not considered 'in' the rectangle
				if (lhs.left >= rhs.right || lhs.right <= rhs.left || lhs.top >= rhs.bottom || lhs.bottom <= rhs.top)
					return lhs;

				// If 'x' completely covers 'r' then the result is empty
				if (rhs.left <= lhs.left && rhs.right >= lhs.right && rhs.top <= lhs.top && rhs.bottom >= lhs.bottom)
				{
					lhs.right = lhs.left;
					lhs.bottom = lhs.top;
					return lhs;
				}

				// If 'x' spans 'r' horizontally
				if (rhs.left <= lhs.left && rhs.right >= lhs.right)
				{
					// If the top edge of 'lhs' is aligned with the top edge of 'rhs', or within 'rhs'
					// then the top edge of the resulting rectangle is 'rhs.bottom'.
					if (rhs.top    <= lhs.top)    return Rect(lhs.left, rhs.bottom, lhs.right, lhs.bottom);
					if (rhs.bottom >= lhs.bottom) return Rect(lhs.left, lhs.top, lhs.right, rhs.top);
					throw std::runtime_error("The result of subtracting rectangle 'rhs' does not result in a rectangle");
				}

				// If 'rhs' spans 'lhs' vertically
				if (rhs.top <= lhs.top && rhs.bottom >= lhs.bottom)
				{
					if (rhs.left  <= lhs.left)  return Rect(rhs.right, lhs.top, lhs.right, lhs.bottom);
					if (rhs.right >= lhs.right) return Rect(lhs.left, lhs.top, rhs.left, lhs.bottom);
					throw std::runtime_error("The result of subtracting rectangle 'rhs' does not result in a rectangle");
				}

				throw std::runtime_error("The result of subtracting rectangle 'rhs' does not result in a rectangle");
			}
			Rect NormalizeRect() const
			{
				auto r = *this;
				if (r.left > r.right) std::swap(r.left, r.right);
				if (r.top > r.bottom) std::swap(r.top, r.bottom);
				return r;
			}

			Rect operator +() const { return Rect(+left, +top, +right, +bottom); }
			Rect operator -() const { return Rect(-left, -top, -right, -bottom); }
			bool operator == (Rect const& rhs) const { return left == rhs.left && top == rhs.top && right == rhs.right && bottom == rhs.bottom; }
			bool operator != (Rect const& rhs) const { return !(*this == rhs); }

			static Rect invalid() { return Rect(INT_MAX, INT_MAX, -INT_MAX, -INT_MAX); }
			static void Grow(Rect& lhs, Rect const& rhs)
			{
				if (lhs.left   > rhs.left  ) lhs.left   = rhs.left  ;
				if (lhs.top    > rhs.top   ) lhs.top    = rhs.top   ;
				if (lhs.right  < rhs.right ) lhs.right  = rhs.right ;
				if (lhs.bottom < rhs.bottom) lhs.bottom = rhs.bottom;
			}
		};

		inline Point operator + (Point p, Size s) { return Point(p.x + s.cx, p.y + s.cy); }
		inline Size  operator + (Size l, Size r) { return Size(l.cx + r.cx, l.cy + r.cy); }
		inline Size  operator - (Point l, Point r) { return Size(l.x + r.x, l.y - r.y); }
		inline Point operator - (Point l, Size r) { return Point(l.x - r.cx, l.y - r.cy); }
		inline bool  operator == (Point const& l, Point const& r) { return l.x == r.x && l.y == r.y; }
		inline bool  operator != (Point const& l, Point const& r) { return !(l == r); }
		inline bool  operator == (Size const& l, Size const& r) { return l.cx == r.cx && l.cy == r.cy; }
		inline bool  operator != (Size const& l, Size const& r) { return !(l == r); }
		inline Size::operator Rect() const { return Rect(*this); }

		// Range
		struct RangeI
		{
			int beg;
			int end;
			RangeI() :beg() ,end() {}
			RangeI(int b, int e) :beg(b) ,end(e) {}
			int size() const { return end - beg;}
		};

		// Window min/max size and position
		struct MinMaxInfo :MINMAXINFO
		{
			// Set mask bits write to the min/max info
			// Unset bits read from the min/max info
			enum EMask
			{
				MaxSize      = 1 << 0,
				MaxPosition  = 1 << 1,
				MinTrackSize = 1 << 2,
				MaxTrackSize = 1 << 3,
				allow_bitops = 0,
			};
			EMask m_mask;
			
			MinMaxInfo(UINT dpi)
				:MINMAXINFO()
				, m_mask()
			{
				ptMaxSize.x      = GetSystemMetricsForDpi(SM_CXVIRTUALSCREEN, dpi);
				ptMaxSize.y      = GetSystemMetricsForDpi(SM_CYVIRTUALSCREEN, dpi);
				ptMaxPosition.x  = GetSystemMetricsForDpi(SM_XVIRTUALSCREEN, dpi) + ptMaxSize.x;
				ptMaxPosition.y  = GetSystemMetricsForDpi(SM_YVIRTUALSCREEN, dpi) + ptMaxSize.y;
				ptMinTrackSize.x = GetSystemMetricsForDpi(SM_CXMINTRACK, dpi);
				ptMinTrackSize.y = GetSystemMetricsForDpi(SM_CYMINTRACK, dpi);
				ptMaxTrackSize.x = GetSystemMetricsForDpi(SM_CXMAXTRACK, dpi);
				ptMaxTrackSize.y = GetSystemMetricsForDpi(SM_CYMAXTRACK, dpi);
			}
			Rect Bounds() const
			{
				return Rect(ptMaxPosition.x - ptMaxSize.x, ptMaxPosition.y - ptMaxSize.y, ptMaxPosition.x, ptMaxPosition.y);
			}
		};

		// Window position information
		struct WindowPos :WINDOWPOS
		{
			WindowPos(HWND hwnd_ = nullptr)
				:WindowPos(hwnd_, 0, 0, 0, 0, EWindowPos::NoMove|EWindowPos::NoSize|EWindowPos::NoZorder)
			{}
			WindowPos(HWND hwnd_, Rect const& rect, EWindowPos flags_ = EWindowPos::NoZorder)
				:WindowPos(hwnd_, rect.left, rect.top, rect.width(), rect.height(), flags_)
			{}
			WindowPos(HWND hwnd_, int x_, int y_, int cx_, int cy_, EWindowPos flags_ = EWindowPos::NoZorder) :WINDOWPOS()
			{
				hwnd = hwnd_;
				x = x_;
				y = y_;
				cx = cx_;
				cy = cy_;
				flags = (UINT)flags_;
			}
			Rect Bounds() const
			{
				return Rect(x, y, x + cx, y + cy);
			}
		};

		// Monitor info
		struct MonitorInfo :MONITORINFO
		{
			MonitorInfo() :MONITORINFO() { cbSize = sizeof(MONITORINFO); }
			static MonitorInfo FromWindow(HWND hwnd, DWORD flags = MONITOR_DEFAULTTONEAREST)
			{
				MonitorInfo info;
				auto hmon = MonitorFromWindow(hwnd, flags);
				Check(::GetMonitorInfoW(hmon, &info), "Get monitor info failed");
				return info;
			}
		};

		// Metrics for the non-client regions of windows
		struct NonClientMetrics :NONCLIENTMETRICSW
		{
			NonClientMetrics() :NONCLIENTMETRICSW()
			{
				cbSize = sizeof(NONCLIENTMETRICSW);
				Check(::SystemParametersInfoW(SPI_GETNONCLIENTMETRICS, sizeof(NONCLIENTMETRICS), this, 0), "Failed to read non-client system metrics");
			}
		};

		// Device context
		struct DC
		{
			// Never cache a DC. If you have an expensive initialisation of a DC
			// use a ClassDC (CS_CLASSDC) or PrivateDC (CS_OWNDC) instead.
			HDC  m_hdc;
			bool m_owned;
			DC(HDC hdc, bool owned = false) :m_hdc(hdc) ,m_owned(owned) {}
			~DC() { if (m_owned && m_hdc) ::DeleteDC(m_hdc); }
			operator HDC() const { return m_hdc; }
		};
		struct MemDC :DC
		{
			HDC m_hdc_orig;
			Rect m_rect;
			HBITMAP m_bmp;
			HBITMAP m_bmp_old;
			bool m_owns_bmp;

			MemDC(HDC hdc, Rect const& rect, HBITMAP bmp)
				:DC(::CreateCompatibleDC(hdc), true)
				,m_hdc_orig(hdc)
				,m_rect(rect)
				,m_bmp(bmp ? bmp : ::CreateCompatibleBitmap(hdc, rect.width(), rect.height()))
				,m_bmp_old(HBITMAP(::SelectObject(m_hdc, m_bmp)))
				,m_owns_bmp(bmp == nullptr)
			{
				assert(m_bmp != nullptr);
				::SetViewportOrgEx(m_hdc, -m_rect.left, -m_rect.top, nullptr);
			}
			~MemDC()
			{
				::BitBlt(m_hdc_orig, m_rect.left, m_rect.top, m_rect.width(), m_rect.height(), m_hdc, m_rect.left, m_rect.top, SRCCOPY);
				::SelectObject(m_hdc, m_bmp_old);
				if (m_owns_bmp) ::DeleteObject(m_bmp);
			}
		};
		struct ClientDC :DC
		{
			// A ClientDC is restricted to the client area of the window
			// The DC returned by PaintStruct is just a ClientDC with the
			// clipping region set to the update region.
			HWND m_hwnd;
			ClientDC(HWND hwnd) :DC(::GetDC(hwnd), false) ,m_hwnd(hwnd) {}
			~ClientDC() { if (m_hwnd && m_hdc) ::ReleaseDC(m_hwnd, m_hdc); }
		};
		struct WindowDC :DC
		{
			// A WindowDC can access both client and non-client areas of a window
			HWND m_hwnd;
			WindowDC(HWND hwnd) :DC(::GetWindowDC(hwnd), false) ,m_hwnd(hwnd) {}
			~WindowDC() { ::ReleaseDC(m_hwnd, m_hdc); }
		};

		// Font
		struct Font
		{
			enum class ETypes { Raster, Vector, TrueType };
			enum EFaceName { CourierNew, Tahoma, SegoeUI };
			static constexpr wchar_t const* FaceName(EFaceName fam)
			{
				switch (fam)
				{
					case EFaceName::CourierNew: return L"CourierNew";
					case EFaceName::Tahoma:     return L"Tahoma";
					case EFaceName::SegoeUI:    return L"Segoe UI";
					default: return L"";
				}
			}

			struct Params
			{
				wchar_t const* m_face_name = FaceName(EFaceName::Tahoma);
				int m_point_size = 12;
				int m_weight = FW_NORMAL;
				bool m_italic = false;
				bool m_underline = false;
				bool m_strike_out = false;
				uint8_t m_charset = DEFAULT_CHARSET;
				HFONT m_hfont = nullptr;
				HDC m_hdc = nullptr;

				Params& family(EFaceName fname)
				{
					m_face_name = FaceName(fname);
					return *this;
				}
				Params& pt_size(int sz)
				{
					m_point_size = sz;
					return *this;
				}
				Params& bold()
				{
					return weight(600);
				}
				Params& weight(int w)
				{
					m_weight = w;
					return *this;
				}
				Params& italic(bool i)
				{
					m_italic = i;
					return *this;
				}
				Params& underline(bool u)
				{
					m_underline = u;
					return *this;
				}
				Params& strike_out(bool s)
				{
					m_strike_out = s;
					return *this;
				}
				Params& charset(uint8_t cs)
				{
					m_charset = cs;
					return *this;
				}
				Params& hfont(HFONT hf)
				{
					m_hfont = hf;
					return *this;
				}
				Params& hdc(HDC hdc)
				{
					m_hdc = hdc;
					return *this;
				}
			};
			struct Modify
			{
				enum class EMask
				{
					None      = 0,
					PointSize = 1 << 0,
					Weight    = 1 << 1,
					Italic    = 1 << 2,
					Underline = 1 << 3,
					StrikeOut = 1 << 4,
					allow_bitops = 0,
				};

				EMask m_modify = EMask::None;
				int m_point_size = {};
				int m_weight = {};
				bool m_italic = {};
				bool m_underline = {};
				bool m_strikeout = {};
				HDC m_hdc = nullptr;

				Modify& point_size(int ps) noexcept
				{
					m_point_size = ps;
					m_modify |= EMask::PointSize;
					return *this;
				}
				Modify& weight(int w) noexcept
				{
					m_weight = w;
					m_modify |= EMask::Weight;
					return *this;
				}
				Modify& italic(bool i) noexcept
				{
					m_italic = i;
					m_modify |= EMask::Italic;
					return *this;
				}
				Modify& underline(bool u) noexcept
				{
					m_underline = u;
					m_modify |= EMask::Underline;
					return *this;
				}
				Modify& strikeout(bool s) noexcept
				{
					m_strikeout = s;
					m_modify |= EMask::StrikeOut;
					return *this;
				}
				Modify& hdc(HDC hdc) noexcept
				{
					m_hdc = hdc;
					return *this;
				}
			};

			HFONT m_obj;
			bool m_owned;

			Font()
				:Font(HFONT(GetStockObject(DEFAULT_GUI_FONT)), false)
			{}
			Font(HFONT obj, bool owned = true)
				:m_obj(obj)
				,m_owned(owned)
			{}
			explicit Font(Params const& p)
			{
				auto client_dc = ClientDC(nullptr);
				auto hdc_ = p.m_hdc ? p.m_hdc : client_dc.m_hdc;

				auto lf = LOGFONTW{};
				lf.lfCharSet   = p.m_charset;
				lf.lfWeight    = p.m_weight;
				lf.lfItalic    = p.m_italic ? TRUE : FALSE;
				lf.lfUnderline = p.m_underline ? TRUE : FALSE;
				lf.lfStrikeOut = p.m_strike_out ? TRUE : FALSE;
				lf.lfHeight	   = -MulDiv(p.m_point_size, ::GetDeviceCaps(hdc_, LOGPIXELSY), 72);
				::wcsncpy_s(lf.lfFaceName, _countof(lf.lfFaceName), p.m_face_name, _TRUNCATE);

				m_obj = ::CreateFontIndirectW(&lf);
				m_owned = true;
			}
			Font(HFONT font, Modify const& p)
			{
				auto lf = LOGFONTW{};
				GetObjectW(font, sizeof(LOGFONTW), &lf);

				if (AllSet(p.m_modify, Modify::EMask::PointSize))
				{
					// convert point_size to logical units based on hDC
					auto client_dc = ClientDC(nullptr);
					auto hdc_ = p.m_hdc ? p.m_hdc : client_dc.m_hdc;
					auto pt = Point(0, ::MulDiv(::GetDeviceCaps(hdc_, LOGPIXELSY), p.m_point_size, 720)); // 72 points/inch, 10 'decipoints'/point
					auto ptOrg = Point{};
					::DPtoLP(hdc_, &pt, 1);
					::DPtoLP(hdc_, &ptOrg, 1);
					lf.lfHeight = -abs(pt.y - ptOrg.y);
					lf.lfWeight = 0;
				}
				if (AllSet(p.m_modify, Modify::EMask::Weight))
				{
					lf.lfWeight = p.m_weight;
				}
				if (AllSet(p.m_modify, Modify::EMask::Italic))
				{
					lf.lfItalic = p.m_italic;
				}
				if (AllSet(p.m_modify, Modify::EMask::Underline))
				{
					lf.lfUnderline = p.m_underline;
				}
				if (AllSet(p.m_modify, Modify::EMask::StrikeOut))
				{
					lf.lfStrikeOut = p.m_strikeout;
				}
				m_obj = ::CreateFontIndirectW(&lf);
				m_owned = true;
			}
			~Font()
			{
				if (m_owned)
					::DeleteObject(m_obj);
			}
			operator HFONT() const
			{
				return m_obj;
			}
		};
		struct TextMetrics :TEXTMETRICW
		{
			TextMetrics()
				:TEXTMETRICW()
			{}
		};

		// Brush
		// Note: ownership is lost with copying
		// Note: implicit conversion constructors are deliberate
		struct Brush
		{
			HBRUSH m_obj;
			bool m_owned;

			~Brush()
			{
				if (!m_owned || !m_obj) return;
				Check(::DeleteObject(m_obj), "Delete brush failed. It's likely still in use");
			}
			Brush()
				:m_obj(nullptr)
				,m_owned(false)
			{}
			Brush(HBRUSH obj, bool owned = false)
				:m_obj(obj)
				,m_owned(owned)
			{}
			Brush(COLORREF col)
				:m_obj(CreateSolidBrush(col))
				,m_owned(true)
			{
				Check(m_obj != 0, "Failed to create solid brush");
			}
			Brush(Brush&& rhs) noexcept
				:m_obj(rhs.m_obj)
				,m_owned(rhs.m_owned)
			{
				rhs.m_owned = false;
			}
			Brush(Brush const& rhs)
				:m_obj(rhs.m_obj)
				,m_owned(false)
			{}
			Brush& operator =(Brush&& rhs) noexcept
			{
				if (this != &rhs)
				{
					std::swap(m_obj, rhs.m_obj);
					std::swap(m_owned, rhs.m_owned);
				}
				return *this;
			}
			Brush& operator =(Brush const& rhs)
			{
				if (this != &rhs)
				{
					this->~Brush();
					new (this) Brush(rhs.m_obj, false);
				}
				return *this;
			}
			operator HBRUSH() const
			{
				return m_obj;
			}
			operator COLORREF() const
			{
				return colour();
			}
			COLORREF colour() const
			{
				if (m_obj == nullptr)
					return CLR_INVALID;

				LOGBRUSH lb = {};
				::GetObjectW(m_obj, sizeof(lb), &lb);
				return lb.lbColor;
			}

			static Brush Halftone()
			{
				// Create a 'gray' pattern
				WORD pat[8] = {0x5555, 0xaaaa, 0x5555, 0xaaaa, 0x5555, 0xaaaa, 0x5555, 0xaaaa};
				auto bm_gray = CreateBitmap(8, 8, 1, 1, &pat);
				Check(bm_gray != nullptr, "Failed to create half-tone brush");
				auto bsh = ::CreatePatternBrush(bm_gray);
				::DeleteObject(bm_gray);
				return std::move(Brush(bsh, true));
			}
		};

		// Image - bitmap, cursor, or icon
		// Note: ownership is lost with copying
		// Note: implicit conversion constructors are deliberate
		struct Image
		{
			enum class EType { Bitmap = IMAGE_BITMAP, Icon = IMAGE_ICON, Cursor = IMAGE_CURSOR, EnhMetaFile = IMAGE_ENHMETAFILE, Jpeg, Png, Unknown, };
			enum class EFit { Unchanged = 0, Tile, Zoom, Stretch };
			HANDLE m_obj;
			EType m_type;
			bool m_owned;

			~Image()
			{
				if (!m_owned || !m_obj) return;
				switch (m_type) {
				case EType::Bitmap: Check(::DeleteObject(HGDIOBJ(m_obj)), "Delete bitmap failed. It's likely still in use"); break;
				case EType::Icon:   Check(::DestroyIcon(HICON(m_obj)), "Delete icon failed. It's likely still in use"); break;
				case EType::Cursor: Check(::DestroyCursor(HCURSOR(m_obj)), "Delete cursor failed. It's likely still in use"); break;
				}
			}
			Image()
				:m_obj()
				,m_type(EType::Unknown)
				,m_owned()
			{}
			Image(HANDLE obj, EType type, bool owned = false)
				:m_obj(obj)
				,m_type(type)
				,m_owned(owned)
			{}
			Image(Image&& rhs) noexcept
				:m_obj(rhs.m_obj)
				,m_type(rhs.m_type)
				,m_owned(rhs.m_owned)
			{
				rhs.m_owned = false;
			}
			Image(Image const& rhs)
				:m_obj(rhs.m_obj)
				,m_type(rhs.m_type)
				,m_owned(false)
			{}
			Image& operator = (Image&& rhs) noexcept
			{
				if (this != &rhs)
				{
					std::swap(m_obj, rhs.m_obj);
					std::swap(m_type, rhs.m_type);
					std::swap(m_owned, rhs.m_owned);
				}
				return *this;
			}
			Image& operator = (Image const& rhs)
			{
				if (this != &rhs)
				{
					this->~Image();
					new (this) Image(rhs.m_obj, rhs.m_type, false);
				}
				return *this;
			}
			operator HBITMAP() const
			{
				assert("Image is not a bitmap" && m_type == EType::Bitmap);
				return HBITMAP(m_obj);
			}
			operator HICON() const // Cursor is just a typedef of icon
			{
				assert("Image is not an icon or cursor" && (m_type == EType::Icon || m_type == EType::Cursor));
				return HICON(m_obj);
			}
			bool operator == (nullptr_t) const { return m_obj == nullptr; }
			bool operator != (nullptr_t) const { return m_obj != nullptr; }

			// Create a bitmap. Note: to use this with a device context, compatibility is checked each time. Use the other CreateBitmap function for performance
			static Image CreateBitmap(int sx, int sy, UINT planes = 1, UINT bit_count = 32, void const* data = nullptr)
			{
				auto obj = ::CreateBitmap(sx, sy, planes, bit_count, data);
				Check(obj != 0, "Failed to create bitmap");
				return std::move(Image(obj, EType::Bitmap, true));
			}

			// This is more efficient if the bitmap will be selected into 'hdc' because compatibility is already known
			static Image CreateBitmap(HDC hdc, int sx, int sy)
			{
				auto obj = ::CreateCompatibleBitmap(hdc, sx, sy);
				Check(obj != 0, "Failed to create bitmap");
				return std::move(Image(obj, EType::Bitmap, true));
			}

			// Convert an image type to a resource type
			static wchar_t const* ResType(EType img_type)
			{
				// Create a GDI+ bitmap from the resource data
				switch (img_type)
				{
				default: assert(!"Unknown image type"); return L"RCDATA";
				case EType::Bitmap: return L"BITMAP";
				case EType::Icon:   return L"ICON";
				case EType::Cursor: return L"CURSOR";
				case EType::Jpeg:   return L"JPEG";
				case EType::Png:    return L"PNG";
				}
			}

			// Load a bitmap, cursor, or icon from file
			static Image Load(wchar_t const* filepath, EType type, EFit fit = EFit::Unchanged, int cx = 0, int cy = 0, UINT flags = LR_DEFAULTCOLOR|LR_DEFAULTSIZE)
			{
				return LoadInternal(true, filepath, nullptr, type, fit, cx, cy, flags);
			}

			// Load a bitmap, cursor, or icon from a resource.
			// Note: to load an OEM resource (cursor, icon, etc) you need to specify 'hinst=nullptr' and 'flags|=LR_SHARED'
			static Image Load(HINSTANCE hinst, wchar_t const* resource, EType type, EFit fit = EFit::Unchanged, int cx = 0, int cy = 0, UINT flags = LR_DEFAULTCOLOR|LR_DEFAULTSIZE)
			{
				return LoadInternal(false, resource, hinst, type, fit, cx, cy, flags);
			}

			// Return bitmap info for a bitmap handle
			static BITMAP Info(HBITMAP hbmp)
			{
				BITMAP info = {};
				if (hbmp) Check(::GetObjectW(hbmp, sizeof(BITMAP), &info), "Get Bitmap info failed");
				return info;
			}

		private:

			// Load a bitmap, cursor, or icon from a resource.
			static Image LoadInternal(bool file, wchar_t const* resource, HINSTANCE hinst, EType type, EFit fit, int cx, int cy, UINT flags)
			{
				// Note: GDI can only load icons, bitmaps, or cursors from resources, not PNGs, JPGS, etc.
				// To load the other image formats, we need to treat the resource data as a stream and use
				// GDI+ to create a bitmap from it. GDI does not handle smart scaling either, so only use
				// the basic LoadImage function for the simple cases.
				// Some examples:
				//   Load(GetModuleHandle(), MakeIntResourceW(OCR_NORMAL), EType::Cursor);
				
				// The simple case
				if (type == EType::Icon || type == EType::Cursor || type == EType::EnhMetaFile || (type == EType::Bitmap && fit == EFit::Unchanged))
				{
					auto h = ::LoadImageW(file ? nullptr : hinst, resource, (int)type, cx, cy, flags);
					Check(h != nullptr, "LoadImage failed");
					return std::move(Image(h, type, true));
				}

				// Create a GDI+ bitmap from the resource data then get a GDI bitmap from the GDI+ bitmap.
				// We can allow the GDI+ bitmap to be destroyed because windows makes a copy.
				
				// Create a COM stream of the resource data
				CComPtr<IStream> stream;

				// A buffer to load the image from file into
				std::vector<char> buf;
				if (file)
				{
					std::ifstream infile(resource, std::ios::binary);
					buf.assign(std::istreambuf_iterator<char>(infile), std::istreambuf_iterator<char>());
					stream = CComPtr<IStream>(SHCreateMemStream(rcast<BYTE*>(buf.data()), int(buf.size())));
				}
				else
				{
					stream = gui::StreamFromResource(hinst, resource, ResType(type));
				}

				// Create a GDI+ bitmap from the stream
				gdi::Bitmap orig(stream.p);
				HBITMAP hbmp;

				// No scaling
				if (fit == EFit::Unchanged)
				{
					// Create the GDI bitmap from the GDI+ bitmap
					Check(orig.GetHBITMAP(gdi::ARGB(gdi::Color::White), &hbmp), "Failed to get HBITMAP from GDI+ bitmap");
				}
				else // Scaled/tiled/etc
				{
					if (cx == 0) cx = int(orig.GetWidth());
					if (cy == 0) cy = int(orig.GetHeight());

					// Create a GDI+ bitmap of the required size
					gdi::Bitmap bmp(cx, cy, orig.GetPixelFormat());
					gdi::Graphics gfx(&bmp);
					gfx.SetInterpolationMode(gdi::InterpolationMode::InterpolationModeHighQuality);

					// Apply the scaling type to the bitmap
					switch (fit)
					{
					default:
						{
							assert(!"Unknown image fit type");
							break;
						}
					case EFit::Tile:
						{
							gdi::TextureBrush bsh(&orig, gdi::WrapMode::WrapModeTile);
							gfx.FillRectangle(&bsh, 0, 0, bmp.GetWidth(), bmp.GetHeight());
							break;
						}
					case EFit::Stretch:
						{
							gfx.DrawImage(&orig, 0, 0, bmp.GetWidth(), bmp.GetHeight());
							break;
						}
					case EFit::Zoom:
						{
							// Scale preserving aspect
							float x = 0, y = 0, w, h;
							auto orig_aspect = float(orig.GetWidth()) / orig.GetHeight();
							if (bmp.GetWidth() * orig.GetHeight() > orig.GetWidth() * bmp.GetHeight()) // height constrained
							{
								w = float(bmp.GetHeight()) * orig_aspect;
								h = float(bmp.GetHeight());
								x = fabsf(w - float(bmp.GetWidth())) / 2.0f;
							}
							else // width constrained
							{
								w = float(bmp.GetWidth());
								h = float(bmp.GetWidth()) / orig_aspect;
								y = fabsf(h - float(bmp.GetHeight())) / 2.0f;
							}
							gfx.DrawImage(&orig, x, y, w, h);
							break;
						}
					}

					// Create the GDI bitmap from the GDI+ bitmap
					Check(bmp.GetHBITMAP(gdi::ARGB(gdi::Color::White), &hbmp), "Failed to get HBITMAP from GDI+ bitmap");
				}

				// Return the scaled image
				return std::move(Image(hbmp, EType::Bitmap, true));
			}
		};

		// Keyboard accelerators
		// Note: ownership is lost with copying
		// Note: implicit conversion constructors are deliberate
		struct Accel
		{
			HACCEL m_obj;
			bool m_owned;

			~Accel()
			{
				if (!m_owned || !m_obj) return;
				Check(::DestroyAcceleratorTable(m_obj), "Delete accelerators failed. It's likely still in use");
			}
			Accel()
				:m_obj(nullptr)
				,m_owned(false)
			{}
			Accel(HACCEL obj, bool owned = false)
				:m_obj(obj)
				,m_owned(owned)
			{}
			Accel(Accel&& rhs) noexcept
				:m_obj(rhs.m_obj)
				,m_owned(rhs.m_owned)
			{
				rhs.m_owned = false;
			}
			Accel(Accel const& rhs)
				:m_obj(rhs.m_obj)
				,m_owned(false)
			{}
			Accel& operator = (Accel&& rhs) noexcept
			{
				if (this != &rhs)
				{
					std::swap(m_obj, rhs.m_obj);
					std::swap(m_owned, rhs.m_owned);
				}
				return *this;
			}
			Accel& operator = (Accel const& rhs)
			{
				if (this != &rhs)
				{
					this->~Accel();
					new (this) Accel(rhs.m_obj, false);
				}
				return *this;
			}
			operator HACCEL() const
			{
				return m_obj;
			}
		};

		// Paint
		struct PaintStruct :PAINTSTRUCT
		{
			HWND m_hwnd;
			PaintStruct(HWND hwnd) :m_hwnd(hwnd) { Check(::BeginPaint(m_hwnd, this) != nullptr, "BeginPaint failed"); }
			~PaintStruct()                       { Check(::EndPaint(m_hwnd, this), "EndPaint failed"); }
		};

		// TrackMouseEvent
		struct TrackMouseEvent :TRACKMOUSEEVENT
		{
			TrackMouseEvent()
				:TRACKMOUSEEVENT()
			{
				cbSize = sizeof(TRACKMOUSEEVENT);
			}
			TrackMouseEvent(DWORD flags, HWND wnd_to_track, DWORD hovertime_ms = HOVER_DEFAULT)
				:TrackMouseEvent()
			{
				dwFlags     = flags;
				hwndTrack   = wnd_to_track;
				dwHoverTime = hovertime_ms;
			}
			TrackMouseEvent(TRACKMOUSEEVENT tme)
				:TRACKMOUSEEVENT(tme)
			{}
		};

		// Theme
		struct Theme
		{
			static bool Available() { return ::IsAppThemed() != 0; }

			HTHEME m_htheme;

			// 'class_list' is a semi-colon separated list of class names
			Theme(HWND hwnd, wchar_t const* class_list)
				:m_htheme(::OpenThemeData(hwnd, class_list))
			{}
			~Theme()
			{
				::CloseThemeData(m_htheme);
			}
			operator HTHEME() const
			{
				return m_htheme;
			}

			// 'hdc' = what to draw on
			// 'part_id','state_id' = see https://msdn.microsoft.com/en-us/library/windows/desktop/bb773210(v=vs.85).aspx
			// 'text' = the text to draw
			// 'count' = the number of characters in 'text' to draw, use -1 for all up to null terminator
			// 'flags' = https://msdn.microsoft.com/en-us/library/windows/desktop/bb773199(v=vs.85).aspx
			// 'rect' = where to draw the text (in logical units) (or where it would be drawn)
			// 'opts' = https://msdn.microsoft.com/en-us/library/windows/desktop/bb773236(v=vs.85).aspx
			void Text(HDC hdc, int part_id, int state_id, LPCWSTR text, int count, DWORD flags, RECT* rect, DTTOPTS const* opts)
			{
				if (m_htheme != nullptr) [[likely]]
					Check(::DrawThemeTextEx(m_htheme, hdc, part_id, state_id, text, count, flags, rect, opts), "Draw theme text failed");
				else
					Check(false, "Themes not available");
			}

			// 'hdc' = what to draw on
			// 'part_id','state_id' = see https://msdn.microsoft.com/en-us/library/windows/desktop/bb773210(v=vs.85).aspx
			// 'rect' = where to draw (in logical units)
			// 'opts' = https://msdn.microsoft.com/en-us/library/windows/desktop/bb773233(v=vs.85).aspx
			void Bkgd(HDC hdc, int part_id, int state_id, RECT const* rect, DTBGOPTS const* opts) const
			{
				if (m_htheme != nullptr) [[likely]]
					Check(::DrawThemeBackgroundEx(m_htheme, hdc, part_id, state_id, rect, opts), "Draw themed background failed");
				else
					Check(false, "Themes not available");
			}

			// Retrieves the size of the content area for the background defined by the visual style.
			Rect BkgdContentRect(HDC hdc, int part_id, int state_id, RECT const* bounding_rect)
			{
				Rect res;
				if (m_htheme != nullptr) [[likely]]
					Check(::GetThemeBackgroundContentRect(m_htheme, hdc, part_id, state_id, bounding_rect, &res), "Get themed background content rect failed");
				else
					Check(false, "Themes not available");
				return res;
			}
		};

		// A window class is a template from which window instances are created.
		struct WndClassEx :WNDCLASSEXW
		{
			HINSTANCE m_hinst;
			ATOM      m_atom;
			bool      m_unreg; // True to un-register on destruction

			// 'm_unreg' is set to false on copying
			~WndClassEx()
			{
				if (!m_unreg) return;
				::UnregisterClassW(lpszClassName, m_hinst);
			}
			WndClassEx()
				:WNDCLASSEXW()
				,m_hinst()
				,m_atom()
				,m_unreg()
			{
				cbSize = sizeof(WNDCLASSEXW);
			}
			WndClassEx(HINSTANCE hinst) :WndClassEx()
			{
				m_hinst = hinst;
			}
			WndClassEx(wchar_t const* class_name, HINSTANCE hinst = GetModuleHandleW(nullptr)) :WndClassEx(hinst)
			{
				if (class_name == nullptr) return;
				m_atom = ATOM(::GetClassInfoExW(hinst, class_name, this));
			}
			explicit WndClassEx(HWND hwnd) :WndClassEx(WndClassName(hwnd).c_str())
			{}
			WndClassEx(WndClassEx&& rhs) noexcept
				:WNDCLASSEXW(rhs)
				,m_hinst(rhs.m_hinst)
				,m_atom(rhs.m_atom)
				,m_unreg(rhs.m_unreg)
			{
				rhs.m_unreg = false;
			}
			WndClassEx(WndClassEx const& rhs)
				:WNDCLASSEXW(rhs)
				,m_hinst(rhs.m_hinst)
				,m_atom(rhs.m_atom)
				,m_unreg(false)
			{}
			WndClassEx& operator = (WndClassEx&& rhs) noexcept
			{
				if (this != &rhs)
				{
					static_cast<WNDCLASSEXW&>(*this) = rhs;
					m_hinst = rhs.m_hinst;
					m_atom = rhs.m_atom;
					std::swap(m_unreg, rhs.m_unreg);
				}
				return *this;
			}
			WndClassEx& operator = (WndClassEx const& rhs)
			{
				if (this != &rhs)
				{
					static_cast<WNDCLASSEXW&>(*this) = rhs;
					m_hinst = rhs.m_hinst;
					m_atom = rhs.m_atom;
					m_unreg = false;
				}
				return *this;
			}

			// Register the this window class
			WndClassEx& Register()
			{
				m_atom = ATOM(::GetClassInfoExW(m_hinst, lpszClassName, this));
				if (m_atom == 0)
				{
					m_atom = ATOM(::RegisterClassExW(this));
					Check(m_atom != 0, "RegisterClassEx failed");
					m_unreg = true;
				}
				return *this;
			}

			// Return the INTATOM used in CreateWindowEx
			wchar_t const* IntAtom() const
			{
				assert(m_atom != 0);
				return MakeIntAtomW(m_atom);
			}
		};

		// A struct passed to WM_CREATE and OnCreate()
		struct CreateStruct :CREATESTRUCTW
		{
			CreateStruct()
				:CREATESTRUCTW()
			{}
			CreateStruct(DWORD style_ex, wchar_t const* class_name, wchar_t const* window_name, DWORD style_, int x_, int y_, int w, int h, HWND parent, HMENU menu, HINSTANCE hinst, void* params)
				:CreateStruct()
			{
				lpCreateParams = params;
				hInstance      = hinst;
				hMenu          = menu;
				hwndParent     = parent;
				cy             = h;
				cx             = w;
				y              = y_;
				x              = x_;
				style          = style_;
				lpszName       = window_name;
				lpszClass      = class_name;
				dwExStyle      = style_ex;
			}
		};
		#pragma endregion

		#pragma region Static Functions

		// Initialise common controls (makes them look modern). Must be called before creating any controls
		inline void InitCtrls(ECommonControl classes = ECommonControl::StandardClasses)
		{
			auto iccx = INITCOMMONCONTROLSEX{sizeof(INITCOMMONCONTROLSEX), DWORD(classes)};
			Check(::InitCommonControlsEx(&iccx), "Common control initialisation failed");
		}

		// Create a COM IStream from resource data
		inline CComPtr<IStream> StreamFromResource(HINSTANCE inst, wchar_t const* resource, wchar_t const* res_type)
		{
			// Find the resource and it's size
			auto hres = ::FindResourceW(inst, resource, res_type);
			auto res = hres != nullptr ? ::LoadResource(inst, hres) : nullptr;
			auto data = res != nullptr ? ::LockResource(res) : nullptr;
			auto size = hres != nullptr ? ::SizeofResource(inst, hres) : 0;
			if (!data || !size)
				throw std::runtime_error("Bitmap resource not found");

			// Create a GDI+ bitmap from the resource
			return CComPtr<IStream>(SHCreateMemStream(static_cast<BYTE const*>(data), size));
		}

		// Convert a mouse key to an index
		inline int MouseKeyToIndex(EMouseKey mk)
		{
			// Mouse keys are all single bit, so log2() returns the bit index
			return int(log2(int(mk)) + 0.5f);
		}

		// Default fonts
		template <typename Ctx>
		inline HFONT DefaultFontImpl(LOGFONTW NonClientMetrics::* logfont, bool refresh = false)
		{
			static NonClientMetrics ncm;
			static HFONT font = CreateFontIndirectW(&(ncm.*logfont));
			if (refresh)
			{
				ncm = NonClientMetrics();
				font = CreateFontIndirectW(&(ncm.*logfont));
			}
			return font;
		}
		inline HFONT DefaultGuiFont()
		{
			return HFONT(GetStockObject(DEFAULT_GUI_FONT));
		}
		inline HFONT DefaultMessageFont(bool refresh = false)
		{
			return DefaultFontImpl<struct DefMessageFont>(&NonClientMetrics::lfMessageFont, refresh);
		}
		inline HFONT DefaultMenuFont(bool refresh = false)
		{
			return DefaultFontImpl<struct DefMenuFont>(&NonClientMetrics::lfMenuFont, refresh);
		}
		inline HFONT DefaultStatusFont(bool refresh = false)
		{
			return DefaultFontImpl<struct DefStatusFont>(&NonClientMetrics::lfStatusFont, refresh);
		}
		inline HFONT DefaultCaptionFont(bool refresh = false)
		{
			return DefaultFontImpl<struct DefCaptionFont>(&NonClientMetrics::lfCaptionFont, refresh);
		}
		inline HFONT DefaultSmallCaptionFont(bool refresh = false)
		{
			return DefaultFontImpl<struct DefSmCaptionFont>(&NonClientMetrics::lfSmCaptionFont, refresh);
		}

		// Measure 'text' using the font in this control
		inline Size MeasureString(string_view text, HWND hwnd = nullptr, HFONT hfont = nullptr, int max_width = 0, UINT flags = 0)
		{
			if (text.empty())
				return {};

			// If a window is provided measure using it's DC, otherwise create a dummy window and use that.
			auto hwnd_ = hwnd ? hwnd : ::CreateWindowExW(0, L"STATIC", L"", 0, 0, 0, 0, 0, nullptr, nullptr, nullptr, nullptr);
			auto cleanup = OnScopeExit([=] { if (hwnd != hwnd_) ::DestroyWindow(hwnd_); });
			Check(::IsWindow(hwnd_), "MeasureString failed. No window available");

			ClientDC dc(hwnd_);
			SelectObject sel_font(dc, hfont ? hfont : DefaultGuiFont());

			Rect sz(0, 0, max_width, 0);
			if (max_width != 0) flags |= DT_WORDBREAK;
			Check(DrawTextW(dc, text.data(), static_cast<int>(text.size()), &sz, flags | DT_CALCRECT), "DrawTextW failed");
			return sz.size();
		}

		#pragma endregion

		#pragma region Menu
		struct MenuItem :MENUITEMINFOW
		{
			#pragma region Enums
			enum ESeparator { Separator };
			enum class EMask :UINT
			{
				None       = 0,
				Bitmap     = MIIM_BITMAP,
				CheckMarks = MIIM_CHECKMARKS,
				Data       = MIIM_DATA,
				FType      = MIIM_FTYPE,
				Id         = MIIM_ID,
				State      = MIIM_STATE,
				String     = MIIM_STRING,
				Submenu    = MIIM_SUBMENU,
				Type       = MIIM_TYPE,
				allow_bitops = 0,
			};
			enum class EType :UINT
			{
				None         = 0,
				Bitmap       = MFT_BITMAP,
				MenuBarBreak = MFT_MENUBARBREAK,
				MenuBreak    = MFT_MENUBREAK,
				OwnerDraw    = MFT_OWNERDRAW,
				RadioCheck   = MFT_RADIOCHECK,
				RightJustify = MFT_RIGHTJUSTIFY,
				RightOrder   = MFT_RIGHTORDER,
				Separator    = MFT_SEPARATOR,
				String       = MFT_STRING,
				allow_bitops = 0,
			};
			enum class EState :UINT
			{
				Default  = MFS_DEFAULT,
				Grayed   = MFS_GRAYED,
				Checked  = MFS_CHECKED,
				Uncheced = MFS_UNCHECKED,
				Enabled  = MFS_ENABLED,
				Disabled = MFS_DISABLED,
				Hilite   = MFS_HILITE,
				Unhilite = MFS_UNHILITE,
				allow_bitops = 0,
			};
			enum class EStockBmp :INT_PTR
			{
				Callback        = (INT_PTR)HBMMENU_CALLBACK       ,
				System          = (INT_PTR)HBMMENU_SYSTEM         ,
				MBar_Restore    = (INT_PTR)HBMMENU_MBAR_RESTORE   ,
				MBar_Minimize   = (INT_PTR)HBMMENU_MBAR_MINIMIZE  ,
				MBar_Close      = (INT_PTR)HBMMENU_MBAR_CLOSE     ,
				MBar_Close_D    = (INT_PTR)HBMMENU_MBAR_CLOSE_D   ,
				MBar_Minimize_D = (INT_PTR)HBMMENU_MBAR_MINIMIZE_D,
				PopUp_Close     = (INT_PTR)HBMMENU_POPUP_CLOSE    ,
				PopUp_Restore   = (INT_PTR)HBMMENU_POPUP_RESTORE  ,
				PopUp_Maximize  = (INT_PTR)HBMMENU_POPUP_MAXIMIZE ,
				PopUp_Minimize  = (INT_PTR)HBMMENU_POPUP_MINIMIZE ,
			};
			#pragma endregion

			MenuItem()
				:MENUITEMINFOW()
			{
				cbSize = sizeof(MENUITEMINFOW);
			}
			MenuItem(ESeparator)
				:MenuItem(EMask::FType, EType::Separator)
			{}
			MenuItem(wchar_t const* text, int id)
				:MenuItem(text, id, MenuItem::EState::Enabled)
			{}
			MenuItem(wchar_t const* text_, int id_, MenuItem::EState state_)
				:MenuItem()
			{
				text(text_).id(id_).state(state_);
			}
			MenuItem(wchar_t const* text_, HMENU submenu_)
				:MenuItem()
			{
				text(text_).id(ID_UNUSED).submenu(submenu_);
			}
			MenuItem(EMask mask, EType type, LPWSTR type_data = nullptr, size_t type_data_size = 0, int id = 0 ,EState state = EState::Default, HMENU submenu = nullptr, HBITMAP bmp = nullptr ,HBITMAP checked = nullptr ,HBITMAP unchecked = nullptr ,void* data = nullptr)
				:MenuItem()
			{
				fMask         = UINT(mask);           // Flags the fields that contain valid data
				fType         = UINT(type);           // used if MIIM_TYPE (4.0) or MIIM_FTYPE (>4.0)
				fState        = UINT(state);          // used if MIIM_STATE
				dwTypeData    = LPWSTR(type_data);    // used if MIIM_TYPE (4.0) or MIIM_STRING (>4.0)
				cch           = UINT(type_data_size); // used if MIIM_TYPE (4.0) or MIIM_STRING (>4.0)
				wID           = id;                   // used if MIIM_ID
				hSubMenu      = submenu;              // used if MIIM_SUBMENU
				hbmpChecked   = checked;              // used if MIIM_CHECKMARKS
				hbmpUnchecked = unchecked;            // used if MIIM_CHECKMARKS
				dwItemData    = ULONG_PTR(data);      // used if MIIM_DATA
				hbmpItem      = bmp;                  // used if MIIM_BITMAP
			}
			MenuItem(MENUITEMINFOW const& mii)
				:MENUITEMINFOW(mii)
			{}

			MenuItem& type(EType ty)                       { fMask |= (UINT)EMask::FType; fType |= (UINT)ty; return *this; }
			MenuItem& text(wchar_t const* t)               { fMask |= (UINT)EMask::String; dwTypeData = (LPWSTR)t; cch = (UINT)wcslen(t); return *this; }
			MenuItem& id(int id_)                          { fMask |= (UINT)EMask::Id; wID = (UINT)id_; return *this; }
			MenuItem& state(EState s)                      { fMask |= (UINT)EMask::State; fState = (UINT)s; return *this; }
			MenuItem& bitmap(HBITMAP bm)                   { fMask |= (UINT)EMask::Bitmap; hbmpItem = bm; return *this; }
			MenuItem& chkmarks(HBITMAP chk, HBITMAP unchk) { fMask |= (UINT)EMask::CheckMarks; hbmpChecked = chk; hbmpUnchecked = unchk; return *this; }
			MenuItem& item_data(void const* data)          { fMask |= (UINT)EMask::Data; dwItemData = (ULONG_PTR)data; return *this; }
			MenuItem& submenu(HMENU m)                     { fMask |= (UINT)EMask::Submenu; hSubMenu = m; return *this; }

			// Out parameters are used by GetMenuItemInfo()
			MenuItem& text_out(wchar_t* buf, size_t sz) { fMask |= (UINT)EMask::Type;  dwTypeData = buf; cch = (UINT)sz; return *this; }
			MenuItem& item_data_out(void* data)         { fMask |= (UINT)EMask::Data;  dwItemData = (ULONG_PTR)data; return *this; }
		};
		struct Menu
		{
			enum class EKind { Strip, Popup };
			using EType = MenuItem::EType;
			using EMask = MenuItem::EMask;
			using EState = MenuItem::EState;
			using ItemList = std::initializer_list<MenuItem>;

			HMENU m_menu;
			bool m_owned;

			// Note: ownership is lost with copying.
			// Controls/Forms don't own menus. Menu ownership is a convenience for callers
			// to automatically destroy menus, almost all other uses should be with non-owned menus.
			// Note: implicit conversion constructors are deliberate
			~Menu()
			{
				if (m_owned)
					DestroyMenu();
			}
			Menu()
				:m_menu(nullptr)
				,m_owned(false)
			{}
			Menu(HMENU menu, bool owned = false)
				:m_menu(menu)
				,m_owned(owned)
			{}
			Menu(int menu_id, HINSTANCE hinst = ::GetModuleHandleW(nullptr))
				:Menu(menu_id != ID_UNUSED ? ::LoadMenuW(hinst, MakeIntResourceW(menu_id)) : nullptr, false)
			{}
			Menu(EKind type, ItemList const& items = ItemList(), bool owned = false)
				:Menu(type == EKind::Strip ? ::CreateMenu() : type == EKind::Popup ? ::CreatePopupMenu() : nullptr, owned)
			{
				// Construct a menu from a type and a comma separated list of items
				// Allows syntax: Menu(Menu::Strip, {{L"&File",ID_UNUSED}, {L"&Help", ID_HELP}});
				for (auto& item : items)
					Insert(item);
			}

			Menu(Menu&& rhs)
				:Menu()
			{
				std::swap(m_menu, rhs.m_menu);
				std::swap(m_owned, rhs.m_owned);
			}
			Menu(Menu const& rhs)
				:Menu(rhs.m_menu, false)
			{}
			Menu& operator =(Menu&& rhs)
			{
				if (this != &rhs)
				{
					std::swap(m_menu, rhs.m_menu);
					std::swap(m_owned, rhs.m_owned);
				}
				return *this;
			}
			Menu& operator =(Menu const& rhs)
			{
				if (this != &rhs)
				{
					DestroyMenu();
					m_menu = rhs.m_menu;
					m_owned = false;
				}
				return *this;
			}

			// Destroy this menu (if owned)
			void DestroyMenu()
			{
				if (m_owned && m_menu)
					::DestroyMenu(m_menu);

				m_menu = nullptr;
				m_owned = false;
			}

			// Implicit conversion to HMENU
			operator HMENU() const
			{
				return m_menu;
			}

			// Returns the number of menu items in this menu
			size_t Count() const
			{
				assert(m_menu != nullptr);
				return size_t(::GetMenuItemCount(m_menu));
			}

			// Returns the index of a child menu item with the given text
			int IndexByName(wchar_t const* text) const
			{
				// Look for an existing menu called 'text'
				int index = 0;
				wchar_t item[256] = {};
				for (int i = 0, iend = ::GetMenuItemCount(m_menu); i != iend; ++i, ++index)
				{
					// Get the text name of the menu item
					auto len = ::GetMenuStringW(m_menu, UINT(i), item, _countof(item), MF_BYPOSITION);
					if (std::char_traits<wchar_t>::compare(text, item, len) != 0) continue;
					break;
				}
				return index;
			}

			// Insert a menu item at index position 'idx'. Use 'idx == -1' to mean append to the end
			void Insert(MenuItem const& info, int idx = -1)
			{
				assert(m_menu != nullptr);
				auto i = (idx == -1) ? DWORD(Count()) : DWORD(idx);
				//Check(::InsertMenuW(m_menu, i, MF_BYPOSITION, info.wID, info.dwTypeData), "InsertMenu failed");
				Check(::InsertMenuItemW(m_menu, i, TRUE, &info), "Insert menu item failed");
			}

			// Set a pop-up menu by name. If it exists already, then it is replaced, otherwise insert
			void Set(wchar_t const* text, Menu& submenu)
			{
				auto index = IndexByName(text);
				auto& info = MenuItem().text(text).submenu(submenu);
				Check(::SetMenuItemInfoW(m_menu, index, TRUE, &info), "Set menu item failed");
			}

			// Return a sub menu by address
			// Use: auto menu = menu.SubMenuByName("&File,&Recent Files");
			// Returns Menu() if the sub menu isn't found
			template <typename Char> static Menu ByName(HMENU root, Char const* address)
			{
				assert(root != nullptr);
				for (auto addr = address; *addr != 0 && *addr != ',';)
				{
					// Find the next ',' in 'address'
					auto end = addr;
					for (; *end != 0 && *end != ','; ++end) {}

					// Look for the first part of the address in the items of 'root'
					for (int i = 0, iend = ::GetMenuItemCount(root); i != iend; ++i)
					{
						// Get the menu item name and length
						// Check the item name matches the first part of the address
						Char item_name[256] = {};
						auto item_name_len = Win32<Char>::MenuString(root, UINT(i), item_name, _countof(item_name), MF_BYPOSITION);
						if (item_name_len != end - addr || std::char_traits<Char>::compare(addr, item_name, item_name_len) != 0)
							continue;

						// If this is the last part of the address, then return the sub-menu
						// Note, if the menu is not a sub-menu then you'll get 0, turn it into a pop-up menu before here
						auto sub_menu = ::GetSubMenu(root, i);
						if (*end == 0 || sub_menu == 0)
							return Menu(sub_menu, false);

						root = sub_menu;
						addr = end + 1;
						break;
					}
				}
				return Menu();
			}
		};
		#pragma endregion

		#pragma region EventHandler

		// Returns an identifier for uniquely identifying event handlers
		using EventHandlerId = unsigned long long;
		inline EventHandlerId GenerateEventHandlerId()
		{
			static std::atomic_uint s_id = {};
			auto id = s_id.load();
			for (;!s_id.compare_exchange_weak(id, id + 1);) {}
			return static_cast<EventHandlerId>(id) + 1;
		}

		// Place-holder for events that take no arguments. (Makes the templating consistent)
		struct EmptyArgs
		{};

		// Event args used in cancel-able operations
		struct CancelEventArgs :EmptyArgs
		{
			bool m_cancel;
			CancelEventArgs(bool cancel = false)
				:m_cancel(cancel)
			{}
		};

		// Event args used when an index has changed
		struct SelectedIndexEventArgs :EmptyArgs
		{
			int m_index;
			int m_prev_index;

			SelectedIndexEventArgs(int index, int prev_index)
				:m_index(index)
				,m_prev_index(prev_index)
			{}
		};

		// Event args for paint events
		struct PaintEventArgs :EmptyArgs
		{
			enum class EParts
			{
				Background = 1 << 0,
				Foreground = 1 << 1,
				All = Background | Foreground,
				allow_bitops = 0,
			};

			EParts m_parts;    // The parts to be painted
			HWND   m_hwnd;     // The window being painted. Null if 'm_dc' is not the ClientDC for the control
			DC     m_dc;       // The device context to draw on
			Brush  m_bsh_back; // The back colour brush
			bool   m_handled;  // True to prevent any further painting

			PaintEventArgs(HWND hwnd, HDC alternate_hdc, Brush const& bsh_back)
				:m_parts(EParts::All)
				,m_hwnd(!alternate_hdc ? hwnd : nullptr)
				,m_dc(!alternate_hdc ? ::GetDC(hwnd) : alternate_hdc)
				,m_bsh_back(bsh_back)
				,m_handled(false)
			{}
			~PaintEventArgs()
			{
				if (m_hwnd) ::ReleaseDC(m_hwnd, m_dc);
			}

			// Returns the area the needs painting
			// Using 'erase' == true, causes a WM_ERASEBKGND message to be sent.
			// It's probably better to just fill the area in your paint handler instead
			Rect UpdateRect(BOOL erase = false) const
			{
				Rect rect;
				return ::GetUpdateRect(m_hwnd, &rect, erase) != 0 ? rect : Rect();
			}

			// Fill the update rect using the background brush, without validating the region
			void PaintBackground()
			{
				if (m_hwnd == nullptr || m_bsh_back == nullptr)
					return;

				// Fill the client area of 'm_hwnd' with the background colour
				Rect cr; GetClientRect(m_hwnd, &cr);
				auto r = cr.Intersect(UpdateRect());
				if (!r.empty())
					::FillRect(m_dc, &r, m_bsh_back);

				m_parts = SetBits(m_parts, EParts::Background, false);
			}
		};

		// Event args for window sizing events
		struct WindowPosEventArgs :EmptyArgs
		{
			WindowPos* m_wp;     // The new position/size info.
			bool       m_before; // True if this event is before the window pos change, false if after

			WindowPosEventArgs(WindowPos& wp, bool before)
				:m_wp(&wp)
				,m_before(before)
			{
				//// If the WindowPos does not contain size/position information, add it
				//if (AnySet(wp.flags, SWP_NOMOVE|SWP_NOSIZE))
				//{
				//	Rect r;
				//	::GetClientRect(wp.hwnd, &r);
				//	::MapWindowPoints(wp.hwnd, ::GetParent(wp.hwnd), r.points(), 2);
				//	
				//	if (AllSet(wp.flags, SWP_NOMOVE))
				//	{
				//		wp.x = r.left;
				//		wp.y = r.top;
				//	}
				//	if (AllSet(wp.flags, SWP_NOSIZE))
				//	{
				//		wp.cx = r.width();
				//		wp.cy = r.height();
				//	}
				//}
			}

			// True if the event represents a relocation of the window
			bool IsReposition() const
			{
				return !AllSet(m_wp->flags, SWP_NOMOVE);
			}

			// True if the event represents a resize of the window
			bool IsResize() const
			{
				return !AllSet(m_wp->flags, SWP_NOSIZE);
			}

			// True if the window is minimised
			bool Iconic() const
			{
				return ::IsIconic(m_wp->hwnd) != 0;
			}

			// The new position of this window within it's parent (in parent client space). Valid both before and after
			Rect ParentRect() const
			{
				return m_wp->Bounds();
			}

			// The new location in parent client space of the resized window
			Point Location() const
			{
				return Point(m_wp->x, m_wp->y);
			}

			// The new size of the resized window
			Size Size() const
			{
				return gui::Size(m_wp->cx, m_wp->cy);
			}
		};

		// Event args for shown events
		struct VisibleEventArgs :EmptyArgs
		{
			bool m_visible; // True if showing, false if hiding
			VisibleEventArgs(bool shown)
				:m_visible(shown)
			{}
		};

		// Event args for keyboard key events
		struct KeyEventArgs :EmptyArgs
		{
			Control* m_owner; // The control that the key event is for
			UINT m_vk_key;    // The VK_ key that was pressed
			UINT m_repeats;   // Repeat count
			UINT m_flags;     //
			HWND m_hwnd;      // The handle of the window that the key event is for
			bool m_down;      // True if this is a key down event, false if key up
			bool m_handled;   // Set to true to prevent further handling of this key event

			KeyEventArgs(Control* owner, UINT vk_key, bool down, HWND hwnd, UINT repeats, UINT flags)
				:m_owner(owner)
				,m_vk_key(vk_key)
				,m_repeats(repeats)
				,m_flags(flags)
				,m_hwnd(hwnd)
				,m_down(down)
				,m_handled(false)
			{}

			// The location of the mouse (client space) at the key event
			Point point() const;

			// Get the point in absolute pixels
			Point point_px() const;
		};

		// Event args for mouse button events
		struct MouseEventArgs :EmptyArgs
		{
			Control*  m_owner;     // The control that the mouse event is for
			Point     m_point;     // The location of the mouse at the button event relative to (unpadded) client space (in DIP client space)
			EMouseKey m_button;    // The button that triggered the event
			EMouseKey m_key_state; // The state of all mouse buttons and control keys
			bool      m_down;      // True if the button was a down event, false if an up event
			bool      m_handled;   // Set to true to prevent further handling of this key event

			MouseEventArgs(Control* owner, EMouseKey btn, bool down, Point point, EMouseKey key_state)
				:m_owner(owner)
				,m_point(point)
				,m_button(btn)
				,m_key_state(key_state)
				,m_down(down)
				,m_handled(false)
			{}

			// Get the point in absolute pixels
			Point point_px() const;
		};

		// Event args for mouse wheel events
		struct MouseWheelArgs :EmptyArgs
		{
			Control*  m_owner;     // The control that the mouse event is for
			Point     m_point;     // The (unpadded) client space location of the mouse at the time of the event.
			EMouseKey m_key_state; // The state of all mouse buttons and control keys
			short     m_delta;     // The amount the mouse wheel has turned
			bool      m_handled;   // Set to true to prevent further handling of this key event

			MouseWheelArgs(Control* owner, short delta, Point point, EMouseKey key_state)
				:m_owner(owner)
				,m_point(point)
				,m_key_state(key_state)
				,m_delta(delta)
				,m_handled(false)
			{}

			// Get the point in absolute pixels
			Point point_px() const;
		};

		// Event args for timer events
		struct TimerEventArgs :EmptyArgs
		{
			UINT_PTR m_event_id;
			TimerEventArgs(UINT_PTR event_id)
				:m_event_id(event_id)
			{}
		};

		// Event args for dropped files
		struct DropFilesEventArgs :EmptyArgs
		{
			HDROP               m_drop_info; // The windows drop info
			std::vector<string> m_filepaths; // The file paths dropped
			DropFilesEventArgs(HDROP drop_info)
				:m_drop_info(drop_info)
				,m_filepaths()
			{}
		};

		// EventHandler<>
		// Use:
		//   btn.Click += [&](Button&,EmptyArgs const) {...}
		//   btn.Click += std::bind(&MyDlg::HandleBtn, this, _1, _2);
		template <typename Sender, typename Args> struct EventHandler
		{
			// Note: This isn't thread safe
			using Delegate = std::function<void(Sender,Args)>;
			struct Func
			{
				Delegate m_delegate;
				EventHandlerId m_id;
				Func(Delegate delegate, EventHandlerId id)
					:m_delegate(delegate)
					,m_id(id)
				{}
			};
			std::vector<Func> m_handlers;

			EventHandler() :m_handlers() {}
			EventHandler(EventHandler&& rhs) :m_handlers(std::move(rhs.m_handlers)) {}
			EventHandler(EventHandler const&) = delete;
			EventHandler& operator=(EventHandler const&) = delete;

			// 'Raise' the event notifying subscribed observers
			void operator()(Sender& s, Args const& a) const
			{
				for (auto& h : m_handlers)
					h.m_delegate(s,a);
			}

			// Detach all handlers. NOTE: this invalidates all associated Handler's
			void reset()
			{
				m_handlers.clear();
			}

			// Number of attached handlers
			size_t count() const
			{
				return m_handlers.size();
			}

			// Attach/Detach handlers
			EventHandlerId operator += (Delegate func)
			{
				auto handler = Func{func, GenerateEventHandlerId()};
				m_handlers.push_back(handler);
				return handler.m_id;
			}
			EventHandlerId operator = (Delegate func)
			{
				reset();
				return *this += func;
			}
			void operator -= (EventHandlerId handler_id)
			{
				// Note, can't use -= (Delegate function) because std::function<> does not allow operator ==
				auto iter = std::find_if(begin(m_handlers), end(m_handlers), [=](Func const& func){ return func.m_id == handler_id; });
				if (iter != end(m_handlers)) m_handlers.erase(iter);
			}

			// Boolean test for no assigned handlers
			struct bool_tester { int x; }; typedef int bool_tester::* bool_type;
			operator bool_type() const { return !m_handlers.empty() ? &bool_tester::x : static_cast<bool_type>(0); }
		};

		#pragma endregion
		
		#pragma region MessageLoop
		namespace wingui
		{
			// PR_CODE_SYNC_BEGIN(MessageLoop)

			// An interface for types that need to handle messages from the message loop before TranslateMessage is called.
			// Typically these are dialog windows or windows with keyboard accelerators that need to call 'IsDialogMessage'
			// or 'TranslateAccelerator'
			struct IMessageFilter
			{
				virtual ~IMessageFilter() = default;

				// Implementers should return true to halt processing of the message.
				// Typically, if you're just observing messages as they go past, return false.
				// If you're a dialog return the result of IsDialogMessage()
				// If you're a window with accelerators, return the result of TranslateAccelerator()
				virtual bool TranslateMessage(MSG&)
				{
					return false;
				}
			};

			// Message loop that also manages and runs a priority queue of loops 
			struct MessageLoop :IMessageFilter
			{
				// Notes:
				//  - For event driven applications, don't add any loops. Then the message loop will just pump messages as normal.
				//  - Fixed step loops run at exactly the requested rate, accumulating time and catching up if behind.
				//  - Variable step loops run whenever possible, receiving the actual wall-clock elapsed time.
				//  - Fixed step loops have priority over variable step loops when both are due.

				using clock_t = std::chrono::steady_clock;
				using time_point_t = clock_t::time_point;
				using duration_t = clock_t::duration;
				using step_func_t = std::function<void(double)>; // receives elapsed time in seconds

				// The maximum number of fixed-step catch-up iterations before skipping ahead (death spiral protection)
				static constexpr int MaxCatchUpSteps = 4;

				// Compact history of frame rates 
				union FpsHistory
				{
					enum { Length = sizeof(uint64_t) };

					uint64_t u64;
					uint8_t ms_per_frame[sizeof(uint64_t)];
					void Add(double fps)
					{
						u64 <<= 8;
						ms_per_frame[0] = static_cast<uint8_t>(std::clamp(1000.0 / fps, 0.0, 255.0));
					}
					template <std::floating_point S> void Fps(std::span<S, Length> out_fps) const
					{
						// Output fps from newest to oldest
						for (size_t i = 0; i != sizeof(u64); ++i)
							out_fps[i] = ms_per_frame[i] > 0 ? S(1000.0 / ms_per_frame[i]) : S(0.0);
					}
				};

				// Controls for the message loop behaviour
				struct Config
				{
					// The number of window messages to process between calls to StepLoops.
					// Simulation apps would typically set this to 1.
					int msgs_per_loop = 1000;
				};

			private:

				// A loop represents a process that should be run at a given rate
				struct Loop
				{
					step_func_t m_step;       // The function to call to step the loop
					duration_t m_interval;    // The (minimum) time between steps
					time_point_t m_last_time; // The time this loop was last stepped
					time_point_t m_next_due;  // When this loop is next due to be stepped
					FpsHistory m_fps_history; // Ring buffer of recent FPS values
					bool m_variable;          // Variable step rate (true = run as fast as possible, false = fixed step)

					Loop(step_func_t step, duration_t interval, bool variable)
						: m_step(step)
						, m_interval(interval)
						, m_last_time()
						, m_next_due()
						, m_fps_history()
						, m_variable(variable)
					{}
				};

				using LoopCont = std::vector<Loop>;
				using Filters = std::vector<IMessageFilter*>;

				LoopCont m_loop;          // The loops to execute
				Filters m_filters;        // Message filters to process messages before TranslateMessage is called
				time_point_t m_clock0;    // The time when 'Run' was called
				bool m_quit_pending;      // Set when PostQuitMessage has been called, enables aggressive drain

			public:

				// Configurable parameters for the message loop
				Config m_config;

				MessageLoop()
					: m_loop()
					, m_filters()
					, m_clock0()
					, m_config()
					, m_quit_pending(false)
				{
					m_filters.push_back(this);
				}
				virtual ~MessageLoop() = default;

				// Access FPS history for a loop by index
				FpsHistory const& LoopFps(int loop_index) const
				{
					static FpsHistory null = {};
					if (loop_index < 0 || loop_index >= ssize(m_loop)) return null;
					return m_loop[loop_index].m_fps_history;
				}

				// Return the running time since 'Run()' was called (in seconds)
				double Clock() const
				{
					return std::chrono::duration<double>(clock_t::now() - m_clock0).count();
				}

				// Add a loop to be stepped by this message pump.
				// 'fps' is the target frame rate. 'variable' true means run as fast as possible (fps is minimum rate).
				void AddLoop(double fps, bool variable, step_func_t step)
				{
					auto interval = std::chrono::duration_cast<duration_t>(std::chrono::duration<double>(1.0 / fps));
					m_loop.emplace_back(std::move(step), interval, variable);
				}

				// Add/Remove an instance that needs to handle messages before TranslateMessage is called
				void AddMessageFilter(IMessageFilter& filter)
				{
					m_filters.insert(--std::end(m_filters), &filter);
				}
				void RemoveMessageFilter(IMessageFilter& filter)
				{
					m_filters.erase(std::remove(std::begin(m_filters), std::end(m_filters), &filter), std::end(m_filters));
				}

				// Run the thread message pump while maintaining the desired loop rates
				virtual int Run()
				{
					// Initialise clocks
					m_clock0 = clock_t::now();
					auto now = m_clock0;
					for (auto& loop : m_loop)
					{
						loop.m_last_time = now;
						loop.m_next_due = now;
					}

					// Run the message pump loop
					for (;;)
					{
						// Step any pending loops and get the time till the next loop is due
						auto timeout = StepLoops();

						// Pump any queued messages, waiting up to 'timeout' for new ones
						auto exit_code = Pump(timeout);
						if (exit_code)
							return *exit_code;
					}
				}

				// Pump messages. Returns null or an exit code if a WM_QUIT message was pumped
				std::optional<int> Pump(DWORD timeout_ms = 0)
				{
					MSG msg = {};

					// Wait for messages or until timeout (efficient idle, no busy-spin)
					::MsgWaitForMultipleObjects(0, nullptr, FALSE, timeout_ms, QS_ALLPOSTMESSAGE | QS_ALLINPUT | QS_ALLEVENTS);
					for (int msgs_per_loop = m_config.msgs_per_loop; msgs_per_loop-- != 0 && ::PeekMessageW(&msg, 0, 0, 0, PM_REMOVE); )
					{
						if (msg.message == WM_QUIT)
							return static_cast<int>(msg.wParam);

						HandleMessage(msg);
					}

					// WM_QUIT has the lowest priority in the Windows message queue. It is only
					// synthesized by PeekMessage/GetMessage when no other messages are pending.
					// If any source continuously generates messages (timers, paint, posted messages
					// from background threads), WM_QUIT can be starved indefinitely.
					// Check explicitly by draining all remaining messages in a second pass.
					if (m_quit_pending)
					{
						while (::PeekMessageW(&msg, nullptr, 0, 0, PM_REMOVE))
						{
							if (msg.message == WM_QUIT)
								return static_cast<int>(msg.wParam);
							// Discard non-quit messages during shutdown drain
						}
					}
					return std::nullopt;
				}

				// Post WM_QUIT to the thread message queue and enable aggressive message
				// draining so that WM_QUIT (which has the lowest priority) is not starved
				// by continuously generated messages from timers, paint, or background threads.
				void RequestQuit(int exit_code = 0)
				{
					m_quit_pending = true;
					::PostQuitMessage(exit_code);
				}

				// Call 'Step' on all loops that are pending. Returns the time in milliseconds until the next loop is due.
				DWORD StepLoops()
				{
					if (m_loop.empty())
						return INFINITE;

					auto now = clock_t::now();

					// Step fixed-rate loops first (they have priority), then variable-rate loops
					for (int pass = 0; pass != 2; ++pass)
					{
						for (auto& loop : m_loop)
						{
							// Pass 0 = fixed loops, Pass 1 = variable loops
							if (loop.m_variable != (pass == 1))
								continue;

							if (loop.m_variable)
							{
								// Variable step: run whenever due, with the actual elapsed wall-clock time
								if (now < loop.m_next_due)
									continue;

								auto elapsed = std::chrono::duration<double>(now - loop.m_last_time).count();
								loop.m_step(elapsed);
								loop.m_last_time = now;
								loop.m_next_due = now + loop.m_interval;
								loop.m_fps_history.Add(elapsed > 0 ? (1.0 / elapsed) : 0.0f);
							}
							else
							{
								// Fixed step: run at exactly the requested rate, catching up if behind
								auto stepped = false;
								for (int catch_up = 0; catch_up != MaxCatchUpSteps && loop.m_next_due <= now; ++catch_up)
								{
									auto dt = std::chrono::duration<double>(loop.m_interval).count();
									loop.m_step(dt);
									loop.m_next_due += loop.m_interval;
									stepped = true;
								}
								if (stepped)
								{
									auto elapsed = max(now - loop.m_last_time, loop.m_interval);
									loop.m_fps_history.Add(1.0 / std::chrono::duration<double>(elapsed).count());
								}

								// Death spiral protection: if still behind, skip ahead
								if (loop.m_next_due < now)
									loop.m_next_due = now;

								loop.m_last_time = now;
							}
						}
					}

					// Calculate time until the next loop is due
					auto next_due = m_loop[0].m_next_due;
					for (auto const& loop : m_loop)
					{
						if (loop.m_next_due < next_due)
							next_due = loop.m_next_due;
					}

					now = clock_t::now();
					if (next_due <= now)
						return 0;

					auto wait = std::chrono::duration_cast<std::chrono::milliseconds>(next_due - now).count();
					return static_cast<DWORD>(std::max(0LL, wait));
				}

			protected:

				// Pass the message to each filter. The last filter is this message loop which always handles the message.
				void HandleMessage(MSG& msg)
				{
					for (auto filter : m_filters)
						if (filter->TranslateMessage(msg))
							break;
				}

				// The message loop is always the last filter in the chain
				virtual bool TranslateMessage(MSG& msg) override
				{
					::TranslateMessage(&msg);
					::DispatchMessageW(&msg);
					return true;
				}
			};

			// PR_CODE_SYNC_END()
		}

		// gui::Message loop is shared with other code. A different name is used here to prevent name clashes
		using WinGuiMsgLoop = wingui::MessageLoop;
		using IWinGuiMsgFilter = wingui::IMessageFilter;
		#pragma endregion

		#pragma region WndRef
		// A helper for referencing a Control* or an HWND.
		// When a Control* is given, the parent is a Control using this framework (preferred).
		// When an HWND is given, this is the case when parenting to a window not using this framework.
		namespace impl
		{
			template <typename TControl> struct WndRef
			{
			private:
				TControl* m_ctrl; // If 'm_ctrl' != nullptr, then we get hwnd from m_ctrl->m_hwnd
				HWND      m_hwnd; // Only used when 'm_ctrl' == nullptr

				template <typename T> friend struct WndRef;
				WndRef(TControl* ctrl, HWND hwnd)
					:m_ctrl(ctrl)
					,m_hwnd(hwnd)
				{}

			public:

				WndRef() :WndRef(nullptr, nullptr) {}
				WndRef(nullptr_t) :WndRef(nullptr, nullptr) {}
				WndRef(HWND hwnd) :WndRef(FindCtrl(hwnd), hwnd) {}
				WndRef(TControl* ctrl) :WndRef(ctrl, nullptr) {}

				TControl* ctrl() const { return m_ctrl; }
				HWND      hwnd() const { return m_ctrl != nullptr ? HWND(*m_ctrl) : m_hwnd; }
				TControl* operator -> () const { return m_ctrl; }

				operator WndRef<TControl const>() { return WndRef<TControl const>(m_ctrl, m_hwnd); }
				operator TControl*() const        { return ctrl(); }
				operator HWND() const             { return hwnd(); }

				// Deleted because this is ambiguous:
				// could be:
				//    m_hwnd != nullptr
				// or m_ctrl != nullptr
				// or m_ctrl->m_hwnd != nullptr,
				operator bool() const = delete;

				// Attempts to get the Control* for 'hwnd'
				static TControl* FindCtrl(HWND hwnd)
				{
					return hwnd != nullptr ? reinterpret_cast<TControl*>(::SendMessageW(hwnd, WM_GETCTRLPTR, 0, 0)) : nullptr;
				}
			};

			// Equal if 'lhs' and 'rhs' represent the same window
			template <typename TControl> inline bool operator == (impl::WndRef<TControl> lhs, impl::WndRef<TControl> rhs)
			{
				// Careful, if one has a ctrl pointer and the other doesn't, but the ctrl
				// doesn't yet have an hwnd, then they're different windows.
				auto lhs_ptr = lhs.ctrl() - (Control*)0;
				auto rhs_ptr = rhs.ctrl() - (Control*)0;
				return
					(lhs_ptr ^ rhs_ptr) == 0 && // both the same (possibly null) control pointers
					lhs.hwnd() == rhs.hwnd();   // both refer to the same window handle
			}
			template <typename TControl> inline bool operator != (impl::WndRef<TControl> lhs, impl::WndRef<TControl> rhs)
			{
				return !(lhs == rhs);
			}
		}
		using WndRefC = impl::WndRef<Control const>;
		using WndRef  = impl::WndRef<Control>;
		#pragma endregion

		#pragma region DPI Scaling
		struct DpiScale
		{
			// Controls/Forms are laid out assuming 96 DPI.
			// This struct is used to scale positions/sizes to pixels.
			// All operations for a control after scaling are in pixels
			int m_dt_dpi; // The design-time DPI
			int m_rt_dpi; // The run-time DPI

			DpiScale(int dt_dpi, bool from_font = false)
				:m_dt_dpi(dt_dpi)
				,m_rt_dpi(from_font ? DPIFromFont() : DPI())
			{}

			// Get/Set the current DPI
			int CurrentDPI() const
			{
				return m_rt_dpi;
			}
			void CurrentDPI(int dpi)
			{
				m_rt_dpi = dpi;
			}

			// Scale from 96 DPI to the current DPI
			int   X(int   x) const { return int(x * m_rt_dpi / float(m_dt_dpi) + (x >= 0 ? 0.5f : -0.5f)); }
			int   Y(int   y) const { return int(y * m_rt_dpi / float(m_dt_dpi) + (y >= 0 ? 0.5f : -0.5f)); }
			float X(float x) const { return x * m_rt_dpi / float(m_dt_dpi); }
			float Y(float y) const { return y * m_rt_dpi / float(m_dt_dpi); }
			Point Pt(Point pt) const
			{
				return Point(X(static_cast<int>(pt.x)), Y(static_cast<int>(pt.y)));
			}

			// Return the current DPI
			static int DPI()
			{
				// Windows assumes square DPI
				ClientDC dc(nullptr);
				auto x = ::GetDeviceCaps(dc, LOGPIXELSX);
				auto y = ::GetDeviceCaps(dc, LOGPIXELSY);
				return std::max(x, y);
			}

			// Return the estimated DPI by comparing the size of system font to the system font size at 96 DPI
			static int DPIFromFont()
			{
				// Measure the size of the system font at the current DPI.
				// We'll use this to auto-scale all control positions and sizes.
				// Size of the system font at 96 DPI is 6.71153831f, 13.0f
				auto font = (HFONT)::GetStockObject(DEFAULT_GUI_FONT);
				
				ClientDC dc(nullptr);
				SelectObject old(dc, font);
				Size sz; TEXTMETRICW tm;
				Check(::GetTextExtentPointW(dc, L"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz", 52, &sz), "GetTextExtentPoint failed when calculating scaling factor");
				Check(::GetTextMetricsW(dc, &tm), "GetTextMetrics failed when calculating scaling factor");

				// Size of the system font 'MS Shell Dlg' at 96 DPI is 6.0f, 13.0f
				return std::max(
					static_cast<int>((sz.cx*1.9230769e-2f * 96.0f) /  6.0f), //1/52 = 0.01923...
					static_cast<int>((tm.tmHeight         * 96.0f) / 13.0f));
			}
		};
		#pragma endregion

		#pragma region Auto size position
		// Use: e.g. Left|LeftOf|id,
		// Read: left edge of this control, aligned to the left of control with id 'id'
		namespace auto_size_position
		{
			// The mask for auto positioning control bits
			inline constexpr int AutoPosMask = int(0xFF000000);

			// The mask for auto sizing control bits
			inline constexpr int AutoSizeMask = int(0xF0000000);

			// Used as a size value, Fill means expand w,h to match parent, Auto means resize to suit content
			inline constexpr int Dflt = int(CW_USEDEFAULT); // 0x80000000
			inline constexpr int Fill = int(0x90000000);
			inline constexpr int Auto = int(0xA0000000);

			// The mask for the control id
			inline constexpr int IdMask = int(0x0000FFFF);
			static_assert((ID_UNUSED & IdMask) == ID_UNUSED, "");

			// The X,Y coord of the control being positioned. Note: 'Dflt' can be used for position as well
			inline constexpr int Left   = int(0x81000000);
			inline constexpr int Right  = int(0x82000000);
			inline constexpr int Centre = int(0x83000000);

			// The X coord of the reference control to align to
			inline constexpr int LeftOf   = int(0x84000000);
			inline constexpr int RightOf  = int(0x88000000);
			inline constexpr int CentreOf = int(0x8C000000);
			inline constexpr int CentreP  = Centre | CentreOf;

			// True if 'x' contains auto position information (where 'x' is x or y)
			inline constexpr bool IsAutoPos(int x)
			{
				// If the top 4 bits are not '0b1000' then 'X' is just a negative number.
				// Otherwise, the top 8 bits are the auto position bits and the lower 16
				// are the id of the control to position relative to.
				return (x & 0xF0000000) == 0x80000000;
			}

			// True if 'x' contains auto size information (where 'x' is width or height)
			inline constexpr bool IsAutoSize(int x)
			{
				return (x & AutoSizeMask) != 0;
			}

			// Handle auto position/size
			// Adjusts x,y,w,h to be positioned and sized related to the parent or sibling controls.
			// 'measure' is a callback that provides the dimensions of the parent client area or sibling controls in parent client coords.
			// 'measure(0)' means return the parent rect in parent client coordinates;
			// 'measure(-1)' means return the size of the control being positioned using preferred size.
			// 'measure([1,0xffff))' means return the parent space rect of a sibling control with 'id' in the range 1,0xffff.
			// All aligning is done after margins have been added. 'measure' should return bounds that include margins.
			inline void CalcPosSize(int& x, int& y, int& w, int& h, Rect const& margin, std::function<Rect(int)> measure)
			{
				auto parent_area = measure(0);

				// Set the width/height and x/y position
				// 'X' is the x position, 'W' is the width, 'L' is the left margin, 'R' is the right margin
				// 'i' is 0 for the X-axis, 1 for the Y-axis (i.e. Y, height positioning)
				auto auto_size = [=](int& X, int& W, int L, int R, int i)
				{
					auto fill = false;
					
					// Set auto size
					if (IsAutoSize(W))
					{
						if ((W & AutoSizeMask) == Fill)
						{
							// Get the parent control client area (in parent space, including padding)
							W = parent_area.size(i) - (L+R);
							fill = true;
						}
					}

					// Set auto position
					if (IsAutoPos(X))
					{
						// Get the ref point on the parent. Note, order is important here because Centre = Left|Right
						int ref = 0;
						if (AllSet(X, CentreOf))
						{
							// Position relative to the centre of 'b' (including margin)
							auto b = measure(X & IdMask);
							ref = b.centre()[i];
						}
						else if (AllSet(X, LeftOf))
						{
							// Position relative to the left edge of 'b' (including margin)
							auto b = measure(X & IdMask);
							ref = b.topleft()[i];
						}
						else if (AllSet(X, RightOf))
						{
							// Position relative to the right edge of 'b' (including margin)
							auto b = measure(X & IdMask);
							ref = b.bottomright()[i];
						}

						// Position the control relative to 'ref' including margin
						if (AllSet(X, Centre))
						{
							// If 'fill', fill to left/right edges (ignore X)
							if (fill) X = L;
							else X = ref - (W+L+R)/2 + L;
						}
						else if (AllSet(X, Left))
						{
							// If 'fill', fill from X to the right edge
							X = ref + L;
							if (fill) W -= ref;
						}
						else if (AllSet(X, Right))
						{
							// If 'fill', fill to the left edge
							if (fill) { X = L; W = ref - (L+R); }
							else X = ref - (W+L+R) + L;
						}
					}
					else if (X < 0)
					{
						// Position relative to the BR
						if (fill) W += (X+1);
						X = parent_area.bottomright()[i] - (W+L+R) + (X+1) + L;
					}
					else
					{
						// Position relative to the TL
						if (fill) W -= X;
						X = parent_area.topleft()[i] + X + L;
					}
				};

				// If any of the x,y,w,h values are 'Dflt' create a temporary window to get the position that the window manager would choose.
				if (x == Dflt || y == Dflt || w == Dflt || h == Dflt)
				{
					auto hwnd = CreateWindowExW(0, L"STATIC", L"", 0, IsAutoPos(x)?Dflt:x, IsAutoPos(y)?Dflt:y, IsAutoSize(w)?Dflt:w, IsAutoSize(h)?Dflt:h, nullptr, nullptr, nullptr, nullptr);
					auto cleanup = OnScopeExit([=]{ ::DestroyWindow(hwnd); });
					Check(hwnd != nullptr, "Failed to create temporary window");
					Rect rc; ::GetWindowRect(hwnd, &rc);
					if (x == Dflt) x = rc.left;
					if (y == Dflt) y = rc.top;
					if (w == Dflt) w = rc.width();
					if (h == Dflt) h = rc.height();
				}

				// If any of the w,h values are 'Auto', measure their preferred size only once
				if (w == Auto || h == Auto)
				{
					auto sz = measure(-1);
					if (w == Auto) w = sz.width();
					if (h == Auto) h = sz.height();
				}

				// Auto size in each dimension
				auto_size(x, w, -margin.left, margin.right, 0);
				auto_size(y, h, -margin.top, margin.bottom, 1);
			}
		}
		#pragma endregion

		#pragma region Dialog Template
		// A structure for defining a dialog template
		struct DlgTemplate
		{
			enum { DefW = 640, DefH = 480 };
			static DlgTemplate const& Empty()
			{
				static DlgTemplate s_null;
				return s_null;
			}

			#pragma region Auto Size Position
			static auto const AutoPosMask = auto_size_position::AutoPosMask;
			static auto const AutoSizeMask = auto_size_position::AutoSizeMask;
			static auto const Fill = auto_size_position::Fill;

			// The point on this control to position
			static auto const Left = auto_size_position::Left;
			static auto const Right = auto_size_position::Right;
			static auto const Centre = auto_size_position::Centre;
			static auto const Top = Left;
			static auto const Bottom = Right;

			// The point of the referenced control to align to
			static auto const LeftOf = auto_size_position::LeftOf;
			static auto const RightOf = auto_size_position::RightOf;
			static auto const CentreOf = auto_size_position::CentreOf;
			static auto const TopOf = LeftOf;
			static auto const BottomOf = RightOf;

			static auto const CentreP = Centre | CentreOf;
			#pragma endregion

			std::vector<byte> m_mem;
			std::vector<size_t> m_item_base;
			bool m_has_menu; // Flag to indicate the dialog will have a menu. Used for auto size/position

			DlgTemplate()
				:m_mem()
				,m_item_base()
				,m_has_menu()
			{}
			template <typename TParams, typename = typename TParams::this_type>
			DlgTemplate(TParams const& p)
				:m_mem()
				,m_item_base()
				,m_has_menu()
			{
				m_has_menu = p.m_menu != nullptr;  // m_menu just has to be non-null

				// In a standard template for a dialog box, the DLGTEMPLATE structure is always immediately followed
				// by three variable-length arrays that specify the menu, class, and title for the dialog box.
				// When the DS_SETFONT style is specified, these arrays are also followed by a 16-bit value specifying
				// point size and another variable-length array specifying a typeface name. Each array consists of one
				// or more 16-bit elements. The menu, class, title, and font arrays must be aligned on WORD boundaries.

				// Make local copies
				auto x = p.m_x;
				auto y = p.m_y;
				auto w = p.m_w;
				auto h = p.m_h;
				auto style = p.m_style;
				auto style_ex = p.m_style_ex;
				assert("Auto position not supported for dialog templates" && x != CW_USEDEFAULT);
				assert("Auto position not supported for dialog templates" && y != CW_USEDEFAULT);

				// Auto size to the parent
				auto_size_position::CalcPosSize(x, y, w, h, Rect(), [&](int id) -> Rect
				{
					if (id == 0) return p.m_parent.hwnd() != nullptr ? Control::ClientRect(p.m_parent, true) : MinMaxInfo(p.m_dpi).Bounds();
					if (id == -1) throw std::runtime_error("Auto size not supported for dialog templates");
					throw std::runtime_error("DlgTemplate can only be positioned related to the screen or owner window");
				});

				// If 'style' includes DS_SETFONT then windows expects the header to be followed by
				// font data consisting of a 16-bit font size, and Unicode font name string
				if (false)//p.m_font_name[0])
					style |= DS_SETFONT;
				else
					style &= ~DS_SETFONT;

				// Add the header
				DLGTEMPLATE hd = {cast<DWORD>(style), cast<DWORD>(style_ex), cast<WORD>(0), cast<short>(x), cast<short>(y), cast<short>(w), cast<short>(h)};
				append(m_mem, &hd, sizeof(hd));

				// Immediately following the DLGTEMPLATE structure is a menu array that identifies a menu resource for the dialog box.
				// If the first element of this array is 0x0000, the dialog box has no menu and the array has no other elements.
				// If the first element is 0xFFFF, the array has one additional element that specifies the ordinal value of a menu
				// resource in an executable file. If the first element has any other value, the system treats the array as a
				// null-terminated Unicode string that specifies the name of a menu resource in an executable file.
				AddWord(p.m_menu.m_res_id != nullptr ? p.m_menu.id() : 0);

				// Following the menu array is a class array that identifies the window class of the control. If the first element
				// of the array is 0x0000, the system uses the predefined dialog box class for the dialog box and the array has no
				// other elements. If the first element is 0xFFFF, the array has one additional element that specifies the ordinal
				// value of a predefined system window class. If the first element has any other value, the system treats the array
				// as a null-terminated Unicode string that specifies the name of a registered window class.
				AddString(p.wcn());

				// Following the class array is a title array that specifies a null-terminated Unicode string that contains the title
				// of the dialog box. If the first element of this array is 0x0000, the dialog box has no title and the array has no other elements.
				AddString(p.m_text);

				// The 16-bit point size value and the typeface array follow the title array, but only if the style member specifies the
				// DS_SETFONT style. The point size value specifies the point size of the font to use for the text in the dialog box and
				// its controls. The typeface array is a null-terminated Unicode string specifying the name of the typeface for the font.
				// When these values are specified, the system creates a font having the specified size and typeface (if possible) and
				// sends a WM_SETFONT message to the dialog box procedure and the control window procedures as it creates the dialog box and controls.
				if (style & DS_SETFONT)
				{
					//append(m_mem, &p.m_font_size, sizeof(p.m_font_size));
					//AddString(p.m_font_name);
				}

				// Following the DLGTEMPLATE header in a standard dialog box template are one or more DLGITEMTEMPLATE structures that
				// define the dimensions and style of the controls in the dialog box. The 'cdit' member specifies the number of
				// DLGITEMTEMPLATE structures in the template. These DLGITEMTEMPLATE structures must be aligned on *DWORD* boundaries.
			}

			// True if the template contains a dialog description, false if not
			bool valid() const
			{
				return !m_mem.empty();
			}

			// Access to the template header
			DLGTEMPLATE const& hdr() const
			{
				return *reinterpret_cast<DLGTEMPLATE const*>(m_mem.data());
			}
			DLGTEMPLATE& hdr()
			{
				return *reinterpret_cast<DLGTEMPLATE*>(m_mem.data());
			}
			operator DLGTEMPLATE const*() const
			{
				return &hdr();
			}

			// Returns the dialog item by index
			DLGITEMTEMPLATE const& item(size_t idx) const
			{
				if (idx >= m_item_base.size()) throw std::runtime_error("Dialog template item index out of range");
				return *reinterpret_cast<DLGITEMTEMPLATE const*>(&m_mem[m_item_base[idx]]);
			}

			// Add a control to the template
			template <typename TParams, typename = typename TParams::this_type>
			DlgTemplate& Add(TParams const& p, WORD creation_data_size_in_bytes = 0, void* creation_data = nullptr)
			{
				// In a standard template for a dialog box, the DLGITEMTEMPLATE structure is always immediately followed by three
				// variable-length arrays specifying the class, title, and creation data for the control. Each array consists of
				// one or more 16-bit elements.

				// Each DLGITEMTEMPLATE structure in the template must be aligned on a DWORD boundary. The class and title arrays
				// must be aligned on WORD boundaries. The creation data array must be aligned on a WORD boundary.
				auto pad = m_mem.size() & 0x3;
				if (pad)
				{
					byte const padding[4] = {};
					append(m_mem, &padding[0], 4 - pad);
				}

				// Add the dialog item to the header count
				hdr().cdit++;

				// Auto size/position
				auto x = p.m_x;
				auto y = p.m_y;
				auto w = p.m_w;
				auto h = p.m_h;
				auto_size_position::CalcPosSize(x, y, w, h, p.m_margin, [this](int id) -> Rect
				{
					if (id == 0)
					{
						// Get the size of the dialog window client area
						auto& h = hdr();
						auto adj = Rect();
						Check(::AdjustWindowRectEx(&adj, h.style, BOOL(m_has_menu), h.dwExtendedStyle), "AdjustWindowRectEx failed.");
						return Rect(h.x - adj.left, h.y - adj.top, h.x + h.cx - adj.right, h.y + h.cy - adj.bottom);
					}
					else if (id == -1)
					{
						// Get the size of this control based on it's content
						throw std::runtime_error("Auto size not supported for dialog templates");
					}
					else
					{
						// Get the sibling control position in parent space
						for (int i = 0; i != hdr().cdit; ++i)
						{
							auto& itm = this->item(size_t(i));
							if (itm.id != id) continue;

							// This should include the item margin, but it's not available here.
							// This class needs to be changed so that it stores the added items and only
							// generates the memory layout when the template is needed.
							return Rect(itm.x, itm.y, itm.x + itm.cx, itm.y + itm.cy);
						}
						throw std::runtime_error("Sibling control not found");
					}
				});

				// Add a description of the item
				m_item_base.push_back(m_mem.size());
				auto item = DLGITEMTEMPLATE{cast<DWORD>(p.m_style), cast<DWORD>(p.m_style_ex), cast<short>(x), cast<short>(y), cast<short>(w), cast<short>(h), cast<WORD>(p.m_id.value)};
				append(m_mem, &item, sizeof(item));

				// Immediately following each DLGITEMTEMPLATE structure is a class array that specifies the window class of the control.
				// If the first element of this array is any value other than 0xFFFF, the system treats the array as a null-terminated
				// Unicode string that specifies the name of a registered window class. If the first element is 0xFFFF, the array has
				// one additional element that specifies the ordinal value of a predefined system class. The ordinal can be one of the following atom values.
				auto wcn = p.wcn();
				enum class EStdCtrlType :WORD { None = 0, BUTTON = 0x0080, EDIT = 0x0081, STATIC = 0x0082, LISTBOX = 0x0083, SCROLLBAR = 0x0084, COMBOBOX = 0x0085};
				auto ctrl_atom = EStdCtrlType::None;
				if      (wcn == nullptr) {}
				else if (wcscmp(wcn, L"BUTTON"   ) == 0) ctrl_atom = EStdCtrlType::BUTTON;
				else if (wcscmp(wcn, L"EDIT"     ) == 0) ctrl_atom = EStdCtrlType::EDIT;
				else if (wcscmp(wcn, L"STATIC"   ) == 0) ctrl_atom = EStdCtrlType::STATIC;
				else if (wcscmp(wcn, L"LISTBOX"  ) == 0) ctrl_atom = EStdCtrlType::LISTBOX;
				else if (wcscmp(wcn, L"SCROLLBAR") == 0) ctrl_atom = EStdCtrlType::SCROLLBAR;
				else if (wcscmp(wcn, L"COMBOBOX" ) == 0) ctrl_atom = EStdCtrlType::COMBOBOX;
				if (ctrl_atom != EStdCtrlType::None)
					AddWord(WORD(ctrl_atom));
				else
					AddString(wcn);

				// Following the class array is a title array that contains the initial text or resource identifier of the control.
				// If the first element of this array is 0xFFFF, the array has one additional element that specifies an ordinal value
				// of a resource, such as an icon, in an executable file. You can use a resource identifier for controls, such as
				// static icon controls, that load and display an icon or other resource rather than text. If the first element is any
				// value other than 0xFFFF, the system treats the array as a null-terminated Unicode string that specifies the initial text.
				AddString(p.m_text);

				// The creation data array begins at the next WORD boundary after the title array. This creation data can be of any size
				// and format. If the first word of the creation data array is non-zero, it indicates the size, in bytes, of the creation
				// data (including the size word). The control's window procedure must be able to interpret the data. When the system
				// creates the control, it passes a pointer to this data in the lParam parameter of the WM_CREATE message that it sends to the control.
				creation_data_size_in_bytes += (creation_data_size_in_bytes != 0) * sizeof(creation_data_size_in_bytes); // include the size of 'creation_data_size_in_bytes'
				append(m_mem, &creation_data_size_in_bytes, sizeof(creation_data_size_in_bytes));
				if (creation_data_size_in_bytes != 0)
					append(m_mem, creation_data, creation_data_size_in_bytes);

				return *this;
			}

		private:

			// Append a string or null terminator to the memory buffer
			template <typename Char> void AddString(Char const* str)
			{
				if (str == nullptr)
				{
					WORD x = 0;
					append(m_mem, &x, sizeof(x));
				}
				else
				{
					auto s = Widen(str); s.push_back(0);
					append(m_mem, s.data(), s.size() * sizeof(wchar_t));
				}
			}

			// Append a {0xffff, val} pattern to the memory buffer
			void AddWord(WORD val)
			{
				if (val == 0)
				{
					WORD m = 0;
					append(m_mem, &m, sizeof(m));
				}
				else
				{
					WORD m[2] = {0xFFFF, val};
					append(m_mem, &m[0], sizeof(m));
				}
			}
		};
		#pragma endregion

		#pragma region Control ID
		struct CtrlId
		{
			int value;
			CtrlId(int id) :value(id & 0xFFFF) {}
			static CtrlId NextId()
			{
				static int s_id = ID_BEG;
				return CtrlId(++s_id);
			}
		};
		#pragma endregion

		#pragma region CreateParams

		// Notes:
		//  - 'Derived' should be the type at the bottom of the inheritance hierarchy subclassed from 'CtrlParams' (i.e. CRTP).
		//      e.g.
		//         struct CustomParams :CtrlParams<CustomParams> {};
		//  - If you're implementing a parameters type that may be further subclassed, use this pattern:
		//     e.g.
		//         template <typename Derived = void>
		//         struct CustomParams :CtrlParams<not_void_t<Derived, CustomParams<Derived>> {};
		//     This forwards the derived type to the base class, or uses 'CustomParams'
		//     automatically if CustomParams has not been subclassed.

		// Control parameters
		template <typename Derived = void>
		struct CtrlParams
		{
			using this_type = typename not_void_t<Derived, CtrlParams<Derived>>;

			char              m_name[64];       // A name for the control. Mainly for debugging
			HINSTANCE         m_hinst;          // The owning module handle
			wchar_t const*    m_wcn;            // Window class name
			WndClassEx const* m_wci;            // Window class information
			wchar_t const*    m_text;           // Control text
			int               m_x, m_y;         // Negative values for 'x,y' mean relative to the right,bottom of the parent. Remember auto position Left|RightOf, etc..
			int               m_w, m_h;         // Can use Control::Fill, Control::Auto, etc
			CtrlId            m_id;             // The control id, used to match the control to windows created in dialog resources
			WndRef            m_parent;         // The control that contains this control, or form that owns this form
			EAnchor           m_anchor;         // How to move/resize this control when the parent resizes
			EDock             m_dock;           // How to position/size this control within its parent
			DWORD             m_style;          // Window styles
			DWORD             m_style_ex;       // Extra window styles
			ResId<HMENU>      m_menu;           // The resource id of a menu or a menu handle. The control takes ownership if a handle is passed
			ResId<HICON>      m_icon_bg;        // The resource id of an icon or an icon handle. The control takes ownership if a handle is passed
			ResId<HICON>      m_icon_sm;        // The resource id of an icon or an icon handle. The control takes ownership if a handle is passed
			COLORREF          m_colour_fore;    // The foreground colour of the control
			COLORREF          m_colour_back;    // The background colour of the control
			Font const*       m_font;           // The font for the control
			bool              m_client_wh;      // True if width and height parameters are desired client width/height, rather than screen bounds
			bool              m_selectable;     // True if the control gains keyboard input focus when selected
			bool              m_allow_drop;     // True if the control is a drag and drop target
			bool              m_dbl_buffer;     // True if painting is double buffered for this control
			void*             m_init_param;     // The initialisation data to pass through to WM_CREATE
			int               m_dpi;            // The design-time DPI of the control's size and position.
			Rect              m_margin;         // Stored as an addition to the bounding rect (i.e. negative l,t)
			Rect              m_padding;        // Stored as an addition to the bounding rect (i.e. negative r,b)
			MinMaxInfo        m_min_max_info;   // The size limits on the control

			// Parameter copy static function - prevents all derived types needing to overload a virtual clone function
			this_type* (*clone)(this_type const&);

			CtrlParams()
				:m_name()
				,m_hinst(::GetModuleHandleW(nullptr))
				,m_wcn()
				,m_wci()
				,m_text(L"")
				,m_x(0)
				,m_y(0)
				,m_w(50)
				,m_h(50)
				,m_id(CtrlId::NextId())
				,m_parent()
				,m_anchor(EAnchor::None)
				,m_dock(EDock::None)
				,m_style(DefaultControlStyle)
				,m_style_ex(DefaultControlStyleEx)
				,m_menu()
				,m_icon_bg()
				,m_icon_sm()
				,m_colour_fore(CLR_INVALID)
				,m_colour_back(CLR_INVALID)
				,m_font()
				,m_client_wh(false)
				,m_selectable(false)
				,m_allow_drop(false)
				,m_dbl_buffer(false)
				,m_init_param()
				,m_dpi(USER_DEFAULT_SCREEN_DPI)
				,m_margin(-0, -0, 0, 0)
				,m_padding()
				,m_min_max_info(m_dpi)
				,clone(Clone)
			{}
			virtual ~CtrlParams() {}
			template <typename TParams> TParams const& as() const
			{
				using param_type = typename TParams::this_type;
				return *reinterpret_cast<param_type const*>(this);
			}
			template <typename TParams> TParams& as()
			{
				using param_type = typename TParams::this_type;
				return *reinterpret_cast<param_type*>(this);
			}
			static this_type* Clone(this_type const& p)
			{
				return new this_type(p);
			}
			this_type& me()
			{
				return *static_cast<this_type*>(this);
			}

			// Return the window class ATOM
			wchar_t const* atom() const
			{
				return m_wci != nullptr ? m_wci->IntAtom() : m_wcn;
			}

			// Return the window class name from 'm_wci' or 'm_wcn', 'm_wci' preferred.
			wchar_t const* wcn() const
			{
				return m_wci != nullptr ? m_wci->lpszClassName : m_wcn;
			}

			// True for top-level controls, i.e. forms, pop-up windows, or overlapped windows
			bool top_level() const
			{
				return !AllSet(m_style, WS_CHILD);
			}

			// Fluent initialisation
			this_type& name(char const* n)
			{
				StrCopy(m_name, n);
				return me();
			}
			this_type& hinst(HINSTANCE i)
			{
				m_hinst = i;
				return me();
			}
			this_type& wndclass(wchar_t const* wcn)
			{
				m_wcn = wcn;
				return me();
			}
			this_type& wndclass(WndClassEx const& wci)
			{
				m_wci = &wci;
				return me();
			}
			this_type& wndclass(nullptr_t)
			{
				m_wci = nullptr;
				m_wcn = nullptr;
				return me();
			}
			this_type& text(wchar_t const* t)
			{
				m_text = t ? t : L"";
				return me();
			}
			this_type& dpi(int dpi)
			{
				m_dpi = dpi;
				return me();
			}
			this_type& dlu() // Dialog units
			{
				// Note: If you're transferring a resource dialog to code, this scaling factor assumes
				// the dialog resource uses 'MS Shell Dlg (8)'. If not, then the scaling factor will be different
				m_client_wh = true;
				return dpi(static_cast<int>(std::max(4 * 96 / 6.0f, 8 * 96 / 13.0f)));
			}
			this_type& xy(int x, int y)
			{
				m_x = x;
				m_y = y;
				return me();
			}
			this_type& w(int w_)
			{
				return wh(w_, m_h, m_client_wh);
			}
			this_type& h(int h_)
			{
				return wh(m_w, h_, m_client_wh);
			}
			this_type& wh(int w, int h)
			{
				return wh(w, h, m_client_wh);
			}
			this_type& wh(int w, int h, bool client)
			{
				m_w = w;
				m_h = h;
				m_client_wh = client;
				return me();
			}
			this_type& id(CtrlId id_)
			{
				m_id = id_;
				return me();
			}
			this_type& parent(WndRef p)
			{
				m_parent = p;
				return me();
			}
			this_type& anchor(EAnchor a)
			{
				m_anchor = a;
				return me();
			}
			this_type& dock(EDock d)
			{
				m_dock = d;
				return me();
			}
			this_type& style(char op, DWORD s)
			{
				switch (op)
				{
				case '=': m_style = s; break;
				case '+': m_style |= s; break;
				case '-': m_style &= ~s; break;
				}
				return me();
			}
			this_type& style_ex(char op, DWORD s)
			{
				switch (op)
				{
				case '=': m_style_ex = s; break;
				case '+': m_style_ex |= s; break;
				case '-': m_style_ex &= ~s; break;
				}
				return me();
			}
			this_type& menu(Menu::ItemList const& items)
			{
				m_menu.m_handle = Menu(Menu::EKind::Strip, items, false);
				return me();
			}
			this_type& menu(ResId<HMENU> m)
			{
				m_menu = m;
				return me();
			}
			this_type& icon(ResId<HICON> i)
			{
				return icon_sm(i).icon_bg(i);
			}
			this_type& icon_bg(ResId<HICON> i)
			{
				m_icon_bg = i;
				return me();
			}
			this_type& icon_sm(ResId<HICON> i)
			{
				m_icon_sm = i;
				return me();
			}
			this_type& fr_col(COLORREF c)
			{
				assert((c & 0xFF000000) == 0 && "Don't use alpha");
				m_colour_fore = c;
				return me();
			}
			this_type& bk_col(COLORREF c)
			{
				assert((c & 0xFF000000) == 0 && "Don't use alpha");
				m_colour_back = c;
				return me();
			}
			this_type& font(Font const& f)
			{
				m_font = &f;
				return me();
			}
			this_type& selectable(bool on = true)
			{
				m_selectable = on;
				return me();
			}
			this_type& allow_drop(bool on = true)
			{
				m_allow_drop = on;
				return me();
			}
			this_type& border(bool on = true)
			{
				return style(on ? '+' : '-', WS_BORDER);
			}
			this_type& visible(bool yes = true)
			{
				return style(yes ? '+' : '-', WS_VISIBLE);
			}
			this_type& dbl_buffer(bool yes = true)
			{
				m_dbl_buffer = yes;
				return me();
			}
			this_type& init_param(void* ip)
			{
				m_init_param = ip;
				return me();
			}
			this_type& margin(int m)
			{
				return margin(m, m, m, m);
			}
			this_type& margin(int lr, int tb)
			{
				return margin(lr, tb, lr, tb);
			}
			this_type& margin(int l, int t, int r, int b)
			{
				m_margin = Rect(-l, -t, r, b);
				return me();
			}
			this_type& padding(int p)
			{
				return padding(p, p, p, p);
			}
			this_type& padding(int lr, int tb)
			{
				return padding(lr, tb, lr, tb);
			}
			this_type& padding(int l, int t, int r, int b)
			{
				m_padding = Rect(l, t, -r, -b);
				return me();
			}
			this_type& size_min(int w, int h)
			{
				m_min_max_info.m_mask |= MinMaxInfo::EMask::MinTrackSize;
				m_min_max_info.ptMinTrackSize.x = w;
				m_min_max_info.ptMinTrackSize.y = h;
				return me();
			}
			this_type& size_max(int w, int h)
			{
				m_min_max_info.m_mask |= MinMaxInfo::EMask::MaxTrackSize;
				m_min_max_info.ptMaxTrackSize.x = w;
				m_min_max_info.ptMaxTrackSize.y = h;
				return me();
			}
			this_type& resizeable(bool yes = true)
			{
				return style(yes ? '+' : '-', WS_THICKFRAME);
			}
		};

		// Form parameters
		template <typename Derived = void>
		struct FormParams :CtrlParams<not_void_t<Derived, FormParams<Derived>>>
		{
			using this_type = typename FormParams::this_type;
			using base_type = typename CtrlParams<not_void_t<Derived, FormParams<Derived>>>;

			EStartPosition     m_start_pos;     // Where the form should
			DlgTemplate const* m_templ;         // A dialog template for creating dialogs without a resource
			WinGuiMsgLoop*     m_msg_loop;      // The thread message pump
			ResId<HACCEL>      m_accel;         // The resource id of accelerators or an accelerator handle. The control takes ownership if a handle is passed
			bool               m_main_wnd;      // Main application window, closing it exits the application
			bool               m_dlg_behaviour; // True if this form has dialog-like keyboard shortcuts
			bool               m_hide_on_close; // True if closing the form only makes it hidden
			bool               m_pin_window;    // True if this form moves with it's parent
			bool               m_dpi_aware;     // True if per-monitor DPI awareness should be enabled

			FormParams()
				:m_start_pos    (EStartPosition::Default)
				,m_templ        ()
				,m_msg_loop     ()
				,m_accel        ()
				,m_main_wnd     (false)
				,m_dlg_behaviour(false)
				,m_hide_on_close(false)
				,m_pin_window   (false)
				,m_dpi_aware    (true)
			{
				// Form's with Id's are normally dialogs in a resource
				this->m_id.value = ID_UNUSED;

				this->m_style = DefaultFormStyle;
				this->m_style_ex = DefaultFormStyleEx;

				// Default start position and size
				this->m_x = CW_USEDEFAULT;
				this->m_y = CW_USEDEFAULT;
				this->m_w = CW_USEDEFAULT;
				this->m_h = CW_USEDEFAULT;

				this->m_text = L"Form";
				this->m_main_wnd = true;
				this->m_padding = Rect(8,8,-8,-8);
			}
			this_type& dlg()
			{
				// Default dialog parameters
				return main_wnd(false).wh(640, 480).style('=', DefaultDialogStyle).style_ex('=', DefaultDialogStyleEx).dlg_behaviour();
			}
			this_type& parent(WndRef p)
			{
				m_main_wnd &= p.hwnd() == nullptr;
				return base_type::parent(p);
			}
			this_type& xy(int x, int y)
			{
				m_start_pos = EStartPosition::Manual;
				return base_type::xy(x, y);
			}
			this_type& title(wchar_t const* t)
			{
				return this->text(t).style('+', WS_CAPTION);
			}
			this_type& start_pos(EStartPosition pos)
			{
				m_start_pos = pos;
				return this->me();
			}
			this_type& templ(DlgTemplate const& t)
			{
				m_templ = t.valid() ? &t : nullptr;
				return this->me();
			}
			this_type& msg_loop(WinGuiMsgLoop* ml)
			{
				m_msg_loop = ml;
				return this->me();
			}
			this_type& accel(ResId<HACCEL> a)
			{
				m_accel = a;
				return this->me();
			}
			this_type& main_wnd(bool mw = true)
			{
				m_main_wnd = mw;
				return this->me();
			}
			this_type& tool_window(bool on = true)
			{
				return base_type::style(on ? '+' : '-', WS_EX_TOOLWINDOW);
			}
			this_type& mdi_child(bool mdi = true)
			{
				return base_type::style(mdi ? '+' : '-', WS_CHILD);
			}
			this_type& dlg_behaviour(bool on = true)
			{
				m_dlg_behaviour = on;
				return this->me();
			}
			this_type& hide_on_close(bool h = true)
			{
				m_hide_on_close = h;
				return this->me();
			}
			this_type& pin_window(bool p = true)
			{
				m_pin_window = p;
				this->m_anchor = p ? EAnchor::TopLeft : EAnchor::None;
				return this->me();
			}
			this_type& dpi_aware(bool on = true)
			{
				m_dpi_aware = on;
				return this->me();
			}
		};

		#pragma endregion

		#pragma region Control
		// Base class for all windows/controls
		struct Control
		{
			// Notes:
			//  - All controls should be designed for 96x96 DPI, controls auto scale to the system DPI (no per-monitor support yet)
			//  - All controls use the DEFAULT_GUI_FONT, this ensures auto scaling works properly.

			// These are here to import the types within Control
			using EAnchor        = gui::EAnchor;
			using EDock          = gui::EDock;
			using EDialogResult  = gui::EDialogResult;
			using EStartPosition = gui::EStartPosition;
			using WndRef         = gui::WndRef;
			using Controls       = std::vector<Control*>;

			template <typename Lhs, typename Rhs>
			using not_void_t = gui::not_void_t<Lhs, Rhs>;
			template <typename Derived = void>
			using Params = CtrlParams<Derived>;
			template <typename P>
			using Ptr = std::unique_ptr<P>;

			decltype(std::placeholders::_1) const _1{};
			decltype(std::placeholders::_2) const _2{};
			decltype(std::placeholders::_3) const _3{};
			decltype(std::placeholders::_4) const _4{};

			static constexpr int ID_UNUSED = gui::ID_UNUSED;

			#pragma region Auto Size Position
			static auto const AutoPosMask  = auto_size_position::AutoPosMask;
			static auto const AutoSizeMask = auto_size_position::AutoSizeMask;
			static auto const Fill         = auto_size_position::Fill;
			static auto const Auto         = auto_size_position::Auto;
			static auto const Dflt         = auto_size_position::Dflt;

			// The point on this control to position
			static auto const Left   = auto_size_position::Left;
			static auto const Right  = auto_size_position::Right;
			static auto const Centre = auto_size_position::Centre;
			static auto const Top    = Left;
			static auto const Bottom = Right;

			// The point of the referenced control to align to
			static auto const LeftOf   = auto_size_position::LeftOf;
			static auto const RightOf  = auto_size_position::RightOf;
			static auto const CentreOf = auto_size_position::CentreOf;
			static auto const TopOf    = LeftOf;
			static auto const BottomOf = RightOf;

			static auto const CentreP = Centre | CentreOf;
			static auto const IdMask = auto_size_position::IdMask;

			inline bool IsAutoPos (int x) const { return auto_size_position::IsAutoPos(x); }
			inline bool IsAutoSize(int w) const { return auto_size_position::IsAutoSize(w); }
			#pragma endregion

		protected:
			using BtnDownMap = std::unordered_map<EMouseKey, LONG>;

			Ptr<Params<>>   m_cp;              // Initial creation parameters for the control
			HWND            m_hwnd;            // Window handle for the control
			WndRef          m_parent;          // The parent that contains this control, or owner form
			Controls        m_child;           // The controls nested with this control
			DpiScale        m_metrics;         // Used to scale this control based on the current DPI
			Menu            m_menu;            // Associated menu
			Image           m_icon_bg;         // Associated icon
			Image           m_icon_sm;         // Associated icon
			Brush           m_brush_fore;      // Foreground colour brush
			Brush           m_brush_back;      // Background colour brush
			Rect            m_pos_offset;      // Distances from this control to the edges of the parent client area
			bool            m_pos_ofs_suspend; // Disables the saving of the position offset when the control is moved
			BtnDownMap      m_down_at;         // Button down time stamp
			bool            m_handle_only;     // True if this object does not own 'm_hwnd'
			HBITMAP         m_dbl_buffer;      // Non-null if the control is double buffered
			WndClassEx      m_wci;             // Window class info
			StdCallThunk    m_thunk;           // WndProc thunk, turns a '__stdcall' into a '__thiscall'
			WNDPROC         m_oldproc;         // The window class default wndproc function
			std::thread::id m_thread_id;       // The thread that this control was created on

		public:

			enum { DefW = 50, DefH = 50 };
			Control* this_; // Use to prevent the 'this' used in base member initializer list' warning

			// Create the control
			Control()
				:Control(Params<>())
			{}
			template <typename TParams, typename = typename TParams::this_type>
			explicit Control(TParams const& p)
				:m_cp()
				,m_hwnd()
				,m_parent()
				,m_child()
				,m_metrics(p.m_dpi)
				,m_menu()
				,m_icon_bg()
				,m_icon_sm()
				,m_brush_fore()
				,m_brush_back()
				,m_pos_offset()
				,m_pos_ofs_suspend()
				,m_down_at()
				,m_handle_only()
				,m_dbl_buffer()
				,m_wci()
				,m_thunk()
				,m_oldproc()
				,m_thread_id(std::this_thread::get_id())
			{
				// Don't call create from the constructor, CreateHandle should be called after the
				// top level form is fully constructed. The reason for this is that derived control
				// types are not yet fully constructed. CreateHandle can be called at any point which
				// will cause child controls to be created as well.
				this_ = this;
				m_thunk.Init(DWORD_PTR(StaticWndProc), this);

				// Save the creation parameters
				SaveParams(p.template as<Params<>>());

				// Record the parent
				Parent(p.m_parent);
			}

			// Allow construction from existing handle
			Control(HWND hwnd)
				:Control(Params<>()
					.id(::GetDlgCtrlID(hwnd))
					.style('=', cast<DWORD>(::GetWindowLongPtrW(hwnd, GWL_STYLE)))
					.style_ex('=', cast<DWORD>(::GetWindowLongPtrW(m_hwnd, GWL_EXSTYLE)))
					.anchor(EAnchor::None))
			{
				m_handle_only = true;
				m_hwnd = hwnd;
			}
			virtual ~Control()
			{
				if (!m_handle_only)
				{
					// Clean up the double buffer
					if (m_dbl_buffer != nullptr)
						::DeleteObject(m_dbl_buffer);

					// Orphan child controls
					for (; !m_child.empty(); )
						m_child.front()->Parent(nullptr);

					// Detach from our parent
					Parent(nullptr);

					// Destroy the window
					if (::IsWindow(m_hwnd))
						::DestroyWindow(m_hwnd);
				}
				assert((m_hwnd = (HWND)0xdddddddddddddddd) != 0);// mark as destructed
			}

			Control(Control&&) = delete;
			Control(Control const&) = delete;

			// Implicit conversion to HWND
			operator HWND() const
			{
				return m_hwnd;
			}

			// Control ID
			int id() const
			{
				return m_cp->m_id.value;
			}

			// The parameters used to create this control (but updated to the current state)
			template <typename TParams = Params<>> TParams const& cp() const
			{
				return m_cp->as<TParams>();
			}
			template <typename TParams = Params<>> TParams& cp()
			{
				return m_cp->as<TParams>();
			}

			// Create this control and any child controls that have not yet been created.
			// If the control has a parent, then creation starts with the top level parent
			// and cascades down through all child controls. This method is idempotent, you
			// can call it multiple times as more child controls get added.
			Control& CreateHandle()
			{
				if (m_hwnd == nullptr)
				{
					// If this control isn't yet created, create from the highest ancestor
					if (!cp().top_level() && m_parent.ctrl() != nullptr)
					{
						m_parent->CreateHandle();
					}
					else
					{
						// Create this control, then ensure all children exist
						Create(cp());
						if (m_hwnd != nullptr)
							CreateHandle();
					}
				}
				else
				{
					// Ensure all child controls exist
					for (auto c : m_child)
					{
						if (c->m_hwnd == nullptr)
							c->Create(c->cp());
					}
				}
				return *this;
			}

			// Create the hwnd for this control.
			virtual void Create(Params<> const& p_)
			{
				assert("Window handle already exists" && m_hwnd == nullptr);
				assert("No window class given. Called p.wndclass(RegisterWndClass<>()) before create" && p_.atom() != nullptr);
				assert("Child controls can only be created after the parent has been created" && (p_.top_level() || (p_.m_parent.hwnd() != nullptr && ::IsWindow(p_.m_parent.hwnd()))));

				// Save creation properties, we need to set these, even tho they're
				// set in the constructor, because 'p' might be newer than what was
				// used when the control was constructed.
				// Don't do (p.m_id & IdMask). Controls created by windows use the window
				// handle as the id. IdMask is only needed to AutoPosSize which the caller
				// controls.
				SaveParams(p_);

				// Update members based on the creation parameters
				auto& p = cp();
				{
					auto sz_ico_bg = ::GetSystemMetrics(SM_CXICON);
					auto sz_ico_sm = ::GetSystemMetrics(SM_CXSMICON);
					m_brush_fore =
						p.m_colour_fore != CLR_INVALID ? Brush(p.m_colour_fore) :
						Brush();
					m_brush_back =
						p.m_colour_back != CLR_INVALID ? Brush(p.m_colour_back) :
						WndBackground() != nullptr ? Brush(WndBackground()) :
						Brush();
					m_wci =
						p.m_wci ? *p.m_wci :
						p.m_wcn ? WndClassEx(p.m_wcn, p.m_hinst) :
						WndClassEx();
					m_menu =
						p.m_menu.m_handle != nullptr ? Menu(p.m_menu.m_handle, true) :
						p.m_menu.m_res_id != nullptr ? Menu(::LoadMenuW(p.m_hinst, p.m_menu.m_res_id)) :
						Menu();
					m_icon_bg =
						p.m_icon_bg.m_handle != nullptr ? Image(p.m_icon_bg.m_handle, Image::EType::Icon, false) :
						p.m_icon_bg.m_res_id != nullptr ? Image::Load(p.m_hinst, p.m_icon_bg.m_res_id, Image::EType::Icon, Image::EFit::Zoom, sz_ico_bg, sz_ico_bg) :
						Image();
					m_icon_sm =
						p.m_icon_sm.m_handle != nullptr ? Image(p.m_icon_sm.m_handle, Image::EType::Icon, false) :
						p.m_icon_sm.m_res_id != nullptr ? Image::Load(p.m_hinst, p.m_icon_sm.m_res_id, Image::EType::Icon, Image::EFit::Zoom, sz_ico_sm, sz_ico_sm) :
						Image();
				}

				// Set w,h based on docking to the parent
				switch (p.m_dock)
				{
					case EDock::None: break;
					case EDock::Fill:   p.m_w = Fill; p.m_h = Fill; break;
					case EDock::Top:    p.m_w = Fill; break;
					case EDock::Bottom: p.m_w = Fill; break;
					case EDock::Left:   p.m_h = Fill; break;
					case EDock::Right:  p.m_h = Fill; break;
					default: throw std::runtime_error("Unknown dock style");
				}

				// Scale the control position/size for the current DPI (preserving the auto pos size bits)
				// Note: If this control is a pop-up or overlapped window (i.e. not a child), then x,y,w,h should be in screen coords.
				auto x = auto_size_position::IsAutoPos (p.m_x) ? p.m_x : m_metrics.X(p.m_x);
				auto y = auto_size_position::IsAutoPos (p.m_y) ? p.m_y : m_metrics.Y(p.m_y);
				auto w = auto_size_position::IsAutoSize(p.m_w) ? p.m_w : m_metrics.X(p.m_w);
				auto h = auto_size_position::IsAutoSize(p.m_h) ? p.m_h : m_metrics.Y(p.m_h);

				// Handle auto position/size
				AutoSizePosition(x, y, w, h, p.m_parent);

				// Adjust width/height if client space size was requested
				if (p.m_client_wh)
				{
					auto r = Rect(0, 0, w, h);
					Check(::AdjustWindowRectEx(&r, p.m_style, p.m_menu != nullptr, p.m_style_ex), "AdjustWindowRectEx failed.");
					w = r.width();
					h = r.height();
				}

				// If the start location is manual, and this is a top level window but owned by
				// another window, convert the (x,y) position to screen space. (i.e. assume its given as parent relative)
				if (p.top_level() && p.m_parent.hwnd() != nullptr && ::IsWindow(p.m_parent))
				{
					auto sr = ScreenRect(p.m_parent);
					x += sr.left;
					y += sr.top;
				}

				// Determine the value to pass as the HMENU parameter in CreateWindowEx.
				// For pop-up and overlapped windows this should be a valid menu handle or null
				// Otherwise, it should be the id of the control being created
				auto menu = p.top_level() ? static_cast<HMENU>(m_menu) : (HMENU() + p.m_id.value);

				// Break just before a control/form is created
				//if (strcmp(p.m_name, "ldr-measure-ui") == 0) _CrtDbgBreak();

				// CreateWindowEx failure reasons:
				//  invalid menu handle - if the window style is overlapped or pop-up, then 'menu' must be null
				//     or a valid menu handle otherwise it is the id of the control being created.
				InitParam init(this, p.m_init_param);
				auto hwnd = ::CreateWindowExW(p.m_style_ex, p.atom(), p.m_text, p.m_style, x, y, w, h, p.m_parent, menu, p.m_hinst, &init);
				Check(hwnd != nullptr, FmtS("CreateWindowEx failed for window class '%S', instance '%s'.", p.wcn(), p.m_name));

				// If we're creating a control whose window class we don't control (i.e. a system/third party control),
				// then Attach won't have been called because the InitialWndProc function was not called (indicated by m_hwnd == nullptr).
				// In this case, we want to subclass the window and install our wndproc. OnCreate won't have been called either.
				if (m_hwnd == nullptr)
				{
					Attach(hwnd);
					CreateStruct cs(p.m_style_ex, p.atom(), p.m_text, p.m_style, x, y, w, h, p.m_parent, menu, p.m_hinst, &init);
					SendMsg<int>(WM_CREATE, TRUE, &cs);
				}

				// Set the parent after the window is created so that the anchor position is recorded
				Parent(p_.m_parent);
			}

			// SendMessage helper functions
			template <typename TRet, typename WP = WPARAM, typename LP = LPARAM>
			TRet SendMsg(UINT msg, WP wparam = 0, LP lparam = 0) const
			{
				assert(m_hwnd != nullptr);
				return gui::SendMsg<TRet,WP,LP>::Send(m_hwnd, msg, wparam, lparam);
			}

			#pragma region Accessors

			// Get/Set the parent of this control.
			// If the control is a top-level control (i.e. not WS_CHILD), then the parent is the owning window
			virtual WndRef Parent() const
			{
				return m_parent;
			}
			virtual void Parent(WndRef parent)
			{
				// Change the window that contains this control
				if (m_parent != parent)
				{
					if (m_parent.ctrl() != nullptr)
					{
						// Remove from existing parent. Do this for all WS_CHILD and owned windows
						// because it may have changed just prior to the parent being changed.
						auto& c = m_parent->m_child;
						c.erase(std::remove(c.begin(), c.end(), this), c.end());
					}

					// Check we're not parenting to ourself or a child
					#ifndef NDEBUG
					if (parent.ctrl() != nullptr)
					{
						std::vector<Control*> stack;
						for (stack.push_back(this); !stack.empty();)
						{
							auto x = stack.back(); stack.pop_back();
							assert("Cannot parent to a child" && parent.ctrl() != x);
							for (auto c : x->m_child) stack.push_back(c);
						}
					}
					#endif

					// Try to get the Control* from 'parent' if possible
					m_parent = parent.ctrl() != nullptr ? parent : WndRef(parent.hwnd());
					cp().m_parent = m_parent;

					if (m_parent.ctrl() != nullptr)
					{
						// Add to the children of 'm_parent' (for WS_CHILD and owned windows)
						m_parent->m_child.push_back(this);
					}
				}

				// If this window is alive...
				if (::IsWindow(m_hwnd))
				{
					// Update the parent from windows' point of view for WS_CHILD windows
					if (::GetParent(m_hwnd) != m_parent.hwnd() && !cp().top_level())
					{
						Check(::SetParent(m_hwnd, m_parent.hwnd()) != nullptr, "SetParent failed");

						// Update the UI state based on the new parent
						auto hwnd = m_parent.hwnd() ? m_parent.hwnd() : m_hwnd;
						auto uis = SendMessageW(hwnd, WM_QUERYUISTATE, 0, 0);
						SendMessageW(hwnd, WM_CHANGEUISTATE, WPARAM(MakeWord(uis, UIS_INITIALIZE)), 0);
					}

					// Update the anchor position offset relative to the new parent
					if (m_parent.hwnd() != nullptr && !cp().top_level())
					{
						// Record the position of the control within the parent
						RecordPosOffset();

						// Resize to the parent (only moves the control if Dock/Anchors are used since we just set the offset)
						ResizeToParent();
					}
				}
			}

			// Get the number of child controls
			size_t ChildCount() const
			{
				return m_child.size();
			}

			// Get a child control
			WndRef Child(int i) const
			{
				return m_child[i];
			}

			// Get/Set the index of this control within it's parent
			int Index() const
			{
				assert("Control is not parented" && m_parent.ctrl() != nullptr);
				assert("Control's parent does not contain a reference to this control" && std::find(std::begin(m_parent->m_child), std::end(m_parent->m_child), this) != std::end(m_parent->m_child));
				return cast<int>(std::find(std::begin(m_parent->m_child), std::end(m_parent->m_child), this) - std::begin(m_parent->m_child));
			}
			void Index(int idx)
			{
				assert("Control is not parented" && m_parent.ctrl() != nullptr);
				assert("Control's parent does not contain a reference to this control" && std::find(std::begin(m_parent->m_child), std::end(m_parent->m_child), this) != std::end(m_parent->m_child));
				m_parent->m_child.erase(std::find(std::begin(m_parent->m_child), std::end(m_parent->m_child), this));
				m_parent->m_child.insert(std::begin(m_parent->m_child) + idx, this);
			}

			// Get the collection of child controls
			Controls const& Children() const
			{
				return m_child;
			}

			// Get the top level control. This is typically the window containing this control
			Control const* TopLevelControl() const
			{
				auto p = this;
				for (;!p->cp().top_level() && p->m_parent.ctrl() != nullptr; p = p->m_parent) {}
				return p;
			}
			Control* TopLevelControl()
			{
				auto p = this;
				for (;!p->cp().top_level() && p->m_parent.ctrl() != nullptr; p = p->m_parent) {}
				return p;
			}

			// Get the top level control as a form.
			template <typename = void> Form const* TopLevelForm() const
			{
				auto top = TopLevelControl();
				return top && top->cp().top_level() ? static_cast<Form const*>(top) : nullptr;
			}
			template <typename = void> Form* TopLevelForm()
			{
				auto top = TopLevelControl();
				return top && top->cp().top_level() ? static_cast<Form*>(top) : nullptr;
			}

			// Get/Set the window style
			DWORD Style() const
			{
				assert(::IsWindow(m_hwnd));
				return cast<DWORD>(::GetWindowLongPtrW(m_hwnd, GWL_STYLE));
			}
			Control& Style(char op, DWORD style)
			{
				CreateHandle();
				auto old_style = Style();
				switch (op)
				{
				default: assert(!"Unknown set style operation"); break;
				case '=': break;
				case '+': style = old_style |  style; break;
				case '-': style = old_style & ~style; break;
				case '^': style = old_style ^  style; break;
				}
				::SetWindowLongPtrW(m_hwnd, GWL_STYLE, cast<LONG_PTR>(style));
				cp().m_style = style;
				return *this; // return this for method chaining
			}

			// Get/Set the extended window style
			DWORD StyleEx() const
			{
				assert(::IsWindow(m_hwnd));
				return cast<DWORD>(::GetWindowLongPtrW(m_hwnd, GWL_EXSTYLE));
			}
			Control& StyleEx(char op, DWORD style)
			{
				CreateHandle();
				auto old_style = StyleEx();
				switch (op)
				{
				default: assert(!"Unknown set style ex operation"); break;
				case '=': break;
				case '+': style = old_style |  style; break;
				case '-': style = old_style & ~style; break;
				case '^': style = old_style ^  style; break;
				}
				::SetWindowLongPtrW(m_hwnd, GWL_EXSTYLE, cast<LONG_PTR>(style));
				cp().m_style_ex = style;
				return *this; // return this for method chaining
			}

			// Get/Set the text in the combo
			string Text() const
			{
				assert(::IsWindow(m_hwnd));
				string s;
				s.resize(static_cast<size_t>(::GetWindowTextLengthW(m_hwnd)) + 1, 0);
				if (!s.empty()) s.resize(::GetWindowTextW(m_hwnd, &s[0], int(s.size())));
				return s;
			}
			void Text(wchar_t const* text)
			{
				CreateHandle();
				::SetWindowTextW(m_hwnd, text);
			}
			void Text(char const* text)
			{
				CreateHandle();
				::SetWindowTextA(m_hwnd, text);
			}
			void Text(std::string const& text)
			{
				Text(text.c_str());
			}
			void Text(std::wstring const& text)
			{
				Text(text.c_str());
			}

			// Enable/Disable the control
			bool Enabled() const
			{
				assert(::IsWindow(m_hwnd));
				return ::IsWindowEnabled(m_hwnd) != 0;
			}
			void Enabled(bool enabled)
			{
				CreateHandle();
				::EnableWindow(m_hwnd, enabled);
			}

			// Get/Set visibility of this control
			bool Visible() const
			{
				return ::IsWindow(m_hwnd) && AllSet(Style(), WS_VISIBLE);
			}
			void Visible(bool vis)
			{
				vis ? cp().m_style |= WS_VISIBLE : cp().m_style &= WS_VISIBLE;
				if (::IsWindow(m_hwnd))
					::ShowWindow(m_hwnd, vis ? SW_SHOW : SW_HIDE);
			}

			// Returns true if the window is minimised
			bool Minimised() const
			{
				return ::IsIconic(m_hwnd) != 0;
			}

			// Get/Set the anchor mode for the window
			// Note: Dock() overrides this if not == EDock::None
			EAnchor Anchor() const
			{
				return cp().m_anchor;
			}
			void Anchor(EAnchor anchor)
			{
				cp().m_anchor = anchor;
			}

			// Get/Set the dock style for the control
			EDock Dock() const
			{
				return cp().m_dock;
			}
			void Dock(EDock dock)
			{
				cp().m_dock = dock;
				ResizeToParent(false);
				Invalidate();
			}

			// Get/set the padding for the control
			Rect Padding() const
			{
				return cp().m_padding;
			}
			void Padding(Rect padding)
			{
				cp().m_padding = padding;
				ResizeToParent(false);
				Invalidate();
			}

			// Get/set the margin for the control
			Rect Margin() const
			{
				return cp().m_margin;
			}
			void Margin(Rect margin)
			{
				cp().m_margin = margin;
				ResizeToParent(false);
				Invalidate();
			}

			// Get/Set Drag/Drop allowed
			bool AllowDrop() const
			{
				return (StyleEx() & WS_EX_ACCEPTFILES) != 0;
			}
			void AllowDrop(bool allow)
			{
				::DragAcceptFiles(*this, allow);
			}

			// Set focus to this control, returning the handle of the previous window with focus
			HWND Focus()
			{
				assert(::IsWindow(m_hwnd));
				return ::SetFocus(m_hwnd);
			}

			// Get/Set the font for the control
			HFONT Font() const
			{
				assert(::IsWindow(m_hwnd));
				return HFONT(::SendMessageW(m_hwnd, WM_GETFONT, 0, 0));
			}
			void Font(HFONT font)
			{
				assert(::IsWindow(m_hwnd));
				::SendMessageW(m_hwnd, WM_SETFONT, WPARAM(font), TRUE);
			}
			TextMetrics FontInfo() const
			{
				assert(::IsWindow(m_hwnd));
				ClientDC dc(m_hwnd);
				TextMetrics tm;
				Check(::GetTextMetricsW(dc, &tm), "GetTextMetrics failed");
				return tm;
			}

			// Invalidate the control for redraw
			virtual void Invalidate(bool erase = false, Rect* rect = nullptr, bool include_children = false)
			{
				assert(::IsWindow(m_hwnd));
				Check(::InvalidateRect(m_hwnd, rect, erase), "InvalidateRect failed");
				if (include_children)
				{
					for (auto c : m_child)
						c->Invalidate(erase, rect, include_children);
				}
			}

			// Validate a rectangular area of the control
			virtual void Validate(Rect const* rect = nullptr)
			{
				assert(::IsWindow(m_hwnd));
				Check(::ValidateRect(m_hwnd, rect), "ValidateRect failed");
			}

			// Get/Set the control's background colour
			COLORREF BackColor() const
			{
				return cp().m_colour_back;
			}
			void BackColor(COLORREF col)
			{
				m_brush_back = col != CLR_INVALID ? std::move(Brush(col)) : Brush();
				cp().m_colour_back = col;
				Invalidate();
			}

			// Get/Set the control's foreground colour
			COLORREF ForeColor() const
			{
				return cp().m_colour_fore;
			}
			void ForeColor(COLORREF col)
			{
				m_brush_fore = col != CLR_INVALID ? std::move(Brush(col)) : Brush();
				cp().m_colour_fore = col;
				Invalidate();
			}

			// Return the position of this control in parent client space
			Point loc() const
			{
				return ParentRect().topleft();
			}
			Size size() const
			{
				auto r = ParentRect();
				return Size(r.width(), r.height());
			}
			int width() const
			{
				return ParentRect().width();
			}
			int height() const
			{
				return ParentRect().height();
			}

			// Set the position of this control in parent client space
			void loc(Point xy, bool repaint = true)
			{
				PositionWindow(xy.x, xy.y, 0, 0, EWindowPos::NoSize|(repaint?EWindowPos::None:EWindowPos::NoRedraw));
			}
			void size(Size sz, bool repaint = true)
			{
				PositionWindow(0, 0, sz.cx, sz.cy, EWindowPos::NoMove|(repaint?EWindowPos::None:EWindowPos::NoRedraw));
			}
			void width(int w, bool repaint = true)
			{
				PositionWindow(0, 0, w, height(), EWindowPos::NoMove|(repaint?EWindowPos::None:EWindowPos::NoRedraw));
			}
			void height(int h, bool repaint = true)
			{
				PositionWindow(0, 0, width(), h, EWindowPos::NoMove|(repaint?EWindowPos::None:EWindowPos::NoRedraw));
			}

			// The DPI scaling for this control
			DpiScale Metrics() const
			{
				return m_metrics;
			}

			// Return the ideal size for this control. Includes padding, excludes margin.
			// This method can be called before the HWND is created.
			virtual Size PreferredSize() const
			{
				auto sz = ChildBounds(true);
				auto& padding = cp().m_padding;
				return Size(
					sz.width() + padding.left - padding.right,
					sz.height() + padding.top  - padding.bottom);
			}

			// If 'grow' is true, returns 'rect' increased by the non-client areas of the window.
			// If false, then a rect reduced by the non-client areas is returned.
			// Remember, ClientRect is [inclusive,inclusive] (if 'rect' is the client rect)
			Rect AdjRect(Rect const& rect = Rect(), bool grow = true) const
			{
				Rect r;
				if (grow)
				{
					r = rect;
					Check(::AdjustWindowRectEx(&r, DWORD(Style()), BOOL(::GetMenu(m_hwnd) != nullptr), DWORD(StyleEx())), "AdjustWindowRectEx failed.");
				}
				else
				{
					r = Rect();
					Check(::AdjustWindowRectEx(&r, DWORD(Style()), BOOL(::GetMenu(m_hwnd) != nullptr), DWORD(StyleEx())), "AdjustWindowRectEx failed.");
					r = rect.Adjust(-r.left, -r.top, -r.right, -r.bottom);
				}
				return r;
			}

			// Return 'rect' with areas removed that correspond to docked child controls (up to but excluding 'end_child')
			Rect ExcludeDockedChildren(Rect rect, int end_child = -1) const
			{
				int idx = -1;
				for (auto& child : m_child)
				{
					if (++idx == end_child) break;
					if (!child->m_hwnd || !child->Visible()) continue;
					if (child->cp().m_dock == EDock::None) continue;
					auto child_rect = child->ParentRect().Adjust(child->cp().m_margin);
					switch (child->cp().m_dock)
					{
					case EDock::Fill:   return Rect();
					case EDock::Left:   rect.left   += child_rect.width(); break;
					case EDock::Right:  rect.right  -= child_rect.width(); break;
					case EDock::Top:    rect.top    += child_rect.height(); break;
					case EDock::Bottom: rect.bottom -= child_rect.height(); break;
					default: throw std::runtime_error("Unknown dock style");
					}
				}
				return rect;
			}

			// Return the bounding rectangle (in client space) that the child controls occupy
			Rect ChildBounds(bool visible_only) const
			{
				auto bbox = m_child.empty() ? Rect() : Rect::invalid();
				for (auto& child : m_child)
				{
					Rect rc;
					if (::IsWindow(*child))
					{
						// Children that have been created but aren't visible don't contribute
						if (visible_only && !child->Visible()) continue;
						rc = child->ParentRect().Adjust(child->cp().m_margin);
					}
					else
					{
						// Determine the expect window size from the creation parameters
						if (visible_only && !AllSet(child->cp().m_style, WS_VISIBLE)) continue;
						int x = child->cp().m_x;
						int y = child->cp().m_y;
						int w = child->cp().m_w;
						int h = child->cp().m_h;
						AutoSizePosition(x, y, w, h, this);
						rc = Rect(x,y,x+w,y+h);
					}
					Rect::Grow(bbox, rc);
				}
				return bbox;
			}

			// Get the client rect [TL,BR) for the window in this controls client space.
			// Note: Menus are part of the non-client area, you don't need to offset the client rect for the menu.
			// ClientRect also includes padding by default. Use 'padded' = false for the unpadded client area.
			Rect ClientRect(bool padded) const
			{
				// Call the protected virtual implementation
				return ClientRectCore(padded);
			}
			static Rect ClientRect(HWND hwnd, bool padded)
			{
				assert(::IsWindow(hwnd));

				Rect r;
				Check(::GetClientRect(hwnd, &r), "GetClientRect failed.");
				
				// If 'hwnd' is a 'Control', adjust for it's padding
				if (padded)
				{
					auto wr = WndRefC(hwnd);
					if (wr.ctrl() != nullptr)
					{
						r = r.Adjust(wr->cp().m_padding);
						if (r.right < r.left) r.right = r.left;
						if (r.bottom < r.top) r.bottom = r.top;
					}
				}
				return r;
			}

			// Get/Set the control bounds [TL,BR) in screen space
			Rect ScreenRect() const
			{
				return ScreenRect(m_hwnd);
			}
			static Rect ScreenRect(HWND hwnd)
			{
				assert(::IsWindow(hwnd));
				Rect r;
				Check(::GetWindowRect(hwnd, &r), "GetWindowRect failed.");
				return r;
			}
			void ScreenRect(Rect r, bool repaint = true, HWND prev = nullptr, EWindowPos flags = EWindowPos::NoZorder)
			{
				assert(::IsWindow(m_hwnd));
				if (!repaint) flags = flags | EWindowPos::NoRedraw;

				// SetWindowPos takes client space coordinates
				if (Style() & WS_CHILD)
				{
					auto hwndparent = ::GetParent(m_hwnd);
					::MapWindowPoints(nullptr, hwndparent, r.points(), 2);
				}

				// Use prev = ::GetWindow(m_hwnd, GW_HWNDPREV) for the current z-order
				Check(::SetWindowPos(m_hwnd, prev, r.left, r.top, r.width(), r.height(), (UINT)flags), "SetWindowPos failed");
				RecordPosOffset();
				Invalidate();
			}

			// Get/Set the bounds [TL,BR) of this control within it's parent client space.
			// Only applies to WS_CHILD windows, owned windows are still positioned relative to the screen
			// Does not include margins.
			Rect ParentRect() const
			{
				assert(::IsWindow(m_hwnd));

				// If the control has no parent, then the screen is the parent
				auto phwnd = ::GetParent(m_hwnd);
				if (phwnd == nullptr)
					return ScreenRect();

				// Return the bounds of this control relative to 'parent'
				auto rect = ScreenRect(); // Not using ClientRect() because we don't want this control's padding included
				::MapWindowPoints(nullptr, phwnd, rect.points(), 2);
				return rect;
			}
			void ParentRect(Rect r, bool repaint = true, HWND prev = nullptr, EWindowPos flags = EWindowPos::NoZorder)
			{
				assert(::IsWindow(m_hwnd));
				if (!repaint) flags = flags | EWindowPos::NoRedraw;

				// Doesn't seem to be needed
				//// Invalidate the previous and new rect on the parent
				//auto hwndparent = ::GetParent(m_hwnd);
				//if (hwndparent != nullptr)
				//{
				//	auto pr = ParentRect();
				//	::InvalidateRect(hwndparent, &pr, FALSE);
				//	::InvalidateRect(hwndparent, &r, FALSE);
				//}

				// SetWindowPos takes client space coordinates
				// Use prev = ::GetWindow(m_hwnd, GW_HWNDPREV) for the current z-order
				// Note: this does not call Invalidate internally (tested using GetUpdateRect)
				Check(::SetWindowPos(m_hwnd, prev, r.left, r.top, r.width(), r.height(), (UINT)flags), "SetWindowPos failed");
				RecordPosOffset();
				Invalidate();
			}

			// Get/Set the position of this control within the parent's client space
			Point ParentPos() const
			{
				return ParentRect().topleft();
			}
			void ParentPos(int x, int y, bool repaint = false)
			{
				auto r = ParentRect();
				ParentRect(r.Shifted(x - r.left, y - r.top), repaint);
			}

			// Convert a screen space point to client window space
			Point PointToClient(Point pt) const
			{
				Check(::ScreenToClient(m_hwnd, &pt), "ScreenToClient failed");
				return pt;
			}

			// Convert a client window space point to screen space
			Point PointToScreen(Point pt) const
			{
				Check(::ClientToScreen(m_hwnd, &pt), "ClientToScreen failed");
				return pt;
			}

			// Convert a screen space rectangle to client window space
			Rect RectToClient(Rect rect) const
			{
				return Rect(PointToClient(rect.topleft()), rect.size());
			}

			// Convert a client window space rectangle to screen space
			Rect RectToScreen(Rect rect) const
			{
				return Rect(PointToScreen(rect.topleft()), rect.size());
			}

			// Set redraw mode on or off
			void Redraw(bool redraw)
			{
				assert(::IsWindow(m_hwnd));
				::SendMessageW(m_hwnd, WM_SETREDRAW, WPARAM(redraw ? TRUE : FALSE), 0);
			}

			// Centre this control within another control or the desktop
			void CenterWindow(HWND centre_hwnd = nullptr)
			{
				assert(::IsWindow(m_hwnd));
				assert("'centre_hwnd' is the window to centre relative to. It shouldn't be this window" && m_hwnd != centre_hwnd);

				// Determine the owning window to centre against
				auto style = Style();
				if (centre_hwnd == nullptr)
					centre_hwnd = (style & WS_CHILD) ? ::GetParent(m_hwnd) : ::GetWindow(m_hwnd, GW_OWNER);

				Rect area, centre;

				// Get the coordinates of the window relative to 'centre_hwnd'
				if (!(style & WS_CHILD))
				{
					// Don't centre against invisible or minimized windows
					if (centre_hwnd != nullptr)
					{
						auto parent_state = ::GetWindowLong(centre_hwnd, GWL_STYLE);
						if (!(parent_state & WS_VISIBLE) || (parent_state & WS_MINIMIZE))
							centre_hwnd = nullptr;
					}

					// Centre within screen coordinates
					HMONITOR monitor = ::MonitorFromWindow(centre_hwnd ? centre_hwnd : m_hwnd, MONITOR_DEFAULTTONEAREST);
					Check(monitor != nullptr, "Failed to determine the monitor containing the centre on window");

					MonitorInfo minfo;
					Check(::GetMonitorInfoW(monitor, &minfo), "Failed to get info on monitor containing centre on window");

					area = minfo.rcWork;
					centre = centre_hwnd ? ::GetWindowRect(centre_hwnd, &centre),centre : area;
				}
				else
				{
					// centre within parent client coordinates
					auto p = ::GetParent(m_hwnd);

					assert(::IsWindow(p));
					::GetClientRect(p, &area);

					assert(::IsWindow(centre_hwnd));
					::GetClientRect(centre_hwnd, &centre);

					::MapWindowPoints(centre_hwnd, p, (POINT*)&centre, 2);
				}

				auto r = ScreenRect();

				// Find this control's upper left based on centre
				auto l = (centre.left + centre.right - r.width()) / 2;
				auto t = (centre.top + centre.bottom - r.height()) / 2;

				// if the dialog is outside the screen, move it inside
				if (l + r.width() > area.right)   l = area.right - r.width();
				if (l < area.left)                l = area.left;
				if (t + r.height() > area.bottom) t = area.bottom - r.height();
				if (t < area.top)                 t = area.top;

				// Map screen coordinates to child coordinates
				Check(::SetWindowPos(m_hwnd, ::GetWindow(m_hwnd, GW_HWNDPREV), l, t, -1, -1, SWP_NOSIZE | SWP_NOZORDER | SWP_NOACTIVATE), "Failed to centre window");
			}

			// Position this window relative to it's parent
			// AutoSizePosition values can be used, e.g. Left|LeftOf|<sibbling_ctrl_id>
			// Use SWP_ flags to ignore position or size changes
			void PositionWindow(int x, int y, int w, int h, EWindowPos flags = EWindowPos::None)
			{
				if (AllSet(flags, EWindowPos::NoMove))
				{
					auto r = ParentRect();
					x = r.left;
					y = r.top;
				}
				if (AllSet(flags, EWindowPos::NoSize))
				{
					auto r = ParentRect();
					w = r.width();
					h = r.height();
				}
				AutoSizePosition( x, y, w, h, m_parent);
				ParentRect(Rect(x, y, x + w, y + h), false, nullptr, flags | EWindowPos::NoZorder | EWindowPos::NoActivate);
			}
			void PositionWindow(int x, int y)
			{
				PositionWindow(x, y, 0, 0, EWindowPos::NoSize);
			}

			// Get/Set whether this window is above other windows
			bool TopMost() const
			{
				return AllSet(StyleEx(), WS_EX_TOPMOST);
			}
			void TopMost(bool yes)
			{
				::SetWindowPos(m_hwnd, yes ? HWND_TOPMOST : HWND_NOTOPMOST, 0, 0, 0, 0, SWP_NOMOVE|SWP_NOSIZE);
			}

			// Return the mouse location at the time of the last message
			Point MousePosition() const
			{
				auto pos = ::GetMessagePos();
				return Point(GetXLParam(pos), GetYLParam(pos));
			}

			// Return the key state at the time of the last message
			EModifierKey ModifierKeys() const
			{
				auto state = EModifierKey::None;
				if (::GetKeyState(VK_LSHIFT  ) & 0x8000) state = state | EModifierKey::LShift;
				if (::GetKeyState(VK_RSHIFT  ) & 0x8000) state = state | EModifierKey::RShift;
				if (::GetKeyState(VK_LCONTROL) & 0x8000) state = state | EModifierKey::LCtrl;
				if (::GetKeyState(VK_RCONTROL) & 0x8000) state = state | EModifierKey::RCtrl;
				if (::GetKeyState(VK_LMENU   ) & 0x8000) state = state | EModifierKey::LAlt;
				if (::GetKeyState(VK_RMENU   ) & 0x8000) state = state | EModifierKey::RAlt;
				return state;
			}
			bool KeyState(int vk_key) const
			{
				return (::GetKeyState(vk_key) & 0x8000) != 0;
			}

			#pragma endregion

			#pragma region Events

			// Paint event - note: all erase background events are ignored
			EventHandler<Control&, PaintEventArgs&> Paint;

			// Window position changing or changed
			EventHandler<Control&, WindowPosEventArgs const&> WindowPosChange;

			// Window shown or hidden
			EventHandler<Control&, VisibleEventArgs const&> VisibilityChanged;

			// Key down/up
			EventHandler<Control&, KeyEventArgs&> KeyPreview;
			EventHandler<Control&, KeyEventArgs&> Key;

			// Mouse button down/up
			EventHandler<Control&, MouseEventArgs&> MouseButton;

			// Mouse button single click
			EventHandler<Control&, MouseEventArgs&> MouseClick;

			// Mouse move
			EventHandler<Control&, MouseEventArgs&> MouseMove;

			// Mouse wheel events
			EventHandler<Control&, MouseWheelArgs&> MouseWheel;

			// Timer events
			EventHandler<Control&, TimerEventArgs const&> Timer;

			// Dropped files
			EventHandler<Control&, DropFilesEventArgs const&> DropFiles;

			#pragma endregion

			#pragma region Handlers

			// Called after the window handle is created. Note: this is *not* called during
			// WM_CREATE because controls are often created during construction which means
			// overrides of OnCreate() in derived types would never called.
			virtual void OnCreate(CreateStruct const&)
			{
				// Recursively create child controls.
				for (auto& c : m_child)
				{
					if (c->m_hwnd == nullptr)
						c->Create(c->cp());
					else
						c->Parent(this);
				}

				// Initialise with a default true type font.
				Font(cp().m_font ? static_cast<HFONT>(*cp().m_font) : DefaultGuiFont());
				m_metrics.CurrentDPI(static_cast<int>(::GetDpiForWindow(m_hwnd)));
						
				// Enable drag and drop
				AllowDrop(cp().m_allow_drop);
			}

			// Called when this control is about to be destroyed
			virtual void OnDestroy()
			{
				// Enable drag and drop
				AllowDrop(false);
			}

			// Handle window size changing starting or stopping
			virtual void OnWindowPosChange(WindowPosEventArgs const& args)
			{
				WindowPosChange(*this, args);
			}

			// Handle window shown or hidden
			virtual void OnVisibilityChanged(VisibleEventArgs const& args)
			{
				VisibilityChanged(*this, args);
			}

			// Handle the Paint event. Return true, to prevent anything else handling the event
			virtual void OnPaint(PaintEventArgs& args)
			{
				Paint(*this, args);
			}

			// Called on parent controls (from root down) when a key down/up event is received
			virtual void OnKeyPreview(KeyEventArgs& args)
			{
				if (m_parent.ctrl() != nullptr)
				{
					m_parent->OnKeyPreview(args);
					if (args.m_handled) return;
				}
				KeyPreview(*this, args);
			}

			// Handle keyboard key down/up events.
			virtual void OnKey(KeyEventArgs& args)
			{
				Key(*this, args);
			}

			// Handle mouse button down/up events.
			virtual void OnMouseButton(MouseEventArgs& args)
			{
				MouseButton(*this, args);
			}

			// Handle mouse button single click events
			// Single clicks occur between down and up events
			virtual void OnMouseClick(MouseEventArgs& args)
			{
				MouseClick(*this, args);
			}

			// Handle mouse move events
			virtual void OnMouseMove(MouseEventArgs& args)
			{
				MouseMove(*this, args);
			}

			// Handle mouse wheel events
			virtual void OnMouseWheel(MouseWheelArgs& args)
			{
				MouseWheel(*this, args);
			}

			// Handle timer events
			virtual void OnTimer(TimerEventArgs const& args)
			{
				Timer(*this, args);
			}

			// Handle files dropped onto the control
			virtual void OnDropFiles(DropFilesEventArgs const& args)
			{
				DropFiles(*this, args);
			}

			#pragma endregion

		protected:

			// This method is the window procedure for this control. 'ProcessWindowMessage' is
			// used to process messages sent to the parent window that contains this control.
			// WndProc is called by windows, Forms forward messages to their child controls using 'ProcessWindowMessage'
			virtual LRESULT WndProc(UINT message, WPARAM wparam, LPARAM lparam)
			{
				//WndProcDebug(m_hwnd, message, wparam, lparam, FmtS("CtrlWPMsg: %s",cp().m_name));
				switch (message)
				{
					case WM_GETCTRLPTR:
					{
						return LRESULT(this);
					}
					case WM_CREATE:
					{
						OnCreate(*rcast<CreateStruct const*>(lparam));
						if (cp().m_dbl_buffer) EnsureDoubleBuffer();
						if (wparam) return S_OK; // 'wparam != 0' indicates 'WM_CREATE' is being called manually so don't pass to DefWndProc
						break;
					}
					case WM_DESTROY:
					{
						OnDestroy();
						Detach();
						break;
					}
					case WM_ACTIVATE:
					{
						UpdateWindow(*this);
						break;
					}
					case WM_ERASEBKGND:
					{
						// Notes:
						//  - WM_ERASEBKGND means clear the client area background. When we're double
						//    buffering, we need to fill with the background colour.
						//  - WM_ERASEBKGND occurs before WM_PAINT

						// Prevent erase background. It results in flickering.
						return 1;
					}
					case WM_PAINT:
					{
						// Notes:
						//  - Only create a gui::PaintStruct if you intend to do all the painting yourself,
						//    otherwise DefWndProc will do it (i.e. most controls are drawn by DefWndProc).
						//  - The update rect in the paint args is the area needing painting.
						//  - Typical behaviour is to create a PaintStruct, however you can choose to not do
						//    this and use this sequence instead:
						//      ::GetUpdateRect(*this, &r, FALSE); <- (TRUE sends the WM_ERASEBKGND if needed)
						//      Draw(); <- do your drawing
						//      Validate(&r); <- tell windows the update rect has been updated,
						//  - Non-client window parts are drawn in DefWndProc
						//  - The WS_BORDER style decreases the client rectangle by 1 on all sides.
						//  - The Border is part of the non-client area of the control.

						// Create arguments for the paint event
						auto back_brush = m_brush_back != nullptr ? m_brush_back : Brush(m_wci.hbrBackground);
						PaintEventArgs args(m_hwnd, HDC(wparam), back_brush);

						// If double buffering, do all drawing in a memory DC
						if (cp().m_dbl_buffer)
						{
							EnsureDoubleBuffer();

							// Create a memory DC the same size as the client area using the double buffer bitmap
							ClientDC dc(m_hwnd);
							auto client_rect = ClientRect(false);
							MemDC mem(dc, client_rect, m_dbl_buffer);

							// Replace the DC in 'args' (and make sure it doesn't get released)
							assert(args.m_dc.m_owned == false);
							args.m_dc.m_hdc = mem.m_hdc;
							//args.m_hwnd = nullptr; //why null?

							// Allow sub-classes to handle painting
							OnPaint(args);
							if (args.m_handled)
								return S_OK;

							// If the sub-class hasn't totally handled the paint, paint the remaining parts.
							// Manually draw the background, since we're swallowing WM_ERASEBKGND.
							if (AllSet(args.m_parts, PaintEventArgs::EParts::Background))
								args.PaintBackground();

							// Render the window into the memory DC
							if (AllSet(args.m_parts, PaintEventArgs::EParts::Foreground))
								DefWndProc(WM_PRINTCLIENT, (WPARAM)mem.m_hdc, LPARAM(PRF_CHECKVISIBLE | PRF_NONCLIENT | PRF_CLIENT));

							// Copy to the screen buffer
							Check(::BitBlt(dc, 0, 0, client_rect.width(), client_rect.height(), mem.m_hdc, 0, 0, SRCCOPY), "Bitblt failed");

							// Mark the update rect as updated
							Validate();

							// Painting is done, don't fall through to DefWndProc
							return S_OK;
						}
						// Otherwise, draw in the given DC
						else
						{
							// Allow sub-classes to handle painting
							OnPaint(args);
							if (args.m_handled)
							{
								// Mark the update rect as updated
								Validate();
								return S_OK;
							}

							// If the sub-class hasn't totally handled the paint, paint the remaining parts.
							// Manually draw the background, since we're swallowing WM_ERASEBKGND.
							if (AllSet(args.m_parts, PaintEventArgs::EParts::Background))
								args.PaintBackground();
						}
						break;
					}
					case WM_DPICHANGED:
					{
						auto dpi = static_cast<int>(::GetDpiForWindow(m_hwnd));
						m_metrics.CurrentDPI(dpi);
						[[fallthrough]];
					}
					case WM_WINDOWPOSCHANGING:
					{
						[[fallthrough]];
					}
					case WM_WINDOWPOSCHANGED:
					{
						// WM_WINDOWPOSCHANGING/ED is sent to a form/control when it is resized using SetWindowPos (or similar).
						// It is not recursive, i.e. if a window is resized, then only that window receives the WM_WINDOWPOSCHANGED
						// message.
						// The logical way to handle WM_WINDOWPOSCHANGED is for the receiving window to resize all of it's children
						// recursively so that the entire control tree is resized. However, resizing a child results in WM_WINDOWPOSCHANGED
						// being sent to that child window. Logically then, you'd make the WM_WINDOWPOSCHANGED messages drive the
						// recursion. However, in between the WM_WINDOWPOSCHANGED messages, WM_PAINT messages are sent which has the
						// effect of making the UI look rubbery as parts resize amidst the redraws. What is needed is for the initial
						// WM_WINDOWPOSCHANGED receiving window to resize all of its descendants, and for the WM_WINDOWPOSCHANGED messages,
						// that get sent as a result of resizing the children, to be ignored. Ignoring messages is hard tho, so at the
						// moment, rubbery it is...
						//
						// WinGui handles this as follows:
						// - If a Control receives WM_WINDOWPOSCHANGED => Handled by Control::WndProc
						// - If a Form receives WM_WINDOWPOSCHANGED => Handled by Form::ProcessWindowMessage
						// - The Form::ProcessWindowMessage handler for WM_WINDOWPOSCHANGED does some special case work for initial form
						//   position and window pinning, then allows the message to fall through to the Control::WndProc
						// - Control::WndProc handles WM_WINDOWPOSCHANGED for any form or control.
						//   Using a local stack, the control resizes its descendants in breadth-first order. Note: ClientRect cannot be
						//   used during this because the window does not report the new size until after the WM_WINDOWPOSCHANGED for that
						//   window has been handled.
						//
						// Other Notes:
						// - WM_WINDOWPOSCHANGED supersedes WM_SHOWWINDOW, WM_SIZE, WM_MOVE. These older messages
						//   are only sent if WM_WINDOWPOSCHANGED is handled by DefWndProc.
						// - Don't resize child controls during 'WM_WINDOWPOSCHANGING' because the 'm_pos_offset'
						//   gets recorded relative to the old client area.

						auto& wp = *rcast<WindowPos*>(lparam);
						auto before = message == WM_WINDOWPOSCHANGING;
						if (before)
						{
							// Notify of position changing
							OnWindowPosChange(WindowPosEventArgs(wp, before));
						}
						else
						{
							auto is_resize = !AllSet(wp.flags, int(EWindowPos::NoSize));
							auto is_move = !AllSet(wp.flags, int(EWindowPos::NoMove));

							// "NoRedraw" means, the *system* isn't going to redraw the window so you have to.
							auto manual_redraw = AllSet(wp.flags, int(EWindowPos::NoRedraw));

							// Record the new position/size in the parameters
							if (is_move)
							{
								cp().m_x = wp.x;
								cp().m_y = wp.y;
							}
							if (is_resize)
							{
								cp().m_w = wp.cx;
								cp().m_h = wp.cy;
							}

							// Resize all descendants
							if (is_resize || is_move)
							{
								auto client = ClientRect(true);
								auto screen = ScreenRect();
								for (auto c : m_child)
								{
									if (IsForm(c) && !IsPinnedForm(c))
										continue;

									// If 'child' is a child control and this is a resize, reposition within this control's client area
									if (IsChild(c) && is_resize)
										c->ResizeToParent(client);

									// If 'child' is a pinned form and we have moved, move the child as well
									if (IsForm(c) && is_move)
										c->ResizeToParent(screen);
								}
							}

							// If we're double buffering, resize the back buffer
							if (is_resize && cp().m_dbl_buffer)
								EnsureDoubleBuffer();

							// Notify of position changed
							OnWindowPosChange(WindowPosEventArgs(wp, before));

							// Notify of visibility changed
							if (AllSet(wp.flags, int(EWindowPos::ShowWindow)))
								OnVisibilityChanged(VisibleEventArgs(true));
							if (AllSet(wp.flags, int(EWindowPos::HideWindow)))
								OnVisibilityChanged(VisibleEventArgs(false));

							// Invalidate the window
							if (manual_redraw)
							{
								// Todo: we need to track what needs drawing and invalidate those areas
								Invalidate(true, nullptr, true);
							}
						}
						break;
					}
					case WM_GETMINMAXINFO:
					{
						auto& a = *reinterpret_cast<MinMaxInfo*>(lparam);
						auto& b = cp().m_min_max_info;
						if ((b.m_mask & MinMaxInfo::EMask::MaxSize) != 0) a.ptMaxSize = b.ptMaxSize; else b.ptMaxSize = a.ptMaxSize;
						if ((b.m_mask & MinMaxInfo::EMask::MaxPosition) != 0) a.ptMaxPosition = b.ptMaxPosition; else b.ptMaxPosition = a.ptMaxPosition;
						if ((b.m_mask & MinMaxInfo::EMask::MinTrackSize) != 0) a.ptMinTrackSize = b.ptMinTrackSize; else b.ptMinTrackSize = a.ptMinTrackSize;
						if ((b.m_mask & MinMaxInfo::EMask::MaxTrackSize) != 0) a.ptMaxTrackSize = b.ptMaxTrackSize; else b.ptMaxTrackSize = a.ptMaxTrackSize;
						break;
					}
					case WM_KEYDOWN:
					case WM_KEYUP:
					{
						// Key down/up is sent to the window that has keyboard focus. Typically this is a text
						// box, etc. However, if the window has focus but no controls with keyboard focus, the
						// key down/up messages are sent to the parent window instead.
						// Note: if your control is not getting key events, check whether 'selectable()' has been
						// set in the creation parameters.

						// Handle the key down/up event
						auto vk_key = UINT(wparam);
						auto repeats = UINT(lparam & 0xFFFF);
						auto flags = UINT(lparam >> 16) & 0xFFFF;
						//auto is_extended = (flags & KF_EXTENDED) != 0;
						//auto scan_code = (is_extended ? 0xE000 : 0x0000) | (flags & 0xFF);
						auto args = KeyEventArgs(this, vk_key, message == WM_KEYDOWN, m_hwnd, repeats, flags);

						// Allow parent controls to filter the key events
						OnKeyPreview(args);
						if (args.m_handled)
							return true;

						// Process the key event
						OnKey(args);
						if (args.m_handled)
							return true;

						break;
					}
					case WM_COMMAND:
					{
						// WM_COMMAND, when sent from a button, is sent to the parent control/window.
						// Typically this would be a Form but it can be any container control such as Panel,GroupBox.
						// Instead of this, we want the control to receive the WM_COMMAND so forward the message to the control.
						auto ctrl_hwnd = HWND(lparam); // Only valid if sent from a control (i.e. not sent from a menu or accelerator)
						if (ctrl_hwnd != nullptr && ctrl_hwnd != m_hwnd)
							return ::SendMessageW(ctrl_hwnd, message, wparam, lparam);

						break;
					}
					case WM_LBUTTONDOWN:
					case WM_RBUTTONDOWN:
					case WM_MBUTTONDOWN:
					case WM_XBUTTONDOWN:
					case WM_LBUTTONUP:
					case WM_RBUTTONUP:
					case WM_MBUTTONUP:
					case WM_XBUTTONUP:
					{
						Point pt(lparam);
						auto key_state = EMouseKey(GET_KEYSTATE_WPARAM(wparam)) | (::GetKeyState(VK_MENU) < 0 ? EMouseKey::Alt : EMouseKey());
						auto down =
							message == WM_LBUTTONDOWN ||
							message == WM_RBUTTONDOWN ||
							message == WM_MBUTTONDOWN ||
							message == WM_XBUTTONDOWN;
						auto btn =
							(message == WM_LBUTTONDOWN || message == WM_LBUTTONUP) ? EMouseKey::Left :
							(message == WM_RBUTTONDOWN || message == WM_RBUTTONUP) ? EMouseKey::Right :
							(message == WM_MBUTTONDOWN || message == WM_MBUTTONUP) ? EMouseKey::Middle :
							(message == WM_XBUTTONDOWN || message == WM_XBUTTONUP) ? (HiWord(wparam) == XBUTTON1 ? EMouseKey::XButton1 : EMouseKey::XButton2) :
							EMouseKey();

						// Event order is: down, click, up
						MouseEventArgs args(this, btn, down, pt, key_state);
						if (down) OnMouseButton(args);
						DetectSingleClicks(args);
						if (!down) OnMouseButton(args);
						if (args.m_handled)
							return true;

						break;
					}
					case WM_MOUSEWHEEL:
					{
						auto pt = PointToClient(Point(lparam));
						auto delta = GET_WHEEL_DELTA_WPARAM(wparam);
						auto key_state = EMouseKey(GET_KEYSTATE_WPARAM(wparam)) | (::GetKeyState(VK_MENU) < 0 ? EMouseKey::Alt : EMouseKey());
						MouseWheelArgs args(this, delta, pt, key_state);
						OnMouseWheel(args);
						if (args.m_handled)
							return true;

						break;
					}
					case WM_MOUSEMOVE:
					{
						auto pt = Point(lparam);
						auto key_state = EMouseKey(GET_KEYSTATE_WPARAM(wparam)) | (::GetKeyState(VK_MENU) < 0 ? EMouseKey::Alt : EMouseKey());
						MouseEventArgs args(this, key_state, false, pt, key_state);
						OnMouseMove(args);
						if (args.m_handled)
							return true;

						break;
					}
					case WM_MOUSEACTIVATE:
					{
						if (cp().m_selectable)
						{
							SetFocus(m_hwnd);
							return MA_ACTIVATE;
						}
						break;
					}
					case WM_DROPFILES:
					{
						// Remember to call 'AllowDrop()' on this control

						// Files dropped onto this control
						auto drop_info = HDROP(wparam);
						DropFilesEventArgs drop(drop_info);

						// Read the file paths of the dropped files
						int i = 0;
						drop.m_filepaths.resize(::DragQueryFileW(drop_info, 0xFFFFFFFF, nullptr, 0));
						for (auto& path : drop.m_filepaths)
						{
							path.resize(static_cast<size_t>(::DragQueryFileW(drop_info, i, 0, 0)) + 1, 0);
							Check(::DragQueryFileW(drop_info, i, &path[0], UINT(path.size())) != 0, "Failed to query file name from dropped files");
							++i;
						}

						// Call the handler
						OnDropFiles(drop);
						return true;
					}
				}
				return DefWndProc(message, wparam, lparam);
			}

			// Message map function
			// 'hwnd' is the handle of the top level window that contains this control.
			// Messages processed here are the messages sent to the parent window, *not* messages for this window
			// Only change 'result' when specifically returning a result (it defaults to S_OK)
			// Return true to halt message processing, false to allow other controls to process the message
			virtual bool ProcessWindowMessage(HWND toplevel_hwnd, UINT message, WPARAM wparam, LPARAM lparam, LRESULT& result)
			{
				// Default handling of parent window messages for all controls (including forms)
				// Sub-classed controls should override this method.
				// For Controls:
				//   - This method is called for every message received by the top-level window.
				//   - When you're handling messages, think "is this a message the window receives or the control receives"
				// For Forms:
				//   - WndProc cannot be sub-classed, this method *is* the WndProc for the Form.
				//   - Forms can be parented to other forms, remember to check 'm_hwnd == toplevel_hwnd'
				//     to detect messages received by this form as opposed to messages received
				//     by a parent form and then forwarded on to this form.
				// Remember:
				//   - This control is not necessarily a child control, it can be the Form using
				//     the default implementation, or an owned form.

				//WndProcDebug(toplevel_hwnd, message, wparam, lparam, FmtS("CtrlMsg: %s", m_name.c_str()));
				switch (message)
				{
					case WM_INITDIALOG:
					{
						// When the parent dialog is initialising (i.e. it's received this WM_INITDIALOG message and
						// forwarded it on to this child control), make sure the WndProc is set.
						if (m_hwnd == nullptr)
						{
							// Since this is a control on a dialog, the control will have been created by the 
							// ::DialogBox... function. We should have a control id to allow the hwnd to be extracted
							// from the dialog. If not, then we can't attach this Control to the created control, in
							// which case why does it exist? It can't be used to interact with the control on the dialog.
							// However, allow this control to not match a control on the dialog, for debugging convenience.
							// The first use of it's zero m_hwnd will catch it
							assert("Controls on a dialog must have IDs" && cp().m_id.value != ID_UNUSED);
							auto hwnd = ::GetDlgItem(toplevel_hwnd, cp().m_id.value);
							if (hwnd != nullptr)
								Attach(hwnd);
						}
						if (m_hwnd != nullptr)
						{
							RecordPosOffset();
							ResizeToParent(); // Resize to the parent (only moves the control if Dock/Anchors are used since we just set the offset)
						}
						return ForwardToChildren(toplevel_hwnd, message, wparam, lparam, result, AllChildren);
					}
					case WM_DESTROY:
					{
						// Notify children of the WM_DESTROY before destroying this window so that destruction
						// occurs from leaves to root of the control tree
						if (ForwardToChildren(toplevel_hwnd, message, wparam, lparam, result, AllChildren))
							return true;

						// Parent window is being destroyed, destroy this window too
						if (toplevel_hwnd != m_hwnd)
							::DestroyWindow(m_hwnd);

						// Allow WM_DESTROY to be passed to the parent window's WndProc
						return false;
					}
					case WM_WINDOWPOSCHANGED:
					{
						// See the notes in Control::WndProc for this message
						return false;
					}
					case WM_TIMER:
					{
						// Timer event, forwarded to all child controls
						auto event_id = UINT_PTR(wparam);
						OnTimer(TimerEventArgs(event_id));
						return ForwardToChildren(toplevel_hwnd, message, wparam, lparam, result, AllChildren);
					}
					case WM_CTLCOLORSTATIC:
					case WM_CTLCOLORBTN:
					case WM_CTLCOLOREDIT:
					case WM_CTLCOLORLISTBOX:
					case WM_CTLCOLORSCROLLBAR:
					{
						// WM_CTLCOLORDLG is sent to the dialog itself, all other WM_CTLCOLOR messages
						// are sent to the parent window of the control that they refer to. 

						// This is a request to set the foreground and background colour in the DC for the specified label control.
						if (HWND(lparam) == m_hwnd)
						{
							auto hdc = HDC(wparam); // The DC to set colours in
							if (m_brush_fore != nullptr)
							{
								auto col = m_brush_fore.colour();

								// If we have a fore colour, set it, otherwise leave it as the default
								Check(::SetTextColor(hdc, col) != CLR_INVALID, "Set text fore colour failed");
							}
							if (m_brush_back != nullptr)
							{
								auto col = m_brush_back.colour();

								// If we have a background colour, set it and return the background brush
								Check(::SetBkMode(hdc, OPAQUE) != 0, "Set back colour mode failed");
								Check(::SetBkColor(hdc, col) != CLR_INVALID, "Set text back colour failed");
								result = LRESULT(static_cast<HBRUSH>(m_brush_back));
								return true;
							}
							// If we don't have a background brush, let the WndProc handle it
							return false;
						}

						// Not for this control, forward to children
						return ForwardToChildren(toplevel_hwnd, message, wparam, lparam, result, IsChild);
					}
					case WM_MOUSEWHEEL:
					{
						// WM_MOUSEWHEEL is only sent to the focused window, unlike mouse
						// button/move messages which are sent to the control with focus.

						// Forward to the leaf controls first
						if (ForwardToChildren(toplevel_hwnd, message, wparam, lparam, result, IsChild))
							return true;

						auto delta = GET_WHEEL_DELTA_WPARAM(wparam);
						auto pt = PointToClient(Point(lparam));
						auto keystate = EMouseKey(GET_KEYSTATE_WPARAM(wparam)) | (::GetKeyState(VK_MENU) < 0 ? EMouseKey::Alt : EMouseKey());
						MouseWheelArgs args(this, delta, pt, keystate);
						OnMouseWheel(args);
						if (args.m_handled)
							return true;

						// Pass to WndProc, I don't think it does anything, but doesn't hurt either
						return false;
					}
					case WM_KEYDOWN:
					case WM_KEYUP:
					{
						// Don't forward key events to child controls. (see the notes in WndProc)
						// Key events are sent directly to the window with keyboard focus. This may
						// be the parent window if no child controls have input focus. The control that
						// receives the events can choose to forward it to child controls
						return false;
					}
					default:
					{
						// By default, forward the parent window message to the child controls only
						return ForwardToChildren(toplevel_hwnd, message, wparam, lparam, result, IsChild);
					}
				}
			}

			// Forward a message to child controls or owned windows based on 'pred'
			template <typename Pred> bool ForwardToChildren(HWND toplevel_hwnd, UINT message, WPARAM wparam, LPARAM lparam, LRESULT& result, Pred pred)
			{
				// Message handling can cause children to be added/removed.
				// We need to locally buffer pointers to the child controls.
				// Buffer on the stack unless there are a large number of child controls
				struct ChildBuf
				{
					size_t    m_count;
					Control*  m_stack[256];
					Controls  m_heap;
					Control** m_buf;

					ChildBuf(Controls const& children)
						:m_count(children.size())
						,m_stack()
						,m_heap(m_count > _countof(m_stack) ? m_count    : 0U)
						,m_buf (m_count > _countof(m_stack) ? &m_heap[0] : &m_stack[0])
					{
						std::copy(children.begin(), children.end(), m_buf);
					}
					Control** begin() const
					{
						return &m_buf[0];
					}
					Control** end() const
					{
						return begin() + m_count;
					}
				} children(m_child);

				// Forward the message to each child
				for (auto child : children)
				{
					if (!pred(child)) continue;
					if (child->ProcessWindowMessage(toplevel_hwnd, message, wparam, lparam, result))
						return true;
				}
				return false;
			}
			static bool AllChildren(Control const*)
			{
				return true;
			}
			static bool IsForm (Control const* c)
			{
				assert(c != nullptr);
				return c->cp().top_level();
			}
			static bool IsChild(Control const* c)
			{
				return !IsForm(c);
			}
			static bool IsPinnedForm(Control const* c)
			{
				return IsForm(c) && c->cp<FormParams<>>().m_pin_window;
			}

			// Save a copy of 'p' to 'm_cp'
			void SaveParams(Params<> const& p)
			{
				if (m_cp.get() == &p) return;
				assert("Control ID is out of range" && p.m_id.value >= ID_BEG && p.m_id.value < ID_END);

				auto cp = p.clone(p);
				assert("You've forgotten to overload 'clone' for this Params type" && typeid(*cp).hash_code() == typeid(p).hash_code());
				m_cp.reset(cp);
			}

			// Attach/Detach this control wrapper to/form the associated window handle
			// WARNING: The overload of this method will not be called when Create() is called from the Control constructor
			virtual void Attach(HWND hwnd)
			{
				assert(m_hwnd == nullptr && hwnd != nullptr);
				m_hwnd = hwnd;

				// If the wndproc for this control is not our thunk, hook it
				auto wndproc = (WNDPROC)::GetWindowLongPtrW(m_hwnd, GWLP_WNDPROC);
				auto dlgproc = (DLGPROC)::GetWindowLongPtrW(m_hwnd, DWLP_DLGPROC);
				if (wndproc != m_thunk.GetCodeAddress() && dlgproc != m_thunk.GetCodeAddress())
				{
					if (dlgproc == nullptr)
						m_oldproc = (WNDPROC)::SetWindowLongPtrW(m_hwnd, GWLP_WNDPROC, LONG_PTR(m_thunk.GetCodeAddress()));
					else
						m_oldproc = (WNDPROC)::SetWindowLongPtrW(m_hwnd, DWLP_DLGPROC, LONG_PTR(m_thunk.GetCodeAddress()));
				}
			}
			virtual void Detach()
			{
				if (m_hwnd == nullptr)
					return;

				// Restore the original wndproc
				auto wndproc = (WNDPROC)::GetWindowLongPtrW(m_hwnd, GWLP_WNDPROC);
				auto dlgproc = (DLGPROC)::GetWindowLongPtrW(m_hwnd, DWLP_DLGPROC);
				if (wndproc == m_thunk.GetCodeAddress())
					::SetWindowLongPtrW(m_hwnd, GWLP_WNDPROC, LONG_PTR(m_oldproc));
				else if (dlgproc == m_thunk.GetCodeAddress())
					::SetWindowLongPtrW(m_hwnd, DWLP_DLGPROC, LONG_PTR(0));

				m_oldproc = nullptr;
				m_hwnd = nullptr;
			}

			#pragma region Window Class
			// Register the window class for 'WndType'
			// Custom window types need to call this before using Create() or WndClassInfo()
			template <typename WndType> static WndClassEx const& RegisterWndClass(HINSTANCE hinst =  GetModuleHandleW(nullptr))
			{
				// Register on initialisation
				static WndClassEx s_wc = [=]
				{
					// Get the window class name
					static wchar_t auto_class_name[64];
					auto class_name = WndType::WndClassName() ? WndType::WndClassName()
						: (_swprintf_c(auto_class_name, _countof(auto_class_name), L"wingui::%p", &s_wc), &auto_class_name[0]);

					// See if the wndclass is already registered
					WndClassEx wc(class_name, hinst);
					if (wc.m_atom != 0)
						return wc;

					// Register the window class
					wc.cbSize        = sizeof(WNDCLASSEXW);
					wc.style         = WndType::WndClassStyle();
					wc.cbClsExtra    = 0;
					wc.cbWndExtra    = 0;
					wc.hInstance     = hinst;
					wc.hIcon         = WndType::WndIcon(hinst, true);
					wc.hIconSm       = WndType::WndIcon(hinst, false);
					wc.hCursor       = WndType::WndCursor(hinst);
					wc.hbrBackground = WndType::WndBackground();
					wc.lpszMenuName  = WndType::WndMenu();
					wc.lpfnWndProc   = WndType::WndProcPtr();
					wc.lpszClassName = class_name;
					return std::move(wc.Register());
				}();
				return s_wc;
			}

			// These methods are provided so that windows that inherit from ControlT
			// can use WndClassInfo<MyControl> specialising just parts of the wndclass.
			static wchar_t const* WndClassName()
			{
				// Returning null causes a name to be automatically generated
				return nullptr;
			}
			static int WndClassStyle()
			{
				// Don't include CS_HREDRAW and CS_VREDRAW, these just add unnecessary redraws
				return CS_DBLCLKS;
			}
			static HICON WndIcon(HINSTANCE hinst, bool large)
			{
				(void)hinst,large;
				//::LoadIcon(hinst, MAKEINTRESOURCE(IDI_ICON));
				//::LoadIcon(hinst, MAKEINTRESOURCE(IDI_SMALL));
				return nullptr;
			}
			static HCURSOR WndCursor(HINSTANCE hinst)
			{
				(void)hinst;
				auto cur = ::LoadCursor(nullptr, IDC_ARROW); // Load arrow from the system, not this EXE image
				Check(cur != nullptr, "Failed to load default arrow cursor");
				return cur;
			}
			static HBRUSH WndBackground()
			{
				// Returning null forces handling of WM_ERASEBKGND
				// Don't return null by default, subclasses can return S_FALSE
				// from WM_ERASEBKGND to not erase the background. Most controls
				// use the background brush.
				return ::GetSysColorBrush(COLOR_3DFACE);
			}
			static wchar_t const* WndMenu()
			{
				// Typically you might return: MAKEINTRESOURCE(IDM_MENU);
				return nullptr; 
			}
			static WNDPROC WndProcPtr()
			{
				return &InitWndProc;
			}
			#pragma endregion

			#pragma region WndProc

			// Window creation initialisation parameter wrapper
			struct InitParam
			{
				Control* m_this;
				void* m_lparam;
				InitParam(Control* this_, void* lparam) :m_this(this_) ,m_lparam(lparam) {}
			};

			// The initial wndproc function used in forms, dialogs, and custom controls.
			static LRESULT __stdcall InitWndProc(HWND hwnd, UINT message, WPARAM wparam, LPARAM lparam)
			{
				//WndProcDebug(hwnd, message, wparam, lparam);
				if (message == WM_NCCREATE)
				{
					auto init = reinterpret_cast<InitParam*>(reinterpret_cast<CREATESTRUCT*>(lparam)->lpCreateParams);

					// Set the wndproc to the default before replacing it with the thunk in Attach()
					assert((WNDPROC)::GetWindowLongPtrW(hwnd, GWLP_WNDPROC) == &InitWndProc);
					::SetWindowLongPtrW(hwnd, GWLP_WNDPROC, LONG_PTR(&::DefWindowProcW));
					init->m_this->Attach(hwnd);
					return init->m_this->WndProc(message, wparam, lparam);
				}
				if (message == WM_INITDIALOG)
				{
					auto init = reinterpret_cast<InitParam*>(lparam);

					// The DWLP_DLGPROC is the user wndproc supplied in CreateDialog.
					// GWLP_WNDPROC is an internal dialog wndproc (user32::_DefDlgProc != DefDlgProc)
					// 'user32::_DefDlgProc' calls the user DLGPROC which, on returning FALSE, then
					// handles the message internally (DefWindowProc is never called)
					// Restore 'DWLP_DLGPROC' to the default (nullptr) before replacing it with the thunk
					assert((WNDPROC)::GetWindowLongPtrW(hwnd, DWLP_DLGPROC) == &InitWndProc);
					::SetWindowLongPtrW(hwnd, DWLP_DLGPROC, LONG_PTR(&::DefDlgProcW));
					init->m_this->Attach(hwnd);
					return init->m_this->WndProc(message, wparam, LPARAM(init->m_lparam));
				}
				return ::DefWindowProcW(hwnd, message, wparam, lparam);
			}
			static LRESULT __stdcall StaticWndProc(HWND hwnd, UINT message, WPARAM wparam, LPARAM lparam)
			{
				// 'm_thunk' causes 'hwnd' to actually be the pointer to the control rather than the hwnd
				auto& ctrl = *reinterpret_cast<Control*>(hwnd);
				assert("Message received for destructed control" && ctrl.m_hwnd != nullptr);
				return ctrl.WndProc(message, wparam, lparam);
			}
			LRESULT DefWndProc(UINT message, WPARAM wparam, LPARAM lparam)
			{
				if (m_oldproc == &::DefDlgProcW) return FALSE;
				if (m_oldproc != nullptr) return ::CallWindowProcW(m_oldproc, m_hwnd, message, wparam, lparam);
				return ::DefWindowProcW(m_hwnd, message, wparam, lparam);
			}

			// Handy debugging method for displaying WM_MESSAGES
			// Call with 'hwnd' == 0, message = 0/1 to disable/enable trace
			#if PR_WNDPROCDEBUG
			static void WndProcDebug(HWND hwnd, UINT message, WPARAM wparam, LPARAM lparam, char const* name = nullptr)
			{
				if (true
					// && (name != nullptr && strncmp(name,"lbl-version",5) == 0)
					// && (message == WM_WINDOWPOSCHANGING || message == WM_WINDOWPOSCHANGED || message == WM_ERASEBKGND || message == WM_PAINT)
					)
				{
					auto out = [](char const* s) { OutputDebugStringA(s); };
					//auto out = [](char const* s) { std::ofstream("\\dump\\wingui.log", std::ofstream::app).write(s,strlen(s)); };

					// Display the message
					static int msg_idx = 0; ++msg_idx;
					auto m = gui::DebugMessage(hwnd, message, wparam, lparam);
					if (*m)
					{
						for (int i = 1; i < wnd_proc_nest(); ++i) out("\t");
						out(pr::FmtX<struct X, 256, char>("%5d|%-30s|%s\n", msg_idx, name, m));
					}
					if (msg_idx == 0) _CrtDbgBreak();
				}
			}
			static int& wnd_proc_nest()
			{
				static int s_wnd_proc_nest = 0;
				return s_wnd_proc_nest;
			}
			#else
			static void WndProcDebug(HWND , UINT , WPARAM , LPARAM , char const* = nullptr) {}
			#endif
			static void WndProcDebug(MSG& msg, char const* name = nullptr)
			{
				WndProcDebug(msg.hwnd, msg.message, msg.wParam, msg.lParam, name);
			}
	
			#pragma endregion

			// Adjust the size of this control relative to 'parent_client'
			// 'parent_client' is the available client area on the parent in parent client coordinates
			// (typically 0,0 -> w,h. But not always, e.g. TabControl has the tabs in client space).
			// 'parent_client' may be the client area that the parent *will* have soon.
			virtual void ResizeToParent(Rect const& parent_client, bool repaint = false)
			{
				// Resize even if not visible so that the control has the correct size on becoming visible
				// Top level controls (i.e forms) will only call this if they are pinned.
				if (!m_hwnd || !m_parent.hwnd())
					return;

				// Get the available area and this control's area relative to it (including margin).
				Rect p = parent_client;
				auto c = ParentRect().Adjust(cp().m_margin);
				auto w = c.width();
				auto h = c.height();
				if (cp().m_dock == EDock::None)
				{
					// Note: m_pos_offset = [c.left-p.left, c.top-p.top, c.right-p.right, c.bottom-p.bottom]
					if (AllSet(cp().m_anchor, EAnchor::Left))
					{
						c.left = p.left + m_pos_offset.left;
						if (!AllSet(cp().m_anchor, EAnchor::Right))
							c.right = c.left + w;
					}
					if (AllSet(cp().m_anchor, EAnchor::Top))
					{
						c.top = p.top + m_pos_offset.top;
						if (!AllSet(cp().m_anchor, EAnchor::Bottom))
							c.bottom = c.top + h;
					}
					if (AllSet(cp().m_anchor, EAnchor::Right))
					{
						c.right = p.right + m_pos_offset.right;
						if (!AllSet(cp().m_anchor, EAnchor::Left))
							c.left = c.right - w;
					}
					if (AllSet(cp().m_anchor, EAnchor::Bottom))
					{
						c.bottom = p.bottom + m_pos_offset.bottom;
						if (!AllSet(cp().m_anchor, EAnchor::Top))
							c.top = c.bottom - h;
					}
				}
				else
				{
					// Find the region remaining after earlier children have been docked.
					p = m_parent.ctrl() != nullptr ? m_parent->ExcludeDockedChildren(p, Index()) : p;
					switch (cp().m_dock)
					{
						case EDock::Fill:   c = p; break;
						case EDock::Top:    c = Rect(p.left, p.top, p.right, p.top + h); break;
						case EDock::Bottom: c = Rect(p.left, p.bottom - h, p.right, p.bottom); break;
						case EDock::Left:   c = Rect(p.left, p.top, p.left + w, p.bottom); break;
						case EDock::Right:  c = Rect(p.right - w, p.top, p.right, p.bottom); break;
						default: throw std::runtime_error("Unknown dock style");
					}
				}
				RAII<bool> no_save_ofs(m_pos_ofs_suspend, true);
				ParentRect(c.Adjust(-cp().m_margin), repaint);
			}
			void ResizeToParent(bool repaint = false)
			{
				if (m_parent.hwnd() == nullptr) return;
				ResizeToParent(ClientRect(m_parent, true), repaint);
			}

			// Handle auto position/size
			void AutoSizePosition(int& x, int& y, int& w, int& h, Control const* parent) const
			{
				using namespace auto_size_position;

				// Adjust 'x,y,w,h' based on auto size/position bits
				CalcPosSize(x, y, w, h, cp().m_margin, [&](int id) -> Rect
				{
					if (id == 0)
					{
						// Get the parent control bounds in parent client space
						if (parent == nullptr) return MinMaxInfo(m_metrics.CurrentDPI()).Bounds();
						return parent->ExcludeDockedChildren(parent->ClientRect(true)); // Includes padding in the parent
					}
					else if (id == -1)
					{
						// Get the preferred size of this control
						auto sz = PreferredSize();
						return Rect(0, 0, sz.cx, sz.cy);
					}
					else
					{
						// Find the child 'id' and return it's parent space rect including margins
						assert("Sibling control id given without a parent" && parent != nullptr);
						for (auto child : parent->m_child)
						{
							if (child->cp().m_id.value != id)
								continue;

							// If the sibling exists, get it's parent space rect
							if (::IsWindow(child->m_hwnd))
							{
								auto rect = child->ParentRect().Adjust(child->Margin());
								if (!child->Visible()) { rect.right = rect.left; rect.bottom = rect.top; } // Invisible controls act as points
								return rect;
							}
							// If not, then try to get the size from the creation parameters
							else
							{
								auto& p = child->cp();
								auto x = IsAutoPos(p.m_x) ? 0 : p.m_x;
								auto y = IsAutoPos(p.m_y) ? 0 : p.m_y;
								auto w = IsAutoPos(p.m_w) ? 0 : p.m_w;
								auto h = IsAutoPos(p.m_h) ? 0 : p.m_h;
								return Rect(x, y, x+w, y+h);
							}
						}
						throw std::runtime_error("Sibling control not found");
					}
				});
			}

			// Record the position of the control relative to 'm_parent'
			void RecordPosOffset()
			{
				// Store distances so that this control's position equals
				// parent.left + m_pos_offset.left, parent.right + m_pos_offset.right, etc..
				// (i.e. right and bottom offsets are typically negative)
				if (!m_hwnd || m_pos_ofs_suspend || m_parent.hwnd() == nullptr)
					return;

				// Record the offset relative to the parent
				auto p = cp().top_level() ? ScreenRect(m_parent) : ClientRect(m_parent, true);
				auto c = ParentRect().Adjust(cp().m_margin);
				m_pos_offset = Rect(c.left - p.left, c.top - p.top, c.right - p.right, c.bottom - p.bottom);
			}

			// Measure 'text' using the font in this control
			Size MeasureString(string const& text, int max_width = 0, UINT flags = 0) const
			{
				return gui::MeasureString(text, m_hwnd, cp().m_font ? *cp().m_font : nullptr, max_width, flags);
			}

			// Return the client area of this control, including or excluding padding
			virtual Rect ClientRectCore(bool padded) const
			{
				// By default, call the static implementation
				return Control::ClientRect(m_hwnd, padded);
			}

		private:

			// Mouse single click detection.
			void DetectSingleClicks(MouseEventArgs args)
			{
				// Get a reference to the 'down-at' time for the btn
				auto now = ::GetMessageTime();
				auto& down_at = m_down_at[args.m_button];

				// If this is a click, call the OnMouseClick handler
				auto const click_thres = 150;
				if (!args.m_down && now - down_at < click_thres)
				{
					// Add all buttons that went down within the click
					// threshold time. This allows for button click combos
					for (auto& d : m_down_at)
					{
						if (now - d.second > click_thres) continue;
						args.m_button = args.m_button|d.first;
					}

					// Handle the click
					OnMouseClick(args);
				}

				// Reset the 'down-at' time
				down_at = args.m_down ? now : 0;
			}

			// Create or resize the double buffer
			void EnsureDoubleBuffer()
			{
				if (!cp().m_dbl_buffer)
				{
					if (m_dbl_buffer) ::DeleteObject(m_dbl_buffer);
					m_dbl_buffer = nullptr;
				}
				else
				{
					// Ensure the double buffer is the correct size
					auto client = ClientRect(false);
					auto bm = Image::Info(m_dbl_buffer);
					if (bm.bmWidth != client.width() || bm.bmHeight != client.height())
					{
						ClientDC dc(m_hwnd);
						if (m_dbl_buffer) ::DeleteObject(m_dbl_buffer);
						m_dbl_buffer = ::CreateCompatibleBitmap(dc, client.width(), client.height());

						// Invalidate the whole window if we've just created the
						// double buffer as the whole surface will need painting.
						Invalidate(false, &client, true);
					}
				}
			}
		};
		#pragma endregion

		#pragma region Form
		// A common, non-template, base class for all forms
		struct Form :Control, IWinGuiMsgFilter
		{
		protected:
			// Notes:
			// Neither Form or Control define a load of OnXYZ handlers. This is because it adds a
			// load of potentially unneeded entries to the 'vtbl'. The expected way to use this class
			// is to override ProcessWindowMessage and decode/handle the window messages you actually
			// need. Notice, WM_CREATE is typically not needed, the constructor of your derived type
			// is where OnCreate() code should go.
			// Re WndProc from MSDN:
			//  "You should use the dialog box procedure only if you use the dialog box wndclass for the dialog box.
			//   This is the default class and is used when no explicit wndclass is specified in the dialog box template"

			enum { IDC_PINWINDOW_OPT = 0x4e50, IDC_PINWINDOW_SEP };

			EDialogResult  m_dialog_result; // The code to return when the form closes
			Accel          m_accel;         // Keyboard accelerators
			bool           m_modal;         // True if this is a dialog being displayed modally

		public:

			enum { DefW = 800, DefH = 600 };
			template <typename Derived = void> using Params = FormParams<Derived>;

			// Form Constructor
			// The creation of the window is kept for the Form class to do, rather than the control,
			// so that a window or dialog gets created as appropriate. Also, passing parent == nullptr
			// to 'Control' because the virtual Parent() function will not call our overload until
			// construction is complete.
			// Callers should use m_create == ECreate::Create for modeless frame windows, ECreate::Defer
			// for windows that will be created later (either with Create() or ShowDialog()). ECreate::Auto
			// should not typically be used for forms.
			Form()
				:Form(Params<>())
			{}
			template <typename TParams, typename = typename TParams::this_type>
			explicit Form(TParams const& p)
				:Control(p)
				,m_dialog_result()
				,m_accel()
				,m_modal()
			{
				if (p.m_dpi_aware)
				{
					// This needs to be called before any HWNDs are created.
					// 'ERROR_ACCESS_DENIED' is returned if the DPI awareness is already set (from the manifest).
					if constexpr (WINVER >= 0x0605)
						Check(::SetProcessDpiAwarenessContext(DPI_AWARENESS_CONTEXT_PER_MONITOR_AWARE_V2) || GetLastError() == ERROR_ACCESS_DENIED, "DPI Aware mode not set");
				}
			}
			virtual ~Form()
			{
				// Close on destruction
				HideOnClose(false);
				Close();
			}

			// The parameters used to create this form (but updated to the current state)
			template <typename TParams = FormParams<>> TParams& cp() const
			{
				return m_cp->as<TParams>();
			}

			// Create the HWND for this window
			// After calling this, only 'Show()' can be used.
			// This function should be called after construction with m_create == ECreate::Defer
			void Create(Control::Params<> const& p_) override
			{
				// Save the creation data
				SaveParams(p_);
				auto& p = cp();
				{
					// Sanity check form parameters
					assert("window already created" && m_hwnd == nullptr);
					assert("Use EStartPosition::Manual when specifying screen X,Y coordinates for a window" && (
						((p.m_x == 0 || p.m_x == CW_USEDEFAULT) && (p.m_y == 0 || p.m_y == CW_USEDEFAULT)) || p.m_start_pos == EStartPosition::Manual));

					// If no window class is given, register this Form
					if (p.m_wcn == nullptr && p.m_wci == nullptr)
						p.m_wci = &RegisterWndClass<Form>();

					// Load accelerators
					m_accel =
						p.m_accel.m_handle != nullptr ? Accel(p.m_accel.m_handle, true) :
						p.m_accel.m_res_id != nullptr ? Accel(::LoadAcceleratorsW(p.m_hinst, p.m_accel.m_res_id)) :
						Accel();
				}

				InitParam lparam(this, p.m_init_param);

				// If this form has a dialog template, then create the window as a modeless dialog
				if (p.m_templ != nullptr)
				{
					assert(p.m_templ->valid());
					m_hwnd = ::CreateDialogIndirectParamW(p.m_hinst, *p.m_templ, p.m_parent, (DLGPROC)&InitWndProc, LPARAM(&lparam));
					Check(m_hwnd != nullptr, "CreateDialogIndirectParam failed");
					Parent(p.m_parent);
				}
				else if (p.m_id.value != ID_UNUSED) // Create from a dialog resource id
				{
					m_hwnd = ::CreateDialogParamW(p.m_hinst, MakeIntResourceW(p.m_id.value), p.m_parent, (DLGPROC)&InitWndProc, LPARAM(&lparam));
					Check(m_hwnd != nullptr, "CreateDialogParam failed");
					Parent(p.m_parent);
				}
				else // Otherwise create as a normal window
				{
					Control::Create(p.as<Control::Params<>>());
				}

				// Initialise the offset and menu state for pinned windows
				if (PinWindow() && m_parent.hwnd() != nullptr)
					PinWindow(true);
			}

			// Display as a mode-less form, creating the window first if necessary
			virtual void Show(int show = SW_SHOW)
			{
				return ShowInternal(show);
			}

			// Display the form modally, creating the window first if necessary
			virtual EDialogResult ShowDialog(WndRefC parent = nullptr)
			{
				return ShowDialogInternal(parent);
			}

			// Close this form
			virtual bool Close(EDialogResult dialog_result = EDialogResult::None)
			{
				return CloseInternal(dialog_result);
			}

			// Get/Set whether the form uses dialog-like message handling
			bool DialogBehaviour() const
			{
				return cp().m_dlg_behaviour;
			}
			void DialogBehaviour(bool enabled)
			{
				cp().m_dlg_behaviour = enabled;
			}

			// Get/Set whether the window closes or just hides when closed
			bool HideOnClose() const
			{
				return cp().m_hide_on_close;
			}
			void HideOnClose(bool enable)
			{
				cp().m_hide_on_close = enable;
			}

			// Get/Set whether the window is pinned to it's parent
			bool PinWindow() const
			{
				return cp().m_pin_window;
			}
			void PinWindow(bool pin)
			{
				assert("Pinned window does not have a parent" && (!pin || m_parent.hwnd() != nullptr));
				cp().m_pin_window = pin;
				cp().m_anchor = pin ? EAnchor::TopLeft : EAnchor::None;
				::CheckMenuItem(::GetSystemMenu(m_hwnd, FALSE), IDC_PINWINDOW_OPT, MF_BYCOMMAND|(pin ? MF_CHECKED : MF_UNCHECKED));
				RecordPosOffset();
			}

			// Get/Set the parent of this form
			WndRef Parent() const override
			{
				return Control::Parent();
			}
			void Parent(WndRef parent) override
			{
				if (m_parent.hwnd() != nullptr)
				{
					auto sysmenu = ::GetSystemMenu(m_hwnd, FALSE);
					if (sysmenu)
					{
						::RemoveMenu(sysmenu, IDC_PINWINDOW_SEP, MF_BYCOMMAND|MF_SEPARATOR);
						::RemoveMenu(sysmenu, IDC_PINWINDOW_OPT, MF_BYCOMMAND|MF_STRING);
					}
				}

				Control::Parent(parent);

				if (m_parent.hwnd() != nullptr)
				{
					auto sysmenu = ::GetSystemMenu(m_hwnd, FALSE);
					if (sysmenu)
					{
						auto idx = ::GetMenuItemCount(sysmenu) - 2;
						Check(::InsertMenuW(sysmenu, idx++, MF_BYPOSITION|MF_SEPARATOR, IDC_PINWINDOW_SEP, 0), "InsertMenu failed");
						Check(::InsertMenuW(sysmenu, idx++, MF_BYPOSITION|MF_STRING|(PinWindow()?MF_CHECKED:MF_UNCHECKED), IDC_PINWINDOW_OPT, L"Pin Window"), "InsertMenu failed");
					}
				}
			}

			// Get/Set the menu. Set returns the previous menu.
			// If replacing a menu, remember to call DestroyMenu on the previous one
			Menu MenuStrip() const
			{
				assert(::IsWindow(m_hwnd));
				return Menu(::GetMenu(m_hwnd), false);
			}
			Menu MenuStrip(Menu menu)
			{
				CreateHandle();
				auto prev = MenuStrip();
				Check(::SetMenu(m_hwnd, menu) != 0, "Failed to set menu");
				return prev;
			}
			Menu MenuStrip(Menu::EKind kind, Menu::ItemList const& items)
			{
				return MenuStrip(Menu(kind, items));
			}

			// Get/Set the control's icon
			HICON Icon(bool big_icon) const
			{
				assert(::IsWindow(m_hwnd));
				return SendMsg<HICON>(WM_GETICON, big_icon ? ICON_BIG : ICON_SMALL, 0);
			}
			HICON Icon(HICON icon, bool big_icon)
			{
				CreateHandle();
				return SendMsg<HICON>(WM_SETICON, big_icon ? ICON_BIG : ICON_SMALL, LPARAM(icon));
			}

			// Create a dialog template from the child controls of this form
			DlgTemplate GenerateDlgTemplate()
			{
				// The dialog template must use the dialog window class
				auto templ = DlgTemplate(Params<>(cp()).parent(m_parent).wndclass(nullptr));
				for (auto& child : m_child)
					templ.Add(Control::Params<>(child->cp()).parent(this));

				return templ;
			}

		private:

			// Support dialog behaviour and keyboard accelerators
			bool TranslateMessage(MSG& msg) override
			{
				return
					(m_accel && TranslateAcceleratorW(m_hwnd, m_accel, &msg)) ||
					(cp().m_dlg_behaviour && IsDialogMessageW(m_hwnd, &msg));
			}

			// Window proc. Derived forms should not override WndProc.
			// All messages are passed to 'ProcessWindowMessage' so use that.
			LRESULT WndProc(UINT message, WPARAM wparam, LPARAM lparam) override final
			{
				#if PR_WNDPROCDEBUG
				RAII<int> nest(wnd_proc_nest(), wnd_proc_nest()+1);
				//WndProcDebug(m_hwnd, message, wparam, lparam, FmtS("FormWPMsg: %s", m_name.c_str()));
				#endif

				LRESULT result = S_OK;

				// Forward the message to the message map function
				// which will forward the message to nested controls
				// If the message map doesn't handle the message,
				// pass it to the WndProc for this form.
				if (!ProcessWindowMessage(m_hwnd, message, wparam, lparam, result))
					result = Control::WndProc(message, wparam, lparam);
				
				// This is used for DialogProc somehow
				::SetWindowLongPtrW(m_hwnd, DWLP_MSGRESULT, result);
				return result;
			}

		protected:

			// Display as a mode-less form, creating the window first if necessary
			void ShowInternal(int show)
			{
				// If the window does not yet exist, create it
				CreateHandle();

				// Not showing the window modally
				m_modal = false;

				// Show the window, non-modally
				ShowWindow(m_hwnd, show);
				UpdateWindow(m_hwnd);
			}

			// Display the form modally
			EDialogResult ShowDialogInternal(WndRefC parent)
			{
				// Showing the window modally
				m_modal = true;

				// Set the owner window
				cp().m_parent = parent.hwnd();

				// If the window does not yet exist, create it
				CreateHandle();
				Parent(cp().m_parent);

				// Create a message loop for this dialog
				struct ModalLoop :WinGuiMsgLoop
				{
					Form* m_dialog;

					ModalLoop(Form* dialog)
						:m_dialog(dialog)
					{}
					int Run() override
					{
						MSG msg = {};
						for (int result; m_dialog->m_hwnd != nullptr && (result = ::GetMessageW(&msg, nullptr, 0, 0)) != 0; )
						{
							Check(result > 0, "GetMessage failed"); // GetMessage returns negative values for errors
							//WndProcDebug(msg, "ModalLoop");

							// If the message is handled as a dialog message, don't translate it
							if (!m_dialog->cp().m_dlg_behaviour || !::IsDialogMessageW(m_dialog->m_hwnd, &msg))
							{
								TranslateMessage(msg);
							}

							// WM_QUIT must be manually forwarded
							if (msg.message == WM_QUIT)
							{
								PostMessageW(nullptr, WM_QUIT, 0, 0);
								break;
							}
						}
						return 0;
					}
				};
				ModalLoop modal_loop(this);

				// Show the window modally
				// Once the dialog is closed, destroy the window
				ShowWindow(m_hwnd, SW_SHOW);
				modal_loop.Run();

				return m_dialog_result;
			}

			// Close this form
			bool CloseInternal(EDialogResult dialog_result)
			{
				if (m_hwnd == nullptr)
					return false;

				// Save the dialog result
				m_dialog_result = dialog_result;

				// Post the close message to this form
				SendMsg<int>(WM_CLOSE);
				return true;
			}

			// Message map function
			// 'hwnd' is the handle of the parent window that contains this control.
			// Messages processed here are the messages sent to the parent window.
			// Only change 'result' when specifically returning a result (it defaults to S_OK)
			// Return true to halt message processing, false to allow other controls to process the message
			bool ProcessWindowMessage(HWND hwnd, UINT message, WPARAM wparam, LPARAM lparam, LRESULT& result) override
			{
				// For Forms, this method is effectively the WndProc for the form.
				// By default we don't forward messages to child controls. Use Control::ProcessWindowMessage()
				// to explicitly forward some message types, call ForwardToChildren only when you want different
				// behaviour to the Control default implementation but still want child controls to process the message.
				// Forms can be parented to other forms
				// Any message not handled here or in Control::ProcessWindowMessage() is passed to Control::WndProc
				// I.e the logic goes:
				//  <new message> arrives
				//    Form::ProcessWindowMessage() - form processes it which may mean passing it to 
				//    Control::ProcessWindowMessage() - the base class (Control)
				//      - each message may be passed to child controls using ForwardToChildren(), this is
				//      what Control::ProcessWindowMessage() does by default.
				//  otherwise:
				//     Control::WndProc() - otherwise the WndProc for the window handles it (in the base class)
				//  Some events in Control::WndProc() have handler virtual functions

				//WndProcDebug(hwnd, message, wparam, lparam, FmtS("FormMsg: %s", cp().m_name));
				switch (message)
				{
					case WM_CREATE:
					{
						// Pass WM_CREATE to WndProc
						return false;
					}
					case WM_INITDIALOG:
					{
						// Get the dialog window class information
						m_wci = WndClassEx(hwnd);

						// If no background brush for the dialog is set, use the default
						if (m_wci.hbrBackground == nullptr)
							m_wci.hbrBackground = WndBackground();

						// If this form is running modally, disable the parent
						auto parent = ::GetParent(m_hwnd);
						if (m_modal && parent != nullptr)
							::EnableWindow(parent, false);

						// The default Control::ProcessWindowMessage() handler for WM_INITDIALOG calls
						// 'Attach' which Forms don't need because we call Attach in InitWndProc.
						// Note: typically sub-classed forms will call base::ProcessWindowMessage()
						// before doing whatever they need to do in WM_INITDIALOG.
						// This is so that child controls get attached.
						if (ForwardToChildren(hwnd, message, wparam, lparam, result, AllChildren))
							return true;

						// Return false so that WM_INITDIALOG is passed to the WndProc
						return false;
					}
					case WM_CLOSE:
					{
						// If this form is running modally, re-enable the parent
						auto parent = ::GetParent(m_hwnd);
						if (m_modal && parent != nullptr)
							::EnableWindow(parent, true);

						// If we're only hiding, just go invisible
						if (cp().m_hide_on_close)
						{
							Visible(false);
							return true;
						}

						// Let the DefWndProc call DestroyWindow in response to the WM_CLOSE
						// All forms are created using CreateWindow or CreateDialog, so in all
						// cases we use 'DestroyWindow'.
						// Don't null m_hwnd here, that happens in WM_DESTROY
						return false;
					}
					case WM_DESTROY:
					{
						// Let children know the parent is destroying
						if (ForwardToChildren(hwnd, message, wparam, lparam, result, AllChildren))
							return true;

						// Remove this window from its parent.
						// Don't detach children, that happens when the Form/Control is destructed
						Parent(nullptr);

						// If we're the main app, quit the message loop (or fall back to
						// PostQuitMessage if no loop is registered).
						if (cp().m_main_wnd)
						{
							if (cp().m_msg_loop)
								cp().m_msg_loop->RequestQuit((int)m_dialog_result);
							else
								::PostQuitMessage((int)m_dialog_result);
						}

						// Return false so that WM_DESTROY is passed to the WndProc which
						// will unhook the thunk and null the 'm_hwnd'
						return false;
					}
					case WM_CTLCOLORDLG:
					{
						// Our background brush is only valid if we have valid wndclass info
						// otherwise, let the default handle it.
						result = LRESULT(m_wci.hbrBackground);
						return m_wci.m_atom != 0;
					}
					case WM_COMMAND:
					{
						auto ctrl_hwnd = HWND(lparam); // Only valid if sent from a control (i.e. not sent from a menu or accelerator)
						if (ctrl_hwnd == nullptr)
						{
							auto id = UINT(LoWord(wparam)); // The menu_item id or accelerator id
							auto src = UINT(HiWord(wparam)); // 0 = menu, 1 = accelerator, 2 = control-defined notification code
							return HandleMenu(id, src, ctrl_hwnd);
						}

						// Otherwise fall through to the control's WndProc
						return false;
					}
					case WM_SYSCOMMAND:
					{
						auto id = UINT(LoWord(wparam)); // The menu_item id or accelerator id
						if (id == IDC_PINWINDOW_OPT)
						{
							PinWindow(!PinWindow());
							return true;
						}
						if (id == SC_CLOSE)
						{
							m_dialog_result = EDialogResult::Close;
						}
						// Pass the WM_SYSCOMMAND to the WndProc
						return false;
					}
					case WM_WINDOWPOSCHANGED:
					{
						auto& wp = *rcast<WindowPos*>(lparam);

						// If shown for the first time, apply the start position
						if (AllSet(wp.flags, int(EWindowPos::ShowWindow)) &&
							cp().m_start_pos == EStartPosition::CentreParent)
						{
							// Calling 'CenterWindow' will recursively call WM_WINDOWPOSCHANGED
							cp().m_start_pos = EStartPosition::Manual;
							CenterWindow(m_parent.hwnd());
							return true;
						}

						// If we're a pinned window, record our offset from our target.
						// Test 'hwnd == m_hwnd' because we only record the offset when this window
						// moves relative to it's parent, not when the parent moves relative to this window.
						if (hwnd == m_hwnd && PinWindow())
							RecordPosOffset();

						// Let the WM_WINDOWPOSCHANGED message fall through to be handled by Control::WndProc
						return false;
					}
					case WM_DROPFILES:
					case WM_NOTIFY:
					case WM_MOUSEWHEEL:
					case WM_SETFOCUS:
					case WM_KILLFOCUS:
					case WM_TIMER:
					case WM_ENTERSIZEMOVE:
					case WM_EXITSIZEMOVE:
					case WM_CTLCOLORSTATIC:
					case WM_CTLCOLORBTN:
					case WM_CTLCOLOREDIT:
					case WM_CTLCOLORLISTBOX:
					case WM_CTLCOLORSCROLLBAR:
					{
						// Form messages that get here will be forwarded to child controls as well
						return Control::ProcessWindowMessage(hwnd, message, wparam, lparam, result);
					}
					default:
					{
						// By default, Form messages aren't forwarded to child controls.
						// Returning false will cause this message to get passed to the
						// Control::WndProc for this form.
						return false;
					}
				}
			}

			// Default main menu handler
			// 'item_id' - the menu item id or accelerator id
			// 'event_source' - 0 = menu, 1 = accelerator, 2 = control-defined notification code
			// 'ctrl_hwnd' - the control that sent the notification. Only valid when src == 2
			// Typically you'll only need 'menu_item_id' unless your accelerator ids
			// overlap your menu ids, in which case you'll need to check 'event_source'
			virtual bool HandleMenu(UINT item_id, UINT event_source, HWND ctrl_hwnd)
			{
				(void)event_source,ctrl_hwnd;
				switch (item_id)
				{
				case IDCLOSE:
					Close(EDialogResult::Close);
					return true;
				case IDCANCEL:
					Close(EDialogResult::Cancel);
					return true;
				case IDOK:
					Close(EDialogResult::Ok);
					return true;
				}
				return false;
			}

			// Adjust the size of this control relative to it's parent
			void ResizeToParent(Rect const& parent_client, bool repaint = false) override
			{
				if (!PinWindow()) return;
				Control::ResizeToParent(parent_client, repaint);
			}

			#pragma region Handlers
			void OnCreate(CreateStruct const& cs) override
			{
				Control::OnCreate(cs);

				// Set the window icon
				if (m_icon_sm != nullptr) Icon(m_icon_sm, false);
				if (m_icon_bg != nullptr) Icon(m_icon_bg, true);

				// If this form is running modally, disable the parent
				auto parent = ::GetParent(m_hwnd);
				if (m_modal && parent != nullptr)
					::EnableWindow(parent, false);
			}
			#pragma endregion
		};
		#pragma endregion

		#pragma region Standard Controls
		struct Label :Control
		{
			enum { DefW = 80, DefH = 23 };
			enum :DWORD { DefaultStyle   = (DefaultControlStyle | WS_GROUP | SS_LEFT | SS_NOPREFIX) & ~(WS_TABSTOP | WS_CLIPSIBLINGS) };
			enum :DWORD { DefaultStyleEx = DefaultControlStyleEx };
			static wchar_t const* WndClassName() { return L"STATIC"; }
			
			template <typename Derived = void>
			struct Params :Control::Params<not_void_t<Derived, Params<Derived>>>
			{
				using this_type = typename Params::this_type;
				using base_type = typename Control::Params<not_void_t<Derived, Params<Derived>>>;

				Params()
				{
					this->wndclass(WndClassName())
						.name("lbl")
						.style('=', DefaultStyle)
						.style_ex('=', DefaultStyleEx)
						.wh(Auto, Auto)
						.margin(3);
				}
				this_type& align(int ss)
				{
					// use one of SS_LEFT, SS_RIGHT, SS_CENTER, SS_LEFTNOWORDWRAP
					return base_type::style('-', SS_TYPEMASK).style('+', ss);
				}
				this_type& centre_v(bool on = true)
				{
					return base_type::style(on ? '+' : '-', SS_CENTERIMAGE);
				}
			};

			// Construct
			Label()
				:Label(Params<>())
			{}
			template <typename TParams, typename = typename TParams::this_type>
			explicit Label(TParams const& p)
				:Control(p)
			{}

			// Return the ideal size for this control. Includes padding, excludes margin
			Size PreferredSize() const override
			{
				auto sz = MeasureString(::IsWindow(m_hwnd) ? Text() : cp().m_text);
				auto& padding = cp().m_padding;
				return Size(
					sz.cx + padding.left - padding.right,
					sz.cy + padding.top - padding.bottom);
			}
		};
		struct Button :Control
		{
			enum { DefW = 75, DefH = 23 };
			enum :DWORD { DefaultStyle   = DefaultControlStyle | WS_TABSTOP | BS_PUSHBUTTON | BS_CENTER | BS_TEXT };
			enum :DWORD { DefaultStyleEx = DefaultControlStyleEx };
			static wchar_t const* WndClassName() { return L"BUTTON"; }

			using EType = gui::Image::EType;
			using EFit  = gui::Image::EFit;

			template <typename Derived = void>
			struct Params :Control::Params<not_void_t<Derived, Params<Derived>>>
			{
				using this_type = typename Params::this_type;

				ResId<>       m_img;
				EType         m_img_type;
				EFit          m_img_fit;
				EDialogResult m_dlg_result;

				Params()
					:m_img()
					,m_img_type()
					,m_img_fit(EFit::Zoom)
					,m_dlg_result(EDialogResult::None)
				{
					this->wndclass(WndClassName())
						.name("btn")
						.wh(DefW, DefH)
						.style('=', DefaultStyle)
						.style_ex('=', DefaultStyleEx)
						.margin(3);
				}
				this_type& def_btn(bool on = true)
				{
					return this->style(on ? '+' : '-', BS_DEFPUSHBUTTON);
				}
				this_type& chk_box(bool on = true)
				{
					return this->style(on ? '+' : '-', BS_AUTOCHECKBOX);
				}
				this_type& radio(bool on = true)
				{
					return this->style(on ? '+' : '-', BS_RADIOBUTTON);
				}
				this_type& image(ResId<> img, EType img_type, EFit img_fit = EFit::Zoom, bool show_text = false)
				{
					// BS_BITMAP == text only
					// BS_BITMAP + SendMsg(BM_SETIMAGE) == icon only
					// BS_TEXT + SendMsg(BM_SETIMAGE) == icon + text
					m_img = img;
					m_img_type = img_type;
					m_img_fit = img_fit;
					return this->style(img != nullptr && !show_text ? '+' : '-', BS_BITMAP);
				}
				this_type& dlg_result(EDialogResult dlg_result)
				{
					m_dlg_result = dlg_result;
					return this->me();
				}
			};

			gui::Image m_img;

			// Construct
			Button()
				:Button(Params<>())
			{}
			template <typename TParams, typename = typename TParams::this_type>
			explicit Button(TParams const& p)
				:Control(p)
				,m_img()
			{}

			// Get/Set the checked state
			bool Checked() const
			{
				return SendMsg<int>(BM_GETCHECK) == BST_CHECKED;
			}
			void Checked(bool checked)
			{
				auto is_checked = Checked();
				SendMsg<int>(BM_SETCHECK, checked ? BST_CHECKED : BST_UNCHECKED, 0);
				if (is_checked != checked) OnCheckedChanged();
			}

			// Get/Set the image
			// Get - 'img_type' should be either bitmap, icon, cursor (i.e. one of the standard IMAGE_BITMAP types).
			// Set - 'img_type' should be the exact type of the resource e.g. PNG, JPEG, Bitmap, etc. (i.e. types supported by GDI+)
			// The return type depends on 'img_type'. It can be one of: HBITMAP, HICON, HCURSOR
			void* Image(EType img_type = EType::Bitmap) const
			{
				assert(::IsWindow(m_hwnd));
				return SendMsg<void*>(BM_GETIMAGE, img_type, 0);
			}
			void Image(ResId<> id, EType img_type, EFit fit = EFit::Zoom)
			{
				auto rc = ClientRect(true);
				m_img = std::move(
					id.m_handle != nullptr ? gui::Image(id.m_handle, img_type, true) :
					id.m_res_id != nullptr ? gui::Image::Load(cp().m_hinst, id.m_res_id, img_type, fit, rc.width(), rc.height()) :
					gui::Image(nullptr, EType::Bitmap)); // Clear the reference to the bitmap object and set the button image to null

				if (::IsWindow(m_hwnd))
					SendMsg<int>(BM_SETIMAGE, m_img.m_type, m_img.m_obj);
			}

			// Get/Set the dialog result associated with this button
			EDialogResult DlgResult() const
			{
				return cp<Params<>>().m_dlg_result;
			}
			void DlgResult(EDialogResult r)
			{
				cp<Params<>>().m_dlg_result = r;
			}

			// Events
			// Click += [&](gui::Button&, gui::EmptyArgs const&){}
			EventHandler<Button&, EmptyArgs const&> Click;
			EventHandler<Button&, EmptyArgs const&> CheckedChanged;

			// Handlers
			virtual void OnClick()
			{
				Click(*this, EmptyArgs());

				// If this button as a non-None dialog result, send a WM_CLOSE to the top level control
				auto dlg_result = cp<Params<>>().m_dlg_result;
				if (dlg_result != EDialogResult::None)
				{
					auto top = TopLevelForm();
					if (top) top->Close(dlg_result);
				}
			}
			virtual void OnCheckedChanged()
			{
				CheckedChanged(*this, EmptyArgs());
			}

			// Perform the action as though the button was clicked
			void PerformClick()
			{
				SendMsg<int>(WM_COMMAND, MakeWParam(BN_CLICKED, cp().m_id.value), m_hwnd);
			}

			// Return the ideal size for this control. Includes padding, excludes margin
			Size PreferredSize() const override
			{
				auto sz = MeasureString(::IsWindow(m_hwnd) ? Text() : cp().m_text);
				auto& padding = cp().m_padding;
				return Size(
					sz.cx + padding.left - padding.right,
					sz.cy + padding.top - padding.bottom);
			}

			#pragma region Handlers
			LRESULT WndProc(UINT message, WPARAM wparam, LPARAM lparam) override
			{
				switch (message)
				{
				case WM_CREATE:
					{
						// Set the button image
						auto& p = cp<Params<>>();
						if (p.m_img != nullptr)
							Image(p.m_img, p.m_img_type, p.m_img_fit);

						break;
					}
				case WM_COMMAND:
					{
						// Need to test this before any handler possibly closes the window
						auto is_chk = AllSet(Style(), BS_AUTOCHECKBOX);
						switch (HiWord(wparam))
						{
						case BN_CLICKED:
							OnClick();
							if (is_chk) OnCheckedChanged();
							return S_OK;
						}
						break;
					}
				}
				return Control::WndProc(message, wparam, lparam);
			}
			#pragma endregion
		};
		struct TextBox :Control
		{
			enum { DefW = 80, DefH = 20 };
			enum :DWORD { DefaultStyle = DefaultControlStyle | WS_BORDER | WS_TABSTOP | ES_AUTOHSCROLL | ES_AUTOVSCROLL | ES_LEFT };
			enum :DWORD { DefaultStyleEx = DefaultControlStyleEx };
			static wchar_t const* WndClassName() { return L"EDIT"; }
			static HBRUSH WndBackground() { return (HBRUSH)::GetStockObject(WHITE_BRUSH); }// ::GetSysColorBrush(COLOR_WINDOW); }

			template <typename Derived = void>
			struct Params :Control::Params<not_void_t<Derived, Params<Derived>>>
			{
				using this_type = typename Params::this_type;
				Params()
				{
					this->wndclass(WndClassName())
						.name("edit")
						.wh(DefW, DefH)
						.style('=', DefaultStyle)
						.style_ex('=', DefaultStyleEx)
						.bk_col(0xFFFFFF)
						.fr_col(0)
						.margin(3);
				}
				this_type& align(int ss)
				{
					return this->style('-', ES_LEFT | ES_CENTER | ES_RIGHT).style('+', ss);
				}
				this_type& multiline(bool on = true)
				{
					return this->style(on ? '+' : '-', ES_MULTILINE);
				}
				this_type& upper_case(bool on = true)
				{
					return this->style(on ? '+' : '-', ES_UPPERCASE).style(on ? '-' : '+', ES_LOWERCASE);
				}
				this_type& lower_case(bool on = true)
				{
					return this->style(on ? '+' : '-', ES_LOWERCASE).style(on ? '-' : '+', ES_UPPERCASE);
				}
				this_type& password(bool on = true)
				{
					return this->style(on ? '+' : '-', ES_PASSWORD);
				}
				this_type& hide_sel(bool on = false)
				{
					return this->style(on ? '-' : '+', ES_NOHIDESEL);
				}
				this_type& read_only(bool on = true)
				{
					return this->style(on ? '+' : '-', ES_READONLY);
				}
				this_type& want_return(bool on = true)
				{
					return this->style(on ? '+' : '-', ES_WANTRETURN);
				}
				this_type& number(bool on = true)
				{
					return this->style(on ? '+' : '-', ES_NUMBER);
				}
			};

			// Construct
			TextBox()
				:TextBox(Params<>())
			{}
			template <typename TParams, typename = typename TParams::this_type> 
			explicit TextBox(TParams const& p)
				:Control(p)
			{}

			// The number of characters in the text
			int TextLength() const
			{
				assert(::IsWindow(m_hwnd));
				auto len = GETTEXTLENGTHEX{GTL_DEFAULT, CP_ACP};
				return SendMsg<int>(EM_GETTEXTLENGTHEX, &len);
			}

			// The number of lines of text
			int LineCount() const
			{
				assert(::IsWindow(m_hwnd));
				return int(::SendMessageW(m_hwnd, EM_GETLINECOUNT, 0, 0));
			}

			// The length (in characters) of the line containing the character at the given index
			// 'char_index = -1' means the number of *unselected* characters on the lines spanned by the selection.
			int LineLength(int char_index) const
			{
				assert(::IsWindow(m_hwnd));
				return int(::SendMessageW(m_hwnd, EM_LINELENGTH, WPARAM(char_index), 0));
			}

			// Gets the character index of the first character on the given line
			// 'line_index = -1' means the current line containing the caret
			int CharFromLine(int line_index) const
			{
				assert(::IsWindow(m_hwnd));
				return int(::SendMessageW(m_hwnd, EM_LINEINDEX, WPARAM(line_index), 0));
			}

			// Gets the index of the line that contains 'char_index'
			int LineFromChar(int char_index) const
			{
				assert(::IsWindow(m_hwnd));
				return int(::SendMessageW(m_hwnd, EM_EXLINEFROMCHAR, 0, LPARAM(char_index)));
			}

			// Get/Set the range of selected text
			RangeI Selection() const
			{
				assert(::IsWindow(m_hwnd));
				RangeI r;
				::SendMessageW(m_hwnd, EM_GETSEL, WPARAM(&r.beg), LPARAM(&r.end));
				return r;
			}
			void Selection(RangeI range, bool scroll_to_caret = true)
			{
				assert(::IsWindow(m_hwnd));
				::SendMessageW(m_hwnd, EM_SETSEL, range.beg, range.end);
				if (scroll_to_caret) ScrollToCaret();
			}

			// Select all text in the control
			void SelectAll(bool scroll_to_caret = true)
			{
				Selection(RangeI(0, -1), scroll_to_caret);
			}

			// Scroll to the caret position
			void ScrollToCaret()
			{
				assert(::IsWindow(m_hwnd));

				// There is a bug that means scrolling only works if the control has focus
				// There is a workaround however, using hide selection
				auto nohidesel = AllSet(Style(), ES_NOHIDESEL); // save the old state of ES_NOHIDESEL
				Style('+', ES_NOHIDESEL);                       // temporarily turn it on
				::SendMessageW(m_hwnd, EM_SCROLLCARET, 0, 0);   // scroll the caret
				Style(nohidesel ? '+' : '-', ES_NOHIDESEL);     // restore the setting
			}

			// Return the ideal size for this control. Includes padding, excludes margin
			Size PreferredSize() const override
			{
				return PreferredSize(0);
			}
			Size PreferredSize(int max_width) const
			{
				auto sz = MeasureString(::IsWindow(m_hwnd) ? Text() : cp().m_text, max_width);
				auto& padding = cp().m_padding;
				return Size(
					sz.cx + padding.left - padding.right,
					sz.cy + padding.top - padding.bottom);
			}

			// Events
			EventHandler<TextBox&, EmptyArgs const&> TextChanged;

			// Handlers
			virtual void OnTextChanged()
			{
				TextChanged(*this, EmptyArgs());
			}

			// Message map function
			LRESULT WndProc(UINT message, WPARAM wparam, LPARAM lparam) override
			{
				switch (message)
				{
				case WM_COMMAND:
					{
						switch (HiWord(wparam))
						{
						case EN_CHANGE:
							OnTextChanged();
							return S_OK;
						//case EN_UPDATE:
						//	ShowScrollBar()
						//	return S_OK;
						}
						break;
					}
				}
				return Control::WndProc(message, wparam, lparam);
			}
		};
		struct NumberBox :TextBox
		{
			enum class ENumberStyle { Integer, FloatingPoint, };

			template <typename Derived = void>
			struct Params :TextBox::Params<not_void_t<Derived, Params<Derived>>>
			{
				using this_type = typename Params::this_type;

				ENumberStyle m_num_style;
				int          m_radix;
				bool         m_lower_case;

				Params()
					:m_num_style(ENumberStyle::Integer)
					,m_radix(10)
					,m_lower_case(false)
				{}
				this_type& number_style(ENumberStyle style)
				{
					m_num_style = style;
					return this->me();
				}
				this_type& radix(int r)
				{
					m_radix = r;
					return this->me();
				}
				this_type& upper_case(bool on = true)
				{
					m_lower_case = on;
					return this->me();
				}
			};

			NumberBox()
				:NumberBox(Params<>())
			{}
			template <typename TParams, typename = typename TParams::this_type>
			explicit NumberBox(TParams const& p)
				:TextBox(p)
			{
				assert(p.m_num_style == ENumberStyle::Integer && "not supported");
			}

			// Get/Set the text content as an integer
			long long Value() const
			{
				auto text = Text();
				if (text.empty())
					return 0;

				errno = 0;
				auto val = ::wcstoll(text.c_str(), nullptr, cp<Params<>>().m_radix);
				if (errno == ERANGE) throw std::runtime_error("Value is out of range");
				if (errno != 0) throw std::runtime_error("Value is not a number");
				return val;
			}
			void Value(long long value)
			{
				wchar_t buf[128] = {};
				if (::_i64tow_s(value, buf, _countof(buf), cp<Params<>>().m_radix) != 0) throw std::runtime_error("Failed to convert number to text");
				if (cp<Params<>>().m_lower_case) _wcslwr(buf); else _wcsupr(buf);
				Text(buf);
			}

			// Get/Set the radix of the number field
			int Radix() const
			{
				return cp<Params<>>().m_radix;
			}
			void Radix(int radix)
			{
				if (radix == cp<Params<>>().m_radix) return;
				auto val = Value();
				cp<Params<>>().m_radix = radix;
				Value(val);
			}

			// Message map function
			LRESULT WndProc(UINT message, WPARAM wparam, LPARAM lparam) override
			{
				if (message == WM_CHAR)
				{
					auto radix = cp<Params<>>().m_radix;
					if (wparam == '-' || wparam == '+' || wparam == '.')
						{}
					else if (radix <= 10 &&
						!(wparam >= '0' && wparam < WPARAM('0'+radix)))
						return 0;
					else if (radix <= 36 &&
						!(wparam >= '0' && wparam <= '9') &&
						!(wparam >= 'a' && wparam < WPARAM('a'+radix-10)) &&
						!(wparam >= 'A' && wparam < WPARAM('A'+radix-10)))
						return 0;

					if (cp<Params<>>().m_lower_case)
						wparam = tolower(int(wparam));
					else
						wparam = toupper(int(wparam));
				}
				return TextBox::WndProc(message, wparam, lparam);
			}
		};
		struct ComboBox :Control
		{
			enum { DefW = 121, DefH = 21 };
			enum :DWORD { DefaultStyle   = DefaultControlStyle | WS_TABSTOP | CBS_DROPDOWNLIST | CBS_AUTOHSCROLL };
			enum :DWORD { DefaultStyleEx = DefaultControlStyleEx };
			static wchar_t const* WndClassName() { return L"COMBOBOX"; }

			template <typename Derived = void>
			struct Params :Control::Params<not_void_t<Derived, Params<Derived>>>
			{
				using this_type = typename Params::this_type;
				using base_type = typename Control::Params<not_void_t<Derived, Params<Derived>>>;

				Params()
				{
					this->wndclass(WndClassName())
						.name("combo")
						.wh(DefW, DefH)
						.style('=', DefaultStyle)
						.style_ex('=', DefaultStyleEx)
						.margin(3);
				}
				this_type& editable(bool on = true)
				{
					return base_type::style('-', CBS_SIMPLE | CBS_DROPDOWN | CBS_DROPDOWNLIST).style('+', on ? CBS_DROPDOWN : CBS_DROPDOWNLIST);
				}
				this_type& sorted(bool on = true)
				{
					return base_type::style(on ? '+' : '-', CBS_SORT);
				}
			};

			int m_prev_sel_index;

			// Construct
			ComboBox()
				:ComboBox(Params<>())
			{}
			template <typename TParams, typename = typename TParams::this_type>
			explicit ComboBox(TParams const& p)
				:Control(p)
				,m_prev_sel_index(-1)
			{}

			// Get the number of items in the combo box
			int Count() const
			{
				assert(::IsWindow(m_hwnd));
				auto c = int(::SendMessageW(m_hwnd, CB_GETCOUNT, 0, 0L));
				Check(c != CB_ERR, "Error retrieving combo box item count");
				return c;
			}

			// Get the item at index position 'index'
			string Item(int index) const
			{
				assert(::IsWindow(m_hwnd));

				string s;
				auto len = ::SendMessageW(m_hwnd, CB_GETLBTEXTLEN, index, 0);
				Check(len != CB_ERR, std::string("ComboBox: Invalid item index ").append(std::to_string(index)));
				if (len == 0) return s;

				s.resize(len);
				s.resize(::SendMessageW(m_hwnd, CB_GETLBTEXT, index, (LPARAM)&s[0]));
				return s;
			}

			// Get/Set the selected index
			int SelectedIndex() const
			{
				assert(::IsWindow(m_hwnd));
				auto index = int(::SendMessageW(m_hwnd, CB_GETCURSEL, 0, 0L));
				return index;
			}
			void SelectedIndex(int index)
			{
				assert(::IsWindow(m_hwnd));
				::SendMessageW(m_hwnd, CB_SETCURSEL, index, 0L);
			}

			// Get the selected item
			string SelectedItem() const
			{
				return Item(SelectedIndex());
			}

			// Remove all items from the combo drop down list
			void ResetContent()
			{
				assert(::IsWindow(m_hwnd));
				::SendMessageW(m_hwnd, CB_RESETCONTENT, 0, 0L);
			}

			// Add a string to the combo box drop down list
			int AddItem(wchar_t const* item)
			{
				assert(::IsWindow(m_hwnd));
				return int(::SendMessageW(m_hwnd, CB_ADDSTRING, 0, (LPARAM)item));
			}
			void AddItems(std::initializer_list<wchar_t const*> items)
			{
				for (auto i : items)
					AddItem(i);
			}

			// Events
			EventHandler<ComboBox&, EmptyArgs const&> DropDown;
			EventHandler<ComboBox&, SelectedIndexEventArgs const&> SelectedIndexChanged;

			#pragma region Handlers
			void OnCreate(CreateStruct const& cs) override
			{
				Control::OnCreate(cs);
				m_prev_sel_index = SelectedIndex();
			}
			LRESULT OnDropDown()
			{
				DropDown(*this, EmptyArgs());
				return S_OK;
			}
			LRESULT OnSelectedIndexChanged()
			{
				SelectedIndexChanged(*this, SelectedIndexEventArgs(SelectedIndex(), m_prev_sel_index));
				return S_OK;
			}
			LRESULT WndProc(UINT message, WPARAM wparam, LPARAM lparam) override
			{
				switch (message)
				{
				case WM_COMMAND:
					{
						switch (HiWord(wparam))
						{
						case CBN_DROPDOWN:
							return OnDropDown();
						case CBN_SELCHANGE:
							auto result = OnSelectedIndexChanged();
							m_prev_sel_index = SelectedIndex();
							return result;
						}
						break;
					}
				}
				return Control::WndProc(message, wparam, lparam);
			}
			#pragma endregion
		};
		struct ListView :Control
		{
			enum { DefW = 80, DefH = 80 };
			enum :DWORD { DefaultStyle   = DefaultControlStyle | LVS_ALIGNLEFT | LVS_SHOWSELALWAYS | LVS_EDITLABELS | LVS_NOLABELWRAP | LVS_REPORT };
			enum :DWORD { DefaultStyleEx = DefaultControlStyleEx | LVS_EX_HEADERDRAGDROP | LVS_EX_FULLROWSELECT };//| LVS_EX_TWOCLICKACTIVATE | LVS_EX_FLATSB | LVS_EX_AUTOSIZECOLUMNS };
			static wchar_t const* WndClassName() { return L"SysListView32"; }

			// Invalid item handle
			using HITEM = int;
			static HITEM const NoItem() { return -1; }

			// Modes for the list view
			enum class EViewType
			{
				Icon   = LVS_ICON,
				SmIcon = LVS_SMALLICON,
				List   = LVS_LIST,
				Report = LVS_REPORT,
			};

			// List Item
			struct ItemInfo :LVITEMW
			{
				ItemInfo(HITEM item, int mask_ = 0) :LVITEMW() { iItem = item; mask = mask_; }
				ItemInfo(wchar_t const* text_ = L"", int index_ = -1) :LVITEMW() { text(text_).index(index_); }
				ItemInfo& index(int i)                 { iItem = i; return *this; }
				ItemInfo& subitem(int i)               { iSubItem = i; return *this; }
				ItemInfo& text(wchar_t const* text_)   { mask |= LVIF_TEXT; pszText = const_cast<wchar_t*>(text_); return *this; }
				ItemInfo& image(int img_idx)           { mask |= LVIF_IMAGE; iImage = img_idx; return *this; }
				ItemInfo& state(int state_, int mask_) { mask |= LVIF_STATE; LVITEMW::state = state_; stateMask = mask_; return *this; }
				ItemInfo& user(void* ctx)              { mask |= LVIF_PARAM; lParam = LPARAM(ctx); return *this; }
			};

			// Details view column
			struct ColumnInfo :LVCOLUMNW
			{
				ColumnInfo(wchar_t const* text_ = L"", int fmt = LVCFMT_LEFT) :LVCOLUMNW()
				{
					constexpr int margin = 8;
					auto sz = gui::MeasureString(text_) + Size(2*margin, 2*margin);
					text(text_).format(fmt).min_width(10).def_width(80).ideal_width(sz.cx).width(sz.cx);
				}
				ColumnInfo& text(wchar_t const* text)  { mask |= LVCF_TEXT; pszText = const_cast<wchar_t *>(text); return *this; }
				ColumnInfo& width(int w)               { mask |= LVCF_WIDTH; cx = w; return *this; }
				ColumnInfo& format(int lvcfmt)         { mask |= LVCF_FMT; fmt = lvcfmt; return *this; }
				ColumnInfo& subitem(int i)             { mask |= LVCF_SUBITEM; iSubItem = i; return *this; }
				ColumnInfo& image(int img_idx)         { mask |= LVCF_IMAGE; iImage = img_idx; return *this; }
				ColumnInfo& min_width(int w)           { mask |= LVCF_MINWIDTH; cxMin = w; return *this; }
				ColumnInfo& def_width(int w)           { mask |= LVCF_DEFAULTWIDTH; cxDefault = w; return *this; }
				ColumnInfo& ideal_width(int w)         { mask |= LVCF_IDEALWIDTH; cxIdeal = w; return *this; }
			};

			// Creation parameters
			template <typename Derived = void>
			struct Params :Control::Params<not_void_t<Derived, Params<Derived>>>
			{
				using this_type = typename Params::this_type;
				std::vector<ColumnInfo> m_columns;

				Params()
					:m_columns()
				{
					this->wndclass(WndClassName())
						.name("listview")
						.wh(DefW, DefH)
						.style('=', DefaultStyle)
						.style_ex('=', DefaultStyleEx)
						.mode(EViewType::Report)
						.dbl_buffer();
				}
				this_type& mode(EViewType mode)
				{
					return this->style('-', LVS_TYPEMASK).style('+', DWORD(mode) & LVS_TYPEMASK);
				}
				this_type& report()
				{
					return this->style('+', LVS_REPORT);
				}
				this_type& no_hdr_sort()
				{
					return this->style('+', LVS_NOSORTHEADER);
				}
				this_type& columns(std::vector<std::wstring_view> const& names)
				{
					m_columns.clear();
					for (auto& name : names) add_column(ColumnInfo(name.data()));
					return this->me();
				}
				this_type& add_column(ColumnInfo const& cinfo)
				{
					m_columns.push_back(cinfo);
					return this->me();
				}
			};

			// Construct
			ListView()
				:ListView(Params<>())
			{}
			template <typename TParams, typename = typename TParams::this_type>
			explicit ListView(TParams const& p)
				:Control(p)
			{}

			#pragma region Accessors

			// Get/Set view type.
			EViewType ViewType() const
			{
				assert(::IsWindow(m_hwnd));
				return EViewType(Style() & LVS_TYPEMASK);
			}
			void ViewType(EViewType vt)
			{
				assert(::IsWindow(m_hwnd));
				Style('-', LVS_TYPEMASK).Style('+', DWORD(vt) & LVS_TYPEMASK);
			}

			// Remove all items
			void Clear()
			{
				assert(::IsWindow(m_hwnd));
				Check((BOOL)::SendMessageW(m_hwnd, LVM_DELETEALLITEMS, 0, 0L), "Delete all list items failed");
			}

			// Get the number of elements in the list
			size_t ItemCount() const
			{
				assert(::IsWindow(m_hwnd));
				return (size_t)::SendMessageW(m_hwnd, LVM_GETITEMCOUNT, 0, 0L);
			}

			// Get the number of selected list items
			size_t SelectedCount() const
			{
				assert(::IsWindow(m_hwnd));
				return (size_t)::SendMessageW(m_hwnd, LVM_GETSELECTEDCOUNT, 0, 0L);
			}

			// Returns the next item with state matching 'flags' (e.g. LVNI_SELECTED) or -1
			HITEM NextItem(int flags, HITEM item = -1) const
			{
				assert(::IsWindow(m_hwnd));
				return (int)::SendMessageW(m_hwnd, LVM_GETNEXTITEM, item, MakeLParam(flags, 0));
			}

			// Add a row to the list
			HITEM InsertItem(ItemInfo const& info)
			{
				assert(::IsWindow(m_hwnd));
				return (HITEM)::SendMessageW(m_hwnd, LVM_INSERTITEMW, 0, (LPARAM)&info);
			}

			// Remove an item from the list
			bool DeleteItem(HITEM item)
			{
				assert(::IsWindow(m_hwnd));
				return ::SendMessageW(m_hwnd, LVM_DELETEITEM, item, 0L) != 0;
			}

			// Get/Set an item. Construct 'info' with the item handle and mask for the data you want to get/set
			ItemInfo Item(ItemInfo info) const
			{
				assert(::IsWindow(m_hwnd));
				Check((BOOL)::SendMessageW(m_hwnd, LVM_GETITEMW, 0, (LPARAM)&info), "Get list item failed");
				return info;
			}
			void Item(ItemInfo info)
			{
				assert(::IsWindow(m_hwnd));
				Check((BOOL)::SendMessageW(m_hwnd, LVM_SETITEMW, 0, (LPARAM)&info), "Set list item failed");
			}

			// Get/Set the state of an item
			UINT ItemState(HITEM item, UINT state_mask) const
			{
				assert(::IsWindow(m_hwnd));
				return (UINT)::SendMessageW(m_hwnd, LVM_GETITEMSTATE, (WPARAM)item, (LPARAM)state_mask) & state_mask;
			}
			void ItemState(HITEM item, int state, int state_mask)
			{
				assert(::IsWindow(m_hwnd));
				ItemInfo info = ItemInfo(item).state(state, state_mask);
				Check((BOOL)::SendMessageW(m_hwnd, LVM_SETITEMSTATE, item, (LPARAM)&info), "Set list item state failed");
			}

			// Scroll an item into view
			void EnsureVisible(HITEM item, bool partial_ok)
			{
				assert(::IsWindow(m_hwnd));
				Check((BOOL)::SendMessageW(m_hwnd, LVM_ENSUREVISIBLE, (WPARAM)item, MakeLParam(partial_ok, 0)), "Ensure list item is visible failed");
			}

			// Get/Set user data on the item
			template <typename T> T* UserData(HITEM item) const
			{
				assert(item != NoItem());
				return reinterpret_cast<T*>(Item(ItemInfo(item, LVIF_PARAM)).lParam);
			}
			void UserData(HITEM item, void* ctx)
			{
				Item(ItemInfo(item).user(ctx));
			}

			#pragma endregion

			#pragma region Columns
			// Get the number of columns in the list
			size_t ColumnCount() const
			{
				assert(::IsWindow(m_hwnd));
				auto hdr = (HWND)::SendMessageW(m_hwnd, LVM_GETHEADER, 0, 0L);
				return (size_t)::SendMessageW(hdr, HDM_GETITEMCOUNT, 0, 0L);
			}

			// Insert a column into the list
			void InsertColumn(int idx, ColumnInfo const& column)
			{
				assert(::IsWindow(m_hwnd));
				Check(::SendMessageW(m_hwnd, LVM_INSERTCOLUMNW, idx, (LPARAM)&column) != -1, "Insert column failed.");
			}

			// Get/Set the width of a column
			int ColumnWidth(int col) const
			{
				assert(::IsWindow(m_hwnd));
				return (int)::SendMessageW(m_hwnd, LVM_GETCOLUMNWIDTH, col, 0L);
			}
			void ColumnWidth(int col, int width) // use 'LVSCW_AUTOSIZE'
			{
				assert(::IsWindow(m_hwnd));
				Check((BOOL)::SendMessageW(m_hwnd, LVM_SETCOLUMNWIDTH, col, MakeLParam(width, 0)), "Set list column width failed");
			}
			#pragma endregion

			// Events
			struct ItemChangedEventArgs :NMLISTVIEW ,EmptyArgs
			{
				ItemChangedEventArgs(NMLISTVIEW const& nmv)
					:NMLISTVIEW(nmv)
				{}
			};
			struct ItemChangingEventArgs :ItemChangedEventArgs ,CancelEventArgs
			{
				ItemChangingEventArgs(NMLISTVIEW const& nmv, bool cancel = false)
					:ItemChangedEventArgs(nmv)
					,CancelEventArgs(cancel)
				{}
			};

			// [&](gui::ListView&, gui::ListView::ItemChangedEventArgs const&){}
			EventHandler<ListView&, ItemChangingEventArgs const&> ItemChanging;
			EventHandler<ListView&, ItemChangedEventArgs const&> ItemChanged;
			EventHandler<ListView&, ItemChangingEventArgs const&> SelectionChanging;
			EventHandler<ListView&, ItemChangedEventArgs const&> SelectionChanged;

			// Handlers
			virtual void OnItemChanging(ItemChangingEventArgs& args)
			{
				ItemChanging(*this, args);
			}
			virtual void OnItemChanged(ItemChangedEventArgs& args)
			{
				ItemChanged(*this, args);
			}
			virtual void OnSelectionChanging(ItemChangingEventArgs& args)
			{
				SelectionChanging(*this, args);
			}
			virtual void OnSelectionChanged(ItemChangedEventArgs& args)
			{
				SelectionChanged(*this, args);
			}

			// Message map function
			bool ProcessWindowMessage(HWND parent_hwnd, UINT message, WPARAM wparam, LPARAM lparam, LRESULT& result) override
			{
				switch (message)
				{
					case WM_NOTIFY:
					{
						auto notification_hdr = (LPNMHDR)lparam;
						if (notification_hdr->hwndFrom == m_hwnd)
						{
							auto hdr = reinterpret_cast<NMLISTVIEW*>(notification_hdr);
							switch (hdr->hdr.code)
							{
								case LVN_ITEMCHANGING:
								{
									ItemChangingEventArgs args(*hdr);
									OnItemChanging(args);

									// If the selection has changed raise a special event for that case
									if ((hdr->uNewState ^ hdr->uOldState) & LVIS_SELECTED)
										OnSelectionChanging(args);

									result = args.m_cancel ? TRUE : FALSE;
									return true;
								}
								case LVN_ITEMCHANGED:
								{
									ItemChangedEventArgs args(*hdr);
									OnItemChanged(args);

									// If the selection has changed raise a special event for that case
									if ((hdr->uNewState ^ hdr->uOldState) & LVIS_SELECTED)
										OnSelectionChanged(args);

									return true;
								}
							}
						}
						break;
					}
				}
				return Control::ProcessWindowMessage(parent_hwnd, message, wparam, lparam, result);
			}
		
			// Add columns once the control is created
			void OnCreate(CreateStruct const& cs) override
			{
				Control::OnCreate(cs);

				int index = 0;
				for (auto& col : cp<Params<>>().m_columns)
					InsertColumn(index++, col);
			}
		};
		struct TreeView :Control
		{
			enum { DefW = 80, DefH = 80 };
			enum :DWORD { DefaultStyle   = DefaultControlStyle | TVS_EDITLABELS | TVS_HASBUTTONS | TVS_HASLINES | TVS_LINESATROOT | TVS_DISABLEDRAGDROP | TVS_SHOWSELALWAYS | TVS_FULLROWSELECT | TVS_NOSCROLL };
			enum :DWORD { DefaultStyleEx = DefaultControlStyleEx };
			static wchar_t const* WndClassName() { return L"SysTreeView32"; }
		
			template <typename Derived = void>
			struct Params :Control::Params<not_void_t<Derived, Params<Derived>>>
			{
				using this_type = typename Params::this_type;
				Params()
				{
					this->wndclass(WndClassName())
						.name("tree-view")
						.wh(DefW, DefH)
						.style('=', DefaultStyle)
						.style_ex('=', DefaultStyleEx);
				}
			};

			// Invalid item handle
			using HITEM = ::HTREEITEM;
			static HITEM const NoItem() { return nullptr; }

			// Next item codes
			enum EItem
			{
				Root         = TVGN_ROOT,
				Next         = TVGN_NEXT,
				Prev         = TVGN_PREVIOUS,
				Parent       = TVGN_PARENT,
				Child        = TVGN_CHILD,
				FirstVisible = TVGN_FIRSTVISIBLE,
				NextVisible  = TVGN_NEXTVISIBLE,
				PrevVisible  = TVGN_PREVIOUSVISIBLE,
				LastVisible  = TVGN_LASTVISIBLE,
				NextSelected = 0x000B, //TVGN_NEXTSELECTED,
				DropHilite   = TVGN_DROPHILITE,
				Caret        = TVGN_CARET,
			};

			// Expand or collapse codes
			enum EExpand
			{
				Collapse = TVE_COLLAPSE,
				Expand = TVE_EXPAND,
				Toggle = TVE_TOGGLE,

				// Flags that can be combined with above
				ExpandPartial = TVE_EXPANDPARTIAL,
				CollapseReset = TVE_COLLAPSERESET,
			};

			// Tree item
			struct ItemInfo :TVITEMEXW
			{
				ItemInfo(wchar_t const* text_ = L"") :TVITEMEXW() { text(text_); }
				ItemInfo(HITEM item, int mask_ = 0) :TVITEMEXW() { hItem = item; mask = mask_; }
				ItemInfo& text(wchar_t const* text_)   { mask |= TVIF_TEXT; pszText = const_cast<wchar_t*>(text_); return *this; }
				ItemInfo& image(int img_idx)           { mask |= TVIF_IMAGE; iImage = img_idx; return *this; }
				ItemInfo& image_sel(int img_idx)       { mask |= TVIF_SELECTEDIMAGE; iSelectedImage = img_idx; return *this; }
				ItemInfo& state(int state_, int mask_) { mask |= TVIF_STATE; TVITEMEXW::state = state_; stateMask = mask_; return *this; }
				ItemInfo& user(void* ctx)              { mask |= TVIF_PARAM; lParam = LPARAM(ctx); return *this; }
			};

			// Construct
			TreeView()
				:TreeView(Params<>())
			{}
			template <typename TParams, typename = typename TParams::this_type>
			explicit TreeView(TParams const& p)
				:Control(p)
			{}

			// Remove all items
			void Clear()
			{
				assert(::IsWindow(m_hwnd));
				Check((BOOL)::SendMessageW(m_hwnd, TVM_DELETEITEM, 0, (LPARAM)TVI_ROOT), "Delete all tree items failed");
			}

			// Return the root, next sibling, previous sibling, child, parent, etc item relative to 'item'
			// 'item' can be null for Root, and FirstXYZ codes.
			HITEM NextItem(EItem code, HITEM item = nullptr) const
			{
				assert(::IsWindow(m_hwnd));
				return (HITEM)::SendMessageW(m_hwnd, TVM_GETNEXTITEM, (WPARAM)code, (LPARAM)item);
			}

			// Insert an item into the tree
			HITEM InsertItem(ItemInfo const& info, HITEM parent, HITEM insert_after)
			{
				assert(::IsWindow(m_hwnd));
				auto ins = TVINSERTSTRUCTW{};
				ins.hParent = parent;
				ins.hInsertAfter = insert_after;
				ins.itemex = info;
				return (HITEM)::SendMessageW(m_hwnd, TVM_INSERTITEMW, 0, (LPARAM)&ins);
			}

			// Delete an item and it's children from the tree
			bool DeleteItem(HITEM item)
			{
				assert(::IsWindow(m_hwnd));
				return ::SendMessageW(m_hwnd, TVM_DELETEITEM, 0, (LPARAM)item) != 0;
			}

			// Get/Set an item. Construct 'info' with the item handle and mask for the data you want to get/set
			ItemInfo Item(ItemInfo info) const
			{
				assert(::IsWindow(m_hwnd));
				Check((BOOL)::SendMessageW(m_hwnd, TVM_GETITEMW, 0, (LPARAM)&info), "Get tree item failed");
				return info;
			}
			void Item(ItemInfo info)
			{
				assert(::IsWindow(m_hwnd));
				Check((BOOL)::SendMessageW(m_hwnd, TVM_SETITEMW, 0, (LPARAM)&info), "Set tree item failed");
			}

			// Get/Set the state of an item
			UINT ItemState(HITEM item, UINT state_mask) const
			{
				assert(::IsWindow(m_hwnd));
				return (UINT)::SendMessageW(m_hwnd, TVM_GETITEMSTATE, (WPARAM)item, (LPARAM)state_mask) & state_mask;
			}
			void ItemState(HITEM item, int state, int state_mask)
			{
				Item(ItemInfo(item).state(state, state_mask));
			}

			// Scroll an item into view
			void EnsureVisible(HITEM item)
			{
				assert(::IsWindow(m_hwnd));
				Check((BOOL)::SendMessageW(m_hwnd, TVM_ENSUREVISIBLE, 0, (LPARAM)item), "Ensure tree item is visible failed");
			}

			// Get/Set user data on the item
			template <typename T> T* UserData(HITEM item) const
			{
				assert(item != NoItem());
				return reinterpret_cast<T*>(Item(ItemInfo(item, TVIF_PARAM)).lParam);
			}
			void UserData(HITEM item, void* ctx)
			{
				Item(ItemInfo(item).user(ctx));
			}

			// Expand or collapse a node in the tree
			void ExpandItem(HITEM item, EExpand code)
			{
				assert(::IsWindow(m_hwnd));
				Check((BOOL)::SendMessageW(m_hwnd, TVM_EXPAND, code, (LPARAM)item), "Expand tree node failed");
			}

			// Message map function
			bool ProcessWindowMessage(HWND parent_hwnd, UINT message, WPARAM wparam, LPARAM lparam, LRESULT& result) override
			{
				//switch (message)
				//{
				//case WM_WINDOWPOSCHANGED:
				//	{
				//		OutputDebugStringA(FmtS("Tree %s resize\n", m_name.c_str()));
				//		break;
				//	}
				//}
				return Control::ProcessWindowMessage(parent_hwnd, message, wparam, lparam, result);
			}
		};
		struct ProgressBar :Control
		{
			enum { DefW = 100, DefH = 23 };
			enum :DWORD { DefaultStyle   = (DefaultControlStyle | PBS_SMOOTH) & ~WS_TABSTOP };
			enum :DWORD { DefaultStyleEx = DefaultControlStyleEx };
			static wchar_t const* WndClassName() { return L"msctls_progress32"; }

			template <typename Derived = void>
			struct Params :Control::Params<not_void_t<Derived, Params<Derived>>>
			{
				using this_type = typename Params::this_type;
				Params()
				{
					this->wndclass(WndClassName())
						.name("progress")
						.wh(DefW, DefH)
						.style('=', DefaultStyle)
						.style_ex('=', DefaultStyleEx);
				}
			};

			ProgressBar()
				:ProgressBar(Params<>())
			{}
			template <typename TParams, typename = typename TParams::this_type>
			explicit ProgressBar(TParams const& p)
				:Control(p)
			{}

			// Get/Set the progress bar position
			int Pos() const
			{
				assert(::IsWindow(m_hwnd));
				return int(::SendMessageW(m_hwnd, PBM_GETPOS, 0, 0L));
			}
			int Pos(int pos)
			{
				assert(::IsWindow(m_hwnd));
				return int(short(LoWord(::SendMessageW(m_hwnd, PBM_SETPOS, pos, 0L))));
			}

			// Move the bar position by a delta
			int OffsetPos(int delta)
			{
				assert(::IsWindow(m_hwnd));
				return int(short(LoWord(::SendMessageW(m_hwnd, PBM_DELTAPOS, delta, 0L))));
			}

			// Get/Set the progress range
			RangeI Range() const
			{
				assert(::IsWindow(m_hwnd));
				PBRANGE range = {};
				::SendMessageW(m_hwnd, PBM_GETRANGE, TRUE, (LPARAM)&range);
				return RangeI(range.iLow, range.iHigh);
			}
			void Range(RangeI rng)
			{
				assert(::IsWindow(m_hwnd));
				::SendMessageW(m_hwnd, PBM_SETRANGE32, rng.beg, rng.end);
			}
			void Range(int min, int max)
			{
				Range(RangeI(min,max));
			}

			// Get/Set marquee mode
			bool Marquee() const
			{
				assert(::IsWindow(m_hwnd));
				return (Style() & PBS_MARQUEE) != 0;
			}
			bool Marquee(bool marquee, UINT update_time = 0U)
			{
				assert(::IsWindow(m_hwnd));
				return ::SendMessageW(m_hwnd, PBM_SETMARQUEE, WPARAM(marquee), LPARAM(update_time)) != 0;
			}

			// Get/Set the step size
			int StepSize() const
			{
				assert(::IsWindow(m_hwnd));
				return int(::SendMessageW(m_hwnd, PBM_GETSTEP, 0, 0L));
			}
			int StepSize(int step_size)
			{
				assert(::IsWindow(m_hwnd));
				return int(short(LoWord(::SendMessageW(m_hwnd, PBM_SETSTEP, step_size, 0L))));
			}

			// Get/Set the bar colour
			COLORREF BarColor() const
			{
				assert(::IsWindow(m_hwnd));
				return COLORREF(::SendMessageW(m_hwnd, PBM_GETBARCOLOR, 0, 0L));
			}
			COLORREF BarColor(COLORREF clr)
			{
				assert(::IsWindow(m_hwnd));
				return COLORREF(::SendMessageW(m_hwnd, PBM_SETBARCOLOR, 0, (LPARAM)clr));
			}

			// Get/Set the bar background colour
			COLORREF BarBkgdColor() const
			{
				assert(::IsWindow(m_hwnd));
				return COLORREF(::SendMessageW(m_hwnd, PBM_GETBKCOLOR, 0, 0L));
			}
			COLORREF BarBkgdColor(COLORREF clr)
			{
				assert(::IsWindow(m_hwnd));
				return COLORREF(::SendMessageW(m_hwnd, PBM_SETBKCOLOR, 0, (LPARAM)clr));
			}

			// Get/Set the state
			int State() const
			{
				assert(::IsWindow(m_hwnd));
				return int(::SendMessageW(m_hwnd, PBM_GETSTATE, 0, 0L));
			}
			int State(int state)
			{
				assert(::IsWindow(m_hwnd));
				return int(::SendMessageW(m_hwnd, PBM_SETSTATE, state, 0L));
			}

			// Step the bar
			int StepIt()
			{
				assert(::IsWindow(m_hwnd));
				return int(short(LoWord(::SendMessageW(m_hwnd, PBM_STEPIT, 0, 0L))));
			}

			// Events
			EventHandler<ProgressBar&, EmptyArgs const&> ProgressUpdate;

			// Handlers
			virtual void OnProgressUpdate()
			{
				ProgressUpdate(*this, EmptyArgs());
			}

			// Message map function
			bool ProcessWindowMessage(HWND parent_hwnd, UINT message, WPARAM wparam, LPARAM lparam, LRESULT& result) override
			{
				/*switch (message)
				{
				case WM_PROGRESS_UPDATE:
					if (LoWord(wparam) != m_id.value) break;
					switch (HiWord(wparam))
					{
					case BN_CLICKED:
						OnClick();
						break;
					}
				}*/
				return Control::ProcessWindowMessage(parent_hwnd, message, wparam, lparam, result);
			}
		};
		struct Panel :Control
		{
			enum { DefW = 80, DefH = 80 };
			enum :DWORD { DefaultStyle   = DefaultControlStyle & ~(WS_CLIPCHILDREN) };
			enum :DWORD { DefaultStyleEx = DefaultControlStyleEx | WS_EX_CONTROLPARENT };
			static wchar_t const* WndClassName() { return L"gui::Panel"; }

			template <typename Derived = void>
			struct Params :Control::Params<not_void_t<Derived, Params<Derived>>>
			{
				using this_type = typename Params::this_type;
				Params()
				{
					this->wndclass(RegisterWndClass<Panel>())
						.name("panel")
						.wh(DefW, DefH)
						.style('=', DefaultStyle)
						.style_ex('=', DefaultStyleEx);
				}
			};

			Panel()
				:Panel(Params<>())
			{}
			template <typename TParams, typename = typename TParams::this_type>
			explicit Panel(TParams const& p)
				:Control(p)
			{}
		};
		struct GroupBox :Control
		{
			enum { DefW = 80, DefH = 80 };
			enum :DWORD { DefaultStyle   = DefaultControlStyle | BS_GROUPBOX };
			enum :DWORD { DefaultStyleEx = DefaultControlStyleEx | WS_EX_CONTROLPARENT };
			static wchar_t const* WndClassName() { return L"BUTTON"; } // yes, group-box's use the button window class

			template <typename Derived = void>
			struct Params :Control::Params<not_void_t<Derived, Params<Derived>>>
			{
				using this_type = typename Params::this_type;
				Params()
				{
					this->wndclass(WndClassName())
						.name("grp")
						.wh(DefW, DefH)
						.style('=', DefaultStyle)
						.style_ex('=', DefaultStyleEx);
				}
			};

			GroupBox()
				:GroupBox(Params<>())
			{}
			template <typename TParams, typename = typename TParams::this_type>
			explicit GroupBox(TParams const& p)
				:Control(p)
			{}
		};
		struct RichTextBox :TextBox
		{
			enum :DWORD { DefaultStyle   = (TextBox::DefaultStyle | WS_VSCROLL | WS_HSCROLL | ES_MULTILINE | ES_WANTRETURN) & ~(WS_BORDER) };
			enum :DWORD { DefaultStyleEx = (TextBox::DefaultStyleEx) & ~(WS_EX_STATICEDGE | WS_EX_CLIENTEDGE) };
			static wchar_t const* WndClassName()
			{
				// LoadLibrary is reference counted, so only call it once
				static HMODULE richedit_lib = ::LoadLibraryW(L"msftedit.dll");
				return richedit_lib ? L"RICHEDIT50W" : L"RICHEDIT20W";
			}

			template <typename Derived = void>
			struct Params :TextBox::Params<not_void_t<Derived, Params<Derived>>>
			{
				using this_type = typename Params::this_type;
				using base_type = typename TextBox::Params<not_void_t<Derived, Params<Derived>>>;

				bool m_word_wrap;
				bool m_detect_urls;

				Params()
					:m_detect_urls(false)
					,m_word_wrap(false)
				{
					this->wndclass(WndClassName())
						.name("rtb")
						.style('=', DefaultStyle)
						.style_ex('=', DefaultStyleEx);
				}
				this_type& border(bool on = true)
				{
					return base_type::style_ex(on?'+':'-', WS_EX_STATICEDGE);
				}
				this_type& word_wrap(bool on = true)
				{
					m_word_wrap = on;
					return this->me();
				}
				this_type& detect_urls(bool on = true)
				{
					m_detect_urls = on;
					return this->me();
				}
			};

			RichTextBox()
				:RichTextBox(Params<>())
			{}
			template <typename TParams, typename = typename TParams::this_type>
			explicit RichTextBox(TParams const& p)
				:TextBox(p)
			{}
			#if 0
			m_message.LinkClicked += (s,a) =>
			{
				try { System.Diagnostics.Process.Start("explorer.exe", a.LinkText); }
				catch (Exception ex)
				{
					MsgBox.Show(Owner, "Failed to navigate to link\r\nReason: " + ex.Message, "Link Failed", MessageBoxButtons.OK);
				}
			};
			#endif
			#pragma region Handlers
			LRESULT WndProc(UINT message, WPARAM wparam, LPARAM lparam) override
			{
				switch (message)
				{
				case WM_CREATE:
					{
						auto& p = cp<Params<>>();
						#if _RICHEDIT_VER >= 0x0800
						if (p.m_detect_urls) SendMsg<int>(EM_AUTOURLDETECT, AURL_ENABLEURL, 0);
						#endif
						if (p.m_word_wrap)   SendMsg<int>(EM_SETTARGETDEVICE, 0, 0);
						break;
					}
				}
				return TextBox::WndProc(message, wparam, lparam);
			}
			#pragma endregion
		};
		struct ImageBox :Control
		{
			enum { DefW = 23, DefH = 23 };
			enum :DWORD { DefaultStyle   = (DefaultControlStyle | SS_CENTERIMAGE) & ~(WS_CLIPSIBLINGS) };
			enum :DWORD { DefaultStyleEx = DefaultControlStyleEx };
			static wchar_t const* WndClassName() { return L"STATIC"; }

			using EType = gui::Image::EType;
			using EFit  = gui::Image::EFit;

			template <typename Derived = void>
			struct Params :Control::Params<not_void_t<Derived, Params<Derived>>>
			{
				using this_type = typename Params::this_type;

				ResId<> m_img;
				EType   m_img_type;
				EFit    m_img_fit;

				Params()
					:m_img()
					,m_img_type()
					,m_img_fit(EFit::Zoom)
				{
					this->wndclass(WndClassName())
						.name("img")
						.wh(DefW, DefH)
						.style('=', DefaultStyle)
						.style_ex('=', DefaultStyleEx)
						.margin(3);
				}
				this_type& image(ResId<> img, EType img_type, EFit img_fit = EFit::Zoom)
				{
					m_img = img;
					m_img_type = img_type;
					m_img_fit = img_fit;
					return this->style('-', SS_TYPEMASK).style('+',
						img_type == EType::Icon ? SS_ICON :
						img_type == EType::EnhMetaFile ? SS_ENHMETAFILE :
						SS_BITMAP);
				}
			};

			gui::Image m_img;

			// Construct
			ImageBox()
				:ImageBox(Params<>())
			{}
			template <typename TParams, typename = typename TParams::this_type>
			explicit ImageBox(TParams const& p)
				:Control(p)
				,m_img()
			{}

			// Get/Set the image
			// Note: to load an OEM resource (cursor, icon, etc) you need to specify 'hinst=nullptr' and 'flags|=LR_SHARED'
			gui::Image const& Image() const
			{
				return m_img;
			}
			void Image(ResId<> id, EType img_type, EFit fit = EFit::Zoom, UINT flags = LR_DEFAULTCOLOR|LR_DEFAULTSIZE)
			{
				auto rc = ClientRect(true);
				Image(cp().m_hinst, id, img_type, fit, rc.width(), rc.height(), flags);
			}
			void Image(HINSTANCE hinst, ResId<> id, EType img_type, EFit fit = EFit::Zoom, int cx = 0, int cy = 0, UINT flags = LR_DEFAULTCOLOR|LR_DEFAULTSIZE)
			{
				m_img = std::move(
					id.m_res_id != nullptr ? gui::Image::Load(hinst, id.m_res_id, img_type, fit, cx, cy, flags) :
					id.m_handle != nullptr ? gui::Image(id.m_handle, img_type, true) :
					gui::Image(nullptr, EType::Bitmap)); // Clear the reference to the bitmap object and set the button image to null

				if (::IsWindow(m_hwnd))
				{
					// STM_SETICON is needed to display transparent parts of the icon
					if (m_img.m_type == EType::Icon || m_img.m_type == EType::Cursor)
						Style('-', SS_TYPEMASK).Style('+', SS_ICON).SendMsg<int>(STM_SETICON, m_img.m_obj, 0);
					else
						Style('-', SS_TYPEMASK).Style('+', SS_BITMAP).SendMsg<int>(STM_SETIMAGE, m_img.m_type, m_img.m_obj);
				}
			}

			#pragma region Handlers
			LRESULT WndProc(UINT message, WPARAM wparam, LPARAM lparam) override
			{
				switch (message)
				{
				case WM_CREATE:
					{
						// Set the button image
						auto& p = cp<Params<>>();
						if (p.m_img != nullptr)
							Image(p.m_img, p.m_img_type, p.m_img_fit);
						break;
					}
				}
				return Control::WndProc(message, wparam, lparam);
			}
			#pragma endregion
		};
		struct StatusBar :Control
		{
			enum :DWORD { DefaultStyle   = DefaultControlStyle | WS_CLIPCHILDREN | WS_CLIPSIBLINGS | SBARS_SIZEGRIP };
			enum :DWORD { DefaultStyleEx = DefaultControlStyleEx };
			static wchar_t const* WndClassName() { return STATUSCLASSNAMEW; }

			template <typename Derived = void>
			struct Params :Control::Params<not_void_t<Derived, Params<Derived>>>
			{
				using this_type = typename Params::this_type;

				std::initializer_list<int> m_parts;

				Params()
					:m_parts()
				{
					this->wndclass(WndClassName())
						.name("status")
						.style('=', DefaultStyle)
						.style_ex('=', DefaultStyleEx)
						.anchor(EAnchor::LeftBottomRight)
						.dock(EDock::Bottom);
				}
				this_type& parts(std::initializer_list<int> p)
				{
					m_parts = p;
					return this->me();
				}
			};

			StatusBar()
				:StatusBar(Params<>())
			{}
			template <typename TParams, typename = typename TParams::this_type>
			explicit StatusBar(TParams const& p)
				:Control(p)
			{}

			// Get/Set the parts of the status bar
			int Parts(int count, int* parts) const
			{
				assert(::IsWindow(m_hwnd));
				return (int)::SendMessageW(m_hwnd, SB_GETPARTS, WPARAM(count), LPARAM(parts));
			}
			bool Parts(int count, int const* widths)
			{
				assert(::IsWindow(m_hwnd));
				return ::SendMessageW(m_hwnd, SB_SETPARTS, WPARAM(count), LPARAM(widths)) != 0;
			}

			// Get/Set the text in a pane in the status bar
			string Text(int pane, int* type = nullptr) const
			{
				assert(::IsWindow(m_hwnd) && pane >= 0 && pane < 256);

				string s;
				s.resize(static_cast<size_t>(LoWord(::SendMessageW(m_hwnd, SB_GETTEXTLENGTH, WPARAM(pane), LPARAM(0)))) + 1, 0);
				if (!s.empty())
				{
					auto ret = DWORD(::SendMessageW(m_hwnd, SB_GETTEXT, WPARAM(pane), LPARAM(&s[0])));
					if (type) *type = (int)(short)HiWord(ret);
					s.resize(LoWord(ret));
				}
				return s;
			}
			void Text(int pane, string text, int type = 0)
			{
				assert(::IsWindow(m_hwnd) && pane >= 0 && pane < 256);
				Check(::SendMessageW(m_hwnd, SB_SETTEXTW, WPARAM(MakeLong(MakeWord(pane, type), 0)), LPARAM(text.c_str())) != 0, "Failed to set status bar pane text");
			}

			// Get the client area of a pane in the status bar
			gui::Rect Rect(int pane) const
			{
				assert(::IsWindow(m_hwnd) && pane >= 0 && pane < 256);
				gui::Rect rect;
				Check(::SendMessageW(m_hwnd, SB_GETRECT, WPARAM(pane), LPARAM(&rect)) != 0, "Failed to get the client rect for a status bar pane");
				return rect;
			}

			// 
			void ResizeToParent(gui::Rect const& parent_client, bool repaint = false) override
			{
				// Ignore padding in the parent
				gui::Rect rect = parent_client;
				if (m_parent.ctrl() != nullptr) rect = rect.Adjust(-m_parent->cp().m_padding);
				Control::ResizeToParent(rect, repaint);
			}

			#pragma region Handlers
			LRESULT WndProc(UINT message, WPARAM wparam, LPARAM lparam) override
			{
				switch (message)
				{
				case WM_CREATE:
					{
						auto& p = cp<Params<>>();
						if (p.m_parts.size() != 0)
							Parts(int(p.m_parts.size()), std::begin(p.m_parts));
						break;
					}
				}
				return Control::WndProc(message, wparam, lparam);
			}
			#pragma endregion
		};
		struct TabControl :Control
		{
			enum { DefW = 80, DefH = 80 };
			enum :DWORD { DefaultStyle   = DefaultControlStyle }; // TCS_VERTICAL | TCS_RIGHT | TCS_BOTTOM
			enum :DWORD { DefaultStyleEx = DefaultControlStyleEx };
			static wchar_t const* WndClassName() { return L"SysTabControl32"; }

			struct Item :TCITEMW
			{
				Item() :TCITEMW() {}
				Item(wchar_t const* label, int image = -1, LPARAM param = 0) :TCITEMW()
				{
					mask    = TCIF_TEXT | (image != -1 ? TCIF_IMAGE : 0) | (param != 0 ? TCIF_PARAM : 0);
					pszText = const_cast<wchar_t*>(label);
					iImage  = image;
					lParam  = param;
				}
			};
			struct TabEventArgs :EmptyArgs
			{
				Control* m_tab;
				int      m_tab_index;
				TabEventArgs(Control* tab, int tab_index) :EmptyArgs() ,m_tab(tab) ,m_tab_index(tab_index) {}
			};
			struct TabSwitchEventArgs :CancelEventArgs
			{
				// True if 'm_tab' is being switched to (cancel ignored in this case)
				// False if 'm_tab' is being switched away from (cancel will stop the switch)
				bool m_activating;

				// The tab being left/entered
				Control* m_tab;
				int      m_tab_index;

				TabSwitchEventArgs(bool activating, Control* tab, int tab_index)
					:CancelEventArgs(false)
					,m_activating(activating)
					,m_tab(tab)
					,m_tab_index(tab_index)
				{}
			};

			template <typename Derived = void>
			struct Params :Control::Params<not_void_t<Derived, Params<Derived>>>
			{
				using this_type = typename Params::this_type;
				Params()
				{
					this->wndclass(WndClassName())
						.name("tab-ctrl")
						.wh(DefW, DefH)
						.style('=', DefaultStyle)
						.style_ex('=', DefaultStyleEx);
				}
			};

			// The tab pages. Owned externally
			std::vector<Control*> m_tabs;

			TabControl()
				:TabControl(Params<>())
			{}
			template <typename TParams, typename = typename TParams::this_type>
			explicit TabControl(TParams const& p)
				:Control(p)
				,m_tabs()
			{}

			// The number of tabs added
			int TabCount() const
			{
				return int(m_tabs.size());
			}

			// Get a tab by index
			Control& Tab(int index) const
			{
				ValidateTabIndex(index);
				return *m_tabs[index];
			}

			// The active tab
			Control* ActiveTab() const
			{
				auto active_tab_index = SelectedIndex();
				return active_tab_index != -1 ? m_tabs[active_tab_index] : nullptr;
			}

			// Get/Set the active tab by index.
			int SelectedIndex() const
			{
				return int(::SendMessageW(m_hwnd, TCM_GETCURSEL, 0, 0));
			}
			void SelectedIndex(int tab_index)
			{
				auto active_tab_index = SelectedIndex();
				if (tab_index == active_tab_index) return;
				SwitchTab(active_tab_index, tab_index, true);
				Invalidate();
			}

			// Add a tab to the tab control.
			// label - The label to appear on the tab control.
			// tab - The child control to use as the view for the tab.
			// active - true to make the tab active, false to just add the tab.
			// image - The index into the image list for the image to display by the tab label.
			// param - The param value to associate with the tab.
			// Returns the zero based index of the new tab, -1 on failure
			int Insert(wchar_t const* label, Control& tab, int index = -1, bool active = true, int image = -1, LPARAM param = 0)
			{
				// Make sure it's a real window
				assert(::IsWindow(tab));

				// The window must have the WS_CHILD style bit set and the WS_VISIBLE style bit not set.
				tab.Style('+', WS_CHILD).Style('-', WS_VISIBLE);

				// Hide the view window
				tab.Enabled(false);
				tab.Visible(false);

				// Add the tab to the tab control
				Item item(label, image, param);

				// Save the index of the currently selected
				auto sel = SelectedIndex();

				// Insert the item at the end of the tab control
				index = index != -1 ? index : TabCount();
				index = int(::SendMessageW(m_hwnd, TCM_INSERTITEMW, WPARAM(index), LPARAM(&item)));
				Check(index != -1, FmtS("Failed to add tab %s", Narrow(label).c_str()));

				// Add the tab
				m_tabs.push_back(&tab);
				tab.Parent(this);

				// Resize it appropriately
				LayoutTab(tab, ClientRect(true));

				// Select the tab that is being added, if desired
				if (active)
					SwitchTab(sel, index, true);

				OnTabAdded(TabEventArgs(&tab, index));
				return index;
			}

			// Remove a tab by index.
			// tab_index - The index of the tab to remove.
			// Returns the removed tab.
			Control* Remove(int tab_index)
			{
				ValidateTabIndex(tab_index);

				// Save the window handle that is being removed
				auto tab = m_tabs[tab_index];

				// Notify subclasses that a tab was removed
				OnTabRemoved(TabEventArgs(tab, tab_index));

				// Adjust the active tab index if deleting a tab before it
				auto new_tab_count = TabCount() - 1;
				auto active_tab_index = SelectedIndex();
				if (active_tab_index >= new_tab_count)
					active_tab_index = new_tab_count - 1;

				// Remove the item from the view list
				Check(::SendMessageW(m_hwnd, TCM_DELETEITEM, tab_index, 0) != 0, FmtS("Failed to delete tab %d", tab_index));
				m_tabs.erase(std::begin(m_tabs) + tab_index);
				tab->Parent(nullptr);

				// Adjust the active tab index if deleting a tab before it
				SelectedIndex(active_tab_index);

				return tab;
			}

			// Remove all the tabs from the tab control.
			void RemoveAllTabs()
			{
				// Delete tabs in reverse order to preserve indices
				for (auto i = TabCount(); i-- != 0;)
					Remove(i);
			}

			// Return tab info for a tab by index
			Item TabInfo(int tab_index, int mask = TCIF_TEXT|TCIF_IMAGE|TCIF_PARAM, wchar_t* buf = nullptr, int buf_count = 0) const
			{
				ValidateTabIndex(tab_index);

				Item info;
				info.mask = mask;
				info.pszText = buf;
				info.cchTextMax = buf_count;
				Check(::SendMessage(m_hwnd, TCM_GETITEMW, tab_index, LPARAM(&info)) != 0, FmtS("Failed to read item info for tab %d", tab_index));
				return info;
			}

			// Return the label of the specified tab.
			std::wstring TabText(int tab_index) const
			{
				// If the TCIF_TEXT flag is set in the mask member of the TCITEM structure, the control may change the pszText
				// member of the structure to point to the new text instead of filling the buffer with the requested text.
				// The control may set the pszText member to NULL to indicate that no text is associated with the item.
				wchar_t buf[128] = {};
				auto info = TabInfo(tab_index, TCIF_TEXT, buf, _countof(buf));
				return info.pszText != nullptr ? info.pszText : L"";
			}

			// Return the image index for a tab by index
			int TabImage(int tab_index) const
			{
				return TabInfo(tab_index, TCIF_IMAGE).iImage;
			}

			// Return the param for a tab by index
			LPARAM TabParam(int tab_index) const
			{
				return TabInfo(tab_index, TCIF_PARAM).lParam;
			}

			// Update the position of all the contained windows.
			// 'client_rect' is the client area of this tab control (or what it will be soon)
			void UpdateLayout(Rect const& client_rect, bool repaint = false)
			{
				for (auto tab : m_tabs)
					LayoutTab(*tab, client_rect, repaint);
			}
			void UpdateLayout(bool repaint = false)
			{
				UpdateLayout(ClientRect(true), repaint);
			}

			// Events
			EventHandler<TabControl&, TabEventArgs const&> TabAdded;
			EventHandler<TabControl&, TabEventArgs const&> TabRemoved;
			EventHandler<TabControl&, TabSwitchEventArgs const&> TabSwitch;

			// Handlers
			virtual void OnTabAdded(TabEventArgs const& args)
			{
				TabAdded(*this, args);
			}
			virtual void OnTabRemoved(TabEventArgs const& args)
			{
				TabRemoved(*this, args);
			}
			virtual void OnTabSwitch(TabSwitchEventArgs& args)
			{
				TabSwitch(*this, args);
			}

		protected:

			// Message map function
			bool ProcessWindowMessage(HWND parent_hwnd, UINT message, WPARAM wparam, LPARAM lparam, LRESULT& result) override
			{
				switch (message)
				{
				case WM_NCCALCSIZE:
					{
						// todo: Should be using to set the effective client rect.
						if (wparam != 0)
						{
							auto& parms = *reinterpret_cast<NCCALCSIZE_PARAMS*>(lparam);
							auto& proposed = parms.rgrc[0];
							auto& old_bounds_ps = parms.rgrc[1];
							auto& old_client_ps = parms.rgrc[2];
							(void)proposed, old_bounds_ps, old_client_ps;
						}
						else
						{
						}
						break;
					}
				case WM_NOTIFY:
					{
						auto hdr = reinterpret_cast<NMHDR*>(lparam);
						if (hdr->hwndFrom == m_hwnd)
						{
							switch (hdr->code)
							{
							case TCN_SELCHANGING:
								{
									auto tab_index = SelectedIndex();
									auto args = TabSwitchEventArgs(false, m_tabs[tab_index], tab_index);
									OnTabSwitch(args);
									if (args.m_cancel) return TRUE;
									SwitchTab(tab_index, -1, false);
									break;
								}
							case TCN_SELCHANGE:
								{
									auto tab_index = SelectedIndex();
									auto args = TabSwitchEventArgs(true, m_tabs[tab_index], tab_index);
									SwitchTab(-1, tab_index, false);
									OnTabSwitch(args);
									break;
								}
							}
							return true;
						}
						break;
					}
				}
				return Control::ProcessWindowMessage(parent_hwnd, message, wparam, lparam, result);
			}

			// Handle window size changing starting or stopping
			void OnWindowPosChange(WindowPosEventArgs const& args) override
			{
				if (!args.m_before && args.IsResize() && !args.Iconic())
					UpdateLayout(ClientRect(true));

				Control::OnWindowPosChange(args);
			}

			// Resize a tab to fit this control
			void LayoutTab(Control& tab, Rect const& client_rect, bool repaint = false)
			{
				tab.ParentRect(client_rect.Adjust(-tab.Margin()), repaint, nullptr, EWindowPos::NoZorder);
			}

			// Throw if 'tab_index' is invalid
			void ValidateTabIndex(int tab_index) const
			{
				Check(tab_index >= 0 && tab_index < TabCount(), FmtS("Tab index (%d) out of range", tab_index));
			}

			// Switch from tab 'old' to tab 'neu'
			void SwitchTab(int old, int neu, bool setcursel)
			{
				// Disable the old tab
				if (old != -1)
				{
					ValidateTabIndex(old);
					auto& old_tab = *m_tabs[old];
					if (::IsWindow(old_tab))
					{
						old_tab.Enabled(false);
						old_tab.Visible(false);
						old_tab.Invalidate();
					}
				}

				// Enable the new tab
				if (neu != -1)
				{
					ValidateTabIndex(neu);
					auto& neu_tab = *m_tabs[neu];
					if (::IsWindow(neu_tab))
					{
						neu_tab.Enabled(true);
						neu_tab.Visible(true);
						neu_tab.Invalidate();
					}
				}

				// Set the new tab index
				if (setcursel)
				{
					::SendMessageW(m_hwnd, TCM_SETCURSEL, neu, 0);
				}
			}

			// The client rect for a tab control, excludes the tabs
			Rect ClientRectCore(bool padded) const override
			{
				auto cr = Control::ClientRectCore(padded);
				::SendMessageW(m_hwnd, TCM_ADJUSTRECT, FALSE, LPARAM(&cr));

				// TC has built in padding, we want to remove that and use our own padding. There is also a 2px 3D border
				auto style = Style();
				if ((style & TCS_BOTTOM) && !(style & TCS_VERTICAL))    cr = cr.Adjust(-4, -4, +2, +1); // Bottom
				else if ((style & TCS_RIGHT) && (style & TCS_VERTICAL)) cr = cr.Adjust(-4, -4, +2, +4); // Right
				else if ((style & TCS_VERTICAL))                        cr = cr.Adjust(-2, -4, +4, +4); // Left
				else                                                    cr = cr.Adjust(-3, -1, +1, +2); // Top
				return cr;
			}
		};
		struct Splitter :Control
		{
			enum { DefW = 80, DefH = 80 };
			enum :DWORD { DefaultStyle   = DefaultControlStyle };
			enum :DWORD { DefaultStyleEx = DefaultControlStyleEx };
			static wchar_t const* WndClassName() { return L"gui::Splitter"; }

			template <typename Derived = void>
			struct Params :Control::Params<not_void_t<Derived, Params<Derived>>>
			{
				using this_type = typename Params::this_type;

				int   m_bar_width;
				float m_bar_pos;
				int   m_min_pane_size;
				bool  m_vertical;
				bool  m_full_drag;

				Params()
					:m_bar_width(4)
					,m_bar_pos(0.5f)
					,m_min_pane_size(20)
					,m_vertical(false)
					,m_full_drag()
				{
					::SystemParametersInfoW(SPI_GETDRAGFULLWINDOWS, 0, &m_full_drag, 0);
					this->wndclass(RegisterWndClass<Splitter>())
						.name("split")
						.wh(DefW, DefH)
						.style('=', DefaultStyle)
						.style_ex('=', DefaultStyleEx);
				}
				this_type& width(int w)
				{
					m_bar_width = w;
					return this->me();
				}
				this_type& pos(float p)
				{
					m_bar_pos = std::min(1.0f, std::max(0.0f, p));
					return this->me();
				}
				this_type& min_pane_width(int w)
				{
					m_min_pane_size = w;
					return this->me();
				}
				this_type& vertical()
				{
					m_vertical = true;
					return this->me();
				}
				this_type& horizontal()
				{
					m_vertical = false;
					return this->me();
				}
				this_type& full_drag(bool fd = true)
				{
					m_full_drag = fd;
					return this->me();
				}
			};

			Panel Pane0;
			Panel Pane1;
			bool m_vertical;
			bool m_full_drag;
			int m_bar_width;
			float m_bar_pos;
			int m_min_pane_size;
			HCURSOR m_cursor;

			Splitter()
				:Splitter(Params<>())
			{}
			template <typename TParams, typename = typename TParams::this_type>
			explicit Splitter(TParams const& p)
				:Control(p)
				,Pane0(Panel::Params<>().parent(this_).name(FmtS("%s-L", p.m_name)).anchor(EAnchor::None).bk_col(::GetSysColor(COLOR_APPWORKSPACE)))
				,Pane1(Panel::Params<>().parent(this_).name(FmtS("%s-R", p.m_name)).anchor(EAnchor::None).bk_col(::GetSysColor(COLOR_APPWORKSPACE)))
				,m_vertical(p.m_vertical)
				,m_full_drag(p.m_full_drag)
				,m_bar_width(p.m_bar_width)
				,m_bar_pos(p.m_bar_pos)
				,m_min_pane_size(p.m_min_pane_size)
				,m_cursor(::LoadCursor(nullptr, m_vertical ? IDC_SIZEWE : IDC_SIZENS))
			{
				if (IsWindow(*this))
					UpdateLayout();
			}

			// Get/Set the bar position as a fraction
			float BarPos() const
			{
				return m_bar_pos;
			}
			void BarPos(float pos, bool repaint = false)
			{
				// Get the available client size based on orientation
				auto w = m_vertical ? ClientRect(true).width() : ClientRect(true).height();
				if (w > 0)
				{
					auto f = 2*w > m_min_pane_size ? m_min_pane_size / (float)w : 0.5f;
					m_bar_pos = std::min(1.0f - f, std::max(0.0f + f, pos));
					UpdateLayout(repaint);
				}
			}

			// Update the layout of child windows
			// 'client_rect' is the client area of this control (or what it will be soon)
			void UpdateLayout(Rect const& client_rect, bool repaint = false)
			{
				auto bar_pos = BarPos();
				Pane0.Visible(bar_pos != 0.0f);
				Pane1.Visible(bar_pos != 1.0f);

				// Invalidate the current area of the splitter bar
				auto bar_rect = BarRect();
				Invalidate(false, &bar_rect);

				// Update the size of the child panes
				if (Pane0.Visible())
				{
					Pane0.ParentRect(PaneRect(0, client_rect), repaint);
				}
				if (Pane1.Visible())
				{
					Pane1.ParentRect(PaneRect(1, client_rect), repaint);
				}
			}
			void UpdateLayout(bool repaint = false)
			{
				UpdateLayout(ClientRect(true), repaint);
			}

		protected:

			// Return the rect for the bar in client space
			Rect BarRect(Rect client) const
			{
				if (BarPos() == 0.0f)
					return m_vertical
						? Rect(client.left, client.top, client.left, client.bottom)
						: Rect(client.left, client.top, client.right, client.top);

				if (BarPos() == 1.0f)
					return m_vertical
						? Rect(client.right, client.top, client.right, client.bottom)
						: Rect(client.left, client.bottom, client.right, client.bottom);

				auto hw = m_bar_width * 0.5f;
				return m_vertical
					? Rect(client.left + int(client.width() * BarPos() - hw), client.top, client.left +  int(client.width() * BarPos() + hw), client.bottom)
					: Rect(client.left, client.top + int(client.height() * BarPos() - hw), client.right, client.top + int(client.height() * BarPos() + hw));
			}
			Rect BarRect() const
			{
				return BarRect(ClientRect(true));
			}

			// Return the rect for a pane in client space
			Rect PaneRect(int idx, Rect client) const
			{
				auto bar = BarRect(client);
				switch (idx)
				{
				default:
					assert(false);
					return Rect();
				case 0:
					return m_vertical
						? Rect(client.left, client.top, bar.left, client.bottom)
						: Rect(client.left, client.top, client.right, bar.top);
				case 1:
					return m_vertical
						? Rect(bar.right, client.top, client.right, client.bottom)
						: Rect(client.left, bar.bottom, client.right, client.bottom);
				}
			}
			Rect PaneRect(int idx) const
			{
				return PaneRect(idx, ClientRect(true));
			}

			// Draw the ghost bar. Note, drawing twice 'undraws' the ghost bar
			void DrawGhostBar()
			{
				auto rect = BarRect();
				if (!rect.empty())
				{
					// convert client to window coordinates
					auto wndrect = ScreenRect();
					::MapWindowPoints(nullptr, m_hwnd, (LPPOINT)&wndrect, 2);
					::OffsetRect(&rect, -wndrect.left, -wndrect.top);

					// invert the brush pattern (looks just like frame window sizing)
					WindowDC dc(m_hwnd);
					auto brush = Brush::Halftone();

					// Paint the ghost bar
					auto old = ::SelectObject(dc, brush);
					::PatBlt(dc, rect.left, rect.top, rect.right - rect.left, rect.bottom - rect.top, PATINVERT);
					::SelectObject(dc, old);
				}
			}

			// Handle window size changing starting or stopping
			void OnWindowPosChange(WindowPosEventArgs const& args) override
			{
				if (!args.m_before && args.IsResize() && !args.Iconic())
					UpdateLayout(ClientRect(true));

				Control::OnWindowPosChange(args);
			}

			// Handle the Paint event. Return true, to prevent anything else handling the event
			void OnPaint(PaintEventArgs& args) override
			{
				Control::OnPaint(args);
				if (args.m_handled)
					return;

				// Paint the splitter
				PaintStruct p(m_hwnd);

				// Draw the splitter bar
				if (BarPos() != 0.0f && BarPos() != 1.0f)
				{
					auto rect = BarRect();
					auto back_brush = args.m_bsh_back ? static_cast<HBRUSH>(args.m_bsh_back) : WndBackground();
					::FillRect(args.m_dc, &rect, back_brush);
				}

				// Painting is done
				args.m_handled = true;
			}

			// Handle mouse button down/up events. Return true, to prevent anything else handling the event
			void OnMouseButton(MouseEventArgs& args) override
			{
				Control::OnMouseButton(args);
				if (args.m_handled)
					return;

				if (args.m_down)
				{
					auto pt = args.m_point;
					auto bar_rect = BarRect();
					if (::GetCapture() != m_hwnd && bar_rect.Contains(pt))
					{
						args.m_handled = true;
						::SetCapture(m_hwnd);
						::SetCursor(m_cursor);

						if (m_full_drag)
						{}
						else
							DrawGhostBar();
					}
					else if (::GetCapture() == m_hwnd && !bar_rect.Contains(pt))
					{
						// If we have capture but are not over the splitter, this is
						// the case where we alt-tab during a splitter bar drag.
						args.m_handled = true;
						::ReleaseCapture();
					}
				}
				else
				{
					if (::GetCapture() == m_hwnd)
						::ReleaseCapture();
				}
			}

			// Handle mouse move events
			void OnMouseMove(MouseEventArgs& args) override
			{
				Control::OnMouseMove(args);
				if (args.m_handled)
					return;

				auto pt = args.m_point;
				auto bar_rect = BarRect();
				if (::GetCapture() == m_hwnd)
				{
					args.m_handled = true;
					auto client = ClientRect(true);
					auto pos = m_vertical
						? float(pt.x - client.left) / client.width()
						: float(pt.y - client.top ) / client.height();

					if (pos != BarPos())
					{
						if (m_full_drag)
						{
							BarPos(pos, true);
						}
						else
						{
							DrawGhostBar();
							BarPos(pos);
							DrawGhostBar();
						}
					}
				}
				else // Not dragging, just hovering
				{
					if (bar_rect.Contains(pt))
						::SetCursor(m_cursor);
				}
			}
		};
		struct ToolTip :Control
		{
			enum { DefW = 80, DefH = 23 };
			enum :DWORD { DefaultStyle   = (DefaultControlStyle | WS_GROUP | SS_LEFT) & ~WS_TABSTOP };
			enum :DWORD { DefaultStyleEx = DefaultControlStyleEx };
			static wchar_t const* WndClassName() { return L"tooltips_class32"; }
		
			template <typename Derived = void>
			struct Params :Control::Params<not_void_t<Derived, Params<Derived>>>
			{
				using this_type = typename Params::this_type;
				using base_type = typename Control::Params<not_void_t<Derived, Params<Derived>>>;

				Params()
				{
					this->wndclass(WndClassName())
						.name("tt")
						.wh(DefW, DefH)
						.style('=', DefaultStyle)
						.style_ex('=', DefaultStyleEx);
				}
				this_type& show_always(bool on = true)
				{
					base_type::style('+',TTS_ALWAYSTIP);
					return this->me(); 
				}
			};

			// Construct
			ToolTip()
				:ToolTip(Params<>())
			{}
			template <typename TParams, typename = typename TParams::this_type>
			explicit ToolTip(TParams const& p)
				:Control(p)
			{}
		};
		#pragma endregion

		#pragma region Dialogs
		// Options for the Open/Save file UI functions
		struct FileUIOptions
		{
			wchar_t const*           m_def_extn;       // The default extension, e.g. L"txt". It seems a list is supported, e.g. L"doc;docx"
			COMDLG_FILTERSPEC const* m_filters;        // File type filters, e.g. COMDLG_FILTERSPEC spec[] = {{L"JayPegs",L"*.jpg;*.jpeg"}, {L"Bitmaps", L"*.bmp"}, {L"Whatever", L"*.*"}};
			size_t                   m_filter_count;   // The length of the 'm_filters' array
			size_t                   m_filter_index;   // The index to select from the filters
			DWORD                    m_flags;          // Additional options
			IFileDialogEvents*       m_handler;        // A handler for events generated using the use of the file dialog
			mutable DWORD            m_handler_cookie; // Used to identify the handler when registered. Leave this as 0

			FileUIOptions(wchar_t const* def_extn = nullptr, COMDLG_FILTERSPEC const* filters = nullptr, size_t filter_count = 0, size_t filter_index = 0, DWORD flags = 0, IFileDialogEvents* handler = nullptr)
				:m_def_extn(def_extn)
				,m_filters(filters)
				,m_filter_count(filter_count)
				,m_filter_index(filter_index)
				,m_flags(flags)
				,m_handler(handler)
				,m_handler_cookie()
			{}
			FileUIOptions& def_extn(wchar_t const* extn)
			{
				m_def_extn = extn;
				return *this;
			}
			FileUIOptions& filters(COMDLG_FILTERSPEC const* filters, size_t filter_count, size_t filter_index)
			{
				// File type filters, e.g. COMDLG_FILTERSPEC spec[] = {{L"JayPegs",L"*.jpg;*.jpeg"}, {L"Bitmaps", L"*.bmp"}, {L"Whatever", L"*.*"}}
				assert(filter_index >= 0 && filter_index < filter_count);
				m_filters = filters;
				m_filter_count = filter_count;
				m_filter_index = filter_index;
				return *this;
			}
			FileUIOptions& idx(size_t i)
			{
				assert(i >= 0 && i < m_filter_count);
				m_filter_index = i;
				return *this;
			}
			FileUIOptions& flags(DWORD f)
			{
				m_flags = f;
				return *this;
			}
			FileUIOptions& handler(IFileDialogEvents* h)
			{
				m_handler = h;
				return *this;
			}
		};

		// Open or SaveAs file dialog. Returns true if the user did not cancel
		template <typename ResultPred> bool FileUI(CLSID type, HWND parent, FileUIOptions const& opts, ResultPred results)
		{
			// see: https://msdn.microsoft.com/en-us/library/windows/desktop/bb776913(v=vs.85).aspx

			// CoCreate the File Open Dialog object.
			CComPtr<IFileDialog> fd;
			Check(::CoCreateInstance(type, nullptr, CLSCTX_ALL, __uuidof(IFileDialog), (void**)&fd.p), "CoCreateInstance failed. Ensure CoInitialize has been called");

			// Hook up the event handler.
			struct EvtHook
			{
				IFileDialog* m_fd;
				FileUIOptions const* m_opts;
				EvtHook(IFileDialog* fd, FileUIOptions const& opts) :m_fd(fd) ,m_opts()
				{
					if (opts.m_handler == nullptr) return;
					Check(m_fd->Advise(opts.m_handler, &opts.m_handler_cookie), "Failed to assign file open/save event handler");
					m_opts = &opts; // use the saved pointer to indicate 'unadvise' is needed
				}
				~EvtHook()
				{
					if (m_opts == nullptr) return;
					Check(m_fd->Unadvise(m_opts->m_handler_cookie), "Failed to un-register file open/save dialog event handler");
				}
			} evt_hook(fd.p, opts);

			// Set the options on the dialog.
			if (opts.m_flags != 0)
			{
				// Before setting, always get the options first in order not to override existing options.
				DWORD flags;
				Check(fd->GetOptions(&flags), "Failed to set file open/save dialog options");
				Check(fd->SetOptions(flags | opts.m_flags), "");
			}

			// Set the file types to display only.
			if (opts.m_filter_count != 0)
			{
				Check(fd->SetFileTypes(UINT(opts.m_filter_count), opts.m_filters), "Failed to set file type filters");
				Check(fd->SetFileTypeIndex(UINT(opts.m_filter_index)), "Failed to set the file type filter index");
			}

			// Set the default extension to be ".doc" file.
			if (opts.m_def_extn != nullptr)
				Check(fd->SetDefaultExtension(opts.m_def_extn), "Failed to set the default file extension");

			// Show the dialog
			auto r = fd->Show(parent);
			if (r == HRESULT_FROM_WIN32(ERROR_CANCELLED)) return false;
			if (r != S_OK) Check(r, "Failed to show the file open/save dialog");

			// Pass the dialog to 'results' to allow the caller to get what they want
			return results(fd.p);
		}

		// Present the Open file dialog and return the selected filepath
		template <typename = void> std::vector<std::wstring> OpenFileUI(HWND parent = nullptr, FileUIOptions const& opts = FileUIOptions())
		{
			std::vector<std::wstring> results;
			FileUI(CLSID_FileOpenDialog, parent, opts, [&](IFileDialog* fd)
			{
				auto fod = static_cast<IFileOpenDialog*>(fd);

				// Obtain the results once the user clicks the 'Open' button. The result is an IShellItem object.
				CComPtr<IShellItemArray> items;
				if (SUCCEEDED(fod->GetResults(&items.p)) && items != nullptr)
				{
					DWORD count;
					Check(items->GetCount(&count), "Failed to read the number of results from the file open dialog result");
					for (DWORD i = 0; i != count; ++i)
					{
						CComPtr<IShellItem> item;
						Check(items->GetItemAt(i, &item.p), FmtS("Failed to read result %d from the file open dialog results", i));
						
						wchar_t* fpath;
						Check(item->GetDisplayName(SIGDN_FILESYSPATH, &fpath), "Failed to read the filepath from an open file dialog result");
						results.emplace_back(fpath);
						CoTaskMemFree(fpath);
					}
				}
				else
				{
					CComPtr<IShellItem> item;
					Check(fod->GetResult(&item.p), "Failed to read result from the file open dialog results");

					wchar_t* fpath;
					Check(item->GetDisplayName(SIGDN_FILESYSPATH, &fpath), "Failed to read the filepath from an open file dialog result");
					results.emplace_back(fpath);
					CoTaskMemFree(fpath);
				}
				return true;
			});
			return results;
		}

		// Present the SaveAs file dialog and return the selected filepath
		inline std::wstring SaveFileUI(HWND parent = nullptr, FileUIOptions const& opts = FileUIOptions())
		{
			std::wstring filepath;
			FileUI(CLSID_FileSaveDialog, parent, opts, [&](IFileDialog* fd)
			{
				auto fsd = static_cast<IFileSaveDialog*>(fd);
					
				CComPtr<IShellItem> res;
				Check(fsd->GetResult(&res.p), "Failed to read result %d from the file save dialog result");

				wchar_t* fpath;
				Check(res->GetDisplayName(SIGDN_FILESYSPATH, &fpath), "Failed to read the filepath from a the save file dialog result");
				filepath = std::wstring(fpath);
				CoTaskMemFree(fpath);
				return true;
			});
			return filepath;
		}

		// Present the Open Folder dialog and return the selected folder path
		inline std::wstring OpenFolderUI(HWND parent = nullptr, FileUIOptions const& opts = FileUIOptions())
		{
			std::wstring folderpath;
			FileUI(CLSID_FileOpenDialog, parent, opts, [&](IFileDialog* fd)
			{
				(void)fd;
				return true;
			});
			return folderpath;
		}

		// Simple auto-sizing message box with up-to 3 buttons
		struct MsgBox :Form
		{
			Panel       m_panel_btns;
			Button      m_btn_negative;
			Button      m_btn_neutral;
			Button      m_btn_positive;
			Panel       m_panel_msg;
			ImageBox    m_image;
			RichTextBox m_message;

			Button* m_accept_button;
			Button* m_cancel_button;
			bool    m_reflow;        // Set to true to have the dialog automatically line wrap text. False to honour message new lines
			float   m_reflow_aspect; // The ratio of width to height used to decide where to wrap text

			static float DefaultReflowAspect() { return 5.0f; }
			enum class EButtons
			{
				Ok               = (1 << int(EDialogResult::Ok)),
				Cancel           = (1 << int(EDialogResult::Cancel)),
				OkCancel         = Ok | Cancel,
				YesNo            = (1 << int(EDialogResult::Yes)) | (1 << int(EDialogResult::No)),
				YesNoCancel      = YesNo | Cancel,
				AbortRetryIgnore = (1 << int(EDialogResult::Abort)) | (1 << int(EDialogResult::Retry)) | (1 << int(EDialogResult::Ignore)),
				RetryCancel      = (1 << int(EDialogResult::Retry)) | Cancel,
				allow_bitops = 0,
			};
			enum class EIcon
			{
				None,
				Application = int((char*)IDI_APPLICATION - (char*)0), 
				Hand        = int((char*)IDI_HAND        - (char*)0),
				Question    = int((char*)IDI_QUESTION    - (char*)0),
				Exclamation = int((char*)IDI_EXCLAMATION - (char*)0),
				Asterisk    = int((char*)IDI_ASTERISK    - (char*)0),
				WinLogo     = int((char*)IDI_WINLOGO     - (char*)0),
				Shield      = int((char*)IDI_SHIELD      - (char*)0),
				Warning     = Exclamation,
				Error       = Hand,
				Information = Asterisk,
			};

			// Display a modal message box
			static EDialogResult Show(HWND parent, wchar_t const* message, wchar_t const* title, EButtons btns = EButtons::Ok, EIcon icon = EIcon::None, int def_btn = 0, bool reflow = true, float reflow_aspect = DefaultReflowAspect())
			{
				MsgBox dlg(parent, message, title, btns, icon, def_btn, reflow, reflow_aspect);
				return dlg.ShowDialog(parent);
			}

			enum { ID_IMAGE = 100 };
			MsgBox(HWND parent, wchar_t const* message, wchar_t const* title, EButtons btns = EButtons::Ok, EIcon icon = EIcon::None, int def_btn = 0, bool reflow =  true, float reflow_aspect = DefaultReflowAspect())
				:Form          (Form       ::Params<>().name("msg-box"   ).dlg().start_pos(EStartPosition::CentreParent ).title(title).wh(316,176).padding(0).wndclass(RegisterWndClass<MsgBox>()))
				,m_panel_btns  (Panel      ::Params<>().name("panel-btns").parent(this_        ).wh(Fill, 52).dock(EDock::Bottom).border())
				,m_btn_negative(Button     ::Params<>().name("btn-neg"   ).parent(&m_panel_btns).wh(86,23).dock(EDock::Right).margin(8,12,8,12).def_btn(def_btn==0))
				,m_btn_neutral (Button     ::Params<>().name("btn-neu"   ).parent(&m_panel_btns).wh(86,23).dock(EDock::Right).margin(8,12,8,12).def_btn(def_btn==1))
				,m_btn_positive(Button     ::Params<>().name("btn-pos"   ).parent(&m_panel_btns).wh(86,23).dock(EDock::Right).margin(8,12,8,12).def_btn(def_btn==2))
				,m_panel_msg   (Panel      ::Params<>().name("panel-msg" ).parent(this_        ).dock(EDock::Fill).bk_col(0xFFFFFF).border())
				,m_image       (ImageBox   ::Params<>().name("img-icon"  ).parent(&m_panel_msg ).wh(48,48).xy(25,25).margin(8,0,8,0).visible(icon != EIcon::None).id(ID_IMAGE))
				,m_message     (RichTextBox::Params<>().name("tb-msg"    ).parent(&m_panel_msg ).wh(Fill,Fill).xy(Left|RightOf|ID_IMAGE,Top|TopOf|ID_IMAGE).margin(0,0,8,12).style('-',WS_HSCROLL).word_wrap().detect_urls().read_only().anchor(EAnchor::All))
				,m_accept_button()
				,m_cancel_button()
				,m_reflow(reflow)
				,m_reflow_aspect(reflow_aspect)
			{
				m_message.Text(message);

				// Copy the form icon from the parent
				if (parent != nullptr)
				{
					HICON ico;
					if ((ico = rcast<HICON>(SendMessageW(parent, WM_GETICON, ICON_BIG, 0))) != nullptr ||
						(ico = rcast<HICON>(GetClassLongPtrW(parent, GCLP_HICON))) != nullptr)
						Form::Icon(ico, true);
					if ((ico = rcast<HICON>(SendMessageW(parent, WM_GETICON, ICON_SMALL2, 0))) != nullptr ||
						(ico = rcast<HICON>(SendMessageW(parent, WM_GETICON, ICON_SMALL, 0))) != nullptr ||
						(ico = rcast<HICON>(GetClassLongPtrW(parent, GCLP_HICONSM))) != nullptr)
						Form::Icon(ico, false);
				}

				// Initialise the button text and result based on 'btns'
				switch (btns)
				{
				default:
					assert(!"Unknown message box button combination");
					break;
				case EButtons::Ok:
					m_btn_positive.Text(L"OK");
					m_btn_positive.DlgResult(EDialogResult::Ok);
					m_accept_button = &m_btn_positive;
					m_cancel_button = &m_btn_positive;
					break;
				case EButtons::OkCancel:
					m_btn_positive.Text(L"OK");
					m_btn_positive.DlgResult(EDialogResult::Ok);
					m_btn_negative.Text(L"Cancel");
					m_btn_negative.DlgResult(EDialogResult::Cancel);
					m_accept_button = &m_btn_positive;
					m_cancel_button = &m_btn_negative;
					break;
				case EButtons::AbortRetryIgnore:
					m_btn_positive.Text(L"&Abort");
					m_btn_positive.DlgResult(EDialogResult::Abort);
					m_btn_neutral .Text(L"&Retry");
					m_btn_neutral .DlgResult(EDialogResult::Retry);
					m_btn_negative.Text(L"&Ignore");
					m_btn_negative.DlgResult(EDialogResult::Ignore);
					break;
				case EButtons::YesNoCancel:
					m_btn_positive.Text(L"&Yes");
					m_btn_positive.DlgResult(EDialogResult::Yes);
					m_btn_neutral .Text(L"&No");
					m_btn_neutral .DlgResult(EDialogResult::No);
					m_btn_negative.Text(L"Cancel");
					m_btn_negative.DlgResult(EDialogResult::Cancel);
					m_accept_button = &m_btn_positive;
					m_cancel_button = &m_btn_negative;
					break;
				case EButtons::YesNo:
					m_btn_positive.Text(L"&Yes");
					m_btn_positive.DlgResult(EDialogResult::Yes);
					m_btn_neutral .Text(L"&No");
					m_btn_neutral .DlgResult(EDialogResult::No);
					m_accept_button = &m_btn_positive;
					break;
				case EButtons::RetryCancel:
					m_btn_positive.Text(L"&Retry");
					m_btn_positive.DlgResult(EDialogResult::Retry);
					m_btn_negative.Text(L"Cancel");
					m_btn_negative.DlgResult(EDialogResult::Cancel);
					m_accept_button = &m_btn_neutral;
					m_cancel_button = &m_btn_negative;
					break;
				}

				// Set the message icon
				if (icon != EIcon::None)
					m_image.Image(nullptr, int(icon), Image::EType::Icon, Image::EFit::Unchanged, 0, 0, LR_DEFAULTCOLOR|LR_DEFAULTSIZE|LR_SHARED);

				// Hook up button handlers
				m_btn_positive.Click += std::bind(&MsgBox::OnButtonClicked, this, _1, _2);
				m_btn_neutral .Click += std::bind(&MsgBox::OnButtonClicked, this, _1, _2);
				m_btn_negative.Click += std::bind(&MsgBox::OnButtonClicked, this, _1, _2);
			}

			// Display as a modeless form, creating the window first if necessary
			void Show(int show = SW_SHOW) override
			{
				DoLayout();
				return ShowInternal(show);
			}

			// Display the form modally, creating the window first if necessary
			EDialogResult ShowDialog(WndRefC parent = nullptr) override
			{
				DoLayout();
				return ShowDialogInternal(parent);
			}

			// Set the visibility and layout just before showing the message box
			void DoLayout()
			{
				// Set UI element visibility
				m_btn_negative.Visible(!m_btn_negative.Text().empty());
				m_btn_neutral .Visible(!m_btn_neutral .Text().empty());
				m_btn_positive.Visible(!m_btn_positive.Text().empty());
				m_image       .Visible(m_image.Image() != nullptr);

				// Resize the buttons
				{
					// The visible buttons;
					Button* btns[4] = {};
					{
						auto bp = &btns[0];
						if (m_btn_positive.Visible()) *bp++ = &m_btn_positive;
						if (m_btn_neutral .Visible()) *bp++ = &m_btn_neutral;
						if (m_btn_negative.Visible()) *bp++ = &m_btn_negative;
					}

					// The button preferred sizes
					Size size[4] = {}; auto btn_h = 0L;
					for (int i = 0; btns[i] != nullptr; ++i)
					{
						auto pr = btns[i]->ParentRect();
						size[i] = btns[i]->PreferredSize();
						if (size[i].cx < pr.width()) size[i].cx = pr.width();
						if (size[i].cy < pr.height()) size[i].cy = pr.height();
						if (size[i].cy > btn_h) btn_h = size[i].cy;
					}

					// Set the button sizes (position managed by docking)
					m_panel_btns.height(btn_h * 2);
					for (int i = 0; btns[i] != nullptr; ++i)
						btns[i]->size(size[i]);
				}
				// Position, resize the message and set the window size
				{
					// Measure the text to be displayed
					auto text_area = m_message.PreferredSize();

					// Re-flow the text if the aspect ratio is too large
					if (m_reflow && text_area != Size() && text_area.aspect() > m_reflow_aspect)
					{
						// Binary search for an aspect ratio ~= m_reflow_aspect
						auto initial_width = text_area.cx;
						for (float scale0 = 0.0f, scale1 = 1.0f; fabsf(scale1 - scale0) > 0.05f;)
						{
							auto scale = (scale0 + scale1) / 2.0f;
							text_area = m_message.PreferredSize(int(initial_width * scale));
							if      (text_area.aspect() < m_reflow_aspect) scale0 = scale;
							else if (text_area.aspect() > m_reflow_aspect) scale1 = scale;
							else break;
						}
					}

					// If the text area is larger than the screen area, limit the size.
					// Find the screen area to limit how big we go
					auto screen_area = Rect(MonitorInfo::FromWindow(m_hwnd).rcWork);
					screen_area = screen_area.Inflate(-screen_area.width() / 4, -screen_area.height() / 4);
					text_area.cx = std::min(text_area.cx, screen_area.width());
					text_area.cy = std::min(text_area.cy, screen_area.height());

					// Measure the distance from the message text box to the dialog edges
					auto msg_srect = m_message.ScreenRect();
					auto dlg_srect = ScreenRect();
					int dist[] =
					{
						msg_srect.left   - dlg_srect.left,
						msg_srect.top    - dlg_srect.top,
						msg_srect.right  - dlg_srect.right,
						msg_srect.bottom - dlg_srect.bottom - 2*m_message.FontInfo().tmHeight,
					};

					// Set the size of the dialog
					auto sz = Size(
						text_area.cx + dist[0] - dist[2],
						text_area.cy + dist[1] - dist[3]);
					auto pr = ParentRect();
					PositionWindow(pr.centre().x - sz.cx/2, pr.centre().y - sz.cy/2, sz.cx, sz.cy);
				}
			}

			// Button click handler
			void OnButtonClicked(Button& btn, EmptyArgs const&)
			{
				if (btn.DlgResult() != EDialogResult::None)
					Close(btn.DlgResult());
			}

			// Handle keyboard key down/up events.
			void OnKey(KeyEventArgs& args) override
			{
				Form::OnKey(args);
				if (args.m_handled)
					return;

				if (args.m_down == false && args.m_vk_key == VK_RETURN && m_accept_button != nullptr)
					m_accept_button->PerformClick();
				if (args.m_down == false && args.m_vk_key == VK_ESCAPE && m_cancel_button != nullptr)
					m_cancel_button->PerformClick();
			}
		};
		#pragma endregion

		#pragma region Implementation

		// The location of the mouse (client space) at the key event
		inline Point KeyEventArgs::point() const
		{
			Point pt; ::GetCursorPos(&pt);
			::ScreenToClient(m_hwnd, &pt);
			return pt;
		}

		// Get the point in absolute pixels
		inline Point KeyEventArgs::point_px() const
		{
			auto pt = point();
			pt = m_owner->Metrics().Pt(pt);
			return pt;
		}

		// Get the point in absolute pixels
		inline Point MouseEventArgs::point_px() const
		{
			return m_owner->Metrics().Pt(m_point);
		}

		// Get the point in absolute pixels
		inline Point MouseWheelArgs::point_px() const
		{
			return m_owner->Metrics().Pt(m_point);
		}

		#pragma endregion
	}
}

// Restore warnings
#pragma warning(pop)

// Tell the linker to include common controls
#if defined _M_IX86
#pragma comment(linker, "/manifestdependency:\"type='win32' name='Microsoft.Windows.Common-Controls' version='6.0.0.0' processorArchitecture='x86' publicKeyToken='6595b64144ccf1df' language='*'\"")
#elif defined _M_IA64
#pragma comment(linker, "/manifestdependency:\"type='win32' name='Microsoft.Windows.Common-Controls' version='6.0.0.0' processorArchitecture='ia64' publicKeyToken='6595b64144ccf1df' language='*'\"")
#elif defined _M_X64
#pragma comment(linker, "/manifestdependency:\"type='win32' name='Microsoft.Windows.Common-Controls' version='6.0.0.0' processorArchitecture='amd64' publicKeyToken='6595b64144ccf1df' language='*'\"")
#else
#pragma comment(linker, "/manifestdependency:\"type='win32' name='Microsoft.Windows.Common-Controls' version='6.0.0.0' processorArchitecture='*' publicKeyToken='6595b64144ccf1df' language='*'\"")
#endif