//*****************************************************************************************
// GUI Misc
//	Copyright (c) Rylogic 2009
//*****************************************************************************************
#pragma once

#include <string>
#include <filesystem>
#include "pr/common/min_max_fix.h"
#include "pr/common/cast.h"
#include "pr/common/assert.h"
#include "pr/container/byte_data.h"
#include "pr/math/math.h"
#include "pr/gui/wingui.h"

namespace pr
{
	// Return the size of the virtual screen
	inline IRect ScreenBounds()
	{
		return IRect(0, 0, ::GetSystemMetrics(SM_CXSCREEN), ::GetSystemMetrics(SM_CYSCREEN));
	}

	// Return the window bounds as an IRect
	inline IRect WindowBounds(HWND hwnd)
	{
		if (hwnd == nullptr) throw std::runtime_error("window handle must be non-null");
		RECT r; ::GetWindowRect(hwnd, &r);
		return IRect(r.left, r.top, r.right, r.bottom);
	}

	// Return the client area of the window as an IRect
	inline IRect ClientArea(HWND hwnd)
	{
		// Note: Consider using gui::Control::ClientRect(), it takes padding into account
		if (hwnd == nullptr) throw std::runtime_error("window handle must be non-null");
		RECT r; ::GetClientRect(hwnd, &r);
		return IRect(r.left, r.top, r.right, r.bottom);
	}

	// Modifies a rectangle so that it's within 'bounds'
	// Returns true if 'rect' was modified. If 'bounds' is null the screen
	inline bool AdjRectWithin(IRect& rect, IRect const& bounds)
	{
		int w = rect.SizeX();
		int h = rect.SizeY();
		int xofs = 0, yofs = 0;
		if (rect.m_min.x < bounds.m_min.x) xofs = (rect.m_max.x + w < bounds.m_max.x) ?  w : bounds.m_min.x - rect.m_min.x;
		if (rect.m_min.y < bounds.m_min.y) yofs = (rect.m_max.y + h < bounds.m_max.y) ?  h : bounds.m_min.y - rect.m_min.y;
		if (rect.m_max.x > bounds.m_max.x) xofs = (rect.m_min.x - w > bounds.m_min.x) ? -w : bounds.m_min.x - rect.m_min.x;
		if (rect.m_max.y > bounds.m_max.y) yofs = (rect.m_min.y - h > bounds.m_min.y) ? -h : bounds.m_min.y - rect.m_min.y;
		rect = Shifted(rect, xofs, yofs);
		return xofs != 0 || yofs != 0;
	}

	// Return the string in an edit control as a std::string
	template <typename Char, typename Ctrl> inline std::basic_string<Char> GetCtrlText(Ctrl const& ctrl)
	{
		using Win32 = gui::Win32<Char>;

		std::basic_string<Char> str;
		str.resize(Win32::WindowTextLength(ctrl) + 1);
		if (!str.empty()) Win32::WindowText(ctrl, &str[0], (int)str.size());
		while (!str.empty() && *(--str.end()) == 0) str.resize(str.size() - 1);
		return str;
	}
	template <typename Ctrl> inline std::string GetCtrlTextA(Ctrl const& ctrl)
	{
		return GetCtrlText<char,Ctrl>(ctrl);
	}
	template <typename Ctrl> inline std::wstring GetCtrlTextW(Ctrl const& ctrl)
	{
		return GetCtrlText<wchar_t,Ctrl>(ctrl);
	}

	// Convert a client space point to a normalised point
	inline v2 NormalisePoint(HWND hwnd, POINT const& pt)
	{
		return NormalisePoint(Rectangle<float>(ClientArea(hwnd)), To<v2>(pt), 1.0f, -1.0f);
	}

	#if 0
	// Register a window class for a type that inherits WTL::CWindowImpl<> (e.g custom control)
	// Call this before any instances of the control are created
	template <typename T> inline ATOM RegisterClass(CAppModule& app_module)//, HINSTANCE hInstance)
	{
		CWndClassInfo& ci = T::GetWndClassInfo();
		if (ci.m_atom != 0) return ci.m_atom;

		threads::CSLock lock(&app_module.m_csWindowCreate);
		if (ci.m_atom != 0) return ci.m_atom;

		if (ci.m_lpszOrigName != 0) // Windows SuperClassing
		{
			LPCTSTR lpsz = ci.m_wc.lpszClassName;
			WNDPROC proc = ci.m_wc.lpfnWndProc;

			WNDCLASSEX wndClass;
			wndClass.cbSize = sizeof(WNDCLASSEX);
			if (!::GetClassInfoEx(NULL, ci.m_lpszOrigName, &wndClass)) // Try global class
				if (!::GetClassInfoEx(app_module.GetModuleInstance(), ci.m_lpszOrigName, &wndClass)) // Try local class
					return 0;

			memcpy(&ci.m_wc, &wndClass, sizeof(WNDCLASSEX));
			ci.pWndProc = ci.m_wc.lpfnWndProc;
			ci.m_wc.lpszClassName = lpsz;
			ci.m_wc.lpfnWndProc = proc;
		}
		else // Traditionnal registration
		{
			ci.m_wc.hCursor = ::LoadCursor(ci.m_bSystemCursor ? NULL : app_module.GetModuleInstance(), ci.m_lpszCursorID);
			ci.pWndProc     = ::DefWindowProc;
		}

		ci.m_wc.hInstance = app_module.GetModuleInstance();
		ci.m_wc.style |= CS_GLOBALCLASS;

		// Synthetize custom class name
		if (ci.m_wc.lpszClassName == NULL)
		{
			_sntprintf(ci.m_szAutoName, sizeof(ci.m_szAutoName)/sizeof(ci.m_szAutoName[0]), _T("WTL:%8.8X"), (DWORD)&ci.m_wc);
			ci.m_wc.lpszClassName = ci.m_szAutoName;
		}

		// Check previous registration
		WNDCLASSEX wndClassTemp;
		memcpy(&wndClassTemp, &ci.m_wc, sizeof(WNDCLASSEX));
		ci.m_atom = (ATOM)::GetClassInfoEx(ci.m_wc.hInstance, ci.m_wc.lpszClassName, &wndClassTemp);
		if (ci.m_atom == 0)
			ci.m_atom = ::RegisterClassEx(&ci.m_wc);

		return ci.m_atom;
	}

	// Use:
	// virtual BOOL PreTranslateMessage(MSG* pMsg)
	// {
	//    if (pMsg->message == WM_MOUSEWHEEL)
	//       return HoverScroll();
	// }
	inline BOOL HoverScroll(MSG* pMsg)
	{
		PR_ASSERT(PR_DBG, pMsg->message == WM_MOUSEWHEEL, "");

		// Find the control under the mouse at screen position pMsg->lParam
		POINT pos = {GET_X_LPARAM(pMsg->lParam), GET_Y_LPARAM(pMsg->lParam)};
		HWND hwnd = ::WindowFromPoint(pos);
		if (hwnd == 0) return FALSE;                                // No window...
		if (hwnd == pMsg->hwnd) return FALSE;                       // Hovering over the correct control already
		//if (Control.FromHandle(hWnd) == null) return false;			// Not over a control
		//if (m_wnds != null && !m_wnds.Contains(hWnd)) return false;	// Not a control we're hoverscrolling
		::SendMessage(hwnd, pMsg->message, pMsg->wParam, pMsg->lParam);
		return TRUE;
	}
	#endif

	// Return a sub menu by address
	// Use: auto menu = menu.SubMenuByName("&File,&Recent Files");
	// Returns Menu() if the sub menu isn't found
	template <typename Char> HMENU SubMenuByName(HMENU root, Char const* address)
	{
		assert(root != nullptr);
		for (auto addr = address; *addr != 0;)
		{
			// Find ',' in 'address'
			auto end = addr;
			for (; *end != 0 && *end != ','; ++end) {}
			if (end == addr)
				break;

			// Look for the first part of the address in the items of 'root'
			for (int i = 0, iend = ::GetMenuItemCount(root); i != iend; ++i)
			{
				// Get the menu item name and length
				// Check the item name matches the first part of the address
				Char item_name[256] = {};
				auto item_name_len = gui::Win32<Char>::MenuString(root, UINT(i), item_name, _countof(item_name), MF_BYPOSITION);
				if (item_name_len != end - addr || std::char_traits<Char>::compare(addr, item_name, item_name_len) != 0)
					continue;

				// If this is the last part of the address, then return the submenu
				// Note, if the menu is not a submenu then you'll get 0, turn it into a popup menu before here
				auto sub_menu = ::GetSubMenu(root, i);
				if (*end == 0 || sub_menu == 0)
					return sub_menu;

				root = sub_menu;
				addr = end + 1;
				break;
			}
		}
		return nullptr;
	}

	// Helper object for scoped selecting of objects into an HDC
	template <typename GdiHandle> struct DCSelect
	{
		HDC       m_hdc;
		GdiHandle m_obj;
		HGDIOBJ   m_old_obj;
		bool      m_cleanup;

		DCSelect(HDC hdc, GdiHandle obj, bool cleanup) :m_hdc(hdc) ,m_obj(obj) ,m_old_obj(SelectObject(m_hdc, (HGDIOBJ)obj)) ,m_cleanup(cleanup) {}
		~DCSelect() { ::SelectObject(m_hdc, m_old_obj); if (m_cleanup && m_obj) DeleteObject(m_obj); }
		operator GdiHandle() const { return m_obj; }
	};

	// RAII clip region
	struct DCClipRgn
	{
		HDC  m_hdc;
		HRGN m_old_rgn;
		DCClipRgn(HDC hdc, HRGN hrgn) :m_hdc(hdc) ,m_old_rgn() { if (::GetClipRgn(m_hdc, m_old_rgn) != -1 && SelectClipRgn(m_hdc, hrgn) != -1) throw std::exception("failed to set clip region"); }
		~DCClipRgn() { SelectClipRgn(m_hdc, m_old_rgn); }
	};

	// RAII dc graphics mode
	struct DCGfxMode
	{
		HDC m_hdc;
		int m_old_mode;
		DCGfxMode(HDC hdc, int mode) :m_hdc(hdc) ,m_old_mode(::SetGraphicsMode(m_hdc, mode)) {}
		~DCGfxMode() { ::SetGraphicsMode(m_hdc, m_old_mode); }
	};

	// RAII dc graphics mode
	struct DCBkMode
	{
		HDC m_hdc;
		int m_old_mode;
		DCBkMode(HDC hdc, int mode) :m_hdc(hdc) ,m_old_mode(::SetBkMode(m_hdc, mode)) {}
		~DCBkMode() { ::SetBkMode(m_hdc, m_old_mode); }
	};

	// Return the rectangular size of a string if drawn using 'font' in 'dc'
	inline SIZE MeasureString(HDC hdc, LPCTSTR string, size_t len, HFONT font)
	{
		DCSelect<HFONT> sel_font(hdc, font, false);
		SIZE size; GetTextExtentPoint32(hdc, string, (int)len, &size);
		return size;
	}

	// Construct a 2D gdi transform.
	// Remember to call SetGraphicsMode(), e.g:
	//   int old_mode = ::SetGraphicsMode(hdc, GM_ADVANCED);
	//   ...
	//   ::SetGraphicsMode(hdc, old_mode);
	inline XFORM MakeXFORM(float eM11, float eM12, float eM21, float eM22, float eDx, float eDy)
	{
		XFORM m = {eM11, eM12, eM21, eM22, eDx, eDy};
		return m;
	}

	// Save an HBITMAP to a file
	// 'hdc' is a device context that is compatible with the bitmap, used to extract device independent bits
	inline void SaveBmp(std::filesystem::path const& filepath, HBITMAP hbmp, HDC hdc)
	{
		// Retrieve the bitmap color format, width, and height.
		BITMAP bmp; if (!::GetObject(hbmp, sizeof(BITMAP), &bmp))
			throw std::exception("failed to get bitmap info");

		// Convert the color format to a count of bits.
		WORD clrbits = WORD(bmp.bmPlanes * bmp.bmBitsPixel);
		if      (clrbits == 1)  clrbits = 1;
		else if (clrbits <= 4)  clrbits = 4;
		else if (clrbits <= 8)  clrbits = 8;
		else if (clrbits <= 16) clrbits = 16;
		else if (clrbits <= 24) clrbits = 24;
		else                    clrbits = 32;
		DWORD clrs_used = (clrbits < 16) ? (1 << clrbits) : 0;

		typedef std::vector<unsigned char> Bytes;
		Bytes hdr(sizeof(BITMAPFILEHEADER) + sizeof(BITMAPINFOHEADER) + clrs_used * sizeof(RGBQUAD));
		unsigned char* ptr = &hdr[0];
		BITMAPFILEHEADER&   bmfh = *reinterpret_cast<BITMAPFILEHEADER*>(ptr); ptr += sizeof(BITMAPFILEHEADER);
		BITMAPINFOHEADER&   bmih = *reinterpret_cast<BITMAPINFOHEADER*>(ptr); ptr += sizeof(BITMAPINFOHEADER);
		RGBQUAD*            clrs =  reinterpret_cast<RGBQUAD*>(ptr);          ptr += sizeof(RGBQUAD) * clrs_used;
		BITMAPINFO&         bmi  =  reinterpret_cast<BITMAPINFO&>(bmih);
		bmih.biSize         = sizeof(BITMAPINFOHEADER);
		bmih.biWidth        = bmp.bmWidth;
		bmih.biHeight       = bmp.bmHeight;
		bmih.biPlanes       = bmp.bmPlanes;
		bmih.biBitCount     = bmp.bmBitsPixel;
		bmih.biClrUsed      = clrs_used;
		bmih.biCompression  = BI_RGB; // If the bitmap is not compressed, set the BI_RGB flag.
		bmih.biClrImportant = 0; // Set biClrImportant to 0, indicating that all of the device colors are important.
		bmih.biSizeImage    = ((((bmih.biWidth * bmih.biBitCount) + 31) & ~31) >> 3) * bmih.biHeight;
		bmfh.bfType         = 0x4d42; // "BM"
		bmfh.bfOffBits      = sizeof(BITMAPFILEHEADER) + sizeof(BITMAPINFOHEADER) + bmih.biClrUsed*sizeof(RGBQUAD);
		bmfh.bfSize         = bmfh.bfOffBits + bmih.biSizeImage;

		// Retrieve the color table (RGBQUAD array) and the bits (array of palette indices) from the DIB.
		Bytes bits(bmih.biSizeImage);
		if (!::GetDIBits(hdc, hbmp, UINT(0), UINT(bmih.biHeight), &bits[0], &bmi, DIB_RGB_COLORS))
			throw std::exception("failed to read bitmap bits");

		// Write the bitmap file
		std::ofstream file(filepath, std::ios::binary);
		if (!file ||
			!file.write(char_ptr(&bmfh), sizeof(bmfh)) ||
			!file.write(char_ptr(&bmih), sizeof(bmih)) ||
			!file.write(char_ptr(&clrs[0]), bmih.biClrUsed * sizeof(RGBQUAD)) ||
			!file.write(char_ptr(&bits[0]), bmih.biSizeImage))
			throw std::runtime_error("failed to write bmp file");
	}
}
