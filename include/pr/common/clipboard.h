//*******************************************************
// Clipboard
//  Copyright (c) Rylogic Ltd 2007
//*******************************************************
#pragma once

#include <cstring>
#include <type_traits>
#include <windows.h>

namespace pr
{
	// The clipboard format for text made of 'Char'. Narrow text uses CF_TEXT, wide text uses CF_UNICODETEXT.
	template <typename Char> constexpr UINT ClipboardTextFormat()
	{
		// Only 'char' and 'wchar_t' have a standard clipboard text format
		static_assert(std::is_same_v<Char, char> || std::is_same_v<Char, wchar_t>, "Clipboard text must be 'char' or 'wchar_t'");
		return std::is_same_v<Char, char> ? CF_TEXT : CF_UNICODETEXT;
	}

	// Set some text on the clip board. The format is chosen from the character type of 'String'.
	template <typename String> bool SetClipBoardText(HWND hwnd, String const& str)
	{
		// Open and take ownership of the clipboard
		using Char = typename String::value_type;
		if (!OpenClipboard(hwnd)) return false;
		EmptyClipboard();

		// Allocate a global memory object for the text, including the null terminator
		auto size = (str.size() + 1) * sizeof(Char);
		auto hglb = GlobalAlloc(GMEM_MOVEABLE, size);
		if (!hglb) { CloseClipboard(); return false; }

		// Copy the text into the global memory
		auto* buf = GlobalLock(hglb);
		if (!buf) { GlobalFree(hglb); CloseClipboard(); return false; }
		memcpy(buf, str.c_str(), size);
		GlobalUnlock(hglb);

		// Place the handle on the clipboard. The clipboard owns the memory only if this succeeds.
		if (!SetClipboardData(ClipboardTextFormat<Char>(), hglb)) { GlobalFree(hglb); CloseClipboard(); return false; }
		CloseClipboard();
		return true;
	}

	// Get some text from the clip board. The format is chosen from the character type of 'String'.
	template <typename String> bool GetClipBoardText(HWND hwnd, String& str)
	{
		// Check text is available in the format that matches 'String'
		using Char = typename String::value_type;
		auto format = ClipboardTextFormat<Char>();
		if (!IsClipboardFormatAvailable(format)) return false;
		if (!OpenClipboard(hwnd)) return false;

		// Get the clipboard memory. The clipboard owns it, so it must not be freed here.
		auto hglb = GetClipboardData(format);
		if (!hglb) { CloseClipboard(); return false; }

		// Copy the null-terminated text out of the clipboard memory
		auto* text = static_cast<Char const*>(GlobalLock(hglb));
		if (!text) { CloseClipboard(); return false; }
		str = text;
		GlobalUnlock(hglb);
		CloseClipboard();
		return true;
	}
}
