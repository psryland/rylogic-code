//*******************************************************
// GDI+ helpers
//  Copyright (c) Rylogic 2009
//*******************************************************
#pragma once

#ifndef NOGDI

#ifndef GDIPVER
#define GDIPVER 0x0110
#endif

#include <vector>
#include <filesystem>
#include <Windows.h>
#pragma warning(push)
#pragma warning(disable: 5260) // Windows SDK GdiplusEnums.h: const with internal/external linkage mismatch
#include <gdiplus.h>
#pragma warning(pop)
#include "pr/common/to.h"

#pragma comment(lib, "gdiplus.lib")
static_assert(GDIPVER == 0x0110, "");

namespace pr
{
	// Import the 'Gdiplus' namespace into 'pr::gdi'
	namespace gdi = Gdiplus;

	// RAII object for initialised/shutting down the GdiPlus framework
	struct GdiPlus
	{
		ULONG_PTR m_token;
		gdi::GdiplusStartupInput  m_startup_input;
		gdi::GdiplusStartupOutput m_startup_output;

		GdiPlus()  { gdi::GdiplusStartup(&m_token, &m_startup_input, &m_startup_output); }
		~GdiPlus() { gdi::GdiplusShutdown(m_token); }
	};

	namespace convert
	{
		// GDI types to string
		template <typename Str, typename = std::enable_if_t<is_string_v<Str>>>
		struct GdiToString
		{
		};

		// Whatever to GdiColour
		struct ToGdiColor
		{
			static gdi::Color Func(COLORREF col)
			{
				gdi::Color c;
				c.SetFromCOLORREF(col);
				return c;
			}
		};

		// Whatever to GdiRect
		struct ToGdiRect
		{
			static gdi::Rect Func(RECT const& r)
			{
				return gdi::Rect(r.left, r.top, r.right - r.left, r.bottom - r.top);
			}
			static gdi::Rect Func(gdi::RectF const& r)
			{
				return gdi::Rect(int(r.X), int(r.Y), int(r.Width), int(r.Height));
			}
		};

		// Whatever to GdiRectF
		struct ToGdiRectF
		{
			static gdi::RectF Func(RECT const& r)
			{
				return gdi::RectF(float(r.left), float(r.top), float(r.right - r.left), float(r.bottom - r.top));
			}
		};

		// GdiRect to RECT
		struct GdiRectToRECT
		{
			static RECT Func(gdi::RectF const& r)
			{
				return RECT{int(r.GetLeft()), int(r.GetTop()), int(r.GetRight()), int(r.GetBottom())};
			}
			static RECT Func(gdi::Rect const& r)
			{
				return RECT{r.GetLeft(), r.GetTop(), r.GetRight(), r.GetBottom()};
			}
		};
	}
	template <typename TFrom> struct Convert<gdi::Color, TFrom> :convert::ToGdiColor {};
	template <typename TFrom> struct Convert<gdi::Rect , TFrom> :convert::ToGdiRect {};
	template <typename TFrom> struct Convert<gdi::RectF, TFrom> :convert::ToGdiRectF {};
	template <> struct Convert<RECT, gdi::RectF> :convert::GdiRectToRECT {};
	template <> struct Convert<RECT, gdi::Rect> :convert::GdiRectToRECT {};
}

namespace Gdiplus
{
	// Singleton for accessing image codecs
	class ImageCodec
	{
		std::vector<ImageCodecInfo> m_codecs;
		ImageCodec()
		{
			UINT num = 0;  // number of image encoders
			UINT size = 0; // size of the image encoder array in bytes
			if (GetImageEncodersSize(&num, &size) != pr::gdi::Status::Ok)
				throw std::exception("GDI+ Image encoders not available");

			if (size != 0)
			{
				m_codecs.resize(size);
				GetImageEncoders(num, size, m_codecs.data());
			}
		}
		static ImageCodec const& This()
		{
			static ImageCodec inst;
			return inst;
		}

	public:
		static ImageCodecInfo const& Info(wchar_t const* mime)
		{
			for (auto& codec : This().m_codecs)
			{
				if (wcscmp(codec.MimeType, mime) != 0) continue;
				return codec;
			}
			throw std::exception("Image codec not found");
		}
		static CLSID const& Clsid(wchar_t const* mime)
		{
			return Info(mime).Clsid;
		}
	};

	// Helper for saving GDI Images that infers the codec from the file extension
	inline Status Save(Image const& image, std::filesystem::path const& filepath)
	{
		if (!filepath.has_extension())
			throw std::runtime_error("Image save could not infer the image format from the file extension");

		auto extn = filepath.extension().c_str() + 1; // skip the '.'
		auto mime = std::wstring(L"image/").append(extn);
		return const_cast<Image&>(image).Save(filepath.c_str(), &ImageCodec::Clsid(mime.c_str()));
	}

	// Helper for checking GDI return codes
	inline void Throw(Status result, char const* message)
	{
		if (result == Status::Ok) return;
		throw std::exception(message);
	}
}

// Update Manifest
// We use features from GDI+ v1.1 which is new as of Windows Vista. There is no re-distributable for Windows XP.
// This adds information to the .exe manifest to force GDI+ 1.1 version of gdiplus.dll to be loaded on Vista
// without this, Vista defaults to loading version 1.0 and our application will fail to launch with missing entry points.
#if defined _M_IX86
#pragma comment(linker, "\"/manifestdependency:type='Win32' name='Microsoft.Windows.GdiPlus' version='1.1.0.0' processorArchitecture='X86' publicKeyToken='6595b64144ccf1df' language='*'\"")
#elif defined _M_IA64
#pragma comment(linker, "\"/manifestdependency:type='Win32' name='Microsoft.Windows.GdiPlus' version='1.1.0.0' processorArchitecture='ia64' publicKeyToken='6595b64144ccf1df' language='*'\"")
#elif defined _M_X64
#pragma comment(linker, "\"/manifestdependency:type='Win32' name='Microsoft.Windows.GdiPlus' version='1.1.0.0' processorArchitecture='amd64' publicKeyToken='6595b64144ccf1df' language='*'\"")
#else
#pragma comment(linker, "\"/manifestdependency:type='Win32' name='Microsoft.Windows.GdiPlus' version='1.1.0.0' processorArchitecture='*' publicKeyToken='6595b64144ccf1df' language='*'\"")
#endif

#endif // NOGDI