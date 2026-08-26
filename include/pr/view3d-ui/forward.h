//*********************************************
// View3DUI
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#pragma once

#ifndef _WIN32_WINNT
#define _WIN32_WINNT _WIN32_WINNT_WIN10
#elif _WIN32_WINNT < _WIN32_WINNT_WIN10
#error "_WIN32_WINNT >= _WIN32_WINNT_WIN10 required"
#endif

// Standard and Windows dependencies shared by the View3DUI module. Sibling headers in this
// module include only this file and other pr/view3d-ui headers; foreign/STL/Win32 dependencies
// are centralized here so the dependency-minimal public ABI headers stay easy to audit.
#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <format>
#include <functional>
#include <initializer_list>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <type_traits>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <sdkddkver.h>
#include <windows.h>

// The build defines WIN32_LEAN_AND_MEAN, so <windows.h> does not pull in COM support; IUnknown is
// needed to AddRef/Release the externally-owned ID3D12Device without depending on d3d12.h here.
#include <unknwn.h>

// Microsoft::WRL::ComPtr is a header-only smart pointer for COM interfaces (AddRef/Release rather
// than the IncRef/DecRef ADL customization pr::RefPtr expects), used by the M3 renderer/text
// shaper for every D3D12/DirectWrite COM object they own.
#include <wrl/client.h>

// DirectWrite is used only by text_shaper.* (glyph shaping/CPU coverage rasterization for the M3
// renderer); it is centralized here, rather than in a narrower header, for the same reason as
// every other foreign dependency in this file (see the module comment above).
#include <dwrite.h>

// The Input Method Manager is used only by win32_input.* to read the composition/result strings a
// keyboard IME reports and to place its composition/candidate windows at the caret; it is
// centralized here, rather than included ad hoc by that source file, for the same reason as every
// other foreign dependency in this file (see the module comment above).
#include <imm.h>

// OLE Automation's BSTR/SAFEARRAY/VARIANT types and the Microsoft UI Automation provider
// interfaces are used only by uia_*.* to expose the semantic snapshot to assistive technology;
// they are centralized here, rather than included ad hoc by those source files, for the same
// reason as every other foreign dependency in this file (see the module comment above).
// <oleacc.h> supplies the WM_GETOBJECT object identifiers the provider must recognise.
#include <objbase.h>
#include <oleauto.h>
#include <oleacc.h>
#include <uiautomation.h>

// win32::LoadDll<Tag> is used by the eager dynamic facade (view3d-ui.h) to resolve view3d-ui.dll.
#include "pr/win32/win32.h"

namespace pr::view3d::ui
{
	class UiEngine;
	struct Context;
}
