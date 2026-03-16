//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
// The view3d dll is loaded once per application, although an application may have
// multiple windows and may call Initialise/Shutdown a number of times.
// Ldr objects can be created independently to windows. This means we need one global
// context within the dll, one renderer, and one list of objects.
//
// Error/Log handling:
//  Each window represents a separate context from the callers point of view, this
//  means we need an error handler per window. Also, within a window, callers may
//  want to temporarily push a different error handler. Each window maintains a
//  stack of error handlers.
#include "pr/view3d-12/view3d-dll.h"
#include "pr/view3d-12/ldraw/ldraw_ui_script_editor.h"
#include "pr/view3d-12/model/model.h"
#include "pr/view3d-12/resource/stock_resources.h"
#include "pr/view3d-12/resource/resource_factory.h"
#include "pr/view3d-12/texture/texture_desc.h"
#include "pr/view3d-12/texture/texture_2d.h"
#include "pr/view3d-12/texture/texture_cube.h"
#include "pr/view3d-12/sampler/sampler_desc.h"
#include "pr/view3d-12/sampler/sampler.h"
#include "pr/view3d-12/utility/dx9_context.h"
#include "pr/view3d-12/utility/conversion.h"
#include "pr/view3d-12/utility/utility.h"
#include "pr/view3d-12/ldraw/ldraw_object.h"
#include "pr/view3d-12/ldraw/ldraw_gizmo.h"
#include "pr/view3d-12/ldraw/ldraw_reader_text.h"
#include "pr/view3d-12/ldraw/ldraw_reader_binary.h"
#include "view3d-12/src/ldraw/sources/ldraw_sources.h"
#include "view3d-12/src/ldraw/sources/source_base.h"
#include "view3d-12/src/ldraw/sources/source_stream.h"
#include "view3d-12/src/dll/dll_forward.h"
#include "view3d-12/src/dll/context.h"
#include "view3d-12/src/dll/v3d_window.h"

using namespace pr;
using namespace pr::rdr12;
using namespace pr::view3d;

// DLL entry point
HINSTANCE g_hInstance;
BOOL APIENTRY DllMain(HINSTANCE hInstance, DWORD ul_reason_for_call, LPVOID)
{
	switch (ul_reason_for_call)
	{
	case DLL_PROCESS_ATTACH: g_hInstance = hInstance; break;
	case DLL_PROCESS_DETACH: g_hInstance = nullptr; break;
	case DLL_THREAD_ATTACH:  break;
	case DLL_THREAD_DETACH:  break;
	}
	return TRUE;
}

// Global DLL context
static Context* g_ctx = nullptr;
static Context& Dll()
{
	if (g_ctx) return *g_ctx;
	throw std::runtime_error("View3d not initialised");
}

// Types
using LockGuard = std::lock_guard<std::recursive_mutex>;

// Helper macros for exception trapping in API functions
#define DllLockGuard LockGuard lock(Dll().m_mutex)
#define CatchAndReport(func_name, wnd, ret)\
	catch (std::exception const& ex) { Dll().ReportAPIError(std::source_location::current().function_name(), view3d::Window(wnd), &ex); }\
	catch (...)                      { Dll().ReportAPIError(std::source_location::current().function_name(), view3d::Window(wnd), nullptr); }\
	return ret

// Dll Context ****************************

// Initialise calls are reference counted and must be matched with Shutdown calls
// 'initialise_error_cb' is used to report dll initialisation errors only (i.e. it isn't stored)
// Note: this function is not thread safe, avoid race calls
VIEW3D_API DllHandle  __stdcall View3D_Initialise(view3d::ReportErrorCB global_error_cb)
{
	try
	{
		// Create the dll context on the first call
		if (g_ctx == nullptr)
			g_ctx = new Context(g_hInstance, global_error_cb);

		// Generate a unique handle per Initialise call, used to match up with Shutdown calls
		static DllHandle handles = nullptr;
		g_ctx->m_inits.insert(++handles);
		return handles;
	}
	catch (std::exception const& e)
	{
		global_error_cb(FmtS("Failed to initialise View3D.\nReason: %s\n", e.what()), "", 0, 0);
		return nullptr;
	}
	catch (...)
	{
		global_error_cb("Failed to initialise View3D.\nReason: An unknown exception occurred\n", "", 0, 0);
		return nullptr;
	}
}
VIEW3D_API void __stdcall View3D_Shutdown(DllHandle context)
{
	if (!g_ctx) return;

	g_ctx->m_inits.erase(context);
	if (!g_ctx->m_inits.empty())
		return;

	delete g_ctx;
	g_ctx = nullptr;
}

// Replace the global error handler
VIEW3D_API void __stdcall View3D_GlobalErrorCBSet(view3d::ReportErrorCB error_cb, BOOL add)
{
	try
	{
		DllLockGuard;
		if (add)
			Dll().ReportError += error_cb;
		else
			Dll().ReportError -= error_cb;
	}
	CatchAndReport(View3D_GlobalErrorCBSet, , );
}

// Set the callback for progress events when script sources are loaded or updated
VIEW3D_API void __stdcall View3D_ParsingProgressCBSet(view3d::ParsingProgressCB progress_cb, BOOL add)
{
	try
	{
		DllLockGuard;
		if (add)
			Dll().ParsingProgress += progress_cb;
		else
			Dll().ParsingProgress -= progress_cb;
	}
	CatchAndReport(View3D_ParsingProgressCBSet,,);
}

// Set the callback that is called when the sources are reloaded
VIEW3D_API void __stdcall View3D_SourcesChangedCBSet(view3d::SourcesChangedCB sources_changed_cb, BOOL add)
{
	try
	{
		DllLockGuard;
		if (add)
			Dll().SourcesChanged += sources_changed_cb;
		else
			Dll().SourcesChanged -= sources_changed_cb;
	}
	CatchAndReport(View3D_SourcesChangedCBSet,,);
}

// Return the context id for objects created from 'filepath' (if filepath is an existing source)
VIEW3D_API GUID __stdcall View3D_ContextIdFromFilepath(char const* filepath)
{
	try
	{
		DllLockGuard;
		return rdr12::ldraw::ContextIdFromFilepath(filepath);
	}
	CatchAndReport(View3D_ContextIdFromFilepath,,GuidZero);
}

// Data Sources ***************************

// Create an include handler that can load from directories or embedded resources
static PathResolver GetIncludes(view3d::Includes const* includes)
{
	if (includes == nullptr)
		return {};

	PathResolver inc;
	if (includes->m_include_paths != nullptr)
		inc.SearchPathList(includes->m_include_paths);

	if (includes->m_module_count != 0)
		inc.ResourceModules(std::initializer_list<HMODULE>(includes->m_modules, includes->m_modules + includes->m_module_count));

	return inc;
}

// Add an ldr script source. This will create all objects with context id 'context_id' (if given, otherwise an id will be created). Concurrent calls are thread safe.
VIEW3D_API GUID __stdcall View3D_LoadScriptFromString(char const* ldr_script, GUID const* context_id, view3d::Includes const* includes, view3d::AddCompleteCB on_add_cb)
{
	try
	{
		// Concurrent entry is allowed
		rdr12::ldraw::AddCompleteCB on_add = on_add_cb
			? rdr12::ldraw::AddCompleteCB(on_add_cb)
			: static_cast<rdr12::ldraw::AddCompleteCB>(nullptr);

		return Dll().LoadScriptString(std::string_view(ldr_script), EEncoding::utf8, context_id, GetIncludes(includes), on_add);
	}
	CatchAndReport(View3D_LoadScriptFromString, (view3d::Window)nullptr, GuidZero);
}
VIEW3D_API GUID __stdcall View3D_LoadScriptFromFile(char const* ldr_file, GUID const* context_id, view3d::Includes const* includes, view3d::AddCompleteCB on_add_cb)
{
	try
	{
		// Concurrent entry is allowed
		auto on_add = on_add_cb
			? rdr12::ldraw::AddCompleteCB(on_add_cb)
			: static_cast<rdr12::ldraw::AddCompleteCB>(nullptr);

		return Dll().LoadScriptFile(std::filesystem::path(ldr_file), EEncoding::auto_detect, context_id, GetIncludes(includes), on_add);
	}
	CatchAndReport(View3D_LoadScriptFromFile, (view3d::Window)nullptr, GuidZero);
}

// Cancel an in-progress load operation
VIEW3D_API void __stdcall View3D_CancelLoad(GUID const* context_id)
{
	try
	{
		// Concurrent entry is allowed
		Dll().CancelLoad(*context_id);
	}
	CatchAndReport(View3D_CancelLoad, , );
}

// Enumerate all sources in the store
VIEW3D_API void __stdcall View3D_EnumSources(view3d::EnumGuidsCB enum_guids_cb)
{
	try
	{
		DllLockGuard;
		Dll().EnumSources(enum_guids_cb);
	}
	CatchAndReport(View3D_EnumSources,, );
}

// Reload objects from the source associated with 'context_id'
VIEW3D_API void __stdcall View3D_SourceReload(GUID const& context_id)
{
	try
	{
		DllLockGuard;
		return Dll().ReloadScriptSources({ &context_id, 1 });
	}
	CatchAndReport(View3D_SourceReload, , );
}

// Delete all objects and remove the source associated with 'context_id'
VIEW3D_API void __stdcall View3D_SourceDelete(GUID const& context_id)
{
	try
	{
		DllLockGuard;
		return Dll().DeleteAllObjectsById(GuidPredCB{ (void*)&context_id, [](void* ctx, Guid const& id) { return *static_cast<Guid const*>(ctx) == id; } });
	}
	CatchAndReport(View3D_SourceDelete, , );
}

// Get information about a source
VIEW3D_API view3d::SourceInfo __stdcall View3D_SourceInfo(GUID const& context_id)
{
	try
	{
		DllLockGuard;
		return Dll().SourceInfo(context_id);
	}
	CatchAndReport(View3D_SourceInfo, , {});
}

// Get/Set the name of a source
VIEW3D_API BSTR __stdcall View3D_SourceNameGetBStr(GUID const& context_id)
{
	try
	{
		DllLockGuard;
		auto const& src_name = Dll().SourceName(context_id);
		auto name = Widen(src_name);
		return ::SysAllocStringLen(name.c_str(), UINT(name.size()));
	}
	CatchAndReport(View3D_SourceNameGetBStr, , {});
}
VIEW3D_API char const* __stdcall View3D_SourceNameGet(GUID const& context_id)
{
	try
	{
		DllLockGuard;
		return Dll().SourceName(context_id).c_str();
	}
	CatchAndReport(View3D_SourceNameGet, , {});
}
VIEW3D_API void __stdcall View3D_SourceNameSet(GUID const& context_id, char const* name)
{
	try
	{
		DllLockGuard;
		Dll().SourceName(context_id, name);
	}
	CatchAndReport(View3D_SourceNameSet,,);
}

// Reload script sources. This will delete all objects associated with the script sources then reload the files creating new objects with the same context ids.
VIEW3D_API void __stdcall View3D_ReloadScriptSources()
{
	try
	{
		DllLockGuard;
		return Dll().ReloadScriptSources();
	}
	CatchAndReport(View3D_ReloadScriptSources,,);
}

// Delete all objects and object sources
VIEW3D_API void __stdcall View3D_DeleteAllObjects()
{
	try
	{
		DllLockGuard;
		Dll().DeleteAllObjects();
	}
	CatchAndReport(View3D_DeleteAllObjects, ,);
}

// Delete all objects matching (or not matching) a context id
VIEW3D_API void __stdcall View3D_DeleteById(view3d::GuidPredCB pred)
{
	try
	{
		DllLockGuard;
		Dll().DeleteAllObjectsById(pred);
	}
	CatchAndReport(View3D_DeleteById, ,);
}

// Delete all objects not displayed in any windows
VIEW3D_API void __stdcall View3D_DeleteUnused(view3d::GuidPredCB pred)
{
	try
	{
		DllLockGuard;
		Dll().DeleteUnused(pred);
	}
	CatchAndReport(View3D_DeleteUnused, ,);
}

// Poll for changed script sources and reload any that have changed
VIEW3D_API void __stdcall View3D_CheckForChangedSources()
{
	try
	{
		DllLockGuard;
		return Dll().CheckForChangedSources();
	}
	CatchAndReport(View3D_CheckForChangedSources,,);
}

// Enable/Disable streaming script sources.
VIEW3D_API EStreamingState __stdcall View3D_StreamingState()
{
	try
	{
		DllLockGuard;
		switch (Dll().StreamingState())
		{
			case rdr12::ldraw::EStreamingState::Disconnected: return view3d::EStreamingState::Disconnected;
			case rdr12::ldraw::EStreamingState::Listening: return view3d::EStreamingState::Listening;
			case rdr12::ldraw::EStreamingState::Connected: return view3d::EStreamingState::Connected;
			default: throw std::runtime_error("Unknown streaming state");
		}
	}
	CatchAndReport(View3D_StreamingState,,view3d::EStreamingState::Disconnected);
}
VIEW3D_API void __stdcall View3D_StreamingEnable(BOOL enable, int port)
{
	try
	{
		DllLockGuard;

		if ((port & 0xFFFF) != port)
			throw std::runtime_error("Invalid port for ldraw streaming");

		return Dll().Streaming(enable != 0, s_cast<uint16_t>(port));
	}
	CatchAndReport(View3D_StreamingEnable,,);
}

// Windows ********************************

// Create/Destroy a window
VIEW3D_API view3d::Window __stdcall View3D_WindowCreate(HWND hwnd, view3d::WindowOptions const& opts)
{
	try
	{
		DllLockGuard;
		return Dll().WindowCreate(hwnd, opts);
	}
	CatchAndReport(View3D_WindowCreate,, nullptr);
}
VIEW3D_API void __stdcall View3D_WindowDestroy(view3d::Window window)
{
	try
	{
		if (window == nullptr)
			return;

		DllLockGuard;
		Dll().WindowDestroy(window);
	}
	CatchAndReport(View3D_WindowDestroy,window,);
}

// Add/Remove a window error callback. Note: The callback function can be called in a worker thread context.
VIEW3D_API void __stdcall View3D_WindowErrorCBSet(view3d::Window window, view3d::ReportErrorCB error_cb, BOOL add)
{
	try
	{
		Validate(window);

		if (add)
			window->ReportError += error_cb;
		else
			window->ReportError -= error_cb;
	}
	CatchAndReport(View3D_WindowErrorCBSet, window, );
}

// Get/Set the window settings (as ldr script string)
VIEW3D_API BSTR __stdcall View3D_WindowSettingsGetBStr(pr::view3d::Window window)
{
	try
	{
		Validate(window);
		auto settings = Widen(window->Settings());
		return ::SysAllocStringLen(settings.c_str(), UINT(settings.size()));
	}
	CatchAndReport(View3D_WindowSettingsGetBStr, , {});
}
VIEW3D_API char const* __stdcall View3D_WindowSettingsGet(view3d::Window window)
{
	try
	{
		Validate(window);
		return window->Settings();
	}
	CatchAndReport(View3D_WindowSettingsGet, window, "");
}
VIEW3D_API void __stdcall View3D_WindowSettingsSet(view3d::Window window, char const* settings)
{
	try
	{
		Validate(window);
		window->Settings(settings);
	}
	CatchAndReport(View3D_WindowSettingsSet, window,);
}

// Get/Set the dimensions of the render target
// In set, if 'width' and 'height' are zero, the RT is resized to the associated window automatically.
VIEW3D_API SIZE __stdcall View3D_WindowBackBufferSizeGet(view3d::Window window)
{
	try
	{
		Validate(window);

		DllLockGuard;
		auto area = window->BackBufferSize();
		return To<SIZE>(area);
	}
	CatchAndReport(View3D_WindowBackBufferSizeGet, window, {});
}
VIEW3D_API void __stdcall View3D_WindowBackBufferSizeSet(view3d::Window window, SIZE size, BOOL force_recreate)
{
	try
	{
		Validate(window);

		DllLockGuard;
		window->BackBufferSize(To<iv2>(size), force_recreate != 0);
	}
	CatchAndReport(View3D_WindowBackBufferSizeSet, window,);
}

// Get/Set the window viewport (and clipping area)
VIEW3D_API view3d::Viewport __stdcall View3D_WindowViewportGet(view3d::Window window)
{
	try
	{
		Validate(window);

		DllLockGuard;
		return window->Viewport();
	}
	CatchAndReport(View3D_WindowViewportGet, window, view3d::Viewport{});
}
VIEW3D_API void __stdcall View3D_WindowViewportSet(view3d::Window window, view3d::Viewport const& vp)
{
	try
	{
		Validate(window);

		DllLockGuard;
		window->Viewport(vp);
	}
	CatchAndReport(View3D_WindowViewportSet, window,);
}

// Set a notification handler for when a window setting changes
VIEW3D_API void __stdcall View3D_WindowSettingsChangedCB(view3d::Window window, view3d::SettingsChangedCB settings_changed_cb, BOOL add)
{
	try
	{
		Validate(window);

		if (add)
			window->OnSettingsChanged += settings_changed_cb;
		else
			window->OnSettingsChanged -= settings_changed_cb;
	}
	CatchAndReport(View3D_WindowSettingsChangedCB, window,);
}

// Add/Remove a callback that is called when the collection of objects associated with 'window' changes
VIEW3D_API void __stdcall View3D_WindowSceneChangedCB(view3d::Window window, view3d::SceneChangedCB scene_changed_cb, BOOL add)
{
	try
	{
		Validate(window);

		if (add)
			window->OnSceneChanged += scene_changed_cb;
		else
			window->OnSceneChanged -= scene_changed_cb;
	}
	CatchAndReport(View3D_WindowSceneChangedCB, window, );
}

// Add/Remove a callback that is called just prior to rendering the window
VIEW3D_API void __stdcall View3D_WindowRenderingCB(view3d::Window window, view3d::RenderingCB rendering_cb, BOOL add)
{
	try
	{
		Validate(window);

		if (add)
			window->OnRendering += rendering_cb;
		else
			window->OnRendering -= rendering_cb;
	}
	CatchAndReport(View3D_WindowRenderingCB, window,);
}

// Add/Remove an object to/from a window
VIEW3D_API void __stdcall View3D_WindowAddObject(view3d::Window window, view3d::Object object)
{
	try
	{
		Validate(window);
		Validate(object);
		
		DllLockGuard;
		window->Add(object);
	}
	CatchAndReport(View3D_WindowAddObject, window,);
}
VIEW3D_API void __stdcall View3D_WindowRemoveObject(view3d::Window window, view3d::Object object)
{
	try
	{
		Validate(window);
		Validate(object);

		DllLockGuard;
		window->Remove(object);
	}
	CatchAndReport(View3D_WindowRemoveObject, window,);
}

// Add/Remove an gizmo to/from a window
VIEW3D_API void __stdcall View3D_WindowAddGizmo(view3d::Window window, view3d::Gizmo gizmo)
{
	try
	{
		Validate(window);
		Validate(gizmo);
		
		DllLockGuard;
		window->Add(gizmo);
	}
	CatchAndReport(View3D_WindowAddGizmo, window,);
}
VIEW3D_API void __stdcall View3D_WindowRemoveGizmo(view3d::Window window, view3d::Gizmo gizmo)
{
	try
	{
		Validate(window);
		Validate(gizmo);

		DllLockGuard;
		window->Remove(gizmo);
	}
	CatchAndReport(View3D_WindowRemoveGizmo, window,);
}

// Add/Remove objects by context id. This function can be used to add all objects either in, or not in 'context_ids'
VIEW3D_API void __stdcall View3D_WindowAddObjectsById(view3d::Window window, view3d::GuidPredCB pred)
{
	try
	{
		Validate(window);

		DllLockGuard;
		window->Add(Dll().m_sources.Sources(), pred);
	}
	CatchAndReport(View3D_WindowAddObjectsById, window,);
}
VIEW3D_API void __stdcall View3D_WindowRemoveObjectsById(view3d::Window window, view3d::GuidPredCB pred)
{
	try
	{
		Validate(window);

		DllLockGuard;
		window->Remove(pred, false);
	}
	CatchAndReport(View3D_WindowRemoveObjectsById, window,);
}

// Remove all objects 'window'
VIEW3D_API void __stdcall View3D_WindowRemoveAllObjects(view3d::Window window)
{
	try
	{
		Validate(window);

		DllLockGuard;
		window->RemoveAllObjects();
	}
	CatchAndReport(View3D_WindowRemoveAllObjects, window,);
}

// Enumerate the GUIDs associated with 'window'
VIEW3D_API void __stdcall View3D_WindowEnumGuids(view3d::Window window, view3d::EnumGuidsCB enum_guids_cb)
{
	try
	{
		Validate(window);

		DllLockGuard;
		window->EnumGuids(enum_guids_cb);
	}
	CatchAndReport(View3D_WindowEnumGuids, window, );
}

// Enumerate the objects associated with 'window'
VIEW3D_API void __stdcall View3D_WindowEnumObjects(view3d::Window window, view3d::EnumObjectsCB enum_objects_cb)
{
	try
	{
		Validate(window);

		DllLockGuard;
		window->EnumObjects(enum_objects_cb);
	}
	CatchAndReport(View3D_WindowEnumObjects, window, );
}
VIEW3D_API void __stdcall View3D_WindowEnumObjectsById(view3d::Window window, view3d::EnumObjectsCB enum_objects_cb, view3d::GuidPredCB pred)
{
	try
	{
		Validate(window);

		DllLockGuard;
		window->EnumObjects(enum_objects_cb, pred);
	}
	CatchAndReport(View3D_WindowEnumObjectsById, window, );
}

// Return true if 'object' is among 'window's objects
VIEW3D_API BOOL __stdcall View3D_WindowHasObject(view3d::Window window, view3d::Object object, BOOL search_children)
{
	try
	{
		Validate(window);;
		Validate(object);

		DllLockGuard;
		return window->Has(object, search_children != 0);
	}
	CatchAndReport(View3D_WindowHasObject, window, false);
}

// Return the number of objects assigned to 'window'
VIEW3D_API int __stdcall View3D_WindowObjectCount(view3d::Window window)
{
	try
	{
		Validate(window);

		DllLockGuard;
		return window->ObjectCount();
	}
	CatchAndReport(View3D_WindowObjectCount, window, 0);
}

// Return the bounds of a scene
VIEW3D_API view3d::BBox __stdcall View3D_WindowSceneBounds(view3d::Window window, view3d::ESceneBounds bounds, int except_count, GUID const* except)
{
	try
	{
		Validate(window);

		DllLockGuard;
		return To<view3d::BBox>(window->SceneBounds(bounds, except_count, except));
	}
	CatchAndReport(View3D_WindowSceneBounds, window, To<view3d::BBox>(pr::BBox::Unit()));
}

// Render the window
VIEW3D_API void __stdcall View3D_WindowRender(view3d::Window window)
{
	try
	{
		Validate(window);

		DllLockGuard;
		window->Render();
	}
	CatchAndReport(View3D_WindowRender, window,);
}

// Wait for any previous frames to complete rendering within the GPU
VIEW3D_API void __stdcall View3D_WindowGSyncWait(view3d::Window window)
{
	try
	{
		Validate(window);

		DllLockGuard;
		window->GSyncWait();
	}
	CatchAndReport(View3D_WindowPresent, window,);
}

// Replace the swap chain buffers with 'targets'
VIEW3D_API void __stdcall View3D_WindowCustomSwapChain(view3d::Window window, int count, view3d::Texture* targets)
{
	try
	{
		Validate(window);

		DllLockGuard;
		window->CustomSwapChain(std::span<Texture2D*>(targets, s_cast<size_t>(count)));
	}
	CatchAndReport(View3D_WindowCustomSwapChain, window, );
}

// Get the MSAA back buffer (render target + depth stencil)
VIEW3D_API view3d::BackBuffer __stdcall View3D_WindowRenderTargetGet(view3d::Window window)
{
	try
	{
		Validate(window);

		DllLockGuard;
		auto& bb = window->RenderTarget();
		auto sz = SIZE{};
		if (bb.m_render_target)
		{
			auto desc = bb.m_render_target->GetDesc();
			sz.cx = s_cast<int>(desc.Width);
			sz.cy = s_cast<int>(desc.Height);
		}
		return view3d::BackBuffer{
			.m_render_target = bb.m_render_target.get(),
			.m_depth_stencil = bb.m_depth_stencil.get(),
			.m_dim = sz,
		};
	}
	CatchAndReport(View3D_WindowRenderTargetGet, window, {});
}

// Call InvalidateRect on the HWND associated with 'window'
VIEW3D_API void __stdcall View3D_WindowInvalidate(view3d::Window window, BOOL erase)
{
	try
	{
		Validate(window);
		window->Invalidate(erase != 0);
	}
	CatchAndReport(View3D_Invalidate, window, );
}

// Call InvalidateRect on the HWND associated with 'window'
VIEW3D_API void __stdcall View3D_WindowInvalidateRect(view3d::Window window, RECT const& rect, BOOL erase)
{
	try
	{
		Validate(window);
		window->InvalidateRect(&rect, erase != 0);
	}
	CatchAndReport(View3D_InvalidateRect, window,);
}

// Register a callback for when the window is invalidated. This can be used to render in response to invalidation, rather than rendering on a polling cycle.
VIEW3D_API void __stdcall View3D_WindowInvalidatedCB(view3d::Window window, view3d::InvalidatedCB invalidated_cb, BOOL add)
{
	try
	{
		Validate(window);

		if (add)
			window->OnInvalidated += invalidated_cb;
		else
			window->OnInvalidated -= invalidated_cb;
	}
	CatchAndReport(View3D_WindowInvalidatedCB, window,);
}

// Clear the 'invalidated' state of the window.
VIEW3D_API void __stdcall View3D_WindowValidate(view3d::Window window)
{
	try
	{
		Validate(window);

		DllLockGuard;
		window->Validate();
	}
	CatchAndReport(View3D_Validate, window, );
}

// Get/Set the window background colour
VIEW3D_API unsigned int __stdcall View3D_WindowBackgroundColourGet(view3d::Window window)
{
	try
	{
		Validate(window);

		DllLockGuard;
		return window->BackgroundColour().argb().argb;
	}
	CatchAndReport(View3D_WindowBackgroundColourGet, window, 0U);
}
VIEW3D_API void __stdcall View3D_WindowBackgroundColourSet(view3d::Window window, unsigned int argb)
{
	try
	{
		Validate(window);

		DllLockGuard;
		window->BackgroundColour(pr::Colour(Colour32(argb)));
	}
	CatchAndReport(View3D_WindowBackgroundColourSet, window,);
}

// Get/Set the fill mode for the window
VIEW3D_API view3d::EFillMode __stdcall View3D_WindowFillModeGet(view3d::Window window)
{
	try
	{
		Validate(window);

		DllLockGuard;
		return static_cast<view3d::EFillMode>(window->FillMode());
	}
	CatchAndReport(View3D_WindowFillModeGet, window, {});
}
VIEW3D_API void __stdcall View3D_WindowFillModeSet(view3d::Window window, view3d::EFillMode mode)
{
	try
	{
		Validate(window);

		DllLockGuard;
		window->FillMode(static_cast<rdr12::EFillMode>(mode));
	}
	CatchAndReport(View3D_WindowFillModeSet, window,);
}

// Get/Set the cull mode for a faces in window
VIEW3D_API view3d::ECullMode __stdcall View3D_WindowCullModeGet(view3d::Window window)
{
	try
	{
		Validate(window);

		DllLockGuard;
		return static_cast<view3d::ECullMode>(window->CullMode());
	}
	CatchAndReport(View3D_CullModeGet, window, {});
}
VIEW3D_API void __stdcall View3D_WindowCullModeSet(view3d::Window window, view3d::ECullMode mode)
{
	try
	{
		Validate(window);

		DllLockGuard;
		window->CullMode(static_cast<rdr12::ECullMode>(mode));
	}
	CatchAndReport(View3D_CullModeSet, window,);
}

// Get/Set the multi-sampling mode for a window
VIEW3D_API int  __stdcall View3D_MultiSamplingGet(view3d::Window window)
{
	try
	{
		Validate(window);

		DllLockGuard;
		return window->MultiSampling();
	}
	CatchAndReport(View3D_MultiSamplingGet, window, {});
}
VIEW3D_API void __stdcall View3D_MultiSamplingSet(view3d::Window window, int multisampling)
{
	try
	{
		Validate(window);

		DllLockGuard;
		window->MultiSampling(multisampling);
	}
	CatchAndReport(View3D_MultiSamplingSet, window, );
}

// Control animation
VIEW3D_API void __stdcall View3D_WindowAnimControl(view3d::Window window, view3d::EAnimCommand command, double time_s)
{
	try
	{
		Validate(window);

		DllLockGuard;
		window->AnimControl(command, seconds_t(time_s));
	}
	CatchAndReport(View3D_WindowAnimControl, window, );
}

// Set the callback for animation events
VIEW3D_API void __stdcall View3D_WindowAnimEventCBSet(view3d::Window window, view3d::AnimationCB anim_cb, BOOL add)
{
	try
	{
		Validate(window);

		DllLockGuard;
		if (add)
			window->OnAnimationEvent += anim_cb;
		else
			window->OnAnimationEvent -= anim_cb;
	}
	CatchAndReport(View3D_AnimationEventCBSet, , );
}

// Get/Set the animation time
VIEW3D_API BOOL __stdcall View3D_WindowAnimating(view3d::Window window)
{
	try
	{
		Validate(window);

		DllLockGuard;
		return window->Animating();
	}
	CatchAndReport(View3D_WindowAnimating, window, FALSE);
}
VIEW3D_API double __stdcall View3D_WindowAnimTimeGet(view3d::Window window)
{
	try
	{
		Validate(window);

		DllLockGuard;
		return window->AnimTime().count();
	}
	CatchAndReport(View3D_WindowAnimTimeGet, window, 0.0f);
}
VIEW3D_API void __stdcall View3D_WindowAnimTimeSet(view3d::Window window, double time_s)
{
	try
	{
		Validate(window);

		DllLockGuard;
		window->AnimTime(seconds_t(time_s));
	}
	CatchAndReport(View3D_WindowAnimTimeSet, window, );
}

// Return the DPI of the monitor that 'window' is displayed on
VIEW3D_API view3d::Vec2 __stdcall View3D_WindowDpiScale(view3d::Window window)
{
	try
	{
		Validate(window);

		DllLockGuard;
		return To<view3d::Vec2>(DIPtoPhysical(v2::One(), window->Dpi()));
	}
	CatchAndReport(View3d_WindowDPI, window, {});
}

// Set the global environment map for the window
VIEW3D_API void __stdcall View3D_WindowEnvMapSet(view3d::Window window, view3d::CubeMap env_map)
{
	try
	{
		Validate(window);

		DllLockGuard;
		window->EnvMap(env_map);
	}
	CatchAndReport(View3D_WindowEnvMapSet, window, );
}

// Enable/Disable the depth buffer
VIEW3D_API BOOL __stdcall View3D_DepthBufferEnabledGet(view3d::Window window)
{
	try
	{
		Validate(window);

		DllLockGuard;
		return window->DepthBufferEnabled();
	}
	CatchAndReport(View3D_DepthBufferEnabledGet, window, TRUE);
}
VIEW3D_API void __stdcall View3D_DepthBufferEnabledSet(view3d::Window window, BOOL enabled)
{
	try
	{
		Validate(window);

		DllLockGuard;
		return window->DepthBufferEnabled(enabled != 0);
	}
	CatchAndReport(View3D_DepthBufferEnabledSet, window,);
}

// Cast a ray into the scene, returning information about what it hit.
// 'rays' - is an input buffer of rays to cast for hit testing
// 'hits' - are the nearest intercepts with the given rays
// 'ray_count' - is the length of the 'rays' array
// 'snap_distance' - the world space distance to snap to
// 'flags' - what can be hit.
// 'objects' - An array of objects to hit test
// 'object_count' - The length of the 'objects' array.
VIEW3D_API void __stdcall View3D_WindowHitTestObjects(view3d::Window window, view3d::HitTestRay const* rays, view3d::HitTestResult* hits, int ray_count, view3d::Object const* objects, int object_count)
{
	try
	{
		Validate(window);

		DllLockGuard;
		window->HitTest({ rays, s_cast<size_t>(ray_count) }, { hits, s_cast<size_t>(ray_count) }, objects, object_count);
	}
	CatchAndReport(View3D_WindowHitTestObjects, window, );
}

// Cast a ray into the scene, returning information about what it hit
// 'rays' - is an input buffer of rays to cast for hit testing
// 'hits' - are the nearest intercepts with the given rays
// 'ray_count' - is the length of the 'rays' array
// 'snap_distance' - the world space distance to snap to
// 'flags' - what can be hit.
// 'context_ids' - context ids for objects to include/exclude from hit testing
// 'include_count' - the number of context ids that should be included
// 'exclude_count' - the number of context ids that should be excluded
// 'include_count+exclude_count' = the length of the 'context_ids' array. If 0, then all context ids are included for hit testing
VIEW3D_API void __stdcall View3D_WindowHitTestByCtx(view3d::Window window, view3d::HitTestRay const* rays, view3d::HitTestResult* hits, int ray_count, view3d::GuidPredCB pred)
{
	try
	{
		Validate(window);

		DllLockGuard;
		auto ray_span = std::span<view3d::HitTestRay const>{ rays, s_cast<size_t>(ray_count) };
		auto hit_results = std::span<view3d::HitTestResult>{ hits, s_cast<size_t>(ray_count) };
		window->HitTest(ray_span, hit_results, pred, 0);
	}
	CatchAndReport(View3D_WindowHitTestByCtx, window, );
}

// Trigger execution of the async hit test rays. Submits GPU work and returns immediately.
VIEW3D_API void __stdcall View3D_HitTestAsync(view3d::Window window)
{
	try
	{
		Validate(window);

		DllLockGuard;
		window->HitTestAsync();
	}
	CatchAndReport(View3D_HitTestAsync, window, );
}

// Add/Remove a callback for receiving async hit test results
VIEW3D_API void __stdcall View3D_HitTestAsyncCBSet(view3d::Window window, view3d::HitTestAsyncCB cb, BOOL add)
{
	try
	{
		Validate(window);
		if (!cb) throw std::runtime_error("Callback function is null");

		DllLockGuard;
		if (add)
			window->OnHitTestAsyncResults += cb;
		else
			window->OnHitTestAsyncResults -= cb;
	}
	CatchAndReport(View3D_HitTestAsyncCBSet, ,);
}

// Add/Update/Remove an async hit test ray.
VIEW3D_API view3d::HitTestRayId __stdcall View3D_HitTestRayUpdate(view3d::Window window, view3d::HitTestRayId id, view3d::HitTestRay const* ray)
{
	try
	{
		Validate(window);

		DllLockGuard;
		return window->HitTestRayUpdate(id, ray);
	}
	CatchAndReport(View3D_HitTestRayUpdate, window, {});
}

// Camera *********************************

// Position the camera and focus distance
VIEW3D_API void __stdcall View3D_CameraPositionSet(view3d::Window window, view3d::Vec4 position, view3d::Vec4 lookat, view3d::Vec4 up)
{
	try
	{
		Validate(window);

		DllLockGuard;
		window->m_scene.m_cam.LookAt(To<v4>(position), To<v4>(lookat), To<v4>(up), true);
	}
	CatchAndReport(View3D_CameraPositionSet, window,);
}

// Get/Set the current camera to world transform
VIEW3D_API view3d::Mat4x4 __stdcall View3D_CameraToWorldGet(view3d::Window window)
{
	try
	{
		Validate(window);

		DllLockGuard;
		return To<view3d::Mat4x4>(window->m_scene.m_cam.CameraToWorld());
	}
	CatchAndReport(View3D_CameraToWorldGet, window, view3d::Mat4x4{});
}
VIEW3D_API void __stdcall View3D_CameraToWorldSet(view3d::Window window, view3d::Mat4x4 const& c2w)
{
	try
	{
		Validate(window);

		DllLockGuard;
		window->m_scene.m_cam.CameraToWorld(To<m4x4>(c2w));
	}
	CatchAndReport(View3D_CameraToWorldSet, window,);
}

// Move the camera to a position that can see the whole scene. Set 'dist' to 0 to preserve the FoV, or a distance to set the FoV
VIEW3D_API void __stdcall View3D_ResetView(view3d::Window window, view3d::Vec4 forward, view3d::Vec4 up, float dist, BOOL preserve_aspect, BOOL commit)
{
	try
	{
		Validate(window);

		DllLockGuard;
		window->ResetView(To<v4>(forward), To<v4>(up), dist, preserve_aspect != 0, commit != 0);
	}
	CatchAndReport(View3D_ResetView, window,);
}

// Reset the camera to view a bbox. Set 'dist' to 0 to preserve the FoV, or a distance to set the FoV
VIEW3D_API void __stdcall View3D_ResetViewBBox(view3d::Window window, view3d::BBox bbox, view3d::Vec4 forward, view3d::Vec4 up, float dist, BOOL preserve_aspect, BOOL commit)
{
	try
	{
		Validate(window);

		DllLockGuard;
		window->ResetView(To<pr::BBox>(bbox), To<v4>(forward), To<v4>(up), dist, preserve_aspect != 0, commit != 0);
	}
	CatchAndReport(View3D_ResetViewBBox, window,);
}

// Enable/Disable orthographic projection
VIEW3D_API BOOL __stdcall View3D_CameraOrthographicGet(view3d::Window window)
{
	try
	{
		Validate(window);

		DllLockGuard;
		return window->Orthographic();
	}
	CatchAndReport(View3D_CameraOrthographicGet, window, FALSE);
}
VIEW3D_API void __stdcall View3D_CameraOrthographicSet(view3d::Window window, BOOL on)
{
	try
	{
		Validate(window);

		DllLockGuard;
		window->Orthographic(on != 0);
	}
	CatchAndReport(View3D_CameraOrthographicSet, window,);
}

// Get/Set the distance to the camera focus point
VIEW3D_API float __stdcall View3D_CameraFocusDistanceGet(view3d::Window window)
{
	try
	{
		Validate(window);

		DllLockGuard;
		return window->FocusDistance();
	}
	CatchAndReport(View3D_CameraFocusDistanceGet, window, 0.0f);
}
VIEW3D_API void __stdcall View3D_CameraFocusDistanceSet(view3d::Window window, float dist)
{
	try
	{
		Validate(window);

		DllLockGuard;
		window->FocusDistance(dist);
	}
	CatchAndReport(View3D_CameraFocusDistanceSet, window,);
}

// Get/Set the camera focus point position
VIEW3D_API view3d::Vec4 __stdcall View3D_CameraFocusPointGet(view3d::Window window)
{
	try
	{
		Validate(window);

		DllLockGuard;
		return To<view3d::Vec4>(window->FocusPoint());
	}
	CatchAndReport(View3D_CameraFocusPointGet, window, {});
}
VIEW3D_API void __stdcall View3D_CameraFocusPointSet(view3d::Window window, view3d::Vec4 position)
{
	try
	{
		Validate(window);

		DllLockGuard;
		window->FocusPoint(To<v4>(position));
	}
	CatchAndReport(View3D_CameraFocusPointSet, window,);
}

// Get/Set bounds on the camera focus point position
VIEW3D_API view3d::BBox __stdcall View3D_CameraFocusBoundsGet(view3d::Window window)
{
	try
	{
		Validate(window);

		DllLockGuard;
		return To<view3d::BBox>(window->FocusBounds());
	}
	CatchAndReport(View3D_CameraFocusBoundsGet, window, To<view3d::BBox>(pr::BBox::Infinity()));
}
VIEW3D_API void __stdcall View3D_CameraFocusBoundsSet(view3d::Window window, view3d::BBox bounds)
{
	try
	{
		Validate(window);

		DllLockGuard;
		window->FocusBounds(To<pr::BBox>(bounds));
	}
	CatchAndReport(View3D_CameraFocusBoundsSet, window,);
}

// Get/Set the aspect ratio for the camera field of view
VIEW3D_API float __stdcall View3D_CameraAspectGet(view3d::Window window)
{
	try
	{
		Validate(window);

		DllLockGuard;
		return window->Aspect();
	}
	CatchAndReport(View3D_CameraAspectGet, window, {});
}
VIEW3D_API void __stdcall View3D_CameraAspectSet(view3d::Window window, float aspect)
{
	try
	{
		Validate(window);

		DllLockGuard;
		window->Aspect(aspect);
	}
	CatchAndReport(View3D_CameraAspectSet, window,);
}

// Get/Set both the X and Y fields of view (i.e. set the aspect ratio)
VIEW3D_API view3d::Vec2 __stdcall View3D_CameraFovGet(view3d::Window window)
{
	try
	{
		Validate(window);

		DllLockGuard;
		return To<view3d::Vec2>(window->Fov());
	}
	CatchAndReport(View3D_CameraFovSet, window, {});
}
VIEW3D_API void __stdcall View3D_CameraFovSet(view3d::Window window, view3d::Vec2 fov)
{
	try
	{
		Validate(window);

		DllLockGuard;
		window->Fov(To<v2>(fov));
	}
	CatchAndReport(View3D_CameraFovSet, window,);
}

// Adjust the FocusDist, FovX, and FovY so that the average FOV equals 'fov'
VIEW3D_API void __stdcall View3D_CameraBalanceFov(view3d::Window window, float fov)
{
	try
	{
		Validate(window);

		DllLockGuard;
		window->BalanceFov(fov);
	}
	CatchAndReport(View3D_CameraBalanceFov, window,);
}

// Get/Set (using fov and focus distance) the size of the perpendicular area visible to the camera at 'dist' (in world space). Use 'focus_dist != 0' to set a specific focus distance
VIEW3D_API view3d::Vec2 __stdcall View3D_CameraViewRectAtDistanceGet(view3d::Window window, float dist)
{
	try
	{
		Validate(window);

		DllLockGuard;
		return To<view3d::Vec2>(window->ViewRectAtDistance(dist));
	}
	CatchAndReport(View3D_ViewArea, window, {});
}
VIEW3D_API void __stdcall View3D_CameraViewRectAtDistanceSet(view3d::Window window, view3d::Vec2 rect, float focus_dist)
{
	try
	{
		Validate(window);

		DllLockGuard;
		window->ViewRectAtDistance(To<v2>(rect), focus_dist);
	}
	CatchAndReport(View3D_CameraViewRectSet, window,);
}

// Get/Set the near and far clip planes for the camera
VIEW3D_API view3d::Vec2 __stdcall View3D_CameraClipPlanesGet(view3d::Window window, EClipPlanes flags)
{
	try
	{
		Validate(window);

		DllLockGuard;
		return To<view3d::Vec2>(window->ClipPlanes(flags));
	}
	CatchAndReport(View3D_CameraClipPlanesGet, window, {});
}
VIEW3D_API void __stdcall View3D_CameraClipPlanesSet(view3d::Window window, float near_, float far_, EClipPlanes flags)
{
	try
	{
		Validate(window);

		DllLockGuard;
		window->ClipPlanes(near_, far_, flags);
	}
	CatchAndReport(View3D_CameraClipPlanesSet, window,);
}

// Get/Set the scene camera lock mask
VIEW3D_API view3d::ECameraLockMask __stdcall View3D_CameraLockMaskGet(view3d::Window window)
{
	try
	{
		Validate(window);

		DllLockGuard;
		return s_cast<view3d::ECameraLockMask>(window->LockMask());
	}
	CatchAndReport(View3D_CameraLockMaskGet, window, {});
}
VIEW3D_API void __stdcall View3D_CameraLockMaskSet(view3d::Window window, view3d::ECameraLockMask mask)
{
	try
	{
		Validate(window);

		DllLockGuard;
		window->LockMask(s_cast<camera::ELockMask>(mask));
	}
	CatchAndReport(View3D_CameraLockMaskSet, window,);
}

// Get/Set the camera align axis
VIEW3D_API view3d::Vec4 __stdcall View3D_CameraAlignAxisGet(view3d::Window window)
{
	try
	{
		Validate(window);

		DllLockGuard;
		return To<view3d::Vec4>(window->AlignAxis());
	}
	CatchAndReport(View3D_CameraAlignAxisGet, window, {});
}
VIEW3D_API void __stdcall View3D_CameraAlignAxisSet(view3d::Window window, view3d::Vec4 axis)
{
	try
	{
		Validate(window);

		DllLockGuard;
		window->AlignAxis(To<v4>(axis));
	}
	CatchAndReport(View3D_CameraAlignAxisSet, window,);
}

// Reset to the default zoom
VIEW3D_API void __stdcall View3D_CameraResetZoom(view3d::Window window)
{
	try
	{
		Validate(window);

		DllLockGuard;
		window->ResetZoom();
	}
	CatchAndReport(View3D_CameraResetZoom, window,);
}

// Get/Set the FOV zoom
VIEW3D_API float __stdcall View3D_CameraZoomGet(view3d::Window window)
{
	try
	{
		Validate(window);

		DllLockGuard;
		return window->Zoom();
	}
	CatchAndReport(View3D_CameraZoomGet, window, {});
}
VIEW3D_API void __stdcall View3D_CameraZoomSet(view3d::Window window, float zoom)
{
	try
	{
		Validate(window);

		DllLockGuard;
		window->Zoom(zoom);
	}
	CatchAndReport(View3D_CameraZoomSet, window,);
}

// Commit the current O2W position as the reference position
VIEW3D_API void __stdcall View3D_CameraCommit(view3d::Window window)
{
	try
	{
		Validate(window);

		DllLockGuard;
		window->m_scene.m_cam.Commit();
	}
	CatchAndReport(View3D_CameraCommit, window,);
}

// Navigation *****************************

// Direct movement of the camera
VIEW3D_API BOOL __stdcall View3D_Navigate(view3d::Window window, float dx, float dy, float dz)
{
	try
	{
		Validate(window);

		DllLockGuard;
		return window->m_scene.m_cam.Translate(dx, dy, dz);
	}
	CatchAndReport(View3D_Navigate, window, FALSE);
}

// General mouse navigation
// 'ss_pos' is the mouse pointer position in 'window's screen space (i.e. relative to viewport's ScreenW/H)
// 'nav_op' is the navigation type
// 'nav_start_or_end' should be TRUE on mouse down/up events, FALSE for mouse move events
// void OnMouseDown(UINT nFlags, CPoint point) { View3D_MouseNavigate(win, point, nav_op, TRUE); }
// void OnMouseMove(UINT nFlags, CPoint point) { View3D_MouseNavigate(win, point, nav_op, FALSE); } if 'nav_op' is None, this will have no effect
// void OnMouseUp  (UINT nFlags, CPoint point) { View3D_MouseNavigate(win, point, 0, TRUE); }
// BOOL OnMouseWheel(UINT nFlags, short zDelta, CPoint) { if (nFlags == 0) View3D_MouseNavigateZ(win, 0, 0, zDelta / 120.0f); return TRUE; }
VIEW3D_API BOOL __stdcall View3D_MouseNavigate(view3d::Window window, view3d::Vec2 ss_pos, view3d::ENavOp nav_op, BOOL nav_start_or_end)
{
	try
	{
		Validate(window);

		DllLockGuard;
		return window->MouseNavigate(To<v2>(ss_pos), static_cast<camera::ENavOp>(nav_op), nav_start_or_end != 0);
	}
	CatchAndReport(View3D_MouseNavigate, window, FALSE);
}
VIEW3D_API BOOL __stdcall View3D_MouseNavigateZ(view3d::Window window, view3d::Vec2 ss_pos, float delta, BOOL along_ray)
{
	try
	{
		Validate(window);

		DllLockGuard;
		return window->MouseNavigateZ(To<v2>(ss_pos), delta, along_ray != 0);
	}
	CatchAndReport(View3D_MouseNavigate, window, FALSE);
}

// Convert an MK_ macro to a default navigation operation
VIEW3D_API view3d::ENavOp __stdcall View3D_MouseBtnToNavOp(int mk)
{
	return static_cast<view3d::ENavOp>(camera::MouseBtnToNavOp(mk));
}
	
// Convert a length in pixels into a length in normalised screen space
VIEW3D_API view3d::Vec2 __stdcall View3D_PixelsToNSS(view3d::Window window, view3d::Vec2 pixels)
{
	try
	{
		Validate(window);

		DllLockGuard;
		return To<view3d::Vec2>(window->m_scene.m_viewport.PixelsToNSS(To<v2>(pixels)));
	}
	CatchAndReport(View3d_PixelsToNSS, window, {});
}

// Convert a point between 'window' screen space and normalised screen space
VIEW3D_API view3d::Vec2 __stdcall View3D_SSPointToNSSPoint(view3d::Window window, view3d::Vec2 screen)
{
	try
	{
		Validate(window);

		DllLockGuard;
		return To<view3d::Vec2>(window->m_scene.m_viewport.SSPointToNSSPoint(To<v2>(screen)));
	}
	CatchAndReport(View3D_SSPointToNSSPoint, window, view3d::Vec2{});
}
VIEW3D_API view3d::Vec2 __stdcall View3D_NSSPointToSSPoint(view3d::Window window, view3d::Vec2 nss_point)
{
	try
	{
		Validate(window);

		DllLockGuard;
		return To<view3d::Vec2>(window->m_scene.m_viewport.NSSPointToSSPoint(To<v2>(nss_point)));
	}
	CatchAndReport(View3D_NSSPointToSSPoint, window, view3d::Vec2{});
}

// Convert a point between world space and normalised screen space.
// The x,y components of 'screen' should be in normalised screen space, i.e. (-1,-1)->(1,1)
// The z component should be the world space distance from the camera
VIEW3D_API view3d::Vec4 __stdcall View3D_NSSPointToWSPoint(view3d::Window window, view3d::Vec4 screen)
{
	try
	{
		Validate(window);

		DllLockGuard;
		return To<view3d::Vec4>(window->m_scene.m_cam.NSSPointToWSPoint(To<v4>(screen)));
	}
	CatchAndReport(View3D_NSSPointToWSPoint, window, view3d::Vec4());
}
VIEW3D_API view3d::Vec4 __stdcall View3D_WSPointToNSSPoint(view3d::Window window, view3d::Vec4 world)
{
	try
	{
		Validate(window);

		DllLockGuard;
		return To<view3d::Vec4>(window->m_scene.m_cam.WSPointToNSSPoint(To<v4>(world)));
	}
	CatchAndReport(View3D_WSPointToNSSPoint, window, view3d::Vec4{});
}

// Return a point and direction in world space corresponding to a normalised screen space point.
// The x,y components of 'screen' should be in normalised screen space, i.e. (-1,-1)->(1,1)
// The z component should be the world space distance from the camera
VIEW3D_API void __stdcall View3D_NSSPointToWSRay(view3d::Window window, view3d::Vec4 screen, view3d::Vec4& ws_point, view3d::Vec4& ws_direction)
{
	try
	{
		Validate(window);

		DllLockGuard;
		auto [pt, dir] = window->m_scene.m_cam.NSSPointToWSRay(To<v4>(screen));
		ws_point = To<view3d::Vec4>(pt);
		ws_direction = To<view3d::Vec4>(dir);
	}
	CatchAndReport(View3D_NSSPointToWSRay, window,);
}

// Return a point and direction in world space corresponding to a 'window' screen space point.
// 'screen' should be in pixels
VIEW3D_API void __stdcall View3D_SSPointToWSRay(view3d::Window window, view3d::Vec2 screen, view3d::Vec4& ws_point, view3d::Vec4& ws_direction)
{
	try
	{
		Validate(window);

		DllLockGuard;
		auto nss_pt = window->m_scene.m_viewport.SSPointToNSSPoint(To<v2>(screen));
		auto z = static_cast<float>(window->m_scene.m_cam.FocusDist());
		auto [pt, dir] = window->m_scene.m_cam.NSSPointToWSRay(v4{ nss_pt, z, 1 });
		ws_point = To<view3d::Vec4>(pt);
		ws_direction = To<view3d::Vec4>(dir);
	}
	CatchAndReport(View3D_SSPointToWSRay, window,);
}

// Lights *********************************

// Get/Set the properties of the global light
VIEW3D_API view3d::Light __stdcall View3D_LightPropertiesGet(view3d::Window window)
{
	try
	{
		Validate(window);

		DllLockGuard;
		auto global_light = window->GlobalLight();
		return view3d::Light {
			.m_position       = To<view3d::Vec4>(global_light.m_position),
			.m_direction      = To<view3d::Vec4>(global_light.m_direction),
			.m_type           = s_cast<view3d::ELight>(global_light.m_type),
			.m_ambient        = global_light.m_ambient.argb,
			.m_diffuse        = global_light.m_diffuse.argb,
			.m_specular       = global_light.m_specular.argb,
			.m_specular_power = global_light.m_specular_power,
			.m_range          = global_light.m_range,
			.m_falloff        = global_light.m_falloff,
			.m_inner_angle    = global_light.m_inner_angle,
			.m_outer_angle    = global_light.m_outer_angle,
			.m_cast_shadow    = global_light.m_cast_shadow,
			.m_cam_relative   = global_light.m_cam_relative,
			.m_on             = global_light.m_on,
		};
	}
	CatchAndReport(View3D_LightPropertiesGet, window, {});
}
VIEW3D_API void __stdcall View3D_LightPropertiesSet(view3d::Window window, view3d::Light const& light)
{
	try
	{
		Validate(window);
		assert(light.m_position.w == 1);

		DllLockGuard;
		rdr12::Light global_light;
		global_light.m_position       = To<v4>(light.m_position);
		global_light.m_direction      = To<v4>(light.m_direction);
		global_light.m_type           = Enum<rdr12::ELight>::From(light.m_type);
		global_light.m_ambient        = light.m_ambient;
		global_light.m_diffuse        = light.m_diffuse;
		global_light.m_specular       = light.m_specular;
		global_light.m_specular_power = light.m_specular_power;
		global_light.m_range          = light.m_range;
		global_light.m_falloff        = light.m_falloff;
		global_light.m_inner_angle    = light.m_inner_angle;
		global_light.m_outer_angle    = light.m_outer_angle;
		global_light.m_cast_shadow    = light.m_cast_shadow;
		global_light.m_cam_relative   = light.m_cam_relative != 0;
		global_light.m_on             = light.m_on != 0;
		window->GlobalLight(global_light);
	}
	CatchAndReport(View3D_LightPropertiesSet, window,);
}

// Set the global light source for a window
VIEW3D_API void __stdcall View3D_LightSource(view3d::Window window, view3d::Vec4 position, view3d::Vec4 direction, BOOL camera_relative)
{
	try
	{
		Validate(window);
		assert(position.w == 1);

		DllLockGuard;
		auto global_light = window->GlobalLight();
		global_light.m_position = To<v4>(position);
		global_light.m_direction = To<v4>(direction);
		global_light.m_cam_relative = camera_relative != 0;
		window->GlobalLight(global_light);
	}
	CatchAndReport(View3D_LightSource, window,);
}

// Objects ********************************

// Create an object from provided buffers
VIEW3D_API view3d::Object __stdcall View3D_ObjectCreate(char const* name, view3d::Colour colour, int vcount, int icount, int ncount, view3d::Vertex const* verts, UINT16 const* indices, view3d::Nugget const* nuggets, GUID const& context_id)
{
	try
	{
		DllLockGuard;
		return Dll().ObjectCreate(name, colour, { verts, s_cast<size_t>(vcount) }, { indices, s_cast<size_t>(icount) }, { nuggets, s_cast<size_t>(ncount) }, context_id);
	}
	CatchAndReport(View3D_ObjectCreate, , nullptr);
}

// Create objects given in an ldraw string or file.
// If multiple objects are created, the handle returned is to the first object only.
// 'ldr_script' - an ldr string, or filepath to a file containing ldr script
// 'file' - TRUE if 'ldr_script' is a filepath, FALSE if 'ldr_script' is a string containing ldr script
// 'context_id' - the context id to create the LdrObjects with
// 'includes' - information used to resolve include directives in 'ldr_script'
VIEW3D_API view3d::Object __stdcall View3D_ObjectCreateLdrA(char const* ldr_script, BOOL file, GUID const* context_id, view3d::Includes const* includes)
{
	try
	{
		DllLockGuard;
		auto is_file = file != 0;
		auto enc = is_file ? EEncoding::auto_detect : EEncoding::utf8;
		return Dll().ObjectCreateLdr<char>(ldr_script, is_file, enc, context_id, includes);
	}
	CatchAndReport(View3D_ObjectCreateLdr, , nullptr);
}
VIEW3D_API view3d::Object __stdcall View3D_ObjectCreateLdrW(wchar_t const* ldr_script, BOOL file, GUID const* context_id, view3d::Includes const* includes)
{
	try
	{
		DllLockGuard;
		auto is_file = file != 0;
		auto enc = is_file ? EEncoding::auto_detect : EEncoding::already_decoded;
		return Dll().ObjectCreateLdr<wchar_t>(ldr_script, is_file, enc, context_id, includes);
	}
	CatchAndReport(View3D_ObjectCreateLdr, , nullptr);
}

// Load a p3d model file as a view3d object
VIEW3D_API view3d::Object __stdcall View3D_ObjectCreateP3DFile(char const* name, view3d::Colour colour, char const* p3d_filepath, GUID const* context_id)
{
	try
	{
		DllLockGuard;
		return Dll().ObjectCreateP3D(name, colour, p3d_filepath, context_id);
	}
	CatchAndReport(View3D_ObjectCreateP3D, , {});
}

// Load a p3d model in memory as a view3d object
VIEW3D_API view3d::Object __stdcall View3D_ObjectCreateP3DStream(char const* name, view3d::Colour colour, size_t size, void const* p3d_data, GUID const* context_id)
{
	try
	{
		DllLockGuard;
		return Dll().ObjectCreateP3D(name, colour, { byte_ptr(p3d_data), size } , context_id);
	}
	CatchAndReport(View3D_ObjectCreateP3D, , {});
}

// Create an ldr object using a callback to populate the model data.
VIEW3D_API view3d::Object __stdcall View3D_ObjectCreateWithCallback(char const* name, view3d::Colour colour, int vcount, int icount, int ncount, view3d::EditObjectCB edit_cb, GUID const& context_id)
{
	try
	{
		DllLockGuard;
		return Dll().ObjectCreateByCallback(name, To<Colour32>(colour), vcount, icount, ncount, edit_cb, context_id);
	}
	CatchAndReport(View3D_ObjectCreateWithCallback, , nullptr);
}
VIEW3D_API void __stdcall View3D_ObjectEdit(view3d::Object object, view3d::EditObjectCB edit_cb)
{
	try
	{
		Validate(object);

		DllLockGuard;
		Dll().ObjectEdit(object, edit_cb);
	}
	CatchAndReport(View3D_ObjectEdit, , );
}

// Replace the model and all child objects of 'obj' with the results of 'ldr_script'
VIEW3D_API void __stdcall View3D_ObjectUpdate(view3d::Object object, wchar_t const* ldr_script, view3d::EUpdateObject flags)
{
	try
	{
		Validate(object);

		DllLockGuard;
		Dll().UpdateObject(object, std::wstring_view{ ldr_script }, static_cast<rdr12::ldraw::EUpdateObject>(flags));
	}
	CatchAndReport(View3D_ObjectUpdate, ,);
}

// Delete an object, freeing its resources
VIEW3D_API void __stdcall View3D_ObjectDelete(view3d::Object object)
{
	try
	{
		// Delete is idempotent
		if (!object) return;
		
		DllLockGuard;
		Dll().DeleteObject(object);
	}
	CatchAndReport(View3D_ObjectDelete, ,);
}

// Create an instance of 'obj'
VIEW3D_API view3d::Object __stdcall View3D_ObjectCreateInstance(view3d::Object existing)
{
	try
	{
		Validate(existing);

		DllLockGuard;
		auto obj = CreateInstance(existing);
		if (obj) Dll().m_sources.Add(obj);
		return obj.get();
	}
	CatchAndReport(View3D_ObjectCreateInstance, , {});
}

// Return the context id that this object belongs to
VIEW3D_API GUID __stdcall View3D_ObjectContextIdGet(view3d::Object object)
{
	try
	{
		Validate(object);

		DllLockGuard;
		return object->m_context_id;
	}
	CatchAndReport(View3D_ObjectContextIdGet, , {});
}

// Return the root object of 'object' (possibly itself)
VIEW3D_API view3d::Object __stdcall View3D_ObjectGetRoot(view3d::Object object)
{
	try
	{
		Validate(object);

		DllLockGuard;
		auto p = object;
		for (; p->m_parent != nullptr; p = p->m_parent) {}
		return p;
	}
	CatchAndReport(View3D_ObjectGetRoot, , nullptr);
}

// Return the immediate parent of 'object'
VIEW3D_API view3d::Object __stdcall View3D_ObjectGetParent(view3d::Object object)
{
	try
	{
		Validate(object);

		DllLockGuard;
		return object->m_parent;
	}
	CatchAndReport(View3D_ObjectGetParent, , nullptr);
}

// Return a child object of 'object'
VIEW3D_API view3d::Object __stdcall View3D_ObjectGetChildByName(view3d::Object object, char const* name)
{
	try
	{
		Validate(object);

		DllLockGuard;
		return object->Child(name);
	}
	CatchAndReport(View3D_ObjectGetChildByName, , nullptr);
}
VIEW3D_API view3d::Object __stdcall View3D_ObjectGetChildByIndex(view3d::Object object, int index)
{
	try
	{
		Validate(object);

		DllLockGuard;
		return object->Child(index);
	}
	CatchAndReport(View3D_ObjectGetChildByIndex, , nullptr);
}

// Return the number of child objects of 'object'
VIEW3D_API int __stdcall View3D_ObjectChildCount(view3d::Object object)
{
	try
	{
		Validate(object);

		DllLockGuard;
		return int(object->m_child.size());
	}
	CatchAndReport(View3D_ObjectChildCount, object, 0);
}

// Enumerate the child objects of 'object'. (Not recursive)
VIEW3D_API void __stdcall View3D_ObjectEnumChildren(view3d::Object object, view3d::EnumObjectsCB enum_objects_cb)
{
	try
	{
		Validate(object);

		DllLockGuard;
		for (auto& child : object->m_child)
		{
			if (enum_objects_cb(child.get())) continue;
			break;
		}
	}
	CatchAndReport(View3D_ObjectEnumChildren, object, );
}

// Get/Set the name of 'object'
VIEW3D_API BSTR __stdcall View3D_ObjectNameGetBStr(view3d::Object object)
{
	try
	{
		Validate(object);

		DllLockGuard;
		auto name = Widen(object->m_name);
		return ::SysAllocStringLen(name.c_str(), UINT(name.size()));
	}
	CatchAndReport(View3D_ObjectNameGetBStr, , BSTR());
}
VIEW3D_API char const* __stdcall View3D_ObjectNameGet(view3d::Object object)
{
	try
	{
		Validate(object);

		DllLockGuard;
		return object->m_name.c_str();
	}
	CatchAndReport(View3D_ObjectNameGet, , nullptr);
}
VIEW3D_API void __stdcall View3D_ObjectNameSet(view3d::Object object, char const* name)
{
	try
	{
		Validate(object);

		DllLockGuard;
		object->m_name.assign(name);
	}
	CatchAndReport(View3D_ObjectNameGet, ,);
}

// Get the type of 'object'
VIEW3D_API BSTR __stdcall View3D_ObjectTypeGetBStr(view3d::Object object)
{
	try
	{
		Validate(object);

		DllLockGuard;
		auto name = pr::Enum<rdr12::ldraw::ELdrObject>::ToStringW(object->m_type);
		return ::SysAllocStringLen(name, UINT(wcslen(name)));
	}
	CatchAndReport(View3D_ObjectTypeGetBStr, , BSTR());
}
VIEW3D_API char const*  __stdcall View3D_ObjectTypeGet(view3d::Object object)
{
	try
	{
		Validate(object);

		DllLockGuard;
		return Enum<rdr12::ldraw::ELdrObject>::ToStringA(object->m_type);
	}
	CatchAndReport(View3D_ObjectTypeGet, , nullptr);
}

// Get/Set the current or base colour of an object (the first object to match 'name') (See LdrObject::Apply)
VIEW3D_API view3d::Colour __stdcall View3D_ObjectColourGet(view3d::Object object, BOOL base_colour, char const* name)
{
	try
	{
		Validate(object);

		DllLockGuard;
		return To<view3d::Colour>(object->Colour(base_colour != 0, name));
	}
	CatchAndReport(View3D_ObjectColourGet, ,view3d::Colour(0xFFFFFFFF));
}
VIEW3D_API void __stdcall View3D_ObjectColourSet(view3d::Object object, BOOL base_colour, view3d::Colour colour, char const* name, view3d::EColourOp op, float op_value)
{
	try
	{
		Validate(object);

		DllLockGuard;
		object->Colour(base_colour != 0, Colour32(colour), name, static_cast<rdr12::ldraw::EColourOp>(op), op_value);
	}
	CatchAndReport(View3D_ObjectColourSet, ,);
}

// Reset the object colour back to its default
VIEW3D_API void __stdcall View3D_ObjectResetColour(view3d::Object object, char const* name)
{
	try
	{
		Validate(object);

		DllLockGuard;
		object->ResetColour(name);
	}
	CatchAndReport(View3D_ObjectResetColour, ,);
}

// Get/Set the object to world transform for this object or the first child object that matches 'name'.
// Note, setting the o2w for a child object positions the object in world space rather than parent space
// (internally the appropriate O2P transform is calculated to put the object at the given O2W location)
VIEW3D_API view3d::Mat4x4 __stdcall View3D_ObjectO2WGet(view3d::Object object, char const* name)
{
	try
	{
		Validate(object);

		DllLockGuard;
		return To<view3d::Mat4x4>(object->O2W(name));
	}
	CatchAndReport(View3D_ObjectO2WGet, , To<view3d::Mat4x4>(m4x4::Identity()));
}
VIEW3D_API void __stdcall View3D_ObjectO2WSet(view3d::Object object, view3d::Mat4x4 const& o2w, char const* name)
{
	try
	{
		Validate(object);
		
		auto o2w_ = To<m4x4>(o2w);
		if (!IsAffine(o2w_))
			throw std::runtime_error("invalid object to world transform");

		DllLockGuard;
		object->O2W(o2w_, name);
	}
	CatchAndReport(View3D_ObjectO2WSet, ,);
}

// Get/Set the object to parent transform for an object.
// This is the object to world transform for objects without parents.
// Note: In "*Box b { 1 1 1 *o2w{*pos{1 2 3}} }" setting this transform overwrites the "*o2w{*pos{1 2 3}}".
VIEW3D_API view3d::Mat4x4 __stdcall View3D_ObjectO2PGet(view3d::Object object, char const* name)
{
	try
	{
		Validate(object);

		DllLockGuard;
		return To<view3d::Mat4x4>(object->O2P(name));
	}
	CatchAndReport(View3D_ObjectGetO2P, , To<view3d::Mat4x4>(m4x4::Identity()));
}
VIEW3D_API void __stdcall View3D_ObjectO2PSet(view3d::Object object, view3d::Mat4x4 const& o2p, char const* name)
{
	try
	{
		Validate(object);
		if (!FEql(o2p.w.w, 1.0f))
			throw std::runtime_error("invalid object to parent transform");

		DllLockGuard;
		object->O2P(To<m4x4>(o2p), name);
	}
	CatchAndReport(View3D_ObjectSetO2P, ,);
}

// Get/Set the animation time to apply to 'object'
VIEW3D_API float __stdcall View3D_ObjectAnimTimeGet(view3d::Object object, char const* name)
{
	try
	{
		Validate(object);

		DllLockGuard;
		return object->AnimTime(name);
	}
	CatchAndReport(View3D_ObjectAnimTimeGet, , 0.0f);
}
VIEW3D_API void __stdcall View3D_ObjectAnimTimeSet(view3d::Object object, float time_s, char const* name)
{
	try
	{
		Validate(object);

		DllLockGuard;
		object->AnimTime(time_s, name);
	}
	CatchAndReport(View3D_ObjectAnimTimeSet, ,);
}

// Return the model space bounding box for 'object'
VIEW3D_API view3d::BBox __stdcall View3D_ObjectBBoxMS(view3d::Object object, view3d::EBBoxFlags bbox_flags)
{
	try
	{
		Validate(object);

		DllLockGuard;
		return To<view3d::BBox>(object->BBoxMS(static_cast<rdr12::ldraw::EBBoxFlags>(bbox_flags)));
	}
	CatchAndReport(View3D_ObjectBBoxMS, , {});
}

// Get/Set the object visibility. See LdrObject::Apply for docs on the format of 'name'
VIEW3D_API BOOL __stdcall View3D_ObjectVisibilityGet(view3d::Object object, char const* name)
{
	try
	{
		Validate(object);

		DllLockGuard;
		return const_cast<rdr12::ldraw::LdrObject const*>(object)->Visible(name);
	}
	CatchAndReport(View3D_ObjectGetVisibility, ,FALSE);
}
VIEW3D_API void __stdcall View3D_ObjectVisibilitySet(view3d::Object object, BOOL visible, char const* name)
{
	try
	{
		Validate(object);

		DllLockGuard;
		object->Visible(visible != 0, name);
	}
	CatchAndReport(View3D_ObjectSetVisibility, ,);
}

// Get/Set wireframe mode for an object (the first object to match 'name'). (See LdrObject::Apply)
VIEW3D_API BOOL __stdcall View3D_ObjectWireframeGet(view3d::Object object, char const* name)
{
	try
	{
		Validate(object);

		DllLockGuard;
		return const_cast<rdr12::ldraw::LdrObject const*>(object)->Wireframe(name);
	}
	CatchAndReport(View3D_ObjectWireframeGet, , FALSE);
}
VIEW3D_API void __stdcall View3D_ObjectWireframeSet(view3d::Object object, BOOL wire_frame, char const* name)
{
	try
	{
		Validate(object);

		DllLockGuard;
		object->Wireframe(wire_frame != 0, name);
	}
	CatchAndReport(View3D_ObjectWireframeSet, , );
}

// Get/Set the object flags. See LdrObject::Apply for docs on the format of 'name'
VIEW3D_API view3d::ELdrFlags __stdcall View3D_ObjectFlagsGet(view3d::Object object, char const* name)
{
	try
	{
		Validate(object);

		DllLockGuard;
		return static_cast<view3d::ELdrFlags>(object->Flags(name));
	}
	CatchAndReport(View3D_ObjectFlagsGet, ,view3d::ELdrFlags::None);
}
VIEW3D_API void __stdcall View3D_ObjectFlagsSet(view3d::Object object, view3d::ELdrFlags flags, BOOL state, char const* name)
{
	try
	{
		Validate(object);

		DllLockGuard;
		object->Flags(static_cast<rdr12::ldraw::ELdrFlags>(flags), state != 0, name);
	}
	CatchAndReport(View3D_ObjectFlagsSet, ,);
}

// Get/Set the reflectivity of an object (the first object to match 'name') (See LdrObject::Apply)
VIEW3D_API float __stdcall View3D_ObjectReflectivityGet(view3d::Object object, char const* name)
{
	try
	{
		Validate(object);

		DllLockGuard;
		return object->Reflectivity(name);
	}
	CatchAndReport(View3D_ObjectReflectivityGet, ,0);
}
VIEW3D_API void __stdcall View3D_ObjectReflectivitySet(view3d::Object object, float reflectivity, char const* name)
{
	try
	{
		Validate(object);

		DllLockGuard;
		object->Reflectivity(reflectivity, name);
	}
	CatchAndReport(View3D_ObjectReflectivitySet, ,);
}

// Get/Set the sort group for the object or its children. (See LdrObject::Apply)
VIEW3D_API view3d::ESortGroup __stdcall View3D_ObjectSortGroupGet(view3d::Object object, char const* name)
{
	try
	{
		Validate(object);

		DllLockGuard;
		return static_cast<view3d::ESortGroup>(object->SortGroup(name));
	}
	CatchAndReport(View3D_ObjectSortGroupGet, ,view3d::ESortGroup::Default);
}
VIEW3D_API void __stdcall View3D_ObjectSortGroupSet(view3d::Object object, view3d::ESortGroup group, char const* name)
{
	try
	{
		Validate(object);

		DllLockGuard;
		object->SortGroup(static_cast<rdr12::ESortGroup>(group), name);
	}
	CatchAndReport(View3D_ObjectSortGroupSet, ,);
}

// Get/Set 'show normals' mode for an object (the first object to match 'name') (See LdrObject::Apply)
VIEW3D_API BOOL __stdcall View3D_ObjectNormalsGet(view3d::Object object, char const* name)
{
	try
	{
		Validate(object);

		DllLockGuard;
		return const_cast<rdr12::ldraw::LdrObject const*>(object)->Normals(name);
	}
	CatchAndReport(View3D_ObjectNormalsGet, , FALSE);
}
VIEW3D_API void __stdcall View3D_ObjectNormalsSet(view3d::Object object, BOOL show, char const* name)
{
	try
	{
		Validate(object);

		// Normals length is a scene-wide property set in View3D_WindowNormalsLength
		DllLockGuard;
		object->Normals(show, name);
	}
	CatchAndReport(View3D_ObjectNormalsSet, , );
}

// Set the texture/sampler for all nuggets of 'object' or its children. (See LdrObject::Apply)
VIEW3D_API void __stdcall View3D_ObjectSetTexture(view3d::Object object, view3d::Texture tex, char const* name)
{
	try
	{
		Validate(object);

		DllLockGuard;
		object->SetTexture(tex, name);
	}
	CatchAndReport(View3D_ObjectSetTexture, ,);
}
VIEW3D_API void __stdcall View3D_ObjectSetSampler(view3d::Object object, view3d::Sampler sam, char const* name)
{
	try
	{
		Validate(object);

		DllLockGuard;
		object->SetSampler(sam, name);
	}
	CatchAndReport(View3D_ObjectSetSampler, ,);
}

// Get/Set the nugget flags on an object or its children (See LdrObject::Apply)
VIEW3D_API view3d::ENuggetFlag __stdcall View3D_ObjectNuggetFlagsGet(view3d::Object object, char const* name, int index)
{
	try
	{
		Validate(object);

		DllLockGuard;
		return static_cast<view3d::ENuggetFlag>(object->NuggetFlags(name, index));
	}
	CatchAndReport(View3D_ObjectNuggetFlagsGet, ,view3d::ENuggetFlag::None);
}
VIEW3D_API void __stdcall View3D_ObjectNuggetFlagsSet(view3d::Object object, view3d::ENuggetFlag flags, BOOL state, char const* name, int index)
{
	try
	{
		Validate(object);

		DllLockGuard;
		object->NuggetFlags(static_cast<rdr12::ENuggetFlag>(flags), state != 0, name, index);
	}
	CatchAndReport(View3D_ObjectNuggetFlagsSet, ,);
}

// Get/Set the tint colour for a nugget within the model of an object or its children. (See LdrObject::Apply)
VIEW3D_API view3d::Colour __stdcall View3D_ObjectNuggetTintGet(view3d::Object object, char const* name, int index)
{
	try
	{
		Validate(object);

		DllLockGuard;
		return static_cast<view3d::Colour>(object->NuggetTint(name, index));
	}
	CatchAndReport(View3D_ObjectNuggetTintGet, , {});
}
VIEW3D_API void __stdcall View3D_ObjectNuggetTintSet(view3d::Object object, view3d::Colour colour, char const* name, int index)
{
	try
	{
		Validate(object);

		DllLockGuard;
		object->NuggetTint(static_cast<Colour32>(colour), name, index);
	}
	CatchAndReport(View3D_ObjectNuggetTintSet, ,);
}

// Materials ******************************

// Create a texture from data in memory.
// Set 'data' to 0 to leave the texture uninitialised, if not 0 then data must point to width x height pixel data
// of the size appropriate for the given format. 'e.g. uint32_t px_data[width * height] for D3DFMT_A8R8G8B8'
// Note: careful with stride, 'data' is expected to have the appropriate stride for rdr12::BytesPerPixel(format) * width
VIEW3D_API view3d::Texture __stdcall View3D_TextureCreate(int width, int height, void const* data, size_t data_size, view3d::TextureOptions const& options)
{
	try
	{
		Image src(width, height, data, options.m_format);
		if (src.m_data != nullptr && src.m_pitch.y != s_cast<int>(data_size))
			throw std::runtime_error("Incorrect data size provided");

		ResDesc rdesc = ResDesc::Tex2D(src, s_cast<uint16_t>(options.m_mips), s_cast<EUsage>(options.m_usage))
			.multisamp(To<rdr12::MultiSamp>(options.m_multisamp))
			.def_state(options.m_resource_state)
			.clear(options.m_clear_value);
		TextureDesc tdesc = TextureDesc(rdr12::AutoId, rdesc)
			.has_alpha(options.m_has_alpha != 0)
			.name(options.m_dbg_name);

		DllLockGuard;
		ResourceFactory factory(Dll().m_rdr);
		auto tex = factory.CreateTexture2D(tdesc);
		tex->m_t2s = To<m4x4>(options.m_t2s);
		tex->m_t2s =
			IsAffine(tex->m_t2s) ? tex->m_t2s :
			tex->m_t2s == m4x4::Zero() ? m4x4::Identity() :
			throw std::runtime_error("Invalid texture to surface transform");

		// Rely on the caller for correct reference counting
		return tex.release();
	}
	CatchAndReport(View3D_TextureCreate, , nullptr);
}

// Create one of the stock textures
VIEW3D_API view3d::Texture __stdcall View3D_TextureCreateStock(view3d::EStockTexture stock_texture)
{
	try
	{
		DllLockGuard;
		ResourceFactory factory(Dll().m_rdr);
		auto tex = factory.CreateTexture(static_cast<rdr12::EStockTexture>(stock_texture));
		return tex.release();
	}
	CatchAndReport(View3D_TextureCreateStock, , nullptr);
}

// Load a texture from file, embedded resource, or stock assets. Specify width == 0, height == 0 to use the dimensions of the file
VIEW3D_API view3d::Texture __stdcall View3D_TextureCreateFromUri(char const* resource, int width, int height, view3d::TextureOptions const& options)
{
	try
	{
		ResDesc rdesc = ResDesc::Tex2D(Image{width, height, nullptr, options.m_format})
			.multisamp(To<rdr12::MultiSamp>(options.m_multisamp))
			.def_state(options.m_resource_state)
			.clear(options.m_clear_value);
		TextureDesc tdesc = TextureDesc(AutoId, rdesc)
			.has_alpha(options.m_has_alpha != 0)
			.name(options.m_dbg_name);

		DllLockGuard;
		ResourceFactory factory(Dll().m_rdr);
		auto tex = factory.CreateTexture2D(resource, tdesc);
		tex->m_t2s = To<m4x4>(options.m_t2s);
		tex->m_t2s =
			IsAffine(tex->m_t2s) ? tex->m_t2s :
			tex->m_t2s == m4x4::Zero() ? m4x4::Identity() :
			throw std::runtime_error("Invalid texture to surface transform");

		// Rely on the caller for correct reference counting
		return tex.release();
	}
	CatchAndReport(View3D_TextureCreateFromUri, , nullptr);
}

// Load a cube map from file, embedded resource, or stock assets. Specify width == 0, height == 0 to use the dimensions of the file
VIEW3D_API view3d::CubeMap __stdcall View3D_CubeMapCreateFromUri(char const* resource, view3d::CubeMapOptions const& options)
{
	try
	{
		DllLockGuard;
		ResourceFactory factory(Dll().m_rdr);
		auto tdesc = TextureDesc(rdr12::AutoId, ResDesc::TexCube({}));
		auto tex = factory.CreateTextureCube(resource, tdesc);

		// Set the cube map to world transform
		if (m4x4 cube2w; (cube2w = To<m4x4>(options.m_cube2w)) != m4x4::Zero())
		{
			if (!IsAffine(cube2w)) throw std::runtime_error("Invalid cube map orientation transform");
			tex->m_cube2w = cube2w;
		}

		// Rely on the caller for correct reference counting
		return tex.release();
	}
	CatchAndReport(View3D_CubeMapCreateFromUri, , nullptr);
}

// Create a texture sampler
VIEW3D_API view3d::Sampler __stdcall View3D_SamplerCreate(view3d::SamplerOptions const& options)
{
	try
	{
		auto desc = SamDesc(options.m_addrU, options.m_addrV, options.m_addrW, options.m_filter);
		rdr12::SamplerDesc sdesc = rdr12::SamplerDesc(AutoId, desc)
			.name(options.m_dbg_name);

		DllLockGuard;
		ResourceFactory factory(Dll().m_rdr);
		auto sam = factory.CreateSampler(sdesc);

		// Rely on the caller for correct reference counting
		return sam.release();
	}
	CatchAndReport(View3D_TextureCreate, , nullptr);
}

// Create one of the stock samplers
VIEW3D_API view3d::Sampler __stdcall View3D_SamplerCreateStock(view3d::EStockSampler stock_sampler)
{
	try
	{
		DllLockGuard;
		ResourceFactory factory(Dll().m_rdr);
		auto sam = factory.CreateSampler(static_cast<rdr12::EStockSampler>(stock_sampler));
		return sam.release();
	}
	CatchAndReport(View3D_SamplerCreateStock, , nullptr);
}

// Create a shader
VIEW3D_API view3d::Shader __stdcall View3D_ShaderCreate(view3d::ShaderOptions const&)
{
	try
	{
		// todo - create a compiled shader
		return nullptr;
	}
	CatchAndReport(View3D_ShaderCreate, , nullptr);
}

// Create one of the stock shaders
VIEW3D_API view3d::Shader __stdcall View3D_ShaderCreateStock(view3d::EStockShader stock_shader, char const* config)
{
	try
	{
		DllLockGuard;
		ResourceFactory factory(Dll().m_rdr);
		auto shdr = factory.CreateShader(static_cast<rdr12::EStockShader>(stock_shader), config);

		// Rely on the caller for correct reference counting
		return shdr.release();
	}
	CatchAndReport(View3D_ShaderCreate, , nullptr);
}

// Read the properties of an existing texture
VIEW3D_API view3d::ImageInfo __stdcall View3D_TextureGetInfo(view3d::Texture tex)
{
	try
	{
		Validate(tex);

		auto desc = tex->TexDesc();
		return view3d::ImageInfo{
			.m_width = desc.Width,
			.m_height = desc.Height,
			.m_depth = desc.DepthOrArraySize,
			.m_mips = desc.MipLevels,
			.m_format = desc.Format,
			.m_image_file_format = 0,
		};
	}
	CatchAndReport(View3D_TextureGetInfo, , {});
}

// Read the properties of an image file
VIEW3D_API view3d::EResult __stdcall View3D_TextureGetInfoFromFile(char const* tex_filepath, ImageInfo& info)
{
	try
	{
		(void)tex_filepath,info;
		//D3DXIMAGE_INFO tex_info;
		//if (Failed(Dll().m_rdr.m_mat_mgr.TextureInfo(tex_filepath, tex_info)))
		//	return EView3DResult::Failed;

		//info.m_width             = tex_info.Width;
		//info.m_height            = tex_info.Height;
		//info.m_depth             = tex_info.Depth;
		//info.m_mips              = tex_info.MipLevels;
		//info.m_format            = tex_info.Format;
		//info.m_image_file_format = tex_info.ImageFileFormat;
		throw std::runtime_error("not implemented");
		//return EView3DResult::Success;
	}
	CatchAndReport(View3D_TextureGetInfoFromFile, , view3d::EResult::Failed);
}

// Release a reference to a texture
VIEW3D_API void __stdcall View3D_TextureRelease(view3d::Texture tex)
{
	try
	{
		// Release is idempotent
		if (!tex) return;
		tex->Release();
	}
	CatchAndReport(View3D_TextureRelease, ,);
}
VIEW3D_API void __stdcall View3D_CubeMapRelease(view3d::CubeMap tex)
{
	try
	{
		// Release is idempotent
		if (!tex) return;
		tex->Release();
	}
	CatchAndReport(View3D_CubeMapRelease, ,);
}
VIEW3D_API void __stdcall View3D_SamplerRelease(view3d::Sampler sam)
{
	try
	{
		// Release is idempotent
		if (!sam) return;
		sam->Release();
	}
	CatchAndReport(View3D_SamplerRelease, ,);
}
VIEW3D_API void __stdcall View3D_ShaderRelease(pr::view3d::Shader shdr)
{
	try
	{
		// Release is idempotent
		if (!shdr) return;
		shdr->Release();
	}
	CatchAndReport(View3D_ShaderRelease, ,);
}

// Resize this texture to 'size'
VIEW3D_API void __stdcall View3D_TextureResize(view3d::Texture tex, uint64_t width, uint32_t height, uint16_t depth_or_array_len)
{
	try
	{
		Validate(tex);

		DllLockGuard;
		tex->Resize(width, height, depth_or_array_len);
	}
	CatchAndReport(View3D_TextureResize, ,);
}

// Return the ref count of 'tex'
VIEW3D_API ULONG __stdcall View3D_TextureRefCount(view3d::Texture tex)
{
	try
	{
		Validate(tex);
		return tex->m_ref_count;
	}
	CatchAndReport(View3D_TextureRefCount, , 0);
}

// Get/Set the private data associated with 'guid' for 'tex'
VIEW3D_API void __stdcall View3d_TexturePrivateDataGet(view3d::Texture tex, GUID const& guid, UINT& size, void* data)
{
	try
	{
		Validate(tex);

		// 'size' should be the size of the data pointed to by 'data'
		Check(tex->m_res->GetPrivateData(guid, &size, data));
	}
	CatchAndReport(View3d_TexturePrivateDataGet, ,);
}
VIEW3D_API void __stdcall View3d_TexturePrivateDataSet(view3d::Texture tex, GUID const& guid, UINT size, void const* data)
{
	try
	{
		Validate(tex);
		Check(tex->m_res->SetPrivateData(guid, size, data));
	}
	CatchAndReport(View3d_TexturePrivateDataSet, ,);
}
VIEW3D_API void __stdcall View3d_TexturePrivateDataIFSet(view3d::Texture tex, GUID const& guid, IUnknown* pointer)
{
	try
	{
		Validate(tex);
		Check(tex->m_res->SetPrivateDataInterface(guid, pointer));
	}
	CatchAndReport(View3d_TexturePrivateDataIFSet, ,);
}

// Resolve a MSAA texture into a non-MSAA texture
VIEW3D_API void __stdcall View3D_TextureResolveAA(view3d::Texture dst, view3d::Texture src)
{
	try
	{
		Validate(dst);
		Validate(src);
		
		auto src_tdesc = src->TexDesc();
		auto dst_tdesc = dst->TexDesc();
		if (src_tdesc.Format != dst_tdesc.Format)
			throw std::runtime_error("Source and destination textures must has the same format");

		throw std::runtime_error("Not implemented");
#if 0 //todo
		Renderer::Lock lock(Dll().m_rdr);
		lock.ImmediateDC()->ResolveSubresource(dst->m_res.get(), 0U, src->m_res.get(), 0U, src_tdesc.Format);
#endif
	}
	CatchAndReport(View3D_TextureResolveAA, , );
}

// Gizmos *********************************

// Create the 3D manipulation gizmo
VIEW3D_API view3d::Gizmo __stdcall View3D_GizmoCreate(view3d::EGizmoMode mode, view3d::Mat4x4 const& o2w)
{
	try
	{
		DllLockGuard;
		return Dll().GizmoCreate(static_cast<rdr12::ldraw::EGizmoMode>(mode), To<m4x4>(o2w));
	}
	CatchAndReport(View3D_GizmoCreate, , nullptr);
}

// Delete a 3D manipulation gizmo
VIEW3D_API void __stdcall View3D_GizmoDelete(view3d::Gizmo gizmo)
{
	try
	{
		if (!gizmo) return;
		
		DllLockGuard;
		Dll().GizmoDelete(gizmo);
	}
	CatchAndReport(View3D_GizmoDelete, ,);
}

// Attach/Detach callbacks that are called when the gizmo moves
VIEW3D_API void __stdcall View3D_GizmoMovedCBSet(view3d::Gizmo gizmo, view3d::GizmoMovedCB moved_cb, BOOL add)
{
	try
	{
		Validate(gizmo);
		if (!moved_cb) throw std::runtime_error("Callback function is null");

		auto gizmo_moved = [](void* ctx, rdr12::ldraw::LdrGizmo* gizmo, rdr12::ldraw::EGizmoState state)
		{
			auto& cb = *static_cast<view3d::GizmoMovedCB*>(ctx);
			cb(gizmo, static_cast<view3d::EGizmoState>(state));
		};

		DllLockGuard;
		if (add)
			gizmo->Manipulated += { &moved_cb, gizmo_moved };
		else
			gizmo->Manipulated -= { &moved_cb, gizmo_moved };
	}
	CatchAndReport(View3D_GizmoMovedCBSet, ,);
}

// Attach/Detach an object to the gizmo that will be moved as the gizmo moves
VIEW3D_API void __stdcall View3D_GizmoAttach(view3d::Gizmo gizmo, view3d::Object object)
{
	try
	{
		Validate(gizmo);
		Validate(object);

		DllLockGuard;
		gizmo->Attach(object->m_o2p);
	}
	CatchAndReport(View3D_GizmoAttach, ,);
}
VIEW3D_API void __stdcall View3D_GizmoDetach(view3d::Gizmo gizmo, view3d::Object object)
{
	try
	{
		Validate(gizmo);
		Validate(object);

		DllLockGuard;
		gizmo->Detach(object->m_o2p);
	}
	CatchAndReport(View3D_GizmoDetach, ,);
}

// Get/Set the scale factor for the gizmo
VIEW3D_API float __stdcall View3D_GizmoScaleGet(view3d::Gizmo gizmo)
{
	try
	{
		Validate(gizmo);

		DllLockGuard;
		return gizmo->m_scale;
	}
	CatchAndReport(View3D_GizmoScaleGet, , 0.0f);
}
VIEW3D_API void __stdcall View3D_GizmoScaleSet(view3d::Gizmo gizmo, float scale)
{
	try
	{
		Validate(gizmo);

		DllLockGuard;
		gizmo->m_scale = scale;
	}
	CatchAndReport(View3D_GizmoScaleSet, , );
}

// Get/Set the current mode of the gizmo
VIEW3D_API view3d::EGizmoMode __stdcall View3D_GizmoModeGet(view3d::Gizmo gizmo)
{
	try
	{
		Validate(gizmo);
		return static_cast<view3d::EGizmoMode>(gizmo->Mode());
	}
	CatchAndReport(View3D_GizmoModeGet, ,static_cast<view3d::EGizmoMode>(-1));
}
VIEW3D_API void __stdcall View3D_GizmoModeSet(view3d::Gizmo gizmo, view3d::EGizmoMode mode)
{
	try
	{
		Validate(gizmo);
		gizmo->Mode(static_cast<rdr12::ldraw::EGizmoMode>(mode));
	}
	CatchAndReport(View3D_GizmoModeSet, ,);
}

// Get/Set the object to world transform for the gizmo
VIEW3D_API view3d::Mat4x4 __stdcall View3D_GizmoO2WGet(view3d::Gizmo gizmo)
{
	try
	{
		Validate(gizmo);
		return To<view3d::Mat4x4>(gizmo->O2W());
	}
	CatchAndReport(View3D_GizmoO2WGet, , view3d::Mat4x4{});
}
VIEW3D_API void __stdcall View3D_GizmoO2WSet(view3d::Gizmo gizmo, view3d::Mat4x4 const& o2w)
{
	try
	{
		Validate(gizmo);
		gizmo->O2W(To<m4x4>(o2w));
	}
	CatchAndReport(View3D_GizmoO2WSet, ,);
}

// Get the offset transform that represents the difference between the gizmo's transform at the start of manipulation and now.
VIEW3D_API view3d::Mat4x4 __stdcall View3D_GizmoOffsetGet(view3d::Gizmo gizmo)
{
	try
	{
		Validate(gizmo);
		return To<view3d::Mat4x4>(gizmo->Offset());
	}
	CatchAndReport(View3D_GizmoGetOffset, , view3d::Mat4x4{});
}

// Get/Set whether the gizmo is active to mouse interaction
VIEW3D_API BOOL __stdcall View3D_GizmoEnabledGet(view3d::Gizmo gizmo)
{
	try
	{
		Validate(gizmo);
		return gizmo->Enabled();
	}
	CatchAndReport(View3D_GizmoEnabled, ,FALSE);
}
VIEW3D_API void __stdcall View3D_GizmoEnabledSet(view3d::Gizmo gizmo, BOOL enabled)
{
	try
	{
		Validate(gizmo);
		gizmo->Enabled(enabled != 0);
	}
	CatchAndReport(View3D_GizmoEnabledSet, ,);
}

// Returns true while manipulation is in progress
VIEW3D_API BOOL __stdcall View3D_GizmoManipulating(view3d::Gizmo gizmo)
{
	try
	{
		Validate(gizmo);
		return gizmo->Manipulating();
	}
	CatchAndReport(View3D_GizmoManipulating, ,FALSE);
}

// Diagnostics ****************************

// Get/Set whether object bounding boxes are visible
VIEW3D_API BOOL __stdcall View3D_DiagBBoxesVisibleGet(view3d::Window window)
{
	try
	{
		Validate(window);

		DllLockGuard;
		return window->BBoxesVisible();
	}
	CatchAndReport(View3D_DiagBBoxesVisibleGet, window, FALSE);
}
VIEW3D_API void __stdcall View3D_DiagBBoxesVisibleSet(view3d::Window window, BOOL visible)
{
	try
	{
		Validate(window);

		DllLockGuard;
		window->BBoxesVisible(visible != 0);
	}
	CatchAndReport(View3D_DiagBBoxesVisibleSet, window, );
}

// Get/Set the length of the vertex normals
VIEW3D_API float __stdcall View3D_DiagNormalsLengthGet(view3d::Window window)
{
	try
	{
		Validate(window);

		DllLockGuard;
		return window->NormalsLength();
	}
	CatchAndReport(View3D_DiagNormalsLengthGet, window, FALSE);
}
VIEW3D_API void __stdcall View3D_DiagNormalsLengthSet(view3d::Window window, float length)
{
	try
	{
		Validate(window);

		DllLockGuard;
		window->NormalsLength(length);
	}
	CatchAndReport(View3D_DiagNormalsLengthSet, window, );
}

// Get/Set the colour of the vertex normals
VIEW3D_API view3d::Colour __stdcall View3D_DiagNormalsColourGet(view3d::Window window)
{
	try
	{
		Validate(window);

		DllLockGuard;
		return To<view3d::Colour>(window->NormalsColour());
	}
	CatchAndReport(View3D_DiagNormalsColourGet, window, {});
}
VIEW3D_API void __stdcall View3D_DiagNormalsColourSet(view3d::Window window, view3d::Colour colour)
{
	try
	{
		Validate(window);

		DllLockGuard;
		window->NormalsColour(colour);
	}
	CatchAndReport(View3D_DiagNormalsColourSet, window, );
}

// Get/Set the size of the 'Points' fill mode points
VIEW3D_API view3d::Vec2 __stdcall View3D_DiagFillModePointsSizeGet(view3d::Window window)
{
	try
	{
		Validate(window);

		DllLockGuard;
		return To<view3d::Vec2>(window->FillModePointsSize());
	}
	CatchAndReport(View3D_DiagFillModePointsSizeGet, window, {});
}
VIEW3D_API void __stdcall View3D_DiagFillModePointsSizeSet(view3d::Window window, view3d::Vec2 size)
{
	try
	{
		Validate(window);

		DllLockGuard;
		window->FillModePointsSize(To<v2>(size));
	}
	CatchAndReport(View3D_DiagFillModePointsSizeSet, window, );
}

// Miscellaneous **************************

// Create a render target texture on a D3D9 device. Intended for WPF D3DImage
VIEW3D_API view3d::Texture __stdcall View3D_CreateDx9RenderTarget(HWND hwnd, UINT width, UINT height, view3d::TextureOptions const& options, HANDLE* shared_handle)
{
	try
	{
		if (hwnd == nullptr)
			throw std::runtime_error("DirectX 9 requires a window handle");
		
		// Convert the DXGI format to a Dx9 one
		auto fmt = Dx9Context::ConvertFormat(options.m_format);
		if (fmt == D3DFORMAT::D3DFMT_UNKNOWN)
			throw std::runtime_error(FmtS("No compatible DirectX 9 texture format for DXGI format %d", options.m_format));
		
		// Initialise 'handle' from the optional 'shared_handle'
		// If '*shared_handle != nullptr', then CreateTexture will create a Dx9 texture that uses the shared resource.
		// If 'shared_handle == nullptr', then the caller doesn't care about the shared handle, but we still need it so
		// that the created texture will be shared and we can create a Dx11 texture from it.
		HANDLE handle = shared_handle ? *shared_handle : nullptr;

		// Create the shared Dx9 texture
		Dx9Context dx9(hwnd);
		auto tex = dx9.CreateTexture(width, height, 1, D3DUSAGE_RENDERTARGET, fmt, D3DPOOL_DEFAULT, &handle);
		
		// Access the main surface of the render target texture
		D3DPtr<IDirect3DSurface9> surf0;
		tex->GetSurfaceLevel(0, surf0.address_of());

		// Save the shared handle if the caller wants it.
		if (shared_handle != nullptr)
			*shared_handle = handle;

		// Create a texture description
		ResDesc rdesc = ResDesc::Tex2D(Image{int(width), int(height)}, s_cast<uint16_t>(options.m_mips), s_cast<EUsage>(options.m_usage))
			.multisamp(To<rdr12::MultiSamp>(options.m_multisamp))
			.clear(options.m_clear_value);
		TextureDesc tdesc = TextureDesc(rdr12::AutoId, rdesc)
			.has_alpha(options.m_has_alpha != 0)
			.name(options.m_dbg_name);

		DllLockGuard;

		ResourceFactory factory(Dll().m_rdr);
		
		// Create a Dx11 texture using the shared resource
		auto t = factory.OpenSharedTexture2D(handle, tdesc);

		// Save the surface 0 pointer in the private data of the texture. (Adds a reference)
		t->m_res->SetPrivateDataInterface(rdr12::Texture2D::Surface0Pointer, surf0.get());

		// Add a handler to clean up this reference when the texture is destroyed
		auto surf0_ptr = surf0.get(); // Don't capture the RefPtr
		t->OnDestruction += [=](TextureBase&, EmptyArgs const&)
		{
			surf0_ptr->Release();
		};

		return t.release();
	}
	CatchAndReport(View3D_CreateDx9RenderTarget, , nullptr);
}

// Create a Texture instance from a shared d3d resource (created on a different d3d device)
VIEW3D_API view3d::Texture __stdcall View3D_CreateTextureFromSharedResource(IUnknown* shared_resource, view3d::TextureOptions const& options)
{
	try
	{
		if (!shared_resource) throw std::runtime_error("resource pointer is null");

		// Create a texture description
		ResDesc rdesc = ResDesc::Tex2D({}, s_cast<uint16_t>(options.m_mips), s_cast<EUsage>(options.m_usage))
			.multisamp(To<rdr12::MultiSamp>(options.m_multisamp))
			.clear(options.m_clear_value);
		TextureDesc tdesc = TextureDesc(rdr12::AutoId, rdesc)
			.has_alpha(options.m_has_alpha != 0)
			.name(options.m_dbg_name);

		DllLockGuard;
		
		ResourceFactory factory(Dll().m_rdr);
		auto t = factory.OpenSharedTexture2D(shared_resource, tdesc);
		return t.release();
	}
	CatchAndReport(View3D_TextureFromExisting, , nullptr);
}

// Return the supported MSAA quality for the given multi-sampling count
VIEW3D_API int __stdcall View3D_MSAAQuality(int count, DXGI_FORMAT format)
{
	try
	{
		rdr12::MultiSamp ms(count);
		ms.ScaleQualityLevel(Dll().m_rdr.d3d(), format);
		return ms.Quality;
	}
	CatchAndReport(View3D_MSAAQuality, , 0);
}

// Get/Set the visibility of one or more stock objects (focus point, origin, selection box, etc)
VIEW3D_API BOOL __stdcall View3D_StockObjectVisibleGet(view3d::Window window, view3d::EStockObject stock_objects)
{
	try
	{
		Validate(window);

		DllLockGuard;
		return window->StockObjectVisible(stock_objects);
	}
	CatchAndReport(View3D_StockObjectVisibleGet, window, false);
}
VIEW3D_API void __stdcall View3D_StockObjectVisibleSet(view3d::Window window, view3d::EStockObject stock_objects, BOOL show)
{
	try
	{
		Validate(window);

		DllLockGuard;
		window->StockObjectVisible(stock_objects, show != 0);
	}
	CatchAndReport(View3D_StockObjectVisibleSet, window,);
}

// Get/Set the size of the focus point
VIEW3D_API float __stdcall View3D_FocusPointSizeGet(view3d::Window window)
{
	try
	{
		Validate(window);

		DllLockGuard;
		return window->FocusPointSize();
	}
	CatchAndReport(View3D_FocusPointSizeGet, window, 0.0f);
}
VIEW3D_API void __stdcall View3D_FocusPointSizeSet(view3d::Window window, float size)
{
	try
	{
		Validate(window);

		DllLockGuard;
		window->FocusPointSize(size);
	}
	CatchAndReport(View3D_FocusPointSizeSet, window,);
}

// Get/Set the size of the origin point
VIEW3D_API float __stdcall View3D_OriginPointSizeGet(view3d::Window window)
{
	try
	{
		Validate(window);

		DllLockGuard;
		return window->OriginPointSize();
	}
	CatchAndReport(View3D_OriginSizeGet, window, {});
}
VIEW3D_API void __stdcall View3D_OriginPointSizeSet(view3d::Window window, float size)
{
	try
	{
		Validate(window);

		DllLockGuard;
		window->OriginPointSize(size);
	}
	CatchAndReport(View3D_OriginSizeSet, window,);
}

// Get/Set the position and size of the selection box
VIEW3D_API void __stdcall View3D_SelectionBoxGet(view3d::Window window, view3d::BBox& bbox, view3d::Mat4x4& o2w)
{
	try
	{
		Validate(window);

		DllLockGuard;
		auto [b, o] = window->SelectionBox();
		bbox = To<view3d::BBox>(b);
		o2w = To<view3d::Mat4x4>(m4x4(o, v4::Origin()));
	}
	CatchAndReport(View3D_SelectionBoxGet, window, );
}
VIEW3D_API void __stdcall View3D_SelectionBoxSet(view3d::Window window, view3d::BBox const& bbox, view3d::Mat4x4 const& o2w)
{
	try
	{
		Validate(window);

		DllLockGuard;
		window->SelectionBox(To<pr::BBox>(bbox), To<m4x4>(o2w).rot);
	}
	CatchAndReport(View3D_SelectionBoxSet, window, );
}

// Set the selection box to encompass all selected objects
VIEW3D_API void __stdcall View3D_SelectionBoxFitToSelected(view3d::Window window)
{
	try
	{
		Validate(window);

		DllLockGuard;
		window->SelectionBoxFitToSelected();
	}
	CatchAndReport(View3D_SelectionBoxFitToSelected, window, );
}

// Create/Delete the demo scene in the given window
VIEW3D_API GUID __stdcall View3D_DemoSceneCreateText(view3d::Window window)
{
	try
	{
		Validate(window);

		// Get the string of all ldr objects
		auto scene = rdr12::ldraw::CreateDemoSceneText();

		DllLockGuard;

		// Add the demo objects to the sources
		Dll().LoadScriptString(std::string_view(scene), EEncoding::utf8, &Context::GuidDemoSceneObjects, PathResolver{}, [=](Guid const& context_id, bool before)
		{
			if (before)
				window->Remove({ (void*)&context_id, rdr12::ldraw::MatchContextId });
			else
				window->Add(Dll().m_sources.Sources(), GuidPredCB{ &context_id, rdr12::ldraw::MatchContextId });
		});
		return Context::GuidDemoSceneObjects;
	}
	CatchAndReport(View3D_DemoSceneCreateText, window, Context::GuidDemoSceneObjects);
}
VIEW3D_API GUID __stdcall View3D_DemoSceneCreateBinary(view3d::Window window)
{
	try
	{
		Validate(window);

		// Get the string of all ldr objects
		auto scene = rdr12::ldraw::CreateDemoSceneBinary();
		#if PR_DBG && 1
		{
			std::ofstream file("E:/Dump/ldraw/demo_scene_binary.bdr", std::ios::binary);
			file.write(char_ptr(scene.data()), scene.size());
		}
		#endif

		DllLockGuard;

		Dll().LoadScriptBinary(scene, &Context::GuidDemoSceneObjects, [=](Guid const& id, bool before)
		{
			if (before)
				window->Remove({ &id, rdr12::ldraw::MatchContextId });
			else
				window->Add(Dll().m_sources.Sources(), { &id, rdr12::ldraw::MatchContextId });
		});
		return Context::GuidDemoSceneObjects;
	}
	CatchAndReport(View3D_DemoSceneCreateBinary, window, Context::GuidDemoSceneObjects);
}
VIEW3D_API void __stdcall View3D_DemoSceneDelete()
{
	try
	{
		DllLockGuard;
		Dll().DeleteAllObjectsById({ &Context::GuidDemoSceneObjects, rdr12::ldraw::MatchContextId });
	}
	CatchAndReport(View3D_DemoSceneDelete,,);
}

// Show a window containing the demo script
VIEW3D_API void __stdcall View3D_DemoScriptShow(view3d::Window window)
{
	try
	{
		Validate(window);

		DllLockGuard;
		
		auto example = rdr12::ldraw::CreateDemoSceneText();
		window->EditorUI().Show();
		window->EditorUI().Text(example.c_str());
	}
	CatchAndReport(View3D_DemoScriptShow, window,);
}

// Return the example Ldr script as a BSTR
VIEW3D_API BSTR __stdcall View3D_ExampleScriptBStr()
{
	try
	{
		DllLockGuard;
		auto example = Widen(rdr12::ldraw::CreateDemoSceneText());
		return ::SysAllocStringLen(example.c_str(), UINT(example.size()));
	}
	CatchAndReport(View3D_ExampleScriptBStr,,BSTR());
}

// Return the auto complete templates as a BSTR
VIEW3D_API BSTR __stdcall View3D_AutoCompleteTemplatesBStr()
{
	try
	{
		auto templates = Widen(rdr12::ldraw::AutoCompleteTemplates());
		return ::SysAllocStringLen(templates.c_str(), UINT(templates.size()));
	}
	CatchAndReport(View3D_AutoCompleteTemplatesBStr,,BSTR());
}

// Return the hierarchy "address" for a position in an ldr script file.
// The format of the returned address is: "keyword.keyword.keyword...". e.g. Group.Box.O2W.Pos
VIEW3D_API BSTR __stdcall View3D_ObjectAddressAt(wchar_t const* ldr_script, int64_t position)
{
	try
	{
		// 'script' should start from a root level position.
		// 'position' should be relative to 'script'
		//
		// The format of the returned address is: "keyword.keyword.keyword..."
		// e.g. example
		//   *Group { *Width {1} *Smooth *Box
		//   {
		//       *other {}
		//       /* *something { */
		//       // *something {
		//       "my { string"
		//       *o2w { *pos { <-- Address should be: Group.Box.o2w.pos

		std::wstringstream src({ ldr_script, ldr_script + position });
		rdr12::ldraw::TextReader reader(src, {});

		std::wstring path;
		try
		{
			for (rdr12::ldraw::EKeyword kw; !reader.IsSourceEnd(); )
			{
				// Find the next keyword in the current scope
				if (reader.NextKeyword(kw))
				{
					// Add to the path while within this section
					path.append(path.empty() ? L"" : L".").append(rdr12::ldraw::EKeyword_::ToStringW(kw));
					reader.PushSection();
				}
				if (reader.IsSectionEnd())
				{
					// If we've reached the end of the scope, pop that last keyword
					// from the path since 'position' is not within this scope.
					for (; !path.empty() && path.back() != '.'; path.pop_back()) {}
					if (!path.empty()) path.pop_back();
					reader.PopSection();
				}
			}
		}
		catch (std::exception const&)
		{
			// If the script contains errors, we can't be sure that 'path' is correct.
			// Return an empty path, rather than hoping that the path is right.
			path.resize(0);
		}
		return ::SysAllocStringLen(path.c_str(), UINT(path.size()));
	}
	CatchAndReport(View3D_ObjectAddressAt, , BSTR());
}

// Parse a transform description using the Ldr script syntax
VIEW3D_API view3d::Mat4x4 __stdcall View3D_ParseLdrTransform(char const* ldr_script)
{
	try
	{
		mem_istream<char> src(ldr_script);
		rdr12::ldraw::TextReader reader(src, {});
		
		auto o2w = m4x4::Identity();
		reader.Transform(o2w);
		return To<view3d::Mat4x4>(o2w);
	}
	CatchAndReport(View3D_ParseLdrTransform, , To<view3d::Mat4x4>(m4x4::Identity()));
}

// Handle standard keyboard shortcuts. 'key_code' should be a standard VK_ key code with modifiers included in the hi word. See 'EKeyCodes'
VIEW3D_API BOOL __stdcall View3D_TranslateKey(view3d::Window window, int key_code, view3d::Vec2 ss_point)
{
	try
	{
		Validate(window);

		DllLockGuard;
		return window->TranslateKey(static_cast<EKeyCodes>(key_code), To<v2>(ss_point)) ? TRUE : FALSE;
	}
	CatchAndReport(View3D_TranslateKey, window, FALSE);
}

// Return the reference count of a COM interface
VIEW3D_API ULONG __stdcall View3D_RefCount(IUnknown* pointer)
{
	try
	{
		if (pointer == nullptr) throw std::runtime_error("pointer is null");
		return rdr12::RefCount(pointer);
	}
	CatchAndReport(View3D_RefCount, , 0);
}

// Tools **********************************

// Show/Hide the object manager tool
VIEW3D_API BOOL __stdcall View3D_ObjectManagerVisibleGet(view3d::Window window)
{
	try
	{
		Validate(window);

		DllLockGuard;
		return window->ObjectManagerVisible();
	}
	CatchAndReport(View3D_ObjectManagerVisibleGet, window, false);
}
VIEW3D_API void __stdcall View3D_ObjectManagerVisibleSet(view3d::Window window, BOOL show)
{
	try
	{
		Validate(window);

		DllLockGuard;
		window->ObjectManagerVisible(show != 0);
	}
	CatchAndReport(View3D_ObjectManagerVisibleSet, window,);
}

// Show/Hide the script editor tool
VIEW3D_API BOOL __stdcall View3D_ScriptEditorVisibleGet(view3d::Window window)
{
	try
	{
		Validate(window);

		DllLockGuard;
		return window->ScriptEditorVisible();
	}
	CatchAndReport(View3D_ScriptEditorVisibleGet, window, false);
}
VIEW3D_API void __stdcall View3D_ScriptEditorVisibleSet(view3d::Window window, BOOL show)
{
	try
	{
		Validate(window);

		DllLockGuard;
		window->ScriptEditorVisible(show != 0);
	}
	CatchAndReport(View3D_ScriptEditorVisibleSet, window,);
}

// Show/Hide the measurement tool
VIEW3D_API BOOL __stdcall View3D_MeasureToolVisibleGet(view3d::Window window)
{
	try
	{
		Validate(window);

		DllLockGuard;
		return window->MeasureToolVisible();
	}
	CatchAndReport(View3D_MeasureToolVisibleGet, window, false);
}
VIEW3D_API void __stdcall View3D_MeasureToolVisibleSet(view3d::Window window, BOOL show)
{
	try
	{
		Validate(window);

		DllLockGuard;
		window->MeasureToolVisible(show != 0);
	}
	CatchAndReport(View3D_ShowMeasureTool, window,);
}

// Show/Hide the angle measurement tool
VIEW3D_API BOOL __stdcall View3D_AngleToolVisibleGet(view3d::Window window)
{
	try
	{
		Validate(window);

		DllLockGuard;
		return window->AngleToolVisible();
	}
	CatchAndReport(View3D_AngleToolVisibleGet, window, false);
}
VIEW3D_API void __stdcall View3D_AngleToolVisibleSet(view3d::Window window, BOOL show)
{
	try
	{
		Validate(window);

		DllLockGuard;
		window->AngleToolVisible(show != 0);
	}
	CatchAndReport(View3D_AngleToolVisibleSet, window,);
}

// Show/Hide the lighting controls UI
VIEW3D_API void __stdcall View3D_LightingControlsUI(view3d::Window window, BOOL show)
{
	try
	{
		Validate(window);

		DllLockGuard;
		window->LightingUI().Visible(show);
	}
	CatchAndReport(View3D_LightShowDialog, window,);
}
