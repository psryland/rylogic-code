//*********************************************
// View3DUI
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// Dependency-minimal, versioned C ABI for the native View3DUI satellite.
#pragma once

#ifdef VIEW3D_UI_EXPORTS
#define VIEW3D_UI_API __declspec(dllexport)
#else
#define VIEW3D_UI_API __declspec(dllimport)
#endif

#include "pr/view3d-ui/forward.h"
#include "pr/view3d-ui/types.h"

extern "C"
{
	// --- ABI/schema diagnosis --------------------------------------------------------------
	VIEW3D_UI_API std::uint32_t __stdcall View3DUI_ApiVersion();
	VIEW3D_UI_API pr::view3d::ui::EStatus __stdcall View3DUI_StructSize(pr::view3d::ui::EStructId struct_id, std::uint32_t* size);
	VIEW3D_UI_API pr::view3d::ui::EStatus __stdcall View3DUI_LastError(char* buffer, std::uint32_t capacity, std::uint32_t* required);

	// --- Process-level module lifetime -----------------------------------------------------
	VIEW3D_UI_API pr::view3d::ui::RuntimeHandle __stdcall View3DUI_Initialise(pr::view3d::ui::ReportErrorCB global_error_cb);
	VIEW3D_UI_API pr::view3d::ui::EStatus __stdcall View3DUI_Shutdown(pr::view3d::ui::RuntimeHandle runtime);
	VIEW3D_UI_API void __stdcall View3DUI_ContextAbandon(pr::view3d::ui::RuntimeHandle runtime);

	// --- Per-window owner-thread UI context -------------------------------------------------
	// 'external_d3d12_device' is a borrowed ID3D12Device*, passed as void* so this dependency-
	// minimal header never includes d3d12.h; the context AddRef's it as IUnknown and releases it
	// only after the context's own GPU resources (none exist before the M3 renderer) are retired.
	// 'view3d_window' is an optional borrowed pr::rdr12::V3dWindow* (also erased to void* for the
	// same reason); when non-null the context attaches a renderer-neutral Prepare/FinalOverlay
	// provider to it via the private view3d-12 UI host bridge, detached automatically on destroy.
	VIEW3D_UI_API pr::view3d::ui::EStatus __stdcall View3DUI_ContextCreate(pr::view3d::ui::RuntimeHandle runtime, pr::view3d::ui::Config const* config, void* external_d3d12_device, void* view3d_window, pr::view3d::ui::ContextHandle* context);
	VIEW3D_UI_API pr::view3d::ui::EStatus __stdcall View3DUI_ContextDestroy(pr::view3d::ui::ContextHandle context);
	VIEW3D_UI_API void __stdcall View3DUI_UiContextAbandon(pr::view3d::ui::ContextHandle context);

	// --- Retained model ----------------------------------------------------------------------
	VIEW3D_UI_API pr::view3d::ui::EStatus __stdcall View3DUI_TransactionApply(pr::view3d::ui::ContextHandle context, pr::view3d::ui::Transaction const* transaction);

	// --- Input ---------------------------------------------------------------------------------
	// Translate one raw HWND message into the same normalized state machine used by InputInject.
	// '*consumed' and '*invalidate' are 0/1 (bool is not permitted in the public ABI).
	VIEW3D_UI_API pr::view3d::ui::EStatus __stdcall View3DUI_ProcessWindowMessage(pr::view3d::ui::ContextHandle context, HWND hwnd, UINT msg, WPARAM wparam, LPARAM lparam, std::int32_t* consumed, LRESULT* result, std::int32_t* invalidate);
	VIEW3D_UI_API pr::view3d::ui::EStatus __stdcall View3DUI_InputInject(pr::view3d::ui::ContextHandle context, pr::view3d::ui::NormalizedInput const* input);

	// Inject one normalized input record that carries variable-length text: EInputKind::TextInput,
	// CompositionUpdate or CompositionCommit. 'payload' is borrowed for the duration of the call
	// and must be null for CompositionStart/CompositionCancel and for every non-text input kind.
	// This is the deterministic seam that lets a test or a managed caller drive a complete IME
	// composition lifecycle without an installed IME, using the identical state machine the raw
	// WM_IME_* path uses.
	VIEW3D_UI_API pr::view3d::ui::EStatus __stdcall View3DUI_InputInjectText(pr::view3d::ui::ContextHandle context, pr::view3d::ui::NormalizedInput const* input, pr::view3d::ui::InputTextPayload const* payload);

	// The caret rectangle of an editable control, in DIPs within the render target, as measured
	// from the same shaped layout the renderer draws. '*valid' is 0 when 'control_id' is not a
	// focused, editable, laid-out control, in which case '*caret_dip' is zeroed. Intended for
	// candidate-window placement and for a later UI Automation Text provider.
	VIEW3D_UI_API pr::view3d::ui::EStatus __stdcall View3DUI_CaretGeometry(pr::view3d::ui::ContextHandle context, pr::view3d::ui::ControlId control_id, pr::view3d::ui::Rect* caret_dip, std::int32_t* valid);

	// --- Update and render-neutral snapshot -----------------------------------------------------
	VIEW3D_UI_API pr::view3d::ui::EStatus __stdcall View3DUI_Update(pr::view3d::ui::ContextHandle context, pr::view3d::ui::ViewportState const* viewport);

	// --- Events -----------------------------------------------------------------------------
	VIEW3D_UI_API pr::view3d::ui::EStatus __stdcall View3DUI_EventCount(pr::view3d::ui::ContextHandle context, std::uint32_t* count);
	VIEW3D_UI_API pr::view3d::ui::EStatus __stdcall View3DUI_EventsCopy(pr::view3d::ui::ContextHandle context, pr::view3d::ui::Event* events, std::uint32_t capacity, std::uint32_t* required, std::byte* payload_blob, std::uint32_t payload_capacity, std::uint32_t* payload_required);

	// --- Semantics ---------------------------------------------------------------------------
	VIEW3D_UI_API pr::view3d::ui::EStatus __stdcall View3DUI_SemanticsCopy(pr::view3d::ui::ContextHandle context, pr::view3d::ui::SemanticNode* nodes, std::uint32_t capacity, std::uint32_t* required, char* text_blob, std::uint32_t text_capacity, std::uint32_t* text_required);

	// --- Diagnostics ---------------------------------------------------------------------------
	VIEW3D_UI_API pr::view3d::ui::EStatus __stdcall View3DUI_DiagnosticsGet(pr::view3d::ui::ContextHandle context, pr::view3d::ui::Diagnostics* diagnostics);
}
