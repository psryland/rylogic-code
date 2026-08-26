//*********************************************
// View3DUI
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// Eager dynamic native C++ facade for view3d-ui.dll. All exports are resolved by GetProcAddress
// once, at first use, so a missing/renamed export or an incompatible schema fails fast instead of
// on first (possibly much later) call. Callers never link an import library; this header is the
// only build-time dependency on the DLL (implementation-plan.md section 2).
#pragma once
#include "pr/view3d-ui/forward.h"
#include "pr/view3d-ui/types.h"
#include "pr/view3d-ui/transaction_builder.h"

namespace pr::view3d::ui
{
	// Client-side exception mirroring the native EStatus/last-error contract. The native
	// pr::view3d::ui::EngineException (engine.h) never crosses the DLL boundary; this is the type
	// client code actually catches.
	class Exception : public std::runtime_error
	{
		EStatus m_status;

	public:
		Exception(EStatus status, std::string_view message)
			: std::runtime_error(std::string(message))
			, m_status(status)
		{}
		EStatus Status() const
		{
			return m_status;
		}
	};

	// Complete table of view3d-ui.dll exports as free-function pointer types, independent of the
	// native "__declspec(dllimport)" declarations in view3d-ui-dll.h so that resolving them never
	// requires linking against an import library.
	#define VIEW3D_UI_API_TABLE(x)\
	x(ApiVersion             , std::uint32_t (__stdcall*)())\
	x(StructSize             , EStatus (__stdcall*)(EStructId struct_id, std::uint32_t* size))\
	x(LastError              , EStatus (__stdcall*)(char* buffer, std::uint32_t capacity, std::uint32_t* required))\
	x(Initialise             , RuntimeHandle (__stdcall*)(ReportErrorCB global_error_cb))\
	x(Shutdown               , EStatus (__stdcall*)(RuntimeHandle runtime))\
	x(ContextAbandon         , void (__stdcall*)(RuntimeHandle runtime))\
	x(ContextCreate          , EStatus (__stdcall*)(RuntimeHandle runtime, Config const* config, void* external_d3d12_device, void* view3d_window, ContextHandle* context))\
	x(ContextDestroy         , EStatus (__stdcall*)(ContextHandle context))\
	x(UiContextAbandon       , void (__stdcall*)(ContextHandle context))\
	x(TransactionApply       , EStatus (__stdcall*)(ContextHandle context, Transaction const* transaction))\
	x(ProcessWindowMessage   , EStatus (__stdcall*)(ContextHandle context, HWND hwnd, UINT msg, WPARAM wparam, LPARAM lparam, std::int32_t* consumed, LRESULT* result, std::int32_t* invalidate))\
	x(InputInject            , EStatus (__stdcall*)(ContextHandle context, NormalizedInput const* input))\
	x(InputInjectText        , EStatus (__stdcall*)(ContextHandle context, NormalizedInput const* input, InputTextPayload const* payload))\
	x(CaretGeometry          , EStatus (__stdcall*)(ContextHandle context, ControlId control_id, Rect* caret_dip, std::int32_t* valid))\
	x(Update                 , EStatus (__stdcall*)(ContextHandle context, ViewportState const* viewport))\
	x(EventCount             , EStatus (__stdcall*)(ContextHandle context, std::uint32_t* count))\
	x(EventsCopy             , EStatus (__stdcall*)(ContextHandle context, Event* events, std::uint32_t capacity, std::uint32_t* required, std::byte* payload_blob, std::uint32_t payload_capacity, std::uint32_t* payload_required))\
	x(SemanticsCopy          , EStatus (__stdcall*)(ContextHandle context, SemanticNode* nodes, std::uint32_t capacity, std::uint32_t* required, char* text_blob, std::uint32_t text_capacity, std::uint32_t* text_required))\
	x(DiagnosticsGet         , EStatus (__stdcall*)(ContextHandle context, Diagnostics* diagnostics))

	// Dynamically-loaded view3d-ui.dll. Every export is resolved once at construction (i.e. at
	// first use of Dll::Get()); a missing export or an ApiVersion mismatch throws immediately
	// rather than deferring the failure to whichever call happens to use that export first.
	class Dll
	{
		HMODULE m_module;

		// Resolves the complete export table up-front; any single missing export means the loaded
		// DLL cannot be this header's contract at all, so construction fails immediately rather than
		// deferring the failure to whichever call happens to use a missing export first. Private:
		// the only instance is the process-wide singleton returned by Get().
		Dll()
			: m_module(win32::LoadDll<struct View3DUIDllTag>(L"view3d-ui.dll"))
		{
			#define VIEW3D_UI_GET_PROC_ADDRESS(name, function_type)\
			name = reinterpret_cast<name##Fn>(::GetProcAddress(m_module, "View3DUI_" #name));\
			if (name == nullptr)\
				throw std::runtime_error("view3d-ui.dll is missing the '" #name "' export; the loaded DLL does not match this header's ABI");
			VIEW3D_UI_API_TABLE(VIEW3D_UI_GET_PROC_ADDRESS)
			#undef VIEW3D_UI_GET_PROC_ADDRESS

			// Every export resolved; now confirm the loaded DLL's schema generation matches the
			// version this header was compiled against (fail-fast ABI check, section 5.1).
			auto loaded_version = ApiVersion();
			if (loaded_version != VIEW3D_UI_API_VERSION)
				throw Exception(EStatus::AbiMismatch, std::format("view3d-ui.dll API version {:#010x} does not match the version {:#010x} this header expects", loaded_version, static_cast<std::uint32_t>(VIEW3D_UI_API_VERSION)));
		}

	public:
		// Resolved function pointers for every view3d-ui.dll export. Public so free functions and
		// the Runtime/UiContext wrappers below can call 'Dll::Get().ExportName(...)' directly; this
		// class still exposes no way to construct or copy an instance other than through Get().
		#define VIEW3D_UI_FUNCTION_MEMBERS(name, function_type) using name##Fn = function_type; name##Fn name = {};
		VIEW3D_UI_API_TABLE(VIEW3D_UI_FUNCTION_MEMBERS)
		#undef VIEW3D_UI_FUNCTION_MEMBERS

		Dll(Dll&&) = delete;
		Dll(Dll const&) = delete;
		Dll& operator=(Dll&&) = delete;
		Dll& operator=(Dll const&) = delete;

		// The one process-wide loaded module instance, constructed on first use.
		static Dll& Get()
		{
			static Dll s_dll;
			return s_dll;
		}
	};

	#undef VIEW3D_UI_API_TABLE

	namespace impl
	{
		// Copy the calling thread's last recorded diagnostic out of the loaded DLL.
		inline std::string LastErrorMessage()
		{
			auto& dll = Dll::Get();
			auto required = std::uint32_t{};
			dll.LastError(nullptr, 0, &required);

			auto buffer = std::string(required, '\0');
			dll.LastError(buffer.data(), required, &required);
			if (!buffer.empty() && buffer.back() == '\0')
				buffer.pop_back(); // drop the ABI's NUL terminator; std::string carries its own length

			return buffer;
		}

		// Throw Exception carrying the calling thread's last diagnostic if 'status' is a failure.
		inline void Check(EStatus status)
		{
			if (status != EStatus::Success)
				throw Exception(status, LastErrorMessage());
		}
	}

	// Query the ABI version implemented by the loaded view3d-ui.dll.
	inline std::uint32_t ApiVersion()
	{
		return Dll::Get().ApiVersion();
	}

	// Query the current build's size for one public wire structure.
	inline std::uint32_t StructSize(EStructId struct_id)
	{
		auto size = std::uint32_t{};
		impl::Check(Dll::Get().StructSize(struct_id, &size));
		return size;
	}

	// Move-only RAII wrapper for one process-level module initialization token
	// (View3DUI_Initialise/Shutdown/ContextAbandon).
	class Runtime
	{
		RuntimeHandle m_handle;

	public:
		explicit Runtime(ReportErrorCB error_cb = {})
			: m_handle(Dll::Get().Initialise(error_cb))
		{
			if (m_handle == nullptr)
				throw Exception(EStatus::InternalError, impl::LastErrorMessage());
		}
		Runtime(Runtime&& rhs) noexcept
			: m_handle(rhs.m_handle)
		{
			rhs.m_handle = nullptr;
		}
		Runtime(Runtime const&) = delete;
		Runtime& operator=(Runtime&& rhs) noexcept
		{
			if (this == &rhs)
				return *this;

			Release();
			m_handle = rhs.m_handle;
			rhs.m_handle = nullptr;
			return *this;
		}
		Runtime& operator=(Runtime const&) = delete;
		~Runtime()
		{
			Release();
		}

		RuntimeHandle Handle() const
		{
			return m_handle;
		}

	private:
		// Release this initialization token. A live child UiContext makes Shutdown fail with
		// ResourceInUse; a destructor cannot throw, so fall back to the explicit best-effort
		// abandon path instead (section 8.1's explicit abandon/shutdown behavior).
		void Release()
		{
			if (m_handle == nullptr)
				return;

			if (Dll::Get().Shutdown(m_handle) != EStatus::Success)
				Dll::Get().ContextAbandon(m_handle);

			m_handle = nullptr;
		}
	};

	// Element count plus packed variable-length byte count, as reported by a zero-capacity probe
	// call to EventsCopy/SemanticsCopy (see UiContext::EventsPendingSizes/SemanticsPendingSizes).
	struct PendingSizes
	{
		std::uint32_t m_count;
		std::uint32_t m_payload_bytes;
	};

	// Move-only RAII wrapper for one owner-thread UI context. Every member call forwards directly
	// to the corresponding View3DUI_* export and throws Exception on any non-Success EStatus,
	// including WrongThread if called from a thread other than the one that created the context.
	class UiContext
	{
		ContextHandle m_handle;

	public:
		UiContext()
			: m_handle(0)
		{}
		UiContext(Runtime const& runtime, void* external_d3d12_device, void* view3d_window = nullptr)
			: UiContext(runtime, nullptr, external_d3d12_device, view3d_window)
		{}
		UiContext(Runtime const& runtime, Config const* config, void* external_d3d12_device, void* view3d_window = nullptr)
			: m_handle(0)
		{
			impl::Check(Dll::Get().ContextCreate(runtime.Handle(), config, external_d3d12_device, view3d_window, &m_handle));
		}
		UiContext(UiContext&& rhs) noexcept
			: m_handle(rhs.m_handle)
		{
			rhs.m_handle = 0;
		}
		UiContext(UiContext const&) = delete;
		UiContext& operator=(UiContext&& rhs) noexcept
		{
			if (this == &rhs)
				return *this;

			Release();
			m_handle = rhs.m_handle;
			rhs.m_handle = 0;
			return *this;
		}
		UiContext& operator=(UiContext const&) = delete;
		~UiContext()
		{
			Release();
		}

		// True once a live context handle has been created and not yet destroyed/moved-from.
		explicit operator bool() const
		{
			return m_handle != 0;
		}
		ContextHandle Handle() const
		{
			return m_handle;
		}

		// Validate and atomically apply one delta transaction.
		void TransactionApply(Transaction const& transaction) const
		{
			impl::Check(Dll::Get().TransactionApply(m_handle, &transaction));
		}

		// Advance host time, recompute layout/transitions/semantics/draw-packets.
		void Update(ViewportState const& viewport) const
		{
			impl::Check(Dll::Get().Update(m_handle, &viewport));
		}

		// Translate one raw Win32 message through the same normalized input state machine used
		// by InputInject. Returns the resulting '*consumed' value for convenience.
		std::int32_t ProcessWindowMessage(HWND hwnd, UINT msg, WPARAM wparam, LPARAM lparam, LRESULT& result, std::int32_t& invalidate) const
		{
			auto consumed = std::int32_t{};
			impl::Check(Dll::Get().ProcessWindowMessage(m_handle, hwnd, msg, wparam, lparam, &consumed, &result, &invalidate));
			return consumed;
		}

		// Inject one already-normalized input record (used by deterministic tests).
		void InputInject(NormalizedInput const& input) const
		{
			impl::Check(Dll::Get().InputInject(m_handle, &input));
		}

		// Inject one already-normalized text-carrying input record (TextInput or a composition
		// update/commit) together with its borrowed UTF-8 payload. The payload is copied before
		// this call returns, so the caller's buffer need not outlive it.
		void InputInjectText(NormalizedInput const& input, InputTextPayload const& payload) const
		{
			impl::Check(Dll::Get().InputInjectText(m_handle, &input, &payload));
		}

		// Caret rectangle for 'control_id' in viewport DIPs, for placing an IME candidate window.
		// '*valid' is 0 when the control is not a focused editable control, in which case the
		// rectangle is left unspecified rather than reported as an error.
		Rect CaretGeometry(ControlId control_id, std::int32_t& valid) const
		{
			auto caret = Rect{};
			impl::Check(Dll::Get().CaretGeometry(m_handle, control_id, &caret, &valid));
			return caret;
		}

		// Count of currently pending queued events.
		std::uint32_t EventCount() const
		{
			auto count = std::uint32_t{};
			impl::Check(Dll::Get().EventCount(m_handle, &count));
			return count;
		}

		// Required buffer sizes for a subsequent EventsCopy, without draining the queue. There is
		// no standalone ABI export for the payload byte count, so this issues the same zero-
		// capacity probe EventsCopy itself accepts, tolerating its expected BufferTooSmall result.
		PendingSizes EventsPendingSizes() const
		{
			auto required = std::uint32_t{};
			auto payload_required = std::uint32_t{};
			auto status = Dll::Get().EventsCopy(m_handle, nullptr, 0, &required, nullptr, 0, &payload_required);
			if (status != EStatus::Success && status != EStatus::BufferTooSmall)
				impl::Check(status);

			return {required, payload_required};
		}

		// Drain and copy the entire pending event queue plus its packed payload bytes. Throws
		// BufferTooSmall if either span is smaller than the true pending size (see
		// EventsPendingSizes).
		void EventsCopy(std::span<Event> events, std::span<std::byte> payload_blob) const
		{
			auto required = std::uint32_t{};
			auto payload_required = std::uint32_t{};
			impl::Check(Dll::Get().EventsCopy(m_handle, events.data(), static_cast<std::uint32_t>(events.size()), &required, payload_blob.data(), static_cast<std::uint32_t>(payload_blob.size()), &payload_required));
		}

		// Required buffer sizes for a subsequent SemanticsCopy; see EventsPendingSizes for why
		// this is a probe call rather than a dedicated export.
		PendingSizes SemanticsPendingSizes() const
		{
			auto required = std::uint32_t{};
			auto text_required = std::uint32_t{};
			auto status = Dll::Get().SemanticsCopy(m_handle, nullptr, 0, &required, nullptr, 0, &text_required);
			if (status != EStatus::Success && status != EStatus::BufferTooSmall)
				impl::Check(status);

			return {required, text_required};
		}

		// Copy the current (non-destructive) semantic snapshot plus its packed UTF-8 text bytes.
		void SemanticsCopy(std::span<SemanticNode> nodes, std::span<char> text_blob) const
		{
			auto required = std::uint32_t{};
			auto text_required = std::uint32_t{};
			impl::Check(Dll::Get().SemanticsCopy(m_handle, nodes.data(), static_cast<std::uint32_t>(nodes.size()), &required, text_blob.data(), static_cast<std::uint32_t>(text_blob.size()), &text_required));
		}

		// Bounded runtime counters and the last rejected-transaction failure category.
		Diagnostics DiagnosticsGet() const
		{
			auto diagnostics = Diagnostics{};
			impl::Check(Dll::Get().DiagnosticsGet(m_handle, &diagnostics));
			return diagnostics;
		}

	private:
		// Destroy this context. A destructor cannot throw, so fall back to the explicit
		// best-effort abandon path if the graceful destroy call itself fails.
		void Release()
		{
			if (m_handle == 0)
				return;

			if (Dll::Get().ContextDestroy(m_handle) != EStatus::Success)
				Dll::Get().UiContextAbandon(m_handle);

			m_handle = 0;
		}
	};
}
