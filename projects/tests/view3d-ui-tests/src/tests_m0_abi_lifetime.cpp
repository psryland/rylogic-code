//*********************************************
// View3DUI Tests
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// M0 tests (implementation-plan.md sections 5.1/5.2/8): ABI schema discovery, the stable
// status/last-error contract, generation-aware handle staleness, owner-thread enforcement, and
// explicit runtime/context lifetime including the abandon/shutdown fallback paths.
#include "pr/common/unittests.h"
#include "test_support.h"

namespace pr::view3d::ui::tests
{
	PRUnitTest(ApiVersionAndStructSizesEnumerateEveryStructId, Quick)
	{
		PR_EXPECT(ApiVersion() == VIEW3D_UI_API_VERSION);

		// Every EStructId the ABI can ever describe must report the exact native sizeof (section
		// 5.2); this is the complete set, not a hand-selected subset.
		PR_EXPECT(StructSize(EStructId::Config) == sizeof(Config));
		PR_EXPECT(StructSize(EStructId::Transaction) == sizeof(Transaction));
		PR_EXPECT(StructSize(EStructId::Operation) == sizeof(Operation));
		PR_EXPECT(StructSize(EStructId::Control) == sizeof(ControlDesc));
		PR_EXPECT(StructSize(EStructId::Resource) == sizeof(ResourceDesc));
		PR_EXPECT(StructSize(EStructId::Style) == sizeof(StyleDesc));
		PR_EXPECT(StructSize(EStructId::Template) == sizeof(TemplateDesc));
		PR_EXPECT(StructSize(EStructId::NormalizedInput) == sizeof(NormalizedInput));
		PR_EXPECT(StructSize(EStructId::InputTextPayload) == sizeof(InputTextPayload));
		PR_EXPECT(StructSize(EStructId::ViewportState) == sizeof(ViewportState));
		PR_EXPECT(StructSize(EStructId::Event) == sizeof(Event));
		PR_EXPECT(StructSize(EStructId::SemanticNode) == sizeof(SemanticNode));
		PR_EXPECT(StructSize(EStructId::Diagnostics) == sizeof(Diagnostics));
		PR_EXPECT(StructSize(EStructId::HostBridgeVersion) == sizeof(HostBridgeVersion));
		PR_EXPECT(StructSize(EStructId::HostPassContext) == sizeof(HostPassContext));

		// An unknown struct id is a stable, specific rejection, never a default/zero size.
		PR_THROWS(StructSize(static_cast<EStructId>(9999)), Exception);
	}

	PRUnitTest(LastErrorUsesTwoCallSizingContract, Quick)
	{
		// StructSize with a null output pointer is guaranteed to fail, giving a deterministic way
		// to exercise the ABI-wide two-call (probe-then-fetch) LastError sizing pattern (section
		// 5.1) without depending on any other subsystem's own failure path.
		auto probe_status = Dll::Get().StructSize(EStructId::Config, nullptr);
		PR_EXPECT(probe_status != EStatus::Success);

		auto required = std::uint32_t{};
		Dll::Get().LastError(nullptr, 0, &required);
		PR_EXPECT(required > 0);

		auto buffer = std::string(required, '\0');
		auto fetch_status = Dll::Get().LastError(buffer.data(), required, &required);
		PR_EXPECT(fetch_status == EStatus::Success);
		PR_EXPECT(!buffer.empty());
	}

	PRUnitTest(ContextCreateRequiresNonNullDevice, Quick)
	{
		auto runtime = Runtime{};
		PR_THROWS(UiContext(runtime, nullptr), Exception);
	}

	PRUnitTest(ContextCreateRejectsMalformedConfigHeader, Quick)
	{
		auto runtime = Runtime{};
		auto device = FakeDevice{};

		// A Config carrying the wrong struct version must be rejected before any resource is
		// created; the schema mismatch is caught even though 'size' itself is otherwise correct.
		auto bad_config = MakeConfig();
		bad_config.header.version = VIEW3D_UI_STRUCT_VERSION + 1;
		PR_THROWS(UiContext(runtime, &bad_config, &device), Exception);
	}

	PRUnitTest(ContextLifetimeConstructsAndDestructsCleanly, Quick)
	{
		auto runtime = Runtime{};
		auto device = FakeDevice{};
		auto ctx = UiContext(runtime, &device);
		PR_EXPECT(static_cast<bool>(ctx));
		PR_EXPECT(ctx.Handle() != 0);

		// A freshly created context must already report usable diagnostics.
		auto diagnostics = ctx.DiagnosticsGet();
		PR_EXPECT(diagnostics.control_count == 0);
		PR_EXPECT(diagnostics.accepted_revision == 0);
	}

	PRUnitTest(ContextHandleIsStaleAfterDestroy, Quick)
	{
		auto runtime = Runtime{};
		auto device = FakeDevice{};
		auto saved_handle = ContextHandle{};
		{
			auto ctx = UiContext(runtime, &device);
			saved_handle = ctx.Handle();
		} // ctx destructs here, releasing the handle back to the free list

		auto diagnostics = Diagnostics{};
		auto status = Dll::Get().DiagnosticsGet(saved_handle, &diagnostics);
		PR_EXPECT(status == EStatus::StaleHandle);
	}

	PRUnitTest(ContextHandleGenerationChangesOnSlotReuse, Quick)
	{
		auto runtime = Runtime{};
		auto handle_a = ContextHandle{};
		{
			auto device_a = FakeDevice{};
			auto ctx_a = UiContext(runtime, &device_a);
			handle_a = ctx_a.Handle();
		}

		// A second context may reuse the same slot index, but must never alias the first
		// context's now-stale handle: generation-aware handles are the whole point (section 8.1).
		auto device_b = FakeDevice{};
		auto ctx_b = UiContext(runtime, &device_b);
		PR_EXPECT(ctx_b.Handle() != handle_a);

		auto diagnostics = Diagnostics{};
		PR_EXPECT(Dll::Get().DiagnosticsGet(handle_a, &diagnostics) == EStatus::StaleHandle);
		PR_EXPECT(Dll::Get().DiagnosticsGet(ctx_b.Handle(), &diagnostics) == EStatus::Success);
	}

	PRUnitTest(ContextEnforcesOwnerThread, Quick)
	{
		auto runtime = Runtime{};
		auto device = FakeDevice{};
		auto ctx = UiContext(runtime, &device);

		// Every context-scoped call, including a read-only one, must be rejected from any thread
		// other than the one that created the context (section 8.2).
		auto observed_status = EStatus::Success;
		auto worker = std::thread([&]
		{
			try
			{
				ctx.DiagnosticsGet();
			}
			catch (Exception const& ex)
			{
				observed_status = ex.Status();
			}
		});
		worker.join();

		PR_EXPECT(observed_status == EStatus::WrongThread);

		// The owning thread must still be able to use the context normally afterwards.
		auto diagnostics = ctx.DiagnosticsGet();
		PR_EXPECT(diagnostics.control_count == 0);
	}

	PRUnitTest(ShutdownFailsWithResourceInUseWhileContextIsAlive, Quick)
	{
		auto runtime = Runtime{};
		auto device = FakeDevice{};
		auto ctx = UiContext(runtime, &device);
		PR_EXPECT(static_cast<bool>(ctx));

		// Shutdown must not succeed (nor silently detach the context) while any context created
		// from this runtime token is still alive (section 8.1's explicit shutdown behavior).
		auto status = Dll::Get().Shutdown(runtime.Handle());
		PR_EXPECT(status == EStatus::ResourceInUse);

		// 'ctx' destructs before 'runtime' (reverse declaration order), so the runtime's own
		// destructor can complete a graceful Shutdown once the context is gone.
	}

	PRUnitTest(ContextAbandonForgetsRuntimeWithoutContexts, Quick)
	{
		auto handle = Dll::Get().Initialise(ReportErrorCB{});
		PR_EXPECT(handle != nullptr);

		// Abandon must release the token even though Shutdown was never called, and must not
		// throw or crash (section 8.1's explicit abandon path, used as a destructor-safe
		// fallback by the Runtime/UiContext RAII wrappers).
		Dll::Get().ContextAbandon(handle);

		// The now-abandoned runtime handle can no longer create a context.
		auto device = FakeDevice{};
		auto context_out = ContextHandle{};
		auto status = Dll::Get().ContextCreate(handle, nullptr, &device, nullptr, &context_out);
		PR_EXPECT(status == EStatus::InvalidArgument);
	}

	PRUnitTest(UiContextAbandonReleasesHandleWithoutThrowing, Quick)
	{
		auto runtime = Runtime{};
		auto device = FakeDevice{};
		auto ctx = UiContext(runtime, &device);
		auto handle = ctx.Handle();

		// Abandon directly via the raw export, bypassing UiContext's own graceful Destroy path.
		Dll::Get().UiContextAbandon(handle);

		// The handle is now stale, exactly as if ContextDestroy had run.
		auto diagnostics = Diagnostics{};
		PR_EXPECT(Dll::Get().DiagnosticsGet(handle, &diagnostics) == EStatus::StaleHandle);

		// 'ctx' must not double-release or crash when it goes out of scope: its own destructor's
		// ContextDestroy call sees a stale handle and falls back to UiContextAbandon, which is
		// itself a safe no-op against an already-abandoned slot.
	}

	PRUnitTest(ContextCreateWithHostWindowFailsWithoutHostBridgeAndLeavesNoState, Quick)
	{
		// This test process never loads view3d-12.dll, so the private UI host bridge's four named
		// exports (View3D_UIHostApiVersion/StructSize/Attach/Detach) cannot be resolved; a non-null
		// 'view3d_window' must therefore fail with UnsupportedFeature rather than crash, and the
		// attempt must commit no context state (the bridge attach is attempted before anything is
		// mutated - implementation-plan.md section 4).
		auto runtime = Runtime{};
		auto device = FakeDevice{};
		auto fake_window = int{};
		auto observed_status = EStatus::Success;
		try
		{
			auto ctx = UiContext(runtime, &device, &fake_window);
		}
		catch (Exception const& ex)
		{
			observed_status = ex.Status();
		}
		PR_EXPECT(observed_status == EStatus::UnsupportedFeature);

		// The failed attempt left no orphaned slot behind: a normal, windowless context still
		// constructs and destructs cleanly using the same runtime/device.
		auto ctx = UiContext(runtime, &device);
		PR_EXPECT(static_cast<bool>(ctx));
	}
}
