//*********************************************
// View3DUI Tests
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// M3 tests for (a) the private view3d-12 UI host bridge's ABI as actually exported by the built
// view3d-12.dll, entirely independent of host_bridge.cpp's own permanent export-resolution cache,
// and (b) View3DUI_ContextCreate's all-or-nothing COM-rollback contract when the host bridge
// attach step fails. The bridge-integration test only runs its assertions when view3d-12.dll can
// actually be found via the process's normal DLL search order; it always balances its own
// LoadLibrary with a matching FreeLibrary, so it can never change whether a later test in this
// same process observes view3d-12.dll as loaded (see tests_m0_abi_lifetime.cpp's
// ContextCreateWithHostWindowFailsWithoutHostBridgeAndLeavesNoState, whose "this test process
// never loads view3d-12.dll" assumption must remain true regardless of test run order).
#include "pr/common/unittests.h"
#include "test_support.h"
#include "pr/view3d-12/view3d-ui-bridge.h"

namespace pr::view3d::ui::tests
{
	PRUnitTest(BridgeExportsMatchThisModulesAssumedAbiAndStructLayout, Quick)
	{
		// Deliberately bypasses host_bridge.cpp entirely: this resolves the four named exports
		// straight off the freshly-loaded module, exactly as a from-scratch integration check
		// against the real, currently-built view3d-12.dll should, rather than reusing (and so
		// never actually exercising) host_bridge.cpp's own resolution code path.
		auto module = ::LoadLibraryW(L"view3d-12.dll");
		if (module == nullptr)
			return; // not found via the process's normal DLL search order in this build/run; nothing to verify

		auto api_version_fn = reinterpret_cast<ApiVersionFn>(::GetProcAddress(module, ApiVersionExport));
		auto struct_size_fn = reinterpret_cast<StructSizeFn>(::GetProcAddress(module, StructSizeExport));
		auto attach_fn = reinterpret_cast<AttachFn>(::GetProcAddress(module, AttachExport));
		auto detach_fn = reinterpret_cast<DetachFn>(::GetProcAddress(module, DetachExport));
		PR_EXPECT(api_version_fn != nullptr);
		PR_EXPECT(struct_size_fn != nullptr);
		PR_EXPECT(attach_fn != nullptr);
		PR_EXPECT(detach_fn != nullptr);

		if (api_version_fn != nullptr)
			PR_EXPECT(api_version_fn() == HostApiVersion);

		if (struct_size_fn != nullptr)
		{
			auto provider_size = std::uint32_t{};
			PR_EXPECT(struct_size_fn(EHostStructId::Provider, &provider_size) == EHostStatus::Success);
			PR_EXPECT(provider_size == sizeof(Provider));

			auto pass_size = std::uint32_t{};
			PR_EXPECT(struct_size_fn(EHostStructId::Pass, &pass_size) == EHostStatus::Success);
			PR_EXPECT(pass_size == sizeof(Pass));
		}

		// Balance the LoadLibrary above unconditionally, regardless of which assertions above
		// failed, so this test can never leave view3d-12.dll loaded for any test that runs after it.
		::FreeLibrary(module);
	}

	PRUnitTest(ContextCreateRollsBackTheDeviceAddRefWhenTheHostBridgeAttachFails, Quick)
	{
		// Mirrors tests_m0_abi_lifetime.cpp's ContextCreateWithHostWindowFailsWithoutHostBridge
		// AndLeavesNoState, but additionally asserts the COM refcount side of the "all-or-nothing"
		// contract: a failed attach must leave the externally-owned device exactly as it found it,
		// not holding an orphaned AddRef the failed context can never release.
		auto runtime = Runtime{};
		auto device = FakeDevice{};
		auto fake_window = int{};
		auto refcount_before = device.RefCount();

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
		PR_EXPECT(device.RefCount() == refcount_before);
	}
}
