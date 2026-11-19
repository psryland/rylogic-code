//*********************************************
// Compute
//  Copyright (c) Rylogic Ltd 2026
//*********************************************
#pragma once
#include "pr/compute/forward.h"
#include "pr/compute/utility/utility.h"

namespace pr::compute
{
	// Process-wide diagnostics, chosen by the host before any D3D12 device is created.
	enum class EProcessDiagnostics :int
	{
		// Leave existing process settings unchanged.
		None = 0,
		DebugLayer = 1,
		GpuValidation = 2,
	};

	// Configure D3D12 validation and DRED once at process startup. Late changes can remove existing devices.
	// This stateless helper cannot observe devices or calls in other modules; the host owns their ordering.
	// Unavailable interfaces throw before any setting is changed. None does not disable externally enabled diagnostics.
	inline void ConfigureProcessDiagnostics(EProcessDiagnostics mode)
	{
		// Resolve the complete request before applying any process-wide setting.
		switch (mode)
		{
			case EProcessDiagnostics::None:
			{
				// Preserve any diagnostics already chosen by the host.
				return;
			}
			case EProcessDiagnostics::DebugLayer:
			case EProcessDiagnostics::GpuValidation:
			{
				// Acquiring both interfaces first leaves the process unchanged when diagnostics are unavailable.
				auto dred = D3DPtr<ID3D12DeviceRemovedExtendedDataSettings1>{};
				auto debug = D3DPtr<ID3D12Debug1>{};
				Check(D3D12GetDebugInterface(__uuidof(ID3D12DeviceRemovedExtendedDataSettings1), (void**)dred.address_of()));
				Check(D3D12GetDebugInterface(__uuidof(ID3D12Debug1), (void**)debug.address_of()));

				// Retain device-removal evidence alongside validation messages.
				dred->SetAutoBreadcrumbsEnablement(D3D12_DRED_ENABLEMENT_FORCED_ON);
				dred->SetPageFaultEnablement(D3D12_DRED_ENABLEMENT_FORCED_ON);
				dred->SetBreadcrumbContextEnablement(D3D12_DRED_ENABLEMENT_FORCED_ON);
				debug->EnableDebugLayer();
				switch (mode)
				{
					case EProcessDiagnostics::DebugLayer:
					{
						// CPU validation does not request GPU instrumentation.
						debug->SetEnableGPUBasedValidation(FALSE);
						break;
					}
					case EProcessDiagnostics::GpuValidation:
					{
						// GPU instrumentation is an explicit startup choice.
						debug->SetEnableGPUBasedValidation(TRUE);
						break;
					}
					default:
					{
						throw std::invalid_argument("Unknown process diagnostics mode");
					}
				}
				return;
			}
			default:
			{
				throw std::invalid_argument("Unknown process diagnostics mode");
			}
		}
	}

	// True when the device exposes validation messages from the process debug layer.
	inline bool HasDebugLayer(ID3D12Device* device)
	{
		// A normal device does not implement the debug-layer info queue.
		auto info = D3DPtr<ID3D12InfoQueue>{};
		return SUCCEEDED(device->QueryInterface(__uuidof(ID3D12InfoQueue), (void**)info.address_of()));
	}
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
namespace pr::compute
{
	// Test hosts configure validation before running any fixture that can create a device.
	inline constexpr EProcessDiagnostics TestProcessDiagnostics = IsDebug ? EProcessDiagnostics::DebugLayer : EProcessDiagnostics::None;
	inline std::optional<EProcessDiagnostics> RunnerProcessDiagnostics = {};

	// Consume one startup option before the standard test parser; absent options preserve the caller's default.
	// 'unconfigured' skips the startup call, while 'none' explicitly calls the no-change mode.
	inline bool TakeProcessDiagnosticsOption(std::vector<char*>& args, std::optional<EProcessDiagnostics>& mode, std::ostream& err)
	{
		// Validate the whole option before changing the selected mode or remaining arguments.
		constexpr auto prefix = std::string_view{"-process-diagnostics:"};
		auto found = args.end();
		for (auto it = args.begin(); it != args.end(); ++it)
		{
			// Reject duplicate choices rather than silently selecting the last.
			if (!std::string_view{*it}.starts_with(prefix))
				continue;

			if (found != args.end())
			{
				err << "Repeated " << prefix << " argument\n";
				return false;
			}
			found = it;
		}
		if (found == args.end())
			return true;

		// Keep this runner-only syntax out of the class filters.
		auto const value = std::string_view{*found}.substr(prefix.size());
		if (value == "unconfigured")
			mode = std::nullopt;
		else if (value == "none")
			mode = EProcessDiagnostics::None;
		else if (value == "debug")
			mode = EProcessDiagnostics::DebugLayer;
		else if (value == "gbv")
			mode = EProcessDiagnostics::GpuValidation;
		else
		{
			err << "Unknown process diagnostics mode: " << value << '\n';
			return false;
		}
		args.erase(found);
		return true;
	}
}
namespace pr::compute::tests
{
	// These controls run under the host's already selected process diagnostics.
	PRUnitTestClass(ProcessDiagnosticsTests)
	{
		PRUnitTestMethod(RejectsUnknownMode, Quick)
		{
			// Neither rejection nor the no-change mode may remove an existing device.
			auto device = D3DPtr<ID3D12Device>{};
			Check(D3D12CreateDevice(nullptr, D3D_FEATURE_LEVEL_11_0, __uuidof(ID3D12Device), (void**)device.address_of()));
			PR_THROWS(ConfigureProcessDiagnostics(static_cast<EProcessDiagnostics>(99)), std::invalid_argument);
			ConfigureProcessDiagnostics(EProcessDiagnostics::None);
			PR_EXPECT(device->GetDeviceRemovedReason() == S_OK);
		}
		PRUnitTestMethod(DevicesFollowStartupConfiguration, Quick)
		{
			// None and unconfigured make no claim that external diagnostic overrides are absent.
			auto device = D3DPtr<ID3D12Device>{};
			Check(D3D12CreateDevice(nullptr, D3D_FEATURE_LEVEL_11_0, __uuidof(ID3D12Device), (void**)device.address_of()));
			switch (RunnerProcessDiagnostics.value_or(EProcessDiagnostics::None))
			{
				case EProcessDiagnostics::None:
				{
					break;
				}
				case EProcessDiagnostics::DebugLayer:
				case EProcessDiagnostics::GpuValidation:
				{
					PR_EXPECT(HasDebugLayer(device.get()));
					break;
				}
				default:
				{
					throw std::invalid_argument("Unknown process diagnostics mode");
				}
			}
		}
		PRUnitTestMethod(RunnerOptionParses, Quick)
		{
			// Each supported startup choice leaves unrelated test arguments intact.
			char filter[] = "SomeTests";
			char none[] = "-process-diagnostics:none";
			char debug[] = "-process-diagnostics:debug";
			char gbv[] = "-process-diagnostics:gbv";
			char unconfigured[] = "-process-diagnostics:unconfigured";
			char unknown[] = "-process-diagnostics:full";
			auto err = std::ostringstream{};
			auto mode = std::optional<EProcessDiagnostics>{};
			auto args = std::vector<char*>{filter, gbv};
			PR_EXPECT(TakeProcessDiagnosticsOption(args, mode, err) && mode == EProcessDiagnostics::GpuValidation);
			PR_EXPECT(args.size() == 1 && args[0] == filter);
			args = {debug};
			PR_EXPECT(TakeProcessDiagnosticsOption(args, mode, err) && mode == EProcessDiagnostics::DebugLayer && args.empty());
			args = {none};
			PR_EXPECT(TakeProcessDiagnosticsOption(args, mode, err) && mode == EProcessDiagnostics::None && args.empty());
			args = {unconfigured};
			PR_EXPECT(TakeProcessDiagnosticsOption(args, mode, err) && !mode.has_value() && args.empty());
			args = {filter};
			PR_EXPECT(TakeProcessDiagnosticsOption(args, mode, err) && !mode.has_value() && args.size() == 1);
			PR_EXPECT(err.str().empty());

			// Invalid and duplicate options fail without consuming or replacing the input.
			args = {unknown};
			PR_EXPECT(!TakeProcessDiagnosticsOption(args, mode, err) && !mode.has_value() && args.size() == 1);
			args = {debug, none};
			PR_EXPECT(!TakeProcessDiagnosticsOption(args, mode, err) && !mode.has_value() && args.size() == 2);
			PR_EXPECT(!err.str().empty());
		}
	};
}
#endif
