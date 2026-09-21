//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2026
//*********************************************
#pragma once
#include "view3d-12/src/dll/dll_forward.h"

namespace pr::rdr12
{
	// Read the DLL's startup-only diagnostics opt-in before creating an adapter, factory or device.
	inline bool ReadDeviceDebugEnvironment()
	{
		// A two-character buffer accepts only one digit; longer and empty values are configuration errors.
		wchar_t value[2] = {};
		SetLastError(ERROR_SUCCESS);
		auto length = GetEnvironmentVariableW(L"VIEW3D_DEVICE_DEBUG", value, _countof(value));
		if (length == 0 && GetLastError() == ERROR_ENVVAR_NOT_FOUND)
			return false;

		if (length == 1 && (value[0] == L'0' || value[0] == L'1'))
			return value[0] == L'1';

		throw std::invalid_argument("VIEW3D_DEVICE_DEBUG must be unset, 0 (off), or 1 (on), before View3D initialisation");
	}
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
namespace pr::rdr12::tests
{
	// Environment selection is CPU-only and does not create or mutate a graphics device.
	PRUnitTest(DeviceDebugEnvironment, Quick)
	{
		// Restore the exact process value even when an assertion fails.
		struct Restore
		{
			bool m_existed;
			std::wstring m_value;

			Restore()
				:m_existed()
				,m_value()
			{
				// Preserve absence separately because an existing empty value is invalid but distinct from an unset variable.
				SetLastError(ERROR_SUCCESS);
				auto required = GetEnvironmentVariableW(L"VIEW3D_DEVICE_DEBUG", nullptr, 0);
				m_existed = required != 0 || GetLastError() != ERROR_ENVVAR_NOT_FOUND;
				if (required == 0)
					return;

				m_value.resize(required);
				PR_EXPECT(GetEnvironmentVariableW(L"VIEW3D_DEVICE_DEBUG", m_value.data(), required) == required - 1);
				m_value.resize(required - 1);
			}
			~Restore()
			{
				// The fixture owns this process-local override only for the duration of the test.
				SetEnvironmentVariableW(L"VIEW3D_DEVICE_DEBUG", m_existed ? m_value.c_str() : nullptr);
			}
		} restore;
		PR_EXPECT(SetEnvironmentVariableW(L"VIEW3D_DEVICE_DEBUG", nullptr) != FALSE);
		PR_EXPECT(!ReadDeviceDebugEnvironment());
		PR_EXPECT(SetEnvironmentVariableW(L"VIEW3D_DEVICE_DEBUG", L"0") != FALSE);
		PR_EXPECT(!ReadDeviceDebugEnvironment());
		PR_EXPECT(SetEnvironmentVariableW(L"VIEW3D_DEVICE_DEBUG", L"1") != FALSE);
		PR_EXPECT(ReadDeviceDebugEnvironment());
		for (auto value : {L"", L"2", L"true", L"01", L" 1", L"1 "})
		{
			// Reject misspelled options instead of silently running with a different diagnostic policy.
			PR_EXPECT(SetEnvironmentVariableW(L"VIEW3D_DEVICE_DEBUG", value) != FALSE);
			PR_THROWS(ReadDeviceDebugEnvironment(), std::invalid_argument);
		}
	}
}
#endif
