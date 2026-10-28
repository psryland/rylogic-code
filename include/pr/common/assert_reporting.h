//*****************************************************************
// Assert Reporting
//  Copyright (c) Rylogic Ltd 2026
//*****************************************************************
// Routes CRT assertion failures and errors to stderr instead of modal dialogs, so that unattended runs (e.g. build-invoked unit tests)
// report failures and keep running rather than blocking on a dialog.
//
// Each module (exe or dll) that statically links the CRT has its own report settings, so every module must apply them itself:
//  - DLLs call 'ApplyAssertReportingFromEnvironment' from DllMain, which applies the settings when the 'PR_ASSERTS_TO_STDERR'
//    environment variable is set. The variable is inherited by child processes, so it reaches every DLL in a test host process.
//  - Executables call 'RouteAssertsToStdErr' directly when their own command line asks for it.
#pragma once
#include <cstdlib>
#include <stdlib.h>
#if defined(_WIN32) && defined(_DEBUG)
#  include <crtdbg.h>
#endif

namespace pr
{
	// The environment variable that requests stderr assert reporting in every module of a process. Any non-empty value enables it.
	inline constexpr char const* AssertsToStdErrEnvVar = "PR_ASSERTS_TO_STDERR";

	// Send the calling module's CRT assertion failures and errors to stderr and the debugger output, rather than showing dialogs.
	inline void RouteAssertsToStdErr()
	{
		// 'assert()' in non-debug CRTs, and abort messages, use the error mode. Skip the Windows Error Reporting prompt that 'abort()' can raise.
		_set_error_mode(_OUT_TO_STDERR);
		_set_abort_behavior(0, _CALL_REPORTFAULT);

		// The debug CRT reports '_CRT_ASSERT' and '_CRT_ERROR' through '_CrtDbgReport', which shows a dialog by default.
		// '_CRT_WARN' already goes to the debugger output only, and 'pr::AssertionFailed' writes its own message to stderr.
		#if defined(_WIN32) && defined(_DEBUG)
		for (auto report_type : {_CRT_ERROR, _CRT_ASSERT})
		{
			_CrtSetReportMode(report_type, _CRTDBG_MODE_FILE | _CRTDBG_MODE_DEBUG);
			_CrtSetReportFile(report_type, _CRTDBG_FILE_STDERR);
		}
		#endif
	}

	// True when the environment requests stderr assert reporting (see 'AssertsToStdErrEnvVar').
	inline bool AssertsToStdErrRequested()
	{
		// 'getenv_s' reports the required buffer size, which is zero when the variable is not set.
		auto len = size_t{};
		return getenv_s(&len, nullptr, 0, AssertsToStdErrEnvVar) == 0 && len > 1;
	}

	// Apply stderr assert reporting to the calling module when the environment requests it. Safe to call from DllMain.
	inline void ApplyAssertReportingFromEnvironment()
	{
		// Leave the default (dialog) reporting in place for interactive use.
		if (AssertsToStdErrRequested())
			RouteAssertsToStdErr();
	}
}
