//*****************************************************************************************
// Application Framework
//  Copyright (c) Rylogic Ltd 2012
//*****************************************************************************************
#include "pr/app/main.h"
#include "pr/app/main_ui.h"
#ifdef PR_APP_MAIN_INCLUDE
#include PR_APP_MAIN_INCLUDE
#endif

using namespace pr;
using namespace pr::app;

int __stdcall WinMain(HINSTANCE hInstance, HINSTANCE /*hPrevInstance*/, LPTSTR lpstrCmdLine, int nCmdShow)
{
	(void)hInstance;
	int nRet;
	#ifdef PR_APP_COMMAND_LINE_HANDLER

	// Give applications a pre-GUI command mode without duplicating the shared entry-point lifecycle.
	if (PR_APP_COMMAND_LINE_HANDLER(lpstrCmdLine, nRet))
		return nRet;

	#endif
	{
		std::string err_msg;
		std::unique_ptr<IAppMainUI> gui;
		try
		{
			// CoInitialise COM
			InitCom init_com;

			// Create an instance of the main window and start it running
			auto cmd_line = Widen(lpstrCmdLine);
			gui = pr::app::CreateUI(cmd_line.c_str(), nCmdShow);
			nRet = gui->Run();
		}
		catch (std::exception const& ex)
		{
			auto last_error = GetLastError();
			auto res = HRESULT_FROM_WIN32(last_error);

			std::string ex_msg(ex.what());
			ex_msg.substr(0, ex_msg.find_last_not_of(" \t\r\n") + 1).swap(ex_msg);
			err_msg = Fmt("Application shutdown due to unhandled error:\r\nError Message: '%s'", ex_msg.c_str());
			if (res != S_OK) err_msg += Fmt("\r\nLast Error Code: %X - %s", res, HrMsg(res).c_str());
			nRet = -1;
		}
		catch (...)
		{
			err_msg = "Shutting down due to an unknown exception";
			nRet = -1;
		}

		// Attempt to shut the window down gracefully
		gui = nullptr;

		if (nRet == -1)
		{
			std::thread([&] { ::MessageBoxA(0, err_msg.c_str(), "Application Error", MB_OK|MB_ICONERROR); }).join();
		}
	}
	return nRet;
}

#if defined _M_IX86
#pragma comment(linker, "/manifestdependency:\"type='win32' name='Microsoft.Windows.Common-Controls' version='6.0.0.0' processorArchitecture='x86' publicKeyToken='6595b64144ccf1df' language='*'\"")
#elif defined _M_IA64
#pragma comment(linker, "/manifestdependency:\"type='win32' name='Microsoft.Windows.Common-Controls' version='6.0.0.0' processorArchitecture='ia64' publicKeyToken='6595b64144ccf1df' language='*'\"")
#elif defined _M_X64
#pragma comment(linker, "/manifestdependency:\"type='win32' name='Microsoft.Windows.Common-Controls' version='6.0.0.0' processorArchitecture='amd64' publicKeyToken='6595b64144ccf1df' language='*'\"")
#else
#pragma comment(linker, "/manifestdependency:\"type='win32' name='Microsoft.Windows.Common-Controls' version='6.0.0.0' processorArchitecture='*' publicKeyToken='6595b64144ccf1df' language='*'\"")
#endif
