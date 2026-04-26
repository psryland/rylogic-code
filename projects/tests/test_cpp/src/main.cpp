#include <iostream>
#include "pr/common/min_max_fix.h"

namespace tests
{
	namespace input { void Run(); }
	namespace ldraw { void Run(); }
}

#if 0
int APIENTRY tWinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPTSTR lpCmdLine, int nCmdShow) { (void)hInstance, hPrevInstance, lpCmdLine, nCmdShow;
#else
int main(int argc, char* argv[]) { (void)argc, argv;
#endif
	try
	{
		tests::input::Run();
		return 0;
	}
	catch (std::exception const& ex)
	{
		std::cerr << ex.what() << "\n";
		return -1;
	}
}

