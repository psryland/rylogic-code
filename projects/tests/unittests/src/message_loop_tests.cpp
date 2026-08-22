//*********************************************
// Message Loop Tests
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#include "pr/common/unittests.h"
#include "pr/gui/message_loop.h"

namespace pr::unittests
{
	PRUnitTestClass(MessageLoopTests)
	{
		PRUnitTestMethod(QuitStopsScheduledCallbacks, Quick)
		{
			auto callback_called = false;
			auto exit_code = std::optional<int>{};
			auto worker = std::thread([&]
				{
					auto loop = gui::MessageLoop{};

					// Create this thread's message queue before requesting its quit message.
					loop.Pump(0);
					loop.AddLoop(60.0, false, [&](double)
						{
							callback_called = true;
						});
					loop.RequestQuit(7);

					PR_EXPECT(loop.StepLoops() == INFINITE);
					exit_code = loop.Pump(0);
				});
			worker.join();

			PR_EXPECT(!callback_called);
			PR_EXPECT(exit_code && *exit_code == 7);
		}
	};
}
