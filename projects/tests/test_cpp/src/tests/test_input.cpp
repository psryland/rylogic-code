// test_input.cpp - exercise pr::input::Keyboard / Mouse / Joystick
// Press ESC to quit. Mouse state shows accumulated deltas per refresh.

#include <atomic>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <windows.h>
#include "pr/win32/dummy_window.h"
#include "pr/input/keyboard.h"
#include "pr/input/mouse.h"
#include "pr/input/joystick.h"
#include "pr/camera/flight.h" // compile-only check

using namespace pr;

namespace tests::input
{
	// Dummy window subclass that owns the keyboard / mouse and forwards WM_INPUT.
	struct InputWindow : pr::DummyWindow
	{
		pr::input::Keyboard m_kb;
		pr::input::Mouse    m_mouse;
		std::atomic<bool>   m_quit{false};

		InputWindow()
			:DummyWindow()
			,m_kb(Hwnd())
			,m_mouse(Hwnd())
		{
		}

		LRESULT WndProc(HWND hwnd, UINT message, WPARAM wparam, LPARAM lparam) override
		{
			switch (message)
			{
				case WM_INPUT:
				{
					if (!m_kb.OnRawInput(lparam))
						m_mouse.OnRawInput(lparam);
					break;
				}
				default: break;
			}
			return DummyWindow::WndProc(hwnd, message, wparam, lparam);
		}
	};

	// Convert a few common VKs to readable names.
	static char const* VkName(int vk)
	{
		switch (vk)
		{
			case VK_LEFT: return "Left";
			case VK_RIGHT: return "Right";
			case VK_UP: return "Up";
			case VK_DOWN: return "Down";
			case VK_SHIFT: return "Shift";
			case VK_LSHIFT: return "LShift";
			case VK_RSHIFT: return "RShift";
			case VK_CONTROL: return "Ctrl";
			case VK_LCONTROL: return "LCtrl";
			case VK_RCONTROL: return "RCtrl";
			case VK_MENU: return "Alt";
			case VK_LMENU: return "LAlt";
			case VK_RMENU: return "RAlt";
			case VK_SPACE: return "Space";
			case VK_RETURN: return "Enter";
			case VK_TAB: return "Tab";
			case VK_BACK: return "Back";
			case VK_ESCAPE: return "Esc";
			default: break;
		}
		return nullptr;
	}

	void Run()
	{
		std::cout << "test_input - ESC to quit\n";

		InputWindow win;

		// Joysticks - enumerate up-front. New devices plugged in later won't appear
		// until the next call to Enumerate(), but existing entries keep working.
		auto sticks = pr::input::Joystick::Enumerate();
		std::cout << "Joysticks detected: " << sticks.size() << "\n";
		for (auto& js : sticks)
		{
			std::cout << "  " << js.name() << "  axes=" << js.axis_count()
				<< " buttons=" << js.button_count()
				<< " switches=" << js.switch_count()
				<< (js.is_gamepad() ? " [gamepad]" : "")
				<< "\n";
		}
		std::cout << "\n";

		auto last_print = std::chrono::steady_clock::now();
		HANDLE hStdOut = ::GetStdHandle(STD_OUTPUT_HANDLE);

		for (;;)
		{
			// Pump messages so WM_INPUT is delivered to the window's WndProc.
			if (!win.Pump()) break;

			if (win.m_kb.KeyDown(VK_ESCAPE)) break;

			auto now = std::chrono::steady_clock::now();
			if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_print).count() < 50)
			{
				std::this_thread::sleep_for(std::chrono::milliseconds(5));
				continue;
			}
			last_print = now;

			// Move console cursor to top-left for in-place refresh.
			CONSOLE_SCREEN_BUFFER_INFO csbi;
			if (::GetConsoleScreenBufferInfo(hStdOut, &csbi))
			{
				COORD home = {0, 4}; // leave first 4 lines (banner)
				::SetConsoleCursorPosition(hStdOut, home);
			}

			// Keyboard - list all currently-down keys.
			std::cout << "Keys:    ";
			int printed = 0;
			for (int vk = 0; vk < 256; ++vk)
			{
				if (!win.m_kb.KeyDown(vk)) continue;
				if (printed++) std::cout << ' ';
				if (auto n = VkName(vk)) std::cout << n;
				else if (vk >= '0' && vk <= 'Z') std::cout << char(vk);
				else std::cout << "0x" << std::hex << vk << std::dec;
			}
			std::cout << "                                  \n";

			// Mouse - read deltas then snapshot to clear them.
			long dx = win.m_mouse.dx(), dy = win.m_mouse.dy(), dz = win.m_mouse.dz();
			std::cout << "Mouse:   dx=" << dx << " dy=" << dy << " dz=" << dz
				<< "  L=" << int(win.m_mouse.btn(pr::input::Mouse::Left))
				<< " R=" << int(win.m_mouse.btn(pr::input::Mouse::Right))
				<< " M=" << int(win.m_mouse.btn(pr::input::Mouse::Middle))
				<< " X1=" << int(win.m_mouse.btn(pr::input::Mouse::X1))
				<< " X2=" << int(win.m_mouse.btn(pr::input::Mouse::X2))
				<< "                       \n";
			win.m_mouse.Snapshot();

			// Joysticks
			for (size_t s = 0; s != sticks.size(); ++s)
			{
				auto& js = sticks[s];
				js.Sample();
				std::cout << "Js[" << s << "]: ";
				if (!js.valid()) { std::cout << "(disconnected)                                                \n"; continue; }
				if (js.is_gamepad())
				{
					std::cout << "L(" << js.lx() << "," << js.ly() << ") R(" << js.rx() << "," << js.ry()
						<< ") LT=" << js.lt() << " RT=" << js.rt()
						<< " btns=0x" << std::hex << uint16_t(js.buttons()) << std::dec
						<< "  raw_axes[";
					for (size_t a = 0; a != js.axis_count(); ++a)
					{
						if (a) std::cout << ',';
						std::cout << js.axis(a);
					}
					std::cout << "] raw_btns=";
					for (size_t b = 0; b != js.button_count(); ++b) std::cout << int(js.btn(b));
				}
				else
				{
					std::cout << "axes[";
					for (size_t a = 0; a != js.axis_count(); ++a)
					{
						if (a) std::cout << ',';
						std::cout << js.axis(a);
					}
					std::cout << "] btns=";
					for (size_t b = 0; b != js.button_count(); ++b) std::cout << int(js.btn(b));
					if (js.switch_count() > 0)
					{
						std::cout << " hat=";
						for (size_t h = 0; h != js.switch_count(); ++h) std::cout << int(js.hat(h)) << ' ';
					}
				}
				std::cout << "                                          \n";
			}

			std::cout.flush();
			win.m_kb.Snapshot();
		}

		std::cout << "\nExiting.\n";
	}
}
