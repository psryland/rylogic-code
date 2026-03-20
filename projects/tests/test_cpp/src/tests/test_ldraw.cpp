#include <thread>
#include "pr/math/math.h"
#include "pr/network/winsock.h"
#include "pr/network/socket_stream.h"
#include "pr/common/ldraw.h"

using namespace pr;

namespace tests
{
	void Run()
	{
		pr::network::Winsock winsock;
		pr::network::socket_stream ldr;
		ldr.set_non_blocking();

		bool use_text = true;

		// Sleep for 'ms'
		auto wait = [](int ms)
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(ms));
		};
		
		pr::ldraw::Builder frame;
		for (auto t = 0.f; t < 100000.0f; t += 0.01f, wait(10))
		{
			frame.Clear();

			if (!ldr.is_open() || !ldr.good())
			{
				std::cout << "Disconnected\n";
				for (; !ldr.connect("localhost", 1976).good() || !ldr.is_open(); wait(100)) {}
				std::cout << "Connected\n";

				frame.Group("g", 0xFFFF0000).Box("b", 0xFF00FF00).box(1, 2, 3);
				frame.Commands().add_to_scene(0);
			}

			frame.Commands()
				.object_transform("g", m4x4::Transform(RotationRad<m3x4>(0, t, 0), v4::Origin()))
				.render(0);

			if (use_text)
			{
				auto text = frame.ToString();
				ldr.write(reinterpret_cast<std::byte const*>(text.data()), text.size());
				ldr.flush();
			}
			else
			{
				auto data = frame.ToBinary();
				ldr.write(data.data(), data.size());
				ldr.flush();
			}
		}
	}
}
