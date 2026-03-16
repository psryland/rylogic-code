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
		pr::ldraw::Builder builder;
		builder.Group("g", 0xFFFF0000).Box("b", 0xFF00FF00).box(1, 2, 3);

		pr::network::Winsock winsock;
		pr::network::socket_stream ldr;
		ldr.set_non_blocking();
		
		for (auto t = 0.f; t < 100000.0f; t += 0.01f, std::this_thread::sleep_for(std::chrono::milliseconds(10)))
		{
			// Note: Clear(), BinaryStream(), and Command() are not available in the new builder API.
			// This test needs reworking to use the new API for streaming LDraw over sockets.
			pr::ldraw::Builder frame;
			frame.Group("g", 0xFFFF0000).Box("b", 0xFF00FF00).box(1, 2, 3).o2w(m4x4::Transform(RotationRad<m3x4>(0, t, 0), v4::Origin()));

			if (ldr.connect("localhost", 1976).good())
			{
				auto data = frame.ToBinary();
				ldr.write(data.data(), data.size());
				ldr.flush();
			}
		}
	}
}
