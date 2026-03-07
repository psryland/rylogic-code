#if 1
#include <thread>
#include "pr/common/min_max_fix.h"
#include "pr/network/socket_stream.h"
#include "pr/view3d-12/ldraw/ldraw_builder.h"

using namespace pr;

namespace tests
{
	void Run()
	{
		pr::rdr12::ldraw::Builder builder;
		builder.Group("g", 0xFFFF0000).Box("b", 0xFF00FF00).dim(1, 2, 3);

		pr::network::Winsock winsock;
		pr::network::socket_stream ldr;
		ldr.set_non_blocking();
		
		for (auto t = 0.f; t < 100000.0f; t += 0.01f, std::this_thread::sleep_for(std::chrono::milliseconds(10)))
		{
			builder.Clear();
			builder.BinaryStream();
			builder.Command().object_transform("g", m4x4::TransformRad(0, t, 0, v4::Origin()));

			if (ldr.connect("localhost", 1976).good())
				ldr << builder.ToBinary() << std::flush;
		}

		//std::cout << builder.ToString(true);
		//ldr << "*Box bb FF00FF00 { *Data {1 2 3} }";
		//ldr.flush();

	}
}
#endif
