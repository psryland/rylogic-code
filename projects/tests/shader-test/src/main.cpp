//*************************************************************************
// Shader Test
//  Copyright (c) Rylogic Ltd 2024
//*************************************************************************
#include "pr/hlsl/interop.h"

using namespace pr::hlsl;

// Notes:
// - Include hlsl files here as C++ code (one at a time)
// - Update the EntryPoint to if needed
namespace shader
{
	#include "src/example.hlsl"

	int EntryPoint()
	{
		// Do buffer setup here
		InputTriangles.push_back({ {}, {}, {} });
		InputTriangles.push_back({ {}, {}, {} });
		InputTriangles.push_back({ {}, {}, {} });
		InputTriangles.push_back({ {}, {}, {} });
		Constants.NumTriangles = int(InputTriangles.size());
		OutputNormals.resize(Constants.NumTriangles);

		GpuEmulator emu(CSFaceNormal, CSFaceNormal_NumThreads);
		emu.Dispatch({ 1, 1, 1 });
		return 0;
	}
}

int main()
{
	shader::EntryPoint();
	return 0;
}
