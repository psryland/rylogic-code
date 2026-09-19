#pragma once
#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>
#include <windows.h>
#include <d3d12.h>
#include <d3d12sdklayers.h>
#include <wrl/client.h>
#include "pr/view3d-12/view3d-dll.h"
#include "pr/view3d-12/view3d-ui-bridge.h"
#include "pr/view3d-12/model/model_generator.h"
#include "pr/view3d-12/model/pose.h"
#include "pr/view3d-12/model/skeleton.h"
#include "pr/view3d-12/model/animator.h"
#include "pr/view3d-12/scene/far_clip_fade.h"
#include "pr/view3d-12/shaders/shader.h"
#include "pr/view3d-12/material/material_simple.h"
#include "pr/view3d-12/resource/resource_factory.h"
#include "view3d-12/src/render/render_raycast.h"
#include "pr/view3d-12/utility/normal_transform.h"
#include "pr/view3d-12/ldraw/ldraw_object.h"
#include "view3d-12/src/dll/v3d_window.h"
#include "pr/hlsl/interop.h"

// Exercise the same RGB blend expression compiled into the renderer's GPU shaders.
namespace colour_blend_tests
{
	using pr::rdr12::ldraw::RdrInstance;
	static_assert(int(pr::rdr12::EInstComp::ColourBlend32) == int(pr::rdr12::EInstComp::TintColour32) + 1);
	static_assert(pr::rdr12::SizeOf(pr::rdr12::EInstComp::ColourBlend32) == sizeof(pr::Colour32));
	static_assert(std::is_same_v<decltype(RdrInstance::m_colour_blend), pr::Colour32>);
	static_assert(offsetof(RdrInstance, m_colour_blend) == offsetof(RdrInstance, m_colour) + sizeof(pr::Colour32));

	using namespace pr::hlsl;
	#include "view3d-12/src/shaders/hlsl/utility/surface_colour.hlsli"
}

// Fixture-owned bytecode exercises the custom-stage boundary without a renderer library dependency.
namespace fade_tests::compiled
{
	#include "fade_vertex.h"
	#include "procedural_vertex_forward.h"
	#include "procedural_vertex_raycast.h"
	#include "procedural_vertex_shadow.h"
	#include "unsupported_pixel.h"
}

namespace fade_tests
{
	// Exercise native RayCast cancellation and teardown independently of DLL-owned resources.
	void RayCastLifetimeNativeTests();
}
