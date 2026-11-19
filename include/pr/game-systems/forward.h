//************************************
// Game Systems
//  Copyright (c) Rylogic Ltd 2026
//************************************
// Game-agnostic world systems (sky, ocean, climate, particles) built on View3D.
// This header holds every foreign include for the library, like a precompiled header.
#pragma once
#include "pr/view3d-12/forward.h"
#include "pr/view3d-12/instance/instance.h"
#include "pr/view3d-12/main/renderer.h"
#include "pr/view3d-12/main/window.h"
#include "pr/view3d-12/scene/scene.h"
#include "pr/view3d-12/scene/cloud_sky.h"
#include "pr/view3d-12/shaders/shader.h"
#include "pr/view3d-12/shaders/shader_forward.h"
#include "pr/view3d-12/model/model_generator.h"
#include "pr/view3d-12/model/vertex_layout.h"
#include "pr/view3d-12/material/material_simple.h"
#include "pr/view3d-12/resource/resource_factory.h"
#include "pr/view3d-12/resource/resource_store.h"
#include "pr/view3d-12/texture/texture_desc.h"
#include "pr/view3d-12/texture/texture_2d.h"
#include "pr/view3d-12/texture/texture_cube.h"

namespace pr::game_systems
{
	// Game systems are written in terms of the renderer's types, so its names are visible without qualification.
	using namespace pr::rdr12;

	// Sky
	struct ProceduralSky;
	struct ProceduralSkySettings;
	struct WeatherMap;
	using WeatherMapPtr = pr::RefPtr<WeatherMap>;
}
