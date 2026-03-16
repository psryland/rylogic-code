//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
// Feature Wish list:
// Ambient occlusion
//  - use the G-Buffer to sample normals in a spherical volume, scale
//    intensity based on average "up-ness"
// Thick Lines
//  - Use a geometry shader to support thick lines by turning a line list
//    into a tri strip
// Shadow Mapping
//  - Use 'Rylo-Shadows'
// Order Independent Alpha
//  - Try that thing you read about weighted alpha by screen depth
#pragma once

// To use runtime shaders:
//  - Set PR_RDR_RUNTIME_SHADERS=1 in the preprocessor defines.
//  - Rebuild
//  - Edit the HLSL files in '\projects\renderer11\shaders\hlsl'
//  - Run '\script\BuildShader.py <hlsl_filepath> x86 debug dbg' to build the HLSL file (with .cso files)
//  - Make sure the 'BuildShader.py' script is using the same version of 'fxc.exe' as VS.
//  - Note: Runtime shaders are hard coded to read from '\projects\renderer\shaders\hlsl\compiled\'
//  - Put a break point in '\projects\renderer11\shaders\shader.cpp:66' to ensure the compiled shader is being loaded
//  - Tips:
//      - Use Notepad++ and the NppExec->Execute plugin. Command: "py.exe P:\pr\script\BuildShader.py $(FULL_CURRENT_PATH) x86 debug dbg"  (use x86 or x64 depending on configuration being run)
//      - Check the Output window for 'Shader <myshader.cso> replaced'
//      - Use 'Start Graphics Debugging' -> Capture a frame -> Select the frame to launch VSGA -> Select the DrawIndexedPrimitive call -> then the green 'play' button
//      - Make sure fxc is run with the /Zi option to add debug symbols
//      - Add *Dependency "file.hlsl" to the LDraw script for testing the shader

#include "pr/view3d-12/forward.h"
#include "pr/view3d-12/main/renderer.h"
#include "pr/view3d-12/main/window.h"
#include "pr/view3d-12/scene/scene.h"
#include "pr/view3d-12/resource/resource_factory.h"
#include "pr/view3d-12/model/model.h"
#include "pr/view3d-12/model/nugget.h"
#include "pr/view3d-12/model/vertex_layout.h"
#include "pr/view3d-12/model/model_desc.h"
#include "pr/view3d-12/model/model_generator.h"
#include "pr/view3d-12/model/model_tree.h"
#include "pr/view3d-12/model/skeleton.h"
#include "pr/view3d-12/model/skin.h"
#include "pr/view3d-12/model/pose.h"
#include "pr/view3d-12/model/animation.h"
#include "pr/view3d-12/model/animator.h"
#include "pr/view3d-12/instance/instance.h"
#include "pr/view3d-12/texture/texture_desc.h"
#include "pr/view3d-12/texture/texture_2d.h"
#include "pr/view3d-12/texture/texture_loader.h"
#include "pr/view3d-12/shaders/shader.h"
#include "pr/view3d-12/shaders/shader_include_handler.h"
#include "pr/view3d-12/shaders/shader_point_sprites.h"
#include "pr/view3d-12/shaders/shader_show_normals.h"
#include "pr/view3d-12/shaders/shader_smap.h"
#include "pr/view3d-12/shaders/shader_thick_line.h"
#include "pr/view3d-12/ldraw/ldraw_object.h"
#include "pr/view3d-12/ldraw/ldraw_gizmo.h"
#include "pr/view3d-12/ldraw/ldraw_parsing.h"
#include "pr/view3d-12/utility/cmd_list.h"
#include "pr/view3d-12/utility/cmd_alloc.h"
#include "pr/view3d-12/utility/gpu_sync.h"
#include "pr/view3d-12/utility/keep_alive.h"
#include "pr/view3d-12/utility/utility.h"
#include "pr/view3d-12/utility/wrappers.h"
#include "pr/view3d-12/utility/features.h"
#include "pr/view3d-12/utility/barrier_batch.h"
#include "pr/view3d-12/utility/root_signature.h"
#include "pr/view3d-12/utility/update_resource.h"
#include "pr/view3d-12/utility/pipe_state.h"
#include "pr/view3d-12/utility/conversion.h"
