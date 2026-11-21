//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2026
//*********************************************
// Public shader contract for nuggets drawn with GPU-written draw arguments (see NuggetDrawArgs and View3D_ObjectNuggetDrawArgsSet).
//
// Before each view draws such a nugget, the renderer tests each instance's bounding sphere against the view and draws only the instances
// that the view may see. The drawn instances are a compacted, unordered list of the original instance indices. A vertex shader of such a
// nugget must declare the list and find its original instance with 'View3DIndirectInstance':
//
//   ByteAddressBuffer g_instances : register(VIEW3D_INDIRECT_INSTANCES_REGISTER);
//   uint instance = View3DIndirectInstance(g_instances, instance_id); // 'instance_id' is SV_InstanceID
//
// The list is only bound for forward draws of GPU-argument nuggets, which never draw populations, so it shares the population view register.
#ifndef PR_VIEW3D_INDIRECT_INSTANCES_HLSLI
#define PR_VIEW3D_INDIRECT_INSTANCES_HLSLI
#include "pr/view3d-12/shaders/population.hlsli"

// Contract version of the bindings in this file.
#define VIEW3D_INDIRECT_INSTANCES_VERSION 1

// The register of the visible instance list: one uint original instance index per drawn instance.
#define VIEW3D_INDIRECT_INSTANCES_REGISTER VIEW3D_POPULATION_VIEW_REGISTER

// Return the original instance index of drawn instance 'instance_id' (SV_InstanceID).
uint View3DIndirectInstance(ByteAddressBuffer instances, uint instance_id)
{
	return instances.Load(instance_id * 4);
}

#endif
