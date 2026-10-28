//*****************************************************************************
// Procedural animation
//  Copyright (c) Rylogic Ltd 2026
//*****************************************************************************
// A reusable procedural animation core. A rig description (bones, role-tagged chains and contact points) is posed each frame from model-space targets:
//  - 'Rig' validates the description and caches derived data (rig.h). Rig descriptions are stored as '.rig' JSON files (rig_json.h).
//  - 'Pose' holds local and model-space bone transforms (pose.h).
//  - 'TargetPath' and 'Fade' move targets and layer weights smoothly with quintic curves (target_path.h).
//  - 'Layer' and 'ApplyLayers' combine per-chain targets (targets.h).
//  - 'SolvePose' poses spine, neck and limb chains (solve.h).
//  - 'GaitDesc' describes how a character steps (gait.h), and 'Locomotion' poses a rig, including its tails and digits, from its body's motion over the ground (locomotion.h).
#pragma once
#include "pr/animation/forward.h"
#include "pr/animation/rig.h"
#include "pr/animation/rig_json.h"
#include "pr/animation/pose.h"
#include "pr/animation/two_bone_ik.h"
#include "pr/animation/spine_fit.h"
#include "pr/animation/aim_chain.h"
#include "pr/animation/target_path.h"
#include "pr/animation/targets.h"
#include "pr/animation/solve.h"
#include "pr/animation/gait.h"
#include "pr/animation/locomotion.h"
