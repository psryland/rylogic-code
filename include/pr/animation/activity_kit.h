//*****************************************************************************
// Procedural animation - shared parts for activities
//  Copyright (c) Rylogic Ltd 2026
//*****************************************************************************
// Building blocks that activity types share, so that their motion is consistent and their code is not repeated:
//  - Easing curves and body side signs.
//  - The swing arc: the path of a tool point (a fist, an axe head) from a rest position out to a strike and back.
//  - The body drive: the torso turn, lean, sideways bend and hip shift that put the body's weight behind a motion.
//  - Footwork: a foot that turns on its ball and steps forward and back along a low arc.
// Model space has +Z up, forward is -Y, and left is +X.
#pragma once
#include "pr/animation/forward.h"
#include "pr/animation/rig.h"
#include "pr/animation/targets.h"
#include "pr/animation/activity.h"

namespace pr::animation
{
	// A smooth step from 0 at 'lo' to 1 at 'hi', clamped outside that range
	inline float Ease(float lo, float hi, float x)
	{
		// Cubic with zero slope at both ends
		auto t = Clamp((x - lo) / (hi - lo), 0.0f, 1.0f);
		return t * t * (3.0f - 2.0f * t);
	}

	// +1 for the left side of the body (model +X), -1 for the right side, and 0 for the middle
	inline float SideSign(ESide side)
	{
		switch (side)
		{
			case ESide::Centre: { return 0.0f; }
			case ESide::Left: { return +1.0f; }
			case ESide::Right: { return -1.0f; }
			default: throw std::runtime_error("Unknown body side");
		}
	}

	// Return the index of the arm chain of 'rig' on 'side', or -1 if there is none
	inline int ArmChain(Rig const& rig, ESide side)
	{
		// Rigs have few chains
		for (int c = 0, cend = static_cast<int>(rig.chains.size()); c != cend; ++c)
		{
			if (rig.chains[c].role == EChainRole::Arm && rig.chains[c].side == side)
				return c;
		}
		return -1;
	}

	// The first limb in 'limbs' whose chain is a leg on 'side', or null if there is none. Rigs may have several legs on a side.
	inline ActivityLimb const* LegOn(Rig const& rig, std::span<ActivityLimb const> limbs, ESide side)
	{
		// Limb order decides between legs on the same side
		for (auto const& limb : limbs)
		{
			auto const& chain = rig.chains[limb.chain];
			if (chain.role == EChainRole::Leg && chain.side == side)
				return &limb;
		}
		return nullptr;
	}

	// How far out a swing is, in [0,1], at progress 'u' in [0,1] through it. The tool point speeds up all the way out, so it is at its fastest
	// when it reaches full extent at 'out', then it comes back more slowly, reaching rest at 'back'. Requires 0 < out < back <= 1.
	inline float SwingExtend(float u, float out, float back)
	{
		// Quadratic acceleration out, smooth return
		return u < out ? Sqr(u / out) : 1.0f - Ease(out, back, u);
	}

	// The curved path of a tool point from its rest position, through a control point, to its full extent. Positions are offsets from the
	// limb's root joint in units of the limb's reach. Directions and poles turn from their rest values to their values at full extent.
	struct SwingArc
	{
		v4 rest;        // The tool point's rest position
		v4 ctrl;        // The control point that bends the path
		v4 end;         // The tool point's position at full extent
		v4 rest_dir;    // The direction that the tool points at rest (unit, w = 0)
		v4 end_dir;     // The direction that the tool points at full extent (unit, w = 0)
		v4 rest_pole;   // The direction that the limb's middle joint bends towards at rest (unit, w = 0)
		v4 end_pole;    // The direction that the limb's middle joint bends towards at full extent (unit, w = 0)

		// The tool point's offset when the swing is 's' in [0,1] of the way out
		v4 Point(float s) const
		{
			// A quadratic curve from the rest position to the full extent
			return rest * ((1 - s) * (1 - s)) + ctrl * (2 * s * (1 - s)) + end * (s * s);
		}

		// The direction that the tool points when the swing is 's' of the way out
		v4 Direction(float s) const
		{
			// Turn steadily from the rest direction
			return Normalise(Lerp(rest_dir, end_dir, s));
		}

		// The direction that the limb's middle joint bends towards when the swing is 's' of the way out
		v4 Pole(float s) const
		{
			// Turn steadily from the rest pole
			return Normalise(Lerp(rest_pole, end_pole, s));
		}
	};

	// How the torso moves from its guide pose to put the body's weight behind a motion
	struct TorsoDrive
	{
		float yaw = 0;          // The chest's turn (radians) about model +Z. The pelvis turns a share of this (see 'ApplyTorsoDrive').
		float lean = 0;         // The chest's forward lean (radians) about the pelvis
		float tilt = 0;         // The chest's sideways bend (radians) about model +Y, towards +X for positive angles
		v4 hips = v4::Zero();   // The pelvis's model-space shift
		v4 chest = v4::Zero();  // The chest's model-space shift, from the lean, the bend and the hips' shift (see 'DriveChest')
	};

	// Set 'drive.chest' from its angles and hip shift, for a spine from the pelvis at 'pelvis' to the chest at 'chest' (model space). The chest
	// swings about the hips, so it moves with the hips' shift plus the arc of the lean and the bend.
	inline void DriveChest(TorsoDrive& drive, v4 const& pelvis, v4 const& chest)
	{
		// Leaning about model +X by a positive angle moves the chest forward, and bending about model +Y by a positive angle moves it towards +X
		auto span = chest - pelvis;
		drive.chest = drive.hips + Rotate(quat(v4::YAxis(), drive.tilt) * quat(v4::XAxis(), drive.lean), span) - span;
	}

	// Move the spine target 'spine' by 'drive'. The chest and pelvis turn about their own positions, so the spine bends between them. The pelvis
	// turns 'pelvis_share' of the chest's turn.
	inline void ApplyTorsoDrive(Target& spine, TorsoDrive const& drive, float pelvis_share)
	{
		// Turning about model +Z by a positive angle moves the right shoulder (-X) forward (-Y)
		spine.frame = xform(spine.frame.pos + drive.chest, Normalise(quat(v4::ZAxis(), drive.yaw) * quat(v4::YAxis(), drive.tilt) * quat(v4::XAxis(), drive.lean) * spine.frame.rot));
		spine.base = xform(spine.base.pos + drive.hips, Normalise(quat(v4::ZAxis(), pelvis_share * drive.yaw) * spine.base.rot));
	}

	// Move the foot target 'foot' of the leg chain 'chain': turn it by 'turn' radians about model +Z on its ball (the tip of its end bone), turning
	// its heel out, then move it by 'offset' (model space, w = 0). Callers ease both from zero so that the foot returns to its planted frame.
	inline void PivotStep(Rig const& rig, int chain, Target& foot, float turn, v4 const& offset)
	{
		// The ball is the foot bone's tip
		auto length = rig.bones[rig.chains[chain].bones.back()].length;
		auto ball = foot.frame.pos + Rotate(foot.frame.rot, v4::YAxis()) * length;
		auto rot = quat(v4::ZAxis(), turn);
		foot.frame = xform(ball + Rotate(rot, foot.frame.pos - ball) + offset, Normalise(rot * foot.frame.rot));
	}

	// The model-space position of the root joint of the limb chain 'chain', carried by a spine whose chest is at 'chest' instead of where the guide
	// pose has it. Without a spine the body does not move relative to the guide.
	inline v4 LimbRoot(ActivityInput const& input, int chain, xform const& chest)
	{
		// The root keeps its place relative to the chest
		auto root = input.guide_model[input.rig.chains[chain].bones.front()];
		if (input.spine < 0)
			return root.pos;

		return (chest * Invert(input.guide[input.spine].frame) * root).pos;
	}
}
