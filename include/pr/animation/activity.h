//*****************************************************************************
// Procedural animation - activities
//  Copyright (c) Rylogic Ltd 2026
//*****************************************************************************
// An activity is one procedural motion that a character performs in place of, or on top of, its gait, such as throwing punches or chopping.
// Each activity has a type (a class that implements 'IActivity'), parameters for that type read from the '.gait' file, and a name.
//  - An activity is immutable once read. 'IActivity::Evaluate' is a pure function of the activity time and its inputs, so a character that
//    starts animating part way through an activity shows the same pose as one that played it from the start.
//  - An activity controls a fixed set of chains (see 'IActivity::Chains'). It writes goals only for those chains.
//  - The runner (see 'Locomotion') owns the activity's start and stop times and its fades. Types never fade themselves.
//  - Each activity publishes a timeline (its length, whether it loops, and named marks such as "contact") that callers can read without
//    running the animator, so that gameplay events happen at the same moments as the motion.
//  - Types are listed in the registry (see activities.h), which reads their parameters and describes them to tools.
#pragma once
#include "pr/animation/forward.h"
#include "pr/animation/rig.h"
#include "pr/animation/targets.h"

namespace pr::animation
{
	// What happens to the character's root and gait while an activity plays
	enum class ERootMode
	{
		// The feet stay where they are and the body does not travel. The stance may change (see 'IActivity::StanceShift').
		Planted,

		// The gait keeps moving the body, limited to a speed, with a body lean. Not yet supported by the runner.
		Travel,

		// The gait is unchanged. Only the activity's chains change.
		Free,

		// The root follows a moving frame, such as a raft deck. Not yet supported by the runner.
		Mounted,
	};

	// The names of 'ERootMode' values, in enum order
	inline constexpr std::array<std::string_view, 4> RootModeNames = { "planted", "travel", "free", "mounted" };

	// A named moment in an activity's timeline
	struct ActivityMark
	{
		std::string name;   // The kind of moment, such as "contact". Several marks may share a name.
		float time = 0.0f;  // Seconds from the start of the activity, in [0, length]
	};

	// How long an activity lasts, and the named moments within it
	struct ActivityTimeline
	{
		float length = 0.0f;              // > 0. The activity's length in seconds. A looping activity repeats with this period.
		bool loop = false;                // True if the activity repeats until it is stopped. Otherwise it holds its last pose.
		std::vector<ActivityMark> marks;  // Ordered by time. In a looping activity the marks repeat every 'length' seconds.
	};

	// A thing to hit: a body whose centre is 'point' (world space, w = 1) and whose surface is 'radius' (>= 0) from that centre
	struct Strike
	{
		v4 point = v4::Origin();
		float radius = 0.0f;
	};

	// A tool held in one hand, such as an axe, described in the frame of its main grip. A hand holds the tool by placing its hold frame on the grip
	// frame (see 'ActivityLimb::hold'), and its digits close around the handle. The grip frame's origin is on the handle's surface where the palm
	// touches it, its +Z runs along the handle towards the head, and its +X points into the handle.
	struct Tool
	{
		xform head = xform::Identity();  // The striking head in grip space. Its origin is the point that strikes, -Y is the direction the striking face points, and +Z runs along the handle.
		float wrap = 0.0f;               // The handle's radius (metres, > 0) at the grip, which the digits close around
	};

	// The things in the world that an activity acts on. Each type documents which members it reads, and what it does when one is empty.
	struct ActivityTargets
	{
		std::optional<Strike> strike;  // What a striking motion hits
		std::optional<Tool> tool;      // What the hand holds. Without a tool, the hand strikes with its hold frame as a bare fist.
	};

	// What an activity can know about one of the character's limbs
	struct ActivityLimb
	{
		int chain = -1;                      // The limb's chain index in the rig
		float reach = 0.0f;                  // The limb's length from its root joint to its contact
		quat stance_rot = Identity<quat>();  // The end bone's model-space orientation in the current stance
		v4 sole_up = v4::YAxis();            // The unit normal of the sole or palm in the end bone's space, pointing away from the ground
		std::optional<xform> hold;           // The hand's hold frame in the end bone's space, from the first rig attachment on the end bone. Its origin is on the palm's surface, +X points out of the palm, and +Z runs along a held handle towards the tool's head. Holding a tool puts this frame on the tool's grip frame (see 'Tool').
	};

	// The inputs to 'IActivity::Evaluate'. All frames are in model space unless stated.
	struct ActivityInput
	{
		Rig const& rig;
		std::span<ActivityLimb const> limbs;  // The limbs that walk in any family
		std::span<Target const> guide;        // One target per chain that reproduces the guide pose (the stance plus the gait's body motion)
		std::span<xform const> guide_model;   // The guide pose's model-space bone frames
		int spine;                            // The spine chain index, or -1 if the rig has none
		xform world_to_model;                 // Converts world-space targets into model space
		ActivityTargets const& targets;       // The caller's targets for this activity
	};

	// The outputs of 'IActivity::Evaluate'
	struct ActivityOutput
	{
		std::span<Target> targets;  // One target per chain. On entry, the goals from the gait. The activity overwrites the goals of its own chains only.
		std::span<float> curl;      // One angle (radians) per chain, zero on entry. For an arm, how far each of its digits' bones curls towards the palm.
		std::span<float> wrap;      // One radius (metres) per chain, zero on entry. For an arm with a hold frame, the radius of a bar lying against the palm at the hold frame, along its +Z, that the digits close around. A non-zero wrap takes precedence over 'curl'.
	};

	// The interface that each activity type implements. See the top of this file.
	struct IActivity
	{
		virtual ~IActivity() = default;

		// The type's name in the registry, such as "punches"
		virtual std::string_view Type() const = 0;

		// What happens to the character's root while the activity plays
		virtual ERootMode RootMode() const = 0;

		// The activity's length, looping and marks
		virtual ActivityTimeline const& Timeline() const = 0;

		// The chain indices of 'rig' that the activity controls, in increasing order. The rig must be valid for the activity (see 'Validate').
		virtual std::vector<int> Chains(Rig const& rig) const = 0;

		// The model-space offset (w = 0) of the stance contact of the leg chain 'chain' while the activity plays, for legs of mean reach 'leg_reach'.
		// Only used for Planted activities.
		virtual v4 StanceShift(Rig const& rig, int chain, float leg_reach) const = 0;

		// Add to 'errors' a description of each way in which the activity does not fit 'rig'. Each description starts with 'context'.
		virtual void Validate(Rig const& rig, std::string_view context, std::vector<std::string>& errors) const = 0;

		// The activity's parameters, in the form that the registry reads (see activities.h)
		virtual json::Object WriteParams() const = 0;

		// Write the activity's goals at 'time' seconds into it into 'output'. 'time' is in [0, length] (the runner wraps looping activities).
		// The result depends only on 'time' and the inputs.
		virtual void Evaluate(float time, ActivityInput const& input, ActivityOutput const& output) const = 0;
	};
}
