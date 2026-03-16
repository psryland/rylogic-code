#pragma once
#include "src/forward.h"

namespace physics_sandbox
{
	// Diagnostic output helper — writes to both OutputDebugString and a log file.
	// Useful for capturing physics state during interactive testing without a debugger.
	// Enable by defining PR_PHYSICS_DIAGNOSTICS before including this header.
	void DbgLog(char const* fmt, ...);

	// Snapshot of a rigid body's state at a moment in time.
	// Used for before/after comparisons across collision events.
	struct BodySnapshot
	{
		pr::v4 com_pos;  // World-space centre of mass position
		pr::v4 lin_vel;
		pr::v4 ang_vel;
		pr::v8force momentum;
		float mass;
		float ke;

		static BodySnapshot Capture(pr::physics::RigidBody const& rb);
		void Log(char const* label) const;
	};

	// Diagnostic data captured during a collision event.
	// Stores pre/post impulse snapshots and contact geometry.
	struct CollisionDiag
	{
		BodySnapshot before[2]; // State just before impulse (post-Evolve, pre-impulse)
		BodySnapshot after[2];  // State just after impulse
		pr::v4 contact_point_ws;
		pr::v4 contact_normal_ws;
		float depth;
		bool occurred;
		int count;

		CollisionDiag()
			: before{}
			, after{}
			, contact_point_ws{}
			, contact_normal_ws{}
			, depth{}
			, occurred{}
			, count{}
		{
		}

		void Reset()
		{
			occurred = false;
			count = 0;
		}
	};
}
