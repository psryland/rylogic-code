#pragma once
#include "src/forward.h"

namespace physics_sandbox
{
	// Diagnostic output helper — writes to both OutputDebugString and a log file.
	// Useful for capturing physics state during interactive testing without a debugger.
	// Enable by defining PR_PHYSICS_DIAGNOSTICS before including this header.
	#ifdef PR_PHYSICS_DIAGNOSTICS

	static FILE* s_log_file = nullptr;
	static void DbgLog(char const* fmt, ...)
	{
		char buf[1024];
		va_list args;
		va_start(args, fmt);
		vsnprintf(buf, sizeof(buf), fmt, args);
		va_end(args);
		OutputDebugStringA(buf);

		if (!s_log_file)
			s_log_file = fopen("dump\\physics_diag.log", "w");
		if (s_log_file)
		{
			fputs(buf, s_log_file);
			fflush(s_log_file);
		}
	}

	#else

	// No-op when diagnostics are disabled
	static void DbgLog(char const*, ...) {}

	#endif

	// Snapshot of a rigid body's state at a moment in time.
	// Used for before/after comparisons across collision events.
	struct BodySnapshot
	{
		pr::v4 pos;
		pr::v4 lin_vel;
		pr::v4 ang_vel;
		pr::v8force momentum;
		float mass;
		float ke;

		static BodySnapshot Capture(pr::physics::RigidBody const& rb)
		{
			auto vel = rb.VelocityWS();
			auto snap = BodySnapshot{};
			snap.pos = rb.O2W().pos;
			snap.lin_vel = vel.lin;
			snap.ang_vel = vel.ang;
			snap.momentum = rb.MomentumWS();
			snap.mass = rb.Mass();
			snap.ke = rb.KineticEnergy();
			return snap;
		}

		void Log(char const* label) const
		{
			DbgLog("  %s: pos=(%.4f, %.4f, %.4f) lvel=(%.4f, %.4f, %.4f) avel=(%.4f, %.4f, %.4f) mass=%.1f KE=%.6f\n",
				label,
				pos.x, pos.y, pos.z,
				lin_vel.x, lin_vel.y, lin_vel.z,
				ang_vel.x, ang_vel.y, ang_vel.z,
				mass, ke);
		}
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
