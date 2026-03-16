#include "src/forward.h"
#include "src/diagnostics/diagnostics.h"

namespace physics_sandbox
{
	// Diagnostic logging implementation
	#ifdef PR_PHYSICS_DIAGNOSTICS

	static FILE* s_log_file = nullptr;
	void DbgLog(char const* fmt, ...)
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

	void DbgLog(char const*, ...) {}

	#endif

	// Capture a snapshot of a rigid body's current state
	BodySnapshot BodySnapshot::Capture(pr::physics::RigidBody const& rb)
	{
		auto vel = rb.VelocityWS();
		auto snap = BodySnapshot{};
		snap.com_pos = rb.CentreOfMassWS();
		snap.lin_vel = vel.lin;
		snap.ang_vel = vel.ang;
		snap.momentum = rb.MomentumWS();
		snap.mass = rb.Mass();
		snap.ke = rb.KineticEnergy();
		return snap;
	}

	// Log a snapshot's values for diagnostic output
	void BodySnapshot::Log(char const* label) const
	{
		DbgLog("  %s: pos=(%.4f, %.4f, %.4f) lvel=(%.4f, %.4f, %.4f) avel=(%.4f, %.4f, %.4f) mass=%.1f KE=%.6f\n",
			label,
			com_pos.x, com_pos.y, com_pos.z,
			lin_vel.x, lin_vel.y, lin_vel.z,
			ang_vel.x, ang_vel.y, ang_vel.z,
			mass, ke);
	}
}
