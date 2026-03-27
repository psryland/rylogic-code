//************************************
// Physics Engine
//  Copyright (c) Rylogic Ltd 2016
//************************************
// Force-link declarations for physics unit test files.
// When unit tests are compiled into a static library, the linker drops .obj files
// that have no externally-referenced symbols. These force-link functions provide
// an external reference that ensures the test registration code runs.
//
// Include this header in the executable that runs the tests, and call
// ForceLink_PhysicsTests() before RunAllTests().
#pragma once

#if PR_UNITTESTS
namespace pr::physics::tests
{
	void ForceLink_CollisionPairs();
	void ForceLink_CollisionResolution();
	void ForceLink_GpuCollide();
	void ForceLink_GpuCollision();
	void ForceLink_GpuCompare();
	void ForceLink_Impulse();
	void ForceLink_Inertia();
	void ForceLink_Integrator();
	void ForceLink_RigidBody();
	void ForceLink_ShapeBuilder();

	inline void ForceLink_PhysicsTests()
	{
		ForceLink_CollisionPairs();
		ForceLink_CollisionResolution();
		ForceLink_GpuCollide();
		ForceLink_GpuCollision();
		ForceLink_GpuCompare();
		ForceLink_Impulse();
		ForceLink_Inertia();
		ForceLink_Integrator();
		ForceLink_RigidBody();
		ForceLink_ShapeBuilder();
	}
}
#endif
