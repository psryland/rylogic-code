//*********************************************
// Physics engine
//  Copyright (c) Rylogic Ltd 2006
//*********************************************

#include "physics/utility/stdafx.h"
#include "pr/physics/rigidbody/integrate.h"
#include "pr/physics/rigidbody/rigidbody.h"
#include "pr/physics/utility/globalfunctions.h"
#include "pr/physics/engine/igravity.h"
#include "physics/utility/debug.h"

using namespace pr;
using namespace pr::ph;

void EvolveAngularOrder1(Rigidbody& rb, float elapsed_seconds);
void EvolveAngularOrder2(Rigidbody& rb, float elapsed_seconds);
void EvolveAngularOrder5(Rigidbody& rb, float elapsed_seconds);

// Set the micro velocity threshold for 'object'
inline float CalcMicroMomentum(v4 gravity, float mass, float step_time)
{
	// Objects without gravity can't go to sleep.
	// Micro momentum is the momentum after one step under gravity alone
	// mv = mat = mgt. k*g*t (k=constant, g=gravity, t=time_step)
	static float multiple_of_accel_under_gravity = 8.0f;
	return Sqr(multiple_of_accel_under_gravity * step_time * mass) * LengthSq(gravity);
}

// Evolve a rigid body forward in time. This applies the accumulated external
// forces and torques for 'elapsed_seconds' then resets these members.
void pr::ph::Evolve(Rigidbody& rb, float elapsed_seconds)
{
	PR_EXPAND(PR_LOG_RB, Log(rb, "Evolve");)
	PR_ASSERT(PR_DBG_PHYSICS, IsFinite(rb.ObjectToWorld(), ph::OverflowValue), "");
	PR_ASSERT(PR_DBG_PHYSICS, !rb.m_sleeping, "");
	
	// Get the acceleration due to gravity at the current position of the rigidbody
	v4 gravity = rb.Gravity();
	
	// Linear ***************
	// A = F/M
	v4 acceleration = rb.m_inv_mass * rb.m_force + gravity;
	// V = MV/M
	v4 velocity = rb.Velocity();
	// S = So + VoT + 0.5AT^2
	rb.m_object_to_world.pos += (velocity + (0.5f * elapsed_seconds) * acceleration) * elapsed_seconds;
	
	// V = Vo + AT, MV = MVo + FT
	rb.m_lin_momentum += (rb.m_force + rb.m_mass * gravity) * elapsed_seconds;
	
	// Angular ***************
#define PR_PH_TEST_ANGULAR 1
	PR_EXPAND(PR_PH_TEST_ANGULAR, float old_ak = rb.AngularKineticEnergy());
	PR_EXPAND(PR_PH_TEST_ANGULAR, int stepper_used = 0; float thres = 0.0f);
	PR_EXPAND(PR_PH_TEST_ANGULAR, static Rigidbody* break_if = 0);
	PR_ASSERT(PR_PH_TEST_ANGULAR, break_if != &rb, "");
	const float Runge2Threshold = 1.8e-1f;
	const float Runge5Threshold = 2.3e-3f;
	const float VelCapThreshold = 1.0e-6f;//1.0e-8f;
	float ang_vel = Length(rb.AngVelocity());
	for (;;)
	{
		// Decide which integrator to use based on the step size and the angular velocity
		float h = elapsed_seconds; // Keep powers of time in 'h'
		
		// Euler step
		if (h * ang_vel < Runge2Threshold)
		{
			PR_EXPAND(PR_PH_TEST_ANGULAR, stepper_used = 1; thres = h * ang_vel;)
			EvolveAngularOrder1(rb, elapsed_seconds);
			break;
		}
		
		// Midpoint
		h *= elapsed_seconds; // time^2
		if (h * ang_vel < Runge5Threshold)
		{
			PR_EXPAND(PR_PH_TEST_ANGULAR, stepper_used = 2; thres = h * ang_vel;)
			EvolveAngularOrder2(rb, elapsed_seconds);
			break;
		}
		
		// Cap the angular velocity if necessary
		h *= h * elapsed_seconds;
		if (h * ang_vel > VelCapThreshold)
		{
			PR_INFO(PR_PH_TEST_ANGULAR, Fmt("Angular velocity capped: %e : %e", h * ang_vel, VelCapThreshold).c_str());
			rb.m_ang_momentum *= VelCapThreshold / (h * ang_vel);
			PR_EXPAND(PR_PH_TEST_ANGULAR, ang_vel = Length(rb.AngVelocity());)
		}
		
		// Runge 5
		PR_EXPAND(PR_PH_TEST_ANGULAR, stepper_used = 5; thres = h * ang_vel;)
		EvolveAngularOrder5(rb, elapsed_seconds);
		break;
	}
	// May not need to do this every step...
	rb.m_object_to_world = Orthonorm(rb.m_object_to_world);
	
#if PR_PH_TEST_ANGULAR == 1
	PR_EXPAND(PR_PH_TEST_ANGULAR, float new_ak = rb.AngularKineticEnergy();)
	float error = Abs(new_ak - old_ak) / old_ak;    // Error as a percentage of the initial angular kinetic energy
	if (error > 0.05f)
	{   PR_INFO(PR_PH_TEST_ANGULAR, Fmt("Angular integration error: %f.  Stepper %d used, Thres %e", error, stepper_used, thres).c_str()); }
#endif//PR_PH_TEST_ANGULAR == 1
#undef PR_PH_TEST_ANGULAR
	
	// Rate of change of angular momentum is torque, so change is torque * t
	rb.m_ang_momentum += rb.m_torque * elapsed_seconds;
	
	// Prepare this object for the next step
	rb.m_force                  = pr::v4::Zero();
	rb.m_torque                 = pr::v4::Zero();
	rb.m_ws_bbox                = rb.ObjectToWorld() * rb.BBoxOS();
	rb.m_ws_inv_inertia_tensor  = InvInertiaTensorWS(rb.Orientation(), rb.m_os_inv_inertia_tensor); //Iw = (o2w * Io * w2o)^-1  =  w2o^-1 * Io^-1 * o2w^-1  =  o2w * Io^-1 * w2o
	rb.m_micro_mom_sq           = CalcMicroMomentum(gravity, rb.m_mass, elapsed_seconds);
	
	// Update the broad phase now that the rigid body has moved
	rb.m_bp_entity.Update();
}

// Use Euler integration to advance the angular stuff.
void EvolveAngularOrder1(Rigidbody& rb, float elapsed_seconds)
{
	// Rotate the object_to_world by the change in orientation for this time step
	// 'dorientation/dt = cross_product_matrix(ang_velocity) * orientation * elapsed_seconds'
	rb.m_object_to_world.rot += CPM(rb.AngVelocity() * elapsed_seconds) * rb.Orientation();
}

//*****
// Use the midpoint algorithm to advance the angular stuff.
void EvolveAngularOrder2(Rigidbody& rb, float elapsed_seconds)
{
	// Calculate mid-point values
	float half_dt = elapsed_seconds * 0.5f;
	m3x4 mid_orientation            = rb.Orientation() + CPM(rb.AngVelocity() * half_dt) * rb.Orientation();
	m3x4 mid_ws_inv_inertia_tensor  = mid_orientation * rb.m_os_inv_inertia_tensor * Transpose(mid_orientation);
	v4   mid_ang_momentum           = rb.m_ang_momentum + rb.m_torque * half_dt;
	v4   mid_ang_velocity           = mid_ws_inv_inertia_tensor * rb.m_inv_mass * mid_ang_momentum;
	
	// Perform step using mid-point angular velocity
	rb.m_object_to_world.rot += CPM(mid_ang_velocity * elapsed_seconds) * rb.Orientation();
}

//*****
// Use the Runge-Kutta 5th order algorithm to advance the angular stuff.
void EvolveAngularOrder5(Rigidbody& rb, float elapsed_seconds)
{
	// Cash-Karp constants for embedded Runge-Kutta.
	// These numbers come from p717 of numerical recipes.
	const float b00 = 0.2f,             // 1/5
				b10 = 0.075f,           // 3/40
				b11 = 0.225f,           // 9/40
				b20 = 0.3f,             // 3/10
				b21 = -0.9f,            // -9/10
				b22 = 1.2f,             // 6/5
				b30 = -0.2037037037f,   // -11/54
				b31 = 2.5f,             // 5/2
				b32 = -2.59259259259f,  // -70/27
				b33 = 1.29629629629f,   // 35/27
				b40 = 0.029495804398f,  // 1631/55296
				b41 = 0.341796875f,     // 175/512
				b42 = 0.041594328703f,  // 575/13824
				b43 = 0.40034541377f,   // 44275/110592
				b44 = 0.061767578125f,  // 253/4096
				c0  = 0.097883597883f,  // 37/378
				c2  = 0.40257648953f,   // 250/621
				c3  = 0.21043771043f,   // 125/594
				c5  = 0.28910220214f;   // 512/1771
				
	float   step0, step1, step2, step3, step4, step5;
	
	// Temporaries used in each step
	m3x4    step_orientation;
	m3x4    step_ws_inv_inertia_tensor;
	v4      step_ang_momentum;
	v4      step_ang_velocity;
	
	// Get the orientation, angular velocity, and derivative of orientation at t0
	v4      ang_velocity_0 = rb.AngVelocity();
	m3x4    orientation_0  = rb.Orientation();
	m3x4    dorientation_0 = CPM(ang_velocity_0) * orientation_0;
	
	// Step 0
	step0                       = elapsed_seconds * b00;
	step_orientation            = orientation_0 + dorientation_0*step0;
	step_ang_momentum           = rb.m_ang_momentum + rb.m_torque*step0;
	step_ws_inv_inertia_tensor  = rb.Orientation() * rb.m_os_inv_inertia_tensor * Transpose(rb.Orientation());
	step_ang_velocity           = step_ws_inv_inertia_tensor * rb.m_inv_mass * step_ang_momentum;
	m3x4 dorientation_1         = CPM(step_ang_velocity) * step_orientation;
	
	// Step 1
	step0                       = elapsed_seconds * b10;
	step1                       = elapsed_seconds * b11;
	step_orientation            = orientation_0 + dorientation_0*step0 + dorientation_1*step1;
	step_ang_momentum           = rb.m_ang_momentum + rb.m_torque*(step0 + step1);
	step_ws_inv_inertia_tensor  = rb.Orientation() * rb.m_os_inv_inertia_tensor * Transpose(rb.Orientation());
	step_ang_velocity           = step_ws_inv_inertia_tensor * rb.m_inv_mass * step_ang_momentum;
	m3x4 dorientation_2         = CPM(step_ang_velocity) * step_orientation;
	
	// Step 2
	step0                       = elapsed_seconds * b20;
	step1                       = elapsed_seconds * b21;
	step2                       = elapsed_seconds * b22;
	step_orientation            = orientation_0 + dorientation_0*step0 + dorientation_1*step1 + dorientation_2*step2;
	step_ang_momentum           = rb.m_ang_momentum + rb.m_torque*(step0 + step1 + step2);
	step_ws_inv_inertia_tensor  = rb.Orientation() * rb.m_os_inv_inertia_tensor * Transpose(rb.Orientation());
	step_ang_velocity           = step_ws_inv_inertia_tensor * rb.m_inv_mass * step_ang_momentum;
	m3x4 dorientation_3         = CPM(step_ang_velocity) * step_orientation;
	
	// Step 3
	step0                       = elapsed_seconds * b30;
	step1                       = elapsed_seconds * b31;
	step2                       = elapsed_seconds * b32;
	step3                       = elapsed_seconds * b33;
	step_orientation            = orientation_0 + dorientation_0*step0 + dorientation_1*step1 + dorientation_2*step2 + dorientation_3*step3;
	step_ang_momentum           = rb.m_ang_momentum + rb.m_torque*(step0 + step1 + step2 + step3);
	step_ws_inv_inertia_tensor  = rb.Orientation() * rb.m_os_inv_inertia_tensor * Transpose(rb.Orientation());
	step_ang_velocity           = step_ws_inv_inertia_tensor * rb.m_inv_mass * step_ang_momentum;
	m3x4 dorientation_4         = CPM(step_ang_velocity) * step_orientation;
	
	// Step 4
	step0                       = elapsed_seconds * b40;
	step1                       = elapsed_seconds * b41;
	step2                       = elapsed_seconds * b42;
	step3                       = elapsed_seconds * b43;
	step4                       = elapsed_seconds * b44;
	step_orientation            = orientation_0 + dorientation_0*step0 + dorientation_1*step1 + dorientation_2*step2 + dorientation_3*step3 + dorientation_4*step4;
	step_ang_momentum           = rb.m_ang_momentum + rb.m_torque*(step0 + step1 + step2 + step3 + step4);
	step_ws_inv_inertia_tensor  = rb.Orientation() * rb.m_os_inv_inertia_tensor * Transpose(rb.Orientation());
	step_ang_velocity           = step_ws_inv_inertia_tensor * rb.m_inv_mass * step_ang_momentum;
	m3x4 dorientation_5         = CPM(step_ang_velocity) * step_orientation;
	
	// Step 5
	// ori(t1) = ori(t0) + c0*elapsed_seconds*dori_0 + c1*elapsed_seconds*dori_1 + c2*elapsed_seconds*dori_2
	//                   + c3*elapsed_seconds*dori_3 + c4*elapsed_seconds*dori_4 + c5*elapsed_seconds*dori_5
	//  note: c1 and c4 = 0
	step0 = elapsed_seconds * c0;
	step2 = elapsed_seconds * c2;
	step3 = elapsed_seconds * c3;
	step5 = elapsed_seconds * c5;
	rb.m_object_to_world.rot += dorientation_0*step0 + dorientation_2*step2 + dorientation_3*step3 + dorientation_5*step5;
}
