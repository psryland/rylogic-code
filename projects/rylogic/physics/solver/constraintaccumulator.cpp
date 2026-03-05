//*********************************************
// Physics engine
//  Copyright (c) Rylogic Ltd 2006
//*********************************************

#include "physics/utility/stdafx.h"
#include "pr/physics/solver/constraintaccumulator.h"
#include "pr/physics/collision/contactmanifold.h"
#include "pr/physics/collision/contact.h"
#include "pr/physics/rigidbody/rigidbody.h"
#include "pr/physics/engine/engine.h"
#include "pr/physics/material/imaterial.h"

using namespace pr;
using namespace pr::ph;

// The amount of penetration we'll allow without penetration correction
float g_penetration_tolerance = 0.01f;

// The maximum penetration we'll consider objects to have for penetration correction.
// This effectively limits the maximum velocities that objects can separate at.
float g_max_penetration = 0.05f;

ConstraintAccumulator::ConstraintAccumulator(Engine& engine, pr::AllocFunction allocate, pr::DeallocFunction deallocate)
:m_Allocate(allocate)
,m_Deallocate(deallocate)
,m_engine(engine)
,m_map()
,m_num_sets(0)
,m_buffer(0)
,m_buffer_end(0)
,m_buffer_ptr(0)
,m_pairs(0)
,m_num_pairs(0)
,m_max_pairs(0)
,m_step_size(1.0f)
{
	m_map[NoConstraintSet] = NoConstraintSet;
}

ConstraintAccumulator::~ConstraintAccumulator()
{
	SetBufferSize(0);
}

// Set the size of the constraint buffer
void ConstraintAccumulator::SetBufferSize(std::size_t constraint_buffer_size_in_bytes)
{
	PR_ASSERT(PR_DBG_PHYSICS, m_buffer_ptr == m_buffer, "The constraint buffer should be empty when resizing it");
	if( m_buffer )
	{
		m_num_sets = 0;
		m_Deallocate(m_buffer);	m_buffer = 0;
		m_Deallocate(m_pairs);	m_pairs = 0;
		m_buffer_end = m_buffer;
		m_buffer_ptr = m_buffer;
		m_num_pairs = 0;
		m_max_pairs = 0;
	}
	if( constraint_buffer_size_in_bytes > 0 )
	{
		m_num_sets = 0;
		uint32_t estimated_num_pairs = static_cast<uint32_t>(1 + constraint_buffer_size_in_bytes / (sizeof(ConstraintBlock) + sizeof(Constraint)));
		m_buffer	= static_cast<uint8_t*>			(m_Allocate(constraint_buffer_size_in_bytes, std::alignment_of<ConstraintBlock>::value));
		m_pairs		= static_cast<ConstraintBlock**>(m_Allocate(estimated_num_pairs * sizeof(ConstraintBlock*), std::alignment_of<ConstraintBlock*>::value));
		m_buffer_end = m_buffer + constraint_buffer_size_in_bytes;
		m_buffer_ptr = m_buffer;
		m_num_pairs = 0;
		m_max_pairs = estimated_num_pairs;
	}
}

// Initialise the constraint solver for the frame
void ConstraintAccumulator::BeginFrame(float elapsed_seconds)
{
	m_step_size = elapsed_seconds;
}

// Allocates a constraint block from the buffer to be filled in. Handles constraint sets.
ConstraintBlock& ConstraintAccumulator::AllocateConstraints(Rigidbody& rbA, Rigidbody& rbB, uint32_t num_constraints)
{
	uint32_t bytes_needed = sizeof(ConstraintBlock) + num_constraints * sizeof(Constraint);

	// If the constraint buffer is full, solve now.
	// We'll end up with overlap errors because of this but it's better than
	// ignoring constraints completely. Really the client should allocate a
	// bigger constraint buffer.
	if( m_buffer_ptr + bytes_needed > m_buffer_end || m_num_pairs == m_max_pairs )
		Solve();

	// Allocate a constraint block and constraints from the end of the buffer.
	ConstraintBlock& pair = *reinterpret_cast<ConstraintBlock*>(m_buffer_ptr);
	m_buffer_ptr += bytes_needed;

	// Record a pointer to this pair
	m_pairs[m_num_pairs++] = &pair;

	// See if either of the rigid bodies already belong to a constraint set
	uint8_t set_id = pr::Min(m_map[rbA.m_constraint_set], m_map[rbB.m_constraint_set]);
	if( set_id == NoConstraintSet )
	{
		// If there are too many constraint sets then solve the sets we have
		if( m_num_sets == ConstraintSetMappingSize )
			Solve();

		// Allocate a new constraint set
		set_id = m_num_sets++;
		m_map[set_id] = set_id;
	}

	// If the objects belong to different constraint sets, then join the sets
	if( rbA.m_constraint_set != NoConstraintSet ) m_map[rbA.m_constraint_set] = set_id;
	if( rbB.m_constraint_set != NoConstraintSet ) m_map[rbB.m_constraint_set] = set_id;
	rbA .m_constraint_set = set_id;
	rbB .m_constraint_set = set_id;
	pair.m_constraint_set = set_id;

	// Initialise the pair
	pair.m_objA = &rbA;
	pair.m_objB = &rbB;
	pair.m_num_constraints = s_cast<uint16_t>(num_constraints);
	//pair.m_constraint_set = NoConstraintSet;
	pair.pad = 0;
	PR_EXPAND(PR_DBG_PHYSICS, pair.m_constraints = &pair[0]);
	return pair;
}

// Sets the material properties to use for the collision
void ConstraintAccumulator::SetMaterialProperties(Constraint& cons, uint32_t mat_idA, uint32_t mat_idB) const
{
	ph::Material const& materialA = ph::GetMaterial(mat_idA);
	ph::Material const& materialB = ph::GetMaterial(mat_idB);
	cons.m_elasticity       = pr::Min(materialA.m_elasticity		,materialB.m_elasticity);
	cons.m_static_friction  = pr::Min(materialA.m_static_friction	,materialB.m_static_friction);
	cons.m_dynamic_friction = pr::Min(materialA.m_dynamic_friction	,materialB.m_dynamic_friction);

	// Adjust the friction values so that 0->1 corresponds to 0->inf
	cons.m_static_friction	= cons.m_static_friction  / (1.000001f - cons.m_static_friction);
	cons.m_dynamic_friction	= cons.m_dynamic_friction / (1.000001f - cons.m_dynamic_friction);
}

// Set the combined collision mass matrix
// 'mass_mask' is used to treat either object A or B as infinite mass
void ConstraintAccumulator::SetCollisionMatrix(Constraint& cons, Rigidbody const& rbA, Rigidbody const& rbB, int mass_mask) const
{
	v4 pointA = cons.m_pointA;
	v4 pointB = cons.m_pointB;

	// "mass" is a matrix defined as impulse = mass * drelative_velocity.
	// "inv_mass" is also called the 'K' matrix and is equal to:
	//	[(1/MassA + 1/MassB)*Identity - (CrossProductMatrix(pointA)*InvMassTensorWS()A*CrossProductMatrix(pointA) + CrossProductMatrix(pointB)*InvMassTensorWS()B*CrossProductMatrix(pointB))]
	// Say "inv_mass" = "inv_mass1" + "inv_mass2" then
	// "inv_mass1" = [(1/MassA)*Identity - (pointA.CrossProductMatrix()*InvMassTensorWS()A*pointA.CrossProductMatrix())] and
	// "inv_mass2" = [(1/MassB)*Identity - (pointB.CrossProductMatrix()*InvMassTensorWS()B*pointB.CrossProductMatrix())]
	m3x4 inv_mass = m3x4::Zero();
	if( mass_mask & 1 )
	{
		auto cpmA = CPM(pointA); // pointA is object relative
		inv_mass += rbA.m_inv_mass * (m3x4::Identity() - (cpmA * rbA.m_ws_inv_inertia_tensor * cpmA));
	}
	if( mass_mask & 2 )
	{
		auto cpmB = CPM(pointB); // pointB is object relative
		inv_mass += rbB.m_inv_mass * (m3x4::Identity() - (cpmB * rbB.m_ws_inv_inertia_tensor * cpmB));
	}
	cons.m_mass = Invert(inv_mass);
}

// Add a collision or resting contact constraint for a pair of objects that are overlapping
// The collision doesn't need to be increasing penetration
void ConstraintAccumulator::AddContact(Rigidbody& rbA, Rigidbody& rbB, ContactManifold& manifold)
{
	PR_ASSERT(PR_DBG_PHYSICS, manifold.Size() > 0, "");

	// Allocate a constraint from the buffer and fill it in
	// as a collision or resting contact constraint.
	ConstraintBlock& pair = AllocateConstraints(rbA, rbB, manifold.Size());

	// Get approximate gravity values near the contacts
	v4 contact_centre		= manifold.ContactCentre();
	v4 gravity				= GetGravitationalAcceleration(contact_centre);
	pair.m_grav_potential	= GetGravitationalPotential(contact_centre);
	float grav_accel		= Length(gravity);

	// Calculate the resting contact speed due to gravity.
	pair.m_resting_contact_speed = grav_accel * m_step_size;
	//float rest_speed_sq = Sqr(pair.m_resting_contact_speed);
	//if( pair.m_objA->Velocity().LengthSq() < rest_speed_sq )
	//	pair.m_objA->SetVelocity(v4::Zero());
	//if( pair.m_objB->Velocity().LengthSq() < rest_speed_sq )
	//	pair.m_objB->SetVelocity(v4::Zero());
	//if( pair.m_objA->AngMomentum().LengthSq() < rest_speed_sq )
	//	pair.m_objA->SetAngMomentum(v4::Zero());
	//if( pair.m_objB->AngMomentum().LengthSq() < rest_speed_sq )
	//	pair.m_objB->SetAngMomentum(v4::Zero());

	// Fill out the constraints for the pair
	for( uint32_t i = 0, j = 0; i != manifold.Size(); ++i )
	{
		Contact const& contact	= manifold[i];
		Constraint& cons		= pair[j++];
		cons.m_type				= EConstraintType_Collision;
		cons.m_pointA			= contact.m_pointA - rbA.Position();
		cons.m_pointB			= contact.m_pointB - rbB.Position();
		cons.m_normal			= contact.m_normal;

		// This is the minimum separation speed we want the objects to have.
		// If the penetration is less than the tolerance then we don't require
		// the objects to separate.
		cons.m_separation_speed_min = 0.0f;
		if( contact.m_depth > 0.0f )//g_penetration_tolerance )
		{
			float depth = pr::Min(contact.m_depth, g_max_penetration);
			cons.m_separation_speed_min = Sqrt(2.0f * grav_accel * depth);
		}

		// This chooses which object should be considered infinite mass during
		// shock propagation. The lower object should be infinite mass.
		float contact_direction = Dot3(contact.m_normal, gravity) * 10.0f;
		if     ( contact_direction >  grav_accel )	cons.m_shock_propagation_mask = 2;	// rbA is lower, use rbB's mass only
		else if( contact_direction < -grav_accel )	cons.m_shock_propagation_mask = 1;	// rbB is lower, use rbA's mass only
		else										cons.m_shock_propagation_mask = 3;	// No clear bias in direction

		// Set the combined material properties
		SetMaterialProperties(cons, contact.m_material_idA, contact.m_material_idB);

		// Set the combined mass matrix for the collision
		SetCollisionMatrix(cons, rbA, rbB);
	}
}

// Solve *************************************

// Predicate for sorting ConstraintBlocks, sorts by set id first, then grav potential.
struct Pred_PairPtr
{
	uint8_t const* m_map;
	Pred_PairPtr(uint8_t const* map) : m_map(map) {}
	bool operator()(ConstraintBlock const* lhs, ConstraintBlock const* rhs) const
	{
		uint8_t const& Rset = m_map[rhs->m_constraint_set];
		uint8_t const& Lset = m_map[lhs->m_constraint_set];
		if( Lset != Rset ) return Lset < Rset;
		return lhs->m_grav_potential < rhs->m_grav_potential;
	}
};

// Solve the constraints by applying impulses.
// Algorithm outline:
//	Each ContactManifold contains all of the points of contact between rbA an rbB
//	Each ContactManifold corresponds to one ConstraintBlock
//	When adding a ContactManifold, the desired post-collision relative velocity is
//	 calculated for each contact in isolation, along with the collision matrix, etc.
//	 Also, the impulse accumulators for each object are reset.
//	For each ConstraintBlock in order of gravitational potential:
//		For each iteration:
//			For each Constraint:
//				Calculate the relative velocity of the points of contact including any
//				 impulses in the impulse accumulators of each object
//				Calculate the impulse that gives the desired change in relative velocity
//				 assuming the constraint is in isolation.
//				Save the impulse in the constraint.
//			For each Constraint:
//				Add the constraint impulses to the impulse accumulator in each object
//		Apply the accumulator impulses to the object velocities
void ConstraintAccumulator::Solve()
{
	// Only solve if there's actually something to do
	if( m_num_pairs == 0 )
		return;

	// Sort the pair pointers so that they are ordered by gravitational potential.
	std::sort(m_pairs, m_pairs + m_num_pairs, Pred_PairPtr(m_map));

	// ToDo: The set id's should allow us to solve each set in parallel.
	// There should be no shared Rigidbodies between each set. For now
	// solve the whole lot as one set.
	SolverParams params;
	params.m_this  = this;
	params.m_first = m_pairs;
	params.m_last  = m_pairs + m_num_pairs;
	SolveConstraintSet(&params);
	// Begin threads for each set
	// _beginthread(ConstraintAccumulator::SolveConstraintSet, 0, &params);
	// Wait for all threads to complete.

	// Reset the buffers.
	m_num_sets = 0;
	m_buffer_ptr = m_buffer;
	m_num_pairs = 0;
}

// Calculates impulses for groups of objects that share connected constraints
void ConstraintAccumulator::SolveConstraintSet(void* context)
{
	SolverParams& params = *static_cast<SolverParams*>(context);
	ConstraintAccumulator* This = params.m_this;

	#if PR_DBG_COLLISION
	static float scale = 0.1f;
	std::string str;
	ldr::GroupStart("ConstraintBlocks", str);
	for( ConstraintBlock** p = params.m_first; p != params.m_last; ++p )
	{
		ConstraintBlock& pair = **p;
		ldr::PhRigidbody("obj", "FFFF0000", *pair.m_objA, str);
		ldr::LineD("vel", "FFFF0000", pair.m_objA->Position(), pair.m_objA->Velocity()*scale, str);
		ldr::LineD("avel", "FF00FF00", pair.m_objA->Position(), pair.m_objA->AngVelocity()*scale, str);
	}
	ldr::Position(v4::make(-2.5f, 0.0f, 0.0f, 1.0f), str);
	ldr::GroupEnd(str);
	StringToFile(str, "C:/DeleteMe/collision_constraintblocks.pr_script");
	#endif//PR_DBG_COLLISION

	// Do several passes of the constraint set doing normal collision resolution.
	int const NumPasses = 1;//params.m_last - params.m_first;//IntegerSqrt(static_cast<int>(params.m_last - params.m_first));
	for( int pass = 0; pass != NumPasses; ++pass )
	{
		for( ConstraintBlock** p = params.m_first; p != params.m_last; ++p )
		{
			ConstraintBlock& pair = **p;

			#if PR_DBG_COLLISION
			std::string str;
			ldr::GroupStart("ConstraintBlock", str);
			ldr::PhRigidbody("objA", "FFFF0000", *pair.m_objA, str);
			ldr::PhRigidbody("objB", "FF0000FF", *pair.m_objB, str);
			ldr::LineD("velA", "FFFF0000", pair.m_objA->Position(), pair.m_objA->Velocity()*scale, str);
			ldr::LineD("velB", "FFFF0000", pair.m_objB->Position(), pair.m_objB->Velocity()*scale, str);
			ldr::LineD("avelA", "FF00FF00", pair.m_objA->Position(), pair.m_objA->AngVelocity()*scale, str);
			ldr::LineD("avelB", "FF00FF00", pair.m_objB->Position(), pair.m_objB->AngVelocity()*scale, str);
			ldr::GroupEnd(str);
			StringToFile(str, "C:/DeleteMe/collision_constraintblock.pr_script");
			#endif//PR_DBG_COLLISION

			// Solve the constraints for a pair of objects
			This->SolveConstraintBlock(pair, false);

			#if PR_DBG_COLLISION
			str.clear();
			ldr::GroupStart("ConstraintBlock", str);
			ldr::PhRigidbody("objA", "FFFF0000", *pair.m_objA, str);
			ldr::PhRigidbody("objB", "FF0000FF", *pair.m_objB, str);
			ldr::LineD("velA", "FFFF0000", pair.m_objA->Position(), pair.m_objA->Velocity()*scale, str);
			ldr::LineD("velB", "FFFF0000", pair.m_objB->Position(), pair.m_objB->Velocity()*scale, str);
			ldr::LineD("avelA", "FF00FF00", pair.m_objA->Position(), pair.m_objA->AngVelocity()*scale, str);
			ldr::LineD("avelB", "FF00FF00", pair.m_objB->Position(), pair.m_objB->AngVelocity()*scale, str);
			ldr::GroupEnd(str);
			StringToFile(str, "C:/DeleteMe/collision_constraintblock.pr_script");
			#endif//PR_DBG_COLLISION
		}
	}

	// Do one more pass this time with shock propagation
	// (i.e. treat objects lower in the gravity field as infinite mass)
	for( ConstraintBlock** p = params.m_first; p != params.m_last; ++p )
	{
		ConstraintBlock& pair = **p;

		#if PR_DBG_COLLISION
		std::string str;
		ldr::GroupStart("ConstraintBlock", str);
		ldr::PhRigidbody("objA", "FFFF0000", *pair.m_objA, str);
		ldr::PhRigidbody("objB", "FF0000FF", *pair.m_objB, str);
		ldr::LineD("velA", "FFFF0000", pair.m_objA->Position(), pair.m_objA->Velocity()*scale, str);
		ldr::LineD("velB", "FFFF0000", pair.m_objB->Position(), pair.m_objB->Velocity()*scale, str);
		ldr::LineD("avelA", "FF00FF00", pair.m_objA->Position(), pair.m_objA->AngVelocity()*scale, str);
		ldr::LineD("avelB", "FF00FF00", pair.m_objB->Position(), pair.m_objB->AngVelocity()*scale, str);
		ldr::GroupEnd(str);
		StringToFile(str, "C:/DeleteMe/collision_constraintblock.pr_script");
		#endif//PR_DBG_COLLISION

		// Solve the constraints for a pair of objects
		This->SolveConstraintBlock(pair, true);

		#if PR_DBG_COLLISION
		str.clear();
		ldr::GroupStart("ConstraintBlock", str);
		ldr::PhRigidbody("objA", "FFFF0000", *pair.m_objA, str);
		ldr::PhRigidbody("objB", "FF0000FF", *pair.m_objB, str);
		ldr::LineD("velA", "FFFF0000", pair.m_objA->Position(), pair.m_objA->Velocity()*scale, str);
		ldr::LineD("velB", "FFFF0000", pair.m_objB->Position(), pair.m_objB->Velocity()*scale, str);
		ldr::LineD("avelA", "FF00FF00", pair.m_objA->Position(), pair.m_objA->AngVelocity()*scale, str);
		ldr::LineD("avelB", "FF00FF00", pair.m_objB->Position(), pair.m_objB->AngVelocity()*scale, str);
		ldr::GroupEnd(str);
		StringToFile(str, "C:/DeleteMe/collision_constraintblock.pr_script");
		#endif//PR_DBG_COLLISION

		// Reset the set ids
		pair.m_objA->m_constraint_set = NoConstraintSet;
		pair.m_objB->m_constraint_set = NoConstraintSet;
	}
}

// Solve for the impulses to apply between a pair of objects
void ConstraintAccumulator::SolveConstraintBlock(ConstraintBlock& pair, bool shock_propagation) const
{
	PR_EXPAND(PR_DBG_COLLISION, StringToFile("", "C:/DeleteMe/collision_impulses.pr_script"));
	PR_EXPAND(PR_DBG_COLLISION, StringToFile("", "C:/DeleteMe/collision_desired_relvelocity.pr_script"));
	PR_EXPAND(PR_DBG_COLLISION, StringToFile("", "C:/DeleteMe/collision_relvelocity.pr_script"));
	PR_EXPAND(PR_DBG_COLLISION, std::string str);

	// Calculate the desired relative velocities.
	uint32_t solve = CalculateDesiredVelocities(pair, shock_propagation);
	if( solve == pair.m_num_constraints )
		return; // Nothing needs solving

	// Adjust the mass matrices for shock propagation
	if( shock_propagation )
	{
		for( uint32_t c = 0; c != pair.m_num_constraints; ++c )
		{
			Constraint& cons = pair[c];
			if( cons.m_shock_propagation_mask != 3 )
				SetCollisionMatrix(cons, *pair.m_objA, *pair.m_objB, cons.m_shock_propagation_mask);
		}
	}

	// Apply impulses to make the constraints achieve the desired relative velocities
	static int MaxIterations = 10;
	for (int i = 0; solve != pair.m_num_constraints && i != MaxIterations; ++i)
	{
		// Solve the constraint with the greatest error in normal speed
		auto& cons0 = pair[solve];

		bool objA_finite_mass = !shock_propagation || (cons0.m_shock_propagation_mask & 1) != 0;
		bool objB_finite_mass = !shock_propagation || (cons0.m_shock_propagation_mask & 2) != 0;

		// Find the current relative velocity
		v4 relative_velocity0 = pair.m_objB->VelocityAt(cons0.m_pointB) - pair.m_objA->VelocityAt(cons0.m_pointA);

		// Calculate the impulse that will provide the desired change in relative velocity
		cons0.m_impulse = cons0.m_mass * (cons0.m_desired_final_rel_velocity - relative_velocity0);
		PR_EXPAND(PR_DBG_COLLISION, ldr::LineD("impulse", "FFFFFF00", pair.m_objA->Position() + cons0.m_pointA, -cons0.m_impulse, str));
		PR_EXPAND(PR_DBG_COLLISION, StringToFile(str, "C:/DeleteMe/collision_impulses.pr_script"); str.clear());

		// Apply the impulse to the objects
		pair.m_objA->ApplyWSImpulse(-cons0.m_impulse * objA_finite_mass, cons0.m_pointA);
		pair.m_objB->ApplyWSImpulse( cons0.m_impulse * objB_finite_mass, cons0.m_pointB);

		PR_EXPAND(PR_DBG_COLLISION, v4 new_rel_vel = pair.m_objB->VelocityAt(cons0.m_pointB) - pair.m_objA->VelocityAt(cons0.m_pointA));
		PR_EXPAND(PR_DBG_COLLISION, ldr::LineD("rel_vel", "FF000080", pair.m_objA->Position() + cons0.m_pointA, -new_rel_vel, str));
		PR_EXPAND(PR_DBG_COLLISION, StringToFile(str, "C:/DeleteMe/collision_relvelocity.pr_script", true); str.clear());

		// Find the next constraint to solve
		static float error_tolerance_sq = 0.00001f;
		float max_error_sq = error_tolerance_sq;
		uint32_t  next_to_solve = pair.m_num_constraints;
		for (uint32_t c = 0; c != pair.m_num_constraints; ++c)
		{
			// Can skip the constraint we've just solved because its error will be zero.
			if (c == solve)
				continue;

			auto& cons1 = pair[c];
			v4 relative_velocity1 = pair.m_objB->VelocityAt(cons1.m_pointB) - pair.m_objA->VelocityAt(cons1.m_pointA);
			PR_EXPAND(PR_DBG_COLLISION, ldr::LineD("rel_vel", "FF0000FF", pair.m_objA->Position() + cons1.m_pointA, -relative_velocity1, str));
			PR_EXPAND(PR_DBG_COLLISION, StringToFile(str, "C:/DeleteMe/collision_relvelocity.pr_script", true); str.clear());

			float vel_error_sq = LengthSq(relative_velocity1 - cons1.m_desired_final_rel_velocity);
			if( vel_error_sq > max_error_sq )
			{
				max_error_sq = vel_error_sq;
				next_to_solve = c;
			}
		}
		solve = next_to_solve;
	}
}

// Find the isolated desired relative velocities for each constraint.
// Returns the index of the constraint that is most violated or 'num_constraints'
// if all constraints are satisfied.
uint32_t ConstraintAccumulator::CalculateDesiredVelocities(ConstraintBlock& pair, bool shock_propagation) const
{
	PR_EXPAND(PR_DBG_COLLISION, std::string str);

	uint32_t first_to_solve = pair.m_num_constraints;
	float max_error_sq = maths::tiny<float>;

	// Calculate the desired final relative velocities for each constraint
	// before we start applying impulses
	for( uint32_t c = 0; c != pair.m_num_constraints; ++c )
	{
		Constraint& cons = pair[c];

		PR_EXPAND(PR_DBG_COLLISION, ldr::LineD("normal", "FFFFFFFF", pair.m_objA->Position() + cons.m_pointA, cons.m_normal * 0.2f, str));
		PR_EXPAND(PR_DBG_COLLISION, StringToFile(str, "C:/DeleteMe/collision_normal.pr_script", true); str.clear());

		// Relative velocity of the point of contact. Calculated here because
		// object velocities change as we process each constraint block.
		v4 relative_velocity = pair.m_objB->VelocityAt(cons.m_pointB)
							 - pair.m_objA->VelocityAt(cons.m_pointA);
		PR_EXPAND(PR_DBG_COLLISION, ldr::LineD("rel_vel", "FF0000FF", pair.m_objA->Position() + cons.m_pointA, -relative_velocity, str));
		PR_EXPAND(PR_DBG_COLLISION, StringToFile(str, "C:/DeleteMe/collision_relvelocity.pr_script", true); str.clear());

		// Find the desired relative normal and tangential speeds
		float rel_norm_speed = Dot3(relative_velocity, cons.m_normal);
		float rel_tang_speed = Length(relative_velocity - rel_norm_speed * cons.m_normal);

		// The desired post-collision velocity for this contact is basically the incident
		// velocity reflected through the collision normal but with elasticity and friction
		// taken into consideration.
		if( rel_norm_speed > -cons.m_separation_speed_min  )
		{
			float elasticity = !shock_propagation && rel_norm_speed > pair.m_resting_contact_speed ? cons.m_elasticity : 0.0f;
			float desired_norm_speed = -pr::Max(cons.m_separation_speed_min, elasticity * rel_norm_speed);
			float desired_tang_speed = rel_tang_speed; // No friction case

			cons.m_desired_final_rel_velocity = desired_norm_speed * cons.m_normal; // partial

			// If applying a minimum separation speed increases the final velocity then energy
			// will be added. We need to remove this from the tangential speed if possible
			float delta_norm_speed = Abs(desired_norm_speed) - Abs(rel_norm_speed);
			if( delta_norm_speed > 0.0f )
			{
				desired_tang_speed = pr::Max(rel_tang_speed - delta_norm_speed, 0.0f);
			}

			// If the change in tangential speed is less than the static friction coeff
			// multiplied by the change in normal speed then the contact is 'sticky'
			float slip_thres = cons.m_static_friction * (rel_norm_speed - desired_norm_speed);
			if( desired_tang_speed > slip_thres )
			{
				// The desired tangential speed depends on friction. Non-slip collisions
				// have a tangential speed of zero, no-friction collisions have a tangential
				// speed equal to the incident tangential speed. Make up something that gives
				// a simple friction model.
				desired_tang_speed *= 1.0f - pr::Min(cons.m_dynamic_friction, 1.0f);

				// If the tangential speed is now less than the static friction coeff the contact becomes sticky
				if( desired_tang_speed > slip_thres )
				{
					v4 tangent = relative_velocity - rel_norm_speed * cons.m_normal;
					cons.m_desired_final_rel_velocity += (desired_tang_speed / rel_tang_speed) * tangent;
				}
			}
		}
		// If the points of contact are moving out of collision at >= the minimum
		// separation speed, then the desired final velocity is the current velocity.
		else
		{
			cons.m_desired_final_rel_velocity = relative_velocity;
		}

		PR_EXPAND(PR_DBG_COLLISION, ldr::LineD("rel_vel", "FF00FF00", pair.m_objA->Position() + cons.m_pointA, -cons.m_desired_final_rel_velocity, str));
		PR_EXPAND(PR_DBG_COLLISION, StringToFile(str, "C:/DeleteMe/collision_desired_relvelocity.pr_script", true); str.clear());

		// Find the constraint with the greatest error in normal speed.
		// We solve sequentially in order from greatest to smallest
		float vel_error_sq = LengthSq(relative_velocity - cons.m_desired_final_rel_velocity);
		if( vel_error_sq > max_error_sq )
		{
			max_error_sq = vel_error_sq;
			first_to_solve = c;
		}
	}
	return first_to_solve;
}
