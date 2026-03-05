//*********************************************
// Physics engine
//  Copyright (c) Rylogic Ltd 2006
//*********************************************

#include "physics/utility/stdafx.h"
#include "pr/physics/shape/builder/shapebuilder.h"
#include "pr/physics/utility/globalfunctions.h"
#include "pr/physics/shape/shapes.h"
#include "pr/physics/material/imaterial.h"

using namespace pr;
using namespace pr::ph;

// Constructor
ShapeBuilder::ShapeBuilder(const ph::ShapeBuilderSettings& settings)
:m_settings(settings)
,m_model()
{}

// Start a new object
void ShapeBuilder::Reset()
{
	m_model = Model();
}

// Serialise the shape data.
// It should be possible to insert the shape returned from here into a larger shape.
// The highest level shape in a composite shape should have a shape_to_model transform of id.
// Shape flags only apply to composite shape types
ph::Shape* ShapeBuilder::BuildShape(ByteCont& model_data, MassProperties& mp, v4& model_to_CoMframe, EShapeHierarchy hierarchy, EShapeFlags shape_flags)
{
	PR_ASSERT(PR_DBG_PHYSICS, !m_model.m_prim_list.empty(), "No shapes have been added");

	CalculateMassAndCentreOfMass();
	MoveToCentreOfMassFrame(model_to_CoMframe);
	CalculateBoundingBox();
	CalculateInertiaTensor();

	// Save the mass properties we've figured out
	mp = m_model.m_mp;

	std::size_t base = model_data.size();
	switch( hierarchy )
	{
	case EShapeHierarchy_Single:
		PR_ASSERT(PR_DBG_PHYSICS, m_model.m_prim_list.size() == 1, "Only the first primitive will be used");
		pr::AppendData(model_data, m_model.m_prim_list.front()->m_data);
		return reinterpret_cast<Shape*>(&model_data[base]);

	case EShapeHierarchy_Array:
		{
			pr::AppendData(model_data, ShapeArray());
			for( TPrimList::const_iterator p = m_model.m_prim_list.begin(), p_end = m_model.m_prim_list.end(); p != p_end; ++p )
				pr::AppendData(model_data, (*p)->m_data);
			ShapeArray& arr = *reinterpret_cast<ShapeArray*>(&model_data[base]);
			arr.set(m_model.m_prim_list.size(), model_data.size() - base, m4x4::Identity(), 0, shape_flags);
			arr.m_base.m_bbox = m_model.m_bbox;
			return &arr.m_base;
		}
	case EShapeHierarchy_BvTree:
		{
			PR_STUB_FUNC();
			return 0;
		}
	default:
		PR_ASSERT(PR_DBG_PHYSICS, false, "Unknown shape hierarchy");
		return 0;
	}
}

// Calculate the mass of 'm_model' by adding up the mass of all of the primitives.
// Also, calculate the centre of mass for the object
void ShapeBuilder::CalculateMassAndCentreOfMass()
{
	m_model.m_mp.m_mass = 0.0f;
	m_model.m_mp.m_centre_of_mass = pr::v4::Zero();
	for( TPrimList::const_iterator p = m_model.m_prim_list.begin(), p_end = m_model.m_prim_list.end(); p != p_end; ++p )
	{
		Prim const& prim = *(*p).m_ptr;
		PR_ASSERT(PR_DBG_PHYSICS, FEql(prim.m_mp.m_centre_of_mass,pr::v4::Zero()), ""); // All shapes should be centred on their centre of mass when added to the builder
		m_model.m_mp.m_mass				+= prim.m_mp.m_mass;
		m_model.m_mp.m_centre_of_mass	+= prim.m_mp.m_mass * prim.GetShape().m_shape_to_model.pos;
	}
	m_model.m_mp.m_centre_of_mass /= m_model.m_mp.m_mass;
	m_model.m_mp.m_centre_of_mass.w = 0.0f;
}

// Relocate the collision model around the centre of mass
void ShapeBuilder::MoveToCentreOfMassFrame(v4& model_to_CoMframe)
{
	// Save the shift from model space to centre of mass space
	model_to_CoMframe = m_model.m_mp.m_centre_of_mass;

	// Now move all of the models so that they are centred around the centre of mass
	for( TPrimList::iterator p = m_model.m_prim_list.begin(), p_end = m_model.m_prim_list.end(); p != p_end; ++p )
		(*p)->GetShape().m_shape_to_model.pos -= m_model.m_mp.m_centre_of_mass;

	// The offset to the centre of mass is now zero
	m_model.m_mp.m_centre_of_mass = pr::v4::Zero();
}

// Calculate the bounding box for 'm_model'.
void ShapeBuilder::CalculateBoundingBox()
{
	m_model.m_bbox.reset();
	for( TPrimList::const_iterator p = m_model.m_prim_list.begin(), p_end = m_model.m_prim_list.end(); p != p_end; ++p )
	{
		Prim const& prim = *(*p).m_ptr;
		Grow(m_model.m_bbox, prim.GetShape().m_shape_to_model * prim.m_bbox);
	}
}

// Calculates the inertia tensor for 'm_model'
void ShapeBuilder::CalculateInertiaTensor()
{
	m_model.m_mp.m_os_inertia_tensor = m3x4::Zero();
	for( TPrimList::const_iterator p = m_model.m_prim_list.begin(), p_end = m_model.m_prim_list.end(); p != p_end; ++p )
	{
		Prim const& prim = *(*p).m_ptr;
		PR_ASSERT(PR_DBG_PHYSICS, FEql(prim.m_mp.m_centre_of_mass,pr::v4::Zero()), ""); // All primitives should be in their inertial frame

		m3x4 primitive_inertia  = prim.m_mp.m_mass * prim.m_mp.m_os_inertia_tensor;

		// Rotate the inertia tensor into object space
		m3x4 prim_to_model = prim.GetShape().m_shape_to_model.rot;
		primitive_inertia  = prim_to_model * primitive_inertia * Transpose(prim_to_model);

		// Translate the inertia tensor using the parallel axis theorem
		ParallelAxisTranslateInertia(primitive_inertia, prim.GetShape().m_shape_to_model.pos, prim.m_mp.m_mass, ParallelAxisTranslate::AwayFromCoM);

		// Add the inertia to the object inertia tensor
		m_model.m_mp.m_os_inertia_tensor += primitive_inertia;
	}

	// Assume mass == 1.0f for the model inertia tensor
	m_model.m_mp.m_os_inertia_tensor /= m_model.m_mp.m_mass;
}
