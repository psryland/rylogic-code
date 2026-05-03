//*********************************************
// Collision
//  Copyright (C) Rylogic Ltd 2016
//*********************************************
#pragma once
#include "pr/physics/forward.h"
#include "pr/physics/shape/mass.h"
#include "pr/physics/shape/inertia.h"
#include "pr/physics/shape/shape_mass.h"
#include "pr/physics/materials/material.h"

namespace pr::physics
{
	struct ShapeBuilder
	{
		// Notes:
		//  - An object for building collision shapes.
		//  - The shape builder is part of the physics library, not the collision library because it's main job
		//    is to determine the inertia properties of the shape, which depend on physics materials, inertia matrices, etc.

		// Settings for the shape builder
		struct Settings
		{
			float m_min_mass;   // The minimum mass a primitive may have (kg)
			float m_min_volume; // The minimum volume a primitive may have (m^3)
			std::function<Material const&(MaterialId)> m_mat_lookup;

			Settings()
				: m_min_mass(1.0f)
				, m_min_volume(0.001f * 0.001f * 0.001f)
				, m_mat_lookup(NoMaterial)
			{}
			static Material const& NoMaterial(MaterialId)
			{
				static Material s_mat;
				return s_mat;
			}
		};

		// Instances of primitives
		struct alignas(16) Prim
		{
			byte_data<16>  m_data;  // Data containing the shape
			MassProperties m_mp;    // Mass properties for the primitive
			BBox           m_bbox;  // Bounding box for the primitive
			Shape*         m_shape; // Used in the debugger only

			Shape& shape()
			{
				m_shape = m_data.begin<Shape>();
				return *m_shape;
			}
		};

		// A collision model
		struct alignas(16) Model
		{
			using PrimList = std::vector<std::unique_ptr<Prim>>;
			PrimList       m_prim_list; // The primitives in the model
			MassProperties m_mp;        // The combined mass properties
			BBox           m_bbox;      // The model bounding box

			Model() = default;
			Model(Model const&) = delete;
			Model& operator = (Model const&) = delete;
		};

		Settings m_settings;
		std::unique_ptr<Model> m_model;

		ShapeBuilder(Settings const& settings = Settings())
			: m_settings(settings)
			, m_model(std::unique_ptr<Model>(new Model))
		{}

		// Begin a new physics model
		void Reset()
		{
			m_model = std::unique_ptr<Model>(new Model);
		}

		// Add shapes to the current model.
		template <ShapeType TShape>
		void AddShape(TShape const& shape)
		{
			// Create a new primitive to contain the shape
			auto prim = std::unique_ptr<Prim>(new Prim);
			prim->m_data.push_back({ byte_ptr(&shape), s_cast<size_t>(shape.m_base.m_size) });
			auto& s = shape_cast<TShape>(prim->shape());

			// Capture the primitive mass properties. Shape-local geometry is fixed at construction time; any local CoM offset is folded in when the model properties are calculated.
			auto density = m_settings.m_mat_lookup(s.m_base.m_material_id).m_density;
			prim->m_mp = CalcMassProperties(s, density);

			// Set the bounding box
			prim->m_bbox = CalcBBox(s);
			s.m_base.m_bbox = prim->m_bbox;

			// Validate the primitive
			if (prim->m_mp.m_mass < m_settings.m_min_volume * density)
				throw std::runtime_error("Shape volume is too small");
			if (prim->m_mp.m_mass < m_settings.m_min_mass)
				prim->m_mp.m_mass = m_settings.m_min_mass;

			// Add the primitive to the model
			m_model->m_prim_list.push_back(std::move(prim));
		}

		// Serialise the shape data.
		Shape* BuildShape(byte_data<16>& model_data, MassProperties& mp, v4& model_to_CoMframe, EShape container = EShape::Array, Shape::EFlags shape_flags = Shape::EFlags::None)
		{
			// Notes:
			//  - It should be possible to insert the shape returned from here into a larger shape.
			//  - The shape-to-root transform in each shape is independent of the shape hierarchy, because evaluating a tree of transforms
			//    is too expensive for collision detection. This means attaching a shape to a parent composite shape requires updating the
			//    shape-to-root transform of the child shapes.
			//  - Shape flags only apply to composite shape types.
			auto& model = *m_model;

			assert("No shapes have been added" && !model.m_prim_list.empty());
			if (model.m_prim_list.size() == 1)
				container = EShape::NoShape;

			// Calculate the mass and centre of mass of the model
			CalculateMassAndCentreOfMass();

			// Move the model to the centre of mass frame
			MoveToCentreOfMassFrame(model_to_CoMframe);

			// Determine the bounding box for the whole model
			CalculateBoundingBox();

			// Create the inertia for the model
			CalculateInertia();

			// Save the mass properties we've figured out
			mp = model.m_mp;

			auto base = model_data.size();
			switch (container)
			{
				case EShape::NoShape:
				{
					assert("Model contains multiple primitives. 'hierarchy' should be one of the composite shape types" && model.m_prim_list.size() == 1);
					model_data.append(model.m_prim_list.front()->m_data);
					return &model_data.at_byte_ofs<Shape>(base);
				}
				case EShape::Array:
				{
					// Add the array shape header, followed by the shapes in the array
					model_data.push_back<ShapeArray>();
					for (auto& prim_ptr : model.m_prim_list)
						model_data.append(prim_ptr->m_data);

					// Update the array shape header
					auto& arr = model_data.at_byte_ofs<ShapeArray>(base);
					arr.Complete(model.m_prim_list.size());
					arr.m_base.m_flags = shape_flags;

					// Return the shape
					return &arr.m_base;
				}
				default:
				{
					throw std::runtime_error("Unsupported shape container type");
				}
			}
		}

		// Return the primitive centre of mass in model space.
		static v4 PrimitiveCentreOfMass(Prim& prim)
		{
			return (prim.shape().m_s2r * prim.m_mp.m_centre_of_mass.w1()).w0();
		}

		// Return the primitive centre-of-mass frame to model-space transform.
		static m4x4 PrimitiveCoMToModel(Prim& prim)
		{
			return prim.shape().m_s2r * m4x4::Translation(prim.m_mp.m_centre_of_mass);
		}

		// Calculate the mass of the model by adding up the mass of all of the primitives.
		// Also, calculate the centre of mass for the object.
		void CalculateMassAndCentreOfMass()
		{
			auto& model = *m_model;

			model.m_mp.m_mass = 0.0f;
			model.m_mp.m_centre_of_mass = v4::Zero();
			for (auto& prim_ptr : model.m_prim_list)
			{
				auto& prim = *prim_ptr;

				// Accumulate mass and centre of mass
				model.m_mp.m_mass           += prim.m_mp.m_mass;
				model.m_mp.m_centre_of_mass += prim.m_mp.m_mass * PrimitiveCentreOfMass(prim);
			}

			// Find the centre of mass position
			model.m_mp.m_centre_of_mass /= model.m_mp.m_mass;
			model.m_mp.m_centre_of_mass.w = 0.0f;
		}

		// Relocate the collision model around the centre of mass
		void MoveToCentreOfMassFrame(v4& model_to_CoMframe)
		{
			auto& model = *m_model;

			// Save the shift from model space to centre of mass space
			model_to_CoMframe = model.m_mp.m_centre_of_mass;

			// Now move all of the models so that they are centred around the centre of mass
			for (auto& prim : model.m_prim_list)
				prim->shape().m_s2r.pos -= model.m_mp.m_centre_of_mass;

			// The offset to the centre of mass is now zero
			model.m_mp.m_centre_of_mass = v4::Zero();
		}

		// Calculate the bounding box for 'm_model'.
		void CalculateBoundingBox()
		{
			auto& model = *m_model;

			model.m_bbox = BBox::Reset();
			for (auto& prim : model.m_prim_list)
				Grow(model.m_bbox, prim->shape().m_s2r * prim->m_bbox);
		}

		// Calculates the inertia for 'm_model'
		void CalculateInertia()
		{
			auto& model = *m_model;

			auto model_inertia = m3x3{};
			for (auto& p : model.m_prim_list)
			{
				auto& prim = *p;

				// The primitive mass properties are measured about the shape origin. Translate to the primitive CoM, then into the model CoM frame.
				auto primitive_inertia = Inertia{prim.m_mp.m_os_unit_inertia, prim.m_mp.m_mass};
				primitive_inertia = Translate(primitive_inertia, prim.m_mp.m_centre_of_mass, ETranslateInertia::TowardCoM);

				// Transform it to object space
				primitive_inertia = Transform(primitive_inertia, PrimitiveCoMToModel(prim), ETranslateInertia::AwayFromCoM);

				// Add the inertia to the object inertia (mass divided out at the end)
				model_inertia += primitive_inertia.To3x3();
			}

			// Normalised the model inertia
			model.m_mp.m_os_unit_inertia = model_inertia / model.m_mp.m_mass;
		}

		// Return access to a shape
		Shape const& GetShape(int i) const
		{
			auto& model = *m_model;
			return model.m_prim_list[i]->shape();
		}
	};
}

