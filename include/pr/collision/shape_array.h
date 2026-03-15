//*********************************************
// Collision
//  Copyright (C) Rylogic Ltd 2016
//*********************************************
#pragma once
#include "pr/collision/forward.h"
#include "pr/collision/shape.h"

namespace pr::collision
{
	// Shape array
	struct ShapeArray
	{
		Shape m_base;

		// The number of shapes in the array
		size_t m_num_shapes;
		size_t pad[3]; // Pad to 16 bytes

		// Followed by an array of other shape types (with different sizes):
		// ShapeBox s0;
		// ShapeSphere s1;
		// ...

		explicit ShapeArray(m4x4 const& shape_to_parent = m4x4::Identity(), MaterialId material_id = 0, Shape::EFlags flags = Shape::EFlags::None)
			:m_base(EShape::Array, sizeof(ShapeArray), shape_to_parent, material_id, flags)
			,m_num_shapes()
		{
			// Careful: We can't be sure of what follows this object in memory.
			// The shapes that belong to this array may not be there yet.
			// Differ calculating the bounding box to the caller (i.e. Caller should call 'Complete')
		}
		void Complete(size_t num_shapes)
		{
			// Determine the size of the array
			auto ptr = begin();
			for (auto i = num_shapes; i-- != 0;)
				ptr = next(ptr);

			// Calculate the bounding box
			auto bb = BBox::Reset();
			for (Shape const* i = begin(), *i_end = end(); i != i_end; i = next(i))
				Grow(bb, i->m_s2p * i->m_bbox);

			m_num_shapes = num_shapes;
			m_base.m_size = sizeof(ShapeArray) + byte_ptr(ptr) - byte_ptr(begin());
			m_base.m_bbox = CalcBBox(*this);

			// All shape sizes should be a multiple of 16 bytes
			assert((m_base.m_size & 0xf) == 0 && "Shape size must be a multiple of 16 bytes");
		}

		operator Shape const&() const
		{
			return m_base;
		}
		operator Shape&()
		{
			return m_base;
		}
		operator Shape const*() const
		{
			return &m_base;
		}
		operator Shape*()
		{
			return &m_base;
		}

		// Access the shapes in the array. Use 'next(Shape*)' to increment the iterator
		Shape const* begin() const { return reinterpret_cast<Shape const*>(this + 1); }
		Shape*       begin()       { return reinterpret_cast<Shape*      >(this + 1); }
		Shape const* end() const   { return reinterpret_cast<Shape const*>(byte_ptr(this) + m_base.m_size); }
		Shape*       end()         { return reinterpret_cast<Shape*      >(byte_ptr(this) + m_base.m_size); }
		auto shapes() const
		{
			struct I
			{
				Shape const* ptr;
				operator Shape const* () const { return ptr; }
				auto& operator ++() { ptr = next(ptr); return *this; }
				bool operator != (I const& rhs) const { return ptr != rhs.ptr; }
			};
			struct R
			{
				Shape const* b;
				Shape const* e;
				auto begin() const { return I{ b }; }
				auto end() const { return I{ e }; }
			};
			return R{ begin(), end() };
		}
	};
	static_assert(ShapeType<ShapeArray>);
	static_assert((sizeof(ShapeArray) & 0xf) == 0);

	// Calculate the bounding box for the shape.
	inline BBox pr_vectorcall CalcBBox(ShapeArray const& shape)
	{
		auto bb = BBox::Reset();
		for (Shape const* i = shape.begin(), *i_end = shape.end(); i != i_end; i = next(i))
			Grow(bb, CalcBBox(*i));

		return bb;
	}

	// Shift the centre a shape. Updates 'shape.m_shape_to_model' and 'shift'
	inline void pr_vectorcall ShiftCentre(ShapeArray& shape, v4 shift)
	{
		(void)shape, shift;
		throw std::runtime_error("Not implemented");
	}

	// Returns the support vertex for 'shape' in 'direction'. 'direction' is in shape space
	inline v4 pr_vectorcall SupportVertex(ShapeArray const& shape, v4 direction, int hint_vert_id, int& sup_vert_id)
	{
		(void)shape, direction, hint_vert_id, sup_vert_id;
		throw std::runtime_error("Not implemented");
	}

	// Returns the closest point on 'shape' to 'point'. 'shape' and 'point' are in the same space
	inline void pr_vectorcall ClosestPoint(ShapeArray const& shape, v4 point, float& distance, v4& closest)
	{
		(void)shape, point, distance, closest;
		throw std::runtime_error("Not implemented");
	}
}