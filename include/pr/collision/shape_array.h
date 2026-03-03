//*********************************************
// Collision
//  Copyright (C) Rylogic Ltd 2016
//*********************************************
#pragma once
#include "pr/collision/shape.h"

namespace pr::collision
{
	// Shape array
	struct ShapeArray
	{
		Shape m_base;

		// The number of shapes in the array
		size_t m_num_shapes;

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
			for (auto i = num_shapes; i-- != 0; ptr = next(ptr)) {}

			// Calculate the bounding box
			auto bb = BBox::Reset();
			for (Shape const* i = begin(), *i_end = end(); i != i_end; i = next(i))
				Grow(bb, i->m_s2p * i->m_bbox);

			m_num_shapes = num_shapes;
			m_base.m_size = sizeof(ShapeArray) + byte_ptr(ptr) - byte_ptr(begin());
			m_base.m_bbox = CalcBBox(*this);
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
	};
	static_assert(is_shape_v<ShapeArray>);

	// Calculate the bounding box for the shape.
	template <typename>
	BBox pr_vectorcall CalcBBox(ShapeArray const& shape)
	{
		auto bb = BBox::Reset();
		for (Shape const* i = shape.begin(), *i_end = shape.end(); i != i_end; i = next(i))
			Grow(bb, CalcBBox(*i));

		return bb;
	}
}