//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2016
//*********************************************
#pragma once
#include "pr/physics/forward.h"
#include "src/compute/physics_types.h"

namespace pr::physics
{
	struct ShapeCache
	{
		// Note:
		//  - Cached collision shape data that persists across frames.
		//  - Shapes are only re-packed when first seen or after being evicted.
		//  - When no shapes change between frames, the GPU upload can be skipped entirely.

		static constexpr int StaleFrameLimit = 10; // Evict shapes unused for this many frames

		struct Entry
		{
			int gpu_index;      // Index into m_shapes / m_verts arrays
			int vert_offset;    // For polytopes/triangles: offset into m_verts
			int vert_count;     // For polytopes/triangles: number of vertices
			int face_offset;    // For polytopes: offset into m_faces
			int face_count;     // For polytopes: number of faces
			int edge_offset;    // For polytopes: offset into m_edges
			int edge_count;     // For polytopes: number of edges
			int child_count;    // For compounds: number of flattened convex leaves that follow the root entry
			int last_used;      // Frame counter when last referenced
		};

		std::vector<GpuShape> m_shapes;                    // Packed GPU shapes
		std::vector<v4> m_verts;                           // Shared vertex buffer (polytope/triangle verts)
		std::vector<GpuPolytopeFace> m_faces;              // Shared polytope face buffer
		std::vector<GpuPolytopeEdge> m_edges;              // Shared polytope edge buffer
		std::unordered_map<Shape const*, Entry> m_entries; // Shape pointer → cache entry
		int m_frame;                                       // Current frame counter
		bool m_changed;                                    // True if shapes were added/removed since last upload
		bool m_orphaned;                                   // True if evicted shapes left unreferenced data in the packed arrays

		ShapeCache()
			: m_shapes()
			, m_verts()
			, m_faces()
			, m_edges()
			, m_entries()
			, m_frame(0)
			, m_changed(true)
			, m_orphaned(false)
		{}

		// Begin a new frame. Must be called before any GetOrAdd() calls.
		void BeginFrame()
		{
			++m_frame;

			// Periodically flush stale shapes (every 10 frames)
			if (m_frame % StaleFrameLimit == 0)
				Flush();
		}

		// Drop all cached shapes and entries. Call after any caller-owned Shape that has
		// been passed to GetOrAdd() may have been destroyed — entry keys are raw Shape
		// pointers and the cache cannot detect dangling pointers itself. The next
		// GetOrAdd() will rebuild from scratch and will set m_changed so the GPU
		// re-uploads the new shape buffer.
		void Reset()
		{
			m_shapes.clear();
			m_verts.clear();
			m_faces.clear();
			m_edges.clear();
			m_entries.clear();
			m_frame = 0;
			m_changed = true;
			m_orphaned = false;
		}

		// Forget 'shape' before its caller destroys it, so a later shape at the same address is packed from its own data.
		void Evict(Shape const& shape)
		{
			// The packed data of other entries keeps its indices. The evicted data stays in the arrays, unreferenced, until the next Flush compacts them.
			if (m_entries.erase(&shape) != 0)
				m_orphaned = true;
		}

		// Get the GPU shape index for a collision shape, adding it to the cache if new.
		// Returns the index into m_shapes.
		int GetOrAdd(Shape const& shape)
		{
			auto it = m_entries.find(&shape);
			if (it != m_entries.end())
			{
				it->second.last_used = m_frame;
				return it->second.gpu_index;
			}

			// New shape — pack it into the GPU buffers. Compound shapes also append their flattened
			// convex leaves, which are immutable for the lifetime of the cache entry, so per-child data
			// is built once rather than per step.
			auto vert_offset = static_cast<int>(m_verts.size());
			auto face_offset = static_cast<int>(m_faces.size());
			auto edge_offset = static_cast<int>(m_edges.size());
			auto idx = PackShapeTree(shape, m_shapes, m_verts, &m_faces, &m_edges);
			auto vert_count = static_cast<int>(m_verts.size()) - vert_offset;
			auto face_count = static_cast<int>(m_faces.size()) - face_offset;
			auto edge_count = static_cast<int>(m_edges.size()) - edge_offset;

			m_entries[&shape] = Entry{
				.gpu_index = idx,
				.vert_offset = vert_offset,
				.vert_count = vert_count,
				.face_offset = face_offset,
				.face_count = face_count,
				.edge_offset = edge_offset,
				.edge_count = edge_count,
				.child_count = m_shapes[idx].child_count,
				.last_used = m_frame,
			};

			m_changed = true;
			return idx;
		}

		// Update the packed transform and bounds of a cached leaf shape after its 'm_s2r' changed in place.
		// Shapes not in the cache need nothing, because they are packed from current data when first seen.
		void RefreshTransform(Shape const& shape)
		{
			// Only the transform-derived fields change, so the entry keeps its slot and its vertex, face, and edge ranges.
			assert(shape.m_type != collision::EShape::Array && "Compound shapes keep absolute child transforms; refresh is for leaf shapes");
			auto it = m_entries.find(&shape);
			if (it == m_entries.end())
				return;

			auto& gpu = m_shapes[it->second.gpu_index];
			gpu.s2rb = shape.m_s2r;
			gpu.rb_bbox = shape.m_s2r * shape.m_bbox;
			m_changed = true;
		}

		// Evict shapes that haven't been used for StaleFrameLimit frames.
		// This is called periodically (not every frame) to keep the cache compact.
		// After eviction, the shape/vert arrays are rebuilt from surviving entries, which also drops the data of shapes removed by 'Evict'.
		void Flush()
		{
			// Check if any entries are stale
			bool has_stale = m_orphaned;
			for (auto& [ptr, entry] : m_entries)
			{
				if (m_frame - entry.last_used > StaleFrameLimit)
				{
					has_stale = true;
					break;
				}
			}
			if (!has_stale) return;

			// Rebuild: remove stale entries, repack survivors
			auto old_entries = std::move(m_entries);
			m_entries.clear();
			m_shapes.clear();
			m_verts.clear();
			m_faces.clear();
			m_edges.clear();

			for (auto& [ptr, entry] : old_entries)
			{
				if (m_frame - entry.last_used > StaleFrameLimit)
					continue;

				// Re-pack this shape at its new index
				auto new_vert_offset = static_cast<int>(m_verts.size());
				auto new_face_offset = static_cast<int>(m_faces.size());
				auto new_edge_offset = static_cast<int>(m_edges.size());
				auto new_idx = PackShapeTree(*ptr, m_shapes, m_verts, &m_faces, &m_edges);
				auto new_vert_count = static_cast<int>(m_verts.size()) - new_vert_offset;
				auto new_face_count = static_cast<int>(m_faces.size()) - new_face_offset;
				auto new_edge_count = static_cast<int>(m_edges.size()) - new_edge_offset;

				m_entries[ptr] = Entry{
					.gpu_index = new_idx,
					.vert_offset = new_vert_offset,
					.vert_count = new_vert_count,
					.face_offset = new_face_offset,
					.face_count = new_face_count,
					.edge_offset = new_edge_offset,
					.edge_count = new_edge_count,
					.child_count = m_shapes[new_idx].child_count,
					.last_used = entry.last_used,
				};
			}
			m_changed = true;
			m_orphaned = false;
		}
	};
}
