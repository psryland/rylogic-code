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
			int last_used;      // Frame counter when last referenced
		};

		std::vector<GpuShape> m_shapes;                    // Packed GPU shapes
		std::vector<v4> m_verts;                           // Shared vertex buffer (polytope/triangle verts)
		std::vector<GpuPolytopeFace> m_faces;              // Shared polytope face buffer
		std::vector<GpuPolytopeEdge> m_edges;              // Shared polytope edge buffer
		std::unordered_map<Shape const*, Entry> m_entries; // Shape pointer → cache entry
		int m_frame;                                       // Current frame counter
		bool m_changed;                                    // True if shapes were added/removed since last upload

		ShapeCache()
			: m_shapes()
			, m_verts()
			, m_faces()
			, m_edges()
			, m_entries()
			, m_frame(0)
			, m_changed(true)
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

			// New shape — pack it into the GPU buffers
			auto idx = static_cast<int>(m_shapes.size());
			auto vert_offset = static_cast<int>(m_verts.size());
			auto face_offset = static_cast<int>(m_faces.size());
			auto edge_offset = static_cast<int>(m_edges.size());
			m_shapes.push_back(PackShape(shape, m_verts, &m_faces, &m_edges));
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
				.last_used = m_frame,
			};

			m_changed = true;
			return idx;
		}

		// Evict shapes that haven't been used for StaleFrameLimit frames.
		// This is called periodically (not every frame) to keep the cache compact.
		// After eviction, the shape/vert arrays are rebuilt from surviving entries.
		void Flush()
		{
			// Check if any entries are stale
			bool has_stale = false;
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
				auto new_idx = static_cast<int>(m_shapes.size());
				auto new_vert_offset = static_cast<int>(m_verts.size());
				auto new_face_offset = static_cast<int>(m_faces.size());
				auto new_edge_offset = static_cast<int>(m_edges.size());
				m_shapes.push_back(PackShape(*ptr, m_verts, &m_faces, &m_edges));
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
					.last_used = entry.last_used,
				};
			}
			m_changed = true;
		}
	};
}
