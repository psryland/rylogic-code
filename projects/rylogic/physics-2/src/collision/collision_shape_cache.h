//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2016
//*********************************************
#pragma once
#include "pr/physics-2/forward.h"
#include "src/collision/gpu_collision_types.h"

namespace pr::physics
{
	struct CollisionShapeCache
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
			int last_used;      // Frame counter when last referenced
		};

		std::vector<GpuShape> m_shapes;                    // Packed GPU shapes
		std::vector<v4> m_verts;                           // Shared vertex buffer (polytope/triangle verts)
		std::unordered_map<Shape const*, Entry> m_entries; // Shape pointer → cache entry
		int m_frame;                                       // Current frame counter
		bool m_dirty;                                      // True if shapes were added/removed since last upload

		CollisionShapeCache()
			: m_shapes()
			, m_verts()
			, m_entries()
			, m_frame(0)
			, m_dirty(true)
		{}

		// Begin a new frame. Must be called before any GetOrAdd() calls.
		void BeginFrame()
		{
			++m_frame;
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
			m_shapes.push_back(PackShapeGeneric(shape, m_verts));
			auto vert_count = static_cast<int>(m_verts.size()) - vert_offset;

			m_entries[&shape] = Entry{
				.gpu_index = idx,
				.vert_offset = vert_offset,
				.vert_count = vert_count,
				.last_used = m_frame,
			};
			m_dirty = true;
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

			for (auto& [ptr, entry] : old_entries)
			{
				if (m_frame - entry.last_used > StaleFrameLimit)
					continue;

				// Re-pack this shape at its new index
				auto new_idx = static_cast<int>(m_shapes.size());
				auto new_vert_offset = static_cast<int>(m_verts.size());
				m_shapes.push_back(PackShapeGeneric(*ptr, m_verts));
				auto new_vert_count = static_cast<int>(m_verts.size()) - new_vert_offset;

				m_entries[ptr] = Entry{
					.gpu_index = new_idx,
					.vert_offset = new_vert_offset,
					.vert_count = new_vert_count,
					.last_used = entry.last_used,
				};
			}
			m_dirty = true;
		}

		// Returns true if shapes have changed since the last upload.
		// Call ClearDirty() after uploading to reset.
		bool IsDirty() const
		{
			return m_dirty;
		}

		void ClearDirty()
		{
			m_dirty = false;
		}
	};
}
