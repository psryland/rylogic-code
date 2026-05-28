//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#pragma once
#include "pr/physics/forward.h"
#include "pr/physics/integrator/engine.h"

namespace pr::physics
{
	constexpr float AnalyticWaterLevel = 0.0f;
	constexpr float AnalyticFluidDensity = 1000.0f;
	constexpr v4 AnalyticGravityWS = v4{0.0f, 0.0f, -9.81f, 0.0f};

	struct VolumeCentroid
	{
		float m_volume_m3;
		v4 m_centroid_ws;
		bool m_valid;
	};
	
	// Return true if 'point' is on the submerged side of the current flat water plane.
	inline bool IsSubmerged(v4 point, float water_level)
	{
		return point.z <= water_level;
	}

	// Return the point where an edge crosses the flat water plane.
	inline v4 IntersectWaterPlane(v4 a, v4 b, float water_level)
	{
		auto const t = (water_level - a.z) / (b.z - a.z);
		return a + (b - a) * t;
	}


	// Clip a convex polygon against the submerged half-space of the flat water plane.
	inline std::vector<v4> ClipPolygonToWater(std::span<v4 const> polygon, float water_level)
	{
		auto clipped = std::vector<v4>{};
		if (polygon.empty())
			return clipped;

		auto prev = polygon.back();
		auto prev_inside = IsSubmerged(prev, water_level);
		for (auto const curr : polygon)
		{
			auto const curr_inside = IsSubmerged(curr, water_level);
			if (curr_inside != prev_inside)
				clipped.push_back(IntersectWaterPlane(prev, curr, water_level));

			if (curr_inside)
				clipped.push_back(curr);

			prev = curr;
			prev_inside = curr_inside;
		}

		return clipped;
	}

	// Append 'point' if it is not already present within the analytic clipping tolerance.
	inline void AddUniquePoint(std::vector<v4>& points, v4 point)
	{
		for (auto const& existing : points)
		{
			if (Length(point - existing) < 1e-5f)
				return;
		}

		points.push_back(point);
	}

	// Sort the waterline cap vertices around the vertical water-plane normal.
	inline void SortWaterCap(std::vector<v4>& cap)
	{
		auto centre = v4::Zero();
		for (auto const& point : cap)
			centre += point;

		centre /= static_cast<float>(cap.size());
		std::sort(std::begin(cap), std::end(cap), [centre](v4 lhs, v4 rhs)
		{
			auto const lhs_angle = std::atan2(lhs.y - centre.y, lhs.x - centre.x);
			auto const rhs_angle = std::atan2(rhs.y - centre.y, rhs.x - centre.x);
			return lhs_angle < rhs_angle;
		});
	}

	// Return the volume and centroid of the submerged half-space clipped generated box.
	inline VolumeCentroid SubmergedBoxVolumeCentroid(m4x4 const& o2w, v4 half_extents, float water_level)
	{
		auto const hx = half_extents.x;
		auto const hy = half_extents.y;
		auto const hz = half_extents.z;
		auto const corners = std::vector<v4>
		{
			o2w * v4{-hx, -hy, -hz, 1.0f},
			o2w * v4{+hx, -hy, -hz, 1.0f},
			o2w * v4{+hx, +hy, -hz, 1.0f},
			o2w * v4{-hx, +hy, -hz, 1.0f},
			o2w * v4{-hx, -hy, +hz, 1.0f},
			o2w * v4{+hx, -hy, +hz, 1.0f},
			o2w * v4{+hx, +hy, +hz, 1.0f},
			o2w * v4{-hx, +hy, +hz, 1.0f},
		};

		auto submerged_count = 0;
		for (auto const& corner : corners)
		{
			if (IsSubmerged(corner, water_level))
				++submerged_count;
		}

		if (submerged_count == 0)
			return VolumeCentroid{};

		auto const full_volume = 8.0f * hx * hy * hz;
		if (submerged_count == isize(corners))
			return VolumeCentroid{ full_volume, o2w * v4::Origin(), true };

		static constexpr int BoxFaces[6][4] =
		{
			{0, 3, 2, 1},
			{4, 5, 6, 7},
			{0, 1, 5, 4},
			{1, 2, 6, 5},
			{2, 3, 7, 6},
			{3, 0, 4, 7},
		};
		static constexpr int BoxEdges[12][2] =
		{
			{0, 1}, {1, 2}, {2, 3}, {3, 0},
			{4, 5}, {5, 6}, {6, 7}, {7, 4},
			{0, 4}, {1, 5}, {2, 6}, {3, 7},
		};

		auto faces = std::vector<std::vector<v4>>{};
		faces.reserve(7);
		for (auto const& face_indices : BoxFaces)
		{
			auto face = std::vector<v4>
			{
				corners[face_indices[0]],
				corners[face_indices[1]],
				corners[face_indices[2]],
				corners[face_indices[3]],
			};
			auto clipped = ClipPolygonToWater(face, water_level);
			if (isize(clipped) >= 3)
				faces.push_back(std::move(clipped));
		}

		auto cap = std::vector<v4>{};
		for (auto const& edge_indices : BoxEdges)
		{
			auto const a = corners[edge_indices[0]];
			auto const b = corners[edge_indices[1]];
			auto const a_inside = IsSubmerged(a, water_level);
			auto const b_inside = IsSubmerged(b, water_level);
			if (a_inside != b_inside)
				AddUniquePoint(cap, IntersectWaterPlane(a, b, water_level));
		}
		if (isize(cap) >= 3)
		{
			SortWaterCap(cap);
			faces.push_back(std::move(cap));
		}

		auto vertices = std::vector<v4>{};
		for (auto const& face : faces)
		{
			for (auto const& point : face)
				AddUniquePoint(vertices, point);
		}
		if (vertices.empty())
			return VolumeCentroid{};

		auto reference = v4::Zero();
		for (auto const& point : vertices)
			reference += point;

		reference /= static_cast<float>(vertices.size());

		auto volume = 0.0f;
		auto centroid_sum = v4::Zero();
		for (auto const& face : faces)
		{
			auto const& a = face[0];
			for (int i = 1, iend = isize(face) - 1; i != iend; ++i)
			{
				auto const b = face[i];
				auto const c = face[i + 1];
				auto tetra_volume = std::abs(Dot3(a - reference, Cross(b - reference, c - reference))) / 6.0f;
				if (tetra_volume == 0.0f)
					continue;

				auto const tetra_centroid = (reference + a + b + c) * 0.25f;
				volume += tetra_volume;
				centroid_sum += tetra_centroid * tetra_volume;
			}
		}

		if (volume <= 1e-6f)
			return VolumeCentroid{};

		return VolumeCentroid{ volume, centroid_sum / volume, true };
	}
}
