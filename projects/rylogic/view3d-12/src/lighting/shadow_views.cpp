//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#include "pr/view3d-12/lighting/shadow_view.h"

namespace pr::rdr12
{
	// Reset to no views for 'light_count' lights
	void ShadowViewSet::reset(int light_count)
	{
		// Every light starts without shadows
		m_views.resize(0);
		m_light_views.resize(0);
		m_light_views.resize(light_count, iv2(-1, 0));
	}

	// Choose the shadow views for 'lights' and pack them into the atlas
	void BuildShadowViews(std::span<Light const> lights, BBox const& caster_bounds, ShadowSettings const& settings, ShadowViewSet& out)
	{
		// A view is described by its transform and requested resolution until the atlas regions are known
		struct ViewRequest
		{
			m4x4  m_w2s;
			float m_bias_scale; // World size of the view across its width, per unit distance for perspective views
			int   m_size;
		};
		pr::vector<ViewRequest, MaxShadowViews, true> requests;
		out.reset(isize(lights));
		if (settings.m_max_shadow_lights <= 0 || !caster_bounds.valid())
			return;

		// All views are fitted to the bounding sphere of the casters. Keep the radius non-zero so projections stay valid.
		auto centre = caster_bounds.Centre();
		auto radius = std::max(Length(caster_bounds.Radius()), 0.001f);

		// Choose views for the shadow-casting lights in order until a limit is reached
		auto light_count = 0;
		for (int i = 0; i != isize(lights) && light_count != settings.m_max_shadow_lights; ++i)
		{
			auto const& light = lights[i];
			if (!light.CastsShadow())
				continue;

			// Lights need one view, except point lights which need one for each cube face
			auto view_count = light.m_type == ELight::Point ? 6 : 1;
			if (isize(requests) + view_count > MaxShadowViews)
				continue;

			// Point and spot lights that cannot reach any caster do not need shadows
			auto dist = light.m_type == ELight::Directional ? 0.0f : Length(centre - light.m_position);
			if (light.m_type != ELight::Directional && dist - radius >= light.m_range)
				continue;

			// Create the views for this light
			out.m_light_views[i] = iv2(isize(requests), view_count);
			switch (light.m_type)
			{
				case ELight::Directional:
				{
					// An orthographic view from outside the caster sphere that covers the whole sphere
					auto dir = Normalise(light.m_direction);
					auto l2w = m4x4::LookAt(centre - radius * dir, centre, Perpendicular(dir, v4::YAxis()));
					auto proj = m4x4::ProjectionOrthographic(2 * radius, 2 * radius, 0.0f, 2 * radius, true);
					requests.push_back({ proj * InvertOrthonormal(l2w), 2 * radius, settings.m_directional_resolution });
					break;
				}
				case ELight::Spot:
				{
					// A perspective view covering the spot cone. The cone is widened slightly so filtering near the edge stays inside the view.
					auto dir = Normalise(light.m_direction);
					auto zf = std::min(light.m_range, dist + radius);
					auto zn = zf * 0.01f;
					auto tan_half = std::tan(std::min(light.m_outer_angle, math::DegreesToRadians(170.0f)) * 0.5f) * (1.0f + 4.0f / settings.m_spot_resolution);
					auto l2w = m4x4::LookAt(light.m_position, light.m_position + dir, Perpendicular(dir, v4::YAxis()));
					auto proj = m4x4::ProjectionPerspective(2 * zn * tan_half, 2 * zn * tan_half, zn, zf, true);
					requests.push_back({ proj * InvertOrthonormal(l2w), 2 * tan_half, settings.m_spot_resolution });
					break;
				}
				case ELight::Point:
				{
					// Six 90 degree perspective views, one per cube face. Faces are widened slightly so filtering at face edges stays inside the view.
					static v4 const face_dir[] = { v4::XAxis(), -v4::XAxis(), v4::YAxis(), -v4::YAxis(), v4::ZAxis(), -v4::ZAxis() };
					static v4 const face_up[] = { v4::YAxis(), v4::YAxis(), v4::ZAxis(), v4::ZAxis(), v4::YAxis(), v4::YAxis() };
					auto zf = std::min(light.m_range, dist + radius);
					auto zn = zf * 0.01f;
					auto tan_half = 1.0f + 4.0f / settings.m_point_resolution;
					auto proj = m4x4::ProjectionPerspective(2 * zn * tan_half, 2 * zn * tan_half, zn, zf, true);
					for (int f = 0; f != 6; ++f)
					{
						// Each face looks along its axis from the light position
						auto l2w = m4x4::LookAt(light.m_position, light.m_position + face_dir[f], face_up[f]);
						requests.push_back({ proj * InvertOrthonormal(l2w), 2 * tan_half, settings.m_point_resolution });
					}
					break;
				}
				default:
				{
					throw std::runtime_error("Unsupported light type");
				}
			}
			++light_count;
		}

		// Assign atlas regions for the views. Regions may be smaller than requested when the atlas is full.
		int sizes[MaxShadowViews];
		IRect rects[MaxShadowViews];
		for (int i = 0; i != isize(requests); ++i)
			sizes[i] = requests[i].m_size;

		PackShadowAtlas({ &sizes[0], requests.size() }, settings.m_atlas_size, { &rects[0], requests.size() });

		// Create the views. The normal bias is a number of texels of the final region size.
		for (int i = 0; i != isize(lights); ++i)
		{
			auto first = out.m_light_views[i].x;
			auto count = out.m_light_views[i].y;
			for (int v = first; v != first + count; ++v)
			{
				auto const& req = requests[v];
				out.m_views.push_back(ShadowView{
					.m_w2s = req.m_w2s,
					.m_atlas_rect = rects[v],
					.m_normal_bias = req.m_bias_scale * settings.m_normal_bias / rects[v].SizeX(),
					.m_light_index = i,
					.m_face = v - first,
				});
			}
		}
	}

	// Assign square atlas regions for views with the requested sizes
	void PackShadowAtlas(std::span<int const> requested_sizes, int atlas_size, std::span<IRect> out)
	{
		// Views are square with power-of-two sizes. Placing them in order of decreasing size at consecutive positions along a Z-order
		// curve (a curve that visits the quadrants of each square in turn) packs them without gaps, because each view then starts at a
		// position aligned to its own size.
		pr_assert(std::has_single_bit(static_cast<uint32_t>(atlas_size)) && "Atlas size must be a power of two");
		pr_assert(out.size() >= requested_sizes.size());
		auto count = isize(requested_sizes);

		// Round the requests to powers of two that fit in the atlas
		int sizes[MaxShadowViews];
		pr_assert(count <= MaxShadowViews);
		for (int i = 0; i != count; ++i)
			sizes[i] = static_cast<int>(std::bit_floor(static_cast<uint32_t>(std::clamp(requested_sizes[i], 1, atlas_size))));

		// Halve the largest views until the total area fits. Halving all views of the largest size keeps equal requests equal.
		for (;;)
		{
			auto area = int64_t(0);
			auto largest = 0;
			for (int i = 0; i != count; ++i)
			{
				area += int64_t(sizes[i]) * sizes[i];
				largest = std::max(largest, sizes[i]);
			}
			if (area <= int64_t(atlas_size) * atlas_size || largest == 1)
				break;

			for (int i = 0; i != count; ++i)
				sizes[i] = sizes[i] == largest ? largest / 2 : sizes[i];
		}

		// Order the views by decreasing size. Ties keep request order so the result is deterministic.
		int order[MaxShadowViews];
		std::iota(&order[0], &order[0] + count, 0);
		std::stable_sort(&order[0], &order[0] + count, [&](int l, int r) { return sizes[l] > sizes[r]; });

		// Place each view at the next position along the Z-order curve
		auto offset = uint32_t(0);
		for (int i = 0; i != count; ++i)
		{
			// Split the curve position into x (even bits) and y (odd bits)
			auto idx = order[i];
			auto x = 0, y = 0;
			for (int b = 0; b != 16; ++b)
			{
				x |= static_cast<int>((offset >> (2 * b + 0)) & 1) << b;
				y |= static_cast<int>((offset >> (2 * b + 1)) & 1) << b;
			}
			out[idx] = IRect(x, y, x + sizes[idx], y + sizes[idx]);
			offset += static_cast<uint32_t>(sizes[idx]) * static_cast<uint32_t>(sizes[idx]);
		}
	}

	// Return the point light cube face index (+X,-X,+Y,-Y,+Z,-Z) that contains the direction 'light_to_point'
	int ShadowCubeFace(v4 light_to_point)
	{
		// The face is the dominant axis and its sign. Must match 'ShadowCubeFace' in shadow_cast.hlsli.
		auto a = Abs(light_to_point);
		if (a.x >= a.y && a.x >= a.z) return light_to_point.x >= 0 ? 0 : 1;
		if (a.y >= a.z) return light_to_point.y >= 0 ? 2 : 3;
		return light_to_point.z >= 0 ? 4 : 5;
	}

	// True if the world space box 'bbox' may be visible in the clip space defined by 'w2s'
	bool ShadowViewSees(m4x4 const& w2s, BBox const& bbox)
	{
		// The clip volume planes come from the rows of 'w2s': -w <= x <= w, -w <= y <= w, 0 <= z <= w.
		// The box is outside if its corner furthest along a plane normal is behind that plane.
		auto rows = Transpose(w2s);
		v4 const planes[] =
		{
			rows.w + rows.x,
			rows.w - rows.x,
			rows.w + rows.y,
			rows.w - rows.y,
			rows.z,
			rows.w - rows.z,
		};
		auto lower = bbox.Lower();
		auto upper = bbox.Upper();
		for (auto const& plane : planes)
		{
			// Test the box corner that is furthest in the direction of the plane normal
			auto corner = v4(
				plane.x >= 0 ? upper.x : lower.x,
				plane.y >= 0 ? upper.y : lower.y,
				plane.z >= 0 ? upper.z : lower.z,
				1.0f);
			if (Dot(plane, corner) < 0)
				return false;
		}
		return true;
	}
}
