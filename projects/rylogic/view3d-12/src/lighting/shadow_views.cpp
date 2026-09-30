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

	// The smallest shadow view size used for point and spot lights that cover little of the screen (in pixels)
	static constexpr int MinShadowViewSize = 64;

	// A shadow view before its atlas region is known
	struct ShadowViewRequest
	{
		m4x4  m_w2s;        // Perspective views: the final world to clip transform. Orthographic views: the light orientation (rotation only)
		v4    m_centre;     // Orthographic views: centre of the sphere that the view covers
		float m_radius;     // Orthographic views: radius of the sphere that the view covers
		float m_bias_scale; // World size of the view across its width. For perspective views, per unit distance from the light
		int   m_size;       // Requested width and height (in pixels)
		bool  m_ortho;      // True for orthographic views
	};

	// Return the height (in pixels) that a sphere at 'centre' with 'radius' covers on screen. Returns the largest float if unknown or if the camera is inside the sphere.
	static float ScreenFootprint(ShadowCamera const& camera, v4 centre, float radius)
	{
		// Perspective views shrink with distance from the camera. Orthographic views do not.
		auto dist = Length(centre - camera.m_c2w.pos);
		if (camera.m_viewport_height <= 0 || (!camera.m_orthographic && dist <= radius))
			return std::numeric_limits<float>::max();

		return camera.m_viewport_height * 2 * radius / camera.ViewSizeAt(dist).y;
	}

	// Add the cascade views for a directional light. Views are left empty when no casters are within the shadow distance.
	// 'centre' and 'radius' describe the bounding sphere of the casters.
	static void DirectionalViews(Light const& light, v4 centre, float radius, ShadowCamera const& camera, ShadowSettings const& settings, pr::vector<ShadowViewRequest, 6>& views)
	{
		// The light orientation is fixed by its direction only, so that snapping to texels in light space is stable while the camera moves
		auto dir = Normalise(light.m_direction);
		auto rot = m4x4::LookAt(v4::Origin(), v4::Origin() + dir, Perpendicular(dir, v4::YAxis()));

		// Shadows are only needed over the range of view depths that contain casters, limited by the shadow distance
		auto cam_pos = camera.m_c2w.pos;
		auto cam_fwd = -camera.m_c2w.z;
		auto depth = Dot(centre - cam_pos, cam_fwd);
		auto zn = std::max(camera.m_near, depth - radius);
		auto zf = std::min(camera.m_far, depth + radius);
		if (settings.m_shadow_distance > 0)
			zf = std::min(zf, settings.m_shadow_distance);
		if (zf <= zn)
			return;

		// Split the depth range into cascades. Each cascade covers the bounding sphere of its slice of the camera view.
		auto count = std::clamp(settings.m_cascade_count, 1, MaxShadowCascades);
		float splits[MaxShadowCascades + 1];
		ShadowCascadeSplits(zn, zf, count, std::clamp(settings.m_cascade_split_blend, 0.0f, 1.0f), { &splits[0], s_cast<size_t>(count + 1) });
		for (int c = 0; c != count; ++c)
		{
			// The slice is a truncated pyramid with half-diagonals 'h0' and 'h1' at its ends. Place the sphere centre on the view axis
			// where it is equally far from the near and far corners, or at the far end if the far corners are always further.
			// The radius depends only on the slice depths and field of view, so it does not change as the camera rotates.
			auto s0 = splits[c];
			auto s1 = splits[c + 1];
			auto h0 = Length(camera.ViewSizeAt(s0) * 0.5f);
			auto h1 = Length(camera.ViewSizeAt(s1) * 0.5f);
			auto t = std::clamp((s1 * s1 - s0 * s0 + h1 * h1 - h0 * h0) / (2 * (s1 - s0)), s0, s1);
			auto r = std::sqrt(std::max(Sqr(t - s0) + Sqr(h0), Sqr(s1 - t) + Sqr(h1)));

			// Round the radius up to a quarter power of two. The texel size then only changes when the split distances change by a large amount.
			r = std::exp2(std::ceil(std::log2(r) * 4) / 4);

			// A cascade that would cover all of the casters is replaced by one fitted to the casters. Further cascades are not needed.
			if (r >= radius)
			{
				views.push_back({ .m_w2s = rot, .m_centre = centre, .m_radius = radius, .m_bias_scale = 2 * radius, .m_size = settings.m_directional_resolution, .m_ortho = true });
				break;
			}
			views.push_back({ .m_w2s = rot, .m_centre = cam_pos + t * cam_fwd, .m_radius = r, .m_bias_scale = 2 * r, .m_size = settings.m_directional_resolution, .m_ortho = true });
		}
	}

	// Return the transform for an orthographic view of the sphere described by 'req', rendered into a region 'size' pixels wide
	static m4x4 OrthographicViewTransform(ShadowViewRequest const& req, int size)
	{
		// Move the sphere centre, in light space, to a whole multiple of the texel size. The view's texel grid is then fixed in world space,
		// so shadow edges do not shimmer when the camera moves. The half-width is a whole number of texels because 'size' is even.
		auto const& rot = req.m_w2s;
		auto texel = 2 * req.m_radius / size;
		auto ls_centre = InvertOrthonormal(rot) * req.m_centre;
		ls_centre.x = std::floor(ls_centre.x / texel + 0.5f) * texel;
		ls_centre.y = std::floor(ls_centre.y / texel + 0.5f) * texel;

		// Look along the light direction from the front of the sphere. Casters in front of the sphere are clamped to depth 0 when rendering.
		auto l2w = rot;
		l2w.pos = rot * ls_centre + req.m_radius * rot.z;
		auto proj = m4x4::ProjectionOrthographic(2 * req.m_radius, 2 * req.m_radius, 0.0f, 2 * req.m_radius, true);
		return proj * InvertOrthonormal(l2w);
	}

	// Choose the shadow views for 'lights' and pack them into the atlas
	void BuildShadowViews(std::span<Light const> lights, BBox const& caster_bounds, ShadowCamera const& camera, ShadowSettings const& settings, ShadowViewSet& out)
	{
		pr::vector<ShadowViewRequest, MaxShadowViews, true> requests;
		out.reset(isize(lights));
		if (settings.m_max_shadow_lights <= 0 || !caster_bounds.valid())
			return;

		// Views are fitted to the bounding sphere of the casters. Keep the radius non-zero so projections stay valid.
		auto centre = caster_bounds.Centre();
		auto radius = std::max(Length(caster_bounds.Radius()), 0.001f);

		// Choose views for the shadow-casting lights in order until a limit is reached
		auto light_count = 0;
		for (int i = 0; i != isize(lights) && light_count != settings.m_max_shadow_lights; ++i)
		{
			auto const& light = lights[i];
			if (!light.CastsShadow())
				continue;

			// Point and spot lights that cannot reach any caster do not need shadows
			auto dist = light.m_type == ELight::Directional ? 0.0f : Length(centre - light.m_position);
			if (light.m_type != ELight::Directional && dist - radius >= light.m_range)
				continue;

			// Create the views for this light
			pr::vector<ShadowViewRequest, 6> views;
			switch (light.m_type)
			{
				case ELight::Directional:
				{
					// Orthographic cascades along the camera view
					DirectionalViews(light, centre, radius, camera, settings, views);
					break;
				}
				case ELight::Spot:
				{
					// A perspective view covering the spot cone, sized by the light's screen coverage.
					// The cone is widened slightly so filtering near the edge stays inside the view.
					auto dir = Normalise(light.m_direction);
					auto zf = std::min(light.m_range, dist + radius);
					auto zn = zf * 0.01f;
					auto size = std::clamp(s_cast<int>(std::min(ScreenFootprint(camera, light.m_position, zf), 1e6f)), MinShadowViewSize, settings.m_spot_resolution);
					auto tan_half = std::tan(std::min(light.m_outer_angle, math::DegreesToRadians(170.0f)) * 0.5f) * (1.0f + 8.0f / size);
					auto l2w = m4x4::LookAt(light.m_position, light.m_position + dir, Perpendicular(dir, v4::YAxis()));
					auto proj = m4x4::ProjectionPerspective(2 * zn * tan_half, 2 * zn * tan_half, zn, zf, true);
					views.push_back({ .m_w2s = proj * InvertOrthonormal(l2w), .m_bias_scale = 2 * tan_half, .m_size = size, .m_ortho = false });
					break;
				}
				case ELight::Point:
				{
					// Six 90 degree perspective views, one per cube face, sized by the light's screen coverage.
					// Faces are widened slightly so filtering at face edges stays inside the view.
					static v4 const face_dir[] = { v4::XAxis(), -v4::XAxis(), v4::YAxis(), -v4::YAxis(), v4::ZAxis(), -v4::ZAxis() };
					static v4 const face_up[] = { v4::YAxis(), v4::YAxis(), v4::ZAxis(), v4::ZAxis(), v4::YAxis(), v4::YAxis() };
					auto zf = std::min(light.m_range, dist + radius);
					auto zn = zf * 0.01f;
					auto size = std::clamp(s_cast<int>(std::min(ScreenFootprint(camera, light.m_position, zf), 1e6f)), MinShadowViewSize, settings.m_point_resolution);
					auto tan_half = 1.0f + 8.0f / size;
					auto proj = m4x4::ProjectionPerspective(2 * zn * tan_half, 2 * zn * tan_half, zn, zf, true);
					for (int f = 0; f != 6; ++f)
					{
						// Each face looks along its axis from the light position
						auto l2w = m4x4::LookAt(light.m_position, light.m_position + face_dir[f], face_up[f]);
						views.push_back({ .m_w2s = proj * InvertOrthonormal(l2w), .m_bias_scale = 2 * tan_half, .m_size = size, .m_ortho = false });
					}
					break;
				}
				default:
				{
					throw std::runtime_error("Unsupported light type");
				}
			}

			// Lights without views, or whose views do not fit in the frame's view limit, have no shadows
			if (views.empty() || isize(requests) + isize(views) > MaxShadowViews)
				continue;

			out.m_light_views[i] = iv2(isize(requests), isize(views));
			requests.insert(requests.end(), views.begin(), views.end());
			++light_count;
		}

		// Assign atlas regions for the views. Regions may be smaller than requested when the atlas is full.
		int sizes[MaxShadowViews];
		IRect rects[MaxShadowViews];
		for (int i = 0; i != isize(requests); ++i)
			sizes[i] = requests[i].m_size;

		PackShadowAtlas({ &sizes[0], requests.size() }, settings.m_atlas_size, { &rects[0], requests.size() });

		// Create the views. Orthographic views are snapped to texels of their final region size. The normal bias is a number of texels of that size.
		for (int i = 0; i != isize(lights); ++i)
		{
			auto first = out.m_light_views[i].x;
			auto count = out.m_light_views[i].y;
			for (int v = first; v != first + count; ++v)
			{
				auto const& req = requests[v];
				out.m_views.push_back(ShadowView{
					.m_w2s = req.m_ortho ? OrthographicViewTransform(req, rects[v].SizeX()) : req.m_w2s,
					.m_atlas_rect = rects[v],
					.m_normal_bias = req.m_bias_scale * settings.m_normal_bias / rects[v].SizeX(),
					.m_light_index = i,
					.m_face = v - first,
					.m_clamp_depth = req.m_ortho,
				});
			}
		}
	}

	// Return the distances from the camera to the cascade boundaries for 'count' cascades over [zn, zf]
	void ShadowCascadeSplits(float zn, float zf, int count, float blend, std::span<float> out)
	{
		// Even spacing gives the same depth range to each cascade. Logarithmic spacing gives each cascade the same ratio of far to near
		// distance, which keeps shadow texels a similar size on screen but can make the first cascade very short.
		pr_assert(zn > 0 && zf > zn && count >= 1 && isize(out) >= count + 1);
		out[0] = zn;
		for (int i = 1; i != count; ++i)
		{
			auto f = s_cast<float>(i) / count;
			auto even = zn + (zf - zn) * f;
			auto logarithmic = zn * std::pow(zf / zn, f);
			out[i] = Lerp(even, logarithmic, blend);
		}
		out[count] = zf;
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

	// Build the volume from the clip space defined by 'w2s'
	ShadowViewVolume::ShadowViewVolume(m4x4 const& w2s, bool near_plane)
		: m_planes()
		, m_plane_count(near_plane ? 6 : 5)
	{
		// The clip volume planes come from the rows of 'w2s': -w <= x <= w, -w <= y <= w, 0 <= z <= w. The near plane (0 <= z) is last so it can be skipped.
		auto rows = Transpose(w2s);
		m_planes[0] = rows.w + rows.x;
		m_planes[1] = rows.w - rows.x;
		m_planes[2] = rows.w + rows.y;
		m_planes[3] = rows.w - rows.y;
		m_planes[4] = rows.w - rows.z;
		m_planes[5] = rows.z;
	}

	// True if the world space box 'bbox' may be visible in the volume
	bool ShadowViewVolume::Sees(BBox const& bbox) const
	{
		// Test using the box corners
		return Sees(bbox.Lower(), bbox.Upper());
	}

	// True if the world space box with corners 'lower' and 'upper' may be visible in the volume
	bool ShadowViewVolume::Sees(v4 lower, v4 upper) const
	{
		// The box is outside if its corner furthest along a plane normal is behind that plane.
		// Scalar arithmetic keeps this test cheap in unoptimised builds, where it runs for every element and view.
		for (int i = 0; i != m_plane_count; ++i)
		{
			// Test the box corner that is furthest in the direction of the plane normal
			auto const& p = m_planes[i];
			auto d = p.w
				+ p.x * (p.x >= 0 ? upper.x : lower.x)
				+ p.y * (p.y >= 0 ? upper.y : lower.y)
				+ p.z * (p.z >= 0 ? upper.z : lower.z);
			if (d < 0)
				return false;
		}
		return true;
	}

	// Return a bit mask of the views whose atlas region does not already hold the same content, then record the new content
	uint32_t ShadowViewCache::Update(std::span<IRect const> rects, std::span<uint64_t const> hashes, bool enabled)
	{
		// A view is clean if the same region held the same content at the end of the last frame. Regions of one frame never overlap,
		// so a matching entry means nothing has drawn over that region since.
		pr_assert(rects.size() == hashes.size() && isize(rects) <= MaxShadowViews);
		auto dirty = uint32_t(0);
		for (int i = 0; i != isize(rects); ++i)
		{
			auto clean = enabled && std::any_of(m_entries.begin(), m_entries.end(), [&](Entry const& e)
			{
				return All(e.m_rect.m_min == rects[i].m_min) && All(e.m_rect.m_max == rects[i].m_max) && e.m_hash == hashes[i];
			});
			if (!clean)
				dirty |= 1U << i;
		}

		// Record the content of the atlas after this frame's views are rendered
		m_entries.resize(0);
		for (int i = 0; i != isize(rects); ++i)
			m_entries.push_back(Entry{ .m_rect = rects[i], .m_hash = hashes[i] });

		return dirty;
	}

	// Forget all atlas content, so every view is rendered next frame
	void ShadowViewCache::Invalidate()
	{
		m_entries.resize(0);
	}
}
