//************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2025
//************************************
// A CPU-authored cloud cover field used by the procedural sky.
#pragma once
#include "pr/view3d-12/forward.h"

namespace pr::rdr12
{
	// A grid of cloud cover values in [0,1] over an XY rectangle of the sky frame, anchored in world space.
	// Brushes edit the CPU copy, and 'Upload' copies it to the GPU texture that the procedural sky samples.
	// Over the outer 10% of the area at each edge, cover fades to the sky's default cover (see 'CoverAt'). Use on the renderer owner thread.
	struct WeatherMap : RefCounted<WeatherMap>
	{
		Renderer* m_rdr;
		int m_width;
		int m_height;
		v2 m_area_min;
		v2 m_area_max;
		std::vector<float> m_cover;
		Texture2DPtr m_tex;
		std::unique_ptr<ResourceFactory> m_factory;

		// Create a 'width' x 'height' grid of zero cover over the area [area_min, area_max).
		WeatherMap(Renderer& rdr, int width, int height, v2 area_min, v2 area_max);
		WeatherMap(WeatherMap const&) = delete;
		WeatherMap& operator=(WeatherMap const&) = delete;
		~WeatherMap();

		// Move the area that the grid covers, e.g. to keep it centred on the camera. The grid contents are unchanged.
		void Area(v2 area_min, v2 area_max);

		// Set every cell to 'cover'.
		void Fill(float cover);

		// Blend toward 'cover' within a circle. Weight is 1 inside 40% of 'radius' and falls smoothly to 0 at 'radius'.
		void AddStormCell(v2 centre, float radius, float cover);

		// Blend toward 'cover' behind a straight front through 'point'. 'travel_direction' points from the covered side to the uncovered side.
		// The weight changes smoothly over 'width' centred on the front line; a zero width gives a hard edge.
		void AddFront(v2 point, v2 travel_direction, float width, float cover);

		// Add smooth random variation in [-amplitude, +amplitude]. 'scale' is the feature size in world units. The result is clamped to [0,1].
		void AddNoise(float scale, float amplitude, uint32_t seed);

		// Return the cover at 'position', bilinearly filtered and faded to 'default_cover' toward and beyond the area edges, matching the shader.
		float CoverAt(v2 position, float default_cover) const;

		// Copy the CPU grid to the GPU texture. The copy is queued before later frames, so it is visible from the next render.
		void Upload();

		// Ref-counting clean up function
		static void RefCountZero(RefCounted<WeatherMap>* doomed);

	private:

		// Blend each cell toward 'cover' by the weight returned by 'weight(world_position)'.
		template <typename WeightFn> void Blend(float cover, WeightFn weight);
	};

	// Throw if 'weather' is null
	void Validate(WeatherMap const* weather);
}
