//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2026
//*********************************************
// Screen-space post-processing effects applied to a scene's composited colour. See postprocessing.md for details.
#pragma once
#include "pr/view3d-12/forward.h"
#include "pr/view3d-12/texture/texture_2d.h"

namespace pr::rdr12
{
	// Whole-screen "looking through water" effect: a colour tint, distance fog below the water surface, and a moving distortion.
	// Without a surface plane, the caller decides when the camera is submerged and the whole view is treated as in water.
	// With a surface plane, the effect applies only to pixels whose point on the camera's near plane is below the surface.
	struct UnderwaterProps
	{
		// Number of waterline offset samples along each edge of the near plane. See 'm_waterline_offsets'.
		static constexpr int WaterlineGridSize = 8;

		bool m_enabled = false;

		// sRGB colour multiplied into the scene colour. Alpha is ignored.
		Colour32 m_tint = Colour32{0xFFA6D9F2U};

		// sRGB colour that distant surfaces fade towards. Alpha is the fog strength, the largest fraction of the scene colour the fog replaces.
		Colour32 m_fog_colour = Colour32{0xFF0A384DU};

		// Distance (in world units) at which the fog hides 95% of a surface. Pixels without scene geometry are fully fogged.
		// Positive infinity disables the fog, for callers that fog the scene in their own shaders.
		float m_visibility = 40.0f;

		// Largest screen offset of the distortion, as a fraction of the viewport height. Zero disables the distortion.
		float m_distortion_amplitude = 0.002f;

		// Number of distortion ripples per viewport height.
		float m_distortion_frequency = 6.0f;

		// Distortion animation rate, in cycles per second.
		float m_distortion_speed = 0.25f;

		// World-space water surface plane: xyz is the normal pointing out of the water, and Dot(m_surface, point) > 0 above the water.
		// Pixels whose near-plane point is above this plane keep the scene colour, so a camera part-way through the surface shows a
		// waterline. The pass is skipped entirely while the whole near plane is above the surface. Below the surface, fog applies only
		// to the part of each view ray that is in water, so surfaces seen up through the water surface stay visible.
		// Zero means there is no surface and the whole view is in water.
		v4 m_surface = v4::Zero();

		// Depth below the surface (in world units) over which the effect fades in. A pixel's effect strength rises smoothly from none where
		// its near-plane point is at the surface to full where that point is this deep. Zero gives a sharp waterline, smoothed over about
		// one pixel. Ignored without a surface.
		float m_fade_depth = 0.0f;

		// Heights of the real water surface above 'm_surface', measured along its normal, at a regular grid of points on the camera's near plane.
		// Sample (i, j) is at index 'j * WaterlineGridSize + i' and at viewport-normalised position (i, j) / (WaterlineGridSize - 1), where (0, 0)
		// is the top-left corner. The waterline uses the surface height interpolated between the samples, so it can follow waves that a single plane
		// cannot. Fog still uses the plane. All zeros means the surface is the plane. Ignored without a surface.
		std::array<float, WaterlineGridSize * WaterlineGridSize> m_waterline_offsets = {};

		// Reject invalid settings without changing the current scene settings.
		void Validate() const
		{
			if (std::isnan(m_visibility) || m_visibility <= 0.0f)
				throw std::invalid_argument("Underwater visibility must be greater than zero, or positive infinity for no fog");
			if (!std::isfinite(m_distortion_amplitude) || m_distortion_amplitude < 0.0f)
				throw std::invalid_argument("Underwater distortion amplitude must be finite and not negative");
			if (!std::isfinite(m_distortion_frequency) || m_distortion_frequency <= 0.0f)
				throw std::invalid_argument("Underwater distortion frequency must be finite and greater than zero");
			if (!std::isfinite(m_distortion_speed) || m_distortion_speed < 0.0f)
				throw std::invalid_argument("Underwater distortion speed must be finite and not negative");
			if (!IsFinite(m_surface) || (Any(m_surface != v4::Zero()) && LengthSq(m_surface.w0()) == 0.0f))
				throw std::invalid_argument("Underwater surface must be finite, and either zero or have a non-zero normal");
			if (!std::isfinite(m_fade_depth) || m_fade_depth < 0.0f)
				throw std::invalid_argument("Underwater fade depth must be finite and not negative");
			if (!std::ranges::all_of(m_waterline_offsets, [](float offset) { return std::isfinite(offset); }))
				throw std::invalid_argument("Underwater waterline offsets must be finite");
		}

		// Compare all settings, including those retained while disabled.
		friend bool operator == (UnderwaterProps const& lhs, UnderwaterProps const& rhs)
		{
			// Vector comparisons are per component, so the surface needs all components to match.
			return
				lhs.m_enabled == rhs.m_enabled &&
				lhs.m_tint == rhs.m_tint &&
				lhs.m_fog_colour == rhs.m_fog_colour &&
				lhs.m_visibility == rhs.m_visibility &&
				lhs.m_distortion_amplitude == rhs.m_distortion_amplitude &&
				lhs.m_distortion_frequency == rhs.m_distortion_frequency &&
				lhs.m_distortion_speed == rhs.m_distortion_speed &&
				All(lhs.m_surface == rhs.m_surface) &&
				lhs.m_fade_depth == rhs.m_fade_depth &&
				lhs.m_waterline_offsets == rhs.m_waterline_offsets;
		}
	};

	// Number of directions and samples per direction used by the ambient occlusion search.
	enum class EAOQuality
	{
		Low,    // 1 direction, 2 samples each way
		Medium, // 2 directions, 3 samples each way
		High,   // 3 directions, 4 samples each way
	};

	// Resolution at which ambient occlusion is computed, relative to the scene viewport.
	enum class EAOResolution
	{
		Half, // One AO value per 2x2 pixels, then upsampled with depth awareness. About a quarter of the cost of 'Full'.
		Full, // One AO value per pixel.
	};

	// Screen-space ambient occlusion: darkens opaque surfaces in creases and corners where nearby geometry blocks ambient light.
	// Notes:
	//  - Occlusion is estimated from the opaque scene depth only, so it applies only to opaque surfaces. Transparent surfaces are
	//    composited over the darkened opaque colour and are not darkened themselves.
	//  - The darkening multiplies the whole opaque colour, including direct lighting and ray-traced reflections.
	//  - Pixels where no geometry was drawn are not changed.
	struct AmbientOcclusionProps
	{
		bool m_enabled = false;

		// World-space distance within which geometry can occlude a point.
		float m_radius = 0.5f;

		// Strength of the darkening. The visibility 'v' in [0,1] becomes 'pow(v, m_intensity)', so 0 disables the darkening and larger values darken more.
		float m_intensity = 1.0f;

		// Fraction of 'm_radius', at its outer end, over which the influence of occluders fades to zero. Must be in (0,1].
		float m_falloff = 0.6f;

		// Largest search radius on screen, as a fraction of the viewport height. Limits the cost and the halo size of geometry near the camera.
		float m_max_screen_radius = 0.1f;

		// Search quality and resolution. See 'EAOQuality' and 'EAOResolution'.
		EAOQuality m_quality = EAOQuality::Medium;
		EAOResolution m_resolution = EAOResolution::Half;

		// Show the occlusion term as grey scale instead of applying it to the scene colour.
		bool m_debug_view = false;

		// Reject invalid settings without changing the current scene settings.
		void Validate() const
		{
			if (!std::isfinite(m_radius) || m_radius <= 0.0f)
				throw std::invalid_argument("Ambient occlusion radius must be finite and greater than zero");
			if (!std::isfinite(m_intensity) || m_intensity < 0.0f)
				throw std::invalid_argument("Ambient occlusion intensity must be finite and not negative");
			if (!std::isfinite(m_falloff) || m_falloff <= 0.0f || m_falloff > 1.0f)
				throw std::invalid_argument("Ambient occlusion falloff must be in the range (0,1]");
			if (!std::isfinite(m_max_screen_radius) || m_max_screen_radius <= 0.0f)
				throw std::invalid_argument("Ambient occlusion max screen radius must be finite and greater than zero");

			switch (m_quality)
			{
				case EAOQuality::Low:
				case EAOQuality::Medium:
				case EAOQuality::High: { break; }
				default: { throw std::invalid_argument("Ambient occlusion quality is not a known value"); }
			}
			switch (m_resolution)
			{
				case EAOResolution::Half:
				case EAOResolution::Full: { break; }
				default: { throw std::invalid_argument("Ambient occlusion resolution is not a known value"); }
			}
		}

		// Compare all settings, including those retained while disabled.
		friend bool operator == (AmbientOcclusionProps const& lhs, AmbientOcclusionProps const& rhs) = default;
	};

	// The post-processing effects of one scene.
	// Notes:
	//  - Enabled effects run in a fixed engine-defined order after transparent surfaces are composited and before world
	//    and screen-space overlays, so UI is never post-processed. Each effect covers the scene's viewport.
	//    Ambient occlusion is the exception: it darkens the opaque colour before transparent surfaces are composited.
	//  - Effects cost nothing while disabled: no GPU resources are held and no commands are recorded.
	//  - Animated effects use the engine's real-time clock, but they do not request new frames. The caller must keep
	//    rendering for the animation to be visible.
	struct PostProcessing
	{
		explicit PostProcessing(Renderer& rdr);
		PostProcessing(PostProcessing const&) = delete;
		PostProcessing& operator = (PostProcessing const&) = delete;
		~PostProcessing();

		// True if any effect is enabled
		bool AnyEnabled() const;

		// Get/Set the underwater effect settings. Invalid settings throw and leave the current settings unchanged.
		UnderwaterProps const& Underwater() const;
		void Underwater(UnderwaterProps const& props);

		// Get/Set the ambient occlusion settings. Invalid settings throw and leave the current settings unchanged.
		AmbientOcclusionProps const& AmbientOcclusion() const;
		void AmbientOcclusion(AmbientOcclusionProps const& props);

		// Record the enabled effects for 'scene' into 'frame'.
		// Ambient occlusion darkens the opaque colour before transparent surfaces are composited. The other effects run after compositing.
		void Render(Frame& frame, Scene const& scene);

	private:

		struct PassContext;
		using PassFn = void (PostProcessing::*)(PassContext const&);

		Renderer* m_rdr;
		UnderwaterProps m_underwater;
		AmbientOcclusionProps m_ao;
		std::chrono::steady_clock::time_point m_clock_start;

		// Ambient occlusion resources. Each exists only while ambient occlusion is enabled.
		struct AOResources
		{
			// Linear view depth and two occlusion buffers (raw and blurred) at the AO resolution.
			Texture2DPtr m_depth;
			std::array<Texture2DPtr, 2> m_ao;
			iv2 m_size = {};

			// One root signature serves the compute passes that build the occlusion buffer and the full-resolution passes that apply it.
			D3DPtr<ID3D12RootSignature> m_signature;
			D3DPtr<ID3D12PipelineState> m_pso_prefilter;
			D3DPtr<ID3D12PipelineState> m_pso_main;
			D3DPtr<ID3D12PipelineState> m_pso_blur;
			D3DPtr<ID3D12PipelineState> m_pso_apply;
			D3DPtr<ID3D12PipelineState> m_pso_debug;
			DXGI_FORMAT m_pso_format = DXGI_FORMAT_UNKNOWN;
		};
		std::unique_ptr<AOResources> m_ao_res;

		// Commands that apply ambient occlusion, run between the MSAA resolve and the transparent composite.
		// The list holds no GPU memory, and it lives as long as this object because the GPU may still be executing it.
		::pr::compute::GfxCmdList m_ao_cmd_list;

		// Ping-pong copies of the scene colour. The second one exists only while two or more effects are enabled.
		std::array<Texture2DPtr, 2> m_targets;
		iv2 m_target_size;
		DXGI_FORMAT m_target_format;

		// Pipeline objects, created on first use and recreated if the output format changes.
		D3DPtr<ID3D12RootSignature> m_signature;
		D3DPtr<ID3D12PipelineState> m_pso_underwater;
		DXGI_FORMAT m_pso_format;

		// Create or resize the resources needed to run 'pass_count' passes into a target of 'size' and 'format'.
		void EnsureResources(iv2 size, DXGI_FORMAT format, int pass_count);

		// Release the resources of the effects that run after compositing, once the GPU no longer uses them.
		void ReleaseResources();

		// Create or resize the ambient occlusion resources for an AO buffer of 'size' and an output of 'format'.
		void EnsureAOResources(iv2 size, DXGI_FORMAT format);

		// Release the ambient occlusion resources once the GPU no longer uses them.
		void ReleaseAOResources();

		// Record the ambient occlusion passes for 'scene' into their own command list in 'frame'.
		void RecordAmbientOcclusion(Frame& frame, Scene const& scene);

		// Effect passes, in their canonical order.
		void RecordUnderwater(PassContext const& ctx);
	};
}
