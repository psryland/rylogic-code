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
	// Whole-screen "looking through water" effect: a colour tint, depth-based distance fog, and a moving distortion.
	// The caller decides when the camera is submerged; the effect does not test the camera against any water surface.
	struct UnderwaterProps
	{
		bool m_enabled = false;

		// sRGB colour multiplied into the scene colour. Alpha is ignored.
		Colour32 m_tint = Colour32{0xFFA6D9F2U};

		// sRGB colour that distant surfaces fade towards. Alpha is ignored.
		Colour32 m_fog_colour = Colour32{0xFF0A384DU};

		// Distance (in world units) at which the fog hides 95% of a surface. Pixels without scene geometry are fully fogged.
		float m_visibility = 40.0f;

		// Largest screen offset of the distortion, as a fraction of the viewport height. Zero disables the distortion.
		float m_distortion_amplitude = 0.002f;

		// Number of distortion ripples per viewport height.
		float m_distortion_frequency = 6.0f;

		// Distortion animation rate, in cycles per second.
		float m_distortion_speed = 0.25f;

		// Reject invalid settings without changing the current scene settings.
		void Validate() const
		{
			if (!std::isfinite(m_visibility) || m_visibility <= 0.0f)
				throw std::invalid_argument("Underwater visibility must be finite and greater than zero");
			if (!std::isfinite(m_distortion_amplitude) || m_distortion_amplitude < 0.0f)
				throw std::invalid_argument("Underwater distortion amplitude must be finite and not negative");
			if (!std::isfinite(m_distortion_frequency) || m_distortion_frequency <= 0.0f)
				throw std::invalid_argument("Underwater distortion frequency must be finite and greater than zero");
			if (!std::isfinite(m_distortion_speed) || m_distortion_speed < 0.0f)
				throw std::invalid_argument("Underwater distortion speed must be finite and not negative");
		}

		// Compare all settings, including those retained while disabled.
		friend bool operator == (UnderwaterProps const&, UnderwaterProps const&) = default;
	};

	// The post-processing effects of one scene.
	// Notes:
	//  - Enabled effects run in a fixed engine-defined order after transparent surfaces are composited and before world
	//    and screen-space overlays, so UI is never post-processed. Each effect covers the scene's viewport.
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

		// Record the enabled effects for 'scene' into 'frame'.
		void Render(Frame& frame, Scene const& scene);

	private:

		struct PassContext;
		using PassFn = void (PostProcessing::*)(PassContext const&);

		Renderer* m_rdr;
		UnderwaterProps m_underwater;
		std::chrono::steady_clock::time_point m_clock_start;

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

		// Release all GPU resources once the GPU no longer uses them.
		void ReleaseResources();

		// Effect passes, in their canonical order.
		void RecordUnderwater(PassContext const& ctx);
	};
}
