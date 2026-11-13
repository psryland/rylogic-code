//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#pragma once
#include "pr/view3d-12/forward.h"
#include "pr/view3d-12/shaders/shader.h"

namespace pr::rdr12::shaders
{
	namespace fwd
	{
		// This is the index order of parameters added to the root signature
		enum class ERootParam
		{
			CBufFrame = 0,
			ElementIndex,
			CBufFade,
			CBufScreenSpace,
			CBufPbrSurface,
			CBufDiag,
			CBufProcedural,
			CBufDetailNormals,
			DiffTexture,
			EnvMap,
			ShadowAtlas,
			ProjTex,
			PbrMetallicTexture,
			PbrRoughnessTexture,
			OpaqueDepth,
			PbrEmissiveTexture,
			TexStreams, // Table of the optional texture-coordinate lanes, t8..t11
			PbrNormalTexture,
			DiffTextureSampler,
			PbrMetallicSampler,
			PbrRoughnessSampler,
			PbrEmissiveSampler,
			PbrNormalSampler,
			AlphaColour,
			AlphaDepth,
			AlphaRtAttrs,
			SkyTexture,
			Lights,
			ShadowViews,
			ProceduralBuffer0,
			ProceduralBuffer1,
			Elements,
			EnvMapPrev,
			EnvMapDistance,
			SkyHistory,
			SkyHistoryOut,
			PopulationInstances,
			PopulationView,
			PopulationPayload,
			PopulationDraw,
			PopulationSkin,
			ReflectionPlane, // The projected reflection texture
			PbrTintMaskTexture,
			PbrTintMaskSampler,
			ProceduralTextures, // Table of the procedural pixel family's texture slots, t31..t34
		};

		enum class ESampParam
		{
			EnvMap,
			ShadowAtlas,
			ProjTex,
		};
	}

	struct Forward :Shader
	{
		// The procedural sky's low cloud history constants for the frame, or null when the sky does not blend its low cloud over frames.
		// Not owned; the render step that sets it keeps it valid while it renders.
		fwd::SkyHistory const* m_sky_history;

		// The projected planar reflection constants for the frame, or null when the reflection texture is not available to the draws being rendered.
		// Not owned; the render step that sets it keeps it valid while it renders.
		fwd::ReflectionPlane const* m_reflection_plane;

		// A view that replaces the scene camera for the frame constants, or null to use the scene camera. See 'fwd::ViewOverride'.
		// Not owned; the render step that sets it keeps it valid while it renders.
		fwd::ViewOverride const* m_view;

		// A null view of a 'StructuredBuffer<float2>' for texture-coordinate lanes that a draw does not use. Reads return zero.
		Descriptor m_null_texcoord_stream;

		explicit Forward(Renderer& rdr);
		~Forward() override;
		void SetupFrame(ID3D12GraphicsCommandList* cmd_list, GpuUploadBuffer& upload, Scene const& scene) override;

		// Upload one set of element constants per entry in 'drawlist', in drawlist order, using 'camera' for the projection.
		// Returns the GPU address of the table, for use with 'SetupElements'.
		static D3D12_GPU_VIRTUAL_ADDRESS UploadElements(GpuUploadBuffer& upload, Scene const& scene, CameraTransforms const& camera, std::span<DrawListElement const> drawlist);

		// Bind a table of element constants created by 'UploadElements'
		static void SetupElements(ID3D12GraphicsCommandList* cmd_list, D3D12_GPU_VIRTUAL_ADDRESS elements);

		// Select the entry in the bound element table used by the next draw
		static void SetupElement(ID3D12GraphicsCommandList* cmd_list, int index);
	};
}
