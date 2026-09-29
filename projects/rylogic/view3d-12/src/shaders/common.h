//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#pragma once
#include "pr/view3d-12/forward.h"
#include "pr/view3d-12/main/renderer.h"
#include "pr/view3d-12/main/window.h"
#include "pr/view3d-12/model/vertex_layout.h"
#include "pr/view3d-12/scene/scene.h"
#include "pr/view3d-12/scene/scene_camera.h"
#include "pr/view3d-12/model/nugget.h"
#include "pr/view3d-12/model/model.h"
#include "pr/view3d-12/instance/instance.h"
#include "pr/view3d-12/lighting/light.h"
#include "pr/view3d-12/material/components/base_colour.h"
#include "pr/view3d-12/material/components/emissive.h"
#include "pr/view3d-12/material/components/metallic.h"
#include "pr/view3d-12/material/components/normal_map.h"
#include "pr/view3d-12/material/components/roughness.h"
#include "pr/view3d-12/material/components/two_sided.h"
#include "pr/view3d-12/resource/stock_resources.h"
#include "pr/view3d-12/texture/texture_base.h"
#include "pr/view3d-12/texture/texture_2d.h"
#include "pr/view3d-12/texture/texture_cube.h"
#include "pr/view3d-12/utility/normal_transform.h"
#include "view3d-12/src/render/render_smap.h"

#ifdef NDEBUG
#define PR_RDR_SHADER_COMPILED_DIR(file) PR_STRINGISE(view3d-12/src/shaders/hlsl/compiled/release/##file)
#else
#define PR_RDR_SHADER_COMPILED_DIR(file) PR_STRINGISE(view3d-12/src/shaders/hlsl/compiled/debug/##file)
#endif

namespace pr::rdr12
{
	// How To Make A New Shader:
	// - Add an HLSL file:  e.g. '/view3d-12/shaders/hlsl/<whatever>/your_file.hlsl'
	//   The HLSL file should contain the VS,GS,PS,etc shader definition (see existing examples)
	//   Change the Item Type to 'Custom Build Tool'. The default python script should already
	//   be set from the property sheets.
	// - Add a separate HLSLI file: e.g. 'your_file_cbuf.hlsli' (copy from an existing one)
	//   Set the Item Type to 'Does not participate in the build'
	// - Add a 'shdr_your_file.cpp' file (see existing).
	// - Shaders that get referenced externally to the renderer (i.e. most from now on), need
	//   a public header file as well 'shdr_your_file.h'. This will contain the ShaderT<> derived
	//   types, with the implementation in 'shdr_your_file.cpp' (e.g. shdr_screen_space).
	//   Shaders only used by the renderer don't need a header file (e.g. shdr_fwd.cpp)
	// - The 'Setup' function in your ShaderT<> derived object should follow the 'SetXYZConstants'
	//   pattern. You should be able to #include the 'your_file_cbuf.hlsli' file in the 'shdr_your_file.cpp'
	//   where the 'Setup' method is implemented.
	// - If your shader is a stock resource,
	//      - add it to the enum in "stock_resources.h", 
	//      - forward declare the shader struct in "shader_forward.h"

	#if PR_RDR_RUNTIME_SHADERS
	void RegisterRuntimeShader(RdrId id, char const* cso_filepath);
	#endif

	namespace shaders
	{
		using namespace pr::hlsl;

		#include "view3d-12/src/shaders/hlsl/types.hlsli"
		#include "view3d-12/src/shaders/hlsl/lighting/lighting_cbuf.hlsli"

		// The constant buffer definitions
		namespace fwd
		{
			#include "view3d-12/src/shaders/hlsl/forward/forward_cbuf.hlsli"
			static_assert((sizeof(CBufFrame) % 16) == 0);
			static_assert((sizeof(CBufNugget) % 16) == 0);
			static_assert((sizeof(CBufPbrSurface) % 16) == 0);
			static_assert((sizeof(CBufFade) % 16) == 0);
			static_assert((sizeof(CBufScreenSpace) % 16) == 0);
			static_assert((sizeof(CBufDiag) % 16) == 0);
		}
		namespace smap
		{
			#include "view3d-12/src/shaders/hlsl/shadow/shadow_map_cbuf.hlsli"
			static_assert((sizeof(CBufFrame) % 16) == 0);
			static_assert((sizeof(CBufNugget) % 16) == 0);
		}
		namespace ray_cast
		{
			#include "view3d-12/src/shaders/hlsl/ray_cast/ray_cast_cbuf.hlsli"
			static_assert((sizeof(CBufFrame) % 16) == 0);
			static_assert((sizeof(CBufNugget) % 16) == 0);
		}
		namespace rt
		{
			#include "view3d-12/src/shaders/hlsl/ray_tracing/ray_tracing_cbuf.hlsli"
			static_assert((sizeof(CBufFrame) % 16) == 0);
			static_assert((sizeof(RayTracingMaterial) % 16) == 0);
			static_assert(sizeof(RayTracingVertex) == sizeof(Vert));
			static_assert((sizeof(RayTracingGeometry) % 16) == 0);
		}
	}
	
	// Return the padded size of a constants buffer of type 'T'
	template <typename T> constexpr size_t cbuf_size_aligned_v = PadTo<size_t>(sizeof(T), D3D12_CONSTANT_BUFFER_DATA_PLACEMENT_ALIGNMENT);

	// Set the CBuffer model constants flags
	template <typename TCBuf> requires(requires(TCBuf cb) { cb.flags; })
	void SetFlags(TCBuf& cb, BaseInstance const& inst, Material const& material, NuggetDesc const& nug, bool env_mapped)
	{
		auto model_flags = 0;
		{
			// Has normals
			if (AllSet(nug.m_geom, EGeom::Norm))
				model_flags |= shaders::ModelFlags_HasNormals;

			// Treat the surface as two-sided for lit normal orientation.
			auto const* two_sided = material.Component<materials::TwoSided>();
			if (two_sided != nullptr && two_sided->m_enabled)
				model_flags |= shaders::ModelFlags_TwoSided;

			// Is Skinned
			if (ModelPtr const* model = inst.find<ModelPtr>(EInstComp::ModelPtr); model && (*model)->m_skin)
				if (PosePtr const* pose = inst.find<PosePtr>(EInstComp::PosePtr); pose && *pose)
					model_flags |= shaders::ModelFlags_IsSkinned;
		}

		auto texture_flags = 0;
		{
			auto const* base_colour = material.Component<materials::BaseColour>();

			// Has diffuse texture
			Texture2DPtr tex = {};
			if (base_colour != nullptr)
				tex = base_colour->m_tex.m_texture;

			if (AllSet(nug.m_geom, EGeom::Tex0) && tex != nullptr)
			{
				texture_flags |= shaders::TextureFlags_HasDiffuse;

				// Texture by projection from the environment map
				if (tex->m_uri == RdrId(EStockTexture::EnvMapProjection))
					texture_flags |= shaders::TextureFlags_ProjectFromEnvMap;
			}

			// Is reflective
			auto const* reflectivity = material.Component<materials::Reflectivity>();
			auto rel_reflec = reflectivity != nullptr ? reflectivity->m_rel_reflec : 0.0f;
			if (float const* reflec;
				env_mapped &&                                                            // There is an env map
				AllSet(nug.m_geom, EGeom::Norm) &&                                       // The model contains normals
				(reflec = inst.find<float>(EInstComp::EnvMapReflectivity)) != nullptr && // The instance has a reflectivity value
				*reflec * rel_reflec != 0)                                               // and the reflectivity isn't zero
				texture_flags |= shaders::TextureFlags_IsReflective;
		}

		auto alpha_flags = 0;
		{
			// Has alpha pixels
			if (nug.m_sort_key.Group() > ESortGroup::PreAlpha)
				alpha_flags |= shaders::AlphaFlags_HasAlpha;
		}

		auto inst_id = 0;
		{
			// Unique id for this instance
			inst_id = UniqueId(inst);
		}

		cb.flags = iv4{ model_flags, texture_flags, alpha_flags, inst_id };
	}

	// Set the transform properties of a constants buffer
	template <typename TCBuf> requires(requires(TCBuf cb) { cb.o2w; cb.n2w; })
	void SetTxfm(TCBuf& cb, BaseInstance const& inst, Model const* model)
	{
		m4x4 o2w = GetO2W(inst);
		m4x4 m2o = model ? model->m_m2root : m4x4::Identity();

		cb.m2o = m2o;
		cb.o2w = o2w;

		// Transform normals through the complete model placement, including nonuniform scale and shear.
		cb.n2w = NormalTransform(o2w * m2o);
	}
	// Set placement and projection using the current pass's camera transforms, retaining instance-specific projections.
	template <typename TCBuf> requires(requires(TCBuf cb) { cb.o2s; cb.o2w; cb.n2w; })
	void SetTxfm(TCBuf& cb, BaseInstance const& inst, Model const* model, CameraTransforms const& camera)
	{
		// Share one object transform between placement, normals, and projection.
		SetTxfm(cb, inst, model);

		// Preserve the original (projection * world_to_camera) * object_to_world association, including projection overrides.
		m4x4 c2s;
		cb.o2s = FindC2S(inst, c2s)
			? (c2s * camera.m_w2c) * cb.o2w
			: camera.m_w2s * cb.o2w;
	}

	// Decode a packed surface override into linear RGB and its UNORM8 blend weight. Missing components disable the override.
	inline v4 ColourBlendConstant(BaseInstance const& inst)
	{
		// Disabled instances need no colour conversion; packed alpha is a weight, not surface opacity.
		auto colour = inst.find<Colour32>(EInstComp::ColourBlend32);
		if (colour == nullptr || a_cp(*colour) == 0.0f)
			return v4::Zero();

		// Colour decodes sRGB channels but leaves alpha as a linear coefficient.
		return Colour(*colour).rgba;
	}

	// Set the multiplicative tint and independent surface RGB override of a constants buffer.
	template <typename TCBuf> requires(requires(TCBuf cb) { cb.tint; cb.colour_blend; })
	void SetTint(TCBuf& cb, BaseInstance const& inst, Material const& material)
	{
		// Preserve the existing combination of instance and material tint.
		auto col = inst.find<Colour32>(EInstComp::TintColour32);
		auto c = Colour((col ? *col : Colour32White) * material.TintColour());
		cb.tint = c.rgba;

		// Missing components preserve the original surface; this changes no material or alpha flags.
		cb.colour_blend = ColourBlendConstant(inst);
	}

	// Set the texture properties of a constants buffer
	template <typename TCBuf> requires (requires(TCBuf cb) { cb.tex2surf0; })
	void SetTex2Surf(TCBuf& cb, BaseInstance const&, Material const& material)
	{
		Texture2DPtr tex = {};
		if (auto const* base_colour = material.Component<materials::BaseColour>())
			tex = base_colour->m_tex.m_texture;

		// The base texture defines the texture to surface transform. Other maps should
		// use the same transform for correct texture coordinate mapping.
		cb.tex2surf0 = tex != nullptr
			? tex->m_t2s
			: m4x4::Identity();
	}

	// Set the environment map properties of a constants buffer
	template <typename TCBuf> requires (requires(TCBuf cb) { cb.env_reflectivity; })
	void SetReflectivity(TCBuf& cb, BaseInstance const& inst, Material const& material)
	{
		auto reflectivity = inst.find<float>(EInstComp::EnvMapReflectivity);
		auto const* material_reflectivity = material.Component<materials::Reflectivity>();
		cb.env_reflectivity = reflectivity != nullptr && material_reflectivity != nullptr
			? *reflectivity * material_reflectivity->m_rel_reflec
			: 0.0f;
	}

	// Set screen space, per instance constants
	template <typename TCBuf> requires (requires(TCBuf cb) { cb.screen_dim; cb.size; cb.depth; })
	void SetScreenSpace(TCBuf& cb, BaseInstance const& inst, Scene const& scene, v2 size, bool depth)
	{
		auto sz = inst.find<v2>(EInstComp::SSSize);
		auto rt_size = scene.wnd().BackBufferSize();
		cb.screen_dim = To<v2>(rt_size);
		cb.size = sz ? *sz : size;
		cb.depth = depth;
	}

	// Set the scene view constants
	inline void SetViewConstants(shaders::Camera& cb, SceneCamera const& view)
	{
		cb.c2w = view.CameraToWorld();
		cb.c2s = view.CameraToScreen();
		cb.w2c = InvertOrthonormal(cb.c2w);
		cb.w2s = cb.c2s * cb.w2c;
	}

	// Set the frame lighting constants. 'lights_cb' receives the ambient light and the light count; the lights themselves are uploaded by 'UploadLights'.
	template <typename TCBuf> requires (requires(TCBuf cb) { cb.ambient; cb.light_info; })
	void SetLightingConstants(TCBuf& cb, Scene const& scene)
	{
		// Ambient light is scene-wide. The alpha channel is unused.
		cb.ambient = Colour(scene.m_ambient).rgba;
		cb.light_info.x = isize(scene.ResolvedLights());
	}

	// Convert a world space light into the shader light layout.
	// 'shadow_view' is the index of the light's first shadow view, or -1 if the light has no shadow this frame.
	inline shaders::Light ToShaderLight(Light const& light, int shadow_view)
	{
		static_assert(shaders::MaxLights == rdr12::MaxLights, "Shader and renderer light limits must match");
		static_assert(sizeof(shaders::Light) % 16 == 0, "Shader lights are stored in a structured buffer with 16 byte aligned elements");
		return shaders::Light{
			.info = iv4(int(light.m_type), shadow_view, shadow_view >= 0 ? 1 : 0, 0),
			.ws_direction = light.m_direction,
			.ws_position = light.m_position,
			.colour = Colour(light.m_diffuse, light.m_intensity).rgba,
			.specular = Colour(light.m_specular, light.m_specular_power).rgba,
			.spot = v4(light.m_inner_angle, light.m_outer_angle, light.m_range, light.m_falloff),
			.shadow = v4(Clamp(light.m_cast_shadow, 0.0f, 1.0f), 0, 0, 0),
		};
	}

	// Upload the scene's resolved lights for the current frame and return the GPU address of the light array.
	// At least one element is always uploaded so the address is valid even when the scene has no lights.
	template <typename TUploadBuffer>
	D3D12_GPU_VIRTUAL_ADDRESS UploadLights(TUploadBuffer& upload, Scene const& scene)
	{
		// Allocate space for the light array in the upload buffer
		auto lights = scene.ResolvedLights();
		auto shadow_light = scene.ShadowLightIndex();
		auto count = std::max<int64_t>(isize(lights), 1);
		auto alex = upload.Alloc(count * sizeof(shaders::Light), 16);
		auto dst = reinterpret_cast<shaders::Light*>(alex.m_mem + alex.m_ofs);

		// Convert each light. Only the shadow light has a shadow view in this renderer version.
		dst[0] = shaders::Light{};
		for (int i = 0; i != isize(lights); ++i)
			dst[i] = ToShaderLight(lights[i], i == shadow_light ? 0 : -1);

		return alex.m_res->GetGPUVirtualAddress() + alex.m_ofs;
	}

	// Set the shadow map constants
	inline void SetShadowMapConstants(shaders::Shadow& cb, Scene const& scene)
	{
		// Ignore if there is no shadow map step, or no light casts a shadow this frame
		auto smap_step = scene.FindRStep<RenderSmap>();
		if (smap_step == nullptr || scene.ShadowLightIndex() < 0)
			return;

		// Add the shadow maps to the shader params
		int i = 0;
		for (auto& caster : smap_step->Casters())
		{
			if (i == shaders::MaxShadowMaps)
				break;

			cb.info.x = i + 1;
			cb.info.y = caster.m_size;
			cb.w2l[i] = caster.m_params.m_w2ls;
			cb.l2s[i] = caster.m_params.m_ls2s;
			++i;
		}
	}

	// Set the env-map to world orientation
	inline void SetEnvMapConstants(shaders::EnvMap& cb, TextureCube const* env_map)
	{
		if (env_map == nullptr) return;
		cb.w2env = InvertOrthonormal(env_map->m_cube2w);
	}
}
