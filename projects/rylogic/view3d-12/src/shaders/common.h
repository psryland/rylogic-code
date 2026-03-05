//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#pragma once
#include "pr/view3d-12/forward.h"
#include "pr/view3d-12/main/window.h"
#include "pr/view3d-12/scene/scene.h"
#include "pr/view3d-12/scene/scene_camera.h"
#include "pr/view3d-12/model/nugget.h"
#include "pr/view3d-12/model/model.h"
#include "pr/view3d-12/instance/instance.h"
#include "pr/view3d-12/lighting/light.h"
#include "pr/view3d-12/resource/stock_resources.h"
#include "pr/view3d-12/shaders/shader_registers.h"
#include "pr/view3d-12/texture/texture_base.h"
#include "pr/view3d-12/texture/texture_2d.h"
#include "pr/view3d-12/texture/texture_cube.h"
#include "pr/view3d-12/utility/map_resource.h"
#include "pr/view3d-12/utility/utility.h"
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
		#include "view3d-12/src/shaders/hlsl/types.hlsli"

		// The constant buffer definitions
		namespace fwd
		{
			#include "view3d-12/src/shaders/hlsl/forward/forward_cbuf.hlsli"
			static_assert((sizeof(CBufFrame) % 16) == 0);
			static_assert((sizeof(CBufNugget) % 16) == 0);
			static_assert((sizeof(CBufFade) % 16) == 0);
			static_assert((sizeof(CBufScreenSpace) % 16) == 0);
			static_assert((sizeof(CBufDiag) % 16) == 0);
		}
		namespace ds
		{
			#include "view3d-12/src/shaders/hlsl/deferred/gbuffer_cbuf.hlsli"
			static_assert((sizeof(CBufCamera) % 16) == 0);
			static_assert((sizeof(CBufLighting) % 16) == 0);
			static_assert((sizeof(CBufNugget) % 16) == 0);
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
	}
	
	// Return the padded size of a constants buffer of type 'T'
	template <typename T> constexpr size_t cbuf_size_aligned_v = PadTo<size_t>(sizeof(T), D3D12_CONSTANT_BUFFER_DATA_PLACEMENT_ALIGNMENT);

	// Set the CBuffer model constants flags
	template <typename TCBuf> requires(requires(TCBuf x) { x.m_flags; })
	void SetFlags(TCBuf& cb, BaseInstance const& inst, NuggetDesc const& nug, bool env_mapped)
	{
		auto model_flags = 0;
		{
			// Has normals
			if (AllSet(nug.m_geom, EGeom::Norm))
				model_flags |= shaders::ModelFlags_HasNormals;

			// Is Skinned
			if (ModelPtr const* model = inst.find<ModelPtr>(EInstComp::ModelPtr); model && (*model)->m_skin)
				if (PosePtr const* pose = inst.find<PosePtr>(EInstComp::PosePtr); pose && *pose)
					model_flags |= shaders::ModelFlags_IsSkinned;
		}

		auto texture_flags = 0;
		{
			// Has diffuse texture
			if (Texture2DPtr tex; AllSet(nug.m_geom, EGeom::Tex0) && (tex = coalesce(FindDiffTexture(inst), nug.m_tex_diffuse)) != nullptr)
			{
				texture_flags |= shaders::TextureFlags_HasDiffuse;

				// Texture by projection from the environment map
				if (tex->m_uri == RdrId(EStockTexture::EnvMapProjection))
					texture_flags |= shaders::TextureFlags_ProjectFromEnvMap;
			}

			// Is reflective
			if (float const* reflec;
				env_mapped &&                                                            // There is an env map
				AllSet(nug.m_geom, EGeom::Norm) &&                                       // The model contains normals
				(reflec = inst.find<float>(EInstComp::EnvMapReflectivity)) != nullptr && // The instance has a reflectivity value
				*reflec * nug.m_rel_reflec != 0)                                         // and the reflectivity isn't zero
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

		cb.m_flags = iv4{ model_flags, texture_flags, alpha_flags, inst_id };
	}

	// Set the transform properties of a constants buffer
	template <typename TCBuf> requires(requires(TCBuf x) { x.m_o2w; x.m_n2w; })
	void SetTxfm(TCBuf& cb, BaseInstance const& inst, Model const* model)
	{
		m4x4 o2w = GetO2W(inst);
		m4x4 m2o = model ? model->m_m2root : m4x4::Identity();

		cb.m_m2o = m2o;
		cb.m_o2w = o2w;

		// Orthonormalise the rotation part of the normal to world transform (allowing for scale matrices)
		cb.m_n2w = cb.m_o2w;
		cb.m_n2w.x = Normalise(cb.m_n2w.x, v4::Zero());
		cb.m_n2w.y = Normalise(Cross(cb.m_n2w.z, cb.m_n2w.x), v4::Zero());
		cb.m_n2w.z = Cross(cb.m_n2w.x, cb.m_n2w.y);
	}
	template <typename TCBuf> requires(requires(TCBuf x) { x.m_o2s; x.m_o2w; x.m_n2w; })
	void SetTxfm(TCBuf& cb, BaseInstance const& inst, Model const* model, SceneCamera const& view)
	{
		SetTxfm(cb, inst, model);

		m4x4 o2w = GetO2W(inst);
		m4x4 w2c = InvertAffine(view.CameraToWorld());
		m4x4 c2s = FindC2S(inst, c2s) ? c2s : view.CameraToScreen();

		// Set the object to screen projection
		cb.m_o2s = c2s * w2c * o2w;
	}

	// Set the tint properties of a constants buffer
	template <typename TCBuf> requires(requires(TCBuf x) { x.m_tint; })
	void SetTint(TCBuf& cb, BaseInstance const& inst, NuggetDesc const& nug)
	{
		auto col = inst.find<Colour32>(EInstComp::TintColour32);
		auto c = Colour((col ? *col : Colour32White) * nug.m_tint);
		cb.m_tint = c.rgba;
	}

	// Set the texture properties of a constants buffer
	template <typename TCBuf> requires (requires(TCBuf x) { x.m_tex2surf0; })
	void SetTex2Surf(TCBuf& cb, BaseInstance const& inst, NuggetDesc const& nug)
	{
		auto tex = coalesce(FindDiffTexture(inst), nug.m_tex_diffuse);
		cb.m_tex2surf0 = tex != nullptr
			? tex->m_t2s
			: m4x4::Identity();
	}

	// Set the environment map properties of a constants buffer
	template <typename TCBuf> requires (requires(TCBuf x) { x.m_env_reflectivity; })
	void SetReflectivity(TCBuf& cb, BaseInstance const& inst, NuggetDesc const& nug)
	{
		auto reflectivity = inst.find<float>(EInstComp::EnvMapReflectivity);
		cb.m_env_reflectivity = reflectivity != nullptr
			? *reflectivity * nug.m_rel_reflec
			: 0.0f;
	}

	// Set screen space, per instance constants
	template <typename TCBuf> requires (requires(TCBuf x) { x.m_screen_dim; x.m_size; x.m_depth; })
	void SetScreenSpace(TCBuf& cb, BaseInstance const& inst, Scene const& scene, v2 size, bool depth)
	{
		auto sz = inst.find<v2>(EInstComp::SSSize);
		auto rt_size = scene.wnd().BackBufferSize();
		cb.m_screen_dim = To<v2>(rt_size);
		cb.m_size = sz ? *sz : size;
		cb.m_depth = depth;
	}

	// Set the scene view constants
	inline void SetViewConstants(shaders::Camera& cb, SceneCamera const& view)
	{
		cb.m_c2w = view.CameraToWorld();
		cb.m_c2s = view.CameraToScreen();
		cb.m_w2c = InvertAffine(cb.m_c2w);
		cb.m_w2s = cb.m_c2s * cb.m_w2c;
	}

	// Set the lighting constants
	inline void SetLightingConstants(shaders::Light& cb, Light const& light, SceneCamera const& view)
	{
		// If the global light is camera relative, adjust the position and direction appropriately
		auto pos = light.m_cam_relative ? view.CameraToWorld() * light.m_position : light.m_position;
		auto dir = light.m_cam_relative ? view.CameraToWorld() * light.m_direction : light.m_direction;

		cb.m_info         = iv4(int(light.m_type),0,0,0);
		cb.m_ws_direction = dir;
		cb.m_ws_position  = pos;
		cb.m_ambient      = Colour(light.m_ambient).rgba;
		cb.m_colour       = Colour(light.m_diffuse).rgba;
		cb.m_specular     = Colour(light.m_specular, light.m_specular_power).rgba;
		cb.m_spot         = v4(light.m_inner_angle, light.m_outer_angle, light.m_range, light.m_falloff);
	}

	// Set the shadow map constants
	inline void SetShadowMapConstants(shaders::Shadow& cb, RenderSmap const* smap_step)
	{
		// Ignore if there is no shadow map step
		if (smap_step == nullptr)
			return;

		// Add the shadow maps to the shader params
		int i = 0;
		for (auto& caster : smap_step->Casters())
		{
			if (i == shaders::MaxShadowMaps)
				break;

			cb.m_info.x = i + 1;
			cb.m_info.y = caster.m_size;
			cb.m_w2l[i] = caster.m_params.m_w2ls;
			cb.m_l2s[i] = caster.m_params.m_ls2s;
			++i;
		}
	}

	// Set the env-map to world orientation
	inline void SetEnvMapConstants(shaders::EnvMap& cb, TextureCube const* env_map)
	{
		if (env_map == nullptr) return;
		cb.m_w2env = InvertAffine(env_map->m_cube2w);
	}
}
