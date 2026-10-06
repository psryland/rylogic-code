//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2026
//*********************************************
#include "pr/view3d-12/material/material_simple.h"
#include "pr/view3d-12/instance/instance.h"
#include "pr/view3d-12/main/window.h"
#include "pr/view3d-12/model/nugget.h"
#include "pr/view3d-12/render/drawlist_element.h"
#include "pr/view3d-12/sampler/sampler.h"
#include "pr/view3d-12/scene/scene.h"
#include "pr/view3d-12/shaders/shader_forward.h"
#include "pr/view3d-12/shaders/shader_procedural.h"
#include "pr/view3d-12/shaders/shader_ray_cast.h"
#include "pr/view3d-12/shaders/shader_smap.h"
#include "pr/view3d-12/texture/texture_2d.h"
#include "view3d-12/src/shaders/common.h"

namespace pr::rdr12
{
	// Apply shader overlays supported by the active material render step.
	void materials::ApplyShaderOverlays(MaterialPassContext& ctx, bool procedural_only)
	{
		// Preserve broad legacy overlays for Forward while restricting secondary raster passes to the public procedural contract.
		auto const* overlays = ctx.m_material.Component<ShaderOverlays>();
		if (overlays == nullptr)
			return;

		for (auto& shdr_overlay : overlays->m_overlays)
		{
			// Apply only overlays explicitly assigned to this pass.
			if (shdr_overlay.m_rdr_step != ctx.m_step_id)
				continue;

			auto& overlay = *shdr_overlay.m_overlay.get();
			auto* procedural = dynamic_cast<ProceduralVertexShader*>(&overlay);
			if (procedural_only && procedural == nullptr)
				continue;
			if (procedural != nullptr && procedural->m_rdr_step != ctx.m_step_id)
				throw std::runtime_error("Procedural vertex shader render-step contract mismatch");
			if (overlay.m_signature)
			{
				// A complete legacy overlay owns its signature as well as its shader stages.
				ctx.m_pipe_state.Apply(PSO<EPipeState::RootSignature>(overlay.m_signature.get()));
				ctx.m_cmd_list.SetGraphicsRootSignature(overlay.m_signature.get());
				ctx.m_root_signature_changed = true;
			}
			if (overlay.m_code.VS) ctx.m_pipe_state.Apply(PSO<EPipeState::VS>(overlay.m_code.VS));
			if (overlay.m_code.PS) ctx.m_pipe_state.Apply(PSO<EPipeState::PS>(overlay.m_code.PS));
			if (overlay.m_code.DS) ctx.m_pipe_state.Apply(PSO<EPipeState::DS>(overlay.m_code.DS));
			if (overlay.m_code.HS) ctx.m_pipe_state.Apply(PSO<EPipeState::HS>(overlay.m_code.HS));
			if (overlay.m_code.GS) ctx.m_pipe_state.Apply(PSO<EPipeState::GS>(overlay.m_code.GS));

			// Bind overlay-owned resources after the base material has established its stock root contract.
			overlay.SetupFrame(ctx.m_cmd_list.get(), ctx.m_upload, ctx.m_scene);
			overlay.SetupElement(ctx.m_cmd_list.get(), ctx.m_upload, ctx.m_scene, ctx.m_camera, &ctx.m_dle);
		}
	}

	// Replace the layers in use. 'layers' must contain at most MaxLayers entries with finite values.
	void materials::DetailNormalLayers::Set(std::span<DetailNormalLayer const> layers)
	{
		// Reject invalid layers at the public boundary so shaders never see non-finite projections.
		if (layers.size() > MaxLayers)
			throw std::runtime_error("Too many detail-normal layers");

		for (auto const& layer : layers)
		{
			// Every projection row and scale must be finite.
			if (!IsFinite(layer.m_row_u) || !IsFinite(layer.m_row_v) || !std::isfinite(layer.m_height_scale))
				throw std::runtime_error("Detail-normal layers must be finite");
		}

		std::copy(layers.begin(), layers.end(), m_layers.begin());
		m_count = static_cast<int>(layers.size());
	}

	// Replace a stock simple-material forward pixel shader in 'desc' with its detail-normal variant. Throws for any other pixel shader.
	void materials::ApplyDetailNormalsPixelShader(PipeStateDesc& desc)
	{
		// Map each stock simple-material entry point to the variant that perturbs the normal before calling it.
		// Far-clip-fade variants are selected later from the detail family, so only the stock sub-pass entries are mapped here.
		struct Mapping { shader_code::ByteCode const* m_stock; shader_code::ByteCode const* m_detail; };
		static Mapping const mappings[] =
		{
			{ &shader_code::forward_ps, &shader_code::forward_detail_ps },
			{ &shader_code::forward_reflection_attrs_ps, &shader_code::forward_reflection_attrs_detail_ps },
			{ &shader_code::forward_alpha_collect_ps, &shader_code::forward_alpha_collect_detail_ps },
		};

		auto const* gfx = static_cast<D3D12_GRAPHICS_PIPELINE_STATE_DESC const*>(desc);
		for (auto const& mapping : mappings)
		{
			// Pixel shaders are identified by their compiled byte code.
			if (gfx->PS.pShaderBytecode != mapping.m_stock->pShaderBytecode || gfx->PS.BytecodeLength != mapping.m_stock->BytecodeLength)
				continue;

			desc.Apply(PSO<EPipeState::PS>(*mapping.m_detail));
			return;
		}
		throw std::runtime_error("Detail normals require a stock simple-material forward pixel shader");
	}

	namespace
	{
		// The material pass that reproduces default NuggetDesc material handling.
		struct MaterialSimplePass : MaterialPass
		{
			// Return true if this simple material needs alpha rendering.
			bool RequiresAlpha(BaseInstance const&, Material const& material, Nugget const& nugget) const override
			{
				return
					material.RequiresAlpha() ||
					AnySet(nugget.m_nflags, ENuggetFlag::GeometryHasAlpha | ENuggetFlag::AlphaBlend);
			}

			// Contribute texture and shader-overlay state to the sort key.
			SortKey AddSortKey(ERenderStep step, BaseInstance const&, Material const& material, Nugget const&, SortKey key) const override
			{
				switch (step)
				{
					case ERenderStep::RenderForward:
					{
						auto& base_colour = material.ComponentOrDefault<materials::BaseColour>();
						if (!AnySet(key, SortKey::TextureIdMask) && base_colour.m_tex.m_texture != nullptr)
							key = SetBits(key, SortKey::TextureIdMask, base_colour.m_tex.m_texture->SortId() << SortKey::TextureIdOfs);

						if (!AnySet(key, SortKey::ShaderIdMask))
						{
							auto shdr_id = 0;
							if (auto const* overlays = material.Component<materials::ShaderOverlays>(); overlays != nullptr)
							{
								for (auto& overlay : overlays->m_overlays)
								{
									if (overlay.m_rdr_step != step)
										continue;

									shdr_id = shdr_id * 13 ^ overlay.m_overlay->SortId();
								}
							}
							key = SetBits(key, SortKey::ShaderIdMask, shdr_id << SortKey::ShaderIdOfs);
						}
						return key;
					}
					case ERenderStep::ShadowMap:
					{
						auto& base_colour = material.ComponentOrDefault<materials::BaseColour>();
						if (!AnySet(key, SortKey::TextureIdMask) && base_colour.m_tex.m_texture != nullptr)
							key = SetBits(key, SortKey::TextureIdMask, base_colour.m_tex.m_texture->SortId() << SortKey::TextureIdOfs);

						return key;
					}
					case ERenderStep::RayCast:
					case ERenderStep::RayTracing:
					{
						return key;
					}
					case ERenderStep::Invalid:
					default:
					{
						throw std::runtime_error("Unknown render step");
					}
				}
			}

			// Bind resources and constants for the simple material pass.
			void Bind(MaterialPassContext& ctx) const override
			{
				switch (ctx.m_step_id)
				{
					case ERenderStep::RenderForward:
					{
						BindForward(ctx);
						return;
					}
					case ERenderStep::ShadowMap:
					{
						BindShadowMap(ctx);
						return;
					}
					case ERenderStep::RayCast:
					{
						BindRayCast(ctx);
						return;
					}
					case ERenderStep::RayTracing:
					{
						return;
					}
					case ERenderStep::Invalid:
					default:
					{
						throw std::runtime_error("Unknown render step");
					}
				}
			}

			// Apply pipeline changes for the simple material pass.
			void ApplyPipeline(MaterialPassContext& ctx) const override
			{
				// Force two-sided rasterisation for materials that need normal flipping on back faces.
				static auto ApplyTwoSidedPipeline = [](MaterialPassContext& ctx)
				{
					auto const* two_sided = ctx.m_material.Component<materials::TwoSided>();
					if (two_sided == nullptr || !two_sided->m_enabled)
						return;

					// Alpha variants already render front/back faces separately for sorting, so overriding their cull mode would double-submit both sides.
					if (ctx.m_dle.m_nugget->m_variant == AlphaNugget)
						return;

					ctx.m_pipe_state.Apply(PSO<EPipeState::CullMode>(D3D12_CULL_MODE_NONE));
				};

				switch (ctx.m_step_id)
				{
					case ERenderStep::RenderForward:
					{
						// Forward retains all existing overlay behavior. Detail normals then swap in their variant of the resulting stock pixel shader.
						materials::ApplyShaderOverlays(ctx, false);
						if (ctx.m_material.Component<materials::DetailNormals>() != nullptr)
							materials::ApplyDetailNormalsPixelShader(ctx.m_pipe_state);

						ApplyTwoSidedPipeline(ctx);
						return;
					}
					case ERenderStep::ShadowMap:
					case ERenderStep::RayCast:
					{
						// Secondary raster passes accept only the bounded procedural VS overlay.
						materials::ApplyShaderOverlays(ctx, true);
						ApplyTwoSidedPipeline(ctx);
						return;
					}
					case ERenderStep::RayTracing:
					{
						return;
					}
					case ERenderStep::Invalid:
					default:
					{
						throw std::runtime_error("Unknown render step");
					}
				}
			}

			// Bind a diffuse texture descriptor if one is available.
			template <typename RootParam>
			static void BindDiffuseTexture(MaterialPassContext& ctx, RootParam root_param)
			{
				auto* tex = DiffuseTexture(ctx);
				if (tex == nullptr)
					return;

				if (BindMaterialDescriptor(ctx.m_cmd_list, ctx.m_wnd.m_heap_view, root_param, tex->m_srv, ctx.m_last_tex))
				{
					#if PR_DBG_RDR
					auto state = ctx.m_cmd_list.ResState(tex->m_res.get()).Mip0State();
					assert(AllSet(state, D3D12_RESOURCE_STATE_ALL_SHADER_RESOURCE));
					#endif
				}
			}

			// Bind a diffuse sampler descriptor if one is available.
			template <typename RootParam>
			static void BindDiffuseSampler(MaterialPassContext& ctx, RootParam root_param)
			{
				auto* sam = DiffuseSampler(ctx);
				if (sam == nullptr)
					return;

				BindMaterialDescriptor(ctx.m_cmd_list, ctx.m_wnd.m_heap_samp, root_param, sam->m_samp, ctx.m_last_sam);
			}

			// Bind fixed-function style resources for the simple forward pass.
			static void BindForward(MaterialPassContext& ctx)
			{
				BindDiffuseTexture(ctx, shaders::fwd::ERootParam::DiffTexture);
				BindDiffuseSampler(ctx, shaders::fwd::ERootParam::DiffTextureSampler);
				if (dynamic_cast<shaders::Forward*>(ctx.m_shader) == nullptr)
					throw std::runtime_error("Forward material pass requires a shader");

				// Detail normals use the otherwise idle PBR normal-map slot and their own constant buffer.
				if (auto const* detail = ctx.m_material.Component<materials::DetailNormals>(); detail != nullptr)
					BindDetailNormals(ctx, *detail);
			}

			// Bind the slope map and layer constants used by the detail-normal pixel shader variants.
			static void BindDetailNormals(MaterialPassContext& ctx, materials::DetailNormals const& detail)
			{
				// The slot is validated when the component is set, so both resources are present.
				BindMaterialDescriptor(ctx.m_cmd_list, ctx.m_wnd.m_heap_view, shaders::fwd::ERootParam::PbrNormalTexture, detail.m_tex.m_texture->m_srv, nullptr);
				BindMaterialDescriptor(ctx.m_cmd_list, ctx.m_wnd.m_heap_samp, shaders::fwd::ERootParam::PbrNormalSampler, detail.m_tex.m_sampler->m_samp, nullptr);

				// Pack the shared layers into the shader's per-layer arrays.
				auto const& layers = *detail.m_layers;
				auto cb = shaders::fwd::CBufDetailNormals{};
				static_assert(DetailNormalsMaxLayers == materials::DetailNormalLayers::MaxLayers);
				for (int i = 0; i != layers.m_count; ++i)
				{
					// Copy one layer's projection rows and height scale.
					cb.row_u[i] = layers.m_layers[i].m_row_u;
					cb.row_v[i] = layers.m_layers[i].m_row_v;
					cb.height_scale[i] = layers.m_layers[i].m_height_scale;
				}
				cb.info.x = layers.m_count;

				auto gpu_address = ctx.m_upload.Add(cb, D3D12_CONSTANT_BUFFER_DATA_PLACEMENT_ALIGNMENT, false);
				ctx.m_cmd_list.SetGraphicsRootConstantBufferView((UINT)shaders::fwd::ERootParam::CBufDetailNormals, gpu_address);
			}

			// Bind fixed-function style resources for the simple shadow-map pass.
			static void BindShadowMap(MaterialPassContext& ctx)
			{
				BindDiffuseTexture(ctx, shaders::smap::ERootParam::DiffTexture);
				BindDiffuseSampler(ctx, shaders::smap::ERootParam::DiffTextureSampler);
				if (dynamic_cast<shaders::ShadowMap*>(ctx.m_shader) == nullptr)
					throw std::runtime_error("Shadow-map material pass requires a shadow-map shader");
			}

			// Bind fixed-function style resources for the simple ray-cast pass.
			static void BindRayCast(MaterialPassContext& ctx)
			{
				if (auto* shader = dynamic_cast<shaders::RayCast*>(ctx.m_shader); shader != nullptr)
				{
					shader->SetupElement(ctx.m_cmd_list.get(), ctx.m_upload, &ctx.m_dle, ctx.m_material);
					return;
				}
				throw std::runtime_error("Ray-cast material pass requires a ray-cast shader");
			}

			// Return the effective diffuse texture for the simple material.
			static Texture2D* DiffuseTexture(MaterialPassContext const& ctx)
			{
				auto& base_colour = ctx.m_material.ComponentOrDefault<materials::BaseColour>();
				auto tex = base_colour.m_tex.m_texture;
				return tex != nullptr
					? tex.get()
					: ctx.m_default_tex;
			}

			// Return the effective diffuse sampler for the simple material.
			static Sampler* DiffuseSampler(MaterialPassContext const& ctx)
			{
				auto& base_colour = ctx.m_material.ComponentOrDefault<materials::BaseColour>();
				auto sam = base_colour.m_tex.m_sampler;
				return sam != nullptr
					? sam.get()
					: ctx.m_default_sam;
			}
		};
	}

	// Construct a simple material from default material properties.
	MaterialSimple::MaterialSimple(Colour32 tint, Texture2DPtr tex_diffuse, SamplerPtr sam_diffuse, float rel_reflec)
		: m_base_colour(materials::BaseColour{ Colour(tint), {tex_diffuse, sam_diffuse} })
		, m_reflectivity({rel_reflec})
		, m_shaders()
		, m_two_sided()
		, m_optics()
		, m_detail_normals()
	{}

	// Copy simple material properties into a new ref-counted material instance.
	MaterialSimple::MaterialSimple(MaterialSimple const& rhs)
		: m_base_colour(rhs.m_base_colour)
		, m_reflectivity(rhs.m_reflectivity)
		, m_shaders(rhs.m_shaders)
		, m_two_sided(rhs.m_two_sided)
		, m_optics(rhs.m_optics)
		, m_detail_normals(rhs.m_detail_normals)
	{}

	// Return the extensible type id for this material.
	RdrId MaterialSimple::TypeId() const
	{
		return MaterialTypeId;
	}

	// Return the simple material pass for supported render steps.
	MaterialPass const* MaterialSimple::Pass(ERenderStep step) const
	{
		switch (step)
		{
			case ERenderStep::RenderForward:
			case ERenderStep::ShadowMap:
			case ERenderStep::RayCast:
			case ERenderStep::RayTracing:
			{
				static MaterialSimplePass pass;
				return &pass;
			}
			case ERenderStep::Invalid:
			default:
			{
				return nullptr;
			}
		}
	}

	// Create a mutable copy of this material instance.
	RefPtr<Material> MaterialSimple::Clone() const
	{
		return RefPtr<MaterialSimple>(::pr::compute::New<MaterialSimple>(*this), true);
	}

	// Return true if this material requires alpha rendering
	bool MaterialSimple::RequiresAlpha() const
	{
		return
			HasAlpha(m_base_colour.m_colour) ||
			AllSet(m_base_colour.m_tex.m_texture ? m_base_colour.m_tex.m_texture->m_tflags : ETextureFlag::None, ETextureFlag::HasAlpha);
	}

	// Set the base colour.
	MaterialSimple& MaterialSimple::base_colour(Colour colour)
	{
		m_base_colour.m_colour = colour;
		return *this;
	}
	MaterialSimple& MaterialSimple::base_colour(Colour32 colour)
	{
		return base_colour(Colour(colour));
	}
	MaterialSimple& MaterialSimple::base_texture(Texture2DPtr tex, SamplerPtr sam)
	{
		m_base_colour.m_tex = materials::TextureSlot{
			.m_texture = tex,
			.m_sampler = sam,
		};
		return *this;
	}

	// Set the relative reflectivity.
	MaterialSimple& MaterialSimple::rel_reflec(float reflectivity)
	{
		m_reflectivity.m_rel_reflec = reflectivity;
		return *this;
	}

	// Set whether back-facing pixels should flip their lit surface normal.
	MaterialSimple& MaterialSimple::two_sided(bool enabled)
	{
		m_two_sided.m_enabled = enabled;
		return *this;
	}

	// Add a shader overlay to this material.
	MaterialSimple& MaterialSimple::use_shader_overlay(ERenderStep step, ShaderPtr overlay)
	{
		m_shaders.add(step, overlay);
		return *this;
	}

	// Use 'tex' and 'sam' as the detail-normal slope map. Existing layers are kept; a new component starts with no layers.
	MaterialSimple& MaterialSimple::detail_normals(Texture2DPtr tex, SamplerPtr sam)
	{
		// The pixel shader variant samples the map unconditionally, so both resources are required.
		if (tex == nullptr || sam == nullptr)
			throw std::runtime_error("Detail normals require a texture and a sampler");

		// Enabling starts a new layer set, so this material does not share layers with copies made before detail normals were removed.
		if (!m_detail_normals.m_enable)
		{
			m_detail_normals.m_layers = std::make_shared<materials::DetailNormalLayers>();
			m_detail_normals.m_enable = true;
		}

		m_detail_normals.m_tex.m_texture = tex;
		m_detail_normals.m_tex.m_sampler = sam;
		return *this;
	}

	// Remove detail normals so the stock pixel shaders are used.
	MaterialSimple& MaterialSimple::detail_normals_clear()
	{
		// Reset to the disabled default, which also releases the slope map and the layers.
		m_detail_normals = {};
		return *this;
	}

	// Return a component block for 'component_id', or null if this material does not provide that block.
	void const* MaterialSimple::Component(RdrId component_id) const
	{
		if (component_id == materials::DetailNormals::Id)
			return m_detail_normals.m_enable ? &m_detail_normals : nullptr;

		if (component_id == materials::BaseColour::Id)
			return &m_base_colour;

		if (component_id == materials::Optics::Id)
			return &m_optics;

		if (component_id == materials::Reflectivity::Id)
			return &m_reflectivity;

		if (component_id == materials::ShaderOverlays::Id)
			return &m_shaders;

		if (component_id == materials::TwoSided::Id)
			return &m_two_sided;

		return nullptr;
	}

	// Delete this simple material instance.
	void MaterialSimple::Delete()
	{
		::pr::compute::Delete<MaterialSimple>(this);
	}
}
