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
#include "pr/view3d-12/shaders/shader_ray_cast.h"
#include "pr/view3d-12/shaders/shader_smap.h"
#include "pr/view3d-12/texture/texture_2d.h"

namespace pr::rdr12
{
	namespace
	{
		// The material pass that reproduces default NuggetDesc material handling.
		struct MaterialSimplePass : MaterialPass
		{
			// Return true if this simple material needs alpha rendering.
			bool RequiresAlpha(BaseInstance const&, Material const& material, Nugget const& nugget) const override
			{
				return material.RequiresAlpha() || nugget.RequiresAlpha();
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
					case ERenderStep::GBuffer:
					case ERenderStep::DSLighting:
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
					case ERenderStep::GBuffer:
					case ERenderStep::DSLighting:
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

				// Apply legacy shader overlays for the simple forward pass.
				static auto ApplyForwardPipeline = [](MaterialPassContext& ctx)
				{
					auto const* overlays = ctx.m_material.Component<materials::ShaderOverlays>();
					if (overlays == nullptr)
						return;

					for (auto& shdr_overlay : overlays->m_overlays)
					{
						if (shdr_overlay.m_rdr_step != ctx.m_step_id)
							continue;

						auto& overlay = *shdr_overlay.m_overlay.get();
						if (overlay.m_signature)
						{
							ctx.m_pipe_state.Apply(PSO<EPipeState::RootSignature>(overlay.m_signature.get()));
							ctx.m_cmd_list.SetGraphicsRootSignature(overlay.m_signature.get());
							ctx.m_root_signature_changed = true;
						}
						if (overlay.m_code.VS) ctx.m_pipe_state.Apply(PSO<EPipeState::VS>(overlay.m_code.VS));
						if (overlay.m_code.PS) ctx.m_pipe_state.Apply(PSO<EPipeState::PS>(overlay.m_code.PS));
						if (overlay.m_code.DS) ctx.m_pipe_state.Apply(PSO<EPipeState::DS>(overlay.m_code.DS));
						if (overlay.m_code.HS) ctx.m_pipe_state.Apply(PSO<EPipeState::HS>(overlay.m_code.HS));
						if (overlay.m_code.GS) ctx.m_pipe_state.Apply(PSO<EPipeState::GS>(overlay.m_code.GS));

						overlay.SetupFrame(ctx.m_cmd_list.get(), ctx.m_upload, ctx.m_scene);
						overlay.SetupElement(ctx.m_cmd_list.get(), ctx.m_upload, ctx.m_scene, &ctx.m_dle);
					}
				};

				switch (ctx.m_step_id)
				{
					case ERenderStep::RenderForward:
					{
						ApplyForwardPipeline(ctx);
						ApplyTwoSidedPipeline(ctx);
						return;
					}
					case ERenderStep::ShadowMap:
					case ERenderStep::RayCast:
					{
						ApplyTwoSidedPipeline(ctx);
						return;
					}
					case ERenderStep::RayTracing:
					case ERenderStep::GBuffer:
					case ERenderStep::DSLighting:
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

				auto srv_descriptor = ctx.m_wnd.m_heap_view.Add(tex->m_srv);
				if (ctx.m_last_tex == nullptr || srv_descriptor.ptr != ctx.m_last_tex->ptr)
				{
					ctx.m_cmd_list.SetGraphicsRootDescriptorTable(root_param, srv_descriptor);
					if (ctx.m_last_tex != nullptr)
						*ctx.m_last_tex = srv_descriptor;

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

				auto sam_descriptor = ctx.m_wnd.m_heap_samp.Add(sam->m_samp);
				if (ctx.m_last_sam == nullptr || sam_descriptor.ptr != ctx.m_last_sam->ptr)
				{
					ctx.m_cmd_list.SetGraphicsRootDescriptorTable(root_param, sam_descriptor);
					if (ctx.m_last_sam != nullptr)
						*ctx.m_last_sam = sam_descriptor;
				}
			}

			// Bind fixed-function style resources for the simple forward pass.
			static void BindForward(MaterialPassContext& ctx)
			{
				BindDiffuseTexture(ctx, shaders::fwd::ERootParam::DiffTexture);
				BindDiffuseSampler(ctx, shaders::fwd::ERootParam::DiffTextureSampler);
				if (auto* shader = dynamic_cast<shaders::Forward*>(ctx.m_shader); shader != nullptr)
				{
					shader->SetupElement(ctx.m_cmd_list.get(), ctx.m_upload, ctx.m_scene, &ctx.m_dle, ctx.m_material);
					return;
				}
				else
				{
					throw std::runtime_error("Forward material pass requires a shader");
				}
			}

			// Bind fixed-function style resources for the simple shadow-map pass.
			static void BindShadowMap(MaterialPassContext& ctx)
			{
				BindDiffuseTexture(ctx, shaders::smap::ERootParam::DiffTexture);
				BindDiffuseSampler(ctx, shaders::smap::ERootParam::DiffTextureSampler);
				if (auto* shader = dynamic_cast<shaders::ShadowMap*>(ctx.m_shader); shader != nullptr)
				{
					shader->SetupElement(ctx.m_cmd_list.get(), ctx.m_upload, &ctx.m_dle, ctx.m_scene.m_cam, ctx.m_material);
					return;
				}
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
				auto& inst = *ctx.m_dle.m_instance;
				auto& base_colour = ctx.m_material.ComponentOrDefault<materials::BaseColour>();
				auto tex = coalesce(FindDiffTexture(inst), base_colour.m_tex.m_texture);
				return tex != nullptr
					? tex.get()
					: ctx.m_default_tex;
			}

			// Return the effective diffuse sampler for the simple material.
			static Sampler* DiffuseSampler(MaterialPassContext const& ctx)
			{
				auto& inst = *ctx.m_dle.m_instance;
				auto& base_colour = ctx.m_material.ComponentOrDefault<materials::BaseColour>();
				auto sam = coalesce(FindDiffTextureSampler(inst), base_colour.m_tex.m_sampler);
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
	{}

	// Copy simple material properties into a new ref-counted material instance.
	MaterialSimple::MaterialSimple(MaterialSimple const& rhs)
		: m_base_colour(rhs.m_base_colour)
		, m_reflectivity(rhs.m_reflectivity)
		, m_shaders(rhs.m_shaders)
		, m_two_sided(rhs.m_two_sided)
		, m_optics(rhs.m_optics)
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
			case ERenderStep::GBuffer:
			case ERenderStep::DSLighting:
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

	// Return a component block for 'component_id', or null if this material does not provide that block.
	void const* MaterialSimple::Component(RdrId component_id) const
	{
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
