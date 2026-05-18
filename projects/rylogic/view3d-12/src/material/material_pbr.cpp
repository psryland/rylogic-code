//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2026
//*********************************************
#include "pr/view3d-12/material/material_pbr.h"
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
#include "pr/view3d-12/utility/pipe_state.h"
#include "view3d-12/src/shaders/common.h"

namespace pr::rdr12
{
	namespace
	{
		// The material pass used by physically-based materials.
		struct MaterialPBRPass : MaterialPass
		{
			// Return true if this PBR material needs alpha rendering.
			bool RequiresAlpha(BaseInstance const&, Material const& material, Nugget const& nugget) const override
			{
				return material.RequiresAlpha() || nugget.RequiresAlpha();
			}

			// Contribute PBR texture and shader state to the sort key.
			SortKey AddSortKey(ERenderStep step, BaseInstance const&, Material const& material, Nugget const&, SortKey key) const override
			{
				switch (step)
				{
					case ERenderStep::RenderForward:
					{
						if (auto const* base_colour = material.Component<materials::BaseColour>(); base_colour != nullptr)
						{
							if (!AnySet(key, SortKey::TextureIdMask) && base_colour->m_tex.m_texture != nullptr)
								key = SetBits(key, SortKey::TextureIdMask, base_colour->m_tex.m_texture->SortId() << SortKey::TextureIdOfs);
						}

						if (!AnySet(key, SortKey::ShaderIdMask))
						{
							auto shader_id = static_cast<SortKey::value_type>(MaterialPBR::MaterialTypeId) & (SortKey::MaxShaderId - 1U);
							key = SetBits(key, SortKey::ShaderIdMask, shader_id << SortKey::ShaderIdOfs);
						}
						return key;
					}
					case ERenderStep::ShadowMap:
					{
						if (auto const* base_colour = material.Component<materials::BaseColour>(); base_colour != nullptr)
						{
							if (!AnySet(key, SortKey::TextureIdMask) && base_colour->m_tex.m_texture != nullptr)
								key = SetBits(key, SortKey::TextureIdMask, base_colour->m_tex.m_texture->SortId() << SortKey::TextureIdOfs);
						}
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

			// Bind resources and constants for the PBR material pass.
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

			// Apply pipeline changes for the PBR material pass.
			void ApplyPipeline(MaterialPassContext& ctx) const override
			{
				switch (ctx.m_step_id)
				{
					case ERenderStep::RenderForward:
					{
						ApplyForwardPipeline(ctx);
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

			// Return true if a texture slot can be sampled by this draw.
			static bool HasUsableTexture(MaterialPassContext const& ctx, materials::TextureSlot const& slot)
			{
				return
					ctx.m_dle.m_nugget != nullptr &&
					AllSet(ctx.m_dle.m_nugget->m_geom, EGeom::Tex0) &&
					slot.m_texture != nullptr;
			}

			// Return the texture for a PBR slot, or the forward pass fallback texture.
			static Texture2D* TextureOrDefault(MaterialPassContext const& ctx, materials::TextureSlot const& slot)
			{
				auto texture = slot.m_texture;
				return texture != nullptr ? texture.get() : ctx.m_default_tex;
			}

			// Return the sampler for a PBR slot, or the forward pass fallback sampler.
			static Sampler* SamplerOrDefault(MaterialPassContext const& ctx, materials::TextureSlot const& slot)
			{
				auto sampler = slot.m_sampler;
				return sampler != nullptr ? sampler.get() : ctx.m_default_sam;
			}

			// Return true when a colour texture needs shader-side sRGB decoding.
			static bool NeedsShaderSrgbDecode(materials::TextureSlot const& slot)
			{
				switch (slot.m_colour_space)
				{
					case materials::ETextureColourSpace::Linear:
					{
						return false;
					}
					case materials::ETextureColourSpace::Srgb:
					{
						return slot.m_texture != nullptr && !::pr::compute::IsSRGB(slot.m_texture->TexDesc().Format);
					}
					default:
					{
						throw std::runtime_error("Unknown texture colour-space");
					}
				}
			}

			// Convert a scalar texture channel into the shader channel index.
			static int ShaderChannel(materials::ETextureChannel channel)
			{
				return static_cast<int>(channel);
			}

			// Bind a PBR texture descriptor.
			template <typename RootParam>
			static void BindTexture(MaterialPassContext& ctx, RootParam root_param, materials::TextureSlot const& slot, bool cache_diffuse_slot)
			{
				auto* tex = TextureOrDefault(ctx, slot);
				if (tex == nullptr)
					return;

				auto srv_descriptor = ctx.m_wnd.m_heap_view.Add(tex->m_srv);
				if (!cache_diffuse_slot || ctx.m_last_tex == nullptr || srv_descriptor.ptr != ctx.m_last_tex->ptr)
				{
					ctx.m_cmd_list.SetGraphicsRootDescriptorTable(root_param, srv_descriptor);
					if (cache_diffuse_slot && ctx.m_last_tex != nullptr)
						*ctx.m_last_tex = srv_descriptor;

					#if PR_DBG_RDR
					auto state = ctx.m_cmd_list.ResState(tex->m_res.get()).Mip0State();
					assert(AllSet(state, D3D12_RESOURCE_STATE_ALL_SHADER_RESOURCE));
					#endif
				}
			}

			// Bind a PBR sampler descriptor.
			template <typename RootParam>
			static void BindSampler(MaterialPassContext& ctx, RootParam root_param, materials::TextureSlot const& slot, bool cache_diffuse_slot)
			{
				auto* sam = SamplerOrDefault(ctx, slot);
				if (sam == nullptr)
					return;

				auto sam_descriptor = ctx.m_wnd.m_heap_samp.Add(sam->m_samp);
				if (!cache_diffuse_slot || ctx.m_last_sam == nullptr || sam_descriptor.ptr != ctx.m_last_sam->ptr)
				{
					ctx.m_cmd_list.SetGraphicsRootDescriptorTable(root_param, sam_descriptor);
					if (cache_diffuse_slot && ctx.m_last_sam != nullptr)
						*ctx.m_last_sam = sam_descriptor;
				}
			}

			// Bind scalar PBR material constants.
			static void BindPbrConstants(MaterialPassContext& ctx)
			{
				auto const& base_colour = *ctx.m_material.Component<materials::BaseColour>();
				auto const& emissive = *ctx.m_material.Component<materials::Emissive>();
				auto const& metallic = *ctx.m_material.Component<materials::Metallic>();
				auto const& roughness = *ctx.m_material.Component<materials::Roughness>();
				auto const& alpha = *ctx.m_material.Component<materials::Alpha>();
				auto texture_flags = 0;
				if (HasUsableTexture(ctx, base_colour.m_tex) && NeedsShaderSrgbDecode(base_colour.m_tex))
					texture_flags |= shaders::PbrTextureFlag_BaseColourSrgb;
				if (HasUsableTexture(ctx, metallic.m_tex.m_slot))
					texture_flags |= shaders::PbrTextureFlag_HasMetallicMap;
				if (HasUsableTexture(ctx, roughness.m_tex.m_slot))
					texture_flags |= shaders::PbrTextureFlag_HasRoughnessMap;
				if (HasUsableTexture(ctx, emissive.m_tex))
				{
					texture_flags |= shaders::PbrTextureFlag_HasEmissiveMap;
					if (NeedsShaderSrgbDecode(emissive.m_tex))
						texture_flags |= shaders::PbrTextureFlag_EmissiveSrgb;
				}

				auto cb = shaders::fwd::CBufPbrSurface{
					.base_colour = base_colour.m_colour.rgba,
					.emissive = emissive.m_colour.rgba,
					.metallic = metallic.m_factor,
					.roughness = roughness.m_factor,
					.alpha_cutoff = alpha.m_cutoff,
					.alpha_mode = static_cast<int>(alpha.m_mode),
					.texture_flags = texture_flags,
					.metallic_channel = ShaderChannel(metallic.m_tex.m_channel),
					.roughness_channel = ShaderChannel(roughness.m_tex.m_channel),
				};
				auto gpu_address = ctx.m_upload.Add(cb, D3D12_CONSTANT_BUFFER_DATA_PLACEMENT_ALIGNMENT, false);
				ctx.m_cmd_list.SetGraphicsRootConstantBufferView((UINT)shaders::fwd::ERootParam::CBufPbrSurface, gpu_address);
			}

			// Bind resources and constants for the PBR forward pass.
			static void BindForward(MaterialPassContext& ctx)
			{
				auto const& base_colour = *ctx.m_material.Component<materials::BaseColour>();
				auto const& metallic = *ctx.m_material.Component<materials::Metallic>();
				auto const& roughness = *ctx.m_material.Component<materials::Roughness>();
				auto const& emissive = *ctx.m_material.Component<materials::Emissive>();

				BindTexture(ctx, shaders::fwd::ERootParam::DiffTexture, base_colour.m_tex, true);
				BindSampler(ctx, shaders::fwd::ERootParam::DiffTextureSampler, base_colour.m_tex, true);
				BindTexture(ctx, shaders::fwd::ERootParam::PbrMetallicTexture, metallic.m_tex.m_slot, false);
				BindSampler(ctx, shaders::fwd::ERootParam::PbrMetallicSampler, metallic.m_tex.m_slot, false);
				BindTexture(ctx, shaders::fwd::ERootParam::PbrRoughnessTexture, roughness.m_tex.m_slot, false);
				BindSampler(ctx, shaders::fwd::ERootParam::PbrRoughnessSampler, roughness.m_tex.m_slot, false);
				BindTexture(ctx, shaders::fwd::ERootParam::PbrEmissiveTexture, emissive.m_tex, false);
				BindSampler(ctx, shaders::fwd::ERootParam::PbrEmissiveSampler, emissive.m_tex, false);

				if (auto* shader = dynamic_cast<shaders::Forward*>(ctx.m_shader); shader != nullptr)
				{
					shader->SetupElement(ctx.m_cmd_list.get(), ctx.m_upload, ctx.m_scene, &ctx.m_dle, ctx.m_material);
					BindPbrConstants(ctx);
					return;
				}

				throw std::runtime_error("PBR forward material pass requires a forward shader");
			}

			// Bind resources and constants for the PBR shadow-map pass.
			static void BindShadowMap(MaterialPassContext& ctx)
			{
				auto const& base_colour = *ctx.m_material.Component<materials::BaseColour>();
				BindTexture(ctx, shaders::smap::ERootParam::DiffTexture, base_colour.m_tex, true);
				BindSampler(ctx, shaders::smap::ERootParam::DiffTextureSampler, base_colour.m_tex, true);
				if (auto* shader = dynamic_cast<shaders::ShadowMap*>(ctx.m_shader); shader != nullptr)
				{
					shader->SetupElement(ctx.m_cmd_list.get(), ctx.m_upload, &ctx.m_dle, ctx.m_scene.m_cam, ctx.m_material);
					return;
				}

				throw std::runtime_error("PBR shadow-map material pass requires a shadow-map shader");
			}

			// Bind resources and constants for the PBR ray-cast pass.
			static void BindRayCast(MaterialPassContext& ctx)
			{
				if (auto* shader = dynamic_cast<shaders::RayCast*>(ctx.m_shader); shader != nullptr)
				{
					shader->SetupElement(ctx.m_cmd_list.get(), ctx.m_upload, &ctx.m_dle, ctx.m_material);
					return;
				}

				throw std::runtime_error("PBR ray-cast material pass requires a ray-cast shader");
			}

			// Force two-sided rasterisation for PBR surfaces that need normal flipping on back faces.
			static void ApplyTwoSidedPipeline(MaterialPassContext& ctx)
			{
				auto const* two_sided = ctx.m_material.Component<materials::TwoSided>();
				if (two_sided == nullptr || !two_sided->m_enabled)
					return;

				if (ctx.m_dle.m_nugget->m_variant == AlphaNugget)
					return;

				ctx.m_pipe_state.Apply(PSO<EPipeState::CullMode>(D3D12_CULL_MODE_NONE));
			}

			// Apply the PBR forward shader variant for the active forward sub-pass.
			static void ApplyForwardPipeline(MaterialPassContext& ctx)
			{
				auto const* desc = static_cast<D3D12_GRAPHICS_PIPELINE_STATE_DESC const*>(ctx.m_pipe_state);
				auto const& ps =
					desc->NumRenderTargets == 0U ? shader_code::forward_pbr_alpha_collect_ps :
					desc->NumRenderTargets > 1U ? shader_code::forward_pbr_reflection_attrs_ps :
					shader_code::forward_pbr_ps;

				ctx.m_pipe_state.Apply(PSO<EPipeState::PS>(ps));
				ApplyTwoSidedPipeline(ctx);
			}
		};
	}

	// Construct a PBR material from default physically-based properties.
	MaterialPBR::MaterialPBR()
		: m_base_colour()
		, m_metallic()
		, m_roughness()
		, m_emissive()
		, m_normal_map()
		, m_alpha()
		, m_two_sided()
	{}

	// Copy PBR material properties into a new ref-counted material instance.
	MaterialPBR::MaterialPBR(MaterialPBR const& rhs)
		: m_base_colour(rhs.m_base_colour)
		, m_metallic(rhs.m_metallic)
		, m_roughness(rhs.m_roughness)
		, m_emissive(rhs.m_emissive)
		, m_normal_map(rhs.m_normal_map)
		, m_alpha(rhs.m_alpha)
		, m_two_sided(rhs.m_two_sided)
	{}

	// Return the extensible type id for this material.
	RdrId MaterialPBR::TypeId() const
	{
		return MaterialTypeId;
	}

	// Return the PBR material pass for supported render steps.
	MaterialPass const* MaterialPBR::Pass(ERenderStep step) const
	{
		switch (step)
		{
			case ERenderStep::RenderForward:
			case ERenderStep::ShadowMap:
			case ERenderStep::RayCast:
			{
				static MaterialPBRPass pass;
				return &pass;
			}
			case ERenderStep::RayTracing:
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
	RefPtr<Material> MaterialPBR::Clone() const
	{
		return RefPtr<MaterialPBR>(::pr::compute::New<MaterialPBR>(*this), true);
	}

	// Return true if this material requires alpha rendering.
	bool MaterialPBR::RequiresAlpha() const
	{
		return m_alpha.RequiresAlpha();
	}

	// Return the material colour that should be folded into the shared nugget tint constant.
	Colour MaterialPBR::TintColour() const
	{
		return ColourWhite;
	}

	// Set the linear base-colour factor.
	MaterialPBR& MaterialPBR::base_colour(Colour colour)
	{
		m_base_colour.m_colour = colour;
		return *this;
	}
	MaterialPBR& MaterialPBR::base_colour(Colour32 colour)
	{
		return base_colour(Colour(colour));
	}
	MaterialPBR& MaterialPBR::base_texture(Texture2DPtr tex, SamplerPtr sam)
	{
		return base_texture(materials::TextureSlot{
			.m_texture = tex,
			.m_sampler = sam,
			.m_colour_space = materials::ETextureColourSpace::Srgb,
		});
	}
	MaterialPBR& MaterialPBR::base_texture(materials::TextureSlot slot)
	{
		m_base_colour.m_tex = slot;
		return *this;
	}

	// Set the metallic factor.
	MaterialPBR& MaterialPBR::metallic(float value)
	{
		m_metallic.m_factor = value;
		return *this;
	}
	MaterialPBR& MaterialPBR::metallic_texture(materials::ScalarTextureSlot slot)
	{
		m_metallic.m_tex = slot;
		return *this;
	}

	// Set the roughness factor.
	MaterialPBR& MaterialPBR::roughness(float value)
	{
		m_roughness.m_factor = value;
		return *this;
	}
	MaterialPBR& MaterialPBR::roughness_texture(materials::ScalarTextureSlot slot)
	{
		m_roughness.m_tex = slot;
		return *this;
	}

	// Set the linear emissive factor.
	MaterialPBR& MaterialPBR::emissive(Colour colour)
	{
		m_emissive.m_colour = colour;
		return *this;
	}
	MaterialPBR& MaterialPBR::emissive_texture(materials::TextureSlot slot)
	{
		m_emissive.m_tex = slot;
		return *this;
	}

	// Set the texture slot reserved for tangent-space normals.
	MaterialPBR& MaterialPBR::normal_texture(materials::TextureSlot slot)
	{
		m_normal_map.m_tex = slot;
		return *this;
	}

	// Set the alpha interpretation for this material.
	MaterialPBR& MaterialPBR::alpha_mode(materials::EAlphaMode mode, float cutoff)
	{
		m_alpha.m_mode = mode;
		m_alpha.m_cutoff = cutoff;
		return *this;
	}

	// Get/Set whether back-facing pixels should flip their lit surface normal.
	bool MaterialPBR::two_sided() const
	{
		return m_two_sided.m_enabled;
	}
	MaterialPBR& MaterialPBR::two_sided(bool enabled)
	{
		m_two_sided.m_enabled = enabled;
		return *this;
	}

	// Return a component block for 'component_id', or null if this material does not provide that block.
	void const* MaterialPBR::Component(RdrId component_id) const
	{
		if (component_id == materials::BaseColour::Id)
			return &m_base_colour;

		if (component_id == materials::Metallic::Id)
			return &m_metallic;

		if (component_id == materials::Roughness::Id)
			return &m_roughness;

		if (component_id == materials::Emissive::Id)
			return &m_emissive;

		if (component_id == materials::NormalMap::Id)
			return &m_normal_map;

		if (component_id == materials::Alpha::Id)
			return &m_alpha;

		if (component_id == materials::TwoSided::Id)
			return &m_two_sided;

		return nullptr;
	}

	// Delete this PBR material instance.
	void MaterialPBR::Delete()
	{
		::pr::compute::Delete<MaterialPBR>(this);
	}
}

