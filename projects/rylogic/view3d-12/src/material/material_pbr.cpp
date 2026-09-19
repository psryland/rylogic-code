//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2026
//*********************************************
#include "pr/view3d-12/material/material_pbr.h"
#include "pr/view3d-12/instance/instance.h"
#include "pr/view3d-12/main/window.h"
#include "pr/view3d-12/model/vertex_stream.h"
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
		// Wrap an unbounded lattice coordinate into the shader's deterministic 32-bit cell domain.
		int32_t WrappedCell(double cell)
		{
			// Preserve the fractional coordinate separately while defining deliberate modulo-2^32 lattice addressing.
			constexpr auto period = 4294967296.0;
			auto wrapped = std::fmod(cell, period);
			if (wrapped < 0.0)
				wrapped += period;

			return static_cast<int32_t>(static_cast<uint32_t>(wrapped));
		}

		// Convert a procedural surface coordinate contract into precision-preserving shader rows.
		void SetProceduralConstants(shaders::fwd::CBufPbrSurface& cb, MaterialPassContext const& ctx, materials::ProceduralSurface const& surface)
		{
			// Validate at the public material boundary before deriving coordinates or uploading shader constants.
			surface.Validate();
			auto coordinate_from_world = m4x4::Identity();
			switch (surface.m_coordinate_space)
			{
				case materials::EProceduralCoordinateSpace::World:
				{
					break;
				}
				case materials::EProceduralCoordinateSpace::Object:
				{
					coordinate_from_world = Invert(GetO2W(*ctx.m_dle.m_instance));
					break;
				}
				default:
				{
					throw std::runtime_error("Unknown procedural surface coordinate space");
				}
			}

			// Split translation into an integer lattice cell and a small fractional value so camera motion and large origins do not consume shader mantissa bits.
			auto scaled_translation = std::array<double, 3>{
				(static_cast<double>(coordinate_from_world.w.x) - surface.m_coordinate_origin.x) * surface.m_axis_scale.x / surface.m_feature_scale,
				(static_cast<double>(coordinate_from_world.w.y) - surface.m_coordinate_origin.y) * surface.m_axis_scale.y / surface.m_feature_scale,
				(static_cast<double>(coordinate_from_world.w.z) - surface.m_coordinate_origin.z) * surface.m_axis_scale.z / surface.m_feature_scale,
			};
			auto cell = std::array<double, 3>{
				std::floor(scaled_translation[0]),
				std::floor(scaled_translation[1]),
				std::floor(scaled_translation[2]),
			};
			auto reciprocal_scale = v4{
				surface.m_axis_scale.x / surface.m_feature_scale,
				surface.m_axis_scale.y / surface.m_feature_scale,
				surface.m_axis_scale.z / surface.m_feature_scale,
				0,
			};

			// Upload generic channel and coordinate parameters; presets are only factories for these values.
			cb.procedural_colour0 = surface.m_palette[0].rgba;
			cb.procedural_colour1 = surface.m_palette[1].rgba;
			cb.procedural_colour2 = surface.m_palette[2].rgba;
			cb.procedural_colour3 = surface.m_palette[3].rgba;
			cb.procedural_coord_x = v4{
				coordinate_from_world.x.x * reciprocal_scale.x,
				coordinate_from_world.y.x * reciprocal_scale.x,
				coordinate_from_world.z.x * reciprocal_scale.x,
				static_cast<float>(scaled_translation[0] - cell[0]),
			};
			cb.procedural_coord_y = v4{
				coordinate_from_world.x.y * reciprocal_scale.y,
				coordinate_from_world.y.y * reciprocal_scale.y,
				coordinate_from_world.z.y * reciprocal_scale.y,
				static_cast<float>(scaled_translation[1] - cell[1]),
			};
			cb.procedural_coord_z = v4{
				coordinate_from_world.x.z * reciprocal_scale.z,
				coordinate_from_world.y.z * reciprocal_scale.z,
				coordinate_from_world.z.z * reciprocal_scale.z,
				static_cast<float>(scaled_translation[2] - cell[2]),
			};
			cb.procedural_cell_seed = iv4{
				WrappedCell(cell[0]),
				WrappedCell(cell[1]),
				WrappedCell(cell[2]),
				static_cast<int32_t>(surface.m_seed),
			};
			cb.procedural_params0 = v4{1, surface.m_normal_strength, surface.m_roughness_min, surface.m_roughness_max};
			cb.procedural_params1 = v4{surface.m_detail, surface.m_warp, 0, 0};
		}

		// The material pass used by physically-based materials.
		struct MaterialPBRPass : MaterialPass
		{
			static constexpr int MaxExtraTexCoordStreams = 4;

			// Return true if this PBR material needs alpha rendering.
			bool RequiresAlpha(BaseInstance const&, Material const& material, Nugget const& nugget) const override
			{
				return
					material.RequiresAlpha() ||
					AnySet(nugget.m_nflags, ENuggetFlag::GeometryHasAlpha | ENuggetFlag::AlphaBlend);
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
						materials::ApplyShaderOverlays(ctx, false);
						return;
					}
					case ERenderStep::ShadowMap:
					case ERenderStep::RayCast:
					{
						materials::ApplyShaderOverlays(ctx, true);
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

			// Describes the draw-time mapping from material source UV channels to shader texture-coordinate lanes.
			struct TexCoordBindings
			{
				std::array<VertexStream const*, MaxExtraTexCoordStreams> m_streams = {};
				std::array<int, MaxExtraTexCoordStreams> m_texcoords = {};
				int m_count = 0;

				// Return the shader lane used by a source UV channel, or -1 if it is not bound.
				int Lane(int texcoord) const
				{
					if (texcoord == 0)
						return 0;

					for (auto i = 0; i != m_count; ++i)
					{
						if (m_texcoords[i] == texcoord)
							return i + 1;
					}
					return -1;
				}

				// Return the shader lane used by a material texture slot.
				int Lane(materials::TextureSlot const& slot) const
				{
					return Lane(slot.m_texcoord);
				}
			};

			// Return the PBR texture slots in the order used when assigning shader UV lanes.
			static std::array<materials::TextureSlot const*, 5> TextureSlots(MaterialPassContext const& ctx)
			{
				auto const* base_colour = ctx.m_material.Component<materials::BaseColour>();
				auto const* metallic = ctx.m_material.Component<materials::Metallic>();
				auto const* roughness = ctx.m_material.Component<materials::Roughness>();
				auto const* emissive = ctx.m_material.Component<materials::Emissive>();
				auto const* normal_map = ctx.m_material.Component<materials::NormalMap>();
				return {
					base_colour != nullptr ? &base_colour->m_tex : nullptr,
					metallic != nullptr ? &metallic->m_tex.m_slot : nullptr,
					roughness != nullptr ? &roughness->m_tex.m_slot : nullptr,
					emissive != nullptr ? &emissive->m_tex : nullptr,
					normal_map != nullptr ? &normal_map->m_tex : nullptr,
				};
			}

			// Return the model vertex stream that contains a source texture-coordinate channel.
			static VertexStream const* FindTexCoordStream(MaterialPassContext const& ctx, int texcoord)
			{
				auto const* nugget = ctx.m_dle.m_nugget;
				if (nugget == nullptr || nugget->m_model == nullptr)
					return nullptr;

				return nugget->m_model->FindVertexStream(vertex_stream::TexCoord(texcoord));
			}

			// Return the model vertex streams that need shader UV lanes for this draw.
			static TexCoordBindings GatherTexCoordBindings(MaterialPassContext const& ctx)
			{
				auto bindings = TexCoordBindings{};
				for (auto const* slot : TextureSlots(ctx))
				{
					if (slot == nullptr || slot->m_texture == nullptr || slot->m_texcoord == 0)
						continue;
					if (bindings.Lane(slot->m_texcoord) != -1)
						continue;

					auto const* stream = FindTexCoordStream(ctx, slot->m_texcoord);
					if (stream == nullptr)
						continue;

					if (bindings.m_count == MaxExtraTexCoordStreams)
						throw std::runtime_error("PBR material uses too many texture coordinate streams");

					bindings.m_texcoords[bindings.m_count] = slot->m_texcoord;
					bindings.m_streams[bindings.m_count] = stream;
					++bindings.m_count;
				}
				return bindings;
			}

			// Return true if a texture slot can be sampled by this draw.
			static bool HasUsableTexture(MaterialPassContext const& ctx, materials::TextureSlot const& slot, TexCoordBindings const& bindings)
			{
				if (slot.m_texture == nullptr || ctx.m_dle.m_nugget == nullptr)
					return false;

				if (slot.m_texcoord == 0)
					return AllSet(ctx.m_dle.m_nugget->m_geom, EGeom::Tex0);

				return bindings.Lane(slot) != -1;
			}

			// Return the shader lane for a usable texture slot, falling back to tex0 for inactive slots.
			static int ShaderTexCoord(TexCoordBindings const& bindings, materials::TextureSlot const& slot)
			{
				auto lane = bindings.Lane(slot);
				return lane != -1 ? lane : 0;
			}

			// Convert a material texture-coordinate transform into the shader constant layout.
			static shaders::TexXForm ShaderTexXForm(TexXForm const& transform)
			{
				return {
					.m_x = transform.m_x,
					.m_y = transform.m_y,
				};
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
			static void BindPbrConstants(MaterialPassContext& ctx, TexCoordBindings const& texcoords)
			{
				auto const& base_colour = *ctx.m_material.Component<materials::BaseColour>();
				auto const& emissive = *ctx.m_material.Component<materials::Emissive>();
				auto const& metallic = *ctx.m_material.Component<materials::Metallic>();
				auto const& roughness = *ctx.m_material.Component<materials::Roughness>();
				auto const& alpha = *ctx.m_material.Component<materials::Alpha>();
				auto const& normal_map = *ctx.m_material.Component<materials::NormalMap>();
				auto const* procedural_surface = ctx.m_material.Component<materials::ProceduralSurface>();

				auto texture_flags = 0;
				if (HasUsableTexture(ctx, base_colour.m_tex, texcoords))
				{
					texture_flags |= shaders::PbrTextureFlag_HasBaseColourMap;
					if (NeedsShaderSrgbDecode(base_colour.m_tex))
						texture_flags |= shaders::PbrTextureFlag_BaseColourSrgb;
				}
				if (HasUsableTexture(ctx, metallic.m_tex.m_slot, texcoords))
				{
					texture_flags |= shaders::PbrTextureFlag_HasMetallicMap;
				}
				if (HasUsableTexture(ctx, roughness.m_tex.m_slot, texcoords))
				{
					texture_flags |= shaders::PbrTextureFlag_HasRoughnessMap;
				}
				if (HasUsableTexture(ctx, emissive.m_tex, texcoords))
				{
					texture_flags |= shaders::PbrTextureFlag_HasEmissiveMap;
					if (NeedsShaderSrgbDecode(emissive.m_tex))
						texture_flags |= shaders::PbrTextureFlag_EmissiveSrgb;
				}
				if (HasUsableTexture(ctx, normal_map.m_tex, texcoords))
				{
					texture_flags |= shaders::PbrTextureFlag_HasNormalMap;
				}

				auto cb = shaders::fwd::CBufPbrSurface{
					.base_colour = base_colour.m_colour.rgba,
					.emissive = emissive.m_colour.rgba,
					.base_colour_uv_transform = ShaderTexXForm(base_colour.m_tex.m_uv_transform),
					.metallic_uv_transform = ShaderTexXForm(metallic.m_tex.m_slot.m_uv_transform),
					.roughness_uv_transform = ShaderTexXForm(roughness.m_tex.m_slot.m_uv_transform),
					.emissive_uv_transform = ShaderTexXForm(emissive.m_tex.m_uv_transform),
					.normal_uv_transform = ShaderTexXForm(normal_map.m_tex.m_uv_transform),
					.metallic = metallic.m_factor,
					.roughness = roughness.m_factor,
					.normal_scale = normal_map.m_scale,
					.alpha_cutoff = alpha.m_cutoff,
					.alpha_mode = static_cast<int>(alpha.m_mode),
					.texture_flags = texture_flags,
					.metallic_channel = ShaderChannel(metallic.m_tex.m_channel),
					.roughness_channel = ShaderChannel(roughness.m_tex.m_channel),
					.base_colour_texcoord = ShaderTexCoord(texcoords, base_colour.m_tex),
					.metallic_texcoord = ShaderTexCoord(texcoords, metallic.m_tex.m_slot),
					.roughness_texcoord = ShaderTexCoord(texcoords, roughness.m_tex.m_slot),
					.emissive_texcoord = ShaderTexCoord(texcoords, emissive.m_tex),
					.normal_texcoord = ShaderTexCoord(texcoords, normal_map.m_tex),
					.texcoord_count = texcoords.m_count,
				};
				if (procedural_surface != nullptr)
					SetProceduralConstants(cb, ctx, *procedural_surface);

				auto gpu_address = ctx.m_upload.Add(cb, D3D12_CONSTANT_BUFFER_DATA_PLACEMENT_ALIGNMENT, false);
				ctx.m_cmd_list.SetGraphicsRootConstantBufferView((UINT)shaders::fwd::ERootParam::CBufPbrSurface, gpu_address);
			}

			// Return the root parameter that binds a shader texture-coordinate lane.
			static shaders::fwd::ERootParam TexCoordRootParam(int lane)
			{
				switch (lane)
				{
					case 1: { return shaders::fwd::ERootParam::Tex1Stream; }
					case 2: { return shaders::fwd::ERootParam::Tex2Stream; }
					case 3: { return shaders::fwd::ERootParam::Tex3Stream; }
					case 4: { return shaders::fwd::ERootParam::Tex4Stream; }
					default:
					{
						throw std::runtime_error("Unknown shader texture coordinate lane");
					}
				}
			}

			// Bind the model vertex streams used by PBR texture-coordinate lanes.
			static void BindTexCoordStreams(MaterialPassContext& ctx, TexCoordBindings const& texcoords)
			{
				if (texcoords.m_count == 0)
					return;

				// The TexN shader statically references every optional lane, so bind a valid SRV for inactive lanes rather than leaving stale
				// descriptors behind.
				for (auto index = 0; index != MaxExtraTexCoordStreams; ++index)
				{
					auto const* stream = index < texcoords.m_count ? texcoords.m_streams[index] : texcoords.m_streams[0];
					auto srv_descriptor = ctx.m_wnd.m_heap_view.Add(stream->m_srv);
					ctx.m_cmd_list.SetGraphicsRootDescriptorTable(TexCoordRootParam(index + 1), srv_descriptor);
				}
			}

			// Bind resources and constants for the PBR forward pass.
			static void BindForward(MaterialPassContext& ctx)
			{
				auto const& base_colour = *ctx.m_material.Component<materials::BaseColour>();
				auto const& metallic = *ctx.m_material.Component<materials::Metallic>();
				auto const& roughness = *ctx.m_material.Component<materials::Roughness>();
				auto const& emissive = *ctx.m_material.Component<materials::Emissive>();
				auto const& normal_map = *ctx.m_material.Component<materials::NormalMap>();
				auto texcoords = GatherTexCoordBindings(ctx);

				BindTexture(ctx, shaders::fwd::ERootParam::DiffTexture, base_colour.m_tex, true);
				BindSampler(ctx, shaders::fwd::ERootParam::DiffTextureSampler, base_colour.m_tex, true);
				BindTexture(ctx, shaders::fwd::ERootParam::PbrMetallicTexture, metallic.m_tex.m_slot, false);
				BindSampler(ctx, shaders::fwd::ERootParam::PbrMetallicSampler, metallic.m_tex.m_slot, false);
				BindTexture(ctx, shaders::fwd::ERootParam::PbrRoughnessTexture, roughness.m_tex.m_slot, false);
				BindSampler(ctx, shaders::fwd::ERootParam::PbrRoughnessSampler, roughness.m_tex.m_slot, false);
				BindTexture(ctx, shaders::fwd::ERootParam::PbrEmissiveTexture, emissive.m_tex, false);
				BindSampler(ctx, shaders::fwd::ERootParam::PbrEmissiveSampler, emissive.m_tex, false);
				BindTexture(ctx, shaders::fwd::ERootParam::PbrNormalTexture, normal_map.m_tex, false);
				BindSampler(ctx, shaders::fwd::ERootParam::PbrNormalSampler, normal_map.m_tex, false);
				BindTexCoordStreams(ctx, texcoords);

				if (auto* shader = dynamic_cast<shaders::Forward*>(ctx.m_shader); shader != nullptr)
				{
					shader->SetupElement(ctx.m_cmd_list.get(), ctx.m_upload, ctx.m_scene, &ctx.m_dle, ctx.m_material);
					BindPbrConstants(ctx, texcoords);
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
				
				// If the pass uses extra texture coordinate streams, select a shader variant that samples them
				auto texcoords = GatherTexCoordBindings(ctx);
				if (texcoords.m_count != 0)
				{
					ctx.m_pipe_state.Apply(PSO<EPipeState::VS>(shader_code::forward_texn_pbr_vs));
					ctx.m_pipe_state.Apply(PSO<EPipeState::PS>(
						desc->NumRenderTargets == 0U ? shader_code::forward_alpha_collect_texn_pbr_ps :
						desc->NumRenderTargets > 1U ? shader_code::forward_reflection_attrs_texn_pbr_ps :
						shader_code::forward_texn_pbr_ps));
				}
				else
				{
					ctx.m_pipe_state.Apply(PSO<EPipeState::VS>(shader_code::forward_vs));
					ctx.m_pipe_state.Apply(PSO<EPipeState::PS>(
						desc->NumRenderTargets == 0U ? shader_code::forward_alpha_collect_pbr_ps :
						desc->NumRenderTargets > 1U ? shader_code::forward_reflection_attrs_pbr_ps :
						shader_code::forward_pbr_ps));
				}
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
		, m_procedural_surface()
		, m_shaders()
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
		, m_procedural_surface(rhs.m_procedural_surface)
		, m_shaders(rhs.m_shaders)
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
			{
				if (m_procedural_surface.m_enabled)
					throw std::runtime_error("Procedural surface materials support forward, shadow-map, and ray-cast paths only");

				return nullptr;
			}
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

	// Set the texture slot used for tangent-space normals.
	MaterialPBR& MaterialPBR::normal_texture(materials::TextureSlot slot, float scale)
	{
		m_normal_map.m_tex = slot;
		m_normal_map.m_scale = scale;
		return *this;
	}

	// Set the GPU-evaluated procedural surface.
	MaterialPBR& MaterialPBR::procedural_surface(materials::ProceduralSurface surface)
	{
		// Reject invalid caller state before it becomes part of an immutable draw material.
		surface.Validate();
		surface.m_enabled = true;
		m_procedural_surface = surface;
		return *this;
	}

	// Clear the GPU-evaluated procedural surface.
	MaterialPBR& MaterialPBR::procedural_surface_clear()
	{
		// Preserve every ordinary PBR channel while removing procedural evaluation.
		m_procedural_surface = {};
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

		if (component_id == materials::ProceduralSurface::Id)
			return m_procedural_surface.m_enabled ? &m_procedural_surface : nullptr;

		if (component_id == materials::ShaderOverlays::Id)
			return &m_shaders;

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

	// Validate the caller-owned coordinate and channel ranges.
	void materials::ProceduralSurface::Validate() const
	{
		// Reject values that cannot produce a stable finite shader coordinate or documented PBR channel.
		switch (m_coordinate_space)
		{
			case EProceduralCoordinateSpace::Object:
			case EProceduralCoordinateSpace::World:
			{
				break;
			}
			default:
			{
				throw std::invalid_argument("Unknown procedural surface coordinate space");
			}
		}
		if (!std::isfinite(m_feature_scale) || m_feature_scale <= 0.0f)
			throw std::invalid_argument("Procedural surface feature scale must be finite and positive");

		if (!IsFinite(m_coordinate_origin) || !IsFinite(m_axis_scale) || m_axis_scale.x <= 0.0f || m_axis_scale.y <= 0.0f || m_axis_scale.z <= 0.0f)
			throw std::invalid_argument("Procedural surface coordinate origin and axis scale must be finite, with positive XYZ scale");

		for (auto const& colour : m_palette)
		{
			// Non-finite palette channels would contaminate every downstream lighting result.
			if (!IsFinite(colour.rgba))
				throw std::invalid_argument("Procedural surface palette colours must be finite");
		}

		if (!std::isfinite(m_normal_strength) || m_normal_strength < 0.0f)
			throw std::invalid_argument("Procedural surface normal strength must be finite and nonnegative");

		if (!std::isfinite(m_roughness_min) || !std::isfinite(m_roughness_max) || m_roughness_min < 0.04f || m_roughness_max > 1.0f || m_roughness_min > m_roughness_max)
			throw std::invalid_argument("Procedural surface roughness range must satisfy 0.04 <= min <= max <= 1");

		if (!std::isfinite(m_detail) || m_detail < 0.0f || m_detail > 1.0f || !std::isfinite(m_warp) || m_warp < 0.0f)
			throw std::invalid_argument("Procedural surface detail must be in [0,1] and warp must be finite and nonnegative");
	}
}
