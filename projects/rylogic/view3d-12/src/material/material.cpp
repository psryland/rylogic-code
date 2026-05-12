//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2026
//*********************************************
#include "pr/view3d-12/material/material.h"
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
		// Return the effective diffuse texture for the legacy/simple material.
		Texture2D* DiffuseTexture(MaterialPassContext const& ctx)
		{
			auto& inst = *ctx.m_dle.m_instance;
			auto& nugget = *ctx.m_dle.m_nugget;
			auto tex = coalesce(FindDiffTexture(inst), nugget.m_tex_diffuse);
			return tex != nullptr
				? tex.get()
				: ctx.m_default_tex;
		}

		// Return the effective diffuse sampler for the legacy/simple material.
		Sampler* DiffuseSampler(MaterialPassContext const& ctx)
		{
			auto& inst = *ctx.m_dle.m_instance;
			auto& nugget = *ctx.m_dle.m_nugget;
			auto sam = coalesce(FindDiffTextureSampler(inst), nugget.m_sam_diffuse);
			return sam != nullptr
				? sam.get()
				: ctx.m_default_sam;
		}

		// Bind a diffuse texture descriptor if one is available.
		template <typename RootParam>
		void BindDiffuseTexture(MaterialPassContext& ctx, RootParam root_param)
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
		void BindDiffuseSampler(MaterialPassContext& ctx, RootParam root_param)
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
		void BindForward(MaterialPassContext& ctx)
		{
			BindDiffuseTexture(ctx, shaders::fwd::ERootParam::DiffTexture);
			BindDiffuseSampler(ctx, shaders::fwd::ERootParam::DiffTextureSampler);
			if (ctx.m_shader == nullptr)
				throw std::runtime_error("Forward material pass requires a shader");

			ctx.m_shader->SetupElement(ctx.m_cmd_list.get(), ctx.m_upload, ctx.m_scene, &ctx.m_dle);
		}

		// Bind fixed-function style resources for the simple shadow-map pass.
		void BindShadowMap(MaterialPassContext& ctx)
		{
			BindDiffuseTexture(ctx, shaders::smap::ERootParam::DiffTexture);
			BindDiffuseSampler(ctx, shaders::smap::ERootParam::DiffTextureSampler);
			if (auto* shader = dynamic_cast<shaders::ShadowMap*>(ctx.m_shader); shader != nullptr)
			{
				shader->SetupElement(ctx.m_cmd_list.get(), ctx.m_upload, &ctx.m_dle, ctx.m_scene.m_cam);
				return;
			}
			throw std::runtime_error("Shadow-map material pass requires a shadow-map shader");
		}

		// Bind fixed-function style resources for the simple ray-cast pass.
		void BindRayCast(MaterialPassContext& ctx)
		{
			if (auto* shader = dynamic_cast<shaders::RayCast*>(ctx.m_shader); shader != nullptr)
			{
				shader->SetupElement(ctx.m_cmd_list.get(), ctx.m_upload, &ctx.m_dle);
				return;
			}
			throw std::runtime_error("Ray-cast material pass requires a ray-cast shader");
		}

		// Apply legacy shader overlays for the simple forward pass.
		void ApplyForwardPipeline(MaterialPassContext& ctx)
		{
			auto& nugget = *ctx.m_dle.m_nugget;
			for (auto& shdr_overlay : nugget.m_shdr_overlays)
			{
				if (shdr_overlay.m_rdr_step != ctx.m_step_id)
					continue;

				auto& overlay = *shdr_overlay.m_overlay.get();
				if (overlay.m_signature) ctx.m_pipe_state.Apply(PSO<EPipeState::RootSignature>(overlay.m_signature.get()));
				if (overlay.m_code.VS) ctx.m_pipe_state.Apply(PSO<EPipeState::VS>(overlay.m_code.VS));
				if (overlay.m_code.PS) ctx.m_pipe_state.Apply(PSO<EPipeState::PS>(overlay.m_code.PS));
				if (overlay.m_code.DS) ctx.m_pipe_state.Apply(PSO<EPipeState::DS>(overlay.m_code.DS));
				if (overlay.m_code.HS) ctx.m_pipe_state.Apply(PSO<EPipeState::HS>(overlay.m_code.HS));
				if (overlay.m_code.GS) ctx.m_pipe_state.Apply(PSO<EPipeState::GS>(overlay.m_code.GS));

				overlay.SetupFrame(ctx.m_cmd_list.get(), ctx.m_upload, ctx.m_scene);
				overlay.SetupElement(ctx.m_cmd_list.get(), ctx.m_upload, ctx.m_scene, &ctx.m_dle);
			}
		}

		// The material pass that reproduces legacy NuggetDesc material handling.
		struct SimpleMaterialPass : MaterialPass
		{
			// Return true if this simple material needs alpha rendering.
			bool RequiresAlpha(BaseInstance const&, Nugget const& nugget) const override
			{
				return nugget.RequiresAlpha();
			}

			// Contribute legacy texture and shader-overlay state to the sort key.
			SortKey AddSortKey(ERenderStep step, BaseInstance const&, Nugget const& nugget, SortKey key) const override
			{
				switch (step)
				{
					case ERenderStep::RenderForward:
					{
						if (!AnySet(key, SortKey::TextureIdMask) && nugget.m_tex_diffuse != nullptr)
							key = SetBits(key, SortKey::TextureIdMask, nugget.m_tex_diffuse->SortId() << SortKey::TextureIdOfs);

						if (!AnySet(key, SortKey::ShaderIdMask))
						{
							auto shdr_id = 0;
							for (auto& overlay : nugget.m_shdr_overlays)
							{
								if (overlay.m_rdr_step != step)
									continue;

								shdr_id = shdr_id * 13 ^ overlay.m_overlay->SortId();
							}
							key = SetBits(key, SortKey::ShaderIdMask, shdr_id << SortKey::ShaderIdOfs);
						}
						return key;
					}
					case ERenderStep::ShadowMap:
					{
						if (!AnySet(key, SortKey::TextureIdMask) && nugget.m_tex_diffuse != nullptr)
							key = SetBits(key, SortKey::TextureIdMask, nugget.m_tex_diffuse->SortId() << SortKey::TextureIdOfs);

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
				switch (ctx.m_step_id)
				{
					case ERenderStep::RenderForward:
					{
						ApplyForwardPipeline(ctx);
						return;
					}
					case ERenderStep::ShadowMap:
					case ERenderStep::RayCast:
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
		};

		// Return the shared simple material pass.
		SimpleMaterialPass const& SimplePass()
		{
			static SimpleMaterialPass pass;
			return pass;
		}
	}

	// Return true if this material pass needs alpha rendering for 'nugget'.
	bool MaterialPass::RequiresAlpha(BaseInstance const&, Nugget const&) const
	{
		return false;
	}

	// Contribute material state to the draw sort key.
	SortKey MaterialPass::AddSortKey(ERenderStep, BaseInstance const&, Nugget const&, SortKey key) const
	{
		return key;
	}

	// Bind resources and constants needed before the draw call.
	void MaterialPass::Bind(MaterialPassContext&) const
	{}

	// Apply material pipeline state once caller-owned PSO overrides have been applied.
	void MaterialPass::ApplyPipeline(MaterialPassContext&) const
	{}

	// Ref-count clean up function.
	void Material::RefCountZero(RefCounted<Material>* doomed)
	{
		auto* material = static_cast<Material*>(doomed);
		material->Delete();
	}

	// Delete this material instance.
	void Material::Delete()
	{
		rdr12::Delete<Material>(this);
	}

	// Return the shared default material used when a nugget does not specify a custom material.
	Material const& SimpleMaterial::Default()
	{
		static SimpleMaterial material;
		return material;
	}

	// Return the simple material pass for supported render steps.
	MaterialPass const* SimpleMaterial::Pass(ERenderStep step) const
	{
		switch (step)
		{
			case ERenderStep::RenderForward:
			case ERenderStep::ShadowMap:
			case ERenderStep::RayCast:
			case ERenderStep::RayTracing:
			{
				return &SimplePass();
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

	// Return the shared default material used by nuggets with no explicit material.
	Material const& DefaultMaterial()
	{
		return SimpleMaterial::Default();
	}
}
