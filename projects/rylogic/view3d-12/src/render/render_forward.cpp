//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#include "view3d-12/src/render/render_forward.h"
#include "pr/view3d-12/main/renderer.h"
#include "pr/view3d-12/main/window.h"
#include "pr/view3d-12/render/back_buffer.h"
#include "pr/view3d-12/render/drawlist_element.h"
#include "pr/view3d-12/instance/instance.h"
#include "pr/view3d-12/scene/scene.h"
#include "pr/view3d-12/model/nugget.h"
#include "pr/view3d-12/model/model.h"
#include "pr/view3d-12/model/skinned_geometry.h"
#include "pr/view3d-12/model/skin.h"
#include "pr/view3d-12/model/pose.h"
#include "pr/view3d-12/model/vertex_layout.h"
#include "pr/view3d-12/material/material_simple.h"
#include "pr/view3d-12/material/material_pbr.h"
#include "pr/view3d-12/ray_tracing/render_ray_tracing.h"
#include "pr/view3d-12/shaders/shader.h"
#include "pr/view3d-12/shaders/shader_forward.h"
#include "pr/view3d-12/texture/texture_base.h"
#include "pr/view3d-12/texture/texture_2d.h"
#include "pr/view3d-12/texture/texture_cube.h"
#include "pr/view3d-12/sampler/sampler.h"
#include "pr/view3d-12/utility/pipe_state.h"
#include "view3d-12/src/shaders/common.h"

namespace pr::rdr12
{
	using namespace ::pr::compute;

	RenderForward::RenderForward(Scene& scene)
		: RenderStep(Id, scene, scene.wnd().m_gsync)
		, m_shader(scene.rdr())
		, m_cmd_list(scene.d3d(), nullptr, "RenderForward", EColours::Blue)
		, m_alp_list(scene.d3d(), nullptr, "RenderForwardAlpha", EColours::Blue)
		, m_default_tex(rdr().store().StockTexture(EStockTexture::White))
		, m_default_sam(rdr().store().StockSampler(EStockSampler::LinearClamp))
	{
		// Create the default PSO description
		m_default_pipe_state = D3D12_GRAPHICS_PIPELINE_STATE_DESC {
			.pRootSignature = m_shader.m_signature.get(),
			.VS = m_shader.m_code.VS,
			.PS = m_shader.m_code.PS,
			.DS = m_shader.m_code.DS,
			.HS = m_shader.m_code.HS,
			.GS = m_shader.m_code.GS,
			.StreamOutput = StreamOutputDesc{},
			.BlendState = BlendStateDesc{},
			.SampleMask = UINT_MAX,
			.RasterizerState = RasterStateDesc{},
			.DepthStencilState = DepthStateDesc{},
			.InputLayout = Vert::LayoutDesc(),
			.IBStripCutValue = D3D12_INDEX_BUFFER_STRIP_CUT_VALUE_DISABLED,
			.PrimitiveTopologyType = D3D12_PRIMITIVE_TOPOLOGY_TYPE_TRIANGLE,
			.NumRenderTargets = 1U,
			.RTVFormats = {
				// Match the sRGB RTV cast applied in Window::CreateRenderTarget/CreateSwapChain.
				::pr::compute::ToSRGB(scene.wnd().m_rt_props.Format),
				DXGI_FORMAT_UNKNOWN,
				DXGI_FORMAT_UNKNOWN,
				DXGI_FORMAT_UNKNOWN,
				DXGI_FORMAT_UNKNOWN,
				DXGI_FORMAT_UNKNOWN,
				DXGI_FORMAT_UNKNOWN,
				DXGI_FORMAT_UNKNOWN,
			},
			.DSVFormat = scene.wnd().m_ds_props.Format,
			.SampleDesc = scene.wnd().MultiSampling(),
			.NodeMask = 0U,
			.CachedPSO = {
				.pCachedBlob = nullptr,
				.CachedBlobSizeInBytes = 0U,
			},
			.Flags = D3D12_PIPELINE_STATE_FLAG_NONE,
		};

		// Duplicate the PSO for the opaque pass variant that also writes RT reflection attributes.
		auto psdesc = *static_cast<D3D12_GRAPHICS_PIPELINE_STATE_DESC const*>(m_default_pipe_state);
		psdesc.PS = shader_code::forward_reflection_attrs_ps;
		psdesc.NumRenderTargets = 2U;
		psdesc.RTVFormats[1] = RayTracingReflectionAttributeFormat;
		m_reflection_pipe_state = psdesc;

		// Duplicate the PSO for the alpha pass with some modifications
		psdesc = *static_cast<D3D12_GRAPHICS_PIPELINE_STATE_DESC const*>(m_default_pipe_state);
		psdesc.PS = shader_code::forward_alpha_collect_ps;
		psdesc.DepthStencilState = DepthStateDesc{}.Enabled(false);
		psdesc.NumRenderTargets = 0U;
		psdesc.RTVFormats[0] = DXGI_FORMAT_UNKNOWN;
		psdesc.DSVFormat = DXGI_FORMAT_UNKNOWN;
		psdesc.SampleDesc = MultiSamp(1, 0);
		m_alpha_pipe_state = psdesc;

		// Post-alpha overlays render to the resolved target after alpha resolve, so they use a 1x no-depth forward PSO.
		psdesc = *static_cast<D3D12_GRAPHICS_PIPELINE_STATE_DESC const*>(m_default_pipe_state);
		psdesc.DepthStencilState = DepthStateDesc{}.Enabled(false);
		psdesc.DSVFormat = DXGI_FORMAT_UNKNOWN;
		psdesc.SampleDesc = MultiSamp(1, 0);
		m_post_alpha_pipe_state = psdesc;

		// The far clip fade passes read the depth buffer (t0) and take their constants as root values (b0). See 'background_fade.hlsl'.
		m_fade_signature = RootSig(ERootSigFlags::GraphicsOnly)
			.SRV(ESRVReg::t0, 1, D3D12_SHADER_VISIBILITY_PIXEL)
			.U32(ECBufReg::b0, sizeof(float4_t) * 3 / sizeof(uint32_t), D3D12_SHADER_VISIBILITY_PIXEL)
			.Create(scene.d3d(), "BackgroundFadeSig");

		// Both fade passes draw a full-screen triangle into the main MSAA render target without depth.
		auto fade_desc = D3D12_GRAPHICS_PIPELINE_STATE_DESC{
			.pRootSignature = m_fade_signature.get(),
			.VS = shader_code::background_fade_vs,
			.PS = shader_code::background_fade_weight_ps,
			.DS = shader_code::none,
			.HS = shader_code::none,
			.GS = shader_code::none,
			.StreamOutput = StreamOutputDesc{},
			.BlendState = BlendStateDesc{},
			.SampleMask = UINT_MAX,
			.RasterizerState = RasterStateDesc{},
			.DepthStencilState = DepthStateDesc{}.Enabled(false),
			.InputLayout = {},
			.IBStripCutValue = D3D12_INDEX_BUFFER_STRIP_CUT_VALUE_DISABLED,
			.PrimitiveTopologyType = D3D12_PRIMITIVE_TOPOLOGY_TYPE_TRIANGLE,
			.NumRenderTargets = 1U,
			.RTVFormats = { ::pr::compute::ToSRGB(scene.wnd().m_rt_props.Format) },
			.DSVFormat = DXGI_FORMAT_UNKNOWN,
			.SampleDesc = scene.wnd().MultiSampling(),
			.NodeMask = 0U,
			.CachedPSO = {},
			.Flags = D3D12_PIPELINE_STATE_FLAG_NONE,
		};

		// The weight pass replaces only destination alpha, leaving the opaque colour for the background blend.
		fade_desc.BlendState.RenderTarget[0].RenderTargetWriteMask = D3D12_COLOR_WRITE_ENABLE_ALPHA;
		Check(scene.d3d()->CreateGraphicsPipelineState(&fade_desc, __uuidof(ID3D12PipelineState), (void**)m_fade_weight_pso.address_of()));
		DebugName(m_fade_weight_pso, "BackgroundFadeWeightPSO");

		// The clear pass blends the clear colour by source alpha and keeps destination alpha.
		fade_desc.PS = shader_code::background_fade_clear_ps;
		fade_desc.BlendState = BlendStateDesc{}
			.enable(0)
			.blend(0, D3D12_BLEND_OP_ADD, D3D12_BLEND_SRC_ALPHA, D3D12_BLEND_INV_SRC_ALPHA)
			.blend_alpha(0, D3D12_BLEND_OP_ADD, D3D12_BLEND_ZERO, D3D12_BLEND_ONE);
		Check(scene.d3d()->CreateGraphicsPipelineState(&fade_desc, __uuidof(ID3D12PipelineState), (void**)m_fade_clear_pso.address_of()));
		DebugName(m_fade_clear_pso, "BackgroundFadeClearPSO");
	}
	RenderForward::~RenderForward()
	{
		// In-flight frames may still reference the fade pipeline objects.
		rdr().DeferRelease(m_fade_weight_pso);
		rdr().DeferRelease(m_fade_clear_pso);
		rdr().DeferRelease(m_fade_signature);
		m_default_tex = nullptr;
		m_default_sam = nullptr;
	}

	// Add model nuggets to the draw list for this render step.
	void RenderForward::AddNuggets(BaseInstance const& inst, NuggetPtr nuggets, drawlist_t& drawlist)
	{
		auto inst_has_alpha = HasAlpha(inst);
		auto material_override = FindMaterial(inst);

		// Add a draw list element for each nugget in the instance's model
		for (auto& nugget : Enumerate(nuggets))
		{
			auto const& root_material = material_override != nullptr ? *material_override.get() : nugget.mat();
			auto const* root_pass = root_material.Pass(m_step_id);
			if (root_pass == nullptr)
				continue;

			auto has_alpha = inst_has_alpha || root_pass->RequiresAlpha(inst, root_material, nugget);
			for (auto& nug : nugget.Dependents())
			{
				auto const& material = material_override != nullptr ? *material_override.get() : nug.mat();
				auto const* pass = material.Pass(m_step_id);
				if (pass == nullptr)
					continue;

				// Skip the default (opaque) nugget when the instance requires alpha, and skip alpha
				// variants when it doesn't. Other variants (e.g. ShowNormalsNugget) always render.
				if (has_alpha && nug.m_variant == DefaultNugget) continue;
				if (!has_alpha && nug.m_variant == AlphaNugget) continue;

				// Ignore if flagged as not visible
				if (AllSet(nug.m_nflags, ENuggetFlag::Hidden))
					continue;

				// Create the combined sort key for this nugget
				auto sk = nug.m_sort_key;
				if (auto* sko = inst.find<SKOverride>(EInstComp::SortkeyOverride))
					sk = sko->Combine(sk);

				// Don't add alpha back faces when using 'Points' fill mode
				if (nug.FillMode() == EFillMode::Points && sk.Group() == ESortGroup::AlphaBack)
					break;

				// Let the material add pass-specific sort key information.
				sk = pass->AddSortKey(m_step_id, inst, material, nug, sk);

				// Reject unsupported materials before a window starts recording a frame, because a frame abandoned during recording cannot be retired.
				auto element = DrawListElement{ .m_sort_key = sk, .m_nugget = &nug, .m_instance = &inst };
				ValidateMaterial(element);

				// Add an element to the draw list
				drawlist.push_back(element);
				m_sort_needed = true;
			}
		}
	}

	// Evaluate the known material pipeline contract without issuing commands or allocating frame resources.
	void RenderForward::ValidateMaterial(DrawListElement const& dle) const
	{
		// Only procedural pixel families change the pixel shader selection, so other draws need no evaluation.
		auto const& nugget = *dle.m_nugget;
		auto const& instance = *dle.m_instance;
		auto material_override = FindMaterial(instance);
		auto const& material = material_override != nullptr ? *material_override.get() : nugget.mat();
		auto const* procedural_family = materials::ProceduralPixelFamily(material, m_step_id);
		if (procedural_family == nullptr)
			return;

		// Material passes are immutable; apply their documented shader selection after caller-owned overrides.
		auto desc = m_default_pipe_state;
		for (auto const& state : scn().m_pso)
			desc.Apply(state);
		for (auto const& state : nugget.m_pso)
			desc.Apply(state);
		for (auto const& state : GetPipeStates(instance))
			desc.Apply(state);

		// Stock PBR chooses its own pixel variant, which a PBR procedural family may replace; simple materials apply shader overlays in order.
		switch (material.TypeId())
		{
			case MaterialPBR::MaterialTypeId:
			{
				desc.Apply(PSO<EPipeState::PS>(MaterialPBR::UsesExtraTexCoords(material, &nugget) ? shader_code::forward_texn_pbr_ps : shader_code::forward_pbr_ps));
				if (procedural_family != nullptr)
					materials::ApplyForwardPixelFamily(desc, *procedural_family);

				break;
			}
			case MaterialSimple::MaterialTypeId:
			{
				if (auto const* overlays = material.Component<materials::ShaderOverlays>())
				{
					for (auto const& entry : overlays->m_overlays)
					{
						if (entry.m_rdr_step != m_step_id)
							continue;

						auto& overlay = *entry.m_overlay.get();
						if (overlay.m_signature)
							desc.Apply(PSO<EPipeState::RootSignature>(overlay.m_signature.get()));
						if (overlay.m_code.PS)
							desc.Apply(PSO<EPipeState::PS>(overlay.m_code.PS));
					}
				}
				if (procedural_family != nullptr)
					materials::ApplyForwardPixelFamily(desc, *procedural_family);
				if (material.Component<materials::DetailNormals>() != nullptr)
					materials::ApplyForwardPixelFamily(desc, shader_code::forward_detail_family);

				break;
			}
			default:
			{
				// Other material types own their pixel shaders.
				return;
			}
		}
	}

	// Perform the render step
	void RenderForward::Execute(Frame& frame)
	{
		// Reset the command list with a new allocator for this frame
		m_cmd_list.Reset(frame.m_cmd_alloc_pool.Get());
		m_alp_list.Reset(frame.m_cmd_alloc_pool.Get());

		// Add the command lists we're using to the frame.
		frame.m_main.push_back(m_cmd_list);
		frame.m_main.push_back(m_alp_list);

		// Sort the draw list if needed
		dl_boundaries boundaries;
		SortIfNeeded(&boundaries);

		// Upload the constants for every element once, in drawlist order. Each draw selects its entry by drawlist position.
		{
			auto drawlist = m_drawlist.lock();
			m_elements = shaders::Forward::UploadElements(m_upload_buffer, scn(), CameraTransforms(scn().m_cam), std::span{ *drawlist });
		}

		// Bind the descriptor heaps
		auto des_heaps = { wnd().m_heap_view.get(), wnd().m_heap_samp.get() };
		m_cmd_list.SetDescriptorHeaps({ des_heaps.begin(), des_heaps.size() });
		m_alp_list.SetDescriptorHeaps({ des_heaps.begin(), des_heaps.size() });

		// Set the viewport and scissor rect.
		auto const& vp = scn().m_viewport;
		m_cmd_list.RSSetViewports({ &vp, 1 });
		m_alp_list.RSSetViewports({ &vp, 1 });
		m_cmd_list.RSSetScissorRects(vp.m_clip);
		m_alp_list.RSSetScissorRects(vp.m_clip);

		auto& kbuf = wnd().m_alpha_kbuffer;

		// Render the opaque nuggets first
		{
			BindFrameResources(m_cmd_list);

			// The RT reflection sidecar describes the opaque raster-visible surface only. Alpha continues through the K-buffer unchanged;
			// future alpha-aware RT passes should add matching sidecar payloads beside the alpha layer buffers rather than bypassing them.
			auto* ray_tracing = scn().FindRStep<RenderRayTracing>();
			auto* reflection_attrs = ray_tracing != nullptr ? ray_tracing->PrepareReflectionAttributes(frame) : nullptr;
			auto const& pipe_state = reflection_attrs != nullptr ? m_reflection_pipe_state : m_default_pipe_state;

			// Get the back buffer view handle and set the back buffer as the render target.
			auto bind_render_targets = [&]
			{
				// The reflection sidecar is an extra render target beside the main colour target.
				if (reflection_attrs != nullptr)
				{
					D3D12_CPU_DESCRIPTOR_HANDLE rtvs[] = { frame.bb_main().m_rtv, reflection_attrs->RTV() };
					m_cmd_list.OMSetRenderTargets({ rtvs, _countof(rtvs) }, FALSE, &frame.bb_main().m_dsv);
				}
				else
				{
					m_cmd_list.OMSetRenderTargets({ &frame.bb_main().m_rtv, 1 }, FALSE, &frame.bb_main().m_dsv);
				}
			};
			if (reflection_attrs != nullptr)
			{
				BarrierBatch bb(m_cmd_list);
				bb.Transition(reflection_attrs->Attributes(), D3D12_RESOURCE_STATE_RENDER_TARGET);
				bb.Commit();

				reflection_attrs->Clear(m_cmd_list);
			}
			bind_render_targets();

			// Draw the opaques
			auto drawlist = m_drawlist.lock();
			auto opaques = std::span{ *drawlist }.subspan(0, s_cast<size_t>(kbuf ? boundaries[ESortGroup::AlphaBack] : s_cast<int>(drawlist->size())));
			auto pix_opaque = pix::EventScope<ID3D12GraphicsCommandList>(m_cmd_list.get(), 0xFF7EB8E5, "View3D::Opaque");
			if (scn().FarClipFadeProperties().m_enabled)
			{
				// Blend distant scene geometry into the background. The background groups draw after the fade pass so they only fill the faded samples.
				auto background_beg = boundaries[ESortGroup::Skybox];
				auto background_end = boundaries[ESortGroup::PostOpaques];
				DrawNuggets(frame, m_cmd_list, pipe_state, opaques.subspan(0, s_cast<size_t>(background_beg)), 0, ESubPass::Opaque);
				DrawBackgroundFade(frame, opaques.subspan(s_cast<size_t>(background_beg), s_cast<size_t>(background_end - background_beg)), background_beg);

				// The fade pass changed the render targets, viewport, and root signature. Restore them for the remaining opaque groups.
				BindFrameResources(m_cmd_list);
				bind_render_targets();
				m_cmd_list.RSSetViewports({ &vp, 1 });
				m_cmd_list.RSSetScissorRects(vp.m_clip);
				DrawNuggets(frame, m_cmd_list, pipe_state, opaques.subspan(s_cast<size_t>(background_end)), background_end, ESubPass::Opaque);
			}
			else
			{
				DrawNuggets(frame, m_cmd_list, pipe_state, opaques, 0, ESubPass::Opaque);
			}
		}

		// Render the alpha nuggets
		if (kbuf)
		{
			// Separate transparent collection from opaque work recorded on the other command list.
			auto pix_alpha = pix::EventScope<ID3D12GraphicsCommandList>(m_alp_list.get(), 0xFFBA84E5, "View3D::Alpha");
			BindFrameResources(m_alp_list);

			m_alp_list.OMSetRenderTargets({}, FALSE, nullptr);

			BarrierBatch bb(m_alp_list);
			bb.Transition(frame.bb_main().m_depth_stencil.get(), D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE);
			bb.Transition(kbuf.m_alpha_colour->m_res.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			bb.Transition(kbuf.m_alpha_depth->m_res.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			bb.Transition(kbuf.m_alpha_rt_attrs->m_res.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			bb.Commit();

			BindAlphaResources(frame, m_alp_list);

			// Draw the alphas
			auto drawlist = m_drawlist.lock();
			auto alpha_start = boundaries[ESortGroup::AlphaBack];
			auto alpha_end = boundaries[ESortGroup::PostAlpha];

			// Use the alpha collect shader and disable depth writes for the alpha pass
			DrawNuggets(frame, m_alp_list, m_alpha_pipe_state, std::span{ *drawlist }.subspan(s_cast<size_t>(alpha_start), s_cast<size_t>(alpha_end - alpha_start)), alpha_start, ESubPass::Alpha);

			BarrierBatch bb_end(m_alp_list);
			bb_end.Transition(frame.bb_main().m_depth_stencil.get(), D3D12_RESOURCE_STATE_DEPTH_WRITE);
			bb_end.Commit();
		}

		// Render post-alpha overlays after the K-buffer resolve. These are typically NoZTest labels/HUD-style
		// objects, so they must not be included in the alpha collection pass or they disappear behind the resolve.
		if (kbuf)
		{
			auto drawlist = m_drawlist.lock();
			auto post_alpha_start = boundaries[ESortGroup::PostAlpha];
			if (post_alpha_start != s_cast<int>(drawlist->size()) && frame.bb_post().m_render_target != nullptr)
			{
				// Distinguish screen-space overlays that run after the transparent resolve.
				auto pix_overlay = pix::EventScope<ID3D12GraphicsCommandList>(frame.m_composite.get(), 0xFFFFB86C, "View3D::PostAlphaOverlay");
				auto post_alpha_heaps = { wnd().m_heap_view.get(), wnd().m_heap_samp.get() };
				frame.m_composite.SetDescriptorHeaps({ post_alpha_heaps.begin(), post_alpha_heaps.size() });

				BarrierBatch bb(frame.m_composite);
				bb.Transition(frame.bb_post().m_render_target.get(), D3D12_RESOURCE_STATE_RENDER_TARGET);
				bb.Commit();

				BindFrameResources(frame.m_composite);
				frame.m_composite.RSSetViewports({ &vp, 1 });
				frame.m_composite.RSSetScissorRects(vp.m_clip);
				frame.m_composite.OMSetRenderTargets({ &frame.bb_post().m_rtv, 1 }, FALSE, nullptr);
				DrawNuggets(frame, frame.m_composite, m_post_alpha_pipe_state, std::span{ *drawlist }.subspan(s_cast<size_t>(post_alpha_start)), post_alpha_start, ESubPass::Opaque);
			}
		}

		// Close the command list now that we've finished rendering this scene
		m_cmd_list.Close();
		m_alp_list.Close();
	}

	// Set up shader resources that are common to all nuggets in this render step.
	void RenderForward::BindFrameResources(GfxCmdList& cmd_list)
	{
		// Set the signature for the shader used for this nugget
		cmd_list.SetGraphicsRootSignature(m_shader.m_signature.get());

		// Set shader constants for the frame
		m_shader.SetupFrame(cmd_list.get(), m_upload_buffer, scn());
		shaders::Forward::SetupElements(cmd_list.get(), m_elements);

		// Add the shadow atlas. It is only read for lights with shadow views, so it is not needed when there are none.
		if (auto* smap_step = scn().FindRStep<RenderSmap>(); smap_step != nullptr && !smap_step->Views().empty())
		{
			auto gpu = wnd().m_heap_view.Add(smap_step->Atlas()->m_srv);
			cmd_list.SetGraphicsRootDescriptorTable(shaders::fwd::ERootParam::ShadowAtlas, gpu);
		}

		// Add the global environment map. Without a previous map, the current map fills the previous slot so the table is always valid.
		if (auto* envmap = scn().m_global_envmap.get())
		{
			auto gpu = wnd().m_heap_view.Add(envmap->m_srv);
			cmd_list.SetGraphicsRootDescriptorTable(shaders::fwd::ERootParam::EnvMap, gpu);

			auto* envmap_prev = scn().m_global_envmap_prev.get();
			auto gpu_prev = envmap_prev != nullptr ? wnd().m_heap_view.Add(envmap_prev->m_srv) : gpu;
			cmd_list.SetGraphicsRootDescriptorTable(shaders::fwd::ERootParam::EnvMapPrev, gpu_prev);
		}
	}

	// Set up shader resources used by the alpha collection pass.
	void RenderForward::BindAlphaResources(Frame& frame, GfxCmdList& cmd_list)
	{
		auto& kbuf = wnd().m_alpha_kbuffer;
		assert(kbuf);

		auto alpha_colour = wnd().m_heap_view.Add(kbuf.m_alpha_colour->m_uav);
		auto alpha_depth = wnd().m_heap_view.Add(kbuf.m_alpha_depth->m_uav);
		auto alpha_rt_attrs = wnd().m_heap_view.Add(kbuf.m_alpha_rt_attrs->m_uav);
		auto opaque_depth = wnd().m_heap_view.Add(frame.bb_main().m_depth_srv);
		cmd_list.SetGraphicsRootDescriptorTable(shaders::fwd::ERootParam::OpaqueDepth, opaque_depth);
		cmd_list.SetGraphicsRootDescriptorTable(shaders::fwd::ERootParam::AlphaColour, alpha_colour);
		cmd_list.SetGraphicsRootDescriptorTable(shaders::fwd::ERootParam::AlphaDepth, alpha_depth);
		cmd_list.SetGraphicsRootDescriptorTable(shaders::fwd::ERootParam::AlphaRtAttrs, alpha_rt_attrs);
	}

	// Add the nuggets in the draw list to 'cmd_list' for rendering.
	void RenderForward::DrawNuggets(Frame& frame, GfxCmdList& cmd_list, PipeStateDesc const& default_pipe_state, std::span<DrawListElement const> drawlist, int first_index, ESubPass sub_pass)
	{
		// Keep camera inversion and default projection composition outside the per-nugget loop.
		auto const camera = CameraTransforms(scn().m_cam);
		Descriptor last_tex = {}, last_sam = {};
		auto pipe_state_bound = false;
		auto pipe_state_hash = 0;
		auto frame_resources_bound = true;

		// Material passes can temporarily switch root signatures. Restore the render-step frame bindings before drawing the next nugget.
		auto bind_frame_resources = [&]
		{
			BindFrameResources(cmd_list);
			if (sub_pass == ESubPass::Alpha)
				BindAlphaResources(frame, cmd_list);

			last_tex = {};
			last_sam = {};
			pipe_state_bound = false;
			frame_resources_bound = true;
		};

		// Draw each element in the draw list
		for (auto& dle : drawlist)
		{
			// The alpha collection must never collect background or overlay fragments.
			if (sub_pass == ESubPass::Alpha && dle.m_sort_key.Group() < ESortGroup::AlphaBack)
				continue;

			// Something not rendering?
			//  - Check the tint for the nugget isn't 0x00000000.
			// Tips:
			//  - To uniquely identify an instance in a shader for debugging, set the Instance Id (cb1.m_flags.w)
			//    Then in the shader, use: if (m_flags.w == 1234) ...
			auto const& nugget = *dle.m_nugget;
			auto const& instance = *dle.m_instance;
			auto desc = default_pipe_state;
			
			// If the instance is skinned, get the post-skinned vertex buffer for this instance's pose
			auto const* vb_view = &nugget.m_model->m_vb_view;
			if (PosePtr pose = FindPose(instance); pose && nugget.m_model->m_skin)
			{
				vb_view = &rdr().SkinnedGeometry().VBufView(cmd_list, m_upload_buffer, *nugget.m_model, pose);

				// Compute skinning can bind a compute PSO while updating the vertex buffer. The cached graphics PSO state is no longer reliable.
				pipe_state_bound = false;
			}

			// Set pipeline state
			desc.Apply(PSO<EPipeState::TopologyType>(To<D3D12_PRIMITIVE_TOPOLOGY_TYPE>(nugget.m_topo)));
			if (IsStripTopo(nugget.m_topo))
				desc.Apply(PSO<EPipeState::IBStripCutValue>(StripCutValue(nugget.m_model->m_ib_view.Format)));
			cmd_list.IASetPrimitiveTopology(nugget.m_topo);
			cmd_list.IASetVertexBuffers(0U, { vb_view, 1 });
			cmd_list.IASetIndexBuffer(&nugget.m_model->m_ib_view);

			// Let the material bind per-draw resources and constants.
			auto material_override = FindMaterial(instance);
			auto const& material = material_override != nullptr ? *material_override.get() : nugget.mat();
			auto const* pass = material.Pass(m_step_id);
			if (pass == nullptr)
				continue;

			if (!frame_resources_bound)
				bind_frame_resources();

			// Select this element's entry in the uploaded element constants table.
			shaders::Forward::SetupElement(cmd_list.get(), first_index + s_cast<int>(&dle - drawlist.data()));

			auto ctx = MaterialPassContext{
				.m_step_id = m_step_id,
				.m_wnd = wnd(),
				.m_scene = scn(),
				.m_camera = camera,
				.m_dle = dle,
				.m_material = material,
				.m_cmd_list = cmd_list,
				.m_upload = m_upload_buffer,
				.m_pipe_state = desc,
				.m_shader = &m_shader,
				.m_default_tex = m_default_tex.get(),
				.m_default_sam = m_default_sam.get(),
				.m_last_tex = &last_tex,
				.m_last_sam = &last_sam,
			};
			pass->Bind(ctx);

			// Apply scene pipe state overrides
			{
				for (auto& ps : scn().m_pso)
					desc.Apply(ps);
				for (auto& ps : nugget.m_pso)
					desc.Apply(ps);
				for (auto& ps : GetPipeStates(instance))
					desc.Apply(ps);
			}

			// Let the material apply shader or PSO changes after caller-owned overrides.
			pass->ApplyPipeline(ctx);

			// Background objects fill only the samples the fade pass marked, by the weight stored in destination alpha.
			// Depth testing against the fade start depth (held in the viewport depth range) keeps them off unfaded samples.
			switch (sub_pass)
			{
				case ESubPass::Opaque:
				case ESubPass::Alpha:
				{
					break;
				}
				case ESubPass::Background:
				{
					auto blend = RenderTargetBlendDesc{};
					blend.BlendEnable = TRUE;
					blend.SrcBlend = D3D12_BLEND_INV_DEST_ALPHA;
					blend.DestBlend = D3D12_BLEND_DEST_ALPHA;
					blend.BlendOp = D3D12_BLEND_OP_ADD;
					blend.SrcBlendAlpha = D3D12_BLEND_ONE;
					blend.DestBlendAlpha = D3D12_BLEND_ONE;
					blend.BlendOpAlpha = D3D12_BLEND_OP_ADD;
					desc.Apply(PSO<EPipeState::DepthEnable>(TRUE));
					desc.Apply(PSO<EPipeState::DepthWriteMask>(D3D12_DEPTH_WRITE_MASK_ZERO));
					desc.Apply(PSO<EPipeState::DepthFunc>(D3D12_COMPARISON_FUNC_LESS_EQUAL));
					desc.Apply(PSO<EPipeState::BlendState0>(blend));
					break;
				}
				default:
				{
					throw std::runtime_error("Unknown forward sub-pass");
				}
			}

			// Even a bounds-rejected draw can have changed the material's root bindings.
			if (ctx.m_root_signature_changed)
				frame_resources_bound = false;

			// Draw the nugget.
			DrawNugget(cmd_list, nugget, desc, pipe_state_bound, pipe_state_hash);
		}
	}

	// Blend samples beyond the far clip fade start towards the background. See 'far_clip_fade.md'.
	void RenderForward::DrawBackgroundFade(Frame& frame, std::span<DrawListElement const> background, int first_index)
	{
		// Distinguish the fade from the surrounding opaque work in captures.
		auto pix_fade = pix::EventScope<ID3D12GraphicsCommandList>(m_cmd_list.get(), 0xFF5FA8D3, "View3D::FarClipFade");
		auto const& bb_main = frame.bb_main();
		auto const& vp = scn().m_viewport;
		auto const c2s = scn().m_cam.CameraToScreen();
		auto const fade_range = scn().FarClipFadeProperties().DepthRange(scn().m_cam.ClipPlanes(false).y);

		// The depth conversion uses only the z and w rows of the screen-to-camera transform, because view depth is -z/w after the inverse projection.
		auto const s2c = Invert(c2s);
		auto const& clear = bb_main.rt_clear();
		float const constants[] = {
			s2c.z.z, s2c.w.z, s2c.z.w, s2c.w.w,
			fade_range.x, fade_range.y, 0.0f, 0.0f,
			clear[0], clear[1], clear[2], clear[3],
		};

		// Read the opaque depth in the pixel shader and write only the main colour target.
		{
			BarrierBatch bb(m_cmd_list);
			bb.Transition(bb_main.m_depth_stencil.get(), D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE);
			bb.Commit();
		}
		m_cmd_list.OMSetRenderTargets({ &bb_main.m_rtv, 1 }, FALSE, nullptr);
		m_cmd_list.SetPipelineState(background.empty() ? m_fade_clear_pso.get() : m_fade_weight_pso.get());
		m_cmd_list.SetGraphicsRootSignature(m_fade_signature.get());
		m_cmd_list.SetGraphicsRootDescriptorTable(0, wnd().m_heap_view.Add(bb_main.m_depth_srv));
		m_cmd_list.SetGraphicsRoot32BitConstants(1, _countof(constants), &constants[0], 0);
		m_cmd_list.RSSetViewports({ &vp, 1 });
		m_cmd_list.RSSetScissorRects(vp.m_clip);
		m_cmd_list.IASetPrimitiveTopology(ETopo::TriList);
		m_cmd_list.DrawInstanced(3U, 1U, 0U, 0U);

		// Restore depth for testing; the caller restores the remaining bindings.
		{
			BarrierBatch bb(m_cmd_list);
			bb.Transition(bb_main.m_depth_stencil.get(), D3D12_RESOURCE_STATE_DEPTH_WRITE);
			bb.Commit();
		}
		if (background.empty())
			return;

		// Draw the background objects at the fade start depth. A depth range of one value gives every background fragment that depth,
		// so the LESS_EQUAL test passes only on samples at or beyond the fade start. All other samples keep their opaque colour.
		auto start_ss = c2s * v4(0, 0, -fade_range.x, 1);
		auto start_z = std::clamp(start_ss.z / start_ss.w, 0.0f, 1.0f);
		auto background_vp = vp;
		background_vp.MinDepth = start_z;
		background_vp.MaxDepth = start_z;
		m_cmd_list.RSSetViewports({ &background_vp, 1 });
		m_cmd_list.OMSetRenderTargets({ &bb_main.m_rtv, 1 }, FALSE, &bb_main.m_dsv);
		BindFrameResources(m_cmd_list);
		DrawNuggets(frame, m_cmd_list, m_default_pipe_state, background, first_index, ESubPass::Background);
	}

	// Draw a single nugget
	void RenderForward::DrawNugget(GfxCmdList& cmd_list, Nugget const& nugget, PipeStateDesc& desc, bool& pipe_state_bound, int& pipe_state_hash)
	{
		auto set_pipe_state = [&]
		{
			auto hash = desc.hash();
			if (pipe_state_bound && pipe_state_hash == hash)
				return;

			cmd_list.SetPipelineState(m_pipe_state_pool.Get(desc));
			pipe_state_bound = true;
			pipe_state_hash = hash;
		};

		// Resolve the effective fill mode: per-nugget override wins, else scene-level
		auto fill_mode = nugget.FillMode() != EFillMode::Default
			? nugget.FillMode()
			: scn().FillMode();

		// Render with solid or wire fill mode
		if (fill_mode == EFillMode::Default ||
			fill_mode == EFillMode::Solid ||
			fill_mode == EFillMode::Wireframe ||
			fill_mode == EFillMode::SolidWire)
		{
			// Apply the D3D12 fill mode to the PSO when wireframe is requested
			if (fill_mode == EFillMode::Wireframe)
				desc.Apply(PSO<EPipeState::FillMode>(D3D12_FILL_MODE_WIREFRAME));

			set_pipe_state();
			if (nugget.m_irange.empty())
			{
				cmd_list.DrawInstanced(
					s_cast<size_t>(nugget.m_vrange.size()), 1U,
					s_cast<size_t>(nugget.m_vrange.m_beg), 0U);
			}
			else
			{
				// Keep indexed draws at base-vertex zero because optional model vertex streams use SV_VertexID to read buffers parallel to the primary vertex buffer.
				cmd_list.DrawIndexedInstanced(
					s_cast<size_t>(nugget.m_irange.size()), 1U,
					s_cast<size_t>(nugget.m_irange.m_beg), 0, 0U);
			}
		}

		// Render wire frame over solid for 'SolidWire' mode
		if (fill_mode == EFillMode::SolidWire && (
			nugget.m_topo == ETopo::TriList ||
			nugget.m_topo == ETopo::TriListAdj ||
			nugget.m_topo == ETopo::TriStrip ||
			nugget.m_topo == ETopo::TriStripAdj) &&
			!nugget.m_irange.empty())
		{
			// Change the pipe state to wireframe
			auto prev_fill_mode = desc.Get<EPipeState::FillMode>();
			desc.Apply(PSO<EPipeState::FillMode>(D3D12_FILL_MODE_WIREFRAME));
			desc.Apply(PSO<EPipeState::BlendState0>({FALSE}));
			set_pipe_state();

			cmd_list.DrawIndexedInstanced(
				s_cast<size_t>(nugget.m_irange.size()), 1U,
				s_cast<size_t>(nugget.m_irange.m_beg), 0, 0U);

			// Restore it
			desc.Apply(PSO<EPipeState::FillMode>(prev_fill_mode));
		}

		// Render points for 'Points' mode
		if (fill_mode == EFillMode::Points)
		{
			// Change the pipe state and IA topology to point list.
			// Both must agree: PSO topology type and IA primitive topology.
			desc.Apply(PSO<EPipeState::TopologyType>(To<D3D12_PRIMITIVE_TOPOLOGY_TYPE>(ETopo::PointList)));
			desc.Apply(PSO<EPipeState::GS>(wnd().m_diag.m_gs_fillmode_points->m_code.GS));
			cmd_list.IASetPrimitiveTopology(ETopo::PointList);
			set_pipe_state();

			cmd_list.DrawInstanced(
				s_cast<size_t>(nugget.m_vrange.size()), 1U,
				s_cast<size_t>(nugget.m_vrange.m_beg), 0U);
		}
	}
}
