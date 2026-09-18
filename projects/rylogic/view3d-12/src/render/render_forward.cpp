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
		: RenderStep(Id, scene)
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
	}
	RenderForward::~RenderForward()
	{
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

				// Reject unsupported fade materials before a window starts recording a frame.
				auto element = DrawListElement{ .m_sort_key = sk, .m_nugget = &nug, .m_instance = &inst };
				if (scn().FarClipFadeProperties().m_enabled)
					ValidateFarFadeMaterial(element);

				// Add an element to the draw list
				drawlist.push_back(element);
				m_sort_needed = true;
			}
		}
	}

	// Validate the retained draw list before committing an opt-in settings change.
	void RenderForward::ValidateFarClipFade()
	{
		auto drawlist = m_drawlist.lock();
		for (auto const& element : *drawlist)
			ValidateFarFadeMaterial(element);
	}

	// Evaluate the known material pipeline contract without issuing commands or allocating frame resources.
	void RenderForward::ValidateFarFadeMaterial(DrawListElement const& dle) const
	{
		if (!FarClipFadeApplies(dle.m_sort_key.Group()))
			return;

		// Material passes are immutable; apply their documented shader selection after caller-owned overrides.
		auto const& nugget = *dle.m_nugget;
		auto const& instance = *dle.m_instance;
		auto material_override = FindMaterial(instance);
		auto const& material = material_override != nullptr ? *material_override.get() : nugget.mat();
		auto desc = m_default_pipe_state;
		for (auto const& state : scn().m_pso)
			desc.Apply(state);
		for (auto const& state : nugget.m_pso)
			desc.Apply(state);
		for (auto const& state : GetPipeStates(instance))
			desc.Apply(state);

		// Stock PBR chooses its own pixel variant; simple materials apply shader overlays in order.
		switch (material.TypeId())
		{
			case MaterialPBR::MaterialTypeId:
			{
				desc.Apply(PSO<EPipeState::PS>(shader_code::forward_pbr_ps));
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
				break;
			}
			default:
			{
				throw std::runtime_error("Far clip fade requires stock forward simple/PBR material passes");
			}
		}
		ApplyFarFadePipeline(desc, false);
	}

	// Perform the render step
	void RenderForward::Execute(Frame& frame)
	{
		// Fading opaque fragments require real alpha storage rather than alpha-only opaque output.
		auto const fade_enabled = scn().FarClipFadeProperties().m_enabled;
		if (fade_enabled && !wnd().m_alpha_kbuffer)
			throw std::runtime_error("Far clip fade requires the forward alpha K-buffer");

		// Reset the command list with a new allocator for this frame
		m_cmd_list.Reset(frame.m_cmd_alloc_pool.Get());
		m_alp_list.Reset(frame.m_cmd_alloc_pool.Get());

		// Add the command lists we're using to the frame.
		frame.m_main.push_back(m_cmd_list);
		frame.m_main.push_back(m_alp_list);

		// Sort the draw list if needed
		dl_boundaries boundaries;
		SortIfNeeded(&boundaries);

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
			if (reflection_attrs != nullptr)
			{
				BarrierBatch bb(m_cmd_list);
				bb.Transition(reflection_attrs->Attributes(), D3D12_RESOURCE_STATE_RENDER_TARGET);
				bb.Commit();

				reflection_attrs->Clear(m_cmd_list);

				D3D12_CPU_DESCRIPTOR_HANDLE rtvs[] = { frame.bb_main().m_rtv, reflection_attrs->RTV() };
				m_cmd_list.OMSetRenderTargets({ rtvs, _countof(rtvs) }, FALSE, &frame.bb_main().m_dsv);
			}
			else
			{
				m_cmd_list.OMSetRenderTargets({ &frame.bb_main().m_rtv, 1 }, FALSE, &frame.bb_main().m_dsv);
			}

			// Draw the opaques
			auto drawlist = m_drawlist.lock();
			auto opaque_end = kbuf ? boundaries[ESortGroup::AlphaBack] : s_cast<int>(drawlist->size());
			DrawNuggets(frame, m_cmd_list, pipe_state, std::span{ *drawlist }.subspan(0, s_cast<size_t>(opaque_end)), false);
		}

		// Render the alpha nuggets
		if (kbuf)
		{
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
			
			// Recollect only the opaque fragments excluded from their depth-writing pass.
			if (fade_enabled)
				DrawNuggets(frame, m_alp_list, m_alpha_pipe_state, std::span{ *drawlist }.subspan(0, s_cast<size_t>(alpha_start)), true);

			// Use the alpha collect shader and disable depth writes for the alpha pass
			DrawNuggets(frame, m_alp_list, m_alpha_pipe_state, std::span{ *drawlist }.subspan(s_cast<size_t>(alpha_start), s_cast<size_t>(alpha_end - alpha_start)), true);

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
				auto post_alpha_heaps = { wnd().m_heap_view.get(), wnd().m_heap_samp.get() };
				frame.m_composite.SetDescriptorHeaps({ post_alpha_heaps.begin(), post_alpha_heaps.size() });

				BarrierBatch bb(frame.m_composite);
				bb.Transition(frame.bb_post().m_render_target.get(), D3D12_RESOURCE_STATE_RENDER_TARGET);
				bb.Commit();

				BindFrameResources(frame.m_composite);
				frame.m_composite.RSSetViewports({ &vp, 1 });
				frame.m_composite.RSSetScissorRects(vp.m_clip);
				frame.m_composite.OMSetRenderTargets({ &frame.bb_post().m_rtv, 1 }, FALSE, nullptr);
				DrawNuggets(frame, frame.m_composite, m_post_alpha_pipe_state, std::span{ *drawlist }.subspan(s_cast<size_t>(post_alpha_start)), false);
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

		// Add the shadow map textures
		if (auto* smap_step = scn().FindRStep<RenderSmap>())
		{
			// Todo: consider array-of-structs layout for casters
			vector<Descriptor, 8> descriptors;
			for (auto& caster : smap_step->Casters())
				descriptors.push_back(caster.m_smap->m_srv);

			auto gpu = wnd().m_heap_view.Add(descriptors);
			cmd_list.SetGraphicsRootDescriptorTable(shaders::fwd::ERootParam::SMap, gpu);
		}

		// Add the global environment map
		if (auto* envmap = scn().m_global_envmap.get())
		{
			auto gpu = wnd().m_heap_view.Add(envmap->m_srv);
			cmd_list.SetGraphicsRootDescriptorTable(shaders::fwd::ERootParam::EnvMap, gpu);
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
	void RenderForward::DrawNuggets(Frame& frame, GfxCmdList& cmd_list, PipeStateDesc const& default_pipe_state, std::span<DrawListElement const> drawlist, bool alpha_pass)
	{
		D3D12_GPU_DESCRIPTOR_HANDLE last_tex = {}, last_sam = {};
		auto pipe_state_bound = false;
		auto pipe_state_hash = 0;
		auto frame_resources_bound = true;
		auto const fade_enabled = scn().FarClipFadeProperties().m_enabled;

		// Material passes can temporarily switch root signatures. Restore the render-step frame bindings before drawing the next nugget.
		auto bind_frame_resources = [&]
		{
			BindFrameResources(cmd_list);
			if (alpha_pass)
				BindAlphaResources(frame, cmd_list);

			last_tex = {};
			last_sam = {};
			pipe_state_bound = false;
			frame_resources_bound = true;
		};

		// Draw each element in the draw list
		for (auto& dle : drawlist)
		{
			// The extra opaque collection must never collect background or overlay fragments.
			auto const fade_world = fade_enabled && FarClipFadeApplies(dle.m_sort_key.Group());
			if (alpha_pass && dle.m_sort_key.Group() < ESortGroup::AlphaBack && !fade_world)
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

			auto ctx = MaterialPassContext{
				.m_step_id = m_step_id,
				.m_wnd = wnd(),
				.m_scene = scn(),
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

			// Inspect the final output contract rather than excluding compatible custom vertex/geometry stages.
			if (fade_world)
				ApplyFarFadePipeline(desc, alpha_pass);

			// Even a bounds-rejected draw can have changed the material's root bindings.
			if (ctx.m_root_signature_changed)
				frame_resources_bound = false;

			// Avoid rasterising near-only rigid geometry again; deformed/custom stages cannot use rest-pose bounds.
			if (fade_world && alpha_pass && dle.m_sort_key.Group() < ESortGroup::AlphaBack && IsBeforeFarFade(dle, desc))
				continue;

			// Draw the nugget.
			DrawNugget(cmd_list, nugget, dle.m_sort_key.Group(), alpha_pass, fade_world, desc, pipe_state_bound, pipe_state_hash);
		}
	}

	// Use existing model bounds only when all position-producing stages preserve their affine contract.
	bool RenderForward::IsBeforeFarFade(DrawListElement const& dle, PipeStateDesc const& desc) const
	{
		auto const& model = *dle.m_nugget->m_model;
		auto const& instance = *dle.m_instance;
		auto const* pipeline = static_cast<D3D12_GRAPHICS_PIPELINE_STATE_DESC const*>(desc);
		auto const stock_vertex =
			(pipeline->VS.pShaderBytecode == shader_code::forward_vs.pShaderBytecode && pipeline->VS.BytecodeLength == shader_code::forward_vs.BytecodeLength) ||
			(pipeline->VS.pShaderBytecode == shader_code::forward_texn_pbr_vs.pShaderBytecode && pipeline->VS.BytecodeLength == shader_code::forward_texn_pbr_vs.BytecodeLength);
		if (!stock_vertex || pipeline->GS.BytecodeLength != 0 || pipeline->HS.BytecodeLength != 0 || pipeline->DS.BytecodeLength != 0 ||
			model.m_skin || !model.m_bbox.valid())
			return false;

		// Transform the existing conservative bounds into camera space without copying or scanning vertices.
		auto m2c = InvertOrthonormal(scn().m_cam.CameraToWorld()) * GetO2W(instance) * model.m_m2root;
		if (!IsAffine(m2c))
			return false;

		// Leave a small conservative margin for differences between CPU and shader transforms.
		auto bounds = m2c * model.m_bbox;
		auto start_depth = scn().FarClipFadeProperties().DepthRange(scn().m_cam.ClipPlanes(false).y).x;
		return -bounds.Lower().z < start_depth - 0.0001f * std::max(1.0f, start_depth);
	}

	// Keep stock lighting with its correct opaque/reflection/alpha output after caller-owned shader overrides.
	void RenderForward::ApplyFarFadePipeline(PipeStateDesc& desc, bool alpha_pass) const
	{
		auto const* pipeline = static_cast<D3D12_GRAPHICS_PIPELINE_STATE_DESC const*>(desc);
		if (pipeline->pRootSignature != m_shader.m_signature.get())
			throw std::runtime_error("Far clip fade requires the forward root signature");

		// Bytecode identity identifies supported stock pixel contracts without restricting vertex deformation.
		auto same_shader = [&](shader_code::ByteCode const& code)
		{
			return pipeline->PS.pShaderBytecode == code.pShaderBytecode && pipeline->PS.BytecodeLength == code.BytecodeLength;
		};
		auto select_family = [&](shader_code::ByteCode const& opaque, shader_code::ByteCode const& reflection, shader_code::ByteCode const& collect, shader_code::ByteCode const& fade_opaque, shader_code::ByteCode const& fade_reflection, shader_code::ByteCode const& fade_collect)
		{
			if (!same_shader(opaque) && !same_shader(reflection) && !same_shader(collect))
				return false;

			desc.Apply(PSO<EPipeState::PS>(alpha_pass ? fade_collect : pipeline->NumRenderTargets > 1U ? fade_reflection : fade_opaque));
			return true;
		};
		if (!select_family(shader_code::forward_ps, shader_code::forward_reflection_attrs_ps, shader_code::forward_alpha_collect_ps, shader_code::forward_far_fade_ps, shader_code::forward_far_fade_reflection_attrs_ps, shader_code::forward_far_fade_alpha_collect_ps) &&
			!select_family(shader_code::forward_pbr_ps, shader_code::forward_reflection_attrs_pbr_ps, shader_code::forward_alpha_collect_pbr_ps, shader_code::forward_far_fade_pbr_ps, shader_code::forward_far_fade_reflection_attrs_pbr_ps, shader_code::forward_far_fade_alpha_collect_pbr_ps) &&
			!select_family(shader_code::forward_texn_pbr_ps, shader_code::forward_reflection_attrs_texn_pbr_ps, shader_code::forward_alpha_collect_texn_pbr_ps, shader_code::forward_far_fade_texn_pbr_ps, shader_code::forward_far_fade_reflection_attrs_texn_pbr_ps, shader_code::forward_far_fade_alpha_collect_texn_pbr_ps))
			throw std::runtime_error("Far clip fade requires a stock forward simple/PBR pixel shader");

		// Recollected opaque material state cannot re-enable depth testing or writes on the UAV-only pass.
		if (alpha_pass)
		{
			desc.Apply(PSO<EPipeState::DepthEnable>(FALSE));
			desc.Apply(PSO<EPipeState::DepthWriteMask>(D3D12_DEPTH_WRITE_MASK_ZERO));
		}
	}

	// Draw a single nugget using its resolved scene ordering contract.
	void RenderForward::DrawNugget(GfxCmdList& cmd_list, Nugget const& nugget, ESortGroup sort_group, bool alpha_pass, bool fade_world, PipeStateDesc& desc, bool& pipe_state_bound, int& pipe_state_hash)
	{
		// Rebind only when the effective pipeline changes between the solid and diagnostic passes.
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

		// Overlay opaque world triangles only; sky remains a readable backdrop and alpha geometry is not shaded or collected twice.
		if (fill_mode == EFillMode::SolidWire && (
			nugget.m_topo == ETopo::TriList ||
			nugget.m_topo == ETopo::TriListAdj ||
			nugget.m_topo == ETopo::TriStrip ||
			nugget.m_topo == ETopo::TriStripAdj) &&
			sort_group != ESortGroup::Skybox &&
			sort_group < ESortGroup::AlphaBack &&
			(!alpha_pass || fade_world))
		{
			// Write fixed black RGB to target zero only, or replace the matching faded layer without inserting another fragment.
			auto prev_fill_mode = desc.Get<EPipeState::FillMode>();
			auto prev_pixel_shader = desc.Get<EPipeState::PS>();
			auto prev_depth_write = desc.Get<EPipeState::DepthWriteMask>();
			auto prev_depth_func = desc.Get<EPipeState::DepthFunc>();
			desc.Apply(PSO<EPipeState::FillMode>(D3D12_FILL_MODE_WIREFRAME));
			desc.Apply(PSO<EPipeState::PS>(alpha_pass
				? shader_code::forward_far_fade_wire_collect_ps
				: fade_world
					? shader_code::forward_far_fade_wire_ps
					: shader_code::forward_wire_ps));
			if (!alpha_pass)
			{
				// Preserve the visible opaque surface and accept only its coplanar diagnostic edge.
				desc.Apply(PSO<EPipeState::DepthWriteMask>(D3D12_DEPTH_WRITE_MASK_ZERO));
				desc.Apply(PSO<EPipeState::DepthFunc>(D3D12_COMPARISON_FUNC_LESS_EQUAL));
			}
			set_pipe_state();

			if (nugget.m_irange.empty())
			{
				cmd_list.DrawInstanced(
					s_cast<size_t>(nugget.m_vrange.size()), 1U,
					s_cast<size_t>(nugget.m_vrange.m_beg), 0U);
			}
			else
			{
				cmd_list.DrawIndexedInstanced(
					s_cast<size_t>(nugget.m_irange.size()), 1U,
					s_cast<size_t>(nugget.m_irange.m_beg), 0, 0U);
			}

			// Restore the caller-owned material pipeline before drawing the next nugget.
			desc.Apply(PSO<EPipeState::FillMode>(prev_fill_mode));
			desc.Apply(PSO<EPipeState::PS>(prev_pixel_shader));
			desc.Apply(PSO<EPipeState::DepthWriteMask>(prev_depth_write));
			desc.Apply(PSO<EPipeState::DepthFunc>(prev_depth_func));
		}

		// Render points for 'Points' mode
		if (fill_mode == EFillMode::Points)
		{
			// Configure the shader for point sprites
			// Don't need 'dle' if the points aren't in screen space
			wnd().m_diag.m_gs_fillmode_points->SetupElement(cmd_list.get(), m_upload_buffer, scn(), nullptr);

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
