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
#include "pr/view3d-12/resource/resource_factory.h"
#include "pr/view3d-12/scene/procedural_sky.h"
#include "pr/view3d-12/texture/texture_desc.h"
#include "pr/view3d-12/utility/pipe_state.h"
#include "view3d-12/src/shaders/common.h"

namespace pr::rdr12
{
	using namespace ::pr::compute;

	// The procedural sky's low cloud accumulated over frames. The sky shader reads the previous frame's history and writes this frame's.
	struct RenderForward::SkyHistory
	{
		// The history textures, with one texel per 2x2 pixel quad of the render target. [0] is written this frame, [1] holds the previous frame.
		Texture2DPtr m_tex[2];
		iv2 m_size;

		// The previous frame's viewport, world to screen transform, and low cloud drift (see ProceduralSky::m_low_cloud_drift), and whether the previous frame wrote the history.
		v4 m_prev_viewport;
		m4x4 m_prev_w2s;
		std::array<double, 2> m_prev_cloud_drift;
		bool m_valid;

		// The number of frames accumulated, which selects each frame's offset along the march steps.
		uint64_t m_frame;

		// The constants for this frame's shaders.
		shaders::fwd::SkyHistory m_constants;

		SkyHistory()
			: m_tex()
			, m_size()
			, m_prev_viewport()
			, m_prev_w2s(m4x4::Identity())
			, m_prev_cloud_drift()
			, m_valid()
			, m_frame()
			, m_constants()
		{}
	};

	// The reflection of the opaque scene in the scene's reflection plane, projected each frame by three compute passes (see reflection_plane.hlsl).
	// The main view is projected pixel for pixel. A second source, the mirror view, is the opaque scene and the sky drawn from
	// the mirror image of the camera, with the near plane moved onto the reflection plane. It holds the true reflection, so it fills in surfaces
	// that near objects hide in the main view and objects beyond the edges of the screen.
	struct RenderForward::ReflectionPlane
	{
		// Root parameters of the projection passes. Each view gets its own table, because consecutive heap entries can wrap around the ring.
		enum class EParam { Constants, Colour, Depth, Key, Raw, Final };

		// The mirror view's keys are scaled by this, so that where the main view and the mirror view see the same surface, the sharper main view wins
		static constexpr float MirrorKeyScale = 0.95f;

		// Matches 'CBufReflectionPlane' in reflection_plane.hlsl
		struct Constants
		{
			m4x4 s2w;
			m4x4 w2s;
			v4 plane;
			v4 viewport;
			v4 cam;
			v4 src0;
			v4 src1;
			v4 src2;
		};

		// The passes' shared root signature and their pipeline states
		D3DPtr<ID3D12RootSignature> m_sig;
		D3DPtr<ID3D12PipelineState> m_project;
		D3DPtr<ID3D12PipelineState> m_write;
		D3DPtr<ID3D12PipelineState> m_fill;

		// The winning depth key per pixel, the raw projected reflection, and the gap-filled reflection read by the shaders.
		// All are the size of the back buffer.
		Texture2DPtr m_key;
		Texture2DPtr m_raw;
		Texture2DPtr m_final;
		iv2 m_size;

		// The mirror view's colour and depth, and the drawn size in pixels. They use the main target's formats and sampling, so the scene's
		// pipeline states draw into them.
		Texture2DPtr m_mirror_colour;
		Texture2DPtr m_mirror_depth;
		iv2 m_mirror_size;

		// This frame's mirror camera, as seen by the shaders. 'm_mirror_s2w' maps the mirror view's normalised device coordinates to the world
		// position of the surface drawn there, 'm_mirror_w2s' maps world positions to the mirror view's render target clip space, and 'm_mirror_eye'
		// is the mirror image of the camera position.
		shaders::fwd::ViewOverride m_view;
		m4x4 m_mirror_s2w;
		m4x4 m_mirror_w2s;
		v4 m_mirror_eye;

		// True when the mirror view is drawn this frame
		bool m_active;

		// The constants for this frame's shaders.
		shaders::fwd::ReflectionPlane m_constants;

		explicit ReflectionPlane(ID3D12Device4* device)
			: m_sig()
			, m_project()
			, m_write()
			, m_fill()
			, m_key()
			, m_raw()
			, m_final()
			, m_size()
			, m_mirror_colour()
			, m_mirror_depth()
			, m_mirror_size()
			, m_view()
			, m_mirror_s2w(m4x4::Identity())
			, m_mirror_w2s(m4x4::Identity())
			, m_mirror_eye()
			, m_active()
			, m_constants()
		{
			// The three passes read and write the same resources, so they share one signature
			m_sig = RootSig(ERootSigFlags::ComputeOnly)
				.U32(ECBufReg::b0, sizeof(Constants) / sizeof(uint32_t))
				.SRV(ESRVReg::t0, 1, D3D12_SHADER_VISIBILITY_ALL, D3D12_DESCRIPTOR_RANGE_FLAG_DESCRIPTORS_VOLATILE)
				.SRV(ESRVReg::t1, 1, D3D12_SHADER_VISIBILITY_ALL, D3D12_DESCRIPTOR_RANGE_FLAG_DESCRIPTORS_VOLATILE)
				.UAV(EUAVReg::u0, 1, D3D12_SHADER_VISIBILITY_ALL, D3D12_DESCRIPTOR_RANGE_FLAG_DESCRIPTORS_VOLATILE)
				.UAV(EUAVReg::u1, 1, D3D12_SHADER_VISIBILITY_ALL, D3D12_DESCRIPTOR_RANGE_FLAG_DESCRIPTORS_VOLATILE)
				.UAV(EUAVReg::u2, 1, D3D12_SHADER_VISIBILITY_ALL, D3D12_DESCRIPTOR_RANGE_FLAG_DESCRIPTORS_VOLATILE)
				.Create(device, "ReflectionPlaneSig");
			m_project = ComputePSO(m_sig.get(), shader_code::reflection_plane_project_cs).Create(device, "ReflectionPlaneProjectPSO");
			m_write = ComputePSO(m_sig.get(), shader_code::reflection_plane_write_cs).Create(device, "ReflectionPlaneWritePSO");
			m_fill = ComputePSO(m_sig.get(), shader_code::reflection_plane_fill_cs).Create(device, "ReflectionPlaneFillPSO");
		}
	};

	RenderForward::RenderForward(Scene& scene)
		: RenderStep(Id, scene, scene.wnd().m_gsync)
		, m_shader(scene.rdr())
		, m_cmd_list(scene.d3d(), nullptr, "RenderForward", EColours::Blue)
		, m_alp_list(scene.d3d(), nullptr, "RenderForwardAlpha", EColours::Blue)
		, m_default_tex(rdr().store().StockTexture(EStockTexture::White))
		, m_default_sam(rdr().store().StockSampler(EStockSampler::LinearClamp))
		, m_sky_history(std::make_unique<SkyHistory>())
		, m_reflection_plane(std::make_unique<ReflectionPlane>(scene.d3d()))
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
			.DepthStencilState = DepthStateDesc{}.Func(D3D12_COMPARISON_FUNC_GREATER), // Reversed depth: nearer surfaces have greater depth
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
		auto const* sort_key_override = inst.find<SKOverride>(EInstComp::SortkeyOverride);

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
				if (sort_key_override != nullptr)
					sk = sort_key_override->Combine(sk);

				// Let the material add pass-specific sort key information.
				sk = pass->AddSortKey(m_step_id, inst, material, nug, sk);

				// Reject unsupported materials before a window starts recording a frame, because a frame abandoned during recording cannot be retired.
				// Only procedural pixel families can make the pixel shader selection fail, so other draws skip the composition.
				if (materials::ProceduralPixelFamily(material, m_step_id) != nullptr)
				{
					auto desc = m_default_pipe_state;
					ComposePipeline(desc, nug, inst, material, *pass, false, false);
				}

				// Add an element to the draw list
				drawlist.push_back(DrawListElement{ .m_sort_key = sk, .m_nugget = &nug, .m_instance = &inst });
				m_sort_needed = true;
			}
		}
	}

	// Compose the pipeline state for one forward draw.
	void RenderForward::ComposePipeline(PipeStateDesc& desc, Nugget const& nugget, BaseInstance const& instance, Material const& material, MaterialPass const& pass, bool population, bool mirrored) const
	{
		// The material selects its shaders after the geometry and caller-owned overrides, so it can validate and replace the final stock state.
		ApplyGeometryPipeline(desc, nugget);
		ApplyPipeStateOverrides(desc, nugget, instance);
		pass.SelectPipeline(MaterialPipelineContext{ .m_step_id = m_step_id, .m_material = material, .m_nugget = nugget }, desc);

		// A mirror reverses the winding of every triangle on screen, so the faces that are culled swap
		if (mirrored)
		{
			switch (desc.Get<EPipeState::CullMode>())
			{
				case D3D12_CULL_MODE_BACK: desc.Apply(PSO<EPipeState::CullMode>(D3D12_CULL_MODE_FRONT)); break;
				case D3D12_CULL_MODE_FRONT: desc.Apply(PSO<EPipeState::CullMode>(D3D12_CULL_MODE_BACK)); break;
				case D3D12_CULL_MODE_NONE: break;
				default: throw std::runtime_error("Unknown cull mode");
			}
		}
		if (!population)
			return;

		// Population draws place vertices with the population vertex shaders. Procedural vertex shader overlays (accepted by population admission)
		// place population vertices themselves, so they are kept. Stock vertex shaders are swapped for their population variants. Skinned LOD models
		// use the variants that pose each instance from the payload.
		auto vs = desc.Get<EPipeState::VS>().pShaderBytecode;
		auto const* overlay = PopulationVertexOverlay(material, m_step_id);
		auto skinned = static_cast<bool>(nugget.m_model->m_skin);
		if (overlay != nullptr && vs == overlay->m_code.VS.pShaderBytecode)
			return;

		// Only stock vertex shaders have population variants
		if (vs == shader_code::forward_vs.pShaderBytecode)
			desc.Apply(PSO<EPipeState::VS>(skinned ? shader_code::forward_skinned_population_vs : shader_code::forward_population_vs));
		else if (vs == shader_code::forward_texn_pbr_vs.pShaderBytecode)
			desc.Apply(PSO<EPipeState::VS>(skinned ? shader_code::forward_texn_skinned_population_vs : shader_code::forward_texn_population_vs));
		else
			throw std::logic_error("Population draw has a vertex shader without a population variant");
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

		// Classify the populations before any draws, so the classification can overlap earlier work on the GPU. The classification includes the
		// reflection's mirror view, so the mirror camera is set up first.
		PrepareMirror();
		CullPopulations();

		// Bind the descriptor heaps
		auto des_heaps = { wnd().m_heap_view.get(), wnd().m_heap_samp.get() };
		m_cmd_list.SetDescriptorHeaps({ des_heaps.begin(), des_heaps.size() });
		m_alp_list.SetDescriptorHeaps({ des_heaps.begin(), des_heaps.size() });
		PrepareSkyHistory();

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

			// Draw the opaques and the populations, which are opaque. Without a K-buffer, sorted alpha draws follow on this list after the populations.
			auto drawlist = m_drawlist.lock();
			auto alpha_start = boundaries[ESortGroup::Alpha];
			auto pix_opaque = pix::EventScope<ID3D12GraphicsCommandList>(m_cmd_list.get(), 0xFF7EB8E5, "View3D::Opaque");
			DrawOpaques(frame, pipe_state, std::span{ *drawlist }.subspan(0, s_cast<size_t>(alpha_start)), m_populations, 0.0f);
			if (!kbuf)
				DrawNuggets(frame, m_cmd_list, pipe_state, std::span{ *drawlist }.subspan(s_cast<size_t>(alpha_start)), alpha_start, ESubPass::Opaque);

			DrawMirror(frame, std::span{ *drawlist }.subspan(0, s_cast<size_t>(alpha_start)));
		}

		// Render the alpha nuggets
		if (kbuf)
		{
			// Separate transparent collection from opaque work recorded on the other command list.
			auto pix_alpha = pix::EventScope<ID3D12GraphicsCommandList>(m_alp_list.get(), 0xFFBA84E5, "View3D::Alpha");
			ProjectReflectionPlane(frame);
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
			auto alpha_start = boundaries[ESortGroup::Alpha];
			auto alpha_end = boundaries[ESortGroup::PostAlpha];

			// Use the alpha collect shader and disable depth writes for the alpha pass
			DrawNuggets(frame, m_alp_list, m_alpha_pipe_state, std::span{ *drawlist }.subspan(s_cast<size_t>(alpha_start), s_cast<size_t>(alpha_end - alpha_start)), alpha_start, ESubPass::Alpha);

			// Only the alpha surfaces read the reflection. Later passes rebind the frame without it.
			m_shader.m_reflection_plane = nullptr;

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

	// Add the pipeline states of the draw list and the scene's populations, for each sub-pass that can draw them
	void RenderForward::CollectPipelines(std::vector<PipeStateDesc>& descs) const
	{
		// Choose the sub-pass defaults in the same way as 'Execute'. The opaque pass uses the reflection sidecar when ray traced reflections are on.
		auto const* ray_tracing = scn().FindRStep<RenderRayTracing>();
		auto const rt_pass = ray_tracing != nullptr ? ray_tracing->ScreenPass() : ERayTracingScreenPass::None;
		auto const reflections = ray_tracing != nullptr && rdr().RayTracing().Available() && (rt_pass == ERayTracingScreenPass::Reflections || rt_pass == ERayTracingScreenPass::ReflectionsAndCaustics);
		auto const& opaque_state = reflections ? m_reflection_pipe_state : m_default_pipe_state;
		auto const kbuf = static_cast<bool>(wnd().m_alpha_kbuffer);

		// The reflection's mirror view redraws the opaques with the default state and reversed culling. Whether it is drawn depends on the camera,
		// so its states are collected whenever a reflection plane can be projected. See 'ReflectionPlaneNeeded'.
		auto const mirror = kbuf && !All(scn().m_reflection_plane == v4::Zero());

		// Add the states of one nugget drawn from 'base', with the fill mode variants that 'DrawNugget' applies
		auto add = [&](PipeStateDesc const& base, Nugget const& nugget, BaseInstance const& instance, bool population, bool mirrored)
		{
			// Draws without a pass for this step are skipped, as in 'DrawNuggets'
			auto material_override = FindMaterial(instance);
			auto const& material = material_override != nullptr ? *material_override.get() : nugget.mat();
			auto const* pass = material.Pass(m_step_id);
			if (pass == nullptr)
				return;

			auto desc = base;
			ComposePipeline(desc, nugget, instance, material, *pass, population, mirrored);

			// The point fill mode is a diagnostic view, so its pipelines are created when first drawn
			auto const fill_mode = nugget.FillMode() != EFillMode::Default ? nugget.FillMode() : scn().FillMode();
			switch (fill_mode)
			{
				case EFillMode::Default:
				case EFillMode::Solid:
				{
					descs.push_back(desc);
					break;
				}
				case EFillMode::Wireframe:
				{
					desc.Apply(PSO<EPipeState::FillMode>(D3D12_FILL_MODE_WIREFRAME));
					descs.push_back(desc);
					break;
				}
				case EFillMode::SolidWire:
				{
					descs.push_back(desc);
					desc.Apply(PSO<EPipeState::FillMode>(D3D12_FILL_MODE_WIREFRAME));
					desc.Apply(PSO<EPipeState::BlendState0>({FALSE}));
					descs.push_back(desc);
					break;
				}
				case EFillMode::Points:
				{
					break;
				}
				default:
				{
					throw std::runtime_error("Unknown fill mode");
				}
			}
		};

		// Add the states of an opaque draw, including its mirror variant
		auto add_opaque = [&](Nugget const& nugget, BaseInstance const& instance, bool population)
		{
			// The mirror view draws every opaque group, including the skybox
			add(opaque_state, nugget, instance, population, false);
			if (mirror)
				add(m_default_pipe_state, nugget, instance, population, true);
		};

		// Draw list elements use the sub-pass of their sort group. Without a K-buffer, every group draws with the opaque state.
		{
			auto drawlist = m_drawlist.lock();
			for (auto const& dle : *drawlist)
			{
				auto const group = dle.m_sort_key.Group();
				if (!kbuf || group < ESortGroup::Alpha)
					add_opaque(*dle.m_nugget, *dle.m_instance, false);
				else if (group < ESortGroup::PostAlpha)
					add(m_alpha_pipe_state, *dle.m_nugget, *dle.m_instance, false, false);
				else
					add(m_post_alpha_pipe_state, *dle.m_nugget, *dle.m_instance, false, false);
			}
		}

		// Population draws are drawn with the opaques, through each list's proxy instance. Classification can draw any list, so all are added.
		for (auto const& pop : scn().Populations())
		{
			for (auto const& proxy : pop->m_proxies)
			{
				PopulationNuggets(proxy, m_step_id, [&](Nugget const& nug, Material const&)
				{
					add_opaque(nug, proxy.m_base, true);
				});
			}
		}
	}

	// Prepare the sky's low cloud history for this frame.
	void RenderForward::PrepareSkyHistory()
	{
		// Blend frames only when a procedural sky is drawn, in a scene whose camera moves smoothly between frames.
		// Otherwise the shader's history constants are zero, which turns the blending off.
		auto& history = *m_sky_history;
		auto const* sky = scn().CloudSky();
		auto const bb_size = wnd().BackBufferSize();
		if (sky == nullptr || !scn().m_sky_history || bb_size.x == 0 || bb_size.y == 0)
		{
			m_shader.m_sky_history = nullptr;
			history.m_valid = false;
			return;
		}

		// Size the history to one texel per 2x2 pixel quad of the render target. A new size, or a moved viewport, starts a new history.
		auto const& vp = scn().m_viewport;
		auto const viewport = v4(vp.TopLeftX, vp.TopLeftY, vp.Width, vp.Height);
		auto const size = (bb_size + iv2(1, 1)) / 2;
		if (Any(size != history.m_size))
		{
			ResourceFactory factory(rdr());
			auto desc = ResDesc::Tex2D(Image{ size.x, size.y, nullptr, DXGI_FORMAT_R16G16B16A16_FLOAT }, 1U, EUsage::UnorderedAccess)
				.def_state(D3D12_RESOURCE_STATE_ALL_SHADER_RESOURCE);
			history.m_tex[0] = factory.CreateTexture2D(TextureDesc(AutoId, desc).name("SkyHistory0"));
			history.m_tex[1] = factory.CreateTexture2D(TextureDesc(AutoId, desc).name("SkyHistory1"));
			history.m_size = size;
			history.m_valid = false;
		}
		if (Any(viewport != history.m_prev_viewport))
			history.m_valid = false;

		// Read the previous frame's history and write this frame's into the other texture. The written texture is cleared to a negative alpha,
		// which marks pixels the sky does not draw this frame, such as pixels behind scene geometry.
		std::swap(history.m_tex[0], history.m_tex[1]);
		auto& tex_out = *history.m_tex[0].get();
		{
			BarrierBatch bb(m_cmd_list);
			bb.Transition(tex_out.m_res.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			bb.Commit();

			float const cleared[4] = { 0.0f, 0.0f, 0.0f, -1.0f };
			m_cmd_list.ClearUnorderedAccessViewFloat(wnd().m_heap_view.Add(tex_out.m_uav), tex_out.m_uav.m_cpu, tex_out.m_res.get(), cleared);

			// The sky's writes must follow the clear.
			bb.UAV(tex_out.m_res.get());
			bb.Commit();
		}

		// Set the constants that map this frame's cloud to where it was in the previous frame, then remember this frame for the next.
		// Each frame moves the march samples along their steps by the golden ratio, which spreads the samples of consecutive frames evenly.
		auto const& cam = scn().m_cam;
		auto const w2s = cam.CameraToScreen() * InvertOrthonormal(cam.CameraToWorld());
		auto const cloud_drift = std::array<double, 2>{ sky->m_low_cloud_drift[0], sky->m_low_cloud_drift[1] };
		history.m_constants = shaders::fwd::SkyHistory{
			.prev_w2s = history.m_prev_w2s,
			.viewport = viewport,
			.cloud_shift = v2(s_cast<float>(cloud_drift[0] - history.m_prev_cloud_drift[0]), s_cast<float>(cloud_drift[1] - history.m_prev_cloud_drift[1])),
			.frame_jitter = s_cast<float>(std::fmod(s_cast<double>(history.m_frame) * 0.6180339887, 1.0)),
			.valid = history.m_valid ? 1.0f : 0.0f,
			.enabled = 1,
		};
		m_shader.m_sky_history = &history.m_constants;
		history.m_prev_viewport = viewport;
		history.m_prev_w2s = w2s;
		history.m_prev_cloud_drift = cloud_drift;
		history.m_valid = true;
		++history.m_frame;
	}

	// True when this frame needs the reflection of the scene's reflection plane
	bool RenderForward::ReflectionPlaneNeeded() const
	{
		// The reflection needs the K-buffer's separate alpha command list, and is seen only from above the plane
		auto const& vp = scn().m_viewport;
		auto const plane = scn().m_reflection_plane;
		auto const bb_size = wnd().BackBufferSize();
		auto const cam_pos = scn().m_cam.CameraToWorld().pos;
		if (!wnd().m_alpha_kbuffer || All(plane == v4::Zero()) || bb_size.x == 0 || bb_size.y == 0 || vp.Width < 1 || vp.Height < 1)
			return false;

		return Dot(plane.xyz, cam_pos.xyz) - plane.w > 0.0f;
	}

	// Set up this frame's reflection mirror camera, and size its targets
	void RenderForward::PrepareMirror()
	{
		// The mirror view is drawn only when the reflection is projected. A mirror image needs a perspective camera, whose eye can be mirrored.
		auto& rp = *m_reflection_plane;
		rp.m_active = false;
		auto const& cam = scn().m_cam;
		if (!ReflectionPlaneNeeded() || cam.Orthographic())
			return;

		// Reflect world positions through the plane: x -> x - 2 (n.x - d) n. The reflection is its own inverse. Drawing the world through
		// 'w2c * R' shows the main camera what it would see in the mirror, so the mirror camera's camera to world transform is 'R * c2w'.
		auto const plane = scn().m_reflection_plane;
		auto const n = plane.w0();
		auto const d = plane.w;
		auto const R = m4x4{
			v4::XAxis() - 2.0f * n.x * n,
			v4::YAxis() - 2.0f * n.y * n,
			v4::ZAxis() - 2.0f * n.z * n,
			v4::Origin() + 2.0f * d * n,
		};
		auto const c2w = cam.CameraToWorld();
		auto const c2s = cam.CameraToScreen();
		auto const c2w_m = R * c2w;
		auto const w2c_m = InvertOrthonormal(c2w) * R;

		// Move the near plane onto the reflection plane, so that everything below it, which the mirror cannot show, is clipped away. 'P' is the plane
		// in the mirror camera's space, positive above the plane. Depth is reversed, so clip z/w is 1 on the near plane and 0 on the far plane.
		// Replacing the z row with 'row_w - a * P' makes 'w - z' proportional to the height above the plane, so the near plane becomes the reflection
		// plane. 'a' is chosen so that the far plane still passes through the far corner of the view on the plane's side, which keeps every visible
		// point between the near and far planes.
		auto const row = [](m4x4 const& m, int k) { return v4{ m.x[k], m.y[k], m.z[k], m.w[k] }; };
		auto const P = Transpose(c2w_m) * v4{ n.x, n.y, n.z, -d };
		auto const s2c = Invert(c2s);
		auto const P_clip = Transpose(s2c) * P;
		auto const Q = s2c * v4{ P_clip.x < 0.0f ? -1.0f : 1.0f, P_clip.y < 0.0f ? -1.0f : 1.0f, 0.0f, 1.0f };
		auto const row_w = row(c2s, 3);
		auto const a = Dot(row_w, Q) / Dot(P, Q);
		if (!(a > 0.0f) || !std::isfinite(a))
			return;

		auto const row_z = row_w - a * P;
		auto c2s_obl = c2s;
		c2s_obl.x.z = row_z.x;
		c2s_obl.y.z = row_z.y;
		c2s_obl.z.z = row_z.z;
		c2s_obl.w.z = row_z.w;

		// Draw the mirror view at the viewport's resolution, so that its pixels map one to one onto the viewport's pixels. A fractional
		// viewport is covered by rounding up, and the view is scaled to place the viewport's area on that whole number of pixels.
		auto const& vp = scn().m_viewport;
		auto const cw = std::max(s_cast<int>(std::ceil(vp.Width)), 1);
		auto const ch = std::max(s_cast<int>(std::ceil(vp.Height)), 1);
		auto const sx = vp.Width / cw;
		auto const sy = vp.Height / ch;
		auto const remap = m4x4{ v4{ sx, 0, 0, 0 }, v4{ 0, sy, 0, 0 }, v4{ 0, 0, 1, 0 }, v4{ sx - 1.0f, 1.0f - sy, 0, 1 } };

		// The camera seen by the shaders. Element transforms are uploaded for the main camera, so 'clip_remap' replaces the main view's
		// world to clip transform with the mirror's.
		auto const c2s_m = remap * c2s_obl;
		rp.m_view.cam.c2w = c2w_m;
		rp.m_view.cam.c2s = c2s_m;
		rp.m_view.cam.w2c = w2c_m;
		rp.m_view.cam.w2s = c2s_m * w2c_m;
		rp.m_view.clip_remap = rp.m_view.cam.w2s * c2w * s2c;
		rp.m_mirror_w2s = rp.m_view.cam.w2s;
		rp.m_mirror_s2w = Invert(c2s_obl * w2c_m);
		rp.m_mirror_eye = c2w_m.pos;
		rp.m_mirror_size = iv2(cw, ch);

		// Size the textures. They match the main target's formats and sampling, so the scene's pipeline states can draw into them.
		auto const ms = wnd().MultiSampling();
		auto const resize = rp.m_mirror_colour == nullptr ||
			s_cast<int>(rp.m_mirror_colour->m_res->GetDesc().Width) != cw ||
			s_cast<int>(rp.m_mirror_colour->m_res->GetDesc().Height) != ch ||
			rp.m_mirror_colour->m_res->GetDesc().SampleDesc.Count != ms.Count;
		if (resize)
		{
			// Colour is written and read through sRGB views, like the main target
			ResourceFactory factory(rdr());
			auto const rt_format = wnd().m_rt_props.Format;
			auto const ds_format = wnd().m_ds_props.Format;
			auto col_desc = ResDesc::Tex2D(Image{ cw, ch, nullptr, rt_format }, 1U, EUsage::RenderTarget)
				.multisamp(ms)
				.clear(wnd().m_rt_props)
				.def_state(D3D12_RESOURCE_STATE_RENDER_TARGET);
			auto dep_desc = ResDesc::Tex2D(Image{ cw, ch, nullptr, DepthResourceFormat(ds_format) }, 1U, EUsage::DepthStencil)
				.multisamp(ms)
				.clear(wnd().m_ds_props)
				.def_state(D3D12_RESOURCE_STATE_DEPTH_WRITE);
			rp.m_mirror_colour = factory.CreateTexture2D(TextureDesc(AutoId, col_desc).rtv_format(ToSRGB(rt_format)).srv_format(ToSRGB(rt_format)).name("ReflectionMirrorColour"));
			rp.m_mirror_depth = factory.CreateTexture2D(TextureDesc(AutoId, dep_desc).dsv_format(ds_format).srv_format(DepthSrvFormat(ds_format)).name("ReflectionMirrorDepth"));
		}
		rp.m_active = true;
	}

	// Draw the opaque scene and the sky through the reflection's mirror camera
	void RenderForward::DrawMirror(Frame& frame, std::span<DrawListElement const> opaques)
	{
		// Clear the mirror view. The sky does not write depth, so where it shows the depth stays 0, which the projection passes treat as sky.
		auto& rp = *m_reflection_plane;
		if (!rp.m_active)
			return;

		auto pix_mirror = pix::EventScope<ID3D12GraphicsCommandList>(m_cmd_list.get(), 0xFF5EC8D5, "View3D::ReflectionMirror");
		auto const rtv = rp.m_mirror_colour->m_rtv.m_cpu;
		auto const dsv = rp.m_mirror_depth->m_dsv.m_cpu;
		auto const viewport = D3D12_VIEWPORT{ .TopLeftX = 0.0f, .TopLeftY = 0.0f, .Width = s_cast<float>(rp.m_mirror_size.x), .Height = s_cast<float>(rp.m_mirror_size.y), .MinDepth = 0.0f, .MaxDepth = 1.0f };
		auto const scissor = D3D12_RECT{ 0, 0, rp.m_mirror_size.x, rp.m_mirror_size.y };
		m_cmd_list.ClearRenderTargetView(rtv, frame.bb_main().rt_clear());
		m_cmd_list.ClearDepthStencilView(dsv, D3D12_CLEAR_FLAG_DEPTH, frame.bb_main().ds_depth(), 0);
		m_cmd_list.OMSetRenderTargets({ &rtv, 1 }, FALSE, &dsv);
		m_cmd_list.get()->RSSetViewports(1, &viewport);
		m_cmd_list.get()->RSSetScissorRects(1, &scissor);

		// Draw everything through the mirror camera. The sky's cloud history belongs to the main view's pixels, so the mirror's sky neither reads
		// nor writes it. Small populations add little to a reflection, so they are left out.
		auto sky_history = m_shader.m_sky_history;
		m_shader.m_sky_history = nullptr;
		m_shader.m_view = &rp.m_view;
		m_mirrored = true;
		BindFrameResources(m_cmd_list);
		DrawOpaques(frame, m_default_pipe_state, opaques, m_mirror_populations, scn().m_reflection_min_population_radius);

		// Restore the main view for any later draws
		auto const& vp = scn().m_viewport;
		m_shader.m_sky_history = sky_history;
		m_shader.m_view = nullptr;
		m_mirrored = false;
		BindFrameResources(m_cmd_list);
		m_cmd_list.RSSetViewports({ &vp, 1 });
		m_cmd_list.RSSetScissorRects(vp.m_clip);
	}

	// Project the opaque scene into the reflection of the scene's reflection plane.
	void RenderForward::ProjectReflectionPlane(Frame& frame)
	{
		// The reflection must be projected after the opaques are complete and before the alpha draws read it
		auto& rp = *m_reflection_plane;
		auto const& cam = scn().m_cam;
		auto const& vp = scn().m_viewport;
		auto const plane = scn().m_reflection_plane;
		auto const bb_size = wnd().BackBufferSize();
		auto const cam_pos = cam.CameraToWorld().pos;
		m_shader.m_reflection_plane = nullptr;
		if (!ReflectionPlaneNeeded())
			return;

		// Size the textures to the back buffer
		if (Any(bb_size != rp.m_size))
		{
			ResourceFactory factory(rdr());
			auto key_desc = ResDesc::Tex2D(Image{ bb_size.x, bb_size.y, nullptr, DXGI_FORMAT_R32_UINT }, 1U, EUsage::UnorderedAccess).def_state(D3D12_RESOURCE_STATE_ALL_SHADER_RESOURCE);
			auto col_desc = ResDesc::Tex2D(Image{ bb_size.x, bb_size.y, nullptr, DXGI_FORMAT_R16G16B16A16_FLOAT }, 1U, EUsage::UnorderedAccess).def_state(D3D12_RESOURCE_STATE_ALL_SHADER_RESOURCE);
			rp.m_key = factory.CreateTexture2D(TextureDesc(AutoId, key_desc).name("ReflectionPlaneKey"));
			rp.m_raw = factory.CreateTexture2D(TextureDesc(AutoId, col_desc).name("ReflectionPlaneRaw"));
			rp.m_final = factory.CreateTexture2D(TextureDesc(AutoId, col_desc).name("ReflectionPlane"));
			rp.m_size = bb_size;
		}

		auto pix_scope = pix::EventScope<ID3D12GraphicsCommandList>(m_alp_list.get(), 0xFF5EC8D5, "View3D::ReflectionPlane");
		auto& bb_main = frame.bb_main();
		auto* colour = bb_main.m_render_target.get();
		auto* depth = bb_main.m_depth_stencil.get();

		// The opaque colour and depth become inputs, and every target texel starts with no candidate
		BarrierBatch bb(m_alp_list);
		bb.Transition(colour, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		bb.Transition(depth, D3D12_RESOURCE_STATE_ALL_SHADER_RESOURCE);
		if (rp.m_active)
		{
			bb.Transition(rp.m_mirror_colour->m_res.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
			bb.Transition(rp.m_mirror_depth->m_res.get(), D3D12_RESOURCE_STATE_ALL_SHADER_RESOURCE);
		}
		bb.Transition(rp.m_key->m_res.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		bb.Transition(rp.m_raw->m_res.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		bb.Transition(rp.m_final->m_res.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		bb.Commit();

		UINT const cleared[4] = { 0, 0, 0, 0 };
		m_alp_list.ClearUnorderedAccessViewUint(wnd().m_heap_view.Add(rp.m_key->m_uav), rp.m_key->m_uav.m_cpu, rp.m_key->m_res.get(), cleared);
		bb.UAV(rp.m_key->m_res.get());
		bb.Commit();

		// Describe the sources. Both views are read pixel for pixel; the mirror view's pixels map onto the viewport from its top-left corner.
		// 's2w' maps each source's normalised
		// device coordinates to the world position of the surface drawn there. The colour is read through an sRGB view, which converts it to linear
		// colour. Creating a view does not modify the frame's back buffer, so it is safe to drop the const.
		struct Source
		{
			v4 src0, src1, src2;
			m4x4 s2w;
			D3D12_GPU_DESCRIPTOR_HANDLE colour, depth;
		};
		auto colour_srv = D3D12_SHADER_RESOURCE_VIEW_DESC{
			.Format = ToSRGB(wnd().m_rt_props.Format),
			.ViewDimension = D3D12_SRV_DIMENSION_TEXTURE2DMS,
			.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING,
		};
		auto sources = pr::vector<Source, 2, false>{};
		sources.push_back(Source{
			.src0 = v4(2.0f / vp.Width, 0.0f, -1.0f, vp.Width),
			.src1 = v4(0.0f, -2.0f / vp.Height, 1.0f, vp.Height),
			.src2 = v4(vp.TopLeftX, vp.TopLeftY, 1.0f, 0.0f),
			.s2w = cam.CameraToWorld() * Invert(cam.CameraToScreen()),
			.colour = wnd().m_heap_view.Add(const_cast<ID3D12Resource*>(colour), colour_srv),
			.depth = wnd().m_heap_view.Add(bb_main.m_depth_srv),
		});
		if (rp.m_active)
		{
			sources.push_back(Source{
				.src0 = v4(2.0f / vp.Width, 0.0f, -1.0f, s_cast<float>(rp.m_mirror_size.x)),
				.src1 = v4(0.0f, -2.0f / vp.Height, 1.0f, s_cast<float>(rp.m_mirror_size.y)),
				.src2 = v4(0.0f, 0.0f, ReflectionPlane::MirrorKeyScale, 0.0f),
				.s2w = rp.m_mirror_s2w,
				.colour = wnd().m_heap_view.Add(rp.m_mirror_colour->m_srv),
				.depth = wnd().m_heap_view.Add(rp.m_mirror_depth->m_srv),
			});
		}

		// Bind the passes' constants and outputs
		auto const w2s = cam.CameraToScreen() * InvertOrthonormal(cam.CameraToWorld());
		auto const viewport = v4(vp.TopLeftX, vp.TopLeftY, vp.Width, vp.Height);
		auto cb = ReflectionPlane::Constants{
			.w2s = w2s,
			.plane = plane,
			.viewport = viewport,
			.cam = cam_pos,
		};
		using EParam = ReflectionPlane::EParam;
		m_alp_list.SetComputeRootSignature(rp.m_sig.get());
		m_alp_list.SetComputeRootDescriptorTable(EParam::Key, wnd().m_heap_view.Add(rp.m_key->m_uav));
		m_alp_list.SetComputeRootDescriptorTable(EParam::Raw, wnd().m_heap_view.Add(rp.m_raw->m_uav));
		m_alp_list.SetComputeRootDescriptorTable(EParam::Final, wnd().m_heap_view.Add(rp.m_final->m_uav));

		// Run each pass over every source, one thread per source pixel in 8x8 groups. Each pass reads what the previous one wrote.
		for (auto* pso : { rp.m_project.get(), rp.m_write.get() })
		{
			// Dispatch one pass per source, then make its writes visible to the next pass
			m_alp_list.SetPipelineState(pso);
			for (auto const& src : sources)
			{
				// Bind the source and its map
				cb.s2w = src.s2w;
				cb.src0 = src.src0;
				cb.src1 = src.src1;
				cb.src2 = src.src2;
				m_alp_list.SetComputeRoot32BitConstants(EParam::Constants, sizeof(cb) / sizeof(uint32_t), &cb, 0);
				m_alp_list.SetComputeRootDescriptorTable(EParam::Colour, src.colour);
				m_alp_list.SetComputeRootDescriptorTable(EParam::Depth, src.depth);
				m_alp_list.Dispatch((s_cast<int>(src.src0.w) + 7) / 8, (s_cast<int>(src.src1.w) + 7) / 8, 1);
			}
			bb.UAV(rp.m_key->m_res.get());
			bb.UAV(rp.m_raw->m_res.get());
			bb.Commit();
		}

		// Fill the gaps once over the viewport, one thread per viewport pixel
		m_alp_list.SetPipelineState(rp.m_fill.get());
		m_alp_list.Dispatch((s_cast<int>(vp.Width) + 7) / 8, (s_cast<int>(vp.Height) + 7) / 8, 1);
		bb.UAV(rp.m_final->m_res.get());
		bb.Commit();

		// The alpha draws read the final reflection, and the colour targets return to their normal states
		bb.Transition(rp.m_final->m_res.get(), D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE);
		bb.Transition(colour, D3D12_RESOURCE_STATE_RENDER_TARGET);
		if (rp.m_active)
		{
			bb.Transition(rp.m_mirror_colour->m_res.get(), D3D12_RESOURCE_STATE_RENDER_TARGET);
			bb.Transition(rp.m_mirror_depth->m_res.get(), D3D12_RESOURCE_STATE_DEPTH_WRITE);
		}
		bb.Commit();

		rp.m_constants = shaders::fwd::ReflectionPlane{
			.plane = plane,
			.viewport = viewport,
			.enabled = 1,
		};
		m_shader.m_reflection_plane = &rp.m_constants;
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

		// Add the global environment map. Without a previous map or distances, the current map fills those slots so the tables are always valid.
		// The shader reads distances only when the map has them (see 'SetEnvMapConstants').
		if (auto* envmap = scn().m_global_envmap.get())
		{
			auto gpu = wnd().m_heap_view.Add(envmap->m_srv);
			cmd_list.SetGraphicsRootDescriptorTable(shaders::fwd::ERootParam::EnvMap, gpu);

			auto* envmap_prev = scn().m_global_envmap_prev.get();
			auto gpu_prev = envmap_prev != nullptr ? wnd().m_heap_view.Add(envmap_prev->m_srv) : gpu;
			cmd_list.SetGraphicsRootDescriptorTable(shaders::fwd::ERootParam::EnvMapPrev, gpu_prev);

			auto* envmap_distance = envmap->m_distance.get();
			auto gpu_distance = envmap_distance != nullptr ? wnd().m_heap_view.Add(envmap_distance->m_srv) : gpu;
			cmd_list.SetGraphicsRootDescriptorTable(shaders::fwd::ERootParam::EnvMapDistance, gpu_distance);
		}

		// Add the sky's low cloud history. It is only read and written while the history constants are set (see PrepareSkyHistory).
		if (m_shader.m_sky_history != nullptr)
		{
			auto& history = *m_sky_history;
			cmd_list.SetGraphicsRootDescriptorTable(shaders::fwd::ERootParam::SkyHistory, wnd().m_heap_view.Add(history.m_tex[1]->m_srv));
			cmd_list.SetGraphicsRootDescriptorTable(shaders::fwd::ERootParam::SkyHistoryOut, wnd().m_heap_view.Add(history.m_tex[0]->m_uav));
		}

		// Add the projected reflection of the scene's reflection plane. It is only read while the reflection constants are set (see ProjectReflectionPlane).
		if (m_shader.m_reflection_plane != nullptr)
			cmd_list.SetGraphicsRootDescriptorTable(shaders::fwd::ERootParam::ReflectionPlane, wnd().m_heap_view.Add(m_reflection_plane->m_final->m_srv));
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

	// Classify the scene's populations for the camera and the mirror camera, and build the population draws.
	void RenderForward::CullPopulations()
	{
		// Record the classifications on the opaque command list, which also records the draws that consume them. Each view has its own key,
		// so that the populations keep separate results for each.
		auto pix_cull = pix::EventScope<ID3D12GraphicsCommandList>(m_cmd_list.get(), 0xFF5AC8A0, "View3D::PopulationCull");
		CullPopulations(m_populations, PopulationCameraView(reinterpret_cast<uint64_t>(this), scn().m_cam, scn().m_viewport.Height));

		// The mirror view measures draw distances from the mirrored eye, which is as far from each object as the eye's view of its reflection
		auto const& rp = *m_reflection_plane;
		if (rp.m_active)
			CullPopulations(m_mirror_populations, PopulationProjectionView(reinterpret_cast<uint64_t>(this) + 1, rp.m_mirror_w2s, s_cast<float>(rp.m_mirror_size.y), rp.m_mirror_eye));
		else
			m_mirror_populations = PopulationSet{};
	}

	// Classify the scene's populations for 'view' into 'set', and build its draws
	void RenderForward::CullPopulations(PopulationSet& set, PopulationViewDesc const& view)
	{
		// Population draws reference the classification results, so rebuild both each frame.
		set.m_lists.resize(0);
		set.m_dles.resize(0);
		set.m_draws.resize(0);
		set.m_elements = {};
		auto populations = scn().Populations();
		if (populations.empty())
			return;

		// Classify each population. The draws reference the lists, so reserve them first to keep their addresses stable.
		set.m_lists.reserve(populations.size());
		for (auto pop : populations)
		{
			set.m_lists.push_back(rdr().PopulationCulling().Cull(m_cmd_list, m_upload_buffer, m_gsync, *pop.get(), view, m_step_id));
		}

		// Each (LOD, nugget) draw takes its element constants from the LOD's proxy instance, and its sort group from its population
		std::vector<DrawListElement> dles;
		std::vector<PopulationDrawRef> draws;
		for (auto const& lists : set.m_lists)
		{
			for (auto const& draw : lists.m_draws)
			{
				auto sort_key = draw.m_nugget->m_sort_key;
				sort_key.Group(lists.m_population->m_sort_group);
				dles.push_back(DrawListElement{ .m_sort_key = sort_key, .m_nugget = draw.m_nugget, .m_instance = &draw.m_proxy->m_base });
				draws.push_back(PopulationDrawRef{ .m_lists = &lists, .m_draw = &draw });
			}
		}

		// Order the draws by sort key so that they can be merged with the drawlist by group. Within a group, draws that share a shader and texture
		// become adjacent, so they reuse bound descriptors and pipeline state. Draws with equal keys keep their population order.
		std::vector<int> order(dles.size());
		std::iota(order.begin(), order.end(), 0);
		std::ranges::stable_sort(order, {}, [&](int i) { return dles[i].m_sort_key.m_value; });
		set.m_dles.reserve(order.size());
		set.m_draws.reserve(order.size());
		for (auto i : order)
		{
			set.m_dles.push_back(dles[i]);
			set.m_draws.push_back(draws[i]);
		}

		// Upload the element constants of every population draw once. Each population draw selects its entry by position.
		// The constants are for the main camera. Views drawn through another camera correct them with their clip remap.
		if (!set.m_dles.empty())
			set.m_elements = shaders::Forward::UploadElements(m_upload_buffer, scn(), CameraTransforms(scn().m_cam), set.m_dles);
	}

	// Draw the opaque drawlist elements and the classified populations, merged in sort group order
	void RenderForward::DrawOpaques(Frame& frame, PipeStateDesc const& pipe_state, std::span<DrawListElement const> opaques, PopulationSet const& pops, float min_population_radius)
	{
		// Draw one sort group at a time: its drawlist elements, then its population draws. Both sequences are sorted by group.
		auto const pop_dles = std::span<DrawListElement const>{ pops.m_dles };
		for (size_t d = 0, p = 0; d != opaques.size() || p != pop_dles.size();)
		{
			// Find the lowest group that has draws remaining
			auto group = ESortGroup::Max;
			if (d != opaques.size())
				group = std::min(group, opaques[d].m_sort_key.Group());
			if (p != pop_dles.size())
				group = std::min(group, pop_dles[p].m_sort_key.Group());

			// Find the end of the group in both sequences
			auto d_end = d;
			auto p_end = p;
			for (; d_end != opaques.size() && opaques[d_end].m_sort_key.Group() == group; ++d_end) {}
			for (; p_end != pop_dles.size() && pop_dles[p_end].m_sort_key.Group() == group; ++p_end) {}

			// Draw the group's drawlist elements, then its populations
			if (d_end != d)
				DrawNuggets(frame, m_cmd_list, pipe_state, opaques.subspan(d, d_end - d), s_cast<int>(d), ESubPass::Opaque);
			if (p_end != p)
				DrawPopulations(frame, pipe_state, pops, s_cast<int>(p), s_cast<int>(p_end), min_population_radius);

			d = d_end;
			p = p_end;
		}
	}

	// Draw the population draws [beg, end) of 'pops' into the opaque targets.
	void RenderForward::DrawPopulations(Frame& frame, PipeStateDesc const& pipe_state, PopulationSet const& pops, int beg, int end, float min_population_radius)
	{
		// Population draws select element constants from their own table, so swap it in, then restore the drawlist's table for later draws.
		auto pix_population = pix::EventScope<ID3D12GraphicsCommandList>(m_cmd_list.get(), 0xFF5AC8A0, "View3D::Populations");
		auto drawlist_elements = m_elements;
		m_elements = pops.m_elements;
		BindFrameResources(m_cmd_list);

		// Draw the accepted draws in runs, because the position of each run selects its element constants
		auto const dles = std::span<DrawListElement const>{ pops.m_dles };
		auto const draws = std::span<PopulationDrawRef const>{ pops.m_draws };
		auto accepted = [&](int i)
		{
			// Leave out populations that are smaller than the minimum radius
			auto const& ref = draws[i];
			return ref.m_lists->m_population->m_prototypes[ref.m_draw->m_prototype]->m_bound.w >= min_population_radius;
		};
		for (auto r = beg; r != end;)
		{
			// Skip rejected draws, then find the end of the run
			for (; r != end && !accepted(r); ++r) {}
			auto r_end = r;
			for (; r_end != end && accepted(r_end); ++r_end) {}
			if (r_end != r)
			{
				auto const count = s_cast<size_t>(r_end - r);
				DrawNuggets(frame, m_cmd_list, pipe_state, dles.subspan(r, count), r, ESubPass::Opaque, draws.subspan(r, count));
			}
			r = r_end;
		}

		// Restore the drawlist's element constants
		m_elements = drawlist_elements;
		BindFrameResources(m_cmd_list);
	}

	// Add the nuggets in the draw list to 'cmd_list' for rendering.
	void RenderForward::DrawNuggets(Frame& frame, GfxCmdList& cmd_list, PipeStateDesc const& default_pipe_state, std::span<DrawListElement const> drawlist, int first_index, ESubPass sub_pass, std::span<PopulationDrawRef const> populations)
	{
		assert(populations.empty() || populations.size() == drawlist.size());

		// Keep camera inversion and default projection composition outside the per-nugget loop.
		auto const camera = CameraTransforms(scn().m_cam);
		auto bound = BoundDescriptors{};
		auto pipe_state_bound = false;
		auto bound_pipe_state = PipeStateDesc{};
		auto frame_resources_bound = true;

		// Material passes can temporarily switch root signatures. Restore the render-step frame bindings before drawing the next nugget.
		auto bind_frame_resources = [&]
		{
			BindFrameResources(cmd_list);
			if (sub_pass == ESubPass::Alpha)
				BindAlphaResources(frame, cmd_list);

			bound.Reset();
			pipe_state_bound = false;
			frame_resources_bound = true;
		};

		// Draw each element in the draw list
		for (auto& dle : drawlist)
		{
			// The alpha collection must never collect background or overlay fragments.
			if (sub_pass == ESubPass::Alpha && dle.m_sort_key.Group() < ESortGroup::Alpha)
				continue;

			// Something not rendering?
			//  - Check the tint for the nugget isn't 0x00000000.
			// Tips:
			//  - To uniquely identify an instance in a shader for debugging, set the Instance Id (cb1.m_flags.w)
			//    Then in the shader, use: if (m_flags.w == 1234) ...
			auto const& nugget = *dle.m_nugget;
			auto const& instance = *dle.m_instance;
			auto desc = default_pipe_state;
			
			// If the instance is skinned, get the post-skinned vertex buffer for this instance's pose. Only skinned models look up a pose.
			auto const* vb_view = &nugget.m_model->m_vb_view;
			if (PosePtr pose = nugget.m_model->m_skin ? FindPose(instance) : PosePtr{}; pose)
			{
				vb_view = &rdr().SkinnedGeometry().VBufView(cmd_list, m_upload_buffer, *nugget.m_model, pose);

				// Compute skinning can bind a compute PSO while updating the vertex buffer. The cached graphics PSO state is no longer reliable.
				pipe_state_bound = false;
			}

			// Set the input geometry
			cmd_list.IASetPrimitiveTopology(nugget.m_topo);
			cmd_list.IASetVertexBuffers(0U, { vb_view, 1 });
			cmd_list.IASetIndexBuffer(&nugget.m_model->m_ib_view);

			// Compose the pipeline state, then let the material bind per-draw resources and constants.
			auto material_override = FindMaterial(instance);
			auto const& material = material_override != nullptr ? *material_override.get() : nugget.mat();
			auto const* pass = material.Pass(m_step_id);
			if (pass == nullptr)
				continue;

			auto const* population = populations.empty() ? nullptr : &populations[s_cast<size_t>(&dle - drawlist.data())];
			ComposePipeline(desc, nugget, instance, material, *pass, population != nullptr, m_mirrored);

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
				.m_shader = &m_shader,
				.m_default_tex = m_default_tex.get(),
				.m_default_sam = m_default_sam.get(),
				.m_bound = &bound,
			};
			pass->Bind(ctx);

			// Even a bounds-rejected draw can have changed the material's root bindings.
			if (ctx.m_root_signature_changed)
				frame_resources_bound = false;

			// Population draws read the instance records and visible lists.
			if (population != nullptr)
			{
				// The payload and skin slots need valid addresses even when unused, so they alias the instance records.
				auto const& lists = *population->m_lists;
				auto const& pop = *lists.m_population;
				auto skinned = static_cast<bool>(nugget.m_model->m_skin);
				auto draw = PopulationDraw{ .list = s_cast<uint32_t>(population->m_draw->m_list), .list_base = lists.m_list_base, .offsets = lists.m_offsets, .pad = 0 };
				cmd_list.SetGraphicsRootShaderResourceView(shaders::fwd::ERootParam::PopulationInstances, GpuAddress(pop.m_instances));
				cmd_list.SetGraphicsRootShaderResourceView(shaders::fwd::ERootParam::PopulationView, GpuAddress(lists.m_scratch));
				cmd_list.SetGraphicsRootShaderResourceView(shaders::fwd::ERootParam::PopulationPayload, pop.m_payload != nullptr ? GpuAddress(pop.m_payload) : GpuAddress(pop.m_instances));
				cmd_list.SetGraphicsRootShaderResourceView(shaders::fwd::ERootParam::PopulationSkin, skinned ? GpuAddress(nugget.m_model->m_skin.m_res) : GpuAddress(pop.m_instances));
				cmd_list.SetGraphicsRoot32BitConstants(shaders::fwd::ERootParam::PopulationDraw, draw);
			}

			// Draw the nugget.
			DrawNugget(cmd_list, nugget, desc, pipe_state_bound, bound_pipe_state, population);
		}
	}

	// Draw a single nugget
	void RenderForward::DrawNugget(GfxCmdList& cmd_list, Nugget const& nugget, PipeStateDesc& desc, bool& pipe_state_bound, PipeStateDesc& bound_pipe_state, PopulationDrawRef const* population)
	{
		auto set_pipe_state = [&]
		{
			// Consecutive draws often share a state. Comparing descriptions is cheaper than a cache lookup.
			if (pipe_state_bound && desc == bound_pipe_state)
				return;

			cmd_list.SetPipelineState(rdr().PipeStates().Get(desc));
			pipe_state_bound = true;
			bound_pipe_state = desc;
		};

		// Ordinary draws draw one instance. Population draws take their instance counts from the classification's argument records.
		auto draw_indexed = [&]
		{
			// Keep indexed draws at base-vertex zero because optional model vertex streams use SV_VertexID to read buffers parallel to the primary vertex buffer.
			if (population != nullptr)
				cmd_list.ExecuteIndirect(rdr().PopulationCulling().DrawSignature(true), 1, population->m_lists->m_scratch, s_cast<UINT64>(population->m_draw->m_args_indexed));
			else
				cmd_list.DrawIndexedInstanced(s_cast<size_t>(nugget.m_irange.size()), 1U, s_cast<size_t>(nugget.m_irange.m_beg), 0, 0U);
		};
		auto draw_vertices = [&]
		{
			// Draw the nugget's vertex range without indices.
			if (population != nullptr)
				cmd_list.ExecuteIndirect(rdr().PopulationCulling().DrawSignature(false), 1, population->m_lists->m_scratch, s_cast<UINT64>(population->m_draw->m_args_vertex));
			else
				cmd_list.DrawInstanced(s_cast<size_t>(nugget.m_vrange.size()), 1U, s_cast<size_t>(nugget.m_vrange.m_beg), 0U);
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
				draw_vertices();
			else
				draw_indexed();

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
			draw_indexed();

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
			draw_vertices();
		}
	}
}
