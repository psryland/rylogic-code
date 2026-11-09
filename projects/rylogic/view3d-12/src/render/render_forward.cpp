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

	namespace
	{
		// A 2D affine map: (x, y) -> (a*x + b*y + tx, c*x + d*y + ty)
		struct Affine2
		{
			float a, b, tx;
			float c, d, ty;
		};

		// The map that applies 'rhs' and then 'lhs'
		Affine2 operator * (Affine2 const& lhs, Affine2 const& rhs)
		{
			return Affine2{
				.a = lhs.a * rhs.a + lhs.b * rhs.c,
				.b = lhs.a * rhs.b + lhs.b * rhs.d,
				.tx = lhs.a * rhs.tx + lhs.b * rhs.ty + lhs.tx,
				.c = lhs.c * rhs.a + lhs.d * rhs.c,
				.d = lhs.c * rhs.b + lhs.d * rhs.d,
				.ty = lhs.c * rhs.tx + lhs.d * rhs.ty + lhs.ty,
			};
		}

		// The inverse of an invertible affine map
		Affine2 InvertAffine(Affine2 const& m)
		{
			// Invert the linear part, then undo the translation with it
			auto det = m.a * m.d - m.b * m.c;
			auto a = m.d / det;
			auto b = -m.b / det;
			auto c = -m.c / det;
			auto d = m.a / det;
			return Affine2{ .a = a, .b = b, .tx = -(a * m.tx + b * m.ty), .c = c, .d = d, .ty = -(c * m.tx + d * m.ty) };
		}

		// The clip space transform that applies 'm' to normalised device (x, y). Clip x and y are scaled by w, so the translation is scaled by w too.
		// Depth and w are unchanged.
		m4x4 ClipTransform(Affine2 const& m)
		{
			return m4x4{ v4{ m.a, m.c, 0, 0 }, v4{ m.b, m.d, 0, 0 }, v4{ 0, 0, 1, 0 }, v4{ m.tx, m.ty, 0, 1 } };
		}
	}

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
	// Sources just beyond the viewport are rendered at low resolution into guard bands above, left of, and right of the viewport, so that their
	// reflections are found too. The bands are packed side by side into one atlas, and all of them are drawn in one pass using view instancing.
	struct RenderForward::ReflectionPlane
	{
		// Root parameters of the projection passes. Each view gets its own table, because consecutive heap entries can wrap around the ring.
		enum class EParam { Constants, Colour, Depth, Key, Raw, Final, Coverage };

		// The guard bands' resolution relative to the viewport, the top band's resolution relative to the other bands, and the most the side bands
		// can reach beyond the viewport in normalised device units. The top band's lower resolution lets it reach further above the viewport.
		static constexpr float GuardScale = 0.25f;
		static constexpr float GuardTopScale = 0.5f;
		static constexpr float GuardSideMax = 0.5f;

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

		// One guard band
		struct GuardBand
		{
			// Places the guarded area's normalised device coordinates (see 'm_guard_remap') in the band's area of the atlas
			D3D12_VIEWPORT m_viewport;

			// Maps atlas pixel positions in the band to the main view's normalised device coordinates
			Affine2 m_src;

			// The band's area in the atlas, in pixels
			D3D12_RECT m_rect;

			// The size of the square that each band pixel's reflection covers, in viewport pixels
			float m_splat;

			// True when reflections this frame can need sources from the band
			bool m_needed;
		};

		// The passes' shared root signature and their pipeline states
		D3DPtr<ID3D12RootSignature> m_sig;
		D3DPtr<ID3D12PipelineState> m_project;
		D3DPtr<ID3D12PipelineState> m_write;
		D3DPtr<ID3D12PipelineState> m_fill;

		// The winning depth key per pixel, the raw projected reflection, and the gap-filled reflection and its coverage read by the shaders.
		// All are the size of the back buffer.
		Texture2DPtr m_key;
		Texture2DPtr m_raw;
		Texture2DPtr m_final;
		Texture2DPtr m_coverage;
		iv2 m_size;

		// The guard band atlas colour and depth, and the atlas size. They use the main target's formats and sampling, so the scene's pipeline states draw into them.
		Texture2DPtr m_band_colour;
		Texture2DPtr m_band_depth;
		iv2 m_band_size;

		// This frame's bands: top, left, right. 'm_guard' is how far they reach beyond the viewport in normalised device units, (sides, top).
		// 'm_guard_remap' maps the main view's clip space so that the guarded area, x in [-1-gx, 1+gx] and y in [-1, 1+gy], fills the unit square.
		std::array<GuardBand, 3> m_bands;
		v2 m_guard;
		m4x4 m_guard_remap;
		bool m_active;

		// Per opaque drawlist element, non-zero when the element can appear in a needed band this frame
		pr::vector<uint8_t> m_draw_elements;

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
			, m_coverage()
			, m_size()
			, m_band_colour()
			, m_band_depth()
			, m_band_size()
			, m_bands()
			, m_guard()
			, m_guard_remap(m4x4::Identity())
			, m_active()
			, m_draw_elements()
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
				.UAV(EUAVReg::u3, 1, D3D12_SHADER_VISIBILITY_ALL, D3D12_DESCRIPTOR_RANGE_FLAG_DESCRIPTORS_VOLATILE)
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
				if (material.Component<materials::DetailNormals>() != nullptr)
					materials::ApplyForwardPixelFamily(desc, shader_code::forward_detail_family);

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

		// Classify the populations before any draws, so the classification can overlap earlier work on the GPU. The classification includes the
		// reflection guard bands, which are drawn from the same draws.
		PrepareGuardBands();
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
			DrawOpaques(frame, pipe_state, std::span{ *drawlist }.subspan(0, s_cast<size_t>(alpha_start)));
			if (!kbuf)
				DrawNuggets(frame, m_cmd_list, pipe_state, std::span{ *drawlist }.subspan(s_cast<size_t>(alpha_start)), alpha_start, ESubPass::Opaque);

			DrawGuardBands(frame, std::span{ *drawlist }.subspan(0, s_cast<size_t>(alpha_start)));
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

	// Lay out this frame's reflection guard bands, and size their atlas
	void RenderForward::PrepareGuardBands()
	{
		// The bands are drawn only when the reflection is projected, and in one pass that needs view instancing
		auto& rp = *m_reflection_plane;
		rp.m_active = false;
		rp.m_guard = v2::Zero();
		rp.m_guard_remap = m4x4::Identity();
		if (!ReflectionPlaneNeeded() || rdr().Features().Options3.ViewInstancingTier == D3D12_VIEW_INSTANCING_TIER_NOT_SUPPORTED)
			return;

		// Find how far the side bands must reach. Reflections are seen only along view directions that point down towards the plane. A source seen in
		// the reflection along such a direction lies along the mirror image of that direction, or below it when the source is nearer, which only
		// moves it towards the centre of the view. So the side bands must cover the projection of the mirrored downward part of the view frustum.
		// The frustum's directions form a convex polygon, so clipping it and finding its extent needs only its corners.
		using Poly = pr::vector<v4, 8, false>;
		auto clip = [](Poly const& poly, auto&& dist)
		{
			// Keep the part of the polygon where 'dist' is not negative. 'dist' must be linear, so edges cross zero where it interpolates to zero.
			Poly out;
			for (size_t i = 0, iend = poly.size(); i != iend; ++i)
			{
				// Keep inside vertices, and add the crossing point of each edge that crosses the boundary
				auto const& a = poly[i];
				auto const& b = poly[(i + 1) % iend];
				auto const da = dist(a);
				auto const db = dist(b);
				if (da >= 0)
					out.push_back(a);
				if ((da >= 0) != (db >= 0))
					out.push_back(a + (b - a) * (da / (da - db)));
			}
			return out;
		};
		auto need = std::array<float, 3>{}; // How far beyond the top, left and right edges sources are needed, in normalised device units
		{
			// Build the view directions through the viewport corners from two depths in front of the camera, which works for both projections.
			// Reversed depth puts the near plane at 1.
			auto const& cam = scn().m_cam;
			auto const s2w = cam.CameraToWorld() * Invert(cam.CameraToScreen());
			auto const w2s = cam.CameraToScreen() * InvertOrthonormal(cam.CameraToWorld());
			auto const norm = v4(scn().m_reflection_plane.xyz, 0.0f);
			Poly dirs;
			for (auto const& corner : { v2(-1, -1), v2(+1, -1), v2(+1, +1), v2(-1, +1) })
			{
				auto const p0 = s2w * v4(corner.x, corner.y, 1.0f, 1.0f);
				auto const p1 = s2w * v4(corner.x, corner.y, 0.5f, 1.0f);
				dirs.push_back(p1 / p1.w - p0 / p0.w);
			}

			// Keep the downward directions, mirror them in the plane, and project them as points at infinity. Clipping to |x| <= w (1 + max) both
			// removes directions behind the camera and caps the side extent. The top extent is capped similarly, beyond any reach of the top band.
			dirs = clip(dirs, [&](v4 const& d) { return -Dot(d, norm); });
			Poly proj;
			for (auto const& d : dirs)
				proj.push_back(w2s * (d - 2.0f * Dot(d, norm) * norm));

			proj = clip(proj, [](v4 const& c) { return c.w * (1.0f + ReflectionPlane::GuardSideMax) - c.x; });
			proj = clip(proj, [](v4 const& c) { return c.w * (1.0f + ReflectionPlane::GuardSideMax) + c.x; });
			proj = clip(proj, [](v4 const& c) { return c.w * 10.0f - c.y; });
			for (auto const& c : proj)
			{
				// Points on the w = 0 boundary only appear when x and y are zero too, so they add nothing
				if (c.w <= 0.0f)
					continue;

				need[0] = std::max(need[0], c.y / c.w - 1.0f);
				need[1] = std::max(need[1], -c.x / c.w - 1.0f);
				need[2] = std::max(need[2], +c.x / c.w - 1.0f);
			}
		}
		auto const gx = std::max(need[1], need[2]);

		// Size the bands. The scaled viewport is (sw, sh). The atlas size depends only on the viewport, so it is not recreated as the camera turns.
		// The atlas is 'sh' rows high. The top band is on the left, spanning the full width of the guarded area at scale 'ts', with its content
		// against its lower edge. The left and right bands follow, each up to 'bw_max' columns wide. Whole pixel sizes keep each band's edges on
		// pixel boundaries, so the guard extents are recomputed from them.
		auto const& vp = scn().m_viewport;
		auto const ts = ReflectionPlane::GuardTopScale;
		auto const sw = std::max(s_cast<int>(std::round(vp.Width * ReflectionPlane::GuardScale)), 1);
		auto const sh = std::max(s_cast<int>(std::round(vp.Height * ReflectionPlane::GuardScale)), 1);
		auto const bw_max = std::max(s_cast<int>(std::ceil(sw * ReflectionPlane::GuardSideMax * 0.5f)), 1);
		auto const wt_max = s_cast<int>(std::ceil((sw + 2 * bw_max) * ts));
		auto const size = iv2(wt_max + 2 * bw_max, sh);

		// Draw only the bands, and the top band rows, that the reflections can need. The top band's unused rows are dropped from its top, and its
		// width covers the side bands only when they are drawn. The guard extents shrink to match, which also tightens the population cull.
		auto const sides = need[1] > 0.0f || need[2] > 0.0f;
		auto const bw = sides ? std::clamp(s_cast<int>(std::ceil(sw * gx * 0.5f)), 1, bw_max) : 0;
		auto const wt = s_cast<int>(std::ceil((sw + 2 * bw) * ts));
		auto const top_rows = std::clamp(s_cast<int>(std::ceil(need[0] * sh * ts * 0.5f)), 0, sh);
		auto const needed = std::array<bool, 3>{ top_rows != 0, sides && need[1] > 0.0f, sides && need[2] > 0.0f };
		if (!needed[0] && !needed[1] && !needed[2])
			return;

		// The guarded area, and the map from it to the main view's normalised device coordinates: x = X (1 + gx), y = (Y (2 + gy) + gy) / 2.
		// Every band is drawn through the same projection, which maps the guarded area to the unit square. Each band's viewport then places it.
		auto const guard_x = 2.0f * bw / sw;
		auto const guard_y = needed[0] ? 2.0f * top_rows / (sh * ts) : 0.0f;
		auto const g2n = Affine2{ .a = 1.0f + guard_x, .b = 0, .tx = 0, .c = 0, .d = 1.0f + 0.5f * guard_y, .ty = 0.5f * guard_y };
		rp.m_guard = v2(guard_x, guard_y);
		rp.m_guard_remap = ClipTransform(InvertAffine(g2n));

		// The map from the main view's normalised device coordinates to scaled viewport pixels (u, v)
		auto const fsw = s_cast<float>(sw);
		auto const fsh = s_cast<float>(sh);
		auto const n2u = Affine2{ .a = 0.5f * fsw, .b = 0, .tx = 0.5f * fsw, .c = 0, .d = -0.5f * fsh, .ty = 0.5f * fsh };

		// Place each band in the atlas. The top band is scaled by 'ts', so its left edge (u = -bw) is at x = 0 and the viewport's top edge (v = 0)
		// is at the bottom row. The side bands are unscaled, so the left band's columns u in [-bw, 0) and the right band's columns u in [sw, sw + bw)
		// land at the start of their areas. Each map must be a scale and a translation, so that a viewport can express it.
		auto const band_maps = std::array<Affine2, 3>{
			Affine2{ .a = ts, .b = 0, .tx = ts * bw, .c = 0, .d = ts, .ty = fsh },
			Affine2{ .a = 1, .b = 0, .tx = s_cast<float>(wt_max + bw), .c = 0, .d = 1, .ty = 0 },
			Affine2{ .a = 1, .b = 0, .tx = s_cast<float>(wt_max + bw_max - sw), .c = 0, .d = 1, .ty = 0 },
		};
		auto const band_rects = std::array<D3D12_RECT, 3>{
			D3D12_RECT{ 0, sh - top_rows, wt, sh },
			D3D12_RECT{ wt_max, 0, wt_max + bw, sh },
			D3D12_RECT{ wt_max + bw_max, 0, wt_max + bw_max + bw, sh },
		};
		auto const band_splats = std::array<float, 3>{ 1.0f / (ReflectionPlane::GuardScale * ts), 1.0f / ReflectionPlane::GuardScale, 1.0f / ReflectionPlane::GuardScale };
		for (int i = 0; i != 3; ++i)
		{
			// The viewport maps the guarded area's X in [-1, 1] to [left, left + width] and Y in [-1, 1] to [top + height, top], so it follows from
			// the map from guarded area to atlas pixels. The projection passes read the band from its top-left corner, so its source map starts
			// there, at the centre of the first pixel. Each band pixel's reflection covers as many viewport pixels as the pixel spans.
			auto& band = rp.m_bands[i];
			auto const n2p = band_maps[i] * n2u;
			auto const g2p = n2p * g2n;
			auto const origin = Affine2{ .a = 1, .b = 0, .tx = s_cast<float>(band_rects[i].left), .c = 0, .d = 1, .ty = s_cast<float>(band_rects[i].top) };
			band.m_viewport = D3D12_VIEWPORT{ .TopLeftX = g2p.tx - g2p.a, .TopLeftY = g2p.ty + g2p.d, .Width = 2.0f * g2p.a, .Height = -2.0f * g2p.d, .MinDepth = 0.0f, .MaxDepth = 1.0f };
			band.m_src = InvertAffine(n2p) * origin;
			band.m_rect = band_rects[i];
			band.m_splat = std::round(band_splats[i]);
			assert(band.m_splat >= 1.0f && band.m_splat <= 9.0f && "The projection keys record splat sizes up to 'ReflectionMaxSplat' in reflection_plane.hlsl");
			band.m_needed = needed[i];
		}

		// Size the atlas. It matches the main target's formats and sampling, so the scene's pipeline states can draw into it.
		auto const ms = wnd().MultiSampling();
		auto const resize = rp.m_band_colour == nullptr || Any(size != rp.m_band_size) || rp.m_band_colour->m_res->GetDesc().SampleDesc.Count != ms.Count;
		if (resize)
		{
			// Colour is written and read through sRGB views, like the main target
			ResourceFactory factory(rdr());
			auto const rt_format = wnd().m_rt_props.Format;
			auto const ds_format = wnd().m_ds_props.Format;
			auto col_desc = ResDesc::Tex2D(Image{ size.x, size.y, nullptr, rt_format }, 1U, EUsage::RenderTarget)
				.multisamp(ms)
				.clear(wnd().m_rt_props)
				.def_state(D3D12_RESOURCE_STATE_RENDER_TARGET);
			auto dep_desc = ResDesc::Tex2D(Image{ size.x, size.y, nullptr, DepthResourceFormat(ds_format) }, 1U, EUsage::DepthStencil)
				.multisamp(ms)
				.clear(wnd().m_ds_props)
				.def_state(D3D12_RESOURCE_STATE_DEPTH_WRITE);
			rp.m_band_colour = factory.CreateTexture2D(TextureDesc(AutoId, col_desc).rtv_format(ToSRGB(rt_format)).srv_format(ToSRGB(rt_format)).name("ReflectionGuardColour"));
			rp.m_band_depth = factory.CreateTexture2D(TextureDesc(AutoId, dep_desc).dsv_format(ds_format).srv_format(DepthSrvFormat(ds_format)).name("ReflectionGuardDepth"));
			rp.m_band_size = size;
		}
		rp.m_active = true;
	}

	// Draw the opaque scene, without the sky, into the reflection guard bands
	void RenderForward::DrawGuardBands(Frame& frame, std::span<DrawListElement const> opaques)
	{
		// Clear the atlas. Areas the scene does not cover keep depth 0, which the projection passes ignore.
		auto& rp = *m_reflection_plane;
		if (!rp.m_active)
			return;

		auto pix_bands = pix::EventScope<ID3D12GraphicsCommandList>(m_cmd_list.get(), 0xFF5EC8D5, "View3D::ReflectionGuardBands");
		auto const rtv = rp.m_band_colour->m_rtv.m_cpu;
		auto const dsv = rp.m_band_depth->m_dsv.m_cpu;
		m_cmd_list.ClearRenderTargetView(rtv, frame.bb_main().rt_clear());
		m_cmd_list.ClearDepthStencilView(dsv, D3D12_CLEAR_FLAG_DEPTH, frame.bb_main().ds_depth(), 0);
		m_cmd_list.OMSetRenderTargets({ &rtv, 1 }, FALSE, &dsv);

		// The needed bands' areas in the main view's normalised device coordinates. The top band includes the corners above the side bands.
		struct Area { v2 m_lower; v2 m_upper; };
		auto const gx = rp.m_guard.x;
		auto const gy = rp.m_guard.y;
		auto const band_areas = std::array<Area, 3>{
			Area{ v2(-1.0f - gx, 1.0f), v2(1.0f + gx, 1.0f + gy) },
			Area{ v2(-1.0f - gx, -1.0f), v2(-1.0f, 1.0f) },
			Area{ v2(1.0f, -1.0f), v2(1.0f + gx, 1.0f) },
		};

		// Select the drawlist elements that can appear in a needed band. Elements whose screen bounds cannot be known are always drawn.
		// Skinned bounds can need a CPU pose evaluation, and the nuggets of one model share a pose, so each model/pose pair is evaluated once.
		struct PosedBBox { Model const* m_model; Pose const* m_pose; std::optional<BBox> m_bbox; };
		pr::vector<PosedBBox, 8> posed_bboxes;
		auto const w2s = scn().m_cam.CameraToScreen() * InvertOrthonormal(scn().m_cam.CameraToWorld());
		auto in_bands = [&](DrawListElement const& dle)
		{
			// Find the element's world bounds, posed if it is skinned
			auto const& nugget = *dle.m_nugget;
			auto const& instance = *dle.m_instance;
			auto const& mbox = nugget.m_model->m_bbox;
			auto const& i2w = GetO2W(instance);
			if (!mbox.valid() || !IsAffine(i2w))
				return true;

			auto bbox = i2w * mbox;
			if (auto pose = FindPose(instance); pose != nullptr)
			{
				// Posed models without a skin, or without pose bounds, can move outside their rest bounds
				if (!nugget.m_model->m_skin)
					return true;

				auto iter = std::ranges::find_if(posed_bboxes, [&](PosedBBox const& p) { return p.m_model == nugget.m_model && p.m_pose == pose.get(); });
				if (iter == posed_bboxes.end())
				{
					posed_bboxes.push_back(PosedBBox{ .m_model = nugget.m_model, .m_pose = pose.get(), .m_bbox = rdr().SkinnedModelBBox(*nugget.m_model, *pose.get()) });
					iter = posed_bboxes.end() - 1;
				}
				if (!iter->m_bbox)
					return true;

				bbox = i2w * *iter->m_bbox;
			}

			// Bound the element on screen. Bounds that reach behind the camera cannot be bounded this way.
			auto screen = Area{ v2(+std::numeric_limits<float>::infinity()), v2(-std::numeric_limits<float>::infinity()) };
			for (int c = 0; c != 8; ++c)
			{
				auto const corner = w2s * Corner(bbox, c);
				if (corner.w <= math::tiny<float>)
					return true;

				auto const ndc = corner.xy / corner.w;
				screen.m_lower = Min(screen.m_lower, ndc);
				screen.m_upper = Max(screen.m_upper, ndc);
			}

			// Draw the element if it overlaps any needed band
			for (int i = 0; i != 3; ++i)
			{
				auto const& area = band_areas[i];
				if (rp.m_bands[i].m_needed && All(screen.m_lower <= area.m_upper) && All(screen.m_upper >= area.m_lower))
					return true;
			}
			return false;
		};
		rp.m_draw_elements.resize(opaques.size());
		for (size_t i = 0; i != opaques.size(); ++i)
			rp.m_draw_elements[i] = in_bands(opaques[i]);

		// Draw the scene once into all the needed bands. The frame constants map the guarded area to the unit square, and view instancing
		// repeats each draw through each band's viewport, clipped to the band's area of the atlas.
		pr::vector<D3D12_VIEWPORT, 3, false> viewports;
		pr::vector<D3D12_RECT, 3, false> scissors;
		for (auto const& band : rp.m_bands)
		{
			// Bands that are not needed are not drawn
			if (!band.m_needed)
				continue;

			viewports.push_back(band.m_viewport);
			scissors.push_back(band.m_rect);
		}
		auto band_pipe_state = m_default_pipe_state;
		band_pipe_state.ViewCount(isize(viewports));
		m_cmd_list.get()->RSSetViewports(s_cast<UINT>(viewports.size()), viewports.data());
		m_cmd_list.RSSetScissorRects({ scissors.data(), scissors.size() });
		m_shader.m_clip_remap = &rp.m_guard_remap;
		BindFrameResources(m_cmd_list);
		auto const filter = GuardBandFilter{ .m_elements = rp.m_draw_elements, .m_min_population_radius = scn().m_reflection_min_population_radius };
		DrawOpaques(frame, band_pipe_state, opaques, &filter);

		// Restore the main view for any later draws
		auto const& vp = scn().m_viewport;
		m_shader.m_clip_remap = nullptr;
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
			auto cov_desc = ResDesc::Tex2D(Image{ bb_size.x, bb_size.y, nullptr, DXGI_FORMAT_R8_UNORM }, 1U, EUsage::UnorderedAccess).def_state(D3D12_RESOURCE_STATE_ALL_SHADER_RESOURCE);
			rp.m_key = factory.CreateTexture2D(TextureDesc(AutoId, key_desc).name("ReflectionPlaneKey"));
			rp.m_raw = factory.CreateTexture2D(TextureDesc(AutoId, col_desc).name("ReflectionPlaneRaw"));
			rp.m_final = factory.CreateTexture2D(TextureDesc(AutoId, col_desc).name("ReflectionPlane"));
			rp.m_coverage = factory.CreateTexture2D(TextureDesc(AutoId, cov_desc).name("ReflectionPlaneCoverage"));
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
			bb.Transition(rp.m_band_colour->m_res.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
			bb.Transition(rp.m_band_depth->m_res.get(), D3D12_RESOURCE_STATE_ALL_SHADER_RESOURCE);
		}
		bb.Transition(rp.m_key->m_res.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		bb.Transition(rp.m_raw->m_res.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		bb.Transition(rp.m_final->m_res.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		bb.Transition(rp.m_coverage->m_res.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		bb.Commit();

		UINT const cleared[4] = { 0, 0, 0, 0 };
		m_alp_list.ClearUnorderedAccessViewUint(wnd().m_heap_view.Add(rp.m_key->m_uav), rp.m_key->m_uav.m_cpu, rp.m_key->m_res.get(), cleared);
		bb.UAV(rp.m_key->m_res.get());
		bb.Commit();

		// Describe the sources. The main view is read pixel for pixel. Each guard band is read through its own map, and its pixels each cover
		// a square of target pixels as wide as one band pixel is in the viewport. The bands do not draw the sky, so their empty pixels are ignored.
		// The colour is read through an sRGB view, which converts it to linear colour. Creating a view does not modify the frame's back buffer,
		// so it is safe to drop the const.
		struct Source
		{
			v4 src0, src1, src2;
			D3D12_GPU_DESCRIPTOR_HANDLE colour, depth;
		};
		auto colour_srv = D3D12_SHADER_RESOURCE_VIEW_DESC{
			.Format = ToSRGB(wnd().m_rt_props.Format),
			.ViewDimension = D3D12_SRV_DIMENSION_TEXTURE2DMS,
			.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING,
		};
		auto sources = pr::vector<Source, 4, false>{};
		sources.push_back(Source{
			.src0 = v4(2.0f / vp.Width, 0.0f, -1.0f, vp.Width),
			.src1 = v4(0.0f, -2.0f / vp.Height, 1.0f, vp.Height),
			.src2 = v4(vp.TopLeftX, vp.TopLeftY, 1.0f, 0.0f),
			.colour = wnd().m_heap_view.Add(const_cast<ID3D12Resource*>(colour), colour_srv),
			.depth = wnd().m_heap_view.Add(bb_main.m_depth_srv),
		});
		if (rp.m_active)
		{
			for (auto const& band : rp.m_bands)
			{
				// Bands that were not drawn hold no sources
				if (!band.m_needed)
					continue;

				auto const& m = band.m_src;
				auto const w = s_cast<float>(band.m_rect.right - band.m_rect.left);
				auto const h = s_cast<float>(band.m_rect.bottom - band.m_rect.top);
				sources.push_back(Source{
					.src0 = v4(m.a, m.b, m.tx, w),
					.src1 = v4(m.c, m.d, m.ty, h),
					.src2 = v4(s_cast<float>(band.m_rect.left), s_cast<float>(band.m_rect.top), band.m_splat, 1.0f),
					.colour = wnd().m_heap_view.Add(rp.m_band_colour->m_srv),
					.depth = wnd().m_heap_view.Add(rp.m_band_depth->m_srv),
				});
			}
		}

		// Bind the passes' constants and outputs
		auto const w2s = cam.CameraToScreen() * InvertOrthonormal(cam.CameraToWorld());
		auto const s2w = cam.CameraToWorld() * Invert(cam.CameraToScreen());
		auto const viewport = v4(vp.TopLeftX, vp.TopLeftY, vp.Width, vp.Height);
		auto cb = ReflectionPlane::Constants{
			.s2w = s2w,
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
		m_alp_list.SetComputeRootDescriptorTable(EParam::Coverage, wnd().m_heap_view.Add(rp.m_coverage->m_uav));

		// Run each pass over every source, one thread per source pixel in 8x8 groups. Each pass reads what the previous one wrote.
		for (auto* pso : { rp.m_project.get(), rp.m_write.get() })
		{
			// Dispatch one pass per source, then make its writes visible to the next pass
			m_alp_list.SetPipelineState(pso);
			for (auto const& src : sources)
			{
				// Bind the source and its map
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
		bb.UAV(rp.m_coverage->m_res.get());
		bb.Commit();

		// The alpha draws read the final reflection, and the colour targets return to their normal states
		bb.Transition(rp.m_final->m_res.get(), D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE);
		bb.Transition(rp.m_coverage->m_res.get(), D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE);
		bb.Transition(colour, D3D12_RESOURCE_STATE_RENDER_TARGET);
		if (rp.m_active)
		{
			bb.Transition(rp.m_band_colour->m_res.get(), D3D12_RESOURCE_STATE_RENDER_TARGET);
			bb.Transition(rp.m_band_depth->m_res.get(), D3D12_RESOURCE_STATE_DEPTH_WRITE);
		}
		bb.Commit();

		rp.m_constants = shaders::fwd::ReflectionPlane{
			.plane = plane,
			.viewport = viewport,
			.guard = v4(rp.m_guard.x, rp.m_guard.y, 0.0f, 0.0f),
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

		// Add the projected reflection of the scene's reflection plane and its coverage. They are only read while the reflection constants are set
		// (see ProjectReflectionPlane).
		if (m_shader.m_reflection_plane != nullptr)
		{
			cmd_list.SetGraphicsRootDescriptorTable(shaders::fwd::ERootParam::ReflectionPlane, wnd().m_heap_view.Add(m_reflection_plane->m_final->m_srv));
			cmd_list.SetGraphicsRootDescriptorTable(shaders::fwd::ERootParam::ReflectionPlaneCoverage, wnd().m_heap_view.Add(m_reflection_plane->m_coverage->m_srv));
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

	// Classify the scene's populations for the camera and build the population draws.
	void RenderForward::CullPopulations()
	{
		// Population draws reference the classification results, so rebuild both each frame.
		m_population_lists.resize(0);
		m_population_dles.resize(0);
		m_population_draws.resize(0);
		auto populations = scn().Populations();
		if (populations.empty())
			return;

		// Record the classification on the opaque command list, which also records the draws that consume it.
		auto pix_cull = pix::EventScope<ID3D12GraphicsCommandList>(m_cmd_list.get(), 0xFF5AC8A0, "View3D::PopulationCull");
		auto view = PopulationCameraView(reinterpret_cast<uint64_t>(this), scn().m_cam, scn().m_viewport.Height);
		if (auto const& rp = *m_reflection_plane; rp.m_active)
		{
			// Widen the frustum to include the reflection guard bands, without changing the projected sizes that select LODs
			view.m_w2s = rp.m_guard_remap * view.m_w2s;
		}
		m_population_lists.reserve(populations.size());
		for (auto pop : populations)
		{
			m_population_lists.push_back(rdr().PopulationCulling().Cull(m_cmd_list, m_upload_buffer, m_gsync, *pop.get(), view, m_step_id));
		}

		// Each (LOD, nugget) draw takes its element constants from the LOD's proxy instance, and its sort group from its population.
		// 'm_population_lists' no longer changes, so the references are stable.
		for (auto const& lists : m_population_lists)
		{
			for (auto const& draw : lists.m_draws)
			{
				auto sort_key = draw.m_nugget->m_sort_key;
				sort_key.Group(lists.m_population->m_sort_group);
				m_population_dles.push_back(DrawListElement{ .m_sort_key = sort_key, .m_nugget = draw.m_nugget, .m_instance = &draw.m_proxy->m_base });
				m_population_draws.push_back(PopulationDrawRef{ .m_lists = &lists, .m_draw = &draw });
			}
		}

		// Order the draws by sort key so that they can be merged with the drawlist by group. Within a group, draws that share a shader and texture
		// become adjacent, so they reuse bound descriptors and pipeline state. Draws with equal keys keep their population order.
		{
			std::vector<int> order(m_population_dles.size());
			std::iota(order.begin(), order.end(), 0);
			std::ranges::stable_sort(order, {}, [&](int i) { return m_population_dles[i].m_sort_key.m_value; });

			auto dles = std::move(m_population_dles);
			auto draws = std::move(m_population_draws);
			m_population_dles.resize(0);
			m_population_draws.resize(0);
			for (auto i : order)
			{
				m_population_dles.push_back(dles[i]);
				m_population_draws.push_back(draws[i]);
			}
		}

		// Upload the element constants of every population draw once. Each population draw selects its entry by position.
		m_population_elements = !m_population_dles.empty() ? shaders::Forward::UploadElements(m_upload_buffer, scn(), CameraTransforms(scn().m_cam), m_population_dles) : D3D12_GPU_VIRTUAL_ADDRESS{};
	}

	// Draw the opaque drawlist elements and the classified populations, merged in sort group order
	void RenderForward::DrawOpaques(Frame& frame, PipeStateDesc const& pipe_state, std::span<DrawListElement const> opaques, GuardBandFilter const* filter)
	{
		assert(filter == nullptr || filter->m_elements.size() == opaques.size());

		// Draw one sort group at a time: its drawlist elements, then its population draws. Both sequences are sorted by group.
		auto const pops = std::span<DrawListElement const>{ m_population_dles };
		for (size_t d = 0, p = 0; d != opaques.size() || p != pops.size();)
		{
			// Find the lowest group that has draws remaining
			auto group = ESortGroup::Max;
			if (d != opaques.size())
				group = std::min(group, opaques[d].m_sort_key.Group());
			if (p != pops.size())
				group = std::min(group, pops[p].m_sort_key.Group());

			// Find the end of the group in both sequences
			auto d_end = d;
			auto p_end = p;
			for (; d_end != opaques.size() && opaques[d_end].m_sort_key.Group() == group; ++d_end) {}
			for (; p_end != pops.size() && pops[p_end].m_sort_key.Group() == group; ++p_end) {}

			// Draw the group's drawlist elements. A filter draws the accepted elements in runs, because the position of each
			// run in the drawlist selects its element constants.
			auto const draw = filter == nullptr || group != ESortGroup::Skybox;
			if (draw && filter == nullptr && d_end != d)
			{
				DrawNuggets(frame, m_cmd_list, pipe_state, opaques.subspan(d, d_end - d), s_cast<int>(d), ESubPass::Opaque);
			}
			else if (draw && filter != nullptr)
			{
				// Draw each run of consecutive accepted elements
				for (auto r = d; r != d_end;)
				{
					// Skip rejected elements, then find the end of the run
					for (; r != d_end && filter->m_elements[r] == 0; ++r) {}
					auto r_end = r;
					for (; r_end != d_end && filter->m_elements[r_end] != 0; ++r_end) {}
					if (r_end != r)
						DrawNuggets(frame, m_cmd_list, pipe_state, opaques.subspan(r, r_end - r), s_cast<int>(r), ESubPass::Opaque);

					r = r_end;
				}
			}

			// Draw the group's populations
			if (draw && p_end != p)
				DrawPopulations(frame, pipe_state, s_cast<int>(p), s_cast<int>(p_end), filter);

			d = d_end;
			p = p_end;
		}
	}

	// Draw the population draws [beg, end) into the opaque targets.
	void RenderForward::DrawPopulations(Frame& frame, PipeStateDesc const& pipe_state, int beg, int end, GuardBandFilter const* filter)
	{
		// Population draws select element constants from their own table, so swap it in, then restore the drawlist's table for later draws.
		auto pix_population = pix::EventScope<ID3D12GraphicsCommandList>(m_cmd_list.get(), 0xFF5AC8A0, "View3D::Populations");
		auto drawlist_elements = m_elements;
		m_elements = m_population_elements;
		BindFrameResources(m_cmd_list);

		// Draw the accepted draws in runs, because the position of each run selects its element constants. Without a filter, all draws are one run.
		auto const dles = std::span<DrawListElement const>{ m_population_dles };
		auto const draws = std::span<PopulationDrawRef const>{ m_population_draws };
		auto accepted = [&](int i)
		{
			// Leave out populations that are too small to matter in the filtered view
			if (filter == nullptr)
				return true;

			auto const& ref = draws[i];
			return ref.m_lists->m_population->m_prototypes[ref.m_draw->m_prototype]->m_bound.w >= filter->m_min_population_radius;
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
		auto pipe_state_hash = 0;
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
				.m_bound = &bound,
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

			// Even a bounds-rejected draw can have changed the material's root bindings.
			if (ctx.m_root_signature_changed)
				frame_resources_bound = false;

			// Population draws place vertices with the population vertex shaders, and read the instance records and visible lists.
			auto const* population = populations.empty() ? nullptr : &populations[s_cast<size_t>(&dle - drawlist.data())];
			if (population != nullptr)
			{
				// Procedural vertex shader overlays (accepted by population admission) place population vertices themselves, so they are kept.
				// Stock vertex shaders are swapped for their population variants.
				auto vs = desc.Get<EPipeState::VS>().pShaderBytecode;
				auto const* overlay = PopulationVertexOverlay(material, m_step_id);
				if (overlay == nullptr || vs != overlay->m_code.VS.pShaderBytecode)
				{
					// Only stock vertex shaders have population variants
					if (vs == shader_code::forward_vs.pShaderBytecode)
						desc.Apply(PSO<EPipeState::VS>(shader_code::forward_population_vs));
					else if (vs == shader_code::forward_texn_pbr_vs.pShaderBytecode)
						desc.Apply(PSO<EPipeState::VS>(shader_code::forward_texn_population_vs));
					else
						throw std::logic_error("Population draw has a vertex shader without a population variant");
				}

				// The payload slot needs a valid address even when the population has no payload, so it aliases the instance records.
				auto const& lists = *population->m_lists;
				auto const& pop = *lists.m_population;
				auto draw = PopulationDraw{ .list = s_cast<uint32_t>(population->m_draw->m_list), .list_base = lists.m_list_base, .offsets = lists.m_offsets, .pad = 0 };
				cmd_list.SetGraphicsRootShaderResourceView(shaders::fwd::ERootParam::PopulationInstances, GpuAddress(pop.m_instances));
				cmd_list.SetGraphicsRootShaderResourceView(shaders::fwd::ERootParam::PopulationView, GpuAddress(lists.m_scratch));
				cmd_list.SetGraphicsRootShaderResourceView(shaders::fwd::ERootParam::PopulationPayload, pop.m_payload != nullptr ? GpuAddress(pop.m_payload) : GpuAddress(pop.m_instances));
				cmd_list.SetGraphicsRoot32BitConstants(shaders::fwd::ERootParam::PopulationDraw, draw);
			}

			// Draw the nugget.
			DrawNugget(cmd_list, nugget, desc, pipe_state_bound, pipe_state_hash, population);
		}
	}

	// Draw a single nugget
	void RenderForward::DrawNugget(GfxCmdList& cmd_list, Nugget const& nugget, PipeStateDesc& desc, bool& pipe_state_bound, int& pipe_state_hash, PopulationDrawRef const* population)
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
