//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2026
//*********************************************
#include "pr/view3d-12/postprocessing/post_processing.h"
#include "pr/view3d-12/main/frame.h"
#include "pr/view3d-12/main/renderer.h"
#include "pr/view3d-12/main/window.h"
#include "pr/view3d-12/render/back_buffer.h"
#include "pr/view3d-12/resource/resource_factory.h"
#include "pr/view3d-12/scene/scene.h"
#include "pr/view3d-12/shaders/shader.h"
#include "pr/view3d-12/texture/texture_desc.h"
#include "view3d-12/src/shaders/common.h"
#include "view3d-12/src/shaders/hlsl/postprocessing/underwater_cbuf.hlsli"

namespace pr::rdr12
{
	using namespace ::pr::compute;

	// Root parameters shared by all full-screen effect passes.
	enum class EPostRootParam
	{
		Constants = 0,
		SceneColour = 1,
		SceneDepth = 2,
	};

	// The per-pass inputs, shared by every effect pass.
	struct PostProcessing::PassContext
	{
		Frame& m_frame;
		Scene const& m_scene;
		GfxCmdList& m_cmd_list;
		D3D12_GPU_DESCRIPTOR_HANDLE m_scene_colour; // SRV of the effect input colour
		D3D12_GPU_DESCRIPTOR_HANDLE m_scene_depth;  // SRV of the resolved scene depth, or a null SRV
		bool m_has_depth;
		float m_clear_depth;                        // Depth value of pixels where no geometry was drawn
	};

	// Convert a world-space plane to camera space with a unit-length normal. A zero plane stays zero.
	static v4 CameraSpaceSurface(v4 surface, m4x4 const& c2w)
	{
		// Planes transform by the transpose of the point transform: the normal rotates into camera space, and the
		// offset becomes the plane's signed distance at the camera origin.
		if (All(surface == v4::Zero()))
			return v4::Zero();

		auto plane = surface / Length(surface.w0());
		auto normal = InvertAffine(c2w) * plane.w0();
		return v4{ normal.x, normal.y, normal.z, Dot(plane, c2w.pos) };
	}

	// How the camera's near plane meets the water surface.
	enum class EUnderwaterView
	{
		Hidden,   // The whole near plane is above the surface, so nothing is drawn
		Immersed, // The whole near plane is below the surface and deeper than the fade depth, so the effect is at full strength
		Split,    // The surface or the fade region crosses the near plane
	};
	struct UnderwaterView
	{
		EUnderwaterView m_view;
		v4 m_waterline; // Near plane height above the surface as 'x*ndc.x + y*ndc.y + z'. Set only for 'Split'.
	};

	// Classify the near plane of 'cam' against the surface of 'props'.
	static UnderwaterView ClassifyUnderwater(UnderwaterProps const& props, SceneCamera const& cam)
	{
		// Without a surface, the whole view is in water.
		if (All(props.m_surface == v4::Zero()))
			return { EUnderwaterView::Immersed, v4::Zero() };

		// Find the NDC depth of the near plane. The camera looks down -z, so the near plane is at z = -near.
		auto c2s = cam.CameraToScreen();
		auto s2c = Invert(c2s);
		auto near_ndc = c2s * v4{ 0, 0, -s_cast<float>(cam.Near(false)), 1 };
		auto near_z = near_ndc.z / near_ndc.w;

		// The near plane is flat and maps linearly to NDC x and y, so its height above the surface is linear too.
		// Measure the height at the NDC origin and its change per unit of NDC x and y.
		auto surface = CameraSpaceSurface(props.m_surface, cam.CameraToWorld());
		auto Height = [&](float x, float y)
		{
			// Unproject onto the near plane, then measure the signed distance to the surface.
			auto p = s2c * v4{ x, y, near_z, 1 };
			return Dot(surface, p / p.w);
		};
		auto h0 = Height(0, 0);
		auto waterline = v4{ Height(1, 0) - h0, Height(0, 1) - h0, h0, 0 };

		// The highest and lowest points of the near plane are at its corners, NDC (+/-1, +/-1).
		auto spread = Abs(waterline.x) + Abs(waterline.y);
		if (h0 - spread >= 0)
			return { EUnderwaterView::Hidden, v4::Zero() };
		if (h0 + spread < -props.m_fade_depth)
			return { EUnderwaterView::Immersed, v4::Zero() };

		return { EUnderwaterView::Split, waterline };
	}

	PostProcessing::PostProcessing(Renderer& rdr)
		: m_rdr(&rdr)
		, m_underwater()
		, m_clock_start(std::chrono::steady_clock::now())
		, m_targets()
		, m_target_size()
		, m_target_format(DXGI_FORMAT_UNKNOWN)
		, m_signature()
		, m_pso_underwater()
		, m_pso_format(DXGI_FORMAT_UNKNOWN)
	{}
	PostProcessing::~PostProcessing()
	{
		ReleaseResources();
	}

	// True if any effect is enabled
	bool PostProcessing::AnyEnabled() const
	{
		return m_underwater.m_enabled;
	}

	// Get/Set the underwater effect settings
	UnderwaterProps const& PostProcessing::Underwater() const
	{
		return m_underwater;
	}
	void PostProcessing::Underwater(UnderwaterProps const& props)
	{
		props.Validate();
		m_underwater = props;
	}

	// Record the enabled effects for 'scene' into 'frame'.
	void PostProcessing::Render(Frame& frame, Scene const& scene)
	{
		// Disabled effects hold no GPU memory and record nothing.
		if (!AnyEnabled())
		{
			ReleaseResources();
			return;
		}

		// Collect the passes with something to draw, in their canonical order. New effects are added here, in the order they should compose.
		std::array<PassFn, 1> passes = {};
		auto pass_count = 0;
		if (m_underwater.m_enabled && ClassifyUnderwater(m_underwater, scene.m_cam).m_view != EUnderwaterView::Hidden)
			passes[pass_count++] = &PostProcessing::RecordUnderwater;

		// Enabled effects with nothing to draw this frame keep their resources, so a camera moving in and out of view does not recreate them.
		if (pass_count == 0)
			return;

		// Effects need a final colour target to read from and write to.
		auto const& bb_post = frame.bb_post();
		if (bb_post.m_render_target == nullptr)
			return;

		auto& wnd = scene.wnd();
		auto& cmd_list = frame.m_post_effects;
		auto output = const_cast<ID3D12Resource*>(bb_post.m_render_target.get());
		auto output_format = output->GetDesc().Format;
		EnsureResources(bb_post.rt_size(), output_format, pass_count);

		// Produce the single-sample scene depth. The depth resolve list runs before the effects list.
		// Without a depth buffer a null view is bound, which the passes treat as "no scene geometry".
		wnd.RecordDepthResolve(frame.m_depth_resolve, frame.bb_main());
		auto resolved_depth = wnd.ResolvedDepth(frame.bb_main());
		auto depth_srv = wnd.m_heap_view.Add(resolved_depth, D3D12_SHADER_RESOURCE_VIEW_DESC{
			.Format = resolved_depth != nullptr ? wnd.ResolvedDepthSrvFormat() : DXGI_FORMAT_R32_FLOAT,
			.ViewDimension = D3D12_SRV_DIMENSION_TEXTURE2D,
			.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING,
			.Texture2D = {
				.MostDetailedMip = 0U,
				.MipLevels = 1U,
				.PlaneSlice = 0U,
				.ResourceMinLODClamp = 0.0f,
			},
		});

		// Copy the composited scene so the first pass can read it while writing to the next target.
		BarrierBatch copy_barriers(cmd_list);
		copy_barriers.Transition(output, D3D12_RESOURCE_STATE_COPY_SOURCE);
		copy_barriers.Transition(m_targets[0]->m_res.get(), D3D12_RESOURCE_STATE_COPY_DEST);
		copy_barriers.Commit();
		cmd_list.CopyResource(m_targets[0]->m_res.get(), output);

		// Bind the state shared by all passes. The scissor limits each pass to the scene viewport.
		auto heaps = { wnd.m_heap_view.get() };
		cmd_list.SetDescriptorHeaps({ heaps.begin(), heaps.size() });
		cmd_list.RSSetViewports({ &scene.m_viewport, 1 });
		cmd_list.RSSetScissorRects(scene.m_viewport.m_clip);
		cmd_list.IASetPrimitiveTopology(ETopo::TriList);
		cmd_list.SetGraphicsRootSignature(m_signature.get());

		// Run each pass, reading the previous result and writing the next target. The last pass writes the output.
		for (int i = 0; i != pass_count; ++i)
		{
			// Swap the roles of the two targets between passes.
			auto& source = m_targets[i % 2];
			auto last = i == pass_count - 1;
			auto dest = last ? output : m_targets[(i + 1) % 2]->m_res.get();
			auto dest_rtv = last ? bb_post.m_rtv : m_targets[(i + 1) % 2]->m_rtv.m_cpu;

			BarrierBatch bb(cmd_list);
			bb.Transition(source->m_res.get(), D3D12_RESOURCE_STATE_ALL_SHADER_RESOURCE);
			bb.Transition(dest, D3D12_RESOURCE_STATE_RENDER_TARGET);
			bb.Commit();

			cmd_list.OMSetRenderTargets({ &dest_rtv, 1 }, FALSE, nullptr);
			auto ctx = PassContext{
				.m_frame = frame,
				.m_scene = scene,
				.m_cmd_list = cmd_list,
				.m_scene_colour = wnd.m_heap_view.Add(source->m_srv),
				.m_scene_depth = depth_srv,
				.m_has_depth = resolved_depth != nullptr,
				.m_clear_depth = frame.bb_main().ds_depth(),
			};
			(this->*passes[i])(ctx);
		}
	}

	// Create or resize the resources needed to run 'pass_count' passes into a target of 'size' and 'format'.
	void PostProcessing::EnsureResources(iv2 size, DXGI_FORMAT format, int pass_count)
	{
		// Views read and write the sRGB sibling so passes blend in linear colour, matching the back buffer's views.
		auto srgb_format = ToSRGB(format);

		// Recreate the targets when the output changes. Only the targets that the enabled passes need are kept.
		if (Any(m_target_size != size) || m_target_format != format)
		{
			m_targets = {};
			m_target_size = size;
			m_target_format = format;
		}
		auto target_count = std::min(pass_count, 2);
		for (int i = 0; i != 2; ++i)
		{
			// Drop targets that are no longer needed.
			if (i >= target_count)
			{
				m_targets[i] = nullptr;
				continue;
			}
			if (m_targets[i] != nullptr)
				continue;

			ResourceFactory factory(*m_rdr);
			auto desc = ResDesc::Tex2D(Image{ size.x, size.y, nullptr, format }, 1U, EUsage::RenderTarget)
				.clear(ClearValue(format, ColourZero))
				.def_state(D3D12_RESOURCE_STATE_ALL_SHADER_RESOURCE);
			m_targets[i] = factory.CreateTexture2D(TextureDesc(AutoId, desc).name(i == 0 ? "PostEffect-Target0" : "PostEffect-Target1").srv_format(srgb_format).rtv_format(srgb_format));
		}

		// Create the root signature shared by all passes.
		if (m_signature == nullptr)
		{
			m_signature = RootSig(ERootSigFlags::GraphicsOnly)
				.CBuf(hlsl::ECBufReg::b0, D3D12_SHADER_VISIBILITY_PIXEL)
				.SRV(hlsl::ESRVReg::t0, 1, D3D12_SHADER_VISIBILITY_PIXEL)
				.SRV(hlsl::ESRVReg::t1, 1, D3D12_SHADER_VISIBILITY_PIXEL)
				.Samp(D3D12_STATIC_SAMPLER_DESC{
					.Filter = D3D12_FILTER_MIN_MAG_MIP_LINEAR,
					.AddressU = D3D12_TEXTURE_ADDRESS_MODE_CLAMP,
					.AddressV = D3D12_TEXTURE_ADDRESS_MODE_CLAMP,
					.AddressW = D3D12_TEXTURE_ADDRESS_MODE_CLAMP,
					.MipLODBias = 0.0f,
					.MaxAnisotropy = 0,
					.ComparisonFunc = D3D12_COMPARISON_FUNC_NEVER,
					.BorderColor = D3D12_STATIC_BORDER_COLOR_OPAQUE_BLACK,
					.MinLOD = 0.0f,
					.MaxLOD = D3D12_FLOAT32_MAX,
					.ShaderRegister = 0,
					.RegisterSpace = 0,
					.ShaderVisibility = D3D12_SHADER_VISIBILITY_PIXEL,
				})
				.Create(m_rdr->d3d(), "PostEffectSig");
		}

		// Create the pass pipelines for the output format. PSOs are small, so all effects share one lifetime.
		if (m_pso_format != srgb_format)
		{
			// Describe a full-screen pass that writes one sRGB target without depth.
			auto CreatePso = [&](D3D12_SHADER_BYTECODE ps, char const* name)
			{
				// Only the pixel shader differs between effects.
				auto desc = D3D12_GRAPHICS_PIPELINE_STATE_DESC{
					.pRootSignature = m_signature.get(),
					.VS = shader_code::post_effect_vs,
					.PS = ps,
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
					.RTVFormats = { srgb_format },
					.DSVFormat = DXGI_FORMAT_UNKNOWN,
					.SampleDesc = MultiSamp(1, 0),
					.NodeMask = 0U,
					.CachedPSO = {},
					.Flags = D3D12_PIPELINE_STATE_FLAG_NONE,
				};
				D3DPtr<ID3D12PipelineState> pso;
				Check(m_rdr->d3d()->CreateGraphicsPipelineState(&desc, __uuidof(ID3D12PipelineState), (void**)pso.address_of()));
				DebugName(pso, name);
				return pso;
			};

			if (m_pso_underwater != nullptr)
				m_rdr->DeferRelease(m_pso_underwater);

			m_pso_underwater = CreatePso(shader_code::underwater_ps, "PostEffectUnderwaterPSO");
			m_pso_format = srgb_format;
		}
	}

	// Release all GPU resources once the GPU no longer uses them.
	void PostProcessing::ReleaseResources()
	{
		// Textures defer their own release; pipeline objects are handed to the renderer.
		m_targets = {};
		m_target_size = iv2{};
		m_target_format = DXGI_FORMAT_UNKNOWN;
		if (m_pso_underwater != nullptr)
			m_rdr->DeferRelease(m_pso_underwater);
		if (m_signature != nullptr)
			m_rdr->DeferRelease(m_signature);

		m_pso_underwater = nullptr;
		m_signature = nullptr;
		m_pso_format = DXGI_FORMAT_UNKNOWN;
	}

	// Tint, fog, and distort the scene colour.
	void PostProcessing::RecordUnderwater(PassContext const& ctx)
	{
		// Fill the constants. The distortion phase is wrapped in double precision so it stays smooth over long runs.
		auto const& props = m_underwater;
		auto const& vp = ctx.m_scene.m_viewport;
		auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - m_clock_start).count();
		auto cycles = elapsed * props.m_distortion_speed;
		auto view = ClassifyUnderwater(props, ctx.m_scene.m_cam);
		auto cb = shaders::post::CBufUnderwater{
			.s2c = Invert(ctx.m_scene.m_cam.CameraToScreen()),
			.tint = Colour(props.m_tint).rgba,
			.fog_colour = Colour(props.m_fog_colour).rgba,
			.viewport = v4{ vp.TopLeftX, vp.TopLeftY, vp.Width, vp.Height },
			.surface = CameraSpaceSurface(props.m_surface, ctx.m_scene.m_cam.CameraToWorld()),
			.waterline = view.m_waterline,
			.visibility = props.m_visibility,
			.phase = s_cast<float>(constants<double>::tau * (cycles - std::floor(cycles))),
			.distortion_amplitude = props.m_distortion_amplitude,
			.distortion_frequency = props.m_distortion_frequency,
			.has_depth = ctx.m_has_depth ? 1 : 0,
			.orthographic = ctx.m_scene.m_cam.Orthographic() ? 1 : 0,
			.clear_depth = ctx.m_clear_depth,
			.split = view.m_view == EUnderwaterView::Split ? 1 : 0,
			.fade_depth = props.m_fade_depth,
			.dither = ctx.m_scene.wnd().m_dither_amount,
		};

		// Draw the full-screen pass.
		auto& cmd_list = ctx.m_cmd_list;
		cmd_list.SetPipelineState(m_pso_underwater.get());
		cmd_list.SetGraphicsRootConstantBufferView(EPostRootParam::Constants, ctx.m_frame.m_upload.Add(cb, D3D12_CONSTANT_BUFFER_DATA_PLACEMENT_ALIGNMENT, false));
		cmd_list.SetGraphicsRootDescriptorTable(EPostRootParam::SceneColour, ctx.m_scene_colour);
		cmd_list.SetGraphicsRootDescriptorTable(EPostRootParam::SceneDepth, ctx.m_scene_depth);
		cmd_list.DrawInstanced(3, 1, 0, 0);
	}
}
