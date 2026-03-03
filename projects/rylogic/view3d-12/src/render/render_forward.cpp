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
#include "pr/view3d-12/model/skin.h"
#include "pr/view3d-12/model/pose.h"
#include "pr/view3d-12/model/vertex_layout.h"
#include "pr/view3d-12/shaders/shader.h"
#include "pr/view3d-12/shaders/shader_forward.h"
#include "pr/view3d-12/shaders/shader_registers.h"
#include "pr/view3d-12/texture/texture_base.h"
#include "pr/view3d-12/texture/texture_2d.h"
#include "pr/view3d-12/texture/texture_cube.h"
#include "pr/view3d-12/sampler/sampler.h"
#include "pr/view3d-12/utility/wrappers.h"
#include "pr/view3d-12/utility/pipe_state.h"
#include "view3d-12/src/shaders/common.h"

namespace pr::rdr12
{
	RenderForward::RenderForward(Scene& scene)
		: RenderStep(Id, scene)
		, m_shader(scene.rdr())
		, m_cmd_list(scene.d3d(), nullptr, "RenderForward", EColours::Blue)
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
				scene.wnd().m_rt_props.Format,
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
	}
	RenderForward::~RenderForward()
	{
		m_default_tex = nullptr;
		m_default_sam = nullptr;
	}

	// Add model nuggets to the draw list for this render step.
	void RenderForward::AddNuggets(BaseInstance const& inst, TNuggetChain const& nuggets, drawlist_t& drawlist)
	{
		drawlist.reserve(drawlist.size() + nuggets.size());

		// Add a draw list element for each nugget in the instance's model
		for (auto& nug : nuggets)
		{
			// Ignore if flagged as not visible
			if (AllSet(nug.m_nflags, ENuggetFlag::Hidden))
				continue;

			for (;;)
			{
				// Don't add alpha back faces when using 'Points' fill mode
				if (nug.m_id == AlphaNuggetId && nug.FillMode() == EFillMode::Points)
					break;

				// Create the combined sort key for this nugget
				auto sk = nug.m_sort_key;
				auto sko = inst.find<SKOverride>(EInstComp::SortkeyOverride);
				if (sko) sk = sko->Combine(sk);

				// Set the texture id part of the key if not set already
				if (!AnySet(sk, SortKey::TextureIdMask) && nug.m_tex_diffuse != nullptr)
					sk = SetBits(sk, SortKey::TextureIdMask, nug.m_tex_diffuse->SortId() << SortKey::TextureIdOfs);

				// Set the shader id part of the key if not set already
				if (!AnySet(sk, SortKey::ShaderIdMask))
				{
					auto shdr_id = 0;
					for (auto& overlay : nug.m_shdr_overlays)
					{
						// Ignore shader overlays for other render steps
						if (overlay.m_rdr_step != m_step_id)
							continue;

						// hash the sort ids together
						shdr_id = shdr_id*13 ^ overlay.m_overlay->SortId();
					}
					sk = SetBits(sk, SortKey::ShaderIdMask, shdr_id << SortKey::ShaderIdOfs);
				}

				// Add an element to the draw list
				DrawListElement dle = {
					.m_sort_key = sk,
					.m_nugget = &nug,
					.m_instance = &inst,
				};
				drawlist.push_back(dle);
				m_sort_needed = true;
				break;
			}

			// Recursively add dependent nuggets
			if (!nug.m_nuggets.empty())
				AddNuggets(inst, nug.m_nuggets, drawlist);
		}
	}

	// Perform the render step
	void RenderForward::Execute(Frame& frame)
	{
		// Reset the command list with a new allocator for this frame
		m_cmd_list.Reset(frame.m_cmd_alloc_pool.Get());

		// Add the command lists we're using to the frame.
		frame.m_main.push_back(m_cmd_list);

		// Sort the draw list if needed
		SortIfNeeded();

		// Bind the descriptor heaps
		auto des_heaps = { wnd().m_heap_view.get(), wnd().m_heap_samp.get() };
		m_cmd_list.SetDescriptorHeaps({ des_heaps.begin(), des_heaps.size() });

		// Get the back buffer view handle and set the back buffer as the render target.
		m_cmd_list.OMSetRenderTargets({ &frame.bb_main().m_rtv, 1 }, FALSE, &frame.bb_main().m_dsv);

		// Set the viewport and scissor rect.
		auto const& vp = scn().m_viewport;
		m_cmd_list.RSSetViewports({ &vp, 1 });
		m_cmd_list.RSSetScissorRects(vp.m_clip);

		// Set the signature for the shader used for this nugget
		m_cmd_list.SetGraphicsRootSignature(m_shader.m_signature.get());

		// Set shader constants for the frame
		m_shader.SetupFrame(m_cmd_list.get(), m_upload_buffer, scn());

		// Add the shadow map textures
		if (auto* smap_step = scn().FindRStep<RenderSmap>())
		{
			// Todo: consider array-of-structs layout for casters
			pr::vector<Descriptor, 8> descriptors;
			for (auto& caster : smap_step->Casters())
				descriptors.push_back(caster.m_smap->m_srv);

			auto gpu = wnd().m_heap_view.Add(descriptors);
			m_cmd_list.SetGraphicsRootDescriptorTable(shaders::fwd::ERootParam::SMap, gpu);
		}

		// Add the global environment map
		if (auto* envmap = scn().m_global_envmap.get())
		{
			auto gpu = wnd().m_heap_view.Add(envmap->m_srv);
			m_cmd_list.SetGraphicsRootDescriptorTable(shaders::fwd::ERootParam::EnvMap, gpu);
		}

		D3D12_GPU_DESCRIPTOR_HANDLE last_tex = {}, last_sam = {};

		// Draw each element in the draw list
		auto drawlist = m_drawlist.lock();
		for (auto& dle : *drawlist)
		{
			// Something not rendering?
			//  - Check the tint for the nugget isn't 0x00000000.
			// Tips:
			//  - To uniquely identify an instance in a shader for debugging, set the Instance Id (cb1.m_flags.w)
			//    Then in the shader, use: if (m_flags.w == 1234) ...
			auto const& nugget = *dle.m_nugget;
			auto const& instance = *dle.m_instance;
			auto desc = m_default_pipe_state;

			// Set pipeline state
			desc.Apply(PSO<EPipeState::TopologyType>(To<D3D12_PRIMITIVE_TOPOLOGY_TYPE>(nugget.m_topo)));
			m_cmd_list.IASetPrimitiveTopology(nugget.m_topo);
			m_cmd_list.IASetVertexBuffers(0U, { &nugget.m_model->m_vb_view, 1 });
			m_cmd_list.IASetIndexBuffer(&nugget.m_model->m_ib_view);

			// Bind textures to the pipeline
			if (Texture2DPtr tex = coalesce(FindDiffTexture(instance), nugget.m_tex_diffuse, m_default_tex))
			{
				auto srv_descriptor = wnd().m_heap_view.Add(tex->m_srv);
				if (srv_descriptor.ptr != last_tex.ptr)
				{
					m_cmd_list.SetGraphicsRootDescriptorTable(shaders::fwd::ERootParam::DiffTexture, srv_descriptor);
					last_tex = srv_descriptor;
				}
				if constexpr (PR_DBG_RDR)
				{
					// Ensure the diffuse texture is in the correct state
					auto state = m_cmd_list.ResState(tex->m_res.get()).Mip0State();
					assert(AllSet(state, D3D12_RESOURCE_STATE_ALL_SHADER_RESOURCE));
				}
			}

			// Bind samplers to the pipeline
			if (SamplerPtr sam = coalesce(FindDiffTextureSampler(instance), nugget.m_sam_diffuse, m_default_sam))
			{
				auto sam_descriptor = wnd().m_heap_samp.Add(sam->m_samp);
				if (sam_descriptor.ptr != last_sam.ptr)
				{
					m_cmd_list.SetGraphicsRootDescriptorTable(shaders::fwd::ERootParam::DiffTextureSampler, sam_descriptor);
					last_sam = sam_descriptor;
				}
			}

			// Add skinning data for skinned meshes
			if (PosePtr pose = FindPose(instance); pose && nugget.m_model->m_skin)
			{
				pose->Update(m_cmd_list, m_upload_buffer);
				auto srv_pose = wnd().m_heap_view.Add(pose->m_srv);
				auto srv_skin = wnd().m_heap_view.Add(nugget.m_model->m_skin.m_srv);
				m_cmd_list.SetGraphicsRootDescriptorTable(shaders::fwd::ERootParam::Pose, srv_pose);
				m_cmd_list.SetGraphicsRootDescriptorTable(shaders::fwd::ERootParam::Skin, srv_skin);
			}

			// Set shader constants for the nugget
			m_shader.SetupElement(m_cmd_list.get(), m_upload_buffer, scn(), &dle);

			// Apply scene pipe state overrides
			{
				for (auto& ps : scn().m_pso)
					desc.Apply(ps);
				for (auto& ps : nugget.m_pso)
					desc.Apply(ps);
				for (auto& ps : GetPipeStates(instance))
					desc.Apply(ps);
			}

			// Apply nugget shader overrides
			for (auto& shdr_overlay : nugget.m_shdr_overlays)
			{
				// Ignore shader overlays for other render steps
				if (shdr_overlay.m_rdr_step != m_step_id)
					continue;

				auto& overlay = *shdr_overlay.m_overlay.get();

				// Update the pipe state with the non-null stage's byte code
				if (overlay.m_signature) desc.Apply(PSO<EPipeState::RootSignature>(overlay.m_signature.get()));
				if (overlay.m_code.VS) desc.Apply(PSO<EPipeState::VS>(overlay.m_code.VS));
				if (overlay.m_code.PS) desc.Apply(PSO<EPipeState::PS>(overlay.m_code.PS));
				if (overlay.m_code.DS) desc.Apply(PSO<EPipeState::DS>(overlay.m_code.DS));
				if (overlay.m_code.HS) desc.Apply(PSO<EPipeState::HS>(overlay.m_code.HS));
				if (overlay.m_code.GS) desc.Apply(PSO<EPipeState::GS>(overlay.m_code.GS));

				// Set constants for the shader
				overlay.SetupFrame(m_cmd_list.get(), m_upload_buffer, scn());
				overlay.SetupElement(m_cmd_list.get(), m_upload_buffer, scn(), &dle);
			}

			// Draw the nugget **** 
			DrawNugget(nugget, desc);
		}

		// Close the command list now that we've finished rendering this scene
		m_cmd_list.Close();
	}

	// Draw a single nugget
	void RenderForward::DrawNugget(Nugget const& nugget, PipeStateDesc& desc)
	{
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

			m_cmd_list.SetPipelineState(m_pipe_state_pool.Get(desc));
			if (nugget.m_irange.empty())
			{
				m_cmd_list.DrawInstanced(
					s_cast<size_t>(nugget.m_vrange.size()), 1U,
					s_cast<size_t>(nugget.m_vrange.m_beg), 0U);
			}
			else
			{
				m_cmd_list.DrawIndexedInstanced(
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
			m_cmd_list.SetPipelineState(m_pipe_state_pool.Get(desc));

			m_cmd_list.DrawIndexedInstanced(
				s_cast<size_t>(nugget.m_irange.size()), 1U,
				s_cast<size_t>(nugget.m_irange.m_beg), 0, 0U);

			// Restore it
			desc.Apply(PSO<EPipeState::FillMode>(prev_fill_mode));
		}

		// Render points for 'Points' mode
		if (fill_mode == EFillMode::Points)
		{
			// Configure the shader for point sprites
			// Don't need 'dle' if the points aren't in screen space
			wnd().m_diag.m_gs_fillmode_points->SetupElement(m_cmd_list.get(), m_upload_buffer, scn(), nullptr);

			// Change the pipe state and IA topology to point list.
			// Both must agree: PSO topology type and IA primitive topology.
			desc.Apply(PSO<EPipeState::TopologyType>(To<D3D12_PRIMITIVE_TOPOLOGY_TYPE>(ETopo::PointList)));
			desc.Apply(PSO<EPipeState::GS>(wnd().m_diag.m_gs_fillmode_points->m_code.GS));
			m_cmd_list.IASetPrimitiveTopology(ETopo::PointList);
			m_cmd_list.SetPipelineState(m_pipe_state_pool.Get(desc));

			m_cmd_list.DrawInstanced(
				s_cast<size_t>(nugget.m_vrange.size()), 1U,
				s_cast<size_t>(nugget.m_vrange.m_beg), 0U);
		}
	}
}
