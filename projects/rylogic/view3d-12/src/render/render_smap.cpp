//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#include "view3d-12/src/render/render_smap.h"
#include "pr/view3d-12/main/renderer.h"
#include "pr/view3d-12/main/window.h"
#include "pr/view3d-12/scene/scene.h"
#include "pr/view3d-12/model/nugget.h"
#include "pr/view3d-12/model/model.h"
#include "pr/view3d-12/model/skinned_geometry.h"
#include "pr/view3d-12/model/vertex_layout.h"
#include "pr/view3d-12/resource/resource_factory.h"
#include "pr/view3d-12/texture/texture_desc.h"
#include "pr/view3d-12/texture/texture_base.h"
#include "pr/view3d-12/texture/texture_2d.h"
#include "pr/view3d-12/sampler/sampler.h"
#include "pr/view3d-12/utility/pipe_state.h"
#include "pr/view3d-12/utility/diagnostics.h"
#include "view3d-12/src/shaders/common.h"

namespace pr::rdr12
{
	using namespace ::pr::compute;

	RenderSmap::RenderSmap(Scene& scene)
		: RenderStep(Id, scene, scene.wnd().m_gsync)
		, m_shader(scene.rdr())
		, m_cmd_list(scene.d3d(), nullptr, "RenderSmap", EColours::Yellow)
		, m_default_tex(rdr().store().StockTexture(EStockTexture::White))
		, m_default_sam(rdr().store().StockSampler(EStockSampler::LinearClamp))
		, m_atlas()
		, m_settings(scene.Shadows())
		, m_views()
		, m_element_bounds()
	{
		// Create the atlas and pipeline state for the scene's current settings
		CreateAtlas(m_settings);
	}

	// The shadow views for the current frame
	ShadowViewSet const& RenderSmap::Views() const
	{
		return m_views;
	}

	// The shadow atlas depth texture
	Texture2D const* RenderSmap::Atlas() const
	{
		return m_atlas.get();
	}

	// The width and height of the shadow atlas (in pixels)
	int RenderSmap::AtlasSize() const
	{
		return m_settings.m_atlas_size;
	}

	// Create the atlas texture and the pipeline state description for the current shadow settings
	void RenderSmap::CreateAtlas(ShadowSettings const& settings)
	{
		// The atlas is a depth buffer that is also read as a texture. The typeless format allows both views.
		ResourceFactory factory(rdr());
		auto td = ResDesc::Tex2D(Image(settings.m_atlas_size, settings.m_atlas_size, nullptr, DXGI_FORMAT_R16_TYPELESS), 1, EUsage::DepthStencil)
			.def_state(D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE | D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE)
			.clear(DXGI_FORMAT_D16_UNORM, D3D12_DEPTH_STENCIL_VALUE{ .Depth = 1.0f, .Stencil = 0 });
		auto desc = TextureDesc(AutoId, td).srv_format(DXGI_FORMAT_R16_UNORM).dsv_format(DXGI_FORMAT_D16_UNORM).name("ShadowAtlas");
		m_atlas = factory.CreateTexture2D(desc);
		m_settings = settings;

		// Depth-only rendering. The rasterizer depth bias pushes stored depths away from the light to reduce self shadowing.
		auto raster = RasterStateDesc{}.Set(D3D12_CULL_MODE_BACK);
		raster.DepthBias = settings.m_depth_bias;
		raster.SlopeScaledDepthBias = settings.m_slope_bias;
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
			.RasterizerState = raster,
			.DepthStencilState = DepthStateDesc{},
			.InputLayout = Vert::LayoutDesc(),
			.IBStripCutValue = D3D12_INDEX_BUFFER_STRIP_CUT_VALUE_DISABLED,
			.PrimitiveTopologyType = D3D12_PRIMITIVE_TOPOLOGY_TYPE_TRIANGLE,
			.NumRenderTargets = 0U,
			.RTVFormats = {
				DXGI_FORMAT_UNKNOWN,
				DXGI_FORMAT_UNKNOWN,
				DXGI_FORMAT_UNKNOWN,
				DXGI_FORMAT_UNKNOWN,
				DXGI_FORMAT_UNKNOWN,
				DXGI_FORMAT_UNKNOWN,
				DXGI_FORMAT_UNKNOWN,
				DXGI_FORMAT_UNKNOWN,
			},
			.DSVFormat = DXGI_FORMAT_D16_UNORM,
			.SampleDesc = MultiSamp{},
			.NodeMask = 0U,
			.CachedPSO = {
				.pCachedBlob = nullptr,
				.CachedBlobSizeInBytes = 0U,
			},
			.Flags = D3D12_PIPELINE_STATE_FLAG_NONE,
		};
	}

	// Add model nuggets to the draw list for this render step
	void RenderSmap::AddNuggets(BaseInstance const& inst, NuggetPtr nuggets, drawlist_t& drawlist)
	{
		// Ignore instances that don't cast shadows
		if (AnySet(GetFlags(inst), EInstFlag::ShadowCastExclude))
			return;

		auto material_override = FindMaterial(inst);

		// Use the default nuggets. This means alpha objects will cast shadows as if they were opaque
		for (auto& nug : Enumerate(nuggets))
		{
			// Filter out nuggets that can't cast shadows
			if (AnySet(nug.m_nflags, ENuggetFlag::ShadowCastExclude | ENuggetFlag::Hidden))
				continue;

			// Don't add nuggets without a surface area
			if (nug.FillMode() != EFillMode::Default && nug.FillMode() != EFillMode::Solid)
				break;

			// Create the combined sort key for this nugget. The shader part is ignored because all nuggets use the shadow map shader.
			auto sk = nug.m_sort_key;
			if (auto* sko = inst.find<SKOverride>(EInstComp::SortkeyOverride))
				sk = sko->Combine(sk);

			// Only nuggets whose material has a pass for this step can cast shadows
			auto const& material = material_override != nullptr ? *material_override.get() : nug.mat();
			auto const* pass = material.Pass(m_step_id);
			if (pass == nullptr)
				continue;

			sk = pass->AddSortKey(m_step_id, inst, material, nug, sk);

			// Add an element to the draw list
			drawlist.push_back(DrawListElement{ .m_sort_key = sk, .m_nugget = &nug, .m_instance = &inst });
			m_sort_needed = true;
		}
	}

	// Choose the shadow views for the frame
	void RenderSmap::Prepare(Frame& frame)
	{
		// Let the base class prepare, and sort now so that element bounds match the draw order used in 'Execute'
		RenderStep::Prepare(frame);
		SortIfNeeded();

		// Recreate the atlas when the settings change. Earlier frames may still read the old atlas, so wait for them first.
		if (!(scn().Shadows() == m_settings))
		{
			wnd().m_gsync.Wait();
			CreateAtlas(scn().Shadows());
		}

		// Find the world space bounds of each element and of all casters. Elements whose bounds cannot be known
		// (skinned, non-affine, or without a valid model bbox) are drawn into every view.
		auto caster_bounds = BBox::Reset();
		m_element_bounds.resize(0);
		{
			auto drawlist = m_drawlist.lock();
			for (auto& dle : *drawlist)
			{
				auto const& nugget = *dle.m_nugget;
				auto const& instance = *dle.m_instance;
				auto const& mbox = nugget.m_model->m_bbox;
				auto i2w = GetO2W(instance);
				if (!mbox.valid() || !IsAffine(i2w))
				{
					m_element_bounds.push_back(BBox::Reset());
					continue;
				}

				// Skinned models can move outside their bind pose bounds, so they contribute to the caster bounds but are never culled
				auto bbox = i2w * mbox;
				Grow(caster_bounds, bbox);
				m_element_bounds.push_back(FindPose(instance) != nullptr ? BBox::Reset() : bbox);
			}
		}

		// Choose views for the shadow-casting lights and pack them into the atlas
		BuildShadowViews(scn().ResolvedLights(), caster_bounds, m_settings, m_views);
	}

	// Perform the render step
	void RenderSmap::Execute(Frame& frame)
	{
		// Nothing to render if no light has a shadow view
		if (m_views.empty())
			return;

		// Record into a new allocator for this frame
		m_cmd_list.Reset(frame.m_cmd_alloc_pool.Get());
		frame.m_main.push_back(m_cmd_list);

		// Bind the descriptor heaps
		auto des_heaps = { wnd().m_heap_view.get(), wnd().m_heap_samp.get() };
		m_cmd_list.SetDescriptorHeaps({ des_heaps.begin(), des_heaps.size() });

		// Bind the atlas as the depth target and clear only the regions in use
		auto& atlas = *m_atlas.get();
		BarrierBatch barriers(m_cmd_list);
		barriers.Transition(atlas.m_res.get(), D3D12_RESOURCE_STATE_DEPTH_WRITE);
		barriers.Commit();
		m_cmd_list.OMSetRenderTargets({}, false, &atlas.m_dsv.m_cpu);
		{
			pr::vector<D3D12_RECT, MaxShadowViews> clear_rects;
			for (auto const& view : m_views.m_views)
				clear_rects.push_back(D3D12_RECT{ view.m_atlas_rect.m_min.x, view.m_atlas_rect.m_min.y, view.m_atlas_rect.m_max.x, view.m_atlas_rect.m_max.y });

			m_cmd_list.ClearDepthStencilView(atlas.m_dsv.m_cpu, D3D12_CLEAR_FLAG_DEPTH, 1.0f, 0, clear_rects);
		}

		// Upload the view transforms for the whole frame
		m_cmd_list.SetGraphicsRootSignature(m_shader.m_signature.get());
		auto views_gpu = UploadShadowViews(m_upload_buffer, m_views, m_settings.m_atlas_size);
		m_shader.SetupFrame(m_cmd_list.get(), views_gpu);

		// Per-element projections use the scene camera
		auto const camera = CameraTransforms(scn().m_cam);

		// Render the views in batches, one viewport per view in the batch
		auto view_count = s_cast<int>(m_views.m_views.size());
		for (int batch_beg = 0; batch_beg < view_count; batch_beg += ShadowViewBatchSize)
		{
			auto batch_end = std::min(batch_beg + ShadowViewBatchSize, view_count);

			// Set one viewport and scissor rect per view. The viewport index in the shader is relative to 'batch_beg'.
			{
				D3D12_VIEWPORT viewports[ShadowViewBatchSize];
				D3D12_RECT scissors[ShadowViewBatchSize];
				for (int i = batch_beg; i != batch_end; ++i)
				{
					auto const& rect = m_views.m_views[i].m_atlas_rect;
					viewports[i - batch_beg] = D3D12_VIEWPORT{
						.TopLeftX = s_cast<float>(rect.m_min.x),
						.TopLeftY = s_cast<float>(rect.m_min.y),
						.Width = s_cast<float>(rect.SizeX()),
						.Height = s_cast<float>(rect.SizeY()),
						.MinDepth = 0.0f,
						.MaxDepth = 1.0f,
					};
					scissors[i - batch_beg] = D3D12_RECT{ rect.m_min.x, rect.m_min.y, rect.m_max.x, rect.m_max.y };
				}
				m_cmd_list.get()->RSSetViewports(s_cast<UINT>(batch_end - batch_beg), &viewports[0]);
				m_cmd_list.get()->RSSetScissorRects(s_cast<UINT>(batch_end - batch_beg), &scissors[0]);
			}

			// Draw each element once, with one instance per view in this batch that can see it
			auto drawlist = m_drawlist.lock();
			assert(m_element_bounds.size() == drawlist->size() && "Draw list changed between Prepare and Execute");
			for (int e = 0, eend = s_cast<int>(drawlist->size()); e != eend; ++e)
			{
				auto const& dle = (*drawlist)[e];
				auto const& nugget = *dle.m_nugget;
				auto const& instance = *dle.m_instance;
				auto const& bounds = m_element_bounds[e];

				// Find the views in this batch that can see the element
				pr::vector<uint32_t, ShadowViewBatchSize> views;
				for (int i = batch_beg; i != batch_end; ++i)
				{
					if (bounds.valid() && !ShadowViewSees(m_views.m_views[i].m_w2s, bounds))
						continue;

					views.push_back(s_cast<uint32_t>(i));
				}
				if (views.empty())
					continue;

				// Find the material pass for this step
				auto material_override = FindMaterial(instance);
				auto const& material = material_override != nullptr ? *material_override.get() : nugget.mat();
				auto const* pass = material.Pass(m_step_id);
				if (pass == nullptr)
					continue;

				// If the instance is skinned, get the post-skinned vertex buffer for this instance's pose
				auto const* vb_view = &nugget.m_model->m_vb_view;
				if (PosePtr pose = FindPose(instance); pose && nugget.m_model->m_skin)
					vb_view = &rdr().SkinnedGeometry().VBufView(m_cmd_list, m_upload_buffer, *nugget.m_model, pose);

				// Set the pipeline state and geometry
				auto desc = m_default_pipe_state;
				desc.Apply(PSO<EPipeState::TopologyType>(To<D3D12_PRIMITIVE_TOPOLOGY_TYPE>(nugget.m_topo)));
				if (IsStripTopo(nugget.m_topo))
					desc.Apply(PSO<EPipeState::IBStripCutValue>(StripCutValue(nugget.m_model->m_ib_view.Format)));

				m_cmd_list.IASetPrimitiveTopology(nugget.m_topo);
				m_cmd_list.IASetVertexBuffers(0U, { vb_view, 1 });
				m_cmd_list.IASetIndexBuffer(&nugget.m_model->m_ib_view);

				// Let the material bind per-draw resources, constants, and pipeline overrides
				auto ctx = MaterialPassContext{
					.m_step_id = m_step_id,
					.m_wnd = wnd(),
					.m_scene = scn(),
					.m_camera = camera,
					.m_dle = dle,
					.m_material = material,
					.m_cmd_list = m_cmd_list,
					.m_upload = m_upload_buffer,
					.m_pipe_state = desc,
					.m_shader = &m_shader,
					.m_default_tex = m_default_tex.get(),
					.m_default_sam = m_default_sam.get(),
					.m_last_tex = nullptr,
					.m_last_sam = nullptr,
				};
				pass->Bind(ctx);
				pass->ApplyPipeline(ctx);

				// Bind the views and draw one instance per view
				m_shader.SetupDrawViews(m_cmd_list.get(), views, batch_beg);
				m_cmd_list.SetPipelineState(m_pipe_state_pool.Get(desc));
				if (!nugget.m_irange.empty())
					m_cmd_list.DrawIndexedInstanced(s_cast<size_t>(nugget.m_irange.size()), views.size(), s_cast<size_t>(nugget.m_irange.m_beg), 0, 0U);
				else
					m_cmd_list.DrawInstanced(s_cast<size_t>(nugget.m_vrange.size()), views.size(), s_cast<size_t>(nugget.m_vrange.m_beg), 0U);
			}
		}

		// Make the atlas readable by the forward pass
		barriers.Transition(atlas.m_res.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE | D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE);
		barriers.Commit();

		// Close the command list now that the shadow views are rendered
		m_cmd_list.Close();
	}
}
