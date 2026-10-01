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

	namespace
	{
		// Mix the bytes of 'value' into the running content hash 'h'. 'T' must not contain padding bytes.
		template <typename T> void HashMix(uint64_t& h, T const& value)
		{
			// Mix whole 64-bit words, then any remaining bytes. The rotation carries high bits back into the low bits,
			// so a change in any word keeps affecting the whole hash as later words are mixed in.
			auto const* bytes = reinterpret_cast<uint8_t const*>(&value);
			auto mix = [&h](uint64_t word)
			{
				// Multiply by an odd constant, which is a bijection, so a single changed word always changes the hash
				h = std::rotl((h ^ word) * 0x9E3779B97F4A7C15ULL, 31);
			};
			size_t i = 0;
			for (; i + sizeof(uint64_t) <= sizeof(T); i += sizeof(uint64_t))
			{
				// Copy the word because 'value' need not be 8-byte aligned
				uint64_t word;
				std::memcpy(&word, bytes + i, sizeof(word));
				mix(word);
			}
			if (i != sizeof(T))
			{
				// Zero-extend the tail bytes into one word
				uint64_t word = 0;
				std::memcpy(&word, bytes + i, sizeof(T) - i);
				mix(word);
			}
		}

		// The initial value for content hashes
		constexpr uint64_t HashSeed = 0xcbf29ce484222325ULL;
	}

	RenderSmap::RenderSmap(Scene& scene)
		: RenderStep(Id, scene, scene.wnd().m_gsync)
		, m_shader(scene.rdr())
		, m_cmd_list(scene.d3d(), nullptr, "RenderSmap", EColours::Yellow)
		, m_default_tex(rdr().store().StockTexture(EStockTexture::White))
		, m_default_sam(rdr().store().StockSampler(EStockSampler::LinearClamp))
		, m_atlas()
		, m_settings(scene.Shadows())
		, m_views()
		, m_cache()
		, m_element_views()
		, m_dirty()
		, m_volatile()
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

	// The shadow settings used for the current frame
	ShadowSettings const& RenderSmap::Settings() const
	{
		return m_settings;
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

		// The new atlas has no content, so every view must be rendered again
		m_cache.Invalidate();
		m_volatile.clear();

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
		// Let the base class prepare, and sort now so that element data matches the draw order used in 'Execute'
		RenderStep::Prepare(frame);
		SortIfNeeded();

		// Apply changes to the shadow settings. Only the atlas size and depth biases need a new atlas and pipeline state.
		// Earlier frames may still read the old atlas, so wait for them before replacing it.
		if (auto const& settings = scn().Shadows(); !(settings == m_settings))
		{
			if (settings.m_atlas_size != m_settings.m_atlas_size || settings.m_depth_bias != m_settings.m_depth_bias || settings.m_slope_bias != m_settings.m_slope_bias)
			{
				wnd().m_gsync.Wait();
				CreateAtlas(settings);
			}
			else
			{
				m_settings = settings;
			}
		}

		// Find the world space bounds of each element and of all casters. Elements whose bounds cannot be known
		// (skinned, non-affine, or without a valid model bbox) are drawn into every view.
		auto caster_bounds = BBox::Reset();
		pr::vector<BBox> element_bounds;
		{
			auto drawlist = m_drawlist.lock();
			element_bounds.reserve(drawlist->size());
			for (auto& dle : *drawlist)
			{
				auto const& nugget = *dle.m_nugget;
				auto const& instance = *dle.m_instance;
				auto const& mbox = nugget.m_model->m_bbox;
				auto i2w = GetO2W(instance);
				if (!mbox.valid() || !IsAffine(i2w))
				{
					element_bounds.push_back(BBox::Reset());
					continue;
				}

				// Skinned models can move outside their bind pose bounds, so they contribute to the caster bounds but are never culled
				auto bbox = i2w * mbox;
				Grow(caster_bounds, bbox);
				element_bounds.push_back(FindPose(instance) != nullptr ? BBox::Reset() : bbox);
			}
		}

		// Describe the scene camera. Directional light cascades cover depth ranges of this view, and the size of the
		// view of each point or spot light on screen chooses the resolution of its shadow views.
		auto const& cam = scn().m_cam;
		auto const tan_half_fovy = s_cast<float>(std::tan(cam.FovY() * 0.5));
		auto const camera = ShadowCamera{
			.m_c2w = cam.CameraToWorld(),
			.m_near = s_cast<float>(cam.Near(false)),
			.m_far = s_cast<float>(cam.Far(false)),
			.m_view_size = cam.Orthographic() ? cam.ViewRectAtDistance(cam.FocusDist()) : v2(2.0f * tan_half_fovy * s_cast<float>(cam.Aspect()), 2.0f * tan_half_fovy),
			.m_viewport_height = scn().m_viewport.Height,
			.m_orthographic = cam.Orthographic(),
		};

		// Choose views for the shadow-casting lights and pack them into the atlas
		BuildShadowViews(scn().ResolvedLights(), caster_bounds, camera, m_settings, m_views);

		// Decide which views need rendering this frame
		FindDirtyViews(element_bounds);
	}

	// Find the views that each element can see, and the views whose content has changed since they were last rendered
	void RenderSmap::FindDirtyViews(std::span<BBox const> element_bounds)
	{
		// Start each view's content hash from the view itself. A change of transform or atlas region changes the content.
		auto const view_count = isize(m_views.m_views);
		auto const all_views = view_count == 32 ? ~0U : (1U << view_count) - 1;
		std::array<uint64_t, MaxShadowViews> hashes;
		std::array<IRect, MaxShadowViews> rects;
		pr::vector<ShadowViewVolume, MaxShadowViews> volumes;
		for (int v = 0; v != view_count; ++v)
		{
			auto const& view = m_views.m_views[v];
			hashes[v] = HashSeed;
			HashMix(hashes[v], view.m_w2s);
			HashMix(hashes[v], view.m_atlas_rect);
			HashMix(hashes[v], view.m_clamp_depth);
			rects[v] = view.m_atlas_rect;

			// Clamped views also see casters in front of their near plane
			volumes.push_back(ShadowViewVolume(view.m_w2s, !view.m_clamp_depth));
		}

		// Mix a hash of each element into the hash of every view that can see it. Views that see elements drawn with a
		// custom vertex shader are always rendered, because their geometry can change without anything here changing.
		auto forced = uint32_t(0);
		auto drawlist = m_drawlist.lock();
		m_element_views.resize(0);
		for (int e = 0, eend = isize(*drawlist); e != eend; ++e)
		{
			auto const& dle = (*drawlist)[e];
			auto const& nugget = *dle.m_nugget;
			auto const& instance = *dle.m_instance;
			auto const& bounds = element_bounds[e];

			// Find the views that can see the element
			auto mask = uint32_t(0);
			if (!bounds.valid())
			{
				mask = all_views;
			}
			else
			{
				auto const lower = bounds.Lower();
				auto const upper = bounds.Upper();
				for (int v = 0; v != view_count; ++v)
				{
					if (volumes[v].Sees(lower, upper))
						mask |= 1U << v;
				}
			}
			m_element_views.push_back(mask);
			if (mask == 0)
				continue;

			// Hash everything that can change the depth this element writes into a view
			auto h = HashSeed;
			HashMix(h, &instance);
			HashMix(h, &nugget);
			HashMix(h, GetO2W(instance));
			HashMix(h, nugget.m_vrange);
			HashMix(h, nugget.m_irange);
			HashMix(h, nugget.m_model);
			HashMix(h, nugget.m_model->m_revision);
			HashMix(h, FindMaterial(instance).get());
			HashMix(h, &nugget.mat());
			if (auto pose = FindPose(instance); pose != nullptr)
			{
				HashMix(h, pose.get());
				HashMix(h, pose->m_time1);
				HashMix(h, pose->m_revision);
			}

			// Add the element to the content of each view that sees it
			for (int v = 0; v != view_count; ++v)
			{
				if ((mask & (1U << v)) != 0)
					HashMix(hashes[v], h);
			}

			if (m_volatile.contains(&nugget))
				forced |= mask;
		}

		// Compare with the content already in the atlas
		m_dirty = m_cache.Update({ rects.data(), s_cast<size_t>(view_count) }, { hashes.data(), s_cast<size_t>(view_count) }, m_settings.m_cache_views) | forced;
	}

	// Perform the render step
	void RenderSmap::Execute(Frame& frame)
	{
		// Nothing to render if no view has changed. Unchanged views keep their content in the atlas.
		if (m_dirty == 0)
			return;

		// Gather the views to render into a compact list, so that each batch is a contiguous range of the uploaded views
		pr::vector<ShadowView, MaxShadowViews> views;
		pr::vector<int, MaxShadowViews> view_index;
		for (int v = 0, vend = isize(m_views.m_views); v != vend; ++v)
		{
			if ((m_dirty & (1U << v)) == 0)
				continue;

			views.push_back(m_views.m_views[v]);
			view_index.push_back(v);
		}

		// Record into a new allocator for this frame
		m_cmd_list.Reset(frame.m_cmd_alloc_pool.Get());
		frame.m_main.push_back(m_cmd_list);

		// Bind the descriptor heaps
		auto des_heaps = { wnd().m_heap_view.get(), wnd().m_heap_samp.get() };
		m_cmd_list.SetDescriptorHeaps({ des_heaps.begin(), des_heaps.size() });

		// Bind the atlas as the depth target and clear only the regions being rendered
		auto& atlas = *m_atlas.get();
		BarrierBatch barriers(m_cmd_list);
		barriers.Transition(atlas.m_res.get(), D3D12_RESOURCE_STATE_DEPTH_WRITE);
		barriers.Commit();
		m_cmd_list.OMSetRenderTargets({}, false, &atlas.m_dsv.m_cpu);
		{
			pr::vector<D3D12_RECT, MaxShadowViews> clear_rects;
			for (auto const& view : views)
				clear_rects.push_back(D3D12_RECT{ view.m_atlas_rect.m_min.x, view.m_atlas_rect.m_min.y, view.m_atlas_rect.m_max.x, view.m_atlas_rect.m_max.y });

			m_cmd_list.ClearDepthStencilView(atlas.m_dsv.m_cpu, D3D12_CLEAR_FLAG_DEPTH, 1.0f, 0, clear_rects);
		}

		// Upload the transforms of the views being rendered, and the constants of every element that some view can see
		m_cmd_list.SetGraphicsRootSignature(m_shader.m_signature.get());
		auto views_gpu = UploadShadowViews(m_upload_buffer, { views.data(), views.size() }, m_settings);
		{
			auto drawlist = m_drawlist.lock();
			assert(m_element_views.size() == drawlist->size() && "Draw list changed between Prepare and Execute");
			auto elements_gpu = shaders::ShadowMap::UploadElements(m_upload_buffer, std::span{ *drawlist }, std::span{ m_element_views.data(), m_element_views.size() });
			m_shader.SetupFrame(m_cmd_list.get(), views_gpu, elements_gpu);
		}

		// Per-element projections use the scene camera
		auto const camera = CameraTransforms(scn().m_cam);

		// Remember the bound material descriptors so that consecutive elements sharing a texture or sampler skip redundant binds.
		// The draw list is sorted by texture, so most elements reuse the previous descriptors.
		Descriptor last_tex = {}, last_sam = {};

		// Render the views in batches, one viewport per view in the batch. Depth clamping is part of the pipeline state,
		// so a batch only contains views with the same clamp mode.
		auto const count = isize(views);
		for (int batch_beg = 0, batch_end = 0; batch_beg != count; batch_beg = batch_end)
		{
			// Extend the batch while the views share a clamp mode
			auto const clamp_depth = views[batch_beg].m_clamp_depth;
			for (batch_end = batch_beg + 1; batch_end != count && batch_end - batch_beg != ShadowViewBatchSize && views[batch_end].m_clamp_depth == clamp_depth; ++batch_end) {}

			// Set one viewport and scissor rect per view. The viewport index in the shader is relative to 'batch_beg'.
			{
				D3D12_VIEWPORT viewports[ShadowViewBatchSize];
				D3D12_RECT scissors[ShadowViewBatchSize];
				for (int i = batch_beg; i != batch_end; ++i)
				{
					auto const& rect = views[i].m_atlas_rect;
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
			assert(m_element_views.size() == drawlist->size() && "Draw list changed between Prepare and Execute");
			for (int e = 0, eend = isize(*drawlist); e != eend; ++e)
			{
				auto const& dle = (*drawlist)[e];
				auto const& nugget = *dle.m_nugget;
				auto const& instance = *dle.m_instance;
				auto const mask = m_element_views[e];

				// Find the views in this batch that can see the element
				pr::vector<uint32_t, ShadowViewBatchSize> batch_views;
				for (int i = batch_beg; i != batch_end; ++i)
				{
					if ((mask & (1U << view_index[i])) != 0)
						batch_views.push_back(s_cast<uint32_t>(i));
				}
				if (batch_views.empty())
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

				// Select this element's entry in the uploaded element constants table, then let the material bind per-draw resources and pipeline overrides
				shaders::ShadowMap::SetupElement(m_cmd_list.get(), e);
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
					.m_last_tex = &last_tex,
					.m_last_sam = &last_sam,
				};
				pass->Bind(ctx);
				pass->ApplyPipeline(ctx);

				// A root signature change invalidates the bound descriptor tables
				if (ctx.m_root_signature_changed)
				{
					last_tex = {};
					last_sam = {};
				}

				// Remember nuggets drawn with a custom vertex shader, because the view content hash cannot detect changes to their geometry
				if (desc.Get<EPipeState::VS>().pShaderBytecode != m_shader.m_code.VS.pShaderBytecode)
					m_volatile.insert(&nugget);

				// Clamped views keep casters in front of the near plane, at depth 0
				desc.Apply(PSO<EPipeState::DepthClipEnable>(clamp_depth ? FALSE : TRUE));

				// Bind the views and draw one instance per view
				m_shader.SetupDrawViews(m_cmd_list.get(), batch_views, batch_beg);
				m_cmd_list.SetPipelineState(m_pipe_state_pool.Get(desc));
				if (!nugget.m_irange.empty())
					m_cmd_list.DrawIndexedInstanced(s_cast<size_t>(nugget.m_irange.size()), batch_views.size(), s_cast<size_t>(nugget.m_irange.m_beg), 0, 0U);
				else
					m_cmd_list.DrawInstanced(s_cast<size_t>(nugget.m_vrange.size()), batch_views.size(), s_cast<size_t>(nugget.m_vrange.m_beg), 0U);
			}
		}

		// Make the atlas readable by the forward pass
		barriers.Transition(atlas.m_res.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE | D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE);
		barriers.Commit();

		// Close the command list now that the shadow views are rendered
		m_cmd_list.Close();
	}
}
