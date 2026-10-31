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
		// Mix one 64-bit word into a hash. The rotation carries high bits back into the low bits, so a change in any word keeps
		// affecting the whole hash as later words are mixed in.
		inline void HashWord(uint64_t& h, uint64_t word)
		{
			// Multiply by an odd constant, which is a bijection, so a single changed word always changes the hash
			h = std::rotl((h ^ word) * 0x9E3779B97F4A7C15ULL, 31);
		}

		// Mix the bytes of 'value' into a hash
		template <typename T> void HashMix(uint64_t& h, T const& value)
		{
			// Mix whole 64-bit words, then any remaining bytes
			auto const* bytes = reinterpret_cast<uint8_t const*>(&value);
			size_t i = 0;
			for (; i + sizeof(uint64_t) <= sizeof(T); i += sizeof(uint64_t))
			{
				// Copy the word because 'value' need not be 8-byte aligned
				uint64_t word;
				std::memcpy(&word, bytes + i, sizeof(word));
				HashWord(h, word);
			}
			if (i != sizeof(T))
			{
				// Zero-extend the tail bytes into one word
				uint64_t word = 0;
				std::memcpy(&word, bytes + i, sizeof(T) - i);
				HashWord(h, word);
			}
		}

		// Mix a pointer value into a hash
		inline void HashPtr(uint64_t& h, void const* ptr)
		{
			HashWord(h, reinterpret_cast<uintptr_t>(ptr));
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
		, m_casters()
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
		// Float depth keeps fine depth steps when a view's depth range is long, which happens because each view's range covers all shadow casters.
		ResourceFactory factory(rdr());
		auto td = ResDesc::Tex2D(Image(settings.m_atlas_size, settings.m_atlas_size, nullptr, DXGI_FORMAT_R32_TYPELESS), 1, EUsage::DepthStencil)
			.def_state(D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE | D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE)
			.clear(DXGI_FORMAT_D32_FLOAT, D3D12_DEPTH_STENCIL_VALUE{ .Depth = 1.0f, .Stencil = 0 });
		auto desc = TextureDesc(AutoId, td).srv_format(DXGI_FORMAT_R32_FLOAT).dsv_format(DXGI_FORMAT_D32_FLOAT).name("ShadowAtlas");
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
			.DSVFormat = DXGI_FORMAT_D32_FLOAT,
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

		// Find the world space bounds and the content hash of each element, and the bounds of all casters. Elements whose bounds
		// cannot be known (skinned, non-affine, or without a valid model bbox) are drawn into every view.
		auto caster_bounds = BBox::Reset();
		pr::vector<ElementInfo> elements;
		{
			auto drawlist = m_drawlist.lock();
			elements.resize(drawlist->size());
			for (int e = 0, eend = isize(*drawlist); e != eend; ++e)
			{
				auto const& dle = (*drawlist)[e];
				auto const& nugget = *dle.m_nugget;
				auto const& instance = *dle.m_instance;
				auto const& mbox = nugget.m_model->m_bbox;
				auto const& i2w = GetO2W(instance);
				auto const pose = FindPose(instance);
				auto& info = elements[e];

				// Hash everything that can change the depth this element writes into a view
				auto h = HashSeed;
				HashPtr(h, &instance);
				HashPtr(h, &nugget);
				HashMix(h, i2w);
				HashMix(h, nugget.m_vrange);
				HashMix(h, nugget.m_irange);
				HashPtr(h, nugget.m_model);
				HashWord(h, nugget.m_model->m_revision);
				HashPtr(h, FindMaterial(instance).get());
				HashPtr(h, &nugget.mat());
				if (pose != nullptr)
				{
					HashPtr(h, pose.get());
					HashWord(h, std::bit_cast<uint64_t>(pose->m_time1));
					HashWord(h, pose->m_revision);
				}
				info.m_hash = h;
				info.m_cull = false;
				if (!mbox.valid() || !IsAffine(i2w))
					continue;

				// Skinned models can move outside their bind pose bounds, so they contribute to the caster bounds but are never culled
				auto bbox = i2w * mbox;
				Grow(caster_bounds, bbox);
				if (pose != nullptr)
					continue;

				info.m_lower = bbox.Lower();
				info.m_upper = bbox.Upper();
				info.m_cull = true;
			}
		}

		// Find the active blocks and the content hash of each population. The blocks grow the caster bounds, so populations outside the camera view still cast shadows.
		auto const range_origin = scn().m_cam.CameraToWorld().pos;
		m_casters.resize(0);
		for (auto pop : scn().Populations())
		{
			// Populations whose LODs have no shadow-casting nuggets are not casters, so they do not grow the caster bounds
			auto const& proto = *pop->m_prototype.get();
			auto casts = false;
			for (auto const& lod : proto.m_lods)
				PopulationNuggets(*lod.m_model.get(), ERenderStep::ShadowMap, [&](Nugget const&) { casts = true; });

			if (!casts)
				continue;

			// Populations without active blocks draw nothing
			auto caster = PopulationCaster{ .m_pop = pop, .m_blocks = {}, .m_hash = HashSeed, .m_views = 0, .m_volatile = false };
			auto revision = pop->ActiveBlocks([&](BBox const& bbox)
			{
				// Record the block for view selection, and include it in the casters
				caster.m_blocks.push_back(bbox);
				Grow(caster_bounds, bbox);
			});
			if (caster.m_blocks.empty())
				continue;

			// Hash everything that can change the depth the population writes into a view. The revision covers blocks, instances, and payload.
			// The draw distance is measured from the scene camera, so the camera position matters only when the prototype has a draw distance.
			auto& h = caster.m_hash;
			HashPtr(h, pop.get());
			HashWord(h, revision);
			HashPtr(h, &proto);
			for (auto const& lod : proto.m_lods)
			{
				HashPtr(h, lod.m_model.get());
				HashWord(h, lod.m_model->m_revision);

				// Custom shadow vertex shaders can change the geometry without any change to the population, in the same way as for ordinary nuggets
				PopulationNuggets(*lod.m_model.get(), ERenderStep::ShadowMap, [&](Nugget const& nug)
				{
					if (PopulationVertexOverlay(nug.mat(), ERenderStep::ShadowMap) != nullptr)
						caster.m_volatile = true;
				});
			}
			if (proto.m_draw_distance > 0)
				HashMix(h, range_origin);

			m_casters.push_back(std::move(caster));
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
		FindDirtyViews(elements);
	}

	// Find the views that each element can see, and the views whose content has changed since they were last rendered
	void RenderSmap::FindDirtyViews(std::span<ElementInfo const> elements)
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
		auto const check_volatile = !m_volatile.empty();
		m_element_views.resize(0);
		for (int e = 0, eend = isize(*drawlist); e != eend; ++e)
		{
			auto const& info = elements[e];

			// Find the views that can see the element
			auto mask = uint32_t(0);
			if (!info.m_cull)
			{
				mask = all_views;
			}
			else
			{
				for (int v = 0; v != view_count; ++v)
				{
					if (volumes[v].Sees(info.m_lower, info.m_upper))
						mask |= 1U << v;
				}
			}
			m_element_views.push_back(mask);
			if (mask == 0)
				continue;

			// Add the element to the content of each view that sees it
			for (int v = 0; v != view_count; ++v)
			{
				if ((mask & (1U << v)) != 0)
					HashWord(hashes[v], info.m_hash);
			}

			if (check_volatile && m_volatile.contains((*drawlist)[e].m_nugget))
				forced |= mask;
		}

		// Add each population to the content of the views that can see one of its active blocks. As for elements, views that see a population
		// with a custom shadow vertex shader are always rendered.
		for (auto& caster : m_casters)
		{
			// Find the views that can see the population
			caster.m_views = 0;
			for (int v = 0; v != view_count; ++v)
			{
				if (std::ranges::any_of(caster.m_blocks, [&](BBox const& bbox) { return volumes[v].Sees(bbox.Lower(), bbox.Upper()); }))
				{
					caster.m_views |= 1U << v;
					HashWord(hashes[v], caster.m_hash);
				}
			}
			if (caster.m_volatile)
				forced |= caster.m_views;
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

		// Classify each population for each view being rendered that can see it. Classification is compute work, so record it before the depth pass.
		// Each classification supplies element constants for its (LOD, nugget) draws, in draw order, from the population element table.
		struct PopulationShadow
		{
			PopulationViewLists m_lists; // The classification for one view
			int m_view;                  // The index of the view in 'views'
			int m_first_element;         // The index of the element constants of 'm_lists.m_draws[0]' in the population element table
		};
		pr::vector<PopulationShadow> pop_shadows;
		pr::vector<DrawListElement> pop_dles;
		if (!m_casters.empty())
		{
			// Measure the draw distance from the scene camera, as the forward pass does. Each view slot keeps its own classification buffer.
			auto pix_cull = pix::EventScope<ID3D12GraphicsCommandList>(m_cmd_list.get(), 0xFF5AC8A0, "View3D::PopulationShadowCull");
			auto const range_origin = scn().m_cam.CameraToWorld().pos;
			for (auto& caster : m_casters)
			{
				for (int i = 0, iend = isize(views); i != iend; ++i)
				{
					// Only views that can see an active block need a classification
					if ((caster.m_views & (1U << view_index[i])) == 0)
						continue;

					auto key = HashSeed;
					HashPtr(key, this);
					HashWord(key, s_cast<uint64_t>(view_index[i]));
					auto desc = PopulationProjectionView(key, views[i].m_w2s, s_cast<float>(views[i].m_atlas_rect.SizeY()), range_origin);
					auto shadow = PopulationShadow{
						.m_lists = rdr().PopulationCulling().Cull(m_cmd_list, m_upload_buffer, m_gsync, *caster.m_pop.get(), desc, m_step_id),
						.m_view = i,
						.m_first_element = isize(pop_dles),
					};

					// The LOD proxy instances supply the element constants. Instance records place the vertices.
					for (auto const& draw : shadow.m_lists.m_draws)
						pop_dles.push_back(DrawListElement{ .m_sort_key = draw.m_nugget->m_sort_key, .m_nugget = draw.m_nugget, .m_instance = &draw.m_proxy->m_base });

					pop_shadows.push_back(std::move(shadow));
				}
			}
		}

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
		auto elements_gpu = D3D12_GPU_VIRTUAL_ADDRESS{};
		{
			auto drawlist = m_drawlist.lock();
			assert(m_element_views.size() == drawlist->size() && "Draw list changed between Prepare and Execute");
			elements_gpu = shaders::ShadowMap::UploadElements(m_upload_buffer, std::span{ *drawlist }, std::span{ m_element_views.data(), m_element_views.size() });
			m_shader.SetupFrame(m_cmd_list.get(), views_gpu, elements_gpu);
		}

		// Upload the element constants of the population draws. Every entry is drawn, so every entry is written.
		auto pop_elements_gpu = D3D12_GPU_VIRTUAL_ADDRESS{};
		if (!pop_dles.empty())
		{
			pr::vector<uint32_t> pop_mask(pop_dles.size(), ~0U);
			pop_elements_gpu = shaders::ShadowMap::UploadElements(m_upload_buffer, std::span{ pop_dles.data(), pop_dles.size() }, std::span{ pop_mask.data(), pop_mask.size() });
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

			// Draw the populations classified for the views in this batch. A classification belongs to one view, so its draws render into that view only.
			// The population element table replaces the draw list table for these draws, then the draw list table is restored for the next batch.
			if (std::ranges::any_of(pop_shadows, [&](PopulationShadow const& s) { return s.m_view >= batch_beg && s.m_view < batch_end; }))
			{
				m_shader.SetupFrame(m_cmd_list.get(), views_gpu, pop_elements_gpu);
				for (auto const& shadow : pop_shadows)
				{
					// Skip classifications for views outside this batch
					if (shadow.m_view < batch_beg || shadow.m_view >= batch_end)
						continue;

					auto const& lists = shadow.m_lists;
					auto const& pop = *lists.m_population;
					auto const view = s_cast<uint32_t>(shadow.m_view);
					for (int d = 0, dend = isize(lists.m_draws); d != dend; ++d)
					{
						// Filter out nuggets that can't cast shadows, in the same way as 'AddNuggets'
						auto const& draw = lists.m_draws[d];
						auto const& nugget = *draw.m_nugget;
						auto const& dle = pop_dles[shadow.m_first_element + d];
						if (AnySet(nugget.m_nflags, ENuggetFlag::ShadowCastExclude))
							continue;
						if (nugget.FillMode() != EFillMode::Default && nugget.FillMode() != EFillMode::Solid)
							continue;

						// The classification only includes nuggets with a pass for this step
						auto const& material = nugget.mat();
						auto const* pass = material.Pass(m_step_id);
						pr_assert(pass != nullptr && "Population draws only include nuggets with a shadow pass");

						// Set the pipeline state and geometry
						auto desc = m_default_pipe_state;
						desc.Apply(PSO<EPipeState::TopologyType>(To<D3D12_PRIMITIVE_TOPOLOGY_TYPE>(nugget.m_topo)));
						if (IsStripTopo(nugget.m_topo))
							desc.Apply(PSO<EPipeState::IBStripCutValue>(StripCutValue(nugget.m_model->m_ib_view.Format)));

						m_cmd_list.IASetPrimitiveTopology(nugget.m_topo);
						m_cmd_list.IASetVertexBuffers(0U, { &nugget.m_model->m_vb_view, 1 });
						m_cmd_list.IASetIndexBuffer(&nugget.m_model->m_ib_view);

						// Select the draw's element constants, then let the material bind per-draw resources and pipeline overrides
						shaders::ShadowMap::SetupElement(m_cmd_list.get(), shadow.m_first_element + d);
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
						if (ctx.m_root_signature_changed)
						{
							last_tex = {};
							last_sam = {};
						}

						// Procedural vertex shader overlays (accepted by population admission) place population vertices themselves, so they are kept.
						// The stock shadow vertex shader is swapped for its population variant.
						auto vs = desc.Get<EPipeState::VS>().pShaderBytecode;
						auto const* overlay = PopulationVertexOverlay(material, m_step_id);
						if (overlay == nullptr || vs != overlay->m_code.VS.pShaderBytecode)
						{
							// Only the stock shadow vertex shader has a population variant
							if (vs != shader_code::shadow_map_vs.pShaderBytecode)
								throw std::logic_error("Population shadow draw has a vertex shader without a population variant");

							desc.Apply(PSO<EPipeState::VS>(shader_code::shadow_map_population_vs));
						}
						desc.Apply(PSO<EPipeState::DepthClipEnable>(clamp_depth ? FALSE : TRUE));

						// Bind the instance records and visible lists. The payload slot needs a valid address, so it aliases the instance records when there is no payload.
						auto pop_draw = PopulationDraw{ .lod = s_cast<uint32_t>(draw.m_lod), .list_base = lists.m_list_base, .pad0 = 0, .pad1 = 0 };
						m_cmd_list.SetGraphicsRootShaderResourceView(shaders::smap::ERootParam::PopulationInstances, GpuAddress(pop.m_instances));
						m_cmd_list.SetGraphicsRootShaderResourceView(shaders::smap::ERootParam::PopulationView, GpuAddress(lists.m_scratch));
						m_cmd_list.SetGraphicsRootShaderResourceView(shaders::smap::ERootParam::PopulationPayload, pop.m_payload != nullptr ? GpuAddress(pop.m_payload) : GpuAddress(pop.m_instances));
						m_cmd_list.SetGraphicsRoot32BitConstants(shaders::smap::ERootParam::PopulationDraw, pop_draw);

						// Draw into the classification's view, taking the instance counts from its argument records
						m_shader.SetupDrawViews(m_cmd_list.get(), { &view, 1 }, batch_beg);
						m_cmd_list.SetPipelineState(m_pipe_state_pool.Get(desc));
						if (!nugget.m_irange.empty())
							m_cmd_list.ExecuteIndirect(rdr().PopulationCulling().DrawSignature(true), 1, lists.m_scratch, s_cast<UINT64>(draw.m_args_indexed));
						else
							m_cmd_list.ExecuteIndirect(rdr().PopulationCulling().DrawSignature(false), 1, lists.m_scratch, s_cast<UINT64>(draw.m_args_vertex));
					}
				}
				m_shader.SetupFrame(m_cmd_list.get(), views_gpu, elements_gpu);
			}
		}

		// Make the atlas readable by the forward pass
		barriers.Transition(atlas.m_res.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE | D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE);
		barriers.Commit();

		// Close the command list now that the shadow views are rendered
		m_cmd_list.Close();
	}
}
