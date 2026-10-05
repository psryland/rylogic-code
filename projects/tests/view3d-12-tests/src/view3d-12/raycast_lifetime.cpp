#include "forward.h"

namespace fade_tests
{
	namespace rdr = pr::rdr12;
	namespace gpu = pr::compute;

	// Fail a focused lifetime assertion without relying on an interactive Debug assertion dialog.
	void RequireLifetime(bool condition, char const* message)
	{
		if (!condition)
			throw std::runtime_error(message);
	}

	// A normal material that can abandon one recording after reserving several upload pages.
	struct RecordingFailureMaterial : rdr::MaterialSimple, rdr::MaterialPass
	{
		bool m_fail = false;
		mutable size_t m_reserved_pages = 0;

		// Wrap only the RayCast pass; other render steps retain their ordinary material behaviour.
		rdr::MaterialPass const* Pass(rdr::ERenderStep step) const override
		{
			switch (step)
			{
				case rdr::ERenderStep::RayCast: { return this; }
				default: { return MaterialSimple::Pass(step); }
			}
		}

		// Use the real material binding, then optionally throw before the command list can be submitted.
		void Bind(rdr::MaterialPassContext& ctx) const override
		{
			MaterialSimple::Pass(ctx.m_step_id)->Bind(ctx);
			if (m_fail)
			{
				// Each full-block allocation needs a separate page. No handles escape this failed recording.
				for (auto i = 0; i != 3; ++i)
				{
					auto allocation = ctx.m_upload.Alloc(ctx.m_upload.m_blk_size, 256);
					allocation.ptr<std::byte>()[0] = std::byte{0x5A};
				}
				m_reserved_pages = 0;
				for (auto const& page : ctx.m_upload.m_used)
				{
					if (page.m_sync_point > ctx.m_upload.m_gsync->LastAddedSyncPoint())
						++m_reserved_pages;
				}
				throw std::runtime_error("Intentional RayCast pre-submit material failure");
			}
		}

		// Preserve the real shader and pipeline setup on successful casts.
		void ApplyPipeline(rdr::MaterialPassContext& ctx) const override
		{
			MaterialSimple::Pass(ctx.m_step_id)->ApplyPipeline(ctx);
		}

		// Match the fixture's ordinary new allocation rather than a renderer allocator.
		void Delete() override
		{
			delete this;
		}
	};

	// Reject D3D12 errors/corruption, including messages emitted during dependent resource teardown.
	void CheckLifetimeDebug(ID3D12InfoQueue& info)
	{
		for (auto i = UINT64{}; i != info.GetNumStoredMessages(); ++i)
		{
			auto size = SIZE_T{};
			pr::Check(info.GetMessage(i, nullptr, &size));
			auto storage = std::vector<std::byte>(size);
			auto message = reinterpret_cast<D3D12_MESSAGE*>(storage.data());
			pr::Check(info.GetMessage(i, message, &size));
			if (message->Severity == D3D12_MESSAGE_SEVERITY_ERROR || message->Severity == D3D12_MESSAGE_SEVERITY_CORRUPTION)
				throw std::runtime_error(message->pDescription);
		}
	}

	void RayCastLifetimeNativeTests()
	{
		// All native resources belong to this executable's renderer, never the independently linked DLL.
		auto settings = rdr::RdrSettings(GetModuleHandleW(nullptr)).DebugLayer(true, true, false).DefaultAdapter();
		auto renderer = rdr::Renderer(settings);
		auto info = Microsoft::WRL::ComPtr<ID3D12InfoQueue>{};
		pr::Check(renderer.d3d()->QueryInterface(IID_PPV_ARGS(info.GetAddressOf())));
		info->ClearStoredMessages();
		std::cout << "RayCast lifetime: native D3D12 debug validation active" << std::endl;
		{
			// No swap chain, window frame, or window fence signal is needed for any of these casts.
			auto window = rdr::Window(renderer, rdr::WndSettings(nullptr, true, renderer.Settings()).Size(128, 128));
			auto scene = rdr::Scene(window, {});
			auto instance = rdr::ldraw::RdrInstance{};
			instance.m_i2w = pr::m4x4::Identity();
			instance.m_i2w.pos = pr::v4(0, 0, -10, 1);
			instance.m_colour = pr::Colour32White;
			auto material = pr::RefPtr<RecordingFailureMaterial>(new RecordingFailureMaterial, true);
			instance.m_material = material;
			{
				// Finish geometry creation before testing the independent RayCast timeline.
				auto factory = rdr::ResourceFactory(renderer);
				instance.m_model = rdr::ModelGenerator::Box(factory, 1.0f);
				factory.FlushToGpu(rdr::EGpuFlush::Block);
			}
			auto ray = rdr::HitTestRay{.m_ws_direction = pr::v4(0, 0, -1, 0), .m_snap_mode = rdr::ESnapMode::Faces};
			auto window_sync = window.m_gsync.LastAddedSyncPoint();
			auto completed_hits = 0;
			auto receive = [&](std::span<rdr::HitTestResult const> hits)
			{
				// Validate actual mapped intercepts, not just a returned fence or an invoked callback.
				RequireLifetime(!hits.empty(), "RayCast returned no completed intercepts");
				RequireLifetime(hits.front().m_instance == &instance.m_base, "RayCast returned the wrong native instance");
				RequireLifetime(std::abs(hits.front().m_distance - 9.0f) < 0.01f, "RayCast returned the wrong completed distance");
				++completed_hits;
			};
			{
				// A first-use recording failure must leave no future wait target, even when destruction is immediate.
				auto gsync = gpu::GpuSync(renderer.d3d());
				auto cast = rdr::RenderRayCast(scene, gsync, {});
				cast.SetRays({&ray, 1}, [](auto) { return true; });
				cast.AddInstance(instance);
				material->m_fail = true;
				auto rejected = false;
				try
				{
					cast.ExecuteImmediate(receive).get();
				}
				catch (std::runtime_error const& error)
				{
					RequireLifetime(std::string_view(error.what()) == "Intentional RayCast pre-submit material failure", "Unexpected first-use RayCast failure");
					rejected = true;
				}
				RequireLifetime(rejected && material->m_reserved_pages >= 3, "First-use material failure did not reserve multiple pages");
				RequireLifetime(cast.m_gsync.LastAddedSyncPoint() == 0, "Failed first-use RayCast submitted work");
				for (auto const& page : cast.m_upload_buffer.m_used)
				{
					RequireLifetime(page.m_sync_point == 0, "Failed first-use RayCast retained a future reservation");
				}
			}
			std::cout << "PASS first-use multi-page pre-submit failure and immediate teardown" << std::endl;
			{
				// Keep several readbacks pending, then abandon the next recording without touching prior retirement points.
				auto gsync = gpu::GpuSync(renderer.d3d());
				auto cast = rdr::RenderRayCast(scene, gsync, {});
				cast.SetRays({&ray, 1}, [](auto) { return true; });
				cast.AddInstance(instance);
				material->m_fail = false;
				cast.ExecuteAsync(receive);
				cast.ExecuteAsync(receive);
				auto last_submitted = cast.m_gsync.LastAddedSyncPoint();
				auto completed_before_failure = cast.m_gsync.CompletedSyncPoint();
				auto& shared_page = cast.m_upload_buffer.m_used.back();
				auto retained_upload = gpu::GpuTransferAllocation(&shared_page, shared_page.m_res.get(), shared_page.m_mem, 0, shared_page.m_size);
				RequireLifetime(last_submitted == 2 && completed_hits == 0, "Expected two submitted casts with undelivered readbacks");
				material->m_fail = true;
				auto rejected = false;
				try
				{
					cast.ExecuteAsync(receive);
				}
				catch (std::runtime_error const& error)
				{
					RequireLifetime(std::string_view(error.what()) == "Intentional RayCast pre-submit material failure", "Unexpected RayCast recording failure");
					rejected = true;
				}
				RequireLifetime(rejected && material->m_reserved_pages >= 3, "Material failure did not exercise multi-page cancellation");
				RequireLifetime(cast.m_gsync.LastAddedSyncPoint() == last_submitted, "Cancellation changed the submitted timeline");
				RequireLifetime(retained_upload.m_page->m_sync_point == last_submitted, "Cancellation retired a shared submitted page before its fence");
				for (auto const& page : cast.m_upload_buffer.m_used)
				{
					RequireLifetime(page.m_sync_point <= last_submitted, "Cancellation retained a future upload wait");
				}
				std::cout << "RayCast cancellation: last_added=" << last_submitted
					<< " completed_before_failure=" << completed_before_failure
					<< " completed_after_failure=" << cast.m_gsync.CompletedSyncPoint()
					<< " shared_page_target=" << retained_upload.m_page->m_sync_point
					<< " cancelled_page_count=" << material->m_reserved_pages << std::endl;

				// Successful reuse must still produce all earlier results, then the retry's actual readback.
				material->m_fail = false;
				cast.ExecuteImmediate(receive).get();
				cast.m_gsync.Poll();
				RequireLifetime(completed_hits == 3, "Cancellation lost earlier results or the retry's completed readback");
				RequireLifetime(window.m_gsync.LastAddedSyncPoint() == window_sync, "RayCast recovery advanced the unrelated window fence");
			}
			std::cout << "PASS prior pending submissions, multi-page cancellation, and completed retry" << std::endl;
			{
				// Destruction waits submitted work but deliberately discards pending callbacks before releasing readbacks.
				auto gsync = gpu::GpuSync(renderer.d3d());
				auto cast = rdr::RenderRayCast(scene, gsync, {});
				cast.SetRays({&ray, 1}, [](auto) { return true; });
				cast.AddInstance(instance);
				cast.ExecuteAsync(receive);
				cast.ExecuteAsync(receive);
			}
			RequireLifetime(completed_hits == 3, "RayCast teardown unexpectedly invoked pending callbacks");
			RequireLifetime(window.m_gsync.LastAddedSyncPoint() == window_sync, "Async teardown advanced the unrelated window fence");
			std::cout << "PASS multiple pending readbacks and immediate asynchronous teardown" << std::endl;

			// The scene API must clear transient drawlists even when recording fails, and keep its picking timelines separate.
			scene.AddInstance(instance);
			material->m_fail = true;
			auto scene_rejected = false;
			try
			{
				scene.HitTest({&ray, 1}, {}, receive).get();
			}
			catch (std::runtime_error const& error)
			{
				RequireLifetime(std::string_view(error.what()) == "Intentional RayCast pre-submit material failure", "Unexpected scene hit-test failure");
				scene_rejected = true;
			}
			RequireLifetime(scene_rejected && scene.m_raycast_immed->m_drawlist.lock()->empty(), "Scene retained its failed transient drawlist");
			material->m_fail = false;
			scene.HitTest({&ray, 1}, {}, receive).get();
			scene.HitTestAsync({&ray, 1});
			RequireLifetime(scene.m_gsync_immed.LastAddedSyncPoint() == 1 && scene.m_gsync_async.LastAddedSyncPoint() == 1, "Scene picking timelines were coupled");
			RequireLifetime(&scene.m_raycast_immed->m_gsync == &scene.m_gsync_immed && &scene.m_raycast_async->m_gsync == &scene.m_gsync_async, "Scene did not supply the picking fences");
			scene.m_raycast_async.reset();
			scene.m_raycast_immed.reset();
			scene.ClearDrawlists();
			RequireLifetime(completed_hits == 4, "Scene retry did not return exactly one completed result");
			std::cout << "PASS scene retry cleanup and distinct immediate/async fence lifetimes" << std::endl;

			// Each configured RayCast is a self-submitter, including duplicate entries and repeated pipeline replacements.
			for (auto iteration = 0; iteration != 2; ++iteration)
			{
				auto steps = std::array{rdr::ERenderStep::RayCast, rdr::ERenderStep::RayCast};
				scene.SetRenderSteps(steps);
				RequireLifetime(scene.m_gsync_render_steps.size() == 2, "Configured RayCast fences were not allocated per step");
				auto& first = static_cast<rdr::RenderRayCast&>(*scene.m_render_steps[0]);
				auto& second = static_cast<rdr::RenderRayCast&>(*scene.m_render_steps[1]);
				RequireLifetime(&first.m_gsync != &second.m_gsync && &first.m_gsync != &window.m_gsync, "Configured RayCasts share a reservation domain");
				RequireLifetime(&first.m_gsync != &scene.m_gsync_immed && &second.m_gsync != &scene.m_gsync_async, "Configured RayCast shares a picking timeline");
				scene.AddInstance(instance);
				first.SetRays({&ray, 1}, [](auto) { return true; });
				second.SetRays({&ray, 1}, [](auto) { return true; });
				first.ExecuteAsync(receive);
				RequireLifetime(first.m_gsync.LastAddedSyncPoint() == 1 && second.m_gsync.LastAddedSyncPoint() == 0, "First configured RayCast advanced another timeline");
				second.ExecuteAsync(receive);
				RequireLifetime(second.m_gsync.LastAddedSyncPoint() == 1, "Second configured RayCast did not submit independently");
				scene.SetRenderSteps({});
				RequireLifetime(scene.m_gsync_render_steps.empty(), "Pipeline replacement retained old fence generations");
				scene.ClearDrawlists();
				renderer.Poll();
			}
			RequireLifetime(completed_hits == 4, "Replacement invoked discarded RayCast callbacks");
			std::cout << "PASS configured duplicate RayCasts and repeated pending-work replacement" << std::endl;

			// A constructor failure after creating a configured RayCast must destroy it before its fence storage.
			auto constructor_rejected = false;
			try
			{
				auto rejected_scene = rdr::Scene(window, {rdr::ERenderStep::RayCast, rdr::ERenderStep::Invalid});
			}
			catch (std::runtime_error const& error)
			{
				RequireLifetime(std::string_view(error.what()) == "Unknown render step", "Unexpected scene constructor failure");
				constructor_rejected = true;
			}
			RequireLifetime(constructor_rejected, "Scene constructor failure was not exercised");
			renderer.Poll();
			std::cout << "PASS configured RayCast constructor-unwind and callback removal" << std::endl;
			{
				// A one-bone triangle moves four world units in depth. Warm its real GPU skin cache, then abandon the
				// changed pose recording and retry at precisely the same animation time.
				auto skinned = rdr::ldraw::RdrInstance{};
				skinned.m_i2w = instance.m_i2w;
				skinned.m_colour = pr::Colour32White;
				skinned.m_material = material;
				{
					auto factory = rdr::ResourceFactory(renderer);
					auto bone_ids = std::array<uint32_t, 1>{0};
					auto names = std::array<rdr::string32, 1>{"root"};
					auto inverse_bind = std::array{pr::m4x4::Identity()};
					auto hierarchy = std::array<uint8_t, 1>{0};
					auto skeleton = rdr::SkeletonPtr(gpu::New<rdr::Skeleton>(0xD034, bone_ids, names, inverse_bind, hierarchy), true);
					auto animation = rdr::KeyFrameAnimationPtr(gpu::New<rdr::KeyFrameAnimation>(skeleton->Id(), 1.0, 1.0), true);
					animation->m_bone_map = {0};
					animation->m_position = {pr::v3(0, 0, 0), pr::v3(0, 0, -4)};
					auto animator = rdr::AnimatorPtr(gpu::New<rdr::Animator_KeyFrameAnimation>(animation), true);
					skinned.m_pose = rdr::PosePtr(gpu::New<rdr::Pose>(factory, skeleton, animator, rdr::EAnimStyle::Once, rdr::EAnimFlags::None, rdr::TimeRange(0, 1), 1.0, 0.0), true);
					auto vertices = std::array<rdr::Vert, 3>{};
					rdr::SetPCNTI(vertices[0], pr::v4(-2, -2, 0, 1), pr::ColourWhite, pr::v4(0, 0, 1, 0), {}, {0, 0});
					rdr::SetPCNTI(vertices[1], pr::v4(+2, -2, 0, 1), pr::ColourWhite, pr::v4(0, 0, 1, 0), {}, {0, 0});
					rdr::SetPCNTI(vertices[2], pr::v4(0, +2, 0, 1), pr::ColourWhite, pr::v4(0, 0, 1, 0), {}, {0, 0});
					uint16_t indices[] = {0, 1, 2};
					skinned.m_model = factory.CreateModel(rdr::ModelDesc().vbuf(std::span<rdr::Vert const>(vertices)).ibuf(indices).bbox(pr::BBox(pr::v4::Origin(), pr::v4(2, 2, 0, 0))));
					skinned.m_model->CreateNugget(factory, rdr::NuggetDesc(rdr::ETopo::TriList, rdr::EGeom::Vert | rdr::EGeom::Norm).pso<rdr::EPipeState::CullMode>(D3D12_CULL_MODE_NONE));
					auto influence = rdr::Skinfluence{.m_bones = {0}, .m_weights = {65535}};
					skinned.m_model->m_skin = rdr::Skin(factory, {&influence, 1}, skeleton->Id(), 0, 0);
					factory.FlushToGpu(rdr::EGpuFlush::Block);
				}
				auto gsync = gpu::GpuSync(renderer.d3d());
				auto cast = rdr::RenderRayCast(scene, gsync, {});
				cast.SetRays({&ray, 1}, [](auto) { return true; });
				cast.AddInstance(skinned);
				auto expected_distance = 10.0f;
				auto skin_hits = 0;
				auto receive_skin = [&](std::span<rdr::HitTestResult const> hits)
				{
					RequireLifetime(!hits.empty() && hits.front().m_instance == &skinned.m_base, "Skinned RayCast returned no matching completed intercept");
					RequireLifetime(std::abs(hits.front().m_distance - expected_distance) < 0.01f, "Skinned retry reused an abandoned pose or skin-cache result");
					++skin_hits;
				};
				cast.ExecuteImmediate(receive_skin).get();
				RequireLifetime(skin_hits == 1, "Skinned cache warmup did not return a real hit");
				skinned.m_pose->AnimTime(1.0);
				material->m_fail = true;
				auto skin_rejected = false;
				try
				{
					cast.ExecuteImmediate(receive_skin).get();
				}
				catch (std::runtime_error const& error)
				{
					RequireLifetime(std::string_view(error.what()) == "Intentional RayCast pre-submit material failure", "Unexpected skinned recording failure");
					skin_rejected = true;
				}
				auto failed_revision = skinned.m_pose->Revision();
				RequireLifetime(skin_rejected && skin_hits == 1 && gsync.LastAddedSyncPoint() == 1, "Failed skinned recording was submitted or reported results");
				RequireLifetime(skinned.m_pose->m_time0 == 1.0 && skinned.m_pose->m_time1 == 1.0 && skinned.m_pose->m_gpu_update_required, "Failed pose was not invalidated at unchanged animation time");
				material->m_fail = false;
				expected_distance = 14.0f;
				cast.ExecuteImmediate(receive_skin).get();
				RequireLifetime(skin_hits == 2 && skinned.m_pose->Revision() > failed_revision && !skinned.m_pose->m_gpu_update_required, "Skinned retry did not re-upload and recompute");
				std::cout << "PASS skinned failure/retry: unchanged time=1, actual GPU hit depth 10 -> 14" << std::endl;
			}
			RequireLifetime(window.m_gsync.LastAddedSyncPoint() == window_sync, "Lifetime regressions advanced the unrelated window fence");

			// Poll after every cast has been destroyed; removed registrations must not dereference dead fences.
			renderer.Poll();
		}
		CheckLifetimeDebug(*info.Get());
		info.Reset();
		std::cout << "PASS RayCast lifetime debug validation" << std::endl;
	}
}
