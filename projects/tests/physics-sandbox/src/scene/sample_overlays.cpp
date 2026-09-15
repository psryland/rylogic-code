#include "src/scene/sample_overlays.h"

namespace physics_sandbox
{
	namespace
	{
		// Reject unsupported children before constructing a partial or misleading union visualization.
		void ValidatePrimitive(collision::Shape const& shape)
		{
			switch (shape.m_type)
			{
				case collision::EShape::Box:
				case collision::EShape::Sphere:
				case collision::EShape::Triangle:
				case collision::EShape::Polytope:
				{
					return;
				}
				default:
				{
					throw std::runtime_error(std::format("unsupported collision shape type {} (box/sphere/triangle/polytope only)", static_cast<int>(shape.m_type)));
				}
			}
		}

		// Ensure renderer vertex/index counts remain representable; reject rather than decimate the cloud.
		void ValidatePointCount(size_t count)
		{
			if (count > static_cast<size_t>(std::numeric_limits<int>::max()) / (3 * sizeof(rdr12::Vert)))
				throw std::runtime_error("sample overlay exceeds renderer buffer addressing");
		}

		// Instantiate shared models with independent transforms and depth-independent overlay visibility.
		rdr12::ldraw::LdrObjectPtr InstanceOf(rdr12::ldraw::LdrObjectPtr const& model)
		{
			if (!model)
				return nullptr;

			auto instance = rdr12::ldraw::CreateInstance(model.get());
			instance->Apply([](auto* object)
			{
				object->Colour(false, object->Colour(true));
				return true;
			}, "");
			instance->Flags(rdr12::ldraw::ELdrFlags::NoZTest, true, "");
			instance->Flags(rdr12::ldraw::ELdrFlags::NoZWrite, true, "");
			instance->SortGroup(rdr12::ESortGroup::PostAlpha, "");
			return instance;
		}

		// Upload the complete cached shape cloud once, keeping surface and volume models independently selectable.
		SampleOverlays::Model BuildModels(rdr12::Renderer& renderer, SampleOverlayGeometry const& geometry)
		{
			auto model = SampleOverlays::Model{};
			if (!geometry.m_surface.empty())
			{
				ldraw::Builder builder;
				auto& group = builder.Group("surface_samples");
				auto& points = group.Point("samples", 0xFFFFC040U).size(3.0f);
				auto& normals = group.Line("normals", 0xFF40FF80U);
				for (auto const& sample : geometry.m_surface)
				{
					points.pt(sample.m_pos_local);
					normals.line(sample.m_pos_local, sample.m_pos_local + SampleOverlays::NormalLength * sample.m_normal_local);
				}
				auto parsed = rdr12::ldraw::Parse(renderer, builder.ToBinary());
				if (parsed.m_objects.empty())
					throw std::runtime_error("surface overlay model creation produced no object");

				model.m_surface = parsed.m_objects.front();
			}
			if (!geometry.m_volume.empty())
			{
				ldraw::Builder builder;
				auto& points = builder.Point("volume_samples", 0xFF40C0FFU).size(2.0f);
				for (auto const& point : geometry.m_volume)
					points.pt(point);

				auto parsed = rdr12::ldraw::Parse(renderer, builder.ToBinary());
				if (parsed.m_objects.empty())
					throw std::runtime_error("volume overlay model creation produced no object");

				model.m_volume = parsed.m_objects.front();
			}
			return model;
		}
	}

	SampleOverlayGeometry SampleOverlays::BuildGeometry(collision::Shape const& shape, bool surface, bool volume)
	{
		using namespace physics::buoyancy;
		auto geometry = SampleOverlayGeometry{};
		if (!surface && !volume)
			return geometry;

		// Child transforms already target the shape root; the array's transform must not be applied a second time.
		auto const primitives = CollectPrimitives(shape);
		auto root_to_shape = std::vector<m4x4>{};
		for (auto const* primitive : primitives)
		{
			ValidatePrimitive(*primitive);
			root_to_shape.push_back(InvertOrthonormal(primitive->m_s2r));
		}
		auto const extent = MaxElement(collision::CalcBBox(shape).m_radius.w0());
		auto const epsilon = std::max(1e-6f, extent * 1e-5f);

		// Preserve all incident face contributions that survive the existing union-boundary ownership test.
		if (surface)
		{
			auto plans = std::vector<physics::surface::Plan>{};
			auto count = size_t{0};
			for (auto const* primitive : primitives)
			{
				plans.push_back(physics::surface::BuildPlan(*primitive, SurfaceSpacing));
				count += plans.back().m_count;
				ValidatePointCount(count);
			}
			geometry.m_surface.reserve(count);
			for (size_t k = 0; k != primitives.size(); ++k)
			{
				for (uint32_t i = 0; i != plans[k].m_count; ++i)
				{
					auto sample = physics::surface::EmitSurfaceSample(plans[k], i);
					sample.m_pos_local = primitives[k]->m_s2r * sample.m_pos_local;
					sample.m_normal_local = Normalise((primitives[k]->m_s2r * sample.m_normal_local).w0(), v4::Zero());
					auto const probe = sample.m_pos_local + epsilon * sample.m_normal_local;
					auto culled = false;
					for (size_t j = 0; j != primitives.size() && !culled; ++j)
					{
						if (j == k)
							continue;

						culled = ContainsLocal(*primitives[j], root_to_shape[j] * probe, epsilon) ||
							ContainsLocal(*primitives[j], root_to_shape[j] * sample.m_pos_local, -epsilon);
					}
					if (!culled)
						geometry.m_surface.push_back(sample);
				}
			}
		}

		// Missing polytope volume geometry is derived once with the existing exact face-fan routine, never per frame.
		if (volume)
		{
			auto derived = std::vector<byte_data<16>>(primitives.size());
			auto volume_shapes = primitives;
			auto tables = std::vector<VolumeSampleTable>(primitives.size());
			auto measures = std::vector<float>(primitives.size());
			for (size_t k = 0; k != primitives.size(); ++k)
			{
				switch (primitives[k]->m_type)
				{
					case collision::EShape::Polytope:
					{
						auto const& poly = collision::shape_cast<collision::ShapePolytope>(*primitives[k]);
						if (poly.m_tet_count == 0 || poly.m_volume_vert_count == 0)
						{
							derived[k] = collision::BuildPolytopeFromPoints(poly.verts(), poly.m_base.m_s2r, poly.m_base.m_material_id, poly.m_base.m_flags, -1);
							volume_shapes[k] = &derived[k].as<collision::Shape>();
						}
						break;
					}
					case collision::EShape::Box:
					case collision::EShape::Sphere:
					case collision::EShape::Triangle:
					{
						break;
					}
					default:
					{
						throw std::runtime_error("Unexpected volume overlay primitive");
					}
				}
				tables[k] = BuildVolumeSampleTable(*volume_shapes[k]);
				measures[k] = tables[k].m_total;
			}
			auto const counts = DistributeCounts(measures, VolumeSampleCount);
			auto count = size_t{0};
			for (auto n : counts)
				count += n;

			ValidatePointCount(count);
			geometry.m_volume.reserve(count);
			for (size_t k = 0; k != primitives.size(); ++k)
			{
				for (int i = 0; i != counts[k]; ++i)
				{
					auto const sample = EmitVolumeSample(*volume_shapes[k], SampleIndex(0, static_cast<int>(k), i), measures[k] / counts[k], tables[k]);
					auto const point = volume_shapes[k]->m_s2r * sample.m_pos_local;
					auto culled = false;
					for (size_t j = 0; j != k && !culled; ++j)
						culled = ContainsLocal(*primitives[j], root_to_shape[j] * point, epsilon);

					if (!culled)
						geometry.m_volume.push_back(point);
				}
			}
		}
		return geometry;
	}

	bool SampleOverlays::Eligible(physics::RigidBody const& body)
	{
		return body.HasShape() && body.InvMass() > 0.0f && !AllSet(body.StateFlags(), physics::ERigidBodyStateFlags::Static);
	}

	bool SampleOverlays::Eligible(physics::Articulation const& articulation, physics::LinkHandle link)
	{
		switch (articulation.RootType())
		{
			case physics::EArticulationRootType::Floating:
			{
				return true;
			}
			case physics::EArticulationRootType::Fixed:
			{
				for (; link != articulation.Root(); link = articulation.Parent(link))
				{
					if (articulation.JointDofCount(link) != 0)
						return true;
				}
				return false;
			}
			default:
			{
				throw std::runtime_error("Unknown articulation root type");
			}
		}
	}

	bool SampleOverlays::Enabled() const
	{
		return m_surface_enabled || m_volume_enabled;
	}

	void SampleOverlays::Surface(bool enabled)
	{
		if (m_surface_enabled != enabled)
			Invalidate();

		m_surface_enabled = enabled;
	}

	void SampleOverlays::Volume(bool enabled)
	{
		if (m_volume_enabled != enabled)
			Invalidate();

		m_volume_enabled = enabled;
	}

	void SampleOverlays::Invalidate()
	{
		m_refresh_pending = true;
	}

	void SampleOverlays::Reset()
	{
		m_instances.clear();
		m_models.clear();
		m_failed_targets = 0;
		m_first_error.clear();
		m_refresh_pending = false;
	}

	void SampleOverlays::BeginFrame()
	{
		assert(!m_refresh_pending && "Refresh sample overlay resources only after clearing renderer draw lists");
		m_failed_targets = 0;
		m_first_error.clear();
	}

	void SampleOverlays::Add(rdr12::Scene& scene, rdr12::Renderer& renderer, void const* target, collision::Shape const& shape, m4x4 const& root_to_world)
	{
		// Cache both successful models and explicit failures, so unchanged unsupported shapes never rebuild every frame.
		auto [iter, inserted] = m_models.try_emplace(&shape);
		auto& model = iter->second;
		if (inserted)
		{
			try
			{
				model = BuildModels(renderer, BuildGeometry(shape, m_surface_enabled, m_volume_enabled));
			}
			catch (std::exception const& ex)
			{
				model.m_error = ex.what();
			}
		}
		if (!model.m_error.empty())
		{
			++m_failed_targets;
			if (m_first_error.empty())
				m_first_error = model.m_error;

			return;
		}

		// Retain separate object instances for all targets until a renderer-safe reset, including offscreen/sleeping bodies.
		auto& instance = m_instances[target];
		if (instance.m_shape && instance.m_shape != &shape)
			throw std::runtime_error("Sample overlay shape changed without renderer-safe invalidation");

		if (!instance.m_shape)
		{
			instance.m_shape = &shape;
			instance.m_surface = InstanceOf(model.m_surface);
			instance.m_volume = InstanceOf(model.m_volume);
		}
		if (m_surface_enabled && instance.m_surface)
		{
			instance.m_surface->O2W(root_to_world);
			instance.m_surface->AddToScene(scene);
		}
		if (m_volume_enabled && instance.m_volume)
		{
			instance.m_volume->O2W(root_to_world);
			instance.m_volume->AddToScene(scene);
		}
	}
}

#if PR_UNITTESTS
namespace physics_sandbox::tests
{
	// Geometry, selection and renderer-instance contracts for the two additive sample overlays.
	PRUnitTestClass(SampleOverlayTests)
	{
		// Disabling overlays must not inspect unsupported geometry or allocate sample clouds.
		PRUnitTestMethod(IndependentOptionsAndDisabledGeneration, Quick)
		{
			auto overlays = SampleOverlays{};
			auto const line = collision::ShapeLine(1.0f);
			auto const empty = overlays.BuildGeometry(line, false, false);
			PR_EXPECT(empty.m_surface.empty() && empty.m_volume.empty());
			PR_EXPECT(!overlays.Enabled() && !overlays.m_refresh_pending);
			overlays.Surface(true);
			overlays.Volume(true);
			PR_EXPECT(overlays.m_surface_enabled && overlays.m_volume_enabled && overlays.m_refresh_pending);
			overlays.Reset();
			PR_EXPECT(overlays.Enabled() && !overlays.m_refresh_pending);
			overlays.Surface(false);
			PR_EXPECT(!overlays.m_surface_enabled && overlays.m_volume_enabled);
			overlays.Volume(false);
			PR_EXPECT(!overlays.Enabled());
			PR_THROWS(overlays.BuildGeometry(line, true, false), std::runtime_error);
			PR_THROWS(overlays.BuildGeometry(line, false, true), std::runtime_error);
		}

		// Sleeping is neither a selection filter nor a side effect of inspecting the shape.
		PRUnitTestMethod(DynamicSleepingBodiesButNotStaticGround, Quick)
		{
			auto const box = collision::ShapeBox(v4{1, 1, 1, 0});
			auto body = physics::RigidBody(&box.m_base, m4x4::Identity(), physics::Inertia::Box(v4{0.5f, 0.5f, 0.5f, 0}, 1.0f));
			auto const ground = physics::RigidBody(&box.m_base, m4x4::Identity(), physics::Inertia::Infinite());
			auto const shapeless = physics::RigidBody(nullptr, m4x4::Identity(), physics::Inertia::Sphere(0.5f, 1.0f));
			PR_EXPECT(SampleOverlays::Eligible(body));
			body.Sleeping(true);
			auto const force = body.ForceWS();
			auto const geometry = SampleOverlays::BuildGeometry(body.Shape(), true, true);
			PR_EXPECT(SampleOverlays::Eligible(body) && body.Sleeping());
			PR_EXPECT(FEql(body.ForceWS(), force));
			PR_EXPECT(!SampleOverlays::Eligible(ground) && !SampleOverlays::Eligible(shapeless));
			PR_EXPECT(geometry.m_volume.size() == SampleOverlays::VolumeSampleCount);
			PR_EXPECT(geometry.m_surface.size() == physics::surface::BuildPlan(box).m_count);
		}

		// Movable descendants remain eligible even when their own joint is fixed and the tree sleeps.
		PRUnitTestMethod(ArticulationMobilityIncludesFixedDescendantsOfMovingJoints, Quick)
		{
			auto const link = physics::ArticulationLinkDesc{.m_inertia = physics::Inertia::Sphere(0.2f, 1.0f)};
			auto builder = physics::ArticulationBuilder{};
			auto const root = builder.AddFixedRoot(link);
			auto const fixed = builder.AddLink(root, physics::ArticulationJointDesc::Fixed(), link);
			auto const moving = builder.AddLink(fixed, physics::ArticulationJointDesc::Revolute(), link);
			auto const carried = builder.AddLink(moving, physics::ArticulationJointDesc::Fixed(), link);
			auto tree = builder.Build();
			tree.Sleeping(true);
			PR_EXPECT(!SampleOverlays::Eligible(tree, root) && !SampleOverlays::Eligible(tree, fixed));
			PR_EXPECT(SampleOverlays::Eligible(tree, moving) && SampleOverlays::Eligible(tree, carried));
			PR_EXPECT(tree.Sleeping());

			// A floating root makes its complete rigidly attached subtree movable.
			auto floating_builder = physics::ArticulationBuilder{};
			auto const floating_root = floating_builder.AddFloatingRoot(link);
			auto floating = floating_builder.Build();
			PR_EXPECT(SampleOverlays::Eligible(floating, floating_root));
		}

		// Sample positions receive the child-to-root transform exactly once, including all sharp corner normals.
		PRUnitTestMethod(TransformedCompoundAndIncidentNormals, Quick)
		{
			// Collision arrays store child transforms in root space rather than parent-array space.
			struct Compound
			{
				collision::ShapeArray m_array{m4x4::Translation(100.0f, 0.0f, 0.0f)};
				collision::ShapeBox m_a{v4{0.2f, 0.4f, 0.6f, 0}, m4x4::Transform(RotationRad<m3x3>(0.2f, 0.4f, 0.7f), v4{-2, 0, 0, 1})};
				collision::ShapeBox m_b{v4{0.2f, 0.4f, 0.6f, 0}, m4x4::Translation(2.0f, 0.0f, 0.0f)};
			} compound;
			compound.m_array.Complete(2);
			auto const geometry = SampleOverlays::BuildGeometry(compound.m_array, true, true);
			auto const plan = physics::surface::BuildPlan(compound.m_a);
			PR_EXPECT(geometry.m_surface.size() == 2 * plan.m_count);
			PR_EXPECT(geometry.m_volume.size() == SampleOverlays::VolumeSampleCount);
			auto const root_to_world = m4x4::Transform(RotationRad<m3x3>(0.3f, 0.1f, 0.5f), v4{3, 5, 7, 1});
			auto corner_normals = 0;
			for (uint32_t i = 0; i != plan.m_count; ++i)
			{
				auto const expected = physics::surface::EmitSurfaceSample(plan, i);
				auto const& actual = geometry.m_surface[i];
				PR_EXPECT(FEql(actual.m_pos_local, compound.m_a.m_base.m_s2r * expected.m_pos_local));
				PR_EXPECT(FEql(actual.m_normal_local, compound.m_a.m_base.m_s2r * expected.m_normal_local));
				PR_EXPECT(FEql(root_to_world * actual.m_pos_local, (root_to_world * compound.m_a.m_base.m_s2r) * expected.m_pos_local));
				PR_EXPECT(FEql(root_to_world * actual.m_normal_local, (root_to_world * compound.m_a.m_base.m_s2r) * expected.m_normal_local));
				if (FEql(expected.m_pos_local, v4{0.1f, 0.2f, 0.3f, 1}))
					++corner_normals;
			}
			PR_EXPECT(corner_normals == 3);

			// Keep the existing primitive allocation, hashed index and volume emitter, not a second point distribution.
			auto const table = physics::buoyancy::BuildVolumeSampleTable(compound.m_a);
			for (int i = 0; i != SampleOverlays::VolumeSampleCount / 2; ++i)
			{
				auto const sample = physics::buoyancy::EmitVolumeSample(compound.m_a, physics::buoyancy::SampleIndex(0, 0, i), table.m_total / 4096, table);
				PR_EXPECT(FEql(geometry.m_volume[i], compound.m_a.m_base.m_s2r * sample.m_pos_local));
			}
		}

		// An enclosed child must not appear as an exposed surface or a second set of interior ownership samples.
		PRUnitTestMethod(CompoundUnionOwnership, Quick)
		{
			// The first primitive owns the complete overlap volume.
			struct Compound
			{
				collision::ShapeArray m_array;
				collision::ShapeBox m_outer{v4{2, 2, 2, 0}};
				collision::ShapeBox m_inner{v4{1, 1, 1, 0}};
			} compound;
			compound.m_array.Complete(2);
			auto const geometry = SampleOverlays::BuildGeometry(compound.m_array, true, true);
			auto const counts = physics::buoyancy::DistributeCounts({8.0f, 1.0f}, SampleOverlays::VolumeSampleCount);
			PR_EXPECT(geometry.m_surface.size() == physics::surface::BuildPlan(compound.m_outer).m_count);
			PR_EXPECT(geometry.m_volume.size() == counts[0]);
			for (auto const& sample : geometry.m_surface)
				PR_EXPECT(Abs(MaxElement(Abs(sample.m_pos_local.w0())) - 1.0f) < 1e-6f);
		}

		// Volume visualization derives missing tetrahedra without modifying the original surface-only polytope.
		PRUnitTestMethod(SurfaceOnlyPolytopeAndDeterministicSphere, Quick)
		{
			auto const points = std::vector<v4>{{0, 0, 0, 1}, {1, 0, 0, 1}, {0, 1, 0, 1}, {0, 0, 1, 1}};
			auto buffer = collision::BuildPolytopeFromPoints(points, m4x4::Identity(), 0, collision::Shape::EFlags::None, 0);
			auto const& poly = buffer.as<collision::ShapePolytope>();
			PR_EXPECT(poly.m_tet_count == 0);
			auto const surface = SampleOverlays::BuildGeometry(poly, true, false);
			auto const volume = SampleOverlays::BuildGeometry(poly, false, true);
			PR_EXPECT(!surface.m_surface.empty() && surface.m_volume.empty());
			PR_EXPECT(volume.m_surface.empty() && volume.m_volume.size() == SampleOverlays::VolumeSampleCount);
			PR_EXPECT(poly.m_tet_count == 0);
			for (auto const& point : volume.m_volume)
				PR_EXPECT(physics::buoyancy::ContainsLocal(poly, InvertOrthonormal(poly.m_base.m_s2r) * point, 1e-5f));

			// A repeated display request has the same point order, normals and weights.
			auto const sphere = collision::ShapeSphere(0.3f);
			auto const a = SampleOverlays::BuildGeometry(sphere, true, true);
			auto const b = SampleOverlays::BuildGeometry(sphere, true, true);
			PR_EXPECT(a.m_surface.size() == b.m_surface.size() && a.m_volume.size() == b.m_volume.size());
			for (size_t i = 0; i != a.m_surface.size(); ++i)
			{
				PR_EXPECT(FEql(a.m_surface[i].m_pos_local, b.m_surface[i].m_pos_local));
				PR_EXPECT(FEql(a.m_surface[i].m_normal_local, b.m_surface[i].m_normal_local));
				PR_EXPECT(a.m_surface[i].m_darea == b.m_surface[i].m_darea);
				PR_EXPECT(Abs(Length(a.m_surface[i].m_pos_local.w0()) - 0.3f) < 1e-6f);
			}
			for (size_t i = 0; i != a.m_volume.size(); ++i)
				PR_EXPECT(FEql(a.m_volume[i], b.m_volume[i]));
		}

		// Read actual off-screen pixels: samples inside an opaque box must be drawn after it, not merely disable depth.
		PRUnitTestMethod(OffscreenSamplesRenderOverOpaqueGeometry, Quick)
		{
			auto renderer = rdr12::Renderer(rdr12::RdrSettings(GetModuleHandle(nullptr)));
			auto settings = rdr12::WndSettings(nullptr, true, renderer.Settings()).Size(96, 96);
			settings.m_mode.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
			auto window = rdr12::Window(renderer, settings);
			// A separate texture owns its RTV; Window::CreateRenderTarget uses the window's reserved MSAA descriptor slot.
			auto factory = rdr12::ResourceFactory(renderer);
			auto target_desc = rdr12::TextureDesc(rdr12::AutoId, pr::compute::ResDesc::Tex2D(pr::compute::Image{96, 96, nullptr, settings.m_mode.Format}, 1U, pr::compute::EUsage::RenderTarget).clear(window.m_rt_props));
			target_desc.rtv_format(pr::compute::ToSRGB(settings.m_mode.Format));
			auto texture_target = factory.CreateTexture2D(target_desc);
			factory.FlushToGpu(rdr12::EGpuFlush::Block);
			auto target = rdr12::BackBuffer(window, pr::compute::MultiSamp(1), texture_target.get(), nullptr);
			window.CustomSwapChain(std::span{&target, 1});
			auto scene = rdr12::Scene(window);
			scene.m_cam.LookAt(v4{0, 0, 5, 1}, v4::Origin(), v4::YAxis());
			auto builder = ldraw::Builder{};
			builder.Box("occluder", 0xFF808080U).box(2, 2, 2);
			auto opaque = rdr12::ldraw::Parse(renderer, builder.ToBinary()).m_objects.front();
			auto const inner = collision::ShapeBox(v4{0.5f, 0.5f, 0.5f, 0});
			auto overlays = SampleOverlays{};
			auto job = pr::compute::GpuJob<D3D12_COMMAND_LIST_TYPE_DIRECT>(renderer.d3d(), "OverlayReadback", 0);
			for (int test_case = 0; test_case != 4; ++test_case)
			{
				// Cover both overlay types and falsify the draw-order guard locally, without publishing a broken application binary.
				auto const surface = test_case < 2;
				auto const underlay = (test_case & 1) != 0;
				scene.ClearDrawlists();
				overlays.Surface(surface);
				overlays.Volume(!surface);
				overlays.Reset();
				overlays.BeginFrame();
				overlays.Add(scene, renderer, &inner, inner, m4x4::Identity());
				if (underlay)
				{
					scene.ClearDrawlists();
					auto const& instance = overlays.m_instances.at(&inner);
					auto object = surface ? instance.m_surface : instance.m_volume;
					object->SortGroup(rdr12::ESortGroup::PreOpaques, "");
					object->AddToScene(scene);
				}
				opaque->AddToScene(scene);
				auto& frame = window.NewFrame();
				scene.Render(frame);
				window.Present(frame, rdr12::EGpuFlush::Block);

				// Copy the submitted final image, after opaque rendering and alpha composition, into a mapped readback allocation.
				auto* texture = window.FrameOutput().m_render_target.get();
				auto const desc = texture->GetDesc();
				auto footprint = D3D12_PLACED_SUBRESOURCE_FOOTPRINT{};
				auto bytes = UINT64{};
				renderer.d3d()->GetCopyableFootprints(&desc, 0, 1, 0, &footprint, nullptr, nullptr, &bytes);
				auto readback = job.m_readback.Alloc(static_cast<int64_t>(bytes), D3D12_TEXTURE_DATA_PLACEMENT_ALIGNMENT);
				footprint.Offset = readback.m_ofs;
				auto destination = D3D12_TEXTURE_COPY_LOCATION{.pResource = readback.m_res, .Type = D3D12_TEXTURE_COPY_TYPE_PLACED_FOOTPRINT, .PlacedFootprint = footprint};
				auto source = D3D12_TEXTURE_COPY_LOCATION{.pResource = texture, .Type = D3D12_TEXTURE_COPY_TYPE_SUBRESOURCE_INDEX, .SubresourceIndex = 0};
				auto barrier = D3D12_RESOURCE_BARRIER{};
				barrier.Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
				barrier.Transition = {texture, D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES, pr::compute::DefaultResState(texture), D3D12_RESOURCE_STATE_COPY_SOURCE};
				job.m_cmd_list.ResourceBarrier({&barrier, 1});
				job.m_cmd_list.CopyTextureRegion(&destination, 0, 0, 0, &source, nullptr);
				std::swap(barrier.Transition.StateBefore, barrier.Transition.StateAfter);
				job.m_cmd_list.ResourceBarrier({&barrier, 1});
				job.Run();

				// The occluder/background are neutral; only true foreground sample/normal colours can contribute chromatic pixels.
				auto amber = 0;
				auto green = 0;
				auto blue = 0;
				for (UINT y = 0; y != desc.Height; ++y)
				{
					auto const* row = readback.ptr<uint8_t>() + y * footprint.Footprint.RowPitch;
					for (UINT64 x = 0; x != desc.Width; ++x)
					{
						auto const* pixel = row + 4 * x;
						amber += pixel[0] > pixel[1] + 20 && pixel[1] > pixel[2] + 20;
						green += pixel[1] > pixel[0] + 20 && pixel[1] > pixel[2] + 20;
						blue += pixel[2] > pixel[0] + 40;
					}
				}
				scene.ClearDrawlists();
				if (underlay)
					PR_EXPECT(amber == 0 && green == 0 && blue == 0);
				else if (surface)
					PR_EXPECT(amber > 0 && green > 0);
				else
					PR_EXPECT(blue > 0);
			}

			// Retire model instances only after the last submitted frame and its draw-list references are finished.
			window.WaitForGpu();
			scene.ClearDrawlists();
			overlays.Reset();
		}

		// Upload real renderer models and verify shared geometry does not share mutable instance transforms.
		PRUnitTestMethod(RendererModelsAndPersistentInstances, Quick)
		{
			auto renderer = rdr12::Renderer(rdr12::RdrSettings(GetModuleHandle(nullptr)));
			auto const box = collision::ShapeBox(v4{0.2f, 0.2f, 0.2f, 0});
			auto model = BuildModels(renderer, SampleOverlays::BuildGeometry(box, true, true));
			auto a = InstanceOf(model.m_surface);
			auto b = InstanceOf(model.m_surface);
			auto volume = InstanceOf(model.m_volume);
			a->O2W(m4x4::Translation(1.0f, 2.0f, 3.0f));
			b->O2W(m4x4::Translation(4.0f, 5.0f, 6.0f));
			PR_EXPECT(a.get() != b.get() && !a->m_child.empty());
			PR_EXPECT(a->m_child[0]->m_model == b->m_child[0]->m_model);
			PR_EXPECT(!FEql(a->O2W(), b->O2W()));
			for (auto const& instance : {a, b, volume})
			{
				instance->Apply([](auto* object)
				{
					PR_EXPECT(AllSet(object->Flags(), rdr12::ldraw::ELdrFlags::NoZTest | rdr12::ldraw::ELdrFlags::NoZWrite));
					PR_EXPECT(object->SortGroup() == rdr12::ESortGroup::PostAlpha);
					return true;
				}, "");
			}

			// Model ownership remains valid after template retirement while persistent instances still hold the resources.
			model = {};
			PR_EXPECT(a->m_child[0]->m_model != nullptr && volume->m_model != nullptr);
			a = b = volume = nullptr;
			renderer.FlushDeferredReleases();
		}
	};
}
#endif
