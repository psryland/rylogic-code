#include "forward.h"
#include "render_test_support.h"

namespace render_tests
{
	// Exercise the public synchronous hit path and destroy its window without a later frame or explicit window wait.
	void RayCastLifetimeTests()
	{
		// A real DLL-owned hit/readback must survive immediate destruction, including the base upload-buffer destructor.
		{
			auto fixture = Fixture(1);
			Require(fixture.m_info != nullptr, "RayCast lifetime test requires D3D12 debug validation");
			auto object = fixture.Quad(10, 0xFFFFFFFF);
			auto ray = api::HitTestRay{};
			ray.m_ws_origin = api::Vec4{0, 0, 0, 1};
			ray.m_ws_direction = api::Vec4{0, 0, -1, 0};
			auto hit = api::HitTestResult{};
			std::cout << "RayCast lifetime: public immediate hit" << std::endl;
			View3D_WindowHitTestObjects(fixture.m_window, &ray, &hit, 1, &object, 1);
			fixture.CheckErrors();
			Require(hit.m_obj == object && std::abs(hit.m_distance - 10.0f) < 0.01f, "Immediate RayCast did not return the actual completed hit");
			View3D_WindowDestroy(fixture.m_window);
			fixture.m_window = nullptr;
			fixture.CheckErrors();
			fixture.CheckDebugLayer();
			std::cout << "PASS public immediate readback and destroy without another frame" << std::endl;
		}

		// Native-only objects keep allocator, material, and model ownership within one linked image.
		RayCastLifetimeNativeTests();
	}

	// Count heap lookups and root bindings independently so redundant lookups cannot hide behind unchanged GPU handles.
	void DescriptorBindingTests()
	{
		using namespace pr::rdr12;

		// A fixed returned handle also exposes comparisons that incorrectly ignore source generations.
		struct Heap
		{
			int m_calls = 0;
			bool m_fail = false;
			D3D12_GPU_DESCRIPTOR_HANDLE Add(Descriptor const& source)
			{
				++m_calls;
				if (m_fail || !source)
					throw std::runtime_error("Fixture descriptor lookup failed");

				return {123};
			}
		};

		// Record only the command boundary used by the production binding helper.
		struct Commands
		{
			int m_calls = 0;
			void SetGraphicsRootDescriptorTable(unsigned, D3D12_GPU_DESCRIPTOR_HANDLE handle)
			{
				Require(handle.ptr == 123, "Incorrect GPU descriptor forwarded");
				++m_calls;
			}
		};
		auto heap = Heap{};
		auto commands = Commands{};
		auto last = Descriptor{};
		auto source = Descriptor(0, D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV, {64}, 1);
		Require(BindMaterialDescriptor(commands, heap, 0U, source, &last), "First descriptor was skipped");
		for (auto i = 0; i != 1000; ++i)
			Require(!BindMaterialDescriptor(commands, heap, 0U, source, &last), "Unchanged descriptor was rebound");

		Require(heap.m_calls == 1 && commands.m_calls == 1, "Unchanged draws performed heap lookups or bindings");

		// Reusing the same CPU slot must rebind; the saved identity must not alias the mutable source descriptor.
		++source.m_generation;
		Require(BindMaterialDescriptor(commands, heap, 0U, source, &last), "Recycled descriptor slot was skipped");
		Require(heap.m_calls == 2 && commands.m_calls == 2, "Generation change did not reach both boundaries");
		for (auto changed : {Descriptor(1, source.m_type, source.m_cpu, source.m_generation), Descriptor(source.m_index, source.m_type, {128}, source.m_generation), Descriptor(source.m_index, D3D12_DESCRIPTOR_HEAP_TYPE_SAMPLER, source.m_cpu, source.m_generation)})
			Require(changed != source, "Descriptor identity omitted a field");

		// An untracked secondary root slot must not replace the diffuse slot's saved identity.
		auto secondary = Descriptor(2, source.m_type, {256}, 3);
		BindMaterialDescriptor(commands, heap, 1U, secondary, nullptr);
		BindMaterialDescriptor(commands, heap, 1U, secondary, nullptr);
		Require(last == source && heap.m_calls == 4, "Untracked root slot polluted diffuse tracking");
		Require(!BindMaterialDescriptor(commands, heap, 0U, source, &last), "Secondary root slot forced a diffuse lookup");

		// A new batch or invalidated root signature/heap must discard the previous binding.
		last = {};
		Require(BindMaterialDescriptor(commands, heap, 0U, source, &last), "Reset tracking did not rebind");
		Require(heap.m_calls == 5 && commands.m_calls == 5, "Reset did not reach the command list");

		// Failed lookups must neither publish a source identity nor issue a root binding.
		heap.m_fail = true;
		auto rejected = false;
		try { BindMaterialDescriptor(commands, heap, 0U, secondary, &last); }
		catch (std::runtime_error const&) { rejected = true; }
		Require(rejected && last == source && commands.m_calls == 5, "Failed lookup changed binding state");
		heap.m_fail = false;
		last = {};
		rejected = false;
		try { BindMaterialDescriptor(commands, heap, 0U, Descriptor{}, &last); }
		catch (std::runtime_error const&) { rejected = true; }
		Require(rejected, "Empty tracking suppressed invalid-source validation");
		std::cout << "PASS descriptor lookup counts, identity reuse, independent roots, invalidation, and failure handling\n";
	}

	// Compare mixed Simple/PBR batches with isolated draws, where descriptor reuse cannot affect another object.
	void DescriptorBindingRenderTests(int samples)
	{
		using namespace pr::rdr12;
		auto fixture = Fixture(samples);
		auto objects = std::array<api::Object, 6>{};
		auto options = api::TextureOptions{};
		options.m_format = DXGI_FORMAT_R8G8B8A8_UNORM;
		options.m_usage = D3D12_RESOURCE_FLAG_ALLOW_RENDER_TARGET;
		options.m_resource_state = D3D12_RESOURCE_STATE_ALL_SHADER_RESOURCE;
		options.m_clear_value.Format = options.m_format;
		options.m_mips = 1;
		options.m_multisamp = {1, 0};
		options.m_t2s = {{1,0,0,0}, {0,1,0,0}, {0,0,1,0}, {0,0,0,1}};

		// Each colour is shared by three objects and crosses the Simple/PBR material boundary.
		for (auto group = 0; group != 2; ++group)
		{
			auto pixel = group == 0 ? 0xFF0000FFU : 0xFF00FF00U;
			auto texture = View3D_TextureCreate(1, 1, &pixel, sizeof(pixel), options);
			fixture.CheckErrors();
			Require(texture != nullptr, "Descriptor fixture texture creation failed");
			auto release = pr::Scope<void>([&] { View3D_TextureRelease(texture); });
			for (auto i = group * 3; i != group * 3 + 3; ++i)
			{
				// Create materials inside the DLL so their virtual methods and allocations use the owning runtime.
				auto material = i == 2 || i == 3 ? "*Material {*BaseColour{FFFFFFFF} *Metallic{0} *Roughness{1}}" : "";
				auto script = std::string("*Plane Tile FFFFFFFF {*Data{12 90} ") + material + " *o2w{*pos{0 0 -10}}}";
				auto object = objects[i] = View3D_ObjectCreateLdrA(script.c_str(), FALSE, nullptr, nullptr);
				fixture.CheckErrors();
				Require(object != nullptr, "Descriptor fixture object creation failed");
				fixture.m_objects.push_back(object);
				View3D_WindowAddObject(fixture.m_window, object);
				View3D_ObjectSetTexture(object, texture, nullptr);
				auto placement = View3D_ObjectO2WGet(object, nullptr);
				placement.w.x = -37.5f + 15.0f * i;
				View3D_ObjectO2WSet(object, placement, nullptr);
			}
		}

		// Build a full-image reference from non-overlapping isolated objects, including their real material and sampler bindings.
		auto batched = fixture.Image();
		fixture.Clear();
		auto expected = fixture.Image();
		for (auto object : objects)
		{
			View3D_WindowAddObject(fixture.m_window, object);
			auto isolated = fixture.Image();
			auto coloured = 0;
			for (auto i = size_t{}; i != isolated.size(); i += 4)
			{
				for (auto channel = size_t{}; channel != 3; ++channel)
				{
					expected[i + channel] = std::max(expected[i + channel], isolated[i + channel]);
					coloured += isolated[i + channel] != 0;
				}
			}
			Require(coloured != 0, "Isolated descriptor fixture rendered no colour");
			fixture.Clear();
		}
		Require(batched == expected, "Batched descriptor bindings changed pixels relative to isolated draws");
		for (auto object : objects)
			View3D_WindowAddObject(fixture.m_window, object);

		Require(fixture.Image() == expected, "New frame retained stale descriptor bindings");
		fixture.CheckErrors();
		std::cout << "PASS bit-exact mixed Simple/PBR descriptor batches, MSAA " << samples << '\n';
	}

	// Require bit-identical per-draw transforms across camera changes, stereo eyes, projection overrides, and affine placements.
	void CameraTransformTests()
	{
		// A real model exercises model-root placement as well as the no-model constant-buffer path.
		using namespace pr;
		using namespace pr::rdr12;
		auto fixture = Fixture(1);
		auto object = fixture.Quad(10, 0xFFFFFFFF);
		auto& model = *object->m_model.get();
		auto shear = m4x4::Identity();
		shear.x.x = -2.0f;
		shear.y.y = 3.0f;
		shear.z.z = 0.25f;
		shear.y.x = 0.7f;
		shear.z.y = -0.3f;
		shear.pos = v4(2, -3, 4, 1);
		model.m_m2root = m4x4::Transform(v4(0.2f, -0.4f, 0.6f, 0), v4(1, 2, -1, 1)) * shear;
		auto placements = std::array{m4x4::Identity(), shear, m4x4::Transform(v4(-0.7f, 0.3f, 1.1f, 0), v4(-5, 8, -9, 1)) * shear};
		auto projections = std::array{m4x4::Zero(), m4x4::Identity(), m4x4::ProjectionPerspectiveFOV(0.7f, 1.3f, 0.2f, 200.0f, true), m4x4::ProjectionOrthographic(8.0f, 6.0f, 0.1f, 300.0f, true)};
		auto checked = 0;

		// Compare individual matrices, not struct padding, against the original per-draw arithmetic.
		auto check = [&]<typename TCBuf>(BaseInstance const& inst, Model const* model_ptr, SceneCamera const& view, CameraTransforms const& camera, TCBuf)
		{
			auto actual = TCBuf{};
			SetTxfm(actual, inst, model_ptr, camera);
			auto const o2w = GetO2W(inst);
			auto const m2o = model_ptr ? model_ptr->m_m2root : m4x4::Identity();
			auto const n2w = NormalTransform(o2w * m2o);
			auto const w2c = InvertOrthonormal(view.CameraToWorld());
			m4x4 c2s;
			c2s = FindC2S(inst, c2s) ? c2s : view.CameraToScreen();
			auto const o2s = c2s * w2c * o2w;
			Require(std::memcmp(&actual.m2o, &m2o, sizeof(m4x4)) == 0, "Model placement changed");
			Require(std::memcmp(&actual.o2w, &o2w, sizeof(m4x4)) == 0, "Object placement changed");
			Require(std::memcmp(&actual.n2w, &n2w, sizeof(m4x4)) == 0, "Normal transform changed");
			Require(std::memcmp(&actual.o2s, &o2s, sizeof(m4x4)) == 0, "Pass-local projection differs from per-draw arithmetic");
			++checked;
		};

		// Shadow constants carry only the placement, because shadow views supply the projection.
		auto check_placement = [&](BaseInstance const& inst, Model const* model_ptr)
		{
			auto actual = shaders::smap::ElementConstants{};
			SetPlacement(actual, inst, model_ptr);
			auto const o2w = GetO2W(inst);
			auto const m2o = model_ptr ? model_ptr->m_m2root : m4x4::Identity();
			Require(std::memcmp(&actual.m2o, &m2o, sizeof(m4x4)) == 0, "Shadow model placement changed");
			Require(std::memcmp(&actual.o2w, &o2w, sizeof(m4x4)) == 0, "Shadow object placement changed");
			++checked;
		};

		// Reuse each pass snapshot across many objects, then rebuild it for the next camera pose and stereo eye.
		for (auto orthographic : {false, true})
		{
			for (auto position : {v4(0, 0, 10, 1), v4(24, -38, 29, 1)})
			{
				auto view = SceneCamera(m4x4::LookAt(position, v4::Origin(), v4::YAxis()), 0.9f, 1.7f, 10.0f, orthographic, 0.1f, 1000.0f);
				SceneCamera eyes[2];
				view.Stereo(0.065f, eyes);
				for (auto const& eye : {view, eyes[0], eyes[1]})
				{
					auto const camera = CameraTransforms(eye);
					for (auto const& placement : placements)
					{
						for (auto const* model_ptr : {static_cast<Model const*>(nullptr), static_cast<Model const*>(&model)})
						{
							auto plain = rdr12::ldraw::StockInstance{};
							plain.m_i2w = placement;
							check(plain.m_base, model_ptr, eye, camera, shaders::fwd::ElementConstants{});
							check_placement(plain.m_base, model_ptr);
							for (auto const& projection : projections)
							{
								auto projected = rdr12::ldraw::RdrInstance{};
								projected.m_i2w = placement;
								projected.m_c2s = projection;
								check(projected.m_base, model_ptr, eye, camera, shaders::fwd::ElementConstants{});
								check_placement(projected.m_base, model_ptr);
							}
						}
					}
				}
			}
		}
		fixture.CheckErrors();
		std::cout << "PASS " << checked << " bit-exact forward/shadow camera-transform cases\n";
	}

	// Verify normal directions independently of their arbitrary common matrix scale.
	void NormalTransformTests()
	{
		using namespace pr;
		auto const normal = Normalise(v4(1, 2, 3, 0));
		auto scale = m4x4::Identity();
		scale.x.x = 2;
		scale.y.y = 3;
		scale.z.z = 0.25f;
		auto shear = m4x4::Identity();
		shear.y.x = 0.7f;
		shear.z.y = -0.3f;
		auto reflection = m4x4::Identity();
		reflection.x.x = -1;
		auto rotation = m4x4(v4(0, 1, 0, 0), v4(-1, 0, 0, 0), v4(0, 0, 1, 0), v4(2, 3, 4, 1));
		for (auto const& placement : {m4x4::Identity(), scale, shear, reflection, rotation, rotation * scale * shear * reflection})
		{
			auto expected = Normalise(Transpose(Invert(placement)).rot * normal.xyz);
			auto actual = Normalise((rdr12::NormalTransform(placement) * normal).xyz);
			Require(Length(actual - expected) < 0.00001f, "Normal transform differs from inverse transpose");
		}

		// The singular endpoint retains the remaining plane, while a line or point has no area normal.
		auto flat = m4x4::Identity();
		flat.z.z = 0;
		auto flat_normal = Normalise(rdr12::NormalTransform(rotation * flat) * normal);
		Require(Length(flat_normal - rotation.z) < 0.00001f, "Flattening lost the surviving plane normal");
		flat.y.y = 0;
		Require(Length(rdr12::NormalTransform(flat) * normal) == 0, "A line must not invent an area normal");
		Require(Length(rdr12::NormalTransform(m4x4::Zero()) * normal) == 0, "A point must not invent an area normal");

		// Extremely large and small uniform scales must not overflow their intermediate cross products.
		for (auto magnitude : {1.0e-30f, 1.0e30f})
		{
			auto placement = m4x4::Identity();
			placement.x.x = placement.y.y = placement.z.z = magnitude;
			Require(Length(Normalise(rdr12::NormalTransform(placement) * normal) - normal) < 0.00001f, "Normal transform lost extreme scale");
		}
	}

	// Replace a custom-pixel scene with stock world geometry and a real sky without an intervening render.
	void SceneHandoffTests(int samples)
	{
		Fixture fixture(samples);
		for (auto cycle = 0; cycle != 2; ++cycle)
		{
			// Render a custom pixel shader scene, then replace it.
			fixture.Clear();
			auto custom = fixture.Quad(89, 0xFFFF0000, 45, fixture.CustomShader(false));
			fixture.Image();
			View3D_WindowRemoveObject(fixture.m_window, custom);

			// Sky creation and flag changes must preserve its background classification.
			auto sky = View3D_ObjectCreateProceduralSky("HandoffSky", api::ProceduralSkySettings{}, nullptr);
			Require(sky != nullptr, "Handoff sky creation failed");
			fixture.m_objects.push_back(sky);
			Require(View3D_ObjectSortGroupGet(sky, nullptr) == api::ESortGroup::Skybox, "Procedural sky lost its background classification");
			View3D_ObjectFlagsSet(sky, api::ELdrFlags::Hidden, TRUE, nullptr);
			View3D_ObjectFlagsSet(sky, api::ELdrFlags::Hidden, FALSE, nullptr);
			View3D_ObjectFlagsSet(sky, api::ELdrFlags::NoZWrite, FALSE, nullptr);
			Require(View3D_ObjectSortGroupGet(sky, nullptr) == api::ESortGroup::Skybox, "Unrelated or unchanged flags erased the sky sort group");
			View3D_WindowAddObject(fixture.m_window, sky);
			fixture.Image();

			// Distant opaque geometry hides the sky and the farther alpha layer.
			fixture.Quad(94.5f, 0xFFFF0000);
			fixture.Quad(96.75f, 0x800000FF);
			Expect(fixture.Image(), 1,0,0);
			fixture.CheckDebugLayer();
		}

		// Unrelated flags do not erase an explicit overlay sort group.
		fixture.Clear();
		auto post_alpha = fixture.Quad(94.5f, 0xFF0000FF);
		View3D_ObjectSortGroupSet(post_alpha, api::ESortGroup::PostAlpha, nullptr);
		View3D_ObjectFlagsSet(post_alpha, api::ELdrFlags::HitTestExclude, TRUE, nullptr);
		Require(View3D_ObjectSortGroupGet(post_alpha, nullptr) == api::ESortGroup::PostAlpha, "Unrelated flags erased the overlay sort group");

		// Automatic depth ordering changes only on depth-policy transitions, with no-test overlays taking precedence.
		fixture.Clear();
		auto quad = fixture.Quad(89, 0xFFFFFFFF);
		auto default_group = View3D_ObjectSortGroupGet(quad, nullptr);
		View3D_ObjectFlagsSet(quad, api::ELdrFlags::NoZTest, TRUE, nullptr);
		Require(View3D_ObjectSortGroupGet(quad, nullptr) == api::ESortGroup::PostAlpha, "NoZTest must render after alpha");
		View3D_ObjectFlagsSet(quad, api::ELdrFlags::NoZWrite, TRUE, nullptr);
		Require(View3D_ObjectSortGroupGet(quad, nullptr) == api::ESortGroup::PostAlpha, "NoZWrite overrode NoZTest overlay ordering");
		View3D_ObjectFlagsSet(quad, api::ELdrFlags::NoZTest, FALSE, nullptr);
		Require(View3D_ObjectSortGroupGet(quad, nullptr) == api::ESortGroup::PreOpaques, "NoZWrite must render before opaques");
		View3D_ObjectFlagsSet(quad, api::ELdrFlags::NoZWrite, FALSE, nullptr);
		Require(View3D_ObjectSortGroupGet(quad, nullptr) == default_group, "Disabling depth overrides did not restore default ordering");
		fixture.CheckErrors();
		std::cout << "PASS repeated custom-scene/world handoff, real procedural sky overlap, and flag sort-group ownership: MSAA " << samples << '\n';
	}
}

namespace pr::unittests::view3d12
{
	PRUnitTest(View3d12_NormalTransform, Quick)
	{
		render_tests::NormalTransformTests();
	}
	PRUnitTest(View3d12_DescriptorBinding, Quick)
	{
		render_tests::DescriptorBindingTests();
	}
	PRUnitTest(View3d12_DescriptorBindingRender, Extended | Serial)
	{
		render_tests::DescriptorBindingRenderTests(1);
		render_tests::DescriptorBindingRenderTests(4);
	}
	PRUnitTest(View3d12_CameraTransform, Extended | Serial)
	{
		render_tests::CameraTransformTests();
	}
	PRUnitTest(View3d12_SceneHandoff, Extended | Serial)
	{
		render_tests::SceneHandoffTests(1);
		render_tests::SceneHandoffTests(4);
	}
	PRUnitTest(View3d12_RayCastLifetime, Extended | Serial)
	{
		// Serial because the native part turns on GPU-based validation, a process-wide setting that would also apply to devices other running tests create.
		render_tests::RayCastLifetimeTests();
	}
	PRUnitTest(View3d12_InitialiseThreadAffinity, Extended | Serial)
	{
		// A client on another thread must be rejected, because the dll context is bound to the thread that created it
		render_tests::Fixture fixture(1);
		auto errors = std::vector<std::string>{};
		auto handle = pr::view3d::DllHandle{};
		std::thread([&]
		{
			// Attempt to join the existing context from a different thread
			handle = View3D_Initialise({&errors, render_tests::ReportError});
		}).join();
		PR_EXPECT(handle == nullptr);
		PR_EXPECT(errors.size() == 1);
	}
}
