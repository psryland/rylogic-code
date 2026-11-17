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
		auto bound = BoundDescriptors{};
		auto& last = bound.m_slots[0];
		auto source = Descriptor(0, D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV, {64}, 1);
		Require(BindMaterialDescriptor(commands, heap, 0U, source, &bound), "First descriptor was skipped");
		for (auto i = 0; i != 1000; ++i)
			Require(!BindMaterialDescriptor(commands, heap, 0U, source, &bound), "Unchanged descriptor was rebound");

		Require(heap.m_calls == 1 && commands.m_calls == 1, "Unchanged draws performed heap lookups or bindings");

		// Reusing the same CPU slot must rebind; the saved identity must not alias the mutable source descriptor.
		++source.m_generation;
		Require(BindMaterialDescriptor(commands, heap, 0U, source, &bound), "Recycled descriptor slot was skipped");
		Require(heap.m_calls == 2 && commands.m_calls == 2, "Generation change did not reach both boundaries");
		for (auto changed : {Descriptor(1, source.m_type, source.m_cpu, source.m_generation), Descriptor(source.m_index, source.m_type, {128}, source.m_generation), Descriptor(source.m_index, D3D12_DESCRIPTOR_HEAP_TYPE_SAMPLER, source.m_cpu, source.m_generation)})
			Require(changed != source, "Descriptor identity omitted a field");

		// Each root slot is tracked independently, so binding another slot neither replaces nor invalidates slot 0's saved identity.
		auto secondary = Descriptor(2, source.m_type, {256}, 3);
		Require(BindMaterialDescriptor(commands, heap, 1U, secondary, &bound), "First secondary descriptor was skipped");
		Require(!BindMaterialDescriptor(commands, heap, 1U, secondary, &bound), "Unchanged secondary descriptor was rebound");
		Require(last == source && bound.m_slots[1] == secondary && heap.m_calls == 3, "Root slots share tracking");
		Require(!BindMaterialDescriptor(commands, heap, 0U, source, &bound), "Secondary root slot forced a slot 0 lookup");

		// Untracked binds always reach the command list.
		BindMaterialDescriptor(commands, heap, 1U, secondary, nullptr);
		Require(heap.m_calls == 4 && commands.m_calls == 4, "Untracked bind was skipped");

		// A new batch or invalidated root signature/heap must discard the previous bindings.
		bound.Reset();
		Require(BindMaterialDescriptor(commands, heap, 0U, source, &bound), "Reset tracking did not rebind");
		Require(heap.m_calls == 5 && commands.m_calls == 5, "Reset did not reach the command list");

		// Failed lookups must neither publish a source identity nor issue a root binding.
		heap.m_fail = true;
		auto rejected = false;
		try { BindMaterialDescriptor(commands, heap, 0U, secondary, &bound); }
		catch (std::runtime_error const&) { rejected = true; }
		Require(rejected && last == source && commands.m_calls == 5, "Failed lookup changed binding state");
		heap.m_fail = false;
		bound.Reset();
		rejected = false;
		try { BindMaterialDescriptor(commands, heap, 0U, Descriptor{}, &bound); }
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
				auto eyes = std::array<SceneCamera, 2>{ view.StereoEye(EEye::Left, 0.065f, 10.0f, 0.85f), view.StereoEye(EEye::Right, 0.065f, 10.0f, 0.85f) };
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

	// Verify the parallel-axis stereo eye cameras: positions, orientation, and the zero-disparity convergence plane.
	void StereoEyeTests()
	{
		using namespace pr;
		using namespace pr::rdr12;

		// Use a scene scale far from metres to show the eye maths does not assume a unit
		auto const separation = 6.5f;
		auto const convergence = 400.0f;
		auto const eye_aspect = 1920.0f / 1080.0f;
		auto const c2w = m4x4::LookAt(v4(120, -80, 300, 1), v4(10, 20, -30, 1), v4::YAxis());
		auto const logical = SceneCamera(c2w, 0.9f, 2.0f * eye_aspect, 250.0f, false, 0.1f, 10000.0f);
		auto const left = logical.StereoEye(EEye::Left, separation, convergence, eye_aspect);
		auto const right = logical.StereoEye(EEye::Right, separation, convergence, eye_aspect);

		// The eyes sit half the separation either side of the logical camera, with its orientation and lens, but the eye's aspect
		Require(Length(left.CameraToWorld().pos - (c2w.pos - 0.5f * separation * c2w.x)) < 0.0001f, "Left eye position is wrong");
		Require(Length(right.CameraToWorld().pos - (c2w.pos + 0.5f * separation * c2w.x)) < 0.0001f, "Right eye position is wrong");
		for (auto const& eye : {left, right})
		{
			Require(Length(eye.CameraToWorld().z - c2w.z) < 0.00001f && Length(eye.CameraToWorld().y - c2w.y) < 0.00001f, "Stereo eyes must not toe in");
			Require(eye.FovY() == logical.FovY() && eye.FocusDist() == logical.FocusDist(), "Stereo eyes must keep the logical lens");
			Require(std::abs(eye.Aspect() - eye_aspect) < 0.00001, "Stereo eyes must use the eye image aspect");
		}

		// Project a world point through an eye's full projection to normalised screen space
		auto project = [](SceneCamera const& eye, v4 ws)
		{
			auto clip = eye.CameraToScreen() * eye.WorldToCamera() * ws;
			return v2(clip.x / clip.w, clip.y / clip.w);
		};

		// Points on the convergence plane have zero disparity. Nearer points shift right in the left eye, farther points shift left,
		// by (s/2)/tanX * (1/d - 1/C), with the opposite shift in the right eye.
		auto const tan_half_fovx = std::tan(0.5f * s_cast<float>(logical.FovY())) * eye_aspect;
		for (auto [ox, oy, depth] : {std::tuple{0.0f, 0.0f, convergence}, {30.0f, -12.0f, convergence}, {0.0f, 0.0f, 50.0f}, {-20.0f, 9.0f, 2000.0f}})
		{
			auto ws = c2w.pos + ox * c2w.x + oy * c2w.y - depth * c2w.z;
			auto l = project(left, ws);
			auto r = project(right, ws);
			auto expected = (0.5f * separation / tan_half_fovx) * (1.0f / depth - 1.0f / convergence);
			Require(std::abs((l.x - r.x) - 2.0f * expected) < 0.00002f, "Stereo disparity does not match the convergence plane");
			Require(std::abs(l.y - r.y) < 0.00001f, "Stereo eyes must have no vertical disparity");

			// The camera's NSS helpers must agree with the shifted projection and invert it
			auto nss = left.WSPointToNSSPoint(ws);
			Require(std::abs(nss.x - l.x) < 0.00002f && std::abs(nss.y - l.y) < 0.00002f, "WSPointToNSSPoint ignores the lens shift");
			Require(Length(left.NSSPointToWSPoint(nss) - ws) < 0.01f, "NSSPointToWSPoint does not invert the lens shift");
		}

		// Orthographic cameras have no depth disparity, so both eyes see the logical image
		auto ortho = logical;
		ortho.Orthographic(true);
		auto const ol = ortho.StereoEye(EEye::Left, separation, convergence, eye_aspect);
		auto const orr = ortho.StereoEye(EEye::Right, separation, convergence, eye_aspect);
		auto const ol_c2s = ol.CameraToScreen();
		auto const or_c2s = orr.CameraToScreen();
		auto const ol_c2w = ol.CameraToWorld();
		auto const or_c2w = orr.CameraToWorld();
		Require(std::memcmp(&ol_c2s, &or_c2s, sizeof(m4x4)) == 0 && std::memcmp(&ol_c2w, &or_c2w, sizeof(m4x4)) == 0, "Orthographic eyes must be identical");
		Require(std::memcmp(&ol_c2w, &c2w, sizeof(m4x4)) == 0, "Orthographic eyes must use the logical camera");
	}

	// Render side-by-side stereo through the public API: per-eye sizing and aspect, convergence-plane alignment, symmetric disparity, transparency, SSAO,
	// ray traced reflections, odd output widths, and toggling back to mono.
	void StereoRenderTests(int samples)
	{
		// A perspective camera at the origin looking down -Z. Each eye sees 100 units of height at the convergence distance, so 1.28 pixels per unit there.
		Fixture fixture(samples);
		constexpr float Convergence = 50.0f;
		constexpr float Separation = 10.0f;
		// Express the separation as a 65 mm eye separation with 6.5 mm scene units, to cover the millimetre conversion.
		constexpr float SeparationMm = 65.0f;
		constexpr float SceneUnitMm = SeparationMm / Separation;
		constexpr int EyeW = ImageSize / 2;
		View3D_CameraOrthographicSet(fixture.m_window, FALSE);
		Require(std::abs(View3D_CameraAspectGet(fixture.m_window) - 1.0f) < 1e-4f, "Mono camera aspect should match the square output");
		View3D_StereoSet(fixture.m_window, api::StereoProps{ .m_enabled = TRUE, .m_eye_separation_mm = SeparationMm, .m_scene_unit_mm = SceneUnitMm, .m_convergence = Convergence });
		Require(View3D_StereoGet(fixture.m_window).m_enabled, "Stereo did not enable");
		Require(std::abs(View3D_CameraAspectGet(fixture.m_window) - 0.5f) < 1e-4f, "Stereo camera aspect should be the eye image aspect");
		View3D_CameraFovSet(fixture.m_window, api::Vec2{ 2 * std::atan(0.5f), 2 * std::atan(1.0f) });
		fixture.CheckErrors();

		// The pixel in eye 'eye' at eye-local column 'x' of row 'y'.
		auto px = [](std::vector<unsigned char> const& image, int width, int eye, int x, int y)
		{
			// Eyes are laid out left to right, each 'EyeW' wide.
			auto const* p = image.data() + (size_t(y) * width + eye * EyeW + x) * 4;
			return std::array<float, 3>{ Linear(p[0]), Linear(p[1]), Linear(p[2]) };
		};

		// The scene is symmetric about x = 0 and the eyes are symmetric about the logical camera, so the right eye's image is the mirror of the left eye's.
		auto expect_mirrored = [&](std::vector<unsigned char> const& image, int width, float tolerance, char const* what)
		{
			// Allow a few differing pixels for rasterization ties, but not a missing or offset eye.
			auto differing = 0;
			for (auto y = 0; y != ImageSize; ++y)
			{
				for (auto x = 0; x != EyeW; ++x)
				{
					auto l = px(image, width, 0, x, y);
					auto r = px(image, width, 1, EyeW - 1 - x, y);
					for (auto c = 0; c != 3; ++c)
					{
						if (std::abs(l[c] - r[c]) <= tolerance)
							continue;

						++differing;
						break;
					}
				}
			}
			if (differing > ImageSize / 2)
				throw std::runtime_error(std::string(what) + ": eye images are not mirror symmetric (" + std::to_string(differing) + " pixels differ)");
		};

		// Expect a colour at an eye-local pixel.
		auto expect_eye = [&](std::vector<unsigned char> const& image, int width, int eye, int x, float r, float g, float b)
		{
			// Compare against the linear colour with the same tolerance as 'Expect'.
			auto actual = px(image, width, eye, x, ImageSize / 2);
			float expected[] = { r, g, b };
			for (auto c = 0; c != 3; ++c)
			{
				if (std::abs(actual[c] - expected[c]) <= 0.025f)
					continue;

				throw std::runtime_error("Stereo pixel mismatch: eye " + std::to_string(eye) + " x " + std::to_string(x) + " channel " + std::to_string(c) + ": actual " + std::to_string(actual[c]) + ", expected " + std::to_string(expected[c]));
			}
		};

		// A red quad on the convergence plane spans x = +/-10 units, 25.6 pixels, at the same eye-local place in both eyes.
		// A green quad at half the convergence distance has crossed disparity: its 4 unit half-width appears as 5.1 pixels, and its centre moves
		// 5 units (6.4 pixels) towards the other eye's side, i.e. right in the left eye and left in the right eye.
		fixture.Quad(Convergence, 0xFFFF0000, 10);
		fixture.Quad(Convergence / 2, 0xFF00FF00, 2);
		auto anim_time = View3D_WindowAnimTimeGet(fixture.m_window);
		auto width = 0;
		auto image = fixture.Image(&width);
		Require(width == ImageSize, "Stereo must not change the output size");
		Require(View3D_WindowAnimTimeGet(fixture.m_window) == anim_time, "Rendering two eyes advanced the animation clock");
		expect_mirrored(image, width, 0.05f, "Opaque");
		expect_eye(image, width, 0, 24, 1, 0, 0);
		expect_eye(image, width, 0, 40, 0, 1, 0);
		expect_eye(image, width, 0, 50, 0, 0, 0);
		expect_eye(image, width, 0, 12, 0, 0, 0);
		expect_eye(image, width, 1, EyeW - 1 - 40, 0, 1, 0);
		expect_eye(image, width, 1, EyeW - 1 - 24, 1, 0, 0);

		// A translucent blue quad between the two keeps the halves independent: it tints the red and background, but not the nearer green.
		fixture.Quad(Convergence * 0.6f, 0x800000FF, 30);
		image = fixture.Image(&width);
		expect_mirrored(image, width, 0.05f, "Transparent");
		auto const a = 128.0f / 255.0f;
		expect_eye(image, width, 0, 24, 1 - a, 0, a);
		expect_eye(image, width, 0, 40, 0, 1, 0);
		expect_eye(image, width, 0, 4, 0, 0, a);
		expect_eye(image, width, 1, EyeW - 1 - 24, 1 - a, 0, a);

		// Screen space ambient occlusion samples each eye's own depth, so flat interiors far from any occluder are unchanged.
		View3D_AmbientOcclusionSet(fixture.m_window, api::AmbientOcclusionProps{ .m_enabled = TRUE });
		image = fixture.Image(&width);
		expect_eye(image, width, 0, 24, 1 - a, 0, a);
		expect_eye(image, width, 0, 40, 0, 1, 0);
		expect_eye(image, width, 1, EyeW - 1 - 24, 1 - a, 0, a);
		View3D_AmbientOcclusionSet(fixture.m_window, api::AmbientOcclusionProps{ .m_enabled = FALSE });

		// An odd output width gives each eye floor(width / 2) columns and clears the uncovered last column to the background.
		View3D_WindowBackBufferSizeSet(fixture.m_window, SIZE{ ImageSize + 1, ImageSize }, FALSE);
		View3D_WindowViewportSet(fixture.m_window, api::Viewport{ 0, 0, ImageSize + 1, ImageSize, 0, 1, ImageSize + 1, ImageSize });
		Require(std::abs(View3D_CameraAspectGet(fixture.m_window) - 0.5f) < 1e-4f, "Odd output width changed the eye aspect");
		image = fixture.Image(&width);
		Require(width == ImageSize + 1, "Back buffer resize was not applied");
		expect_mirrored(image, width, 0.05f, "Odd width");
		expect_eye(image, width, 1, EyeW, 0, 0, 0);
		View3D_WindowBackBufferSizeSet(fixture.m_window, SIZE{ ImageSize, ImageSize }, FALSE);
		View3D_WindowViewportSet(fixture.m_window, api::Viewport{ 0, 0, ImageSize, ImageSize, 0, 1, ImageSize, ImageSize });

		// The focus and origin points have the disparity of their world depth. With the camera at z = 25 and a focus distance of 40, the origin is at
		// depth 25 (eye-local x = 38.4 in the left eye) and the focus point at depth 40 (x = 33.6). Their green Y axes rise from those columns.
		// A focus point that moved with the eye would instead appear at x = 25.6.
		{
			// Show only the stock points. Clip planes scale with the focus distance, so restate them in absolute units.
			fixture.Clear();
			auto focus_dist = View3D_CameraFocusDistanceGet(fixture.m_window);
			View3D_CameraFocusDistanceSet(fixture.m_window, 40.0f);
			View3D_CameraClipPlanesSet(fixture.m_window, 1, 100, api::EClipPlanes::Both);
			View3D_CameraToWorldSet(fixture.m_window, api::Mat4x4{ {1,0,0,0}, {0,1,0,0}, {0,0,1,0}, {0,0,25,1} });
			View3D_StockObjectVisibleSet(fixture.m_window, api::EStockObject::FocusPoint | api::EStockObject::OriginPoint, TRUE);
			image = fixture.Image(&width);
			View3D_StockObjectVisibleSet(fixture.m_window, api::EStockObject::FocusPoint | api::EStockObject::OriginPoint, FALSE);
			View3D_CameraToWorldSet(fixture.m_window, api::Mat4x4{ {1,0,0,0}, {0,1,0,0}, {0,0,1,0}, {0,0,0,1} });
			View3D_CameraFocusDistanceSet(fixture.m_window, focus_dist);
			View3D_CameraClipPlanesSet(fixture.m_window, 1, 100, api::EClipPlanes::Both);

			// True if any left-eye column in [x0, x1], or its mirror in the right eye, is green in the rows just above the points.
			auto green_in = [&](int eye, int x0, int x1)
			{
				// Mirror the column range for the right eye.
				for (auto x = x0; x <= x1; ++x)
				{
					for (auto y = ImageSize / 2 - 3; y != ImageSize / 2; ++y)
					{
						auto c = px(image, width, eye, eye == 0 ? x : EyeW - 1 - x, y);
						if (c[1] > 0.05f && c[0] < 0.05f && c[2] < 0.05f)
							return true;
					}
				}
				return false;
			};
			for (auto eye = 0; eye != 2; ++eye)
			{
				Require(green_in(eye, 37, 40), "Origin point is not at its world depth");
				Require(green_in(eye, 32, 35), "Focus point is not at its world depth");
				Require(!green_in(eye, 22, 30), "Focus point moved with the eye camera");
			}
		}

		// Ray traced reflections use each eye's camera. A green quad behind the camera, reflected by a mirror on the convergence plane, appears at
		// depth 110, beyond convergence. Its uncrossed disparity moves it 2.7 units (3.5 pixels) left in the left eye and right in the right eye.
		if (View3D_WindowRayTracingInfoGet(fixture.m_window).m_available)
		{
			// Replace the scene with only the mirror and the hidden reflected quad.
			fixture.Clear();
			auto mirror = View3D_ObjectCreateLdrA("*Plane Mirror FF000000 {*Data{90 90} *Reflectivity{1} *o2w{*pos{0 0 -50}}}", FALSE, nullptr, nullptr);
			Require(mirror != nullptr, "Mirror creation failed");
			fixture.m_objects.push_back(mirror);
			View3D_WindowAddObject(fixture.m_window, mirror);
			fixture.Quad(-10, 0xFF00FF00, 8);
			View3D_RayTracingPropertiesSet(fixture.m_window, { api::ERayTracingFeature::Reflections, 1 });
			View3D_WindowRayTracingEnabledSet(fixture.m_window, TRUE);
			Require(View3D_WindowRayTracingEnabledGet(fixture.m_window), "Ray tracing did not enable");
			image = fixture.Image(&width);
			expect_mirrored(image, width, 0.05f, "Ray traced");
			Require(px(image, width, 0, 26, ImageSize / 2)[1] > 0.1f, "Reflected quad missing at its left eye position");
			Require(px(image, width, 0, 37, ImageSize / 2)[1] < 0.02f, "Reflected quad has no uncrossed disparity in the left eye");
			View3D_WindowRayTracingEnabledSet(fixture.m_window, FALSE);
		}
		else
		{
			auto info = View3D_WindowRayTracingInfoGet(fixture.m_window);
			std::cout << "SKIP stereo ray traced reflection test: DXR unavailable (requested " << info.m_requested << ", hardware " << info.m_hardware_supported << ")\n";
		}

		// Disabling stereo restores the mono aspect and one full-width view with no disparity.
		fixture.Clear();
		fixture.Quad(Convergence, 0xFFFF0000, 10);
		fixture.Quad(Convergence / 2, 0xFF00FF00, 2);
		View3D_StereoSet(fixture.m_window, api::StereoProps{ .m_enabled = FALSE, .m_eye_separation_mm = SeparationMm, .m_scene_unit_mm = SceneUnitMm, .m_convergence = Convergence });
		Require(std::abs(View3D_CameraAspectGet(fixture.m_window) - 1.0f) < 1e-4f, "Mono camera aspect was not restored");
		image = fixture.Image(&width);
		Require(width == ImageSize, "Mono output size changed");
		Expect(image, 0, 1, 0, 64, 64);
		Expect(image, 1, 0, 0, 64 + 9, 64);
		Expect(image, 0, 0, 0, 64 + 20, 64);

		// Changing the layout must keep caller-supplied output buffers (see 'View3D_WindowCustomSwapChain'), which the window cannot recreate itself.
		{
			// A window without a swap chain, rendering into a texture that stands in for an externally owned output, such as a WPF D3DImage front buffer
			auto window = View3D_WindowCreate(nullptr, api::WindowOptions().error_cb({&fixture.m_errors, ReportError}).back_colour(0xFF000000).multisamp(samples).name("CustomOutput"));
			Require(window != nullptr, "View3D_WindowCreate without a HWND failed");
			auto destroy = pr::Scope<void>([&] { View3D_WindowGSyncWait(window); View3D_WindowDestroy(window); });
			auto options = api::TextureOptions{};
			options.m_format = DXGI_FORMAT_B8G8R8A8_UNORM_SRGB;
			options.m_usage = D3D12_RESOURCE_FLAG_ALLOW_RENDER_TARGET;
			options.m_resource_state = D3D12_RESOURCE_STATE_PRESENT;
			options.m_clear_value.Format = options.m_format;
			options.m_mips = 1;
			options.m_multisamp = {1, 0};
			options.m_t2s = {{1,0,0,0}, {0,1,0,0}, {0,0,1,0}, {0,0,0,1}};
			auto output = api::TexturePtr(View3D_TextureCreate(ImageSize, ImageSize, nullptr, 0, options));
			fixture.CheckErrors();
			auto output_tex = output.get();
			View3D_WindowCustomSwapChain(window, 1, &output_tex);

			// Toggle stereo on and off, rendering after each change, without any resize.
			for (auto enabled : { TRUE, FALSE })
			{
				View3D_StereoSet(window, api::StereoProps{ .m_enabled = enabled, .m_eye_separation_mm = SeparationMm, .m_scene_unit_mm = SceneUnitMm, .m_convergence = Convergence });
				auto bb_size = View3D_WindowBackBufferSizeGet(window);
				Require(bb_size.cx == ImageSize && bb_size.cy == ImageSize, "Changing the stereo layout released the custom output buffer");
				View3D_WindowRender(window);
				View3D_WindowGSyncWait(window);
				fixture.CheckErrors();
				auto frame = View3D_WindowFrameOutputGet(window);
				Require(frame.m_render_target != nullptr && frame.m_dim.cx == ImageSize, "No frame rendered into the custom output buffer after a stereo change");
			}
			View3D_WindowCustomSwapChain(window, 0, nullptr);
		}
		fixture.CheckErrors();
		fixture.CheckDebugLayer();
		std::cout << "PASS side-by-side stereo: sizing, convergence, disparity, transparency, SSAO, odd width, and mono toggle: MSAA " << samples << '\n';
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
	PRUnitTest(View3d12_StereoEye, Quick)
	{
		render_tests::StereoEyeTests();
	}
	PRUnitTest(View3d12_StereoRender, Extended | Serial)
	{
		render_tests::StereoRenderTests(1);
		render_tests::StereoRenderTests(4);
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
