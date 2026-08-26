//*********************************************
// View3DUI Tests
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// M8 world-anchored roots: descriptor validation, deterministic projection, apparent-vs-world
// sizing, per-policy stage selection and draw ordering, semantic bounds for culled roots, and the
// private host-bridge ABI layout that carries the camera and resolved depth across the boundary.
#include "pr/common/unittests.h"
#include "pr/view3d-12/view3d-ui-bridge.h"
#include "pr/view3d-ui/engine.h"
#include "renderer.h"
#include "test_support.h"
#include "win32_input.h"
#include "world.h"
#include <algorithm>
#include <cmath>
#include <dxgi1_4.h>
#include <optional>

namespace pr::view3d::ui::tests
{
	namespace
	{
		// Absolute tolerance for projected DIP coordinates. Projection is a short chain of
		// single-precision multiplies, so exact equality is not required, but anything looser than
		// this would hide a genuine convention error (a flipped axis, a halved FOV, a lost aspect).
		constexpr float kTol = 1e-3f;

		bool Near(float lhs, float rhs, float tol = kTol)
		{
			return std::abs(lhs - rhs) <= tol;
		}

		// Apply one transaction containing exactly the given control descriptors, in order.
		void ApplyControls(UiEngine& engine, std::initializer_list<ControlDesc> controls, std::uint64_t base_revision = 0)
		{
			auto b = TxnBuilder{};
			for (auto const& c : controls)
				b.Upsert(c);

			engine.TransactionApply(b.Build(base_revision, base_revision + 1));
		}

		// The placement of a single world root, projected exactly as UiEngine::Update would.
		RootPlacement Project(ERootPolicy policy, WorldRootParams const& world, float w, float h, ViewportState const& vp)
		{
			return ProjectWorldRoot(policy, world, w, h, vp);
		}

		// Find the group recorded for 'root_id', or nullptr when the root emitted none.
		DrawGroup const* FindGroup(DrawPacket const& packet, ControlId root_id)
		{
			auto it = std::find_if(packet.groups.begin(), packet.groups.end(), [root_id](DrawGroup const& g) { return g.root_id == root_id; });
			return it != packet.groups.end() ? &*it : nullptr;
		}
	}

	#pragma region Descriptor validation

	PRUnitTest(WorldRootPoliciesAreAcceptedAndScreenRootsAreUnchanged, Quick)
	{
		// Every ERootPolicy is now a legal root descriptor; pre-M8 the three world policies were
		// rejected outright, so this is the gate M8 opens.
		for (auto policy : { ERootPolicy::Screen, ERootPolicy::DepthTested, ERootPolicy::OcclusionFaded, ERootPolicy::Overlay })
		{
			auto engine = UiEngine(MakeConfig());
			auto desc = policy == ERootPolicy::Screen
				? MakeControl(1, 0, EControlType::Root, ELayoutMode::Overlay, Lp(200.0f, 100.0f))
				: MakeWorldRoot(1, policy, 200.0f, 100.0f, WorldParams());

			ApplyControls(engine, { desc });
			engine.Update(Viewport(800, 600, 96.0f, 0.0, PerspectiveCamera()));
		}
	}

	PRUnitTest(RootPolicyOutsideTheEnumRangeIsRejected, Quick)
	{
		auto engine = UiEngine(MakeConfig());
		auto desc = MakeControl(1, 0, EControlType::Root, ELayoutMode::Overlay, Lp(200.0f, 100.0f));
		desc.root_policy = static_cast<ERootPolicy>(static_cast<int>(ERootPolicy::Count));

		auto b = TxnBuilder{};
		b.Upsert(desc);
		PR_THROWS(engine.TransactionApply(b.Build(0, 1)), std::exception);
	}

	PRUnitTest(WorldRootParametersAreValidatedFieldByField, Quick)
	{
		// Each mutation below is individually sufficient to make projection non-deterministic,
		// non-finite, or unbounded, so each must be rejected at transaction time rather than
		// producing NaN geometry or an unbounded fade later.
		auto reject = [](WorldRootParams const& world)
		{
			auto engine = UiEngine(MakeConfig());
			auto b = TxnBuilder{};
			b.Upsert(MakeWorldRoot(1, ERootPolicy::Overlay, 200.0f, 100.0f, world));
			PR_THROWS(engine.TransactionApply(b.Build(0, 1)), std::exception);
		};

		auto const inf = std::numeric_limits<float>::infinity();
		auto const nan = std::numeric_limits<float>::quiet_NaN();

		auto w = WorldParams(); w.anchor = Vec3{ nan, 0, 10 };                       reject(w);
		w = WorldParams(); w.anchor = Vec3{ 0, inf, 10 };                            reject(w);
		w = WorldParams(); w.anchor_h = static_cast<EAnchorPoint>(3);                reject(w);
		w = WorldParams(); w.anchor_v = static_cast<EAnchorPoint>(3);                reject(w);
		w = WorldParams(); w.sizing = static_cast<EWorldSizing>(2);                  reject(w);
		w = WorldParams(); w.world_units_per_dip = 0.0f;                             reject(w);
		w = WorldParams(); w.world_units_per_dip = -0.01f;                           reject(w);
		w = WorldParams(); w.world_units_per_dip = nan;                              reject(w);
		w = WorldParams(); w.depth_offset = -1.0f;                                   reject(w);
		w = WorldParams(); w.depth_offset = inf;                                     reject(w);
		w = WorldParams(); w.occlusion_min_opacity = -0.1f;                          reject(w);
		w = WorldParams(); w.occlusion_min_opacity = 1.1f;                           reject(w);
		w = WorldParams(); w.occlusion_min_opacity = nan;                            reject(w);
		w = WorldParams(); w.occlusion_fade_depth = 0.0f;                            reject(w);
		w = WorldParams(); w.occlusion_fade_depth = -1.0f;                           reject(w);
		w = WorldParams(); w.occlusion_depth_bias = -0.001f;                         reject(w);
		w = WorldParams(); w.occlusion_depth_bias = nan;                             reject(w);
	}

	PRUnitTest(WorldRootsRequireAnExplicitPositiveSizeBecauseTheyCannotAutoSizeToAViewport, Quick)
	{
		auto reject = [](LayoutParams const& lp)
		{
			auto engine = UiEngine(MakeConfig());
			auto desc = MakeControl(1, 0, EControlType::Root, ELayoutMode::Overlay, lp);
			desc.root_policy = ERootPolicy::Overlay;
			desc.world = WorldParams();

			auto b = TxnBuilder{};
			b.Upsert(desc);
			PR_THROWS(engine.TransactionApply(b.Build(0, 1)), std::exception);
		};

		reject(Lp(0.0f, 100.0f));
		reject(Lp(200.0f, 0.0f));
		reject(Lp(-200.0f, 100.0f));
	}

	PRUnitTest(ScreenRootsStillAutoSizeToTheViewport, Quick)
	{
		// The auto-size exception is now conditioned on ERootPolicy::Screen; a zero-sized screen
		// root must still fill the viewport exactly as it did in M2.
		auto engine = UiEngine(MakeConfig());
		ApplyControls(engine, { MakeControl(1, 0, EControlType::Root, ELayoutMode::Overlay, Lp(0.0f, 0.0f)) });
		engine.Update(Viewport(800, 600));

		auto const& packet = engine.DrawPackets();
		PR_EXPECT(packet.groups.size() == 1);
		PR_EXPECT(packet.groups[0].policy == ERootPolicy::Screen);
		PR_EXPECT(Near(packet.items[0].bounds.w, 800.0f));
		PR_EXPECT(Near(packet.items[0].bounds.h, 600.0f));
	}

	PRUnitTest(AnInvalidCameraIsRejectedDuringUpdateRatherThanProducingNaNGeometry, Quick)
	{
		auto const nan = std::numeric_limits<float>::quiet_NaN();

		auto expect_throw = [](CameraState const& cam)
		{
			PR_THROWS(ValidateCameraState(cam), std::exception);
		};

		auto c = PerspectiveCamera(); c.position = Vec3{ nan, 0, 0 };  expect_throw(c);
		c = PerspectiveCamera(); c.near_plane = 0.0f;                  expect_throw(c);
		c = PerspectiveCamera(); c.far_plane = 0.05f;                  expect_throw(c); // far <= near
		c = PerspectiveCamera(); c.fov_y_rad = 0.0f;                   expect_throw(c);
		c = PerspectiveCamera(); c.fov_y_rad = 3.2f;                   expect_throw(c); // >= pi
		c = PerspectiveCamera(); c.projection = static_cast<EProjection>(2); expect_throw(c);
		c = OrthographicCamera(6.0f); c.ortho_height = 0.0f;           expect_throw(c);

		// A camera that declares itself absent is always acceptable; screen-only hosts never fill one in.
		ValidateCameraState(CameraState{});
	}

	PRUnitTest(ADegenerateCameraBasisCullsEveryWorldRootInsteadOfThrowing, Quick)
	{
		// A finite-but-degenerate basis is a transient host state (a camera mid-reconfiguration),
		// not a contract violation, so it must silently cull rather than tear down the frame.
		auto expect_culled = [](CameraState const& cam)
		{
			ValidateCameraState(cam); // finite values, so validation passes
			PR_EXPECT(MakeCameraFrame(cam).valid == 0);

			auto vp = Viewport(800, 600, 96.0f, 0.0, cam);
			PR_EXPECT(Project(ERootPolicy::Overlay, WorldParams(Vec3{ 0, 0, 10 }), 200.0f, 100.0f, vp).visible == 0);
		};

		auto c = PerspectiveCamera(); c.forward = Vec3{ 0, 0, 0 };   expect_culled(c);
		c = PerspectiveCamera(); c.up = Vec3{ 0, 0, 0 };             expect_culled(c);
		c = PerspectiveCamera(); c.up = Vec3{ 0, 0, 1 };             expect_culled(c); // up parallel to forward
	}

	#pragma endregion
	#pragma region Projection math

	PRUnitTest(AnAnchorOnTheViewAxisProjectsToTheViewportCentre, Quick)
	{
		auto const vp = Viewport(800, 600, 96.0f, 0.0, PerspectiveCamera());
		auto const p = Project(ERootPolicy::Overlay, WorldParams(Vec3{ 0, 0, 10 }), 200.0f, 100.0f, vp);

		PR_EXPECT(p.visible == 1);
		PR_EXPECT(Near(p.scale, 1.0f));                 // ConstantDip
		PR_EXPECT(Near(p.rect.w, 200.0f));
		PR_EXPECT(Near(p.rect.h, 100.0f));
		PR_EXPECT(Near(p.rect.x, 400.0f - 100.0f));     // centre-anchored
		PR_EXPECT(Near(p.rect.y, 300.0f - 50.0f));
		PR_EXPECT(Near(p.view_depth, 10.0f));
	}

	PRUnitTest(PerspectiveProjectionMatchesTheClosedFormForAnOffAxisAnchor, Quick)
	{
		// fov_y = 90 degrees at d = 10 puts the top edge of the frustum at y = +10, so an anchor at
		// y = +5 must land exactly a quarter of the viewport height above centre. The x term picks
		// up the aspect divisor, which is what a transposed aspect would break.
		auto const fov = 1.5707963f;
		auto const vp = Viewport(800, 600, 96.0f, 0.0, PerspectiveCamera(Vec3{ 0, 0, 0 }, fov));
		auto const p = Project(ERootPolicy::Overlay, WorldParams(Vec3{ 5, 5, 10 }), 20.0f, 10.0f, vp);

		PR_EXPECT(p.visible == 1);

		// ndc_y = 5 / (10 * tan(45deg)) = 0.5 => 25% of the height above centre.
		PR_EXPECT(Near(p.rect.y + p.rect.h * 0.5f, 300.0f - 150.0f, 0.01f));

		// ndc_x = 5 / (10 * tan(45deg) * 4/3) = 0.375 => 0.375 * 400px from centre. View3D's basis
		// is right-handed with camera +z opposite the look direction, so screen-right is world -x
		// for a camera looking along +z with +y up; an anchor at x = +5 therefore lands left of centre.
		PR_EXPECT(Near(p.rect.x + p.rect.w * 0.5f, 400.0f - 150.0f, 0.01f));
	}

	PRUnitTest(OrthographicProjectionIsIndependentOfDistance, Quick)
	{
		auto const vp = Viewport(800, 600, 96.0f, 0.0, OrthographicCamera(6.0f));
		auto const near_p = Project(ERootPolicy::Overlay, WorldParams(Vec3{ 1, 1, 5 }), 20.0f, 10.0f, vp);
		auto const far_p = Project(ERootPolicy::Overlay, WorldParams(Vec3{ 1, 1, 50 }), 20.0f, 10.0f, vp);

		PR_EXPECT(near_p.visible == 1 && far_p.visible == 1);
		PR_EXPECT(Near(near_p.rect.x, far_p.rect.x));
		PR_EXPECT(Near(near_p.rect.y, far_p.rect.y));

		// ortho_height 6 over 600px => 100px per world unit; y = +1 is 100px above centre, and x = +1
		// is 100px to the *left* under View3D's right-handed basis (see the perspective test above).
		PR_EXPECT(Near(near_p.rect.y + near_p.rect.h * 0.5f, 300.0f - 100.0f, 0.01f));
		PR_EXPECT(Near(near_p.rect.x + near_p.rect.w * 0.5f, 400.0f - 100.0f, 0.01f));

		// Distance still drives the recorded device depth even though the screen position does not.
		PR_EXPECT(far_p.clip_depth > near_p.clip_depth);
	}

	PRUnitTest(AnchorsBehindTheCameraOrNearerThanTheNearPlaneAreCulled, Quick)
	{
		auto const vp = Viewport(800, 600, 96.0f, 0.0, PerspectiveCamera(Vec3{ 0, 0, 0 }, 1.0471976f, 1.0f, 100.0f));

		PR_EXPECT(Project(ERootPolicy::Overlay, WorldParams(Vec3{ 0, 0, -10 }), 20.0f, 10.0f, vp).visible == 0); // behind
		PR_EXPECT(Project(ERootPolicy::Overlay, WorldParams(Vec3{ 0, 0, 0 }), 20.0f, 10.0f, vp).visible == 0);   // at the eye
		PR_EXPECT(Project(ERootPolicy::Overlay, WorldParams(Vec3{ 0, 0, 0.5f }), 20.0f, 10.0f, vp).visible == 0); // inside the near plane
		PR_EXPECT(Project(ERootPolicy::Overlay, WorldParams(Vec3{ 0, 0, 500.0f }), 20.0f, 10.0f, vp).visible == 0); // beyond the far plane
		PR_EXPECT(Project(ERootPolicy::Overlay, WorldParams(Vec3{ 0, 0, 10.0f }), 20.0f, 10.0f, vp).visible == 1); // inside
	}

	PRUnitTest(ADepthOffsetPullsTheRecordedDepthTowardsTheCameraWithoutMovingTheRect, Quick)
	{
		auto const vp = Viewport(800, 600, 96.0f, 0.0, PerspectiveCamera());
		auto plain = WorldParams(Vec3{ 0, 0, 10 });
		auto offset = plain;
		offset.depth_offset = 2.0f;

		auto const a = Project(ERootPolicy::DepthTested, plain, 200.0f, 100.0f, vp);
		auto const b = Project(ERootPolicy::DepthTested, offset, 200.0f, 100.0f, vp);

		PR_EXPECT(Near(a.rect.x, b.rect.x) && Near(a.rect.y, b.rect.y) && Near(a.rect.w, b.rect.w));
		PR_EXPECT(Near(a.view_depth, b.view_depth)); // sizing is unaffected: the root did not move
		PR_EXPECT(b.clip_depth < a.clip_depth);      // but it wins the depth test against the same geometry
	}

	PRUnitTest(RootsProjectedEntirelyOutsideTheViewportAreCulled, Quick)
	{
		auto const vp = Viewport(800, 600, 96.0f, 0.0, OrthographicCamera(6.0f));

		// ortho_height 6 => the viewport spans x in [-4, +4] and y in [-3, +3] world units.
		PR_EXPECT(Project(ERootPolicy::Overlay, WorldParams(Vec3{ 100, 0, 10 }), 20.0f, 10.0f, vp).visible == 0);
		PR_EXPECT(Project(ERootPolicy::Overlay, WorldParams(Vec3{ 0, -100, 10 }), 20.0f, 10.0f, vp).visible == 0);

		// A root whose anchor is just outside but whose box still overlaps must survive.
		auto const edge = Project(ERootPolicy::Overlay, WorldParams(Vec3{ 4.05f, 0, 10 }), 200.0f, 10.0f, vp);
		PR_EXPECT(edge.visible == 1);
	}

	PRUnitTest(EveryAnchorPointCombinationOffsetsTheProjectedRectDeterministically, Quick)
	{
		auto const vp = Viewport(800, 600, 96.0f, 0.0, PerspectiveCamera());
		auto const w = 200.0f, h = 100.0f;

		auto place = [&](EAnchorPoint ah, EAnchorPoint av)
		{
			auto world = WorldParams(Vec3{ 0, 0, 10 });
			world.anchor_h = ah;
			world.anchor_v = av;
			return Project(ERootPolicy::Overlay, world, w, h, vp);
		};

		// The anchor projects to the viewport centre, so Min puts the box's left/top edge there,
		// Centre centres it, and Max puts its right/bottom edge there.
		PR_EXPECT(Near(place(EAnchorPoint::Min, EAnchorPoint::Min).rect.x, 400.0f));
		PR_EXPECT(Near(place(EAnchorPoint::Centre, EAnchorPoint::Min).rect.x, 400.0f - w * 0.5f));
		PR_EXPECT(Near(place(EAnchorPoint::Max, EAnchorPoint::Min).rect.x, 400.0f - w));
		PR_EXPECT(Near(place(EAnchorPoint::Min, EAnchorPoint::Min).rect.y, 300.0f));
		PR_EXPECT(Near(place(EAnchorPoint::Min, EAnchorPoint::Centre).rect.y, 300.0f - h * 0.5f));
		PR_EXPECT(Near(place(EAnchorPoint::Min, EAnchorPoint::Max).rect.y, 300.0f - h));
	}

	PRUnitTest(NormalisedDeviceDepthIsMonotonicAndClampedToTheDepthBufferDomain, Quick)
	{
		for (auto const& cam : { PerspectiveCamera(Vec3{0,0,0}, 1.0471976f, 1.0f, 100.0f), OrthographicCamera(6.0f, Vec3{0,0,0}, 1.0f, 100.0f) })
		{
			PR_EXPECT(Near(NormalisedDeviceDepth(cam, 1.0f), 0.0f, 1e-5f));
			PR_EXPECT(Near(NormalisedDeviceDepth(cam, 100.0f), 1.0f, 1e-5f));
			PR_EXPECT(Near(NormalisedDeviceDepth(cam, 0.5f), 0.0f, 1e-5f));   // clamped, not negative
			PR_EXPECT(Near(NormalisedDeviceDepth(cam, 1000.0f), 1.0f, 1e-5f)); // clamped, not > 1

			auto previous = -1.0f;
			for (auto d = 1.0f; d <= 100.0f; d += 4.5f)
			{
				auto const z = NormalisedDeviceDepth(cam, d);
				PR_EXPECT(z >= previous);
				PR_EXPECT(z >= 0.0f && z <= 1.0f);
				previous = z;
			}
		}
	}

	#pragma endregion
	#pragma region Sizing

	PRUnitTest(ConstantDipSizingKeepsAWorldRootTheSameApparentSizeAtEveryDistance, Quick)
	{
		auto const vp = Viewport(800, 600, 96.0f, 0.0, PerspectiveCamera());

		auto const near_p = Project(ERootPolicy::Overlay, WorldParams(Vec3{ 0, 0, 5 }), 200.0f, 100.0f, vp);
		auto const far_p = Project(ERootPolicy::Overlay, WorldParams(Vec3{ 0, 0, 500 }), 200.0f, 100.0f, vp);

		PR_EXPECT(Near(near_p.scale, 1.0f));
		PR_EXPECT(Near(far_p.scale, 1.0f));
		PR_EXPECT(Near(near_p.rect.w, far_p.rect.w));
		PR_EXPECT(Near(near_p.rect.h, far_p.rect.h));
	}

	PRUnitTest(WorldUnitSizingShrinksWithDistanceUnderPerspective, Quick)
	{
		auto const vp = Viewport(800, 600, 96.0f, 0.0, PerspectiveCamera());
		auto world = WorldParams(Vec3{ 0, 0, 10 });
		world.sizing = EWorldSizing::WorldUnits;

		auto const at_10 = Project(ERootPolicy::Overlay, world, 200.0f, 100.0f, vp);
		world.anchor = Vec3{ 0, 0, 20 };
		auto const at_20 = Project(ERootPolicy::Overlay, world, 200.0f, 100.0f, vp);

		// Twice the distance is exactly half the apparent size under a perspective projection.
		PR_EXPECT(Near(at_20.scale, at_10.scale * 0.5f, 1e-4f));
		PR_EXPECT(Near(at_20.rect.w, at_10.rect.w * 0.5f, 0.01f));
		PR_EXPECT(Near(at_20.rect.h, at_10.rect.h * 0.5f, 0.01f));
	}

	PRUnitTest(WorldUnitSizingIsExactForAKnownOrthographicConfiguration, Quick)
	{
		// ortho_height 6 over 600px is 100px per world unit; at 96 dpi that is 100 DIPs per world
		// unit, so world_units_per_dip 0.01 must produce a scale of exactly 1.
		auto const vp = Viewport(800, 600, 96.0f, 0.0, OrthographicCamera(6.0f));
		PR_EXPECT(Near(DipsPerWorldUnit(vp.camera, vp, 10.0f), 100.0f, 0.01f));

		auto world = WorldParams(Vec3{ 0, 0, 10 });
		world.sizing = EWorldSizing::WorldUnits;
		world.world_units_per_dip = 0.01f;

		auto const p = Project(ERootPolicy::Overlay, world, 200.0f, 100.0f, vp);
		PR_EXPECT(Near(p.scale, 1.0f, 1e-4f));
		PR_EXPECT(Near(p.rect.w, 200.0f, 0.01f));

		// Halving world_units_per_dip halves the world size of one DIP, hence halves the scale.
		world.world_units_per_dip = 0.005f;
		PR_EXPECT(Near(Project(ERootPolicy::Overlay, world, 200.0f, 100.0f, vp).scale, 0.5f, 1e-4f));
	}

	PRUnitTest(WorldUnitSizingScalesTheWholeSubtreeIncludingTextAndBorders, Quick)
	{
		auto engine = UiEngine(MakeConfig());
		auto world = WorldParams(Vec3{ 0, 0, 10 });
		world.sizing = EWorldSizing::WorldUnits;
		world.world_units_per_dip = 0.02f; // 2 DIP-scale under the ortho config below

		auto style = MakeStyle(1);
		for (auto& v : style.visuals)
		{
			v.border_thickness = 3.0f;
			v.corner_radius = 5.0f;
		}

		auto b = TxnBuilder{};
		b.AddStyle(style);
		auto root = MakeWorldRoot(1, ERootPolicy::Overlay, 200.0f, 100.0f, world);
		root.style_id = 1;
		b.Upsert(root);
		auto child = MakeControl(2, 1, EControlType::Text, ELayoutMode::Overlay, Lp(100.0f, 20.0f));
		std::tie(child.text_offset, child.text_length) = b.AddText("Hi");
		child.style_id = 1;
		b.Upsert(child);
		engine.TransactionApply(b.Build(0, 1));

		engine.Update(Viewport(800, 600, 96.0f, 0.0, OrthographicCamera(6.0f)));

		auto const& packet = engine.DrawPackets();
		PR_EXPECT(packet.groups.size() == 1);

		// The root box, its child's box, the border thickness, the corner radius and the font size
		// all pick up the same factor, so the subtree is a faithful magnification, not a stretch.
		auto const& root_item = packet.items[packet.groups[0].first_item];
		PR_EXPECT(Near(root_item.bounds.w, 400.0f, 0.05f));
		PR_EXPECT(Near(root_item.bounds.h, 200.0f, 0.05f));
		PR_EXPECT(Near(root_item.border_thickness, 6.0f, 0.01f));
		PR_EXPECT(Near(root_item.corner_radius, 10.0f, 0.01f));

		auto const text = std::find_if(packet.items.begin(), packet.items.end(), [](DrawItem const& i) { return i.primitive == EVisualPrimitive::TextPresenter; });
		PR_EXPECT(text != packet.items.end());
		PR_EXPECT(Near(text->bounds.w, 200.0f, 0.05f));
		PR_EXPECT(text->font_size > 0.0f);
	}

	PRUnitTest(ScreenRootsAreBitForBitUnaffectedByTheWorldPipeline, Quick)
	{
		// The placement path is shared, so a screen root must come out with the identity placement
		// whether or not a camera is present; this is the guard on the M0-M5 baseline.
		auto const with_camera = ScreenRootPlacement(Rect{ 0, 0, 800, 600 });
		PR_EXPECT(with_camera.policy == ERootPolicy::Screen);
		PR_EXPECT(with_camera.visible == 1);
		PR_EXPECT(with_camera.scale == 1.0f);
		PR_EXPECT(with_camera.view_depth == 0.0f);
		PR_EXPECT(with_camera.clip_depth == 0.0f);

		auto engine_a = UiEngine(MakeConfig());
		ApplyControls(engine_a, { MakeControl(1, 0, EControlType::Root, ELayoutMode::Overlay, Lp(200.0f, 100.0f)) });
		engine_a.Update(Viewport(800, 600));
		auto const a = engine_a.DrawPackets();

		auto engine_b = UiEngine(MakeConfig());
		ApplyControls(engine_b, { MakeControl(1, 0, EControlType::Root, ELayoutMode::Overlay, Lp(200.0f, 100.0f)) });
		engine_b.Update(Viewport(800, 600, 96.0f, 0.0, PerspectiveCamera()));
		auto const& c = engine_b.DrawPackets();

		PR_EXPECT(a.items.size() == c.items.size());
		for (auto i = std::size_t{}; i != a.items.size(); ++i)
		{
			PR_EXPECT(a.items[i].bounds.x == c.items[i].bounds.x);
			PR_EXPECT(a.items[i].bounds.y == c.items[i].bounds.y);
			PR_EXPECT(a.items[i].bounds.w == c.items[i].bounds.w);
			PR_EXPECT(a.items[i].bounds.h == c.items[i].bounds.h);
		}
	}

	#pragma endregion
	#pragma region Draw groups, policy routing and ordering

	PRUnitTest(EachRootEmitsExactlyOneDrawGroupCarryingItsPolicyAndFadeParameters, Quick)
	{
		auto engine = UiEngine(MakeConfig());
		auto faded = WorldParams(Vec3{ 0, 0, 10 });
		faded.occlusion_min_opacity = 0.125f;
		faded.occlusion_fade_depth = 2.5f;
		faded.occlusion_depth_bias = 0.03125f;

		ApplyControls(engine, {
			MakeControl(1, 0, EControlType::Root, ELayoutMode::Overlay, Lp(200.0f, 100.0f)),
			MakeWorldRoot(2, ERootPolicy::DepthTested, 200.0f, 100.0f, WorldParams(Vec3{ 0, 0, 10 })),
			MakeWorldRoot(3, ERootPolicy::OcclusionFaded, 200.0f, 100.0f, faded),
			MakeWorldRoot(4, ERootPolicy::Overlay, 200.0f, 100.0f, WorldParams(Vec3{ 0, 0, 10 })),
		});
		engine.Update(Viewport(800, 600, 96.0f, 0.0, PerspectiveCamera()));

		auto const& packet = engine.DrawPackets();
		PR_EXPECT(packet.groups.size() == 4);

		PR_EXPECT(FindGroup(packet, 1)->policy == ERootPolicy::Screen);
		PR_EXPECT(FindGroup(packet, 2)->policy == ERootPolicy::DepthTested);
		PR_EXPECT(FindGroup(packet, 4)->policy == ERootPolicy::Overlay);

		auto const* g3 = FindGroup(packet, 3);
		PR_EXPECT(g3->policy == ERootPolicy::OcclusionFaded);
		PR_EXPECT(g3->occlusion_min_opacity == 0.125f);
		PR_EXPECT(g3->occlusion_fade_depth == 2.5f);
		PR_EXPECT(g3->occlusion_depth_bias == 0.03125f);
		PR_EXPECT(Near(g3->view_depth, 10.0f));
		PR_EXPECT(g3->clip_depth > 0.0f && g3->clip_depth < 1.0f);

		// Screen roots carry no depth at all, so they can never be mistaken for a world root.
		PR_EXPECT(FindGroup(packet, 1)->clip_depth == 0.0f);
		PR_EXPECT(FindGroup(packet, 1)->view_depth == 0.0f);
	}

	PRUnitTest(DrawGroupsPartitionTheItemArrayContiguouslyAndInRootOrder, Quick)
	{
		auto engine = UiEngine(MakeConfig());
		ApplyControls(engine, {
			MakeControl(1, 0, EControlType::Root, ELayoutMode::Overlay, Lp(200.0f, 100.0f)),
			MakeControl(10, 1, EControlType::Panel, ELayoutMode::Overlay, Lp(50.0f, 50.0f)),
			MakeWorldRoot(2, ERootPolicy::Overlay, 200.0f, 100.0f, WorldParams(Vec3{ 0, 0, 10 })),
			MakeControl(20, 2, EControlType::Panel, ELayoutMode::Overlay, Lp(50.0f, 50.0f)),
			MakeWorldRoot(3, ERootPolicy::DepthTested, 200.0f, 100.0f, WorldParams(Vec3{ 0, 0, 10 })),
		});
		engine.Update(Viewport(800, 600, 96.0f, 0.0, PerspectiveCamera()));

		auto const& packet = engine.DrawPackets();
		PR_EXPECT(packet.groups.size() == 3);

		// Contiguous, non-overlapping and exhaustive: every item belongs to exactly one group, and
		// the groups appear in the same root order the tree walks in.
		auto expected_first = std::uint32_t{};
		for (auto const& g : packet.groups)
		{
			PR_EXPECT(g.first_item == expected_first);
			PR_EXPECT(g.item_count != 0);
			expected_first += g.item_count;
		}
		PR_EXPECT(expected_first == packet.items.size());
		PR_EXPECT(packet.groups[0].root_id == 1);
		PR_EXPECT(packet.groups[1].root_id == 2);
		PR_EXPECT(packet.groups[2].root_id == 3);
	}

	PRUnitTest(CulledWorldRootsEmitNoDrawGroupOrItems, Quick)
	{
		auto engine = UiEngine(MakeConfig());
		ApplyControls(engine, {
			MakeControl(1, 0, EControlType::Root, ELayoutMode::Overlay, Lp(200.0f, 100.0f)),
			MakeWorldRoot(2, ERootPolicy::Overlay, 200.0f, 100.0f, WorldParams(Vec3{ 0, 0, -50 })),
		});
		engine.Update(Viewport(800, 600, 96.0f, 0.0, PerspectiveCamera()));

		auto const& packet = engine.DrawPackets();
		PR_EXPECT(packet.groups.size() == 1);
		PR_EXPECT(packet.groups[0].root_id == 1);
		PR_EXPECT(FindGroup(packet, 2) == nullptr);
	}

	PRUnitTest(WithoutACameraEveryWorldRootIsCulledAndOnlyScreenRootsSurvive, Quick)
	{
		// A screen-only host (or one whose camera is degenerate) never fills in ViewportState::camera;
		// world roots must vanish deterministically rather than projecting through a garbage basis.
		auto engine = UiEngine(MakeConfig());
		ApplyControls(engine, {
			MakeControl(1, 0, EControlType::Root, ELayoutMode::Overlay, Lp(200.0f, 100.0f)),
			MakeWorldRoot(2, ERootPolicy::Overlay, 200.0f, 100.0f, WorldParams(Vec3{ 0, 0, 10 })),
			MakeWorldRoot(3, ERootPolicy::DepthTested, 200.0f, 100.0f, WorldParams(Vec3{ 0, 0, 10 })),
		});
		engine.Update(Viewport(800, 600));

		auto const& packet = engine.DrawPackets();
		PR_EXPECT(packet.groups.size() == 1);
		PR_EXPECT(packet.groups[0].policy == ERootPolicy::Screen);
	}

	PRUnitTest(EachHostPassSelectsExactlyOneRootPolicyAndScreenRootsBelongOnlyToFinalOverlay, Quick)
	{
		PR_EXPECT(Renderer::PolicyForPass(EPass::FinalOverlay) == ERootPolicy::Screen);
		PR_EXPECT(Renderer::PolicyForPass(EPass::DepthTested) == ERootPolicy::DepthTested);
		PR_EXPECT(Renderer::PolicyForPass(EPass::OcclusionFaded) == ERootPolicy::OcclusionFaded);
		PR_EXPECT(Renderer::PolicyForPass(EPass::Overlay) == ERootPolicy::Overlay);

		// Prepare records no geometry at all, so asking it for a policy is a programming error.
		PR_THROWS(Renderer::PolicyForPass(EPass::Prepare), std::exception);

		// The mapping is injective: no two drawing passes can ever pick up the same groups, which
		// is what keeps a screen root out of the three world stages and vice versa.
		auto const policies = {
			Renderer::PolicyForPass(EPass::FinalOverlay),
			Renderer::PolicyForPass(EPass::DepthTested),
			Renderer::PolicyForPass(EPass::OcclusionFaded),
			Renderer::PolicyForPass(EPass::Overlay),
		};
		for (auto a = policies.begin(); a != policies.end(); ++a)
			for (auto b = a + 1; b != policies.end(); ++b)
				PR_EXPECT(*a != *b);
	}

	#pragma endregion
	#pragma region Semantics

	PRUnitTest(CulledWorldRootsAndTheirSubtreesAreReportedOffscreen, Quick)
	{
		auto engine = UiEngine(MakeConfig());
		ApplyControls(engine, {
			MakeWorldRoot(1, ERootPolicy::Overlay, 200.0f, 100.0f, WorldParams(Vec3{ 0, 0, -50 })),
			MakeControl(2, 1, EControlType::Button, ELayoutMode::Overlay, Lp(50.0f, 20.0f)),
		});
		engine.Update(Viewport(800, 600, 96.0f, 0.0, PerspectiveCamera()));

		auto nodes = std::vector<SemanticNode>(engine.SemanticCount());
		auto text = std::vector<char>(engine.SemanticTextBytesPending());
		engine.SemanticsCopy(nodes, text);

		PR_EXPECT(nodes.size() == 2);
		for (auto const& n : nodes)
			PR_EXPECT((n.state_flags & static_cast<std::uint32_t>(ESemanticState::Offscreen)) != 0);
	}

	PRUnitTest(VisibleWorldRootsReportTheirProjectedScreenBounds, Quick)
	{
		auto engine = UiEngine(MakeConfig());
		ApplyControls(engine, { MakeWorldRoot(1, ERootPolicy::Overlay, 200.0f, 100.0f, WorldParams(Vec3{ 0, 0, 10 })) });
		engine.Update(Viewport(800, 600, 96.0f, 0.0, PerspectiveCamera()));

		auto nodes = std::vector<SemanticNode>(engine.SemanticCount());
		auto text = std::vector<char>(engine.SemanticTextBytesPending());
		engine.SemanticsCopy(nodes, text);

		PR_EXPECT(nodes.size() == 1);
		PR_EXPECT((nodes[0].state_flags & static_cast<std::uint32_t>(ESemanticState::Offscreen)) == 0);

		// Assistive technology reads screen-space DIPs; the projected rect is what it must see,
		// not the root's authored local box at the origin.
		PR_EXPECT(Near(nodes[0].bounds.x, 300.0f));
		PR_EXPECT(Near(nodes[0].bounds.y, 250.0f));
		PR_EXPECT(Near(nodes[0].bounds.w, 200.0f));
	}

	#pragma endregion
	#pragma region Bridge ABI

	PRUnitTest(TheHostBridgeAbiCarriesCameraAndResolvedDepthAtAFixedLayout, Quick)
	{
		// The bridge is a private, fixed-layout contract resolved by GetProcAddress across two
		// independently-built DLLs, so every field's placement is part of the contract.
		static_assert(std::is_standard_layout_v<Camera>, "Camera must be standard layout to cross the DLL boundary");
		static_assert(std::is_standard_layout_v<ResolvedDepth>, "ResolvedDepth must be standard layout to cross the DLL boundary");
		static_assert(std::is_standard_layout_v<Pass>, "Pass must be standard layout to cross the DLL boundary");
		static_assert(sizeof(Camera) == 72, "Camera layout changed; bump HostApiVersion");
		static_assert(sizeof(ResolvedDepth) == 24, "ResolvedDepth layout changed; bump HostApiVersion");

		// Camera and ResolvedDepth are appended after every pre-M8 field, so the offsets of the
		// original members are untouched and a layout regression is caught here, not at runtime.
		static_assert(offsetof(Pass, m_camera) + sizeof(Camera) <= sizeof(Pass), "Pass must contain its camera");
		static_assert(offsetof(Pass, m_camera) < offsetof(Pass, m_resolved_depth), "camera precedes resolved depth in Pass");
		static_assert(offsetof(Pass, m_resolved_depth) + sizeof(ResolvedDepth) <= sizeof(Pass), "ResolvedDepth must fit inside Pass");

		// The version pair must move together with the layout; M8 is the second revision.
		PR_EXPECT(HostStructVersion == 2U);
		PR_EXPECT(HostApiVersion == 0x00020000U);

		// EPass is a closed set of five host stages; the renderer switches exhaustively over it.
		PR_EXPECT(static_cast<int>(EPass::Prepare) == 0);
		PR_EXPECT(static_cast<int>(EPass::DepthTested) == 1);
		PR_EXPECT(static_cast<int>(EPass::OcclusionFaded) == 2);
		PR_EXPECT(static_cast<int>(EPass::Overlay) == 3);
		PR_EXPECT(static_cast<int>(EPass::FinalOverlay) == 4);
	}

	PRUnitTest(ThePublicWireAbiCarriesWorldAndCameraStateAtAFixedLayout, Quick)
	{
		static_assert(std::is_standard_layout_v<WorldRootParams>, "WorldRootParams crosses the C ABI");
		static_assert(std::is_standard_layout_v<CameraState>, "CameraState crosses the C ABI");
		static_assert(sizeof(WorldRootParams) == 44, "WorldRootParams layout changed; bump VIEW3D_UI_API_VERSION");
		static_assert(sizeof(CameraState) == 60, "CameraState layout changed; bump VIEW3D_UI_API_VERSION");
		static_assert(sizeof(Vec3) == 12, "Vec3 must be three tightly-packed floats");

		// Both new blocks are appended after every pre-M8 member, so managed callers that only write
		// the pre-M8 prefix still produce a well-defined (camera-absent, screen-only) value once
		// zero-initialised. Trailing padding means these are bounds, not exact end-of-struct equality.
		static_assert(offsetof(ControlDesc, world) > offsetof(ControlDesc, font_resource_id), "WorldRootParams must follow every pre-M8 ControlDesc member");
		static_assert(offsetof(ControlDesc, world) + sizeof(WorldRootParams) <= sizeof(ControlDesc), "WorldRootParams must fit inside ControlDesc");
		static_assert(offsetof(ViewportState, camera) > offsetof(ViewportState, time_ms), "CameraState must follow every pre-M8 ViewportState member");
		static_assert(offsetof(ViewportState, camera) + sizeof(CameraState) <= sizeof(ViewportState), "CameraState must fit inside ViewportState");

		PR_EXPECT(VIEW3D_UI_STRUCT_VERSION == 3U);
		PR_EXPECT(VIEW3D_UI_API_VERSION == 0x00030000U);

		// The policy enum is closed and Screen keeps value 0, so a zero-initialised descriptor is
		// still a screen root exactly as it was before M8.
		PR_EXPECT(static_cast<int>(ERootPolicy::Screen) == 0);
		PR_EXPECT(static_cast<int>(ERootPolicy::Count) == 4);
		PR_EXPECT(!IsWorldPolicy(ERootPolicy::Screen));
		PR_EXPECT(IsWorldPolicy(ERootPolicy::DepthTested));
		PR_EXPECT(IsWorldPolicy(ERootPolicy::OcclusionFaded));
		PR_EXPECT(IsWorldPolicy(ERootPolicy::Overlay));
	}

	#pragma endregion
	#pragma region Determinism and churn

	PRUnitTest(RepeatedUpdatesWithUnchangedInputProduceByteIdenticalWorldPlacements, Quick)
	{
		auto engine = UiEngine(MakeConfig());
		ApplyControls(engine, {
			MakeWorldRoot(1, ERootPolicy::OcclusionFaded, 200.0f, 100.0f, WorldParams(Vec3{ 1.5f, -2.25f, 12.5f })),
			MakeControl(2, 1, EControlType::Panel, ELayoutMode::Overlay, Lp(60.0f, 30.0f)),
		});

		auto const vp = Viewport(1024, 768, 144.0f, 250.0, PerspectiveCamera(Vec3{ -1, 0.5f, -3 }));
		engine.Update(vp);
		auto const first = engine.DrawPackets();
		engine.Update(vp);
		auto const& second = engine.DrawPackets();

		PR_EXPECT(first.groups.size() == second.groups.size());
		for (auto i = std::size_t{}; i != first.groups.size(); ++i)
		{
			PR_EXPECT(first.groups[i].root_id == second.groups[i].root_id);
			PR_EXPECT(first.groups[i].clip_depth == second.groups[i].clip_depth);
			PR_EXPECT(first.groups[i].view_depth == second.groups[i].view_depth);
		}

		PR_EXPECT(first.items.size() == second.items.size());
		for (auto i = std::size_t{}; i != first.items.size(); ++i)
		{
			PR_EXPECT(first.items[i].bounds.x == second.items[i].bounds.x);
			PR_EXPECT(first.items[i].bounds.y == second.items[i].bounds.y);
			PR_EXPECT(first.items[i].bounds.w == second.items[i].bounds.w);
			PR_EXPECT(first.items[i].bounds.h == second.items[i].bounds.h);
		}
	}

	PRUnitTest(ResizingTheViewportReprojectsWorldRootsWithoutInvalidatingTheTree, Quick)
	{
		auto engine = UiEngine(MakeConfig());
		ApplyControls(engine, { MakeWorldRoot(1, ERootPolicy::Overlay, 200.0f, 100.0f, WorldParams(Vec3{ 0, 0, 10 })) });

		engine.Update(Viewport(800, 600, 96.0f, 0.0, PerspectiveCamera()));
		auto const before = engine.DrawPackets().items[0].bounds;

		engine.Update(Viewport(1600, 400, 96.0f, 0.0, PerspectiveCamera()));
		auto const after = engine.DrawPackets().items[0].bounds;

		// The centre tracks the new viewport centre; ConstantDip sizing means the box itself does not change size.
		PR_EXPECT(Near(before.x + before.w * 0.5f, 400.0f));
		PR_EXPECT(Near(after.x + after.w * 0.5f, 800.0f));
		PR_EXPECT(Near(after.y + after.h * 0.5f, 200.0f));
		PR_EXPECT(Near(after.w, before.w));
	}

	PRUnitTest(ChangingAWorldRootsPolicyMovesItsGroupToTheOtherStage, Quick)
	{
		auto engine = UiEngine(MakeConfig());
		ApplyControls(engine, { MakeWorldRoot(1, ERootPolicy::DepthTested, 200.0f, 100.0f, WorldParams(Vec3{ 0, 0, 10 })) });
		engine.Update(Viewport(800, 600, 96.0f, 0.0, PerspectiveCamera()));
		PR_EXPECT(engine.DrawPackets().groups[0].policy == ERootPolicy::DepthTested);

		// An upsert that only changes the policy must retarget the group without churning ids.
		ApplyControls(engine, { MakeWorldRoot(1, ERootPolicy::OcclusionFaded, 200.0f, 100.0f, WorldParams(Vec3{ 0, 0, 10 })) }, 1);
		engine.Update(Viewport(800, 600, 96.0f, 0.0, PerspectiveCamera()));

		auto const& packet = engine.DrawPackets();
		PR_EXPECT(packet.groups.size() == 1);
		PR_EXPECT(packet.groups[0].root_id == 1);
		PR_EXPECT(packet.groups[0].policy == ERootPolicy::OcclusionFaded);
	}

	PRUnitTest(RemovingAWorldRootLeavesNoStaleGroupBehind, Quick)
	{
		auto engine = UiEngine(MakeConfig());
		ApplyControls(engine, {
			MakeControl(1, 0, EControlType::Root, ELayoutMode::Overlay, Lp(200.0f, 100.0f)),
			MakeWorldRoot(2, ERootPolicy::Overlay, 200.0f, 100.0f, WorldParams(Vec3{ 0, 0, 10 })),
		});
		engine.Update(Viewport(800, 600, 96.0f, 0.0, PerspectiveCamera()));
		PR_EXPECT(engine.DrawPackets().groups.size() == 2);

		auto b = TxnBuilder{};
		b.Remove(2);
		engine.TransactionApply(b.Build(1, 2));
		engine.Update(Viewport(800, 600, 96.0f, 0.0, PerspectiveCamera()));

		auto const& packet = engine.DrawPackets();
		PR_EXPECT(packet.groups.size() == 1);
		PR_EXPECT(packet.groups[0].root_id == 1);
	}

	#pragma region Live host wiring

	namespace
	{
		// A device-backed rig carrying everything the three world stages need: a single-sample colour
		// target (the host's post-alpha swap target), a depth-stencil with a DSV (the host's live
		// scene depth), and a separate single-sample resolved-depth texture (what View3D hands the
		// provider for occlusion fading). Returns nullopt when no adapter, hardware or WARP, exists.
		struct WorldFixture
		{
			Microsoft::WRL::ComPtr<ID3D12Device> device;
			Microsoft::WRL::ComPtr<ID3D12CommandAllocator> allocator;
			Microsoft::WRL::ComPtr<ID3D12GraphicsCommandList> command_list;
			Microsoft::WRL::ComPtr<ID3D12DescriptorHeap> rtv_heap;
			Microsoft::WRL::ComPtr<ID3D12DescriptorHeap> dsv_heap;
			Microsoft::WRL::ComPtr<ID3D12Resource> render_target;
			Microsoft::WRL::ComPtr<ID3D12Resource> depth_stencil;
			Microsoft::WRL::ComPtr<ID3D12Resource> resolved_depth;
			D3D12_CPU_DESCRIPTOR_HANDLE rtv{};
			D3D12_CPU_DESCRIPTOR_HANDLE dsv{};
		};

		std::optional<WorldFixture> TryBuildWorldFixture()
		{
			Microsoft::WRL::ComPtr<ID3D12Device> device;
			if (FAILED(D3D12CreateDevice(nullptr, D3D_FEATURE_LEVEL_11_0, IID_PPV_ARGS(&device))))
			{
				Microsoft::WRL::ComPtr<IDXGIFactory4> factory;
				Microsoft::WRL::ComPtr<IDXGIAdapter> warp;
				if (FAILED(CreateDXGIFactory1(IID_PPV_ARGS(&factory))) || FAILED(factory->EnumWarpAdapter(IID_PPV_ARGS(&warp))) || FAILED(D3D12CreateDevice(warp.Get(), D3D_FEATURE_LEVEL_11_0, IID_PPV_ARGS(&device))))
					return std::nullopt;
			}

			auto fx = WorldFixture{};
			fx.device = device;
			if (FAILED(device->CreateCommandAllocator(D3D12_COMMAND_LIST_TYPE_DIRECT, IID_PPV_ARGS(&fx.allocator))))
				return std::nullopt;
			if (FAILED(device->CreateCommandList(0, D3D12_COMMAND_LIST_TYPE_DIRECT, fx.allocator.Get(), nullptr, IID_PPV_ARGS(&fx.command_list))))
				return std::nullopt;

			auto heap_props = D3D12_HEAP_PROPERTIES{};
			heap_props.Type = D3D12_HEAP_TYPE_DEFAULT;

			auto rtv_heap_desc = D3D12_DESCRIPTOR_HEAP_DESC{ .Type = D3D12_DESCRIPTOR_HEAP_TYPE_RTV, .NumDescriptors = 1 };
			auto dsv_heap_desc = D3D12_DESCRIPTOR_HEAP_DESC{ .Type = D3D12_DESCRIPTOR_HEAP_TYPE_DSV, .NumDescriptors = 1 };
			if (FAILED(device->CreateDescriptorHeap(&rtv_heap_desc, IID_PPV_ARGS(&fx.rtv_heap))) || FAILED(device->CreateDescriptorHeap(&dsv_heap_desc, IID_PPV_ARGS(&fx.dsv_heap))))
				return std::nullopt;

			auto tex = D3D12_RESOURCE_DESC{};
			tex.Dimension = D3D12_RESOURCE_DIMENSION_TEXTURE2D;
			tex.Width = 256;
			tex.Height = 128;
			tex.DepthOrArraySize = 1;
			tex.MipLevels = 1;
			tex.SampleDesc.Count = 1;

			tex.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
			tex.Flags = D3D12_RESOURCE_FLAG_ALLOW_RENDER_TARGET;
			auto rt_clear = D3D12_CLEAR_VALUE{ .Format = DXGI_FORMAT_R8G8B8A8_UNORM };
			if (FAILED(device->CreateCommittedResource(&heap_props, D3D12_HEAP_FLAG_NONE, &tex, D3D12_RESOURCE_STATE_RENDER_TARGET, &rt_clear, IID_PPV_ARGS(&fx.render_target))))
				return std::nullopt;

			tex.Format = DXGI_FORMAT_D32_FLOAT;
			tex.Flags = D3D12_RESOURCE_FLAG_ALLOW_DEPTH_STENCIL;
			auto ds_clear = D3D12_CLEAR_VALUE{ .Format = DXGI_FORMAT_D32_FLOAT, .DepthStencil = { 1.0f, 0 } };
			if (FAILED(device->CreateCommittedResource(&heap_props, D3D12_HEAP_FLAG_NONE, &tex, D3D12_RESOURCE_STATE_DEPTH_WRITE, &ds_clear, IID_PPV_ARGS(&fx.depth_stencil))))
				return std::nullopt;

			// The resolved copy is an ordinary shader resource: no depth flag, no clear value, which
			// is exactly how View3D's own single-sample resolved-depth resource is described.
			tex.Format = DXGI_FORMAT_R32_FLOAT;
			tex.Flags = D3D12_RESOURCE_FLAG_NONE;
			if (FAILED(device->CreateCommittedResource(&heap_props, D3D12_HEAP_FLAG_NONE, &tex, D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE, nullptr, IID_PPV_ARGS(&fx.resolved_depth))))
				return std::nullopt;

			fx.rtv = fx.rtv_heap->GetCPUDescriptorHandleForHeapStart();
			fx.dsv = fx.dsv_heap->GetCPUDescriptorHandleForHeapStart();
			device->CreateRenderTargetView(fx.render_target.Get(), nullptr, fx.rtv);
			device->CreateDepthStencilView(fx.depth_stencil.Get(), nullptr, fx.dsv);
			return fx;
		}

		// A Pass describing one host stage exactly as V3dWindow::RecordUIProvider populates it.
		Pass MakeWorldPass(WorldFixture const& fx, EPass stage, CameraState const& camera)
		{
			auto const depth_stage = stage == EPass::DepthTested;

			auto pass = Pass{};
			pass.m_header = { sizeof(Pass), HostStructVersion };
			pass.m_pass = stage;
			pass.m_command_list = fx.command_list.Get();
			pass.m_colour_target = fx.render_target.Get();
			pass.m_depth_target = fx.depth_stencil.Get();
			pass.m_rtv = fx.rtv;
			pass.m_dsv = depth_stage ? fx.dsv : D3D12_CPU_DESCRIPTOR_HANDLE{};
			pass.m_width = 256;
			pass.m_height = 128;
			pass.m_colour_format = DXGI_FORMAT_R8G8B8A8_UNORM;
			pass.m_depth_format = DXGI_FORMAT_D32_FLOAT;
			pass.m_sample_count = 1;
			pass.m_sample_quality = 0;
			pass.m_dpi_x = 96.0f;
			pass.m_dpi_y = 96.0f;
			pass.m_viewport = D3D12_VIEWPORT{ 0.0f, 0.0f, 256.0f, 128.0f, 0.0f, 1.0f };
			pass.m_scissor = D3D12_RECT{ 0, 0, 256, 128 };
			pass.m_camera = Camera{
				.m_position = { camera.position.x, camera.position.y, camera.position.z },
				.m_right = { -1.0f, 0.0f, 0.0f },
				.m_up = { 0.0f, 1.0f, 0.0f },
				.m_forward = { camera.forward.x, camera.forward.y, camera.forward.z },
				.m_near_plane = camera.near_plane,
				.m_far_plane = camera.far_plane,
				.m_fov_y_rad = camera.fov_y_rad,
				.m_ortho_height = camera.ortho_height,
				.m_orthographic = camera.projection == EProjection::Orthographic ? 1U : 0U,
				.m_valid = static_cast<std::uint32_t>(camera.valid),
			};

			// Only the pass that samples it is offered the resolved depth, mirroring the host.
			if (stage == EPass::OcclusionFaded)
			{
				pass.m_resolved_depth = ResolvedDepth{
					.m_resource = fx.resolved_depth.Get(),
					.m_srv_format = DXGI_FORMAT_R32_FLOAT,
					.m_width = 256,
					.m_height = 128,
				};
			}
			return pass;
		}
	}

	PRUnitTest(EveryHostStageRecordsAgainstARealDeviceWithoutTouchingHostState, Quick)
	{
		auto fixture = TryBuildWorldFixture();
		if (!fixture.has_value())
			return; // no D3D12 adapter (hardware or WARP) available in this environment; nothing to verify

		auto const camera = PerspectiveCamera(Vec3{ 0, 0, 0 });

		// One root per policy plus a screen root, all visible, each with text so both the box and
		// the glyph pipelines are exercised in every stage.
		auto engine = UiEngine(MakeConfig());
		auto b = TxnBuilder{};
		auto next_child = ControlId{ 100 };
		b.Upsert(MakeControl(1, 0, EControlType::Root, ELayoutMode::Overlay, Lp(120.0f, 60.0f)));
		for (auto policy : { ERootPolicy::DepthTested, ERootPolicy::OcclusionFaded, ERootPolicy::Overlay })
		{
			auto const root_id = static_cast<ControlId>(2 + static_cast<int>(policy));
			b.Upsert(MakeWorldRoot(root_id, policy, 120.0f, 60.0f, WorldParams(Vec3{ 0, 0, 10 })));

			auto text = MakeControl(next_child++, root_id, EControlType::Text, ELayoutMode::Overlay, Lp(100.0f, 20.0f));
			std::tie(text.text_offset, text.text_length) = b.AddText("Hi");
			b.Upsert(text);
		}
		auto screen_text = MakeControl(next_child++, 1, EControlType::Text, ELayoutMode::Overlay, Lp(100.0f, 20.0f));
		std::tie(screen_text.text_offset, screen_text.text_length) = b.AddText("Hi");
		b.Upsert(screen_text);
		engine.TransactionApply(b.Build(0, 1));

		engine.Update(Viewport(256, 128, 96.0f, 0.0, camera));
		auto const& packet = engine.DrawPackets();
		PR_EXPECT(packet.groups.size() == 4);

		auto renderer = Renderer(fixture->device.Get(), MakeConfig());

		// Prepare must run first; it is what realises the device resources every later stage needs.
		auto prepare = MakeWorldPass(*fixture, EPass::Prepare, camera);
		PR_EXPECT(renderer.Record(prepare, packet) == EStatus::Success);

		// Every drawing stage records real work into the host's list, in host order. This is the
		// end-to-end proof that the world pipelines, the depth-bound PSO, the resolved-depth SRV
		// and the root-constant layout are all mutually consistent on a real device.
		for (auto stage : { EPass::DepthTested, EPass::OcclusionFaded, EPass::Overlay, EPass::FinalOverlay })
		{
			auto pass = MakeWorldPass(*fixture, stage, camera);
			PR_EXPECT(renderer.Record(pass, packet) == EStatus::Success);
		}

		// The provider must never close, execute or present the host's list; it is still open and
		// still owned by the test, which is what lets the host keep its command-list ordering.
		PR_EXPECT(SUCCEEDED(fixture->command_list->Close()));
	}

	PRUnitTest(TheDepthTestedStageDrawsNothingWithoutAHostDepthView, Quick)
	{
		auto fixture = TryBuildWorldFixture();
		if (!fixture.has_value())
			return;

		auto const camera = PerspectiveCamera();
		auto engine = UiEngine(MakeConfig());
		ApplyControls(engine, { MakeWorldRoot(1, ERootPolicy::DepthTested, 120.0f, 60.0f, WorldParams(Vec3{ 0, 0, 10 })) });
		engine.Update(Viewport(256, 128, 96.0f, 0.0, camera));

		auto renderer = Renderer(fixture->device.Get(), MakeConfig());
		auto prepare = MakeWorldPass(*fixture, EPass::Prepare, camera);
		PR_EXPECT(renderer.Record(prepare, engine.DrawPackets()) == EStatus::Success);

		// Depth testing without a depth view is not something the provider may emulate as an
		// overlay, so it degrades to drawing nothing rather than silently changing policy.
		auto pass = MakeWorldPass(*fixture, EPass::DepthTested, camera);
		pass.m_dsv = D3D12_CPU_DESCRIPTOR_HANDLE{};
		PR_EXPECT(renderer.Record(pass, engine.DrawPackets()) == EStatus::Success);

		// Likewise, occlusion fading is defined only against a resolved single-sample depth.
		auto faded = MakeWorldPass(*fixture, EPass::OcclusionFaded, camera);
		faded.m_resolved_depth = ResolvedDepth{};
		PR_EXPECT(renderer.Record(faded, engine.DrawPackets()) == EStatus::Success);

		PR_EXPECT(SUCCEEDED(fixture->command_list->Close()));
	}

	PRUnitTest(AMultiSampledOverlayTargetIsRefusedRatherThanMismatchingThePipeline, Quick)
	{
		auto fixture = TryBuildWorldFixture();
		if (!fixture.has_value())
			return;

		auto const camera = PerspectiveCamera();
		auto engine = UiEngine(MakeConfig());
		ApplyControls(engine, { MakeControl(1, 0, EControlType::Root, ELayoutMode::Overlay, Lp(120.0f, 60.0f)) });
		engine.Update(Viewport(256, 128, 96.0f, 0.0, camera));

		auto renderer = Renderer(fixture->device.Get(), MakeConfig());
		PR_EXPECT(renderer.Record(MakeWorldPass(*fixture, EPass::Prepare, camera), engine.DrawPackets()) == EStatus::Success);

		// The overlay pipelines are built once for the host's single-sample post-alpha target; a
		// multi-sampled one would be a silent pipeline mismatch, so it must be reported, not drawn.
		for (auto stage : { EPass::OcclusionFaded, EPass::Overlay, EPass::FinalOverlay })
		{
			auto pass = MakeWorldPass(*fixture, stage, camera);
			pass.m_sample_count = 4;
			PR_EXPECT(renderer.Record(pass, engine.DrawPackets()) == EStatus::InvalidArgument);
		}

		PR_EXPECT(SUCCEEDED(fixture->command_list->Close()));
	}

	#pragma endregion

	#pragma region Coordinate space

	namespace
	{
		// The bounds the engine laid out for 'id' this update, read back through the semantic
		// snapshot - the same window assistive technology and hit-testing see - or nullopt when
		// the control emitted no node.
		std::optional<Rect> LaidOutBounds(UiEngine& engine, ControlId id)
		{
			auto nodes = std::vector<SemanticNode>(engine.SemanticCount());
			auto text = std::vector<char>(engine.SemanticTextBytesPending());
			engine.SemanticsCopy(nodes, text);

			auto it = std::find_if(nodes.begin(), nodes.end(), [id](SemanticNode const& n) { return n.id == id; });
			return it != nodes.end() ? std::optional<Rect>{ it->bounds } : std::nullopt;
		}

		// The DIP position win32_input's translator derives from a client pixel, obtained by
		// actually running the translator over a WM_MOUSEMOVE. Going through the real translation
		// path is the point: it is what makes a layout/hit-test agreement claim meaningful rather
		// than a restatement of the formula under test.
		Vec2 TranslatedPointerDip(ViewportState const& viewport, Vec2 client_px)
		{
			auto state = Win32InputTranslatorState{};
			auto const lparam = MAKELPARAM(static_cast<std::int16_t>(std::lround(client_px.x)), static_cast<std::int16_t>(std::lround(client_px.y)));
			auto const msg = TranslateWindowMessage(nullptr, WM_MOUSEMOVE, 0, lparam, viewport, 0.0, false, state);
			return msg.inputs.empty() ? Vec2{ -1.0f, -1.0f } : Vec2{ msg.inputs[0].input.pointer_x, msg.inputs[0].input.pointer_y };
		}
	}

	PRUnitTest(AScreenRootFillsTheViewportNotTheClientWindow, Quick)
	{
		// A host rendering at twice its window resolution: the target, and therefore the viewport
		// given to View3DUI, is 512x256 while the window is 256x128. Sizing from the client would
		// lay the UI out at half the area it is actually drawn into.
		auto engine = UiEngine(MakeConfig());
		ApplyControls(engine, { MakeControl(1, 0, EControlType::Root, ELayoutMode::Overlay, Lp(0.0f, 0.0f)) });
		engine.Update(SubViewport(256, 128, 512, 256, 0.0f, 0.0f, 512.0f, 256.0f));

		auto rect = LaidOutBounds(engine, 1);
		PR_EXPECT(rect.has_value());
		PR_EXPECT(Near(rect->w, 512.0f));
		PR_EXPECT(Near(rect->h, 256.0f));
	}

	PRUnitTest(AScreenRootStillFillsTheViewportWhenTheWindowIsLargerThanTheTarget, Quick)
	{
		// The opposite ratio - a 512x256 window rendered at 256x128 - proves the extent tracks the
		// viewport rather than whichever of the two happens to be larger.
		auto engine = UiEngine(MakeConfig());
		ApplyControls(engine, { MakeControl(1, 0, EControlType::Root, ELayoutMode::Overlay, Lp(0.0f, 0.0f)) });
		engine.Update(SubViewport(512, 256, 256, 128, 0.0f, 0.0f, 256.0f, 128.0f));

		auto rect = LaidOutBounds(engine, 1);
		PR_EXPECT(rect.has_value());
		PR_EXPECT(Near(rect->w, 256.0f));
		PR_EXPECT(Near(rect->h, 128.0f));
	}

	PRUnitTest(AScreenRootIsSizedByTheSubViewportAndNotOffsetByItsOrigin, Quick)
	{
		// Given only a 300x200 sub-rectangle at (100, 50) of a 512x256 target, the root occupies
		// that sub-rectangle's extent and starts at the DIP origin: the origin belongs to the
		// render target, and the renderer's viewport transform is the single place it is applied.
		auto engine = UiEngine(MakeConfig());
		ApplyControls(engine, { MakeControl(1, 0, EControlType::Root, ELayoutMode::Overlay, Lp(0.0f, 0.0f)) });
		engine.Update(SubViewport(512, 256, 512, 256, 100.0f, 50.0f, 300.0f, 200.0f));

		auto rect = LaidOutBounds(engine, 1);
		PR_EXPECT(rect.has_value());
		PR_EXPECT(Near(rect->x, 0.0f));
		PR_EXPECT(Near(rect->y, 0.0f));
		PR_EXPECT(Near(rect->w, 300.0f));
		PR_EXPECT(Near(rect->h, 200.0f));
	}

	PRUnitTest(AScreenRootAtHighDpiIsSizedInViewportDips, Quick)
	{
		// DPI and the sub-viewport compose: 300x200 physical pixels at 192dpi is 150x100 DIPs.
		auto engine = UiEngine(MakeConfig());
		ApplyControls(engine, { MakeControl(1, 0, EControlType::Root, ELayoutMode::Overlay, Lp(0.0f, 0.0f)) });
		engine.Update(SubViewport(512, 256, 512, 256, 100.0f, 50.0f, 300.0f, 200.0f, 192.0f));

		auto rect = LaidOutBounds(engine, 1);
		PR_EXPECT(rect.has_value());
		PR_EXPECT(Near(rect->w, 150.0f));
		PR_EXPECT(Near(rect->h, 100.0f));
	}

	PRUnitTest(AWorldAnchorOnTheViewAxisProjectsToTheViewportCentreNotTheTargetCentre, Quick)
	{
		// The anchor is dead ahead, so it must land at the centre of the viewport in viewport-
		// relative DIPs. Adding the viewport origin here would report a position measured from the
		// render target's corner, which no other stage of the pipeline uses.
		auto const camera = PerspectiveCamera();
		auto const viewport = SubViewport(512, 256, 512, 256, 100.0f, 50.0f, 300.0f, 200.0f, 96.0f, camera);
		auto placement = Project(ERootPolicy::Overlay, WorldParams(Vec3{ 0, 0, 10 }), 40.0f, 20.0f, viewport);

		PR_EXPECT(placement.visible == 1);
		PR_EXPECT(Near(placement.rect.x + placement.rect.w * 0.5f, 150.0f));
		PR_EXPECT(Near(placement.rect.y + placement.rect.h * 0.5f, 100.0f));
	}

	PRUnitTest(AWorldAnchorProjectionIsUnaffectedByTheViewportOrigin, Quick)
	{
		// Two viewports differing only in where they sit within the render target must produce
		// identical placements, because the origin is not part of this coordinate space.
		auto const camera = PerspectiveCamera();
		auto const world = WorldParams(Vec3{ 3, -2, 12 });
		auto at_origin = Project(ERootPolicy::Overlay, world, 40.0f, 20.0f, SubViewport(512, 256, 512, 256, 0.0f, 0.0f, 300.0f, 200.0f, 96.0f, camera));
		auto offset = Project(ERootPolicy::Overlay, world, 40.0f, 20.0f, SubViewport(512, 256, 512, 256, 180.0f, 40.0f, 300.0f, 200.0f, 96.0f, camera));

		PR_EXPECT(offset.visible == at_origin.visible);
		PR_EXPECT(Near(offset.rect.x, at_origin.rect.x));
		PR_EXPECT(Near(offset.rect.y, at_origin.rect.y));
		PR_EXPECT(Near(offset.rect.w, at_origin.rect.w));
		PR_EXPECT(Near(offset.rect.h, at_origin.rect.h));
	}

	PRUnitTest(AWorldRootIsCulledAgainstTheViewportBoxNotAnOriginOffsetBox, Quick)
	{
		// A root just beyond the viewport's left edge is offscreen however far into the target the
		// viewport sits. Culling against an origin-offset box would keep it alive here and cull
		// perfectly visible roots near the viewport's own origin.
		auto const camera = PerspectiveCamera();
		auto const viewport = SubViewport(512, 256, 512, 256, 200.0f, 60.0f, 300.0f, 200.0f, 96.0f, camera);

		// Screen-right is world -x for a camera looking along +z with +y up, so a large +x anchor
		// is off the left-hand edge.
		auto off_left = Project(ERootPolicy::Overlay, WorldParams(Vec3{ 40, 0, 10 }), 20.0f, 10.0f, viewport);
		PR_EXPECT(off_left.visible == 0);

		auto centred = Project(ERootPolicy::Overlay, WorldParams(Vec3{ 0, 0, 10 }), 20.0f, 10.0f, viewport);
		PR_EXPECT(centred.visible == 1);
	}

	PRUnitTest(AWorldAnchorUsesTheViewportAspectUnderASubViewport, Quick)
	{
		// The vertical field of view is fixed and the horizontal one is derived from the viewport's
		// aspect, so widening the viewport reveals more world rather than stretching it: an anchor
		// keeps the same pixel offset from the viewport centre. Deriving the aspect from the render
		// target instead would make that offset move with the target's shape.
		auto const camera = PerspectiveCamera();
		auto const square = Project(ERootPolicy::Overlay, WorldParams(Vec3{ 1, 1, 10 }), 0.0f, 0.0f, SubViewport(512, 512, 512, 512, 0.0f, 0.0f, 200.0f, 200.0f, 96.0f, camera));
		auto const wide = Project(ERootPolicy::Overlay, WorldParams(Vec3{ 1, 1, 10 }), 0.0f, 0.0f, SubViewport(512, 512, 512, 512, 0.0f, 0.0f, 400.0f, 200.0f, 96.0f, camera));

		PR_EXPECT(square.visible == 1 && wide.visible == 1);
		PR_EXPECT(Near(wide.rect.y, square.rect.y));
		PR_EXPECT(Near(wide.rect.x - 200.0f, square.rect.x - 100.0f));

		// And the offset is genuinely non-zero, so the equality above is a real invariant rather
		// than two anchors that both happen to sit at the centre.
		PR_EXPECT(!Near(square.rect.x, 100.0f));
	}

	PRUnitTest(AScreenRootIsHitByThePointerPixelItAppearsAt, Quick)
	{
		// The end-to-end contract: a control laid out at a DIP rect is hit by the client pixel that
		// rect maps back to, under a viewport that exercises target != client, a non-zero origin
		// and a sub-viewport extent all at once.
		auto const viewport = SubViewport(256, 128, 512, 256, 100.0f, 50.0f, 300.0f, 200.0f);

		auto engine = UiEngine(MakeConfig());
		auto b = TxnBuilder{};
		b.Upsert(MakeControl(1, 0, EControlType::Root, ELayoutMode::Overlay, Lp(0.0f, 0.0f)));
		b.Upsert(MakeControl(2, 1, EControlType::Button, ELayoutMode::Overlay, Lp(80.0f, 40.0f)));
		engine.TransactionApply(b.Build(0, 1));
		engine.Update(viewport);

		auto rect = LaidOutBounds(engine, 1);
		PR_EXPECT(rect.has_value());

		// The viewport's centre in DIPs, pushed out to a client pixel and translated back through
		// the real Win32 path, must return the same DIP position.
		auto const centre = Vec2{ rect->x + rect->w * 0.5f, rect->y + rect->h * 0.5f };
		auto const dip = TranslatedPointerDip(viewport, DipToClientPixels(viewport, centre));
		PR_EXPECT(Near(dip.x, centre.x, 0.5f));
		PR_EXPECT(Near(dip.y, centre.y, 0.5f));

		// And that position is inside the laid-out rect, which is the property that would break if
		// layout and input disagreed about which extent they measure.
		PR_EXPECT(dip.x >= rect->x && dip.x <= rect->x + rect->w);
		PR_EXPECT(dip.y >= rect->y && dip.y <= rect->y + rect->h);
	}

	PRUnitTest(AWorldRootIsHitByThePointerPixelItAppearsAt, Quick)
	{
		// The same round trip for a projected world root, which is the case that would silently
		// break if world projection kept measuring from the render target's corner.
		auto const camera = PerspectiveCamera();
		auto const viewport = SubViewport(256, 128, 512, 256, 100.0f, 50.0f, 300.0f, 200.0f, 96.0f, camera);

		auto engine = UiEngine(MakeConfig());
		ApplyControls(engine, { MakeWorldRoot(1, ERootPolicy::Overlay, 80.0f, 40.0f, WorldParams(Vec3{ 0, 0, 10 })) });
		engine.Update(viewport);

		auto rect = LaidOutBounds(engine, 1);
		PR_EXPECT(rect.has_value());

		auto const centre = Vec2{ rect->x + rect->w * 0.5f, rect->y + rect->h * 0.5f };
		auto const dip = TranslatedPointerDip(viewport, DipToClientPixels(viewport, centre));
		PR_EXPECT(dip.x >= rect->x && dip.x <= rect->x + rect->w);
		PR_EXPECT(dip.y >= rect->y && dip.y <= rect->y + rect->h);
	}

	PRUnitTest(AFullWindowViewportBehavesExactlyAsBefore, Quick)
	{
		// The baseline every pre-existing test relies on: when the client, the target and the
		// viewport all coincide at the origin, none of the coordinate-space rules are observable.
		auto const camera = PerspectiveCamera();

		auto engine = UiEngine(MakeConfig());
		auto b = TxnBuilder{};
		b.Upsert(MakeControl(1, 0, EControlType::Root, ELayoutMode::Overlay, Lp(0.0f, 0.0f)));
		b.Upsert(MakeWorldRoot(2, ERootPolicy::Overlay, 80.0f, 40.0f, WorldParams(Vec3{ 0, 0, 10 })));
		engine.TransactionApply(b.Build(0, 1));

		engine.Update(Viewport(640, 480, 96.0f, 0.0, camera));
		auto screen = LaidOutBounds(engine, 1);
		auto world = LaidOutBounds(engine, 2);

		PR_EXPECT(screen.has_value() && Near(screen->x, 0.0f) && Near(screen->y, 0.0f));
		PR_EXPECT(Near(screen->w, 640.0f) && Near(screen->h, 480.0f));
		PR_EXPECT(world.has_value());
		PR_EXPECT(Near(world->x + world->w * 0.5f, 320.0f));
		PR_EXPECT(Near(world->y + world->h * 0.5f, 240.0f));
	}

	PRUnitTest(TheShaderNdcDivisorIsTheViewportExtentNotTheTargetExtent, Quick)
	{
		// The renderer's rects are viewport-local and RSSetViewports applies the viewport scale, so
		// dividing by the target extent as well would apply that scale twice.
		auto pass = Pass{};
		pass.m_width = 512;
		pass.m_height = 256;
		pass.m_viewport = D3D12_VIEWPORT{ 100.0f, 50.0f, 300.0f, 200.0f, 0.0f, 1.0f };

		auto divisor = Renderer::NdcDivisorPx(pass);
		PR_EXPECT(Near(divisor.x, 300.0f));
		PR_EXPECT(Near(divisor.y, 200.0f));
	}

	PRUnitTest(TheShaderNdcDivisorFallsBackToTheTargetForADegenerateViewport, Quick)
	{
		// A host that leaves the viewport unset must still produce a finite, non-zero divisor
		// rather than a division by zero that would make every vertex position NaN.
		auto pass = Pass{};
		pass.m_width = 512;
		pass.m_height = 256;
		pass.m_viewport = D3D12_VIEWPORT{};

		auto divisor = Renderer::NdcDivisorPx(pass);
		PR_EXPECT(Near(divisor.x, 512.0f));
		PR_EXPECT(Near(divisor.y, 256.0f));

		// A negative extent is equally unusable and takes the same path.
		pass.m_viewport = D3D12_VIEWPORT{ 0.0f, 0.0f, -4.0f, -4.0f, 0.0f, 1.0f };
		divisor = Renderer::NdcDivisorPx(pass);
		PR_EXPECT(Near(divisor.x, 512.0f));
		PR_EXPECT(Near(divisor.y, 256.0f));
	}

	PRUnitTest(AFullWindowPassLeavesTheNdcDivisorAtTheTargetExtent, Quick)
	{
		// The overwhelmingly common case, and the one every pre-M8 expectation encodes: a viewport
		// covering the whole target divides by exactly what it divided by before.
		auto pass = Pass{};
		pass.m_width = 800;
		pass.m_height = 600;
		pass.m_viewport = D3D12_VIEWPORT{ 0.0f, 0.0f, 800.0f, 600.0f, 0.0f, 1.0f };

		auto divisor = Renderer::NdcDivisorPx(pass);
		PR_EXPECT(Near(divisor.x, 800.0f));
		PR_EXPECT(Near(divisor.y, 600.0f));
	}

	#pragma endregion

	#pragma region Resolved depth lifecycle

	PRUnitTest(TheResolvedDepthViewIsRebuiltWhenTheHostRecreatesTheResourceAtANewSize, Quick)
	{
		auto fixture = TryBuildWorldFixture();
		if (!fixture.has_value())
			return;

		auto const camera = PerspectiveCamera();
		auto engine = UiEngine(MakeConfig());
		ApplyControls(engine, { MakeWorldRoot(1, ERootPolicy::OcclusionFaded, 120.0f, 60.0f, WorldParams(Vec3{ 0, 0, 10 })) });
		engine.Update(Viewport(256, 128, 96.0f, 0.0, camera));

		auto renderer = Renderer(fixture->device.Get(), MakeConfig());
		PR_EXPECT(renderer.Record(MakeWorldPass(*fixture, EPass::Prepare, camera), engine.DrawPackets()) == EStatus::Success);
		PR_EXPECT(renderer.ResolvedDepthViewEpoch() == 0);

		// First faded pass: the view has to be created.
		auto faded = MakeWorldPass(*fixture, EPass::OcclusionFaded, camera);
		PR_EXPECT(renderer.Record(faded, engine.DrawPackets()) == EStatus::Success);
		auto const created = renderer.ResolvedDepthViewEpoch();
		PR_EXPECT(created == 1);

		// An identical pass reuses it; rebuilding every frame would be a per-frame device call on
		// the hot path.
		PR_EXPECT(renderer.Record(MakeWorldPass(*fixture, EPass::OcclusionFaded, camera), engine.DrawPackets()) == EStatus::Success);
		PR_EXPECT(renderer.ResolvedDepthViewEpoch() == created);

		// The hazard this guards: the host released and recreated its resolved depth at a new size
		// and the allocator handed back the same address, so the cached pointer and format both
		// still match while the descriptor describes a resource that no longer exists. Only the
		// extents distinguish the two, so they must be part of the cache key.
		engine.Update(Viewport(512, 256, 96.0f, 0.0, camera));
		PR_EXPECT(renderer.Record(MakeWorldPass(*fixture, EPass::Prepare, camera), engine.DrawPackets()) == EStatus::Success);

		auto resized = MakeWorldPass(*fixture, EPass::OcclusionFaded, camera);
		resized.m_width = 512;
		resized.m_height = 256;
		resized.m_viewport = D3D12_VIEWPORT{ 0.0f, 0.0f, 512.0f, 256.0f, 0.0f, 1.0f };
		resized.m_scissor = D3D12_RECT{ 0, 0, 512, 256 };
		resized.m_resolved_depth.m_width = 512;
		resized.m_resolved_depth.m_height = 256;
		PR_EXPECT(resized.m_resolved_depth.m_resource == faded.m_resolved_depth.m_resource);
		PR_EXPECT(resized.m_resolved_depth.m_srv_format == faded.m_resolved_depth.m_srv_format);

		PR_EXPECT(renderer.Record(resized, engine.DrawPackets()) == EStatus::Success);
		PR_EXPECT(renderer.ResolvedDepthViewEpoch() == created + 1);

		PR_EXPECT(SUCCEEDED(fixture->command_list->Close()));
	}

	PRUnitTest(TheResolvedDepthViewIsRebuiltWhenTheHostChangesTheDepthFormat, Quick)
	{
		auto fixture = TryBuildWorldFixture();
		if (!fixture.has_value())
			return;

		auto const camera = PerspectiveCamera();
		auto engine = UiEngine(MakeConfig());
		ApplyControls(engine, { MakeWorldRoot(1, ERootPolicy::OcclusionFaded, 120.0f, 60.0f, WorldParams(Vec3{ 0, 0, 10 })) });
		engine.Update(Viewport(256, 128, 96.0f, 0.0, camera));

		auto renderer = Renderer(fixture->device.Get(), MakeConfig());
		PR_EXPECT(renderer.Record(MakeWorldPass(*fixture, EPass::Prepare, camera), engine.DrawPackets()) == EStatus::Success);
		PR_EXPECT(renderer.Record(MakeWorldPass(*fixture, EPass::OcclusionFaded, camera), engine.DrawPackets()) == EStatus::Success);
		PR_EXPECT(renderer.ResolvedDepthViewEpoch() == 1);

		// A depth-format change is the other way the host's resource identity moves underneath a
		// cached descriptor, and it must invalidate the view for the same reason.
		auto reformatted = MakeWorldPass(*fixture, EPass::OcclusionFaded, camera);
		reformatted.m_resolved_depth.m_srv_format = DXGI_FORMAT_R16_UNORM;
		PR_EXPECT(renderer.Record(reformatted, engine.DrawPackets()) == EStatus::Success);
		PR_EXPECT(renderer.ResolvedDepthViewEpoch() == 2);

		PR_EXPECT(SUCCEEDED(fixture->command_list->Close()));
	}

	PRUnitTest(AResolvedDepthSmallerThanTheTargetIsRefusedRatherThanSampledAtTheWrongTexels, Quick)
	{
		auto fixture = TryBuildWorldFixture();
		if (!fixture.has_value())
			return;

		auto const camera = PerspectiveCamera();
		auto engine = UiEngine(MakeConfig());
		ApplyControls(engine, { MakeWorldRoot(1, ERootPolicy::OcclusionFaded, 120.0f, 60.0f, WorldParams(Vec3{ 0, 0, 10 })) });
		engine.Update(Viewport(256, 128, 96.0f, 0.0, camera));

		auto renderer = Renderer(fixture->device.Get(), MakeConfig());
		PR_EXPECT(renderer.Record(MakeWorldPass(*fixture, EPass::Prepare, camera), engine.DrawPackets()) == EStatus::Success);

		// The fade shader addresses the copy by SV_POSITION, which is in render-target pixels, so a
		// copy that does not span the target would silently read the wrong texels - and out of
		// bounds reads return zero, the nearest possible depth, fading the whole UI away. Refusing
		// to draw is the only honest response.
		auto mismatched = MakeWorldPass(*fixture, EPass::OcclusionFaded, camera);
		mismatched.m_resolved_depth.m_width = 128;
		mismatched.m_resolved_depth.m_height = 64;
		PR_EXPECT(renderer.Record(mismatched, engine.DrawPackets()) == EStatus::Success);
		PR_EXPECT(renderer.ResolvedDepthViewEpoch() == 0);

		// A correctly sized copy on the very next pass still works, so the refusal is per-pass and
		// not a latch.
		PR_EXPECT(renderer.Record(MakeWorldPass(*fixture, EPass::OcclusionFaded, camera), engine.DrawPackets()) == EStatus::Success);
		PR_EXPECT(renderer.ResolvedDepthViewEpoch() == 1);

		PR_EXPECT(SUCCEEDED(fixture->command_list->Close()));
	}

	PRUnitTest(RepeatedResizesDoNotLeakOrStaleTheResolvedDepthView, Quick)
	{
		auto fixture = TryBuildWorldFixture();
		if (!fixture.has_value())
			return;

		auto const camera = PerspectiveCamera();
		auto engine = UiEngine(MakeConfig());
		ApplyControls(engine, { MakeWorldRoot(1, ERootPolicy::OcclusionFaded, 120.0f, 60.0f, WorldParams(Vec3{ 0, 0, 10 })) });

		auto renderer = Renderer(fixture->device.Get(), MakeConfig());
		auto expected_epoch = std::uint64_t{ 0 };

		// Alternate between two sizes so every pass is a cache miss, then settle: the epoch must
		// advance exactly once per genuine change and never once the size stops moving.
		for (auto i = 0; i != 6; ++i)
		{
			auto const w = (i % 2) == 0 ? std::uint32_t{ 256 } : std::uint32_t{ 512 };
			auto const h = (i % 2) == 0 ? std::uint32_t{ 128 } : std::uint32_t{ 256 };

			engine.Update(Viewport(w, h, 96.0f, static_cast<double>(i), camera));
			PR_EXPECT(renderer.Record(MakeWorldPass(*fixture, EPass::Prepare, camera), engine.DrawPackets()) == EStatus::Success);

			auto pass = MakeWorldPass(*fixture, EPass::OcclusionFaded, camera);
			pass.m_width = w;
			pass.m_height = h;
			pass.m_viewport = D3D12_VIEWPORT{ 0.0f, 0.0f, static_cast<float>(w), static_cast<float>(h), 0.0f, 1.0f };
			pass.m_scissor = D3D12_RECT{ 0, 0, static_cast<LONG>(w), static_cast<LONG>(h) };
			pass.m_resolved_depth.m_width = w;
			pass.m_resolved_depth.m_height = h;
			PR_EXPECT(renderer.Record(pass, engine.DrawPackets()) == EStatus::Success);

			++expected_epoch;
			PR_EXPECT(renderer.ResolvedDepthViewEpoch() == expected_epoch);
		}

		PR_EXPECT(SUCCEEDED(fixture->command_list->Close()));
	}

	#pragma endregion
}
