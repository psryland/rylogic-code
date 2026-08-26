//*********************************************
// View3DUI Tests
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// Shared test-only helpers: a minimal fake external ID3D12Device stand-in, header/record builders,
// and a TxnBuilder alias for the public pr::view3d::ui::TransactionBuilder. Only the fake device and
// the record builders are test-only; TxnBuilder itself is the product's own facade type. Owned
// entirely by projects/tests/view3d-ui-tests.
#pragma once
#include "pr/view3d-ui/view3d-ui.h"
#include "pr/view3d-ui/transaction_builder.h"
#include <initializer_list>
#include <string_view>
#include <utility>
#include <vector>

namespace pr::view3d::ui::tests
{
	// Minimal IUnknown implementation standing in for an external ID3D12Device. View3DUI_ContextCreate
	// only requires a non-null IUnknown* that it can AddRef/Release; no D3D12 interface is queried by
	// the M0-M2 milestones under test here.
	class FakeDevice : public IUnknown
	{
		std::atomic<ULONG> m_refs;

	public:
		FakeDevice()
			: m_refs(1)
		{}
		HRESULT __stdcall QueryInterface(REFIID, void** object) override
		{
			*object = nullptr;
			return E_NOINTERFACE;
		}
		ULONG __stdcall AddRef() override
		{
			return ++m_refs;
		}
		ULONG __stdcall Release() override
		{
			auto remaining = --m_refs;
			if (remaining == 0)
				delete this;

			return remaining;
		}
		ULONG RefCount() const
		{
			return m_refs.load();
		}
	};

	// Populate a StructHeader for T at the current struct version, as every real caller must.
	template <typename T>
	StructHeader Header()
	{
		return StructHeader{ .size = sizeof(T), .version = VIEW3D_UI_STRUCT_VERSION };
	}

	// A Config carrying the same defaults as the native DefaultConfig() (not itself reachable from
	// this eager-facade-only test project), with every bound individually overridable so a test
	// can trigger a specific ResourceLimit deterministically without an impractically large tree.
	inline Config MakeConfig(
		std::uint32_t max_controls = 4096,
		std::uint32_t max_roots = 64,
		std::uint32_t max_tree_depth = 64,
		std::uint32_t max_operations_per_transaction = 4096,
		std::uint32_t max_blob_bytes = 1u << 20,
		std::uint32_t max_templates = 256,
		std::uint32_t max_styles = 256,
		std::uint32_t max_resources = 256,
		std::uint32_t max_queued_events = 1024,
		std::uint32_t max_semantic_records = 4096,
		float max_transition_duration_ms = 2000.0f,
		std::uint32_t max_glyph_cache_bytes = 1u << 24,
		std::uint32_t max_glyph_cache_pages = 16)
	{
		return Config{
			.header = Header<Config>(),
			.max_controls = max_controls,
			.max_roots = max_roots,
			.max_tree_depth = max_tree_depth,
			.max_operations_per_transaction = max_operations_per_transaction,
			.max_blob_bytes = max_blob_bytes,
			.max_templates = max_templates,
			.max_styles = max_styles,
			.max_resources = max_resources,
			.max_glyph_cache_bytes = max_glyph_cache_bytes,
			.max_glyph_cache_pages = max_glyph_cache_pages,
			.max_generated_vertices = 1u << 16,
			.max_generated_indices = 1u << 18,
			.max_queued_events = max_queued_events,
			.max_semantic_records = max_semantic_records,
			.max_transition_duration_ms = max_transition_duration_ms,
		};
	}

	// A fixed-size, non-stretch, zero-margin/padding LayoutParams; tests that need margins/padding
	// or stretch alignment construct LayoutParams directly with designated initializers instead.
	inline LayoutParams Lp(float width, float height, EHAlign h = EHAlign::Left, EVAlign v = EVAlign::Top, float stack_spacing = 0.0f)
	{
		return LayoutParams{
			.width = width,
			.height = height,
			.margin_left = 0.0f,
			.margin_top = 0.0f,
			.margin_right = 0.0f,
			.margin_bottom = 0.0f,
			.padding_left = 0.0f,
			.padding_top = 0.0f,
			.padding_right = 0.0f,
			.padding_bottom = 0.0f,
			.h_align = h,
			.v_align = v,
			.stack_spacing = stack_spacing,
		};
	}

	// A minimally-populated ControlDesc with every non-layout field defaulted; callers customise
	// the returned value (a local, not a container element) before handing it to TxnBuilder::Upsert.
	// 'font_resource_id' defaults to 0 (the built-in Segoe UI fallback); pass a registered Font
	// resource id to exercise font resolution.
	inline ControlDesc MakeControl(ControlId id, ControlId parent_id, EControlType type, ELayoutMode layout_mode, LayoutParams const& layout, ResourceId font_resource_id = 0)
	{
		return ControlDesc{
			.header = Header<ControlDesc>(),
			.id = id,
			.parent_id = parent_id,
			.type = type,
			.root_policy = ERootPolicy::Screen,
			.layout_mode = layout_mode,
			.template_id = 0,
			.style_id = 0,
			.enabled = 1,
			.visible = 1,
			.focusable = (type == EControlType::Button || type == EControlType::TextBox) ? 1 : 0,
			.validation_state = EValidationState::NotApplicable,
			.layout = layout,
			.text_offset = 0,
			.text_length = 0,
			.name_offset = 0,
			.name_length = 0,
			.desc_offset = 0,
			.desc_length = 0,
			.max_text_length = 256,
			.font_resource_id = font_resource_id,
		};
	}

	// A neutral StyleDesc: every state channel shares the same opaque white fill/black border, no
	// transitions. Callers customise individual channels afterwards when a test needs contrast.
	inline StyleDesc MakeStyle(StyleId id)
	{
		auto desc = StyleDesc{ .header = Header<StyleDesc>(), .id = id, .visuals = {}, .transitions = {} };
		for (auto i = std::size_t{}; i != static_cast<std::size_t>(EStateChannel::Count); ++i)
		{
			desc.visuals[i] = StyleVisual{ .fill = Colour{1, 1, 1, 1}, .border_colour = Colour{0, 0, 0, 1}, .border_thickness = 0.0f, .corner_radius = 0.0f, .opacity = 1.0f };
			desc.transitions[i] = TransitionDesc{ .duration_ms = 0.0f, .easing = EEasing::Linear };
		}
		return desc;
	}

	// A minimally-populated ResourceDesc (unnamed; callers append a name via TxnBuilder::AddText).
	inline ResourceDesc MakeResource(ResourceId id, EResourceKind kind, Colour colour, float font_size = 0.0f)
	{
		return ResourceDesc{ .header = Header<ResourceDesc>(), .id = id, .kind = kind, .colour = colour, .name_offset = 0, .name_length = 0, .font_size = font_size };
	}

	// A TemplateDesc with zero parts; callers append parts via TxnBuilder::AddTemplatePart before
	// handing the completed value to TxnBuilder::AddTemplate.
	inline TemplateDesc MakeTemplate(TemplateId id, EControlType applies_to)
	{
		return TemplateDesc{ .header = Header<TemplateDesc>(), .id = id, .applies_to = applies_to, .part_count = 0, .parts = {} };
	}

	// A ViewportState with the viewport filling the whole client area and no host offset. The
	// camera is absent by default, which is exactly the state every screen-only test relies on.
	inline ViewportState Viewport(std::uint32_t width_px, std::uint32_t height_px, float dpi = 96.0f, double time_ms = 0.0, CameraState const& camera = CameraState{})
	{
		return ViewportState{
			.header = Header<ViewportState>(),
			.client_width_px = width_px,
			.client_height_px = height_px,
			.target_width_px = width_px,
			.target_height_px = height_px,
			.viewport_x_px = 0.0f,
			.viewport_y_px = 0.0f,
			.viewport_width_px = static_cast<float>(width_px),
			.viewport_height_px = static_cast<float>(height_px),
			.dpi = dpi,
			.time_ms = time_ms,
			.camera = camera,
		};
	}

	// A ViewportState where the client window, the render target and the viewport within that
	// target are all independent. This is the general case every coordinate-space conversion has
	// to survive: the host may render at a different resolution to its window (target != client)
	// and may give View3DUI only a sub-rectangle of that target.
	inline ViewportState SubViewport(std::uint32_t client_w_px, std::uint32_t client_h_px, std::uint32_t target_w_px, std::uint32_t target_h_px, float viewport_x_px, float viewport_y_px, float viewport_w_px, float viewport_h_px, float dpi = 96.0f, CameraState const& camera = CameraState{})
	{
		return ViewportState{
			.header = Header<ViewportState>(),
			.client_width_px = client_w_px,
			.client_height_px = client_h_px,
			.target_width_px = target_w_px,
			.target_height_px = target_h_px,
			.viewport_x_px = viewport_x_px,
			.viewport_y_px = viewport_y_px,
			.viewport_width_px = viewport_w_px,
			.viewport_height_px = viewport_h_px,
			.dpi = dpi,
			.time_ms = 0.0,
			.camera = camera,
		};
	}

	// The host client pixel that addresses 'dip' under 'viewport', which is the inverse of the
	// mapping win32_input.cpp applies to pointer messages. Tests use it to prove that a rect
	// produced by layout or world projection is hit by the pointer position it appears at.
	inline Vec2 DipToClientPixels(ViewportState const& viewport, Vec2 dip)
	{
		auto px_per_dip = viewport.dpi / 96.0f;
		auto client_per_target_x = viewport.client_width_px / static_cast<float>(viewport.target_width_px);
		auto client_per_target_y = viewport.client_height_px / static_cast<float>(viewport.target_height_px);
		return Vec2{
			(dip.x * px_per_dip + viewport.viewport_x_px) * client_per_target_x,
			(dip.y * px_per_dip + viewport.viewport_y_px) * client_per_target_y,
		};
	}

	// A valid perspective camera at 'position' looking along +z with world +y up. Tests that need
	// another orientation copy the result and overwrite 'forward'/'up'.
	inline CameraState PerspectiveCamera(Vec3 position = Vec3{0, 0, 0}, float fov_y_rad = 1.0471976f, float near_plane = 0.1f, float far_plane = 1000.0f)
	{
		return CameraState{
			.position = position,
			.forward = Vec3{0, 0, 1},
			.up = Vec3{0, 1, 0},
			.projection = EProjection::Perspective,
			.valid = 1,
			.fov_y_rad = fov_y_rad,
			.ortho_height = 0.0f,
			.near_plane = near_plane,
			.far_plane = far_plane,
		};
	}

	// A valid orthographic camera spanning 'ortho_height' world units vertically.
	inline CameraState OrthographicCamera(float ortho_height, Vec3 position = Vec3{0, 0, 0}, float near_plane = 0.1f, float far_plane = 1000.0f)
	{
		return CameraState{
			.position = position,
			.forward = Vec3{0, 0, 1},
			.up = Vec3{0, 1, 0},
			.projection = EProjection::Orthographic,
			.valid = 1,
			.fov_y_rad = 1.0471976f,
			.ortho_height = ortho_height,
			.near_plane = near_plane,
			.far_plane = far_plane,
		};
	}

	// Default world-root parameters: anchored by its centre, constant apparent size, and an
	// occlusion fade that is finite and bounded even for policies that never read it.
	inline WorldRootParams WorldParams(Vec3 anchor = Vec3{0, 0, 10})
	{
		return WorldRootParams{
			.anchor = anchor,
			.anchor_h = EAnchorPoint::Centre,
			.anchor_v = EAnchorPoint::Centre,
			.sizing = EWorldSizing::ConstantDip,
			.world_units_per_dip = 0.01f,
			.depth_offset = 0.0f,
			.occlusion_min_opacity = 0.25f,
			.occlusion_fade_depth = 1.0f,
			.occlusion_depth_bias = 0.0f,
		};
	}

	// A world-anchored root ControlDesc under 'policy', sized explicitly as every world root must be.
	inline ControlDesc MakeWorldRoot(ControlId id, ERootPolicy policy, float width, float height, WorldRootParams const& world)
	{
		auto desc = MakeControl(id, 0, EControlType::Root, ELayoutMode::Canvas, Lp(width, height));
		desc.root_policy = policy;
		desc.world = world;
		return desc;
	}

	// Normalized input record builders (section 7.2), one per EInputKind used by the M2 tests.
	inline NormalizedInput PointerMoveInput(float x, float y, double time_ms = 0.0)
	{
		return NormalizedInput{ .header = Header<NormalizedInput>(), .kind = EInputKind::PointerMove, .pointer_x = x, .pointer_y = y, .button = EPointerButton::None, .button_mask = 0, .wheel_delta = 0.0f, .vk = 0, .modifiers = 0, .char_code = 0, .time_ms = time_ms };
	}
	inline NormalizedInput PointerDownInput(float x, float y, EPointerButton button = EPointerButton::Left, std::uint32_t modifiers = 0, double time_ms = 0.0)
	{
		return NormalizedInput{ .header = Header<NormalizedInput>(), .kind = EInputKind::PointerButtonDown, .pointer_x = x, .pointer_y = y, .button = button, .button_mask = static_cast<std::uint32_t>(1U << (static_cast<int>(button) - 1)), .wheel_delta = 0.0f, .vk = 0, .modifiers = modifiers, .char_code = 0, .time_ms = time_ms };
	}
	inline NormalizedInput PointerUpInput(float x, float y, EPointerButton button = EPointerButton::Left, std::uint32_t modifiers = 0, double time_ms = 0.0)
	{
		return NormalizedInput{ .header = Header<NormalizedInput>(), .kind = EInputKind::PointerButtonUp, .pointer_x = x, .pointer_y = y, .button = button, .button_mask = 0, .wheel_delta = 0.0f, .vk = 0, .modifiers = modifiers, .char_code = 0, .time_ms = time_ms };
	}
	inline NormalizedInput KeyDownInput(std::int32_t vk, std::uint32_t modifiers = 0, double time_ms = 0.0)
	{
		return NormalizedInput{ .header = Header<NormalizedInput>(), .kind = EInputKind::KeyDown, .pointer_x = 0, .pointer_y = 0, .button = EPointerButton::None, .button_mask = 0, .wheel_delta = 0.0f, .vk = vk, .modifiers = modifiers, .char_code = 0, .time_ms = time_ms };
	}
	inline NormalizedInput KeyUpInput(std::int32_t vk, std::uint32_t modifiers = 0, double time_ms = 0.0)
	{
		return NormalizedInput{ .header = Header<NormalizedInput>(), .kind = EInputKind::KeyUp, .pointer_x = 0, .pointer_y = 0, .button = EPointerButton::None, .button_mask = 0, .wheel_delta = 0.0f, .vk = vk, .modifiers = modifiers, .char_code = 0, .time_ms = time_ms };
	}
	inline NormalizedInput CharInputRecord(std::uint32_t char_code, double time_ms = 0.0)
	{
		return NormalizedInput{ .header = Header<NormalizedInput>(), .kind = EInputKind::Char, .pointer_x = 0, .pointer_y = 0, .button = EPointerButton::None, .button_mask = 0, .wheel_delta = 0.0f, .vk = 0, .modifiers = 0, .char_code = char_code, .time_ms = time_ms };
	}

	// M9 text/composition record builders. These carry no text themselves; the text travels in the
	// companion InputTextPayload passed to UiContext::InputInjectText.
	inline NormalizedInput TextInputRecord(EInputKind kind, double time_ms = 0.0)
	{
		return NormalizedInput{ .header = Header<NormalizedInput>(), .kind = kind, .pointer_x = 0, .pointer_y = 0, .button = EPointerButton::None, .button_mask = 0, .wheel_delta = 0.0f, .vk = 0, .modifiers = 0, .char_code = 0, .time_ms = time_ms };
	}

	// Borrowed UTF-8 payload over 'text'. 'text' must outlive the injection call.
	inline InputTextPayload TextPayload(std::string_view text, std::uint32_t caret = 0, std::uint32_t selection_start = 0, std::uint32_t selection_end = 0)
	{
		return InputTextPayload{ .header = Header<InputTextPayload>(), .text_utf8 = text.data(), .text_length = static_cast<std::uint32_t>(text.size()), .caret = caret, .selection_start = selection_start, .selection_end = selection_end };
	}

	// Accumulates one delta transaction's borrowed arrays (operations/controls/child order/blob).
	// Test-project alias for the public pr::view3d::ui::TransactionBuilder (transaction_builder.h),
	// kept under this name/namespace so existing test call sites are unaffected.
	using TxnBuilder = pr::view3d::ui::TransactionBuilder;

}
