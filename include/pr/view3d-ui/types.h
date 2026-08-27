//*********************************************
// View3DUI
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// Dependency-minimal wire records and enums shared by the native C++ facade and the C ABI.
// Rules (see projects/rylogic/view3d-ui/implementation-plan.md section 5.1):
//  - fixed-width integer fields only; enums are stored as int32_t;
//  - no STL, exceptions, references, bool, C++ classes, or ownership-bearing COM pointers;
//  - every extensible record begins with a {size, version} StructHeader;
//  - arrays are passed as a pointer plus a 32-bit count and are copied before the call returns.
#pragma once
#include <cstdint>
#include <cstddef>
#include <type_traits>

namespace pr::view3d::ui
{
	inline constexpr std::uint32_t VIEW3D_UI_API_VERSION = 0x00030000U;
	inline constexpr std::uint32_t VIEW3D_UI_STRUCT_VERSION = 3U;

	// Maximum named parts recorded directly within one TemplateDesc. Bounded so the descriptor
	// stays fixed-layout; the closed template vocabulary (section 6.3) never needs more than this.
	inline constexpr std::uint32_t VIEW3D_UI_MAX_TEMPLATE_PARTS = 8U;

	// Opaque process-level module initialization token. Mirrors pr::audio::DllHandle; many
	// initialise/shutdown pairs may be registered by independent callers within one process.
	using RuntimeHandle = void const*;

	// Generation-aware handle to one owner-thread UI context attached to a View3D window.
	using ContextHandle = std::uint64_t;

	// Application-assigned, non-zero, process-unique identifiers. View3DUI never generates these;
	// the application is the single authority for control/resource/style/template identity.
	using ControlId = std::uint64_t;
	using ResourceId = std::uint64_t;
	using StyleId = std::uint64_t;
	using TemplateId = std::uint64_t;

	// Result of a View3DUI API operation. Every failure is a specific, stable category; no
	// operation silently substitutes a default or reports success for a rejected request.
	enum class EStatus : std::int32_t
	{
		Success = 0,
		InvalidArgument = 1,
		InvalidStruct = 2,
		AbiMismatch = 3,
		SchemaMismatch = 4,
		InvalidHandle = 5,
		StaleHandle = 6,
		WrongThread = 7,
		StaleRevision = 8,
		InvalidTree = 9,
		UnknownType = 10,
		UnknownResource = 11,
		UnsupportedFeature = 12,
		BufferTooSmall = 13,
		QueueOverflow = 14,
		ResourceLimit = 15,
		MissingAsset = 16,
		DeviceLost = 17,
		ResourceInUse = 18,
		InternalError = 19,
	};

	// Public structures discoverable through View3DUI_StructSize. Native and managed startup
	// tests enumerate this complete set rather than a hand-selected subset (section 5.2).
	enum class EStructId : std::int32_t
	{
		Config = 1,
		Transaction = 2,
		Operation = 3,
		Control = 4,
		Resource = 5,
		Style = 6,
		Template = 7,
		NormalizedInput = 8,
		ViewportState = 9,
		Event = 10,
		SemanticNode = 11,
		Diagnostics = 12,
		HostBridgeVersion = 13,
		HostPassContext = 14,
		InputTextPayload = 15,
	};

	// Closed control vocabulary for the first vertical slice (section 6.1). Applications cannot
	// register control classes; adding a control type is a schema change, not a runtime option.
	enum class EControlType : std::int32_t
	{
		Root = 0,
		Panel = 1,
		Text = 2,
		TextBox = 3,
		Button = 4,
		Count = 5,
	};

	// Closed layout vocabulary (section 6.2). Overlay and the two Stack orientations place a
	// child by alignment/margins; Scroll shifts its own content by LayoutParams::scroll_offset_x/y
	// before applying the same placement rule as Overlay; Canvas places each child at the absolute
	// LayoutParams::canvas_x/y offset from its parent's content origin, ignoring alignment/margins
	// entirely because the position is already explicit.
	enum class ELayoutMode : std::int32_t
	{
		Overlay = 0,
		StackHorizontal = 1,
		StackVertical = 2,
		Scroll = 3,
		Canvas = 4,
		Count = 5,
	};

	// Root render policy (section 4). 'Screen' places the root in the host's true final-overlay
	// phase in screen DIP space. The remaining three are world-anchored: the root's DIP box is
	// laid out in its own local space and then projected about WorldRootParams::anchor, and the
	// policy selects which View3D-owned host stage records it.
	//  - DepthTested: a scene-adjacent stage using View3D-owned scene depth state, so world
	//    geometry in front of the root hides it;
	//  - OcclusionFaded: the post-alpha world-overlay stage, blended against a View3D-owned
	//    single-sample resolved depth so occluded pixels fade rather than disappear;
	//  - Overlay: the same post-alpha world-overlay stage with no depth interaction at all.
	// All three record strictly before the screen-space final overlay.
	enum class ERootPolicy : std::int32_t
	{
		Screen = 0,
		DepthTested = 1,
		OcclusionFaded = 2,
		Overlay = 3,
		Count = 4,
	};

	// True when 'policy' anchors its root in world space rather than screen DIP space.
	constexpr bool IsWorldPolicy(ERootPolicy policy)
	{
		switch (policy)
		{
			case ERootPolicy::Screen: return false;
			case ERootPolicy::DepthTested: return true;
			case ERootPolicy::OcclusionFaded: return true;
			case ERootPolicy::Overlay: return true;
			default: return false;
		}
	}

	// How a world-anchored root converts its DIP layout box into apparent screen size.
	//  - ConstantDip: the box keeps a constant apparent DIP size at every distance, so a label
	//    stays legible no matter how far away its anchor is;
	//  - WorldUnits: one DIP of the box measures WorldRootParams::world_units_per_dip world
	//    units, so the box shrinks and grows with distance exactly like scene geometry.
	enum class EWorldSizing : std::int32_t
	{
		ConstantDip = 0,
		WorldUnits = 1,
		Count = 2,
	};

	// Which point of a world root's DIP box coincides with WorldRootParams::anchor, per axis.
	// Deliberately distinct from EHAlign/EVAlign, which include a Stretch value that has no
	// meaning for a single anchor point.
	enum class EAnchorPoint : std::int32_t
	{
		Min = 0,
		Centre = 1,
		Max = 2,
		Count = 3,
	};

	// Host camera projection kind (section 4.4). Stored explicitly rather than inferred from the
	// field-of-view value so an orthographic camera never depends on a sentinel.
	enum class EProjection : std::int32_t
	{
		Perspective = 0,
		Orthographic = 1,
		Count = 2,
	};

	// Horizontal alignment of a control within its parent's cross-axis or overlay slot.
	enum class EHAlign : std::int32_t
	{
		Left = 0,
		Center = 1,
		Right = 2,
		Stretch = 3,
		Count = 4,
	};

	// Vertical alignment of a control within its parent's cross-axis or overlay slot.
	enum class EVAlign : std::int32_t
	{
		Top = 0,
		Center = 1,
		Bottom = 2,
		Stretch = 3,
		Count = 4,
	};

	// Closed visual primitive vocabulary usable by a lookless template (section 6.3). Applications
	// compose these but cannot submit shader names, arbitrary geometry, or draw callbacks.
	enum class EVisualPrimitive : std::int32_t
	{
		SolidBox = 0,
		RoundedBox = 1,
		Border = 2,
		TextPresenter = 3,
		ContentPresenter = 4,
		Clip = 5,
		TransformOpacityGroup = 6,
		Count = 7,
	};

	// Closed style/transition state channel vocabulary (section 6.4). Selected is a durable
	// per-control flag (ControlDesc::selected); Visibility fires for one Update() call whenever a
	// control's own 'visible' field transitions off to on; ValueChanged fires for one Update() call
	// whenever ControlDesc::value_sequence changes from its previously observed value. Priority
	// when more than one channel could apply (highest first): Disabled, Pressed, Invalid, Focused,
	// then Visibility/ValueChanged (whichever is currently active) override Hover/Selected/Normal,
	// finally Hover, Selected, Normal (see style.cpp::StyleResolver::Resolve).
	enum class EStateChannel : std::int32_t
	{
		Normal = 0,
		Hover = 1,
		Pressed = 2,
		Focused = 3,
		Selected = 4,
		Disabled = 5,
		Invalid = 6,
		Visibility = 7,
		ValueChanged = 8,
		Count = 9,
	};

	// Closed transition easing vocabulary.
	enum class EEasing : std::int32_t
	{
		Linear = 0,
		EaseInOut = 1,
		Count = 2,
	};

	// Closed durable validation state for a control's proposed/accepted value (section 7.6). The
	// application, not View3DUI, computes this; View3DUI only reads it to select style/enablement.
	enum class EValidationState : std::int32_t
	{
		NotApplicable = 0,
		Pending = 1,
		Invalid = 2,
		Valid = 3,
		Count = 4,
	};

	// Closed transaction operation vocabulary (section 5.3).
	enum class EOperationKind : std::int32_t
	{
		Upsert = 0,
		Remove = 1,
		Reorder = 2,
		ReplaceSubtree = 3,
		Count = 4,
	};

	// Closed resource kind vocabulary. A Font resource's family name/size (ResourceDesc) is
	// resolved by the M3 renderer via ControlDesc::font_resource_id; a Colour resource's shape is
	// validated but it is not yet cross-referenced by id from any other structure.
	enum class EResourceKind : std::int32_t
	{
		Colour = 0,
		Font = 1,
		Count = 2,
	};

	// Closed normalized input kind vocabulary (section 7.2). Raw Win32 messages and deterministic
	// test injection both translate into this same vocabulary before reaching the state machine.
	enum class EInputKind : std::int32_t
	{
		PointerMove = 0,
		PointerButtonDown = 1,
		PointerButtonUp = 2,
		PointerWheel = 3,
		KeyDown = 4,
		KeyUp = 5,
		Char = 6,
		FocusGained = 7,
		FocusLost = 8,

		// Committed text of arbitrary length, carried by an InputTextPayload. Distinct from Char,
		// which carries exactly one scalar value inline: a paste, an injected string, or the
		// character a dead-key sequence produced can all be more than one code point.
		TextInput = 9,

		// IME composition lifecycle. A composition is transient native state that never reaches
		// the application until CompositionCommit converts it into a text-change proposal.
		// CompositionStart takes no payload, CompositionUpdate carries the composition string with
		// the IME's caret/clause offsets, CompositionCommit carries the result string, and
		// CompositionCancel takes no payload and restores the pre-composition edit exactly.
		CompositionStart = 10,
		CompositionUpdate = 11,
		CompositionCommit = 12,
		CompositionCancel = 13,

		Count = 14,
	};

	// Ordinal pointer button identifier used by PointerButtonDown/Up.
	enum class EPointerButton : std::int32_t
	{
		None = 0,
		Left = 1,
		Right = 2,
		Middle = 3,
		X1 = 4,
		X2 = 5,
		Count = 6,
	};

	// Bitmask of currently held pointer buttons, valid on PointerMove.
	enum class EPointerButtonMask : std::uint32_t
	{
		None = 0,
		Left = 1U << 0,
		Right = 1U << 1,
		Middle = 1U << 2,
		X1 = 1U << 3,
		X2 = 1U << 4,
	};

	// Bitmask of held modifier keys.
	enum class EInputModifier : std::uint32_t
	{
		None = 0,
		Shift = 1U << 0,
		Ctrl = 1U << 1,
		Alt = 1U << 2,
	};

	// Closed typed event vocabulary (section 5.4).
	enum class EEventKind : std::int32_t
	{
		FocusChanged = 0,
		TextChangeProposed = 1,
		CommandInvoked = 2,
		PointerCaptureChanged = 3,
		QueueOverflow = 4,
		Diagnostic = 5,
		Count = 6,
	};

	// Bitmask of UI Automation-style actions a semantic node currently supports (section 5.5).
	enum class ESemanticAction : std::uint32_t
	{
		None = 0,
		Invoke = 1U << 0,
		SetValue = 1U << 1,
		Focus = 1U << 2,

		// The node exposes a caret and a selection range that a text pattern could move. Reported
		// only for an editable control, which is the only kind whose text ranges this module owns.
		SetSelection = 1U << 3,
	};

	// Bitmask of boolean semantic state flags (section 5.5), packed to keep SemanticNode compact.
	enum class ESemanticState : std::uint32_t
	{
		None = 0,
		Enabled = 1U << 0,
		Visible = 1U << 1,
		Focused = 1U << 2,
		Focusable = 1U << 3,
		Selected = 1U << 4,
		Invalid = 1U << 5,
		Offscreen = 1U << 6,
	};

	// Bitmask describing which of a SemanticNode's text-range fields carry meaning. Reporting
	// presence explicitly keeps a legitimate zero offset (a caret at the start of the text)
	// distinguishable from "this node has no caret at all".
	enum class ESemanticTextFlag : std::uint32_t
	{
		None = 0,
		HasCaret = 1U << 0,      // 'caret' is a live caret position within the value text.
		HasSelection = 1U << 1,  // 'selection_start'/'selection_end' describe a non-empty selection.
		Composing = 1U << 2,     // 'composition_start'/'composition_length' describe live IME composition text.
	};

	// Version marker at the start of every extensible public structure. 'size' is the caller's
	// compiled sizeof(T); 'version' is VIEW3D_UI_STRUCT_VERSION at the time the caller was built.
	struct StructHeader
	{
		std::uint32_t size;
		std::uint32_t version;
	};

	// Dependency-minimal 2D point/size in DIPs.
	struct Vec2
	{
		float x;
		float y;
	};

	// Dependency-minimal 3D point/direction in world units.
	struct Vec3
	{
		float x;
		float y;
		float z;
	};

	// Dependency-minimal axis-aligned rectangle in DIPs: (x, y) is the top-left corner.
	struct Rect
	{
		float x;
		float y;
		float w;
		float h;
	};

	// Straight (non-premultiplied) linear-in-byte-order RGBA colour with components in [0, 1].
	struct Colour
	{
		float r;
		float g;
		float b;
		float a;
	};

	// Callback pairing a native function pointer with caller-owned context, matching the audio
	// module's Callback<T> convention so the ABI never carries a bare unmanaged closure.
	template <typename FuncType>
	struct Callback
	{
		using FuncCB = FuncType;
		using CtxPtr = union { void const* cp; void* p; };

		CtxPtr m_ctx = {};
		FuncCB m_cb = {};

		template <typename... Args>
		auto operator()(Args&&... args) const
		{
			return m_cb(m_ctx.p, static_cast<Args&&>(args)...);
		}
		explicit operator bool() const
		{
			return m_cb != nullptr;
		}
	};
	using ReportErrorCB = Callback<void(__stdcall*)(void* ctx, char const* msg, char const* filepath, int line)>;

	// Configurable context capacities and physical constants (section 9.2). Every bounded resource
	// in the implementation is sized from this record at context creation; growth is never silent
	// or unbounded.
	struct Config
	{
		StructHeader header;
		std::uint32_t max_controls;
		std::uint32_t max_roots;
		std::uint32_t max_tree_depth;
		std::uint32_t max_operations_per_transaction;
		std::uint32_t max_blob_bytes;
		std::uint32_t max_templates;
		std::uint32_t max_styles;
		std::uint32_t max_resources;
		std::uint32_t max_glyph_cache_bytes;    // Bounds the renderer's CPU-side glyph atlas; Acquire() past this returns ResourceLimit.
		std::uint32_t max_glyph_cache_pages;    // Bounds the renderer's glyph atlas page count; Acquire() past this returns ResourceLimit.
		std::uint32_t max_generated_vertices;   // Reserved: the M3 renderer draws with root constants and a fixed vertex-less quad, so this is currently unused.
		std::uint32_t max_generated_indices;    // Reserved: the M3 renderer draws with root constants and a fixed vertex-less quad, so this is currently unused.
		std::uint32_t max_queued_events;
		std::uint32_t max_semantic_records;
		float max_transition_duration_ms;
	};

	// Explicit per-control layout inputs, all in DIPs (section 6.2). Width/height are required and
	// must be finite and non-negative for every non-root control; layout never measures content,
	// which keeps M2 layout deterministic without a text-metrics dependency. A Root control is the
	// sole exception: width/height of exactly zero means "size to the viewport's current DIP
	// extent", because a root's size is naturally driven by the host viewport rather than an
	// application-authored constant.
	struct LayoutParams
	{
		float width;
		float height;
		float margin_left;
		float margin_top;
		float margin_right;
		float margin_bottom;
		float padding_left;
		float padding_top;
		float padding_right;
		float padding_bottom;
		EHAlign h_align;
		EVAlign v_align;
		float stack_spacing;

		// Absolute position of this control within its parent's content rect, applied only when
		// the parent's own layout_mode == ELayoutMode::Canvas; ignored otherwise. Any finite value
		// is accepted, including negative, because this is a position rather than a distance.
		float canvas_x;
		float canvas_y;

		// Content shift applied only when this control's own layout_mode == ELayoutMode::Scroll;
		// ignored otherwise. Must be finite and non-negative: a scroll offset is a distance the
		// content has been shifted by, not an arbitrary position.
		float scroll_offset_x;
		float scroll_offset_y;
	};

	// Explicit world-anchoring inputs for a Root whose root_policy is world-anchored; ignored
	// entirely for ERootPolicy::Screen and for every non-Root control. The transform is closed by
	// construction: a world root is always a camera-facing billboard positioned at 'anchor', so
	// the wire model never carries a free-form matrix and the projection stays a pure,
	// deterministic function of the anchor, the sizing rule, and the host camera.
	struct WorldRootParams
	{
		// World-space position the root's anchor point sits at. Any finite value is accepted.
		Vec3 anchor;

		// Which point of the root's DIP box coincides with 'anchor'.
		EAnchorPoint anchor_h;
		EAnchorPoint anchor_v;

		// How the root's DIP box converts to apparent screen size.
		EWorldSizing sizing;

		// World units spanned by one DIP of the root's box under EWorldSizing::WorldUnits; must
		// be finite and greater than zero. Ignored under EWorldSizing::ConstantDip.
		float world_units_per_dip;

		// World units the recorded depth is pulled toward the camera by, so a DepthTested root
		// sitting exactly on a surface does not z-fight with it. Must be finite and non-negative.
		float depth_offset;

		// Occlusion fade bounds, applied only under ERootPolicy::OcclusionFaded. The opacity
		// multiplier ramps linearly from 1 (unoccluded) down to 'occlusion_min_opacity' once the
		// occluding surface is 'occlusion_fade_depth' world units or more in front of the root.
		// 'occlusion_min_opacity' must be finite and within [0, 1], 'occlusion_fade_depth' finite
		// and greater than zero, and 'occlusion_depth_bias' finite and non-negative. The
		// parameters only scale the alpha the renderer already computed; they can neither mutate
		// authoritative state nor select different commands.
		float occlusion_min_opacity;
		float occlusion_fade_depth;
		float occlusion_depth_bias;
	};

	// One closed-vocabulary control descriptor (the 'Control' EStructId). Every field is present
	// for every control type; fields that do not apply to a given type are ignored by validation
	// and layout (for example root_policy only applies when type == Root).
	struct ControlDesc
	{
		StructHeader header;
		ControlId id;
		ControlId parent_id;
		EControlType type;
		ERootPolicy root_policy;
		ELayoutMode layout_mode;
		TemplateId template_id;
		StyleId style_id;
		std::int32_t enabled;
		std::int32_t visible;
		std::int32_t focusable;
		EValidationState validation_state;
		LayoutParams layout;
		std::uint32_t text_offset;
		std::uint32_t text_length;
		std::uint32_t name_offset;
		std::uint32_t name_length;
		std::uint32_t desc_offset;
		std::uint32_t desc_length;
		// Maximum length of this control's text in *Unicode text units* - extended grapheme
		// clusters - not bytes or code points, so a limit of five admits five emoji rather than
		// five bytes of one. Zero means unbounded. Editing operations truncate at a cluster
		// boundary; the application's own text is never truncated by this module.
		std::uint32_t max_text_length;

		// Optional reference to a Resource whose kind is EResourceKind::Font, resolved by the M3
		// renderer for this control's own text (Text) or generated label text (Button/TextBox); 0
		// selects the built-in Segoe UI fallback at a fixed default size. A resource that *does*
		// name a family keeps that name verbatim: an unresolvable family is reported as
		// EStatus::MissingAsset rather than silently substituted with an unrelated face. Ignored by
		// every control type that never displays text, and by validation/layout in the same way as
		// other fields that only apply to a subset of types.
		ResourceId font_resource_id;

		// Durable, application-set flag driving EStateChannel::Selected ("!=0 means true", the
		// same convention as 'enabled'/'visible'/'focusable'); has no other built-in behaviour
		// (e.g. it does not affect hit-testing, focus, or single-selection bookkeeping).
		std::int32_t selected;

		// Application-owned monotonic counter: any change from its previously observed value fires
		// EStateChannel::ValueChanged for exactly one Update() call. The application bumps this
		// (e.g. after committing new TextBox text) to play a "value changed" style transition; the
		// specific numeric value carries no other meaning.
		std::uint32_t value_sequence;

		// World anchoring for a Root whose root_policy is world-anchored; ignored otherwise.
		WorldRootParams world;
	};

	// Explicit child order for one parent (section 5.3). 'offset'/'count' index into the shared
	// Transaction::child_ids array so ordering is independent of Upsert arrival order.
	struct ChildOrder
	{
		ControlId parent_id;
		std::uint32_t offset;
		std::uint32_t count;
	};

	// One closed-vocabulary transaction operation (the 'Operation' EStructId).
	//  - Upsert: create or update the control whose ControlDesc.id == target_id (the descriptor
	//    must be present exactly once in Transaction::controls).
	//  - Remove: delete target_id and its entire subtree.
	//  - Reorder: replace target_id's child order with the ChildOrder entry at child_order_index.
	//  - ReplaceSubtree: delete target_id's existing subtree (if present) before any Upsert in the
	//    same transaction may recreate target_id and its descendants; this lets an application
	//    replace a whole subtree without enumerating every descendant Remove individually.
	struct Operation
	{
		StructHeader header;
		EOperationKind kind;
		ControlId target_id;
		std::uint32_t child_order_index;
		std::uint32_t reserved0;
	};

	// One named, closed-vocabulary resource (the 'Resource' EStructId). When kind == Font, 'name'
	// is the font family (an empty name selects the built-in Segoe UI fallback), 'font_size' is
	// its DIP size (a value <= 0 selects a fixed default size), and 'colour' is the text colour
	// used to paint every TextPresenter item that resolves to this font (a fully-transparent
	// colour, i.e. alpha <= 0 - which includes a zero-initialized/never-set field - selects a
	// fixed opaque-black fallback rather than rendering invisible text).
	//
	// Font resolution policy: a *named* family must exist in the system font collection. It is
	// never replaced by a different face, so a typo or a font the machine does not have is
	// reported as EStatus::MissingAsset instead of rendering in something arbitrary. Only the
	// empty name takes the built-in fallback, and only because it explicitly asks for it.
	// Determinism follows directly: layout and render output for a given string are reproducible
	// across machines that have the exact same font resource installed. Per-character fallback for
	// scripts the requested face does not cover is performed by the system font fallback chain and
	// is *not* claimed to be pixel-identical across machines or OS versions.
	struct ResourceDesc
	{
		StructHeader header;
		ResourceId id;
		EResourceKind kind;
		Colour colour;
		std::uint32_t name_offset;
		std::uint32_t name_length;
		float font_size;
	};

	// Visual appearance applied while a control is in one state channel.
	struct StyleVisual
	{
		Colour fill;
		Colour border_colour;
		float border_thickness;
		float corner_radius;
		float opacity;
	};

	// Bounded, host-time-driven visual transition applied when a control's active state channel
	// changes (section 6.4). Duration is clamped to Config::max_transition_duration_ms.
	struct TransitionDesc
	{
		float duration_ms;
		EEasing easing;
	};

	// One closed-vocabulary style (the 'Style' EStructId): one StyleVisual/TransitionDesc pair per
	// declared state channel, applied to a control as a whole rather than per template part. This
	// keeps the descriptor fixed-layout while still exercising every channel the demo needs.
	struct StyleDesc
	{
		StructHeader header;
		StyleId id;
		StyleVisual visuals[static_cast<std::size_t>(EStateChannel::Count)];
		TransitionDesc transitions[static_cast<std::size_t>(EStateChannel::Count)];
	};

	// One named template part (section 6.3). 'name' is a blob range naming a semantic part such as
	// "PART_ContentPresenter"; behaviour targets these names, not hard-coded geometry.
	struct TemplatePart
	{
		std::uint32_t name_offset;
		std::uint32_t name_length;
		EVisualPrimitive primitive;
		std::int32_t required;
	};

	// One closed-vocabulary lookless template (the 'Template' EStructId). Parts are embedded
	// directly (bounded by VIEW3D_UI_MAX_TEMPLATE_PARTS) so the descriptor stays fixed-layout.
	struct TemplateDesc
	{
		StructHeader header;
		TemplateId id;
		EControlType applies_to;
		std::uint32_t part_count;
		TemplatePart parts[VIEW3D_UI_MAX_TEMPLATE_PARTS];
	};

	// One complete delta transaction (the 'Transaction' EStructId), applied as a single bulk
	// interop call (section 5.3). Every array is a borrowed pointer plus count; View3DUI copies
	// everything it needs into staging storage before validation and never retains these pointers
	// past the call.
	struct Transaction
	{
		StructHeader header;
		std::uint64_t base_revision;
		std::uint64_t revision;
		Operation const* operations;
		std::uint32_t operation_count;
		ControlDesc const* controls;
		std::uint32_t control_count;
		ChildOrder const* child_orders;
		std::uint32_t child_order_count;
		ControlId const* child_ids;
		std::uint32_t child_id_count;
		ResourceDesc const* resources;
		std::uint32_t resource_count;
		StyleDesc const* styles;
		std::uint32_t style_count;
		TemplateDesc const* templates;
		std::uint32_t template_count;

		// Resource/style/template ids to remove after 'operations' has been fully applied and
		// before the whole-tree consistency pass (so a transaction may both stop referencing an id
		// via Upsert and remove it in the same call). Each id must currently exist and must not be
		// referenced by any control still present once 'operations' has run, or the whole
		// transaction is rejected atomically: EStatus::UnknownResource if the id does not exist,
		// EStatus::ResourceInUse if a live control still references it.
		ResourceId const* resource_removals;
		std::uint32_t resource_removal_count;
		StyleId const* style_removals;
		std::uint32_t style_removal_count;
		TemplateId const* template_removals;
		std::uint32_t template_removal_count;

		std::byte const* blob;
		std::uint32_t blob_length;
	};

	// One normalized input record (the 'NormalizedInput' EStructId). Raw Win32 messages translate
	// into this record before entering the control-state machine; deterministic test injection
	// supplies the same record directly (section 7.2).
	struct NormalizedInput
	{
		StructHeader header;
		EInputKind kind;
		float pointer_x;
		float pointer_y;
		EPointerButton button;
		std::uint32_t button_mask;
		float wheel_delta;
		std::int32_t vk;
		std::uint32_t modifiers;
		std::uint32_t char_code;
		double time_ms;
	};

	// Variable-length text accompanying one NormalizedInput record (the 'InputTextPayload'
	// EStructId, section 7.2). Kept separate from NormalizedInput so that record's fixed layout is
	// unchanged and every input still travels as a single bounded struct. 'text_utf8' is borrowed
	// for the duration of the call and is copied before it returns; it must be valid UTF-8, and it
	// is not required to be nul-terminated.
	//
	// 'caret'/'selection_start'/'selection_end' are UTF-8 byte offsets within the payload text and
	// describe the IME's own cursor and target clause during CompositionUpdate; they are ignored
	// for every other input kind. An offset that is out of range, that lands inside a code point,
	// or a selection whose start exceeds its end, is EStatus::InvalidArgument rather than being
	// rounded to something plausible.
	struct InputTextPayload
	{
		StructHeader header;
		char const* text_utf8;
		std::uint32_t text_length;
		std::uint32_t caret;
		std::uint32_t selection_start;
		std::uint32_t selection_end;
	};

	// Explicit host camera description supplied with every ViewportState. View3DUI never queries
	// View3D for a camera: the application is the single authority and is responsible for handing
	// the same camera to View3DUI_Update and to the View3D scene it is rendering, so projected
	// world roots line up with the scene. 'valid' is 0 when no camera exists this frame, which
	// deterministically culls every world-anchored root without failing the update.
	struct CameraState
	{
		Vec3 position;
		Vec3 forward;           // Camera look direction; must be finite and non-degenerate when valid != 0.
		Vec3 up;                // Camera up hint; must be finite and not parallel to 'forward' when valid != 0.
		EProjection projection;
		std::int32_t valid;     // "!=0 means true", the same convention as ControlDesc::enabled.
		float fov_y_rad;        // Vertical field of view; must be finite and within (0, pi) under EProjection::Perspective.
		float ortho_height;     // World units spanned by the viewport height; must be finite and > 0 under EProjection::Orthographic.
		float near_plane;       // Must be finite and > 0.
		float far_plane;        // Must be finite and > near_plane.
	};

	// Explicit host viewport/time state (the 'ViewportState' EStructId), supplied to View3DUI_Update
	// (section 7.4). All coordinate spaces are explicit; View3DUI performs no implicit DPI query.
	//
	// Every DIP value View3DUI produces or consumes - layout rects, hit-test positions, semantic
	// bounds and draw-item bounds - is in one space: viewport-relative target DIPs. Its origin is
	// 'viewport_x_px'/'viewport_y_px' within the render target and one unit is 1/96in at 'dpi', so
	// a client pixel maps in as (client_px * target/client - viewport_origin) * 96/dpi. The
	// client/target pair therefore only ever converts host pointer coordinates in and accessibility
	// rectangles back out; it never sizes or positions anything.
	struct ViewportState
	{
		StructHeader header;
		std::uint32_t client_width_px;
		std::uint32_t client_height_px;
		std::uint32_t target_width_px;
		std::uint32_t target_height_px;
		float viewport_x_px;
		float viewport_y_px;
		float viewport_width_px;
		float viewport_height_px;
		float dpi;
		double time_ms;

		// Camera used to project world-anchored roots this update; zero-initialised (valid == 0)
		// by an application that only uses screen roots.
		CameraState camera;
	};

	// One buffered, owner-thread-drained event (the 'Event' EStructId, section 5.4). 'payload_*'
	// indexes into the caller-owned payload buffer supplied to View3DUI_EventsCopy.
	struct Event
	{
		StructHeader header;
		ControlId control_id;
		EEventKind kind;
		std::uint64_t accepted_revision;
		std::uint64_t sequence;
		std::uint32_t payload_offset;
		std::uint32_t payload_length;
		std::uint32_t edit_generation;
		std::uint32_t reserved0;
	};

	// One flat, deterministic pre-order semantic record (the 'SemanticNode' EStructId, section 5.5).
	struct SemanticNode
	{
		StructHeader header;
		ControlId id;
		ControlId parent_id;
		EControlType role;
		std::uint32_t name_offset;
		std::uint32_t name_length;
		std::uint32_t desc_offset;
		std::uint32_t desc_length;
		std::uint32_t value_offset;
		std::uint32_t value_length;
		std::uint32_t state_flags;
		std::uint32_t supported_actions;

		// Text-range information for an editable control, sufficient for a later UI Automation
		// Text provider to describe the caret, the selection and any in-progress IME composition
		// without needing a second query. All four are UTF-8 byte offsets into the node's value
		// text, always on grapheme-cluster boundaries, and all are 0 when 'text_flags' reports the
		// corresponding feature is absent. 'composition_length' is 0 whenever no composition is
		// active, so a consumer never has to distinguish "no composition" from "empty composition".
		std::uint32_t caret;
		std::uint32_t selection_start;
		std::uint32_t selection_end;
		std::uint32_t composition_start;
		std::uint32_t composition_length;
		std::uint32_t text_flags;          // Bitmask of ESemanticTextFlag.
		std::uint32_t value_grapheme_count; // Length of the value text in Unicode text units, the unit max_text_length bounds.
		std::uint32_t reserved0;

		Rect bounds;
		std::uint64_t accepted_revision;
		std::uint64_t semantic_sequence;
	};

	// Bounded runtime counters and last-failure category (the 'Diagnostics' EStructId).
	struct Diagnostics
	{
		StructHeader header;
		std::uint64_t accepted_revision;
		std::uint64_t rejected_revision_attempts;
		std::uint32_t control_count;
		std::uint32_t queued_event_count;
		std::uint32_t event_overflow_count;
		std::uint32_t semantic_record_count;
		EStatus last_failure_status;
		std::uint32_t reserved0;
	};

	// Reserved placeholder for the private View3D host-bridge version record (the
	// 'HostBridgeVersion' EStructId). Declared here only so View3DUI_StructSize enumerates the
	// complete EStructId set at M0; the bridge itself is implemented by the View3D host, outside
	// this module, in a later milestone.
	struct HostBridgeVersion
	{
		StructHeader header;
		std::uint32_t bridge_abi_version;
	};

	// Reserved placeholder for a future host-owned command-recording context (the
	// 'HostPassContext' EStructId). See HostBridgeVersion; not implemented in this module.
	struct HostPassContext
	{
		StructHeader header;
		std::uint64_t reserved0;
	};

	static_assert(std::is_standard_layout_v<Config>);
	static_assert(std::is_standard_layout_v<LayoutParams>);
	static_assert(std::is_standard_layout_v<WorldRootParams>);
	static_assert(std::is_standard_layout_v<ControlDesc>);
	static_assert(std::is_standard_layout_v<ChildOrder>);
	static_assert(std::is_standard_layout_v<Operation>);
	static_assert(std::is_standard_layout_v<ResourceDesc>);
	static_assert(std::is_standard_layout_v<StyleVisual>);
	static_assert(std::is_standard_layout_v<TransitionDesc>);
	static_assert(std::is_standard_layout_v<StyleDesc>);
	static_assert(std::is_standard_layout_v<TemplatePart>);
	static_assert(std::is_standard_layout_v<TemplateDesc>);
	static_assert(std::is_standard_layout_v<Transaction>);
	static_assert(std::is_standard_layout_v<NormalizedInput>);
	static_assert(std::is_standard_layout_v<InputTextPayload>);
	static_assert(std::is_standard_layout_v<CameraState>);
	static_assert(std::is_standard_layout_v<ViewportState>);
	static_assert(std::is_standard_layout_v<Event>);
	static_assert(std::is_standard_layout_v<SemanticNode>);
	static_assert(std::is_standard_layout_v<Diagnostics>);
	static_assert(std::is_standard_layout_v<HostBridgeVersion>);
	static_assert(std::is_standard_layout_v<HostPassContext>);
	static_assert(sizeof(Vec2) == 8);
	static_assert(sizeof(Vec3) == 12);
	static_assert(sizeof(Rect) == 16);
	static_assert(sizeof(Colour) == 16);
	static_assert(sizeof(WorldRootParams) == 44);
	static_assert(sizeof(CameraState) == 60);
}
