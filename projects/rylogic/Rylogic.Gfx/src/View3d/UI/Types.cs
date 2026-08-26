using System;
using System.Runtime.InteropServices;

namespace Rylogic.Gfx.UI;

/// <summary>Combines the hash codes of several values without requiring System.HashCode (unavailable on the net481 target).</summary>
internal static class HashUtil
{
	/// <summary>Combine the hash codes of 'values' into one hash code.</summary>
	internal static int Combine(params object?[] values)
	{
		unchecked
		{
			var hash = 17;
			foreach (var value in values)
				hash = hash * 31 + (value?.GetHashCode() ?? 0);

			return hash;
		}
	}
}

/// <summary>Result of a View3DUI API operation. Every failure is a specific, stable category; see include/pr/view3d-ui/types.h.</summary>
public enum EStatus
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
}

/// <summary>Identifies one public wire structure discoverable through View3DUI_StructSize.</summary>
public enum EStructId
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
}

/// <summary>Closed control vocabulary for the first vertical slice. Applications cannot register new control classes.</summary>
public enum EControlType
{
	Root = 0,
	Panel = 1,
	Text = 2,
	TextBox = 3,
	Button = 4,
	Count = 5,
}

/// <summary>
/// Closed layout vocabulary. Overlay and the two Stack orientations place a child by alignment/margins; Scroll shifts its own content by
/// LayoutParams.ScrollOffsetX/Y before applying the same placement rule as Overlay; Canvas places each child at the absolute
/// LayoutParams.CanvasX/Y offset from its parent's content origin, ignoring alignment/margins entirely.
/// </summary>
public enum ELayoutMode
{
	Overlay = 0,
	StackHorizontal = 1,
	StackVertical = 2,
	Scroll = 3,
	Canvas = 4,
	Count = 5,
}

/// <summary>
/// Root render policy. 'Screen' places the root in the host's screen-space final-overlay phase. The remaining three are
/// world-anchored: the root's DIP box is laid out in its own local space and then projected about WorldRootParams.Anchor.
/// </summary>
public enum ERootPolicy
{
	Screen = 0,
	DepthTested = 1,
	OcclusionFaded = 2,
	Overlay = 3,
	Count = 4,
}

/// <summary>How a world-anchored root converts its DIP layout box into apparent screen size.</summary>
public enum EWorldSizing
{
	ConstantDip = 0,
	WorldUnits = 1,
	Count = 2,
}

/// <summary>Which point of a world root's DIP box coincides with WorldRootParams.Anchor, per axis.</summary>
public enum EAnchorPoint
{
	Min = 0,
	Centre = 1,
	Max = 2,
	Count = 3,
}

/// <summary>Host camera projection kind, stored explicitly rather than inferred from the field-of-view value.</summary>
public enum EProjection
{
	Perspective = 0,
	Orthographic = 1,
	Count = 2,
}

/// <summary>Horizontal alignment of a control within its parent's cross-axis or overlay slot.</summary>
public enum EHAlign
{
	Left = 0,
	Center = 1,
	Right = 2,
	Stretch = 3,
	Count = 4,
}

/// <summary>Vertical alignment of a control within its parent's cross-axis or overlay slot.</summary>
public enum EVAlign
{
	Top = 0,
	Center = 1,
	Bottom = 2,
	Stretch = 3,
	Count = 4,
}

/// <summary>Closed visual primitive vocabulary usable by a lookless template. Applications compose these but cannot submit shaders or draw callbacks.</summary>
public enum EVisualPrimitive
{
	SolidBox = 0,
	RoundedBox = 1,
	Border = 2,
	TextPresenter = 3,
	ContentPresenter = 4,
	Clip = 5,
	TransformOpacityGroup = 6,
	Count = 7,
}

/// <summary>
/// Closed style/transition state channel vocabulary. Selected is a durable per-control flag (ControlDesc.Selected); Visibility fires for
/// one Update() call whenever a control's own Visible field transitions off to on; ValueChanged fires for one Update() call whenever
/// ControlDesc.ValueSequence changes from its previously observed value. Resolution priority when more than one channel could apply
/// (highest first): Disabled, Pressed, Invalid, Focused, then Visibility/ValueChanged (whichever is active), then Hover, Selected, Normal.
/// </summary>
public enum EStateChannel
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
}

/// <summary>Closed transition easing vocabulary.</summary>
public enum EEasing
{
	Linear = 0,
	EaseInOut = 1,
	Count = 2,
}

/// <summary>Closed durable validation state for a control's proposed/accepted value. The application computes this; View3DUI only reads it.</summary>
public enum EValidationState
{
	NotApplicable = 0,
	Pending = 1,
	Invalid = 2,
	Valid = 3,
	Count = 4,
}

/// <summary>Closed transaction operation vocabulary.</summary>
public enum EOperationKind
{
	Upsert = 0,
	Remove = 1,
	Reorder = 2,
	ReplaceSubtree = 3,
	Count = 4,
}

/// <summary>Closed resource kind vocabulary.</summary>
public enum EResourceKind
{
	Colour = 0,
	Font = 1,
	Count = 2,
}

/// <summary>Closed normalized input kind vocabulary. Raw Win32 messages and deterministic test injection both translate into this vocabulary.</summary>
public enum EInputKind
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

	/// <summary>
	/// Committed text of arbitrary length, injected via UiContext.InjectInputText. Distinct from Char, which carries
	/// exactly one scalar value inline: a paste, an injected string, or the character a dead-key sequence produced can
	/// all be more than one code point.
	/// </summary>
	TextInput = 9,

	/// <summary>
	/// IME composition lifecycle. A composition is transient native state that never reaches the application until
	/// CompositionCommit converts it into a text-change proposal. CompositionStart/CompositionCancel carry no text
	/// payload; CompositionUpdate carries the live composition string with the IME's caret/clause offsets;
	/// CompositionCommit carries the result string.
	/// </summary>
	CompositionStart = 10,
	CompositionUpdate = 11,
	CompositionCommit = 12,
	CompositionCancel = 13,

	Count = 14,
}

/// <summary>Ordinal pointer button identifier used by PointerButtonDown/Up.</summary>
public enum EPointerButton
{
	None = 0,
	Left = 1,
	Right = 2,
	Middle = 3,
	X1 = 4,
	X2 = 5,
	Count = 6,
}

/// <summary>Bitmask of currently held pointer buttons, valid on PointerMove.</summary>
[Flags]
public enum EPointerButtonMask : uint
{
	None = 0,
	Left = 1U << 0,
	Right = 1U << 1,
	Middle = 1U << 2,
	X1 = 1U << 3,
	X2 = 1U << 4,
}

/// <summary>Bitmask of held modifier keys.</summary>
[Flags]
public enum EInputModifier : uint
{
	None = 0,
	Shift = 1U << 0,
	Ctrl = 1U << 1,
	Alt = 1U << 2,
}

/// <summary>Closed typed event vocabulary.</summary>
public enum EEventKind
{
	FocusChanged = 0,
	TextChangeProposed = 1,
	CommandInvoked = 2,
	PointerCaptureChanged = 3,
	QueueOverflow = 4,
	Diagnostic = 5,
	Count = 6,
}

/// <summary>Bitmask of UI-Automation-style actions a semantic node currently supports.</summary>
[Flags]
public enum ESemanticAction : uint
{
	None = 0,
	Invoke = 1U << 0,
	SetValue = 1U << 1,
	Focus = 1U << 2,

	/// <summary>The node exposes a caret and a selection range that a text pattern could move; reported only for an editable control.</summary>
	SetSelection = 1U << 3,
}

/// <summary>Bitmask of boolean semantic state flags, packed to keep SemanticNode compact.</summary>
[Flags]
public enum ESemanticState : uint
{
	None = 0,
	Enabled = 1U << 0,
	Visible = 1U << 1,
	Focused = 1U << 2,
	Focusable = 1U << 3,
	Selected = 1U << 4,
	Invalid = 1U << 5,
	Offscreen = 1U << 6,
}

/// <summary>
/// Bitmask describing which of a SemanticNode's text-range fields carry meaning. Reporting presence explicitly keeps a
/// legitimate zero offset (a caret at the start of the value text) distinguishable from "this node has no caret at all".
/// </summary>
[Flags]
public enum ESemanticTextFlag : uint
{
	None = 0,

	/// <summary>UiSemanticNode.Caret is a live caret position within the value text.</summary>
	HasCaret = 1U << 0,

	/// <summary>UiSemanticNode.SelectionStart/SelectionEnd describe a non-empty selection.</summary>
	HasSelection = 1U << 1,

	/// <summary>UiSemanticNode.CompositionStart/CompositionLength describe live IME composition text.</summary>
	Composing = 1U << 2,
}

/// <summary>An application-assigned, process-unique control identity. View3DUI never generates these; the application is the sole authority.</summary>
[StructLayout(LayoutKind.Sequential)]
public readonly struct ControlId : IEquatable<ControlId>
{
	public readonly ulong m_value;

	/// <summary>Create a control identifier from its application-assigned value. Zero means "no control".</summary>
	public ControlId(ulong value)
	{
		m_value = value;
	}

	/// <summary>The sentinel value meaning "no control".</summary>
	public static readonly ControlId None = new(0);

	/// <summary>True when this identifier is the "no control" sentinel.</summary>
	public bool IsNone => m_value == 0;

	/// <inheritdoc/>
	public bool Equals(ControlId other) => m_value == other.m_value;

	/// <inheritdoc/>
	public override bool Equals(object? obj) => obj is ControlId other && Equals(other);

	/// <inheritdoc/>
	public override int GetHashCode() => m_value.GetHashCode();

	/// <inheritdoc/>
	public override string ToString() => m_value.ToString();

	/// <summary>Compare two control identities.</summary>
	public static bool operator ==(ControlId lhs, ControlId rhs) => lhs.Equals(rhs);

	/// <summary>Compare two control identities.</summary>
	public static bool operator !=(ControlId lhs, ControlId rhs) => !lhs.Equals(rhs);
}

/// <summary>An application-assigned, process-unique resource identity. View3DUI never generates these; the application is the sole authority.</summary>
[StructLayout(LayoutKind.Sequential)]
public readonly struct ResourceId : IEquatable<ResourceId>
{
	public readonly ulong m_value;

	/// <summary>Create a resource identifier from its application-assigned value. Zero means "no resource" (built-in fallback).</summary>
	public ResourceId(ulong value)
	{
		m_value = value;
	}

	/// <summary>The sentinel value meaning "no resource" (selects the built-in fallback).</summary>
	public static readonly ResourceId None = new(0);

	/// <summary>True when this identifier is the "no resource" sentinel.</summary>
	public bool IsNone => m_value == 0;

	/// <inheritdoc/>
	public bool Equals(ResourceId other) => m_value == other.m_value;

	/// <inheritdoc/>
	public override bool Equals(object? obj) => obj is ResourceId other && Equals(other);

	/// <inheritdoc/>
	public override int GetHashCode() => m_value.GetHashCode();

	/// <inheritdoc/>
	public override string ToString() => m_value.ToString();

	/// <summary>Compare two resource identities.</summary>
	public static bool operator ==(ResourceId lhs, ResourceId rhs) => lhs.Equals(rhs);

	/// <summary>Compare two resource identities.</summary>
	public static bool operator !=(ResourceId lhs, ResourceId rhs) => !lhs.Equals(rhs);
}

/// <summary>An application-assigned, process-unique style identity. View3DUI never generates these; the application is the sole authority.</summary>
[StructLayout(LayoutKind.Sequential)]
public readonly struct StyleId : IEquatable<StyleId>
{
	public readonly ulong m_value;

	/// <summary>Create a style identifier from its application-assigned value. Zero means "no style" (built-in default appearance).</summary>
	public StyleId(ulong value)
	{
		m_value = value;
	}

	/// <summary>The sentinel value meaning "no style" (selects the built-in default appearance).</summary>
	public static readonly StyleId None = new(0);

	/// <summary>True when this identifier is the "no style" sentinel.</summary>
	public bool IsNone => m_value == 0;

	/// <inheritdoc/>
	public bool Equals(StyleId other) => m_value == other.m_value;

	/// <inheritdoc/>
	public override bool Equals(object? obj) => obj is StyleId other && Equals(other);

	/// <inheritdoc/>
	public override int GetHashCode() => m_value.GetHashCode();

	/// <inheritdoc/>
	public override string ToString() => m_value.ToString();

	/// <summary>Compare two style identities.</summary>
	public static bool operator ==(StyleId lhs, StyleId rhs) => lhs.Equals(rhs);

	/// <summary>Compare two style identities.</summary>
	public static bool operator !=(StyleId lhs, StyleId rhs) => !lhs.Equals(rhs);
}

/// <summary>An application-assigned, process-unique template identity. View3DUI never generates these; the application is the sole authority.</summary>
[StructLayout(LayoutKind.Sequential)]
public readonly struct TemplateId : IEquatable<TemplateId>
{
	public readonly ulong m_value;

	/// <summary>Create a template identifier from its application-assigned value. Zero means "no template" (built-in default look).</summary>
	public TemplateId(ulong value)
	{
		m_value = value;
	}

	/// <summary>The sentinel value meaning "no template" (selects the built-in default look).</summary>
	public static readonly TemplateId None = new(0);

	/// <summary>True when this identifier is the "no template" sentinel.</summary>
	public bool IsNone => m_value == 0;

	/// <inheritdoc/>
	public bool Equals(TemplateId other) => m_value == other.m_value;

	/// <inheritdoc/>
	public override bool Equals(object? obj) => obj is TemplateId other && Equals(other);

	/// <inheritdoc/>
	public override int GetHashCode() => m_value.GetHashCode();

	/// <inheritdoc/>
	public override string ToString() => m_value.ToString();

	/// <summary>Compare two template identities.</summary>
	public static bool operator ==(TemplateId lhs, TemplateId rhs) => lhs.Equals(rhs);

	/// <summary>Compare two template identities.</summary>
	public static bool operator !=(TemplateId lhs, TemplateId rhs) => !lhs.Equals(rhs);
}

/// <summary>A dependency-minimal 2D point/size in DIPs, mirroring the native Vec2 wire type.</summary>
[StructLayout(LayoutKind.Sequential)]
public readonly struct Vec2 : IEquatable<Vec2>
{
	public readonly float m_x;
	public readonly float m_y;

	/// <summary>Create a 2D point/size.</summary>
	public Vec2(float x, float y)
	{
		m_x = x;
		m_y = y;
	}

	/// <summary>The zero vector.</summary>
	public static readonly Vec2 Zero = new(0, 0);

	/// <inheritdoc/>
	public bool Equals(Vec2 other) => m_x == other.m_x && m_y == other.m_y;

	/// <inheritdoc/>
	public override bool Equals(object? obj) => obj is Vec2 other && Equals(other);

	/// <inheritdoc/>
	public override int GetHashCode() => HashUtil.Combine(m_x, m_y);

	/// <summary>Compare two vectors.</summary>
	public static bool operator ==(Vec2 lhs, Vec2 rhs) => lhs.Equals(rhs);

	/// <summary>Compare two vectors.</summary>
	public static bool operator !=(Vec2 lhs, Vec2 rhs) => !lhs.Equals(rhs);
}

/// <summary>A dependency-minimal 3D point/direction in world units, mirroring the native Vec3 wire type.</summary>
[StructLayout(LayoutKind.Sequential)]
public readonly struct Vec3 : IEquatable<Vec3>
{
	public readonly float m_x;
	public readonly float m_y;
	public readonly float m_z;

	/// <summary>Create a 3D point/direction.</summary>
	public Vec3(float x, float y, float z)
	{
		m_x = x;
		m_y = y;
		m_z = z;
	}

	/// <summary>The zero vector.</summary>
	public static readonly Vec3 Zero = new(0, 0, 0);

	/// <inheritdoc/>
	public bool Equals(Vec3 other) => m_x == other.m_x && m_y == other.m_y && m_z == other.m_z;

	/// <inheritdoc/>
	public override bool Equals(object? obj) => obj is Vec3 other && Equals(other);

	/// <inheritdoc/>
	public override int GetHashCode() => HashUtil.Combine(m_x, m_y, m_z);

	/// <summary>Compare two vectors.</summary>
	public static bool operator ==(Vec3 lhs, Vec3 rhs) => lhs.Equals(rhs);

	/// <summary>Compare two vectors.</summary>
	public static bool operator !=(Vec3 lhs, Vec3 rhs) => !lhs.Equals(rhs);
}

/// <summary>A dependency-minimal axis-aligned rectangle in DIPs, mirroring the native Rect wire type. (X, Y) is the top-left corner.</summary>
[StructLayout(LayoutKind.Sequential)]
public readonly struct Rect : IEquatable<Rect>
{
	public readonly float m_x;
	public readonly float m_y;
	public readonly float m_w;
	public readonly float m_h;

	/// <summary>Create an axis-aligned rectangle.</summary>
	public Rect(float x, float y, float w, float h)
	{
		m_x = x;
		m_y = y;
		m_w = w;
		m_h = h;
	}

	/// <summary>The zero-sized rectangle at the origin.</summary>
	public static readonly Rect Zero = new(0, 0, 0, 0);

	/// <inheritdoc/>
	public bool Equals(Rect other) => m_x == other.m_x && m_y == other.m_y && m_w == other.m_w && m_h == other.m_h;

	/// <inheritdoc/>
	public override bool Equals(object? obj) => obj is Rect other && Equals(other);

	/// <inheritdoc/>
	public override int GetHashCode() => HashUtil.Combine(m_x, m_y, m_w, m_h);

	/// <summary>Compare two rectangles.</summary>
	public static bool operator ==(Rect lhs, Rect rhs) => lhs.Equals(rhs);

	/// <summary>Compare two rectangles.</summary>
	public static bool operator !=(Rect lhs, Rect rhs) => !lhs.Equals(rhs);
}

/// <summary>A straight (non-premultiplied) RGBA colour with components in [0, 1], mirroring the native Colour wire type.</summary>
[StructLayout(LayoutKind.Sequential)]
public readonly struct Colour : IEquatable<Colour>
{
	public readonly float m_r;
	public readonly float m_g;
	public readonly float m_b;
	public readonly float m_a;

	/// <summary>Create a straight RGBA colour.</summary>
	public Colour(float r, float g, float b, float a)
	{
		m_r = r;
		m_g = g;
		m_b = b;
		m_a = a;
	}

	/// <summary>Fully transparent black; also the "use built-in fallback" sentinel for ResourceDesc.Colour on a Font resource.</summary>
	public static readonly Colour TransparentBlack = new(0, 0, 0, 0);

	/// <inheritdoc/>
	public bool Equals(Colour other) => m_r == other.m_r && m_g == other.m_g && m_b == other.m_b && m_a == other.m_a;

	/// <inheritdoc/>
	public override bool Equals(object? obj) => obj is Colour other && Equals(other);

	/// <inheritdoc/>
	public override int GetHashCode() => HashUtil.Combine(m_r, m_g, m_b, m_a);

	/// <summary>Compare two colours.</summary>
	public static bool operator ==(Colour lhs, Colour rhs) => lhs.Equals(rhs);

	/// <summary>Compare two colours.</summary>
	public static bool operator !=(Colour lhs, Colour rhs) => !lhs.Equals(rhs);
}

	/// <summary>The fill/border/opacity appearance applied for one EStateChannel, mirroring the native StyleVisual wire type.</summary>
	[StructLayout(LayoutKind.Sequential)]
	public readonly struct StyleVisual : IEquatable<StyleVisual>
	{
		public readonly Colour m_fill;
		public readonly Colour m_border_colour;
		public readonly float m_border_thickness;
		public readonly float m_corner_radius;
		public readonly float m_opacity;

		/// <summary>Create a state-channel visual.</summary>
		public StyleVisual(Colour fill, Colour border_colour = default, float border_thickness = 0, float corner_radius = 0, float opacity = 1)
		{
			m_fill = fill;
			m_border_colour = border_colour;
			m_border_thickness = border_thickness;
			m_corner_radius = corner_radius;
			m_opacity = opacity;
		}

		/// <inheritdoc/>
		public bool Equals(StyleVisual other) => m_fill == other.m_fill && m_border_colour == other.m_border_colour && m_border_thickness == other.m_border_thickness && m_corner_radius == other.m_corner_radius && m_opacity == other.m_opacity;

		/// <inheritdoc/>
		public override bool Equals(object? obj) => obj is StyleVisual other && Equals(other);

		/// <inheritdoc/>
		public override int GetHashCode() => HashUtil.Combine(m_fill, m_border_colour, m_border_thickness, m_corner_radius, m_opacity);

		/// <summary>Compare two state-channel visuals.</summary>
		public static bool operator ==(StyleVisual lhs, StyleVisual rhs) => lhs.Equals(rhs);

		/// <summary>Compare two state-channel visuals.</summary>
		public static bool operator !=(StyleVisual lhs, StyleVisual rhs) => !lhs.Equals(rhs);
	}

	/// <summary>A bounded, host-time-driven visual transition applied when a control's active state channel changes; duration is clamped to UiConfig.MaxTransitionDurationMs.</summary>
	[StructLayout(LayoutKind.Sequential)]
	public readonly struct TransitionDesc : IEquatable<TransitionDesc>
	{
		public readonly float m_duration_ms;
		public readonly EEasing m_easing;

		/// <summary>Create a state-channel transition.</summary>
		public TransitionDesc(float duration_ms, EEasing easing = EEasing.Linear)
		{
			m_duration_ms = duration_ms;
			m_easing = easing;
		}

		/// <summary>No transition: the new visual applies instantly.</summary>
		public static readonly TransitionDesc None = new(0, EEasing.Linear);

		/// <inheritdoc/>
		public bool Equals(TransitionDesc other) => m_duration_ms == other.m_duration_ms && m_easing == other.m_easing;

		/// <inheritdoc/>
		public override bool Equals(object? obj) => obj is TransitionDesc other && Equals(other);

		/// <inheritdoc/>
		public override int GetHashCode() => HashUtil.Combine(m_duration_ms, m_easing);

		/// <summary>Compare two transitions.</summary>
		public static bool operator ==(TransitionDesc lhs, TransitionDesc rhs) => lhs.Equals(rhs);

		/// <summary>Compare two transitions.</summary>
		public static bool operator !=(TransitionDesc lhs, TransitionDesc rhs) => !lhs.Equals(rhs);
	}

	/// <summary>
	/// One normalized input event (the 'NormalizedInput' EStructId), injected via UiContext.InjectInput. Both raw Win32 message
	/// translation and deterministic test injection produce this same closed vocabulary; only the fields relevant to Kind are
	/// meaningful, the rest are left at their default (zero) value. Use the static factory methods to construct one safely.
	/// </summary>
	[StructLayout(LayoutKind.Sequential)]
	public readonly struct NormalizedInput
	{
		internal readonly NativeHeader m_header;
		public readonly EInputKind m_kind;
		public readonly float m_pointer_x;
		public readonly float m_pointer_y;
		public readonly EPointerButton m_button;
		public readonly EPointerButtonMask m_button_mask;
		public readonly float m_wheel_delta;
		public readonly int m_vk;
		public readonly EInputModifier m_modifiers;
		public readonly uint m_char_code;
		public readonly double m_time_ms;

		/// <summary>Construct a fully-specified input event. Prefer the static factory methods below for the common cases.</summary>
		public NormalizedInput(EInputKind kind, float pointer_x = 0, float pointer_y = 0, EPointerButton button = EPointerButton.None, EPointerButtonMask button_mask = EPointerButtonMask.None, float wheel_delta = 0, int vk = 0, EInputModifier modifiers = EInputModifier.None, uint char_code = 0, double time_ms = 0)
		{
			m_header = NativeHeader.Create<NormalizedInput>();
			m_kind = kind;
			m_pointer_x = pointer_x;
			m_pointer_y = pointer_y;
			m_button = button;
			m_button_mask = button_mask;
			m_wheel_delta = wheel_delta;
			m_vk = vk;
			m_modifiers = modifiers;
			m_char_code = char_code;
			m_time_ms = time_ms;
		}

		/// <summary>A pointer-move event at (x, y), with the currently-held button mask and modifiers.</summary>
		public static NormalizedInput PointerMove(float x, float y, EPointerButtonMask buttons, EInputModifier modifiers, double time_ms) =>
			new(EInputKind.PointerMove, pointer_x: x, pointer_y: y, button_mask: buttons, modifiers: modifiers, time_ms: time_ms);

		/// <summary>A pointer-button-down event at (x, y).</summary>
		public static NormalizedInput PointerButtonDown(float x, float y, EPointerButton button, EPointerButtonMask buttons, EInputModifier modifiers, double time_ms) =>
			new(EInputKind.PointerButtonDown, pointer_x: x, pointer_y: y, button: button, button_mask: buttons, modifiers: modifiers, time_ms: time_ms);

		/// <summary>A pointer-button-up event at (x, y).</summary>
		public static NormalizedInput PointerButtonUp(float x, float y, EPointerButton button, EPointerButtonMask buttons, EInputModifier modifiers, double time_ms) =>
			new(EInputKind.PointerButtonUp, pointer_x: x, pointer_y: y, button: button, button_mask: buttons, modifiers: modifiers, time_ms: time_ms);

		/// <summary>A pointer wheel event at (x, y); positive delta scrolls forward/up.</summary>
		public static NormalizedInput PointerWheel(float x, float y, float wheel_delta, EInputModifier modifiers, double time_ms) =>
			new(EInputKind.PointerWheel, pointer_x: x, pointer_y: y, wheel_delta: wheel_delta, modifiers: modifiers, time_ms: time_ms);

		/// <summary>A virtual-key-down event.</summary>
		public static NormalizedInput KeyDown(int vk, EInputModifier modifiers, double time_ms) =>
			new(EInputKind.KeyDown, vk: vk, modifiers: modifiers, time_ms: time_ms);

		/// <summary>A virtual-key-up event.</summary>
		public static NormalizedInput KeyUp(int vk, EInputModifier modifiers, double time_ms) =>
			new(EInputKind.KeyUp, vk: vk, modifiers: modifiers, time_ms: time_ms);

		/// <summary>A translated character event (e.g. from WM_CHAR).</summary>
		public static NormalizedInput Char(uint char_code, double time_ms) =>
			new(EInputKind.Char, char_code: char_code, time_ms: time_ms);

		/// <summary>The host window gained keyboard focus.</summary>
		public static NormalizedInput FocusGained(double time_ms) => new(EInputKind.FocusGained, time_ms: time_ms);

		/// <summary>The host window lost keyboard focus.</summary>
		public static NormalizedInput FocusLost(double time_ms) => new(EInputKind.FocusLost, time_ms: time_ms);

		/// <summary>Committed text of arbitrary length; pair with UiContext.InjectInputText to carry the text itself.</summary>
		public static NormalizedInput TextInput(double time_ms)
		{
			return new NormalizedInput(EInputKind.TextInput, time_ms: time_ms);
		}

		/// <summary>The start of an IME composition; carries no text payload.</summary>
		public static NormalizedInput CompositionStart(double time_ms)
		{
			return new NormalizedInput(EInputKind.CompositionStart, time_ms: time_ms);
		}

		/// <summary>A live IME composition update; pair with UiContext.InjectInputText to carry the composition string and caret/clause offsets.</summary>
		public static NormalizedInput CompositionUpdate(double time_ms)
		{
			return new NormalizedInput(EInputKind.CompositionUpdate, time_ms: time_ms);
		}

		/// <summary>The IME composition result; pair with UiContext.InjectInputText to carry the committed string.</summary>
		public static NormalizedInput CompositionCommit(double time_ms)
		{
			return new NormalizedInput(EInputKind.CompositionCommit, time_ms: time_ms);
		}

		/// <summary>The IME composition was cancelled, restoring the pre-composition edit exactly; carries no text payload.</summary>
		public static NormalizedInput CompositionCancel(double time_ms)
		{
			return new NormalizedInput(EInputKind.CompositionCancel, time_ms: time_ms);
		}
	}

	/// <summary>
	/// Explicit host camera description supplied with every ViewportState. View3DUI never queries a live camera itself: the
	/// application is the single authority and must hand the same camera to UiContext.Update and to the View3D scene it is
	/// rendering, so projected world roots line up with the scene. Use 'None' for screen-only applications.
	/// </summary>
	[StructLayout(LayoutKind.Sequential)]
	public readonly struct CameraState : IEquatable<CameraState>
	{
		public readonly Vec3 m_position;
		public readonly Vec3 m_forward;
		public readonly Vec3 m_up;
		public readonly EProjection m_projection;
		private readonly int m_valid;
		public readonly float m_fov_y_rad;
		public readonly float m_ortho_height;
		public readonly float m_near_plane;
		public readonly float m_far_plane;

		/// <summary>Create a valid camera description. 'forward' must be finite and non-degenerate; 'up' must not be parallel to 'forward'.</summary>
		public CameraState(Vec3 position, Vec3 forward, Vec3 up, EProjection projection, float fov_y_rad, float ortho_height, float near_plane, float far_plane)
		{
			m_position = position;
			m_forward = forward;
			m_up = up;
			m_projection = projection;
			m_valid = 1;
			m_fov_y_rad = fov_y_rad;
			m_ortho_height = ortho_height;
			m_near_plane = near_plane;
			m_far_plane = far_plane;
		}

		/// <summary>No camera this frame: every world-anchored root is deterministically culled without failing the update.</summary>
		public static readonly CameraState None = default;

		/// <summary>True unless this is the 'None' sentinel.</summary>
		public bool IsValid => m_valid != 0;

		/// <inheritdoc/>
		public bool Equals(CameraState other) =>
			m_position == other.m_position && m_forward == other.m_forward && m_up == other.m_up && m_projection == other.m_projection &&
			m_valid == other.m_valid && m_fov_y_rad == other.m_fov_y_rad && m_ortho_height == other.m_ortho_height &&
			m_near_plane == other.m_near_plane && m_far_plane == other.m_far_plane;

		/// <inheritdoc/>
		public override bool Equals(object? obj) => obj is CameraState other && Equals(other);

		/// <inheritdoc/>
		public override int GetHashCode() => HashUtil.Combine(m_position, m_forward, m_up, m_projection, m_valid, m_fov_y_rad);

		/// <summary>Compare two camera states.</summary>
		public static bool operator ==(CameraState lhs, CameraState rhs) => lhs.Equals(rhs);

		/// <summary>Compare two camera states.</summary>
		public static bool operator !=(CameraState lhs, CameraState rhs) => !lhs.Equals(rhs);
	}

	/// <summary>
	/// Explicit host viewport/time state (the 'ViewportState' EStructId), supplied to UiContext.Update. All coordinate spaces
	/// are explicit; View3DUI performs no implicit DPI query.
	/// </summary>
	[StructLayout(LayoutKind.Sequential)]
	public readonly struct ViewportState
	{
		internal readonly NativeHeader m_header;
		public readonly uint m_client_width_px;
		public readonly uint m_client_height_px;
		public readonly uint m_target_width_px;
		public readonly uint m_target_height_px;
		public readonly float m_viewport_x_px;
		public readonly float m_viewport_y_px;
		public readonly float m_viewport_width_px;
		public readonly float m_viewport_height_px;
		public readonly float m_dpi;
		public readonly double m_time_ms;
		public readonly CameraState m_camera;

		/// <summary>Create a viewport/time snapshot for one Update() call. 'camera' defaults to CameraState.None for screen-only applications.</summary>
		public ViewportState(uint client_width_px, uint client_height_px, uint target_width_px, uint target_height_px, float viewport_x_px, float viewport_y_px, float viewport_width_px, float viewport_height_px, float dpi, double time_ms, CameraState camera = default)
		{
			m_header = NativeHeader.Create<ViewportState>();
			m_client_width_px = client_width_px;
			m_client_height_px = client_height_px;
			m_target_width_px = target_width_px;
			m_target_height_px = target_height_px;
			m_viewport_x_px = viewport_x_px;
			m_viewport_y_px = viewport_y_px;
			m_viewport_width_px = viewport_width_px;
			m_viewport_height_px = viewport_height_px;
			m_dpi = dpi;
			m_time_ms = time_ms;
			m_camera = camera;
		}
	}

	/// <summary>
	/// A point-in-time snapshot of engine-wide bookkeeping counters and the most recent internally-observed failure (the
	/// 'Diagnostics' EStructId), read via UiContext.GetDiagnostics. Only populated by the native runtime; applications read it.
	/// </summary>
	[StructLayout(LayoutKind.Sequential)]
	public struct UiDiagnostics
	{
		internal NativeHeader m_header;
		public ulong m_accepted_revision;
		public ulong m_rejected_revision_attempts;
		public uint m_control_count;
		public uint m_queued_event_count;
		public uint m_event_overflow_count;
		public uint m_semantic_record_count;
		public EStatus m_last_failure_status;
		private uint m_reserved0;

		/// <summary>Create a zeroed diagnostics record with a header ready for View3DUI_DiagnosticsGet to populate.</summary>
		internal static UiDiagnostics ForQuery() => new() { m_header = NativeHeader.Create<UiDiagnostics>() };
	}
