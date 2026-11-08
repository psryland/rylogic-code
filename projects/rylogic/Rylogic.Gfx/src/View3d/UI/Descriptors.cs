using System;
using System.Collections.Generic;

namespace Rylogic.Gfx.UI;

/// <summary>
/// Tunable capacity/behaviour limits for a UiContext, supplied once at UiRuntime.CreateContext. Every limit is a hard cap:
/// exceeding one at runtime fails the offending call with EStatus.ResourceLimit rather than growing silently. Defaults match
/// the native engine's own built-in defaults (see engine.cpp DefaultConfig()). A record: two instances with equal property
/// values compare equal, and 'with' produces an independent copy since every property here is a value type.
/// </summary>
public sealed record UiConfig
{
	public uint MaxControls { get; set; } = 4096;
	public uint MaxRoots { get; set; } = 64;
	public uint MaxTreeDepth { get; set; } = 64;
	public uint MaxOperationsPerTransaction { get; set; } = 4096;
	public uint MaxBlobBytes { get; set; } = 1u << 20;
	public uint MaxTemplates { get; set; } = 256;
	public uint MaxStyles { get; set; } = 256;
	public uint MaxResources { get; set; } = 256;
	public uint MaxGlyphCacheBytes { get; set; } = 1u << 24;
	public uint MaxGlyphCachePages { get; set; } = 16;

	/// <summary>Reserved by the native M3 renderer, which draws with root constants and a fixed vertex-less quad; currently unused.</summary>
	public uint MaxGeneratedVertices { get; set; }

	/// <summary>Reserved by the native M3 renderer, which draws with root constants and a fixed vertex-less quad; currently unused.</summary>
	public uint MaxGeneratedIndices { get; set; }

	public uint MaxQueuedEvents { get; set; } = 1024;
	public uint MaxSemanticRecords { get; set; } = 4096;
	public float MaxTransitionDurationMs { get; set; } = 2000.0f;
}

/// <summary>
/// Explicit per-control layout inputs, all in DIPs. Width/height are required and must be finite and non-negative for every
/// non-root control; layout never measures content. A Root control is the sole exception: width/height of exactly zero
/// means "size to the viewport's current DIP extent". A record: two instances with equal property values compare equal,
/// and 'with' produces an independent copy since every property here is a value type.
/// </summary>
public sealed record UiLayoutParams
{
	public float Width { get; set; }
	public float Height { get; set; }
	public float MarginLeft { get; set; }
	public float MarginTop { get; set; }
	public float MarginRight { get; set; }
	public float MarginBottom { get; set; }
	public float PaddingLeft { get; set; }
	public float PaddingTop { get; set; }
	public float PaddingRight { get; set; }
	public float PaddingBottom { get; set; }
	public EHAlign HAlign { get; set; } = EHAlign.Stretch;
	public EVAlign VAlign { get; set; } = EVAlign.Stretch;
	public float StackSpacing { get; set; }

	/// <summary>Absolute position within the parent's content rect; applied only when the parent's own LayoutMode is Canvas.</summary>
	public float CanvasX { get; set; }
	public float CanvasY { get; set; }

	/// <summary>Content shift applied only when this control's own LayoutMode is Scroll; must be finite and non-negative.</summary>
	public float ScrollOffsetX { get; set; }
	public float ScrollOffsetY { get; set; }
}

/// <summary>
/// Explicit world-anchoring inputs for a Root whose RootPolicy is world-anchored; ignored entirely for ERootPolicy.Screen
/// and for every non-Root control. A world root is always a camera-facing billboard positioned at Anchor. A record: two
/// instances with equal property values compare equal, and 'with' produces an independent copy since every property here
/// is a value type.
/// </summary>
public sealed record UiWorldRootParams
{
	/// <summary>World-space position the root's anchor point sits at.</summary>
	public Vec3 Anchor { get; set; } = Vec3.Zero;

	/// <summary>Which point of the root's DIP box coincides with Anchor, per axis.</summary>
	public EAnchorPoint AnchorH { get; set; } = EAnchorPoint.Centre;
	public EAnchorPoint AnchorV { get; set; } = EAnchorPoint.Centre;

	/// <summary>How the root's DIP box converts to apparent screen size.</summary>
	public EWorldSizing Sizing { get; set; } = EWorldSizing.WorldUnits;

	/// <summary>World units spanned by one DIP under EWorldSizing.WorldUnits; must be finite and greater than zero. Ignored under ConstantDip.</summary>
	public float WorldUnitsPerDip { get; set; } = 1.0f;

	/// <summary>World units the recorded depth is pulled toward the camera by, avoiding z-fighting for a DepthTested root sitting on a surface.</summary>
	public float DepthOffset { get; set; }

	/// <summary>Occlusion fade bounds, applied only under ERootPolicy.OcclusionFaded; see the native WorldRootParams comment for the exact ramp.</summary>
	public float OcclusionMinOpacity { get; set; } = 1.0f;
	public float OcclusionFadeDepth { get; set; } = 1.0f;
	public float OcclusionDepthBias { get; set; }
}

/// <summary>
/// One closed-vocabulary control descriptor (the 'Control' EStructId). Every property is present for every control type;
/// properties that do not apply to a given Type are ignored by native validation and layout (for example RootPolicy only
/// applies when Type == Root). This is a plain copyable snapshot: it carries no live UI-element identity, only the stable
/// Id the application assigned. A record: two instances with equal property values (including a deep, property-wise
/// comparison of Layout/World, which are themselves records) compare equal. Submitting this via UiTransactionBuilder.Upsert
/// copies every field - including Layout and World - into the queued transaction immediately, so mutating this instance (or
/// its Layout/World) afterward can never change an already-queued transaction; use DeepClone() first if you need an
/// independent copy to keep mutating for a later, separate Upsert call.
/// </summary>
public sealed record UiControlDesc
{
	/// <summary>The stable, application-assigned identity this descriptor creates or updates.</summary>
	public ControlId Id { get; set; }

	/// <summary>The parent this control is upserted under. Ignored for a Root (which has no parent).</summary>
	public ControlId ParentId { get; set; } = ControlId.None;

	public EControlType Type { get; set; }
	public ERootPolicy RootPolicy { get; set; } = ERootPolicy.Screen;
	public ELayoutMode LayoutMode { get; set; } = ELayoutMode.Overlay;
	public TemplateId TemplateId { get; set; } = TemplateId.None;
	public StyleId StyleId { get; set; } = StyleId.None;
	public bool Enabled { get; set; } = true;

	/// <summary>Controls rendering/input for this subtree and whether its layout allocation is retained or collapsed.</summary>
	public EVisibility Visibility { get; set; } = EVisibility.Visible;
	public bool Focusable { get; set; }

	/// <summary>Application-computed validation state for this control's proposed/accepted value; View3DUI only reads it.</summary>
	public EValidationState ValidationState { get; set; } = EValidationState.NotApplicable;

	public UiLayoutParams Layout { get; set; } = new();

	/// <summary>This control's own display text (Text) or generated label text (Button/TextBox); ignored by control types that never display text.</summary>
	public string Text { get; set; } = string.Empty;

	/// <summary>An automation-facing name, surfaced through UiContext.CaptureSemantics; independent of Text.</summary>
	public string Name { get; set; } = string.Empty;

	/// <summary>An automation-facing description, surfaced through UiContext.CaptureSemantics.</summary>
	public string Description { get; set; } = string.Empty;

	/// <summary>The maximum accepted length for a TextBox's proposed text; zero means the native default (unbounded).</summary>
	public uint MaxTextLength { get; set; }

	/// <summary>Optional Font resource to render this control's text with; None selects the built-in Segoe UI fallback.</summary>
	public ResourceId FontResourceId { get; set; } = ResourceId.None;

	/// <summary>Durable, application-set flag driving EStateChannel.Selected; has no other built-in behaviour (e.g. no hit-testing or focus effect).</summary>
	public bool Selected { get; set; }

	/// <summary>Application-owned monotonic counter: any change from its previously observed value fires EStateChannel.ValueChanged for one Update() call.</summary>
	public uint ValueSequence { get; set; }

	/// <summary>World anchoring for a Root whose RootPolicy is world-anchored; ignored otherwise.</summary>
	public UiWorldRootParams World { get; set; } = new();

	/// <summary>ProgressBar completion in [0, 1], finite even when IsIndeterminate is true. Ignored by other types.</summary>
	public float Value { get; set; }

	/// <summary>Show host-time-driven activity instead of a percentage. Applies only to ProgressBar.</summary>
	public bool IsIndeterminate { get; set; }

	/// <summary>Slider's finite inclusive lower bound. Ignored by other control types.</summary>
	public float Minimum { get; set; }

	/// <summary>Slider's finite inclusive upper bound. Ignored by other control types.</summary>
	public float Maximum { get; set; } = 1.0f;

	/// <summary>Slider proposal increment, anchored at Minimum; must be finite, positive, and no greater than Maximum - Minimum.</summary>
	public float Step { get; set; } = 0.1f;

	/// <summary>When true for a TextBox, View3DUI displays bullets and never exposes the value through semantics/UI Automation.</summary>
	public bool Masked { get; set; }

	/// <summary>ComboBox items in display order. Applies only to ComboBox. Assigning stores an immutable copy, so later changes to the assigned collection have no effect.</summary>
	public IReadOnlyList<string> Items
	{
		get
		{
			return m_items;
		}
		set
		{
			// Share existing snapshots because they are immutable; copy anything else
			m_items = value as ItemList ?? new ItemList(value);
		}
	}
	private ItemList m_items = ItemList.Empty;

	/// <summary>ComboBox selected item index, or -1 for no selection. Applies only to ComboBox.</summary>
	public int SelectedIndex { get; set; } = -1;

	/// <summary>Maximum visible drop-down rows for ComboBox; must be greater than zero for ComboBox.</summary>
	public uint MaxVisibleItems { get; set; }

	/// <summary>Value axis direction for Slider and ProgressBar. Vertical values increase upward. Ignored by other control types.</summary>
	public EOrientation Orientation { get; set; } = EOrientation.Horizontal;

	/// <summary>
	/// Create an independent deep copy: mutating the clone's Layout/World (or this instance's) afterward cannot affect the
	/// other, because Layout and World are themselves replaced with their own independent copies rather than shared by
	/// reference. Named DeepClone (not Clone) because C# records disallow a member literally named "Clone".
	/// </summary>
	public UiControlDesc DeepClone()
	{
		// Items is an immutable snapshot, so the clone can share it
		return this with { Layout = Layout with { }, World = World with { } };
	}

	/// <summary>An immutable list of strings that compares equal to another list with the same items in the same order.</summary>
	private sealed class ItemList : IReadOnlyList<string>, IEquatable<ItemList>
	{
		/// <summary>The shared empty list.</summary>
		public static readonly ItemList Empty = new(Array.Empty<string>());

		// Private copy, never exposed for mutation
		private readonly string[] m_items;

		/// <summary>Copy 'items' into a new immutable list.</summary>
		public ItemList(IEnumerable<string> items)
		{
			// Take a private copy so the caller cannot change this list later
			m_items = [.. items];
		}

		/// <inheritdoc/>
		public string this[int index]
		{
			get
			{
				return m_items[index];
			}
		}

		/// <inheritdoc/>
		public int Count
		{
			get
			{
				return m_items.Length;
			}
		}

		/// <inheritdoc/>
		public IEnumerator<string> GetEnumerator()
		{
			// Enumerate the private array through the generic interface
			return ((IEnumerable<string>)m_items).GetEnumerator();
		}

		/// <inheritdoc/>
		System.Collections.IEnumerator System.Collections.IEnumerable.GetEnumerator()
		{
			// Forward to the generic enumerator
			return GetEnumerator();
		}

		/// <summary>True when both lists contain the same strings in the same order.</summary>
		public bool Equals(ItemList? other)
		{
			// Compare item by item with ordinal string equality
			return other is not null && System.Linq.Enumerable.SequenceEqual(m_items, other.m_items);
		}

		/// <inheritdoc/>
		public override bool Equals(object? obj)
		{
			// Only another ItemList can compare equal
			return Equals(obj as ItemList);
		}

		/// <inheritdoc/>
		public override int GetHashCode()
		{
			// Combine every item so that equal lists give equal hash codes
			var hash = 17;
			foreach (var item in m_items)
				hash = unchecked(hash * 31 + StringComparer.Ordinal.GetHashCode(item));

			return hash;
		}

		/// <inheritdoc/>
		public override string ToString()
		{
			// Show the items so assertion failures are readable
			return $"[{string.Join(", ", m_items)}]";
		}
	}
}

/// <summary>
/// One named, closed-vocabulary resource (the 'Resource' EStructId). When Kind is Font, Name is the font family (empty
/// selects the built-in Segoe UI fallback), FontSize is its DIP size (&lt;= 0 selects a fixed default size), and Colour is
/// the text colour (fully transparent, i.e. alpha &lt;= 0, selects a fixed opaque-black fallback). When Kind is Image,
/// Texture is a View3D texture handle owned by the host, ImageWidth/ImageHeight are its pixel size (both non-zero), and Name
/// is an optional diagnostic label. A record: two instances with equal property values compare equal, and 'with' produces an
/// independent copy since every property here is a value type or an immutable string. Submitting this via
/// UiTransactionBuilder.AddResource copies every field into the queued transaction immediately, so mutating this instance
/// afterward can never change an already-queued transaction.
/// </summary>
public sealed record UiResourceDesc
{
	/// <summary>The stable, application-assigned identity this descriptor creates or updates.</summary>
	public ResourceId Id { get; set; }

	public EResourceKind Kind { get; set; }
	public Colour Colour { get; set; } = Colour.TransparentBlack;
	public string Name { get; set; } = string.Empty;
	public float FontSize { get; set; }

	/// <summary>
	/// Image resources only: the View3D texture handle (View3d.Texture.Handle). The host owns the texture and must keep it alive while
	/// the resource is registered.
	/// </summary>
	public IntPtr Texture { get; set; }

	/// <summary>Image resources only: the texture width in pixels.</summary>
	public uint ImageWidth { get; set; }

	/// <summary>Image resources only: the texture height in pixels.</summary>
	public uint ImageHeight { get; set; }

	/// <summary>
	/// Create an Image resource for 'texture', taking its handle and pixel size. The caller keeps ownership of 'texture' and must keep it
	/// alive while the resource is registered.
	/// </summary>
	public static UiResourceDesc Image(ResourceId id, View3d.Texture texture, string name = "")
	{
		// Copy the handle and size so the descriptor holds no reference to the managed texture object
		if (texture is null)
			throw new ArgumentNullException(nameof(texture));
		if (texture.Handle == IntPtr.Zero)
			throw new ArgumentException("The texture has been disposed or was never created.", nameof(texture));

		return new UiResourceDesc
		{
			Id = id,
			Kind = EResourceKind.Image,
			Name = name ?? string.Empty,
			Texture = texture.Handle,
			ImageWidth = checked((uint)texture.Info.Width),
			ImageHeight = texture.Info.Height,
		};
	}
}

/// <summary>
/// One closed-vocabulary style (the 'Style' EStructId): one StyleVisual/TransitionDesc pair per declared EStateChannel,
/// applied to a control as a whole rather than per template part. Not a record, because its per-channel state is held in
/// backing arrays rather than simple properties; IEquatable/Clone are implemented explicitly instead, comparing/copying
/// every channel's visual and transition by value. Submitting this via UiTransactionBuilder.AddStyle copies every channel
/// into the queued transaction immediately, so mutating this instance (via SetVisual/SetTransition) afterward can never
/// change an already-queued transaction; use Clone() first if you need an independent copy to keep mutating separately.
/// </summary>
public sealed class UiStyleDesc : IEquatable<UiStyleDesc>
{
	private readonly StyleVisual[] m_visuals = new StyleVisual[(int)EStateChannel.Count];
	private readonly TransitionDesc[] m_transitions = new TransitionDesc[(int)EStateChannel.Count];

	/// <summary>The stable, application-assigned identity this descriptor creates or updates.</summary>
	public StyleId Id { get; set; }

	/// <summary>The visual appearance for 'channel'.</summary>
	public StyleVisual GetVisual(EStateChannel channel)
	{
		return m_visuals[CheckChannel(channel)];
	}

	/// <summary>Set the visual appearance for 'channel'.</summary>
	public void SetVisual(EStateChannel channel, StyleVisual value)
	{
		m_visuals[CheckChannel(channel)] = value;
	}

	/// <summary>The transition played when 'channel' becomes the active state channel.</summary>
	public TransitionDesc GetTransition(EStateChannel channel)
	{
		return m_transitions[CheckChannel(channel)];
	}

	/// <summary>Set the transition played when 'channel' becomes the active state channel.</summary>
	public void SetTransition(EStateChannel channel, TransitionDesc value)
	{
		m_transitions[CheckChannel(channel)] = value;
	}

	/// <summary>A defensive-copy snapshot of every visual, indexed by EStateChannel.</summary>
	public StyleVisual[] Visuals
	{
		get
		{
			return (StyleVisual[])m_visuals.Clone();
		}
	}

	/// <summary>A defensive-copy snapshot of every transition, indexed by EStateChannel.</summary>
	public TransitionDesc[] Transitions
	{
		get
		{
			return (TransitionDesc[])m_transitions.Clone();
		}
	}

	/// <summary>Create an independent copy whose Visuals/Transitions arrays are snapshotted separately from this instance's.</summary>
	public UiStyleDesc Clone()
	{
		var clone = new UiStyleDesc { Id = Id };
		Array.Copy(m_visuals, clone.m_visuals, m_visuals.Length);
		Array.Copy(m_transitions, clone.m_transitions, m_transitions.Length);
		return clone;
	}

	/// <summary>Value-compare every channel's visual and transition, plus Id.</summary>
	public bool Equals(UiStyleDesc? other)
	{
		if (other is null)
			return false;

		if (Id != other.Id)
			return false;

		for (var i = 0; i != m_visuals.Length; ++i)
		{
			if (m_visuals[i] != other.m_visuals[i] || m_transitions[i] != other.m_transitions[i])
				return false;
		}

		return true;
	}

	/// <inheritdoc/>
	public override bool Equals(object? obj)
	{
		return Equals(obj as UiStyleDesc);
	}

	/// <inheritdoc/>
	public override int GetHashCode()
	{
		var hash = Id.GetHashCode();
		for (var i = 0; i != m_visuals.Length; ++i)
		{
			hash = HashUtil.Combine(hash, m_visuals[i], m_transitions[i]);
		}
		return hash;
	}

	/// <summary>Compare two styles by value.</summary>
	public static bool operator ==(UiStyleDesc? lhs, UiStyleDesc? rhs)
	{
		if (ReferenceEquals(lhs, rhs))
			return true;
		if (lhs is null || rhs is null)
			return false;

		return lhs.Equals(rhs);
	}

	/// <summary>Compare two styles by value.</summary>
	public static bool operator !=(UiStyleDesc? lhs, UiStyleDesc? rhs)
	{
		return !(lhs == rhs);
	}

	/// <summary>Reject a channel outside the closed EStateChannel vocabulary.</summary>
	private static int CheckChannel(EStateChannel channel)
	{
		if (channel < 0 || channel >= EStateChannel.Count)
			throw new ArgumentOutOfRangeException(nameof(channel), channel, "EStateChannel is a closed vocabulary; see EStateChannel.Count.");

		return (int)channel;
	}
}

/// <summary>
/// One role-based template part. The part places a styled box inside the control bounds, inset by DIP margins (scaled with the root).
/// Thumb parts use Width x Height as the thumb size; Ticks parts use Ticks to describe the tick band. Lengths must be finite and
/// non-negative. Which roles apply to which control types is described on ETemplatePart.
/// </summary>
public readonly struct UiTemplatePart : IEquatable<UiTemplatePart>
{
	/// <summary>The closed role this part plays.</summary>
	public ETemplatePart Role { get; }

	/// <summary>Style used to draw the part, or StyleId.None to use the control's style.</summary>
	public StyleId StyleId { get; }

	/// <summary>DIP insets from the control bounds.</summary>
	public float MarginLeft { get; }
	public float MarginTop { get; }
	public float MarginRight { get; }
	public float MarginBottom { get; }

	/// <summary>DIP size of a Thumb part. Unused by other roles.</summary>
	public float Width { get; }
	public float Height { get; }

	/// <summary>Tick band parameters for a Ticks part. Unused by other roles.</summary>
	public TickParams Ticks { get; }

	/// <summary>Create a template part.</summary>
	public UiTemplatePart(ETemplatePart role, StyleId style_id = default, float margin_left = 0, float margin_top = 0, float margin_right = 0, float margin_bottom = 0, float width = 0, float height = 0, TickParams ticks = default)
	{
		// Store the fields; validation happens natively when the transaction is applied
		Role = role;
		StyleId = style_id;
		MarginLeft = margin_left;
		MarginTop = margin_top;
		MarginRight = margin_right;
		MarginBottom = margin_bottom;
		Width = width;
		Height = height;
		Ticks = ticks;
	}

	/// <inheritdoc/>
	public bool Equals(UiTemplatePart other)
	{
		// Compare every field
		return
			Role == other.Role && StyleId == other.StyleId &&
			MarginLeft == other.MarginLeft && MarginTop == other.MarginTop && MarginRight == other.MarginRight && MarginBottom == other.MarginBottom &&
			Width == other.Width && Height == other.Height && Ticks == other.Ticks;
	}

	/// <inheritdoc/>
	public override bool Equals(object? obj)
	{
		// Only another part can be equal
		return obj is UiTemplatePart other && Equals(other);
	}

	/// <inheritdoc/>
	public override int GetHashCode()
	{
		// Hash every field
		return HashUtil.Combine(Role, StyleId, MarginLeft, MarginTop, MarginRight, MarginBottom, Width, Height, Ticks);
	}

	/// <summary>Compare two template parts.</summary>
	public static bool operator ==(UiTemplatePart lhs, UiTemplatePart rhs)
	{
		return lhs.Equals(rhs);
	}

	/// <summary>Compare two template parts.</summary>
	public static bool operator !=(UiTemplatePart lhs, UiTemplatePart rhs)
	{
		return !lhs.Equals(rhs);
	}
}

/// <summary>
/// One closed-vocabulary drawing template (the 'Template' EStructId): a bounded list of role parts for one control type.
/// Each role may appear once, except Ticks. The native ABI caps this list at VIEW3D_UI_MAX_TEMPLATE_PARTS entries. Not a
/// record, because its parts are held in a backing list rather than simple properties; IEquatable/Clone are implemented
/// explicitly instead, comparing/copying the part sequence by value. Submitting this via UiTransactionBuilder.AddTemplate
/// copies every part into the queued transaction immediately, so mutating this instance (via AddPart) afterward can never
/// change an already-queued transaction; use Clone() first if you need an independent copy to keep mutating separately.
/// </summary>
public sealed class UiTemplateDesc : IEquatable<UiTemplateDesc>
{
	/// <summary>The maximum number of parts a template may declare, mirroring the native VIEW3D_UI_MAX_TEMPLATE_PARTS constant.</summary>
	public const int MaxParts = 8;

	private readonly List<UiTemplatePart> m_parts = new();

	/// <summary>The stable, application-assigned identity this descriptor creates or updates.</summary>
	public TemplateId Id { get; set; }

	/// <summary>The control type this template may be assigned to via ControlDesc.TemplateId.</summary>
	public EControlType AppliesTo { get; set; }

	/// <summary>The template's ordered parts. Read-only: use AddPart to append, respecting MaxParts.</summary>
	public IReadOnlyList<UiTemplatePart> Parts
	{
		get
		{
			return m_parts;
		}
	}

	/// <summary>Append a role part. Throws InvalidOperationException once MaxParts parts have already been added.</summary>
	public void AddPart(UiTemplatePart part)
	{
		// Enforce the fixed native part capacity
		if (m_parts.Count >= MaxParts)
			throw new InvalidOperationException($"Template already has the maximum of {MaxParts} parts (VIEW3D_UI_MAX_TEMPLATE_PARTS).");

		m_parts.Add(part);
	}

	/// <summary>Create an independent copy whose Parts list is snapshotted separately from this instance's.</summary>
	public UiTemplateDesc Clone()
	{
		var clone = new UiTemplateDesc { Id = Id, AppliesTo = AppliesTo };
		clone.m_parts.AddRange(m_parts);
		return clone;
	}

	/// <summary>Value-compare Id, AppliesTo, and the ordered Parts sequence.</summary>
	public bool Equals(UiTemplateDesc? other)
	{
		if (other is null)
			return false;

		if (Id != other.Id || AppliesTo != other.AppliesTo || m_parts.Count != other.m_parts.Count)
			return false;

		for (var i = 0; i != m_parts.Count; ++i)
		{
			if (m_parts[i] != other.m_parts[i])
				return false;
		}

		return true;
	}

	/// <inheritdoc/>
	public override bool Equals(object? obj)
	{
		return Equals(obj as UiTemplateDesc);
	}

	/// <inheritdoc/>
	public override int GetHashCode()
	{
		var hash = HashUtil.Combine(Id, AppliesTo);
		for (var i = 0; i != m_parts.Count; ++i)
		{
			hash = HashUtil.Combine(hash, m_parts[i]);
		}
		return hash;
	}

	/// <summary>Compare two templates by value.</summary>
	public static bool operator ==(UiTemplateDesc? lhs, UiTemplateDesc? rhs)
	{
		if (ReferenceEquals(lhs, rhs))
			return true;
		if (lhs is null || rhs is null)
			return false;

		return lhs.Equals(rhs);
	}

	/// <summary>Compare two templates by value.</summary>
	public static bool operator !=(UiTemplateDesc? lhs, UiTemplateDesc? rhs)
	{
		return !(lhs == rhs);
	}
}
