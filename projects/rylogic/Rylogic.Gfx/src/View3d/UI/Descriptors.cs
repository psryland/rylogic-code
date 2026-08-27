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
	public bool Visible { get; set; } = true;
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

	/// <summary>
	/// Create an independent deep copy: mutating the clone's Layout/World (or this instance's) afterward cannot affect the
	/// other, because Layout and World are themselves replaced with their own independent copies rather than shared by
	/// reference. Named DeepClone (not Clone) because C# records disallow a member literally named "Clone".
	/// </summary>
	public UiControlDesc DeepClone()
	{
		return this with { Layout = Layout with { }, World = World with { } };
	}
}

/// <summary>
/// One named, closed-vocabulary resource (the 'Resource' EStructId). When Kind is Font, Name is the font family (empty
/// selects the built-in Segoe UI fallback), FontSize is its DIP size (&lt;= 0 selects a fixed default size), and Colour is
/// the text colour (fully transparent, i.e. alpha &lt;= 0, selects a fixed opaque-black fallback). A record: two instances
/// with equal property values compare equal, and 'with' produces an independent copy since every property here is a value
/// type or an immutable string. Submitting this via UiTransactionBuilder.AddResource copies every field into the queued
/// transaction immediately, so mutating this instance afterward can never change an already-queued transaction.
/// </summary>
public sealed record UiResourceDesc
{
	/// <summary>The stable, application-assigned identity this descriptor creates or updates.</summary>
	public ResourceId Id { get; set; }

	public EResourceKind Kind { get; set; }
	public Colour Colour { get; set; } = Colour.TransparentBlack;
	public string Name { get; set; } = string.Empty;
	public float FontSize { get; set; }
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

/// <summary>One named template part. 'Name' names a semantic part such as "PART_ContentPresenter"; behaviour targets these names, not hard-coded geometry.</summary>
public readonly struct UiTemplatePart : IEquatable<UiTemplatePart>
{
	public string Name { get; }
	public EVisualPrimitive Primitive { get; }
	public bool Required { get; }

	/// <summary>Create a named template part.</summary>
	public UiTemplatePart(string name, EVisualPrimitive primitive, bool required = true)
	{
		Name = name ?? throw new ArgumentNullException(nameof(name));
		Primitive = primitive;
		Required = required;
	}

	/// <inheritdoc/>
	public bool Equals(UiTemplatePart other)
	{
		return Name == other.Name && Primitive == other.Primitive && Required == other.Required;
	}

	/// <inheritdoc/>
	public override bool Equals(object? obj)
	{
		return obj is UiTemplatePart other && Equals(other);
	}

	/// <inheritdoc/>
	public override int GetHashCode()
	{
		return HashUtil.Combine(Name, Primitive, Required);
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
/// One closed-vocabulary lookless template (the 'Template' EStructId): a bounded, ordered list of named visual parts a
/// control's applied StyleDesc can target. The native ABI caps this list at VIEW3D_UI_MAX_TEMPLATE_PARTS entries. Not a
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

	/// <summary>Append a named part. Throws InvalidOperationException once MaxParts parts have already been added.</summary>
	public void AddPart(string name, EVisualPrimitive primitive, bool required = true)
	{
		if (m_parts.Count >= MaxParts)
			throw new InvalidOperationException($"Template already has the maximum of {MaxParts} parts (VIEW3D_UI_MAX_TEMPLATE_PARTS).");

		m_parts.Add(new UiTemplatePart(name, primitive, required));
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
