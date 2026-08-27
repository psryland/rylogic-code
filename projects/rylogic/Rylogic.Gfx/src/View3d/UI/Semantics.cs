namespace Rylogic.Gfx.UI;

/// <summary>
/// One decoded automation-facing semantic node (the 'SemanticNode' EStructId), captured via UiContext.CaptureSemantics. This
/// snapshot persists until the next UiContext.Update call; it is not drained like UiEvent and carries no live UI-element identity.
/// </summary>
public sealed class UiSemanticNode
{
	/// <summary>The stable identity of the control this node describes.</summary>
	public ControlId Id { get; }

	/// <summary>The parent control's identity, or ControlId.None for a root.</summary>
	public ControlId ParentId { get; }

	/// <summary>The control's type, exposed here as an automation role.</summary>
	public EControlType Role { get; }

	/// <summary>The control's automation-facing name (ControlDesc.Name).</summary>
	public string Name { get; }

	/// <summary>The control's automation-facing description (ControlDesc.Description).</summary>
	public string Description { get; }

	/// <summary>The control's current automation-facing value text (for example a TextBox's accepted text).</summary>
	public string Value { get; }

	/// <summary>The control's current boolean automation state flags.</summary>
	public ESemanticState State { get; }

	/// <summary>The UI-Automation-style actions currently valid to perform on this control.</summary>
	public ESemanticAction SupportedActions { get; }

	/// <summary>Which of Caret/SelectionStart/SelectionEnd/CompositionStart/CompositionLength currently carry meaning.</summary>
	public ESemanticTextFlag TextFlags { get; }

	/// <summary>The live caret position (a UTF-8 byte offset within Value), meaningful only when TextFlags includes HasCaret.</summary>
	public uint Caret { get; }

	/// <summary>The start of a non-empty selection within Value, meaningful only when TextFlags includes HasSelection.</summary>
	public uint SelectionStart { get; }

	/// <summary>The end of a non-empty selection within Value, meaningful only when TextFlags includes HasSelection.</summary>
	public uint SelectionEnd { get; }

	/// <summary>The start of the live IME composition range within Value, meaningful only when TextFlags includes Composing.</summary>
	public uint CompositionStart { get; }

	/// <summary>The length of the live IME composition range within Value, meaningful only when TextFlags includes Composing.</summary>
	public uint CompositionLength { get; }

	/// <summary>The number of Unicode text units (extended grapheme clusters) in Value, matching ControlDesc's MaxTextLength convention.</summary>
	public uint ValueGraphemeCount { get; }

	/// <summary>The control's current screen-space bounds in DIPs.</summary>
	public Rect Bounds { get; }

	/// <summary>The transaction revision accepted at the time this snapshot was captured.</summary>
	public ulong AcceptedRevision { get; }

	/// <summary>A monotonic, per-control counter that changes whenever this node's semantic fields change.</summary>
	public ulong SemanticSequence { get; }

	/// <summary>Adopt one decoded semantic node snapshot.</summary>
	internal UiSemanticNode(ControlId id, ControlId parent_id, EControlType role, string name, string description, string value, ESemanticState state, ESemanticAction supported_actions, ESemanticTextFlag text_flags, uint caret, uint selection_start, uint selection_end, uint composition_start, uint composition_length, uint value_grapheme_count, Rect bounds, ulong accepted_revision, ulong semantic_sequence)
	{
		Id = id;
		ParentId = parent_id;
		Role = role;
		Name = name;
		Description = description;
		Value = value;
		State = state;
		SupportedActions = supported_actions;
		TextFlags = text_flags;
		Caret = caret;
		SelectionStart = selection_start;
		SelectionEnd = selection_end;
		CompositionStart = composition_start;
		CompositionLength = composition_length;
		ValueGraphemeCount = value_grapheme_count;
		Bounds = bounds;
		AcceptedRevision = accepted_revision;
		SemanticSequence = semantic_sequence;
	}
}
