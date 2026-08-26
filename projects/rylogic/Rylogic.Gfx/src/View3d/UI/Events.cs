namespace Rylogic.Gfx.UI;

/// <summary>
/// One decoded owner-thread UI event (the 'Event' EStructId), drained via UiContext.DrainEvents. A copyable value snapshot
/// identified by the stable ControlId the application assigned; it carries no live UI-element identity.
/// </summary>
public sealed class UiEvent
{
	/// <summary>The control this event concerns.</summary>
	public ControlId ControlId { get; }

	/// <summary>The closed event vocabulary entry this event represents.</summary>
	public EEventKind Kind { get; }

	/// <summary>The transaction revision accepted at the time this event was raised.</summary>
	public ulong AcceptedRevision { get; }

	/// <summary>A monotonic, per-context event ordering key; strictly increasing across every event this context has ever raised.</summary>
	public ulong Sequence { get; }

	/// <summary>The control's edit generation at the time this event was raised, distinguishing edits to a reused ControlId across a Remove/Upsert pair.</summary>
	public uint EditGeneration { get; }

	/// <summary>
	/// The event's decoded UTF-8 payload text (for example TextChangeProposed's proposed text); empty for an event kind that
	/// carries no payload.
	/// </summary>
	public string Payload { get; }

	/// <summary>Adopt one decoded event snapshot.</summary>
	internal UiEvent(ControlId control_id, EEventKind kind, ulong accepted_revision, ulong sequence, uint edit_generation, string payload)
	{
		ControlId = control_id;
		Kind = kind;
		AcceptedRevision = accepted_revision;
		Sequence = sequence;
		EditGeneration = edit_generation;
		Payload = payload;
	}
}
