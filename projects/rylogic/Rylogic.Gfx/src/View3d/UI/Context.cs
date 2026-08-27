using System;
using System.Runtime.InteropServices;
using System.Text;
using Microsoft.Win32.SafeHandles;

namespace Rylogic.Gfx.UI;

/// <summary>Owns one native View3DUI context (one owner-thread retained control tree, its input state machine, and event/semantic buffers).</summary>
public sealed unsafe class UiContext :IDisposable
{
	private readonly UiRuntime m_runtime;
	private readonly uint m_owner_thread_id;
	private ContextSafeHandle? m_handle;

	/// <summary>Adopt a newly-created native context.</summary>
	internal UiContext(UiRuntime runtime, ulong handle, uint owner_thread_id)
	{
		m_runtime = runtime;
		m_owner_thread_id = owner_thread_id;
		m_handle = new ContextSafeHandle(handle);
	}

	/// <summary>True after this context has released its native resources.</summary>
	public bool IsDisposed => m_handle == null || m_handle.IsInvalid || m_handle.IsClosed;

	/// <summary>Translate one raw Win32 window message into the context's normalized input state machine.</summary>
	public (bool Consumed, IntPtr Result, bool Invalidate) ProcessWindowMessage(IntPtr hwnd, uint msg, IntPtr wparam, IntPtr lparam)
	{
		EnsureOwner();
		Native.Check(Native.View3DUI_ProcessWindowMessage(Handle, hwnd, msg, wparam, lparam, out var consumed, out var result, out var invalidate));
		return (consumed != 0, result, invalidate != 0);
	}

	/// <summary>Inject one deterministic normalized input event, bypassing raw Win32 message translation.</summary>
	public void InjectInput(NormalizedInput input)
	{
		EnsureOwner();
		Native.Check(Native.View3DUI_InputInject(Handle, &input));
	}

	/// <summary>
	/// Inject one normalized input event that may carry variable-length text (EInputKind.TextInput, CompositionUpdate, or
	/// CompositionCommit via 'text'), or none (CompositionStart/CompositionCancel, where 'text' must be null). This is the
	/// deterministic seam that drives a complete IME composition lifecycle without an installed IME, using the same state
	/// machine the raw WM_IME_* path uses. 'caret'/'selectionStart'/'selectionEnd' are UTF-8 byte offsets within 'text'
	/// describing the IME's own cursor and target clause during a CompositionUpdate; they are ignored otherwise. Throws
	/// ArgumentException if 'text' is null for a kind that requires it, or non-null for a kind that carries no text.
	/// </summary>
	public void InjectInputText(NormalizedInput input, string? text, uint caret = 0, uint selectionStart = 0, uint selectionEnd = 0)
	{
		EnsureOwner();

		var carries_text = CarriesText(input.m_kind);
		if (carries_text && text == null)
			throw new ArgumentException($"NormalizedInput.Kind {input.m_kind} requires non-null text.", nameof(text));
		if (!carries_text && text != null)
			throw new ArgumentException($"NormalizedInput.Kind {input.m_kind} carries no text; pass null, or call InjectInput directly.", nameof(text));

		// CompositionStart/CompositionCancel and every other non-text-carrying kind must go through View3DUI_InputInject:
		// View3DUI_InputInjectText always requires a non-null InputTextPayload (native rejects a null payload outright),
		// so there is no way to express "no text" through it - the native engine pairs each kind with exactly one of the
		// two entry points, and InjectInput is the one that accepts these kinds.
		if (!carries_text)
		{
			Native.Check(Native.View3DUI_InputInject(Handle, &input));
			return;
		}

		// The UTF-8 bytes must stay pinned for the duration of the call only; the native side copies them before returning.
		var text_bytes = Encoding.UTF8.GetBytes(text!);
		fixed (byte* text_ptr = text_bytes)
		{
			var payload = new Native.InputTextPayload
			{
				m_header = NativeHeader.Create<Native.InputTextPayload>(),
				m_text_utf8 = (IntPtr)text_ptr,
				m_text_length = (uint)text_bytes.Length,
				m_caret = caret,
				m_selection_start = selectionStart,
				m_selection_end = selectionEnd,
			};
			Native.Check(Native.View3DUI_InputInjectText(Handle, &input, &payload));
		}
	}

	/// <summary>Whether 'kind' is one of the three EInputKind values that carry variable-length text via InjectInputText's 'text' parameter, mirroring the native engine's InputKindCarriesText.</summary>
	private static bool CarriesText(EInputKind kind)
	{
		switch (kind)
		{
			case EInputKind.PointerMove:
			{
				return false;
			}
			case EInputKind.PointerButtonDown:
			{
				return false;
			}
			case EInputKind.PointerButtonUp:
			{
				return false;
			}
			case EInputKind.PointerWheel:
			{
				return false;
			}
			case EInputKind.KeyDown:
			{
				return false;
			}
			case EInputKind.KeyUp:
			{
				return false;
			}
			case EInputKind.Char:
			{
				return false;
			}
			case EInputKind.FocusGained:
			{
				return false;
			}
			case EInputKind.FocusLost:
			{
				return false;
			}
			case EInputKind.TextInput:
			{
				return true;
			}
			case EInputKind.CompositionStart:
			{
				return false;
			}
			case EInputKind.CompositionUpdate:
			{
				return true;
			}
			case EInputKind.CompositionCommit:
			{
				return true;
			}
			case EInputKind.CompositionCancel:
			{
				return false;
			}
			default:
			{
				throw new ArgumentOutOfRangeException(nameof(kind), kind, "Unknown EInputKind value.");
			}
		}
	}

	/// <summary>
	/// The caret rectangle of a focused, editable, laid-out control, in DIPs within the render target, as measured from
	/// the same shaped layout the renderer draws. Returns false when 'controlId' has no valid caret, in which case
	/// 'caretDip' is Rect.Zero. Intended for candidate-window placement and a later UI Automation Text provider.
	/// </summary>
	public bool TryGetCaretGeometry(ControlId controlId, out Rect caretDip)
	{
		EnsureOwner();

		var rect = default(Rect);
		var valid = 0;
		Native.Check(Native.View3DUI_CaretGeometry(Handle, controlId, &rect, &valid));
		caretDip = rect;
		return valid != 0;
	}

	/// <summary>Advance the context by one frame: resolve style/state transitions, project world roots, and refresh the semantic snapshot.</summary>
	public void Update(ViewportState viewport)
	{
		EnsureOwner();
		Native.Check(Native.View3DUI_Update(Handle, &viewport));
	}

	/// <summary>The number of events currently queued and awaiting DrainEvents.</summary>
	public int EventCount()
	{
		EnsureOwner();
		Native.Check(Native.View3DUI_EventCount(Handle, out var count));
		return checked((int)count);
	}

	/// <summary>
	/// Drain every currently-queued event into a caller-owned buffer, decoding each event's UTF-8 payload text, and return
	/// the number of events written. Throws EStatus.BufferTooSmall if 'destination' is smaller than EventCount().
	/// </summary>
	public int DrainEvents(Span<UiEvent> destination)
	{
		EnsureOwner();

		// Probe with zero capacity first: the native call still writes the exact event count and payload byte length
		// through the output pointers even though it reports EStatus.BufferTooSmall for the zero-capacity attempt.
		var probe_status = Native.View3DUI_EventsCopy(Handle, null, 0, out var required, null, 0, out var payload_required);
		if (probe_status != EStatus.Success && probe_status != EStatus.BufferTooSmall)
			Native.Check(probe_status);
		if (required == 0)
			return 0;
		if (destination.Length < required)
			throw new View3dUiException(EStatus.BufferTooSmall, $"DrainEvents requires a buffer of at least {required} events; {destination.Length} were provided.");

		var native_events = new Native.Event[required];
		var payload = new byte[payload_required];
		fixed (Native.Event* events_ptr = native_events)
		fixed (byte* payload_ptr = payload)
			Native.Check(Native.View3DUI_EventsCopy(Handle, events_ptr, required, out required, payload_ptr, payload_required, out payload_required));

		for (var i = 0; i != native_events.Length; ++i)
		{
			var native_event = native_events[i];
			var text = DecodeText(payload, native_event.m_payload_offset, native_event.m_payload_length);
			destination[i] = new UiEvent(native_event.m_control_id, native_event.m_kind, native_event.m_accepted_revision, native_event.m_sequence, native_event.m_edit_generation, text);
		}
		return checked((int)native_events.Length);
	}

	/// <summary>Copy and drain every currently-queued event into a newly-allocated array, decoding each event's UTF-8 payload text.</summary>
	public UiEvent[] DrainEvents()
	{
		var count = EventCount();
		if (count == 0)
			return Array.Empty<UiEvent>();

		var events = new UiEvent[count];
		DrainEvents(events.AsSpan());
		return events;
	}

	/// <summary>
	/// Copy the current semantic snapshot into a caller-owned buffer, decoding each node's UTF-8 name/description/value
	/// text, and return the number of nodes written. Not drained: persists until the next Update call. Throws
	/// EStatus.BufferTooSmall if 'destination' is smaller than the current snapshot's node count.
	/// </summary>
	public int CaptureSemantics(Span<UiSemanticNode> destination)
	{
		EnsureOwner();

		// Probe with zero capacity first, exactly as DrainEvents does for its own two output buffers.
		var probe_status = Native.View3DUI_SemanticsCopy(Handle, null, 0, out var required, null, 0, out var text_required);
		if (probe_status != EStatus.Success && probe_status != EStatus.BufferTooSmall)
			Native.Check(probe_status);
		if (required == 0)
			return 0;
		if (destination.Length < required)
			throw new View3dUiException(EStatus.BufferTooSmall, $"CaptureSemantics requires a buffer of at least {required} nodes; {destination.Length} were provided.");

		var native_nodes = new Native.SemanticNode[required];
		var text_blob = new byte[text_required];
		fixed (Native.SemanticNode* nodes_ptr = native_nodes)
		fixed (byte* text_ptr = text_blob)
			Native.Check(Native.View3DUI_SemanticsCopy(Handle, nodes_ptr, required, out required, text_ptr, text_required, out text_required));

		for (var i = 0; i != native_nodes.Length; ++i)
		{
			var native_node = native_nodes[i];
			var name = DecodeText(text_blob, native_node.m_name_offset, native_node.m_name_length);
			var description = DecodeText(text_blob, native_node.m_desc_offset, native_node.m_desc_length);
			var value = DecodeText(text_blob, native_node.m_value_offset, native_node.m_value_length);
			destination[i] = new UiSemanticNode(native_node.m_id, native_node.m_parent_id, native_node.m_role, name, description, value, (ESemanticState)native_node.m_state_flags, (ESemanticAction)native_node.m_supported_actions, (ESemanticTextFlag)native_node.m_text_flags, native_node.m_caret, native_node.m_selection_start, native_node.m_selection_end, native_node.m_composition_start, native_node.m_composition_length, native_node.m_value_grapheme_count, native_node.m_bounds, native_node.m_accepted_revision, native_node.m_semantic_sequence);
		}
		return checked((int)native_nodes.Length);
	}

	/// <summary>Copy the current semantic snapshot into a newly-allocated array, decoding each node's UTF-8 name/description/value text. Not drained: persists until the next Update call.</summary>
	public UiSemanticNode[] CaptureSemantics()
	{
		EnsureOwner();

		// A dedicated count query does not exist natively for semantics, so probe once via a zero-capacity call.
		var probe_status = Native.View3DUI_SemanticsCopy(Handle, null, 0, out var required, null, 0, out _);
		if (probe_status != EStatus.Success && probe_status != EStatus.BufferTooSmall)
			Native.Check(probe_status);
		if (required == 0)
			return Array.Empty<UiSemanticNode>();

		var nodes = new UiSemanticNode[required];
		CaptureSemantics(nodes.AsSpan());
		return nodes;
	}

	/// <summary>Read a point-in-time snapshot of engine-wide bookkeeping counters and the most recent internally-observed failure.</summary>
	public UiDiagnostics GetDiagnostics()
	{
		EnsureOwner();
		var diagnostics = UiDiagnostics.ForQuery();
		Native.Check(Native.View3DUI_DiagnosticsGet(Handle, &diagnostics));
		return diagnostics;
	}

	/// <summary>Release the native context's resources and detach it from any host window it was created against.</summary>
	public void Dispose()
	{
		if (m_handle == null)
			return;

		EnsureOwner();
		Native.Check(Native.View3DUI_ContextDestroy(Handle));
		m_handle.MarkDestroyed();
		m_handle.Dispose();
		m_handle = null;
		m_runtime.Remove(this);
		GC.SuppressFinalize(this);
	}

	/// <summary>Apply one batched transaction to this context. Called by UiTransactionBuilder.Apply, which owns the pinned array/blob layout.</summary>
	internal void ApplyTransactionCore(Native.Transaction* transaction)
	{
		EnsureOwner();
		Native.Check(Native.View3DUI_TransactionApply(Handle, transaction));
	}

	/// <summary>The current stable native context identity.</summary>
	private ulong Handle
	{
		get
		{
			var handle = m_handle ?? throw new ObjectDisposedException(nameof(UiContext));
			return handle.Value;
		}
	}

	/// <summary>Throw if a mutable or lifetime operation runs on a non-owner OS thread.</summary>
	private void EnsureOwner()
	{
		if (Native.GetCurrentThreadId() != m_owner_thread_id)
			throw new InvalidOperationException("View3DUI context mutations and lifetime operations must run on the context's creating OS thread.");
		if (IsDisposed)
			throw new ObjectDisposedException(nameof(UiContext));
	}

	/// <summary>Decode a UTF-8 substring from a copied text blob, or the empty string when 'length' is zero.</summary>
	private static string DecodeText(byte[] blob, uint offset, uint length)
	{
		return length != 0 ? Encoding.UTF8.GetString(blob, checked((int)offset), checked((int)length)) : string.Empty;
	}

	/// <summary>Provides finalizer-safe cleanup without weakening public owner-thread disposal.</summary>
	private sealed class ContextSafeHandle :SafeHandle
	{
		/// <summary>Adopt a native generation-aware context handle.</summary>
		internal ContextSafeHandle(ulong handle)
			: base(IntPtr.Zero, true)
		{
			if (IntPtr.Size != sizeof(ulong))
				throw new PlatformNotSupportedException("Rylogic.Gfx.UI requires a 64-bit process.");

			SetHandle(unchecked((IntPtr)(long)handle));
		}

		/// <inheritdoc/>
		public override bool IsInvalid => handle == IntPtr.Zero;

		/// <summary>The typed unsigned context identity.</summary>
		internal ulong Value => unchecked((ulong)handle.ToInt64());

		/// <summary>Prevent finalizer cleanup after explicit owner-thread destruction succeeds.</summary>
		internal void MarkDestroyed()
		{
			SetHandleAsInvalid();
		}

		/// <inheritdoc/>
		protected override bool ReleaseHandle()
		{
			Native.View3DUI_UiContextAbandon(Value);
			return true;
		}
	}
}
