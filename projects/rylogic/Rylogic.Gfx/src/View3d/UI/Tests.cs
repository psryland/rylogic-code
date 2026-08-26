#if PR_UNITTESTS
using System;
using System.IO;
using System.Reflection;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading;
using Rylogic.D3D12;
using Rylogic.Gfx;
using Rylogic.Gfx.UI.Json;
using Rylogic.Interop.Win32;
using Rylogic.UnitTests;
using Rylogic.Utility;

namespace Rylogic.Gfx.UI;

/// <summary>
/// Lazily acquires a live D3D12 device lease (via View3d) for the small subset of tests below that need a real native
/// View3DUI context. Mirrors LDraw.Builder's View3dValidator: initialisation is attempted at most once per process, and
/// any failure - most commonly no GPU being available in the current environment - degrades to "unavailable" so those
/// tests skip cleanly rather than failing the whole run.
/// </summary>
internal static class UiTestDevice
{
	private static bool s_attempted;
	private static View3d? s_view3d;
	private static DeviceLease? s_lease;

	/// <summary>A shared device lease to pass to UiRuntime.CreateContext, or null if no live D3D12 device is available here.</summary>
	public static DeviceLease? Lease
	{
		get
		{
			if (!s_attempted)
			{
				s_attempted = true;
				s_lease = TryAcquire();
			}
			return s_lease;
		}
	}

	/// <summary>Locate the repo root, load view3d-12.dll and its dxcompiler.dll dependency, and acquire a device lease. Returns null on any failure.</summary>
	private static DeviceLease? TryAcquire()
	{
		try
		{
			// Walk up from the test assembly's directory to the repo root, identified by Directory.Build.props.
			var dir = Path.GetDirectoryName(typeof(UiTestDevice).Assembly.Location);
			for (; dir != null; dir = Path.GetDirectoryName(dir))
			{
				if (File.Exists(Path.Combine(dir, "Directory.Build.props")))
					break;
			}
			if (dir == null)
				return null;

			// view3d-12.dll depends on dxcompiler.dll, which lives alongside its own build output rather than under /lib.
			var platform = Environment.Is64BitProcess ? "x64" : "x86";
			var config = Util.IsDebug ? "Debug" : "Release";
			var dxc_dir = Path.Combine(dir, "obj", "view3d-12", platform, config, "lib", platform);
			if (Directory.Exists(dxc_dir))
				Kernel32.AddDllDirectory(dxc_dir);

			if (!View3d.LoadDll(dir + @"\lib\$(platform)\$(config)", throw_if_missing: false))
				return null;

			s_view3d = View3d.Create();
			return s_view3d.AcquireDeviceLease();
		}
		catch
		{
			return null;
		}
	}
}

/// <summary>Validates the managed/native ABI mirror and the native status-to-exception mapping; neither needs a live device.</summary>
[TestFixture]
public sealed class TestUiAbi
{
	/// <summary>Every wire structure discoverable through View3DUI_StructSize must match its managed mirror's size, and the two reported API versions must agree.</summary>
	[Test]
	public void Layout()
	{
		Native.EnsureLoaded();
		Assert.Equal(Native.ApiVersion, Native.View3DUI_ApiVersion());
		AssertNativeSize(EStructId.Config, Marshal.SizeOf<Native.Config>());
		AssertNativeSize(EStructId.Transaction, Marshal.SizeOf<Native.Transaction>());
		AssertNativeSize(EStructId.Operation, Marshal.SizeOf<Native.Operation>());
		AssertNativeSize(EStructId.Control, Marshal.SizeOf<Native.ControlDesc>());
		AssertNativeSize(EStructId.Resource, Marshal.SizeOf<Native.ResourceDesc>());
		AssertNativeSize(EStructId.Style, Marshal.SizeOf<Native.StyleDesc>());
		AssertNativeSize(EStructId.Template, Marshal.SizeOf<Native.TemplateDesc>());
		AssertNativeSize(EStructId.NormalizedInput, Marshal.SizeOf<NormalizedInput>());
		AssertNativeSize(EStructId.ViewportState, Marshal.SizeOf<ViewportState>());
		AssertNativeSize(EStructId.Event, Marshal.SizeOf<Native.Event>());
		AssertNativeSize(EStructId.SemanticNode, Marshal.SizeOf<Native.SemanticNode>());
		AssertNativeSize(EStructId.Diagnostics, Marshal.SizeOf<UiDiagnostics>());
		AssertNativeSize(EStructId.HostBridgeVersion, Marshal.SizeOf<Native.HostBridgeVersion>());
		AssertNativeSize(EStructId.HostPassContext, Marshal.SizeOf<Native.HostPassContext>());
		AssertNativeSize(EStructId.InputTextPayload, Marshal.SizeOf<Native.InputTextPayload>());

		// Constructing a runtime independently re-verifies the same complete EStructId set through its own VerifyStructLayout pass.
		using var runtime = new UiRuntime();
	}

	/// <summary>A non-Success native status must surface from Native.Check as a View3dUiException carrying that same status.</summary>
	[Test]
	public void StatusMapsToException()
	{
		Native.EnsureLoaded();
		ExpectStatus(EStatus.AbiMismatch, () => Native.Check(EStatus.AbiMismatch));
		ExpectStatus(EStatus.InvalidArgument, () => Native.Check(EStatus.InvalidArgument));
	}

	/// <summary>Compare one managed wire-structure mirror's size against native ABI discovery for 'struct_id'.</summary>
	private static void AssertNativeSize(EStructId struct_id, int managed_size)
	{
		Native.Check(Native.View3DUI_StructSize(struct_id, out var native_size));
		Assert.Equal((uint)managed_size, native_size);
	}

	/// <summary>Require 'action' to throw a View3dUiException whose Status equals 'expected'.</summary>
	private static void ExpectStatus(EStatus expected, Action action)
	{
		try
		{
			action();
			throw new UnitTestException($"Expected native status {expected}.");
		}
		catch (View3dUiException ex)
		{
			Assert.Equal(expected, ex.Status);
		}
	}
}

/// <summary>Validates value semantics for stable ids, value descriptors, and closed-vocabulary bounds checks; none of this requires a native call.</summary>
[TestFixture]
public sealed class TestUiValueSemantics
{
	/// <summary>Stable ids compare by value, expose a "None" sentinel, and are otherwise process-opaque.</summary>
	[Test]
	public void StableIds()
	{
		Assert.True(ControlId.None.IsNone);
		Assert.False(new ControlId(1).IsNone);
		Assert.Equal(new ControlId(7), new ControlId(7));
		Assert.NotEqual(new ControlId(7), new ControlId(8));
		Assert.True(new ControlId(7) == new ControlId(7));
		Assert.True(new ControlId(7) != new ControlId(8));

		Assert.True(ResourceId.None.IsNone);
		Assert.True(StyleId.None.IsNone);
		Assert.True(TemplateId.None.IsNone);
		Assert.Equal(new ResourceId(3), new ResourceId(3));
		Assert.Equal(new StyleId(4), new StyleId(4));
		Assert.Equal(new TemplateId(5), new TemplateId(5));
	}

	/// <summary>Value structs (Vec3, Colour, StyleVisual, TransitionDesc) compare structurally, including their nested fields.</summary>
	[Test]
	public void ValueStructs()
	{
		Assert.Equal(new Vec3(1, 2, 3), new Vec3(1, 2, 3));
		Assert.NotEqual(new Vec3(1, 2, 3), new Vec3(1, 2, 4));
		Assert.Equal(Vec3.Zero, new Vec3(0, 0, 0));

		Assert.Equal(new Colour(1, 0, 0, 1), new Colour(1, 0, 0, 1));
		Assert.NotEqual(new Colour(1, 0, 0, 1), new Colour(0, 1, 0, 1));

		var visual_a = new StyleVisual(new Colour(1, 0, 0, 1), new Colour(0, 0, 0, 1), border_thickness: 2, corner_radius: 4, opacity: 0.5f);
		var visual_b = new StyleVisual(new Colour(1, 0, 0, 1), new Colour(0, 0, 0, 1), border_thickness: 2, corner_radius: 4, opacity: 0.5f);
		Assert.Equal(visual_a, visual_b);
		Assert.NotEqual(visual_a, new StyleVisual(new Colour(1, 0, 0, 1)));

		Assert.Equal(TransitionDesc.None, new TransitionDesc(0, EEasing.Linear));
		Assert.NotEqual(new TransitionDesc(100, EEasing.Linear), new TransitionDesc(100, EEasing.EaseInOut));
	}

	/// <summary>UiStyleDesc rejects a channel outside the closed EStateChannel vocabulary; a valid channel round-trips its visual/transition.</summary>
	[Test]
	public void StyleChannelBounds()
	{
		var style = new UiStyleDesc { Id = new StyleId(1) };
		Assert.Throws<ArgumentOutOfRangeException>(() => style.GetVisual((EStateChannel)(-1)));
		Assert.Throws<ArgumentOutOfRangeException>(() => style.GetVisual(EStateChannel.Count));
		Assert.Throws<ArgumentOutOfRangeException>(() => style.SetVisual(EStateChannel.Count, default));

		var visual = new StyleVisual(new Colour(0.2f, 0.4f, 0.6f, 1f));
		style.SetVisual(EStateChannel.Hover, visual);
		Assert.Equal(visual, style.GetVisual(EStateChannel.Hover));

		var transition = new TransitionDesc(250, EEasing.EaseInOut);
		style.SetTransition(EStateChannel.Hover, transition);
		Assert.Equal(transition, style.GetTransition(EStateChannel.Hover));
	}

	/// <summary>UiTemplateDesc.AddPart throws once MaxParts parts have already been added, matching the native VIEW3D_UI_MAX_TEMPLATE_PARTS cap.</summary>
	[Test]
	public void TemplatePartCap()
	{
		var template = new UiTemplateDesc { Id = new TemplateId(1), AppliesTo = EControlType.Button };
		for (var i = 0; i != UiTemplateDesc.MaxParts; ++i)
			template.AddPart($"Part{i}", EVisualPrimitive.SolidBox);

		Assert.Equal(UiTemplateDesc.MaxParts, template.Parts.Count);
		Assert.Throws<InvalidOperationException>(() => template.AddPart("Overflow", EVisualPrimitive.SolidBox));
	}
}

/// <summary>
/// Proves that UiTransactionBuilder.Upsert/AddResource/AddStyle/AddTemplate snapshot every field of the descriptor they
/// are given immediately, so mutating the caller's descriptor instance afterward can never change what has already been
/// queued into the builder. None of this requires a native call.
/// </summary>
[TestFixture]
public sealed class TestUiDescriptorSnapshotSemantics
{
	/// <summary>Mutating a UiControlDesc (including its nested Layout/World) after Upsert must not change the already-queued native snapshot.</summary>
	[Test]
	public void ControlMutationAfterUpsertDoesNotAffectQueuedTransaction()
	{
		var control = new UiControlDesc
		{
			Id = new ControlId(1),
			Type = EControlType.Button,
			Text = "Original",
			Enabled = true,
			Layout = new UiLayoutParams { Width = 10, Height = 20 },
			World = new UiWorldRootParams { Anchor = new Vec3(1, 2, 3) },
		};

		var builder = new UiTransactionBuilder();
		builder.Upsert(control);

		// Mutate every field the snapshot copied, including through the nested Layout/World records.
		control.Text = "Mutated";
		control.Enabled = false;
		control.Layout.Width = 999;
		control.World.Anchor = new Vec3(9, 9, 9);

		var snapshot = builder.DebugControls[0];
		Assert.Equal(1, snapshot.m_enabled);
		Assert.Equal(10f, snapshot.m_layout.m_width);
		Assert.Equal(new Vec3(1, 2, 3), snapshot.m_world.m_anchor);
		Assert.Equal("Original", builder.DebugDecodeBlobText(snapshot.m_text_offset, snapshot.m_text_length));
	}

	/// <summary>Mutating a UiResourceDesc after AddResource must not change the already-queued native snapshot.</summary>
	[Test]
	public void ResourceMutationAfterAddDoesNotAffectQueuedTransaction()
	{
		var resource = new UiResourceDesc { Id = new ResourceId(1), Kind = EResourceKind.Font, Name = "Original", FontSize = 12 };

		var builder = new UiTransactionBuilder();
		builder.AddResource(resource);

		resource.Name = "Mutated";
		resource.FontSize = 999;

		var snapshot = builder.DebugResources[0];
		Assert.Equal(12f, snapshot.m_font_size);
		Assert.Equal("Original", builder.DebugDecodeBlobText(snapshot.m_name_offset, snapshot.m_name_length));
	}

	/// <summary>Mutating a UiStyleDesc (via SetVisual/SetTransition) after AddStyle must not change the already-queued native snapshot.</summary>
	[Test]
	public void StyleMutationAfterAddDoesNotAffectQueuedTransaction()
	{
		var style = new UiStyleDesc { Id = new StyleId(1) };
		style.SetVisual(EStateChannel.Normal, new StyleVisual(new Colour(1, 0, 0, 1)));

		var builder = new UiTransactionBuilder();
		builder.AddStyle(style);

		style.SetVisual(EStateChannel.Normal, new StyleVisual(new Colour(0, 1, 0, 1)));

		var snapshot = builder.DebugStyles[0];
		Assert.Equal(new Colour(1, 0, 0, 1), snapshot.GetVisual((int)EStateChannel.Normal).m_fill);
	}

	/// <summary>Appending a new part to a UiTemplateDesc after AddTemplate must not change the already-queued native snapshot's part count.</summary>
	[Test]
	public void TemplateMutationAfterAddDoesNotAffectQueuedTransaction()
	{
		var template = new UiTemplateDesc { Id = new TemplateId(1), AppliesTo = EControlType.Button };
		template.AddPart("PART_Background", EVisualPrimitive.SolidBox);

		var builder = new UiTransactionBuilder();
		builder.AddTemplate(template);

		template.AddPart("PART_Extra", EVisualPrimitive.SolidBox);

		var snapshot = builder.DebugTemplates[0];
		Assert.Equal(1u, snapshot.m_part_count);
	}
}

/// <summary>Validates UiEvent/UiSemanticNode as plain decoded data records, independent of any native decode.</summary>
[TestFixture]
public sealed class TestUiEventsAndSemantics
{
	/// <summary>A UiEvent exposes exactly the fields the native Event ABI carries, unchanged.</summary>
	[Test]
	public void EventRoundTrips()
	{
		var evt = new UiEvent(new ControlId(11), EEventKind.CommandInvoked, accepted_revision: 3, sequence: 42, edit_generation: 2, payload: "clicked");
		Assert.Equal(new ControlId(11), evt.ControlId);
		Assert.Equal(EEventKind.CommandInvoked, evt.Kind);
		Assert.Equal(3UL, evt.AcceptedRevision);
		Assert.Equal(42UL, evt.Sequence);
		Assert.Equal(2U, evt.EditGeneration);
		Assert.Equal("clicked", evt.Payload);
	}

	/// <summary>A UiSemanticNode exposes exactly the fields the native SemanticNode ABI carries, unchanged.</summary>
	[Test]
	public void SemanticNodeRoundTrips()
	{
		var bounds = new Rect(1, 2, 100, 30);
		var text_flags = ESemanticTextFlag.HasCaret | ESemanticTextFlag.HasSelection | ESemanticTextFlag.Composing;
		var node = new UiSemanticNode(new ControlId(2), new ControlId(1), EControlType.Button, "OkButton", "Confirms the dialog", "OK",
			ESemanticState.Enabled | ESemanticState.Visible | ESemanticState.Focusable, ESemanticAction.Invoke | ESemanticAction.Focus | ESemanticAction.SetSelection,
			text_flags, caret: 2, selection_start: 0, selection_end: 2, composition_start: 1, composition_length: 1, value_grapheme_count: 2,
			bounds, accepted_revision: 5, semantic_sequence: 9);

		Assert.Equal(new ControlId(2), node.Id);
		Assert.Equal(new ControlId(1), node.ParentId);
		Assert.Equal(EControlType.Button, node.Role);
		Assert.Equal("OkButton", node.Name);
		Assert.Equal("Confirms the dialog", node.Description);
		Assert.Equal("OK", node.Value);
		Assert.Equal(ESemanticState.Enabled | ESemanticState.Visible | ESemanticState.Focusable, node.State);
		Assert.Equal(ESemanticAction.Invoke | ESemanticAction.Focus | ESemanticAction.SetSelection, node.SupportedActions);
		Assert.Equal(text_flags, node.TextFlags);
		Assert.Equal(2U, node.Caret);
		Assert.Equal(0U, node.SelectionStart);
		Assert.Equal(2U, node.SelectionEnd);
		Assert.Equal(1U, node.CompositionStart);
		Assert.Equal(1U, node.CompositionLength);
		Assert.Equal(2U, node.ValueGraphemeCount);
		Assert.Equal(bounds, node.Bounds);
		Assert.Equal(5UL, node.AcceptedRevision);
		Assert.Equal(9UL, node.SemanticSequence);
	}
}

/// <summary>Validates owner-thread affinity for UiRuntime; none of this requires a live device since EnsureOwner runs before any device access.</summary>
[TestFixture]
public sealed class TestUiLifecycle
{
	/// <summary>CreateContext and Dispose must reject a call from any OS thread other than the one that constructed the runtime.</summary>
	[Test]
	public void RuntimeThreadAffinity()
	{
		using var runtime = new UiRuntime();

		Exception? create_error = null;
		var create_thread = new Thread(() =>
		{
			try { runtime.CreateContext(); }
			catch (Exception ex) { create_error = ex; }
		});
		create_thread.Start();
		create_thread.Join();
		Assert.Equal(typeof(InvalidOperationException), create_error?.GetType());

		Exception? dispose_error = null;
		var dispose_thread = new Thread(() =>
		{
			try { runtime.Dispose(); }
			catch (Exception ex) { dispose_error = ex; }
		});
		dispose_thread.Start();
		dispose_thread.Join();
		Assert.Equal(typeof(InvalidOperationException), dispose_error?.GetType());
	}

	/// <summary>
	/// A forgotten (never-Disposed) UiRuntime's SafeHandle finalizer must not leak the process-level native runtime
	/// token even when View3DUI_Shutdown fails. Invoke the private RuntimeHandle.ReleaseHandle override directly via
	/// reflection - simulating exactly what the GC finalizer thread does - while a context is still alive, so
	/// View3DUI_Shutdown is guaranteed to report ResourceInUse and ReleaseHandle's View3DUI_ContextAbandon fallback
	/// must run instead.
	/// </summary>
	[Test]
	public void ForgottenRuntimeFallsBackToContextAbandon()
	{
		var lease = UiTestDevice.Lease;
		if (lease == null)
			return; // No live D3D12 device is available in this environment; skip - View3DUI_ContextCreate requires one.

		var runtime = new UiRuntime();
		var context = runtime.CreateContext(device: lease);
		Assert.False(context.IsDisposed);

		var runtime_field = typeof(UiRuntime).GetField("m_runtime", BindingFlags.NonPublic | BindingFlags.Instance);
		var runtime_handle = runtime_field?.GetValue(runtime) as SafeHandle;
		Assert.True(runtime_handle != null);

		var release_method = runtime_handle!.GetType().GetMethod("ReleaseHandle", BindingFlags.NonPublic | BindingFlags.Instance);
		Assert.True(release_method != null);

		var released = (bool)release_method!.Invoke(runtime_handle, null)!;
		Assert.True(released);

		// If the fallback had not run, the process-level singleton would still hold this forgotten runtime's live
		// context, and a fresh independent runtime/context pair would either fail or silently reuse leaked state.
		using var fresh_runtime = new UiRuntime();
		using var fresh_context = fresh_runtime.CreateContext(device: lease);
		Assert.False(fresh_context.IsDisposed);
	}
}

/// <summary>
/// Exercises a full UiContext lifecycle against a live native context: transaction construction/application, revision
/// rejection, viewport update, event drain, semantic capture, and disposal. Each test skips cleanly (returns without
/// asserting anything) when no live D3D12 device is available via UiTestDevice.Lease in the current environment.
/// </summary>
[TestFixture]
public sealed class TestUiContext
{
	/// <summary>Apply a hand-built transaction, then exercise revision rejection, viewport update, event drain, semantic capture, and disposal.</summary>
	[Test]
	public void CreateApplyDrainDispose()
	{
		var lease = UiTestDevice.Lease;
		if (lease == null)
			return; // No live D3D12 device is available in this environment; skip.

		using var runtime = new UiRuntime();
		var context = runtime.CreateContext(device: lease);

		var root = new UiControlDesc { Id = new ControlId(1), Type = EControlType.Root };
		var button = new UiControlDesc
		{
			Id = new ControlId(2),
			ParentId = new ControlId(1),
			Type = EControlType.Button,
			Text = "OK",
			Name = "OkButton",
			Focusable = true,
			Layout = new UiLayoutParams { Width = 100, Height = 30 },
		};

		var builder = new UiTransactionBuilder();
		builder.Upsert(root).Upsert(button);
		builder.Apply(context, base_revision: 0, revision: 1);

		var diagnostics = context.GetDiagnostics();
		Assert.Equal(2U, diagnostics.m_control_count);
		Assert.Equal(1UL, diagnostics.m_accepted_revision);

		// Re-applying the same builder with a now-stale base_revision must be rejected atomically, leaving the accepted revision unchanged.
		ExpectStatus(EStatus.StaleRevision, () => builder.Apply(context, base_revision: 0, revision: 1));

		// A requested revision that is not exactly base_revision + 1 is rejected as malformed, independent of staleness.
		ExpectStatus(EStatus.InvalidArgument, () => builder.Apply(context, base_revision: 1, revision: 5));

		var diagnostics_after_rejections = context.GetDiagnostics();
		Assert.Equal(1UL, diagnostics_after_rejections.m_accepted_revision);
		Assert.True(diagnostics_after_rejections.m_rejected_revision_attempts >= 2);

		context.Update(new ViewportState(800, 600, 800, 600, 0, 0, 800, 600, 96f, 0.0));

		var semantics = context.CaptureSemantics();
		Assert.Equal(2, semantics.Length);
		var button_semantics = Array.Find(semantics, n => n.Id == new ControlId(2));
		Assert.NotNull(button_semantics);
		Assert.Equal("OkButton", button_semantics!.Name);
		Assert.Equal(EControlType.Button, button_semantics.Role);
		Assert.Equal(new ControlId(1), button_semantics.ParentId);

		// No input was ever injected, so a deterministic frame must raise no events.
		Assert.Equal(0, context.DrainEvents().Length);

		context.Dispose();
		Assert.True(context.IsDisposed);
		Assert.Throws<ObjectDisposedException>(() => context.GetDiagnostics());
	}

	/// <summary>A transaction built from a parsed UiDocument must be accepted identically to an equivalent hand-built transaction.</summary>
	[Test]
	public void JsonTransactionAppliesIdentically()
	{
		var lease = UiTestDevice.Lease;
		if (lease == null)
			return; // No live D3D12 device is available in this environment; skip.

		const string json = @"{
			""schema_version"": 1,
			""tree"": [
				{ ""id"": 1, ""type"": ""Root"", ""children"": [
					{ ""id"": 2, ""type"": ""Button"", ""text"": ""OK"", ""name"": ""OkButton"", ""focusable"": true,
					  ""layout"": { ""width"": 100, ""height"": 30 } }
				] }
			]
		}";

		using var runtime = new UiRuntime();
		var context = runtime.CreateContext(device: lease);
		try
		{
			UiDocument.Parse(json).ToTransactionBuilder().Apply(context, base_revision: 0, revision: 1);

			var diagnostics = context.GetDiagnostics();
			Assert.Equal(2U, diagnostics.m_control_count);
			Assert.Equal(1UL, diagnostics.m_accepted_revision);

			// The semantic snapshot only reflects the tree as of the most recent Update() call.
			context.Update(new ViewportState(800, 600, 800, 600, 0, 0, 800, 600, 96f, 0.0));
			var semantics = context.CaptureSemantics();
			Assert.Equal(2, semantics.Length);
		}
		finally
		{
			context.Dispose();
		}
	}

	/// <summary>
	/// Drives a full IME composition lifecycle (CompositionStart -> CompositionUpdate -> CompositionCommit) against a live
	/// focused TextBox, proving both the mid-composition semantic snapshot and the post-commit event exactly match the
	/// native engine's documented behaviour (see input.cpp ProcessNormalizedInput/ApplyTextRanges).
	/// </summary>
	[Test]
	public void CompositionLifecycleUpdatesSemanticsAndProducesTextChangeProposed()
	{
		var lease = UiTestDevice.Lease;
		if (lease == null)
			return; // No live D3D12 device is available in this environment; skip.

		using var runtime = new UiRuntime();
		var context = runtime.CreateContext(device: lease);
		try
		{
			var (viewport, _) = SetUpFocusedTextBox(context);

			// CompositionStart carries no text; CompositionUpdate carries the live, uncommitted composition string.
			context.InjectInputText(NormalizedInput.CompositionStart(time_ms: 1), null);
			context.InjectInputText(NormalizedInput.CompositionUpdate(time_ms: 2), "ab", caret: 2, selectionStart: 0, selectionEnd: 2);

			// While composing, the live text is only ever visible through the semantic snapshot, never through an event.
			context.Update(viewport);
			var mid_composition = Array.Find(context.CaptureSemantics(), n => n.Id == new ControlId(2));
			Assert.NotNull(mid_composition);
			Assert.True((mid_composition!.TextFlags & ESemanticTextFlag.Composing) != 0);
			Assert.Equal(0U, mid_composition.CompositionStart);
			Assert.Equal(2U, mid_composition.CompositionLength);

			context.InjectInputText(NormalizedInput.CompositionCommit(time_ms: 3), "AB");

			// The completed focusing click was drained by the setup helper, so only the committed proposal remains.
			var events = context.DrainEvents();
			Assert.Equal(1, events.Length);
			Assert.Equal(EEventKind.TextChangeProposed, events[0].Kind);
			Assert.Equal(new ControlId(2), events[0].ControlId);
			Assert.Equal("AB", events[0].Payload);

			// The composition is over: a fresh snapshot must show no residual composition state.
			context.Update(viewport);
			var after_commit = Array.Find(context.CaptureSemantics(), n => n.Id == new ControlId(2));
			Assert.NotNull(after_commit);
			Assert.True((after_commit!.TextFlags & ESemanticTextFlag.Composing) == 0);
		}
		finally
		{
			context.Dispose();
		}
	}

	/// <summary>CompositionCancel must discard the pending composition without ever proposing a text change, restoring the pre-composition edit exactly.</summary>
	[Test]
	public void CompositionCancelDiscardsPendingComposition()
	{
		var lease = UiTestDevice.Lease;
		if (lease == null)
			return; // No live D3D12 device is available in this environment; skip.

		using var runtime = new UiRuntime();
		var context = runtime.CreateContext(device: lease);
		try
		{
			var (viewport, _) = SetUpFocusedTextBox(context);

			context.InjectInputText(NormalizedInput.CompositionStart(time_ms: 1), null);
			context.InjectInputText(NormalizedInput.CompositionUpdate(time_ms: 2), "ab", caret: 2, selectionStart: 0, selectionEnd: 2);
			context.InjectInputText(NormalizedInput.CompositionCancel(time_ms: 3), null);

			// The completed focusing click was drained by the setup helper; cancellation proposes nothing.
			var events = context.DrainEvents();
			Assert.Equal(0, events.Length);

			context.Update(viewport);
			var semantics = Array.Find(context.CaptureSemantics(), n => n.Id == new ControlId(2));
			Assert.NotNull(semantics);
			Assert.True((semantics!.TextFlags & ESemanticTextFlag.Composing) == 0);
		}
		finally
		{
			context.Dispose();
		}
	}

	/// <summary>InjectInputText must reject a text-carrying kind given no text, and a non-text-carrying kind given text, before ever reaching native code.</summary>
	[Test]
	public void InjectInputTextRejectsInconsistentTextCombinations()
	{
		var lease = UiTestDevice.Lease;
		if (lease == null)
			return; // No live D3D12 device is available in this environment; skip.

		using var runtime = new UiRuntime();
		var context = runtime.CreateContext(device: lease);
		try
		{
			// CompositionUpdate has nothing to compose without text.
			Assert.Throws<ArgumentException>(() => context.InjectInputText(NormalizedInput.CompositionUpdate(time_ms: 0), null));

			// CompositionStart never carries a text payload.
			Assert.Throws<ArgumentException>(() => context.InjectInputText(NormalizedInput.CompositionStart(time_ms: 0), "unexpected"));
		}
		finally
		{
			context.Dispose();
		}
	}

	/// <summary>
	/// Apply a one-root-one-TextBox transaction, update the viewport once so layout/hit-testing is current, then press
	/// inside the TextBox's laid-out bounds to focus it. Returns the viewport used, so callers can Update() again later.
	/// </summary>
	private static (ViewportState Viewport, ControlId TextBoxId) SetUpFocusedTextBox(UiContext context)
	{
		var root = new UiControlDesc { Id = new ControlId(1), Type = EControlType.Root };
		var text_box = new UiControlDesc
		{
			Id = new ControlId(2),
			ParentId = new ControlId(1),
			Type = EControlType.TextBox,
			Focusable = true,
			Layout = new UiLayoutParams { Width = 100, Height = 30, HAlign = EHAlign.Left, VAlign = EVAlign.Top },
		};

		var builder = new UiTransactionBuilder();
		builder.Upsert(root).Upsert(text_box);
		builder.Apply(context, base_revision: 0, revision: 1);

		var viewport = new ViewportState(800, 600, 800, 600, 0, 0, 800, 600, 96f, 0.0);
		context.Update(viewport);

		// A completed click inside the TextBox focuses it and leaves no pointer capture active.
		context.InjectInput(NormalizedInput.PointerButtonDown(10, 10, EPointerButton.Left, EPointerButtonMask.Left, EInputModifier.None, time_ms: 0));
		context.InjectInput(NormalizedInput.PointerButtonUp(10, 10, EPointerButton.Left, EPointerButtonMask.None, EInputModifier.None, time_ms: 0));

		// Keep composition assertions independent of the focus and capture events produced by setup.
		var setup_events = context.DrainEvents();
		Assert.Equal(2, setup_events.Length);
		Assert.Equal(EEventKind.FocusChanged, setup_events[0].Kind);
		Assert.Equal(EEventKind.PointerCaptureChanged, setup_events[1].Kind);
		return (viewport, new ControlId(2));
	}

	/// <summary>Require 'action' to throw a View3dUiException whose Status equals 'expected'.</summary>
	private static void ExpectStatus(EStatus expected, Action action)
	{
		try
		{
			action();
			throw new UnitTestException($"Expected native status {expected}.");
		}
		catch (View3dUiException ex)
		{
			Assert.Equal(expected, ex.Status);
		}
	}
}

/// <summary>
/// Validates the JSON layer (Json.UiDocument): pure-managed parsing must produce field-for-field identical descriptors to
/// the same document built directly through Descriptors.cs, and every malformed/unknown/dangling/duplicate input must be
/// rejected with a bounded, path-qualified UiJsonException before any native call would ever be attempted.
/// </summary>
[TestFixture]
public sealed class TestUiJson
{
	private const string ValidDocument = @"{
		""schema_version"": 1,
		""resources"": [ { ""id"": 1, ""kind"": ""Font"", ""name"": ""Segoe UI"", ""font_size"": 14 } ],
		""styles"": [ {
			""id"": 1,
			""states"": {
				""normal"": { ""visual"": { ""fill"": ""#3060C0FF"" } },
				""hover"": { ""visual"": { ""fill"": ""#4070D0FF"" }, ""transition"": { ""duration_ms"": 120, ""easing"": ""EaseInOut"" } }
			}
		} ],
		""templates"": [ {
			""id"": 1,
			""applies_to"": ""Button"",
			""parts"": [
				{ ""name"": ""PART_Background"", ""primitive"": ""SolidBox"" },
				{ ""name"": ""PART_ContentPresenter"", ""primitive"": ""ContentPresenter"", ""required"": false }
			]
		} ],
		""tree"": [ {
			""id"": 1,
			""type"": ""Root"",
			""children"": [ {
				""id"": 2,
				""type"": ""Button"",
				""style_id"": 1,
				""template_id"": 1,
				""font_resource_id"": 1,
				""text"": ""OK"",
				""name"": ""OkButton"",
				""description"": ""Confirms the dialog"",
				""focusable"": true,
				""layout"": { ""width"": 100, ""height"": 30, ""h_align"": ""Center"", ""v_align"": ""Center"" }
			} ]
		} ]
	}";

	/// <summary>A parsed document must produce the exact same descriptors as building the equivalent model directly through Descriptors.cs.</summary>
	[Test]
	public void ParseConvergesWithRuntimeConstruction()
	{
		var document = UiDocument.Parse(ValidDocument);
		Assert.Equal(1, document.SchemaVersion);
		Assert.Equal(1, document.Resources.Count);
		Assert.Equal(1, document.Styles.Count);
		Assert.Equal(1, document.Templates.Count);
		Assert.Equal(2, document.Controls.Count);

		var expected_resource = new UiResourceDesc { Id = new ResourceId(1), Kind = EResourceKind.Font, Name = "Segoe UI", FontSize = 14 };
		AssertResourcesEqual(expected_resource, document.Resources[0]);

		var expected_style = new UiStyleDesc { Id = new StyleId(1) };
		expected_style.SetVisual(EStateChannel.Normal, new StyleVisual(new Colour(0x30 / 255f, 0x60 / 255f, 0xC0 / 255f, 1f)));
		expected_style.SetVisual(EStateChannel.Hover, new StyleVisual(new Colour(0x40 / 255f, 0x70 / 255f, 0xD0 / 255f, 1f)));
		expected_style.SetTransition(EStateChannel.Hover, new TransitionDesc(120, EEasing.EaseInOut));
		AssertStylesEqual(expected_style, document.Styles[0]);

		var expected_template = new UiTemplateDesc { Id = new TemplateId(1), AppliesTo = EControlType.Button };
		expected_template.AddPart("PART_Background", EVisualPrimitive.SolidBox);
		expected_template.AddPart("PART_ContentPresenter", EVisualPrimitive.ContentPresenter, required: false);
		AssertTemplatesEqual(expected_template, document.Templates[0]);

		var expected_root = new UiControlDesc { Id = new ControlId(1), Type = EControlType.Root };
		AssertControlsEqual(expected_root, document.Controls[0]);

		var expected_button = new UiControlDesc
		{
			Id = new ControlId(2),
			ParentId = new ControlId(1),
			Type = EControlType.Button,
			StyleId = new StyleId(1),
			TemplateId = new TemplateId(1),
			FontResourceId = new ResourceId(1),
			Text = "OK",
			Name = "OkButton",
			Description = "Confirms the dialog",
			Focusable = true,
			Layout = new UiLayoutParams { Width = 100, Height = 30, HAlign = EHAlign.Center, VAlign = EVAlign.Center },
		};
		AssertControlsEqual(expected_button, document.Controls[1]);

		// Every descriptor Parse() produced must also build a transaction builder that packs without error.
		var builder = document.ToTransactionBuilder();
		Assert.NotNull(builder);
	}

	/// <summary>Serialize() called twice on the same parsed instance must produce byte-identical text; canonical serialization has no hidden non-determinism (e.g. dictionary/hash-set ordering).</summary>
	[Test]
	public void SerializeIsDeterministicAcrossCalls()
	{
		var document = UiDocument.Parse(ValidDocument);
		Assert.Equal(document.Serialize(), document.Serialize());
	}

	/// <summary>Serializing already-canonical text, reparsing it, and serializing again must reproduce the exact same text unchanged (Serialize() is idempotent under round-trip).</summary>
	[Test]
	public void SerializeIsIdempotent()
	{
		var canonical = UiDocument.Parse(ValidDocument).Serialize();
		var reparsed = UiDocument.Parse(canonical).Serialize();
		Assert.Equal(canonical, reparsed);
	}

	/// <summary>
	/// Two source documents that are semantically equivalent but differently shaped - reordered top-level properties,
	/// mixed-case hex colours, and a 6-digit colour equivalent to an explicit 8-digit-with-FF-alpha colour - must
	/// canonicalize to byte-identical serialized text, proving Serialize() output can be compared as plain strings to
	/// test document equivalence.
	/// </summary>
	[Test]
	public void SerializeCanonicalizesEquivalentInputs()
	{
		const string variant_a = @"{
			""schema_version"": 1,
			""resources"": [ { ""id"": 1, ""kind"": ""Colour"", ""colour"": ""#3060C0"" } ],
			""styles"": [],
			""templates"": [],
			""tree"": [ { ""id"": 1, ""type"": ""Root"" } ]
		}";
		const string variant_b = @"{
			""tree"": [ { ""type"": ""Root"", ""id"": 1 } ],
			""templates"": [],
			""styles"": [],
			""resources"": [ { ""colour"": ""#3060c0ff"", ""kind"": ""Colour"", ""id"": 1 } ],
			""schema_version"": 1
		}";

		var canonical_a = UiDocument.Parse(variant_a).Serialize();
		var canonical_b = UiDocument.Parse(variant_b).Serialize();
		Assert.Equal(canonical_a, canonical_b);
	}

	/// <summary>Malformed JSON text must be rejected before any tree-walking begins.</summary>
	[Test]
	public void MalformedJsonIsRejected()
	{
		Assert.Throws<UiJsonException>(() => UiDocument.Parse("{ not valid json"));
	}

	/// <summary>A non-object document root is rejected at the document root path.</summary>
	[Test]
	public void NonObjectRootIsRejected()
	{
		ExpectJsonError("$", () => UiDocument.Parse("[1, 2, 3]"));
	}

	/// <summary>An unsupported schema_version is rejected before any resource/style/template/control content is parsed.</summary>
	[Test]
	public void UnsupportedSchemaVersionIsRejected()
	{
		ExpectJsonError("$.schema_version", () => UiDocument.Parse(@"{ ""schema_version"": 2 }"));
	}

	/// <summary>An unrecognised closed-vocabulary enum string is rejected with the offending property's path.</summary>
	[Test]
	public void UnknownControlTypeIsRejected()
	{
		var json = @"{ ""schema_version"": 1, ""tree"": [ { ""id"": 1, ""type"": ""Bogus"" } ] }";
		ExpectJsonError("$.tree[0].type", () => UiDocument.Parse(json));
	}

	/// <summary>A control referencing a style id not declared in the document's "styles" array is rejected.</summary>
	[Test]
	public void DanglingStyleReferenceIsRejected()
	{
		var json = @"{ ""schema_version"": 1, ""tree"": [ { ""id"": 1, ""type"": ""Root"", ""style_id"": 99 } ] }";
		ExpectJsonError("$.tree[0].style_id", () => UiDocument.Parse(json));
	}

	/// <summary>A control referencing a template id not declared in the document's "templates" array is rejected.</summary>
	[Test]
	public void DanglingTemplateReferenceIsRejected()
	{
		var json = @"{ ""schema_version"": 1, ""tree"": [ { ""id"": 1, ""type"": ""Root"", ""template_id"": 42 } ] }";
		ExpectJsonError("$.tree[0].template_id", () => UiDocument.Parse(json));
	}

	/// <summary>A control referencing a font resource id not declared in the document's "resources" array is rejected.</summary>
	[Test]
	public void DanglingFontResourceReferenceIsRejected()
	{
		var json = @"{ ""schema_version"": 1, ""tree"": [ { ""id"": 1, ""type"": ""Root"", ""font_resource_id"": 7 } ] }";
		ExpectJsonError("$.tree[0].font_resource_id", () => UiDocument.Parse(json));
	}

	/// <summary>A control id of 0 (ControlId.None) is never a valid control identity.</summary>
	[Test]
	public void ZeroControlIdIsRejected()
	{
		var json = @"{ ""schema_version"": 1, ""tree"": [ { ""id"": 0, ""type"": ""Root"" } ] }";
		ExpectJsonError("$.tree[0].id", () => UiDocument.Parse(json));
	}

	/// <summary>Two controls declaring the same id anywhere in the tree are rejected as a duplicate identity.</summary>
	[Test]
	public void DuplicateControlIdIsRejected()
	{
		var json = @"{ ""schema_version"": 1, ""tree"": [
			{ ""id"": 1, ""type"": ""Root"", ""children"": [ { ""id"": 2, ""type"": ""Panel"" } ] },
			{ ""id"": 2, ""type"": ""Root"" }
		] }";
		ExpectJsonError("$.tree[1].id", () => UiDocument.Parse(json));
	}

	/// <summary>Two resources declaring the same id are rejected as a duplicate identity.</summary>
	[Test]
	public void DuplicateResourceIdIsRejected()
	{
		var json = @"{ ""schema_version"": 1, ""resources"": [
			{ ""id"": 1, ""kind"": ""Font"" },
			{ ""id"": 1, ""kind"": ""Colour"" }
		] }";
		ExpectJsonError("$.resources[1].id", () => UiDocument.Parse(json));
	}

	/// <summary>A template may not declare more than UiTemplateDesc.MaxParts parts.</summary>
	[Test]
	public void TemplatePartOverflowIsRejected()
	{
		var parts = new StringBuilder();
		for (var i = 0; i != UiTemplateDesc.MaxParts + 1; ++i)
		{
			if (i != 0)
				parts.Append(',');
			parts.Append($@"{{ ""name"": ""Part{i}"", ""primitive"": ""SolidBox"" }}");
		}
		var json = $@"{{ ""schema_version"": 1, ""templates"": [ {{ ""id"": 1, ""applies_to"": ""Button"", ""parts"": [ {parts} ] }} ] }}";
		ExpectJsonError($"$.templates[0].parts[{UiTemplateDesc.MaxParts}]", () => UiDocument.Parse(json));
	}

	/// <summary>A colour string that is not exactly "#RRGGBB" or "#RRGGBBAA" is rejected.</summary>
	[Test]
	public void MalformedColourHexIsRejected()
	{
		var json = @"{ ""schema_version"": 1, ""resources"": [ { ""id"": 1, ""kind"": ""Colour"", ""colour"": ""#12345"" } ] }";
		ExpectJsonError("$.resources[0].colour", () => UiDocument.Parse(json));
	}

	/// <summary>Compare two resource descriptors field-by-field for a more precise failure message than the record's own value equality gives.</summary>
	private static void AssertResourcesEqual(UiResourceDesc expected, UiResourceDesc actual)
	{
		Assert.Equal(expected.Id, actual.Id);
		Assert.Equal(expected.Kind, actual.Kind);
		Assert.Equal(expected.Colour, actual.Colour);
		Assert.Equal(expected.Name, actual.Name);
		Assert.Equal(expected.FontSize, actual.FontSize);
	}

	/// <summary>Compare two style descriptors field-by-field, across every EStateChannel.</summary>
	private static void AssertStylesEqual(UiStyleDesc expected, UiStyleDesc actual)
	{
		Assert.Equal(expected.Id, actual.Id);
		for (var channel = 0; channel != (int)EStateChannel.Count; ++channel)
		{
			Assert.Equal(expected.GetVisual((EStateChannel)channel), actual.GetVisual((EStateChannel)channel));
			Assert.Equal(expected.GetTransition((EStateChannel)channel), actual.GetTransition((EStateChannel)channel));
		}
	}

	/// <summary>Compare two template descriptors field-by-field, including their ordered parts.</summary>
	private static void AssertTemplatesEqual(UiTemplateDesc expected, UiTemplateDesc actual)
	{
		Assert.Equal(expected.Id, actual.Id);
		Assert.Equal(expected.AppliesTo, actual.AppliesTo);
		Assert.Equal(expected.Parts.Count, actual.Parts.Count);
		for (var i = 0; i != expected.Parts.Count; ++i)
			Assert.Equal(expected.Parts[i], actual.Parts[i]);
	}

	/// <summary>Compare two control descriptors field-by-field for a more precise failure message than the record's own value equality gives.</summary>
	private static void AssertControlsEqual(UiControlDesc expected, UiControlDesc actual)
	{
		Assert.Equal(expected.Id, actual.Id);
		Assert.Equal(expected.ParentId, actual.ParentId);
		Assert.Equal(expected.Type, actual.Type);
		Assert.Equal(expected.RootPolicy, actual.RootPolicy);
		Assert.Equal(expected.LayoutMode, actual.LayoutMode);
		Assert.Equal(expected.TemplateId, actual.TemplateId);
		Assert.Equal(expected.StyleId, actual.StyleId);
		Assert.Equal(expected.Enabled, actual.Enabled);
		Assert.Equal(expected.Visible, actual.Visible);
		Assert.Equal(expected.Focusable, actual.Focusable);
		Assert.Equal(expected.ValidationState, actual.ValidationState);
		Assert.Equal(expected.Text, actual.Text);
		Assert.Equal(expected.Name, actual.Name);
		Assert.Equal(expected.Description, actual.Description);
		Assert.Equal(expected.MaxTextLength, actual.MaxTextLength);
		Assert.Equal(expected.FontResourceId, actual.FontResourceId);
		Assert.Equal(expected.Selected, actual.Selected);
		Assert.Equal(expected.ValueSequence, actual.ValueSequence);
		Assert.Equal(expected.Layout.Width, actual.Layout.Width);
		Assert.Equal(expected.Layout.Height, actual.Layout.Height);
		Assert.Equal(expected.Layout.HAlign, actual.Layout.HAlign);
		Assert.Equal(expected.Layout.VAlign, actual.Layout.VAlign);
	}

	/// <summary>Require 'action' to throw a UiJsonException whose Path contains 'path_contains'.</summary>
	private static void ExpectJsonError(string path_contains, Action action)
	{
		try
		{
			action();
			throw new UnitTestException($"Expected a UiJsonException whose Path contains \"{path_contains}\".");
		}
		catch (UiJsonException ex)
		{
			Assert.True(ex.Path.Contains(path_contains));
		}
	}
}
#endif
