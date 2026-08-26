using System;
using System.Runtime.InteropServices;
using System.Text;
using Rylogic.Interop.Win32;

namespace Rylogic.Gfx.UI;

/// <summary>Fixed-layout P/Invoke surface for the versioned native View3DUI ABI (include/pr/view3d-ui/view3d-ui-dll.h).</summary>
internal static unsafe class Native
{
	internal const string Dll = "view3d-ui";
	internal const uint ApiVersion = 0x00030000U;
	internal const uint StructVersion = 3U;
	private static IntPtr m_module;

	/// <summary>Load the configuration-appropriate native runtime before the first P/Invoke.</summary>
	internal static void EnsureLoaded()
	{
		if (m_module != IntPtr.Zero)
			return;

		m_module = Win32.LoadDll(Dll + ".dll", out var load_error);
		if (m_module == IntPtr.Zero)
			throw load_error ?? new DllNotFoundException($"Unable to load {Dll}.dll.");
	}

	/// <summary>Native error-reporting callback shape: (ctx, msg, filepath, line). Unlike Physics, View3DUI has no source-position parameter.</summary>
	[UnmanagedFunctionPointer(CallingConvention.StdCall)]
	internal delegate void ReportErrorFn(IntPtr ctx, [MarshalAs(UnmanagedType.LPStr)] string msg, [MarshalAs(UnmanagedType.LPStr)] string filepath, int line);

	[StructLayout(LayoutKind.Sequential)]
	internal struct ReportErrorCallback
	{
		internal IntPtr m_ctx;
		internal ReportErrorFn m_cb;
	}

	[StructLayout(LayoutKind.Sequential)]
	internal struct Config
	{
		internal NativeHeader m_header;
		internal uint m_max_controls;
		internal uint m_max_roots;
		internal uint m_max_tree_depth;
		internal uint m_max_operations_per_transaction;
		internal uint m_max_blob_bytes;
		internal uint m_max_templates;
		internal uint m_max_styles;
		internal uint m_max_resources;
		internal uint m_max_glyph_cache_bytes;
		internal uint m_max_glyph_cache_pages;
		internal uint m_max_generated_vertices;
		internal uint m_max_generated_indices;
		internal uint m_max_queued_events;
		internal uint m_max_semantic_records;
		internal float m_max_transition_duration_ms;

		/// <summary>Convert a public config into its versioned ABI representation.</summary>
		internal static Config From(UiConfig options)
		{
			return new Config
			{
				m_header = NativeHeader.Create<Config>(),
				m_max_controls = options.MaxControls,
				m_max_roots = options.MaxRoots,
				m_max_tree_depth = options.MaxTreeDepth,
				m_max_operations_per_transaction = options.MaxOperationsPerTransaction,
				m_max_blob_bytes = options.MaxBlobBytes,
				m_max_templates = options.MaxTemplates,
				m_max_styles = options.MaxStyles,
				m_max_resources = options.MaxResources,
				m_max_glyph_cache_bytes = options.MaxGlyphCacheBytes,
				m_max_glyph_cache_pages = options.MaxGlyphCachePages,
				m_max_generated_vertices = options.MaxGeneratedVertices,
				m_max_generated_indices = options.MaxGeneratedIndices,
				m_max_queued_events = options.MaxQueuedEvents,
				m_max_semantic_records = options.MaxSemanticRecords,
				m_max_transition_duration_ms = options.MaxTransitionDurationMs,
			};
		}
	}

	[StructLayout(LayoutKind.Sequential)]
	internal struct LayoutParams
	{
		internal float m_width;
		internal float m_height;
		internal float m_margin_left;
		internal float m_margin_top;
		internal float m_margin_right;
		internal float m_margin_bottom;
		internal float m_padding_left;
		internal float m_padding_top;
		internal float m_padding_right;
		internal float m_padding_bottom;
		internal EHAlign m_h_align;
		internal EVAlign m_v_align;
		internal float m_stack_spacing;
		internal float m_canvas_x;
		internal float m_canvas_y;
		internal float m_scroll_offset_x;
		internal float m_scroll_offset_y;
	}

	[StructLayout(LayoutKind.Sequential)]
	internal struct WorldRootParams
	{
		internal Vec3 m_anchor;
		internal EAnchorPoint m_anchor_h;
		internal EAnchorPoint m_anchor_v;
		internal EWorldSizing m_sizing;
		internal float m_world_units_per_dip;
		internal float m_depth_offset;
		internal float m_occlusion_min_opacity;
		internal float m_occlusion_fade_depth;
		internal float m_occlusion_depth_bias;
	}

	[StructLayout(LayoutKind.Sequential)]
	internal struct ControlDesc
	{
		internal NativeHeader m_header;
		internal ControlId m_id;
		internal ControlId m_parent_id;
		internal EControlType m_type;
		internal ERootPolicy m_root_policy;
		internal ELayoutMode m_layout_mode;
		internal TemplateId m_template_id;
		internal StyleId m_style_id;
		internal int m_enabled;
		internal int m_visible;
		internal int m_focusable;
		internal EValidationState m_validation_state;
		internal LayoutParams m_layout;
		internal uint m_text_offset;
		internal uint m_text_length;
		internal uint m_name_offset;
		internal uint m_name_length;
		internal uint m_desc_offset;
		internal uint m_desc_length;
		internal uint m_max_text_length;
		internal ResourceId m_font_resource_id;
		internal int m_selected;
		internal uint m_value_sequence;
		internal WorldRootParams m_world;
	}

	[StructLayout(LayoutKind.Sequential)]
	internal struct ChildOrder
	{
		internal ControlId m_parent_id;
		internal uint m_offset;
		internal uint m_count;
	}

	[StructLayout(LayoutKind.Sequential)]
	internal struct Operation
	{
		internal NativeHeader m_header;
		internal EOperationKind m_kind;
		internal ControlId m_target_id;
		internal uint m_child_order_index;
		internal uint m_reserved0;
	}

	[StructLayout(LayoutKind.Sequential)]
	internal struct ResourceDesc
	{
		internal NativeHeader m_header;
		internal ResourceId m_id;
		internal EResourceKind m_kind;
		internal Colour m_colour;
		internal uint m_name_offset;
		internal uint m_name_length;
		internal float m_font_size;
	}

	// StyleDesc.visuals/transitions are fixed 9-element (EStateChannel::Count) arrays in native code. C# structs cannot embed a
	// fixed-size array of non-primitive blittable elements without either [InlineArray] (unavailable on net481) or explicit
	// per-element fields, so each element is named m_visual_N/m_transition_N and indexed via pointer arithmetic below; sequential
	// same-type fields are laid out contiguously with no inter-element padding, reproducing the native array layout exactly.
	[StructLayout(LayoutKind.Sequential)]
	internal struct StyleDesc
	{
		internal NativeHeader m_header;
		internal StyleId m_id;
		internal StyleVisual m_visual_0;
		internal StyleVisual m_visual_1;
		internal StyleVisual m_visual_2;
		internal StyleVisual m_visual_3;
		internal StyleVisual m_visual_4;
		internal StyleVisual m_visual_5;
		internal StyleVisual m_visual_6;
		internal StyleVisual m_visual_7;
		internal StyleVisual m_visual_8;
		internal TransitionDesc m_transition_0;
		internal TransitionDesc m_transition_1;
		internal TransitionDesc m_transition_2;
		internal TransitionDesc m_transition_3;
		internal TransitionDesc m_transition_4;
		internal TransitionDesc m_transition_5;
		internal TransitionDesc m_transition_6;
		internal TransitionDesc m_transition_7;
		internal TransitionDesc m_transition_8;

		/// <summary>Read the state-channel visual at 'channel' (0..EStateChannel.Count-1).</summary>
		internal StyleVisual GetVisual(int channel)
		{
			fixed (StyleVisual* first = &m_visual_0)
				return first[channel];
		}

		/// <summary>Write the state-channel visual at 'channel' (0..EStateChannel.Count-1).</summary>
		internal void SetVisual(int channel, StyleVisual value)
		{
			fixed (StyleVisual* first = &m_visual_0)
				first[channel] = value;
		}

		/// <summary>Read the state-channel transition at 'channel' (0..EStateChannel.Count-1).</summary>
		internal TransitionDesc GetTransition(int channel)
		{
			fixed (TransitionDesc* first = &m_transition_0)
				return first[channel];
		}

		/// <summary>Write the state-channel transition at 'channel' (0..EStateChannel.Count-1).</summary>
		internal void SetTransition(int channel, TransitionDesc value)
		{
			fixed (TransitionDesc* first = &m_transition_0)
				first[channel] = value;
		}
	}

	[StructLayout(LayoutKind.Sequential)]
	internal struct TemplatePart
	{
		internal uint m_name_offset;
		internal uint m_name_length;
		internal EVisualPrimitive m_primitive;
		internal int m_required;
	}

	// TemplateDesc.parts is a fixed VIEW3D_UI_MAX_TEMPLATE_PARTS(8)-element array in native code; see the StyleDesc comment above
	// for why this is reproduced as explicit named fields (m_part_0..m_part_7) rather than a true fixed-size array.
	[StructLayout(LayoutKind.Sequential)]
	internal struct TemplateDesc
	{
		internal NativeHeader m_header;
		internal TemplateId m_id;
		internal EControlType m_applies_to;
		internal uint m_part_count;
		internal TemplatePart m_part_0;
		internal TemplatePart m_part_1;
		internal TemplatePart m_part_2;
		internal TemplatePart m_part_3;
		internal TemplatePart m_part_4;
		internal TemplatePart m_part_5;
		internal TemplatePart m_part_6;
		internal TemplatePart m_part_7;

		/// <summary>Read the template part at 'index' (0..VIEW3D_UI_MAX_TEMPLATE_PARTS-1).</summary>
		internal TemplatePart GetPart(int index)
		{
			fixed (TemplatePart* first = &m_part_0)
				return first[index];
		}

		/// <summary>Write the template part at 'index' (0..VIEW3D_UI_MAX_TEMPLATE_PARTS-1).</summary>
		internal void SetPart(int index, TemplatePart value)
		{
			fixed (TemplatePart* first = &m_part_0)
				first[index] = value;
		}
	}

	// Transaction's array fields borrow caller-owned, pinned memory for the duration of one TransactionApply call; they are
	// declared as IntPtr (rather than T*) so this struct itself needs no unsafe context, matching how the pointers are supplied
	// by UiTransactionBuilder.Apply via GCHandle-pinned managed arrays.
	[StructLayout(LayoutKind.Sequential)]
	internal struct Transaction
	{
		internal NativeHeader m_header;
		internal ulong m_base_revision;
		internal ulong m_revision;
		internal IntPtr m_operations;
		internal uint m_operation_count;
		internal IntPtr m_controls;
		internal uint m_control_count;
		internal IntPtr m_child_orders;
		internal uint m_child_order_count;
		internal IntPtr m_child_ids;
		internal uint m_child_id_count;
		internal IntPtr m_resources;
		internal uint m_resource_count;
		internal IntPtr m_styles;
		internal uint m_style_count;
		internal IntPtr m_templates;
		internal uint m_template_count;
		internal IntPtr m_resource_removals;
		internal uint m_resource_removal_count;
		internal IntPtr m_style_removals;
		internal uint m_style_removal_count;
		internal IntPtr m_template_removals;
		internal uint m_template_removal_count;
		internal IntPtr m_blob;
		internal uint m_blob_length;
	}

	// NormalizedInput, CameraState, ViewportState, and Diagnostics contain no strings or oversized fixed arrays, so they are
	// defined once as public blittable structs in Types.cs and reused directly here as the P/Invoke wire format.

	[StructLayout(LayoutKind.Sequential)]
	internal struct Event
	{
		internal NativeHeader m_header;
		internal ControlId m_control_id;
		internal EEventKind m_kind;
		internal ulong m_accepted_revision;
		internal ulong m_sequence;
		internal uint m_payload_offset;
		internal uint m_payload_length;
		internal uint m_edit_generation;
		internal uint m_reserved0;
	}

	[StructLayout(LayoutKind.Sequential)]
	internal struct SemanticNode
	{
		internal NativeHeader m_header;
		internal ControlId m_id;
		internal ControlId m_parent_id;
		internal EControlType m_role;
		internal uint m_name_offset;
		internal uint m_name_length;
		internal uint m_desc_offset;
		internal uint m_desc_length;
		internal uint m_value_offset;
		internal uint m_value_length;
		internal uint m_state_flags;
		internal uint m_supported_actions;

		// Text-range fields (M9): a legitimate zero offset (a caret at the start of the value text) must stay
		// distinguishable from "this node has no caret at all", so 'm_text_flags' reports which of these fields
		// carry meaning rather than the caller inferring presence from a sentinel value.
		internal uint m_caret;
		internal uint m_selection_start;
		internal uint m_selection_end;
		internal uint m_composition_start;
		internal uint m_composition_length;
		internal uint m_text_flags;
		internal uint m_value_grapheme_count;
		internal uint m_reserved0;

		internal Rect m_bounds;
		internal ulong m_accepted_revision;
		internal ulong m_semantic_sequence;
	}

	// Variable-length text accompanying one NormalizedInput record (the 'InputTextPayload' EStructId, section 7.2).
	// Kept separate from NormalizedInput so that record's fixed layout is unchanged. 'm_text_utf8' borrows caller-pinned
	// memory for the duration of one View3DUI_InputInjectText call only; UiContext.InjectInputText owns the pin.
	[StructLayout(LayoutKind.Sequential)]
	internal struct InputTextPayload
	{
		internal NativeHeader m_header;
		internal IntPtr m_text_utf8;
		internal uint m_text_length;
		internal uint m_caret;
		internal uint m_selection_start;
		internal uint m_selection_end;
	}

	/// <summary>Throw a managed exception containing the native thread-local error message.</summary>
	internal static void Check(EStatus status)
	{
		if (status == EStatus.Success)
			return;

		uint required;
		View3DUI_LastError(null, 0, out required);
		var bytes = new byte[Math.Max(required, 1)];
		fixed (byte* ptr = bytes)
		{
			View3DUI_LastError(ptr, (uint)bytes.Length, out required);
		}
		var count = Array.IndexOf(bytes, (byte)0);
		if (count < 0)
			count = bytes.Length;

		var message = Encoding.UTF8.GetString(bytes, 0, count);
		throw new View3dUiException(status, string.IsNullOrWhiteSpace(message) ? $"Native View3DUI call failed with status {status}." : message);
	}

	[DllImport(Dll)] internal static extern uint View3DUI_ApiVersion();
	[DllImport(Dll)] internal static extern EStatus View3DUI_StructSize(EStructId struct_id, out uint size);
	[DllImport(Dll)] private static extern EStatus View3DUI_LastError(byte* buffer, uint capacity, out uint required);
	[DllImport(Dll)] internal static extern IntPtr View3DUI_Initialise(ReportErrorCallback global_error_cb);
	[DllImport(Dll)] internal static extern EStatus View3DUI_Shutdown(IntPtr runtime);
	[DllImport(Dll)] internal static extern void View3DUI_ContextAbandon(IntPtr runtime);

	[DllImport(Dll)] internal static extern EStatus View3DUI_ContextCreate(IntPtr runtime, Config* config, IntPtr external_d3d12_device, IntPtr view3d_window, out ulong context);
	[DllImport(Dll)] internal static extern EStatus View3DUI_ContextDestroy(ulong context);
	[DllImport(Dll)] internal static extern void View3DUI_UiContextAbandon(ulong context);

	[DllImport(Dll)] internal static extern EStatus View3DUI_TransactionApply(ulong context, Transaction* transaction);
	[DllImport(Dll)] internal static extern EStatus View3DUI_ProcessWindowMessage(ulong context, IntPtr hwnd, uint msg, IntPtr wparam, IntPtr lparam, out int consumed, out IntPtr result, out int invalidate);
	[DllImport(Dll)] internal static extern EStatus View3DUI_InputInject(ulong context, NormalizedInput* input);
	[DllImport(Dll)] internal static extern EStatus View3DUI_InputInjectText(ulong context, NormalizedInput* input, InputTextPayload* payload);
	[DllImport(Dll)] internal static extern EStatus View3DUI_CaretGeometry(ulong context, ControlId control_id, Rect* caret_dip, int* valid);
	[DllImport(Dll)] internal static extern EStatus View3DUI_Update(ulong context, ViewportState* viewport);

	[DllImport(Dll)] internal static extern EStatus View3DUI_EventCount(ulong context, out uint count);
	[DllImport(Dll)] internal static extern EStatus View3DUI_EventsCopy(ulong context, Event* events, uint capacity, out uint required, byte* payload_blob, uint payload_capacity, out uint payload_required);
	[DllImport(Dll)] internal static extern EStatus View3DUI_SemanticsCopy(ulong context, SemanticNode* nodes, uint capacity, out uint required, byte* text_blob, uint text_capacity, out uint text_required);
	[DllImport(Dll)] internal static extern EStatus View3DUI_DiagnosticsGet(ulong context, UiDiagnostics* diagnostics);

	[DllImport("kernel32.dll")] internal static extern uint GetCurrentThreadId();

	// Reserved placeholders for the private View3D host-bridge structures (EStructId.HostBridgeVersion/HostPassContext).
	// Neither is implemented or exposed by this managed layer; they exist only so the ABI struct-size validation loop
	// enumerates the complete EStructId set, matching the native placeholder comment in types.h.
	[StructLayout(LayoutKind.Sequential)]
	internal struct HostBridgeVersion
	{
		internal NativeHeader m_header;
		internal uint m_bridge_abi_version;
	}

	[StructLayout(LayoutKind.Sequential)]
	internal struct HostPassContext
	{
		internal NativeHeader m_header;
		internal ulong m_reserved0;
	}
}

[StructLayout(LayoutKind.Sequential)]
internal readonly struct NativeHeader
{
	internal readonly uint m_size;
	internal readonly uint m_version;

	internal NativeHeader(uint size)
	{
		m_size = size;
		m_version = Native.StructVersion;
	}

	internal static NativeHeader Create<T>()
	{
		return new NativeHeader(NativeSize<T>.Value);
	}
}

/// <summary>Caches one managed ABI record size so hot command construction performs no reflection.</summary>
internal static class NativeSize<T>
{
	internal static readonly uint Value = (uint)Marshal.SizeOf(typeof(T));
}
