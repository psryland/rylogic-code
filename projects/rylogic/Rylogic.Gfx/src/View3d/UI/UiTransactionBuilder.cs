using System;
using System.Collections.Generic;
using System.Text;

namespace Rylogic.Gfx.UI;

/// <summary>
/// Accumulates one delta transaction's operations, control/resource/style/template upserts, removals, and shared UTF-8 text
/// blob, then applies them all to a UiContext in exactly one native TransactionApply call (implementation-plan.md section
/// 5.3). Every Add*/Upsert/Remove* method appends; nothing is ever removed from the builder itself, so reuse one builder
/// per transaction and discard it after Apply.
/// </summary>
public sealed unsafe class UiTransactionBuilder
{
	private readonly List<Native.Operation> m_operations = new();
	private readonly List<Native.ControlDesc> m_controls = new();
	private readonly List<Native.ChildOrder> m_child_orders = new();
	private readonly List<ControlId> m_child_ids = new();
	private readonly List<Native.ResourceDesc> m_resources = new();
	private readonly List<Native.StyleDesc> m_styles = new();
	private readonly List<Native.TemplateDesc> m_templates = new();
	private readonly List<ResourceId> m_resource_removals = new();
	private readonly List<StyleId> m_style_removals = new();
	private readonly List<TemplateId> m_template_removals = new();
	private readonly List<byte> m_blob = new();

	/// <summary>Append 'text' to the shared blob, returning its (offset, length) for a *_offset/*_length pair.</summary>
	public (uint Offset, uint Length) AddText(string text)
	{
		text ??= string.Empty;
		var bytes = Encoding.UTF8.GetBytes(text);
		var offset = checked((uint)m_blob.Count);
		m_blob.AddRange(bytes);
		return (offset, checked((uint)bytes.Length));
	}

	/// <summary>Queue an Upsert of 'control' (create-or-update semantics), packing its Text/Name/Description into the shared blob.</summary>
	public UiTransactionBuilder Upsert(UiControlDesc control)
	{
		if (control == null)
			throw new ArgumentNullException(nameof(control));

		m_controls.Add(ToNative(control));
		m_operations.Add(new Native.Operation
		{
			m_header = NativeHeader.Create<Native.Operation>(),
			m_kind = EOperationKind.Upsert,
			m_target_id = control.Id,
			m_child_order_index = 0,
			m_reserved0 = 0,
		});
		return this;
	}

	/// <summary>Queue a Remove of 'id' and its entire subtree.</summary>
	public UiTransactionBuilder Remove(ControlId id)
	{
		m_operations.Add(new Native.Operation
		{
			m_header = NativeHeader.Create<Native.Operation>(),
			m_kind = EOperationKind.Remove,
			m_target_id = id,
			m_child_order_index = 0,
			m_reserved0 = 0,
		});
		return this;
	}

	/// <summary>Queue a ReplaceSubtree of 'id'; an Upsert recreating 'id' must follow this operation in the same transaction.</summary>
	public UiTransactionBuilder ReplaceSubtree(ControlId id)
	{
		m_operations.Add(new Native.Operation
		{
			m_header = NativeHeader.Create<Native.Operation>(),
			m_kind = EOperationKind.ReplaceSubtree,
			m_target_id = id,
			m_child_order_index = 0,
			m_reserved0 = 0,
		});
		return this;
	}

	/// <summary>Queue a Reorder of 'parent_id''s children to exactly 'children' (must be a permutation of the parent's current children once the transaction is staged).</summary>
	public UiTransactionBuilder Reorder(ControlId parent_id, IReadOnlyList<ControlId> children)
	{
		if (children == null)
			throw new ArgumentNullException(nameof(children));

		var offset = checked((uint)m_child_ids.Count);
		foreach (var child in children)
			m_child_ids.Add(child);

		var index = checked((uint)m_child_orders.Count);
		m_child_orders.Add(new Native.ChildOrder
		{
			m_parent_id = parent_id,
			m_offset = offset,
			m_count = checked((uint)children.Count),
		});
		m_operations.Add(new Native.Operation
		{
			m_header = NativeHeader.Create<Native.Operation>(),
			m_kind = EOperationKind.Reorder,
			m_target_id = parent_id,
			m_child_order_index = index,
			m_reserved0 = 0,
		});
		return this;
	}

	/// <summary>Append a resource descriptor (upsert-by-id semantics: an id already present in the accepted tree is updated, not duplicated), packing its Name into the shared blob.</summary>
	public UiTransactionBuilder AddResource(UiResourceDesc resource)
	{
		if (resource == null)
			throw new ArgumentNullException(nameof(resource));

		var (name_offset, name_length) = AddText(resource.Name);
		m_resources.Add(new Native.ResourceDesc
		{
			m_header = NativeHeader.Create<Native.ResourceDesc>(),
			m_id = resource.Id,
			m_kind = resource.Kind,
			m_colour = resource.Colour,
			m_name_offset = name_offset,
			m_name_length = name_length,
			m_font_size = resource.FontSize,
		});
		return this;
	}

	/// <summary>Append a style descriptor (upsert-by-id semantics).</summary>
	public UiTransactionBuilder AddStyle(UiStyleDesc style)
	{
		if (style == null)
			throw new ArgumentNullException(nameof(style));

		var native = new Native.StyleDesc
		{
			m_header = NativeHeader.Create<Native.StyleDesc>(),
			m_id = style.Id,
		};
		for (var channel = 0; channel != (int)EStateChannel.Count; ++channel)
		{
			native.SetVisual(channel, style.GetVisual((EStateChannel)channel));
			native.SetTransition(channel, style.GetTransition((EStateChannel)channel));
		}
		m_styles.Add(native);
		return this;
	}

	/// <summary>Append a template descriptor (upsert-by-id semantics), packing each part's name into the shared blob.</summary>
	public UiTransactionBuilder AddTemplate(UiTemplateDesc template)
	{
		if (template == null)
			throw new ArgumentNullException(nameof(template));

		var native = new Native.TemplateDesc
		{
			m_header = NativeHeader.Create<Native.TemplateDesc>(),
			m_id = template.Id,
			m_applies_to = template.AppliesTo,
			m_part_count = checked((uint)template.Parts.Count),
		};
		for (var i = 0; i != template.Parts.Count; ++i)
		{
			var part = template.Parts[i];
			var (name_offset, name_length) = AddText(part.Name);
			native.SetPart(i, new Native.TemplatePart
			{
				m_name_offset = name_offset,
				m_name_length = name_length,
				m_primitive = part.Primitive,
				m_required = part.Required ? 1 : 0,
			});
		}
		m_templates.Add(native);
		return this;
	}

	/// <summary>Queue removal of a previously-accepted resource id. Rejected atomically if the id does not exist or is still referenced by a control present once this transaction's operations have applied.</summary>
	public UiTransactionBuilder RemoveResource(ResourceId id)
	{
		m_resource_removals.Add(id);
		return this;
	}

	/// <summary>Queue removal of a previously-accepted style id. Rejected atomically if the id does not exist or is still referenced by a control present once this transaction's operations have applied.</summary>
	public UiTransactionBuilder RemoveStyle(StyleId id)
	{
		m_style_removals.Add(id);
		return this;
	}

	/// <summary>Queue removal of a previously-accepted template id. Rejected atomically if the id does not exist or is still referenced by a control present once this transaction's operations have applied.</summary>
	public UiTransactionBuilder RemoveTemplate(TemplateId id)
	{
		m_template_removals.Add(id);
		return this;
	}

	/// <summary>
	/// Materialise every accumulated operation/descriptor/removal and the shared blob as one Transaction, pin them for the
	/// duration of a single native call, and apply them to 'context'. Safe to call more than once on the same builder (for
	/// example to retry after a StaleRevision failure with an updated 'revision'); each call re-pins the same accumulated content.
	/// </summary>
	public void Apply(UiContext context, ulong base_revision, ulong revision)
	{
		if (context == null)
			throw new ArgumentNullException(nameof(context));

		var operations = m_operations.ToArray();
		var controls = m_controls.ToArray();
		var child_orders = m_child_orders.ToArray();
		var child_ids = m_child_ids.ToArray();
		var resources = m_resources.ToArray();
		var styles = m_styles.ToArray();
		var templates = m_templates.ToArray();
		var resource_removals = m_resource_removals.ToArray();
		var style_removals = m_style_removals.ToArray();
		var template_removals = m_template_removals.ToArray();
		var blob = m_blob.ToArray();

		fixed (Native.Operation* operations_ptr = operations)
		fixed (Native.ControlDesc* controls_ptr = controls)
		fixed (Native.ChildOrder* child_orders_ptr = child_orders)
		fixed (ControlId* child_ids_ptr = child_ids)
		fixed (Native.ResourceDesc* resources_ptr = resources)
		fixed (Native.StyleDesc* styles_ptr = styles)
		fixed (Native.TemplateDesc* templates_ptr = templates)
		fixed (ResourceId* resource_removals_ptr = resource_removals)
		fixed (StyleId* style_removals_ptr = style_removals)
		fixed (TemplateId* template_removals_ptr = template_removals)
		fixed (byte* blob_ptr = blob)
		{
			var transaction = new Native.Transaction
			{
				m_header = NativeHeader.Create<Native.Transaction>(),
				m_base_revision = base_revision,
				m_revision = revision,
				m_operations = (IntPtr)operations_ptr,
				m_operation_count = checked((uint)operations.Length),
				m_controls = (IntPtr)controls_ptr,
				m_control_count = checked((uint)controls.Length),
				m_child_orders = (IntPtr)child_orders_ptr,
				m_child_order_count = checked((uint)child_orders.Length),
				m_child_ids = (IntPtr)child_ids_ptr,
				m_child_id_count = checked((uint)child_ids.Length),
				m_resources = (IntPtr)resources_ptr,
				m_resource_count = checked((uint)resources.Length),
				m_styles = (IntPtr)styles_ptr,
				m_style_count = checked((uint)styles.Length),
				m_templates = (IntPtr)templates_ptr,
				m_template_count = checked((uint)templates.Length),
				m_resource_removals = (IntPtr)resource_removals_ptr,
				m_resource_removal_count = checked((uint)resource_removals.Length),
				m_style_removals = (IntPtr)style_removals_ptr,
				m_style_removal_count = checked((uint)style_removals.Length),
				m_template_removals = (IntPtr)template_removals_ptr,
				m_template_removal_count = checked((uint)template_removals.Length),
				m_blob = (IntPtr)blob_ptr,
				m_blob_length = checked((uint)blob.Length),
			};
			context.ApplyTransactionCore(&transaction);
		}
	}

	/// <summary>
	/// The already-snapshotted native control descriptors queued so far, exposed only so unit tests can prove that mutating a
	/// UiControlDesc after Upsert cannot change what was queued; production code should never need this.
	/// </summary>
	internal IReadOnlyList<Native.ControlDesc> DebugControls
	{
		get
		{
			return m_controls;
		}
	}

	/// <summary>
	/// The already-snapshotted native resource descriptors queued so far, exposed only so unit tests can prove that mutating a
	/// UiResourceDesc after AddResource cannot change what was queued; production code should never need this.
	/// </summary>
	internal IReadOnlyList<Native.ResourceDesc> DebugResources
	{
		get
		{
			return m_resources;
		}
	}

	/// <summary>
	/// The already-snapshotted native style descriptors queued so far, exposed only so unit tests can prove that mutating a
	/// UiStyleDesc after AddStyle cannot change what was queued; production code should never need this.
	/// </summary>
	internal IReadOnlyList<Native.StyleDesc> DebugStyles
	{
		get
		{
			return m_styles;
		}
	}

	/// <summary>
	/// The already-snapshotted native template descriptors queued so far, exposed only so unit tests can prove that mutating a
	/// UiTemplateDesc after AddTemplate cannot change what was queued; production code should never need this.
	/// </summary>
	internal IReadOnlyList<Native.TemplateDesc> DebugTemplates
	{
		get
		{
			return m_templates;
		}
	}

	/// <summary>Decode 'length' UTF-8 bytes starting at 'offset' in the shared blob; used by unit tests to verify packed Text/Name/Description snapshots.</summary>
	internal string DebugDecodeBlobText(uint offset, uint length)
	{
		if (length == 0)
			return string.Empty;

		return Encoding.UTF8.GetString(m_blob.ToArray(), checked((int)offset), checked((int)length));
	}

	/// <summary>Convert a public control descriptor to its versioned ABI representation, packing its Text/Name/Description into the shared blob.</summary>
	private Native.ControlDesc ToNative(UiControlDesc control)
	{
		var (text_offset, text_length) = AddText(control.Text);
		var (name_offset, name_length) = AddText(control.Name);
		var (desc_offset, desc_length) = AddText(control.Description);
		return new Native.ControlDesc
		{
			m_header = NativeHeader.Create<Native.ControlDesc>(),
			m_id = control.Id,
			m_parent_id = control.ParentId,
			m_type = control.Type,
			m_root_policy = control.RootPolicy,
			m_layout_mode = control.LayoutMode,
			m_template_id = control.TemplateId,
			m_style_id = control.StyleId,
			m_enabled = control.Enabled ? 1 : 0,
			m_visible = control.Visible ? 1 : 0,
			m_focusable = control.Focusable ? 1 : 0,
			m_validation_state = control.ValidationState,
			m_layout = ToNative(control.Layout),
			m_text_offset = text_offset,
			m_text_length = text_length,
			m_name_offset = name_offset,
			m_name_length = name_length,
			m_desc_offset = desc_offset,
			m_desc_length = desc_length,
			m_max_text_length = control.MaxTextLength,
			m_font_resource_id = control.FontResourceId,
			m_selected = control.Selected ? 1 : 0,
			m_value_sequence = control.ValueSequence,
			m_world = ToNative(control.World),
		};
	}

	/// <summary>Convert a public layout descriptor to its ABI representation.</summary>
	private static Native.LayoutParams ToNative(UiLayoutParams layout)
	{
		return new Native.LayoutParams
		{
			m_width = layout.Width,
			m_height = layout.Height,
			m_margin_left = layout.MarginLeft,
			m_margin_top = layout.MarginTop,
			m_margin_right = layout.MarginRight,
			m_margin_bottom = layout.MarginBottom,
			m_padding_left = layout.PaddingLeft,
			m_padding_top = layout.PaddingTop,
			m_padding_right = layout.PaddingRight,
			m_padding_bottom = layout.PaddingBottom,
			m_h_align = layout.HAlign,
			m_v_align = layout.VAlign,
			m_stack_spacing = layout.StackSpacing,
			m_canvas_x = layout.CanvasX,
			m_canvas_y = layout.CanvasY,
			m_scroll_offset_x = layout.ScrollOffsetX,
			m_scroll_offset_y = layout.ScrollOffsetY,
		};
	}

	/// <summary>Convert a public world-root descriptor to its ABI representation.</summary>
	private static Native.WorldRootParams ToNative(UiWorldRootParams world)
	{
		return new Native.WorldRootParams
		{
			m_anchor = world.Anchor,
			m_anchor_h = world.AnchorH,
			m_anchor_v = world.AnchorV,
			m_sizing = world.Sizing,
			m_world_units_per_dip = world.WorldUnitsPerDip,
			m_depth_offset = world.DepthOffset,
			m_occlusion_min_opacity = world.OcclusionMinOpacity,
			m_occlusion_fade_depth = world.OcclusionFadeDepth,
			m_occlusion_depth_bias = world.OcclusionDepthBias,
		};
	}
}
