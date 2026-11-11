using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text;
using System.Text.Json;
using Rylogic.Gfx.UI;

namespace Rylogic.Gfx.UI.Json;

/// <summary>
/// A parsed and fully-validated View3DUI JSON document (schema version 3): the same closed-vocabulary resource, style,
/// template, and control-tree descriptors a runtime application would construct directly via Descriptors.cs and
/// UiTransactionBuilder, produced instead from a versioned JSON authoring format. Parsing performs every validation
/// this layer is responsible for - malformed JSON, unknown schema versions, unknown closed-vocabulary kinds, and
/// dangling id references - before any native call is made; the native DLL never sees the original JSON text.
/// Image resources name their texture with a caller-defined key (JSON cannot hold texture handles); ToTransactionBuilder
/// resolves each key through a caller-supplied lookup.
/// </summary>
public sealed class UiDocument
{
	/// <summary>The only schema_version this parser currently accepts.</summary>
	public const int CurrentSchemaVersion = 3;

	/// <summary>The schema_version declared by the parsed document.</summary>
	public int SchemaVersion { get; }

	/// <summary>
	/// Every resource declared by the document, in declaration order. Image resources have no texture handle or size here; those are
	/// filled in by ToTransactionBuilder from the texture named in TextureKeys.
	/// </summary>
	public IReadOnlyList<UiResourceDesc> Resources { get; }

	/// <summary>The caller-defined texture key of every Image resource, by resource id.</summary>
	public IReadOnlyDictionary<ResourceId, string> TextureKeys { get; }

	/// <summary>Every style declared by the document, in declaration order.</summary>
	public IReadOnlyList<UiStyleDesc> Styles { get; }

	/// <summary>Every template declared by the document, in declaration order.</summary>
	public IReadOnlyList<UiTemplateDesc> Templates { get; }

	/// <summary>Every control declared by the document's "tree" section, flattened into parent-before-child (pre-order) order with ParentId already resolved from JSON nesting.</summary>
	public IReadOnlyList<UiControlDesc> Controls { get; }

	/// <summary>Adopt an already-validated set of descriptors.</summary>
	private UiDocument(int schema_version, IReadOnlyList<UiResourceDesc> resources, IReadOnlyDictionary<ResourceId, string> texture_keys, IReadOnlyList<UiStyleDesc> styles, IReadOnlyList<UiTemplateDesc> templates, IReadOnlyList<UiControlDesc> controls)
	{
		SchemaVersion = schema_version;
		Resources = resources;
		TextureKeys = texture_keys;
		Styles = styles;
		Templates = templates;
		Controls = controls;
	}

	/// <summary>Parse and fully validate a View3DUI JSON document. Throws UiJsonException for any malformed, unrecognised, or dangling content.</summary>
	public static UiDocument Parse(string json)
	{
		if (json == null)
			throw new ArgumentNullException(nameof(json));

		using var document = ParseDocument(json);
		return ParseRoot(document.RootElement);
	}

	/// <summary>
	/// Build one UiTransactionBuilder containing every resource/style/template/control this document declares, in an
	/// order safe to Apply directly to a fresh (or compatible) context: resources, styles, and templates first (so
	/// controls that reference them upsert cleanly), then controls in parent-before-child order. 'texture_lookup' maps each Image
	/// resource's texture key to a View3D texture; it is required when the document declares Image resources. The caller keeps
	/// ownership of the returned textures and must keep them alive while the resources are registered. Throws UiJsonException
	/// (with the resource's "texture" path) when a key cannot be resolved.
	/// </summary>
	public UiTransactionBuilder ToTransactionBuilder(Func<string, View3d.Texture?>? texture_lookup = null)
	{
		var builder = new UiTransactionBuilder();

		// Resources go first; Image resources take their handle and size from the caller's texture for that key
		for (var i = 0; i != Resources.Count; ++i)
		{
			var resource = Resources[i];
			switch (resource.Kind)
			{
				case EResourceKind.Image:
				{
					var path = $"$.resources[{i}].texture";
					var key = TextureKeys[resource.Id];
					if (texture_lookup == null)
						throw new UiJsonException(path, $"Image resource {resource.Id} names texture \"{key}\", but no texture lookup was supplied.");

					var texture = texture_lookup(key);
					if (texture == null || texture.Handle == IntPtr.Zero)
						throw new UiJsonException(path, $"The texture lookup has no texture for key \"{key}\".");

					var image = UiResourceDesc.Image(resource.Id, texture, resource.Name);
					builder.AddResource(resource with { Texture = image.Texture, ImageWidth = image.ImageWidth, ImageHeight = image.ImageHeight });
					break;
				}
				case EResourceKind.Colour:
				case EResourceKind.Font:
				{
					builder.AddResource(resource);
					break;
				}
				default:
				{
					throw new ArgumentOutOfRangeException(nameof(resource.Kind), resource.Kind, "Unknown resource kind.");
				}
			}
		}

		// Then styles, templates, and controls in parent-before-child order
		foreach (var style in Styles)
			builder.AddStyle(style);
		foreach (var template in Templates)
			builder.AddTemplate(template);
		foreach (var control in Controls)
			builder.Upsert(control);

		return builder;
	}

	/// <summary>Writer settings shared by every Serialize() call: indented for readability, with no cached state between calls.</summary>
	private static readonly JsonWriterOptions CanonicalWriterOptions = new() { Indented = true };

	/// <summary>
	/// Serialize this document to its canonical JSON form: a fixed declaration order for every property, every field
	/// written out explicitly (optional fields are never omitted just because a source document left them absent),
	/// colours always written as "#RRGGBBAA", and numbers written through Utf8JsonWriter's own invariant, round-trip
	/// formatting rather than manual string conversion. Two UiDocument instances that are property-wise equal (see
	/// Descriptors.cs equality) always serialize to byte-identical text, and re-parsing then re-serializing already
	/// canonical text reproduces it unchanged; this is the "canonical serialization" the JSON authoring format is built
	/// on, letting two differently-shaped-but-equivalent source documents be compared for equivalence as plain strings.
	/// </summary>
	public string Serialize()
	{
		using var stream = new MemoryStream();
		using (var writer = new Utf8JsonWriter(stream, CanonicalWriterOptions))
		{
			WriteRoot(writer);
		}
		return Encoding.UTF8.GetString(stream.ToArray());
	}

	/// <summary>Write the document root object: schema_version, then resources/styles/templates/tree in that fixed order.</summary>
	private void WriteRoot(Utf8JsonWriter writer)
	{
		writer.WriteStartObject();
		writer.WriteNumber("schema_version", SchemaVersion);

		writer.WriteStartArray("resources");
		foreach (var resource in Resources)
			WriteResource(writer, resource, TextureKeys.TryGetValue(resource.Id, out var key) ? key : string.Empty);
		writer.WriteEndArray();

		writer.WriteStartArray("styles");
		foreach (var style in Styles)
			WriteStyle(writer, style);
		writer.WriteEndArray();

		writer.WriteStartArray("templates");
		foreach (var template in Templates)
			WriteTemplate(writer, template);
		writer.WriteEndArray();

		WriteTree(writer);

		writer.WriteEndObject();
	}

	/// <summary>Reconstruct and write the nested "tree" array from the flat, pre-order Controls list, preserving each parent's original child order.</summary>
	private void WriteTree(Utf8JsonWriter writer)
	{
		// Controls are stored flattened in pre-order; grouping by ParentId reconstructs the nested shape "tree" represents in JSON.
		var children_by_parent = new Dictionary<ulong, List<UiControlDesc>>();
		foreach (var control in Controls)
		{
			if (!children_by_parent.TryGetValue(control.ParentId.m_value, out var siblings))
				children_by_parent[control.ParentId.m_value] = siblings = new List<UiControlDesc>();

			siblings.Add(control);
		}

		writer.WriteStartArray("tree");
		if (children_by_parent.TryGetValue(ControlId.None.m_value, out var roots))
		{
			foreach (var root in roots)
				WriteControlNode(writer, root, children_by_parent);
		}
		writer.WriteEndArray();
	}

	/// <summary>Write one control node and, recursively, its "children" array (always present, empty when there are none).</summary>
	private static void WriteControlNode(Utf8JsonWriter writer, UiControlDesc control, Dictionary<ulong, List<UiControlDesc>> children_by_parent)
	{
		writer.WriteStartObject();
		writer.WriteNumber("id", control.Id.m_value);
		writer.WriteString("type", control.Type.ToString());
		writer.WriteString("root_policy", control.RootPolicy.ToString());
		writer.WriteString("layout_mode", control.LayoutMode.ToString());
		writer.WriteNumber("style_id", control.StyleId.m_value);
		writer.WriteNumber("template_id", control.TemplateId.m_value);
		writer.WriteBoolean("enabled", control.Enabled);
		writer.WriteString("visibility", control.Visibility switch
		{
			EVisibility.Visible => "Visible",
			EVisibility.Hidden => "Hidden",
			EVisibility.Collapsed => "Collapsed",
			_ => throw new ArgumentOutOfRangeException(nameof(control.Visibility)),
		});
		writer.WriteBoolean("focusable", control.Focusable);
		writer.WriteString("validation_state", control.ValidationState.ToString());
		WriteLayout(writer, control.Layout);
		writer.WriteString("text", control.Text);
		writer.WriteString("name", control.Name);
		writer.WriteString("description", control.Description);
		writer.WriteNumber("max_text_length", control.MaxTextLength);
		writer.WriteNumber("font_resource_id", control.FontResourceId.m_value);
		writer.WriteBoolean("selected", control.Selected);
		writer.WriteNumber("value_sequence", control.ValueSequence);
		writer.WriteNumber("value", control.Value);
		writer.WriteBoolean("is_indeterminate", control.IsIndeterminate);
		writer.WriteNumber("minimum", control.Minimum);
		writer.WriteNumber("maximum", control.Maximum);
		writer.WriteNumber("step", control.Step);
		writer.WriteBoolean("masked", control.Masked);
		writer.WriteStartArray("items");
		foreach (var item in control.Items)
			writer.WriteStringValue(item);
		writer.WriteEndArray();
		writer.WriteNumber("selected_index", control.SelectedIndex);
		writer.WriteNumber("max_visible_items", control.MaxVisibleItems);
		writer.WriteString("orientation", control.Orientation.ToString());
		WriteWorld(writer, control.World);

		writer.WriteStartArray("children");
		if (children_by_parent.TryGetValue(control.Id.m_value, out var children))
		{
			foreach (var child in children)
				WriteControlNode(writer, child, children_by_parent);
		}
		writer.WriteEndArray();

		writer.WriteEndObject();
	}

	/// <summary>Write a control's nested "layout" object with every UiLayoutParams field present.</summary>
	private static void WriteLayout(Utf8JsonWriter writer, UiLayoutParams layout)
	{
		writer.WriteStartObject("layout");
		writer.WriteNumber("width", layout.Width);
		writer.WriteNumber("height", layout.Height);
		writer.WriteNumber("margin_left", layout.MarginLeft);
		writer.WriteNumber("margin_top", layout.MarginTop);
		writer.WriteNumber("margin_right", layout.MarginRight);
		writer.WriteNumber("margin_bottom", layout.MarginBottom);
		writer.WriteNumber("padding_left", layout.PaddingLeft);
		writer.WriteNumber("padding_top", layout.PaddingTop);
		writer.WriteNumber("padding_right", layout.PaddingRight);
		writer.WriteNumber("padding_bottom", layout.PaddingBottom);
		writer.WriteString("h_align", layout.HAlign.ToString());
		writer.WriteString("v_align", layout.VAlign.ToString());
		writer.WriteNumber("stack_spacing", layout.StackSpacing);
		writer.WriteNumber("canvas_x", layout.CanvasX);
		writer.WriteNumber("canvas_y", layout.CanvasY);
		writer.WriteNumber("scroll_offset_x", layout.ScrollOffsetX);
		writer.WriteNumber("scroll_offset_y", layout.ScrollOffsetY);
		writer.WriteNumber("h_fraction", layout.HFraction);
		writer.WriteNumber("v_fraction", layout.VFraction);
		writer.WriteEndObject();
	}

	/// <summary>Write a control's nested "world" object with every UiWorldRootParams field present.</summary>
	private static void WriteWorld(Utf8JsonWriter writer, UiWorldRootParams world)
	{
		writer.WriteStartObject("world");
		WriteVec3(writer, "anchor", world.Anchor);
		writer.WriteString("anchor_h", world.AnchorH.ToString());
		writer.WriteString("anchor_v", world.AnchorV.ToString());
		writer.WriteString("sizing", world.Sizing.ToString());
		writer.WriteNumber("world_units_per_dip", world.WorldUnitsPerDip);
		writer.WriteNumber("depth_offset", world.DepthOffset);
		writer.WriteNumber("occlusion_min_opacity", world.OcclusionMinOpacity);
		writer.WriteNumber("occlusion_fade_depth", world.OcclusionFadeDepth);
		writer.WriteNumber("occlusion_depth_bias", world.OcclusionDepthBias);
		writer.WriteEndObject();
	}

	/// <summary>Write one "resources" array entry with every UiResourceDesc field present. 'texture_key' is empty for non-Image resources.</summary>
	private static void WriteResource(Utf8JsonWriter writer, UiResourceDesc resource, string texture_key)
	{
		writer.WriteStartObject();
		writer.WriteNumber("id", resource.Id.m_value);
		writer.WriteString("kind", resource.Kind.ToString());
		writer.WriteString("colour", FormatColour(resource.Colour));
		writer.WriteString("name", resource.Name);
		writer.WriteNumber("font_size", resource.FontSize);
		writer.WriteString("texture", texture_key);
		writer.WriteEndObject();
	}

	/// <summary>Write one "styles" array entry, always emitting every EStateChannel key (in enum order) with its visual and transition present, whether or not the source document declared that channel.</summary>
	private static void WriteStyle(Utf8JsonWriter writer, UiStyleDesc style)
	{
		writer.WriteStartObject();
		writer.WriteNumber("id", style.Id.m_value);

		writer.WriteStartObject("states");
		for (var channel = EStateChannel.Normal; channel != EStateChannel.Count; ++channel)
		{
			writer.WriteStartObject(channel.ToString());

			var visual = style.GetVisual(channel);
			writer.WriteStartObject("visual");
			writer.WriteString("fill", FormatColour(visual.m_fill));
			writer.WriteString("border_colour", FormatColour(visual.m_border_colour));
			writer.WriteNumber("border_thickness", visual.m_border_thickness);
			writer.WriteNumber("corner_radius", visual.m_corner_radius);
			writer.WriteNumber("opacity", visual.m_opacity);
			writer.WriteString("foreground", FormatColour(visual.m_foreground));
			writer.WriteString("thumb", FormatColour(visual.m_thumb));
			WriteImageBrush(writer, visual.m_image);
			writer.WriteEndObject();

			var transition = style.GetTransition(channel);
			writer.WriteStartObject("transition");
			writer.WriteNumber("duration_ms", transition.m_duration_ms);
			writer.WriteString("easing", transition.m_easing.ToString());
			writer.WriteEndObject();

			writer.WriteEndObject();
		}
		writer.WriteEndObject();

		writer.WriteEndObject();
	}

	/// <summary>Write one "templates" array entry with its ordered role "parts" list.</summary>
	private static void WriteTemplate(Utf8JsonWriter writer, UiTemplateDesc template)
	{
		writer.WriteStartObject();
		writer.WriteNumber("id", template.Id.m_value);
		writer.WriteString("applies_to", template.AppliesTo.ToString());

		writer.WriteStartArray("parts");
		foreach (var part in template.Parts)
		{
			// Write every part field, including unused size and tick fields, so the form stays canonical
			writer.WriteStartObject();
			writer.WriteString("role", part.Role.ToString());
			writer.WriteNumber("style_id", part.StyleId.m_value);
			writer.WriteNumber("margin_left", part.MarginLeft);
			writer.WriteNumber("margin_top", part.MarginTop);
			writer.WriteNumber("margin_right", part.MarginRight);
			writer.WriteNumber("margin_bottom", part.MarginBottom);
			writer.WriteNumber("width", part.Width);
			writer.WriteNumber("height", part.Height);
			WriteTicks(writer, part.Ticks);
			writer.WriteEndObject();
		}
		writer.WriteEndArray();

		writer.WriteEndObject();
	}

	/// <summary>Write a style visual's nested "image" object with every ImageBrush field present.</summary>
	private static void WriteImageBrush(Utf8JsonWriter writer, ImageBrush image)
	{
		// Fixed field order; the source rectangle is a nested {x, y, w, h} object
		writer.WriteStartObject("image");
		writer.WriteNumber("image_id", image.m_image_id.m_value);
		writer.WriteStartObject("source");
		writer.WriteNumber("x", image.m_source.m_x);
		writer.WriteNumber("y", image.m_source.m_y);
		writer.WriteNumber("w", image.m_source.m_w);
		writer.WriteNumber("h", image.m_source.m_h);
		writer.WriteEndObject();
		writer.WriteNumber("slice_left", image.m_slice_left);
		writer.WriteNumber("slice_top", image.m_slice_top);
		writer.WriteNumber("slice_right", image.m_slice_right);
		writer.WriteNumber("slice_bottom", image.m_slice_bottom);
		writer.WriteNumber("slice_scale", image.m_slice_scale);
		writer.WriteString("tint", FormatColour(image.m_tint));
		writer.WriteEndObject();
	}

	/// <summary>Write a template part's nested "ticks" object with every TickParams field present.</summary>
	private static void WriteTicks(Utf8JsonWriter writer, TickParams ticks)
	{
		// Fixed field order
		writer.WriteStartObject("ticks");
		writer.WriteNumber("interval", ticks.m_interval);
		writer.WriteNumber("major_every", ticks.m_major_every);
		writer.WriteNumber("major_length", ticks.m_major_length);
		writer.WriteNumber("minor_length", ticks.m_minor_length);
		writer.WriteNumber("thickness", ticks.m_thickness);
		writer.WriteString("align", ticks.m_align.ToString());
		writer.WriteNumber("label_font_id", ticks.m_label_font_id.m_value);
		writer.WriteNumber("label_decimals", ticks.m_label_decimals);
		writer.WriteNumber("label_gap", ticks.m_label_gap);
		writer.WriteEndObject();
	}

	/// <summary>Write a {"x":.., "y":.., "z":..} object for one Vec3-valued property.</summary>
	private static void WriteVec3(Utf8JsonWriter writer, string property_name, Vec3 value)
	{
		writer.WriteStartObject(property_name);
		writer.WriteNumber("x", value.m_x);
		writer.WriteNumber("y", value.m_y);
		writer.WriteNumber("z", value.m_z);
		writer.WriteEndObject();
	}

	/// <summary>Format a colour as its canonical "#RRGGBBAA" hex string, always 8 digits (never the 6-digit RGB-only form ParseColour also accepts).</summary>
	private static string FormatColour(Colour colour)
	{
		return $"#{ToByte(colour.m_r):X2}{ToByte(colour.m_g):X2}{ToByte(colour.m_b):X2}{ToByte(colour.m_a):X2}";
	}

	/// <summary>Quantize one straight [0, 1] colour channel to its nearest byte, clamping out-of-range input rather than throwing.</summary>
	private static byte ToByte(float channel)
	{
		var clamped = channel < 0.0f ? 0.0f : channel > 1.0f ? 1.0f : channel;
		return (byte)(clamped * 255.0f + 0.5f);
	}

	/// <summary>Parse raw JSON text into a document tree, translating a malformed-JSON failure into a bounded UiJsonException.</summary>
	private static JsonDocument ParseDocument(string json)
	{
		try
		{
			return JsonDocument.Parse(json);
		}
		catch (JsonException ex)
		{
			throw new UiJsonException(ex.Path ?? "<root>", $"The document is not valid JSON: {ex.Message}", ex);
		}
	}

	/// <summary>Parse and validate the document root, producing a fully-resolved UiDocument.</summary>
	private static UiDocument ParseRoot(JsonElement root)
	{
		if (root.ValueKind != JsonValueKind.Object)
			throw new UiJsonException("$", $"Expected a JSON object at the document root, found {root.ValueKind}.");

		var schema_version = GetInt32(root, "schema_version", "$");
		if (schema_version != CurrentSchemaVersion)
			throw new UiJsonException("$.schema_version", $"Unsupported schema_version {schema_version}; this parser only accepts {CurrentSchemaVersion}.");

		// Resources, styles, and templates are parsed first because control nodes may reference their ids by value.
		// Each collection is parsed after the collections it may reference: styles name Image resources, and templates name styles and Font resources.
		var resource_entries = ParseArray(root, "resources", "$", ParseResource);
		var resources = resource_entries.Select(x => x.Resource).ToList();
		var resource_ids = ToIdSet(resources, r => r.Id.m_value, "$.resources", "resource");
		var resource_kinds = resources.ToDictionary(r => r.Id.m_value, r => r.Kind);
		var texture_keys = resource_entries.Where(x => x.Resource.Kind == EResourceKind.Image).ToDictionary(x => x.Resource.Id, x => x.TextureKey);

		var styles = ParseArray(root, "styles", "$", (element, path) => ParseStyle(element, path, resource_kinds));
		var style_ids = ToIdSet(styles, s => s.Id.m_value, "$.styles", "style");

		var templates = ParseArray(root, "templates", "$", (element, path) => ParseTemplate(element, path, style_ids, resource_kinds));
		var template_ids = ToIdSet(templates, t => t.Id.m_value, "$.templates", "template");

		var controls = new List<UiControlDesc>();
		var control_ids = new HashSet<ulong>();
		if (root.TryGetProperty("tree", out var tree_element))
		{
			if (tree_element.ValueKind != JsonValueKind.Array)
				throw new UiJsonException("$.tree", $"Expected an array, found {tree_element.ValueKind}.");

			var index = 0;
			foreach (var node in tree_element.EnumerateArray())
			{
				ParseControlNode(node, $"$.tree[{index}]", ControlId.None, controls, control_ids, resource_ids, style_ids, template_ids);
				++index;
			}
		}

		return new UiDocument(schema_version, resources, texture_keys, styles, templates, controls);
	}

	/// <summary>Recursively parse one control node and its "children", appending flattened UiControlDesc values in pre-order.</summary>
	private static void ParseControlNode(JsonElement node, string path, ControlId parent_id, List<UiControlDesc> controls, HashSet<ulong> control_ids, HashSet<ulong> resource_ids, HashSet<ulong> style_ids, HashSet<ulong> template_ids)
	{
		if (node.ValueKind != JsonValueKind.Object)
			throw new UiJsonException(path, $"Expected a JSON object, found {node.ValueKind}.");

		var id = new ControlId(GetUInt64(node, "id", path));
		if (id.IsNone)
			throw new UiJsonException($"{path}.id", "A control id of 0 (ControlId.None) is not a valid control identity.");
		if (!control_ids.Add(id.m_value))
			throw new UiJsonException($"{path}.id", $"Duplicate control id {id.m_value}; every control id in the document must be unique.");

		var style_id = TryGetUInt64(node, "style_id", out var style_value) ? new StyleId(style_value) : StyleId.None;
		if (!style_id.IsNone && !style_ids.Contains(style_id.m_value))
			throw new UiJsonException($"{path}.style_id", $"References style id {style_id.m_value}, which is not declared in this document's \"styles\" array.");

		var template_id = TryGetUInt64(node, "template_id", out var template_value) ? new TemplateId(template_value) : TemplateId.None;
		if (!template_id.IsNone && !template_ids.Contains(template_id.m_value))
			throw new UiJsonException($"{path}.template_id", $"References template id {template_id.m_value}, which is not declared in this document's \"templates\" array.");

		var font_resource_id = TryGetUInt64(node, "font_resource_id", out var font_value) ? new ResourceId(font_value) : ResourceId.None;
		if (!font_resource_id.IsNone && !resource_ids.Contains(font_resource_id.m_value))
			throw new UiJsonException($"{path}.font_resource_id", $"References resource id {font_resource_id.m_value}, which is not declared in this document's \"resources\" array.");

		// Reject the superseded boolean rather than silently treating an old hidden control as Visible.
		if (node.TryGetProperty("visible", out _))
			throw new UiJsonException($"{path}.visible", "Use visibility: Visible, Hidden, or Collapsed.");

		var control = new UiControlDesc
		{
			Id = id,
			ParentId = parent_id,
			Type = GetEnum<EControlType>(node, "type", path, required: true),
			RootPolicy = GetEnum<ERootPolicy>(node, "root_policy", path, required: false, ERootPolicy.Screen),
			LayoutMode = GetEnum<ELayoutMode>(node, "layout_mode", path, required: false, ELayoutMode.Overlay),
			TemplateId = template_id,
			StyleId = style_id,
			Enabled = GetBool(node, "enabled", path, true),
			Visibility = GetEnum<EVisibility>(node, "visibility", path, required: false, EVisibility.Visible),
			Focusable = GetBool(node, "focusable", path, false),
			ValidationState = GetEnum<EValidationState>(node, "validation_state", path, required: false, EValidationState.NotApplicable),
			Layout = ParseLayout(node, $"{path}.layout"),
			Text = GetString(node, "text", path, string.Empty),
			Name = GetString(node, "name", path, string.Empty),
			Description = GetString(node, "description", path, string.Empty),
			MaxTextLength = GetUInt32(node, "max_text_length", path, 0),
			FontResourceId = font_resource_id,
			Selected = GetBool(node, "selected", path, false),
			ValueSequence = GetUInt32(node, "value_sequence", path, 0),
			Value = GetFloat(node, "value", path, 0),
			IsIndeterminate = GetBool(node, "is_indeterminate", path, false),
			Minimum = GetFloat(node, "minimum", path, 0),
			Maximum = GetFloat(node, "maximum", path, 1),
			Step = GetFloat(node, "step", path, 0.1f),
			Masked = GetBool(node, "masked", path, false),
			Items = ParseStringArray(node, "items", path),
			SelectedIndex = TryGetInt32(node, "selected_index", out var selected_index) ? selected_index : -1,
			MaxVisibleItems = GetUInt32(node, "max_visible_items", path, 0),
			Orientation = GetEnum<EOrientation>(node, "orientation", path, required: false, EOrientation.Horizontal),
			World = ParseWorld(node, $"{path}.world"),
		};
		// Reject invalid completion at the authoring boundary, before constructing a transaction.
		switch (control.Type)
		{
			case EControlType.ProgressBar:
			{
				if (float.IsNaN(control.Value) || float.IsInfinity(control.Value) || control.Value < 0 || control.Value > 1)
					throw new UiJsonException($"{path}.value", "ProgressBar value must be finite and within [0, 1].");

				break;
			}
			case EControlType.Slider:
			{
				// Reject every invalid range relationship before a transaction can be constructed.
				if (float.IsNaN(control.Minimum) || float.IsInfinity(control.Minimum) || float.IsNaN(control.Maximum) || float.IsInfinity(control.Maximum) || control.Maximum <= control.Minimum)
					throw new UiJsonException($"{path}.maximum", "Slider bounds must be finite and Maximum must be greater than Minimum.");
				if (float.IsNaN(control.Value) || float.IsInfinity(control.Value) || control.Value < control.Minimum || control.Value > control.Maximum)
					throw new UiJsonException($"{path}.value", "Slider value must be finite and within [Minimum, Maximum].");
				if (float.IsNaN(control.Step) || float.IsInfinity(control.Step) || control.Step <= 0 || control.Step > control.Maximum - control.Minimum)
					throw new UiJsonException($"{path}.step", "Slider step must be finite, positive, and no greater than Maximum - Minimum.");

				break;
			}
			case EControlType.ComboBox:
			{
				if (control.MaxVisibleItems == 0)
					throw new UiJsonException($"{path}.max_visible_items", "ComboBox max_visible_items must be greater than zero.");
				if (control.SelectedIndex < -1 || control.SelectedIndex >= control.Items.Count)
					throw new UiJsonException($"{path}.selected_index", "ComboBox selected_index must be -1 or within the items array.");

				break;
			}
			case EControlType.Root:
			case EControlType.Panel:
			case EControlType.Text:
			case EControlType.TextBox:
			case EControlType.Button: { break; }
			default: { throw new UiJsonException($"{path}.type", "Unknown control type."); }
		}
		controls.Add(control);

		if (node.TryGetProperty("children", out var children_element))
		{
			if (children_element.ValueKind != JsonValueKind.Array)
				throw new UiJsonException($"{path}.children", $"Expected an array, found {children_element.ValueKind}.");

			var index = 0;
			foreach (var child in children_element.EnumerateArray())
			{
				ParseControlNode(child, $"{path}.children[{index}]", id, controls, control_ids, resource_ids, style_ids, template_ids);
				++index;
			}
		}
	}

	/// <summary>Parse an optional "layout" object into a UiLayoutParams, defaulting every field when the object (or the whole property) is absent.</summary>
	private static UiLayoutParams ParseLayout(JsonElement parent, string path)
	{
		var layout = new UiLayoutParams();
		if (!parent.TryGetProperty("layout", out var element))
			return layout;
		if (element.ValueKind != JsonValueKind.Object)
			throw new UiJsonException(path, $"Expected a JSON object, found {element.ValueKind}.");

		layout.Width = GetFloat(element, "width", path, 0);
		layout.Height = GetFloat(element, "height", path, 0);
		layout.MarginLeft = GetFloat(element, "margin_left", path, 0);
		layout.MarginTop = GetFloat(element, "margin_top", path, 0);
		layout.MarginRight = GetFloat(element, "margin_right", path, 0);
		layout.MarginBottom = GetFloat(element, "margin_bottom", path, 0);
		layout.PaddingLeft = GetFloat(element, "padding_left", path, 0);
		layout.PaddingTop = GetFloat(element, "padding_top", path, 0);
		layout.PaddingRight = GetFloat(element, "padding_right", path, 0);
		layout.PaddingBottom = GetFloat(element, "padding_bottom", path, 0);
		layout.HAlign = GetEnum<EHAlign>(element, "h_align", path, required: false, EHAlign.Stretch);
		layout.VAlign = GetEnum<EVAlign>(element, "v_align", path, required: false, EVAlign.Stretch);
		layout.StackSpacing = GetFloat(element, "stack_spacing", path, 0);
		layout.CanvasX = GetFloat(element, "canvas_x", path, 0);
		layout.CanvasY = GetFloat(element, "canvas_y", path, 0);
		layout.ScrollOffsetX = GetFloat(element, "scroll_offset_x", path, 0);
		layout.ScrollOffsetY = GetFloat(element, "scroll_offset_y", path, 0);
		layout.HFraction = GetFloat(element, "h_fraction", path, 0);
		layout.VFraction = GetFloat(element, "v_fraction", path, 0);
		return layout;
	}

	/// <summary>Parse an optional "world" object into a UiWorldRootParams, defaulting every field when the object (or the whole property) is absent.</summary>
	private static UiWorldRootParams ParseWorld(JsonElement parent, string path)
	{
		var world = new UiWorldRootParams();
		if (!parent.TryGetProperty("world", out var element))
			return world;
		if (element.ValueKind != JsonValueKind.Object)
			throw new UiJsonException(path, $"Expected a JSON object, found {element.ValueKind}.");

		world.Anchor = ParseVec3(element, "anchor", path, Vec3.Zero);
		world.AnchorH = GetEnum<EAnchorPoint>(element, "anchor_h", path, required: false, EAnchorPoint.Centre);
		world.AnchorV = GetEnum<EAnchorPoint>(element, "anchor_v", path, required: false, EAnchorPoint.Centre);
		world.Sizing = GetEnum<EWorldSizing>(element, "sizing", path, required: false, EWorldSizing.WorldUnits);
		world.WorldUnitsPerDip = GetFloat(element, "world_units_per_dip", path, 1.0f);
		world.DepthOffset = GetFloat(element, "depth_offset", path, 0);
		world.OcclusionMinOpacity = GetFloat(element, "occlusion_min_opacity", path, 1.0f);
		world.OcclusionFadeDepth = GetFloat(element, "occlusion_fade_depth", path, 1.0f);
		world.OcclusionDepthBias = GetFloat(element, "occlusion_depth_bias", path, 0);
		return world;
	}

	/// <summary>
	/// Parse one "resources" array entry into a UiResourceDesc and its texture key. Image resources must name a texture key;
	/// other kinds must not. The texture handle and size are left empty for ToTransactionBuilder to resolve.
	/// </summary>
	private static (UiResourceDesc Resource, string TextureKey) ParseResource(JsonElement element, string path)
	{
		// Read the shared fields
		var id = new ResourceId(GetUInt64(element, "id", path));
		if (id.IsNone)
			throw new UiJsonException($"{path}.id", "A resource id of 0 (ResourceId.None) is not a valid resource identity.");

		var resource = new UiResourceDesc
		{
			Id = id,
			Kind = GetEnum<EResourceKind>(element, "kind", path, required: true),
			Colour = ParseColour(element, "colour", path, Colour.TransparentBlack),
			Name = GetString(element, "name", path, string.Empty),
			FontSize = GetFloat(element, "font_size", path, 0),
		};

		// Only Image resources name a texture, because JSON cannot hold a texture handle
		var texture_key = GetString(element, "texture", path, string.Empty);
		switch (resource.Kind)
		{
			case EResourceKind.Image:
			{
				if (texture_key.Length == 0)
					throw new UiJsonException($"{path}.texture", "An Image resource must name its texture with a non-empty \"texture\" key.");

				break;
			}
			case EResourceKind.Colour:
			case EResourceKind.Font:
			{
				if (texture_key.Length != 0)
					throw new UiJsonException($"{path}.texture", $"Only Image resources may name a texture; this resource is {resource.Kind}.");

				break;
			}
			default:
			{
				throw new UiJsonException($"{path}.kind", "Unknown resource kind.");
			}
		}
		return (resource, texture_key);
	}

	/// <summary>
	/// Parse one "styles" array entry into a UiStyleDesc, resolving each declared per-channel state. Undeclared channels keep StyleVisual.Empty.
	/// 'resource_kinds' maps every declared resource id to its kind, so image brushes can be checked against Image resources.
	/// </summary>
	private static UiStyleDesc ParseStyle(JsonElement element, string path, IReadOnlyDictionary<ulong, EResourceKind> resource_kinds)
	{
		var id = new StyleId(GetUInt64(element, "id", path));
		if (id.IsNone)
			throw new UiJsonException($"{path}.id", "A style id of 0 (StyleId.None) is not a valid style identity.");

		var style = new UiStyleDesc { Id = id };
		if (!element.TryGetProperty("states", out var states_element))
			return style;
		if (states_element.ValueKind != JsonValueKind.Object)
			throw new UiJsonException($"{path}.states", $"Expected a JSON object keyed by state channel name, found {states_element.ValueKind}.");

		foreach (var state_property in states_element.EnumerateObject())
		{
			var state_path = $"{path}.states.{state_property.Name}";
			var channel = ParseEnumName<EStateChannel>(state_property.Name, state_path);
			var state = state_property.Value;
			if (state.ValueKind != JsonValueKind.Object)
				throw new UiJsonException(state_path, $"Expected a JSON object, found {state.ValueKind}.");

			if (state.TryGetProperty("visual", out var visual_element))
			{
				var visual_path = $"{state_path}.visual";
				if (visual_element.ValueKind != JsonValueKind.Object)
					throw new UiJsonException(visual_path, $"Expected a JSON object, found {visual_element.ValueKind}.");

				style.SetVisual(channel, new StyleVisual(
					fill: ParseColour(visual_element, "fill", visual_path, Colour.TransparentBlack),
					border_colour: ParseColour(visual_element, "border_colour", visual_path, Colour.TransparentBlack),
					border_thickness: GetFloat(visual_element, "border_thickness", visual_path, 0),
					corner_radius: GetFloat(visual_element, "corner_radius", visual_path, 0),
					opacity: GetFloat(visual_element, "opacity", visual_path, 1),
					foreground: ParseColour(visual_element, "foreground", visual_path, Colour.TransparentBlack),
					thumb: ParseColour(visual_element, "thumb", visual_path, Colour.TransparentBlack),
					image: ParseImageBrush(visual_element, $"{visual_path}.image", resource_kinds)
				));
			}
			if (state.TryGetProperty("transition", out var transition_element))
			{
				var transition_path = $"{state_path}.transition";
				if (transition_element.ValueKind != JsonValueKind.Object)
					throw new UiJsonException(transition_path, $"Expected a JSON object, found {transition_element.ValueKind}.");

				style.SetTransition(channel, new TransitionDesc(
					duration_ms: GetFloat(transition_element, "duration_ms", transition_path, 0),
					easing: GetEnum<EEasing>(transition_element, "easing", transition_path, required: false, EEasing.Linear)));
			}
		}
		return style;
	}

	/// <summary>
	/// Parse an optional "image" object into an ImageBrush. An absent object means no image. A present object needs an "image_id" that is
	/// 0 (no image) or names an Image resource; its "tint" defaults to opaque white so the image draws unchanged.
	/// </summary>
	private static ImageBrush ParseImageBrush(JsonElement parent, string path, IReadOnlyDictionary<ulong, EResourceKind> resource_kinds)
	{
		// An absent image means the style draws no image
		if (!parent.TryGetProperty("image", out var element))
			return ImageBrush.None;
		if (element.ValueKind != JsonValueKind.Object)
			throw new UiJsonException(path, $"Expected a JSON object, found {element.ValueKind}.");

		// The image id must name an Image resource declared earlier in this document
		var image_id = new ResourceId(GetUInt64(element, "image_id", path));
		if (!image_id.IsNone && (!resource_kinds.TryGetValue(image_id.m_value, out var kind) || kind != EResourceKind.Image))
			throw new UiJsonException($"{path}.image_id", $"References resource id {image_id.m_value}, which is not an Image resource declared in this document's \"resources\" array.");

		// The source rectangle is optional; zero size means the whole image
		var source = Rect.Zero;
		if (element.TryGetProperty("source", out var source_element))
		{
			var source_path = $"{path}.source";
			if (source_element.ValueKind != JsonValueKind.Object)
				throw new UiJsonException(source_path, $"Expected a JSON object, found {source_element.ValueKind}.");

			source = new Rect(GetFloat(source_element, "x", source_path, 0), GetFloat(source_element, "y", source_path, 0), GetFloat(source_element, "w", source_path, 0), GetFloat(source_element, "h", source_path, 0));
		}

		// Read the nine-slice insets and tint
		return new ImageBrush(
			image_id,
			source,
			slice_left: GetFloat(element, "slice_left", path, 0),
			slice_top: GetFloat(element, "slice_top", path, 0),
			slice_right: GetFloat(element, "slice_right", path, 0),
			slice_bottom: GetFloat(element, "slice_bottom", path, 0),
			slice_scale: GetFloat(element, "slice_scale", path, 0),
			tint: ParseColour(element, "tint", path, Colour.White)
		);
	}

	/// <summary>
	/// Parse one "templates" array entry into a UiTemplateDesc, enforcing the UiTemplateDesc.MaxParts cap. 'style_ids' and 'resource_kinds'
	/// hold the declared styles and resources, so part styles and tick label fonts can be checked.
	/// </summary>
	private static UiTemplateDesc ParseTemplate(JsonElement element, string path, HashSet<ulong> style_ids, IReadOnlyDictionary<ulong, EResourceKind> resource_kinds)
	{
		var id = new TemplateId(GetUInt64(element, "id", path));
		if (id.IsNone)
			throw new UiJsonException($"{path}.id", "A template id of 0 (TemplateId.None) is not a valid template identity.");

		var template = new UiTemplateDesc
		{
			Id = id,
			AppliesTo = GetEnum<EControlType>(element, "applies_to", path, required: true),
		};

		if (element.TryGetProperty("parts", out var parts_element))
		{
			if (parts_element.ValueKind != JsonValueKind.Array)
				throw new UiJsonException($"{path}.parts", $"Expected an array, found {parts_element.ValueKind}.");

			var index = 0;
			foreach (var part_element in parts_element.EnumerateArray())
			{
				var part_path = $"{path}.parts[{index}]";
				if (part_element.ValueKind != JsonValueKind.Object)
					throw new UiJsonException(part_path, $"Expected a JSON object, found {part_element.ValueKind}.");
				if (template.Parts.Count >= UiTemplateDesc.MaxParts)
					throw new UiJsonException(part_path, $"Template already has the maximum of {UiTemplateDesc.MaxParts} parts (VIEW3D_UI_MAX_TEMPLATE_PARTS).");

				// A part's style is optional, but when given it must be declared in this document
				var style_id = TryGetUInt64(part_element, "style_id", out var style_value) ? new StyleId(style_value) : StyleId.None;
				if (!style_id.IsNone && !style_ids.Contains(style_id.m_value))
					throw new UiJsonException($"{part_path}.style_id", $"References style id {style_id.m_value}, which is not declared in this document's \"styles\" array.");

				template.AddPart(new UiTemplatePart(
					GetEnum<ETemplatePart>(part_element, "role", part_path, required: true),
					style_id,
					margin_left: GetFloat(part_element, "margin_left", part_path, 0),
					margin_top: GetFloat(part_element, "margin_top", part_path, 0),
					margin_right: GetFloat(part_element, "margin_right", part_path, 0),
					margin_bottom: GetFloat(part_element, "margin_bottom", part_path, 0),
					width: GetFloat(part_element, "width", part_path, 0),
					height: GetFloat(part_element, "height", part_path, 0),
					ticks: ParseTicks(part_element, $"{part_path}.ticks", resource_kinds)
				));
				++index;
			}
		}
		return template;
	}

	/// <summary>
	/// Parse an optional "ticks" object into TickParams, defaulting every field when the object is absent. A non-zero "label_font_id"
	/// must name a Font resource.
	/// </summary>
	private static TickParams ParseTicks(JsonElement parent, string path, IReadOnlyDictionary<ulong, EResourceKind> resource_kinds)
	{
		// Absent ticks mean the part draws no ticks
		if (!parent.TryGetProperty("ticks", out var element))
			return default;
		if (element.ValueKind != JsonValueKind.Object)
			throw new UiJsonException(path, $"Expected a JSON object, found {element.ValueKind}.");

		// Tick labels need a Font resource declared in this document
		var label_font_id = TryGetUInt64(element, "label_font_id", out var font_value) ? new ResourceId(font_value) : ResourceId.None;
		if (!label_font_id.IsNone && (!resource_kinds.TryGetValue(label_font_id.m_value, out var kind) || kind != EResourceKind.Font))
			throw new UiJsonException($"{path}.label_font_id", $"References resource id {label_font_id.m_value}, which is not a Font resource declared in this document's \"resources\" array.");

		// Read the tick spacing, sizes, and label settings
		return new TickParams(
			interval: GetFloat(element, "interval", path, 0),
			major_every: GetUInt32(element, "major_every", path, 0),
			major_length: GetFloat(element, "major_length", path, 0),
			minor_length: GetFloat(element, "minor_length", path, 0),
			thickness: GetFloat(element, "thickness", path, 0),
			align: GetEnum<EAnchorPoint>(element, "align", path, required: false, EAnchorPoint.Min),
			label_font_id: label_font_id,
			label_decimals: TryGetInt32(element, "label_decimals", out var label_decimals) ? label_decimals : 0,
			label_gap: GetFloat(element, "label_gap", path, 0)
		);
	}

	/// <summary>Parse a JSON array property into a list, defaulting to empty when the property is absent.</summary>
	private static List<T> ParseArray<T>(JsonElement parent, string property_name, string parent_path, Func<JsonElement, string, T> parse_element)
	{
		var result = new List<T>();
		if (!parent.TryGetProperty(property_name, out var array_element))
			return result;
		if (array_element.ValueKind != JsonValueKind.Array)
			throw new UiJsonException($"{parent_path}.{property_name}", $"Expected an array, found {array_element.ValueKind}.");

		var index = 0;
		foreach (var element in array_element.EnumerateArray())
		{
			result.Add(parse_element(element, $"{parent_path}.{property_name}[{index}]"));
			++index;
		}
		return result;
	}

	/// <summary>Parse an optional string array property, rejecting non-string elements with their exact path.</summary>
	private static List<string> ParseStringArray(JsonElement parent, string property_name, string parent_path)
	{
		return ParseArray(parent, property_name, parent_path, (element, path) =>
		{
			if (element.ValueKind != JsonValueKind.String)
				throw new UiJsonException(path, $"Expected a string, found {element.ValueKind}.");

			return element.GetString() ?? string.Empty;
		});
	}

	/// <summary>Build a duplicate-checked set of ids for one closed collection (resources/styles/templates), reporting the first duplicate found.</summary>
	private static HashSet<ulong> ToIdSet<T>(IReadOnlyList<T> items, Func<T, ulong> get_id, string collection_path, string kind_name)
	{
		var ids = new HashSet<ulong>();
		for (var i = 0; i != items.Count; ++i)
		{
			var id = get_id(items[i]);
			if (!ids.Add(id))
				throw new UiJsonException($"{collection_path}[{i}].id", $"Duplicate {kind_name} id {id}; every {kind_name} id in the document must be unique.");
		}
		return ids;
	}

	/// <summary>Parse a required int32 property.</summary>
	private static int GetInt32(JsonElement element, string property_name, string path)
	{
		if (!element.TryGetProperty(property_name, out var value))
			throw new UiJsonException(path, $"Missing required property \"{property_name}\".");
		if (value.ValueKind != JsonValueKind.Number || !value.TryGetInt32(out var result))
			throw new UiJsonException($"{path}.{property_name}", $"Expected an integer, found {value}.");

		return result;
	}

	/// <summary>Try to parse an optional int32 property.</summary>
	private static bool TryGetInt32(JsonElement element, string property_name, out int value)
	{
		value = 0;
		return element.TryGetProperty(property_name, out var json_value) && json_value.ValueKind == JsonValueKind.Number && json_value.TryGetInt32(out value);
	}

	/// <summary>Parse a required uint64 property (used for every stable id).</summary>
	private static ulong GetUInt64(JsonElement element, string property_name, string path)
	{
		if (!element.TryGetProperty(property_name, out var value))
			throw new UiJsonException($"{path}.{property_name}", "Missing required property.");
		if (value.ValueKind != JsonValueKind.Number || !value.TryGetUInt64(out var result))
			throw new UiJsonException($"{path}.{property_name}", $"Expected a non-negative integer, found {value}.");

		return result;
	}

	/// <summary>Try to parse an optional uint64 property (used for optional id references, which default to the "None" sentinel).</summary>
	private static bool TryGetUInt64(JsonElement element, string property_name, out ulong result)
	{
		if (!element.TryGetProperty(property_name, out var value) || value.ValueKind == JsonValueKind.Null)
		{
			result = 0;
			return false;
		}
		if (value.ValueKind != JsonValueKind.Number || !value.TryGetUInt64(out result))
			throw new UiJsonException($"{property_name}", $"Expected a non-negative integer, found {value}.");

		return result != 0;
	}

	/// <summary>Parse an optional uint32 property, defaulting when absent.</summary>
	private static uint GetUInt32(JsonElement element, string property_name, string path, uint default_value)
	{
		if (!element.TryGetProperty(property_name, out var value))
			return default_value;
		if (value.ValueKind != JsonValueKind.Number || !value.TryGetUInt32(out var result))
			throw new UiJsonException($"{path}.{property_name}", $"Expected a non-negative integer, found {value}.");

		return result;
	}

	/// <summary>Parse an optional float property, defaulting when absent.</summary>
	private static float GetFloat(JsonElement element, string property_name, string path, float default_value)
	{
		if (!element.TryGetProperty(property_name, out var value))
			return default_value;
		if (value.ValueKind != JsonValueKind.Number || !value.TryGetSingle(out var result) || float.IsNaN(result) || float.IsInfinity(result))
			throw new UiJsonException($"{path}.{property_name}", $"Expected a finite number, found {value}.");

		return result;
	}

	/// <summary>Parse an optional bool property, defaulting when absent.</summary>
	private static bool GetBool(JsonElement element, string property_name, string path, bool default_value)
	{
		if (!element.TryGetProperty(property_name, out var value))
			return default_value;
		if (value.ValueKind != JsonValueKind.True && value.ValueKind != JsonValueKind.False)
			throw new UiJsonException($"{path}.{property_name}", $"Expected a boolean, found {value}.");

		return value.GetBoolean();
	}

	/// <summary>Parse an optional string property, defaulting when absent.</summary>
	private static string GetString(JsonElement element, string property_name, string path, string default_value)
	{
		if (!element.TryGetProperty(property_name, out var value))
			return default_value;
		if (value.ValueKind != JsonValueKind.String)
			throw new UiJsonException($"{path}.{property_name}", $"Expected a string, found {value.ValueKind}.");

		return value.GetString() ?? string.Empty;
	}

	/// <summary>Parse a required or defaulted closed-vocabulary enum property from its snake_case or PascalCase JSON string.</summary>
	private static TEnum GetEnum<TEnum>(JsonElement element, string property_name, string path, bool required, TEnum default_value = default) where TEnum : struct, Enum
	{
		if (!element.TryGetProperty(property_name, out var value))
		{
			if (required)
				throw new UiJsonException(path, $"Missing required property \"{property_name}\".");

			return default_value;
		}
		if (value.ValueKind != JsonValueKind.String)
			throw new UiJsonException($"{path}.{property_name}", $"Expected a string, found {value.ValueKind}.");

		return ParseEnumName<TEnum>(value.GetString() ?? string.Empty, $"{path}.{property_name}");
	}

	/// <summary>Match 'text' against the closed vocabulary of TEnum, accepting snake_case or PascalCase and ignoring case; rejects the "Count" sentinel.</summary>
	private static TEnum ParseEnumName<TEnum>(string text, string path) where TEnum : struct, Enum
	{
		if (string.IsNullOrWhiteSpace(text))
			throw new UiJsonException(path, "A value is required.");

		var normalized = text.Replace("_", string.Empty);
		foreach (var name in Enum.GetNames(typeof(TEnum)))
		{
			if (string.Equals(name, "Count", StringComparison.Ordinal))
				continue; // Count is a closed-vocabulary bound, not an authorable value.
			if (string.Equals(name, normalized, StringComparison.OrdinalIgnoreCase))
				return (TEnum)Enum.Parse(typeof(TEnum), name);
		}
		throw new UiJsonException(path, $"Unknown {typeof(TEnum).Name} value \"{text}\".");
	}

	/// <summary>Parse an optional "#RRGGBB"/"#RRGGBBAA" hex colour string property, defaulting when absent.</summary>
	private static Colour ParseColour(JsonElement parent, string property_name, string path, Colour default_value)
	{
		if (!parent.TryGetProperty(property_name, out var value))
			return default_value;
		if (value.ValueKind != JsonValueKind.String)
			throw new UiJsonException($"{path}.{property_name}", $"Expected a \"#RRGGBB\" or \"#RRGGBBAA\" hex colour string, found {value.ValueKind}.");

		var text = value.GetString() ?? string.Empty;
		if (text.Length != 7 && text.Length != 9 || text[0] != '#')
			throw new UiJsonException($"{path}.{property_name}", $"\"{text}\" is not a \"#RRGGBB\" or \"#RRGGBBAA\" hex colour string.");

		try
		{
			var r = byte.Parse(text.Substring(1, 2), NumberStyles.HexNumber, CultureInfo.InvariantCulture);
			var g = byte.Parse(text.Substring(3, 2), NumberStyles.HexNumber, CultureInfo.InvariantCulture);
			var b = byte.Parse(text.Substring(5, 2), NumberStyles.HexNumber, CultureInfo.InvariantCulture);
			var a = text.Length == 9 ? byte.Parse(text.Substring(7, 2), NumberStyles.HexNumber, CultureInfo.InvariantCulture) : (byte)255;
			return new Colour(r / 255.0f, g / 255.0f, b / 255.0f, a / 255.0f);
		}
		catch (FormatException ex)
		{
			throw new UiJsonException($"{path}.{property_name}", $"\"{text}\" is not a valid hex colour string.", ex);
		}
	}

	/// <summary>Parse an optional {"x":.., "y":.., "z":..} object property into a Vec3, defaulting when absent.</summary>
	private static Vec3 ParseVec3(JsonElement parent, string property_name, string path, Vec3 default_value)
	{
		if (!parent.TryGetProperty(property_name, out var value))
			return default_value;
		if (value.ValueKind != JsonValueKind.Object)
			throw new UiJsonException($"{path}.{property_name}", $"Expected a {{\"x\":.., \"y\":.., \"z\":..}} object, found {value.ValueKind}.");

		var vec_path = $"{path}.{property_name}";
		return new Vec3(GetFloat(value, "x", vec_path, 0), GetFloat(value, "y", vec_path, 0), GetFloat(value, "z", vec_path, 0));
	}
}
