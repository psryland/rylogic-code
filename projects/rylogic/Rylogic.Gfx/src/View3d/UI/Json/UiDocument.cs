using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Text;
using System.Text.Json;
using Rylogic.Gfx.UI;

namespace Rylogic.Gfx.UI.Json;

/// <summary>
/// A parsed and fully-validated View3DUI JSON document (schema version 1): the same closed-vocabulary resource, style,
/// template, and control-tree descriptors a runtime application would construct directly via Descriptors.cs and
/// UiTransactionBuilder, produced instead from a versioned JSON authoring format. Parsing performs every validation
/// this layer is responsible for - malformed JSON, unknown schema versions, unknown closed-vocabulary kinds, and
/// dangling id references - before any native call is made; the native DLL never sees the original JSON text.
/// </summary>
public sealed class UiDocument
{
	/// <summary>The only schema_version this parser currently accepts.</summary>
	public const int CurrentSchemaVersion = 1;

	/// <summary>The schema_version declared by the parsed document.</summary>
	public int SchemaVersion { get; }

	/// <summary>Every resource declared by the document, in declaration order.</summary>
	public IReadOnlyList<UiResourceDesc> Resources { get; }

	/// <summary>Every style declared by the document, in declaration order.</summary>
	public IReadOnlyList<UiStyleDesc> Styles { get; }

	/// <summary>Every template declared by the document, in declaration order.</summary>
	public IReadOnlyList<UiTemplateDesc> Templates { get; }

	/// <summary>Every control declared by the document's "tree" section, flattened into parent-before-child (pre-order) order with ParentId already resolved from JSON nesting.</summary>
	public IReadOnlyList<UiControlDesc> Controls { get; }

	/// <summary>Adopt an already-validated set of descriptors.</summary>
	private UiDocument(int schema_version, IReadOnlyList<UiResourceDesc> resources, IReadOnlyList<UiStyleDesc> styles, IReadOnlyList<UiTemplateDesc> templates, IReadOnlyList<UiControlDesc> controls)
	{
		SchemaVersion = schema_version;
		Resources = resources;
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
	/// controls that reference them upsert cleanly), then controls in parent-before-child order.
	/// </summary>
	public UiTransactionBuilder ToTransactionBuilder()
	{
		var builder = new UiTransactionBuilder();
		foreach (var resource in Resources)
			builder.AddResource(resource);
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
			WriteResource(writer, resource);
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
		writer.WriteBoolean("visible", control.Visible);
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

	/// <summary>Write one "resources" array entry with every UiResourceDesc field present.</summary>
	private static void WriteResource(Utf8JsonWriter writer, UiResourceDesc resource)
	{
		writer.WriteStartObject();
		writer.WriteNumber("id", resource.Id.m_value);
		writer.WriteString("kind", resource.Kind.ToString());
		writer.WriteString("colour", FormatColour(resource.Colour));
		writer.WriteString("name", resource.Name);
		writer.WriteNumber("font_size", resource.FontSize);
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

	/// <summary>Write one "templates" array entry with its ordered "parts" list.</summary>
	private static void WriteTemplate(Utf8JsonWriter writer, UiTemplateDesc template)
	{
		writer.WriteStartObject();
		writer.WriteNumber("id", template.Id.m_value);
		writer.WriteString("applies_to", template.AppliesTo.ToString());

		writer.WriteStartArray("parts");
		foreach (var part in template.Parts)
		{
			writer.WriteStartObject();
			writer.WriteString("name", part.Name);
			writer.WriteString("primitive", part.Primitive.ToString());
			writer.WriteBoolean("required", part.Required);
			writer.WriteEndObject();
		}
		writer.WriteEndArray();

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
		var resources = ParseArray(root, "resources", "$", ParseResource);
		var styles = ParseArray(root, "styles", "$", ParseStyle);
		var templates = ParseArray(root, "templates", "$", ParseTemplate);

		var resource_ids = ToIdSet(resources, r => r.Id.m_value, "$.resources", "resource");
		var style_ids = ToIdSet(styles, s => s.Id.m_value, "$.styles", "style");
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

		return new UiDocument(schema_version, resources, styles, templates, controls);
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
			Visible = GetBool(node, "visible", path, true),
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
			World = ParseWorld(node, $"{path}.world"),
		};
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

	/// <summary>Parse one "resources" array entry into a UiResourceDesc.</summary>
	private static UiResourceDesc ParseResource(JsonElement element, string path)
	{
		var id = new ResourceId(GetUInt64(element, "id", path));
		if (id.IsNone)
			throw new UiJsonException($"{path}.id", "A resource id of 0 (ResourceId.None) is not a valid resource identity.");

		return new UiResourceDesc
		{
			Id = id,
			Kind = GetEnum<EResourceKind>(element, "kind", path, required: true),
			Colour = ParseColour(element, "colour", path, Colour.TransparentBlack),
			Name = GetString(element, "name", path, string.Empty),
			FontSize = GetFloat(element, "font_size", path, 0),
		};
	}

	/// <summary>Parse one "styles" array entry into a UiStyleDesc, resolving each declared per-channel state.</summary>
	private static UiStyleDesc ParseStyle(JsonElement element, string path)
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
					opacity: GetFloat(visual_element, "opacity", visual_path, 1)));
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

	/// <summary>Parse one "templates" array entry into a UiTemplateDesc, enforcing the UiTemplateDesc.MaxParts cap.</summary>
	private static UiTemplateDesc ParseTemplate(JsonElement element, string path)
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

				var name = GetRequiredString(part_element, "name", part_path);
				var primitive = GetEnum<EVisualPrimitive>(part_element, "primitive", part_path, required: true);
				var required = GetBool(part_element, "required", part_path, true);
				template.AddPart(name, primitive, required);
				++index;
			}
		}
		return template;
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

	/// <summary>Parse a required string property.</summary>
	private static string GetRequiredString(JsonElement element, string property_name, string path)
	{
		if (!element.TryGetProperty(property_name, out var value))
			throw new UiJsonException(path, $"Missing required property \"{property_name}\".");
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
