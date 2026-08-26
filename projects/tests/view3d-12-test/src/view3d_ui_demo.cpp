//*********************************************
// View3d 12 Test
//  Copyright (c) Rylogic Ltd 2026
//*********************************************
#include "view3d_ui_demo.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <string_view>
#include <vector>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include "pr/common/cast.h"
#include "pr/common/min_max_fix.h"
#include "pr/gui/wingui.h"
#include "pr/view3d-ui/view3d-ui.h"

namespace view3d_test
{
	namespace
	{
		using namespace pr::view3d::ui;
		using UIColour = pr::view3d::ui::Colour;

		// Stable per-control ids for the retained screen-space gallery tree.
		inline constexpr ControlId UI_Root = 1;
		inline constexpr ControlId UI_Panel = 2;
		inline constexpr ControlId UI_Label = 3;
		inline constexpr ControlId UI_Dimension = 4;
		inline constexpr ControlId UI_Update = 5;
		inline constexpr ControlId UI_Title = 6;
		inline constexpr ControlId UI_DimensionSection = 7;
		inline constexpr ControlId UI_DimensionRow = 8;
		inline constexpr ControlId UI_ButtonLabel = 9;
		inline constexpr ControlId UI_ButtonRow = 10;
		inline constexpr ControlId UI_SoftButton = 11;
		inline constexpr ControlId UI_PillButton = 12;
		inline constexpr ControlId UI_DisabledButton = 13;
		inline constexpr ControlId UI_OverlayPanel = 14;
		inline constexpr ControlId UI_OverlayText = 15;
		inline constexpr ControlId UI_OverlayButton = 16;
		inline constexpr ControlId UI_CanvasPanel = 17;
		inline constexpr ControlId UI_CanvasText = 18;
		inline constexpr ControlId UI_CanvasButton = 19;
		inline constexpr ControlId UI_ScrollPanel = 20;
		inline constexpr ControlId UI_ScrollText = 21;
		inline constexpr ControlId UI_Status = 22;

		// Stable per-control ids for the three clickable world-anchored roots.
		inline constexpr ControlId UI_WorldOverlayRoot = 40;
		inline constexpr ControlId UI_WorldOverlayButton = 41;
		inline constexpr ControlId UI_WorldDepthRoot = 42;
		inline constexpr ControlId UI_WorldDepthButton = 43;
		inline constexpr ControlId UI_WorldFadeRoot = 44;
		inline constexpr ControlId UI_WorldFadeButton = 45;

		// Stable resource/style/template ids for the gallery's look and lookless templates.
		inline constexpr ResourceId UI_Font = 100;
		inline constexpr StyleId UI_RootStyle = 200;
		inline constexpr StyleId UI_PanelStyle = 201;
		inline constexpr StyleId UI_TextStyle = 202;
		inline constexpr StyleId UI_TextBoxStyle = 203;
		inline constexpr StyleId UI_ButtonStyle = 204;
		inline constexpr StyleId UI_SectionStyle = 205;
		inline constexpr StyleId UI_SoftButtonStyle = 206;
		inline constexpr StyleId UI_PillButtonStyle = 207;
		inline constexpr StyleId UI_SquareButtonStyle = 208;
		inline constexpr StyleId UI_WorldOverlayStyle = 209;
		inline constexpr StyleId UI_WorldDepthStyle = 210;
		inline constexpr StyleId UI_WorldFadeStyle = 211;
		inline constexpr TemplateId UI_TextBoxTemplate = 300;
		inline constexpr TemplateId UI_ButtonTemplate = 301;

		// Populate a current-version structure header for one public View3DUI record.
		template <typename T>
		StructHeader UIHeader()
		{
			return StructHeader{sizeof(T), VIEW3D_UI_STRUCT_VERSION};
		}

		// Build one fixed-layout control description before its strings are packed into a transaction.
		ControlDesc UIControl(ControlId id, ControlId parent_id, EControlType type, ELayoutMode layout_mode, LayoutParams layout, StyleId style_id, TemplateId template_id = 0)
		{
			auto desc = ControlDesc{};
			desc.header = UIHeader<ControlDesc>();
			desc.id = id;
			desc.parent_id = parent_id;
			desc.type = type;
			desc.root_policy = ERootPolicy::Screen;
			desc.layout_mode = layout_mode;
			desc.template_id = template_id;
			desc.style_id = style_id;
			desc.enabled = 1;
			desc.visible = 1;
			desc.focusable = type == EControlType::TextBox || type == EControlType::Button;
			desc.validation_state = EValidationState::NotApplicable;
			desc.layout = layout;
			desc.max_text_length = 32;
			desc.font_resource_id = type == EControlType::Root || type == EControlType::Panel ? 0 : UI_Font;
			return desc;
		}

		// Create a layout value with explicit size and alignment while leaving optional spacing at zero.
		LayoutParams UILayout(float width, float height, EHAlign h_align = EHAlign::Left, EVAlign v_align = EVAlign::Top)
		{
			auto layout = LayoutParams{};
			layout.width = width;
			layout.height = height;
			layout.h_align = h_align;
			layout.v_align = v_align;
			return layout;
		}

		// Build one explicit world-anchored root using the bounded billboard vocabulary.
		ControlDesc UIWorldRoot(ControlId id, ERootPolicy policy, pr::view3d::ui::Vec3 anchor, EWorldSizing sizing)
		{
			auto root = UIControl(id, 0, EControlType::Root, ELayoutMode::Overlay, UILayout(164, 52), UI_RootStyle);
			root.root_policy = policy;
			root.layout.padding_left = 4;
			root.layout.padding_top = 4;
			root.layout.padding_right = 4;
			root.layout.padding_bottom = 4;
			root.world = WorldRootParams{
				.anchor = anchor,
				.anchor_h = EAnchorPoint::Centre,
				.anchor_v = EAnchorPoint::Centre,
				.sizing = sizing,
				.world_units_per_dip = 0.01f,
				.depth_offset = 0.03f,
				.occlusion_min_opacity = 0.25f,
				.occlusion_fade_depth = 1.5f,
				.occlusion_depth_bias = 0.05f,
			};
			return root;
		}

		// Create one style with an identical base visual in every state channel.
		StyleDesc UIStyle(StyleId id, UIColour fill, UIColour border, float border_thickness, float corner_radius, float opacity = 1.0f)
		{
			auto style = StyleDesc{};
			style.header = UIHeader<StyleDesc>();
			style.id = id;
			for (auto i = std::size_t{}; i != static_cast<std::size_t>(EStateChannel::Count); ++i)
			{
				style.visuals[i] = StyleVisual{fill, border, border_thickness, corner_radius, opacity};
				style.transitions[i] = TransitionDesc{100.0f, EEasing::EaseInOut};
			}
			return style;
		}

		// Add one application-owned font resource to a public transaction builder.
		void AddUIFont(TransactionBuilder& builder, ResourceId id, std::string_view family, float size, UIColour colour)
		{
			auto resource = ResourceDesc{};
			resource.header = UIHeader<ResourceDesc>();
			resource.id = id;
			resource.kind = EResourceKind::Font;
			resource.colour = colour;
			resource.font_size = size;
			builder.AddNamedResource(resource, family);
		}

		// Parsed numeric state owned by the demonstration application.
		struct DimensionValue
		{
			EValidationState m_validation;
			float m_value;
		};

		// Parse the deliberately invariant positive-decimal grammar used by the demonstration.
		DimensionValue ParseDimension(std::string_view text)
		{
			if (text.empty() || text == "." || text == "+" || text == "-")
				return {EValidationState::Pending, 0.0f};

			auto index = std::size_t{};
			auto negative = false;
			if (text[index] == '+' || text[index] == '-')
			{
				negative = text[index] == '-';
				++index;
			}

			auto value = 0.0;
			auto digit_count = 0;
			while (index != text.size() && text[index] >= '0' && text[index] <= '9')
			{
				value = value * 10.0 + (text[index] - '0');
				++digit_count;
				++index;
			}

			if (index != text.size() && text[index] == '.')
			{
				++index;
				if (index == text.size())
					return {EValidationState::Pending, 0.0f};

				auto scale = 0.1;
				while (index != text.size() && text[index] >= '0' && text[index] <= '9')
				{
					value += (text[index] - '0') * scale;
					scale *= 0.1;
					++digit_count;
					++index;
				}
			}

			if (index != text.size() || digit_count == 0)
				return {EValidationState::Invalid, 0.0f};

			value = negative ? -value : value;
			return std::isfinite(value) && value > 0.0 && value <= std::numeric_limits<float>::max()
				? DimensionValue{EValidationState::Valid, static_cast<float>(value)}
				: DimensionValue{EValidationState::Invalid, 0.0f};
		}

		// Create a UI context while balancing the temporary View3D device lease on every path.
		pr::view3d::ui::UiContext CreateUIContext(pr::view3d::ui::Runtime const& runtime, pr::view3d::DllHandle view3d_context, pr::view3d::Window window)
		{
			auto device = static_cast<IUnknown*>(View3D_DeviceLeaseAcquire(view3d_context));
			if (device == nullptr)
				throw std::runtime_error("Failed to acquire the View3D device for View3DUI");

			try
			{
				auto ui = pr::view3d::ui::UiContext(runtime, device, window);
				device->Release();
				return ui;
			}
			catch (...)
			{
				device->Release();
				throw;
			}
		}
	}

	// Private implementation behind the pimpl façade. Owns every stable id/resource/template/style
	// used to author the gallery, the retained View3DUI runtime/context, the dimension editor's
	// parse/validation state, gallery action status, and the per-frame event buffers.
	struct View3dUiDemo::Impl
	{
		pr::view3d::Window m_win3d;
		UpdateBoxDimensionsCB m_update_box_dimensions;
		pr::view3d::ui::Runtime m_ui_runtime;
		pr::view3d::ui::UiContext m_ui;
		std::uint64_t m_ui_revision;
		double m_ui_time_ms;
		std::string m_dimension_text;
		DimensionValue m_dimension;
		std::string m_gallery_status;
		std::uint32_t m_gallery_status_sequence;
		std::vector<pr::view3d::ui::Event> m_ui_events;
		std::vector<std::byte> m_ui_event_payload;

		// Acquire the runtime/context for 'window' and build the initial retained gallery tree in
		// one revision. Member declaration order places 'm_ui' after 'm_ui_runtime' so the context
		// is destroyed before the runtime it depends on.
		Impl(pr::view3d::DllHandle view3d, pr::view3d::Window window, UpdateBoxDimensionsCB update_box_dimensions)
			: m_win3d(window)
			, m_update_box_dimensions(std::move(update_box_dimensions))
			, m_ui_runtime()
			, m_ui(CreateUIContext(m_ui_runtime, view3d, window))
			, m_ui_revision(0)
			, m_ui_time_ms(0.0)
			, m_dimension_text("1.23")
			, m_dimension(ParseDimension(m_dimension_text))
			, m_gallery_status("Last action: none")
			, m_gallery_status_sequence(0)
		{
			ApplyInitialUI();
		}

		void ApplyInitialUI();
		void ApplyUIState();
		void SetGalleryStatus(std::string_view action);
		void DrainUIEvents();
		void UpdateUI(HWND hwnd);
	};

	// Apply the complete initial UI tree, resources, templates, and styles in one revision.
	void View3dUiDemo::Impl::ApplyInitialUI()
	{
		using namespace pr::view3d::ui;

		TransactionBuilder builder;
		AddUIFont(builder, UI_Font, "Segoe UI", 18.0f, UIColour{1, 1, 1, 1});

		auto root_style = UIStyle(UI_RootStyle, UIColour{0, 0, 0, 0}, UIColour{0, 0, 0, 0}, 0, 0, 0);
		auto panel_style = UIStyle(UI_PanelStyle, UIColour{0.07f, 0.09f, 0.14f, 0.90f}, UIColour{0.25f, 0.38f, 0.58f, 0.85f}, 1.0f, 24.0f);
		auto section_style = UIStyle(UI_SectionStyle, UIColour{0.10f, 0.13f, 0.20f, 0.94f}, UIColour{0.22f, 0.31f, 0.47f, 0.9f}, 1.0f, 18.0f);
		auto text_style = UIStyle(UI_TextStyle, UIColour{0, 0, 0, 0}, UIColour{1, 1, 1, 1}, 0, 0);
		auto textbox_style = UIStyle(UI_TextBoxStyle, UIColour{0.08f, 0.10f, 0.16f, 1}, UIColour{0.38f, 0.50f, 0.68f, 1}, 1.0f, 12.0f);
		textbox_style.visuals[static_cast<std::size_t>(EStateChannel::Hover)].border_colour = UIColour{0.7f, 0.7f, 0.7f, 1};
		textbox_style.visuals[static_cast<std::size_t>(EStateChannel::Focused)].border_colour = UIColour{0.15f, 0.5f, 1.0f, 1};
		textbox_style.visuals[static_cast<std::size_t>(EStateChannel::Focused)].border_thickness = 2.0f;
		textbox_style.visuals[static_cast<std::size_t>(EStateChannel::Invalid)].border_colour = UIColour{1.0f, 0.2f, 0.2f, 1};
		textbox_style.visuals[static_cast<std::size_t>(EStateChannel::Invalid)].border_thickness = 2.0f;
		auto button_style = UIStyle(UI_ButtonStyle, UIColour{0.22f, 0.36f, 0.58f, 1}, UIColour{0.35f, 0.55f, 0.85f, 1}, 1.0f, 8.0f);
		button_style.visuals[static_cast<std::size_t>(EStateChannel::Hover)].fill = UIColour{0.30f, 0.48f, 0.74f, 1};
		button_style.visuals[static_cast<std::size_t>(EStateChannel::Pressed)].fill = UIColour{0.14f, 0.25f, 0.43f, 1};
		button_style.visuals[static_cast<std::size_t>(EStateChannel::Focused)].border_colour = UIColour{0.15f, 0.5f, 1.0f, 1};
		button_style.visuals[static_cast<std::size_t>(EStateChannel::Focused)].border_thickness = 2.0f;
		button_style.visuals[static_cast<std::size_t>(EStateChannel::Disabled)].opacity = 0.45f;
		auto soft_button_style = UIStyle(UI_SoftButtonStyle, UIColour{0.18f, 0.48f, 0.48f, 1}, UIColour{0.32f, 0.72f, 0.70f, 1}, 1.0f, 16.0f);
		soft_button_style.visuals[static_cast<std::size_t>(EStateChannel::Hover)].fill = UIColour{0.24f, 0.62f, 0.60f, 1};
		soft_button_style.visuals[static_cast<std::size_t>(EStateChannel::Pressed)].fill = UIColour{0.10f, 0.34f, 0.34f, 1};
		auto pill_button_style = UIStyle(UI_PillButtonStyle, UIColour{0.48f, 0.25f, 0.62f, 1}, UIColour{0.72f, 0.42f, 0.88f, 1}, 1.0f, 22.0f);
		pill_button_style.visuals[static_cast<std::size_t>(EStateChannel::Hover)].fill = UIColour{0.62f, 0.34f, 0.78f, 1};
		pill_button_style.visuals[static_cast<std::size_t>(EStateChannel::Selected)].fill = UIColour{0.72f, 0.38f, 0.24f, 1};
		auto square_button_style = UIStyle(UI_SquareButtonStyle, UIColour{0.44f, 0.38f, 0.18f, 1}, UIColour{0.78f, 0.66f, 0.30f, 1}, 1.0f, 2.0f);
		square_button_style.visuals[static_cast<std::size_t>(EStateChannel::Hover)].fill = UIColour{0.58f, 0.50f, 0.24f, 1};
		auto world_overlay_style = UIStyle(UI_WorldOverlayStyle, UIColour{0.14f, 0.50f, 0.72f, 0.94f}, UIColour{0.40f, 0.82f, 1.0f, 1}, 1.5f, 18.0f);
		auto world_depth_style = UIStyle(UI_WorldDepthStyle, UIColour{0.20f, 0.62f, 0.30f, 0.94f}, UIColour{0.48f, 0.92f, 0.56f, 1}, 1.5f, 12.0f);
		auto world_fade_style = UIStyle(UI_WorldFadeStyle, UIColour{0.72f, 0.34f, 0.14f, 0.94f}, UIColour{1.0f, 0.62f, 0.28f, 1}, 1.5f, 22.0f);
		builder.AddStyle(root_style);
		builder.AddStyle(panel_style);
		builder.AddStyle(section_style);
		builder.AddStyle(text_style);
		builder.AddStyle(textbox_style);
		builder.AddStyle(button_style);
		builder.AddStyle(soft_button_style);
		builder.AddStyle(pill_button_style);
		builder.AddStyle(square_button_style);
		builder.AddStyle(world_overlay_style);
		builder.AddStyle(world_depth_style);
		builder.AddStyle(world_fade_style);

		auto textbox_template = TemplateDesc{};
		textbox_template.header = UIHeader<TemplateDesc>();
		textbox_template.id = UI_TextBoxTemplate;
		textbox_template.applies_to = EControlType::TextBox;
		builder.AddTemplatePart(textbox_template, "PART_Text", EVisualPrimitive::TextPresenter);
		builder.AddTemplatePart(textbox_template, "PART_Selection", EVisualPrimitive::SolidBox);
		builder.AddTemplatePart(textbox_template, "PART_Caret", EVisualPrimitive::SolidBox);
		builder.AddTemplatePart(textbox_template, "PART_ValidationOutline", EVisualPrimitive::Border);
		builder.AddTemplatePart(textbox_template, "PART_FocusOutline", EVisualPrimitive::Border);
		builder.AddTemplate(textbox_template);

		auto button_template = TemplateDesc{};
		button_template.header = UIHeader<TemplateDesc>();
		button_template.id = UI_ButtonTemplate;
		button_template.applies_to = EControlType::Button;
		builder.AddTemplatePart(button_template, "PART_ContentPresenter", EVisualPrimitive::ContentPresenter);
		builder.AddTemplatePart(button_template, "PART_FocusOutline", EVisualPrimitive::Border);
		builder.AddTemplate(button_template);

		// Screen-space gallery: vertical composition containing examples of every layout mode.
		auto root_layout = UILayout(0, 0, EHAlign::Stretch, EVAlign::Stretch);
		auto panel_layout = UILayout(430, 570, EHAlign::Right, EVAlign::Top);
		panel_layout.margin_top = 24;
		panel_layout.margin_right = 24;
		panel_layout.padding_left = 16;
		panel_layout.padding_top = 16;
		panel_layout.padding_right = 16;
		panel_layout.padding_bottom = 16;
		panel_layout.stack_spacing = 8;
		auto title_layout = UILayout(398, 32, EHAlign::Stretch);
		auto dimension_section_layout = UILayout(398, 118, EHAlign::Stretch);
		dimension_section_layout.padding_left = 12;
		dimension_section_layout.padding_top = 10;
		dimension_section_layout.padding_right = 12;
		dimension_section_layout.padding_bottom = 10;
		dimension_section_layout.stack_spacing = 8;
		auto label_layout = UILayout(374, 22, EHAlign::Stretch);
		auto dimension_row_layout = UILayout(374, 40, EHAlign::Stretch);
		dimension_row_layout.stack_spacing = 8;
		auto textbox_layout = UILayout(230, 40, EHAlign::Left, EVAlign::Stretch);
		auto button_layout = UILayout(136, 40, EHAlign::Left, EVAlign::Stretch);
		auto button_row_layout = UILayout(398, 42, EHAlign::Stretch);
		button_row_layout.stack_spacing = 8;

		builder.Upsert(UIControl(UI_Root, 0, EControlType::Root, ELayoutMode::Overlay, root_layout, UI_RootStyle), {}, "View3DUI screen root");
		builder.Upsert(UIControl(UI_Panel, UI_Root, EControlType::Panel, ELayoutMode::StackVertical, panel_layout, UI_PanelStyle), {}, "View3DUI gallery");
		builder.Upsert(UIControl(UI_Title, UI_Panel, EControlType::Text, ELayoutMode::Overlay, title_layout, UI_TextStyle), "View3DUI gallery", "Gallery title");
		builder.Upsert(UIControl(UI_DimensionSection, UI_Panel, EControlType::Panel, ELayoutMode::StackVertical, dimension_section_layout, UI_SectionStyle), {}, "Dimension editor section");
		builder.Upsert(UIControl(UI_Label, UI_DimensionSection, EControlType::Text, ELayoutMode::Overlay, label_layout, UI_TextStyle), "Box dimension", "Box dimension label");
		builder.Upsert(UIControl(UI_DimensionRow, UI_DimensionSection, EControlType::Panel, ELayoutMode::StackHorizontal, dimension_row_layout, UI_RootStyle), {}, "Dimension editor row");
		auto textbox = UIControl(UI_Dimension, UI_DimensionRow, EControlType::TextBox, ELayoutMode::Overlay, textbox_layout, UI_TextBoxStyle, UI_TextBoxTemplate);
		textbox.validation_state = m_dimension.m_validation;
		builder.Upsert(textbox, m_dimension_text, "Box dimensions", "A positive decimal value applied uniformly to the box");
		auto button = UIControl(UI_Update, UI_DimensionRow, EControlType::Button, ELayoutMode::Overlay, button_layout, UI_ButtonStyle, UI_ButtonTemplate);
		button.enabled = m_dimension.m_validation == EValidationState::Valid;
		builder.Upsert(button, "Update", "Update box dimensions");
		builder.Upsert(UIControl(UI_ButtonLabel, UI_Panel, EControlType::Text, ELayoutMode::Overlay, UILayout(398, 22, EHAlign::Stretch), UI_TextStyle), "Horizontal Stack + style states", "Button styles label");
		builder.Upsert(UIControl(UI_ButtonRow, UI_Panel, EControlType::Panel, ELayoutMode::StackHorizontal, button_row_layout, UI_RootStyle), {}, "Horizontal button stack");
		builder.Upsert(UIControl(UI_SoftButton, UI_ButtonRow, EControlType::Button, ELayoutMode::Overlay, UILayout(124, 42, EHAlign::Left, EVAlign::Stretch), UI_SoftButtonStyle, UI_ButtonTemplate), "Soft", "Soft rounded button");
		auto pill_button = UIControl(UI_PillButton, UI_ButtonRow, EControlType::Button, ELayoutMode::Overlay, UILayout(124, 42, EHAlign::Left, EVAlign::Stretch), UI_PillButtonStyle, UI_ButtonTemplate);
		pill_button.selected = 1;
		builder.Upsert(pill_button, "Selected", "Selected pill button");
		auto disabled_button = UIControl(UI_DisabledButton, UI_ButtonRow, EControlType::Button, ELayoutMode::Overlay, UILayout(124, 42, EHAlign::Left, EVAlign::Stretch), UI_SquareButtonStyle, UI_ButtonTemplate);
		disabled_button.enabled = 0;
		builder.Upsert(disabled_button, "Disabled", "Disabled square button");

		// Overlay demonstrates alignment-based overlap inside a softly rounded panel.
		auto overlay_layout = UILayout(398, 62, EHAlign::Stretch);
		overlay_layout.padding_left = 10;
		overlay_layout.padding_top = 10;
		overlay_layout.padding_right = 10;
		overlay_layout.padding_bottom = 10;
		builder.Upsert(UIControl(UI_OverlayPanel, UI_Panel, EControlType::Panel, ELayoutMode::Overlay, overlay_layout, UI_SectionStyle), {}, "Overlay layout example");
		builder.Upsert(UIControl(UI_OverlayText, UI_OverlayPanel, EControlType::Text, ELayoutMode::Overlay, UILayout(220, 24, EHAlign::Left, EVAlign::Center), UI_TextStyle), "Overlay layout", "Overlay layout label");
		builder.Upsert(UIControl(UI_OverlayButton, UI_OverlayPanel, EControlType::Button, ELayoutMode::Overlay, UILayout(132, 36, EHAlign::Right, EVAlign::Center), UI_SoftButtonStyle, UI_ButtonTemplate), "Overlay", "Overlay example button");

		// Canvas demonstrates absolute child placement within the same retained tree.
		auto canvas_layout = UILayout(398, 62, EHAlign::Stretch);
		builder.Upsert(UIControl(UI_CanvasPanel, UI_Panel, EControlType::Panel, ELayoutMode::Canvas, canvas_layout, UI_SectionStyle), {}, "Canvas layout example");
		auto canvas_text_layout = UILayout(210, 24);
		canvas_text_layout.canvas_x = 14;
		canvas_text_layout.canvas_y = 19;
		builder.Upsert(UIControl(UI_CanvasText, UI_CanvasPanel, EControlType::Text, ELayoutMode::Overlay, canvas_text_layout, UI_TextStyle), "Canvas: absolute position", "Canvas layout label");
		auto canvas_button_layout = UILayout(132, 36);
		canvas_button_layout.canvas_x = 252;
		canvas_button_layout.canvas_y = 13;
		builder.Upsert(UIControl(UI_CanvasButton, UI_CanvasPanel, EControlType::Button, ELayoutMode::Overlay, canvas_button_layout, UI_PillButtonStyle, UI_ButtonTemplate), "Canvas", "Canvas example button");

		// Scroll demonstrates descriptor-authored content offset without adding another layout type.
		auto scroll_layout = UILayout(398, 50, EHAlign::Stretch);
		scroll_layout.padding_left = 10;
		scroll_layout.padding_top = 8;
		scroll_layout.padding_right = 10;
		scroll_layout.padding_bottom = 8;
		scroll_layout.scroll_offset_x = 18;
		builder.Upsert(UIControl(UI_ScrollPanel, UI_Panel, EControlType::Panel, ELayoutMode::Scroll, scroll_layout, UI_SectionStyle), {}, "Scroll layout example");
		auto scroll_text_layout = UILayout(360, 24, EHAlign::Left, EVAlign::Center);
		scroll_text_layout.margin_left = 30;
		builder.Upsert(UIControl(UI_ScrollText, UI_ScrollPanel, EControlType::Text, ELayoutMode::Overlay, scroll_text_layout, UI_TextStyle), "← Scroll offset shifts this content", "Scrolled content");
		auto status = UIControl(UI_Status, UI_Panel, EControlType::Text, ELayoutMode::Overlay, UILayout(398, 26, EHAlign::Stretch), UI_TextStyle);
		status.value_sequence = m_gallery_status_sequence;
		builder.Upsert(status, m_gallery_status, m_gallery_status);

		// One clickable button per world policy, using both apparent-DIP and world-unit sizing.
		auto world_button_layout = UILayout(0, 0, EHAlign::Stretch, EVAlign::Stretch);
		builder.Upsert(UIWorldRoot(UI_WorldOverlayRoot, ERootPolicy::Overlay, pr::view3d::ui::Vec3{-1.6f, 0.0f, 1.8f}, EWorldSizing::ConstantDip), {}, "World overlay root");
		builder.Upsert(UIControl(UI_WorldOverlayButton, UI_WorldOverlayRoot, EControlType::Button, ELayoutMode::Overlay, world_button_layout, UI_WorldOverlayStyle, UI_ButtonTemplate), "World Overlay", "World overlay button");
		builder.Upsert(UIWorldRoot(UI_WorldDepthRoot, ERootPolicy::DepthTested, pr::view3d::ui::Vec3{-1.4f, -1.4f, 0.6f}, EWorldSizing::WorldUnits), {}, "Depth-tested world root");
		builder.Upsert(UIControl(UI_WorldDepthButton, UI_WorldDepthRoot, EControlType::Button, ELayoutMode::Overlay, world_button_layout, UI_WorldDepthStyle, UI_ButtonTemplate), "Depth Tested", "Depth-tested world button");
		builder.Upsert(UIWorldRoot(UI_WorldFadeRoot, ERootPolicy::OcclusionFaded, pr::view3d::ui::Vec3{-0.6f, -1.8f, -0.2f}, EWorldSizing::ConstantDip), {}, "Occlusion-faded world root");
		builder.Upsert(UIControl(UI_WorldFadeButton, UI_WorldFadeRoot, EControlType::Button, ELayoutMode::Overlay, world_button_layout, UI_WorldFadeStyle, UI_ButtonTemplate), "Occlusion Fade", "Occlusion-faded world button");

		builder.Reorder(UI_Root, {UI_Panel});
		builder.Reorder(UI_Panel, {UI_Title, UI_DimensionSection, UI_ButtonLabel, UI_ButtonRow, UI_OverlayPanel, UI_CanvasPanel, UI_ScrollPanel, UI_Status});
		builder.Reorder(UI_DimensionSection, {UI_Label, UI_DimensionRow});
		builder.Reorder(UI_DimensionRow, {UI_Dimension, UI_Update});
		builder.Reorder(UI_ButtonRow, {UI_SoftButton, UI_PillButton, UI_DisabledButton});
		builder.Reorder(UI_OverlayPanel, {UI_OverlayText, UI_OverlayButton});
		builder.Reorder(UI_CanvasPanel, {UI_CanvasText, UI_CanvasButton});
		builder.Reorder(UI_ScrollPanel, {UI_ScrollText});
		builder.Reorder(UI_WorldOverlayRoot, {UI_WorldOverlayButton});
		builder.Reorder(UI_WorldDepthRoot, {UI_WorldDepthButton});
		builder.Reorder(UI_WorldFadeRoot, {UI_WorldFadeButton});

		m_ui.TransactionApply(builder.Build(m_ui_revision, m_ui_revision + 1));
		++m_ui_revision;
	}

	// Reconcile application-owned text, validation, and command enablement in one revision.
	void View3dUiDemo::Impl::ApplyUIState()
	{
		using namespace pr::view3d::ui;

		TransactionBuilder builder;
		auto textbox = UIControl(UI_Dimension, UI_DimensionRow, EControlType::TextBox, ELayoutMode::Overlay, UILayout(230, 40, EHAlign::Left, EVAlign::Stretch), UI_TextBoxStyle, UI_TextBoxTemplate);
		textbox.validation_state = m_dimension.m_validation;
		builder.Upsert(textbox, m_dimension_text, "Box dimensions", "A positive decimal value applied uniformly to the box");
		auto button = UIControl(UI_Update, UI_DimensionRow, EControlType::Button, ELayoutMode::Overlay, UILayout(136, 40, EHAlign::Left, EVAlign::Stretch), UI_ButtonStyle, UI_ButtonTemplate);
		button.enabled = m_dimension.m_validation == EValidationState::Valid;
		builder.Upsert(button, "Update", "Update box dimensions");
		auto status = UIControl(UI_Status, UI_Panel, EControlType::Text, ELayoutMode::Overlay, UILayout(398, 26, EHAlign::Stretch), UI_TextStyle);
		status.value_sequence = m_gallery_status_sequence;
		builder.Upsert(status, m_gallery_status, m_gallery_status);

		m_ui.TransactionApply(builder.Build(m_ui_revision, m_ui_revision + 1));
		++m_ui_revision;
	}

	// Publish the most recent gallery command through the same retained descriptor model.
	void View3dUiDemo::Impl::SetGalleryStatus(std::string_view action)
	{
		m_gallery_status = "Last action: ";
		m_gallery_status.append(action);
		++m_gallery_status_sequence;
		ApplyUIState();
	}

	// Drain typed UI events and dispatch centrally by stable control id.
	void View3dUiDemo::Impl::DrainUIEvents()
	{
		auto pending = m_ui.EventsPendingSizes();
		if (pending.m_count == 0)
			return;

		m_ui_events.resize(pending.m_count);
		m_ui_event_payload.resize(pending.m_payload_bytes);
		m_ui.EventsCopy(m_ui_events, m_ui_event_payload);

		for (auto const& event : m_ui_events)
		{
			switch (event.kind)
			{
				case pr::view3d::ui::EEventKind::TextChangeProposed:
				{
					if (event.control_id != UI_Dimension)
						break;

					auto const* text = reinterpret_cast<char const*>(m_ui_event_payload.data() + event.payload_offset);
					m_dimension_text.assign(text, event.payload_length);
					m_dimension = ParseDimension(m_dimension_text);
					ApplyUIState();
					break;
				}
				case pr::view3d::ui::EEventKind::CommandInvoked:
				{
					switch (event.control_id)
					{
						case UI_Update:
						{
							if (m_dimension.m_validation == pr::view3d::ui::EValidationState::Valid)
								m_update_box_dimensions(m_dimension.m_value);

							break;
						}
						case UI_SoftButton: { SetGalleryStatus("soft button"); break; }
						case UI_PillButton: { SetGalleryStatus("selected pill button"); break; }
						case UI_OverlayButton: { SetGalleryStatus("overlay layout button"); break; }
						case UI_CanvasButton: { SetGalleryStatus("canvas layout button"); break; }
						case UI_WorldOverlayButton: { SetGalleryStatus("world overlay button"); break; }
						case UI_WorldDepthButton: { SetGalleryStatus("depth-tested world button"); break; }
						case UI_WorldFadeButton: { SetGalleryStatus("occlusion-faded world button"); break; }
						default: break;
					}
					break;
				}
				case pr::view3d::ui::EEventKind::FocusChanged:
				case pr::view3d::ui::EEventKind::PointerCaptureChanged:
				case pr::view3d::ui::EEventKind::QueueOverflow:
				case pr::view3d::ui::EEventKind::Diagnostic:
				{
					break;
				}
				case pr::view3d::ui::EEventKind::Count:
				default:
				{
					throw std::runtime_error("Unknown View3DUI event kind");
				}
			}
		}
	}

	// Recompute deterministic layout, semantics, transitions, and draw packets at explicit UI time.
	void View3dUiDemo::Impl::UpdateUI(HWND hwnd)
	{
		auto client = pr::gui::Control::ClientRect(hwnd, false);
		auto target = View3D_WindowBackBufferSizeGet(m_win3d);
		auto viewport = View3D_WindowViewportGet(m_win3d);
		auto c2w = View3D_CameraToWorldGet(m_win3d);
		auto fov = View3D_CameraFovGet(m_win3d);
		auto clip = View3D_CameraClipPlanesGet(m_win3d, pr::view3d::EClipPlanes::Both);
		auto focus_distance = View3D_CameraFocusDistanceGet(m_win3d);
		auto view_rect = View3D_CameraViewRectAtDistanceGet(m_win3d, focus_distance);
		auto camera = pr::view3d::ui::CameraState{
			.position = {c2w.w.x, c2w.w.y, c2w.w.z},
			.forward = {-c2w.z.x, -c2w.z.y, -c2w.z.z},
			.up = {c2w.y.x, c2w.y.y, c2w.y.z},
			.projection = View3D_CameraOrthographicGet(m_win3d) != FALSE ? pr::view3d::ui::EProjection::Orthographic : pr::view3d::ui::EProjection::Perspective,
			.valid = 1,
			.fov_y_rad = fov.y,
			.ortho_height = view_rect.y,
			.near_plane = clip.x,
			.far_plane = clip.y,
		};
		auto state = pr::view3d::ui::ViewportState{
			.header = UIHeader<pr::view3d::ui::ViewportState>(),
			.client_width_px = pr::s_cast<std::uint32_t>(std::max(0, client.width())),
			.client_height_px = pr::s_cast<std::uint32_t>(std::max(0, client.height())),
			.target_width_px = pr::s_cast<std::uint32_t>(std::max(0L, target.cx)),
			.target_height_px = pr::s_cast<std::uint32_t>(std::max(0L, target.cy)),
			.viewport_x_px = viewport.m_x,
			.viewport_y_px = viewport.m_y,
			.viewport_width_px = viewport.m_width,
			.viewport_height_px = viewport.m_height,
			.dpi = static_cast<float>(GetDpiForWindow(hwnd)),
			.time_ms = m_ui_time_ms,
			.camera = camera,
		};
		m_ui.Update(state);
	}

	// Create the View3DUI runtime/context for 'window' and populate the initial gallery tree.
	View3dUiDemo::View3dUiDemo(pr::view3d::DllHandle view3d, pr::view3d::Window window, UpdateBoxDimensionsCB update_box_dimensions)
		: m_impl(std::make_unique<Impl>(view3d, window, std::move(update_box_dimensions)))
	{}

	// Defined where 'Impl' is a complete type so the default 'unique_ptr' deleter can run.
	View3dUiDemo::~View3dUiDemo() = default;

	// Forward one raw Win32 message to View3DUI before the host window's own input handling.
	bool View3dUiDemo::ProcessWindowMessage(HWND hwnd, UINT message, WPARAM wparam, LPARAM lparam, LRESULT& result)
	{
		auto invalidate = std::int32_t{};
		auto ui_result = LRESULT{};
		if (m_impl->m_ui.ProcessWindowMessage(hwnd, message, wparam, lparam, ui_result, invalidate) == 0)
			return false;

		result = ui_result;
		return true;
	}

	// Advance the UI clock, drain and dispatch queued gallery events, then refresh View3DUI.
	void View3dUiDemo::Update(HWND hwnd, double elapsed_seconds)
	{
		m_impl->m_ui_time_ms += elapsed_seconds * 1000.0;
		m_impl->DrainUIEvents();
		m_impl->UpdateUI(hwnd);
	}
}
