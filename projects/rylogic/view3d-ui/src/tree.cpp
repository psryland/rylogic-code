//*********************************************
// View3DUI
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#include "tree.h"

namespace pr::view3d::ui
{
	namespace
	{
		// Copy 'length' bytes starting at 'offset' out of 'blob' as a std::string, validating the
		// range lies entirely within the blob (section 5.3: every blob range is bounds-checked).
		std::string CopyBlobRange(std::byte const* blob, std::uint32_t blob_length, std::uint32_t offset, std::uint32_t length, char const* what)
		{
			if (length == 0)
				return {};

			// Reject overflowing ranges explicitly rather than relying on unsigned wraparound.
			if (offset > blob_length || length > blob_length - offset)
				throw EngineException(EStatus::InvalidStruct, std::format("{}: blob range [{}, {}) exceeds blob length {}", what, offset, offset + length, blob_length));

			return std::string(reinterpret_cast<char const*>(blob) + offset, length);
		}

		// Required template part names per closed control type (section 6.3). Button needs a
		// content presenter and a focus outline; TextBox additionally needs its text/selection/
		// caret/validation visuals. Root/Panel/Text have no required parts.
		std::array<std::string_view, 2> const g_button_parts = { "PART_ContentPresenter", "PART_FocusOutline" };
		std::array<std::string_view, 5> const g_textbox_parts = { "PART_Text", "PART_Selection", "PART_Caret", "PART_ValidationOutline", "PART_FocusOutline" };

		// Recursively remove 'id' and its whole subtree from 'tree', unlinking it from its parent's
		// children and, if it is a root, from the root list. No-op if 'id' is not present.
		void RemoveSubtree(TreeModel& tree, ControlId id)
		{
			auto it = tree.m_controls.find(id);
			if (it == tree.m_controls.end())
				return;

			// Depth-first: remove every descendant before erasing this node.
			auto children = it->second.children; // copy: erasing children mutates the map
			for (auto child_id : children)
				RemoveSubtree(tree, child_id);

			auto parent_id = it->second.desc.parent_id;
			if (auto parent_it = tree.m_controls.find(parent_id); parent_it != tree.m_controls.end())
			{
				auto& siblings = parent_it->second.children;
				siblings.erase(std::remove(siblings.begin(), siblings.end(), id), siblings.end());
			}
			if (it->second.desc.type == EControlType::Root)
			{
				auto& roots = tree.m_roots;
				roots.erase(std::remove(roots.begin(), roots.end(), id), roots.end());
			}
			tree.m_controls.erase(it);
		}

		// Validate the world-anchoring block of a Root whose policy is world-anchored. Every value
		// must be finite and inside its documented bound so projection and fade are deterministic
		// and can never produce NaN geometry or an unbounded opacity multiplier.
		void ValidateWorldRootParams(ControlDesc const& desc)
		{
			auto const& world = desc.world;
			if (!std::isfinite(world.anchor.x) || !std::isfinite(world.anchor.y) || !std::isfinite(world.anchor.z))
				throw EngineException(EStatus::InvalidArgument, std::format("control {}: world anchor ({}, {}, {}) must be finite", desc.id, world.anchor.x, world.anchor.y, world.anchor.z));
			if (static_cast<std::uint32_t>(world.anchor_h) >= static_cast<std::uint32_t>(EAnchorPoint::Count))
				throw EngineException(EStatus::UnknownType, std::format("control {}: unknown horizontal EAnchorPoint {}", desc.id, static_cast<int>(world.anchor_h)));
			if (static_cast<std::uint32_t>(world.anchor_v) >= static_cast<std::uint32_t>(EAnchorPoint::Count))
				throw EngineException(EStatus::UnknownType, std::format("control {}: unknown vertical EAnchorPoint {}", desc.id, static_cast<int>(world.anchor_v)));
			if (static_cast<std::uint32_t>(world.sizing) >= static_cast<std::uint32_t>(EWorldSizing::Count))
				throw EngineException(EStatus::UnknownType, std::format("control {}: unknown EWorldSizing {}", desc.id, static_cast<int>(world.sizing)));

			// world_units_per_dip only participates under WorldUnits sizing, but is validated
			// unconditionally so a descriptor cannot carry a NaN that becomes live on a later
			// sizing change.
			if (!std::isfinite(world.world_units_per_dip) || world.world_units_per_dip <= 0.0f)
				throw EngineException(EStatus::InvalidArgument, std::format("control {}: world_units_per_dip {} must be finite and greater than zero", desc.id, world.world_units_per_dip));
			if (!std::isfinite(world.depth_offset) || world.depth_offset < 0.0f)
				throw EngineException(EStatus::InvalidArgument, std::format("control {}: depth_offset {} must be finite and non-negative", desc.id, world.depth_offset));
			if (!std::isfinite(world.occlusion_min_opacity) || world.occlusion_min_opacity < 0.0f || world.occlusion_min_opacity > 1.0f)
				throw EngineException(EStatus::InvalidArgument, std::format("control {}: occlusion_min_opacity {} must be finite and within [0, 1]", desc.id, world.occlusion_min_opacity));
			if (!std::isfinite(world.occlusion_fade_depth) || world.occlusion_fade_depth <= 0.0f)
				throw EngineException(EStatus::InvalidArgument, std::format("control {}: occlusion_fade_depth {} must be finite and greater than zero", desc.id, world.occlusion_fade_depth));
			if (!std::isfinite(world.occlusion_depth_bias) || world.occlusion_depth_bias < 0.0f)
				throw EngineException(EStatus::InvalidArgument, std::format("control {}: occlusion_depth_bias {} must be finite and non-negative", desc.id, world.occlusion_depth_bias));

			// A world root has no host viewport to inherit an extent from, so the autosize
			// convention that a screen root relies on has no meaning here.
			if (desc.layout.width <= 0.0f || desc.layout.height <= 0.0f)
				throw EngineException(EStatus::InvalidArgument, std::format("control {}: a world-anchored root requires an explicit positive layout size, got {}x{}", desc.id, desc.layout.width, desc.layout.height));
		}

		// Validate the numeric fields of one ControlDesc that do not depend on the rest of the
		// tree (finite/non-negative dimensions, enum ranges, supported feature subset).
		void ValidateControlDescLocal(ControlDesc const& desc)
		{
			if (desc.id == 0)
				throw EngineException(EStatus::InvalidTree, "control id 0 is reserved and cannot be assigned to a control");
			if (static_cast<std::uint32_t>(desc.type) >= static_cast<std::uint32_t>(EControlType::Count))
				throw EngineException(EStatus::UnknownType, std::format("control {}: unknown EControlType {}", desc.id, static_cast<int>(desc.type)));
			if (desc.type == EControlType::Root && desc.parent_id != 0)
				throw EngineException(EStatus::InvalidTree, std::format("control {}: a Root control must have parent_id 0", desc.id));
			if (desc.type != EControlType::Root && desc.parent_id == 0)
				throw EngineException(EStatus::InvalidTree, std::format("control {}: only a Root control may have parent_id 0", desc.id));
			if (desc.type == EControlType::Root && static_cast<std::uint32_t>(desc.root_policy) >= static_cast<std::uint32_t>(ERootPolicy::Count))
				throw EngineException(EStatus::UnknownType, std::format("control {}: unknown ERootPolicy {}", desc.id, static_cast<int>(desc.root_policy)));

			if (static_cast<std::uint32_t>(desc.layout_mode) >= static_cast<std::uint32_t>(ELayoutMode::Count))
				throw EngineException(EStatus::UnknownType, std::format("control {}: unknown ELayoutMode {}", desc.id, static_cast<int>(desc.layout_mode)));

			if (static_cast<std::uint32_t>(desc.validation_state) >= static_cast<std::uint32_t>(EValidationState::Count))
				throw EngineException(EStatus::UnknownType, std::format("control {}: unknown EValidationState {}", desc.id, static_cast<int>(desc.validation_state)));

			auto const& layout = desc.layout;
			auto is_root_autosize = desc.type == EControlType::Root && desc.root_policy == ERootPolicy::Screen && layout.width == 0.0f && layout.height == 0.0f;
			if (!is_root_autosize)
			{
				if (!std::isfinite(layout.width) || layout.width < 0.0f)
					throw EngineException(EStatus::InvalidArgument, std::format("control {}: layout width {} must be finite and non-negative", desc.id, layout.width));
				if (!std::isfinite(layout.height) || layout.height < 0.0f)
					throw EngineException(EStatus::InvalidArgument, std::format("control {}: layout height {} must be finite and non-negative", desc.id, layout.height));
			}
			float const* margins_and_padding[] = { &layout.margin_left, &layout.margin_top, &layout.margin_right, &layout.margin_bottom, &layout.padding_left, &layout.padding_top, &layout.padding_right, &layout.padding_bottom, &layout.stack_spacing };
			for (auto* value : margins_and_padding)
			{
				if (!std::isfinite(*value) || *value < 0.0f)
					throw EngineException(EStatus::InvalidArgument, std::format("control {}: margin/padding/spacing value {} must be finite and non-negative", desc.id, *value));
			}
			if (static_cast<std::uint32_t>(layout.h_align) >= static_cast<std::uint32_t>(EHAlign::Count))
				throw EngineException(EStatus::UnknownType, std::format("control {}: unknown EHAlign {}", desc.id, static_cast<int>(layout.h_align)));
			if (static_cast<std::uint32_t>(layout.v_align) >= static_cast<std::uint32_t>(EVAlign::Count))
				throw EngineException(EStatus::UnknownType, std::format("control {}: unknown EVAlign {}", desc.id, static_cast<int>(layout.v_align)));

			// canvas_x/y are a position (may be negative, e.g. to place a child partially off the
			// left/top edge of a Canvas parent); scroll_offset_x/y are a distance the control's own
			// content has been shifted by and so cannot be negative.
			if (!std::isfinite(layout.canvas_x) || !std::isfinite(layout.canvas_y))
				throw EngineException(EStatus::InvalidArgument, std::format("control {}: canvas position ({}, {}) must be finite", desc.id, layout.canvas_x, layout.canvas_y));
			if (!std::isfinite(layout.scroll_offset_x) || layout.scroll_offset_x < 0.0f || !std::isfinite(layout.scroll_offset_y) || layout.scroll_offset_y < 0.0f)
				throw EngineException(EStatus::InvalidArgument, std::format("control {}: scroll offset ({}, {}) must be finite and non-negative", desc.id, layout.scroll_offset_x, layout.scroll_offset_y));

			// The world block only exists for world-anchored roots; a screen root or a non-root
			// control carries whatever the application left there and it is never read.
			if (desc.type == EControlType::Root && IsWorldPolicy(desc.root_policy))
				ValidateWorldRootParams(desc);
		}
	}

	std::span<std::string_view const> RequiredTemplateParts(EControlType type)
	{
		switch (type)
		{
			case EControlType::Button: return std::span<std::string_view const>(g_button_parts);
			case EControlType::TextBox: return std::span<std::string_view const>(g_textbox_parts);
			case EControlType::Root:
			case EControlType::Panel:
			case EControlType::Text:
			{
				return {};
			}
			case EControlType::Count:
			default:
			{
				throw EngineException(EStatus::UnknownType, std::format("RequiredTemplateParts: unknown EControlType {}", static_cast<int>(type)));
			}
		}
	}

	TemplateRecord const& TreeModel::DefaultTemplate(EControlType type)
	{
		// One process-wide default template per control type, synthesized in code (never via a
		// transaction) so every control has a usable visual tree even before the application
		// authors an explicit TemplateDesc (section 6.3).
		static std::unordered_map<EControlType, TemplateRecord> const s_defaults = []
		{
			std::unordered_map<EControlType, TemplateRecord> map;
			auto make = [](EControlType applies_to, std::initializer_list<std::pair<std::string_view, EVisualPrimitive>> parts)
			{
				TemplateRecord rec{};
				rec.desc.header = { sizeof(TemplateDesc), VIEW3D_UI_STRUCT_VERSION };
				rec.desc.id = 0;
				rec.desc.applies_to = applies_to;
				rec.desc.part_count = static_cast<std::uint32_t>(parts.size());
				auto i = std::uint32_t{};
				for (auto const& [name, primitive] : parts)
				{
					rec.part_names[i] = name;
					rec.desc.parts[i] = { 0, static_cast<std::uint32_t>(name.size()), primitive, 1 };
					++i;
				}
				return rec;
			};
			map.emplace(EControlType::Root, make(EControlType::Root, { { "PART_ContentPresenter", EVisualPrimitive::ContentPresenter } }));
			map.emplace(EControlType::Panel, make(EControlType::Panel, { { "PART_ContentPresenter", EVisualPrimitive::ContentPresenter } }));
			map.emplace(EControlType::Text, make(EControlType::Text, { { "PART_Text", EVisualPrimitive::TextPresenter } }));
			map.emplace(EControlType::Button, make(EControlType::Button, { { "PART_ContentPresenter", EVisualPrimitive::ContentPresenter }, { "PART_FocusOutline", EVisualPrimitive::Border } }));
			map.emplace(EControlType::TextBox, make(EControlType::TextBox, {
				{ "PART_Text", EVisualPrimitive::TextPresenter },
				{ "PART_Selection", EVisualPrimitive::SolidBox },
				{ "PART_Caret", EVisualPrimitive::SolidBox },
				{ "PART_ValidationOutline", EVisualPrimitive::Border },
				{ "PART_FocusOutline", EVisualPrimitive::Border },
			}));
			return map;
		}();
		auto it = s_defaults.find(type);
		if (it == s_defaults.end())
			throw EngineException(EStatus::UnknownType, std::format("DefaultTemplate: unknown EControlType {}", static_cast<int>(type)));

		return it->second;
	}

	StyleRecord const& TreeModel::DefaultStyle()
	{
		// One process-wide default style: opaque light fill, no border, fully-opaque, identical
		// across every state channel (no visual state feedback until the application supplies its
		// own StyleDesc). Kept intentionally neutral rather than theme-like.
		static StyleRecord const s_default = []
		{
			StyleRecord rec{};
			rec.desc.header = { sizeof(StyleDesc), VIEW3D_UI_STRUCT_VERSION };
			rec.desc.id = 0;
			for (auto i = std::size_t{}; i != static_cast<std::size_t>(EStateChannel::Count); ++i)
			{
				rec.desc.visuals[i] = StyleVisual{ Colour{0.82f, 0.82f, 0.82f, 1.0f}, Colour{0.4f, 0.4f, 0.4f, 1.0f}, 1.0f, 0.0f, 1.0f };
				rec.desc.transitions[i] = TransitionDesc{ 0.0f, EEasing::Linear };
			}
			return rec;
		}();
		return s_default;
	}

	TreeModel TreeModel::Apply(Transaction const& txn, Config const& config) const
	{
		// Structural bounds on the transaction itself (section 9.2); checked before any mutation.
		if (txn.base_revision != m_revision)
			throw EngineException(EStatus::StaleRevision, std::format("transaction base_revision {} does not match the currently accepted revision {}", txn.base_revision, m_revision));
		if (txn.revision != txn.base_revision + 1)
			throw EngineException(EStatus::InvalidArgument, std::format("transaction revision {} must be exactly base_revision + 1 ({})", txn.revision, txn.base_revision + 1));
		if (txn.operation_count > config.max_operations_per_transaction)
			throw EngineException(EStatus::ResourceLimit, std::format("transaction has {} operations, exceeding max_operations_per_transaction {}", txn.operation_count, config.max_operations_per_transaction));
		if (txn.blob_length > config.max_blob_bytes)
			throw EngineException(EStatus::ResourceLimit, std::format("transaction blob is {} bytes, exceeding max_blob_bytes {}", txn.blob_length, config.max_blob_bytes));
		if (txn.blob_length != 0 && txn.blob == nullptr)
			throw EngineException(EStatus::InvalidStruct, "transaction blob_length is non-zero but blob is null");
		if (txn.operation_count != 0 && txn.operations == nullptr)
			throw EngineException(EStatus::InvalidStruct, "transaction operation_count is non-zero but operations is null");
		if (txn.control_count != 0 && txn.controls == nullptr)
			throw EngineException(EStatus::InvalidStruct, "transaction control_count is non-zero but controls is null");
		if (txn.resource_removal_count != 0 && txn.resource_removals == nullptr)
			throw EngineException(EStatus::InvalidStruct, "transaction resource_removal_count is non-zero but resource_removals is null");
		if (txn.style_removal_count != 0 && txn.style_removals == nullptr)
			throw EngineException(EStatus::InvalidStruct, "transaction style_removal_count is non-zero but style_removals is null");
		if (txn.template_removal_count != 0 && txn.template_removals == nullptr)
			throw EngineException(EStatus::InvalidStruct, "transaction template_removal_count is non-zero but template_removals is null");

		// Stage all mutation on a copy; 'next' is only ever returned once every check has passed,
		// which makes rejection-preserves-last-accepted-snapshot trivially correct (section 5.3).
		TreeModel next = *this;

		// Merge resources/styles/templates first (upsert-by-id) so operations below can already
		// resolve template_id/style_id/resource references introduced by this same transaction.
		for (auto i = std::uint32_t{}; i != txn.resource_count; ++i)
		{
			auto const& r = txn.resources[i];
			ValidateHeader<ResourceDesc>(r.header, "ResourceDesc");
			if (r.id == 0)
				throw EngineException(EStatus::InvalidArgument, "resource id 0 is reserved");
			if (static_cast<std::uint32_t>(r.kind) >= static_cast<std::uint32_t>(EResourceKind::Count))
				throw EngineException(EStatus::UnknownType, std::format("resource {}: unknown EResourceKind {}", r.id, static_cast<int>(r.kind)));

			ResourceRecord rec{};
			rec.desc = r;
			rec.name = CopyBlobRange(txn.blob, txn.blob_length, r.name_offset, r.name_length, "resource name");
			next.m_resources[r.id] = std::move(rec);
		}
		if (next.m_resources.size() > config.max_resources)
			throw EngineException(EStatus::ResourceLimit, std::format("{} resources exceeds max_resources {}", next.m_resources.size(), config.max_resources));

		for (auto i = std::uint32_t{}; i != txn.style_count; ++i)
		{
			auto const& s = txn.styles[i];
			ValidateHeader<StyleDesc>(s.header, "StyleDesc");
			if (s.id == 0)
				throw EngineException(EStatus::InvalidArgument, "style id 0 is reserved");
			for (auto c = std::size_t{}; c != static_cast<std::size_t>(EStateChannel::Count); ++c)
			{
				auto const& t = s.transitions[c];
				if (!std::isfinite(t.duration_ms) || t.duration_ms < 0.0f)
					throw EngineException(EStatus::InvalidArgument, std::format("style {}: transition duration {} must be finite and non-negative", s.id, t.duration_ms));
				if (t.duration_ms > config.max_transition_duration_ms)
					throw EngineException(EStatus::ResourceLimit, std::format("style {}: transition duration {} exceeds max_transition_duration_ms {}", s.id, t.duration_ms, config.max_transition_duration_ms));
				if (static_cast<std::uint32_t>(t.easing) >= static_cast<std::uint32_t>(EEasing::Count))
					throw EngineException(EStatus::UnknownType, std::format("style {}: unknown EEasing {}", s.id, static_cast<int>(t.easing)));
			}
			next.m_styles[s.id] = StyleRecord{ s };
		}
		if (next.m_styles.size() > config.max_styles)
			throw EngineException(EStatus::ResourceLimit, std::format("{} styles exceeds max_styles {}", next.m_styles.size(), config.max_styles));

		for (auto i = std::uint32_t{}; i != txn.template_count; ++i)
		{
			auto const& t = txn.templates[i];
			ValidateHeader<TemplateDesc>(t.header, "TemplateDesc");
			if (t.id == 0)
				throw EngineException(EStatus::InvalidArgument, "template id 0 is reserved");
			if (static_cast<std::uint32_t>(t.applies_to) >= static_cast<std::uint32_t>(EControlType::Count))
				throw EngineException(EStatus::UnknownType, std::format("template {}: unknown EControlType {}", t.id, static_cast<int>(t.applies_to)));
			if (t.part_count > VIEW3D_UI_MAX_TEMPLATE_PARTS)
				throw EngineException(EStatus::ResourceLimit, std::format("template {}: {} parts exceeds VIEW3D_UI_MAX_TEMPLATE_PARTS {}", t.id, t.part_count, VIEW3D_UI_MAX_TEMPLATE_PARTS));

			TemplateRecord rec{};
			rec.desc = t;
			std::unordered_set<std::string> seen_names;
			for (auto p = std::uint32_t{}; p != t.part_count; ++p)
			{
				auto const& part = t.parts[p];
				if (static_cast<std::uint32_t>(part.primitive) >= static_cast<std::uint32_t>(EVisualPrimitive::Count))
					throw EngineException(EStatus::UnknownType, std::format("template {}: unknown EVisualPrimitive {}", t.id, static_cast<int>(part.primitive)));

				rec.part_names[p] = CopyBlobRange(txn.blob, txn.blob_length, part.name_offset, part.name_length, "template part name");
				if (rec.part_names[p].empty())
					throw EngineException(EStatus::InvalidArgument, std::format("template {}: part {} has an empty name", t.id, p));
				if (!seen_names.insert(rec.part_names[p]).second)
					throw EngineException(EStatus::InvalidTree, std::format("template {}: duplicate part name '{}'", t.id, rec.part_names[p]));
			}

			// A template must supply every required part for the control type it applies to.
			for (auto required_name : RequiredTemplateParts(t.applies_to))
			{
				auto found = std::find(rec.part_names, rec.part_names + t.part_count, required_name) != rec.part_names + t.part_count;
				if (!found)
					throw EngineException(EStatus::InvalidTree, std::format("template {}: missing required part '{}' for control type {}", t.id, required_name, static_cast<int>(t.applies_to)));
			}
			next.m_templates[t.id] = std::move(rec);
		}
		if (next.m_templates.size() > config.max_templates)
			throw EngineException(EStatus::ResourceLimit, std::format("{} templates exceeds max_templates {}", next.m_templates.size(), config.max_templates));

		// Apply operations in the order given; each one is fully validated as it is applied.
		for (auto i = std::uint32_t{}; i != txn.operation_count; ++i)
		{
			auto const& op = txn.operations[i];
			ValidateHeader<Operation>(op.header, "Operation");
			switch (op.kind)
			{
				case EOperationKind::Remove:
				{
					if (op.target_id == 0 || next.m_controls.find(op.target_id) == next.m_controls.end())
						throw EngineException(EStatus::InvalidTree, std::format("Remove: control {} does not exist", op.target_id));
					RemoveSubtree(next, op.target_id);
					break;
				}
				case EOperationKind::ReplaceSubtree:
				{
					// Deleting an absent subtree is a no-op: the following Upsert(s) simply create
					// 'target_id' fresh, which is indistinguishable from a first-time Upsert.
					RemoveSubtree(next, op.target_id);
					break;
				}
				case EOperationKind::Upsert:
				{
					// Find the matching ControlDesc in the transaction's flat control array.
					ControlDesc const* found = nullptr;
					for (auto c = std::uint32_t{}; c != txn.control_count; ++c)
					{
						ValidateHeader<ControlDesc>(txn.controls[c].header, "ControlDesc");
						if (txn.controls[c].id == op.target_id)
						{
							if (found != nullptr)
								throw EngineException(EStatus::InvalidTree, std::format("Upsert: control {} appears more than once in this transaction", op.target_id));
							found = &txn.controls[c];
						}
					}
					if (found == nullptr)
						throw EngineException(EStatus::InvalidTree, std::format("Upsert: no ControlDesc with id {} was supplied", op.target_id));

					ValidateControlDescLocal(*found);

					// template_id/style_id of 0 select the built-in defaults; non-zero values must
					// already be registered (by this transaction's merge step above, or earlier).
					if (found->template_id != 0 && next.m_templates.find(found->template_id) == next.m_templates.end())
						throw EngineException(EStatus::UnknownResource, std::format("control {}: unknown template_id {}", found->id, found->template_id));
					if (found->style_id != 0 && next.m_styles.find(found->style_id) == next.m_styles.end())
						throw EngineException(EStatus::UnknownResource, std::format("control {}: unknown style_id {}", found->id, found->style_id));

					// font_resource_id of 0 selects the built-in Segoe UI fallback; a non-zero value
					// must already be registered and must specifically name a Font resource, not a
					// Colour one, so the renderer never has to defensively re-check the kind later.
					if (found->font_resource_id != 0)
					{
						auto font_it = next.m_resources.find(found->font_resource_id);
						if (font_it == next.m_resources.end())
							throw EngineException(EStatus::UnknownResource, std::format("control {}: unknown font_resource_id {}", found->id, found->font_resource_id));
						if (font_it->second.desc.kind != EResourceKind::Font)
							throw EngineException(EStatus::UnknownResource, std::format("control {}: font_resource_id {} does not reference a Font resource", found->id, found->font_resource_id));
					}

					auto existing = next.m_controls.find(found->id);
					if (existing == next.m_controls.end())
					{
						// First-time Upsert: the parent must already exist (unless this is itself a
						// Root, whose implicit parent is the viewport). New children always append
						// at the end of their parent's order; explicit Reorder operations or a
						// ChildOrder supplied for the same parent may permute this afterwards.
						if (found->type != EControlType::Root)
						{
							auto parent_it = next.m_controls.find(found->parent_id);
							if (parent_it == next.m_controls.end())
								throw EngineException(EStatus::InvalidTree, std::format("Upsert: control {}'s parent {} does not exist yet", found->id, found->parent_id));
							parent_it->second.children.push_back(found->id);
						}
						else
						{
							next.m_roots.push_back(found->id);
						}

						ControlNode node{};
						node.desc = *found;
						node.text = CopyBlobRange(txn.blob, txn.blob_length, found->text_offset, found->text_length, "control text");
						node.name = CopyBlobRange(txn.blob, txn.blob_length, found->name_offset, found->name_length, "control name");
						node.description = CopyBlobRange(txn.blob, txn.blob_length, found->desc_offset, found->desc_length, "control description");
						next.m_controls.emplace(found->id, std::move(node));
					}
					else
					{
						// Updating an existing control cannot re-parent it: re-parenting is done by
						// an explicit Remove + Upsert pair so child-order/depth invariants stay easy
						// to reason about.
						if (existing->second.desc.parent_id != found->parent_id)
							throw EngineException(EStatus::InvalidTree, std::format("Upsert: control {} cannot change parent from {} to {}; remove and re-add instead", found->id, existing->second.desc.parent_id, found->parent_id));

						existing->second.desc = *found;
						existing->second.text = CopyBlobRange(txn.blob, txn.blob_length, found->text_offset, found->text_length, "control text");
						existing->second.name = CopyBlobRange(txn.blob, txn.blob_length, found->name_offset, found->name_length, "control name");
						existing->second.description = CopyBlobRange(txn.blob, txn.blob_length, found->desc_offset, found->desc_length, "control description");
					}
					break;
				}
				case EOperationKind::Reorder:
				{
					if (op.child_order_index >= txn.child_order_count)
						throw EngineException(EStatus::InvalidArgument, std::format("Reorder: child_order_index {} is out of range (child_order_count {})", op.child_order_index, txn.child_order_count));

					auto const& order = txn.child_orders[op.child_order_index];
					if (order.parent_id != op.target_id)
						throw EngineException(EStatus::InvalidTree, std::format("Reorder: ChildOrder.parent_id {} does not match Operation.target_id {}", order.parent_id, op.target_id));

					auto parent_it = next.m_controls.find(op.target_id);
					if (parent_it == next.m_controls.end())
						throw EngineException(EStatus::InvalidTree, std::format("Reorder: control {} does not exist", op.target_id));
					if (order.offset > txn.child_id_count || order.count > txn.child_id_count - order.offset)
						throw EngineException(EStatus::InvalidStruct, std::format("Reorder: child id range [{}, {}) exceeds child_id_count {}", order.offset, order.offset + order.count, txn.child_id_count));

					std::vector<ControlId> permuted(txn.child_ids + order.offset, txn.child_ids + order.offset + order.count);
					auto current = parent_it->second.children; // copy for order-independent comparison
					auto sorted_permuted = permuted;
					auto sorted_current = current;
					std::sort(sorted_permuted.begin(), sorted_permuted.end());
					std::sort(sorted_current.begin(), sorted_current.end());
					if (sorted_permuted != sorted_current)
						throw EngineException(EStatus::InvalidTree, std::format("Reorder: the supplied child list for control {} is not a permutation of its current children", op.target_id));

					parent_it->second.children = std::move(permuted);
					break;
				}
				case EOperationKind::Count:
				default:
				{
					throw EngineException(EStatus::UnknownType, std::format("unknown EOperationKind {}", static_cast<int>(op.kind)));
				}
			}
		}

		// Remove resources/styles/templates after operations so a transaction may both stop
		// referencing an id (via Upsert) and remove it in the same call, but before the whole-tree
		// consistency pass so that pass's template-part checks run against the post-removal state.
		// Three explicit, near-identical loops rather than one generic helper, matching the merge
		// step above which likewise duplicates per-table rather than sharing a template.
		for (auto i = std::uint32_t{}; i != txn.resource_removal_count; ++i)
		{
			auto const id = txn.resource_removals[i];
			if (id == 0 || next.m_resources.find(id) == next.m_resources.end())
				throw EngineException(EStatus::UnknownResource, std::format("resource removal: unknown resource id {}", id));

			for (auto const& [control_id, node] : next.m_controls)
			{
				if (node.desc.font_resource_id == id)
					throw EngineException(EStatus::ResourceInUse, std::format("resource removal: resource {} is still referenced by control {}", id, control_id));
			}
			next.m_resources.erase(id);
		}

		for (auto i = std::uint32_t{}; i != txn.style_removal_count; ++i)
		{
			auto const id = txn.style_removals[i];
			if (id == 0 || next.m_styles.find(id) == next.m_styles.end())
				throw EngineException(EStatus::UnknownResource, std::format("style removal: unknown style id {}", id));

			for (auto const& [control_id, node] : next.m_controls)
			{
				if (node.desc.style_id == id)
					throw EngineException(EStatus::ResourceInUse, std::format("style removal: style {} is still referenced by control {}", id, control_id));
			}
			next.m_styles.erase(id);
		}

		for (auto i = std::uint32_t{}; i != txn.template_removal_count; ++i)
		{
			auto const id = txn.template_removals[i];
			if (id == 0 || next.m_templates.find(id) == next.m_templates.end())
				throw EngineException(EStatus::UnknownResource, std::format("template removal: unknown template id {}", id));

			for (auto const& [control_id, node] : next.m_controls)
			{
				if (node.desc.template_id == id)
					throw EngineException(EStatus::ResourceInUse, std::format("template removal: template {} is still referenced by control {}", id, control_id));
			}
			next.m_templates.erase(id);
		}

		// Whole-tree consistency and bounded-capacity validation (section 9.2/9.3).
		if (next.m_controls.size() > config.max_controls)
			throw EngineException(EStatus::ResourceLimit, std::format("{} controls exceeds max_controls {}", next.m_controls.size(), config.max_controls));
		if (next.m_roots.size() > config.max_roots)
			throw EngineException(EStatus::ResourceLimit, std::format("{} roots exceeds max_roots {}", next.m_roots.size(), config.max_roots));

		for (auto const& [id, node] : next.m_controls)
		{
			// Children/parent must be bidirectionally consistent: every id in a node's children
			// list must itself report that node as its parent, and vice versa (catches Reorder
			// permutation bugs and any orphaned attachment).
			for (auto child_id : node.children)
			{
				auto child_it = next.m_controls.find(child_id);
				if (child_it == next.m_controls.end() || child_it->second.desc.parent_id != id)
					throw EngineException(EStatus::InvalidTree, std::format("control {}: child {} is not consistently parented", id, child_id));
			}

			// Walk up to a Root, bounded by max_tree_depth + 1 steps, to reject cycles and orphans.
			auto depth = std::uint32_t{};
			auto walk_id = id;
			for (;;)
			{
				auto walk_it = next.m_controls.find(walk_id);
				if (walk_it == next.m_controls.end())
					throw EngineException(EStatus::InvalidTree, std::format("control {}: ancestor chain references a non-existent control {}", id, walk_id));
				if (walk_it->second.desc.parent_id == 0)
				{
					if (walk_it->second.desc.type != EControlType::Root)
						throw EngineException(EStatus::InvalidTree, std::format("control {}: ancestor chain terminates at non-root control {}", id, walk_id));
					break;
				}
				walk_id = walk_it->second.desc.parent_id;
				if (++depth > config.max_tree_depth)
					throw EngineException(EStatus::InvalidTree, std::format("control {}: ancestor chain exceeds max_tree_depth {} (cycle or overly deep tree)", id, config.max_tree_depth));
			}

			// A control's resolved template must supply every required part for its type.
			auto const& tmpl = node.desc.template_id == 0 ? DefaultTemplate(node.desc.type) : next.m_templates.at(node.desc.template_id);
			for (auto required_name : RequiredTemplateParts(node.desc.type))
			{
				auto found = false;
				for (auto p = std::uint32_t{}; p != tmpl.desc.part_count; ++p)
					found |= tmpl.part_names[p] == required_name;
				if (!found)
					throw EngineException(EStatus::InvalidTree, std::format("control {}: resolved template {} is missing required part '{}'", id, tmpl.desc.id, required_name));
			}
		}

		next.m_revision = txn.revision;
		return next;
	}
}
