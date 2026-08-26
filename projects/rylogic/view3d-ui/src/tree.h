//*********************************************
// View3DUI
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// Internal retained control tree: denormalized accepted state plus staging validation/apply for
// one delta Transaction (implementation-plan.md section 5.3).
#pragma once
#include "pr/view3d-ui/forward.h"
#include "pr/view3d-ui/types.h"
#include "pr/view3d-ui/engine.h"

namespace pr::view3d::ui
{
	// One accepted control, denormalized so the tree is independent of the transaction's borrowed
	// blob buffer once TransactionApply returns.
	struct ControlNode
	{
		ControlDesc desc;
		std::string text;
		std::string name;
		std::string description;
		std::vector<ControlId> children; // explicit, authoritative child order
	};

	// One accepted named resource/style/template, likewise denormalized from the transaction blob.
	struct ResourceRecord
	{
		ResourceDesc desc;
		std::string name;
	};
	struct StyleRecord
	{
		StyleDesc desc;
	};
	struct TemplateRecord
	{
		TemplateDesc desc;
		std::string part_names[VIEW3D_UI_MAX_TEMPLATE_PARTS];
	};

	// Immutable-once-accepted retained tree plus resource/style/template tables. TreeModel values
	// are copied wholesale during staging (see Apply), which keeps atomicity trivial to reason
	// about at the scale this module targets (thousands of controls, transactions applied at UI
	// edit rate rather than per frame).
	class TreeModel
	{
	public:
		std::uint64_t m_revision = 0;
		std::unordered_map<ControlId, ControlNode> m_controls;
		std::vector<ControlId> m_roots; // stable creation order of type==Root controls
		std::unordered_map<ResourceId, ResourceRecord> m_resources;
		std::unordered_map<StyleId, StyleRecord> m_styles;
		std::unordered_map<TemplateId, TemplateRecord> m_templates;

		// Validate 'txn' against '*this' and 'config', returning the fully-applied successor tree.
		// Throws EngineException on any rejection; '*this' is never mutated by a failed attempt because
		// all mutation happens on a local copy that is only returned once every check has passed.
		TreeModel Apply(Transaction const& txn, Config const& config) const;

		// The built-in template used when a control's template_id == 0.
		static TemplateRecord const& DefaultTemplate(EControlType type);

		// The built-in style (EStateChannel::Normal only) used when a control's style_id == 0.
		static StyleRecord const& DefaultStyle();
	};

	// Required template part names for closed control types with a required-parts contract
	// (section 6.3). Types absent from this table (Root/Panel/Text) require no specific parts.
	std::span<std::string_view const> RequiredTemplateParts(EControlType type);
}
