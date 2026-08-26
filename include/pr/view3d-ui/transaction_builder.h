//*********************************************
// View3DUI
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// Reusable native transaction builder for the public C++ facade (implementation-plan.md section
// 5.3). Owns the borrowed arrays/blob one Transaction needs, so a native application composing a
// delta transaction (Upsert/Remove/Reorder/ReplaceSubtree plus resource/style/template upserts and
// removals) does not have to hand-manage parallel std::vectors and offset bookkeeping itself. This
// header is part of the STL-friendly facade layer (like view3d-ui.h), not the dependency-minimal
// ABI layer (types.h): only the values it stages and the Transaction it materialises are ABI types.
#pragma once
#include "pr/view3d-ui/forward.h"
#include "pr/view3d-ui/types.h"

namespace pr::view3d::ui
{
	// Accumulates one delta transaction's borrowed arrays (operations/controls/child order/
	// resources/styles/templates/removals/blob). Every accessor appends; nothing is ever removed
	// from the builder itself, so a Transaction returned by Build() remains valid until this
	// builder is destroyed or a further mutating call reallocates its backing vectors. Use one
	// TransactionBuilder per Transaction to keep that lifetime trivially safe.
	class TransactionBuilder
	{
		std::vector<Operation> m_operations;
		std::vector<ControlDesc> m_controls;
		std::vector<ChildOrder> m_child_orders;
		std::vector<ControlId> m_child_ids;
		std::vector<ResourceDesc> m_resources;
		std::vector<StyleDesc> m_styles;
		std::vector<TemplateDesc> m_templates;
		std::vector<ResourceId> m_resource_removals;
		std::vector<StyleId> m_style_removals;
		std::vector<TemplateId> m_template_removals;
		std::vector<std::byte> m_blob;

		// Populate a StructHeader for T at the current struct version, as every real caller must.
		template <typename T>
		static StructHeader Header()
		{
			return StructHeader{ .size = sizeof(T), .version = VIEW3D_UI_STRUCT_VERSION };
		}

	public:
		// Append 'text' to the shared blob, returning its (offset, length) for a *_offset/*_length pair.
		std::pair<std::uint32_t, std::uint32_t> AddText(std::string_view text)
		{
			auto offset = static_cast<std::uint32_t>(m_blob.size());
			auto bytes = std::as_bytes(std::span{ text });
			m_blob.insert(m_blob.end(), bytes.begin(), bytes.end());
			return { offset, static_cast<std::uint32_t>(text.size()) };
		}

		// Queue an Upsert of 'desc' (create-or-update semantics; section 5.3).
		void Upsert(ControlDesc const& desc)
		{
			m_controls.push_back(desc);
			m_operations.push_back(Operation{ .header = Header<Operation>(), .kind = EOperationKind::Upsert, .target_id = desc.id, .child_order_index = 0, .reserved0 = 0 });
		}

		// Queue an Upsert while packing the control's UTF-8 content and semantic strings.
		void Upsert(ControlDesc desc, std::string_view text, std::string_view name, std::string_view description = {})
		{
			std::tie(desc.text_offset, desc.text_length) = AddText(text);
			std::tie(desc.name_offset, desc.name_length) = AddText(name);
			std::tie(desc.desc_offset, desc.desc_length) = AddText(description);
			Upsert(static_cast<ControlDesc const&>(desc));
		}

		// Queue a Remove of 'id' and its entire subtree.
		void Remove(ControlId id)
		{
			m_operations.push_back(Operation{ .header = Header<Operation>(), .kind = EOperationKind::Remove, .target_id = id, .child_order_index = 0, .reserved0 = 0 });
		}

		// Queue a ReplaceSubtree of 'id'; any Upsert recreating 'id' in the same transaction must
		// follow this operation.
		void ReplaceSubtree(ControlId id)
		{
			m_operations.push_back(Operation{ .header = Header<Operation>(), .kind = EOperationKind::ReplaceSubtree, .target_id = id, .child_order_index = 0, .reserved0 = 0 });
		}

		// Queue a Reorder of 'parent_id''s children to exactly 'children' (must be a permutation of
		// the parent's current children once the transaction is staged).
		void Reorder(ControlId parent_id, std::initializer_list<ControlId> children)
		{
			auto offset = static_cast<std::uint32_t>(m_child_ids.size());
			for (auto id : children)
				m_child_ids.push_back(id);

			auto index = static_cast<std::uint32_t>(m_child_orders.size());
			m_child_orders.push_back(ChildOrder{ .parent_id = parent_id, .offset = offset, .count = static_cast<std::uint32_t>(children.size()) });
			m_operations.push_back(Operation{ .header = Header<Operation>(), .kind = EOperationKind::Reorder, .target_id = parent_id, .child_order_index = index, .reserved0 = 0 });
		}

		// Append a resource/style/template descriptor to this transaction's flat arrays (upsert-by-
		// id semantics: an id already present in the accepted tree is updated, not duplicated).
		void AddResource(ResourceDesc const& desc)
		{
			m_resources.push_back(desc);
		}

		// Append a named resource while packing its UTF-8 family/asset name into the shared blob.
		void AddNamedResource(ResourceDesc desc, std::string_view name)
		{
			std::tie(desc.name_offset, desc.name_length) = AddText(name);
			AddResource(desc);
		}
		void AddStyle(StyleDesc const& desc)
		{
			m_styles.push_back(desc);
		}
		void AddTemplate(TemplateDesc const& desc)
		{
			m_templates.push_back(desc);
		}

		// Queue removal of a previously-accepted resource/style/template id. Rejected atomically if
		// the id does not exist (EStatus::UnknownResource) or is still referenced by a control
		// present once this transaction's operations have applied (EStatus::ResourceInUse).
		void RemoveResource(ResourceId id)
		{
			m_resource_removals.push_back(id);
		}
		void RemoveStyle(StyleId id)
		{
			m_style_removals.push_back(id);
		}
		void RemoveTemplate(TemplateId id)
		{
			m_template_removals.push_back(id);
		}

		// Append one named part to a template being built locally (before it is handed to
		// AddTemplate), sourcing the part's name bytes from this builder's shared blob.
		void AddTemplatePart(TemplateDesc& tmpl, std::string_view name, EVisualPrimitive primitive, bool required = true)
		{
			if (tmpl.part_count >= VIEW3D_UI_MAX_TEMPLATE_PARTS)
				throw std::length_error("View3DUI template exceeds VIEW3D_UI_MAX_TEMPLATE_PARTS");

			auto [offset, length] = AddText(name);
			auto& part = tmpl.parts[tmpl.part_count++];
			part.name_offset = offset;
			part.name_length = length;
			part.primitive = primitive;
			part.required = required ? 1 : 0;
		}

		// Materialise the accumulated arrays as one Transaction, ready for immediate use with
		// UiContext::TransactionApply. The returned value borrows this builder's storage, so it
		// must not outlive (or be used after further mutation of) 'this'.
		Transaction Build(std::uint64_t base_revision, std::uint64_t revision) const
		{
			return Transaction{
				.header = Header<Transaction>(),
				.base_revision = base_revision,
				.revision = revision,
				.operations = m_operations.data(),
				.operation_count = static_cast<std::uint32_t>(m_operations.size()),
				.controls = m_controls.data(),
				.control_count = static_cast<std::uint32_t>(m_controls.size()),
				.child_orders = m_child_orders.data(),
				.child_order_count = static_cast<std::uint32_t>(m_child_orders.size()),
				.child_ids = m_child_ids.data(),
				.child_id_count = static_cast<std::uint32_t>(m_child_ids.size()),
				.resources = m_resources.data(),
				.resource_count = static_cast<std::uint32_t>(m_resources.size()),
				.styles = m_styles.data(),
				.style_count = static_cast<std::uint32_t>(m_styles.size()),
				.templates = m_templates.data(),
				.template_count = static_cast<std::uint32_t>(m_templates.size()),
				.resource_removals = m_resource_removals.data(),
				.resource_removal_count = static_cast<std::uint32_t>(m_resource_removals.size()),
				.style_removals = m_style_removals.data(),
				.style_removal_count = static_cast<std::uint32_t>(m_style_removals.size()),
				.template_removals = m_template_removals.data(),
				.template_removal_count = static_cast<std::uint32_t>(m_template_removals.size()),
				.blob = m_blob.data(),
				.blob_length = static_cast<std::uint32_t>(m_blob.size()),
			};
		}
	};
}
