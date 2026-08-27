//*********************************************
// View3DUI
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#include "text_layout.h"

namespace pr::view3d::ui
{
	ResolvedFont ResolveControlFont(TreeModel const& tree, ResourceId font_resource_id)
	{
		if (font_resource_id != 0)
		{
			auto it = tree.m_resources.find(font_resource_id);
			if (it != tree.m_resources.end())
			{
				// A named family is passed through verbatim so an unknown name surfaces as
				// MissingAsset from the shaper; only the "no name at all" case takes the fallback.
				auto const& family = it->second.name.empty() ? std::string(DefaultFontFamily) : it->second.name;
				auto size = it->second.desc.font_size > 0.0f ? it->second.desc.font_size : DefaultFontSize;
				auto colour = it->second.desc.colour.a > 0.0f ? it->second.desc.colour : DefaultFontColour;
				return ResolvedFont{ .family = family, .size = size, .colour = colour };
			}
		}
		return ResolvedFont{ .family = DefaultFontFamily, .size = DefaultFontSize, .colour = DefaultFontColour };
	}

	TextPlacement TextPlacementFor(EControlType type)
	{
		switch (type)
		{
			case EControlType::Text:
			{
				return TextPlacement{ .align = ETextAlign::Left, .inset_dip = 0.0f }; // a bare label has no box of its own to inset from
			}
			case EControlType::TextBox:
			{
				return TextPlacement{ .align = ETextAlign::Left, .inset_dip = TextBoxHorizontalInsetDip };
			}
			case EControlType::Button:
			{
				return TextPlacement{ .align = ETextAlign::Center, .inset_dip = 0.0f };
			}
			case EControlType::Root:
			case EControlType::Panel:
			case EControlType::Count:
			default:
			{
				throw EngineException(EStatus::InvalidArgument, "control type has no text placement");
			}
		}
	}

	float ControlScale(TreeModel const& tree, std::unordered_map<ControlId, RootPlacement> const* placements, ControlId id)
	{
		if (placements == nullptr)
			return 1.0f;

		// Scale is a property of the whole root subtree, so walk to the root that owns 'id'.
		auto root_id = id;
		for (;;)
		{
			auto it = tree.m_controls.find(root_id);
			if (it == tree.m_controls.end())
				return 1.0f;

			if (it->second.desc.parent_id == 0)
				break;

			root_id = it->second.desc.parent_id;
		}

		auto placement_it = placements->find(root_id);
		return placement_it != placements->end() ? placement_it->second.scale : 1.0f;
	}
}
