//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#pragma once
#include "pr/view3d-12/forward.h"

namespace pr::rdr12
{
	// Bit layout:
	// 11111111 11111111 11111111 11111111
	//                     ###### ######## texture id  lowest priority, most common thing changed when processing the drawlist
	//          ######## ##                shader id
	//        #                            has alpha
	// #######                             sort group  highest priority, least common thing changed when processing the drawlist
	//
	// General sorting notes: From the word of Al:
	// Z Buffering:
	//   Always try to maintain the z buffer (i.e. write enable) even for HUDs etc
	//   Stereoscopic rendering requires everything to have correct depth
	//   Render the sky box after all opaques to reduce overdraw
	// Alpha:
	//   Alpha nuggets are drawn once without culling. The K-buffer sorts their fragments by depth per pixel,
	//   so front and back faces do not need separate, ordered draws.
	//

	// Define sort groups
	enum class ESortGroup
	{
		// Notes:
		//  - Can't use a 2's complement value here because stuffing a negative value
		//    into the sortkey will mess up the ordering. This means that a sort key
		//    of 0 will *NOT* be in the default sort group.
		
		Min = 0,               // The minimum sort group value
		PreOpaques = 63,       // 
		Default = 64,          // Make opaques the middle group
		Skybox,                // Sky-box after opaques
		PostOpaques,           // 
		PreAlpha = Default+16, // Last group before the alpha group
		Alpha,                 // Alpha nuggets, collected into the K-buffer
		AlphaWeighted,         // Alpha nuggets drawn with weighted blended transparency (see ENuggetFlag::AlphaWeighted)
		PostAlpha,             // First group after the alpha groups
		Max = 127,             // The maximum sort group value

		_arith_enum = 0,
	};
	static_assert(is_arith_enum_v<ESortGroup>);

	// The sort key type (wraps a uint32)
	struct SortKey
	{
		using value_type = unsigned int;
		static constexpr int Bits = sizeof(value_type) * 8;

		// GGGGGGGA SSSSSSSS SSTTTTTT TTTTTTTT
		static constexpr int TextureIdBits = 14U;
		static constexpr int ShaderIdBits  = 10U;
		static constexpr int AlphaBits     = 1U ;
		static constexpr int SortGroupBits = Bits - (AlphaBits + ShaderIdBits + TextureIdBits);
		static_assert(Bits > AlphaBits + ShaderIdBits + TextureIdBits, "Sort key is not large enough");

		static constexpr int TextureIdOfs  = 0;
		static constexpr int ShaderIdOfs   = 0 + TextureIdBits;
		static constexpr int AlphaOfs      = 0 + TextureIdBits + ShaderIdBits;
		static constexpr int SortGroupOfs  = 0 + TextureIdBits + ShaderIdBits + AlphaBits;

		static constexpr value_type TextureIdMask = (~value_type() >> (Bits - TextureIdBits)) << TextureIdOfs;
		static constexpr value_type ShaderIdMask  = (~value_type() >> (Bits - ShaderIdBits )) << ShaderIdOfs ;
		static constexpr value_type AlphaMask     = (~value_type() >> (Bits - AlphaBits    )) << AlphaOfs    ;
		static constexpr value_type SortGroupMask = (~value_type() >> (Bits - SortGroupBits)) << SortGroupOfs;

		static constexpr value_type MaxTextureId  = 1U << TextureIdBits;
		static constexpr value_type MaxShaderId   = 1U << ShaderIdBits ;
		static constexpr value_type MaxSortGroups = 1U << SortGroupBits;
		static_assert(int(ESortGroup::Max) - int(ESortGroup::Min) < MaxSortGroups, "Not enough bits to represent the sort groups");

		value_type m_value;

		SortKey() = default;
		explicit SortKey(value_type value)
			:m_value(value)
		{}
		explicit SortKey(ESortGroup grp)
			:m_value()
		{
			Group(grp);
		}
		operator value_type() const
		{
			return m_value;
		}

		// Get/Set the sort group
		ESortGroup Group() const
		{
			return static_cast<ESortGroup>((m_value & SortGroupMask) >> SortGroupOfs);
		}
		void Group(ESortGroup group)
		{
			auto g = value_type(group);
			PR_ASSERT(PR_DBG_RDR, g >= 0 && g < MaxSortGroups, "sort group out of range");
			m_value = SetBits(m_value, SortGroupMask, g << SortGroupOfs);
		}

		SortKey& operator |= (SortKey::value_type rhs)
		{
			m_value |= rhs;
			return *this;
		}
		SortKey& operator &= (SortKey::value_type rhs)
		{ 
			m_value &= rhs;
			return *this;
		}
	};
	static_assert(std::is_trivially_copyable_v<SortKey>, "Sort key must be POD so that draw list elements are PODs");

	// A sort key override is a mask that is applied to a sort key
	// to override specific parts of the sort key.
	struct SKOverride
	{
		using value_type = SortKey::value_type;

		value_type m_mask; // The bits to override
		value_type m_key;  // The overridden bit values

		SKOverride()
			:m_mask(0)
			,m_key(0)
		{}

		// Combine this override with a sort key to produce a new sort key
		SortKey Combine(SortKey key) const
		{
			return SortKey(SetBits(key.m_value, m_mask, m_key));
		}

		// Get/Set the alpha component of the sort key
		bool HasAlpha() const
		{
			// True if we're overriding the alpha value
			return AllSet(m_mask, SortKey::AlphaMask);
		}
		bool Alpha() const
		{
			// The overridden state of the alpha value
			return (m_key & SortKey::AlphaMask) != 0;
		}
		SKOverride& ClearAlpha()
		{
			m_mask = SetBits(m_mask, SortKey::AlphaMask, false);
			m_key  = SetBits(m_key, SortKey::AlphaMask, false);
			return *this;
		}
		SKOverride& Alpha(bool has_alpha)
		{
			m_mask = SetBits(m_mask, SortKey::AlphaMask, true);
			m_key  = SetBits(m_key, SortKey::AlphaMask, has_alpha << SortKey::AlphaOfs);
			return *this;
		}

		// Get/Set the sort group component of the sort key
		bool HasGroup() const
		{
			// True if we're overriding the sort group
			return AllSet(m_mask, SortKey::SortGroupMask);
		}
		ESortGroup Group() const
		{
			// The value of the overridden sort group
			return static_cast<ESortGroup>((m_key & SortKey::SortGroupMask) >> SortKey::SortGroupOfs);
		}
		SKOverride& ClearGroup()
		{
			m_mask = SetBits(m_mask, SortKey::SortGroupMask, false);
			m_key  = SetBits(m_key, SortKey::SortGroupMask, false);
			return *this;
		}
		SKOverride& Group(ESortGroup group)
		{
			auto g = value_type(group);
			PR_ASSERT(PR_DBG_RDR, g >= 0 && g < SortKey::MaxSortGroups, "sort group out of range");
			m_mask = SetBits(m_mask, SortKey::SortGroupMask, true);
			m_key  = SetBits(m_key, SortKey::SortGroupMask, g << SortKey::SortGroupOfs);
			return *this;
		}
	};
}
