//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
// Definition of the instance base class and built-in instances for the renderer.
// Usage:
//   Client code can use the instance structs provided here or define their own with
//   PR_RDR12_INSTANCE_MEMBERS. Use cast<InstanceType> when the concrete instance type is known.
//
// Instance data layout:
//   BaseInstance (references one immutable component layout shared by the instance type)
//   component
//   component
//   component
// Component offsets come from the compiler, including padding between members. The header must be the first member.
#pragma once
#include "pr/view3d-12/forward.h"
#include "pr/view3d-12/render/sortkey.h"
#include "pr/view3d-12/utility/pipe_state.h"

namespace pr::rdr12
{
	// Instance component types
	enum class EInstComp :uint8_t
	{
		None,                // invalid component
		ModelPtr,            // rdr::ModelPtr
		MaterialPtr,         // rdr::MaterialPtr
		I2WTransform,        // pr::m4x4
		I2WTransformPtr,     // pr::m4x4*
		I2WTransformFuncPtr, // m4x4Func (function and context)
		C2STransform,        // pr::m4x4
		C2SOptional,         // pr::m4x4 (set to m4x4::Zero() to indicate not used)
		C2STransformPtr,     // pr::m4x4*
		C2STransformFuncPtr, // m4x4Func (function and context)
		SortkeyOverride,     // rdr::SKOverride
		PipeStates,          // rdr::PipeStates
		Flags,               // EInstFlag
		TintColour32,        // pr::Colour32
		ColourBlend32,       // pr::Colour32 - sRGB target; alpha is the linear UNORM8 RGB blend weight, not opacity
		DitherWord,          // uint32 - screen-door dither word, see 'pr/view3d-12/shaders/population.hlsli'. Zero draws every pixel
		PosePtr,             // A skeleton pose to skin to
		EnvMapReflectivity,  // float
		UniqueId,            // int32
		SSSize,              // pr::v2 (screen space size)
		Float1,              // float - for general use
		Float2,              // pr::v2 - for general use
		Float4,              // pr::v4 - for general use
		EmissiveScale,       // float - multiplies the material emissive colour; missing means 1
	};

	// Instance flags
	enum class EInstFlag :uint32_t
	{
		None = 0,

		// The object to world transform is not an affine transform
		NonAffine = 1 << 5,

		// Doesn't cast a shadow
		ShadowCastExclude = 1 << 12,

		// Not rendered by scenes that exclude environment map capture content
		EnvMapCaptureExclude = 1 << 15,

		// flags
		_flags_enum = 0,
	};

	// A component that gets a transform via function pointer.
	struct m4x4Func
	{
		using m4x4FuncPtr = m4x4 const& (*)(void* ctx);

		m4x4FuncPtr m_func;
		void*       m_ctx;

		// Return the transform supplied by the function and its borrowed context.
		m4x4 const& Txfm() const
		{
			// Keep the context paired with the function that interprets it.
			return m_func(m_ctx);
		}
	};

	// Return whether a declared member provides the storage required by its renderer component.
	template <typename Comp> constexpr bool IsComponentType(EInstComp comp)
	{
		// Check the complete value type, including both words of a transform function component.
		switch (comp)
		{
			case EInstComp::None:                { return false; }
			case EInstComp::ModelPtr:            { return std::is_same_v<Comp, ModelPtr>; }
			case EInstComp::MaterialPtr:         { return std::is_same_v<Comp, MaterialPtr>; }
			case EInstComp::PosePtr:             { return std::is_same_v<Comp, PosePtr>; }
			case EInstComp::I2WTransform:        { return std::is_same_v<Comp, m4x4>; }
			case EInstComp::I2WTransformPtr:     { return std::is_same_v<Comp, m4x4*> || std::is_same_v<Comp, m4x4 const*>; }
			case EInstComp::I2WTransformFuncPtr: { return std::is_same_v<Comp, m4x4Func>; }
			case EInstComp::C2STransform:        { return std::is_same_v<Comp, m4x4>; }
			case EInstComp::C2SOptional:         { return std::is_same_v<Comp, m4x4>; }
			case EInstComp::C2STransformPtr:     { return std::is_same_v<Comp, m4x4*> || std::is_same_v<Comp, m4x4 const*>; }
			case EInstComp::C2STransformFuncPtr: { return std::is_same_v<Comp, m4x4Func>; }
			case EInstComp::SortkeyOverride:     { return std::is_same_v<Comp, SKOverride>; }
			case EInstComp::PipeStates:          { return std::is_same_v<Comp, PipeStates>; }
			case EInstComp::Flags:               { return std::is_same_v<Comp, EInstFlag>; }
			case EInstComp::TintColour32:        { return std::is_same_v<Comp, Colour32>; }
			case EInstComp::ColourBlend32:       { return std::is_same_v<Comp, Colour32>; }
			case EInstComp::DitherWord:          { return std::is_same_v<Comp, uint32_t>; }
			case EInstComp::EnvMapReflectivity:  { return std::is_same_v<Comp, float>; }
			case EInstComp::UniqueId:            { return std::is_same_v<Comp, int32_t>; }
			case EInstComp::SSSize:              { return std::is_same_v<Comp, v2>; }
			case EInstComp::Float1:              { return std::is_same_v<Comp, float>; }
			case EInstComp::Float2:              { return std::is_same_v<Comp, v2>; }
			case EInstComp::Float4:              { return std::is_same_v<Comp, v4>; }
			case EInstComp::EmissiveScale:       { return std::is_same_v<Comp, float>; }
			default: { throw std::runtime_error("Unknown instance component type"); }
		}
	}

	// One member in an instance type's immutable layout, in declaration order.
	struct InstanceComponent
	{
		EInstComp m_type;
		size_t m_offset;
	};

	// The header for an instance. All instances must start with one of these
	struct BaseInstance
	{
		std::span<InstanceComponent const> m_components;

		// Enumerate the shared component layout without permitting per-instance changes.
		InstanceComponent const* begin() const
		{
			// Use raw iteration over the immutable entries during component lookup.
			return m_components.data();
		}
		InstanceComponent const* end() const
		{
			// The shared span bounds include duplicates and exclude unrelated object members.
			return m_components.data() + m_components.size();
		}

		// Access the component at 'ofs'
		template <typename Comp> Comp const* get(size_t ofs) const
		{
			return reinterpret_cast<Comp const*>(byte_ptr(this) + ofs);
		}
		template <typename Comp> Comp* get(size_t ofs)
		{
			return reinterpret_cast<Comp*>(byte_ptr(this) + ofs);
		}

		// Find the 'index'th component of type 'comp' in this instance. Returns non-null if the component was found
		template <typename Comp> Comp const* find(EInstComp comp, int index = 0) const
		{
			// Preserve duplicate occurrence order while using the compiler's member offset directly.
			for (auto const& c : *this)
			{
				// Count only matches so unrelated components and padding cannot affect the requested index.
				if (c.m_type == comp && index-- == 0)
					return get<Comp>(c.m_offset);
			}
			return nullptr;
		}
		template <typename Comp> Comp* find(EInstComp comp, int index = 0)
		{
			// The shared layout is immutable; only the selected component in this mutable instance may change.
			return const_cast<Comp*>(static_cast<BaseInstance const&>(*this).find<Comp>(comp, index));
		}

		// Get the 'index'th component in this instance
		template <typename Comp> inline Comp const& get(EInstComp comp, int index = 0) const
		{
			auto c = find<Comp>(comp, index);
			if (c == nullptr) throw std::exception("This instance does not have the requested component");
			return *c;
		}
		template <typename Comp> inline Comp& get(EInstComp comp, int index = 0)
		{
			auto c = find<Comp>(comp, index);
			if (c == nullptr) throw std::exception("This instance does not have the requested component");
			return *c;
		}
	};

	// Return a pointer to the model that this is an instance of.
	ModelPtr const& GetModel(BaseInstance const& inst);

	// Return the chain of nuggets to render for this instance
	NuggetPtr GetNuggets(BaseInstance const& inst);

	// Return true if this instance requires alpha blending.
	bool HasAlpha(BaseInstance const& inst);

	// Return the instance to world transform for an instance.
	// An instance must have an i2w transform or a shared i2w transform.
	m4x4 const& GetO2W(BaseInstance const& inst);

	// Look for a camera to screen (or instance specific projection) transform for an instance.
	// Returns null if the instance doesn't have one.
	bool FindC2S(BaseInstance const& inst, m4x4& camera_to_screen);

	// Return the instance flags associated with 'inst'
	EInstFlag GetFlags(BaseInstance const& inst);

	// Return the id assigned to this instance, or '0' if not found
	int UniqueId(BaseInstance const& inst);

	// Return any pipe state overrides in the instance
	PipeStates const& GetPipeStates(BaseInstance const& inst);

	// Return the material to render this instance with.
	MaterialPtr FindMaterial(BaseInstance const& inst);

	// Return the skin override in this instance (if exists)
	PosePtr FindPose(BaseInstance const& inst);

	// Cast from a 'BaseInstance' pointer to an instance type
	template <typename InstType> constexpr InstType const* cast(BaseInstance const* base_ptr)
	{
		return type_ptr<InstType>(byte_ptr(base_ptr) - offsetof(InstType, m_base));
	}
	template <typename InstType> constexpr InstType* cast(BaseInstance* base_ptr)
	{
		return type_ptr<InstType>(byte_ptr(base_ptr) - offsetof(InstType, m_base));
	}

	// Use this to define class types that are compatible with the renderer
	// Example:
	//  struct MyInstanceType
	//  {
	//     #define PR_RDR_INST(x)\
	//     x(rdr12::ModelPtr ,m_model  ,rdr12::EInstComp::ModelPtr)\
	//     x(pr::Colour32        ,m_colour ,rdr12::EInstComp::TintColour32)
	//     PR_RDR12_INSTANCE_MEMBERS(MyInstance, PR_RDR_INST);
	//     #undef PR_RDR_INST
	//  };
	#pragma region Instance Type Generator

	#pragma region Macro Helpers
	#define PR_RDR12_INST_MEMBERS(ty,nm,em)           ty nm;
	#define PR_RDR12_INST_MEMBER_COUNT(ty,nm,em)      + 1
	#define PR_RDR12_INST_INITIALISERS(ty,nm,em)      ,nm()
	#define PR_RDR12_INST_COMPONENTS(ty,nm,em)        {em, offsetof(ThisType, nm)},
	#define PR_RDR12_INST_TYPE_CHECK(ty,nm,em)        static_assert(pr::rdr12::IsComponentType<ty>(em), "Instance member type does not match its component");
	#pragma endregion

	#define PR_RDR12_INSTANCE_MEMBERS(name, fields)\
		static constexpr int CompCount = 0 fields(PR_RDR12_INST_MEMBER_COUNT);\
		pr::rdr12::BaseInstance m_base;\
		fields(PR_RDR12_INST_MEMBERS)\
		\
		/* Return the single immutable compiler-derived layout for this instance type. */\
		static std::span<pr::rdr12::InstanceComponent const> InstanceComponents()\
		{\
			/* Define offsets only after the enclosing type is complete; no instance data is captured. */\
			using ThisType = name;\
			fields(PR_RDR12_INST_TYPE_CHECK)\
			static_assert(offsetof(ThisType, m_base) == 0, "'m_base' must be the first member");\
			static constexpr std::array<pr::rdr12::InstanceComponent, CompCount> components = {{fields(PR_RDR12_INST_COMPONENTS)}};\
			return components;\
		}\
		\
		/* Initialise component values and reference this type's shared layout. */\
		name()\
			:m_base{InstanceComponents()}\
			fields(PR_RDR12_INST_INITIALISERS)\
		{}

	#pragma endregion
}
