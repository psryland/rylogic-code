//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
// Definition of the instance base class and built-in instances for the renderer.
// Usage:
//   Client code can use the instance structs provided here or derive there own from
//   InstanceBase. If custom instances are used in conjunction with custom shaders
//   dynamic casts should be used to down-cast the instance struct to the appropriate type.
//
// Instance data layout:
//   BaseInstance
//   EInstComp[NumCpts]
//   component
//   component
//   component
#pragma once
#include "pr/view3d-12/forward.h"
#include "pr/view3d-12/render/sortkey.h"
#include "pr/view3d-12/utility/pipe_state.h"

namespace pr::rdr12
{
	// Instance component types
	enum class EInstComp :uint8_t
	{
		None,                // invalid entry (used for padding)
		ModelPtr,            // rdr::ModelPtr
		I2WTransform,        // pr::m4x4
		I2WTransformPtr,     // pr::m4x4*
		I2WTransformFuncPtr, // pr::m4x4 const& (*func)(void* context);
		C2STransform,        // pr::m4x4
		C2SOptional,         // pr::m4x4 (set to m4x4::Zero() to indicate not used)
		C2STransformPtr,     // pr::m4x4*
		C2STransformFuncPtr, // pr::m4x4 const& (*func)(void* context);
		SortkeyOverride,     // rdr::SKOverride
		PipeStates,          // rdr::PipeStates
		Flags,               // EInstFlag
		TintColour32,        // pr::Colour32
		DiffTexture,         // Texture2DPtr. An override of the main diffuse texture
		DiffTextureSampler,  // SamplerPtr. An override of the main diffuse texture sampler
		PosePtr,             // A skeleton pose to skin to
		EnvMapReflectivity,  // float
		UniqueId,            // int32
		SSSize,              // pr::v2 (screen space size)
		Float1,              // float - for general use
		Float2,              // pr::v2 - for general use
		Float4,              // pr::v4 - for general use
	};
	static_assert(sizeof(EInstComp) == 1, "Padding of Instance types relies on this");

	// Instance flags
	enum class EInstFlag :uint32_t
	{
		None = 0,

		// The object to world transform is not an affine transform
		NonAffine = 1 << 5,

		// Doesn't cast a shadow
		ShadowCastExclude = 1 << 12,

		// flags
		_flags_enum = 0,
	};

	// The size of an instance component
	constexpr int SizeOf(EInstComp const comp)
	{
		switch (comp)
		{
			case EInstComp::None:                return 0;
			case EInstComp::ModelPtr:            return sizeof(ModelPtr);
			case EInstComp::PosePtr:             return sizeof(PosePtr);
			case EInstComp::I2WTransform:        return sizeof(m4x4);
			case EInstComp::I2WTransformPtr:     return sizeof(m4x4*);
			case EInstComp::I2WTransformFuncPtr: return sizeof(m4x4 const& (*)(void* context));
			case EInstComp::C2STransform:        return sizeof(m4x4);
			case EInstComp::C2SOptional:         return sizeof(m4x4);
			case EInstComp::C2STransformPtr:     return sizeof(m4x4*);
			case EInstComp::C2STransformFuncPtr: return sizeof(m4x4 const& (*)(void* context));
			case EInstComp::SortkeyOverride:     return sizeof(SKOverride);
			case EInstComp::PipeStates:          return sizeof(PipeStates);
			case EInstComp::Flags:               return sizeof(EInstFlag);
			case EInstComp::TintColour32:        return sizeof(Colour32);
			case EInstComp::DiffTexture:         return sizeof(Texture2DPtr);
			case EInstComp::DiffTextureSampler:  return sizeof(SamplerPtr);
			case EInstComp::EnvMapReflectivity:  return sizeof(float);
			case EInstComp::UniqueId:            return sizeof(int32_t);
			case EInstComp::SSSize:              return sizeof(v2);
			case EInstComp::Float1:              return sizeof(float);
			case EInstComp::Float2:              return sizeof(v2);
			case EInstComp::Float4:              return sizeof(v4);
			default: throw std::runtime_error("Unknown instance component type");
		}
	}

	// The header for an instance. All instances must start with one of these
	struct BaseInstance
	{
		int m_cpt_count;

		static BaseInstance make(int cpt_count)
		{
			BaseInstance b = {cpt_count};
			return b;
		}

		// Enumerate the component types
		EInstComp const* begin() const
		{
			return pr::type_ptr<EInstComp>(this + 1);
		}
		EInstComp const* end() const
		{
			return begin() + m_cpt_count;
		}
		EInstComp* begin()
		{
			return pr::type_ptr<EInstComp>(this + 1);
		}
		EInstComp* end()
		{
			return begin() + m_cpt_count;
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
			auto byte_ofs = PadTo<size_t>(sizeof(BaseInstance) + m_cpt_count * sizeof(EInstComp), 16);
			for (auto& c : *this)
			{
				if (c == comp && index-- == 0) return get<Comp>(byte_ofs);
				byte_ofs += SizeOf(c);
			}
			return nullptr;
		}
		template <typename Comp> Comp* find(EInstComp comp, int index = 0)
		{
			auto byte_ofs = PadTo<size_t>(sizeof(BaseInstance) + m_cpt_count * sizeof(EInstComp), 16);
			for (auto& c : *this)
			{
				if (c == comp && index-- == 0) return get<Comp>(byte_ofs);
				byte_ofs += SizeOf(c);
			}
			return nullptr;
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

	// A component that gets a transform via function pointer.
	struct m4x4Func
	{
		using m4x4FuncPtr = m4x4 const& (*)(void* ctx);

		m4x4FuncPtr m_func;
		void*       m_ctx;

		m4x4 const& Txfm() const
		{
			return m_func(m_ctx);
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

	// Return the texture override in this instance (if exists)
	Texture2DPtr FindDiffTexture(BaseInstance const& inst);

	// Return the sampler override in this instance (if exists)
	SamplerPtr FindDiffTextureSampler(BaseInstance const& inst);

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
	#define PR_RDR12_INST_INIT_COMPONENTS(ty,nm,em)   m_cpt[i++] = em;
	#define PR_RDR12_INST_COMPONENT_SIZES(ty,nm,em)   SizeOf(em),
	#define PR_RDR12_INST_ALIGNMENT_CHECK(ty,nm,em)   if (offsetof(ThisType, nm) == ofs) { ofs += Sizes[i++]; } else { return false; }
	#pragma endregion

	#define PR_RDR12_INSTANCE_MEMBERS(name, fields)\
		static constexpr int CompCount = 0 fields(PR_RDR12_INST_MEMBER_COUNT);\
		static constexpr int CompCapacity = CompCount + static_cast<int>(pr::Pad(sizeof(pr::rdr12::BaseInstance) + CompCount*sizeof(pr::rdr12::EInstComp), 16U));\
		pr::rdr12::BaseInstance m_base;\
		pr::rdr12::EInstComp m_cpt[CompCapacity];\
		fields(PR_RDR12_INST_MEMBERS)\
		\
		name()\
			:m_base({CompCount})\
			,m_cpt()\
			fields(PR_RDR12_INST_INITIALISERS)\
		{\
			using namespace pr::rdr12;\
			using ThisType = name;\
			int i = 0;\
			fields(PR_RDR12_INST_INIT_COMPONENTS)\
			constexpr auto AlignCheck = []()\
			{\
				constexpr int Sizes[] = {fields(PR_RDR12_INST_COMPONENT_SIZES)};\
				int i = 0, ofs = static_cast<int>(pr::PadTo(sizeof(BaseInstance) + CompCount*sizeof(EInstComp), 16));\
				fields(PR_RDR12_INST_ALIGNMENT_CHECK)\
				return true;\
			};\
			static_assert(AlignCheck(), "Member alignment issue");\
			static_assert(offsetof(name, m_base) == 0, "'m_base' must be be the first member");\
		}

	#pragma endregion
}