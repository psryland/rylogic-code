//*********************************************
// Renderer Instance Layout Tests
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#include "pr/common/unittests.h"
#include "pr/view3d-12/instance/instance.h"

namespace pr::unittests
{
	using namespace pr::rdr12;

	// Deliberately interleave alignment requirements, duplicate types, and two-word function components.
	struct PaddedInstance
	{
		#define PR_RDR_INST(x)\
		x(float    ,m_first ,EInstComp::Float1             )\
		x(v4       ,m_vector,EInstComp::Float4             )\
		x(float    ,m_second,EInstComp::Float1             )\
		x(m4x4Func ,m_i2w   ,EInstComp::I2WTransformFuncPtr)\
		x(uint32_t ,m_dither,EInstComp::DitherWord         )\
		x(m4x4Func ,m_c2s   ,EInstComp::C2STransformFuncPtr)\
		x(int32_t  ,m_id    ,EInstComp::UniqueId           )
		PR_RDR12_INSTANCE_MEMBERS(PaddedInstance, PR_RDR_INST);
		#undef PR_RDR_INST
	};

	// Resolve a borrowed matrix through the same function/context contract used by renderer accessors.
	m4x4 const& InstanceTransform(void* context)
	{
		// Each component's context identifies its own matrix.
		return *static_cast<m4x4 const*>(context);
	}

	// Exercise actual component addresses and values rather than reconstructing offsets in the test.
	PRUnitTestClass(RendererInstanceLayoutTests)
	{
		PRUnitTestMethod(PaddingDuplicatesAndMissingComponents, Quick)
		{
			// Compiler padding must not change lookup addresses or declaration-order duplicate indices.
			auto inst = PaddedInstance{};
			inst.m_first = 2.0f;
			inst.m_vector = v4{3, 5, 7, 11};
			inst.m_second = 13.0f;
			PR_EXPECT(byte_ptr(&inst.m_vector) > byte_ptr(&inst.m_first) + sizeof(inst.m_first));
			PR_EXPECT(inst.m_base.find<float>(EInstComp::Float1, 0) == &inst.m_first);
			PR_EXPECT(inst.m_base.find<float>(EInstComp::Float1, 1) == &inst.m_second);
			PR_EXPECT(inst.m_base.find<v4>(EInstComp::Float4) == &inst.m_vector);
			PR_EXPECT(inst.m_base.find<float>(EInstComp::Float1, 2) == nullptr);
			PR_EXPECT(inst.m_base.find<Colour32>(EInstComp::TintColour32) == nullptr);
			PR_THROWS(inst.m_base.get<Colour32>(EInstComp::TintColour32), std::exception);

			// Mutable access updates only the selected member; const access resolves the same live value.
			inst.m_base.get<float>(EInstComp::Float1, 1) = 17.0f;
			auto const& const_inst = inst;
			static_assert(std::is_same_v<decltype(const_inst.m_base.find<float>(EInstComp::Float1)), float const*>);
			PR_EXPECT(const_inst.m_base.get<float>(EInstComp::Float1, 0) == 2.0f);
			PR_EXPECT(const_inst.m_base.get<float>(EInstComp::Float1, 1) == 17.0f);
			PR_EXPECT(All(const_inst.m_base.get<v4>(EInstComp::Float4) == v4{3, 5, 7, 11}));
		}

		PRUnitTestMethod(FunctionContextAndFollowingFields, Quick)
		{
			// Both transform components contain a function and context, with another component after each.
			auto i2w = m4x4::Identity();
			auto c2s = m4x4::Identity();
			i2w.pos = v4{2, 3, 5, 1};
			c2s.x.x = 7.0f;
			auto inst = PaddedInstance{};
			inst.m_i2w = {InstanceTransform, &i2w};
			inst.m_dither = 0x12345678;
			inst.m_c2s = {InstanceTransform, &c2s};
			inst.m_id = 42;
			PR_EXPECT(&inst.m_base.get<m4x4Func>(EInstComp::I2WTransformFuncPtr) == &inst.m_i2w);
			PR_EXPECT(&inst.m_base.get<uint32_t>(EInstComp::DitherWord) == &inst.m_dither);
			PR_EXPECT(&inst.m_base.get<m4x4Func>(EInstComp::C2STransformFuncPtr) == &inst.m_c2s);
			PR_EXPECT(&inst.m_base.get<int32_t>(EInstComp::UniqueId) == &inst.m_id);
			PR_EXPECT(inst.m_base.get<m4x4Func>(EInstComp::I2WTransformFuncPtr).m_ctx == &i2w);
			PR_EXPECT(inst.m_base.get<m4x4Func>(EInstComp::C2STransformFuncPtr).m_ctx == &c2s);
			PR_EXPECT(inst.m_base.get<uint32_t>(EInstComp::DitherWord) == 0x12345678);
			PR_EXPECT(UniqueId(inst.m_base) == 42);
			PR_EXPECT(&GetO2W(inst.m_base) == &i2w);
			auto projection = m4x4::Zero();
			PR_EXPECT(FindC2S(inst.m_base, projection));
			PR_EXPECT(FEql(projection, c2s));

			// Reject pointer-only declarations for either two-word component at the construction boundary.
			static_assert(!IsComponentType<m4x4Func::m4x4FuncPtr>(EInstComp::I2WTransformFuncPtr));
			static_assert(!IsComponentType<m4x4Func::m4x4FuncPtr>(EInstComp::C2STransformFuncPtr));
		}

		PRUnitTestMethod(CopiesAndMovesShareLayoutNotValues, Quick)
		{
			// Copy and move operations must retain shared metadata without retaining pointers into source members.
			auto original = PaddedInstance{};
			original.m_first = 19.0f;
			original.m_second = 23.0f;
			original.m_id = 29;
			auto copied = original;
			auto moved = std::move(copied);
			auto copy_assigned = PaddedInstance{};
			copy_assigned = original;
			auto move_assigned = PaddedInstance{};
			move_assigned = std::move(copy_assigned);
			for (auto* inst : {&original, &copied, &moved, &copy_assigned, &move_assigned})
			{
				// Every object resolves its own members through exactly the same immutable table.
				PR_EXPECT(inst->m_base.begin() == original.m_base.begin());
				PR_EXPECT(inst->m_base.end() == original.m_base.end());
				PR_EXPECT(inst->m_base.find<float>(EInstComp::Float1, 1) == &inst->m_second);
				PR_EXPECT(inst->m_base.get<float>(EInstComp::Float1, 0) == 19.0f);
				PR_EXPECT(UniqueId(inst->m_base) == 29);
			}

			// Mutating a destination through lookup cannot change the copied-from object's component.
			moved.m_base.get<float>(EInstComp::Float1, 1) = 31.0f;
			PR_EXPECT(moved.m_second == 31.0f);
			PR_EXPECT(original.m_second == 23.0f);
			PR_EXPECT(copied.m_second == 23.0f);
		}
	};
}
