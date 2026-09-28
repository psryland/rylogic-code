//************************************
// Lost at Sea
//  Copyright (c) Rylogic Ltd 2026
//************************************
#if PR_UNITTESTS
#include "src/world/water/water_system.h"
using namespace las::water;

namespace pr::unittests
{
	using namespace las::water;
	using namespace pr::hlsl;

	namespace
	{
		// Create a compact caller-defined event for lifecycle tests.
		StoneDrop TestStoneDrop(float x, double start_time_s, float lifetime_s)
		{
			return {
				.m_position = {x, 0.0f},
				.m_start_time_s = start_time_s,
				.m_amplitude = 1.0f,
				.m_wavelength = 4.0f,
				.m_packet_half_width = 2.0f,
				.m_propagation_speed = 3.0f,
				.m_lifetime_s = lifetime_s,
				.m_attack_time_s = 0.1f,
				.m_attenuation_scale = 10.0f,
			};
		}

		// Compare the active element payloads in two snapshots.
		bool SameElements(Snapshot const& lhs, Snapshot const& rhs)
		{
			// Deterministic generation is defined by the exact shared payload uploaded to consumers.
			auto lhs_elements = lhs.Elements();
			auto rhs_elements = rhs.Elements();
			return lhs_elements.size() == rhs_elements.size() &&
				std::memcmp(lhs_elements.data(), rhs_elements.data(), sizeof(WaterFieldElement) * lhs_elements.size()) == 0;
		}
	}

	// Verify deterministic generation and bounded lifecycle around the shared water field.
	PRUnitTestClass(LostAtSeaWaterTests)
	{
		PRUnitTestMethod(DeterministicGeneration, Quick)
		{
			auto lhs = System{12345};
			auto rhs = System{12345};

			lhs.Update(0.0, v2{10.0f, -4.0f});
			rhs.Update(0.0, v2{10.0f, -4.0f});
			auto lhs_snapshot = lhs.CurrentSnapshot();
			auto rhs_snapshot = rhs.CurrentSnapshot();
			PR_EXPECT(lhs.ActiveEventCount() == rhs.ActiveEventCount());
			PR_EXPECT(SameElements(lhs_snapshot, rhs_snapshot));
		}
		PRUnitTestMethod(ExpiryAndSnapshotAge, Quick)
		{
			auto system = System{};
			auto settings = system.GeneratorSettingsSnapshot();
			settings.m_enabled = false;
			system.SetGeneratorSettings(settings);
			system.AddStoneDrop(TestStoneDrop(3.0f, 5.0, 1.0f));

			system.Update(5.5, v2::Zero());
			auto snapshot = system.CurrentSnapshot();
			auto elements = snapshot.Elements();
			PR_EXPECT(system.ActiveEventCount() == 1);
			PR_EXPECT(Abs(elements[System::BaseWaveCount].timing.x - 0.5f) < 1.0e-6f);

			system.Update(6.0, v2::Zero());
			PR_EXPECT(system.ActiveEventCount() == 0);
		}
		PRUnitTestMethod(CapacityAndOldestEviction, Quick)
		{
			auto system = System{};
			auto settings = system.GeneratorSettingsSnapshot();
			settings.m_enabled = false;
			system.SetGeneratorSettings(settings);

			for (int i = 0; i != System::MaxStoneDropCount + 1; ++i)
				system.AddStoneDrop(TestStoneDrop(static_cast<float>(i), static_cast<double>(i), 100.0f));

			system.Update(System::MaxStoneDropCount, v2::Zero());
			auto snapshot = system.CurrentSnapshot();
			auto elements = snapshot.Elements();
			PR_EXPECT(system.ActiveEventCount() == System::MaxStoneDropCount);
			PR_EXPECT(isize(elements) == System::BaseWaveCount + System::MaxStoneDropCount);
			PR_EXPECT(Abs(elements[System::BaseWaveCount].position.x - 1.0f) < 1.0e-6f);
		}
	};
}
#endif
