//************************************
// Lost at Sea
//  Copyright (c) Rylogic Ltd 2026
//************************************
#if PR_UNITTESTS
#include "src/world/water/water_system.h"
using namespace las::water;
#include "src/world/water/shaders/water_field.hlsli"

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

		// Materialise a stone-drop element directly for tests of the shared C++/HLSL evaluator.
		las::water::WaterFieldElement TestStoneDropElement(float age_s)
		{
			auto element = WaterFieldElement{};
			element.info = {WaterFieldElementTypeStoneDrop, 0, 0, 0};
			element.position = {0.0f, 0.0f, 0.0f, 0.0f};
			element.wave = {1.0f, 4.0f, 2.0f, 3.0f};
			element.timing = {age_s, 4.0f, 0.1f, 10.0f};
			return element;
		}
	}

	// Verify deterministic generation, bounded lifecycle, and the shared CPU/HLSL evaluator.
	PRUnitTestClass(LostAtSeaWaterTests)
	{
		PRUnitTestMethod(DeterministicGeneration)
		{
			auto lhs = System{12345};
			auto rhs = System{12345};

			lhs.Update(0.0, v2{10.0f, -4.0f});
			rhs.Update(0.0, v2{10.0f, -4.0f});
			auto lhs_snapshot = lhs.CurrentSnapshot();
			auto rhs_snapshot = rhs.CurrentSnapshot();
			PR_EXPECT(lhs.ActiveEventCount() == rhs.ActiveEventCount());
			PR_EXPECT(std::memcmp(lhs_snapshot.m_elements.data(), rhs_snapshot.m_elements.data(), sizeof(WaterFieldElement) * lhs_snapshot.m_element_count) == 0);
		}
		PRUnitTestMethod(ExpiryAndSnapshotAge)
		{
			auto system = System{};
			auto settings = system.GeneratorSettingsSnapshot();
			settings.m_enabled = false;
			system.SetGeneratorSettings(settings);
			system.AddStoneDrop(TestStoneDrop(3.0f, 5.0, 1.0f));

			system.Update(5.5, v2::Zero());
			auto snapshot = system.CurrentSnapshot();
			PR_EXPECT(system.ActiveEventCount() == 1);
			PR_EXPECT(Abs(snapshot.m_elements[System::BaseWaveCount].timing.x - 0.5f) < 1.0e-6f);

			system.Update(6.0, v2::Zero());
			PR_EXPECT(system.ActiveEventCount() == 0);
		}
		PRUnitTestMethod(CapacityAndOldestEviction)
		{
			auto system = System{};
			auto settings = system.GeneratorSettingsSnapshot();
			settings.m_enabled = false;
			system.SetGeneratorSettings(settings);

			for (int i = 0; i != System::MaxStoneDropCount + 1; ++i)
				system.AddStoneDrop(TestStoneDrop(static_cast<float>(i), static_cast<double>(i), 100.0f));

			system.Update(System::MaxStoneDropCount, v2::Zero());
			auto snapshot = system.CurrentSnapshot();
			PR_EXPECT(system.ActiveEventCount() == System::MaxStoneDropCount);
			PR_EXPECT(snapshot.m_element_count == System::BaseWaveCount + System::MaxStoneDropCount);
			PR_EXPECT(Abs(snapshot.m_elements[System::BaseWaveCount].position.x - 1.0f) < 1.0e-6f);
		}
		PRUnitTestMethod(StoneDropCompactSupport)
		{
			auto element = TestStoneDropElement(1.0f);
			auto outside_packet = EvaluateStoneDrop(element, float2{6.0f, 0.0f});
			auto expired_packet = EvaluateStoneDrop(TestStoneDropElement(4.0f), float2{3.0f, 0.0f});
			PR_EXPECT(outside_packet.height == 0.0f);
			PR_EXPECT(outside_packet.radial_height_gradient == 0.0f);
			PR_EXPECT(expired_packet.height == 0.0f);
			PR_EXPECT(expired_packet.vertical_velocity == 0.0f);
		}
		PRUnitTestMethod(StoneDropRadialSymmetry)
		{
			auto element = TestStoneDropElement(0.75f);
			auto positive_x = EvaluateStoneDrop(element, float2{2.5f, 0.0f});
			auto negative_x = EvaluateStoneDrop(element, float2{-2.5f, 0.0f});
			PR_EXPECT(FEql(positive_x.height, negative_x.height));
			PR_EXPECT(FEql(positive_x.vertical_velocity, negative_x.vertical_velocity));
			PR_EXPECT(FEql(positive_x.radial_height_gradient, negative_x.radial_height_gradient));
			PR_EXPECT(FEql(positive_x.radial_direction, -negative_x.radial_direction));
		}
		PRUnitTestMethod(OverlappingSuperposition)
		{
			auto first = TestStoneDropElement(0.75f);
			auto second = first;
			second.position = float4{0.5f, 0.0f, 0.0f, 0.0f};
			second.wave.x = 0.6f;

			auto world_xy = float2{2.5f, 0.0f};
			auto expected_height =
				EvaluateWaterFieldHeightElement(first, world_xy, 0.75f) +
				EvaluateWaterFieldHeightElement(second, world_xy, 0.75f);
			auto sample = WaterFieldSampleZero();
			AccumulateWaterFieldElement(first, world_xy, 0.75f, sample);
			AccumulateWaterFieldElement(second, world_xy, 0.75f, sample);
			PR_EXPECT(FEql(sample.displacement_foam.z, expected_height));
			PR_EXPECT(std::isfinite(sample.displacement_foam.x) && std::isfinite(sample.displacement_foam.y) &&
				std::isfinite(sample.displacement_foam.z) && std::isfinite(sample.displacement_foam.w));
			PR_EXPECT(std::isfinite(sample.normal_delta.x) && std::isfinite(sample.normal_delta.y) &&
				std::isfinite(sample.normal_delta.z) && std::isfinite(sample.normal_delta.w));
		}
	};
}
#endif
