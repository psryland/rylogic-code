//************************************
// Lost at Sea
//  Copyright (c) Rylogic Ltd 2026
//************************************
#include "src/forward.h"
#include "src/world/water/water_system.h"

namespace las::water
{
	namespace
	{
		// Create one typed Gerstner element using the canonical shared field layout.
		WaterFieldElement MakeGerstnerWave(v2 direction, float amplitude, float wavelength, float phase_speed, float steepness)
		{
			direction = Normalise(direction);

			auto element = WaterFieldElement{};
			element.info = {WaterFieldElementTypeGerstnerWave, 0, 0, 0};
			element.position = {direction.x, direction.y, 0.0f, 0.0f};
			element.wave = {amplitude, wavelength, phase_speed, steepness};
			return element;
		}

		// Report whether a scalar is suitable for deterministic field materialisation.
		bool IsFinite(float value)
		{
			return std::isfinite(value);
		}
	}

	// Return the populated prefix of the fixed-capacity field.
	std::span<WaterFieldElement const> Snapshot::Elements() const
	{
		return {m_elements.data(), static_cast<size_t>(m_element_count)};
	}

	// Construct the field with the default Gerstner ocean and a reproducible generator sequence.
	System::System(uint32_t random_seed)
		: m_mutex()
		, m_generator_settings{
			.m_enabled = true,
			.m_spawn_interval_s = 0.8f,
			.m_amplitude_min = 0.8f,
			.m_amplitude_max = 1.8f,
			.m_wavelength_min = 4.0f,
			.m_wavelength_max = 8.0f,
			.m_packet_half_width_min = 3.0f,
			.m_packet_half_width_max = 6.0f,
			.m_propagation_speed_min = 8.0f,
			.m_propagation_speed_max = 14.0f,
			.m_lifetime_min_s = 7.0f,
			.m_lifetime_max_s = 12.0f,
			.m_attack_time_s = 0.25f,
			.m_attenuation_scale = 24.0f,
			.m_spawn_radius_min = 12.0f,
			.m_spawn_radius_max = 35.0f,
		}
		, m_base_field{}
		, m_stone_drops{}
		, m_rng(random_seed)
		, m_next_spawn_time_s(0.0)
		, m_last_update_time_s(-std::numeric_limits<double>::infinity())
		, m_snapshot{}
	{
		static_assert(BaseWaveCount + MaxStoneDropCount <= MaxWaterFieldElementCount);

		InitialiseBaseField();
		RebuildSnapshot(0.0);
	}

	// Advance event lifetime and generation, then publish a value snapshot for this simulation time.
	void System::Update(double simulation_time_s, v2 const& generator_position)
	{
		auto lock = std::lock_guard{m_mutex};

		if (!std::isfinite(simulation_time_s) || simulation_time_s < m_last_update_time_s)
			throw std::invalid_argument("Water simulation time must be finite and monotonic");
		if (!IsFinite(generator_position.x) || !IsFinite(generator_position.y))
			throw std::invalid_argument("Water generator position must be finite");

		ValidateGeneratorSettings(m_generator_settings);
		ExpireEvents(simulation_time_s);

		// Generation follows simulation time rather than render frames. Catch-up preserves a deterministic sequence when a simulation step spans multiple intervals.
		if (m_generator_settings.m_enabled)
		{
			while (m_next_spawn_time_s <= simulation_time_s)
			{
				SpawnRandomStoneDrop(m_next_spawn_time_s, generator_position);
				m_next_spawn_time_s += m_generator_settings.m_spawn_interval_s;
			}
		}
		else
		{
			m_next_spawn_time_s = simulation_time_s + m_generator_settings.m_spawn_interval_s;
		}

		m_last_update_time_s = simulation_time_s;
		RebuildSnapshot(simulation_time_s);
	}

	// Add one caller-defined event, evicting the oldest event if the bounded active set is full.
	void System::AddStoneDrop(StoneDrop const& stone_drop)
	{
		ValidateStoneDrop(stone_drop);
		auto lock = std::lock_guard{m_mutex};
		AddStoneDropInternal(stone_drop);
		RebuildSnapshot(std::max(m_last_update_time_s, 0.0));
	}

	// Insert a validated event while the caller holds the system lock.
	void System::AddStoneDropInternal(StoneDrop const& stone_drop)
	{

		// Capacity is enforced before insertion so every published snapshot remains bounded. Start time plus stable insertion order defines deterministic eviction.
		if (std::ssize(m_stone_drops) == MaxStoneDropCount)
		{
			auto oldest = std::min_element(m_stone_drops.begin(), m_stone_drops.end(), [](StoneDrop const& lhs, StoneDrop const& rhs)
			{
				return lhs.m_start_time_s < rhs.m_start_time_s;
			});
			m_stone_drops.erase(oldest);
		}

		m_stone_drops.push_back(stone_drop);
	}

	// Remove all active disturbances while retaining the base ocean.
	void System::ClearEvents()
	{
		auto lock = std::lock_guard{m_mutex};
		m_stone_drops.clear();
		RebuildSnapshot(std::max(m_last_update_time_s, 0.0));
	}

	// Return a value copy of the latest immutable field snapshot.
	Snapshot System::CurrentSnapshot() const
	{
		auto lock = std::lock_guard{m_mutex};
		return m_snapshot;
	}

	// Return the number of finite disturbances currently contributing to snapshots.
	int System::ActiveEventCount() const
	{
		auto lock = std::lock_guard{m_mutex};
		return static_cast<int>(m_stone_drops.size());
	}

	// Return a thread-safe value copy of the live generator controls.
	GeneratorSettings System::GeneratorSettingsSnapshot() const
	{
		auto lock = std::lock_guard{m_mutex};
		return m_generator_settings;
	}

	// Replace the live generator controls after validating every range.
	void System::SetGeneratorSettings(GeneratorSettings const& settings)
	{
		ValidateGeneratorSettings(settings);
		auto lock = std::lock_guard{m_mutex};
		m_generator_settings = settings;
	}

	// Build the fixed base field from the canonical ocean parameters.
	void System::InitialiseBaseField()
	{
		m_base_field = {
			MakeGerstnerWave(v2{+1.0f, +0.3f}, 1.00f, 80.0f, 11.2f, 0.35f),
			MakeGerstnerWave(v2{+0.8f, -0.6f}, 0.40f, 40.0f,  7.9f, 0.30f),
			MakeGerstnerWave(v2{-0.3f, +1.0f}, 0.20f, 20.0f,  5.6f, 0.25f),
			MakeGerstnerWave(v2{+0.5f, +0.5f}, 0.08f, 10.0f,  3.9f, 0.20f),
		};
	}

	// Validate editable generator ranges before they are used for generation.
	void System::ValidateGeneratorSettings(GeneratorSettings const& settings)
	{
		auto valid_range = [](float minimum, float maximum, bool strictly_positive)
		{
			auto lower_bound = strictly_positive ? minimum > 0.0f : minimum >= 0.0f;
			return IsFinite(minimum) && IsFinite(maximum) && lower_bound && maximum >= minimum;
		};

		if (!IsFinite(settings.m_spawn_interval_s) || settings.m_spawn_interval_s <= 0.0f)
			throw std::invalid_argument("Stone-drop spawn interval must be positive");
		if (!valid_range(settings.m_amplitude_min, settings.m_amplitude_max, true))
			throw std::invalid_argument("Stone-drop amplitude range is invalid");
		if (!valid_range(settings.m_wavelength_min, settings.m_wavelength_max, true))
			throw std::invalid_argument("Stone-drop wavelength range is invalid");
		if (!valid_range(settings.m_packet_half_width_min, settings.m_packet_half_width_max, true))
			throw std::invalid_argument("Stone-drop packet-width range is invalid");
		if (!valid_range(settings.m_propagation_speed_min, settings.m_propagation_speed_max, true))
			throw std::invalid_argument("Stone-drop propagation-speed range is invalid");
		if (!valid_range(settings.m_lifetime_min_s, settings.m_lifetime_max_s, true))
			throw std::invalid_argument("Stone-drop lifetime range is invalid");
		if (!IsFinite(settings.m_attack_time_s) || settings.m_attack_time_s < 0.0f || settings.m_attack_time_s >= settings.m_lifetime_min_s)
			throw std::invalid_argument("Stone-drop attack time must be shorter than every generated lifetime");
		if (!IsFinite(settings.m_attenuation_scale) || settings.m_attenuation_scale <= 0.0f)
			throw std::invalid_argument("Stone-drop attenuation scale must be positive");
		if (!valid_range(settings.m_spawn_radius_min, settings.m_spawn_radius_max, false))
			throw std::invalid_argument("Stone-drop spawn-radius range is invalid");
	}

	// Validate one event before accepting it into the active set.
	void System::ValidateStoneDrop(StoneDrop const& stone_drop)
	{
		if (!IsFinite(stone_drop.m_position.x) || !IsFinite(stone_drop.m_position.y) || !std::isfinite(stone_drop.m_start_time_s))
			throw std::invalid_argument("Stone-drop origin and start time must be finite");
		if (!IsFinite(stone_drop.m_amplitude) || stone_drop.m_amplitude <= 0.0f)
			throw std::invalid_argument("Stone-drop amplitude must be positive");
		if (!IsFinite(stone_drop.m_wavelength) || stone_drop.m_wavelength <= 0.0f)
			throw std::invalid_argument("Stone-drop wavelength must be positive");
		if (!IsFinite(stone_drop.m_packet_half_width) || stone_drop.m_packet_half_width <= 0.0f)
			throw std::invalid_argument("Stone-drop packet half-width must be positive");
		if (!IsFinite(stone_drop.m_propagation_speed) || stone_drop.m_propagation_speed <= 0.0f)
			throw std::invalid_argument("Stone-drop propagation speed must be positive");
		if (!IsFinite(stone_drop.m_lifetime_s) || stone_drop.m_lifetime_s <= 0.0f)
			throw std::invalid_argument("Stone-drop lifetime must be positive");
		if (!IsFinite(stone_drop.m_attack_time_s) || stone_drop.m_attack_time_s < 0.0f || stone_drop.m_attack_time_s >= stone_drop.m_lifetime_s)
			throw std::invalid_argument("Stone-drop attack time must be shorter than its lifetime");
		if (!IsFinite(stone_drop.m_attenuation_scale) || stone_drop.m_attenuation_scale <= 0.0f)
			throw std::invalid_argument("Stone-drop attenuation scale must be positive");
	}

	// Remove events whose smooth packet lifetime has ended.
	void System::ExpireEvents(double simulation_time_s)
	{
		auto expired = [simulation_time_s](StoneDrop const& stone_drop)
		{
			return stone_drop.m_start_time_s + stone_drop.m_lifetime_s <= simulation_time_s;
		};
		m_stone_drops.erase(std::remove_if(m_stone_drops.begin(), m_stone_drops.end(), expired), m_stone_drops.end());
	}

	// Generate one reproducible event around the supplied plain world-space point.
	void System::SpawnRandomStoneDrop(double start_time_s, v2 const& generator_position)
	{
		constexpr float Tau = 6.2831853071795864769f;

		auto angle = RandomRange(0.0f, Tau);
		auto radius = RandomRange(m_generator_settings.m_spawn_radius_min, m_generator_settings.m_spawn_radius_max);
		auto direction = v2{std::cos(angle), std::sin(angle)};
		auto stone_drop = StoneDrop{
			.m_position = generator_position + radius * direction,
			.m_start_time_s = start_time_s,
			.m_amplitude = RandomRange(m_generator_settings.m_amplitude_min, m_generator_settings.m_amplitude_max),
			.m_wavelength = RandomRange(m_generator_settings.m_wavelength_min, m_generator_settings.m_wavelength_max),
			.m_packet_half_width = RandomRange(m_generator_settings.m_packet_half_width_min, m_generator_settings.m_packet_half_width_max),
			.m_propagation_speed = RandomRange(m_generator_settings.m_propagation_speed_min, m_generator_settings.m_propagation_speed_max),
			.m_lifetime_s = RandomRange(m_generator_settings.m_lifetime_min_s, m_generator_settings.m_lifetime_max_s),
			.m_attack_time_s = m_generator_settings.m_attack_time_s,
			.m_attenuation_scale = m_generator_settings.m_attenuation_scale,
		};
		AddStoneDropInternal(stone_drop);
	}

	// Draw one value from a closed editable range.
	float System::RandomRange(float minimum, float maximum)
	{
		auto distribution = std::uniform_real_distribution<float>{minimum, maximum};
		return distribution(m_rng);
	}

	// Materialise age and all active parameters into the fixed-stride GPU field.
	void System::RebuildSnapshot(double simulation_time_s)
	{
		m_snapshot = {};
		m_snapshot.m_water_level = 0.0f;
		m_snapshot.m_time_s = static_cast<float>(simulation_time_s);

		// Base waves and disturbances share one typed array. Event age is calculated once in double precision and stored as a small float for both GPU consumers.
		for (auto const& element : m_base_field)
			m_snapshot.m_elements[m_snapshot.m_element_count++] = element;

		for (auto const& stone_drop : m_stone_drops)
		{
			auto age_s = static_cast<float>(simulation_time_s - stone_drop.m_start_time_s);
			auto& element = m_snapshot.m_elements[m_snapshot.m_element_count++];
			element.info = {WaterFieldElementTypeStoneDrop, 0, 0, 0};
			element.position = {stone_drop.m_position.x, stone_drop.m_position.y, 0.0f, 0.0f};
			element.wave = {stone_drop.m_amplitude, stone_drop.m_wavelength, stone_drop.m_packet_half_width, stone_drop.m_propagation_speed};
			element.timing = {age_s, stone_drop.m_lifetime_s, stone_drop.m_attack_time_s, stone_drop.m_attenuation_scale};
		}
	}
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
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
		WaterFieldElement TestStoneDropElement(float age_s)
		{
			auto element = WaterFieldElement{};
			element.info = {WaterFieldElementTypeStoneDrop, 0, 0, 0};
			element.position = {0.0f, 0.0f, 0.0f, 0.0f};
			element.wave = {1.0f, 4.0f, 2.0f, 3.0f};
			element.timing = {age_s, 4.0f, 0.1f, 10.0f};
			return element;
		}
	}

	PRUnitTest(WaterSystemDeterministicGeneration)
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

	PRUnitTest(WaterSystemExpiryAndSnapshotAge)
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

	PRUnitTest(WaterSystemCapacityAndOldestEviction)
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

	PRUnitTest(WaterFieldStoneDropCompactSupport)
	{
		auto element = TestStoneDropElement(1.0f);
		auto outside_packet = EvaluateStoneDrop(element, float2{6.0f, 0.0f});
		auto expired_packet = EvaluateStoneDrop(TestStoneDropElement(4.0f), float2{3.0f, 0.0f});
		PR_EXPECT(outside_packet.height == 0.0f);
		PR_EXPECT(outside_packet.radial_height_gradient == 0.0f);
		PR_EXPECT(expired_packet.height == 0.0f);
		PR_EXPECT(expired_packet.vertical_velocity == 0.0f);
	}

	PRUnitTest(WaterFieldStoneDropRadialSymmetry)
	{
		auto element = TestStoneDropElement(0.75f);
		auto positive_x = EvaluateStoneDrop(element, float2{2.5f, 0.0f});
		auto negative_x = EvaluateStoneDrop(element, float2{-2.5f, 0.0f});
		PR_EXPECT(FEql(positive_x.height, negative_x.height));
		PR_EXPECT(FEql(positive_x.vertical_velocity, negative_x.vertical_velocity));
		PR_EXPECT(FEql(positive_x.radial_height_gradient, negative_x.radial_height_gradient));
		PR_EXPECT(FEql(positive_x.radial_direction, -negative_x.radial_direction));
	}

	PRUnitTest(WaterFieldOverlappingSuperposition)
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
}
#endif
