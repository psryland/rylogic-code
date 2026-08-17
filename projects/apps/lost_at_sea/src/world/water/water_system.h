//************************************
// Lost at Sea
//  Copyright (c) Rylogic Ltd 2026
//************************************
#pragma once
#include "src/forward.h"
#include "src/world/water/shaders/water_field_types.hlsli"

namespace las::water
{
	// A point-source circular wave packet in world space.
	struct StoneDrop
	{
		v2 m_position;
		double m_start_time_s;
		float m_amplitude;
		float m_wavelength;
		float m_packet_half_width;
		float m_propagation_speed;
		float m_lifetime_s;
		float m_attack_time_s;
		float m_attenuation_scale;
	};

	// Live-editable parameters for the deterministic proof-of-concept generator.
	struct GeneratorSettings
	{
		bool m_enabled;
		float m_spawn_interval_s;
		float m_amplitude_min;
		float m_amplitude_max;
		float m_wavelength_min;
		float m_wavelength_max;
		float m_packet_half_width_min;
		float m_packet_half_width_max;
		float m_propagation_speed_min;
		float m_propagation_speed_max;
		float m_lifetime_min_s;
		float m_lifetime_max_s;
		float m_attack_time_s;
		float m_attenuation_scale;
		float m_spawn_radius_min;
		float m_spawn_radius_max;
	};

	// An immutable, fixed-capacity field value materialised for one simulation time.
	struct Snapshot
	{
		std::array<WaterFieldElement, MaxWaterFieldElementCount> m_elements;
		int32_t m_element_count;
		float m_water_level;
		float m_time_s;

		// Return the populated prefix of the fixed-capacity field.
		std::span<WaterFieldElement const> Elements() const;
	};

	// Owns the base ocean and finite water events independently of rendering and physics consumers.
	struct System
	{
		static constexpr int BaseWaveCount = 4;
		static constexpr int MaxStoneDropCount = 32;

	private:
		mutable std::mutex m_mutex;
		GeneratorSettings m_generator_settings;
		std::array<WaterFieldElement, BaseWaveCount> m_base_field;
		std::vector<StoneDrop> m_stone_drops;
		std::mt19937 m_rng;
		double m_next_spawn_time_s;
		double m_last_update_time_s;
		Snapshot m_snapshot;

	public:
		// Construct the field with the default Gerstner ocean and a reproducible generator sequence.
		explicit System(uint32_t random_seed = 0x4C615357u);

		// Advance event lifetime and generation, then publish a value snapshot for this simulation time.
		void Update(double simulation_time_s, v2 const& generator_position);

		// Add one caller-defined event, evicting the oldest event if the bounded active set is full.
		void AddStoneDrop(StoneDrop const& stone_drop);

		// Remove all active disturbances while retaining the base ocean.
		void ClearEvents();

		// Return a value copy of the latest immutable field snapshot.
		Snapshot CurrentSnapshot() const;

		// Return the number of finite disturbances currently contributing to snapshots.
		int ActiveEventCount() const;

		// Return a thread-safe value copy of the live generator controls.
		GeneratorSettings GeneratorSettingsSnapshot() const;

		// Replace the live generator controls after validating every range.
		void SetGeneratorSettings(GeneratorSettings const& settings);

	private:
		// Build the fixed base field from the canonical ocean parameters.
		void InitialiseBaseField();

		// Validate editable generator ranges before they are accepted for generation.
		static void ValidateGeneratorSettings(GeneratorSettings const& settings);

		// Validate one event before accepting it into the active set.
		static void ValidateStoneDrop(StoneDrop const& stone_drop);

		// Insert a validated event while the caller holds the system lock.
		void AddStoneDropInternal(StoneDrop const& stone_drop);

		// Remove events whose smooth packet lifetime has ended.
		void ExpireEvents(double simulation_time_s);

		// Generate one reproducible event around the supplied plain world-space point.
		void SpawnRandomStoneDrop(double start_time_s, v2 const& generator_position);

		// Draw one value from a closed editable range.
		float RandomRange(float minimum, float maximum);

		// Materialise age and all active parameters into the fixed-stride GPU field.
		void RebuildSnapshot(double simulation_time_s);
	};
}
