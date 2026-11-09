//*********************************************
// Physics Terrain Water
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#include "pr/physics/terrain/water/water_field.h"
#include "pr/physics/terrain/water/water_field.hlsli"
#include "pr/physics/terrain/water/water_depth.hlsli"
#include "pr/physics/terrain/water/wind_sea.hlsli"

namespace pr::physics::terrain::water
{
	namespace
	{
		// Require one scalar water-field value to be finite.
		void RequireFinite(double value, char const* name)
		{
			if (!std::isfinite(value))
				throw std::invalid_argument(std::format("Water field '{}' must be finite", name));
		}

		// Require one wave or packet scalar to be finite and strictly positive.
		void RequirePositive(float value, char const* name)
		{
			RequireFinite(value, name);
			if (!(value > 0.0f))
				throw std::invalid_argument(std::format("Water field '{}' must be greater than zero", name));
		}

		// Validate one element so CPU and GPU evaluation never meet a division by zero or an unknown type.
		void Validate(WaterFieldElement const& element)
		{
			// Every payload component must be finite, whatever the element type.
			for (auto i = 0; i != 4; ++i)
			{
				RequireFinite(element.position[i], "position");
				RequireFinite(element.wave[i], "wave");
				RequireFinite(element.timing[i], "timing");
			}

			switch (element.info.x)
			{
				case shared::WaterFieldElementNone:
				{
					return;
				}
				case shared::WaterFieldElementSineWave:
				case shared::WaterFieldElementGerstnerWave:
				{
					// Waves need a unit direction so amplitude, gradient, and phase keep their documented meaning.
					RequirePositive(element.wave.y, "wavelength");
					auto const length = Length(v2{element.position.x, element.position.y});
					if (std::abs(length - 1.0f) > 1e-4f)
						throw std::invalid_argument("Water field wave direction must be unit length");

					// The crest profile is only defined for sharpness below one; the limit keeps its slopes bounded.
					if (!(element.timing.x >= 0.0f && element.timing.x <= shared::WaterFieldMaxCrestSharpness))
						throw std::invalid_argument(std::format("Water field crest sharpness must be in [0, {}]", shared::WaterFieldMaxCrestSharpness));

					// A wind-sea band tag is a band index plus one.
					if (element.info.y < 0)
						throw std::invalid_argument("Water field wind-sea band must not be negative");

					return;
				}
				case shared::WaterFieldElementRadialPacket:
				{
					// Packets with non-positive shape values are inactive in the evaluator, so only the wavelength must be usable.
					// Only waves can belong to a wind sea.
					RequirePositive(element.wave.y, "wavelength");
					if (element.info.y != 0)
						throw std::invalid_argument("Water field radial packets cannot belong to a wind sea");

					return;
				}
				default:
				{
					throw std::invalid_argument(std::format("Water field element type {} is not supported", element.info.x));
				}
			}
		}

		// Require every band-tagged element to name a band of 'wind_sea'. Tags are ignored without a wind sea.
		void ValidateBands(std::span<WaterFieldElement const> elements, WindSea const* wind_sea)
		{
			if (wind_sea == nullptr)
				return;

			for (auto const& element : elements)
			{
				if (shared::WaterFieldWindSeaBand(element) >= wind_sea->Grid().band_count)
					throw std::invalid_argument(std::format("Water field element band {} is not in the wind sea", shared::WaterFieldWindSeaBand(element)));
			}
		}

		// Return a unit wave direction, rejecting a zero-length input.
		v2 UnitDirection(v2 direction)
		{
			auto const length = Length(direction);
			if (!(length > tiny<float>) || !std::isfinite(length))
				throw std::invalid_argument("Water field wave direction must be non-zero and finite");

			return direction / length;
		}
	}

	// Return a sine wave element with height A*sin(k*dot(d, xy) + omega*t + phase).
	WaterFieldElement SineWave(v2 direction, float amplitude, float wavelength, float angular_frequency, float phase)
	{
		auto const d = UnitDirection(direction);
		return WaterFieldElement{
			.info = {shared::WaterFieldElementSineWave, 0, 0, 0},
			.position = {d.x, d.y, phase, 0},
			.wave = {amplitude, wavelength, angular_frequency, 0},
			.timing = {0, 0, 0, 0},
		};
	}

	// Return a Gerstner wave element with height A*P(k*dot(d, xy) - k*c*t + phase), where P is the crest profile for 'sharpness'.
	WaterFieldElement GerstnerWave(v2 direction, float amplitude, float wavelength, float phase_speed, float steepness, float sharpness, float phase)
	{
		auto const d = UnitDirection(direction);
		return WaterFieldElement{
			.info = {shared::WaterFieldElementGerstnerWave, 0, 0, 0},
			.position = {d.x, d.y, phase, 0},
			.wave = {amplitude, wavelength, phase_speed, steepness},
			.timing = {sharpness, 0, 0, 0},
		};
	}

	// Return a finite radial packet travelling outward from 'source'.
	WaterFieldElement RadialPacket(v2 source, float amplitude, float wavelength, float half_width, float propagation_speed, float age, float lifetime, float attack_time, float attenuation_scale)
	{
		return WaterFieldElement{
			.info = {shared::WaterFieldElementRadialPacket, 0, 0, 0},
			.position = {source.x, source.y, 0, 0},
			.wave = {amplitude, wavelength, half_width, propagation_speed},
			.timing = {age, lifetime, attack_time, attenuation_scale},
		};
	}

	// Construct a validated water field.
	WaterField::WaterField(double level, std::span<WaterFieldElement const> elements)
		: m_level()
		, m_elements()
		, m_amplitude_bound()
		, m_swash_depth()
		, m_bathymetry()
		, m_wind_sea()
		, m_breaking_ratio(DefaultBreakingRatio)
		, m_shore_sharpness(DefaultShoreSharpness)
		, m_repeat_period()
	{
		Level(level);
		Elements(elements);
	}
	WaterField::WaterField()
		: m_level()
		, m_elements()
		, m_amplitude_bound()
		, m_swash_depth()
		, m_bathymetry()
		, m_wind_sea()
		, m_breaking_ratio(DefaultBreakingRatio)
		, m_shore_sharpness(DefaultShoreSharpness)
		, m_repeat_period()
	{
		// Zero level with no elements is already a valid flat field.
	}

	// The still-water level in metres.
	double WaterField::Level() const noexcept
	{
		return m_level;
	}
	void WaterField::Level(double level)
	{
		RequireFinite(level, "level");
		m_level = level;
	}

	// The active elements, in upload order.
	std::span<WaterFieldElement const> WaterField::Elements() const noexcept
	{
		return m_elements;
	}
	void WaterField::Elements(std::span<WaterFieldElement const> elements)
	{
		// Validate everything before replacing the current elements so a failed update leaves the field unchanged.
		if (std::ssize(elements) > MaxElementCount)
			throw std::invalid_argument(std::format("Water field supports at most {} elements", MaxElementCount));

		for (auto const& element : elements)
			Validate(element);

		ValidateBands(elements, m_wind_sea.get());
		m_elements.assign(elements.begin(), elements.end());
		UpdateBounds();
	}

	// The local winds that set the amplitude and sharpness of band-tagged elements.
	std::shared_ptr<WindSea const> const& WaterField::WindSea() const noexcept
	{
		return m_wind_sea;
	}
	void WaterField::WindSea(std::shared_ptr<water::WindSea const> wind_sea)
	{
		// Validate against the current elements before replacing the wind sea so a failed update leaves the field unchanged.
		ValidateBands(m_elements, wind_sea.get());
		m_wind_sea = std::move(wind_sea);
		UpdateBounds();
	}

	// Return the elements as they act at 'xy', localised for the wind sea and corrected for depth.
	std::span<WaterFieldElement const> WaterField::LocalElements(v2d xy, std::span<WaterFieldElement, MaxElementCount> buffer) const
	{
		// Use the same localisation and correction sequence as the samplers, so evaluating the result matches Height, Sample, and Velocity.
		auto const xy_f = v2{static_cast<float>(xy.x), static_cast<float>(xy.y)};
		auto const depth = WaveDepth(xy);
		auto const elements = Localise(xy_f, buffer, true);
		auto const correction = Correction(elements, depth);

		// Localise returns the stored elements without a wind sea, so the corrected copies always go into 'buffer'.
		auto const count = elements.size();
		if (elements.data() != buffer.data())
			std::copy(elements.begin(), elements.end(), buffer.begin());
		for (size_t i = 0; i != count; ++i)
			buffer[i] = shared::WaterFieldDepthCorrected(buffer[i], depth, correction.m_breaking_scale, correction.m_min_sharpness);

		return std::span<WaterFieldElement const>{buffer.data(), count};
	}

	// The terrain heights used for depth correction.
	std::shared_ptr<Bathymetry const> const& WaterField::TerrainHeights() const noexcept
	{
		return m_bathymetry;
	}
	void WaterField::TerrainHeights(std::shared_ptr<Bathymetry const> bathymetry)
	{
		m_bathymetry = std::move(bathymetry);
	}

	// Ratio of the largest wave height to the water depth.
	float WaterField::BreakingRatio() const noexcept
	{
		return m_breaking_ratio;
	}
	void WaterField::BreakingRatio(float ratio)
	{
		RequirePositive(ratio, "breaking ratio");
		m_breaking_ratio = ratio;
	}

	// The crest sharpness that waves reach at the breaking height.
	float WaterField::ShoreSharpness() const noexcept
	{
		return m_shore_sharpness;
	}
	void WaterField::ShoreSharpness(float sharpness)
	{
		RequireFinite(sharpness, "shore sharpness");
		if (!(sharpness >= 0.0f && sharpness <= shared::WaterFieldMaxCrestSharpness))
			throw std::invalid_argument(std::format("Water field shore sharpness must be in [0, {}]", shared::WaterFieldMaxCrestSharpness));

		m_shore_sharpness = sharpness;
	}

	// The period after which every element repeats exactly.
	double WaterField::RepeatPeriod() const noexcept
	{
		return m_repeat_period;
	}
	void WaterField::RepeatPeriod(double period)
	{
		RequireFinite(period, "repeat period");
		if (period < 0.0)
			throw std::invalid_argument("Water field 'repeat period' must not be negative");

		m_repeat_period = period;
	}

	// Wrap a simulation time into [0, RepeatPeriod).
	float WaterField::LocalTime(double time_s) const noexcept
	{
		// Wrapping in double keeps full precision for long-running clocks before the float conversion used by the evaluators.
		if (!(m_repeat_period > 0.0))
			return static_cast<float>(time_s);

		auto local = std::fmod(time_s, m_repeat_period);
		if (local < 0.0)
			local += m_repeat_period;

		return static_cast<float>(local);
	}

	// The water depth used for wave corrections at a world-space position.
	float WaterField::WaveDepth(v2d xy) const
	{
		return m_bathymetry != nullptr ? static_cast<float>(m_level + m_swash_depth - m_bathymetry->HeightAt(xy)) : shared::WaterFieldDeepWater;
	}

	// True when no element can change the height.
	bool WaterField::IsFlat() const noexcept
	{
		return m_amplitude_bound == 0.0;
	}

	// Conservative bounds on the surface height over all positions and times.
	double WaterField::MaxHeight() const noexcept
	{
		return m_level + AmplitudeBound(m_bathymetry != nullptr ? m_level + m_swash_depth - m_bathymetry->MinHeight() : shared::WaterFieldDeepWater);
	}
	double WaterField::MinHeight() const noexcept
	{
		return m_level - AmplitudeBound(m_bathymetry != nullptr ? m_level + m_swash_depth - m_bathymetry->MinHeight() : shared::WaterFieldDeepWater);
	}

	// Conservative bounds on the surface height over a world-space rectangle.
	double WaterField::MaxHeight(v2d lo, v2d hi) const
	{
		return m_level + AmplitudeBound(m_bathymetry != nullptr ? m_level + m_swash_depth - m_bathymetry->MinHeight(lo, hi) : shared::WaterFieldDeepWater);
	}
	double WaterField::MinHeight(v2d lo, v2d hi) const
	{
		return m_level - AmplitudeBound(m_bathymetry != nullptr ? m_level + m_swash_depth - m_bathymetry->MinHeight(lo, hi) : shared::WaterFieldDeepWater);
	}

	// Sample the surface height at a world-space XY position and simulation time.
	double WaterField::Height(v2d xy, double time_s) const
	{
		// Element contributions are evaluated in float, matching the GPU, and added to the double still-water level.
		auto const xy_f = v2{static_cast<float>(xy.x), static_cast<float>(xy.y)};
		auto const t = LocalTime(time_s);
		auto const depth = WaveDepth(xy);
		// Waves shoal to nothing where there is no water, even allowing for swash, so dry ground has the still level without evaluating any element.
		if (!(depth > 0.0f))
			return m_level;

		// Sum the depth-corrected elements
		auto buffer = std::array<WaterFieldElement, MaxElementCount>{};
		auto const elements = Localise(xy_f, buffer, true);
		auto const correction = Correction(elements, depth);
		auto height = 0.0;
		for (auto const& element : elements)
			height += shared::WaterFieldElementHeight(shared::WaterFieldDepthCorrected(element, depth, correction.m_breaking_scale, correction.m_min_sharpness), xy_f, t);

		return m_level + height;
	}

	// Sample the surface height and slope at a world-space XY position and simulation time.
	WaterSample WaterField::Sample(v2d xy, double time_s) const
	{
		// Accumulate the float element contributions onto the double still-water level.
		auto const xy_f = v2{static_cast<float>(xy.x), static_cast<float>(xy.y)};
		auto const t = LocalTime(time_s);
		auto const depth = WaveDepth(xy);
		auto sample = WaterSample{.m_height = m_level, .m_gradient_xy = v2d::Zero()};
		// Waves shoal to nothing where there is no water, even allowing for swash, so dry ground has the still level and no slope without evaluating any element.
		if (!(depth > 0.0f))
			return sample;

		// Sum the depth-corrected elements
		auto buffer = std::array<WaterFieldElement, MaxElementCount>{};
		auto const elements = Localise(xy_f, buffer, true);
		auto const correction = Correction(elements, depth);
		for (auto const& element : elements)
		{
			// Each contribution is (height, dh/dx, dh/dy).
			auto const contribution = shared::WaterFieldElementHeightAndGradient(shared::WaterFieldDepthCorrected(element, depth, correction.m_breaking_scale, correction.m_min_sharpness), xy_f, t);
			sample.m_height += contribution.x;
			sample.m_gradient_xy += v2d{contribution.y, contribution.z};
		}
		return sample;
	}

	// Sample the dimensionless dynamic pressure gradient used by buoyancy forces.
	v4 WaterField::PressureGradient(v2 xy, double time_s, float gravity) const
	{
		// Without gravity there is no hydrostatic pressure to form a gradient.
		if (!(gravity > tiny<float>))
			return v4::Zero();

		auto const t = LocalTime(time_s);
		auto const depth = WaveDepth(v2d{xy.x, xy.y});
		// Waves shoal to nothing where there is no water, even allowing for swash, so dry ground has no flow.
		if (!(depth > 0.0f))
			return v4::Zero();

		// Accumulate the depth-corrected elements' flow
		auto buffer = std::array<WaterFieldElement, MaxElementCount>{};
		auto const elements = Localise(xy, buffer, true);
		auto const correction = Correction(elements, depth);
		auto flow = shared::WaterFieldFlowSampleZero();
		auto const pos = v3{xy.x, xy.y, static_cast<float>(m_level)};
		for (auto const& element : elements)
		{
			// Only the pressure gradient part of the flow state is needed.
			shared::WaterFieldAccumulateFlow(shared::WaterFieldDepthCorrected(element, depth, correction.m_breaking_scale, correction.m_min_sharpness), pos, t, pos.z, gravity, flow);
		}
		return v4{flow.height_pressure_gradient.y, flow.height_pressure_gradient.z, flow.height_pressure_gradient.w, 0.0f};
	}

	// Sample the world-space water particle velocity at a world-space position.
	v4 WaterField::Velocity(v4 pos_ws, double time_s) const
	{
		// Orbital flow decays with depth below the still-water level.
		auto const t = LocalTime(time_s);
		auto const depth = WaveDepth(v2d{pos_ws.x, pos_ws.y});
		// Waves shoal to nothing where there is no water, even allowing for swash, so dry ground has no flow.
		if (!(depth > 0.0f))
			return v4::Zero();

		// Accumulate the depth-corrected elements' flow
		auto buffer = std::array<WaterFieldElement, MaxElementCount>{};
		auto const elements = Localise(v2{pos_ws.x, pos_ws.y}, buffer, true);
		auto const correction = Correction(elements, depth);
		auto flow = shared::WaterFieldFlowSampleZero();
		auto const level = static_cast<float>(m_level);
		for (auto const& element : elements)
		{
			// Only the velocity part of the flow state is needed. The pressure gradient's gravity is irrelevant here, so any positive value serves.
			shared::WaterFieldAccumulateFlow(shared::WaterFieldDepthCorrected(element, depth, correction.m_breaking_scale, correction.m_min_sharpness), pos_ws.xyz, t, level, 1.0f, flow);
		}
		return v4{flow.velocity.x, flow.velocity.y, flow.velocity.z, 0};
	}

	// Recompute the amplitude bound and swash depth.
	void WaterField::UpdateBounds()
	{
		// Elements with their own amplitude add their own bounds. Wind-sea elements are counted per band instead.
		auto amplitude_bound = 0.0;
		auto amplitude_squares = 0.0f;
		auto band_counts = std::array<int, MaxElementCount>{};
		for (auto const& element : m_elements)
		{
			auto const band = m_wind_sea != nullptr ? shared::WaterFieldWindSeaBand(element) : -1;
			if (band >= 0)
			{
				++band_counts[band];
				continue;
			}
			auto const amplitude = shared::WaterFieldElementAmplitudeBound(element);
			amplitude_bound += amplitude;
			amplitude_squares += amplitude * amplitude;
		}

		// The crests of a band's 'n' elements sum to at most 'n' times the largest crest, and, by the Cauchy-Schwarz inequality, to at most
		// sqrt(n) times their root sum of squares. Either bound holds, so the smaller is used.
		if (m_wind_sea != nullptr)
		{
			for (int b = 0; b != m_wind_sea->Grid().band_count; ++b)
			{
				auto const n = band_counts[b];
				if (n == 0)
					continue;

				amplitude_bound += std::min(n * m_wind_sea->PeakBound(b), std::sqrt(static_cast<float>(n)) * m_wind_sea->SpreadBound(b));
				amplitude_squares += m_wind_sea->SquareBound(b);
			}
		}

		m_amplitude_bound = amplitude_bound;
		m_swash_depth = shared::WaterFieldSwashDepth(amplitude_squares);
	}

	// Return the elements with their local amplitude and sharpness at 'xy'.
	std::span<WaterFieldElement const> WaterField::Localise(v2 xy, std::span<WaterFieldElement, MaxElementCount> buffer, bool compact) const
	{
		// Without a wind sea every element keeps its own values.
		if (m_wind_sea == nullptr)
			return m_elements;

		// Find each band's local state once, then apply it to the band's elements. Elements that are calm here add nothing to any sample,
		// so 'compact' drops them to save their evaluation.
		auto bands = std::array<WindSeaBand, MaxElementCount>{};
		m_wind_sea->Bands(xy, std::span{bands}.first(m_wind_sea->Grid().band_count));
		auto count = size_t{};
		for (auto const& element : m_elements)
		{
			auto const band = shared::WaterFieldWindSeaBand(element);
			auto const local = band >= 0 ? shared::WindSeaElement(element, bands[band]) : element;
			if (compact && local.wave.x == 0.0f)
				continue;

			buffer[count++] = local;
		}
		return std::span<WaterFieldElement const>{buffer.data(), count};
	}

	// Return the amplitude bound near a point whose deepest water, including the swash allowance, is 'max_depth'.
	double WaterField::AmplitudeBound(double max_depth) const noexcept
	{
		// Without terrain heights there is no depth correction, so the elements' own amplitudes are the bound.
		if (m_bathymetry == nullptr)
			return m_amplitude_bound;

		// Shoaling can at most double an amplitude, and shore sharpening can raise each crest by at most a factor of 1 + ShoreSharpness.
		// Breaking limits the total to half the breaking height of the deepest water.
		if (!(max_depth > 0.0))
			return 0.0;

		return std::min(shared::WaterFieldMaxShoaling * (1.0 + m_shore_sharpness) * m_amplitude_bound, 0.5 * m_breaking_ratio * max_depth);
	}

	// Return the depth correction of 'elements' for 'depth'.
	WaterField::DepthCorrection WaterField::Correction(std::span<WaterFieldElement const> elements, float depth) const
	{
		// Deep water everywhere needs no correction.
		if (m_bathymetry == nullptr)
			return DepthCorrection{.m_breaking_scale = 1.0f, .m_min_sharpness = 0.0f};

		auto total = 0.0f;
		for (auto const& element : elements)
			total += shared::WaterFieldElementAmplitudeBound(element) * shared::WaterFieldShoaling(element, depth);

		auto const min_sharpness = shared::WaterFieldShoreSharpness(total, depth, m_breaking_ratio, m_shore_sharpness);
		return DepthCorrection{
			.m_breaking_scale = shared::WaterFieldBreakingScale(total, min_sharpness, depth, m_breaking_ratio),
			.m_min_sharpness = min_sharpness,
		};
	}
}
