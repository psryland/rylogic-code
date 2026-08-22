//***************************************************************************************************
// Audio
//  Copyright (c) Rylogic Ltd 2017
//***************************************************************************************************
// A basic tone generator
#pragma once
#include "pr/audio/forward.h"
#include "pr/audio/synth/note.h"

namespace pr::audio
{
	// Tone generator
	struct Synth
	{
		// Return the number of samples needed for the given sequence of notes at the given sample rate
		static int SampleCount(std::span<Note const> notes, ESampleRate sample_rate)
		{
			// Calculate the total number of samples required
			auto total_samples = 0;
			for (auto const& note : notes)
				total_samples += SampleCount(note, sample_rate);
			
			return total_samples;
		}
		static int SampleCount(Note const& note, ESampleRate sample_rate)
		{
			return (static_cast<int>(sample_rate)* note.m_duration_ms + 999) / 1000;
		};

		// Generate wave data for the given sequence of 'notes'
		template <typename Elem, typename Out>
		static void GenerateWaveData(std::span<Note const> notes, ESampleRate sample_rate, Out out)
		{
			// Scale a normalised value by 'note's volume and bit depth
			auto ScaleSample = [=](Note const& note, float value)
			{
				value = (value > 1.0f) ? 1.0f : (value < -1.0f) ? -1.0f : value;
				if constexpr (std::is_floating_point_v<Elem>)
					return static_cast<Elem>(value * note.m_velocity / 0xFF);
				else if (std::is_signed_v<Elem>)
					return static_cast<Elem>(std::numeric_limits<Elem>::max() * value * note.m_velocity / 0xFF);
				else
					return static_cast<Elem>(std::numeric_limits<Elem>::max() * 0.5f * (1.0f + value * note.m_velocity / 0xFF));
			};

			std::normal_distribution<float> dist(0.0f, +1.0f);
			std::default_random_engine rng;

			// Fill the buffer
			auto sbeg = 0;
			auto phase = 0.0f;
			auto sec_p_sample = 1.0f / sample_rate;
			for (auto const& note : notes)
			{
				auto count = SampleCount(note, sample_rate);
				auto freq  = Frequency(note.m_note);
				auto time  = freq != 0 ? phase / freq : 0.0f;
				auto s     = sbeg;
				auto send  = sbeg + count;
				auto duty  = count * note.m_duty / 0xFF;

				// Fill the first half of the duty cycle with tone
				auto prev_value = 0.0f;
				for (; s != send; ++s, time += sec_p_sample)
				{
					// Notes:
					//  - All wave forms should start at 0 and end at 0 with the first
					//    half being positive and the second negative. This is so that
					//    phase can be matched between tone types.
					auto value = 0.0f;
					switch (note.m_tone)
					{
					case ETone::Sine:
						{
							value = std::sin(time * freq * constants<float>::tau);
							break;
						}
					case ETone::Square:
						{
							value = std::fmod(time * freq, 1.0f) < 0.5f ? 1.0f : -1.0f;
							break;
						}
					case ETone::Triangle:
						{
							value = 4.0f * std::fmod(time * freq, 1.0f);
							value =
								(value < 1.0f) ? (value + 0.0f) :
								(value < 2.0f) ? (2.0f - value) :
								(value < 3.0f) ? (2.0f - value) :
								(value < 4.0f) ? (value - 4.0f) :
								0.0f;
							break;
						}
					case ETone::SawTooth:
						{
							value = 2.0f * std::fmod(time * freq, 1.0f);
							value =
								(value < 1.0f) ? (value + 0.0f) :
								(value < 2.0f) ? (value - 2.0f) :
								0.0f;
							break;
						}
					case ETone::Noise:
						{
							value = dist(rng);
							break;
						}
					default:
						{
							throw std::runtime_error("Unknown tone type");
						}
					}

					// When we reach the end of the duty cycle, wait for the signal to cross zero
					if (s > duty && std::signbit(prev_value) != std::signbit(value))
					{
						break;
					}
					else
					{
						out(ScaleSample(note, value));
						prev_value = value;
					}
				}

				// Fill the second half with silence
				for (; s != send; ++s)
				{
					out(ScaleSample(note, 0.0f));
				}

				// Find the ending phase
				phase = freq != 0 ? std::fmodf(time * freq, 1.0f) : 0.0f;
			}
		}
	};
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
namespace pr::audio
{
	PRUnitTest(SynthTests, Quick)
	{
		int const sample_rate = 48000;
		Note const data[] =
		{
			{"A4", 1000, 1.0f, 1.0f, ETone::Sine},
		};

		auto samples = std::vector<float>{};
		samples.reserve(Synth::SampleCount(data, sample_rate));
		Synth::GenerateWaveData<float>(data, sample_rate, [&](auto sample)
			{
				samples.push_back(sample);
			});

		PR_EXPECT(samples.size() == static_cast<std::size_t>(sample_rate));
		PR_EXPECT(std::abs(Frequency(data[0].m_note) - 440.0f) < 0.01f);
		auto peak = *std::max_element(samples.begin(), samples.end());
		PR_EXPECT(peak > 0.99f && peak <= 1.0f);
		PR_EXPECT(std::abs(samples.front()) < 1.0e-6f);

		// Duty-cycle termination must not emit the first silent sample twice.
		Note const clipped[] =
		{
			{"C4", 120, 0.8f},
			{"G4", 600, 0.5f},
			{"C5", 70, 1.0f, 0.5f, ETone::Noise},
		};
		auto clipped_count = 0;
		Synth::GenerateWaveData<std::uint8_t>(clipped, sample_rate, [&](std::uint8_t)
			{
				++clipped_count;
			});
		PR_EXPECT(clipped_count == Synth::SampleCount(clipped, sample_rate));
	}
}
#endif