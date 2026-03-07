//***************************************************************************************************
// Audio
//  Copyright (c) Rylogic Ltd 2017
//***************************************************************************************************
// A basic tone generator
#pragma once
#include <stdexcept>
#include <type_traits>
#include <chrono>
#include <limits>
#include <cassert>
#include <random>
#include "pr/math/math.h"
#include "pr/container/span.h"
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
							value = std::sinf(time * freq * constants<float>::tau);
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
						value = 0.0f;
						out(ScaleSample(note, value));
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
#include <fstream>
#include "pr/common/unittests.h"
#include "pr/container/byte_data.h"
#include "pr/audio/waves/wave_file.h"
namespace pr::audio
{
	PRUnitTest(SynthTests)
	{
		int const sample_rate = 100000;
		Note const data[] =
		{
			{"C0", 300, 1.0f, 0.9f, ETone::Sine},
			{"G1", 300, 0.5f, 0.9f, ETone::Sine},
			{"G2", 300, 1.0f, 0.9f, ETone::Square},
			{"D1", 300, 1.0f, 0.9f, ETone::Square},
			{"D1", 300, 1.0f, 0.9f, ETone::Triangle},
			{"D1", 300, 1.0f, 0.9f, ETone::SawTooth},
			{"G0", 300, 1.0f, 0.9f, ETone::Noise},
			{"G2", 300, 1.0f, 0.9f, ETone::Noise},
			{"G5", 300, 1.0f, 0.9f, ETone::Noise},
		};

		pr::ByteData<4> buf;
		buf.push_back(WaveHeader(Synth::SampleCount(data, sample_rate), sample_rate, 1, 8));
		Synth::GenerateWaveData<uint8_t>(data, sample_rate, [&](auto b){ buf.push_back(b); });

		// Use Audacity to view the audio file data
		//std::ofstream("P:\\dump\\audio.wav", std::ios::binary).write((char const*)buf.data(), buf.size());
	}
}
#endif