//***************************************************************************************************
// Audio
//  Copyright (c) Rylogic Ltd 2017
//***************************************************************************************************
// A basic tone generator
#pragma once
#include "pr/audio/forward.h"

namespace pr::audio
{
	static constexpr int OctaveMax = 10;
	static constexpr int NotesPerOctave = 12;
	static constexpr int NoteBits = 4;
	static constexpr int OctaveBits = 4;
	static constexpr int NoteMask = 0x0F;
	static constexpr int OctaveMask = 0xF0;

	// Note names
	enum class ENote :uint8_t
	{
		// Notes
		C = 0,
		Cs, Db = Cs,
		D,
		Ds, Eb = Ds,
		E,
		F,
		Fs, Gb = Fs,
		G,
		Gs, Ab = Gs,
		A,
		As, Bb = As,
		B,

		// Octaves
		O0 = 0 << NoteBits,
		O1 = 1 << NoteBits,
		O2 = 2 << NoteBits,
		O3 = 3 << NoteBits,
		O4 = 4 << NoteBits,
		O5 = 5 << NoteBits,
		O6 = 6 << NoteBits,
		O7 = 7 << NoteBits,
		O8 = 8 << NoteBits,
		O9 = 9 << NoteBits,

		_flags_enum = 0,
	};

	// Tone names
	enum class ETone :uint8_t
	{
		Sine,
		Square,
		Triangle,
		SawTooth,
		Noise,
	};

	// Convert a note and octave to a frequency (in hz)
	constexpr float Frequency(ENote note)
	{
		// http://www.sengpielaudio.com/calculator-notenames.htm
		// Oct\Note     C        Db       D        Eb       E        F        Gb       G        Ab       A        Bb       B   
		//    3      130.813  138.591  146.832  155.563  164.814  174.614  184.997  195.998  207.652  220.000  233.082  246.942
		//    4      261.626  277.183  293.665  311.127  329.628  349.228  369.994  391.995  415.305  440.000  466.164  493.883
		//    5      523.251  554.365  587.330  622.254  659.255  698.456  739.989  783.991  830.609  880.000  932.328  987.767
		//    6     1046.502 1108.731 1174.659 1244.508 1318.510 1396.913 1479.978 1567.982 1661.219 1760.000 1864.655 1975.533
		//    7     2093.005 2217.461 2349.318 2489.016 2637.020 2793.826 2959.955 3135.963 3322.438 3520.000 3729.310 3951.066
		constexpr float NoteFrequencies[OctaveMax * NotesPerOctave] =
		{
			 16.3516f ,   17.324f ,    18.354f ,    19.445f ,    20.602f ,    21.827f ,    23.125f ,    24.500f ,    25.957f ,    27.500f ,    29.135f ,    30.868f ,// Octave 0
			 32.7032f ,   34.648f ,    36.708f ,    38.891f ,    41.204f ,    43.654f ,    46.249f ,    49.000f ,    51.913f ,    55.000f ,    58.271f ,    61.736f ,// Octave 1
			 65.4065f ,   69.296f ,    73.416f ,    77.782f ,    82.407f ,    87.307f ,    92.499f ,    97.999f ,   103.826f ,   110.000f ,   116.541f ,   123.471f ,// Octave 2
			 130.813f ,  138.591f ,   146.832f ,   155.563f ,   164.814f ,   174.614f ,   184.997f ,   195.998f ,   207.652f ,   220.000f ,   233.082f ,   246.942f ,// Octave 3
			 261.626f ,  277.183f ,   293.665f ,   311.127f ,   329.628f ,   349.228f ,   369.994f ,   391.995f ,   415.305f ,   440.000f ,   466.164f ,   493.883f ,// Octave 4
			 523.251f ,  554.365f ,   587.330f ,   622.254f ,   659.255f ,   698.456f ,   739.989f ,   783.991f ,   830.609f ,   880.000f ,   932.328f ,   987.767f ,// Octave 5
			1046.502f , 1108.731f ,  1174.659f ,  1244.508f ,  1318.510f ,  1396.913f ,  1479.978f ,  1567.982f ,  1661.219f ,  1760.000f ,  1864.655f ,  1975.533f ,// Octave 6
			2093.005f , 2217.461f ,  2349.318f ,  2489.016f ,  2637.020f ,  2793.826f ,  2959.955f ,  3135.963f ,  3322.438f ,  3520.000f ,  3729.310f ,  3951.066f ,// Octave 7
			4186.010f , 4434.922f ,  4698.636f ,  4978.032f ,  5274.040f ,  5587.652f ,  5919.910f ,  6271.926f ,  6644.876f ,  7040.000f ,  7458.620f ,  7902.132f ,// Octave 8
			8372.020f , 8869.844f ,  9397.272f ,  9956.064f , 10548.080f , 11175.304f , 11839.820f , 12543.852f , 13289.752f , 14080.000f , 14917.240f , 15804.264f ,// Octave 9
		};
		auto n = (static_cast<int>(note) & NoteMask  ) >> 0;
		auto o = (static_cast<int>(note) & OctaveMask) >> NoteBits;
		if (n < 0 || n >= NotesPerOctave || o < 0 || o >= OctaveMax) throw std::runtime_error("Invalid note");
		return NoteFrequencies[o * NotesPerOctave + n];
	}

	// A single sound
	struct Note
	{
		ENote    m_note;        // Note name and octave
		ETone    m_tone;        // The type of sound to generate
		uint16_t m_duration_ms; // The length of the note (in ms)
		uint8_t  m_duty;        // The fraction of the note length that isn't silence (0xFF = 100%)
		uint8_t  m_velocity;    // The volume of the note (0xFF = max)
		uint8_t pad[2];

		Note()
			: m_note()
			, m_tone()
			, m_duration_ms()
			, m_duty()
			, m_velocity()
			, pad()
		{}
		Note(char const* note, uint16_t duration_ms, float duty = 1.0f, float velocity = 0.5f, ETone tone = ETone::Sine)
			: m_note()
			, m_tone(tone)
			, m_duration_ms(duration_ms)
			, m_duty(duty < 0 ? 0 : duty > 1.0f ? 0xFF : static_cast<uint8_t>(duty * 0xFF))
			, m_velocity(velocity < 0 ? 0 : velocity > 1.0f ? 0xFF : static_cast<uint8_t>(velocity * 0xFF))
			, pad()
		{
			if (note == nullptr || note[0] == '\0' || note[1] == '\0')
				throw std::runtime_error("Invalid note");
			
			auto idx =
				(note[0] >= 'a' && note[0] <= 'g') ? note[0] - 'a' :
				(note[0] >= 'A' && note[0] <= 'G') ? note[0] - 'A' :
				-1;
			auto ofs =
				(note[1] == 's') ? +1 :
				(note[1] == 'b') ? -1 :
				0;
			auto oct = 
				(note[1 + (ofs != 0)] - '0');

			static ENote const notes[] = {ENote::A, ENote::B, ENote::C, ENote::D, ENote::E, ENote::F, ENote::G};
			if (idx < 0 || idx >= _countof(notes) || oct < 0 || oct >= OctaveMax)
				throw std::runtime_error("Invalid note");

			auto n = (static_cast<int>(notes[idx]) + ofs + NotesPerOctave) % NotesPerOctave;
			auto o = (oct << NoteBits);
			m_note = static_cast<ENote>(n | o);
		}
	};
	static_assert(sizeof(Note) == 8);
}
