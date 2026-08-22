#pragma once

#include "pr/app/forward.h"
#include "pr/app/main.h"
#include "pr/app/main_ui.h"
#include "pr/app/default_setup.h"
#include "pr/audio/audio.h"
#include "pr/view3d-12/material/material_simple.h"
#include "src/space_invaders.h"

namespace ace
{
	struct Main;
	struct MainUI;

	// Create a UserSettings object for loading/saving app settings
	struct UserSettings
	{
		explicit UserSettings(int) {}
	};

	// Derive a application logic type from pr::app::Main
	struct Main
		:pr::app::Main<Main, MainUI, UserSettings>
		,pr::SpaceInvaders::ISystem
	{
		using base = pr::app::Main<Main, MainUI, UserSettings>;
		using ResourceFactory = pr::rdr12::ResourceFactory;
		using Texture2DPtr = pr::rdr12::Texture2DPtr;
		using ResDesc = pr::compute::ResDesc;
		using Image = pr::compute::Image;
		using UpdateSubresourceScope = pr::compute::GfxUpdateSubresourceScope;
		using SpaceInvaders = pr::SpaceInvaders;
		using Screen = pr::SpaceInvaders::Screen;
		static char const* AppName() { return "AceInspaders"; };
		static constexpr auto SoundVoiceCount = 4;

		// Cache one generated clip and a bounded overlap pool for one game effect.
		struct SoundEffect
		{
			pr::audio::ClipHandle m_clip;
			std::array<pr::audio::VoiceHandle, SoundVoiceCount> m_voices;
			std::size_t m_next_voice;
		};

		struct ScreenQuad
		{
			#define PR_RDR_INST(x)\
			x(pr::m4x4           , m_i2w, pr::rdr12::EInstComp::I2WTransform)\
			x(pr::rdr12::ModelPtr, m_model, pr::rdr12::EInstComp::ModelPtr)
			PR_RDR12_INSTANCE_MEMBERS(ScreenQuad, PR_RDR_INST);
			#undef PR_RDR_INST
		};

		SpaceInvaders m_space_invaders;
		ResourceFactory m_factory;
		Texture2DPtr m_screen_tex;
		ScreenQuad m_screen_quad;
		pr::audio::Engine m_audio;
		std::array<SoundEffect, static_cast<std::size_t>(SpaceInvaders::ESound::NumberOf)> m_sounds;
		Screen m_display;

		Main(MainUI& ui)
			: base(pr::app::DefaultSetup(), ui)
			, m_factory(m_rdr)
			, m_space_invaders(this)
			, m_screen_tex()
			, m_screen_quad()
			, m_audio()
			, m_sounds()
			, m_display()
		{
			using namespace pr;
			using namespace pr::rdr12;
		
			// Orthographic camera
			m_cam.Orthographic(true);

			ResDesc rdesc = ResDesc::Tex2D(Image{SpaceInvaders::ScreenDimX, SpaceInvaders::ScreenDimY});
			TextureDesc tdesc = TextureDesc(rdr12::AutoId, rdesc).name("ScreenBuf");
			m_screen_tex = m_factory.CreateTexture2D(tdesc);
			m_screen_tex->m_t2s.y = -m_screen_tex->m_t2s.y;
			m_screen_tex->m_t2s.pos.y = 1.0f;

			// Setup a flat light
			m_scene.m_global_light.m_type = ELight::Ambient;
			m_scene.m_global_light.m_ambient = 0xFF808080;

			// Set up the renderer to render a quad containing a texture
			auto material = MaterialPtr(::pr::compute::New<MaterialSimple>(Colour32White, m_screen_tex, m_factory.CreateSampler(EStockSampler::PointClamp)), true);
			ModelGenerator::CreateOptions opts = ModelGenerator::CreateOptions().material(material);
			m_screen_quad.m_model = ModelGenerator::Quad(m_factory, &opts);
			m_screen_quad.m_i2w = m4x4::Scale((float)SpaceInvaders::ScreenDimX / SpaceInvaders::ScreenDimY, 1.0f, 1.0f, v4::Origin());

			// Add the quad to the scene
			m_scene.OnUpdateScene += std::bind(&Main::UpdateScene, this, _1);

			// Generate and cache the complete procedural sound bank before gameplay starts.
			InitSounds();

			// Initialise the display
			DoRender();
		}
		~Main()
		{
			// Voices retain clip references, so release each pool before destroying its cached clip.
			for (auto& sound : m_sounds)
			{
				for (auto voice : sound.m_voices)
				if (voice != 0)
					m_audio.VoiceDestroy(voice);

				if (sound.m_clip != 0)
					m_audio.ClipDestroy(sound.m_clip);
			}

			// Clear the draw lists so that destructing models
			// don't assert because they're still in a drawlist.
			m_scene.ClearDrawlists();
		}

		// Step the game
		void Step(double elapsed_s)
		{
			// Publish completions before gameplay requests another overlapping instance.
			m_audio.Update();
			m_space_invaders.Step(static_cast<int>(elapsed_s * 1000));
		}

		// Prepare the scene for render
		void UpdateScene(pr::rdr12::Scene& scene)
		{
			using namespace pr;
			using namespace pr::rdr12;

			// Render the display
			m_space_invaders.Render(m_display);
			{
				UpdateSubresourceScope update(m_factory.CmdList(), m_factory.UploadBuffer(), m_screen_tex->m_res.get(), 0, 0, 1, alignof(uint32_t));
				for (int y = 0; y != m_display.m_dimy; ++y)
				{
					auto* px = update.ptr<uint32_t>(iv3{ 0, y, 0 });
					for (int x = 0; x != m_display.m_dimx; ++x)
					{
						if (m_display(x, y))
							*px++ = 0xFF000000;
						else
							*px++ = 0xFFA0A0A0;
					}
				}
				update.Commit();
				m_factory.FlushToGpu(EGpuFlush::Block);
			}

			scene.AddInstance(m_screen_quad);
		}

		// Generate a complete mono PCM8 WAV file for one procedural note sequence.
		static std::vector<std::byte> MakeWave(std::span<pr::audio::Note const> notes, pr::audio::ESampleRate sample_rate)
		{
			auto sample_count = pr::audio::Synth::SampleCount(notes, sample_rate);
			auto data_size = static_cast<std::uint32_t>(sample_count);
			auto wave = std::vector<std::byte>(44 + data_size);
			auto write = [&](std::size_t offset, auto value)
			{
				std::memcpy(wave.data() + offset, &value, sizeof(value));
			};

			// Emit the canonical 44-byte PCM header expected by the resident clip loader.
			write(0, std::uint32_t{0x46464952});
			write(4, std::uint32_t{36 + data_size});
			write(8, std::uint32_t{0x45564157});
			write(12, std::uint32_t{0x20746D66});
			write(16, std::uint32_t{16});
			write(20, std::uint16_t{1});
			write(22, std::uint16_t{1});
			write(24, static_cast<std::uint32_t>(sample_rate));
			write(28, static_cast<std::uint32_t>(sample_rate));
			write(32, std::uint16_t{1});
			write(34, std::uint16_t{8});
			write(36, std::uint32_t{0x61746164});
			write(40, data_size);

			auto sample = std::size_t{44};
			pr::audio::Synth::GenerateWaveData<std::uint8_t>(notes, sample_rate, [&](std::uint8_t value)
				{
					wave[sample++] = static_cast<std::byte>(value);
				});
			return wave;
		}

		// Cache one generated clip and pre-create its reusable non-spatial effect voices.
		void InitSound(SpaceInvaders::ESound sound_id, std::span<pr::audio::Note const> notes)
		{
			auto& sound = m_sounds[static_cast<std::size_t>(sound_id)];
			auto wave = MakeWave(notes, pr::audio::ESampleRate::_44100);
			sound.m_clip = m_audio.ClipCreateWave(wave);
			auto voice_desc = pr::audio::VoiceDesc{
				.header = {sizeof(pr::audio::VoiceDesc), pr::audio::AUDIO_STRUCT_VERSION},
				.clip = sound.m_clip,
				.bus = pr::audio::EBus::Effects,
				.spatial = false,
				.loop_count = 0,
				.priority = 100,
				.volume = 1.0f,
				.pitch = 1.0f,
			};
			for (auto& voice : sound.m_voices)
				voice = m_audio.VoiceCreate(voice_desc);
		}

		// Build the original synthesized effect set once so gameplay never generates or parses audio on demand.
		void InitSounds()
		{
			using namespace pr::audio;

			Note const level_start[] =
			{
				{"C4", 120, 0.8f},
				{"C4", 120, 0.8f},
				{"C4", 120, 0.8f},
				{"G4", 600},
			};
			InitSound(SpaceInvaders::ESound::LevelStart, level_start);

			Note const alien_advance[] = {{"C2", 50, 1.0f, 0.1f}};
			InitSound(SpaceInvaders::ESound::AlienAdvance, alien_advance);

			Note const player_shoot[] = {{"G6", 10}, {"Gb6", 10}, {"F6", 10}, {"E6", 10}, {"Eb6", 10}, {"D6", 10}};
			InitSound(SpaceInvaders::ESound::PlayerShoot, player_shoot);

			// AlienBombDrop had no generated waveform in the original sound bank and remains intentionally silent.
			Note const alien_destroyed[] = {{"C5", 70, 1.0f, 0.5f, ETone::Noise}};
			InitSound(SpaceInvaders::ESound::AlienDestroyed, alien_destroyed);

			Note const player_destroyed[] =
			{
				{"C3" , 30, 1.0f, 0.5f, ETone::Noise},
				{"Db3", 30, 1.0f, 0.5f, ETone::Noise},
				{"C3" , 30, 1.0f, 0.5f, ETone::Noise},
			};
			InitSound(SpaceInvaders::ESound::PlayerDestroyed, player_destroyed);

			Note const bunker_damaged[] = {{"C4", 20, 1.0f, 0.5f, ETone::Noise}};
			InitSound(SpaceInvaders::ESound::BunkerDamaged, bunker_damaged);

			Note const bomb_destroyed[] = {{"C6", 70, 1.0f, 0.5f, ETone::Noise}};
			InitSound(SpaceInvaders::ESound::BombDestroyed, bomb_destroyed);

			Note const level_completed[] =
			{
				{"F4", 220, 0.91f},
				{"G4", 120, 0.8f},
				{"F4", 120, 0.8f},
				{"G4", 120, 0.8f},
				{"Bb4", 1200},
			};
			InitSound(SpaceInvaders::ESound::LevelCompleted, level_completed);

			Note const game_over[] =
			{
				{"Eb4", 220, 0.91f},
				{"D4" , 220, 0.91f},
				{"Db4", 220, 0.91f},
				{"C4" , 1200},
			};
			InitSound(SpaceInvaders::ESound::GameOver, game_over);
		}

		// Play the indicated sound
		void SpaceInvaders::ISystem::PlaySound(SpaceInvaders::ESound);

		// Return user input
		SpaceInvaders::UserInputData SpaceInvaders::ISystem::UserInput();
	};

	// Derive a GUI class from pr::app::MainGUI
	struct MainUI :pr::app::MainUI<MainUI, Main, pr::gui::WinGuiMsgLoop>
	{
		using base_type = pr::app::MainUI<MainUI, Main, pr::gui::WinGuiMsgLoop>;

		static int const Scale = 2;
		static wchar_t const* AppTitle() { return L"Ace Inspaders"; };
		MainUI(wchar_t const*, int)
			:base_type(Params()
			.title(AppTitle())
			.padding(0)
			.wh(Scale* pr::SpaceInvaders::ScreenDimX, Scale* pr::SpaceInvaders::ScreenDimY, true)
			.default_mouse_navigation(false))
		{
			m_msg_loop.AddLoop(60.0, false, [this](double dt) { m_main->Step(dt); });
			m_msg_loop.AddLoop(60.0, true, [this](double) { m_main->DoRender(true); });
		}
	};

	// Play the indicated sound
	inline void Main::PlaySound(SpaceInvaders::ESound sound)
	{
		switch (sound)
		{
			case SpaceInvaders::ESound::LevelStart:
			case SpaceInvaders::ESound::AlienAdvance:
			case SpaceInvaders::ESound::PlayerShoot:
			case SpaceInvaders::ESound::AlienDestroyed:
			case SpaceInvaders::ESound::PlayerDestroyed:
			case SpaceInvaders::ESound::BunkerDamaged:
			case SpaceInvaders::ESound::BombDestroyed:
			case SpaceInvaders::ESound::LevelCompleted:
			case SpaceInvaders::ESound::GameOver:
			{
				break;
			}
			case SpaceInvaders::ESound::AlienBombDrop:
			{
				return;
			}
			case SpaceInvaders::ESound::NumberOf:
			default:
			{
				throw std::out_of_range("Unknown AceInspaders sound effect");
			}
		}

		// Prefer an idle instance so rapid effects overlap; if the pool is full, restart its oldest slot.
		auto& effect = m_sounds[static_cast<std::size_t>(sound)];
		for (auto offset = std::size_t{}; offset != effect.m_voices.size(); ++offset)
		{
			auto index = (effect.m_next_voice + offset) % effect.m_voices.size();
			if (m_audio.VoiceStateGet(effect.m_voices[index]).playback != pr::audio::EPlaybackState::Stopped)
				continue;

			m_audio.VoicePlay(effect.m_voices[index]);
			effect.m_next_voice = (index + 1) % effect.m_voices.size();
			return;
		}

		m_audio.VoicePlay(effect.m_voices[effect.m_next_voice]);
		effect.m_next_voice = (effect.m_next_voice + 1) % effect.m_voices.size();
	}

	// Return user input
	inline Main::SpaceInvaders::UserInputData Main::UserInput()
	{
		auto rect = m_ui.ClientRect(false);
		auto pt = m_ui.MousePosition();
		pt = m_ui.PointToClient(pt);

		SpaceInvaders::UserInputData data = {};
		data.JoystickX =  static_cast<int>((2.0f * pt.x / rect.width() - 1.0f) * SpaceInvaders::UserInputData::AxisMaxAbs);
		data.FireButton = m_ui.KeyState(VK_LBUTTON);
		return data;
	}
}

namespace pr::app
{
	// Create the GUI window
	inline std::unique_ptr<IAppMainUI> CreateUI(wchar_t const* lpstrCmdLine, int nCmdShow)
	{
		return std::unique_ptr<IAppMainUI>(new ace::MainUI(lpstrCmdLine, nCmdShow));
	}
}
