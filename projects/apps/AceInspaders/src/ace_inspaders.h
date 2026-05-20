#pragma once

#include "pr/app/forward.h"
#include "pr/app/main.h"
#include "pr/app/main_ui.h"
#include "pr/app/default_setup.h"
#include "pr/container/byte_data.h"
#include "pr/view3d-12/material/material_simple.h"
#include "pr/audio/audio.h"
#include "pr/audio/synth/note.h"
#include "pr/audio/synth/synth.h"
#include "pr/audio/waves/wave_file.h"
#include "src/space_invaders.h"

#pragma comment(lib, "audio.lib")
//#pragma comment(lib, "winmm.lib")

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
		using SoundBank = std::vector<pr::ByteData<4>>;
		using AudioManager = pr::audio::AudioManager;
		static char const* AppName() { return "AceInspaders"; };

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
		AudioManager m_audio;
		SoundBank m_sounds;
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

			// Load sounds
			InitSounds();

			// Initialise the display
			DoRender();
		}
		~Main()
		{
			// Clear the draw lists so that destructing models
			// don't assert because they're still in a drawlist.
			m_scene.ClearDrawlists();
		}

		// Step the game
		void Step(double elapsed_s)
		{
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

		// Populate the sound bank
		void InitSounds()
		{
			using namespace pr::audio;
			pr::ByteData<4> buf; buf.reserve(1024);
			ESampleRate const sample_rate = ESampleRate::_44100;
			m_sounds.resize(static_cast<int>(SpaceInvaders::ESound::NumberOf));

			// LevelStart
			{
				Note const data[] =
				{
					{"C4", 120, 0.8f},
					{"C4", 120, 0.8f},
					{"C4", 120, 0.8f},
					{"G4", 600}
				};
				WaveHeader const hdr(Synth::SampleCount(data, sample_rate), sample_rate, 1, 8);

				buf.resize(0);
				buf.push_back(hdr);
				Synth::GenerateWaveData<uint8_t>(data, sample_rate, [&](auto b) { buf.push_back(b); });
				m_sounds[static_cast<int>(SpaceInvaders::ESound::LevelStart)] = buf;
			}
			// AlienAdvance:
			{
				Note const data[] = {{"C2", 50, 1.0f, 0.1f}};
				WaveHeader const hdr(Synth::SampleCount(data, sample_rate), sample_rate, 1, 8);

				buf.resize(0);
				buf.push_back(hdr);
				Synth::GenerateWaveData<uint8_t>(data, sample_rate, [&](auto b) { buf.push_back(b); });
				m_sounds[static_cast<int>(SpaceInvaders::ESound::AlienAdvance)] = buf;
			}
			// PlayerShoot
			{
				Note const data[] = {{"G6", 10}, {"Gb6", 10}, {"F6", 10}, {"E6", 10}, {"Eb6", 10}, {"D6", 10}};
				WaveHeader const hdr(Synth::SampleCount(data, sample_rate), sample_rate, 1, 8);

				buf.resize(0);
				buf.push_back(hdr);
				Synth::GenerateWaveData<uint8_t>(data, sample_rate, [&](auto b) { buf.push_back(b); });
				m_sounds[static_cast<int>(SpaceInvaders::ESound::PlayerShoot)] = buf;
			}
			// AlienBombDrop
			{
				//static speaker_tone_t const tune_data[] = {{NOTE_G5, 10}, {NOTE_Gb5, 10}, {NOTE_F5, 10}, {NOTE_E5, 10}, {NOTE_Eb5, 10}, {NOTE_D5, 10}};
				//static tune_sequence_t const seq = {&tune_data[0], countof(tune_data), ALERT_PRIORITY_INFO, 0};
				//Audio_PlayTune(&seq, 50, TUNE_SEQUENCING_TRANSIENT);
				//SpaceInvaders::ESound::AlienBombDrop:
			}
			// AlienDestroyed
			{
				Note const data[] = {{"C5", 70, 1.0f, 0.5f, ETone::Noise}};
				WaveHeader const hdr(Synth::SampleCount(data, sample_rate), sample_rate, 1, 8);

				buf.resize(0);
				buf.push_back(hdr);
				Synth::GenerateWaveData<uint8_t>(data, sample_rate, [&](auto b) { buf.push_back(b); });
				m_sounds[static_cast<int>(SpaceInvaders::ESound::AlienDestroyed)] = buf;
			}
			// PlayerDestroyed
			{
				Note const data[] =
				{
					{"C3" , 30, 1.0f, 0.5f, ETone::Noise},
					{"Db3", 30, 1.0f, 0.5f, ETone::Noise},
					{"C3" , 30, 1.0f, 0.5f, ETone::Noise},
				};
				WaveHeader const hdr(Synth::SampleCount(data, sample_rate), sample_rate, 1, 8);

				buf.resize(0);
				buf.push_back(hdr);
				Synth::GenerateWaveData<uint8_t>(data, sample_rate, [&](auto b) { buf.push_back(b); });
				m_sounds[static_cast<int>(SpaceInvaders::ESound::PlayerDestroyed)] = buf;
			}
			// BunkerDamaged
			{
				Note const data[] = {{"C4", 20, 1.0f, 0.5f, ETone::Noise}};
				WaveHeader const hdr(Synth::SampleCount(data, sample_rate), sample_rate, 1, 8);

				buf.resize(0);
				buf.push_back(hdr);
				Synth::GenerateWaveData<uint8_t>(data, sample_rate, [&](auto b) { buf.push_back(b); });
				m_sounds[static_cast<int>(SpaceInvaders::ESound::BunkerDamaged)] = buf;
			}
			// BombDestroyed
			{
				Note const data[] = {{"C6", 70, 1.0f, 0.5f, ETone::Noise}};
				WaveHeader const hdr(Synth::SampleCount(data, sample_rate), sample_rate, 1, 8);

				buf.resize(0);
				buf.push_back(hdr);
				Synth::GenerateWaveData<uint8_t>(data, sample_rate, [&](auto b) { buf.push_back(b); });
				m_sounds[static_cast<int>(SpaceInvaders::ESound::BombDestroyed)] = buf;
			}
			// LevelCompleted
			{
				Note const data[] =
				{
					{"F4", 220, 0.91f},
					{"G4", 120, 0.8f},
					{"F4", 120, 0.8f},
					{"G4", 120, 0.8f},
					{"Bb4", 1200}
				};
				WaveHeader const hdr(Synth::SampleCount(data, sample_rate), sample_rate, 1, 8);

				buf.resize(0);
				buf.push_back(hdr);
				Synth::GenerateWaveData<uint8_t>(data, sample_rate, [&](auto b) { buf.push_back(b); });
				m_sounds[static_cast<int>(SpaceInvaders::ESound::LevelCompleted)] = buf;
			}
			// GameOver
			{
				Note const data[] =
				{
					{"Eb4", 220, 0.91f},
					{"D4" , 220, 0.91f},
					{"Db4", 220, 0.91f},
					{"C4" , 1200}
				};
				WaveHeader const hdr(Synth::SampleCount(data, sample_rate), sample_rate, 1, 8);

				buf.resize(0);
				buf.push_back(hdr);
				Synth::GenerateWaveData<uint8_t>(data, sample_rate, [&](auto b) { buf.push_back(b); });
				m_sounds[static_cast<int>(SpaceInvaders::ESound::GameOver)] = buf;
			}
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
		(void)sound;
		//auto data = reinterpret_cast<char const*>(m_sounds[static_cast<int>(sound)].data());
		//::PlaySoundA(data, nullptr, SND_MEMORY | SND_ASYNC);
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
