#pragma once
#include "src/forward.h"
#include "src/icommand.h"

using namespace pr::geometry;

namespace fbx_cmd
{
	struct DumpFbx :ICommand
	{
		// Notes:
		//  - Test command lines:
		//   fbx-cmd -dump E:\Rylogic\Code\art\models\AnimCharacter\AnimatedCharacter.fbx

		std::filesystem::path m_filepath;
		ESceneParts m_parts = ESceneParts::All;

		void ShowHelp() const override
		{
			std::cout <<
				"Dump the structure of an FBX file\n"
				" Syntax: fbx-cmd -dump filename.fbx\n"
				;
		}

		bool CmdLineOption(std::string const& option, TArgIter& arg, TArgIter arg_end) override
		{
			if (pr::str::EqualI(option, "-dump")) { return true; }
			//fbx::EParts::NodeHierarchy |
			//fbx::EParts::Meshes |
			//fbx::EParts::Skeletons |
			//fbx::EParts::Skins |
			//fbx::EParts::None,
			return ICommand::CmdLineOption(option, arg, arg_end);
		}

		bool CmdLineData(TArgIter& arg, TArgIter) override
		{
			m_filepath = *arg++;
			return true;
		}

		int Run() override
		{
			if (m_filepath.empty())
				throw std::runtime_error("No input file specified");

			std::ifstream ifile(m_filepath, std::ios::binary);
			fbx::Scene scene(ifile, fbx::LoadOptions{
				.target_axes = {
					.right = fbx::ECoordAxis::PosX,
					.up = fbx::ECoordAxis::PosY,
					.front = fbx::ECoordAxis::NegZ,
				},
			});

			scene.Dump(std::cout, {
				.m_parts = m_parts,
			});

			return 0;
		}
	};
}
