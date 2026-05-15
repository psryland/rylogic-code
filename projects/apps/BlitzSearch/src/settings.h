//***********************************************************************
// BlitzSearch
//  Copyright (c) Rylogic Ltd 2024
//***********************************************************************
#pragma once
#include "src/forward.h"

namespace blitzsearch
{
	struct Settings
	{
		std::vector<std::filesystem::path> SearchPaths;
		std::vector<std::string> FileExtensions;
		int64_t MaxFileSize;

		Settings()
			:SearchPaths()
			,FileExtensions()
			,MaxFileSize()
		{
			// Find the settings file path
			auto settings_path = pr::filesys::GetUserDocumentsPath() / "Rylogic" / "BlitzSearch" / "settings.json";
#if _DEBUG
			settings_path = pr::filesys::GetExecutablePath().parent_path() / "settings.json";
#endif
			if (!std::filesystem::exists(settings_path))
				throw std::runtime_error(std::format("Settings file ({}) not found", settings_path.string()));

			// Load the settings
			auto settings_data = pr::json::Read(settings_path, { .AllowComments = true, .AllowTrailingCommas = true });

			for (auto x : settings_data["SearchPaths"].to_array())
				SearchPaths.push_back(x.to<std::filesystem::path>());
			for (auto x : settings_data["FileExtensions"].to_array())
				FileExtensions.push_back(x.to<std::string>());
			MaxFileSize = settings_data["MaxFileSize"].to<int64_t>();
		}
	};
}