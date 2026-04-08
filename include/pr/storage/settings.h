//*****************************************
// Settings
// Copyright (c) Rylogic Ltd 2013
//*****************************************
// Usage:
// #include "pr/storage/settings.h"
//
//  struct MySettings : SettingsBase<MySettings>
//  {
//      #define PR_SETTING(x)\
//      x(type, name, default_value, description)\
//      x(type, name, default_value, description)\
//      x(type, name, default_value, description)
//      PR_SETTINGS_MEMBERS(MySettings, PR_SETTING);
//      #undef PR_SETTING
//  };
//
#pragma once
#include <string_view>
#include <string>
#include <filesystem>
#include <format>
#include <stdexcept>
#include "pr/common/crc.h"
#include "pr/common/event_handler.h"
#include "pr/math/math.h"
#include "pr/gfx/colour.h"
#include "pr/storage/json.h"

namespace pr::settings
{
	// Event args used to report an error code and error message
	struct ErrorArgs :EmptyArgs
	{
		std::string m_msg;
		int m_code;

		ErrorArgs(std::string_view msg = {}, int code = 0)
			:m_msg(msg)
			,m_code(code)
		{}
	};

	// A CRTP base class for settings types
	template<typename Derived>
	struct SettingsBase
	{
		std::filesystem::path m_filepath; // The file path to save the settings
		CRC m_crc;

		// Settings constructor
		SettingsBase(std::filesystem::path const& filepath, bool throw_on_error = true)
			: m_filepath(filepath)
			, m_crc()
		{
			if (throw_on_error)
			{
				OnError += [](auto&, ErrorArgs const& err)
				{
					throw std::runtime_error(err.m_msg);
				};
			}
		}

		// Raised on error conditions
		EventHandler<SettingsBase&, ErrorArgs const&> OnError;

		// Load settings from file
		bool Load(std::filesystem::path const& filepath, json::Options opts = { .AllowComments = true, .AllowTrailingCommas = true })
		{
			m_filepath = filepath;

			// Check the file exists
			if (!std::filesystem::exists(m_filepath))
				OnError(*this, { std::format("User settings file '{}' not found", m_filepath.string()) });

			try
			{
				auto root = json::Read(m_filepath, opts).to_object();
				static_cast<Derived*>(this)->Import(root);
				return true;
			}
			catch (std::exception const& e)
			{
				OnError(*this, { std::format("Error found while parsing user settings file '{}'.\n{}", m_filepath.string(), e.what()) });
				return false;
			}
		}
		bool Load()
		{
			return Load(m_filepath);
		}

		// Save settings to file
		bool Save(std::filesystem::path const& filepath, json::Options opts = { .AllowComments = true, .AllowTrailingCommas = true })
		{
			m_filepath = filepath;

			// Check the save location is valid
			auto dir = m_filepath.parent_path();
			if (!std::filesystem::exists(dir) && !std::filesystem::create_directories(dir))
			{
				OnError(*this, { std::format("Failed to save user settings file '{}'", m_filepath.string()) });
				return false;
			}

			try
			{
				// Export to json
				json::Value root = static_cast<Derived const*>(this)->Export();
				json::Write(m_filepath, root, opts);
				m_crc = Crc(root);
				return true;
			}
			catch (std::exception const& e)
			{
				OnError(*this, { std::format("Error found while exporting user settings to json.\n{}", e.what()) });
				return false;
			}
		}
		bool Save()
		{
			return Save(m_filepath);
		}

		// Returns true if the settings have changed since last saved
		bool SaveRequired() const
		{
			auto root = static_cast<Derived const*>(this)->Export();
			auto crc = Crc(root);
			return m_crc != crc;
		}

		// Calculate a crc for the given json value.
		CRC Crc(json::Value const& value, CRC crc = 0) const
		{
			if (auto jobj = value.as<json::Object>(); jobj)
			{
				Crc(*jobj, crc);
			}
			else if (auto jarr = value.as<json::Array>(); jarr)
			{
				Crc(*jarr, crc);
			}
			else
			{
				auto value_str = value.str();
				crc = pr::Crc(value_str.size(), value_str.data(), crc);
			}
			return crc;
		}
		CRC Crc(json::Object const& jobj, CRC crc = 0) const
		{
			for (auto const& [key, val] : jobj)
			{
				crc = pr::Crc(key.size(), key.data(), crc);
				crc = Crc(val, crc);
			}
			return crc;
		}
		CRC Crc(json::Array const& jarr, CRC crc = 0) const
		{
			for (auto const& item : jarr)
			{
				crc = Crc(item, crc);
			}
			return crc;
		}

		// Write overloads for supported types.
		template <typename T> json::Value Write(T const& value) const requires (requires (T v) { json::Value(v); })
		{
			return json::Value(value);
		}
		json::Value Write(v2 value) const
		{
			return json::Array{ value.x, value.y };
		}
		json::Value Write(v4 value) const
		{
			return json::Array{ value.x, value.y, value.z, value.w };
		}
		json::Value Write(Colour32 value) const
		{
			return std::format("{:08X}", value.argb);
		}

		// Read overloads for supported types.
		// Uses out-parameter pattern so derived classes can add overloads (template specialisation doesn't work across base/derived).
		template <typename T> void Read(T& out, json::Value const& value) const
		{
			out = value.to<T>();
		}
		void Read(v2& out, json::Value const& value) const
		{
			auto arr = value.to_array();
			out = v2{
				arr[0].to<float>(),
				arr[1].to<float>(),
			};
		}
		void Read(v4& out, json::Value const& value) const
		{
			auto arr = value.to_array();
			out = v4{
				arr[0].to<float>(),
				arr[1].to<float>(),
				arr[2].to<float>(),
				arr[3].to<float>(),
			};
		}
		void Read(Colour32& out, json::Value const& value) const
		{
			auto str = value.to<std::string>();
			if (str.size() != 8)
				throw std::runtime_error(std::format("Invalid colour string '{}'", str));

			uint32_t argb = 0;
			for (size_t i = 0; i < 8; ++i)
			{
				char c = str[i];
				uint32_t nibble = 0;
				if (c >= '0' && c <= '9') nibble = c - '0';
				else if (c >= 'A' && c <= 'F') nibble = c - 'A' + 10;
				else if (c >= 'a' && c <= 'f') nibble = c - 'a' + 10;
				else throw std::runtime_error(std::format("Invalid character '{}' in colour string '{}'", c, str));
				argb = (argb << 4) | nibble;
			}
			out = Colour32(argb);
		}
	};

	// Settings generator.
	#pragma region Settings Generator

	#define PR_SETTINGS_INSTANTIATE(type, name, default_value, description)   type name;
	#define PR_SETTINGS_COUNT(type, name, default_value, description)         +1
	#define PR_SETTINGS_CONSTRUCT(type, name, default_value, description)     ,name(default_value)
	#define PR_SETTINGS_WRITE(type, name, default_value, description)         root[#name] = Write(name);
	#define PR_SETTINGS_READ(type, name, default_value, description)          Read(name, root[#name]);

	// Generator
	#define PR_SETTINGS_MEMBERS(settings_name, fields)\
		/* Members */\
		fields(PR_SETTINGS_INSTANTIATE)\
		static constexpr int NumberOf = 0 fields(PR_SETTINGS_COUNT);\
		\
		/* Constructor */\
		settings_name(std::filesystem::path const& filepath = {}, bool throw_on_error = true)\
			:SettingsBase(filepath, throw_on_error)\
			fields(PR_SETTINGS_CONSTRUCT)\
		{\
			m_crc = Crc(Export());\
		}\
		\
		/* Export to json*/\
		pr::json::Object Export() const\
		{\
			pr::json::Object root;\
			fields(PR_SETTINGS_WRITE)\
			return root;\
		}\
		\
		/* Import from json */\
		void Import(json::Object const& root)\
		{\
			fields(PR_SETTINGS_READ)\
			m_crc = Crc(root);\
		}
	
	#pragma endregion
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
namespace pr::settings::tests
{
	PRUnitTestClass(SettingsTest)
	{
		enum class Enum1 { One, Two, Three, };
		struct Settings :settings::SettingsBase<Settings>
		{
			#define PR_SETTINGS(x)\
			x(int          , count    , 2                 , "")\
			x(float        , scale    , 3.14f             , "")\
			x(int64_t      , mask     , 0xABCU            , "")\
			x(pr::Colour32 , colour   , pr::Colour32Green , "the colour")\
			x(pr::v2       , area     , pr::v2(1,2)       , "")\
			x(pr::v4       , position , pr::v4(1,2,3,1)   , "")\
			x(std::string  , name     , "hello settings"  , "")\
			x(Enum1        , emun     , Enum1::Two        , "")
			PR_SETTINGS_MEMBERS(Settings, PR_SETTINGS);
			#undef PR_SETTINGS

			// Bring base Read/Write into scope so derived overloads don't hide them
			using SettingsBase<Settings>::Read;
			using SettingsBase<Settings>::Write;

			json::Value Write(Enum1 value) const
			{
				switch (value)
				{
					case Enum1::One: return "One";
					case Enum1::Two: return "Two";
					case Enum1::Three: return "Three";
					default: throw std::runtime_error(std::format("Unknown enum value '{}'", static_cast<int>(value)));
				}
			}
			void Read(Enum1& out, json::Value const& value) const
			{
				auto str = value.to<std::string>();
				if (str == "One") out = Enum1::One;
				else if (str == "Two") out = Enum1::Two;
				else if (str == "Three") out = Enum1::Three;
				else throw std::runtime_error(std::format("Unknown enum string '{}'", str));
			}
		};

		PRUnitTestMethod(BasicUse)
		{
			Settings s;
			PR_EXPECT(s.count == 2);
			PR_EXPECT(s.scale == 3.14f);
			PR_EXPECT(s.mask == 0xABCU);
			PR_EXPECT(s.colour == pr::Colour32Green);
			PR_EXPECT(All(s.area == pr::v2(1, 2)));
			PR_EXPECT(All(s.position == pr::v4(1, 2, 3, 1)));
			PR_EXPECT(s.name == "hello settings");
			PR_EXPECT(s.emun == Enum1::Two);
			PR_EXPECT(!s.SaveRequired());

			s.count = 4;
			s.scale = 1.6f;
			s.mask = 0xCDEU;
			s.colour = pr::Colour32Blue;
			s.area = pr::v2::One();
			s.position = pr::v4(3, 2, 1, 1);
			s.name = "renamed";
			s.emun = Enum1::Three;
			PR_EXPECT(s.SaveRequired());
			PR_EXPECT(s.count == 4);
			PR_EXPECT(s.scale == 1.6f);
			PR_EXPECT(s.mask == 0xCDEU);
			PR_EXPECT(s.colour == pr::Colour32Blue);
			PR_EXPECT(All(s.area == pr::v2::One()));
			PR_EXPECT(All(s.position == pr::v4(3, 2, 1, 1)));
			PR_EXPECT(s.name == "renamed");
			PR_EXPECT(s.emun == Enum1::Three);

			auto root = s.Export();

			Settings s2;
			s2.Import(root);
			PR_EXPECT(s2.count == 4);
			PR_EXPECT(s2.scale == 1.6f);
			PR_EXPECT(s2.mask == 0xCDEU);
			PR_EXPECT(s2.colour == pr::Colour32Blue);
			PR_EXPECT(All(s2.area == pr::v2::One()));
			PR_EXPECT(All(s2.position == pr::v4(3, 2, 1, 1)));
			PR_EXPECT(s2.name == "renamed");
			PR_EXPECT(s2.emun == Enum1::Three);
			PR_EXPECT(s2.SaveRequired() == false);
		}
	};
}
#endif