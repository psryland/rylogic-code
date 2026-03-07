//***********************************************************************
// Tweakable
//  Copyright (c) Rylogic Ltd 2008
//***********************************************************************
#pragma once
#include <string>
#include <fstream>
#include <unordered_map>
#include <type_traits>
#include <filesystem>
#include <memory>
#include <chrono>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include "pr/common/to.h"
#include "pr/common/hresult.h"
#include "pr/math/math.h"

namespace pr::tweakables
{
	namespace impl
	{
		// Helper to allow string literals to be used as template arguments
		template <size_t N> struct StringLiteral
		{
			char value[N];
			constexpr StringLiteral(const char (&str)[N]) { std::copy_n(str, N, value); }
		};
	}

	// Tweakables singleton
	struct Tweakables
	{
		static Tweakables& Instance()
		{
			static Tweakables s_instance;
			return s_instance;
		}

		// Master switch
		inline static constexpr bool Enable = true;

		// A file to contain the tweakables
		inline static std::filesystem::path filepath = "tweakables.ini";

		// The rate at which the tweakables file is polled for changes
		inline static std::chrono::system_clock::duration poll_rate = std::chrono::milliseconds(1000);

		// Return the value of a tweakable. The first call to this function must have 'def != nullptr'
		template <typename T, impl::StringLiteral S>
		static T Value(T const* def = nullptr)
		{
			static T s_value = *def;
			static int s_issue = 0;

			if constexpr (Enable)
			{
				// Ensure the tweakable exists in the file
				if (s_issue == 0)
					Instance().Add(S.value, To<std::string>(s_value));

				// If the tweakables have changed, update the value
				auto issue = Instance().m_issue.load();
				if (s_issue != issue)
				{
					std::lock_guard<std::mutex> lock(Instance().m_mutex);
					map_ptr_t map = Instance().m_variables;

					// Read the value
					auto it = map->find(S.value);
					if (it != std::end(*map))
						s_value = To<T>(it->second.c_str());

					s_issue = issue;
				}
			}
			return s_value;
		}

		// Once only changed detection
		template <typename T, impl::StringLiteral S>
		static bool Changed()
		{
			auto changed = false;
			if constexpr (Enable)
			{
				static T s_previous = Value<T,S>(nullptr);
				auto value = Value<T, S>(nullptr);
				changed = value != s_previous;
				s_previous = value;
			}
			return changed;
		}

		// Block this thread until a change is detected
		static void WaitForChange()
		{
			auto& this_ = Instance();

			std::unique_lock<std::mutex> lock(this_.m_mutex);
			this_.m_cv_changed.wait(lock);
		}

	private:

		using path_t = std::filesystem::path;
		using ftime_t = std::filesystem::file_time_type;
		using timepoint_t = std::chrono::system_clock::time_point;
		using map_t = std::unordered_map<std::string, std::string>;
		using map_ptr_t = std::shared_ptr<map_t>;
		using cv_t = std::condition_variable;
		
		std::thread m_thread;
		std::atomic_bool m_shutdown;
		std::atomic_int m_issue;
		std::mutex m_mutex;
		map_ptr_t m_variables;
		ftime_t m_last_write_time;
		cv_t m_cv_changed;

		Tweakables()
			:m_thread()
			,m_shutdown(false)
			,m_issue(1)
			,m_mutex()
			,m_variables()
			,m_last_write_time()
			,m_cv_changed()
		{
			if constexpr (Enable)
			{
				m_variables = LoadVariables();
				m_last_write_time = LastWrite();
				m_thread = std::thread([this]
				{
					for (; !m_shutdown; std::this_thread::sleep_for(poll_rate))
					{
						auto last_write = LastWrite();
						if (m_last_write_time == last_write)
							continue;

						auto variables = LoadVariables();
						Variables(variables, last_write);
					}
				});
			}
		}
		~Tweakables()
		{
			if constexpr (Enable)
			{
				m_shutdown = true;
				m_thread.join();
			}
		}
		
		// Return a pointer to the latest version of the variables
		void Variables(map_ptr_t variables, ftime_t last_write)
		{
			std::lock_guard<std::mutex> lock(m_mutex);

			m_variables = variables;
			m_last_write_time = last_write;
			++m_issue;

			m_cv_changed.notify_all();
		}

		// Ensure a tweakable value exists in the file
		void Add(std::string const& key, std::string const& value)
		{
			std::lock_guard<std::mutex> lock(m_mutex);

			map_ptr_t variables = m_variables;
			if (!variables->contains(key))
			{
				(*variables)[key] = value;
				SaveVariables(*variables);
			}
		}

		// Return the time the tweakables file was last written to
		ftime_t LastWrite()
		{
			return std::filesystem::exists(filepath)
				? std::filesystem::last_write_time(filepath)
				: ftime_t::min();
		}

		// Function to trim leading and trailing whitespace from a string
		std::string Trim(std::string const& str) const
		{
			const std::string whitespace = " \t";

			const size_t start = str.find_first_not_of(whitespace);
			if (start == std::string::npos)
				return ""; // no content

			const size_t end = str.find_last_not_of(whitespace);
			return str.substr(start, end - start + 1);
		};

		// Read the tweakables from file
		map_ptr_t LoadVariables() const
		{
			map_ptr_t variables(new map_t);

			std::ifstream ifile(filepath);
			for (std::string line; std::getline(ifile, line); )
			{
				// Ignore comment lines
				if (line.starts_with(";"))
					continue;

				// Find the position of the '=' character
				auto pos = line.find('=');
				if (pos == std::string::npos)
					continue; // skip lines without '='

				// Extract the key and value, and trim any whitespace
				auto key = Trim(line.substr(0, pos));
				auto value = Trim(line.substr(pos + 1));

				// Store the key-value pair in the map
				(*variables)[key] = value;
			}

			return variables;
		}

		// Save the tweakables to file
		void SaveVariables(map_t const& variables)
		{
			path_t tmp_filepath = path_t(filepath).concat(".tmp");
			{
				std::ofstream ofile(tmp_filepath);
				std::vector<std::pair<std::string, std::string>> sorted(variables.begin(), variables.end());
				std::sort(sorted.begin(), sorted.end());
				for (auto const& [key, value] : sorted)
					ofile << key << " = " << value << std::endl;
			}
			auto p0 = tmp_filepath.wstring();
			auto p1 = filepath.wstring();
			Check(MoveFileExW(p0.c_str(), p1.c_str(), MOVEFILE_REPLACE_EXISTING), HrMsg(GetLastError()));
		}

		#if PR_UNITTESTS
		friend struct TestClass_TweakablesTests;
		#endif
	};


	// A tweakable value.
	//  Use: Tweakable<float, "MY_VALUE"> my_value = 1.0f;
	template <typename T, impl::StringLiteral S>
	struct Tweakable
	{
		template <typename U> requires std::is_convertible_v<U, T>
		Tweakable(U value)
		{
			m_value = value;
			if constexpr (Tweakables::Enable)
				m_value = Tweakables::Value<T, S>(&m_value);
		}

		operator T() const
		{
			return get_latest();
		}

		template <typename U> requires std::is_convertible_v<U, T>
		friend bool operator == (Tweakable<T, S> const& lhs, U const& rhs)
		{
			return lhs.get_latest() == T(rhs);
		}

		template <typename U> requires std::is_convertible_v<U, T>
		friend bool operator != (Tweakable<T, S> const& lhs, U const& rhs)
		{
			return lhs.get_latest() != T(rhs);
		}

	private:
		T const& get_latest() const
		{
			if constexpr (Tweakables::Enable)
				m_value = Tweakables::Value<T, S>(nullptr);

			return m_value;
		}

		mutable T m_value; // To ensure the size and alignment of this type is the same as T (and for debugging)
	};
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
namespace pr::tweakables
{
	PRUnitTest(TweakablesTests)
	{
		#if 1
		using namespace pr::tweakables;
		Tweakables::filepath = temp_dir() / "tweakables.ini";
		Tweakables::poll_rate = std::chrono::milliseconds(100);

		Tweakable<bool, "MY_BOOL"> my_bool = true;
		Tweakable<int, "MY_INT"> my_int = 2;
		Tweakable<float, "MY_FLOAT"> my_float = 1.0f;
		Tweakable<std::string, "MY_STRING"> my_string = "hello";

		PR_EXPECT(my_bool == true);
		PR_EXPECT(my_int == 2);
		PR_EXPECT(my_float == 1.0f);
		PR_EXPECT(my_string == "hello");

		if constexpr (Tweakables::Enable)
		{
			auto issue = Tweakables::Instance().m_issue.load();

			Tweakables::map_t variables = {};
			variables["MY_BOOL"] = "false";
			variables["MY_INT"] = "3";
			variables["MY_FLOAT"] = "-1.0";
			variables["MY_STRING"] = "world";
			Tweakables::Instance().SaveVariables(variables);

			for (; issue == Tweakables::Instance().m_issue; )
				std::this_thread::sleep_for(Tweakables::poll_rate);

			PR_EXPECT(my_bool == false);
			PR_EXPECT(my_int == 3);
			PR_EXPECT(my_float == -1.0f);
			PR_EXPECT(my_string == "world");
		}
		#endif
	}
}
#endif
