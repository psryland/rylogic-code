// INI File Expected format:
// Lines that do not start with one of these formats are ignored
//   ; ....   -> comment line
//   [Section] -> Section start definition
//   <identifier> = <value> ... -> key/value pair
#pragma once
#include <string>
#include <string_view>
#include <iostream>

namespace pr::ini_file
{
	enum class EElement
	{
		EoF,
		Section,
		KeyValue,
	};

	class Iterator
	{
		std::istream& m_text;
		std::string m_line;
		std::string_view m_key;
		std::string_view m_val;
		EElement m_type;
		int m_line_no;

	public:

		Iterator(std::istream& text)
			: m_text(text)
			, m_line()
			, m_key()
			, m_val()
			, m_type()
			, m_line_no(1)
		{
			Next();
		}
		Iterator(Iterator&&) = default;
		Iterator(Iterator const&) = delete;
		Iterator& operator =(Iterator&&) = default;
		Iterator& operator =(Iterator const&) = delete;

		// Get the current line number
		int LineNo() const
		{
			return m_line_no;
		}

		// Get the value for the current key/value pair
		std::string_view Value() const
		{
			return m_val;
		}

		// Match the key name against the given string
		bool IsMatch(EElement type) const
		{
			return m_type == type;
		}
		bool IsMatch(EElement type, std::string_view name) const
		{
			return m_type == type && m_key == name;
		}

		// True when the ini stream is exhausted
		bool AtEnd() const
		{
			return m_type == EElement::EoF;
		}

		// Read the next line from the ini stream
		void Next()
		{
			// Remove leading and trailing whitespace
			constexpr auto TrimWS = [](std::string_view str) -> std::string_view
			{
				for (; !str.empty() && std::isspace(str.front()); str.remove_prefix(1)) {}
				for (; !str.empty() && std::isspace(str.back()); str.remove_suffix(1)) {}
				return str;
			};
			
			// Read the next line from the ini stream
			for (; std::getline(m_text, m_line); ++m_line_no)
			{
				// Blank line
				if (m_line.empty())
					continue;

				// Comment
				if (m_line[0] == ';')
					continue;

				// Section
				if (m_line[0] == '[')
				{
					auto end = m_line.find(']');
					if (end == -1)
						continue;

					m_key = TrimWS({ m_line.data() + 1, end - 1 });
					m_val = { m_line.data(), 0 };
					m_type = EElement::Section;
					return;
				}

				// Key/Value pair
				else
				{
					auto eq = m_line.find('=');
					if (eq == -1)
						continue;

					m_key = TrimWS({ m_line.data(), eq });
					m_val = TrimWS({ m_line.data() + (eq + 1), m_line.size() - (eq + 1) });
					m_type = EElement::KeyValue;
					return;
				}
			}

			m_key = {};
			m_val = {};
			m_type = EElement::EoF;
		}
		Iterator& operator ++()
		{
			Next();
			return *this;
		}
	};
}

#if PR_UNITTESTS
#include <sstream>
#include "pr/common/unittests.h"
namespace pr::storage
{
	PRUnitTest(IniFileTests, Quick)
	{
		char const test_data[] =
			"[Numbers]\n"
			"One=1\n"
			"Two=2\n"
			"\n"
			"[Strings]\n"
			" Hello	= World   \n"
			"   Goodbye  =  World\n";

		std::stringstream ss(test_data);
		ini_file::Iterator iter(ss);

		PR_EXPECT(iter.IsMatch(ini_file::EElement::Section, "Numbers"));
		++iter;

		PR_EXPECT(iter.IsMatch(ini_file::EElement::KeyValue, "One"));
		PR_EXPECT(iter.Value() == "1");
		++iter;

		PR_EXPECT(iter.IsMatch(ini_file::EElement::KeyValue, "Two"));
		PR_EXPECT(iter.Value() == "2");
		++iter;

		PR_EXPECT(iter.IsMatch(ini_file::EElement::Section, "Strings"));
		++iter;

		PR_EXPECT(iter.IsMatch(ini_file::EElement::KeyValue, "Hello"));
		PR_EXPECT(iter.Value() == "World");
		++iter;

		PR_EXPECT(iter.IsMatch(ini_file::EElement::KeyValue, "Goodbye"));
		PR_EXPECT(iter.Value() == "World");
		++iter;
	}
}
#endif
