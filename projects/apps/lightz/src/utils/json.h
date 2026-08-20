#pragma once
#include <vector>
#include <string>
#include <string_view>
#include <variant>
#include <algorithm>
#include <ostream>
#include <sstream>
#include <fstream>
#include <filesystem>
#include <type_traits>
#include "utils/utf8.h"

namespace pr::json
{
	struct Value;
	Value const& NullValue();

	// Compound types
	struct Array
	{
		std::vector<Value> items;

		Array() = default;
		Array(std::initializer_list<Value> values)
			: items(values)
		{}

		// Array interface
		bool empty() const
		{
			return items.empty();
		}
		size_t size() const
		{
			return items.size();
		}
		auto begin() const
		{
			return items.begin();
		}
		auto end() const
		{
			return items.end();
		}

		// Access the value associated with 'index'
		Value const& operator [] (size_t index) const
		{
			if (index >= items.size()) return NullValue();
			return items[index];
		}
		Value& operator[](size_t index)
		{
			items.resize(std::max<size_t>(items.size(), index + 1));
			return items[index];
		}
	};
	struct Object
	{
		using key_t = std::string;
		using keys_t = std::vector<key_t>;
		using vals_t = std::vector<Value>;
		
		keys_t keys;
		vals_t values;

		Object() = default;
		Object(std::initializer_list<std::pair<std::string_view, Value>> items);

		// Array interface
		bool empty() const
		{
			return keys.empty();
		}
		size_t size() const
		{
			return keys.size();
		}

		// Range based for iteration
		template <typename key_iter_t, typename val_iter_t> struct Iter
		{
			using pair_t = struct {
				typename key_iter_t::value_type const& key;
				typename val_iter_t::value_type const& val;
			};
			key_iter_t key;
			val_iter_t value;

			pair_t operator * () const
			{
				return pair_t{ *key, *value };
			}
			Iter& operator ++ ()
			{
				++key;
				++value;
				return *this;
			}
			friend bool operator != (Iter const& lhs, Iter const& rhs)
			{
				return lhs.key != rhs.key;
			}
		};
		auto begin() const
		{
			return Iter<keys_t::const_iterator, vals_t::const_iterator>{ keys.begin(), values.begin() };
		}
		auto begin()
		{
			return Iter<keys_t::iterator, vals_t::iterator>{ keys.begin(), values.begin() };
		}
		auto end() const
		{
			return Iter<keys_t::const_iterator, vals_t::const_iterator>{ keys.end(), values.end() };
		}
		auto end()
		{
			return Iter<keys_t::iterator, vals_t::iterator>{ keys.end(), values.end() };
		}

		// Find 'key' in the object and return its index
		size_t index_of(std::string_view key) const
		{
			auto it = std::find_if(keys.begin(), keys.end(), [key](auto& k) { return k == key; });
			return std::distance(keys.begin(), it);
		}

		// Access the value associated with 'index'
		Value const& operator [] (size_t index) const
		{
			if (index >= values.size()) return NullValue();
			return values[index];
		}
		Value& operator[](size_t index)
		{
			if (index >= values.size()) throw std::out_of_range("Index out of range");
			return values[index];
		}

		// Access the value associated with 'key'
		Value const& operator [] (std::string_view key) const
		{
			auto idx = index_of(key);
			return (*this)[idx];
		}
		Value& operator[](std::string_view key)
		{
			auto idx = index_of(key);
			if (idx == keys.size())
			{
				idx = keys.size();
				keys.push_back(std::string{ key });
				values.push_back(NullValue());
			}
			return (*this)[idx];
		}

		// Look for an item. Returns nullptr if not found
		Value const* find(std::string_view key) const
		{
			auto idx = index_of(key);
			return idx < keys.size() ? &values[idx] : nullptr;
		}
		Value* find(std::string_view key)
		{
			auto idx = index_of(key);
			return idx < keys.size() ? &values[idx] : nullptr;
		}
	};

	// Variant type
	struct Value
	{
		using variant_t = std::variant<nullptr_t, bool, std::string, double, Array, Object>;
		variant_t value;

		Value() = default;
		Value(Value&& rhs) = default;
		Value(Value const& rhs) = default;
		Value& operator = (Value&& rhs) = default;
		Value& operator = (Value const& rhs) = default;

		// Want implicit conversion to Value for all types but don't want implicit conversion to bool.
		// Only way to avoid this is to provide a constructor for each type.
		Value(nullptr_t)
			: value(nullptr)
		{}
		Value(bool value_)
			: value(value_)
		{}
		Value(std::string&& value_)
			: value(std::move(value_))
		{}
		Value(std::string const& value_)
			: value(value_)
		{}
		Value(std::string_view value_)
			: Value(std::string{ value_ })
		{}
		Value(char const* value_)
			: Value(std::string{ value_ })
		{}
		Value(double value_)
			: value(value_)
		{}
		Value(int value_)
			: Value(static_cast<double>(value_))
		{}
		Value(Array&& value_)
			: value(std::move(value_))
		{}
		Value(Object&& value_)
			: value(std::move(value_))
		{}

		// Convert to a json type
		template <typename T> T const* as() const
		{
			return std::get_if<T>(&value);
		}

		// Value accessors - only const because setting non-variant types is not supported
		template <typename T> T to() const
		{
			return std::get<std::decay_t<T>>(value);
		}
		Array const& to_array() const
		{
			return std::get<Array>(value);
		}
		Object const& to_object() const
		{
			return std::get<Object>(value);
		}
		Array& to_array()
		{
			return std::get<Array>(value);
		}
		Object& to_object()
		{
			return std::get<Object>(value);
		}

		// Object/Array accessors
		size_t size() const
		{
			if (auto object = as<Object>())
				return object->size();
			if (auto list = as<Array>())
				return list->size();

			throw std::runtime_error("Not an object or array");
		}
		Value const& operator [] (std::string_view key_) const
		{
			if (auto object = as<Object>())
				return (*object)[key_];

			throw std::runtime_error("Not an object");
		}
		Value const& operator [] (size_t index) const
		{
			if (auto object = as<Object>())
				return (*object)[index];
			if (auto list = as<Array>())
				return (*list)[index];

			throw std::runtime_error("Not an object or array");
		}

		// Comparison operators
		friend bool operator == (Value const& lhs, nullptr_t)
		{
			return lhs.value.valueless_by_exception() || std::holds_alternative<nullptr_t>(lhs.value);
		}
		friend bool operator != (Value const& lhs, nullptr_t)
		{
			return !(lhs == nullptr);
		}
	};
	template <> inline std::string_view Value::to<std::string_view>() const
	{
		return std::string_view{ to<std::string const&>() };
	}
	template <> inline int64_t Value::to<int64_t>() const
	{
		return std::llround(to<double>());
	}
	template <> inline int Value::to<int>() const
	{
		return static_cast<int>(to<int64_t>());
	}

	// JSON DOM
	struct Document : Value
	{
		Document()
			:Value(Object{})
		{}
		Object const& root() const
		{
			return std::get<Object>(value);
		}
		Object& root()
		{
			return std::get<Object>(value);
		}
	};

	// Json parsing options
	struct Options
	{
		// Deserialize options
		bool AllowComments = false;
		bool AllowTrailingCommas = false;
		
		// Serialize options
		bool Indent = false;
		std::string_view IndentString = "\t";
	};

	// Static null value returned for non-existent keys/indices
	inline Value const& NullValue()
	{
		static Value s_null_value;
		return s_null_value;
	}

	// Convert a normal string to an escaped string
	inline std::string EscapeString(std::string_view str)
	{
		auto ptr = str.data();
		auto end = str.data() + str.size();

		std::string out;
		out.reserve(str.size() * 2);

		for (; ptr != end;)
		{
			if (*ptr == '\\' || *ptr == '"' || *ptr == '/' || *ptr == '\b' || *ptr == '\f' || *ptr == '\n' || *ptr == '\r' || *ptr == '\t')
			{
				out.push_back('\\');
				out.push_back(*ptr++);
			}
			else if (str::utf8::ByteLength(*ptr) > 1)
			{
				auto code = str::utf8::CodePoint(ptr, end);
				str::utf8::Escape(code, out);
			}
			else
			{
				out.push_back(*ptr++);
			}
		}
		return out;
	}

	// Convert an escaped string to a normal string
	inline std::string UnescapeString(std::string_view str)
	{
		auto ptr = str.data();
		auto end = str.data() + str.size();

		std::string result(str.size(), '\0');
		auto out = result.data();

		for (; ptr != end; ++ptr)
		{
			if (*ptr != '\\')
			{
				*out++ = *ptr;
				continue;
			}
			switch (*++ptr)
			{
				case '\\': *out++ = '\\'; break;
				case '"': *out++ = '"'; break;
				case '/': *out++ = '/'; break;
				case 'b': *out++ = '\b'; break;
				case 'f': *out++ = '\f'; break;
				case 'n': *out++ = '\n'; break;
				case 'r': *out++ = '\r'; break;
				case 't': *out++ = '\t'; break;
				case 'u':
				{
					if (end - ptr < 5)
						throw std::runtime_error("Incomplete unicode escape sequence");

					auto code =
						((ptr[1] - '0') << 12) +
						((ptr[2] - '0') << 8) +
						((ptr[3] - '0') << 4) +
						((ptr[4] - '0') << 0);

					if (code < 0x80)
					{
						*out++ = static_cast<char>(code);
					}
					else if (code < 0x800)
					{
						*out++ = static_cast<char>(0xC0 | (code >> 6));
						*out++ = static_cast<char>(0x80 | (code & 0x3F));
					}
					else
					{
						*out++ = static_cast<char>(0xE0 | (code >> 12));
						*out++ = static_cast<char>(0x80 | ((code >> 6) & 0x3F));
						*out++ = static_cast<char>(0x80 | (code & 0x3F));
					}
					ptr += 4;
					break;
				}
				default:
				{
					throw std::runtime_error("Unknown escape sequence");
				}
			}
		}

		result.resize(out - result.data());
		return result;
	}

	// Initializer list constructor for objects
	inline Object::Object(std::initializer_list<std::pair<std::string_view, Value>> items)
		: keys()
		, values()
	{
		for (auto& [key, value] : items)
		{
			keys.push_back(std::string{ key });
			values.push_back(value);
		}
	}

	// Implementation
	namespace impl
	{
		enum class EToken
		{
			EndOfString,
			Null,
			False,
			True,
			String,
			Number,
			OpenBrace,
			CloseBrace,
			OpenBracket,
			CloseBracket,
			Colon,
			Comma,
		};
		struct Token
		{
			EToken token;
			std::string_view data;
		};

		// Format a string with arguments
		template <typename... Args>
		inline std::string Fmt(char const* fmt, Args... args)
		{
			std::string result;
			result.resize(std::snprintf(nullptr, 0, fmt, args...));
			std::snprintf(result.data(), result.size() + 1, fmt, args...);
			return result;
		}

		// Advance the string pointer to the next non-whitespace character
		inline bool EatWS(std::string_view& src, bool allow_eos)
		{
			for (; !src.empty() && std::isspace(src[0]); )
				src.remove_prefix(1);
			if (src.empty() && !allow_eos)
				throw std::runtime_error("Unexpected end of string");

			// Always return true to allow if chaining
			return true;
		}

		// Return the next token in the json string
		inline Token NextToken(std::string_view& src, Options const& opts)
		{
			EatWS(src, true);
			if (src.empty())
				return { EToken::EndOfString, {} };

			switch (src[0])
			{
				case '{':
				{
					src.remove_prefix(1);
					return { EToken::OpenBrace, {} };
				}
				case '}':
				{
					src.remove_prefix(1);
					return { EToken::CloseBrace, {} };
				}
				case '[':
				{
					src.remove_prefix(1);
					return { EToken::OpenBracket, {} };
				}
				case ']':
				{
					src.remove_prefix(1);
					return { EToken::CloseBracket, {} };
				}
				case ':':
				{
					src.remove_prefix(1);
					return { EToken::Colon, {} };
				}
				case ',':
				{
					src.remove_prefix(1);
					return { EToken::Comma, {} };
				}
				case 't':
				case 'T':
				{
					if (src.size() < 4 || tolower(src[1]) != 'r' || tolower(src[2]) != 'u' || tolower(src[3]) != 'e')
						throw std::runtime_error("Unknown token");

					src.remove_prefix(4);
					return { EToken::True, {} };
				}
				case 'f':
				case 'F':
				{
					if (src.size() < 5 || tolower(src[1]) != 'a' || tolower(src[2]) != 'l' || tolower(src[3]) != 's' || tolower(src[4]) != 'e')
						throw std::runtime_error("Unknown token");

					src.remove_prefix(5);
					return { EToken::False, {} };
				}
				case 'n':
				{
					if (src.size() < 4 || tolower(src[1]) != 'u' || tolower(src[2]) != 'l' || tolower(src[3]) != 'l')
						throw std::runtime_error("Unknown token");

					src.remove_prefix(4);
					return { EToken::Null, {} };
				}
				case '"':
				case '\'':
				{
					auto quote = src[0];

					auto ptr = src.data();
					auto end = src.data() + src.size();

					// Find the end of the string literal
					for (bool esc = true; ptr != end && (esc || *ptr != quote); esc = !esc && *ptr == '\\', ++ptr) {}
					if (ptr == end)
						throw std::runtime_error("Incomplete literal string or character");

					// Unescape the string
					auto str = src.substr(1, ptr - src.data() - 1);
					src.remove_prefix(ptr - src.data() + 1);
					return { EToken::String, str };
				}
				case '-':
				case '+':
				{
					if (src.size() < 2 || !std::isdigit(src[1]))
						throw std::runtime_error("Unknown token");

					[[fallthrough]];
				}
				case '0': case '1': case '2': case '3': case '4':
				case '5': case '6': case '7': case '8': case '9':
				{
					auto ptr = src.data();
					auto end = src.data() + src.size();

					// Find the end of the number
					auto allow_sign = true;
					auto allow_decimal_point = true;
					auto allow_exponent = true;
					for (; ptr != end; ++ptr)
					{
						if (std::isdigit(*ptr)) continue;
						if (allow_sign && (*ptr == '-' || *ptr == '+')) { allow_sign = false; continue; }
						if (allow_decimal_point && *ptr == '.') { allow_decimal_point = false; continue; }
						if (allow_exponent && (*ptr == 'e' || *ptr == 'E')) { allow_sign = true; allow_decimal_point = false; allow_exponent = false; continue; }
						break;
					}
					if (ptr == end)
						throw std::runtime_error("Incomplete number");

					auto str = src.substr(0, ptr - src.data());
					src.remove_prefix(ptr - src.data());
					return { EToken::Number, str };
				}
				case '/':
				{
					if (src.size() < 2 || src[1] != '/')
						throw std::runtime_error("Unknown token");

					if (!opts.AllowComments)
						throw std::runtime_error("Comments not allowed");

					auto ptr = src.data();
					auto end = src.data() + src.size();

					// Eat up to the end of the line
					for (; ptr != end && *ptr != '\n'; ++ptr) {}
					if (ptr == end)
						return { EToken::EndOfString, {} };

					// Tail recursion optimisation should prevent stack overflow
					src.remove_prefix(ptr - src.data());
					return NextToken(src, opts);
				}
				default:
				{
					throw std::runtime_error("Unknown token");
				}
			}
		}

		// Return the next key in the json string
		inline std::string_view NextKey(std::string_view& src, Options const& opts)
		{
			auto tok = NextToken(src, opts);
			if (tok.token != EToken::String)
				throw std::runtime_error("Expected key");

			return tok.data;
		}

		// Return the next value in the json string
		inline Value NextValue(std::string_view& src, Options const& opts)
		{
			auto tok = NextToken(src, opts);
			switch (tok.token)
			{
				case EToken::EndOfString:
				{
					throw std::runtime_error("Unexpected end of string");
				}
				case EToken::Null:
				{
					return nullptr;
				}
				case EToken::True:
				{
					return true;
				}
				case EToken::False:
				{
					return false;
				}
				case EToken::String:
				{
					return UnescapeString(tok.data);
				}
				case EToken::Number:
				{
					size_t end = 0;
					auto d = std::stod(std::string{ tok.data }, &end);
					if (end == tok.data.size())
						return d;

					throw std::runtime_error("Invalid number");
				}
				case EToken::OpenBracket:
				{
					auto list = Array{};
					auto require_comma = false;
					for (;;)
					{
						if (EatWS(src, false) && src[0] == ']')
						{
							std::ignore = NextToken(src, opts);
							break;
						}
						if (require_comma && NextToken(src, opts).token != EToken::Comma)
						{
							throw std::runtime_error("Expected comma");
						}
						if (opts.AllowTrailingCommas && EatWS(src, false) && src[0] == ']')
						{
							std::ignore = NextToken(src, opts);
							break;
						}

						// Not the end of the array, so add the next value
						list.items.push_back(NextValue(src, opts));
						require_comma = true;
					}
					return list;
				}
				case EToken::OpenBrace:
				{
					auto obj = Object{};
					auto require_comma = false;
					for (;;)
					{
						if (EatWS(src, false) && src[0] == '}')
						{
							std::ignore = NextToken(src, opts);
							break;
						}
						if (require_comma && NextToken(src, opts).token != EToken::Comma)
						{
							throw std::runtime_error("Expected comma");
						}
						if (opts.AllowTrailingCommas && EatWS(src, false) && src[0] == '}')
						{
							std::ignore = NextToken(src, opts);
							break;
						}

						// Not the end of the object, so add the next key/value pair
						auto key = NextKey(src, opts);

						if (NextToken(src, opts).token != EToken::Colon)
							throw std::runtime_error("Expected colon");

						auto val = NextValue(src, opts);

						// Add the item
						obj.keys.push_back(std::string{ key });
						obj.values.push_back(val);
						require_comma = true;
					}
					return obj;
				}
				default:
				{
					throw std::runtime_error("Unknown token");
				}
			}
		}
	}

	/// <summary>Parse a UTF-8 JSON string into a DOM tree.</summary>
	inline Value Parse(std::string_view src, Options const& opts)
	{
		using namespace impl;

		auto start = src;
		try
		{
			return NextValue(src, opts);
		}
		catch (std::runtime_error& ex)
		{
			ex = std::runtime_error(impl::Fmt("Parsing failed at offset %zu - %s", src.data() - start.data(), ex.what()));
			throw;
		}
	}

	/// <summary>Read a JSON file into a DOM tree.</summary>
	inline Value Read(std::filesystem::path const& path, Options const& opts)
	{
		std::ifstream file(path);
		if (!file.is_open())
			throw std::runtime_error(impl::Fmt("Failed to open file '%s'", path.string().c_str()));

		auto size = std::filesystem::file_size(path);
		if (size > std::numeric_limits<size_t>::max())
			throw std::runtime_error(impl::Fmt("File '%s' is too large", path.string().c_str()));

		std::string data(static_cast<size_t>(size), '\0');
		file.read(data.data(), data.size());
		return Parse(data, opts);
	}

	/// <summary>Write a JSON DOM to a string</summary>
	inline std::ostream& Serialize(std::ostream& out, Value const& value, Options const& opts)
	{
		struct L {
			static void DoSerialize(std::ostream& out, Value const& value, Options const& opts, int indent)
			{
				switch (value.value.index())
				{
					case 0:
					{
						out << "null";
						break;
					}
					case 1:
					{
						out << (std::get<bool>(value.value) ? "true" : "false");
						break;
					}
					case 2:
					{
						out << '"' << EscapeString(std::get<std::string>(value.value)) << '"';
						break;
					}
					case 3:
					{
						out << std::get<double>(value.value);
						break;
					}
					case 4:
					{
						auto& arr = std::get<Array>(value.value);
						char const* comma = "";

						out << '[';
						for (auto& item : arr.items)
						{
							out << comma;
							Indent(out, indent + 1, opts);
							DoSerialize(out, item, opts, indent + 1);
							comma = ",";
						}
						Indent(out, indent, opts);
						out << ']';
						break;
					}
					case 5:
					{
						auto const& obj = std::get<Object>(value.value);
						char const* comma = "";
						char const* space = opts.Indent ? " " : "";

						out << '{';
						for (auto const& [key, val] : obj)
						{
							out << comma;
							Indent(out, indent + 1, opts);
							out << '"' << EscapeString(key) << '"' << ":" << space;
							DoSerialize(out, val, opts, indent + 1);
							comma = ",";
						}
						Indent(out, indent, opts);
						out << '}';
						break;
					}
				}
			}
			static void Indent(std::ostream& out, int indent, Options const& opts)
			{
				if (!opts.Indent || opts.IndentString.empty())
					return;

				out << '\n';
				for (int i = 0; i < indent; ++i)
					out << opts.IndentString;
			}
		};
		L::DoSerialize(out, value, opts, 0);
		return out;
	}
	inline std::string Serialize(Value const& value, Options const& opts)
	{
		std::ostringstream out;
		Serialize(out, value, opts);
		return out.str();
	}
	inline void Serialize(std::filesystem::path filepath, Value const& value, Options const& opts)
	{
		std::ofstream file(filepath);
		if (!file.is_open())
			throw std::runtime_error(impl::Fmt("Failed to open file '%s'", filepath.string().c_str()));

		Serialize(file, value, opts);
	}
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
namespace pr::storage
{
	PRUnitTest(JsonTests, Quick)
	{
		{
			char const test_data[] =
				"{\n"
				"	\"key1\": \"value1\",\n"
				"	\"key2\": 123,\n"
				"	\"key3\": true,\n"
				"	\"key4\": false,\n"
				"	\"key5\": null,\n"
				"	\"key6\": {\n"
				"		\"key7\": \"value7\",\n"
				"		\"key8\": 456,\n"
				"		\"key9\": true,\n"
				"		\"key10\": false,\n"
				"		\"key11\": null,\n"
				"		\"key12\": {\n"
				"			\"key13\": \"value13\",\n"
				"			\"key14\": 789,\n"
				"			\"key15\": true,\n"
				"			\"key16\": false,\n"
				"			\"key17\": null\n"
				"		},\n"
				"		// Comments allowed\n"
				"		\"key18\": [\n"
				"			\"value19\",\n"
				"			123,\n"
				"			true,\n"
				"			false,\n"
				"			null\n"
				"		]\n"
				"	},\n"
				"	\"key20\": [\n"
				"		\"value21\",\n"
				"		123,\n"
				"		true,\n"
				"		false,\n"
				"		null,\n"
				"	]\n"
				"}\n";

			auto root = json::Parse(test_data, { .AllowComments = true, .AllowTrailingCommas = true });
			PR_EXPECT(root["key1"].to<std::string_view>() == "value1");
			PR_EXPECT(root["key2"].to<double>() == 123);
			PR_EXPECT(root["key3"].to<bool>() == true);
			PR_EXPECT(root["key4"].to<bool>() == false);
			PR_EXPECT(root["key5"] == nullptr);
			PR_EXPECT(root["key6"] != nullptr);
			PR_EXPECT(root["key6"]["key12"]["key14"].to<int>() == 789);
			PR_EXPECT(root["key6"]["key12"].to_object().size() == 5);
			PR_EXPECT(root["key6"]["key18"].to_array().empty() == false);
		}
		{
			char const test_data[] =
				"{\n"
				"	\"EscapedString\": \"This is a string with a \\\"quote\\\" in it\",\n"
				"	\"SearchPaths\": [\n"
				"		\"C:\\\\Work\\\\Path\",\n"
				"	],\n"
				"}\n";

			auto root = json::Parse(test_data, { .AllowComments = true, .AllowTrailingCommas = true });
			PR_EXPECT(root["SearchPaths"][0].to<std::string>() == "C:\\Work\\Path");
			PR_EXPECT(root["EscapedString"].to<std::string>() == "This is a string with a \"quote\" in it");
		}
		{
			json::Document doc;
			auto& root = doc.root();

			root["key1"] = nullptr;
			root["key2"] = true;
			root["key3"] = false;
			root["key4"] = "value1";
			root["key5"] = 123;
			root["key6"] = 456.78;
			root["key7"] = json::Array{ 1, 2, 3, 4 };
			root["key8"] = json::Object{};

			auto& child = root["key8"].to_object();
			child["key10"] = nullptr;
			child["key11"] = true;
			child["key12"] = false;
			child["key13"] = "value2";
			child["key14"] = 456;
			child["key15"] = 123.45;
			child["key16"] = json::Array{
				true,
				"value3",
				80085,
			};
			child["key17"] = json::Object{
				{"one", "1"},
				{"two", 2},
				{"three", true},
			};
			
			char const* expected = 
				"{\n"
				"	\"key1\": null,\n"
				"	\"key2\": true,\n"
				"	\"key3\": false,\n"
				"	\"key4\": \"value1\",\n"
				"	\"key5\": 123,\n"
				"	\"key6\": 456.78,\n"
				"	\"key7\": [\n"
				"		1,\n"
				"		2,\n"
				"		3,\n"
				"		4\n"
				"	],\n"
				"	\"key8\": {\n"
				"		\"key10\": null,\n"
				"		\"key11\": true,\n"
				"		\"key12\": false,\n"
				"		\"key13\": \"value2\",\n"
				"		\"key14\": 456,\n"
				"		\"key15\": 123.45,\n"
				"		\"key16\": [\n"
				"			true,\n"
				"			\"value3\",\n"
				"			80085\n"
				"		],\n"
				"		\"key17\": {\n"
				"			\"one\": \"1\",\n"
				"			\"two\": 2,\n"
				"			\"three\": true\n"
				"		}\n"
				"	}\n"
				"}";

			PR_EXPECT(root["key1"] == nullptr);
			PR_EXPECT(root["key2"].to<bool>() == true);
			PR_EXPECT(root["key3"].to<bool>() == false);
			PR_EXPECT(root["key4"].to<std::string_view>() == "value1");
			PR_EXPECT(root["key5"].to<int>() == 123);
			PR_EXPECT(root["key6"].to<double>() == 456.78);
			PR_EXPECT(root["key7"] != nullptr);
			
			auto& arr = root["key7"].to_array();
			PR_EXPECT(arr.size() == 4);
			PR_EXPECT(arr[0].to<int>() == 1);
			PR_EXPECT(arr[1].to<int>() == 2);
			PR_EXPECT(arr[2].to<int>() == 3);
			PR_EXPECT(arr[3].to<int>() == 4);

			auto& obj = root["key8"].to_object();
			PR_EXPECT(obj.size() == 8);
			PR_EXPECT(obj["key10"] == nullptr);
			PR_EXPECT(obj["key11"].to<bool>() == true);
			PR_EXPECT(obj["key12"].to<bool>() == false);
			PR_EXPECT(obj["key13"].to<std::string_view>() == "value2");
			PR_EXPECT(obj["key14"].to<int>() == 456);
			PR_EXPECT(obj["key15"].to<double>() == 123.45);

			PR_EXPECT(obj["key16"] != nullptr);
			PR_EXPECT(obj["key16"].size() == 3);
			PR_EXPECT(obj["key16"][0].to<bool>() == true);
			PR_EXPECT(obj["key16"][1].to<std::string_view>() == "value3");
			PR_EXPECT(obj["key16"][2].to<int>() == 80085);

			PR_EXPECT(obj["key17"] != nullptr);
			PR_EXPECT(obj["key17"].size() == 3);
			PR_EXPECT(obj["key17"]["one"].to<std::string_view>() == "1");
			PR_EXPECT(obj["key17"]["two"].to<int>() == 2);
			PR_EXPECT(obj["key17"]["three"].to<bool>() == true);

			std::string str = json::Serialize(doc, { .Indent = true });
			PR_EXPECT(str == expected);
		}
	}
}
#endif
